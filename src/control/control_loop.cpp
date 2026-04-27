#include "control/control_loop.hpp"

#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/protocol.hpp"
#include "crazyflie/protocol/types.hpp"
#include "logging/console.hpp"
#include "logging/types.hpp"
#include "mission/mission.hpp"
#include "mission/types.hpp"
#include "safety/safety.hpp"

#include <chrono>
#include <thread>

namespace cfo {

namespace {

const char* abort_str(AbortReason r) noexcept {
    switch (r) {
        case AbortReason::None:            return "ok";
        case AbortReason::TelemetryStale:  return "telemetry_stale";
        case AbortReason::LowBattery:      return "low_battery";
        case AbortReason::LinkLost:        return "link_lost";
        case AbortReason::OperatorAbort:   return "operator_abort";
        case AbortReason::MissionTimeout:  return "mission_timeout";
    }
    return "unknown";
}

// Returns std::nullopt for NoOp (no packet to send this tick).
std::optional<RawPacket> to_packet(const MissionCommand& c) noexcept {
    using K = MissionCommand::Kind;
    switch (c.kind) {
        case K::Stop:    return make_setpoint_stop();
        case K::Hover:   return make_setpoint_hover(
                             c.vx_mps, c.vy_mps, c.yaw_rate_dps, c.z_target_m);
        case K::HlcTakeoff: return make_hlc_takeoff(
                                c.hlc_height_m, c.hlc_duration_s);
        case K::HlcGoTo:  return make_hlc_go_to(
                              c.hlc_goto_x_m, c.hlc_goto_y_m, c.hlc_goto_z_m,
                              c.hlc_goto_yaw_rad, c.hlc_duration_s,
                              c.hlc_goto_relative);
        case K::HlcLand:  return make_hlc_land(
                              c.hlc_height_m, c.hlc_duration_s);
        case K::HlcStop:  return make_hlc_stop();
        case K::NoOp:    return std::nullopt;
    }
    return std::nullopt;
}

// SetpointCommandEvent today only carries Stop/Hover. We emit it for
// those two kinds; HlcLand and NoOp don't have a parallel decoded record
// (the raw packet, if any, still goes to /command/raw).
std::optional<SetpointCommandEvent>
to_event(const MissionCommand& c,
         std::chrono::system_clock::time_point t) {
    using K = MissionCommand::Kind;
    if (c.kind != K::Stop && c.kind != K::Hover) return std::nullopt;
    SetpointCommandEvent e{};
    e.kind = (c.kind == K::Stop) ? SetpointCommandEvent::Kind::Stop
                                 : SetpointCommandEvent::Kind::Hover;
    e.vx_mps       = c.vx_mps;
    e.vy_mps       = c.vy_mps;
    e.yaw_rate_dps = c.yaw_rate_dps;
    e.z_target_m   = c.z_target_m;
    e.t = t;
    return e;
}

} // namespace

void run_control_loop(ICrazyflieLink& link,
                      const StateStore& state,
                      const AppStatusStore& app_status,
                      MCAPLogger& logger,
                      const ControlLoopConfig& cfg,
                      std::atomic<bool>& shutdown_requested,
                      MissionStateCallback on_state_change,
                      SetpointCallback     on_setpoint) {
    using clock = std::chrono::steady_clock;

    MissionContext ctx{};
    ctx.mission_started = clock::now();
    ctx.state_entered = ctx.mission_started;

    cfo::console::info("control: loop started (period={}ms)",
                       cfg.period.count());

    auto next_tick = clock::now();
    while (true) {
        const auto now = clock::now();
        const auto t_host = std::chrono::system_clock::now();

        MissionTickInput in{};
        in.vehicle = state.snapshot();
        in.safety  = check_safety(in.vehicle, now, cfg.safety);
        in.now     = now;
        in.operator_shutdown =
            shutdown_requested.load(std::memory_order_acquire);
        in.forward_obstacle = app_status.snapshot().obstacle_status;

        const auto out = mission_tick(ctx, in, cfg.mission);
        ctx = out.next;

        if (out.state_changed) {
            if (ctx.state == MissionState::Aborted) {
                cfo::console::info("mission: → {} ({}: {})",
                                   state_name(ctx.state),
                                   abort_str(ctx.abort_reason),
                                   ctx.abort_detail);
            } else {
                cfo::console::info("mission: → {}", state_name(ctx.state));
            }
            logger.log(MissionStateEvent{
                ctx.state, ctx.abort_reason, ctx.abort_detail, t_host});
            if (on_state_change) {
                on_state_change(ctx.state, ctx.abort_reason, ctx.abort_detail);
            }
        }

        // The mission is now all-HLC end-to-end (TAKEOFF → GO_TO → LAND),
        // so there is no low-level commander to notify-stop. Manual /
        // keyboard / gamepad paths that stream low-level setpoints will
        // own their own handoff to HLC if/when they're added.

        if (auto pkt = to_packet(out.command)) {
            if (auto r = link.send(*pkt); !r) {
                cfo::console::warn("control: send failed");
            }
            logger.log(RawCommandEvent{*pkt, t_host});
        }
        if (auto ev = to_event(out.command, t_host)) {
            logger.log(*ev);
            if (on_setpoint) on_setpoint(*ev);
        }

        if (out.terminate) {
            cfo::console::info("control: loop exiting ({})",
                               state_name(ctx.state));
            // Don't touch shutdown_requested — that's an *app-level* signal
            // owned by main.cpp. The control loop just returns; main decides
            // whether to wind everything down (headless) or wait for the
            // operator to quit (TUI).
            return;
        }

        next_tick += cfg.period;
        if (next_tick < now) next_tick = now + cfg.period;
        std::this_thread::sleep_until(next_tick);
    }
}

} // namespace cfo
