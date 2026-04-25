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

RawPacket to_packet(const MissionCommand& c) noexcept {
    if (c.kind == MissionCommand::Kind::Stop) return make_setpoint_stop();
    return make_setpoint_hover(c.vx_mps, c.vy_mps, c.yaw_rate_dps,
                                c.z_target_m);
}

SetpointCommandEvent to_event(const MissionCommand& c,
                              std::chrono::system_clock::time_point t) {
    SetpointCommandEvent e{};
    e.kind = (c.kind == MissionCommand::Kind::Stop)
                 ? SetpointCommandEvent::Kind::Stop
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
                      MCAPLogger& logger,
                      const ControlLoopConfig& cfg,
                      std::atomic<bool>& shutdown_requested) {
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

        const auto out = mission_tick(ctx, in, cfg.mission);
        ctx = out.next;

        if (out.state_changed) {
            cfo::console::info("mission: → {}", state_name(ctx.state));
            logger.log(MissionStateEvent{
                ctx.state, ctx.abort_reason, ctx.abort_detail, t_host});
        }

        const auto pkt = to_packet(out.command);
        if (auto r = link.send(pkt); !r) {
            cfo::console::warn("control: send failed");
        }
        logger.log(RawCommandEvent{pkt, t_host});
        logger.log(to_event(out.command, t_host));

        if (out.terminate) {
            cfo::console::info("control: loop exiting ({})",
                               state_name(ctx.state));
            shutdown_requested.store(true, std::memory_order_release);
            return;
        }

        next_tick += cfg.period;
        if (next_tick < now) next_tick = now + cfg.period;
        std::this_thread::sleep_until(next_tick);
    }
}

} // namespace cfo
