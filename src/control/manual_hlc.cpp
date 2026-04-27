#include "control/manual_hlc.hpp"

#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/protocol.hpp"
#include "crazyflie/protocol/types.hpp"
#include "logging/console.hpp"
#include "logging/types.hpp"
#include "safety/safety.hpp"

#include <chrono>
#include <cmath>
#include <thread>

namespace cfo {

StepDelta step_to_delta(ManualStep s, const ManualHlcConfig& cfg,
                        float current_yaw_rad) noexcept {
    // Body-frame deltas first.
    float bx = 0.0f, by = 0.0f, bz = 0.0f, dyaw = 0.0f;
    switch (s) {
        case ManualStep::XPlus:    bx =  cfg.step_xy_m; break;
        case ManualStep::XMinus:   bx = -cfg.step_xy_m; break;
        case ManualStep::YPlus:    by =  cfg.step_xy_m; break;
        case ManualStep::YMinus:   by = -cfg.step_xy_m; break;
        case ManualStep::ZPlus:    bz =  cfg.step_z_m;  break;
        case ManualStep::ZMinus:   bz = -cfg.step_z_m;  break;
        case ManualStep::YawPlus:  dyaw =  cfg.step_yaw_rad; break;
        case ManualStep::YawMinus: dyaw = -cfg.step_yaw_rad; break;
        case ManualStep::None:     break;
    }
    // Rotate (bx, by) into world frame by the current yaw — so "forward"
    // means the nose direction regardless of how the drone is yawed.
    const float cy = std::cos(current_yaw_rad);
    const float sy = std::sin(current_yaw_rad);
    StepDelta out{};
    out.dx       = bx * cy - by * sy;
    out.dy       = bx * sy + by * cy;
    out.dz       = bz;
    out.dyaw_rad = dyaw;
    return out;
}

namespace {

using clock = std::chrono::steady_clock;

float to_seconds(std::chrono::milliseconds d) noexcept {
    return std::chrono::duration<float>(d).count();
}

void send_pkt(ICrazyflieLink& link, MCAPLogger& logger, const RawPacket& p) {
    if (auto r = link.send(p); !r) {
        cfo::console::warn("manual: send failed");
    }
    logger.log(RawCommandEvent{p, std::chrono::system_clock::now()});
}

void set_manual_state(AppStatusStore& app_status, ManualState s) {
    app_status.update([s](AppStatus& st) { st.manual_state = s; });
}

const char* abort_str(AbortReason r) noexcept {
    switch (r) {
        case AbortReason::None:           return "ok";
        case AbortReason::TelemetryStale: return "telemetry_stale";
        case AbortReason::LowBattery:     return "low_battery";
        case AbortReason::LinkLost:       return "link_lost";
        case AbortReason::OperatorAbort:  return "operator_abort";
        case AbortReason::MissionTimeout: return "mission_timeout";
    }
    return "unknown";
}

} // namespace

void run_manual_hlc(ICrazyflieLink& link,
                    const StateStore& state,
                    MCAPLogger& logger,
                    AppStatusStore& app_status,
                    OperatorIntents& intents,
                    const ManualHlcConfig& cfg,
                    std::atomic<bool>& shutdown_requested) {
    using namespace std::chrono_literals;

    cfo::console::info("manual: entering ManualHlc mode");

    // Arm before any HLC engagement. HLC STOP at the end will disarm,
    // matching the mission lifecycle exactly.
    {
        const auto t = std::chrono::system_clock::now();
        const auto a_modern = make_arm_request(true);
        const auto a_legacy = make_arm_request_legacy(true);
        const bool ok_m = static_cast<bool>(link.send(a_modern));
        const bool ok_l = static_cast<bool>(link.send(a_legacy));
        if (!(ok_m || ok_l)) {
            cfo::console::error("manual: arm send failed — exiting");
            return;
        }
        if (ok_m) logger.log(RawCommandEvent{a_modern, t});
        if (ok_l) logger.log(RawCommandEvent{a_legacy, t});
        cfo::console::info("manual: armed");
    }

    ManualState ms = ManualState::OnGround;
    auto state_entered = clock::now();
    set_manual_state(app_status, ms);

    auto enter = [&](ManualState next) {
        ms = next;
        state_entered = clock::now();
        set_manual_state(app_status, ms);
        cfo::console::info("manual: → {}", manual_state_name(ms));
    };

    auto since_entered = [&]() {
        return clock::now() - state_entered;
    };

    while (!shutdown_requested.load(std::memory_order_acquire)) {
        const auto frame_start = clock::now();

        // exit_manual (operator pressed 'm' again). If on ground we
        // exit immediately; if flying we initiate land first.
        if (intents.exit_manual.exchange(false, std::memory_order_acq_rel)) {
            if (ms == ManualState::OnGround) {
                cfo::console::info("manual: exit requested");
                break;
            }
            // Airborne → kick off landing, then we'll return after
            // touchdown_dwell elapses.
            cfo::console::info("manual: exit requested while airborne — landing");
            send_pkt(link, logger,
                     make_hlc_land(0.0f, to_seconds(cfg.land_duration)));
            enter(ManualState::Landing);
        }

        // Safety check while airborne. Mirrors the mission abort path —
        // emergency-land via HLC, then stop.
        if (ms == ManualState::Flying || ms == ManualState::TakingOff) {
            const auto safety = check_safety(state.snapshot(),
                                             clock::now(), cfg.safety);
            const bool operator_abort = intents.abort_mission.exchange(
                false, std::memory_order_acq_rel);
            if (cfo::aborted(safety) || operator_abort) {
                const char* reason = operator_abort
                    ? "operator_abort"
                    : abort_str(safety.reason);
                cfo::console::warn("manual: abort — {} — landing fast",
                                   reason);
                logger.log(SafetyEvent{
                    operator_abort ? AbortReason::OperatorAbort : safety.reason,
                    operator_abort ? std::string{"manual abort"} : safety.detail,
                    std::chrono::system_clock::now()});
                send_pkt(link, logger,
                         make_hlc_land(0.0f, to_seconds(cfg.abort_descent_dwell)));
                // Use abort dwell as the effective land duration.
                enter(ManualState::Landing);
            }
        }

        // Per-state actions.
        switch (ms) {
            case ManualState::OnGround: {
                if (intents.manual_takeoff.exchange(false,
                                                    std::memory_order_acq_rel)) {
                    cfo::console::info("manual: takeoff to {:.2f}m",
                                       cfg.takeoff_height_m);
                    send_pkt(link, logger,
                             make_hlc_takeoff(cfg.takeoff_height_m,
                                              to_seconds(cfg.takeoff_duration)));
                    enter(ManualState::TakingOff);
                }
                break;
            }

            case ManualState::TakingOff: {
                // No movement commands during takeoff — accepted only
                // once the firmware-side trajectory has finished.
                (void)intents.manual_step.exchange(ManualStep::None,
                                                    std::memory_order_acq_rel);
                (void)intents.manual_land.exchange(false,
                                                    std::memory_order_acq_rel);
                if (since_entered() >= cfg.takeoff_duration) {
                    enter(ManualState::Flying);
                }
                break;
            }

            case ManualState::Flying: {
                if (intents.manual_land.exchange(false,
                                                  std::memory_order_acq_rel)) {
                    cfo::console::info("manual: land");
                    send_pkt(link, logger,
                             make_hlc_land(0.0f, to_seconds(cfg.land_duration)));
                    enter(ManualState::Landing);
                    break;
                }
                const auto step = intents.manual_step.exchange(
                    ManualStep::None, std::memory_order_acq_rel);
                if (step != ManualStep::None) {
                    const auto v = state.snapshot();
                    const auto d = step_to_delta(step, cfg, v.attitude.yaw);
                    cfo::console::info(
                        "manual: step Δ=({:+.2f}, {:+.2f}, {:+.2f}) "
                        "yaw{:+.1f}°",
                        d.dx, d.dy, d.dz,
                        d.dyaw_rad * 180.0f /
                            static_cast<float>(M_PI));
                    send_pkt(link, logger,
                             make_hlc_go_to(
                                 d.dx, d.dy, d.dz, d.dyaw_rad,
                                 to_seconds(cfg.step_duration),
                                 /*relative=*/true));
                }
                break;
            }

            case ManualState::Landing: {
                // HLC LAND in flight, then post-land dwell, then HLC STOP.
                if (since_entered() >=
                        cfg.land_duration + cfg.touchdown_dwell) {
                    cfo::console::info("manual: HLC stop");
                    send_pkt(link, logger, make_hlc_stop());
                    enter(ManualState::OnGround);
                    // If exit_manual was requested mid-flight, the loop
                    // top will exit at the next iteration.
                }
                break;
            }
        }

        std::this_thread::sleep_until(frame_start + cfg.tick_period);
    }

    // Final cleanup: if app is shutting down while airborne, emergency
    // land before returning so the operator doesn't lose the vehicle.
    if (ms != ManualState::OnGround) {
        cfo::console::warn("manual: leaving while airborne — forcing land");
        send_pkt(link, logger,
                 make_hlc_land(0.0f, to_seconds(cfg.abort_descent_dwell)));
        std::this_thread::sleep_for(cfg.abort_descent_dwell + cfg.touchdown_dwell);
        send_pkt(link, logger, make_hlc_stop());
    }

    cfo::console::info("manual: exited ManualHlc mode");
}

} // namespace cfo
