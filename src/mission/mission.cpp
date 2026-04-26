#include "mission/mission.hpp"

#include <algorithm>
#include <chrono>

namespace cfo {

namespace {

using clock = std::chrono::steady_clock;

float linear_ramp(float from, float to, clock::duration elapsed,
                  clock::duration total) noexcept {
    if (total.count() <= 0) return to;
    const float u = std::clamp(
        std::chrono::duration<float>(elapsed).count() /
            std::chrono::duration<float>(total).count(),
        0.0f, 1.0f);
    return from + (to - from) * u;
}

bool elapsed_ge(clock::time_point started, clock::time_point now,
                clock::duration d) noexcept {
    return (now - started) >= d;
}

MissionCommand hover(float vx, float vy, float yaw_rate, float z) noexcept {
    return MissionCommand{MissionCommand::Kind::Hover, vx, vy, yaw_rate, z};
}

MissionCommand stop_cmd() noexcept {
    MissionCommand c{};
    c.kind = MissionCommand::Kind::Stop;
    return c;
}

MissionCommand hlc_stop_cmd() noexcept {
    MissionCommand c{};
    c.kind = MissionCommand::Kind::HlcStop;
    return c;
}

MissionCommand noop_cmd() noexcept {
    MissionCommand c{};
    c.kind = MissionCommand::Kind::NoOp;
    return c;
}

MissionCommand hlc_takeoff_cmd(float height_m, float duration_s) noexcept {
    MissionCommand c{};
    c.kind = MissionCommand::Kind::HlcTakeoff;
    c.hlc_height_m   = height_m;
    c.hlc_duration_s = duration_s;
    return c;
}

MissionCommand hlc_goto_cmd(float x, float y, float z, float yaw_rad,
                            float duration_s, bool relative = true) noexcept {
    MissionCommand c{};
    c.kind = MissionCommand::Kind::HlcGoTo;
    c.hlc_goto_x_m       = x;
    c.hlc_goto_y_m       = y;
    c.hlc_goto_z_m       = z;
    c.hlc_goto_yaw_rad   = yaw_rad;
    c.hlc_goto_relative  = relative;
    c.hlc_duration_s     = duration_s;
    return c;
}

MissionCommand hlc_land_cmd(float height_m, float duration_s) noexcept {
    MissionCommand c{};
    c.kind = MissionCommand::Kind::HlcLand;
    c.hlc_height_m   = height_m;
    c.hlc_duration_s = duration_s;
    return c;
}

float to_seconds(std::chrono::milliseconds d) noexcept {
    return std::chrono::duration<float>(d).count();
}

// Transition helper — returns a new context entered into `s` at `now`.
MissionContext enter(const MissionContext& prev, MissionState s,
                     clock::time_point now) noexcept {
    MissionContext c = prev;
    c.state = s;
    c.state_entered = now;
    return c;
}

// Build an Aborted context from any non-terminal state, capturing the
// reason so the operator can see what tripped it in MCAP.
MissionContext enter_abort(const MissionContext& prev, AbortReason r,
                           std::string detail,
                           clock::time_point now) noexcept {
    MissionContext c = prev;
    c.state = MissionState::Aborted;
    c.state_entered = now;
    c.abort_reason = r;
    c.abort_detail = std::move(detail);
    return c;
}

bool is_terminal(MissionState s) noexcept {
    return s == MissionState::Completed || s == MissionState::Aborted;
}

} // namespace

MissionTickOutput mission_tick(const MissionContext& ctx,
                                const MissionTickInput& in,
                                const MissionConfig& cfg) {
    MissionTickOutput out{};
    out.next = ctx;

    // ---- abort gates: evaluated before normal state transitions, so a
    //      mid-state abort overrides whatever the trajectory would do. ----
    if (!is_terminal(ctx.state)) {
        if (in.operator_shutdown) {
            out.next = enter_abort(ctx, AbortReason::OperatorAbort,
                                   "shutdown signal", in.now);
            out.state_changed = true;
        } else if (cfo::aborted(in.safety)) {
            out.next = enter_abort(ctx, in.safety.reason, in.safety.detail,
                                   in.now);
            out.state_changed = true;
        } else if (in.now - ctx.mission_started > cfg.total_timeout) {
            out.next = enter_abort(ctx, AbortReason::MissionTimeout,
                                   "wall-clock cap", in.now);
            out.state_changed = true;
        }
    }

    const auto& s = out.next;

    // ---- per-state command + transitions ----
    switch (s.state) {
        case MissionState::Idle: {
            // First tick: stamp mission_started, enter TakingOff, fire
            // HLC TAKEOFF. The firmware then owns the climb + position
            // hold; subsequent ticks just wait the duration.
            out.next = enter(s, MissionState::TakingOff, in.now);
            out.next.mission_started = in.now;
            out.state_changed = true;
            out.command = hlc_takeoff_cmd(cfg.target_height_m,
                                          to_seconds(cfg.takeoff_duration));
            out.next.last_commanded_z = cfg.target_height_m;
            return out;
        }

        case MissionState::TakingOff: {
            // HLC TAKEOFF is in flight; do not send anything until the
            // firmware-side trajectory completes.
            if (elapsed_ge(s.state_entered, in.now, cfg.takeoff_duration)) {
                out.next = enter(out.next, MissionState::HoverStabilizing, in.now);
                out.state_changed = true;
            }
            out.command = noop_cmd();
            return out;
        }

        case MissionState::HoverStabilizing: {
            // HLC continues holding the position from TAKEOFF. We just
            // wait the configured stabilizing window.
            if (elapsed_ge(s.state_entered, in.now, cfg.hover_duration)) {
                out.next = enter(out.next, MissionState::ForwardSegment, in.now);
                out.state_changed = true;
                // Fire HLC GO_TO on entry to ForwardSegment. Forward
                // distance = velocity × duration in the body's +x (relative).
                const float fwd = cfg.forward_velocity_mps *
                                  to_seconds(cfg.forward_duration);
                out.command = hlc_goto_cmd(
                    /*x=*/fwd, /*y=*/0.0f, /*z=*/0.0f, /*yaw=*/0.0f,
                    to_seconds(cfg.forward_duration),
                    /*relative=*/true);
            } else {
                out.command = noop_cmd();
            }
            return out;
        }

        case MissionState::ForwardSegment: {
            // HLC GO_TO is executing. Wait it out; HLC will hold at the
            // target position once the trajectory finishes.
            if (elapsed_ge(s.state_entered, in.now, cfg.forward_duration)) {
                out.next = enter(out.next, MissionState::PreLandHover, in.now);
                out.state_changed = true;
            }
            out.command = noop_cmd();
            return out;
        }

        case MissionState::PreLandHover: {
            // HLC continues holding at the forward target. Wait, then
            // hand off to LAND.
            if (elapsed_ge(s.state_entered, in.now, cfg.preland_hover_duration)) {
                out.next = enter(out.next, MissionState::HlcLanding, in.now);
                out.state_changed = true;
                out.command = hlc_land_cmd(
                    cfg.hlc_land_target_height_m,
                    to_seconds(cfg.hlc_land_duration));
            } else {
                out.command = noop_cmd();
            }
            return out;
        }

        case MissionState::HlcLanding: {
            // HLC LAND in flight. After its duration the firmware should
            // have settled the vehicle; emit HLC STOP to disarm cleanly
            // on the same authority. Sending low-level setpoint_stop here
            // is what triggered `is_locked` between runs (mismatched
            // controller authority on the disarm tick).
            if (elapsed_ge(s.state_entered, in.now, cfg.hlc_land_duration)) {
                out.next = enter(out.next, MissionState::Completed, in.now);
                out.state_changed = true;
                out.command = hlc_stop_cmd();
                out.terminate = true;
            } else {
                out.command = noop_cmd();
            }
            return out;
        }

        case MissionState::Completed: {
            // Already emitted HLC STOP on entry; should not normally re-tick.
            out.command = hlc_stop_cmd();
            out.terminate = true;
            return out;
        }

        case MissionState::Aborted: {
            // Entry tick: fire HLC LAND with the abort dwell as duration.
            // Subsequent ticks: wait it out, then HLC STOP to disarm on
            // the same authority that just landed the vehicle.
            if (out.state_changed) {
                out.command = hlc_land_cmd(0.0f,
                                           to_seconds(cfg.abort_descent_dwell));
            } else {
                const auto since = in.now - s.state_entered;
                if (since >= cfg.abort_descent_dwell) {
                    out.command = hlc_stop_cmd();
                    out.terminate = true;
                } else {
                    out.command = noop_cmd();
                }
            }
            return out;
        }
    }
    // Unreachable.
    out.command = hlc_stop_cmd();
    out.terminate = true;
    return out;
}

} // namespace cfo
