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
    return MissionCommand{MissionCommand::Kind::Stop, 0, 0, 0, 0};
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
            // First tick: stamp mission_started and enter TakingOff.
            out.next = enter(s, MissionState::TakingOff, in.now);
            out.next.mission_started = in.now;
            out.state_changed = true;
            // Fall through to TakingOff to emit a useful command on this
            // first tick rather than wasting a frame at z=0.
            out.command = hover(0, 0, 0, 0.0f);
            out.next.last_commanded_z = 0.0f;
            return out;
        }

        case MissionState::TakingOff: {
            const auto z = linear_ramp(0.0f, cfg.target_height_m,
                                       in.now - s.state_entered,
                                       cfg.takeoff_duration);
            out.command = hover(0, 0, 0, z);
            out.next.last_commanded_z = z;
            if (elapsed_ge(s.state_entered, in.now, cfg.takeoff_duration)) {
                out.next = enter(out.next, MissionState::HoverStabilizing, in.now);
                out.state_changed = true;
            }
            return out;
        }

        case MissionState::HoverStabilizing: {
            out.command = hover(0, 0, 0, cfg.target_height_m);
            out.next.last_commanded_z = cfg.target_height_m;
            if (elapsed_ge(s.state_entered, in.now, cfg.hover_duration)) {
                out.next = enter(out.next, MissionState::ForwardSegment, in.now);
                out.state_changed = true;
            }
            return out;
        }

        case MissionState::ForwardSegment: {
            out.command = hover(cfg.forward_velocity_mps, 0, 0,
                                cfg.target_height_m);
            out.next.last_commanded_z = cfg.target_height_m;
            if (elapsed_ge(s.state_entered, in.now, cfg.forward_duration)) {
                out.next = enter(out.next, MissionState::PreLandHover, in.now);
                out.state_changed = true;
            }
            return out;
        }

        case MissionState::PreLandHover: {
            out.command = hover(0, 0, 0, cfg.target_height_m);
            out.next.last_commanded_z = cfg.target_height_m;
            if (elapsed_ge(s.state_entered, in.now, cfg.preland_hover_duration)) {
                out.next = enter(out.next, MissionState::Landing, in.now);
                out.state_changed = true;
            }
            return out;
        }

        case MissionState::Landing: {
            const auto z = linear_ramp(cfg.target_height_m, 0.0f,
                                       in.now - s.state_entered,
                                       cfg.land_duration);
            out.command = hover(0, 0, 0, z);
            out.next.last_commanded_z = z;
            if (elapsed_ge(s.state_entered, in.now, cfg.land_duration)) {
                out.next = enter(out.next, MissionState::Completed, in.now);
                out.state_changed = true;
                out.command = stop_cmd();   // disarming stop on completion
                out.terminate = true;
            }
            return out;
        }

        case MissionState::Completed: {
            // Already emitted Stop on entry; should not normally re-tick.
            out.command = stop_cmd();
            out.terminate = true;
            return out;
        }

        case MissionState::Aborted: {
            // Ramp from where we were down to 0, then disarm.
            const auto since = in.now - s.state_entered;
            if (since >= cfg.abort_descent_dwell) {
                out.command = stop_cmd();
                out.terminate = true;
                return out;
            }
            const auto z = linear_ramp(s.last_commanded_z, 0.0f, since,
                                       cfg.abort_descent_dwell);
            out.command = hover(0, 0, 0, std::max(z, 0.0f));
            out.next.last_commanded_z = out.command.z_target_m;
            return out;
        }
    }
    // Unreachable.
    out.command = stop_cmd();
    out.terminate = true;
    return out;
}

} // namespace cfo
