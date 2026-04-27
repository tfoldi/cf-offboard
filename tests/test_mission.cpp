#include "mission/mission.hpp"
#include "mission/types.hpp"
#include "safety/types.hpp"
#include "state/types.hpp"

#include <doctest/doctest.h>

#include <chrono>

namespace {

using clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

cfo::MissionTickInput healthy_input(clock::time_point now) {
    cfo::MissionTickInput in{};
    // Fresh telemetry, healthy battery.
    in.vehicle.battery_voltage = 3.8f;
    in.vehicle.update_count = 1;
    in.vehicle.last_update = now;
    in.now = now;
    return in;
}

cfo::MissionTickInput aborting_input(clock::time_point now,
                                     cfo::AbortReason reason,
                                     std::string detail) {
    auto in = healthy_input(now);
    in.safety = cfo::SafetyDecision{reason, std::move(detail)};
    return in;
}

cfo::MissionConfig small_cfg() {
    cfo::MissionConfig c{};
    c.target_height_m = 0.30f;
    c.forward_velocity_mps = 0.15f;
    c.forward_step_m = 0.10f;
    c.forward_max_steps = 3;
    c.forward_step_duration = 100ms;
    c.takeoff_duration = 100ms;
    c.hover_duration = 100ms;
    c.preland_hover_duration = 100ms;
    c.hlc_land_duration = 100ms;
    c.obstacle_hold_duration = 100ms;
    c.abort_descent_dwell = 100ms;
    c.total_timeout = 10s;
    return c;
}

// Run mission_tick repeatedly, advancing the clock by `step` each tick,
// until either `terminate` is set or `max_ticks` is reached. Returns the
// final context and the trace of state changes.
struct RunResult {
    cfo::MissionContext final;
    std::vector<cfo::MissionState> states;   // each unique state visited
    std::vector<cfo::MissionCommand> commands;
    bool terminated{false};
};

RunResult run(cfo::MissionContext ctx, const cfo::MissionConfig& cfg,
              clock::time_point t0, clock::duration step,
              std::size_t max_ticks,
              auto&& shape_input) {
    RunResult r{};
    r.states.push_back(ctx.state);
    auto now = t0;
    for (std::size_t i = 0; i < max_ticks; ++i) {
        cfo::MissionTickInput in = shape_input(now);
        const auto out = cfo::mission_tick(ctx, in, cfg);
        ctx = out.next;
        r.commands.push_back(out.command);
        if (out.state_changed) r.states.push_back(ctx.state);
        if (out.terminate) {
            r.terminated = true;
            break;
        }
        now += step;
    }
    r.final = ctx;
    return r;
}

} // namespace

TEST_CASE("mission_tick: full happy-path progression") {
    auto cfg = small_cfg();
    cfo::MissionContext ctx{};
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    auto r = run(ctx, cfg, t0, 10ms, /*max_ticks=*/200,
                 [&](auto now) { return healthy_input(now); });

    REQUIRE(r.terminated);
    CHECK(r.final.state == cfo::MissionState::Completed);
    // Linear progression: Idle → TakingOff → HoverStabilizing →
    // ForwardProgress → PreLandHover → HlcLanding → Completed.
    REQUIRE(r.states.size() == 7);
    CHECK(r.states[0] == cfo::MissionState::Idle);
    CHECK(r.states[1] == cfo::MissionState::TakingOff);
    CHECK(r.states[2] == cfo::MissionState::HoverStabilizing);
    CHECK(r.states[3] == cfo::MissionState::ForwardProgress);
    CHECK(r.states[4] == cfo::MissionState::PreLandHover);
    CHECK(r.states[5] == cfo::MissionState::HlcLanding);
    CHECK(r.states[6] == cfo::MissionState::Completed);
}

TEST_CASE("mission_tick: Idle entry fires HlcTakeoff with config height + duration") {
    auto cfg = small_cfg();
    cfg.target_height_m = 0.40f;
    cfg.takeoff_duration = 1500ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::Idle;
    const auto t0 = clock::now();
    ctx.state_entered = t0;
    ctx.mission_started = t0;

    auto out = cfo::mission_tick(ctx, healthy_input(t0), cfg);
    CHECK(out.next.state == cfo::MissionState::TakingOff);
    CHECK(out.state_changed);
    CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcTakeoff);
    CHECK(out.command.hlc_height_m   == doctest::Approx(0.40f));
    CHECK(out.command.hlc_duration_s == doctest::Approx(1.5f));
}

TEST_CASE("mission_tick: TakingOff emits NoOp until takeoff_duration elapses") {
    auto cfg = small_cfg();
    cfg.takeoff_duration = 100ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::TakingOff;
    const auto t0 = clock::now();
    ctx.state_entered = t0;
    ctx.mission_started = t0;

    SUBCASE("mid-takeoff: NoOp, no transition") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 50ms), cfg);
        CHECK(out.next.state == cfo::MissionState::TakingOff);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
    }
    SUBCASE("at deadline: transition to HoverStabilizing, still NoOp") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.next.state == cfo::MissionState::HoverStabilizing);
        CHECK(out.state_changed);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
    }
}

TEST_CASE("mission_tick: HoverStabilizing -> ForwardProgress fires first HlcGoTo step") {
    auto cfg = small_cfg();
    cfg.hover_duration = 100ms;
    cfg.forward_step_m = 0.20f;
    cfg.forward_step_duration = 1500ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::HoverStabilizing;
    const auto t0 = clock::now();
    ctx.state_entered = t0;
    ctx.mission_started = t0;

    SUBCASE("mid-hover: NoOp") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 50ms), cfg);
        CHECK(out.next.state == cfo::MissionState::HoverStabilizing);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
    }
    SUBCASE("at deadline: GO_TO step_m forward, relative=true") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.next.state == cfo::MissionState::ForwardProgress);
        CHECK(out.state_changed);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcGoTo);
        CHECK(out.command.hlc_goto_x_m   == doctest::Approx(0.20f));
        CHECK(out.command.hlc_goto_y_m   == 0.0f);
        CHECK(out.command.hlc_goto_z_m   == 0.0f);
        CHECK(out.command.hlc_goto_relative == true);
        CHECK(out.command.hlc_duration_s == doctest::Approx(1.5f));
        CHECK(out.next.forward_steps_done == 1);
    }
    SUBCASE("at deadline with Blocked: skip ForwardProgress, go to ObstacleHold") {
        auto in = healthy_input(t0 + 100ms);
        in.forward_obstacle = cfo::ObstacleStatus::Blocked;
        auto out = cfo::mission_tick(ctx, in, cfg);
        CHECK(out.next.state == cfo::MissionState::ObstacleHold);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
    }
}

TEST_CASE("mission_tick: ForwardProgress steps + obstacle gating") {
    auto cfg = small_cfg();
    cfg.forward_step_m = 0.10f;
    cfg.forward_step_duration = 100ms;
    cfg.forward_max_steps = 3;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::ForwardProgress;
    ctx.forward_steps_done = 1;        // first step already issued on entry
    const auto t0 = clock::now();
    ctx.state_entered = t0;
    ctx.last_step_emit = t0;
    ctx.mission_started = t0;

    SUBCASE("mid-step, clear: NoOp (HLC GO_TO is executing)") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 50ms), cfg);
        CHECK(out.next.state == cfo::MissionState::ForwardProgress);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
    }
    SUBCASE("mid-step, Blocked: halt + transition to ObstacleHold immediately") {
        // Blocked must preempt the in-flight step. The mission should
        // emit a relative-zero GO_TO to settle the drone in place.
        auto in = healthy_input(t0 + 50ms);
        in.forward_obstacle = cfo::ObstacleStatus::Blocked;
        auto out = cfo::mission_tick(ctx, in, cfg);
        CHECK(out.next.state == cfo::MissionState::ObstacleHold);
        CHECK(out.state_changed);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcGoTo);
        CHECK(out.command.hlc_goto_x_m == 0.0f);
        CHECK(out.command.hlc_goto_y_m == 0.0f);
        CHECK(out.command.hlc_goto_z_m == 0.0f);
        CHECK(out.command.hlc_goto_relative == true);
    }
    SUBCASE("step boundary, clear: emit next step, increment counter") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.next.state == cfo::MissionState::ForwardProgress);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcGoTo);
        CHECK(out.command.hlc_goto_x_m == doctest::Approx(0.10f));
        CHECK(out.next.forward_steps_done == 2);
    }
    SUBCASE("step boundary, blocked: halt + transition to ObstacleHold") {
        auto in = healthy_input(t0 + 100ms);
        in.forward_obstacle = cfo::ObstacleStatus::Blocked;
        auto out = cfo::mission_tick(ctx, in, cfg);
        CHECK(out.next.state == cfo::MissionState::ObstacleHold);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcGoTo);
        CHECK(out.command.hlc_goto_x_m == 0.0f);
    }
    SUBCASE("step boundary at max_steps: transition to PreLandHover") {
        ctx.forward_steps_done = cfg.forward_max_steps;   // already done
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.next.state == cfo::MissionState::PreLandHover);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
    }
    SUBCASE("Caution does NOT block forward progress (v1 policy)") {
        auto in = healthy_input(t0 + 100ms);
        in.forward_obstacle = cfo::ObstacleStatus::Caution;
        auto out = cfo::mission_tick(ctx, in, cfg);
        CHECK(out.next.state == cfo::MissionState::ForwardProgress);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcGoTo);
    }
}

TEST_CASE("mission_tick: ObstacleHold holds, then transitions to PreLandHover") {
    auto cfg = small_cfg();
    cfg.obstacle_hold_duration = 100ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::ObstacleHold;
    const auto t0 = clock::now();
    ctx.state_entered = t0;
    ctx.mission_started = t0;

    SUBCASE("mid-hold: NoOp, no transition") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 50ms), cfg);
        CHECK(out.next.state == cfo::MissionState::ObstacleHold);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
    }
    SUBCASE("hold elapsed: transition to PreLandHover for graceful land") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.next.state == cfo::MissionState::PreLandHover);
        CHECK(out.state_changed);
    }
}

TEST_CASE("mission_tick: PreLandHover transitions to HlcLanding and fires HlcLand") {
    auto cfg = small_cfg();
    cfg.preland_hover_duration = 100ms;
    cfg.hlc_land_duration       = 200ms;
    cfg.hlc_land_target_height_m = 0.0f;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::PreLandHover;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    SUBCASE("mid-PreLandHover: NoOp (HLC holding from previous GO_TO)") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 50ms), cfg);
        CHECK(out.next.state == cfo::MissionState::PreLandHover);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
    }
    SUBCASE("end of PreLandHover transitions to HlcLanding with HlcLand command") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.next.state == cfo::MissionState::HlcLanding);
        CHECK(out.state_changed);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcLand);
        CHECK(out.command.hlc_height_m   == doctest::Approx(cfg.hlc_land_target_height_m));
        CHECK(out.command.hlc_duration_s == doctest::Approx(0.2f));
        CHECK_FALSE(out.terminate);
    }
}

TEST_CASE("mission_tick: HlcLanding emits NoOp during the dwell (no setpoint streaming)") {
    auto cfg = small_cfg();
    cfg.hlc_land_duration = 200ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::HlcLanding;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    SUBCASE("immediately after entry: NoOp, no transition") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 1ms), cfg);
        CHECK(out.next.state == cfo::MissionState::HlcLanding);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
        CHECK_FALSE(out.terminate);
    }
    SUBCASE("mid-dwell: still NoOp, still HlcLanding") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.next.state == cfo::MissionState::HlcLanding);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
        CHECK_FALSE(out.terminate);
    }
}

TEST_CASE("mission_tick: HlcLanding -> Completed after duration, emits Stop and terminates") {
    auto cfg = small_cfg();
    cfg.hlc_land_duration = 200ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::HlcLanding;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    SUBCASE("just before deadline: still HlcLanding, NoOp") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 199ms), cfg);
        CHECK(out.next.state == cfo::MissionState::HlcLanding);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
        CHECK_FALSE(out.terminate);
    }
    SUBCASE("at deadline: transition to Completed, emit HLC STOP, terminate") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 200ms), cfg);
        CHECK(out.next.state == cfo::MissionState::Completed);
        CHECK(out.state_changed);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcStop);
        CHECK(out.terminate);
    }
}

TEST_CASE("mission_tick: operator shutdown mid-flight enters Aborted, fires HlcLand") {
    auto cfg = small_cfg();
    cfg.abort_descent_dwell = 800ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::ForwardProgress;
    ctx.last_commanded_z = 0.30f;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    auto in = healthy_input(t0 + 50ms);
    in.operator_shutdown = true;

    auto out = cfo::mission_tick(ctx, in, cfg);
    CHECK(out.next.state == cfo::MissionState::Aborted);
    CHECK(out.next.abort_reason == cfo::AbortReason::OperatorAbort);
    CHECK(out.state_changed);
    // Abort path is now all-HLC: fire LAND with the dwell as duration,
    // then NoOp until the firmware-side trajectory completes.
    CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcLand);
    CHECK(out.command.hlc_height_m   == 0.0f);
    CHECK(out.command.hlc_duration_s == doctest::Approx(0.8f));
}

TEST_CASE("mission_tick: safety abort propagates reason and detail") {
    auto cfg = small_cfg();
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::HoverStabilizing;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    auto in = aborting_input(t0 + 10ms, cfo::AbortReason::LowBattery,
                             "vbat=2.95V");
    auto out = cfo::mission_tick(ctx, in, cfg);
    CHECK(out.next.state == cfo::MissionState::Aborted);
    CHECK(out.next.abort_reason == cfo::AbortReason::LowBattery);
    CHECK(out.next.abort_detail == "vbat=2.95V");
}

TEST_CASE("mission_tick: Aborted (already-entered) NoOps then terminates with Stop") {
    auto cfg = small_cfg();
    cfg.abort_descent_dwell = 100ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::Aborted;
    ctx.last_commanded_z = 0.30f;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;
    // ctx already in Aborted on entry to this test → state_changed=false
    // path: HLC LAND was sent on the original abort-entry tick (covered
    // by the operator-shutdown test above); from here we just wait it out.

    SUBCASE("mid-dwell — NoOp (HLC LAND still executing)") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 50ms), cfg);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::NoOp);
        CHECK_FALSE(out.terminate);
    }
    SUBCASE("dwell elapsed — HLC STOP and terminate") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::HlcStop);
        CHECK(out.terminate);
    }
}

TEST_CASE("mission_tick: total_timeout aborts even on healthy inputs") {
    auto cfg = small_cfg();
    cfg.total_timeout = 100ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::HoverStabilizing;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    auto out = cfo::mission_tick(ctx, healthy_input(t0 + 200ms), cfg);
    CHECK(out.next.state == cfo::MissionState::Aborted);
    CHECK(out.next.abort_reason == cfo::AbortReason::MissionTimeout);
}

