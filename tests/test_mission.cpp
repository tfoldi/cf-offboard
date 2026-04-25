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
    c.takeoff_duration = 100ms;
    c.hover_duration = 100ms;
    c.forward_duration = 100ms;
    c.preland_hover_duration = 100ms;
    c.land_duration = 100ms;
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
    // States visited in order — Idle is the entry state, the rest follow.
    REQUIRE(r.states.size() == 7);
    CHECK(r.states[0] == cfo::MissionState::Idle);
    CHECK(r.states[1] == cfo::MissionState::TakingOff);
    CHECK(r.states[2] == cfo::MissionState::HoverStabilizing);
    CHECK(r.states[3] == cfo::MissionState::ForwardSegment);
    CHECK(r.states[4] == cfo::MissionState::PreLandHover);
    CHECK(r.states[5] == cfo::MissionState::Landing);
    CHECK(r.states[6] == cfo::MissionState::Completed);
}

TEST_CASE("mission_tick: TakingOff ramps z linearly from 0 to target_height") {
    auto cfg = small_cfg();
    cfg.takeoff_duration = 100ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::TakingOff;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    SUBCASE("at t=0, z is 0") {
        auto in = healthy_input(t0);
        auto out = cfo::mission_tick(ctx, in, cfg);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::Hover);
        CHECK(out.command.z_target_m == doctest::Approx(0.0f));
    }
    SUBCASE("at t=50ms (halfway), z is 0.15") {
        auto in = healthy_input(t0 + 50ms);
        auto out = cfo::mission_tick(ctx, in, cfg);
        CHECK(out.command.z_target_m == doctest::Approx(0.15f).epsilon(0.01));
    }
    SUBCASE("at t=duration, z hits target and we transition") {
        auto in = healthy_input(t0 + 100ms);
        auto out = cfo::mission_tick(ctx, in, cfg);
        CHECK(out.next.state == cfo::MissionState::HoverStabilizing);
        CHECK(out.state_changed);
    }
}

TEST_CASE("mission_tick: ForwardSegment commands forward velocity at hover height") {
    auto cfg = small_cfg();
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::ForwardSegment;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    auto out = cfo::mission_tick(ctx, healthy_input(t0 + 50ms), cfg);
    CHECK(out.command.kind == cfo::MissionCommand::Kind::Hover);
    CHECK(out.command.vx_mps == doctest::Approx(cfg.forward_velocity_mps));
    CHECK(out.command.vy_mps == doctest::Approx(0.0f));
    CHECK(out.command.z_target_m == doctest::Approx(cfg.target_height_m));
}

TEST_CASE("mission_tick: Landing ramps z back to 0 and emits Stop on transition to Completed") {
    auto cfg = small_cfg();
    cfg.land_duration = 100ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::Landing;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    SUBCASE("mid-landing emits hover with descending z") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 50ms), cfg);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::Hover);
        CHECK(out.command.z_target_m == doctest::Approx(0.15f).epsilon(0.01));
    }
    SUBCASE("end of landing window emits Stop and terminates") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.next.state == cfo::MissionState::Completed);
        CHECK(out.terminate);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::Stop);
    }
}

TEST_CASE("mission_tick: operator shutdown mid-flight enters Aborted") {
    auto cfg = small_cfg();
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::ForwardSegment;
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
    CHECK(out.command.kind == cfo::MissionCommand::Kind::Hover);
    // First abort tick should still command roughly current altitude.
    CHECK(out.command.z_target_m == doctest::Approx(0.30f).epsilon(0.05));
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

TEST_CASE("mission_tick: Aborted state ramps z to 0 then terminates with Stop") {
    auto cfg = small_cfg();
    cfg.abort_descent_dwell = 100ms;
    cfo::MissionContext ctx{};
    ctx.state = cfo::MissionState::Aborted;
    ctx.last_commanded_z = 0.30f;
    const auto t0 = clock::now();
    ctx.mission_started = t0;
    ctx.state_entered = t0;

    SUBCASE("mid-dwell — descending hover") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 50ms), cfg);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::Hover);
        CHECK(out.command.z_target_m == doctest::Approx(0.15f).epsilon(0.05));
        CHECK_FALSE(out.terminate);
    }
    SUBCASE("dwell elapsed — Stop and terminate") {
        auto out = cfo::mission_tick(ctx, healthy_input(t0 + 100ms), cfg);
        CHECK(out.command.kind == cfo::MissionCommand::Kind::Stop);
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

TEST_CASE("mission_tick: Idle on first call enters TakingOff and emits z=0") {
    auto cfg = small_cfg();
    cfo::MissionContext ctx{};
    const auto t0 = clock::now();
    ctx.state = cfo::MissionState::Idle;
    ctx.state_entered = t0;
    ctx.mission_started = t0;

    auto out = cfo::mission_tick(ctx, healthy_input(t0), cfg);
    CHECK(out.next.state == cfo::MissionState::TakingOff);
    CHECK(out.state_changed);
    CHECK(out.command.kind == cfo::MissionCommand::Kind::Hover);
    CHECK(out.command.z_target_m == doctest::Approx(0.0f));
}
