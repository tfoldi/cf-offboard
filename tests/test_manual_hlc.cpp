#include "control/manual_hlc.hpp"
#include "ui/types.hpp"

#include <doctest/doctest.h>

#include <cmath>
#include <numbers>

namespace {
constexpr float kPi = static_cast<float>(std::numbers::pi);
}

TEST_CASE("step_to_delta: at yaw=0, body frame == world frame") {
    cfo::ManualHlcConfig cfg{};
    cfg.step_xy_m = 0.20f;
    cfg.step_z_m  = 0.10f;
    cfg.step_yaw_rad = 0.5f;
    constexpr float yaw0 = 0.0f;

    SUBCASE("forward / back along world +x") {
        const auto fwd = cfo::step_to_delta(cfo::ManualStep::XPlus, cfg, yaw0);
        CHECK(fwd.dx == doctest::Approx(+0.20f));
        CHECK(fwd.dy == doctest::Approx(0.0f).epsilon(1e-5));
        CHECK(fwd.dz == 0.0f);
        CHECK(fwd.dyaw_rad == 0.0f);
        const auto back = cfo::step_to_delta(cfo::ManualStep::XMinus, cfg, yaw0);
        CHECK(back.dx == doctest::Approx(-0.20f));
    }
    SUBCASE("left / right strafe along world +y / -y") {
        const auto left = cfo::step_to_delta(cfo::ManualStep::YPlus, cfg, yaw0);
        CHECK(left.dx == doctest::Approx(0.0f).epsilon(1e-5));
        CHECK(left.dy == doctest::Approx(+0.20f));
        const auto right = cfo::step_to_delta(cfo::ManualStep::YMinus, cfg, yaw0);
        CHECK(right.dy == doctest::Approx(-0.20f));
    }
    SUBCASE("up / down use step_z_m, no rotation") {
        const auto up = cfo::step_to_delta(cfo::ManualStep::ZPlus, cfg, yaw0);
        CHECK(up.dz == doctest::Approx(+0.10f));
        const auto down = cfo::step_to_delta(cfo::ManualStep::ZMinus, cfg, yaw0);
        CHECK(down.dz == doctest::Approx(-0.10f));
    }
    SUBCASE("None yields zero delta") {
        const auto z = cfo::step_to_delta(cfo::ManualStep::None, cfg, yaw0);
        CHECK(z.dx == 0.0f);
        CHECK(z.dy == 0.0f);
        CHECK(z.dz == 0.0f);
        CHECK(z.dyaw_rad == 0.0f);
    }
}

TEST_CASE("step_to_delta: yaw deltas pass through unchanged (no rotation applied to yaw)") {
    cfo::ManualHlcConfig cfg{};
    cfg.step_yaw_rad = 0.2618f;   // ~15°

    SUBCASE("yaw left is positive (CCW)") {
        const auto y = cfo::step_to_delta(cfo::ManualStep::YawPlus, cfg, 0.0f);
        CHECK(y.dyaw_rad == doctest::Approx(+0.2618f));
        CHECK(y.dx == 0.0f);
        CHECK(y.dy == 0.0f);
    }
    SUBCASE("yaw right is negative (CW)") {
        const auto y = cfo::step_to_delta(cfo::ManualStep::YawMinus, cfg, 0.0f);
        CHECK(y.dyaw_rad == doctest::Approx(-0.2618f));
    }
    SUBCASE("body yaw doesn't affect the dyaw value") {
        // Yaw of 90° doesn't change the yaw step itself — it's a delta,
        // not a coordinate.
        const auto a = cfo::step_to_delta(cfo::ManualStep::YawPlus, cfg, 0.0f);
        const auto b = cfo::step_to_delta(cfo::ManualStep::YawPlus, cfg, kPi/2);
        CHECK(a.dyaw_rad == doctest::Approx(b.dyaw_rad));
    }
}

TEST_CASE("step_to_delta: body→world rotation by current yaw") {
    cfo::ManualHlcConfig cfg{};
    cfg.step_xy_m = 1.0f;   // unit step for clean trig

    SUBCASE("yaw = +90° rotates body +x into world +y") {
        const auto fwd = cfo::step_to_delta(cfo::ManualStep::XPlus, cfg, kPi/2);
        CHECK(fwd.dx == doctest::Approx(0.0f).epsilon(1e-5));
        CHECK(fwd.dy == doctest::Approx(+1.0f));
    }
    SUBCASE("yaw = +90° rotates body +y (left strafe) into world -x") {
        const auto left = cfo::step_to_delta(cfo::ManualStep::YPlus, cfg, kPi/2);
        CHECK(left.dx == doctest::Approx(-1.0f));
        CHECK(left.dy == doctest::Approx(0.0f).epsilon(1e-5));
    }
    SUBCASE("yaw = 180° flips body +x to world -x") {
        const auto fwd = cfo::step_to_delta(cfo::ManualStep::XPlus, cfg, kPi);
        CHECK(fwd.dx == doctest::Approx(-1.0f));
        CHECK(fwd.dy == doctest::Approx(0.0f).epsilon(1e-5));
    }
    SUBCASE("yaw = +45° splits body +x evenly across world +x and +y") {
        const auto fwd = cfo::step_to_delta(cfo::ManualStep::XPlus, cfg, kPi/4);
        const float expected = 1.0f / std::sqrt(2.0f);
        CHECK(fwd.dx == doctest::Approx(expected));
        CHECK(fwd.dy == doctest::Approx(expected));
    }
    SUBCASE("z step is yaw-independent") {
        cfg.step_z_m = 0.10f;
        const auto up = cfo::step_to_delta(cfo::ManualStep::ZPlus, cfg, kPi/3);
        CHECK(up.dx == 0.0f);
        CHECK(up.dy == 0.0f);
        CHECK(up.dz == doctest::Approx(+0.10f));
    }
}

TEST_CASE("is_airborne: only true while motors are on") {
    cfo::AppStatus s{};
    CHECK_FALSE(cfo::is_airborne(s));

    SUBCASE("Mission active is airborne") {
        s.mission_active = true;
        CHECK(cfo::is_airborne(s));
    }
    SUBCASE("ManualHlc OnGround is NOT airborne") {
        s.mode = cfo::AppMode::ManualHlc;
        s.manual_state = cfo::ManualState::OnGround;
        CHECK_FALSE(cfo::is_airborne(s));
    }
    SUBCASE("ManualHlc TakingOff is airborne") {
        s.mode = cfo::AppMode::ManualHlc;
        s.manual_state = cfo::ManualState::TakingOff;
        CHECK(cfo::is_airborne(s));
    }
    SUBCASE("ManualHlc Flying is airborne") {
        s.mode = cfo::AppMode::ManualHlc;
        s.manual_state = cfo::ManualState::Flying;
        CHECK(cfo::is_airborne(s));
    }
    SUBCASE("ManualHlc Landing is airborne") {
        s.mode = cfo::AppMode::ManualHlc;
        s.manual_state = cfo::ManualState::Landing;
        CHECK(cfo::is_airborne(s));
    }
    SUBCASE("Idle mode never airborne by itself") {
        s.mode = cfo::AppMode::Idle;
        CHECK_FALSE(cfo::is_airborne(s));
    }
}
