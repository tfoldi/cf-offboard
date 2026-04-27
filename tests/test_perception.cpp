#include "crazyflie/protocol/types.hpp"
#include "perception/perception.hpp"
#include "perception/types.hpp"
#include "state/types.hpp"

#include <doctest/doctest.h>

#include <chrono>
#include <numbers>

namespace {

constexpr float kPi = static_cast<float>(std::numbers::pi);
using clock = std::chrono::steady_clock;

cfo::VehicleState fresh_state(float x, float y, float z, float yaw_rad,
                              clock::time_point t) {
    cfo::VehicleState s{};
    s.position.x = x; s.position.y = y; s.position.z = z;
    s.attitude.yaw = yaw_rad;
    s.last_update = t;
    s.update_count = 1;
    return s;
}

} // namespace

TEST_CASE("make_range_snapshot: validity window 0.05–4.0 m") {
    cfo::PerceptionConfig cfg{};
    cfo::RangeBlockSample raw{};
    raw.timestamp_ms = 12345;
    raw.front_mm = 800;     // 0.80 m → valid
    raw.back_mm  = 30;      // 0.03 m → below min, invalid
    raw.left_mm  = 4500;    // 4.5 m → above max, invalid
    raw.right_mm = 100;     // 0.10 m → valid (just above min)
    raw.up_mm    = 0;       // sensor returned 0 → invalid

    const auto t = clock::now();
    const auto s = cfo::make_range_snapshot(raw, cfg, t, 1);

    CHECK(s.fw_timestamp_ms == 12345);
    CHECK(s.front_m == doctest::Approx(0.80f));
    CHECK(s.valid_front);
    CHECK_FALSE(s.valid_back);
    CHECK_FALSE(s.valid_left);
    CHECK(s.valid_right);
    CHECK_FALSE(s.valid_up);
    CHECK(s.t_recv == t);
    CHECK(s.update_count == 1);
}

TEST_CASE("rays_from_snapshot: five canonical body-frame directions") {
    cfo::RangeSnapshot s{};
    s.front_m = 0.5f; s.valid_front = true;
    s.back_m  = 0.6f; s.valid_back  = true;
    s.left_m  = 0.7f; s.valid_left  = true;
    s.right_m = 0.8f; s.valid_right = true;
    s.up_m    = 0.9f; s.valid_up    = true;

    const auto rays = cfo::rays_from_snapshot(s);
    REQUIRE(rays.size() == 5);

    CHECK(rays[0].sensor == cfo::RangeSensor::Front);
    CHECK(rays[0].dx == +1.0f); CHECK(rays[0].dy == 0.0f); CHECK(rays[0].dz == 0.0f);
    CHECK(rays[0].range_m == doctest::Approx(0.5f));
    CHECK(rays[0].valid);

    CHECK(rays[1].sensor == cfo::RangeSensor::Back);
    CHECK(rays[1].dx == -1.0f);
    CHECK(rays[2].sensor == cfo::RangeSensor::Left);
    CHECK(rays[2].dy == +1.0f);
    CHECK(rays[3].sensor == cfo::RangeSensor::Right);
    CHECK(rays[3].dy == -1.0f);
    CHECK(rays[4].sensor == cfo::RangeSensor::Up);
    CHECK(rays[4].dz == +1.0f);
}

TEST_CASE("project_to_odom: yaw=0 places forward ray at vehicle.x + range") {
    const auto t = clock::now();
    auto v = fresh_state(/*x=*/2.0f, /*y=*/-1.0f, /*z=*/0.5f,
                         /*yaw=*/0.0f, t);
    cfo::PerceptionConfig cfg{};

    cfo::BodyFrameRay front{};
    front.sensor = cfo::RangeSensor::Front;
    front.dx = 1.0f; front.range_m = 1.0f; front.valid = true;

    auto pt = cfo::project_to_odom(front, v, t, cfg);
    REQUIRE(pt);
    CHECK(pt->sensor == cfo::RangeSensor::Front);
    CHECK(pt->x == doctest::Approx(3.0f));   // 2.0 + 1.0
    CHECK(pt->y == doctest::Approx(-1.0f));
    CHECK(pt->z == doctest::Approx(0.5f));
}

TEST_CASE("project_to_odom: yaw rotates body-frame +x into world +y at +90°") {
    const auto t = clock::now();
    auto v = fresh_state(0.0f, 0.0f, 0.0f, kPi / 2.0f, t);
    cfo::PerceptionConfig cfg{};

    cfo::BodyFrameRay front{};
    front.sensor = cfo::RangeSensor::Front;
    front.dx = 1.0f; front.range_m = 0.5f; front.valid = true;

    auto pt = cfo::project_to_odom(front, v, t, cfg);
    REQUIRE(pt);
    CHECK(pt->x == doctest::Approx(0.0f).epsilon(1e-5));
    CHECK(pt->y == doctest::Approx(0.5f));
}

TEST_CASE("project_to_odom: invalid ray returns nullopt") {
    const auto t = clock::now();
    auto v = fresh_state(0, 0, 0, 0, t);
    cfo::PerceptionConfig cfg{};
    cfo::BodyFrameRay r{};
    r.dx = 1.0f; r.range_m = 1.0f; r.valid = false;
    CHECK_FALSE(cfo::project_to_odom(r, v, t, cfg).has_value());
}

TEST_CASE("project_to_odom: stale pose returns nullopt") {
    const auto t = clock::now();
    auto v = fresh_state(0, 0, 0, 0, t - std::chrono::seconds{2});
    cfo::PerceptionConfig cfg{};
    cfg.max_pose_age = std::chrono::milliseconds{500};
    cfo::BodyFrameRay r{};
    r.dx = 1.0f; r.range_m = 1.0f; r.valid = true;
    CHECK_FALSE(cfo::project_to_odom(r, v, t, cfg).has_value());
}

TEST_CASE("project_to_odom: never-updated state (update_count=0) returns nullopt") {
    cfo::VehicleState v{};   // update_count = 0
    cfo::PerceptionConfig cfg{};
    cfo::BodyFrameRay r{};
    r.dx = 1.0f; r.range_m = 1.0f; r.valid = true;
    CHECK_FALSE(cfo::project_to_odom(r, v, clock::now(), cfg).has_value());
}

TEST_CASE("classify_forward: thresholds and invalidity") {
    cfo::PerceptionConfig cfg{};
    cfg.blocked_m = 0.50f;
    cfg.caution_m = 1.00f;

    cfo::RangeSnapshot s{};
    s.valid_front = true;

    SUBCASE("front 0.30 m → Blocked") {
        s.front_m = 0.30f;
        CHECK(cfo::classify_forward(s, cfg) == cfo::ObstacleStatus::Blocked);
    }
    SUBCASE("front 0.75 m → Caution") {
        s.front_m = 0.75f;
        CHECK(cfo::classify_forward(s, cfg) == cfo::ObstacleStatus::Caution);
    }
    SUBCASE("front 2.00 m → Clear") {
        s.front_m = 2.00f;
        CHECK(cfo::classify_forward(s, cfg) == cfo::ObstacleStatus::Clear);
    }
    SUBCASE("invalid front → Clear (nothing seen close)") {
        s.valid_front = false;
        s.front_m = 0.30f;     // value is meaningless when invalid
        CHECK(cfo::classify_forward(s, cfg) == cfo::ObstacleStatus::Clear);
    }
}

TEST_CASE("ObstacleStore: prunes points older than the lifetime window") {
    cfo::PerceptionConfig cfg{};
    cfg.obstacle_lifetime = std::chrono::milliseconds{1000};
    cfo::ObstacleStore store{cfg};

    const auto t0 = clock::now();
    cfo::ObstaclePoint p{};
    p.t = t0; p.x = 1; store.push(p);
    p.t = t0 + std::chrono::milliseconds{500}; p.x = 2; store.push(p);
    p.t = t0 + std::chrono::milliseconds{1500}; p.x = 3; store.push(p);

    CHECK(store.size() == 3);

    // At t0 + 1.6 s, the first two are stale (older than 1 s).
    store.prune(t0 + std::chrono::milliseconds{1600});
    CHECK(store.size() == 1);

    const auto remaining = store.snapshot();
    REQUIRE(remaining.size() == 1);
    CHECK(remaining[0].x == doctest::Approx(3.0f));
}
