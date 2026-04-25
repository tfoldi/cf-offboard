#include "crazyflie/log/log_session.hpp"
#include "crazyflie/protocol/types.hpp"
#include "state/state_store.hpp"
#include "state/types.hpp"

#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <numbers>

namespace {

cfo::LogBlockSample make_sample(float x, float y, float z,
                                float vbat,
                                float roll, float pitch, float yaw,
                                std::uint32_t fw_t = 0) {
    cfo::LogBlockSample s{};
    s.timestamp_ms = fw_t;
    s.x = x; s.y = y; s.z = z;
    s.vbat = vbat;
    s.roll = roll; s.pitch = pitch; s.yaw = yaw;
    return s;
}

constexpr float kPi = static_cast<float>(std::numbers::pi);

} // namespace

TEST_CASE("decoded log-block sample updates VehicleState") {
    cfo::StateStore store;
    const auto t = std::chrono::steady_clock::now();
    const auto sample = make_sample(1.0f, 2.0f, 3.0f, 3.7f,
                                    0.0f, 0.0f, 0.0f);

    store.apply(cfo::pose_from_log_sample(sample, t));
    store.apply(cfo::battery_from_log_sample(sample, t));

    const auto s = store.snapshot();
    CHECK(s.position.x == 1.0f);
    CHECK(s.position.y == 2.0f);
    CHECK(s.position.z == 3.0f);
    CHECK(s.battery_voltage == doctest::Approx(3.7f));
    CHECK(s.update_count == 2);              // two apply() calls
    CHECK(s.last_update == t);
}

TEST_CASE("attitude deg → rad conversion is correct") {
    cfo::StateStore store;
    const auto t = std::chrono::steady_clock::now();

    SUBCASE("zero stays zero") {
        store.apply(cfo::pose_from_log_sample(
            make_sample(0,0,0, 0, 0,0,0), t));
        const auto s = store.snapshot();
        CHECK(s.attitude.roll == 0.0f);
        CHECK(s.attitude.pitch == 0.0f);
        CHECK(s.attitude.yaw == 0.0f);
    }
    SUBCASE("180 deg maps to pi") {
        store.apply(cfo::pose_from_log_sample(
            make_sample(0,0,0, 0, 180.0f, 0, 0), t));
        CHECK(store.snapshot().attitude.roll
              == doctest::Approx(kPi).epsilon(1e-5));
    }
    SUBCASE("-90 deg maps to -pi/2") {
        store.apply(cfo::pose_from_log_sample(
            make_sample(0,0,0, 0, 0, -90.0f, 0), t));
        CHECK(store.snapshot().attitude.pitch
              == doctest::Approx(-kPi / 2).epsilon(1e-5));
    }
    SUBCASE("45 deg maps to pi/4") {
        store.apply(cfo::pose_from_log_sample(
            make_sample(0,0,0, 0, 0, 0, 45.0f), t));
        CHECK(store.snapshot().attitude.yaw
              == doctest::Approx(kPi / 4).epsilon(1e-5));
    }
    SUBCASE("free-form deg_to_rad helper agrees with ratio") {
        CHECK(cfo::deg_to_rad(180.0f) == doctest::Approx(kPi).epsilon(1e-5));
        CHECK(cfo::deg_to_rad(0.0f) == 0.0f);
        CHECK(cfo::deg_to_rad(90.0f) == doctest::Approx(kPi / 2).epsilon(1e-5));
    }
}

TEST_CASE("battery update does not clobber pose") {
    cfo::StateStore store;
    const auto t1 = std::chrono::steady_clock::now();

    // Set a known pose first.
    const auto pose_sample = make_sample(5.0f, 6.0f, 7.0f, 0.0f,
                                         10.0f, -20.0f, 30.0f);
    store.apply(cfo::pose_from_log_sample(pose_sample, t1));

    // Now apply only a battery update with a different value.
    const auto t2 = t1 + std::chrono::milliseconds{50};
    store.apply(cfo::BatteryTelemetry{4.05f, t2});

    const auto s = store.snapshot();
    // Pose preserved...
    CHECK(s.position.x == 5.0f);
    CHECK(s.position.y == 6.0f);
    CHECK(s.position.z == 7.0f);
    CHECK(s.attitude.roll  == doctest::Approx(cfo::deg_to_rad(10.0f)));
    CHECK(s.attitude.pitch == doctest::Approx(cfo::deg_to_rad(-20.0f)));
    CHECK(s.attitude.yaw   == doctest::Approx(cfo::deg_to_rad(30.0f)));
    // ...battery updated, timestamp advanced, count bumped.
    CHECK(s.battery_voltage == doctest::Approx(4.05f));
    CHECK(s.update_count == 2);
    CHECK(s.last_update == t2);
}

TEST_CASE("update counter and last_update advance with each apply") {
    cfo::StateStore store;
    {
        const auto s0 = store.snapshot();
        CHECK(s0.update_count == 0);
        CHECK(s0.last_update == cfo::VehicleState::time_point{});
    }

    const auto t1 = std::chrono::steady_clock::now();
    store.apply(cfo::PoseTelemetry{cfo::Vec3{}, cfo::Attitude{}, t1});
    {
        const auto s1 = store.snapshot();
        CHECK(s1.update_count == 1);
        CHECK(s1.last_update == t1);
    }

    const auto t2 = t1 + std::chrono::milliseconds{10};
    store.apply(cfo::BatteryTelemetry{3.9f, t2});
    {
        const auto s2 = store.snapshot();
        CHECK(s2.update_count == 2);
        CHECK(s2.last_update == t2);
    }

    // Out-of-order timestamps: apply() does not enforce monotonicity — it
    // simply stamps with whatever the message carries. Document that here.
    const auto t_old = t1 - std::chrono::seconds{1};
    store.apply(cfo::BatteryTelemetry{3.85f, t_old});
    {
        const auto s3 = store.snapshot();
        CHECK(s3.update_count == 3);
        CHECK(s3.last_update == t_old);
    }
}
