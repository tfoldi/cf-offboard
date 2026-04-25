#include "safety/safety.hpp"
#include "safety/types.hpp"
#include "state/types.hpp"

#include <doctest/doctest.h>

#include <chrono>

namespace {

cfo::VehicleState make_state(float vbat,
                             cfo::VehicleState::time_point last_update,
                             std::uint64_t update_count = 1) {
    cfo::VehicleState s{};
    s.battery_voltage = vbat;
    s.last_update = last_update;
    s.update_count = update_count;
    return s;
}

} // namespace

TEST_CASE("check_safety: no telemetry yet → TelemetryStale") {
    cfo::SafetyConfig cfg{};
    cfo::VehicleState s{};       // update_count = 0
    const auto now = std::chrono::steady_clock::now();
    auto d = cfo::check_safety(s, now, cfg);
    CHECK(d.reason == cfo::AbortReason::TelemetryStale);
    CHECK(cfo::aborted(d));
}

TEST_CASE("check_safety: fresh sample, healthy battery → None") {
    cfo::SafetyConfig cfg{};
    const auto now = std::chrono::steady_clock::now();
    auto s = make_state(3.7f, now);
    auto d = cfo::check_safety(s, now, cfg);
    CHECK(d.reason == cfo::AbortReason::None);
    CHECK_FALSE(cfo::aborted(d));
}

TEST_CASE("check_safety: telemetry older than max_age → TelemetryStale") {
    cfo::SafetyConfig cfg{};
    cfg.max_telemetry_age = std::chrono::milliseconds{200};
    const auto now = std::chrono::steady_clock::now();
    auto s = make_state(3.7f, now - std::chrono::milliseconds{500});
    CHECK(cfo::check_safety(s, now, cfg).reason
          == cfo::AbortReason::TelemetryStale);
}

TEST_CASE("check_safety: battery below threshold → LowBattery") {
    cfo::SafetyConfig cfg{};
    cfg.battery_min_volt = 3.10f;
    const auto now = std::chrono::steady_clock::now();
    auto s = make_state(3.05f, now);
    auto d = cfo::check_safety(s, now, cfg);
    CHECK(d.reason == cfo::AbortReason::LowBattery);
    CHECK(d.detail.find("3.05") != std::string::npos);
}

TEST_CASE("check_safety: battery exactly at threshold is NOT aborted") {
    cfo::SafetyConfig cfg{};
    cfg.battery_min_volt = 3.10f;
    const auto now = std::chrono::steady_clock::now();
    auto s = make_state(3.10f, now);
    CHECK(cfo::check_safety(s, now, cfg).reason == cfo::AbortReason::None);
}

