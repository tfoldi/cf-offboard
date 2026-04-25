#pragma once

#include "safety/types.hpp"
#include "state/types.hpp"

#include <chrono>
#include <string>

namespace cfo {

// Pure: evaluate the safety supervisor against a single state snapshot.
// No IO, no time source — `now` is passed in so the same code is used in
// replay. Caller decides what to do with the decision (typically: send a
// stop/descent setpoint and exit the control loop).
//
// If `state.update_count == 0`, telemetry has never arrived; treated as
// stale.
[[nodiscard]] inline SafetyDecision
check_safety(const VehicleState& state,
             VehicleState::time_point now,
             const SafetyConfig& cfg) noexcept {
    // Telemetry freshness — a never-updated state is also stale.
    if (state.update_count == 0) {
        return SafetyDecision{AbortReason::TelemetryStale,
                              "no telemetry yet"};
    }
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - state.last_update);
    if (age > cfg.max_telemetry_age) {
        return SafetyDecision{AbortReason::TelemetryStale,
                              "age=" + std::to_string(age.count()) + "ms"};
    }

    // Battery — strictly less-than so a value exactly at the threshold is OK.
    if (state.battery_voltage < cfg.battery_min_volt) {
        return SafetyDecision{
            AbortReason::LowBattery,
            "vbat=" + std::to_string(state.battery_voltage) + "V"};
    }

    return SafetyDecision{};  // None
}

} // namespace cfo
