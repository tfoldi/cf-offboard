#pragma once

#include "state/types.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace cfo {

enum class AbortReason : std::uint8_t {
    None,
    TelemetryStale,    // last_update older than max_telemetry_age
    LowBattery,        // battery_voltage below threshold
    LinkLost,          // (placeholder — set by control loop on send failure)
    OperatorAbort,     // shutdown signal received mid-flight
    MissionTimeout,    // total mission duration exceeded
};

// Tunables for the safety supervisor. Defaults are conservative for a
// Crazyflie 2.x with flow deck running the slice-(a) demo (≤ 0.4 m hover).
struct SafetyConfig {
    // Maximum age of the most recent telemetry sample before we consider
    // the link/state estimator unreliable. Set to several inter-sample
    // periods (samples arrive at 20 Hz = 50 ms) so transient host or radio
    // jitter doesn't trip a false abort during otherwise-healthy flight.
    std::chrono::milliseconds max_telemetry_age{500};

    // Battery cutoff. Set well below the firmware's own low-voltage
    // threshold so motor-startup current sag (≈0.4–0.7 V transient drop)
    // does not trip a false abort during takeoff. The firmware initiates
    // its own auto-land in the ~3.0 V range as a backstop.
    float battery_min_volt{2.90f};
};

struct SafetyDecision {
    AbortReason reason{AbortReason::None};
    std::string detail;
};

[[nodiscard]] inline bool aborted(const SafetyDecision& d) noexcept {
    return d.reason != AbortReason::None;
}

} // namespace cfo
