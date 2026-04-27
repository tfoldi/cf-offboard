#pragma once

#include "crazyflie/link/interfaces.hpp"
#include "logging/mcap_logger.hpp"
#include "safety/types.hpp"
#include "state/state_store.hpp"
#include "ui/types.hpp"

#include <atomic>
#include <chrono>

namespace cfo {

// Tunables for keyboard-driven manual control via HLC primitives.
// Bounded step sizes keep a single keypress operator-safe — even rapid
// presses can only commit to the small per-step displacement.
struct ManualHlcConfig {
    float takeoff_height_m{0.30f};
    std::chrono::milliseconds takeoff_duration{1500};

    // Discrete step deltas applied as relative HLC GO_TO commands.
    float step_xy_m{0.15f};      // forward/back/left/right step
    float step_z_m{0.10f};       // up/down step
    float step_yaw_rad{0.2618f}; // yaw left/right step (~15°)
    std::chrono::milliseconds step_duration{1000};

    // Land + post-land dwell before the HLC STOP that finally disarms.
    std::chrono::milliseconds land_duration{2000};
    std::chrono::milliseconds touchdown_dwell{500};

    // Faster descent on safety/operator abort.
    std::chrono::milliseconds abort_descent_dwell{1000};

    // Fixed-rate poll period for safety + intent processing.
    std::chrono::milliseconds tick_period{20};

    SafetyConfig safety{};
};

// Pure helper: convert a ManualStep into a *world-frame* relative
// (dx, dy, dz, dyaw). The body-frame translation step is rotated by
// the supplied current yaw so "forward" means the nose direction.
// Yaw deltas pass through unchanged. Exposed for testing.
struct StepDelta { float dx, dy, dz, dyaw_rad; };
[[nodiscard]] StepDelta step_to_delta(ManualStep s,
                                       const ManualHlcConfig& cfg,
                                       float current_yaw_rad) noexcept;

// Operator-driven control loop. Runs on the calling thread. Owns the
// arm bracket and the manual sub-state machine; uses HLC primitives only.
//
// Returns when:
//   - operator presses 'm' again (intents.exit_manual) — exits cleanly,
//     auto-landing first if airborne;
//   - app-wide shutdown (Ctrl-C / 'q' from idle) — same auto-land path;
//   - safety abort (telemetry stale / low battery) — auto-land then stop.
//
// AppStatus is updated per state transition so the TUI sees live state.
void run_manual_hlc(ICrazyflieLink& link,
                    const StateStore& state,
                    MCAPLogger& logger,
                    AppStatusStore& app_status,
                    OperatorIntents& intents,
                    const ManualHlcConfig& cfg,
                    std::atomic<bool>& shutdown_requested);

} // namespace cfo
