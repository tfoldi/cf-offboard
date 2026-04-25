#pragma once

#include "mission/types.hpp"
#include "safety/types.hpp"

#include <chrono>

namespace cfo {

// Control-loop wiring. Trajectory, hold times, and abort dwell live in the
// MissionConfig; this struct only contains what the loop itself needs.
struct ControlLoopConfig {
    std::chrono::milliseconds period{20};   // 50 Hz
    SafetyConfig  safety{};
    MissionConfig mission{};
};

} // namespace cfo
