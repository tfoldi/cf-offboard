#pragma once

#include "state/types.hpp"

#include <chrono>
#include <cstdint>

namespace cfo {

// Identifier for a Multiranger sensor — kept tight (uint8) so we can
// stamp it on every projected obstacle point cheaply.
enum class RangeSensor : std::uint8_t {
    Front = 0,
    Back  = 1,
    Left  = 2,
    Right = 3,
    Up    = 4,
};

[[nodiscard]] inline const char* range_sensor_name(RangeSensor s) noexcept {
    switch (s) {
        case RangeSensor::Front: return "front";
        case RangeSensor::Back:  return "back";
        case RangeSensor::Left:  return "left";
        case RangeSensor::Right: return "right";
        case RangeSensor::Up:    return "up";
    }
    return "?";
}

// Snapshot of all five range readings at a single firmware tick. Values
// in metres; `valid_<sensor>` reflects whether the raw reading was inside
// the deck's usable range (~0.05 m – 4 m).
struct RangeSnapshot {
    std::uint32_t fw_timestamp_ms{};
    float front_m{}, back_m{}, left_m{}, right_m{}, up_m{};
    bool valid_front{}, valid_back{}, valid_left{}, valid_right{}, valid_up{};
    std::chrono::steady_clock::time_point t_recv{};
    std::uint64_t update_count{};
};

// One Multiranger ray defined in `base_link`. origin is the drone body
// origin (we model all five rangers as co-located for slice 6). `dx/dy/dz`
// is a body-frame unit vector along the sensor's axis. `range_m` is the
// measured distance; `valid` is true only when the firmware returned a
// usable reading.
struct BodyFrameRay {
    RangeSensor sensor;
    float origin_x{}, origin_y{}, origin_z{};
    float dx{}, dy{}, dz{};
    float range_m{};
    bool  valid{};
};

// One projected obstacle point in `odom`. Generated only when the source
// range is valid AND the vehicle pose is fresh enough.
struct ObstaclePoint {
    RangeSensor sensor;
    float x{}, y{}, z{};      // odom frame
    std::chrono::steady_clock::time_point t{};
};

// Coarse forward-obstacle classification for mission logic. v1 uses just
// Clear vs Blocked (per MISSION_OBSTACLE_FORWARD.md — the simplest viable
// signal), with Caution available for future use.
enum class ObstacleStatus : std::uint8_t {
    Clear,
    Caution,
    Blocked,
};

[[nodiscard]] inline const char* obstacle_status_name(ObstacleStatus s) noexcept {
    switch (s) {
        case ObstacleStatus::Clear:   return "clear";
        case ObstacleStatus::Caution: return "caution";
        case ObstacleStatus::Blocked: return "blocked";
    }
    return "?";
}

} // namespace cfo
