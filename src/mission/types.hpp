#pragma once

#include "safety/types.hpp"
#include "state/types.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace cfo {

// First-flight mission states (per MISSION_FIRST_FLIGHT.md). Linear
// progression Idle → TakingOff → HoverStabilizing → ForwardSegment →
// PreLandHover → Landing → Completed. Aborted is the terminal failure
// branch entered from any non-terminal state on safety/operator abort.
enum class MissionState : std::uint8_t {
    Idle,
    TakingOff,
    HoverStabilizing,
    ForwardSegment,
    PreLandHover,
    Landing,
    Completed,
    Aborted,
};

[[nodiscard]] inline const char* state_name(MissionState s) noexcept {
    switch (s) {
        case MissionState::Idle:             return "Idle";
        case MissionState::TakingOff:        return "TakingOff";
        case MissionState::HoverStabilizing: return "HoverStabilizing";
        case MissionState::ForwardSegment:   return "ForwardSegment";
        case MissionState::PreLandHover:     return "PreLandHover";
        case MissionState::Landing:          return "Landing";
        case MissionState::Completed:        return "Completed";
        case MissionState::Aborted:          return "Aborted";
    }
    return "Unknown";
}

// Tunables for the first-flight mission. Hardcoded into the binary today;
// will move into AppConfig as a follow-up.
struct MissionConfig {
    float target_height_m{0.30f};
    float forward_velocity_mps{0.15f};

    std::chrono::milliseconds takeoff_duration{1500};
    std::chrono::milliseconds hover_duration{1500};
    std::chrono::milliseconds forward_duration{1500};
    std::chrono::milliseconds preland_hover_duration{1000};
    std::chrono::milliseconds land_duration{1500};

    // After abort: ramp current target_z down to 0 over this window, then
    // emit the disarming stop.
    std::chrono::milliseconds abort_descent_dwell{1000};

    // Hard wall-clock cap. Belt-and-braces against any logic bug that
    // would otherwise stream setpoints forever.
    std::chrono::milliseconds total_timeout{20'000};
};

// Outbound command produced by mission_tick. The caller (control loop)
// translates this into a CRTP packet and sends it.
struct MissionCommand {
    enum class Kind : std::uint8_t { Stop, Hover };
    Kind  kind{Kind::Hover};
    float vx_mps{};
    float vy_mps{};
    float yaw_rate_dps{};
    float z_target_m{};
};

// Per-mission-instance state. Updated by mission_tick on every call —
// the field set is small enough that the value is cheap to copy.
struct MissionContext {
    MissionState state{MissionState::Idle};
    std::chrono::steady_clock::time_point state_entered{};
    std::chrono::steady_clock::time_point mission_started{};

    AbortReason abort_reason{AbortReason::None};
    std::string abort_detail;

    // Last commanded altitude — used to start the abort descent ramp from
    // wherever we were, rather than yanking from any value to 0.
    float last_commanded_z{0.0f};
};

// Inputs the supervisor and trajectory shaper need each tick.
struct MissionTickInput {
    VehicleState vehicle{};
    SafetyDecision safety{};
    std::chrono::steady_clock::time_point now{};
    bool operator_shutdown{false};
};

struct MissionTickOutput {
    MissionContext next{};
    MissionCommand command{};
    bool state_changed{false};
    bool terminate{false};   // control loop should exit after sending command
};

} // namespace cfo
