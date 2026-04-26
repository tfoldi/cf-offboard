#pragma once

#include "safety/types.hpp"
#include "state/types.hpp"

#include <chrono>
#include <cstdint>
#include <string>

namespace cfo {

// First-flight mission states. Linear progression
// Idle → TakingOff → HoverStabilizing → ForwardSegment → PreLandHover →
// HlcLanding → Completed. Aborted is the terminal failure branch entered
// from any non-terminal state on safety/operator abort.
//
// Landing is delegated to the firmware's high-level commander (HLC LAND
// on CRTP port 8). Slice (b)'s hover-setpoint Landing+Touchdown sequence
// dropped the last few cm because the flow deck can't track below ~3 cm
// and setpoint_stop was disarming with the drone still airborne; HLC
// LAND has firmware-side touchdown logic.
enum class MissionState : std::uint8_t {
    Idle,
    TakingOff,
    HoverStabilizing,
    ForwardSegment,
    PreLandHover,
    HlcLanding,
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
        case MissionState::HlcLanding:       return "HlcLanding";
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

    // HLC LAND parameters. The firmware-side high-level commander
    // executes the descent over `hlc_land_duration` and includes its own
    // touchdown logic; we just wait it out and then send setpoint_stop.
    float hlc_land_target_height_m{0.0f};
    std::chrono::milliseconds hlc_land_duration{2000};

    // After abort: ramp current target_z down to 0 over this window, then
    // emit the disarming stop. (Aborts use hover-setpoint streaming
    // because they may need to fire mid-flight before HLC is engaged.)
    std::chrono::milliseconds abort_descent_dwell{1000};

    // Hard wall-clock cap. Belt-and-braces against any logic bug that
    // would otherwise stream setpoints forever.
    std::chrono::milliseconds total_timeout{20'000};
};

// Outbound command produced by mission_tick. The caller (control loop)
// translates this into a CRTP packet (or no packet, for NoOp) and sends.
//
// The first-flight mission emits only the HLC kinds (Takeoff / GoTo /
// Land) plus Stop and NoOp — keeping a single coherent control authority
// from start to finish. Hover is preserved for future low-level paths
// (manual / keyboard / gamepad control).
struct MissionCommand {
    enum class Kind : std::uint8_t {
        Stop,         // setpoint_stop — low-level disarming (manual paths)
        Hover,        // hover_setpoint (low-level — not used by mission today)
        NoOp,         // skip this tick — HLC owns the trajectory
        HlcTakeoff,   // one-shot CRTP port-8 TAKEOFF
        HlcGoTo,      // one-shot CRTP port-8 GO_TO
        HlcLand,      // one-shot CRTP port-8 LAND
        HlcStop,      // one-shot CRTP port-8 STOP (HLC-side disarm)
    };
    Kind kind{Kind::NoOp};

    // Hover fields.
    float vx_mps{};
    float vy_mps{};
    float yaw_rate_dps{};
    float z_target_m{};

    // HLC TAKEOFF / LAND share these (height + duration).
    float hlc_height_m{};
    float hlc_duration_s{};

    // HLC GO_TO extras (duration shared with TAKEOFF/LAND above).
    float hlc_goto_x_m{};
    float hlc_goto_y_m{};
    float hlc_goto_z_m{};
    float hlc_goto_yaw_rad{};
    bool  hlc_goto_relative{true};
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
