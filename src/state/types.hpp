#pragma once

#include <chrono>
#include <cstdint>
#include <variant>

namespace cfo {

// ---------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------

struct Vec3 {
    float x{};
    float y{};
    float z{};
};

struct Attitude {
    float roll{};
    float pitch{};
    float yaw{};
};

enum class LinkStatus : std::uint8_t {
    Disconnected,
    Connected,
};

// ---------------------------------------------------------------------------
// Vehicle state — the single authoritative value type.
// Plain struct: copyable, default-constructible, no invariants beyond
// what the fields express. All updates flow through the pure `apply`
// functions declared below.
// ---------------------------------------------------------------------------

struct VehicleState {
    using clock = std::chrono::steady_clock;
    using time_point = clock::time_point;

    Vec3 position{};               // m, world frame
    Vec3 velocity{};               // m/s, world frame
    Attitude attitude{};           // rad
    Vec3 angular_rate{};           // rad/s, body frame

    float height_above_ground{};   // m, from flow/tof deck
    float battery_voltage{};       // V

    LinkStatus link{LinkStatus::Disconnected};

    time_point last_update{};      // timestamp of most recent telemetry applied
    std::uint64_t update_count{};  // monotonic counter, useful for change detection
};

// ---------------------------------------------------------------------------
// Telemetry messages — decoded packets from the Crazyflie.
// Each carries its own timestamp captured at decode time so that state
// updates remain replayable from a log.
// ---------------------------------------------------------------------------

struct PoseTelemetry {
    Vec3 position;
    Attitude attitude;
    VehicleState::time_point t;
};

struct VelocityTelemetry {
    Vec3 velocity; // world frame
    VehicleState::time_point t;
};

struct ImuTelemetry {
    Vec3 angular_rate; // body frame
    VehicleState::time_point t;
};

struct FlowTelemetry {
    float height_above_ground;
    VehicleState::time_point t;
};

struct BatteryTelemetry {
    float voltage;
    VehicleState::time_point t;
};

struct LinkTelemetry {
    LinkStatus status;
    VehicleState::time_point t;
};

using Telemetry = std::variant<
    PoseTelemetry,
    VelocityTelemetry,
    ImuTelemetry,
    FlowTelemetry,
    BatteryTelemetry,
    LinkTelemetry>;

// ---------------------------------------------------------------------------
// Pure state updates: produce next state from current state + a message.
// No IO, deterministic, replayable.
// ---------------------------------------------------------------------------

[[nodiscard]] VehicleState apply(VehicleState s, const PoseTelemetry& m);
[[nodiscard]] VehicleState apply(VehicleState s, const VelocityTelemetry& m);
[[nodiscard]] VehicleState apply(VehicleState s, const ImuTelemetry& m);
[[nodiscard]] VehicleState apply(VehicleState s, const FlowTelemetry& m);
[[nodiscard]] VehicleState apply(VehicleState s, const BatteryTelemetry& m);
[[nodiscard]] VehicleState apply(VehicleState s, const LinkTelemetry& m);

[[nodiscard]] VehicleState apply(VehicleState s, const Telemetry& m);

} // namespace cfo
