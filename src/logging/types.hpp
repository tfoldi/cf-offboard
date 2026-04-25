#pragma once

#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/types.hpp"
#include "mission/types.hpp"
#include "safety/types.hpp"

#include <chrono>
#include <cstdint>
#include <string>
#include <variant>

namespace cfo {

enum class LinkState : std::uint8_t {
    Opened,
    Closed,
    Error,
};

// Link state transition. `detail` carries the URI on Opened/Closed and a
// short human-readable reason on Error.
struct LinkEvent {
    LinkState state;
    std::string detail;
    std::chrono::system_clock::time_point t;
};

// Raw inbound CRTP packet captured at the link boundary, before any
// decoding. Logging raw bytes alongside decoded data (mandated by
// LOGGING.md) lets logs be reprocessed by improved decoders.
struct RawTelemetryEvent {
    RawPacket packet;
    std::chrono::system_clock::time_point t;
};

// Raw outbound CRTP packet handed to the link. Captures the host->vehicle
// side so command issuance is auditable in the same log.
struct RawCommandEvent {
    RawPacket packet;
    std::chrono::system_clock::time_point t;
};

// Decoded firmware console output (CRTP port 0). Stored alongside the raw
// packet so log readers do not have to re-decode.
struct ConsoleEvent {
    std::string text;
    std::chrono::system_clock::time_point t;
};

// Decoded sample from our single fixed log block (slice 2). Logged as one
// composite record per packet so analysis tools can reconstruct position,
// attitude, and battery on a common firmware timeline.
struct LogBlockEvent {
    LogBlockSample sample;
    std::chrono::system_clock::time_point t;  // host receive time
};

// Decoded outbound setpoint produced by the control loop. Mirrors the raw
// bytes (also logged) so analysis tools don't need to re-decode.
struct SetpointCommandEvent {
    enum class Kind : std::uint8_t { Stop, Hover };
    Kind  kind;
    float vx_mps;            // Hover only
    float vy_mps;            // Hover only
    float yaw_rate_dps;      // Hover only
    float z_target_m;        // Hover only
    std::chrono::system_clock::time_point t;
};

// Safety supervisor decision — emitted whenever the supervisor moves into
// an aborted state, so the abort is auditable in MCAP.
struct SafetyEvent {
    AbortReason reason;
    std::string detail;
    std::chrono::system_clock::time_point t;
};

// Snapshot of the firmware's supervisor state bitfield, captured during the
// pre-arm sanity check (and any future post-arm verification).
struct SupervisorStateEvent {
    SupervisorState state;
    std::chrono::system_clock::time_point t;
};

// Mission state machine transition. Emitted whenever mission_tick reports
// a state change. abort_reason/detail are populated only when entering
// MissionState::Aborted.
struct MissionStateEvent {
    MissionState state;
    AbortReason  abort_reason{AbortReason::None};
    std::string  abort_detail;
    std::chrono::system_clock::time_point t;
};

// Closed set of events the logger knows how to write. Extended per slice
// as new event categories are added.
using LogEvent = std::variant<
    LinkEvent,
    RawTelemetryEvent,
    RawCommandEvent,
    ConsoleEvent,
    LogBlockEvent,
    SetpointCommandEvent,
    SafetyEvent,
    SupervisorStateEvent,
    MissionStateEvent>;

} // namespace cfo
