#pragma once

#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/types.hpp"

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

// Closed set of events the logger knows how to write. Extended per slice as
// new event categories are added (state snapshots, safety decisions, ...).
using LogEvent = std::variant<
    LinkEvent,
    RawTelemetryEvent,
    RawCommandEvent,
    ConsoleEvent,
    LogBlockEvent>;

} // namespace cfo
