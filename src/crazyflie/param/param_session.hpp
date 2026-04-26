#pragma once

#include "crazyflie/link/interfaces.hpp"
#include "crazyflie/link/types.hpp"

#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <string>
#include <string_view>

namespace cfo {

// Narrow PARAM support: just enough to write one named uint8_t parameter.
// Used by slice (d) to set `commander.enHighLevel = 1` so HLC LAND works.
// Not a generic parameter framework — adding more parameter operations
// (read, write of other types, etc.) is intentionally not supported here.

enum class ParamSetupError : std::uint8_t {
    SendFailed,
    Timeout,
    TocInfoFailed,
    TocItemFailed,
    VarNotFound,
    WrongType,
    WriteAckTimeout,
};

struct ParamSetupFailure {
    ParamSetupError code;
    std::string detail;
};

// Walk the PARAM TOC to find a named uint8_t variable, then write `value`
// to it and wait for the firmware's echo ack. Holds the receive line for
// the duration; the RX thread must not be running concurrently (same
// rule as setup_log_block / query_supervisor_state). Inbound packets that
// are not setup responses go to `passthrough` so MCAP stays complete.
std::expected<void, ParamSetupFailure>
write_uint8_param(ICrazyflieLink& link,
                  std::string_view group,
                  std::string_view name,
                  std::uint8_t value,
                  std::function<void(const RawPacket&)> passthrough,
                  std::chrono::milliseconds total_timeout = std::chrono::seconds{5});

} // namespace cfo
