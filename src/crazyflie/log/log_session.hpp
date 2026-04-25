#pragma once

#include "crazyflie/link/interfaces.hpp"
#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/types.hpp"
#include "state/types.hpp"

#include <chrono>
#include <cstdint>
#include <expected>
#include <functional>
#include <numbers>
#include <string>

namespace cfo {

// Slice 2's single hardcoded log block. Fixed layout:
//
//   block_id    = 0
//   period      = 50 ms (20 Hz)
//   variables   = stateEstimate.x   (float32)
//                 stateEstimate.y   (float32)
//                 stateEstimate.z   (float32)
//                 pm.vbat           (FP16   — fits within the 30 B payload)
//                 stabilizer.roll   (float32)
//                 stabilizer.pitch  (float32)
//                 stabilizer.yaw    (float32)
//
// Decoder: protocol.hpp::decode_log_block_sample. Adding a second block
// would require its own decoder — this is deliberately not generic.

inline constexpr std::uint8_t kLogBlockId      = 0;
inline constexpr std::uint8_t kLogPeriod10ms   = 5;   // 50 ms
inline constexpr std::uint8_t kLogVarCount     = 7;

enum class LogSetupError : std::uint8_t {
    SendFailed,
    Timeout,
    TocInfoFailed,
    TocItemFailed,
    VarNotFound,
    BlockCreateFailed,
    BlockStartFailed,
};

// Where the failure occurred + a human-readable detail (the missing var
// name on VarNotFound, "during TOC walk" on Timeout, etc.). main.cpp logs
// `detail` verbatim.
struct LogSetupFailure {
    LogSetupError code;
    std::string detail;
};

struct LogResolved {
    std::uint16_t state_x_id{};
    std::uint16_t state_y_id{};
    std::uint16_t state_z_id{};
    std::uint16_t vbat_id{};
    std::uint16_t roll_id{};
    std::uint16_t pitch_id{};
    std::uint16_t yaw_id{};
};

// Progress milestones emitted during setup. Lets main.cpp print structured
// console lines without log_session knowing about console formatting.
struct LogSetupProgress {
    enum class Phase : std::uint8_t {
        ResetSent,
        TocInfoReceived,    // toc_info populated
        VarResolved,        // var_name set; resolved == count so far
        BlockCreated,
        BlockStarted,       // period_10ms set
    };
    Phase phase{};
    LogTocInfo toc_info{};
    std::string var_name{};
    std::uint8_t resolved{};
    std::uint8_t total{kLogVarCount};
    std::uint8_t period_10ms{};
};

using LogSetupNotifier = std::function<void(const LogSetupProgress&)>;

// Synchronously bring up the log subsystem (reset → TOC walk → create →
// start). Holds the receive line for the duration; the RX thread must not
// be running concurrently. Inbound packets that are not setup responses
// are forwarded to `passthrough` so MCAP stays complete.
std::expected<LogResolved, LogSetupFailure>
setup_log_block(ICrazyflieLink& link,
                std::function<void(const RawPacket&)> passthrough,
                LogSetupNotifier on_progress = nullptr,
                std::chrono::milliseconds total_timeout = std::chrono::seconds{5});

// One-shot supervisor state query. Sends CMD_GET_STATE_BITFIELD on the
// info channel and returns the decoded SupervisorState. Same threading
// constraint as setup_log_block — caller must own the receive line.
std::expected<SupervisorState, LogSetupFailure>
query_supervisor_state(ICrazyflieLink& link,
                       std::function<void(const RawPacket&)> passthrough,
                       std::chrono::milliseconds timeout = std::chrono::milliseconds{500});

// ---------------------------------------------------------------------------
// LogBlockSample → Telemetry adapters. Pure functions; tested directly.
// Stabilizer angles arrive in degrees; VehicleState::Attitude is in radians.
// ---------------------------------------------------------------------------

inline constexpr float deg_to_rad(float d) noexcept {
    return d * static_cast<float>(std::numbers::pi) / 180.0f;
}

[[nodiscard]] inline PoseTelemetry
pose_from_log_sample(const LogBlockSample& s,
                     VehicleState::time_point t) noexcept {
    return PoseTelemetry{
        Vec3{s.x, s.y, s.z},
        Attitude{deg_to_rad(s.roll), deg_to_rad(s.pitch), deg_to_rad(s.yaw)},
        t,
    };
}

[[nodiscard]] inline BatteryTelemetry
battery_from_log_sample(const LogBlockSample& s,
                        VehicleState::time_point t) noexcept {
    return BatteryTelemetry{s.vbat, t};
}

} // namespace cfo
