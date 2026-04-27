#pragma once

#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/types.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <span>
#include <string>

namespace cfo {

// ---------------------------------------------------------------------------
// Classification
// ---------------------------------------------------------------------------

[[nodiscard]] inline PacketKind classify(const RawPacket& pkt) noexcept {
    if (pkt.port == crtp::kPortConsole &&
        pkt.channel == crtp::kChannelConsole) {
        return PacketKind::Console;
    }
    if (pkt.port == crtp::kPortLog) {
        switch (pkt.channel) {
            case crtp::kChannelLogToc:      return PacketKind::LogTocReply;
            case crtp::kChannelLogSettings: return PacketKind::LogSettingsAck;
            case crtp::kChannelLogData:     return PacketKind::LogData;
            default:                         return PacketKind::Unknown;
        }
    }
    if (pkt.port == crtp::kPortLinkControl &&
        pkt.channel == crtp::kChannelLinkControl) {
        return PacketKind::LinkControl;
    }
    return PacketKind::Unknown;
}

// ---------------------------------------------------------------------------
// Console decoder
// ---------------------------------------------------------------------------

[[nodiscard]] inline std::expected<ConsoleMessage, DecodeError>
decode_console(const RawPacket& pkt,
               std::chrono::system_clock::time_point t) {
    if (pkt.port != crtp::kPortConsole ||
        pkt.channel != crtp::kChannelConsole) {
        return std::unexpected(DecodeError::WrongPort);
    }
    return ConsoleMessage{
        std::string{reinterpret_cast<const char*>(pkt.payload.data()), pkt.size},
        t,
    };
}

// ---------------------------------------------------------------------------
// FP16 → float32 (IEEE 754 half precision).
// We only need the decode direction; the firmware never asks us to encode.
// ---------------------------------------------------------------------------

[[nodiscard]] inline float fp16_to_float(std::uint16_t h) noexcept {
    const std::uint32_t sign = (h >> 15) & 0x1u;
    const std::uint32_t exp  = (h >> 10) & 0x1Fu;
    const std::uint32_t mant = h & 0x3FFu;

    std::uint32_t f = sign << 31;
    if (exp == 0) {
        if (mant != 0) {
            // Subnormal: normalize.
            std::uint32_t e = 127 - 15 + 1;
            std::uint32_t m = mant;
            while ((m & 0x400u) == 0) {
                m <<= 1;
                --e;
            }
            f |= (e << 23) | ((m & 0x3FFu) << 13);
        }
        // else +/-0 — sign already set.
    } else if (exp == 0x1F) {
        // Inf or NaN: max exponent + mantissa bits preserved.
        f |= (0xFFu << 23) | (mant << 13);
    } else {
        // Normal.
        f |= ((exp - 15 + 127) << 23) | (mant << 13);
    }

    float result;
    std::memcpy(&result, &f, sizeof(result));
    return result;
}

// ---------------------------------------------------------------------------
// Outbound packet builders
// ---------------------------------------------------------------------------

[[nodiscard]] inline RawPacket make_null_packet() noexcept {
    return RawPacket{
        crtp::kPortLinkControl,
        crtp::kChannelLinkControl,
        /* size  */ 0,
        /* bytes */ {},
    };
}

// CMD_RESET_LOGGING — clears any log blocks left behind from a previous
// session. Sent on settings channel; firmware acks with [cmd, error_code].
[[nodiscard]] inline RawPacket make_log_reset() noexcept {
    RawPacket p{};
    p.port    = crtp::kPortLog;
    p.channel = crtp::kChannelLogSettings;
    p.size    = 1;
    p.payload[0] = crtp::kCmdLogResetLogging;
    return p;
}

// CMD_TOC_INFO_V2 request — returns count and CRC.
[[nodiscard]] inline RawPacket make_log_toc_info_v2_request() noexcept {
    RawPacket p{};
    p.port    = crtp::kPortLog;
    p.channel = crtp::kChannelLogToc;
    p.size    = 1;
    p.payload[0] = crtp::kCmdTocInfoV2;
    return p;
}

// CMD_TOC_ITEM_V2 request — returns the item at `index`.
[[nodiscard]] inline RawPacket
make_log_toc_item_v2_request(std::uint16_t index) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortLog;
    p.channel = crtp::kChannelLogToc;
    p.size    = 3;
    p.payload[0] = crtp::kCmdTocItemV2;
    p.payload[1] = static_cast<std::uint8_t>(index & 0xFF);
    p.payload[2] = static_cast<std::uint8_t>((index >> 8) & 0xFF);
    return p;
}

// CMD_CREATE_BLOCK_V2 — define a log block. Each var contributes 3 bytes
// (type_byte + uint16 var_id LE). Caller is responsible for keeping the
// total under the CRTP payload cap.
[[nodiscard]] inline RawPacket
make_log_create_block_v2(std::uint8_t block_id,
                         std::span<const LogVarSpec> vars) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortLog;
    p.channel = crtp::kChannelLogSettings;
    p.payload[0] = crtp::kCmdLogCreateBlockV2;
    p.payload[1] = block_id;
    std::size_t off = 2;
    for (const auto& v : vars) {
        p.payload[off++] = v.type_byte;
        p.payload[off++] = static_cast<std::uint8_t>(v.var_id & 0xFF);
        p.payload[off++] = static_cast<std::uint8_t>((v.var_id >> 8) & 0xFF);
    }
    p.size = static_cast<std::uint8_t>(off);
    return p;
}

// CMD_START_LOGGING — start a previously-created block. `period_10ms` is
// the inter-sample interval in 10 ms ticks (cflib semantics, e.g. 5 → 50 ms).
[[nodiscard]] inline RawPacket
make_log_start_block(std::uint8_t block_id,
                     std::uint8_t period_10ms) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortLog;
    p.channel = crtp::kChannelLogSettings;
    p.size    = 3;
    p.payload[0] = crtp::kCmdLogStartLogging;
    p.payload[1] = block_id;
    p.payload[2] = period_10ms;
    return p;
}

// Modern arm/disarm request (CRTP protocol v12+). Required on recent
// firmware: motors will not spin without an explicit arm, and stop_setpoint
// disarms — so a second consecutive run silently drops setpoints unless we
// re-arm first. cflib's `Crazyflie.platform.send_arming_request` forwards
// to the supervisor service on v12+ firmware.
[[nodiscard]] inline RawPacket make_arm_request(bool arm) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortSupervisor;
    p.channel = crtp::kChannelSupervisorCommand;
    p.size    = 2;
    p.payload[0] = crtp::kCmdSupervisorArm;
    p.payload[1] = arm ? std::uint8_t{1} : std::uint8_t{0};
    return p;
}

// Legacy arm/disarm request for pre-v12 firmware. Same payload shape, sent
// to the PLATFORM port instead of SUPERVISOR. cflib's supervisor service
// falls back to this when the firmware reports an older protocol version.
// We send both modern and legacy packets at startup so we don't need to
// negotiate the protocol version up front.
[[nodiscard]] inline RawPacket make_arm_request_legacy(bool arm) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortPlatform;
    p.channel = crtp::kChannelPlatformCommand;
    p.size    = 2;
    p.payload[0] = crtp::kCmdPlatformRequestArm;
    p.payload[1] = arm ? std::uint8_t{1} : std::uint8_t{0};
    return p;
}

// Clear the supervisor's locked/crashed flags so subsequent arm requests
// take effect. After a controlled landing the supervisor commonly latches
// into "locked"; a power-cycle clears it, and so does this command.
// Mirrors cflib's `Crazyflie.supervisor.send_crash_recovery_request()`.
[[nodiscard]] inline RawPacket make_supervisor_recover() noexcept {
    RawPacket p{};
    p.port    = crtp::kPortSupervisor;
    p.channel = crtp::kChannelSupervisorCommand;
    p.size    = 1;
    p.payload[0] = crtp::kCmdSupervisorRecover;
    return p;
}

// Ask the supervisor for its current state bitfield (CMD_GET_STATE_BITFIELD).
// The reply arrives on the same port/channel with the response flag (0x80)
// OR'd onto the command byte and the bitfield bytes following.
[[nodiscard]] inline RawPacket make_supervisor_info_request() noexcept {
    RawPacket p{};
    p.port    = crtp::kPortSupervisor;
    p.channel = crtp::kChannelSupervisorInfo;
    p.size    = 1;
    p.payload[0] = crtp::kCmdSupervisorGetStateBitfield;
    return p;
}

// Decode a supervisor state response (port 9 / ch 0, payload[0] == 0x8C,
// followed by little-endian bitfield bytes). Returns one bool per known
// state bit.
[[nodiscard]] inline std::expected<SupervisorState, DecodeError>
decode_supervisor_state(const RawPacket& pkt) {
    if (pkt.port != crtp::kPortSupervisor ||
        pkt.channel != crtp::kChannelSupervisorInfo) {
        return std::unexpected(DecodeError::WrongPort);
    }
    if (pkt.size < 2) return std::unexpected(DecodeError::Truncated);
    const std::uint8_t expected =
        crtp::kCmdSupervisorGetStateBitfield | crtp::kCmdResponseFlag;
    if (pkt.payload[0] != expected) {
        return std::unexpected(DecodeError::UnexpectedCmd);
    }

    // Read up to 4 bytes of bitfield (11 bits comfortably fit in 16, but
    // firmware versions vary — accept whatever's there, capped).
    std::uint32_t bits = 0;
    const std::size_t n = std::min<std::size_t>(pkt.size - 1, 4);
    for (std::size_t i = 0; i < n; ++i) {
        bits |= static_cast<std::uint32_t>(pkt.payload[1 + i]) << (8 * i);
    }
    auto bit = [&](std::uint8_t pos) {
        return ((bits >> pos) & 1u) != 0;
    };
    SupervisorState s{};
    s.can_be_armed         = bit(0);
    s.is_armed             = bit(1);
    s.is_auto_armed        = bit(2);
    s.can_fly              = bit(3);
    s.is_flying            = bit(4);
    s.is_tumbled           = bit(5);
    s.is_locked            = bit(6);
    s.is_crashed           = bit(7);
    s.hl_control_active    = bit(8);
    s.hl_traj_finished     = bit(9);
    s.hl_control_disabled  = bit(10);
    return s;
}

// Generic-commander STOP setpoint (type 0). Type byte only; firmware
// disarms motors on receipt. SAFE TO SEND ONLY ON THE GROUND. While
// airborne this drops the vehicle.
[[nodiscard]] inline RawPacket make_setpoint_stop() noexcept {
    RawPacket p{};
    p.port    = crtp::kPortGenericSetpoint;
    p.channel = crtp::kChannelGenericSetpoint;
    p.size    = 1;
    p.payload[0] = crtp::kSetpointStop;
    return p;
}

// Generic-commander NOTIFY_SETPOINT_STOP (type 0xFF). Wire layout:
//   [type:1=0xFF] [remain_valid_ms:u32 LE]
// Tells the firmware "low-level setpoint streaming has ended, hand off
// to the high-level commander after `remain_valid_ms` of grace". Without
// this packet, the low-level commander stays priority-active and HLC
// LAND is silently ignored. cflib calls this `send_notify_setpoint_stop`.
[[nodiscard]] inline RawPacket
make_setpoint_notify_stop(std::uint32_t remain_valid_ms = 0) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortGenericSetpoint;
    p.channel = crtp::kChannelGenericSetpoint;
    p.size    = 5;
    p.payload[0] = crtp::kSetpointNotifyStop;
    p.payload[1] = static_cast<std::uint8_t>(remain_valid_ms & 0xFF);
    p.payload[2] = static_cast<std::uint8_t>((remain_valid_ms >> 8)  & 0xFF);
    p.payload[3] = static_cast<std::uint8_t>((remain_valid_ms >> 16) & 0xFF);
    p.payload[4] = static_cast<std::uint8_t>((remain_valid_ms >> 24) & 0xFF);
    return p;
}

// Generic-commander HOVER setpoint (type 5). Wire layout:
//   [type:1] [vx:f32 LE] [vy:f32 LE] [yaw_rate:f32 LE] [z_distance:f32 LE]
// Body-frame velocity (m/s), yaw rate (deg/s), absolute target height
// above takeoff (m). Total payload = 1 + 16 = 17 bytes. The firmware
// expects this to be streamed at >= ~10 Hz; gaps cause the commander to
// time out and stop accepting setpoints.
[[nodiscard]] inline RawPacket
make_setpoint_hover(float vx_mps, float vy_mps,
                    float yaw_rate_dps, float z_target_m) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortGenericSetpoint;
    p.channel = crtp::kChannelGenericSetpoint;
    p.size    = 17;
    p.payload[0] = crtp::kSetpointHover;
    auto write_f32 = [&](std::size_t off, float v) {
        std::uint32_t bits;
        std::memcpy(&bits, &v, 4);
        p.payload[off + 0] = static_cast<std::uint8_t>(bits & 0xFF);
        p.payload[off + 1] = static_cast<std::uint8_t>((bits >> 8) & 0xFF);
        p.payload[off + 2] = static_cast<std::uint8_t>((bits >> 16) & 0xFF);
        p.payload[off + 3] = static_cast<std::uint8_t>((bits >> 24) & 0xFF);
    };
    write_f32(1,  vx_mps);
    write_f32(5,  vy_mps);
    write_f32(9,  yaw_rate_dps);
    write_f32(13, z_target_m);
    return p;
}

// ---------------------------------------------------------------------------
// LOG TOC decoders (for setup-time use)
// ---------------------------------------------------------------------------

// CMD_TOC_INFO_V2 response: [cmd:1] [count:2 LE] [crc:4 LE] [+ extra bytes].
// We read only the first 7 bytes; later fields (max_packet, max_blocks) are
// not used by this slice.
[[nodiscard]] inline std::expected<LogTocInfo, DecodeError>
decode_log_toc_info_v2(const RawPacket& pkt) {
    if (pkt.port != crtp::kPortLog ||
        pkt.channel != crtp::kChannelLogToc) {
        return std::unexpected(DecodeError::WrongPort);
    }
    if (pkt.size < 7) return std::unexpected(DecodeError::Truncated);
    if (pkt.payload[0] != crtp::kCmdTocInfoV2) {
        return std::unexpected(DecodeError::UnexpectedCmd);
    }
    LogTocInfo info{};
    info.count = static_cast<std::uint16_t>(
        pkt.payload[1] | (pkt.payload[2] << 8));
    info.crc =
        static_cast<std::uint32_t>(pkt.payload[3]) |
        (static_cast<std::uint32_t>(pkt.payload[4]) << 8) |
        (static_cast<std::uint32_t>(pkt.payload[5]) << 16) |
        (static_cast<std::uint32_t>(pkt.payload[6]) << 24);
    return info;
}

// CMD_TOC_ITEM_V2 response:
//   [cmd:1] [index:2 LE] [type:1] [group\0] [name\0]
// The type byte's low nibble is the variable's storage type code; bit 4
// indicates read/write access (cflib uses it but we do not).
[[nodiscard]] inline std::expected<LogTocItem, DecodeError>
decode_log_toc_item_v2(const RawPacket& pkt) {
    if (pkt.port != crtp::kPortLog ||
        pkt.channel != crtp::kChannelLogToc) {
        return std::unexpected(DecodeError::WrongPort);
    }
    if (pkt.size < 4) return std::unexpected(DecodeError::Truncated);
    if (pkt.payload[0] != crtp::kCmdTocItemV2) {
        return std::unexpected(DecodeError::UnexpectedCmd);
    }
    LogTocItem item{};
    item.index = static_cast<std::uint16_t>(
        pkt.payload[1] | (pkt.payload[2] << 8));
    item.type_code = pkt.payload[3] & 0x0Fu;

    // Two consecutive null-terminated strings.
    std::size_t i = 4;
    auto read_cstr = [&](std::string& out) -> bool {
        const std::size_t start = i;
        while (i < pkt.size && pkt.payload[i] != 0) ++i;
        if (i >= pkt.size) return false;  // no terminator
        out.assign(reinterpret_cast<const char*>(&pkt.payload[start]),
                   i - start);
        ++i;  // skip the NUL
        return true;
    };
    if (!read_cstr(item.group)) return std::unexpected(DecodeError::Truncated);
    if (!read_cstr(item.name))  return std::unexpected(DecodeError::Truncated);
    return item;
}

// CMD_CREATE_BLOCK_V2 / CMD_START_LOGGING / CMD_STOP_LOGGING / RESET ack:
//   [cmd:1] [block_id:1] [error:1]   (block_id absent for RESET)
// We only decode the form with block_id since that's what create/start emit.
[[nodiscard]] inline std::expected<LogSettingsAck, DecodeError>
decode_log_settings_ack(const RawPacket& pkt) {
    if (pkt.port != crtp::kPortLog ||
        pkt.channel != crtp::kChannelLogSettings) {
        return std::unexpected(DecodeError::WrongPort);
    }
    if (pkt.size < 3) return std::unexpected(DecodeError::Truncated);
    return LogSettingsAck{pkt.payload[0], pkt.payload[1], pkt.payload[2]};
}

// ---------------------------------------------------------------------------
// PARAM TOC + WRITE — minimal support for setting a single named uint8_t
// parameter. Same TOC command codes as LOG; element layout differs.
// ---------------------------------------------------------------------------

[[nodiscard]] inline RawPacket make_param_toc_info_v2_request() noexcept {
    RawPacket p{};
    p.port    = crtp::kPortParam;
    p.channel = crtp::kChannelParamToc;
    p.size    = 1;
    p.payload[0] = crtp::kCmdTocInfoV2;
    return p;
}

[[nodiscard]] inline RawPacket
make_param_toc_item_v2_request(std::uint16_t index) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortParam;
    p.channel = crtp::kChannelParamToc;
    p.size    = 3;
    p.payload[0] = crtp::kCmdTocItemV2;
    p.payload[1] = static_cast<std::uint8_t>(index & 0xFF);
    p.payload[2] = static_cast<std::uint8_t>((index >> 8) & 0xFF);
    return p;
}

// PARAM_WRITE_V2 for a uint8_t value. Wire layout:
//   [var_id_lo] [var_id_hi] [value:1]
// The firmware echoes the var_id back on the same channel as ack.
[[nodiscard]] inline RawPacket
make_param_write_uint8(std::uint16_t var_id, std::uint8_t value) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortParam;
    p.channel = crtp::kChannelParamWrite;
    p.size    = 3;
    p.payload[0] = static_cast<std::uint8_t>(var_id & 0xFF);
    p.payload[1] = static_cast<std::uint8_t>((var_id >> 8) & 0xFF);
    p.payload[2] = value;
    return p;
}

// CMD_TOC_INFO_V2 reply on the PARAM port — same shape as LOG version.
[[nodiscard]] inline std::expected<ParamTocInfo, DecodeError>
decode_param_toc_info_v2(const RawPacket& pkt) {
    if (pkt.port != crtp::kPortParam ||
        pkt.channel != crtp::kChannelParamToc) {
        return std::unexpected(DecodeError::WrongPort);
    }
    if (pkt.size < 7) return std::unexpected(DecodeError::Truncated);
    if (pkt.payload[0] != crtp::kCmdTocInfoV2) {
        return std::unexpected(DecodeError::UnexpectedCmd);
    }
    ParamTocInfo info{};
    info.count = static_cast<std::uint16_t>(
        pkt.payload[1] | (pkt.payload[2] << 8));
    info.crc =
        static_cast<std::uint32_t>(pkt.payload[3]) |
        (static_cast<std::uint32_t>(pkt.payload[4]) << 8) |
        (static_cast<std::uint32_t>(pkt.payload[5]) << 16) |
        (static_cast<std::uint32_t>(pkt.payload[6]) << 24);
    return info;
}

// CMD_TOC_ITEM_V2 reply on the PARAM port. Layout:
//   [cmd:1] [index:2 LE] [metadata:1] [group\0] [name\0]
// Metadata's low nibble is the type, bit 4 is extended, bit 6 is read-only.
[[nodiscard]] inline std::expected<ParamTocItem, DecodeError>
decode_param_toc_item_v2(const RawPacket& pkt) {
    if (pkt.port != crtp::kPortParam ||
        pkt.channel != crtp::kChannelParamToc) {
        return std::unexpected(DecodeError::WrongPort);
    }
    if (pkt.size < 4) return std::unexpected(DecodeError::Truncated);
    if (pkt.payload[0] != crtp::kCmdTocItemV2) {
        return std::unexpected(DecodeError::UnexpectedCmd);
    }
    ParamTocItem item{};
    item.index = static_cast<std::uint16_t>(
        pkt.payload[1] | (pkt.payload[2] << 8));
    const std::uint8_t metadata = pkt.payload[3];
    item.type_code = metadata & crtp::kParamTypeMaskCore;
    item.read_only = (metadata & crtp::kParamTypeFlagReadonly) != 0;

    std::size_t i = 4;
    auto read_cstr = [&](std::string& out) -> bool {
        const std::size_t start = i;
        while (i < pkt.size && pkt.payload[i] != 0) ++i;
        if (i >= pkt.size) return false;
        out.assign(reinterpret_cast<const char*>(&pkt.payload[start]),
                   i - start);
        ++i;
        return true;
    };
    if (!read_cstr(item.group)) return std::unexpected(DecodeError::Truncated);
    if (!read_cstr(item.name))  return std::unexpected(DecodeError::Truncated);
    return item;
}

// PARAM_WRITE response: the firmware echoes back the var_id on the WRITE
// channel. We treat reception as success (cflib does the same — there's
// no distinct "rejected" reply for V2 writes).
[[nodiscard]] inline bool
is_param_write_ack(const RawPacket& pkt, std::uint16_t expected_var_id) noexcept {
    if (pkt.port != crtp::kPortParam ||
        pkt.channel != crtp::kChannelParamWrite ||
        pkt.size < 2) {
        return false;
    }
    const std::uint16_t got = static_cast<std::uint16_t>(
        pkt.payload[0] | (pkt.payload[1] << 8));
    return got == expected_var_id;
}

// ---------------------------------------------------------------------------
// High-level commander LAND. Wire format:
//   [opcode:1=0x08] [group_mask:1] [height:f32 LE] [yaw:f32 LE]
//   [use_current_yaw:1] [duration:f32 LE]
// Total: 15 bytes.
// ---------------------------------------------------------------------------

// Internal helper — write a little-endian float at the given offset of a
// RawPacket payload. Used by the HLC builders below.
inline void hlc_write_f32(RawPacket& p, std::size_t off, float v) noexcept {
    std::uint32_t bits;
    std::memcpy(&bits, &v, 4);
    p.payload[off + 0] = static_cast<std::uint8_t>(bits & 0xFF);
    p.payload[off + 1] = static_cast<std::uint8_t>((bits >> 8) & 0xFF);
    p.payload[off + 2] = static_cast<std::uint8_t>((bits >> 16) & 0xFF);
    p.payload[off + 3] = static_cast<std::uint8_t>((bits >> 24) & 0xFF);
}

[[nodiscard]] inline RawPacket
make_hlc_land(float height_m, float duration_s,
              bool use_current_yaw = true,
              float yaw_rad = 0.0f,
              std::uint8_t group_mask = 0) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortHighLevelCmd;
    p.channel = crtp::kChannelHighLevelCmd;
    p.size    = 15;
    p.payload[0] = crtp::kCmdHlcLand2;
    p.payload[1] = group_mask;
    hlc_write_f32(p, 2,  height_m);
    hlc_write_f32(p, 6,  yaw_rad);
    p.payload[10] = use_current_yaw ? std::uint8_t{1} : std::uint8_t{0};
    hlc_write_f32(p, 11, duration_s);
    return p;
}

// HLC STOP (cflib COMMAND_STOP). 2-byte payload:
//   [opcode:1=0x03] [group_mask:1]
// Cuts the high-level commander cleanly. cflib uses this — not
// setpoint_stop — at end of HLC missions. Mixing low-level disarm with
// HLC trajectories is what triggers the firmware's `is_locked` latch.
[[nodiscard]] inline RawPacket
make_hlc_stop(std::uint8_t group_mask = 0) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortHighLevelCmd;
    p.channel = crtp::kChannelHighLevelCmd;
    p.size    = 2;
    p.payload[0] = crtp::kCmdHlcStop;
    p.payload[1] = group_mask;
    return p;
}

// HLC TAKEOFF (cflib COMMAND_TAKEOFF_2). Same 15-byte layout as LAND, only
// the opcode differs. Drone climbs from current position to absolute
// `height_m` over `duration_s`, then HLC holds the new position.
[[nodiscard]] inline RawPacket
make_hlc_takeoff(float height_m, float duration_s,
                 bool use_current_yaw = true,
                 float yaw_rad = 0.0f,
                 std::uint8_t group_mask = 0) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortHighLevelCmd;
    p.channel = crtp::kChannelHighLevelCmd;
    p.size    = 15;
    p.payload[0] = crtp::kCmdHlcTakeoff2;
    p.payload[1] = group_mask;
    hlc_write_f32(p, 2,  height_m);
    hlc_write_f32(p, 6,  yaw_rad);
    p.payload[10] = use_current_yaw ? std::uint8_t{1} : std::uint8_t{0};
    hlc_write_f32(p, 11, duration_s);
    return p;
}

// HLC GO_TO (cflib COMMAND_GO_TO_2). 24-byte payload:
//   [opcode:1=0x0C] [group_mask:1] [relative:1] [linear:1]
//   [x:f32] [y:f32] [z:f32] [yaw:f32] [duration:f32]
// `relative=true` makes (x, y, z, yaw) deltas from the current state;
// `relative=false` treats them as absolute targets. `linear=false` lets
// the firmware planner generate a smooth polynomial trajectory; setting
// it true skips the planner and drives a straight-line motion.
[[nodiscard]] inline RawPacket
make_hlc_go_to(float x, float y, float z, float yaw_rad, float duration_s,
               bool relative = true, bool linear = false,
               std::uint8_t group_mask = 0) noexcept {
    RawPacket p{};
    p.port    = crtp::kPortHighLevelCmd;
    p.channel = crtp::kChannelHighLevelCmd;
    p.size    = 24;
    p.payload[0] = crtp::kCmdHlcGoTo2;
    p.payload[1] = group_mask;
    p.payload[2] = relative ? std::uint8_t{1} : std::uint8_t{0};
    p.payload[3] = linear   ? std::uint8_t{1} : std::uint8_t{0};
    hlc_write_f32(p, 4,  x);
    hlc_write_f32(p, 8,  y);
    hlc_write_f32(p, 12, z);
    hlc_write_f32(p, 16, yaw_rad);
    hlc_write_f32(p, 20, duration_s);
    return p;
}

// ---------------------------------------------------------------------------
// LOG DATA decoder for *our* fixed block layout (slice 2).
//
// Wire format for our block (block_id=0):
//   [block_id:1] [timestamp:3 LE] [x:f32] [y:f32] [z:f32]
//                                 [vbat:f16] [roll:f32] [pitch:f32] [yaw:f32]
// Total: 1 + 3 + 24 + 2 = 30 bytes (exactly at the firmware payload cap).
//
// Hardcoded, not generic: this decoder belongs to *this* block. If we ever
// add a second block, it gets its own decoder.
// ---------------------------------------------------------------------------

inline constexpr std::size_t kLogBlockPayloadSize = 30;
inline constexpr std::size_t kRangeBlockPayloadSize = 14;

[[nodiscard]] inline std::expected<LogBlockSample, DecodeError>
decode_log_block_sample(const RawPacket& pkt, std::uint8_t expected_block_id) {
    if (pkt.port != crtp::kPortLog ||
        pkt.channel != crtp::kChannelLogData) {
        return std::unexpected(DecodeError::WrongPort);
    }
    if (pkt.size != kLogBlockPayloadSize) {
        return std::unexpected(DecodeError::WrongSize);
    }
    if (pkt.payload[0] != expected_block_id) {
        return std::unexpected(DecodeError::WrongBlockId);
    }

    LogBlockSample s{};
    s.timestamp_ms =
        static_cast<std::uint32_t>(pkt.payload[1]) |
        (static_cast<std::uint32_t>(pkt.payload[2]) << 8) |
        (static_cast<std::uint32_t>(pkt.payload[3]) << 16);

    auto read_f32 = [&](std::size_t off) {
        float v;
        std::memcpy(&v, &pkt.payload[off], 4);
        return v;
    };
    auto read_f16 = [&](std::size_t off) {
        std::uint16_t h = static_cast<std::uint16_t>(
            pkt.payload[off] | (pkt.payload[off + 1] << 8));
        return fp16_to_float(h);
    };

    s.x     = read_f32(4);
    s.y     = read_f32(8);
    s.z     = read_f32(12);
    s.vbat  = read_f16(16);
    s.roll  = read_f32(18);
    s.pitch = read_f32(22);
    s.yaw   = read_f32(26);
    return s;
}

// Multiranger range block decoder.
[[nodiscard]] inline std::expected<RangeBlockSample, DecodeError>
decode_range_block_sample(const RawPacket& pkt, std::uint8_t expected_block_id) {
    if (pkt.port != crtp::kPortLog ||
        pkt.channel != crtp::kChannelLogData) {
        return std::unexpected(DecodeError::WrongPort);
    }
    if (pkt.size != kRangeBlockPayloadSize) {
        return std::unexpected(DecodeError::WrongSize);
    }
    if (pkt.payload[0] != expected_block_id) {
        return std::unexpected(DecodeError::WrongBlockId);
    }
    RangeBlockSample s{};
    s.timestamp_ms =
        static_cast<std::uint32_t>(pkt.payload[1]) |
        (static_cast<std::uint32_t>(pkt.payload[2]) << 8) |
        (static_cast<std::uint32_t>(pkt.payload[3]) << 16);
    auto read_u16 = [&](std::size_t off) -> std::uint16_t {
        return static_cast<std::uint16_t>(
            pkt.payload[off] | (pkt.payload[off + 1] << 8));
    };
    s.front_mm = read_u16(4);
    s.back_mm  = read_u16(6);
    s.left_mm  = read_u16(8);
    s.right_mm = read_u16(10);
    s.up_mm    = read_u16(12);
    return s;
}

} // namespace cfo
