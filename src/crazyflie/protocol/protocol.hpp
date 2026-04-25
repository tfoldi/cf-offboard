#pragma once

#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/types.hpp"

#include <chrono>
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

} // namespace cfo
