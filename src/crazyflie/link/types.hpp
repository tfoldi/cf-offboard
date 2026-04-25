#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace cfo {

// Maximum CRTP payload size. CRTP frames are 1 header byte + up to 31 payload
// bytes (the vendor library's CRTP_MAXSIZE is 32 including the header).
inline constexpr std::size_t kMaxCrtpPayload = 31;

// A CRTP packet at the link boundary — port, channel, and raw payload bytes.
// No further decoding. Fixed-size buffer keeps packets allocation-free on the
// RX hot path.
struct RawPacket {
    std::uint8_t port{};
    std::uint8_t channel{};
    std::uint8_t size{};                                    // payload bytes used
    std::array<std::uint8_t, kMaxCrtpPayload> payload{};
};

enum class LinkError : std::uint8_t {
    InvalidUri,
    NotConnected,
    SendFailed,
};

} // namespace cfo
