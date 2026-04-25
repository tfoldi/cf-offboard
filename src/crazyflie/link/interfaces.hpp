#pragma once

#include "types.hpp"

#include <chrono>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace cfo {

// The single link to one Crazyflie. Implementations wrap a concrete radio
// transport (Crazyradio via crazyflie-link-cpp) and expose only the
// byte-level CRTP packet I/O — no decoding, no callbacks, no observer lists.
//
// Threading contract:
//   - send() and receive() may be called concurrently from different threads.
//   - receive() is intended for a single dedicated RX thread that calls it
//     in a loop with a small timeout (e.g. 50 ms) so the thread can observe
//     a stop signal between calls.
//   - Implementations may run their own internal threads to drive the radio,
//     but they never invoke callbacks into client code. All hand-off is via
//     the receive() return value.
//
// Receive loop contract (the canonical RX-thread shape):
//
//     while (!stop_requested) {
//         if (auto pkt = link.receive(50ms)) {
//             // hand off to decoder / state store
//         }
//     }
//
class ICrazyflieLink {
public:
    virtual ~ICrazyflieLink() = default;

    ICrazyflieLink(const ICrazyflieLink&) = delete;
    ICrazyflieLink& operator=(const ICrazyflieLink&) = delete;

    // Queue a packet for transmission. Returns LinkError::SendFailed if the
    // underlying transport rejected the packet or the link is closed.
    virtual std::expected<void, LinkError> send(const RawPacket& pkt) = 0;

    // Block up to `timeout` waiting for the next packet.
    // Returns std::nullopt if no packet arrived within the timeout.
    virtual std::optional<RawPacket> receive(
        std::chrono::milliseconds timeout) = 0;

protected:
    ICrazyflieLink() = default;
};

// Open a link to the Crazyflie at `uri`, e.g. "radio://0/80/2M/E7E7E7E7E7".
[[nodiscard]] std::expected<std::unique_ptr<ICrazyflieLink>, LinkError>
open_crazyflie_link(const std::string& uri);

// Scan attached Crazyradios for reachable Crazyflies. Returns URIs.
[[nodiscard]] std::vector<std::string> scan_crazyflie_links();

} // namespace cfo
