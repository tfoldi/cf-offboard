#include "interfaces.hpp"
#include "types.hpp"

#include <crazyflieLinkCpp/Connection.h>
#include <crazyflieLinkCpp/Packet.hpp>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <utility>

namespace cfo {

namespace {

namespace bz = bitcraze::crazyflieLinkCpp;

bz::Packet to_vendor(const RawPacket& pkt) {
    bz::Packet out{pkt.port, pkt.channel, pkt.size};
    if (pkt.size > 0) {
        out.setPayloadAt(0, pkt.payload.data(), pkt.size);
    }
    return out;
}

RawPacket from_vendor(const bz::Packet& vp) {
    RawPacket out{};
    out.port = vp.port();
    out.channel = vp.channel();
    const std::size_t n = std::min(vp.payloadSize(), kMaxCrtpPayload);
    out.size = static_cast<std::uint8_t>(n);
    std::copy_n(vp.payload(), n, out.payload.begin());
    return out;
}

class CrazyradioLink final : public ICrazyflieLink {
public:
    explicit CrazyradioLink(const std::string& uri) : conn_{uri} {}

    std::expected<void, LinkError> send(const RawPacket& pkt) override {
        try {
            conn_.send(to_vendor(pkt));
            return {};
        } catch (...) {
            return std::unexpected(LinkError::SendFailed);
        }
    }

    std::optional<RawPacket> receive(
        std::chrono::milliseconds timeout) override {
        // Clamp into the vendor's [TimeoutNone, TimeoutBlock) range so a
        // very large user timeout never accidentally triggers infinite block.
        constexpr auto kMaxMs = std::numeric_limits<unsigned int>::max() - 1u;
        const auto raw = timeout.count();
        const unsigned int ms =
            raw <= 0                                         ? 0u
            : (static_cast<std::uintmax_t>(raw) >= kMaxMs)   ? kMaxMs
                                                             : static_cast<unsigned int>(raw);
        bz::Packet vp = conn_.receive(ms);
        if (!vp.valid()) return std::nullopt;
        return from_vendor(vp);
    }

private:
    bz::Connection conn_;
};

} // namespace

std::expected<std::unique_ptr<ICrazyflieLink>, LinkError>
open_crazyflie_link(const std::string& uri) {
    if (uri.empty()) {
        return std::unexpected(LinkError::InvalidUri);
    }
    try {
        return std::make_unique<CrazyradioLink>(uri);
    } catch (...) {
        return std::unexpected(LinkError::NotConnected);
    }
}

std::vector<std::string> scan_crazyflie_links() {
    try {
        return bz::Connection::scan();
    } catch (...) {
        return {};
    }
}

} // namespace cfo
