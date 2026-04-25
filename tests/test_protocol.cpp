#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/protocol.hpp"
#include "crazyflie/protocol/types.hpp"

#include <doctest/doctest.h>

#include <chrono>
#include <cstdint>
#include <string>

namespace {

cfo::RawPacket pkt_with_payload(std::uint8_t port, std::uint8_t channel,
                                std::initializer_list<std::uint8_t> bytes) {
    cfo::RawPacket p{};
    p.port = port;
    p.channel = channel;
    p.size = static_cast<std::uint8_t>(bytes.size());
    std::size_t i = 0;
    for (auto b : bytes) p.payload[i++] = b;
    return p;
}

constexpr auto t_zero = std::chrono::system_clock::time_point{};

} // namespace

TEST_CASE("classify maps known port/channel pairs") {
    SUBCASE("console packet (port 0, ch 0)") {
        const auto p = pkt_with_payload(0, 0, {'h', 'i'});
        CHECK(cfo::classify(p) == cfo::PacketKind::Console);
    }
    SUBCASE("link-control (port 15, ch 3)") {
        // Per live capture this channel is not always empty, hence the rename
        // away from "LinkNull". Classification depends only on (port, channel).
        const auto p = pkt_with_payload(15, 3, {0x05, 0x01});
        CHECK(cfo::classify(p) == cfo::PacketKind::LinkControl);
    }
    SUBCASE("log data (port 5, ch 2)") {
        const auto p = pkt_with_payload(5, 2, {});
        CHECK(cfo::classify(p) == cfo::PacketKind::LogData);
    }
    SUBCASE("log toc reply (port 5, ch 0)") {
        const auto p = pkt_with_payload(5, 0, {});
        CHECK(cfo::classify(p) == cfo::PacketKind::LogTocReply);
    }
    SUBCASE("log settings ack (port 5, ch 1)") {
        const auto p = pkt_with_payload(5, 1, {});
        CHECK(cfo::classify(p) == cfo::PacketKind::LogSettingsAck);
    }
    SUBCASE("garbage port/channel is Unknown") {
        const auto p = pkt_with_payload(99, 99, {});
        CHECK(cfo::classify(p) == cfo::PacketKind::Unknown);
    }
    SUBCASE("right port, wrong channel is Unknown") {
        const auto p = pkt_with_payload(0, 1, {});
        CHECK(cfo::classify(p) == cfo::PacketKind::Unknown);
    }
}

TEST_CASE("decode_console yields the payload as text") {
    SUBCASE("ASCII text") {
        const auto p = pkt_with_payload(0, 0,
            {'H', 'e', 'l', 'l', 'o', ' ', 'C', 'F'});
        auto r = cfo::decode_console(p, t_zero);
        REQUIRE(r);
        CHECK(r->text == "Hello CF");
        CHECK(r->t == t_zero);
    }
    SUBCASE("empty payload yields empty string (legal — fragments may be empty)") {
        const auto p = pkt_with_payload(0, 0, {});
        auto r = cfo::decode_console(p, t_zero);
        REQUIRE(r);
        CHECK(r->text.empty());
    }
    SUBCASE("embedded NUL byte is preserved (string length, not C-string)") {
        const auto p = pkt_with_payload(0, 0,
            {'a', 0x00, 'b'});
        auto r = cfo::decode_console(p, t_zero);
        REQUIRE(r);
        CHECK(r->text.size() == 3);
        CHECK(r->text[0] == 'a');
        CHECK(r->text[1] == '\0');
        CHECK(r->text[2] == 'b');
    }
    SUBCASE("non-ASCII bytes are passed through verbatim") {
        const auto p = pkt_with_payload(0, 0, {0xFF, 0x80, 0x01});
        auto r = cfo::decode_console(p, t_zero);
        REQUIRE(r);
        CHECK(r->text.size() == 3);
        CHECK(static_cast<unsigned char>(r->text[0]) == 0xFF);
        CHECK(static_cast<unsigned char>(r->text[1]) == 0x80);
        CHECK(static_cast<unsigned char>(r->text[2]) == 0x01);
    }
    SUBCASE("max payload size (31 bytes) is decoded entirely") {
        cfo::RawPacket p{};
        p.port = 0;
        p.channel = 0;
        p.size = static_cast<std::uint8_t>(cfo::kMaxCrtpPayload);
        for (std::size_t i = 0; i < cfo::kMaxCrtpPayload; ++i) {
            p.payload[i] = static_cast<std::uint8_t>('A' + (i % 26));
        }
        auto r = cfo::decode_console(p, t_zero);
        REQUIRE(r);
        CHECK(r->text.size() == cfo::kMaxCrtpPayload);
        CHECK(r->text.front() == 'A');
    }
    SUBCASE("wrong port is rejected — caller must classify first") {
        const auto p = pkt_with_payload(5, 0, {'x'});
        auto r = cfo::decode_console(p, t_zero);
        REQUIRE_FALSE(r);
        CHECK(r.error() == cfo::DecodeError::WrongPort);
    }
    SUBCASE("right port, wrong channel is rejected") {
        const auto p = pkt_with_payload(0, 1, {'x'});
        auto r = cfo::decode_console(p, t_zero);
        REQUIRE_FALSE(r);
        CHECK(r.error() == cfo::DecodeError::WrongPort);
    }
}

TEST_CASE("make_null_packet has the link-control shape") {
    const auto p = cfo::make_null_packet();
    CHECK(p.port == 15);
    CHECK(p.channel == 3);
    CHECK(p.size == 0);
    // Classification round-trip — the same packet that any decoder would
    // see if it came back from the wire.
    CHECK(cfo::classify(p) == cfo::PacketKind::LinkControl);
}
