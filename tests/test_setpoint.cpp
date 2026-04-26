#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/protocol.hpp"
#include "crazyflie/protocol/types.hpp"

#include <doctest/doctest.h>

#include <cstdint>
#include <cstring>

namespace {

float read_f32(const cfo::RawPacket& p, std::size_t off) {
    std::uint32_t bits =
        static_cast<std::uint32_t>(p.payload[off]) |
        (static_cast<std::uint32_t>(p.payload[off + 1]) << 8) |
        (static_cast<std::uint32_t>(p.payload[off + 2]) << 16) |
        (static_cast<std::uint32_t>(p.payload[off + 3]) << 24);
    float v;
    std::memcpy(&v, &bits, 4);
    return v;
}

} // namespace

TEST_CASE("make_setpoint_stop: type-only on generic-commander port") {
    const auto p = cfo::make_setpoint_stop();
    CHECK(p.port == 7);
    CHECK(p.channel == 0);
    CHECK(p.size == 1);
    CHECK(p.payload[0] == 0);   // kSetpointStop
}

TEST_CASE("make_setpoint_hover: 17-byte payload, IEEE-754 LE") {
    const auto p = cfo::make_setpoint_hover(0.25f, -0.10f, 30.0f, 0.30f);
    CHECK(p.port == 7);
    CHECK(p.channel == 0);
    CHECK(p.size == 17);
    CHECK(p.payload[0] == 5);   // kSetpointHover

    CHECK(read_f32(p, 1)  == doctest::Approx(0.25f));
    CHECK(read_f32(p, 5)  == doctest::Approx(-0.10f));
    CHECK(read_f32(p, 9)  == doctest::Approx(30.0f));
    CHECK(read_f32(p, 13) == doctest::Approx(0.30f));
}

TEST_CASE("hover with zero target_z still has 17 bytes (firmware expects fixed size)") {
    const auto p = cfo::make_setpoint_hover(0, 0, 0, 0);
    CHECK(p.size == 17);
    CHECK(p.payload[0] == 5);
    for (std::size_t i = 1; i < 17; ++i) CHECK(p.payload[i] == 0);
}

TEST_CASE("hover with negative z (descent target below ground) is allowed by builder") {
    // The builder is dumb — clamping is the safety supervisor's job.
    const auto p = cfo::make_setpoint_hover(0, 0, 0, -0.5f);
    CHECK(p.size == 17);
    CHECK(read_f32(p, 13) == doctest::Approx(-0.5f));
}

TEST_CASE("make_setpoint_notify_stop: type 0xFF + uint32 LE remain_valid_ms") {
    SUBCASE("default (immediate handoff)") {
        const auto p = cfo::make_setpoint_notify_stop();
        CHECK(p.port == 7);
        CHECK(p.channel == 0);
        CHECK(p.size == 5);
        CHECK(p.payload[0] == 0xFF);
        CHECK(p.payload[1] == 0);
        CHECK(p.payload[2] == 0);
        CHECK(p.payload[3] == 0);
        CHECK(p.payload[4] == 0);
    }
    SUBCASE("non-zero grace window encodes little-endian") {
        const auto p = cfo::make_setpoint_notify_stop(0x12345678u);
        CHECK(p.payload[0] == 0xFF);
        CHECK(p.payload[1] == 0x78);
        CHECK(p.payload[2] == 0x56);
        CHECK(p.payload[3] == 0x34);
        CHECK(p.payload[4] == 0x12);
    }
}
