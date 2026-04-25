#include "crazyflie/link/types.hpp"
#include "crazyflie/protocol/protocol.hpp"
#include "crazyflie/protocol/types.hpp"

#include <doctest/doctest.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace {

cfo::RawPacket pkt(std::uint8_t port, std::uint8_t channel,
                   std::initializer_list<std::uint8_t> bytes) {
    cfo::RawPacket p{};
    p.port = port;
    p.channel = channel;
    p.size = static_cast<std::uint8_t>(bytes.size());
    std::size_t i = 0;
    for (auto b : bytes) p.payload[i++] = b;
    return p;
}

void put_f32(cfo::RawPacket& p, std::size_t off, float v) {
    std::uint32_t bits;
    std::memcpy(&bits, &v, 4);
    p.payload[off + 0] = static_cast<std::uint8_t>(bits & 0xFF);
    p.payload[off + 1] = static_cast<std::uint8_t>((bits >> 8) & 0xFF);
    p.payload[off + 2] = static_cast<std::uint8_t>((bits >> 16) & 0xFF);
    p.payload[off + 3] = static_cast<std::uint8_t>((bits >> 24) & 0xFF);
}

void put_u24(cfo::RawPacket& p, std::size_t off, std::uint32_t v) {
    p.payload[off + 0] = static_cast<std::uint8_t>(v & 0xFF);
    p.payload[off + 1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
    p.payload[off + 2] = static_cast<std::uint8_t>((v >> 16) & 0xFF);
}

} // namespace

TEST_CASE("fp16_to_float covers IEEE 754 half-precision corner cases") {
    SUBCASE("zero")            { CHECK(cfo::fp16_to_float(0x0000) == 0.0f); }
    SUBCASE("negative zero")   { CHECK(cfo::fp16_to_float(0x8000) == 0.0f); }
    SUBCASE("one")             { CHECK(cfo::fp16_to_float(0x3C00) == 1.0f); }
    SUBCASE("negative one")    { CHECK(cfo::fp16_to_float(0xBC00) == -1.0f); }
    SUBCASE("two")             { CHECK(cfo::fp16_to_float(0x4000) == 2.0f); }
    SUBCASE("smallest > 1") {
        // 1 + 2^-10
        const float expected = 1.0f + 1.0f / 1024.0f;
        CHECK(cfo::fp16_to_float(0x3C01) == doctest::Approx(expected));
    }
    SUBCASE("battery-typical 3.7 V is well-represented") {
        // Encode 3.7 as fp16, decode, check round-trip is within fp16 precision.
        // 3.7 in fp16: exponent = 1 (bias 15 → biased 16), mantissa for 1.85
        // → 1.85 = 1 + 0.85; 0.85 * 1024 ≈ 870 → 0x366
        const std::uint16_t encoded = 0x4366;
        const float v = cfo::fp16_to_float(encoded);
        CHECK(v == doctest::Approx(3.7f).epsilon(0.01));
    }
    SUBCASE("infinity") {
        const float v = cfo::fp16_to_float(0x7C00);
        CHECK(std::isinf(v));
        CHECK(v > 0);
    }
    SUBCASE("nan") {
        const float v = cfo::fp16_to_float(0x7E00);
        CHECK(std::isnan(v));
    }
}

TEST_CASE("LOG outbound packet builders match cflib semantics") {
    SUBCASE("make_log_reset: settings ch, single byte CMD_RESET_LOGGING") {
        const auto p = cfo::make_log_reset();
        CHECK(p.port == 5);
        CHECK(p.channel == 1);
        CHECK(p.size == 1);
        CHECK(p.payload[0] == 0x05);
    }
    SUBCASE("make_log_toc_info_v2_request: TOC ch, CMD_TOC_INFO_V2") {
        const auto p = cfo::make_log_toc_info_v2_request();
        CHECK(p.port == 5);
        CHECK(p.channel == 0);
        CHECK(p.size == 1);
        CHECK(p.payload[0] == 0x03);
    }
    SUBCASE("make_log_toc_item_v2_request: cmd + uint16 LE index") {
        const auto p = cfo::make_log_toc_item_v2_request(0x0142);
        CHECK(p.port == 5);
        CHECK(p.channel == 0);
        CHECK(p.size == 3);
        CHECK(p.payload[0] == 0x02);
        CHECK(p.payload[1] == 0x42);
        CHECK(p.payload[2] == 0x01);
    }
    SUBCASE("make_log_create_block_v2: cmd + block_id + N×(type, id_lo, id_hi)") {
        const std::array<cfo::LogVarSpec, 2> vars{{
            {0x77, 0x00AB},   // float-stored, float-fetched
            {0x78, 0x0102},   // float-stored, fp16-fetched
        }};
        const auto p = cfo::make_log_create_block_v2(7, vars);
        CHECK(p.port == 5);
        CHECK(p.channel == 1);
        CHECK(p.size == 2 + 3 * 2);
        CHECK(p.payload[0] == 0x06);   // CMD_CREATE_BLOCK_V2
        CHECK(p.payload[1] == 7);      // block_id
        CHECK(p.payload[2] == 0x77);
        CHECK(p.payload[3] == 0xAB);
        CHECK(p.payload[4] == 0x00);
        CHECK(p.payload[5] == 0x78);
        CHECK(p.payload[6] == 0x02);
        CHECK(p.payload[7] == 0x01);
    }
    SUBCASE("make_log_start_block: cmd + block_id + period (10ms units)") {
        const auto p = cfo::make_log_start_block(0, 5);
        CHECK(p.port == 5);
        CHECK(p.channel == 1);
        CHECK(p.size == 3);
        CHECK(p.payload[0] == 0x03);   // CMD_START_LOGGING
        CHECK(p.payload[1] == 0);
        CHECK(p.payload[2] == 5);
    }
}

TEST_CASE("decode_log_toc_info_v2") {
    SUBCASE("happy path") {
        // payload: cmd, count_lo, count_hi, crc_b0..b3, [+ extras ignored]
        const auto p = pkt(5, 0, {0x03, 0x39, 0x01, 0x12, 0x34, 0x56, 0x78});
        auto r = cfo::decode_log_toc_info_v2(p);
        REQUIRE(r);
        CHECK(r->count == 0x0139);
        CHECK(r->crc == 0x78563412u);
    }
    SUBCASE("wrong port") {
        const auto p = pkt(0, 0, {0x03, 0, 0, 0, 0, 0, 0});
        CHECK(cfo::decode_log_toc_info_v2(p).error() == cfo::DecodeError::WrongPort);
    }
    SUBCASE("truncated") {
        const auto p = pkt(5, 0, {0x03, 0x01});
        CHECK(cfo::decode_log_toc_info_v2(p).error() == cfo::DecodeError::Truncated);
    }
    SUBCASE("wrong cmd byte") {
        const auto p = pkt(5, 0, {0x99, 0, 0, 0, 0, 0, 0});
        CHECK(cfo::decode_log_toc_info_v2(p).error() == cfo::DecodeError::UnexpectedCmd);
    }
}

TEST_CASE("decode_log_toc_item_v2") {
    SUBCASE("happy path: stateEstimate.x at index 0x0042, type float") {
        // cmd, idx_lo, idx_hi, type, "stateEstimate\0", "x\0"
        cfo::RawPacket p{};
        p.port = 5; p.channel = 0;
        const std::uint8_t bytes[] = {
            0x02, 0x42, 0x00, 0x07,
            's','t','a','t','e','E','s','t','i','m','a','t','e','\0',
            'x','\0'
        };
        p.size = sizeof(bytes);
        std::memcpy(p.payload.data(), bytes, sizeof(bytes));
        auto r = cfo::decode_log_toc_item_v2(p);
        REQUIRE(r);
        CHECK(r->index == 0x0042);
        CHECK(r->type_code == 7);   // LOG_FLOAT
        CHECK(r->group == "stateEstimate");
        CHECK(r->name == "x");
    }
    SUBCASE("missing name terminator → Truncated") {
        cfo::RawPacket p{};
        p.port = 5; p.channel = 0;
        const std::uint8_t bytes[] = {
            0x02, 0x00, 0x00, 0x07,
            'p','m','\0',
            'v','b','a','t'   // no NUL
        };
        p.size = sizeof(bytes);
        std::memcpy(p.payload.data(), bytes, sizeof(bytes));
        CHECK(cfo::decode_log_toc_item_v2(p).error() == cfo::DecodeError::Truncated);
    }
    SUBCASE("wrong cmd byte") {
        const auto p = pkt(5, 0, {0x99, 0, 0, 7, 0, 0});
        CHECK(cfo::decode_log_toc_item_v2(p).error() == cfo::DecodeError::UnexpectedCmd);
    }
}

TEST_CASE("decode_log_settings_ack") {
    SUBCASE("ok ack for create-block-v2 of block 0") {
        const auto p = pkt(5, 1, {0x06, 0x00, 0x00});
        auto r = cfo::decode_log_settings_ack(p);
        REQUIRE(r);
        CHECK(r->cmd == 0x06);
        CHECK(r->block_id == 0);
        CHECK(r->error_code == 0);
    }
    SUBCASE("EEXIST ack — caller policy decides whether to accept") {
        const auto p = pkt(5, 1, {0x06, 0x00, 0x11});  // 17 = EEXIST on Linux
        auto r = cfo::decode_log_settings_ack(p);
        REQUIRE(r);
        CHECK(r->error_code == 0x11);
    }
    SUBCASE("wrong channel") {
        const auto p = pkt(5, 0, {0x06, 0x00, 0x00});
        CHECK(cfo::decode_log_settings_ack(p).error() == cfo::DecodeError::WrongPort);
    }
    SUBCASE("truncated") {
        const auto p = pkt(5, 1, {0x06, 0x00});
        CHECK(cfo::decode_log_settings_ack(p).error() == cfo::DecodeError::Truncated);
    }
}

TEST_CASE("decode_log_block_sample (slice 2's fixed layout)") {
    // Build a synthetic 30-byte log packet. Layout:
    //   block_id | timestamp_ms (24 LE) | x f32 | y f32 | z f32
    //   | vbat fp16 | roll f32 | pitch f32 | yaw f32
    cfo::RawPacket p{};
    p.port = 5; p.channel = 2;
    p.size = static_cast<std::uint8_t>(cfo::kLogBlockPayloadSize);
    p.payload[0] = 0;                 // block_id
    put_u24(p, 1, 0x123456);          // 1193046 ms
    put_f32(p, 4,  1.5f);             // x
    put_f32(p, 8, -2.25f);            // y
    put_f32(p, 12, 0.75f);            // z
    p.payload[16] = 0x66; p.payload[17] = 0x43;  // vbat fp16 ≈ 3.7
    put_f32(p, 18, 10.0f);            // roll deg
    put_f32(p, 22, -5.5f);            // pitch deg
    put_f32(p, 26, 180.0f);           // yaw deg

    SUBCASE("happy path") {
        auto r = cfo::decode_log_block_sample(p, /*expected_block_id=*/0);
        REQUIRE(r);
        CHECK(r->timestamp_ms == 0x123456u);
        CHECK(r->x == 1.5f);
        CHECK(r->y == -2.25f);
        CHECK(r->z == 0.75f);
        CHECK(r->vbat == doctest::Approx(3.7f).epsilon(0.01));
        CHECK(r->roll == 10.0f);
        CHECK(r->pitch == -5.5f);
        CHECK(r->yaw == 180.0f);
    }
    SUBCASE("wrong port → WrongPort") {
        cfo::RawPacket q = p; q.port = 0;
        CHECK(cfo::decode_log_block_sample(q, 0).error() == cfo::DecodeError::WrongPort);
    }
    SUBCASE("wrong channel → WrongPort") {
        cfo::RawPacket q = p; q.channel = 1;
        CHECK(cfo::decode_log_block_sample(q, 0).error() == cfo::DecodeError::WrongPort);
    }
    SUBCASE("undersized payload → WrongSize") {
        cfo::RawPacket q = p; q.size = 29;
        CHECK(cfo::decode_log_block_sample(q, 0).error() == cfo::DecodeError::WrongSize);
    }
    SUBCASE("oversized payload → WrongSize") {
        // Empty over the cap shouldn't ever happen via the link, but the
        // decoder still rejects it rather than reading uninitialized bytes.
        cfo::RawPacket q = p; q.size = 31;
        CHECK(cfo::decode_log_block_sample(q, 0).error() == cfo::DecodeError::WrongSize);
    }
    SUBCASE("wrong block id → WrongBlockId") {
        CHECK(cfo::decode_log_block_sample(p, /*expected_block_id=*/1).error()
              == cfo::DecodeError::WrongBlockId);
    }
}
