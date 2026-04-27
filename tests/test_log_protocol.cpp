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

float read_f32(const cfo::RawPacket& p, std::size_t off) {
    std::uint32_t bits =
        static_cast<std::uint32_t>(p.payload[off + 0]) |
        (static_cast<std::uint32_t>(p.payload[off + 1]) << 8) |
        (static_cast<std::uint32_t>(p.payload[off + 2]) << 16) |
        (static_cast<std::uint32_t>(p.payload[off + 3]) << 24);
    float v;
    std::memcpy(&v, &bits, 4);
    return v;
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

TEST_CASE("make_hlc_land: 15-byte payload, IEEE-754 LE") {
    const auto p = cfo::make_hlc_land(/*height=*/0.0f, /*duration=*/2.0f);
    CHECK(p.port == 8);    // SETPOINT_HL
    CHECK(p.channel == 0);
    CHECK(p.size == 15);
    CHECK(p.payload[0] == 0x08);   // COMMAND_LAND_2
    CHECK(p.payload[1] == 0);      // group_mask
    CHECK(read_f32(p, 2)  == doctest::Approx(0.0f));   // height
    CHECK(read_f32(p, 6)  == doctest::Approx(0.0f));   // yaw
    CHECK(p.payload[10] == 1);                          // use_current_yaw=true (default)
    CHECK(read_f32(p, 11) == doctest::Approx(2.0f));   // duration
}

TEST_CASE("make_hlc_stop: 2-byte payload (opcode + group_mask)") {
    SUBCASE("default group_mask=0") {
        const auto p = cfo::make_hlc_stop();
        CHECK(p.port == 8);
        CHECK(p.channel == 0);
        CHECK(p.size == 2);
        CHECK(p.payload[0] == 0x03);   // COMMAND_STOP
        CHECK(p.payload[1] == 0);
    }
    SUBCASE("explicit group_mask") {
        const auto p = cfo::make_hlc_stop(0xAA);
        CHECK(p.size == 2);
        CHECK(p.payload[1] == 0xAA);
    }
}

TEST_CASE("make_hlc_takeoff: same 15-byte layout as LAND, opcode 0x07") {
    const auto p = cfo::make_hlc_takeoff(/*height=*/0.30f, /*duration=*/1.5f);
    CHECK(p.port == 8);
    CHECK(p.channel == 0);
    CHECK(p.size == 15);
    CHECK(p.payload[0] == 0x07);   // COMMAND_TAKEOFF_2
    CHECK(p.payload[1] == 0);
    CHECK(read_f32(p, 2)  == doctest::Approx(0.30f));
    CHECK(read_f32(p, 6)  == doctest::Approx(0.0f));
    CHECK(p.payload[10] == 1);     // use_current_yaw default
    CHECK(read_f32(p, 11) == doctest::Approx(1.5f));
}

TEST_CASE("make_hlc_go_to: 24-byte payload (cflib COMMAND_GO_TO_2)") {
    SUBCASE("relative=true (default), linear=false") {
        const auto p = cfo::make_hlc_go_to(0.225f, 0, 0, 0, 1.5f);
        CHECK(p.port == 8);
        CHECK(p.channel == 0);
        CHECK(p.size == 24);
        CHECK(p.payload[0] == 0x0C);   // COMMAND_GO_TO_2
        CHECK(p.payload[1] == 0);      // group_mask
        CHECK(p.payload[2] == 1);      // relative
        CHECK(p.payload[3] == 0);      // linear
        CHECK(read_f32(p, 4)  == doctest::Approx(0.225f));   // x
        CHECK(read_f32(p, 8)  == doctest::Approx(0.0f));     // y
        CHECK(read_f32(p, 12) == doctest::Approx(0.0f));     // z
        CHECK(read_f32(p, 16) == doctest::Approx(0.0f));     // yaw
        CHECK(read_f32(p, 20) == doctest::Approx(1.5f));     // duration
    }
    SUBCASE("absolute, linear, group_mask, all axes set") {
        const auto p = cfo::make_hlc_go_to(1.0f, -0.5f, 0.4f,
                                           1.5708f, 2.0f,
                                           /*relative=*/false,
                                           /*linear=*/true,
                                           /*group_mask=*/0x0F);
        CHECK(p.payload[1] == 0x0F);
        CHECK(p.payload[2] == 0);   // relative=false
        CHECK(p.payload[3] == 1);   // linear=true
        CHECK(read_f32(p, 4)  == doctest::Approx(1.0f));
        CHECK(read_f32(p, 8)  == doctest::Approx(-0.5f));
        CHECK(read_f32(p, 12) == doctest::Approx(0.4f));
        CHECK(read_f32(p, 16) == doctest::Approx(1.5708f));
        CHECK(read_f32(p, 20) == doctest::Approx(2.0f));
    }
}

TEST_CASE("make_hlc_land: explicit yaw, group_mask, custom height") {
    const auto p = cfo::make_hlc_land(0.05f, 2.5f,
                                       /*use_current_yaw=*/false,
                                       /*yaw_rad=*/1.5708f,
                                       /*group_mask=*/0xAA);
    CHECK(p.payload[1] == 0xAA);
    CHECK(read_f32(p, 2)  == doctest::Approx(0.05f));
    CHECK(read_f32(p, 6)  == doctest::Approx(1.5708f));
    CHECK(p.payload[10] == 0);
    CHECK(read_f32(p, 11) == doctest::Approx(2.5f));
}

TEST_CASE("make_param_toc_info_v2_request: PARAM TOC, single byte CMD_TOC_INFO_V2") {
    const auto p = cfo::make_param_toc_info_v2_request();
    CHECK(p.port == 2);
    CHECK(p.channel == 0);
    CHECK(p.size == 1);
    CHECK(p.payload[0] == 0x03);
}

TEST_CASE("make_param_toc_item_v2_request: PARAM TOC, cmd + uint16 LE index") {
    const auto p = cfo::make_param_toc_item_v2_request(0x0142);
    CHECK(p.port == 2);
    CHECK(p.channel == 0);
    CHECK(p.size == 3);
    CHECK(p.payload[0] == 0x02);
    CHECK(p.payload[1] == 0x42);
    CHECK(p.payload[2] == 0x01);
}

TEST_CASE("make_param_write_uint8: var_id LE + value byte on WRITE channel") {
    const auto p = cfo::make_param_write_uint8(0x0BAD, 0x77);
    CHECK(p.port == 2);
    CHECK(p.channel == 2);   // WRITE_CHANNEL
    CHECK(p.size == 3);
    CHECK(p.payload[0] == 0xAD);
    CHECK(p.payload[1] == 0x0B);
    CHECK(p.payload[2] == 0x77);
}

TEST_CASE("decode_param_toc_info_v2: same wire shape as LOG TOC info") {
    const auto p = pkt(2, 0, {0x03, 0x39, 0x01, 0xDE, 0xAD, 0xBE, 0xEF});
    auto r = cfo::decode_param_toc_info_v2(p);
    REQUIRE(r);
    CHECK(r->count == 0x0139);
    CHECK(r->crc == 0xEFBEADDEu);
}

TEST_CASE("decode_param_toc_item_v2: metadata byte holds type + flags + r/o bit") {
    SUBCASE("uint8 commander.enHighLevel") {
        cfo::RawPacket p{};
        p.port = 2; p.channel = 0;
        const std::uint8_t bytes[] = {
            0x02, 0x42, 0x00,                       // cmd + index 0x0042
            0x08,                                   // metadata: type uint8, no flags
            'c','o','m','m','a','n','d','e','r','\0',
            'e','n','H','i','g','h','L','e','v','e','l','\0'
        };
        p.size = sizeof(bytes);
        std::memcpy(p.payload.data(), bytes, sizeof(bytes));
        auto r = cfo::decode_param_toc_item_v2(p);
        REQUIRE(r);
        CHECK(r->index == 0x0042);
        CHECK(r->type_code == 0x08);   // uint8
        CHECK(r->read_only == false);
        CHECK(r->group == "commander");
        CHECK(r->name == "enHighLevel");
    }
    SUBCASE("read-only bit (0x40) is decoded") {
        cfo::RawPacket p{};
        p.port = 2; p.channel = 0;
        const std::uint8_t bytes[] = {
            0x02, 0x00, 0x00,
            0x48,                       // 0x40 (r/o) | 0x08 (uint8)
            'g','\0', 'n','\0'
        };
        p.size = sizeof(bytes);
        std::memcpy(p.payload.data(), bytes, sizeof(bytes));
        auto r = cfo::decode_param_toc_item_v2(p);
        REQUIRE(r);
        CHECK(r->type_code == 0x08);
        CHECK(r->read_only == true);
    }
}

TEST_CASE("is_param_write_ack: matches firmware echo on WRITE channel") {
    SUBCASE("matching var_id returns true") {
        const auto p = pkt(2, 2, {0x42, 0x01, 0x77});
        CHECK(cfo::is_param_write_ack(p, 0x0142) == true);
    }
    SUBCASE("non-matching var_id returns false") {
        const auto p = pkt(2, 2, {0x42, 0x01});
        CHECK(cfo::is_param_write_ack(p, 0x0143) == false);
    }
    SUBCASE("wrong channel is not an ack") {
        const auto p = pkt(2, 0, {0x42, 0x01});
        CHECK(cfo::is_param_write_ack(p, 0x0142) == false);
    }
    SUBCASE("truncated packet is not an ack") {
        const auto p = pkt(2, 2, {0x42});
        CHECK(cfo::is_param_write_ack(p, 0x0042) == false);
    }
}

TEST_CASE("decode_range_block_sample: 14-byte payload, five uint16 LE ranges") {
    cfo::RawPacket p{};
    p.port = 5; p.channel = 2;
    p.size = static_cast<std::uint8_t>(cfo::kRangeBlockPayloadSize);
    p.payload[0] = 1;                  // block_id
    put_u24(p, 1, 0x0ABCDE);
    auto put_u16 = [&](std::size_t off, std::uint16_t v) {
        p.payload[off]     = static_cast<std::uint8_t>(v & 0xFF);
        p.payload[off + 1] = static_cast<std::uint8_t>((v >> 8) & 0xFF);
    };
    put_u16(4,  800);    // front  0.80 m
    put_u16(6,  1500);   // back   1.50 m
    put_u16(8,  4000);   // left   4.00 m
    put_u16(10, 50);     // right  0.05 m
    put_u16(12, 8000);   // up     8.00 m

    SUBCASE("happy path") {
        auto r = cfo::decode_range_block_sample(p, /*expected=*/1);
        REQUIRE(r);
        CHECK(r->timestamp_ms == 0x0ABCDEu);
        CHECK(r->front_mm == 800);
        CHECK(r->back_mm  == 1500);
        CHECK(r->left_mm  == 4000);
        CHECK(r->right_mm == 50);
        CHECK(r->up_mm    == 8000);
    }
    SUBCASE("wrong block_id rejected") {
        CHECK(cfo::decode_range_block_sample(p, /*expected=*/0).error()
              == cfo::DecodeError::WrongBlockId);
    }
    SUBCASE("undersized rejected") {
        cfo::RawPacket q = p; q.size = 13;
        CHECK(cfo::decode_range_block_sample(q, 1).error()
              == cfo::DecodeError::WrongSize);
    }
    SUBCASE("wrong port/channel rejected") {
        cfo::RawPacket q = p; q.channel = 1;
        CHECK(cfo::decode_range_block_sample(q, 1).error()
              == cfo::DecodeError::WrongPort);
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
