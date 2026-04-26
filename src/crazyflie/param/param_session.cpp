#include "crazyflie/param/param_session.hpp"

#include "crazyflie/protocol/protocol.hpp"
#include "crazyflie/protocol/types.hpp"

#include <chrono>
#include <string>
#include <string_view>

namespace cfo {

namespace {

using clock = std::chrono::steady_clock;

template <typename Predicate>
std::expected<RawPacket, ParamSetupFailure>
wait_for(ICrazyflieLink& link,
         const std::function<void(const RawPacket&)>& passthrough,
         clock::time_point deadline,
         std::string_view step,
         Predicate&& pred) {
    using namespace std::chrono_literals;
    while (clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - clock::now());
        const auto poll =
            (remaining < 50ms) ? remaining : std::chrono::milliseconds{50};
        auto pkt = link.receive(poll);
        if (!pkt) continue;
        if (pred(*pkt)) return *pkt;
        if (passthrough) passthrough(*pkt);
    }
    return std::unexpected(ParamSetupFailure{
        ParamSetupError::Timeout, std::string{"timeout "} + std::string{step}});
}

bool is_param_toc_info_v2(const RawPacket& p) {
    return p.port == crtp::kPortParam &&
           p.channel == crtp::kChannelParamToc &&
           p.size >= 1 && p.payload[0] == crtp::kCmdTocInfoV2;
}

bool is_param_toc_item_v2(const RawPacket& p, std::uint16_t expected_index) {
    if (!(p.port == crtp::kPortParam &&
          p.channel == crtp::kChannelParamToc &&
          p.size >= 4 && p.payload[0] == crtp::kCmdTocItemV2)) {
        return false;
    }
    const std::uint16_t idx = static_cast<std::uint16_t>(
        p.payload[1] | (p.payload[2] << 8));
    return idx == expected_index;
}

} // namespace

std::expected<void, ParamSetupFailure>
write_uint8_param(ICrazyflieLink& link,
                  std::string_view group,
                  std::string_view name,
                  std::uint8_t value,
                  std::function<void(const RawPacket&)> passthrough,
                  std::chrono::milliseconds total_timeout) {
    const auto deadline = clock::now() + total_timeout;

    // 1) Get TOC info to learn how many entries to walk.
    if (auto r = link.send(make_param_toc_info_v2_request()); !r) {
        return std::unexpected(ParamSetupFailure{
            ParamSetupError::SendFailed, "send param toc_info_v2"});
    }
    auto info_pkt = wait_for(link, passthrough, deadline,
                             "during param toc info",
                             is_param_toc_info_v2);
    if (!info_pkt) return std::unexpected(info_pkt.error());
    auto info = decode_param_toc_info_v2(*info_pkt);
    if (!info) {
        return std::unexpected(ParamSetupFailure{
            ParamSetupError::TocInfoFailed, "decode param toc_info_v2 reply"});
    }

    // 2) Walk the TOC until we find the named variable.
    std::uint16_t found_id = 0;
    bool found = false;
    for (std::uint16_t i = 0; i < info->count; ++i) {
        if (auto r = link.send(make_param_toc_item_v2_request(i)); !r) {
            return std::unexpected(ParamSetupFailure{
                ParamSetupError::SendFailed, "send param toc_item_v2"});
        }
        auto item_pkt = wait_for(link, passthrough, deadline,
            std::string{"awaiting param toc_item_v2#"} + std::to_string(i),
            [i](const RawPacket& p) {
                return is_param_toc_item_v2(p, i);
            });
        if (!item_pkt) return std::unexpected(item_pkt.error());
        auto item = decode_param_toc_item_v2(*item_pkt);
        if (!item) {
            return std::unexpected(ParamSetupFailure{
                ParamSetupError::TocItemFailed,
                "decode param toc_item_v2#" + std::to_string(i)});
        }
        if (item->group == group && item->name == name) {
            if (item->type_code != crtp::kParamTypeUint8) {
                return std::unexpected(ParamSetupFailure{
                    ParamSetupError::WrongType,
                    std::string{group} + "." + std::string{name} +
                    " has type_code=" + std::to_string(item->type_code) +
                    " (expected uint8)"});
            }
            found_id = item->index;
            found = true;
            break;
        }
    }
    if (!found) {
        return std::unexpected(ParamSetupFailure{
            ParamSetupError::VarNotFound,
            "missing param: " + std::string{group} + "." + std::string{name}});
    }

    // 3) WRITE the value, wait for the var_id echo as ack.
    if (auto r = link.send(make_param_write_uint8(found_id, value)); !r) {
        return std::unexpected(ParamSetupFailure{
            ParamSetupError::SendFailed, "send param write"});
    }
    auto ack = wait_for(link, passthrough, deadline,
        "awaiting param write ack",
        [found_id](const RawPacket& p) {
            return is_param_write_ack(p, found_id);
        });
    if (!ack) return std::unexpected(ParamSetupFailure{
        ParamSetupError::WriteAckTimeout,
        "no echo for var " + std::to_string(found_id)});
    return {};
}

} // namespace cfo
