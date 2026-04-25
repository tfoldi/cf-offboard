#include "crazyflie/log/log_session.hpp"

#include "crazyflie/protocol/protocol.hpp"
#include "crazyflie/protocol/types.hpp"

#include <array>
#include <cerrno>
#include <chrono>
#include <string>
#include <string_view>

namespace cfo {

namespace {

using clock = std::chrono::steady_clock;

struct WantedVar {
    std::string_view group;
    std::string_view name;
    std::uint16_t* slot;
};

std::string fqn(std::string_view group, std::string_view name) {
    std::string s;
    s.reserve(group.size() + 1 + name.size());
    s.append(group);
    s.push_back('.');
    s.append(name);
    return s;
}

template <typename Predicate>
std::expected<RawPacket, LogSetupFailure>
wait_for(ICrazyflieLink& link,
         const std::function<void(const RawPacket&)>& passthrough,
         clock::time_point deadline,
         std::string_view step,
         Predicate&& predicate) {
    using namespace std::chrono_literals;
    while (clock::now() < deadline) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - clock::now());
        const auto poll =
            (remaining < 50ms) ? remaining : std::chrono::milliseconds{50};
        auto pkt = link.receive(poll);
        if (!pkt) continue;
        if (predicate(*pkt)) return *pkt;
        if (passthrough) passthrough(*pkt);
    }
    return std::unexpected(LogSetupFailure{
        LogSetupError::Timeout,
        std::string{"timeout "} + std::string{step}});
}

bool is_settings_ack(const RawPacket& p, std::uint8_t cmd) {
    return p.port == crtp::kPortLog &&
           p.channel == crtp::kChannelLogSettings &&
           p.size >= 1 && p.payload[0] == cmd;
}

bool is_toc_info_v2(const RawPacket& p) {
    return p.port == crtp::kPortLog &&
           p.channel == crtp::kChannelLogToc &&
           p.size >= 1 && p.payload[0] == crtp::kCmdTocInfoV2;
}

bool is_toc_item_v2(const RawPacket& p, std::uint16_t expected_index) {
    if (!(p.port == crtp::kPortLog &&
          p.channel == crtp::kChannelLogToc &&
          p.size >= 4 && p.payload[0] == crtp::kCmdTocItemV2)) {
        return false;
    }
    const std::uint16_t idx = static_cast<std::uint16_t>(
        p.payload[1] | (p.payload[2] << 8));
    return idx == expected_index;
}

void notify(const LogSetupNotifier& on_progress, const LogSetupProgress& p) {
    if (on_progress) on_progress(p);
}

} // namespace

std::expected<LogResolved, LogSetupFailure>
setup_log_block(ICrazyflieLink& link,
                std::function<void(const RawPacket&)> passthrough,
                LogSetupNotifier on_progress,
                std::chrono::milliseconds total_timeout) {
    using namespace std::chrono_literals;
    const auto deadline = clock::now() + total_timeout;

    // 1) RESET_LOGGING — best effort. Drain ack briefly so it doesn't pollute
    //    later waits, but tolerate timeout (firmware variants differ).
    if (auto r = link.send(make_log_reset()); !r) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::SendFailed, "send reset"});
    }
    (void)wait_for(link, passthrough, clock::now() + 200ms, "draining reset ack",
        [](const RawPacket& p) {
            return is_settings_ack(p, crtp::kCmdLogResetLogging);
        });
    notify(on_progress, LogSetupProgress{LogSetupProgress::Phase::ResetSent});

    // 2) GET TOC_INFO_V2.
    if (auto r = link.send(make_log_toc_info_v2_request()); !r) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::SendFailed, "send toc_info_v2"});
    }
    auto info_pkt = wait_for(link, passthrough, deadline,
                             "during TOC info", is_toc_info_v2);
    if (!info_pkt) return std::unexpected(info_pkt.error());
    auto info = decode_log_toc_info_v2(*info_pkt);
    if (!info) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::TocInfoFailed, "decode toc_info_v2 reply"});
    }
    {
        LogSetupProgress p{};
        p.phase = LogSetupProgress::Phase::TocInfoReceived;
        p.toc_info = *info;
        notify(on_progress, p);
    }

    // 3) Walk the TOC for our 7 named variables.
    LogResolved resolved{};
    std::array<WantedVar, kLogVarCount> wants{{
        {"stateEstimate", "x",     &resolved.state_x_id},
        {"stateEstimate", "y",     &resolved.state_y_id},
        {"stateEstimate", "z",     &resolved.state_z_id},
        {"pm",            "vbat",  &resolved.vbat_id},
        {"stabilizer",    "roll",  &resolved.roll_id},
        {"stabilizer",    "pitch", &resolved.pitch_id},
        {"stabilizer",    "yaw",   &resolved.yaw_id},
    }};
    std::uint8_t found = 0;
    for (std::uint16_t i = 0;
         i < info->count && found < kLogVarCount;
         ++i) {
        if (auto r = link.send(make_log_toc_item_v2_request(i)); !r) {
            return std::unexpected(LogSetupFailure{
                LogSetupError::SendFailed, "send toc_item_v2"});
        }
        auto item_pkt = wait_for(link, passthrough, deadline,
            std::string{"awaiting toc_item_v2#"} + std::to_string(i),
            [i](const RawPacket& p) { return is_toc_item_v2(p, i); });
        if (!item_pkt) return std::unexpected(item_pkt.error());
        auto item = decode_log_toc_item_v2(*item_pkt);
        if (!item) {
            return std::unexpected(LogSetupFailure{
                LogSetupError::TocItemFailed,
                "decode toc_item_v2#" + std::to_string(i)});
        }
        for (auto& w : wants) {
            if (w.slot && item->group == w.group && item->name == w.name) {
                *w.slot = item->index;
                w.slot = nullptr;
                ++found;
                LogSetupProgress p{};
                p.phase = LogSetupProgress::Phase::VarResolved;
                p.var_name = fqn(w.group, w.name);
                p.resolved = found;
                notify(on_progress, p);
                break;
            }
        }
    }
    if (found < kLogVarCount) {
        // Report the first missing var name. Multiple missing is rare; one
        // identifier is enough to tell the operator which firmware mismatch
        // they're looking at.
        std::string missing;
        for (auto& w : wants) {
            if (w.slot) {
                missing = fqn(w.group, w.name);
                break;
            }
        }
        return std::unexpected(LogSetupFailure{
            LogSetupError::VarNotFound, "missing var: " + missing});
    }

    // 4) CREATE_BLOCK_V2.
    constexpr std::uint8_t kFloat = crtp::log_type_byte(
        crtp::kLogTypeFloat, crtp::kLogTypeFloat);
    constexpr std::uint8_t kVbat  = crtp::log_type_byte(
        crtp::kLogTypeFloat, crtp::kLogTypeFP16);

    const std::array<LogVarSpec, kLogVarCount> specs{{
        {kFloat, resolved.state_x_id},
        {kFloat, resolved.state_y_id},
        {kFloat, resolved.state_z_id},
        {kVbat,  resolved.vbat_id},
        {kFloat, resolved.roll_id},
        {kFloat, resolved.pitch_id},
        {kFloat, resolved.yaw_id},
    }};
    if (auto r = link.send(make_log_create_block_v2(kLogBlockId, specs)); !r) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::SendFailed, "send create_block_v2"});
    }
    auto create_ack = wait_for(link, passthrough, deadline,
        "awaiting create_block ack",
        [](const RawPacket& p) {
            return is_settings_ack(p, crtp::kCmdLogCreateBlockV2);
        });
    if (!create_ack) return std::unexpected(create_ack.error());
    auto ack = decode_log_settings_ack(*create_ack);
    if (!ack) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::BlockCreateFailed, "malformed create_block ack"});
    }
    if (ack->error_code != 0 && ack->error_code != EEXIST) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::BlockCreateFailed,
            "create_block error_code=" + std::to_string(ack->error_code)});
    }
    notify(on_progress, LogSetupProgress{LogSetupProgress::Phase::BlockCreated});

    // 5) START_LOGGING.
    if (auto r = link.send(make_log_start_block(kLogBlockId, kLogPeriod10ms)); !r) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::SendFailed, "send start_logging"});
    }
    auto start_ack = wait_for(link, passthrough, deadline,
        "awaiting start_logging ack",
        [](const RawPacket& p) {
            return is_settings_ack(p, crtp::kCmdLogStartLogging);
        });
    if (!start_ack) return std::unexpected(start_ack.error());
    auto sack = decode_log_settings_ack(*start_ack);
    if (!sack) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::BlockStartFailed, "malformed start_logging ack"});
    }
    if (sack->error_code != 0) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::BlockStartFailed,
            "start_logging error_code=" + std::to_string(sack->error_code)});
    }
    {
        LogSetupProgress p{};
        p.phase = LogSetupProgress::Phase::BlockStarted;
        p.period_10ms = kLogPeriod10ms;
        notify(on_progress, p);
    }
    return resolved;
}

std::expected<SupervisorState, LogSetupFailure>
query_supervisor_state(ICrazyflieLink& link,
                       std::function<void(const RawPacket&)> passthrough,
                       std::chrono::milliseconds timeout) {
    if (auto r = link.send(make_supervisor_info_request()); !r) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::SendFailed, "send supervisor info"});
    }
    const auto deadline = clock::now() + timeout;
    const std::uint8_t expected_cmd =
        crtp::kCmdSupervisorGetStateBitfield | crtp::kCmdResponseFlag;
    auto reply = wait_for(link, passthrough, deadline,
        "awaiting supervisor info",
        [expected_cmd](const RawPacket& p) {
            return p.port == crtp::kPortSupervisor &&
                   p.channel == crtp::kChannelSupervisorInfo &&
                   p.size >= 1 &&
                   p.payload[0] == expected_cmd;
        });
    if (!reply) return std::unexpected(reply.error());
    auto state = decode_supervisor_state(*reply);
    if (!state) {
        return std::unexpected(LogSetupFailure{
            LogSetupError::TocItemFailed,
            "decode supervisor state"});
    }
    return *state;
}

} // namespace cfo
