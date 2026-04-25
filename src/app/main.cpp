#include "app/config.hpp"
#include "crazyflie/link/interfaces.hpp"
#include "crazyflie/link/types.hpp"
#include "crazyflie/log/log_session.hpp"
#include "crazyflie/protocol/protocol.hpp"
#include "crazyflie/protocol/types.hpp"
#include "logging/console.hpp"
#include "logging/mcap_logger.hpp"
#include "logging/types.hpp"
#include "state/state_store.hpp"
#include "state/types.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <numbers>
#include <optional>
#include <thread>
#include <utility>

namespace {

std::atomic<bool> g_shutdown{false};

extern "C" void on_signal(int) {
    g_shutdown.store(true, std::memory_order_relaxed);
}

cfo::LinkEvent make_link_event(cfo::LinkState s, std::string detail) {
    return cfo::LinkEvent{s, std::move(detail), std::chrono::system_clock::now()};
}

const char* setup_error_str(cfo::LogSetupError e) noexcept {
    switch (e) {
        case cfo::LogSetupError::SendFailed:        return "send_failed";
        case cfo::LogSetupError::Timeout:           return "toc_timeout";
        case cfo::LogSetupError::TocInfoFailed:     return "toc_info_failed";
        case cfo::LogSetupError::TocItemFailed:     return "toc_item_failed";
        case cfo::LogSetupError::VarNotFound:       return "var_not_found";
        case cfo::LogSetupError::BlockCreateFailed: return "block_create_failed";
        case cfo::LogSetupError::BlockStartFailed:  return "block_start_ack_failed";
    }
    return "unknown";
}

void log_setup_progress(const cfo::LogSetupProgress& p) {
    using P = cfo::LogSetupProgress::Phase;
    switch (p.phase) {
        case P::ResetSent:
            cfo::console::info("LOG: reset sent");
            break;
        case P::TocInfoReceived:
            cfo::console::info("LOG: TOC info received ({} entries, crc=0x{:08x})",
                               p.toc_info.count, p.toc_info.crc);
            break;
        case P::VarResolved:
            cfo::console::info("LOG: resolved {}/{}: {}",
                               p.resolved, p.total, p.var_name);
            break;
        case P::BlockCreated:
            cfo::console::info("LOG: block {} created", cfo::kLogBlockId);
            break;
        case P::BlockStarted:
            cfo::console::info("LOG: block {} started @ {} ms",
                               cfo::kLogBlockId, p.period_10ms * 10);
            break;
    }
}

} // namespace

int main(int argc, char** argv) {
    cfo::AppConfig cfg;
    if (argc >= 2) cfg.crazyflie_uri = argv[1];
    if (argc >= 3) cfg.mcap_path = argv[2];

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    cfo::console::info("cf-offboard starting");
    cfo::console::info("config: uri={} rx_poll={}ms log={}",
                       cfg.crazyflie_uri,
                       cfg.rx_poll_timeout.count(),
                       cfg.mcap_path.string());

    auto logger_r = cfo::MCAPLogger::create(cfg.mcap_path);
    if (!logger_r) {
        cfo::console::error("MCAP open failed: {}", cfg.mcap_path.string());
        return 1;
    }
    auto& logger = **logger_r;
    cfo::console::info("MCAP recording started: {}", cfg.mcap_path.string());

    auto link_r = cfo::open_crazyflie_link(cfg.crazyflie_uri);
    if (!link_r) {
        logger.log(make_link_event(cfo::LinkState::Error,
                                   "open_failed: " + cfg.crazyflie_uri));
        cfo::console::error("link open failed: {}", cfg.crazyflie_uri);
        logger.close();
        return 2;
    }
    auto& link = **link_r;

    logger.log(make_link_event(cfo::LinkState::Opened, cfg.crazyflie_uri));
    cfo::console::info("link opened: {}", cfg.crazyflie_uri);

    // Outbound smoke test — also verifies the send path before we start
    // pushing LOG/TOC traffic.
    {
        const auto null_pkt = cfo::make_null_packet();
        const auto t = std::chrono::system_clock::now();
        if (auto r = link.send(null_pkt); r) {
            logger.log(cfo::RawCommandEvent{null_pkt, t});
        } else {
            cfo::console::warn("initial null packet send failed");
        }
    }

    // -------------------------------------------------------------------
    // LOG bring-up. Synchronous: holds the receive line until the block
    // is created and started, then releases to the RX thread. Inbound
    // packets that arrive during setup (NULLs, console fragments, etc.)
    // are forwarded to the logger via passthrough so MCAP stays complete.
    // -------------------------------------------------------------------
    cfo::console::info("LOG: setup starting (block {} target {} ms)",
                       cfo::kLogBlockId, cfo::kLogPeriod10ms * 10);
    auto setup_r = cfo::setup_log_block(
        link,
        [&](const cfo::RawPacket& p) {
            const auto t = std::chrono::system_clock::now();
            logger.log(cfo::RawTelemetryEvent{p, t});
            if (cfo::classify(p) == cfo::PacketKind::Console) {
                if (auto m = cfo::decode_console(p, t); m) {
                    logger.log(cfo::ConsoleEvent{std::move(m->text), t});
                }
            }
        },
        log_setup_progress);
    if (!setup_r) {
        const auto& err = setup_r.error();
        const char* code = setup_error_str(err.code);
        cfo::console::error("LOG setup failed: {} ({})", code, err.detail);
        logger.log(make_link_event(cfo::LinkState::Error,
                                   std::string{"log_setup_"} + code + ": " + err.detail));
        logger.log(make_link_event(cfo::LinkState::Closed, cfg.crazyflie_uri));
        logger.close();
        return 3;
    }
    cfo::console::info("LOG: setup complete — telemetry active");

    cfo::StateStore state;
    std::atomic<bool> first_sample_seen{false};

    std::thread rx_thread{[&] {
        cfo::console::info("RX started (poll={}ms)", cfg.rx_poll_timeout.count());
        std::uint64_t log_samples = 0;
        constexpr std::uint64_t kRateWindow = 100;
        std::optional<std::chrono::steady_clock::time_point> rate_start;

        while (!g_shutdown.load(std::memory_order_relaxed)) {
            auto pkt = link.receive(cfg.rx_poll_timeout);
            if (!pkt) continue;
            const auto t = std::chrono::system_clock::now();

            logger.log(cfo::RawTelemetryEvent{*pkt, t});

            switch (cfo::classify(*pkt)) {
                case cfo::PacketKind::Console:
                    if (auto m = cfo::decode_console(*pkt, t); m) {
                        logger.log(cfo::ConsoleEvent{std::move(m->text), t});
                    }
                    break;

                case cfo::PacketKind::LogData: {
                    auto sample_r = cfo::decode_log_block_sample(*pkt, cfo::kLogBlockId);
                    if (!sample_r) break;
                    const auto& s = *sample_r;
                    logger.log(cfo::LogBlockEvent{s, t});

                    const auto t_steady = std::chrono::steady_clock::now();
                    state.apply(cfo::pose_from_log_sample(s, t_steady));
                    state.apply(cfo::battery_from_log_sample(s, t_steady));

                    ++log_samples;
                    if (log_samples == 1) {
                        cfo::console::info(
                            "first state: pos=({:.2f},{:.2f},{:.2f}) "
                            "att(deg)=({:.1f},{:.1f},{:.1f}) vbat={:.2f}V",
                            s.x, s.y, s.z, s.roll, s.pitch, s.yaw, s.vbat);
                        first_sample_seen.store(true, std::memory_order_release);
                        rate_start = t_steady;
                    } else if (log_samples % kRateWindow == 0) {
                        const auto window =
                            std::chrono::duration<double>(t_steady - *rate_start).count();
                        const double hz = (window > 0.0) ? kRateWindow / window : 0.0;
                        cfo::console::info(
                            "state stream: {:.1f} Hz over last {} samples",
                            hz, kRateWindow);
                        rate_start = t_steady;
                    }
                    break;
                }

                case cfo::PacketKind::LogTocReply:
                case cfo::PacketKind::LogSettingsAck:
                case cfo::PacketKind::LinkControl:
                case cfo::PacketKind::Unknown:
                    // raw already logged; no decode required for this slice
                    break;
            }
        }
        cfo::console::info("RX stopping ({} log samples decoded)", log_samples);
    }};

    // Watchdog: warn if the firmware never sends a sample within 3 s. This
    // catches "everything looked successful but no data is flowing" — e.g.,
    // block was created and started but the firmware silently dropped it.
    {
        using namespace std::chrono_literals;
        const auto deadline = std::chrono::steady_clock::now() + 3s;
        while (std::chrono::steady_clock::now() < deadline) {
            if (first_sample_seen.load(std::memory_order_acquire)) break;
            if (g_shutdown.load(std::memory_order_relaxed)) break;
            std::this_thread::sleep_for(50ms);
        }
        if (!first_sample_seen.load(std::memory_order_acquire) &&
            !g_shutdown.load(std::memory_order_relaxed)) {
            cfo::console::warn("no log sample within 3s of start");
        }
    }

    cfo::console::info("running — Ctrl-C to stop");
    rx_thread.join();

    logger.log(make_link_event(cfo::LinkState::Closed, cfg.crazyflie_uri));
    cfo::console::info("link closed");

    logger.close();
    if (auto dropped = logger.dropped_count(); dropped > 0) {
        cfo::console::warn("dropped {} log events (queue full)", dropped);
    }
    cfo::console::info("cf-offboard exited");
    return 0;
}
