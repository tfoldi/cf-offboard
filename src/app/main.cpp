#include "app/config.hpp"
#include "control/control_loop.hpp"
#include "control/types.hpp"
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
#include "ui/tui.hpp"
#include "ui/types.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <unistd.h>
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

struct CliArgs {
    std::optional<std::string> uri;
    std::optional<std::string> mcap_path;
    bool force_headless{false};
};

CliArgs parse_args(int argc, char** argv) {
    CliArgs a;
    for (int i = 1; i < argc; ++i) {
        std::string_view s = argv[i];
        if (s == "--headless") {
            a.force_headless = true;
        } else if (!a.uri) {
            a.uri = std::string{s};
        } else if (!a.mcap_path) {
            a.mcap_path = std::string{s};
        }
    }
    return a;
}

} // namespace

int main(int argc, char** argv) {
    const auto cli = parse_args(argc, argv);

    cfo::AppConfig cfg;
    if (cli.uri)       cfg.crazyflie_uri = *cli.uri;
    if (cli.mcap_path) cfg.mcap_path     = *cli.mcap_path;

    const bool tui_mode =
        !cli.force_headless && ::isatty(STDIN_FILENO) != 0 &&
        ::isatty(STDOUT_FILENO) != 0;

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    cfo::EventLog        events;
    cfo::AppStatusStore  app_status;
    cfo::OperatorIntents intents;

    app_status.update([&](cfo::AppStatus& s) {
        s.uri = cfg.crazyflie_uri;
        s.mcap_path = cfg.mcap_path.string();
    });

    // Wire console: always feed the event log; silence stderr only when the
    // TUI owns the screen.
    cfo::console::set_event_log(&events);
    cfo::console::set_stderr_silent(tui_mode);

    cfo::console::info("cf-offboard starting ({} mode)",
                       tui_mode ? "TUI" : "headless");
    cfo::console::info("config: uri={} rx_poll={}ms log={}",
                       cfg.crazyflie_uri,
                       cfg.rx_poll_timeout.count(),
                       cfg.mcap_path.string());

    // ----- spawn TUI early so the operator sees boot lines in the panel ----
    cfo::StateStore state;
    std::thread tui_thread;
    if (tui_mode) {
        tui_thread = std::thread{[&] {
            cfo::run_tui(state, app_status, events, intents, g_shutdown);
        }};
    }

    auto bail = [&](int code) {
        // Always make sure the TUI tears down its terminal state before we
        // return; the guard inside run_tui only runs when the loop exits.
        g_shutdown.store(true, std::memory_order_release);
        if (tui_thread.joinable()) tui_thread.join();
        return code;
    };

    auto logger_r = cfo::MCAPLogger::create(cfg.mcap_path);
    if (!logger_r) {
        cfo::console::error("MCAP open failed: {}", cfg.mcap_path.string());
        return bail(1);
    }
    auto& logger = **logger_r;
    app_status.update([](cfo::AppStatus& s) { s.log_active = true; });
    cfo::console::info("MCAP recording started: {}", cfg.mcap_path.string());

    auto link_r = cfo::open_crazyflie_link(cfg.crazyflie_uri);
    if (!link_r) {
        logger.log(make_link_event(cfo::LinkState::Error,
                                   "open_failed: " + cfg.crazyflie_uri));
        cfo::console::error("link open failed: {}", cfg.crazyflie_uri);
        logger.close();
        return bail(2);
    }
    auto& link = **link_r;
    app_status.update([](cfo::AppStatus& s) { s.link_open = true; });
    logger.log(make_link_event(cfo::LinkState::Opened, cfg.crazyflie_uri));
    cfo::console::info("link opened: {}", cfg.crazyflie_uri);

    // Outbound smoke test.
    {
        const auto null_pkt = cfo::make_null_packet();
        const auto t = std::chrono::system_clock::now();
        if (auto r = link.send(null_pkt); r) {
            logger.log(cfo::RawCommandEvent{null_pkt, t});
        } else {
            cfo::console::warn("initial null packet send failed");
        }
    }

    // ----- LOG bring-up ----------------------------------------------------
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
        return bail(3);
    }
    cfo::console::info("LOG: setup complete — telemetry active");

    // ----- supervisor pre-arm sanity check + recover ----------------------
    bool supervisor_ok = true;
    {
        auto raw_passthrough = [&](const cfo::RawPacket& p) {
            const auto t = std::chrono::system_clock::now();
            logger.log(cfo::RawTelemetryEvent{p, t});
        };
        auto report = [&](const cfo::SupervisorState& s, const char* label) {
            cfo::console::info(
                "supervisor: {} — armed={} can_arm={} can_fly={} "
                "tumbled={} locked={} crashed={}",
                label, s.is_armed, s.can_be_armed, s.can_fly,
                s.is_tumbled, s.is_locked, s.is_crashed);
        };

        auto sup_r = cfo::query_supervisor_state(link, raw_passthrough);
        if (sup_r) {
            logger.log(cfo::SupervisorStateEvent{
                *sup_r, std::chrono::system_clock::now()});
            report(*sup_r, "pre-arm");

            if (sup_r->is_locked || sup_r->is_crashed) {
                cfo::console::warn(
                    "supervisor: locked/crashed — sending CMD_RECOVER_SYSTEM");
                const auto rec_pkt = cfo::make_supervisor_recover();
                const auto t_rec = std::chrono::system_clock::now();
                if (auto r = link.send(rec_pkt); r) {
                    logger.log(cfo::RawCommandEvent{rec_pkt, t_rec});
                } else {
                    cfo::console::warn("supervisor: recover send failed");
                }
                auto sup2_r = cfo::query_supervisor_state(link, raw_passthrough);
                if (sup2_r) {
                    logger.log(cfo::SupervisorStateEvent{
                        *sup2_r, std::chrono::system_clock::now()});
                    report(*sup2_r, "post-recover");
                    if (sup2_r->is_locked || sup2_r->is_crashed) {
                        cfo::console::error(
                            "supervisor: still locked after recover — flight disabled");
                        supervisor_ok = false;
                    }
                } else {
                    cfo::console::warn(
                        "supervisor: post-recover query failed ({}) — proceeding",
                        sup2_r.error().detail);
                }
            }
        } else {
            cfo::console::warn(
                "supervisor: pre-arm query failed ({}) — proceeding",
                sup_r.error().detail);
        }
    }

    // ----- RX thread -------------------------------------------------------
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
                    break;
            }
        }
        cfo::console::info("RX stopping ({} log samples decoded)", log_samples);
    }};

    // ----- watchdog: first sample within 3 s ------------------------------
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

    const bool ready_to_fly =
        supervisor_ok &&
        first_sample_seen.load(std::memory_order_acquire) &&
        !g_shutdown.load(std::memory_order_relaxed);

    app_status.update([&](cfo::AppStatus& s) {
        s.ready_to_fly = ready_to_fly;
    });
    if (ready_to_fly) {
        cfo::console::info("ready to fly");
    }

    // ----- mission start: explicit in TUI mode, automatic in headless -----
    bool armed = false;
    std::thread control_thread;
    if (ready_to_fly) {
        if (tui_mode) {
            cfo::console::info("press [s] to start the mission");
            using namespace std::chrono_literals;
            while (!g_shutdown.load(std::memory_order_acquire) &&
                   !intents.start_mission.exchange(false,
                                                    std::memory_order_acq_rel)) {
                std::this_thread::sleep_for(20ms);
            }
        }

        if (!g_shutdown.load(std::memory_order_acquire)) {
            const auto t_arm = std::chrono::system_clock::now();
            const auto arm_modern = cfo::make_arm_request(true);
            const auto arm_legacy = cfo::make_arm_request_legacy(true);
            const bool ok_modern = static_cast<bool>(link.send(arm_modern));
            const bool ok_legacy = static_cast<bool>(link.send(arm_legacy));
            if (ok_modern || ok_legacy) {
                if (ok_modern) logger.log(cfo::RawCommandEvent{arm_modern, t_arm});
                if (ok_legacy) logger.log(cfo::RawCommandEvent{arm_legacy, t_arm});
                cfo::console::info("supervisor: arm requested (modern={}, legacy={})",
                                   ok_modern, ok_legacy);
                armed = true;
            } else {
                cfo::console::error("supervisor: arm send failed — aborting flight");
                g_shutdown.store(true, std::memory_order_release);
            }

            if (armed) {
                cfo::ControlLoopConfig ctrl_cfg{};
                const auto& m = ctrl_cfg.mission;
                cfo::console::info(
                    "mission: takeoff {:.2f}m, fwd {:.2f}m/s "
                    "(takeoff {}ms / hover {}ms / fwd {}ms / preland {}ms / land {}ms)",
                    m.target_height_m, m.forward_velocity_mps,
                    m.takeoff_duration.count(), m.hover_duration.count(),
                    m.forward_duration.count(), m.preland_hover_duration.count(),
                    m.land_duration.count());
                app_status.update([](cfo::AppStatus& s) {
                    s.mission_active = true;
                    s.mission_state  = cfo::MissionState::Idle;
                });

                // `control_shutdown` is the mission-scoped abort signal,
                // distinct from app-scoped `g_shutdown`. mission completion
                // doesn't shut down the app — in TUI mode we still wait
                // for the operator to press 'q'.
                std::atomic<bool> control_shutdown{false};

                control_thread = std::thread{[&] {
                    cfo::run_control_loop(
                        link, state, logger, ctrl_cfg, control_shutdown,
                        [&](cfo::MissionState ms, cfo::AbortReason ar) {
                            app_status.update([&](cfo::AppStatus& s) {
                                s.mission_state = ms;
                                if (ar != cfo::AbortReason::None) {
                                    s.last_abort_reason = ar;
                                }
                            });
                        });
                }};

                // Bridge: operator abort or app-wide shutdown both turn
                // into a mission abort. Polled — finer-grained signaling
                // is overkill for v1.
                std::thread mission_watchdog{[&] {
                    using namespace std::chrono_literals;
                    while (!control_shutdown.load(std::memory_order_acquire)) {
                        if (intents.abort_mission.exchange(
                                false, std::memory_order_acq_rel)) {
                            cfo::console::warn("operator abort requested");
                            control_shutdown.store(
                                true, std::memory_order_release);
                            break;
                        }
                        if (g_shutdown.load(std::memory_order_acquire)) {
                            // App-wide shutdown — also bring down the mission
                            // gracefully (control loop will see this via its
                            // shutdown_requested parameter).
                            control_shutdown.store(
                                true, std::memory_order_release);
                            break;
                        }
                        std::this_thread::sleep_for(20ms);
                    }
                }};

                control_thread.join();
                // Wake the watchdog if it's still spinning (mission
                // completed naturally without an abort).
                control_shutdown.store(true, std::memory_order_release);
                mission_watchdog.join();

                app_status.update([](cfo::AppStatus& s) {
                    s.mission_active = false;
                });
            }
        }
    }

    // Explicit disarm.
    if (armed) {
        const auto t_disarm = std::chrono::system_clock::now();
        const auto disarm_modern = cfo::make_arm_request(false);
        const auto disarm_legacy = cfo::make_arm_request_legacy(false);
        if (auto r = link.send(disarm_modern); r) {
            logger.log(cfo::RawCommandEvent{disarm_modern, t_disarm});
        }
        if (auto r = link.send(disarm_legacy); r) {
            logger.log(cfo::RawCommandEvent{disarm_legacy, t_disarm});
        }
        cfo::console::info("supervisor: disarm requested");
    }

    // ----- TUI mode: wait for explicit quit before tearing down -----------
    if (tui_mode) {
        cfo::console::info("press [q] to exit");
        using namespace std::chrono_literals;
        while (!g_shutdown.load(std::memory_order_acquire) &&
               !intents.quit.load(std::memory_order_acquire)) {
            std::this_thread::sleep_for(50ms);
        }
        // signal RX to wind down so it can be joined cleanly
        g_shutdown.store(true, std::memory_order_release);
    } else {
        // Headless: just exit when we get here.
        g_shutdown.store(true, std::memory_order_release);
    }

    rx_thread.join();
    if (tui_thread.joinable()) tui_thread.join();

    logger.log(make_link_event(cfo::LinkState::Closed, cfg.crazyflie_uri));
    cfo::console::info("link closed");

    logger.close();
    if (auto dropped = logger.dropped_count(); dropped > 0) {
        cfo::console::warn("dropped {} log events (queue full)", dropped);
    }
    cfo::console::info("cf-offboard exited");
    return 0;
}
