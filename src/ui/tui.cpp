#include "ui/tui.hpp"

#include "mission/types.hpp"
#include "safety/types.hpp"
#include "state/types.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <numbers>
#include <string>
#include <string_view>
#include <sys/ioctl.h>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace cfo {

namespace {

// ---- ANSI escape helpers ----------------------------------------------------

constexpr std::string_view kAltScreenOn  = "\033[?1049h";
constexpr std::string_view kAltScreenOff = "\033[?1049l";
constexpr std::string_view kHideCursor   = "\033[?25l";
constexpr std::string_view kShowCursor   = "\033[?25h";
constexpr std::string_view kClearScreen  = "\033[2J";
constexpr std::string_view kCursorHome   = "\033[H";
constexpr std::string_view kClearToEol   = "\033[K";
constexpr std::string_view kReset        = "\033[0m";
constexpr std::string_view kFgGreen      = "\033[32m";
constexpr std::string_view kFgYellow     = "\033[33m";
constexpr std::string_view kFgRed        = "\033[31m";
constexpr std::string_view kFgCyan       = "\033[36m";
constexpr std::string_view kFgGray       = "\033[90m";
constexpr std::string_view kBold         = "\033[1m";

void write_raw(std::string_view s) {
    std::fwrite(s.data(), 1, s.size(), stdout);
}

void cursor_to(int row, int col) {
    char buf[32];
    const int n = std::snprintf(buf, sizeof(buf), "\033[%d;%dH", row, col);
    std::fwrite(buf, 1, static_cast<std::size_t>(n), stdout);
}

// ---- terminal RAII ----------------------------------------------------------

class TerminalGuard {
public:
    TerminalGuard() {
        if (::tcgetattr(STDIN_FILENO, &saved_) != 0) return;
        active_ = true;

        struct termios raw = saved_;
        raw.c_lflag &= ~(ICANON | ECHO | ISIG);
        raw.c_iflag &= ~(ICRNL | IXON);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        // Non-blocking stdin so read() returns immediately when nothing's there.
        const int flags = ::fcntl(STDIN_FILENO, F_GETFL);
        ::fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

        // Drain anything that was sitting in stdin before raw mode kicked
        // in (shell echo, paste buffer, type-ahead). Otherwise the TUI's
        // first poll would interpret it as keystrokes.
        char drain_buf[256];
        while (::read(STDIN_FILENO, drain_buf, sizeof(drain_buf)) > 0) {}

        write_raw(kAltScreenOn);
        write_raw(kHideCursor);
        write_raw(kClearScreen);
        std::fflush(stdout);
    }

    ~TerminalGuard() {
        if (!active_) return;
        write_raw(kShowCursor);
        write_raw(kAltScreenOff);
        write_raw(kReset);
        std::fflush(stdout);
        ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }

    TerminalGuard(const TerminalGuard&) = delete;
    TerminalGuard& operator=(const TerminalGuard&) = delete;

private:
    struct termios saved_{};
    bool active_{false};
};

struct Size {
    int rows{24};
    int cols{80};
};

Size term_size() {
    struct winsize ws{};
    if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 &&
        ws.ws_row > 0 && ws.ws_col > 0) {
        return Size{ws.ws_row, ws.ws_col};
    }
    return Size{};
}

// ---- formatting helpers -----------------------------------------------------

std::string fmt_age_ms(std::chrono::steady_clock::time_point last_update,
                       std::chrono::steady_clock::time_point now) {
    if (last_update.time_since_epoch().count() == 0) return "  --";
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_update).count();
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%4lld ms", static_cast<long long>(age));
    return buf;
}

constexpr float rad_to_deg(float r) {
    return r * 180.0f / static_cast<float>(std::numbers::pi);
}

std::string format_clock(std::chrono::system_clock::time_point tp) {
    const auto t = std::chrono::system_clock::to_time_t(tp);
    std::tm tm{};
    ::localtime_r(&t, &tm);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

std::string_view level_color(LogEntry::Level l) {
    switch (l) {
        case LogEntry::Level::Info:  return kFgCyan;
        case LogEntry::Level::Warn:  return kFgYellow;
        case LogEntry::Level::Error: return kFgRed;
    }
    return {};
}

std::string_view level_tag(LogEntry::Level l) {
    switch (l) {
        case LogEntry::Level::Info:  return "INFO ";
        case LogEntry::Level::Warn:  return "WARN ";
        case LogEntry::Level::Error: return "ERROR";
    }
    return "?    ";
}

const char* abort_str(AbortReason r) {
    switch (r) {
        case AbortReason::None:           return "none";
        case AbortReason::TelemetryStale: return "telemetry_stale";
        case AbortReason::LowBattery:     return "low_battery";
        case AbortReason::LinkLost:       return "link_lost";
        case AbortReason::OperatorAbort:  return "operator_abort";
        case AbortReason::MissionTimeout: return "mission_timeout";
    }
    return "?";
}

// ---- per-section painters ---------------------------------------------------

void paint_status(const AppStatus& s, int& row) {
    cursor_to(row++, 1);
    std::printf("%s== cf-offboard ==%s%s", kBold.data(), kReset.data(),
                kClearToEol.data());

    cursor_to(row++, 1);
    std::printf(" link:    %s%s%s%s",
                s.link_open ? kFgGreen.data() : kFgRed.data(),
                s.link_open ? "connected" : "disconnected",
                kReset.data(), kClearToEol.data());

    cursor_to(row++, 1);
    std::printf(" uri:     %s%s", s.uri.c_str(), kClearToEol.data());

    cursor_to(row++, 1);
    std::printf(" mcap:    %s%s%s%s",
                s.log_active ? kFgGreen.data() : kFgGray.data(),
                s.mcap_path.c_str(), kReset.data(), kClearToEol.data());

    cursor_to(row++, 1);
    const char* mode_color =
        (s.mode == AppMode::Mission)   ? kFgYellow.data() :
        (s.mode == AppMode::ManualHlc) ? kFgCyan.data()   : kFgGray.data();
    if (s.mode == AppMode::Mission) {
        std::printf(" mode:    %s%s%s — %s%s",
                    mode_color, mode_name(s.mode), kReset.data(),
                    state_name(s.mission_state),
                    kClearToEol.data());
    } else if (s.mode == AppMode::ManualHlc) {
        std::printf(" mode:    %s%s%s — %s%s",
                    mode_color, mode_name(s.mode), kReset.data(),
                    manual_state_name(s.manual_state),
                    kClearToEol.data());
    } else {
        std::printf(" mode:    %s%s%s%s",
                    mode_color, mode_name(s.mode), kReset.data(),
                    kClearToEol.data());
    }

    if (s.last_abort_reason != AbortReason::None) {
        cursor_to(row++, 1);
        std::printf(" last:    %s%s%s%s",
                    kFgRed.data(), abort_str(s.last_abort_reason),
                    kReset.data(), kClearToEol.data());
    }

    cursor_to(row++, 1);
    const char* ready_color =
        s.ready_to_fly ? kFgGreen.data() : kFgYellow.data();
    const char* ready_label;
    if (!s.ready_to_fly) {
        ready_label = "preparing…";
    } else if (s.mission_active) {
        ready_label = "mission active";
    } else if (s.mode == AppMode::ManualHlc) {
        ready_label = "manual mode";
    } else {
        ready_label = "ready — [s] mission, [m] manual";
    }
    std::printf(" ready:   %s%s%s%s",
                ready_color, ready_label, kReset.data(), kClearToEol.data());
}

// 3×7 instrument-style top-down quad glyph. State drives the centre
// character; motors are 'o' when telemetry is fresh, '.' otherwise. No
// animation — appearance changes only when the underlying values do.
//
//   center: + not ready, X armed/idle, * flying-level, ^ pitch up,
//           v pitch down, < roll left, > roll right, = completed, ! aborted
//
// Caller picks the (row, col) so the glyph slots into the vehicle panel.
void paint_glyph(const VehicleState& v, const AppStatus& s,
                 int row, int col) {
    char center;
    if (s.mode == AppMode::Mission &&
        s.mission_state == MissionState::Aborted) {
        center = '!';
    } else if (s.mode == AppMode::Mission &&
               s.mission_state == MissionState::Completed) {
        center = '=';
    } else if (is_airborne(s)) {
        // While flying — manual or mission — encode dominant attitude.
        const float roll  = rad_to_deg(v.attitude.roll);
        const float pitch = rad_to_deg(v.attitude.pitch);
        constexpr float kThr = 5.0f;
        if (std::fabs(roll) > std::fabs(pitch)) {
            if      (roll >  kThr) center = '>';
            else if (roll < -kThr) center = '<';
            else                   center = '*';
        } else {
            if      (pitch >  kThr) center = '^';
            else if (pitch < -kThr) center = 'v';
            else                    center = '*';
        }
    } else if (s.ready_to_fly) {
        center = 'X';
    } else {
        center = '+';
    }

    const bool fresh = v.update_count > 0;
    const char m = fresh ? 'o' : '.';

    cursor_to(row,     col);
    std::printf(" %c\\ /%c %s", m, m, kClearToEol.data());
    cursor_to(row + 1, col);
    std::printf("   %c   %s", center, kClearToEol.data());
    cursor_to(row + 2, col);
    std::printf(" %c/ \\%c %s", m, m, kClearToEol.data());
}

void paint_vehicle(const VehicleState& v, int& row) {
    cursor_to(row++, 1);
    std::printf("%s-- vehicle state --%s%s", kBold.data(), kReset.data(),
                kClearToEol.data());

    cursor_to(row++, 1);
    std::printf("  pos    x: %+7.3f   y: %+7.3f   z: %+7.3f%s",
                v.position.x, v.position.y, v.position.z,
                kClearToEol.data());

    cursor_to(row++, 1);
    std::printf("  vel    x: %+7.3f   y: %+7.3f   z: %+7.3f%s",
                v.velocity.x, v.velocity.y, v.velocity.z,
                kClearToEol.data());

    cursor_to(row++, 1);
    std::printf("  att    roll: %+6.1f°   pitch: %+6.1f°   yaw: %+6.1f°%s",
                rad_to_deg(v.attitude.roll),
                rad_to_deg(v.attitude.pitch),
                rad_to_deg(v.attitude.yaw),
                kClearToEol.data());

    cursor_to(row++, 1);
    const char* vcolor = (v.battery_voltage > 0.0f && v.battery_voltage < 3.30f)
                             ? kFgYellow.data()
                             : kFgGreen.data();
    std::printf("  vbat   %s%.2f V%s%s", vcolor, v.battery_voltage,
                kReset.data(), kClearToEol.data());

    cursor_to(row++, 1);
    std::printf("  fresh  %s%s",
                fmt_age_ms(v.last_update, std::chrono::steady_clock::now()).c_str(),
                kClearToEol.data());
}

// One row showing front/back/left/right/up + obstacle status. Greyed
// out and labelled "no deck" when perception isn't active.
void paint_perception(const AppStatus& s, int& row) {
    cursor_to(row++, 1);
    std::printf("%s-- perception --%s%s",
                kBold.data(), kReset.data(), kClearToEol.data());

    cursor_to(row++, 1);
    if (!s.perception_active) {
        std::printf("  %sno multiranger deck%s%s",
                    kFgGray.data(), kReset.data(), kClearToEol.data());
        return;
    }

    auto rng = [](float m, bool valid) -> std::string {
        if (!valid) return "  --m";
        char b[16];
        std::snprintf(b, sizeof(b), "%5.2fm", m);
        return std::string{b};
    };

    std::printf(
        "  front:%s  back:%s  left:%s  right:%s  up:%s%s",
        rng(s.front_m, s.valid_front).c_str(),
        rng(s.back_m,  s.valid_back).c_str(),
        rng(s.left_m,  s.valid_left).c_str(),
        rng(s.right_m, s.valid_right).c_str(),
        rng(s.up_m,    s.valid_up).c_str(),
        kClearToEol.data());

    cursor_to(row++, 1);
    const char* color =
        (s.obstacle_status == ObstacleStatus::Blocked) ? kFgRed.data()  :
        (s.obstacle_status == ObstacleStatus::Caution) ? kFgYellow.data() :
                                                          kFgGreen.data();
    std::printf("  status:  %s%s%s%s",
                color, obstacle_status_name(s.obstacle_status),
                kReset.data(), kClearToEol.data());
}

void paint_events(const std::vector<LogEntry>& entries, int start_row,
                  int rows_available, int cols) {
    cursor_to(start_row, 1);
    std::printf("%s-- events --%s%s", kBold.data(), kReset.data(),
                kClearToEol.data());

    const int slots = std::max(0, rows_available - 1);
    const int show = std::min<int>(slots, static_cast<int>(entries.size()));
    const int pad = slots - show;

    int row = start_row + 1 + pad;
    for (int i = static_cast<int>(entries.size()) - show;
         i < static_cast<int>(entries.size()); ++i) {
        const auto& e = entries[static_cast<std::size_t>(i)];
        cursor_to(row, 1);
        const auto color = level_color(e.level);
        const auto tag = level_tag(e.level);
        // Truncate text to terminal width (timestamp + tag + space + ...).
        const int prefix = 8 /*HH:MM:SS*/ + 1 + 5 /*level*/ + 2;
        const int max_text = std::max(0, cols - prefix - 1);
        std::string text = e.text;
        if (static_cast<int>(text.size()) > max_text) {
            text.resize(static_cast<std::size_t>(max_text));
        }
        std::printf(" %s %s%s%s %s%s",
                    format_clock(e.t).c_str(),
                    color.data(), tag.data(), kReset.data(),
                    text.c_str(), kClearToEol.data());
        ++row;
    }
    // Clear leftover slots above the visible entries.
    for (int r = start_row + 1; r < start_row + 1 + pad; ++r) {
        cursor_to(r, 1);
        std::fputs(kClearToEol.data(), stdout);
    }
}

void paint_footer(const AppStatus& s, int row) {
    cursor_to(row, 1);
    if (s.mission_active) {
        std::printf("%s[a]%s abort   %s[q]%s quit (refused while flying)%s",
                    kBold.data(), kReset.data(), kFgGray.data(), kReset.data(),
                    kClearToEol.data());
        return;
    }
    if (s.mode == AppMode::ManualHlc) {
        switch (s.manual_state) {
            case ManualState::OnGround:
                std::printf("%s[t]%s takeoff   %s[m]%s exit manual   "
                            "%s[q]%s quit%s",
                            kBold.data(), kReset.data(),
                            kBold.data(), kReset.data(),
                            kBold.data(), kReset.data(),
                            kClearToEol.data());
                break;
            case ManualState::TakingOff:
                std::printf("%staking off…%s   %s[a]%s abort%s",
                            kFgYellow.data(), kReset.data(),
                            kBold.data(), kReset.data(),
                            kClearToEol.data());
                break;
            case ManualState::Flying:
                std::printf(
                    "%sw/a/s/d%s move   %s[q/e]%s yaw   %s[r/f]%s up/down"
                    "   %s[l]%s land   %s[x]%s abort   %s[m]%s exit%s",
                    kBold.data(), kReset.data(),
                    kBold.data(), kReset.data(),
                    kBold.data(), kReset.data(),
                    kBold.data(), kReset.data(),
                    kBold.data(), kReset.data(),
                    kBold.data(), kReset.data(),
                    kClearToEol.data());
                break;
            case ManualState::Landing:
                std::printf("%slanding…%s%s",
                            kFgYellow.data(), kReset.data(),
                            kClearToEol.data());
                break;
        }
        return;
    }
    if (s.ready_to_fly) {
        const char* hint =
            (s.mission_state == MissionState::Completed) ? " (new)"  :
            (s.mission_state == MissionState::Aborted)   ? " (retry)":
            "";
        std::printf("%s[s]%s start%s   %s[m]%s manual mode   "
                    "%s[q]%s quit%s",
                    kBold.data(), kReset.data(), hint,
                    kBold.data(), kReset.data(),
                    kBold.data(), kReset.data(),
                    kClearToEol.data());
        return;
    }
    std::printf("%s[q]%s quit (forces shutdown)%s",
                kBold.data(), kReset.data(), kClearToEol.data());
}

// ---- input handling ---------------------------------------------------------

// Direction step intents — only fire while in ManualHlc.Flying.
void try_step(OperatorIntents& intents, const AppStatus& s,
              ManualStep step) {
    if (s.mode == AppMode::ManualHlc &&
        s.manual_state == ManualState::Flying) {
        intents.manual_step.store(step, std::memory_order_release);
    }
}

void handle_keys(OperatorIntents& intents, AppStatusStore& mutable_status,
                 std::atomic<bool>& shutdown) {
    char buf[32];
    while (true) {
        const ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
        if (n <= 0) return;

        for (ssize_t i = 0; i < n; ) {
            const auto s = mutable_status.snapshot();

            // Arrow key escape sequences: ESC '[' (A|B|C|D).
            if (i + 2 < n && buf[i] == 0x1B && buf[i + 1] == '[') {
                const char arrow = buf[i + 2];
                switch (arrow) {
                    case 'A': try_step(intents, s, ManualStep::XPlus);  break;
                    case 'B': try_step(intents, s, ManualStep::XMinus); break;
                    case 'D': try_step(intents, s, ManualStep::YPlus);  break;
                    case 'C': try_step(intents, s, ManualStep::YMinus); break;
                    default: break;
                }
                i += 3;
                continue;
            }

            const char c = buf[i++];
            const bool airborne = is_airborne(s);
            const bool manual_flying =
                s.mode == AppMode::ManualHlc &&
                s.manual_state == ManualState::Flying;

            // Ctrl-C: always usable — quit when safe, abort mid-flight.
            if (c == 0x03) {
                if (!airborne) {
                    intents.quit.store(true, std::memory_order_release);
                    shutdown.store(true, std::memory_order_release);
                } else {
                    intents.abort_mission.store(
                        true, std::memory_order_release);
                }
                continue;
            }

            // Letter keys are mode-overloaded: most chars mean one thing
            // on the ground (mission start, quit, etc.) and movement once
            // in manual flight. The footer reflects the active mapping.
            if (manual_flying) {
                    // FPS-style movement bindings while manually flying.
                    switch (c) {
                        case 'w': case 'W':
                            try_step(intents, s, ManualStep::XPlus);  break;
                        case 's': case 'S':
                            try_step(intents, s, ManualStep::XMinus); break;
                        case 'a': case 'A':
                            try_step(intents, s, ManualStep::YPlus);  break;
                        case 'd': case 'D':
                            try_step(intents, s, ManualStep::YMinus); break;
                        case 'r': case 'R':
                            try_step(intents, s, ManualStep::ZPlus);  break;
                        case 'f': case 'F':
                            try_step(intents, s, ManualStep::ZMinus); break;
                        case 'q': case 'Q':
                            try_step(intents, s, ManualStep::YawPlus);  break;
                        case 'e': case 'E':
                            try_step(intents, s, ManualStep::YawMinus); break;
                        case 'l': case 'L':
                            intents.manual_land.store(
                                true, std::memory_order_release);
                            break;
                        case 'm': case 'M':
                            // Exit manual: auto-lands first if airborne.
                            intents.exit_manual.store(
                                true, std::memory_order_release);
                            break;
                        case 'x': case 'X':   // 'x' = abort while flying
                            intents.abort_mission.store(
                                true, std::memory_order_release);
                            break;
                        default: break;
                    }
                    continue;
                }

                // Non-flying letter bindings.
                switch (c) {
                    case 'q': case 'Q':
                        if (!airborne) {
                            intents.quit.store(true, std::memory_order_release);
                            shutdown.store(true, std::memory_order_release);
                        }
                        break;
                    case 's': case 'S':
                        if (s.mode == AppMode::Idle && s.ready_to_fly) {
                            intents.start_mission.store(
                                true, std::memory_order_release);
                        }
                        break;
                    case 'm': case 'M':
                        if (s.mode == AppMode::Idle && s.ready_to_fly) {
                            intents.enter_manual.store(
                                true, std::memory_order_release);
                        } else if (s.mode == AppMode::ManualHlc) {
                            // ManualHlc.OnGround / TakingOff / Landing —
                            // 'm' requests exit (manual loop will auto-
                            // land first if airborne).
                            intents.exit_manual.store(
                                true, std::memory_order_release);
                        }
                        break;
                    case 't': case 'T':
                        if (s.mode == AppMode::ManualHlc &&
                            s.manual_state == ManualState::OnGround) {
                            intents.manual_takeoff.store(
                                true, std::memory_order_release);
                        }
                        break;
                    case 'a': case 'A':
                        // Abort in mission flying. (Manual flying handled
                        // above; manual on ground has nothing to abort.)
                        if (s.mission_active) {
                            intents.abort_mission.store(
                                true, std::memory_order_release);
                        }
                        break;
                    default: break;
            }
        }
    }
}

} // namespace

void run_tui(const StateStore& state,
             const AppStatusStore& status,
             const EventLog& events,
             OperatorIntents& intents,
             std::atomic<bool>& shutdown) {
    TerminalGuard guard;

    // Main thread reads status; key handling consults it as well. We need a
    // mutable reference to call snapshot(), but no writes happen here.
    auto& mutable_status = const_cast<AppStatusStore&>(status);

    using clock = std::chrono::steady_clock;
    constexpr auto kFrame = std::chrono::milliseconds{100};

    while (!shutdown.load(std::memory_order_acquire)) {
        const auto frame_start = clock::now();

        handle_keys(intents, mutable_status, shutdown);

        const auto sz = term_size();
        const auto s  = mutable_status.snapshot();
        const auto v  = state.snapshot();
        const auto ev = events.tail(50);

        std::fputs(kCursorHome.data(), stdout);

        int row = 1;
        paint_status(s, row);

        ++row;  // blank separator
        const int vehicle_top = row;
        paint_vehicle(v, row);

        // Slot the 3-line instrument glyph next to the vehicle numbers if
        // the terminal is wide enough. The glyph aligns with the vel /
        // att / fresh rows (offsets 2..4 from the panel header).
        if (sz.cols >= 70) {
            paint_glyph(v, s, vehicle_top + 2, 60);
        }

        ++row;  // blank separator
        paint_perception(s, row);

        ++row;  // blank separator
        const int events_top = row;
        const int footer_row = sz.rows;
        const int events_rows = std::max(2, footer_row - events_top - 1);
        paint_events(ev, events_top, events_rows, sz.cols);

        paint_footer(s, footer_row);
        std::fflush(stdout);

        std::this_thread::sleep_until(frame_start + kFrame);
    }
}

} // namespace cfo
