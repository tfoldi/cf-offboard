#include "logging/console.hpp"

#include "ui/types.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <string>
#include <unistd.h>

namespace cfo::console {

namespace {
std::atomic<cfo::EventLog*> g_event_log{nullptr};
std::atomic<bool>           g_stderr_silent{false};
std::atomic<std::FILE*>     g_log_file{nullptr};
}

void set_event_log(cfo::EventLog* log) {
    g_event_log.store(log, std::memory_order_release);
}

void set_stderr_silent(bool silent) {
    g_stderr_silent.store(silent, std::memory_order_release);
}

void set_log_file(std::FILE* f) {
    g_log_file.store(f, std::memory_order_release);
}

} // namespace cfo::console

namespace cfo::console::detail {

namespace {

std::mutex g_mutex;

// Color detection: respects the NO_COLOR convention (https://no-color.org)
// and only enables ANSI sequences when stderr is an interactive terminal.
bool color_enabled() {
    static const bool enabled = [] {
        if (std::getenv("NO_COLOR") != nullptr) return false;
        return ::isatty(::fileno(stderr)) != 0;
    }();
    return enabled;
}

const char* level_tag(Level l) {
    switch (l) {
        case Level::Info:  return "INFO ";
        case Level::Warn:  return "WARN ";
        case Level::Error: return "ERROR";
    }
    return "?    ";
}

const char* level_color(Level l) {
    switch (l) {
        case Level::Info:  return "\033[36m";  // cyan
        case Level::Warn:  return "\033[33m";  // yellow
        case Level::Error: return "\033[31m";  // red
    }
    return "";
}

constexpr const char* kReset = "\033[0m";

} // namespace

void emit(Level level, std::string_view formatted) {
    using namespace std::chrono;
    const auto now = system_clock::now();

    // Mirror to event log if configured (TUI consumes from there).
    if (auto* log = ::cfo::console::g_event_log.load(std::memory_order_acquire)) {
        ::cfo::LogEntry e{};
        switch (level) {
            case Level::Info:  e.level = ::cfo::LogEntry::Level::Info;  break;
            case Level::Warn:  e.level = ::cfo::LogEntry::Level::Warn;  break;
            case Level::Error: e.level = ::cfo::LogEntry::Level::Error; break;
        }
        e.t = now;
        e.text.assign(formatted.data(), formatted.size());
        log->push(std::move(e));
    }

    const bool to_stderr =
        !::cfo::console::g_stderr_silent.load(std::memory_order_acquire);
    auto* logf = ::cfo::console::g_log_file.load(std::memory_order_acquire);
    if (!to_stderr && !logf) return;

    const auto t = system_clock::to_time_t(now);
    const auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    std::tm tm{};
    ::localtime_r(&t, &tm);

    char ts[24];
    std::snprintf(ts, sizeof(ts),
                  "%04d-%02d-%02d %02d:%02d:%02d.%03d",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec,
                  static_cast<int>(ms.count()));

    std::lock_guard lock{g_mutex};

    if (to_stderr) {
        const bool color = color_enabled();
        const char* col = color ? level_color(level) : "";
        const char* rst = color ? kReset : "";
        std::fprintf(stderr, "%s %s%s%s %.*s\n",
                     ts, col, level_tag(level), rst,
                     static_cast<int>(formatted.size()), formatted.data());
        std::fflush(stderr);
    }

    if (logf) {
        // No ANSI colors in the file — it's a plain log for grep/diff.
        std::fprintf(logf, "%s %s %.*s\n",
                     ts, level_tag(level),
                     static_cast<int>(formatted.size()), formatted.data());
        std::fflush(logf);
    }
}

} // namespace cfo::console::detail
