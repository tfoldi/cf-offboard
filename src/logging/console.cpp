#include "logging/console.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <mutex>
#include <unistd.h>

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

    const bool color = color_enabled();
    const char* col = color ? level_color(level) : "";
    const char* rst = color ? kReset : "";

    std::lock_guard lock{g_mutex};
    std::fprintf(stderr, "%s %s%s%s %.*s\n",
                 ts, col, level_tag(level), rst,
                 static_cast<int>(formatted.size()), formatted.data());
    std::fflush(stderr);
}

} // namespace cfo::console::detail
