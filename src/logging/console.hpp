#pragma once

// Human-facing console logging — stderr, timestamped, level-tagged.
//
// Concerns kept deliberately separate from MCAPLogger:
//   - Console: live operator visibility while running (startup, link state,
//     warnings, errors, high-level transitions).
//   - MCAP:    structured events for replay, reproducibility, analysis.
// Some events are written to both at the same call site in app code.
//
// Output is synchronous (stderr is fast); a single mutex serializes lines so
// concurrent threads do not interleave. Color is auto-detected via isatty.

#include <cstdio>
#include <format>
#include <string_view>
#include <utility>

namespace cfo {
class EventLog;  // src/ui/types.hpp
}

namespace cfo::console {

// Configure runtime sinks. By default emit() writes to stderr only. The
// app may additionally feed an EventLog (so the TUI shows console lines)
// and/or silence stderr when the TUI owns the screen.
void set_event_log(cfo::EventLog* log);
void set_stderr_silent(bool silent);

// Mirror every console line to a FILE* (no ANSI colors). Pass nullptr to
// detach. Caller owns the FILE* lifetime; typical use is to fopen at
// startup, set here, and fclose at shutdown after detaching.
void set_log_file(std::FILE* f);

namespace detail {

enum class Level { Info, Warn, Error };

void emit(Level level, std::string_view formatted);

} // namespace detail

template <typename... Args>
void info(std::format_string<Args...> fmt, Args&&... args) {
    detail::emit(detail::Level::Info,
                 std::format(fmt, std::forward<Args>(args)...));
}

template <typename... Args>
void warn(std::format_string<Args...> fmt, Args&&... args) {
    detail::emit(detail::Level::Warn,
                 std::format(fmt, std::forward<Args>(args)...));
}

template <typename... Args>
void error(std::format_string<Args...> fmt, Args&&... args) {
    detail::emit(detail::Level::Error,
                 std::format(fmt, std::forward<Args>(args)...));
}

} // namespace cfo::console
