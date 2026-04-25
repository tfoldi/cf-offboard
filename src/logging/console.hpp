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

#include <format>
#include <string_view>
#include <utility>

namespace cfo::console {

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
