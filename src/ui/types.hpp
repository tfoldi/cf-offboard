#pragma once

#include "mission/types.hpp"

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace cfo {

// ---------------------------------------------------------------------------
// AppStatus — coarse application state visible to the TUI: link health,
// recording status, mission state, and the URI/path strings the operator
// wants to see in the status bar. Updated by main / control / RX threads.
// ---------------------------------------------------------------------------

struct AppStatus {
    bool link_open{false};
    bool log_active{false};
    bool ready_to_fly{false};   // pre-arm + supervisor checks all clear
    bool mission_active{false}; // control loop currently running

    MissionState mission_state{MissionState::Idle};
    AbortReason  last_abort_reason{AbortReason::None};

    std::string uri;
    std::string mcap_path;
};

class AppStatusStore {
public:
    [[nodiscard]] AppStatus snapshot() const {
        std::lock_guard lock{m_};
        return status_;
    }

    template <typename Fn>
    void update(Fn&& fn) {
        std::lock_guard lock{m_};
        std::forward<Fn>(fn)(status_);
    }

private:
    mutable std::mutex m_;
    AppStatus status_{};
};

// ---------------------------------------------------------------------------
// OperatorIntents — typed intents emitted by the TUI, read by main and the
// control loop. Plain atomic flags (latched). Consumers clear them after
// acting (e.g. main consumes start_mission once it spawns the control loop).
// ---------------------------------------------------------------------------

struct OperatorIntents {
    std::atomic<bool> start_mission{false};
    std::atomic<bool> abort_mission{false};
    std::atomic<bool> quit{false};
};

// ---------------------------------------------------------------------------
// EventLog — small ring buffer of recent high-level events for the TUI's
// event panel. Populated by console.cpp (when configured to do so) so any
// console::info/warn/error line shows up here automatically. Bounded to
// avoid unbounded growth on a long-running app.
// ---------------------------------------------------------------------------

struct LogEntry {
    enum class Level : std::uint8_t { Info, Warn, Error };
    Level level{Level::Info};
    std::chrono::system_clock::time_point t{};
    std::string text;
};

class EventLog {
public:
    explicit EventLog(std::size_t capacity = 200) : capacity_{capacity} {}

    void push(LogEntry e) {
        std::lock_guard lock{m_};
        entries_.push_back(std::move(e));
        while (entries_.size() > capacity_) entries_.pop_front();
    }

    // Most recent `n` entries, oldest first.
    [[nodiscard]] std::vector<LogEntry> tail(std::size_t n) const {
        std::lock_guard lock{m_};
        const std::size_t take = std::min(n, entries_.size());
        return std::vector<LogEntry>{
            entries_.end() - static_cast<std::ptrdiff_t>(take),
            entries_.end()};
    }

private:
    mutable std::mutex m_;
    std::deque<LogEntry> entries_;
    std::size_t capacity_;
};

} // namespace cfo
