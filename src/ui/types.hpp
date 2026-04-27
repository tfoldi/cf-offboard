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
// recording status, current mode + sub-state, and the URI/path strings.
// Updated by main / control / manual / RX threads.
// ---------------------------------------------------------------------------

// Top-level operating mode. Only one is active at a time. The split is
// explicit per the design intent: scripted mission = HLC; manual = HLC
// (this slice); future low-level paths (gamepad, attitude rate, etc.)
// live alongside but never inside the Mission or ManualHlc paths.
enum class AppMode : std::uint8_t {
    Idle,        // no mission running, no manual mode active
    Mission,     // mission state machine is driving control
    ManualHlc,   // operator-driven HLC (this slice)
};

[[nodiscard]] inline const char* mode_name(AppMode m) noexcept {
    switch (m) {
        case AppMode::Idle:      return "Idle";
        case AppMode::Mission:   return "Mission";
        case AppMode::ManualHlc: return "ManualHlc";
    }
    return "Unknown";
}

// Sub-state within the ManualHlc mode. Distinct from MissionState
// because the lifecycle is operator-driven, not trajectory-driven.
enum class ManualState : std::uint8_t {
    OnGround,    // motors off, ready for takeoff
    TakingOff,   // HLC TAKEOFF in flight, no movement commands accepted
    Flying,      // HLC holding; movement / land commands accepted
    Landing,     // HLC LAND in flight, then HLC STOP
};

[[nodiscard]] inline const char* manual_state_name(ManualState s) noexcept {
    switch (s) {
        case ManualState::OnGround:  return "OnGround";
        case ManualState::TakingOff: return "TakingOff";
        case ManualState::Flying:    return "Flying";
        case ManualState::Landing:   return "Landing";
    }
    return "Unknown";
}

struct AppStatus {
    bool link_open{false};
    bool log_active{false};
    bool ready_to_fly{false};

    AppMode mode{AppMode::Idle};

    // Mission sub-state (meaningful when mode == Mission).
    bool mission_active{false};
    MissionState mission_state{MissionState::Idle};
    AbortReason  last_abort_reason{AbortReason::None};

    // Manual sub-state (meaningful when mode == ManualHlc).
    ManualState manual_state{ManualState::OnGround};

    std::string uri;
    std::string mcap_path;
};

// Convenience: is the drone currently airborne (motors on)?
[[nodiscard]] inline bool is_airborne(const AppStatus& s) noexcept {
    if (s.mission_active) return true;
    if (s.mode == AppMode::ManualHlc &&
        s.manual_state != ManualState::OnGround) return true;
    return false;
}

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

// Discrete-step direction for ManualHlc movement. The TUI sets this on a
// keypress; the manual control thread polls + clears it (latest-wins).
// X/Y axes are *body-frame* — "forward" is wherever the drone's nose
// points. The control loop rotates by the current yaw before issuing
// world-frame GO_TO commands.
enum class ManualStep : std::uint8_t {
    None = 0,
    XPlus,     // forward (body)
    XMinus,    // back    (body)
    YPlus,     // strafe left  (body)
    YMinus,    // strafe right (body)
    ZPlus,     // up
    ZMinus,    // down
    YawPlus,   // yaw left  (CCW)
    YawMinus,  // yaw right (CW)
};

struct OperatorIntents {
    std::atomic<bool> start_mission{false};
    std::atomic<bool> abort_mission{false};   // also aborts manual flight
    std::atomic<bool> quit{false};

    // ManualHlc.
    std::atomic<bool> enter_manual{false};
    std::atomic<bool> exit_manual{false};
    std::atomic<bool> manual_takeoff{false};
    std::atomic<bool> manual_land{false};
    std::atomic<ManualStep> manual_step{ManualStep::None};
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
