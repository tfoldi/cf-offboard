#pragma once

#include "control/types.hpp"
#include "crazyflie/link/interfaces.hpp"
#include "logging/mcap_logger.hpp"
#include "mission/types.hpp"
#include "perception/types.hpp"
#include "safety/types.hpp"
#include "state/state_store.hpp"
#include "ui/types.hpp"

#include <atomic>
#include <functional>

namespace cfo {

// Run the control loop on the calling thread. Drives the mission state
// machine (mission/mission.hpp) at `cfg.period`, translating each emitted
// MissionCommand into a CRTP packet on the link.
//
// The loop exits when:
//   - mission_tick reports terminate=true (Completed or fully Aborted), or
//   - `shutdown_requested` becomes true (operator Ctrl-C — surfaces into
//     the mission machine as MissionTickInput::operator_shutdown).
//
// All side effects (link send, logging, console) happen at this boundary.
// Trajectory math, safety checks, and state transitions are pure.
//
// `on_state_change` (optional) is invoked from the control thread on every
// mission_tick state transition — used by the app to push the change into
// AppStatusStore so the TUI sees it. May be nullptr.
using MissionStateCallback =
    std::function<void(MissionState, AbortReason)>;

void run_control_loop(ICrazyflieLink& link,
                      const StateStore& state,
                      const AppStatusStore& app_status,
                      MCAPLogger& logger,
                      const ControlLoopConfig& cfg,
                      std::atomic<bool>& shutdown_requested,
                      MissionStateCallback on_state_change = nullptr);

} // namespace cfo
