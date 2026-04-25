#pragma once

#include "control/types.hpp"
#include "crazyflie/link/interfaces.hpp"
#include "logging/mcap_logger.hpp"
#include "state/state_store.hpp"

#include <atomic>

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
void run_control_loop(ICrazyflieLink& link,
                      const StateStore& state,
                      MCAPLogger& logger,
                      const ControlLoopConfig& cfg,
                      std::atomic<bool>& shutdown_requested);

} // namespace cfo
