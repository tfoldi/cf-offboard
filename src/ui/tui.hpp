#pragma once

#include "state/state_store.hpp"
#include "ui/types.hpp"

#include <atomic>

namespace cfo {

// Runs the terminal UI on the calling thread. Returns when the operator
// requests quit or when `shutdown` is set externally (signal handler).
//
// The TUI is a strict observer of state and a strict source of typed
// intents (per TUI.md):
//   - Reads:  StateStore (vehicle), AppStatusStore (link/mission/log
//             state), EventLog (recent console lines).
//   - Writes: OperatorIntents (start_mission, abort_mission, quit) +
//             the shared shutdown flag on quit-when-safe.
//
// Hand-rolled ANSI rendering (no widget framework). Switches stdin to
// raw mode and uses the alternate screen buffer; restores both on exit.
// Refresh rate is fixed at 10 Hz; full-frame redraw is acceptable.
void run_tui(const StateStore& state,
             const AppStatusStore& status,
             const EventLog& events,
             OperatorIntents& intents,
             std::atomic<bool>& shutdown);

} // namespace cfo
