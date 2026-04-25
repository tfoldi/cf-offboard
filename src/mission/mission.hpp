#pragma once

#include "mission/types.hpp"

namespace cfo {

// Pure: advance the mission state machine by one tick.
//
//   ctx  — current mission state (immutable; the new state is in the output)
//   in   — sensor + supervisor + clock inputs
//   cfg  — trajectory and timing tunables
//
// No IO, no time source — `in.now` is passed in so the same code is used
// in replay. The control loop is responsible for translating the returned
// MissionCommand into a CRTP packet and sending it.
//
// Termination semantics:
//   - `terminate=true` is set when the loop should exit after sending the
//     final command. That happens on completion (Completed state) or after
//     the abort descent dwell (Aborted state with command=Stop).
[[nodiscard]] MissionTickOutput
mission_tick(const MissionContext& ctx,
             const MissionTickInput& in,
             const MissionConfig& cfg);

} // namespace cfo
