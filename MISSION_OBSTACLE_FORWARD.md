# MISSION_OBSTACLE_FORWARD

## Purpose

Define the next mission target after the first assisted flight.

This mission uses Multiranger-based local obstacle awareness to validate:
- range sensing
- local obstacle representation
- obstacle-aware mission logic
- Rerun spatial visualization
- safe mission behavior under simple environmental constraints

---

## Mission Goal

A bounded forward mission with conservative obstacle handling.

Nominal sequence:
1. take off
2. move forward toward a bounded target
3. monitor forward and side obstacle state continuously
4. if path remains clear, continue
5. if obstacle is detected, react conservatively
6. complete or fail safely
7. land

---

## Scope

This is not path planning.
This is not mapping-driven autonomy.
This is not dynamic obstacle prediction.

The mission should remain small and explicit.

---

## Recommended First Behavior

For the first version, obstacle handling should be conservative.

Preferred behavior:
- if the path is clear, continue
- if a forward obstacle is too close, stop and hover
- then abort or land safely

Optional later behavior:
- sidestep left or right using simple rules
- continue after obstacle clears

Do not start with autonomous rerouting.

---

## Inputs

The mission may use:
- vehicle state
- current mission clock
- Multiranger obstacle summary
- maybe projected obstacle points for debugging only

The mission should not directly read raw transport packets.

---

## Mission States

Suggested states:
- `Idle`
- `TakingOff`
- `HoverStabilizing`
- `ForwardProgress`
- `ObstacleHold`
- `Landing`
- `Completed`
- `Aborted`

State transitions must be explicit and testable.

---

## Obstacle Policy

Define simple obstacle zones based on forward range.

Example:
- clear: safe to continue
- caution: may slow or prepare to stop later
- blocked: stop immediately and hold

For the first version, it is acceptable to use only:
- clear
- blocked

This is enough to validate the full loop.

---

## Command Model

Prefer HLC-based movement for this mission as well.

Examples:
- HLC takeoff
- bounded HLC go-to segments
- HLC land

A practical approach is to break forward movement into small bounded steps.
Before each step:
- check obstacle state
- if clear, issue next step
- if blocked, do not issue the next step

This keeps behavior explicit and easy to debug.

---

## Logging Requirements

The mission must log:
- mission start
- state transitions
- obstacle state changes
- stop/abort decisions caused by obstacles
- commanded forward steps
- mission completion or abort

These events should appear in:
- TUI event area
- MCAP
- Rerun annotations where useful

---

## Rerun Expectations

Rerun should show:
- actual path
- commanded path
- current obstacle rays
- projected obstacle points
- mission state changes
- goal or step markers if convenient

This mission is one of the main reasons to add Rerun now.

---

## Success Criteria

A successful first obstacle-forward mission means:
- vehicle takes off
- vehicle makes bounded forward progress while path is clear
- vehicle stops or aborts safely when an obstacle is too close
- obstacle state is visible in TUI
- obstacle geometry is visible in Rerun
- logs are sufficient to replay and analyze the run

---

## Non-Goals

Do not implement:
- full path planning
- map optimization
- dynamic avoidance policy
- obstacle classification
- generic behavior tree
- mission scripting language

Keep the mission small and conservative.

---

## Practical Rule

If a simpler stop-and-hold behavior works, prefer it over a clever avoidance behavior.

The first job of this mission is to prove the sensing-to-action loop, not to impress.
