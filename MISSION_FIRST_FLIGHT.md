# MISSION_FIRST_FLIGHT

## Purpose

Define the first meaningful assisted-flight mission for cf-offboard.

This mission is intentionally small but real:

- take off
- go forward
- land

It exists to validate:
- command path
- mission sequencing
- safety supervision
- operator workflow
- logging and replay

---

## Context

Hardware assumptions:
- Crazyflie 2.x
- Flow deck available
- assisted flight mode available
- Crazyradio for control and telemetry
- laptop-side app provides mission logic

The low-level stabilization remains onboard.

The laptop-side application is responsible for:
- mission sequencing
- command generation
- operator interaction
- safety checks
- logging

---

## Mission Goal

The first mission should execute the following sequence:

1. operator starts mission
2. drone takes off to a modest target hover height
3. drone moves forward in a controlled way
4. drone lands safely
5. mission completes cleanly or aborts safely

This is not full autonomy.
This is a bounded assisted-flight scenario.

---

## Mission States

Use explicit mission states.

Minimum set:
- `Idle`
- `TakingOff`
- `HoverStabilizing`
- `ForwardSegment`
- `PreLandHover`
- `Landing`
- `Completed`
- `Aborted`

State transitions must be explicit and testable.

Do not introduce a generic mission framework.

---

## Command Model

Use a simple high-level command model aligned with assisted flight capabilities.

Examples:
- hover target height
- forward velocity command
- land command or controlled descent

Do not start with laptop-side low-level attitude-rate control.

The mission should operate through the same command path that later manual control and other missions will use.

---

## Initial Mission Profile

Recommended first profile:
- take off to `0.3–0.4 m`
- hold briefly to stabilize, for example `1–2 s`
- move forward slowly for a short bounded interval
- hold briefly
- land

For the first implementation, prefer a **time-based** forward segment over a distance-based segment.

Example:
- forward velocity: low and conservative
- forward duration: `1–2 s`
- hard timeout always enforced

This is simpler and less dependent on estimator quality.

---

## Safety Requirements

Mission execution must check:
- telemetry freshness
- link health
- battery threshold
- mission timeout
- operator abort
- emergency stop

Safety must override mission output.

Any of the following should abort or safely terminate the mission:
- stale telemetry
- link loss
- low battery
- command rejection
- operator abort
- mission timeout exceeded

---

## Operator Interaction

The mission should be startable and visible from the TUI.

The operator must be able to:
- select the mission
- start the mission
- abort the mission
- trigger emergency stop

The operator should always be able to see:
- current mission state
- current flight mode
- warnings
- whether the mission is active, completed, or aborted

---

## Logging Requirements

The mission must log:
- mission start
- each mission state transition
- mission completion or abort
- commands sent during the mission
- safety interventions
- relevant vehicle state snapshots

These events should appear in:
- console or TUI high-level event area
- MCAP structured log

---

## Success Criteria

A successful first mission means:

- operator starts the mission from the app
- drone takes off to a modest height
- drone moves forward in a controlled way
- drone lands safely
- mission states are visible live
- mission events are recorded to MCAP
- the mission can be replayed and analyzed afterward

---

## Non-Goals

Do not implement:
- path planning
- mapping-driven behavior
- behavior trees
- mission scripting language
- generic autonomy stack
- multi-mission orchestration framework

The goal is one clean first mission.

---

## Future Extensions

Later, this mission framework may grow to include:
- distance-based movement
- square or circle routes
- camera-assisted behaviors
- ToF or mapping-assisted behaviors

But the first implementation should remain narrow and explicit.

---

## Rule of Thumb

If implementing this mission requires a large abstraction layer, the design is too broad.

Build only what is required for:
- take off
- go forward
- land
