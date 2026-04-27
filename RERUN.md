# Rerun

## Purpose

Rerun is the live and offline visualization layer for cf-offboard.

It is used for:
- live operator and developer visibility
- spatial debugging
- trajectory inspection
- local obstacle visualization
- later camera and perception visualization
- post-flight visualization from recorded data

Rerun is **not** the canonical persisted log format.

The canonical persisted record remains:
- **MCAP**

Rerun receives the same semantic runtime data as MCAP, but as a visualization sink.

---

## Core Policy

### Canonical persistence

All authoritative runtime recording must go to:
- MCAP

This includes:
- commands
- state
- mission state changes
- safety events
- telemetry
- range measurements
- local obstacle state
- camera metadata later
- perception outputs later

### Rerun role

Rerun is:
- a live visualization sink
- an optional offline visualization target

Rerun is not:
- the source of truth
- the only copy of flight data
- a required dependency for safe flight

If Rerun is disabled, unavailable, or crashes:
- flight behavior must remain unaffected
- MCAP logging must continue

---

## Architectural Model

The app should produce one internal stream of structured events and state snapshots.

Those are then forwarded to:
- console / TUI
- MCAP logger
- Rerun logger

This does **not** imply an internal pub/sub framework.
It should remain explicit and simple.

The same semantic data model should be used across sinks.

---

## Visualization Goals

Rerun should answer questions like:
- Where is the drone?
- What trajectory was commanded?
- What did it actually do?
- What mode or mission state was active?
- What do the Multiranger sensors currently see?
- Where are nearby obstacles in relation to odom and base_link?
- What did later perception detect?

---

## Initial Scope

### First signals to log to Rerun

- vehicle pose
- vehicle attitude
- battery voltage
- mission state
- active high-level command
- recent events as annotations
- commanded path
- actual path
- Multiranger rays
- projected obstacle points in odom

### Near-term additions

- simple local occupancy slice
- AI Deck frame stream
- frame metadata
- AprilTag detections
- target pose
- landing target or goal marker

### Later additions

- ToF / richer mapping outputs
- obstacles / point clouds
- estimated landmarks
- perception confidence overlays

---

## Coordinate Frames

Rerun logging must use explicit frame names.

Minimum frames:
- `world` or `odom`
- `base_link`

Optional near-term:
- `multiranger/front`
- `multiranger/back`
- `multiranger/left`
- `multiranger/right`
- `multiranger/up`

Optional later:
- `camera_link`
- `tag/<id>`
- `goal`
- `landing_target`

### Frame policy

- `odom` should be the root visualization frame for early flight and obstacle work
- vehicle pose should be logged as a transform of `base_link` in `odom`
- sensor rays may be defined in `base_link` or explicit sensor frames
- child frames should be explicit
- do not invent implicit frame relationships in visualization code

### Important note

Rerun frame logging is a visualization concern, not a replacement for an internal TF system.

The application should keep frame ownership explicit in core logic.
Rerun just visualizes those relationships.

---

## Transforms

At minimum, log:
- transform of `base_link` in `odom`

Near-term:
- Multiranger rays anchored in `base_link`
- projected obstacle points in `odom`

Later:
- transform of `camera_link` in `base_link`
- transform of detected targets in `odom`

---

## 3D Geometry

### Drone geometry

Rerun should eventually show a simple 3D representation of the drone.

Preferred options, in order:
1. a lightweight static mesh
2. a simplified procedural geometry
3. URDF-derived mesh if practical

Do not block Rerun integration on perfect robot geometry.

Start simple:
- axis triad
- path traces
- maybe a small bounding box or drone mesh if convenient

### Mapping geometry

For Multiranger-driven local mapping, start with:
- rays
- points
- optionally a simple 2D occupancy slice

Do not jump straight to a full 3D map.

---

## Blueprint

A default Rerun blueprint should be defined for live operation.

Goal:
- make live visualization useful without manual setup every run

### First blueprint should include

- 3D view
  - drone transform / pose
  - commanded path
  - actual path
  - obstacle points
  - sensor rays
  - goal marker if present

- time-series views
  - battery
  - altitude
  - maybe x/y/z position

- text / event panel
  - mission transitions
  - warnings
  - errors

### Blueprint rules

- keep default layout minimal
- optimize for operator/developer usefulness
- do not try to expose every signal in v1

---

## Logging Policy

### Required Rerun streams

At minimum, when Rerun is enabled, log:
- vehicle transform in `odom`
- mission state
- battery scalar
- path history
- high-level command markers
- Multiranger rays
- projected obstacle points

### Optional streams

- local occupancy slice
- camera frames
- 3D mesh
- detections
- goals
- safety overlays

---

## Live vs Offline

### Live mode

When enabled, Rerun receives selected runtime data live.

Requirements:
- best-effort only
- must not block control loop
- must not block MCAP logging
- can drop visualization updates under load

### Offline mode

Offline visualization should be possible by replaying MCAP into Rerun.

This is preferred over maintaining a second persisted format.

---

## Performance Rules

- Rerun publishing must be asynchronous or otherwise non-blocking for flight-critical code
- camera/image logging may require throttling or selective enablement
- obstacle/path history should be bounded or decimated as needed
- visualization richness must never degrade control or safety behavior

---

## Suggested Initial Entities

Examples of entities to expose:
- `odom/vehicle`
- `odom/vehicle/path_actual`
- `odom/vehicle/path_commanded`
- `odom/obstacles/points`
- `base_link/ranges/front`
- `base_link/ranges/back`
- `base_link/ranges/left`
- `base_link/ranges/right`
- `base_link/ranges/up`
- `odom/goal`
- `events`
- `timeseries/battery_v`
- `timeseries/altitude_m`
- `timeseries/mission_state`

These names do not need to be final, but they should be stable and readable.

---

## Non-Goals

Do not:
- make Rerun a second authoritative log store
- write `.rrd` by default
- duplicate all MCAP persistence behavior in Rerun
- design internal architecture around Rerun concepts
- block on perfect 3D assets before integrating live visualization

---

## Practical Rule

MCAP is the source of truth.
Rerun is the live and replay visualization layer built on the same semantic event/state model.
