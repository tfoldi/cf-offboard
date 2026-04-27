# PERCEPTION

## Purpose

Define the near-term perception scope for cf-offboard after adding the Multiranger deck.

This is not a full perception stack.
This is a narrow local-obstacle pipeline that supports:
- visibility
- local obstacle representation
- simple obstacle-aware missions
- later expansion to camera-based perception

---

## Near-Term Goal

Use Multiranger data to build a small, explicit local obstacle pipeline:

1. ingest range measurements
2. represent them in body frame
3. project them into `odom`
4. maintain a short-lived local obstacle view
5. expose them to:
   - TUI summaries
   - MCAP logging
   - Rerun visualization
   - obstacle-aware mission logic

---

## Non-Goals

Do not implement:
- full SLAM
- loop closure
- dense mapping
- generic perception framework
- global map persistence
- path planning stack

The first goal is local spatial awareness, not full mapping.

---

## Inputs

The Multiranger deck provides directional range measurements.

Near-term expected channels:
- front
- back
- left
- right
- up

Each measurement should carry:
- timestamp
- sensor identifier
- range in meters
- validity / out-of-range semantics if available

---

## Data Model

### Raw range measurement

Use a small typed value for each sample.

Example shape:
- timestamp
- sensor id
- range_m
- valid

### Body-frame ray

Derived representation:
- origin in `base_link`
- direction unit vector in `base_link`
- measured distance
- validity

### Projected obstacle point

Derived representation:
- timestamp
- point in `odom`
- source sensor id
- freshness

This point is only meaningful if:
- the range is valid
- the vehicle pose is fresh enough

---

## Frame Policy

Core frames:
- `odom`
- `base_link`

Rules:
- raw ranges are defined in sensor or base_link frame
- projected obstacle points are defined in `odom`
- the transform from `base_link` to `odom` must come from the current vehicle state

Do not create a general TF framework for this slice.

---

## Local Obstacle View

Start with a minimal local representation.

Recommended first form:
- rolling set of recent obstacle points in `odom`

Optional second form:
- simple 2D occupancy slice near the current flight altitude

### Rolling point view rules

- points expire after a short configurable lifetime
- stale points are removed automatically
- no global persistence in the first slice

This keeps the representation simple and useful for debugging.

---

## Freshness Rules

Obstacle projection should only occur if:
- the range sample is valid
- the vehicle pose is fresh enough
- the transform used is recent enough

If freshness is insufficient:
- keep the raw range
- do not project to odom
- optionally emit a warning/event

---

## TUI Expectations

The TUI does not need a map.
It should show a concise local summary, for example:
- front range
- left range
- right range
- up range
- obstacle status: clear / near obstacle / blocked

This is enough for operator awareness.

---

## Rerun Expectations

Rerun should show:
- sensor rays from `base_link`
- projected obstacle points in `odom`
- later optional local occupancy slice

This is the primary spatial debugging surface for the first perception slice.

---

## Mission Use

Perception data should be consumable by a mission in a simple way.

Near-term mission logic should use:
- latest valid forward range
- maybe side ranges
- simple obstacle condition such as:
  - clear
  - caution
  - blocked

Do not expose raw transport or sensor code directly to missions.

Provide clean perception-facing types or summaries instead.

---

## Evolution Path

Recommended progression:

1. raw Multiranger ingestion
2. body-frame rays
3. projected obstacle points
4. simple obstacle summary for mission logic
5. optional 2D occupancy slice
6. later camera and tag detections

This order preserves clarity and keeps the system small.

---

## Practical Rule

If the perception slice starts to look like a full robotics perception framework, it is too broad.
Keep it local, explicit, and mission-driven.
