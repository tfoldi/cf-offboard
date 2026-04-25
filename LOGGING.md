# Logging

## Purpose

Logging is a first-class component of this system.

It is not for debugging only. It enables:

- flight analysis
- reproducibility
- offline replay
- controller validation
- system debugging without hardware

All critical runtime behavior must be observable through logs.

---

## Format

The primary logging format is **MCAP**.

All flight sessions must be recorded as MCAP files.

The system must produce logs that are directly usable with:
- Rerun
- custom analysis tools
- offline replay tools

---

## Design Principles

### 1. Logging is asynchronous

- Logging must never block the control loop
- All log writes go through a bounded queue
- A dedicated logger thread handles serialization and disk I/O

---

### 2. Internal model is NOT topic-based

Internally, the system uses:
- structs
- function calls
- explicit data flow

MCAP topics are an **output format only**, not an internal architecture.

Do NOT introduce pub/sub internally to “match” MCAP.

---

### 3. Log both raw and processed data

For all telemetry:

- raw packets MUST be logged
- decoded/structured data MUST be logged

This allows:
- debugging protocol issues
- reprocessing with improved decoders
- validating assumptions

---

### 4. Deterministic replay

Logs must contain enough information to:

- reconstruct state over time
- replay control decisions offline

Controllers and safety logic should be reusable in replay mode.

---

### 5. Timestamping

All events must include timestamps:

- use a consistent clock source
- distinguish between:
  - receive time (host)
  - sensor time (if available)

---

## Event Model

The system produces structured events internally.

Example:

```cpp
using LogEvent = std::variant<
    RawTelemetryEvent,
    TelemetryEvent,
    StateSnapshotEvent,
    CommandEvent,
    SafetyEvent,
    ModeChangeEvent,
    LinkEvent,
    CameraEvent
>;
