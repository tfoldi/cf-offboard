# TUI

## Purpose

The terminal UI is the primary operator interface for cf-offboard.

It exists for:
- operator visibility
- mode and status awareness
- command entry
- warnings and errors
- confidence during live operation

The TUI must remain thin. It is not a second application and not a control framework.

---

## Architectural Role

The TUI is:

- an observer of application state
- a source of explicit operator intents

The TUI is not allowed to:
- talk directly to Crazyflie transport
- talk directly to protocol code
- own flight state
- implement control logic

It must interact only through a narrow app-facing interface, for example:
- read current snapshot or status
- submit typed operator commands

---

## Design Principles

- keyboard-first
- fixed layout
- readable over SSH
- no mouse dependency
- low complexity
- clear, stable regions
- no hidden behavior

The TUI should feel like an operational console, not a decorative dashboard.

---

## Scope

### First version should provide

- connection and app status
- current flight mode
- mission status
- key vehicle state values
- warnings and safety state
- recent high-level events
- simple command entry

### First version should not provide

- plots
- camera display
- complex menus
- parameter editors
- tuning screens
- generic widget framework

---

## Layout

Use a fixed-frame layout with a small number of stable panels.

### 1. Top status bar

Show:
- app state
- link state
- recording state
- current flight mode
- current mission state
- current URI

### 2. Vehicle state panel

Show:
- position x/y/z
- velocity x/y/z
- attitude roll/pitch/yaw
- battery voltage
- height above ground
- telemetry freshness

### 3. Command and mission panel

Show:
- selected mission
- mission parameters
- active action
- command state
- operator help or hints

### 4. Safety and warnings panel

Show:
- stale telemetry
- low battery
- link issues
- command rejection
- failsafe state

### 5. Event log panel

Show recent high-level events only:
- connect or disconnect
- mission started, completed, aborted
- warnings and errors
- recording started or stopped

Do not print packet-level traffic here.

---

## Input Model

The TUI should support explicit keyboard-driven commands.

Initial examples:
- `q` quit
- `c` connect
- `d` disconnect
- `r` start recording
- `m` select mission
- `s` start mission
- `x` stop or abort mission
- `e` emergency stop
- `t` take off
- `l` land

These bindings are not final, but the model should remain command-oriented and simple.

---

## Data Model Expectations

The TUI should consume:
- current snapshot
- current app status
- recent events

The TUI should emit:
- typed operator intents only

Examples:
- connect
- disconnect
- start mission
- abort mission
- emergency stop

The TUI must not directly construct protocol packets.

---

## Performance and Behavior

- TUI refresh must not block control or logging
- TUI failures must not crash the flight core
- rendering should be stable and low-noise
- values should update at a human-usable rate, not necessarily every packet

---

## Future Direction

Later, the TUI may support:
- mission parameter entry
- richer state summaries
- camera stream status
- Rerun connection status

But it should remain a thin operational surface.

Rerun is the preferred direction for rich visualization.
The TUI is for operation and awareness, not visualization-heavy workflows.

---

## Rule of Thumb

If the TUI starts to look like a framework, a browser app, or a second control loop, it has gone too far.
