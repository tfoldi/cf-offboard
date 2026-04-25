# Architecture

## Core Principles

- Single process
- One authoritative state
- Fixed-rate control loop
- Explicit data flow
- Logging as a first-class concern

## Functional Bias

Core logic (control, safety, trajectory) should follow a functional style:

- inputs → outputs
- no hidden state
- deterministic behavior

State mutation is restricted to:

- state store updates
- IO boundaries

## Execution Model

Threads:

- RX thread (Crazyradio telemetry)
- Control loop (100 Hz)
- Logger thread
- Optional UI thread

## Data Flow

Incoming:

Crazyradio → raw packet → decode → state store → log

Outgoing:

state snapshot → controller → safety → command → Crazyradio → log


--------------------------------
REPOSITORY STRUCTURE (GUIDELINE)
--------------------------------

Use a simple, flat, feature-oriented structure.

Example:

src/
  app/
  crazyflie/
    link/
    telemetry/
  state/
  control/
  safety/
  logging/
  camera/
  ui/

include/ (optional, if needed)

tests/

Do NOT introduce deep nesting or generic framework layers.

--------------------------------
RULES
--------------------------------

- Organize by feature/domain, not by technical pattern
- Keep modules small and focused
- Avoid cross-module coupling
- Prefer simple headers + source pairs
- No "core framework" or "engine" modules

--------------------------------
ANTI-PATTERNS
--------------------------------

Do NOT introduce:

- messaging/
- middleware/
- pubsub/
- framework/
- platform/

If a directory name sounds generic, it is probably wrong.

--------------------------------
INTERNAL STRUCTURE
--------------------------------

Inside a module:

- types.hpp        (data structures)
- interfaces.hpp   (if needed)
- implementation.cpp

Keep it minimal. Do not over-separate too early.

## Key Rule

There is no internal pub/sub system.
