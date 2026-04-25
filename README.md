# cf-offboard

Single-process C++23 offboard control application for Crazyflie 2.x.

## Goals

- Replace ROS-style distributed systems with a single, explicit application
- Maintain full control over state, timing, and safety
- Support real hardware via Crazyradio
- Support camera input via AI Deck (WiFi)
- Enable reproducible experiments via structured logging and replay

## Non-goals

- No ROS / ROS2
- No DDS or pub/sub middleware
- No plugin-based architecture
- No multi-process system unless explicitly added later

## Hardware

- Crazyflie 2.x
- Flow deck (velocity + height)
- AI deck (camera over WiFi)
- ToF deck (incoming, for mapping experiments)

## Initial Use Cases

1. Live telemetry + state visualization
2. Offboard velocity / hover control
3. Predefined trajectory execution (square, circle)
4. Camera stream ingestion from AI deck
5. Logging + replay

## Stretch Goals

- External vision-based control loop
- ToF-based mapping experiments
- Minimal software-in-the-loop simulator (firmware-based)

## Build

### Requirements

- **C++23 compiler** with `<expected>` support — either:
  - GCC ≥ 12 (libstdc++ 12+), or
  - Clang ≥ 16 with libc++ ≥ 14 (`-stdlib=libc++`)
- **CMake** ≥ 3.20
- **libusb-1.0** development headers (`libusb-1.0-0-dev` on Debian/Ubuntu)

See `DEPENDENCIES.md` for the full list of third-party libraries.

### First-time setup

```sh
scripts/bootstrap_vendor.sh        # fetch pinned vendor sources into vendor/
cmake -S . -B build
cmake --build build
```

If the system compiler is older than GCC 12, configure with Clang + libc++:

```sh
cmake -S . -B build \
  -DCMAKE_C_COMPILER=clang \
  -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_CXX_FLAGS=-stdlib=libc++
```

The build auto-detects a vendored copy of `crazyflie-link-cpp` under
`vendor/`. To use a system-installed copy instead, configure with
`-DCFOFFBOARD_LINK_SOURCE=system`.
