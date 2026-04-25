# Dependencies

Third-party libraries used by `cf-offboard`. Vendored libraries are pinned to
a specific commit and fetched into `vendor/` by `scripts/bootstrap_vendor.sh`.
The `vendor/` directory itself is not tracked.

## System packages

| Name           | Purpose                                  | Notes                                            |
|----------------|------------------------------------------|--------------------------------------------------|
| `libusb-1.0`   | USB I/O for the Crazyradio dongle        | Pulled in transitively via `crazyflie-link-cpp`. |
| C++ toolchain  | Build the project                        | See compiler requirements in `README.md`.        |

## Vendored libraries

Each entry lists upstream, pinned commit, license, and the single point of
contact in our code that depends on it. **No other source file may include the
vendor's headers** — keeping the dependency surface small is intentional.

### crazyflie-link-cpp

- **Upstream:** https://github.com/bitcraze/crazyflie-link-cpp
- **Pinned commit:** `598dea123326652e8e82ea642abaff3496e922fe` (2025-12-02)
- **License:** MIT
- **Used by:** `src/crazyflie/link/link.cpp` only.
- **Why:** byte-level CRTP packet I/O over Crazyradio (USB). Scope-matches our
  `ICrazyflieLink` adapter — `Connection::send` / `Connection::receive` is
  already a queue/poll API, no callbacks leak into client code.
- **Build flags:** `BUILD_PYTHON_BINDINGS=OFF`, `BUILD_CPP_EXAMPLES=OFF`
  (set by our top-level `CMakeLists.txt` before `add_subdirectory`).

### doctest

- **Upstream:** https://github.com/doctest/doctest
- **Pinned commit:** `d44d4f6e66232d716af82f00a063759e9d0e50d6`
- **License:** MIT
- **Used by:** `tests/` only.
- **Why:** single-header C++ test framework. Fast to compile, zero runtime
  dependencies. Used to assert decoder/builder behavior on known, empty,
  oversized, and wrong-port inputs.

### mcap (foxglove/mcap)

- **Upstream:** https://github.com/foxglove/mcap
- **Pinned commit:** `c3cab6bd3ce79199e362766daec3a4689f3a0335`
- **License:** MIT
- **Used by:** `src/logging/mcap_logger.cpp` only.
- **Why:** MCAP is the project's primary log format (see `LOGGING.md`). The
  C++ writer is header-only; we use it to serialize `LogEvent` into `.mcap`
  files for replay and analysis (Rerun, Foxglove Studio, custom tools).
- **Build flags:** `MCAP_COMPRESSION_NO_LZ4` and `MCAP_COMPRESSION_NO_ZSTD`
  defined via the `mcap_headers` INTERFACE target. We write uncompressed for
  now; revisit if log size becomes an issue (would require adding lz4/zstd
  system packages or vendoring them).

## Updating a pinned dependency

1. Edit the SHA in `scripts/bootstrap_vendor.sh` (and update the entry above).
2. Run `scripts/bootstrap_vendor.sh` — it checks out the new SHA in place.
3. Rebuild and run the relevant compile checks.
4. Commit the changes to this file and the script in the same commit.
