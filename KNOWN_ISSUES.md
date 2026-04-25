# Known Issues

Operator-facing issues with workarounds. Investigation TODOs are listed
separately; the workaround keeps the system usable in the meantime.

---

## Supervisor lock after `setpoint_stop` end-of-flight

**Symptom.** A second consecutive run of `cf-offboard` (no power-cycle in
between) starts up cleanly through link/log setup, then silently fails to
fly. The pre-arm supervisor query reports:

    armed=false can_arm=false can_fly=false tumbled=false locked=true crashed=false

`CMD_RECOVER_SYSTEM` (cflib's `Crazyflie.supervisor.send_crash_recovery_request`)
is sent automatically on detection but does not clear the `locked` bit on
the firmware build we tested — post-recover state is byte-identical to
pre-recover.

**Workaround.** Power-cycle the Crazyflie between flights. The lock is
volatile firmware state and clears on boot.

**Hypothesis.** `setpoint_stop` (CRTP port 7, type 0) sent at the end of a
hover-setpoint-driven flight is interpreted by the firmware supervisor as
an unsafe termination, latching `is_locked`. cflib's published recovery
command does not address this specific lock cause on this firmware build.

**Investigation TODO.**
- Compare with cfclient: take off → land → take off again, see whether the
  same lock latches and how cfclient unlocks (if it does).
- Revisit when switching from `setpoint_stop` to high-level-commander land
  (CRTP port 8). A commanded landing may not trigger the same lock.
- Try variations: longer dwell at `z=0` before stop, `setpoint_stop` only
  after reading `is_flying=false` from the supervisor.
- Check firmware version + supervisor.c source for what specifically sets
  `is_locked` and what clears it.
