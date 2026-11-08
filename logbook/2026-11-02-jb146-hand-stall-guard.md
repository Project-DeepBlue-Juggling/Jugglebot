---
title: "BUG-56: can-bridge FW 36 stall guard: an ACTIVATE / HAND_MOVE_TO that holds the Hand at high current without motion for 300 ms commands it IDLE; the legs carry on"
type: bugfix
date: 2026-11-02
status: open
phase: "BUG-56 (Sitting 2026-11-02 incident)"
subsystem: can-bridge
related_entries:
  - 2026-11-02-hand-top-margin-and-70a.md
  - 2026-11-02-hand-odrive-limits-backup.md
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_activate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/telemetry.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - docs/teensy-udp-protocol.md
  - teensy_link/activate.py
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/firmware/native/test_leg_activate.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_hand_stall_guard_xref.py
  - tests/teensy_link/test_activate.py
  - tests/teensy_link/test_hand_move_to.py
---

# BUG-56: the Hand stall guard (can-bridge FW 36)

**What happened.** Sitting 2026-11-02, 16:52: an ODrive GUI save reset the
Hand's zero. The next ACTIVATE parked the Hand into its stop, the position
loop wound up to about 70 A in 0.55 s, and it held that for 5.5 s until
MOTOR_OVER_TEMP. Nothing on the bridge watched current during the cold-start
ladder.

**What changed (FW 35 → 36).**
- The ACTIVATE / HAND_MOVE_TO ladder (`leg_activate.cpp`) aborts the Hand when
  `|iq_measured| >= 25 A` (`HAND_STALL_IQ_A`) and `|vel| <= 0.5 rev/s`
  (`HAND_STALL_VEL_RPS`) hold continuously for 300 ms (`HAND_STALL_HOLD_US`;
  owner rule: no motion with high current for 300 ms or more aborts the move).
  Axis 6 is commanded IDLE and dropped from the ladder.
- **Hand only.** The legs of an ACTIVATE carry on to their active pose: a Hand
  stall never drops the Platform. Legs are not guarded: there is no
  config-derivable leg stall threshold, and a leg abort would IDLE a loaded
  Platform.
- Named on the wire in two ways, both values that were zero before. A
  HAND_MOVE_TO is answered ERR_REJECTED with the new outcome
  `HAND_MOVE_STALLED = 4` (an older host prints `outcome_4`). The Hand's
  Diagnostic `flags` gains bit2 `activate_stall`, sticky until the next ladder
  that targets the Hand. `flags` is a change-detect term, so the edge is sent
  at once. `teensy_bridge_node` logs the rising edge as a STALL error naming
  the zero reset. PROTOCOL_VERSION stays 10.
- The same FW 36 build carries the stream STOP's Hand deceleration
  3500 → 4500 rev/s² (BUG-57, owner decision; see
  `2026-11-02-hand-top-margin-and-70a`), so there is one bump and one flash.
- Build: `pio run -e teensy41` 2026-11-02, text 251200 / data 35520 / bss
  111488. NOT flashed.

## Limits of the guard (audit 2026-11-02)

- **It ends at ARRIVED.** A Hand that stalls within 0.01 rev
  (`TARGET_REACHED_POS_TOL_REV`) of its target is declared arrived and handed
  to PASSTHROUGH, and nothing watches it after that. At the shipped gains that
  residual winds up about 4.4 A/s (70 A after about 16 s), much slower than the
  incident. The drive's own 60 / 70 °C thermistor limits are what bound it.
- **IDLE drops a Hand that stalls high.** The response de-energises the Hand
  where it stalled. A stall near the top (for example a wrong-frame top-stop
  stall) lets the carriage fall the stroke unpowered: up to about 50 rev/s at
  the bottom stop by a friction-free estimate. The existing timeout and bus-down
  aborts already behave the same way; holding position at reduced current would
  be the alternative.
- **A late Hand stall does not fail the activate.** The bridge's activate
  returns when the legs finish (about 1 s), but the Hand's park from a high
  prime walks about 4 s. A stall after that point is still IDLEd by the
  firmware and logged as a STALL error, but the activate itself reports
  success. Waiting for the Hand monitor instead was tried and reverted: nothing
  guarantees it finishes (a hand-less bench, a missing Hand report), so every
  activate could hang up to 25 s. The incident itself is caught, because its
  Hand stalled within 0.55 s.
- A STALL bit already set at node start is logged as sticky, not as a live
  event.

## Verification

- Native firmware tests (2026-11-02,
  `pytest -q tests/firmware/test_native_firmware.py`, the agent's run): 17
  passed. The `test_leg_activate` binary has 33 cases, including the 6
  stall-guard ones; with the guard call disabled the 3 positive cases fail.
- Full tier (2026-11-02, `./run_tests.sh --full`, with C-HAND-6, after
  rebasing onto f81984b2): 8014 passed, 56 skipped + 8 serial. The two
  failures are listed in `2026-11-02-hand-top-margin-and-70a`; neither is in
  this guard.
- Flashed 2026-11-02 ~23:40 with the launch down (`pio run -e teensy41 -t
  upload` from the live checkout at 9ec01762): SUCCESS. BridgeIdentity receipt:
  **FW 36 / PROTOCOL 10**. The live checkout was rebuilt (`colcon build
  --packages-select jugglebot`, install == source).
- Owner on the "IDLE drops a Hand that stalls high" limit: it will not happen.
  The Hand is stiff enough that it does not move under gravity even holding a
  Ball, so IDLE is the response.
