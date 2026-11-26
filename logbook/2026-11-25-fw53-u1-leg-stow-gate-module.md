---
title: "FW 53 unit 1: the Leg Stow gate module (predicate, de-energise wrappers, hold, HELD census) lands in the can-bridge image with no call site migrated, plus the closure lint holding today's 18 direct sites"
type: feature
date: 2026-11-25
status: in-progress
phase: "FW 53 U1"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_stow_gate.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_stow_gate.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_homing.cpp
  - tests/firmware/native/test_leg_stow_gate.cpp
  - tests/firmware/native/coldstart_hal.h
  - tests/firmware/native/coldstart_hal.cpp
  - tests/firmware/native/build.py
  - tests/firmware/test_native_firmware.py
  - tests/firmware/test_leg_deenergise_lint.py
  - tests/firmware/test_hand_single_master.py
  - logbook/2026-11-25-fw53-u1-leg-stow-gate-module.md
subsystem: [can-bridge, safety]
---

## Summary

Owner rule, 2026-11-25: "A Leg should never be made IDLE unless the Platform is already at the Stow position. Once the Platform is at Stow, then any/all Legs can freely enter IDLE." The Hand is exempt. FW 53 enforces it in the can-bridge (to land as I-FW-19). This is unit 1 of 7: the enforcement module exists, is compiled into the image and is tested natively, but **nothing calls it yet**. `FW_VERSION` stays 52 and the board's behaviour does not change. Units 2 to 5 move the call sites onto it, one ladder at a time.

## What changed

- **`leg_stow_gate.{h,cpp}` (new).** It holds:
  - `platform_at_stow(now, except_leg)`. Every present Leg must have a heartbeat no older than 0.5 s that arrived at least `CL_ENGAGE_SETTLE_US` after the gate's last CLOSED_LOOP send, and that heartbeat must report one of:
    - IDLE or ENCODER_INDEX_SEARCH;
    - CLOSED_LOOP with no errors, an encoder sample no older than 150 ms, position in [−0.11, +0.01] rev and |vel| ≤ 0.1 rev/s.
  - `all_present_legs_idle(now)`, the firmware twin of A6.
  - `odrive_set_state_tx`, `odrive_reboot_tx` and `odrive_sdo_write_tx`. The Hand passes through. CLOSED_LOOP stamps the Leg and is not gated. HOMING_OWNER may IDLE only the Leg that is homing. Every other Leg request asks the predicate. On a refusal nothing goes on CAN, the census counts it, and the Leg's HELD bit is set when the caller is a LADDER or STOW.
  - `leg_freeze_tx`: the stopping point, clamped to the commanded segment. Nothing is sent on a stale encoder.
  - The HELD mask and the refusal census.
- **`rx_older_than` moved to `time_base.h`** with its body unchanged. `fault_machine.cpp` now uses the shared copy, so its behaviour is identical.
- **`canbridge_config.h`** gains:
  - `STOW_WINDOW_MIN_REV` = −`Homing::LEG_ABS_POS_REV` − `TARGET_REACHED_POS_TOL_REV`;
  - `CL_ENGAGE_SETTLE_US` = 10 ms;
  - two `static_assert`s: the window contains the off pose, and DEACTIVATE's arrival band lies inside the window.
- **`leg_homing`** gains a read-only accessor, `homing_axis_in_flight()` (`s_axis`). The gate needs it in order to link. Nothing else changes in `leg_homing`.
- **Tests:**
  - `test_leg_stow_gate.cpp`: the design's 13 cases, plus "arrival implies the gate" (sweep) and the A6-twin case.
  - `coldstart_hal`: `cs_stage_leg` and `cs_clear_axes`.
  - `build.py` (object and binary) and the pytest wrapper.
  - `test_leg_deenergise_lint.py`: the 18 FW 52 direct sites as an exact multiset allowlist, the HOMING_OWNER confinement check and a scanner self-test.

## Discussion (departures from `fw53_design.md`, and why)

- **The HOMING_OWNER carve-out covers only `set_state(IDLE)`.** The design allowed it on all three wrappers. A reboot or SDO write from the homing owner has no call site, and allowing them would widen the one ungated path for nothing. A HOMING_OWNER request for any other Leg, or any other request, **falls through to the predicate**; it is not refused outright. The carve-out adds a permission and never removes the general rule.
- **`homing_axis_in_flight()` lands in U1, not U4.** The image links `leg_stow_gate.o`, which calls it. It reads `s_axis`, which `finish()` clears only *after* its IDLE frame. U4's HOMING_OWNER IDLE in `finish()` must therefore stay before `s_axis = 0xFF`.
- **Read order, stamp first.** The RX decode writes `axis_state` before `last_heartbeat_us`. Reading the stamp first means a fresh stamp can never certify a state older than the one it stamped.
- **Extra API:**
  - `leg_held_clear_all()` (for CLEAR_ERRORS(AXIS_ALL));
  - `leg_cl_cmd_us()` and `leg_freeze_target()` (pure; tested);
  - `leg_stow_gate_init()`.
- **Freeze:** it sets HELD even when it sends nothing, because the caller has decided to hold. Non-finite inputs send nothing.
- **The single-master lint (I-FW-15) needed a new entry.** It is argued above under Verification. **U4 must freeze only after `interp_end_stow()`.** That way the stow stream and the hold are never two writers of the same Leg.
- **Design §9 "does `gpio_poll.cpp` use the generic `encode_sdo(`?"** No. It calls `encode_sdo_write(HAND_AXIS, get_gpio_states, 0)`, a Hand SDO function-invoke. It is allowlisted for U5, where it moves to `odrive_sdo_write_tx` (Hand pass-through, unchanged on the wire). `encode_sdo_read` (`rpc.cpp`) is a read and stays allowed.

## Verification

All runs 2026-11-25 in worktree `offline/fw53`:

- **The native gate test.** `g++ … test_leg_stow_gate.cpp axis_state.o ball_butler_state.o coldstart_hal.o leg_stow_gate.o && ./t_gate`: **15/15 cases, 594/594 assertions pass**.
- **Mutations.** Each of the 19 mutations was compiled once against a *copy* of `leg_stow_gate.cpp`. **Every one is CAUGHT** (table below).
- **The lint.** `pytest -q tests/firmware/test_leg_deenergise_lint.py`: **4 passed**. It was then shown to fail twice, each time restored:
  - with the `gpio_poll.cpp` allowlist entry deleted: 1 failed (the site was reported);
  - with a direct `encode_set_state(3, IDLE)` appended to `version_check.cpp`: 1 failed.
- **The image build.** `pio run -e teensy41` (build only, no upload): **SUCCESS in 74 s**. Text is 259392 B. `leg_stow_gate.cpp` compiles with no warning.
- **The scoped suite.** `flock /tmp/jugglebot-tests.lock pytest -q tests/firmware tests/teensy_link tests/ros/test_gui_geometry.py tests/ros/test_trajectory_node.py tests/motion/{test_bench_sysid_bridge,test_hand_jam,test_hand_stream_bench_stall,test_kt_lib,test_leg_torque_ff}.py tests/sim/test_night_shift_gate.py` gave **1 failed, 2098 passed, 4 skipped in 787 s**. The full native build and every native binary passed, including `test_native_leg_stow_gate`.
  - The one failure was `test_hand_single_master.py::test_only_the_sanctioned_tus_emit_a_motion_command`. `leg_freeze_tx` encodes `set_input_pos`, so `leg_stow_gate.cpp` is a new motion-emitting TU, and the design had missed that lint.
  - I added it to `_ALLOWED_MOTION_TUS` with the argument written next to the entry: it is not a writer of axis 6. It refuses axis ≥ NUM_LEGS before encoding (pinned by case 13), and it runs only on behalf of a ladder or the deferred stow.
  - Re-run: `pytest -q tests/firmware/test_hand_single_master.py tests/firmware/test_leg_deenergise_lint.py` gave **10 passed**.
- **Expected red until the orchestrator integrates:** `tests/sim/test_logbook_index.py`. The INDEX is not regenerated here, by rule.

| # | Mutation of `leg_stow_gate.cpp` | Result (failing case) |
|---|---|---|
| 1 | drop clause (a) | CAUGHT (1, 2, 10, …) |
| 2 | evaluate only the target Leg | CAUGHT (2, 11, 12) |
| 3a | two-sided \|pos\| ≤ tol | CAUGHT (3, 8) |
| 3b | no lower bound | CAUGHT (3) |
| 3c | no velocity term | CAUGHT (3) |
| 4 | `CAN_AXIS_SILENCE_TIMEOUT_US` for the heartbeat age | CAUGHT (4, 15) |
| 5 | ignore the encoder stamp | CAUGHT (5) |
| 6 | drop the error term | CAUGHT (6) |
| 7 | allow every non-CL state | CAUGHT (7) |
| 8 | drop the CL-confirmation clause | CAUGHT (8) |
| 9 | require all six Legs | CAUGHT (9) |
| 10 | raw `now - hb` subtraction | CAUGHT (10) |
| 11 | HOMING_OWNER for any axis | CAUGHT (11) |
| 12 | HELD set for an RPC caller | CAUGHT (12) |
| 13a | freeze at the raw position | CAUGHT (13) |
| 13b | freeze without the clamp | CAUGHT (13) |
| 13c | freeze sends on a stale encoder | CAUGHT (13) |
| 14 | window upper bound diverges from the arrival tolerance | CAUGHT (3, 14) |
| 15 | A6 twin counts ENCODER_INDEX_SEARCH as idle | CAUGHT (15) |
