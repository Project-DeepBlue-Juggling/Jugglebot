---
title: "BUG-93: the can-bridge silence predicates no longer wrap on a receive stamp newer than the clock read, so a live Leg cannot trip CAN_BUS_DOWN for one tick (can-bridge FW 46, NOT flashed)"
type: bugfix
date: 2026-11-15
status: complete
phase: "BUG-93"
subsystem: can-bridge
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - teensy_link/rpc_args.py
  - tests/firmware/native/test_fault_machine.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
---

# BUG-93: the silence predicates no longer wrap on a stamp newer than the clock read

**What.** `fault_machine.cpp` gains one helper, `rx_older_than(now, ts, limit)` = `now > ts && now - ts > limit`, and every age test over a receive-stamped timestamp goes through it: the fatal `any_present_leg_silent` and its inverse `all_present_legs_alive` (the CAN_BUS_DOWN trip and its reconnect), `first_silent_leg` (the `can_fault_leg` latch), `fault_hb_stale_mask`, the per-axis and Ball Butler `heartbeat_stale` bookkeeping, and the two MOTOR_FB_STALE checks (already guarded by hand since FW 27; same semantics, now through the helper). can-bridge FW 45 → 46 (renumbered twice, below), no wire change, PROTOCOL_VERSION stays 14.

**Why.** Each predicate captures `now = micros64()` and then reads `last_rx_us`. `task_can_rx` (priority 5) preempts the fault task (priority 3) between the two and can stamp a frame newer than `now`; the unsigned `now - ts` wraps to about 2^64 and the 2 s threshold is exceeded. On the fatal predicate a live Leg read silent for one evaluation: `s_fatal_can_error`, output suppressed, Stow armed mid-Attempt, latched as Leg 0xFF / age 0 because `first_silent_leg` re-reads a clock that has moved on. A stamp at or after `now` is now age zero: not silent, and alive in the inverted predicate, so the two stay exact complements over the present Legs. `jetson_link_up` is left as is and commented: it reads the stamp before the clock, so the order cannot invert.

**Host mirror.** `teensy_link/fault_logic.py` takes the predicates as `stale`/`fresh` booleans and does not transcribe the subtraction, so it is unchanged, and the golden (`tests/firmware/native/fault_golden.json`) is byte-identical: no golden scenario stamps a future timestamp.

**Tests.** Four new cases in `tests/firmware/native/test_fault_machine.cpp` stamp `ts = now + 1 us` and `now + 1 h` (the board's interleaving, reproduced without preemption): no trip, `any_present_leg_silent` false, `all_present_legs_alive` true, `first_silent_leg` 0xFF, empty `fault_hb_stale_mask`, no `heartbeat_stale`; a future stamp on Leg 0 neither hides nor steals a real silence on Leg 3 (latch names 3, age 2000 ms) and reads alive at reconnect; the boundary is unchanged (age = timeout does not trip, timeout + 1 does); `rx_older_than` is strict on both sides. The xref gains the FW 46 history clause.

**NOT flashed.** The branch is held for a flash and a Sitting; until then BRIDGE_FW_CHECK reports a skew against a board on FW 43. That check logs an ERROR-level `BRIDGE_FW_CHECK: FAIL` but refuses nothing (`teensy_link/rpc_args.py`), so merge this branch together with the flash, not before it. The Item is closed on the branch by the Night Shift's convention (the tip that merges carries the closure); it is not verified on the board.

**Same class outside this file (audit, open).** `can_buses.h` (`bus_partner_present`, `classify_bus_health`, `classify_command_gate`) and `telemetry.cpp` (the diagnostic `stale` bit) keep the unguarded `now - last_rx_us`. None is fatal: the worst case is one refused CAN3 TX or RPC, or one wrong diagnostic bit. The one enforcement point therefore covers `fault_machine.cpp` only until those sites route through the helper.

## Verification

- Fail-without-change (2026-11-15, helper body temporarily `return (now - ts) > limit;`, `night_shift_nonet.sh python -m pytest tests/firmware/test_native_firmware.py -q -k native_fault_machine_binary_passes`): **1 failed**. doctest reported 33 cases, 3 failed: the all-future case (`any_present_leg_silent() == false` at ahead = 1 us), the steal case (`first_silent_leg == 3` read 0) and the strictness case. The boundary case passed, as it should. Guard restored.
- Scoped run (2026-11-15, `night_shift_nonet.sh python -m pytest tests/firmware/ tests/teensy_link/ -q --continue-on-collection-errors`): **792 passed, 5 skipped, 2 errors in 704.79 s**. The 2 errors are collection errors in `test_ball_butler_xref.py` and `test_odrive_protocol_xref.py` (`No module named 'jugglebot_interfaces'`: they import `jugglebot.can`, whose mock lives in `tests/ros/conftest.py`, which a firmware-only run does not load). They do not depend on this change. `test_committed_golden_matches_live_firmware` passed and `fault_golden.json` is unchanged.
- Those two files plus the `EXPECTED_BRIDGE_FW_VERSION` readers (2026-11-15, `night_shift_nonet.sh python -m pytest tests/ros/test_teensy_bridge_node_install_skew.py tests/ros/test_teensy_bridge_node_bridge_diag.py tests/firmware/test_ball_butler_xref.py tests/firmware/test_odrive_protocol_xref.py -q`): **79 passed in 20.86 s**.
- The default gate (`./run_tests.sh`) was not run by this unit. It is owed before merge.

**Renumbered 2026-11-18.** This change was built as FW 44. The robot bridge was flashed on 2026-11-18 with a different FW 44 (per-group scheduler grace, entry `2026-11-18-leg-scheduler-grace-30ms`), so the held branch was merged up to main and this change became FW 45: `FW_VERSION`, the `44→45` history line, `EXPECTED_BRIDGE_FW_VERSION`, the cross-reference test and the native test names. No predicate or test logic changed in the renumber. Verification of the renumbered branch is in the commit message.

**Renumbered again 2026-11-20 (Night Shift).** FW 45 went to the Platform IMU relay (IDEA-20, owner decision 2026-11-19), so the held branch was merged up to main and this change became FW 46. The changes are `FW_VERSION`, the `45→46` history line after main's `44→45`, `EXPECTED_BRIDGE_FW_VERSION`, the cross-reference test (main's FW 45 assertions kept, this change's moved to `m46`), the native test names and the helper's comment. No predicate or test logic changed. Verification (2026-11-20, `night_shift_nonet.sh timeout 900 python -m pytest tests/firmware/test_bridge_fw_version_xref.py tests/firmware/test_native_firmware.py tests/firmware/test_fault_logic.py tests/teensy_link/ -q -p no:cacheprovider`): **684 passed in 456.92 s**. The version readers (2026-11-20, `... pytest tests/ros/test_teensy_bridge_node_install_skew.py tests/ros/test_teensy_bridge_node_bridge_diag.py tests/firmware/test_ball_butler_xref.py tests/firmware/test_odrive_protocol_xref.py tests/sim/test_mapkit_sheet.py tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py -q -p no:cacheprovider`) also pass after the index was regenerated. The bridge must be flashed with FW 46 from this branch for the Sitting. A bridge on FW 46 fails BRIDGE_FW_CHECK on main (45) until this branch lands.
