---
title: ODrive errors decoded by name and axis — shell, robot_state and GUI event log
type: feature
date: 2026-08-21
status: resolved
phase: "operator-observability F3"
related_plan: operator-observability.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/gui/js/odrive-errors.js
  - ros_ws/gui/js/main.js
  - ros_ws/docs/can-node-teensy-parity.md
  - tests/hardware/supported_platform_test.py
  - tests/hardware/single_leg_test.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_gui_geometry.py
  - logbook/INDEX.md
subsystem:
  - ros
  - can
  - gui
tags:
  - observability
  - odrive
  - parity
---

# ODrive errors decoded by name and axis

## Summary

The full 32-bit `active_errors` + `disarm_reason` masks already reached the
Jetson on every DIAGNOSTIC frame; the detail was destroyed here and only here.
`robot_state` collapsed them to booleans plus a fixed string, the guard-latch
hint printed a bare `(leg 3)`, and `jugglebot.can.odrive.ERROR_CODES` — the
decode table — had **zero consumers in the launched node graph**. F3 restores
what `can_node` did (`7c7f61b^:can_node.py:419-447`), reusing the throttle
scaffolding that survived the cutover unused in `can/motor_state.py`.

- **C1** `_guard_fault_leg_hint` decodes both masks and scans **every** axis in
  `_latest_diag`, not `range(6)` — a hand/BB fault previously produced a
  completely empty hint.
- **C2** `_log_odrive_errors`, off the 10 Hz `_publish_link_status`: one ERROR
  line per faulted axis, throttled 10 s per (axis, code) via a **second**
  `MotorStateTracker` (`self._error_log`; `self._versions` is the firmware
  handshake's and must not be overloaded).
- **C3** decoded per-axis lines appended to `robot_state.error[]` **only inside
  the already-fatal branch** — `orchestrator_node._tick` force-FAULTs on any
  non-empty `error[]`, so observability must never declare a fault. Appended,
  never prepended: `state-minimap.js` renders `error[0]` as the headline.
- **C4** the orchestrator logs `ctx.errors` once on entry to FAULT, so the
  shell says *why* beside `[SM] Entering FAULT`.
- **C5/C6** `gui/js/odrive-errors.js` (hand-mirrored table + `errorNames`) feeds
  per-axis change-detected rows in the GUI event log, handling the 9→7 axis
  shrink by forgetting the vanished baselines.
- **C7** parity rows 35 (still PARTIAL — the unconditional `"Disarmed axes:"`
  entry stays dropped deliberately, it would FAULT on any benign disarm) and 62
  (→ ported+validated) restated, with the status-summary counts reconciled.

Both masks are decoded everywhere: the 2026-08-10 leg-0 spinout had
`active_errors == 0` and the truth sticky in `disarm_reason`.

**Bug found by the new drift pin:** `tests/hardware/supported_platform_test.py`
mapped `0x1000000` to `ESTOP_REQUESTED` (it is `WATCHDOG_TIMER_EXPIRED`) and
stopped at `0x10000`, so on that bench harness a watchdog expiry printed the
wrong cause and `SPINOUT_DETECTED` printed nothing. All four hand-mirrored
copies of the table now sit behind equality pins against `can/odrive.py`.

## Verification

- `./run_tests.sh` (run 2026-08-21, worktree `~/Desktop/Jugglebot-obs`):
  **5327 passed, 5 skipped in 250.44 s**, serial phase empty (0 selected),
  `RESULT: PASS`.
- JS↔Python decoder equivalence checked by probe (`node` vs the AST-extracted
  Python table) over eight masks including bit 31 and an unknown-bit residue:
  byte-identical output, `UNKNOWN(0x…)` aggregate included.
- Not yet hardware-validated — the bench check (induce an ODrive error, confirm
  shell + event-log decode) is item (1) of the plan's § 8 operator checks.
  Deploy needs `colcon build --packages-select jugglebot` + relaunch; the GUI
  half is a browser reload.
