---
title: BB calibration gated on QTM, and hardened against partial sweeps
type: feature
date: 2026-08-21
status: resolved
phase: "operator-observability F4"
related_plan: operator-observability.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/mocap_status.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/tests/test_bb_calibration.py
  - ros_ws/gui/js/panels.js
  - ros_ws/docs/choreography.md
  - tests/ros/conftest.py
  - tests/ros/test_mocap_node.py
  - tests/ros/test_mocap_status.py
  - tests/ros/test_bb_calibration_arc_span.py
  - tests/ros/test_teensy_bridge_node_bb.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_gui_geometry.py
  - logbook/INDEX.md
subsystem:
  - ros
  - mocap
  - gui
tags:
  - observability
  - ball-butler
  - calibration
---

# BB calibration gated on QTM, and hardened against partial sweeps

## Summary

The calibrate COMMAND path (button/HOMING → `bb/calibrate` → BB firmware
sweeps) and the calibrate DATA path (QTM → `mocap_node` → the circle fit)
shared no edge, and `MocapInterface.is_receiving()` never left its process.
With QTM down the sweep ran blind; with a *mid-sweep* dropout the solver could
fit `min_points=50` (0.25 s at 200 Hz) of stubby arc and publish a plausible BB
pose that `ball_butler_node` then aimed every throw with.

- **Q1** `mocap_node` publishes `mocap/status` (DiagnosticStatus, 5 Hz):
  `qtm_receiving`, `bb_markers_visible`, `marker3_visible`, `aligned`,
  `qtm_synced`. Pure snapshot read — no existing publication changed.
- **Q2/Q3** the bridge REFUSES `bb/calibrate` with a named code and **no RPC
  dispatched**; the orchestrator SKIPS the HOMING step (WARN +
  `bb_calibration_skipped` + `operation_result=True`). The asymmetry is
  deliberate: `HomingHandler` turns `operation_result is False` straight into
  FAULT, so a refusal reaching HOMING would fault the robot over an optional
  subsystem.
- **Q4** the GUI Calibrate button follows the mocap flag, tooltip naming why.
- **Q5** `MIN_ARC_DEG = 20.0` refuses a truncated sweep by name
  (`ARC_SPAN_TOO_SMALL`, via a branch-cut-safe `360° − largest gap` span);
  `QTM_DROPOUT_MID_SWEEP` latches a window invalid mid-collection;
  `CALIBRATION_TIMEOUT` caps the CALIBRATING window at 60 s.
- **Q6** `ros_ws/docs/choreography.md` regenerated.

**Deviation from the plan** (§ 3 says "the orchestrator's own `_qtm_ready()`"):
the predicate and both thresholds were extracted to a new pure module
`jugglebot/mocap_status.py`. Two nodes evaluating the same question with
independently drifting thresholds fails silently and asymmetrically — HOMING
skipping while the GUI button dispatches, or the reverse. Both nodes keep a
`_qtm_ready()` method; only the cached snapshot is per-node.

Implementation spanned a **salvaged agent hand-off**: a predecessor died on a
spend limit with `bb_calibration.py`, `mocap_node.py` and
`teensy_bridge_node.py` edited but never once executed. All three were re-read
against the spec. One real defect was found and fixed: `_check_calibration_health`
returned early on an already-invalidated window *before* the timeout branch, so
a sweep that lost QTM and then wedged at CALIBRATING left `_calibrating=True`
forever — and the start edge requires `not _calibrating`, so **every** later
calibration would have been silently ignored. Regression test:
`test_timeout_closes_a_window_that_already_dropped_out`.

`mocap_node` had **no direct test coverage at all** because it could not be
imported under the mocked-ROS conftest (`TransformStamped`, `MocapDataMulti`,
and `tf2_ros` — whose first import line is `from rclpy.duration import
Duration`, unusable against a mocked non-package `rclpy`). Stubs added.

## Verification

- `./run_tests.sh` (run 2026-08-22, worktree `~/Desktop/Jugglebot-obs`, venv
  `~/Desktop/PDJ_venv/venv`): **5401 passed, 5 skipped in 231.04 s**, serial
  phase empty (5841 deselected), `RESULT: PASS` (total 244 s). Up 74 tests on
  F3's 5327 — 26 `test_mocap_node.py`, 15 `test_bb_calibration_arc_span.py`,
  14 `test_mocap_status.py`, 8 bridge refusals, 6 orchestrator skips, 5 GUI.
- `python -m jugglebot.tests.test_bb_calibration` (run 2026-08-22, from
  `ros_ws/src/jugglebot`): **33/33 passed, 0 failed** — the pre-existing 28
  unchanged (case 9 "partial marker visibility" still succeeds untouched: its
  markers trace the full 180° sweep, far above the floor) plus 5 arc-span cases.
- Arc-span behaviour confirmed by probe before the tests were written
  (`/tmp/probe_arc.py`, 2026-08-22): synthetic 5°/25°/180° sweeps measure
  4.98°/24.88°/179.1°, an arc straddling ±π measures 5.73° (not ~360°), and
  degenerate input returns 0° — fails closed.
- Not hardware-validated. Plan § 8 item (2) — QTM off, press Calibrate, confirm
  the refusal code, then run HOMING and confirm WARN skip with no FAULT — and
  item (4), confirm or raise `MIN_ARC_DEG` from a real calibrate's logged yaw
  span, are the operator's next powered session. Deploy needs
  `colcon build --packages-select jugglebot` + relaunch; the GUI half is a
  browser reload.
