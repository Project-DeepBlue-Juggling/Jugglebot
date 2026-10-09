---
title: A failed BB calibration sweep no longer wipes the last good calibration — bb/calibration_result carries only the calibration in force, every sweep's outcome goes to a new latched bb/calibration_attempt, and the failure line is one short line (≤ 240 chars; the 2026-10-09 live one was ~900)
type: bugfix
date: 2026-10-10
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/gui/js/bb-calibration-status.js
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/css/panels.css
  - ros_ws/docs/choreography.md
  - tests/ros/test_mocap_node_keep_last_good.py
  - tests/ros/test_gui_bb_calibration_status.py
  - tests/ros/test_mocap_node_yaw_gate.py
  - logbook/2026-10-10-bb-calibration-keep-last-good.md
  - logbook/INDEX.md
subsystem:
  - ros
  - gui
  - tracking
tags:
  - testing
---

# A failed BB calibration sweep no longer wipes the last good calibration; the failure line is one short line

## Problem

Owner: "If a prior calibration succeeded but a subsequent one failed, the calibration should NOT be wiped; the failed result should simply be discarded — the calibration is only updated by successful results." And the failure message was excessively long.

`mocap_node._publish_calibration_failure` published `success=False` on the LATCHED `bb/calibration_result`. `ball_butler_node._on_bb_calibration` already ignored a failure in memory, but the latch now held the failure, so a `ball_butler_node` (re)started after it got no calibration; the GUI flipped to "Not Calibrated" and ghosted BB; and the BallButler accuracy runner's drift check (`run_local_calibration.py`, `latest.success` false) aborted a session on a failed recalibration although the calibration in force had not changed. The state file was already success-only. The failure text was the gate verdict plus the whole estimator summary (`[...]`), ~900 characters on 2026-10-09.

## Discussion — why a second topic, not a new message field

Two mechanisms were considered:

1. **Add fields to `BallButlerCalibrationResult`** (e.g. `last_attempt_failed`, `last_attempt_message`) and republish the last success with them set. Rejected: a `.msg` change needs every consumer rebuilt in step: the jugglebot packages, rosbridge's type lookup, and the BallButler runner, which imports the installed interfaces. Under Foxy a node built against the old definition does not refuse a mismatched type; it misreads it. It would also republish the pose on every failure, so the runner's exact-equality drift check depends on float round-trips.
2. **Chosen: a second latched topic of the same type.** `bb/calibration_result` = the calibration IN FORCE (only a success replaces it); `bb/calibration_attempt` = the most recent sweep's outcome, success or failure. Nothing needs rebuilding, and every existing subscriber of `bb/calibration_result` keeps working unchanged. The attempt topic is latched (depth 1), so a reloaded GUI can still show why the last attempt failed.

If this process has never succeeded, a failure still goes to `bb/calibration_result` as before (no calibration exists to protect). A restarted `mocap_node` does not republish the state file's calibration. That would be a separate decision: the file is the gate's reference, not an announcement.

## Fix

- `mocap_node`: `pub_calibration_attempt` (`bb/calibration_attempt`, TRANSIENT_LOCAL depth 1). `_publish_calibration_result` publishes to both topics and remembers the last good message and its acceptance time. `_publish_calibration_failure` always publishes the attempt, and publishes the result only if there is no last good calibration. ERROR line: `BB calibration FAILED: <reason>`, plus ` (kept the calibration from <time>)` when one was kept. The success message now starts `Calibration successful (accepted 2026-10-10T12:34:56Z)`, and the GUI reads the time from it. `_persist_accepted` returns that stamp. The health check's timeout/dropout path is unchanged: it routes through the same publisher, and its "don't overwrite the more specific cause" rule now protects the attempt latch.
- Short-message contract: `MAX_CALIBRATION_FAILURE_CHARS = 240`, enforced in `_publish_calibration_failure` (whitespace collapsed to one line; an over-long reason is cut with `…` and the full text is logged at DEBUG). This is a safety net only: every reason the pipeline produces fits without truncation (tests). The estimator summary moved to DEBUG (it was already in the `Yaw offset …` DEBUG line). The gate reference label is `accepted <iso to the second>`. CALIBRATION_STATE_UNREADABLE names the file's basename and the exception class. Its full exception is now DEBUG; it was a second ~200-char ERROR line. A WARN covers the bb_moved-overridden case.
- `bb_calibration`: message strings only. The gate's logic and thresholds are untouched. Example: `CALIBRATION_INCONSISTENT: Δyaw -0.366° exceeds ±0.150° vs 0.926° (accepted 2026-10-09T13:24:52) — set bb_moved:=true if BB or QTM moved`, with `, axis point moved 1.60 mm > 1.5 mm` added when the axis point also moved. TEMPLATE_RESIDUAL drops the gate head. The CONSTELLATION_* ValueErrors and the circle-centre deviation are trimmed. The per-marker breakdown is kept (`Marker k x.xx`, pinned by the consensus test), and at 225 chars it is the longest reason, which sets the 240 cap.
- GUI: `bb-calibration-status.js` (pure, no DOM) holds the indicator logic (`Calibrated · HH:MM` from the acceptance time) and the note logic (`Last attempt failed: <reason>`). `panels.js` gains the `bb-calib-note` line under the indicator, and the disconnect reset clears it. In `main.js`, `bb/calibration_result` drives the indicator and BB's 3D placement, and `bb/calibration_attempt` drives the note and the Event Log (one event per sweep). The stale-latch skip moved with the event.
- `ball_butler_node`: docstring only. It already ignored `success=False`. Its `_cal_*` FSM is the accuracy-throw volley: it never waits on a calibration message, so it cannot hang. The mocap-side window FSM closes after a discarded failure, and the next sweep runs (tested).
- Launch bag list records `/bb/calibration_attempt`. `ros_ws/docs/choreography.md` regenerated (the attempt topic has no ROS subscriber; the GUI is its consumer).

external_changes: none. `~/Desktop/BallButler/zTesting/throw_testing/accuracy_testing/run_local_calibration.py` needs no change: it reads `bb/calibration_result`, which no longer flips to a failure mid-session. It does not record `/bb/calibration_attempt` in its bag.

## Seen, NOT changed (out of scope: gate logic)

`mocap_node._gate_reference` builds the reference from `yaw_offset_deg` and `position_mm` only. It drops the state file's `yaw_offset_std_deg`, so in the live node the gate's combined limit `max(3·√(σ_new² + σ_ref²), 0.15°)` (98ad0d05) still runs with σ_ref = 0. The tests that pin the combined limit call `check_calibration_consistency` directly, so they cannot see this. It is a two-line fix in the gate's input, left to the owner of the gate.

## Verification

- Scoped (`~/Desktop/PDJ_venv/venv/bin/python -m pytest -q tests/ros/test_mocap_node.py tests/ros/test_mocap_node_yaw_gate.py tests/ros/test_mocap_node_keep_last_good.py tests/ros/test_perception_console_lines.py tests/ros/test_ball_butler_node.py tests/ros/test_gui_geometry.py tests/ros/test_gui_bb_calibration_status.py tests/ros/test_choreography_map.py tests/ros/test_bb_calibration_*.py`, run 2026-10-10): **398 passed, 1 skipped**. The new tests are 21 in `test_mocap_node_keep_last_good.py` and 3 in `test_gui_bb_calibration_status.py` (the view logic run under node). The yaw-gate helper now reads `bb/calibration_attempt`.
- Full gate (`./run_tests.sh --full`, run 2026-10-10 01:05–01:15 local, this tree minus this Verification line): **PASS — 6227 passed, 9 skipped, 1 xfailed (parallel, 365 s) + 6 passed (serial, 23 s); total 388 s**. After this line was written: `pytest -q tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_plans_index.py` (2026-10-10), see the commit message.
- Not verified on hardware: rosbridge delivering the latched `bb/calibration_attempt` to the GUI, and the live failure line.
