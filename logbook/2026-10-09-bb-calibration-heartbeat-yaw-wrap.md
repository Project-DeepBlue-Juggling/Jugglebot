---
title: BB calibration reads the heartbeat yaw on its principal branch, so a sweep that starts parked just below zero (359.6° on the wire) no longer fails CONSTELLATION_TOO_FEW_MOVING; the first CALIBRATING heartbeat's yaw is recorded
type: bugfix
date: 2026-10-09
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - tests/ros/test_bb_calibration_constellation.py
  - tests/ros/test_mocap_node_yaw_gate.py
  - logbook/2026-10-09-bb-calibration-heartbeat-yaw-wrap.md
  - logbook/INDEX.md
subsystem:
  - ros
  - tracking
tags:
  - kinematics
  - testing
---

# BB calibration reads the heartbeat yaw on its principal branch; the first CALIBRATING heartbeat's yaw is recorded

## Problem

The owner ran 7 calibration sweeps on 2026-10-09 (bag `~/Desktop/rosbags/2026-10-09_23-49-07/`, mocap_node log `~/.ros/log/python3_1458901_1791550150362.log`). Sweeps 4 and 7 succeeded (+0.4866° / +0.4938°, lag 129 / 127.5 ms). Sweeps 1, 2, 3, 5 and 6 failed with `CONSTELLATION_TOO_FEW_MOVING: 0 moving heartbeat yaw samples` (n = 0), although their marker counts matched the good sweeps.

## Root cause

- **The wrap.** `/bb/heartbeat.yaw_deg` is wrapped to [0, 360). BB parks at −0.2…−0.5°, which is 359.5–359.8° on the wire, after every sweep in this bag. `fit_yaw_latency` ran a bare `np.unwrap`, which keeps the branch of the first sample. A sweep whose first recorded sample was on the 360 side was therefore read as 359.7 … 484°. The selection against E's valid range (−5…130°) then rejected every sample, so the fit returned None with n = 0.
- **The pattern matches exactly.** The first recorded samples were 357.72 / 359.98 / 359.96 / 0.11 / 359.61 / 359.63 / 0.09. The five on the 360 side are the five failures.
- **The lost sample.** `mocap_node._on_bb_heartbeat` appended the yaw before it detected the CALIBRATING start edge, so the first CALIBRATING heartbeat was never recorded. This did not decide pass or fail; the first heartbeat is also parked.

## Fix

- **`bb_calibration.canonical_yaw_deg`**: wraps each sample to [−180, 180), then unwraps from the first. BB's physical yaw range never approaches ±180°. It is used in three places:
  - `fit_yaw_latency`;
  - the moving-frame selection in `estimate_sweep_yaw_offset` (the two stay consistent);
  - the parked-hold mean of the retired anchor estimator `calculate_yaw_offset`. That one is a diagnostic on the constellation path; a hold straddling 359.9/0.1 would have averaged to about 180°.
- **Paths that needed no change.** `angular_span_deg` (the `MIN_ARC_DEG` check) is already circular. The stamped `bb_yaw` (firmware-unwrapped, may be negative) passes through unchanged.
- **`mocap_node._on_bb_heartbeat`**: records the yaw after the start edge and before the end edge. The first CALIBRATING sample is now in the window, and the first post-sweep sample still is, as before.

## Offline replay (the real `run_calibration`, the installed template)

**Replay method.** The replay feeds what mocap_node feeds:
- **Frames:** the BB markers `Ball Butler - 1..7` from each `/mocap_data` message received while calibrating, at the message stamp. That stamp is the QTM frame time mapped to ROS by mocap_node. There is one frame per distinct stamp, and labels are dropped for the constellation.
- **Heartbeat yaw:** stamped at the bag's `log_time`, which is the recorder's receive time standing in for the node's `get_clock().now()`.
- **Window:** from the first CALIBRATING heartbeat to the first heartbeat after it.

Scripts: `/tmp/claude-1000/…/scratchpad/wrapfix/{extract,replay,gate_sim,frames_stats}.py`.

| sweep | first recorded yaw (old / new ordering) | before the fix | after the fix: yaw offset | lag | n_moving | lag residual | template residual | frames (moving) |
|---|---|---|---|---|---|---|---|---|
| 1 | 357.72 / 357.70 | TOO_FEW_MOVING (0) | +0.4124° ±0.047 | 128.2 ms | 38 | 0.19° | 0.30 mm | 1246 (742) |
| 2 | 359.98 / 359.76 | TOO_FEW_MOVING (0) | +0.6095° ±0.063 | 125.2 ms | 38 | 0.24° | 0.24 mm | 1178 (715) |
| 3 | 359.96 / 359.52 | TOO_FEW_MOVING (0) | +0.4920° ±0.051 | 124.3 ms | 36 | 0.23° | 0.34 mm | 1101 (610) |
| 4 | 0.11 / 359.81 | OK +0.4533° (live +0.4866°, 129.0 ms, 37) | +0.4533° ±0.049 | 125.7 ms | 38 | 0.22° | 0.36 mm | 1217 (713) |
| 5 | 359.61 / 359.54 | TOO_FEW_MOVING (0) | +0.4718° ±0.067 | 127.2 ms | 38 | 0.26° | 0.35 mm | 1160 (679) |
| 6 | 359.63 / 359.61 | TOO_FEW_MOVING (0) | +0.4624° ±0.057 | 129.1 ms | 39 | 0.22° | 0.29 mm | 1214 (725) |
| 7 | 0.09 / 359.72 | OK +0.5075° (live +0.4938°, 127.5 ms, 37) | +0.5075° ±0.046 | 125.4 ms | 38 | 0.19° | 0.27 mm | 1200 (713) |

**What the replay shows.**
- **All seven sweeps succeed after the fix:** mean +0.4870°, SD 0.062°, range 0.197°. The before/after columns use the pre-fix module with the old ordering and the fixed module with the new ordering.
- **The heartbeat reordering changes no number.** The extra first sample is parked, so it is not a moving sample.
- **Replay vs live.** Sweeps 4 and 7 reproduce the live pass/fail but not the live values to better than 0.033° / 0.014°, and the lag reads 2–3 ms shorter. This is the instrument, not the fix:
  - **Receive stamps.** The replay stamps the heartbeat at the recorder's receive time, not the node's.
  - **Constant shift.** Shifting all heartbeat stamps by +3 ms moves the lag by +3 ms and the offsets by ≤ 0.005°.
  - **Jitter.** Adding 2 ms RMS of receive jitter moves the 7-sweep mean by ±0.013° and individual sweeps by several hundredths.
- **The spread is wider than expected.** It is wider than the 0.03–0.05° hoped for and than the template's leave-out repeatability (0.031°, from the earlier 7-sweep bag). Against each sweep's own σ (0.046–0.067°) it is consistent: χ²/dof ≈ 1.2, and sweep 2 is the largest outlier at +1.9σ.

## Risks seen, NOT changed here

- **The gate refuses a good sweep.** Replaying the gate with the reference updated on each accept, sweep 1 (+0.412°) is accepted and sweep 2 (+0.610°) is refused with `CALIBRATION_INCONSISTENT`: Δ 0.197° against a limit of max(3·0.063, 0.15) = 0.189°. This happens both from no state file and from the current state file (0.4938°). Sweeps 3–7 are accepted. With the heartbeat source, the gate's limit is about 3σ of a single sweep, but the difference of two sweeps has √2 times that spread. A refusal of a good sweep is about a 1-in-30 event per pair at these σ.
  **Changed in the follow-up commit (2026-10-10):** the limit is now max(3·√(σ_new² + σ_ref²), 0.15°), the reference's `yaw_offset_std_deg` read from the state file (0 if absent). For this pair it is 0.236°, so sweep 2 is accepted; `test_gate_accepts_the_sweep_pair_it_refused_on_2026_10_09` pins it.
  **Correction (2026-10-10, found by the keep-last-good unit):** `mocap_node._gate_reference` built the reference without the state file's `yaw_offset_std_deg`, so the live node still ran with σ_ref = 0 (the direct `check_calibration_consistency` tests could not see it). Fixed after the merge of the keep-last-good and spread branches; `test_gate_limit_uses_the_state_files_sigma` drives the node with a state file carrying σ. Gate on the merged tree (`./run_tests.sh --full`, 2026-10-10 01:30–01:36): **PASS — 6273 passed, 9 skipped, 1 xfailed + 6 serial**; the one earlier failure (the GUI replay allowlist missing `/bb/calibration_attempt`) was fixed by adding the topic to `ros_ws/gui/replay/schema.py`.
- **Lag is per session.** It is 124–129 ms in this sitting, against 80–85 ms in the earlier 7-sweep bag and 94 ms in session A. It is well inside the −50…300 ms grid, so there is no edge risk.
- **The stamped yaw was not live.** `/bb/axis_estimates` carried only `bb_pitch` and `bb_hand` (11478 messages), so every sweep used the heartbeat. The can-bridge was not on FW 28 at this sitting.
- **Missed QTM frames.** The calibration windows hold 162–181 distinct frame stamps per second, and 9–16 % of the 200 Hz publishes duplicate a stamp. The 200 Hz timer snapshot misses some QTM frames. The largest gap between frames was 34 ms, below the 50 ms break, so nothing was lost to the track.
- **QTM clock offset.** `/qtm_clock_offset_sec` moved by 5.9 ms over the 2-minute bag. The per-sweep lag fit absorbs this.
- **The solver blocks the node.** The solver runs about 1.4 s inside the heartbeat callback (log: sweep end to result). Heartbeats queue meanwhile; it does not touch the window.

## Verification

- Scoped (`~/Desktop/PDJ_venv/venv/bin/python -m pytest -q tests/ros/test_bb_calibration_constellation.py tests/ros/test_mocap_node_yaw_gate.py tests/ros/test_mocap_node.py tests/ros/test_bb_calibration_{arc_span,consensus,coplanar}.py`, run 2026-10-10): **136 passed, 1 skipped**. On the pre-fix code, the 4 new tests (wrapped series at −0.3° and −2.3°, canonical no-op, first CALIBRATING sample) fail.
- Full gate (`./run_tests.sh --full`, run 2026-10-10 00:06 local, the code and tests above in the tree): **PASS — 6201 passed, 9 skipped, 1 xfailed (parallel, 310 s) + 6 passed (serial); total 336 s**.
- Follow-up (the gate limit with both σ, two new tests, doc lines): scoped `pytest -q tests/ros/test_bb_calibration_constellation.py tests/ros/test_mocap_node_yaw_gate.py tests/ros/test_mocap_node.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py` 2026-10-10 00:14: **114 passed**; full gate `./run_tests.sh --full` 2026-10-10 00:16–00:22: **PASS (parallel 313 s, serial 24 s, total 337 s)**; `colcon build --packages-select jugglebot` rebuilt the install (copy install, files match).
- Replay (`replay.py`, the bag above, run 2026-10-10): before 2/7 sweeps succeed, after 7/7 (table above).
