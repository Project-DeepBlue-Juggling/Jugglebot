---
title: BB yaw-offset sweep spread on the stamped-yaw bag is the mocap clock, not the stamped yaw — the QTM->ROS offset (an EMA of receive latency) wandered 4–59 ms per sweep on a loaded Jetson; the estimator now refuses such sweeps (CONSTELLATION_MOCAP_CLOCK) and the yaw source is a parameter defaulting to the heartbeat
type: investigation
date: 2026-10-10
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - tests/ros/test_bb_calibration_constellation.py
  - tests/ros/test_mocap_node_yaw_gate.py
  - logbook/2026-10-10-bb-yaw-offset-spread-stamped-source.md
  - logbook/INDEX.md
subsystem:
  - ros
  - tracking
tags:
  - kinematics
  - testing
---

# BB yaw-offset spread on the stamped-yaw bag is the mocap clock, not the stamped yaw

## Symptom

- **The sitting.** The first sitting with the stamped 100 Hz yaw (bag `~/Desktop/rosbags/2026-10-10_00-24-06/`, bridge FW 28 / BB FW 6, mocap_node log `~/.ros/log/python3_1485201_1791552250549.log`) ran 7 sweeps.
- **The scatter.** The offsets were +0.96 / +0.93 / +0.56 / +0.65 / +0.43 / +0.21 / +0.66°: **SD 0.27°**, against 0.062° for the heartbeat on the 2026-10-09_23-49-07 bag (BB untouched in between).
- **Other symptoms:**
  - lag residual 0.31–0.74° RMS (2–3× the heartbeat's);
  - fitted lag −8…−16 ms where the sample age predicts about +3 ms;
  - formal σ 0.04–0.19° (χ²/dof 9.4).

## Investigation

**Analysis record.** Scripts and outputs are in `~/bb_calibration_sessions/yaw_spread_20261010/` (`replay.py` emulates mocap_node as in `2026-10-09-bb-calibration-heartbeat-yaw-wrap`). Five hypotheses were pre-registered before any result.

- **H3, decisive: replay bag 2 with the heartbeat yaw.**
  - The heartbeat gives **SD 0.252°**, the stamped yaw at its stamp 0.272°, and the stamped yaw at its receive time 0.258°.
  - The per-sweep values of heartbeat and stamped correlate at r = 0.95.
  - Two independent yaw timing paths carry the same error, so the cause is in what they share: the mocap frames.
- **Yaw side is clean (no mocap involved).** The heartbeat runs 121–128 ms behind the bridge-stamped yaw, IQR 1–5 ms per sweep, and the two legs agree.
- **H1 → the mocap clock.**
  - **How a frame is stamped.** Each frame's stamp is its QTM time plus `mocap_interface._update_qtm_clock_sync`'s offset, an **EMA (α = 0.01 per packet, about 0.3 s) of the packets' receive latency**.
  - **How far it wandered.** On bag 2, `/qtm_clock_offset_sec` steps by ±10–54 ms between its 1 Hz samples, and it spans 4–59 ms within a sweep. On bag 1 it spans 0.6–2.3 ms.
  - **Signs of load.** Receive − stamp spreads −27…+48 ms; 41 % of `/mocap_data` messages are re-published snapshots (bag 1: 12 %); the `/mocap_data` publish rate fell from 181 to 118 Hz; distinct frames per sweep halved.
  - **The legs disagree.** Per-leg lag fits differ by up to 27 ms (return vs outbound).
  - **The published offset predicts φ.** The bias −mean(ω·e) predicted from the interpolated 1 Hz offset correlates with φ at r = 0.77, slope 0.91.
  - **Re-timing pulls bag 2 onto bag 1.** Subtracting the interpolated 1 Hz offset from the frame stamps moves the bag-2 mean from +0.654 to **+0.485°** for both sources (bag 1: +0.487°) and lowers the SD to 0.16–0.17°.
  - **One null result.** The instantaneous residual-implied timing (−r/ω) does not correlate with the 1 Hz offset (r = 0.03, n = 28). The bias-carrying part is absorbed into φ, and the baseline pose residual (about 4 ms equivalent) masks the rest.
- **H2, sample age: rejected as a bias.** A −3 ms shift moves φ by ≤ 0.001°. Even and odd sample subsets agree within ≤ 0.03° in 6 of 7 sweeps.
- **H4, geometry: same in both bags.** Marker visibility, frame-fit RMS (median 0.6–0.8 mm), template residual and rejected frames match between bags, and sweeps 1–2 (high) look like 5–6 (low). Only timing differs (max frame gap 62–121 ms vs 17–34 ms).
- **H5, the timing-free parked anchor** gives the same mean on both bags: +0.766° (SD 0.053) and +0.761° (SD 0.063). BB, the QTM calibration and the template did not change.

**Mechanism.**
- A timing error e(t) that the per-sweep lag fit leaves behind biases φ by −mean(ω·e), about ω·(e_out − e_ret)/2. That is 0.3° for 10 ms at 67°/s.
- It is constant along each leg, so it does not show in the lag residual. The Jetson was loaded (a test suite finished at 00:22 and another session was running tests), which stretched the receive latency that the EMA averages.

## Discussion

- **Why not the obvious suspects.**
  - The brief's prime suspect, the stamped yaw's timing or its 0–6 ms age, is cleared by H3 and H2.
  - So is the brief's fallback: making the heartbeat the default does *not* fix this bag (heartbeat SD 0.25°).
- **What the fallback is for.** It is still taken because the pre-registered criterion requires it. No offline-verifiable fix brings bag 2 to SD ≤ 0.08°: re-timing by the 1 Hz offset reaches 0.16°, and regridding gets worse. The heartbeat is also the only source verified to repeat (bag 1).
  - The stamped source is *unverified, not shown bad*: its only bag had a broken clock.
- **Why a gate.** The fault is detectable from the frames alone. Raw QTM frame times lie on a k·P grid, and a wandering mapping moves consecutive stamp differences off it. The fraction more than P/8 off grid reads:
  - bag 1: 0.0–0.4 %;
  - bag 2: 11.5–35 %;
  - 182 six-second windows of five other bags (2026-10-05..09): median 0.0 %, p90 ≤ 0.9 %, one window at 7.5 % that spans a QTM sync reset.
  - The limit is 5 %.
- **Why refuse rather than inflate σ.** σ is honest where the clock is clean (bag 1 χ²/dof 1.2). An inflated σ would also widen the consistency gate's 3·√(σ²+σ_ref²) and let a biased value through.
- **Rejected fixes.**
  - Regrid by rounding: ambiguous k once the offset moves more than P/2 per interval, and it stretches time when P is misestimated (bag 1 degraded to mean +0.36, SD 0.089).
  - Per-leg lag fits: φ and lag are collinear within a leg.
- **"Unwander", not shipped.** It subtracts the cumulative grid residual, detrended. In simulation it cuts the bias 5–10×; it leaves bag 1 unchanged (SD 0.064); it cannot rescue bag 2 (wrap errors, SD 0.39). It is left for an owner decision because it changes every clean value by up to 0.04° with only simulation behind it.
- **Accepted blind spot.** A simulated mild load still passes the gate: one QTM packet per frame with 1–3 stalls/s of 20–80 ms gives ≤ 1.3 % off grid but up to 0.18° bias (probe `/tmp/probe_bbclock`, not committed). The gate catches the observed regime, not every load.
- **The root fix is out of scope.** It is the clock sync itself (a lower-envelope / minimum-latency offset with a slew limit, or raw QTM time carried into the calibration window). `mocap_interface` is shared with ball tracking and catch timing, and this unit was confined away from mocap_node's accumulation path.

## Fix

- **Estimator gate.** `bb_calibration.mocap_clock_off_grid(t_frames)` computes the metric (P estimated from the data, minimum over P0/n with n = 1..4 so snapshot-skipping never inflates it). `estimate_sweep_yaw_offset` raises **`CONSTELLATION_MOCAP_CLOCK`** above `MOCAP_CLOCK_MAX_OFF_GRID = 0.05`, for either yaw source. `SweepYawEstimate.frame_clock_off_grid` is reported in the summary ("mocap clock X % off grid").
- **Yaw-source parameter.** `mocap_node` gains `bb_yaw_source` = `heartbeat` (default) | `stamped` | `auto`.
  - `stamped` without ≥ 100 stamped samples is refused (`BB_YAW_SOURCE_UNAVAILABLE`), not substituted.
  - An invalid value is rejected at set time and refused at calibration (`BB_YAW_SOURCE_INVALID`).
  - `auto` is the previous behaviour.

## Verification

- **Replay** (`replay.py`, both bags, run 2026-10-10):
  - bag 1, heartbeat: before = after, 7/7, mean +0.4870°, SD 0.0619°, off grid 0.0–0.4 %;
  - bag 2, stamped / heartbeat / stamped-at-receive: before 7/7 published with SD 0.272 / 0.252 / 0.258°; after, 7/7 refused `CONSTELLATION_MOCAP_CLOCK` (11.5–35.0 % off grid).
- **Scoped tests** (`pytest tests/ros/test_bb_calibration_constellation.py tests/ros/test_mocap_node_yaw_gate.py -q`, run 2026-10-10): **60 passed**.
- **Full suite** (`./run_tests.sh --full`, run 2026-10-10): **PASS** — parallel 6212 passed, 9 skipped, 1 xfailed in 336 s; serial 6 passed.

## Next

- **Owner sync on the clock-sync fix** in `mocap_interface` (it affects ball tracking).
- **Re-run 7 sweeps on an unloaded Jetson** with `bb_yaw_source:=auto`. If they pass the gate with stamped SD ≤ 0.08°, make the stamped source the default.
- **Do not run test suites during a calibration sitting.**
