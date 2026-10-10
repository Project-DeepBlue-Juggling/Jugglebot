---
title: QTM->ROS clock offset is now the minimum receive latency over a 16 s window (drift term, 1 ms/s slew limit) instead of an EMA of every packet's latency — bag-2 within-sweep offset wander 15–85 ms -> 0.06–0.69 ms in replay; BB calibration SD 0.27° -> 0.094°, off-grid 11–35 % -> 0 %
type: feature
date: 2026-10-10
status: in-progress
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/qtm_clock_sync.py
  - ros_ws/src/jugglebot/jugglebot/mocap_interface.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - ros_ws/src/jugglebot_interfaces/msg/MocapDataMulti.msg
  - tests/ros/test_qtm_clock_sync.py
  - tests/ros/test_mocap_interface.py
  - logbook/2026-10-10-mocap-min-latency-clock-sync.md
  - logbook/INDEX.md
subsystem:
  - ros
  - tracking
tags:
  - testing
---

# QTM->ROS clock offset from the minimum receive latency

## Summary

`MocapInterface._update_qtm_clock_sync` used to keep the QTM->ROS offset as an EMA (α 0.01/packet) of
`receive − QTM time`. That is the clock offset plus the mean receive latency, so the Jetson's queueing
delay was in every mocap frame stamp. Under load (bag 2026-10-10_00-24-06) it wandered 4–59 ms within a
7 s BB sweep (`2026-10-10-bb-yaw-offset-spread-stamped-source`). The new estimator,
`jugglebot.qtm_clock_sync.MinLatencyClockSync`, follows the lower envelope instead, since queueing only
ever adds delay. The accessors, `/qtm_clock_offset_sec` and the units are unchanged. **Status in-progress:
nothing has run live; the catch-timing effect is untested.**

## Motivation

The wander biased the BB yaw-offset calibration by up to ±0.4° (per-sweep SD 0.27° vs 0.06° unloaded).
The calibration's grid gate (`CONSTELLATION_MOCAP_CLOCK`) refused all 7 sweeps of that bag. The same
stamps time ball tracking and catch prediction.

## Design

How it works (constants and their rationale are in the module):

- **Window.** 0.5 s bins of receive time, each keeping its minimum, over a 16 s window.
- **Drift term.** Each time a bin closes, the slope of the window's lower support line (Moon et al.
  1999: the lower-hull edge spanning the bins' mean time) is clamped to ±200 ppm and low-passed
  (τ 30 s).
- **Level.** The highest line with that slope that lies on or below every bin minimum.
- **Output.** Slew-limited to 1 ms/s after a 16 s startup. During startup the output is the estimate
  itself, so it is usable from the first packet.
- **Outlier clip.** After startup a sample lowers its bin to at most 0.5 ms below the envelope.
- **Re-anchor** on a QTM restart (time backwards / > 5 s jump), 5 packets > 20 ms below the output,
  or 2 s of packets all > 250 ms above it.
- **Diagnostics.** These keys are merged into `get_qtm_sync_status()` and logged at DEBUG at 1 Hz by
  `mocap_node`: excess latency, envelope − output, window fill, drift, slew clamps, outlier clips,
  re-anchors, restarts. They stay out of `mocap/status` because its five-key set is pinned.

## Discussion

**Why a support line with a low-passed slope:**

- A pure window minimum lags a drifting offset by drift × window: 0.8 ms at 50 ppm over 16 s.
- A per-window linear fit of the envelope extrapolates its own slope noise. A first variant used the
  raw hull slope and measured ±60–80 ppm of slope noise, which put 1–3 ms into unloaded bags.
- The clocks drift by 3–10 ppm, and that drift changes over minutes, so the 30 s low-pass loses no
  tracking. A median-of-hull-edges slope was tried as well and was no better.

**Why 16 s.** The window was chosen from replays at 4/6/10/16 s. On the reconstructed bag-2 traces a
near-floor packet arrives only every few seconds, and the worst within-sweep range was 3.8/3.4/2.1/0.69 ms
respectively. 16 s changed nothing unloaded.

**Why the outlier clip.** The replay found isolated packets 1.2–2 ms below an otherwise steady floor,
about one per 15 s. A plain minimum followed each of them for a whole window. They may be real fast
frames or reconstruction artefacts; their neighbours do not show the signature of a QTM-grid timestamp
error.

**Accepted tradeoff: frame stamps move EARLIER.** The new offset sits about 3 ms below the old EMA on an
unloaded box (the mean minus the floor of the receive latency), and 15–35 ms below it under load.
Consequences:

- The BB lag fit absorbs the shift.
- The ball tracker's fitted landing times and trajectory_node's catch timing shift by the same amount.
- No compensating constant was added, because it would re-introduce the mean.
- **An owner check of catch timing on the next sitting is the open item.**

## Implementation

The on_packet race (the offset is updated before the frame's time is written) is left as it was. With
the slew-limited output it can move a stamp by at most one slew step, 3.3 µs.

## Verification

Offline evidence lives in `~/bb_calibration_sessions/clock_sync_20261010/`. These are reconstruction
and replay checks only; no hardware was involved.

- **Reconstruction from the bags** (`recon_lib.py`). A Viterbi over QTM frame numbers yields two
  latency traces:
  - **m** inverts the EMA. It is exact for single-update gaps; a multi-update gap gives the EMA-weighted
    mean of its packets.
  - **y** is the recorder log time minus the QTM time, a strict upper bound on the latency.
- **The recorded frame offsets match `/qtm_clock_offset_sec`** within 0.5 µs (p95, all three bags).
- **Today's EMA fed the independent y trace reproduces the recorded offset** to within the pipe delay
  (corr 0.94–0.99; bag 2's 87 ms wander tracked at SD 2.1 ms).
- **Within-sweep offset range (peak-to-peak), old EMA → new on m / new on y:**

  | bag | old EMA | new (m) | new (y) |
  |---|---|---|---|
  | 2026-10-09_23-49-07 | 1.1–4.6 ms | 0.03–0.40 ms (1.49 in the replay's startup) | 0.01–0.19 ms |
  | 2026-10-10_00-24-06 | 15–85 ms | 0.06–0.69 ms | 0.07–0.40 ms |
  | 2026-10-10_10-32-31 | 0.7–1.7 ms | 0.06–0.66 ms | 0.02–0.34 ms |

- **BB calibration replay of bag 2 re-stamped with the new offset.** It used the repo's
  `estimate_sweep_yaw_offset` and the installed template.
  - **Pre-registered expectation:** the SD falls from 0.27° toward 0.06–0.08°, and the off-grid
    metric goes below 5 %.

  | | stamped yaw | heartbeat yaw |
  |---|---|---|
  | SD before | 0.272° | 0.252° |
  | SD after (m / y trace) | **0.094° / 0.095°** | 0.117° / 0.111° |
  | mean after | +0.466° | +0.48–0.49° |

  - **The SD expectation was half met.** It fell 2.9× but did not reach 0.06–0.08°.
  - **The off-grid expectation was met:** 11.5–35 % → 0.0 %, so all 7 sweeps now pass the gate.
  - **The mean moved onto the unloaded value:** bag 1 heartbeat +0.487°; bag 3 +0.476° (heartbeat)
    and +0.488° (stamped).
  - **Sweep 7 stays at +0.656° in every variant**; without it the stamped SD is 0.05°. It is reported,
    not excluded.
  - **Unloaded bags 1 and 3 move by ≤ 0.008° in SD and ≤ 0.003° in mean.**
- **Synthetic traces** (`tests/ros/test_qtm_clock_sync.py`; seed 21):
  - Old EMA, 7 s-window peak-to-peak: bursty load (1–3 stalls/s of 20–80 ms) 10.7 ms; bag-2-like heavy
    load (wander 4–59 ms) 63.9 ms.
  - New estimator, same metric: 0.01 ms bursty, 0.06 ms heavy.
  - ±50 ppm drift is tracked within 0.45 ms, including under heavy load.
  - The slew bound holds on every trace.
  - A QTM restart re-anchors in one packet (≤ 0.3 ms error from 1 s after it).
  - Clock steps re-anchor.
  - 120 s of heavy load give no spurious re-anchor.
- **Scoped tests** (run 2026-10-10): `pytest tests/ros/test_mocap_interface.py tests/ros/test_qtm_clock_sync.py
  tests/ros/test_mocap_node.py tests/ros/test_mocap_node_keep_last_good.py tests/ros/test_mocap_node_yaw_gate.py
  tests/ros/test_mocap_status.py tests/ros/test_clock_offset.py tests/ros/test_trajectory_node_console.py
  tests/ros/test_install_segment.py -q` → **168 passed in 71.10 s**.
- **Full gate** (`./run_tests.sh --full`, run 2026-10-10 11:29–11:38): **PASS** — parallel 6391 passed, 9 skipped, 1 xfailed in 364.67 s; serial 6 passed; total 392 s. Only this entry and its INDEX row were edited afterwards; the tests that read them were rerun (`pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_plans_index.py -q`, run 2026-10-10: **111 passed in 0.99 s**).


**Merge into skill-stack (0c63c664, 2026-10-10).** Gate on the merged tree: two runs under concurrent load (an 8-minute `colcon build` of `jugglebot_interfaces`, then isolation reruns) failed only on `tests/ros/test_skill_node.py::test_installer_reports_service_unavailable` (both runs) and once on `test_teensy_bridge_node_setpoint.py::test_hand_step_violation_not_sent` — neither file is touched by this branch, both pass alone (8/8) and the skill-node file passes 3/3 under `-n 4` on an idle box on both the pre-merge (6ca9494c) and merged trees, and the first is a pre-existing load-sensitive flake (FAILED in the gate logs of 2026-09-30, 10-04 and 10-05). Third run, nothing else on the box, 12:04–12:10: **PASS — 6394 passed, 9 skipped, 1 xfailed (319 s) + 6 serial**. Install rebuilt (`colcon build --packages-select jugglebot_interfaces jugglebot`).

## Open Questions

- **Live catch timing with stamps about 3 ms earlier is untested.** So is anything tuned against the old
  stamps.
- **The next sitting can confirm the estimator live.** Its DEBUG line should show drift of a few ppm,
  window fill ≈ 1, and few slew clamps; then rerun 7 sweeps under load.
- **What are the isolated sub-floor packets?**
- **What makes bag-2 sweep 7 an outlier?**
