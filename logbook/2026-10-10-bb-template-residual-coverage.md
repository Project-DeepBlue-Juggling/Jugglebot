---
title: BB template residual is pose-binned, so QTM's pose-dependent reconstruction error at BB's location no longer reads as a moved marker; the 5 sweeps of 2026-10-10_10-32-31 refused TEMPLATE_RESIDUAL now pass, a real displacement still refuses
type: bugfix
date: 2026-10-10
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - tests/ros/test_bb_calibration_constellation.py
  - logbook/2026-10-10-bb-template-residual-coverage.md
  - logbook/INDEX.md
subsystem:
  - ros
  - tracking
tags:
  - kinematics
  - testing
---

# BB template residual is pose-binned: QTM reconstruction error at BB's location no longer reads as a moved marker

## Problem

The owner ran 5 calibration sweeps on 2026-10-10. The bag is `~/Desktop/rosbags/2026-10-10_10-32-31/` and the mocap_node log is `~/.ros/log/python3_1817015_1791588754519.log`. The yaw source was the heartbeat, the Jetson was unloaded, and the mocap clock was 0.0–0.1 % off grid.

- **All five were refused** with `TEMPLATE_RESIDUAL` at 0.57–0.64 mm, against the 0.5 mm gate.
- **The yaw offsets were fine.** Their mean was +0.485° (SD 0.067°). Bag 2026-10-09_23-49-07 (7 sweeps, same placement, residual 0.24–0.36 mm) gave +0.487°.
- **The owner asked** whether the constellation should require only ~5 visible markers, as the circle-centre fit does.

Analysis scripts and outputs are in `~/bb_calibration_sessions/template_residual_20261010/`, with a summary in `REPORT.md`. The fix and its fallback were pre-registered in `PREREG.md` before any code changed.

## Root cause

- **Which pairs carry it.** On every sweep the largest pair is Q3–Q6 (0.56–0.64 mm; Q2–Q6 on sweep 4). Q2–Q6 is 0.51 mm and Q4–Q6 / Q1–Q3 are about 0.4 mm. Pairs involving Q6 grew the most from the good bag. The per-sweep SD of each pair is ≤ 0.09 mm (`permarker.txt`).
- **Visibility did not change.** Labelled visibility per marker was 0.86/1.00/0.56/1.00/0.44/0.85/0.72 on both bags. Markers matched per posed frame were 3–7 on both.
- **The pair error depends on pose.** Pair Q3–Q6 swings from +0.4 to −2.1 mm across the body yaw (−1.4…−2.1 mm at 15–45°, about 0 at 45–100°, −1.3 mm at 105–120°). The out and return legs agree bin by bin (`yawbins.txt`). The good bag has the same shape at about half the amplitude (−1.1…−1.4 mm at 15–45°).
  - A marker that moved on BB would shift a pair by the same amount at every pose. This is QTM's reconstruction error at BB's poorly covered location. Its window mean is what the old statistic read.
- **One shared pattern across sittings.** The 21-pair error vector correlates with the 23:49 bag's at r = 0.93 (scale 1.66), 0.88 for the 18:56 bag (scale 0.84) and 0.87 for the 00:24 bag (scale 0.50). Scaling that one pattern leaves 0.09 mm RMS of 0.28 mm. Moving one marker explains the change less well: the best case, Q6 moved 0.28 mm, leaves 0.084 of 0.139 mm (`pattern.txt`). Dropping any single marker still leaves 0.43–0.74 mm (`repose.txt`), so no one marker is "the bad one".
- **QTM's solution changes from sitting to sitting.** Seven static scene markers (unlabelled, stable to < 0.2 mm within each bag) shift 0.3–1.5 mm between every pair of sittings. The shift is non-rigid (0.3–0.5 mm RMS after a rigid fit), including 23:49 → 00:24, whose BB residuals were fine (`static.txt`). The BB markers' own QTM 3D residuals are 15–25 % higher than at 23:49 (Q4 1.19 vs 0.96 mm) and the same as the template-build bag 18:56. The posed BB origin moved only (+0.17, −0.26, +0.15) mm against 23:49. The ~1 mm the arc-fit position moved is that fit's own sensitivity to pose-dependent error.

**Verdict:** this is reconstruction bias from poor coverage, with a sitting-dependent amplitude. It is not a physically moved marker. Whether QTM was recalibrated between 00:24 and 10:32 cannot be read from the bags, and it does not matter: the frame moves between every sitting at this level.

## Discussion

- **What the residual does to the yaw** (`repose.txt`; mean offset over sweeps, 10:32 vs 23:49):

  | markers posed | 10:32 | 23:49 |
  |---|---|---|
  | all 7 | +0.476° | +0.487° |
  | without Q6 | +0.512° | +0.504° |
  | without Q3 | +0.491° | +0.456° |
  | without Q3 and Q6 | +0.532° | +0.467° |

  - No single-marker removal moves the 10:32 − 23:49 difference by more than 0.056° (Q5, the least-visible marker). The SE of a 5-sweep mean is about 0.03°.
  - Posing only the 5 best-seen markers (dropping Q3 and Q5) raises the per-sweep SD to 0.09–0.15°, because the poorly seen markers supply the leverage at the poses where they are seen.
  - So the refusal protected nothing here.
- **The owner's ≥ 5-marker question.** The constellation already poses every frame with ≥ 3 matched markers (`CONSTELLATION_MIN_MATCHED`), and visibility was identical to the good bag, so visibility did not refuse these sweeps. The template residual did. Requiring 5 would discard frames, and posing only the best-seen 5 is worse (above).
- **Rejected alternatives.**
  - *"Tolerate one bad marker"*: no single marker is bad, and the −Q6 residual is still 0.43–0.56 mm.
  - *Per-pair visibility floor*: it would drop the Q3/Q5 pairs, and those markers are the most yaw-sensitive to a displacement (up to 0.11°/mm radially).
  - *Rebuilt template*: nothing moved. A template fitted to this sitting would absorb one sitting's bias amplitude, which varies 0.5–1.7× between sittings. It would also need a re-pin of the gauge.
  - *Raised gate*: rejected. A displacement that biases yaw by 0.1° reads only 0.43–0.47 mm (window mean) in the weakest direction (Q3 vertical, 2.4 mm), so the gate has no headroom to give.
- **Sensitivity** (`displace.txt`, 23:49 bag, real coverage; yaw bias per mm of a marker moved on BB):
  - Q3/Q5 radial: 0.09–0.11°/mm.
  - Q1/Q2/Q3/Q4/Q5/Q6 tangential: 0.06–0.10°/mm.
  - QTM 4: 0.061°/mm tangential, ≤ 0.015°/mm radial or vertical.
  - So a QTM 4 move that biases yaw by 0.1° is 1.65 mm tangential and reads 1.7–1.8 mm on either statistic, 3.4× the gate. The single-anchor estimator would have taken 0.3° from a 0.6 mm move. The constellation takes 0.04° from it, and the gate refuses that move at about 0.6 mm.
- **Chosen fix.** Keep the gate and the template, and change the statistic to target the physical signature of a moved marker: invariance with pose.
  - Per pair: the mean error in each 15° bin of the posed body yaw (≥ 20 frames), then the median over bins.
  - A pair seen in < 3 bins keeps its window mean.
- **Trade-off accepted.** The median ignores an error that appears in fewer than half of a pair's pose bins. A moved marker shows in all of them. Per sweep, every pair has 4–9 populated bins of 9; the fewest is Q5–Q7, with 4–5 (`nbins.txt`). On displacements injected to bias yaw by 0.1° (capped at 3 mm; `binned_B.txt`), the binned statistic reads 0.44–3.25 mm on the 23:49 bag against 0.30–3.08 mm for the window mean. No injection there is refused by the window mean and passed by the binned one.
- **Detection gap, now explicit.** On the 10:32 bag the weakest injection, Q3 vertical (2.4 mm), reads 0.34–0.46 mm binned. Its 0.1° bias is below what the gate can see, as it was before. That gap is a coverage limit, not a statistic choice.

## Fix

- **`bb_calibration.py`**:
  - New method `ConstellationTrack.pair_residual_pose_binned_mm(mask, bin_deg, min_frames, min_bins)`, with constants `TEMPLATE_RESIDUAL_BIN_DEG = 15`, `TEMPLATE_RESIDUAL_MIN_BIN_FRAMES = 20` and `TEMPLATE_RESIDUAL_MIN_BINS = 3`.
  - `template_residual_mm` (read by the estimator's 1.0 mm `CONSTELLATION_RESIDUAL` and by the gate's 0.5 mm `TEMPLATE_RESIDUAL`) now reads the binned statistic.
  - The window-mean statistic stays as `template_residual_window_mm`, a diagnostic field on `SweepYawEstimate`, and is shown in its `summary()`.
  - The constant's comment carries the evidence.
- **Unchanged**: the thresholds, the template, the gauge pin, E(y), and `mocap_node`.
- **Tests** in `tests/ros/test_bb_calibration_constellation.py`:
  - A 3 mm error confined to 15–50° of yaw takes the window mean past 0.5 mm, but the binned statistic stays < 0.15 mm and the gate accepts.
  - A 0.8 mm displacement reads the same on both statistics and is refused even with `bb_moved`.
  - A sweep spanning less than 30° falls back to the window mean exactly.

## Verification

Replay is mocap_node emulation at the bag log time, heartbeat yaw. The gate reference is `~/bb_calibration_sessions/bb_calibration_last_accepted.json` (0.658°). Outputs are `before.txt` and `after.txt`.

| bag / sweep | offset | residual before (window) | after (binned) | gate before → after |
|---|---|---|---|---|
| 10:32 / 1–5 | +0.468 +0.389 +0.593 +0.478 +0.448 | 0.59 0.61 0.56 0.59 0.64 | 0.40 0.31 0.40 0.41 0.39 | 5 refused → 5 accepted |
| 23:49 / 1–7 | +0.412 +0.610 +0.492 +0.453 +0.472 +0.462 +0.508 | 0.24–0.36 | 0.26–0.34 | 7 accepted → 7 accepted |
| 18:56 / 1–7 | +0.428…+0.493 | 0.20–0.46 | 0.24–0.29 | accepted (after) |

The offsets are identical before and after; the estimator's yaw path did not change. The 00:24 bag (all sweeps still refused by `CONSTELLATION_MOCAP_CLOCK`) reads 0.22–0.42 mm binned.

- **Scoped** (`~/Desktop/PDJ_venv/venv/bin/python -m pytest tests/ros/test_bb_calibration_constellation.py tests/ros/test_mocap_node_yaw_gate.py tests/ros/test_perception_console_lines.py tests/ros/test_mocap_node_keep_last_good.py -q`, run 2026-10-10): **98 passed in 29.85 s**.
- **Full** (`./run_tests.sh --full`, run 2026-10-10): **PASS — parallel 6372 passed, 9 skipped, 1 xfailed in 406.10 s; serial 6 passed in 21.84 s** (log `temp/logs/run_tests_full_tres_20261010.log`)

## Seen, not changed

- **Axis-point gate margin.** The live reference is the 00:24 stamped-yaw calibration, made on a loaded Jetson. The replayed sweeps sit 1.1–1.3 mm from its position against a 1.5 mm limit, and the 18:56 sweeps 0.95–1.2 mm. QTM's sitting-to-sitting frame shift (0.3–1.5 mm on static markers) uses most of that limit.
- **The replay does not match the live run exactly.** Replayed offsets differ from the live ones by up to 0.05° (sweep 2: +0.389° vs +0.438°), because the replay uses the bag's log time, not the node's receive time.
- **The underlying coverage problem stands.** More camera coverage at BB, or a QTM recalibration, would shrink the bias amplitude itself.
