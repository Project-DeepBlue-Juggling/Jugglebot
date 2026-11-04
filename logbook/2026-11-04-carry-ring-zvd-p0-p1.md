---
title: "Carry ring suppression P0 + P1 (JB-102): the Platform mode is not one number (encoder 5.4 Hz, mocap 4.45 Hz), so ZVD rows at a provisional 4.9 Hz land behind carry_zvd and the identification row moves to the A/B Sitting"
type: investigation
date: 2026-11-04
status: in-progress
phase: "two-ball oval P4"
related_plan: carry-ring-zvd.md
related_issues: []
files_changed:
  - tools/probes/platform_mode_fit.py
  - tools/probes/README.md
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv
  - config/hardware_config.yaml
  - tools/admissible_sweep.py
  - tools/admissible_sweep_run.py
  - sim/skills_gate.py
  - tests/motion/test_cup_zvd.py
  - tests/fixtures/pose_flag_off_golden.npz
  - plans/active/carry-ring-zvd.md
  - map/feature-pre-release-hold-rung-release-from-a-moving-platform-hold-0.md
  - logbook/INDEX.md
subsystem: [motion, ros, config]
tags: [dynamics, performance, testing]
---

# Carry ring suppression P0 + P1 (JB-102)

## Summary

P0 fitted the Platform's lateral mode from the four bags on disk and its gate FAILED: the encoder and the mocap see different frequencies and the encoder fit over-predicts the separation excess about 2x. P1 was built anyway at provisional values (4.9 Hz, zeta 0.20, delay 20 ms): I-PLAN-16, eight ZVD equality rows on the pose carry behind `trajectory_op.carry_zvd`, a seed mode state, a `RING` refusal, a live parameter and a box stamp. Flag off is byte-identical. P2 (the box re-sweep, 55 families) is running; P3 now opens with an identification row. Plan: `plans/active/carry-ring-zvd.md`.

## Context

The 2026-11-03 Sitting's debrief (`2026-11-04-sitting-debrief.md`) found carried oval Throws landing +76 mm (Dwell 0.40) and +122 mm (0.35) long against +48 mm from rest. The minimum-energy carry decelerates into the release knot and a roughly 6 Hz Platform lateral mode is still swinging forward at separation. The plan's P0 hypothesis was that the 2026-10-31 hold rungs (a free-decay window) and the 2026-11-03 carried releases (a forced response) would give two estimates of one mode that agree within 10 %. That hypothesis is refuted below.

## Findings

**P0 numbers** (x axis; per-release fits, medians; `tools/probes/platform_mode_fit.py` on `2026-10-31_13-06-17`, `_13-13-19`, `_13-14-00` and `2026-11-03_22-29-00`, run 2026-11-04):

| estimator | source | bag | n good / n | f_n Hz | zeta | delay ms |
|---|---|---|---|---|---|---|
| forced, catch to release+0.25 s | encoder | 11-03 | 96/109 | 5.40 | 0.28 | 21.0 |
| forced | encoder | 10-31 | 99/233 | 5.42 | 0.14 | 22.4 |
| forced | mocap | 11-03 | 66/109 | 4.48 | 0.22 | 16.6 |
| forced | mocap | 10-31 | 41/233 | 4.41 | 0.20 | 18.3 |
| free decay | encoder / mocap | both | 2-7 windows | unusable | | |

- **Two-mode signature.** The encoder and the mocap frequencies differ by about 20 % on both days with non-overlapping IQRs, and the mocap-to-encoder excess ratio is 2.14 at Dwell 0.40 and 1.65 at 0.35. The structure seen by the ball rings lower than the motor side. A one-mode forced fit also leaves a 5-6 Hz residual of +/-1.3 mm around the release and over-predicts the post-release overshoot (+124 against +74 mm/s at +40 ms): one linear mode driven by commanded acceleration cannot fit both the carry lag and the post-release ring on the encoder. Candidate causes, none tested: leg current saturation in the 4-6 m/s^2 deceleration (the 10-31 bag ran 15 A, 11-03 ran 20 A), two modes, the Hand stroke's reaction at separation.
- **The excess prediction fails.** Measured encoder excess over release +20..+40 ms is +35.2 (0.40) and +73.8 mm/s (0.35). The pooled encoder fit predicts +104.7 / +132.1 (error +69.5 / +58.3), Model A +101.4 / +128.0, each release's own fit +103.2 / +121.5. Model A matches the mocap excess (+127 vs +122 at 0.35), so the debrief's landing-bias validation was a mocap-like match.
- **The hold rungs show zero excess.** On 10-31 at holds 0.100 to 0.050 the measured excess is about 0 on both sources (-2.8 at Dwell 0.45, +0.3 at 0.50); Model A predicted +25 at 0.50.
- **The free-decay estimator is not evaluable.** The composed Platform command is never acceleration-quiet around a release: the hold pins the plan-frame Platform but the FK command still carries +500..+1300 mm/s^2 of composition through it.
- **A composition ramp in the realised command (not resolved).** Every Throw's bag command has v_c going -9 to +52 mm/s over the last 60 ms before release and a -1100 mm/s^2 pulse at +10..+20 ms. Any offline ZVD evaluation must replay the composed FK command, not the plan-frame pose. Whether this ramp is intended is a follow-up.
- **P2: the rows empty the Columns pose cells (gate 2).** With the rows on, the Columns window at Dwell 0.27 is refused `RING` on every probe (18522 / 18522), so the Runbook's Columns canary cell, Apex 0.85-0.95 and the turned cells have no admissible box. Physical reading: Columns has no lateral carry, so the only lateral Platform excitation is the prescribed tilt sweep's lever-arm term; a 0.27 s window (11 knots, most pinned by the catch / release triples and the holds) cannot shape it around a 204 ms mode period, so the eight hard rows plus the jerk box are infeasible while the ring-less program solves.
- **y is unidentified.** Commanded carry acceleration is 345 mm/s^2 rms in y against 2950-3670 in x; the y fits rail 50-60 % of the time.

**P1 measurements at the live yaml values (4.9 Hz, zeta 0.20, delay 20 ms; x axis e' at release in mm/s; 2026-11-04)**, oval 150 mm, Apex 1.00, 600 / 8000 / 200000, from `scratchpad/zvd-brief/measure49.py` (the test file's helpers with the mode constants overridden; log `scratchpad/zvd-brief/measure49.log`). The committed test file itself still evaluates at the earlier 6.0 Hz / 0.17 / 15 ms constants (see Follow-ups).

| Dwell | flag off | on, nominal | on x0.85 | on x1.15 | off x0.85 / x1.15 | realised plan (on) |
|---|---|---|---|---|---|---|
| 0.50 | -29.1 | 0.00 | -0.8 | +6.4 | -37.4 / -22.7 | -0.005 |
| 0.40 | -54.7 | 0.00 | -14.1 | +16.0 | -79.7 / -43.2 | -0.020 |
| 0.35 | -84.8 | 0.00 | -40.4 | +28.3 | -120.7 / -61.9 | -0.035 |

| Dwell | flag | leg v | leg a | leg j | peak Platform abs(vx) |
|---|---|---|---|---|---|
| 0.40 | off | 239 | 1824 | 71311 | 656 |
| 0.40 | on | 275 | 3260 | 77138 | 725 |
| 0.35 | off | 276 | 2514 | 107956 | 760 |
| 0.35 | on | 316 | 5096 | 123041 | 820 |

Leg acceleration at 0.35 reaches 64 % of the 8000 cap and jerk 62 % of 200000; all inside the limits. At the earlier 6 Hz values P1a measured 18.8 mm/s at 0.35 x1.15 (the pinned xfail) and leg a 3734; the lower frequency makes the +/-15 % band wider in seconds, and the bound is missed at two cells (0.40 x1.15, 0.35 both sides). Core solve at Dwell 0.65 (the budget's binding cell): 42.3 ms off, 47.6 ms on, against 50 (P1b, OPENBLAS 1 thread, loaded box).

## Discussion

**Why proceed despite the failed gate.** The plan said: if the bags disagree, fly the identification row before P1 lands. The orchestrator decided otherwise (the owner has been asked and can overturn it): the ZVD rows are feedforward shaping that only needs the mode the ball sees to be at rest at release, and the rows' +/-15 % robustness band centred on 4.9 Hz spans 4.2-5.6 Hz, covering both the mocap (4.45) and the encoder (5.4) frequencies. Centring on either alone leaves the other at the band edge (5.4 is +21 % from 4.45); the old 6.0 Hz sits outside the band for the mocap mode. Nothing in P1 or P2 is stranded by a later re-fit: the frequency is config, and the box is swept with the flag as a stamp. The identification row is therefore scheduled at the P3 Sitting ahead of the A/B, and it is cheaper there (the robot is powered for the A/B anyway). The cost accepted: the P2 box is swept at values that may move, and a re-fit that moves the mode far outside the band would re-open the sweep.

**Why hard rows.** The plan's decision 2 stands: a window that cannot meet the rows is refused, honestly, and the box shows what it costs. A quadratic penalty never refuses, which is the weaker contract (a window could silently ship a ring). The soft penalty stays the pre-registered fallback if P2 loses the oval cell.

**Why ship a box that fails gate 2.** The box is an honest record of what the planner can do: an empty flag-on Columns cell means a flag-on Columns launch is refused loudly at the box, nothing is silently clipped, and the flag is off by default, so no flight changes until someone sets it. Not shipping would leave the repo without the oval cells P3 needs; the alternative, the pre-registered soft-penalty fallback and a second sweep (about 3.5 h), would run into the 04:45 Night Shift on the same box, and the fallback was written for a gate-1 failure (the oval cell), which passed. The remedy for Columns (accept, soft penalty, or assemble the rows only where the window can hold them) is a design choice with consequences for the two-ball patterns at Dwell 0.30-0.35, so it is put to the owner (plan § 8) with a recommendation: accept for P3, decide with the A/B data. P3 flies the Columns canary with the flag off. Cost accepted: the tested decision gate is recorded as failed, not reinterpreted.

**Why the RING re-solve.** When the QP is infeasible with the ring rows the original reason (INFEASIBLE or SINGULAR) says only that something does not fit, and the operator cannot tell a ring-caused loss from a window that was never feasible. Re-solving the same program without the ring rows on the refusal path only separates the two: a window that solves ring-less is refused `RING` with the residual it would have had (e, e' per axis); one that fails both ways keeps the original reason. The cost is zero on the accept path. The alternative, a "time it would need" estimate, is not derivable from equality rows and was dropped from the INVARIANTS text.

**Why a live switch instead of a yaml flip.** The yaml constant only takes effect at a relaunch. An A/B on one plant within one Sitting needs to flip the flag between Attempts, as `planner_mode` already can (I-PLAN-13 (d)). So `carry_zvd` rides on every InstallSegment request, the response reports what was solved, and the box carries a stamp that `select()` matches exactly; a box swept for the other setting refuses with a stamp-mismatch message instead of flying cells swept for something else. The trajectory_node deliberately has no second parameter for it: one switch only. The price is an `.srv` change, so a `colcon build` of `jugglebot_interfaces` and `jugglebot` is mandatory before the launch (a stale install raises AttributeError on every install), and all gated files change the box hash.

**The seed approximation.** A chained window's rows need the ring state it starts with. The seed integrates the mode ODE over the head's realised Hermite acceleration with the head's mode at rest at knot 0, so a head that itself started mid-ring does not fold in its own seed's decay. Exact seeding would need the head's own seed carried through every chain; P3 decides whether a flown seam shows the difference.

**Math kept in the gated file.** The mode kernels live in `cup_cycle.py` (a new module would not be in the box's gated-file list and would let a math change leave the box stamp valid).

## Fix

- `cup_cycle.py`: `PoseConfig` fields (`carry_zvd`, `mode_hz`, `mode_zeta`, `mode_delay_s`, validated), the closed-form mode kernels and exact free response, the ring rows after the skid block of `_pose_augment`, `_Program` ring fields, `_refuse_ring` and the ring-less re-solve in `plan_window`, `platform_mode_state`. `unified_cycle.py`: `RING` reason, `CycleState.mode_state`, `_seed_mode_state`, `PoseSpec.mode0`. `INVARIANTS.md`: I-PLAN-16 and a carry_zvd sentence in I-PLAN-13 (d).
- `hardware_config.yaml` (+ the six generated copies): `trajectory_op.carry_zvd: false`, `platform_mode_hz: [4.9, 4.9]`, `platform_mode_zeta: [0.20, 0.20]`, `platform_mode_delay_s: 0.020`.
- Live switch: `InstallSegment.srv`, `skill_node.py`, `trajectory_node.py`, `executor.py`, `segments.py`, `admissible.py` (the `carry_zvd` stamp and match), `tools/admissible_sweep.py`, `tools/admissible_sweep_run.py` (55 families: level 11, pose 22 flag off, pose 22 flag on), `sim/skills_gate.py`.
- `tools/probes/platform_mode_fit.py` (+ README row): the committed mode-identification probe.
- Tests: `tests/motion/test_cup_zvd.py` (new), the golden `tests/fixtures/pose_flag_off_golden.npz` captured from the pre-change tree (`git archive 18422c10`, one BLAS thread), the seam, budget, box and node tests extended for the flag.

## Verification

- 2026-11-04, P1b: `./run_tests.sh --full` in the worktree: 39 failed, 8162 passed, 54 skipped, 1 xfailed (406 s parallel, serial tail 9 passed). All 39 are the committed-box class, red until the P2 re-sweep: `tests/motion/test_skills_admissible.py` 2 (the stamp and gate_hash pins), `tests/ros/test_skill_node.py` 22, `test_skill_node_resend_param.py` 5, `test_skill_node_robot_frame.py` 2, `tests/sim/test_skills_gate.py` 8. The xfail is the strict 0.35 x1.15 pin.
- 2026-11-04, P1b scoped: `pytest tests/motion/test_cup_zvd.py test_splice_tilt_rate_seam.py test_pose_planner.py test_unified_cycle_budget.py test_admissible_sweep_run.py test_cup_cycle.py -q -p no:cacheprovider`: 175 passed, 1 xfailed.
- 2026-11-04, paperwork unit: `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 pytest tests/motion/test_cup_zvd.py -q -p no:cacheprovider -s -rxX`: 25 passed, 1 xfailed in 7.81 s (the file evaluates at 6.0 Hz constants). The 4.9 Hz table above comes from `measure49.py`, not from this run.
- Golden: flag-off plans byte-identical to the pre-change golden at Dwell 0.50 / 0.40 / 0.35 (passes under bare pytest and `run_tests.sh`; BLAS-thread dependent at 1e-10 mm without the pinned single thread).
- 2026-11-05, P2 sweep: `python tools/admissible_sweep_run.py --out temp/sweeps/zvd_2026-11-04` from the worktree, started 2026-11-04 20:08, finished ~23:55: 55 families, gate_hash `e8d303ae93bc`, 195 cells (level 31, pose 82 flag off, 82 flag on of which 25 EMPTY), limits 600 / 8000 / 200000 / 4500. Round b (ovalx040_pose, columnsx1_pose_zvd, columns_pose) identical, `CMP_STATUS 0`. All 113 old cells present flag off and value-equal (landing_xy_m, apex_m).
- Gate 1 (oval Dwell 0.40, Apex 1.00, rows on, at least 20 mm x room): **PASS**, 30 mm x and y, same as flag off; 0.35 at Apex 1.00 also 30 / 30, 0.35 at Apex 1.10 drops 30 to 10 mm x, oval 0.30 flag-on cells empty (3). Gate 2 (no Columns pose cell the Runbook flies lost): **FAIL**, the canary cell (Sep 125, Apex 0.95, Dwell 0.27) is empty with the rows on.
- Per-cell summary (rooms in mm, min(abs x_min, x_max)): oval 0.40-0.65 identical on / off; hop and self_toss identical; Columns Apex 1.03-1.52 30 / 30 both ways; Columns Apex 1.00 30 to 10 mm x (P1/P1, P2/P2); Columns Apex 0.85-0.95 EMPTY (P1/P1, P2/P2); turned 45 / 90 deg Columns 0.85-0.97 EMPTY; columns_highlow Apex 1.00 EMPTY, 1.05 30 to 8 mm x (y 30 to 25). No flag-on cell has more room than its flag-off twin. Refusal tallies: `CUP_SEATING_CONE` 0 and `POSE_WINDOW` 0 in every log; `RING` 18522 of 18522 in columns_pose_zvd and turned90_pose_zvd, 6174 in turned45_pose_zvd, 612 at oval 0.30; `HAND_LIMIT_ACC` 375 at ovalx035. Family names in the refusal tallies are inferred from log filenames.
- 2026-11-05 00:55, `./run_tests.sh --full` on the rebased commit (worktree of origin/main 4c996edf, the re-swept box installed): **8210 passed, 55 skipped, 0 failed in 403 s; serial tail 9 passed in 33 s; exit 0.** The earlier run before three box-dependent test updates (2026-11-05 00:30, same command) was 3 failed / 8207 passed: `test_p4_resweep_kept_every_pose_cell_room_and_added_one_ball_oval_dwells` (pinned 113 cells; now filters the flag-off cells and records the 25 empty flag-on cells), `test_param_backed_points_sit_in_their_own_admissible_cells` (collected the empty flag-on Columns cells; now flag-off only, the GUI default), `test_a_biased_plant_fires_the_forward_clip_in_a_small_hop_run` (its frozen pre-h4500 fixture box carries no `carry_zvd` stamp, which matches nothing; the test now stamps a tmp copy false).

## Follow-ups

- **The Columns decision** (plan § 8): accept flag-off Columns, the soft-penalty fallback with a re-sweep, or a "rows assembled only when the window can hold them" scope rule. Owner question outstanding.
- **Columns below Dwell 0.30 with the rows** and **the two-ball dwells (0.30-0.35)**: the same window-length limit applies; the P3 A/B data decides whether hard rows are worth it there.

- **The test file evaluates the old 6.0 Hz mode**, not the yaml 4.9 Hz values: `MODE_W`, `MODE_Z`, `MODE_TAU` and `ZVD_ON` are hard-coded. At 4.9 Hz the +/-15 % cells 0.40 x1.15 (16.0) and 0.35 x0.85 / x1.15 (-40.4 / +28.3) miss the 15 mm/s bound, so the xfail is one of three. Parametrise the test on the yaml values, then decide the bound.
- **The +/-15 % bound** (owner decision): accept about 19-28 mm/s worst case at 0.35 (against 62-121 flag off), or change the bound. ZVDD rows were probed and only move the excess to the other side.
- **y axis unidentified**: the identification row adds a y excitation; the y values copy x.
- **The realised-command composition ramp** (+52 mm/s at release, -1100 mm/s^2 pulse after) is in every Throw and is not modelled by the rows; find where it comes from and whether the rows should see it.
- **One mode or two**: if the identification row confirms separate encoder and mocap modes, the rows grow to two modes (config, not code) or the leg acceleration feedforward option is reopened (owner).
- **Re-fit the mode on every oval Sitting bag** with `platform_mode_fit.py` (the plan's running drift check).
