---
title: "BB calibration Phase 2 (template residual): the kappa step survives every marker subset, leave-one-out and label-free refit on 75 sweeps of 22 sittings - it is at BB, not in labelling or the template; label-free pickup has nothing to pick up; one residual statistic kept, reference 0.18-0.46 mm"
type: investigation
date: 2026-10-16
status: in-progress
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration)"
related_plan: bb-calibration-one-attempt.md
files_changed:
  - tools/probes/bb_calibration_replay.py
  - tools/probes/README.md
  - tests/ros/test_bb_calibration_replay.py
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - plans/active/bb-calibration-one-attempt.md
  - logbook/2026-10-16-bb-calibration-phase2-template-residual.md
  - logbook/2026-10-16-bb-calibration-phase1-gate-inversion.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
  - tools
tags:
  - kinematics
  - testing
  - docs
---

# BB calibration Phase 2: template residual, labelling and the controlled pair

## Summary

Phase 2 of `plans/active/bb-calibration-one-attempt.md`. The first deliverable was a measurement, not code: the 2026-10-16 10:51 and 15:08 bags (one wand calibration, one world frame, kappa 179.57 vs 179.26) re-solved with every marker subset, each marker left out in turn, and label-free pickup of every reconstructed marker near BB. Kappa keeps its two levels under every variant (gap 0.22 to 0.43 deg against 0.02 deg within a sitting), both sweep legs carry the step, the shelf heading is identical to 0.002 deg and the axis point moved under 0.5 mm. Extended to every bag (2026-10-10 to 10-16: 90 sweeps replayed, 82 accepted, 75 with a kappa, 22 sittings) the result is the same: the K2 epoch sits +0.23 to +0.38 deg above K1 and +0.25 to +0.47 deg above K3 in every variant. By the plan's criterion the step is at BB. The BB firmware (read for the owner) cannot be the cause: the encoder is read absolutely every sample, the zero offset is a compile-time constant, nothing is stored or homed. Label-free pickup is a null result on every bag (an unlabelled BB marker never exists in the moving window; what QTM does not label it has not reconstructed), so no pickup lands in production; the replay keeps it as a standing check. The template residual stays one statistic with its 0.90 mm advisory and 1.0 mm cap; its reference level under one wand calibration is 0.18 to 0.46 mm and is carried by marker Q3's pairs. No template re-fit (the per-marker body-frame residuals flip sign between sittings). Owner decisions 2026-10-16 (this session): keep the statistic and document the reference, no pickup in production, no re-fit, and the next step is the owner's BB-only power-cycle A/B with QTM untouched.

## Symptoms

From the Phase 1 entry (`2026-10-16-bb-calibration-phase1-gate-inversion.md` § Hardware reading): kappa read 179.572 / 179.565 at 10:51 (old code, after the ~10:28 re-wand) and 179.25 to 179.28 at 15:08 under the same wand calibration and world frame; two-state kappa (+0.31 deg on 10-13, -0.33 on 10-16), QTM reconstruction and the wand calibration excluded; BB state at power-up vs a per-session labelling/template artefact left open for this phase. Also open from the 10-14 investigation entry: only 5 of 7 BB markers label at rest (QTM3 in 4 bags, QTM5 never); rest pair separations vary 0.5 to 4 mm bag to bag; the shelf-marker residual against the as-built template went 0.004 to 0.30-0.35 mm and did not recover after the re-wand. A third sitting at 16:10 (launch `2026-10-16-16-10-37`, one sweep, accepted with no advisory) arrived during this session and reads kappa 179.339 with the lowest template residual on record, 0.18 mm. A fourth sitting at 16:23 (launch `2026-10-16-16-23-01`, one sweep, accepted with no advisory) reads kappa 179.24, template 0.25 mm.

## Diagnosis

Driver: a throwaway probe (`/tmp/probe_bb_phase2.py`, uncommitted by convention) that subclasses the replay's `NodeSim` to keep every sweep's raw solver inputs plus every marker of every frame, pickles one decode per bag, then re-solves each sweep through the SAME library calls (`track_constellation`, `estimate_sweep_yaw_offset`, `estimate_base_pose`, `bb_in_base`) under each variant. The committed form of the measurement is the harness extension in § Fix (run directory in § Verification).

### A. The controlled pair (and the 16:10 sweep)

Label coverage (fraction of window frames in which QTM labelled "Ball Butler - i"; rest = 15 s before the sweep):

| sweep | Q1 | Q2 | Q3 | Q4 | Q5 | Q6 | Q7 | rest Q3 | rest Q5 |
|---|---|---|---|---|---|---|---|---|---|
| 10-50-42 #1, #2 | 0.85 | 1.00 | 0.58, 0.65 | 1.00 | 0.43, 0.41 | 0.84 | 0.73 | 0.00 | 0.00 |
| 15-07-58 #1, #2, #3, #6, #7 | 0.85-0.87 | 1.00 | 0.94-0.98 | 1.00 | 0.43-0.45 | 0.82-0.84 | 0.72-0.73 | 0.00, 1.00, 0.99, 0.19, 0.96 | 0.00 (0.08 on #7) |
| 16-10-37 #1 | 0.85 | 1.00 | 0.94 | 1.00 | 0.44 | 0.83 | 0.72 | 1.00 | 0.00 |

Six of seven markers are labelled identically in both sittings; only Q3 differs, and only at rest (it labels at 0.88 to 0.97 of posed moving frames in all three).

Kappa (deg) per refit variant. common5 = Q1,Q2,Q4,Q6,Q7 (the markers labelled in every sitting); plane5 = Q3..Q7; LOO = the range over the seven leave-one-marker-out refits; LF = label-free pickup (every marker within 250 mm of the fitted axis point, any label or none); OUT / RET = the yaw-rising and yaw-falling legs at the fitted lag:

| sweep | prod | common5 | plane5 | LOO min..max | LF | OUT | RET | lag ms |
|---|---|---|---|---|---|---|---|---|
| 10-50-42 #1 | 179.572 | 179.510 | 179.516 | 179.488..179.613 | 179.570 | 179.640 | 179.504 | +5.4 |
| 10-50-42 #2 | 179.565 | 179.485 | 179.488 | 179.465..179.596 | 179.563 | 179.624 | 179.509 | +4.4 |
| 15-07-58 #1 | 179.254 | 179.161 | 179.221 | 179.165..179.295 | 179.254 | 179.285 | 179.219 | +1.5 |
| 15-07-58 #2 | 179.258 | 179.143 | 179.210 | 179.161..179.282 | 179.256 | 179.274 | 179.237 | +1.3 |
| 15-07-58 #3 | 179.262 | 179.160 | 179.214 | 179.160..179.289 | 179.262 | 179.299 | 179.226 | +1.4 |
| 15-07-58 #6 | 179.276 | 179.193 | 179.210 | 179.168..179.309 | 179.276 | 179.319 | 179.231 | +0.6 |
| 15-07-58 #7 | 179.271 | 179.161 | 179.242 | 179.221..179.286 | 179.272 | 179.290 | 179.252 | +2.5 |
| 16-10-37 #1 | 179.339 | 179.251 | 179.282 | 179.256..179.352 | 179.339 | 179.337 | 179.350 | -11.9 |

The 16:23 sweep (committed run only): prod 179.242, common5 179.166, label-free 179.242, lag -11.4 ms. The 10:51 minus 15:08 gap is 0.29-0.32 (prod), 0.29-0.37 (common5), 0.25-0.31 (plane5), 0.22-0.43 (any single marker dropped), 0.29-0.32 (label-free); within a sitting the scatter is 0.02. Both legs carry it (OUT 179.63 vs 179.30, RET 179.51 vs 179.23); the direction split itself (OUT minus RET, +0.04 to +0.14, -0.01 at 16:10) is a per-launch lag residue and is not the step. Per-marker fit of the production track over moving frames: RMS 0.5-0.8 mm at 10:51, 0.2-0.55 at 15:08, 0.4-0.6 at 16:10; body-frame mean residual y: Q3 +0.16 / -0.18 / -0.03 mm and Q6 -0.13 / +0.22 / +0.01 mm at 10:51 / 15:08 / 16:10 (sign flips between sittings). Shelf heading -179.127 / -179.128 (10:51) vs -179.129 / -179.128 / -179.129 (15:08, same frame); world axis point (-831.73, -399.34, 1739.28) vs (-831.90..-832.00, -398.96..-399.08, 1738.89..1739.00) mm, a 0.5 mm move with the in-plane part 0.36 mm, inside the 0.26 mm per-axis scatter. A 0.32 deg rotation of the body about a point 125 mm off the yaw axis would have moved it 0.7 mm; the data are a rotation about the yaw axis.

### B. Every bag, 2026-10-10 to 10-16

75 sweeps with a kappa (82 accepted of 90 replayed: 8 solver refusals, 7 accepted without the shelf), 22 sittings. Means per epoch and variant: prod, common5, label-free, OUT and RET from the committed harness run `temp/probes/bb_calibration_replay/20261016-181127` (`sweeps.csv`, per-sweep rows in its `summary.md` Phase 2 section); the leave-one-out range from the probe, whose K3 has 6 sweeps (before the 16:23 sitting). Epochs are the Phase 0 kappa epochs plus K3 (10-16 from 15:08):

| epoch | n | prod | common5 | LF | OUT | RET | LOO range (probe) |
|---|---|---|---|---|---|---|---|
| K0 (10-10) | 28 | 179.327 (SD 0.037) | 179.253 | 179.326 | 179.379 | 179.312 | 179.284..179.341 |
| K1 (10-11 .. 10-13 00:18) | 7 | 179.284 (0.030) | 179.266 | 179.281 | 179.357 | 179.224 | 179.230..179.303 |
| K2 (10-13 22:00 .. 10-16 10:51) | 33 | 179.590 (0.055) | 179.499 | 179.589 | 179.640 | 179.550 | 179.546..179.659 |
| K3 (10-16 15:08 on) | 7 | 179.272 (0.029) | 179.176 | 179.272 | 179.295 | 179.248 | 179.188..179.296 (n 6) |
| K2 minus K1 | | +0.306 | +0.233 | +0.308 | +0.282 | +0.326 | +0.256..+0.375 |
| K2 minus K3 | | +0.318 | +0.323 | +0.317 | +0.344 | +0.302 | +0.250..+0.470 (n 6) |

The probe's plane5 subset (Q3..Q7) reads 179.299 / 179.256 / 179.609 / 179.230 for K0..K3, gaps +0.352 and +0.379. No variant collapses the step. Q3's window label fraction varies by sitting (0.56 to 0.98) with no relation to the epoch (K2 sittings read 0.57, 0.63, 0.90, 0.92, 0.97); every other marker's fraction is constant across the probe's 21 sittings to 0.05 (Q1 0.85, Q2 1.00, Q4 1.00, Q5 0.44, Q6 0.82-0.92, Q7 0.72).

### C. Label-free pickup is a null result

Over the 82 accepted sweeps of the committed run, 587,243 unlabelled points lay within 250 mm of the axis point and 46 of them (0.008 %, in 11 sweeps, most in the two 10-14 evening bags) came within 3 mm of a posed BB marker; per-marker visibility of the posed track is identical with and without pickup in 80 of 82 sweeps (largest change 0.009), kappa changes by at most 0.014 deg (10-11 15:14 #1, the node-stall sweep), and the label-free solve never refused. On the pair, about 5 unlabelled points per frame lie within 250 mm of the axis point, and their minimum distance to any posed BB marker is 54 mm or more (5th percentile) with a median of 103-107 mm: none within 3 mm, over about 80k points. They are the four shelf markers and one other fixed point, not BB markers. So a BB marker QTM does not label is a marker QTM did not reconstruct, and "labelling" reduces to reconstruction coverage, which the subset rows already bound. The owner's question (2026-10-16, "would label-free pickup on our side make labelling robust") is answered no: there is nothing to pick up.

### D. BB firmware (owner question: does BB zero the absolute encoder at boot?)

Read at `~/Desktop/BallButler/ball_butler_main` (5e6eee8, the flashed FW 9): the AS5047P is read every control-loop sample and the position is the converted absolute reading (`YawAxis.cpp` ~line 781, `pos_deg_ = new_pos_deg`; glitch rejection zeroes the velocity only). User degrees = encoder degrees minus `ZERO_OFFSET_DEG_`, a volatile RAM float set to `BOOT_ZERO_OFFSET_DEG = 5.0` at boot (`ball_butler_main.ino:119`, `hardware_config.h:401`) and again to `YAW_MIN_ANGLE_DEG = 5.0` after the HAND homing completes (`StateMachine.cpp:253-254`); the yaw itself is never homed, nothing reads or writes EEPROM, and no CAN message sets the offset. The only runtime writers are the USB serial console commands `zero` (set zero here) and `offset <deg>`, lost at the next power cycle. Answer: no, the reported yaw at a given shaft angle is boot-invariant to the encoder's own repeatability (Phase 0 § C stands). The step therefore lies between the encoder magnet and the marker carrier.

### E. The template residual: one statistic, its reference level

Since Phase 1 one statistic feeds both the hard cap and the advisory: `ConstellationTrack.template_residual_mm` (the pose-binned max over marker pairs) is what `estimate_sweep_yaw_offset` compares with the 1.0 mm cap and what `mocap_node` hands to `assess_sweep_vs_reference` for the 0.90 mm advisory (lines 1128, 1164 and 1348 of the node); the window mean is a logged diagnostic only, and no path depends on pool size. The brief's framing of two statistics (a marker-pair one for the cap, a window mean for the advisory) does not match the code. Per sitting the statistic reads 0.18 to 0.76 mm across the probe's 21 sittings (the 16:23 sweep, 0.25 mm, is inside that range) (0.35 / 0.44-0.46 / 0.18 for 10:51 / 15:08 / 16:10 under one wand calibration); with Q3 and Q5 excluded (common5) it reads 0.16 to 0.44, and the worst pair is 3-x or 5-7 in 17 of those 21 sittings. The residual tracks per-sitting reconstruction quality at the shelf and not the kappa level (0.35 at 179.57, 0.45 at 179.26, 0.18 at 179.34). Reference level: 0.18 to 0.46 mm; band and cap unchanged; no remedy named (the Phase 1 comment that a 562 h old wand calibration "tripled" it is withdrawn, see below).

### F. Rest pose and the shelf residual

At rest (yaw about 0) Q3 and Q5 are mostly not reconstructed and the labelled pairs are non-rigid by 1-2 mm with a sitting-dependent pattern (pair 1-2: -1.75 / -0.2..+0.6 / -0.81 mm at 10:51 / 15:08 / 16:10; 2-3 +2.11 at 16:10), against 0.2-0.5 mm RMS per marker while moving. The solver already excludes the parked frames (the moving mask, since 2026-10-09), so this is QTM-side camera coverage at the parked pose and needs no code. The shelf residual is 0.005 mm on the bag the as-built template was learned from (10-10 12:26, in-sample) and 0.05 to 0.48 mm on every other sitting (0.29 / 0.30-0.35 / 0.20 on 10-16), with all four markers visible in every frame and every pair shorter than the template by 0.02 to 0.35 mm (C-S the largest). It never "recovered" because 0.004 was in-sample; 0.2-0.35 mm is the sitting-to-sitting reconstruction variation at the shelf and a re-wand sets a new state rather than restoring the training one. For Phase 3 that bounds the measured robot-frame heading at about 0.07 deg (0.3 mm over 240 mm).

### G. Replay fidelity on the 10-16 launches

The harness now scores the 10-16 launches against the node's logged values. On the 15:08 bag (smoke run `temp/probes/bb_calibration_replay/20261016-171315`) sweeps #4-#7 match on every measurement and #1-#3 do not (yaw +0.03 / -0.05 / -0.003 deg, lag -0.5 / -0.65 ms). The node logged 2343 frames for #1 where the bag holds 2037 (the recorder dropped), and 1937 for #2 where the bag holds 2114 (the node dropped: the frame-queue starvation the parallel session diagnosed the same evening); #3 differs by 32 frames. Where the frame sets are equal the solver agrees to the fidelity tolerances, so these are input-set differences and are reported, not tuned away. The 10:51 sweeps match to 0.002 deg; the 16:23 sweep differs by 0.009 deg in yaw and kappa (replayed -11.621 vs logged -11.630), the same pattern. Committed run: 227 of 238 measurements match, 46 of 50 sweeps on every measurement, 3 logged sweeps (10-15 launches) have no bag.

## Discussion

(i) Why a measurement first, and why the pair before the population. The plan's decision criterion is binary (kappa collapses under a common subset or label-free pickup = artefact; keeps two levels = BB) and the pair is the only place where QTM is a control. Solving the population first would have mixed in the re-wand and the frame re-alignments. The probe decoded each bag once into a cache and iterated the variants on the cache, which is why eleven refits per sweep over the probe's 74 sweeps cost under ten minutes.

(ii) What was ruled out, in order. Labelling (six of seven markers labelled identically; the common subset keeps the step); the template (every single-marker drop keeps it; body-frame means flip sign between sittings, so there is no stable template error to re-fit); reconstruction (one wand calibration, shelf heading identical to 0.002 deg, axis point within 0.5 mm); timing (the lag is fitted per sweep, both legs carry the step, the direction split is 0.04-0.14 deg and per-launch); firmware (absolute per sample, constant zero, nothing stored). What remains is physical and between the encoder magnet and the marker carrier: a slip of the magnet or of the yaw stage on its shaft, or a drivetrain state set at power-up.

(iii) The unanchored read. An Opus agent read the pair table without the earlier entries and returned the same verdict with two useful additions: the direction split as the last software escape (closed in § A) and the observation that the body-frame residual pattern moved with kappa across the three 10-16 sittings. The 21-sitting table does not support the second as a rule (Q3 and Q6 means vary by sitting with no ordering by epoch), so it is recorded as a three-sitting reading, not a finding.

(iv) Why nothing lands in the node for pickup (owner decision). A mode the data never exercises is an untested mode (Phase 1 discussion v), and the node's collection window would gain a radius parameter and a second frame builder for a benefit of at most 0.014 deg of kappa on 82 sweeps. The replay keeps the label-free solve as a column so a future labelling failure would show up as kappa_labelfree diverging from kappa; that is cheaper and is measured on every bag.

(v) The statistic stays (owner decision). The window mean is pose-dependent (the 2026-10-10 entry) and would flag reconstruction geometry; the pose-binned statistic flags a moved marker and reads reconstruction quality as its level. Excluding Q3 from it would make the gate blind to a Q3 move. The reference level moves to 0.18-0.46 mm in the docs and comments; no remedy is named because nothing at the console changes per-sitting reconstruction.

(vi) Control-system implications. None: no published quantity changes. The yaw offset, position, residual statistic, band and cap are untouched; the only code changes are harness columns, tests and comments. A calibration finalisation and an aim walk through exactly as before this phase.

(vii) Tradeoff accepted. Protection against a bumped BB still rests on the kappa epoch advisory; this phase adds no new protection and removes none.

## Fix

| file | change |
|---|---|
| `tools/probes/bb_calibration_replay.py` | Phase 2 columns per sweep: label fractions (window and rest), per-marker visibility / RMS / body-frame mean, worst pair, label-free refit (yaw, kappa, residual, matched range, unlabelled points per frame, count within 3 mm, p05 min distance), yaw and kappa by sweep direction, rest pair error, shelf pair errors; opt-in `--subset 1,2,4,6,7` and `--loo`; launch-log glob extended to the 10-16 launches; summary.md gains a Phase 2 section and the pickup null line |
| `tests/ros/test_bb_calibration_replay.py` | the pool-size pin (same data, same TEMPLATE_RESIDUAL verdict on 0, 1, 5 and 50 records, with the shipped band and a lowered one); the Phase 2 statistics on the fixture; the pickup mechanism test (a stripped marker is recovered from the unlabelled pool, shelf points are ignored); CLI parse of the two flags |
| `ros_ws/src/jugglebot/jugglebot/bb_calibration.py` | comments only: the advisory's reference level 0.18-0.46 mm and what sets it; the "562 h old wand calibration tripled this residual" clause replaced |
| `tools/probes/README.md` | replay row: Phase 2 columns and the null-check role of the label-free column |
| plan, Phase 1 entry, INDEX | status, Outcome note, the two-state question re-pointed here |

## Verification

- 2026-10-16, baseline before any change (worktree `Jugglebot-bbcal` fast-forwarded to 205c527c), `./run_tests.sh` (venv, log `temp/logs/run_tests_baseline_phase2.log`): parallel 6666 passed, 11 skipped in 309.25 s; serial 4 passed in 16.80 s; exit 0 (the brief's 6653 was the count at a51c3c20; the two later commits add 13 tests).
- 2026-10-16, probe decode + analysis of the 10-16 bags and of every 2026-10-10..14 bag (`/tmp/probe_bb_phase2.py decode/analyse --subset common5=1,2,4,6,7 --subset plane5=3,4,5,6,7`, LOO on): 74 sweeps analysed, 15 skipped; the tables in § Diagnosis A-C.
- 2026-10-16, `pytest tests/ros/test_bb_calibration_replay.py -q` (harness unit, Sonnet agent): 12 passed in 29.7 s.
- 2026-10-16, harness smoke run `python tools/probes/bb_calibration_replay.py --bags ~/Desktop/rosbags/2026-10-16_15-07-58 --subset 1,2,4,6,7 --loo -v`: 243 s, run `temp/probes/bb_calibration_replay/20261016-171315`; 55,987 near-axis unlabelled points over 7 sweeps, 0 within 3 mm; visibility identical with and without pickup in 7 of 7; 4 of 7 sweeps match the node on every measurement (§ G).
- 2026-10-16, `pytest tests/sim/test_logbook*.py tests/sim/test_plans_index.py tests/ros/test_bb_calibration_replay.py tests/ros/test_bb_calibration_constellation.py -q` (after the comment, glob, plan, entry, README and INDEX edits): 170 passed, 2 skipped in 56.60 s.
- 2026-10-16, `./run_tests.sh --full` (venv, log `temp/logs/run_tests_full_phase2.log`, run while the harness replay was decoding bags on the same box): parallel 6672 passed, 1 failed, 11 skipped in 321.62 s; serial 7 passed in 28.40 s. The failure is `tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version_neither_reports_computing_nor_blocks_the_requeue` (GUI rosbag replay, untouched by this phase, the same test Phase 1 classified as a load flake with a latent `_drain` race); scoped rerun 2026-10-16 `pytest tests/ros/test_replay_overview.py -q`: 11 passed in 22.06 s. Count check: the baseline's 6666 plus this phase's 7 new test cases (one parametrised over four pool sizes, three others) is 6673 = 6672 passed + 1 flake.
- 2026-10-16, `python tools/probes/bb_calibration_replay.py --subset 1,2,4,6,7 -v` (default bag glob `2026-10-1[0-6]*`, 27 bag directories, ~41 min, log `temp/logs/bb_replay_phase2_full.log`), run `temp/probes/bb_calibration_replay/20261016-181127`: 90 sweeps replayed, Phase 1 ACCEPTED 82, refused 8 (CONSTELLATION_MOCAP_CLOCK 7, CIRCLE_CENTRES 1); 23 of 24 logged refusals ACCEPTED and the circle-centres sweep still refused (the Phase 1 gate holds); 227 of 238 measurements match (§ G); flags QTM_FRAME_SHIFT 10, BASE_FRAME_RESIDUAL 2, KAPPA_EXCURSION 2, YAW_SHIFT 1; K1 opened at 10-13 22:00 #1. The 15:08 #1 sweep REJOINS K0 in the replay, whose state is seeded from the resource records (K0 median 179.32 at today's pin), where the live node opened K1 because its K0 was the migrated 8-record pool at 179.59: the rejoin path of the nearest-epoch rule exercised on real data.
- 2026-10-16, audit (one pass at phase end, audit-reporter on the unstaged diff plus this entry): no production-behaviour risk; two narrative warnings (the probe's 74 sweeps / 21 sittings against the committed run's 75 / 22 in the title, plan C1 and three sentences; the K2 range quoted against K3) and four narrative notes (two test docstrings, the README bag range, an unasserted figure in a test docstring) applied before the commit; two notes tagged behaviour left unapplied and recorded in § Open Questions.
- 2026-10-16, after the audit fixes (test docstrings only in `.py`): `pytest tests/ros/test_bb_calibration_replay.py tests/sim/test_logbook*.py tests/sim/test_plans_index.py -q`: 122 passed, 2 skipped in 34.32 s; pre-commit default gate `./run_tests.sh` (log `temp/logs/run_tests_precommit_phase2.log`): parallel 6673 passed, 11 skipped in 366.78 s; serial 4 passed in 16.77 s; exit 0.

## Outcome

True now: the kappa step is measured to be at BB and not in labelling, the template, reconstruction, timing or firmware; the owner's hardware step is defined (§ Open Questions); label-free pickup is a recorded null with a standing check in the replay; the template residual is one statistic with a documented reference level and a pool-size pin.

NOT true: the mechanism at BB is not identified (the owner's BB-only A/B was run the same evening: not a power-up state, see `2026-10-16-bb-calibration-power-cycle-ab.md`); plan acceptance 2's wording is still the owner's pending decision; the Phase 1 question whether BB was powered off between 10:58 and 15:05 on 10-16 is unanswered.

## Withdrawn claims

- Phase 1 entry and the `TEMPLATE_RESIDUAL_ADVISORY_MM` comment, 2026-10-16 morning: "a 562 h old wand calibration tripled this residual". Withdrawn: the residual is per-sitting reconstruction quality (0.18 to 0.46 mm under one fresh wand calibration, 0.18 to 0.76 across 21 sittings) and the re-wand set a new state rather than restoring the 0.26 mm of 10-10.

## Open Questions

- The BB mechanism. Owner decision 2026-10-16: (1) the firmware question is answered above (no); (2) a BB-only power-cycle A/B within one launch, QTM untouched: N sweeps, cycle only BB's supply, N more, repeated two or three times. Kappa jumping only across cycles pins a power-up state of the yaw stage; kappa holding across cycles and stepping only between sittings points at something that happens between sessions. The owner's check of the encoder magnet seat, the yaw stage on its shaft and the marker carrier follows from the result. **Done 2026-10-16 evening (`2026-10-16-bb-calibration-power-cycle-ab.md`): kappa held 179.27 deg across two BB-only cycles (72 s, 86 min); not a power-up state. The 86 min off deformed the constellation instead (Q3-Q6 1.2 mm, calibration refuses).**
- Was BB powered off between 10:58 and 15:05 on 2026-10-16 (Phase 1 question, unchanged)?
- Kappa moved 179.34 -> 179.24 between the 16:10 and 16:23 sittings, 13 minutes and one power cycle apart, and both late sittings' lag read -11.4 / -11.9 ms against +0.6 to +5.4 ms earlier in the day (a per-launch quantity). Within-day motion at the 0.1 deg level is inside the band; the A/B needs enough sweeps per leg to separate 0.1 from 0.3.
- Acceptance 2 wording (plan § 0): owner decision pending, unchanged. The 16:10 and 16:23 sweeps are accepted with no override and no action, kappa +0.07 and -0.04 deg inside K1's band: tally 3 of 10 under either wording.
- The mocap node's QTM frame-queue drops: diagnosed by the parallel skill-stack session as the calibration solve running synchronously in the heartbeat callback (`logbook/2026-10-16-mocap-frame-queue-starvation.md`, owner chose the worker thread the same evening, implemented on that session's branch `mocap-solve-worker` with a `solve_inline=True` test seam); it touches `_finalize_calibration`, is not blocked by this branch (which changes no node code), and does not change the solver the replay mirrors.
- Two audit notes left unapplied (behaviour-tagged, owner's call): the harness's `solve_variant` catches `ValueError` only, so a `LinAlgError` or `IndexError` from a degenerate `--loo` or `--subset` refit would abort a multi-bag run at that sweep (the committed run and the one-bag `--loo` run were clean); and the `labelfree_matched` column holds strings like `3-7`, which a spreadsheet may read as a date.
- The shelf-marker residual of 0.2-0.35 mm bounds the Phase 3 robot-frame heading at about 0.07 deg; Phase 3 should carry it as a stated uncertainty rather than a gate.
