---
title: "BB calibration Phase 0 (Characterise): a production-faithful replay reproduces 38 of 39 logged verdicts and 185 of 185 measurements, the pose-mix explanation of the kappa step is withdrawn, advisory bands are published, and the landing bias does not track the calibration"
type: investigation
date: 2026-10-15
status: resolved
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration)"
related_plan: bb-calibration-one-attempt.md
files_changed:
  - tools/probes/bb_calibration_replay.py
  - tools/probes/bb_calibration_bands.py
  - tools/probes/data/bb_calibration_replay_fixture.json
  - tools/probes/README.md
  - tests/ros/test_bb_calibration_replay.py
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - tests/ros/test_mocap_node_attempt_fields.py
  - tests/ros/test_mocap_node_keep_last_good.py
  - tests/ros/test_mocap_node_base_frame.py
  - tests/ros/test_perception_console_lines.py
  - tests/hardware/session_bb_kappa_pin.md
  - plans/archived/bb-calibration-one-attempt.md
  - logbook/2026-10-14-bb-calibration-one-attempt-investigation.md
  - logbook/2026-10-15-bb-calibration-phase0-characterise.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
tags:
  - safety
  - kinematics
  - testing
  - docs
---

# BB calibration Phase 0: characterise

## Summary

Phase 0 of `plans/archived/bb-calibration-one-attempt.md`. Three deliverables: (1) a replay harness that re-runs the node's calibration code on the recorded bags and reproduces what the node logged; (2) cross-sitting distributions and advisory bands for Phase 1; (3) the attempt-message fields the hardware pin needs. The pin's offline part is done and it **excludes the pose-mix explanation of the kappa step** (hypothesis withdrawn, § Discussion). The mechanism of the 0.3 deg step was left UNRESOLVED at the 2026-10-15 commit; on 2026-10-16 the owner's answers and a re-read of the replay moved it to reconstruction drift on a 562 h old QTM wand calibration as the leading candidate, the power-cycle pin was dropped (its runsheet deleted) and the first Phase 1 sweep after the re-wand is the remaining reading (§ Open Questions) - **superseded 2026-10-16 evening: reconstruction drift is ruled out and kappa is two-state (the 10:51 sweeps after the re-wand still read 179.57; 15:08 read 179.26 with QTM unchanged); see the dated corrections in Discussion and Open Questions.** The entry stays `open` for the two-state question and the node stall. The throw cross-check is negative: the landing bias has another owner. No gate logic or threshold changed in this entry.

## Symptoms

Carried from the 2026-10-14 investigation: 25 of 42 sweeps refused, 23 false positives; kappa stepped +0.25 deg at the 10-13 power cycle against a 0.09 deg gate. That entry attributed the step to pose-mix dependence of the fit (sweeps ended 3-5 deg shorter after the step). This entry tests that.

## Diagnosis

### A. Replay fidelity

`python tools/probes/bb_calibration_replay.py -v`, run 2026-10-15 10:08 (~28 min), output `temp/probes/bb_calibration_replay/20261015-100834/{sweeps.csv,fidelity.md,summary.md}`. 79 sweeps from 20 bags (21 bag directories; 10-14 21:47 held no sweep); the 10-10 bags only build the gate state and are not scored. Of the 42 logged sweeps 10-11..10-14, 39 have a bag.

| check | result | tolerance |
|---|---|---|
| measurements (yaw offset, kappa, template residual, lag, sigma) | 185 of 185 match | 0.002 deg, 0.002 deg, 0.005 mm, 0.5 ms, 0.002 deg |
| verdicts | 38 of 39 match | exact |
| pool n (informational) | differs: replay 44 vs logged 24 before 10-11 11:27; 51 vs 34 at 10-13 22:29 | none (node ran older code on 10-10); agrees on every sweep after the 10-13 22:29 reset; no verdict changes |

The one verdict miss is 10-11 15:14 sweep 1, logged CONSTELLATION_TOO_FEW_MOVING ("8 moving heartbeat yaw samples"). The heartbeat left CALIBRATING after 6.9 s, but the node's FAILED line came 35 s after "started" and it fell back to the heartbeat yaw with fewer than 100 stamped samples; the bag holds 690. That is a ~28 s node-side stall 10 s after launch, state the bag cannot show (NEW finding, open question). The stall sweep is excluded from the band population.

Plan acceptance 1: 23 of the 24 replayable logged-refused sweeps are ACCEPTED under `verdict_intrinsic` (history ignored): 12 BB_IN_BASE_MOVED, 7 TEMPLATE_RESIDUAL, 3 CALIBRATION_INCONSISTENT, the stall refusal. The one still refused is 10-14 20:54 sweep 3 CIRCLE_CENTRES (4.05 mm worst marker), judged on its data as the plan allows. The 25th refused sweep (10-11 13:33 / 14:01 launches) has no bag.

### B. Sweep kinematics (from the bags)

| quantity | before the step (7 sweeps, 10-11..10-13 00:18) | after (32 sweeps, from 10-13 22:00) | 10-10 (40 sweeps, 33 intrinsic-accepted) |
|---|---|---|---|
| max yaw, mean (range) | 125.1 deg (123.4-126.8) | 120.7 deg (119.1-122.9) | ~122.5 deg |
| peak speed | ~102-103 deg/s | ~100-101 deg/s | |
| return undershoot | -0.3..-1.4 deg | 0.0..+0.4 deg | |

The start yaw is unchanged across the step (~0.3 deg). The 10-10 sweeps already ended as short as the post-step ones, so "short sweep" alone does not mark the post-step epoch.

### C. BB firmware check (BB repo, read-only)

| fact | source |
|---|---|
| CALIBRATE_LOC is payloadless; the sweep is a position setpoint to compile-time `CALIB_MAX_YAW_DEG = 120` under P control (KP 300, FF 10 PWM, 50 PWM/s slew) | `CALIB_MAX_YAW_DEG` in `hardware_config.h`, BB repo at 5e6eee8 |
| reversal when abs(yaw - 120) < 1.0, so recorded max = 119 + overshoot | `handleCalibrating_` sub-state 0 in `StateMachine.cpp`, BB repo at 5e6eee8 |
| FW 8 to 9 (5e6eee8, flashed 2026-10-13 20:09) touches only CANCEL_CMD; no yaw, sweep, CALIBRATING or reporting hunk; YawAxis last changed FW 5 (18c5d59) | git, BB repo |
| AS5047P absolute encoder, constant 5 deg boot offset, no stored offset, no homing | firmware |

So the sweep end is NOT commandable by the operator, and a power cycle cannot change the reported yaw in firmware. Shorter sweeps after the step are reduced overshoot, i.e. a drive-dynamics change (supply state, friction, or a mechanical change). Consequence for the plan: the hardware pin cannot "command a longer sweep" and was redesigned to power cycles with recorded sweep start/end/duration/lag.

### D. Clip experiment (the decisive test)

Run 2026-10-15 10:28, `--no-bb-moved --clip-yaw-deg 1.0:118.5` over the 10-10 control and long-sweep bags plus 10-11..10-14; output `temp/probes/bb_calibration_replay/20261015-102849/clip_experiment.md`. Each sweep is re-solved using only frames with yaw inside the range every sweep shares.

| group | n | kappa full (SD) | kappa clipped (SD) | delta |
|---|---|---|---|---|
| pre-step | 20 | 179.3031 (0.0376) | 179.2876 (0.0214) | -0.0155 |
| post-step (the control) | 23 | 179.6021 (0.0408) | 179.5955 (0.0423) | -0.0067 |
| step (post - pre) | | +0.299 | +0.308 | |

Nine post-step sweeps refuse CIRCLE_CENTRES once clipped (an arc too short for the circle-centre check; all of 10-14 20:54, 10-13 22:00 #1-2, 10-13 23:12 #1) and carry no kappa, hence 23 of 32. The bands unit's independent check agrees: at matched sweep ends 120-123 deg, K0 reads 179.336 (n 19) and K2 179.598 (n 24), still 0.26 deg apart; within an epoch kappa vs sweep end has slope -0.0071 deg/deg (r -0.17).

## Discussion

**Withdrawn hypothesis.** The 2026-10-14 investigation (§ The kappa step mechanism) concluded "solver-side, pose-dependent, not BB and not the encoder", with the 5 deg shorter sweep end as the suggested trigger. D withdraws it: restricting every sweep to the common pose range leaves the step at +0.308 deg, where the hypothesis predicted it would collapse to under 0.10 deg; the matched-end comparison and the within-epoch slope say the same. The reasoning that excluded "BB moved" rested on (i) the rest heading, whose between-bag scatter is +-0.25 deg so it resolves a step only down to ~0.15 deg, and (ii) the encoder reading. Neither excludes a 0.3 deg physical change, and the argument was by elimination, not experiment (the investigation flagged this itself). Reset: the mechanism is UNRESOLVED. Candidates, each with what would show it:

| # | candidate | what would show it |
|---|---|---|
| 1 | BB rotated ~0.3 deg on its shelf, or its encoder/magnet was re-seated at the power cycle (the one physical contact in the window; firmware shows no software path, C) | hardware pin part 2 across power cycles; a BB-side physical check |
| 2 | the shelf base markers shifted: base residual vs the as-built template rose 0.10 to 0.18-0.34 mm at the step; 0.3 deg at the 240 mm C-E baseline is 1.3 mm | compare the base inter-marker distances before/after in the bags |
| 3 | a QTM reconstruction change at the shelf (template residual also rose 0.41 to 0.43-0.76; a camera moved) | QTM's own residuals around 10-13 |

**What the Phase 1 design is robust to.** All three. The epoch band (below) treats any of them as a new kappa epoch with an advisory, never a refusal, and the pool is kept. The design does not need the mechanism; it needs the measurement to be fresh and the history to be advisory.

**Why the sweep end is not an operator lever (C), and what that did to P7.** The sweep end is `120 + overshoot` from a compile-time constant, so P7 as first written ("sweep 5 deg longer and see kappa follow") cannot be run. The plan's Phase 0 text was rewritten: three power cycles, a park reading and two sweeps each, with sweep start/end/duration/lag recorded per attempt (§ G, runsheet).

**Why the bands are stated the way they are.** One rule for every band: band = max(3 x the relevant SD, 1.25 x the largest observed intrinsic-good excursion), rounded up. The relevant SD is the spread of sitting MEANS within the epoch the advisory compares against, never one sitting's per-sweep sigma (the old 0.09 deg floor was 2 x within-sitting sigma). The 1.25 x max term makes ~all recorded intrinsic-good sweeps fall inside, which is the owner's intent: none of the 23 false positives fires. The kappa band existed in two readings until D came back: if clipping had moved kappa by ~+0.25 deg the step would be a measurement artefact and the band would need to cover it (+-0.45 deg vs the whole pool); D chose the other branch, so the recommended reading is +-0.20 deg vs the current kappa epoch's median, and the +-0.45 deg whole-pool band is kept as the rejected alternative. A real 2 mm bump at r ~143 mm is 0.80 deg, 4.0 x the recommended band.

**Why the throw cross-check is negative and what that frees the plan from (F).** Calibration error at the level the bands describe predicts 0.4-2.8 mm lateral landing shift; the observed launch-to-launch differences are 8-20 mm in range and 3.4-8.1 mm lateral, and a yaw error cannot move range at all. The plan therefore claims no throw-side meaning for the bands (mm at the catch per degree of kappa stays unquantified) and does not try to fix the landing bias by calibration.

**2026-10-16 update.** Candidate 1 and the physical candidates fall away on the owner's answers (BB and its frame are bolted to the shelf and nothing was touched; BB sits on the main motor PSU, so every session boundary is a power cycle and kappa survived all but one) and on the flat shelf tilt in world (0.90-1.04 deg over the window). The reconstruction candidate (3) is now the leading one: QTM's wand calibration was 562 h old across the window and was re-wanded 2026-10-16. Not proven; the first post-re-wand sweep tests it (see Open Questions, "Mechanism, 2026-10-16 reading").

**2026-10-16 evening correction.** The 10:51 / 15:08 pair (one wand calibration, one world frame, kappa 179.57 vs 179.26) rules reconstruction drift out and the lean above was wrong; kappa is two-state, cause BB state or a labelling artefact (Phase 1 entry § Hardware reading).

**Fixture tolerances.** The committed fixture stores frames rounded to 4 decimals; the re-solve test asserts 1e-5 deg class tolerances, 3-4 orders below any gate, so rounding cannot flip a verdict.

## Fix

Instrumentation only, no gate or threshold change.

| piece | what |
|---|---|
| `tools/probes/bb_calibration_replay.py` | production-faithful replay (pool/base-frame/gate state simulated), `--clip-yaw-deg`, `--emit-fixture` with a recipe key |
| `tools/probes/bb_calibration_bands.py` | cross-sitting distributions, world and kappa epochs, bands, throw cross-check, P7 prediction |
| `bb_calibration.py` | `sweep_kinematics()`, `sweep_suffix()`; `SweepYawEstimate` gains `sweep_start_deg` / `sweep_end_deg` / `sweep_duration_s`; `summary()` appends "sweep 0.0→121.3° in 12.4 s" |
| `mocap_node.py` | every `/bb/calibration_attempt` text, the success text on `/bb/calibration_result` and the ERROR console line end with " · sweep <start>→<end>° <dur> s, lag <±x.x> ms" (lag n/a before the estimator); `MAX_CALIBRATION_FAILURE_CHARS` 240 to 300 with the suffix reserved first so truncation cannot drop it |
| `tests/hardware/session_bb_kappa_pin.md` | pin part 2 runsheet (three power cycles, park reading + two sweeps each) |
| `plans/archived/bb-calibration-one-attempt.md` | `bb_moved` deleted outright, pin redesigned, status row |

### Advisory bands (input to Phase 1)

`python tools/probes/bb_calibration_bands.py temp/probes/bb_calibration_replay/20261015-100834 --feeds /home/jetson/Desktop/Jugglebot-skills/temp/probes/u4_feeds.csv`, 2026-10-15, output `bands.md` / `bands.json` there. Population: 70 intrinsic-good sweeps over 19 bags (18 with a kappa).

| advisory | derived from | band | coverage | 0.80 deg bump / band |
|---|---|---|---|---|
| dkappa vs current kappa-epoch median (RECOMMENDED) | 1.25 x max 0.136 | +-0.20 deg | 62/62 | 4.0 |
| dkappa vs whole pool, step covered (rejected by D) | 3 x 0.147 | +-0.45 deg | 64/64 | 1.8 |
| dyaw vs last accepted, same world epoch | 1.25 x max 0.136 | +-0.20 deg | 60/60 | 4.0 |
| d p_b (BB in base, 3-D) vs pool | 3 x 0.258 | 1.0 mm | 64/64 | 2.0 (for 2 mm) |
| d axis point (world, 3-D), same epoch | 3 x 0.461 | 1.75 mm, information only | 60/60 | 1.14 (for 2 mm) |
| template residual (absolute) | rule 1.10, clamped to cap - 0.1 | 0.90 mm advisory; 1.0 mm hard cap stays | 70/70 | n/a |
| abs(lag) | 3 x cross-sitting SD 3.87 | 13 ms | 70/70 | n/a |

Notes on the table. The axis point is information only because the base origin wobbles 0.32 mm SD in world inside an epoch; use p_b for the position advisory. The template residual tracks template age (0.26-0.30 mm on 10-10, 0.43-0.76 after), so Phase 2's refit is what tightens it. Lag is a per-launch offset (sitting means -9.7..+4.6 ms, within-sitting SD 1.2 ms), so its band only catches a broken yaw source; LAG_AT_EDGE stays the hard check.

Epochs (cross-sitting kappa SD of sitting means 0.147 deg over 18 sittings with a kappa, of 19 bags in the population (10-10 10:32 never saw the base); within-sitting pooled SD 0.044 deg):

| epoch | n | mean | sweep SD | sitting-mean SD |
|---|---|---|---|---|
| K0 (10-10) | 28 | 179.327 | 0.038 | 0.048 |
| K1 (10-11..10-13 00:18) | 6 | 179.280 | 0.034 | 0.037 |
| K2 (10-13 22:00 on) | 31 | 179.591 | 0.058 | 0.031 |

World epochs: 9 (W0-W8), base heading -179.06, -178.69, -177.52, -177.31, -177.05, -177.31, -177.25, -176.63, -176.53 deg. **Epoch mechanism (Phase 1 input):** an excursion beyond +-0.20 deg opens a new kappa epoch, the non-destructive replacement for `bb_moved` (pool kept, nothing discarded, advisory only).

Caveats on the population: the 33 sweeps of 10-10 have no console log, so axis point, p_b, base origin, base residual and sweep end were never logged and are unvalidated against the node (K0 rests on them alone; same code, same bag). Bag 10-10_10-32-31 never saw the base; its 5 sweeps feed only the world and intrinsic statistics.

**Outlier sitting 10-14 20:54** (an open question for the owner): kappa drifted 179.664 to 179.445 (-0.22 deg) over ~110 s at a constant ~119 deg sweep end, world yaw following (3.04 to 2.81 deg), base heading steady +-0.004 deg, p_b moved 0.45 mm, within-sitting SD 0.099 deg against 0.013-0.048 elsewhere; it recovered to 179.64 by the 21:27 launch. It sets the 0.136 deg maxima behind two bands. Flagged, not excluded. It is the only candidate in the record for something changing physically during a sitting.

### F. Throw cross-check (plan Phase 0, last bullet)

Launches matched to bags by feed time; calibration in force = last logged ACCEPTED sweep in the bag; throw range 935-946 mm, ~27 deg off world +x.

| run | bag | in force | yaw (deg) | kappa (deg) | src / pool n | median miss dx/dy (mm) | predicted lateral from yaw error |
|---|---|---|---|---|---|---|---|
| s8 | 10-13_23-12-50 | #4 | 2.339 | 179.584 | sweep / 2 | -7.5 / -5.8 | +0.023 deg, 0.4 mm |
| s1 | 10-14_20-54-20 | #7 | 2.815 | 179.445 | sweep / 4 | -16.1 / -6.3 | -0.139 deg, -2.3 mm |
| s2 | 10-14_21-27-22 | #6 | 3.084 | 179.617 | sweep / 5 | +5.6 / -4.5 | -0.027 deg, -0.4 mm |

In-force dkappa 0.03-0.17 deg predicts 0.5-2.8 mm lateral; axis-point errors in force (0.13-0.38 mm) predict at most 0.1 mm of range; per-axis landing SD is 14-19 mm over 12-19 feeds per launch (median SE ~4-5 mm). **Verdict: the landing bias does not track the calibration; it has another owner, and this plan does not claim it.**

### G. P7 prediction

Kappa noise: within-sitting SD 0.044 deg, within-epoch sitting-mean SD 0.040 deg.

| design | detectable difference |
|---|---|
| 3 vs 3 sweeps, one sitting | >= 0.11 deg (3 SE 0.036) |
| 3 vs 3, different sittings | >= 0.16 deg |
| power cycles, ONE sweep per cycle | >= 0.19 deg (3 x sqrt(2) x 0.044) |
| power cycles, TWO sweeps per cycle (pair mean vs pair mean) | >= 0.13 deg |

The step (0.26-0.31 deg) clears one-sweep detection at ~4 sigma, but 10-14 20:54 drifted 0.22 deg inside one sitting, so the runsheet uses two sweeps per cycle. Criterion for "kappa changed across a power cycle": more than **0.20 deg** (the within-epoch band).

## Verification

| what | date | command | result |
|---|---|---|---|
| attempt fields + calibration + node tests | 2026-10-15 | `pytest tests/ros/test_bb_calibration_*.py tests/ros/test_mocap_node*.py -q` | 203 passed, 1 skipped in 35.9 s |
| replay harness test | 2026-10-15 | `pytest tests/ros/test_bb_calibration_replay.py -q` | 4 passed in 2.2 s (fixture re-solve ~1.9 s, bands summarise, fixture recipe, CLI smoke) |
| final-code replay | 2026-10-15 | `source ~/Desktop/PDJ_venv/venv/bin/activate && python tools/probes/bb_calibration_replay.py` (all 21 bag directories, ~28 min, output `temp/probes/bb_calibration_replay/20261015-104816/`) | 79 sweeps from 20 bags; 185 of 185 measurements match, 38 of 39 verdicts match; 23 of 24 logged-refused sweeps accepted under the intrinsic verdict |
| full gate | 2026-10-15 | `./run_tests.sh --full` (second run, after the console-line test fix; the first run failed only `test_perception_console_lines.py::test_calibration_failure_is_one_error_line` on the exact ERROR text, 1 failed / 6641 passed) | PASS: parallel 6642 passed, 11 skipped in 291.48 s; serial 7 passed in 27.39 s; total 324 s |

Replay fidelity (§ A) and the clip experiment (§ D) are the (2026-10-15, command, result) triples given in their sections.

## Outcome

Open. Done: harness, bands, attempt-message fields, pin part 1 (offline: pose mix EXCLUDED). Owed: pin part 2 on hardware. Phase 1 (gate inversion, kappa epochs, `bb_moved` deletion) consumes the band table above. Plan status row: "code done, hardware pin owed".

2026-10-16: pin re-scoped (power-cycle sitting dropped, runsheet deleted); QTM re-wanded 2026-10-16; Phase 1 started 2026-10-16.

## Open Questions

- Hardware pin part 2 (`tests/hardware/session_bb_kappa_pin.md`, deleted 2026-10-16): DROPPED (owner, 2026-10-16, on recommendation). Every session boundary is already a BB power cycle (BB is on the main motor PSU) and kappa survived all but one of them; Phase 1's hardware acceptance (first sweep of ten consecutive sittings) is ten more. Replaced by (i) the owner's QTM check, done, and (ii) the first Phase 1 sweep read as in the mechanism bullet below. The runsheet was removed with `git rm`.
- Owner questions: ANSWERED 2026-10-15 evening. BB is powered from the MAIN motor PSU, so whenever Jugglebot's motors are unpowered BB is unpowered (BB also has its own E-stop); on 10-13 it was simply powered off normally, with no contact. Neither BB nor the shelf was moved. Neither BB nor its frame can move (both fixed to the shelf); the shelf itself can rock a little about an axis roughly parallel to world x but tends to settle in the same position. Nothing was touched near BB 10-14 20:54-21:00. QTM's calibration report: the owner did not think it showed a change (from memory, not checked at the time). QTM, 2026-10-16: the wand calibration was 562 h old (about 2026-09-22), so the whole 10-10..14 window ran on an 18-23 day old calibration; the owner re-wanded QTM on 2026-10-16. QTM's own residual for the BB body was 0.30 mm before and 0.23 mm after.
- Mechanism, 2026-10-16 reading (replay run 20261015-104816, `sweeps.csv`, intrinsic-accepted sweeps with the base seen, sitting medians):

  | at the step, 10-13 00:18 -> 22:00 | before | after |
  |---|---|---|
  | kappa (deg) | 179.271 | 179.554 |
  | shelf heading in world (deg) | -177.308 | -177.258 |
  | shelf tilt in world (deg) | 0.919 | 0.922 |
  | BB template residual (mm) | 0.413 | 0.807 |
  | shelf marker (base) residual (mm) | 0.098 | 0.333 |

  Over the whole window the shelf tilt in world stayed flat at 0.90-1.04 deg (no rock settled anywhere new); the BB template residual went 0.26 mm (10-10 noon) -> 0.4-0.8 mm (10-13/14), i.e. tripled; the fitted BB axis tilt drifted 0.93 -> 0.50 deg against a flat shelf, which a bolted BB cannot do physically; kappa held to under 0.05 deg across every OTHER session boundary in the window (each one a power cycle). A power cycle is not a sufficient cause; the physical candidates fall away on the owner's answers and the flat shelf tilt; the data LEAN toward 3D reconstruction drift at the shelf. Not proven. The test is the FIRST SWEEP after the 2026-10-16 re-wand: BB template residual back near 0.26 mm with kappa moving off the K2 median (179.59 deg) says reconstruction owned the step; kappa staying on K2 with the residual down says something else at the shelf. Phase 1 does not depend on the answer.

  **Correction, 2026-10-16 evening: the lean was wrong.** The 10:51 sweeps (old code, after the ~10:28 re-wand) still read kappa 179.57 with template 0.35 mm, and by 15:08 kappa was 179.25-179.28 with QTM unchanged (same wand calibration, shelf pose 0.50 mm / 0.001 deg). Reconstruction drift is RULED OUT; kappa is a two-state quantity (+0.31 deg on 10-13, -0.33 deg on 10-16, both between sittings). See `logbook/2026-10-16-bb-calibration-phase1-gate-inversion.md` § Hardware reading.
- The ~28 s node-side stall of 10-11 15:14 sweep 1: what blocked the node 10 s after launch (the bag cannot show it)? Unchanged.
- The 10-14 20:54 transient: physical or measurement? The owner touched nothing; with the shelf markers steady and that sitting's template residual at 0.48-0.50 mm, measurement is the leading reading.
- Who owns the per-launch landing bias (8-20 mm range)? Unchanged.
