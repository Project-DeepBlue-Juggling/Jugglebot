---
title: Why Ball Butler calibration is not a one-attempt process - 25 of 42 sweeps refused 2026-10-11..14, 23 of them (every BB_IN_BASE_MOVED, CALIBRATION_INCONSISTENT and TEMPLATE_RESIDUAL one) false positives; one causal chain (kappa step, floor-width gate, destructive bb_moved, world-frame gate, template residual) and seven proposals
type: investigation
date: 2026-10-14
status: open
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration)"
related_plan: bb-calibration-one-attempt.md
files_changed:
  - logbook/2026-10-14-bb-calibration-one-attempt-investigation.md
  - logbook/INDEX.md
  - plans/active/bb-calibration-one-attempt.md
  - plans/active/INDEX.md
subsystem:
  - tracking
  - ros
tags:
  - safety
  - kinematics
---

# BB calibration is not a one-attempt process

## Summary

The owner wants Ball Butler (BB) calibration to succeed on the first sweep, measuring BB fresh each time. Since the last calibration code change (b37faf8f, 2026-10-11 00:59) 25 of 42 sweep starts were refused. The owner states nothing physical was touched in the window. So 23 of the 25 refusals (every BB_IN_BASE_MOVED, CALIBRATION_INCONSISTENT and TEMPLATE_RESIDUAL one; the two remaining refusals are intrinsic sweep-quality calls judged on their own data) were false positives. The cause is one chain, not five bugs: the gates protect a pooled reference from a bumped BB, but the setup legitimately changes the reference's frame and the measurement's own day-to-day scatter exceeds the gate widths. No code changed in this entry. The fix is in the plan `bb-calibration-one-attempt.md`.

## Problem

Owner's ask: one attempt, every time. Owner facts (AskUserQuestion, 2026-10-14):

- Nothing on BB, the shelf, the base markers or the BB markers was touched 2026-10-11..14.
- BB's yaw encoder is absolute (no homing).
- BB was power-cycled and the QTM frame re-aligned to Jugglebot's base between the 10-13 00:18 and 10-13 22:00 sittings. No wand recalibration.
- The shelf is room-fixed. Jugglebot is not: it is nudged or moved, and the owner re-aligns the QTM global frame to its base before most session blocks.
- The owner wants BB measured fresh each time (history at most a warning), and the robot frame measured in software, not by manual QTM re-alignment.

Headline (census, SYNTHESIS § 1): 17 launches, 42 sweep starts, 17 accepted, 25 refused. Operator response to the refusals was to re-run within a minute (11 to 39 s apart across the launches), then arm `bb_moved:=true`.

## Evidence census

Source for all rows: `census/census.md` and `census/attempts.csv` (scratchpad), built from the `/bb/calibration_attempt` topic in the twelve bags `~/Desktop/rosbags/2026-10-11_11-27-53` onward plus `~/.ros/log/2026-10-1*/launch.log`.

| Launch (local) | Outcome |
|---|---|
| 10-11 11:27 .. 10-13 00:18 (seven launches) | 10 starts, 8 accepted. One BB_IN_BASE_MOVED (13:33, +0.120 deg), one CONSTELLATION_TOO_FEW_MOVING (15:14). Neither repeated. |
| 10-13 22:00 | 6 refused, BB_IN_BASE_MOVED |
| 10-13 22:29 | 6 refused, `bb_moved` armed 22:31:36, 2 accepted |
| 10-13 23:12 | 3 TEMPLATE_RESIDUAL (0.76, 0.75, 0.52 mm), 1 accepted at 0.49 mm |
| 10-14 20:54 | 2 CALIBRATION_INCONSISTENT (+0.702 deg / 7.13 mm, +0.694 deg / 7.12 mm vs accepted 10-13 12:14Z), 1 circle-centres (7.9 s sweep), `bb_moved` armed 20:55:16, 1 TEMPLATE_RESIDUAL (0.52), then accepted on the override, 2 more accepted |
| 10-14 21:27 | 1 CALIBRATION_INCONSISTENT (+0.298 deg / 5.11 mm vs a reference 30 min old), 3 TEMPLATE_RESIDUAL (0.53, 0.54, 0.54), `bb_moved` armed 21:27:53, 2 accepted |
| 10-14 21:56 | accepted |

Totals from `attempts.csv`: 42 starts, 17 accepted, 25 refused. Before 10-13 22:00: 10 starts, 8 accepted, 2 refused. From 10-13 22:00 on: 32 starts, 9 accepted, 23 refused (22:00 6/0/6, 22:29 8/2/6, 23:12 4/1/3, 10-14 20:54 7/3/4, 21:27 6/2/4, 21:56 1/1/0).

Per class: BB_IN_BASE_MOVED 13 (delta-kappa +0.12 .. +0.34 deg vs limit +-0.093 deg, pooled n 28/34), TEMPLATE_RESIDUAL 7 (0.52 .. 0.76 mm vs 0.50 mm), CALIBRATION_INCONSISTENT 3, CONSTELLATION_TOO_FEW_MOVING 1, circle centres 1 (up to 4.05 mm).

## Analysis

The causal chain (SYNTHESIS § 2, gate mechanics in `pipeline_and_gates.md`):

- **C1. kappa stepped +0.25 deg.** Kappa is BB's yaw offset in the shelf base frame. Between 10-13 00:18 and 22:00 it stepped. Refused sweeps read delta-kappa +0.244 deg (SD 0.021, 22:00 launch) and +0.286 deg (SD 0.041, 22:29 launch). Accepted pre-step delta-kappa was -0.034 deg (SD 0.031, n 8). A stable offset with noise one-eighth of its size. Mechanism in the next section.
- **C2. The gate had no basis for its width.** The limit is max(3*hypot(sigma_src, pool SE), 0.09 deg). sigma_src is 0.030 deg for the stamped source, measured in one sitting (see `2026-10-10-bb-base-frame-robustness.md`). Pool SE is near 0 at n 34, so the limit sat at its 0.09 deg floor. Cross-sitting kappa repeatability was never characterised.
- **C3. The only remedy is destructive.** The refusal line offers `bb_moved:=true`. That resets the pool to n=1 (34 to 1, 28 records lost, no backup). With fewer than 5 pooled records the node routes every later sweep through the world-frame gate path (CALIBRATION_INCONSISTENT) and enables the 0.50 mm TEMPLATE_RESIDUAL gate that base_frame mode skips.
- **C4. The world gate refuses by construction.** It compares yaw and axis point in QTM world coordinates with the last accepted sweep. Marker geometry (`census/marker_geometry.md`, 12 sittings): Jugglebot Base markers are static in world to 0.6 mm / 0.03 deg across 10 of 12 sittings. Shelf and BB moved in world by up to 9.8 mm and -0.26 .. +0.63 deg per sitting, +29 mm / +1.0 deg cumulative. The 10-14 20:54 step of +0.629 deg is the "delta-yaw +0.70 deg" refusal. So the first sweep after each re-alignment is refused, which forces `bb_moved`, which resets the pool. A positive-feedback loop.
- **C5. TEMPLATE_RESIDUAL sits inside the scatter.** Five refusals at 0.52 .. 0.54 mm, two at 0.75 .. 0.76. Accepted sweeps in the same sittings read 0.43 .. 0.50. Two sweeps were accepted at 0.65 and 0.70 mm only because they took the base_frame path, which skips the check. A sweep passes by which path it took, not by its data. At rest the single-pose per-pair error vs the template is 0.5 .. 3.6 mm and varies 0.5 .. 4 mm bag to bag, with temporal noise only 0.04 .. 0.17 mm. Only 5 of 7 BB markers label at rest (QTM3 in 4 bags, QTM5 never). The base residual itself drifted 0.10 to 0.44 mm against its 0.5 gate.
- **C6. The design premise is inverted.** Base_frame mode treats the pooled kappa as the output ("pose source base_frame: +3.0823 deg (kappa n 6) vs sweep +3.0636 deg") and the sweep as a check on the pool. In the window the gates caught zero real moves and refused 23 good sweeps.

## Discussion

**Withdrawn hypothesis: "the QTM frame was re-defined within the window."** It was the working explanation at the start, from the owner's note that the frame had been re-aligned once. The marker data does not support it. The Base markers are static in world across 10-11..10-14. The one true re-definition is 10-10 23:32 to 10-11 11:27 ((+49.8, -66.1) mm, +1.17 deg; the node logged "base moved 82.9 mm, 1.176 deg"), which is before the window. What moves is Jugglebot, with the frame following it, so shelf and BB move in world while the Base does not. The one exception, 10-14 21:27 to 21:47 (Base -3.0, -7.6, +1.0 mm, -0.37 deg, shelf still), is a bump without re-alignment. This replaced the hypothesis with C4: the world gate is wrong for this setup, regardless of when the frame was set.

**Why not "keep the bump detector as a hard refusal."** The tempting minimal fix is to widen the thresholds and keep the gates. Three failure classes argue against it:

1. The measurement's own cross-sitting scatter (about 0.25 deg in kappa, about 1 mm in template residual) is not characterised, so any new number is another one-sitting guess (C2, C5). C2 is the same error that produced the 0.09 deg floor.
2. A hard refusal needs a remedy. The only remedy destroys the reference and changes the path (C3), so a refusal is not recoverable by the operator.
3. The detector watched a physical event that did not occur in 25 refusals and caught nothing real. Its cost is certain, its benefit unobserved. A real bump would show in the sweep's own intrinsic quality (axis consensus, constellation residual) and in the downstream catch.

"Measure fresh" wins on root cause: the output of a calibration should be the thing measured, and history should inform the operator, not veto the measurement. This also matches the owner's stated intent.

**Premise inversion (C6).** The pool-as-output design made sense when the sweep was the noisy quantity and the pooled value the precise one. The census shows the opposite at the cross-sitting level: the pool is precise within a sitting and wrong across them. Inverting it (sweep is the output, pool is advisory) removes C3, C4 and C6 together.

**Frame-assumption answer (owner's question, SYNTHESIS § 4; `frame_convention_trace.md`).** Does the code require mocap = Jugglebot base = world? Yes. No mocap-to-robot transform exists anywhere; "world" is the QTM global frame verbatim. The Base alignment check (`MocapInterface` alignment check in `mocap_interface.py`, lines 538-557 at HEAD ef7753af) has 2000 mm / 180 deg thresholds and never fires, and publishing is ungated. The only compensation is a session-start, translation-only (up to 25 mm) Platform offset in `skill_node.py` (`SkillNode._frame_check` / `_frame_offset_check`, lines 3887-3995 at HEAD ef7753af). Sites that need mocap origin and heading to equal the robot base: BB aim xy (the `_mocap_aim_point_mm` call site in `skill_node.py`, line 2803 at HEAD ef7753af), stow plane z 578.2 (`mocap_interface.py:532-533`, static TF `mocap_node.py:946`), landing plane z 830 (`ball_tracker_node.py:58`); `columns_feed_bb_bias_mm` is not rotated. Gravity as -z is frame-rotation sensitive only to tilt. Frame-invariant: BB IK given a target in the same frame, the BB-in-base gate, platform levelling (IMU). Artefacts embedding a world frame: `bb_calibration_last_accepted.json`, `bb_base_frame_state.json` (base_ref and per-record world yaw), `resources/bb_base_frame.json` base_pose_at_build, the template gauge pin (landing-derived), `throw_affine_correction.json` (disabled), `columns_feed_bb_bias_mm`. Learner memory is release-relative, so it needs no migration. The owner's manual re-alignment is the only thing making the assumption true.

## The kappa step mechanism

Source: `census/kappa_step.md` (12 bags, quiet pre-attempt windows). Verdict: (c) solver-side, not BB and not the encoder.

- Kappa_rest (BB template yaw in the shelf frame minus the parked bb_yaw encoder) is flat across the step: about 179.9 deg over the 5 bags before, about 179.8 deg over the 7 after. The node's sweep kappa moved 179.28 to 179.57 deg over the same boundary.
- The parked encoder did not step (heartbeat yaw 0.25 deg at the start of both the 10-13 00:18 and 22:00 launches, +-0.4 deg either side). Not (a) encoder. The rest heading did not step. Not (b) BB moved.
- Caveat: between-bag scatter of the rest heading is +-0.25 deg (within-window SD 0.02 .. 0.06), so the rest test alone resolves a step only down to about 0.15 deg. (b) is excluded on encoder plus heading together and on the owner's account, not on heading alone.
- No BB BOOT, disconnect or gap in any bag. A power cycle in the uncovered 10-13 00:19 .. 21:09 gap cannot be excluded, but would not change the encoder/heading relation (absolute encoder).
- Kinematics: every sweep is 0 to about 121 deg and back at about 100 deg/s, 11 .. 14 s. From 10-13 22:00 every sweep ends 3 .. 5 deg shorter (ymax 119 .. 122 vs 123 .. 127 deg before) and that persists through 10-14. It is the only kinematic change coincident with the step. The fitted yaw-source lag (-9.7 .. +6.3 ms) does not track kappa (a 0.25 deg bias at 100 deg/s needs 2.5 ms one-sided).
- Consistent mechanism: QTM reconstruction at the shelf carries mm-level, pose-dependent systematics (rest pair separations 0.5 .. 3.6 mm off the template, varying 0.5 .. 4 mm bag to bag; a 6-marker set reads about 0.5 deg off the 5-marker heading). 0.65 mm at the roughly 150 mm marker radius is 0.25 deg. A sweep that samples a different pose mix fits a different kappa.

So kappa's cross-sitting error band is about 0.25 deg, about 3x the gate's 0.09 deg floor. The gate refused the measurement's own systematic. Downstream cost of 0.25 deg is about 4 mm lateral at a 1 m throw, inside what the learner and the catch already absorb (R5 gate met with 32 consecutive catches).

**Open point, marked explicitly:** (c) is established by elimination of (a) and (b) plus the consistency argument. It is not shown by a controlled experiment. The 5 deg shorter sweep end as the trigger is coincidence of timing only. P7 is the experiment that pins it.

## Proposals

Ranked in SYNTHESIS § 5; phased work, owners and tests live in `plans/active/bb-calibration-one-attempt.md`.

- **P1** Invert gate semantics (closes C3, C4, C6). Hard refusals only for intrinsic sweep quality without history (CONSTELLATION_MIN_FRAMES, TOO_FEW_MOVING, LAG_AT_EDGE, MOCAP_CLOCK, axis consensus, CONSTELLATION_RESIDUAL 1.0 mm). History comparisons become advisory lines. `bb_moved` retired as a gate input.
- **P2** Characterise before thresholding (closes C2, C5). Re-solve every sweep in the 10-10..10-14 bags offline (about 60 sweeps; the solver is pure Python), publish cross-sitting distributions of kappa, yaw, axis point, lag and template residual, and keep the replay as a `tools/probes` harness.
- **P3** Non-destructive reference (closes C3). Pool keeps all records in epochs with a backup on write. Under 5 records the base path outputs the sweep's kappa and skips the comparison. Startup console prints the loaded state.
- **P4** Measure the robot frame in software (closes C4, owner's choice). `mocap_node` fits the Jugglebot Base rigid body (labelled "Base - 1..7") at session start, publishes T_world-to-base, and downstream works in the base frame. World-frame gate retired, world-embedding artefacts re-expressed once.
- **P5** Template residual (closes C5). One path, one statistic, advisory at the P2 band, hard cap 1.0 mm. Investigate the 5-of-7 labelling and the day-to-day rest pair variation.
- **P6** Operator surface. One outcome line per attempt with every advisory at once, a rehearsal row for the calibration rung, no refusal naming an action that is not the remedy.
- **P7** Pin the kappa step on hardware. In one launch alternate old (about 126 deg) and new (about 121 deg) sweep ends, 3 each; 3 BB power cycles each followed by a park reading and a sweep; log sweep end angle and fitted lag in the attempt message.

Acceptance for "one attempt, every time": the offline replay accepts the 25 refused sweeps (the two intrinsic-quality refusals judged on their data); then the first sweep of each of the next ten sittings is accepted with no override and its kappa and yaw lie inside the P2 band.

## Verification

Nothing was fixed here, so nothing is verified on hardware. **No code changed and no test suite was run for this entry.** The only runs are offline, read-only analyses on 2026-10-14, scripts in the session scratchpad (not committed):

- Census builder (`census/build_census.py`, `summ.py`, `gen_md.py`): reads `/bb/calibration_attempt` and `/bb/calibration_result` from the twelve bags from `2026-10-11_11-27-53` plus the `~/.ros/log/2026-10-1*/launch.log` console lines (`calibration_console_lines.txt`). Output `census/census.md`, `attempts.csv`.
- Marker-geometry analysis (`census/marker_geometry.py`, `fixed_ref.py`, `extra_prior.py`): Base, shelf and BB marker positions per sitting from `/mocap_data`. Output `marker_geometry.md`, `markers_by_sitting.json`.
- Kappa-step analysis (`census/kappa_step.py`): rest kappa, parked encoder, sweep kinematics and lag per bag. Output `kappa_step.md`.
- Static reads: gate mechanics (`pipeline_and_gates.md`) and frame-convention trace (`frame_convention_trace.md`), from the code at the working-tree state of 2026-10-14.

Catalogue check: `pytest tests/sim/test_logbook*.py tests/sim/test_plans_index.py -q` is run only to confirm this entry parses. It says nothing about the findings.

## Open questions

- Is the kappa step caused by the shorter sweep end (P7), by a BB power cycle, or by QTM reconstruction drift? Not yet separable.
- What is the true cross-sitting distribution of kappa, yaw, axis point and template residual (P2)? Every threshold in the plan waits on it.
- Why do only 5 of 7 BB markers label at rest, and why do rest pair separations vary mm-level day to day? Camera coverage at the shelf is the first suspect, unconfirmed.
- Does the 10-13 00:19 .. 21:09 uncovered gap hide a second event beyond the owner-reported power cycle and re-alignment?
- Would a QTM rigid-body definition for BB make labelling robust (P5)?
- Per-launch feed landing bias (parallel session, 2026-10-14 columns analysis, `logbook/2026-10-14-two-ball-columns-sessions-why-attempts-end.md`, which lands in that session's own commit): with an identical BB command every feed, the landing bias relative to the request is per launch (median x -17 mm on 10-14 20:54, +6 mm on 21:27, -7 mm at sitting 8). BB's stroke, pitch and timing were identical by encoder, so the per-launch accepted calibration is the natural suspect. Data, not a conclusion: Phase 0 of the plan correlates it with the per-launch calibration.
