---
title: "BB calibration power-cycle A/B (2026-10-16 evening): kappa holds 179.27 deg across two BB-only power cycles (72 s and 86 min), so the step is not a power-up state; the long off deformed the marker constellation (pair Q3-Q6 1.2 mm against the template) and calibration refuses 7 of 8 sweeps until it is addressed"
type: investigation
date: 2026-10-16
status: in-progress
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration)"
related_plan: bb-calibration-one-attempt.md
files_changed:
  - logbook/2026-10-16-bb-calibration-power-cycle-ab.md
  - logbook/2026-10-16-bb-calibration-phase2-template-residual.md
  - logbook/INDEX.md
  - plans/active/bb-calibration-one-attempt.md
  - ros_ws/docs/bb_calibration_readings.md
subsystem:
  - tracking
  - ros
tags:
  - kinematics
  - hardware
  - docs
---

# BB calibration power-cycle A/B: kappa holds, the constellation deforms

## Summary

The owner ran the A/B the Phase 2 entry asked for: in one launch (20:30:57) four calibration sweeps, BB's own power off for 72 s, two more sweeps, BB off for 86 minutes, four more; then a launch restart (22:01:50) and four more. QTM, the frame and the room were untouched. Result: kappa holds. All 14 sweeps read 179.22 to 179.38 deg (median 179.27, the 15:08 level), including the seven refused sweeps re-solved with the residual cap lifted. Under the plan's 0.20 deg criterion a power-up state of the yaw stage is NOT the mechanism of the 0.3 deg step; the step happens between sittings and a BB-only power cycle does not reproduce it. What the long off did do: the marker constellation deformed. The Q3-Q6 pair separation went 0.05-0.16 mm off the template before the first cycle, 0.33-0.42 after the 72 s cycle, 1.14-1.25 mm after the 86 min off, and every sweep after the long off but one (0.89 mm, inside the 1.0 mm cap by 0.11) was refused with CONSTELLATION_RESIDUAL. The launch restart changed nothing: the refusals are a property of the geometry, not of the node. The deformation is geometric, not reconstruction (QTM's per-marker residuals after the long off equal the pre-off ones), it grew at each power event, and it persisted across the restart and three minutes of attempts. Owner's next (superseded by § E): the owner inspected the posts and found all seven markers rigid; the next sitting's first sweep reads the current state. Also flown tonight without being the subject: the calibration solve on a worker thread (7e74f42f) ran 14 sweeps with zero post-sweep frame-loss bursts. Late addendum (§ E): the owner found all seven markers rigid on BB's base, and the ODrive temperatures recorded in every bag split the kappa levels cleanly: every sitting at 15.5-17.2 C (legs' motor thermistors, idle) read 179.55-179.64 and every sitting at 19.7-22.9 C read 179.22-179.40, with no resolvable slope inside either cluster. Kappa's level is correlated with ambient temperature as a threshold (a correlation over 23 sittings with about two independent transitions, no controlled test yet); the Q3-Q6 deformation is not (it is set at each BB power-on). Both are now tabulated in a running ledger, `ros_ws/docs/bb_calibration_readings.md`.

## Symptoms

Owner's report: calibrated a few times, powered BB off for about a minute, calibrated a few times, powered BB off for a bit over an hour, then BB "would not calibrate" until the launch was restarted, and then only once in four tries. The two launch logs (`~/.ros/log/2026-10-16-20-30-57-*` and `2026-10-16-22-01-50-*`) show six `BB calibrated` lines before the long off and, after it, four `BB calibration FAILED: CONSTELLATION_RESIDUAL: marker-pair separations off the template by 1.14-1.25 mm > 1.00 mm` in the first launch and three more (1.19-1.23) plus one acceptance at 0.89 mm in the second. BB did sweep on every attempt.

## Diagnosis

Sources: the two bags (`2026-10-16_20-30-57`, 3.2 GB, deleted after extraction at the owner's request; `2026-10-16_22-01-50`, kept), the launch logs, the replay harness and two throwaway probes whose copies sit in the run directory's `extract/`.

### A. Timeline from `/bb/heartbeat` (10 Hz; local time)

| event | time | BB state / rest yaw (deg) |
|---|---|---|
| launch 20:30:57; BB BOOT | 20:31:02 | boot then IDLE, parks at 0.16 |
| sweeps #1-#4 | 20:31:13, 20:31:36, 20:32:00, 20:32:22 | rest 359.6-359.7 between |
| BB off (heartbeat `connected` false) | 20:32:49 to 20:34:01 (72 s) | last rest yaw 359.63; BOOT, IDLE reads 356.5 |
| sweeps #5, #6 | 20:34:03, 20:34:31 | rest 359.6 then 359.9 |
| BB off | 20:34:57 to 22:00:31 (85.6 min) | last 359.92; BOOT, IDLE reads 357.8; `bb_hand` ODrive reports DC_BUS_UNDER_VOLTAGE every 10 s for the whole window |
| sweeps #7-#10 | 22:00:39, 22:00:56, 22:01:12, 22:01:26 | all refused; rest 359.4-359.7 |
| Ctrl-C; launch 22:01:50 | pool 21 records, K1 median 179.27 n 13 | |
| sweeps #1-#4 (second launch) | 22:01:55, 22:02:12, 22:02:40, 22:03:04 | refused, ACCEPTED (0.89 mm), refused, refused |

The rest yaw after each reboot (356.5, 357.8) sits 2-3 deg below the reading before the off (359.6, 359.9) and the next sweep returns it to 359.6: the stage moves while de-energised and the absolute encoder reports the new physical angle. The leg ODrives logged nothing: only BB's supply was cycled.

### B. Kappa per sweep (replay; the refused sweeps solved with `max_residual_mm` lifted to 10)

| sweeps | kappa (deg) | template residual (mm) | lag (ms) |
|---|---|---|---|
| #1-#4, before any cycle | 179.220, 179.231, 179.379, 179.270 | 0.25, 0.38, 0.26, 0.25 | +1.2, +0.4, +1.3, +2.1 |
| #5, #6, after the 72 s cycle | 179.273, 179.268 | 0.33, 0.42 | +1.6, +1.2 |
| #7-#10, after the 86 min off (refused) | 179.268, 179.257, 179.249, 179.228 | 1.20, 1.20, 1.25, 1.14 | -1.2, +0.5, -1.3, +0.7 |
| second launch #1-#4 | 179.223, 179.273 (accepted), 179.371, 179.255 | 1.19, 0.89, 1.20, 1.23 | +0.3, -1.3, +0.1, +1.0 |

Node and replay agree on the seven accepted sweeps (34 of 35 measurements; the one miss is #3's template residual 0.2700 logged vs 0.2648 replayed) and on the seven refusals (0 of 7 accepted intrinsically). The shelf heading read 169.150-169.153 in the first launch and 169.130-169.132 after the long off (residual 0.24-0.42 then 0.25-0.29); the axis point moved 0.4 mm across the short cycle and 0.9 mm across the long one. Label coverage in the window is the same throughout (Q1 0.85, Q2 1.00, Q3 0.5-0.6, Q4 1.00, Q5 0.44, Q6 0.83, Q7 0.71), and the label-free pickup is null again (76,957 unlabelled near-axis points over the seven accepted sweeps, 0 within 3 mm of a posed marker).

### C. Which pair, and when it moved

Pose-binned |median| pair residual against the template (mm), the pairs that changed:

| sweeps | Q3-Q6 | Q1-Q3 | Q3-Q7 | Q3-Q4 | Q3-Q5 | Q4-Q6 | Q2-Q6 | Q5-Q6 | Q1-Q6 | Q6-Q7 |
|---|---|---|---|---|---|---|---|---|---|---|
| #1-#4 | 0.05-0.16 | 0.01-0.07 | 0.02-0.07 | 0.10-0.16 | 0.08-0.13 | 0.01-0.08 | 0.07-0.11 | 0.02-0.25 | 0.05-0.06 | 0.08-0.09 |
| #5, #6 | 0.33, 0.42 | 0.18, 0.21 | 0.19, 0.21 | 0.18, 0.22 | 0.16, 0.21 | 0.19, 0.23 | 0.23, 0.26 | 0.10, 0.12 | 0.02 | 0.09, 0.07 |
| after the 86 min off (8 sweeps) | 0.89-1.25 | 0.66-0.75 | 0.51-0.58 | 0.44-0.45 | 0.37-0.40 | 0.50-0.56 | 0.46-0.51 | 0.46-0.49 | 0.01-0.02 | 0.00-0.03 |

Pairs among Q1, Q2, Q4, Q5, Q7 stayed at or below 0.34 mm throughout. Q3 moved against every other marker; Q6 moved against Q2, Q3, Q4, Q5 but not against Q1 or Q7. In the body frame the mean residual y went Q3 +0.05 to -0.15 to -0.46 mm and Q6 -0.01 to +0.18 to +0.47 mm across the three stages (Q1 -0.18 to -0.02 to +0.21, Q7 +0.03 to +0.08 to +0.21, Q2 +0.10 to +0.01 to -0.16). Template geometry: Q3 at azimuth -47 deg, radius 118 mm, z -22; Q6 at +29 deg, 118 mm, z -20; Q4 between them at 0 deg; Q1 and Q2 on the lower ring (z -47/-48, radius 133); Q7 at 60 deg, radius 75. QTM's per-marker reconstruction residual inside the sweeps: Q3 0.24-0.32 mm before the cycles, 0.09-0.13 after the 72 s cycle, 0.16-0.34 after the long off; Q6 0.15-0.31, 0.07-0.12, 0.27-0.33. Reconstruction quality after the long off is the pre-cycle quality; the pair separations are not.

### D. The frame-loss bursts

Two `QTM frames lost` warnings in 91 minutes, both "never received from QTM" (9.8 % at 20:31:12 as the launch came up, 9.9 % at 21:34:44 with BB off); none in the 10 s after any of the 14 sweep ends. The morning's 30-52 % post-sweep bursts (`2026-10-16-mocap-frame-queue-starvation.md`) are gone with the worker-thread solve (7e74f42f), which this sitting flew for the first time. The parallel session's read of the same logs (its row 30b receipt): an accepted solve now takes 14.4-15.7 s on the worker thread against 4.3-7.2 s inline (the GIL is shared with the 300 Hz frame work), a refusal 5.4-6.3 s; the outcome line arrives that much later and the operator should expect it.

### E. Owner's marker check, the direction split and the temperature reading (2026-10-16 late)

The owner inspected BB after reading § C: all seven markers are rigidly attached to BB's base, and their hypothesis was heat (the room, or the cameras warming over a long day). Three further readings:

- **Direction split.** The signed Q3-Q6 pose-binned pair residual computed on the yaw-rising and yaw-falling legs separately (`extract/dirsplit_report.txt`): +1.21/+0.93 (#7), +1.00/+0.96 (#8), +1.02/+1.01 (#9), +1.00/+0.96 (#10), +1.03/+0.93, +0.83/+0.99, +1.05/+1.03, +1.02/+0.94 (second launch), +0.32/+0.37 and +0.43/+0.41 (#5, #6). Same sign and size on both legs, so the error is geometric, not a camera-timing artefact (which displaces moving markers along their velocity and flips with direction). The 10:51 sweeps are the one exception in the day: Q3-Q6 -0.50 on the rising leg against -0.06 on the falling leg, with the day's largest positive lag (+5.4 and +4.4 ms; the 16:10 sweep's lag was -11.9 ms with a symmetric split of -0.03/+0.11, and 16:23 was not decoded).
- **The deformation is set at BB power-on and holds within a sitting.** Signed Q3-Q6 across the day's power events, all at 20.0-20.6 C from 15:08 on (measured at 15:08, 16:10, 16:23 and 22:01; 20:31 and 20:34 not measured, that bag is deleted, so inferred from the neighbours): 10:51 -0.07, 15:08 +0.44 to +0.46 (seven sweeps), 16:10 -0.05, 20:31 +0.05 to +0.16, 20:34 (after 72 s off) +0.33/+0.42, 22:00 (after 86 min off) +1.14 to +1.25, 22:02 (launch restart, no BB power event) +0.89 to +1.23. Not thermal: the three 20 C sittings span the whole range. The shelf constellation (240 mm span, 244 mm from BB's axis by the replay's base origin and axis point, so the same cameras) held 0.20-0.31 mm across the day (0.25-0.29 after the long off).
- **Kappa's level is correlated with ambient temperature, as a threshold.** `/robot_state` carries every ODrive's FET and motor thermistor temperatures; the legs' motor temperature at a sitting's start (motors idle for hours) is a room-temperature proxy. Joined to the replay rows for every bag from 10-10 to 10-16 (`extract/motor_temps.txt`, probe `extract/probe_motor_temps.py`):

| legs motor temperature (C) | sittings | kappa |
|---|---|---|
| 15.5, 16.2, 16.4, 16.5, 16.7, 16.8, 17.2 | 10-14 20:54, 10-14 21:27, 10-13 22:00, 10-14 22:00, 10-13 22:29, 10-16 10:51, 10-13 23:13 | 179.63, 179.64, 179.55, 179.59, 179.60, 179.57, 179.57 |
| 19.7, 19.9, 20.0, 20.0, 20.0, 20.1, 20.6, 21.0, 21.1, 21.4, 21.6, 21.7, 21.7, 22.1, 22.3, 22.9 | 10-12 23:24, 10-11 11:28, 10-13 00:18, 10-16 15:08, 10-16 16:10, 10-16 16:23, 10-16 22:01, 10-10 23:32, 10-10 12:26, 10-10 13:46, 10-11 15:14, 10-10 14:11, 10-10 14:13, 10-10 14:49, 10-10 19:23, 10-11 22:16 | 179.27, 179.23, 179.27, 179.26, 179.34, 179.24, 179.27, 179.22, 179.33, 179.33, 179.32, 179.40, 179.32, 179.30, 179.27, 179.31 |

No overlap, and no resolvable slope inside either cluster: 15.5 C and 17.2 C both read about 179.6 (the cold cluster drifts about -0.06 deg over 1.7 K, inside the sitting scatter), 19.7 C and 22.9 C both read about 179.3. A linear coefficient of 0.3 deg per 3.2 K (the 10:51 to 15:08 pair) would put 0.30 deg of slope across the warm cluster's 3.2 K; the warm cluster's spread is unordered. This is a correlation over 23 sittings that sort into one contiguous cold block (10-13 22:00 to 10-16 10:51, with 15:08 the first warm sitting after it) and the warm rest, so about two independent transitions; no controlled test yet. The BB hand ODrive's FET temperature splits the same way (18-22 C cold, 23-27 C warm). The two sittings that defined the step in Phase 1 are the two sides of it: 10:51 at 16.8 C after a cold night, 15:08 at 20.0 C. The full per-sitting table, the per-event rows and a hypotheses register with the discriminator for each live in `ros_ws/docs/bb_calibration_readings.md`, kept as a ledger to which later sittings add rows.

## Discussion

1. **Why the A/B reads "not a power-up state".** The Phase 2 entry pre-registered it: kappa jumping only across cycles pins a power-up state of the yaw stage; kappa holding across cycles and stepping only between sittings points at something that happens between sessions. Tonight it held to within the sitting's own scatter (0.16 deg range, 179.27 median) across a reboot, a 72 s off and an 86 min off, with the refused sweeps counted (their cap-lifted kappa sits in the same range). Two cycles are not ten, but the 0.3 deg step would have been about 2 times the sitting's full range (0.16 deg) and 6 times its standard deviation (0.05 deg), and it is absent in both. The firmware reading stands with it: the stage drifted 2-3 deg while unpowered, the encoder reported the new angle, the sweep took it back, and the encoder-to-marker mapping (kappa) did not move.

2. **What a sitting boundary has that tonight's cycles did not: the room temperature (§ E).** Written before the temperature join, this item read "the whole stack's power-down, the hours between sittings, QTM's session start, whatever the room does"; the join strongly suggests it. Every step-up was a cold sitting and the one step-down was cold to warm, and tonight's cycles were all at 20-21 C. The form of the dependence (two flat levels, a threshold between 17.2 and 19.7 C) suggests a bistable thing rather than a coefficient: a clearance or preload in BB's yaw drive that temperature closes or opens (the stage already swings 2-3 deg when de-energised, so something loads it), the encoder magnet's geometry, or a QTM-side effect local to BB's small constellation. The shelf control argues against the last: a global thermal drift cancels in kappa by construction and the shelf heading held to 0.001-0.002 deg across the step. The cheapest BB-side test needs no mocap: read BB's yaw at a mechanical reference (a hard limit) cold and warm in one day; a 0.3 deg shift at the stop is the drive or the encoder.

3. **The deformation is geometric and power-event-locked, so it is not reconstruction and not thermal (§ E settles the second half: the 20 C sittings span the whole range).** QTM's residuals after the long off equal the pre-cycle ones, the shelf residual did not move, and the label coverage is identical, so the 1.2 mm is in the marker geometry. It stepped at the 72 s cycle (0.1 to 0.35 mm) two seconds after BB came back, which rules out cooling for that step; the long off added 0.8 mm, which cooling could contribute to but handling could equally. Both cycles involved the owner at BB's switch and the stage's unpowered swing. Q3 and Q6 are the carriers, on the upper plane at radius 118 mm on opposite sides of Q4; the fact that Q6 kept its separation to Q1 and Q7 to 0.03 mm says Q6, Q1 and Q7 moved together relative to Q2, Q4, Q5 and especially Q3. A 0.5 mm tangential shift at 118 mm is 0.24 deg of local rotation of whatever carries those posts.

4. **The cap is doing its job; do not relax it.** The one acceptance at 0.89 mm was the edge of the band, not a recovery (0.89 is also just under the 0.90 advisory, by 0.01, so no advisory fired). Letting 1.2 mm through would fit a rigid template to a non-rigid constellation; the cap-lifted kappas show the yaw bias is small tonight (within 0.1 deg of the accepted sweeps) but that is not a property to rely on. The Phase 2 reference level 0.18-0.46 mm remains the healthy range.

5. **The template builder is gone.** The template's provenance names `temp/bbyaw/sweep/build1_template.py .. build4_write.py` in the `Jugglebot-bbyaw` worktree, uncommitted; that worktree and its `temp/` no longer exist. A rebuild (Generalised Procrustes over labelled frames, yaw-balanced at 40 frames per 2 deg bin, parked frames excluded) is a small unit with the harness's tracking already in place, but rebuilding to a deformed geometry is the owner's decision against fixing the posts, and the gauge pin, E(y) and the repeatability travel with the template.

6. **Control-system implications: none.** No code changed. The refusals leave the node on the 2026-10-16T11:02:34Z calibration (kappa 179.27), which is the sitting's own level.

## Fix

None in code. This entry, the Phase 2 entry's open question answered, the plan rows, the readings ledger `ros_ws/docs/bb_calibration_readings.md` (new: per-sitting table with temperatures, per-event rows, hypotheses register, how to add a sitting), and the run directory `temp/probes/bb_calibration_replay/20261016-221458` (replay outputs plus `extract/`: the two decoded bags' sweep windows as pickles, the heartbeat timeline, the probe reports and copies of the three probes).

## Verification

- 2026-10-16, merge of `bb-calibration` into `skill-stack` (ce97ba38; INDEX row-order conflict only), `./run_tests.sh` on the merged tree (venv, log `temp/logs/run_tests_merge_bbcal_phase2.log`): parallel 6709 passed, 11 skipped in 322.61 s; serial 4 passed in 17.67 s; exit 0. One pre-existing warning: the `replay-overview` thread exception in `tests/ros/test_replay_overview.py` (the known flake's latent `_drain` race).
- 2026-10-16, `python tools/probes/bb_calibration_replay.py --bags ~/Desktop/rosbags/2026-10-16_20-30-57 ~/Desktop/rosbags/2026-10-16_22-01-50 --subset 1,2,4,6,7 --loo -v` (run `temp/probes/bb_calibration_replay/20261016-221458`, 459 s): 14 sweeps replayed, 7 accepted, 7 CONSTELLATION_RESIDUAL; 34 of 35 measurements match the node; 0 of 7 refusals accepted intrinsically.
- 2026-10-16, `python /tmp/probe_bb_heartbeat.py <bag>` and `python /tmp/probe_bb_evening.py decode <bags>` then `analyse` (copies in the run directory's `extract/`): the tables in § A to § C (`extract/heartbeat_2030.txt`, `extract/evening_report.txt`).
- 2026-10-16, `pytest tests/sim/test_logbook*.py tests/sim/test_plans_index.py -q` after the documentation edits (this entry, INDEX, plan, Phase 2 entry): 111 passed, 2 skipped in 1.02 s; re-run 2026-10-16 late after § E, the ledger and the plan addendum: 111 passed, 2 skipped in 0.88 s.
- 2026-10-16 late, `python /tmp/probe_motor_temps.py temp/probes/bb_calibration_replay/20261016-181127/sweeps.csv temp/probes/bb_calibration_replay/20261016-221458/sweeps.csv` over the 25 retained bags with sweeps (the deleted 20:30 bag reports "no bag"; copy and output under `extract/`): temperatures for 88 sweeps in 23 sittings with a kappa; the table in § E. `python /tmp/probe_bb_dirsplit.py <pickles>`: `extract/dirsplit_report.txt`.

## Outcome

True now: the 0.3 deg kappa step is not a BB power-up state (two BB-only cycles, kappa 179.27 throughout) and its level is correlated with ambient temperature as a threshold between 17.2 and 19.7 C (§ E, every sitting 10-10 to 10-16 with a temperature; correlation, no controlled test yet); the firmware reading holds; the marker constellation deformed at the power events (Q3-Q6 1.2 mm) and calibration refuses until the geometry is back inside 1.0 mm or the template is rebuilt; the worker-thread calibration solve flew clean over 14 sweeps; the acceptance-2 tally is 4 of 10 (15:08, 16:10, 16:23, 20:30 first sweeps accepted with no override), with the 22:01 launch's first sweep refused on a true residual.

NOT true: the step's mechanism is not identified (temperature says where to look, not what); the deformation's cause is not identified (set at BB power-on; markers rigid; not thermal); there is no committed template builder.

## Open Questions

- Kappa vs temperature, BB-side or QTM-side: (a) read BB's yaw at a mechanical reference (a hard limit) cold and warm in one day, no mocap needed; (b) a rigid dummy constellation with BB-like spans next to BB, its fitted yaw against the shelf heading cold and warm; (c) a controlled room-temperature change inside one launch (heater), sweeps every 15 min, QTM camera temperatures noted from QTM. The ledger `ros_ws/docs/bb_calibration_readings.md` takes the rows.
- The Q3-Q6 deformation: markers are rigid (owner, 2026-10-16), so the candidates are a BB part whose position is set at power-on partially occluding Q3 or Q6 (QTM's per-camera 2D views across a power cycle decide it), a base sub-assembly re-seated by the unpowered swing, or the cable harness. The next sitting's first sweep reads the current state; calibration refuses until it is back inside 1.0 mm or the template is rebuilt (builder gone; rebuild is the owner's call against the hardware check).
- Acceptance 2 wording (plan § 0): owner decision pending; whether the 22:01 launch's refused first sweep (a true residual refusal, not a gate false positive) resets the "consecutive" count is part of it.
- The 10:51 sweeps' direction asymmetry (Q3-Q6 -0.50 rising vs -0.06 falling, lag +5 ms): the only such case in the day, on the cold side; unexplained.
- The worker-thread flight result is the parallel session's row 30b receipt (PASSED on these logs; `2026-10-16-mocap-frame-queue-starvation.md`); the 14-16 s accepted-solve time it measured is a calibration-UX item for that session, not this plan.
