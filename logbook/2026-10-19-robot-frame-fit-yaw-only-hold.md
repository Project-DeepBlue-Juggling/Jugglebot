---
title: "Robot frame fit, second cut: yaw-only, drop-worst down to 5 markers, RMS cap 1.5 mm, a hold rule, no tilt guard - the first sitting showed 1.0-1.6 mm of per-configuration QTM reconstruction bias, not scatter"
type: bugfix
date: 2026-10-19
status: resolved
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration), Phase 3 second cut"
related_plan: bb-calibration-one-attempt.md
related_entries:
  - 2026-10-19-bb-calibration-phase3-robot-frame
  - 2026-10-17-bb-calibration-residual-cap-1p5
files_changed:
  - ros_ws/src/jugglebot/jugglebot/robot_frame.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/resources/jugglebot_robot_frame.json
  - tools/robot_frame_build.py
  - tools/probes/robot_frame_bag_check.py
  - tools/probes/README.md
  - tests/ros/test_robot_frame.py
  - tests/ros/test_robot_frame_resource.py
  - tests/hardware/session_skill_stack_rehearsals.md
  - plans/archived/bb-calibration-one-attempt.md
  - plans/active/INDEX.md
  - ros_ws/docs/bb_calibration_readings.md
  - logbook/2026-10-19-bb-calibration-phase3-robot-frame.md
  - logbook/2026-10-19-robot-frame-fit-yaw-only-hold.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
tags:
  - safety
  - testing
  - docs
---

# Robot frame fit, second cut

## Summary

Code landed 2026-10-19 (second cut), NOT flown. The first cut of the software robot frame (entry `2026-10-19-bb-calibration-phase3-robot-frame`) was flown the same morning in three launches and flapped. The frame fit is now yaw-only (robot z pinned to world z), drops the worst-residual marker while the RMS is over the cap (floor 5 markers), uses an RMS cap of 1.5 mm, and a failed fit whose uncapped pose agrees with the running frame does not count toward the clear. The tilt-vs-build guard is retired.

## Problem

Three launches the same morning, same QTM wand calibration, same AIM model, QTM not re-aligned: bags 2026-10-19_11-04-03 (home, 537 s), 11-13-17 and 11-14-54 (Jugglebot moved by hand: 108 mm / 4.2 deg at +65 s in the first, 40 mm / 11.8 deg at +37-40 s in the second; the runbook said "a few cm").
- 11:04: the frame flapped `RMS_OVER_CAP` / `TILT_FROM_BUILD` while the platform was parked high. Logged transitions at 125.2 UNK, 128.0 OK, 134.3, 150.8, 258.0, 261.5, 355.9, 359.7, 372.7, 384.3, 386.3 s.
- 11:13: a flap at +38.7/+40.6 s (RMS 0.999 against the 1.000 cap), then the +66/+67/+69 s transitions around the physical move.
- 11:14: OK at 39.8 s, UNKNOWN at 41.8 s, and the frame never locked in again (4.1 % of feeds with a frame).

## Analysis

Diagnosis (unit D1, `temp/probes/robot_frame_diag/report.md`, every `/mocap_data` frame decoded, about 300 Hz; a 5 Hz tracker replay reproduces 11:04 and 11:14 to within 0.1 s and the 11:13 move; the 11:13 flap at 0.999 vs 1.000 depends on feed phase).

The RMS is reconstruction bias that steps with configuration, not noise. Within a static stretch the per-frame fit scatters 0.02-0.05 mm about the stretch's own mean shape, but that mean shape sits 0.17-1.62 mm off the template:

| stretch | robot origin (mm), yaw (deg) | platform | RMS p50 (mm) |
|---|---|---|---|
| 11:04, 0-13 s | (0.1, 1.0, 2.2), 0.01 | at rest | 0.17 |
| 11:04, 93-105 s | same | raised, z 174 | 0.53 |
| 11:04, 135-150 s | same | raised, x -123 | 1.62 |
| 11:04, 431-520 s | same | at rest | 0.28 |
| 11:13, 0-65 s | (72.6, -15.1, 0.9), 8.45 | at rest | 1.00 |
| 11:13 68-91 s and 11:14 0-36 s | (-34.6, -3.6, 1.2), 4.24 | at rest | 1.05 |
| 11:14, 40-50 s | (-1.9, 18.9, 1.0), -7.51 | at rest | 1.06 |

The 35 mm and 108 mm moves read the same (1.00-1.06 mm), so the bias is not "grows with distance". Hypotheses:
- H1 one marker off: partly. Dropping the worst marker lowers the RMS ~25 % at the moved positions (the worst marker differs by position), but halves it in the occluded 11:04 stretches where Base - 5 is 1.8-2.9 mm off after Base - 2 or Base - 3 drops out.
- H2 position-dependent reconstruction bias: supported. Pair lengths (invariant to any fit) change up to 1.6 mm, e.g. Base 3-5 only 37 mm apart. The template matches only near home where it was learned.
- H3 label swaps: killed (no swap signature, no duplicate labels).
- H4 missing or ghost labels: partly. No ghosts; missing labels at home only (Base - 2 7.8 %, Base - 3 4.8 %); Base - 5 has the highest QTM residual and sits in line with 3 and 6.
- H5 platform: supported as platform POSE (occlusion), not motion. RMS vs platform z Spearman 0.72; 83 % of frames over 1.25 mm have the platform parked raised, not in transit.
Marker averaging over 5 or 25 feeds adds 2-5 points of lock at most: it removes the 0.02-0.05 mm scatter and leaves the bias. Re-learning the template at one position only moves the 1 mm to the others.

The fitted tilt-vs-build at the 11:04 floor spot reached 0.228 deg (segments 372-383 s) and varies with platform pose: bias, not the robot. The 0.2 deg tilt guard was a second flap source.

Variants replayed (tracker rules, cap / tilt / drop-worst min 5 / hold):

| variant | 11:04 | 11:13 | 11:14 |
|---|---|---|---|
| A shipped (cap 1.0, tilt 0.2) | lock 0 s, 91.5 % frame, 15 transitions | 74.8 %, 4 | lock 39.8 s, 4.1 %, 3 |
| B cap 1.5, tilt 0.3 | 96.1 %, 9 | 100 %, 1 | 100 %, 1 |
| C = B + drop-worst min 5 | 99.5 %, 3 | 100 %, 1 | 100 %, 1 |
| D = A + hold 2 mm | 99.5 %, 3 | 75.9 %, 2 | 4.1 %, 3 |
| E = C + hold 2 mm | 100 %, 1 | 100 %, 1 | 100 %, 1 |

Owner decision 2026-10-19: E, with a yaw-only fit in place of 6-DoF + tilt cap; the cap stays at 1.5 mm after the sweep below; Base - 5 recorded as residue.

## Discussion

**1. The morning's reading was reframed.** The first entry read the 0.66 mm template residual in the 10-16 bags as per-wand-calibration QTM scatter and set the cap at 1.0 mm as safe. The flown data say the residual is a per-configuration reconstruction BIAS of 1.0-1.6 mm: stable within a pose (scatter 0.02-0.05 mm), different between poses. A bound set from bags of one pose class (and from a cap that sat exactly on the 1.00-1.06 mm moved-position floor) could not have predicted that. The lesson for the cap: it must be set from the poses the robot will actually be flown in, not the pose class of the template-building bag.

**2. Tradeoff accepted: a looser cap and outlier dropping buy lock-and-hold at the cost of the cap no longer catching one displaced marker by itself.** With bias spread over all seven markers, no cap near the 1.0 mm floor can both reject a real fault and accept the bias; a per-marker cap fails the same way (moved-position per-marker residuals already reach 1.65-1.93 mm, Base - 5 reaches 2.9 at home). What now carries the single-marker job is drop-worst (an off marker is removed, not averaged into the pose), the 5-marker floor and `MIN_SPREAD` on the retained set. The cap itself stays at 1.5 mm, not 2.0 or 2.5, because of the replay sweep (`--replay --caps 1.5 2.0 2.5`, eight bags, 2026-10-19): every bag locks at 0 s and holds 100 % at every cap, so lock does not choose between them, but a single marker shifted 5 mm (Base - 1 or Base - 5) is dropped at 1.5 (error 0.18 / 0.10 mm) and accepted at 2.0 and 2.5 (7-wide, error 0.84 mm); only at 10 mm do all three drop it. The cost of 1.5 is that a few percent of fits drop one marker (5.3 % at home 11:04, mostly Base - 5; 15.5 % on the 09-27 kincal bag) at about 0.1 mm of origin.
The hold rule is defence for a position whose p50 would cross the cap; it was NOT exercised on any of the eight bags at 1.5 (no fit failed), and nothing here claims it fired.

**3. Yaw-only beat the proposed 6-DoF + 0.3 deg tilt cap.** The design already assumed a yaw-only robot frame (the tracker's gravity and the landing plane are world z); the 6-DoF fit invented a tilt degree of freedom that the physics does not have. The markers are nearly coplanar (z span 43 mm), so reconstruction bias leaks into fitted tilt (0.228 deg at the home floor spot, 0.16 deg built into the template), and a spurious 0.3 deg tilt shifts a point 830 mm up the robot by about 4 mm laterally (830 mm x 0.3 deg in radians). Pinning z to world z removes that failure class outright instead of bounding it with a guard that itself flaps. The price, accepted: yaw-only pushes the retired 0.16 deg build tilt into the residual (accepted-fit RMS p50 0.53 mm at home, 1.19 mm at the moved positions; p99 1.42 mm at 11:14 and 1.46 mm on the 09-27 kincal bag, 0.04 mm under the cap), which is why the cap is not lower.

## Change

- `robot_frame.py`: new public `yaw_fit` (2-D Kabsch on xy after centroid removal, z translation = mean z difference, R a 3x3 pure yaw with exact 0s and 1; residuals and RMS are 3-D; identity bit-for-bit). `RobotFrameFit` gains `dropped` and `uncapped`. Constants `ROBOT_FRAME_MAX_RMS_MM = 1.5`, `ROBOT_FRAME_MIN_FIT_MARKERS = 5`, `ROBOT_FRAME_HOLD_MM = 2.0`, `ROBOT_FRAME_HOLD_DEG = 0.3`; `ROBOT_FRAME_MIN_MARKERS`, `ROBOT_FRAME_MAX_TILT_FROM_BUILD_DEG`, `TILT_FROM_BUILD` and `z_axis_world_at_build` retired (39 grep hits -> 0). Rule: fewer than 5 matched markers -> `TOO_FEW_MARKERS`; retained-set spread under 10 mm -> `DEGENERATE_GEOMETRY` (neither has an `uncapped` pose); otherwise fit all, drop the worst-residual marker and refit while RMS > cap and more than 5 remain (a drop that would leave a degenerate set is not made). `n_matched` / `per_marker_mm` describe the retained markers. Tracker hold: a failed fit whose `uncapped` pose is within 2.0 mm / 0.3 deg of the running frame neither counts toward the 10-feed clear nor resets the count. `status()` gains `dropped` and `held`. Constant comments cite the diagnosis report.
- `mocap_node.py`: the OK line reads `Robot frame OK: rms .. mm, 6 markers (dropped Base - 5), origin .., yaw ..`.
- `resources/jugglebot_robot_frame.json`: tilt field removed, template points untouched. `tools/robot_frame_build.py` updated to match.
- `tests/ros/test_robot_frame.py`, `tests/ros/test_robot_frame_resource.py`: yaw-only, drop-worst, hold and the retired constants.
- `tools/probes/robot_frame_bag_check.py`: `--replay`, `--caps`, `--displace LABEL:MM` (production tracker class, mocap_node 5 Hz feed rule); README row updated. Its default mode still prints a "tilt of R" column that now always reads 0 (cosmetic).
- Docs: plan Phase 3 amended, rehearsal rows 35-38 rewritten, ledger note.

## Control analysis

The frame feeds landings, BB aim points, the Platform offset and the tracker landing plane once per attempt (snapshot at the session-start frame check, frozen for the attempt); the 40 Hz planner never sees it. A 1 mm origin error and a 0.05 deg yaw error (origin p95 0.03-1.45 mm and yaw p95 0.001-0.057 deg across the eight replayed bags) displace an aim point by about 1 mm plus 0.05 deg x its radius (about 0.1 mm at 125 mm), far inside the catch tolerance and the tracker's 200 mm gate; it is a constant offset per attempt, so there is no mid-flight discontinuity. The hold rule cannot mask a real move: a hold needs the uncapped pose within 2 mm / 0.3 deg of the running frame, and a moved robot disagrees by more than that. Measured on the 11:13 move: the frame never goes UNKNOWN (every fit during the slide is accepted, a rigid robot in motion still fits at 0.7-1.5 mm RMS), so it tracks the move with about 2.5 s of lag, passes through wrong mid-slide poses (up to about 85 mm off) for about 1.5 s and settles by about 3.7 s after the move starts. An attempt whose frame check fires within about 4 s of a hand move could snapshot a mid-slide pose (Residue).

## Verification

All dated 2026-10-19, PDJ venv.
- 11:56 AEDT, `pytest tests/ros/test_robot_frame.py tests/ros/test_robot_frame_resource.py tests/ros/test_mocap_node_robot_frame.py tests/ros/test_mocap_node.py tests/ros/test_mocap_frame_queue.py tests/ros/test_perception_console_lines.py tests/ros/test_skill_node_robot_frame.py tests/ros/test_ball_tracker_robot_frame.py tests/ros/test_skill_node.py tests/motion/test_skills_executor.py -q -n 4` -> 734 passed in 286.40 s. `cd ros_ws && colcon build --packages-select jugglebot` -> 1 package finished, exit 0.
- `pytest tests/ros/test_robot_frame.py -q` (after the probe edits) -> 54 passed in 0.59 s.
- Replay sweep: `python -I tools/probes/robot_frame_bag_check.py --replay --caps 1.5 2.0 2.5 --out temp/probes/robot_frame/replay_2026-10-19` -> 34 min 40 s, `replay.md`, log `temp/logs/robot_frame_replay_sweep.log`: eight bags x three caps, every combination locks at 0.0 s, 100 % of feeds with a running frame, one transition (OK at 0 s); no fit failure, no hold, no UNKNOWN at 1.5. At cap 1.5 origin p95 is 0.03-1.45 mm and yaw p95 0.001-0.057 deg over the eight bags; on the three 2026-10-19 bags origin p95 1.45 / 0.10 / 0.11 mm and yaw p95 0.033 / 0.002 / 0.002 deg.
- Invariance across QTM re-alignments (default mode, `--frames 1500`, four bags): fit origin minus QTM Base body (0.08, 0.88, 2.11) mm on three bags and (0.08, 0.89, 2.16) on 10-16_22-01; worst difference 0.05 mm (z), per-axis sd < 0.03 mm; it was (0.02, 0.92, 2.19) under the 6-DoF fit (moved <= 0.08 mm). Output `temp/probes/robot_frame/invariance_2026-10-19/bag_check.md`.
- Displacement guard (11:04 first 100 feeds, one marker shifted +x): table in Discussion 2 (5 mm dropped at 1.5, accepted with 0.84 mm error at 2.0 and 2.5).
- Post-sweep edits re-run (the probe lost a no-output in-replay invariance block after the sweep, and its default mode's always-zero "tilt of R" column was replaced by a yaw-only column): 2026-10-19 12:40 AEDT, `python -I tools/probes/robot_frame_bag_check.py --replay --bags 2026-10-19_11-14-54 --out temp/probes/robot_frame/replay_rerun_2026-10-19` -> exit 0, row identical to the sweep (246 feeds, lock 0.0 s, frame share 1.000, 1 transition, origin p95 0.11 mm, yaw p95 0.002 deg, drop share 0.016 (Base - 2), RMS p50/p99 1.19/1.42 mm); 2026-10-19 12:41 AEDT, `python -I tools/probes/robot_frame_bag_check.py --bags 2026-10-16_22-01-50 --frames 1500` -> exit 0 (`temp/probes/robot_frame/20261019-124113/bag_check.md`).
- Full gate 2026-10-19 12:50 AEDT, `./run_tests.sh --full` (log `temp/logs/gate_robot_frame_cut2_run2.log`): 6900 passed, 9 skipped in 373.58 s + serial 7 passed, exit 0. Build: `cd ros_ws && colcon build --packages-select jugglebot` 2026-10-19, exit 0.

## Residue / not done

- NOT flown: the hardware check (rehearsal rows 35-38; row 39 is the first-cut record for comparison, not a procedure) is re-owed against the second cut.
- Base - 5 and the 3-5-6 cluster stay the weak spot: highest QTM residual, in line with 3 and 6, 1.8-2.9 mm off when 2 or 3 drops out. Drop-worst absorbs it; only re-wanding QTM or camera coverage of that cluster lowers the floor.
- The hold rule is untested on real data (no fit failed at 1.5 on eight bags).
- A "frame settled" criterion (window spread) before the per-attempt snapshot is NOT built; until then, wait more than 5 s after setting the robot down before the first `skills/check`, or launch after the move.
- The GUI 3D view still draws markers and bodies in QTM world. (Resolved 2026-10-19 pm: the model follows `robot_frame`, see `2026-10-19-robot-frame-move-observability`.)
- The runbook's "few cm" move was actually 40-108 mm and 4-12 deg; the rows now say so.
- The 11:13 flap at +38.7/+40.6 s (RMS 0.999 vs 1.000) does not reproduce in the replay (feed-phase dependent); moot at 1.5 mm.
- First-cut record for the next sitting to compare (flown against the FIRST cut): home 11:04 locked at 0 s and flapped 15 times; 11:13 74.8 % frame; 11:14 locked once at 39.8 s and was UNKNOWN from 41.8 s.
- Update 2026-10-19 pm: the second cut was flown at 14:24 and the cap was raised to 2.0 mm the same afternoon (5 markers ran 0.2 mm under 1.5 after the move; lock-every-launch outranks the single-marker guard), see `2026-10-19-robot-frame-move-observability`. The 1.5 mm statements above are history.
