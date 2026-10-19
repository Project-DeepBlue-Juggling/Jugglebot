---
title: "BB calibration Phase 3: the robot frame is measured in software (fitted from the labelled Base - 1..7 markers, the kincal frame) and the BB side stays in QTM world - two join points carry the transform, a robot frame never received this launch always refuses, the transform is frozen per attempt, the tilt guard is tilt-vs-build"
type: feature
date: 2026-10-19
status: resolved
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration), Phase 3"
related_plan: bb-calibration-one-attempt.md
related_entries:
  - 2026-10-17-bb-calibration-residual-cap-1p5
  - 2026-10-16-bb-calibration-phase1-gate-inversion
  - 2026-09-23-cup-contact-first-sitting
files_changed:
  - config/generate_config.py
  - config/generated/geometry-config.js
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - config/protocol_config.yaml
  - ros_ws/docs/bb_calibration_readings.md
  - ros_ws/docs/choreography.md
  - ros_ws/gui/js/geometry-config.js
  - ros_ws/gui/js/panels.js
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/protocol_config.h
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - ros_ws/src/jugglebot/jugglebot/mocap_interface.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/mocap_status.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - ros_ws/src/jugglebot/jugglebot/robot_frame.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/resources/jugglebot_robot_frame.json
  - tests/hardware/session_bb_calibration_phase1.md
  - tests/hardware/session_cup_contact.md
  - tests/hardware/session_kincal.md
  - tests/hardware/session_phase8_toss_trace.md
  - tests/hardware/session_skills_r3.md
  - tests/hardware/session_skills_r4.md
  - tests/hardware/session_skill_stack_rehearsals.md
  - tests/motion/test_skills_executor.py
  - tests/ros/test_ball_tracker_robot_frame.py
  - tests/ros/test_mocap_frame_queue.py
  - tests/ros/test_mocap_node.py
  - tests/ros/test_mocap_node_robot_frame.py
  - tests/ros/test_perception_console_lines.py
  - tests/ros/test_robot_frame.py
  - tests/ros/test_robot_frame_resource.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_robot_frame.py
  - tools/probes/README.md
  - tools/probes/robot_frame_bag_check.py
  - tools/probes/robot_frame_residual_probe.py
  - tools/robot_frame_build.py
  - plans/archived/bb-calibration-one-attempt.md
  - plans/active/INDEX.md
  - logbook/2026-10-19-bb-calibration-phase3-robot-frame.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
  - motion
  - config
  - gui
tags:
  - kinematics
  - safety
  - testing
  - docs
---

# BB calibration Phase 3: robot frame measured in software

## Summary

Code landed 2026-10-19, NOT flown. `mocap_node` fits the Jugglebot robot frame from the labelled `Base - 1..7` markers (new pure module `robot_frame.py`, frozen template `resources/jugglebot_robot_frame.json`) and publishes it as the latched `robot_frame` topic; `aligned` is now that fit's status. QTM is no longer re-aligned to the base. The transform enters at two join points only (`skill_node` and the `ball_tracker_node` landing plane); the BB side is untouched. With an identity transform every output equals today's bit for bit.

## Problem

Plan Phase 3 (C4, and the standing assumption that the QTM origin IS the robot base). Today the robot frame is whatever QTM's global frame happens to be: an operator re-aligns QTM to the base marker before sittings, the `Base` body alignment check (2000 mm / 180 deg) never fires, and every re-alignment (10-11, 10-12, 10-14, 10-16) silently moves the world frame under the aim points, the catch sites and the Platform offset. Nothing in software knows where the robot is; the sitting relies on a manual step. A shifted or tilted QTM frame displaces where the robot is sent and where it catches.

## Analysis

Census (probe 2026-10-19, 3000 frames per bag): seven labels `Base - 1..7`, each present in 91-100 % of frames, per-frame sd 0.01-0.05 mm. The constellation relative to the QTM `Base` 6DOF body is unchanged from 2026-09-27 to 2026-10-16 to about 0.4 mm; QTM was aligned to the body within about 3 mm / 0.05 deg in every bag, so the transform is near-identity today. The kincal bag (`2026-09-27_14-31-13`) CSV header says `frame: Base = QTM global`, and its Base body pose is (-0.04, -0.66, -1.73) mm, 0.036 deg: that frame is the one the legs' kinematic calibration was fitted in, so it is the robot frame by definition (the ~0.8 deg lever-arm tilt of logbook 2026-09-23 is inside it; the Platform offset check keeps absorbing its residual at rest height).

Residual probe (`tools/probes/robot_frame_residual_probe.py`, kincal-shape template, 3000 frames per bag): kincal bag RMS median 0.019 / p99 0.057 / max 0.32 mm; 2026-10-16_16-23-01 0.665 / 0.679 / 0.70 mm; 2026-10-16_22-01-50 0.655 / 0.681 / 1.145 mm. 1 mm on one marker adds about 0.33 mm RMS. The markers are nearly coplanar (z span 43 mm; Base 3/5/6/7 near-collinear, spread 3.3 mm): 1 mm of Base-7 z is only 0.11 mm RMS but 0.12 deg of tilt. Between-bag z-axis spread is 0.022-0.036 deg, per-frame p99 0.006-0.011 deg. Same-wand bags differ by about 0.17 mm.

Bag check (`tools/probes/robot_frame_bag_check.py`, 4000 frames per bag, every 2nd fitted; run `temp/probes/robot_frame/20261019-101439/bag_check.md`): fit origin minus the QTM Base body is (0.02, 0.91..0.92, 2.19) mm in all five bags (sd <= 0.03 mm) while the fit's world translation moves by up to 3.2 mm: the robot frame follows QTM's re-alignments, which is the invariance the phase needs. The 23:32 re-definition of 2026-10-10 is invisible at the Base body (fit minus body identical before and after).

## Discussion

Three decisions changed the plan's shape; each is written here before the Change.

**1. BB side stays in QTM world, robot side transformed at two join points (not "everything in the base frame + artefact migration").** The plan said to re-express every consumer in the base frame and migrate the five BB artefacts once. The root cause of the frame problem is that the ROBOT's position in QTM is unknown, not that the BB calibration is in the wrong frame: Ball Butler, the shelf base frame, the sweep calibration, the affine correction and the gauge pin are all measured in the QTM world and consumed in it, and `bb/throw_at_target` takes world points. Migrating them would have rewritten safety-adjacent state (a wrong migration moves every throw) to fix a problem that lives on one side. Transforming only the robot side needs the transform at exactly two places where world data meets robot-frame data: `skill_node` (landings and announcements in, BB aim points out, the Platform frame check, the magazine) and the tracker's landing plane. No artefact migration is then needed. Failure mode prevented: a migration bug silently displacing BB throws; and the class "some consumer still reads QTM world" is closed by the two-join-point structure rather than a sweep of every module.

**2. The 0.66 mm template residual is QTM scatter, NOT moved markers (reading withdrawn).** The first reading of the probe table was that the base markers had moved about 1 mm between 09-27 and 10-16 (so the cap would need relaxing or the template a per-day refit). The owner withdrew it on 2026-10-19: the base markers have not moved, and the 0.66 mm shape difference is QTM's per-wand-calibration reconstruction scatter (QTM was re-wanded 2026-10-16; the shelf markers show 0.05-0.48 mm sitting-to-sitting pair variation, and same-wand bags differ by only 0.17 mm). That changed the reasoning on two points: the template SHAPE is taken from the freshest bag (live fit 0.03 mm) and REGISTERED once into the kincal frame (registration RMS 0.695 mm, per marker 0.73, 0.88, 0.37, 0.56, 0.33, 1.18, 0.38, recorded as expected, not an error), and the RMS cap stays 1.0 mm instead of being widened to cover a shift that was not real. Failure mode prevented: widening a safety cap to absorb a misread.

**3. The tilt guard is tilt-vs-build (0.2 deg), not a per-marker cap.** A per-marker residual cap does not bound the tilt: with the markers nearly coplanar, a 1 mm vertical error on one marker is invisible to RMS (0.11 mm) yet tilts the fit 0.12 deg, and 0.2 deg of tilt is 0.9 mm per 250 mm of height. The guard therefore compares the fitted z-axis with the z-axis recorded at build (`z_axis_world_at_build`), against a cap 0.2 deg, about 5 times the between-bag spread (0.022-0.036 deg). Failure mode prevented: a marker mislabelled or bumped vertically passing the RMS test and tilting every aim.

**Also decided (owner, 2026-10-19):** the robot frame is the kincal frame (the legs' calibration frame, so the planner never sees a transform); a robot frame never received this launch ALWAYS refuses, at any learner authority; a later loss of the markers warns and reuses the frame received earlier (the alternative, falling back to identity, silently reintroduces the manual step that this phase removes); "known" means a `robot_frame` received this launch, older than 10 s is one WARN not a refusal; the transform is frozen per attempt (a mid-attempt fit change would jump the aims), the tracker plane follows live (sub-mm effect); `rigid_body_poses` keeps its contract (world xy, z shifted by `GEOM_INITIAL_HEIGHT_MM`) and the frame check transforms what it reads; runsheet rows are rewritten, not bannered.

## Change

- `robot_frame.py` (new, pure Python + numpy, no ROS): `RobotFrame` (R robot to world, t; `to_world`, `to_robot`, velocity rotation; the EXACT identity returns its input bit for bit), the template (load / write / learn / `register_template`), `fit_robot_frame` (reasons `TOO_FEW_MARKERS`, `DEGENERATE_GEOMETRY`, `RMS_OVER_CAP`, `TILT_FROM_BUILD`) and `RobotFrameTracker` (window 10, median origin, averaged rotation, outlier drop 1.0 mm / 0.05 deg, clears after `window` failed fits). Constants: min markers 4, min spread 10 mm, RMS cap 1.0 mm, tilt-vs-build 0.2 deg.
- `resources/jugglebot_robot_frame.json` and `tools/robot_frame_build.py`: template from bag 2026-10-16_22-01-50 registered into the kincal bag frame (2400 frames per bag, `--every 5`), `z_axis_world_at_build` [-0.002601, 0.001058, 0.999996], tracker fit RMS on the shape bag 0.027 mm.
- `mocap_node.py`, `mocap_interface.py`, `mocap_status.py`, `config/protocol_config.yaml` + `generate_config.py` (+ regenerated artefacts), `panels.js`, `choreography.md`: labelled markers fed to the tracker at 5 Hz; latched `robot_frame` PoseStamped (frame_id `world`) on first fit and on a move over 0.2 mm or 0.005 deg; `aligned` and `robot_frame_ok` := the tracker's running (windowed) frame exists, so one bad feed changes nothing and 10 consecutive misses (~2 s at 5 Hz) clear it; an unchanged frame is re-published every 5 s as a latched heartbeat (skill_node's age is receive-time based); status keys `robot_frame_ok`, `_rms_mm`, `_n_matched`, `_age_s`, `_reason`; one INFO `Robot frame OK` / one WARN `Robot frame UNKNOWN: <reason>` per transition; static TF world to platform_start follows the fit. Retired: the Base-body alignment check, `set_alignment_thresholds`, the `mocap_alignment` block and `MOCAP_ALIGNMENT_*` constants. GUI pill reads "Robot frame OK / Robot frame UNKNOWN".
- `motion/skills/executor.py`, `INVARIANTS.md`: ladder row `REJECTED_ROBOT_FRAME_UNKNOWN` (after the mocap-stale row, dropped for a REST), `Observations.robot_frame_known` (default True, so the sim gate is unchanged).
- `skill_node.py`: latched `robot_frame` subscription; `_snapshot_robot_frame` first inside `_frame_check_error` (every start path, the magazine, `skills/check`), refuses when none was received, never re-snapshots while an attempt runs; `_world_to_schedule` (pos to robot frame, vel rotated, then minus the Platform offset) for `_on_balls` and the announced-reload install; `_mocap_aim_point_mm` takes the aim to world with the frame; the Platform frame check and the magazine carry their samples into the robot frame; `_maybe_announce` converts positions to world and rotates velocities.
- `ball_tracker_node.py`: landing plane = world z of the robot-frame point (0, 0, `landing_z_mm`) through the latched topic (new parameter `landing_z_mm`, default `CATCH_CUP_Z_MM`); one INFO only if the plane moves over 0.5 mm; about 2 mm across the +-125 mm sites at 1 deg, inside the 20 mm landing threshold.
- Docs/probes: `tools/probes/robot_frame_residual_probe.py`, `robot_frame_bag_check.py` and their README rows; runsheet rows rewritten (R3 10a / 21a, R4 7, cup_contact 6 / 9 / notes, kincal 1 / 3, phase8 toss trace, bb_calibration_phase1 R4 retired); new rehearsal rows 35-38; plan Phase 3 and status; ledger note.

## Control analysis

The 40 Hz planner (`trajectory_node`) commands in the robot frame and never sees the transform. It enters only per-attempt inputs: tracker landings, BB aim points, the Platform offset and the tracker landing plane. Frozen per attempt, so no mid-flight jump from a fit change; a landing can change only through the tracker as today (bounded by I-CATCH-5). One cycle with the transform applied: BB throw, aim_robot = site + offset, aim_world = R aim_robot + t, BB resolves world to BB-local with its (world) calibration and throws, the announcement landing (world) goes through `_on_announcement` as R.T (p - t) minus the offset into a LandingPrior (robot frame), the catch is planned, `/balls` landings (world) take the same transform in `_on_balls`, the executor re-aims. With identity every number equals today's.

## Verification

All dated 2026-10-19; scoped runs in the PDJ venv. Nothing is flown.

- Unit 0a: `pytest tests/ros/test_robot_frame.py -q` -> 45 passed in 0.72 s.
- Unit 0b: `pytest tests/ros/test_robot_frame_resource.py -q` -> 4 passed in 0.33 s. Bag check (`python -I tools/probes/robot_frame_bag_check.py`, 4000 frames, every 2nd): table in Analysis.
- Unit 1: `tests/ros/test_mocap_node_robot_frame.py` 7 passed; updated `test_mocap_frame_queue.py`, `test_mocap_node.py`, `test_perception_console_lines.py` 14 passed; choreography drift tests pass after regeneration.
- Unit 2: `python -m pytest tests/ros/test_skill_node*.py tests/motion/test_skills_executor.py -q -x -n 4` -> 603 passed in 259.15 s; `python -m pytest tests/ros/test_skill_node_robot_frame.py -q` -> 11 passed in 3.56 s (identity bit-exact on five paths; yaw 30 deg, roll 1 deg and (100, -50, 5) mm invariance; refusal at authority 0 and 40; `skills/check` ladder line; malformed message; freeze twice; an 11 s stale frame warns once).
- Unit 3a: `pytest tests/ros/test_ball_tracker*.py -q` -> passed, including the 5 new.
- Full gate (`./run_tests.sh --full`, run 2026-10-19 after the audit fixes, log `temp/logs/gate_phase3_run2.log`): **6891 passed, 9 skipped in 310.99 s + serial 7 passed in 27.94 s, exit 0**. (Run 1, before the audit: 6884 passed / 3 failed - the choreography map needed regeneration after the later units added `robot_frame` subscribers, and the not-seen-yet state logged a WARN; both fixed before run 2.) Build: `cd ros_ws && colcon build --packages-select jugglebot`, 2026-10-19: 1 package finished in 3.46 s, exit 0.

## Residue / not done

**Superseded in part 2026-10-19:** the 1.0 mm RMS cap, the "0.66 mm is per-wand scatter" reading and the 0.2 deg tilt-vs-build guard did not survive the first sitting (three launches flapped; the residual is 1.0-1.6 mm of per-configuration QTM reconstruction bias). Replaced by a yaw-only fit, drop-worst down to 5 markers, cap 1.5 mm, a hold rule and no tilt guard: see `2026-10-19-robot-frame-fit-yaw-only-hold.md`. The rest of this entry (join points, freezing, refusal) stands.

- Poses on `rigid_body_poses` are published in WORLD xy (z shifted) under frame_id `platform_start`; the static TF world->platform_start (R@(0,0,z0)+t, rotation R) is for visualisation only and a tf2 consumer must not apply it to those poses (none exists today; grep confirmed).
- NOT flown: dress rehearsal and the plan's hardware check (move Jugglebot, do not re-align QTM; rehearsal rows 35-38) are owed.
- The GUI 3D view still draws markers and bodies in QTM world with the robot at the origin; if the robot is moved without re-alignment the drawing is offset by the transform.
- `_maybe_announce` (Jugglebot's own `ThrowAnnouncement`) does not add the Platform offset (identity case bit-identical; a sub-cm effect inside the tracker's 200 mm gate).
- The two bags the plan named for the replay pin (2026-10-10_19-23-29 and 23-32-12) carry no `Base - n` labels (labels exist from 10-10_14-13-18 on), so the pin used 10-10_14-13-18 and 10-11_11-27-53; the Platform-at-rest comparison is only partly conclusive (different rest poses per bag).
- The generator also rewrote `/home/jetson/Desktop/BallButler/ball_butler_main/protocol_config.h`; that repo shows a modified file.
- The kincal bag has isolated later frames over the 1.0 mm cap (p99 RMS 1.349 mm), absorbed by the tracker window; two bags have fit-OK < 1 only because some frames carry no labelled markers.
