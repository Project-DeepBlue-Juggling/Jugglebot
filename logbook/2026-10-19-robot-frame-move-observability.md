---
title: "Robot frame moves are announced with their size, an attempt is refused while the frame settles, the GUI model follows the robot frame (world-anchored scene), and the fit cap goes 1.5 -> 2.0 mm - the 14:24 sitting showed the move was tracked but silent"
type: feature
date: 2026-10-19
status: in-progress
phase: "two-ball-skill-stack - post-R6 (Ball Butler calibration), Phase 3 move observability"
related_plan: bb-calibration-one-attempt.md
related_entries:
  - 2026-10-19-robot-frame-fit-yaw-only-hold
  - 2026-10-19-bb-calibration-phase3-robot-frame
files_changed:
  - ros_ws/src/jugglebot/jugglebot/robot_frame.py
  - ros_ws/src/jugglebot/jugglebot/mocap_status.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/gui/js/robot-frame.js
  - ros_ws/gui/js/viewer.js
  - ros_ws/gui/js/stewart-model.js
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/css/panels.css
  - ros_ws/gui/js/replay/allowlist.js
  - ros_ws/gui/js/replay/policy.js
  - ros_ws/gui/replay/schema.py
  - ros_ws/docs/choreography.md
  - tests/ros/test_robot_frame.py
  - tests/ros/test_mocap_node_robot_frame.py
  - tests/ros/test_mocap_node.py
  - tests/ros/test_skill_node_robot_frame.py
  - tests/motion/test_skills_executor.py
  - tests/hardware/session_skill_stack_rehearsals.md
  - plans/active/bb-calibration-one-attempt.md
  - plans/active/INDEX.md
  - ros_ws/docs/bb_calibration_readings.md
  - logbook/2026-10-19-robot-frame-fit-yaw-only-hold.md
  - logbook/2026-10-19-robot-frame-move-observability.md
  - logbook/INDEX.md
subsystem:
  - tracking
  - ros
  - gui
tags:
  - safety
  - testing
  - docs
---

# Robot frame move observability

## Summary

The second-cut robot frame fit was FLOWN at 14:24 on 2026-10-19 and held: hops 8/8 caught across a 48.5 mm / 14.5 deg hand move, no refusals, never UNKNOWN. The move itself was silent, and the GUI's 3D model never moved. Code landed the same afternoon (NOT flown): `mocap_node` announces every settled move with its magnitude, `mocap/status` carries the settle/move keys, `skill_node` refuses an attempt while the frame is settling (`REJECTED_ROBOT_FRAME_SETTLING`), the GUI scene is world-anchored (the Stewart model follows `robot_frame`) with a three-state pill and a sticky move banner, and the fit's RMS cap is raised 1.5 -> 2.0 mm (reversing the morning's choice).

## Problem

Owner's two gaps after the 14:24 sitting: (1) the move was silent, (2) the GUI 3D model is static. The sitting (QTM not aligned to the base; BB feeds and hop throws worked; base moved by hand mid-session with a pause before the next throw) is the first flight of the second cut. The bag recorded neither `/robot_frame` nor `/mocap/status`, so the frame timeline below is a replay of the production `RobotFrameTracker` over `/mocap_data` (830 feeds at 5 Hz); every `skill_node` snapshot in the launch log matches the replay to 0.3 mm / 0.03 deg.

## Analysis

Source: bag `~/Desktop/rosbags/2026-10-19_14-24-15`, the launch log of the same launch, probe report `temp/probes/robot_frame_sitting_1424/report.md`.

Frame timeline (replay): lock 0.67 s after bag start ("Robot frame OK" 14:24:18.5): origin (23.84, 19.46, 0.87) mm, yaw -15.467 deg, 7 markers, rms 0.87 mm; pre-move jitter <= 0.5 mm / 0.03 deg per step. The hand move began ~14:26:24.0 (raw fit left the old pose) and ended ~0.8 s later; the running frame (window medoid) walked in four steps, (13.6, 20.6) yaw -12.48; (-0.8, 22.8) -8.33; (-13.7, 25.8) -4.12; (-23.7, 28.7) -0.85, settled at 14:26:25.9, 1.9 s after the move began. Move size 48.5 mm (dx -47.5, dy +9.5), +14.5 deg yaw. UNKNOWN: never (0 of 830 feeds; `mocap_data.aligned` flipped once, at the lock). Accepted-fit RMS p50 0.85 mm, max 1.491 mm against the 1.5 cap; 15 fits over 1.4 mm. Drop-worst fired on 195 of 826 accepted fits (Base - 5 187x, Base - 6 85x, Base - 3 5x, Base - 2 3x, Base - 4 1x); after the move Base - 5 and Base - 6 were dropped on almost every fit: 5 markers, rms 1.25-1.33 mm, ~0.2 mm under the cap.

| # | accept (AEDT) | type | outcome | frame snapshot |
|---|---|---|---|---|
| A1 | 14:25:29 | reload (BB feed) | COMPLETED (BB-feed catches are not logged) | yaw -15.425 |
| A2 | 14:25:54 | hop, 4 throws | 4/4 CAUGHT, worst landing 40 mm | yaw -15.416 |
| - | 14:26:24-25.9 | HAND MOVE | | |
| A3 | 14:26:40 | hop, 4 throws | 4/4 CAUGHT, worst landing 32 mm | origin (-23.7, 28.7), yaw -0.851 |
| A4 | 14:26:56 | reload (BB feed) | COMPLETED | yaw -0.785 |

No `REJECTED_ROBOT_FRAME_UNKNOWN` / `REJECTED_FRAME_OFFSET` or any `REJECTED_*` at accept (only the usual re-aim refusals from the trajectory cycle gate). No attempt was live during the move; A3 was accepted 14.2 s after the frame settled, so a move within 5 s of a throw was not tested. The BB aim used the new frame: world x -19.8 (A1) -> -70.7 (A4), about the 48 mm base shift; landing scatter after the move is the same size as before.

The observability gap as measured: `mocap_node` logged only OK/UNKNOWN transitions, so it logged nothing at the move. The one visible line was `ball_tracker`'s "landing plane" INFO (an accident of its > 0.5 mm z-change threshold) and it printed step 1 of the 4-step walk, (13.6, 20.6) yaw -12.48; the final frame was never logged at INFO. `aligned`, `robot_frame_ok`, rms, `n_matched`, `age_s` and `reason` all stayed "ok" through the move.

GUI frame map: the scene assumed world == base. The Stewart model group was added directly to `scene` at the scene origin (QTM world origin); markers, balls, trails, history, the BB, the cone and the rigid-body triads are all QTM-sourced and drawn in world; "Global Axes" sat at the scene origin and was read as the robot origin. So after any base move the markers moved and the model did not.

## Discussion

**1. Hypothesis reframed: a move is tracked, not cleared.** The morning's reading (entry `2026-10-19-robot-frame-fit-yaw-only-hold`) was that a real move might clear the frame (UNKNOWN, then a refusal). The 14:24 flight confirms the other half of that entry's measurement: every fit during a slide is accepted, so the running frame walks through mid-slide poses (four steps, up to ~85 mm off in the 11:13 bag) and is never UNKNOWN. The hazard is therefore not a refusal but a wrong frame silently used: the per-attempt snapshot (`_snapshot_robot_frame`) freezes whatever the running frame is at accept, for the whole attempt. That is why settling is an explicit refusal and why the move announcement waits for the settled frame (the first line after the move would otherwise be step 1 of 4, as `ball_tracker` showed).

**2. Tradeoff accepted: world-anchored GUI over robot-anchored.** Either the model moves to `robot_frame` in a fixed world scene, or the scene is re-expressed in the robot frame and everything else shifts. Anchoring to world is chosen because the QTM-sourced layers (markers, balls, trails, history, BB, cone, rigid-body triads) already arrive in world, and trails and history store raw positions: a robot-anchored scene would need every stored point re-transformed on every move (and a moved robot would corrupt the history already drawn). World-anchored needs one group transform on one object. The cost accepted: the "Global Axes" triad now sits at the QTM world origin, not the robot origin (renamed "World Axes (QTM)"), and the camera orbit target is unchanged, so after a large move the model can leave the view centre.

**3. Tradeoff accepted: cap 1.5 -> 2.0 mm reverses the morning.** The morning set 1.5 mm because only 1.5 still dropped a single 5 mm-displaced marker. The 14:24 sitting ran on 5 markers at rms 1.25-1.33 mm after the move, ~0.2 mm under the cap (max 1.491 mm), i.e. one more bias step from losing the lock in the middle of a session. A frame that locks on every launch outranks the sub-mm single-marker guard. The guard that is lost is small: at 2.0 a 5 mm displaced marker passes 7-wide with a 0.84 mm origin error, which is inside the frame's own scatter (origin p95 up to 1.45 mm); a 10 mm displacement is still dropped. Drop-worst, the 5-marker floor and `MIN_SPREAD` still carry the single-marker job.

**4. One threshold pair, 5 mm / 0.3 deg.** Used for both the settle spread and the move announcement. Measured scatter: origin p95 1.45 mm at home with the platform parked high, per-configuration bias steps ~1 mm, yaw p95 0.06 deg. A 2 mm threshold would false-trip during active juggling; 5 mm / 0.3 deg is 3.4x the origin scatter and 5x the yaw scatter. The owner expects occasional false-positive "moves" and wants every move announced with its magnitude; a false positive costs one refusal that clears itself and one WARN.

## Change

- `robot_frame.py`: `ROBOT_FRAME_MOVE_MM = 5.0`, `ROBOT_FRAME_MOVE_DEG = 0.3`; `ROBOT_FRAME_MAX_RMS_MM = 2.0` (comment cites the 14:24 sitting). `RobotFrameTracker.settled` (window full and every fit in it within 5 mm / 0.3 deg of the running frame), `pop_event()` returning `('settled', frame, None)` once at the initial lock and `('moved', frame, (d_mm, d_yaw_deg, from_frame))` once per settled move over threshold (at most one event per feed; `announced` survives a clear, so frames lost and restored at the same pose give no move), `move_count`, `last_move`; `status()` gains `settled`, `move_count`, `last_move_mm`, `last_move_deg`, `last_move_stamp_ns`.
- `mocap_node.py`: `_feed_robot_frame` polls `pop_event()`; INFO `Robot frame settled: origin (x, y, z) mm, yaw w deg`; WARN `Robot frame MOVED: 20.0 mm / 0.0 deg -> origin (x, y, z) mm, yaw w deg (was (x0, y0, z0) mm, w0 deg)`. One severity per call site (Foxy rclpy). OK/UNKNOWN lines unchanged.
- `mocap_status.py`: `mocap/status` keys `robot_frame_settled` ('1'/'0'; '0' with no template), `robot_frame_move_count`, `robot_frame_last_move_mm`, `robot_frame_last_move_deg`, `robot_frame_last_move_s` (the move feed's mocap stamp on the ROS clock via the QTM clock sync, not wall-clock now), `robot_frame_origin_mm` ('x,y,z', '' if no frame), `robot_frame_yaw_deg`. Existing keys and the `robot_frame` topic policy unchanged.
- `skill_node.py`: subscribes `mocap/status`; `_on_mocap_status` sets `_robot_frame_settled` False only on '0' (default True, absent key leaves it unchanged, a quiet topic keeps the last value). `_snapshot_robot_frame` (the accept path: goal accept, magazine, `skills/check`) returns the operator refusal after the `_attempt_running` early-return, so a mid-attempt `skills/check` is never refused. The per-attempt snapshot is otherwise unchanged. Operator text: `REJECTED_ROBOT_FRAME_SETTLING: Robot frame is settling (moved < 2 s ago or being moved) - wait a few seconds and retry; mocap_node logs 'Robot frame MOVED' with the size`; the `skills/check` ladder line carries only the bare code.
- `executor.py` and `INVARIANTS.md`: `REJECTED_ROBOT_FRAME_SETTLING` constant, `Observations.robot_frame_settled: bool = True`, ladder row right after ROBOT_FRAME_UNKNOWN with the same `skip_mocap` gating; matching INVARIANTS row.
- `jugglebot_launch.py`: the bag record list gains `/robot_frame` and `/mocap/status` (their absence is why the 14:24 timeline had to be replayed).
- GUI (`ros_ws/gui`): new `js/robot-frame.js` (`onRobotFrame`, `onMocapStatus`, the banner); `viewer.js` exports `robotFrameGroup` (child of `scene`) and renames "Global Axes" to "World Axes (QTM)"; `stewart-model.js` adds the model to `robotFrameGroup`; `main.js` subscribes `robot_frame` (PoseStamped) and `mocap/status` (DiagnosticStatus); `panels.js` makes the pill three-state (UNKNOWN red, SETTLING amber, OK green) with a secondary text `rms 0.87 mm · 7 mk · (23.8, 19.5) mm · -15.5°`; `css/panels.css` adds `.flag-icon.settling`, `.flag-detail`, `.robot-frame-banner`. Yaw sign: robot (x,y,z) -> Three (x,z,-y), quaternion (qx,qy,qz,qw) -> (qx,qz,-qy,qw) as in `mocap-markers.js`, so a +10 deg `robot_frame` yaw turns the model as a QTM Base body triad with +10 deg does. Banner appears when `robot_frame_move_count` increases and on the first status after a reload if count > 0; "now" origin is the settled pose at announcement and is not refreshed. The replay layer is a pinned mirror of the live subscribe set (`tests/ros/test_replay_allowlist.py`, `test_gui_replay_policy.py`, `test_replay_mcap_oracle.py`, `test_rosbridge_websocket_lean.py`): both topics were added to `GUI_SUBSCRIBED_TOPICS` (main.js), `ros_ws/gui/replay/schema.py` SUBSCRIBED, `js/replay/allowlist.js` and `js/replay/policy.js` (`/mocap/status` state-render, unthrottled; `/robot_frame` state-edge, `every`), and the two subscribe calls use named handlers because the throttle test parses a bare identifier. `ros_ws/docs/choreography.md` regenerated (the new skill_node subscription). Deploy: browser hard-refresh (static files are served `no-cache`); no `jugglebot-gui` restart.

## Control analysis

The new refusal sits in the accept-time ladder only: goal accept, magazine and `skills/check`. The per-attempt snapshot is unchanged (frozen at accept, the tracker plane follows the latched topic), and a mid-attempt `skills/check` is never refused, so a running attempt cannot be disturbed. The frame still feeds landings, BB aim points, the Platform offset and the tracker plane once per attempt; the 40 Hz planner and the leg path never see `settled`, the new status keys or the GUI, and no setpoint, feedforward or timing path changes. One accept-time walk: the operator sets the robot down; the window medoid reaches the final pose ~1.9 s after the slide (14:24 timing); `settled` needs a full 10-fit window all within 5 mm / 0.3 deg of the running frame, so it flips True ~2 s after rest (10 fits at 5 Hz); a `skills/check` before that returns the refusal with the other refusals at once; after it, the snapshot freezes a settled frame. A false-positive move costs one refusal that clears ~2 s after rest and one WARN. The pair 5 mm / 0.3 deg against the measured scatter (origin p95 1.45 mm, yaw p95 0.06 deg) means a bias step of ~1 mm cannot unsettle the window. The cap 2.0 mm widens what a single displaced marker may do to the origin to 0.84 mm for a 5 mm displacement (inside the frame's own scatter); a 1.5 -> 2.0 change does not alter the frozen-per-attempt contract.

## Verification

All dated 2026-10-19, PDJ venv.
- Tracker, node and status: `pytest tests/ros/test_robot_frame.py tests/ros/test_robot_frame_resource.py tests/ros/test_mocap_node_robot_frame.py tests/ros/test_mocap_node.py tests/ros/test_mocap_frame_queue.py tests/ros/test_perception_console_lines.py -q -n 4` -> 145 passed in 28 s (includes the 2.0 cap: `test_cap_is_two_mm_on_a_uniform_bias`, bias 1.85 passes and 2.45 fails). After the key swap in `skill_node`: `pytest tests/ros/test_skill_node_robot_frame.py -q -n 2` -> 13 passed in 3.40 s.
- Settling refusal: `pytest tests/motion/test_skills_executor.py tests/ros/test_skill_node_robot_frame.py tests/ros/test_skill_node.py -q -n 4` -> 622 passed, 1 failed in 300 s. The failure, `tests/ros/test_skill_node.py::test_a_slow_test_run_cannot_age_the_fixture_sample_past_the_accept_check`, is a sleep/timing test that failed under 4-worker load and passed on a scoped rerun (1 passed in 3.4 s); its path was not touched. The full gate below is the authority.
- GUI: `node --check` on every `ros_ws/gui/js/*.js` passes. **NOT browser-tested**: no visual check has been made of the model move, the pill, or the banner.
- Full gate: 2026-10-19 15:09 AEDT, `./run_tests.sh --full` (log `temp/logs/gate_robot_frame_obs_run4.log`): 6969 passed, 9 skipped in 312.76 s + serial 7 passed, exit 0. Build: `cd ros_ws && colcon build --packages-select jugglebot` 2026-10-19, exit 0.

## Residue / not done

- NOT flown. The GUI needs a visual check in a browser (hard-refresh): banner position against the connection chip and the replay banner (top: 84px), pill detail layout in the narrow flags grid, and the model's yaw direction against the Base body triad.
- Not exercised: a move within 5 s of a throw, and a move mid-attempt (the snapshot is frozen for the attempt by design; the refusal only guards accept).
- Out of scope oddity: A3 throw 1 logged TRACKER-IDENTITY-REFUSED (fitted landing 1.164 s off the schedule) yet was scored CAUGHT.
- Base - 5 and Base - 6 are dropped on nearly every post-move fit: the 3-5-6 cluster residue stands; only re-wanding QTM or camera coverage of that cluster lowers the floor.
- A quiet `mocap/status` whose last value was `robot_frame_settled=0` keeps refusing at accept until the topic returns (REJECTED_MOCAP_STALE co-fires and names the cause); no timeout was added.
- The banner's "now" pose is the settled pose at announcement, not a live readout.
- The "GUI 3D view still draws markers and bodies in QTM world" residue of the second-cut entry is RESOLVED here (the model now follows `robot_frame`).
