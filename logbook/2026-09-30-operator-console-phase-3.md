---
title: "Operator console, phase 3: every other node condensed to one line per event, every operator action gets one outcome line, the Ctrl-C tracebacks addressed, runsheets on tee -i"
type: feature
date: 2026-09-30
status: in-progress
files_changed:
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/mocap_interface.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_correlation_node.py
  - ros_ws/src/jugglebot/jugglebot/spacemouse_handler.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/blas_threads.py
  - tools/probes/levelling_tilt_bag_check.py
  - tests/hardware/session_skills_r3.md
  - tests/hardware/session_skills_r4.md
  - tests/hardware/session_skills_r2_plan_gate.md
  - tests/hardware/session_skills_r3_apex_ladder.md
  - tests/hardware/session_kincal.md
  - tests/hardware/session_kincal_apply.md
  - tests/hardware/session_cup_contact.md
  - tests/hardware/session_hand_ball_sensor.md
  - tests/hardware/session_phase1_hold.md
  - tests/hardware/mvp_bench_runbook.md
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_unified7_cycle_ladder.md
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_teensy_bridge_node_console.py
  - tests/ros/test_teensy_bridge_node_read.py
  - tests/ros/test_teensy_bridge_node_version.py
  - tests/ros/test_perception_console_lines.py
  - tests/ros/test_ball_tracker_frame_stamp.py
  - tests/ros/test_trajectory_node_console.py
  - tests/ros/test_trajectory_tilt_map.py
  - tests/ros/test_skill_node.py
  - tests/motion/test_blas_threads.py
subsystem:
  - ros
  - tools
tags:
  - docs
---

# Operator console, phase 3

## Summary

Phases 1–2 (`2026-09-29-operator-console-phase-1`, `-phase-2`) fixed the screen format and the
per-throw lines. Phase 3 applies the same rules to every other node. Four Sonnet units did
the work, one per file group, from one brief. The brief's rules: logging only; one severity per
rclpy call site (`2026-09-29-skill-node-log-severity-crash`); fault, latch, E-STOP and FW-mismatch
lines keep their severity; and the node that receives an operator action reports its outcome
exactly once.

- **orchestrator:** a command reads as one line, `activate: IDLE -> ACTIVE` or `mode: STANDBY ->
  TRAJECTORY`, instead of three. A refused command is now a WARN saying why; it used to be
  silent. Levelling is one line: `levelled: tilt (…) rad -> gravity offset (…) rad, saved`.
  The "mocap gravity alignment check not yet implemented" stub WARN is now DEBUG. The Juggle relay
  lines are DEBUG, because skill_node owns them.
- **teensy_bridge:** startup OK-checks lose their paths, shas and per-axis version lists (all
  kept at DEBUG), and their greppable prefixes stay. Each of activate, deactivate, configure,
  homing and encoder search is now one line with its duration. Arming reads `setpoint output
  ARMED` (kept at WARN: the leg path goes live, and the yellow line is the cue) / `DISARMED`
  (INFO). Heartbeat gaps are one short INFO line: the line itself always said "diagnostic only, not
  a fault", and the long-episode `STILL stale` onset WARN is unchanged. `bb/throw OK` and `Level state persisted` are DEBUG
  (duplicates). `/recover`, `/clear_errors`, `/park_hand` and `/reboot_odrives` get one outcome
  line each.
- **perception and Ball Butler:** the Ball Butler calibration's 12 lines become one: `BB
  calibrated: pos (…) mm · axis tilt 0.62° · yaw offset +1.78° ±0.02° · swept 119°`. The
  tracker's `Ball N announced` (~650 a week) is DEBUG. A Ball Butler throw is one line with delay,
  flight, speed and aim correction. Throw refusals were WARN and are now ERROR. `Mocap base
  misaligned: pos=nan` now reads `Base body not visible to QTM`. Aim and accuracy-calibration
  refusals, previously silent, now log.
- **trajectory and skill startup:** `trajectory up: 40 Hz stream on :5557 · BLAS 1 thread ·
  pre/post-release hold …` and `skill_node ready · BLAS 1 thread`. The tilt-map line has no
  paths. `go_to_pose` and `go_home` successes now log, and `go_home` refusals are now ERROR.
  Stream on/off is DEBUG (the orchestrator reports modes), and `stream live: holding at …` is
  the stream-start line.
- **Shutdown:** skill, orchestrator, Ball Butler and bridge now destroy their action
  servers/clients before `destroy_node`. Foxy garbage-collects them after the node handle is
  gone, which was the `ActionServer.__del__` / `InvalidHandle` Ctrl-C traceback. This is
  untested on the robot: the mock has no real handles.
- **DEBUG is recorded:** orchestrator, bridge, mocap, tracker and Ball Butler now `set_level(DEBUG)`.
  Each unit checked its node has no sustained-rate `.debug(` call first.

## Discussion

**A runsheet check nearly became a false STOP.** The R2/R3/R4 sheets say `grep 'blas threads'`
the tee'd launch log, and "anything else: stop". The 1-thread confirmation is now DEBUG, so it
is in launch.log but not on screen. That grep would find nothing on a healthy launch, and the
operator would abort the sitting. The reusable sheets now grep `BLAS`, which matches `BLAS 1
thread` on both up lines, or the WARN `blas threads: N` for an uncapped pool. The same sweep
updated the `Command received:` checks (mvp bench, phase-1 hold, hand ball sensor), the per-axis
FW list check, `hand park complete`, and the levelling probe's pointer to `Gravity offset
published`. Superseded sheets (`session_unified7_cycle_ladder.md`,
`session_anomaly_fixes.md`) keep their text as history under a dated banner. The banner says
a missing old string is not an abort, because other sheets and probes still link to them. The lesson for any later demotion: grep
`tests/hardware/` and `tools/` for the old text, because runsheets read the SCREEN log.

**Every runsheet launch line is now `| tee -i`.** A plain `tee` dies on Ctrl-C. The owner's
first phase-2 launch showed no shutdown lines and no `bag saved:` line, while launch.log had
the whole clean shutdown (`2026-09-29-skill-node-log-severity-crash` § Notes).

## Verification

- Unit-scoped runs (2026-09-30):
  - orchestrator: `tests/ros/test_orchestrator_node.py`, 118 passed.
  - bridge: `tests/ros/test_teensy_bridge_node*.py` + `test_launch_console.py`, 493 passed.
  - perception: 15 files, 276 passed, 1 skipped.
  - trajectory/skill: 5 files, 336 passed.
- Gate, `./run_tests.sh` in the separate `console-phase2` worktree, rebased onto `0cb47745`,
  2026-09-30 (that worktree was removed later that day; its gate logs moved to the
  `Jugglebot-skills` worktree's `temp/logs`):
  - **Run 1 (ending 00:24): FAIL, 2 failed, 5665 passed, 9 skipped.** The failures were
    `test_teensy_bridge_node_udp_diag.py::test_udp_diag_counts_tx_by_type` (3 == 0 + 2) and
    `test_teensy_bridge_node_recover.py::test_recover_hand_park_is_a_noop_when_the_hand_is_already_parked`
    (`CLEAR_ERRORS: ERR_UNKNOWN_METHOD`). Both are UDP loopback tests against a fake Teensy. The
    first is a known flake: it failed the same way on 09-27 and 09-28 (`kincal_skipfix_gate_1335.log`,
    `run_tests_r4b_pre_sweep_20260928_1239.log`, in the main worktree's `temp/logs`), before this change. Both files then passed
    3/3 scoped (42 passed each time).
  - **Run 2 (ending 00:29): PASS, 5667 passed, 9 skipped; serial 3 passed**
    (`temp/logs/gate_console_phase3_run2_20260930.log`).
  - The co-failure in one run (an extra TX counted in one test, an unknown-method answer in the
    other) suggests the two tests' fake Teensies exchange packets under xdist. That is a test-
    isolation follow-up, not this change.
- **On the robot, 2026-09-30 09:16–09:35.** There were four launches, two with `record:=true`,
  and the owner confirmed the colour works over ssh. Ctrl-C printed no `Traceback`,
  `Exception ignored` or `InvalidHandle` line in any of the four `launch.log`s (the 09:40
  `~/.ros/log` dirs are the probe below, not the owner's). The owner's
  23:35 launch the night before, which had no fix, printed 24.
- **Why a record:=true Ctrl-C still showed nothing.** The owner reported no shutdown lines and
  no bag line, again. Both launch logs show a clean shutdown, and both bags have
  `metadata.yaml`. The two record runs were piped through a plain `| tee
  temp/logs/launch_logging_test_*.log`, and both copies end BEFORE launch's own Ctrl-C line.
  The two record:=false runs were not piped. Record was a confound for tee. A probe sent a real
  Ctrl-C keystroke through a pseudo-terminal to a minimal launch with the console and bag
  line armed. A bare launch printed `Ctrl-C: shutting down` and ended with `bag saved:`, and so
  did the same launch through `| tee -i`. Through a plain `| tee`, nothing printed after `^C`.
  A first attempt, as background jobs, proved nothing: a non-interactive shell starts a
  background job with SIGINT ignored.

## Open Questions

- Those launches covered the startup, activate, go_home, deactivate and Ctrl-C lines. No
  throw, Ball Butler, reload or levelling line has been seen live yet. The first throwing
  sitting is the check.
- A `/recover` or `/clear_errors` failure can still print two ERRORs: the inner cause and the
  outcome line.
- A Stop prints `attempt stopped` straight away, then the end line when the rest tail
  finishes. That is kept deliberately as the button's acknowledgement.
