---
title: "GUI takes 20 Hz gui/ copies of the mocap stream: rosbridge 102 % -> 62 % of a core on the replica (JB-76 load trim, option 1)"
type: optimization
date: 2026-10-28
status: resolved
phase: "JB-76 - SPLICE_TOO_LATE solve tail (load trim)"
related_plan:
  - two-ball-oval.md
related_entries:
  - 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented.md
  - 2026-09-14-rosbridge-cpu-leak-and-spin.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/panels.js
  - ros_ws/gui/js/ros-bridge.js
  - ros_ws/gui/css/panels.css
  - ros_ws/docs/choreography.md
  - tools/probes/rosbridge_cpu_probe.py
  - tools/probes/gui_synthetic_stack.py
  - tools/probes/gui_dom_probe.py
  - tools/probes/README.md
  - tests/ros/test_mocap_node_gui_copy.py
  - tests/ros/test_gui_mocap_copies.py
  - tests/ros/test_mocap_node.py
  - tests/ros/test_replay_allowlist.py
  - tests/ros/test_gui_replay_policy.py
subsystem:
  - gui
  - tracking
tags:
  - rosbridge
  - jetson-load
  - splice-too-late
---

# GUI takes 20 Hz copies of the mocap stream

## What

The GUI's rosbridge was the largest single CPU consumer in the 2026-10-28 Sitting (86 % of a core live, demand over 100 %), and the solver worker it starves is JB-76. An offline replica of the GUI's subscription set fed from that Sitting's bag reproduced the number (98.5 %) and located it: 59 % of the ~1515 messages/s entering rosbridge are the 300 Hz mocap stream on `/mocap_data`, `/rigid_body_poses` and `/bb/markers`, and rosbridge deserialises every message *before* it applies a client's throttle, so the GUI's 50 ms throttle saved nothing (report: `temp/probes/rosbridge_cpu_2026-10-28/report.md`).

Fix, owner-approved 2026-10-28 (option 1 of the surveyed nine):

- `mocap_node` publishes `gui/mocap_data` and `gui/rigid_body_poses`, the identical message objects, on every `gui_copy_divisor`-th frame (parameter, default 15: 300 Hz → 20 Hz, the rate the GUI showed before). One counter and one modulo per frame, nothing allocated. `mocap/status` gains the key `qtm_frame_hz` (frames since the last 5 Hz tick / dt).
- The GUI subscribes the two copies with throttle 0 and no longer spies the three 300 Hz topics; their topic-monitor rows stay and show `qtm_frame_hz`, labelled "via mocap/status" (reported by mocap_node, not observed), going to "--" after 3 s without a status. The two 100 Hz spies (`robot_state`, `hand_telemetry`) are untouched, per the 2026-09-14 owner requirement. In-browser MCAP replay maps the bag's originals onto the `gui/` handlers (`REPLAY_TOPIC_ALIASES` in `ros-bridge.js`), so old bags still render markers.
- `rosbridge_cpu_probe.py`'s synthetic census is refreshed to today's launch (the 300 Hz trio, `balls`, `leg_setpoint_echo`, the copies, `mocap/status`), with `--client gui` (today's main.js) and `--client gui-old` (the pre-change set) so the before/after is one probe; it sources the live checkout's install, not the retired `Jugglebot-skills` worktree.
- A throttle equal to the publish period is a defect: the replica received only 10 Hz with throttle 50 on the 20 Hz copy (jitter drops every other message). Live throttle on the copies is therefore 0, pinned by `tests/ros/test_gui_replay_policy.py`, while the replay policy keeps 50 ms for the bag's 300 Hz originals.

## Discussion

This reverses the 2026-09-14 rejection of source-decimated GUI copies. That rejection was about `teensy_bridge_node` copies of `robot_state` / `hand_telemetry`: the saving was about 4 points, the spin-thread fix already captured it, and the copies added work to the bridge node. None of those hold here: the copies come from `mocap_node`, the saving is about 40 points measured, and the cause (300 Hz mocap since 2026-10-10) did not exist then. The one 09-14 requirement the change touches, "the monitor is always open and must keep watching", is met by reporting the QTM rate from `mocap/status` rather than by having rosbridge take 900 messages/s to observe it. Option 4 (deserialise only after the throttle, in the lean server) stays available for another ~9–28 points.

## Verification

- Replica, 2026-10-28 11:10–11:17, launch down, box load 0.9–1.4, `python tools/probes/rosbridge_cpu_probe.py load --server lean --client {gui-old,gui,none} --seconds 30 --warm 8`, three interleaved rounds (`temp/probes/rosbridge_option1_20261028-111005/summary.md`):

  | client | server % of a core, 3 runs | mean |
  |---|---|---|
  | gui-old (pre-change set) | 102.7, 102.1, 101.1 | **102.0** |
  | gui, first run (client throttle 50 on the copies: received 10 Hz) | 52.9, 52.4, 54.0 | 53.1 |
  | **gui, as shipped (throttle 0: received 19.9 Hz)** | 62.3, 59.7, 62.9 | **61.6** |
  | none | 0.0, 0.1, 0.1 | 0.1 |

  The old figure is pegged at one core, so the ~40-point saving is a lower bound. The first `gui` run (found by the phase audit) still had throttle 50 on the copies and received 10 Hz; the as-shipped row was re-measured 2026-10-28 11:37–11:40 (`temp/probes/rosbridge_option1_remeasure_20261028-1137.log`) with the probe's `gui` client at throttle 0, three runs, load average 2.1–5.3 (a peer session's agents were active, so it may read a point or two high). The census is smaller than the 29-topic replica of the survey (no rosout/camera/cone spies), so the absolute 53 is below that replica's 61; the delta is the claim.
- GUI, 2026-10-28: `python3 tools/probes/gui_dom_probe.py --scenario mocap` 3/3 PASS and `--scenario scenario1` 17/17 PASS against the live :8081 GUI with headless Chromium (`temp/logs/gui_dom_mocap.log`).
- mocap, 2026-10-28: `pytest -q tests/ros/test_mocap*.py` 157 passed (6 new in `test_mocap_node_gui_copy.py`: every 15th frame, identical object, divisor honoured, no `bb/markers` copy, `qtm_frame_hz` 300.0 for 60 frames in 0.2 s and 0.0 on the first tick).
- Replay/GUI tests, 2026-10-28: `pytest -q tests/ros/test_gui_replay_policy.py tests/ros/test_replay_allowlist.py` 7 passed; `tests/ros/test_choreography_map.py` 29 passed after `python tools/gen_choreography_map.py`.
- Gate: the `./run_tests.sh` triple is in the commit message.
- The topic monitor shows two rows per stream: `mocap_data` / `rigid_body_poses` reported from status, and `gui/mocap_data` / `gui/rigid_body_poses` observed at ~20 Hz (a row can be hidden by clicking it).
- Not verified live: no Sitting has run with the copies yet. The GUI shows no markers until `mocap_node` publishes them, so relaunch before (or with) refreshing the browser; a browser refresh is enough for the JS (the GUI service serves the checkout with `Cache-Control: no-cache`). Old bags replay markers through the alias but show "--" on the three reported rows (no `qtm_frame_hz` in them).

## Outcome

Expected live effect: rosbridge from ~86 % (demand ≥ 100 %) to ~60–65 % of a core, about 0.4 core returned to the box, on top of fix 1's `nice -n 10`. The next Sitting's `solve_starvation_scan.py` measures both together; JB-76 stays open until the CATCH wall/CPU ratio is read there.
