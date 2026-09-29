---
title: "Operator console, phase 1: clock time, short node names and colour in the launch shell; third-party chatter off the screen"
type: feature
date: 2026-09-29
status: in-progress
files_changed:
  - ros_ws/src/jugglebot/jugglebot/launch_console.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/launch/teensy_bridge_launch.py
  - tests/ros/test_launch_console.py
subsystem:
  - ros
tags:
  - docs
---

# Operator console, phase 1

## Summary

The owner found the launch shell opaque and information-sparse. A survey of the 31 tee'd shell logs
(09-13 → 09-29) and ~40 `launch.log`s: the 19:11 R4 sitting printed 1,906 lines for ~117 throws,
61 % skill_node and 22 % trajectory_node, ~14 lines per throw, 93 ERRORs in a sitting that met its
gate, and 93 of the first ~115 startup lines were rosbridge per-topic subscriptions, rosbag2's topic
list and launch's "process started" lines. Phase 1 changes only how lines are SHOWN; phases 2–3
(skill_node + trajectory_node message redesign, then the other nodes and a one-line-per-operator-
action audit) change what our nodes say. Owner choices: clock time, colour, one line per GUI
(dis)connect, every major event at least one line and most exactly one.

## Design

`jugglebot/launch_console.py` puts a formatter + filter on launch's ONE screen handler
(`launch.logging.launch_config.get_screen_handler()`); both launch files install it first thing
(never fatal; `JUGGLEBOT_CONSOLE=raw` opts out, `NO_COLOR` drops colour). The screen gets
`HH:MM:SS.mmm name  [WARN|ERROR] message` — the time from the node's own rcutils stamp, the name
short (`skill`, `trajectory`, `teensy`, ...), no `[skill_node-10]` prefix, yellow warnings, red
errors, DEBUG never shown. Third-party reductions: rosbridge → one line per GUI connect/disconnect
plus its errors (client UUIDs stripped); rosbag2's topic list and hidden-topics warning hidden;
launch's "process started" hidden, "finished cleanly" → "exited cleanly", "has died" →
"PROCESS DIED (killed by SIGINT)"; qtm_rt's INFO chatter inside mocap_node hidden. Also: launch now
prints `recording bag -> <dir>` / `NOT recording a bag` (the runsheets said "note the bag folder it
prints"; nothing printed it), and trajectory_node + teensy_bridge_node run `output='both'` — with
`'screen'` their lines never reached `~/.ros/log/<run>/launch.log`.

**The contract** (module docstring, pinned by a test): the console rewords or hides only
third-party output. Jugglebot's own messages are fixed at their `get_logger()` call, never here —
a presentation layer that rewrites our messages is a second place their meaning lives.

## Discussion

Three obvious knobs were rejected, each for a reason the code does not show:

- **`RCUTILS_CONSOLE_OUTPUT_FORMAT`.** Foxy's rcutils (1.1.5) has no wall-clock token (epoch
  `{time}` only), and rcl formats the per-process `~/.ros/log/python3_<pid>_*.log` files with the
  same format (`rcutils_logging_format_message`), so trimming it would strip the stamps from the
  record too.
- **`output_format='{line}'` per node.** Removes the prefix but nothing else, and costs raw lines
  (tracebacks) their only attribution; the screen hook strips the prefix itself and keeps the
  process name for raw lines.
- **Log levels to quiet a node.** A ROS 2 logger level gates the screen, the per-process file and
  `/rosout` in the bag together — demoting detail to DEBUG deletes it from the bag. A screen-only
  hook is the only way to get a quiet screen and a full record; it is also the mechanism phase 2
  will use (detail at DEBUG, those loggers set to DEBUG, the screen hiding DEBUG).

Accepted: launch prints its first two lines ("All log files can be found below ...", "Default
logging verbosity ...") before any launch file loads, so they stay stock. Colour codes land in
tee'd files (`less -R` reads them). `tools/probes/selftoss_landing_decomposition.py` already reads
`~/.ros/log/<run>/launch.log`, not the tee'd screen, so it is unaffected.

## Verification

- `tests/ros/test_launch_console.py` (28 tests, incl. one end to end through a real Foxy
  `LaunchService` in a subprocess: screen reformatted, `launch.log` raw) + the launch-file parser
  `tests/ros/test_choreography_map.py`, run 2026-09-29: 57/57 pass.
- Real `ros2 launch` CLI, clean env with the worktree install sourced, a demo launch file
  (2026-09-29 20:37): prefix gone, clock time, rosbridge subscription hidden, WARN/ERROR tagged.
- `ros2 launch jugglebot jugglebot_launch.py --show-args` after `colcon build --packages-select
  jugglebot` (2026-09-29): loads.
- Gate, `./run_tests.sh`, run 2026-09-29 20:40–20:44: **PASS, 5571 passed, 9 skipped (parallel);
  serial 3 passed** (`temp/logs/gate_console_phase1_20260929.log`). This entry and its INDEX row
  were written during that run, so the final tree was gated again, same command, 20:46–20:50:
  **PASS, 5571 passed, 9 skipped; serial 3 passed** (`temp/logs/gate_console_phase1_final_20260929.log`).
- Replay preview of the 19:11 sitting: `temp/logs/console_preview_skills_r4_20260929_1911.txt`
  (`less -R`).

## Open Questions

- Not yet seen on a full-stack launch; the next launch is the check.
- Phase 2: skill_node + trajectory_node (83 % of the lines). The Juggle action result counts
  throws/catches by substring-matching `OUTCOME` log lines (`skill_node.py` `_juggle_execute`), so
  outcomes must become data before their wording changes.
