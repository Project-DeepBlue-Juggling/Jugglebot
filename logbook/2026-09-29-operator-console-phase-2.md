---
title: "Operator console, phase 2: an attempt reads as one start line, one line per throw (with release and catch timing) and one end line; the detail is recorded at DEBUG"
type: feature
date: 2026-09-29
status: in-progress
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/launch_console.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot_interfaces/action/Juggle.action
  - tests/motion/test_skills_report.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_trajectory_node_console.py
  - tests/ros/test_launch_console.py
  - tests/ros/conftest.py
subsystem:
  - motion
  - ros
tags:
  - docs
---

# Operator console, phase 2

## Summary

Phase 1 (`2026-09-29-operator-console-phase-1`) changed how lines look; this changes what
skill_node and trajectory_node say. They made 83 % of the 19:11 sitting's 1,906 lines, about 14
per throw, and 93 ERRORs in a sitting that met its gate. Now an attempt reads:

```
self_toss started: 5 throws at apex 0.90 m · memory 4 rows · mocap frame offset 1.5 mm
throw 1/5 CAUGHT  apex 0.896 m  landed -4, +2 mm  release +12 ms  arrival -3 ms  seat +0.19 s  re-aimed 1x
throw 2/5 MISSED  apex 0.929 m  landed +31, -3 mm  ...                    (yellow)
self_toss done: 4/5 caught · apex 0.90–0.93 m · worst landing 31 mm · release -12 ms..+30 ms
```

and every detail line the executor and planner used to print at INFO (dispatches, splices,
catch aims, re-aim decisions, OUTCOME rows, memory rows, cycle seeds, the 30 s clock-offset
refresh, install refusals) is unchanged in text but logged at DEBUG. That keeps it in
`launch.log`, the per-process log and `/rosout`, and off the screen. An install refusal is
no longer an ERROR twice. A refused re-aim, which the catch survives, is part of its throw
line. A refusal that ends the attempt is the end line, one ERROR. Also: the bag folder is now the
launch shell's LAST line (owner ask), printed after every process has exited, with its size.

## Design

- `motion/skills/report.py` (pure): `ThrowReport`, and the start / throw / end line formatters,
  including `short_refusal` ("LIMIT_JERK: leg jerk 258k > 150k mm/s³").
- The executor appends one `ThrowReport` per finalised release, row or no row. It numbers
  releases (`throw 3/5`), records each catch's final aim and its accepted or refused re-aims
  on the flight it meets, and states why an attempt ended (`end_message`, `end_kind`). None
  of this is read by dispatch, re-aim or the learner.
- skill_node logs tick lines at DEBUG, drains reports into throw lines, and logs the end line
  when the executor retires. When `_end_attempt` hits a live executor, the retirement line
  carries the reason instead of a second ERROR. The Juggle result counts throws and catches
  from reports; it used to substring-match `'caught=True'` in log text. `per_throw` is each
  throw's line.
- Both nodes `set_level(DEBUG)`. trajectory_node's clock-offset refresh WARNs at ≥ 100 µs:
  320 refreshes over 09-13..09-29 had median 0.0, p99 0.2 and max 3.3 µs.

## Discussion

**Release timing: the fitted parabola traced back to the release height, not a same-height
model. A hypothesis reversed mid-unit.** The first draft did the trace-back. It was then
"corrected" to the schedule's same-height form (`t_land_obs − flight_s(apex_obs)`), reasoning
that `flight_s` is what the schedule times flights with. That was wrong, and the error was caught
before commit. The planner commands the REAL flight: release at the 860 mm throw height, land on
the 830 mm catch plane `flight_s` later. The robot's own log shows it: a 0.9 m skill announces
`|v| 4.166 m/s`, which is exactly the 860→830 launch (4166.3 mm/s). A same-height flight would
launch at 4201.8 mm/s. That flight arrives with a fitted apex of 0.915 m, and the same-height
metric reads a perfectly timed release as −7 ms. The trace-back reads it as 0. Pinned by
`test_a_throw_released_on_time_reads_zero_release_error`, which rebuilds that 4.166 m/s flight.
The input is the tracker's converged fit: a 1 % error in the fitted arrival speed moves the
release about 9 ms. **Arrival** is measured against the catch's final aim, not the schedule:
it asks whether the catch was timed right. **Seat** keeps its OUTCOME meaning (vs the scheduled
landing), because the owner already reads it that way.

**Why DEBUG, and why on the node's own logger.** A ROS 2 logger level gates the screen, the file
and `/rosout` together, so the console's screen-only DEBUG filter (phase 1) is what gives a
quiet screen with a full record. Foxy has no per-logger `--log-level` (probed:
`probe_node:=debug` fails to parse). A child logger at DEBUG would work, but Foxy's rosout
handler publishes only loggers it has a publisher for, so the detail would silently leave the
bag. So each node sets its own logger to DEBUG. The cost is rclpy's action-server DEBUG lines,
a few per goal, in the record only.

## Verification

- Scoped, 2026-09-29, in the `console-phase2` worktree:
  `pytest tests/motion/test_skills_report.py tests/motion/test_skills_executor.py
  tests/ros/test_skill_node.py tests/ros/test_trajectory_node_console.py
  tests/ros/test_launch_console.py -q`: all pass (16 + 139 + 148 + 5 + 31).
- Sim preview, 2026-09-29 22:58, after the metric fix. The real executor ran under
  `sim/skills_gate.py --learn --seeds 1` with a one-off probe printing the new lines. It gave
  23/23 throw lines, e.g. `throw 3/23 CAUGHT  apex 0.910 m  landed +0, +7 mm  release +18 ms
  arrival +0 ms  seat +0.06 s  re-aimed 2x`. The sim's release reads -1..+18 ms, about +7 on
  average. The sim releases the ball at the first loop tick at or after the commanded instant,
  from wherever its cup physically is, so it is not the ideal flight the zero-test pins. That
  spread was not investigated further; the first hardware sitting is the check.
- Gate, `./run_tests.sh` in the `console-phase2` worktree, run 2026-09-29 23:03–23:07: **PASS,
  5609 passed, 9 skipped; serial 3 passed**. Rebased onto `723ed66d` (the parallel session's
  stop-after-a-drop; only `logbook/INDEX.md` conflicted, `_on_tick` merged cleanly: its stop call
  stays outside the tick lock, reports drain every tick, the end line prints at retirement).
  The rebased tree was gated again with the same command, ending 23:22: **PASS, 5618 passed, 9 skipped
  (+9 = that commit's tests); serial 3 passed** (`temp/logs/gate_console_phase2_rebased_20260929.log`
  in the console worktree).

## Open Questions

- First sitting is the check. Watch that `release` agrees with what you see, and that the end
  line's tally matches the Juggle result.
- Phase 3: the other nodes (orchestrator's three lines per transition, the Ball Butler throw's
  four lines, startup banners, the 12-line BB calibration result, the Ctrl-C tracebacks) and a
  one-line-per-operator-action audit.
- The R4 runsheets say to read OUTCOME fields live. The throw line carries the same numbers.
  R5 runsheets should name the throw line.
