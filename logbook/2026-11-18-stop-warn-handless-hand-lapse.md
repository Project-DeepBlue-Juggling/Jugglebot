---
title: "STOP WARN names a Hand cover lapse under a plan with no Hand track instead of calling it clean-host (BUG-119)"
type: bugfix
date: 2026-11-18
status: complete
phase: "BUG-119 STOP WARN verdict"
subsystem: can-bridge
related_entries:
  - 2026-11-06-jb174-stream-socket-queue.md
  - 2026-11-18-sitting-debrief.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
---

# STOP WARN names a Hand cover lapse under a plan with no Hand track (BUG-119)

## What

Plans with no Hand track (`go_to_pose`, Level, the guard descents) send frames with `HAS_HAND` clear. The can-bridge then feeds only the legs, and the Hand group's cover runs out by design. `teensy_bridge_node`'s STOP WARN did not know which group stopped. On a clean host stream it called these "host forward cadence clean, no knot dropped at the drain, cover expired downstream or at a stamp", which reads as an unexplained firmware STOP.

On 2026-11-18, 43 of 44 such verdicts across five launches had `no_hand` > 0. Each was followed by a Hand STOP record and none by a legs record. On 2026-11-06 the count was 40 of 42. Debrief counts of "clean-host STOPs" for BUG-58 included them.

## Fix

The 10 Hz check now passes the heartbeat's per-group STOP record sequence numbers (`sr_seq`, FW 42) to `_warn_on_sched_stop`, which compares them with the previous check. A STOP gets the new verdict when all three hold:

- only the Hand slot changed;
- it was not a C1 stop;
- frames without a Hand arrived in the last three heartbeat windows (`_HANDLESS_WINDOW_HB`).

The new verdict is "Hand cover lapsed under a plan with no Hand track", with the frame count, ahead of the host-cadence verdicts.

In every other case, including a legs STOP and a heartbeat without `sr_seq`, the old verdicts stand. The line keeps its WARN level and `firmware scheduler STOP` prefix, so the debrief greps still find it. Advisory only: nothing gates on it, and no control path changes.

## Verification

- 2026-11-18, `python -m pytest -q tests/ros/test_teensy_bridge_node_bridge_diag.py`: **32 passed in 7.18 s**. Six new tests cover:
  - the Hand-only lapse;
  - a legs STOP;
  - no hand-less frames;
  - hand-less frames older than the window;
  - a C1 stop;
  - a missing `sr_seq`.
- 2026-11-18, `flock /tmp/jugglebot-tests.lock ./run_tests.sh -q` on this tree: **RESULT: PASS** (parallel 700 s rc=0, serial 33 s rc=0, total 733 s).
