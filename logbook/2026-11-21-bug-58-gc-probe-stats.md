---
title: "BUG-58: the gen-2 pauses walk the import-time heap, and gc_freeze does remove them (the 2026-11-18 reading is withdrawn); GC_PROBE now says how big each collection was"
type: investigation
date: 2026-11-21
status: complete
phase: "BUG-58 offline diagnosis"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/gc_probe.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tools/probes/sched_probe.py
  - tools/probes/gc_promotion_probe.py
  - tools/probes/README.md
  - tests/motion/test_gc_probe.py
  - tests/sim/test_sched_probe.py
  - logbook/2026-11-21-bug-58-gc-probe-stats.md
  - logbook/INDEX.md
subsystem: [stream, tooling]
---

## Summary

Offline work on BUG-58 (the 40 Hz setpoint stream starved past the firmware's 70 ms tolerance). Two results: a diagnosis of the 47 to 95 ms garbage-collection pauses in `teensy_bridge_node` and `trajectory_node`, and an extension of the opt-in 1 Hz `GC_PROBE` line. No stream timing, threshold, freeze or default changed.

## What the pauses are

- **Why they are long (measured).** A gen-2 pass walks every tracked object. Importing `teensy_bridge_node` alone gives 67.0k tracked objects and a 50 to 56 ms `gc.collect(2)`; `trajectory_node` gives 40.3k and 29 to 32 ms (this Jetson, Python 3.8.10, load 4 to 6). Imports are 96 % and 80 % of the heap the nodes report freezing. About 42 % of the bridge's heap comes from two imports the stream does not need: numpy-quaternion (pulls in scipy.interpolate) and python-can (pkg_resources, asyncio, sqlite3).
- **What triggers them.** Python 3.8 runs a full pass once the objects promoted into gen 2 exceed a quarter of gen 2: about 17k promotions in the bridge and 12.6k in `trajectory_node`. The bridge's first pass comes 2.7 to 5 min after launch in 7 of 7 launches without the freeze, then every 16 to 24 min. `trajectory_node` passes come only while juggling, about 73 s into an oval and then every 3 to 3.6 min.
- **The motion code is not the source.** The sim oval with planner, executor, installs and emitter in one process promoted 18 objects/s with no full pass in 217 s. The live cadence needs 60 to 170/s, so the rest is from what the sim lacks; the rclpy layer is the hypothesis and needs a live node to test.

## Discussion

**Withdrawn: "60 to 95 ms pauses also appear with gc_freeze on" (the BUG-58 Item's 2026-11-18 note, repeated in the 2026-11-20 debrief).** Re-reading the logs:

- In the three launches with the flag on (2026-11-13 launch 2; 2026-11-18 launches 3 and 3-1) the only pauses over 10 ms are the freeze's own startup `gc.collect()`, 59 to 84 ms, in `__init__` before arming. The probe is installed before the freeze, so its first line logs that collect as a gen-2 pause.
- The 95 ms pause on 2026-11-18 was in launch 4, which has no `gc_freeze ON` line.
- After startup the three frozen launches logged 0 collections of any generation over 51 min (6,053 `GC_PROBE` lines, all `n=0`). Launches without the flag logged 17 pauses of 47 to 95 ms in 179 min, so about 5 were expected; the chance of 0 by luck is about 0.7 %.

The 2026-11-20 Sitting flew with the flag off. What is still unflown is a 200-cycle oval with it on: the longest frozen oval so far was 45 s.

**Options, none taken here (the launch default is an owner decision):**

1. Freeze at startup and keep Python's standard thresholds. Later passes walk only objects made since the freeze; the estimate is 1 to 5 ms, not measured. Needs a `thresholds=None` option in `freeze_gc`.
2. Make today's `gc_freeze:=true` the default, with its raised thresholds (50000, 50, 1000). Measured: no collections at all. Cost: up to 50k uncollected cyclic objects, and a gen-0 pass of about 4 ms when one runs.
3. Re-freeze at each Attempt end, on top of 1 or 2.
4. Drop numpy-quaternion and python-can from the bridge's imports: about 42 % shorter pauses, not enough alone.

Suppressing gen 2 during Attempts without freezing is not recommended: the full walk would land at Idle while the firmware is still Armed and streaming.

## Probe change

`GC_PROBE` lines gain, appended after the existing fields so every parser still matches: `worst_collected`, `worst_uncollectable`, `gc_count`, `gc_colls`, `gc_frozen` and `gc_objects0` (the tracked-object count, taken once at install). A live `gc_objects` appears only with `JUGGLEBOT_GC_PROBE_OBJECTS_EVERY_S` above 0, default off, because `len(gc.get_objects())` measured 1.4 ms at 29k objects. The added cost is about 4 us per 1 Hz line. `tools/probes/sched_probe.py` prints the new fields and still reads old logs. `tools/probes/gc_promotion_probe.py` is the offline probe behind the numbers above (`population <module>`, `attempt`). The nodes' change needs a colcon rebuild.

## Verification

- 2026-11-21, `pytest tests/motion/test_gc_probe.py tests/sim/test_sched_probe.py tests/ros -k gc_probe tests/motion/test_stream_tuning.py -q` (the building agent's scoped set, as it reported): 36 passed in 9.11 s.
- 2026-11-21, `flock /tmp/jugglebot-tests.lock ./run_tests.sh` on the merge with main: parallel 10674 passed, 61 skipped, 1 xfailed, 4 warnings in 765.89s (0:12:45); serial 7 passed, 10746 deselected in 29.22s (rc=0 both phases).
