---
title: "JB-149 opt-in stream cures, both default off: gc_freeze:=true and cpu_partition:=true"
type: feature
date: 2026-11-03
status: complete
phase: "pattern progression"
related_plan: pattern-progression.md
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/stream_tuning.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/solver_worker.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tests/ros/test_stream_tuning.py
  - tests/ros/test_teensy_bridge_node_bridge_diag.py
  - map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
  - logbook/2026-11-03-jb149-opt-in-cures.md
  - logbook/INDEX.md
subsystem: [ros]
tags: [performance, safety]
---

# JB-149 opt-in stream cures (default off)

## What

The owner asked for two cure flags, both default off (2026-11-02). They are built so the probe Sitting's verdict can be acted on at the next launch without a code change.

- **`gc_freeze:=true`** (`stream_tuning.freeze_gc`, run once at the end of `__init__` in `trajectory_node` and `teensy_bridge_node`):
  - Runs `gc.collect()` then `gc.freeze()`, so every startup object is skipped by later collections.
  - Sets the thresholds to (50000, 50, 1000).
  - Never calls `gc.disable()`, so a run-time cycle stays collectable.
- **`cpu_partition:=true`**:
  - Both stream nodes pin themselves to cpus 0-3 as the first thing in `__init__`, so every later thread inherits it.
  - The CATCH-solve worker is spawned with `--cpus 4,5`.
  - A failed pin logs a warning and fails open.
- **The GC test fix.** `test_rearm_resets_the_gap_trackers...` now runs with GC off. A gen2 pause in the test process (130-190 ms measured on the loaded Jetson) had tripped its 100 ms bound.

## What was dropped, and why

The 2026-11-02 working tree also carried a stall probe: a `gc.callbacks` hook plus a /proc sampler, classifying each stream gap as GC, run-queue or GIL. It was not landed. The opt-in probe that jugglebot-50 landed the same day covers the same question, and the JB-149 Sitting row is written around it and FW 38's counters:
- `gc_probe:=true` (`gc_probe.py`);
- `tools/probes/sched_probe.py`;
- logbook `2026-11-03-jb149-gc-sched-probe`.

Two GC hooks in the same nodes would have confused the record. The owner chose cures only (2026-11-03).

## How the flags are meant to be used

The JB-149 item's options are unchanged.
- Fly the probe row first.
- **Class 1** (GC pauses over 10 ms, or run-queue delay around the solves): A/B `gc_freeze:=true` or `cpu_partition:=true` at the next Sitting.
- **Class 2** (a firmware-side cover expiry with a clean host stream): neither flag helps.

**Accepted risk of `cpu_partition`.** The worker's two cores are not exclusive: the unpinned nodes (mocap, tracker, skill_node, rosbag) still use all six. Read the CATCH-solve wall time against the re-aim cliff when it is on.

## Verification

- 2026-11-03, `pytest -q tests/ros/test_stream_tuning.py tests/ros/test_teensy_bridge_node_bridge_diag.py tests/motion/test_gc_probe.py tests/ros/test_teensy_bridge_node_gc_probe.py tests/ros/test_trajectory_node_rt_param.py`: 41 passed. The launch test was updated to the `PythonExpression` bool idiom of `gc_probe`.
- 2026-11-03 21:38, `./run_tests.sh` on a quiet box: **PASS**, 8143 passed and 52 skipped (parallel 405 s), serial 5 passed.
- An earlier run at 21:20 failed 4 tests: three in `test_skill_node.py` and `test_reload_does_not_reframe_a_live_plan`. It overlapped b1's P4 box sweep and a peer's gate, with a 15-minute load average of about 18 and the parallel phase at 1331 s. All four were timing symptoms (a mocap sample 1.20 s stale, a mode-change abort, a mocked client call that never arrived). The same two files passed when re-run on their own once the load had dropped (`pytest -q tests/ros/test_skill_node.py tests/ros/test_trajectory_tilt_map.py`, 2026-11-03 21:20: 430 passed).
