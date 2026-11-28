---
title: "jugglebot_launch gains a solver_rt_priority argument (default 0, off) so a Sitting can A/B the SCHED_FIFO solver worker with cpu_partition (FEAT-31)"
type: feature
date: 2026-11-29
status: complete
phase: FEAT-31
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tests/ros/test_stream_tuning.py
  - logbook/2026-11-29-solver-rt-priority-launch-argument.md
  - logbook/INDEX.md
subsystem: [skills, launch]
tags: [solver-worker, sched-fifo, cpu-partition, launch]
---

## What and why

Default-off plumbing for a Sitting A/B. `trajectory_node` already had a `solver_rt_priority` ROS parameter (default 0) that starts the CATCH-solve worker on SCHED_FIFO, but the launch file could not set it. `jugglebot_launch.py` now declares `solver_rt_priority` (default `'0'`) and passes it to `trajectory_node` as `PythonExpression(["int('", x, "')"])`, so the node receives the int 0 by default and behaviour is unchanged. Flown once on 2026-10-28 without a cpu partition, the FIFO worker stalled the setpoint stream 60-96 ms per CATCH solve and E-stopped the Hand on 3 of 5 goals; the 2026-11-28 owner decision is to re-try it together with BUG-58 `cpu_partition`, and the argument description says to fly it only with `cpu_partition:=true`.

## Finding: can the FIFO worker share a cpu with the stream processes?

Not with `cpu_partition:=true` and the pin applied. `trajectory_node` pins itself to `stream_cpus` (0-3, `stream_tuning.py` `DEFAULT_STREAM_CPUS`) and passes `solver_cpus` (4-5) to the worker together with `rt_priority` (`trajectory_node.py` 584-600, 1284-1291). In the worker `_main` (`solver_worker.py` 816) `apply_rt_policy(args.rt_prio, cpus or WORKER_RT_CPUS)` takes the partition cpus over `WORKER_RT_CPUS` (1-5, line 89), so the FIFO pin is 4-5 and the worker's affinity is set again to 4-5 (lines 817-822). The sets 0-3 and 4-5 are disjoint. Two caveats. Without `cpu_partition` the FIFO worker uses 1-5 and overlaps the unpinned stream processes, which is the 2026-10-28 configuration. With `cpu_partition:=true` but a failed pin (`trajectory_node.py` 599-600) `_solver_cpus` is reset to None and the worker again gets 1-5. The partition is fail-open, so the log line "cpu_partition ON" must be checked in the Sitting. Logic not changed.

## Verification

- 2026-11-29, `night_shift_nonet.sh pytest -q tests/ros/test_stream_tuning.py`: **13 passed in 9.09 s**; `tests/ros/test_launch_nodes.py tests/ros/test_launch_priority.py`: **19 passed in 2.13 s**.
- The new test `test_launch_exposes_solver_rt_priority_default_zero_to_trajectory_node` failed (`KeyError: 'solver_rt_priority'`) with the launch hunk stashed, and passes with it.
- Not run against hardware.
