---
title: "BUG-58 probe: opt-in GC pause timing in the stream nodes and a /proc scheduler sampler, to tell CPU preemption from a whole-process pause"
type: feature
date: 2026-11-03
status: complete
phase: ""
files_changed:
  - ros_ws/src/jugglebot/jugglebot/gc_probe.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tools/probes/sched_probe.py
  - tools/probes/README.md
  - tests/motion/test_gc_probe.py
  - tests/sim/test_sched_probe.py
  - tests/ros/test_trajectory_node_rt_param.py
  - tests/ros/test_teensy_bridge_node_gc_probe.py
  - logbook/INDEX.md
subsystem: tools
---

# BUG-58 probe (TKT-32): GC pause vs CPU preemption

**What.** Every forward gap over the firmware's 70 ms setpoint cover ended a goal in
LANE_REFUSED (3 of 3, 2026-11-02), each inside a burst of CATCH solves whose wall time far
exceeded CPU time. The logs cannot separate CPU preemption from a whole-process Python pause
(GC or GIL). Owner choice (TKT-32): probe first, no behaviour change.

- `jugglebot/gc_probe.py` (pure): `gc.callbacks` hook recording generation and pause per
  collection into a bounded deque; `summary()` drains it.
- `trajectory_node` and `teensy_bridge_node`: boolean parameter `gc_probe` (default False),
  launch argument `gc_probe:=true`. On: hook installed at startup and a dedicated 1 Hz timer
  logs `GC_PROBE <node> n= max_ms= total_ms= worst_gen= worst_wall=` at INFO, plus a WARN at a
  separate call site when a single pause exceeded 10 ms (Foxy: one severity per call site).
  A dedicated timer rather than an existing one so the OFF path has no new code at all.
- `tools/probes/sched_probe.py`: `record` (20 Hz /proc sampler of the launch's python processes
  and children) and `report` (per CATCH solve window: involuntary-switch and CPU deltas per
  process, worst GC_PROBE line).

**Control-system walk-through (nothing changes on the hot path).**
- OFF (default): `gc_probe` is declared and read once in `__init__`; no hook, no timer, no
  deque. One 40 Hz `trajectory_node` cycle and one `teensy_bridge_node` forwarder tick execute
  the same instructions as before.
- ON: a 40 Hz cycle / forwarder tick adds nothing of its own. The only added work is the
  `gc.callbacks` function, which fires when CPython runs a collection (in whichever thread
  allocates): two `perf_counter()` calls and one `deque.append`, no logging, no I/O, no lock.
  The drain and the log line run on a separate 1 Hz timer callback, never inside the 40 Hz
  or 500 Hz paths. Residual effect: the 1 Hz callback occupies an executor thread for
  microseconds, and the probe's own callback adds well under 10 us per collection; neither
  can move the setpoint stream by anything near 70 ms.

**Verification.** (2026-11-03, `pytest -q -p no:cacheprovider tests/motion/test_gc_probe.py
tests/sim/test_sched_probe.py tests/ros/ tests/sim/test_logbook_front_matter.py
tests/sim/test_docs_links.py`): result recorded in the commit message. `sched_probe.py record
--duration 1` ran live against the running skill_node and wrote a CSV with children found.
Not flown: the Sitting run (one long Columns goal, `gc_probe:=true`, sampler beside it).
