---
title: "BUG-57 diagnostic: an opt-in, temporary full-rate Hand capture to CSV (`hand_capture:=true`), every telemetry, diagnostic and command-echo sample with its own stamp"
type: feature
date: 2026-11-02
status: complete
phase: "BUG-57 diagnosis"
subsystem: teensy_link
related_entries:
  - 2026-11-02-hand-release-step.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/hand_capture.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tests/ros/test_hand_capture.py
  - tools/probes/hand_capture_view.py
  - tools/probes/README.md
  - logbook/INDEX.md
---

# BUG-57: a temporary full-rate Hand capture

**Why.**
- BUG-57's overshoot (one launch at 168 rev/s against ~151 planned, Apex
  1.35) could not be explained from `/hand_telemetry`.
- That topic is a 100 Hz timer that resamples the latest cached values, all
  stamped with Jetson time.
- Owner, 2026-11-02: a temporary mechanism used only for this diagnosis is fine.

**What.**
- Launching with `hand_capture:=true` sets the bridge parameter
  `hand_capture_dir` to `temp/hand_capture/`.
- Every Hand sample the bridge receives becomes one CSV row:
  - axis-6 telemetry (pos, vel; Teensy stamp `t_teensy_us`);
  - the axis-6 ODrive diagnostic (iq setpoint and measured, axis state, errors;
    on change or a 1 Hz heartbeat; Jetson receive time only);
  - the sniffed Hand command echo (pos, vel_ff, torque_ff; bridge CAN stamp).
- Every row also carries the Jetson monotonic and wall time; align on
  `t_jetson_mono_s`.
- Writes go through a bounded queue (20000 rows) drained by a daemon writer
  thread. `offer()` is a `put_nowait` that drops and counts on overflow, so the
  receive path never blocks; with capture off the cost is one `None` test.
- `tools/probes/hand_capture_view.py` prints the merged table around the
  fastest Hand moment (or a window) and can save a PNG.

**Removal.** Every block is marked `TEMPORARY (BUG-57 diagnostic capture)`.
Delete them, plus `hand_capture.py`, its test, the viewer and its README row,
once BUG-57 is diagnosed.

**Verification.**
- 2026-11-02, `python -m pytest tests/ros tests/motion/test_hand_jam.py -q -n 3
  -p no:cacheprovider` (the agent's run): 3597 passed, 2 skipped.
- Default gate (2026-11-02, `./run_tests.sh`): 7993 passed + 4 serial; two
  failures, neither from this change. `test_item_bodies_are_readable` fails on
  main itself (another session's map edit). `test_per_cycle_planning_budget`
  was a load flake (gate 911 s vs ~430 s with three agents testing); re-run
  alone it gave 2 passed.

**Update 2026-11-02 late: ON by default.** The owner asked for the capture to
run by default until it has served BUG-57: the launch argument `hand_capture`
now defaults to `true`. Flip it back to `false`, or remove the whole TEMPORARY
set, when BUG-57 closes. Gate (2026-11-02, `./run_tests.sh`): 8018 passed, 54
skipped + 5 serial, RESULT PASS.
