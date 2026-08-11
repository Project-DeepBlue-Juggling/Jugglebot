---
title: "P0 — the Teensy's executed leg command is echoed to ROS and bagged alongside /profile"
type: feature
date: 2026-08-11
status: resolved
phase: "bridge-temporal-trustworthiness P0"
related_plan: bridge-temporal-trustworthiness.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/docs/choreography.md
  - tests/ros/conftest.py
  - tests/ros/test_launch_nodes.py
  - tests/ros/test_teensy_bridge_node_leg_cmd.py
subsystem:
  - can
  - ros
tags:
  - performance
  - IPC
  - testing
  - docs
---

# P0 — LEG_CMD echo published, `/profile` bagged

`teensy_bridge_node` now subscribes the **existing** 100 Hz LEG_CMD uplink
(`MsgType` 0x88 — the Teensy's post-lead, post-stroke-clamp *executed* leg
command, stamped with the bridge wall clock) and republishes it as
`sensor_msgs/JointState` on **`/leg_cmd_executed`**: the RX-thread callback only
queues under the node lock (bounded 4000), an executor-side 100 Hz timer drains
and publishes. That split mirrors `bb/axis_estimates` and is the determinism
rule, not a style choice — no DDS work on the socket thread. The launch rosbag
record list gains `/leg_cmd_executed` **and** `/profile` (1 Hz firmware profile,
`udp_rtt_us` / `udp_jitter_us`).

**Why.** This closes two of the three telemetry gaps named by
`2026-07-18-teensy-uptime-tracking-degradation` — the third (recover-slew /
extrapolation occupancy) is firmware, phase P1. One degraded bag can now split
transport vs interp vs ODrive, and RTT-vs-uptime is bagged instead of
watched live. Landing this does **not** reset the bridge's aging (`colcon build`
+ relaunch only), which is why P0 precedes the S1 aged-bridge sitting.

**Two discoveries.** (a) `tests/ros/conftest.py` claimed `sensor_msgs` was
mocked; it never was, and the mocked `std_msgs` had no `Header`, so a real
`JointState` raised `ImportError` — `_publish_bb_axis_estimates` was **never
executable under the test harness** until now (`MockHeader` added, docstring
corrected). (b) `/leg_setpoint_echo` carries no header stamp (bare
`Float64MultiArray`), so bag joins against it lean on rosbag receive time —
exactly what a transport drift corrupts. Flagged for P3's monitor design,
deliberately **not** fixed in P0.

**Deployment.** Needs `colcon build --packages-select jugglebot` + relaunch, and
must be live **before** the S1 aging window starts.

## Verification

`tests/ros/` (`python -m pytest tests/ros/ -q`, run 2026-08-11): **1906 passed
in 198.67 s**. Gate (`./run_tests.sh`, run 2026-08-11): **4981 passed in 224 s
(parallel rc=0, serial phase empty), RESULT: PASS.** `ros_ws/docs/choreography.md` is regenerated, not hand-edited
(pinned by `test_choreography_map.py`); the record-list change is pinned by
`test_launch_nodes.py`. audit-reporter returned one NOTE — a test docstring
overclaimed the stamp check's mechanism (the header routes through the mocked
`std_msgs`, not real `builtin_interfaces`) — fixed before commit.
