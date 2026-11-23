---
title: "BUG-129 experiment: the launch removes stale Fast DDS shared-memory files before any node starts (shm_clean, default on), and bridge_shm:=false runs teensy_bridge_node on UDPv4 only (default off); the stall watchdog counts whether the freezes continue"
type: feature
date: 2026-11-23
status: complete
phase: "BUG-129 diagnosis and instrument"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/dds_shm.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot/resources/fastdds_bridge_udp_only.xml
  - tests/ros/test_dds_shm.py
subsystem: [can-bridge, ros]
tags: [experiment, stall, dds, shared-memory]
---

# BUG-129 experiment: clean Fast DDS shared memory at launch, optionally take the bridge off it

**What.** Two switches in `jugglebot_launch.py`. `shm_clean` (default `true`) runs
`jugglebot/dds_shm.py` once, after the arguments and before the first node, and logs one
INFO line: `shm clean: removed N stale Fast DDS files from /dev/shm (K kept in use by a live
process, ...) in T ms`. `bridge_shm` (default `true`, today's behaviour) set to `false` points
`FASTRTPS_DEFAULT_PROFILES_FILE` at `resources/fastdds_bridge_udp_only.xml` in
teensy_bridge_node's environment only, so the bridge runs on UDPv4 and every other node
keeps shared memory; the launch logs which state the bridge is in.

**Why.** All three stall-watchdog dumps show the `robot_state` publish blocked inside the
middleware with the GIL held, next to discovery events, with about 100 `fastrtps` files in
/dev/shm, many from dead launches. The cleaning is passive: no node changes at runtime.

**Not `fastdds shm clean`.** The Foxy copy (Fast DDS 2.1.4,
`/opt/ros/foxy/tools/fastdds/shm/clean.py`) raises AttributeError at the first in-use segment
(`self.segments_in_use` vs the mangled `__segments_in_use`; the `ros2 daemon` always holds
one), finds ports only through an `_el` file that stale ports lack, and probes liveness with
an flock on live lock files. `dds_shm.py` never opens a Fast DDS file: a group goes only if
no inode is in /proc/locks, no file is mapped or open by any process this user can read, it
is ours, and older than 30 s. A port the live daemon still maps is kept (unlinking it would
orphan the daemon's mapping). Bounded at 2 s; on timeout or unreadable /proc nothing goes.

**Switch off.** `shm_clean:=false`; the bridge part is off unless `bridge_shm:=false`.

**What it does NOT prove.** Stalls are at most one per launch and not reproducible on demand,
so a few clean launches are not a cure. While the daemon runs, its stale ports stay (dry run
2026-11-23: 50 of 108 files would go, 58 kept), so `shm_clean` alone barely tests a stale-port
mechanism; `bridge_shm:=false` is the decisive arm. Neither identifies the lock inside Fast DDS.

**Verification.**
- Probe 2026-11-23, Foxy, `ROS_DOMAIN_ID=87`, rclpy `std_msgs/String` pub/sub at 100 Hz: a
  publisher on the shipped profile mapped 0 /dev/shm files and a default subscriber received
  294 of 300; an empty `FASTRTPS_DEFAULT_PROFILES_FILE` logs `XMLPARSER Error ... filename
  empty` (hence the launch renames the variable rather than blanking it when `bridge_shm` is true).
- `python3 -m jugglebot.dds_shm` (dry run, system Python 3.8, 2026-11-23): `would remove 50 ...
  58 kept in use ... in 70 ms`; /dev/shm still 108 files.
- `pytest tests/ros/test_dds_shm.py tests/ros/test_launch_nodes.py tests/ros/test_bridge_launch_sigterm.py -q`
  (2026-11-23): **32 passed in 1.70 s**.
- `pytest tests/ros/test_launch_console.py tests/ros/test_launch_priority.py tests/ros/test_rosbridge_websocket_lean.py tests/ros/test_runtime_writes_untracked.py tests/ros/test_teensy_bridge_node_install_skew.py tests/ros/test_stream_tuning.py tests/motion/test_blas_threads.py -q`
  (2026-11-23): **130 passed in 10.00 s**.
- Not run: a real launch (needs `colcon build`; check the two INFO lines in the next launch log).

- 2026-11-23, `flock /tmp/jugglebot-tests.lock ./run_tests.sh -q` on this change over main at db95073e: first run FAIL on one test outside this change (`tests/ros/test_teensy_bridge_node_platform_imu.py::test_set_rate_relays_set_odr_and_reports_the_platforms_verdict`, a 0.3 s RPC timeout while an analysis job loaded the box; the file then passed 3 times scoped, 11 passed each); second run on a quiet box: RESULT: PASS (parallel rc=0 769 s, serial rc=0 33 s, total 802 s).
