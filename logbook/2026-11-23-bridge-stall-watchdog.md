---
title: "BUG-129: teensy_bridge_node gets a stall watchdog (faulthandler stacks of every thread, taken mid-stall, default on at launch); the bag history splits the 340 ms gaps into two real bridge freezes and four recorder-side stalls"
type: feature
date: 2026-11-23
status: in-progress
phase: "BUG-129 diagnosis and instrument"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/stall_watchdog.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - teensy_link/gap_monitor.py
  - tests/ros/test_stall_watchdog.py
  - tests/ros/test_teensy_bridge_node_stall_watchdog.py
  - tests/teensy_link/test_gap_monitor.py
subsystem: [can-bridge, ros]
tags: [diagnostic, stall, dds, faulthandler]
---

# BUG-129: a stall watchdog for teensy_bridge_node

**What.** `jugglebot/stall_watchdog.py` (pure Python) re-arms
`faulthandler.dump_traceback_later(0.15 s)` from its own 20 Hz thread. When the
whole process stops (the thread cannot re-arm) or the setpoint loop alone stops
(its tick is older than 50 ms, so the thread stops re-arming), the GIL-free C
timer writes every Python thread's stack while the stall is still going on.
After recovery the watchdog copies the dump to
`temp/logs/stall_teensy_bridge_node_<time>_<pid>_<n>.txt` (thread ids mapped to
names) and logs one WARN naming the file and the gap. Node parameters
`stall_watchdog` (default False, so tests arm nothing), `stall_watchdog_timeout_s`
(0.15), `stall_watchdog_dir` ('' = `<repo>/temp/logs`); launch argument
`stall_watchdog` default **true** (`stall_watchdog:=false` turns it off).
`GapTracker.last_unlocked()` lets the watchdog read the setpoint loop's last
tick without the tracker's lock. No guard limit, lane cover or timeout changed.

**Why.** The freeze happens at most once a launch and the bag cannot name the
thread or call that holds the process; only a stack taken during the stall can.

**Cost and safety.** One re-arm (cancel the old C thread, start a new one) is
44 us median on the Jetson under Python 3.8 (5 x 2000 calls, 40-49 us,
2026-11-23), so 20 Hz costs about 0.9 ms of CPU a second, all on the watchdog's
thread, none on the setpoint path. A 1 kHz loop A/B with and without the
watchdog showed the same p50/p99 within box noise. CPython's re-arm waits WITH
THE GIL for a dump in progress, so the dump goes to an unlinked tmpfs file in
`/dev/shm` (a memory copy); the copy to `temp/logs` uses plain Python file I/O,
which releases the GIL, so a slow disk delays only the watchdog thread. Reports
are capped at 20 a process. `stop()` (in `on_shutdown`, after the stow) cancels
the timer before closing the file, under the lock the re-arm takes.

## Discussion

The brief said four bridge freezes of 328-349 ms since 2026-11-18, all the same
event. The robot_state message carries the bridge's own timer stamp, and the
bag's log time is when the recorder received it. Comparing the two splits the
gaps into two classes with the same size:

| Event (bag log time) | Since launch | State | robot_state log gap | robot_state STAMP gap | Other processes' topics (mocap, trajectory, orchestrator) | Class |
|---|---|---|---|---|---|---|
| 2026-11-13 13:38:49.247 | 6.8 s | IDLE, disarmed; startup discovery, GUI rosbridge subscribe burst at 47.7-48.1, skills/nominate 49.02 | 347.5 ms | 340.1 ms | on time (mocap max 18 ms) | bridge freeze |
| 2026-11-13 13:39:20.357 | 38 s | IDLE, disarmed | 335.1 ms | 19.5 ms | all late ~340-420 ms (mocap's own /rosout stamped 20.503 arrived 20.696) | recorder-side |
| 2026-11-18 12:00:39.614 | 49 s | no trigger in log | 348.7 ms | 60.2 ms | all late (mocap 333 ms, trajectory 374, orchestrator 352) | recorder-side |
| 2026-11-20 12:50:15.587 | 50 s | disarmed; first CLI (set_upload_after_attempt) | 342.8 ms | 24.2 ms | all late (mocap 342, trajectory 530, orchestrator 358) | recorder-side |
| 2026-11-20 22:24:26.492 | 6 min | drives browning out (DC_BUS_UNDER_VOLTAGE 22:24:24) | 327.6 ms | 40.0 ms | all late (trajectory 384, orchestrator 383) | recorder-side |
| 2026-11-22 21:05:33.565 | 1 min 51 s | Armed, ACTIVE:TRAJECTORY hold; first CLI (platform_imu/upload) | 342.7 ms | 345.2 ms | on time (mocap max 16 ms); setpoint loop gap 333.7 ms | bridge freeze |

No other /robot_state gap over 150 ms in the 22 bags since 2026-11-13 (including 2026-11-22_21-48-38, the unpowered repro).

So "the bridge froze four times" is withdrawn: two events (11-13 13:38:49 and
11-22 21:05:33) are bridge freezes. In the other four the bridge's timer kept
stamping every 10 ms while every node's topics reached the recorder about 340 ms
late, so the recorder (also a Fast DDS participant) stalled, not the bridge;
those are harmless to the robot. The 11-20 12:50 "first CLI" case is one of
them, so the first-CLI trigger rests on the 11-22 event alone. In both freezes the last two robot_state messages before
the freeze were stamped 10 ms apart and delivered only when it ended: DDS
delivery from the bridge stopped about 20 ms BEFORE its Python threads did. A
kernel-level pause of the process would have delivered the first of them within
a millisecond. That ordering points at a lock inside the bridge's Fast DDS
participant that is held for about 340 ms (a discovery-time cost: the 11-13
freeze came 6.8 s after launch, during startup discovery and a GUI rosbridge
subscription burst; the 11-22 one with the first ros2 CLI participant), with a
Python thread then calling into rclpy (which in Foxy keeps the GIL) and blocking
on it. The recorder stalls have the same size, which fits the same Fast DDS
cost landing in a different participant. Not verified: the watchdog's dump will
show the Python frame that held the GIL. Full diagnosis: `temp/debrief/2026-11-22/stale.md` and this unit's
scratch write-up (BUG-129 body).

## Verification

- `flock /tmp/jugglebot-tests.lock pytest tests/ros -k "teensy_bridge or stall_watchdog" -q` (run 2026-11-23): **649 passed, 3679 deselected in 243.76 s** (every bridge test plus the two new files).
- `flock /tmp/jugglebot-tests.lock pytest tests/teensy_link/test_gap_monitor.py tests/ros/test_launch_nodes.py tests/ros/test_launch_console.py tests/ros/test_launch_priority.py tests/ros/test_stream_tuning.py tests/ros/test_rosbridge_websocket_lean.py tests/ros/test_choreography_map.py tests/ros/test_runtime_writes_untracked.py tests/motion/test_blas_threads.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py -q` (run 2026-11-23, the readers of gap_monitor, of the launch file and of this entry): **221 passed in 20.76 s**.
- Re-arm micro-benchmark (`measure_rearm_cost`, system python3 3.8, 2026-11-23): 40-49 us per call, median 44 us.
- Not verified on a launch: the WARN and the dump in a real `ros2 launch`; the cause of the freeze. The full gate was not run in this unit.
