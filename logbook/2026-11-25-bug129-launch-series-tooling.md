---
title: "BUG-129: the unpowered launch-series runner and parser, and a synthetic discovery-churn reproduction that finds nothing on a quiet idle node"
type: feature
date: 2026-11-25
status: in-progress
phase: "BUG-129 diagnosis and instrument"
related_plan: ""
related_issues: []
files_changed:
  - tools/probes/bug129_launch_series.sh
  - tools/probes/bug129_series_report.py
  - tools/probes/bug129_synthetic_churn.py
  - tools/probes/README.md
  - tests/sim/test_bug129_series_report.py
  - logbook/2026-11-25-bug129-launch-series-tooling.md
subsystem: [can-bridge, ros, tooling]
tags: [diagnostic, dds, probe]
---

## Summary

Tooling for BUG-129 (the roughly 340 ms `/robot_state` publish freeze in `teensy_bridge_node`). Nothing under `ros_ws/` changed, so the next robot launch behaves exactly as before.

- `tools/probes/bug129_launch_series.sh` runs the spec'd unpowered series: arm A default, arm B `bridge_shm:=false`, blocks A B B A, `auto_arm:=false`, motor supply off. It does the 60 s manipulation check, the t=90 s discovery churn, a bounded SIGINT shutdown, one `flock` per launch, a load check, a required `--stop-at` that can never run into 01:30-04:00, and an abort when HEAD or the installed node files change. It refuses to start while `live_session` is LIVE, and aborts on an arm or motion line in the launch's own log. It never uses `kill -STOP`. It asserts at t=30 s of the first launch that `live_session` reports LIVE (the first meeting of the BUG-85 argv guard with a real launch). `--dry-run` and `--launches-per-arm N` exist.
- `tools/probes/bug129_series_report.py` turns logs, meta files, stall dumps and bags into `temp/reports/bug129_series_<date>.md`: validity, bridge-side vs recorder-side gaps, P/K/O dump class, the asymmetric verdict, Mann-Whitney on the secondary measures.
- `tools/probes/bug129_synthetic_churn.py` is the 45-minute-boxed synthetic reproduction.

## Process shapes (live_session meets a real launch)

All 15 console scripts under `ros_ws/install/jugglebot/lib/jugglebot/` have the shebang `#!/usr/bin/python3`, so a launched node's argv is `/usr/bin/python3 <install>/lib/jugglebot/<node> --ros-args ...` and `ros2 launch` is `/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch jugglebot jugglebot_launch.py ...`. `_live_argv_is_robot` matches both through argv[1]. Checked with sentinel scripts of exactly those shapes (2026-11-25): both matched and `live_session` reported LIVE. The real-launch check is in the runner.

## Synthetic reproduction

Domain 142, localhost only, one bare rclpy node at 100 Hz on a 4-thread `MultiThreadedExecutor`, about 215 churn cycles per arm (a 1200 s cap per arm cut the planned 300), the echo killed alternately with SIGKILL and SIGINT, an extra pub+sub SIGKILLed every 20 cycles, the daemon restarted every 50. Load average was about 5 during both arms (it was 18 for the 30 minutes before; the run waited).

| arm | cycles | publishes | max publish() | hits over 100 ms |
|---|---|---|---|---|
| default transports | 218 | 119,575 | 15.8 ms | 0 |
| UDP-only profile | 216 | 119,187 | 20.4 ms | 0 |

Source: `$S/out/bug129-series/run1/summary.json` (scratch, not committed). A 2-cycle smoke run at load 18 DID record one 315 ms publish() about 0.17 s after a SIGINT-killed echo: not a result (load confound, no control), but it shows the node can freeze under load. **A null result on a quiet, idle node is uninformative about the real stack**: no bridge threads, GIL load, IMU traffic or bag subscriber. It does not clear shm.

## Verification

- 2026-11-25, `python -m pytest tests/sim/test_bug129_series_report.py -q`: 11 passed. Mutations, each made once and restored: classifying every log gap as bridge-side, disabling the B-refutes rule, and breaking the K-dump match each made the matching test fail; adding a real `kill -STOP` line to the runner fails the dry-run test.
- 2026-11-25, `bash tools/probes/bug129_launch_series.sh --dry-run --stop-at 07:00 --launches-per-arm 3`: plan printed, exit 0; `--stop-at 02:30` refused, exit 64.
- 2026-11-25, `python tools/probes/bug129_synthetic_churn.py --out ... --cycles 300 --budget-s 1200`: as in the table. Afterwards no process or ros2 daemon remained, `live_session` reported clear, and the run's `live_session` check read clear while it ran.
- The real series was NOT run.

## Manipulation check counted libraries, not shared memory (2026-11-25, third attempt)

Arm B read `fastrtps_maps=64` for the bridge although `bridge_shm:=false` was in force. Cause: the check did
`grep -c fastrtps /proc/<pid>/maps`, which also counts the 16 `libfastrtps` / `*_typesupport_fastrtps_c.so`
libraries every rclpy process maps (4 maps each). Probe launches 2026-11-25 (arm B, then A, `auto_arm:=false`):
the B bridge has `FASTRTPS_DEFAULT_PROFILES_FILE=...fastdds_bridge_udp_only.xml` in its environment and maps
0 `/dev/shm/fastrtps_*` files while trajectory_node maps 11 `fastrtps_<hex>` + 22 `fastrtps_port<N>`; the A
bridge (profile path in the inert `JUGGLEBOT_BRIDGE_UDP_PROFILE_UNUSED`) maps 12 + 24. So the switch works and the
check was wrong. The runner now counts only `/dev/shm/fastrtps_` maps (meta key name unchanged).
Verification: 2026-11-25, `bash -n tools/probes/bug129_launch_series.sh` clean; `pytest tests/sim/test_bug129_series_report.py -q`: 11 passed.

## HEAD check scoped to code, and --first-slot (2026-11-25, series run)

The series aborted after 8 launches (09:13) because the auto-pull timer fast-forwarded main by four `map:` commits
(docs and map/ files only; installed bridge files unchanged). The abort check now compares the start HEAD to HEAD
only over `ros_ws/src teensy_link controller config sim` (what the live nodes load), and `--first-slot N` resumes at
slot N keeping earlier logs. Verification: 2026-11-25, `bash -n tools/probes/bug129_launch_series.sh` clean;
`pytest tests/sim/test_bug129_series_report.py -q` (parser untouched).
