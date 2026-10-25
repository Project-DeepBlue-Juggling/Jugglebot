---
title: "Wave camera adapter on CAN3: bridge FW 31 WAVE_CMD, the wave_camera decoder, camera/* topics and record_start/record_stop services - code landed, not flashed"
type: feature
date: 2026-10-19
status: in-progress
phase: "wave-camera-can3-integration - Phase 2 (bridge FW 31 + Jetson node)"
related_plan: wave-camera-can3-integration.md
subsystem: can
tags:
  - testing
files_changed:
  - config/protocol_config.yaml
  - config/generate_config.py
  - config/generate_udp_protocol.py
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/protocol_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - teensy_link/protocol.py
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/can/wave_camera.py
  - ros_ws/src/jugglebot/jugglebot/can/__init__.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot_interfaces/msg/WaveCameraHeartbeat.msg
  - ros_ws/src/jugglebot_interfaces/msg/WaveRecordEvent.msg
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/docs/choreography.md
  - tests/ros/conftest.py
  - tests/ros/test_wave_camera.py
  - tests/ros/test_teensy_bridge_node_wave.py
  - tests/ros/test_launch_nodes.py
  - tests/firmware/native/test_rpc_dispatch.cpp
  - tests/firmware/test_bridge_fw_version_xref.py
  - tests/firmware/test_udp_protocol_xlang.py
  - docs/adr/0014-wave-camera-on-can3.md
  - docs/adr/index.md
  - mkdocs.yml
  - plans/active/wave-camera-can3-integration.md
  - plans/active/INDEX.md
  - logbook/2026-10-19-wave-camera-can3-bridge-and-node.md
  - logbook/INDEX.md
---

# Wave camera adapter on CAN3: bridge FW 31 and the Jetson node

## Motivation

The robot needs to start and stop the Freefly Wave from ROS 2 and know each
start as a wall-clock instant good to about 1 ms. An ESP32 adapter on the
cone-role bus does the camera side ([`ros_ws/docs/wave_camera_can_adapter.md`](../ros_ws/docs/wave_camera_can_adapter.md), firmware in `ros_ws/src/jugglebot/WaveCamera_ESP_code/`,
approved 2026-10-19). This entry is the Jugglebot half: config, bridge
firmware, node, messages and tests. The decision record is ADR-0014.

## Design

- IDs 0x7E2-0x7E7 (`WaveCameraCanId`, `CAN_ID_WAVE_*`), all DLC 8, LE. The cone
  and the adapter are mutually exclusive on the drop.
- One additive RPC, `WAVE_CMD` 0x0062 (`ArgWaveCmd`, 8 B), sent verbatim as
  0x7E2 by `send_cone_frame()`; `can_cone_tx` FAILED (no partner) maps to
  ERR_BUS_DOWN; `desired > 1` or a nonzero reserved byte is ERR_BAD_ARGS.
  Bridge FW 30 → 31, `PROTOCOL_VERSION` stays 10 (BB_CANCEL precedent).
- The node streams WAVE_CMD at 5 Hz while the heartbeat is fresh (desired 0 or
  1, `retries=0`, 0.2 s); `camera/record_start|stop` bump `cmd_seq`, send once
  and wait up to 2.5 s for the REC_EVENT echoing that seq. Start success = START
  (CLIP_MISSING with a warning) or REFUSED/NO_CHANGE; a failed start resets
  desired to 0 with a fresh seq. `session_id` = node start time.
- Event time: 40-bit wall µs when synced (±6.4-day host window), else
  `ConeFrame.t_bridge_us - age`, flagged unsynced.

## Implementation

`jugglebot/can/wave_camera.py` holds the codecs, `reconstruct_event_time_us`,
`event_wall_us`, `SeqUnwrapper`, `WaveRecordingTracker` and `evaluate_ack`; the
node only wires them. Publishers: `camera/heartbeat` (10 Hz, connected=False
before the first frame), `camera/record_event`, `camera_diag` (1 Hz, the INFO
pages). All three are bagged. The GUI panel deliberately subscribes to
`camera/heartbeat` only; `camera/record_event` is bagged and available to
tools but not rendered by the panel. A disconnected (stale) heartbeat publishes
`recording=False`, `elapsed_ms=0` and a zero `start_stamp` rather than
extrapolating the last-known recording. Stale "CAN2" cone comments in `rpc.cpp` fixed;
the two in `telemetry.cpp` and `time_sync_master.cpp` are left for a later
touch of those files.

## Verification

Run on Windows on 2026-10-19 (scratch venv: system Python 3.11 + hypothesis +
pytest-xdist; `sensor_msgs` stubbed by a scratch-only pytest plugin, because the
bridge-node tests cannot import without a ROS install). `./run_tests.sh` was NOT
run (Linux/Jetson-only); it must run on the Jetson before the commit.

- `python config/generate_config.py --check` (2026-10-19): **CONFIG FRESH: 14 artifact(s)**, one EXTERNAL DRIFT line for `../BallButler` (written with `--no-external`).
- `pytest tests/ros/test_wave_camera.py tests/ros/test_teensy_bridge_node_wave.py tests/ros/test_teensy_bridge_node_cone.py -p sensor_stub` (2026-10-19): **all pass** (40 codec + 28 node/cone).
- `pytest tests/ros -n 4 --dist loadfile -m "not nightly" -p sensor_stub` (2026-10-19): **3050 passed, 121 failed, 37 errors**; HEAD (`git archive`, same command): 2975 passed, 121 failed, 37 errors. No failure is new: the one set difference (`test_hand_step_violation_not_sent`) also fails on HEAD when its file runs alone.
- `pytest tests/firmware -p sensor_stub` (2026-10-19): 205 passed, 3 failed, 19 errors; HEAD: identical failure set (Windows: native harness `windows.h` macros, path separators).
- Native `test_rpc_dispatch` built by hand with `g++ -DNOGDI -DWIN32_LEAN_AND_MEAN` (2026-10-19): **35/35 cases, 264/264 assertions**.
- `pio run -e teensy41` in `Teensy_code_canbridge` (2026-10-19): **SUCCESS**.
- `pytest tests/teensy_link -n 4` (2026-10-19): **451 passed, 3 skipped**.
- `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/ros/test_choreography_map.py tests/ros/test_launch_nodes.py tests/ros/test_wave_camera.py` (2026-10-19): **203 passed**.

Re-run after the review fixes (stale-heartbeat zeroing, three more node tests,
`encode_wave_cmd` byte vector), 2026-10-19, same scratch venv:

- `pytest tests/ros/test_wave_camera.py tests/ros/test_teensy_bridge_node_wave.py tests/ros/test_teensy_bridge_node_cone.py -p sensor_stub`: **71 passed**.
- `pytest tests/teensy_link -n 4`: **452 passed, 3 skipped**.
- `pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/ros/test_choreography_map.py tests/ros/test_launch_nodes.py -p sensor_stub`: **163 passed**.
- `python config/generate_config.py --check`: **CONFIG FRESH: 14 artifact(s)**.

Not yet done: colcon build, bridge reflash, bench rows (plan Phase 4).
