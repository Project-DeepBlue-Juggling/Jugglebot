---
title: "Wave camera adapter on CAN3: bridge FW 33 WAVE_CMD, the wave_camera decoder, camera/* topics and record_start/record_stop services - flashed and brought up on the Jetson 2026-10-25"
type: feature
date: 2026-10-19
status: in-progress
phase: "wave-camera-can3-integration - Phase 2 (bridge FW 33 + Jetson node)"
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
  - tests/ros/test_gui_record_panel.py
  - tests/ros/js/record_panel_harness.js
  - ros_ws/gui/js/camera-panel.js
  - ros_ws/gui/css/camera-panel.css
  - ros_ws/docs/wave_camera_can_adapter.md
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

# Wave camera adapter on CAN3: bridge FW 33 and the Jetson node

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
  Bridge FW 32 → 33 (FW 30 → 31 when written; renumbered at the 2026-10-25 merge with main, whose FW 31/32 are the leg lead clamp), `PROTOCOL_VERSION` stays 10 (BB_CANCEL precedent).
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

### Jetson bring-up, 2026-10-25 (`plans/active/PROMPT-wave-camera-jetson-bringup.md`)

The branch was 64 commits behind `main`, where FW 31 and 32 had become the leg
lead clamp. Merged `origin/main` in and renumbered the Wave bridge to **FW 33**
(`459725eb`). No wire change: PROTOCOL_VERSION 10, RPC 0x0062 and CAN IDs
0x7E2-0x7E7 are as before, so the ESP32 firmware is unaffected. Robot ODrives
unpowered throughout (DC_BUS_UNDER_VOLTAGE at launch, expected); ESP32 adapter
fw 1 and the Wave camera on the cone-role bus (CAN3 peripheral) as the only node.

1. Checkout `2026-10_wave-camera-can3` @ `459725eb` in `~/Desktop/Jugglebot`.
2. `./run_tests.sh` (2026-10-25, merged tree): **PASS, 7250 passed, 13 skipped**.
3. `colcon build --packages-select jugglebot_interfaces jugglebot` (2026-10-25): **2 packages finished**; `ros2 interface show jugglebot_interfaces/msg/WaveCameraHeartbeat` prints the message.
4. `pio run -e teensy41 -t upload` (2026-10-25): **SUCCESS**; BridgeIdentity sniff on :5005 **fw_version=33, protocol_version=10**; launch log `BRIDGE_FW_CHECK: OK — can-bridge v33 [install_skew=0]`.
5. `ros2 launch jugglebot jugglebot_launch.py` (2026-10-25 15:06): up; GUI (system `jugglebot-gui.service`) serves `#camera-panel` and `js/camera-panel.js` from the checkout.
6. Bus (rclpy probe on `camera/heartbeat`, 12 s): **121 heartbeats (10 Hz)**, connected, state IDLE, camera_on, uart_seen, intent_fresh. Raw relay 0x7E3 every 100.0 ms by bridge stamp, ~14 cone frames/s, CAN3 err_tx/err_rx 0. (With the ESP unplugged the bus was silent: can3_rx 0, no errors.)
7. Time sync: **time_synced true, sync_age_ms 0** from the first heartbeat.
8. Three start/stop pairs (`ros2 service call camera/record_{start,stop}`, 15:08:03-15:08:38): all **success** (node log `recording started` / `recording stopped`); START (kind 1, reason 0, time_synced) and STOP (kind 2) events for each; heartbeat RECORDING with elapsed_ms counting in 500 ms steps; recording_seq 2, 3, 4. CLI wall time 1.0-3.9 s per call, but the in-node path is fast: `record_stop` from a persistent rclpy client while stopped returned `already stopped` in **9-17 ms** (5 calls), and a timed `record_start` returned `recording started` in **0.071 s**. The 3.9 s outlier (pair 3 stop) is CLI start-up and discovery, not the ESP.
9. Keepalive (recording, then SIGINT to the launch at 15:12:02.009, raw :5005 sniff): heartbeat RECORDING -> STOPPING at **+2.84 s** -> IDLE; REC_EVENT `74 b4 81 6a a2 5b 0c 05` = **AUTO_STOP, reason INTENT_TIMEOUT**, SYNCED, TICK_STAMPED, cmd_seq 12, recording_seq 5; event wall time 1.1 ms before bridge RX. Status 0x5f -> 0x17 (INTENT_FRESH and GPI2_LOW cleared). The fault byte read 0x08 before the test too (latched earlier), so the event, not the fault bit, is the evidence.

Fixed in this bring-up: `recordings_this_session` read one high (1 before the session's first recording, then 2, 3, 4 against raw 1, 2, 3). The node's first heartbeat always predates its own session_id at the adapter (intents stream only once a heartbeat is fresh), so it carries the previous session's count; `SeqUnwrapper` seeded its total from that raw 1, and the adapter's reset to 0 one intent later read as a counter restart and added 0. `WaveRecordingTracker` now counts recordings from 0 (`SeqUnwrapper(seed_from_zero=True)`); `recording_seq`, a since-boot counter, still seeds from the raw value. New test `test_tracker_recordings_ignore_the_previous_sessions_count` replays the hardware sequence; `test_heartbeat_published_connected` now feeds a first heartbeat before the one it asserts. `pytest tests/ros/test_wave_camera.py tests/ros/test_teensy_bridge_node_wave.py` (2026-10-25): **62 passed**. After colcon build and relaunch (15:16), the first heartbeat reads recordings_this_session 0 (the adapter still held 4 from the previous launch).

Against the plan's Phase 4 gate (`wave_camera_can_adapter.md` s10 rows 2-7 and 9): row 2 = steps 6-7, passed; row 4 = step 8 starts, passed except the camera clip counter and `last_tick_to_clip_us`, not checked; row 5 = step 8 stops, passed; row 6 = step 9, passed (AUTO_STOP INTENT_TIMEOUT; the fault bit was already latched); row 8 = step 10. Rows 3, 7 and 9 were not part of the prompt's steps; all three were run afterwards, and the owner accepted the row 3 and row 9 outcomes below.

Row 3 (2026-10-25, 600 s rclpy monitor of `camera/heartbeat` + `camera_diag`, `temp/logs/wave_row3_sync.log`; row 9 ran during its first ~2 min): 6001 heartbeats, **0 disconnected, 0 unsynced, max sync_age_ms 0**; offset_spread_us **p95 86, max 144** (< 200); accepted_per_s min 99, rejected_per_s max 0; steps 1 -> 1 (no new step; the 1 is the step-10 replug). track_err_us **p95 54 but max 124**: at least one 1 s window broke the row's < 100. Passed except that one bound.

Row 7 (2026-10-25 16:03, `record_start` from an rclpy probe, owner switched the camera off then on, `temp/logs/wave_row7_tick_lost.log`): **AUTO_STOP reason TICK_LOST** stamped 16:03:40.112 (tick_stamped false: the release edge, no tick to stamp), received 56 ms later; RECORDING -> STOPPING -> IDLE within 100 ms; fault 0x02 TICK_LOST_MID_REC latched; status_bits 0x5F -> 0x1E (**GPI2 released**). Power-up: ticks resumed 16:03:53.6, flapped for ~11 s through the camera boot (ticks_missed 255), steady from 16:04:04.5; **no START** in the 20 s that followed although host_desired stayed 1 (the adapter acts only on a new seq or a desired edge). The 0.3 s bound cannot be timed exactly without the power-off instant; the adapter's rule is 200 ms without a tick, and the stop followed it. Passed.

Fixed after row 7 (owner): the node left desired at 1 after the auto-stop and kept streaming it. The adapter ignores that (no new seq, no edge), but an adapter reboot would read the next frame as a new session and START from IDLE unasked. The REC_EVENT ingest now sets desired to 0 on an AUTO_STOP whose cmd_seq echo is the node's current seq (no seq bump: a desired 1->0 edge in IDLE is a no-op at the adapter; a newer operator start stands). Tests `test_auto_stop_resets_desired_so_an_adapter_reboot_cannot_restart`, `test_a_stale_auto_stop_does_not_cancel_a_newer_start`; `pytest tests/ros/test_teensy_bridge_node_wave.py tests/ros/test_wave_camera.py` (2026-10-25): **74 passed**. Row 7 repeated after colcon build and relaunch (16:07-16:10, `temp/logs/wave_row7_autostop_fix.log`): AUTO_STOP TICK_LOST at 16:09:21.101 and **host_desired 1 -> 0** with it (status_bits 0x0E, DESIRED clear, against 0x1E before the fix); camera back and steady from 16:09:45.5, no START. The same run shows record_start's CLEAR_FAULTS on hardware (faults 0x02 -> 0x00 on the start frame). Its start returned CLIP_MISSING (reason 5, fault 0x04): no "Created new clip." line within the window, about three minutes after the first power-cycle. The owner is fairly sure a clip was recorded at 16:07, so this reads as a missed or late UART line rather than a camera that did not record: an ESP-side question (first start after a camera power-cycle; start tick 16:07:13.553, the service reply 168 ms later at 16:07:13.721) for the bench PC, not investigated here.

Side reading for row 4: `p3.last_tick_to_clip_us` read 10758 us on the Jetson against the row's 15-17 ms (bench 15.5 ms); not investigated here.

Row 9 (2026-10-25, 20 start/stop runs from one persistent rclpy client, 1.2 s recording + 1.0 s gap, `temp/logs/wave_row9_latency.log`): **20/20 confirmed**, every reply `recording started` with reason NONE (no CLIP_MISSING), synced and tick-stamped; start tick minus request **min 2.4, median 35.7, max 62.1 ms** (inside [0, 534 ms]); service reply 54-96 ms. The **spread is 59.7 ms, about two tick periods, not the "about one" (33.3 ms) the row expects**. The row measures from the node's RPC send, which the node does not log, so this ran from the client's request wall time; the extra ~27 ms of jitter therefore includes ROS service delivery into the node and cannot be split from the bridge, CAN and ESP part without a node-side send stamp.

10. GUI (owner, 2026-10-25): **passed**. Panel state correct, REC then STOP with elapsed counting, correct with the minimap expanded, and with the adapter unplugged or the camera off (button disabled, clear status text).

Added in this bring-up (owner decision after step 10): fault clearing. The fault byte latches and the node had no way to send `CLEAR_FAULTS` (WAVE_CMD flags bit0), so the GUI's amber glyph kept INTENT_TIMEOUT_AUTOSTOP (latched before the session) and gained SYNC_STEP during step 10. The bag dates SYNC_STEP: heartbeat lost 15:18:52 (the CAN unplug, ESP still powered), back 15:19:04, fault byte 0x08 -> 0x28 at 15:19:07 with INFO page 2 reading accepted 18/s, rejected 82/s, track_err 65535 us, steps 0 -> 1. Reading `time_slave_math.h`: the 11 s silence passed SYNC_STALE_MS (5 s), so the slave re-anchored on the first beacons after the replug; those were most likely delayed, the line sat low, normal beacons read as early, and two all-rejected windows forced a re-anchor (the counted step). Live afterwards: track_err 0-1 us, 100 accepted/s, 0 rejected; only the latch remained. Now:
- `record_start` sets CLEAR_FAULTS on its own new-seq frame, so a take's faults are that take's; the 5 Hz repeats always send flags 0, so the clear can never keep wiping faults mid-take.
- New `camera/clear_faults` (Trigger) sends the CURRENT desired and cmd_seq with the flag. An unchanged seq is a keepalive to the adapter's state machine, so it never starts or stops a recording; a bumped seq would start one from IDLE if desired were still 1 (the node leaves desired at 1 after a TICK_LOST auto-stop). It succeeds when the second heartbeat after the send reads faults 0, and fails naming anything that re-latched.
- GUI: the warning glyph is clickable (`canClearFaults`: ROS up, not fenced, not pending, fresh, adapter connected, faults non-zero; the camera need not be on), its tooltip ends "(click to clear faults)", and the result flashes on the status line.

Verification: `pytest tests/ros/test_teensy_bridge_node_wave.py tests/ros/test_wave_camera.py` (2026-10-25): **71 passed**; `pytest tests/ros/test_gui_record_panel.py` (2026-10-25): **15 passed**; `pytest tests/ros/test_choreography_map.py` after `python tools/gen_choreography_map.py` (2026-10-25): **29 passed**. Hardware, after colcon build and relaunch (2026-10-25 15:31, persistent rclpy client): `camera/clear_faults` returned **success, 'faults cleared (were: INTENT_TIMEOUT_AUTOSTOP, SYNC_STEP)' in 0.188 s**, a second call 'no faults were latched' (0.196 s), and the heartbeat then read faults 0x00. The record_start clear is covered by the node tests only: exercising it on hardware writes a clip.
