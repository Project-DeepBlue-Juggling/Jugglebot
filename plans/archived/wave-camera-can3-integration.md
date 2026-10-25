---
title: Wave camera adapter on CAN3 — ROS 2 record start/stop with a tick-stamped start instant
created: 2026-10-19
status: completed
owner: Harrison
last_updated: 2026-10-25
completed: 2026-10-25
archived: 2026-10-25
pathway: wave-camera-can3
scope: >-
  One-button Wave camera recording from ROS 2: an ESP32 adapter on the cone-role bus (CAN3) drives the
  camera's GPI2 record line and stamps its 30 Hz tick, the can-bridge relays it (WAVE_CMD RPC 0x0062,
  FW 33), and teensy_bridge_node exposes camera/* topics, record_start/record_stop services and a GUI
  REC/STOP panel. Phases: adapter firmware, bridge and Jetson node, GUI, Jetson bring-up on hardware.
related_logbook:
  - 2026-10-19-wave-camera-can3-bridge-and-node.md
  - 2026-10-19-gui-camera-record-panel.md
  - 2026-10-25-wave-camera-gpio-findings.md
related_config:
  - config/protocol_config.yaml → can_ids.wave_camera
  - config/protocol_config.yaml → wave_camera
related_code:
  - ros_ws/src/jugglebot/jugglebot/can/wave_camera.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_wave_record
  - ros_ws/src/jugglebot/Teensy_code_canbridge/rpc.cpp::send_cone_frame
  - ros_ws/src/jugglebot/WaveCamera_ESP_code/ (ESP32 adapter firmware, PlatformIO)
---

# Wave camera adapter on CAN3

## Context

The robot will start and stop recording on the Freefly Wave from ROS 2 and
report each start and stop as a robot wall-clock instant good to about 1 ms, so
footage lines up with the session bag. An ESP32 (the adapter, firmware in
`ros_ws/src/jugglebot/WaveCamera_ESP_code/`, moved in from a standalone repo
on 2026-10-25) drives the camera's GPI2 in LVL mode, stamps the
first 30 Hz camera tick after the falling edge, and talks to the can-bridge on
the cone-role bus (the CAN3 peripheral). The binding wire contract is
[`ros_ws/docs/wave_camera_can_adapter.md`](../../ros_ws/docs/wave_camera_can_adapter.md)
(the Wave Camera CAN adapter design, approved 2026-10-19; bench results in
[`wave_camera_findings.md`](../../ros_ws/docs/wave_camera_findings.md)); the architectural decision is
[ADR-0014](../../docs/adr/0014-wave-camera-on-can3.md).

Owner decisions (2026-10-19): `session_id` is the node start time (Unix s);
`record_start` succeeds on START (reason NONE, or CLIP_MISSING with a warning)
or REFUSED/NO_CHANGE; `record_stop` succeeds on STOP or REFUSED/NO_CHANGE; the
intent streams at 5 Hz whenever the adapter heartbeat is fresh, `desired=0`
included; the adapter auto-stops after 3 s without intent; the cone and the
camera adapter are mutually exclusive on the drop; bridge FW 32 → 33 (FW 31 on the branch; renumbered at the 2026-10-25 merge with main, whose FW 31/32 are the leg lead clamp).

## Architecture

```
GUI camera panel ── rosbridge ── camera/heartbeat (10 Hz), camera/record_event,
                                 camera/record_start|stop (std_srvs/Trigger), camera_diag
teensy_bridge_node ── RPC WAVE_CMD 0x0062 (5 Hz) ──▶ can-bridge FW 33 ── 0x7E2 ──▶ ESP32
                   ◀── CONE_FRAME 0x85 (verbatim) ◀── 0x7E3 / 0x7E4 / 0x7E5 ◀──┘
```

IDs 0x7E2-0x7E7 (`WaveCameraCanId`), all DLC 8, little-endian. Event time is
40 bits of wall µs when synced, else an age that the node subtracts from
`ConeFrame.t_bridge_us`. No new MsgType; `PROTOCOL_VERSION` stays 10.

## Implementation Phase Summary

| # | Phase | Repo | Status | Gate |
|---|-------|------|--------|------|
| 1 | Adapter firmware (CAN link, time slave, record FSM, heartbeat/INFO) | wave | done: firmware in `WaveCamera_ESP_code/`; rows 1 and 10 verified by the owner 2026-10-25 | native tests; bench rows 1 and 10 with a USB-CAN sniffer |
| 2 | Bridge FW 33 + Jetson node, msgs, config, tests | JB | done: FW 33 flashed + verified on the Jetson 2026-10-25 | pytest; `pio run -e teensy41`; colcon + reflash on the Jetson |
| 3 | GUI camera panel | JB | done: verified on the robot 2026-10-25 (owner) | node harness tests; bench row 8 |
| 4 | Bench bring-up on the robot | both | done 2026-10-25: rows 2, 5, 6, 7 and 8 passed; row 4 completed by the owner (clip counter and tick-to-clip); rows 3 and 9 accepted by the owner (track_err max 124 us; spread 59.7 ms from the client) | CAN adapter design s10 rows 2-7 and 9 |
| 5 | Optical frame calibration | both | later: not in this plan's closure, filed as a map Feature | a clapboard flash maps the start tick to a recorded frame |

## Implementation Phases

### Phase 1 — Adapter firmware

Lives in `ros_ws/src/jugglebot/WaveCamera_ESP_code/` (CAN adapter design s5, s12
phases 0-1); built and flashed from the bench PC, never from the Jetson. This plan tracks it
only as the prerequisite for Phase 4.

### Phase 2 — Bridge FW 33 and the Jetson node

Landed in the working tree on 2026-10-19 (logbook
`2026-10-19-wave-camera-can3-bridge-and-node.md`):

- `config/protocol_config.yaml` gains `can_ids.wave_camera` and a `wave_camera`
  section; `generate_config.py` emits `WaveCameraCanId`, `WaveCamera`,
  `WaveCameraState`/`EventKind`/`Reason` (C++) and `CAN_ID_WAVE_*`, `WAVE_*`,
  `WaveState`/`WaveEventKind`/`WaveReason` (Python).
- `generate_udp_protocol.py` adds RpcMethod `WAVE_CMD` 0x0062 and `ArgWaveCmd`.
- `rpc.cpp`: `case RpcMethod::WAVE_CMD` and `send_cone_frame()`;
  `FW_VERSION` 31; `EXPECTED_BRIDGE_FW_VERSION` 31 (both 33 since the 2026-10-25 merge, see Context).
- `jugglebot/can/wave_camera.py`, `WaveCameraHeartbeat.msg`,
  `WaveRecordEvent.msg`, the bridge node's decoder branches, publishers,
  intent timer and Trigger services; launch rosbag list.

Remaining for this phase at 2026-10-19, on the Jetson (all four done 2026-10-25, logbook § Jetson bring-up):

1. `colcon build --packages-select jugglebot_interfaces jugglebot`.
2. Reflash the bridge (`Teensy_code_canbridge/BRINGUP.md` Step 5) and confirm
   `bridge_fw_version` 31 on BRIDGE_IDENTITY.
3. Full relaunch, so rosbridge loads the new message types.
4. `./run_tests.sh` on the Jetson (the Windows run could not execute the gate).

### Phase 3 — GUI camera panel

`camera-panel.js` with a pure view core, the REC/STOP button, `mm:ss`, and the
state line (CAN adapter design s8). `main.js` will add `camera/heartbeat` and
`camera/record_event` to `GUI_SUBSCRIBED_TOPICS`, which
`tests/ros/test_rosbridge_websocket_lean.py` pins, and the replay allowlist
will need the two topics if replays are to show the panel.

### Phase 4 — Bench bring-up

The Jetson session prompt for this phase is
[`PROMPT-wave-camera-jetson-bringup.md`](PROMPT-wave-camera-jetson-bringup.md).
Run CAN adapter design s10 rows 2-7 and 9 in order: heartbeat at 10 Hz and
`connected=true`; 10 min sync stability; `record_start` success within 2 s with
a tick-stamped synced START; `record_stop`; node kill while recording gives
AUTO_STOP INTENT_TIMEOUT 3.0-3.4 s later; camera power-off gives AUTO_STOP
TICK_LOST; 20-run start latency with no CLIP_MISSING. Run `./run_tests.sh
--full` before the sitting.

### Phase 5 — Optical frame calibration (later)

A clapboard flash recorded by the camera will map the START tick to a frame
index, and confirm the assumed stop quantisation. Not scheduled; out of scope
for this plan's first closure.

## Testing Plan

- `tests/ros/test_wave_camera.py`: hand-built byte vectors for every frame,
  40-bit reconstruction across a wrap, the unsynced age path, ID
  non-collision, sequence unwrap, ack rules.
- `tests/ros/test_teensy_bridge_node_wave.py`: heartbeat and staleness, event
  publishing, service success / CLIP_MISSING / refusal / timeout paths, intent
  stream on and off with heartbeat freshness.
- `tests/firmware/native/test_rpc_dispatch.cpp`: WAVE_CMD frame bytes, gate
  closed gives ERR_BUS_DOWN, bad args; `test_bridge_fw_version_xref.py` FW 33
  clause; `test_udp_protocol_xlang.py` re-pinned layout digest.

## Notes for Collaborators

- The 2.5 s service wait covers the 500 ms GPI2 stretch, one tick, the 150 ms
  clip window and relay margin, and stays under the adapter's 3 s timeout.
- A failed or timed-out `record_start` resets `desired` to 0 with a fresh
  `cmd_seq`, so the robot is never recording without a confirmation.
- The shared JSON wire-vector fixture the firmware and the node both consume (CAN adapter design s7, the
  clapboard precedent) is not written yet; the vectors live in
  `test_wave_camera.py` until it is.

## Archival note (2026-10-25)

Archived complete; the Pathway `wave-camera-can3` is marked complete with it, and
the bring-up prompt `PROMPT-wave-camera-jetson-bringup.md` is archived alongside.
Shipped: the ESP32 adapter firmware, bridge FW 33 with RPC `WAVE_CMD` 0x0062, the
`camera/*` topics and `record_start` / `record_stop` / `clear_faults` services in
`teensy_bridge_node`, and the GUI REC/STOP panel. On-robot rows 2, 5, 6, 7 and 8
passed in the 2026-10-25 bring-up (logbook
`2026-10-19-wave-camera-can3-bridge-and-node.md` § Jetson bring-up); the owner
verified rows 1, 4 and 10 independently and accepted rows 3 (track_err max 124 us
against < 100) and 9 (start spread 59.7 ms, measured from the client) as they are.
The `2026-10_wave-camera-can3` branch has no commits beyond `main`.

Residue, filed as map items under this Pathway: Phase 5 optical frame
calibration (Feature); the shared JSON wire-vector fixture (Feature, the vectors
live in `tests/ros/test_wave_camera.py` until then); the CLIP_MISSING reply on the
first start after a camera power-cycle, read as a missed or late UART line on the
adapter (Bug, for the bench PC).
