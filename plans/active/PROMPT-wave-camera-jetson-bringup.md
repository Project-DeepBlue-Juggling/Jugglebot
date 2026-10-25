---
title: "Wave Camera: Jetson bring-up prompt"
status: active
kind: reference
owner: harrison
created: 2026-10-25
last_updated: 2026-10-25
pathway: wave-camera-can3
scope: >-
  Session prompt for the Wave camera Jetson bring-up (Phase 4 of wave-camera-can3-integration.md): verify,
  build, flash the can-bridge, launch, and walk the bus, time-sync, service, keepalive and GUI checks.
---
# Wave Camera: Jetson bring-up prompt

You are working on the Jugglebot Jetson (Ubuntu 20.04, ROS2 Foxy, Python 3.8) in the repo checkout at ~/Desktop/Jugglebot (or the worktree I name below). Read CLAUDE.md and DOCUMENTATION_GUIDE.md first and follow them, including the test gate and commit rules. I commit; do not commit or push unless I say so.

## Goal
Get one-button camera recording working end to end: a REC/STOP button in the GUI starts and stops recording on a Freefly Wave camera, shows a running indicator and elapsed time, and shows adapter and camera state. The camera is driven by an ESP32 adapter that is already flashed and connected to CAN3 (the "cone role" bus, FlexCAN CAN3, Teensy pins 31/30). The cone is not attached; the ESP is the only other node on that bus.

## What already exists (do not rewrite it)
All Jugglebot-side code is written and reviewed on branch `2026-10_wave-camera-can3`, cut from origin/skill-stack. The Wave Camera sub-project lives entirely on that branch: the ESP32 adapter firmware is in `ros_ws/src/jugglebot/WaveCamera_ESP_code/` (a standalone PlatformIO project, for reading only on the Jetson) and its design docs are in `ros_ws/docs/`. It was developed on a Windows PC, so none of it has run on a Jetson, a Teensy or real ROS. Your job is to verify, deploy, bring up and fix what is broken, not to redesign.

First fetch the branch: `git fetch && git branch -a | grep wave-camera` (you check it out in step 1).

Read these from the branch before touching anything: ros_ws/docs/wave_camera_can_adapter.md (the binding wire and behaviour contract), docs/adr/0014-wave-camera-on-can3.md, plans/active/wave-camera-can3-integration.md, logbook/2026-10-19-wave-camera-can3-bridge-and-node.md, logbook/2026-10-19-gui-camera-record-panel.md, ros_ws/src/jugglebot/jugglebot/can/wave_camera.py. For background: ros_ws/docs/wave_camera_findings.md (measured camera behaviour), ros_ws/docs/wave_camera_gpio_explorer.md, and ros_ws/src/jugglebot/WaveCamera_ESP_code/README.md. Summary of the contract:

- CAN frames, all DLC 8, little-endian, 11-bit IDs, block 0x7E2-0x7E7:
  - 0x7E2 WAVE_CMD, Jetson to ESP at 5 Hz while the ESP heartbeat is fresh: desired(u8, 1 record / 0 stop), cmd_seq(u8), flags(u8), reserved, session_id(u32, node start time). Sent through the new bridge RPC WAVE_CMD 0x0062 (bridge FW_VERSION 31).
  - 0x7E3 WAVE_HEARTBEAT, ESP to Jetson at 10 Hz from boot: state, status bits, fault bits, elapsed (100 ms units), sync_age, recordings, cmd_seq echo.
  - 0x7E4 WAVE_REC_EVENT, ESP to Jetson on each start, stop, auto-stop or refusal: 40-bit time, kind/flags, seq echo, recording_seq.
  - 0x7E5 WAVE_INFO, four rotating pages at 1 Hz each: identity, camera tick, time sync, link.
- ROS: topics `camera/heartbeat` (WaveCameraHeartbeat), `camera/record_event` (WaveRecordEvent), `camera_diag` (DiagnosticStatus); services `camera/record_start` and `camera/record_stop` (std_srvs/Trigger, up to about 2.5 s). GUI panel is `#camera-panel`, right of the collapsed state minimap, in ros_ws/gui/js/camera-panel.js.
- ESP behaviour you must know: it owns the camera record line from boot. Recording only happens while intent frames keep arriving; if they stop for 3 s the ESP stops the recording by itself. It sends heartbeats from boot, which is what opens the bridge's CAN3 transmit gate (a node must transmit first). Its event stamps are only wall-clock accurate once it has received the bridge's 0x7DD time beacon, which needs a fresh Jetson time anchor.
- The camera must be fully booted (about 12 s after power-on) and set to GPI = LVL for recording to start.

## Steps, in order. Stop and report at the first failure; do not skip ahead.
1. Environment: activate the venv, then `git checkout 2026-10_wave-camera-can3`. Confirm `git status` is clean and note the commit.
2. Tests: `./run_tests.sh`. Record the date, command and result. Any failure that also fails on origin/skill-stack is pre-existing: list it and continue. Any new failure is yours to fix.
3. Build: `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot`, then `source install/setup.bash`. Verify `ros2 interface show jugglebot_interfaces/msg/WaveCameraHeartbeat` works.
4. Flash the bridge Teensy from the Jetson using the documented path in Teensy_code_canbridge/README.md (`pio run -e teensy41 -t upload`, which calls the local teensy_loader_cli). Do not flash the ESP32, and do not build or upload anything from ros_ws/src/jugglebot/WaveCamera_ESP_code/; the ESP is flashed from my bench PC. After the flash confirm the bridge reports firmware version 31 and the Jetson node accepts it (EXPECTED_BRIDGE_FW_VERSION is 31).
5. Launch the full stack with `ros2 launch jugglebot jugglebot_launch.py` (a full restart is required after an interface change). Make sure the GUI server is up (jugglebot-gui.service or `python3 ros_ws/gui/gui_server.py --port 8081`).
6. Bus check: `ros2 topic echo camera/heartbeat`. Pass: `connected: true` within a few seconds, state IDLE, camera_on true once the camera has finished booting. If the heartbeat never appears, check in this order: CAN3 wiring and the 120 ohm terminations, bridge CAN3 RX counters in the bridge diagnostics, that CONE_FRAME packets arrive (the Jetson node logs unknown IDs), then the ID and DLC of the frames. Remember the bridge has a known load-dependent fault on its CAN3 drive path; it is fine with one node but note any TX errors.
7. Time sync: watch sync_age and the time_synced flag. Pass: time_synced true and sync_age under 5 within 10 s of the heartbeat appearing. If not, check that the bridge is time-synced to the Jetson (its 0x7DD only runs with a fresh anchor) and report.
8. Services: with the camera in standby, call `ros2 service call camera/record_start std_srvs/srv/Trigger`. Pass: success true within about 2.5 s, the camera starts a clip, heartbeat shows RECORDING and elapsed counting up, and a `camera/record_event` START arrives with `time_synced: true`. Wait about 5 s, then call `camera/record_stop`. Pass: success true, STOP event, camera stops. Run the pair three times and note each service reply message.
9. Keepalive: start a recording, then stop the node or the launch. Pass: the camera stops by itself about 3 s later and the next heartbeat shows an INTENT_TIMEOUT fault.
10. GUI: open the GUI, find the panel at the bottom left next to the state minimap, confirm it shows the right state, press REC and then STOP, and watch the elapsed time. Also check it with the minimap expanded and with the adapter or camera off (button disabled, clear status text). Take a screenshot if you can.
11. Record keeping: update the logbook entry's verification section with the date, command and result for each step. Do not edit the plan beyond ticking the phases you actually verified.

## Rules
- Anything that fails: find the cause in the code, fix it with the smallest change, add a test where the repo has a test seam, and tell me. If the cause is in the ESP32 firmware (frame layout, timing, state machine), do not guess and do not flash it; you may read ros_ws/src/jugglebot/WaveCamera_ESP_code/ to explain the behaviour, but write exactly what you observed (frame bytes, timestamps) so I can fix it on the PC.
- Do not change the frame IDs, byte layouts or service names without telling me, since the ESP firmware has to match.
- Starting recordings writes clips to the camera. Ask me before running more than the three start/stop pairs plus the keepalive test.
- Do not touch other devices on the bus (Ball Butler, platform, ODrives) and do not restart anything that is running a juggle or a hardware sitting.

## Report back
For each step: pass or fail with the evidence (command output, the heartbeat fields, event bytes). Then list the fixes you made, files changed, the final `git status`, and the suggested commit message with its Logbook-Entry trailer.
