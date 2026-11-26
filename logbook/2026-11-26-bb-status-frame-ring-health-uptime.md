---
title: "Ball Butler FW 10 reports its CAN receive-ring health and uptime every 500 ms on the CMD_RESULT channel; /link_status carries them as the bb_status keys, 'absent' on FW 9 (BUG-133, held until the FW 10 flash)"
type: feature
date: 2026-11-26
status: complete
phase: "Offline 2026-11-26"
related_plan: ""
related_issues: [BUG-133]
files_changed:
  - config/protocol_config.yaml
  - config/generated/protocol_config.h
  - config/generated/protocol_config.py
  - ros_ws/src/jugglebot/jugglebot/protocol_config.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/protocol_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/protocol_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/protocol_config.h
  - ros_ws/src/jugglebot/jugglebot/can/ball_butler_status.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/rpc_args.py
  - tools/probes/gui_synthetic_stack.py
  - tests/ros/test_ball_butler_status.py
  - tests/ros/test_teensy_bridge_node_bb.py
subsystem: [ball-butler, firmware, ros, observability]
tags:
  - bridge
---

# Ball Butler STATUS frame: receive-ring health and uptime in the bag

**Why.** BUG-133's offline measurement could not tell whether the Ball Butler's CAN receive ring strands frames (a delay line that makes it release late, growing with uptime): no recorded topic carried its uptime or any ring counter. The owner decided on 2026-11-26 to add them with the FW 10 flash.

**What.**

- Ball Butler FW 10 (BallButler branch `offline/bug133-flexcan`, still FW 10 because FW 10 has never been flashed): `CanInterface::loop()` probes the patched FlexCAN_T4 RX ring (`rxRingProbe`, true head/tail occupancy) before each `events()` pop, and every 500 ms sends one unsolicited CMD_RESULT (0x7D5) of the new command_type `STATUS` (5): byte 1 `rx_ring_depth_min`, byte 2 `rx_ring_depth_max`, byte 3 `rx_ring_leak_max` (frames, 255 sat, reset each window), bytes 4-6 `uptime_s` (uint24 LE, wraps at 194 days), byte 7 hardware FIFO overruns since boot (255 sat). Layout in `config/protocol_config.yaml` `ball_butler.command_types`.
- `jugglebot/can/ball_butler_status.py` (new, pure): `BallButlerStatus.from_cmd_result` and `is_status_frame`.
- `teensy_bridge_node._on_cmd_result` routes STATUS frames to `_record_bb_status` before the THROW / CANCEL waiters; `/link_status` gains `bb_uptime_s`, `bb_rx_ring_depth_min`, `bb_rx_ring_depth_max`, `bb_rx_ring_leak_max`, `bb_rx_fifo_overflows`, `bb_status_age_s`, all reading `absent` until the first frame. One info log on the first frame; a throttled warning when the leak is above 0.
- `BB_FW_VERSION_EXPECTED` 9 to 10 (this branch only: it merges with the flash).

**Reading it.** `rx_ring_depth_min` above 0 means the ring never emptied for a whole 500 ms window: a standing queue, the BUG-133 signature (each queued frame delays the 0x7DD sync and every command by about one frame time, 2.6 ms). With the patched library the leak must read 0. FW 10 can show the fix holds; it cannot convict FW 9 after the fact.

## Discussion

Four carriers were weighed; the bridge was flashed today (FW 53, unflown), so any option needing a can-bridge change or a PROTOCOL_VERSION bump was out.

- **Heartbeat 0x7D1:** all 8 bytes are used, and the bridge decodes it into fixed HeartbeatT2J fields. Only byte 1 (`state_data`, 0 outside ERROR) passes through raw. Multiplexing 10 nibble slots over it would pass three free-running 10 Hz stages (BB, bridge, host timer), which drop and repeat slots, so a value could tear across seconds. Rejected.
- **FW_UPDATE_REPLY 0x7D7 (unsolicited):** relayed verbatim, but the flasher's BEGIN and COMMIT waits take the first 0x7D7 frame without checking the opcode, so a status frame could be read as BEGIN's answer. Rejected.
- **INFO poll at 1 Hz:** costs a blocking bridge RPC every second for the life of the launch, beside the throw RPCs. Rejected.
- **CMD_RESULT 0x7D5 with a new command_type (chosen):** relayed verbatim with the bridge wall stamp through an 8-deep ring (drained 4 per telemetry tick; 2 frames/s cannot evict a throw verdict). Every host so far drops command types other than THROW and CANCEL, so an old host ignores it and FW 9 never sends it.

**Control-safety walk-through.** Ball Butler loop: two index reads and a compare per pass, plus one 8-byte frame every 500 ms on 0x7D5, which sits below every ODrive id on CAN1. Nothing touches motion, time sync or dispatch. Host: the RX thread does one 8-byte decode under the node lock; the 10 Hz link_status timer adds six KeyValues. No setpoint, feedforward or timing path changes.

## Verification

- 2026-11-26, BallButler worktree, `cd ball_butler_main; PLATFORMIO_LIB_EXTRA_DIRS=/home/jetson/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_canbridge/lib pio run -e teensy40`: SUCCESS, `firmware.hex` sha256 `e49156f1d6483b2c9d31faae19f92e831ee95ccbb7724b01c409a798cff89ffe`. Not uploaded.
- 2026-11-26, `python -m pytest -q tests/ros/test_ball_butler_status.py tests/ros/test_teensy_bridge_node_bb.py -k "status or bb_fw"`: 23 passed. With `teensy_bridge_node.py` restored to origin/main and `ball_butler_status.py` removed: the decode file fails at collection and the four new node tests fail.
- 2026-11-26, scoped run, `JUGGLEBOT_BALLBUTLER_DIR=<BallButler worktree on offline/bug133-flexcan> python -m pytest -q -rs tests/ros/test_ball_butler_status.py tests/ros/test_ball_butler.py tests/ros/test_teensy_bridge_node_bb.py tests/ros/test_teensy_bridge_node_bb_yaw.py tests/ros/test_gui_geometry.py tests/firmware/test_bb_fw_update_xref.py tests/firmware/test_ball_butler_xref.py tests/firmware/test_config_drift.py tests/ros/test_ball_butler_node.py`: **352 passed, 0 skipped, in 28.69 s**. `test_bb_fw_update_xref.py` pins `BB_FW_VERSION_EXPECTED` to the BallButler `FwUpdate.h`: it passes only against the FW 10 branch, and it fails against the live `../BallButler` (FW 9) until both branches merge.
