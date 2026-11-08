---
title: "FEAT-46: can-bridge FW 42 / PROTOCOL 13 uplinks a STOP record per lane group, and the bridge classifies each STOP as a stamp gap, a mapping step, a frame queued but not promoted, or none of these (built and tested, NOT flashed)"
type: feature
date: 2026-11-08
status: in-progress
phase: "BUG-58 firmware STOP record"
related_issues:
  - FEAT-46
  - BUG-58
related_entries:
  - 2026-11-07-sitting-debrief.md
  - 2026-11-06-jb174-stream-socket-queue.md
files_changed:
  - config/generate_udp_protocol.py
  - config/generated/udp_protocol.h
  - config/generated/udp_protocol.py
  - docs/teensy-udp-protocol.md
  - tools/probes/teensy_link_profiling/jetson/udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/udp_protocol.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.cpp
  - ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/Teensy_code_canbridge.ino
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - teensy_link/stop_record.py
  - teensy_link/protocol.py
  - teensy_link/rpc_args.py
subsystem:
  - can-bridge
  - teensy_link
tags:
  - scheduler
  - observability
  - wire-protocol
---

## What and why

At the 2026-11-07 Sitting 20 of 23 scheduler STOPs had a clean host stream: frames at most 35.7 ms old on arrival, lost 0 downlink, every drop counter at 0. The cover runs from each frame's stamped play time, so the STOP has three candidates (a stamp gap, a step in the wall-offset mapping, a due frame left unpromoted), and nothing on the wire could tell them apart.

Each STOP now writes a record for its lane group (0 legs, 1 Hand): seq, overwritten count, the entry instant on the wall clock, Hand C1 flag, the playing frame's t_start, queue depth and head, the last four queued frames (t_origin_us, mapped t_start, arrival; a per-group ring the net task fills inside the queue's existing PRIMASK section), the first frame queued after the STOP (filled by the net task), and the last wall-offset change > 1 ms (`time_base_last_jump`, noted inside `set_wall_anchor`'s critical section). Times are int32 us relative to the STOP entry, saturated, so exact within ±35.8 min. `start_rel - origin_rel` is the offset change since that frame was queued, so a mapping step reads directly. HeartbeatT2J grows 190 -> 380 B (two slots), PROTOCOL_VERSION 12 -> 13, FW_VERSION 41 -> 42. Two slots rather than one because the legs and the Hand stop in the same tick at a C2 stop, so a single slot would always lose one.

Observability only: the ISR builds a record only on the tick a STOP is declared (a 4-deep queue scan and a 4-entry copy), with no allocation and nothing that blocks; no lane reads any of it.

Host: `teensy_link/stop_record.py` decodes both slots and classifies (`classify_stop_record`); `teensy_bridge_node` logs one `firmware STOP record #n <group> (...)` WARN per record when its next-frame part is filled or 1 s after it appears, on its own call site. The existing STOP WARN is unchanged.

## Verification

- 2026-11-08, `pio run -e teensy41` (Teensy_code_canbridge): SUCCESS, text 254272, data 35520, bss 111936.
- 2026-11-08, `temp/firmware_native/test_leg_interp` (built by `python tests/firmware/native/build.py`): 65 of 65 cases, 9679 assertions pass (the new FW 42 case: playing t_start, ring, depth and head, time-base change, the next frame filled by the net task, an unread record overwritten and counted; and the C1 record checks in the FW 37 case).
- 2026-11-08, `python -m pytest tests/firmware/test_udp_protocol_xlang.py tests/firmware/test_bridge_fw_version_xref.py tests/teensy_link/ tests/ros/test_teensy_bridge_node_bridge_diag.py tests/ros/test_teensy_bridge_node_read.py -q -n 4`: 657 passed.
- Gate: `./run_tests.sh` result in the commit message.

NOT flashed and no colcon build: the parent session does both. Until the lockstep flash an FW 41 board is DARK to this host.
