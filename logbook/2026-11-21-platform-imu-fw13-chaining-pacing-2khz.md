---
title: "Platform Teensy FW 13: READ chaining, fixed-schedule frame pacing and a selectable 2 kHz capture rate; Jetson chains one READ ahead, live pace parameter, set_rate service"
type: feature
date: 2026-11-21
status: in-progress
phase: "IDEA-25 / IDEA-20 P5 preparation"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_proto.h
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.cpp
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_platform/platformio.ini
  - config/protocol_config.yaml
  - teensy_link/platform_imu.py
  - teensy_link/rpc_args.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/platform_imu_gate.py
  - ros_ws/docs/platform_imu_protocol.md
subsystem: [platform-imu, platform-teensy, teensy-link]
tags: [imu, firmware, upload, can]
---

# Platform Teensy FW 13

## What and why

Owner decisions of 2026-11-21 after the upload-time review: larger read blocks, fixed-schedule pacing, a 2 kHz capture
rate for comparison tests, and a 1600 frames/s trial at the next Sitting. No decimated or compressed upload (the
owner wants the full data; delta coding measured only 30 % saving on a real Attempt capture).

## Discussion

**Chaining instead of larger blocks.** The approved goal was a larger `read_max_words`. The can-bridge (FW 46)
refuses a READ above 256 words, and that limit is compiled from the same generated constant, so raising it would have
needed a bridge flash and would silently change the bridge's clamp at its next build. Instead the Platform appends a
READ whose `first_seq` equals the active block's end (reply READ_EXTENDED 0x02); any other READ still replaces the
block. The Jetson keeps one READ ahead, so the roughly 16 ms pause per READ (about 4 s per whole ring) disappears,
where 2048-word blocks would still have paid 32 pauses. Tradeoff accepted: two blocks are outstanding at once, so the
host bookkeeping is more involved; a chained READ answered with OK (the board replaced the block) is handled as holes
for the normal refill, and two such replies turn chaining off for the upload. Chaining is used only when the frozen
capture's own header reports FW 13 or later, so an FW 12 board is never sent a chained READ.

**The 2 kHz rate needs the bridge for a run-time switch.** A new op SET_ODR (0x07) switches 1 kHz and 2 kHz when no
capture is frozen (refused ERR_FROZEN otherwise); the ring restarts as a fresh capture and holds 16 s. The FW 46
bridge does not relay ops it does not list, so until a bridge release adds the case the 2 kHz rate is reached with
the build environment `teensy40_imu2k`, an image that boots at 2 kHz. The bridge change was not made here because
another Session holds a branch that also takes FW 47.

**Pacing.** Each frame is due one gap after the previous frame's due time, with lateness clamped to two gaps, so the
delivered pace equals the request (0.83 ms was measured for 0.80 requested) and a stall cannot burst more than two
frames.

## Changes

- Platform firmware: chaining, schedule pacing, SET_ODR, STATUS flag 0x20 (ODR_2K), header `odr_hz` is the real
  rate, drain period and anchor back-dating follow the rate. At 2 kHz the chip FIFO holds 64 ms, so a loop pass over
  about 60 ms costs one overflow marked DISCONTINUITY (a data gap, not a fault). No RAM growth; IMU still optional,
  no new interrupt sources, one I2C transaction per loop pass.
- Jetson: chaining (`UploadMachine._maybe_chain`, lead 80 ms), service `platform_imu/set_rate` (SetString "1000" or
  "2000") with plain-words refusals, `odr_hz` on `platform_imu/status`, the Attempt span follows the ring length,
  `imu_read_gap_10us` settable live (40 to 255, applies from the next READ), new capture `stats` keys
  (`pass1_missing`, `pass1_missing_seqs`, `reads_chained`, `reads_replaced`).
- Model (fake board, whole ring, gap 80): FW 12 55.5 s, FW 13 52.0 s without loss; 59.7 s and 55.4 s at 0.4 % loss;
  44.4 s at gap 63 with loss.

## Verification

Tests and the at-rest board check: see the commit messages (the flash and the board readings follow in a later
commit to this entry).
