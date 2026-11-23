---
title: "Platform Teensy FW 13: READ chaining, fixed-schedule frame pacing and a selectable 2 kHz capture rate; Jetson chains one READ ahead, live pace parameter, set_rate service"
type: feature
date: 2026-11-21
status: complete
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

## Board check at rest (2026-11-21, drive power off, can-bridge FW 46)

Flashed over CAN with `pio run -e teensy40 -t upload` (receipt "Platform Teensy FW version: 12 -> 13"), then
`teensy40_imu2k` and back to `teensy40` (both "13 -> 13"). The board is left on the 1 kHz default image. Whole-ring
uploads, 32000 samples each, all complete with the CRC OK and 0 discontinuities:

| Capture | Rate | Pace | Wall | READs (chained) | Duplicates | Block timeouts | Missing after pass 1 |
|---|---|---|---|---|---|---|---|
| `20261121T000652Z_7C19` | 1 kHz | gap 80 | 52.6 s | 263 (250) | 0 | 1 | 261 |
| `20261121T000820Z_7C1A` | 1 kHz | gap 63 | 41.4 s | 258 (250) | 0 | 1 | 258 |
| `20261121T001437Z_AFCB` | 2 kHz | gap 80 | 51.8 s | 259 (250) | 0 | 0 | 6 |

Reference on FW 12, at rest, gap 80 (`20261120T095142Z_C33B`): 56.25 s, 263 READs, 158 duplicates.

- Sensor at 1 kHz: |a| 0.9999 g, noise 1.3 mg and 0.047 deg/s per axis, fitted rate 993.93 Hz. At 2 kHz: |a| 1.0001 g,
  1.4 mg and 0.048 deg/s, fitted rate 1987.86 Hz, the ring spans 16.1 s. `i2c_errors` and `fifo_overflows` 0 in every
  header and over 70 s of status at 2 kHz (135 messages).
- Spectra at rest (Welch, 4096 points): the same noise floor at both rates (about 6e-5 g/rtHz and 2.3e-3
  (deg/s)/rtHz below 400 Hz), no narrow lines. In the 2 kHz capture 20 to 21 % of the acceleration variance and 13 to
  15 % of the gyro variance lies above 500 Hz. At rest that is sensor noise; the comparison that matters is during an
  Attempt, which is on the next Sitting.
- `platform_imu/set_rate` returns the expected refusal naming the bridge firmware.
- Unexplained: in both 1 kHz uploads exactly one whole 256-word block was missing after the first pass (seqs
  28672..28927 and 22528..22783) with one block timeout and `reads_replaced` 0, so one READ was not served. It was
  refilled and costs under a second. It did not happen in the 2 kHz upload, and the launch logs hold no READ error at
  those times. Not diagnosed.
- Not checked: anything Armed, and the FW 13 behaviour during an Attempt.

## Verification

Tests: see the commit messages (`git log --grep "Logbook-Entry: 2026-11-21-platform-imu-fw13-chaining-pacing-2khz"`).

## Closed (2026-11-23)

Closed with the plan's archival. The code ran Armed and in an Attempt at the Sitting of 2026-11-23 (`2026-11-23-sitting-debrief-robot`): uploads complete and checksum-verified. Upload time is carried by IDEA-25.
