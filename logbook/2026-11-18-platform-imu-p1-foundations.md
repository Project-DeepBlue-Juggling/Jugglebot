---
title: "Platform IMU foundations: protocol constants and spec (P1a), capture ring, protocol logic and non-blocking driver with native tests (P1b) (IDEA-20)"
type: feature
date: 2026-11-18
status: complete
phase: "platform-imu P1"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - config/protocol_config.yaml
  - ros_ws/docs/platform_imu_protocol.md
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_capture.h
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_proto.h
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.h
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.cpp
  - tests/firmware/native/test_platform_imu.cpp
  - tests/firmware/test_platform_imu_native.py
subsystem: [platform-teensy, firmware]
tags: [imu, capture, can, native-tests]
---

# Platform IMU foundations (P1)

**What.** P1a (4bca0665) fixed the wire contract: `ros_ws/docs/platform_imu_protocol.md` plus the IDs and `platform_imu:` constants in `config/protocol_config.yaml`, generated into `PlatformImu*`. P1b adds the hardware-free firmware logic the Platform Teensy will run in P2: `imu_capture.h` (capture ring over injected storage, anchors, the seq-addressed frozen image, chunkable CRC-32, header build), `imu_proto.h` (frame pack/unpack, the state machine and its refusal codes, READ pacing under an injected wrap-safe clock), and a copy of the bench ICM-42688 driver whose init is a non-blocking step function (`initStart` / `initStep`: one bounded I2C transaction per call, waits as due times) configured for +/-16 g, +/-1000 dps, 1 kHz. Nothing is wired into the `.ino` except the pin static_assert now naming 18/19 beside the Hand fan pin.

**Why.** The ring, the address space and the state table are where a capture silently goes wrong (an off-by-one at a region edge, a RESUME that thaws the wrong capture, a CRC that disagrees with the Jetson's). Pure headers under host g++ make those executable before any board is flashed; the CRC is cross-checked against `zlib.crc32` on a wrapped capture with a discontinuity anchor.

**Open.** The UI filter code for ODR/4 in GYRO_ACCEL_CONFIG0 was not verified against DS-000347 text (no readable mirror was reachable), so `icm42688.h` keeps the bench value 0x00 with a TODO; the header records the raw register so the Jetson sees what was applied.

## Verification

- P1a: `pytest tests/firmware/test_platform_imu_protocol.py -q -p no:cacheprovider` (2026-11-18) passed in its own commit; see 4bca0665.
- P1b native test binary (`g++ -std=c++17 -O2 -Wall -I hal_shims -I . test_platform_imu.cpp`, run 2026-11-18): 27 test cases, 1860 assertions, 0 failed.
- P1b scoped pytest: see the commit message for the command and result line (2026-11-18).
- `pio run -e teensy40` on a scratchpad copy of the Platform sketch with a throwaway translation unit including the new headers and driver (2026-11-18): SUCCESS; probe object `.dmabuffers` 393836 B for the 384 KB ring plus anchors, driver 2157 B text.

**Not verified.** Nothing ran on hardware, and the driver's step function was compiled for the Teensy but never exercised against a chip. The full gate result is in the phase-closing commit.
