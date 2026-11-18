---
title: "Platform IMU foundations: protocol constants and spec (P1a), capture ring, protocol logic and non-blocking driver with native tests (P1b) (IDEA-20)"
type: feature
date: 2026-11-18
status: in-progress
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
  - tests/firmware/test_platform_imu_protocol.py
  - tests/firmware/native/test_icm42688.cpp
  - tests/firmware/native/build.py
  - tests/firmware/native/hal_shims/Wire.h
  - tests/firmware/test_native_firmware.py
  - tests/firmware/test_platform_hand_fan_xref.py
  - config/generate_config.py
  - config/generate_udp_protocol.py
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - plans/active/platform-imu.md
  - map/idea-platform-accelerometer.md
  - tools/probes/README.md
subsystem: [platform-teensy, firmware]
tags: [imu, capture, can, native-tests]
---

# Platform IMU foundations (P1)

**What.** P1a (4bca0665) fixed the wire contract: `ros_ws/docs/platform_imu_protocol.md` plus the IDs and `platform_imu:` constants in `config/protocol_config.yaml`, generated into `PlatformImu*`. P1b adds the hardware-free firmware logic the Platform Teensy will run in P2: `imu_capture.h` (capture ring over injected storage, anchors, the seq-addressed frozen image, chunkable CRC-32, header build), `imu_proto.h` (frame pack/unpack, the state machine and its refusal codes, READ pacing under an injected wrap-safe clock), and a copy of the bench ICM-42688 driver whose init is a non-blocking step function (`initStart` / `initStep`: one bounded I2C transaction per call, waits as due times) configured for +/-16 g, +/-1000 dps, 1 kHz. Nothing is wired into the `.ino` except the pin static_assert now naming 18/19 beside the Hand fan pin.

**Why.** The ring, the address space and the state table are where a capture silently goes wrong (an off-by-one at a region edge, a RESUME that thaws the wrong capture, a CRC that disagrees with the Jetson's). Pure headers under host g++ make those executable before any board is flashed; the CRC is cross-checked against `zlib.crc32` on a wrapped capture with a discontinuity anchor.

**Open.** The UI filter code for ODR/4 in GYRO_ACCEL_CONFIG0 was not verified against DS-000347 text (no readable mirror was reachable), so `icm42688.h` keeps the bench value 0x00 with a TODO; the header records the raw register so the Jetson sees what was applied.

**Review.** The phase-end audit found 12 behaviour and test findings and 9 narrative ones; all are closed. The four that mattered: stale samples were served after RESUME (the post-RESUME FIFO flush was not awaited before sampling resumed); the driver init did up to six I2C transactions in one call (now one per call); init returned OK when it had not started; and the u32 sample counter would wrap at 49.7 days (now wrap-safe).

**Discussion.** A host-side Wire fake (`hal_shims/Wire.h`, `test_icm42688.cpp`) was added because the owner declined a bench rehearsal on 2026-11-18, so the host tests are the only pre-flash evidence for the driver's write sequence, wait handling across the `micros()` wrap and error recovery. The Python test that compares the write sequence with the bench driver parsed from source keeps the lifted copy honest. The 2026-11-18 and 2026-11-19 owner decisions (uploads while Armed, the under-a-minute target) amended the spec and the plan in the docs commit that closes P1; they change no header logic.

## Verification

- P1a: `pytest tests/firmware/test_platform_imu_protocol.py -q -p no:cacheprovider` (2026-11-18): passed in its own commit (dc7e3df2 after the rebase onto main).
- P1b scoped (`pytest tests/firmware/test_native_firmware.py tests/firmware/test_platform_imu_native.py` plus the tests/sim docs, plans, logbook and mapkit readers, `-q -p no:cacheprovider`, 2026-11-18): 317 passed, 1 failed in 433.71 s; the one failure was the dump case (a discontinuity anchor trimmed by design), fixed in the test. Rerun of `pytest tests/firmware/test_platform_imu_native.py tests/firmware/test_native_firmware.py -q -p no:cacheprovider` (2026-11-18): 20 passed in 278.92 s.
- Audit-fix commit, native binaries (`g++ -std=c++17 -O2`, 2026-11-18): test_platform_imu 41 cases, 326525 assertions, 0 failed; test_icm42688 10 cases, 1109 assertions, 0 failed. With `-fsanitize=address,undefined` (2026-11-18): 41 cases, 327002 assertions and 10 cases, 1110 assertions, no sanitizer output.
- Audit-fix commit, `source ~/Desktop/PDJ_venv/venv/bin/activate && pytest tests/firmware -q -p no:cacheprovider` (started 2026-11-18, finished after midnight): 343 passed, 4 skipped in 324.51 s; then `pytest tests/firmware/test_native_firmware.py -q -p no:cacheprovider -k "icm or platform_imu"` (2026-11-19): 2 passed.
- Docs commit (2026-11-19): `g++ -std=c++17 -O2 -Wall -I hal_shims -I <Teensy_code_platform> test_platform_imu.cpp` after the N8 test-data fix (accel config 0x06): 41 cases, 326525 assertions, 0 failed.
- `pio run -e teensy40` on a scratchpad copy of the Platform sketch plus a throwaway TU including `imu_proto.h` and `icm42688.h` (2026-11-18): SUCCESS, text 62784, data 12992, bss 29088, no warnings from the IMU files.
- Full gate (`./run_tests.sh --full`): FULL-GATE-TRIPLE-PENDING

**Not verified.** Nothing ran on hardware, and the driver's step function was compiled for the Teensy but never exercised against a chip. The entry stays in-progress until the full-gate line above is filled.
