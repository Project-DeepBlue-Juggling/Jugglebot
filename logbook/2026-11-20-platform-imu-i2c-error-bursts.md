---
title: "Platform IMU: recurring I2C error bursts on the robot and a permanent PROBING state; FW 12 retries failed transfers and recovers the bus before every re-probe, the cause of the corruption is still open"
type: investigation
date: 2026-11-20
status: in-progress
phase: "platform-imu P3"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - ros_ws/docs/platform_imu_protocol.md
  - ros_ws/src/jugglebot/Teensy_code_platform/Teensy_code_platform.ino
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.cpp
  - ros_ws/src/jugglebot/Teensy_code_platform/icm42688.h
  - ros_ws/src/jugglebot/Teensy_code_platform/imu_service.h
  - ros_ws/src/jugglebot/Teensy_code_platform/platformio.ini
  - teensy_link/rpc_args.py
  - tests/firmware/native/build.py
  - tests/firmware/native/test_platform_imu_diag.cpp
  - tests/firmware/native/test_platform_imu_service.cpp
  - tests/firmware/test_native_firmware.py
subsystem:
  - can
  - ros
  - tools
tags:
  - safety
  - testing
  - docs
---

# Platform IMU: I2C error bursts and a permanent PROBING state

## Summary

The ICM-42688-P on the Platform Teensy (I2C on pins 18/19 at 1 MHz, Platform FW 11) logs an I2C error every 15 to 40 s on
the robot. Each error costs a 1 s BACKOFF and a DISCONTINUITY, so a 32 s capture would be riddled with 1 s holes. The
bench proof of concept (`2026-11-18-imu-bench-proof-of-concept`) saw 0 errors with the same breakout and cable. Separately,
one morning the IMU sat in PROBING for good. Two things are in this entry: a firmware defect that made the PROBING state
permanent, which is fixed in Platform FW 12, and a corruption on the I2C wires whose cause is not found. FW 12 is built
and tested but has not been flashed, so this entry stays open.

## Symptoms

- 2026-11-19 evening, first listen after wiring: RUNNING, one I2C error about 40 s after first contact.
- 2026-11-20 morning: the launch came up (PLATFORM_FW_CHECK OK v11, BRIDGE_FW_CHECK OK v45) but the IMU was permanently in
  PROBING, with i2c_errors saturated at 255 and fifo_overflows at 20.
- A Platform Teensy reboot (re-flash over CAN; the chip stays powered) restored RUNNING. The chip and cable were therefore
  alive and the stuck state was on the firmware side.
- Steady state afterwards: one i2c_errors increment per 15 to 40 s.

## Diagnosis

Four hypotheses were tested.

- Bus speed or signal integrity at 1 MHz. An A/B build at 400 kHz gave 18 errors in 300 s, no better. Withdrawn.
- The CAN receive interrupt racing the Wire transfer. A diagnostic variant masked the CAN1 interrupt around every
  transfer (pio env `teensy40_imudiag_mask`): 13 errors in 4 min against 14 in 5 min unmasked. Withdrawn.
- Firmware logic. A diagnostic build (env `teensy40_imudiag`, `-DPLATFORM_IMU_DIAG`, which pages the error kind into the
  STATUS bytes) recorded what failed. The kinds were fifo_count and burst. Wire stages were a short read (1 to 127 bytes
  received of 64 to 133 requested) and a register-address NACK. The LPI2C controller flags were SDF (stop seen
  mid-read), NDF|MBF|BBF (NACK) and ALF (arbitration lost on a bus with a single controller). The two runs saw 103 and
  68 bad FIFO headers. Transfers took 0.4 to 1.3 ms with no timeouts, and the longest loop gap was 2 ms (one of 29 ms).
  Conclusion: the corruption is on the I2C wires, not in the firmware logic.
- Weak pull-ups. The owner measured 3.9 kOhm from SDA and from SCL to 3V3, both lines about 3.3 V idle, same cable as
  the bench, wires soldered to the Teensy pins. Ruled out. There is no oscilloscope at the robot.

Not yet answered: what on the robot differs from the bench. Candidates are other nets on pins 18/19 of the carrier board,
motor and CAN noise coupling, and cable routing. A question is pending with the owner on whether anything else is
connected to pins 18/19 on the carrier board.

The permanent PROBING is a second, separate defect. The re-probe did not reset the I2C controller or the bus, so a bus
that had wedged stayed wedged until the Teensy rebooted.

## Discussion

Why retry instead of fixing the cause first. Finding the cause needs hardware access and may end in rewiring or a
different bus. The retry removes the stuck state and most of the data loss in either case, and it is needed even on a
clean bus, because one dropped transfer per hour should not cost a capture a 1 s hole. So the retry ships first and the
cause stays open.

The accepted tradeoff. I2C has no checksum. If FIFO headers are being corrupted (about 100 per 5 min in the diagnostic
runs), the sample bytes probably are too, and a corrupted sample can enter a capture undetected. The retry does nothing
about that. It only makes the detected failures cheap.

Decision deferred until FW 12 is measured on the robot: harden I2C (stronger pull-ups, twisted or shielded cable,
plausibility rejection of samples) or move the IMU to SPI on the breakout's spare pads. If the error rate on FW 12 is low
and the captures pass a plausibility check, I2C stays. If undetected bad samples show up, SPI.

## Fix

Platform FW 12, three commits on branch `imu-i2c-fix`, not yet flashed at the time of writing.

1. Unconditional bus recovery (9 SCL pulses, STOP, `Wire.begin`) before every re-probe.
2. A retry policy in RUNNING. A failed FIFO count read triggers a recovery and a re-count, with no BACKOFF and no
   DISCONTINUITY. A failed burst or a bad header drops the drain, flushes, anchors a DISCONTINUITY and stays RUNNING. A
   third failure in a run goes to BACKOFF as before. A run containing a failed burst is ended only by a successful burst.
   Every failure still counts in i2c_errors.
3. The closing anchor of an unfinished drain is moved back by unread packets x 980 us.

The recovery pass takes about 0.12 ms. One I2C transaction per loop pass is kept. Nothing runs in the 10 s boot settle or
after FwUpdate BEGIN. `PLATFORM_FW_VERSION_EXPECTED` goes from 11 to 12 in `teensy_link/rpc_args.py`. The state machine
text is updated in `ros_ws/docs/platform_imu_protocol.md` section 4.

One audit pass found no blocking issue. Two findings were fixed: a persistent burst failure never reached BACKOFF, and the
closing-anchor stamp was late.

## Verification

- 2026-11-20, native `test_platform_imu_service` under `g++ -fsanitize=address,undefined`: 28/28 pass.
- 2026-11-20, native `test_platform_imu_diag` under the same sanitizers: 3/3 pass.
- 2026-11-20, `flock /tmp/jugglebot-tests.lock pytest tests/firmware -q -p no:cacheprovider`: 355 passed, 4 skipped in
  400.28 s.
- 2026-11-20, `pio run -e teensy40` and `pio run -e teensy40_imudiag`: both build SUCCESS.

Pending (to be filled in by the owner's session):

- Full gate (`flock /tmp/jugglebot-tests.lock ./run_tests.sh --full`, run 2026-11-20 on the branch tip before this line was added): parallel 10626 passed, 58 skipped, 1 xfailed in 814.09 s; serial 10 passed in 41.11 s; RESULT: PASS.
- Flash receipt for Platform FW 12 (BridgeIdentity sniff): pending.
- Hardware i2c_errors rate on FW 12, same measurement as the FW 11 runs: pending.

## Outcome

Open. FW 12 is expected to remove the permanent PROBING and most of the 1 s holes. Whether the corruption itself is
fixed depends on the cause, which is unresolved.

## Open Questions

- What on the robot differs from the bench that corrupts the I2C wires (carrier board nets on pins 18/19, noise coupling,
  cable routing)? Is anything else connected to pins 18/19 on the carrier board?
- Do corrupted sample bytes reach captures undetected, and does that force plausibility rejection or a move to SPI?

## Related observations

From the Sitting of 2026-11-20 (FW 11's first flight, IMU upload toggle OFF): the tilt request read (+0.0107, +0.0084) rad
at Level, and the Hand fan worked on command and in auto.
