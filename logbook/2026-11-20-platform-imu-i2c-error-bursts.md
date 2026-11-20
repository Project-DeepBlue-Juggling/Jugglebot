---
title: "Platform IMU: recurring I2C error bursts on the robot and a permanent PROBING state; FW 12 retries and recovers the bus, and 2.2 kOhm external pull-ups on SDA and SCL removed the corruption"
type: investigation
date: 2026-11-20
status: resolved
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
permanent, fixed in Platform FW 12, and a corruption on the I2C wires. The corruption is fixed by hardware: the owner
added a 2.2 kOhm resistor from SDA to 3V3 and one from SCL to 3V3 at the Platform Teensy, in parallel with the existing
3.9 kOhm (about 1.4 kOhm total). After that, 300 s of known-data reads and 300 s of the normal FIFO path showed zero
errors. Residual: no longer soak has been run with 48 V on and the robot Armed, and three observations below are still
unexplained.

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
  the bench, wires soldered to the Teensy pins. This was first ruled out; the ruling-out was wrong and is withdrawn (see
  Discussion and Withdrawn claims). There is no oscilloscope at the robot.

Owner facts gathered afterwards: nothing else is on pins 18/19. A 12 V to 5 V buck module feeds the Platform Teensy and
the IMU sits on the Teensy's 3V3 pin. The four IMU wires (3V3, GND, SDA, SCL) are twisted together and run near the fan,
CAN and inclinometer wires; the cable is identical to the bench. In the code, the Hand fan output is on/off (no PWM) and
is off with 48 V off, and the inclinometer is read only on a tilt request, so neither was active in the earlier runs.

Known-data diagnostic (branch `imu-i2c-diag2`, commit 8d71c392, unpushed; envs `teensy40_imudiag_const`,
`_const_400k`, `_const_100k`). In RUNNING it reads the 32-byte register block 0x4C-0x6B, which is fixed after init (the
reference is learnt from the first 8 good reads), about 500 times per second instead of draining the FIFO, and counts
mismatches. This separates corruption on the wires from any FIFO logic. 2026-11-20, 300 s each, launch down, robot at
rest, 48 V off, before the pull-up change:

| run | reads | failed reads | corrupted reads | corrupted bytes | bits 1->0 | bits 0->1 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 MHz fan off | 143954 | 22 | 59 | 1333 | 14 | 8134 |
| 1 MHz fan forced on | 131949 | 31 | 70 | 1499 | 11 | 12859 |
| 400 kHz | 143936 | 28 | 79 | 1732 | 13 | 11697 |
| 100 kHz (period 8 ms) | 35844 | 13 | 3422 | 3851 | 21 | 4504 |

The signature at 1 MHz and 400 kHz: a corrupted read is a run of about 22 bytes reading 0xFF to the end of the transfer
(the last mismatch was expected 0x00, read 0xFF at offset 31). The chip stops driving SDA mid-read, as after a spurious
stop/start. The error rate follows the number of transactions, not the time on the bus (400 kHz has 2.5 times the bus
time of 1 MHz for the same reads), which argues against asynchronous outside noise. The fan effect is weak, within
scatter. The FIFO-path diagnostic (env `teensy40_imudiag`) over 300 s before the fix: i2c_errors 1 -> 24 (14 count
fails, 8 burst fails) and 116 bad FIFO headers (6 -> 122), the values mostly 0x00 or data-like (0x05, 0x0D, 0x71, 0x16),
last good header 0x68: misaligned packets.

Fix confirmed. After the owner added the 2.2 kOhm pull-ups (about 20:00), `teensy40_imudiag_const` at 1 MHz for 300 s:
143964 reads, 0 failed, 0 corrupted, 0 flipped bits, i2c_errors 0 -> 0. `teensy40_imudiag` (FIFO path) for 300 s:
RUNNING 300/300, i2c_errors 0 -> 0, 0 count fails, 0 burst fails, 0 bad headers, 0 bus recoveries, 0 FIFO overflows.

Still unexplained:

- Why the bench was clean with the same cable and the same 3.9 kOhm.
- The 3 FIFO overflows in the FW 12 baseline run (fifo_overflows 0 -> 3 in 330 s, below).
- The 3422 single-byte mismatches at 100 kHz, nearly all single-byte (the last: expected 0x0F, read 0x7F at register
  0x4E), where the 1 MHz and 400 kHz runs show 22-byte 0xFF tails.

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

The earlier decision was deferred until FW 12 was measured: harden I2C (stronger pull-ups, twisted or shielded cable,
plausibility rejection of samples) or move the IMU to SPI. It is now made: I2C stays, hardened with stiffer pull-ups.
The known-data run showed no corrupted reads after the change, so SPI is not needed on this evidence.

Hypothesis withdrawn: the pull-ups were not the cause. This entry, and the Sitting's working session, had ruled out the
pull-ups because 3.9 kOhm is a normal value and 400 kHz was no better than 1 MHz. That reasoning was wrong. The failure
is per-edge, not a per-bit timing budget: with slow rising edges there is a window in which a glitch registers on the
chip as a spurious stop or start, and that is exactly the observed signature (the chip stops driving SDA mid-read). A
slower clock does not shorten the rising edge, so 400 kHz and 100 kHz did not help, while a stiffer pull-up shortens
the rising edge and did. "3.9 kOhm is a normal value" was a statement about the textbook case, not a measurement of this
bus with its twisted cable and its neighbours. What the 400 kHz result did show correctly was that bit timing is not the
limit; it was misread as clearing the pull-ups. Not shown: the exact mechanism. There is no oscilloscope at the robot,
so the rising edge was never looked at; the fix is supported by the before and after counts, not by a waveform.

## Fix

Two changes. Firmware: Platform FW 12, three commits on branch `imu-i2c-fix` (now on main). Hardware: 2.2 kOhm resistors
SDA-to-3V3 and SCL-to-3V3 at the Platform Teensy, in parallel with the existing 3.9 kOhm, added by the owner on
2026-11-20 at about 20:00. FW 12 stays as the safety net: its retry policy and bus recovery remain.

Firmware part:

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

- Full gate (`flock /tmp/jugglebot-tests.lock ./run_tests.sh --full`, run 2026-11-20 on the branch tip before this line was added): parallel 10626 passed, 58 skipped, 1 xfailed in 814.09 s; serial 10 passed in 41.11 s; RESULT: PASS.
- Flash receipt for Platform FW 12, 2026-11-20 16:01-16:02: `pio run -e teensy40 -t upload`, image 89088 B, crc32
  0x39C78251, 0 rewinds, VERIFY OK, "Platform Teensy FW version: 11 -> 12". The can-bridge was FW 46 / PROTOCOL 14
  (BridgeIdentity (46, 14)).
- FW 12 baseline before the pull-up change, 2026-11-20, 330 s direct `TeensyLinkClient` listen (launch down, robot at
  rest, 48 V off): state RUNNING in 330 of 330 STATUS frames; i2c_errors 0 -> 23; fifo_overflows 0 -> 3; sync_peak
  median 1.18 ms, max saturated 2.54 ms. The retry policy works (FW 11 spent 1 s in BACKOFF per error) and the error
  rate was unchanged, as expected.
- After the pull-up change, 2026-11-20: `teensy40_imudiag_const` 300 s, 143964 reads, 0 failed, 0 corrupted, i2c_errors
  0 -> 0; `teensy40_imudiag` 300 s, RUNNING 300/300, i2c_errors 0 -> 0, 0 bad headers, 0 bus recoveries, 0 FIFO
  overflows. The listener scripts and raw outputs were kept in the session scratchpad and are not committed.
- Board restored to main's FW 12 at 20:18: crc32 0x39C78251, 9 rewinds handled, VERIFY OK, "12 -> 12".

## Outcome

Resolved. FW 12 removed the permanent PROBING and the 1 s holes (RUNNING 330 of 330 frames despite 23 errors); the
2.2 kOhm pull-ups removed the corruption itself (0 errors, 0 corrupted reads in 300 s runs at 1 MHz). Residual: no
longer soak has been run with 48 V on and the robot Armed, where the fan, the Leg and Hand motors and CAN traffic are
live. Every earlier run had 48 V off.

## Withdrawn claims

- "Pull-ups are ruled out: 3.9 kOhm is a normal value and 400 kHz was no better than 1 MHz." Withdrawn 2026-11-20. The
  failure is per-edge (slow rising edges leave a window for a glitch to register as a spurious stop or start), so
  slowing the clock does not help and stiffer pull-ups do. See Discussion.

## Open Questions

- Why was the bench clean with the same cable and the same 3.9 kOhm? (Diagnosis, unexplained.)
- Why did the FW 12 baseline show 3 FIFO overflows (0 -> 3 in 330 s)?
- Why were the 3422 corrupted reads at 100 kHz nearly all single-byte, unlike the 0xFF tails at 1 MHz and 400 kHz?
- Does the bus stay clean in a soak with 48 V on and the robot Armed? (The residual of this entry.)
- Do I2C errors need to be visible to the operator? Follow-up: expose i2c_errors and the related counters on the ROS
  status topic.
- Land the diagnostic branch `imu-i2c-diag2` (commit 8d71c392, unpushed) or keep it local? Undecided.
- Record the pull-ups in the hardware documentation (done in `plans/active/platform-imu.md` section 3 by this change).

## Related observations

From the Sitting of 2026-11-20 (FW 11's first flight, IMU upload toggle OFF): the tilt request read (+0.0107, +0.0084) rad
at Level, and the Hand fan worked on command and in auto.
