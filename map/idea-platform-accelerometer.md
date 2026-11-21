---
id: IDEA-20
kind: idea
title: Platform Accelerometer?
status: in_progress
created: '2026-11-08'
updated: '2026-11-20T12:38:38Z'
owners:
- claude
priority: medium
effort: m
hardware: robot
owner: hands
radius: subsystem
area: platform-teensy
blocked_by: []
links:
- plans/active/platform-imu.md
- logbook/2026-11-20-sitting-debrief-robot-2.md
filed_by: harrison
night_skip: owner-session
night_skip_on: '2026-11-22'
night_skip_note: Rung P4 passed 2026-11-20; Rung P5 (mount calibration) is Sitting work in a day Session.
---

Let's discuss the possible advantages that we might receive as a result of adding an accelerometer to the 2020 extrusion on which Jugglebot's hand's linear rail is mounted. Might this help with our 'input shaping' work, by allowing us to capture high resolution data on Platform acceleration? I imagine we have the Teensy pins available for this? What specific sensor might be best to use?

## Feasibility (2026-11-09)

Read from `Teensy_code_platform.ino`, with no sensor datasheet opened tonight, so the part figures below are from memory and need checking.

- **Pins.** The Platform Teensy already runs the SCL3300 inclinometer on SPI (pins 10 to 13). A second SPI sensor needs only one more chip-select pin. The Hand fan's pin is checked against those same pins at compile time.
- **What it would add.** A direct Platform acceleration signal at about 1 kHz or more. Mocap gives position, so acceleration comes from differentiating it twice, which is where high-frequency content is lost. It would show ringing after a Throw or a Banking move, which is what input shaping needs.
- **Existing sensor.** The SCL3300 is an accelerometer too, but its filter is tens of Hz (from memory), too slow for ringing. One bench capture of it would confirm that cheaply.
- **Sensor candidates.** ICM-42688-P: cheap, SPI, very fast, 6-axis. ADXL355: lower noise, 3-axis, slower, dearer.
- **Effort m, hardware robot.** A board on the 2020 extrusion, a cable, firmware, and a capture path.
- **Risk.** Streaming 1 kHz over CAN adds load on a bus that carries the Attempt, so a capture buffered on the Teensy and uploaded after the Attempt is the safer first version. The cable also rides a moving Platform.

## Options (2026-11-09)

1. Mount an ICM-42688-P, buffer a few seconds on the Platform Teensy and upload after the Attempt. No added CAN load in flight.
2. The same sensor, streamed live at a reduced rate over CAN, so a signal is available to future control. Needs a bus-load check first.
3. An ADXL355 instead, if noise floor matters more than bandwidth.
4. Read the SCL3300 at its fastest setting first and decide after that capture. No new hardware.

Recommendation: 4 as a quick look, then 1. A buffered capture answers the input-shaping question without touching the bus during an Attempt.

## Decision (2026-11-11)

From TKT-54. You ordered an ICM-42688-P breakout, and it is on its way. You want option 2 together with option 1: a low-rate live stream during the Attempt, plus the full-rate data buffered on the Platform Teensy and uploaded after the Attempt.

- What comes next is for a day Session, once the board is here. It covers the Platform Teensy firmware (SPI read, a ring buffer, the upload after the Attempt) and a protocol addition for the live rate.
- The live stream needs a CAN bus-load check before it is sized.
- The board needs a mount on the 2020 extrusion beside the Hand's rail.
- The Night Shift does not build Platform Teensy firmware.

## Bench proof of concept (2026-11-18)

Working at `tools/probes/imu_bench/` (commit afdf0e91, logbook `2026-11-18-imu-bench-proof-of-concept`).

- **Wiring.** Teensy 4.0 and ICM-42688-P over I2C, chosen over the SPI this Item first assumed; Stemma QT to the Platform Teensy pins 18 and 19.
- **Driver.** Chip-FIFO driver, 0 FIFO overflows and 0 I2C errors at 1 and 2 kHz; sensor rate 999.97 Hz at 1 kHz.
- **Rest check.** |a| 1.004 g at rest on one face; the other five faces and a 90 degree rotation are still open.
- **Next.** A day Session started 2026-11-18: the question round, calibration options and CAN transport design come before any Platform Teensy firmware.

## Sitting

- setup: level, tilt, armed, bb-fed
- why: Rung P5 of plans/active/platform-imu.md (mount calibration at Level and two Tilts), plus the FW 13 checks Armed: the whole-ring upload at 1250 and 1600 frames/s and a 1 kHz against 2 kHz oval Attempt. Still without a mechanism, so left out: the forced-gap upload and the upload after a Fault.
- do:
  - Launch with Mocap tracking the Platform and the Cup empty (FW 13 is already on the board, no flash). `cd ~/Desktop/Jugglebot/ros_ws && source /opt/ros/foxy/setup.zsh && source install/setup.zsh && ros2 launch jugglebot jugglebot_launch.py cpu_partition:=false 2>&1 | tee -i ~/Desktop/Jugglebot/temp/logs/launch_2026-11-21_1.log`. Wait for Homing; expect "PLATFORM_FW_CHECK: OK - Platform Teensy v13".
  - Hold LEVELLING on the state minimap to confirm; the Platform rises to full Active height. Leave it still for 35 s (the ring holds 32 s), then note the wall-clock time and upload the whole ring at Level with `ros2 service call /platform_imu/upload jugglebot_interfaces/srv/SetString "{data: \"{}\"}"`. Wait until the GUI IMU panel reads "recording" again; touch nothing else meanwhile.
  - Hold ACTIVE:TRAJECTORY to confirm (this arms). Hold still for a 60 s baseline and note its start and end times. Then upload the whole ring Armed with the same service call as above (the FW 13 and hole-fill check at gap 80). Read `grep -n "Platform IMU upload finished" ~/Desktop/Jugglebot/temp/logs/launch_2026-11-21_1.log` and note the line.
  - Tilts, NEEDS A REHEARSAL AT THE LAUNCH (the jog panel was not flown on 2026-11-20): hold ACTIVE:GUI on the minimap; the Jog panel appears. Click the Rx +5 button once (5 degrees about the Platform X), hold still 8 s by the clock, click the Rx centre button (back to 0), click Ry +5 once, hold still 8 s. Moves are profiled by the GUI follower. If the readout snaps back, use +2 instead of +5 and note it.
  - Click the Jog home button (all axes to 0) and, within 12 s of the end of the Y hold, upload the whole ring (same service call); note the time. Hold ACTIVE:TRAJECTORY to confirm.
  - 1600 frames/s trial: `ros2 param set /teensy_bridge_node imu_read_gap_10us 63`, then the same whole-ring upload Armed in the Active pose, and read the new "Platform IMU upload finished" line from the launch log (seconds, duplicates, READs chained, timeouts, CRC). Then `ros2 param set /teensy_bridge_node imu_read_gap_10us 80` and `ros2 param get /teensy_bridge_node imu_read_gap_10us` must read 80.
  - Bring up Ball Butler (Cup empty). `ros2 param set /skill_node plant_id imu-oval-2026-11-21`, then read it back. Fly one Attempt: `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.50, num_cycles: 3}" --feedback`. The upload starts by itself; wait for "recording" and note the finished line. Set plant_id back to jugglebot, hold IDLE, Ctrl-C the launch.
  - 2 kHz, last, so the board always ends on 1 kHz: with the launch down, `cd ~/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_platform` then, alone on a line, `pio run -e teensy40_imu2k -t upload` ("Platform Teensy FW version: 13 -> 13"). Relaunch as in the first step with log name launch_2026-11-21_2.log (Homing runs in full), Level, ACTIVE:TRAJECTORY, Ball Butler, plant_id imu-oval-2026-11-21, and fly the same 3-Throw goal once. Note the finished line.
  - Return to 1 kHz: hold IDLE, Ctrl-C the launch, and on separate lines `cd ~/Desktop/Jugglebot/ros_ws/src/jugglebot/Teensy_code_platform` then `pio run -e teensy40 -t upload` ("Platform Teensy FW version: 13 -> 13"). Do not leave the board on teensy40_imu2k.
- pass:
  - Calibration: the Level and Tilt captures complete and CRC-checked, and (computed at the debrief) the fitted mount rotation reproduces gravity in the Platform frame within 1 degree at Level and both Tilts (1 degree is a proposal, awaiting the owner's confirmation).
  - Armed uploads complete with CRC OK; at gap 80 fewer than 1000 duplicates and about 60 s or less; at gap 63 about 41 s; no bus-load counter worse than the 2026-11-20 baseline.
  - At 2 kHz i2c_errors and fifo_overflows stay 0 through the Attempt, and the capture is complete.
- record:
  - Wall-clock time of each upload, each Tilt hold, each Attempt, and the flash receipts; the Tilt angle used (5 or 2 degrees).
  - GUI IMU panel text at each upload end and the jog readout at each hold. The "upload finished" line is in the launch log, nothing to type.
- minutes: 80

## Sitting result (2026-11-20 23:45)

- verdict: pass
- notes: setpoints_coalesced is now 10 (sheet 2026-11-20-robot-2, step 15) [derived: owner flipped inconclusive to pass. Replug returned to RUNNING at 22:52:41 on the same boot; i2c_errors and fifo_overflows 0 in all 1593 Armed status messages; four captures complete, ring CRC OK. Not met literally: forced gap unflown; Armed uploads 79-84 s with CAN1 at 74-77 %]

## Progress

- 2026-11-09: Night Shift 2026-11-09: feasibility and options written; awaiting the owner's answer
- 2026-11-11: Night Shift 2026-11-11: TKT-54 answered (ICM-42688-P, live low-rate plus buffered upload); decision recorded, re-rated owner hands for a day Session
- 2026-11-18: Bench proof of concept working (ICM-42688-P on a Teensy 4.0, 0 FIFO overflows and 0 I2C errors at 1 and 2 kHz); day Session started, no Platform Teensy firmware before the question round, calibration options and CAN transport design (detail below)
- 2026-11-18: Design round done: decisions agreed with the owner and written up in plans/active/platform-imu.md (phases P1 to P5, buffered capture only, no live stream).
- 2026-11-19: Owner decisions recorded in the plan: uploads run while Armed between Attempts (the Jetson gates; the bridge clamps the READ gap), target under 60 s for the full window, P4 gains a rate ramp.
- 2026-11-19: FW 11 written and audited on branch imu-p2, awaiting the gate and the flash.
- 2026-11-20: Rung P4 at rest done on FW 12 (identities, tilt, fan, status counters, first capture and upload: 32000 samples, CRC OK, |a| 1.0004 g, 56.25 s); open: capture header temperature reads -128, GYRO_ACCEL_CONFIG0 reads 0x00 (ODR/2, ODR/4 code unverified); the Sitting section now lists only the Armed checks (logbook 2026-11-20-platform-imu-status-counters-and-first-capture)
- 2026-11-20: Correction to the line above: the header temperature -128 was a valid 24.03 C (TEMP_DATA counts); the json now carries temp_c_start and temp_c_freeze. The header GYRO_ACCEL_CONFIG0 is the firmware constant, not a chip read-back. Owner decided the UI filter stays at ODR/2 (logbook 2026-11-20-platform-imu-capture-header-temperature).
- 2026-11-20: Rung P4 passed at the Sitting of 2026-11-20 (owner): replug recovered without a power cycle, i2c_errors and fifo_overflows 0 Armed through three oval Attempts, four complete captures. Residue: forced gap unflown, Armed uploads 79-84 s (IDEA-25), rate ramp. Next: Rung P5 mount calibration.
- 2026-11-21: FW 13 flashed and checked at rest (chaining, fixed-schedule pacing, 2 kHz image); the Sitting section now covers Rung P5 calibration (Level plus two Tilts via the Jog panel), the 1600 frames/s Armed trial, the first Armed FW 13 uploads and a 1 kHz against 2 kHz oval Attempt.
