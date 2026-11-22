---
id: IDEA-20
kind: idea
title: Platform Accelerometer?
status: in_progress
created: '2026-11-08'
updated: '2026-11-22T07:26:42Z'
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
- why: Rung P5 of plans/active/platform-imu.md (mount calibration at Level and two Tilts), plus the FW 13 checks Armed: the whole-ring upload at 1250 and 1600 frames/s, and the same 8-Throw oval Attempt at 1 kHz and at 2 kHz. One launch, no flash: can-bridge v47 switches the capture rate at run time. Still without a mechanism, so left out: the forced-gap upload and the upload after a Fault.
- do:
  - Launch with Mocap tracking the Platform and the Cup empty. `cd ~/Desktop/Jugglebot/ros_ws && source /opt/ros/foxy/setup.zsh && source install/setup.zsh && ros2 launch jugglebot jugglebot_launch.py cpu_partition:=false 2>&1 | tee -i ~/Desktop/Jugglebot/temp/logs/launch_2026-11-22_1.log`. Wait for Homing; expect "BRIDGE_FW_CHECK: OK — can-bridge v47" and "PLATFORM_FW_CHECK: OK — Platform Teensy v13".
  - Hold LEVELLING on the state minimap to confirm; the Platform rises to full Active height. Leave it still for 35 s (the ring holds 32 s), then note the wall-clock time and upload the whole ring at Level with `ros2 service call /platform_imu/upload jugglebot_interfaces/srv/SetString "{data: \"{}\"}"`. Wait until the GUI IMU panel reads "recording" again; touch nothing else meanwhile.
  - Rehearse the Attempt (moves nothing): `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'oval_1ball_fed,apex_m=1.00,separation_mm=150.0,dwell_s=0.50'}"`; it must report no refusal.
  - Hold ACTIVE:TRAJECTORY to confirm (this arms). Hold still for a 60 s baseline and note its start and end times. Then upload the whole ring Armed with the same upload call (the FW 13 and hole-fill check at gap 80). Read `grep -n "Platform IMU upload finished" ~/Desktop/Jugglebot/temp/logs/launch_2026-11-22_1.log` and note the line.
  - Tilts (first rotation jog on the robot; the follower was checked offline, 5 degrees in 0.35 s, then holds): hold ACTIVE:GUI on the minimap. On the Jog panel's Rx row click +5 ONCE (each click adds 5 degrees). The WARN "follower input loss — installed a graceful stop" 0.4 s later is expected. Check Roll reads 5.0, hold still 8 s. Click the "Rx" label button (back to 0), wait 2 s, click +5 on the Ry row ONCE, check Pitch reads 5.0, hold still 8 s.
  - Click Home in the Jog panel header (all axes to 0) and, within 12 s of the end of the Ry hold, upload the whole ring (same upload call); note the time. Then hold ACTIVE:TRAJECTORY to confirm.
  - 1600 frames/s trial: `ros2 param set /teensy_bridge_node imu_read_gap_10us 63`, then the same whole-ring upload Armed in the Active pose, and read the new "Platform IMU upload finished" line from the launch log. Then `ros2 param set /teensy_bridge_node imu_read_gap_10us 80` and `ros2 param get /teensy_bridge_node imu_read_gap_10us` must read 80.
  - Bring up Ball Butler (Cup empty). `ros2 param set /skill_node plant_id imu-oval-2026-11-22`, then read it back. Fly one Attempt at 1 kHz: `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: oval_1ball_fed, apex_m: 1.00, separation_mm: 150.0, dwell_s: 0.50, num_cycles: 8}" --feedback`. The upload starts by itself; wait for "recording" and note the finished line.
  - 2 kHz: `ros2 service call /platform_imu/set_rate jugglebot_interfaces/srv/SetString "{data: '2000'}"`; the launch log must say "capture rate 2000 Hz". Wait 5 s, fly the same 8-Throw goal once (8 Throws is the most that keeps the Feed inside the 16 s ring), wait for "recording" and note the finished line.
  - Back to 1 kHz: the same set_rate call with '1000'; the launch log must say "capture rate 1000 Hz". `ros2 param set /skill_node plant_id jugglebot` and read it back, hold IDLE, Ctrl-C the launch.
- pass:
  - Calibration: the Level and Tilt captures complete and CRC-checked, and (computed at the debrief) the fitted mount rotation reproduces gravity in the Platform frame within 1 degree at Level and both Tilts (owner-confirmed 2026-11-22).
  - Armed uploads complete with CRC OK; at gap 80 fewer than 1000 duplicates and about 60 s or less; at gap 63 about 41 s; no bus-load counter worse than the 2026-11-20 baseline.
  - At 2 kHz i2c_errors and fifo_overflows stay 0 through the Attempt, the capture is complete, and the board ends the Sitting on 1 kHz.
- record:
  - Wall-clock time of each upload, each Tilt hold and each Attempt; the Roll and Pitch readouts at each hold.
  - GUI IMU panel text at each upload end. The "upload finished" and "capture rate" lines are in the launch log, nothing to type.
- minutes: 50

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
- 2026-11-22: 2026-11-22: can-bridge FW 47 flashed; set_rate switches 1/2 kHz at run time (at rest: 2 kHz whole-ring upload 52.9 s, 0 duplicates, CRC ok; back to 1 kHz). Sitting section rewritten: one launch, no flash, Jog Tilts confirmed offline (5 deg in 0.35 s, holds), 8-Throw oval at 1 and 2 kHz, 1 degree criterion owner-confirmed.
