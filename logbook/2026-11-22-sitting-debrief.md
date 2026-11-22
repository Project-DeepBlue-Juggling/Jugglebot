---
title: "Robot Sitting 2026-11-22 debrief: Platform IMU mount fitted (gravity within 0.013 deg, board tilted 3 deg about X), 2 kHz clean, 8/8 caught at both rates; a 340 ms bridge freeze latched SETPOINT_STALE (BUG-129); the held Active pose is 0.6 deg off Level (BUG-130)"
type: investigation
date: 2026-11-22
status: complete
phase: "Sitting debrief"
related_plan: "platform-imu.md"
related_issues: []
files_changed:
  - logbook/2026-11-22-sitting-debrief.md
  - logbook/INDEX.md
  - map/sheets/2026-11-22-robot.results.yaml
  - map/idea-platform-accelerometer.md
  - map/idea-minimise-the-platform-imu-upload-time-after-an-attempt.md
  - map/bug-teensy-bridge-node-freezes-for-about-340-ms-once-in-some.md
  - map/bug-held-active-pose-is-0-6-degrees-off-level-the-levelling.md
  - plans/active/platform-imu.md
  - docs/records.md
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/ros/test_teensy_bridge_node_gc_probe.py
  - tests/ros/test_trajectory_node_rt_param.py
---

## Summary

One sheet (`2026-11-22-robot`, IDEA-20 only), one launch (21:03 to 21:17, can-bridge v47, Platform Teensy v13),
launch log `temp/logs/launch_2026-11-22_1.log`, bag `~/Desktop/rosbags/2026-11-22_21-03-42`. IDEA-20's criterion
failed literally on its upload-speed clause only; the owner flipped it to pass and that clause moves to IDEA-25.
No held branch, nothing merged. Filed BUG-129 and BUG-130. The analysis write-ups, figures and the fit script are
kept in `temp/debrief/2026-11-22/` (stale.md, mountfit.md, uploads.md).

## Big picture

The Platform IMU now has a measured orientation. The mount fit reproduces gravity within 0.013 degrees at Level
and at both 5 degree Tilts; the CAD axis signs are right and the board sits tilted 3 degrees about Platform X. The
honest accuracy is about 0.4 degrees in tilt and 0.3 degrees in yaw, because three gravity directions 5 degrees
apart fit easily and the Level reference itself is uncertain by the 0.38 degree trim.

The 2 kHz rate works in a real Attempt (no errors, 8 of 8 caught, the run-time switch behaved), and it is not
needed: the Catch response is at 78 to 80 Hz with smaller peaks near 31 and 50 to 60 Hz, all far below 500 Hz.
1 kHz stays the working rate.

Upload time improved but not to target. Whole-ring uploads Armed took about 60 s against 79 to 84 s two days
earlier. The faster pace gained nothing per sample, because frames lost while Armed are re-read and that eats the
gain. IDEA-25 carries this.

Two things went wrong and neither was the IMU. A one-off 340 ms freeze of the bridge node latched the
SETPOINT_STALE guard while Armed; the upload request that seemed to cause it arrived after the freeze. The guard
response was as designed, but the same freeze mid-Attempt would end the Attempt (BUG-129, cause unknown). And the
held Active pose turned out to be 0.6 degrees off the levelled orientation (BUG-130).

Next: for IDEA-20 and Rung P5, write the calibration file and decide how an Attempt capture counts as
CRC-checked; for IDEA-25, find where frames are lost while Armed; for BUG-129, instrument the bridge process.

## Results

| Item | step | verdict | measured |
|---|---|---|---|
| IDEA-20 | 15 | pass (owner flipped the literal fail, 2026-11-22 21:55) | calibration clause pass, 2 kHz clause pass, upload-speed clause fail |

Steps: 4 skipped (LEVELLING does not hold; the Level window was taken from the Jog Home hold instead), 7 hit the
guard latch and was repeated in STANDBY, all others done.

Uploads (all Armed; operator uploads in STANDBY, attempt_end in TRAJECTORY):

| capture | what | samples | time | duplicates | READs | CRC |
|---|---|---|---|---|---|---|
| 572D | whole ring, gap 80 | 32000 | 59.7 s | 24 | 368 | ok |
| 572E | whole ring after the Tilts, gap 80 | 32000 | 63.3 s | 1694 | 413 | ok |
| 572F | gap 63 | 21896 | 45.6 s | 23 | 466 | ok |
| 5730 | 8-Throw oval, 1 kHz | 21101 | 48.1 s | 2132 | 369 | not checked (span) |
| 5731 | 8-Throw oval, 2 kHz | 29825 | 71.7 s | 2499 | 636 | not checked (span) |

Mount (IMU to Platform), Wahba fit over Level, Rx +5 and Ry +5:

```
R_PI = [[-0.00257 -0.99993  0.01128]
        [ 0.05287 -0.01140 -0.99854]
        [ 0.99860 -0.00197  0.05290]]
```

Deviation from the CAD nominal (IMU x = Platform +z, y = -x, z = -y): (-3.03, -0.13, +0.65) degrees about
Platform x, y, z. Residuals 0.007, 0.011, 0.013 degrees; leave-one-out at most 0.032. The IMU read the Tilts as
4.998 and 5.005 degrees (Mocap 5.03). The lever arm could not be fitted from the Jog moves (R squared 0.13).

## Discussion

**The guard latch was not the upload.** The operator saw the latch at the instant of the upload call, and the
sheet step invited that reading. The bag refutes it: every bridge topic stopped from 21:05:33.565 to 33.907
(`/robot_state` gap 342.7 ms) and the request was handled 57 ms after the freeze ended, with 0 READs sent. A
garbage-collection pause is also ruled out (0.45 ms in that window), as are the mode and the code new this week.

**The first-call trigger is a hypothesis that did not reproduce.** Four such freezes exist in bags since
2026-11-18, at most one per launch, and in the two with a known trigger the freeze ended just before the first
command-line call to the bridge. A no-power launch at 21:48 with three spaced calls showed no gap over 42.5 ms
(bag `2026-11-22_21-48-38`). So the trigger is unconfirmed and BUG-129 is filed with diagnosis unknown. The 250 ms
limit and the lane covers were deliberately not touched: the guard did its job.

**"Level equals Active" was wrong.** The sheet's Level capture was skipped on that assumption. Mocap, the IMU and
the Leg commands agree that the held Active pose is uncorrected by 0.6 degrees, while Jog targets are corrected.
Treating Active as Level leaves fit residuals of 0.13 to 0.34 degrees, about twenty times the noise. The fit
therefore uses the 6 s Jog Home hold as Level, which the owner accepted.

**The upload-speed clause did not belong on IDEA-20.** It failed (1694 duplicates against a limit of 1000; gap 63
at about 67 s per full ring against a 41 s target) while everything the IMU itself had to prove passed. The owner
moved it to IDEA-25 instead of holding the IMU Item open on it.

**Not explained:** why two consecutive gap 80 uploads had 24 and 1694 duplicates; where the frames are lost
while Armed (723 in the 2 kHz Attempt upload against 5 at rest).

## Fix

GC_PROBE lines are logged at DEBUG instead of INFO in `teensy_bridge_node` and `trajectory_node` (owner request):
both node loggers run at DEBUG and the launch console hides DEBUG, so the lines stay in `launch.log` and the bag
and leave the screen. A pause over the warn threshold still prints as a WARN. `tools/probes/sched_probe.py` parses
the line at any level.

## Verification

- 2026-11-22, `pytest tests/ros/test_teensy_bridge_node_gc_probe.py tests/ros/test_trajectory_node_rt_param.py tests/motion/test_gc_probe.py tests/sim/test_sched_probe.py -q`: 24 passed.
- 2026-11-22, `python tools/trapped_ball_finder.py ~/Desktop/rosbags/2026-11-22_21-03-42`: no Hand stalls.
- The gate result is in the commit message.
