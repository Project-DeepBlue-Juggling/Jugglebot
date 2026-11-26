---
id: FEAT-27
kind: feature
title: 'Kinematic calibration acceptance: the flying frame check, and a Fit that assumes gravity along -z'
status: inbox
created: '2026-10-28'
updated: '2026-11-26T03:18:10Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: hands
radius: subsystem
area: tracking
blocked_by: []
links: []
parent: learner-generalisation
filed_by: claude
night_skip: too-large
night_skip_on: '2026-11-22'
night_skip_note: Rated l, and it needs a Sitting.
---

Kinematic-calibration acceptance was never finished.

- The flying frame check and the tilt-map recapture after Level (kincal steps 6-7) were never done; `_frame_offset_check` is still live (`skill_node.py:605`).
- `flight_fit` assumes gravity along -z, which gives a +62 mm/s^2 y bias.
- The early Fit lands about 9 mm short in x.
- The Pathway's P2 open questions (centre-pull cause, apex drift) are candidate symptoms of an uncorrected frame tilt; a global residual would otherwise absorb a geometry error.

Acceptance (Destination): the kinematic model, the Robot frame and the Tracker's Fit agree closely enough that the flying frame check passes and `_frame_offset_check` is deleted.

Source entries: [logbook/2026-09-23-kinematic-calibration-design-and-fit-tool.md](logbook/2026-09-23-kinematic-calibration-design-and-fit-tool.md), [logbook/2026-09-23-cup-contact-first-sitting.md](logbook/2026-09-23-cup-contact-first-sitting.md), [logbook/2026-09-29-skill-stack-r4-sitting-3-analysis.md](logbook/2026-09-29-skill-stack-r4-sitting-3-analysis.md)

## Gravity in the Mocap frame (2026-11-26)

Offline, 1,139 free flights from six Sittings (2026-09-29 to 2026-11-22). Outputs: temp/probes/feat27-gravity/.

- Gravity in the Mocap world frame is (+43, +83, -9782) mm/s^2 after removing drag: 0.25 deg toward +x and 0.48 deg toward +y, 0.55 deg in total.
- It belongs to the Mocap frame: the vector held while the robot base moved over 150 mm and through 11 deg of yaw. The Robot frame transform is yaw-only and cannot cause it.
- Cost of assuming gravity along -z, Robot frame: an early Fit lands (-8, -21) mm off and the final Fit (-3, -11) mm. With the measured vector both are within 1.5 mm from 0.2 s of flight on.
- The "early Fit lands 9 mm short in x" is this tilt. Drag is separate and small for Hand Throws (under 6 mm/s^2 sideways).
- Proposed, not built, owner to decide: a config value for the horizontal gravity components used by `flight_fit`, a zero vector reproducing today's landing exactly. Re-levelling the Mocap frame would invalidate the Robot frame template and the kinematic calibration.
- Not checked: which Fit age sets the Cup's final position, whether the Learner has absorbed part of the bias, and why the magnitude reads 9782 against 9806.

## Sitting

- setup: level, armed
- why: confirm with a plumb line that the Mocap z axis is 0.55 deg from gravity, then A/B the Fit's measured gravity vector on the robot's own Throws and Catches.
- do:
  - Hang a weighted line with two Mocap markers at least 800 mm apart near the Cup; record 10 s of `/mocap_data` once the swing has damped.
  - Repeat about 500 mm away, near the Ball Butler.
  - Put a digital level on the floor where the Mocap L-frame was placed and read both axes.
  - If both lines pass, fly one Attempt of the usual Pattern with the vector off (the tracker's start-up line shows `gravity_world_xy=(0.0, 0.0)`).
  - Between Attempts run `ros2 param set /ball_tracker_node gravity_world_xy_mmps2 "[43.0, 83.0]"` and fly at least 40 Throws on the same Pattern. Keep the Memory.
  - If the recorded landing error of the robot's own Throws is still above 5 mm after 50 Throws, set `"[0.0, 0.0]"` and stop.
- pass: both lines lean toward +x by 4.4 mrad and toward +y by 8.5 mrad, each within 1.5 mrad. With the vector on, Catches are no worse than with it off, re-aims per Catch fall, and the robot's own recorded landing error returns below 2 mm by Throw 40.
- record: the bag name, both line directions in mrad, the floor level in two axes, whether Mocap was recalibrated since 2026-11-13, the time of the param switch, Catches and re-aims per arm, and the landing error over Throws 1-10, 11-30 and 31-50 after the switch.
- minutes: 30

## Progress

- 2026-11-26: Offline gravity fit done: the Mocap frame is tilted 0.55 deg from gravity and the Fit lands (-8, -21) mm off early, (-3, -11) mm at the end. Fix proposed as a config gravity vector, waiting on the owner (detail below).
- 2026-11-26: Built and on main: config ball_tracking.gravity_world_xy_mmps2, default [0, 0], which is today's Fit bit for bit; switched live by a parameter on the tracker node. The Fit, its landing, the human Throw event and the Kalman fallback use it.
- 2026-11-26: Learner check: Memory rows hold the final-Fit landing, so the learned command absorbed the (-3, -11) mm bias. At the switch the true landings do not move; the recorded error jumps about +11 mm in y and the Learner walks it back in about 30 Throws (model, not flown). Keep the Memory. Ball Butler's aim correction was fitted on measured crossings and is unaffected.
