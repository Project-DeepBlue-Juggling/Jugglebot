---
id: FEAT-27
kind: feature
title: 'Kinematic calibration acceptance: the flying frame check, and a Fit that assumes gravity along -z'
status: inbox
created: '2026-10-28'
updated: '2026-11-26T01:40:05Z'
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

- setup: level
- why: confirm with a plumb line, with no ball dynamics, that the Mocap z axis is 0.55 deg from gravity before the Fit takes a measured gravity vector.
- do:
  - Hang a weighted line with two Mocap markers at least 800 mm apart near the Cup; record 10 s of `/mocap_data` once the swing has damped.
  - Repeat about 500 mm away, near the Ball Butler.
  - Put a digital level on the floor where the Mocap L-frame was placed and read both axes.
- pass: both lines lean toward +x by 4.4 mrad and toward +y by 8.5 mrad, each within 1.5 mrad.
- record: the bag name, both line directions in mrad, the floor level in two axes, and whether the Mocap system was recalibrated since 2026-11-13.
- minutes: 10

## Progress

- 2026-11-26: Offline gravity fit done: the Mocap frame is tilted 0.55 deg from gravity and the Fit lands (-8, -21) mm off early, (-3, -11) mm at the end. Fix proposed as a config gravity vector, waiting on the owner (detail below).
