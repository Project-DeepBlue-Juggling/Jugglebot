---
id: IDEA-28
kind: idea
title: Measure QTM's true latency with a physical edge seen by QTM and the bridge
status: inbox
created: '2026-11-23'
updated: '2026-11-23T15:48:54Z'
owners: []
priority: low
effort: m
hardware: robot
owner: hands
radius: cross-cutting
area: tracking
blocked_by: []
links:
- logbook/2026-11-23-bridge-stamp-on-robot-state-and-hand-telemetry.md
filed_by: claude
night_skip: low-priority
night_skip_on: '2026-11-24'
night_skip_note: 'Feasibility noted 2026-11-24: an LED circuit, a bridge input-capture and a flash, then one Sitting.'
---

QTM's exposure-to-packet latency has never been measured. Offline fits against the executed command bound it to about 4 to 13 ms, and a Ball Butler yaw cross-check disagrees with the command route.

## Wanted

- Put one physical edge (an LED or strobe) in view of both QTM and a bridge-stamped input, and read the delay between the two stamps.

## Then

- Decide whether to correct the live Mocap stamp. That moves Tracker landing times, trips the bb_calibration 13 ms lag advisory and invalidates the Ball Butler release constants, so it needs a Ball Butler re-centring Sitting (BUG-134).

## Feasibility (2026-11-24)

- What it would take: an LED (or a strobe) that QTM sees as a marker, driven from a pin whose edge the can-bridge Teensy also stamps on its clock. That is a small circuit, a firmware input-capture and an additive message to export the stamp (a flash), then one short Sitting with the cameras on.
- Effort: m. Hardware: robot (the QTM rig and the bridge). Owner: hands, for the circuit and the capture.
- One risk: QTM may not report a blinking marker on the frame it first lights (marker detection thresholds, frame exposure), so the measured edge can be one frame late; the LED must stay on for several frames and the first lit frame is the one to read.
- Follow-on cost the Item already names: correcting the live Mocap stamp needs a Ball Butler re-centring Sitting.
