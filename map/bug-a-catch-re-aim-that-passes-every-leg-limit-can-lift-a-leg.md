---
id: BUG-163
kind: bug
title: 'A Catch re-aim that passes every Leg limit can lift a Leg off the Base: the gate checks along-Leg motion, not the load at the Base joint'
status: inbox
created: '2026-11-29'
updated: '2026-11-28T14:23:56Z'
owners: []
priority: high
effort: m
hardware: robot
owner: none
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- bug-two-ball-columns-runs-end-on-a-missed-catch-of-ball-2-with
- logbook/2026-11-28-sitting-debrief-evening.md
parent: learner-generalisation
filed_by: claude
---

A Catch re-aim installed late can lift a Leg off its Base joint although it passes every Leg limit: the gate bounds along-Leg motion, and the failure is at the Base joint. Found at the Sitting of 2026-11-28 (evening).

## The event

- BUG-78's 25 mm trial, last Attempt (skill 17): a RESEND re-aim of 30.8 mm (21.5 mm outward, 30.8 mm across, clamped from 51.5) installed 0.567 s before contact.
- Realized Leg peaks 348 mm/s, 5015 mm/s^2, 137k mm/s^3: 58 to 69 % of the limits 600 / 8000 / 200000, so every check passed.
- The Platform rose about 8 mm above its median (184.5 mm) and tilted 4.8 deg within 125 ms; Leg 1 separated from the Base and reseated on the deceleration.
- Near-miss at the default 10 mm: BUG-160's step 7 Attempt 3, last Catch, 5754 mm/s^2, z 179.3 mm, tilt 2.4 deg.

## The gap

`validate_cycle` and the bump gate check only along-Leg velocity, acceleration and jerk on the plan. Nothing checks Leg axial tension, lateral load at the Base joint, or Platform lateral acceleration.

## Benign envelope (BUG-78 at 15 mm, 2026-11-28 morning)

- RESEND up to 34.9 mm at 0.544 s or more before contact; BUMP up to 28.8 mm at 0.265 s or more.
- Realized Leg acceleration at most 4394 mm/s^2, tilt at most 2.9 deg, z within 1 mm.

## Decisions (owner, 2026-11-29)

- Default Catch outward authority becomes 15 mm, and a value above 15 is refused at nomination.
- The Leg acceleration Sitting limit goes from 8000 to 4500 mm/s^2, which sits above the benign maximum of 4394 and below both bad events (5015 and 5754).

## Options not taken

- Cap the combined lateral re-aim vector at 30 mm (today 30 across plus 25 outward allows 39 mm).
- A Mocap lift guard: abort on tilt above 3.5 deg or z more than 3 mm above the median.
- Longer term, a Leg tension check (J^-T W including inertia) in the gate.

## Sitting

- setup: level, armed, bb-fed
- why: the 15 mm default with the 4500 mm/s^2 Leg acceleration limit has to be shown not to lift a Leg; BUG-78's 15 mm trial is this Item's flight.
- do:
  - Fly it with BUG-78's 15 mm trial (see BUG-78's Sitting section), after BUG-160's refit.
  - Before arming: `ros2 service call /skills/nominate jugglebot_interfaces/srv/SetString "{data: 'columns,apex_m=0.95,separation_mm=125,dwell_s=0.27'}"` and read the margins.
  - Fly 8 Columns Attempts (Sep. 125, Apex 0.95, Dwell 0.27, Throws 21) at the default authority.
- pass: no realized Leg acceleration above 4500 mm/s^2 at any Catch and no Leg base joint moves.
- record: caught of thrown and the end code per Attempt, anything heard or seen at the Leg base joints.
- minutes: 15
