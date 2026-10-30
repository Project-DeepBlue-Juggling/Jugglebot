---
id: JB-115
kind: bug
title: 'Launch THROW with the ball aboard may demand more than the 50 A hand current limit from Apex about 0.99 (envelope arithmetic, unverified): check hand current in a columns bag'
status: inbox
created: '2026-10-30'
updated: '2026-10-30T09:56:23Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: hands
radius: subsystem
diagnosis: hypothesis
area: hand
blocked_by: []
links: []
filed_by: claude (jugglebot-f5, R0 study 2026-10-30)
---

Side finding of the R0 brake-shape study 2026-10-30 (scratchpad bumper/r0_brake_shape.md).

## Finding

- Using the throw envelope's own constants, the launch (first THROW from rest) at Apex 1.05 asks for 53.6 A at its peak knot with the ball aboard; the crossing is about Apex 0.99.
- The 3900 rev/s2 hand gate is a no-ball braking number, so it does not see the loaded ascent.
- Columns at 1.05 has flown 49/49, so if true the drive is clipping and tracking through it.

## Check

Hand iq against the planned acceleration in an existing Apex 1.0-1.05 columns bag (2026-10-28 14:33 or 21:04), with tools/probes/hand_stroke_timeline.py. If confirmed, the loaded ascent needs its own gate before any Apex climb (JB-114).
