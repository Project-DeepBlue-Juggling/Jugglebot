---
id: JB-115
kind: bug
title: 'Launch THROW with the ball aboard may demand more than the 50 A hand current limit from Apex about 0.99 (envelope arithmetic, unverified): check hand current in a columns bag'
status: backlog
created: '2026-10-30'
updated: '2026-10-30T11:50:57Z'
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

## Progress

- 2026-10-30: 2026-10-30: bag check done (logbook 2026-10-30-jb115-launch-current-bag-check.md). Peak hand iq 39.90 / 39.83 A in the 2026-10-28 14:29 and 21:02 bags, 0 samples at or above 49 A, record launch 26.3 A: the drive does not clip today. The claim is INCONCLUSIVE because those bags flew release-relative Apex at most 0.935 (planned peak acc at most 2797 rev/s2 against the 3509 the claim needs); the filing mixed R0's release-relative Apex with the Catch-plane goal apex_m (the 50 A crossing is a Catch-plane Apex of about 1.02, never flown). Extrapolation brackets 50 A (35 A regression to 56 A worst stroke); B3 not refuted. Decided at a Sitting at Catch-plane Apex 1.00 to 1.05 with hand iq_setpoint captured and a 50.0 A stop rule (plan pattern-progression.md decision D-c). Diagnosis stays hypothesis.
