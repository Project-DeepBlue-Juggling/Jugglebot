---
id: JB-115
kind: bug
title: 'Launch THROW with the ball aboard may demand more than the 50 A hand current limit from Apex about 0.99 (envelope arithmetic, unverified): check hand current in a columns bag'
status: backlog
created: '2026-10-30'
updated: '2026-10-30T11:52:31Z'
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

## Result of the check (2026-10-30)

- Bags 2026-10-28 14:29 and 21:02: peak hand iq 39.90 / 39.83 A, 0 samples at or above 49 A, the record Attempt's launch 26.3 A. The drive does not clip today.
- Inconclusive on the claim itself: those bags flew release-relative Apex at most 0.935 (planned peak acceleration at most 2797 rev/s2 against the 3509 the claim needs).
- The filing mixed two Apex frames: R0's Apex is release-relative, the goal `apex_m` is above the Catch plane. The 50 A crossing is a Catch-plane Apex of about 1.02, never flown. Extrapolation brackets 50 A (35 A by regression, 56 A by the worst stroke); the B3 model is not refuted.
- Decided at a Sitting at Catch-plane Apex 1.00 to 1.05 with hand `iq_setpoint` captured and a 50.0 A stop rule: decision D-c of `plans/active/pattern-progression.md`. Full write-up: logbook `2026-10-30-jb115-launch-current-bag-check.md`.

## Progress

- 2026-10-30: bag check done, see Result above; no clipping in the flown band, inconclusive at the claimed Apex; back to backlog, diagnosis stays hypothesis.
