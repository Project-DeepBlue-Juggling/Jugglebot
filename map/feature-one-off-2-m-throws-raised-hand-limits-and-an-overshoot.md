---
id: FEAT-37
kind: feature
title: 'One-off 2 m Throws: raised Hand limits and an overshoot protection the speed-scaled margin cannot give'
status: inbox
created: '2026-11-02'
updated: '2026-11-03T02:51:12Z'
owners: []
priority: medium
effort: l
hardware: robot
owner: decision
radius: subsystem
area: hand
blocked_by: []
links: []
filed_by: jugglebot-3d
---

Owner goal (2026-11-02): occasional single Throws to about 2 m Apex (not continuous juggling); the owner is confident the hardware can do it.

## Estimates (from the C-HAND-5 models, not probe runs)

A 2 m Apex means releasing at about 6.26 m/s, 192 rev/s of Hand. Launch about 4630 rev/s^2 at kappa 1.28 (about 70 A with the Ball: over the 4500 limit, at the 70 A code limit). Brake to the box top about 4300 rev/s^2 (about 52 A). The per-knot Hand step is about 4.3 rev against the 4.0 rev bound, so the session Hand speed limit must rise from 200 to about 250 rev/s.

## The open problem

The BUG-57 overshoot margin (fitted at 200 rev/s in C-HAND-6) needs 1.5-2 rev of the 4.7 rev above the release at this speed, which pushes the brake past what the drive can do. One-off 2 m Throws need another protection:
- explain and remove the overshoot (the BUG-57 capture);
- the IDEA-17 bumper as a physical stop;
- or a deliberate, supervised one-off risk.
Catching a Ball arriving at about 6.3 m/s is unchecked. Related: FEAT-36, BUG-57, IDEA-17.

## Progress

- 2026-11-03: 2026-11-03 Sitting: narrowed. Apex 1.35-1.50 already ends SPINOUT_DETECTED by the torque-loss class (7 of 9 attempts, see BUG-57); the speed-scaled margin cost 34-37 mm of stroke and did not prevent it. 2 m waits on BUG-57.
