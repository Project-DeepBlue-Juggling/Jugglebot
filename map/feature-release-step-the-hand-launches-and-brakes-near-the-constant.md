---
id: JB-144
kind: feature
title: 'Release step: the Hand launches and brakes near the constant-acceleration ideal (C-HAND-5)'
status: inbox
created: '2026-11-02'
updated: '2026-11-02T06:18:01Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: normative
area: hand
blocked_by: []
links: []
filed_by: jugglebot-3d
---

The cup planner pinned the cup acceleration to g at every release knot, so on the 25 ms grid the launch ramped from +A to -g over its last knot and the brake from -g to -B over its first: launch and brake peaks ran 1.8x and 1.6-1.7x the constant-acceleration ideal (JB-114 R0).

C-HAND-5 lets the acceleration STEP at a release knot, along the take-off axis (a slider step; the Platform stays continuous). The gate admits it only at a declared release knot and only downward. Owner on JB-142: "I lean (1). I'm happy with a bounded exception to the general C2 continuity"; bumper pinned for later.

Offline (release 860, Hand 4500): Apex 1.05 peak 3484 -> 2362 rev/s^2; Apex 1.42 refused (4859) -> planned at 3138 (about 48 A launch with the Ball, 38 A brake). Logbook 2026-11-02-hand-release-step.

Box re-swept (100 cells): pose Columns cells added at Apex 1.00-1.40, all admissible; pose cells other than Hop unchanged (Hop moved by selection, JB-143). The LEVEL planner's oval narrowed (Dwell 0.45 lost two cells to LIMIT_JERK); fly pose.

Watch at the first Sitting: the Learner's Memory was built on the continuous shape (expect a landing bias at first); the Hand torque feedforward (K 0.7) steps about 0.2 N m at release.

## Sitting

- setup: armed, bb-fed
- why: confirm the stepped release flies on the robot and that the Columns Apex can climb past 1.05 inside the Hand current limit.
- do:
  - Fly Columns, Sep. 125, Apex 0.95, Throws 20.
  - Fly Columns, Sep. 125, Apex 1.05, Throws 20.
  - Fly Columns, Sep. 125, Apex 1.15, Throws 20.
  - Fly Columns, Sep. 125, Apex 1.25, Throws 20.
  - Fly Columns, Sep. 125, Apex 1.35, Throws 20.
  - Stop the ladder at the first rung with Hand iq over 54 A, a MAX_DEVIATION or stream STOP, a LANE_REFUSED, or an END on any HAND_LIMIT code.
- pass: the 1.05 rung catches at least 18 of 20 with no stop condition, and every flown rung's peak Hand iq stays at or under 54 A.
- record: per rung the catches, the peak Hand iq and planned peak Hand acceleration (install lines), the Ball 1 / Ball 2 landing bias, and the highest rung flown clean.
- minutes: 35

## Sitting result (2026-11-02)

- verdict: pass
- notes: All heights worked well! The first attempt at 1.35 m led to a hand motor SPINOUT error, which was interesting and unexpected.
