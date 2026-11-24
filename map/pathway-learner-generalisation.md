---
id: PWAY-2
kind: pathway
title: Learner generalisation
status: active
created: '2026-10-24'
updated: '2026-11-24T12:00:00Z'
owners:
- harrison
priority: medium
effort: xl
hardware: robot
owner: hands
radius: subsystem
area: learner
blocked_by: []
links: []
destination: A global plant model with a physics-keyed residual replaces the local-linear Memory and the offline Admissible box, so any Site, Apex, Dwell or yaw can be commanded and changed mid-Pattern.
---

Live plan: `plans/active/learner-efficacy.md` (written 2026-11-24 after the learner-efficacy review). Its first step is E0, the unsettled-ball test from existing bags.

Earlier plans: `plans/active/learner-generalisation.md` (P3a, P3b, P3c, P1, P2 and P4 complete 2026-11-06; P5 retired 2026-11-24, its items moved to the live plan) and `plans/archived/box-sweep-same-site-fidelity.md` (archived 2026-11-06 as superseded: P3c retired the box).

The Destination still names two things no plan now carries: per-Site xy ("any Site") and a mid-Pattern change on the robot. Narrowing the Destination is an owner decision not yet taken.
