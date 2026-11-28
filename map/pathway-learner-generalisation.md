---
id: PWAY-2
kind: pathway
title: Learner generalisation
status: active
created: '2026-10-24'
updated: '2026-11-24T14:00:00Z'
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
destination: Fewer Attempts are lost to Throw accuracy. The launch scatter is measured and traced to the release (about 15 mm/s per axis, accepted as the floor of this Hand), the Catch absorbs it, and the Learner on one Memory, with the plant model's lateral law fitted per release kind, keeps carried and Columns Throws from biasing each other, on top of the global plant model and online admissibility delivered 2026-11-06.
---

Live plan: `plans/active/learner-efficacy.md` (written 2026-11-24 after the learner-efficacy review). Its first step is E0, the unsettled-ball test from existing bags.

Earlier plans: `plans/archived/learner-generalisation.md` (P3a, P3b, P3c, P1, P2 and P4 complete 2026-11-06; P5 retired and the plan archived 2026-11-24, its open items moved to the live plan) and `plans/archived/box-sweep-same-site-fidelity.md` (archived 2026-11-06 as superseded: P3c retired the box).

The Destination was narrowed on 2026-11-24 (owner) to the live plan's scope. It used to promise that any Site, Apex, Dwell or yaw could be commanded and changed mid-Pattern; per-Site xy and a mid-Pattern change on the robot are no longer scheduled here.

The Destination was narrowed again on 2026-11-28 (owner). It no longer promises bias for new configurations or yaw, and carried Throws only as far as the per-release-kind plant model (BUG-138). Why: the launch scatter is a floor of this Hand (about 15 mm/s per axis, landing sd 14 to 17 mm), so those biases of 9 to 15 mm sit under it. The transit term (BUG-79), yaw (IDEA-12), the shrinkage estimator (FEAT-65) and the IMU study (IDEA-27) stay as Items.
