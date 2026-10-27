---
id: JB-76
kind: bug
title: 'SPLICE_TOO_LATE ends half the goals: CATCH solve tail now crosses the 250 ms deadline'
status: inbox
created: '2026-10-27'
updated: '2026-10-27T12:25:16Z'
owners: []
priority: high
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
- plans/archived/handoff-solve-chaining.md
parent: two-ball-oval
filed_by: claude
---

SPLICE_TOO_LATE now ends half the goals of a powered Sitting. The 2026-10-27 lead-clamp P3 Sitting (planner level, dwell 0.50 and 0.65, BB-fed oval and hop) had 10 goals, and 5 ended SPLICE_TOO_LATE at a CATCH. Simple hop (reload) goals were among them. There were two variants: "reply transit outran the wire deadline" and "solver did not answer by the wire deadline".

The solve-time median is unchanged, but the tail now crosses the 250 ms handoff deadline:

| Launch | Solve median | p90 | max | Goals ended SPLICE_TOO_LATE |
|---|---|---|---|---|
| 2026-10-23 11:06 | 123 ms | 182 ms | 219 ms | 0 of 5 |
| 2026-10-24 10:44 | 161 ms | 207 ms | 332 ms | 0 of 14 |
| 2026-10-24 11:34 | 179 ms | 272 ms | 338 ms | 2 of 4 |
| 2026-10-27 22:59 | 168 ms | 276 ms | 380 ms | 5 of 10 |

The figures come from `grep -oE "solve [0-9.]+ ms"` over each launch.log and the skill_node ENDED lines. The 2026-10-27 launch log is /home/jetson/.ros/log/2026-10-27-22-59-39-920154-jetson-2913798/launch.log.

Until now this residue lived only in plans/archived/skill-stack-dress-rehearsals-and-residue.md § 2 (the handoff lead was raised to 17 knots in handoff-solve-chaining.md). It blocks any 30-catch gate, including two-ball-oval § 9's P3 re-gate. First step: find what grew the tail since 2026-10-23, whether Jetson load (the bag ran 345 MB in 9 min, the Wave camera, the map app) or the planner. Look at solve-time against goal and segment type, and at the CPU and load profile during the slow solves.
