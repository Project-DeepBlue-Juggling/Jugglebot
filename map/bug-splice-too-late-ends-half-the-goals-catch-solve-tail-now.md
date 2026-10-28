---
id: JB-76
kind: bug
title: 'SPLICE_TOO_LATE ends half the goals: CATCH solve tail now crosses the 250 ms deadline'
status: in_progress
created: '2026-10-27'
updated: '2026-10-28T03:43:22Z'
owners: []
priority: high
blocked_by: []
links:
- plans/archived/skill-stack-dress-rehearsals-and-residue.md
- plans/archived/handoff-solve-chaining.md
- logbook/2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented.md
- logbook/2026-10-28-gui-mocap-copies-cut-rosbridge-load.md
parent: two-ball-oval
filed_by: claude
---

SPLICE_TOO_LATE now ends half the goals of a powered Sitting. The 2026-10-27 lead-clamp P3 Sitting (planner level, dwell 0.50 and 0.65, BB-fed oval and hop) had 10 goals, and 5 ended SPLICE_TOO_LATE at a CATCH. Simple `hop (reload)` goals were among them. There were two variants: "reply transit outran the wire deadline" and "solver did not answer by the wire deadline".

The solve-time median is unchanged, but the tail now crosses the 250 ms handoff deadline:

| Launch | Solve median | p90 | max | Goals ended SPLICE_TOO_LATE |
|---|---|---|---|---|
| 2026-10-23 11:06 | 123 ms | 182 ms | 219 ms | 0 of 5 |
| 2026-10-24 10:44 | 161 ms | 207 ms | 332 ms | 0 of 14 |
| 2026-10-24 11:34 | 179 ms | 272 ms | 338 ms | 2 of 4 |
| 2026-10-27 22:59 | 168 ms | 276 ms | 380 ms | 5 of 10 |

The figures come from `grep -oE "solve [0-9.]+ ms"` over each launch.log and the skill_node ENDED lines. The 2026-10-27 launch log is /home/jetson/.ros/log/2026-10-27-22-59-39-920154-jetson-2913798/launch.log.

Until now this residue lived only in plans/archived/skill-stack-dress-rehearsals-and-residue.md § 2 (the handoff lead was raised to 17 knots in handoff-solve-chaining.md). It blocks any 30-catch gate, including two-ball-oval § 9's P3 re-gate. First step: find what grew the tail since 2026-10-23, whether Jetson load (the bag ran 345 MB in 9 min, the Wave camera, the map app) or the planner. Look at solve-time against goal and segment type, and at the CPU and load profile during the slow solves.

## Progress

- 2026-10-28: 2026-10-28 deep dive: the CATCH solve is ~65 ms intrinsically (sim, worker replay and the first live CATCH of each goal all agree); it runs 2-5x slower only mid-juggle, tracking box load. Ruled out: the worker path, OpenBLAS spin, memory bandwidth, thermals, CPU frequency, uptime. Leading hypothesis: starvation by preemption churn, with the mocap queue overflow as a fellow victim. Worker replies now log cpu / ivcsw / vcsw after solve X ms. Next: fly one Sitting with tools/probes/profile_session.py running, then pin the worker to its own core if starvation is confirmed. Entry: logbook/2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented.md
- 2026-10-28: 2026-10-28 Sitting 09:49 with the recorder: starvation CONFIRMED. Every CATCH did ~65 ms of CPU and took 153 ms p50 / 226 ms p90 of wall with 59 involuntary context switches; 162 of 270 CATCH-class solves over 150 ms wall, zero over 150 ms CPU. Cause: the launch itself runs the box at 5.5 of 6 cores while juggling (rosbridge 86 %, ball_tracker 83 %, teensy_bridge 79 %, skill_node 73 %, mocap 71 %), run queue 13. Mocap queue had zero overflows this time, so the overflow is a fellow victim, not the cause; a Jetson restart is not a fix. Fix 1: nice -n 10 on rosbridge, bag record, spacemouse, ball_butler in the launch. Next: re-fly and scan with tools/probes/solve_starvation_scan.py, then trim tracker/rosbridge load, then worker core isolation.
- 2026-10-28: 2026-10-28 later: fix 1 (nice -n 10 on rosbridge / bag record / spacemouse / ball_butler, commit 0b6ddad7) and load-trim option 1 (GUI takes mocap_node 20 Hz gui/ copies of the mocap stream, rosbridge 102 -> 53 % of a core on the refreshed rosbridge_cpu_probe replica, commit b2e70285) are on main and built. Both UNVERIFIED live: next Sitting runs with tools/probes/profile_session.py and is read with solve_starvation_scan.py; the pass mark is CATCH wall/CPU well under 2.18 and re-aims surviving. Then: tracker BLAS cap test via tracker_bag_replay, py-spy on skill_node + teensy_bridge, worker core isolation last.
- 2026-10-28: Sitting 2026-10-28 14:29 (fix 1 nice + option 1 gui copies live, columns only, 79 throws): re-aim refusals 41 % -> 1.3 % (one SPLICE_TOO_LATE, throw 48/50), CATCH wall p50 153 -> 117 ms, p90 226 -> 160, over-150 ms 60 % -> 13 %, rosbridge 86 -> 56 % of a core, 49/49 columns record. Root cause NOT removed: wall/cpu 2.18 -> 1.97 (pass mark 'well under 2.18' missed), box still 537 of 600 % during goals, ball_tracker_node 148 % during goals with an uncapped BLAS pool is the main nice-0 consumer. Improved, not fixed; next = tracker OPENBLAS_NUM_THREADS=1 test offline, then py-spy skill_node/teensy_bridge, then worker core isolation. Nice values unverifiable (ps snapshot lacked the ni column; recorder fixed).
