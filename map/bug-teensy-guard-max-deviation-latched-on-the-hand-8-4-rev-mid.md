---
id: JB-98
kind: bug
title: Teensy guard MAX_DEVIATION latched on the hand (+8.4 rev) mid-oval; recover_lane found nothing, re-dispatch rejected WRONG_MODE
status: inbox
created: '2026-10-28'
updated: '2026-10-28T10:22:56Z'
owners: []
priority: high
area: can-bridge
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (JB-76 21:02 Sitting read-out)
---

21:02 Sitting 2026-10-28, 21:07:28, oval goal 4 at 29/29 caught: FAULT LATCHED MAX_DEVIATION, hand first to cross, dev +8.419 rev; orchestrator ACTIVE -> FAULT, goal ended LANE_REFUSED. The follow-up recover_lane reported nothing to do, not latched. Re-dispatch 6 s later REJECTED WRONG_MODE (mode STANDBY after the fault) plus 5 go_to_pose requires TRAJECTORY mode errors; the next dispatch flew 50/50. Two questions: why did the hand setpoint-vs-encoder deviation reach 8.4 rev (a stroke commanded while the hand was held, or a stale hand setpoint?), and why the recovery path disagreed with the guard about what was latched. Bridge uptime 6.7 h; 10 leg heartbeat dropouts and 2 LATENCY MONITOR CLAMP_DUTY 0.27 warnings in the same log.
