---
id: JB-98
kind: bug
title: Teensy guard MAX_DEVIATION latched on the hand (+8.4 rev) mid-oval; recover_lane found nothing, re-dispatch rejected WRONG_MODE
status: inbox
created: '2026-10-28'
updated: '2026-10-30T06:12:19Z'
owners: []
priority: high
area: can-bridge
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (JB-76 21:02 Sitting read-out)
---

## What happened

21:02 Sitting 2026-10-28, 21:07:28, oval goal 4 at 29/29 caught:
- FAULT LATCHED MAX_DEVIATION, hand first to cross, dev +8.419 rev; orchestrator ACTIVE -> FAULT, goal ended LANE_REFUSED.
- The follow-up recover_lane reported nothing to do, not latched.
- Re-dispatch 6 s later REJECTED WRONG_MODE (mode STANDBY after the fault) plus 5 go_to_pose requires TRAJECTORY mode errors; the next dispatch flew 50/50.
- Bridge uptime 6.7 h; 10 leg heartbeat dropouts and 2 LATENCY MONITOR CLAMP_DUTY 0.27 warnings in the same log.

## Open questions

Two questions:
- Why did the hand setpoint-vs-encoder deviation reach 8.4 rev (a stroke commanded while the hand was held, or a stale hand setpoint?).
- Why the recovery path disagreed with the guard about what was latched.

## Three more trips (2026-10-28)

22:26 Sitting 2026-10-28: three more hand MAX_DEVIATION trips, all explained by JB-95.
- Mechanism: FIFO worker stalled the stream past the 70 ms cover; firmware STOP, refused resume, plan walked away from the held hand (deviation = plan minus held hand, sign = plan direction).
- The hand tracked its held command to 0.01 rev in every case; bridge state, thermal, jammed ball ruled out.
- The 21:02 trip (+8.4 rev, upward stroke start) is NOT explained: no Jetson-side gap in that bag; the firmware sched_expired/superseded/stale counters (leg_interp.cpp:403-405) reach only the serial console.
- Next step for this item: expose those counters in the Diagnostic/link_status so a refused resume can be attributed from the bag.
- Also: recover_lane 'nothing to do, not latched' after an E-stop is the guard's output suppression replacing the lane latch 0.1 s later, a consequence not a disagreement (consider saying so in that log line).

## Reframe (2026-10-30)

2026-10-30 reframe (P3 re-gate Sitting 16:17, bag 2026-10-30_16-17-11):
- The O5 pose 0.45 goal ended at throw 41 by ONE 80 ms emitter wake gap during a 125.7 ms wall / 101.5 ms CPU CATCH solve (93 involuntary context switches) under the default CFS worker.
- Result: firmware scheduler STOP, LANE_REFUSED, hand MAX_DEVIATION +6.56 rev, recovery failed.
- Same signature as the JB-95 FIFO stall, so it very likely explains the unexplained 21:02 trip too.
- JB-98 is now: the setpoint stream has a 70 ms firmware STOP cliff and the loaded box can stall past it.
- Stays open.

## Progress

- 2026-10-28: 22:26 Sitting 2026-10-28: three more hand MAX_DEVIATION trips, all explained by JB-95 (detail in the section above)
- 2026-10-30: 2026-10-30 reframe (P3 re-gate Sitting 16:17, bag 2026-10-30_16-17-11): the O5 pose 0.45 goal ended at throw 41 by ONE 80 ms emitter wake gap (detail in the section above)
