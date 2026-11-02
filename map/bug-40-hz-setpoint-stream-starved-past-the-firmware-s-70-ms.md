---
id: JB-149
kind: bug
title: '40 Hz setpoint stream starved past the firmware''s 70 ms cover during CATCH-solve bursts: 3 LANE_REFUSED ends on 2026-11-02'
status: inbox
created: '2026-11-02'
updated: '2026-11-02T08:49:29Z'
owners: []
priority: high
effort: m
hardware: robot
owner: decision
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- bug-emitter-gap-47ms
- bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid
- feature-solver-worker-on-sched-fifo-take-the-catch-solve-out-of-the
filed_by: claude
---

Filed at the 2026-11-02 Sitting debrief (logbook/2026-11-02-sitting-debrief.md). Supersedes JB-23 (emitter gap) and owns the 70 ms cliff class JB-98 named.

## What happens

All three LANE_REFUSED ends of the two 2026-11-02 Sittings crossed the firmware's 70 ms setpoint-gap cover (2 knots x 25 ms + 20 ms), then the resume was refused:

- 17:00 Sitting, launch-log line 4334 (Columns Apex 1.25, 17/17): teensy_bridge_node starved, forward gap 101.3 ms, bridge loop gap 74.0 ms; Hand guard MAX_DEVIATION +9.899 rev 101 ms after the END.
- 18:13 Sitting, line 1699 (3/3): teensy_bridge_node starved, forward 79.6 ms, loop 70.8 ms; legs lane refused, recovered in 0.7 s.
- 18:13 Sitting, line 4941 (`columns (reload)`, 58/58): trajectory_node emitter stalled 81.7 ms (bridge loop clean, 12.8 ms); Hand guard MAX_DEVIATION -3.308 rev.

Every forward gap over 70 ms while streaming produced a LANE_REFUSED (3 of 3), and every LANE_REFUSED had one. Each stall sits inside a burst of CATCH solves whose wall time is far above their CPU time, with high involuntary-switch counts (102 ms wall against 66 ms CPU, ivcsw 81; 116 against 57). The emitter also exceeded 35 ms in other long goals (40.9 ms, and 63.2 ms in the hop goal) without ending them.

## Why the 2026-10-28 work did not cover it

JB-76 and JB-95 protected the CATCH solve from the rest of the box: nicing the background load took catch refusals from 42 % to 1.3 %. Here the solve is the aggressor and the 40 Hz setpoint stream is the victim. JB-95 already showed this inversion: the worker on SCHED_FIFO stalled the stream 60-96 ms per solve, so it went back to CFS. Nothing yet gives the stream priority over the solve.

## Not yet known

The logs cannot tell CPU preemption from a whole-process Python pause (GC or GIL). In the bridge case every timer in the process froze together.

## Next

1. Probe (no behaviour change): time `gc.callbacks` in teensy_bridge_node and trajectory_node, and sample `/proc/<pid>/sched` involuntary switches around CATCH solves, over one long Columns goal.
2. If it is preemption: a CPU partition, with the solver worker on cores 4-5 and trajectory_node plus teensy_bridge_node on 0-3.
3. Alternatives, each an owner decision:
   - **4-knot setpoint frame (120 ms cover).** Covers all four gaps seen so far (79.6, 82.7, 101.3 and 109 ms on 2026-11-01). Costs a PROTOCOL bump and a firmware flash, and a real stream death would hold 50 ms later.
   - **Bridge-side cliff reaction.** Start lane recovery the moment the forwarder sees a gap over 70 ms. This saves about 200 ms of the heartbeat → skill_node → trajectory_node chain. It is safety-bearing, so it needs an offline replay first.

Unit write-up: the debrief scratchpad lane.md, summarised in the logbook entry.
