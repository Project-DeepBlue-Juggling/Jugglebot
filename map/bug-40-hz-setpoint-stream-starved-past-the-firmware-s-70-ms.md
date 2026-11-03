---
id: JB-149
kind: bug
title: '40 Hz setpoint stream starved past the firmware''s 70 ms cover during CATCH-solve bursts: 3 LANE_REFUSED ends on 2026-11-02'
status: inbox
created: '2026-11-02'
updated: '2026-11-02T18:15:07Z'
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

## Options (2026-11-03)

1. Probe first (the body's step 1, no behaviour change): time `gc.callbacks` and sample `/proc/<pid>/sched` around CATCH solves over one long Columns goal. This tells CPU preemption from a whole-process pause.
2. A CPU partition: the solver worker on cores 4-5, `trajectory_node` and the bridge node on 0-3. This is right if the probe says preemption.
3. A 4-knot setpoint frame (120 ms cover). It covers every gap seen so far, but costs a PROTOCOL bump and a flash, and delays a real stream death's hold by 50 ms.
4. A bridge-side cliff reaction: start lane recovery on a forward gap over 70 ms. This is safety-bearing and needs an offline replay first.

Recommendation: 1, flown at the next Sitting with the instrumentation, then 2 or a GC fix depending on what it shows. Options 3 and 4 move the cliff instead of removing the stall.

## Sitting 2026-11-03 findings

**Verdict: the host-side starvation hypothesis is refuted for this Sitting, including both LANE_REFUSED ends (2 of 2 with a clean host stream).** Source: launch.log `~/.ros/log/2026-11-03-12-30-00-967281-jetson-3807496/launch.log` and the bag `/link_status` (21,230 msgs) and `/trajectory/status` (10,616 msgs).

- Whole Sitting: emitter gap max 38.9 ms (0 of 10,616 samples above 40 ms); forward gap above 70 ms in exactly one episode (81.6 ms, L6066, loop 63.8 ms); `setpoints_sent` advanced a steady 4 per 100 ms through both LANE_REFUSED STOPs.
- Bag counter `sched_stops` went 0 -> 48: 17 produced a WARN (L2549 ... L14780), 31 were silent (21 with `mpc_active` 0 at faults and disarms, in pairs).
- 10 of the 17 WARNs are the +1 STOP at the pre-level `go_to_pose` at attempt start (forward 32.1-44.7 ms); all 10 ran on with no refusal.

| L | forward / loop (ms) | context | ended an Attempt |
|---|---|---|---|
| 5962 | 41.7 / 21.5 | columns 1.35, throw 10/20, 16 ms after a CATCH solve | yes, LANE_REFUSED 10/10 (L5991), `sched_refused` 0 -> 4 |
| 13842 | 42.1 / 19.0 | turned 90 deg, hop entry, 1.06 s into the REST, 0 throws | yes, LANE_REFUSED 0/0 (L13844), `sched_refused` 25 -> 28 |
| 6068 | 81.6 / 63.8 | columns 1.35, hop entry, REST | no: the only real gap gave a STOP but no refusal |

- The refusal is the second half of a chain: STOP (cause unseen), then the resume is refused by the firmware's discontinuity tolerance (`leg_interp.cpp:626-631`). The cover is measured from the frame stamp, not arrival (`leg_interp.cpp:582`), so a host-to-firmware latency step eats it without any host gap.
- CPU load not supported: 946 CATCH solves (wall median 79.2 ms, p99 124.4, max 161.4) but only 1 of 17 STOPs sits within 20 ms of one.
- The discriminating counters are firmware-side and absent from `/link_status`: `sched_expired` (`udp_protocol.h:269`), superseded, stale and future drops, plus a per-frame `rx_age_us`.

