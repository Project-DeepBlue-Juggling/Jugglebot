---
id: TKT-32
kind: ticket
title: 'BUG-58 setpoint stream starved past the 70 ms cover: probe first, a CPU partition, a 4-knot frame, or a bridge-side cliff reaction?'
status: complete
created: '2026-11-03'
updated: '2026-11-02T22:53:39Z'
owners: []
priority: high
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms
type: question
---

BUG-58 records that every forward gap over the firmware's 70 ms cover while streaming ended in LANE_REFUSED: 3 of 3 on 2026-11-02, one with Hand MAX_DEVIATION +9.9 rev. Each sat in a burst of CATCH solves whose wall time far exceeds their CPU time. The logs cannot tell CPU preemption from a whole-process Python pause.

Night Shift 2026-11-03 put the body's next steps into `## Options (2026-11-03)` on the Item.

## Options

1. Probe first: GC timing and `/proc/<pid>/sched` switch counts around CATCH solves, over one long Columns goal (no behaviour change).
2. A CPU partition: the solver worker on cores 4-5, the stream nodes on 0-3.
3. A 4-knot setpoint frame (120 ms cover): a PROTOCOL bump and a flash.
4. A bridge-side cliff reaction on a gap over 70 ms: safety-bearing, needs an offline replay first.

Recommendation: 1, then 2 or a GC fix on its result.

## What each answer unblocks

- 1: an instrumentation commit in the owner's Session and a robot Sitting row (one long Columns goal).
- 2: a launch change and a Sitting.
- 3 or 4: a firmware or bridge design unit, safety-bearing.

## Answer

Agreed. Let's wait on making any changes until we get the results from the probe. If this requires a dedicated Sitting, write it up into the Next Sitting list
