---
id: JB-137
kind: bug
title: 'Hand ODrive feedback starvation: 701 ms heartbeat dropout left the encoder 100 ms stale, the 2.0 rev lead clamp pinned the throw to stale+2.0, guard MAX_DEVIATION +3.707 fired correctly (turned-90 at 16/16)'
status: inbox
created: '2026-11-01'
updated: '2026-11-02T08:49:48Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: normative
diagnosis: hypothesis
area: can-bridge
blocked_by: []
links:
- map/bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid.md
filed_by: claude
---

Found at the 2026-11-01 Sitting, turned-90 goal at 16/16 (launch log L4695): a 701 ms hand heartbeat dropout, then GUARD_LATCHED.

## Findings

- The hand encoder was 100 ms stale during the dropout.
- The 2.0 rev lead clamp pinned the throw to stale encoder + 2.0 rev.
- The guard MAX_DEVIATION +3.707 rev fired correctly: the real deviation was real.
- Hypothesis: node 6 loses CAN arbitration to leg traffic. Not tested.
- Different mechanism from JB-98's high-low trip (a bridge stall).

## Wanted

- Encoder age in the census, and a look at hand-node frame loss under leg load.

## Progress

- 2026-11-02: Sitting 2026-11-02: not involved in any end (hb_stale and lead_clamp masks 0 before all 3 LANE_REFUSED). Sub-hypothesis 'node 6 loses CAN arbitration' WITHDRAWN: 7 hand dropouts all had a fresh encoder (<=24 ms) while the legs reached 99 ms. The 11-01 100 ms stale encoder did not recur: cannot tell.
