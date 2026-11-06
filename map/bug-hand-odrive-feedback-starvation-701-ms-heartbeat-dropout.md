---
id: JB-137
kind: bug
title: 'Hand ODrive feedback starvation: 701 ms heartbeat dropout left the encoder 100 ms stale, the 2.0 rev lead clamp pinned the throw to stale+2.0, guard MAX_DEVIATION +3.707 fired correctly (turned-90 at 16/16)'
status: in_progress
created: '2026-11-01'
updated: '2026-11-06T05:58:21Z'
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
- map/bug-max-deviation-latched-on-40-90-ms-stale-hand-telemetry.md
- logbook/2026-11-06-sitting-debrief-2.md
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
- 2026-11-03: 2026-11-03 Sitting: 90 deg turned flown with no GUARD_LATCHED, no SPINOUT, no Hand error (L11704-14714), so untestable here. Related new Item JB-160: MAX_DEVIATION (L2528) fired on 40-90 ms stale Hand telemetry without MOTOR_FB_STALE.
- 2026-11-06: Second occurrence, 2026-11-06 P2 Sitting goal 10: the Hand's encoder frames fell to about 10 Hz for 0.6 s while its current frames kept arriving.
- 2026-11-06: The lead clamp held the launch at the stale position + 2.0 rev (Throw reached Apex 0.32 m), then released a 1.2 rev step: iq 81.3 A over the 80 A hard max, CURRENT_LIMIT_VIOLATION. The Hand coasted about 1 mm past the top stop at 8 rev/s or less.
- 2026-11-06: No firmware guard acts in a 30 to 150 ms feedback gap by design. Cause of the frame loss unknown (selective by message; Leg 0 showed the same cadence). The host now warns per episode (`teensy_link/hand_feedback_watch.py`). Owner decisions open: the Hand clamp anchor (port the Legs' FW 31 dead-reckoned anchor, stop, or slew back) and the 70 / 80 A gap against an 11.3 A overshoot.
- 2026-11-06: Evidence: `temp/probes/sitting_debrief_2026-11-06b/unit_hand.md`.
