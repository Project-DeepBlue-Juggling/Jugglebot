---
id: BUG-119
kind: bug
title: STOP WARN calls a Hand cover lapse under a plan with no Hand track a clean-host firmware STOP
status: inbox
created: '2026-11-18'
updated: '2026-11-18T12:46:53Z'
owners: []
priority: low
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: can-bridge
blocked_by: []
links:
- map/bug-40-hz-setpoint-stream-starved-past-the-firmware-s-70-ms.md
filed_by: claude
---

Frames from a plan with no Hand track (`go_to_pose`, Level, the guard descents) carry `HAS_HAND` clear, so the can-bridge feeds only the legs and the Hand group's scheduler cover runs out about 70 ms after the last Hand frame. That STOP is by design: the Hand stops where it was and the next Hand frame resumes it.

`teensy_bridge_node`'s STOP WARN does not know which group stopped, so it judges these by the host forward cadence and calls them "host forward cadence clean, no knot dropped at the drain, cover expired downstream or at a stamp". That reads as an unexplained firmware-side STOP.

## Evidence

- 2026-11-06 Sittings: 40 of 42 such verdicts had `no_hand` > 0 in their window.
- 2026-11-18, five launches: 43 of 44 had `no_hand` > 0, and all 44 were followed by a Hand STOP record, none by a legs record. Example: launch 12:14, line 1306, a `go_to_pose` over 0.20 s started 115 ms before the WARN.
- Cost: debrief counts of "clean-host STOPs" (BUG-58, BUG-66) mix these in with the STOPs that matter.

## Fix

Name the group from the per-group STOP record sequence numbers in the heartbeat. A Hand-only, non-C1 STOP while frames without a Hand arrive gets its own verdict, so it is no longer called clean-host.
