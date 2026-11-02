---
id: JB-138
kind: bug
title: Hand iq reached 53.3 A on a 45 deg turned hop launch, over the 50 A watch the high-low row names
status: inbox
created: '2026-11-01'
updated: '2026-11-02T08:49:49Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: hands
radius: subsystem
diagnosis: confirmed
area: hand
blocked_by: []
links:
- map/bug-hand-brake-and-hold-currents-sit-far-below-the-throw.md
filed_by: claude
---

Found at the 2026-11-01 Sitting: hand iq 53.3 A at launch-log L3521 on a 45 deg turned hop launch, over the 50 A watch the high-low row names. The high-low goals peaked at 32.1 and 41.5 A. Related: JB-114, JB-117 (hand current headroom).

## Progress

- 2026-11-02: 2026-11-01: hand_curr_limit_a is now 60 A (hand ODrive current_hard_max 70 A); the 50 A cited here is the limit at filing/measurement time.
- 2026-11-02: Sitting 2026-11-02 (60 A limit): Hand iq at or under 44.7 A in every goal of both Sittings (Apex 0.95-1.35) except -60.4 A at the 1.35 SPINOUT (JB-147). The 45 deg turned hop launch was not re-flown.
