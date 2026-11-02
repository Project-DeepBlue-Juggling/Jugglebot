---
id: JB-138
kind: bug
title: Hand iq reached 53.3 A on a 45 deg turned hop launch, over the 50 A watch the high-low row names
status: inbox
created: '2026-11-01'
updated: '2026-11-01T17:51:40Z'
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
