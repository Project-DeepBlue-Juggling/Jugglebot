---
id: BUG-148
kind: bug
title: Columns P4 nomination returns a different rung under box load (test_columns_at_p4 fails loaded, passes quiet)
status: inbox
created: '2026-11-26'
updated: '2026-11-26T05:49:09Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: unknown
area: skill-stack
blocked_by: []
links: []
filed_by: claude
---

`tests/motion/test_skills_nominate.py::test_columns_at_p4_plans_every_segment_and_matches_the_p3a_probe_cell` fails when the box is heavily loaded and passes in every gate on a quieter box.

## Findings

- Under load it returns a worst rung of (5, 0.1, 'jerk') where the test pins (5, -0.2, 'jerk'). Seen 2026-11-25 (load 15 to 18) and twice on 2026-11-26 (load about 23), on main as well as on branches.
- It passed in the full gates of 2026-11-25 14:58 and 2026-11-26 10:47, 12:18, 14:19 and 16:31.
- A planning result should not depend on wall-clock load. Cause not investigated; a time-budgeted search inside the nomination is the first suspect (unchecked).

## Wanted

- Find what makes the nominated rung load-dependent. If a wall-clock budget cuts the search short, either make the budget deterministic in the test or report the cut in the nomination.
