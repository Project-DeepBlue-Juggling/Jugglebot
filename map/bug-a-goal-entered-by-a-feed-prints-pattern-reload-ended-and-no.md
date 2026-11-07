---
id: JB-190
kind: bug
title: A goal entered by a Feed prints `<pattern> (reload) ENDED` and no `done` line, and `Self-toss` does not print its Site at the start
status: complete
created: '2026-11-06'
updated: '2026-11-06T18:57:44Z'
owners:
- night-shift
priority: low
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief-2.md
- logbook/2026-11-07-jb190-feed-goal-end-line-searchable.md
filed_by: claude
---

Two log gaps found at the debrief of 2026-11-06.

## Findings

1. P2 log L9214: `columns_turned (reload) ENDED (MISSED_CATCH)`; a search for `columns_turned ENDED` or `done` misses it, so the goal looked as if it had no end.
2. The `self_toss started` line prints Apex, dwell and a `sep 250 mm` that does not apply, but not the Site, which is set by `ros2 param set /skill_node site_x_mm / site_y_mm`.
3. At the P2 Sitting seven one-ball goals flew at x = -50 instead of 0 and nothing on the console said so until the first Memory row.

## Wanted

- Print the Site in the started line.
- Make the end line's pattern name searchable.

## Progress

- 2026-11-06: The Site half is fixed 2026-11-06: a one-ball goal's start line now ends with its Site (skill_node, test in tests/ros/test_skill_node.py TestOperatorLines). Still open: the end line of a goal entered by a Feed.
- 2026-11-07: Night Shift 2026-11-07: held on night-shift/2026-11-07-JB-190: gate (b), a probe regex reads the old end line; unblocked by the owner's answer
