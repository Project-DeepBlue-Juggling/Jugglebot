---
id: TKT-108
kind: ticket
title: 'BUG-150 held on night-shift/2026-11-28-BUG-150: re-gate it after BUG-154, merge it in your Session, or drop it?'
status: complete
created: '2026-11-28'
updated: '2026-11-28T00:46:40Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
parent: bug-odrive-charts-throw-catch-markers-disappear-for-future-event
type: question
---

BUG-150 is built on night-shift/2026-11-28-BUG-150: in replay, the ODrive charts now draw the selected capture's Throw/Catch markers on both sides of the playhead, the way the IMU charts already did. The audit found nothing open (three narrative fixes applied). With no IMU capture selected, the ODrive charts still show only past markers; the span list is filled only for a selected capture.

The gate held it at (a): the suite failed `test_moved_frame_announcement_landing_is_the_robot_frame_prior`, which fails on main itself (BUG-154), and `test_reload_does_not_reframe_a_live_plan`, which passed when re-run on the quiet box. Neither touches the GUI files this branch changes.

## Options

1. Re-gate it after BUG-154 lands: the next night merges main into the branch, re-gates, and merges on MERGE.
2. Merge it in your Session after a look at the replay charts, since how the markers look needs your eyes anyway.
3. Drop it.

## What each answer unblocks

1 or 2: the markers on the ODrive charts in replay. 3: BUG-150 reopens.

Recommendation: 2 if you are in a Session with the GUI open; otherwise 1.

## Answer

With no IMU capture selected, the ODrive charts should still show ALL markers in the visible window. Please correct this.
