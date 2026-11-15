---
id: TKT-75
kind: ticket
title: 'BUG-94 held on night-shift/2026-11-16-BUG-94: add the yaw-offset test before the Sitting, fly it as is, or drop?'
status: open
created: '2026-11-16'
updated: '2026-11-15T19:01:55Z'
owners: []
priority: high
effort: xs
hardware: none
owner: decision
radius: local
area: tracking
blocked_by: []
links: []
parent: bug-ball-butler-throw-announcement-gives-the-tracker-the
type: question
---

BUG-94 is built on night-shift/2026-11-16-BUG-94: Ball Butler's Throw announcement now starts the ball at the release point (`bb_release_state`) instead of the calibrated origin, as the 2026-11-14 offline replay showed it should (117 of 117 Feed flights closer). It is held for a Sitting (the Item's `## Sitting` section, branch line set). The audit found the source correct but one gap in the new test: it uses a zero yaw offset, so a doubled or missing offset would still pass.

## Options

1. **Add the nonzero-offset test, then fly it:** the next night adds a second case with a yaw offset of 0.3 rad, re-gates, and the branch waits for the Sitting as now.
2. **Fly it as is:** the Sitting proves the announcement on the robot's real offset; the test gap stays.
3. **Drop the branch.**

## What each answer unblocks

- 1: the next night builds the test; then the Sitting row.
- 2: the Sitting row only.
- 3: BUG-94 goes back to open.

Recommendation: 1. It is a few lines, and the offset is the one argument the replay cannot catch if a later edit doubles it.
