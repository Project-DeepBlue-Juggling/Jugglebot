---
id: TKT-82
kind: ticket
title: 'BUG-94 held on night-shift/2026-11-16-BUG-94: add the three test gaps before the Sitting, fly it as is, or drop?'
status: open
created: '2026-11-17'
updated: '2026-11-16T19:00:13Z'
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

BUG-94's branch night-shift/2026-11-16-BUG-94 now has the 0.3 rad yaw-offset test you chose on TKT-75. It computes the release point by hand, and it failed with the offset doubled and with it dropped. The branch still waits for its Sitting (branch line set). The audit found three more test gaps, all in tests, none in the source.

## Options

1. **Add the three tests before the Sitting:** pin the fallback to the origin when `bb_release_state` raises (it warns once and still announces); tighten the bearing tolerance (0.15 rad) by checking against an independent solver call; and raise the origin-gap floor from 50 mm to about 150 mm (the gap is about 179 mm). The next night builds them and re-gates; the branch then waits for the Sitting as now.
2. **Fly it as is:** the Sitting proves the announcement on the robot, and the gaps stay.
3. **Drop the branch.**

## What each answer unblocks

- 1: the next night builds the tests; then the Sitting row.
- 2: the Sitting row only.
- 3: BUG-94 goes back to open.

Recommendation: 2. The fallback test (option 1's first item) would be worth adding, but none of the three gaps changes what the Sitting will see, and the branch is ready to fly.
