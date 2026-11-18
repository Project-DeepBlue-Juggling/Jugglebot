---
id: TKT-85
kind: ticket
title: 'BUG-90 held after failing its Sitting on 2026-11-18: add a hard bound and fly again, take the Hand out of the ZVD lever, or drop it and leave carry_zvd off?'
status: open
created: '2026-11-19'
updated: '2026-11-18T15:54:22Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-carry-zvd-rows-make-the-hand-stop-and-go-at-about-5-hz-in
type: question
---

BUG-90 (the `carry_zvd` rows make the Hand stop and go in flight) is held on night-shift/2026-11-15-BUG-90 with your TKT-69 option 1. It failed its Sitting on 2026-11-18. At Dwell 0.40 the Hand averaged 0.77 turning points per flight, against 2.05 on 2026-11-13 and 0.03 with the flag off. Dwell 0.35 was refused LIMIT_JERK three times at the first learned Catch.

## Options

1. **Revise the branch:** add a hard bound on Hand turning points within the flight window (TKT-69's hard-bound option) on top of option 1, then fly it again.
2. **Take the Hand out of the ZVD lever:** keep `carry_zvd` for the Legs only (TKT-69's other option). This is a new build.
3. **Drop the branch and leave `carry_zvd` off:** the flag-off Hand is already smooth (0.03 turning points per flight).

## What each answer unblocks

- 1 or 2: a day Session or a night builds it on the branch (planner math, so Opus), then it goes back on the Sitting index.
- 3: BUG-90 closes with the flag off, and the branch is left for you to delete.

Recommendation: 3, unless `carry_zvd` is buying something the Legs need. The failed Sitting shows option 1 only halves the stop-and-go, and with the flag off it is gone.
