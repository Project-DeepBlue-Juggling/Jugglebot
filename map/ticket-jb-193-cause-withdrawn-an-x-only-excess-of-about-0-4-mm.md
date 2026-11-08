---
id: TKT-47
kind: ticket
title: 'BUG-80 cause withdrawn, an x-only excess of about 0.4 mm remains: close it, fly the 40-Throw check, or put a marker on the Cup?'
status: complete
created: '2026-11-07'
updated: '2026-11-06T22:55:38Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: learner
blocked_by: []
links: []
parent: bug-the-flyback-over-corrects-a-catch-offset-by-1-2-to-1-4-times
type: question
---

BUG-80's cause is withdrawn: the planner has no flyback gain that is too strong. It returns 1.00 mm per mm of release offset on paper and in an offline replay, and 0.96 from the commanded Tilt in the 15:53 bag. Full tables in `## Findings (2026-11-07)` on BUG-80.

## What is left

The over-correction is real only on Robot x and only in one-ball Throws: lag-1 slope x -0.26 +- 0.07, y +0.04 +- 0.07. About 0.9 mm/s per mrad of ball velocity on x has no measured cause. Removing it all would take x scatter from 18.17 to 17.76 mm.

## Options

1. Close it with no change. A gain would trade x for y, and the effect is about 0.4 mm of 18 mm.
2. Keep it low, and fly the 40-Throw check when a Sitting has room: from rest, offsets of +20 and -20 mm on each axis. It separates a Tilt gain from an effect of the catch-and-rethrow fold.
3. Put a marker on the Cup, which would show the lateral path during a stroke directly. That also serves BUG-79 and BUG-74, which lean on the same lateral picture.

Recommendation: 2, as a rider on a Sitting that is already flying one-ball goals. Do not change the gain.

## What each answer unblocks

- 1: the next night closes BUG-80.
- 2: the next night writes its `## Sitting` section and re-rates it `robot`.
- 3: a hardware step for you, then a read of the next bag.

## Answer

Agreed. Let's go for another Sitting.
