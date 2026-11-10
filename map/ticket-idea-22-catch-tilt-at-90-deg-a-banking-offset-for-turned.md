---
id: TKT-62
kind: ticket
title: 'IDEA-22 Catch tilt at 90 deg: a Banking offset for turned Columns, a fixed Catch attitude per Site, measure the commanded tilt in the bag first, or fix BUG-63''s landing first?'
status: complete
created: '2026-11-11'
updated: '2026-11-10T21:45:12Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: idea-turned-columns-at-90-deg-tilt-the-platform-back-x-at-the
type: question
---

At 90 deg turned Columns, Catches keep missing on the Platform top (4 Attempts on 2026-11-10, all MISSED_CATCH). You suggested tilting the Platform back (+x) at the Catch. Every Catch is already banked toward the ball's arrival, up to 12 deg, and only the Feed Catch is pinned Level.

## Options

1. **A constant Banking offset for turned Columns:** a Pattern parameter added to the auto-banked Catch tilt, checked offline by a nominate probe at 90 deg, then a Sitting row.
2. **A fixed Catch attitude per Site at 90 deg:** simpler, but it gives up the arrival-matched seat.
3. **Measure first, no code:** the Night Shift reads the 2026-11-10 bag for the commanded Catch tilt at 90 and 45 deg, whether the 12 deg clamp binds, and where each miss struck.
4. **Fix the landing first:** the misses were Ball 2's first Catch after the Feed, landing 14 to 47 mm outward, which is BUG-63.

Recommendation: 3, then 1.

## What each answer unblocks

- 1 or 2: the next night builds the parameter on a branch, with the nominate probe result, held for a Sitting.
- 3: the next night reads the bag and writes Findings on IDEA-22.
- 4: IDEA-22 waits on BUG-63's Sitting row.

## Answer

I lean Option 2. Once we've validated this idea on the 90 deg columns, we can look to generalizing it for other angles. I think we should set a default of ~3-5 deg tilt back from the currently-computed catch orientation, but it'd be good if this was runtime configurable
