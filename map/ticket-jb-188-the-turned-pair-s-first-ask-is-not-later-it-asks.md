---
id: TKT-51
kind: ticket
title: 'BUG-75: the turned pair''s first ask is not later, it asks twice more often at no measurable cost: close it, read why it asks twice, or decline those asks in the fence?'
status: complete
created: '2026-11-08'
updated: '2026-11-08T00:18:18Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-catch-re-aim-refused-infeasible-on-8-4-of-throws-at-the-p2
type: question
---

You asked why the turned pair's first ask is later. In the 15:53 bag it is not: the first ask comes 0.273 s after release on `columns_turned` and 0.265 s on plain `columns`, and both splice 0.317 s before landing. The gap in the Item compared the turned pair with one-ball `self_toss`, which splices by a different rule. Full tables in `## Findings (2026-11-08)` on BUG-75.

## What is real

The turned pair asks a second time far more often: 27 of 72 Catches against 3 of 51. Why is not explained. Fit drift, Ball, first-ask size and the 30 mm clamp do not separate them.

It costs nothing measurable: all 68 summarised Catches were caught, and the landing radius is 25.1 mm after a refused second ask against 25.0 mm with no second ask.

## Options

1. Close BUG-75 as designed, with no change.
2. Read why the turned pair asks twice, in the Robot frame. A bag read of about one unit; it needs the tracker-to-Robot transform.
3. Decline these asks in the pre-solve fence, so they spend no solve. A small executor change that needs a control walk and tidies only a count.

Recommendation: 1. Nothing here costs a Catch, and no limit or timing change is proposed.

## What each answer unblocks

- 1: the next night closes BUG-75.
- 2: the next night runs the bag read.
- 3: the next night builds it on a branch, held for your decision on its control walk.

## Answer

Let's go with Option 2. Once we have that information, we can decide how to best proceed
