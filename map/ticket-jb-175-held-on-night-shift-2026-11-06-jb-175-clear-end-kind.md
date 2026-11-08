---
id: TKT-40
kind: ticket
title: 'BUG-68 held on night-shift/2026-11-06-BUG-68: clear end_kind on the re-labelled end and merge, merge as is, or drop?'
status: complete
created: '2026-11-06'
updated: '2026-11-05T22:56:17Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-a-hand-or-guard-fault-inside-a-catch-s-judging-window-ends
type: question
---

BUG-68 is built on night-shift/2026-11-06-BUG-68: a Hand or guard fault during a Catch's judging window now ends the Attempt `ABORTED_FAULT`, and the Catch is booked unverifiable. The full gate passed, but the audit left one finding open, so the merge gate held it.

## The open finding

When the fault arrives just after the verdict (the 2026-11-05 case), the end is re-labelled but keeps `end_kind` CATCH, so the operator line reads "ENDED (ABORTED_FAULT) at the CATCH". The fault did coincide with that Catch, but it was not the Catch's verdict. No motion path reads it.

## Options

1. Apply it and merge: clear `end_kind` beside the re-label, so the line reads "ENDED (ABORTED_FAULT)". One line plus a test.
2. Merge as is: the line keeps "at the CATCH", which still tells you where the fault landed.
3. Drop the branch.

Recommendation: 1. The Item exists to stop that line blaming the Catch.

## What else to know before you merge

- A guard-only latch now ends the Attempt `ABORTED_FAULT` at the next tick. Before, it ran on and its installs were refused `GUARD_LATCHED`. A guard latch never pre-empts `LANE_REFUSED`, so the forced hold is unchanged.
- A genuine `MISSED_CATCH` goal result now arrives up to 0.4 s later, so a late fault can correct it. Nothing physical waits on it.

## What each answer unblocks

- 1: the next night applies it, re-gates and merges.
- 2: the next night re-gates and merges.
- 3: BUG-68 goes back to open.

## Answer

I agree with your recommendation
