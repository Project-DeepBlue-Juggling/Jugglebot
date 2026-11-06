---
id: JB-197
kind: ticket
title: 'JB-169 held again on night-shift/2026-11-06-JB-169: limit the ledger count to a completed goal with a landing and merge, limit it to a completed goal only, merge as is, or drop?'
status: complete
created: '2026-11-07'
updated: '2026-11-06T22:51:13Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-full-length-columns-goal-ends-missed-catch-on-its-final
type: question
---

JB-169 now does what you chose in JB-180: a booked final Catch stays unverifiable whatever the cup reads later, the rule runs only when no other END has fired, and the ledger reads N/N. The full gate passed on the tip (`./run_tests.sh --full`, 2026-11-07, `EXIT=0`). The audit then found the ledger count reaches further than "the final Catch", so the merge gate held it again. Nothing is flown.

## The open findings

1. **A Catch with no evidence.** The ledger adds every booked row, including one with no landing estimate, an Apex outside the band, or a Throw JB-189 refused a Memory row. That flight would count as thrown and caught.
2. **An aborted goal.** The booking raises no END, so a fault, Stop or mode change in the 0.5 s before the row finalises ends the goal aborted, and the ledger still adds the Catch.
3. **A tilted tail, unchecked.** The motion walk assumes no tilt on the final skills. A turned pair with Banking has no test. No sign of a problem, but it is not shown either.

## Options

1. Apply all three and merge: count it only on a `COMPLETED` goal whose booked row has a landing, and add a tilted-tail test (a code change only if that test shows a difference).
2. Apply 2 and 3 only: the ledger still counts a Catch with no landing estimate.
3. Merge as is.
4. Drop the branch.

Recommendation: 1. It keeps the N/N you asked for on the goals that raised this, the 2026-11-04 and 2026-10-28 ones.

## What each answer unblocks

- 1 or 2: the next night applies them, re-gates and merges.
- 3: the next night re-gates and merges.
- 4: JB-169 goes back to open.

## Answer

Go ahead with your recommendation.
