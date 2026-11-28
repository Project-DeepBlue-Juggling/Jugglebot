---
id: TKT-105
kind: ticket
title: 'BUG-149 held on night-shift/2026-11-28-BUG-149: apply the audit''s 2 test fixes and merge, merge as is, or drop?'
status: complete
created: '2026-11-28'
updated: '2026-11-28T00:39:43Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-catch-install-cpu-grows-about-1-ms-per-100-record-knots
type: question
---

BUG-149 is built on night-shift/2026-11-28-BUG-149. The growth is not the record-head copy the debrief suspected: `extend()` re-measured every mark's stroke-clear and arm-lead twin at each splice. Now only marks whose measurement can reach the seam are re-measured; offline `splice_at` CPU falls from 24.1 to 12.2 ms at 1279 Knots, with no plan array, Knot time, splice knot or refusal changed.

The audit traced the skip rule and found the planner change sound, but left two test-strength findings open, so the gate was not asked.

## Open findings

1. The head-identity test's seam assertion is vacuous (it uses the new plan's last knot, not the splice knot) and compares only pose and Hand position, not the velocity channels the twins read. The fix takes the real seam from the install and compares every channel below it.
2. There are no direct boundary tests of the two skip helpers (None values, a release at the seam, catches one to three knots before it), no randomized skip-implies-equal check, and no assertion that the chain truncated the head.

## Options

1. Apply both: the next night strengthens the tests on the branch, re-audits, gates it, and merges on MERGE.
2. Merge as is: the planner change stands on the reviewer's proof and the 14-install equality test; a day Session merges it.
3. Drop the branch: BUG-149 goes back to backlog.

## What each answer unblocks

1: a merge the next night if the gate says MERGE, then a Sitting reads the install CPU through a 32-Throw oval. 2: a day Session merges. 3: the Item reopens without a branch.

Recommendation: 1, because the fix sits on the live splice path and the tests are what will catch a future edit that breaks the skip rule.

## Answer

Agreed. Go with Option 1
