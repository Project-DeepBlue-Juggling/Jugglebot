---
id: JB-152
kind: ticket
title: 'JB-120 held on night-shift/2026-11-01-JB-120: apply the audit''s 2 fixes and merge, merge as is, or drop?'
status: complete
created: '2026-11-03'
updated: '2026-11-02T22:48:42Z'
owners: []
priority: high
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-admissible-box-gate-passes-a-cell-whose-solve-exceeds-hand
type: question
---

JB-120 has been held on `night-shift/2026-11-01-JB-120` since 2026-11-01. The branch makes the learned-prior CATCH fallback (re-solve at the unlearned aim) also fire on a `HAND_LIMIT_ACC` / `HAND_LIMIT_VEL` refusal. Its audit left two behaviour-affecting findings open, and no question was filed then.

Since then two Sittings put 7 of 7 refusals, and then all 3, on catches seeded from a large inward bump. Offline the aim moves the peak by at most 0.7 points; the seed moves it by about 12. Main now carries the bump brake-seed fix (2026-11-02 debrief), so the branch's fallback was never flown and could not have rescued those catches.

## Options

1. Apply both findings and merge: match the exact set `HAND_LIMIT_ACC` / `HAND_LIMIT_VEL` (today the `HAND_LIMIT_` prefix also admits `HAND_LIMIT_C2`, untested), and add a test with both solves refused. This costs one extra solve, and only on a refusal. A unit and a re-gate.
2. Merge as is: `HAND_LIMIT_C2` refusals also get the fallback, which nothing tests or documents.
3. Drop the branch: the fallback cannot move a seed-driven peak, and the structural fix filed on JB-120 replaces it. That fix plans the next catch at bump time and declines or shrinks the bump.

Recommendation: 3. Keep the Sitting section's re-fly of main's bump brake-seed fix, and take out its `- branch:` line so a pass does not merge a fallback that the evidence says is inert.

## What each answer unblocks

- 1: the Night Shift applies the two fixes, re-gates and merges, and the Sitting keeps the branch line.
- 2: a re-gate on the merged tip and a merge.
- 3: the branch is deleted, the `- branch:` line comes out, and JB-120 waits only on its Sitting and on the structural fix.

## Answer

I agree with your recommendation
