---
id: TKT-95
kind: ticket
title: 'BUG-139: fix the slope''s corrupted value with process noise on b, directional forgetting, one forgetting factor for all, or leave it?'
status: open
created: '2026-11-26'
updated: '2026-11-25T15:58:48Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: learner
blocked_by: []
links: []
parent: bug-plant-model-slope-value-stays-corrupted-after-a-long-single
type: question
---

BUG-139 is the residue of BUG-137: after a long single-Site Memory, the plant model's slope s holds a wrong value, because the per-parameter forgetting manufactures information about it. One row at a second Site then applies a line up to 59 mm off at a far Site. The live Memory copy (2026-11-21) is not affected at any flown Site, and the fix moves the P1 replay probe's pin, which is why it waits on you.

## Options

1. Process noise on b (a random walk, P_bb += q per row) in place of the per-parameter forgetting. This forgets b without creating information about s. The night builds it with the strict-xfail pin turned into a passing test, re-pins the P1 probe's band+struct variant, replays the 2026-11-21 Memory copy, and holds it for a day Session.
2. Directional forgetting: forget only in the excited direction. More machinery, same intent; same build and hold as 1.
3. Forget s as well (one lambda for every parameter). The probe rejected this: s falls back to its prior after 410 to 1958 single-Site rows.
4. Leave it. The BUG-137 guard holds a far Site at the identity, so the residue matters only in the first rows after a long single-Site run; the strict xfail stays as the record.

## Note on BUG-138

Tonight's BUG-138 build (your TKT-93 answer, option 3) splits the lateral filters by release kind. Each kind's lateral filter still has this flaw, so the answer here applies to both.

## What each answer unblocks

Answer 1 or 2: the next night builds it as above and holds it for a day Session, since it changes every commanded u and the replay pin. Answer 3: not recommended (rejected by the probe). Answer 4: the Item is closed as accepted, with the xfail kept.

Recommendation: 1, because it removes the mechanism that manufactures the information and is the smallest change to the filter.
