---
id: TKT-109
kind: ticket
title: 'BUG-146 on night-shift/2026-11-28-BUG-146: relax BUG-152''s knot-0 test to a tolerance on the later Hand rates, feed the spline the old start rate, or drop the branch?'
status: open
created: '2026-11-29'
updated: '2026-11-28T16:24:19Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-aimed-throw-from-a-level-rest-is-refused-hand-limit-c2-past
type: question
---

BUG-146's branch now has the spline solver's direct test, as you chose in TKT-106. Merging main into it brought in BUG-152's fix: a plan seeded from rest starts at zero rate, I-PLAN-17. The two meet in `decompose`, and one of BUG-152's tests now fails on the branch.

## What fails and why

`test_seed_rest_knot0_rate.py::test_decompose_seed_at_rest_changes_knot_zero_only` asserts that seeding from rest changes only Knot 0, with every later Hand rate bit-identical.

The branch's clamped spline couples the whole series to its pinned start rate. So when the start rate is the seed's zero, the interior Hand rates move by about 1e-5 rev/s (Knot 1: 8.5e-6 against -1.7e-6). That is the spline doing its job: the rates stay acceleration-continuous from a true zero start. I-PLAN-17 itself requires only Knot 0 to be zero, and it still holds.

## Options

1. Relax that test on the branch: Knot 0 exactly zero, later Hand rates within 1e-4 rev/s, plus a check that the series stays C2 from the zero start. The night does it, re-audits, and the branch stays held for its Sitting.
2. Feed the spline the old one-sided start rate and zero only Knot 0 afterwards. BUG-152's test passes unchanged, but the Hand gets an acceleration step at Knot 1 again, the class BUG-146 removes.
3. Drop BUG-146: aimed reach stays capped at about 6.2 deg of release tilt.

## What each answer unblocks

1: the branch is consistent with main and ready for its Sitting. 2: the same, with a small Knot-1 step at every at-rest start. 3: the Item reopens without a branch.

Recommendation: 1, because the moved rates are five orders below the Hand's limits and option 2 brings back the discontinuity BUG-146 exists to remove.
