---
id: TKT-106
kind: ticket
title: 'BUG-146 held on night-shift/2026-11-28-BUG-146: add the spline solver''s unit test before the Sitting, fly it as is, or drop it?'
status: complete
created: '2026-11-28'
updated: '2026-11-28T00:40:19Z'
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

BUG-146 is built on night-shift/2026-11-28-BUG-146 as you chose in TKT-98 (option 1): the Hand's tilt-drop rates are a clamped cubic spline with both end rates pinned. The C2 jump inside each window is now zero at every tilt, and the 6.82 deg aimed Throw that was refused is accepted. Seam rates and Hand positions are unchanged; Hand knot rates move by at most 0.0094 rev/s at 6 deg and 0.021 at 9 deg.

It is held for its Sitting (a fed Columns Attempt) either way. The audit traced the solver by algebra and found it correct, but left one finding open.

## Open finding

The new solver `_clamped_spline_rates` has no direct unit test. It is checked only through whole plans of about 31 Knots, so an off-by-one at 3 or 4 Knots would go unseen, and the 3-Knot seam window is a live case. The fix is a parametrized test against a dense reference solve at 0 to 5 Knots.

## Options

1. Add the test first: the next night writes it on the branch, re-audits, and leaves it held for the Sitting.
2. Fly as is: the Sitting may fly the branch now, and a day Session adds the test before any merge.
3. Drop the branch: aimed reach stays capped at about 6.2 deg of release tilt.

## What each answer unblocks

1: the Sitting section stands as written after the next night. 2: the Sitting may fly it now. 3: BUG-146 goes back to open.

Recommendation: 1, because the 3-Knot case is on the live seam path and a reference test is cheap.

## Answer

Agreed. Go with Option 1
