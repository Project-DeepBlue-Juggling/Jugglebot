---
id: TKT-98
kind: ticket
title: 'BUG-146: fix the Hand C2 artefact with spline knot rates, a fourth-order stencil, exact tilt rates, or leave it?'
status: open
created: '2026-11-27'
updated: '2026-11-26T15:59:02Z'
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

BUG-146: an aimed Throw from a level rest is refused HAND_LIMIT_C2 past about 6.2 deg of release tilt. The Night Shift traced it on 2026-11-27 (the Item's Findings): the Hand's tilt-drop term gets its knot rates from a finite difference, and the C2 gate's Hermite rebuild turns that rate error into an acceleration jump of about 0.116·θ² rev/s². The jump peaks in the SETTLE tail just after the release, and every tilted plan carries it today.

## Options

1. Clamped cubic-spline knot rates for the drop term in `decompose`, with knot 0 clamped to the seed's Hand rate. The jump goes to zero at every tilt, positions are unchanged, and Hand knot rates move by at most 0.0094 rev/s at 6 deg. The night builds it with a test on a branch and holds it for a Sitting (a Columns Attempt), since it changes every tilted plan's rates.
2. A fourth-order stencil: one line, cuts the jump about 3.6x, still fails near the 12 deg cap.
3. Exact rates from the tilt schedule's derivatives: a residual of 0.16 to 0.24 remains, and `tilt_schedule` must return derivatives.
4. Leave it: aimed reach stays capped at about 6.2 deg of release tilt (about 280, 415 and 548 mm at Apex 1.5, 1.8 and 2.1 m).

## What each answer unblocks

1, 2 or 3: the next night builds it on a branch, with the C2 test and a splice-seam test, writes the Sitting section, and holds it for a Sitting. 4: the Item is closed as accepted and the reach table stands for PWAY-11.

Recommendation: 1, because it removes the artefact at its source and leaves every Hand position unchanged.
