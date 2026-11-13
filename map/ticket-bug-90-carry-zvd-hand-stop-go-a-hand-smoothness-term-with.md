---
id: TKT-69
kind: ticket
title: 'BUG-90 carry_zvd Hand stop-go: a Hand smoothness term with the ZVD rows on, take the Hand out of the ZVD lever, a hard bound, or leave the flag off?'
status: open
created: '2026-11-14'
updated: '2026-11-13T17:50:57Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-carry-zvd-rows-make-the-hand-stop-and-go-at-about-5-hz-in
type: question
---

With `carry_zvd` on, the pose planner gives the Hand a stop-and-go velocity of about 5 Hz in flight (5 turning points at Dwell 0.35, 4 at 0.40, 2 at 0.50; none with the flag off). The operator saw it at the Sitting of 2026-11-13, step 21; a strict xfail in `tests/motion/test_cup_zvd.py` pins it, and flag-on Dwell 0.35 must not fly until it is fixed.

## Options

1. **A Hand smoothness term with the ZVD rows on:** penalise Hand jerk across the flight window only when the flag is set; the xfail judges it. The night builds it (an Opus unit, planner math) and holds it for a Sitting.
2. **Take the Hand out of the ZVD lever:** reformulate the rows so a Hand wiggle cannot cancel the mode. For a day Session.
3. **A hard bound:** no Hand velocity turning point in flight. Risks infeasible solves at short Dwell.
4. **Leave the flag off** (the default) and park the ZVD arm of FEAT-32.

## What each answer unblocks

- 1: the next night builds it on a branch; a Sitting at Dwell 0.40 with the flag on judges it, comparing the Platform residual too.
- 2: a day Session. 3: the next night, same as 1. 4: BUG-90 is parked with FEAT-32's ZVD arm.

Recommendation: 1. The smallest change the existing xfail can judge, and the flag-off plan stays byte for byte as now.
