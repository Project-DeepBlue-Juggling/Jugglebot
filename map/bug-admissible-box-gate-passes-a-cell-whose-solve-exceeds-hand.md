---
id: JB-120
kind: bug
title: Admissible-box gate passes a cell whose solve exceeds HAND_LIMIT_ACC by ~5%
status: inbox
created: '2026-10-31'
updated: '2026-10-31T04:51:56Z'
owners: []
priority: high
effort: m
hardware: robot
owner: none
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links: []
parent: two-ball-oval
filed_by: claude-books-2026-10-31
---

The admissible-box gate reports a Columns Ball 2 catch as `inside box` while the realised solve is refused on Hand acceleration, so the attempt ends at the first install.

## Evidence (2026-10-31, goal 4, Columns with a Feed, launch 13-14-00)

- Install refused: `REJECTED_CYCLE_INFEASIBLE(HAND_LIMIT_ACC: peak hand acceleration 4112.5 rev/s^2 > 3900.0)` at skill 13, the CATCH of Ball 2, after 12 of 13 caught.
- The same second the gate printed `BOX-EXPECTED-LANDING skill 13: u=(-16.6, -20.0) mm ... inside box x[-2.0,+1.0] y[-20.0,+8.0]`.
- The learner command for Ball 2 y sat at -20.0 mm, the box edge, on every Ball 2 catch of that goal (memory 2112+ rows, k=16).
- Gate and solver disagree by about 5 percent at the y = -20 edge. Zero such refusals in the 10-28 and 10-30 logs; the cell was 50/50 on 10-30.
- The 10-30 prep entry (C1) already put this cell within 2-6 percent of the Hand limit at hold 0.100.

## Fix candidates

1. Shrink the box y lower edge from -20 toward -15 for the Columns Ball 2 cell.
2. Add a `CATCH-AIM-FALLBACK` for `HAND_LIMIT_ACC`, the same shape as the existing `LIMIT_VEL` fallback to the unlearned aim.
3. Replay u = (-16.6, -20.0) offline with the box tool first to see which u produces the peak.

## Sitting

- setup: armed, bb-fed
- why: confirm the Columns Ball 2 catch no longer ends on HAND_LIMIT_ACC after the box edge or fallback change.
- do:
  - Fly Columns, Sep. 125, Apex 0.95, Throws 50, twice.
- pass: no END HAND_LIMIT_ACC and no refused first install in either 50.
- record: per-catch peak Hand acceleration from the install lines and the learner y for Ball 2.
- minutes: 15
