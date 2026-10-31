---
id: JB-120
kind: bug
title: Admissible-box gate passes a cell whose solve exceeds HAND_LIMIT_ACC by ~5%
status: in_progress
created: '2026-10-31'
updated: '2026-10-31T18:27:12Z'
owners:
- night-shift
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

## Night Shift replay (2026-11-01)

Offline replay through the production `plan_segment` (flown tree `48bf4fcb^` and HEAD, hold 0.100 and 0, the sweep's seed and a live-like seed, catch aim 0 and -20 mm y, throw u up to (-16.6, -20) mm): **no case reaches 3900 rev/s^2**; the highest is 91 percent, the live 4112.5 is 14 points above it.

The lateral inputs move the peak by at most 0.7 points, the seed by 12. Fix candidate 1 (shrink the y edge) is withdrawn. The input left is the live seed (Ball 1's installed plan at k_s 293 after its re-aim and bump), which is not logged.

- Fixed on the branch: every goal-4 catch dispatched `source=schedule:learned`, so the unit-A2 fallback to y_d should have run and did not, because `install_segment` returns the bare check code `HAND_LIMIT_ACC` and the classifier only knew `LIMIT_*` / `INFEASIBLE`. The branch adds the `HAND_LIMIT_` prefix (one more solve, only on a refusal). The replay says y_d alone is unlikely to rescue this cell.
- Two sweep gaps, real but not shown causal: `_within_margin` applies the 0.9 margin to the legs only (the Hand is certified at 100 percent; this cell sits at 90 in the sweep's shape), and the box bounds the expected landing while the sweep certified command = landing (the live u x = -16.6 mm is outside the box x [-2, +1] and the sweep's shape refuses LIMIT_VEL there).
- Today's defaults (hold 0, stroke top +16 mm) put every replayed case at 66-79 percent.

Entry: logbook/2026-11-01-jb120-box-gate-vs-hand-limit-acc-replay.md.

## Sitting

- setup: armed, bb-fed
- why: confirm the Columns Ball 2 catch no longer ends on HAND_LIMIT_ACC after the box edge or fallback change.
- do:
  - Fly Columns, Sep. 125, Apex 0.95, Throws 50, twice.
- pass: no END HAND_LIMIT_ACC and no refused first install in either 50.
- record: per-catch peak Hand acceleration from the install lines and the learner y for Ball 2.
- minutes: 15
- branch: night-shift/2026-11-01-JB-120

## Progress

- 2026-11-01: Night Shift 2026-11-01: held on night-shift/2026-11-01-JB-120: hardware robot; audit open on HAND_LIMIT_C2 admitted by the new HAND_LIMIT_ prefix and no both-refused test; unblocked by the Sitting in the section below plus the owner's call on the C2 finding (detail below)
