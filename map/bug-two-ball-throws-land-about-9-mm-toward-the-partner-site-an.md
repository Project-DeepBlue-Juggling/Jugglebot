---
id: JB-192
kind: bug
title: 'Two-ball Throws land about 9 mm toward the partner Site: an arrival transient the plant model has no term for'
status: inbox
created: '2026-11-06'
updated: '2026-11-06T06:13:44Z'
owners: []
priority: high
hardware: none
blocked_by: []
links:
- map/bug-mid-air-ball-collisions-end-half-of-two-ball-columns.md
- plans/active/learner-generalisation.md
- logbook/2026-11-06-sitting-debrief-2.md
parent: learner-generalisation
filed_by: claude
---

At the P2 Sitting of 2026-11-06 and in the 13:19 data, two-ball Throws landed pulled toward the partner Site, and the plant model has no term that holds it. It is not the "centre pull" the plan assumed.

## Findings
- One-ball Throws show no significant pull (`s_x` -0.073 +- 0.061, `s_y` +0.067 +- 0.059 per m). Two-ball Throws land pulled toward the partner Site by -0.18 +- 0.02 per m of Site coordinate (about 11 mm at Sep. 120), equally along x and along a pair turned 90 degrees.
- The first Throw of an Attempt, with the Platform at rest, has no pull (+8.9 +- 1.8 mm outward of later Throws, 117 Attempts). A departure term is null. So it is an arrival effect.
- In the P2 bag half is position: the commanded Carry creeps 5 to 6 mm toward the partner between touch-down and release, and the recorded release does not hold it, so the flyback never returns it.
- The other half is velocity: the Platform still moves about 7 mm/s toward the partner at release beyond its command, the ring left by the arrival (about 5 Hz, the same ring the carry-ring work measured near 6 Hz).
- An arrival term `c * (Site - arrived_from) / 2`, c = -0.150 +- 0.010, beats today's model on 382 held-out Attempts by 6.5 +- 2.8 mm^2. The pull grows faster than linearly with Sep. and vanishes at Apex 1.05 and above, so a scalar c over-corrects by about 6 mm at Apex 1.10.
- Ruled out: a radial Site term, a departure effect, a ballistic-velocity term, Platform tilt (it is commanded and points outward).

## Decision taken
The arrival term is not added to the plant model now. Half of it may be a planner defect to fix at its source, the other half is the ring the carry-ring-zvd plan targets, a scalar term is wrong at Apex 1.05 and above, and it needs a Memory column (`arrived_from`) whose old rows defaulted to zero would double-count about 9 mm in the residual memory. It gates `yaw_rad`: a pair whose angle changes each beat cannot learn it from Robot-frame Site keys.

## Options
- Fix at source: probe the Carry creep with `_carried_release_mm` against the streamed Cup position at the release knot in a sim replay, and the ring through the carry-ring-zvd plan.
- Learn it: the arrival term with an `arrived_from` Memory column (schema v3; reconstruct stored rows, never default to zero).
- Both: fix the creep, then identify c on what remains.

## Cheapest hardware check (a proposal, not rehearsed, not on the Runbook)
Columns Sep. 125, Apex 0.93, alternating Dwell 0.27 and 0.40 (three Attempts each), bag recorded. It separates a dependence on Apex from one on the time from arrival to release.

## Links
Evidence: `temp/probes/sitting_debrief_2026-11-06b/unit_transit.md`, `unit_B_D.md`. Related: JB-186 (collisions), plan `learner-generalisation.md`, `logbook/2026-11-06-sitting-debrief-2.md`.
