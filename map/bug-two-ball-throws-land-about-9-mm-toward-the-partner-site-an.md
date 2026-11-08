---
id: BUG-79
kind: bug
title: 'Two-ball Throws land about 9 mm toward the partner Site: an arrival transient the plant model has no term for'
status: inbox
created: '2026-11-06'
updated: '2026-11-08T00:13:20Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: subsystem
diagnosis: hypothesis
area: skill-stack
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

## Cheapest hardware check (as first proposed; the rehearsed row is the Sitting section)
Columns Sep. 125, Apex 0.93, alternating Dwell 0.27 and 0.40 (three Attempts each), bag recorded. It separates a dependence on Apex from one on the time from arrival to release.

## Links
Evidence: `temp/probes/sitting_debrief_2026-11-06b/unit_transit.md`, `unit_B_D.md`. Related: BUG-74 (collisions), plan `learner-generalisation.md`, `logbook/2026-11-06-sitting-debrief-2.md`.

## Dwell A/B check (off the Runbook since 2026-11-08)
Flown 2026-11-07 and taken off the Runbook by the owner until it is redesigned: at Apex 0.93 Sep. 125 Dwell 0.32 and 0.30 each separated Legs 1 and 2 (BUG-72), and no planned Leg peak separates safe transits from unsafe ones, so a redesign needs a Dwell contrast inside what has flown on the executed command. Detail in `logbook/2026-11-07-sitting-debrief.md`.

## Progress

- 2026-11-06: Offline probe, planner defect ruled out: the release follows the caught position by design (asymmetric clamp predicts 5.3 mm, observed 6.0). Against the release that flew the pull is +1.1 / +1.3 mm position and -10.7 / -11.1 mm velocity (plan +16.6 / +13.3 mm/s, delivered +4.2 / +0.4). Position half withdrawn. BUG-81 filed. (detail in the logbook).
- 2026-11-06: Fix at source now means the ring after arrival (carry-ring-zvd) or the lateral launch delivery, not the planner; the planner-defect branch is closed. Cheapest next offline check: regress ball lateral v on planned v, encoder v at release and tilt, split RESEND and BUMP. The Dwell A/B hardware check stands.
- 2026-11-06: Dwell A/B rehearsed offline (nominate, Sitting limits, pose): Apex 0.93 Sep. 125 Dwell 0.27 accepted (margin 0.337, re-aim 0.121, Legs 310 mm/s 4076 mm/s^2); Dwell 0.40 REFUSED (21 refusals), 0.35 accepted (0.110, re-aim -0.298, 384 mm/s 6207 mm/s^2). Long arm set to 0.35.
- 2026-11-06: Runbook row set to Dwell 0.27 against 0.32 (both inside the flown envelope: 353 mm/s, 5180 mm/s^2 at 0.32). 0.35 is left out: it plans 6207 mm/s^2, within 2 % of the 6296 that separated Legs 1 and 2 (BUG-72). The debrief reads the Platform's velocity at release from the bag, a sharper measure than landings over 0.05 s.
- 2026-11-06: 2026-11-06: owner decision on carry-ring-zvd section 8: the ZVD rows stay off on Columns until BUG-72 has a bound. This row's own goals (Apex 0.93, Dwell 0.27 and 0.32) are RING refused with the rows on anyway, so the suppress-at-source branch needs both a bound and a window that holds the rows.
- 2026-11-08: Failed at the Sitting of 2026-11-07: Dwell 0.32 and 0.30 each separated Legs 1 and 2 in their first Attempt; later Attempts refused the Segment after Ball 2's first Throw (Feed aim clamped -30 mm). No answer: pull n 34/5/4. Row off the Runbook until redesigned (owner): 'inside what has flown' compared nominal with live peaks, and no plan peak separates the events.
