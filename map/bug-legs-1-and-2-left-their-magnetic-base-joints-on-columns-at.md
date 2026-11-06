---
id: JB-184
kind: bug
title: 'Legs 1 and 2 left their magnetic base joints on Columns at Sep. 170: acceptance bounds Leg kinematics only and does not see the bridge feedforward cap'
status: inbox
created: '2026-11-06'
updated: '2026-11-06T05:58:30Z'
owners: []
priority: high
hardware: none
owner: decision
radius: cross-cutting
diagnosis: hypothesis
area: legs
blocked_by: []
links:
- logbook/2026-11-06-sitting-debrief.md
- logbook/2026-11-06-sitting-debrief-2.md
filed_by: claude
---

At the Sitting of 2026-11-06 a two-ball Columns goal at Apex 1.00, Sep. 170, Dwell 0.27 (limits 600 / 8000 / 200000) pulled Legs 1 and 2 out of their magnetic base joints about 9 s after Start. The owner pressed the E-stop button; nothing was damaged.

**Owner decision (2026-11-06): no bound on transit aggressiveness for now.** The consequence was minor and a conservative limit has its own cost; the owner will be mindful of the Patterns requested. Do not add a limit, margin or guard under this Item without a new owner decision.

## Evidence
- Each transit at Sep. 170 was about 187 mm in 0.3 s (170 plus the 10 mm outward re-aim at each end). The install lines asked Legs 1 and 2 for 486 mm/s and 6296 mm/s^2 (81.0 % and 78.7 % of the limits); the clean Attempts of the same launch (Sep. 100, 125, 135) asked for at most 361 mm/s and 4672 mm/s^2.
- The can-bridge caps each Leg's velocity feedforward at 3.5 rev/s, which is 249 mm/s. The rest of a faster plan is made up by position error: Legs 1 and 2 logged 55 and 65 lead-clamp ticks in two transits, against at most 36 in 21 transits in a clean Attempt.
- The joints let go at the end of the second transit (the Platform mocap pose left the pose implied by the Leg encoders from 1793931867.53). That transit's own loads were ordinary; the first one carried the highest estimated Leg tension of the Sitting (59 N on Leg 1 against a clean worst of 47 N; the estimate was not checked against a load cell).
- The gate and the nomination bound Leg velocity, acceleration and jerk only. Nothing bounds Leg tension, Platform lateral acceleration or joint pull-off, and nothing knows the bridge's feedforward cap.
- Width alone is not the cause: the hop at Sep. 250 flew on 2026-10-19 to 2026-10-22 at limits 350 / 5000 without separation.

## Not settled
- Whether the first transit unseated the joints and the second finished it, or the arrival of a long fast transit rolled the joint out with little axial preload. A bench pull-off test of one magnetic joint (axial force, and how it falls off-axis) would decide it.

## What would reopen this
- A second separation, or a wish to fly Columns wider than Sep. 135 at Apex 1.00. Numbers for choosing a bound are in `logbook/2026-11-06-sitting-debrief.md`.

## Progress

- 2026-11-06: 2026-11-06 P2 Sitting: two-ball goals at Sep. 120 planned Leg peaks of 321 to 371 mm/s and 3656 to 5488 mm/s^2 (goal 17, L9651), above the previous clean envelope (361 / 4672) and under the separating transit (486 / 6296). Owner: base joints stayed attached throughout. The Runbook's offline prediction (264 / 3011) read low: it did not include the Catch re-aim.
