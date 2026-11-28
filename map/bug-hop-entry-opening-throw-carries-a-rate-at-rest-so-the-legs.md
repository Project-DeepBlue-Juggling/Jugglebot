---
id: BUG-152
kind: bug
title: Hop entry opening Throw carries a rate at rest, so the Legs ripple before the first Throw
status: complete
created: '2026-11-27'
updated: '2026-11-28T00:10:27Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: decision
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-28-rest-seed-knot0-rate.md
parent: pattern-progression-columns-to-centre-and-orbit
filed_by: claude
---

Found 2026-11-27 while gating JP-11 rotating Columns (IDEA-12) in the sim.

## Findings

- The opening Throw of a fed Columns Attempt (the Hop entry) is planned from rest, but its first Knot carries a rate: -1.13 mm/s in x and -0.018 rad/s in tilt at Sep. 125 and Apex 0.95.
- The rate comes from the one-sided knot derivative in `cup_realize` applied to the Banking tilt series, not from the rest seed, whose rate is zero.
- Before the start time the emitter sends frames with equal positions and that rate, so the firmware's Hermite interpolation ripples each Leg by up to 8.6 micrometres and +3.6 to -1.8 mm/s at 40 Hz for about 0.3 s.
- The sim gate's Leg mirror check reads 1.20e-4 rev against its 1.0e-4 band (1.26e-4 at Apex 0.90). Plain Columns from a timed start reads 1.0e-5.
- Fed Columns has carried this at every Sitting; the Learner gate never scored the Leg mirror on a Hop entry. Read from the code, not measured in a bag.
- Physically small: far inside the firmware deviation Guard. It is a band violation in the gate, and it blocks the JP-11 sim gate.

## Options

(1) seed the first Knot's rate from the rest seed (zero) in the planner, which changes the bytes of every Hop entry and fresh-origin opening and needs a control-cycle walk; (2) fly JP-11 with this Bug open.

## Unchecked

Whether other fresh-origin plans from rest (the Feed Throw, the one-ball opening Throw) carry the same rate. Probes: scratchpad out/jp11diag of the session that filed this.

## Progress

- 2026-11-28: Fixed in the planner (I-PLAN-17): a plan seeded from rest takes the seed's zero rate at its first Knot. Hop entry Leg mirror on the opening plan 1.196e-4 to 1.15e-7 rev in the sim. The one-ball opening Throw and a fresh REST carried it too. Moving seeds are unchanged: their first Knot is dropped at the splice. Unflown.
