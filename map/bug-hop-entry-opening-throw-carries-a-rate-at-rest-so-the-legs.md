---
id: BUG-152
kind: bug
title: Hop entry opening Throw carries a rate at rest, so the Legs ripple before the first Throw
status: inbox
created: '2026-11-27'
updated: '2026-11-27T15:50:38Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: decision
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links: []
parent: pattern-progression-columns-to-centre-and-orbit
filed_by: claude
night_skip: input-decision
night_skip_on: '2026-11-28'
night_skip_note: Builds the fix you choose with a Hop-entry Leg mirror test and a Sitting section, then holds it for a Sitting.
night_skip_input: 'Pick 1-3 in TKT-101: seed the Hop entry''s first-Knot rate from rest, fly JP-11 with it open, or fix the whole Platform rate class after BUG-146.'
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
