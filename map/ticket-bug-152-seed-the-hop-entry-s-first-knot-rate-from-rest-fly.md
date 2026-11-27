---
id: TKT-101
kind: ticket
title: 'BUG-152: seed the Hop entry''s first-Knot rate from rest, fly JP-11 with it open, or fix the whole Platform rate class?'
status: open
created: '2026-11-28'
updated: '2026-11-27T15:51:49Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-hop-entry-opening-throw-carries-a-rate-at-rest-so-the-legs
type: question
---

BUG-152: the opening Throw of a fed Columns Attempt (the Hop entry) is planned from rest, but its first Knot carries a rate (-1.13 mm/s in x, -0.018 rad/s in tilt) from the one-sided knot derivative in `cup_realize` on the Banking tilt series. Physically small (Legs ripple up to 8.6 micrometres for about 0.3 s, far inside the deviation Guard), but it reads 1.20e-4 rev against the sim gate's 1.0e-4 Leg mirror band and so blocks the JP-11 sim gate (IDEA-12).

It is the Platform half of the same finite-difference class as BUG-146, whose fix (clamped spline rates for the Hand's drop term, TKT-98) the night is building tonight on its own branch.

## Options

1. Seed the first Knot's Platform rates from the rest seed (zero) when a plan starts at rest (the Hop entry and the fresh-origin openings). Changes the bytes of every such opening. The night builds it with a control-cycle walk, a test that the sim gate's Leg mirror check passes on the Hop entry, and a check of the Feed Throw and one-ball opening Throw (unchecked today), then holds it for a Sitting.
2. Fly JP-11 with it open: nothing changes on the robot; the JP-11 sim gate stays blocked at the mirror band.
3. Fix the whole Platform class at once: clamped spline rates for shift, tilt and dz like BUG-146's Hand fix, start rate pinned to the seed. Larger (effort m) and changes every tilted plan's Leg rates, so it is better after BUG-146 has flown.

## What each answer unblocks

1: the next night builds and holds it for a Sitting (a fed Columns Attempt). 2: the Item is accepted as is and IDEA-12's gate needs another route. 3: the night drafts the class fix after BUG-146's Sitting.

Recommendation: 1, because it removes the rate at its source with the smallest change, and the class fix can follow once BUG-146 has flown.
