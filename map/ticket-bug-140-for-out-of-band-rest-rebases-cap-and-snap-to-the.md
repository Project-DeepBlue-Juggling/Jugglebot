---
id: TKT-96
kind: ticket
title: 'BUG-140: for out-of-band REST rebases, cap and snap to the Knot grid, snap only, cap only at a bound you name, or leave both?'
status: complete
created: '2026-11-26'
updated: '2026-11-25T21:20:28Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: decision
radius: local
area: skill-stack
blocked_by: []
links: []
parent: bug-fresh-rest-rebases-outside-the-band-are-uncapped-up-to-1-0-s
type: question
---

A fresh REST outside the BUG-95 band still rebases with no cap (30 of 185 logged at-rest rebases slid more than 0.200 s, the largest +1.015 s) and starts off the Knot grid, so the emitter samples between Knots for the whole REST. One large slide ended an Attempt TILT_PIN at the Feed Catch (2026-10-28). Both changes alter flown motion, so this is your call and any build is held for a Sitting.

## Options

1. Cap and snap, as the band REST: whole Knots on the grid, capped at 8 (0.200 s), SPLICE_TOO_LATE past it. Refuses 30 of 185 logged rebases.
2. Snap only: on the grid, no cap. Removes the between-Knot sampling and refuses nothing that flies today.
3. Cap only, at a looser bound you name (for example 0.5 s).
4. Leave both.

## What each answer unblocks

1, 2 or 3: the next night builds it with tests (grid origin; cap and its refusal) on a branch, writes the Sitting section, and holds it for a Sitting. 4: the Item is closed as accepted.

Recommendation: 2 now, then decide the cap from the next Sittings' slide table.

## Answer

I agree with your recommendation; go with option 2 with a decision of the cap made based on the next Sittings' slide table.
