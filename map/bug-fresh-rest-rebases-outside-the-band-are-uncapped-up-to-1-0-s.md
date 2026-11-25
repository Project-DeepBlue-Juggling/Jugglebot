---
id: BUG-140
kind: bug
title: Fresh REST rebases outside the band are uncapped (up to 1.0 s logged) and land off the knot grid
status: inbox
created: '2026-11-25'
updated: '2026-11-25T15:59:12Z'
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
filed_by: claude
---

A fresh REST outside the BUG-95 band still rebases with no cap, and lands off the knot grid. Found while finishing BUG-95 (2026-11-25).

## Findings

- 200 rebases are logged from 2026-09-28 to 2026-11-20. Of 185 at-rest rebases, 30 slid more than 0.200 s; the largest is +1.015 s (2026-10-16 16:23).
- All the large slides are long pre-tilt RESTs outside the band. One, +0.673 s on 2026-10-28 09:53, ended its Attempt TILT_PIN at the Feed Catch.
- An at-rest rebase outside the band starts at `t_inst + 75 ms`, off the knot grid, so the emitter samples between knots for the whole REST. Unchecked on hardware.
- The table is `rebase_slides.csv` from the BUG-95 session; the band REST itself is capped at 8 knots and on the grid.

## Wanted

- Decide whether to cap these slides too, and whether to snap the origin to the grid. Both change behaviour that has flown, so this needs its own decision.

## Options (2026-11-26)

Night Shift 2026-11-26 drafted these; TKT-96 carries them. Both changes alter flown motion, so any build is held for a Sitting.

1. Cap and snap, as the band REST: slide whole Knots on the grid, capped at 8 (0.200 s), SPLICE_TOO_LATE past it. On the logged corpus that refuses 30 of 185 at-rest rebases, so those Attempts would end where today they slid.
2. Snap only: start the rebased REST on the Knot grid, no cap. The emitter stops sampling between Knots; no Attempt that flies today is refused.
3. Cap only, at a looser bound chosen from the slide table (for example 0.5 s), no snap.
4. Leave both: one Attempt in 200 rebases is known to have ended on a large slide (TILT_PIN, 2026-10-28).

Recommendation: 2 now, since it removes the off-grid sampling and refuses nothing; then decide the cap from the next Sittings' slide table.
