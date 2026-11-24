---
id: BUG-140
kind: bug
title: Fresh REST rebases outside the band are uncapped (up to 1.0 s logged) and land off the knot grid
status: inbox
created: '2026-11-25'
updated: '2026-11-24T17:39:42Z'
owners: []
priority: medium
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
