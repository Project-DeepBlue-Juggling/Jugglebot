---
id: BUG-54
kind: bug
title: 'Box at hand acc 4500 traded hop landing width for apex band: the apex 0.95 hop cell''s x range shrank from +/-10 to +/-4 mm'
status: complete
created: '2026-11-02'
updated: '2026-11-03T07:09:01Z'
owners:
- night-shift
- jugglebot-50
priority: high
effort: m
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: skill-stack
blocked_by: []
links:
- map/bug-two-ball-columns-hop-start-throw-lands-25-56-mm-inboard-past.md
- map/bug-admissible-box-gate-passes-a-cell-whose-solve-exceeds-hand.md
- logbook/2026-11-03-jb143-margin-first-box-and-jb139-30mm-authority.md
filed_by: claude
---

Found 2026-11-02 reviewing the re-sweep at hand acc 4500 (temp/sweeps/h4500, gate_hash dd4ae01a93a0, 82 cells, 32 changed).

## What changed

- All eight hop cells gained apex band at the bottom (lower apex about 0.69 to 0.61 m).
- Hop 0.82-0.88 and 0.92-0.97 (the cell Columns at apex 0.95 flies) shrank in landing x: from +/-10 mm to +/-6 and +/-4 mm. Hop 0.78-0.83 and 0.88-0.93 widened to +/-20 mm.
- Columns, turned and high-low cells are unchanged or wider; oval cells gained apex band only.

## Why

- tools/admissible_sweep.py _flight_band_and_rect grows the apex band outward while the zero-offset throw passes, then fits the largest landing rectangle that passes at EVERY flight in the band.
- At 3900 the shortest hop flights (0.65-0.73 s) failed at zero offset and stopped the band. At 4500 they pass at zero offset but fail HAND_LIMIT_C2 at large x offsets, so the band grew and the rectangle shrank.
- At the operating flights (0.73 s and longer) every offset on the grid passes: the narrowing is a selection artefact, not a physical limit.

## Decision for the owner

- Keep the band-first rule (hop learner commands clamped to +/-4 mm x at apex 0.95), or change the rule so the band never grows at the cost of rectangle width. The second is a selection change over the existing pass grids (no new solves) but re-states every cell.
- Matters for BUG-43: the hop-start throw is where the landing bias lives.

## Progress

- 2026-11-02: Owner decision 2026-11-02: keep the landing rectangle as wide as possible; change the sweep's band rule so a cell never gives up landing width for apex band, then re-sweep. Next unit after the 4500 commit.
- 2026-11-03: Night Shift 2026-11-03: held on night-shift/2026-11-03-BUG-54: the band rule built (gate green, audit clean), held as an effort-m slice; unblocked by your call on TKT-34 and a re-sweep
- 2026-11-03: Merged to main 2026-11-03 per TKT-34 (option 2): the band-first rule is in tools/admissible_sweep.py and takes effect at the next re-sweep; the committed box is unchanged until then
- 2026-11-03: Box re-swept 2026-11-03 with the rectangle chosen room-around-the-target first, then area, then band (supersedes the area-ranked rule of f462835b); 104 cells, gate_hash 12ec30249323, room fell in no cell, the four 0 mm cells now 2-30 mm.
