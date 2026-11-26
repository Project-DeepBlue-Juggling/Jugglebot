---
id: BUG-140
kind: bug
title: Fresh REST rebases outside the band are uncapped (up to 1.0 s logged) and land off the knot grid
status: in_progress
created: '2026-11-25'
updated: '2026-11-26T17:23:22Z'
owners:
- night-shift
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
night_skip: input-decision
night_skip_on: '2026-11-26'
night_skip_note: Builds the change you choose with grid and cap tests, writes its Sitting section, then holds it for a Sitting.
night_skip_input: 'Pick option 1-4 in TKT-96: cap and snap to the Knot grid, snap only, cap only at a bound you name, or leave both.'
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

## Sitting

- setup: level, armed, bb-fed
- why: offline, a REST rebased from a seed at rest outside the BUG-95 band now starts on the first Knot at or after the wire read (up to one Knot later than before), and its install line reports the slide in whole Knots. Only flown Attempts show that rebased RESTs still fly with no new refusal, and they give the slide table that the cap decision (TKT-96) waits on.
- do:
  - Launch from branch night-shift/2026-11-27-BUG-140 and fly the Sitting's Ball-Butler-fed Attempts as planned; no extra step: the change shows on every REBASED REST, mostly the long pre-tilt RESTs.
- pass: every skill_node `fresh origin REBASED` line for a non-band REST ends in `; slide N knots on the knot grid, snapped +X s past the wire read` with X under 0.025; no refusal code absent from the 2026-11-20 to 11-26 launches; no TILT_PIN or LIMIT refusal in the REST that follows a REBASED install or in the Catch spliced into its tail.
- record: the launch log path, the count of REBASED lines (band and non-band), each slide in Knots (max, median, count over 8), and the refusal codes with their counts.
- minutes: 10
- branch: night-shift/2026-11-27-BUG-140

## Progress

- 2026-11-27: Night Shift 2026-11-27: held on night-shift/2026-11-27-BUG-140: snap to the Knot grid, no cap (TKT-96 option 2); 2 audit findings open; unblocked by TKT answer, then a Sitting
