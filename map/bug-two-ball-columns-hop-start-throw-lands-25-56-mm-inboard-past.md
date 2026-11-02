---
id: JB-121
kind: bug
title: Two-ball Columns hop-start throw lands 25-56 mm inboard, past the 35 mm catch authority
status: complete
created: '2026-10-31'
updated: '2026-11-02T08:49:46Z'
owners: []
priority: high
effort: m
hardware: robot
owner: none
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links:
- logbook/2026-11-02-sitting-debrief.md
parent: two-ball-oval
filed_by: claude-books-2026-10-31
---

At the two-ball Columns cold start the hop-start throw lands inboard of the catch aim by more than the catch can correct, so the cup is empty and the attempt ends `MISSED_CATCH` with 0 of 0 caught. Recurring on 10-30 and 10-31.

## Evidence

- 10-31: four hop starts landed 56, 40, 37 and 25 mm inboard (x), against a catch inward authority of 35 mm (outward 10 mm); goals 1 and 3 ended 0/0, goal 2 ended 4/4 on the +62 site.
- 10-30: hop starts already ended `MISSED_CATCH` at 3/3, 8/8 and 3/3.
- 10-28 landed outward (+13, +19 mm), so the sign changed after the move to `pose` and the 0.100 hold.
- The hop throw has no learner row: `HOP-COMMAND` applies a constant P1 column correction (apex 0.877, dy -20/-18 mm, `executor.py` ~2740-2770).
- `AIM-LATERAL-CLAMPED skill 2: aim -56.0 mm clamped to -35.0` precedes each end.
- Ball 1 y scatter at the +62 site is 40-80 mm on the cold learner, against a 20 mm y authority.

## Not proven

The 2026-10-31 hold analysis shows hold 0.100 lands about 10 mm short and shorter holds land long, so hold 0 (the new default) may move this bias. No hop-start data exists at hold 0 yet.

## Fix candidates

1. Re-measure the hop start at hold 0 before any code change.
2. Make the hop correction learnable (a memory row keyed to the hop).
3. Let the first re-aim after a hop throw use a wider inward authority.

## Sitting

- setup: armed, bb-fed
- why: read the hop-start landing bias at the new default hold 0.
- do:
  - Fly Columns, Sep. 125, Apex 0.95, Throws 12, five starts from a cold hand.
- pass: five of five starts past the first two catches, with no AIM-LATERAL-CLAMPED inward of 35 mm.
- record: the HOP aim clamp line and the landing x error of each hop throw.
- minutes: 20

## Sitting result (2026-11-02)

- verdict: pass
- notes: Should be marked off with the JB-120/139 tests

## Progress

- 2026-11-01: 2026-11-01 Sitting: 7 of 10 straight hop starts passed the first two catches at hold 0. The 3 failures were HAND_STROKE at the first CATCH (hand -0.004/-0.002/-0.001 rev vs a 0.000 floor), not a lateral miss; one AIM-LATERAL-CLAMPED inward of 35 mm (-38.9 mm x, L546). Hop-start failure at hold 0 is the floor dip, not the landing bias.
- 2026-11-02: Passed at the Sitting of 2026-11-02: 16 of 16 Columns starts past the first two catches, no inward 35 mm clamp in any start window; hop-start throw landing x mean +3.8 mm, max 38 (the 25-56 mm inboard bias is gone).
