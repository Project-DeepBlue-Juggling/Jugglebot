---
id: JB-121
kind: bug
title: Two-ball Columns hop-start throw lands 25-56 mm inboard, past the 35 mm catch authority
status: inbox
created: '2026-10-31'
updated: '2026-10-31T04:51:57Z'
owners: []
priority: high
effort: m
hardware: robot
owner: none
radius: subsystem
diagnosis: hypothesis
area: skill-stack
blocked_by: []
links: []
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
