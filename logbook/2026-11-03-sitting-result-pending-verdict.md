---
title: "A Sitting result can be typed as pending: flown, but the pass needs the logs; /sitting-debrief derives the verdict"
type: feature
date: 2026-11-03
status: complete
phase: ""
files_changed:
  - mapkit/sitting.py
  - tests/sim/test_sitting_sections.py
  - .claude/commands/sitting-debrief.md
  - docs/agents/map-writes.md
  - docs/runbook.md
  - logbook/INDEX.md
subsystem: tools
---

# Sitting result verdict `pending`

**What.** The Next Sitting page's result strip offered only Pass and Fail. Some rows cannot
be judged during the Sitting because their `pass:` needs a launch log, a bag or an analysis.
The owner asked for a way to mark such a row flown with its verdict still open. A
third verdict, `pending`, now exists:

- `mapkit/sitting.py`: `VERDICTS = ("pass", "fail", "pending")` is the one list of
  verdicts; `append_sitting_result` validates against it, and the Runbook's Close-out line
  names the new choice. The map app (jugglebot-map fork, same day) reads `VERDICTS` from
  the checkout's mapkit, falling back to pass/fail on an older checkout, and shows a third
  **Pending** button with an amber result.
- `/sitting-debrief` § 2.3: a `pending` result takes section 1's log-derived verdict,
  which the owner confirms in the C table. Sections 3 onward merge or close on a
  pending-resolved-to-pass exactly as on a typed pass. An `unverified` one is neither closed
  nor merged and gets a Progress line naming what is missing. The typed section stays on
  the Item. The Night Shift's "Unaddressed Sitting results" table lists `pending` like any
  other verdict and never merges on one.

**Verification.** 2026-11-03, `python -m pytest -q -n 4` over every test file that reads
`mapkit` or the debrief/map-writes docs (`grep -rln "mapkit\|sitting-debrief\|map-writes\|docs/runbook" tests/`):
**543 passed, 38 skipped, exit 0**. jugglebot-map against this mapkit
(`MAPKIT_SRC=~/Desktop/Jugglebot python -m pytest -q`): **101 passed**; frontend
`npm run lint`, `tsc -b`, `vitest run src/pages/NextSitting`: **7 passed**.
