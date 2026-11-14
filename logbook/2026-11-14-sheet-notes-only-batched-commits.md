---
title: "Sitting sheet results: notes only (schema 2), verdicts derived and confirmed at the debrief (close-out C9a)"
type: feature
date: 2026-11-14
status: complete
phase: "night-shift close-out C9"
subsystem: tooling
files_changed:
  - mapkit/sheet.py
  - mapkit/sitting.py
  - tests/sim/test_mapkit_sheet.py
  - logbook/INDEX.md
---

## What

The owner's live pass/fail on the Sitting sheet was wrong, missing or contradicted by the
logs on seven Sittings, and the app's one-commit-per-step saves re-triggered work
elsewhere 36 times a day. The mapkit half of the fix:

- **Results schema 2.** `verdicts[n][ID] = {notes, saved_at}`: no verdict field. A
  `verdict` key in a save is refused. A verdict step may also take a `status`, so a step
  is completable with no notes. A schema 1 file still loads and its verdicts are kept.
- **The debrief's three calls**, all in `mapkit/sheet.py` and on the CLI:
  `mapkit sheet derive <name> <file.yaml>` (sets `derived[n][ID] = {verdict, evidence,
  criterion, by}`), `mapkit sheet confirm <name> --at "YYYY-MM-DD HH:MM" [--changed ID...]`
  (sets `confirmed = {at, changed}`; refuses until every verdict-carrying Item has a derived
  entry) and `mapkit sheet close <name>` (writes each Item's `## Sitting result` once).
- **Exactly once.** `close` refuses a file with no `confirmed`, a missing derived verdict, a
  schema 1 file (its verdicts already went into the Items live) and a file already `closed`;
  it writes `closed = {at, items}` last and skips an Item that already holds this sheet's
  marker, so a run that died halfway can be repeated.
- `sitting.VERDICTS` gains `data` and `inconclusive` (closes FEAT-42; the map file is untouched).

The app half (batched commits, notes-only UI) is in the jugglebot-map repo, branch
`closeout-c9`. Neither half works against the other's old half: the old app sends a verdict,
which the new `apply_step_result` refuses; the new app sends notes only, which the old one
rejects as a bad verdict. So the two land together, outside a Sitting: merge this repo, then
deploy the app straight after (the auto-pull restarts the server on a mapkit change, so the
window between them is the pull interval).

## Verification

2026-11-14, `pytest tests/sim/test_sitting_*.py tests/sim/test_mapkit*.py
tests/sim/test_night_shift_candidate.py tests/sim/test_night_shift_landed.py
tests/sim/test_logbook_index.py tests/sim/test_logbook_front_matter.py
tests/sim/test_docs_links.py -q -n 2`: 432 passed in 39.95 s. New tests in
`tests/sim/test_mapkit_sheet.py`: notes-only apply, verdict refused, done-without-notes,
schema 1 round trip, derive validation, confirm gate, close once and resume, schema 1 refusal.
