---
title: "Night Shift walks only the delta: NOTHING_NEW on an empty delta, a Sunday full walk, carry from the report"
type: feature
date: 2026-11-14
status: complete
phase: "night-shift close-out"
subsystem: tools
files_changed:
  - tools/night_shift_preflight.py
  - tools/night_shift.sh
  - tests/sim/test_night_shift_preflight.py
  - tests/sim/test_night_shift_runner.py
  - docs/night-shift/README.md
  - logbook/INDEX.md
---

## What

Last night's report walked and re-listed 112 skipped Items, mostly answered question
tickets. The preflight now enforces "the night examines only the delta"
(added + changed + carry + unclassified + unblocked; `delta_why` names the first
applicable reason: full-walk, added, answered, changed, unblocked, carry, unclassified).
Question tickets enter only as added, answered or changed. `removed` is no longer news.
`candidates-<date>.json` keeps `items` and gains `delta[]`, `delta_why{}`, `unchanged`,
`full_walk`; exit 3 `NOTHING_NEW items=N unchanged=N` on an empty delta, before claude
starts. The runner passes `--full-walk` on Sundays (`NIGHT_SHIFT_WEEKDAY` overrides
`date +%u` for tests) or while `$REPORTS/full-walk.request` exists (removed after the night
that ends in a snapshot), and the post-run snapshot passes `--walked` and `--report` so
`carry[]` = tonight's delta minus the IDs in the report's Item rows. A failed night writes
no snapshot, so the next delta is taken against the older baseline.

Until the protocol (C5) walks `delta[]` it still walks the old list, and no Item has a
`night_skip` yet, so every non-question workable Item is `unclassified` and NOTHING_NEW
will not fire on the real board; that is expected.

## Verification

2026-11-14, `flock /tmp/jugglebot-tests.lock python -m pytest -n 2 -q` over
`tests/sim/test_night_shift_{preflight,runner,candidate,report,landed,gate}.py`,
`test_logbook_index.py`, `test_logbook_front_matter.py`, `test_docs_links.py`: 354 passed
and 6 failed (the fixture's fake report had no Item rows, so every night carried);
after giving the fake report rows, `test_night_shift_runner.py` plus
`test_night_shift_report.py`: 99 passed in 79 s. Preflight file alone: 34 passed.
