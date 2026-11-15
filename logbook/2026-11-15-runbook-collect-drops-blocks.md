---
title: "mapkit's Runbook collect() drops the blocks and rigs compatibility fields; the app serves rows per rig"
type: refactor
date: 2026-11-15
status: complete
phase: "night-shift close-out"
subsystem: mapkit
files_changed:
  - mapkit/sitting.py
  - tests/sim/test_sitting_sections.py
  - plans/active/night-shift-closeout.md
  - logbook/INDEX.md
---

# collect() returns rows only (close-out C11 c)

**What.** `mapkit.sitting.collect()` returns `rows`, `needs_section` and `questions`.
The `blocks` and `rigs` fields, kept on 2026-11-14 for the map app of that day, are
gone. The app went first (jugglebot-map 09e879b, deployed 2026-11-15): `GET
/runbook` now answers `rigs[].rows` built from `rows` alone, and its dead "Before
power" (Setup 0) parser is deleted. `RunbookRow.score` stays, always 0, because
the app still serialises it.

**Why.** The Runbook page has been an index since C11 (a); nothing groups rows
any more, and a field that looks like grouping invites a reader to order by it.
Order of landing matters: an app that reads `blocks` against a mapkit without it
answers 500 on the Next Sitting page.

**Verification.** Readers of `collect()`: `grep -rn "sitting.collect\|collect(" mapkit
tests tools .claude` finds `mapkit/sheet.py` (reads `rows`), `render_runbook` and
`tests/sim/test_sitting_sections.py`. 2026-11-15, `python -m pytest -q
tests/sim/test_sitting_sections.py tests/sim/test_mapkit_sheet.py
tests/sim/test_sitting_debrief_protocol.py tests/sim/test_night_shift_gate.py
tests/sim/test_mapkit_gen.py`: 303 passed in 167.64 s. App, same day:
`MAPKIT_SRC=<this tree> pytest -q tests/test_runbook.py`: 7 passed; before the
mapkit change the app's full suite passed 146 and its frontend 377 (1 skipped).
Live after the app deploy: `curl http://localhost:8765/api/v1/runbook` returns
keys `needs_section, page, questions, rigs` with 11, 1 and 2 rows for robot,
bench leg and bench Hand.

Still open in C11 (c): the GLOSSARY entries "Runbook" and "Next Sitting Runbook",
which change only through grilling and domain-modeling with the owner.
