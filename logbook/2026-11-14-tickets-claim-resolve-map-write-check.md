---
title: "mapkit claim and resolve run the map-write check before committing (close-out C16)"
type: bugfix
date: 2026-11-14
status: complete
phase: "Night Shift close-out C16"
subsystem: tooling
files_changed:
  - mapkit/tickets.py
  - tests/sim/test_mapkit_tickets.py
  - logbook/INDEX.md
---

## What

`mapkit claim` and `mapkit resolve` committed their ticket file in `tickets._commit` without the
map-write check that every `mapkit item ... --commit` has run since C13
(`logbook/2026-11-14-map-write-check-in-item-commits.md`). `_commit` now calls
`itemcli._check_before_commit` first; a failure raises `MapCheckRefused`, and the two CLI
runners print it and exit 4 (the same code `item` uses), with nothing committed and the write left
in the working tree. No other behaviour changes. The existing tests' fixture tickets and pathway were
unrated, which the check refuses, so `tests/sim/test_mapkit_tickets.py` now rates them
(`_RATED`) and gains `test_claim_and_resolve_refuse_a_commit_that_breaks_the_map_checks`.

## Why

A write path that commits map files outside the one check is how a `map:` push paints the gate
RED. The same commit realigns the standing docs with the 2026-11-13/14 close-out (CLAUDE.md ticker
rule and box-wide test lock, `map/pathway-night-shift.md`, `plans/active/night-shift.md`,
`.claude/commands/inbox.md`, `docs/night-shift/README.md`); that is pure rewording of behaviour the
2026-11-14 entries on the delta walk, the runner housekeeping, the report input section and the
sitting merge script already record.

## Verification

2026-11-14, `python -m pytest tests/sim/test_mapkit_tickets.py tests/sim/test_question_tickets.py -q`:
25 passed.
