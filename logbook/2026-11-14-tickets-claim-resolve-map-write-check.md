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

Docs pass (commit 6bf1a1d5), 2026-11-14: `python -m mapkit check --full`: clean.
`flock /tmp/jugglebot-tests.lock python -m pytest tests/sim/test_mapkit*.py test_night_shift_gate.py
test_night_shift_protocol.py test_plans_index.py test_docs_links.py test_logbook_index.py
test_logbook_front_matter.py test_question_tickets.py test_map_items.py test_map_items_rated.py
test_glossary_avoid_words.py test_night_shift_candidate.py test_sitting_debrief_protocol.py
test_sitting_merge.py test_night_shift_report.py test_itemcli_commit_check.py -q -n 2`:
802 passed, 61 skipped in 113.06 s. `test_night_shift_runner.py`, same lock, `-n 2`: 86 passed in 72.42 s.

Audit-fix commit, 2026-11-14: `claim` now commits a kept write on retry (a refused claim left the
owner in the file, so the retry returned early): `test_claim_retry_after_a_refused_check_commits_the_kept_write`
in `tests/sim/test_mapkit_tickets.py`; and the Night Shift protocol pin
`test_commit_merge_and_item_commands_carry_no_free_text` in `tests/sim/test_night_shift_protocol.py`
(no `-m "`, `&&`, heredoc or `<title>` in any commit, merge or `mapkit item` example). Results in that commit's message.
