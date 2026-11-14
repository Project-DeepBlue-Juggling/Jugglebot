---
title: "Night Shift runner tests pin a weekday, so the Sunday full walk no longer fails four of them"
type: bugfix
date: 2026-11-15
status: complete
phase: "night-shift close-out"
subsystem: tests
files_changed:
  - tests/sim/test_night_shift_runner.py
  - tests/sim/test_no_legacy_item_ids.py
  - tests/sim/no_legacy_item_ids.tsv
  - logbook/INDEX.md
---

# Night Shift runner tests pin a weekday (BUG-107)

**What.** The `ns` fixture's default environment in `tests/sim/test_night_shift_runner.py`
now sets `NIGHT_SHIFT_WEEKDAY=3`. Tests of the full walk still pass
`NIGHT_SHIFT_WEEKDAY=7` themselves.

**Why.** Close-out step C4 (2026-11-14) made the runner do a full walk when
`${NIGHT_SHIFT_WEEKDAY:-$(date +%u)}` is 7. Four older tests run the runner twice
without the variable and expect `NOTHING_NEW` the second time; on a Sunday the
second run is a full walk and ends `OK`. The 04:00 nightly of 2026-11-15 (a
Sunday) was RED on exactly those four, and every Night Shift gate that night
failed condition (a). The C4 gates all ran on a Saturday, so none saw it.
The class: a test that reads the wall-clock date through the code under test.
A grep for `date +%` in `tools/night_shift.sh` shows the weekday is the only
calendar branch; the other two uses (`date +%F`, `date +%H%M%S`) only name things.

**Verification.** 2026-11-15 (a Sunday), `flock /tmp/jugglebot-tests.lock python -m
pytest -q -n 2 tests/sim/test_night_shift_runner.py`: 86 passed in 72.51 s.
Scoped tier: the only other reader of the file's path is a name list in
`tools/probes/id_cutover/_common.py` (`grep -rn test_night_shift_runner .`).

## Second red from the same night: the history ledger names old-form branches

`docs/night-shift/history.jsonl` (close-out C2) got its first row on 2026-11-15, and
that row's `pruned` list names 19 branches of the old ID scheme, so
`tests/sim/test_no_legacy_item_ids.py` failed on main from that commit on (after the
04:00 nightly, so the nightly did not show it). The test now has a guard rule
`night-shift-history-ledger`: a branch-name token in that file is `historical` and
pinned by count. No old-form branch is left on origin (`git branch -r` lists no
`night-shift/` branch with an old-form ID, 2026-11-15), so no later row can change the count. Any
other old-form token in the ledger still fails.

2026-11-15, `python -m pytest -q tests/sim/test_no_legacy_item_ids.py
tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py
tests/sim/test_docs_links.py`: 48 passed in 2.49 s.
