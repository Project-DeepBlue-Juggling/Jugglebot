---
title: "test_records asserted the committed ledger holds no landing scores: main went red when a Sitting booked some"
type: bugfix
date: 2026-11-13
status: complete
phase: "night-shift close-out"
subsystem: tests
files_changed:
  - tests/motion/test_records.py
  - logbook/INDEX.md
---

## What

`test_a_ledger_with_no_scores_renders_unchanged_and_the_committed_file_still_parses`
(added 2026-11-12 with the landing score, IDEA-21) read `docs/records.md` and asserted
it held no scores. The 2026-11-13 Sitting booked the first landing scores and the
debrief committed them (1e794f15), so the default gate failed on `main` from that
commit on, deterministically.

## Why this fix

The committed ledger is live data that every Sitting changes; a test may assert that it
parses and round-trips, never what it contains. The test now does exactly that for the
committed file, and makes the no-scores claim on a ledger it builds itself (the
committed tables and drops with the scores removed). No production code changed.

Found by the close-out stack's gate run on the rebased tip. Why the debrief's own gate
did not see it was not traced (inferred: it gated before its records commit). The close-out plan's C8 moves runtime records under
`temp/runtime` and has the debrief merge them; its merge step should gate after the
records commit.

## Verification

- 2026-11-13, `pytest tests/motion/test_records.py -q` on origin/main 1e794f15: 1 failed, 87 passed.
- 2026-11-13, `pytest tests/motion/test_records.py -q` with the fix: 88 passed in 1.50 s.
- Importers of the changed file: none (`grep -rn test_records . --include="*.py" --include="*.sh" --include="*.toml"` finds only two docstring mentions in `ros_ws/src/jugglebot/jugglebot/records.py`).
