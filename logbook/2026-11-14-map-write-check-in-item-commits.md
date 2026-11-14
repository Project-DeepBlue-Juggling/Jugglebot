---
title: "The map-write check runs inside every mapkit item commit (close-out C13)"
type: feature
date: 2026-11-14
status: complete
phase: "Night Shift close-out C13"
subsystem: tooling
files_changed:
  - mapkit/checks.py
  - mapkit/itemcli.py
  - tests/sim/test_itemcli_commit_check.py
  - tests/sim/test_map_items.py
  - tests/sim/test_map_items_rated.py
  - tests/sim/test_glossary_avoid_words.py
  - tests/sim/test_mapkit_itemcli.py
  - tests/sim/test_mapkit_item_ids_cli.py
  - .claude/commands/night-shift.md
  - docs/agents/map-writes.md
  - logbook/INDEX.md
---

## What

Rule 10 of the Night Shift protocol ("run four map tests before any `map:` push") was prose a
night or a Session had to remember; a write that broke one was found by the next gate. The four
checks (every `map/*.md` loads, an Item past `inbox` is rated, the per-block readability cap, no
GLOSSARY Avoid-word) now live in `mapkit/checks.py` as functions returning problem lists. The four
tests call them, and every `mapkit item ... --commit` calls them on the working tree before it
commits (one place: `itemcli._commit_paths`, which `set-status`, `rate`, `skip`, `skip --batch` and
`new` all use). A failure refuses the commit with exit 4, prints every problem and leaves the write
in the tree. `python -m mapkit check [--full]` runs the same checks for a hand-edited file. A ticket
filed with `--parent` is exempt from the similar-title refusal only; an exact open duplicate is
still refused. The commit path scans Avoid-words in `map/` only (a logbook offender must not block
a map write); the test and `check --full` keep the wider scope. Rule 10 is now one line.

## Why

One enforcement point, so the test and the commit path cannot drift. Not covered, by design of the
call: the map app's own writer (`server/gitwriter.py`) and a plain `git commit`.

## Verification

2026-11-14, `python -m mapkit check` on the real map: 0.76 s wall (`--full`, plans and logbook
included: 1.95 s). Tests: see the commit message for the exact commands and counts.
