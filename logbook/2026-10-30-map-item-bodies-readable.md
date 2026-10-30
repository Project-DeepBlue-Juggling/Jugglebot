---
title: "Map: Item bodies must be readable - 49 wall-of-text Items restructured, a length rule in mapkit and a gate test keep them that way"
type: feature
date: 2026-10-30
status: resolved
phase: "map-improvements"
files_changed:
  - mapkit/items.py
  - tests/sim/test_map_items.py
  - tests/sim/test_mapkit_items.py
  - docs/agents/map-writes.md
  - .claude/commands/inbox.md
  - .claude/commands/work-inbox.md
  - logbook/2026-10-30-map-item-bodies-readable.md
subsystem:
  - project-map
tags:
  - map-improvements
  - readability
---

## What changed

The owner found JB-114 illegible in the app's drawer: it was filed as one 3,500-character paragraph. 49 of 115 Items had the same problem, as either a body paragraph over 500 characters or a Progress line over 400. Claude writers produced most of them, through `item new --body` and `set-status --note`.

- **Rule, one enforcement point**: `mapkit.items.readability_violations`. A paragraph or list item (with its continuation lines) is at most 500 characters, a `## Progress` line at most 400, and a body over 1,200 characters (Progress excluded) needs a `##` heading. Tables and fenced code are exempt. The caps were calibrated on the owner-approved JB-114 rewrite, which passes them with room to spare.
- **Gate**: `tests/sim/test_map_items.py::test_item_bodies_are_readable` checks every committed Item. `/inbox` already runs that file before its push, so a filing fails before it reaches `main`. `test_readability_rule_clauses` pins each clause.
- **Guidance**: `docs/agents/map-writes.md` § "Writing an Item body" covers sections, bullets and tables, `--body-file` for structured bodies, and a short Progress note whose detail goes in a dated `## <topic> (YYYY-MM-DD)` section. `/inbox` and `/work-inbox` point at it.
- **Backfill**: five Sonnet agents restructured the 49 Items with no fact changed. Long Progress lines kept their dated headline and moved their detail into a dated section just above `## Progress`, so every Item's staleness clock is unchanged. That commit is separate (`map: restructure 49 Item bodies for readability`), so it can be reverted alone.

Nothing is screened at write time: app edits, `item new --body` and `set-status --note` are caught when the scoped map tests run (`/inbox` step 4, `docs/agents/map-writes.md` Writes step 4) or by the gate.

## Verification

- 2026-10-30, a scratchpad checker comparing every rewritten Item with its original at the base commit `e932402d`: frontmatter byte-identical on all 49 Items. No number, identifier or capitalised word was lost; the only lost words were connectives (and, with, or, while, then, on, is). `stale_clock` was unchanged on every Item.
- 2026-10-30, `./run_tests.sh` in the commit worktree, after the audit fixes (only an H1/H2 opens or closes Progress, as in `_progress_dates`; only `##` satisfies the heading clause; `~~~` fences exempt): **7569 passed, 64 skipped in 337.31 s; serial phase 5 passed**.
