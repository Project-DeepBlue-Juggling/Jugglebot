---
title: "Map item IDs P3: one resolver for every mapkit command, item new with the push loop, ID columns and the /inbox rewrite"
type: feature
date: 2026-10-26
status: resolved
phase: "map-item-ids - P3"
related_plan: map-item-ids.md
files_changed:
  - mapkit/itemcli.py
  - mapkit/tickets.py
  - mapkit/import_issues.py
  - tests/sim/test_mapkit_item_ids_cli.py
  - tests/sim/test_map_stale.py
  - docs/agents/map-writes.md
  - .claude/commands/inbox.md
  - .claude/commands/work-inbox.md
subsystem:
  - docs
tags:
  - tooling
  - project-map
---

## Summary

Every mapkit command that names an Item now goes through `mapkit.ids.resolve` (stem, then slug, then ID): `find_item` (so `item set-status`), `claim` / `resolve` (via `find_item` plus a ticket-kind check, replacing the `ticket-%s` path join), `show` and `items --pathway`. Read commands accept bare digits; writes refuse them (GitHub issue numbers live alongside, and a one-digit typo in one global sequence lands on another real Item). Every write prints `JB-n <stem> ... <title>`; Claude-side subjects cite the ID (`map: in_progress JB-12 <title>`, `map: claim JB-9 q`), and stay unchanged for an ID-less Item.

`python -m mapkit item new` is the one Claude-side creation path: slug free across every kind (the app's uuid is slug-keyed), next ID, `--parent`/`--link` IDs stored as stems, docs regenerated; with `--commit`, at most 3 rounds of `pull --rebase` -> `ids.repair(own=[this stem])` -> push, a `map: renumber` commit when a rebase brought in the same ID, exit 2 with the local commits listed after 3 rejected pushes. `/inbox` steps 3-6 became that command; `/work-inbox` takes IDs. `items`, `show` and `frontier` carry an ID column (`test_map_stale.py`'s column index moved by one); `import-issues` allocates per ticket. Not done here: the ID column in `docs/project-map.md` (left for P6 with the normative docs).

## Verification

2026-10-26, `pytest -q` over test_mapkit_item_ids_cli, test_mapkit_ids, test_map_items, test_map_stale, test_mapkit_itemcli, test_mapkit_tickets, test_mapkit_import_issues, test_mapkit_docs, test_plans_index, test_glossary_avoid_words, test_mapkit_no_ros: 93 passed (8 new in test_mapkit_item_ids_cli.py, including the remote advancing between the check and the push).
Gate 2026-10-26, `./run_tests.sh`: RESULT PASS (parallel 330 s rc=0, 7324 passed / 13 skipped; serial 22 s rc=0; total 352 s).
