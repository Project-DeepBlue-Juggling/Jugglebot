---
title: "Map item IDs P1-P2: mapkit ids module, uniqueness test and the JB-<n> backfill"
type: feature
date: 2026-10-26
status: resolved
phase: "map-item-ids - P1, P2"
related_plan: map-item-ids.md
files_changed:
  - mapkit/ids.py
  - mapkit/items.py
  - mapkit/__main__.py
  - tests/sim/test_mapkit_ids.py
  - tests/sim/test_map_items.py
subsystem:
  - docs
tags:
  - tooling
  - project-map
---

## Summary

`mapkit/ids.py` gives map Items a short ID `JB-<n>` (one global sequence, frontmatter `id:` as the first key). `allocate` is 1 + max over the working tree and `origin/main` (`git grep`, no network); `duplicates`, `check` (duplicate, malformed, and an id changed against `origin/main` with no `map: renumber` commit; ID-less Items are reported but not a failure yet), `repair` (keeper = the Item holding the ID on the ref, else the earliest by created, first-add commit position on the ref, stem; `own=` limits a writer to its own files), `assign_missing` (the backfill, same order) and `resolve` (stem, then slug, then ID; bare digits only with `allow_bare`, i.e. read commands; a slug/ID disagreement is an error). CLI `python -m mapkit ids [--check | --assign-missing | --repair] [--ref]`; `mapkit gen` warns on duplicates. `Item.short_id`; `dump_item` writes `id` first.

`load_item` deliberately does not validate `id`: `load_items` raises on the first bad Item and the app hides a malformed file, so a hand-typed `jb-12` must not make an Item vanish. The gate pins format and uniqueness instead (`test_item_ids_unique`); the presence test waits for P4, when every writer allocates.

First-add ordering uses commit *position* (`git log --reverse --diff-filter=A`), not `%ct`: two commits in the same second tied and fell back to stem order in the first test run.

P2 backfill (`python -m mapkit ids --assign-missing`, its own `map:` commit): JB-1..JB-62 in created order; only `id:` lines inserted, `updated` untouched.

## Verification

2026-10-26, `pytest -q tests/sim/test_map_items.py tests/sim/test_mapkit_ids.py tests/sim/test_mapkit_items.py tests/sim/test_mapkit_itemcli.py tests/sim/test_mapkit_no_ros.py tests/sim/test_mapkit_gen.py`: 46 passed (15 new in test_mapkit_ids.py).
Gate 2026-10-26, `./run_tests.sh` with P1 and the P2 backfill both in the tree: RESULT PASS (parallel 363 s rc=0, 7301 passed / 13 skipped; serial 22 s rc=0; total 385 s). After the backfill, `python -m mapkit ids --check` exits 0 with no missing Items.
