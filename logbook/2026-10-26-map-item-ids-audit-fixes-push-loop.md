---
title: "Map item IDs: item new push loop fixes from the P0-P3 audit (upstream-aware repair, rebase abort, validate before write, frontmatter-only ref scan)"
type: bugfix
date: 2026-10-26
status: resolved
phase: "map-item-ids - P0-P3 audit fixes"
related_plan: map-item-ids.md
files_changed:
  - mapkit/ids.py
  - mapkit/itemcli.py
  - tests/sim/test_mapkit_ids.py
  - tests/sim/test_mapkit_item_ids_cli.py
  - docs/agents/map-writes.md
subsystem:
  - docs
tags:
  - tooling
  - project-map
---

## Summary

The phase audit of P0-P3 reproduced four behaviour bugs in `mapkit item new --commit` and `ids`:

1. **Duplicate pushed on a feature upstream.** `repair` picked the keeper from `origin/main` while the loop rebased onto the branch's upstream; an Item that arrived via the rebase was no holder there, both sorted at add-order infinity, and when our stem sorted first nothing was renumbered. Now `ids.upstream_ref` (`@{u}`, else `origin/main`) drives allocation and repair, `allocate` takes the max over that ref and `origin/main`, and with `own=` any group member outside it is the keeper: a file the writer has not pushed can never be the older holder.
2. **Repo left mid-rebase.** A failed `pull --rebase --autostash` now aborts the rebase and raises naming the unmerged paths, the local commit and any pending autostash. After a clean rebase `docs/project-map.md` is regenerated and amended into the commit before the push (a textual merge of two generated tables need not equal a fresh gen).
3. **Invalid file left in `map/`.** An Item that fails validation (pathway without `--destination`, ticket without a parent) is removed, so later loads keep working; a git failure in `item new` is reported, not a traceback.
4. **Body `id:` lines inflated `allocate`.** `ref_ids` keeps only an `id:` between the frontmatter's opening and closing `---` (the tree scan already read frontmatter through the loader).

`docs/agents/map-writes.md` states the rebase-conflict outcome.

## Verification

2026-10-26, `pytest -q tests/sim/test_mapkit_item_ids_cli.py tests/sim/test_mapkit_ids.py`: 30 passed (6 new). The upstream test fails on the pre-fix `ids.py` for the `aardvark` (ours sorts first) order and passes after; the slug-clash test fails pre-fix on the missing abort.
Gate 2026-10-26, `./run_tests.sh`: RESULT PASS (parallel 333 s rc=0, 7337 passed / 13 skipped; serial 22 s rc=0; total 355 s).
