---
title: "Map app test: the next Item ID counts an origin/main Item the checkout has not pulled"
type: feature
date: 2026-11-08
status: complete
phase: night-shift
files_changed:
  - tests/test_ids.py
subsystem:
  - map
tags:
  - map-app
  - item-ids
  - BUG-36
---

# Map app test: next ID counts an unpulled origin/main Item

**What.** BUG-36 (jugglebot-map repo, `tests/test_ids.py`): `test_an_item_on_origin_not_yet_pulled_still_bumps_the_next_id`. A bare remote and a clone with JB-1..5; a second clone pushes JB-6 and JB-7; the first clone fetches (tracking ref moves, tree does not); creating an Item through the API must answer JB-8. No production code changed.

**Why no production change.** `server/store.py` `_next_id` calls `mapkit.ids.allocate`, which takes the max over the tree and `origin/main` (`mapkit/ids.py:336-337`). Nothing fetches first, so a ref that was never fetched is still stale; that case ends in the git writer's post-rebase repair (`server/gitwriter.py` `_repair_ids`, `_own_unpushed`), already pinned by `test_two_own_unpushed_items_colliding_after_the_rebase_are_both_renumbered` and `test_a_rejected_push_then_the_next_flush_repairs_and_pushes`. No collision reaches `origin`.

## Verification

- Without the change (2026-11-08): `_next_id` temporarily replaced by an index-only count; `pytest -q tests/test_ids.py -k not_yet_pulled` FAILED (the POST answered JB-6). Restored afterwards.
- With it (2026-11-08, `tools/night_shift_nonet.sh env MAPKIT_SRC=... python -m pytest -q tests/test_ids.py tests/test_p5_writer_watcher.py`, app clone): 29 passed, 2 skipped (includes the new test).
