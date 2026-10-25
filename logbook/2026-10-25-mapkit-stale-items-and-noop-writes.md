---
title: "mapkit: is_stale, items --stale and no-op writes (Jugglebot half of the stale-items Feature)"
type: feature
date: 2026-10-25
status: resolved
phase: "map-improvements - stale items"
files_changed:
  - mapkit/items.py
  - mapkit/itemcli.py
  - tests/sim/test_map_stale.py
  - docs/agents/map-writes.md
  - .claude/commands/work-inbox.md
  - DOCUMENTATION_GUIDE.md
subsystem:
  - docs
tags:
  - tooling
  - project-map
---

## Summary

`mapkit.items` gains `STALE_DAYS` (inbox 30, backlog 60, in_progress 14), `stale_clock(item)` (newest dated `## Progress` line, else `created`) and `is_stale(item, today)` (strictly greater than the threshold; other statuses never stale). `updated` is deliberately ignored: every write bumps it, so it measures the last save, not attention. `python -m mapkit items --stale` filters to stale items and appends `stale <n>d since <clock date>`; `/work-inbox` lists them before classifying.

`write_item` now returns a bool and skips the write when the canonical text built with the file's OLD `updated` equals the file, so a save that changes nothing neither touches the file nor bumps `updated` (the clock cannot be reset by a no-op save). `item set-status` re-setting the same status without `--note` prints "unchanged" and regenerates and commits nothing. DOCUMENTATION_GUIDE § 2.10 amended: `updated` is bumped by every write that changes the item. The Progress date matcher accepts `YYYY-MM-DD - x`, `YYYY-MM-DD: x` and the bullet forms, since live items use the colon form. The app keeps its own threshold copy and is tested against the same 10-row fixture (`tests/sim/test_map_stale.py`, ids `row1`..`row10`).

## Verification

2026-10-25, `pytest tests/sim/test_map_stale.py tests/sim/test_mapkit_itemcli.py -q`: 24 pass (10 fixture rows, `items --stale`, no-op write, unchanged set-status). Gate 2026-10-25, `./run_tests.sh -q`: parallel 327 s, serial 22 s; one failure, `tests/ros/test_replay_overview.py::test_a_pass_still_exiting_for_an_older_version...`, a GUI replay timing test that passes alone (`pytest tests/ros/test_replay_overview.py -q`: 11 passed) and touches nothing under mapkit, so read as a load flake. An earlier gate run caught `test_show_and_items` (its Namespace lacks `stale`); `_run_items` now reads it with `getattr`.
