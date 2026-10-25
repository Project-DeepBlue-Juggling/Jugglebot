---
title: "mapkit item CLI (P9a): item set-status, show and items subcommands"
type: feature
date: 2026-10-25
status: resolved
phase: "project-map - P9a"
related_plan: project-map.md
files_changed:
  - mapkit/itemcli.py
  - mapkit/__main__.py
  - tests/sim/test_mapkit_itemcli.py
  - docs/agents/issue-tracker.md
subsystem:
  - docs
tags:
  - tooling
  - project-map
---

## Summary

`python -m mapkit item set-status <slug> <status> [--note] [--link]... [--owner] [--force] [--commit] [--no-push]`, `mapkit show <pathway> [--json]` and `mapkit items [--pathway] [--kind] [--status | --open] [--json]` land in `mapkit/itemcli.py`, so the slash commands can call one tested recipe instead of repeating prose.

`set-status` validates the status against the kind's set, refuses to re-open a `complete` item or to start an item another owner holds (`--force` overrides), appends a dated line under `## Progress` (creating it), adds links without duplicates, writes through `write_item` and regenerates the docs pages. Without `--commit` it stages nothing and prints the two paths to stage, so a close can ride in the fix's commit with its `Logbook-Entry` trailer (owner's hybrid rule, 2026-10-25); with `--commit` it commits exactly the item and `docs/project-map.md` as `map: <status> <title>` (no trailer), then `pull --rebase --autostash` and push. `--autostash` because the caller may have other work staged.

## Verification

2026-10-25, `pytest tests/sim/test_mapkit_itemcli.py tests/sim/test_mapkit_tickets.py -q`: 9 new tests pass alongside the 6 ticket tests. Gate 2026-10-25, `./run_tests.sh -q`: RESULT PASS (parallel 326 s rc=0, serial 22 s rc=0, total 348 s).
