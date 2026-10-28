---
title: "Item metrics: six optional validated fields (effort, hardware, owner, radius, diagnosis, area), CLI, project-map columns, quick-capture pills"
type: feature
date: 2026-10-28
status: resolved
phase: "inbox-wall"
files_changed:
  - mapkit/items.py
  - mapkit/itemcli.py
  - mapkit/plans.py
  - mapkit/steps.py
  - mapkit/gen_docs.py
  - docs/project-map.md
  - docs/agents/map-writes.md
  - DOCUMENTATION_GUIDE.md
  - tests/sim/test_map_items_rated.py
  - .claude/commands/inbox.md
  - ros_ws/gui/js/quick-capture.js
  - ros_ws/gui/index.html
  - ros_ws/gui/css/app-tabs.css
  - tests/sim/test_mapkit_metrics.py
  - tests/sim/test_mapkit_docs.py
  - tests/ros/test_gui_quick_capture.py
subsystem:
  - docs
  - gui
tags:
  - inbox-wall
  - project-map
---

## Summary

Map items gain six optional lowercase-enum fields after `priority` (owner-approved spec,
2026-10-28; `DOCUMENTATION_GUIDE.md` § 2.10): `effort`, `hardware`, `owner`, `radius`,
`diagnosis` (bugs only) and `area`. They are optional on a new item (quick capture stays one
line) and validated by the loader when present; `mapkit.items.METRIC_ENUMS` is the one
in-code table. Plan steps may carry `effort` / `hardware` (validated by `check_step_vocab`).

- `item new` takes `--effort --hardware --owner --radius --diagnosis --area`; the new
  `item rate JB-n <flags>` sets them on an existing item (canonical key order, bumps
  `updated`, optional `--commit`). Both validate before writing anything.
- `mapkit gen`: the Inbox table gains Effort / HW / Owner / Radius / Area columns and a
  legend line; `docs/glossary.md` is untouched.
- Quick-capture form: two optional pill rows (effort, hardware), none preselected, a second
  click clears one; POSTed as `effort` / `hardware` only when chosen; kept in the draft.
- The backfill (same batch) rated all 87 items and 30 plan steps; `tests/sim/test_map_items_rated.py`
  pins that every item past triage stays rated (`inbox` items are exempt, per the
  optional-on-new rule). The audit also caught that a second click on a chosen pill was
  re-checked by the label's synthetic click; the handler now cancels that activation.

## Verification

- 2026-10-28, `./run_tests.sh` (K1 worktree, before the backfill merge): PASS, parallel 7465 passed / 13 skipped (374 s, rc=0), serial 5 passed (rc=0).
- 2026-10-28, `./run_tests.sh` (merge worktree, schema + backfill + JB-80 follow-up): PASS, parallel 7553 passed / 13 skipped (371 s, rc=0), serial 5 passed (rc=0).
- The audit-fix commit's re-run is cited in that commit's message.
