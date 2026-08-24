---
title: Operator observability quartet — plan opened on its own worktree branch
type: feature
date: 2026-08-21
status: in-progress
phase: "operator-observability planning"
related_plan: operator-observability.md
files_changed:
  - plans/active/operator-observability.md
  - plans/active/INDEX.md
  - logbook/INDEX.md
subsystem:
  - gui
  - can
  - ros
  - mocap
tags:
  - planning
  - observability
---

# Operator observability quartet — plan opened

## Summary

Owner requested four features 2026-08-21: (F1) GUI charts in physical units
(mm / mm/s legs+hands, absolute deg / deg/s BB pitch), (F2) the ROS2 Topics
panel toggleable to true per-type UDP message rates, (F3) ODrive errors decoded
by name+axis through to the launch shell and GUI event log, (F4) BB Calibrate
refused/skipped when QTM is not delivering BB markers — plus calibration
hardening (arc-span floor, mid-sweep invalidation, collection timeout).

Four read-only survey agents mapped the current state; the headline findings
are in the plan: the full ODrive error bitmasks already reach `/robot_state`
(detail is destroyed Jetson-side only, decode table exists unused), and
`teensy_link.LinkStats` already counts RX per message type (unpublished). All
four features are software-only — no firmware flash.

Work proceeds on worktree `~/Desktop/Jugglebot-obs`, branch
`2026-08_operator-observability`, based at `5de4a1c` — the close of the
hand-tuning/throw-envelope unit, immediately before the ILC fold-in merge —
so it runs fully independent of the live ILC session in the main tree.
Order F3 → F4 → F1 → F2; two audit points (one per pair); details and scope
decisions (owner, 2026-08-21) in `plans/active/operator-observability.md`.

## Verification

Docs-only. Coverage traced, not assumed:

- The plan + its `plans/active/INDEX.md` row are covered by
  `tests/sim/test_plans_index.py` (`test_every_plan_has_an_index_row`,
  `test_index_mentions_no_plan_that_left_its_directory`,
  `test_active_index_row_has_a_status_and_a_scope`,
  `test_no_plan_filename_is_claimed_by_two_boards`).
- This entry's front matter is covered by
  `tests/sim/test_logbook_front_matter.py::test_every_entry_has_required_front_matter`
  and `::test_every_entry_file_is_loaded`.
- ⚠ `logbook/INDEX.md` is covered by NOTHING — `sim/analysis/logbook_search.py`
  `_SKIP_FILES` excludes it and no test in `tests/` reads it, so the new row was
  checked by hand (link resolves; date/status/phase match this entry's front
  matter). See `logbook/README.md` § "What the logbook tests actually check".

`pytest tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py
tests/sim/test_logbook_search.py -q` (run 2026-08-21): **103 passed in 0.60 s**.
