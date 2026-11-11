---
title: "Drops to date: the way to enter the history figure (FEAT-47 slice 4)"
type: feature
date: 2026-11-11
status: complete
phase: night-shift
subsystem: skill-stack/gui
files_changed:
  - ros_ws/src/jugglebot/jugglebot/records.py
  - ros_ws/gui/js/records-model.js
  - tools/set_drops_history.py
  - tests/motion/test_records.py
  - tests/ros/test_gui_records_panel.py
  - tests/ros/js/records_panel_harness.js
  - logbook/INDEX.md
tags: [FEAT-47]
related_items: [FEAT-47]
---

# Drops to date: the way to enter the history figure

## What changed

A history backfill has counted every drop before the live counter; this slice is the way to write its figure into the records ledger, not the figure itself (that lands in a later commit).

- **Entry point:** `records.set_drops_history(path, history, through, since=, first=)`, run as `python tools/set_drops_history.py --history N --from YYYY-MM-DD --through YYYY-MM-DD [--since D] [--path docs/records.md]`. It creates the drops section when absent (live 0, `since` = `--since` or `--through`) and otherwise changes only the history fields. A malformed existing marker, a negative figure or a bad or inverted date is refused with the file untouched (exit 1).
- **Marker:** gains the optional `history_from=<date> history_through=<date>`; a marker without them parses and re-renders byte-identically, and every live write keeps them. The sentence under it names the figure and its dates: "Drops to date: 840 (12 counted live since 2026-11-12; 828 from history, 2026-07-23 to 2026-11-11)".
- **GUI and JSON:** `to_json` carries `history_from`/`history_through`; the Records panel reads "Drops to date: 840 · 12 live since 2026-11-12, 828 from history (2026-07-23 to 2026-11-11)" once a history is counted, and is unchanged for history none or not counted yet.

No motion, timing or control path is touched.

## Verification

2026-11-11, `./run_tests.sh -q`, RESULT: PASS, exit 0 (parallel 601 s, serial 31 s).
