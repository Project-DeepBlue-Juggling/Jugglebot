---
title: "Drops to date: the GUI line shows the total only, with an effect when it rises"
type: feature
date: 2026-11-11
status: complete
phase: night-shift
subsystem: skill-stack/gui
files_changed:
  - ros_ws/gui/js/records-model.js
  - ros_ws/gui/js/records-panel.js
  - ros_ws/gui/css/records-panel.css
  - tests/ros/test_gui_records_panel.py
  - tests/ros/js/records_panel_harness.js
  - logbook/INDEX.md
tags: [FEAT-47]
---

# Drops to date: the GUI line shows the total only

## What changed

The line atop the Records panel read "Drops to date: 819 · 0 live since 2026-11-11, 819 from
history (2026-07-23 to 2026-11-10)". The owner asked for the total alone (2026-11-11), so it now
reads "Drops to date: 819". The breakdown moved to the line's hover text. When the total rises,
the line plays a 0.9 s pop and colour flash (off under `prefers-reduced-motion`). The first load
of a page never plays it, and neither does a total that falls.

## How the total updates

The panel re-reads `/api/records` on every Attempt end (`juggle_end`), which is when `skill_node`
books an Attempt's drops into the ledger. So the number rises without a page reload at the end
of the Attempt that dropped, not at the instant of each drop inside an Attempt. Most drops end
their Attempt, so the two usually coincide; the exception is a two-ball Attempt that continues
after losing one ball.

## Also learned

The GUI server (`jugglebot-gui.service`, a system unit) keeps `records.py` loaded in memory. After
a change to `records.py` the panel shows stale data until the service is restarted
(`sudo systemctl restart jugglebot-gui`): on 2026-11-11 the panel read "not counted yet" against a
ledger that held 819, because the server process dated from 2026-11-08. A `colcon build` does not
restart it.

## Verification

- 2026-11-11, `pytest tests/ros/test_gui_records_panel.py -q`: 16 passed.
- The gate triple is in the commit message.
- Not checked in a browser by this Session.
