---
title: GUI juggle panel pre-fills apex and separation per pattern from the certified box points
type: bugfix
date: 2026-10-12
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / two-ball columns"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/gui/js/juggle-panel.js
  - tests/ros/test_gui_juggle_defaults.py
  - logbook/2026-10-12-gui-juggle-panel-pattern-defaults.md
  - logbook/INDEX.md
subsystem:
  - gui
tags:
  - bugfix
---

# Juggle panel per-pattern defaults

**What.** The juggle panel now pre-fills apex and separation from a `PATTERN_POINTS` table whenever the pattern
is selected. An operator edit wins until the pattern changes again, and "↺ defaults" returns to the pattern's
point rather than blanking. The points are:

- `self_toss`: 0.90 m, no separation.
- `hop`: 0.90 m, 250 mm.
- `columns`, `columns_1ball` and `columns_1ball_fed`: 0.95 m, 125 mm.

Throws stays blank, which keeps skill_node's own `n_throws`.

**Why.** Blank fields flew skill_node's single 0.9 m / 250 mm default, and a typed value stuck across pattern
switches. But the admissible box is swept at different points per pattern, so sitting 7b (launch.log
`2026-10-11-22-15-28`) had operator inputs refused:

- L547: `columns_1ball` ran at 250 mm while columns is swept at 125.
- L580, L606, L921, L987: a sticky apex of 0.90 against the columns box, whose 0.875-0.925 band is empty.
- L2706, L2725: hop at 125, while hop is swept only at 250.

The table is hand-copied, because `gui_server.py` is stdlib-only and cannot read the YAML. The new test pins
every row inside a swept box of `config/generated/admissible_box.yaml`. Hop at 125 was deliberately not swept.

**Verification.** 2026-10-12, `pytest tests/ros/test_gui_juggle_defaults.py tests/ros/test_gui_replay_ui_hide.py -q
-p no:cacheprovider`: 12 passed. Not exercised in a browser. The live GUI needs a `jugglebot-gui.service` restart
and a hard reload. Gate: in the commit message.
