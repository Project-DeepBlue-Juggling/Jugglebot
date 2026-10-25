---
title: "GUI Pattern labels and operator prose use Ball 1 / Ball 2"
type: refactor
date: 2026-10-26
status: resolved
phase: "ball12-labels"
files_changed:
  - ros_ws/gui/js/juggle-panel.js
  - tests/ros/test_gui_juggle_defaults.py
subsystem:
  - gui
tags:
  - glossary
  - project-map
---

## Summary

GLOSSARY.md (Ball 1 / Ball 2) says operator-facing text uses those names. The Juggle panel's Pattern labels and its "ball A phantom" cup-preview string now do (scope A of the rename survey, `map/idea-ball1-ball2-pattern-rename.md`). The mapping lives in one place, the `label` fields of `JUGGLE_PATTERNS`:

| Pattern value | old label | new label |
|---|---|---|
| `columns_1ball` | Columns (1 ball) | Columns — Ball 1 |
| `columns_1ball_fed` | Columns (1 ball, fed) | Columns — Ball 2 (fed) |
| `oval_1ball` | Oval (1 ball) | Oval — Ball 1 |
| `oval_1ball_fed` | Oval (1 ball, fed) | Oval — Ball 2 (fed) |

(The em dash is the one the diagram title already uses.) Labels feed the Event Log, so log text changes with them. `CUP_PREVIEW.needsEmptyPhantom` now reads "needs an empty cup (Ball 1 phantom)" (columns_1ball_fed: Ball 1 is the phantom); the pinned string in `tests/ros/test_gui_juggle_defaults.py` follows. Code comments in the file that said ball A / B now say Ball 1 / Ball 2 (id 0 / id 1).

## Deliberately NOT renamed

- The Pattern identifiers (`columns_1ball*`, `oval_1ball*`): `oval_1ball` is the admissible-box key and all four ride the rosbag / attempt-report join key.
- `motion/skills/sites.py` and `motion/unified_cycle.py`: their comments carry the old names but both are bytes-hashed into the box `gate_hash`; editing them forces a ~1 h re-sweep.
- Anything under `ros_ws/src` (incl. `Juggle.action` comments): would owe an interface rebuild. GLOSSARY.md, runbooks (`tests/hardware/*.md`, 57 lines quote the old labels as sitting records), the box yaml and logbook history are untouched.
- `plans/active/two-ball-oval.md`: only identifiers and a verbatim owner quote match; nothing operator-facing to change.

## Verification

2026-10-26, `pytest tests/ros/ -q -k gui`: 474 passed, 1 skipped (93 s). Live GUI needs only a browser page refresh (static files).
2026-10-26, `./run_tests.sh -q`: RESULT PASS (parallel 327 s rc=0, serial 22 s rc=0, total 349 s). An earlier run failed only on the avoid-word scan of this entry's own wording, fixed before the passing run.
