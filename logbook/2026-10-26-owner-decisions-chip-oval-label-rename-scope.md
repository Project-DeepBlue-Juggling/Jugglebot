---
title: "Owner decisions: nightly chip stays on, 'Oval — fed' label, Ball 1 / Ball 2 rename stays at the labels, JB prefix kept"
type: refactor
date: 2026-10-26
status: resolved
phase: "map-improvements"
files_changed:
  - ros_ws/gui/js/juggle-panel.js
  - CLAUDE.md
  - plans/active/map-item-ids.md
  - map/feature-nightly-status-chip-in-the-map-header.md
  - map/idea-ball1-ball2-pattern-rename.md
subsystem:
  - gui
  - project-map
tags:
  - project-map
  - glossary
---

## Summary

On 2026-10-26 the owner answered four questions batched while they were away:
- **Nightly chip:** the map app's read-only chip for the 04:00 run stays on. The owner confirmed it does not reverse the "no email, no GUI" choice. The CLAUDE.md nightly sentence now says so, and Claude sessions still claim the ticker. This closes the Feature (`map/feature-nightly-status-chip-in-the-map-header.md`).
- **Oval label:** the fed one-ball oval's label changes from "Oval — Ball 2 (fed)" to "Oval — fed" in `JUGGLE_PATTERNS` (`ros_ws/gui/js/juggle-panel.js`). The oval has a single ball, so "Ball 2" read as a second one. The Pattern value `oval_1ball_fed` is unchanged. The other three labels stay as set in `2026-10-26-ball12-gui-labels`.
- **Rename scope:** the Ball 1 / Ball 2 rename stays at the labels. The identifiers `columns_1ball*` and `oval_1ball*` are the admissible-box key and the rosbag join key, so they are not renamed. This closes the Idea `map/idea-ball1-ball2-pattern-rename.md`.
- **Prefix:** the item ID prefix `JB` is confirmed, recorded in `plans/active/map-item-ids.md` § 7.

## Verification

- 2026-10-26, `./run_tests.sh`: RESULT PASS (parallel 7361 passed, 9 skipped, 336 s; serial 5 passed, 22 s), run on these edits before this Verification text was written.
- 2026-10-26, after that edit: `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py tests/sim/test_plans_index.py tests/ros/test_gui_juggle_defaults.py -q` (the tests that read the changed paths), result in the commit message.
- The live GUI needs a hard refresh, and only once the live checkout pulls main.
