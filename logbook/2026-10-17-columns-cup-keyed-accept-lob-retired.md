---
title: "Columns P1 + P2: accept rule keyed on (pattern, cup evidence), human-lob columns feed retired"
type: refactor
date: 2026-10-17
status: in-progress
phase: "two-ball-skill-stack - post-R6 (columns empty-cup two-feed, P1)"
related_plan: columns-empty-cup-two-feed.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/hardware/session_skill_stack_rehearsals.md
  - tests/ros/test_skill_node.py
  - ros_ws/src/jugglebot_interfaces/action/Juggle.action
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/gui/css/juggle-panel.css
  - ros_ws/gui/test_juggle_panel.html
  - tests/hardware/skills_plan_bench.py
  - tests/ros/conftest.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_gui_juggle_defaults.py
  - tests/ros/test_skill_node_reload_only.py
  - tests/ros/test_skills_plan_bench.py
  - logbook/2026-10-17-columns-cup-keyed-accept-lob-retired.md
subsystem:
  - skills
  - ros
tags:
  - testing
---

## Summary

Phase 1 of `plans/active/columns-empty-cup-two-feed.md`. The accept rule is now
a function of (pattern, cup evidence) and the human-lob columns feed (decision
D2) is gone. Columns is Ball-Butler-fed only.

## Changes

- `accept_cup_decision` + `_CUP_RULE` in `skill_node.py` replace `accept_cup_requirement`. EMPTY makes self_toss / hop / columns_1ball reload-first; SEATED makes them throw-first; UNKNOWN refuses every pattern (`REJECTED_BALL_UNKNOWN`). reload / columns_1ball_fed need EMPTY; magazine needs SEATED. Columns EMPTY is refused `REJECTED_NO_BALL` until P3 (the two-feed chain).
- The node ignores `goal.reload` (field removed at P2). The derived reload still goes into the attempt label and the start event (`reload='1'/'0'`).
- Deleted: lob branch of `_run_columns`, `_maybe_resolve_columns_feed`, `await_feed`, `COLUMNS_FEED_BOUND_*`, `COLUMNS_TRACKER_FEED_DEADLINE_S`, `ABORTED_NO_COLUMNS_FEED`.
- Caveat for bag readers: bags before this entry carry columns `reload='1'` meaning "ball B from BB", not "ball A reloaded".
- `INVARIANTS.md` rows 58 / 119 / 120 and rehearsal rows 21-22 amended.
- Tests: 12 deleted; `_columns_fed` helper rewritten onto the BB announcement path; `test_accept_cup_decision_table` (21 cells) and six accept tests added; nine columns tests gained a `_reload_ready` line because a seated columns goal now always calls BB.

## Verification

- 2026-10-17, `pytest tests/ros/ -q -p no:cacheprovider`: 3114 passed, 2 skipped in 537 s.
- Full gate (`./run_tests.sh --full`, run 2026-10-17): **6716 passed, 9 skipped in 305.80 s** (parallel phase) + **7 passed in 27.87 s** (serial tail), exit 0.

## Phase 2 (same day)

- Interface: `bool reload` deleted from `ros_ws/src/jugglebot_interfaces/action/Juggle.action` (interface rebuild; a FULL launch restart is required before the next sitting). `orchestrator_node._parse_juggle_request` now refuses a `reload` token after the pattern as an unknown token; the bare `reload` PATTERN still dispatches. Bench `--reload` and the fake goal's reload field removed.
- GUI (`juggle-panel.js`): the "Reload first" checkbox, `effectiveReload`/`reloadRequired` and the request-string `reload` token are gone. A cup PREVIEW line (`#juggle-cup-preview`, strings in exported `CUP_PREVIEW`) reads the existing ball-held pill that `panels.js` drives from `/hand_telemetry` (no second subscription): "cup EMPTY: Ball Butler loads first" / "cup SEATED: throw first" / "cup UNKNOWN: will be refused", with fixed-requirement text for reload, columns_1ball_fed and magazine. The node's decision stays the authority; the preview is advisory.
- Diagram: lob branch/arrow removed, `m.feed` became `m.feeds[]` (columns SEATED = B, EMPTY = A then B, "BB x2"); three dead lob CSS rules deleted. The mock harness takes `?cup=held|empty|unknown`.
- Rehearsal rows 19/21/22 drop `reload: true`; historical sitting records untouched. Stale `reload=True/False` docstrings in `schedule.py`, `skill_node.py` and `test_skill_node.py` reworded.
- Verification (P2): scoped, run 2026-10-17, `pytest tests/ros/test_orchestrator_node.py tests/ros/test_skill_node.py tests/ros/test_skill_node_reload_only.py tests/ros/test_skills_plan_bench.py -q -p no:cacheprovider`: **510 passed in 44.70 s**; `pytest tests/ros/test_gui_juggle_defaults.py -q`: **6 passed**.
- Full gate (P2, `./run_tests.sh --full`, run 2026-10-17 after the audit fixes): **6716 passed, 9 skipped in 307.14 s** (parallel phase) + **7 passed in 27.43 s** (serial tail), exit 0.
- Build (`colcon build --packages-select jugglebot_interfaces jugglebot`, run 2026-10-17, launch down): 2 packages finished in 1 min 55 s, exit 0. FULL launch restart required before the next sitting (Juggle.action changed). Open GUI tabs need a hard refresh after the restart (a stale tab sending `,reload` is refused as an unknown token).

## Outcome

P1 and P2 code landed; P3 (two-feed chain) remains.
