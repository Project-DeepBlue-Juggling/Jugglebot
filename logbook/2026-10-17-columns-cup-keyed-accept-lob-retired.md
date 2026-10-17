---
title: "Columns P1: accept rule keyed on (pattern, cup evidence), human-lob columns feed retired"
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

## Outcome

P1 code landed; P2 (interface removal) and P3 (two-feed chain) remain.
