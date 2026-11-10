---
title: "Drops to date counts Throws lost to an empty Cup (FEAT-47 slice 3)"
type: feature
date: 2026-11-11
status: complete
phase: night-shift
subsystem: skill-stack/gui
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_report.py
  - tests/motion/test_records.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_magazine.py
  - logbook/INDEX.md
tags: [night-shift, FEAT-47]
related_items: [FEAT-47]
---

# Drops to date counts Throws lost to an empty Cup

## What changed

The third slice of FEAT-47 (slice 2: `2026-11-11-drops-count-feed-misses`). When a Catch ends MISSED_CATCH because the Cup read EMPTY, `_advance_catch_evidence` drops every pending outcome row of that ball before `_finalise_outcome`, so the lost Throw never gets a ThrowReport and slice 1/2 never counted it (about 216 of 633 drops in six weeks of launch logs). The Cup-EMPTY end on a non-Feed Catch, the survivor ending, a ball in the air at a refused Segment/Stop/cancel whose Catch was already dispatched, and both balls lost all pass through that one drop.

- **Executor:** `SkillExecutor.missed_catch_drops`, a recorded fact only (no motion, timing, row handling or end-line text changed). Where the rows are dropped it counts each one with `t_release_s < pend.t_land_scheduled_s and not shadow_landing`: the inbound flight only.
- **Report/node:** `rp.drop_count(..., missed_catches=)` adds it (negatives ignored); `_goal_missed_catches` is summed at each executor retire, reset at accept, and passed from `_juggle_execute`. `magazine` still books zero. One INFO line names the goal's lost Throws.

## The double-count trap, and the rule

A "+1 per non-Feed miss" would double count an outcome row that finalised as a caught-False ThrowReport before the miss rule fired, and would count the carried Throw's row from an empty Cup, which is not a ball. The rule avoids both by construction: a finalised row is no longer pending, so it is never in the dropped set; the carried row releases at or after the landing, so the `t_release_s < t_land` test skips it. The Feed Catch has no inbound row and stays counted by `missed_feed_catches` alone.

## Not a drop

A Throw that never released (ABORTED_NO_RELEASE, DROPPED_SURVIVOR_STOPPED) is not a drop: it is not shown to have been thrown. Nothing changed there.

## Open follow-up

The end line's N/M tally and the Juggle result (`rp.summarise`) still omit these lost Throws; out of FEAT-47's scope (the ledger's drops are the only consumer changed).

## Verification

Run 2026-11-11 (scratch worktree of main + the Feed-miss slice): `./run_tests.sh -q` (the default gate): **PASS, exit 0, parallel 603 s + serial 32 s = 635 s**. Before the change the seven new executor tests failed (`missed_catch_drops` absent); after it `pytest tests/motion/test_skills_executor.py -q` was 309 passed. New tests: executor (non-Feed miss counts 1 with no report; carried row not counted; Feed Catch counts 0 here; finalised row not double counted with the miss rule firing; survivor ending with the survivor caught True and with it also lost = 2 via `drop_count`; attempt-already-ended), `test_drop_count_adds_lost_throws_and_ignores_a_negative`, node (sum across chained executors, reset at accept, real ledger), `test_a_magazine_end_books_no_drop` now with lost Throws set, and the source pin in `test_records.py`.
