---
title: "Drops to date counts Ball Butler Feed misses (FEAT-47)"
type: feature
date: 2026-11-11
status: complete
phase: night-shift
subsystem: skill-stack/gui
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_records.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_magazine.py
  - logbook/INDEX.md
tags: [night-shift, FEAT-47]
related_items: [FEAT-47]
---

# Drops to date counts Ball Butler Feed misses

## What changed

The second slice of FEAT-47 (slice 1: `2026-11-10-drops-to-date-live-counter`). A Ball Butler Feed that misses the Cup ends the Attempt MISSED_CATCH with no ThrowReport, so slice 1 could not count it.

- **Executor:** `SkillExecutor.missed_feed_catches` is a counter incremented where the MISSED_CATCH verdict fires on a `_PendingCatch` with `is_feed` (a Catch carrying `landing_prior`). It is a recorded fact only: nothing in the executor reads it, and no decision, timing, plan, verdict or log line changed.
- **Node:** `_on_tick` adds it to `_goal_feed_missed` as each executor retires (reset at accept). `_juggle_execute` passes `feed_missed` to `rp.drop_count` for the fed Patterns, and a `magazine` run books exactly `feed_missed` (0 otherwise). `_offer_record` is unchanged.

## Why

The owner's TKT-55 answer counts a Feed miss as a drop and TKT-61 requires it before FEAT-47 closes. The verdict already knew which Catch was the Feed's; only the node could not see it.

## Verification

Run 2026-11-11, `tools/night_shift_nonet.sh pytest -q -p no:cacheprovider tests/ros/test_skill_node_magazine.py tests/ros/test_skill_node.py tests/motion/test_skills_executor.py tests/motion/test_records.py tests/motion/test_skills_report.py -k "magazine or drop or record or Juggle or feed_miss or missed or Records or report"`. New tests: `test_a_missed_catch_on_a_feed_catch_is_exposed_as_a_missed_feed` and `..._on_a_throws_catch_is_not_...` (executor), `test_a_fed_goal_ending_missed_catch_on_the_feed_books_one_drop` (node, real ledger), `test_a_magazine_feed_miss_books_one_drop`. Each of the Feed-side tests was confirmed to fail with its change reverted. `test_records.py::test_skill_node_passes_the_drop_count_...` (reads the node source) was updated to the new booking line.
