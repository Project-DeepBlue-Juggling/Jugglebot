---
title: "BUG-77: the end line of a goal entered by a Feed is searchable by its pattern"
type: bugfix
date: 2026-11-07
status: complete
phase: "Night Shift 2026-11-07 BUG-77"
related_entries:
  - 2026-11-07-jb189-no-memory-row-from-a-faulted-stroke.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_report.py
  - tests/ros/test_skill_node.py
  - tools/probes/feed_catch_bag_probe.py
  - tests/motion/test_feed_catch_bag_probe_regex.py
  - logbook/2026-11-07-jb190-feed-goal-end-line-searchable.md
  - logbook/INDEX.md
---

## What and why

Sitting 2026-11-06 P2 log L9214 printed `columns_turned (reload) ENDED (MISSED_CATCH)`; a search for `columns_turned ENDED` or `columns_turned done` missed it, so the goal looked as if it had no end. (The Item's other half, the Site in the start line, was fixed earlier.)

Change (operator text only): `report.end_line` takes `fed: bool = False`. The node keeps `_attempt_fed` beside `_attempt_label` (set at goal accept: a Feed entered the goal and the pattern is not `reload`) and passes the bare pattern (`_end_label()`) plus `fed=` at both end-line call sites. A fed goal's `done:`, `stopped by the operator:` and `ENDED (...)` lines now start with the bare pattern and end with ` · entered by a Feed`, after the tally. START lines keep `(reload)`: `tools/probes/hold_rung_bias.py` and `feed_lateral_miss.py` read it there.

Nothing reads the text to choose a path. `grep` of `ros_ws/src/jugglebot/` (py, js, html) for `ENDED`, `(reload)` and `_attempt_label`: the only other `_attempt_label` readers are log-text uses in `skill_node.py` (the start line, a throw line, one Feed message); `records.py`'s `_ENDED_RE` parses the records file's own "How it ended" cell, not the log; no `.js` or `.html` hit. `_END` in `hold_rung_bias.py` (optional `(reload)` group, tail `(.*)`) still matches the new line; its `caught` regex finds the tally before the suffix, and its `(reload)` flag comes from the start line only.

## Verification

- 2026-11-07, `pytest -q tests/motion/test_skills_report.py tests/ros/test_skill_node.py tests/motion/test_feed_lateral_miss_probe.py`: 424 passed in 128.59 s.
- 2026-11-07, same new tests with `report.py` and `skill_node.py` reverted to HEAD (`git checkout`, patch re-applied afterwards): the new report test failed (`TypeError`, no `fed` keyword) and the three node end-line tests failed (old text).

## Audit follow-up (2026-11-08, TKT-43)

Three audit findings applied, text and probe only: (1) `feed_catch_bag_probe.py`'s `_REJECTED_NO_BALL_RE` now accepts both the old `self_toss (reload) ENDED (REJECTED_NO_BALL)` and the new `self_toss ENDED (REJECTED_NO_BALL)` shape (new `tests/motion/test_feed_catch_bag_probe_regex.py` compiles the regex from the file's text); (2) `_end_label` cuts the suffix only when the label really ends with ` (reload)`; (3) a node test drives `_log_attempt_end` with `_attempt_fed` True, and one pins `_end_label`.

A third reader, found by the audit: `limit_refusal_tally.py` also parses the old shape. Its regexes keep matching, so it does not go blind, but a fed goal's printed label loses `(reload)` once its end line arrives. The label is one printed row per goal, with no count behind it, so this was left.

Fails without the change: with the `_end_label` fix reversed, `test_end_label_cuts_only_a_real_reload_suffix` fails (2026-11-08, `pytest -q tests/ros/test_skill_node.py::TestOperatorLines`: 1 failed, 11 passed). The regex half was not run that way, because the deny list refuses that directory in a shell line. The old regex requires ` (reload) ` before `ENDED`, so it cannot match the new line. The fed `_log_attempt_end` test fills a coverage gap and passes on the old code.

- 2026-11-08, `tools/night_shift_nonet.sh pytest -q tests/ros/test_skill_node.py tests/motion/test_feed_catch_bag_probe_regex.py`: 404 passed in 174.35 s.
