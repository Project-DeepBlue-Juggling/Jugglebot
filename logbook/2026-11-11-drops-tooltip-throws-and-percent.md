---
title: "Drops to date: the hover text also shows the Throws total and the percentage caught"
type: feature
date: 2026-11-11
status: complete
phase: night-shift
subsystem: skill-stack/gui
files_changed:
  - ros_ws/src/jugglebot/jugglebot/records.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/gui/js/records-model.js
  - tools/set_drops_history.py
  - docs/records.md
  - tests/motion/test_records.py
  - tests/motion/test_skills_report.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_magazine.py
  - tests/ros/test_gui_records_panel.py
  - tests/ros/test_gui_records_api.py
  - tests/ros/js/records_panel_harness.js
  - logbook/INDEX.md
tags: [FEAT-47]
---

# Drops to date: Throws total and percentage caught in the hover text

## What changed

The owner asked (2026-11-11) for the hover text on "Drops to date: N" to also give the total
number of Throws and the share caught versus dropped, e.g. "7,304 Throws and Feeds to catch ·
88.8 % caught, 11.2 % dropped · 0 live since 2026-11-11, 819 from history (2026-07-23 to
2026-11-10)". The visible line is unchanged.

The ledger now stores CAUGHT next to drops, so the total is `caught + drops` by construction and
the denominator cannot disagree with the drops figure. The marker gains optional `caught=<n>`
(live) and `history_caught=<m>`; a marker without them parses and re-renders byte-identically.
`to_json` sends `caught` and `throws`, both `null` unless every part that has drops also has its
caught figure (never a percentage over a denominator that misses a part). The history figure is
entered with `tools/set_drops_history.py --history-caught N` (refuses a negative).

## The shared-base rule

`report.caught_count` sits next to `report.drop_count` and both docstrings say they share one
base: a ball thrown to be caught by Jugglebot is counted by exactly one of them. Caught = the
ThrowReports with `caught` True that are neither the columns Stop nor an unverified final Catch,
plus the Ball Butler Feeds that seated (`feed_reports`, counted in `_drain_reports` as
`_goal_feeds_caught`). A caught Feed must count because a missed Feed counts as a drop.
No double count: `SkillExecutor._report_feed` appends a `FeedCatchReport` only; a Feed has no
`ThrowReport` of its own, and the ball Jugglebot later throws is its own Throw. `magazine` books
0 caught and 0 drops.

An Attempt with catches and no drops now writes the ledger too (live caught rises), so the file is
rewritten (atomic, a few KB) once per Attempt that caught anything, not only on drops or records.
The never-raises guard and the log lines in `_offer_record` are unchanged.

## What the denominator counts

The history denominator (7,304 = 6,485 caught + 819 drops, recomputed from
`logbook/artifacts/2026-11-11-drops-history.csv`, columns `throws_to_catch`, `caught`, `drops`)
counts Feeds as well as Throws, hence "Throws and Feeds to catch". Live caught starts at 0 on the
first Attempt after the history figure and is never started on a ledger that already holds live
drops without a caught figure (it would miss them; the percentage then stays hidden).

## To see it

`records.py` is loaded in memory by both the GUI server (`jugglebot-gui.service`) and the launch
(`skill_node`). Restart both (`sudo systemctl restart jugglebot-gui`, relaunch) or the panel and
the live writes stay on the old code.

## Verification

- 2026-11-11, `pytest tests/motion/test_records.py tests/motion/test_skills_report.py tests/ros/test_gui_records_panel.py tests/ros/test_gui_records_api.py tests/ros/test_skill_node_magazine.py -q`: 153 passed (the new tests failed first, before `records.py` and `report.py` changed); `pytest tests/ros/test_skill_node.py -q -k "caught or feed_catch_counter or drops or record"`: 16 passed.
- 2026-11-11, `./run_tests.sh -q`: RESULT PASS (parallel 607 s rc=0, serial 31 s rc=0); re-run after the last prose edit, see the commit message.
- 2026-11-12, `pytest tests/sim/test_glossary_avoid_words.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py -q`: 20 passed.
- The same commit fixes the avoid-word "reload" (plain prose) in `2026-11-11-drops-line-total-only.md`.
- Not checked in a browser by this Session.
