---
title: "Drops to date: a live counter in the records ledger and atop the GUI Records panel (FEAT-47)"
type: feature
date: 2026-11-10
status: complete
phase: "gui improvements"
subsystem: records
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/gui/js/records-model.js
  - ros_ws/gui/js/records-panel.js
  - ros_ws/gui/index.html
  - ros_ws/gui/css/records-panel.css
  - tests/motion/test_skills_report.py
  - tests/motion/test_records.py
  - tests/ros/test_gui_records_panel.py
  - tests/ros/test_skill_node.py
  - tests/ros/js/records_panel_harness.js
  - logbook/INDEX.md
related_items: [FEAT-47]
---

# Drops to date: live counter

## What changed

FEAT-47 asks for a count of every drop to date atop "Records" in the GUI. This builds the live count only.

- **Definition as built** (owner's answers, TKT-55): a drop is a ball thrown to be caught by Jugglebot that was not caught. `report.drop_count(reports, feed_missed=0)` counts each `ThrowReport` that is not the columns Stop and not an `unverified` final Catch (BUG-67) and whose `caught` is not True, so a miss and a ball still in the air when the Attempt ended (a Fault, a refused Segment, a Stop) both count.
- **Ledger:** `docs/records.md` gets a "Drops to date" section from a machine marker `<!-- drops: live=<n> since=<date> history=<m|none> -->`. The total is history plus live; `history` is the slot for the later backfill (none reads as 0, shown "history not yet counted"). A file without the section parses as "not counted yet", so today's v2 file is unchanged until the first live write adds it. `record_attempt` now also writes when the Attempt carries drops, and once to add the section; a malformed file is still never overwritten. Feeds (`reload`, `magazine`) book their drops without a record row.
- **Node:** `_juggle_execute` passes `rp.drop_count(self._goal_reports)`; one INFO line `records: +N drops (total T)` when drops were added. The Juggle Result, feedback and end line are untouched.
- **GUI:** "Drops to date: T · since <date>" atop the Records panel, with "(history not yet counted)" and "not counted yet" cases.

## Ball Butler Feed misses: not counted

A Feed that misses the Cup ends the Attempt `MISSED_CATCH` with no `ThrowReport` (only a seated Feed gets a `FeedCatchReport`), and the executor keeps no count of which missed CATCH was the Feed's (`_PendingCatch.is_feed` is internal). It is not cleanly countable from what the node has at Attempt end, so only Throws are counted and `feed_missed` stays 0 at the call site. Counting them needs a small executor change that exposes missed Feed Catches; left for a follow-up rather than guessed.

## Not done

The history backfill (a day Session). The live count starts at the first Attempt after a `colcon build` and relaunch; `docs/records.md` was not edited by hand.

## Verification

Run 2026-11-10: `tools/night_shift_nonet.sh pytest -q tests/motion/test_skills_report.py tests/motion/test_records.py tests/ros/test_gui_records_api.py tests/ros/test_gui_records_panel.py tests/ros/test_gui_records_hover_place.py tests/ros/test_gui_record_panel.py tests/ros/test_skill_node.py` — **535 passed in 136.56 s**.
