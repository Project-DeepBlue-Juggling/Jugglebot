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
- **Ledger:** `docs/records.md` gets a "Drops to date" section from a machine marker `<!-- drops: live=<n> since=<date> history=<m|none> -->`. The total is history plus live; `history` is the slot for the later backfill (none reads as 0, shown "history not yet counted"). A file without the section parses as "not counted yet", so today's v2 file is unchanged until the first live write adds it. `record_attempt` now also writes when the Attempt carries drops, and once to add the section; a malformed file is still never overwritten. Feeds (`reload`, `magazine`) book their drops without a record row; a Feed pattern (`magazine`) books only Feed misses, which is 0 until they are identified (see the 2026-11-11 resume).
- **Node:** `_juggle_execute` passes `rp.drop_count(self._goal_reports)` (0 for a `magazine` run); one INFO line `records: +N drops (total T)` when drops were added. The Juggle Result, feedback and end line are untouched.
- **GUI:** "Drops to date: T · since <date>" atop the Records panel, with "(history not yet counted)" and "not counted yet" cases.

## Ball Butler Feed misses: not counted

A Feed that misses the Cup ends the Attempt `MISSED_CATCH` with no `ThrowReport` (only a seated Feed gets a `FeedCatchReport`), and the executor keeps no count of which missed CATCH was the Feed's (`_PendingCatch.is_feed` is internal). It is not cleanly countable from what the node has at Attempt end, so only Throws are counted and `feed_missed` stays 0 at the call site. Counting them needs a small executor change that exposes missed Feed Catches; left for a follow-up rather than guessed.

## Not done

The history backfill (a day Session). The live count starts at the first Attempt after a `colcon build` and relaunch; `docs/records.md` was not edited by hand.

## Verification

Run 2026-11-10: `tools/night_shift_nonet.sh pytest -q tests/motion/test_skills_report.py tests/motion/test_records.py tests/ros/test_gui_records_api.py tests/ros/test_gui_records_panel.py tests/ros/test_gui_records_hover_place.py tests/ros/test_gui_record_panel.py tests/ros/test_skill_node.py` — **535 passed in 136.56 s**.

## 2026-11-11 resume: the three audit fixes (TKT-61)

1. **A `magazine` run booked a false drop.** `_magazine_finish` writes a placeholder `ThrowReport` (`caught=False`), so `drop_count` booked one drop per magazine Attempt. `_juggle_execute` now books 0 for `magazine` (a Feed books only Feed misses, not identified yet). Test: `tests/ros/test_skill_node_magazine.py::test_a_magazine_end_books_no_drop` drives the real `_magazine_finish` into the real `_juggle_execute` and ledger with `drop_count` unpatched.
2. **The counter hid with no record rows.** `records-model.js::panelVisible` (used by `records-panel.js`) shows the panel when the ledger has a drops section, rows or not. Test: `tests/ros/test_gui_records_panel.py::test_panel_shows_for_a_drops_section_even_with_no_rows` (harness `panelVisible` cases).
3. **The `total_drops` log line sat outside the guard.** `_offer_record` now runs it inside the try (its own inner guard keeps a booked record's result). Test: `tests/ros/test_skill_node.py::TestJuggleAction::test_a_raising_drops_total_never_escapes_offer_record`.

Each new test was confirmed to fail with its fix reverted.

Verification, run 2026-11-11: `tools/night_shift_nonet.sh pytest -q -p no:cacheprovider tests/ros/test_skill_node.py tests/ros/test_skill_node_magazine.py tests/ros/test_gui_records_panel.py tests/motion/test_records.py tests/motion/test_skills_report.py`: **525 passed in 818.90 s** (slow under a concurrent duplicate run). `tests/motion/test_records.py::test_skill_node_passes_the_drop_count_and_feeds_reach_the_ledger` reads the node source and was updated to the new booking line.

Feed misses are still not counted. The owner's TKT-61 answer requires them before FEAT-47 closes, so the Item stays in progress.
