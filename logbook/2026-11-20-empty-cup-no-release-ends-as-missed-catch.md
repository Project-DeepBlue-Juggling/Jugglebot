---
title: "An empty-Cup Throw with no release evidence ends as the missed Catch it follows, not ABORTED_NO_RELEASE (BUG-33)"
type: bugfix
date: 2026-11-20
status: complete
phase: "BUG-33 end-code relabel"
subsystem: skills
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_executor.py
  - logbook/INDEX.md
---

# Empty-Cup Throw ends as the missed Catch (BUG-33)

**What.** Five Sittings (last 2026-11-20) ended Attempts `ABORTED_NO_RELEASE`
("throw N/M never left the hand") when the Hand stroke was normal and the Cup
read EMPTY throughout: the Ball had been lost on the flight before, and one
such end (2026-11-20 launch 2 Attempt 3) had no MISSED-BALL line at all.
Owner decision 2026-11-20: relabel. `_advance_release_evidence` now asks
`_empty_cup_before_release` first: the Catch verdict's own rule (LIVE window,
`MISSED_CATCH_MIN_VALID_FRAC` / `MISSED_CATCH_MAX_GAP_S`, zero raw-SEATED
samples) over `[previous release + MISSED_CATCH_OPEN_AFTER_RELEASE_S,
t_release + RELEASE_SEAT_EPS_S]` of the same `seat_window`. Empty and live:
`MISSED_CATCH`, message "the Cup was empty at throw N/M; the CATCH of Ball X
(skill k) produced no ball" plus the existing `MISSED-BALL` line (thrown by,
scheduled release, last fit); with no uncaught flight on record, "the Cup was
empty at throw N/M; no lost flight was recorded". Anything else (a SEATED
sample, blind/thin/gappy window, no `seat_window`, no previous release) stays
`ABORTED_NO_RELEASE` exactly as before. No new thresholds. Contract first:
INVARIANTS.md § 8 `ABORTED_NO_RELEASE` row.

**No motion change (evidence).** The relabel is inside the existing
`if not self.attempt_ended` branch that already set `attempt_ended` and dropped
the row; it sets only `end_code`, `end_message`, `end_kind` and appends lines.
It does NOT set `_fault_settle_until_s` (that would delay `done` and arm the
fault relabel), increments no counter (`missed_catch_drops`,
`missed_feed_catches`, `missed_human_catches`), and the D3 survivor branch above
it is untouched. Readers of the code (grep `ABORTED_NO_RELEASE|MISSED_CATCH`):
`skill_node._on_tick` copies `end_code` to `_goal_end_code` and calls
`_maybe_hold_pending_event` (forces a hold only for `LANE_REFUSED`, so neither
code does); `report.end_line` formats it (the only reader of `end_kind`);
`records.category` maps both to the DROP category; `human_catch` policy
`CONTINUE_CODES` lists `MISSED_CATCH` but a `human_catch` schedule carries no
Throw, so no empty-Cup Throw end can arise there. Non-test readers of `ABORTED_NO_RELEASE` by `grep -rn` over ros_ws, controller, sim,
tools: 20 lines (executor 14 incl. comments, skill_node 2 comments, records 2,
sim/skills_gate 4 comments/forbidden-code tuples); none branches on the pair
differently. Learner
Memory: the dropped row was never written either way; the lost inbound flight
keeps its own `OUTCOME` row/ThrowReport as before.
`test_the_empty_cup_relabel_changes_no_motion_or_bookkeeping` pins identical
installs, dispatched set, counters, settle and experiences against the old label.

**Verification.**
- 2026-11-20, `pytest tests/motion/test_skills_executor.py tests/motion/test_records.py tests/motion/test_skills_report.py -q`: 1201 passed in 364.89 s (the same files plus `tests/motion/test_skills_executor_beats.py`, `test_skills_executor_human_catch.py`, `tests/ros/test_possession_replay.py`, `test_ball_possession.py`, `test_skill_node.py`, `tests/sim/test_skills_gate.py`, `test_logbook_front_matter.py`, `test_logbook_index.py`, `test_docs_links.py`, `test_no_legacy_item_ids.py`).
- 2026-11-21, `flock /tmp/jugglebot-tests.lock ./run_tests.sh` on the merge with main: parallel 10667 passed, 61 skipped, 1 xfailed in 802.62 s (rc=0); serial 7 passed in 29.01 s (rc=0). The wider scoped set named in the line above was not re-run separately; the gate covers it.
