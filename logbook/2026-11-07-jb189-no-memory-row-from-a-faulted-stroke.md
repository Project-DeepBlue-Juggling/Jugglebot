---
title: "BUG-76: a Throw released inside a fault's settle writes no Memory row"
type: bugfix
date: 2026-11-07
status: complete
phase: "Night Shift 2026-11-07 BUG-76"
related_entries:
  - 2026-11-06-jb175-fault-preempts-catch-verdict.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - tests/motion/test_skills_executor.py
  - logbook/2026-11-07-jb189-no-memory-row-from-a-faulted-stroke.md
  - logbook/INDEX.md
---

## What and why

Sitting 2026-11-06 goal 10: the Throw the lead clamp cut short (Apex 0.318 m against 1.00 m commanded, released about 80 ms before the Hand drive faulted) became a Memory row with `caught=True`. The plant model's inlier rule rejects it, so the fit was unaffected, but the row sat on disk for any reader that trusts `caught`.

Change (bookkeeping only; no hold, REST, decay, dispatch, install, end code or `Observations` field is touched): the executor latches `_fault_seen_s`, the first instant it saw `machine_fault` or `guard_latched` (`_note_fault`, called from the live branch of `tick` and from the two later reads that already exist, `_advance_fault_settle` and `_advance_closing`; one extra read per tick only while an OUTCOME row is pending, no fault has been seen, and neither of those two will read). `_finalise_outcome` then writes no row (`on_experience` not called, `report.row=False` with the reason, the OUTCOME line says why) when `t_release + MISSED_CATCH_FAULT_SETTLE_S >= _fault_seen_s`. The fault is seen up to the settle (0.4 s, the measured status lag) after it happens, so a Throw released inside that span may have been cut short by its cause. A release more than the settle before the first sighting left a sound stroke and keeps its row, as BUG-68 keeps OUTCOME rows. `caught` on the report is unchanged, and `report.py` tallies `caught`, not `row`. `observations=None` (sim gate, old callers) changes nothing. A dropped row is the safe direction for the Learner.

Lead-clamp episodes: `grep -n -i "lead.\?clamp"` over `skill_node.py` and `executor.py` finds nothing, so the host has no tick-rate signal for one, and none was added. A clamp-only episode with no fault stays covered by the inlier rule; the goal-10 Throw is covered by its fault 80 ms later. `INVARIANTS.md` states no Memory no-row rule (no `entry_hop`, `shadow_landing` or offset row), so no row was added there.

## Verification

- 2026-11-07, `pytest -q tests/motion/test_skills_executor.py -k "settle_after_release or more_than_the_settle or no_fault_row"`: 4 passed.
- 2026-11-07, same command with the `_finalise_outcome` condition temporarily disabled (`False and ...`): the two no-row tests failed; with it widened to any fault (settle bound removed): the keep-row test failed. Restored afterwards.
- 2026-11-07, `pytest -q tests/motion/test_skills_executor.py tests/ros/test_skill_node.py tests/motion/test_memory.py`: 716 passed in 135.33 s.
- 2026-11-07, `pytest -q tests/motion/test_skills_executor.py` (adds `test_a_fault_first_seen_after_a_fault_free_end_writes_no_memory_row`, which covers the standalone read in `tick`: Attempt ends on a mode change, `machine_fault` rises at release + 0.1 s): 288 passed in 9.52 s.
