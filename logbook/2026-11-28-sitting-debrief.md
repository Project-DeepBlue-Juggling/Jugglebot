---
title: "Robot Sittings 2026-11-27 evening and 2026-11-28 debrief: the Learner merge passed its first flight; the 15 mm Catch authority leans positive; Ball 2's Feed lands 25 mm short; the Hand guard ends are the bridge-stall race"
type: investigation
date: 2026-11-28
status: complete
phase: "Sitting debrief"
related_plan: learner-efficacy.md
related_issues: []
files_changed:
  - logbook/2026-11-28-sitting-debrief.md
  - logbook/INDEX.md
  - docs/records.md
  - docs/project-map.md
subsystem: [skills, learner, ball-butler, can-bridge]
tags: [sitting, debrief, columns]
---

## Summary

Four launches since the last debrief, all flown without a sheet from the Items' own `## Sitting` sections: 2026-11-27 18:55 (FEAT-68, single-ball launch baseline), 2026-11-27 19:13 (the owner's Columns run, no Item), 2026-11-28 12:21 (BUG-158) and 2026-11-28 12:31 (BUG-78's 15 mm trial). The analysis is in two entries and is not repeated here: `2026-11-27-learner-efficacy-e1-launch-baseline` for FEAT-68 and `2026-11-28-first-flight-of-the-learner-merge-and-the-15-mm-catch-authority-trial` for the two Sittings of 2026-11-28 (with the 19:13 run as the comparison).

| Item | Sitting | Verdict | What happened |
|---|---|---|---|
| FEAT-68 | 2026-11-27 18:55 | pass (owner), closed 2026-11-27 | Launch scatter about 15 mm/s per axis whatever the settling or Dwell |
| BUG-158 | 2026-11-28 12:21 | pass (owner confirmed 2026-11-28), closed | 0 unusable Memory rows in 10 Attempts, no Learner-attributable refusal, offsets inside the caps |
| BUG-78 | 2026-11-28 12:31 | pending (owner: try a larger value next) | 0 of 50 Ball 2 Landing Catches missed at 15 mm; 8 Attempts, confounded with the Learner merge; next trial 25 mm |

No Item had a held branch, so nothing went through `tools/sitting_merge.sh`.

## Big picture

What went well: the merged Learner (lateral law per release kind, process noise) flew ten Attempts with nothing to answer for, and plan step E2 is complete. The Catch at 15 mm outward authority missed no Ball 2 Landing in 50, where the previous evening's Columns run at 10 mm lost about half its Attempts that way.

What did not: only 5 of 18 Attempts on 2026-11-28 ran their full 21 Throws. Most of the rest were lost to Ball 2's Feed, three to a Hand MAX_DEVIATION, one to a mid-air collision.

What it means. The Feed losses are not a collision or geometry problem: Ball Butler lands Ball 2 25 to 29 mm short of its own announced landing on every Feed, with only 5 to 7 mm of scatter, and the Cup's 10 mm outward reach cannot cover it. Nothing on BB has changed in weeks (owner), and the bias read +4 mm on 2026-10-11, so the cause is unknown; the trim treats the symptom. A second defect made it worse: a re-aim clamped to exactly 10 mm was then discarded as within tolerance, so the Cup did not move at all (BUG-161, fixed and merged this day). The Hand guard ends are not the overheated Hand motor: in all three the bridge node stalled, a knot arrived after the firmware's 70 ms cover, and the host plan walked away from the held Hand before its hold landed. That race now costs about one Attempt in ten and is the largest remaining cause of lost full-length Attempts.

Next steps. BUG-160: fly the Feed bias trim `[-25.0, 5.0]` at the default authority (also the first flight of the BUG-161 fix). BUG-78: then the 25 mm authority trial. FEAT-40, BUG-89, FEAT-31, BUG-58: a dedicated investigation of the bridge stall is prepared for a fresh Session. Pathway PWAY-2: the plan stays active for step E5 only.

## Trapped-Ball scan

`tools/trapped_ball_finder.py` on all four bags (reports in `temp/logs/trapped_<bag>.txt`). One Hand stall with a trapped marker: bag `2026-11-28_12-31-28`, 12:35:40.683 to .741, Hand at 1.66 rev against a command of 0.42, peak 67.2 A, TRAPPED MARKER COINCIDES at (21, 3, -41) mm, Ball 58; the same Ball dwelt under the Hand at (55, 64, 25) mm for 0.42 s from 12:35:40.094. This is the Hand obstruction of that Sitting's Attempt 4 (the lost Ball 2 of that Attempt's Feed), and it is BUG-84 confirmed for this event. The two long "stalls" the finder lists (12:26:36 in the 12:21 bag, 12:35:22 in the 12:31 bag, 3 A, cannot see) are the latched holds after the MAX_DEVIATION ends, not stalls. Bag `2026-11-27_19-13-16` shows three under-Hand dwells and no stall; bag `2026-11-27_18-55-16` shows neither.

## Paperwork

- Records and Memory: `python tools/runtime_records.py merge --resolve-records keep-runtime`. The tracked ledger had changed since the seed only by two pattern sections the runtime copy already held (checked by diff: the runtime ledger was a superset, no tracked line lost). Memory `jugglebot`: 286 rows merged, 5,960 rows.
- Follow-ups filed: BUG-160 (Feed bias), BUG-161 (clamped re-aim, closed the same day); progress notes on FEAT-40, BUG-89 and BUG-84.
- Rebuild: `colcon build --packages-select jugglebot_interfaces jugglebot` in the live checkout after the BUG-161 merge; the installed `executor.py` matches source.
- Bags compressed per the debrief protocol (bags younger than 24 h are skipped until the next debrief).

## Verification

- 2026-11-28, `flock /tmp/jugglebot-tests.lock ./run_tests.sh --full` on the BUG-161 fix rebased onto main (the only code that landed): 11,780 passed, 51 skipped, 3 xfailed; serial 16 passed; RESULT PASS. Everything after it on main is documentation, records and Memory.
- 2026-11-28, `pytest tests/sim/test_runtime_records.py tests/motion/test_learner_replay.py tests/motion/test_memory.py tests/motion/test_plant_model.py tests/sim/test_docs_links.py -q` after the records and Memory merge: 126 passed, 1 xfailed.
