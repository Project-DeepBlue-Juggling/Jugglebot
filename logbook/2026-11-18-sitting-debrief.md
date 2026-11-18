---
title: "Robot Sitting 2026-11-18 debrief: 12 firmware lane stops with the Hand following (early emit, host GC, slow bridge loop), cpu_partition makes no measurable difference, a stuck-Attempt hang fixed, BUG-63 closed, BUG-98 and BUG-94 landed, BUG-92 held on frozen paths"
type: investigation
date: 2026-11-18
status: complete
phase: "Sitting debrief"
related_plan: ""
related_issues: []
files_changed:
  - logbook/2026-11-18-sitting-debrief.md
  - logbook/INDEX.md
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_orchestrator_node.py
  - tests/sim/no_legacy_item_ids.tsv
  - docs/records.md
  - temp/learn/jugglebot/memory_v2.csv
  - map/sheets/2026-11-18-robot.results.yaml
  - docs/project-map.md
  - docs/runbook.md
subsystem: [can-bridge, skill-stack, hand, tracking, tooling]
tags: [sitting, hardware, diagnostics, debrief]
---

# Robot Sitting 2026-11-18 debrief

Sheet `2026-11-18-robot`, 51 steps, five launches (launch 3 was relaunched after a hang). Launches 1, 2 and 4 flew `main`; launches 3 and 3-1 flew the candidate `night-shift/sitting-2026-11-18` (BUG-90, BUG-98, BUG-94, BUG-92, IDEA-22). Times are local (UTC+11). The verdicts were derived from the logs and bags and confirmed once by the owner at 13:05, with two flips.

## Summary

| Item | Step | Derived | Outcome |
|---|---|---|---|
| BUG-63 | 13 | pass | complete. Ball 2's first Throw landed x +22, -7, +23, +21 mm on a fresh plant (limit 60) |
| BUG-89 | 22 | fail (clause 2) | open. Counter works; 4 of 7 stamp-gap STOPs lack the dropped-a-knot verdict |
| IDEA-11 | 23 | data | open. 45 deg 20/20 twice, scatter inside the control's bars; 90 deg not on the step |
| BUG-92 | 30 | pass | held: frozen config paths, `tools/sitting_merge.sh` refuses (code 4) |
| BUG-98 | 32 | pass | complete, landed as a5037ab3 through tools/sitting_merge.sh |
| BUG-94 | 34 | pass | complete, landed as 8236822f through tools/sitting_merge.sh |
| BUG-90 | 35 | fail (owner; derived inconclusive) | held. Stop-and-go reduced, not gone |
| IDEA-22 | 39 | data | held. Tilt-back 0.5 not better than 0.0 |
| FEAT-49 | 48 | inconclusive | open. No Hand drive fault occurred while armed |
| BUG-58 | 49 | data (owner; literal pass) | open. The record exists, the starvation still happens |

## Inventory

- Launch 1 (10:38, `cpu_partition:=false`): 8 Columns, 8 oval (six 32-cycle, two 200-cycle), 4 high-low.
- Launch 2 (11:07, `cpu_partition:=true`): 8 Columns, 9 oval, 2 turned Columns at 45 deg.
- Launch 3 (11:31, candidate, `gc_freeze:=true`): human Throws, Ball Butler calibrate, 5 Columns, 6 oval with `carry_zvd`, 6 turned at 45 deg, 2 at 90 deg; hung at 11:55:29.
- Launch 3-1 (11:59, candidate): 11 turned Columns at 90 deg.
- Launch 4 (12:14, `main`): 10 Columns with the FEAT-49 capture armed.
- Ends: 12 LANE_REFUSED, 34 MISSED_CATCH, 1 ABORTED_FAULT (Hand SPINOUT), 3 NO_ADMISSIBLE_COMMAND, 1 ABORTED_NO_RELEASE.
- Trapped-Ball scan on all five bags (`temp/logs/trapped_2026-11-18_*.txt`): no Hand stall at the current clamp.

## Findings

### Firmware lane stops (12 LANE_REFUSED ends)

- The Hand did not fail to follow. In the two guard latches checked against the Hand capture (launch 4 line 1835, launch 1 line 1072) it tracked within 0.15 rev before the scheduler STOP and held within 0.01 rev after it; the 4-5 rev MAX_DEVIATION is the refused plan moving away from the held Hand. Six latches were not checked.
- Split by mechanism: early emit 3 (plus one where it contributed), host GC 3, slow bridge loop 3, plain lateness 2, Hand C1 stop 1.
- The rise is real and it is in Columns: 12 mid-Attempt STOPs in 15.6 min of Columns and turned Columns today against 5 in 24.9 min over 2026-11-02 to 2026-11-13 (3.8 times). The oval's rate held (0.28 then 0.23 per minute). Why Columns stalls the stream more is not explained.
- This week's code adds no latency to the stream path: `trajectory_node` is untouched, the drop counter counts after the forward, the TX sequence lock is held for microseconds, and the executor refactor, BUG-97 and C8 run in `skill_node`, a separate process.
- The dropped knot is not new. The 2026-11-13 launch 1 dropped 44 frames; today 42, 37, 27, 8 and 29 per launch. Only the WARN line is new.
- `cpu_partition` (BUG-89 A/B): 3.21 drops per minute off, 3.34 on (ratio 1.04, 95 % interval about 0.65 to 1.67). No measurable effect; only a halving or doubling would have shown.
- Early emit (BUG-110): after a late tick the emitter's catch-up branch emits the next knot up to 25 ms before its stamp and the bridge's newest-only drain discards the one that was due. Signature: a drop line with a 50 ms stamp step and a negative forwarded lag; 5 of 126 drop lines, all right before an Attempt-ending STOP.
- Host GC (BUG-58): three ends followed a 47-54 ms gen-2 pause in `trajectory_node`. Two were the first 200-cycle oval of their launch, at +72.93 s and +72.89 s. Hypothesis, not tested: the full collection fires on an object-count threshold and both launches flew the same Attempts before it.

### Hand SPINOUT at 10:41:49 (BUG-111)

Hypothesis; the plan was not read from the bag. A setpoint frame arrived 12.6 ms late, the firmware extrapolated the Hand for about 13.6 ms at the peak of a Throw (inside the 14 ms Hand C1 stop grace), the command peaked at 137 rev/s against 122-124 on the 17 earlier Throws, and the Hand hit the top stop at 10.90 rev. Braking then fell to about 500 rev/s^2 at -52 to -58 A, BUG-57's signature.

### The hang at 11:55:29 (BUG-113, fixed)

Two defects in `main`. A fed Attempt was started while the orchestrator was in STANDBY after a guard-latch recovery; the executor ended itself after Ball Butler's announcement had claimed the Feed context (`_reload_ctx`), and the declined-swap branch never released it, so no end line was logged and every new Juggle was refused. Separately, the rejection of a new goal cleared the stored handle of the live goal, so Stop answered "no attempt is running".

### Missed Catches (34 ends)

- NO FIT in the end line is not a Tracker miss: in 24 of 34 ends the carried Throw never flew, and `ball_held` was false before its release in 33 of 33. The loss is the same Ball one flight earlier (BUG-114).
- Of 28 lost flights with a Fit, 10 were within 40 mm of that Ball's earlier landings, 9 at 40-100 mm and 9 at 100 mm or more, 7 of those outward at Ball 2's Site, beyond BUG-78's 10 mm outward Catch authority.
- Collisions: 1 certain (turned 90 deg, 28 mm at 12:03:23.70), 1 possible; none under 70 mm in the launch 1, 2, 3 and 4 bags. BUG-74 is refuted as the main killer for Columns on `main`.
- Columns on `main` did not get worse this week: 9 of 26 Attempts completed against 3 of 7 on 2026-11-13. The candidate's Columns completed 0 of 5 (2 LANE_REFUSED, 3 MISSED_CATCH); too few to blame a branch.

### Other

- `/robot_state`: the operator's 45 Hz was the `ros2 topic hz` terminal; no 5 s bin in any bag is below 70 Hz. The publisher does dip to 80-89 Hz during Attempts on every launch and launch 3 drifted from 98 to about 80 Hz (BUG-112).
- BUG-90 at Dwell 0.35: NO_ADMISSIBLE_COMMAND LIMIT_JERK three times at the first learned Catch (margins -0.15 to -0.20; the nominate probe read +0.129).
- BUG-92: 8 phantom tracks still started outside the box (a floor cell, and near x -900, z 2150-2305).

## Discussion

- Withdrawn: "the Hand cannot follow" (the operator's reading of MAX_DEVIATION at the Hand, and this debrief's opening question). The latch follows the firmware's own stop.
- Withdrawn: "this week's code slowed the stream". No changed code is on the stream path and the drop rate matches 2026-11-13.
- Reframed: the dropped knot (BUG-89) is a marker of lateness, and the cause only in the early-emit cases. The 2026-11-13 debrief kept newest-only forwarding on purpose; that stands, and BUG-110 fixes the emitter side instead.
- Withdrawn: the sheet's claim that FEAT-49's bench precondition was unmet. FEAT-45 rungs B and C passed on 2026-11-13 (the owner's typed result); the sheet's author missed it.
- Accepted tradeoff: BUG-58's literal pass was flipped to data by the owner so the Item stays open while the fault still happens.
- The BUG-110 emitter fix was not made here although its diagnosis is traced: it changes when knots reach the Legs and the Hand, so it goes to the owner with its control-cycle walk (in the Item).

## Fixes made this debrief

- `skill_node`: a declined executor swap after an END releases the claimed Feed context (`_reload_ctx`), so the Attempt ends, logs its end line and frees the goal (BUG-113).
- `orchestrator_node`: a rejected new goal no longer clears the live goal's handle (BUG-113).
- `skill_node`: the MISSED-BALL evidence line is logged at INFO (BUG-114, which stays open for the end line's wording).
- BUG-109: the Night Shift clone held stale remote-tracking refs for branches already deleted on origin; `git fetch --prune` there, five fully merged local branches deleted, and a one-time re-pin (22 to 34), per the owner's TKT-79 answer. `main` was red on this test, so no held branch could land before it.

## Owner decisions open

- `SCHED_GRACE_S` 20 to 30 ms: the plain-lateness ends had a forwarded lag of 21-23 ms; the GC and slow-loop ends overran by 2-14 ms more.
- `HAND_C1_STOP_GRACE_US` 14000: launch 3 line 1339 missed it by 1.3 ms (more), the SPINOUT came during 13.6 ms of extrapolation (less; it was 6000 before 2026-11-05).
- Emit lead 0 to 1 knot (BUG-89's option, unchanged).
- BUG-110: the emitter wait, or the due-aware bridge rule.
- IDEA-22: the default tilt-back (the branch's 3 deg is refused at Apex 0.975).
- BUG-92: hand merge of a frozen-path branch; whether to widen the box to the z 2150-2305 cells.
- BUG-113: should a fed Attempt be refused at accept outside TRAJECTORY?
- BUG-78: the 10 mm outward Catch authority against 7 lost flights 100 mm or more outward.

## Follow-ups filed

BUG-110 (early emit), BUG-111 (Hand SPINOUT on a late setpoint), BUG-112 (`/robot_state` rate), BUG-113 (hang; closed by the fixes), BUG-114 (MISSED_CATCH end line).

## Verification

- Records and Memory commit (2026-11-18, python -m pytest tests/sim/test_runtime_records.py tests/motion/test_learner_replay.py tests/motion/test_memory.py tests/motion/test_records.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_skills_gate.py -q): 234 passed, 1 failed (the BUG-109 test, red on main before this debrief).
- Re-pin (2026-11-18, python -m pytest tests/sim/test_no_legacy_item_ids.py tests/sim/test_map_items.py tests/sim/test_mapkit_docs.py -q): 42 passed.
- BUG-98 on its updated tip a5037ab3 (2026-11-18, tools/sitting_merge.sh prepare, which runs ./run_tests.sh --full): 10027 passed, 63 skipped, 1 xfailed in 702.19 s; serial phase 10 passed. A first prepare on the base before the re-pin was red on the BUG-109 test alone.
- BUG-94 on its updated tip 8236822f (2026-11-18, same command): 10029 passed, 63 skipped, 1 xfailed in 678.84 s; serial phase 10 passed.
- The three fixes with this entry and the Item files (2026-11-18, flock /tmp/jugglebot-tests.lock ./run_tests.sh): 10027 passed, 2 failed, 64 skipped, 1 xfailed in 693.77 s; serial phase 7 passed. The two failures were paperwork in this commit (BUG-113 unrated; an Avoid word in this entry and in BUG-113), fixed without touching code, then (2026-11-18, python -m pytest tests/sim/test_map_items_rated.py tests/sim/test_glossary_avoid_words.py tests/sim/test_map_items.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_sitting_sections.py tests/sim/test_mapkit_sheet.py tests/sim/test_mapkit_docs.py tests/sim/test_sitting_debrief_protocol.py -q): 459 passed, 51 skipped.
- Not run: a build and a launch on the fixed tree (the rebuild is done at the end of the debrief; the fixes are unflown).
