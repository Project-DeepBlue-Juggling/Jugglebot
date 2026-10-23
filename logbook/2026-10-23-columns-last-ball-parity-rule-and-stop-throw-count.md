---
title: "The one-ball columns last ball is caught: a parity rule in compile_columns, and the Stop throw is no longer counted"
type: bugfix
date: 2026-10-23
status: in-progress
phase: "two-ball-skill-stack - post-R6"
related_plan: two-ball-oval.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - tests/motion/test_skills_schedule.py
  - tests/motion/test_skills_report.py
  - tests/motion/test_skills_executor.py
subsystem:
  - skills
tags:
  - testing
---

## Summary

Owner, 2026-10-23: "We need to actually catch last ball in columns_1ball_fed (rather than have it slam into the hand at its bottom position)." Decision: "Parity rule + count fix". `compile_columns` now lengthens a one-ball (phantom) run by one throw whenever the Stop throw would belong to the real ball, so the Stop is always the phantom's and the real ball's last throw gets an ordinary transit CATCH. Separately the Stop's report is tagged `stop` and excluded from the tally. NOT FLOWN.

## Problem

`columns_1ball_fed` goals read k/(k+1) by design: goal 4 of the 2026-10-22 sitting ended `1/2 caught` (launch log L1108-1131; goals 5 and 8 the same). The last real ball was released toward a static, level cup at rest with no CATCH planned, landed 87 ms before the closing REST's event with the hand at its rest floor, and was labelled `MISSED (landed on the held ball)`.

## Root Cause

The Stop throw (`is_stop_throw = (i == n-1 and n >= 2)`, `schedule.py`) exists to give a two-ball schedule somewhere to put the ball it cannot catch: D2 (2026-09-30, `2026-09-30-skill-stack-r5-columns-bb-start.md` section 6) aims it cross-site at the site already holding a ball so the platform never moves again. In a one-ball pattern the other ball is a phantom, so the cup at that site is EMPTY, not holding a ball. With an even `num_cycles` the Stop is the real ball's throw (ball `i % 2`), aimed at the phantom's last catch site: a real ball into an empty stationary cup, which no stroke decelerates. The "landed on the held ball" label was false for this pattern. The missing condition: the Stop must never be a real ball's throw when the other ball is a phantom. The sim gate already knew the parity (`sim/skills_gate.py` ~2440: the Stop lands on ball B whenever `n_throws` is even); the robot did not enforce it.

## Discussion

Design memo: the candidates were weighed against what the planner already flies.

**Why the one-ball last ball could never be caught.** D2 first tried a REST that travelled to the last ball's site. It had 0.228 s to cover 100 mm from rest and was refused `LIMIT_VEL 315 > 300`, and it failed the rest-terminal assertion at 0.88 mm/s; carrying over the whole beat needed about 10 deg of banking. D2 then removed the travel by aiming the throw at the already-occupied site. At 250 mm separation (2.5x) a travelling REST is less feasible still.

**Why two-ball columns has no schedule fix.** One-seat cup, and at every instant a ball is airborne, so one ball's final landing is never a single-seat catch. Catching the Stop ball on top of the seated ball: no stable seat (assumption, cup geometry not measured), the sensor already reads SEATED from the held ball so it cannot tell caught from bounced, and it is a CATCH spliced into the last catch's 0.30 s tail, the REST_FRESH_MARGIN LIMIT_JERK seam class. Discarding the held ball first: needs a MAGAZINE segment (planned only from a stopped machine, `MAGAZINE_NOT_AT_REST`) or about 2 s of flight (about 4.9 m apex); infeasible without a new receptacle, an owner and hardware decision. The travelling REST: ruled out above. The two-ball Stop therefore stays a designed non-catch; only its reporting changes.

**Why a rule in `compile_columns` (the schedule), and why n+1 rather than n-1.** The defect is a missing condition, not a missing planner capability, and the rule uses only segment shapes already flown every beat (steady transit catch, mirrored cross-site catch-with-throw, REST holding a ball), so its LIMIT_* exposure is the steady pattern's. `compile_columns` is the one place the Stop is defined; a rule in `skill_node` would be a second definition. n+1 keeps the real throws the operator asked for (n=4 and n=5 both give real throws i=1 and i=3); n-1 would silently drop one. `columns_1ball_fed` (phantom {0}): 4 -> 5, 5 -> 5, 6 -> 7; `columns_1ball` (phantom {1}): 5 -> 6, 4 -> 4. Two-ball columns, n=1 and other patterns are unchanged.

**Accepted side effects.**
- The cup now ends SEATED with the real ball, so the next fed goal is `REJECTED_BALL_IN_HAND` until the ball is removed (correct, operator-visible); `columns` accepts it.
- Two-ball `throws` in the result reads n-1 (`summarise` drops the Stop); any consumer expecting `throws == n_throws` sees one fewer.
- The GUI time estimate `(n-1)*beta + t_f` is one beat (0.56 s) short on lengthened one-ball runs (the JS does not know phantom patterns).
- The end tally's apex range and worst-landing still include the Stop report's own landing.
- The executor's "throw k/N" N counts compiled releases (n+1 when lengthened).

**Residue.** The terminal MISSED_CATCH's own throw is not in the count: the 2026-10-22 48-run read 47/48 with the real miss dropped (its rows were dropped as "attempt already ended"; the only MISSED line in the tally was the shadow throw, and the goal ended `MISSED_CATCH` on `the CATCH of Ball 1 (skill 48) produced no ball`). Fixing the shadow label alone would read 47/47 on a goal that ended on a real miss, so the end code stays beside the count. The mirrored P2 -> P1 phantom Stop at 250 mm is unconfirmed offline (the sim refuses 0.9/250 on both parities before the closing); the first odd-n validate-only on the loaded Jetson answers it. The obsolete `max(3, remaining + 1)` comment in `sim/skills_gate.py` was not edited.

## Fix

- `schedule.py`: in `compile_columns`, after the `n < 1` check, `if n >= 2 and pattern.phantom_balls and (n-1) % 2 not in phantom_balls: n += 1`; docstring gains the parity rule and closing walk-through. `schedule_has_shadow_landing` ignores a phantom's Stop (no "last ball rests on the platform / not resumable" end line for one-ball).
- `report.py`: `ThrowReport.stop`; throw line `STOP (landed on the held ball, by design; not counted)` at INFO not WARN; `summarise` excludes stop reports; `end_line` uses `summarise`.
- `executor.py`: `_report` sets `stop=pend.shadow_landing`; OUTCOME text reworded. `caught` for the Stop is still False; no learner row is written for it (unchanged guards), and the real last ball now writes one more row.

Closing walk-through (fed, n=4 -> 5, apex 0.9, dwell 0.27, from THROW 0): real throw i=3 released +1.690 into CATCH +1.420; phantom Stop i=4 released +2.254 P2 -> P1 (shadow, empty stroke); real ball lands P1 +2.547 as a standalone transit CATCH; REST at +3.197 at P1 holding it.

## Verification

- 2026-10-23, `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MUJOCO_GL=egl APEX=0.95 python /home/jetson/.claude/jobs/db285742/tmp/probe_closing.py 125 4 5 6 7` (production planner through `SkillsGate.run_columns_1ball_fed_attempt`, apex 0.95, separation 125 mm, dwell 0.27, cold memory, seeds 0-2): n=4 2/2 (was 1/2), n=5 2/2, n=6 3/3 (was 2/3), n=7 3/3; end code '' on all 12 runs, no refusals. The sim does not model the slam, so this shows the bookkeeping and planner verdict, not the physical difference.
- 2026-10-23, `./run_tests.sh -q` (implementer, mid-session): parallel 321 s rc=1, serial 20 s rc=0; 7 failures, all `tests/ros` (`test_skill_node_robot_frame` x2, `test_skill_node_resend_param` x5), attributed to the concurrently edited gated file / stale box gate hash in the dirty tree, not this change: `resend_param` still failed with `schedule.py` and `report.py` reverted to HEAD, and a detached HEAD worktree passed them. `robot_frame` was not separately bisected. Re-run after the sweep below. Scoped `tests/motion` and `tests/sim/test_skills_gate.py` passed (schedule 173).
- Full gate: 2026-10-23, `./run_tests.sh --full` (BLAS single-thread, after the box re-sweep) -> 7054 passed, 9 skipped in 312.33 s + serial tail 7 passed, exit 0 (`temp/logs/gate_parity_restslew_20261023.log`).

## Outcome

Code landed, NOT FLOWN. Check at the next sitting: an odd `num_cycles` `columns_1ball_fed` validate-only on the loaded Jetson (answers the mirrored Stop at 250 mm), then a flown goal reading N/N with the cup SEATED at the end. Open: the terminal MISSED_CATCH throw missing from the count.
