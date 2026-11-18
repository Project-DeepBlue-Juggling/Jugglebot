---
title: "human_catch Pattern built and audited (PWAY-11 P2); the offline gate is not met on the existing bags, which cannot score a one-Ball Pattern"
type: feature
date: 2026-11-18
status: in-progress
phase: "PWAY-11 P2 (offline)"
related_plan: "human-partner-juggling.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/human_catch.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/nominate.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - docs/records.md
  - tools/probes/human_catch_reach.py
  - tools/probes/human_catch_replay.py
  - tools/probes/README.md
  - tests/motion/test_skills_human_catch.py
  - tests/motion/test_skills_human_catch_bounded.py
  - tests/motion/test_skills_human_catch_policy.py
  - tests/motion/test_skills_executor_human_catch.py
  - tests/motion/test_skills_nominate_human_catch.py
  - tests/ros/test_skill_node_human_catch.py
  - tests/ros/test_skill_node_human_catch_audit.py
  - tests/ros/test_skill_node_human_catch_book.py
  - tests/ros/test_skill_node_nominate.py
  - tests/ros/test_skill_node_online.py
  - tests/ros/test_skill_node.py
  - tests/tracking/test_human_catch_replay.py
subsystem: [skill-stack, ros, tools]
tags: [human-throw, performance, testing]
---

# human_catch: the reactive Catch of a human Throw

## Summary

PWAY-11 P2 adds the Pattern `human_catch` (provisional label): Platform Level, Cup waiting at a wait Site; on a human Throw event with enough lead it plans one Catch at the fitted Landing through the command-time admissibility check, and does nothing when no admissible Catch exists. It is built (U4 to U6), audited once, and replayed over the three bench bags. The offline gate (D10) is **not met**: 1/11, 4/21 and 5/46 in-reach Throws are good against the 80 % bar. The reading of this entry is that the bags cannot score the Pattern, not that the Pattern is shown to fail; that is a hypothesis until single Balls lobbed at the Cup are recorded. Nothing has been flown.

## Discussion

Written before the Fix section because hypotheses were withdrawn and tradeoffs accepted. Numbers come from `human-catch-reach.md`, `p2-gate.md`, `p1-gate.md` and the handoffs.

**(a) The design's reach numbers were replaced by the sweep.** The design (`design-human-catch.md`, D3 table) estimated the quintic reach at about 38 mm at 0.50 s of lead, 69 mm at 0.55 s and 104 mm at 0.60 s. `tools/probes/human_catch_reach.py` runs the install's own plan and gate (`executor.check_segment`, Sitting limits 600 mm/s, 8000 mm/s^2, jerk 200000 mm/s^3). It shows a hard first-plan lead floor of about 0.51 s (at 0.50 s even a zero move is refused: the Hand cannot reach the closing speed in a 0.225 s window); 20 to 50 mm at 0.55 s, not 69; 100 mm in all eight directions at 0.575 to 0.60 s; and beyond that the limit is the +-150 mm cup workspace box (`SETTLE_SITE`), not the Legs. The design's 300 mm case is refused `SETTLE_SITE`, not `LIMIT_VEL`. The reach is not monotone in lead (0.525 s reaches 50 to 80 mm, 0.55 s reaches 20 to 50 mm) because the window quantises to knots (10 knots against 11); read the grid points, not a curve. Late re-aims are refused: with 0.35 to 0.45 s of lead left every shift of 10 to 100 mm is refused for jerk. The sweep's first-plan fraction of recorded Throws at lead 0.60 s is 5/5, 13/36 and 21/65 (the 0.55 s figures are 1/5, 2/36 and 1/65). Consequence: the design's "re-aims on revisions correct the first event" does not hold, and the correction has to happen before the plan.

**(b) "Plan on the first event" was withdrawn for "plan once from the latest revision when the lead falls to 0.62 s".** The design planned at revision 0 (rule B). With re-aim effectively dead, committing at the first event commits the worst estimate of the flight (about 70 mm off on a high Throw, entry `2026-11-18-human-throw-event.md`). The policy now holds an early key as PENDING, keeps replacing it with newer revisions, and `poll` resolves it once, from the latest revision, when the lead reaches `plan_lead_s` (PROPOSED 0.62 s, constructor kwarg, `math.inf` restores the original rule). An event already inside [0.55 s, 0.62 s] plans at once. The cost is that the 0.62 s window sits only slightly above the 0.575 to 0.60 s needed for a 100 mm move, so lead lost to message transport or a tick comes straight out of the reachable move.

**(c) The executor gap, and the narrow field.** `compile_human_catch` produces one CATCH and no release. `_register_catch` anchored the MISSED_CATCH window on the CATCH's own release, found none, and armed no verdict: the executor reported done the tick the CATCH dispatched, about 0.5 s before touch-down, no Catch verdict existed and the live re-aims never ran (`handoff-hc-u5a.md`; 12 of 21 node tests failed on it). Three fixes were weighed. Re-anchoring every release-less CATCH on its `landing_prior` would change the Feed CATCH, which also has no release and today is unarmed, so it would alter `missed_feed_catches` for the Ball Butler Feed path. A Cup timer in the node would be a second definition of caught beside the executor's. The chosen fix is the optional `Skill.catch_open_abs_s` (CATCH only, default None, set only by `compile_human_catch` to the plan instant; commit `0ddfaeb7`). Anchoring at the plan instant rather than the Tracker's `flight_start` is safe because the window's open only keeps an earlier Ball's seat out of this Catch's evidence: the Pattern plans only from WAITING with the Cup known EMPTY, so any SEATED sample after the plan instant is this Ball. Regression pins: every other compile carries None; a release-less Feed CATCH still arms nothing; a release-anchored CATCH keeps `[R + 0.10, L + 0.50]`.

**(d) `_reload_ctx` reused with its own kind.** The node reuses `_reload_ctx` with `kind='human_catch'` rather than a second context. The readers table in `handoff-hc-u5a.md` lists every leak guard that keys on it (`_maybe_signal_goal_done`, `_refuse_if_running`, `_attempt_running`, `_log_attempt_end`, `_end_attempt`, `_stop_attempt`, `_check_reload_lane_refused`); a second context would need each duplicated, which is the leak class of commit 467752b8. The Feed-chain hooks (the ones that key on that context) are no-ops for it (`phase='human_catch'`, `announced=True`, infinite deadline). The leak tests Stop the Attempt from waiting, catching, holding and returning and expect exactly one end line, a freed node and a new `reload` goal accepted.

**(e) A human Catch is booked as its own count, not as a Feed.** The human CATCH carries `landing_prior`, so `_PendingCatch.is_feed` was True: a caught one would print a Feed line and a missed one would increment `missed_feed_catches`. A human Throw is not a Feed. `is_feed` is now false for the `human_catch` Schedule; a confirmed seat counts `executor.human_caught`, a `MISSED_CATCH` counts `missed_human_catches`, a blind or gappy Cup window abstains in both. Every other Schedule books exactly as before (pinned). The Juggle result has no natural field, so `result.throws` and `caught` carry the human tally for this Pattern and one line says so. The `records.PATTERNS` row books Throws planned and Catches confirmed and no drops, so Jugglebot's drops-to-date is untouched. Memory is never opened for it (tested over a full Attempt).

**(f) The audit.** One Opus audit pass over the production code found no regression to existing Patterns and nine items that touch only the new Pattern, all fixed with tests in commit `c11137ba`: (1) a race reading the running executor twice; (2) a second hold and end line on a lane refusal during a live Catch in one tick ordering; (3) an unschedulable return REST mistaken for "returned"; (4) an unguarded executor build; (5) the Throw count including refused plans; (6) an unbounded decline list; (7) a wrong Stop message between executors; (8) one Tracker parameter (`stable_ms`) serving two purposes, now split; (9) no visibility while holding (a log line every 5 s, no timeout).

**(g) The P2 gate (2026-11-18, `human_catch_replay.py --sweep`, `p2-gate.md`).** Chain: real Tracker and emitter, real `HumanCatchPolicy` on a 40 Hz tick, `compile_human_catch` plus `check_segment` at the Sitting limits, the executor's re-aim fences re-implemented in the probe, scored against the bench truth crossing. Capture radius 35 mm (`GEOM_HAND_RADIUS_MM`). Frame `hand`: each release hand's direction is translated onto the home Site (-50, 0); D11's question.

| | 11-18 launch 3 | 11-02 passes | 11-02 cascade |
|---|---|---|---|
| plan made and admitted | 5 of 11 | 6 of 44 | 13 of 82 |
| planned Cup vs crossing, xy p50 / p90 (mm) | 28 / 52 | 18 / 33 | 30 / 46 |
| touch-down time error p50 / p90 (ms) | 17 / 22 | 9 / 13 | 10 / 15 |
| within 35 mm and 20 ms | 2/5 | 6/6 | 7/13 |
| D10 good (30 mm, 20 ms) over in-reach Throws | 1/11 | 4/21 | 5/46 |
| re-aims requested / admitted | 4 / 0 | 2 / 0 | 9 / 0 |
| plans on a phantom stream | 0 | 0 | 0 (1 stream, declined `NOT_AT_ROBOT`) |

Verdict against D10: no-go as written (at least 80 % good; observed 9 %, 19 %, 11 %). Zero plans on phantoms passes. Skid was not measured. The decline breakdown over the 78 in-reach Throws of the three bags: planned 22; `BUSY` 21; `LEAD_TOO_SHORT` 17 plus `NO_EVENT` 15 (32 of the 35 non-busy losses); refused 3. Where a plan is made the aim is good (62 % within the capture radius, 15 of 24 admitted plans).

The three bags are cascades and passes with several Balls in the air, so they cannot score a one-Ball Pattern: `BUSY` is the test population, not the Pattern, and the 11-02 bags still land 21 and 25 crossings outside the +-150 mm box after the `hand` translation. In the `robot` frame the Throws land 400 mm or more from the Platform (the owner threw between his hands beside the robot). Re-aims are refused at the proposed plan lead: 15 of 15 at 0.62 s and 0.58 s (`LIMIT_JERK` 11, `REPLAN_WINDOW` 4 at the baseline), 4 of 23 admitted at 0.66 s. So the first aim is final, and it is 28 / 52 mm p50 / p90 from truth on the 2026-11-18 Throws against a 35 mm capture radius (3 of 5 plans outside it by the gate's own wording). No one-at-a-time change (`plan_lead` 0.58 to 0.66 s, first lead 0.52 s, authority 30 to 80 mm, a fixed wait Site) moves the headline by more than 1 Throw per bag. Transport delay is not modelled: the replay uses the event stamp as `now`, while the node uses wall-clock callback time, so any delay is lead lost from a margin that is already the second biggest loss.

## What was built

- `schedule.compile_human_catch`: one level CATCH, rest-terminal at the Landing xy, window = `t_land - t_now - dispatch_lead - LEAD_S`; no `site` argument (the wait Site is the executor's seed REST).
- `motion/skills/human_catch.py`: `HumanCatchPolicy` (OPENING, WAITING, CATCHING, HOLDING, RETURNING), the verified end-code allowlist (a refused CATCH install ends the executor with the bare gate subcode, read from `executor.py`), the wait Site clipped into the workspace box.
- `skill_node.py`: the Attempt lifecycle on `human_throws`, parameters `human_catch_*` read at goal accept, per-Throw executors, operator lines `HUMAN-CATCH-*`.
- `nominate.py`: `human_catch` is nominated at accept as one nominal CATCH at the wait Site for a Ball arriving vertically at 1.9 m absolute (4.58 m/s). The nomination margin is 1.000 by construction (the Legs do not move), so it exercises the Hand's gates only; displaced Balls are the reach table, not an accept rule.
- `records.PATTERNS` row `human_catch`; `tools/probes/human_catch_reach.py` and `human_catch_replay.py`.

## Verification

- (2026-11-18, `python -m pytest tests/motion/test_skills_human_catch_policy.py tests/motion/test_skills_human_catch.py -q`, U5a) 85 passed in 10.57 s.
- (2026-11-18, executor and schedule `-m "not nightly"`, loaded box) 501 passed in 225.71 s; the node file with `test_skill_node.py` and `test_skill_node_polygon.py` 466 passed in 801.05 s; the other nine `test_skill_node_*.py` 137 passed in 103.03 s.
- (2026-11-18, U5b, loaded box, `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`) human_catch, records and nominate files 231 passed in 28.09 s; `tests/motion/test_skills_nominate.py` 47 passed in 441.42 s; `tests/ros/test_skill_node.py` 438 passed in 1263.17 s. Without those BLAS settings `test_skills_nominate.py::test_columns_at_p4_...` fails on a margin tie, unrelated to this work.
- (2026-11-18, audit fixes, box load about 2) the human_catch, tracking and `test_ball_tracker*` files, 289 passed in 19.92 s; `tests/ros/test_skill_node.py` 438 passed in 1021.92 s.
- (2026-11-18, `python tools/probes/human_catch_reach.py --dirs 8 --step 25`) 18 s; `pytest tests/tracking/test_human_catch_replay.py -q` 3 passed.
- `./run_tests.sh` was not run in any of these; the full gate result is in the commit that files this entry.

## Outcome

P2 is built and audited; its offline gate is not met on the existing bags. The Pattern's own evidence (aim within about 30 mm where a plan is made, no phantom plans, no ended Attempts) is promising and a hypothesis only, because the bags do not contain a Ball thrown at the Cup. Next: record single Balls lobbed at the Cup (plan P3, first row).

## Owner decisions open

All values below are PROPOSED, in code as constructor defaults or node parameters, none confirmed (design table D1 to D11):

- D1 phantom start speed 500 mm/s. D2 usable bar: rms 6 mm, agreement 30 mm / 20 ms. D10's P1 first-event criterion (fails on 2026-11-18, 2/11) and its P2 threshold (no-go on the existing bags).
- D3 latest first-plan lead 0.55 s (the sweep suggests 0.60 s). The plan window `plan_lead_s` 0.62 s. D4 association radius 400 mm (the plannable box is 300 x 300 mm). D5 re-aim authority 50 mm (never exercised at the plan window). D6 re-sends per Catch 2. D7 wait Site: mean of 5, clamp 150 mm, deadband 20 mm. D8 clear 0.5 s, return REST 1.0 s. D9 nominal arrival Apex 1.9 m absolute (4.58 m/s). D11 whether the 2026-11-18 bag counts (scored per direction in the `hand` frame).
- A holding timeout (HOLDING waits for the Cup to read EMPTY and has none). Dropping a pending key whose track vanished. Re-keying events across a Tracker restart.
- The wording of three INVARIANTS.md rows (I-ADMIT-2, I-PLAN-8 and a new CATCH-window row), drafted in `handoff-hc-u5b.md`. They are pending the owner's approval; INVARIANTS.md is not edited.
- Whether the Juggle result's human tally and the end line (still 0/0 for this Pattern) are acceptable, and whether a human miss counts as a drop.

## Not flown

No launch, no Platform motion, no live `human_catch` Attempt. Not checked offline: skid against 175 mm/s; the policy under a real `SkillExecutor` (the probe re-implements the re-aim fences); node tick jitter and solve wall under load; the Cup sensor's evidence and HOLDING on hardware; a thrower aiming at the Cup; the Hand's catch runway at 4.6 to 5.2 m/s on hardware (the nomination checks planner gates only); transport delay; the GUI (not wired).
