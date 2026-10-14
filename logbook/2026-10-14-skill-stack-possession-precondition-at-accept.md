---
title: Possession precondition at goal accept - the cup must match what the first skill needs, or the goal is refused before any Ball Butler dispatch
type: investigation
date: 2026-10-14
status: resolved
phase: "two-ball-skill-stack — residue (post-R6)"
related_plan: skill-stack-dress-rehearsals-and-residue.md
related:
  - 2026-10-13-skill-stack-r5-sitting-8-gate-met.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_reload_only.py
  - tests/hardware/session_skill_stack_rehearsals.md
subsystem:
  - skill-stack
  - ros
tags:
  - safety
  - accept-path
---

# Possession precondition at goal accept

## Summary
Owner ask (2026-10-14): acknowledge the state of Jugglebot's ball before each attempt or Ball Butler (BB) feed. A RELOAD with a ball already in the hand went ahead; expected: plain RELOADs and JUGGLE attempts with `reload:=true` are cancelled, reason "ball already in hand". Sitting 8's attempt 4 is the other half of the same class: a goal whose opening premise about the cup was false was accepted anyway.

## What happened
- Sitting 8 attempt 3 ended LIMIT_ACC with two balls out of the cup; attempt 4 (`columns`, hop entry) was accepted, BB was asked `in 5.9 s`, and nothing at accept looked at the cup (sitting-8 entry, finding 3b).
- The cup was tested only when skill 0 fired, 5.4 s later: REJECTED_NO_BALL, then the BB cancel went out with the release 0.09 s away and BB answered `CANCEL_TOO_LATE`. The ball flew at an idle platform - the one of 14 BB throws that went to nothing.
- The owner's reload case is the mirror: a seated ball plus a reload that wants an empty cup.

## Discussion
**1. The rule.** The cup's debounced evidence must match what the FIRST skill of the goal needs (`skill_node.accept_cup_requirement(pattern, reload)`):

| pattern | reload | required cup | wrong-state code |
|---|---|---|---|
| reload | any | EMPTY | REJECTED_BALL_IN_HAND |
| columns_1ball_fed | True | EMPTY | REJECTED_BALL_IN_HAND |
| self_toss / hop / columns_1ball | True | EMPTY | REJECTED_BALL_IN_HAND |
| self_toss / hop / columns_1ball | False | SEATED | REJECTED_NO_BALL |
| columns | any | SEATED | REJECTED_NO_BALL |
| magazine | any | SEATED | REJECTED_NO_BALL |
| any | any | UNKNOWN | REJECTED_BALL_UNKNOWN |

`columns` with `reload` is the exception to "reload means EMPTY": the hop entry throws Ball 1 from the cup first and the BB feed lands after, so the first skill needs a seated ball. The code does not key on `columns_entry`: `_run_columns` bridges with `holds_ball=not phantom_a` under both entries, so `transit` is SEATED too (pinned by its own test). Failure mode prevented: a throw-first attempt launched at an empty cup, and a reload catch opened over an occupied cup.

**2. Debounced, not raw; stale refuses.** The check uses `_possession_evidence_stable` (the debounced evidence the executor already trusts), not the raw sensor bit, which flickers while a ball seats or rattles. Telemetry older than `_HAND_STATE_STALE_S` (0.5 s, the same window as `hand_fresh`), never received, or an invalid sensor all mint REJECTED_BALL_UNKNOWN with distinct reasons: a premise nobody can verify is not a premise, and guessing "empty" would reopen exactly the attempt-4 hole.

**3. Why at accept, before any BB dispatch.** BB commits a wind-up whose cancel is late-bounded (`CANCEL_TOO_LATE` inside ~23 ms of release); once dispatched the ball is in flight whatever the check later says. The launch-time ladder (executor `precondition_refusals`) stays unchanged as the second line - it covers a cup that changes between accept and skill 0.

## Fix
`skill_node.py`: `_ACCEPT_EMPTY_ALWAYS`, `_ACCEPT_SEATED_ALWAYS`, `accept_cup_requirement()`, `SkillNode._cup_state()`, `SkillNode._accept_possession_refusal()`, called in `_start_pattern` after `_refuse_if_running` / `_refuse_if_lane_latched`, before the bookkeeping reset and any `_run_*`. `skills/check` gains a `cup: SEATED|EMPTY|UNKNOWN (age N ms)` line (reported, not gated). `executor.py`: `REJECTED_BALL_IN_HAND` beside `REJECTED_NO_BALL`. `INVARIANTS.md` § 8: the NO_BALL row extended with the accept clause, a new BALL_IN_HAND row. Rehearsal runsheet rows 21 (refusals, no motion) and 22 (cup state per row).

## Verification
- 2026-10-14, `pytest -q -p no:cacheprovider tests/ros/test_skill_node.py` -> 294 passed in 22.82 s
- 2026-10-14, `pytest -q -p no:cacheprovider tests/motion/test_skills_executor.py tests/motion/test_skills_report.py` -> 252 passed in 12.09 s
- 2026-10-14, `pytest -q -p no:cacheprovider tests/ros/ -rf` -> 3052 passed, 2 skipped in 445.24 s (a first `-x` run failed 1: `test_skill_node_reload_only.py` seated the cup; fixture fixed, no assertion changed)
- Regression pinned: `test_accept_possession_columns_reload_hop_with_an_empty_cup_is_refused_before_bb` (attempt 4). Existing reload-path fixtures now empty the cup first.

## Open
- Flown (2026-10-14 ~21:59; runsheet row 21 from the GUI; launch.log `2026-10-14-21-56-45`): cup EMPTY `{pattern: reload}` accepted (`Throw at jugglebot`, line 538); cup SEATED `{pattern: reload}` refused `REJECTED_BALL_IN_HAND` (lines 574 and 588), no BB throw, no motion; cup EMPTY `{pattern: columns, reload: true}` refused `REJECTED_NO_BALL` (line 620) before any Throw line. PASSED. See `2026-10-14-skill-stack-dress-rehearsals-flown.md`.
- Rehearsal rows 16/18/19 were re-worded for the check: row 18 now runs WITH the seated ball (the throw lands uncaught on the platform), and rows 19-21 need an empty cup (remove the row-18 ball first).
- Expectation as built for the hop entry's `transit` branch: SEATED, same as `hop` - not separately flown.
