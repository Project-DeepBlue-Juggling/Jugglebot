---
title: "Columns P1 + P2 + P3: accept rule keyed on (pattern, cup evidence), human-lob columns feed retired"
type: refactor
date: 2026-10-17
status: resolved
phase: "two-ball-skill-stack - post-R6 (columns empty-cup two-feed, P1)"
related_plan: columns-empty-cup-two-feed.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/hardware/session_skill_stack_rehearsals.md
  - tests/ros/test_skill_node.py
  - ros_ws/src/jugglebot_interfaces/action/Juggle.action
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/gui/js/juggle-panel.js
  - ros_ws/gui/css/juggle-panel.css
  - ros_ws/gui/test_juggle_panel.html
  - tests/hardware/skills_plan_bench.py
  - tests/ros/conftest.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_gui_juggle_defaults.py
  - tests/ros/test_skill_node_reload_only.py
  - tests/ros/test_skills_plan_bench.py
  - logbook/2026-10-17-columns-cup-keyed-accept-lob-retired.md
subsystem:
  - skills
  - ros
tags:
  - testing
---

## Summary

Phase 1 of `plans/archived/columns-empty-cup-two-feed.md`. The accept rule is now
a function of (pattern, cup evidence) and the human-lob columns feed (decision
D2) is gone. Columns is Ball-Butler-fed only.

## Changes

- `accept_cup_decision` + `_CUP_RULE` in `skill_node.py` replace `accept_cup_requirement`. EMPTY makes self_toss / hop / columns_1ball `reload`-first; SEATED makes them throw-first; UNKNOWN refuses every pattern (`REJECTED_BALL_UNKNOWN`). `reload` / columns_1ball_fed need EMPTY; magazine needs SEATED. Columns EMPTY is refused `REJECTED_NO_BALL` until P3 (the two-feed chain).
- The node ignores `goal.reload` (field removed at P2). The derived `reload` still goes into the attempt label and the start event (`reload='1'/'0'`).
- Deleted: lob branch of `_run_columns`, `_maybe_resolve_columns_feed`, `await_feed`, `COLUMNS_FEED_BOUND_*`, `COLUMNS_TRACKER_FEED_DEADLINE_S`, `ABORTED_NO_COLUMNS_FEED`.
- Caveat for bag readers: bags before this entry carry columns `reload='1'` meaning "ball B from BB", not "ball A reloaded".
- `INVARIANTS.md` rows 58 / 119 / 120 and rehearsal rows 21-22 amended.
- Tests: 12 deleted; `_columns_fed` helper rewritten onto the BB announcement path; `test_accept_cup_decision_table` (21 cells) and six accept tests added; nine columns tests gained a `_reload_ready` line because a seated columns goal now always calls BB.

## Verification

- 2026-10-17, `pytest tests/ros/ -q -p no:cacheprovider`: 3114 passed, 2 skipped in 537 s.
- Full gate (`./run_tests.sh --full`, run 2026-10-17): **6716 passed, 9 skipped in 305.80 s** (parallel phase) + **7 passed in 27.87 s** (serial tail), exit 0.

## Phase 2 (same day)

- Interface: `bool reload` deleted from `ros_ws/src/jugglebot_interfaces/action/Juggle.action` (interface rebuild; a FULL launch restart is required before the next sitting). `orchestrator_node._parse_juggle_request` now refuses a `reload` token after the pattern as an unknown token; the bare `reload` PATTERN still dispatches. Bench `--reload` and the fake goal's `reload` field removed.
- GUI (`juggle-panel.js`): the "Reload first" checkbox, `effectiveReload`/`reloadRequired` and the request-string `reload` token are gone. A cup PREVIEW line (`#juggle-cup-preview`, strings in exported `CUP_PREVIEW`) reads the existing ball-held pill that `panels.js` drives from `/hand_telemetry` (no second subscription): "cup EMPTY: Ball Butler loads first" / "cup SEATED: throw first" / "cup UNKNOWN: will be refused", with fixed-requirement text for `reload`, columns_1ball_fed and magazine. The node's decision stays the authority; the preview is advisory.
- Diagram: lob branch/arrow removed, `m.feed` became `m.feeds[]` (columns SEATED = B, EMPTY = A then B, "BB x2"); three dead lob CSS rules deleted. The mock harness takes `?cup=held|empty|unknown`.
- Rehearsal rows 19/21/22 drop `reload: true`; historical sitting records untouched. Stale `reload=True/False` docstrings in `schedule.py`, `skill_node.py` and `test_skill_node.py` reworded.
- Verification (P2): scoped, run 2026-10-17, `pytest tests/ros/test_orchestrator_node.py tests/ros/test_skill_node.py tests/ros/test_skill_node_reload_only.py tests/ros/test_skills_plan_bench.py -q -p no:cacheprovider`: **510 passed in 44.70 s**; `pytest tests/ros/test_gui_juggle_defaults.py -q`: **6 passed**.
- Full gate (P2, `./run_tests.sh --full`, run 2026-10-17 after the audit fixes): **6716 passed, 9 skipped in 307.14 s** (parallel phase) + **7 passed in 27.43 s** (serial tail), exit 0.
- Build (`colcon build --packages-select jugglebot_interfaces jugglebot`, run 2026-10-17, launch down): 2 packages finished in 1 min 55 s, exit 0. FULL launch restart required before the next sitting (Juggle.action changed). Open GUI tabs need a hard refresh after the restart (a stale tab sending `,reload` is refused as an unknown token).

## Phase 3 (same day)

- Node (`skill_node.py`): columns on an EMPTY cup is ACCEPTED (RELOAD_FIRST). `_run_columns(reload_a=True)` runs `_prelevel`, then `_start_reload(kind='reload', then_columns=plan)` with ball A's catch at the columns FEED site (`a_reload_site = bridge_site`; the (-50, 0) fallback is commented above that line). Chain ctx kind `columns_chain`, phases `await_reload_a` -> `await_chain_gate` -> `chaining`. Gate = A's own executor retired clean AND now >= decay_end_abs AND debounced cup SEATED AND BB heartbeat fresh + connected + IDLE + ball_in_hand. New: `_maybe_advance_chain` (ticked), `_resolve_chain_on_retire`, `_chain_guarded` (tick lock + `reload` lock; applies only if `_reload_ctx is chain_ctx and not _ended_code_for_swap()`). `_start_columns_feed(plan, chain_ctx=None)` is the ONE tail shared with the SEATED path.
- `COLUMNS_CHAIN_BB_REFILL_TIMEOUT_S = 15.0` is PROVISIONAL (owner: BB auto-refills ~5-10 s; not measured on a bag; the first sitting measures it; plan rule: raise it if the max refill > 12 s). Gate timeouts: cup UNKNOWN -> `REJECTED_BALL_UNKNOWN`, EMPTY -> `REJECTED_NO_BALL`, else `ABORTED_BB_NOT_READY(state, ball_in_hand)`; a bridge compile refusal during chaining -> `REJECTED_COLUMNS_UNSCHEDULABLE`.
- Two latent holes closed on the way: `_on_announcement` now claims only phase `await_announcement` (a late A announcement could be claimed by the next ctx), and `_start_reload`'s executor/ctx writes are claim-then-swap when chained (a Stop could be resurrected); its three `_end_attempt` calls go through `end_unless_dropped`. GUI `TWO_FEED_LANDED = true` (preview now says "cup EMPTY: Ball Butler loads first" for columns).
- Sim probe (i), 2026-10-17 (scratchpad script, output `temp/probes/reload_at_feed_site.json`): the `reload` choreography is accepted and caught 5/5 seeds at x = -50, -62.5 and -125; site has no effect. The LIVE default separation is 250 mm, so the live feed site is x = -125 (the plan's "-62.5 at sep 125" was the wrong number; corrected in the plan).
- SEPARATE FINDING (not P3; bisected the same day to `08ec3fe6` 2026-09-30, the R5 columns feed-catch commit, good at `8364808c` 2026-09-24, plan § 6 has the detail): the stock `python sim/skills_gate.py --reload --seeds 0 1 2 3 4` is already FAIL at HEAD 4cd33254 (seeds 0, 2, 3, 4 `LIMIT_JERK` in the continuation throws AFTER the catch, seed 1 PASS; its docstring says it once passed 5/5; no test pins it; it predates P1/P2). Not a P3 blocker because A's `reload` is catch-only (`n_throws=0`). Recorded as plan § 6 residue; bisect in progress.
- Coverage honesty: the sim gate certifies each half (`reload` catch at the site; the columns tail), not the join. The join, the phases and the Stop races are mocked-ROS tests only, which are blind to choreography; the first hardware sitting is the join's real gate.
- Full gate (P3, `./run_tests.sh --full`, run 2026-10-18 after the audit fixes W1/W2): **6735 passed, 9 skipped in 310.40 s** (parallel phase) + **7 passed in 27.56 s** (serial tail), exit 0. Interim run 2026-10-17 before the audit: 6729 passed, 9 skipped, exit 0.
- Build (P3, `colcon build --packages-select jugglebot`, run 2026-10-18, launch down): 1 package finished in 3.42 s, exit 0 (interfaces unchanged since P2; the FULL launch restart owed since P2 still stands).

## Outcome

P1-P3 code landed 2026-10-17/18 and the P3 two-feed chain was flown on 2026-10-21 and 2026-10-22 (6 empty-cup chains fed B, refill <= 6.8 s; rows 33/34 unexercised), after which the plan was archived on 2026-10-23.
