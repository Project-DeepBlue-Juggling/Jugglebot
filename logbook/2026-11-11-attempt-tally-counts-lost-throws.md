---
title: "The Attempt tally counts a Throw lost to an empty Cup in M (FEAT-48)"
type: feature
date: 2026-11-11
status: resolved
phase: ""
subsystem: skill-stack/gui
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/report.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/motion/test_skills_report.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_magazine.py
  - logbook/INDEX.md
  - map/feature-attempt-end-line-and-juggle-result-count-a-throw-lost-to-an.md
tags: [FEAT-48, FEAT-47, tally]
related_items: [FEAT-48, FEAT-47]
---

# The Attempt tally counts a Throw lost to an empty Cup in M

## What and why

A Catch that ends MISSED_CATCH on an EMPTY Cup drops its ball's outcome rows, so the lost Throw never gets a ThrowReport and the operator's tally omitted it: 10 Throws, 9 caught, the tenth lost, read "9/9 caught" (210 Attempts in six weeks of logs). Owner decision 2026-11-11: M (Throws) counts the lost Throw; N (caught) is unchanged, so that example reads "9/10 caught".

`SkillExecutor` is untouched. The per-goal count FEAT-47 already records (`missed_catch_drops`, summed in `skill_node` as `_goal_missed_catches`) is threaded into the tally under the same name as `drop_count`'s argument, `missed_catches`: `report.summarise`, `report.record_counts` and `report.end_line` take it, negatives ignored, and it adds to Throws only. The end line says why, only when it is nonzero: `9/10 caught (1 lost to an empty Cup)`; with zero the text is byte-identical to before.

- Before: `columns ENDED (MISSED_CATCH) · 9/9 caught · apex ...`. After: `columns ENDED (MISSED_CATCH) · 9/10 caught (1 lost to an empty Cup) · apex ...`.
- Agree on M: the end line (both call sites, `_log_attempt_end` and the no-executor `_end_attempt` line), the Juggle Result, the Attempt end event (`throws`), the ledger row's Throws (`record_counts`).
- Not changed: the action Feedback keeps `rp.summarise(self._goal_reports)`. Lost Throws are summed only when an executor retires, so a running figure would be late or wrong mid-goal; the Result at the end is exact.
- The Feed rule: a missed Ball Butler Feed is NOT added to M (M counts Jugglebot's Throws); it stays in drops only (`_goal_feed_missed`). `magazine` is unchanged (`missed_catches` forced to 0 for the tally; zero drops as before). Stop Throws and an unverifiable final Catch keep their treatment. The drops count is unchanged (one lost Throw is 1 drop, not 2).

## Readers of these numbers and the end-line text

Enumerated with `grep -rn` for `summarise(`, `end_line(`, `record_counts(`, `drop_count(`, the Result/Feedback fields and `caught` / `ENDED (` / ` done:` in `tools/`, `mapkit/`, `.claude/`, `ros_ws/gui/`, `sim/`, `tests/`:
- Display only: `skill_node.py` (end lines, Result, Attempt event, feedback), `records.py` (the ledger's Throws column is shown; ranking is by Catches only, `apply_attempt`: Throws never enters the ranking or the tie-break, so no "new record" decision moves), `docs/records.md`, `.claude/commands/sitting-debrief.md` (`grep -n 'ENDED (.*caught'`, a human read), `tools/probes/limit_refusal_tally.py` (`re.search(r'(\d+)/(\d+) caught')`, tolerant of the suffix).
- A verdict: `tools/probes/hold_rung_bias.py` reads the end line's `N/M caught` (`_CAUGHT.search`, suffix-tolerant) into `end_caught` / `end_total`, and its `adopt()` rung rule requires `caught == total`. A goal that lost a Throw to an empty Cup now has total > caught where it used to read all caught, so the rung can no longer be adopted over a lost Throw. That is the intended truth (a lost Throw is a miss), it affects only logs written from now on, and it is an offline analysis probe, not a gate or the Runbook; flagged here for the owner.
- No learner input, Night Shift or debrief parser reads these numbers.
- Ledger rows written before today were counted WITHOUT lost Throws; they are not rewritten.

## Coverage gap closed

`test_the_executors_own_counts_reach_the_goal_through_the_retire_path` drives a REAL `SkillExecutor` (the executor tests' own `_run_inbound_lost` / `_feed_catch_schedule` fixtures) to a Cup-EMPTY MISSED_CATCH and to a Feed-Catch MISSED_CATCH, retires each through the node's `_on_tick`, and shows `_goal_missed_catches` / `_goal_feed_missed` arrive from the executor and the end line follows.

## Verification

Run 2026-11-11 (scratch worktree of origin/main): `./run_tests.sh -q` (the default gate): **PASS, exit 0, parallel 560 s + serial 32 s = 592 s** (an earlier run of the same command failed only `test_map_items_rated` for FEAT-48 being unrated on close; rated owner none, radius subsystem, then the rerun above). `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_mapkit*.py tests/sim/test_plans_index.py tests/sim/test_map_items.py tests/sim/test_map_items_rated.py -q`: **481 passed, 37 skipped**. New tests: four in `test_skills_report.py` (summarise, end line 9/10 and byte-identical at zero, record_counts, drops unchanged), three node tests (9/10 on the end line, Result, Attempt event and ledger row with drops still 1; the unchanged tally; a Feed miss leaves M), the real-executor retire-path test, and the magazine Result pin (throws 1 even with lost Throws set; it passes without the change too, a guard only).
