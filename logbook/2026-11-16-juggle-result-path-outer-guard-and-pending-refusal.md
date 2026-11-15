---
title: "Juggle action Result path: an outer guard that always ends the action, and a refusal while the Result is being built (BUG-97)"
type: bugfix
date: 2026-11-16
status: complete
phase: night-shift
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - logbook/INDEX.md
subsystem:
  - skill-stack
related_items: [BUG-97]
---

# Juggle action Result path: outer guard and pending-Result refusal

**What.** Both weaknesses in BUG-97 were confirmed by reading the code and by a test that fails without the change.

1. *No outer guard: confirmed.* `_juggle_execute` had only two local try/excepts; a raise in `rp.record_counts`, `rp.drop_count`, `_offer_record` and the rest propagated out of the callback, so the goal handle was never ended. The body is now `_juggle_execute_body`; `_juggle_execute` wraps it, and on any exception logs, brings a still-running attempt to its rest tail (`_stop_attempt`, only if the done event is not set), aborts the goal handle, clears `_goal_armed` when the attempt is already done, and returns a Result with `outcome='ERROR'`, `success=False`. A `finally` drops the pending mark. The success path is the old body, unchanged.
2. *Reset race: confirmed.* `_maybe_signal_goal_done` sets the done event; from then `_executor is None`, no reload context, no magazine context, `_goal_starting` False, so `_refuse_if_running` returned `None` (`_goal_armed` is not part of it and stays True after the attempt ends). A second `_start_pattern` in that window runs `_start_pattern_body`'s field reset while `_juggle_execute` still reads `_goal_reports`, `_goal_missed_catches` and the Feed fields. New flag `_goal_result_pending`, set under `_reload_lock` where the done event is set (the tick path and `_magazine_finish`) but only while a `_juggle_execute` callback is in flight (`_juggle_execute_active`, set at its top, cleared with the pending mark in its `finally`; if the done event is already set at the top, pending is set there). Without that gate, an execute that raised mid-Attempt left `_goal_armed` True, the stopped Attempt later set pending with nobody to clear it, and every request was refused until a relaunch (orchestrator review). `_refuse_if_running` counts it as running (this also makes `skills/nominate` answer "Attempt running" until the Result is out, which is correct: the fields are still in use). The smaller of the two fixes the Item offered; no lock is taken in the Result path.

**Why this and not a snapshot.** A snapshot at event-set time would still let the second goal's start clear the event and re-arm while the first Result is in flight, and would leave two goals sharing the ledger window. Refusing keeps one goal's fields owned by one goal. Residual: none known for the pending mark; it can only be set while an execute callback is live, and that callback's `finally` always clears it. No timing, Schedule, executor or learner code was touched.

## Verification

- 2026-11-16, `night_shift_nonet.sh nice -n 19 env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python -m pytest -q tests/ros/test_skill_node.py tests/ros/test_skill_node_magazine.py tests/ros/test_skill_node_reload_only.py`: 441 passed in 187.72s (0:03:07).
- New tests `test_a_raise_while_building_the_result_still_ends_the_action`, `test_a_second_request_is_refused_until_the_result_is_built` (checks the refusal from inside the Result path) and `test_a_raise_mid_attempt_does_not_leave_a_later_request_refused` each FAILED with the matching `skill_node.py` change removed (the first two with the whole change stashed, the third with only the `_juggle_execute_active` gate removed) and pass with it.
