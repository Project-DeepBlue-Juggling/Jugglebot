---
title: "JB-175: a fault inside a Catch's judging window ends the Attempt ABORTED_FAULT, not MISSED_CATCH"
type: bugfix
date: 2026-11-06
status: complete
phase: "Night Shift 2026-11-06 JB-175"
related_entries:
  - 2026-11-05-sitting-debrief.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/records.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skill_node.py
  - tests/motion/test_records.py
---

## What and why

Sitting 2026-11-05 22:46 (launch log L2589-2615): a Hand SPINOUT disarm at 503.683 fell 0.03 s before skill 5's judging window closed. The verdict fired at 503.767 ("END MISSED_CATCH: the CATCH of Ball 1 (skill 5) produced no ball") and skill 6's rows were dropped as a miss; the cause was only in earlier lines. The race: orchestrator `ACTIVE -> FAULT` 503.750, trajectory_node mode ERROR 503.756, but the executor reads the fault only through the 5 Hz `trajectory/status`.

Bag measurement (`/trajectory/status`, bag 2026-11-05_22-46-00): last status `TRAJECTORY` 503.692, first `mode=ERROR` 503.894 -- **0.211 s after the disarm, 0.127 s after the verdict**. An operator disarm publishes a different mode (state_machine blanks it to `''`/STANDBY; `ERROR` is published only on a real FAULT), so `ERROR` is the discriminating signal.

Change (a label fix, no motion): `Observations.machine_fault` (default False) from a fresh status with mode `ERROR`; new end code `ABORTED_FAULT`, checked before `in_trajectory_mode` in `tick`; a Catch window due on a faulted machine logs `CATCH-EVIDENCE-UNVERIFIABLE` (no verdict, no lost-ball line, OUTCOME rows kept); a `MISSED_CATCH` end is provisional for `MISSED_CATCH_FAULT_SETTLE_S = 0.4` s (worst case: orchestrator 0.07 s + one 0.2 s status period + margin; measured 0.211 s), inside which a fault seen re-labels `end_code`/`end_message` (one log line, dropped rows stay dropped). `done` is held through the settle. `ABORTED_FAULT` joins `CLOSING_EXCLUDED_END_CODES`; a pending closing decay also cancels on `machine_fault`.

Stop path: `skill_node._on_tick` reads `stop_code` every tick but `_maybe_hold_pending_event` is one-shot per pending event (`_pending_event_mono` cleared on first use), `_force_hold` is armed only for `LANE_REFUSED` behind `_hold_forced_code`, and `_maybe_cancel_bb_throw` pops its record once; a later code change re-enters nothing. `done` only gates retiring the executor (`_goal_end_code`), so the 0.4 s delay waits on no physical action. A guard-only latch keeps the streaming mode on the status, so it is read separately: `Observations.guard_latched` from a fresh `/link_status` `fault_state` not NONE/UNKNOWN/'' (the orchestrator's own latch read; absent or stale = not latched). In `tick` it is checked LAST (`machine_fault` -> mode -> `hand_lane_refused` -> `guard_latched`) because a latch must never pre-empt `LANE_REFUSED`, whose forced hold stops the plan walking into the guard; the settle rewrite touches only a `MISSED_CATCH` end, never `LANE_REFUSED`. Log lines name which signal fired. If the fault appears between the tick's first observation and the evidence pass, the evidence pass ends the attempt itself (`LANE_REFUSED` if the lane is refused, else `ABORTED_FAULT`), never dropping the row with no end.

Bag `/link_status` `fault_state` (same bag): NONE at 503.692, first `ODRIVE_FATAL` at 503.780, i.e. 0.097 s after the disarm, 0.013 s AFTER the 503.767 verdict, and 0.114 s before `trajectory/status` showed ERROR. The 0.4 s settle is unchanged (the guard path is faster than the status path).

Only `guard_latched` is ordered after `LANE_REFUSED`; `machine_fault` keeps the precedence `ABORTED_MODE_CHANGED` already had (a status with mode `ERROR` comes first, and the state machine silences the producer in that case). The evidence pass, which can see a fault the tick's own observation did not, prefers `LANE_REFUSED` when the lane is also refused.

Start of an Attempt with a latched guard: the install refuses `GUARD_LATCHED` and the orchestrator forces FAULT from ACTIVE, but nothing in `skill_node` pre-checks `fault_state`; a latch present at the first tick therefore ends the Attempt `ABORTED_FAULT` at that tick (accepted: it is true, and no motion was dispatched).

Readers of the end code treated: `executor.py` (`CLOSING_EXCLUDED_END_CODES`, closing cancel), `records.py` (falls in `other`; docstring and `test_records.py` pin it), `INVARIANTS.md` (new row), GUI/report/orchestrator carry the string through unchanged (no `ABORTED_MODE_CHANGED` reader there).

## Verification

- Scoped run: 2026-11-06, `pytest -q tests/motion/test_skills_executor.py tests/ros/test_skill_node.py tests/motion/test_skills_report.py tests/motion/test_records.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py`: 774 passed in 89.91 s (after the guard-latch change).
- New tests fail with the production hunk reverted (`git apply -R` of the `ros_ws` diff, then re-applied): 6 failed, as expected.
- The full gate (`./run_tests.sh --full`) runs on the committed tip and its result is in the Night Shift report `docs/night-shift/2026-11-06.md`.
