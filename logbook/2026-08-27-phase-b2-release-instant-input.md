---
title: "The release instant becomes an input — one beat, one derivation"
type: refactor
date: 2026-08-27
status: resolved
phase: "toss-pipelined-preamble — Phase B2"
related_plan: toss-pipelined-preamble.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_session.py
  - tests/ros/test_toss_coordinator.py
subsystem:
  - ros
tags:
  - refactor
  - toss
  - cadence
---

# The release instant becomes an input — one beat, one derivation

## Summary

Phase C replaces "next release = previous landing + dwell" with a free-running
beat clock. Phase B2 makes that a change of *who computes a number*, not a change
of either FSM (plan § 2.6).

* **`TossSequencer.release_at_perf`** — the ABSOLUTE scheduled release on the perf
  clock. `start(now)` sets `_t_release = release_at_perf or (now + throw_delay_s)`,
  so 0.0 (the shipped value on both call sites) is bit-for-bit today's arithmetic.
  `_t_release` is now written in exactly one place and only READ thereafter — the
  guard, the release/settle deadlines, Tier 8b's time-triggered reach, the
  announcement, the dispatch's `event_delay`, the cancel cutoff and the record all
  already read it. **No consumer re-derives it; none had to be changed.**
* **The sentinel doctrine gets its other half.** 0.0 means "derive"; a negative
  (the preserved sign typo) or non-finite value is refused loudly at CHECKING as
  `REJECTED_RELEASE_SCHEDULE`. Without it a NaN release would pass every gate
  above it — they are all keyed on `throw_delay_s` — and then wedge the cycle:
  nothing compares true against NaN, so neither deadline can fire and the goal
  sits armed until the node ceiling SAFE_ABORTs under a throw that never came.
* **`TossSequencer.scheduled_lead_s`** — accept → scheduled release. `throw_delay_s`
  is that lead only on the derived path, and `_toss_deadline_s` sizes the per-goal
  ceiling on it: size the ceiling off the delay while the cycle runs on an absolute
  release further out and the ceiling lands INSIDE a legitimate window, whose exit
  path is a SAFE_ABORT (retract under an airborne ball). The accessor answers
  `throw_delay_s` **exactly** on the derived path and for an unstarted sequencer
  (the throwaway one the session ceiling budgets from), so today's ceilings are
  bit-unchanged — deliberately not `_t_release − _t_accept`, which is the same
  number only to within a float ulp at a real `perf_counter` origin.
* **`TossSessionSequencer.next_release_at(landing)`** — THE beat, `landing + dwell`,
  one derivation with two callers: the session's own `note_cycle_result` (which
  subtracts one `throw_delay_s` to get the cycle START) and the node's
  `_set_toss_next_cycle_perf`, which had carried a second copy of the same
  expression for the cadence clamp. Phase C replaces that body and nothing else in
  either FSM learns where the beat came from.
* **`_build_toss_cycle(..., release_at_perf=0.0)`** threads it to the FSM, so the
  cycle builder stays ignorant of where a beat comes from.

## Discussion — why the session call site still takes the derived default

The tempting last step is to have the session hand `next_release_at(...)` straight
to `_build_toss_cycle`. Not yet, and not for tidiness: `START_CYCLE` is **polled**,
so `now` is up to one loop period past `next_cycle_at` and handing the beat over
would silently ABSORB that jitter into the cycle's lead — silently, because
nothing measures it until B4 populates `slip_s`. Worse, nothing would catch the
shortfall: CHECKING's accept-time floor (`min_throw_delay_for_cycle_s`) is charged
against `throw_delay_s`, so a lead shortened by an absolute release clears the gate
and dies at the runtime guard instead — `ABORTED_CANT_MAKE_RELEASE` with the
platform moved and the latch up, which is the exact accept-vs-runtime gap the
2026-08-22 audit closed. B4's `commit_budget_s` and COMMIT gate are what re-close
it for an absolute schedule, so B4 supplies the argument. The reasoning is written
at the call site, not only here.

`slip_s` ships as scaffolding for the same reason — it reads 0.0 for every cycle
until B4's COMMITTING phase writes it. Defining it now is § 2.6 rule 3: B4 and
Phase C's bounded-slip policy then *populate* an agreed quantity rather than each
inventing one, and a slip the FSM absorbs silently is indistinguishable from a
cadence that never slipped.

## Verification

Scoped, 2026-08-27: `python -m pytest tests/ros/test_toss_sequencer.py
tests/ros/test_toss_session.py tests/ros/test_toss_coordinator.py
tests/ros/test_toss_continuous_node.py -q` → **531 passed in 97.47 s**. Eleven
tests added; **no existing assertion edited** (the four files carried 520 before).

Whole directory, 2026-08-27: `python -m pytest tests/ros/ -q -p no:randomly` →
**2428 passed, 1 skipped in 321.16 s**.

Cross-check on the toss-adjacent files, 2026-08-27: `python -m pytest
tests/ros/test_toss_calibration.py tests/ros/test_toss_ilc_node.py
tests/ros/test_toss_trim_node.py tests/ros/test_toss_integration.py
tests/ros/test_possession_replay.py tests/motion/test_toss_cal.py
tests/motion/test_toss_record.py tests/motion/test_cadence_rung_check.py -q` →
**225 passed in 24.52 s**. The full gate is the orchestrator's.

The acceptance test is T-U12,
`test_the_release_instant_is_an_input_not_a_rederivation` (parametrised over both
tiers): the same goal built two ways — derived (delay 5.0 from `start(0.0)`) and
absolute (`release_at_perf` 5.0 carrying a **different** 3.0 s delay) — produces
identical `(now, phase, action, done, outcome)` traces across an entire uncaught
cycle, with the tail instants straddling `t_release`, the CATCHING threshold and
the settle deadline. The 3.0 is what makes it evidence rather than tautology: a
re-derivation is the only thing that could put the release at 3.0, and the test
also asserts that the FSM a re-derivation *would* have produced traces
differently, so the comparison cannot pass on a trace too coarse to see the
schedule.

## Outcome

B2 done, behaviour-preserving. B3 (session-scoped arming + drain-before-`go_home`)
is next and does not depend on this; B4 depends on both. B4's entry points are
exactly three lines: the `release_at_perf=` argument at the session's
`_build_toss_cycle` call, `commit_at = _t_release − commit_budget_s`, and the
write to `_commit_slip_s` from `_step_committing`.
