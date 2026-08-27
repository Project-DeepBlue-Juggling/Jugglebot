---
title: "Session-scoped arming — the arm-mid-move seam closed by construction"
type: refactor
date: 2026-08-27
status: resolved
phase: "toss-pipelined-preamble — Phase B3"
related_plan: toss-pipelined-preamble.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_integration.py
subsystem:
  - ros
tags:
  - refactor
  - toss
  - cadence
  - safety
---

# Session-scoped arming — the arm-mid-move seam closed by construction

## Summary

Phase B3 lands invariants **S6** (the catch latch and the catch-coordinator holds
are session-scoped) and **S7** (the pipeline is drained before any `go_home`), and
re-argues **S5′** in writing. **This is a deliberate behaviour change** — the first
in this plan — and it is valuable on its own whether or not B4 ever lands.

* **`_arm_session_declare` + `_arm_session`** — a contiguous run of chained cycles
  now raises `catch/prime_hold`, `catch/reach_center`, `catch/pretilt_hold`,
  `set_hand_gains`, `trajectory/arm_catch` and `catch/vel_scale` **exactly once**,
  on the FIRST cycle's verified-arrival and PREPARE ticks. Every later cycle finds
  them standing.
* **`_prepare_toss_catch` narrows** to `catch/prime_dispatched`, `catch/armed` and
  the phantom-flight snapshot — three topic publishes and **no service round trip
  at all** on a chained cycle.
* **`catch/armed` stays per-cycle** (the plan's 2026-08-27 S6 amendment): it
  installs no graceful stop, and the bench trace recorder's `cycle_spans` segments
  every CS check off its edges.
* **`_drain_pipeline_and_disarm`** fronts every `go_home` call site — six of them.
  At B3 the drain half is a documented placeholder (no staged slot exists yet) and
  the disarm half is real, so B4 adds the discard in one place instead of
  re-auditing six teardown ladders.
* **`_disarm_session`** lowers all of it once, in the unchanged order: `catch/armed`
  False → latch → `prime_hold` → `pretilt_hold`.
* **A reach-centre drift guard** refuses a cycle whose nominated B has left the
  session's declared envelope, `REJECTED_REACH_CENTER_DRIFT`, before anything is
  armed.
* **The single `Toss` action is untouched.** `_toss_session_live` is False for it,
  so it takes the per-cycle branch and its publish sequence is byte-identical.

## Discussion

### Why session scope, stated as the failure it prevents

`trajectory_node` prints *"catch latch armed mid-move — installed a graceful stop
(move silenced)"* whenever an `arm_catch` raise lands while a profile is in flight.
It fired on **10 of the 16 post-MISS toss cycles** of bag `2026-08-26_14-25-16`:
the next cycle's PREPARE re-raised the latch while the previous cycle's SAFE_ABORT
`go_home` was still traversing. The remedy shipped then was to stretch
`DEFAULT_SESSION_MISS_CLEANUP_S` to 2.80 s so the arm lands after the profile —
**a timing fence over a race**, and the thing a race gets you is a throw from a
site the aim was not solved for: a mis-aimed ball at 4.4 m/s.

S6 removes the raise. A resource that is never lowered mid-run cannot be re-raised
at the wrong moment. S7 removes the other half: no `go_home` is installed while
anything is armed. Together the seam is closed **by construction**, not by
arithmetic. The cleanup floor stays — it protects the retract's descent and the
throw site — but it stops being the only thing holding the seam shut, and
`DEFAULT_SESSION_MISS_CLEANUP_S` must not be lowered while B3 is in (plan § 9.5).

### The accepted tradeoff: 97 % → 100 % armed duty cycle

The honest cost is that the dwell is no longer quiescent. `catch/armed` still goes
down at each cycle's terminal, but the *latch* and both holds stand for the whole
run, so `catch_coordinator`'s reactive path is live across the between-cycle gap
too. At the milestone dwells the armed window was **already ~97 % of wall time**
(0.435 s dwell inside a 1.34 s period, against a catch armed from PREPARE to
terminal), so S6 converts a 97 % duty cycle into 100 % rather than creating a new
state — but "already 97 %" is an argument about magnitude, not about kind, and the
kind does change: **a foreign tracked ball entering the volume between cycles now
meets an armed machine where before it met one 97 % of the time.**

Accepted, on three shipped and unconditional mitigations: `catch/pretilt_hold`
stands for the whole run so no announcement pre-tilt can command motion;
`catch/prime_hold` stands so no armed-edge auto-prime can ascend with a seated
ball; and C-REACH-1 centres the reach envelope on the nominated B, so any commanded
reach is bounded at 80 mm. The runbook keeps the by-eye watch (row PIPE-5: **any**
commanded platform motion between a verdict and the next release stops the
sitting), and PIPE-7 is an ABSENCE acceptance — zero *"catch latch armed mid-move"*
lines across the B6 sitting.

S5′'s other two points were re-taken and are in `toss_session.py`'s module
docstring rather than only here, because that is where a future session reads them:
point 2 (an armed dwell looks like an about-to-throw machine) was already true and
already documented at these cadences; point 3 (a stretched delay moves every
cycle's internal timing) is **unaffected**, and it is the load-bearing half — S6
moves no motion instant. From PREPARE onward a chained cycle is byte-identical to
a validated single toss. What left the cycle is three service round trips and three
topic publishes, none of which command anything.

### Why the session arm is split across two FSM ticks

The plan asked for one `_arm_session`. It is two methods, and the reason is a
contract this phase must not weaken.

`catch/reach_center` is a **topic publish**; `trajectory/arm_catch` is a **service
call**; they travel on different transports with no ordering guarantee, and the
raise is where `trajectory_node` read-and-clears the declaration. The pre-S6 code
bought that ordering with one full FSM tick (`_TICK_S` = 20 ms, ~2 orders of
magnitude above localhost topic latency) and `test_reach_centre_declared_a_tick_
before_the_arm_raise` pins it. Collapsing the two into one call would have made the
gap a race — and S6 makes losing that race **strictly worse**, not better: with one
raise per session, a lost declaration mis-centres the envelope for the **whole
run** instead of for one cycle. Same argument for `catch/prime_hold`, whose whole
reason for being published alone on the earlier tick is that it must land in an
*earlier* catch_coordinator wait-set cycle than the armed edge.

So `_arm_session_declare` runs on the verified-arrival tick (holds + declaration)
and `_arm_session` on the PREPARE tick one tick later (gains → raise → vel_scale).
The **relative** order the plan called load-bearing is preserved exactly, and it is
now strictly stronger than what it replaced: gains → arm raise → vel_scale precede
every cycle's armed edge by *seconds* rather than by microseconds.

### Placement: after cycle 1's arrival, not at session start

`_arm_catch(True)` C2-stops any in-flight move. The plan admitted two placements —
before cycle 1's positioning is dispatched, or after that move's verified arrival.
§ 9.3 states the second and this takes it, because the first is worse in exactly
the case PIPE-7 greps for: nothing refuses a goal issued while a move is executing
(C-REACH-1 residual 8), and arming at session start would C2-stop that move and
print the very line the acceptance is an absence of. Arming after arrival keeps
cycle 1 identical to today and still leaves zero raises for cycles 2…N.

### Q-3's foreclosure, and why the guard fails loudly

The diagnosis behind Q-3 is worse than the plan first assumed: `_svc_arm_catch`
read-and-clears the pending `catch/reach_center` **before** its idempotent early
return, so under a standing latch every per-cycle declaration is *consumed and
discarded* and the envelope centre stays frozen at whatever the session raise
captured. The adopted fix is to scope the declaration the way S6 scopes the raise —
zero `trajectory_node` change, and it also closes C-REACH-1 residual 5's
leaked-pending hazard, since there is no later raise to consume a stale one.

That forecloses per-cycle-varying B, so it must fail loudly rather than
mid-flight. The guard's bound is **derived, not chosen**:

```
    tol = JB_TRAJ_CATCH_REACH_ENVELOPE_MM                       # 80.00 mm
        − HAND_CATCH_OFFSET_MM · sin(MAX_TILT_DEG)              # 64.78 · sin 12° = 13.47 mm
        = 66.53 mm
```

The subtrahend is the systematic swing shift the reach itself carries:
`_publish_toss_reach` goes through the same `predicted_catch_command` /
`compute_catch_orientation` policy the reload does, and the tilt clamp **saturates**
on every real arrival, so 13.47 mm is a bound and not a datum. That is the same
arithmetic `_RELOAD_CENTERED_TOL_MM` makes one path over, for the same reason —
a gate that says yes to a band the envelope will then say no to *after the
irreversible commitment*. The two constants stay separate names (they answer
different questions) and sit next to each other so an edit to either sees the
other.

What the margin does **not** reserve is the drift budget: C-REACH-1 § 1 allocates
the whole 80 mm to *unrequested* excursion, so every millimetre of B-drift is spent
out of it. The guard is a last line, not a design point. Every session in scope
sits at drift **0** — `catch_position` is one goal field, constant for the whole
session — so a non-zero drift means the machine is doing something this phase
foreclosed. The documented forward path is the redundant-raise capture in
`trajectory_node`, taken with its own evidence, never a widening of this number.

The refusal is a `REJECTED_`, not an `ABORTED_`, and that needed one new seam:
`note_prepare_result(ok, reject_code='')`. A REJECTED names an operator-visible
refusal that carries the forward path in its log line; an ABORTED reads as a plant
fault and sends the operator to the wrong subsystem — the same two-codes-for-two-
subsystems rule `REJECTED_NO_BALL` / `REJECTED_BALL_UNKNOWN` were split under. The
default is unchanged, so no existing producer changes meaning.

### The tick S6 makes free is deliberately not reclaimed

The ≥1-tick armed→announce gap exists because `catch_coordinator` drops
announcement pre-tilts that arrive unarmed. Under S6 that gap is satisfied by a
raise seconds earlier — but the constant stays and the rationale is restated at
both places that encode it (`_step_preparing`'s deferral and
`pre_dispatch_budget_s`'s ladder). Reclaiming the tick *here* would shorten the
accept-time floor under a runtime guard that had not moved, which is the exact
accept-vs-runtime gap the 2026-08-22 audit closed and the two
`ABORTED_CANT_MAKE_RELEASE` cycles of `2026-08-26_14-25-16` cost. It is returned in
B4's `commit_budget_s`, where the whole ladder is re-derived at once by a gate that
charges the real lead. Two branches, one derivation each.

### Why `_toss_session_owns_holds` keys on the live flag, not the declaration

First cut keyed the per-cycle hold release on "has this run declared a centre".
`_disarm_session` clears that latch as its last act, so `_toss_safe_abort` — which
drains first and then runs the per-cycle tail — found the session "no longer
owning" holds it had *just* released and published a second `prime_hold` False.
Harmless, but it is the wrong question. The tail asks *whose holds are these*, and
in a session the answer is the session's for the whole run, drain included. Caught
by `test_a_chained_safe_abort_drains_before_the_go_home` asserting the exact
ladder rather than a set of orderings.

## Verification

Scoped, 2026-08-27: `python -m pytest tests/ros/test_toss_coordinator.py
tests/ros/test_toss_continuous_node.py tests/ros/test_toss_integration.py -q`
→ **304 passed in 98.95 s**.

Whole directory, 2026-08-27: `python -m pytest tests/ros -q -p no:randomly` →
**2452 passed, 1 skipped in 325.45 s** (2428 + 1 before this phase; 24 tests
added).

Cross-check on the adjacent readers, 2026-08-27: `python -m pytest
tests/ros/test_toss_calibration.py tests/ros/test_reload_sequencer.py
tests/ros/test_reload_coordinator_node.py tests/motion/test_cadence_rung_check.py
tests/motion/test_toss_record.py tests/ros/test_possession_replay.py
tests/ros/test_ball_possession.py -q` → **340 passed in 11.78 s**. The full
`./run_tests.sh --full` gate is the orchestrator's, before the commit.

**Changed assertions: one.** `test_a_chained_safe_abort_drains_before_the_go_home`
is new, so nothing pre-existing moved — the whole suite passed unedited both
before and after the `_toss_session_owns_holds` fix above. Every other test in this
phase is additive.

Two acceptance tests are worth naming:

* **T-I2** (`test_session_holds_suppress_ccn_autoprime_for_a_whole_session`) drives
  the real `CatchCoordinatorNode` through a session arm, three chained cycles and
  the session disarm, and pins an **absence**: no auto-prime on any armed edge, on
  any between-cycle retry tick, or anywhere else, and no announcement pre-tilt
  installed. Non-vacuity is built in — the last two lines prove the auto-prime path
  is reachable by firing it once the hold is released. A probe confirmed the same
  for the retry tick specifically (with the hold released it primes, so
  `primed == []` is a suppression and not a blocked path).
* **T-I3** (`test_the_session_declaration_is_captured_once_and_admits_every_cycle`)
  drives the real `trajectory_node` and asserts the **captured value** of
  `_catch_envelope_center`, never publish ordering. That distinction is the whole
  Q-3 finding: the trace recorder's CS-4 (one declaration per cycle, ≥1 tick before
  the arm) stays green while the declaration goes unapplied, so CS-4 alone is a
  false green. The test also pins that a later declaration under the standing latch
  is consumed and discarded without moving the centre, and that a divergent B is
  refused by the drift guard *before* the mid-flight `WORKSPACE` reject it
  forecloses — which the test then demonstrates on the next line.

## Open questions

* **CS-4 must be re-cut for pipelined sessions** — the trace recorder's per-cycle
  declaration check is now a false green for a chained run. Flagged, not fixed:
  `tests/hardware/toss_trace_recorder.py` is B6's job.
* **PIPE-7 is the acceptance and it is an absence** — a bench grep for *"catch latch
  armed mid-move"* across the B6 sitting must return zero lines. Nothing offline can
  establish that.
* A session that continues past a SAFE_ABORT (`stop_on_miss` False) or through a
  reload interlude **re-arms** on its next cycle, by design: the drain ends the
  contiguous run and the next cycle declares afresh. That is one extra `arm_catch`
  pair per interruption, all of it off the cadence path.

## Outcome

B3 done. B4 (the two-slot pipeline) depends on B1, B2 and this; its entry point
here is the one commented placeholder inside `_drain_pipeline_and_disarm`. Deploy
needs `cd ros_ws && colcon build --packages-select jugglebot` — no firmware, no
config regeneration.
