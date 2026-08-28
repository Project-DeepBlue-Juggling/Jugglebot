---
title: "First contact with the pipeline — one late tick, one deadlock, three anomalies"
type: bugfix
date: 2026-08-28
status: resolved
phase: "toss-pipelined-preamble — first pipelined sitting, fix wave"
related_plan: toss-pipelined-preamble.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - ros_ws/src/jugglebot_interfaces/action/TossContinuous.action
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_session.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_toss_pipeline_properties.py
  - tests/motion/test_toss_record.py
  - tests/motion/test_cadence_rung_check.py
  - tools/probes/cadence_rung_check.py
  - tools/probes/toss_loop_census.py
subsystem:
  - ros
tags:
  - bugfix
  - toss
  - pipeline
  - cadence
  - safety
  - deadlock
---

# First contact with the pipeline — one late tick, one deadlock, three anomalies

## Summary

The first sitting with `toss_pipeline_enabled: true` produced three operator
anomalies — cycles dying `ABORTED_CANT_MAKE_RELEASE`, goals that stopped
responding, and later ball-ops refused `REJECTED_BUSY`. **They are one
mechanism, in two stages**, and the bag confirms it end to end:

1. **a single loop iteration longer than `NODE_LOOP_PERIOD_S` at the commit
   crossing aborted the staged cycle.** `commit_budget_s` grants exactly one
   NOMINAL period of polling lateness plus 1 µs of representation slack, so the
   runtime release-window guard fails on any overshoot at all —
   velocity-independent, cadence-independent, unbuyable with lead. 4 of the
   sitting's 15 staged slots died this way;
2. **that abort then deadlocked the session.** `note_stage_abandoned` raised
   `_stage_declined`, whose only clearer is `note_cycle_result` — but for a
   COMMIT-TIME abandonment the upstream cycle terminalised earlier in the same
   tick, so the clearer had already run. The session answered
   `DWELL / ACTION_NONE / done=False` for the rest of the goal, holding the
   node's cross-action `_goal_claimed` until something else ended the goal.

Five owner-approved fixes landed. **F1** breaks the deadlock (belt and braces).
**F2** converts the commit gate's late-tick shortfall from an abort into a SLIP,
which is the class fix — the plan's own § 9.2 doctrine ("slip rather than
refuse") already routes `hand_parked` and `ball_seated` that way. **F3** adds a
session no-progress watchdog behind F1. **F5** makes the surviving
(slip-bound-exhausted) abort forensics-complete in one line. **F6** labels the
staged slot's `COMMITTING` feedback in the log without touching the wire
vocabulary.

`NODE_LOOP_PERIOD_S` is deliberately **NOT re-cut** — the owner deferred that
past the next sitting, and § Discussion explains why the census's 0.060 ask was
an artefact of abort semantics.

## Symptom

Operator report from the sitting, and the bag
`temp/logs/toss_records_20260828-103139-1125795.jsonl` (28 records, 7 goals,
one node process):

| outcome | n |
| --- | --- |
| `CAUGHT` | 12 |
| `MISSED` | 5 |
| `DISCARDED_DRAINED` | 4 |
| `ABORTED_CANT_MAKE_RELEASE` | 4 |
| `REJECTED_NO_BALL` | 3 |

* **the four aborts are all STAGED slots** (`staged_at_s` set,
  `staged_discarded_reason == ABORTED_CANT_MAKE_RELEASE`), at cycle index 2 or
  3, across four different goals — `1f47bc7d`, `28757d5d`, `ce5d560a`,
  `4bb3ae9e`;
* **every one of those four goals ends there.** No further record was written
  for any of them, on a `num_throws = 5` goal that had completed one or two
  cycles. Those are the four hung goals;
* **15 staged slots in total**, so the abort rate on the pipeline's new path was
  **4/15 (27 %)**;
* the sitting's throw parameters were unremarkable and varied — `h` 0.50 to
  1.30 m, `T` 0.639 to 1.030 s, dwell 0.45 to 0.76 s, delay 5.0 s throughout,
  tier 8b. The aborts are spread across three different heights, which is the
  first hint that the mechanism is not a cadence one.

## Diagnosis

### D1 — the commit gate's runtime guard is one nominal loop period wide

`toss_sequencer._step_committing`, rung 6:

```python
if self._t_release - now < self.min_event_delay_for_throw_s:
    return self._abort('CANT_MAKE_RELEASE')
```

and the budget the schedule is built from:

```python
commit_budget_s = dispatch_s + loop_period_s + FLOOR_REPRESENTATION_SLACK_S
```

The commit instant is `_t_release − commit_budget_s`, so on the tick that
CROSSES it the remaining lead is `commit_budget_s − (now − commit_at)`, where
`now − commit_at` is the polling lateness. The guard therefore fails **iff the
iteration that reached the commit was longer than
`NODE_LOOP_PERIOD_S + 1 µs`** — and the same arithmetic holds after a slip,
because a slip re-arms `_t_release = now + commit_budget_s`.

Two properties of that inequality are what make it a design finding:

* **`dispatch_s` cancels.** It is on both sides — inside the budget and as the
  threshold — so the failure threshold is `NODE_LOOP_PERIOD_S + slack` at every
  flight time and every release speed. No lead, no dwell and no cadence buys
  immunity. (Pinned by
  `test_the_late_tick_shortfall_is_independent_of_the_release_speed`.)
* **the machine supplies that overshoot routinely.** From the sitting's own
  census fields, on the four aborted cycles: `loop_period_max_pre_s` 0.0406 to
  0.0462 s against a 0.040 s nominal, with `loop_n_over_pre` 3 to 5 out of 5 to
  7 pre-dispatch iterations. The Jetson's `time.sleep` is non-RT.

The `min_stage_lead_for_release_s` docstring claimed this made
`ABORTED_CANT_MAKE_RELEASE` **"structurally unreachable on the pipelined
path"**, and `_slip`'s claimed its release bound **"holds by construction"**.
Both are refuted: the first is a statement about arithmetic in a machine whose
loop period is measured rather than guaranteed; the second holds only while the
actual period does not exceed the nominal one.

### D2 — the abort then deadlocks the session, by construction

`reload_coordinator_node._tick_toss_pipeline` steps the committed slot first
(S1′), and on a CAUGHT terminal it runs, in this order, inside ONE tick:

```
note_cycle_result(...)            # _committed_live -> False, _stage_declined -> False
note_upstream_terminalised()      # the staged slot's commit gate may now pass
<staged slot steps>               # …and refuses at rung 6
note_stage_abandoned(...)         # _cycle_live -> False, _stage_declined -> True
```

`_stage_declined`'s **only** clearer is `note_cycle_result`, and it has already
run. `step()` gated on `if self._cycle_live or self._stage_declined:` — with
`_cycle_live` False, `_committed_live` False and `_stage_declined` True, every
subsequent `step()` returns `ACTION_NONE`, `done=False`, forever. Reproduced
offline against the real `TossSessionSequencer` at **+90 000 s** (~25 h of
session time).

**The deadlock is specific to the CAUGHT path**, and that is why three of the
sitting's seven goals survived their own staged discards. A not-caught committed
terminal runs the S7 drain on its way to `go_home`, and the drain discards the
staged slot **before** `note_cycle_result` — so `_committed_live` is still true,
the flag's wait has a waitee, and the very next `note_cycle_result` clears it.
The bag agrees exactly: the four `DISCARDED_DRAINED` records all belong to goals
that continued.

The wedge's cost is `_goal_claimed`, taken at accept in `_goal_callback` and
released only in the execute callback's `finally`. It is shared across all three
ball-op actions, so for as long as the goal hangs, every Reload / Toss /
TossContinuous the operator sends is `REJECTED_BUSY`. That is anomaly three.

### What the evidence did NOT support

**"the goal hangs for the process lifetime."** It does not. The outer session
loop's own ceiling (`_toss_session_deadline_s`) is evaluated on the pipelined
path every tick, and for the sitting's goals it is **270.9 s**:

```
5 × (_MAX_SEQUENCE_S 30.0 + dwell 0.50)   = 152.5 s   num_throws 5
+ _reload_interlude_budget_s              = 113.4 s   ← the term that gets forgotten
+ _SEQUENCE_CEILING_MARGIN_S              =   5.0 s
                                          = 270.9 s   (270.6 s at dwell 0.45)
```

A wedged goal terminalises `ABORTED_TIMEOUT` there and the `finally` releases
the claim.

⚠ **This paragraph read 157.5 s until the fix wave's audit**, which is the
ceiling with `reload_budget_s` dropped — and it is not droppable for this
sitting: **every one of the 28 records carries `on_empty_cup: RELOAD` with
`max_reloads: 3`**, so all seven goals paid the full 113.4 s allowance. That
term is three attempts of (BB arrival window + go-home + recentre verify pad +
a whole reload-sequence ceiling + the post-reload settle floor), and it is in
the ceiling precisely so a session that uses its reload budget is not aborted
for doing what it was asked to do. Under-quoting the ceiling by 113 s
under-states what a wedge costs, which is the direction that matters here.

And the bag says even *that* is not what happened: the gaps between a hung
goal's last record and the next goal's first record are **32.5 s, 50.0 s and
53.9 s** — all far short of 270.9 s (and short of 157.5 s too, so the
correction does not disturb the inference). So the four goals were released by
something faster than the ceiling, and the only such path is a cancel
(`goal_handle.is_cancel_requested and not session.cycle_live` at the top of the
loop, which the deadlock state satisfies because `cycle_live` is False). This
does not change any of the five fixes, but it changes what F3 is FOR: it is not
adding a bound where there was none, it is replacing a **270.9 s** backstop that
blames the clock with a 7.8 s verdict that names the fault — and it removes the
dependence on the operator noticing and cancelling.

## Discussion

### One mechanism, three anomalies — and why that is worth stating

The operator reported three things and they looked like three problems: some
cycles abort, some goals stop responding, and then the machine refuses
everything. Treating them separately would have produced three fixes, at least
two of them wrong. The abort is D1. The hang is D2, *caused by* D1 — it needs a
commit-time abandonment to reach it, and the only thing that abandons a staged
slot at commit time is a commit-gate refusal. The `REJECTED_BUSY` is not a fault
at all: it is the one-ball-op claim doing exactly its job, against a goal that
never finished.

The tell that they were one thing was the bag's *shape* rather than its counts:
every one of the four aborted goals ends at the abort, and no goal that did not
abort ends early. That is a functional dependence, not a correlation.

### Why slip-not-abort is the class fix, and not just this bug's fix

The narrow fix is F1 alone — break the deadlock and let the abort stand. It is
one line and it would have kept the sitting alive. It is also the wrong altitude
for three reasons:

1. **The abort is not a fault report.** A late iteration on a healthy,
   idle-most-of-the-time loop is a cadence fact. `ABORTED_CANT_MAKE_RELEASE`
   routes the operator to the throw budget and the delay floor — the R5
   mis-routing the plan already named for `hand_parked` — when the actual
   subject is the Jetson's scheduler. The sitting is the proof: the operator
   varied `h` across three values chasing it, because the verdict named a
   quantity that has nothing to do with it.
2. **The doctrine already existed and this rung was the exception.**
   § 9.2's "slip rather than refuse" governs rungs 3 (`hand_parked`) and 4
   (`ball_seated`) for reasons that apply verbatim here: nothing is armed, no
   announcement has gone out, and the condition resolves by waiting. Rung 6 was
   an abort only because its docstring believed it was unreachable.
3. **The slip machinery already does the right thing.** `_slip` re-arms
   `_t_release = now + commit_budget_for_cycle_s`, so the next tick gets a
   *full fresh* budget, and the node's absolute tick grid
   (`_pace_to_next_tick`) makes the iteration after an overrun SHORT by exactly
   the overrun. The retry is the likely case, not the hopeful one. The sitting's
   census is the number behind that claim: the pre-dispatch **mean** period per
   cycle was **0.0389–0.0412 s on the cycles with ≥ 5 censused pre-dispatch
   iterations** (0.0370–0.0436 s across all 25 censused cycles of the bag) —
   the population is named because both extremes of the wider range are
   4-iteration samples, and a "mean period" over four iterations is a thin one.
   Either way the grid holds the mean at the set-point and individual
   iterations straddle it, with roughly half over on any given cycle.

So the shortfall goes back into `_slip` with its own reason, sharing the
existing bound (`catch_confirm_window_s` from the ORIGINAL `_commit_at_sched`).
Sharing matters: one bound means a cycle cannot launder an unbounded wait by
alternating its reasons across sources.

When the bound IS exhausted, the cycle terminalises
`ABORTED_CANT_MAKE_RELEASE` — same name, same `ABORTED_` family. Keeping the
string was deliberate over introducing `REJECTED_TICK_OVERRUN` or similar: the
runbooks, the record corpus and the cadence ladder key on it, and the terminal
it names is now *honest* rather than premature. A machine that slipped the whole
0.56 s window and still could not make the release genuinely cannot make the
release.

### The walked T-P5 beat-monotonicity argument

The slip is the only writer of `_t_release` after `start()`, and F2 routes a new
source into it — so T-P5 (`t_release(k+1) > t_release(k)` under every slip
sequence) had to be re-walked before the code was written. A slip that ran
backwards would schedule a stroke into a live catch.

1. **Within a cycle, `_t_release` has exactly two writers.** `start()` sets it
   once from `release_at_perf` (or the derived default); `_slip` sets it to
   `now + commit_budget_for_cycle_s`. F2 added no third writer — it re-routes
   into the existing one.
2. **Every slip moves it FORWARD.** The gate only runs at `now >= _commit_at`,
   and `_commit_at` was last set either at `start` (to
   `_t_release − commit_budget`) or by the previous slip (to that slip's `now`).
   In both cases `now + budget >= _commit_at + budget = _t_release`, with
   equality only in a degenerate zero-length tick. Ticks are strictly
   increasing, so a cycle's own successive releases are strictly increasing.
3. **Across cycles the ordering is fixed BEFORE any slip.** Cycle `k+1` stages
   only after cycle `k` has DISPATCHED, and its release is
   `t_release(k) + flight(k) + dwell` — strictly greater (flight > 0, the dwell
   floor is positive). By (2) it can only grow from there, while cycle `k`'s
   release froze the instant it dispatched (a committed cycle never re-enters
   the gate). So `t_release(k+1) > t_release(k)` survives every slip sequence.
4. **A cycle that never dispatches contributes no release at all**, so
   converting an abort into a slip cannot reorder anything: the cycle either
   ends in a dispatch obeying (2)/(3), or in no dispatch.

**Conclusion: the new slip source can only DELAY a release, never advance one.**
It cannot schedule a stroke into a live catch. What it *can* do is stretch the
cadence, and that is measured (`slip_s`, and the new `commit_slips`) rather than
absorbed silently.

The argument is now mechanical: `test_the_schedule_is_monotone_with_late_ticks_in_the_slip_sequence`
asserts it over the extended strategy, and the strategy change is itself the
finding — `_JITTER` drew ONE spacing and applied it to every tick of a run, so
a run was uniformly fast or uniformly slow and **the interleaving that actually
bit (one late iteration among nominal ones, on the commit crossing) was never
generated**. `_TICK_GAPS` draws per-tick.

### Why `NODE_LOOP_PERIOD_S` is NOT re-cut, and why the census's 0.060 ask was an artefact

`tools/probes/toss_loop_census.py`'s first read (2026-08-27, Phase B0) printed a
SIZING line asking for **0.060 s**, and the obvious response to D1 is to take
it. The owner deferred that past the next sitting, and the deferral is right for
a reason the fix itself creates:

**the 0.060 ask was computed under ABORT semantics.** Under abort, the constant
has to bound the WORST iteration, because one overshoot kills the cycle — so
sizing it means taking a tail percentile of a measured distribution and ceiling
it, which is exactly what the probe did (p50 0.0447, p90 0.0519, max 0.0626 on
the pre-B5 corpus).

Under SLIP semantics the constant stops being a bound and becomes a **set-point**
— and B5's absolute tick grid already holds the machine to it: the sitting's
per-cycle mean pre-dispatch period is **0.0389–0.0412 s on the cycles with ≥ 5
censused pre-dispatch iterations** (0.0370–0.0436 s across all 25 censused
cycles, whose two extremes are both 4-iteration samples) against a 0.040
nominal. Jitter around a set-point is no longer a terminal; it is measured slip,
reported per cycle in `commit_slip_s` and now `commit_slips` — **both of which
are record fields** (`toss_record.py`, `'pipeline'` group, additive so no SCHEMA
bump), because a distribution the deferred decision is waiting on has to be in
the corpus rather than only in a log line. Re-cutting to 0.060 under
slip semantics would (a) pay 20 ms of real lead per cycle at every cadence rung
for a hazard that no longer exists, and (b) throw away the instrument: a loop
that IS degrading would be absorbed by the larger constant instead of showing up
as rising slip.

The right sequence is therefore: ship the slip, fly a sitting, read
`commit_slip_s` / `commit_slips` distributions off the corpus, and only then ask
whether the set-point should move. If the answer turns out to be yes, the number
will be derived from what the loop does at the set-point rather than from what
it did when a single overshoot was fatal.

### F3's bound, and why it is not a second session ceiling

The watchdog needed a bound that is generous enough to be unreachable in healthy
operation and tight enough to be useful. Two existing constants give both ends:

```
_SESSION_STALL_S = DEFAULT_SESSION_MISS_CLEANUP_S   # 2.80 s
                 + _SEQUENCE_CEILING_MARGIN_S       # 5.00 s
                 = 7.80 s
```

`DEFAULT_SESSION_MISS_CLEANUP_S` is the longest quiescent wait **the session FSM
itself imposes** with both slots empty — it is the floor `note_cycle_result`
applies to `_next_cycle_at` after a continued MISS, and the same constant fronts
the `ABORTED_NO_RELEASE` retry floor and the reload interlude's rung 4, so
covering it covers every wait the FSM schedules for itself.
`_SEQUENCE_CEILING_MARGIN_S` is the pad every other ceiling in the node already
adds so it cannot land inside a legitimate window.

⚠ **It is NOT a bound on every legitimate between-cycle wait, and the first
draft of this section said it was.** The *operator* chooses the dwell, and the
quiescent part of it — `dwell − throw_delay` on the serial ladder, since the
cycle itself spends the delay before its release — can exceed 2.80 s freely
(9.00 s at a 14 s dwell against a 5 s delay). So the arithmetic above picks a
number of the right ORDER; what actually makes the bound unreachable in healthy
operation is the **next paragraph** — `_toss_session_progressing`'s clause 4,
the progress clock re-anchoring while `now < next_cycle_at`. (The constant's own
comment in `reload_coordinator_node.py` carries the same correction, where it
numbers that clause "reason 1".) Getting this backwards would invite someone to
"fix" a false firing by growing the constant, which is the wrong lever.

So the bound is only the second line of defence. The first is that the progress
clock does not run at all while the session is legitimately waiting:
`_toss_session_progressing` re-anchors on a terminal, on either slot being
occupied, on any emitted action, and — the important one — on
`now < session.next_cycle_at`. That last clause is what lets a dwell be
arbitrarily long without the watchdog knowing anything about cadence arithmetic:
the session names the instant it intends to start the next cycle, and waiting
for an instant it named is not a stall.

`test_the_watchdog_never_fires_on_a_healthy_chained_session` is what makes that
mechanical, and **the audit found its guard was on the wrong quantity**. It
asserted `DWELL > _SESSION_STALL_S` on the module fixture's 8.0 s dwell, but the
progress clock would only ever run over the QUIESCENT part of that wait —
`dwell − throw_delay` = 3.0 s at the fixture's 5.0 s delay, comfortably inside
the 7.8 s bound. Measured by instrumenting `_toss_session_progressing` and
tracking `now − t_progress` across the whole session (2026-08-28), the largest
no-progress gap that session actually reaches is **0.0000 s** at BOTH the 8.0 s
and the 14.0 s dwell — clause 4 re-anchors on every tick of the wait, so the
clock never accumulates at all. The test was green for a reason that had nothing
to do with clause 4, and it still passed with clause 4 deleted. It now runs a
LOCAL dwell of
**14.0 s against a 5.0 s delay — a 9.0 s quiescent wait against the 7.8 s
bound** — and the guard reads `dwell − delay > _SESSION_STALL_S`. Verified both
ways on 2026-08-28: it passes as shipped, and stubbing clause 4 out makes it
fail `ABORTED_STALLED != COMPLETED`. (The session ceiling at those values is
~137 s, so there is ample room for the three cycles.)

The reload interlude gets two independent protections and the test would catch
the loss of either: it runs inside a `continue` that never reaches the check,
AND the loop re-anchors on the instant the interlude actually returned.

### F6 — why the label went to the log and not to the wire

The staged slot's `COMMITTING` feedback is genuinely ambiguous: the same string
is what a serially-run first cycle reports at its own arm point, so an operator
watching feedback cannot tell which slot is speaking. The tempting fix is a
distinct string (`COMMITTING(staged)`).

Grepped first, as the rule requires. `TossContinuous.action`'s phase vocabulary
is a published contract that documents these by value;
`tests/ros/test_toss_continuous_node.py::test_the_two_additive_phases_actually_reach_the_wire`
asserts the exact literal `'COMMITTING'` appears in the published stream; and
`tests/ros/test_toss_session.py` pins `SESSION_PHASE_COMMITTING == 'COMMITTING'`.
Something pins the exact string, so the additive-string option is out — breaking
a wire consumer for a labelling convenience is a bad trade.

So the disambiguation went to an INFO log line, emitted on the TRANSITION into
`COMMITTING` rather than per tick (a slipping commit re-enters that branch every
iteration, and one line per slip would bury the arm point it is announcing).
Minimal, and it costs no consumer anything.

## Fix

**F1 — break the deadlock (`toss_session.py`), belt and braces.**

* `step()`'s gate becomes
  `if self._cycle_live or (self._stage_declined and self._committed_live):` —
  the flag can only ever gate a slot that something else is going to free;
* `note_stage_abandoned` sets `self._stage_declined = bool(self._committed_live)`
  — the wait is not raised when there is nothing to wait for;
* the docstring's "rebuilt on the SERIAL path once the committed cycle
  terminalises" is replaced by the actual two-case behaviour (drain case
  unchanged; commit-time case rebuilds on the very next `step()`), and the
  `_stage_declined` field comment states the invariant.

**F2 — a late commit tick SLIPS (`toss_sequencer.py`).**

* rung 6 routes through `_slip(..., abort_code='CANT_MAKE_RELEASE')`;
* `_slip` gains `abort_code`, shares the existing `catch_confirm_window_s`
  bound counted from `_commit_at_sched`, and counts re-arms in `_commit_slips`
  (exposed as the `commit_slips` property);
* **`commit_slips` reaches the RECORD**, next to `commit_slip_s`: a `Field` in
  `toss_record.py`'s `'pipeline'` group (additive, so no SCHEMA bump — five
  pipeline fields now, all null on a serial cycle) and a line in
  `_toss_record_fields`. The pair is the point: `commit_slip_s` says HOW LATE
  and `commit_slips` says HOW MANY ITERATIONS, and one unlucky tick on a healthy
  loop is a different finding from a loop chronically over period even when the
  two produce the same lateness. That distinction is the stated input to the
  deferred `NODE_LOOP_PERIOD_S` decision, so it has to be in the corpus;
* the two refuted docstrings are corrected in place — `min_stage_lead_for_release_s`'s
  "structurally unreachable" and `_slip`'s "holds by construction" — each
  stating what actually holds now rather than being quietly deleted.

**F3 — session no-progress watchdog (`reload_coordinator_node.py`).**
`_SESSION_STALL_S` (derived above) plus `_toss_session_progressing`, a
`@staticmethod` so the policy is testable without a node. On a stall the goal
terminalises `ABORTED_STALLED` through `_finish_session`, so the execute
callback's `finally` drains the pipeline, lowers the session arming and releases
`_goal_claimed`. Documented additively in `TossContinuous.action` (comment only
— no field added, removed or retyped, so no `jugglebot_interfaces` rebuild).

**F5 — instrument the abort (`toss_sequencer._commit_forensics`).** The
slip-bound-exhausted terminal carries, in the outcome string itself: the
shortfall `_t_release − now` against the dispatch budget it was measured
against, the lateness `now − _commit_at_sched`, the number of slips, and the
last iteration's length against the nominal period. It rides the outcome rather
than a second log line because the outcome is what every consumer already sees —
`_log_toss_outcome`, the staged-discard WARN, and the record's `outcome` /
`staged_discarded_reason`.

**F6 — label the staged slot's COMMITTING feedback**: an INFO log line on the
transition, plus the comment explaining why not a new wire string.

**Also landed with this commit**: the operator's flag flip
(`jugglebot_operational.toss_pipeline_enabled: true`) and its regenerated
artifacts, as the new committed default.

## Verification

Nightly at session start was **GREEN** (`cat temp/reports/nightly/status`, read
2026-08-28: `GREEN 6567/6574 passed, 0 failed, 0 errored, 3 xfailed, 4 skipped
2026-08-28T04:04:50+10:00`).

**Non-vacuity, checked by reverting the production files and re-running** (the
house rule that a regression test must fail against the code it regresses):

* `python -m pytest tests/ros/test_toss_session.py -q -p no:randomly -k "commit_time_stage_abandonment or wedged_session_shape"`
  (run 2026-08-28, `toss_session.py` at HEAD): **2 failed** — both new tests.
  With the fix: **135 passed**;
* `python -m pytest tests/ros/test_toss_sequencer.py -q -p no:randomly -k "late_tick or chronically"`
  (run 2026-08-28, `toss_sequencer.py` at HEAD): **2 failed, 1 passed**. With
  the fix: all pass;
* `python -m pytest tests/ros/test_toss_pipeline_properties.py -q -p no:randomly`
  (run 2026-08-28, `toss_sequencer.py` at HEAD): **1 failed** —
  `test_the_late_tick_model_actually_generates_late_ticks`, on
  `['CAUGHT', 'CAUGHT', 'ABORTED_CANT_MAKE_RELEASE'] != ['CAUGHT'] * 3`, which
  is the sitting's failure reproduced in the model.

**The extended property does NOT reach the state it quantifies over, and the
reachable case is now pinned by a COMMITTED test rather than a `/tmp` probe.**
A one-off probe over 400 runs of the model with per-tick gaps drawn uniformly
from 0.25× to 2.5× the nominal period did observe 48
`ABORTED_CANT_MAKE_RELEASE` terminals, every one with
`slip_s > CATCH_CONFIRM_WINDOW_S` — but that is not the driver the property
runs under. Under Hypothesis, `test_a_late_iteration_never_aborts_a_commit_inside_the_slip_bound`
gets **0 of 1000 examples past its `CANT_MAKE_RELEASE` filter at ci-deep**
(measured 2026-08-28): `_HEALTHY_STREAM` caps a run at 60 ticks and `_run`
CYCLES the gap list, so exhausting a 0.56 s bound (~14 consecutive late
iterations) inside a run that also has to reach cycle 2's commit is a corner the
generator effectively never draws. The property is green over an empty set.

Widening the strategy does not fix it, because reachability is **non-monotonic
in the gap size** — at a constant per-tick spacing the model gives three CAUGHT
at 1.2×, the bound-exhausted abort at 1.5×, the abort one cycle earlier at 2.0×,
and three CAUGHT again at 2.5×. So the reachable case is pinned explicitly by
`tests/ros/test_toss_pipeline_properties.py::test_the_model_reaches_the_slip_bound_exhausted_abort`
(constant 1.5× spacing, 300 ticks, 3 cycles), which asserts the terminal set is
NON-EMPTY and that every member of it slipped past the window. Run 2026-08-28,
`python -m pytest tests/ros/test_toss_pipeline_properties.py -q -p no:randomly`:
**13/13 pass in 3.61 s**. The universal stays as it is — it is what forbids the
regression at every OTHER spacing — with the vacuity warning in its own
docstring.

**The probe reconciliation reproduces every pinned number** (follow-up 3, run
2026-08-28, `python tools/probes/cadence_rung_check.py --solve --frontier
--pipeline --grid`): **0 violations** over the pipelined `(T, dwell, ilc)` grid
at the modelled floors, **196 violations** on the `budget_loop_s=0.0`
counter-check (all of them `ABORTED_CANT_MAKE_RELEASE`), **all six § 6.2 rungs
FLY**, and the § 1.4 predicted slips are unchanged at **P4 0.0446 s / P5
0.0646 s**. The serial grid is 0 violations, untouched. The one number that
MOVED is the printed achieved dwell of the two rungs that actually slip (P4
0.53 → 0.49 s, P5 0.51 → 0.47 s), and it moved toward the truth: the release a
slipped commit flies on is the one the LAST slip armed, one tick before the
committing tick, not one re-armed at the commit itself.

**Scoped suites** (run 2026-08-28, venv `~/Desktop/PDJ_venv`, after the fix
wave's audit corrections):

| command | result |
| --- | --- |
| `python -m pytest tests/ros/test_toss_sequencer.py tests/ros/test_toss_session.py tests/ros/test_toss_pipeline_properties.py -q -p no:randomly` | **300/300 pass in 5.91 s** |
| `python -m pytest tests/ros/test_toss_continuous_node.py -q -p no:randomly` | **158/158 pass in 10.78 s** |
| `python -m pytest tests/motion/test_cadence_rung_check.py tests/motion/test_toss_record.py -q` | **84/84 pass in 1.61 s** |
| `python -m pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py tests/sim/test_plans_index.py -q` | **106/106 pass in 0.66 s** |
| `python -m pytest tests/ros/ -q -p no:randomly` | **2543 passed, 1 skipped in 330.56 s** |

`python tools/probes/toss_loop_census.py --csv` (run 2026-08-28) reads the
corpus clean with the histogram now bucketed on the bare outcome code, so F5's
forensics-bearing outcome strings cannot fragment it into one bucket per event;
the full string is still written verbatim to the per-row CSV, which is where a
post-mortem reads it.

⚠ **`./run_tests.sh` was NOT run** — this work was scoped to the touched files
plus a `tests/ros/` sweep by instruction. `controller/` and `sim/` are untouched,
so the `nightly` tier this commit's paths sit under is `tests/ros/`,
`tests/motion/` and the three `tests/sim/` logbook/plans guards, all swept above;
the gate still owes a run before the commit is written.

`test_the_census_never_feeds_a_budget` passes untouched — F5's last-iteration
length is measured by the FSM's own clock between `step()` calls, not plumbed in
from `LoopPeriodCensus`, precisely so the census stays strictly downstream of
every decision.

## Outcome

The deadlock class is closed at two altitudes: F1 removes the state, F3 makes
any future member of the class cost one goal and name itself in 7.8 s instead of
holding the machine for **270.9 s** (this sitting's real session ceiling, the
`reload_budget_s` term included) and blaming the clock. The abort class is
closed at the right altitude too: a late iteration is now a cadence fact the
machine absorbs and measures, and the terminal that survives is one a machine
genuinely earned.

The pipeline flag ships **on**.

**Deploy: `cd ros_ws && colcon build --packages-select jugglebot` before the
next sitting.** The launch runs the install space, so the flag flip and all five
fixes are inert until it is rebuilt (`jugglebot_interfaces` is NOT affected —
the `.action` change is comment-only).

## Open questions / follow-ups

1. **⚠ THE GUI HAS NO CANCEL — flagged for the owner.** The deadlock state is
   cancellable (`goal_handle.is_cancel_requested and not session.cycle_live` is
   satisfied, so a cancel is honoured immediately and commands nothing), and the
   bag's 32–54 s inter-goal gaps say that is how the sitting's four hung goals
   were actually released. But a cancel needs a cancel button. If the operator's
   GUI cannot send one, the only remaining escape from ANY session-level wedge —
   including ones F1 and F3 do not anticipate — is the session ceiling. Worth a
   decision: add cancel to the GUI, or accept the watchdog as the sole escape.
2. **The session-accounting tests have no pipelined twin.**
   `tests/ros/test_toss_continuous_node.py`'s `_stub_cycles` harness scripts
   `_run_toss_cycle` — the SERIAL blocking wrapper, which the pipelined loop
   never calls — so stop_on_miss, cancel-inside-a-cycle, the retry ladder, the
   reload interlude and the S6/S7 teardown assertions are all serial-only. That
   was invisible while the flag shipped false; the flip made 28 of them go red
   without a line of production code changing, and they are now pinned serial by
   an autouse fixture (no assertion changed). **The machine ships pipelined, so
   this gap is live rather than latent.** A pipelined twin of the accounting
   suite is the follow-up.

   **⚠ ONE instance of it was load-bearing enough to close now: F3's own
   non-firing guarantee.** Both
   `test_the_watchdog_never_fires_on_a_healthy_chained_session` and
   `test_the_watchdog_never_fires_across_a_reload_interlude` FAIL when the
   autouse fixture is flipped to True (verified 2026-08-28 by flipping it), so
   the watchdog that ships ON was pinned against FALSE FIRING only on the serial
   ladder — the machine the operator no longer runs, and false firing is the
   failure mode that costs a good sitting. Closed by
   `test_the_watchdog_never_fires_on_a_healthy_PIPELINED_session`, a full
   three-cycle session through `_run_pipelined` (the real FSMs and the real
   `_tick_toss_pipeline`) asserting COMPLETED with no `STALLED` in the outcome
   or in any published feedback. The rest of the accounting suite is still
   serial-only.
3. **✅ DONE in this wave — `tools/probes/cadence_rung_check.py` is reconciled.**
   Its `commit_tick` modelled the RETIRED abort in three ways: the release-window
   inequality was evaluated ONCE after the wait loop instead of per tick, an
   over-bound wait returned a `'SLIP_UNBOUNDED'` verdict no FSM ever mints, and
   the bound was counted from the LANDING rather than from `_commit_at_sched`.
   A lateness sweep against that model would have over-reported the abort by
   ~14×. All three are now the shipped rules, and past the bound the model
   terminalises by the name of the gate that held it
   (`ABORTED_CANT_MAKE_RELEASE` / `REJECTED_HAND_NOT_PARKED` / `REJECTED_NO_BALL`;
   `REJECTED_BALL_UNKNOWN` is in the FSM's vocabulary but unreachable in a model
   whose cup has a definite seat instant).

   **The re-arm ORDERING is the finding inside the finding.** `_slip` sets
   `_t_release = now + budget` AT the slip instant and the tick advances after,
   so the next tick measures a lead of `budget − loop` = `dispatch + 1 µs` — the
   razor edge that IS D1. Modelling it the other way round (advance, then
   re-arm) hands the machine a full fresh budget every tick and looks harmless:
   the default grid stays at 0 violations and every rung still FLIES. What it
   silently destroys is the regression test — `pipelined_grid_violations(budget_loop_s=0.0)`
   collapses from **196 violations to 0**, i.e.
   `test_the_pipelined_grid_reds_when_the_commit_budget_forgets_the_polled_tick`
   goes red for the opposite reason it was written. Both orderings were run;
   only the FSM's keeps every pinned number.
4. **Re-read `commit_slip_s` and `commit_slips` after the next sitting** — that
   is the input the deferred `NODE_LOOP_PERIOD_S` decision is waiting on.
