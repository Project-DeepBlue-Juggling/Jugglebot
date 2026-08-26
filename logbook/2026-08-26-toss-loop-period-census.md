---
title: The toss tick loop starts measuring itself, before anything tries to fix it
type: feature
date: 2026-08-26
status: resolved
related_plan: toss-selftuning.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_record.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_record_publisher.py
  - tests/motion/test_toss_record.py
subsystem:
  - ros
tags:
  - cadence
  - measurement
  - performance
  - instrument
---

# The toss tick loop starts measuring itself, before anything tries to fix it

## Summary

`_run_toss_cycle`'s loop is `work; time.sleep(_TICK_S)`, so an iteration costs the
sleep **plus** the tick's own work. D3 (same day,
[[2026-08-26-possession-verdicts-become-sensor-only]]) established that the work
term is real — 7–18 ms on top of a 20 ms sleep — and re-denominated every
pre-dispatch budget in a measured `NODE_LOOP_PERIOD_S = 0.040` instead of the
sleep. That fixed the arithmetic. It did **not** give the machine any way to tell
us when the constant stops being true.

This lands that: `LoopPeriodCensus`, a per-cycle wall-clock census of the loop's
own iteration, reported into the toss record (ten `timing` fields) and onto the
outcome line (a `tick_max=` suffix and a separate WARN, but only on an overrun).

**No behaviour changes.** Nothing reads it back; a test enforces that.

## Discussion

### Why measure first, when two plausible fixes were already on the table

The owner proposed two, and both are good:

1. **Deadline-compensated sleep** — `sleep = max(0, TARGET − elapsed)`, giving a
   constant period instead of a constant sleep. It converts the period from an
   *output* (whatever `20 + work` happens to be) into an *input*, which matters
   because every budget in the toss stack is `n × period`. If the period is an
   accident, so is every budget.
2. **Event-driven wake** — a `threading.Event` set by executor callbacks, waited
   on with a timeout equal to the target period. Prompt when evidence lands, idle
   when it hasn't. This is the owner's standing "RTOS principles / async-cache"
   call applied to this loop.

Neither was implemented, and the reason is not that they're wrong. It is that
**`NODE_LOOP_PERIOD_S` rests on 28 cycle starts from a single bag**, inferred
from log-line spacing rather than measured directly, on one Jetson, at one
configuration, with the ILC in one state. Compensation requires choosing TARGET,
and TARGET has to sit above the worst work the loop ever does. Choosing it from
28 samples of one sitting would replace a measured-but-thin constant with a
*guessed* parameter that now also governs the loop's actual behaviour rather than
just its accounting. The failure mode is worse than today's: a TARGET set too low
means the compensation silently degenerates to "no sleep at all" on exactly the
loaded ticks that need the yield most.

So the order is: measure, then choose. Both fixes stay on the table and the census
is what will size them.

### What the census had to be shaped like, and why

**The boundary is the loop's own `now`.** Not a fresh `perf_counter()`. `now` is
the clock every FSM guard compares against, so the interval between successive
`now` values *is* the granularity at which time appears to move to the
release-window guard. Any other pair of stamps measures something adjacent but
different, and the whole point is to measure the quantity the budgets are
denominated in.

**The statistics are split pre-/post-dispatch.** A cycle spends most of its ticks
idling out a 0.5 s flight on `ACTION_NONE`. `pre_dispatch_budget_s` charges only
CHECKING → POSITIONING → PREPARING, and those are the ticks that run a blocking
`arm_catch` raise-and-confirm and an announcement publish. Pooling them would
report a comfortable mean for a ladder that is not comfortable — the cheap
majority would hide the expensive minority, which is precisely the quantity that
needs bounding. This is the single decision that makes the census honest rather
than merely present.

**The sleep is measured, not assumed.** Each iteration decomposes as
`obs | body | sleep`, with `sleep` timed rather than taken to be `NODE_TICK_S`.
On a loaded Jetson `time.sleep(0.020)` can return at 0.026, and folding that
overshoot into `body` would charge six milliseconds to code that did not run
slowly. The three suspected costs — blocking service calls, the per-tick
observation rebuild, executor/GIL contention — are only separable if scheduler
overshoot gets its own column. One extra `perf_counter()` per tick buys that.

**Rows lag by one call.** A period and its sleep are only knowable once the next
iteration starts, so `note_iteration_start` commits the previous row. The terminal
iteration returns from the middle of the loop and never reaches its sleep, so it
is never committed. That is deliberate, not an off-by-one: charging it a period
would report time the loop did not spend.

### The tradeoff accepted: the test suite cannot validate this

The mocked-ROS suite has no executor, no publishers and no GIL contention. It is
structurally blind to the thing the census exists to measure, so the unit tests
verify **arithmetic only** and say so in a comment block. The only check that can
answer "is the loop fast" is a sitting's corpus.

What the tests *can* protect is that the numbers mean what their field docs say —
an off-by-one in the lag, or a phase attributed to the wrong bucket, would make a
whole sitting's census quietly wrong while every test stayed green.

The one that earns its keep is
`test_the_cycle_loop_feeds_the_census_and_drops_only_the_terminal`. Every other
census test injects a pre-filled census or exercises the class standalone — all
of them stay green if `_run_toss_cycle` stops calling it, and the corpus would
then report a null timing block that reads exactly like "no cycle ran".

### Why the census may never feed a budget, and why that is pinned by a test

The tempting next step is to have `NODE_LOOP_PERIOD_S` re-derive itself from the
last sitting. It must not. **A bound that tracks its own degradation hides the
degradation** — the floors would rise silently to accommodate a loop that is
getting slower, and the operator would learn about it as a shrinking frontier with
no named cause. That is the same failure the D3 correction was: a floor short by a
factor of two, invisible until two cycles aborted `ABORTED_CANT_MAKE_RELEASE` with
the latch raised and the announcement out.

So the constant stays a hand-set, reviewed number, and `test_the_census_never_
feeds_a_budget` inspects the *compiled identifier set* of `pre_dispatch_budget_s`
and `min_throw_delay_for_release_s` and fails if either ever reaches for census
state. (Compiled identifiers, not source text: "census" is an unrelated word in
this codebase — the B1/B6 census notes — and a source grep matches those
docstrings.)

**This generalises.** Any instrument that measures a bound is one refactor away
from becoming that bound's source. The pattern to reuse is: the instrument answers
*"should a human move it?"* and stops, and a test makes the separation structural
rather than a matter of discipline.

### Field naming looks ahead to compensation

Both `loop_period_max_pre_s` and `loop_work_max_pre_s` are recorded, though they
differ only by the sleep. Today the period is the number `NODE_LOOP_PERIOD_S` must
bound. If deadline compensation ever lands, the period becomes a set parameter and
**work** becomes the interesting quantity — how much headroom is left, i.e. how
far TARGET could come down. Recording both now means that decision needs no schema
change and arrives with history already in the corpus.

Worth naming the hazard compensation would introduce: under a fixed period, work
growing from 10 ms to 30 ms produces **no visible symptom** — the period stays at
TARGET and everything looks fine while all the headroom burns. `loop_work_max_pre_s`
is what would close that gap, which is an argument for the instrument existing
*before* the fix rather than after it.

## Implementation

* `toss_sequencer.py` — `LoopPeriodCensus`, `PRE_DISPATCH_PHASES`,
  `CENSUS_FIELD_NAMES`, placed directly beneath `NODE_LOOP_PERIOD_S` so an edit to
  either sees the other. `__slots__`, no container touched per tick, no lock (the
  census is written and read on the cycle thread alone — locking it would put
  contention into the loop it measures).
* `reload_coordinator_node.py` — `note_iteration_start` at the top of
  `_run_toss_cycle`, `note_iteration_end` immediately before the sleep;
  `_step_toss_sequence` stamps `_toss_obs_build_s` rather than changing its return
  type, so it stays the isolated-testable decision function its docstring promises.
  `_log_toss_outcome` appends `tick_max=` on an overrun, raises a separate WARN,
  and then **consumes** the census (clears it) so a later `REJECTED_BAD_GOAL`
  terminal — which runs no loop at all — cannot inherit another cycle's timings.
* `toss_record.py` — a ten-field `timing` block, origin `D`, all nullable. No
  `SCHEMA` bump: the schema's own rule is that purely additive fields do not bump.
  All-`None` when nothing was censused, which reads as "not measured" and is a
  different fact from a measured zero.

The two field lists are pinned equal by `tests/motion/test_toss_record.py` rather
than single-sourced by an import: `toss_record` deliberately takes exactly ONE
jugglebot import (`ball_possession`, pure and leaf) so a corpus reader never drags
in the FSM. Same trade the `_TICK_S` / `NODE_TICK_S` mirror already makes.

## Verification

Full gate, run **2026-08-26**: `./run_tests.sh` — **6006 passed, 4 skipped in
275 s** (parallel 265 s, serial phase empty).

Full tier, run **2026-08-26**: `./run_tests.sh --full` — parallel **6429 passed,
4 skipped, 3 xfailed in 531.07 s**; serial **9 passed in 41.42 s**; total 579 s,
`RESULT: PASS`. Run because the census sits in the toss cycle's hot path and the
nine `serial`-marked allocation/timing tests are the ones that would notice a loop
that grew per-tick work. They did not.

**15** new tests: 8 on the census arithmetic
(`tests/ros/test_toss_sequencer.py`), 2 on the loop wiring and consumption
(`tests/ros/test_toss_coordinator.py`), 5 on the record and log seams
(`tests/ros/test_toss_record_publisher.py`), plus the extended field pin in
`tests/motion/test_toss_record.py`. (Those three files gained 22 tests in
`f997470` — the other 7 are the possession work sharing the commit; see
§ Provenance anomaly.)

**Not yet verified on hardware.** The census has never run against a real executor;
its first output is the next sitting's corpus. That is the point of it.

## Deploy

`cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash`,
then relaunch. The install space was current as of 19:45 on 2026-08-26 but a
parallel session is active on this branch — rebuild before the sitting rather than
trusting that.

## Outcome

Open questions the next sitting's corpus answers directly:

1. **Is `NODE_LOOP_PERIOD_S = 0.040` still a bound?** `max(loop_period_max_pre_s)`
   across the session. R4/R5/R5-prime clear their floors by 2–9 ms, so this is not
   an academic question.
2. **Which cost dominates?** `loop_obs_max_pre_s` vs `loop_body_max_pre_s` vs
   `loop_sleep_max_pre_s` — the three-way split that decides whether the next move
   is caching the observation rebuild, deferring the blocking PREPARE call, or
   addressing executor contention.
3. **How low could a compensated TARGET go?** `loop_work_max_pre_s` is the answer,
   and the pre-dispatch budget is `4 × period`, so it is charged four times over.

`loop_n_over_pre` should be 0 on every cycle. **Nonzero on a *successful* cycle is
the early warning that did not previously exist** — the previous route to that
finding was noticing an `ABORTED_CANT_MAKE_RELEASE` after the fact.

Still open, deliberately: only `_run_toss_cycle` is instrumented. The reload
loop (in `_execute_reload`) and the session loop (in `_execute_toss_continuous`)
have different bodies and feed different budgets (`SAFE_ABORT_LADDER_S`, `DEFAULT_SESSION_MISS_CLEANUP_S`); the class is
reusable but their numbers must not be pooled under one constant. No probe-side
reader yet either — the first sitting means reading the JSONL directly.

## Provenance anomaly

**The code for this entry landed inside commit `f997470`**, whose message and
`Logbook-Entry:` trailer are the possession-verdict work. A parallel session on
this branch committed the whole working tree — including this instrument,
mid-session — and pushed it, so `git blame` on any census line leads to a commit
whose stated scope is possession verdicts, and
`git log --grep "Logbook-Entry: 2026-08-26-toss-loop-period-census"` does not
reach that code.

Not rewritable: `f997470` is on `origin`. Recorded here instead, and the trailer
lands on the commit carrying this entry so the entry is at least reachable. The
follow-on census work named above (the other two loops, the probe reader) is the
natural place for a properly-scoped trailer.

The general lesson is one this repo already knows and a parallel session was not
applying: **verify the staged set before committing** — `git add -A` on a branch
with concurrent sessions attributes other people's in-progress work to your commit
message.
