---
title: "The slow post-RELOAD resume was never waiting for BallButler — it was a naked throw delay, a double-charged floor, and a go_home to where it was already going"
type: investigation
date: 2026-08-29
status: resolved
phase: "toss-pipelined-preamble — 2026-08-29 second wave, 2 of 2"
related_plan: toss-selftuning.md
sessions:
  - 2026-08-29_15-30-29
  - 2026-08-29_15-40-40
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/ros/test_toss_continuous_node.py
  - tests/hardware/session_anomaly_fixes.md
  - sim/analysis/log_index.json
subsystem:
  - ros
tags:
  - toss
  - reload
  - cadence
  - performance
  - safety
---

# The slow post-RELOAD resume was never waiting for BallButler — it was a naked throw delay, a double-charged floor, and a go_home to where it was already going

## Summary

Operator report: **the resume after a RELOAD interlude is slow**, with the
hypothesis that *JB is waiting for BallButler to settle.*

**VERDICT: FALSE.** Bag-measured across `2026-08-29_15-30-29` and `_15-40-40`
(n = 13 caught interludes): there is **not one BB-keyed wait anywhere on the resume
path.** Every BB dependency in the interlude sits *before* the catch, in the gate
and the countdown; once the ball is in the cup, BallButler is out of the loop.

The ~8 s of dead time decomposes into three things that have nothing to do with
BallButler:

1. **`throw_delay_s` = 5.0, paid NAKEDLY.** The interlude drains the pipeline, so
   the resumed cycle's release lead is spent in the open where steady-state
   pipelining normally hides it.
2. **A 2.80 s teardown floor** — of which **0.56 s was double-charged**:
   `DEFAULT_SESSION_MISS_CLEANUP_S` is anchored on the **scheduled landing**, while
   `_settle_after_reload` starts its clock at the **verdict**, by which time that
   leading `CATCH_CONFIRM_WINDOW_S` term has already been spent in real time.
3. **A ~2.0 s `go_home` profile whose destination WAS the next cycle's throw
   site** — in **7 of 12** resumed cycles `trajectory_node` logged *"POSITIONING
   skipped — platform is already at (0.0, 0.0, 170.0)"*. Measured cost of that
   pair: **2.803–2.833 s of total log silence** between `Reload CAUGHT` and
   `reload interlude CAUGHT`, n = 13.

Two fixes, both owner-approved. **Fix A**: the interlude's CAUGHT terminal now
**stays at the catch pose** — a new `_recenter_stay()` ladder (RECENTER *minus*
`go_home`), selected by a keyword-only `stay_on_caught=True` at `_step_sequence`'s
dispatch. **Fix B**: a verdict-anchored floor,
`RELOAD_STAY_SETTLE_S = SAFE_ABORT_LADDER_S + 2 × NODE_LOOP_PERIOD_S = 0.240 s`,
charged only on that terminal; every other terminal keeps 2.80 s byte-for-byte.

Net: a caught reload hands back **~2.56 s sooner**, with a further **~4.5 s**
available to the operator from lowering `throw_delay_s`.

*Wave 2 of 2026-08-29, sibling of [[2026-08-29-displacement-caps-removed]].
Unflown.*

## Symptoms

From the operator, after the 2026-08-29 sittings: **the session takes a long time
to get going again after a reload.** Stated hypothesis: *JB waits for BB to
settle.*

That hypothesis is the right shape — the interlude genuinely does wait on
BallButler in two places, and this morning's [[2026-08-29-bb-reload-busy-patience]]
made both of those waits *longer* (bounded, 10 s). It is reasonable to suspect the
thing that just changed.

## Diagnosis

### D1 — there is no BB-keyed wait on the resume path (the refutation)

Every BB dependency in the interlude is **pre-catch**:

| BB wait | where | when |
| --- | --- | --- |
| `_wait_for_bb_ready` (the gate's rung 2) | before the recentre | before the reload is even attempted |
| the FSM's `_step_checking` BB-ready patience | `reload_sequencer` CHECKING | before `ACTION_PRIME_HAND` |
| the BB throw countdown | the reload's flight | before the ball arrives |

**After `Reload CAUGHT`, nothing reads `/bb/heartbeat` again.** Confirmed in the
bags and pinned structurally by
`test_a_successful_attempt_never_re_enters_the_gate_or_the_bb_wait`. The operator's
perception was accurate and their attribution was not — which is exactly the case
where a measurement is worth more than an argument.

### D2 — where the time actually goes

Measured, `2026-08-29_15-30-29` + `_15-40-40`:

| term | measured | anchored on |
| --- | --- | --- |
| `Reload CAUGHT` → `reload interlude CAUGHT` (the recentre profile **and** the settle floor, overlapping in real time) | **2.803–2.833 s** of total log silence, n = 13 | the FSM terminal |
| the resumed cycle's release lead, `throw_delay_s` | **5.0 s**, paid in the open | the cycle start |

≈ **7.8–7.9 s** between a caught reload ball and the next throw, of which the
machine's own teardown is a third and the release lead is two thirds.

### D3 — the 0.56 s that was charged twice

`_settle_after_reload` reused `DEFAULT_SESSION_MISS_CLEANUP_S` (2.80 s), whose own
comment states it is *"measured from the cycle's SCHEDULED landing"*:

```
  CATCH_CONFIRM_WINDOW_S   0.56   the settle window before the verdict is minted
+ SAFE_ABORT_LADDER_S      0.16   the ladder's dispatch cost, verdict -> go_home
+ GO_HOME_DURATION_S       2.00   the recentre profile
+ 2 x NODE_LOOP_PERIOD_S   0.08   observe-the-terminal + start-the-next-cycle
  = 2.80 s
```

But `_settle_after_reload` is called from the interlude's `finally`, i.e. **at the
terminal** — the possession verdict that minted CAUGHT. That leading
`CATCH_CONFIRM_WINDOW_S` term is *the window that elapses before a verdict exists*,
and by this call site it has already been spent in wall-clock time: the same corpus
puts the terminal **+0.09…+0.22 s** past the cup's `[SENSOR_ARRIVED]` edge (n = 13).

**A floor may only charge for work that is still AHEAD of its clock.** Charging
0.56 s of already-elapsed settle is a straightforward double-count.

### D4 — the go_home was the next cycle's positioning, run early and serially

This is the finding that made Fix A obvious. In **7 of 12** resumed cycles,
`trajectory_node` logged:

> `POSITIONING skipped — platform is already at (0.0, 0.0, 170.0)`

The recentre's destination **was** the throw site the resumed cycle wanted. So the
session paid a 2.0 s profile *plus* a 2.80 s floor sized to cover that profile, in
front of a throw-delay countdown that would have absorbed the same move **for
free**. The 5 of 12 episodes that genuinely needed a pose move spent
**1.09–1.31 s** on it *inside* the countdown, with `position_busy_wait_s` = 0.000
on every `reload_settle` record.

Two costs for one move, and the expensive copy is the serial one.

## Discussion

### The tradeoff that decided Fix A: the platform now HOLDS a 12°-clamped tilt with a ball in the cup

This is the real price of Fix A, it is not small, and it is **not** the few-degree
analogue of the toss's `_toss_stay` — which is the comparison a future reader will
reach for first, because `_recenter_stay` *is* `_toss_stay` transposed.

A reload catch **saturates the tilt clamp on every single arrival**. Real
BallButler arrivals are 18–40° off vertical and `compute_catch_orientation` clamps
at `MAX_TILT_DEG` = 12°, so the receive tilt is at the ceiling every time — and the
cup shift it produces is therefore an *invariant*, not a distribution:

> `64.78 · sin(12°)` = **13.47 mm** of lateral cup offset (measured across
> 18/25/30/40° arrivals: **13.469 mm, every time**) — the same saturation
> `_RELOAD_CENTERED_TOL_MM` is derived from.

So this path holds the **maximum tilt the platform will ever hold with a ball in
the cup**, not a typical one. It is accepted at **2.6× of margin** against the
35 mm `GEOM_HAND_RADIUS_MM`, and accepted on the *worst* case rather than an
average one, which is the honest way to state it.

⚠ **35 mm is an operator-scored runbook threshold, not a runtime gate.** Nothing
in the code aborts on cup offset. That is why the acceptance comes with a new
runbook row rather than an assertion: **CONT-2.7** in
`tests/hardware/session_anomaly_fixes.md`, the twin of the toss-side **DISP-5.6**,
scoring commanded `rx/ry` (FK of `/leg_setpoint_echo`) plus the ball's mocap
position from `reload interlude CAUGHT` until the resumed cycle's POSITIONING
commands the next move. It carries its own **escape hatch**, and the escape hatch
is one keyword: drop `stay_on_caught=True` at the interlude's `_step_sequence`
dispatch and `_recenter` — byte-identical to what shipped before — comes back. Cost
of reverting: the 2.80 s serial recentre returns.

The hold is also **bounded** in a way `_toss_stay`'s is not: it ends at the next
commanded move, which on this path is at most one floor plus a CHECKING pass away.
`_toss_stay`'s hold is indefinite.

### The cup-flicker risk was discharged by MEASUREMENT, not by argument

Shortening the floor from 2.80 s to 0.24 s moves the resumed cycle's CHECKING
**much closer to the catch**, and CHECKING reads the **raw** cup bit (C-POSSESS-1
§ 3.5 — raw is what fails closed at every dwell). The obvious objection: read too
early and a still-bouncing ball mints `REJECTED_NO_BALL` on a *good* catch, routing
a full cup into a phantom second interlude.

That objection deserved a number, and it got one. `/hand_telemetry` at ~99 Hz over
both bags, **107 catch episodes** (15 reload + 92 toss), **zero invalid samples**:

* the **worst seat bounce anywhere in the corpus ends +0.183 s** past the sensor
  arrival edge, with no post-settle re-bounce;
* the raw bit read **TRUE at terminal + 0.24 s in 107 of 107 episodes**;
* this floor's clock starts at the **terminal**, which the same corpus puts
  +0.095…+0.223 s past that edge — so the earliest possible read is edge **+0.335 s**
  against a **0.183 s** bound.

Pinned by `test_the_stay_floor_clears_the_measured_seat_settle`, which asserts the
margin exceeds a whole loop period rather than merely clearing zero.

The corpus's **other** finding is recorded deliberately, because it is the one a
future reader will otherwise re-derive and mis-attribute to this change: **42
isolated ≤ 60 ms raw dropouts under a demonstrably seated ball** (~1 per 21 s of
held time, P ≈ 0.13 % per single raw sample), **every one absorbed by the debounced
bit**. That class is **time-independent** — identical at 2.80 s and at 0.24 s — so
it is a standing property of the raw-bit gate, not a cost of shortening the floor.
The test docstring keeps the two apart in writing.

### A hazard the fix CREATES, found and structurally pinned: census D6 on the interlude

Fix A makes an existing exclusion **load-bearing** that was previously running with
seconds of slack.

The reload ball keeps its `IN_FLIGHT` status until the tracker mints CAUGHT —
**landing +0.202…+0.442 s** — and the resumed cycle's CHECKING refuses
`REJECTED_TRACK_ACTIVE` on any live track destined for us. At the old 2.80 s floor,
CHECKING ran clear of that window by seconds. **At the 0.24 s stay floor it runs
INSIDE it.**

It is safe, because the announced-ball latch chain already excludes our own ball —
census **D6** (`logbook/2026-08-21-ilc-primary-foldin.md`), a four-link chain
across `_run_one_reload_attempt` → `_build_observations` → `_build_toss_cycle`'s
roll-forward → `_build_toss_observations`' `own_ids` exclusion. **The chain is
pre-existing and this change does not touch it.** What this change adds is the
**pin**: `test_the_reload_balls_track_is_excluded_from_the_resumed_cycles_gate`.

It is a **structural** test (AST inspection of the four methods, including that the
roll *reads* the latch before the clear wipes it) rather than a behavioural one, and
the reason is stated in its docstring: the property is a three-site chain and the
failure it guards is a **broken link**, not a wrong value — a behavioural test would
still pass with the chain intact and the exclusion accidentally right. Before today
this was luck-adjacent; now it is tested.

### Why the seam is a keyword at `_step_sequence`'s dispatch

Three places could have carried this choice, and two are wrong:

* **In the FSM** — wrong, because the choice is not a property of the *sequence*.
  The reload that ran is identical either way and mints the identical `CAUGHT`
  outcome; `reload_sequencer` is a pure module that must not learn which caller is
  driving it.
* **A flag inside `_recenter`** — wrong, because it makes a shipping,
  hardware-validated ladder's behaviour depend on node state set somewhere else.
* **A keyword at the one dispatch both callers share** — right, and
  **keyword-only, defaulted `False`**, so every existing call site keeps its current
  behaviour *without being touched*.

The **byte-identical standalone path** claim is pinned two ways. Directly, by
`test_the_standalone_reload_action_still_recentres` (`_step_sequence` without the
keyword ⇒ `calls == ['go_home']`). And, more persuasively, by what the working tree
does **not** contain: `tests/ros/test_reload_coordinator_node.py`,
`test_reload_sequencer.py` and `test_reload_integration.py` are **all unmodified**.
The standalone-action suites needed no edit at all.

There is in-repo precedent for exactly this doctrine one layer down: `_recenter`
and `_safe_abort` are the two `go_home` dispatchers deliberately **exempt** from
carrying the S7 pipeline drain themselves (`_GO_HOME_DRAINED_BY_WRAPPER`, with the
toss-side wrappers draining for them) — precisely so they stay byte-identical for
the reload. Same reasoning, same seam placement.

### Why the settle-hold term is DELEGATED rather than added

The stay floor carries **no `GO_HOME_DURATION_S` term** (Fix A's consequence — there
is no profile to traverse) and **no settle-hold term**, and the second omission is a
genuine hand-off rather than an oversight.

The platform is left holding the **catch plan**, which carries
`JB_TRAJ_CATCH_SETTLE_HOLD_S` = 0.50 s of quiescent hold from its arrival, and
`go_to_pose` answers `BUSY` for as long as that plan has time remaining. The next
cycle's POSITIONING **already absorbs exactly that** — this morning's
[[2026-08-29-position-busy-repoll]] shipped `TOSS_POSITION_BUSY_PATIENCE_S` = the
settle hold **plus one loop period**, re-polled at 0.10 s.

And our clock starts *later* than the arrival that starts the hold (by the verdict
latency above), so the residual this floor would have to cover is **strictly less
than the hold** the re-poll already covers with a loop period to spare. **The
coverage is by construction, not by arithmetic luck** — which is why
`test_the_busy_repoll_covers_the_stay_floors_settle_residual` pins the *inequality*
(`patience ≥ hold`, and `patience − hold == one loop period`) rather than the two
numbers.

This is the wave's one genuinely pleasing property: fix 2 of the morning wave is
what makes fix B of the afternoon wave safe to leave incomplete.

### The 7-of-12 is the fix's justification and NOT its forecast

Stated loudly in the code, because it is the number most likely to be re-measured
and mis-read: **the skips were CREATED by the very `go_home` this drops.** The
recentre parked the platform at `(0, 0, 170)`, which is where the resumed cycle
wanted to be, so its POSITIONING found itself already there.

Remove the recentre and the coincidence goes with it. The platform is left at the
reload **catch** pose, which is not the resumed cycle's throw site, so **essentially
every resumed cycle now COMMANDS the move**. That is the intended trade: the same
move, paid inside the throw-delay countdown (1.09–1.31 s for the 5 of 12 that
already paid it there, against 2.80 s serial), and a countdown too short to absorb
it is **not silently overrun** — `_build_toss_cycle`'s `lead_floor` grant raises a
cadence cycle's delay to `min_throw_delay_for_release_s(event_vel, True)`, loudly,
once.

**A future re-measure of the skip rate should expect ~0 of N and must not read that
as the fix having failed.** The corollary is that the savings **shrink at tight
cadence**: at a short dwell the countdown has less room to hide the move in, so the
2.56 s handed back is the *headroom*, not a guaranteed net.

### Banked, named rather than fixed

* **The S7 `_recenter` drain-exemption seam.** The exemption's premise — *"these two
  are the reload's ladders too, keep them byte-identical"* — does not hold on the
  interlude path any more, since the interlude now uses a *third* ladder. It is also
  **more** load-bearing than before, because `_recenter_stay` installs no profile at
  all, so S7 has nothing to front for it. Nothing is wrong today; the reasoning that
  justifies the exemption has simply moved out from under it.
* **`_reload_interlude_budget_s` is left over-counting.** It still charges
  `DEFAULT_SESSION_MISS_CLEANUP_S` per attempt even though a CAUGHT attempt now
  charges 0.24 s. That is deliberate and documented in place: an attempt that
  **fails** still pays the full floor, a ceiling has to bound the **worst** case, and
  this ceiling's exit path **ABORTs** a session for doing what it was asked to do.
  *Do not "tighten" it to the stay floor.*
* **No YAML revert knob.** The escape hatch is the one keyword and the runbook row
  names it; a config key for a behaviour whose revert is a one-line edit would be a
  third place for the truth to live.

## Fix

**Fix A — `_recenter_stay()`** (`reload_coordinator_node.py`). RECENTER *minus*
`go_home`. The entire executable body is two lines — `self._arm_catch(False)` then
`self._publish_catch_armed(False)` — i.e. `_recenter` verbatim with the last rung
removed. It issues **no new setpoint of any kind**; the platform holds the catch
pose because the emitter's terminal hold already does that when nobody commands
otherwise.

Ordering is `_recenter`'s verbatim and is load-bearing for `_toss_stay`'s reason:
`arm_catch(False)` **before** the `catch/armed` publish, so `catch_coordinator`'s
prime-retry tick stands down against a latch that is already down. Two rungs where
the toss's is four, because the reload's PREPARE raises neither the prime nor the
pre-tilt hold.

Selected by **keyword-only** `stay_on_caught: bool = False` on `_step_sequence`,
dispatched at the `ACTION_RECENTER` branch. Exactly one caller passes it:
`_run_one_reload_attempt` (the auto-reload interlude). `_execute_reload` (the
standalone `jugglebot/reload` action) does not, and is untouched.

**Fix B — `RELOAD_STAY_SETTLE_S`** (`reload_coordinator_node.py`):

```python
RELOAD_STAY_SETTLE_S = SAFE_ABORT_LADDER_S + 2.0 * TOSS_LOOP_PERIOD_S   # 0.240 s
```

`SAFE_ABORT_LADDER_S` (0.160 s) is **borrowed rather than re-derived**: the stay
ladder is a strict **subset** of the ladder that constant prices — 2 of its 4 rungs,
and the two it drops are the expensive ones (the telemetry-verified hand retract and
the `go_home` dispatch). The sibling measurement puts rungs 1–3 at +0.022…0.025 s
*total*, so 0.160 s over-prices our two by ~6×. That is the safe direction, it costs
0.135 s of a 2.56 s saving, and a second smaller ladder constant would need its own
measurement and its own drift guard.

`_settle_after_reload` gains a keyword-only `stayed: bool = False`:

```python
floor = RELOAD_STAY_SETTLE_S if stayed else DEFAULT_SESSION_MISS_CLEANUP_S
```

`stayed` is computed as `decision.action == ACTION_RECENTER` at the interlude's
terminal, so it is **False for every exit that is not the FSM's own CAUGHT**
(timeout, shutdown and cancel all route through `_safe_on_early_exit`, which *does*
`go_home`). Those keep the full landing-anchored floor **byte-for-byte** — this is
not a shortening of the MISS floor. The floor stays **skipped on a cancel**,
unchanged.

**Tests** (`tests/ros/test_toss_continuous_node.py`) — seven new, plus a
`_stub_terminal` helper that drives `_run_one_reload_attempt` to one terminal
*carrying the ACTION the real FSM would carry there*, because the floor is now keyed
on it:

* `test_the_interlude_caught_terminal_commands_no_go_home` — **the Fix-A
  assertion**: `'go_home' not in calls` and
  `calls == [('arm_catch', False), ('armed', False)]`;
* `test_a_caught_interlude_leaves_the_platform_for_the_resumed_cycle` — the
  acceptance test, and it **counts `go_home` dispatches because that count IS the
  change**: exactly **one** per caught interlude (the gate's rung-2
  `_recentre_for_reload` survives; only the terminal's second `go_home` went);
* `test_the_standalone_reload_action_still_recentres` — the byte-identical pin;
* `test_the_caught_terminal_charges_the_stay_floor_not_the_miss_floor`;
* `test_the_stay_floor_is_derived_from_its_own_sources` — the identity, written in
  **relations only, with no bare numbers**:
  `DEFAULT_SESSION_MISS_CLEANUP_S − RELOAD_STAY_SETTLE_S == CATCH_CONFIRM_WINDOW_S +
  GO_HOME_DURATION_S` (i.e. `2.80 − 0.24 == 0.56 + 2.00`), plus
  `RELOAD_STAY_SETTLE_S < CATCH_CONFIRM_WINDOW_S` and `< GO_HOME_DURATION_S`;
* `test_the_busy_repoll_covers_the_stay_floors_settle_residual` — the delegation
  inequality;
* `test_the_stay_floor_clears_the_measured_seat_settle` — the seat-bounce guard,
  against module constants `MEASURED_SEAT_SETTLE_MAX_S = 0.183` and
  `MEASURED_RELOAD_TERMINAL_LAG_MIN_S = 0.095`;
* `test_a_successful_attempt_never_re_enters_the_gate_or_the_bb_wait` — the
  refutation, as a regression;
* `test_the_reload_balls_track_is_excluded_from_the_resumed_cycles_gate` — the D6
  structural pin.

`test_the_reload_attempt_settles_before_handing_back` was **re-keyed** to the
not-caught half (it now stubs `MISSED` / `ACTION_SAFE_ABORT` and still asserts the
full 2.80 s), and four cancel/defer tests had their `_step_sequence` stubs widened
with `**kw`.

**Runbook**: `tests/hardware/session_anomaly_fixes.md` gains the **CONT-2.7** watch
row described in the Discussion, verdict **REPORT**, PASS at < 35 mm, with the
one-keyword escape hatch spelled out.

## Verification

* Scoped (`python -m pytest tests/ros/test_toss_continuous_node.py
  tests/ros/test_toss_session.py tests/ros/test_reload_sequencer.py
  tests/ros/test_reload_coordinator_node.py tests/ros/test_toss_sequencer.py -q`,
  run 2026-08-29): **620 passed in 23.53 s**.
* ROS sweep (`python -m pytest tests/ros/ -q -n 4 --dist loadfile`, run
  2026-08-29): **2615 passed, 1 skipped in 96.00 s** — measured on the
  **pre-reach-gate** tree. The post-gate sweep in
  [[2026-08-29-displacement-caps-removed]] (**2630 passed, 1 skipped in 102.49 s**,
  same day, same invocation) covers both changes together, and is the number to
  cite for the combined commit.
* `full tier: `./run_tests.sh --full` (run 2026-08-29) WEDGED at ~4.5 h in the known OOM class (xdist workers in futex_wait, 1 GB available; killed per the documented remedy) and was NOT re-run at wrap-up (owner asked to close out). Coverage stands on, all 2026-08-29 on this tree: `./run_tests.sh` default gate PASS (**6252 passed, 4 skipped in 275.73 s**); unfiltered `pytest tests/motion/ tests/sim/ -q -n 4 --dist loadfile` (**3389 passed, 3 skipped, 3 xfailed in 435.32 s** — nightly-marked included); post-reach-gate `pytest tests/sim/ -x --ignore=tests/sim/test_toss_gate.py` (**1446 passed, 3 xfailed in 1028.95 s**) + `test_toss_gate.py` (**18 passed**); `pytest tests/ros/ -q -n 4` (**2630 passed, 1 skipped in 102.49 s**). The sole unrun tier is the firmware-native compile battery, on a tree with zero firmware-source changes (the removed generated constants have zero references); the 04:00 nightly re-runs the full tier as the backstop — read `temp/reports/nightly/status` at next session start.`
* The three standalone-reload suites (`test_reload_coordinator_node.py`,
  `test_reload_sequencer.py`, `test_reload_integration.py`) are **unmodified in the
  working tree** — which is the byte-identical claim's strongest evidence, and it is
  evidence a test cannot fake.

## Outcome

A caught reload hands the session back **~2.56 s sooner** (2.80 s floor → 0.24 s,
and the 2.0 s `go_home` profile gone with it), and the operator has a further
**~4.5 s** available by lowering `throw_delay_s` from 5.0 now that the resume path
no longer needs it to cover a serial recentre.

The operator's hypothesis was wrong and the operator's *perception* was right, which
is the useful pattern here: the resume really was slow, and none of the slowness was
BallButler's. Two of the three terms were the machine charging itself twice for the
same work — once in the floor's anchor, once in the recentre's destination.

⚠ **Deploy**: `cd ros_ws && colcon build --packages-select jugglebot` before the
next sitting — the launch runs the install space, so this and its sibling are inert
until it is rebuilt (one build covers the whole wave; `jugglebot_interfaces` is
untouched). Note the sibling **regenerates config**, so the build is not optional in
the way a pure-Python edit's is.

**Unflown.** The first sitting to run `on_empty_cup: RELOAD` is the first evidence.
Two things to watch:

1. **CONT-2.7** — the ball stays seated over the held 12° tilt, for the whole hold,
   within 35 mm. This is the accepted tradeoff and the one with an escape hatch.
2. **The first post-reload throw from the tilted pose** — the resumed cycle now
   commands its positioning move *from* the reload catch pose rather than from
   home, which is a starting condition no sitting has flown yet.

Siblings: [[2026-08-29-displacement-caps-removed]] (same commit); this morning's
wave [[2026-08-29-position-busy-repoll]] (whose BUSY re-poll this entry's floor
delegates its settle-hold coverage to), [[2026-08-29-bb-reload-busy-patience]] (the
two BB waits this entry's measurement exonerates from the *resume* path) and
[[2026-08-29-rejection-message-enrichment]].

## Withdrawn claims

- [2026-08-29] Operator hypothesis, from the report that opened this
  investigation: *"the post-RELOAD resume is slow because JB waits for BB to
  settle."*
  **WITHDRAWN by measurement** (bags `2026-08-29_15-30-29` and `_15-40-40`, n = 13
  caught interludes): **zero BB-keyed waits exist on the resume path.** All three
  BallButler dependencies are pre-catch — the gate's rung 2, the FSM's CHECKING
  patience, and the throw countdown — and nothing reads `/bb/heartbeat` after
  `Reload CAUGHT`.
  **Superseded by**: Diagnosis §§ D1–D4 (a naked `throw_delay_s`, a
  landing-anchored floor charged at the verdict, and a `go_home` to the next
  cycle's own throw site), and pinned as a regression by
  `test_a_successful_attempt_never_re_enters_the_gate_or_the_bb_wait`.

## Open questions / follow-ups

* **`throw_delay_s = 5.0` is the biggest remaining lever and it is the operator's
  to pull.** ⚠ Note for accuracy: 5.0 s is *also* the shipped code default
  (`DEFAULT_TOSS_THROW_DELAY_S`, `toss_sequencer.py`), carrying its own documented
  budget rationale (`CHECK ~0.1 + POSITION ≤ ~2 + PREPARE ≤ ~0.5 + event_delay
  ≥ 1.0 + slack`) — so "lower it" is a **goal-level** change, not a config edit, and
  the default's rationale is unrelated to the level-out. Separately, the operator
  *did* reach for 5.0 s as a remedy during the 2026-08-28 debrief, and
  [[2026-08-29-position-busy-repoll]] § D2 proved that use had the **wrong sign**
  (raising the delay moves POSITIONING *earlier*, deeper into the settle hold) and
  bought nothing. Both facts are true and they are about different things; the entry
  does not claim 5.0 was minted as a level-out workaround.
* **The S7 exemption's premise has moved.** `_recenter` / `_safe_abort` are exempt
  from carrying the drain so they stay byte-identical for the reload — but the
  interlude no longer uses `_recenter`, and `_recenter_stay` installs no profile at
  all. Nothing is broken; the justification is now standing on ground that shifted.
* **The savings shrink at tight cadence.** The 7-of-12 skip evidence is *pre-fix*;
  post-fix essentially every resumed cycle commands the move inside the countdown,
  and `lead_floor` covers a countdown too short for it. Re-measure the net at a real
  dwell before quoting 2.56 s as a cadence improvement.
* **`_reload_interlude_budget_s` stays over-counting on purpose.** Named here so the
  next reader does not "fix" it; the constant's own comment says why.
