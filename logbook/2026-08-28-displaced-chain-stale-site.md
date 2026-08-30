---
title: "The displaced chain threw from B with an aim solved for A — a cached truth outliving its instant"
type: bugfix
date: 2026-08-28
status: resolved
phase: "toss-pipelined-preamble — first pipelined sitting, fix wave 2"
related_plan: toss-pipelined-preamble.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_integration.py
  - tests/ros/test_toss_pipeline_properties.py
  - tests/motion/test_cadence_rung_check.py
  - tests/hardware/toss_trace_recorder.py
  - tools/probes/cadence_rung_check.py
subsystem:
  - ros
tags:
  - bugfix
  - toss
  - pipeline
  - tier8b
  - safety
  - contract
---

# The displaced chain threw from B with an aim solved for A — a cached truth outliving its instant

## Summary

Bag `2026-08-28_14-48-38`. On a **displaced** `toss_continuous` chain (catch
pose B ≠ the throw site A) the sitting alternated caught/missed, and the missed
balls did not scatter — they came **home**. The aimed throw at a +70 mm target
landed at **x = +3.98 mm**, i.e. the ball left the platform along the residual
receive tilt as though the platform were still at the origin.

It was: the first STAGED cycle of each chain nominated its throw site from the
LIVE `trajectory/commanded_position`, read **at stage time** — during the
previous cycle's flight, with the platform still at A — and cached
`positioning_move=False` (the census-B1 skip), which was **honestly true at that
instant**. The previous cycle's deferred A→B reach then moved the platform ~71 mm
and mirrored the tilt. Nothing re-read. The staged cycle released from B with a
release state, a tilt aim and a pre-tilt pose all solved for A.

The defect is not the answer, it is the **timing of the question**. A serial
cycle's nomination makes itself true — POSITIONING *commands* the platform to
the pose derived from A and CHECKING waits for the arrival, so "the platform is
at A at release" holds by construction. A staged cycle's POSITIONING is a SKIP
by construction (§ 2.4.1: it may not move the platform under an airborne ball),
so it commands nothing, makes nothing true, and the by-construction argument
evaporates — precisely in the window in which the cycle ahead of it moves the
platform.

Two owner-approved layers landed, and they close the CLASS rather than the bug:

1. **the staged cycle nominates the PREDICTION**, not the live read —
   `_predicted_chain_site_mm`, the same single derivation the accept-time
   `REJECTED_CHAIN_UNREACHABLE` check already consulted (it *knew the right
   answer, on the same node, all along*). The serial path's live read is
   unchanged and correct;
2. **the honest-cache contract**: at the COMMIT tick — the instant the throw
   becomes irrevocable — the FSM re-validates that the live platform pose is
   still the pose the nomination assumed, within the existing
   `_TOSS_ALREADY_THERE_TOL_MM` / `_TOSS_ALREADY_THERE_TOL_RAD`. A mismatch is
   `REJECTED_SITE_MOVED`: the staged slot is discarded and the cycle is rebuilt
   on the serial path, with nothing announced and nothing armed.

Layer 1 also delivers, for free and as ONE decision rather than two, the guard
that stops a displaced cycle staging at all: with the honest nomination its
positioning pose is at B while the platform is at A, so `positioning_move` is
honestly True and the existing skip-only rule declines it.

## Symptom

Operator report, verbatim:

> Displaced tosses alternate — one caught, one missed, one caught, one missed,
> like clockwork. And the misses aren't wild; the ball comes back to the middle,
> near where the platform started, not anywhere near where it's supposed to land.

## Diagnosis

### D1 — the three sites, and the one that was wrong

Tier 8b's cycle has three positions that a co-located chain conflates and a
displaced chain does not:

| | co-located chain | displaced chain |
| --- | --- | --- |
| where the cycle THROWS from (A) | B | A |
| where the cycle CATCHES (B) | B | B |
| where the platform ENDS the cycle | B | **B**, via the deferred reach |

The deferred A→B reach is published at `t_release` and arrives by the landing,
so the pose a cycle *ends holding* is the pose it **caught** at, not the pose it
**threw** from. On a co-located chain those are one place — which is exactly why
this survived every co-located sitting.

### D2 — the stage-time read, and the cached skip

`_build_toss_cycle`, tier 8b: `throw_site = self._live_commanded_position(...)`,
carrying the comment *"after a CAUGHT toss the platform STAYS at B, so the next
goal's A is that B for free"* — true of a co-located chain, false of a displaced
one. Ten lines later the positioning decision is taken **once** and cached
(2026-08-23, and rightly: the CHECKING delay gate has already charged a
pre-dispatch budget keyed on that boolean, so a second derivation could take the
cheap branch under an expensive budget). `_position_platform_for_toss` then
reads the cache and re-evaluates nothing:

```python
with self._lock:
    already_positioned = not bool(state.positioning_move)
```

Both are correct in isolation. Together, on a staged cycle, they are a truth
cached across the one interval in which it stops being true.

### D3 — the node's own confession, in its own log

The staged cycle logged

> `POSITIONING skipped — platform is already at (…) mm within 17.5 mm (census B1); nothing commanded`

**3.7 s before the previous cycle's release** — i.e. before the previous cycle's
reach had even been published. The line is not wrong. The platform *was* there.
It simply was not going to be there when the throw fired.

The reconstructed harness timeline reproduces exactly this ordering (a `_Clock`
run, t₀ = 1000.000 s, throw_delay 5.0, height 1.30 m, B = (70, 0, 170)):

| t (s) | event |
| --- | --- |
| 1000.158 | cycle 1 DISPATCHES (serial path — the CAN frame carries a 4.84 s `event_delay`), so `note_cycle_committed` frees the staging slot |
| 1000.198 | cycle 2 is STAGED — live read `(0, 0)`, the platform is at A |
| 1000.24 | cycle 2 logs `POSITIONING skipped` — **4.8 s before cycle 1's release** |
| 1005.000 | cycle 1 releases; the deferred A→B reach is published |
| 1006.030 | the reach ARRIVES: the platform is at `(70.873, 0)` |
| 1006.2791 | cycle 2's COMMIT window OPENS (`seq.commit_at`) |
| 1006.318 | cycle 2's COMMIT TICK — pre-fix it throws here, from 70.873, aimed as if from 0; under the fix this is the tick the site gate refuses |

The 3.7 s in the bag and the 4.8 s here are the same number wearing a different
`throw_delay`; the ordering is what matters and it is identical.

**On the commit instant, because this entry quotes it twice.** The Verification
section below cites `REJECTED_SITE_MOVED` at **t = 1006.318**, and this table
first read 1006.44 — the two are the same event and 1006.318 is the correct
value. Both are cycle 2's COMMIT tick in the same harness
(`_DisplacedPipelineClock`, `t₀ = 1000.0`, `throw_delay 5.0`, `h = 1.30 m`,
`B = (70, 0, 170)`, `_TICK_S` 0.040); 1006.44 was a hand-reconstruction from
before the harness existed, three ticks late. Re-measured 2026-08-28 by
instrumenting `TossSequencer.step` under
`pytest tests/ros/test_toss_continuous_node.py::test_a_displaced_chain_never_throws_from_a_site_it_did_not_nominate`:
`commit_at = 1006.2791`, refusal at `now = 1006.318`, `t_release = 1006.5798`.

### D4 — the dwell hold, and why the ball went HOME rather than somewhere random

The record's dwell-tilt block shows the platform held **0.0003 mm** of motion
across the dwell: nothing drifted, nothing was disturbed. The ball left a
platform that was exactly where the reach had put it, at the orientation the
reach had commanded — the **receive** tilt, which for the incoming displaced
catch is the MIRROR of cycle 1's throw tilt. A ball released along a mirrored
throw tilt from B goes back the way it came. x = +3.98 mm against a +70 mm
target is not a miss with a large error; it is a **correct throw of the wrong
goal**.

### D5 — only the FIRST chained cycle after each un-staged one carries it

Cycles 3+ are built at B, where the census-B1 check honestly fails or honestly
passes against the right pose, and they work. The alternation the operator saw
is the defect's own period: a defective cycle MISSES → the miss's `go_home`
returns the platform to the origin → the next cycle is displaced again and runs
serially (correctly) → the cycle after it stages (defectively) → miss. Two-cycle
period, deterministic.

### What the evidence did NOT support

* **not a calibration or aim-magnitude error.** The landing is at the ORIGIN's
  answer, not at a scaled version of the target's. A gain error moves the ball
  along the aim; this put it on the wrong aim entirely.
* **not the deadlock class** fixed earlier the same day
  (`2026-08-28-pipeline-first-contact-deadlock`): no stalls, no hangs, and the
  session terminalised normally every time.
* **not `_predicted_chain_site_mm` being wrong.** It was RIGHT, on the same
  node, in the same process, computed from the same policy object — it was
  simply advisory, feeding only the accept-time `REJECTED_CHAIN_UNREACHABLE`
  check. See the Discussion.

## Adjudication — is chaining-at-B intent, or accident?

Before treating the live read as a defect it had to be established that the
displaced chain is a SHIPPED behaviour rather than an unconsidered path. It is,
in three independent places:

1. `_build_toss_cycle`'s own site comment: *"Reading it live is also what makes a
   session CHAIN: after a CAUGHT toss the platform STAYS at B, so the next
   goal's A is that B for free."*
2. `_toss_already_positioned`'s *"Why this fires on a chain, and only on a
   chain"* paragraph, which reasons explicitly about a chained session ending
   `ACTION_STAY`;
3. `_predicted_chain_site_mm`'s whole existence — a method whose docstring says
   *"the throw site cycle 2 of a session will read off
   `trajectory/commanded_position`"*, with measured frontier numbers for it.

So the chain is intended, and `_toss_already_positioned` answered the question
it was asked, honestly. **The defect is that a staged cycle asks it at an
instant that has no authority over the throw.**

## Discussion

### Why the fix is a re-timed question and not a widened tolerance

The obvious "fix" — widen `_TOSS_ALREADY_THERE_TOL_MM` until 71 mm passes, or
drop the check for staged cycles — is the forbidden one, for the same reason
P-4 (`_toss_reach_quat`) refused to widen the angular tolerance: that tolerance's
job is to make a platform that has NOT applied the commanded correction
impossible to mistake for one that has. Widening it to admit the bug's 71 mm
would make the skip answer YES to a platform two-thirds of a workspace away.

The correct move, and the one this repo has now made twice, is to **change what
is commanded (or when it is asked) so the existing check can answer honestly**.
P-4 published the pre-tilt so the orientation check could pass truthfully; this
fix nominates the site the platform will actually be at, and re-asks the
question at the last instant that can still act on the answer.

### Why the class closure is at the COMMIT tick and nowhere else

The staged ladder has four instants where the question could be re-asked: at
stage, at PREPARE, on every STAGED tick, and at COMMIT. Only the last is a
CLASS closure, because only it has the property that nothing can move the
platform between the answer and the CAN frame — the announcement and the
dispatch go out in that same tick, by design (§ 2.4.2), precisely because a
slipped release invalidates an already-published announcement.

Re-asking earlier would close *this* bug (the reach fires at `t_release`, before
the commit) and leave the class open: a SpaceMouse nudge, a reload interlude
recentre, a tracker-refined catch that parks the platform a few mm off, or any
future producer of platform motion would walk straight back in. The contract is
therefore stated as a temporal invariant, not as a list of movers:

> **A staged nomination is only valid if the platform is where the nomination
> assumed at the moment the throw becomes irrevocable.**

One normative sentence (`TossObservations.staged_site_ok`'s comment plus
`_staged_site_ok`'s docstring), one enforcement point (`_step_committing`
rung 3), one test that fails if it is violated
(`test_the_commit_refuses_a_site_that_moved_under_the_staged_cycle`, plus the
end-to-end `test_a_displaced_chain_never_throws_from_a_site_it_did_not_nominate`
with the pre-fix nomination restored as a parametrisation).

### Why it REJECTS rather than SLIPS

Wave 1 of this same fix arc converted the commit gate's release-window shortfall
from an abort into a SLIP, on the doctrine "a cadence fact on a healthy machine
is not a machine fault". `SITE_MOVED` is deliberately NOT routed that way, and
the reason is that a slip is a bet that waiting helps. Here it cannot: the
orientation the reach commanded is the one it *meant* to command, and the
platform is not on its way anywhere. Slipping would burn the whole
`catch_confirm_window_s` bound before falling back to serial — a cadence
disaster in exchange for nothing — whereas the serial rebuild re-nominates from
a fresh live read and COMMANDS the move, which is strictly more correct than
waiting for a stale nomination to come true.

### Why the staged nomination reuses `_predicted_chain_site_mm` verbatim

The brief's requirement was one derivation, not a second copy, and the honest
reading of the situation is that the derivation already existed and was already
right — it was simply advisory. Making the staged nomination call the same
method means the accept-time chain reachability check and the runtime nomination
can never disagree about where a chained cycle will be; if the prediction is
ever wrong, both move together and the accept gate refuses before a ball flies.

The alternative considered and rejected was to have the staged cycle nominate
`catch_pose` (B) directly. It is nearly right and would have been simpler, but
it is a *second* answer to the same question: B is where the CUP goes, and the
wire republishes the CENTROID, and the two differ by the cup swing (up to
~3 mm at the box edge — measured, `_predicted_chain_site_mm`'s own docstring).
That difference is exactly what `_predicted_chain_site_mm` exists to model.

### The control-system walk, and what it says about displaced chains today

Walked one displaced chained cycle under the fix, then confirmed it against the
real code in a harness with a plant that actually moves
(`_DisplacedPipelineClock`):

1. cycle 2 is staged during cycle 1's flight; it nominates the predicted chain
   site ≈ B = (70.873, 0);
2. its positioning pose is therefore at B — level (the throw B→B is co-located)
   or aim-tilted — while the platform is still at A = (0, 0) with cycle 1's
   13.5 mrad throw pre-tilt;
3. `_toss_already_positioned` fails on **both** components: 70.9 mm against a
   17.5 mm bound, and 13.5 mrad against a 2.71 mrad bound;
4. `positioning_move` is honestly True, `TossSequencer.__post_init__`'s
   skip-only belt forces `staged=False`, and `_start_pipelined_cycle` drops the
   slot with `note_stage_abandoned('POSITIONING_MOVE')` — before any record is
   opened, so the cost is one build and one solve, not a drain;
5. the session rebuilds cycle 2 SERIALLY once cycle 1 terminalises; its
   POSITIONING commands the move to B (measured in the harness: target x = 70.9,
   `positioning_move` True) and it throws correctly.

**So the answer to "can a staged cycle ever survive commit on a displaced chain
today?" is stronger than the expected 'no': it never reaches the commit,
because it never stages.** And the follow-on question — "is the stage's work
therefore wasted every cycle?" — resolves the other way from the brief's
expectation, which is worth stating plainly because it changes the cadence
story:

> **Only the FIRST chained cycle after a displacement runs serially. From cycle
> 3 the chain is CO-LOCATED (the platform is parked at B and throws from B), so
> it stages and pipelines normally.**

Measured in the same harness run: cycle 2 declines (`POSITIONING_MOVE`), cycle 3
stages, reaches `PHASE_STAGED`, and commits — 153 ticks with both slots live.
A displaced session pays the serial floor once, not forever.

That is also why **no separate "don't stage a displaced cycle" guard was
added**, though the brief offered it as an option. The skip-only rule already
asks exactly that question, and asking it twice — once as `positioning_move` and
once as a displacement test — creates two answers free to disagree, which is the
accept-vs-runtime gap the 2026-08-23 single-decision rule closed. The guard the
brief wanted is the honest nomination; it is one line, and it is already there.

### The co-located identity, and why it is asserted as `==`

The requirement was that no co-located decision moves. It does better than that:
on a co-located chain the prediction is the live read's own **fixed point**, bit
for bit. A self-toss lands with a purely vertical velocity, so the catch
policy's receive tilt is identity, the cup swing is zero, and the parked centroid
is B exactly. Measured through the live modules at two poses:

| catch pose | live read | predicted chain site | delta |
| --- | --- | --- | --- |
| (0, 0, 170) | (0.0, 0.0) | (0.0, 0.0) | **0** |
| (30, −20, 170) | (30.0, −20.0) | (30.0, −20.0) | **0** |

`test_on_a_colocated_chain_the_prediction_is_the_live_read_bit_for_bit` asserts
`==` rather than a tolerance on purpose: a tolerance would hide the day the
co-located chain acquires a residual, and somebody would need to know why.

### Side findings, banked rather than fixed

Recorded here because they were established from the same bag and would
otherwise be lost:

* **the loop overruns are the BLOCKING POSITIONING call.** 48 overruns, all
  slipped (the wave-1 fix working), worst 537 ms; body ≈ work ≈ overrun on every
  one, and the work is the `go_to_pose` round trip. The fix is to unblock the
  loop, not to widen a budget. **Banked, not done here.**
* **`toss_cal_loaded` was false**, and the uncalibrated tilt→lateral gain runs
  ~1.3–1.5× — aimed throws overshoot. Independent of this defect (it scales the
  aim; this bug used the wrong aim) but it is on the board for the next sitting.
* **`mocap "base misaligned: pos=nan" ×69`** — a NaN passing a threshold check.
  Same class as the `_on_commanded_pose` non-finite guard; **banked**.
* **`/trajectory/commanded_pose` is missing from the recorded topic set.** The
  orientation half of this diagnosis had to be FK-reconstructed from leg
  lengths. Recommend adding it to the recorder — it is the topic the census-B1
  skip and now the commit gate both read, and it is currently invisible in a bag.
* **the tilt-only re-orient timing law**: **T ≈ 0.196·θ_deg^(1/3) s**, fitted
  over the tilt-only (`Δxy = 0`) column of
  `plans/archived/toss-multi-catch-pose.md` § 1.4 Finding 3's lean-off table
  (0.2471 s at 2°, 0.3121 s at 4° ⇒ 0.1961 / 0.1966 — the 0.189 first written
  here was fitted before that table existed). Against the **lean-off** plan the
  shipped `go_to_pose` runs **~2.6–3.0×** longer — 2.62× at the ring's hop 0
  (0.5238 vs 0.2000 s) and 3.03× at hop 1 (0.6060 vs 0.2000 s); the plan's
  P2 probe quotes 2.4× because it compares the shaped and bare plans of the
  *same* 2 mm pre-tilt pose (0.6069 vs 0.2532 s), which is a different pairing
  and both are stated rather than blurred. The reach path does 72 mm + 2° in
  0.861 s. Relevant to the future constant-beat work, not to this fix.

### Deadlock-fix validation harvest (wave 1, from the same bag)

The bag is also the first sitting with the wave-1 deadlock fixes, and it clears
them. Produced by
`python tools/probes/toss_loop_census.py --csv --date 2026-08-28` over
`temp/logs/`'s toss-record JSONL for bag `2026-08-28_14-48-38` (run
2026-08-28), which is the same census invocation
`logbook/2026-08-28-pipeline-first-contact-deadlock.md` § Verification cites —
so the two entries' numbers come from one reader, not two:

**zero stalls, zero hangs**; 48 overruns, **all slipped**, worst 537 ms;
`commit_slip_s` n=35, median 0.078 s, healthy p90 ≈ 0.20 s, healthy max
0.238 s — and the four 0.41 s / 11-slip outliers are all on the drain path, not
the healthy path. 25/25 co-located catches down to dwell 0.45.

## Fix

Two production files.

**`toss_sequencer.py`**

* `TossObservations.staged_site_ok: bool = False` — the staged slot's own
  observation, FAIL-CLOSED by the same doctrine as `platform_levelled` and
  `throw_site_known`;
* `_step_committing` gains **rung 3**, between the static link gates and
  `hand_parked`: `if not obs.staged_site_ok: return self._reject('SITE_MOVED')`.
  Placed there so an invalid nomination routes the operator by name instead of
  slipping on `hand_parked` / `ball_seated` for a reason that will never
  resolve, and so the fall-back to serial happens as early as possible.

**`reload_coordinator_node.py`**

* `_build_toss_cycle`: a tier-8b **staged** cycle nominates
  `_predicted_chain_site_mm(catch_pose, flight)`; the serial path keeps the live
  read. A prediction that fails for a reason that is not a missing pose sets a
  local `stage_ok = False` (⚠ **not** `staged`, which routes the slot INSTALL —
  flipping that would publish the cycle over the one that owns the hand) so the
  cycle takes the serial path;
* `_staged_site_ok(state)` — the re-validation. It calls the SAME
  `_toss_already_positioned` against the SAME cached `state.platform_target_mm`
  and `state.release_cmd` the build decided from; only the live pose read inside
  it is fresh. Fail-closed on every unknown;
* `_staged_observations(obs, state)` feeds it into the staged slot's snapshot
  every tick, so the value the commit gate reads is the commit tick's own read;
  `_build_toss_observations` also computes it when handed a staged state, so any
  path that steps a staged cycle gets the honest answer rather than the default.
  ⚠ Those are two producers, and when the committed slot is empty
  `_tick_toss_pipeline` builds the tick's ONE snapshot **from the staged
  state** — so both fired and the live pose was read **twice for one
  decision**, free to disagree across the gap. That is the split-observation
  shape C-POSSESS-1 § 3.3 edit 1 closed for the cup, re-created for the site.
  `_staged_observations` now takes `site_ok_already_read` and **carries** the
  snapshot's value in exactly that case; the caller is the only code that knows
  which state the snapshot came from, so the decision lives there. Both
  producers are kept — dropping `_build_toss_observations`' would leave a
  staged cycle stepped through any other path on the fail-closed default;
* `_start_pipelined_cycle`'s decline reason is **carried on the state**
  (`TossCycleState.stage_decline_reason`, set to `'CHAIN_SITE_UNKNOWN'` at the
  one place that knows why, `'POSITIONING_MOVE'` as the fallback) instead of
  being re-derived from `positioning_move_expected` at the decline site. The
  re-derivation was a second answer to a question the build had already
  answered, and a cycle that is BOTH a mover and a failed prediction would have
  been reported as the wrong one;
* `_staged_site_ok`'s docstring gains the operator clause the trace recorder's
  `REJECT_WIRE_MAP` also carries: a `REJECTED_SITE_MOVED` can equally mean
  `trajectory/commanded_pose` went stale or absent, **which reads the same way
  by design**, so check the topic is publishing before hunting for a mover —
  and it is not in the bag topic list, so a recording cannot answer it.

**Docstrings corrected, because they were the load-bearing false premises:**

* `_toss_already_positioned` — *"leaves the platform exactly where the cycle
  threw and caught from"* is TWO places on tier 8b, and the entry says so with
  the bag cited;
* `_build_toss_cycle`'s chain-read comment — the live read is the SERIAL path's
  nomination and only its, with the reason (`by construction` rests on
  POSITIONING commanding a move, and a staged cycle's does not);
* `_build_toss_cycle`'s `staged` paragraph — the throw site is now the ONE thing
  a staged cycle computes differently, and the identity that survives is *both
  paths nominate the site the platform will be at when the ball leaves the
  hand*;
* `_position_platform_for_toss`'s cached-decision comment and
  `_predicted_chain_site_mm`'s docstring gain the cross-references.

**Test-fixture changes (inputs, not assertions):** the four healthy
`TossObservations` fixtures gain `staged_site_ok=True` (the same shape
`platform_levelled` takes there); `test_toss_integration._obs_ok` re-stamps
`_commanded_pose_mono` alongside `_commanded_pos_mono` (the real graph publishes
both on one timer); `test_toss_coordinator._install_toss_goal` sets
`release_cmd = release` as `_build_toss_cycle` does. **No existing assertion
changed.**

## Verification

* FSM + node + pipeline + integration + coordinator + cadence, scoped
  (`python -m pytest tests/ros/test_toss_sequencer.py
  tests/ros/test_toss_continuous_node.py
  tests/ros/test_toss_pipeline_properties.py tests/ros/test_toss_integration.py
  tests/ros/test_toss_coordinator.py tests/motion/test_cadence_rung_check.py -q
  -p no:randomly`, run 2026-08-28): **529 passed in 80.32 s** (before the new
  tests were added — the no-regression baseline, with zero assertion edits).
* **THE GATE** (`./run_tests.sh`, run 2026-08-28): **6162 passed, 4 skipped in
  274.12 s** (parallel 277 s, serial phase empty; total 287 s, RESULT: PASS).
* Full ROS sweep (`python -m pytest tests/ros/ -q -p no:randomly`, **with the
  new tests**, run 2026-08-28): **2554 passed, 1 skipped in 324.55 s**.
* Full motion sweep (`python -m pytest tests/motion/ -q -p no:randomly`, **with
  the new tests**, run 2026-08-28): **1954 passed, 3 skipped in 321.75 s**.
* Cadence grid (`python tools/probes/cadence_rung_check.py --pipeline --grid`,
  run 2026-08-28): **0 violations at the modelled floors**, 196 with the loop
  period removed from the commit budget (the non-vacuity control) — all six
  pipelined rungs still COMMIT, floors unmoved.
* The regression itself, both layers
  (`test_a_displaced_chain_never_throws_from_a_site_it_did_not_nominate`,
  parametrised): with the shipped nomination the first chained cycle declines
  `POSITIONING_MOVE` and never stages; with the **pre-fix nomination restored**
  the staged slot really does cache the stale site and the commit gate refuses
  it `REJECTED_SITE_MOVED` at t = 1006.318 — and in BOTH cases every one of the
  three dispatches nominates a site within 17.5 mm of where the platform
  actually is. The harness's own premise is asserted too (the plant traverses
  > 60 mm, and a `reach` command is among the movers), so the test cannot pass
  vacuously on a platform that never moved.

## Outcome

The displaced chain now throws from where it is. The first chained cycle after
a displacement pays the **serial MOVING floor** once — **0.8970 s at
`h = 1.0 m`**, from the shipped `min_throw_delay_for_release_s(v, True)`
(0.7908 s, measured 2026-08-28) plus the handoff margin. It is a *moving* floor,
not the ~0.55–0.65 s the first draft of this section guessed, because the
rebuilt cycle really does COMMAND a positioning move; and the real arrival is
longer still than the budget charged for it, because
`pre_dispatch_budget_s(True)` assumes the move plans at
`min_move_duration_s` while the shipped `lean_gain` 0.6 makes it plan
0.52–0.61 s — see `plans/archived/toss-multi-catch-pose.md` § 2.7 for the
derivation and § 1.4 Finding 4 for why the budget is a lie about the arrival.
From the cycle after that the chain is co-located and pipelines exactly as
before — the
co-located path is bit-for-bit unchanged, proven by an identity rather than a
tolerance. `REJECTED_SITE_MOVED` gives the class a name and a counter
(`staged_discarded_reason`), so if anything else ever moves the platform between
a stage and its commit, the machine says so instead of throwing.

Owner decisions recorded: the fix as specified; **the catch stays at RECEIVE
tilt** (the mirrored orientation is correct for catching, and the throw's
requirements are the next cycle's business); displaced-chain floors rise
honestly rather than being bought back with a fast path; constant-beat is
deferred, with the **pre-tilt catch** documented as the future lever that would
let a displaced chain stage again.

## Open questions / follow-ups

* ⚠ **OPERATIONAL FENCE — the declined-then-serially-rebuilt cycle is a MOVING
  cycle, and at the shipped lean gain it does not fit the cadence lead.** Its
  arrival exceeds `pre_dispatch_budget_s(True)`'s 0.400 s allowance (the budget
  assumes the positioning move plans at `min_move_duration_s`; at
  `trajectory_op.lean_gain = 0.6` it plans 0.52–0.61 s, so the real arrival is
  0.72–0.81 s), and the serial `_step_preparing` guard has **no slip** — wave
  1's slip is on the pipelined commit tick and is not on this path — so
  `_step_preparing` aborts `ABORTED_CANT_MAKE_RELEASE` outright. On this
  sitting's 5 s operator delay the lead covers it and nothing was seen; on a
  cadence lead it will not. **Cadence-lead displaced sittings should not be
  flown until `plans/archived/toss-multi-catch-pose.md` M2's honest budget, or
  `lean_gain = 0.0` for the tilt-in-place move class, lands.** Co-located
  sittings are unaffected — they take the skip and command no move.
* **the blocking POSITIONING call** — the sole source of the sitting's 48 loop
  overruns. Unblocking the loop is the fix; banked.
* **`toss_cal_loaded` false** — the uncalibrated ~1.3–1.5× tilt→lateral gain
  overshoots aimed throws. Independent of this defect, but it is what a displaced
  sitting will hit next.
* **`/trajectory/commanded_pose` is not recorded** — add it to the recorder. Two
  gates now read it and neither is visible in a bag.
* **the mocap `pos=nan` × 69** — a NaN passing a threshold check; same class as
  the guards already in `_on_commanded_pose`.
* **a fast path for displaced chains** would need the catch to arrive pre-tilted
  for the *next* throw rather than for the current receive — deliberately not
  done, and the reason it is a real design question rather than a tuning knob is
  that it trades catch quality for cadence.
