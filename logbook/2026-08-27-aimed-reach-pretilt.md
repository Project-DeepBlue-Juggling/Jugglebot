---
title: The deferred A→B reach stops levelling the aim out of the platform, and the aimed chain finally stages
type: investigation
date: 2026-08-27
status: resolved
phase: "Phase B — pipelining the toss preamble (P-4)"
related_plan: toss-pipelined-preamble.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_continuous_node.py
  - tests/hardware/session_cadence_ladder.md
  - plans/active/toss-pipelined-preamble.md
subsystem:
  - ros
tags:
  - toss
  - cadence
  - ilc
  - contracts
---

# The deferred A→B reach stops levelling the aim out of the platform, and the aimed chain finally stages

## Summary

On the shipped tier (`JB_OP_TOSS_TIER = '8b'`) with an aim armed, every chained
toss cycle re-commanded its pre-positioning move, so the census-B1 skip never
fired, every cycle paid the 0.520 s moving budget, and — under Phase B4's rule
that *a cycle stages only if its positioning decision is SKIP* — no cycle ever
staged and the whole pipeline was inert on the tier that ships. The cause was not
a stale check: the platform really was being tilted for the throw, **levelled at
`t_release` by the deferred A→B reach**, and tilted back on the next cycle, every
cycle, to end where it started. `_publish_toss_reach` now publishes the throw's
commanded **pre-tilt** quaternion instead of the catch policy's receive tilt
whenever the two agree to within the ±1° aim authority, which closes
`tests/hardware/session_cadence_ladder.md`'s carried finding 2 (plan § 8.1 P-4)
and satisfies § 6.1's "do not book any rung" gate.

## Symptoms

From the 2026-08-24 audit of the cadence ladder, carried unfixed for three days:

* on tier 8b with an aim armed (an ILC artifact, a calibration map, or a
  displaced site), `_toss_already_positioned` returned False on **every** chained
  cycle, so POSITIONING commanded a `go_to_pose` that traversed ~1 mm and
  re-tilted the platform by the armed aim (≤1°);
* each such cycle was charged `pre_dispatch_budget_s` = 0.520 s instead of
  0.160 s — 0.360 s of throw delay, and one-for-one the same in dwell;
* after Phase B4 the consequence got worse rather than better: a cycle that must
  move may not occupy the staging slot (it would traverse the platform during the
  previous cycle's flight, under a ball the catch is armed for), so **an aimed
  session never engaged the pipeline at all**;
* every cadence number in the ladder was therefore produced on the
  8a-equivalent (unaimed) path.

## Diagnosis

The two orientations in a chained cycle have **two different authors and no
arbiter**:

| what | who chooses it | value on a self-toss |
|---|---|---|
| the cycle's TERMINAL orientation | the catch policy — `_publish_toss_reach` → `predicted_catch_command` → `compute_catch_orientation(landing_velocity)` | **exactly identity** |
| the next cycle's INITIAL orientation | the throw aim — `release_cmd.tilt_rx/ry` → `_position_platform_for_toss` | the armed aim, ≤ 1° |

They agree only at exactly zero aim. The "exactly identity" is not an
approximation and it is worth naming, because it is what makes the fix's bound
exact: contract C-TOSS-CAL-1's D4 keeps the **announcement** on the UNCORRECTED
release (`_announce_toss` reads `state.release_state`, never `release_cmd`), so
for a co-located throw the announced landing velocity is exactly vertical,
`compute_catch_orientation` takes its `angle < 1e-9` branch, and the receive
quaternion is literally `[1, 0, 0, 0]`. The delta the platform was being asked to
traverse each cycle was therefore **exactly the armed aim magnitude** —
`hypot(aim_rx, aim_ry)`, already clamped ≤ `toss_cal.TOTAL_MAX_RAD` by
`clamp_total_aim`.

Measured (`/tmp/probe_p4_reach_delta.py`, 2026-08-27, flight 0.5029 s, aim about
+x), delta = ‖rotvec(receive) − rotvec(pre-tilt)‖:

| regime | delta | vs the 2.71 mrad B1 tolerance |
|---|---|---|
| zero aim, co-located | 0.000000 rad (both sides exactly identity) | skip fires |
| aim 0.155° | 2.705 mrad | on the boundary, by construction |
| aim 0.5° | 8.727 mrad | skip DECLINES |
| aim 1.0° (saturated) | 17.453 mrad | skip DECLINES |
| displaced 20 mm | 32.256 mrad | skip DECLINES |
| displaced 150 mm (the cap) | 240.772 mrad | skip DECLINES |

The displaced row is ~2θ, and that is the geometry: a ball thrown with tilt θ
lands with its lateral velocity on the same side, so the receive orientation is
the **mirror** of the throw orientation.

**The open sub-question the ladder left is answered: only the ORIENTATION ever
broke.** The reach's position residual is the cup swing alone — the next cycle's
swing-compensated pre-tilt pose sits **1.013 mm** from the parked centroid at a
full ±1° aim, against a 17.5 mm tolerance. 17× of margin; the position half was
never at risk, at any aim the clamp admits.

## Discussion

### The hypothesis that died: "this is a bookkeeping mismatch"

The finding was written — and the fix was scoped — as *a check that cannot see
what it needs*. That framing had a precedent that made it very plausible: the
2026-08-23 fix to this exact skip was precisely that (the check could not verify
an orientation because `trajectory/commanded_position` published only a position;
the fix published `trajectory/commanded_pose` and the check got **stricter**, not
looser). Under that framing the obvious repair here is symmetrical — teach
`_toss_already_positioned` to accept the pose the platform is holding.

The code does not support it. `_publish_toss_reach` publishes a
`catch/dynamic_target` whose orientation trajectory_node plans a real reach to;
the platform physically levels. `_toss_already_positioned` was answering
**honestly** the whole time: the platform genuinely was not at the pre-tilt any
more. The mismatch was not between a check and reality, it was between two
subsystems that each independently decide what the platform's orientation should
be, with nothing reconciling them — and the machine paid 0.360 s per cycle to
execute their disagreement.

Reframing it that way changed which repairs are even admissible. A check-side fix
is now *forbidden*, not merely inferior: widening
`_TOSS_ALREADY_THERE_TOL_RAD` to admit an armed aim would make the skip answer
YES to a platform that has **not** applied the correction, and fire an ILC throw
with its aim missing. That constant is 1/6.4 of the aim authority for exactly
that reason, and its own comment says so. The only honest repairs move what is
*commanded*.

### Why the reach yields to the throw, and not the other way round

Two things could reconcile: make the throw accept the level pose (drop the aim),
or make the catch accept the tilted pose (hold the pre-tilt through the seat).
The first is not a candidate — the aim is the correction the whole ILC/calibration
programme exists to apply.

The second costs something real: the cup seats the ball with up to 1° of tilt
instead of level. That is a physical-intuition question, not a code question, so
it went to the owner, who accepted it (2026-08-27, verbatim: *"Yes, let's go with
a 1° max error for 8b"*).

What made the decision easy — and this is the part a future reader would not
recover from the code — is that **tier 8a has always done exactly this**. 8a emits
no deferred reach at all (`toss_sequencer._reach_action_if_due` is 8b-only), so an
aimed 8a cycle pre-tilts, throws, and catches the ball in a cup still held at the
full aim tilt. Every hardware-validated aimed cycle this machine has thrown has
seated its ball that way. The fix does not introduce a ≤1° seat tilt; it stops 8b
from paying 0.360 s per cycle to avoid one that 8a never avoided.

Two further things bound the accepted cost, both already in the tree:

* the ball then RESTS in that tilted cup until the next command, and that state is
  already accepted and instrumented — `_toss_stay`'s docstring records the held
  pose carrying *"the catch's RECEIVE TILT (up to ~3.6° at the 150 mm cap)"*
  indefinitely, scored as runbook row DISP-5. The comparison is better than that
  constant makes it look, because the constant is **flight-time dependent and
  never named its own flight time**. Measured 2026-08-27 at the 150 mm cap, the
  receive tilt is **6.9° at the ladder's 0.5029 s** and 2.7° at `T` = 0.80 s;
  3.6° is the value at `T` ≈ 0.70 s. At the flight time this ladder actually
  flies, ≤1° is therefore *several* times inside a residual the machine already
  lives with, not 3.6×. (`reload_coordinator_node.py`'s `_toss_stay` docstring
  has been annotated with those two measured points rather than left implying a
  single number.);
* the commanded motion during the flight strictly SHRINKS. The reach now asks the
  platform to hold its tilt and translate ~1 mm, where before it asked for the
  same ~1 mm plus a ≤1° rotation. Nothing new is commanded during a flight with a
  ball in the air; something is *un*-commanded.

**That divergence is the deeper finding.** Two tiers of the same action disagreed
about what the platform holds at the seat, nobody had noticed, and the tier that
"did the right thing" was the one that could not chain. It is now closed by making
both tiers hold the pre-tilt.

### Why the bound is `toss_cal.TOTAL_MAX_RAD` and not a number chosen to work

The substitution's entire cost is that the seat is re-aimed by the delta between
the two orientations. So the bound on the substitution IS the bound on the
accepted seat error, and the number is the one that already means "how far this
machine is allowed to aim": `TOTAL_MAX_RAD` = 1°, the D7 authority.

Two independent things then fall out rather than being arranged:

* on an aimed co-located cycle the delta is *exactly* the aim magnitude, which
  `clamp_total_aim` has already bounded by the same constant — so the rule fires
  **by construction**, not by luck, on every cycle the fix exists for;
* the same bound caps the lateral cup shift the substitution introduces
  (`HAND_CATCH_OFFSET_MM · sin δ`) at 64.78·sin(1°) = **1.131 mm** — which is
  literally one of the three reasons recorded in `toss_cal.TOTAL_MAX_RAD`'s own
  docstring for why that constant is 1°. The same number doing the same job.

An honest caveat, stated because the plan's P-4 row said "displaced never fires"
and that is *slightly* stronger than the truth: the bound is on the **angle**, so
it corresponds to ~10.8 mm of displacement at flight 0.5029 s (and grows as
`g·T²`). A genuinely displaced goal below that does take the substitution. "Never"
was the wrong word and the crossover is now pinned by test.

The first draft of that caveat then got the *aimed* case backwards, and the error
is worth recording because it is the kind that survives review by sounding
conservative. It claimed the displaced regime "stays outside the bound with a full
±1° aim added in either direction (36.7 mrad at 20 mm)" — i.e. that an aim can
only ever push the delta further out. It cannot. `aim_target_offset_mm` maps
`rx → −y` and `ry → +x`, so relative to a given displacement an aim has an
**orthogonal** component and a **co-axial** one, and the co-axial component has a
sign that *subtracts* from the throw tilt. The 36.7 mrad figure is real but it is
the orthogonal case only — the quadrature sum, and the easy direction. Measured at
20 mm of displacement and flight 0.5029 s:

| aim (saturated, ±1°) | delta | inside the bound? |
|---|---|---|
| none | 32.256 mrad | no |
| orthogonal (`rx`, either sign) | 36.673 mrad | no |
| co-axial, same sense (`ry+`) | 49.699 mrad | no |
| **co-axial, opposing (`ry−`)** | **14.802 mrad** | **yes** |

So the worst case is not 10.8 mm but **~21.6 mm** of displacement — the rule fires
at 21.5 mm and declines at 21.75 mm, bracketing a measured crossover of 21.644 mm.
That is twice the band the first draft implied, and it is the number that actually
bounds the shipped system.

It remains the bound working rather than leaking, and for a reason the corrected
framing makes *stronger* rather than weaker: the co-axial-opposing aim is the case
where the commanded pre-tilt has very nearly cancelled the throw tilt outright, so
the orientation being substituted in is 1.413 mrad against the 16.216 mrad receive
tilt it replaces. Where this rule reaches furthest into the displaced regime, it
is putting the platform *closer* to level at the seat, not further from it. Both
ends of the band are inside the ≤1° the owner accepted.

### What was deliberately NOT changed

The reach's **position** is untouched. Re-deriving `target_pos` so the cup lands
exactly on B under the pre-tilt was tempting (it removes the 1.131 mm) and was
rejected: `_predicted_chain_site_mm` predicts the parked centroid through the same
`_toss_catch_policy` object, and the whole point of that single-sourcing is that
the prediction cannot drift from what is commanded. Moving the commanded position
would have broken that for a millimetre nobody can measure. The centroid — not the
cup — is what `trajectory/commanded_position` republishes and what the next cycle
reads as its throw site A, so leaving it alone keeps the prediction exact.

## Fix

`ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py`:

* **new `_toss_reach_quat(policy_quat, release)`** (classmethod, beside its only
  caller). Returns `(w, x, y, z, pretilt_held)`. A release with no commanded tilt
  returns the policy's components verbatim — the level path does not reach one
  float of the arithmetic. Otherwise it decodes the policy quaternion to a rotvec
  (compared **before** the encoding, because quaternions double-cover — the same
  reason `_toss_already_positioned` compares rotvecs) and substitutes the pre-tilt
  iff `‖rotvec(receive) − (tilt_rx, tilt_ry, 0)‖ ≤ toss_cal.TOTAL_MAX_RAD`. The
  pre-tilt goes through `_tilt_quaternion`, the **same** encoder
  `_position_platform_for_toss` uses, so trajectory_node's inverse round trip
  hands the next cycle's B1 check back the identical rotvec. The docstring carries
  the derivation, the measured regime separation and both ripples;
* `_publish_toss_reach` calls it, and its INFO line gains a
  `— holding the throw PRE-TILT (P-4)` suffix when the substitution fires (one
  line per reach, unchanged in volume);
* `_toss_already_positioned` and `_predicted_chain_site_mm` docstrings record the
  two ripples: why the "fires on a chain" paragraph was false for an aimed 8b
  chain until today, and why the centroid prediction stays consistent.

`tests/ros/test_toss_coordinator.py` — five new tests and five helpers
(`_armed_aim`, `_announce_and_reach`, `_policy_answer`, `_park`, `_wire_quat`):
`_the_reach_quat_is_the_policys_verbatim_when_the_release_is_LEVEL` (byte
identity against a *non*-vertical policy answer, `==` not `approx`),
`_a_zero_aim_chain_publishes_the_policy_quat_unchanged` (the same, end to end
through the real builder), `_an_aimed_colocated_reach_holds_the_pretilt_and_
closes_the_chain` (the closure: reach quat == pre-tilt, position == the policy's,
the next cycle's `_toss_already_positioned` True and `positioning_move` False —
plus the **counterfactual**, parked at the policy's level answer, where the same
cycle declines and commands the move), `_a_displaced_reach_keeps_the_policys_
receive_tilt` (× 3 aims), and `_the_displaced_regime_is_separated_by_more_than_
the_bound` (re-derives the deltas from the live modules, including the ~10.8 mm
crossover and the 1.013 mm position residual at a saturated aim).

`tests/ros/test_toss_continuous_node.py` —
`test_an_aimed_colocated_chain_stages_now_that_the_reach_holds_the_pretilt`, with
an `_AimedPipelineClock` whose plant reads the published reach back off the wire
and decodes it exactly as `trajectory_node._catch_target_from_msg` does. The
existing `test_a_cycle_that_must_move_does_not_stage` is untouched and still
right.

`tests/hardware/session_cadence_ladder.md` — carried finding 2 CLOSED (root
cause, the answered position sub-question, the fix and its bound, the owner's
acceptance, the 8a/8b divergence). `plans/active/toss-pipelined-preamble.md` —
§ 8.1 P-4 ✅ CLOSED, § 6.1's booking gate satisfied, § 2.4.1's ⚠ annotated.

## Verification

* The gate (`./run_tests.sh --full`, run 2026-08-27): parallel **6565
  selected, rc=0, in 524 s** + serial **9 passed (rc=0) in 45 s**, total
  569 s, **RESULT: PASS**.
* Scoped, on the Jetson venv, 2026-08-27:
  `python -m pytest tests/ros/test_toss_coordinator.py tests/ros/test_toss_continuous_node.py -q`
  → **329 passed in 104.99 s**.
* `python -m pytest tests/ros/test_trajectory_node.py tests/ros/test_toss_sequencer.py -q`
  (2026-08-27) → **294 passed in 10.04 s**; the six `test_creach1_*` C-REACH-1
  tests pass untouched.
* `python -m pytest tests/ros/ -q -p no:randomly` (2026-08-27) → **2526 passed,
  1 skipped in 333.92 s** (re-run after the last test edit; an earlier identical
  invocation gave 2526 / 1 in 335.69 s).
* `python -m pytest tests/motion/ -q -m "not nightly"` (2026-08-27) → **1951
  passed, 3 skipped, 3 deselected in 308.01 s** — the aim/release/cal modules the
  bound is derived from live there.
* **Counterfactual runs**, because a test that passes both with and without a fix
  measures nothing. With the substitution disabled at its first branch
  (`if True or not cls._release_is_tilted(release)`), 2026-08-27:
  `test_an_aimed_colocated_reach_holds_the_pretilt_and_closes_the_chain` FAILS at
  the pre-tilt assertion (`(1.0, 0.0, 0.0, 0.0) != (0.99996611…)`), and
  `test_an_aimed_colocated_chain_stages_now_that_the_reach_holds_the_pretilt`
  FAILS at `tests/ros/test_toss_continuous_node.py:2752` — the **per-reach
  pre-tilt** assertion, `(1.0, 0.0, 0.0, 0.0) == (0.9999661115…, …)`: the reach
  published a LEVEL orientation where the fix publishes the throw's pre-tilt.
  That *is* carried finding 2 reproduced as a unit-test failure — the platform
  being un-tilted at `t_release` is the finding, and the staging failure is its
  consequence. Stated precisely because the first draft of this entry claimed
  the test fails with its both-slots-live sample **empty**, and it does not:
  that staging assertion is three statements further down (`:2757`) and is
  **unreachable** in the counterfactual run, because the pre-tilt assertion
  aborts the test first. The non-staging half is not left unpinned by that —
  `test_a_cycle_that_must_move_does_not_stage` pins "a cycle that must move
  cannot stage" independently, and is untouched by this change.
* Regime numbers re-derived from the live modules by
  `/tmp/probe_p4_reach_delta.py` (2026-08-27) and pinned in the suite by
  `test_the_displaced_regime_is_separated_by_more_than_the_bound`.

## Outcome

Carried finding 2 is closed and P-4 with it, so `plans/active/
toss-pipelined-preamble.md` § 6.1's "do not book any rung" gate is satisfied and
B6's rungs are bookable. On the shipped tier with an aim armed, a chained cycle
now takes the census-B1 skip (0.360 s per cycle, banked whatever the pipeline
flag says) and therefore STAGES once `toss_pipeline_enabled` is on — which is
what makes Phase B4's pipeline do anything at all in an ILC-primary sitting. The
flag itself still ships FALSE (plan § 9.5).

Deployment is a `colcon build --packages-select jugglebot` — this is node code,
not a `teensy_link` wire change, so the install space must be rebuilt before the
next sitting.

**Not measured, and stated so it is not assumed**: every cadence number in
`tests/hardware/session_cadence_ladder.md` was produced on the 8a-equivalent
(unaimed) path. The first aimed 8b sitting measures the aimed chain for the first
time rather than confirming a known number — and the ≤1° seat tilt now held
through the catch on those cycles is the by-eye item to watch (it is what aimed 8a
has always done, but it has never been scored on 8b).
