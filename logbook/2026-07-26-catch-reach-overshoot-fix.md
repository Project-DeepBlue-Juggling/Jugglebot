---
title: "C-CATCH-1 — the catch through-seat aims at the ball, not at the levelling correction, and the planner may not manufacture motion nobody asked for"
type: bugfix
date: 2026-07-26
status: in-progress
phase: "Self-toss anomaly fixes — catch-reach-degenerate-overshoot Phase 2"
related_plan: "catch-reach-degenerate-overshoot.md"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/docs/catch_arrival_contract.md
  - ros_ws/docs/levelling_frame.md
  - tools/probes/catch_reach_replay.py
  - tools/probes/levelling_tilt_bag_check.py
  - tools/probes/README.md
  - tests/motion/test_trajectory_planner_catch.py
  - tests/ros/test_levelling_frame.py
  - tests/hardware/session_anomaly_fixes.md
  - plans/active/catch-reach-degenerate-overshoot.md
  - plans/active/levelling-frame-contract.md
commits:
  - 407154f
subsystem:
  - motion
  - ros
tags:
  - safety
  - kinematics
  - testing
  - docs
---

# C-CATCH-1 — the catch through-seat aims at the ball, not at the levelling correction

## Summary

`planner.build_catch` read `catch_pose[3:5]` as "the receive tilt" and manufactured
a terminal arrival rate along it. That premise holds only while the commanded frame
IS the gravity frame; with a C-LEVEL-1 correction loaded, a gravity-level catch
arrives as a plan-frame tilt that *is* the correction — so the through-seat aimed
along the correction, the reach acquired a `+2.32°` swing opposite a `−0.78°`
request, and the platform settled `0.3008°` off gravity and held it through release
(16.5 mm of throw-direction error). The fix passes the gravity-referenced receive
tilt to `build_catch` as its own argument and lands contract **C-CATCH-1**
(`ros_ws/docs/catch_arrival_contract.md`): where the planner *derives* an arrival
twist rather than being handed one, no departure from the target it creates may
exceed `40/81` of the catch's physical tilt scale. A second, separately-committed
deliverable fixes the `peak_leg_*` predicted-peak staleness.

## Problem

Three measured features of every self-toss catch reach in
`~/Desktop/rosbags/2026-07-25_15-17-48`, reproduced offline in Phases 0–1
(`logbook/2026-07-25-catch-reach-overshoot-repro.md`):

1. a `+2.32°` commanded tilt excursion in the direction **opposite** the
   `−0.78°` target, peaking ~2.0 s before release;
2. a settle `1.385×` the requested tilt — `−1.078408°` against a `−0.778784°`
   target — leaving `0.3008°` of residual tilt versus gravity at ball contact,
   held *through release* by `hold_after=True`, worth `0.005250 rad × 3.93 m/s ×
   0.8 s = 16.5 mm` of throw-direction error against 16 mm measured;
3. an acceleration/jerk spike (`142.4 mm/s²` / `3950 mm/s³`) at release − 0.6 s,
   an order of magnitude above the reach's own `13.9 / 35`.

## Root Cause

One line, and it is a **frame premise buried inside the builder** where no call
site could see it or contradict it:

```python
tilt = target[3:5]          # "the receive tilt" — true only pre-C-LEVEL-1
```

`catch/dynamic_target` carries one orientation, and the node derived one quantity
from it. But two are needed. `target[3:6]` is where the legs must be *commanded*
(C-LEVEL-1-corrected, plan frame); the direction the through-seat must *aim* along
is where the ball is coming from (gravity frame, uncorrected). Once a levelling
correction is loaded those are different vectors, and for a gravity-level catch
the commanded tilt **is** the correction — so the seat aimed 100 % wrongly, at an
artefact of the levelling frame.

All three features follow from that single rate, quantitatively:

- feature 2 is `1 + frac·rate·decay/|tilt|` = `1 + 0.5·0.07·0.15/0.0136458` =
  **1.3847324**, against `1.078408/0.778784` = **1.3847228** measured — seven
  significant figures, a prediction rather than a bracket;
- feature 1 is `p(s) = d·ψ(s) + v1·T·φ(s)` with `max|φ| = 16/81` at `s = 2/3`,
  giving `+2.32391°` at `T = 3.7072 s` against `+2.3236°` (harness) and `+2.3224°`
  (bag, adjacent 25 ms knot);
- feature 3 is structurally the 0.15 s decay segment, and it disappears entirely
  post-fix because a zero arrival rate skips that segment.

C-LEVEL-1 made it **worse**, not better: it drives the requested tilt of a level
catch toward zero while the manufactured rate stays constant, and the
amplification goes as `1/|tilt|`.

## Discussion

### Why a contract and not a `|tilt| ≈ 0` guard

The cheap fix is to suppress the through-seat when the tilt is near zero. It was
rejected because it hides the amplifier instead of bounding it: the amplification
is `(16/81)·rate·T / |d|`, so *any* threshold leaves a band just above it where the
reach still swings several times its request, and post-C-LEVEL-1 that band is the
normal operating point rather than an edge case. The class of failure is "the
planner manufactures a boundary condition and nothing measures it against what was
asked for", and closing the class needs an invariant, one enforcement point and a
test — not a guard.

### Bound the rate, do not reject the plan

The violation is manufactured *by the builder itself*. Rejecting would turn a
perfectly reasonable caller request — a small genuine receive tilt at a long lead —
into `TrajectoryInfeasible` and **no catch at all**, which is strictly worse than a
rim that rotates through its seat more slowly. Every effect of the bound is a
reduction in commanded motion.

### The bound factor is geometry, and the repo's published one was wrong

`40/81 ≈ 0.4938` is `(16/81)·(5/2)`, where `5/2 = min_s ψ(s)/|φ(s)|` is the exact
ratio at which a rest-seeded quintic first leaves its seed on the far side from its
target. `ψ/|φ| = (10−15s+6s²)/(4−7s+3s²)` has derivative numerator `10−12s+3s²`
whose roots (1.18, 2.82) both lie outside `[0,1]`, so the ratio is strictly
increasing with infimum `5/2` at `s → 0⁺`. Measured over a 4M-point sweep:
`2.500000000625`; the wrong-side excursion is `0` at 2.50, `+1.6e-8` at 2.51,
`0.126·|d|` at 4.00.

**A published number in this repo turned out wrong, in the permissive direction.**
`plans/active/catch-reach-degenerate-overshoot.md` and the replay probe both stated
the sign reverses above `ψ(2/3) = 0.790`. That is where the value *at* `s = 2/3`
crosses zero, not where the reach first leaves the park — `0.790` corresponds to
`|v1|T/|d| = 4.0`, so it is **1.6× too permissive** and a gate sized off it would
pass a reach that has already reversed. Corrected in the contract, the planner
comment, both probes and the plan (with the original sentence left visible beside
the correction, per the plan's own convention). The corrected leads: the toss target
reverses beyond `0.487 s`, not 0.78 s; the reload target beyond `6.72 s`, not
10.75 s.

### The hypothesis that was withdrawn: the bound's denominator

This is the part worth reading. The bound was first sized against the **residual
seed → target tilt travel** — the natural reading of "the displacement the caller
requested". Three reviewers, from three different starting points (physics,
contract integrity, regression), independently converged on the same defect, and it
reproduced immediately.

On the shipping **reload** path the coordinator's open-loop safety net
(`catch_coordinator._republish_pretilt`) re-asserts the pre-tilt pose every balls
tick. `trajectory_node` releases the reach freeze at `arrival + settle_hold` and
re-latches it at `arrival − reach_freeze`, so a burst of further catch installs is
accepted in the last ~0.7 s — 9 and 11 of them in the reference bag, across
`landing−0.78..−0.31 s` and `landing−0.83..−0.29 s` — and the **last** of them is
the plan frozen through ball contact. Each is seeded already on the target, so the
residual travel is ~0.04° against a 10.87° seat.

Sized on that residual, the bound collapsed. Replaying the recorded burst through
the production planner, the arrival rate at contact fell **`0.070000` →
`0.004460 rad/s`** (4.011 → 0.256 °/s, a **15.7×** de-rate): a parked tilted rim at
the instant of contact, which is precisely the condition the through-seat exists to
prevent — and a parked tilted rim deflects the ball, the bb-sim geometry finding
the whole feature rests on. On the path carrying the session's 15/19 catch rate.
Nothing flagged it: the segment count stays 3 (the throttled rate is far above
`_REST_TWIST_EPS`), the shipped tests all seed from rest, and the replay probe
scores only the pre-tilt install — so the contract doc, the probe output and the
operator runbook *all three* asserted "does not bind" for a plan that binds hard.

The root cause of that second defect is not arithmetic, it is meaning. The reach
bound's physical reading is *"the reach must not leave its seed on the far side
from its target"* — which presumes there **is** a meaningful displacement. Once the
target *is* the seed, "away from the target" has no content: every nonzero arrival
velocity is wrong-side by definition, so the bound degenerates into "no arrival
velocity at all", in exactly the regime where its own justification has evaporated.

So the scale is the **larger** of the two quantities a catch is physically sized
by: how far the rim must travel, and how tilted it must end up. The second is what
justifies a nonzero arrival rate in the first place. The 2026-07-25 defect stays
closed by construction — its wire receive tilt was exactly zero, so the seat
magnitude is 0, the scale *is* the travel, and nothing about that case changes.

Two alternatives were considered and rejected:

- **Bound only where the seat direction was inferred (`receive_tilt is None`).**
  This would exempt the entire ROS path — the only caller that *has* a levelling
  concept and therefore the only one the contract was written for. It closes the
  supersede symptom by disabling the invariant where it matters most.
- **Accept the throttle and just document it.** Rejected because it changes
  commanded motion at ball contact on the shipping path as a *side effect* of
  fixing a different defect. The status quo (full seat at contact, and a ~0.9°
  commanded round trip in the last second) is what produced the known 15/19 catch
  rate; whether that round trip is good or bad is a bench question with an operator
  attached, not a finalizer's judgement call. Preserving it makes this phase a
  clean closure of the recorded defect and leaves the physical question open and
  *named* rather than silently answered.

### Why the settle overshoot is bounded too

The same rate manufactures two departures, and `decay` appears in neither the reach
bound nor the `_wrong_side_deg` metric the tests sample (a settle overshoot sits on
the toward-target side of the seed, so that metric scores it `0`). Yet the settle
residual is this contract's own headline evidence — the `0.3008°` that predicts the
16.5 mm throw error. Bounding only the reach would let a future seat-tuning session
raise `tilt_decay_s` from 0.15 s to 0.6 s and quadruple that residual with every
`test_ccatch1_*` still green. Both halves use the same `40/81`, so no second tuning
constant enters; at the shipped decay the settle half is slack by ~18× on the reload
and the shipped overshoot is unchanged at `0.3008°`.

### `receive_tilt=None` falls back rather than being mandatory

Making it mandatory would churn ~25 call sites to state a fact true by construction
at 24 of them — neither `sim/` nor `controller/` has any levelling concept, so there
the catch pose *is* gravity-referenced. The risk of a silent fallback (a future
levelled caller forgets) is closed instead by the AST manifest in
`tests/ros/test_levelling_frame.py`, which keys on the argument's **source text** at
both call sites, so passing the corrected tilt fails the test.

### An explicit rate stays unbounded

An arrival twist a caller passes explicitly is *requested* motion and is honoured
verbatim. This is the seam a future moving-platform catch needs, and it is what lets
the offline replay probes rebuild pre-fix captures — without it the probes would
score every capture recorded before today as NOT-REPRODUCED, an instrument failing a
working system. It also resolves the plan's open "does C-CATCH-1 enshrine a
stationary platform?" risk without any appeal to authority: the contract never
mentions stationarity. A blanket stationarity clause would mandate a parked rim just
as surely as the old constant mandated motion.

### Normative home

New `ros_ws/docs/catch_arrival_contract.md`, not
`controller/REFERENCE_LAYER_CONTRACT.md` — that document's scope is the MPC
reference layer and explicitly not `ros_ws`. C-LEVEL-1 already established
`ros_ws/docs/` as the home for ROS-package contracts.

## Fix

**C-CATCH-1** (commit `407154f`):

- `trajectory_node._catch_target_from_msg` now returns **three** quantities from
  the one wire orientation: `target` (C-LEVEL-1-corrected, where the legs are
  commanded), the twist, and `receive_tilt` (the raw wire rotvec's `rx, ry`,
  gravity-referenced and deliberately **uncorrected**).
- `planner.build_catch` takes `receive_tilt` as its own argument and aims the
  through-seat along it. A level receive tilt yields a zero arrival twist and a
  two-segment plan (reach + hold), by construction.
- `tilt_through_rate_radps` defaults to `None` (sentinel). Explicit = requested =
  verbatim; `None` = the planner's own constant = bounded.
- One enforcement point, `planner._catch_arrival_rate`, bounding **both** the reach
  excursion (`|v1|·T ≤ 2.5·scale`) and the settle overshoot
  (`frac·rate·decay ≤ (40/81)·scale`), where `planner._catch_scale` is
  `max(residual travel, receive-tilt magnitude)`.

**`peak_leg_*` diagnostics** — a separately-diagnosed bug surfaced by this
investigation, landed as its own commit with its own entry:
`logbook/2026-07-26-peak-leg-predicted-staleness.md`. Both were gated by the same
full-suite run below.

## Verification

**Pre-fix failure, verified by running rather than assumed.** With the production
files stashed, `test_ccatch1_bounds_the_unrequested_excursion_at_a_long_lead` and
`test_ccatch1_holds_across_the_lead_and_tilt_envelope` both fail **on their own
assertions**, not on a signature error: `2.3237 ≤ (40/81)·0.7819` and
`0.4302 ≤ (40/81)·0.0500` at tilt 0.05° / lead 0.6 s. The `2.3237°` the pre-fix code
produces is the number the bag recorded (`+2.3224°` at the adjacent 25 ms knot).

**The two tests added during finalize also fail against what they guard**, verified
by monkeypatch (2026-07-26):

- `test_ccatch1_keeps_the_seat_on_an_on_pose_supersede` → `0.000931 == 0.07` fails
  under the residual-only scale;
- `test_ccatch1_bounds_the_settle_overshoot_under_a_raised_decay` → `1.2032°` past
  the seat fails with the settle half removed.

**Full suite:** `pytest tests/ -q`, run 2026-07-26 on the Jetson in the project venv: **3543 passed, 3 xfailed in 1376.13 s (22:56)**

**Probe acceptance (two-sided), 2026-07-26.**
`tools/probes/catch_reach_replay.py --self-check` → `SELF-CHECK: PASS`, 10/10 `OK`,
exit 0 — including the two-sided C-CATCH-1 counterfactual and the case that
re-derives the bound factor from `min ψ/|φ|` each run so it cannot silently become a
tuning value. All **seven** recorded reaches reproduce (`--toss 1..5` and
`--thrower ball_butler --toss 1,2`) → `REPRODUCED`, exit 0.
`tools/probes/levelling_tilt_bag_check.py --self-check` → PASS.

**Measured effect on the reference bag.** Self-toss (×5): unrequested excursion
`2.3218–2.3340° → 0.0000°`; settle `rx −1.078408 → −0.778784` (the target exactly),
`ry −0.095775 → −0.069165`; segments 3 → 2; predicted peaks
`14.2/142.4/3950 → 1.4/1.2/3`. Reload (×2): seat survives, aim rotates `4.0997°`,
settle `+1.823550 → +1.844635` and `−10.933038 → −10.928741`, predicted acc/jerk
`139.7/3873 → 142.0/3935` (+1.6 %, three orders under the 5000/30000 session
ceilings). Seat rate at ball contact **unchanged** at `0.070000 rad/s`.

Note the +1.6 % rise: "the bound can only reduce commanded motion" is true of the
*rate*, but re-aiming the seat is a **rotation**, not a reduction, and it
redistributes the same rate across the tilt axes and hence across the legs. The leg
peaks can move either way.

**Known and accepted:** the two **advisory** `T = 0.95` Tier-8b spot checks in
`sim/toss_gate.py` bind (`0.070000 → 0.059469 rad/s`, settle overshoot
`0.3008° → 0.2555°`). Correct behaviour — there the pre-tilt at A leans opposite B's
seat, so travel (1.2948°) exceeds the seat (0.6484°) and the manufactured excursion
reaches 0.581 of the scale. Both points lie outside the binding 50 mm ring, so no
gate PASS band moves. Pinned with its number by
`test_ccatch1_clips_the_tier_8b_advisory_spot_checks`.

**Not yet verified on hardware.** `tests/hardware/session_anomaly_fixes.md`
§ Section CCATCH (CCATCH-1..5) carries the operator's commands and numeric
PASS/ABORT criteria. Requires `colcon build --packages-select jugglebot` **and a
relaunch** — the launch runs the installed copy, and a stale install reproduces the
pre-fix behaviour exactly. No firmware flash, no config regeneration.

## Related

- `plans/active/catch-reach-degenerate-overshoot.md` — the plan; Phase 2 Outcome.
- `logbook/2026-07-25-catch-reach-overshoot-repro.md` — Phases 0–1, the offline
  reproduction this fix is built on.
- `ros_ws/docs/catch_arrival_contract.md` — C-CATCH-1, normative.
- `ros_ws/docs/levelling_frame.md` — C-LEVEL-1, the sibling contract that made the
  premise false; its "what C-LEVEL-1 does NOT close" section now carries a dated
  update saying both items are closed.

## Open questions

1. **Is `_CATCH_TILT_THROUGH_RATE_RADPS = 0.07` still the right magnitude?** It is
   the conservative sim default its own docstring calls "a physical-tuning
   parameter the operator refines on hardware", it has never been validated on the
   machine, and until today its *aim* was wrong on every levelled catch — so any
   prior hardware impression of it was formed on a mis-aimed seat. Now that the aim
   is correct, the next reload sitting could sweep it.
2. **Should the reload supersede burst exist at all?** Characterised here with
   numbers for the first time: 9–11 accepted re-installs in the last 0.7 s, each
   re-engaging a fresh `0.30°` through-seat overshoot, producing a `≈0.9°` commanded
   round trip in the last second before contact. This phase deliberately preserved
   it (it is the status quo behind the 15/19 catch rate), but it is unrequested
   motion near contact and is worth an operator opinion.
3. **Pre-existing and untouched:** the other half of `mvp_bench_runbook.md` open
   item 7 — realized peaks on the SpaceMouse / reactive-catch path are per-install
   rather than rolling-window.
