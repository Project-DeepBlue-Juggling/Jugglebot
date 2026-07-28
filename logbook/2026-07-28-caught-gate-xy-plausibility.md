---
title: The CAUGHT possession verdict was structurally always False — C-POSSESS-1 closes the class
type: bugfix
date: 2026-07-28
status: tuned
phase: "Self-toss anomaly fixes — follow-on item B (caught-gate)"
related_plan: "PROMPT-anomaly-fixes-orchestration.md"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/docs/ball_possession_contract.md
  - tests/ros/test_ball_possession.py
  - tests/ros/possession_fixtures.py
  - tests/ros/test_reload_coordinator_node.py
  - tools/probes/possession_verdict_bag_check.py
  - tools/probes/README.md
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_phase7_reload.md
  - tests/hardware/session_phase8_toss_hardware.md
  - plans/active/PROMPT-anomaly-fixes-orchestration.md
  - plans/active/catch-reach-degenerate-overshoot.md
  - plans/active/single-ball-toss.md
  - plans/active/levelling-frame-contract.md
commits:
  - 463a031
subsystem:
  - ros
  - tracking
tags:
  - testing
  - docs
  - safety
---

# The CAUGHT possession verdict was structurally always False — C-POSSESS-1 closes the class

## Summary

Every ball operation the machine has ever run reported `MISSED`. The coordinator's
CAUGHT gate ANDed two spatial bounds and the vertical half, `z_err <= 150 mm`, was
**unsatisfiable by a real catch** — so `success` was False by construction on every
toss and every reload. The z bound is deleted (retained as a REPORT-only
diagnostic, forever), the horizontal bound is single-sourced from the catching
aperture, and the whole verdict moves behind a normative contract
(**C-POSSESS-1**, `ros_ws/docs/ball_possession_contract.md`) with one enforcement
point and a pluggable source seam. Self-toss verdicts go **0/17 → 17/17** on the
2026-07-27 sitting's own data; all 18 corrupt reload tracks stay refused, which is
the honest verdict because none of them carries real evidence.

## Problem

On the 2026-07-27 validation sitting all **17** self-tosses and all **18** scored
reloads returned `success = False`, while the operator watched 17 catches land and
a mocap floor census independently confirmed 13 of 16 reload catches. This had
taxed every sitting with judge-by-eye scoring and blocked `toss_continuous`, whose
`stop_on_miss` and per-cycle accounting need a real verdict.

## Root Cause

Not "150 mm was too tight". The defect was **a plausibility bound applied to an
observable whose error model was never written down.**

`tracking/matcher.py::_check_lifecycle` declares a ball CAUGHT *because its mocap
marker disappeared* around the predicted landing time. So `BallState.position` at
that instant is not an observation — it is the Kalman filter's dead-reckoned
free-fall extrapolation from the last real sighting, then frozen for the ~2 s the
terminal track is retained (`_associate_confirmed_balls` skips non-`IN_FLIGHT`
balls, `matcher.py:261`; measured positional variance after the first CAUGHT sample
is **exactly 0.000 mm** on all 60+ CAUGHT tracks in the bag).

Under free fall that extrapolation error splits by axis:

| axis | error term | measured, 17 self-toss catches |
|---|---|---|
| z | `\|v_z\|·dt + g·dt²/2` | **305 – 1007 mm** |
| xy | `\|v_xy\|·dt` | **0.30 – 3.88 mm** |

**z carries the artefact; xy carries the information.** A bound on z is a bound on
how long the tracker had already lost sight of the ball — i.e. on the very thing
CAUGHT *means*.

A **second instance of the same class was live in the same expression**: the
`200 mm` xy bound sat **4.9 mm** under the corrupt-track floor the next sitting
measured (204.9 mm) — a **1.02x** margin against minting a *false* CAUGHT. One half
unreachably tight, the other one dead-reckoned track away from unreachably loose,
both written the same day from the same absence of an error model.

## Discussion

### Why a contract, not two new numbers

Re-tuning `150` and `200` would have left the third instance of the class to be
found on hardware. C-POSSESS-1 instead requires that **every bound a source applies
is accompanied, in the source, by the error model of the observable it is applied
to**, and that an observable whose error model makes it non-discriminating is
**REPORT-ONLY**. That is a rule which, had it existed on 2026-07-23, would have
prevented the z bound from being written at all. The three parts landed together:
the normative document, one enforcement point (`ball_possession.py`, reached only
through the node's single `_possession_confirmed` seam), and a test file scored on
35 *measured* CAUGHT estimates rather than synthetic ones.

### The fate term the brief specified, and why it did not ship — an explicit deviation

The brief specified a **two-part** verdict — arrival AND a post-arrival
persistence/fate term — precisely so the gate would not carry the false positive
the sitting had already demonstrated (a ball that enters the cup, is minted CAUGHT,
then bounces out). **What shipped is arrival-only.** This is a real deviation and
it is recorded in contract § 7 rather than left to be inferred.

Two candidate fate terms were measured and rejected:

1. **Dead-reckoning depth as a confidence signal** — separates nothing. Self-toss
   305–1007 mm vs reload 711–1514 mm, and ball 123 (a true catch) sits at 1007
   while ball 32 (also a true catch) sits at 1096.
2. **Lateral velocity at CAUGHT** — and this one nearly shipped. It separates the
   data **75x** (self-toss 0.8–7.0 mm/s vs reload 530–1124 mm/s). It was rejected
   because *that separation is geometry, not retention*: a self-toss arrives
   vertically so its lateral velocity is ~0 by construction, while a **legitimate**
   reload catch arrives from the Butler at ~1 m/s and the same threshold would
   refuse it. It would have looked validated on 17 fixtures and silently
   false-negatived every reload catch the moment the tracker was fixed.

A third — post-CAUGHT track evolution — is genuinely unobservable, and that is
structural rather than incidental: the track freezes and is pruned.

**A hypothesis withdrawn during finalize.** The original justification went
further, claiming flatly that *"a bounce-out raises no successor track … There is
nothing for it to observe"*, on the strength of zero new `/balls` tracks appearing
between each 2026-07-27 bounce-out's CAUGHT and its floor arrival. **That
measurement is confounded and the claim was narrowed.** Those three tracks were
mis-associated: the real marker was *already* carried by a separate untagged track,
so no *new* track could appear — the probe asked a question the corruption had
already answered. In a healthy tracker the CAUGHT track goes terminal, stops
consuming its markers, and they fall through to `_detect_parabolic`
(`matcher.py:220`, `parabolic_min_frames = 3`) — a real successor channel that was
never evaluated. Two further signals are likewise unevaluated: `/mocap_data`
(200 Hz unlabelled markers, which the offline floor census already uses; the
coordinator does not subscribe to it), and the matcher's own CAUGHT precondition,
which is *itself* weak retention evidence — `matcher.py:575-582` mints CAUGHT only
after ≥ 10 consecutive missed frames and explicitly lets a still-visible ball
continue (*"it might be bouncing"*), so a bounce-out that stays in view never mints
CAUGHT at all. The honest statement is **"unobservable from the terminal track"**.

The `/mocap_data` option was the strongest and was still declined, on a concrete
failure mode rather than cost: it needs a verdict delay of ~0.5 s spent inside
`catch_confirm_window_s` (0.70 s) and the `_toss_deadline_s` budget — **a budget
whose timeout path is `SAFE_ABORT`, i.e. it retracts the hand under a seated ball.**
Buying retention evidence by moving the toss closer to a retract-under-ball timeout
is a bad trade on the one path where the cost is a hazard. The ball-in-cup hand
sensor (installed 2026-07-28) observes retention directly and continuously, and is
what closes this properly.

### The arrival threshold, and a claim that finalize measured and refuted

The bound became `float(hw.GEOM_ARM_RADIUS_MM)` = 70 mm — the catching structure's
entry aperture, single-sourced so a geometry change moves it. It clears the
true-catch maximum (3.88 mm) by **18x** and sits **2.9x** under the corrupt floor.

The source's first draft stated the reload-path risk as *"derived from the session
geometry, not measured — no reload track in the capture carries real velocity to
measure it from, which is the same reason the risk is moot today."* **Finalize
measured it, and that sentence was wrong.** Scanning `/balls` for reload-era
*untagged* tracks (the ones carrying the real marker, i.e. how a tagged track will
read once the mis-association is fixed) that reach CAUGHT near the catch point:

    ids 57 / 33 / 69 / 15  ->  34.4 / 34.9 / 37.6 / 68.4 mm

So a genuine reload catch already sits **1.6 mm inside** the bound — a **1.02x**
margin, *the identical shape of the defect this contract was written to close*. A
second, independent term pushes the same way: `JB_TRAJ_CATCH_REACH_ENVELOPE_MM` is
**80 mm** and the reference point does not move with the reach, so a reached-to
catch can read up to the full envelope even with a perfect estimate. Three
reviewers reached this from three different starting points (dead-reckoning drift,
measured untagged tracks, and the reach envelope), which is why it was verified
first.

**It was deliberately NOT re-tuned.** Re-tuning needs data from a *fixed* tracker,
which does not exist; the only number available today is the 204.9 mm corrupt
floor, an artefact of the very bug being fixed, and sizing a bound against a bug is
this contract's own § 1 defect one level down. The risk is inert today — every
tagged reload track is refused at 204.9–752.9 mm whether the bound is 70 or 200 —
so nothing observable changes until the tracker phase lands, which is exactly the
phase that must re-derive it. The obligation is pinned by
`test_the_measured_reload_band_sits_against_the_bound` and carried at the bench by
row `POSS-1.6`.

One suggested alternative was measured and **refuted**: judging on
`landing_position.xy` instead of `position.xy` is *worse* for these tracks (ids 33
and 69 read 140.0 and 120.3 mm there, against 34.9 and 37.6 mm) — the ballistic
projection amplifies the velocity error it was meant to dodge. A second suggestion,
tightening to `GEOM_HAND_RADIUS_MM` (35 mm), was refuted for pulling the wrong way
against a *measured* under-sizing.

### The handed-off tracker signature was factually wrong

The contract's § 4 originally described the open tracker investigation's signature
as *"every `destination='jugglebot'` reload track is a track whose Kalman filter
received **no** measurements at all (its lateral velocity is constant across the
whole descent)"*, repeated across five artefacts. **Finalize re-scanned the bag and
the claim is false.** All 18 tagged tracks reach `tracking=CONFIRMED`, which
`matcher.py:344-347` sets *only* inside `kf.update(marker)`; and their in-flight
`velocity.x` spread is **31.2 – 515.9 mm/s**, not constant (an open-loop track would
hold the announced launch `vx = 1032.2 mm/s` exactly).

The filters are **not starved — they are mis-fed.** The mechanism to start from is
the adaptive announced-ball gate at `matcher.py:325-327`,
`threshold = base + speed*0.05 + t_since_throw*50`, capped at 400 mm, which widens
with time-since-throw until it can capture a near-stationary marker beside the
Butler. This mattered enough to fix in all five places: the next session would have
hunted for why a filter was starved of measurements, which is not the defect. The
operational conclusion (refuse all 18) is undisturbed.

### Decisions taken deliberately, with the failure mode each prevents

- **z retained as REPORT-only `plane_drop_mm`, not deleted outright** — it is the
  single most useful diagnostic a verdict carries (how much of it is
  extrapolation). `test_plane_drop_is_report_only_and_cannot_veto` uses the
  deepest-extrapolated real catch in the session (ball 123, 1007 mm) so any
  reintroduction of a z bound *in any form* goes red.
- **The possession LATCH keeps arrival-only semantics.** Requiring retention would
  leave it permanently dead until the sensor lands, delivering nothing; and turning
  on a precondition that can *refuse a goal* is a behaviour decision belonging to
  whoever validates the sensor (sitting decision row (e)).
- **No verdict delay added.** The only fate evidence a delay could gather is
  post-CAUGHT track evolution, which is provably frozen — so a delay buys zero
  information while spending latency in the budget whose timeout retracts under a
  ball.
- **The seam's limits are documented rather than redesigned.** All three call sites
  sit inside a `status == CAUGHT` guard, so the swap is a *filter* on a tracker
  CAUGHT, not an origination point: a source cannot originate a claim, answer late,
  or clear the latch. The earlier flat "a new source changes no call site" promise
  held only for the arrival-filter role and would have misled the sensor phase;
  contract § 3 now states the three consequences it inherits. Hoisting the query
  into the tick is design work with actuation implications and belongs to that
  phase, not to a finalize pass.
- **The `describe()` bound wart is documented, not coded around.** The log line
  quotes the node's module constant rather than the source's own bound — correct
  only while the tracker source is installed. Carrying the bound in the verdict
  would be cleaner, but it changes a NamedTuple's shape for a wart that cannot fire
  until a second source exists, and the sensor phase rewrites `describe` anyway.
  Contract § 3 carries it as a named obligation.

## Fix

- **New pure module** `ros_ws/src/jugglebot/jugglebot/ball_possession.py` (no ROS,
  no config imports): `PossessionVerdict` (arrival + retention + both diagnostics),
  `PossessionSource` protocol, `TrackerArrivalSource`, `lateral_miss_mm`,
  `describe`. `confirmed = arrival_ok AND retention != REJECTED`, so an `UNKNOWN`
  retention cannot veto a catch the source *did* observe arriving — otherwise the
  contract would recreate the original defect inverted.
- **`_CAUGHT_MAX_Z_ERROR_MM` deleted**; `_CAUGHT_MAX_XY_ERROR_MM` becomes
  `float(hw.GEOM_ARM_RADIUS_MM)`.
- **`_caught_is_plausible` → `_possession_confirmed`** (semantics changed from one
  spatial test to a two-part verdict), routing all three consumers through
  `self._possession_source`. `_implausible_logged_ids` → `_possession_logged`, now
  keyed by `(ball_id, verdict)` so the *accepted* line is emitted too — it is what
  the operator scores gate-against-eye with.
- **`_catch_error_from_ball` routes through `lateral_miss_mm`**, so the number the
  gate decides on and the number the operator reads are one computation.
- **Contract** `ros_ws/docs/ball_possession_contract.md` (C-POSSESS-1), including
  § 4's corrected tracker signature and named residuals, § 5's two un-measured
  residuals on the newly-live `RECENTER` path, and § 7's explicit fate-term
  deviation.
- **Committed instrument** `tools/probes/possession_verdict_bag_check.py`, with a
  two-sided `--self-check` and `--emit-fixtures` regeneration.
- **Operator documentation repaired where this change broke it**: three greps in
  `session_phase7_reload.md`, three unqualified "verdicts may read MISSED" caveats
  in `session_phase8_toss_hardware.md`, standing rule 3 and three further
  unqualified statements in `session_anomaly_fixes.md`, plus four stale claims
  across three plan documents.

### The one behavioural consequence

The FSMs' `obs.ball_caught` branch has been dead on hardware for the machine's
whole life and becomes live. A caught self-toss now terminates at the CAUGHT tick
(landing + 0.202–0.442 s, median 0.209 s) with `RECENTER` instead of at the settle
deadline (0.70 s) with `SAFE_ABORT`. No commanded magnitude, feedforward term, CAN
encoding or planner call changes. Two deltas, both checked against the bag:

1. **The hand retract disappears.** It cannot interrupt a moving hand — `pos_meas`
   at the CAUGHT instant is within **±0.045 rev** on all 17, where 0.3 s earlier 7
   of 17 were still descending through 0.30–3.10 rev — and it does not strand the
   next goal: worst excursion over the following 3 s is **0.069 rev** against a
   ±0.5 rev park band (**7.2x**).
2. **`catch/prime_hold` releases 0.26–0.50 s earlier.** Not a new hazard class:
   today's `SAFE_ABORT` releases the same hold at the settle deadline with the ball
   equally seated. Both terminals keep `armed False` strictly before the release.

**Two residuals on this path are un-measured and go to the bench rather than being
argued away**: `RECENTER` is the one terminal with no telemetry-verified hand step
(rows POSS-2.1/2.2), and the ball's settle time before `go_home` was argued from
hand `pos_meas`, which cannot see the ball (row **POSS-2.4**, which retires that
assertion). A zero-risk decoupling — resolve the verdict early, hold `go_home` to
the old deadline — is pre-written in contract § 5 if the operator wants it.

## Verification

**Full suite**: `source ~/Desktop/PDJ_venv/venv/bin/activate && python -m pytest
tests/ -q`, run 2026-07-28 on the Jetson in the project venv: **4059 passed,
3 xfailed, 198 warnings in 1405.75 s (0:23:25)**, exit 0.

Delta against the `a5972ec` baseline (`pytest tests/ -q`, run 2026-07-28:
**3967 passed, 3 xfailed in 1396.74 s**) is **+92 passed**, accounted for exactly:
`tests/ros/test_ball_possession.py` contributes **53** (17 + 18 parametrised legs,
18 plain — including finalize's `test_the_measured_reload_band_sits_against_the_bound`)
and the `test_reload_coordinator_node.py` additions contribute **39** (17 + 18 + 4).
53 + 39 = 92. **xfailed unchanged at 3** — no test was weakened, skipped, deleted or
xfailed, and no skip/xfail marker was added.

**Mutation runs** (each reverted, tree re-verified green after each), 2026-07-28:

| mutation | result |
|---|---|
| M1 — delete the arrival bound (`ok = True`) | 44 failed |
| M2 — source returns `RETENTION_CONFIRMED` | 4 failed, incl. the retention pin |
| M3 — reinstate `\|drop\| <= 150` | 40 failed |
| M4 — loosen the tolerance to 200.0 | the drift guard fails |

M4 initially broke only **one** test, which exposed that the margin assertions were
reading a test-local constant instead of the shipped one; they were moved onto
`_CAUGHT_MAX_XY_ERROR_MM` and M4 re-verified.

**Instrument acceptance, two-sided** (an instrument validated only against the
broken case scores a working fix as a failure at the bench):
`python tools/probes/possession_verdict_bag_check.py --self-check` →
`SELF-CHECK: PASS 8/8`, exit 0, run 2026-07-28 — the ACCEPT side (17/17), the
REFUSE side (0/18), >10x separation, the bound strictly between, the retention pin,
a 2026-07-23 second-session negative, and two drift guards mirroring the node's
tolerance and catch point. Scoring the reference bag: **17 CAUGHT / 35 tagged
tracks, 18 refused.**

**Finalize's independent bag verification** (read-only, `2026-07-27_15-39-38`):
the tagged reload tracks' `tracking` values and in-flight `velocity.x` spread
(refuting the "no measurements" signature); the reload-era untagged tracks' arrival
errors (34.4/34.9/37.6/68.4 mm) and their `landing_position` counterparts
(140.0/120.3 mm, refuting that alternative).

**Not deployed.** `colcon build --packages-select jugglebot` **+ relaunch** are
required — the launch runs the *installed* copy, so without the relaunch the
machine reproduces the old always-MISSED behaviour while the repo, the tests and
the contract all say otherwise. No `jugglebot_interfaces` rebuild, no config
regeneration, **no firmware flash**. `ball_possession.py` is a **new module**, which
is the one shape that can land half-applied from a cached build, so pre-flight
**PF-7** was added to stage 3 (before the capture) rather than relying only on the
§ SECTION POSS checks read at scoring time.

## Open items

- The tracker split-track **mis-association** is still open and is not this entry's.
  Corrected signature and the gate to start from: contract § 4.
- The **70 mm bound is knowingly under-sized for the reload path** and must be
  re-derived by the tracker phase (contract § 4, row `POSS-1.6`).
- **No retention term** until the ball-in-cup sensor lands (contract § 7, row
  `POSS-1.2b` — deliberately a REPORT row, not an ABORT).
- Unrelated defect, recorded not fixed: the reload coordinator scores a
  **BB-aborted throw as `MISSED`** (goals 9 and 18, balls 31 and 62 —
  `THROW_ABORTED_NOT_SETTLED`, so no ball ever flew). It pollutes every catch-rate
  denominator.
- `_possession_logged` is mutated from the subscription and action threads without
  `_lock`. `set.add` is atomic under the GIL so the worst case is a duplicate log
  line; this is the pre-existing pattern and was not changed.
