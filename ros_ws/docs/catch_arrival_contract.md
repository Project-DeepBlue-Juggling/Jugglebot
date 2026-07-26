# The Catch Arrival Contract — C-CATCH-1

**Normative.** This document specifies what a catch plan's *arrival boundary
conditions* may contain, and — the load-bearing half — where they may come from.
It is the written third of the repo's contract pattern (normative statement + one
enforcement point + a test that fails without it); the other two are
`jugglebot/motion/trajectory/planner.py::_catch_arrival_rate` and
`tests/motion/test_trajectory_planner_catch.py`'s `test_ccatch1_*` block.

Scope: `planner.build_catch` and its callers. `build_timed`, `build_follow`,
`build_move` and the chase path all arrive at a **caller-supplied** twist and
none of them manufacture one, so none of them can violate this contract; the
catch path is the only place in the motion layer where the planner invents a
boundary condition. (Verified 2026-07-26 by reading every `planner.build_*`
signature; the AST manifest in `tests/ros/test_levelling_frame.py` fails if a new
planner entry appears without being declared.)

Sibling contract: `ros_ws/docs/levelling_frame.md` (C-LEVEL-1). The two are
tightly coupled and the coupling is the whole story below.

## Why this exists — the failure it closes

On the self-toss session of 2026-07-25 15:17:48 the commanded platform pose (FK
of `/leg_setpoint_echo`) did something a rest-to-rest reach cannot do. The catch
target asked for `rx = −0.77878°`. The platform swung to **+2.32°** — the
*opposite* direction, 3.0× the requested magnitude — peaking 2.0 s before
release, then settled and held at `−1.0784°`, which is **1.385× the target** and
matches neither a single nor a double application of the levelling correction.

All three features are one plan doing exactly what `build_catch` specified, and
the mechanism is a single premise that stopped being true:

> `build_catch` read `catch_pose[3:5]` as "the receive tilt".

That holds only while the commanded frame **is** the gravity frame. Once
C-LEVEL-1's levelling correction is loaded, a *gravity-level* catch arrives as a
non-zero **plan-frame** tilt — the correction itself — so the through-seat
residual aimed along the correction, and the reach acquired an arrival rate
nobody asked for. Given a specified arrival rate `v1`, a rest-seeded quintic is
exactly

```
p(s) = p0 + d·ψ(s) + v1·T·φ(s)
ψ = 10s³ − 15s⁴ + 6s⁵     (rest-to-rest, monotone 0 → 1)
φ = −4s³ + 7s⁴ − 3s⁵      (arrival-rate basis, extremum −16/81 at s = 2/3)
```

so a *negative* arrival rate drives a *positive* excursion of `(16/81)·|v1|·T`
before turning back. The amplification is

```
peak unrequested excursion / requested displacement = (16/81)·rate·T / |tilt|
```

which depends on the tilt **direction** and the lead but **never on the tilt
magnitude** — so it is inversely proportional to how much tilt was asked for.
Measured through `tools/probes/catch_reach_replay.py` on that one session:
**3.76** for the 0.78° toss pre-tilt (3.71 s lead) against **0.17** for the
11.08° reload pre-tilt (2.37 s lead). One code path, 22× apart.

The bug was not a wrong constant. It was a **frame premise buried inside the
builder**, where no call site could see it or contradict it. That is the class
this contract closes.

### Why the levelling fix made it worse, not better

C-LEVEL-1 drives the *requested* tilt of a level catch toward zero, and the
amplification goes as `1/|tilt|`. Post-C-LEVEL-1 the same session's reach swung
**+2.9219°** above the park, against **+2.3204°** before. Removing the
displacement while leaving the manufactured arrival rate is strictly the wrong
half to remove first.

## C-CATCH-1

> **A catch plan may not command motion nobody asked for.**
>
> Every boundary condition of a catch plan shall be either supplied by the
> caller (the seed state, the target pose, an explicitly specified arrival
> twist) or derived from a caller-supplied **physical** quantity (the
> gravity-referenced receive tilt). Where the planner *derives* an arrival
> twist rather than being handed one, **no departure from the target that twist
> creates** — the excursion during the reach, or the overshoot past the seat
> during the decay — shall exceed **`40/81`** of the catch's physical tilt
> **scale**.
>
> The scale is the LARGER of the two quantities a catch is physically sized by:
> the tilt displacement requested of this plan (`|p1 − p0|`), and the
> receive-tilt magnitude the ball asks the rim to hold. For the reach half that
> is equivalently `|v1|·T ≤ (5/2)·scale`.
>
> Motion the caller asked for is not bounded. Motion the planner invented is.

Two consequences, both deliberate:

* **An arrival twist the caller supplies is requested motion**, is measured into
  the bound rather than against it, and is honoured verbatim. A future planner
  that decides from an optimisation that a *moving* platform catches better may
  say so and will be obeyed. Platform motion during a catch or a throw is
  **permitted**; what this contract forbids is *mandating* it from a constant in
  the trajectory builder.
* **An arrival twist `build_catch` derives from its own module constant is not
  requested motion**, so the 2026-07-25 plan fails the bound — which is the
  point.

### The bound factor is geometry, not tuning

`40/81 ≈ 0.4938` is not fitted to the recorded case. It is the exact ratio at
which a rest-seeded quintic *first leaves its seed pose on the far side from its
target*:

```
p(s) leaves p0 away from p1  ⟺  |v1|·T / |d| > min_s ψ(s)/|φ(s)|
ψ/|φ| = (10 − 15s + 6s²)/(4 − 7s + 3s²), strictly increasing on [0,1),
        infimum 5/2 as s → 0⁺
```

and the excursion is `(16/81)·|v1|·T`, so `(16/81)·(5/2) = 40/81`. Measured
2026-07-26: `min_s ψ/|φ| = 2.500000000625` at `s = 1e-9`; at `|v1|T/|d| = 2.50`
the wrong-side excursion is `0`, at `2.51` it is `+1.6e-8`, at `4.00` it is
`0.126·|d|`.

So the bound has a physical reading with no free parameter in it: **whenever the
reach's own travel is the catch's dominant scale, the platform never moves away
from the catch target.** That covers every from-rest install — the case where
"away from the target" has content at all. Where the *seat* is the dominant scale
(an on-pose supersede, § below) the reading weakens to "the departure stays under
`40/81` of the seat angle", because once the target *is* the seed, every nonzero
arrival velocity departs by definition and the strict reading has nothing left to
say.

> **Correction to an earlier claim, kept because a future reader will meet it.**
> `plans/active/catch-reach-degenerate-overshoot.md` and the replay probe both
> stated that the commanded tilt "leaves the park in the wrong direction once
> the ratio exceeds `ψ(2/3) = 0.790`". That is where the value **at `s = 2/3`**
> crosses zero, not where the reach first leaves the park — the first crossing
> is at `40/81 = 0.4938`, verified numerically above. Anyone sizing a threshold
> off 0.790 would permit an excursion that has already reversed.

## Enforcement: `_catch_arrival_rate`

One function, in `motion/trajectory/planner.py`, is the only place a catch
plan's arrival-twist magnitude is decided:

```python
def _catch_scale(seed_tilt, target_tilt, seat_mag):
    return max(|target_tilt − seed_tilt|, seat_mag)     # the catch's physical size

def _catch_arrival_rate(seed_tilt, target_tilt, seat_mag, duration_s, decay_s,
                        requested_rate):
    if requested_rate is not None:
        return max(float(requested_rate), 0.0)          # requested → verbatim
    scale = _catch_scale(seed_tilt, target_tilt, seat_mag)
    return min(_CATCH_TILT_THROUGH_RATE_RADPS,
               _CATCH_ARRIVAL_RATE_BOUND * scale / duration_s,        # reach half
               _CATCH_EXCURSION_FRAC_BOUND * scale                     # settle half
               / (_CATCH_TILT_OVERSHOOT_FRAC * decay_s))
```

### The manufactured rate SHIPS AT ZERO (operator decision, 2026-07-26)

`_CATCH_TILT_THROUGH_RATE_RADPS = 0.0`. The planner manufactures nothing, so
`_catch_arrival_rate` returns `0.0` for every caller that has no opinion, and
**the bound below does not bind on anything the robot currently runs.** Every
catch is reach + quiescent hold; the rim is stationary at ball contact and
through the throw that follows it.

Read this correctly, because the distinction is the whole point and it is easy to
collapse:

* This is **not** a stationarity clause, and C-CATCH-1 still contains none. The
  contract's subject is where a boundary condition *came from*, never whether the
  platform is moving. Platform motion during a catch or a throw remains
  **permitted**; it is simply not **mandated** by a constant in the trajectory
  builder. Setting the manufactured fallback to zero is the cleanest possible
  statement of "the builder has no opinion".
* The caller seam is **unchanged and load-bearing**. An explicitly-supplied
  `tilt_through_rate_radps` is still returned verbatim and unbounded, still
  produces the decay segment and the settle overshoot, and is still what a future
  optimising planner uses to command a deliberately moving rim. Neither the
  parameter nor the decay code path may be removed as "dead code at the default" —
  removing them would convert this default into a mandate.
* **The bound stays live because a default is a value, and values move.** The
  seat-tuning session `_CATCH_TILT_THROUGH_RATE_RADPS`'s own docstring anticipates
  is exactly the moment a manufactured rate reappears — and the 2026-07-25 failure
  was a manufactured rate outrunning the request by 3.76×. A bound deleted on the
  day it stopped binding is a bound that is absent on the day the value returns.
  `tests/motion/test_trajectory_planner_catch.py` therefore reaches the bound by
  **monkeypatching the module constant** (`_set_seat_rate`), never by passing
  `tilt_through_rate_radps` — which would take the deliberately-unbounded requested
  branch and prove nothing.

**The physical risk this accepts, stated rather than solved.** The 0.07 rad/s
existed because *a parked tilted rim deflects the ball* (the bb-sim geometry
finding). A gravity-level catch seats level, so zero costs it nothing — under this
contract its receive-tilt magnitude is already zero and the seat never engaged.
The **reload** catch is the one that pays: it seats at 11.08°, which is precisely
the geometry that finding concerns, and its rim is now parked at contact. Two
facts bound the risk without removing it: 0.07 rad/s has never been validated on
hardware, and until commit `407154f` its aim was wrong on every levelled catch, so
no bench impression of it was formed on a correctly-aimed seat. Scored at the
bench by `tests/hardware/session_anomaly_fixes.md` § Section ZSEAT; a reload
seating or bounce-out failure routes back here.

### Why the scale is a MAX, and not the requested displacement alone

This is the one part of the contract that was got wrong first and corrected by
measurement, so the reasoning is recorded rather than just the result.

The reach bound's physical reading is *"the reach must not leave its seed on the
far side from its target"*. That reading presumes there **is** a meaningful
displacement. On the shipping reload path there is not:
`catch_coordinator._republish_pretilt` re-asserts the pre-tilt pose every balls
tick, and `trajectory_node` releases the reach freeze at `arrival + settle_hold`
and re-latches it at `arrival − reach_freeze` — so a burst of further catch
installs is accepted in the last ~0.7 s, each seeded **already on** the target
(residual travel ~0.04°, against a 10.87° seat), and the last of them is the plan
frozen through ball contact.

Sized on that residual alone, the bound collapses exactly where its own meaning
evaporates: once the target *is* the seed, every nonzero arrival velocity is
"wrong-side" by definition. Measured 2026-07-26 by replaying the recorded burst
through the production planner, the arrival rate at contact fell **`0.070000` →
`0.004460 rad/s`** (4.011 → 0.256 °/s, a **15.7×** de-rate) — a parked tilted rim
at the instant of contact, which is the condition the through-seat exists to
prevent, on the path with the session's 15/19 catch rate. Nothing flagged it: the
segment count stays 3, and the replay probe scores only the pre-tilt install.

With the seat magnitude in the max, the reload keeps its full seat (scale 10.87°,
bound `0.200 rad/s` ≫ the `0.07` rate then shipped) and the 2026-07-25 defect stays
closed by construction — its wire receive tilt was exactly **zero**, so `seat_mag`
is 0, the scale *is* the displacement, and nothing about that case changes. Pinned
by `test_ccatch1_keeps_the_seat_on_an_on_pose_supersede`, which fails against a
residual-only bound.

*(All the rates in this subsection are the 2026-07-26 sizing measurements, taken
at the then-shipped `0.07`. The manufactured rate is now `0.0` — see above — so
none of them is a live number; the `max` is retained unchanged because it is what
makes the bound correct **when** a rate returns, which is the only time it
matters.)*

### Why the settle half exists

The same rate manufactures two departures, and `decay` appears in neither the
reach bound nor the `_wrong_side_deg` metric the tests sample (a settle overshoot
sits on the TOWARD-target side of the seed, so that metric scores it `0`). Yet the
settle residual is this contract's own headline evidence: `0.3008°` off gravity,
held *through release* by `hold_after=True`, predicting the session's 16.5 mm
throw-direction error. Bounding only the reach would leave a seat-tuning session
free to raise `tilt_decay_s` from 0.15 s to 0.6 s and quadruple that residual with
every `test_ccatch1_*` still green. Both halves use the same `40/81`, so no second
tuning constant enters the contract. At a `0.07` rate and the shipped 0.15 s decay
the settle half is slack by ~18× on the reload and never binds before the reach
half, leaving the overshoot at `0.3008°`. **At the shipped `0.0` rate the overshoot
is identically zero** and neither half binds — and the seat-tuning session that
raises the rate and then reaches for `tilt_decay_s` is precisely the situation this
half exists for. Pinned by
`test_ccatch1_bounds_the_settle_overshoot_under_a_raised_decay`, which restores a
non-zero manufactured rate to get there.

`build_catch` aims that rate along `receive_tilt` — the **gravity-referenced**
receive tilt, passed as its own argument — and derives the settle pose from the
same direction. A level receive tilt yields a zero arrival twist, hence a reach
with no excursion at all and no through-seat decay segment, **by construction**
rather than by a `|tilt| ≈ 0` threshold. A threshold would have to be tuned, and
would fail exactly in the near-degenerate band where the amplification is worst.

### Why bound the rate rather than reject the plan

The violation is manufactured *by the builder itself*, so a rejection would
refuse a catch the caller had requested perfectly reasonably — and no catch at
all is strictly worse than a rim that rotates through the seat more slowly.
Every effect of the bound is a **reduction** in commanded motion, and it is a
no-op wherever the catch's own scale already justifies the motion — the shipping
reload sits at 0.174 of its scale, well under 0.494, and came out at the full
manufactured rate at **both** the pre-tilt install and every on-pose supersede
through ball contact when that rate was `0.07`. (It is `0.0` today, so the bound is
a no-op there for the trivial reason as well; the sentence records that the bound
was never what removed the reload's seat.)

Note "reduction in commanded motion" is a statement about the *rate*, and it does
not extend to the aim. Re-aiming the seat from the plan-frame tilt to the
gravity-referenced one is a **rotation**, not a reduction, and it redistributes
the same rate across the tilt axes and hence across the legs — measured +1.6 % on
the reload's predicted acc and jerk (139.7/3873 → 142.0/3935). The bound can only
shrink `|v1|`; the frame fix can move the leg peaks either way.

### Where the bound DOES bind

**Nowhere, at the shipped `0.0` rate.** The one place it demonstrably binds is
below, and it binds only once a manufactured rate exists again — which is why
`test_ccatch1_clips_the_tier_8b_advisory_spot_checks` restores `0.07` before
measuring. Kept, with its numbers, because "the bound has never been observed to
fire" and "the bound cannot fire" are different claims and only the second would
justify deleting it.

One shipping-adjacent place, and it is the bound working rather than a
regression. `sim/toss_gate.py` Tier 8b seeds `build_catch` from the
swing-compensated pre-tilt pose at A, which leans OPPOSITE B's receive tilt — the
only geometry in the repo where the travel (1.2948°) exceeds the seat (0.6484°).
At the two **advisory** `T = 0.95` spot checks that puts the manufactured
excursion at 0.581 of the scale, over the 0.4938 allowed, so the rate is clipped
`0.070000 → 0.059469 rad/s` (85 %) and the settle overshoot `0.3008° → 0.2555°`. A
0.75° wrong-side swing on a 1.29° traverse is the 2026-07-25 defect's shape,
milder. Both points sit outside the binding 50 mm ring, so no gate PASS band
moves; recorded here and pinned with its number by
`test_ccatch1_clips_the_tier_8b_advisory_spot_checks` so a Tier-8b re-run reading
0.2555° is explained rather than re-derived.

### What the bound deliberately does NOT cover

The **seed** twist and acceleration. A C2 replan mid-reach inherits them from
the plan it supersedes; they are a continuity requirement, not a request, and
bounding them would reject legitimate `catch/dynamic_target` supersedes in
flight — turning a diagnostic invariant into a live catch failure. The contract
bounds only what `build_catch` *adds*.

## The frame split at the ingest

`catch/dynamic_target` carries **one** orientation and
`trajectory_node._catch_target_from_msg` derives **two** quantities from it:

| quantity | frame | who owns it |
|---|---|---|
| `target[3:6]` — where the legs are commanded | plan frame (C-LEVEL-1-corrected) | C-LEVEL-1 ingest E2 |
| `receive_tilt` — where the ball is coming from | gravity frame (**uncorrected**) | C-CATCH-1 |

The coordinator computes the wire orientation from the ball's arrival velocity
against gravity (`compute_catch_orientation` → `tilt_to_receive`), so it is
gravity-referenced by construction. Correcting it on the way to `build_catch` is
a one-token edit that restores the 2026-07-25 defect in full while leaving every
other assertion in `tests/ros/test_levelling_frame.py` green — which is why that
file's AST manifest now keys on `receive_tilt`'s **source text** at both call
sites, and why `test_the_receive_tilt_leaves_the_ingest_UNCORRECTED` exists.

C-LEVEL-1's last clause already forbids rotating a *velocity* through the
correction ("the correction is a bias on the commanded rotation, not a
re-expression of the platform frame"). The arrival tilt rate is such a velocity,
so it is aimed along the receive tilt verbatim and is not composed with the
correction. The residual that ignores is second-order (≈1.4 % of the rate,
perpendicular) and far below the physical uncertainty in the 0.07 rad/s tuning
constant itself; the defect being closed here is first-order (a direction that is
100 % wrong whenever the receive tilt is zero).

### The `receive_tilt=None` fallback, and why it is not a hole

`None` means "the catch pose is already gravity-referenced" and falls back to
`catch_pose[3:5]`. That is **correct** for every caller with no levelling concept
— `sim/toss_gate.py`, `sim/reload_gate.py` and the planner's own tests, none of
which have a correction to be wrong about (grepped 2026-07-25 for
`ros_ws/docs/levelling_frame.md`: neither `sim/` nor `controller/` has any
gravity-levelling concept at all). It is wrong for any caller that *does* have
one, and the only such caller is `trajectory_node`, which is pinned by the AST
manifest above. Making the argument mandatory would have churned ~25 call sites
to state a fact that is true by construction at 24 of them.

## Consequences at the machine

Each of these is correct behaviour, and each will look like a change at the
bench. Measured 2026-07-26 through `tools/probes/catch_reach_replay.py` on bag
`2026-07-25_15-17-48`.

> **Two landings, and both tables below are the FIRST one.** The "post-fix"
> columns are C-CATCH-1 as it landed at `407154f`, with the manufactured rate
> still at `0.07`. The zero-default decision came after, and it changes the
> reload rows again — see § *The manufactured rate ships at ZERO* and the third
> table below. The 0.07 columns are kept because they are what separates the
> **frame** fix's effect from the **zero-rate** decision's effect, and a bench
> capture that disagrees with the shipped behaviour needs to know which of the two
> it is disagreeing with.

**A gravity-level catch (the self-toss pre-tilt, 3.707 s lead).** The wire
receive tilt is exactly zero, so the through-seat does not engage at all:

| | pre-fix | post-fix |
|---|---|---|
| unrequested (wrong-side) excursion | **2.3239°** | **0.0000°** |
| peak off the park | 2.3239° | 0.7788° (= the request, monotone) |
| settle `rx` | −1.078408° | **−0.778784°** (exactly the target) |
| residual vs gravity at ball contact | 0.3008° | **0.0000°** |
| segments | reach / decay / hold | reach / hold |
| predicted leg peaks (vel/acc/jerk) | 14.2 / 142.4 / 3950 | 1.4 / 1.2 / 3 |

The 0.3008° residual is the one C-LEVEL-1 could not close, and the one that
predicts the session's tracker catch error: the quiescent hold runs to
release − 0.05 s and `hold_after=True` holds the settle pose *through* release,
so the hand threw from a platform 0.3008° off gravity-level —
`0.005250 rad × 3.93 m/s × 0.8 s = 16.5 mm`, against 16 mm measured.

**A real receive tilt (the reload pre-tilt, 10.87° wire, 2.371 s lead), at the
`0.07` rate.** The seat still engages at the full rate — the bound is
`0.19842 rad/s` against the `0.07` default, so it does not bind — but its **aim
rotates by 4.0997°**, from the plan-frame tilt to the gravity-referenced one:

| | pre-fix | post-fix (rate 0.07) | delta |
|---|---|---|---|
| settle `rx` | +1.823550° | +1.844635° | **+0.021086°** |
| settle `ry` | −10.933038° | −10.928741° | **+0.004297°** |
| peak off park | 10.9330° | 10.9287° | −0.0043° |
| predicted acc / jerk | 139.7 / 3873 | 142.0 / 3935 | +1.6 % / +1.6 % |

Both remain three orders under the session ceilings (5000 mm/s², 30000 mm/s³).

**The same reload reach at the shipped `0.0` rate** — the second landing, and the
one the bench will actually see. Measured 2026-07-26 through the same harness
(`--thrower ball_butler --toss 2`), against the `0.07` column above:

| | rate 0.07 | **rate 0.0 (shipped)** | delta |
|---|---|---|---|
| arrival tilt rate at contact | 0.070000 rad/s (4.011 °/s) | **0.000000** | −0.070000 |
| settle `rx` | +1.844635° | **+1.774062°** (= the target) | −0.070573° |
| settle `ry` | −10.928741° | **−10.636334°** (= the target) | +0.292407° |
| residual past the seat | 0.300803° | **0.000000°** | −0.3008° |
| segments | reach / decay / hold | **reach / hold** | −1 |
| plan duration | lead + 0.65 s | **lead + 0.50 s** | −0.15 s |
| predicted vel / acc / jerk | 23.8 / 139.7 / 3935 | **29.0 / 37.9 / 170** | see below |

Note the **velocity goes UP** while acc and jerk collapse, and that is arithmetic,
not a surprise: a terminal tilt rate in the travel direction lets the reach coast
slower in the middle (`p' = d·ψ'/T + v1·φ'`, and `φ'(0.5) = −0.4375`), so removing
it restores the plain rest-to-rest `1.875·d/T` peak. 29 mm/s against a 1000 mm/s
session ceiling — the point is only that "everything gets smaller" would be false.

This is a real, intended change on the **shipping** reload path: watch for it at
the bench rather than discovering it there.

**What that table does NOT cover, and why it matters.** Every row above is
anchored on the pre-tilt install, whose arrival is `landing − 1.5 s`. On the
reload path that is not the plan that runs through ball contact — the reference
bag carries **9 and 11** further accepted catch installs per attempt, across
`landing−0.78..−0.31 s` and `landing−0.83..−0.29 s`, and the last of each is what
is frozen through the catch. Those installs are why the scale is a `max` (above);
under the corrected scale their arrival rate was unchanged at `0.070000 rad/s` (it
is `0.000000` at the shipped rate, along with every other install — the supersede
seeded exactly on the target is then a literal no-motion plan, all predicted peaks
`0.0`), so the table's silence is harmless — but `tools/probes/catch_reach_replay.py`
prints a loud `!! NOT SCORED BY THIS ROW: N further catch install(s)` line for
them regardless, because an instrument that silently ignores a class of installs
reads PASS on a machine whose behaviour at contact has changed.

## Related

- `ros_ws/docs/levelling_frame.md` — C-LEVEL-1, the sibling contract that made
  this premise false.
- `plans/active/catch-reach-degenerate-overshoot.md` — the investigation.
- `tools/probes/catch_reach_replay.py` — reproduces all seven catch reaches in
  `2026-07-25_15-17-48` and prints the pre/post counterfactual for each.
- `tests/motion/test_trajectory_planner_catch.py` — the `test_ccatch1_*` block;
  `test_ccatch1_bounds_the_unrequested_excursion_at_a_long_lead` fails on its own
  assertion against pre-fix code (2.3237° against a 0.3846° allowance).
- `tests/ros/test_levelling_frame.py` — the frame split at the ingest, and the
  AST manifest that freezes it.
