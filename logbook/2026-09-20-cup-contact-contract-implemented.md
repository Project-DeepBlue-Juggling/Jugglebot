---
title: "Cup-contact contract implemented — the dive does not coast, it spikes in the knot before the window opens, and the tilt-jerk lever has no closed form and became a per-cycle measurement"
type: investigation
date: 2026-09-20
status: open
phase: "two-ball-skill-stack — R3"
related_plan: cup-contact-contract.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - config/generated/admissible_box.yaml
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cycle_plan.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - sim/skills_gate.py
  - tools/admissible_sweep.py
  - tests/hardware/session_skills_r3.md
  - tests/motion/test_cup_contact_contract.py
  - tests/motion/test_cup_realize.py
  - tests/motion/test_cup_cycle.py
  - tests/motion/test_validate_cycle.py
  - tests/motion/test_unified_cycle.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_admissible.py
  - tests/ros/test_skill_node.py
  - tests/sim/test_skills_gate.py
  - plans/active/cup-contact-contract.md
  - plans/active/two-ball-skill-stack.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
  - config
tags:
  - safety
  - performance
  - testing
  - dynamics
---

## Symptom

Four symptoms motivated `plans/active/cup-contact-contract.md` § 0: a 3.9 mm learner
lateral-aim correction at 0.9 m commanded 2.24° of tilt and 13.5 mm of platform excursion
in the catch window (the "0.9 m wobble", 2026-09-16); the learner's lateral channel and the
tracker's catch aim are both pinned/clamped to the schedule's own site because the platform
cannot be allowed to move laterally inside the dive; 18 of 21 timing-only re-aims refused
`LIMIT_JERK` on 2026-09-18 16:16 because the dive already sat at the jerk ceiling at zero
lateral offset; and the cup accelerates downward at ~15 m/s² (> g) in its post-catch run-up,
so a ball that arrives early loses contact (HELD/EMPTY/HELD), which is why the catch
tolerance is ±20 ms instead of ±100. All four trace to one mechanism: `tilt_to_receive`'s
banking prescription has no solution once the cup dives faster than g (apparent gravity
points up), so it clamps to a fixed 12° with a scale-free azimuth, and the tilt smoother's
widen loop then picks a discrete branch on 0.001 rad/s² of tilt acceleration. This entry is
the implementation of the three-clause contract (C-CUP-1/2/3) that mechanism motivated.

## Diagnosis

### 1 — C-CUP-1 + C-CUP-3 before/after at the R3 point

At the R3 single-site operating point (apex 0.9 m, `sites.columns_sites(100.0)[0]`, CATCH
with `then_throw`, launch limits 300/5000/150000, hand 3500 rev/s²), the raw banking
prescription pegged at exactly `12.000°` for every nonzero lateral offset before C-CUP-1;
after it, the raw series is proportional to the offset (`0.000°` at 0 mm, `0.941°` at 4 mm,
`7.254°` at 31 mm — every `raw = 12.000°` knot before the fix was exactly a knot with
seating force `s < ε·g`). The smoothed, commanded tilt at 4 mm dropped from **2.24° to
0.21°**, and the reproduction case at dx = 3.9 mm dropped from **137 250 to 19 870 mm/s³**
of peak leg jerk — 13 % of the 150 000 session cap where it was 92 %. Both quantities are
now monotone and very nearly linear in the offset (leg jerk ≈ 5 100 mm/s³ per mm of aim
across the whole grid) instead of flat-then-cliff.

### 2 — C-CUP-2: what binds is the DIVE into the catch, not the run-up

The brief's premise going in (from the 2026-09-17/18 sittings) was "after touch-down the
cup accelerates downward at ~15 m/s² in its run-up to the next throw." The planner says
otherwise: in the CATCH+throw window the post-touch-down run-up to the release never goes
below **−3.09 m/s² (−0.32 g)**, well inside `κ = 0.7`. The floor binds on the **approach**
instead — knots 30–34, the last 125 ms before touch-down, at −13.1 to −34.8 m/s² (3.55 g at
the worst knot) — which is exactly the "arrived early ⇒ HELD/EMPTY/HELD" failure mode.
With the row at the bare floor the QP lands exactly on it: the gate read the planner's own
output back 1.2e-8…7.7e-8 mm/s² low at the R3 point, but on the `test_validate_cycle_budget`
1.4 s STEADY window (peak tilt 2.56°) the gap was **36 mm/s²** — the term the cup QP's own
model omits, the tilt drop (`cup_z = level_z − arm·(1 − cos θ)`, a small, heavily-smoothed
second difference). Once the gate carries that term the residual falls to **4.7 mm/s²**,
and `CONTACT_ACC_QP_SLACK_MPS2 = 0.02 g = 196 mm/s²` — 40× the measured residual, growing
as θ² to the 12° cap — so the planner's own output is never refused by its own gate.

### 3 — the dive does not coast, and the τ table

With the row on, the QP does not re-phase into a longer, gentler dive from a higher hover
(the design's prediction). It is stroke-limited: coasting through the window at the
touch-down match speed costs more travel than the hand has above the touch-down height, so
the QP sits **at** the floor through the whole window and buys the entire touch-down
velocity in the **one knot before the window opens** — at τ = 0.060 s that knot's
`a_cup,z` goes from −13 067 to **−61 900 mm/s² (−6.3 g)**. Lengthening τ trades that spike
for a slower touch-down (measured 2026-09-20, apex 0.9 m; pre-contract baseline: peak dive
−34 820 mm/s², touch-down `v_z` −2.74 m/s against a 4.20 m/s ball):

| τ (s) | pre-window knot `a_z` (mm/s²) | touch-down `v_z` (m/s) |
|---|---:|---:|
| 0.060 | −61 900 (−6.3 g) | −1.90 |
| 0.100 | −43 400 | −1.67 |
| **0.125** | **−29 600** | **−1.54** |
| 0.150 | −21 700 | −1.47 |
| 0.200 | −11 300 | −1.44 |

τ = 0.125 s is the smallest knot multiple whose pre-window dive is no harsher than the dive
flown before the contract existed. The cost is the touch-down velocity match itself: the
ball now meets the cup **1.2 m/s faster in relative speed (2.66 vs 1.46 m/s)** than the
uncontracted plant. This table is also reproduced in `config/hardware_config.yaml`'s
`cup_contact_window_lead_s` comment (§ 2 of this repo's config section) and in plan § 7.

### 4 — the widen loop's dips were branch quantisation, not a design defect

A probe of the widen loop found `half` (the triangular smoother's integer half-width) was
**constant** across the whole R3 grid — the smoother never widened — while the blend
half-width `L` stepped **8 → 9 → 11 → 12 knots** at dx = 4 → 8 → 16 → 31 mm, purely from an
`np.ceil` on the analytic width. The achieved tilt jerk sawtoothed with it — **32.2 → 59.2 →
36.6 → 48.7 rad/s³** at those same four offsets, the dips landing exactly where `L` ticks —
and a second quantiser, the widen loop's own ×1.4 retry ladder, produced a further dip in
the throw window between dx 16 and 31 mm (4 ladder attempts, 55.1 → 30.6 rad/s³). The peak
knot moving between windows (36 → 50 → 44) was a *consequence* of which window carries the
peak, not a cause. R3 catch-with-throw grid, before/after making the widths real-valued and
bisecting for the tightest passing width instead of stepping an integer ladder:

| dx mm | tilt° before | tilt° after | leg jerk before | leg jerk after | peak knot b/a | attempts b/a | wall ms b/a |
|---:|---:|---:|---:|---:|---|---:|---:|
| 0.0 | 0.000 | 0.000 | 0 | 0 | 0/0 | 2/2 | 2.9 / 3.4 |
| 0.5 | 0.056 | 0.056 | 5 094 | 5 094 | 36/36 | 2/2 | 4.6 / 5.2 |
| 3.9 | 0.323 | 0.352 | 32 260 | 34 310 | 37/37 | 2/2 | 4.7 / 5.2 |
| 8.0 | 0.631 | 0.660 | 52 566 | 65 976 | 36/37 | 2/2 | 4.6 / 5.2 |
| 16.0 | 1.010 | 1.086 | 46 079 | 71 927 | 50/36 | 2/9 | 4.7 / 10.2 |
| 31.0 | 1.519 | 1.629 | 43 319 | 74 783 | 44/35 | 5/20 | 5.9 / 14.5 |

Monotone non-decreasing in dx, peak 74 783 mm/s³ = 50 % of the session cap (was a
52.6 k → 46.1 k → 43.3 k *fall*). Solve cost: +0.5 ms where nothing binds, +8.6 ms worst
case on this grid, against a 150 ms splice budget. At 0.6 m one residual non-monotonicity
survives, both points inside the saturation band: 82 266 → 78 578 mm/s³ (−4.5 %) between
dx 16 and 31 mm, because the composite is not monotone in the blend width (a wider blend
lowers the third difference but raises the tilt magnitude the `|j_z|·|θ|` term multiplies,
so bisection brackets the bound rather than landing on it exactly).

### 5 — the three `test_unified_cycle.py` reds, diagnosed

**Red 1** (`test_the_joined_report_carries_the_WHOLE_plans_peaks`) — pre-existing, unmasked
by this work, not caused by it. The differing field is `peak_leg_acc_mmps2` (merged
404.4773 vs whole 417.4568). The pose channel is a cubic Hermite, so leg acceleration is
discontinuous at a knot, and `validate_cycle` samples each knot once — which side of the
discontinuity it reads is decided by a floating-point ULP in `t / dt`. A two-sided scan
shows the plan's true supremum is **468.2153 mm/s²**; the one-sided grid reads 417.4568 —
under-measured by **10.8–13.6 %** depending on chain length. A detached-HEAD baseline read
merged = whole = grid = true sup = 739.4640 (the coin happened to land the same way on both
sides at HEAD); this work lowered the acc peak 739 → 417 and moved it *onto* a knot, which
is what made the pre-existing lottery visible. Fixed test-side only, against the plan's own
two-sided knot accelerations; the production fix (two-sided knot sampling in
`validate_cycle`) is deferred — it touches `gate_hash()` and moves every acc golden number
in the suite, so it is its own unit (Carried, below).

**Red 2** (`test_a_tightened_limit_applies_from_the_NEXT_window_not_retroactively`) — a
fixture defect. The test's `tight = head_peak * 0.5` fraction was never a derived number; it
passed on 2026-09-07 by coincidence. This work's whole uncommitted tree lowered the ring
head's peak jerk by 39 % but the 2.0 s window's own minimum-plannable-jerk floor by only
20 %, so the hand-picked 0.5 fell below the floor (14 229 > 12 454 with the contact floor
on). The contact floor narrows the gap by ~11 % (0.514 → 0.584 as a ratio) but is not the
primary actor — confirmed by toggling `contact_floor_enabled` off, where the fixture still
fails at 12 803 > 12 454. Fixed test-side: the fixture now **probes** the window's actual
floor at run time and asserts `tight = floor * 1.02 < head_jerk` — a tripwire that fails
loudly if the gap ever closes, instead of a fraction someone has to keep re-guessing.

**Red 3** (`test_the_timing_twins_are_read_off_the_plan`) — a fixture read plus a real,
independently-traced bug. With the contact floor on, the QP now hovers where it used to
hold still and then reverses harder into the dive (a genuine hand velocity zero-crossing);
the old test read the *nearest knot* rather than the interpolated crossing instant, which
happened to read below the 1.0 rev/s bound before this work and no longer does. While
tracing it, a second, independent defect was found: `_zero_crossing_s` interpolated
**linearly** between two knot velocities, but the hand channel is a cubic Hermite, so its
velocity is *quadratic* inside a span — a ~4.6 ms error at the reversal (the linear estimate
landed at a point where the plan's own accessor reads +1.197 rev/s, not 0). Fixed in
`unified_cycle.py` by bisecting the plan's own `hand_at` accessor instead
(`_ZERO_CROSSING_BISECTIONS = 40`, ~2.3e-14 s): `arm_lead_s` moved 0.18928 → 0.18470 s, and
the velocity at the returned instant is now 0 to ≤ 1e-11 rev/s. No control implications —
`arm_lead_s`/`stroke_clear_s` are reporting-only service-response fields with no command-path
consumer (grepped repo-wide).

A related flake surfaced along the way, not fixed: `test_the_qp_and_the_gate_stay_within_an_
order_of_magnitude` now sees 2/10 runs exceed its 10× bar on a quiet box (min 2.49×, max
25.16×) because C-CUP-2's acceleration rows inflate the QP solve time 5–9× relative to the
gate on the 1.4 s window. The test's *rationale* (catching BLAS starvation, which still
moves the ratio ~70×) survives; the bar does not. Carried, owner call.

### 6 — the sim's last-of-chain `caught=False` was a lateral miss, not the contract (U9)

The sim gate's chained self-toss run intermittently read `caught=False` (no `seat=`) on its
last throw, both before and after this work; traced (`handoff_u9.md`) to a SIM/TEST artefact,
not a symptom of the contract. `sim/skills_gate.py::_make_tracker` returned a fit
(`from_fit=True`) as soon as the ballistic estimator held **3** samples — a parabola over a
10 ms baseline (`OBS_PERIOD_S = 5 ms`), ~12 ms after release, extrapolating 0.5 mm of
observation noise. The executor's live catch re-aim (`_resend_live_catch`, gated 10 mm /
10 ms, cap 2) fired on the garbage n=3 fit. Measured (seed 0, policy A, 2026-09-21), the
chain's last flight's first fits: n=3 at (+27.5, −69.2) mm, n=4 at (−52.8, −32.0), n=5 at
(−83.6, +18.5), n=6 at (−89.4, +1.4), converging to the truth (−54.6, +8.2) mm by n≥35. The
n=3 re-aim was ACCEPTED; the corrective second re-aim was REFUSED
`REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK, peak leg jerk 481 688 mm/s³)` — the banking-saturation
class of `logbook/2026-09-16-banking-saturates-on-small-lateral-offsets.md` — and the re-aim
cap was then spent, so the cup dived to the bogus n=3 aim: at the crossing the ball was at
(−54.6, +8.2, 830) and the cup opening at (+27.8, −69.6) — **113.4 mm lateral, −1.3 mm
vertical**, outside the ~75–100 mm grab radius this capture model shows on every made catch
elsewhere. No contact → no kinematic capture → `caught=False`. Pristine HEAD ran the same
identical executor and got the same class of garbage n=3 fit (its own re-aim moved the landing
110.9 mm) but its QP happened to REFUSE that first re-aim, so the cup stayed on the schedule
aim and caught the ball — the contract changed which garbage re-aim the QP accepts, not the
underlying defect.

Fixed sim-side only (`sim/skills_gate.py`, `tests/sim/test_skills_gate.py`; no edit in
`feasibility.py`/`segments.py`): `_FIT_MIN_SAMPLES = 12` — 12 samples at this gate's 5 ms
observation period is 55 ms of span, one number mirroring both of the robot's
`tracking/flight_fit.BallisticFit` admission rules (`min_samples=12`, `min_span_s=0.050`);
`_make_tracker` gates on it instead of `est.n < 3`. After the fix: 3 attempts, end_codes
`['', '', '']`, drops 0, **makes 5** (all five caught), xy 37 → 7 mm by throw 3, apex 9.9 mm
at throw 5; every re-aim is now 2.6–37.9 mm and none is refused — the `LIMIT_JERK` end code
that used to appear in attempt 1 was itself downstream of a bogus re-aim splice and is gone
too.

Seat delay, all five throws (`seat=` vs scheduled landing, s):

| throw | this tree (before fix) | pristine HEAD | this tree (after fix) |
|---|---|---|---|
| 1 | +0.079 | +0.067 | +0.085 |
| 2 | +0.081 | +0.063 | +0.083 |
| 3 | +0.079 | +0.111 | +0.077 |
| 4 | +0.066 | +0.050 | +0.082 |
| 5 | — (no seat; the miss) | +0.059 | +0.063 |
| mean | +0.076 (n=4) | +0.070 | +0.078 |

Reading: **the contract's gentler, stroke-limited dive did not delay the seat** (+6…+8 ms of
mean shift, inside a spread that already runs ±25 ms). The sim's capture model is
`contact_carry=False` (`SkillsGate.plant` docstring: first ball/cup geom contact, kinematic,
no relative-velocity or persistence gate) — it is **blind to closing speed by construction**,
so it says NOTHING about rebound at the contract's 2.66 m/s touch-down (Diagnosis #3) either
way. On hardware, treat `seat=` past **~+0.2 s**, or a HELD→EMPTY→HELD gap, as the rebound
signature this sim cannot show.

### 7 — the phase audit: `CUP_CONTACT_ACC` sat one slot too early in the ladder

The single phase-end audit (2026-09-21, `/audit --unstaged`, one read-only pass over the
33-file diff) verified the sign convention, the QP row and slack, the gate's one-sided Hermite
formulas and units, all seven `CyclePlan` construction paths, the widen loop's bounded
iteration, `holds_ball`, and every fail-closed branch of the frame check — and found one
behaviour defect. The `elif cup_contact_reason is not None:` branch in `validate_cycle`,
whose own comment said "LAST in the code ladder, deliberately", had landed ABOVE
`elif peak_hand_c2 > hand_c2_tol:`. A plan that broke both the contact floor and the hand C2
tolerance reported `CUP_CONTACT_ACC` alone: the `HAND_LIMIT_C2` branch never ran, and the
"append the contact reason under whichever code wins" line only covers codes EARLIER in the
chain — so the torque-step refusal, the one that says the machine cannot execute the plan,
appeared nowhere. `report.ok` was `False` either way (nothing unsafe was admitted); what was
wrong was the surfaced code and reason, i.e. exactly the "report every refusal at once"
property this gate was written to have. No test could see it: every contact test drove
`_diving_plan`'s exact-quadratic dive, which is C2 by construction — and one test,
`test_cup_contact_multi_range_violation_in_the_second_range_is_caught`, carried a genuine
280 rev/s² hold→dive step at its knot 4 and read `CUP_CONTACT_ACC` only BECAUSE of the defect.

## Discussion

- **WITHDRAWN: plan § 7's prediction "the dive re-phases earlier and coasts."** Measured
  (Diagnosis #3): the cup QP is stroke-limited, sits at the floor through the whole contact
  window, and buys the dive in the one knot before it opens — the opposite of coasting. The
  owner chose "lengthen τ only" over a two-tier approach floor, a lower velocity-match
  weight, and accepting the design as drawn; τ = 0.125 s by the criterion "the smallest knot
  multiple whose pre-window dive is no harsher than the dive flown before the contract." The
  accepted tradeoff is a touch-down closing speed of 2.66 m/s against the pre-contract
  1.46 m/s, argued acceptable from the 2026-09-17/18 observation that the smoothest catches
  (`seat=` +0.10 s) met a cup moving only 0.3 m/s — what made a catch bad was the cup
  falling *away*, not the closing speed. That is a sitting-level hypothesis for the first
  sitting, not a result; τ should be revisited if catches start to bounce.
- **WITHDRAWN: "the jerk cap derives through the same lever as the accel cap"** (plan § 3).
  First landed as a fitted factor 2.1 (measured lever 903–1046 mm/rad vs the static
  478.7 mm/rad), then the owner asked for a derivation, and the derivation showed **no
  closed form survives the data**: a knot-scale closure (`Δ^(3−m)θ ≈ Δ³θ·dt^m`) bounds the
  R3 catch/throw windows to within 1.1–2.4× but under-reads the `test_unified_cycle` ring
  window's composite by 4.45× (it is carried there by `|j_z|·|θ|`, a release-stroke vertical
  jerk term unrelated to the schedule's own third difference); closing with the enforced
  rate/accel/angle caps instead is rigorous and gives a **negative** cap, because the rate
  cap is never approached at these speeds. Which term dominates is a property of the cycle,
  so the guarantee became a **measurement** — the tilt channel's Leibniz-expanded composite
  is evaluated on the series just produced and is a third exit condition in the widen loop,
  the same shape `_accel_bounded_schedule` already uses for the second difference. No
  literal factor survives (`grep -c TILT_JERK_LEVER_FACTOR` = 0 in the shipped code).
- **WITHDRAWN: a hypothesis that the merged-vs-whole `peak_leg_acc_mmps2` gap (3.1 %,
  Diagnosis #5 Red 1) was the seam-rate gap U2 found.** Dead: it is a knot-side sampling
  lottery in `validate_cycle` on a Hermite track whose leg acceleration is discontinuous at
  knots, latent since HEAD (where the coin happened to land the same way on both sides) —
  this work's own acceleration reduction moved the peak onto a knot, which is what made the
  lottery visible. The gate under-measures leg acceleration at a knot by 10.8–13.6 % on this
  fixture. Carried as its own item (two-sided knot sampling), owner unit — do not cite this
  test as evidence for the seam-rate class.
- **WITHDRAWN: a main-session hypothesis that the sim's intermittent last-of-chain
  `caught=False` was caused by the contract's higher touch-down closing speed** (2.66 vs
  1.46 m/s, Diagnosis #3 — a rebound suspected from the higher relative speed). Traced instead
  (Diagnosis #6, `handoff_u9.md`) to a sim/test artefact — a premature 3-sample tracker fit
  accepted as a lateral re-aim, unrelated to contact physics; the ball never touched the cup.
  The hypothesis is not just unconfirmed, it is **untestable in this sim**: the capture model
  is `contact_carry=False` (first-contact, kinematic), blind to closing speed by construction.
  It stays open for the first sitting, with Diagnosis #6's hardware-only tell (`seat=` past
  ~+0.2 s, or HELD→EMPTY→HELD).
- **REFUTED along the way: an early claim that `tests/ros/test_unified_cycle_integration.py`
  was "already 6-red at HEAD."** One unit's own toggle-plugin measurement read "both clauses
  OFF ≡ HEAD → 6 failed, 94 passed" on that file. Two later, independent, pristine runs of
  the same file — one the same day, one two days later, neither behind a toggle plugin —
  both read **100 passed, 0 failed**. The toggle plugin was measuring a moving, uncommitted
  tree against its own earlier snapshot, not HEAD; it was never a HEAD baseline. Record this
  as the reason every "before" measurement in this phase that mattered was taken from a
  reverted-to-HEAD copy of the file (or a detached worktree, per Diagnosis #5 Red 1's
  baseline), never from a toggle plugin diffing a live tree against its own history.
- **Tradeoffs accepted:**
  - `contact_knots` as a **tuple of ranges** rather than one range — a chained plan (e.g.
    LAUNCH+STEADY) has two contact windows with a ballistic flight between them; keeping
    only the later one (the first cut) silently made the first window's gate vacuous on
    every chained plan.
  - `RestTerminal.holds_ball` default **True** (fail-closed), AND-ed with
    `not seed.post_release` at the one production call site — the contract says a ball "is
    or MAY BE" held, so the skill layer defaults conservative and the seed's own explicit
    `post_release` flag is the one thing allowed to relax it.
  - **Geometry early-returns now carry the contact reason** alongside their own refusal code
    (`WORKSPACE`/`HAND_STROKE`/condition-number `UNREACHABLE`) — the "report every refusal
    at once" rule, so a refusal for one reason doesn't hide a second one riding with it.
  - **The tilt shaper now spends its reserve**: plan peak leg jerk rose from 29 % to 50 % of
    the 150 k session cap at dx 31 mm (Diagnosis #4) in exchange for a monotone, saturating
    response instead of a flat-then-cliff one. Real margin given up, worth an explicit ack
    before the next sitting.
  - The Red 2 fixture now **derives** its tightened-limit floor from a runtime probe instead
    of a hard-coded fraction that, on inspection, had never been derived in the first place.
- **Why the seam tilt-RATE pin was NOT built in this phase, although it was found early.**
  `start_tilt` pins a re-planned window's seam **value** in every channel but pins nothing
  about the tilt channel's **rate** — a re-plan splices two independently-shaped tilt
  schedules whose slopes at the seam only agree by construction under the old, always-
  saturated banking. It is masked again today (`test_unified_cycle_integration.py` reads
  100/100 with C-CUP-1/2 both in), and it is outside the contract as written in plan § 2/3.
  Building it means extending `tilt_schedule` with a `start_tilt_rate` and pinning knots 0
  AND 1 through the existing anchor machinery — kept on the carried list for R4, where two
  sites 250 mm apart make every throw a lateral re-aim.

## Fix

1. **`config/hardware_config.yaml`** (+ generated consumers: `config/generated/hardware_config.{h,py}`,
   `ros_ws/src/jugglebot/{CatchingCone_code,Teensy_code_canbridge,Teensy_code_platform}/hardware_config.h`,
   `ros_ws/src/jugglebot/jugglebot/hardware_config.py`) — three new constants, one definition
   each: `cup_banking_seating_min_g` (ε = 0.2), `cup_contact_acc_floor_g` (κ = 0.7),
   `cup_contact_window_lead_s` (τ = 0.125), with the τ derivation (Diagnosis #3's table)
   recorded in the YAML comment.
2. **`cup_realize.py`** — `_banking_raw(acc, cfg, start)` (C-CUP-1: prescribe only where
   `s ≥ ε·g`, else carry the last valid attitude, with the `start_tilt` / back-fill / level
   no-predecessor rule in precedence order); `_tilt_leg_jerk_mmps3` (the Leibniz-expanded
   composite that replaced the fitted lever); a third exit condition in
   `_accel_bounded_schedule` on that composite; blend/smoother widths made real-valued (the
   `np.ceil` removed) with a bisection between the last-failing and first-passing width
   instead of an integer ladder, so the schedule saturates at the bound rather than landing
   on whichever rung fired.
3. **`cup_cycle.py`** — `contact_window(n, dt, has_throw, catch_t_s, holds_ball_at_start)`;
   `_assemble` appends the acceleration-row block after every pre-existing column and
   refuses knot 0 analytically (`CupCycleInfeasible(reason='CUP_CONTACT_ACC')`) when a
   held-from-the-start seed already dives past the floor; `CupCycleConfig.contact_floor_
   enabled = True` with the runway row's own precedent (`_RUNWAY_DEFAULTS['contact_floor_
   enabled'] = False`, so sim parity is structural); `CONTACT_ACC_QP_SLACK_MPS2` (the
   0.02 g slack that absorbs the tilt-drop term the QP doesn't model, Diagnosis #2).
4. **`feasibility.py`** — `CUP_CONTACT_ACC` + `CUP_CONTACT_ACC_FLOOR_G/_MMPS2`;
   `_cup_contact_floor_check` (vectorised across every range in `contact_knots` in one
   pass, naming the worst knot/value and the violation count); pass 6 in `validate_cycle`,
   last in the ladder — genuinely last since the phase audit moved it below `HAND_LIMIT_C2`
   (Diagnosis #7; pinned by `test_cup_contact_reason_survives_a_coincident_HAND_LIMIT_C2_refusal`,
   and the multi-range test now asserts the code its own fixture really earns) — its reason
   always appended to `reasons` whatever wins `code`; the
   geometry early-returns (`WORKSPACE`/`HAND_STROKE`/`UNREACHABLE`) now compute and append
   it too (Discussion, tradeoffs).
5. **`cycle_plan.py`** — `CyclePlan.contact_knots` as `None` / a bare pair / a tuple of
   sorted, non-overlapping ranges (unsorted/overlapping is refused, never auto-sorted).
6. **`unified_cycle.py`** — plumbing at all seven `CyclePlan`/`from_realized` construction
   sites (ctor, `from_realized`, `_realize`, `_concat_plans`, the tail slice, the head
   slice, `splice`); `CycleGoals.holds_ball`; `_shift_contact` (duck-typed to accept both a
   bare pair and a normalised tuple) / `_join_contact` (concatenates head+tail ranges,
   merging only where they touch, rather than keeping only the later window);
   `build_realize_config` re-derives the jerk cap from `limits.leg_jerk_mmps3` the same way
   it already re-derives the accel cap; `_zero_crossing_s` now bisects the plan's own
   `hand_at` accessor (`_ZERO_CROSSING_BISECTIONS = 40`) instead of interpolating linearly
   across a knot span whose velocity is actually quadratic (Diagnosis #5 Red 3).
7. **`segments.py`** — `RestTerminal.holds_ball` default `True`; `_plan_rest` passes
   `terminal.holds_ball and not seed.post_release` into `CycleGoals.holds_ball`.
8. **`skill_node.py`** — session-start frame check (plan § 1): module constants, a pure
   `FrameCheckResult` dataclass + `_frame_offset_check()`, two bounded `deque` buffers
   fed by `_on_mocap`/`_on_commanded_position`, `_frame_check`/`_frame_check_error` methods,
   wired as an early-refusal precondition in both `_svc_start_columns` and
   `_svc_start_self_toss`, and (added later) into `skills/check` (`_svc_check`) too — so a
   dry-run sees the same `REJECTED_FRAME_OFFSET` refusal a powered start would hit, instead
   of only discovering it there.
9. **`tests/hardware/session_skills_r3.md`** — row `10a` (QTM `Platform` body precondition),
   row `21a`/`21b` (dress-rehearsal + reading the `skills/check` response's own frame-check
   line), and a `REJECTED_FRAME_OFFSET` row in the pre-registered verdict table.
10. **Tests** — new `tests/motion/test_cup_contact_contract.py` (the contract's own
    acceptance tests: amplitude invariance, the 2026-09-16 reproduction, the contact bound
    per segment kind, reason-survives-a-higher-priority-refusal); new/updated sections in
    `test_cup_realize.py` (banking carry/amplitude/no-predecessor rule, the widen loop's
    third-difference exit and saturation band, the lever's config-derived reduction),
    `test_cup_cycle.py` (the contact-floor row, the QP slack, the parity-fixture opt-outs),
    `test_validate_cycle.py` (reason format, multi-range violation, the chained
    LAUNCH+STEADY production plan passing the full gate), `test_unified_cycle.py` (the
    `extend` shift+concatenate, construction-path survival across all seven sites, the
    tilt-jerk-cap sibling to the existing accel-cap test, and the three fixed reds from
    Diagnosis #5), `test_skills_executor.py` (the tracker-clamp and learner-clamp share one
    `lateral_authority_m`), and `tests/ros/test_skill_node.py` (14 tests for the frame
    check's start-path precondition, 5 more for its `skills/check` surfacing).
11. **The unpinning (plan § 5 step 4)** — `skill_node.py`'s `learner_lateral_authority_mm`
    launch default moved **0.0 → 40.0** (owner decision 2026-09-21, plan § 6); the comment and
    every in-file cross-reference that called 0 "R3's own default" or "pinned" now says an
    explicit `:=0` is a re-pin, not the shipped default. `_frame_check_error`
    (`_svc_start_columns` / `_svc_start_self_toss` / `_svc_check`) is what now guards a live
    authority against an unverified mocap frame at every default-parameter start — a
    default-parameter node with no `Platform` mocap sample is refused
    `REJECTED_FRAME_OFFSET`, not a silent pass (new test, `tests/ros/
    test_skill_node.py::test_default_parameters_refuse_every_start_path_with_no_frame_data`).
    `tests/ros/test_skill_node.py::_node_with_client` grew a `frame=True` default (a ready,
    in-tolerance frame via `_frame_ready`) so tests not ABOUT the frame check keep passing at
    the new default; tests that ARE about it (missing/stale/offset frame, or asserting the raw
    buffer contents) opt out with `frame=False`. Two further owner decisions the same day
    (plan § 6): `τ = 0.125 s` CONFIRMED — the box re-sweep (above) found 0 `CUP_CONTACT_ACC`
    refusals in 14 731 rows at the launch limits, so no further change was needed; and
    authority 40 CONFIRMED **uniformly** across apexes despite the 0.6 m single-site box's y
    range being gate-limited to ±30 mm (box table, above) — the admissible box clips any
    out-of-band throw command regardless of the executor's lateral-authority clamp, so the two
    layers are orthogonal and the narrower box does not need a narrower authority to stay safe.

## Verification

All runs venv `~/Desktop/PDJ_venv/venv`, `PYTHONPATH=ros_ws/src/jugglebot:$PYTHONPATH`,
from `/home/jetson/Desktop/Jugglebot-skills`. The scoped rows are per file/unit; the last rows are the
full-tier gate on the committed tree.

| what | date, command | result |
|---|---|---|
| cup_realize + contract + segments + cup_cycle | 2026-09-20, `pytest tests/motion/test_cup_realize.py tests/motion/test_cup_contact_contract.py tests/motion/test_skills_segments.py tests/motion/test_cup_cycle.py -q -p no:cacheprovider` | **159 passed in 2.76 s** |
| unified_cycle + validate_cycle + contract + cup_realize + cup_cycle + segments | 2026-09-20, `pytest tests/motion/test_unified_cycle.py tests/motion/test_validate_cycle.py tests/motion/test_cup_contact_contract.py tests/motion/test_cup_realize.py tests/motion/test_cup_cycle.py tests/motion/test_skills_segments.py -q -p no:cacheprovider` | **295 passed, 1 failed in 15.36 s** — the one red is the QP/gate order-of-magnitude flake (Diagnosis #5), untouched by design |
| ROS integration (splice/plan-bench) | 2026-09-20, `pytest tests/ros/test_unified_cycle_integration.py tests/ros/test_skills_plan_bench.py -q -p no:randomly -p no:cacheprovider` | **162 passed in 10.52 s** |
| frame-check tests only | 2026-09-20, `pytest tests/ros/test_skill_node.py -q -p no:randomly -p no:cacheprovider -k "frame_check or frame_offset or check_reports or check_lists or on_mocap_buffers or on_commanded_position_buffers"` | **23 passed, 1 failed** — the 1 failure (`test_check_reports_ok_when_everything_is_fresh_and_the_box_is_valid`) is the pre-existing `admissible_box.yaml` `gate_hash` mismatch, unrelated to the frame check |
| full `tests/motion/` sweep | 2026-09-18, `pytest tests/motion/ -q -p no:cacheprovider` | **1905 passed, 1 failed in 308.15 s** — the one red (`test_leg_jerk_reproduction_and_monotonicity`) was the deliberately-unfixed measure-and-report item, later closed green (unedited) by the widen-loop fix (Diagnosis #4) |
| full `tests/ros/test_skill_node.py` | 2026-09-20, `pytest tests/ros/test_skill_node.py -q -p no:randomly -p no:cacheprovider` | **75 passed, 17 failed** — all 17 are the pre-existing `admissible_box.yaml` `gate_hash` casualties named in Diagnosis; none of the frame-check tests are among them |
| box re-sweep (launch limits 300/5000/150000, apex units, 0–10 mm lateral sampled explicitly) | 2026-09-21, `python tools/admissible_sweep.py --site-pairs both --single-apex 0.5 0.6 0.7 0.8 0.9`, logs `temp/logs/admissible_sweep_u4_run1_20260921.log` / `…run2_20260921.log` | **Deterministic — run 1 == run 2 byte-for-byte (`swept_at` excluded), 0 `CUP_CONTACT_ACC` refusals in 14 731 rows.** `gate_hash 37d68192b0e1` (2026-09-16) → `84c7af81fb1b`. Old → new box table below. |
| `sim/skills_gate.py --learn --policy A --seeds 0-4` against the re-swept box (authority still unset in the gate itself — Discussion #6) | 2026-09-21, `python sim/skills_gate.py --learn --policy A --seeds 0 1 2 3 4`, log `temp/logs/sim_gate_contract_20260921.log` | **PASS, all 5 seeds** — band_xy 3, band_apex 5, mono True/True, 0 drops (makes 23/22/25/21/22 of 25 this run, BEFORE the U9 tracker fix below) |
| `sim/skills_gate.py` after the U9 sim-tracker fix (`_FIT_MIN_SAMPLES = 12`, Diagnosis #6) | 2026-09-21, `python sim/skills_gate.py --learn --policy A --seeds 0 1` then `--seeds 2 3 4` (`handoff_u9.md`) | **PASS / PASS** — all five seeds band_xy 3, band_apex 5, mono True/True, 3 attempts, 0 drops, **25 makes of 25** (against 23/22/25/21/22 makes before the fix) |
| `pytest tests/sim/test_skills_gate.py -q -p no:cacheprovider` (U9) | 2026-09-21 | **12 passed in 29.47 s** — the previously-xfailing xy-band test (R3 carried item (k)) now passes as a plain test |
| box re-sweep #3, after the audit's ladder fix changed `feasibility.py`'s bytes | 2026-09-21, `python tools/admissible_sweep.py --site-pairs both --single-apex 0.5 0.6 0.7 0.8 0.9`, log `temp/logs/admissible_sweep_run3_20260921.log` | **1708.1 s; box content IDENTICAL to run 1** (`gate_hash`/`swept_at` lines excluded, 104 lines compared) — `gate_hash` `84c7af81fb1b` → `baecbc55a839`, matching the live gate |
| gate tests after the ladder fix | 2026-09-21, `pytest tests/motion/test_validate_cycle.py tests/motion/test_cup_contact_contract.py -q -p no:cacheprovider` | **58 passed in 1.41 s** |
| `./run_tests.sh --full`, first run (the tree before the audit) | 2026-09-21, `./run_tests.sh --full`, log `temp/logs/cup_contact_full_20260921.log` | **FAIL — parallel 1 failed, 6371 passed, 9 skipped, 1 xfailed in 289.07 s; serial 6 passed in 19.25 s.** The one red, `tests/motion/test_skills_admissible.py::test_tiny_sweep_yaml_round_trips_and_validates`, was test-side: its `_live_limits()` helper hard-codes jerk 200 000 while the sweep tool's defaults now read the generated launch constant (150 000); the test now reads the tool's own limits (41 passed, `pytest tests/motion/test_skills_admissible.py -q -p no:cacheprovider`) |
| `./run_tests.sh --full`, the committed tree (after the audit's two fixes and re-sweep #3) | 2026-09-21, `./run_tests.sh --full`, log `temp/logs/cup_contact_full2_20260921.log` | **PASS — parallel 6373 passed, 9 skipped, 1 xfailed in 288.09 s; serial 6 passed in 19.26 s** |

**Box re-sweep, old (2026-09-16, `gate_hash 37d68192b0e1`) → new (2026-09-21):**

| box | old xy mm | new xy mm | apex m |
|---|---|---|---|
| single 0.9 m | x[0,40] y[0,0] | ±40 × ±40 (grid edge) | 0.576–0.900 (unchanged) |
| single 0.8 m | (0,0) | ±40 × ±40 | 0.512–0.882 |
| single 0.7 m | (0,0) | ±40 × ±40 | 0.448–0.847 |
| single 0.6 m | x[0,10] y[0,0] | ±40 × ±30 (y gate-limited: `LIMIT_JERK`/`MARGIN`) | 0.384–0.726 |
| single 0.5 m | (0,0) | ±20 × ±10 | 0.320–0.605 |
| columns P1→P2 | x[−20,10] y[−20,20] | ±20 × ±20 | 0.850–0.900 → **0.900 only** |
| columns P2→P1 | x[−20,20] y[−10,20] | ±20 × ±20 | 0.850–0.900 → **0.900 only** |

The columns apex-band collapse (both edge apexes, 0.850/0.950 m, now fail `THROW:MARGIN` at
zero offset) is a real narrowing, not a grid artefact — Carried, R4 (below).

## Carried

* **Two-sided knot sampling in `validate_cycle`** — `peak_leg_acc_mmps2` is under-measured
  at a knot by 10.8–13.6 % on the fixture that surfaced it (Diagnosis #5 Red 1); touches
  `gate_hash()` and every acc golden number in the suite, so it is its own unit.
* **The seam tilt-rate pin** (`start_tilt_rate`, pinning knots 0 and 1 as one anchor group)
  — `start_tilt` pins the seam's value, nothing pins its rate; masked today, on the R4
  re-aim critical path (Discussion).
* **The QP/gate wall-clock ratio test** (`test_the_qp_and_the_gate_stay_within_an_order_of_
  magnitude`) is now a 2-in-10 flake on a quiet box, because the contact rows inflate the QP
  5–9× on the 1.4 s window; needs a re-derived bar or a median-of-N, owner call.
* **The −4.5 % residual dip at 0.6 m** (16 → 31 mm), inside the saturation band — the widen
  composite is not monotone in blend width, so bisection brackets rather than lands on the
  bound (Diagnosis #4).
* **Re-read ε after a sitting** — no contact-window knot is ever undefined now that
  `s ≥ 0.3 g` over the window (C-CUP-2 moved the resume knot outside the contact window
  entirely at the operating points measured).
* **The columns apex-band collapse** — both edge apexes (0.850/0.950 m) now fail
  `THROW:MARGIN` at zero offset, so the re-swept columns boxes cover only the centre apex
  (0.900 m) instead of the old 0.850–0.900 m band (box table, above). R4 item (two sites
  250 mm apart is on R4's critical path).
* **The unpinning itself is DONE** (plan § 5 step 4, this entry's Fix #11) — what remains is
  the sitting criteria: `seat=` staying in +0.05…+0.15 s, and the median lateral miss
  trending to 0 within five throws.
* **`colcon build`** owed before any sitting (`skill_node.py`, `unified_cycle.py` and the
  rest of the ROS package changed).
