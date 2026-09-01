---
title: "Unified 7-DoF planner Phase 1 — the planner core lands: a numpy QP that refuses instead of lying, a bit-exact tilt decomposition, a 7-channel gate, and an 0.80 s flight lid"
type: investigation
date: 2026-09-01
status: resolved
phase: "unified-7dof-planner — Phase 1"
related_plan: unified-7dof-planner.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cycle_plan.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/limits.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/__init__.py
  - sim/cycle_gate.py
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - tests/motion/test_cup_cycle.py
  - tests/motion/test_cup_realize.py
  - tests/motion/test_cycle_plan.py
  - tests/motion/test_validate_cycle.py
  - tests/motion/test_shaped_batch.py
  - tests/motion/test_trajectory_feasibility.py
  - tests/sim/test_cycle_gate.py
  - tools/probes/capture_cup_cycle_refs.py
  - tools/probes/data/cup_cycle_qp_refs.npz
  - tools/probes/README.md
  - plans/active/unified-7dof-planner.md
  - plans/active/INDEX.md
  - logbook/2026-09-01-unified-7dof-planner-phase1-planner-core.md
  - logbook/INDEX.md
subsystem:
  - motion
  - config
  - sim
  - tools
tags:
  - trajectory
  - kinematics
  - dynamics
  - performance
  - testing
  - safety
---

# Unified 7-DoF planner Phase 1 — the planner core

## Summary

Phase 1 of [`plans/active/unified-7dof-planner.md`](../plans/active/unified-7dof-planner.md)
is the software-only planner core: the whole-cycle cup QP, the tilt schedule and
cup→platform+slider decomposition, the 7-channel plan object, the 7-channel
feasibility gate, and the sim phase gate that accepts them. All four work
packages landed and the phase gate **PASSES**.

1. **WP1 — `cup_cycle.py`**: `sim/juggle_planner/juggle_planner.py::plan_cup_cycle`
   ported to a dependency-free **Goldfarb–Idnani dual active-set QP** with an
   event-timeline `plan_window` API and the catch-runway hard constraint. Parity
   against the frozen CasADi/IPOPT fixtures is **1.906e-4 mm / 2.471e-3 mm/s**
   worst case, against bars of 1 mm / 10 mm/s. It also **found and closed a
   second silent-wrongness class** the Phase 0 probe had not reached.
2. **WP2 — `cup_realize.py` + `cycle_plan.py`**: apparent-gravity banking with a
   2-norm cap and rate limit, the `CUP_TILT_CENTER_Z_MM = 744.3` lever
   decomposition generalised to a floating base, and a `TrajectoryPlan`-contract
   7-channel `CyclePlan`. **Zero-banking parity vs `sim/juggle_tilt.py::realize_tilted`
   is asserted at `max|Δ| == 0.0`** — bit-exact, not "close".
3. **WP3 — config + `validate_cycle`**: six new config keys (owner-signed
   two-tier hand caps, the unified-mode flag, the z-float pair), and a 452-line
   `validate_cycle` in `feasibility.py` with three new refusal codes, each
   probe-confirmed deterministic before it was pinned.
4. **WP4 — the phase gate**: `sim/cycle_gate.py`, driving the production chain
   end-to-end. **PASS**, 11/11 points, capture distance **0.000 mm** everywhere.
   Getting there required two real fixes (accel-bounded banking; cfg-gated cup
   accel boxes) and two harness bugs found, fixed and regression-tested.

**The headline for tier planning:** under the owner-signed 3500 rev/s² hand
acceleration cap with z pinned at 170 mm, **the maximum plannable flight time is
0.80 s**. The lever on that number is **release height**, not any planner knob.

Four owner decisions are recorded here and **all four were resolved by the owner
on 2026-09-01** (§ Open Questions) — the largest being that every gate cycle
refuses at the *shipped* leg-jerk limit, for a structural reason; the resolution
is that unified sittings ride raised session limits, with nothing shipped
changing.

## Motivation

Phase 0 closed the four probe decisions; Phase 1 is where they become code. The
phase exists to answer three questions that no amount of desk work could:

- **Does the QP port survive contact with a real grid?** Phase 0 measured parity
  on six cases. A planner that is right on six cycles and silently wrong on the
  seventh is worse than no planner, and the Phase 0 entry had already caught one
  silent-wrongness mode (the naive KKT loop cycling to answers 124 m off).
- **Does the ball-frame formulation actually buy anything?** Banking is only
  worth its complexity if it measurably reduces the lateral specific force a
  seated ball feels — and it has to do so without shipping a trajectory the legs
  cannot follow.
- **Does the runway constraint answer Rung 3?** `bb-online-juggle-tilt-rearchitecture.md`
  Rung 3's P2 attempt died on *"the slam is the seat's runway"*: every cleanup of
  the whole-cycle sim catch dropped MAKE to 0/12. Owner resolution 1 (2026-08-29)
  says the runway becomes a hard planner constraint and the sim authority becomes
  kinematic capture. Phase 1 is where that claim is either demonstrated or not.

The four work-package module docstrings carry the design detail and are the
canonical reference for *how* each piece works — `cup_cycle.py`,
`cup_realize.py`, `cycle_plan.py`, `feasibility.validate_cycle` and
`sim/cycle_gate.py`. This entry records the *decisions, measurements and
surprises*, and does not re-derive them.

## WP1 — `cup_cycle.py`: the QP port

`ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py` (969 lines,
`wc -l` 2026-09-01). Goldfarb–Idnani dual active set, per the Phase 0 binding.

**The event-timeline API** (owner resolution 3, 2026-08-29): `plan_window(events,
…)` takes an ordered event list; the one-throw-one-catch case is **bit-identical**
to the legacy `plan_cup_cycle` signature, asserted as such
(`tests/motion/test_cup_cycle.py`, T-U2's event-timeline twin). Three-ball later
becomes data rather than an interface rewrite, and the cost today is one
signature. Warm-start input is accepted for the Phase 4 replan path.

**The catch-runway hard constraint** is applied in two places and the split is
the interesting part. Inside the QP it is a linear inequality row on the target
catch velocity — genuinely shaping, because `pos[catch_k, 2]` is free. But the
catch-position *equality* pins all three components of the cup position at
touch-down, so on a **descending** catch there is no freedom left at that instant:
there, the analytic `CATCH_RUNWAY` gate is a **refusal gate, not a shaper**, and
the module says so in as many words. Raising `CupCycleInfeasible` with the
numbers beats letting the same fact reappear as an unbounded dual step deep
inside the solver.

### Parity and the T-U1 grid

(2026-08-30, `python -m pytest tests/motion/test_cup_cycle.py -q`, **51 passed in
3.83 s**)

| Measure | Result | Bar |
|---|---|---|
| Parity vs the frozen CasADi/IPOPT fixtures, 6 cycles | **1.906e-4 mm / 2.471e-3 mm/s** | 1 mm / 10 mm/s |
| T-U1 grid (30 points): release `‖a − g‖` | **6.0e-14** | < 1e-6 |
| T-U1 grid: detach collinearity `‖cross(a − g, axis)‖` | **3.0e-15** | < 1e-6 |
| T-U1 grid: catch position error | **2.4e-11 mm** | < 0.1 mm |

The reference fixtures are `tools/probes/data/cup_cycle_qp_refs.npz`, regenerated
by `tools/probes/capture_cup_cycle_refs.py --emit-fixture` under the PDJ venv
(the only interpreter with CasADi). A re-run reproduces the fixture
**bit-exactly**, which is what makes the committed `.npz` a fixture rather than a
snapshot of one lucky afternoon.

### The silent-wrongness class this phase found and closed

Building the T-U1 grid surfaced a **knife-edge infeasible 0.40 s cycle** that the
solver "converged" on: working set 38 of 48 rows, `NᵀH⁻¹N` numerically singular,
**every inequality satisfied**, and an **equality residual of 121**. Finite
termination is not correctness. IPOPT refuses that same cycle, so it is genuinely
infeasible — the QP was inventing an answer.

The fix is a hard post-condition, not a heuristic: **every solve now verifies its
feasibility residual ≤ 1e-7 before returning** (converged solves land at
**1e-13..1e-12**, four to five orders inside the bar), and refuses loudly
otherwise. Iteration exhaustion raises too.

This is the same shape as the Phase 0 cycling failure reached by a different
road — conditioning rather than cycling — which is the argument for a *verify* as
a standing invariant rather than a patch on the one case that exposed it.

**One documented wrinkle:** the *refusal path* is BLAS-dependent. Under the venv
the knife-edge case exits via an unbounded dual step; under system python it
exits via `_verify`. **Both refuse**, which is the property that matters, and the
test asserts the refusal rather than the route. The docstring records it so a
future reader does not treat a route change as a regression.

## WP2 — `cup_realize.py` and `cycle_plan.py`

`cup_realize.py` (774 lines) and `cycle_plan.py` (226 lines), `wc -l` 2026-09-01.

**`tilt_schedule`** banks the cup axis into apparent gravity `g − a_cup`,
saturated at `tilt_geometry.MAX_TILT_DEG` (12°), rate-limited, and **pinned
exactly** to the receive tilt at catch and the throw tilt at release — those two
are boundary conditions of the ball-frame physics and win over both the banking
objective and the rate limit.

**A real bug was caught here by a test, not by review.** The first rate limiter
was per-axis, which leaks past a 2-norm cap on a diagonal: it produced
**12.124°** against the 12° cap. The **T-U3 cap assertion caught it** and the
limiter was rewritten to operate on the 2-norm. That is the whole argument for
asserting the cap as an invariant rather than assuming the construction respects
it: a per-axis rate limit *looks* obviously safe.

**`decompose`** generalises `sim/juggle_tilt.py::realize_tilted` from one pose to
a knot series using the same height-invariant `CUP_TILT_CENTER_Z_MM = 744.3` mm
lever model. `CUP_Z_BASE_MM` is generalised from a literal to `base(z)` so the
z-float mode has somewhere to go — and it **reduces to the literal bit-exactly**
at the pin, so the default path is unchanged by construction rather than by
inspection.

**Zero-banking parity is asserted at `max|Δ| == 0.0`**, including a stroke-clamp
fixture. (2026-08-30, `python -m pytest tests/motion/test_cup_realize.py
tests/motion/test_cycle_plan.py -q`, **60 passed in 0.72 s**)

**A numerics finding that binds anyone touching this file:** `tilt_geometry` has
two lever helpers — `cup_axis` (rotation matrix through the production IK helper)
and `shaping._cup_axis_xy` (closed-form Rodrigues). They are *mathematically*
identical and differ at **8.3e-17**. `realize_tilted` uses the matrix form, so
bit-parity requires `cup_realize` to use it too. An 8.3e-17 difference is
invisible in review and fatal to a `== 0.0` assertion.

**`CyclePlan`** subclasses `TrajectoryPlan` and honours `state_at` exactly, so
the 40 Hz emitter and `trajectory_node`'s `_install` continuity machinery work on
it unchanged; `hand_at(t) → (rev, rev_s)` is the additional surface Phase 2
reads. Piecewise-cubic between knots by construction — the same reconstruction
the can-bridge performs from `(u0, v0, u1, v1)`, which is what makes Phase 0
decision 2's float-exactness claim apply to this plan's output.

Two hazards are documented **and pinned blind**, which is the honest formulation:

- **Zero segments.** A cycle plan is knot-based, so `self.segments` is empty.
  Every existing `feasibility` entry point loops `plan.segments` — so on a
  `CyclePlan`, `validate` takes the `HoldPlan` branch and reports `t=0` only, and
  `validate_follow` reports `0/0/0 OK`. **Both silently pass a plan they measured
  nothing of.** Tests pin exactly that blindness, so the day someone "fixes"
  `validate` to handle cycle plans, the pins say so out loud. This is why
  `validate_cycle` samples the knot grid itself and must not delegate.
- **The terminal-hold cliff.** The `TrajectoryPlan` contract's hold is
  `final_pose` with **zero twist** — but a cycle ends at the throw, moving at
  takeoff velocity. Sampling past `total_duration` reads a hard stop. That is
  inherited deliberately (deviating would break the contract `CyclePlan` exists
  to satisfy); Phase 4's orchestrator owns installing the next cycle before the
  clock runs off the end.

### Owner sign-offs taken during WP2

(2026-08-30, `AskUserQuestion`, all three accepted)

| Question | Decision |
|---|---|
| Slider frame origin | **20 mm offset** — the catch prime lands exactly on `JB_OP_HAND_CATCH_PRIME_REV` |
| `unified_z_band_mm` | **30 mm** |
| Tilt rate limit | **3.0 rad/s** |

## WP3 — config, `TrajectoryLimits`, and `validate_cycle`

**Six config keys** landed in `config/hardware_config.yaml`:
`jugglebot_operational.unified_cycle_enabled` (**false** — a reviewed config
commit, not a runtime toggle, so "which build ran the unified planner" is
answerable from git alone); `trajectory_op.hand_vel_limit_rps` **200** /
`hand_vel_ceiling_rps` **300** and `hand_acc_limit_rps2` **3500** /
`hand_acc_ceiling_rps2` **3900** — the **two-tier limit/ceiling shape mirroring
the legs**, so a `set_limits` ramp is bounded by something the YAML pins;
`unified_z_float_enabled` (**false**) and `unified_z_band_mm` (**30**).
`TrajectoryLimits` carries the hand pair with the identical clamp rule.

(2026-09-01, `python config/generate_config.py --check`, **CONFIG FRESH,
14 artifacts**)

**`SetTrajectoryLimits.srv` was deliberately NOT widened.** Adding hand fields
needs an interfaces rebuild, which belongs with Phase 2/4's wire work; the
session hand limits are YAML-set for now. Recorded here so the omission reads as
a decision rather than an oversight.

**`validate_cycle`** (+452 lines in `feasibility.py`) samples the knot grid
directly, never delegating, for the reason above. New codes `HAND_STROKE`,
`HAND_LIMIT_VEL`, `HAND_LIMIT_ACC` join the existing vocabulary. Three
properties are worth the entry:

- **Each of the three codes was probe-confirmed deterministic ×2 before it was
  pinned** (`/tmp/probe_validate_codes.py`, 2026-08-30), per the empirical-probe
  rule; the confirmed recipes are a table in the test docstring, so the next
  reader gets the recipe, not the archaeology.
- **Hand extrema are CLOSED-FORM, not sampled.** Knot-grid finite differencing
  **under-measures the hand acceleration by 38 %** — **2763.5 vs an analytic
  4432.5 rev/s²** on the case that exposed it. Against the 3500 cap that is a
  **false accept**: the sampler says 2763.5 (pass), the metal sees 4432.5. The
  hand channel's per-span extrema are therefore computed in closed form and are
  independent of `samples_per_knot`.
- **The runway is re-checked post-hoc with the ACHIEVED catch velocity**, against
  the slider's true bottom of travel — the QP bounds it with the *target* speed
  against a cup-frame floor that defaults to `z_min_m`, which sits ~0.21 m below
  the real floor. Not a duplicate check; the first evaluation against both real
  quantities.
- The terminal sample is taken at `nextafter(total_duration, 0)` so the release
  velocity is not masked by the terminal hold's zero twist.

Triples:
(2026-09-01, `python -m pytest tests/motion/ -q`, **2108 passed / 3 skipped in
325.76 s**);
(2026-09-01, `python -m pytest tests/ros/ -q`, **2630 passed / 1 skipped in
334.85 s**);
(2026-09-01, `python -m pytest tests/firmware/test_config_drift.py -q`,
**21 passed**).

## WP4 — `sim/cycle_gate.py`, the phase gate

`sim/cycle_gate.py` (1020 lines) plus `tests/sim/test_cycle_gate.py` (17 tests,
**unmarked**, matching `test_toss_gate.py`'s tier placement). The harness drives
the production chain — `plan_window` → `tilt_schedule` → `decompose` →
`CyclePlan.from_realized` → `validate_cycle` — and **re-implements none of it**;
a harness that re-derived any stage would gate a different machine than the one
Phase 2 ships.

Two fixes were needed before it could pass, and both are real planner content
rather than harness tuning.

### Fix A — accel-bounded banking

The first banking schedule was smooth in *angle* and violent in *acceleration*:
the demo cycle reached **240 rad/s²** of tilt acceleration, which the 744.3 mm
lever turned into **112 577 mm/s²** of leg acceleration and **9.66 M mm/s³** of
leg jerk. Smooth-looking, unflyable.

The fix bounds tilt acceleration directly at `tilt_accel_limit_rad_s2`
(**default 5.326 rad/s²**, config-derived from the leg-acceleration budget over
the 469.4 mm worst lever) using a construction that **preserves the cap and the
endpoint pins by convexity** rather than by iteration. Result on the demo cycle:
tilt accel **240 → 4.75 rad/s²**, leg accel **112 577 → 1192 mm/s²**, leg jerk
**9.66 M → 80 k mm/s³**.

**The projection-sweep alternative was measured and rejected**, not dismissed:
iterated projection onto the accel bound reached **38.9 rad/s² after 16 sweeps**
and **5.5 after 800**, against a target of 5.33 — i.e. it converges, but only
asymptotically and with unbounded iteration count. A planner stage whose runtime
depends on how hard the cycle is, is not a planner stage that belongs on a
per-cycle budget.

### Fix B — cfg-gated cup acceleration boxes, OFF by default

Optional acceleration boxes on the cup trajectory, `parity 0.0` proven when off.
**Recorded as a negative result:** the box **does not buy flight time**. An 0.8 s
flight genuinely needs **134 m/s²** below release; a box does not reshape the
trajectory into feasibility, it just **refuses earlier**. Shipping it off-default
keeps a lever available for Phase 4 without pretending it solves the tier lid.

### Two harness bugs, found, fixed, regression-tested

Both would have produced *confidently wrong* gate numbers:

1. **Catch scored at `catch_k` instead of the interpolated touch-down** — a
   **33.7 mm phantom capture error** on a chain that is actually exact.
2. **The carry window opened one knot early**, which flips apparent-up by
   **177°** — the seat criterion would have scored the banking as almost exactly
   backwards.

### Gate result

(2026-09-01, `python sim/cycle_gate.py`, **PASS**, 10.5 s wall)

| Band | Result |
|---|---|
| Points | **11** — flights 0.5–0.8 s + displaced sites to ±80 mm + an f0.9 advisory |
| `capture_ok` | **0.000 mm** capture distance everywhere |
| `parity_max_abs` (zero banking) | **0.0** |
| Banking vs level, apparent-gravity misalignment | banking wins **11/11** |
| `no_slam` | slam-free everywhere |
| `runway_active` | worst margin **106 mm** |
| MuJoCo contact (**advisory only, never gates**) | **4/11 makes** |

**The 4/11 MuJoCo result was predicted, and is the Rung-3 P2 contact-model
signature.** It is reported and never gated, per owner resolution 1.

Scoped runs: (2026-09-01, the 5-file scoped command, **164 passed in 8.07 s**);
(2026-09-01, `python -m pytest tests/motion/ -q`, **2115 passed / 3 skipped in
331.03 s**).

## The headline: 0.80 s is the flight lid, and release height is the lever

Under the owner-signed **3500 rev/s²** hand acceleration cap with **z pinned**:

| Flight | Required hand accel | Verdict |
|---|---|---|
| 0.80 s at release z **0.86 m** (170 mm runway) | **3256 rev/s²** | plannable — the lid |
| 0.85 s | **3678 rev/s²** | over the cap |
| ≥ 1.05 s | — | refuses outright |
| 0.70 s at release z **0.78 m** | **4280 rev/s²** | over the cap |
| above release z **0.88 m** | — | the pre-launch dip no longer fits |

So the number that moves the tier ceiling is **release height**, not a planner
knob: raising the release from 0.78 m to 0.86 m is worth more than 0.1 s of
flight, and the ceiling above 0.88 m is geometric (the pre-launch dip stops
fitting), not a limit that can be tuned. Tier planning for Phase 5 should be read
off this table, not off the cap alone.

## Discussion

### Why Goldfarb–Idnani *and* a feasibility verify — two measured classes, not one

Phase 0 bound the port to G–I because the naive KKT-plus-violated-boxes loop
**cycles** and returns silently wrong trajectories. That is class one, and G–I
closes it: monotone dual objective, finite termination by construction.

Phase 1 found class two, and G–I does **not** close it. A knife-edge infeasible
cycle drove the working set to 38 of 48 rows, made `NᵀH⁻¹N` numerically singular,
and **terminated normally** with every inequality satisfied and an equality
residual of 121. Finite termination guarantees the algorithm stops; it guarantees
nothing about what it stopped on when the working-set system is singular.

The two classes share a symptom (a plausible-looking trajectory, no exception)
and differ in mechanism (cycling vs conditioning). That is exactly the situation
CLAUDE.md's "climb one level of abstraction" rule describes: the right response
is not a patch on the singular case but an **invariant on every solve** — the
returned point satisfies the equalities to 1e-7 or the solve refuses. Converged
solves land at 1e-13..1e-12, so the bar costs nothing in false refusals and buys
immunity to every future member of the class, including mechanisms not yet seen.

The BLAS-dependence of the *refusal route* is the honest residual. Two
interpreters take two different exits on the same case. Asserting the route would
pin an accident of the linear-algebra backend; asserting the **refusal** pins the
property. The docstring records the observation so a route change is not misread
as a regression.

### Why hand extrema are closed-form and not sampled

The knot grid is the natural place to sample, every other channel is measured
there, and it under-measures hand acceleration by **38 %** (2763.5 vs 4432.5
rev/s²). Against a 3500 rev/s² cap that is not a small error — it is the
difference between a refusal and a **false accept on the fastest axis in the
machine**, where the planner cap is (per Phase 0 decision 4) the only practical
overspeed guard that exists today.

The alternative was to mesh the hand channel finely enough that the error falls
under some margin. Rejected: it converts a correctness property into a tuning
parameter, and the required mesh depends on the cycle. The hand channel is
piecewise-cubic by construction, so its per-span extrema have a closed form —
taking it makes the gate exact and independent of `samples_per_knot` at the same
time.

### Why the convexity construction beat projection sweeps

Both approaches produce an accel-bounded tilt schedule. The projection sweep is
simpler to write and was measured: **38.9 rad/s² after 16 sweeps, 5.5 after 800**,
target 5.33. It converges — asymptotically, with an iteration count that depends
on the cycle, and with the cap and the endpoint pins only approximately held at
any finite sweep count.

The convexity construction holds the cap and both endpoint pins **exactly, by
construction, in one pass**. On a stage that must run inside Phase 4's ≤ 50 ms
per-cycle budget, a bounded-work construction that is exactly right beats an
unbounded-work iteration that is approximately right — and the endpoint pins are
not preferences, they are the ball-frame physics boundary conditions, so
"approximately pinned" is a physics error, not a numerical one.

### Why kinematic capture is the authority, and what 4/11 MuJoCo makes means

Owner resolution 1 (2026-08-29) made the kinematic-capture model the sim
authority and MuJoCo contact advisory-only. Phase 1 is the first evidence for
that call rather than an assertion of it.

The kinematic chain scores **capture distance 0.000 mm on all 11 points** — the
cup opening arrives exactly where the plan says, because the decomposition is
exact and the QP pins the catch position. MuJoCo scores **4/11 makes** on the
same trajectories. Those two numbers do not disagree about the robot; they
disagree about the **contact model**, and the disagreement was **predicted** —
it is the Rung-3 P2 signature, the same fragile equilibrium that dropped MAKE to
0/12 whenever the whole-cycle sim catch was cleaned up.

Gating on the contact model would optimise the planner against the low-fidelity
element. The three grounds, restated as root causes rather than as an appeal to
the resolution: `sim/toss_gate.py` already avoids MuJoCo contact deliberately for
this reason; hardware catches are already smooth (standing bench fact), so the
sim contact model is known to be the pessimistic party; and a gate that can be
passed by re-tuning contact parameters is not a gate. The hardware ladder
(Phase 5) is the seating authority, and 4/11 is recorded so that a future reading
of it as "the catch does not work" has this paragraph to argue with.

### Why the cup acceleration boxes ship off

They were built to buy flight time and **do not**. The 0.8 s flight needs 134
m/s² below release as a matter of ballistics; bounding the cup acceleration does
not reshape the trajectory into feasibility, it makes the planner **refuse
earlier**. Shipping them on would trade a late, informative refusal for an early,
less informative one and buy nothing. Shipping them off with `parity 0.0` proven
keeps the lever available for Phase 4 (where an early refusal may be worth more
than a late one, on the replan path) without letting anyone read their existence
as a solution to the tier lid.

### Tradeoffs accepted

- **The 0.80 s flight lid is accepted as-is.** Raising the hand acceleration cap
  is the wrong lever — 3500/3900 is bounded by the C-HAND-2 authority bound
  3925.5 rev/s², so there is no headroom to take. Release height is the lever,
  and it is a Phase 5 tier-planning input rather than a Phase 1 defect.
- **The shipped-limit question is deferred to the owner rather than resolved by
  the harness.** The gate runs catch-capable session limits and *reports* the
  shipped-limit verdict beside it. Tuning the harness past a structural
  disagreement is exactly the "just relax this one invariant" move the
  engineering-philosophy section forbids.
- **`SetTrajectoryLimits.srv` stays 6-DoF this phase.** Hand session limits are
  YAML-set until Phase 2/4 pays for the interfaces rebuild.
- **The zero-segments blindness is pinned rather than fixed.** `validate` and
  `validate_follow` silently pass a `CyclePlan`. Fixing them is Phase 2/4 surface;
  pinning the blindness now means the fix cannot land silently.

### What was ruled out, and on what evidence

| Ruled out | Evidence |
|---|---|
| Trusting G–I's termination as correctness | A 0.40 s cycle terminated with every inequality satisfied and equality residual **121** (working set 38/48, `NᵀH⁻¹N` singular) |
| Sampling hand extrema on the knot grid | Under-measures accel **38 %** (2763.5 vs 4432.5 rev/s²) ⇒ false accept against the 3500 cap |
| Per-axis tilt rate limiting | Leaked **12.124°** past the 12° 2-norm cap; caught by the T-U3 cap assertion |
| Iterated projection for accel-bounded banking | **38.9 rad/s² @ 16 sweeps, 5.5 @ 800** vs target 5.33 — unbounded iteration, pins only approximate |
| `shaping._cup_axis_xy` in `decompose` | Differs from `cup_axis` at **8.3e-17**; `realize_tilted` uses the matrix form, so bit-parity requires it |
| Cup acceleration boxes as a flight-time lever | 0.8 s needs **134 m/s²** below release regardless; the box refuses earlier, it does not reshape |
| MuJoCo contact as the gating authority | **4/11 makes** against **0.000 mm** kinematic capture — the predicted Rung-3 P2 contact-model signature |
| Delegating `validate_cycle` to `validate` | `CyclePlan.segments` is empty ⇒ `validate` measures nothing and reports OK |

## Deliverables

**New:**

- `motion/trajectory/cup_cycle.py`, `cup_realize.py`, `cycle_plan.py` — the
  planner core.
- `sim/cycle_gate.py` — the Phase 1 sim phase gate.
- `tests/motion/test_cup_cycle.py` (T-U1, T-U2), `test_cup_realize.py`
  (T-U3, T-U4), `test_cycle_plan.py`, `test_validate_cycle.py` (T-U5);
  `tests/sim/test_cycle_gate.py`.
- `tools/probes/capture_cup_cycle_refs.py` + `tools/probes/data/cup_cycle_qp_refs.npz`
  — the committed CasADi reference fixture and its regenerator (promoted from
  Phase 0's throwaway `/tmp/probe_qp_ref.py`, per `tools/probes/README.md`).

**Modified:** `motion/trajectory/feasibility.py` (`validate_cycle`, +452 lines),
`limits.py` (the hand tier), `__init__.py` (export);
`config/hardware_config.yaml` + the six regenerated artifacts;
`tests/motion/test_shaped_batch.py` and `test_trajectory_feasibility.py`
(`TrajectoryLimits` construction); `tools/probes/README.md`;
`plans/active/unified-7dof-planner.md`, `plans/active/INDEX.md`,
`logbook/INDEX.md`.

**Not in the tree, by design:** `/tmp/probe_validate_codes.py` — a one-off code
driver, not a reusable harness; its confirmed recipes are a table in
`tests/motion/test_validate_cycle.py`'s docstring.

## Verification

- **Inaugural baseline reference** — the plan's baseline was recorded at Phase 0:
  (2026-08-30, `./run_tests.sh`, **6253 passed / 4 skipped, 269.50 s parallel
  phase + empty serial phase, total 283 s — PASS**) at HEAD
  `a6877ebf468778270f026247b112c17ce6598b6b`.
- WP1: (2026-08-30, `python -m pytest tests/motion/test_cup_cycle.py -q`,
  **51 passed in 3.83 s**).
- WP2: (2026-08-30, `python -m pytest tests/motion/test_cup_realize.py
  tests/motion/test_cycle_plan.py -q`, **60 passed in 0.72 s**).
- WP3 config freshness: (2026-09-01, `python config/generate_config.py --check`,
  **CONFIG FRESH, 14 artifacts**).
- WP3 motion tier: (2026-09-01, `python -m pytest tests/motion/ -q`,
  **2108 passed / 3 skipped in 325.76 s**).
- WP3 ROS tier: (2026-09-01, `python -m pytest tests/ros/ -q`,
  **2630 passed / 1 skipped in 334.85 s**).
- WP3 config drift: (2026-09-01,
  `python -m pytest tests/firmware/test_config_drift.py -q`, **21 passed**).
- WP4 the gate itself: (2026-09-01, `python sim/cycle_gate.py`, **PASS**, 11
  points, capture 0.000 mm, parity 0.0, banking 11/11, worst runway margin
  106 mm, MuJoCo advisory 4/11 makes, **10.5 s wall**).
- WP4 scoped: (2026-09-01, the 5-file scoped command, **164 passed in 8.07 s**).
- WP4 motion tier after the gate fixes: (2026-09-01,
  `python -m pytest tests/motion/ -q`, **2115 passed / 3 skipped in 331.03 s**).
- Pre-commit gate (2026-09-01, `./run_tests.sh --full`, after the audit fixes):
  **6847 passed, 1 failed, 4 skipped, 3 xfailed in 556.08 s** (parallel) + 9
  passed (serial, 42.19 s), total 604 s — RESULT FAIL **solely on the
  pre-existing, tracked `tests/sim/test_solver_failures.py::TestWarmStartIntegrity`**
  (dormant MPC; zero `controller/` files in this diff; Hypothesis's local DB now
  replays the ci-deep example at every tier). **Owner-acknowledged 2026-09-01**
  (commit-with-acknowledgement chosen over fix-first); the fix is the next
  scoped task — see § Tracking.

## Outcome

**Phase 1 is COMPLETE.** The planner core exists, is pure-Python/numpy-only under
Python 3.8, and the sim phase gate passes on the production chain.

- **The Rung-3 blocker is answered in sim.** Capture distance is 0.000 mm on all
  11 points with the runway constraint active and **no ceiling-overshoot slam
  anywhere** — the P2 failure's "the slam is the seat's runway" premise is
  removed by construction rather than tuned around.
- **Banking is demonstrated to pay.** It beats the level arm on apparent-gravity
  misalignment **11/11** with matched pins, and the zero-banking arm reproduces
  `realize_tilted` at **exactly 0.0**.
- **Phase 2 is cleared to start** (software-only: the v6 wire and the host
  7-channel path). Phases 1–2 are permitted while the plan is `proposed`;
  **Phase 3 onward requires `active`**. (Addendum, same day: the plan was
  promoted to `active` on 2026-09-01, so the Phase 3+ status gate is met.)
- **Four owner decisions are recorded below, and all four are RESOLVED**
  (owner, 2026-09-01) — none of them changes shipped behaviour.

## Open Questions — owner decisions, all four resolved 2026-09-01

1. **The shipped-limit verdict.** Every gate cycle reads `LIMIT_JERK` at the
   shipped leg jerk of **30 000 mm/s³**, and the reason is structural, not
   marginal: with z pinned, the cup's z channel carries the whole launch at up to
   **1433 m/s³** of jerk, and **any** platform tilt leaks `sin(tilt) × 744.3 mm`
   of that into the centroid xy through the rotation-centre lever. Measured
   2026-09-01 on the demo cycle at constant tilt with a motionless schedule:
   **0° → 17 005**, **3.5° → 28 970**, **4° → 33 339 (over)**, **12° → 107 815
   mm/s³**, and the finite-difference jerk is **mesh-converged** (16 967 at 2
   samples/knot, 17 035 at 32), so it is the trajectory's real jerk and not a
   reconstruction artifact. **The shipped limits admit banking only below about
   3.5° of tilt.** The gate therefore runs the catch-capable session limits
   (250 / 3000 / 150 000 — `toss_gate`/`reload_gate` parity), and the shipped
   verdict is pinned by a test and reported as `shipped_limit_verdicts` beside
   the gating one. **The owner's decision: raise the shipped limits, or make
   unified mode always ride session limits.** A harness may not tune past this.
   **RESOLVED (owner, 2026-09-01): nothing shipped changes.** Unified sittings
   raise the session limits at session start — the exact pattern the existing
   toss sittings already use — and the concrete jerk value is decided
   empirically at the UH-3 banked-carry rung. Context: at the current 30 000
   working point banking is legal only below ~3.5° of tilt; refusals stay loud
   and `validate_cycle` enforces whatever ceiling the operator sets.
2. **The 0.9 s flight advisory band.** It needs **4371 rev/s²** against the 3500
   cap, so it is carried as advisory. A test **fails if it ever fits** — i.e. if
   a future change silently makes it plannable, that is surfaced rather than
   celebrated.
   **CLOSED — informational (owner, 2026-09-01), not a decision.** The 0.80 s v1
   lid is deliberate conservatism (z pin, the 3500 cap below the metal, a
   conservative cup region, free-fall handover shaping ⇒ ~17 cm effective
   runway, about half the legacy throw's). Named levers if the tiers must ramp:
   release-height / region shaping (measured strongest), z-float (ships off),
   raising the cap toward the 3900 ceiling, a Scope-B joint launch. The legacy
   throw path stays flyable until Phase 6; if the levers have not recovered
   current tiers by the end of the Phase 5 ladder, that is an explicit owner
   go/no-go input at Phase 5/6. The advisory test stands.
3. **The smoothing accel cap yields to the endpoint pins on short cycles.**
   **2 of 30** grid cases land ~6 % over. The pins are physics boundary
   conditions and win by design; `validate_cycle` stays the authority on whether
   the resulting plan is flyable.
   **RESOLVED (owner, 2026-09-01): recorded, no action.** Endpoint pins always
   win and the smoothness cap is best-effort — it is not a guarantee and must
   not be treated as one. `validate_cycle`, which measures the actual resulting
   motion, remains the sole authority.
4. **`v_match` is deferred, mirroring `toss_gate`.** It measures a uniform
   **0.316 = 1 − `catch_slider_vel_ratio`** by design — the cup QP matches catch
   velocity *softly* at 0.7, so a hard band would fail every cycle for a designed
   reason rather than a defect. Reported, not gated.
   **RESOLVED (owner, 2026-09-01): recorded, no action.** The ~30 % velocity
   difference at the catch is the designed 70 % softening ratio inherited from
   the existing system, so no naive velocity-match gate will be added — it would
   fail every catch for a designed reason, the same call `sim/toss_gate.py`
   made. Hardware remains the judge of catch quality.

## Tracking (orthogonal to this phase, unfixed per the phase charter)

The nightly is **RED 2026-09-01**:
`tests/sim/test_solver_failures.py::TestWarmStartIntegrity` (`RED 6795/6803
passed, 1 failed, 3 xfailed, 4 skipped, 2026-09-01T04:04`). It is a
**dormant-MPC Tier-1a Hypothesis-stateful test, ci-deep only**, touching
`controller/mpc.py:1041/1048` — nothing Phase 1 changed is on that path. It needs
its own scoped session and is recorded here so the RED is not invisible.
