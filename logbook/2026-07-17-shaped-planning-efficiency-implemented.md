---
title: Shaped-planning efficiency — batched 1600-sample gate + retiming-model duration search (~12× faster, 7× more honest on jerk)
type: optimization
date: 2026-07-17
status: resolved
phase: "MVP trajectory bringup — shaped-planning efficiency"
related_plan: shaped-planning-efficiency.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/shaping.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/retime.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - tests/motion/test_shaped_batch.py
  - tests/motion/test_retime.py
  - tests/motion/test_trajectory_feasibility.py
  - tests/motion/test_trajectory_shaping.py
  - tests/ros/test_trajectory_node.py
  - plans/archived/shaped-planning-efficiency.md
commits:
  - 4acaefe
  - 54c1e75
  - 806e8fb
subsystem:
  - motion
  - config
tags:
  - performance
  - testing
  - safety
  - kinematics
---

# Shaped-planning efficiency — batched 1600-sample gate + retiming-model duration search

## Summary

`lean_gain 0.6` became the shipped default on 2026-07-17
(`logbook/2026-07-17-s4-closed-working-point-persisted.md`), so **nearly every
service `go_to_pose` move now pays shaped-planning cost**. Before this work a
shaped `build_move` ran **6 analytic `validate` passes** at ~185 ms each on the
Jetson — **~1.1 s per shaped move** — and the gate it ran under-measured the
binding shaped leg-jerk constraint by ~23 % versus a dense reference (it silently
under-stretched the exact "sharp" lean the operator had reported).

Three phases landed, in composition:

- **Phase 1a (`4acaefe`)** — a numpy-vectorised twin of the shaped feasibility
  gate (`batched_shaped_states` + `_validate_shaped_batched`), at the *current*
  n=200 mesh. Pure parity refactor, zero emitted-number change. Per-pass
  **185.6 → 7.5 ms** at n=200.
- **Phase 1b (`54c1e75`)** — the accuracy fix: `_SHAPED_VALIDATE_SAMPLES`
  **200 → 1600**, now affordable on the vectorised path. Shaped FD-jerk
  under-measurement **23 % → 3 %** vs a 6400-sample reference; jerk-bound lateral
  moves plan **+4.9 % longer** (the intended fix for the silent under-stretch).
  Per-pass ~39 ms at n=1600.
- **Phase 2 (`806e8fb`)** — a retiming-model duration search (`retime.py`):
  a per-sample `u = 1/T` polynomial model picks the smallest feasible duration in
  **one** verify pass through the UNCHANGED gate, replacing the 6-pass
  stretch+bisection loop. Behind `JB_TRAJ_RETIME_MODEL` (default ON). Shaped
  `build_move` **228.7 → 90.9 ms (2.52×)**; durations uniformly **+2.0 % over
  optimum** (the legacy bisection was +1.2 % to +14.7 %); **0 verify rejections**
  over 54 optimality + 120 fuzz cases.

**Cumulative:** shaped `go_to_pose` planning **~1.1 s (2026-07-16 evening) →
~91 ms (~12×)**, unshaped ~100 ms, with the binding jerk constraint measured
**7× more honestly**. Every returned plan is still gate-verified by the same
analytic `validate` — the model only *chooses* T, the gate remains the source of
truth. Status **resolved** (everything is software-verified; hardware behaviour
is unchanged bit-for-bit from Phase-1b's gate — only the planned duration is
shorter and planning is faster). Deployment for the operator:
`colcon build --packages-select jugglebot` + relaunch (Python-only; the YAML flag
regenerates).

## Motivation

The immediate trigger is the `lean_gain 0.6` default flip, but the deeper reason
is a genuine tension the operator posed as a *binding constraint*: **more
efficiency WITHOUT compromising numerical accuracy.** The lean-latency
investigation (`logbook/2026-07-16-lean-planning-latency-and-boundary-step.md`)
had already sped up the Jacobian and dropped the unshaped mesh to 80, but its
Open Questions named the two levers that would actually close the shaped-path gap:
a **vectorised shaped gate** ("~100× endgame") and an **analytic shaped stretch
seed**. Both landed here. The prior arc also flagged that the shaped path was
**anti-conservative on jerk sampling** (~22–27 % under-measured at 200 samples) —
a latent accuracy problem on the exact lean path the operator called "sharp".

So the goal was never "just make it faster". It was to make it faster **and**
fix the accuracy problem — and to prove the two are not in tension (see
Discussion). The retiming search then recovers the speed the accuracy mesh costs,
without ever bypassing the gate.

## Approach

**Phase 1a — vectorise the shaped gate at bit-parity (n=200).** The scalar
`_ShapedPlan.state_at` recomputes the quintic Hermite coefficients, the C2 lean
plateau window, the tilt/shift superposition and the 5° cap *per sample* in a
Python loop — the dominant cost. `batched_shaped_states(plan, ts)` (`shaping.py`)
evaluates the **identical** shaping law over a whole time grid: coefficients once
per segment, smoothstep/window/superposition/cap as clamped `np.where` /
product-rule array ops. `feasibility._validate_shaped_batched` runs the leg
vel/acc/jerk and condition SVD over that grid (batched Jacobian + Jacobian-dot,
central FD dt=1e-7 identical to the scalar chain; knot-step pass bit-identical at
its fixed 25 ms spacing). The scalar `state_at` and scalar `validate` stay the
frozen references. Reached only via the existing `isinstance(_ShapedPlan)` branch
of `validate` — `validate_follow`'s shaped-plan `TypeError` guard is untouched, so
the single-enforcement-point contract holds.

**Phase 1b — bump the accuracy mesh 200 → 1600.** One constant. The shaped FD
jerk peak under-converges (the C2 lean window concentrates high `w″` curvature in
the 15 % edge ramps; coarse meshes read a *smaller* jerk ⇒ the planner stretches
*less* ⇒ the emitted plan carries more true jerk than the gate believes). At 1600
the FD jerk peak is within ~3 % of a 6400-sample reference (convergence:
400 → −15 %, 800 → −7 %, 1600 → −3 %, 3200 → −1 %); `_VALIDATE_JERK_MARGIN` (1.05)
covers the residual, so the gate stays strictly conservative. A shaped request
below 1600 is clamped UP to the floor; an explicit denser request (a 6400-sample
convergence test) is honoured.

**Phase 2 — retiming-model duration search.** A shaped rest-to-rest move is a
one-parameter family in `u = 1/T` on a fixed s-grid: the base quintic's leg peaks
scale exactly `1/Tⁿ`, and the windowed lean superposes as fixed `u`-powers
(pose ∝ u², twist ∝ u³, accel ∝ u⁴), so leg vel/acc/jerk are per-sample
polynomials in `u`. `retime.build_model(...)` extracts the coefficient arrays on
the batched shaped evaluation at n=1600 (built at a **cap-free** `T_ref` so the
extracted lean geometry is un-capped ⇒ conservative in the cap region);
`RetimeModel.min_feasible_T` is a monotone bisection on `u`;
`propose_move_duration` returns `T_final = max(T_min·1.02, min_move)`. The planner
runs **one** `validate` pass on the resulting plan. If that verify rejects (or the
model cannot bracket / the seed is moving / the flag is off / the plan is
unshaped), it falls through to the proven legacy stretch+bisection loop. The gate
is never bypassed: `_try_retime_model` never returns an unvalidated plan.

## Benchmarks

Per-pass and per-move numbers are measured on this Jetson (offline harnesses that
import the live repo; the durable numbers are here because the harnesses live on
volatile `/tmp`).

| stage | shaped `build_move` | passes | shaped-jerk error vs 6400-ref | unshaped |
|---|---|---|---|---|
| **Before (2026-07-16 eve)** | ~1.1 s | 6 @ ~185 ms | **−23 %** (under-measures) | ~100 ms |
| **After 1a** (vectorise @ n=200) | ~45 ms | 6 @ **7.5 ms** | −23 % (same mesh, bit-parity 1.4e-10) | ~100 ms |
| **After 1b** (n=1600) | ~233 ms | 6 @ **~39 ms** | **−3 %** (7× better; strictly more conservative) | ~100 ms |
| **After 2** (retiming search) | **90.9 ms** | ~2 gate-equiv | −3 % (verify at n=1600) | ~100 ms |

- **Sampler parity (1a):** `batched_shaped_states` vs scalar `state_at` — pose
  **7.1e-15**, twist **5.7e-14**, accel **4.5e-13** across a move × gain ×
  duration grid; batched gate == scalar `validate` at equal mesh to **1.41e-10**
  (shared 1e-7 J-dot FD). Cutover changes only the deliberate mesh, nothing else.
- **Accuracy (1b):** jerk-bound lateral moves plan **+4.9 % longer** under the
  denser mesh (T ∝ jerk^(1/3) on the ~25 % more jerk now sized); conservativeness
  corpus **0 violations** (`@1600`-accept implies `@200`-accept everywhere).
- **Duration optimality (2):** model `T_final` is **+2.0 % over the
  gate-boundary minimum**, uniformly (all from the 1.02 safety inflation); the
  legacy 4-bisection loop overshot **+1.2 % to +14.7 %**. So moves are both faster
  to *plan* and shorter to *execute*. **0 verify rejections** over a 54-case
  optimality corpus + a 120-case fuzz corpus; model-predicted peaks land within
  **<0.3 %** of the gate's.

## Discussion

**(a) Why accuracy and speed were NOT in tension here.** The operator's binding
constraint sounded like a classic trade — spend compute to get accuracy, or spend
accuracy to get speed. It wasn't, and the reason is worth stating precisely:
**vectorisation decoupled the mesh from the cost.** Once the shaped gate runs in
numpy (Phase 1a), the per-sample price collapses ~25× (185.6 → 7.5 ms), so the
accuracy mesh becomes a nearly-free knob. The 200 → 1600 bump (Phase 1b) is the
*whole* accuracy fix, and it costs ~39 ms/pass — a number that would have been
**~300 ms/pass and utterly unaffordable on the pre-1a Python loop** (1600 × the
old per-sample cost). The mesh bump was only *thinkable* because 1a happened
first; that is the entire argument for landing the batched gate before the mesh
bump, and it is why Phase 1a alone was explicitly **not** a valid stopping point
(it keeps the 23 % under-measure). Then Phase 2 recovers most of 1a's raw speed by
cutting the pass count 6 → ~2, and it does so **without touching accuracy at all**:
the retiming model never gates anything — it *proposes* a duration and the
unchanged `validate` at n=1600 accepts or rejects it. Accuracy is a property of the
gate; speed is a property of how few times you call it. Because those live in
different places, both improved at once. The end-state is simultaneously the
fastest, the most accurate, and the tightest on duration.

**(b) Two plan hedges were empirically false — a lesson about @denser-reference
framings.** The plan proposed two natural-sounding acceptance framings that turned
out wrong when actually run, and correcting them (rather than encoding them)
mattered:

1. *"@6400 never accepts what dense rejects"* as an acceptance oracle. Empirically
   **false ~1/90 near the jerk boundary** — a denser reference is a *convergence*
   reference, not an *acceptance* oracle. Point-sampled analytic maxima on a finer
   grid can land nearer a sharp peak and flip a marginal accept/reject either way;
   density buys convergence-toward-truth, not a monotone accept relation. The
   committed corpus therefore asserts the **true cutover property** — `@1600`-accept
   ⟹ `@200`-accept, plus FD-jerk mesh-monotonicity — with the ~3 % residual covered
   by the 1.05 margin.
2. *vel/acc peaks are mesh-monotone*, like jerk. **False.** Only the FD **jerk**
   is systematically mesh-monotone (it resolves the same finite curvature ever
   more completely); analytic vel/acc **point-samples** can read *higher* on a
   coarse grid if a coarse node happens to sit nearer the continuous peak. The
   committed corpus asserts jerk monotonicity **only**.

The lesson: **"just use a denser reference as ground truth" quietly assumes the
quantity converges monotonically to the reference — true for a finite-difference
of a bounded quantity, false for point-samples of a continuous curve and false as
an accept/reject oracle near a boundary.** When a plan hedges with "the denser
mesh is the oracle", verify *which* property (convergence vs acceptance vs
monotonicity) you actually need and *which* the quantity actually has. This is the
same discipline the CLAUDE.md "empirical probe before writing threshold tests" rule
protects — the corpus was prototyped in scratchpad before the assertions were
written, which is exactly why both hedges were caught before they became
false-green tests.

**(c) Builder deviations accepted.** Two, both benign and both improvements on the
plan's letter:

- *Phase 1b's parity-test re-anchoring (`_batched_at`).* Phase 1b's clamp — a
  shaped request below 1600 is clamped **up** to the floor — means the public
  `validate` entry can no longer be used to pin the **Phase-1a equal-mesh
  identity** (any n=200 comparison through `validate` now silently runs at 1600).
  The parity tests in `test_shaped_batch.py` were re-anchored onto a
  `_batched_at(plan, limits, geom, n)` helper that evaluates the batched gate at an
  **explicit** mesh, orthogonal to the clamp; the clamp itself is pinned separately
  by the accuracy-floor test in `test_trajectory_feasibility.py`. This keeps the
  Phase-1a parity identity (batched == scalar at equal mesh) independently testable
  after the Phase-1b floor lands on top of it — a cleaner separation than the plan
  described.
- *Phase 2 module/API naming drift.* The plan named a module `retime_model.py`
  with `build_leg_peak_model` / `cap_free_reference_T` / `predict_peaks`. The
  landed module is `retime.py` with `build_model` / `RetimeModel.peaks` /
  `RetimeModel.min_feasible_T` / `_cap_free_reference_T` / `propose_move_duration`.
  Structurally identical (per-sample u-polynomial model, cap-free reference T,
  monotone bisection, one verify) — only the names drifted, folding the peak-model
  and the search behind a `RetimeModel` class + a single `propose_move_duration`
  entry the planner calls. Accepted: the API shape is better and the plan's names
  were provisional.

**(d) What a verify-reject looks like in production.** Because the retiming model
is a *proposal* confirmed by the unchanged gate, the honest failure mode is a
**verify rejection**: the model's `T_final` fails the one `validate` pass. When
that happens the planner falls straight through to the legacy stretch+bisection
loop and returns a correct plan — the only cost is that *that move* plans at legacy
speed (~6 passes) instead of ~2. It is counted:
`_RETIME_STATS = {'model_used', 'verify_rejected', 'model_no_solution'}` in
`planner.py`, incremented in `_try_retime_model`. In the 54 + 120 offline cases the
`verify_rejected` count was **0**, and the model's raw `T_min` matched the
gate-boundary minimum to ~1 ms — but the counter exists precisely because "never
observed offline" is not "cannot happen on hardware with a pose the corpus didn't
cover". A persistent nonzero `verify_rejected` in a session would mean the model is
systematically mispredicting for some move class and silently costing latency —
**worth a diagnostics surface someday** (e.g. a periodic `trajectory/diagnostics`
field or a throttled WARN when the ratio climbs). Noted, **not built** here — the
counters are in-process only, readable by a debugger or a future publisher.

## Verification

**Phase 1a scoped (`4acaefe`, 2026-07-17):** the parity/dispatch suite in
`tests/motion/test_shaped_batch.py` — **33 tests** (sampler parity 7.1e-15 /
5.7e-14 / 4.5e-13; gate parity 1.41e-10 at equal mesh across the ok/reject paths;
`test_shaped_dispatch_is_batched`).

**Phase 1b scoped (`54c1e75`, 2026-07-17):** the conservativeness corpus and
accuracy-floor tests in `test_shaped_batch.py` +
`test_trajectory_feasibility.py` — `@1600`-accept ⟹ `@200`-accept (0 violations),
FD-jerk mesh-monotonicity, and the ≤5 % (measured 3.2–3.7 %) 6400-reference floor.

**Phase 2 scoped (`806e8fb`, 2026-07-17):** `tests/motion/test_retime.py` —
**318 lines / new suite**: optimality vs gate-boundary ground truth (±2 %),
boundary bracketing (a plan at 0.98·T_min rejected), cap-regime conservatism,
rest-seed gating (moving seed → legacy loop), and a hypothesis fuzz asserting the
load-bearing invariant `validate(returned_plan).ok` always holds. Plus the
every-returned-plan-is-gate-verified property proved from code in the audit.

**Full-suite gate at each landing (`pytest tests/ -q`, 2026-07-17):**

- **1a (`4acaefe`): 2860 passed / 1 xfailed in 584.45 s** — the one known
  order/load-flaky allocation test passed isolated (see
  `project_hot_loop_alloc_test_flaky`).
- **1b (`54c1e75`): 2862 passed / 1 xfailed in 608.38 s.**
- **2 (`806e8fb`): 2872 passed / 1 xfailed in 937.31 s** — the final pre-commit
  gate over the whole arc.

## Outcome

Shaped `go_to_pose` planning is **~1.1 s → ~91 ms (~12×)**, the binding shaped
leg-jerk constraint is measured **~7× more honestly** (fixing the silent
under-stretch on the "sharp" lean the operator reported), and planned durations
land **~2 % over optimum** instead of the legacy loop's +1–15 %. No hardware
behaviour changes bit-for-bit from the Phase-1b gate — the retiming model only
picks a (shorter) duration that the unchanged gate accepts, and plans it faster.

`JB_TRAJ_RETIME_MODEL` ships **ON** (`config/hardware_config.yaml`
`trajectory_op.retime_model`); the legacy 6-pass loop is one branch away for a
hardware A/B or a fallback with no redeploy. Deployment:
`colcon build --packages-select jugglebot` + relaunch.

## Open Questions

- **A retime verify-reject has no operator-visible surface.** The
  `_RETIME_STATS` counters are in-process only; a persistent `verify_rejected` in
  the field would silently cost latency (fall-back-to-legacy) with no telemetry.
  A diagnostics surface (periodic field / throttled WARN on a rising ratio) is
  worth building if the model is ever suspected of mispredicting — noted here, not
  built.
- **The vectorised path is single-segment rest-to-rest only.** The lean window is
  per-segment (`shaping.py`), and the retiming `1/Tⁿ` invariance holds only for a
  rest seed. A future multi-segment or moving-seed shaped plan needs a per-segment
  grid loop, a plan-scoped window, and would route to the legacy loop today (the
  rest-seed gate already handles the fall-back correctly).
- **The unshaped path was left untouched** (already fast at 80 samples,
  closed-form vel/acc). Vectorising it is an available follow-on, out of scope.

## Related

- `plans/archived/shaped-planning-efficiency.md` — the plan; phase gates marked
  LANDED with these SHAs and results; ARCHIVED 2026-07-17 after a fresh
  plan-reviewer critique (ARCHIVE_READY, zero blocking items).
- `logbook/2026-07-16-lean-planning-latency-and-boundary-step.md` — the lean
  latency/boundary arc; its Open Questions named the vectorised gate + stretch
  seed that this entry delivers (cross-referenced there).
- `logbook/2026-07-17-s4-closed-working-point-persisted.md` — the `lean_gain 0.6`
  default flip that made shaped planning apply to nearly every service move.
- `tests/hardware/mvp_bench_runbook.md` — S4 lean A/B section, updated with the
  dated ~91 ms shaped / ~100 ms unshaped planning note.
- Offline harnesses lived in the session scratchpad (volatile `/tmp` — the durable
  numbers are in this entry and the plan's prototype-evidence digest).
</content>
</invoke>
