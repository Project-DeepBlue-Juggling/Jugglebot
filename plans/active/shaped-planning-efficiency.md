# Plan: Shaped-planning efficiency — batched dense gate + retiming-invariant duration search

**Status:** PROPOSED — Phase 1a ready to start (created 2026-07-17, from the
two-prototype efficiency exploration; both prototypes measured on this Jetson)
**Branch:** mvp-trajectory-bringup
**Operator constraint (binding):** more efficiency WITHOUT compromising numerical
accuracy — Phase 1b *improves* accuracy (shaped-jerk under-measurement 23% → 3%),
every other phase preserves it exactly.
**Why now:** lean_gain 0.6 is the shipped default (2026-07-17), so shaped planning
cost (~1.1 s/move) applies to nearly every service move.

---

## Recommendation: land the two wins in composition, batched gate first

The two prototypes **multiply**: the batched gate makes each validate pass ~5x cheaper *and* ~7x more accurate; the retiming search cuts the number of passes from 6 to ~2. Composed, shaped `build_move` goes from **~1.1 s → ~0.09-0.11 s (≈10-12x)** while the binding constraint (shaped leg jerk) goes from **~23% under-measured → ~3% under-measured** vs a 6400-sample reference. Both wins independently satisfy the operator's binding constraint ("more efficiency WITHOUT compromising numerical accuracy") — the batched gate *improves* accuracy, the retiming search *preserves* it exactly (its verify pass is the unchanged gate).

**Landing order: batched gate first, retiming search on top.** Rationale by root cause, not sequence-convenience:

1. **The batched gate is the shared substrate.** The retiming model must extract its base/lean decomposition and be verified on the *same* shaped-pose evaluation the gate uses; Phase 1 delivers `batched_shaped_states`, which Phase 2 then reuses for both its geometry build and its verify. Landing the search first would force building the model against the slow scalar `state_at`, then re-baselining onto the batched one — wasted work and a parity seam.
2. **The batched gate improves BOTH axes and is independently testable.** It is a drop-in behind the existing `validate` entry (all callers covered by the single-gate contract), parity-lockable to 1.3e-10 at equal mesh, and strictly more conservative after the mesh bump. It can ship and be validated alone.
3. **The retiming search restructures control flow** (new module, rest-seed gating, 5° cap handling, legacy fallback) — higher risk. It benefits from landing on an already-fast, already-accurate, already-parity-tested gate so its own A/B compares only the search change.

**Split Phase 1 into two commits for rollback granularity:** (1a) vectorize the shaped branch at the *current* n=200 — bit-parity, zero behavior change, pure refactor; (1b) bump `_SHAPED_VALIDATE_SAMPLES` 200→1600 — the accuracy fix, isolated behind a single-constant change, gated by the conservativeness corpus. This makes any future bisection trivial: 1a answers "did vectorization change a number?" (must be no), 1b answers "did the mesh bump change durations?" (yes, ~8% longer on jerk-bound moves, and that is the *intended* fix for the "sharp" lean the operator reported).

**Flags:** no separate flag for Phase 1 — the vectorization is parity-locked and `_SHAPED_VALIDATE_SAMPLES` *is* the revertible knob. Phase 2 lands behind `JB_TRAJ_RETIME_MODEL` (default ON, legacy 6-pass loop one branch away) so a hardware A/B can compare planned durations and emitted leg-jerk directly, and any model pathology falls back to the proven loop without a redeploy.

---

# Plan: Shaped-planning latency — batched gate + retiming-invariant duration search

**Branch:** `mvp-trajectory-bringup` (or a dedicated `traj-shaped-planning-speedup`)
**Status:** ACTIVE (draft)
**Subsystem:** `ros_ws/.../motion/trajectory/` (pure Python, ROS-free)

## Context (measured, this Jetson, 2026-07-16/17)

Lean shaping ships ON at `JB_TRAJ_LEAN_GAIN = 0.6` (shaping.py:1, since 2026-07-17), so **nearly every service `go_to_pose` move pays shaped planning cost**. Today a shaped `build_move` runs **6 `validate` passes** (2 stretch + `_SHAPED_REFINE_ITERS=4` bisections; planner.py:82, 249, 296) at **~185-197 ms/pass** post the 2026-07-16 component-cross Jacobian speedup, i.e. **~1.1-1.18 s per shaped move** (the prompt's "~450 ms/pass, ~1.2-1.5 s" is the *pre-speedup* number; both prototypes re-measured the post-speedup reality). The operator's verdict: still too slow.

The cost lives in `validate` (feasibility.py:212): a per-segment Python loop over `_SHAPED_VALIDATE_SAMPLES=200` samples (feasibility.py:116), each doing `compute_jacobian` + `accel_to_leg_accels` (2 extra FD Jacobians) + a condition SVD, plus `_ShapedPlan.state_at` (shaping.py:284) recomputing quintic Hermite coefficients c2..c5 per sample (`_seg_xy_derivs`, shaping.py:210 — a known hot spot). The 200 floor exists because the shaped FD jerk peak **under-converges** with sample count (window-edge w″ curvature since the C2 plateau window): 200 samples under-measures shaped leg jerk by **~21-23%** vs a 6400-sample reference (feasibility.py:98-116 rationale). The fast `validate_follow` (feasibility.py:442, ~1.6 ms) cannot gate shaped plans — it raises `TypeError` on a `_ShapedPlan` (feasibility.py:464) because its batched sampler evaluates the *base* quintic, under-measuring the lean.

**Two independent, composing wins were prototyped and measured (scratchpad, no repo edits):**

- **Batched shaped gate** — a numpy-vectorized twin of the shaped-plan gate. Reproduces `validate` at equal mesh to **1.30e-10**; the batched sampler matches `_ShapedPlan.state_at` to machine precision (pose 7.1e-15, twist 5.7e-14, accel 4.5e-13). At n=1600 it runs **39.7 ms/pass vs 185.6 ms (4.7x)** while measuring shaped jerk to **~3% of the 6400-ref vs today's ~23% (7x more accurate)**. At n=200 (bit-parity mesh) it is **7.5 ms/pass (24.8x)**.
- **Retiming-invariant search** — exploits that a shaped rest-to-rest move is a one-parameter family in `u=1/T` on a fixed s-grid (base leg peaks scale exactly 1/Tⁿ; the windowed lean superposes as fixed u-powers: pose∝u², twist∝u³, accel∝u⁴). Leg vel/acc/jerk become per-sample polynomials in u, so the smallest feasible T is a monotone root-find + **one** verify pass = **2 gate-equivalent passes vs 6**. Measured **~410 ms vs ~1180 ms (2.6-3.1x)** with a Python-loop model build; **0/42 verify failures**; duration optimality **+1.99% vs today's +7.94%** (moves are both faster to plan AND shorter to execute).

## Binding constraint (operator-stated)

More efficiency **without compromising numerical accuracy**. Every phase must **hold or improve** gate accuracy vs today's 200-sample Python loop. This is enforceable because: Phase 1 *improves* it (23%→3% under-measure, and is strictly more conservative — 0/400 jerk violations in a random corpus); Phase 2 *preserves* it exactly (the verify is the unchanged batched gate at the same mesh — the model only picks T, the gate remains the source of truth).

## Phase 1a — Vectorize the shaped gate at bit-parity (n=200)

**Goal:** replace the Python per-sample shaped loop with the batched machinery, at the *current* mesh, so this commit changes NO emitted number.

**Files/tasks:**
- `shaping.py`: add `batched_shaped_states(seg_or_plan, ts) -> (pose,twist,accel) each (N,6)` — a vectorized `_ShapedPlan.state_at`. Precompute c2..c5 **once** per segment (vs per-sample today, shaping.py:210); batch `_lean_window` + smoothstep via clamped `np.where`; batch the full product-rule windowing (shaping.py:314-322) and the per-sample 5° cap (shaping.py:336-340). Keep scalar `state_at` as the pinned reference.
- `feasibility.py`: add `_batched_jacobian(poses, geom)` and `_batched_jacobian_dot(poses, twists, geom)` (central FD, dt=1e-7, identical to `ik_solver.compute_jacobian_dot`); reuse the shipped `_batched_leg_vectors` (feasibility.py:409) and `_batched_condition` (feasibility.py:424). In `validate`, when `isinstance(plan, _ShapedPlan)` (the existing branch, feasibility.py:229), dispatch to a vectorized pass over `batched_shaped_states` instead of the per-sample loop (feasibility.py:238-289). Leg vel = `einsum(J, twist)`; leg acc = `einsum(J,accel)+einsum(Jdot,twist)`; jerk = FD of leg-acc on the dense grid × `_VALIDATE_JERK_MARGIN`; knot-step pass unchanged (bit-identical, fixed 25 ms spacing). Run condition SVD **full** (all N) for strict parity.
- Non-shaped path: **untouched** (already fast at 80 samples; closed-form vel/acc, ≤3.7% jerk). Vectorizing it is an optional later follow-on, out of scope here.

**Gate (must all pass, cite date/command/result triple):**
- Sampler parity in `test_trajectory_shaping.py`: `batched_shaped_states` vs `state_at`, ≤1e-12 target (pose 7e-15, twist 5.7e-14, accel 4.5e-13) across ≥6 moves × 4 gains {0,0.3,0.6,1.0} × 3 durations.
- Gate parity in `test_trajectory_feasibility.py`: batched shaped branch == python `validate` at the SAME n∈{200,800} to ≤1e-9 (measured 1.30e-10).
- Full `pytest tests/ -q` green.

**K-contract:** shaping still happens at plan construction (`planner._build_rest_move` wraps `shaper.shape()` before validate, planner.py:216-225 — untouched); the batched path samples the already-constructed `_ShapedPlan`, so peaks are measured on the SHAPED plan. The `validate_follow` `TypeError` guard (feasibility.py:464) stays; the batched shaped gate is reached ONLY via the `isinstance(_ShapedPlan)` branch of `validate`, never wired into `validate_follow`. Single enforcement point preserved.

## Phase 1b — Bump the shaped mesh 200 → 1600 (the accuracy fix)

**Goal:** flip `_SHAPED_VALIDATE_SAMPLES` 200→1600 (feasibility.py:116) — now affordable (39.7 ms vs 185 ms/pass) — cutting shaped-jerk under-measurement 23%→3%. Update the constant's long rationale block to record the new ~3%-vs-6400 headroom.

**Behavioral change (intended, surfaced in the logbook):** the gate now measures ~25% MORE shaped jerk, so jerk-bound shaped moves stretch **~8% longer** (T∝jerk^(1/3)). This is the **fix** for the operator's "sharp" lean, not a regression — today's gate silently under-stretches and emits more true leg jerk than it believes on the exact lean path.

**Gate:**
- **Conservativeness corpus** in `test_trajectory_feasibility.py` (the cutover-gating differential test): N≥400 random moves, assert batched(1600) peak jerk ≥ python(200) peak jerk everywhere (measured 0/400 violations; ratio median 1.257, max 1.362). Also assert vel/acc/step batched(1600) ≥ python(200) (free bonus — sampled maxima resolve better).
- **Convergence-reference test:** batched(1600) jerk error vs an n=6400 reference ≤3% on the corpus (pins the mesh choice against future regression).
- **build_move monotonicity:** for a fixed move set, planned durations under the new gate are equal-or-longer (never shorter) than today's — the "no silent under-stretch" invariant.
- Full `pytest tests/ -q` green.

**No flag:** `_SHAPED_VALIDATE_SAMPLES` is itself the single revertible knob; the vectorization it rides on is parity-locked by 1a.

## Phase 2 — Retiming-invariant duration search (2 passes, not 6)

**Goal:** replace the shaped branch of the duration search (2 stretch + 4 bisection) with a per-sample u-polynomial model + one verify. `validate`/`validate_follow` untouched — the gate stays the gate.

**Files/tasks:**
- New `trajectory/retime_model.py`:
  - `build_leg_peak_model(pose,twist,accel,target,shaper,geom,T_ref,n_samples)` → coefficient arrays `(av1,av3,aa2,aa4,aa6,aj3,aj5,aj7)`. **Reuse Phase 1's `batched_shaped_states` + `_batched_jacobian`/`_batched_jacobian_dot`** so the model is built on the exact shaped-pose evaluation the verify uses (no scalar/batched skew), at n=1600, in ~40 ms.
  - `cap_free_reference_T(...)` — cheap pose-space bump (~18 ms, no Jacobian) to pick a `T_ref` where the 5° cap is inactive, so the extracted lean geometry is un-capped (model then over-predicts in the cap region — conservative).
  - `predict_peaks(model,T)` and `min_feasible_T(model,limits)` — monotone bisection on u (60 iters, deterministic). Include a vel-scaled step term for robustness (step-bound is otherwise covered by the verify backstop).
- `planner.py`:
  - `_min_feasible_move` (planner.py:249): when `shaped and _is_at_rest(twist,accel)` (planner.py:92) and `JB_TRAJ_RETIME_MODEL` on → model path: `T_ref = cap_free_reference_T(...)`; `model = build_leg_peak_model(...)`; `T_min = min_feasible_T(...)`; `T_final = max(T_min*1.02, min_move_duration)`; `report = validate(_build_rest_move(...,T_final), limits, geom)`; if `report.ok` return, else a small corrective `_stretch_factor` loop, then re-raise `TOO_FAST`/spatial exactly as today.
  - **Fallback:** moving seeds (base path bends with T — the 1/Tⁿ invariance holds only for a rest seed, planner.py:56-62), model non-convergence, or `JB_TRAJ_RETIME_MODEL` off → today's iterate-stretch loop (`_min_feasible_move` body + `_refine_shaped_min`). Keep `_stretch_factor` (planner.py:228) and `_refine_shaped_min` (planner.py:296) for the fallback.

**Gate:**
- **Optimality/parity** (primary regression, `test_trajectory_planner_move.py`): over the move×tier corpus, assert `model T_final` passes `validate` and `|T_final − ground_truth_minT|/GT ≤ 2%` (GT = bisection on the batched `validate`). Measured model +1.99% vs today +7.94%.
- **Boundary bracketing:** returned plan `report.ok`; a plan at `0.98·T_min` is rejected.
- **Cap regime:** a high-gain/large-lateral move where the cap binds inside the feasible band → still gate-passing (conservative over-prediction).
- **Rest-seed gating:** a moving-seed shaped move routes to the legacy loop and still passes.
- **Hypothesis fuzz** over targets/limits: `validate(returned_plan).ok` always (the load-bearing invariant).
- Full `pytest tests/ -q` and one `--hypothesis-profile=ci-deep` green (cite the triple).

**Flag:** `JB_TRAJ_RETIME_MODEL` (default ON), legacy loop one branch away, for a hardware A/B comparing planned durations and emitted leg-jerk. Because the verify is the unchanged gate, every emitted plan is bit-identical-gated to Phase-1b behavior — the only observable change is a *shorter* planned duration and faster planning.

## Verification (incl. parity strategy)

**Parity anchor:** the scalar `validate` + scalar `_ShapedPlan.state_at` are the frozen references. Phase 1a proves batched == scalar at equal mesh (1.3e-10); Phase 1b proves the mesh bump is strictly more conservative (0/400 jerk violations); Phase 2 proves the model's chosen T is one the unchanged gate accepts and within 2% of the gate-boundary minimum. Promote the scratchpad corpus (`bsg/corpus.py`, `retime/decision.py`) into committed tests as the durable parity harness.

**Differential/conservativeness strategy:** the cutover to a more-accurate gate (1b) is safe *because* it measures ≥ jerk everywhere — batched-accept ⟹ python-accept. The random-corpus differential test (N≥400) is the gate on 1b; the optimality-vs-ground-truth corpus is the gate on Phase 2. Each test cites the (date, command, result) triple per CLAUDE.md.

**K-contract (shaping-before-validate) preserved at every phase:** plan construction shapes before validate (planner.py:216-225, untouched); the gate always sees the shaped plan; `validate_follow`'s shaped-plan `TypeError` is never weakened; the batched shaped gate is reached only via the analytic `validate`'s shaped branch.

**Logbook:** one entry per phase with a real Discussion — 1b's Discussion must record the intended ~8% duration lengthening as the fix for the "sharp" lean (a hypothesis-withdrawn-style note: today's gate under-stretched); Phase 2's must record why the per-sample u-polynomial model beats the aggregate 2-point power-law refit (`retime/approach2.py`: 39/42 gate-rejects, T collapses to 0.058 s — the binding sample switches with T and the cap distorts the aggregate power).

## Out of scope

Firmware; gains / lean-gain value; the ZOH torque staircase; move chaining / multi-segment shaped plans (the lean window is per-segment, shaping.py:117-121 — a plan-scoped window is future work; today's shaped path is single-segment rest-to-rest only); vectorizing the unshaped path (already fast at 80 samples).

---

## Projected end-state (measured basis)

## Projected shaped `build_move` planning time and gate accuracy after each phase (this Jetson)

| stage | shaped build_move | passes | shaped-jerk error vs 6400-ref | unshaped build_move | conservativeness |
|---|---|---|---|---|---|
| **Today** | ~1.1-1.18 s | 6 @ ~185-197 ms | **−23%** (under-measures) | ~100 ms (2 @ 80-smp py) | under-stretches (silent) |
| **After 1a** (vectorize @ n=200) | **~45 ms** (6 @ 7.5 ms) | 6 | −23% (UNCHANGED — same mesh) | ~100 ms (untouched) | identical to today (bit-parity 1.3e-10) |
| **After 1b** (n=1600) | **~236 ms** (6 @ ~40 ms) | 6 | **−3%** (7x better) | ~100 ms (untouched) | strictly more conservative (0/400 jerk violations; ~8% longer jerk-bound moves) |
| **After Phase 2** (retiming search) | **~90-110 ms** (≈2 gate-equiv passes) | ~2 | −3% (verify at n=1600) | ~100 ms (untouched) | conservative by construction (verify is the unchanged gate); durations tighter (+2% over optimum vs today's +8%) |

**Net for the operator:** shaped `go_to_pose` planning **~1.1 s → ~0.1 s (≈10-12x)**, and the motion is *calmer* not just faster — the gate now sizes ~25% more shaped jerk (fixing the "sharp" lean under-stretch) and the retiming search lands durations ~6 pp closer to optimum (shorter moves).

**Accuracy monotonicity (the binding constraint) holds at every phase:** 1a preserves it exactly (parity), 1b improves it (23%→3%), Phase 2 preserves 1b's accuracy exactly (same verify mesh; the model only chooses T, the gate remains source of truth). No phase ever measures *less* jerk than today.

**Composition note:** Phase 1a alone is the fastest (~45 ms) but is NOT a valid stopping point — it keeps the 23% under-measure, violating the operator's accuracy constraint. 1b trades some of 1a's speed for the accuracy fix (still 4.7x faster than today); Phase 2 recovers most of that speed by cutting pass count. The deliverable end-state is Phase 2: fastest AND most accurate AND tightest durations.

---

## Prototype evidence digest

### Batched shaped gate
- The elegant resolution HOLDS: a vectorized shaped gate at n=1600 is BOTH faster and more accurate than today's 200-sample python loop — 39.7 ms/pass vs 185.6 ms/pass (4.7x) AND shaped-jerk error ~3% vs ~23% (7x) against a 6400-sample reference.
- Batched shaped sampler matches _ShapedPlan.state_at to machine precision: pose 7.1e-15, twist 5.7e-14, accel 4.5e-13 (all < the ~1e-12 target) across 6 moves x 4 gains x 3 durations.
- Batched gate == python validate at the SAME mesh to 1.30e-10 (shared 1e-7 J-dot FD) — so cutover changes ONLY the deliberate mesh, nothing else in the semantics.
- Today's shaped floor (200 samples) under-measures shaped leg jerk by ~21-23% vs 6400; batched n=400 already halves that (-15%), n=800 = -7%, n=1600 = -3%, n=3200 = -1%. Recommend n*=1600.
- The cutover is STRICTLY MORE CONSERVATIVE: random corpus N=400 shows 0/400 jerk violations, batched/python-200 jerk ratio median 1.257. Consequence: jerk-bound shaped moves stretch ~8% longer (T proportional to jerk^(1/3)) — this FIXES the silent under-stretch on the exact lean path the operator called 'sharp', it is not a regression.
- The closed-form/analytic-bound pre-filter is rigorous but ~105-135x too loose (the Jacobian's mixed mm + mm/rad columns blow up under sup-decoupling), so it is NOT viable as a fast ACCEPT gate — the batched dense gate is the winner, not the analytic bound.
- Measured python-200 = 185.6 ms/pass, not the task's cited 450 ms/pass; the 450 figure predates the shipped component-cross Jacobian speedup. Post-speedup shaped build_move (6 passes) is ~1.1 s today; batched n=1600 takes it to ~240 ms.
- Condition-number check and knot-step pass batch cleanly by reusing the shipped _batched_condition/_batched_leg_vectors; step-bound stays bit-identical (fixed 25 ms knots, mesh-invariant). Full per-sample condition at n=1600 costs only ~15 ms; keep it full for strict parity.
- Vel/acc peaks are also mildly mesh-dependent (1.3% at 200 vs 3200) since they are sampled maxima of a continuous curve — the dense batched gate improves these too, a free conservativeness bonus.
- Scope note: prototype and current shipped shaped path are single-segment rest-to-rest (build_move); a future multi-segment shaped plan needs a per-segment grid loop (mirroring _segment_grids) and a plan-scoped window.

### Retiming-invariant search
- Shaped build_move today: 6 validate passes/case uniformly, mean 1178.7 ms/case (lateral ~1330 ms) at ~197 ms/pass on this Jetson — the '450 ms/pass' in the prompt is pre-speedup; post the 2026-07-16 component-cross it's ~197 ms.
- The retiming structure holds: base leg peaks scale EXACTLY as 1/Tⁿ on a fixed s-grid (verified bit-constant), and the entire windowed lean superposition scales as fixed u-powers (pose∝u², twist∝u³, accel∝u⁴) — so leg vel/acc/jerk are per-sample polynomials in u=1/T with base=low-order and lean=high-order terms.
- MODEL SEARCH + 1 VERIFY = 2 gate-passes/case vs today's 6; measured wall-time ~410 ms vs ~1180 ms = 2.6–3.1× speedup. Build (~208 ms) ≈ verify (~195 ms) ≈ one validate pass each; solve ~4 ms.
- Zero verify failures across 42 cases: the model's raw T_min matches ground-truth gate-boundary min-T to ~1 ms. The pipeline is conservative BY CONSTRUCTION (final plan is one the unchanged validate@200 accepted).
- Duration optimality: model +1.99% mean (all from the 1.02 safety inflation, max +2.04%) — and BETTER than today, whose 4-bisection loop overshoots optimum by +7.94% mean, up to +28.4%. The change makes moves both faster to plan AND shorter to execute.
- The J-at-shaped-pose second-order effect (the task's CAUTION) is <1.5% wherever the search operates (well under the 2–3% expected); pure-z/rotation moves are exact (zero lean).
- KEY COMPLICATION at gain 0.6: the 5° tilt cap (shaping.py:333) DOES bind on large lateral moves — but only at T≲0.6 s, far below any feasible answer (feasible durations ≥1.4 s, all cap-free). Building the model at a cheap cap-free T_ref keeps it un-capped and conservative in the (infeasible) cap region.
- The lighter 'analytic seed' (approach 2) via aggregate 2-point power-law refit FAILS (39/42 gate-rejects, T collapses to 0.058 s): the binding sample switches with T and the cap distorts the aggregate power. Correctness requires the per-sample model — approach 2 collapses into approach 1.
- Accuracy is NOT compromised: the verify is the identical shipped validate@200, so the pre-existing 21% shaped-jerk mesh-dependence is preserved unchanged (neither loosened nor tightened). The dense-vectorized-gate route could additionally FIX that 21% for free, but is decoupled from this speed win.
- Gate the model path on a rest seed (_is_at_rest) and fall back to today's iterate-stretch loop for moving seeds; step-bound/WORKSPACE stay covered by the reference build's full geometry + the verify backstop.

---

## Related

- `logbook/2026-07-16-lean-planning-latency-and-boundary-step.md` — the lean
  latency/boundary arc; its Open Questions named both of this plan's mechanisms.
- `logbook/2026-07-17-s4-closed-working-point-persisted.md` — the default flip
  that makes this plan's cost apply to every service move.
- Prototype artifacts: session scratchpad `bsg/` + retiming harnesses (volatile
  /tmp — the durable numbers are in this plan; the prototypes are re-derivable
  from the approach sections).
