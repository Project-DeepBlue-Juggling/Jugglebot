---
title: Lean-0.3 A/B pauses were planning compute (not motion) and the "sharp tilt" was a boundary vel_ff step — Jacobian sped up ~2.5–7×, lean windowed to C2 boundaries
type: investigation
date: 2026-07-16
status: resolved
phase: "MVP trajectory bringup — S4 lean A/B: planning latency + boundary vel_ff step"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/shaping.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/motion/test_kinematics.py
  - tests/motion/test_trajectory_feasibility.py
  - tests/motion/test_trajectory_shaping.py
  - tests/hardware/mvp_bench_runbook.md
commits:
  - 274af6b
  - 475e3e7
subsystem:
  - motion
tags:
  - performance
  - kinematics
  - safety
---

# Lean-0.3 A/B pauses were planning compute, and the "sharp tilt" was a boundary vel_ff step

## Summary

During the S4 lean A/B the operator observed two things at `--lean-gain 0.3`:
the between-move pauses grew **much** longer than the unshaped (`0.0`) run, and
the preparatory tilt at the start of each move felt **sharp / disconnected**
rather than a smooth lean-into-the-move. A three-agent investigation (code
map, bag forensics on the 929 s session `2026-07-16_18-45-29`, offline
profiling of the live `build_move` path) **confirmed both operator
hypotheses**:

1. **The pauses are planning compute, not motion.** A shaped (lean>0) plan
   runs 6 analytic `validate()` passes vs 2 for an unshaped plan, and each pass
   was dominated by `compute_jacobian`'s `np.cross` generic-dispatch overhead.
2. **The "sharp tilt" is a boundary `vel_ff` STEP.** The lean tilt is ∝ the
   base quintic's acceleration, so its *rate* is ∝ the base *jerk*, which is
   nonzero at a rest-to-rest quintic's endpoints — so every shaped-move
   boundary emitted a leg `vel_ff` step of ~70–182 mm/s (0.00 for unshaped).

Two fixes landed this commit: the Jacobian is vectorized to a component-form
cross product (numerically identical, ~2.5–7× faster planning), and the lean
contribution is multiplied by a C2 plateau window that drives the boundary
`vel_ff` step to exactly 0.00 mm/s. Status was in-progress pending the
hardware re-run; the same-night re-run confirmed the fixes and the entry is
now **resolved** — see the Addendum.

## Symptoms

- At `--lean-gain 0.3`, matched-battery between-move pauses grew from
  ~1.7 s (unshaped) to ~4.3 s (shaped) — the operator reported the pause
  growth as the single most noticeable regression.
- Every shaped move began with a **sharp tilt transition** ("sharp change at
  the preparatory tilt"), and the overall lean motion felt **disconnected**
  from the translation it was supposed to smooth.

## Diagnosis

**Bag scope.** Forensics used the 929 s session `2026-07-16_18-45-29`; the
three short evening bags (`18-26-32`, `18-26-51`, `19-08-17`/`19-12-51`) are
`ODRIVE_FATAL` false-starts and carry no usable A/B moves.

### 1 — The pauses are planning compute (operator hypothesis CONFIRMED)

Matched A/B moves at session limits `(1500, 5000, 20000)` (leg-space mm units),
`seq 111–121` (unshaped) vs `seq 122–132` (shaped): mean between-move pause
**1.70 s → 4.33 s (+2.64 s)**. Decomposing that delta, **~94 % is planning
compute** — the platform's post-move settle grew only **+0.13 s**. Offline the
same `build_move` path (harness in scratchpad, imports the live repo) measured
**~0.73 s → ~2.6 s per move** unshaped → shaped.

Root-cause chain (three multiplying factors):

- **Shaped plans are barred from the fast vectorized feasibility gate**
  (`feasibility.py:403-407` raises `TypeError` on a `_ShapedPlan`), so they
  fall through to the dense analytic gate.
- **`_SHAPED_REFINE_ITERS = 4`** extra bisections in the shaped min-feasible
  refinement (`planner._refine_shaped_min`), on top of the shared min-feasible
  stretch loop — so 6 `validate()` passes vs 2.
- **`compute_jacobian` (`ik_solver.py:190-218`) was 68 % of `validate()` wall
  time**, called 3× per sample (once at `feasibility.py:207`, twice inside
  `compute_jacobian_dot`'s central difference at `ik_solver.py:294-295`). Its
  cost was NOT the arithmetic — it was `np.cross`'s numpy generic dispatch
  (`__array_function__` / `moveaxis`), six times per call.

### 2 — The "sharp tilt" is a boundary `vel_ff` step (jerk-step hypothesis CONFIRMED)

The `LeanShaper` superposes a tilt ∝ the base plan's lateral acceleration
(`shaping.py:222`), so `tilt_rate ∝ base jerk`. A rest-to-rest quintic has
`j(0) = j(T) = 60·d/T³ ≠ 0`, so the shaped plan's twist **STEPS** at every move
boundary. Emitted as `vel_ff = twist_to_leg_velocities(state_at(0).twist)`,
that step measured **~70–182 mm/s (1.0–2.6 rev/s)** depending on tier
(**0.00 mm/s** for unshaped). Bag-corroborated at matched limits: peak leg
`iq` rose **5.93 → 8.48 A (+43 %)** with lean 0.3, and the first-60-ms leg
velocity was **1.52×** the unshaped move's.

`validate()` is **structurally blind** to this: it samples each segment's
interior but never differences across the hold→move seam, so the gate never
sees the step. This is the trajectory-layer analogue of the **K5
twist-discontinuity class** from the reference-layer contract — a
discontinuity in a feedforward quantity that no in-segment check can catch.

The **"disconnected" feel is structural, not a bug**: the lean is a
self-cancelling pulse (a window peaking near `s ≈ 0.211·T` and returning the
platform to upright at every stop), so its onset is a step against the smooth,
step-free onset of the translation it accompanies.

### 3 — Safety envelope at the largest tier run

At `(1500, 5000, 40000)`: peak deviation **0.359 rev = 36 %** of the 1.0 rev
`MAX_DEVIATION` guard; peak leg velocity **285 mm/s = 54 %** of the clamp cap;
**zero faults across 143 moves**. Jerk remains the binding constraint (the
standing project finding), which is why the fixes below treat jerk as the
quantity the gate must never under-measure.

## Discussion

**Why a C1/C2 plateau window over the alternatives (the boundary-step fix).**
The requirement is to remove the boundary twist step *without* breaking the
self-consistency invariant this module guards — the emitted twist and accel
must remain the **exact analytic derivatives of the emitted pose**, or we
reintroduce the feedforward-mismatch bug class (the 2026-05-08 lesson). Three
alternatives were rejected:

- **Low-pass filter the emitted `vel_ff`.** Rejected: filtering the velocity
  channel alone decouples it from the pose and accel channels, which is exactly
  the pose/twist/accel inconsistency that caused violent oscillation before.
  A filter is a patch on a symptom; it does not make the plan self-consistent.
- **Use a higher-order (septic) base profile with zero boundary jerk.**
  Rejected: replacing the quintic base changes *every* move (huge blast
  radius), re-tunes the whole stretch/feasibility stack, and is far more than
  the lean path needs.
- **Fewer shaped-refine iterations** (to cut the pause instead). Rejected as a
  *latency* fix, but note it would also be *wrong*: the duration-stretch
  `_stretch_factor` assumes a `1/Tⁿ` scaling for the base quintic, but the lean
  contribution scales as `1/T³` (tilt_rate ∝ jerk), so the stretch seed is a
  poor match for shaped plans and dropping refine iterations balloons the
  planned durations (observed 1.04 s → 6.9 s on a compressed move). The
  bisections are doing real work.

The window multiplies the **entire** lean contribution — tilt, lever-arm xy
shift, and all their time derivatives — by `w(s) = S(s/EDGE)·S((1−s)/EDGE)`,
`S` the C2 quintic smoothstep `6u⁵−15u⁴+10u³`, `EDGE = 0.15`. `w` is C2-zero
at both boundaries (`w = w' = w'' = 0`), so `vel_ff` **and** its
acceleration-spike cousin no longer step; `w ≡ 1` on `[0.15, 0.85]`, which
contains the fixed rest-to-rest accel peak at `s ≈ 0.211`, so the lean is left
**untouched where it does its work**. The window is applied via the full
product rule inside `state_at`, so the windowed twist/accel stay exact
derivatives of the windowed pose. Window-then-cap ordering is preserved (the
`LEAN_TILT_CAP_DEG = 5°` safety clamp binds no more often, since `w ≤ 1` only
shrinks the tilt).

**Why dropping the gate sampling 200 → 80 needs the 1.05 jerk margin (the
speedup fix).** The analytic vel/acc peaks are closed-form-exact at any sample
count, and the step-bound pass samples at the fixed knot spacing — neither
depends on the sample count. **Only the jerk finite-difference does**, and FD
under-measurement is **anti-conservative on the binding constraint**: fewer
samples read a *smaller* jerk, which the planner would then stretch *less*,
emitting *more* true jerk. For an unshaped rest-to-rest quintic the leg jerk is
bounded, so the FD peak converges — at 80 samples it under-measures by ≤3.7 %
(measured), and `_VALIDATE_JERK_MARGIN = 1.05` more than covers it, mirroring
the existing `_FOLLOW_JERK_MARGIN = 1.05` on the fast follower gate. So the
gate stays **provably conservative on jerk** while paying ~2.5× less compute.
Over-conservatism only ever rejects a marginal plan (the planner stretches its
duration); it never accepts an over-jerk one.

**Why shaped plans keep the 200-sample floor.** A shaped plan's boundary jerk
does not converge under FD sampling (see below), so lowering *its* sampling
would silently under-gate the exact path the operator already calls "sharp".
Shaped validation is therefore floored at `_SHAPED_VALIDATE_SAMPLES = 200`
(status quo) inside `validate()`; shaped moves still get the ~2.3× win from the
component-form cross alone.

**The window is arguably the better root-cause fix for the same problem the
sample-floor defends.** The floor treats a *symptom* — the shaped plan's
near-impulsive boundary jerk doesn't converge under FD sampling — while the
window removes the *cause*: post-window the boundary jerk is C2-finite. Offline
the 80-vs-400 shaped-jerk shortfall halved (0.1722 → 0.0941) and the 200-vs-1600
shortfall dropped 27.4 % → 20.7 %. The window is **not a full cure**, though: on
short/compressed moves the window EDGE (15 % of the move) carries high `w''`
curvature that still doesn't converge by 200 samples (6–10 % `j200→j400` gap for
`x+150`). So a sample floor remains partially justified — but now for
**window-edge curvature**, not boundary impulse. Reconciling the two changes'
prose is a follow-up (Open Questions).

**The shaped path was already anti-conservative on jerk sampling before either
change** (≈22–27 % under-measured at 200 samples vs a 3200-sample reference — a
pre-existing latent property of the shaped path, untouched here). Both fixes
narrow it; neither introduces it.

## Fix

**`ik_solver.py` — vectorized `compute_jacobian`.** The per-leg Python loop and
its six `np.cross` calls are replaced with a batched component-form cross
product (`J[:,3] = ay·lz − az·ly`, etc.). No generic dispatch, arithmetic-only.
Numerically identical: bit-equal to `np.cross` on the same batched operands
(pinned by `test_compute_jacobian_component_cross_bit_identical`), and
**≤2.84e-14 (~1 ulp, from the batched matmul)** vs the previous per-leg loop.

**`feasibility.py` — leaner sampling + a jerk margin.**
- `_VALIDATE_SAMPLES = 80` is the new default for `validate()` (unshaped).
- `_SHAPED_VALIDATE_SAMPLES = 200` — inside `validate()`, a `_ShapedPlan` is
  floored to ≥200 (an explicit denser request is still honoured).
- `_VALIDATE_JERK_MARGIN = 1.05` inflates the stored FD jerk peak before the
  `LIMIT_JERK` comparison in `validate`, adopting the idiom `validate_follow`
  already used (its pre-existing `_FOLLOW_JERK_MARGIN` is untouched). Both
  gates' `FeasibilityReport.peak_leg_jerk_mmps3` now carry a ×1.05-inflated
  value (`validate`'s is the new one), and every downstream consumer sizes off
  the conservative peak — the planner's `_stretch_factor`,
  `trajectory_node._last_peak_jerk_mmps3`, **and the published
  `trajectory/diagnostics` predicted peak** (predicted-vs-realized jerk
  comparisons now include the ×1.05; realized stays raw). Shaped planned
  durations grow a few % (gentler) from the margin; unshaped stretch is
  essentially unchanged.

**`shaping.py` — the C2 lean plateau window.** `LEAN_WINDOW_EDGE_FRAC = 0.15`,
`_smoothstep` (C2 quintic + its two derivatives), `_lean_window`
(`w(s)=S(s/edge)·S((1−s)/edge)` with product-rule `w'`, `w''`), applied to the
full lean contribution (tilt + shift + derivatives) in `_ShapedPlan.state_at`,
window-then-cap.

**`trajectory_node.py` — install-continuity comment.** The stale
`~1.5 s (4-5 validate passes)` estimate is corrected to the measured reality:
an unshaped move now runs ~2 passes (~0.1 s offline-measured) and a shaped
move ~6 passes (~1.2–1.3 s — the drift window the guard protects against).
No behavior change; the drift-reject guard is unchanged.

## Verification

**Scoped tests (2026-07-16).**

- `pytest tests/motion/test_trajectory_feasibility.py
  tests/motion/test_trajectory_shaping.py tests/motion/test_kinematics.py -q`
  → **60 passed** (6 pre-existing return-vs-assert warnings, not new). New
  tripwires include:
  `test_compute_jacobian_component_cross_bit_identical`,
  `test_validate_80_never_accepts_what_dense_400_rejects`,
  `test_validate_80_unshaped_jerk_within_margin_of_dense_400`,
  `test_shaped_plan_gated_at_status_quo_sample_floor`,
  `test_state_at_T_boundary_is_C2_continuous_with_the_terminal_hold`,
  `test_lean_window_is_c2_zero_at_ends_and_unity_on_plateau`,
  `test_lean_window_derivatives_match_finite_difference`,
  `test_window_kills_boundary_vel_ff_step` (parametrized over axis × v/a/j),
  `test_window_preserves_lean_untouched_at_the_accel_peak`,
  `test_window_attenuates_lean_inside_the_edge_but_stays_smooth`,
  `test_windowed_shaped_ff_is_self_consistent_including_the_edges`.
- `pytest tests/motion/ -q` (2026-07-16) → **675 passed in 147.09 s** (whole
  motion scope, no regressions).

**Offline before/after (live `build_move`, scratchpad harnesses, 2026-07-16).**
Numbers are durable here because the harnesses live on volatile `/tmp`.

- **Planning latency, per move (`x+150`, median of 7):**

  | tier (leg mm) | gain | before (component-cross alone) | after (this commit) | passes |
  |---|---|---|---|---|
  | (200, 660, 10500) | 0.0 | 736 → 234 ms | **101 ms** | 2 |
  | (200, 660, 10500) | 0.3 | 2680 → 1183 ms | **1275 ms** | 6 |
  | (1500, 5000, 40000) | 0.0 | — | **98 ms** | 2 |
  | (1500, 5000, 40000) | 0.3 | — | **1191 ms** | 6 |

  Net per move: unshaped **~0.73 s → ~0.10 s** (~7×; component-cross +
  80-sample gate), shaped **~2.6 s → ~1.2 s** (~2.3×; component-cross only, the
  200-sample floor holds shaped fidelity at status quo).

- **Boundary `vel_ff` step (analytic emitter truth, all tiers/gains):**
  `vel_ff@t=0` is now **0.00 mm/s** for every move (`x+150`, `y+150`, `rx+10`,
  `z→220`) at gain 0.0/0.1/0.2/0.3 — the ~70–182 mm/s pre-window step is
  **eliminated**. (First interior knot `vel_ff` at 1 knot is small and smooth,
  0.2–5.3 mm/s, as expected of a C2 ramp.)

- **Jerk sampling convergence (`jerk_convergence.py`):** unshaped 0.8 s move at
  80 samples reads −3.68 % vs a 3200-sample reference (inside the 1.05 margin);
  shaped 0.8 s gain-0.3 move reads −21.9 % at 200 samples (the residual,
  window-edge, pre-existing shaped non-convergence documented above).

**Full suite (`pytest tests/ -q`, run 2026-07-16): 2826 passed, 1 xfailed in
620.43 s** — the pre-commit gate over both this change set and the same-commit
timed-target interface change (whose scoped run was `pytest tests/ros/ -q`:
905 passed).

## Outcome

Both operator hypotheses confirmed and both root causes fixed in software.
Planning latency is cut ~7× unshaped / ~2.3× shaped, and the boundary `vel_ff`
step is driven to exactly 0.00 mm/s. **Not yet verified on hardware** — see the
pre-registered re-run below. No config/YAML touched; no codegen needed.

**Pre-registered hardware re-run expectation** (operator re-runs the lean A/B
after `colcon build --packages-select jugglebot` + relaunch):

- Between-move pauses should drop from ~4.3 s toward ~2 s, now **dominated by
  the platform settle** (~1.2 s shaped planning + settle), not compute.
- The boundary `iq` spikes at each shaped-move onset should be **gone** (the
  5.93→8.48 A rise was the vel_ff step; it is now 0.00).
- **Open A/B question:** whether lean *helps* realized smoothness at matched
  limits is still undecided — pre-fix, at matched limits, lean 0.3 made peak
  `iq` **worse** (the step). Keep lean only if measured leg jerk drops **and**
  the motion looks/sounds calmer; else log the null and leave `lean_gain: 0.0`.

## Addendum — same night: hardware re-run confirms the fixes and answers the A/B question; entry resolved

The operator rebuilt (21:56 build, interfaces included) and ran two more
sessions: a limits A/B (bag `2026-07-16_21-58-59`, incl. a same-session
gain-0-vs-0.3 A/B at `(2000,5000,30000)`) and a lean-gain sweep (bag
`2026-07-16_22-06-30`, gains 0.60 and 1.00 at the same limits). Verdicts
against the pre-registration:

- **Pauses**: 1.67–1.74 s per move (was ~4.3 s). One honest correction to the
  pre-registered wording: the residual pause is **planning-dominated after
  all** (~1.47 s shaped-plan block + ~0.25 s settle), not settle-dominated —
  further cuts need the vectorized-shaped-gate / stretch-seed follow-ups.
- **Onset iq spikes**: GONE and reversed — leaned onset iq (4.1–4.3 A max) now
  sits BELOW gain-0's (6.9 A); pre-fix it was 8.48 A, +43 % over that
  session's gain-0 (5.93 A).
- **Does lean help? YES, decisively.** At matched `(2000,5000,30000)` limits:
  gain 0 peaked **0.94 rev deviation (94 % of the 1.0 rev guard!)** and
  7.94 A; lean 0.3 → 0.63 rev / 5.42 A; lean 0.6 and 1.0 → **0.25 rev /
  ~4.4 A**, lead-clamp engagement 69 % → 29 %. *(Correction 2026-07-17: the
  per-arm deviation figures here are not all the same move — the gain-0
  0.94 is the ±150 x-traverse; the same traverse re-extracts to **0.45 rev
  at lean 0.6** (0.46 at 1.0) from bag `22-06-30`. Like-for-like the lean
  benefit on the worst move is 0.94 → 0.45, not 0.94 → 0.25 — see
  `logbook/2026-07-17-wobble-latch-unshaped-traverse.md`.)* The pre-registered metric —
  **measured (realized peak) leg jerk** — also drops: 19,270 mm/s³ at gain 0
  → **13,519 at 0.3 (−30 %)** and **14,863 at 0.6 (−23 %)**, but is nearly
  back to baseline at 1.0 (18,698, −3 % — the larger 1.85° tilt injects its
  own leg jerk). So the keep-criterion (jerk drops AND calmer) is MET at
  0.3–0.6 and only marginal at 1.0. Honest mechanism: lean is not
  free smoothness — the shaped gate stretches leaned moves ~1.5× (realized
  peak vel 246 → ~100 mm/s), and slower-plus-tilted is what reads as calm.
  The operator independently reports liking the aesthetics up to gain 1.0.
- **The 5° tilt cap NEVER bound**, even at gain 1.0 (peak added tilt 1.85°;
  this battery's lateral accel ~317 mm/s² vs the 856 mm/s² gain-1.0 binding
  threshold) — the known cap-derivative caveat stayed dormant. Re-check if
  future choreography commands lateral accel > ~856/gain mm/s².
- **Recommended default: `lean_gain = 0.6`** (all of the benefit, lowest
  deviation, 5× cap headroom); 1.0 endorsed for aesthetics on gentle
  batteries. Not yet persisted to YAML (`lean_gain: 0.0` still ships) —
  operator decision pending alongside the session-limit persistence.

Status flips to **resolved**. Analysis details: the limits-A/B preference and
per-gain tables live in the session scratchpad artifacts and the 2026-07-16
wrap-up conversation; the durable numbers are above.

## Open Questions

- **Shaped-floor test pinning.** The prose reconciliation between the window
  and the sample-floor landed in this same commit (`feasibility.py`'s two-era
  rationale and the floor test's docstring both attribute the residual
  non-convergence to window-edge `w″` curvature, not the pre-window boundary
  impulse). The surviving caveat: the floor test's `j_400 > j_200·1.05`
  assertion is pinned on a 1.0 s `x+150` move and can weaken on shorter moves
  — if it ever flakes, relax it to `>=` and revisit whether the 200 floor is
  still earning its cost.
- **The vectorized shaped gate is the real latency endgame** (~100× if a
  `_ShapedPlan` could take the fast feasibility path instead of falling to the
  dense analytic gate at `feasibility.py:403-407`). Out of scope here.
- **An analytic shaped stretch seed** (the current `_stretch_factor` assumes
  `1/Tⁿ`; the lean scales `1/T³`) would cut shaped refine iterations honestly,
  without under-gating.
- *(both landed 2026-07-17 — see
  `logbook/2026-07-17-shaped-planning-efficiency-implemented.md`: a
  numpy-vectorised shaped gate (Phase 1a) and a `1/T`-polynomial retiming-model
  duration search that replaces the stretch/refine loop (Phase 2); shaped
  `build_move` ~1.1 s → ~91 ms.)*
- **The true fluidity fix is carrying tilt across moves** (move chaining /
  supersede), so the platform does not return to upright at every stop — the
  self-cancelling-pulse structure is *why* lean still feels "disconnected" even
  with the step removed. Needs the supersede/chaining machinery that does not
  yet exist.

## Related

- Rosbags: `~/Desktop/rosbags/2026-07-16_18-45-29` (the 929 s A/B session);
  `18-26-32` / `18-26-51` / `19-08-17` / `19-12-51` are `ODRIVE_FATAL`
  false-starts (no usable A/B moves). Resolution re-run: `21-58-59` (limits
  A/B + the gain-0/0.3 baselines at (2000,5000,30000)) and `22-06-30`
  (gain 0.60/1.00 sweep).
- `logbook/2026-07-16-max-deviation-guard-tracking-lag.md` — the 1.0 rev guard
  and vel_limit headroom this envelope runs inside (the safety framing in
  Diagnosis §3).
- `plans/parked/accel-ff-inertia.md` — the acceleration-feedforward chapter
  that will change realized leg current at these limits (relevant to the open
  "does lean help?" A/B question).
- `tests/hardware/mvp_bench_runbook.md` — S4 lean A/B section, updated with the
  dated planning-latency + boundary-step note.
- Offline harnesses (volatile `/tmp` scratchpad — durable numbers are in this
  entry): `harness.py` + `quickwins.py` (build_move timing, cross fix, sample
  sweep), `boundary_step.py` (analytic vel_ff boundary step), `measure_after.py`
  (post-fix x+150 timing), `jerk_convergence.py` (shaped/unshaped jerk vs
  sample count — a candidate for `tools/probes/` if shaped non-convergence gets
  a follow-up).
