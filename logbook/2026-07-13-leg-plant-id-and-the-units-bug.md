---
title: The leg gain hunt ends — a rev/mm units bug manufactured the accuracy knee, and the plant-ID says feedforward, not feedback, is the lever
type: investigation
date: 2026-07-13
status: resolved
phase: "S4 gain-retune (Stage 1, bench) — CLOSED"
related_plan: leg-gain-tuning-methodology.md
files_changed:
  - tests/hardware/bench_leg_sysid.py
  - tests/hardware/sysid_lib.py
  - tests/motion/test_bench_sysid_bridge.py
  - tools/probes/bench_leg_plant_id.py
  - tools/probes/README.md
  - plans/active/leg-gain-tuning-methodology.md
  - tests/hardware/session_gain_retune.md
  - tests/hardware/mvp_bench_runbook.md
  - logbook/2026-07-12-bench-leg-gain-tuning-stage1.md
  - logbook/2026-04-27-friction-feedforward-bench-validation.md
commits:
  - c766aa8
subsystem:
  - motion
  - can
tags:
  - dynamics
  - performance
  - testing
  - methodology
---

# The leg gain hunt ends — a units bug manufactured the accuracy knee, and the plant says feedforward is the lever

## Summary

The operator ran the "real sustained S4-class datapoint" that Stage 1 was missing
(`--mode strokes --track-gains 40:0.2:0.32,70:0.35:0.56 --knot-hz 100 --fast-iq`), said he was
**not convinced the gain search had found the right answer**, and asked to question the
methodology rather than run more points. That instinct was correct and it saved the project from
shipping a gain change it did not need.

Three findings, all from data **already on disk** — no new hardware runs:

1. **A rev→mm units bug inflated the entire stroke-error table by ~14×.** The harness printed the
   tracking error as `err_rms * 1e3` under an `mm` label. `err_rms` is in **revolutions**, so
   `× 1e3` yields **milli-revolutions**. The correct scale is the bench leg's measured
   **71.5708 mm/rev** (factor `1000/71.5708` = **13.97×**). The "10.3 mm → 6.9 mm accuracy knee at
   pos 70" is really **0.74 mm → 0.49 mm**.
2. **The spec is met at the only instant that is scored.** The operator's acceptance criterion is
   **±1 mm at the catch/throw instant**. Measured at production gains: **error at arrival = 0.054 mm
   median, 0.192 mm worst** across 21 strokes — **0/21 exceed 1 mm**, roughly 5× inside spec.
3. **The plant, identified from the same CSVs: `J_eff` ≈ 0.72–1.18 × `J_rotor`** — refuting the
   unsourced "J_eff ~10× rotor" guess the methodology has carried since 2026-04-27 **by an order of
   magnitude**. The implied production cascade is `ω_v` ≈ 23 Hz over `ω_p` = 6.4 Hz — **ratio ≈ 3.6,
   textbook-healthy**. The ladder's `vel_gain ∝ √pos_gain` rule holds that ratio *constant*, so
   climbing it never improved damping; it only slid both loops toward the resonance. That is what
   the operator's ear heard at pos 110.

**Outcome: production gains `40 / 0.20 / 0.32` stand. Stage 2's candidate `70 / 0.35 / 0.56` and
its `MAX_LEAD = 0.057` re-pin are retracted. The gain ladder is demoted from "the method" to "a
stability-envelope probe", and the effort moves to feedforward** — where `torque_ff` is already
plumbed end-to-end to the ODrive and carrying zeros.

**Probe (cite, do not reproduce):** `tools/probes/bench_leg_plant_id.py --all`, over
`temp/probes/bench_sysid_20260712_222801/` (63 stroke CSVs) and
`temp/probes/bench_sysid_20260713_091748/` (42).

## Symptoms

- The operator, after the sustained-stroke run, reported he was **unconvinced the gains were
  right** — and separately noted that gains which look stable in the bench suite produce **loud
  vibration and sharp, aggressive movement under SpaceMouse teleop**, while doubting SpaceMouse
  deserved that much weight (it is a test bench and a toy, not a first-class control mode).
- He also questioned whether **static-setpoint PID intuition transfers at all** to a robot whose
  targets are re-issued at 40–100 Hz from ball mocap with a **fixed arrival deadline and a
  shrinking horizon**.
- Both doubts were correct, and both pointed at the same root cause.

## Diagnosis

### 1. The units bug (`bench_leg_sysid.py:2548`)

```python
f"errRMS={err_rms * 1e3:.1f}mm peak={err_peak * 1e3:.1f}mm "
```

`err = |cmd − pos|`, where both operands come from the CSV columns `cmd_rev` and `pos_rev`, and the
manifest field is (correctly) named `track_err_rms_rev`. **The error is in revolutions.** `× 1e3`
produces **milli-revolutions**, printed under an `mm` label. The correct scale is the leg's measured
`mm_per_rev` — **71.5708** on the bench leg (`single_leg_test.py:106`). The factor is
`1000 / 71.5708 = 13.97`.

Recomputed from the stored manifests (which held the honest `_rev` values all along):

| gain point | reported (2026-07-12) | actual |
|---|---|---|
| pos 40 / 0.20 / 0.32 | 10.3 mm | **0.74 mm** |
| pos 70 / 0.35 / 0.56 | 6.9 mm | **0.49 mm** |
| pos 90 / 0.45 / 0.72 | 6.8 mm | **0.48 mm** |
| pos 40 / pos 70 (2026-07-13 run) | — | **0.75 / 0.50 mm** |

The *relative* claims survive — ratios are scale-free, so "−33 % from 40→70, then +1 % to 90" is
still true. **The decision they supported does not.** A 10 mm RMS tracking error on a leg whose
entire stroke is 280 mm is a catastrophe that would have been obvious in the room; a 0.74 mm one is
already excellent. Nobody — including the analysis tracks and the adversarial verify stage — asked
whether 10 mm was physically plausible.

### 2. The spec, and the metric it is scored on

**Operator spec (given this session): ±1 mm of leg accuracy, judged at the catch/throw instant.**
That metric choice is load-bearing, so it is measured explicitly here. Per-stroke, at production
gains (`091748` run, 21 strokes):

| metric | median | worst | strokes > 1 mm |
|---|---|---|---|
| **error at arrival** (settled, \|v\| < 0.05 rev/s) | **0.054 mm** | **0.192 mm** | **0 / 21** ✅ |
| peak error mid-transit | 2.47 mm | 4.70 mm | 21 / 21 |
| battery mean errRMS (whole stroke) | — | — | 0.75 mm |

**The error is almost entirely velocity-proportional**: every peak occurs at 3.6–4.0 rev/s, and it
collapses to the encoder noise floor at rest. That is the signature of a **transport delay**, not a
soft servo — a stiffness deficit would leave a *steady-state* offset, and there is none (0.05 mm at
hold).

**Why arrival is the right metric:** the planner **forces translational velocity to zero at a
catch** — a baked-in safety invariant, because *"velocity matching is the hand's job, so the
platform is translationally still at contact"* (`planner.py:739-745`). The catch therefore lands
exactly where the error is 0.05 mm, not where it is 2.5 mm. (For the MVP the platform is also static
at *release* — `mvp-trajectory-bringup.md:1091-1094`. A future platform-assisted throw would score
mid-motion accuracy, and would re-open the transit number — see Open Questions.)

**Honest caveat:** mid-transit error *is* 2.5 mm median and does exceed 1 mm on every stroke. If the
spec were ever read as a *path* tolerance rather than an *arrival* tolerance, the robot is out of
spec today. But **gains are still not the lever** — pos 70 moves the transit peak only 2.47 → 2.16 mm
(−13 %), because the dominant term is gain-independent delay. The lever there is the knot-phase lead
and the feedforward, exactly as below.

### 3. The plant — identified from the CSVs, no new hardware

Regressing the rigid-body torque balance on the (a, v, iq) triples in each stroke CSV (`a` = centred
difference of a 5-sample boxcar-smoothed `vel_rps`; only clearly-moving samples `|v| > 0.3 rev/s`, so
`sgn(v)` is well-defined and stiction does not pollute the kinetic fit):

    2π·J_eff·a  =  Kt·iq  −  τ_c·sgn(v)  −  b·v

Five independent fits, across two runs and three gain points
(`tools/probes/bench_leg_plant_id.py`, run 2026-07-13):

| run / gain | R² | J_eff | × J_rotor | τ_c |
|---|---|---|---|---|
| 222801 / p40 | 0.868 | 2.66e-4 | **0.97** | 1.22 A |
| 222801 / p70 | 0.616 | 2.00e-4 | 0.73 | 1.00 A |
| 222801 / p90 | 0.499 | 1.99e-4 | 0.72 | 0.88 A |
| 091748 / p40 | 0.864 | 2.68e-4 | **0.97** | 1.18 A |
| 091748 / p70 | 0.718 | 3.24e-4 | 1.18 | 1.05 A |

- **`J_eff` = 0.72–1.18 × `J_rotor`** (pooled 2.51e-4 = 0.91×). The two highest-R² fits (0.868 /
  0.864, both at pos 40, where the iq trace is least churny) independently agree at **0.97×**. The
  rotor dominates; the leg mechanism and spool add almost nothing.
- **This refutes "J_eff ~10× rotor"**
  (`logbook/2026-04-27-friction-feedforward-bench-validation.md:257`, a hypothesis-table cell with
  no derivation, propagated into the tuning methodology). 10× would mean `J_eff ≈ 2.75e-3`; we
  measure 2.0–3.2e-4. **The refutation holds at every extreme of the scatter.**
- **`τ_c` = 0.88–1.22 A** brackets the 2026-04-27 friction bench's **1.094 A**, reproduced from
  completely unrelated data. That fit is trustworthy.
- Viscous `b` ≈ 0 (sign flips across fits — consistent with negligible).

Which finally gives the cascade the ladder was climbing blind (`ω_v = vel_gain/2πJ`, `ω_p = pos_gain`).
From the pooled `J_eff` = 2.51e-4:

| gains | velocity loop | position loop | ratio ω_v/ω_p |
|---|---|---|---|
| **40 / 0.20 (production)** | **22.8 Hz** | **6.4 Hz** | **3.58** |
| 70 / 0.35 | 39.9 Hz | 11.1 Hz | 3.58 |
| 90 / 0.45 | 51.3 Hz | 14.3 Hz | 3.58 |
| 110 / 0.50 | 57.0 Hz | 17.5 Hz | 3.26 |

> **⚠️ CORRECTED 2026-07-13 (same day).** The first version of this table read `ω_v` = 20.2 Hz /
> ratio 3.17. That **omitted the ODrive's `torque_constant` division** and understated `ω_v` by
> 13 %. On ODrive 0.6.x the velocity controller outputs **torque**, and the drive then computes
> `Iq = torque / torque_constant` using **its own configured value** (0.055133 — ODrive's default
> `8.27/Kv`), which is *not* the motor's true Kt (~0.0624). So the torque the motor actually
> produces per unit velocity error is `vel_gain × (Kt_true / Kt_config)` — i.e. **the shipping
> velocity loop runs ~13 % stiffer than its nominal `vel_gain` implies.** (Operator-confirmed:
> `vel_gain` is in Nm/(rev/s).)
>
> **`ω_v` is INVARIANT to the true Kt** — `J_eff` scales linearly with the assumed Kt and the
> measured regression slope scales as `1/Kt`, so it cancels; only the *configured* value enters.
> **The cascade conclusion therefore holds regardless of how the open 0.0551-vs-0.0624 Kt question
> lands.** (`tools/probes/bench_leg_plant_id.py` now carries the derivation and computes it.)


*Scatter (the honest uncertainty on every row): `J_eff` 1.99–3.24e-4 moves the production `ω_v` over
**17.7–28.8 Hz** and the ratio over **2.78–4.53**. `bench_leg_plant_id.py --all` prints this.*

Production is a **healthy cascade** — inner loop ~3× the outer, which is what a cascade wants, and
it stays acceptable (≥2.5) even at the pessimistic end of the scatter. Two things follow:

- **The ladder's `vel_gain = 0.45·√(pg/90)` rule holds `ω_v/ω_p` CONSTANT.** Climbing it therefore
  never improved the loop's damping *structure*; it slid both crossovers upward together. **The
  ladder was a resonance-proximity dial that looked like a performance dial.**
- **It is `ω_p` that walks into the resonance, not `ω_v`.** At production, `ω_p` = 6.4 Hz sits far
  below the 15–19 Hz band where the 2026-07-12 chirp measured ζ ≈ 0.5; at pos 110 it lands **on it**
  (17.5 Hz). Meanwhile `ω_v` is already *above* that band at production. So the honest stiffening
  knob, if one were ever needed, is **`vel_gain` up** (raises the inner loop and the ratio) — **not
  `pos_gain` up**, which lowers the ratio and marches the outer loop into the resonance.

### 4. The error budget — how much is even attackable by feedback

Decomposing `err(t) = cmd_u0 − encoder` as `τ_d·v + k_a·a + k_f·sgn(v) + c` on the `091748` run
(R² = 0.725):

| term | pos 40 | pos 70 | attackable by gain? |
|---|---|---|---|
| **total errRMS** (pooled over all samples) | **0.83 mm** | **0.58 mm** | |
| delay-shaped `τ_d·v` (τ_d = 6.2 ms) | 0.57 mm | 0.37 mm (4.1 ms) | **no** — gain-independent |
| inertial `k_a·a` | 0.17 mm | 0.07 mm | weakly (∝1/Kp·Kv) — **or exactly, by a torque FF** |
| friction `k_f·sgn(v)` | 0.15 mm | 0.06 mm | weakly — **or exactly, by a friction FF** |
| residual (noise floor) | 0.44 mm | 0.40 mm | no |

**Three caveats, all necessary to read this table honestly:**

1. **These are NOT a partition.** They are RMS contributions of *correlated* regressors, so they do
   not sum to the total in any norm (`√(0.57² + 0.17² + 0.15² + 0.44²) = 0.76 ≠ 0.83`). Read each as
   "this term alone contributes X mm RMS".
2. **This is the `091748` fit.** The same decomposition on `222801` fits far worse (R² 0.42 / 0.18 /
   0.17 at p40/p70/p90; pooled totals 1.22 / 1.74 / 1.95 mm) — its stroke set carries more
   low-velocity and hold content, where the `sgn(v)` and delay regressors are ill-conditioned. The
   *shape* (delay dominant, gain-independent) is the same in both; the magnitudes are not.
3. **`total_rms_mm` here pools every sample across all CSVs at a gain**, so it is *not* the same
   statistic as the harness's per-stroke `track_err_rms_rev` mean (0.75 mm). Expect the pooled figure
   to be higher.

The load-bearing conclusion survives all three: **the delay term is the largest single contributor
and no gain can touch it.** Worse, much of it is the *metric*: `cmd_u0` is the knot at the **start**
of the 10 ms segment the firmware is interpolating **through**, so it lags the executed Hermite by up
to one knot of travel (mean ≈ T/2 = 5 ms at 100 Hz knots; **12.5 ms at the production 40 Hz**). The
fitted `τ_d` of 4–6 ms sits right in that band.

The two terms a feedforward would cancel exactly — inertial and friction — contribute 0.17 and
0.15 mm. **There is essentially no servo tracking error left for a feedback gain to remove.**

## Discussion

### (a) Why the units bug survived four rounds of adversarial verification

The 2026-07-12 session was, by construction, careful: four analysis rounds, parallel Opus tracks, and
**every load-bearing claim adversarially verified against the raw CSVs and source**. That process
caught three wrong hypotheses. It did not catch this one, and the reason is worth naming.

**Every verification was internal to the number.** The tracks checked that the errRMS values were
computed from the right arrays, over the right windows, with the right samples excluded, and that the
pos-to-pos *delta* was the honest discriminator. All of that was true. Nobody performed the one check
that would have caught it in a second: **is 10 mm a physically sane tracking error for this leg?**
The stroke amplitudes were ~1 rev ≈ 70 mm; a 10 mm RMS error is 14 % of the stroke — visible across
the room, and it would have latched `MAX_DEVIATION` long before it was tolerable. **The number was
internally consistent and externally absurd, and the verification was all internal.**

The durable defence is not "verify harder" — it is **a unit-carrying enforcement point**. The
conversion now lives in `sysid_lib.format_stroke_error(err_rms_rev, err_peak_rev, mm_per_rev)`, a
named pure function in the unit-tested layer, with three regression tests that fail if the scale is
hard-coded back to a constant. The bug survived because the conversion was one inline expression
inside a 60-line method, and **nothing in 123 bridge tests asserted it**.

*(This entry's own first draft repeated the class: it quoted the **platform** leg's 70.5 mm/rev while
correcting **bench** leg data — a further 1.5 % error — and hand-assembled a cascade table whose rows
did not divide. Both were caught by the audit pass, and the probe now prints its own scatter so the
numbers cannot be hand-made again.)*

### (b) Why the operator's two doubts were the same doubt

The operator raised SpaceMouse-vs-mission and static-PID-vs-tracking as separate worries. They have
one root: **the methodology was tuning a feedback loop to solve a feedforward problem.**

In a classic regulation problem — hold a setpoint, reject a disturbance — the reference is a step,
there is no model, and `Kp` genuinely *is* the lever. That is the intuition the operator had, and
correctly suspected did not transfer.

Jugglebot is the opposite regime. **The reference is known in advance, analytically, with exact
derivatives.** `plan.state_at(τ)` returns `(pose, twist, accel)`; `ik_solver.py` already computes the
exact velocity IK (`J·ẋ`, `twist_to_leg_velocities`) and the exact acceleration IK (`J·ẍ + J̇·ẋ`,
`accel_to_leg_accels`). When you know the reference and its derivatives, **feedforward follows the
trajectory and feedback only cleans up model error.** The division of labour:

- **Feedforward sets accuracy.** Its ceiling is model error.
- **Feedback sets robustness.** Its ceiling is resonance and sensor noise.

The gain ladder inverted this: it used the stability-limited lever (bounded by a resonance) to chase
an accuracy target the accuracy-limited lever had already met. That is why every rung bought less and
less, and why the last rungs bought vibration instead.

**The project's own history says this louder than any argument.** The single largest improvement in
the 2026-04-27 investigation was `vel_ff` — tracking lag 31 → 6 ms, ~5× — and it is a *feedforward*,
not a gain. `vel_ff` is live today (the firmware Hermite's analytic derivative,
`leg_interp.cpp:334-337`), and it is precisely *why* the tracking error is already sub-millimetre. We
then spent a bench session trying to improve on that with the other lever.

**Feedforward is not free, and this project has the scar to prove it.** A feedforward is an open-loop
injection into a closed loop: a *discontinuous or mis-modelled* one can absolutely destabilise. On
2026-05-08 a friction FF with a hard boost band at v≈0 **bootstrapped a self-sustaining ~5 Hz platform
limit cycle** (hold `act_std` 21 → 862 µm, 40×), and it took a smooth gate to tame
(`logbook/2026-05-08-friction-ff-platform-limit-cycle.md`). The correct claim is therefore the
qualified one: **a smooth, correctly-signed feedforward costs little or nothing in loop stability** —
and any FF we ship must reuse the existing gate, not rebuild it.

### (c) The SpaceMouse question, answered properly

The operator's instinct ("SpaceMouse might be a red herring, I'm weighting it too highly") is right,
but the reasoning that matters is not "it isn't a first-class control mode".

**It is not off-path at all.** The SpaceMouse follower is *architecturally identical* to a
ball-tracked catch: both lay down a fresh C2 quintic every 25 ms, seeded from the commanded state,
through the same `validate_follow` gate, onto the same 40 Hz knot stream. Same code. Dismissing it as
"just a toy" would discard a genuine signal.

Two things nonetheless make it a **bad gain selector**:

- **It is a worst-case soak, not the operating point.** The chase clamp deliberately drives the legs
  to **85 % of their limits, continuously and indefinitely** (`chase.py`: `|w + α·u| ≤ 0.85·L`). A
  juggling catch is a short, bounded profile that then **freezes 300 ms before arrival**
  (`JB_TRAJ_CATCH_REACH_FREEZE_S`) and coasts in on a single smooth plan.
- **The harshness has a specific, non-gain cause.** Every 25 ms the follower starts a *new* quintic.
  Pose, velocity and acceleration are matched (C2) — but **jerk restarts at every tick**, so the
  reference's jerk is a ~40 Hz square wave. On top of that, the firmware's Mode-1 **cubic** Hermite
  makes the *executed* acceleration piecewise-linear with a **step at every knot**. So the commanded
  acceleration carries 40 Hz content **by construction**. Raising `pos_gain` (and `vel_gain` with it)
  raises the loop gain at 40 Hz, so those steps get amplified straight into the current loop. That is
  why a gain set can be quiet on a smooth planned stroke and harsh under teleop.

This also retro-explains the Round-2 observation that the 100 Hz-knot build "sounded less
staircase-y" — the entry attributed it to actuation texture and was right, without connecting it to
the gain-harshness mechanism.

**So: SpaceMouse is a pass/fail robustness screen ("must not ring, latch, or saturate"), never a gain
selector.** But its harshness is a legitimate bug report — against *reference smoothness*, and the fix
is a quintic firmware interpolator (the wire already carries the `accel[6]` field Mode-1 ignores), not
a gain.

### (d) The "fixed deadline, shrinking horizon" concern

Worth answering directly, because it was the operator's own framing and because it closes the loop
with everything above. The architecture already handles it:

- Every replan is **seeded from the commanded state** with matched (pose, vel, accel)
  (`trajectory_node.py:1336-1343` → `planner.py:468-469`), so a correction can never produce a step —
  no matter how late it arrives.
- If a correction is too large for the time remaining, `validate_follow` (~1.6 ms) **loudly rejects**
  it as `TOO_FAST` carrying the minimum feasible lead. It never silently violates the jerk limit.
- The 300 ms **reach-freeze** stops late target jitter from chasing itself into the catch.

But here is the part that matters: **a late correction is an acceleration demand** — and acceleration
is exactly the term with no feedforward. So if late corrections are to land accurately, the
**accel/torque feedforward is the lever that buys it**, and `pos_gain` is not. The operator's worry
and the methodology's answer point in opposite directions, and the worry is right.

### (e) What the S4 6 Hz ring actually was (and why it is not a gain problem either)

The one piece of evidence that production gains might be wrong is the 2026-07-10 loaded-robot
5.9–6.1 Hz limit cycle at `40/0.20/0.32`. But that entry's own diagnosis was **structural, not
tuning**: `MAX_LEAD = 0.15` gave `pos_gain × lead = 6.0 rev/s`, *above* the 4.0 rev/s `vel_limit`, and
`vel_ff` was discontinuously **zeroed at clamp engage** — a bang-bang excitation whose frequency is
naturally set by the loop's own bandwidth (hence the `≈ pos_gain/2π` signature that made it *look*
like a gain problem). Both were fixed in v3 firmware (`MAX_LEAD` → 0.10, vel_ff kept through the
clamp), and the bench under v3 **cannot reproduce the ring in any regime** — not on the ladder, not in
sustained tracking, not riding the clamp at 46 %.

So the loaded S4 replay is still worth running, but it is a **regression test of the v3 firmware fix
at production gains** — not a gain-selection experiment. Reframed accordingly in the methodology, the
session protocol, and the bench runbook.

### (f) What we accepted, and the one thing that could still reopen this

The bench leg is **unloaded**: no gravity, no inter-leg coupling, no hand reaction forces. Its
inertial + friction terms (0.17 + 0.15 mm) are a *floor*, not the loaded-robot budget. So the honest
position is not "feedforward is unnecessary" — it is:

- **Gains are settled** (the ±1 mm arrival spec is met with ~5× margin, and the cascade is healthy).
- **Feedforward is where the remaining margin comes from**, and the **loaded robot** is where it will
  be needed — which is also the only place it can be honestly measured.

The `J_eff` fit is ±~20 % (it differentiates a 100–250 Hz velocity signal). That is ample to kill a
10× hypothesis and to size a feedforward, but it is **not** a precision inertia measurement, and a
torque-FF gain derived from it must be validated by A/B, not trusted open-loop.

## Fix

One commit (`c766aa8`), all bench/documentation-scoped. **No production config value
changed** — the committed leg gains remain `40 / 0.20 / 0.32` — and **no firmware was touched**.

- **`tests/hardware/sysid_lib.py`** — new pure `format_stroke_error(err_rms_rev, err_peak_rev,
  mm_per_rev)`: the named, tested enforcement point for the rev→mm conversion, with the failure mode
  documented in its docstring.
- **`tests/hardware/bench_leg_sysid.py:2546-2549`** — the strokes print now routes through it
  (reusing the already-correct `hw_mm_per_rev()` helper, which the chirp stage had been using properly
  all along).
- **`tests/motion/test_bench_sysid_bridge.py`** — 3 regression tests: the exact 0.0102 rev → 0.73 mm
  case (asserting `10.2`/`10.3` do **not** appear), linearity in `mm_per_rev` (so a hard-coded
  constant fails), and zero/signed handling. Verified genuine: reverting the scale to `1e3` fails all
  three.
- **`tools/probes/bench_leg_plant_id.py`** (new, committed per the reusable-probe rule) — the
  plant-ID and error-budget regressions, re-runnable on any future stroke battery, printing their own
  fit scatter. Catalogued in `tools/probes/README.md`.
- **`plans/active/leg-gain-tuning-methodology.md`** — the gain hunt closed; the ±1 mm arrival spec
  recorded as the stopping rule it never had; a new **Feedforward tier**; the ladder demoted.
- **`tests/hardware/session_gain_retune.md`** + **`tests/hardware/mvp_bench_runbook.md` § S4b** —
  retraction banners, so the **operator-facing** procedure no longer prescribes the cancelled sweep.
- **`logbook/2026-07-12-...`** and **`logbook/2026-04-27-...:257`** — retraction/refutation banners at
  the source of each superseded claim.

## Verification

Full suite `pytest tests/ -q`, run **2026-07-13** as the pre-commit gate on the final tree:
**2578 passed, 1 xfailed** in 601.77 s (the single pre-existing xfail). Bridge-logic subset `pytest tests/motion/test_bench_sysid_bridge.py -q`, run
2026-07-13: **126 passed** in 1.67 s (123 pre-existing + 3 new).

Harness dry-run `python tests/hardware/bench_leg_sysid.py --dry-run --mode strokes` — validation
passes, no socket opened, no motor commanded.

Probe reproduction `python tools/probes/bench_leg_plant_id.py --all` (2026-07-13) — five independent
plant fits across the two stroke batteries; `J_eff` 0.72–1.18 × `J_rotor`, `τ_c` 0.88–1.22 A, pooled
cascade 22.8 / 6.4 Hz (ratio 3.58), as tabulated above.

**Regression tests proven genuine:** reverting `format_stroke_error` to a hard-coded `* 1e3` fails all
three new tests.

**No firmware change** — `Teensy_code_canbridge` untouched; no `pio` rebuild needed.

## Outcome

- **Production leg gains `40 / 0.20 / 0.32` stand.** They meet the operator's **±1 mm at the
  catch/throw instant** spec with ~5× margin (**0.054 mm median / 0.192 mm worst** error at arrival,
  0/21 strokes over 1 mm, unloaded).
- **The Stage-2 candidate `70 / 0.35 / 0.56` is RETRACTED**, along with its `MAX_LEAD = 0.057` re-pin
  (derived as `4.0/pos_gain` at pos 70; at pos 40 the correct value is 0.10 — **already the shipped
  value, so no firmware change is needed**).
- **The gain ladder is demoted** from "the method" to "a stability-envelope probe". Feedback gains are
  henceforth sized for **disturbance rejection and hold quiescence** — genuine regulation problems
  where the operator's PID intuition applies exactly — never for tracking accuracy.
- **The methodology gains a stopping rule it never had**: ±1 mm at arrival. Its absence is *why* the
  ladder climbed until something broke; there was no other termination condition.
- **If a future round ever needs a stiffer loop, `vel_gain` is the honest knob, not `pos_gain`** — the
  measured cascade shows `pos_gain` alone *lowers* `ω_v/ω_p` and marches the outer loop into the
  15–19 Hz resonance.
- **The effort moves to feedforward.** `torque_ff` is plumbed end-to-end and carrying zeros: the
  firmware already forwards it to the ODrive (`leg_interp.cpp:366` → `:488` → `:499`), leg acceleration
  is already computed exactly and already on the ZMQ frame as `acc_mm_s2`, and one line zeroes it
  (`controller/teensy_link/setpoint_pump.py:263`). `J_eff` — the one parameter an inertial FF needs,
  and which had never been measured — is measured here.
- **The loaded S4 replay is reframed** as a v3-firmware regression test at production gains, in the
  methodology **and** in the two operator-facing hardware docs.

## Open Questions

- **A 13 % Kt mismatch is latent under any future torque FF.** The ODrive's
  `motor.torque_constant = 0.0551` Nm/A vs our measured `Kt = 0.0624` (`hardware_config.yaml:60`,
  bench fit R² = 0.994). The ODrive divides commanded torque by *its* number, so every torque FF would
  land **+13 % hot** until one is made authoritative. **Blocking pre-req for any FF work.**
- **Two feedforward terms are off, and only one was dropped on purpose.** The friction-FF drop was
  pre-registered and measured (breakaway-onset penalty null — because the smooth gate suppresses FF at
  v≈0, which is exactly what makes its loss free *at breakaway* and says nothing about sustained
  motion, where the gate is 1). The **gravity + platform-inertia FF** looks like collateral: the
  cutover justified the drop on the claim that the `:5557` stream "carries `torque_Nm = zeros`"
  (`logbook/2026-06-25-phase11-u4-production-cutover.md:63`), but `controller/hardware_plant.py:488`
  publishes it live whenever `_has_ff_torque` is set. **Two honest qualifications**: (i) the magnitude
  cited for it (0.013–0.041 Nm/leg, non-zero in 99.6 % of samples) comes from
  `plans/archived/2026-05-08 friction-ff-motor-guard-integration.md:37` and has **not** been
  re-measured on the current stack; (ii) that is the **MPC** path, and **MPC is dormant on this
  branch** — the live MVP trajectory path (`emitter.py:89`) sends `torque_Nm=np.zeros(6)` and has
  simply never computed gravity FF at all. So the *current* loss is a gap, not a regression.
  Quantifying it is step 1 of the follow-up.
- **`docs/motion_planner/dynamics.md:3` is stale** — still claims these torques "are sent as the
  `torque_ff` feedforward field to the ODrive controllers", false since the Teensy cutover.
- **The harness still scores against `u0`, not the executed Hermite.** Reconstructing the firmware's
  Mode-1 cubic at telemetry timestamps would remove the sample-and-hold artifact and, by
  cross-correlation, measure the *true* transport delay (worth knowing: at 4 rev/s even 4 ms is
  ~1.1 mm). **Until that lands, treat any errRMS from this harness as an upper bound.**
- **Mid-transit error is 2.5 mm median and exceeds 1 mm on every stroke.** Irrelevant to the MVP
  (nothing is scored mid-flight; the platform is static at both catch and release). It becomes
  load-bearing the moment a **platform-assisted throw** is attempted, where release *velocity* and
  mid-motion path accuracy are scored. The lever there is the knot-phase lead + FF, not gains.
- **The 40 Hz acceleration staircase** (fresh quintic per tick + cubic firmware interp) is the prime
  suspect for the teleop harshness. A quintic firmware interpolator using the existing `accel[6]` wire
  field would remove it. Deferred until measurement shows it matters.
- **The loaded robot remains unmeasured.** Everything here is an unloaded bench leg. The ±1 mm spec
  must be re-confirmed on the platform, and that is where a feedforward would actually earn its keep.

## Withdrawn claims

- **2026-07-12 — "The accuracy knee is pos 70 (battery mean errRMS 10.3 → 6.9 mm, −33 %)."**
  Retracted as a *decision basis*. The values were **milli-revolutions**, not millimetres
  (`bench_leg_sysid.py:2548` scaled by `1e3` instead of `mm_per_rev` = 71.5708). The true figures are
  **0.74 → 0.49 mm**. The −33 % ratio stands (scale-free); the magnitude does not, and a 0.25 mm
  improvement is irrelevant against a ±1 mm spec that production already meets by ~5× at the instant
  it is scored.
- **2026-07-12 — "Stage-2 robot recommendation: pos 70 / vel 0.35 / vel_int 0.56, with `MAX_LEAD`
  re-pinned to 0.057."** Retracted. Its sole quantitative basis was the mis-scaled accuracy knee. No
  gain change is warranted; `MAX_LEAD` stays 0.10 (= `4.0 / pos_gain` at pos 40, the shipped value).
- **2026-04-27 — "estimated J_eff ~10× rotor"** (`:257`, propagated into the tuning methodology).
  **Refuted by measurement**: five independent torque-balance fits give `J_eff` = 0.72–1.18 ×
  `J_rotor` (two highest-R² fits both 0.97×). 10× would require `J_eff ≈ 2.75e-3`; the data says
  2.0–3.2e-4. The rotor dominates the reflected inertia. Banner added at the source.
- **2026-07-13 (this entry, first draft — caught by the audit pass) — "the factor is 14.2× and the
  corrected values are 0.72 / 0.48 / 0.48 mm."** Those used the **platform** leg's 70.5 mm/rev on
  **bench** leg data. The bench leg is 71.5708 mm/rev (`single_leg_test.py:106`), so the factor is
  **13.97×** and the values are **0.74 / 0.49 / 0.48 mm**. Same class of error as the bug being
  corrected — which is why the probe now prints its own numbers rather than having them transcribed.

## Related

- `logbook/2026-07-12-bench-leg-gain-tuning-stage1.md` — the Stage-1 session this corrects.
- `logbook/2026-07-10-s4-stutter-guard-forensics-recovery-stack.md` — the 6 Hz ring; diagnosed
  structural (lead clamp + vel_ff zeroing), fixed in v3.
- `logbook/2026-04-27-friction-feedforward-bench-validation.md` — the friction params this session
  independently corroborates, and the source of the refuted 10× J_eff guess.
- `logbook/2026-05-08-friction-ff-platform-limit-cycle.md` — why the friction-FF smooth gate is
  safety-critical (a hard boost band at v≈0 bootstrapped a 5 Hz limit cycle; hold `act_std`
  21 → 862 µm). Any future FF work must reuse the gate, not rebuild it.
- `logbook/2026-06-25-phase11-u4-production-cutover.md` — where the FF terms were dropped.
- `tools/probes/bench_leg_plant_id.py` — the reusable plant-ID / error-budget probe.
