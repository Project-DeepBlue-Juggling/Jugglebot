---
title: The Kt mismatch is a stealth gain change — reconciled on the wire, and gravity feedforward re-applied (default OFF)
type: investigation
date: 2026-07-14
status: in-progress
phase: "Leg feedforward — Kt reconciliation + gravity FF (bench validation PENDING)"
related_plan: leg-gain-tuning-methodology.md
files_changed:
  - config/hardware_config.yaml
  - config/generate_config.py
  - ros_ws/src/jugglebot/jugglebot/motion/torque_ff.py
  - ros_ws/src/jugglebot/jugglebot/motion/dynamics.py
  - ros_ws/src/jugglebot/jugglebot/motion/motor_commands.py
  - controller/hardware_plant.py
  - controller/teensy_link/setpoint_pump.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/emitter.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/hardware/kt_bench_test.py
  - tests/hardware/kt_lib.py
  - tests/hardware/single_leg_test.py
  - tests/hardware/session_torque_ff.md
  - tests/motion/test_kt_lib.py
  - tests/motion/test_leg_torque_ff.py
  - tests/sim/test_hardware_plant_failure_paths.py
  - tools/probes/bench_leg_plant_id.py
  - docs/motion_planner/dynamics.md
commits:
  - 6113187
  - be4252c
subsystem:
  - motion
  - can
tags:
  - dynamics
  - safety
  - testing
---

# The Kt mismatch is a stealth gain change — reconciled on the wire, gravity FF re-applied (default OFF)

## Summary

Follow-on to `logbook/2026-07-13-leg-plant-id-and-the-units-bug.md`, which closed the leg gain hunt
and moved the effort to feedforward. That entry left two blocking items. Both are addressed here,
and **the first turned out not to be the bookkeeping fix everyone assumed.**

1. **The 13 % Kt mismatch is a hidden velocity-loop coupling, not a stale constant.** On ODrive
   0.6.x the velocity controller outputs **torque**, and the drive computes
   `Iq = torque / torque_constant`. So the amps delivered per unit velocity error are
   `vel_gain / torque_constant` — and **"just set `torque_constant` to the measured Kt" would
   silently detune the shipping, bench-validated velocity loop by −11.7 %**, with no error and no
   telemetry. That single action must never happen. **Resolution: don't touch the drives.**
   Pre-scale the leg `torque_ff` on the wire by `Kt_config/Kt_measured = 0.8835` so the *delivered
   current* is physically correct — the only thing a torque FF actually needs — at zero risk to the
   loop.
2. **Gravity + platform-inertia feedforward is re-applied, DEFAULT OFF**, behind a config flag, with
   a per-leg clamp, a ramp-in, and a validated sign. It is inert until an operator enables it.

**And the same `torque_constant` division falsified a table this project published yesterday.** The
2026-07-13 cascade (`ω_v` = 20.2 Hz, ratio 3.17) computed `ω_v = vel_gain/(2πJ)`, treating `vel_gain`
as if it were true Nm. Corrected: **`ω_v` = 22.8 Hz, ratio 3.58.** The conclusion strengthens (the
cascade is *healthier* than published), and — usefully — **`ω_v` turns out to be INVARIANT to the
true Kt**, so it survives however the open Kt question lands.

**The Kt value itself is NOT settled and is now under re-measurement.** A bench harness that
cancels friction exactly is written and ready to run.

## Diagnosis

### 1. `vel_gain` is in Nm/(rev/s) — three independent confirmations

This is the load-bearing fact, so it was pinned three ways:

- **Operator-confirmed** (2026-07-14), directly from the ODrive docs.
- **The version-pinned 0.6.11 API reference** states it verbatim: `vel_gain : [Nm / (rev/s)]`,
  `vel_integrator_limit : [Nm]`, `pos_gain : [(rev/s) / rev]`. The amps→Nm units change landed in
  **0.5.1** (ODrive CHANGELOG, 2020-09-27), so the "maybe it's still amps like 0.5.x" escape hatch
  is closed.
- **The robot's own stability proves it.** If `vel_gain = 0.20` were in **amps**/(rev/s), the
  peak 6.09 A observed on a real stroke would require a velocity error of `6.09/0.20 = 30.5 rev/s` —
  impossible against a 4.0 rev/s `vel_limit`. It would also put `ω_v` at **1.26 Hz**, *below* the
  6.37 Hz position loop: an inverted cascade that rings unconditionally. The leg instead arrives to
  0.054 mm. **Amps-semantics is physically excluded by the machine working at all.**

### 2. Therefore `torque_constant` is a gain, not just a unit

`iq_per_velocity_error = vel_gain / torque_constant = 0.20 / 0.055133 = 3.628 A/(rev/s)`.

The drives are flashed with `torque_constant = 0.055133` — which is **ODrive's nameplate heuristic
`8.27/Kv = 8.27/150`, to 15 significant figures**. It was never calibrated (git shows it was never
once edited). The motor's bench-measured Kt is ~0.0624. So the **real** torque the motor produces
per unit velocity error is `vel_gain × (Kt_true/Kt_config)` — i.e. **the shipping loop already runs
~13 % stiffer than its nominal `vel_gain` implies**, and has done all along.

Consequences:

- **`set_tc_only` is the one action that must never happen.** Setting `torque_constant := 0.0624`
  alone drops the loop's authority from 3.628 to 3.205 A/(rev/s) — **−11.7 %** — silently.
- If it is ever changed, it must land on **three surfaces atomically**: (i) the YAML gains that
  `teensy_bridge_node` pushes at configure-time, (ii) the **flashed** ODrive JSON gains (because
  HOMING and ACTIVATE run *before* configure and therefore use the flashed values), and (iii)
  `torque_constant` itself, on all six drives. Any one landing without the others moves the loop
  ±13 %.
- **We therefore do not touch the drives.** The wire pre-scale (`× 0.8835`) achieves the only thing
  the FF needs — correct delivered current — with zero exposure to the above.

### 3. The published cascade was wrong (and why the correction is Kt-proof)

`ω_v` must include the `torque_constant` division:

    ω_v = vel_gain × (Kt_true / Kt_config) / (2π·J_eff)
        = vel_gain / (Kt_config × x0)          [x0 = d(iq)/d(accel), MEASURED]

| gains | ω_v | ω_p | ratio |
|---|---|---|---|
| **40 / 0.20 (production)** | **22.8 Hz** | 6.4 Hz | **3.58** |
| 110 / 0.50 | 57.0 Hz | 17.5 Hz | 3.26 |

Scatter across the five plant fits: `ω_v` 17.7–28.8 Hz, ratio 2.78–4.53 — healthy (≥2.8) even at
the pessimistic end.

**`ω_v` is invariant to the true Kt.** `J_eff` scales linearly with the assumed Kt while the measured
slope `x0` scales as `1/Kt`, so it cancels; only the *configured* value enters. **The cascade
conclusion holds regardless of how the Kt question lands** — which is what makes it safe to publish
while Kt is still open. The derivation now lives in `tools/probes/bench_leg_plant_id.py` so it cannot
be re-derived wrong.

### 4. Kt itself: what is solid, and what is not

**SOLID (high confidence): Kt is NOT 0.055133.** That number is ODrive's uncalibrated nameplate
heuristic with zero empirical backing, and the ODrive datasheet **cannot adjudicate it** —
`3.86 Nm / 70 A = 0.05514 = 8.27/150` exactly. *The datasheet is the formula, circularly.* An implied
true Kv of `8.27/0.0624 = 132.5` rpm/V is a 12 % nameplate error, which is utterly ordinary for a
hobby outrunner. ODrive's own docs concede the input is unreliable and that the constant is partly a
scaling convention ("error can be absorbed into the gain").

**NOT SOLID: that Kt = 0.0624 exactly.** Two independent problems in the fit that produced it:

- **A lever-arm bug, live in the code until today** (`single_leg_test.py:841`). The fit used
  `SPOOL_RADIUS_MM[axis]` = **11.221 mm** — the *standard* leg's radius — on the **bench** leg, whose
  radius is **11.391 mm** (the same file *defines* `TEST_LEG_SPOOL_RADIUS_MM` for exactly this reason,
  and every other bench test in it uses the bench geometry). The lever arm was **1.51 % short**, so
  torque-per-kg was understated, the fitted slope was 1.51 % too steep, and **Kt came out 1.51 % LOW**.
  Corrected, the same data gives **0.0633** — *further* from ODrive's 0.0551, not nearer. **Fixed this
  session.**
- **Stiction bias, unquantified.** The fit held the leg in **position** under hanging masses. Coulomb
  friction (τ_c ≈ 1.1 A — *not* small against a few-amp signal) supports part of a static load, so
  less current is needed than gravity demands ⇒ torque/iq inflated ⇒ **Kt biased HIGH**. Opposite in
  direction to the lever-arm bug; the net is unknown.

**Both plant-ID conclusions from 2026-07-13 survive at either Kt** — the Kt question does not threaten
them:

- **`τ_c` is completely Kt-invariant**: the probe returns it as the raw regression coefficient on
  `sgn(v)`, fitted against measured iq **in amps**. The 2026-04-27 corroboration (1.094 A) was itself
  fitted on steady-state mean `iq_measured` in amps, so **both sides of that comparison are in amps**
  and the agreement is Kt-free.
- **`J_eff` scales linearly with Kt**, so at Kt = 0.0551 the table shrinks ×0.8835 (pooled
  0.91 → 0.80 × J_rotor). The headline — "J_eff is ~1× J_rotor, not the ~10× a big reflected-load
  story needs" — **survives by an order of magnitude at either value**: 10 × J_rotor = 2.75e-3, the
  largest fit is 3.24e-4, and you would need Kt ≈ 0.53 Nm/A (8.5× any plausible value) to reach it.

> **A caveat worth recording, and a circularity worth naming.** Pooled `J_eff/J_rotor` = **0.91**,
> which sits *below* the hard physical floor of 1.0 (the rotor alone cannot be less than itself).
> Either errors-in-variables attenuation (regressing iq on a *noisy differentiated* accel biases the
> slope low — the likely explanation) or `motor_rotor_inertia_kgm2 = 2.75e-4` is a few percent high.
> It has never been measured — and note its "Method 2" cross-check (τ/α from the datasheet's 3.86 Nm)
> **is circular on the very 8.27/Kv constant under dispute**. **Do not use the J_eff-vs-J_rotor
> comparison to adjudicate the Kt question in either direction.**

## Fix

### The Kt reconciliation — `leave_tc_prescale_ff`

Leave `torque_constant = 0.055133` on the drives. Leave `vel_gain 0.20 / vel_int 0.32 / pos_gain 40`
exactly as validated. **Nothing about the shipping loop changes, so nothing needs re-validating.**
Instead, `config/hardware_config.yaml` now carries **both** constants —
`motor_kt_nm_per_a` (physical truth) and `motor_kt_odrive_config_nm_per_a` (what is actually flashed)
— and codegen derives

    ODRIVE_LEG_TORQUE_WIRE_SCALE = kt_odrive_config / kt_measured = 0.8835

**This is deliberately double-correction-proof.** If a future session ever *does* flash the true Kt
onto the drives, it updates the YAML mirror, the scale automatically becomes 1.0, and nothing
double-applies. A bare constant would eventually cause exactly that bug.

### The gravity feedforward — `motion/torque_ff.py`, DEFAULT OFF

`LegTorqueFeedforward` turns a planned `(pose, twist, accel)` into six **true-Nm, extension-positive**
motor torques: gravity wrench (default on when the master switch is on) and, separately gated,
Newton-Euler platform inertia + reflected rotor inertia. It reuses `motion/dynamics.py` and the
emitter's already-computed Jacobian; force decomposition is `np.linalg.solve(J.T, W)`, never `J.T @ W`.

`SetpointPump` is the **single wire enforcement point** (it also covers the MPC's `HardwarePlant`
producer): **clamp → Kt wire scale → ramp**.

- **Clamp** (`torque_ff_max_nm = 0.15` true Nm). Load-bearing, because **there is no other clamp at
  any layer** — the firmware only saturates at int16 (±3.2767 ODrive-Nm ≈ **59 A** of demand). The
  ODrive **adds** `input_torque` to the velocity loop's output *before* the torque limit, so a
  feedforward above ~0.55 Nm (= 10 A × 0.055133) saturates the current clamp, **the position loop
  loses all authority**, and the leg runs open-loop at full current until `MAX_DEVIATION` E-STOPs it
  mid-motion. 0.15 Nm is ~3.7× the largest gravity torque in the workspace and still leaves ~7.6 A of
  the 10 A budget for disturbance rejection.
- **Ramp-in** (`torque_ff_ramp_s = 2.0`, counted in *accepted frames*, restarted by `reset()`). When
  streaming begins, the leg is already held at the activate pose and the ODrive's velocity
  **integrator has already wound up to carry gravity** — and `vel_integrator_limit` is **`inf`**
  (operator-confirmed), i.e. genuinely unbounded. Applying the FF as a step would briefly command
  **~2× gravity** until that integrator unwinds. Ramping on accepted frames makes it deterministic,
  and a dropped frame *lengthens* the ramp — the safe direction.
- **Platform-inertia term is default OFF even when the master switch is on**, for a specific reason
  found while reading the firmware: `leg_interp.cpp:366` holds the last `torque_ff` **undecayed**
  through the stale-link extrapolation window (it sits *after* the velocity-decay block). An
  acceleration-proportional feedforward would keep pushing at full magnitude while the commanded
  velocity decays to zero. **Gravity is a static term, so holding it during a stale window is not
  merely harmless but correct — gravity is still there.**

`single_leg_test.py:841` lever-arm bug fixed (above).

## Verification

**The sign — verified independently by energy balance, not by reading the code.** For a virtual 1 mm
platform lift, the virtual work done by the six feedforward torques equals *mgh* **exactly**:

| pose | torques (Nm) | all positive? | FF work / mm-lift vs mgh |
|---|---|---|---|
| ACTIVE (0,0,170) | 0.013 – 0.039 | ✅ | 11.76720 mJ vs 11.76720 mJ — **ratio 1.00000** |
| z = 220 | 0.012 – 0.039 | ✅ | **ratio 1.00000** |
| extreme (0,−100,200) | 0.003 – 0.038 | ✅ | **ratio 1.00000** |
| tilt rx = 0.1 rad | 0.011 – 0.041 | ✅ | **ratio 1.00000** |

A flipped sign gives −1; a `J.T @ W` substituted for the solve gives garbage. Magnitudes land in the
historically-measured **0.013–0.041 Nm/leg** band. **Mutation-tested**: flipping the gravity sign in
the source fails **11 tests**, one of which specifically catches the transpose-product substitution.

**Flag-OFF is genuinely inert**: the shipped default emits `torque_ff = zeros`, and a producer sending
a **NaN** `torque_Nm` while OFF neither leaks nor causes a frame reject (which would matter — a NaN on
the wire makes the firmware drop the *whole* setpoint frame).

**Clamp + scale**: an absurd 5.0 Nm command clamps to 0.15 and scales to 0.132532 on the wire
(= 0.15 × 0.8835); a normal 0.04 Nm → 0.035342.

Full suite `pytest tests/ -q`, run **2026-07-14**: **2701 passed, 1 xfailed** (the single pre-existing
xfail). One further failure, `test_t3b_h4_on_post_solve_allocates_within_budget`, is the **known
load-flaky allocation test** — re-run isolated 2026-07-14: **1 passed in 7.35 s**. Not a regression
(see `project_hot_loop_alloc_test_flaky`).

**No firmware change. No production gain change. No ODrive re-flash.**

## Outcome

- **`torque_ff` is wired end-to-end and physically correct — and OFF.** Turning it on is an operator
  bench action (`tests/hardware/session_torque_ff.md`).
- **The ODrives are not touched.** The wire pre-scale makes the delivered current right without
  exposing the validated loop.
- **A Kt experiment is ready to run** (`tests/hardware/kt_bench_test.py`), which settles the number
  properly: a constant-velocity **up/down** traverse under known masses, where
  `(iq_up + iq_down)/2 = τ_g/Kt` **cancels friction exactly** (friction flips sign with direction,
  gravity does not) and `(iq_up − iq_down)/2 = τ_f/Kt` yields a **free, independent friction
  measurement** that cross-checks the known 0.88–1.22 A. Fitted with a free intercept, which absorbs
  the leg's own weight. Recommended: **1.0/1.5/2.0/2.5/3.0 kg at 0.6 rev/s** (2.4× the stiction knee).
  Resolves 0.0551 vs 0.0624 at **~7.5σ**; it **cannot** resolve 0.0624 vs 0.0637 (~1σ) and says so.
  **Requires `BENCH_SYSID_BUILD`** — the DIAGNOSTIC frame gates on `iq_setpoint`, not `iq_measured`
  (`telemetry.cpp:59`), so at the constant steady current this experiment deliberately produces, stock
  v3 yields **2–3 samples per traverse**. The harness measures the rate and **hard-refuses below
  150 Hz** rather than returning a confidently wrong number.
- **Run `--mode torque_ff_check` FIRST.** It injects a known `torque_ff` and reads the resulting iq,
  empirically settling the **sign through the firmware's `leg_sign` negation** and the int16 scaling
  — de-risking the gravity FF before it ever moves the platform.

## Discussion

### The iq sign-frame trap (a code-read would have given the wrong answer)

`can_buses.cpp:85-86` runs pos/vel through `leg_sign()` (negating for legs ⇒ extension-positive) but
`:92` stores `iq_measured` **raw**, in the ODrive frame. Taken at face value that means an extending
torque should read as **negative** iq — which would make the 2026-07-13 plant-ID regression sign-broken
and its `J_eff`/`τ_c` negative.

They are not. **The recorded data settles it and the code-read is misleading**: over the stroke battery,
`corr(iq, accel) = +0.39` and `corr(iq, sign(vel)) = +0.81`; a +36 rev/s² extension draws **+0.89 A**,
a −36 rev/s² one draws **−0.82 A**. On this rig **positive iq = extending torque** — the ODrive's own
motor/encoder direction calibration evidently inverts the raw sign back. Both fitted quantities come out
positive, and `τ_c` reproduces the independent 1.094 A, which a flipped frame could not do.

The lesson generalises: **per-axis direction calibration is a property of each drive, not of the
firmware**, so this must be re-verified empirically before porting the probe to the assembled robot's
legs. It is why the Kt harness lets **gravity self-calibrate the frame** and merely *asserts* against
the code-read prediction rather than trusting it — and it is why `--mode torque_ff_check` exists.

### Why not just fix the drives?

The tempting move — "the ODrive has the wrong number, set it right" — is the one action that silently
breaks the robot, because on 0.6.x `torque_constant` is **in the velocity-loop gain path**, not merely a
reporting unit. And "fix it and rescale the gains to compensate" is worse than it sounds: it must land
atomically on three surfaces (YAML gains, *flashed* gains, *flashed* torque_constant), because homing
and activate run before configure and therefore use the flashed values. Two of three landing is a silent
±13 % loop change.

The wire pre-scale sidesteps all of it, costs one multiply, and is self-cancelling if the drives are
ever corrected. **Prefer the change that cannot be half-applied.**

### What we deliberately did NOT do

Friction FF is **not** re-enabled here. It is a velocity-signed model that needs the smooth
low-velocity gate whose absence once bootstrapped a **5 Hz platform limit cycle** (hold `act_std`
21 → 862 µm, `logbook/2026-05-08-friction-ff-platform-limit-cycle.md`). Gravity is a *static*, smooth,
pose-continuous term with none of that character — which is precisely why it is the safe one to land
first. **A feedforward is not automatically stability-free**; this one is safe because it is smooth and
correctly signed, and both were verified rather than assumed.

## Open Questions

- **Kt is not settled.** Run `kt_bench_test.py`. Until then `torque_ff_enabled` stays FALSE. The wire
  scale is *derived* from the YAML, so correcting `motor_kt_nm_per_a` automatically corrects it.
- **The gravity FF has never touched hardware.** Everything above is bench-verified in software. The
  operator validation procedure is `tests/hardware/session_torque_ff.md`.
- **`motor_rotor_inertia_kgm2 = 2.75e-4` has never been measured**, and one of its two derivations is
  circular on the disputed 8.27/Kv constant. The pooled `J_eff/J_rotor` = 0.91 sitting below the
  physical floor of 1.0 is a hint that it (or the errors-in-variables attenuation in the fit) is a few
  percent off.
- **The platform-inertia FF term is gated off** pending a decision on the firmware's undecayed
  `torque_ff` hold through a stale-link window.
- ~~**Three adversarial FF reviewers could not run** (monthly spend limit) … A fresh adversarial
  review before the flag is ever enabled would still be worth its cost.~~ **DONE, same day — and it
  was worth its cost. See the next section.**

## Adversarial review (post-commit, same day) — 4/4 lenses FIX_FIRST, 15 findings, all software fixes landed

The deferred review ran after `6113187` landed: four independent lenses (sign-and-physics /
safety-and-transients / wiring-and-contracts / the Kt harness itself) over the committed diff.
**All four returned FIX_FIRST.** Fifteen findings — 6 HIGH, 4 MEDIUM, 5 LOW — clustering into four
real problems, every one verified against source before fixing:

1. **The recovery-step hole (HIGH ×2).** The FF ramp restarted only on link-restore and disarm→arm.
   The most common integrator-rewind event — guard latch → `/recover`, which by design keeps
   `mpc_active = 1` — fired **neither**, so after the firmware recovery slew (which zeroes `cmd_tor`)
   the feedforward returned as a **full-magnitude single-tick step** into a re-wound integrator: the
   exact transient the ramp exists to prevent, on the exact path the jolt-free-clear work engineered
   to be step-free. **Fixed:** `SetpointPump.restart_torque_ramp()` — distinct from `reset()` on
   purpose (it must NOT waive the position-step gate, which `reset()` does) — called from both
   `/recover` clear paths. Residual step at slew hand-back ≈ `ramp(0.3 s/2 s) × 0.04 Nm ≈ 0.1 A` — noise.
2. **A third producer nobody audited (HIGH ×2 + MEDIUM).** `HardwarePlant` (the `run_mpc` path)
   computed gravity **+ platform-inertia** torque unconditionally and read **none** of the config
   flags — so enabling the master switch for the trajectory path would silently have armed the
   acceleration-proportional term on the MPC path, the exact term `torque_ff_platform_inertia: false`
   exists to gate (the firmware holds `torque_ff` undecayed through a stale-link window). **Fixed:**
   `skip_gravity` / `skip_platform_inertia` threaded through `compute_full_feedforward_torques` →
   `cartesian_to_motor_commands` → `HardwarePlant.__init__`, which now ANDs the constructor flag with
   the config master + per-term flags. Pinned by two new tests (master-off ⇒ zeros even with
   `enable_torque_ff=True`; inertia-off ⇒ acceleration-independent FF).
3. **The Kt harness would have hurt the operator (HIGH ×2 + MEDIUM + LOW)** — see the hardware-run
   section below; fixed in the same pass (verdict gating, friction-band ladder, park-path honesty,
   shaped traverse, quit).
4. **Physics + pinning gaps (MEDIUM ×2 + LOW ×3).** (a) `compute_inertia_wrench` **omitted the
   Newton-Euler transport moment** `r_com × (m·a_com)` — the wrench is taken about the geometric
   centre but the momentum changes at the CoM, so a pure linear acceleration with the real CoM offset
   commanded ZERO moment where physics requires ~0.33 N·m (same order as the entire gravity moment).
   Pre-existing since March; newly consumed. **Fixed + pinned by an independent rotational
   virtual-work test.** (b) Deleting the 2π from the reflected-rotor term survived all 592 motion
   tests — **magnitude now pinned to rtol 1e-12.** (c) The only clamp in the chain had no ceiling —
   `torque_ff_max_nm` now rejects > 0.30 true-Nm (the ODrive adds `input_torque` *before* its torque
   limit; ~0.62 true-Nm would consume the entire 10 A budget and the position loop loses all
   authority). (d) A singular/ill-conditioned Jacobian used to step a live FF to exact zeros in one
   tick (the 2026-05-08 discontinuity class) — the **trajectory producer** now **holds-and-decays**
   over 0.5 s, with a physical sanity bound (`|τ| > 0.5 Nm` ⇒ degrade) because `np.linalg.solve`
   returns huge *finite* forces on an ill-conditioned J that would sail through any isfinite check.
   (The MPC producer's `compute_full_feedforward_torques` still zero-steps on `LinAlgError` — a
   known-open item, doubly gated off today by the master flag and MPC dormancy.) (e) One doc error (the session
   doc claimed the MPC path includes reflected rotor; it is skipped).

One LOW deferred with reason: the firmware stroke-clamp zeroes `cmd_tor` while engaged, so a leg
oscillating at a stroke limit sees FF square pulses — a **firmware** change, folded into the proposed
ingest-clamp firmware sitting rather than patched ad hoc.

## The first hardware contact — a messy run that validated the review, not the feature

The operator ran `--mode torque_ff_check` (0.8 kg, mass at an odd angle, operator's hand partially
supporting it at times). The harness printed **confidently wrong verdicts over garbage data** —
"SIGN: MUST NEGATE" and "SCALE: MISMATCH −63 %" from a fit with **R² = −0.099** (explains less than a
horizontal line) and slope −6.7 ± 3.8 A/Nm (< 2σ from zero). That is review finding #1 *demonstrated
live*: the default ladder (±0.10 Nm) exceeds the ±0.049–0.067 Nm static-friction band the
"position loop holds station" premise requires.

**Read correctly, the run *agrees* with everything previously established:**

- ~~The two rungs **inside** the friction band (±0.02 Nm) show a clean slope of **−20 to
  −22.5 A/Nm**… the magnitude is **~17 % high (≈5σ)**… `1/21.3 = 0.047`, so the bench drive's
  *configured* torque_constant may not be 0.055133…~~
  **⚠️ SUPERSEDED 2026-07-15.** That "clean pair" was **drift-aliased AND load-contaminated** —
  both members carried the hand-support (hold 0.57 A vs 1.43–1.62 A expected free-hanging), and a
  monotonic fixed-cadence ladder converts −0.13 A/s of drift into −21 A/Nm of fake slope. Three
  clean inverted-rig runs settle the channel the other way: **positive wire tff = extension, gain
  ≈ 18.2 A/Nm vs 18.14 expected** — the 0.047 torque_constant speculation is resolved (the drive
  is consistent with 0.055133). See `logbook/2026-07-15-kt-first-measurement-and-tff-channel.md`.
- The ±0.10 Nm rungs saturated at the band edge or were re-absorbed by the integrator after the leg
  moved — the review's predicted mechanism, observed.
- The −0.05 rung read iq ≈ 0.05 A ≈ *unloaded* — the "hand was supporting the mass" outlier.
- ~~The **negative** slope (positive wire torque → retraction on the bench drive) is **consistent
  with the 2026-04-27 `--ff-sign −1` bench finding**…~~ **⚠️ SUPERSEDED 2026-07-15.** The negative
  slope was the drift alias, not a torque response — three clean runs show positive wire tff =
  **extension** on this same drive, ≥10σ, **matching the production chain**. (The 2026-04-27
  `--ff-sign −1` finding concerned the *friction* FF path through can_node, a different sign
  question; it does not transfer to this channel.) The production gravity-FF sign stands, now
  anchored by **direct measurement** as well as the platform validation. See
  `logbook/2026-07-15-kt-first-measurement-and-tff-channel.md` §2 / Withdrawn claims.

**Harness fixes landed in response:** mode 2 now **refuses to emit SIGN/SCALE verdicts** unless
R² ≥ 0.9 and |slope|/σ ≥ 3 (printing a NO-CONCLUSION block with the specific quality failures and
probable-disturbance rungs instead); the default ladder stays inside the friction band
(±0.010/0.020/0.035 Nm); park-at-bottom now actually executes on abort paths (it silently no-oped
while printing "parking…", and clobbered the real abort diagnostic); the traverse start is
accel-shaped and the over-current abort requires 3 consecutive samples; `q` quits; the run
instructions demand a free-hanging untouched mass and carry the bench-vs-platform sign warning.

## Related

- `logbook/2026-07-13-leg-plant-id-and-the-units-bug.md` — closed the gain hunt; this is its follow-on.
- `logbook/2026-05-08-friction-ff-platform-limit-cycle.md` — why a feedforward is not free.
- `logbook/2026-04-27-friction-feedforward-bench-validation.md` — the source of the disputed 0.0624.
- `tools/probes/bench_leg_plant_id.py` — carries the corrected `ω_v` derivation.
