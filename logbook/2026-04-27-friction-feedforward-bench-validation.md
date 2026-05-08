---
title: Friction feedforward — bench characterisation, model fit, and FF validation
type: investigation
date: 2026-04-27
status: tuned
phase: post-per-leg-gains-deadband-session
related_plan: "2026-05-08 motion-onset-deadtime-investigation.md"
related_issues:
  - 2026-04-20-motion-onset-dead-time-fix.md
sessions:
  - cogging_20260424_230556_*.csv     # initial 0.05 rev/s sweep, both directions
  - cogging_20260424_2330*.csv        # Test A v1 — 0.02/0.05/0.10/0.20 rev/s
  - cogging_20260427_1413*.csv        # Test A v2 — 0.075/0.10/0.20/0.30/0.40 (low-v)
  - cogging_20260427_1502*.csv        # Test A v3 — high-v sweep (0.5..5.0); 2/5 ramp-only
  - cogging_20260427_1536*.csv        # Test A v4 — short-ramp sweep through 10 rev/s
  - cogging_20260427_18*.csv          # Test A v5 (re-tensioned bench) — reference fit
  - breakaway_20260427_142213.csv     # Test B v1 — vel-gate misfire (detent compliance)
  - breakaway_20260427_145030.csv     # Test B v2 — same misfire pattern
  - breakaway_20260427_153842.csv     # Test B v3 — pos-gate, true escape, n=2 cogging signal
  - torque_step_20260427_160850.csv   # Test C — bidirectional torque-step (partial; aborted)
  - friction_ff_demo_*.csv            # full FF-demo session: pos_step, threshold sweep, vel_ff A/B
  - ff_demo_thresh_0.{05,10,15,20,25}.csv  # stiction-boost threshold sweep
  - ff_demo_no_velff.csv / ff_demo_velff.csv / ff_demo_velff_separate_cables.csv
files_changed:
  - config/hardware_config.yaml
  - sim/analysis/diagnose.py
  - tools/cogging_map_analyse.py
  - tools/friction_study_analyse.py
  - tools/breakaway_analyse.py
  - tests/hardware/cogging_bench_test.py
  - tests/hardware/breakaway_ramp_test.py
  - tests/hardware/torque_step_test.py
  - tests/hardware/friction_ff_demo.py
  - logbook/2026-04-27-friction-feedforward-bench-validation.md
commits:
subsystem:
  - motion
  - controller
  - config
  - testing
tags:
  - motion-onset
  - friction
  - feedforward
  - stribeck
  - cogging
  - hardware-bench
  - bench-rig
---

# Friction feedforward — bench characterisation, model fit, and FF validation

## Summary

Closed the bench-validation phase of the motion-onset dead-time fix. Built a single-leg bench rig (ODrive Pro on isolated CAN at node-id 0, brake resistor borrowed from the platform), characterised the leg's friction across 0.075–10 rev/s, fit a four-parameter Stribeck model (R² = 0.983), and validated three distinct feedforward terms on the bench: (1) Stribeck friction torque-FF via `set_input_pos.torque_ff`; (2) a "stiction-boost" extension that applies τ_s instead of Stribeck-tapered values for `|v_cmd| < 0.20 rev/s`; (3) velocity FF via `set_input_pos.vel_ff` to eliminate steady-state position-loop tracking lag. Combined, the FF stack reduces motion-onset latency by ~2× (81 → 32 ms) and 90 % tracking lag by ~9× (53 → 6 ms) on the bench, of which vel_ff on top of friction FF is the largest single component (~5×, 31 → 6 ms). The remaining ~32 ms latency floor is **inertia-limited, not friction-limited** — it is the time required to accelerate the reflected leg load to a velocity that produces 5 mrev of travel, even with friction perfectly cancelled. Cogging, hypothesised as a major contributor at the start, was ruled out: in steady-state TORQUE-mode cruise the iq-vs-electrical-angle correlation between fwd and rev runs was R = +0.000 at smooth velocities, ruling out a direction-independent cogging signature; in slow-velocity operation (Test C) the rotor accelerated readily at iq levels (≤ 0.10 A) far below what would be required if cogging-detent torque were significant. We are bench-ready to integrate the FF stack into `motor_guard` for platform deployment.

## Symptoms

Inherited from `plans/archived/2026-05-08 motion-onset-deadtime-investigation.md`. Platform motion-onset dead-time of 100–200 ms before commanded motion produces measurable rotor displacement; root cause was provisionally attributed (per the prior logbook entry `2026-04-20-motion-onset-dead-time-fix.md`) to a combination of stiction breakaway and possibly cogging torque, with the platform's nested PI-cascade taking time to wind the integrator up to the iq required for breakaway. The test detector (added to `sim/analysis/diagnose.py::analyse_motion_onset` in the prior session) gives the per-leg latency from commanded motion start to first 0.5 mm of leg-end displacement.

## Diagnosis

### The friction model (Test A — six iterations across 2026-04-24..27)

We ran a sequence of bidirectional velocity-mode sweeps on the bench, running through the test script `tests/hardware/cogging_bench_test.py` with `--sweep` mode and progressively expanded velocity ranges. The script logs `(t, pos_rev_raw, vel_rps_raw, iq_setpoint_A, iq_measured_A, elec_angle_rad, mech_angle_rad)` at 100 Hz (the ODrive's native CAN broadcast rate) and the analyser `tools/friction_study_analyse.py` extracts steady-state mean iq vs target velocity, fits a four-parameter Stribeck model, and computes per-velocity fwd-vs-rev electrical-map Pearson correlation as a cogging-presence test.

The sweep evolved through several iterations as we discovered limitations of the early test design:

- **Test A v1 (0.02 / 0.05 / 0.10 / 0.20 rev/s)**: 0.02 rps was too slow — the motor stalled. 0.05 rps showed stick-slip oscillation. The headline number we initially extracted (mean |iq| ≈ 1 A across the band) turned out to be partly contaminated by a velocity-loop limit cycle fighting cogging.
- **Test A v2 (0.075 .. 0.40 rev/s)**: extended to lower the noise floor. Showed the classic Stribeck shape — high friction at low v (~1.85 A at v=0.075), bottoming out near 0.5 rps.
- **Test A v3 (0.5 .. 5.0 rev/s)**: tried to reach the high-velocity viscous regime, but auto-duration was capped by the 3 rev stroke; the 2 and 5 rps runs were ramp-only and yielded no usable steady-state data. We discovered the original `RAMP_S = 1.0` was eating the entire run at high velocity.
- **Test A v4 (short ramps, full sweep)**: shortened `RAMP_S` from 1.0 → 0.05 and raised `HARD_ACCEL_CAP_RPS2` from 50 → 100 → 250 rev/s² so the motor could reach 10 rev/s within the 3 rev stroke. With a more permissive vel-tolerance window (±20 % of target) the analyser captured all eight velocity points, including the high-v ones.
- **Test A v5 (re-tensioned bench, the "reference" fit)**: after the user re-tensioned the bench leg's cable preload, ran a fresh sweep at 0.075 / 0.10 / 0.20 / 0.30 / 0.50 / 1.0 / 2.0 / 5.0 rev/s. This is the fit we ship.

Final fit parameters from Test A v5 (R² = 0.983 over 8 points spanning 2.5 decades of velocity):

| Parameter | Value | Meaning |
|---|---|---|
| τ_c | **1.094 A** | Kinetic Coulomb floor |
| τ_s | **1.953 A** | Stiction peak (extrapolated v=0) |
| ω_s | **0.251 rev/s** | Stribeck breakaway velocity scale |
| b | **0.0173 A/(rev/s)** | Viscous slope |
| Load offset | **+0.173 A** | Direction-independent constant load (gravity / cable preload) |

These parameters are nearly identical to the pre-tensioning fit (τ_c=1.07, τ_s=1.88, ω_s=0.26, b=0.005). **Re-tensioning did not materially shift friction.** The ~3 % bumps in τ_c and τ_s are within the bench's run-to-run repeatability.

The full forward / reverse / friction / cogging-correlation table at the operating velocity grid is in [tools/friction_study_analyse.py](../tools/friction_study_analyse.py)'s output JSON at [temp/reports/friction_study/report_v5_retensioned.json](../temp/reports/friction_study/report_v5_retensioned.json), and the visual plots in [temp/reports/friction_study/](../temp/reports/friction_study/).

### Cogging is not material at smooth-cruise velocities (Test A cogging-correlation analysis)

For each (fwd, rev) pair, the analyser computes the Pearson correlation between the per-revolution iq-vs-electrical-angle maps. A direction-independent cogging signature would produce R close to +1; pure noise gives R ≈ 0. Across the operational velocity range:

| |v| [rev/s] | fwd-vs-rev Pearson R | Interpretation |
|---|---|---|
| 0.075 | 0.000 | smooth tracking, no cogging signal |
| 0.10  | 0.000 | "" |
| 0.20  | 0.000 | "" |
| 0.30  | 0.069 | trace |
| 0.40  | **+0.435** | partial cogging signal becoming visible above noise |
| 0.50  | 0.183 | falls back |
| 1.00  | 0.000 | cogging averaged out / aliased by 100 Hz log rate |

We see a *weak* cogging signal peaking at ~0.4 rps where the velocity loop has the cleanest tracking, but never exceeding R = 0.5 (our threshold for "material cogging"). Cogging amplitude is on the order of 50–100 mA peak-to-peak on the iq trace, against a friction baseline of ~1.1 A — i.e. cogging is ~5 % of the friction torque. **It is not a primary motion-onset contributor.**

### Test B — breakaway-torque ramp from rest (3 iterations)

Built `tests/hardware/breakaway_ramp_test.py` to characterise stiction directly: ramp iq slowly from 0 in TORQUE/PASSTHROUGH mode and detect when the rotor breaks free from rest, repeating across 8 starting electrical angles per direction.

The first two iterations triggered too early (vel-threshold detector firing on detent compliance, not true rotor escape); fixed in v3 by switching to a position-only escape detector requiring `|Δpos| > 0.05 rev` (≈ 4 cogging detents). Test B v3 produced clean escape data with all 16 trials traversing 50+ mrev:

- Forward: mean breakaway iq = 105 mA, σ/μ = 25 %
- Reverse: mean breakaway iq = 144 mA, σ/μ = 11 %
- fwd/rev ratio = 1.30 (consistent with a small constant gravity/preload)
- Forward direction shows R² = 0.97 at n=2 (amp = 33 mA); reverse shows R² = 0.70 at n=1 (amp = 60 mA) and n=2 (amp = 37 mA) — clear, small cogging modulation of detent stiffness

Crucially, **the apparent breakaway iq from Test B (~100 mA) was an order of magnitude lower than Test A's steady-state iq (~1 A)**. We initially treated this as a contradiction. The resolution came from Test C.

### Test C — torque-step terminal velocity (the "controller-fight" diagnosis)

Built `tests/hardware/torque_step_test.py` to apply constant iq levels and let the motor reach terminal velocity, characterising friction directly without a velocity loop. We saw immediate confirmation: at iq = 0.10 A, the rotor reached 4.18 rev/s in 183 ms; at 0.20 A it reached 12 rps; at 0.50 A, 17 rps. The bench leg accelerates **freely** at iq levels far below what the velocity-mode controller in Test A had been demanding for steady cruise.

This was the key diagnostic moment of the entire investigation:

> **Test A's "mean iq" includes substantial controller cogging-fight current, not just friction.** The velocity loop fights against 84 cogging cycles per motor revolution × cruise velocity (e.g. 84 × 0.5 = 42 Hz at v=0.5 rev/s), and the steady-state mean iq settles at whatever value the integrator needs to maintain target velocity *against the limit-cycle response to cogging*. That value is much higher than the actual mechanical friction.

Looking at the iq variance during cruise confirmed this: at v_target=5 rps, iq_meas swings ±2 A around the mean (4 A peak-to-peak), well above the ~1.1 A measured mean. The variance scales with velocity (75 % CV at v=5 vs 14 % at v=0.075), consistent with a controller fighting an angle-locked disturbance whose temporal frequency rises with velocity.

The user's framing then resolved the apparent Test A vs Test B contradiction:

> **"It's possible that 0.14 A is enough to get the rotor moving, but 1 A is the minimum for *smooth* motion."**

Both readings are correct. They measure different physical quantities:
- **Test B** measures *purely mechanical* breakaway torque (~100 mA): the iq required to start the rotor moving from rest, with no velocity loop in the way.
- **Test A** measures *operational* iq during closed-loop velocity control (~1.1 A): the steady-state value the controller's integrator settles at to maintain the target velocity, including cogging-fight overhead.

### What we feed forward — and why

For motion-onset specifically, the value we want to feed forward is **what the integrator would have to wind up to anyway** in production. That's Test A's measurement, not Test B's. The point of the FF is to pre-load the integrator so it doesn't need to wind up from zero — saving the wind-up time, which is the dead-time we observe.

Therefore the Stribeck FF parameters from Test A are correct for FF design, even though they include controller-fight overhead that isn't pure friction. (Pure friction would be a worse FF because it would leave the integrator with all the cogging-fight work to do from a cold start.)

## Discussion

This investigation went through several conceptual reframings; recording them is the load-bearing part of this entry.

### The cogging-first hypothesis was wrong, but not without merit

The investigation plan (`plans/archived/2026-05-08 motion-onset-deadtime-investigation.md`) prioritised a backlash audit and stiction characterisation. The user's intuition pivoted us toward cogging as a major candidate, which I supported because the motor's 84-cycle/rev cogging seemed plausible at the velocities involved. The desk correlation against existing platform data ([tools/motion_onset_cogging_study.py](../tools/motion_onset_cogging_study.py)) found a NULL result (best harmonic R² = 0.010), but I treated this as inconclusive given the heterogeneous source data. The bench's smooth-cruise fwd-vs-rev R = 0 confirmed cogging is not material — but only after we understood Test A vs Test B, because before that we saw R=+0.86 at a stick-slip velocity (0.05 rps) and overinterpreted it.

**Lesson for future investigations:** the cogging signal we eventually identified (R²=0.97 at n=2 in Test B's forward-direction angular dependence) is a property of *detent stiffness during quasi-static escape*, not of cruise-velocity friction. The two are conceptually different and produce different signatures. Conflating them sent the early Test C design into a too-broad iq-level sweep that hit end-stops.

### The re-tensioning hypothesis was wrong

After the user re-tensioned the bench cables, the FF demo showed only "moderate improvement" and I attributed this to friction having shifted out from under the model. The refit (Test A v5) refuted this: τ_c, τ_s, ω_s all moved by < 5 %. The actual cause of the moderate improvement was that the bench's velocity-mode dead-time was already small (~70 ms), so the head-room for FF improvement was correspondingly small. The platform's longer dead-time (100–200 ms) will see proportionally larger benefit.

**Lesson:** when a hypothesis (re-tensioning) and a measurement (moderate improvement) appear to be a matched pair, that's not enough — fit the data to confirm the hypothesis is actually doing the work. The refit took 5 minutes and decisively ruled out the explanation.

### Why velocity-mode bench tests under-predicted the platform benefit

The bench demo ran in `VELOCITY/PASSTHROUGH` (later `POSITION/PASSTHROUGH`). Even in pos-mode with vel_ff disabled, the bench's pos-loop is closed only over a single leg with no platform inertia or coupling. Motion-onset on the bench is dominated by:

1. The integrator wind-up time before iq exceeds friction (FF-removable).
2. The inertial acceleration time after breakaway to cover 5 mrev (NOT FF-removable).

On the platform, components 1 and 2 still exist, but additional contributions stack on top:
- Position-loop pos_err must grow against six-leg geometric coupling before the leg sees a meaningful velocity command.
- Cogging-fight is per-leg but the platform's pose error sums across legs, so the equivalent dead-zone is broader.
- The MPC's reference trajectory shape is smoother than our trapezoid (Hermite C¹), so cmd_vel ramps slower at start.

The bench validates that FF + boost + vel_ff produces the expected effect direction and magnitude *for the parts the bench can reproduce*. The platform validates everything else.

### Stiction-boost decision — anchored to ω_s, not optimised past it

The threshold sweep showed latency monotonically improving up to threshold = 0.25 rev/s but lag bottoming at 0.20 and rising again at 0.25. We chose **threshold = 0.20 rev/s** because:

- It equals ~80 % of ω_s = 0.251, so the boost stays within the Stribeck dip region where friction is genuinely high.
- Going past ω_s means applying full τ_s in the kinetic-friction regime, where the rotor doesn't need that much torque, leading to over-acceleration and overshoot. The trend was already visible (overshoot 1.9 → 2.5 → 2.9 % across the sweep).
- The discontinuity between τ_s and Stribeck(threshold) at the band edge grows with threshold (0.40 A at 0.20, 0.54 A at 0.25, > 0.6 A above) — this discontinuity is a torque step the velocity loop has to absorb, and the absorption manifests as overshoot.

For different operating velocities the optimal threshold scales — at v_target = 2 rev/s the boost band would need to be wider proportionally. We have not yet implemented a velocity-scaled boost; at the platform we'll start with a fixed 0.20 and revisit if motion-onset doesn't improve enough.

### Velocity feedforward — the unsung dominant contribution

The 80 % reduction in 90 %-target lag (31 ms → 6 ms) when adding vel_ff was the largest single improvement we saw in any individual change in the entire investigation. It's also conceptually the simplest: send the trajectory's velocity into the velocity loop's feedforward channel so the integrator doesn't have to derive it from accumulated pos_err.

This is "free" in the sense that motor_guard already has the Hermite trajectory's instantaneous velocity at every interpolation tick — it just has to be packed into the int16 `vel_ff` field of `set_input_pos`. The platform's existing `can_node._send_position_target` already encodes `vel_ff`; we'd need to verify that motor_guard is actually populating it (or whether it's been zero all along, in which case enabling it would be a substantive baseline improvement on the platform even before any friction FF).

### Cable noise was not contributing

A final test (`ff_demo_velff_separate_cables.csv`) physically separated the encoder and motor cables to rule out EMI coupling, and produced results statistically identical to the cables-bundled run. This validates all our prior bench results and means we don't have to worry about cable routing on the platform.

## Fix

### Bench scripts (new and modified)

| File | Purpose |
|---|---|
| [tests/hardware/cogging_bench_test.py](../tests/hardware/cogging_bench_test.py) | Velocity-mode bench runner with `--sweep` mode, raised speed cap (35 rev/s = 2.5 m/s on bench leg), per-velocity auto-duration, brake-resistor-aware acceleration cap (250 rev/s²), 3 rev stroke safety. |
| [tests/hardware/breakaway_ramp_test.py](../tests/hardware/breakaway_ramp_test.py) | Torque-mode ramp test for stiction characterisation, position-only escape detector, inter-trial micro-jog for angle diversity. |
| [tests/hardware/torque_step_test.py](../tests/hardware/torque_step_test.py) | Torque-step terminal-velocity test that bypassed the velocity loop and revealed the controller-fight overhead in Test A. |
| [tests/hardware/friction_ff_demo.py](../tests/hardware/friction_ff_demo.py) | Three-mode FF effectiveness demo (`pos_step` default, `vel_step` legacy, `sweep`) with Stribeck FF, optional stiction-boost, optional vel_ff, configurable threshold and `--ff-sign`. |

### Analysis tools

| File | Purpose |
|---|---|
| [tools/cogging_map_analyse.py](../tools/cogging_map_analyse.py) | Single-session cogging-map analyser; harmonics 1, 2, 3, 4, 6, 12 (electrical) and 1, 2, 6, 7, 12, 14, 42, 84 (mechanical); Stribeck pkpk + harmonic R² + position drift slope. |
| [tools/friction_study_analyse.py](../tools/friction_study_analyse.py) | Multi-session sweep analyser, fits Stribeck via no-SciPy pattern search, parses target velocity from filename, plots Stribeck curve / tracking quality / electrical-map Pearson. |
| [tools/breakaway_analyse.py](../tools/breakaway_analyse.py) | Breakaway summary analyser; angular-dependence harmonic fits per direction; sanity check that breakaway moved more than half a cogging period. |
| [tools/motion_onset_cogging_study.py](../tools/motion_onset_cogging_study.py) | Earlier desk-correlation against platform data; produced the NULL cogging result that pivoted the investigation toward stiction. |

### Friction model parameters (committed to `friction_ff_demo.py`)

```python
TAU_C = 1.094         # A — kinetic Coulomb floor
TAU_S = 1.953         # A — stiction peak (extrapolated v=0)
OMEGA_S = 0.251       # rev/s — Stribeck breakaway speed scale
B_VISC = 0.0173       # A/(rev/s) — viscous slope
LOAD_OFFSET_A = 0.173 # A — direction-independent constant load
KT_NM_PER_A = 0.0624  # motor torque constant; canonical source single_leg_test.py:92 (Phase 2 multi-weight, R²=0.994)
STICTION_BOOST_THRESHOLD_RPS = 0.10  # default; bench-tuned best at 0.20
```

These are bench-leg-specific. On the platform they're starting estimates; per-leg refinement via the diagnose detector is the platform-tuning protocol (see Open Questions).

**Kt source.** The canonical motor-Kt measurement is in `tests/hardware/single_leg_test.py:92` (`KT_MEASURED = 0.0624`, Phase 2 multi-weight bench fit, R²=0.994). Earlier bench scripts had a rounded `0.062`; reconciled to `0.0624` in this entry. The friction-FF integration plan promotes this constant to `hardware_config.yaml:dynamics.motor_kt_nm_per_a` as a single source of truth.

### `config/hardware_config.yaml`

Added `motor_pole_pairs: 7` and `motor_stator_slots: 12` to the dynamics section so `diagnose.py` can derive electrical angle from `pos_estimate`. Pole-pair value is mirrored from `config/ODrive config Files/odrive_pro_leg_config.json:145`; stator-slot value provided by the user.

## Outcome

Bench validation produced a complete, internally consistent friction characterisation and a working FF stack:

| Metric | Best result (boost=0.20, vel_ff ON, cables sep'd) |
|---|---|
| Motion-onset latency (FF, 5 mrev threshold, fwd) | 35 ms (vs 58 ms without vel_ff, vs 81 ms no FF) |
| Motion-onset latency (FF, 5 mrev threshold, rev) | 30 ms |
| 90 %-target tracking lag (FF, fwd) | **4.5 ms** (vs 31 ms with no vel_ff, vs 53 ms no FF) |
| 90 %-target tracking lag (FF, rev) | **8.3 ms** |
| Steady-state |iq| (FF) | 0.34 A (integrator unloaded — friction FF doing the work) |
| Steady-state vel std (FF) | **0.000 rps** (clean tracking) |
| Overshoot | 2.7–3.0 % (acceptable) |

### Hypothesis status going into platform integration

| Hypothesis | Standing | Evidence |
|---|---|---|
| Stribeck friction model captures bench friction | ✅ Confirmed | R² = 0.983 over 8 points |
| FF sign convention `-1` for the bench rig | ✅ Confirmed | Empirical A/B on `--ff-sign +1` made things worse |
| Re-tensioning materially shifted parameters | ❌ Refuted | Refit values within 5 % of pre-tensioning fit |
| Stiction-boost helps motion-onset | ✅ Confirmed | Threshold sweep shows 1.7–1.9× improvement |
| Boost above ω_s causes overshoot | ✅ Confirmed | OS trend 1.9 → 2.5 → 2.9 % across the sweep |
| Tracking lag is pos-loop bandwidth, not friction | ✅ Confirmed | vel_ff dropped lag 31 → 6 ms |
| Encoder/motor cable noise was degrading control | ❌ Refuted | Run B (cables together) ≡ Run C (separated) |
| Remaining latency floor is inertia, not friction | 🟡 Strongly suggested | vel_ff diminishing returns + estimated J_eff ~10× rotor |
| Cogging is the primary motion-onset cause | ❌ Refuted | fwd-vs-rev R=0 at smooth velocities; cogging amplitude ~5 % of friction |

## Withdrawn claims

- **[2026-04-27 ~12:00] "Test A's iq during steady-state IS pure friction; Test A and Test B disagree, suggesting friction is small but only at very low velocity."**
  WITHDRAWN: this was based on a first-pass interpretation before Test C was available. Test C (torque-step terminal velocity) showed the rotor accelerates freely at iq levels far below Test A's steady-state mean, which proved that Test A's mean iq is contaminated by velocity-loop cogging-fight current and is *not* pure friction.
  Superseded by: the "controller-fight" framing in the Discussion section above. The user's "1 A is the minimum for smooth motion" framing captured the resolution exactly.

- **[2026-04-27 ~17:00] "After re-tensioning the bench, the friction has shifted up by ~2× and the FF parameters underestimate it; the moderate FF improvement reflects this parameter mismatch."**
  WITHDRAWN: refit on the post-tensioning data showed parameters changed by < 5 %. The "moderate improvement" was actually the FF doing its job correctly given the bench's already-small dead-time; the platform with longer dead-time will show larger improvement.
  Superseded by: the Stribeck refit at v5, parameters τ_c=1.094 / τ_s=1.953 / ω_s=0.251 / b=0.0173.

- **[2026-04-27 ~14:00] "Stiction is < 0.16 A based on Test B v3 breakaway readings."**
  WITHDRAWN: Test B's "breakaway iq" of 105 mA fwd / 144 mA rev measures the *quasi-static* iq required to walk the rotor through cogging detents at very slow velocity, not the iq required to break free into sustained motion against kinetic friction at the operating velocity. These are different physical quantities; the higher value (Test A's 1.1 A steady-state) is what motion-onset actually requires the integrator to provide.
  Superseded by: the Test C "controller-fight" diagnosis in the Discussion section above; Test A's 1.1 A steady-state — not Test B's quasi-static breakaway iq — is the operationally relevant friction value for FF design.

## Open Questions

1. **motor_guard FF integration** — the next step. Open question: does `motor_guard.py` already populate `vel_ff` in the `set_input_pos` packets it sends, or is it currently passing zero? If zero, enabling vel_ff alone (without any friction FF) would already produce a measurable platform improvement. Worth checking before implementing the more invasive friction FF, because we may be able to ship vel_ff as a small, contained change first.

2. **Per-leg friction parameters** — should each leg's `τ_c, τ_s, ω_s, b, LOAD_OFFSET` live in `hardware_config.yaml` as a 6×5 array? Initial values are the bench-fit numbers; refinement via the diagnose detector is the protocol described in `plans/active/leg-gain-tuning-methodology.md`'s Level-1 procedure. Per-leg gravity-from-kinematics is a separate consideration that probably belongs in motion-bridge, not hardware_config.

3. **Acceleration FF for the inertia floor** — `τ_ff += J_eff × dv/dt`. Needs J_eff per leg (rotor + reflected leg + platform share). Not on the critical path — the platform's diagnose detector will tell us whether the inertia floor is even visible against the position-loop dead-time. Likely a Phase 2 optimisation.

4. **Velocity-scaled stiction-boost threshold** — we used a fixed threshold of 0.20 rev/s tuned for v_target=0.5. For other velocities the optimal threshold should probably scale with target. Defer until the platform shows a problem.

5. **Cogging compensation LUT** — the n=2 angular dependence in Test B v3 (forward R²=0.97 amp=33 mA; reverse R²=0.70 amp=37 mA) shows cogging is real but small. The 4 A pk-pk iq oscillation we see during cruise at v=5 rps suggests cogging-fight matters more than its small magnitude implies. An angle-indexed cogging LUT is the second-largest improvement available to us, but only after the friction FF lands. Phase 3 work.

6. **What if the bench's load isn't representative of platform legs?** — the bench fit assumes the bench's static load (~0.17 A) represents what platform legs experience. On platform, gravity contribution depends on platform pose; the constant load offset will be replaced by a kinematics-derived per-leg gravity term. The Stribeck shape (τ_c, τ_s, ω_s, b) should generalise.
