---
title: Motion-onset dead-time — cogging-torque-first investigation
type: investigation
date: 2026-04-20
status: in-progress
phase: post-per-leg-gains-deadband-session
related_plan: motion-onset-deadtime-investigation.md
related_issues:
  - 2026-04-18-hold-fighting-motion-onset-jitter.md
  - 2026-04-19-leg1-pose-dependent-hold-twitch.md
sessions:
  - mpc_20260418_164119.csv
  - mpc_20260418_181020.csv
  - mpc_20260419_134901.csv
  - mpc_20260419_134919.csv
  - mpc_20260419_134931.csv
  - mpc_20260419_134947.csv
  - mpc_20260419_135002.csv
  - mpc_20260419_135030.csv
  - mpc_20260419_135059.csv
  - mpc_20260419_135140.csv
  - mpc_20260419_135251.csv
  - mpc_20260420_160401.csv
  - mpc_20260420_182945.csv
files_changed:
  - config/hardware_config.yaml
  - sim/analysis/diagnose.py
  - tools/motion_onset_cogging_study.py
  - logbook/2026-04-20-motion-onset-dead-time-fix.md
commits:
subsystem:
  - motion
  - can
  - controller
  - config
tags:
  - motion-onset
  - stiction
  - cogging
  - feedforward
  - hardware-bringup
---

# Motion-onset dead-time — cogging-torque-first investigation

## Summary

Session picking up from `plans/active/motion-onset-deadtime-investigation.md` §4.2 (mechanical audit). Working hypothesis revised: stiction + cogging torque are the primary candidates, backlash demoted. User has a standalone ODrive + "tester leg" bench rig available (no platform, no brake resistors, no fast movements allowed), which unlocks a cogging-torque bench measurement that was not tractable on the platform. Plan is four steps: (0) desk-check existing data for a cogging fingerprint, (1) bench cogging map on unloaded tester leg, (2) bench breakaway fingerprint vs rotor angle, (3) loaded repeat. Fix design follows from the decision tree at Step 4.

## Symptoms

Inherited from `plans/active/motion-onset-deadtime-investigation.md` §0. Detector already landed (commit `a41b17f`). Baseline across ~7 sessions: median onset latency 100–190 ms, max up to 250 ms after cold start, 6-leg synchrony within a single 25 ms tick, first-tick leap 0.5–3.5 mm.

Cogging-specific predictions (to be tested in Step 0):

- Leap magnitude should correlate with rest electrical angle if cogging dominates: detent-parked rotors need more breakaway torque, produce larger overshoot on release.
- 7 pole pairs × 12 stator slots → cogging period of `2π / lcm(7,12) = 2π / 84` electrical rad per mechanical revolution, i.e. 84 detent cycles per motor revolution. Expect cogging signal at `84 × (motor rev / leg mm)⁻¹ ≈ 84 × 0.000275 ≈ 0.023 rev`-worth of leg extension per period, i.e. **~0.023 mm per cogging period** (if `mm_to_rev ≈ 2.75e-4 rev/mm`). Verify per-leg via `hardware_config.yaml:125`.

## Diagnosis

### Step 0 — Desk correlation: NULL result (2026-04-20)

Executed `tools/motion_onset_cogging_study.py` across all 131 session CSVs under `temp/logs/`, yielding 873 motion-onset events with derived rest electrical angle.

Per-leg verdict: **no cogging signature** on any leg.

Pooled harmonic-fit R² (leap magnitude vs `sin(n·θ_mech) + cos(n·θ_mech) + C`):

| n    | Meaning                          | R²     | Amplitude | Spearman ρ vs detent distance |
|------|----------------------------------|--------|-----------|--------------------------------|
| 7    | pole pairs                       | 0.001  | 0.9 mm    | +0.167                         |
| 12   | stator slots                     | 0.002  | 1.1 mm    | +0.063                         |
| 14   | 2× pole pairs (magnetic cycle)   | 0.006  | 2.2 mm    | +0.219                         |
| 42   | 6th-harmonic of magnetic cycle   | 0.004  | 1.8 mm    | +0.185                         |
| 84   | lcm(14,12) cogging fundamental   | 0.010  | 2.6 mm    | +0.238                         |

Per-leg counts: 142–150 events per leg, 873 total. Mean leap 3.5–5.7 mm per leg, std 13–22 mm — leap distribution is very wide, dominated by commanded-velocity variability across the 131 heterogeneous sessions (different poses, different tune iterations, different warmup states).

**Interpretation.** The data does NOT confirm a cogging fingerprint in the expected harmonics. Spearman at n=84 (+0.24) is directionally correct for cogging — parking near a detent producing a larger leap — but below the |ρ| > 0.3 threshold chosen for a clean verdict. R² of 0.01 is noise-floor.

**Caveats before demoting the cogging hypothesis:**

1. The leap metric conflates stiction-release leap with ordinary commanded-velocity leap. Events where the detector caught a fast cmd ramp mid-transit inflate leap without being breakaway signatures. This adds variance orthogonal to cogging and depresses any cogging-dependent signal.
2. 131 sessions span ~6 weeks of tune history. Gain asymmetries that existed mid-session (the Iteration-3 state with legs 1 and 4 at 30/0.24) are pooled with the current uniform state, further adding variance unrelated to cogging.
3. Cogging on this motor is *predicted* to be small in absolute terms (D6374 is a gimbal-style outrunner, typically 50–200 mA peak-to-peak of `iq` ripple). Spatial cogging torque is finite; whether it *dominates* motion-onset is a separate question answered only by a direct measurement.

**Conclusion.** Step 0 fails to demote stiction below cogging but also fails to promote cogging to #1. A bench measurement is the decisive test because it isolates cogging from every other noise source in one shot. Proceed to Step 1.

### Artifacts

- `temp/reports/motion_onset_cogging_study/scatter.png`: leap vs rest electrical angle, per leg.
- `temp/reports/motion_onset_cogging_study/harmonic.png`: R² per harmonic per leg.
- `temp/reports/motion_onset_cogging_study/report.json`: full per-leg + pooled result dict.

### Flagged Issues

None from this step. Detector changes are additive (new fields on existing event dicts, no breaking schema).

## Plan

### Step 0 — Desk correlation of leap magnitude vs rest electrical angle (no hardware)

Objective: determine whether a cogging fingerprint is present in existing session data before committing bench time.

Implementation:

1. Add `motor_pole_pairs: 7` and `motor_stator_slots: 12` to `config/hardware_config.yaml` under the leg motor parameters; regenerate config constants.
2. Extend `sim/analysis/diagnose.py::analyse_motion_onset` to additionally compute, per event per leg:
   - `rest_elec_angle_rad`: `(pos_estimate[onset − 1] × pole_pairs × 2π) mod 2π`
   - `rest_mech_angle_rad`: `pos_estimate[onset − 1] × 2π mod 2π`
   - Already-present `first_tick_leap_mm`, `latency_ms`.
3. Write a one-shot analysis script `tools/motion_onset_cogging_study.py` that iterates every `temp/logs/mpc_*.csv`, invokes the enhanced detector, and produces:
   - Scatter plot: leap_mm vs rest_elec_angle, one subplot per leg.
   - Histogram: rest angles binned to 12 cogging periods; median leap_mm per bin.
   - Fit: `A·sin(n·θ + φ) + B` sweep over n ∈ {7, 12, 14, 84} to identify dominant cogging period.
   - Repeat with `latency_ms` as the y-axis.
4. Interpret:
   - Clear sinusoidal modulation at n=7, 12, 14, or 84 → cogging material. Proceed to Step 1.
   - No modulation, flat distribution → cogging demoted. Fall back to stiction-only branch (friction-ID bench test as §3.5 prerequisite).
   - Partial modulation with high scatter → cogging present but not dominant. Proceed to Step 1 but with lowered expectation.

Decision threshold: Spearman correlation coefficient between leap magnitude and nearest-cogging-detent-distance. |ρ| > 0.3 is evidence of cogging signal above the session noise.

### Step 1 — Bench: slow-rotation cogging map (unloaded tester leg)

Pending Step 0 outcome. Protocol will be:

- Tester leg on standalone ODrive, no load, no brake resistor → velocity command ≤ 0.1 rev/s only (safe operating envelope).
- Velocity mode at 0.05 rev/s across one full motor revolution (~20 s duration).
- Log `iq_measured` and `pos_estimate` via `odrivetool` at max available rate.
- Extract `iq(θ_elec)` at steady-state velocity → direct cogging torque map.
- Decision:
  - Ripple < 30 mA pk-pk: demote cogging, revisit stiction branch.
  - 30–100 mA: contributing but not dominant; Stribeck FF likely sufficient.
  - \> 100 mA: material; feedforward needs cogging-compensation table.

### Step 2 — Bench: breakaway fingerprint (unloaded tester leg)

Pending Step 1 outcome. Protocol will be:

- ODrive in torque control, slow `iq` ramp from 0 upward until motion begins.
- Repeat 10× at 10 different starting rotor angles (~36° mechanical apart).
- Measure `iq_breakaway(θ_start)` → combined stiction + cogging breakaway per angle.
- Position-independent → pure stiction. 2×+ variation → cogging modulates breakaway.

### Step 3 — Bench: loaded repeat

Pending Step 2 outcome. Protocol will be:

- Attach 5 kg representative load to tester leg output.
- Repeat Steps 1 and 2.
- Expected: cogging ripple unchanged (load-independent), stiction breakaway scales with load.

### Step 4 — Fix design

Decided from Steps 0–3 outcomes. Options:

- **Cogging dominant**: electrical-angle-indexed torque FF LUT in motor_guard, 500 Hz, indexed on `pos_estimate × pole_pairs`.
- **Cogging + stiction**: overlay cogging LUT with velocity-indexed Stribeck FF (§3.5).
- **Stiction + cogging modulation of breakaway**: velocity-Stribeck with a small cogging bias.
- **Stiction only**: plain Stribeck FF per §3.5.

## Discussion

User's prior (documented 2026-04-20): "all legs are very similar to each other; stiction AND cogging torque are primary candidates." Matches the observed 6-leg synchrony signature and the session-to-session variance in leap magnitude across similar poses. Tester leg is asserted representative of the platform legs (same build tolerances).

Electrical angle is not exposed over CAN directly (verified 2026-04-20: `Get_Encoder_Estimates` frame 0x09 publishes `pos_estimate` in rev, no dedicated electrical-angle frame). Derivation is straightforward: `θ_elec = (pos_estimate × pole_pairs × 2π) mod 2π`. Pole pairs = 7 from `config/ODrive config Files/odrive_pro_leg_config.json:145`. Stator slots = 12 per user confirmation; adding both to `hardware_config.yaml` so derivation is a config lookup.

## Fix

Pending — no code touch until Step 0 analysis yields a verdict.

## Open Questions

- Does Step 0 show a cogging fingerprint in existing data?
- If cogging is material, is the compensation best applied in motor_guard (500 Hz, pre-CAN-encode) or pushed down to the ODrive as a per-leg `torque_offset` table? Motor_guard side keeps the control story in one place; ODrive side offloads the 500 Hz work.
- Tester leg is asserted representative. How much inter-leg cogging variance do we expect in practice? If leg-to-leg cogging ripple varies by more than a factor of ~2, a single bench map won't generalise and we'd need per-leg measurement.
