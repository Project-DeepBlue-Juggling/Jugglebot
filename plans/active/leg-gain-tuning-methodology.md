---
title: Leg ODrive PID Gain Tuning Methodology
status: active
owner: harrison
created: 2026-04-18
related_logbook:
  - 2026-04-18-hold-fighting-motion-onset-jitter.md
related_config:
  - config/hardware_config.yaml → jugglebot_odrive_defaults.leg_pos_gains / leg_vel_gains / leg_vel_int_gains
related_code:
  - ros_ws/src/jugglebot/jugglebot/can_node.py::_set_leg_gains
  - ros_ws/src/jugglebot/jugglebot/can/odrive.py::DEFAULT_LEG_GAINS
---

# Leg ODrive PID Gain Tuning Methodology

## Purpose

Per-leg ODrive position-loop gains (`pos_gain`, `vel_gain`, `vel_integrator_gain`)
are now configurable via `config/hardware_config.yaml` and applied over CAN at
boot by `can_node._set_leg_gains()`. This document describes how to pick the
numbers that go in the YAML.

Three levels are described, in increasing order of rigour and time cost. Pick
the level that matches the claim you want to defend:

- Level 1 — empirical A/B → "this gain makes leg N quieter at hold without
  hurting tracking"
- Level 2 — step-response → "this gain gives leg N a 45 ms rise time and 8 %
  overshoot to a 5 mm position step"
- Level 3 — system ID + loop shaping → "this gain gives leg N a 60° phase
  margin at a crossover frequency 10× its mechanical resonance"

## Context: why per-leg gains?

The six Jugglebot legs are mechanically near-identical but not truly identical.
`mm_to_rev` already varies per leg (`config/hardware_config.yaml:125`), reflecting
build-to-build differences in spool diameter and thread engagement. Bearings,
couplers, and cable routing add further leg-specific compliance and friction.
A single uniform gain vector that works for the stiffest leg will over-drive
the softest leg and produce the exact audible motor-fighting signature
described in `logbook/2026-04-18-hold-fighting-motion-onset-jitter.md`.

Per-leg gains let us push each leg as hard as its mechanics will tolerate,
without sacrificing any leg to the weakest member.

## Assumptions and invariants

Across all three levels:

- `MAX_LEAD_REV = 0.15 rev` clamp in `motor_guard.py` remains in place as the
  hard safety floor. No amount of gain mis-tuning can cause a runaway faster
  than this clamp allows.
- Workspace/position limits in `can_node._send_position_target` remain in
  place. A leg cannot be driven outside its stroke.
- All gain edits go through `config/hardware_config.yaml`, then
  `python config/generate_config.py`, then a ROS2 rebuild. Never set gains
  live via ODriveGUI as a durable configuration — those values are lost on
  flash re-program and are invisible to the codebase.
- Keep `min(pos_gain) / max(pos_gain) ≥ 0.4` across the six legs. Larger
  asymmetry drives platform twist on straight-line moves (leg N lags → pose
  tilts during transit).
- Keep the ratio `pos_gain : vel_integrator_gain ≈ 125 : 1` unless you have a
  specific reason to break it (Level 2 or 3 tuning). This ratio preserves the
  outer-to-inner-loop bandwidth relationship that the ODrive firmware assumes.

## Level 1 — Empirical A/B on the outlier

Use when: you have an observed hold-phase jitter outlier and want to bring
it into line with the quietest leg. Cost: ~30 min per leg. Defensibility:
low — you're tuning against a single operating point and a single metric.

### Procedure

1. **Baseline measurement.** With all six legs uniform (current default
   `40.0 / 0.20 / 0.32`), run one Move 1 (Active → z=220, 15 s) and hold for
   5+ s at the target. In the resulting `temp/logs/mpc_*.csv`, compute
   `actual_ext_i` stdev per leg over the last 3 s at hold. That stdev in
   micrometres is the hold-quiescence metric. Rank the six legs by it.
2. **Target.** The quietest leg sets the floor — call it μ_best. Any leg with
   `actual_ext stdev > 1.5 × μ_best` is an outlier to tune.
3. **Halve → test → go between or halve again.**
   - Try `pos_gain × 0.5` on the outlier. Keep all other legs unchanged.
   - Edit `config/hardware_config.yaml`, run `python config/generate_config.py`,
     rebuild ROS2 (`cd ros_ws && colcon build --packages-select jugglebot &&
     source install/setup.bash`).
   - Re-run Move 1.
   - If hold stdev drops toward μ_best and tracking RMS does not rise more than
     1.5× on the pure-Z move: good direction.
   - If halving overshoots (new stdev < μ_best *but* tracking RMS or final
     settling error grows materially): try `pos_gain × 0.75` instead.
   - If halving didn't help at all: `pos_gain` wasn't the problem on this leg.
     Move to `vel_integrator_gain` (halve it independently, keep `pos_gain`
     at baseline).
   - `vel_gain` stays constant unless both outer and integrator tuning fail —
     that's a signal the inner loop bandwidth is wrong, which is rare.
4. **Ratio rule.** When you drop `pos_gain` by factor α, drop
   `vel_integrator_gain` by the same α. This keeps the integrator wind-up
   proportional to the now-sluggish position loop.
5. **Stop condition.** All six legs within 1.5× of μ_best on hold stdev AND
   aggregate tracking RMS within 10 % of the pre-tuning value on every move
   in the recommended test battery (pure-Z up, pure-Z down, pure-X, pure-Y,
   diagonal).
6. **Record.** Final values get committed to `config/hardware_config.yaml`;
   the tuning session gets a logbook entry with before/after hold-stdev
   numbers, final gain values, and the `/diagnose --compare` pre/post
   comparison.

### Failure modes and what they look like

| Symptom | Diagnostic | Fix |
|---|---|---|
| Platform drifts in gravity; sluggish start of motion | Tracking RMS up, final settling > 1 mm | Position loop too soft — back off gain reduction |
| Sudden 5–10 mm correction long after move ends | Integrator wind-up under load | Reduce `vel_integrator_gain` first, before touching `pos_gain` |
| Platform twists on straight-line move | `min_pos_gain / max_pos_gain < 0.4` between legs | Tighten the asymmetry — raise the weakest leg or lower the stiffest |
| Hold stdev improved but a new leg now fights | Re-ranked outlier | Expected — iterate |

### Test battery for Level 1 acceptance

Every round of gain changes must pass all of these before the new values
are committed:

1. `Active → z=220, hold 5 s` (long Z-up)
2. `z=220 → Active, hold 5 s` (long Z-down)
3. `Active → x=50, hold 5 s` (pure X)
4. `Active → y=50, hold 5 s` (pure Y)
5. `Active → (30, 40, 200, 0.05, 0.05, 0), hold 5 s` (diagonal with tilt)

Pass criteria: per-leg hold stdev within 1.5× of μ_best on every move, no
error-severity flags from `/diagnose`, no motor errors in the rosbag.

## Level 2 — Step-response tuning

Use when: you need a defensible number for a logbook entry, a paper, or a
spec document. Cost: ~2 h per leg. Defensibility: medium — you're tuning
against a standardised input and a small set of well-known metrics.

### Procedure

1. **Switch the leg under test to TRAP_TRAJ control mode** (via CAN:
   `encode_set_controller_mode(axis_id, 'POSITION', 'TRAP_TRAJ')`). Other
   legs stay in PASSTHROUGH under normal operation or are disarmed — the
   point is to isolate one leg's dynamics.
2. **Support the platform mechanically** so the leg under test sees
   approximately its operational load (the platform + gravity share allocated
   to that leg at home pose). Do not run this test with the platform in free
   hang — the load is wrong.
3. **Command a 5 mm step** on `input_pos` using a Python script that talks
   to the leg directly over CAN. 5 mm is large enough to exit the noise
   floor but small enough to stay in the linear regime.
4. **Capture `pos_estimate` at 500 Hz** from the rosbag `/leg_lengths_topic`.
5. **Fit:**
   - `t_rise` (10 % → 90 % of step height)
   - `t_settle_2%` (time until error stays within ±2 % of final)
   - `% overshoot` (peak deviation past final, as fraction of step height)
6. **Targets for a compliant Stewart leg:** `t_rise ≈ 30–50 ms`,
   `overshoot ≈ 5–10 %`, `t_settle_2% ≈ 100 ms`. These come from the
   mechanical-bandwidth-to-control-bandwidth separation required for
   Stewart platforms at this scale; confirm with a small pilot before
   using them as hard targets.
7. **Binary-search `pos_gain`** to hit `t_rise` and `overshoot` targets:
   - Too slow rise → raise `pos_gain`
   - Too much overshoot → lower `pos_gain` OR raise `vel_gain`
8. **Add `vel_integrator_gain`** until final error goes to zero within
   `t_settle_2% × 2`. Increase by 50 % per trial; watch for late oscillation
   (the signature of too-high integrator gain).
9. **Repeat per leg.** Legs with different `mm_to_rev` will settle on
   different gains — that's the expected outcome.
10. **Record** each leg's fitted rise/overshoot/settle alongside its final
    gains in the logbook entry.

### Gotchas

- **Load matters.** A leg tuned with no platform weight will oscillate when
  the platform is added. A leg tuned at home pose may be soft at extension
  extremes.
- **Backlash eats the first ~0.2 mm of any step.** Use a 5 mm step or larger
  to keep backlash a small fraction of the signal.
- **The first move after a long rest has higher stiction than subsequent
  moves.** Do three warmup steps before collecting data.
- **TRAP_TRAJ itself imposes a velocity/accel profile.** This is the right
  mode for PID step response (versus PASSTHROUGH which gives an instantaneous
  step) because it isolates tracking performance from setpoint slew. If you
  want to characterise the PID against an instantaneous step, use
  PASSTHROUGH and a jog command — but be ready for the clamp on leg stroke
  to fire if the response is unstable.

## Level 3 — System ID + loop shaping

Use when: Level 2 hits a ceiling, or when you want explainable reproducible
gain selection as part of a spec. Cost: ~1 day per leg. Defensibility:
high — you're deriving gains from a measured plant model with specified
margins.

### Procedure

1. **Command a multisine chirp** into `input_pos` from 0.5 Hz to 80 Hz. 80 Hz
   comfortably exceeds the 40 Hz MPC rate and captures structural resonance.
   The chirp should have no DC component and amplitude small enough to stay
   linear (1–2 mm peak-to-peak is a reasonable starting point).
2. **Capture `pos_estimate` at 500 Hz and motor current at the fastest
   available rate.** Both are needed for the full plant model.
3. **Fit a second-order plant model per leg:**

   ```
   G(s) = ω_n² / (s² + 2ζω_n·s + ω_n²)
   ```

   Expected characteristics: ω_n in the 20–80 Hz range (mechanical
   resonance), ζ in the 0.1–0.4 range (moderately underdamped). Large
   inter-leg variance in ω_n is a signal of mechanical unit-to-unit
   variation and explains Level 1 outlier behaviour.
4. **Synthesise a PID** with:
   - Phase margin ≥ 60°
   - Gain margin ≥ 6 dB
   - Crossover frequency ≈ 10× the mechanical resonance of the softest leg
     (not the average — protect the weakest member)
5. **Verify on an independent trajectory** (not the chirp): command a real
   juggling-style move and measure tracking RMS, peak tracking error,
   disturbance rejection (apply a small mechanical tap during hold and
   measure settling).
6. **Iterate** until margins hold across all six legs simultaneously.

### When Level 3 becomes necessary

- You're making a claim in a paper, spec, or customer-facing document about
  dynamic performance
- You want to predict how the platform will behave under disturbances
  that weren't in your test battery (e.g., ball impact during a catch)
- Level 2 tuning leaves you with gains that pass the step-response criterion
  but fail in practice on full trajectories — this usually means the plant
  is higher-order than second-order and Level 2's targets were too loose
- You want to add a feedforward path based on the fitted plant inverse

## Recommendation for the current hardware-bringup stage

Stay at Level 1 until at least one of:

- Level 1 converges (per-leg gains committed, hold-phase fighting measured
  down to within 1.5× of μ_best across all six legs)
- Level 1 hits a ceiling (can't get leg N below some floor without hurting
  tracking; floor is above μ_best by more than 1.5×)
- A juggling-timing claim starts depending on gain-level behaviour

If Level 1 converges and stays converged across a week of test sessions,
we probably never need Level 2 for the current juggling-accuracy budget.
If Level 1 hits a ceiling, escalate to Level 2 for the leg in question
only — don't rebuild Level 2 across all six legs unless needed.

## Open questions for later

- Does leg 1 + leg 3 being the worst pair correlate with a specific
  manufacturing batch or assembly position in the hex? If so, we can
  predict outliers from the physical leg serial numbers rather than
  discovering them at each recommission.
- Would a simple online adaptation (e.g., auto-tune the integrator from
  hold-phase stdev during the first 5 s at Active) remove the need to
  commit gain numbers to YAML at all? Explore in Phase 5+ once we have
  a baseline Level 1 result.
- How do the gains need to change when catching a ball (brief high
  transient load)? Level 3 would let us predict this; Level 1 cannot.
