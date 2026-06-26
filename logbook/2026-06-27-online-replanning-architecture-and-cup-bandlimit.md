---
title: Pivot to online per-throw task-space planning (Kai-style) + cup-tracking band-limit characterisation; planner module built, integration next
type: feature
date: 2026-06-27
status: in-progress
phase: "Sim2Real fidelity — concern 1: re-architecting to an online cup planner (replacing the offline optimiser); planner built + tested, integration WIP"
related_plan: "bb-led-two-ball-juggle-demo.md"
files_changed:
  - controller/demo/juggle_planner.py
  - tests/sim/test_demo_juggle_planner.py
  - tools/probes/juggle_cup_bandlimit.py
  - tools/probes/README.md
commits:
  - TBD
subsystem:
  - controller
  - sim
tags:
  - control
  - planning
  - tracking
  - kinematics
  - testing
---

# Online per-throw task-space planning + cup-tracking band limit

## Summary

The offline-trajectory architecture is fundamentally defeated by the platform
band limit (prior entry `2026-06-27-throw-aim-band-limit-and-closed-loop-catch`):
replaying a fixed plan that assumes perfect tracking accumulates errors a
band-limited platform can't track. The user chose to **re-architect** toward Kai
Ploeger's kinematic toss-juggling approach
(`kinematic_planning_for_nball_toss_juggling`, IEEE 9981678): an **online**
planner re-solved **once per throw**, in **task space** (the cup's Cartesian
trajectory), **against the observed ball** and from the **achieved** cup state —
so errors self-correct each cycle instead of accumulating. The MPC/planner
becomes a *higher-level* planner feeding the low-level tracker, which is how the
hardware stack is meant to work.

This session **characterised the band limit** that grounds the planner,
**committed to a realisation design** (level-platform decoupling), and **built +
unit-tested the planner module**. Integration into the runner (replacing the
offline optimiser + open-loop player) is the next step; a known catch-geometry
issue is documented for it.

## Band-limit characterisation (`tools/probes/juggle_cup_bandlimit.py`)

Lock-in Bode + static sweeps of the two cup actuators, at the pattern's
operating point (centroid z=170 mm, cup ~0.67 m):

- **Slider (vertical cup): near-perfect.** Static 1:1 exactly; dynamic amplitude
  ratio ~1.00 up to 5 Hz (0.98 at 8 Hz), phase lag a few degrees, achieved
  velocities to ~4 m/s. The slider is our fast, accurate actuator — Kai's
  kinematic hand, essentially.
- **Platform (lateral cup): static-exact, modestly band-limited.** At z=170,
  static tracking is **exact** (cmd 40/80/100/120/150 mm → achieved
  40/80/100/120/150) and ±150 mm is reachable; dynamic **−3 dB at ~5 Hz**
  (0.95 / 17° at the 1.6 Hz cycle freq, 0.85 / 30° at 3 Hz). It tracks *smooth*
  lateral motion well; only *whippy* (fast-reversing) motion is attenuated.
- **Artefact ruled out**: an initial z=0 sweep showed a "0.68 static gain" that
  was pure **leg saturation** (a leg pinned at the −5 mm ctrl floor) — gone at
  the operating z=170 where the legs have headroom. The platform's lateral
  problem is dynamic (phase lag at fast maneuvers), not a static deficit.

## Design decision — level-platform decoupling

Plan the cup's 3-D Cartesian trajectory (Kai-style); realise it with a **level**
platform:

- `centroid_xy = cup_xy` → the platform supplies the *lateral* cup motion (slow,
  smooth → tracks at 0.95; ±150 mm workspace),
- `slider = cup_z − z_fixed − offset` → the **slider** supplies the *fast
  vertical* throw stroke (perfect 1:1 tracking).

This puts the fast, hypersensitive part of the throw on the **perfect** actuator
and only smooth lateral motion on the band-limited one — and **drops banking
entirely** (no tilt → no ω×r, no large centroid-vs-slider cancellation that
wrecked the offline optimiser). Per-axis jerk limits encode the split: high
vertical (slider), modest lateral (platform, to stay in its good-tracking band).

This is the significant design decision of the session; it is well-grounded in
the characterisation above and reversible on the branch, so it was taken under
the overnight-autonomy grant rather than blocking.

## Planner module (`controller/demo/juggle_planner.py`)

Adapts Kai's `plan_throw` to our single cup, SI units, pure CasADi:

- Decision: the cup's Cartesian **jerk** over one cycle; triple integrator →
  pos/vel/acc; objective = mean-squared accel + a lateral-accel smoothness term.
- **Throw** (end of cycle, HARD): cup pos = throw point, cup vel = the ballistic
  take-off velocity to land the ball at the throw target one flight later, cup
  **acc = gravity** (free-fall at release → clean detach), and a 2-knot
  no-lateral-acceleration detach.
- **Catch** (interpolated touch-down time, HARD pos + SOFT vel): cup pos = the
  *observed* ball's predicted touch-down (via `ballistic_touchdown`), cup
  velocity softly matched to `0.7 ×` the ball's arrival velocity. The velocity
  match is **soft**, not Kai's hard collinearity, because a hard velocity
  constraint conflicts with the band-limited platform's modest lateral jerk and
  makes the NLP infeasible.
- Per-axis jerk bounds (`max_jerk_z` high / `max_jerk_xy` modest), xy workspace
  box, z box. Re-solved each cycle from the achieved cup state + observed ball.

Validated by `tests/sim/test_demo_juggle_planner.py` (7 tests): the throw
constraints are exact (pos/vel/acc to 1e-6), the catch position is matched at
the interpolated time, the ballistics helpers are correct, and the per-axis
jerk bounds hold.

## Discussion — the catch-geometry issue surfaced (for integration)

The standalone planner exposed a real constraint of **our pattern** vs the
platform: with throw at +100 mm and catch at −100 mm (200 mm separation) and a
0.22 s dwell (catch→throw), the cup must move *with* the ball (−x) at the catch
**and** reverse to +x to reach the throw in time — a double lateral reversal that
exceeds the platform's lateral jerk (saturates at the bound). The planner's
best effort leaves the cup moving **+0.99 m/s** laterally at the catch while the
ball moves −0.19 → a ~1.2 m/s lateral swipe that will likely spoil the seat.
Stronger catch-velocity weight fixes only the vertical (slider); the lateral is
geometry-limited.

This is the band limit re-expressed as a pattern constraint. Kai sidesteps it by
**carrying inward** (catch at the outer point moving inward, carry, throw from
the inner point — the hand moves consistently, no double reversal). Options for
integration, in order of preference:
1. Re-time / re-phase so the catch isn't at the lateral extreme with a hard
   reversal (a Kai-style carry, or a longer dwell / gentler tempo).
2. Reduce the lateral separation (smaller shuttle) — but the user set 200 mm.
3. Accept a lower catch velocity-match and lean on the cup geometry + per-cycle
   re-planning (observe-and-reach). Resolve empirically against the sim.

## Integration plan (handoff — next step)

1. **Realisation** `cup_target_mm → (platform pose, slider mm)` (level
   decoupling): `pose = [cup_x, cup_y, z_active, 0,0,0]`; `slider = clamp(cup_z −
   C, 0, stroke)`, with `C` from the static map (cup_z_world ≈ 659.6 + slider at
   z_active=170, i.e. `slider ≈ cup_z_mm − 659.6`). **De-risked** (`/tmp
   probe_realize`): driving a planned cycle through this mapping, the cup
   *follows the plan* — lateral RMS ~26 mm (the band-limit lag on the ±100 mm
   shuttle, to be absorbed by the per-cycle re-plan), and the throw/catch z
   stroke tracks. Two realisation constraints surfaced and MUST be honoured: the
   planner z-box must equal the slider's reach (~0.66–1.01 m — a 0.45 m floor
   clamped 124 mm), and the throw release must sit MID-range (~0.85 m), because
   the cup overshoots ~0.12 m after a ~5 m/s release before it can decelerate (a
   0.80 m release overshot to 1.04 m, above the slider top → infeasible/clamp).
2. **Runner**: replace `juggle_optimizer` + `TrajectoryPlayer` in
   `sim/juggle_demo.py` with the online loop — each throw, observe the incoming
   ball (`ballistic_touchdown` to the catch height), `plan_cup_cycle(...)` from
   the achieved cup state, and drive the platform+slider via the realisation
   (sub-stepped). Keep the `MasterTimeline` event phasing + `BallButlerSim`
   priming + contact physics.
3. **Iterate** the 2-ball pattern; address the catch-geometry issue (above);
   re-tune.
4. Tests / plan update / audit / commit. The two `test_demo_juggle_sim` headline
   cases remain `xfail` until the pattern closes.

## Verification

- `pytest tests/sim/test_demo_juggle_planner.py -q` (run 2026-06-27): **7
  passed**. Full suite unaffected (the planner is a new standalone module; the
  runner is untouched, so the demo still runs at the prior 2 catches).
- Band-limit numbers above from `tools/probes/juggle_cup_bandlimit.py`
  (run 2026-06-27).
