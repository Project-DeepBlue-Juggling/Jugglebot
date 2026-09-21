---
title: "Ball Butler throw solver: the pitch sweep is replaced by the steepest feasible pitch, found directly — near-field targets are refused"
type: optimization
date: 2026-09-18
status: done
phase: "ball-butler"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py (`_pitch_eval`, `_steepest_feasible_pitch`, `_NoFeasiblePitch` / `_NearFieldTarget`; `solve_throw_local` loses its grid and its `pitch_step_deg` argument)
  - sim/ball_butler/sim.py (verbatim twin of the same core; `_solve_throw` loses its sweep)
  - tests/_bb_pitch_grid_oracle.py (NEW — the retired sweep, arithmetic untouched, test oracle only)
  - tests/ros/test_throw_ballistics.py (four-regime oracle test, apex-on-the-cap, near-field refusals; the finer-grid smoke test deleted with its argument)
  - tests/sim/test_ball_butler_sim.py (`TestPitchCoreTwin` — the sim copy is bit-identical to the ROS core)
subsystem:
  - can
  - sim
tags:
  - performance
  - testing
---

## Motivation

`solve_throw_local` (and its hand-ported sim twin `_solve_throw`) picked the pitch that minimises
horizontal landing velocity by sweeping 12°–85° on a 0.5° grid — 147 projectile evaluations per
call. Owner's observation (2026-09-18): no search is needed. Horizontal velocity is `R / tof`, and
for a given release and target `tof` grows monotonically with apex height, so the softest landing is
simply the **highest throw the limits allow**: the steepest pitch satisfying `pitch_max`, the apex
cap and the speed cap.

## Discussion

**The claim is true in the far field and false near the yaw axis.** The release point travels
`l = 150 mm` with pitch. Beyond a few hundred mm that is a perturbation and the sweep's argmin was
always the steepest feasible grid pitch. Closer in, a flatter barrel parks the release point almost
over the target (`R → 0`) and wins. Owner's ruling: refuse those targets outright — no real target is
that close (the working target is ~1.2 m out at z ≈ 0).

**Withdrawn: "steepest must beat flattest" as the near-field guard.** It assumed horizontal velocity
is quasi-concave in pitch, so checking the two ends would suffice. The oracle probe found the optimum
at an *interior* pitch (20–28°) for A ≈ 110–400 mm. The guard is now a sufficient condition for the
claim itself — `d(v_h²)/dφ < 0` over the whole pitch range, each trig factor bounded by its maximum:
`R_lo² − K1·l·R_lo − K2·l·max(−z,0) − l²/2 > 0`, `R_lo = A + d − l·cos(pitch_min)`. One evaluation,
conservative: it refuses A < ~235 mm for a level target and < ~555 mm for a 1.5 m drop, against a
true boundary near 290 mm.

**Withdrawn: fixed-point iteration for the apex-cap pitch, and then a bracketed fallback built on
"apex rises with pitch".** The cap is measured above the *release* point, so it pins the vertical
launch speed at exactly `√(2·g·h_max)` and the pitch follows in closed form — but the release point
moves, so it is a fixed-point equation. Plain iteration contracts at ~l/R everywhere except within
millimetres of the cap, where `√disc` has unbounded slope; and there the moving release point makes
apex(φ) NON-monotone, opening a window of feasible pitches with two cap roots. Newton on the residual
`r(φ) = φ − atan2(vz·tof, R)` from `pitch_max` is the fix: `r` is convex on the descending-arrival
set, so Newton descends monotonically onto the *largest* root, and running out of domain or slope
first means there is none. 3–5 passes in the ordinary case.

**Two defects of the retired sweep surfaced by the oracle, fixed here.** (1) It returned
*fly-throughs*: a high, close target reached on the rising limb, where `predict_throw` (descending
crossing) disagrees about the landing. The steepest pitch has the longest flight, so if it arrives
rising nothing lands — refused. Unreachable inside the default envelope (asserted). (2) Directly
below the yaw axis it returned pitch −90°, outside the 12° pitch limit; that is a near-field target
and is now refused as one.

**Not bit-identical, by construction — owner chose the continuous optimum.** The result lies within
one grid step *above* the sweep's answer (≤ 0.5°, ≤ ~0.16 m/s faster at far range), never has a
larger horizontal velocity, and lands on the target to < 1e-6 mm through `predict_throw`. The visible
hardware change: the typical throw's apex is now exactly the 500 mm cap instead of wandering
470–500 mm with where the grid fell, so `tof` is a smooth function of the target. The aim-correction
table was fitted on grid solutions; the shift is small and smooth, but it has not been flown.

**The CPU saving is real and irrelevant.** 264.7 → 17.8 µs per call on the Jetson (typical target,
3000-call loop, 2026-09-18), but the solver runs once per throw plus 2 Hz during calibration
keep-alive. The value is exactness, the smooth `tof`, and the two sweep defects closed.

**Twin, not import.** `sim/` has no ROS-package dependency, so the core is copied into
`sim/ball_butler/sim.py`; `TestPitchCoreTwin` pins result-or-refusal bit for bit over 3000 samples.

## Benchmarks

Probe (uncommitted, scratchpad) against the retired sweep, 2026-09-18: 210 000 targets over seven
regimes (default, high targets, speed-bound, pitch_max 60°, pitch_max 45° wide, a second default
seed, apex knife-edge) — 0 disagreements, 0 limit violations, 0 targets lost other than near-field
refusals and 1 050 rising-arrival fly-throughs; 104 targets GAINED (feasible window narrower than a
grid step).

## Verification

Full tier (`./run_tests.sh --full`, run 2026-09-18 in the `bb-closed-form-pitch` worktree, ci-fast):
**PASS — parallel 6314 passed / 9 skipped / 2 xfailed in 383.10 s, serial 6 passed in 19.16 s, total
407 s, rc 0.** Scoped (`pytest tests/ros/test_throw_ballistics.py tests/sim/test_ball_butler_sim.py
tests/ros/test_ball_butler_node.py -q`, 2026-09-18): 77 passed. Not flown on hardware.
