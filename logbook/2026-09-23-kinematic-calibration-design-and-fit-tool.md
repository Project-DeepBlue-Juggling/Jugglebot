---
title: "Kinematic calibration: the platform-pose error is model geometry, fixed by a parametric fit to a mocap sweep, not a pose servo. Design settled with the owner; fit tool proven on synthetic data"
type: feature
date: 2026-09-23
status: in-progress
phase: "kinematic-calibration — § 6 step 1"
related_plan: kinematic-calibration.md
files_changed:
  - plans/active/kinematic-calibration.md
  - plans/active/INDEX.md
  - tools/kincal_fit.py
  - tests/sim/test_kincal_fit.py
subsystem:
  - motion
  - config
tags:
  - kinematics
  - testing
---

# Kinematic calibration: design and fit tool

## Summary

The owner asked how to bring the Platform back to its target pose. Today a
session-start offset is subtracted from tracker landings, which "feels sketchy", and
the owner leaned toward a dedicated pose-servo controller. A grilling session settled
the design instead: `plans/active/kinematic-calibration.md`, whose § 9 is the full
decision record. Fit the Stewart geometry (51 parameters) to a mocap pose sweep. Then
the IK is right everywhere, and the landing subtraction retires once a flying sitting
reads ≤ 1–2 mm. Step 1, the offline fit tool, is built and proven on synthetic data.

## Discussion

**Hypothesis reframed mid-session: "frame artefact" → "model geometry".** The
2026-09-23 entries explained the 8.5 mm offset as a tilt between the QTM frame and the
base plane. The owner corrected the premise: the QTM global frame *is* the Base body,
which is defined from markers in CNC'd holes. So a height-proportional offset in that
frame is the platform really being elsewhere. The mocap origin sitting 7 mm above the
commanded centre fits the same reading: that origin is trusted, placed at the
leg-joint centroid with a jig. The likely cause is geometry error in a hand-built FDM
machine.

**Why not the pose servo (the owner's opening idea).** It would correct a static
geometric error with a dynamic loop. A mocap loop around the 40 Hz leg path cannot act
inside a throw stroke. It would also add a mocap-dependent feedback path (marker swaps,
occlusion) to the leg command, where the Teensy `MAX_DEVIATION` guard is the only safety
authority. The design keeps the servo as a pre-registered fallback: it returns only if
the two-direction repeat test reads RANDOM (plan § 3).

**Why not a residual map (the pattern of the existing tilt map).** A 3-D-plus workspace
needs hundreds of nodes. Interpolation kinks in the map's gradient become leg jerk, and
`LIMIT_JERK` is already the dominant refusal. A correction applied outside the IK also
means the gate checks legs other than the ones commanded.

**What the synthetic proof changed.** The registration tilt is near-degenerate with the
six L0, and each L0 with its base node's z, over a stroke-limited sweep. The fit
therefore usually freezes the tilt at zero, and pose-space accuracy is unaffected. This
departs from the owner's Q14 decision (zero attitude = joint pattern) in tilt only; it
is benign because `level` absorbs a constant attitude offset. It has been surfaced to
the owner. The capture now tilts every pose (plan § 5.1).

**Two tool-side lessons.**
- `pinv(JᵀJ)` blew up every posterior sd, because the k columns are scaled around 1e4
  and the gauge rows are weighted ×1e3; the covariance now uses the SVD of the
  column-scaled Jacobian.
- A multi-threaded OpenBLAS on the loaded Jetson took a 400×51 SVD from 20 ms to 4 s,
  so the CLI pins one thread, as `run_tests.sh` already does.

## Verification

- `pytest tests/sim/test_kincal_fit.py -q` (2026-09-23): **18 passed in 3.68 s**. The
  tests cover: agreement with the production IK; hold-out pose error within the § 8
  criteria (0.15 mm RMS against 10.6 mm under the config geometry); honest posterior sd;
  the § 3 and § 4 verdicts; the offsets-only re-fit; CSV and CLI round trips.
- Gate: `./run_tests.sh` (2026-09-23 21:39, log `temp/logs/kincal_gate_2139.log`):
  **6396 passed, 22 failed, 9 skipped in 747.95 s.** Every failure belongs to the
  skill-stack R4 session's *uncommitted* work on the same working tree:
  - `tests/ros/test_skill_node.py` ×18
  - `test_skills_gate.py` ×2
  - `test_skills_plan_bench.py` ×1
  - `test_unified_cycle_integration.py` ×1

  All of them are refused with "PRE-R4 admissible box file … Regenerate", pending R4's
  own box re-sweep. None touches a file in this entry. `test_kincal_fit.py`,
  `test_plans_index.py` and `test_logbook_front_matter.py` all passed within that run.

## 2026-09-27: the first real sweep, and two changes to the fit

**Changes (owner-agreed).** The six base-joint heights are held at CAD
(`HELD_PARAMS`). Each is near-degenerate with its leg's L0, and the owner judges
the machined base far closer to CAD than the hand-built legs. The freeze decision
now reads the posterior sd at `sigma_mm`. A quiet fit (χ² < 1) can scale that sd
down, but a misfit can no longer scale it up.

**Why the freeze rule changed.** On the preview capture (97 rows,
`kincal_sweep_20260927_131410`), the old rule scaled sd by √χ², with χ² = 27. That
froze 22 parameters, all six L0 included, and freezing them made the misfit worse:
hold-out 4.4 mm RMS. With nothing frozen, the same rows fit to 0.37 mm per leg.
Identifiability is a property of the sweep and the noise, not of the misfit. An
unconditional "sd at σ" rule was tried first and rejected: the synthetic sweep is
quieter than σ (χ² 0.13), and that rule froze its L0 values, which are well pinned.

**Results on `kincal_sweep_20260927_143217`** (166 rows: 148 fit, 10 hold-out,
8 post-home). Report: `temp/reports/kincal/kincal_sweep_20260927_143217/report.md`.
- Leg residual 0.376 mm RMS, χ² 1.64. Only `reg.x` and `reg.y` froze.
- Hold-out position: **1.076 mm RMS** and 1.773 mm max, against 12.27 mm and
  17.74 mm under the config geometry. Fit rows: 0.93 mm RMS, so the fit is not
  overfitted.
- Hold-out attitude: max **0.198°**. Its mean is about 0.003°, so this is not a
  constant offset that `level` would absorb.
- § 3 **DIRECTIONAL**. R02, R03 and R06 have an overall spread of 1.2–1.5 mm, with
  the within-direction spread ≤ 0.8 mm; the other groups are STATIC. Mocap attitude
  repeats to 0.03–0.13° within a group.
- § 4 **PASS**: 0.73 mm; per-leg ΔL0 up to 0.51 mm.
- § 8: position max passes. Position RMS fails narrowly (1.08 against ≤ 1). Attitude
  fails (0.20° against ≤ 0.1°). Node plausibility fails (6.4 mm against ≤ 5).
- Fitted L0: legs 4 and 5 are +7.1 and +4.5 mm; the others are within ±2.7 mm, all
  with sd about 0.7 mm.
- Fitted k: five legs are 0.7–1.3 % low and leg 2 is 0.5 % high, with sd about 0.07 %.
- **Every node shifts radially inward.** Base: −3.5 to −6.1 mm, mean −4.1 mm (about
  −1.0 % of the 410 mm radius). Platform: −0.6 to −3.7 mm, mean −2.0 mm (about
  −0.9 %). A mocap scale error cannot produce this, because it would move k in the
  opposite direction from the nodes. Whether the joint centres really sit inboard of
  the CAD nodes is for the owner to judge.
- STOW under the fitted geometry is (−3.6, −7.7, 578.2) mm and tilts 0.88° about x.
  For comparison, the inclinometer's levelling offset read at that sitting's cold
  start was (0.0140, 0.0010) rad, which is 0.80° about x.

**Reading.** The residual sits at the machine's own repeatability. Directional
spread is up to 1.5 mm and attitude repeats only to 0.13°. The criteria below that
floor (1 mm RMS, 0.1°) cannot be met by any static geometry, whatever the fit. The
decision to apply is the owner's, and so is the § 3 consequence (a fixed final
approach direction).

- `pytest tests/sim/test_kincal_fit.py tests/sim/test_kincal_capture.py -q`
  (2026-09-27): **61 passed in 17.54 s**.
