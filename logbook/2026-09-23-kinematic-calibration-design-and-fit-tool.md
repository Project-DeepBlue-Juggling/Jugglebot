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
