---
title: Ball Butler yaw solve always takes the positive-range root (targets behind the yaw-axis plane were aimed the wrong way or refused)
type: bugfix
date: 2026-10-09
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/can/throw_ballistics.py
  - sim/ball_butler/sim.py
  - tests/ros/test_throw_ballistics.py
  - tests/sim/test_ball_butler_sim.py
  - logbook/2026-10-09-bb-yaw-root-fix.md
  - logbook/INDEX.md
subsystem:
  - ros
  - sim
tags:
  - kinematics
  - testing
---

# Ball Butler yaw solve always takes the positive-range root (targets behind the yaw-axis plane were aimed the wrong way or refused)

## Problem

Found while flipping the hand offset `s` to +105.65 mm (`2026-10-09-bb-positive-s-and-refitted-aim-correction.md`): with the correct sign of `s`, a target 2 m directly behind Ball Butler, BB-local (−2000, 0, 0), "solved" at yaw 3° with no error. The ball would have flown forward, away from the target. The same rule had been refusing targets BB can physically reach by yawing round (bearing 90–180° on the hand side, within the hardware's [0°, 185°] yaw range).

## Root Cause

`throw_ballistics.yaw_solve_thetas` solves `x = r·cosθ − s·sinθ, y = r·sinθ + s·cosθ`. Writing `x + iy = (r + is)·e^{iθ}` gives `θ = atan2(y, x) − atan2(s, r)`. With the physical `r > 0` that is one root, `t1 = base − asin(s/hyp)`. The second root, `t2 = base − π + asin(s/hyp)`, is the same equation with `r < 0`: the target behind the release point along the throw line. It is never a throw.

The function returned whichever root had the smaller |yaw|. For every target in front of the yaw-axis plane (|bearing| < 90°) that is `t1`; for every target behind it, `t2`. What then happened depended on the sign of `s` and the yaw limits:

- under `s = −105.65` (the old, wrong sign) `t2` fell below 0° for the hand-side quadrant and was refused, which hid the bug there and refused reachable targets; for the far-side-behind quadrant `t2` wrapped into range and aimed forward;
- under `s = +105.65` the quadrants swap: (−2000, 0) → 3°, accepted.

The sim twin `sim/ball_butler/sim.py::_yaw_solve` had the same rule.

## Fix

`chosen` is always `t1` in both; `t2` is still returned for diagnostics, with the derivation in the docstring. Consequences:

- **No change for any target in front of the yaw-axis plane.** The whole calibrated region (BB-local x 386–1570, y 195–928 mm; bearings 7–67°) and every feed target took `t1` already. Pinned by `test_forward_root_is_what_the_old_rule_chose_in_front_of_the_yaw_axis_plane`, and checked end to end by the BallButler runner's new solver-equivalence check, which re-solves the 277 recorded calibration throws with the new file and demands identical solutions (BallButler `run_local_calibration.solver_reproduces_fit`).
- Targets behind the yaw-axis plane on the hand side are now solved at 90–185°; e.g. (−100, 1000) at 89.7°.
- Targets behind on the far side are refused ("out of BB range") instead of being aimed forward; e.g. (−1000, −1000).
- The solver file's sha256 changes (`bbb80fa5…` → see Verification). The BallButler runner's `--expect-solver-sha` guard and its candidate-vs-solver guard both refer to the file hash; the latter now falls back to the behavioural check above when the hash differs, so the 2026-10-09 candidate remains usable with the rebuilt stack.

## Verification

- Solver tests (2026-10-09, `python -m pytest tests/ros/test_throw_ballistics.py tests/sim/test_ball_butler_sim.py -q -p no:cacheprovider`): **52 passed in 8.27 s**. The two strict xfails that recorded the bug are now ordinary passing tests, with the root reconstruction, the in-front equivalence, the reachable hand-side-behind target and the refused far-side-behind target added.
- BallButler runner equivalence check (2026-10-09, `run_local_calibration.solver_reproduces_fit` with this file's `solve_throw_local`): all **277** released throws of fit session `20261009T002142_931068Z` and all **110** corrected commands of validation session `20261009T031319_612936Z` re-solve to the recorded yaw/pitch/speed/tof within 1e-9. Spot checks: (−2000, 0) → 176.97°, (−100, 1000) → 89.68°, (−1000, −1000) refused (−139.3°). This file's sha256 is `cb09095e2e9db0b8ff38d68fc1da215457966383be0fb39def541d1626c9c05f`.
- Full gate `./run_tests.sh --full`: the (date, command, result) triple is in the commit message (`git log --grep "Logbook-Entry: 2026-10-09-bb-yaw-root-fix"`).
