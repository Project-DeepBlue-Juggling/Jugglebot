---
title: Ball Butler hand offset s flips to +105.65 mm and the aim correction is refitted for it (validated on hardware; merge and build pending)
type: bugfix
date: 2026-10-09
status: in-progress
phase: "two-ball-skill-stack — R5 (Ball Butler calibration)"
related_plan: two-ball-skill-stack.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - config/generated/geometry-config.js
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/gui/js/geometry-config.js
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/src/jugglebot/resources/throw_affine_correction.json
  - ros_ws/src/jugglebot/resources/throw_affine_correction_2026-06-09_negative_s.json
  - tests/ros/test_ball_butler_node.py
  - tests/ros/test_throw_ballistics.py
  - tests/ros/test_perception_console_lines.py
  - tests/ros/test_gui_robot_assets.py
  - tests/sim/test_ball_butler_sim.py
  - tests/sim/test_juggle_catch.py
  - sim/juggle_catch.py
  - sim/juggle_bb_catch.py
  - config/generated/admissible_box.yaml
  - logbook/2026-10-09-bb-positive-s-and-refitted-aim-correction.md
  - logbook/INDEX.md
subsystem:
  - config
  - ros
tags:
  - kinematics
  - testing
---

# Ball Butler hand offset s flips to +105.65 mm and the aim correction is refitted for it (validated on hardware; merge and build pending)

## Problem

The aim model put Ball Butler's hand on the wrong side of its yaw axis. `bb_release_state` placed it to the RIGHT of the throw, while the hardware and the CAD have it on the LEFT (the GUI entry 2026-10-05 raised this as an open `s`-sign question). The 2026-06-09 affine and the columns feed bias (`columns_feed_bb_bias_mm`, measured at about (+26, +27) mm) were partly compensating for that wrong geometry.

## Root Cause

`ball_butler_geometry.yaw_s_offset_mm` was −105.65 mm. The magnitude came from Onshape; the sign was wrong. The BallButler repo's calibration campaign flew with `s = +105.65 mm` and the old affine bypassed. The record is in BallButler `logbook/2026-10-09-bb-local-calibration-result.md`.

- **Campaign:** 277 throws; 275 measured from the raw MCAP after BB's reinstall.
- **Uncorrected error:** 54.4 mm RMS per throw, bias (+44, +9) mm.
- **What remains is affine:** an 8 % radial gain, stable for an hour.
- **Expected after the refitted affine:** about 21.8 mm per throw over the grid and 21.3 mm in the core, judged on held-out cells. That equals the throw-to-throw repeatability (20.7 mm).

## Fix

All changes are on branch `bb-positive-s-affine-2026-10-09`, from `skill-stack`, in worktree `~/Desktop/Jugglebot-bbcal`. Nothing is built, flashed or deployed yet (see Outcome).

- **`s` flipped.** `yaw_s_offset_mm: −105.65 → +105.65`.
  - Regenerated with `generate_config.py --no-external`. Only the `s` lines changed, across 9 files.
  - `../BallButler` was not written; its working tree carries the owner's uncommitted header edits.
  - No firmware reads `YAW_S_OFFSET_MM` (Teensy platform, cone, can-bridge, BB: only the generated headers define it), so no reflash is needed.
  - Consumers are `throw_ballistics` (the solver), `sim/ball_butler`, and the GUI's BB model, whose hand moves to the correct side.
  - BB's pose calibration does not use `s`.
- **Affine replaced.**
  - `resources/throw_affine_correction.json` is now the 2026-10-09 refit: desired → commanded BB-local XY, applied as-is, `requires_corrected_positive_s: true`, valid for BB-local x 386–1570 and y 195–928 mm.
  - Its provenance records the hardware validation and the owner's acceptance (Verification below): `validated_on_hardware: true`, session `20261009T031319_612936Z`.
  - The 2026-06-09 matrix is kept as `throw_affine_correction_2026-06-09_negative_s.json`. Never stack the two.
- **Pairing guard.** `ball_butler_node._load_aim_correction` refuses a matrix fitted for the other sign of `s`, so the node throws uncorrected with a loud ERROR. A half-deployed pair would otherwise aim wrong with no error.
- **Feed bias.** `columns_feed_bb_bias_mm` stays at its default (0, 0). The new matrix corrects the error it compensated for. Re-measure it with `tools/probes/feed_lateral_miss.py` only if feeds still miss.
- **Admissible box re-swept (S8).** `hardware_config.py` is one of `admissible.gate_hash`'s eight gated files, so the one-number edit moved the live gate from `f9e081deeaea` to `1cd76c2d3c4a` and `skill_node` refused every pattern (25 of the gate's failures). Re-swept with S7's grids and arguments (`tools/admissible_sweep.py --dwell-s 0.27 --leg-vel 350 --leg-acc 5000 --leg-jerk 200000 --hand-acc 3900`, columns / hop / single in parallel at one BLAS thread, `temp/sweeps/sweep_S8.sh`, logs `temp/logs/admissible_sweep_S8_*_20261009.log`), merged single + hop + columns. `s` does not enter the platform's planning, so the expectation was a content-identical box under the new hash; see Verification.
- **BB-feed sims throw from the measured placement.** `sim/juggle_catch.py` / `sim/juggle_bb_catch.py` threw from an invented demo placement, (−872, −630, 1430) mm with the local +x axis aimed at the origin, so the feed target sat dead ahead of the yaw axis; under positive `s` that needs yaw −5.6° and the solver refuses it (the other 7 gate failures). They now throw from the 2026-10-09 mocap pose, `BB_PLACEMENT_MM` (−975.6, −389.3, 1734.9) mm and yaw offset 0.208°: the origin is at BB-local (977, 386) mm, bearing 21.6°, inside the calibrated region, solved at yaw 15.8°. The feed arrives at ~5.5 m/s, 12° from vertical (was ~4.9 m/s, ~15°), and the catch still seats. `SingleCatchConfig.bb_yaw_offset_rad=None` now aims the hand's throw line (not the local +x axis) at the origin.
- **Tests.** 35 tests failed after the flip, all through physics rather than pinned values. The fixtures aimed straight down BB-local +x (y = 0–50 mm), which now needs a negative yaw.
  - Targets moved to local y = +400 mm (the calibration's targets span 195–928 mm).
  - The sim fixture's heading is skewed 30° so the target is on the hand side.
  - The old-sign pins were updated.
  - Pairing and shipped-file tests were added.

## Verification

**Hardware: the corrected validation run** (BallButler `LOCAL_CALIBRATION.md` § "Validating a candidate correction on hardware"; BallButler `logbook/2026-10-09-bb-local-calibration-result.md` § Outcome has the full analysis). Session `20261009T031319_612936Z`, 2026-10-09: the BallButler runner mapped 110 desired targets (seed-1042 plan shifted half a grid step, 95 cells, 30 core throws) through this matrix and solved them with `s = +105.65`, exactly as the node will; all 110 released and were measured.

| quantity | measured | pre-registered criterion |
|---|---|---|
| mean error (BB-local, measured − desired) | (+1.6, −8.2) mm | within ±6 mm per axis — **fails on y** |
| per-throw RMS, grid | 21.0 mm | ≤ 26 (predicted 21.8) — passes |
| per-throw RMS, core | 20.9 mm | ≤ 25 (predicted 21.3) — passes |

The verdict is FAIL on the lateral mean alone. That mean is consistent in sign and size with the two sessions' BB pose calibrations differing by 0.47° in yaw offset (0.208° vs 0.681°, each quoting σ ≤ 0.07°): a +0.47° frame rotation moves a landing at the 0.97 m mean range by −8.0 mm in local y, and in the calibration session's frame the mean is (−2.9, −1.6) mm. **The owner accepted the candidate on 2026-10-09** as a documented deviation from the pre-registered PASS requirement: the RMS criteria pass at the predicted repeatability floor, and a per-session pose-calibration offset is not something a matrix fitted in one session can remove. Follow-up, outside this entry: the yaw-offset repeatability of `bb_calibration` (one anchor marker's angle about the fitted axis point; the two axis points differ by 1.7 mm).

**Offline, on the branch:**

- BB-feed sims after the placement move (2026-10-09, `python -m pytest tests/sim/test_juggle_bb_catch.py tests/sim/test_juggle_catch.py -q -p no:cacheprovider -n 2 --dist loadfile`): **14 passed in 42.49 s** (was 7 failed on `Yaw -5.6° outside limits`).
- The touched files before the sweep: 254 passed, 1 skipped, 2 xfailed. The pre-change baseline on `skill-stack` was 252 passed.
- `generate_config.py --check` is clean.
- **Admissible re-sweep S8** (2026-10-09 14:56–15:18, `temp/sweeps/sweep_S8.sh`: columns 12.4 min, hop 20.4 min, single 22.1 min, all rc 0; logs `temp/logs/admissible_sweep_S8_{columns,hop,single}_20261009.log`, driver `temp/logs/sweep_S8_driver.log`): merged single + hop + columns into `config/generated/admissible_box.yaml` (19 boxes, gate `1cd76c2d3c4a`, limits 350/5000/200000, hand 3900, dwell 0.27, md5 `a1d4fe60f360e22a5627c7ea6db4e409`). **Every box is content-identical to the committed S7 box** (`4757e6b9`, gate `f9e081deeaea`); only `swept_at` and `gate_hash` differ, as expected for a change that does not enter the platform's planning.
- Full gate `./run_tests.sh --full` on the branch: the (date, command, result) triple is in the commit message (`git log --grep "Logbook-Entry: 2026-10-09-bb-positive-s-and-refitted-aim-correction"`).

**Found here, fixed in a sibling entry: the yaw-root choice.** `yaw_solve_thetas` (and its sim twin) returned the root with the smaller |yaw|; `t2 = base − π + δ` always has the target behind the release point, so every target behind the yaw-axis plane took the wrong root, and under positive `s` (−2000, 0, 0) "solved" at yaw 3°. Every target in the calibrated region takes `t1`, so no calibration command is affected. Recorded here as two strict xfails; fixed in `2026-10-09-bb-yaw-root-fix.md` (its own commit, since it changes the solver's sha).

## Outcome

Committed on `bb-positive-s-affine-2026-10-09` and pushed; **not merged, built or deployed**. To deploy:

1. Merge the branch into `skill-stack` (`~/Desktop/Jugglebot-skills`), `colcon build` the workspace.
2. Start the stack; check the node logs `Aim correction source: …/throw_affine_correction.json` with no pairing ERROR, and that `skill_node` accepts the re-swept box (gate `1cd76c2d3c4a`).
3. Throw a handful of targets on the hand side (BB-local y ≳ 200 mm) before a pattern.
4. Re-measure `columns_feed_bb_bias_mm` with `tools/probes/feed_lateral_miss.py` only if feeds still miss; it stays (0, 0) until then.
