---
title: "Kinematic calibration: the platform-pose error is model geometry, fixed by a parametric fit to a mocap sweep, not a pose servo. Design settled with the owner; fit tool proven on synthetic data"
type: feature
date: 2026-09-23
status: in-progress
phase: "kinematic-calibration — § 6 step 5 (applied)"
related_plan: kinematic-calibration.md
files_changed:
  - plans/active/kinematic-calibration.md
  - plans/active/INDEX.md
  - tools/kincal_fit.py
  - tests/sim/test_kincal_fit.py
  - config/hardware_config.yaml
  - config/tilt_calibration.yaml
  - config/compute_geometry.py
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - config/generated/geometry-config.js
  - config/generated/admissible_box.yaml
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py
  - sim/model/generate_mjcf.py
  - sim/model/jugglebot.xml
  - sim/hand/planner.py
  - sim/input/scripted.py
  - sim/input/toss_loop.py
  - sim/ball_butler/sim.py
  - tests/motion/test_skills_admissible.py
  - tests/ros/test_gui_geometry.py
  - tests/sim/test_model.py
  - tests/sim/test_planner.py
  - tests/hardware/session_kincal_apply.md
  - docs/motion_planner/kinematics.md
  - docs/motion_planner/architecture.md
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

## 2026-09-27: applied (§ 6 step 5)

The fitted geometry is now `config/hardware_config.yaml`'s `jugglebot_geometry`
(owner's acceptance above; procedure in the plan § 6 step 5). Nothing physical
changed and no firmware was flashed: the three firmware `hardware_config.h` copies
carry the new constants, but no firmware code reads them.

**What changed.** `base_nodes_mm` (x/y fitted, z held at 0), `init_plat_nodes_mm`
(fitted, z ±2.9 mm), `init_leg_lengths_mm` (the per-leg L0), `mm_to_rev` (fitted)
and `initial_height_mm` 574.3 → 578.2. `config/tilt_calibration.yaml` is `git rm`'d:
it was captured against the old geometry, so it encoded part of the error the fit
removes. An absent file is the documented C-LEVEL-1 fallback. (Attribution note: this
worktree's index is shared with the R4 session, and the staged deletion rode that
session's runsheet commit `1b187e8` — the R4 sheet's new row 8b, `level` first under
the new geometry — before the apply commit was written. The deletion is part of THIS
change; the SHA is recorded only so `git log -- config/tilt_calibration.yaml` makes
sense.) The generated
constants, the GUI copy and the MuJoCo model (`sim/model/jugglebot.xml`, a committed
generated artifact the plan did not list) were regenerated.

**Transcription check.** Under the applied YAML the IK returns the fitted STOW pose
((−3.6, −7.7, 578.2) mm, 0.88° about x) as leg extensions within ±0.7 mm of zero, so
the numbers went in as fitted. The IK's nominal STOW (centred, level, at 578.2 mm)
is now a different pose from motor zero: legs 5 and 6 read −5.1 / −4.9 mm there, legs
1–4 +0.6..+5.6 mm. From a 50 mm lift up every leg is positive (≥ 39.7 mm at 50, ≥
149.7 mm at the 170 mm ACTIVE lift). Homing puts the motors at 0 directly and every
planned pose sits at or above the session floor lift, so nothing commands that pose;
the one thing that did was `tests/sim/test_model.py`'s `home` FK case, moved to a
10 mm lift (every leg ≥ +3.8 mm). The YAML comment carries these numbers.

**Consumers swept.** Every `574.3` outside history: `sim/hand/planner.py`,
`sim/input/scripted.py` (three), `sim/input/toss_loop.py`, `sim/ball_butler/sim.py`
(the catch-height default) and `tests/sim/test_model.py` / `test_planner.py` now read
`hardware_config.GEOM_INITIAL_HEIGHT_MM`; the two `docs/motion_planner/` mentions say
578.2 (fitted). No consumer assumes platform-node z = 0: the IK/FK rotate the
3-vector, MuJoCo reads it, the GUI FK (`stewart-fk.js`) rotates it.
`config/compute_geometry.py` now warns that its verify mode reports a mismatch by
design and that `--update` would revert the calibration.

**Tests that asserted the CAD shape.** `tests/ros/test_gui_geometry.py`'s two
"nodes on the circle" tests (0.5 mm) cannot hold for calibrated nodes 1–6 mm inward;
they now assert "near the circle" (10 mm), which still catches a dropped digit or a
swapped row. `base_radius_mm` / `plat_radius_mm` stay as CAD nominals (Jacobian
normaliser, GUI/MuJoCo ring); the fit tool's "from CAD" prior and plausibility bound
read `nominal_geometry()` from the generated config, so a future re-fit measures
from *this* fit, not CAD — noted, not fixed (a CAD-nominal source would be
`compute_geometry.py`'s circles).

**Gate hash now covers the geometry (the R4 session's ask, § 7).**
`admissible.gate_hash()` hashed six planner files but not the IK geometry, so this
commit would have left every box "valid" while swept under the old IK — the re-sweep
depended on § 7 being remembered. `config/generated/hardware_config.py` (as the
package copy `jugglebot/hardware_config.py`) is the seventh gated file; the
parametrised test covers it and a new test pins the test's file list to the live one.
Hash `c2736ce2e2f7` → `f43f2f7b6742`.

**Box re-sweep.** `python tools/admissible_sweep.py --site-pairs all --single-apex 0.5 0.6
0.7 0.8 0.9`, run twice side by side on 2026-09-27 (`OPENBLAS_NUM_THREADS=1`), after
the tilt-centre and cup-base fixes below: 17 931 rows, 1 243 refused, **1 971.2 s and
1 973.2 s**; the two YAMLs are byte-identical, so the sweep is deterministic under the
new IK. Boxes 9 → 9, limits unchanged (300/5000/150k, hand 3500). **Bounds moved for the
first time since R2** (every R4 re-sweep was bit-identical; the IK changed this time).
The grid steps 2 mm inside ±10 mm and then jumps to ±20 (±30/40 for self-toss), so one
refused edge cell moves a bound by 10 mm:

| box | old | new |
|---|---|---|
| columns P1, 0.85–0.95 m | x [−20, 20], y [−20, 20], apex 0.90 only | x [−20, **10**], y [**−10**, 20], apex **0.85–0.90** |
| columns P2, 0.85–0.95 m | x [−20, 20], y [−20, 20], apex 0.90 only | x [−20, **10**], y [−20, **10**], apex **0.85–0.90** |
| hop P1→P2, 0.85–0.95 m | x [−20, +6], apex 0.85–0.90 | x [−20, **+2**], apex 0.85–**0.95** |
| hop P2→P1, 0.85–0.95 m | x [−6, +20], apex 0.85–0.90 | x [**−0.5**, +20], apex 0.85–**0.95** |
| self_toss P1, 0.55–0.65 m | [−40, 40] × [−30, 30] | [−40, **30**] × [**−40**, 30] |
| self_toss 0.45–0.55, 0.65–0.95 (×4) | — | unchanged |

**The hop boxes the first R4 sitting flies narrowed on x** (toward the target site) by
4–6 mm while their apex band widened; the R4 session was told. The columns and 0.6 m
self-toss boxes traded 10 mm of one lateral axis for a wider apex band — the
largest-box search picking a different aspect out of a reshaped admitted set. The
identity prior's own command (0, 0) is inside every box. New file:
`config/generated/admissible_box.yaml`, `swept_at: 2026-09-27`, `gate_hash:
a905116c2177`. An earlier pair, run before the cup-base fix, had shown only the 2 mm
hop widening reported to the R4 session first; that pair was discarded.

**What the full suite found (41 failures on the first `./run_tests.sh --full`,
2026-09-27) — the class the plan's grep could not see.** The plan's step 5.3 said to
grep every consumer of `574.3`. That misses every *derived* literal: a number equal
to `574.3 + something` that nobody wrote as a sum. Four were live code:

- `motion/trajectory/shaping.py` `CUP_TILT_CENTER_Z_MM = 744.3` (= 574.3 + the 170 mm
  lift), the lever the always-on lean shaper, `cup_realize`, `unified_cycle` and
  `toss_release` all swing the cup about. Left alone it would have mis-levered every
  cup by 3.9 mm in z on the machine — the sim showed it exactly: the cup opening
  settled 3.9 mm above its target in `test_realize_tilted_lands_opening_on_target`.
  Now `GEOM_INITIAL_HEIGHT_MM + JB_OP_DEFAULT_ACTIVE_Z_MM` (748.2), which also puts it
  under the box gate hash through the geometry file. `LEAN_CUP_Z_MM` (839.4) is the
  same centroid plus the characterised 95.1 mm arm and is now written that way, so
  the lean shaper's arm is unchanged by the calibration.
- `motion/trajectory/cup_realize.py` `CUP_Z_BASE_MM = 659.6` (= 744.3 − 84.7), the
  cup's world height at slider zero that every cup-z → slider conversion uses: the
  literal would have set the slider 3.9 mm low for any commanded cup height. The
  84.7 mm cup-below-centroid distance is the measured morphology; the world value
  now rides on the tilt centre (663.5). The same literal sat in `sim/juggle_tilt.py`,
  `sim/gate_common.py`, `sim/juggle_throw.py` and `sim/juggle_selfcatch.py` (the sim
  mirrors by design; the units-trap and parity tests pin them) and in three
  `tools/probes/juggle_*` scripts, which are left as they were with this note.
- `sim/ball_butler/sim.py`'s catch-height default 783.5 (= 574.3 + 80 + 129.2).

Found in two passes: the tilt centre first, then — with the eight cup-realise sim
cases still 3.9 mm high — the cup base. Each pass invalidated the running sweep
(`cup_realize.py` is a gated file), so the box was swept three times; the pair
reported above is the last. `tests/motion/test_unified_cycle.py`'s lever-residual
characterisation (1.349379 mm at a 115.7 mm arm) re-pins to 1.303894 mm at the
111.8 mm arm the new centre gives; the ratio is exact.

The rest of the 41 were pinned worked examples: `test_toss_release.py` (ten literals
744.3 / 809.08 / 802.344…, all +3.9 mm), the GUI FK golden file (regenerated with
`tools/gen_gui_fk_golden.py`), the ball-possession z bound (305.03 → 308.93), the
ODrive reflash blurb (8 rev of leg 0 = 568 mm, was 564), the sim plant's "zero
extensions at home" (now the per-leg `geometric_home − L0` offset, because the sim's
home is the level pose), a follower stop pose and a capture-reach pose that had 0.1 mm
of stroke to spare, and the kincal synthetic-fit threshold. Two files were
characterisations recorded under the CAD geometry and would have been silently
re-characterised: `tests/motion/test_fk_convergence.py` (recorded hardware extension
vectors, an empirically found Newton recipe, and `−648.419` as "zero absolute
length") and the v5 wire-byte capture in `test_trajectory_emitter.py` (bytes encode
`mm × mm_to_rev`; its capture script says never regenerate). `tests/motion/
_cad_geometry.py` freezes the retired constants for exactly those two, and the FK
file gains live-geometry sweep coverage so it still proves the shipped solver
converges on the machine.

**The sim's cup heights were slider positions in disguise.** `sim/juggle_selfcatch.py`
and `sim/juggle_throw.py` chose their release/catch/aim heights (0.85 / 0.84 / 0.70 m)
against the 659.6 mm cup base; kept as world literals under the new base they moved
every contact 3.9 mm down the slider and re-rolled the nightly self-catch
characterisations (a CasADi `Restoration_Failed` on seed 2, the pose-chaos gain
falling to 0.5). Written as `CUP_Z_BASE_MM + offset` they keep their slider
positions and the characterisations return, except the column reach-amplification
mode, which moved from seed 3 to seed 4 (6.45 → 110.7 → 212.1 mm; every seed still
diverges) — its docstring already says the mode follows the catch contact, and it
moved the same way on 2026-08-21. `build_cup_config`'s slider band re-pins 0.690 /
0.6796 → 0.6935 / 0.6835 for the same 3.9 mm. One sim pin was re-set on evidence
rather than restored: the vertical column toss now lands 22.7 mm off (was 8.3), with
the platform level and still through the contact window (< 0.01°, < 0.5 mm/s) and
the cup velocity vertical at release — the ball rolling in the cup, the documented
pose-chaos, so `test_juggle_throw`'s bound is 30 mm (still deep inside the catch's
60–80 mm reach).

⚠ **OPEN — firmware stroke backstop.** `tests/firmware/test_hermite_xref.py::
test_firmware_stroke_bounds_match_motor_guard` is now `xfail(strict=True)`:
`Teensy_code_canbridge/canbridge_config.h` `STROKE_MIN/MAX_REV` were captured
2026-06-01 from motor_guard under the old `mm_to_rev`, so the firmware's per-leg rev
bounds now sit ~0.7 % (about 2 mm at the top of stroke) from what the calibrated
scales give. Physically nothing changed — the bounds were always this far from the
true stroke in mm — but bringing the firmware in line is a firmware edit + flash
(the plan's "no firmware flash" held only for the generated header, which the
firmware does not read). Owner's call; not scheduled here.

`test_the_qp_and_the_gate_stay_within_an_order_of_magnitude` failed once at 16.8×
while two sweeps had the CPU; it is a wall-clock ratio and passed on the re-run.

**Sweep tooling note.** Two concurrent sweeps with default BLAS threading thrashed
(load average 18 on 6 cores, ~0.2 cells/s each); with `OPENBLAS_NUM_THREADS=1`
`OMP_NUM_THREADS=1` they ran at ~19 cells/s each side by side. The live nodes already
pin `blas threads: 1` for the same reason; the sweep tool does not. A second
run was lost to a docstring edit in `unified_cycle.py` (a gated file) while it
ran: each box records the gate hash at write time and the file refuses a mix.
Both rules are now in the tool's docstring.

**Verification.** `./run_tests.sh --full` (2026-09-27, the final tree, launch down):
**5471 passed + 6 serial, 8 skipped, 2 xfailed in 298 s, RESULT: PASS** — the first run
of the day on the same tree had 41 failures (all above). The narrative edits made
after that run are covered by `pytest tests/sim/test_logbook_front_matter.py
tests/sim/test_logbook_search.py tests/sim/test_plans_index.py -q` (2026-09-27):
**109 passed in 0.77 s**. `colcon build --packages-select jugglebot` done in this
worktree; the stale `share/jugglebot/config/tilt_calibration.yaml` removed by hand
(colcon does not remove a file the source tree stopped installing).

**Next sitting** (`tests/hardware/session_kincal_apply.md`): `level` FIRST — the
persisted inclinometer offset (0.80° about x) was measured against the old IK and
tilts every commanded pose by ~0.8° until re-measured; expect it to shrink to the
fit's ~0.2° attitude floor. Then the tilt-map recapture (§ 6 step 6, expected much
smaller), then the flying frame check at every z (§ 6 step 7, the § 8 acceptance).

