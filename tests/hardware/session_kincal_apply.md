# Kinematic-calibration apply — first sitting on the fitted geometry

`plans/active/kinematic-calibration.md` § 6 steps 5–7. The IK geometry changed
on 2026-09-27 (logbook `2026-09-23-kinematic-calibration-design-and-fit-tool.md`
§ "2026-09-27: applied"). **Nothing physical changed; the only firmware flash is the
can-bridge stroke-clamp regeneration (FW 24, item 6 below).** This sitting re-levels the machine under the new geometry, recaptures
the tilt map, and runs the flying frame check that is the plan's § 8 acceptance
test. The R3/R4 sheets (`session_skills_r3.md`, `session_skills_r4.md`) stay the
reference for everything this one does not restate (QTM preconditions, the
launch rows, the guard/`/recover` flow).

**What changed, in five lines.**
1. `hardware_config.yaml`'s nodes, per-leg zero lengths (`init_leg_lengths_mm`)
   and per-leg `mm_to_rev` are the fitted values; `initial_height_mm` is 578.2
   (was 574.3). The GUI's displayed platform height moves by the same 3.9 mm.
2. The machine's real 0-rev pose is now modelled as what it is: (−3.6, −7.7) mm
   off-axis and tilted 0.88° about x. The IK's "STOW" (centred, level) is a
   pose the legs cannot quite reach (legs 5/6 at −5 mm). Nothing commands it.
3. **The persisted inclinometer offset is stale.** It was measured against the
   old IK and absorbed its tilt error (0.80° about x). It is re-pushed at every
   boot and re-measured only by `level`. Until `level` runs, every commanded
   pose is tilted ~0.8°. Hence rung A.
4. `config/tilt_calibration.yaml` is retired (deleted from the tree). It
   encoded part of the same error. The trajectory node falls back to
   offset-only levelling (C-LEVEL-1) when no file resolves — **but a stale copy
   in the colcon install tree would still load** (row 4 below).
5. The admissible boxes were re-swept under the new IK and the gate hash now
   covers the geometry file, so a box swept under the old IK refuses at accept.
6. The can-bridge runs **FW 24** (flashed 2026-09-27, identity read back): its
   per-leg stroke clamp `STROKE_MIN/MAX_REV` is now generated from the YAML
   (`leg_hard_margin_mm` × the fitted `mm_to_rev`) instead of the 2026-06-01
   hand-captured table. No wire change (protocol 9). Expect the boot banner /
   GUI identity to show FW 24; a 23 means the wrong board or a failed flash.

---

## 0. Roles & safety framing

Operator runs every MOTION command. Claude reads logs by path
(`temp/logs/…`), never by paste. **If your physical intuition disagrees with
any framing here — the direction the platform leans after `level`, a leg that
sounds wrong at a pose it used to reach quietly — that is load-bearing signal:
say so.** Two things are genuinely new to the plant's control path and deserve
a hand on the E-stop the first time they run:

- the first `level` (rung A): the platform will settle at a *different*
  attitude from every previous sitting — by design, roughly 0.8° about x.
- the first move to the ACTIVE pose after `level` (rung A row A5): under the
  new per-leg `mm_to_rev` and L0, the same commanded pose sends every leg
  slightly different revolutions than before (0.7–1.3 % scale, ±7 mm zero).
  The trajectory is profiled as always; the *destination* is what moved.

Every gate reports every refusal at once — fix each named one, never
re-dispatch around a refusal by hand.

## 1. Preconditions (loaded Jetson, launch DOWN)

| # | Do | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git status -sb` | Clean, at or after the commit that landed this sheet (`git log --oneline -1 -- tests/hardware/session_kincal_apply.md`). |
| 2 | `grep -n "initial_height_mm:" config/hardware_config.yaml` | `578.2`. |
| 3 | (ROS) `cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash && cd ..` | Builds clean. The geometry reaches `trajectory_node` through the installed `hardware_config.py`; a stale build runs the OLD geometry with the NEW box and refuses at accept (hash mismatch) — which is the gate doing its job, not a reason to bypass it. |
| 4 | `ls ros_ws/install/jugglebot/share/jugglebot/config/` | **`tilt_calibration.yaml` must be ABSENT.** colcon does not remove a file the source tree stopped installing. If present: `rm ros_ws/install/jugglebot/share/jugglebot/config/tilt_calibration.yaml` and re-run row 3. (`setup.py`'s own comment documents this trap; `tilt_cal_grid.py --force-uninstall` moves every candidate aside too.) |
| 5 | `python3 -c "import os,sys; sys.path.insert(0,'ros_ws/src/jugglebot'); from jugglebot.motion import tilt_map as t; print(t.resolve_tilt_map_path())"` | `None`. Anything else names a file that would load. |
| 6 | (venv) `./run_tests.sh --full` | Green (plan § 0 rigor: a dress rehearsal on the loaded Jetson before any hardware sitting). |
| 7 | QTM: `Platform` and `Base` bodies tracked; cone body disabled; BB reflectors masked (R4 sheet row 10). | The frame check and `kincal_capture.py --check` both read the Platform body. |
| 8 | Read the retired map once, for the comparison in rung B: `git show 1b187e8^:config/tilt_calibration.yaml \| head -40` (the deletion rode the R4 runsheet commit `1b187e8`, so its parent still has the file) | Note its `level_offset_rad` and the node residual magnitudes. |

## 2. Rung A — `level` FIRST

Bring-up as the R4 sheet rows 11–16 (load capture, launch with
`record:=true`, `BLAS 1 thread` on both up lines, GUI up), then:

| # | Do | Expect |
|---|---|---|
| A1 | `grep -n "tilt map" temp/logs/launch_kincal_*.log` | `tilt map: none found — single gravity offset only` from the trajectory node — offset-only levelling (C-LEVEL-1). A `tilt map loaded: <version>, grid …` line means row 4 was skipped — stop, fix, relaunch. (Before 2026-09-30 these read `no tilt calibration map found (tried: …)` / `tilt calibration loaded: …`; the long forms are now DEBUG, in launch.log.) |
| A2 | Note the boot-pushed offset: `ros2 topic echo /robot_state --once \| grep -A3 pose_offset` (or the GUI state panel). | The OLD persisted value, about (0.0140, 0.0010) rad = 0.80° about x. Record it. |
| A3 | GUI: **Home**, then **Activate**. **Do NOT move the platform anywhere else first.** | Robot at the active pose, hand parked at 0 rev. The platform may visibly sit ~0.8° off level here — that is the stale offset, expected. |
| A4 | GUI: **Level** (publishes `level` on `orchestrator_command`). Hand near the E-stop for the settle. | LEVELLING → IDLE. Read the NEW `pose_offset_rad`. **Prediction (the fit's own claim): it shrinks from 0.80° to ≲ 0.2° about x** (the fit's hold-out attitude floor is 0.20°). If it stays ~0.8°, the geometry did not reach the node (row 3/4) or the sign of the modelled STOW tilt is wrong — stop and say so; do not proceed to rung B on a stale offset. |
| A5 | GUI: **Activate** again (or a `go_to_pose` to the active pose) so the platform re-seats under the fresh offset. | Quiet, profiled move. Record `min`/`max` leg revolutions from the GUI: with the fitted L0 the legs will read a few tenths of a rev different from previous sittings at the same pose. |
| A6 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK` (after A4 the frame is gravity-corrected), `box OK … ` rows, and the **frame check line**: record the offset it reports. Under the old geometry at this z it read ~8.5 mm; the plan's step-7 acceptance is ≤ 1–2 mm at every z (rung C). A `hop box REFUSED` or a hash complaint here means the install (row 3) does not match the tree — stop. |

## 3. Rung B — tilt-map recapture (§ 6 step 6)

Fresh `level` immediately before (A4 counts if nothing moved since); flat
floor, no shims; hand parked and empty. Same rung C1 procedure as
`session_tilt_calibration.md`, same C0 pins:

```bash
python3 tests/hardware/tilt_cal_grid.py --dwell-s 2.0 \
    --base-condition "kincal apply 2026-09-27: fitted geometry, flat floor, no shims"
```

| # | Expect |
|---|---|
| B1 | The tool captures 5×5 over ±150 mm at z = 170, writes a NEW `config/tilt_calibration.yaml`, reloads it live, re-measures 6 check poses, prints PASS/FAIL per pose. **Nonzero exit = FAIL.** |
| B2 | **Prediction: the residual map is much smaller than the retired one** (row 8) — the old map was correcting the same geometry error the fit removed. If the new map is as large as the old, the fit removed something other than what the map was correcting — surface it before committing the map. |
| B3 | Commit the new map in its own commit (it is machine-written; the commit message names this sheet and the apply commit it was captured against). |

## 4. Rung C — the flying frame check (§ 6 step 7, the § 8 acceptance)

| # | Do | Expect |
|---|---|---|
| C1 | (system python3, ROS sourced, ACTIVE, TRAJECTORY mode, ARMED, hand parked) `python3 tests/hardware/kincal_capture.py --check 2>&1 \| tee temp/logs/kincal_check_console.log` | The § 4 per-session homing check: ~8 poses, under a minute, CSV under `temp/logs/kincal_check_<ts>.csv`. |
| C2 | (venv) `python tools/kincal_fit.py temp/logs/kincal_check_<ts>.csv --offsets-only --geometry temp/reports/kincal/kincal_sweep_20260927_143217/proposed_geometry.yaml` (the applied geometry; `--offsets-only` refuses without `--geometry`) | Per-leg zero offsets against the applied geometry: expect them within the 2026-09-27 sweep's re-home repeat (0.73 mm, § 4 PASS). Larger = homing moved between sittings; that is the § 4 "part two" the plan keeps per-session. |
| C3 | Self-toss regression, R4 sheet rung order (dress rehearsal → self-toss, nothing physical changed for it): `python3 tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss --arm A --attempts 3`, then the via-action rows. | Each catch's OUTCOME line. Record the frame-check offset the node reports at **every z it visits**; the acceptance is ≤ 1–2 mm at each. |
| C4 | Decide on the landing subtraction (`skill_node._on_balls`'s mocap-offset subtraction, adopted 2026-09-23 under the 25 mm bound). | If C3 reads ≤ 2 mm at every z, it retires in the NEXT session's commit (a code change, not a sitting action). If it still reads several mm at some z, the plan's § 8 is not met: record the per-z numbers and stop there — the owner decides between another sweep and living with the subtraction. |

## 5. Exit criteria and what to record

- A4: offset before / after `level` (rad and degrees about x).
- A6 and C3: the frame-check offset at every z visited.
- B1: new map's PASS/FAIL and its largest node residual against the retired
  map's.
- C2: the per-leg zero offsets.
- C3: caught / total, and any guard latch with its trip reason.

Logbook: one short-form entry for the sitting (the plan's § 6 steps 6–7
verdicts, the (date, command, result) triples), `related_plan:
kinematic-calibration.md`.

## 6. Rollback

`git revert <apply-commit>` restores the CAD geometry, the old tilt map, the
old boxes and the old gate hash together; then `python config/generate_config.py`,
`python sim/model/generate_mjcf.py`, `colcon build`, and — because the
persisted offset would by then have been re-measured under the fitted geometry
— `level` again before anything moves.
