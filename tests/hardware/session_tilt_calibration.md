# Hardware Session — Tilt calibration grid (C-LEVEL-2), rungs C0–C3

**Plan**: `plans/active/tilt-calibration-grid.md` § Phase 4
**Contract**: `ros_ws/docs/levelling_frame.md` § **C-LEVEL-2** (composes with
C-LEVEL-1, never replaces it)
**Tools**: `tests/hardware/tilt_cal_grid.py` (capture, operator-run),
`tools/tilt_cal_analyse.py` (offline)
**Logbook**: one entry per rung; C2 almost certainly meets a Discussion trigger
(it tests a design hypothesis).

**Goal**: measure the platform's *pose-dependent* tilt error over an (x, y) grid
at the operating height, write it as an interpolatable residual map, apply it,
and prove it removes the error at poses that were never measured — then prove
the map is invariant under a deliberately tilted base.

**Why**: `level` measures tilt against gravity at **one** pose and applies that
offset at **every** pose, so pose-dependent kinematic error is invisible to it by
construction. Measured 2026-07-28: commanded-level tilt error grows from
**0.041° at (60, 0) to 0.604° at (150, −150)**, is *not* linear in (x, y), and
repeats to 0.001–0.014°. At **41.9 mm/°** of landing displacement on a 0.6 m toss
against a ~30–40 mm cup basin, the corner of the workspace is already the
"occasional drop" regime — which is the reported symptom this plan exists to
close.

---

> **⚡ READ THIS FIRST — the platform moves continuously for several minutes.**
> A default capture drives **25 grid nodes plus 6 check poses plus a return to
> centre**, each a ±150 mm traverse at z = 170 — about **4 minutes of near-
> continuous motion** with only a ~1 s dwell between moves. It looks
> unattended; it is not.
>
> - **E-STOP in reach for the whole capture.** Do not walk away between nodes.
> - **The hand stays quiescent.** No ball ops, no reloads, no hand moves during
>   a capture: tilt reads block the Platform-Teensy loop that streams hand
>   moves. Do not load a ball "ready for later".
> - Nothing here throws, and nothing here arms. The tool issues
>   `trajectory/go_to_pose` requests **only** — it never arms, never changes
>   control mode, never touches limits, never commands the hand.
> - Every exit path (abort, Ctrl-C, exception, node failure under
>   `--on-fail abort`) returns the platform to the centre node **first**. It is
>   never left parked at a raised displaced pose. If you ever see
>   `RETURN TO CENTRE FAILED`, bring it home manually before deactivating.

## Roles & safety framing

- **The operator (Harrison) runs every robot-actuating command below.** The
  implementing session prepared these exact commands + PASS/ABORT criteria and
  verifies read-only.
- **If your physical intuition disagrees with any framing here, that is
  load-bearing signal — say so before proceeding.** In particular: the claim
  that base tilt is common-mode and the map is invariant under it (rung C2) is a
  *design hypothesis*, not a measurement. If the platform looks wrong at a
  shimmed base before any number says so, stop.
- E-STOP always available. Any ABORT criterion ⇒ cut power / trigger the guard,
  then debrief before retrying.
- A missing or rejected map is **never** a fault: C-LEVEL-2 is non-gating and
  absence degrades to exactly today's behaviour. There is no rung here you must
  finish to keep juggling.

## Preconditions

- Jugglebot powered, ODrives up, CAN3 healthy (green `link_status`).
- **POWER-CYCLE THE CAN-BRIDGE TEENSY before the sitting** (the uptime-lag
  discipline). **Quote `uptime_ms` with every number you record** — tracking lag
  grows with can-bridge uptime (10 ms fresh → ~240 ms at 30 h), and a capture on
  a degraded plant is a capture of the degradation. The tool echoes it at
  preflight and records first/last in `_meta.json` **and** the map's `captured`
  block; it warns above 30 min and never refuses.
- `run_mpc.py` is **NOT** running (sole-binder :5557 interlock).
- **Build gate — BOTH packages. This is not optional:**
  ```bash
  cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot
  source install/setup.bash
  ```
  then **relaunch** `jugglebot_launch.py` (launch runs the *installed* copy).
  `TrajectoryStatus.msg` gained **two** fields in Phase 2 (`tilt_map_loaded`,
  `tilt_map_version`). A partial rebuild is a module-scope ImportError and
  **`trajectory_node` exits ~200 ms after launch** — that exact signature. If
  the launch window shows `trajectory_node` dying immediately, this is why;
  rebuild both packages, do not debug anything else first.
- Arm per the Phase-1 sequence: launch → home → **`level`** → activate →
  confirm the 40 Hz hold stream → TRAJECTORY → **zero motion at arm**.
- **A FRESH `level`, run from IDLE, immediately before the capture.** This is
  not the usual "check the flag, only `level` if missing" rule — a grid capture
  is *meaningless* without it. Every residual is **defined** relative to that
  reference, so a stale one shifts every node by the same amount and produces a
  map that looks perfectly plausible and aims every throw wrong. The tool asks
  you to confirm it and gates the home node on it, but neither check can
  distinguish "levelled twenty minutes and one relaunch ago" from "levelled just
  now". **Also re-`level` before each rung's verification reads** — the Teensy
  persistence truncates at 1 mrad/axis (worst 0.081° combined), so a reboot in
  between puts truncation under the map.
- Remember the correction is **per-PROCESS**: any relaunch (including this
  build gate) silently drops it while `RobotState.levelling_complete` still
  reads True.

### Pre-flight — read-only, confirms the freshly-built code is live

```bash
ros2 service list | grep reload_tilt_map                                      # Phase-2 service present
ros2 topic echo /trajectory/status --once | grep tilt_map                     # both NEW fields present
ros2 topic echo /trajectory/status --once | grep gravity_correction_loaded    # -> true
ros2 topic echo /link_status --once | grep -A1 uptime_ms                      # log this number
python3 tests/hardware/tilt_cal_grid.py --dry-run                             # node order + ETA, zero ROS calls
```

- `trajectory/reload_tilt_map` missing, or `grep tilt_map` returning nothing ⇒
  the build gate did not take. Rebuild **both** packages and relaunch.
- `gravity_correction_loaded: false` ⇒ run `level` from IDLE (over
  `/orchestrator_command`, **not** the `/activate` service).
- `tilt_map_loaded: true` before a capture ⇒ see `--force-uninstall` below. Do
  **not** capture with a map loaded: it bakes the map into its own successor and
  the error compounds silently on every recapture.

---

## Rung C0 — probe (pins every threshold the later rungs assert)

**No threshold below C1 is asserted in a test until this rung pins it.** The
SCL3300's noise floor, its settling time, and its orientation-dependence are all
unmeasured in this repo. Every default in `tilt_cal_grid.py` marked PROVISIONAL
(`--dwell-s 1.0`, `--n-reads 8`, `--read-gap-s 0.15`, `--threshold-deg 0.15`,
and the home-gate floor/ceiling) is a placeholder waiting on these numbers.

This is also the **first hardware exercise of mid-TRAJECTORY tilt reads** —
`get_platform_tilt` has no state gating, and that it works while holding a pose
is assumed, not demonstrated.

Fresh can-bridge boot; fresh `level`.

```bash
# 3x3 probe at two dwell settings — captures, writes CSV+meta, applies NOTHING
python3 tests/hardware/tilt_cal_grid.py --no-apply --nodes 3 --n-reads 30 \
    --dwell-s 0.5 --base-condition "C0 probe, flat floor"
python3 tests/hardware/tilt_cal_grid.py --no-apply --nodes 3 --n-reads 30 \
    --dwell-s 2.0 --base-condition "C0 probe, flat floor"
```

`--no-apply` is the point of the rung: a read-noise probe must not leave a
calibration behind. The tool still assembles and validates the map and reports
whether it *would* be accepted.

Then the **orientation-dependence probe**, which the grid tool cannot do (it
commands the level orientation only, by design — the map is *defined* at that
orientation). Hold one ~6° tilted pose and read manually:

```bash
# ~6 deg about x at centre, slow; then read the inclinometer several times
ros2 service call /trajectory/go_to_pose jugglebot_interfaces/srv/GoToPose \
  "{pose: {position: {x: 0.0, y: 0.0, z: 170.0}, orientation: {x: 0.0523, y: 0.0, z: 0.0, w: 0.9986}}, duration_s: 3.0, lean_gain: 0.0}"
ros2 service call /get_platform_tilt jugglebot_interfaces/srv/GetTiltReadingService "{}"   # repeat ~10x
ros2 service call /trajectory/go_to_pose jugglebot_interfaces/srv/GoToPose \
  "{pose: {position: {x: 0.0, y: 0.0, z: 170.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, duration_s: 3.0, lean_gain: 0.0}"
```

**PASS**
- Reads return **finite** at all four poses (centre, two extremities, tilted).
- N-read **mean repeatability ≤ 0.03°** across the repeats at a given pose and
  dwell (this is the number the later thresholds are built on).
- Per-read sd is **stable between the two dwell settings** — if 2.0 s is
  materially quieter than 0.5 s, the platform was still settling and
  `--dwell-s` must rise for C1.

**ABORT**
- Any NaN read at any pose ⇒ the tilt path is not healthy mid-TRAJECTORY; stop
  and diagnose before capturing anything.
- Mean repeatability > 0.03° ⇒ the noise floor is comparable to the 0.041°
  smallest signal, and a grid capture cannot resolve the field. Raise
  `--n-reads` and re-probe before proceeding.

**After the rung, record:** `uptime_ms` first/last, the per-read sd at each
dwell, the chosen `--dwell-s` / `--n-reads` / `--read-gap-s`, the achievable
accuracy **θ_acc**, and the tilted-pose reading vs the centre reading (this
sizes orientation-dependence — a large difference means the map's
apply-at-all-orientations assumption needs a follow-on tilt sweep, which is
**not** in this plan). Retain both CSVs. Then update the PROVISIONAL defaults in
`tilt_cal_grid.py` and the analyser's `CURVATURE_*` constants in the same
commit as the C0 logbook entry.

---

## Rung C1 — baseline capture + verify

Fresh can-bridge boot; **fresh `level` immediately before**; flat floor, no
shims; hand quiescent and empty.

```bash
python3 tests/hardware/tilt_cal_grid.py \
    --base-condition "C1 baseline, flat floor, no shims"
# if a map is already loaded from an earlier attempt:
python3 tests/hardware/tilt_cal_grid.py --force-uninstall \
    --base-condition "C1 baseline, flat floor, no shims"
```

Use the `--dwell-s` / `--n-reads` / `--threshold-deg` C0 pinned. The tool
captures 5×5 over ±150 mm at z = 170 (home node first as the stale-reference
gate), writes `config/tilt_calibration.yaml`, calls `reload_tilt_map`, confirms
the applied `tilt_map_version` matches what it wrote, then re-measures 6
off-node check poses and prints PASS/FAIL per pose. **Nonzero exit = FAIL.**

Then analyse:

```bash
python tools/tilt_cal_analyse.py                       # the map just applied
python tools/tilt_cal_analyse.py --csv temp/logs/tilt_cal_grid_<ts>.csv
```

**PASS**
- Home node residual ≈ 0 (the tool's own gate; it aborts otherwise).
- **Every** check-pose residual magnitude ≤ **θ_acc** (provisional **0.15°**
  until C0 pins it). 0.15° ≈ 8 mm at the 0.78 m default toss — well inside the
  30–40 mm basin.
- Map min/max **consistent with the 2026-07-28 seed table**: same order of
  magnitude (0.04–0.6°) and the **same worst quadrant**. A map whose corner is
  10× the seed table is measuring something else.
- Analyser: no flagged outlier nodes, and **top/median curvature ratio near
  1.0** (a smooth field). A ratio above ~2 means one node is kinked — most
  likely a leg pinned on its stroke clamp at an extremity, which corrupts that
  node without any rejection.
- `tilt_map_loaded: true` with the expected version on `trajectory/status`.

**ABORT**
- `STALE LEVEL REFERENCE` at the home node ⇒ re-`level` from IDLE and start
  over. Do not override this.
- The tool refuses to write ("unusable node") ⇒ re-run with a longer
  `--dwell-s`; check whether the named nodes sit on a leg stroke clamp.
- `APPLIED THE WRONG FILE` ⇒ the node resolved a different path than the tool
  wrote. Stop: check `$JUGGLEBOT_TILT_CAL` on both sides and whether an ament
  share copy is shadowing the source tree. Do not "just re-run".
- Any check pose > θ_acc ⇒ record it and stop before C2; the map is not doing
  its job and C2's result would be uninterpretable.

**After the rung, record:** `uptime_ms` first/last, the map version, the min/max
residual and worst quadrant vs the seed table, every check-pose residual, the
analyser's curvature ratio. Retain the CSV, `_meta.json`, and the analyser
report dir. **Commit `config/tilt_calibration.yaml`** with the C1 logbook entry.

---

## Rung C2 — deliberately tilted base (the extreme validation)

This is the rung that tests the **design hypothesis** of the whole plan: base
tilt is *common-mode* and is absorbed by `level`, while the map captures only
the robot's own pose-dependent error — so the map should be **invariant** under
base tilt. If that is false, the layered design is wrong and the map would have
to be recaptured whenever the machine is moved.

Shim the base by **~1–2°** (objects under Jugglebot). Keep it stable and
document exactly what you used.

### C2a — re-`level` only, NO recapture

```bash
# operator: re-level from IDLE, then re-arm to TRAJECTORY
python3 tests/hardware/tilt_cal_grid.py --verify-only
```

`--verify-only` re-measures the **same** check poses (they are derived
deterministically from the loaded map's own axes) against the already-applied
C1 map. It captures nothing and writes no map.

**PASS**: every check residual still ≤ θ_acc — base tilt was absorbed as
common-mode by `level` and the C1 map is still valid at a tilted base.
**Note the known, accepted limitation**: C-LEVEL-1 is rotation-only, so under a
tilted base the base-frame and gravity-frame x/y diverge by
`sin(tilt) · displacement` ≈ **5 mm at 2° over 150 mm**. That is a position
error, not a tilt error, and it is out of scope here — do not read it as a C2
failure.

### C2b — recapture and diff

**Back up the C1 map BEFORE the recapture — the capture overwrites it, and the
C1 map is the thing C2b diffs against:**

```bash
cp config/tilt_calibration.yaml temp/logs/tilt_cal_C1.yaml
```

(`--force-uninstall` also renames the original to
`config/tilt_calibration.yaml.<stamp>.bak`, which is gitignored — so a forgotten
`cp` is recoverable. Do not rely on it.)

```bash
python3 tests/hardware/tilt_cal_grid.py --force-uninstall \
    --base-condition "C2 recapture, ~1.5 deg shim under <describe>"
python tools/tilt_cal_analyse.py config/tilt_calibration.yaml \
    --diff temp/logs/tilt_cal_C1.yaml
```

> **The ament share copy.** `setup.py` installs
> `config/tilt_calibration.yaml` into `share/jugglebot/config/` whenever it
> exists at build time, and the loader resolves env → source tree → share. So
> once you have committed the C1 map and run **any** `colcon build`, there are
> **two** copies and removing only the source one falls through to the share
> one. `--force-uninstall` handles this — it moves aside *every* existing
> candidate — but if it reports the map is **STILL loaded** it will print the
> path it resolved: remove or rename that share copy (or rebuild with the
> source file absent) and re-run. Do not proceed with a map loaded; a capture
> under a loaded map bakes that map into its own successor.
>
> If `$JUGGLEBOT_TILT_CAL` is set, `--force-uninstall` **refuses** rather than
> renaming a file you named. Unset it, or move that file aside yourself.

**PASS**: `max node delta ≤ max(2 × noise, 0.05°)`. The analyser computes that
bound from the two captures' own recorded per-read sd and prints
`VERDICT: PASS/FAIL`; its exit code is nonzero on FAIL.

**ABORT**
- Any `go_to_pose` rejection pattern suggesting the stacked tilt is approaching
  workspace or feasibility limits (`UNREACHABLE`, `WORKSPACE`) ⇒ stop and
  reduce the shim. Stacked tilt eats workspace at the extremities.
- C2a residuals > θ_acc ⇒ the invariance hypothesis is in trouble. **Do not
  proceed to C2b to "get more data" — stop and discuss.** A recapture at a
  tilted base with an unexplained C2a failure produces a map nobody can
  interpret.

**After the rung, record:** the shim geometry and measured base tilt,
`uptime_ms` first/last for both captures, C2a's per-pose residuals, C2b's max
node delta and the threshold the analyser computed, and both map versions.
Retain both maps and both CSVs.

**Then restore the C1 map — all four steps.** The C2 recapture is still applied
in memory until you reload, so skipping this leaves the *shim-contaminated* map
running for the rest of the session and into anything that follows:

```bash
# 0. SAVE THE C2 MAP FIRST — it exists ONLY at config/tilt_calibration.yaml.
#    The tool writes the map to the source tree and only the CSV/_meta.json to
#    temp/logs/, and --force-uninstall's .bak is the C1 map, not this one. Step 1
#    overwrites it. This is the artefact of the rung that tests the plan's
#    central hypothesis, and reproducing it costs another shimmed-base sitting.
cp config/tilt_calibration.yaml temp/logs/tilt_cal_C2.yaml

# 1. put the C1 map back in the source tree (the recapture overwrote it)
cp temp/logs/tilt_cal_C1.yaml config/tilt_calibration.yaml

# 2. make the RUNNING node load it — a file copy alone changes nothing
ros2 service call /trajectory/reload_tilt_map std_srvs/srv/Trigger

# 3. confirm the applied version
ros2 topic echo /trajectory/status --once | grep tilt_map_version
```

**The confirmation is a comparison, not a glance:** step 3's `tilt_map_version`
must equal **the version you recorded at C1** ("After the rung, record: …the map
version"). Reading the field without comparing it to that number cannot tell the
C1 map from the C2 recapture — both report a plausible-looking version string.
If they differ, the reload did not pick up the copy: check that step 1 wrote the
source-tree path the node resolves, and re-read the share-copy note above.

**This rung meets a Discussion trigger** — it tests a design hypothesis, and
either outcome (invariant / not invariant) is a result a future session must be
able to reconstruct. Write the full investigation form.

---

## Rung C3 — symptom closure (displaced-toss A/B) — optional but recommended

This is the rung that supplies the **quantitative symptom record** the
motivating sittings never got: displaced vertical tosses clipping platform
hardware before landing in the hand.

Same session, fresh can-bridge boot, flat floor, C1 map restored. Run the
**exact symptom geometry** — vertical tosses at a displaced pose, not at
(0, 0, 170) — with the map **off**, then **on**, back to back.

```bash
# --- arm A: map OFF ---
cp config/tilt_calibration.yaml temp/logs/tilt_cal_C1.yaml     # keep a copy
mv config/tilt_calibration.yaml temp/logs/tilt_cal_OFF.yaml    # move it aside
ros2 service call /trajectory/reload_tilt_map std_srvs/srv/Trigger
ros2 topic echo /trajectory/status --once | grep tilt_map_loaded   # -> MUST be false
# ... operator runs the displaced tosses per session_phase8_toss_hardware.md ...

# --- arm B: map ON ---
cp temp/logs/tilt_cal_C1.yaml config/tilt_calibration.yaml     # restore
ros2 service call /trajectory/reload_tilt_map std_srvs/srv/Trigger
ros2 topic echo /trajectory/status --once | grep tilt_map       # -> loaded true + the C1 version
# ... repeat the IDENTICAL toss set ...
```

Confirm `tilt_map_loaded` before **each** arm of the A/B — an A/B where both
arms ran in the same state is the easiest way to waste this rung.

> **Arm A's `mv` is not sufficient on its own — verify, do not assume.** Once
> the C1 map has been committed and **any** `colcon build` has run, a second
> copy exists at `share/jugglebot/config/tilt_calibration.yaml`, and the loader
> falls through to it when the source-tree file disappears. The reload then
> reports **success** with the map **still loaded**, and arm A silently runs
> with the map ON — which is exactly the "both arms in the same state" waste
> this rung cannot afford, except invisible.
>
> That is why the `grep` above says **MUST be false**. If it is `true`:
>
> ```bash
> # find the copy the node is actually resolving, and move it aside too
> ros2 topic echo /trajectory/status --once | grep tilt_map_version
> mv "$(ros2 pkg prefix jugglebot)/share/jugglebot/config/tilt_calibration.yaml" \
>    temp/logs/tilt_cal_SHARE.yaml
> ros2 service call /trajectory/reload_tilt_map std_srvs/srv/Trigger
> ros2 topic echo /trajectory/status --once | grep tilt_map_loaded   # -> false
> ```
>
> Restore that share copy after the rung, or leave it out and rebuild — but
> record which you did.

**PASS**: hardware-clip behaviour **disappears or measurably reduces** with the
map on, at the same displaced poses, same session, same boot.

**ABORT**: any clip that damages hardware; any toss rejection pattern that
differs between the two arms (that would confound the comparison).

**After the rung, record:** `uptime_ms` first/last for both arms, the toss poses
and heights, clip counts / catch counts per arm, and run `/diagnose --latest`
over the session's telemetry. Retain the rosbag.

---

## Exit criteria

C0 pins the thresholds and the PROVISIONAL defaults are updated in code; C1
produces a committed `config/tilt_calibration.yaml` that passes its own
verification at ≥ 6 off-node poses; C2 either confirms map invariance under a
tilted base or produces a written account of why it does not; C3 (if run) closes
the symptom quantitatively. Each rung closes with a logbook entry quoting its
`uptime_ms` first/last and its (date, command, result) verification triple.

---

## Session result — <date>: **<PASS/FAIL>**

*(fill in per rung, in the format of `session_phase1_hold.md` § Session result —
a criterion/measured/verdict table, then the sequencing observed)*
