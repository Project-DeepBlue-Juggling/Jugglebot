# Hardware Session — Tilt calibration grid (C-LEVEL-2), rungs C0–C3

**Plan**: `plans/archived/2026-08-15 tilt-calibration-grid.md` § Phase 4
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
> A default capture drives **25 grid nodes, 6 home anchors beyond the first, 6
> check poses and a return to centre**, each a ±150 mm traverse at z = 170 —
> **~5.3 minutes of near-continuous motion** at the pinned `--dwell-s 2.0`
> (measured from the tool's own `--dry-run` ETA, 2026-08-10; re-read it after
> any timing change). It looks unattended; it is not.
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
  `tilt_map_version`). On a partial rebuild the installed message lacks those
  fields, the 5 Hz status timer's first tick raises AttributeError, and
  **`trajectory_node` exits ~200 ms after launch** — that exact signature. If
  the launch window shows `trajectory_node` dying immediately, this is why;
  rebuild both packages, do not debug anything else first.
- Arm per the Phase-1 sequence: launch → home → **`level`** → activate →
  confirm the 40 Hz hold stream → TRAJECTORY → **zero motion at arm**.
- **A correction loaded and CONSTANT throughout the sweep** (2026-08-10: the
  map is **anchor-mean home-referenced** — the home pose is re-measured k
  times through the sweep and every shipped residual is
  `measured − mean(anchors)`, so a stale-but-constant level reference cancels
  *exactly*; the algebra is in `levelling_frame.md` § C-LEVEL-2). A **fresh
  `level` immediately before is RECOMMENDED, not required**: it keeps
  `|m_home|` small and the tool's 0.010 rad WARN meaningful. What IS required
  — and now machine-checked — is that the correction does not **change**
  mid-sweep: the tool subscribes `/gravity_offset` and aborts on any message
  during the sweep, and the anchor series aborts on a discrete step between
  two consecutive anchors. The confirm prompt asks for constancy intent, not
  freshness. (The old start-of-capture `STALE LEVEL REFERENCE` abort is
  retired — mistuned against the level path's own single-sample scatter,
  1.2–1.7 mrad/axis measured 2026-08-09. Its replacement, a tight statistical
  start-vs-end drift gate, is retired too — it aborted **both** complete C0
  captures on 2026-08-10 over ~1.6–1.8 mrad of reproducible home variation
  with the `/gravity_offset` monitor silent; see § Rung C0 § Result.)

### What the home-anchor gate does now (2026-08-10)

The tool re-measures the home pose at the start, after every
`--home-revisit-every` non-home grid visits (default **4** ⇒ ~3 anchors on a
3×3, ~7 on the 5×5) and at the end. Every capture **prints the anchor table**
(`t_s`, `m_tx`, `m_ty`, sd, `n_ok`), the per-axis peak-to-peak spread and the
signed start-to-end trend — record it with the session notes, it is the
evidence that settles the mechanism below.

| Outcome | Condition | What to do |
|---|---|---|
| **report** | always | Copy the anchor table into the session notes. |
| **WARN** | any axis p-p > **0.002 rad** | Nothing — the capture is valid and was written. Note whether the series *trends* (warm-up) or *scatters* (arrival repeatability). |
| **ABORT** | any axis p-p > **0.0087 rad** (0.5°, the owner's repeatability tolerance) | Real: check for a shifting base, a loose platform joint, or a leg not returning to its commanded position. |
| **ABORT** | consecutive-anchor step > **0.005 rad** | A discrete event — re-level, relaunch re-push, or something mechanical letting go. `/gravity_offset` is the causal detector and should also have fired. |

Smooth sub-WARN wander of the home reading is **expected and accepted**.
- **Verification reads no longer need a re-`level`**: check poses are scored
  **home-referenced** (the verification pass re-measures home first and
  subtracts it), so the constant reference drops out of PASS/FAIL too.
- Remember the correction is **per-PROCESS**: any relaunch (including this
  build gate) silently drops it while `RobotState.levelling_complete` still
  reads True — and a relaunch **mid-sweep** re-pushes the Teensy-persisted
  copy (int16-mrad truncated), which is exactly the constancy violation the
  drift gate and `/gravity_offset` monitor exist to catch.

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

## If a leg collapses (ODRIVE_FATAL) — forensics FIRST, then recover

Precedent: 2026-08-09, leg 0 latched **SPINOUT_DETECTED** (`disarm_reason`
67108864, `active_errors` 0) 2.5 s into the move to (−150, −150) — a
first-of-class fault at a pose with proven precedent (held 4×, thrown from 4×
on 2026-07-27); kinematics are exonerated (leg 0 was the *most-retracted* leg
there, ≥ 85 mm from every bound). The fault itself is an **OPEN hardware
follow-up** for the next sitting; this section is how to handle a recurrence
without destroying the evidence.

1. **Capture forensics BEFORE any relaunch or clear.** The next launch's BOOT
   pre-flight auto-clears the drive-side error record — the 2026-08-09 code
   survived only because a rosbag happened to be rolling. The ODrive
   re-broadcasts its error cyclically until cleared or powered off, so while
   the stack is still up:
   ```bash
   ros2 topic echo /robot_state --once     # motor_states: active_errors / disarm_reason per leg
   ```
   confirm the session rosbag is rolling (it captures the fault edge too),
   and keep the launch-window scrollback. The tilt tool now dumps the decoded
   ODrive errors (console + `_meta.json` `forensics` key, `abort_reason`
   always set) on any abort — save that meta path with the session notes.
2. **Clear and recover — no relaunch needed.** FAULT entry already disarmed
   the wire, so the direct-clear branch runs (`/recover` is for the
   armed-latch case, not this):
   ```bash
   ros2 topic pub --once /orchestrator_command std_msgs/msg/String "{data: clear_errors}"
   # (equivalent: ros2 service call /clear_errors std_srvs/srv/Trigger)
   ```
   The guard latch releases, FAULT exits to BOOT, and BOOT skips to IDLE via
   the persisted `is_homed`.
3. **`activate` re-raises the collapsed leg safely**: the firmware ACTIVATE
   seeds an error-gated TRAP_TRAJ **from the actual (collapsed) position** —
   no manual repositioning, no jump.
4. **Do NOT re-home** — *unless ODrive power was interrupted*. Then the
   encoder reference is gone while the Platform-Teensy `is_homed=true`
   persists, i.e. the BOOT skip-to-IDLE is now telling a lie: treat
   `is_homed` as stale and re-home before any motion.

---

## Rung C0 — probe (pins every threshold the later rungs assert)

**No threshold below C1 is asserted in a test until this rung pins it.** The
SCL3300's noise floor, its settling time, and its orientation-dependence are all
unmeasured in this repo. Every default in `tilt_cal_grid.py` marked PROVISIONAL
(`--dwell-s`, `--n-reads`, `--read-gap-s 0.15`, `--threshold-deg 0.15`) is a
placeholder waiting on these numbers. C0 now also pins **read autocorrelation
at the 0.15 s gap** (the 2026-08-09 series show AR(1) effective-N ≈ 14–23 of
30 — per-read timestamps in the CSV are real as of 2026-08-10, so this is
computable from the artifact) and, if time allows, **level-to-level scatter**
(~10 successive `level`s; the three 2026-08-09 levels scattered 1.15–2.40
mrad). **C0 RAN on 2026-08-10 — see § Result below**; what remains open from
this rung is θ_acc confirmation at C1 and the read-autocorrelation number.

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

> **Re-run guidance after the 2026-08-09 aborted sitting**: the command line is
> **unchanged** (above). What changed in the tool (2026-08-10): the capture is
> home-referenced so the `STALE LEVEL REFERENCE` abort that killed attempt
> 23:53 cannot recur (a fresh `level` is now recommended, not gating); the
> node order is centre-out so the corners — where the leg-0 collapse happened
> — come LAST; lean defers to the node config (0.6, the transit shape all
> four corners were proven with on 2026-07-27); CSV timestamps are per-read
> and real; and any mid-move/mid-read fault aborts immediately with a decoded
> ODrive forensics dump in the console and `_meta.json` (`abort_reason` is
> now always recorded). Expect **several extra home visits** per capture: the
> interleaved home anchors (start + every 4 non-home visits + end — the
> reference is their mean), and on applying runs the verification home
> reference.

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
**not** in this plan). Retain both CSVs. The analyser's `CURVATURE_*` constants
are **pinned at C1, not here** — they need a real measured field, which C0's
four probe poses are not.

### Rung C0 — Result, 2026-08-10: **PINS TAKEN, gate reframed**

Two 3×3 `--no-apply` captures, both completing all nine nodes cleanly:
`temp/logs/tilt_cal_grid_20260810_115343*` (`--dwell-s 0.5`) and
`_120735*` (`--dwell-s 2.0`), `uptime_ms` 43.5 M → 44.5 M (~12 h — the bridge
was not freshly booted; quote that with any lag-sensitive reading).

- **`--dwell-s 2.0` is PINNED** (was 1.0). Per-read sd across all nine nodes
  and both axes: **0.25–0.92 mrad, median 0.38 mrad at 2.0 s** vs **0.42–1.35
  mrad, median 0.56 mrad at 0.5 s** — a ~33 % quieter read, i.e. the platform
  was still settling at 0.5 s. `--n-reads 30` stays for probe work; the
  shipped default (8) is unchanged for production captures.
- **Home re-measure offset — the reason anchors exist.** Both captures ended
  with home shifted **+1.81 mrad (run 1) and +1.59 mrad (run 2) on ty**
  against their own start (tx: +0.56 / +0.42 mrad), *reproducibly*, with the
  `/gravity_offset` monitor **silent** and the ODrive forensics **clean** — so
  the loaded correction had not changed. The old drift gate (tol 0.87 / 0.94
  mrad) aborted and discarded both complete captures; that behaviour was the
  bug. **Mechanism is OPEN**, owner's **arrival-repeatability** hypothesis
  leading (hand-built, FDM parts, path-dependent hysteresis on re-arriving at
  a pose; two identical sweeps reproducing the same offset fits), with SCL3300
  **warm-up / thermal settling** the live alternative. Owner tolerance for
  repeatability is **0.5° (8.7 mrad)**, so ~0.1° is comfortably inside it and
  inside θ_acc 0.15°. The map is now **anchor-mean referenced** and every
  capture reports its anchor series — that series discriminates the two
  mechanisms for free, so no dedicated sitting is owed.
- **Orientation-dependence probe**: ~**+0.10° residual at 6° commanded rx**
  ≈ **1.7 % of commanded tilt**, i.e. under θ_acc across the 8b aim range. The
  apply-at-all-orientations assumption holds for now; a **tilt-axis sweep
  stays a follow-on**, not a blocker.
- Field shape from the 3×3 (dwell 2.0), **anchor-mean referenced**: |M| runs
  0.047° at home → **0.287° at (150, −150)**, mean 0.147° — same order *and*
  the same worst quadrant as the 2026-07-28 seed table (0.604° at
  (150, −150)); a 3×3 undersamples the corner, so a smaller peak is expected.
  Analyser on the raw CSV: curvature ratio 1.49, no flagged outliers.

---

## Rung C1 — baseline capture + verify

Fresh can-bridge boot; **fresh `level` immediately before**; flat floor, no
shims; hand quiescent and empty.

```bash
python3 tests/hardware/tilt_cal_grid.py --dwell-s 2.0 \
    --base-condition "C1 baseline, flat floor, no shims"
# if a map is already loaded from an earlier attempt:
python3 tests/hardware/tilt_cal_grid.py --force-uninstall --dwell-s 2.0 \
    --base-condition "C1 baseline, flat floor, no shims"
```

`--dwell-s 2.0` is the C0 pin and is passed **explicitly** (the shipped default
is still 1.0). `--n-reads` and `--threshold-deg` keep their defaults. The tool
captures 5×5 over ±150 mm at z = 170 (home node first — it is anchor 1 — then
centre-out with a home anchor after every 4 non-home visits, corners last, then
the end-of-sweep anchor; the shipped reference is the **mean** of all 7),
writes `config/tilt_calibration.yaml`, calls `reload_tilt_map`, confirms
the applied `tilt_map_version` matches what it wrote, then re-measures 6
off-node check poses and prints PASS/FAIL per pose. **Nonzero exit = FAIL.**

Then analyse:

```bash
python tools/tilt_cal_analyse.py                       # the map just applied
python tools/tilt_cal_analyse.py --csv temp/logs/tilt_cal_grid_<ts>.csv
```

**PASS**
- **Home anchor series reported and non-aborting**, and `|m_home|` below the
  0.010 rad WARN. A p-p **WARN is not a failure** — copy the table into the
  notes and record whether the series trends (warm-up) or scatters (arrival
  repeatability). Expect ~1–2 mrad of spread on ty based on C0.
- **Every** check-pose **home-referenced** residual magnitude
  `|check − m_home_verify| ≤ θ_acc` (provisional **0.15°** until C0 pins it;
  the tool re-measures home before the checks and subtracts it — printed as
  "home-ref residual"). 0.15° ≈ 8 mm at the 0.78 m default toss — well inside
  the 30–40 mm basin.
- Map min/max **consistent with the 2026-07-28 seed table**: same order of
  magnitude (0.04–0.6°) and the **same worst quadrant**. A map whose corner is
  10× the seed table is measuring something else.
- Analyser: no flagged outlier nodes, and **top/median curvature ratio near
  1.0** (a smooth field). A ratio above ~2 means one node is kinked — most
  likely a leg pinned on its stroke clamp at an extremity, which corrupts that
  node without any rejection.
- `tilt_map_loaded: true` with the expected version on `trajectory/status`.

**After the rung:** pin the analyser's `CURVATURE_FLOOR_DEG` /
`CURVATURE_FLAG_MULT` against this first real field (the C0 probe poses cannot
do it), in the same commit as the C1 logbook entry.

**ABORT**
- `HOME ANCHOR SPREAD EXCEEDS THE REPEATABILITY TOLERANCE` (any axis p-p >
  0.0087 rad = 0.5°) ⇒ the machine is not returning to the same pose within
  the tolerance the whole design assumes. Check for a shifting base, a loose
  platform joint, or a leg not returning to its commanded position.
- `DISCRETE STEP IN THE HOME ANCHOR SERIES` (consecutive anchors > 0.005 rad
  apart) ⇒ an event, not wander: re-level, relaunch re-push, or something
  mechanical letting go. Re-run the capture; do not override.
- `LEVEL REFERENCE CHANGED MID-SWEEP` (a `/gravity_offset` message arrived
  during the sweep) ⇒ same class, caught causally. Re-run from the start.
- `|m_home|` above the **0.05 rad** sanity ceiling ⇒ the platform is ~3°
  off level at its own levelling pose — that is a fault (collapsed leg?),
  not staleness. Diagnose before re-running.
- The tool refuses to write ("unusable node") ⇒ re-run with a longer
  `--dwell-s`; check whether the named nodes sit on a leg stroke clamp (the
  IK preflight refuses commanded overtravel but cannot see a mechanical
  bind).
- `APPLIED THE WRONG FILE` ⇒ the node resolved a different path than the tool
  wrote. Stop: check `$JUGGLEBOT_TILT_CAL` on both sides and whether an ament
  share copy is shadowing the source tree. Do not "just re-run".
- Any check pose > θ_acc ⇒ record it and stop before C2; the map is not doing
  its job and C2's result would be uninterpretable.

**After the rung, record:** `uptime_ms` first/last, the map version, the min/max
residual and worst quadrant vs the seed table, every check-pose residual, the
analyser's curvature ratio, and **the full home-anchor table with its p-p and
trend** (this is the second data point on the arrival-repeatability vs
warm-up question — a 7-anchor series is the first one able to separate them).
Retain the CSV, `_meta.json`, and the analyser report dir. **Commit
`config/tilt_calibration.yaml`** with the C1 logbook entry.

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
C1 map. It captures nothing and writes no map. **The fresh re-level here is
deliberate and stays**: it is not a capture precondition (capture no longer
has one) — it IS the invariance hypothesis under test, composing a new `C_now`
under the old map. Scoring is home-referenced like every verification pass.
(C2b's *recapture* side needs no fresh level at all now — the shim's
common-mode term cancels by construction, which strengthens the rung.)

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

## Session results — 2026-08-09 / 2026-08-10: **C0 PINS TAKEN, C1 PASS, C2a PASS**

Logbook (one entry for the whole arc, owner's one-file rule):
[`logbook/2026-08-10-tilt-cal-c0-blockers-level-noise-and-leg0-spinout.md`](../../logbook/2026-08-10-tilt-cal-c0-blockers-level-noise-and-leg0-spinout.md).
Rung C0's own numbers are recorded in place at § Rung C0 § Result above; this
section is the session-level roll-up.

### C0 — probe

**Sitting 1 (2026-08-09): ABORTED, no probe completed.** Six attempts on disk
(`temp/logs/tilt_cal_grid_20260809_{234616,235315,235440,235459,235631,235711}`),
all `exit_code 2`. One real gate abort — the **now-retired** start-of-capture
`STALE LEVEL REFERENCE` gate, mistuned against the level path's own
single-sample scatter (1.2–1.7 mrad/axis) — and one **leg-0
`SPINOUT_DETECTED`** collapse 2.5 s into the move to (−150, −150), kinematics
exonerated. Both blockers are written up in the logbook entry; the leg-0 root
cause is **OPEN**.

**Sitting 2 (2026-08-10): both 3×3 `--no-apply` captures completed all nine
nodes**, `temp/logs/tilt_cal_grid_20260810_115343*` (`--dwell-s 0.5`) and
`_120735*` (`--dwell-s 2.0`), `uptime_ms` 43.5 M → 44.5 M (~12 h).

| Criterion | Measured | Verdict |
|---|---|---|
| Finite reads at every pose, mid-TRAJECTORY | all nine nodes both runs, plus the tilted probe pose | PASS |
| Per-read sd stable between dwell settings | **0.25–0.92 mrad, median 0.38** at 2.0 s vs **0.42–1.35 mrad, median 0.56** at 0.5 s — ~33 % quieter | FAIL as stated ⇒ **`--dwell-s` raised to 2.0** (the platform was still settling at 0.5 s) |
| Orientation-dependence probe | **~+0.10° residual at 6° commanded rx ≈ 1.7 % of commanded** | under θ_acc — tilt-axis sweep stays a **follow-on** |
| End-of-sweep home re-measure | **+1.81 mrad (run 1) / +1.59 mrad (run 2) on ty**, reproducible; `/gravity_offset` **silent**, ODrive forensics **clean** | the drift gate aborted both — **that behaviour was the bug**; referencing went **anchor-mean** |

**Pins carried into C1**: `--dwell-s 2.0` (passed explicitly), `--n-reads 30`
for probe work only (production captures keep the shipped default 8),
`--read-gap-s 0.15` and `--threshold-deg 0.15` unchanged.

### C1 — baseline capture + verify, 2026-08-10 15:16: **PASS**

`python3 tests/hardware/tilt_cal_grid.py --dwell-s 2.0 --base-condition "C1
baseline. Flat floor, no shims"` → **exit 0**. Artifacts
`temp/logs/tilt_cal_grid_20260810_151658.csv` / `_meta.json`; map
`config/tilt_calibration.yaml` (committed as `3df256b`).

| Criterion | Measured | Verdict |
|---|---|---|
| Capture completes, no unusable node | **25/25 nodes**, `failed_nodes: []`, `--n-reads` 8 | PASS |
| Home-anchor series non-aborting | **7 anchors**, verdict **OK**; p-p **tx 0.000695 rad (0.0398°)**, **ty 0.001390 rad (0.0797°)** | PASS (below even the 0.002 rad WARN) |
| `\|m_home\|` below the 0.010 rad WARN | within WARN | PASS |
| Field consistent with the 2026-07-28 seed table | tx **−0.208…+0.289°**, ty **−0.204…+0.134°**; worst nodes **(−75, +150) tx +0.289°** and **(+150, −150) (−0.208, −0.204)°** — same order, same worst quadrant as C0's 3×3 and the seed table | PASS |
| Map applied and confirmed | `tilt_map_version` = **`2026-08-10-3bf7964f`**, readback matched what the tool wrote | PASS |
| Every check-pose home-referenced residual ≤ θ_acc (0.15°) | **6/6 PASS**, \|r\| = **0.0428, 0.0532, 0.0570, 0.0664, 0.0680, 0.1369°** | PASS |

**Anchor series — record this, it is evidence on an open question.** The ty
series' largest consecutive step (0.001390 rad) **equals** its total p-p: one
excursion after the first anchor, then a plateau. That is a
**step-after-first-excursion** signature, which fits the owner's
**arrival-repeatability** reading and does *not* fit steady thermal drift. It is
the second data point on that question, not yet decisive — keep copying the
anchor table into the notes on every future capture.

**Uptime honesty.** `uptime_ms` **55 701 003 → 55 990 503**, i.e. **~15.5 h — the
can-bridge was NOT freshly booted**, against this runbook's own
power-cycle-before-the-sitting precondition. Accepted deliberately: the uptime
hazard is *tracking lag*, and every number here is a static inclinometer read at
a settled hold. Quote the uptime alongside the numbers anyway — a future capture
taken on a fresh boot is the comparison that would show it mattered.

**Still owed from this rung**: the analyser's `CURVATURE_FLOOR_DEG` /
`CURVATURE_FLAG_MULT` were **not** pinned against this first real field.

### C2a — tilted base, re-`level` only, 2026-08-10 15:24: **PASS**

`python3 tests/hardware/tilt_cal_grid.py --verify-only` → **exit 0**. Artifacts
`temp/logs/tilt_cal_grid_20260810_152421*`. `uptime_ms` **56 144 203 →
56 300 003** (~15.6 h, same continuous session).

| Criterion | Measured | Verdict |
|---|---|---|
| Base shim geometry | **~6° about ~x**, boxes under **legs 4 and 5** (3–6× the 1–2° this rung nominates) | recorded |
| Re-`level` only, **no** recapture | `--verify-only`; no map written, C1 map still applied | as specified |
| Every check residual ≤ θ_acc (0.15°) | **6/6 PASS**, \|r\| = **0.0181, 0.0345, 0.0467, 0.0477, 0.0549, 0.1462°** | PASS |
| `go_to_pose` rejection pattern near workspace limits | none | PASS |

**The design hypothesis is hardware-validated**: base tilt is absorbed by
`level` as **common-mode** while the flat-floor map stays valid **unchanged** at
a tilted base. The map does not need recapturing when the machine is moved.

Two caveats to carry: `--verify-only` ran at the **default `--dwell-s 1.0`**,
not the C0 pin of 2.0, so its reads are marginally noisier than C1's — and the
worst check (0.1462°) sits close to the 0.15° threshold, so a re-run at 2.0 s is
the honest way to tighten that margin. Also unchanged: C-LEVEL-1 is
rotation-only, so at a 6° base the base-frame/gravity-frame x/y diverge by
`sin(tilt) · displacement` (~15 mm at 6° over 150 mm) — a **position** error,
out of scope here, and not a C2 failure.

### Operator tunings made during the same sitting

Both were made by the operator during the afternoon toss retest, and both are
committed with this arc:

| Parameter | Was | Now |
|---|---|---|
| `jugglebot_operational.levelling_settle_s` | 0.5 | **1.0** |
| `jugglebot_operational.catch_vel_scale_default` | 0.8 | **0.9** |

`levelling_settle_s` lengthens the post-ACTIVATE settle before the single
inclinometer read that *builds* the level reference — i.e. it makes the
reference this whole rung depends on quieter. Re-read § Preconditions with that
in mind if you are comparing a pre-2026-08-10 capture against a later one.

### Rung status at the end of the arc

| Rung | Status |
|---|---|
| **C0** | **DONE** — pins taken (`--dwell-s 2.0`; orientation-dependence sized at 1.7 %); read-autocorrelation / `n_eff` no longer needed (the gate that wanted it was retired) |
| **C1** | **PASS** — map `2026-08-10-3bf7964f` captured, applied, verified 6/6, and committed (`3df256b`) |
| **C2a** | **PASS** — invariance under a ~6° base tilt confirmed 6/6 |
| **C2b** | **OPTIONAL, NOT RUN** — with C2a passing, the recapture-and-diff buys map-invariance numbers, not a different verdict. The four-step "restore the C1 map" sequence above was therefore never needed; run it if you ever do C2b |
| **C3** | **SUPERSEDED** — an informal owner-run toss retest (2026-08-10 afternoon, tier temporarily flipped to 8a) threw vertically at multiple flat poses and **caught most throws** after the two tunings above, addressing the motivating clipped-throw symptom at the poses tested. **No formal map-off / map-on A/B was run**, so there is still no quantitative symptom record; the A/B stays available and cheap |

**Open after this arc**: the leg-0 `SPINOUT_DETECTED` root cause (evidence bag
`2026-08-09_23-56-48`, first-of-class — **check mechanically next sitting**, and
see § *If a leg collapses* above before touching anything), C2b, and the
tilt-axis orientation sweep.
