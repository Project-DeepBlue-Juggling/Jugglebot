---
title: Tilt calibration grid — pose-dependent gravity-level reference (residual map over the workspace)
created: 2026-08-02
status: active
related_logbook:
  - 2026-07-28-anomaly-fixes-validation-sitting.md
related_config: config/tilt_calibration.yaml → (new, machine-written); config/hardware_config.yaml → jugglebot_operational.inclinometer_offset_deg
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/levelling.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py::_svc_get_platform_tilt
  - ros_ws/src/jugglebot/jugglebot/state_machine.py::LevellingHandler
  - ros_ws/docs/levelling_frame.md
---

# Plan — Tilt calibration grid (pose-dependent level reference)

**Branch:** `mvp-trajectory-bringup`
**Extends:** contract C-LEVEL-1 / C-LEVEL-1.O (`ros_ws/docs/levelling_frame.md`,
plan `levelling-frame-contract.md`) from a single gravity offset to a
pose-dependent residual map. The single-offset `level` routine keeps every role
it has today; this plan layers on top of it.

## Context

Operator report (2026-08-02): on **every** sitting in which Jugglebot threw
vertically to a non-`(0, 0, 170)` pose, many throws clipped platform hardware
before landing in the hand — the ball leaves slightly "backwards" (+ve platform
rotation about x). The motivating sittings themselves are unlogged (owner
decision 2026-08-02: build the tool canonically rather than reconstruct them;
rung C3 of this plan captures the symptom quantitatively instead).

The recorded seed evidence is the extremity-tilt table in
`logbook/2026-07-28-anomaly-fixes-validation-sitting.md` (§ lines 354–383):
mocap-measured platform tilt error at commanded-level extremity poses grows
from **0.041° at (60, 0) to 0.604° at (150, −150)**, is **not linear** in
(x, y) (a 2-gain fit is refuted), and has revisit repeatability 0.001–0.014° —
15–40× smaller than the effect. Tilt converts to landing displacement at
v·θ·T ≈ **41.9 mm/° at a 0.6 m toss** (103 mm/° at the 1.48 m band ceiling),
and the cup basin is ~30–40 mm — so tilt error in the 0.7–0.95° range strikes
platform hardware at 0.6 m, and the measured 0.6° corner is already the
"occasional drop" regime. That entry itself nominated an inclinometer sweep as
the discriminator.

The root cause is structural: the `level` routine measures the platform's tilt
against gravity at **one pose** (the firmware ACTIVATE pose) and applies that
single offset at **every** pose. Any pose-dependent kinematic error (geometry,
compliance, assembly) is invisible to it by construction.

**What this plan builds:** an operator-run calibration that drives the platform
over a configurable (x, y) grid at the operating height, dwells and reads the
on-board inclinometer at each node, writes an interpolatable **residual tilt
map**, and auto-applies it so every commanded pose gets a gravity-true level
reference. Validation includes a deliberately tilted base (objects under
Jugglebot) to prove the layered design: base tilt is common-mode, absorbed by
`level`; the map captures only the robot's own pose-dependent error and should
be **invariant under base tilt**.

## Architecture

### What exists (verified 2026-08-02, architecture scan)

- **Sensor**: Murata SCL3300 on the Platform Teensy (mode 4, 10 Hz internal
  LPF), read on demand — one sample per trigger, no averaging
  (`Teensy_code_platform.ino:377–406`). Jetson-side service
  `get_platform_tilt` (`teensy_bridge_node.py:929–930`, handler `:4569`) has
  **no state gating** — callable while holding a pose in TRAJECTORY; 3 relay
  retries, |tilt| ≤ 0.785 rad validity bound, NaN on failure.
- **Level routine**: `level` from IDLE → ACTIVATE → settle 0.5 s → one read →
  `pose_offset_rad = reading + radians(inclinometer_offset_deg)` (**ADD**,
  `state_machine.py:457–460`; the `:129` "Accumulated" comment is wrong — it
  overwrites) → publish `/gravity_offset` → persist 2×int16 mrad on the
  Platform Teensy (RAM-only, truncating cast, worst case 0.081° combined).
- **Application (C-LEVEL-1)**: `trajectory_node` stores one rotation
  `R = Rodrigues([-tx, -ty, 0])` and pre-multiplies it onto every **external**
  pose at six enumerated ingest sites (E1–E6), rotation-only, once, at plan
  install (`motion/levelling.py:63–115`; `trajectory_node.py:1373–1437,
  1785–1808, 2185–2220`). Live plans keep their frame (in-flight rule).
  Observability: `gravity_correction_loaded` on `/trajectory/status` → toss
  `REJECTED_NOT_LEVELLED` (C-LEVEL-1.O).
- **Dwell is free**: the 40 Hz emitter streams a terminal hold forever;
  MPC_STALE keys on setpoint *age*, never value. **No firmware change is
  needed anywhere in this plan** — the map is invisible to firmware, which
  works purely in motor-rev space.

### Owner-ratified decisions (2026-08-02)

1. **Grid domain**: (x, y) at the operating height. The throw stroke is the
   *hand* on its slider; the platform COM holds one z for all juggling
   patterns. z stays a scalar in the grid spec so a z sweep is a config
   change, not a redesign.
2. **Layered, not replacing**: `level` keeps its boot-gate role and
   persistence; the map stores pose-dependent **residuals** relative to the
   levelled reference.
3. **Storage**: committed, machine-written `config/tilt_calibration.yaml`
   with capture metadata.
4. **UX**: CLI acquisition tool + session runbook; the operator runs every
   actuating command.
5. **Non-gating**: a missing/invalid map never blocks a session — behaviour
   degrades to today's. The toss gate is unchanged.

### Map semantics and math

- **Residual definition**: at grid node i, with a *fresh* `level` correction
  loaded and the map **not** loaded, command the level orientation at
  (xᵢ, yᵢ, z_grid), dwell, read N times:
  `residual_i = mean(raw_reading) + radians(inclinometer_offset_deg)` — the
  exact LevellingHandler formula. At the home node this is ≈ 0 by
  construction (the tool asserts it; a violation means the level reference is
  stale → abort capture).
- **Application**: `combined_offset(pose) = level_offset +
  bilinear(map, intent_xy)`, then the existing single Rodrigues
  `correction_from_offset(combined_offset)`. Additive rotation-vector
  composition of two sub-degree tilts has second-order error < 1e-4 rad —
  negligible against the 0.04–0.6° signal. One implementation point
  (`motion/levelling.py`), one new `levelling.*` entry, manifest rows updated
  in the same commit.
- **Keying**: the lookup keys on the **uncorrected intent pose** (x, y) —
  evaluated before the rotation is rewritten, once per external-pose ingest.
  No fixed-point iteration, no per-knot lookup (an emitter-side lookup is
  disqualified: it desyncs u0/u1/u2 from declared Hermite velocities, escapes
  the feasibility gate, and steps the wire below every guard threshold).
- **Outside the hull**: clamp to the nearest hull point. Never extrapolate,
  never NaN.
- **Orientation scope**: the map is captured at the level orientation and
  applied additively at all commanded orientations (8b tilt-aims up to 5.75°,
  reload receive tilt 10.8°). Orientation-dependence of the residual is
  untested — rung C0 takes one tilted-pose probe reading to size it; a tilt
  axis is a follow-on sweep, not this plan.

### Delivery, persistence, observability

- **Loader** in `trajectory_node` at startup, resolution order:
  `$JUGGLEBOT_TILT_CAL` env override → **repo source tree** → ament share
  fallback. This deliberately inverts the `friction_ff_params.py` order
  because the calibration tool rewrites the source-tree file at runtime; an
  ament-copy-first order would silently serve stale calibration until the
  next colcon build (the exact trap `friction_ff_params.py:24–33` documents).
- **Reload**: new service `trajectory/reload_tilt_map` (`std_srvs/Trigger` —
  no interfaces rebuild for the service). The acquisition tool calls it after
  writing the file; the response message carries the loaded version.
- **Status**: `TrajectoryStatus` gains `tilt_map_loaded` (bool) and
  `tilt_map_version` (string). This **does** require
  `colcon build --packages-select jugglebot_interfaces jugglebot` — the
  runbook's build gate says so explicitly (a partial rebuild takes down all
  ball-op actions via module-scope ImportError).
- **Load validation** (all-or-nothing, loud): finiteness of every node,
  monotonic axes, shape match, |residual| ≤ 0.05 rad sanity bound, known
  schema version. Any failure → map not loaded, `tilt_map_loaded` stays
  false, ERROR log. Absent file → silent identity (soft absence, non-gating).
- **Map file schema** (v1): `version`, `captured` block (ISO date, git SHA,
  tool + args, can-bridge `uptime_ms` first/last, `level_offset_rad` loaded
  at capture, base-condition free-text), `grid` block (`z_mm`,
  `orientation: level`, `x_mm[]`, `y_mm[]`), `residual_rad` block
  (`tx[iy][ix]`, `ty[iy][ix]`), `stats` block (per-node sd, n_reads,
  failed_nodes). `tilt_map_version` = capture date + content hash prefix.

### Explicit non-goals

- **No firmware change.** The Teensy 2-float persistence channel keeps
  serving the single offset only.
- **No sim plumbing.** `sim/` and `controller/` have zero levelling concept
  (verified); the tilted-base validation is hardware-only.
- **No position correction.** C-LEVEL-1 stays rotation-only; under a tilted
  base, base-frame vs gravity-frame xy diverge by ~sin(tilt)·displacement
  (≈5 mm at 2° over 150 mm) — an accepted, documented limitation the
  validation criteria account for.
- **Dormant MPC path**: `mpc_bridge_node` (B1) gets a contract-doc row noting
  the map must reach it on MPC revival (it currently lacks even offset
  validation); no code now.

## Implementation Phase Summary

| Phase | Scope | Gate | Status |
|-------|-------|------|--------|
| 1 | Contract amendment (doc-first) + pure map core (`motion/tilt_map.py`, `levelling.py` extension) + stale-cite fix | `./run_tests.sh` + `/audit` (normative doc) | **done** (2026-08-02) |
| 2 | `trajectory_node` integration: loader, reload service, status fields, ingest keying, structural-test manifest | `./run_tests.sh --full` | **done** (2026-08-02) |
| 3 | Acquisition tool + offline analyser + session runbook | `./run_tests.sh --full` (pre-sitting) | **done** (2026-08-03) |
| 4 | Hardware sittings C0–C3 (operator) | per-rung PASS/ABORT | pending |

## Implementation Phases

### Phase 1 — Contract amendment + map core (pure Python)

Doc first: amend `ros_ws/docs/levelling_frame.md` with the pose-dependent
extension (**C-LEVEL-2**): residual-map semantics, additive composition rule
and its second-order bound, uncorrected-intent keying, per-target-evaluation
requirement (per-knot lookup explicitly forbidden with the three
disqualifiers), clamp-to-hull rule, load-validation contract, observability
fields, non-gating semantics, capture preconditions (fresh `level`, map
unloaded, hand quiescent), and the B1 revival-obligation row. Update the E/D
manifest table for the new `levelling.*` entry point.

Code: `motion/tilt_map.py` (pure) — schema parse/validate → `TiltMap`,
bilinear lookup with hull clamp; `motion/levelling.py` gains the single new
entry point composing `level_offset + map residual` into the existing
Rodrigues path. No behaviour change when no map is present (identity
residual).

Also in this phase (docs hygiene): fix the stale
`teensy_bridge_node.py:1430` citation in
`tests/hardware/session_phase8_toss_hardware.md:81` (actual: decode
`:349-372`, publish `:1747-1755`) — flagged independently by every scan
scout. (Line 348 is the comment above the namedtuple; the spec's original
`:348–372` was off by one and is corrected here to match what shipped.)

Tests (`tests/motion/test_tilt_map.py`, extend `test_levelling.py`): bilinear
exactness at nodes and midpoints, hull clamping on all four edges + corners,
rejection of NaN/shape-mismatch/non-monotonic-axes/over-bound maps, additive
composition vs full-rotation ground truth at worked examples, home-node
identity property.

Gate: `./run_tests.sh`; `/audit --unstaged` before commit (normative doc
touched). Logbook entry (short form unless a trigger fires).

#### Phase 1 — Outcome (2026-08-02)

**Done as specified.** `ros_ws/docs/levelling_frame.md` carries **C-LEVEL-2**
(composes with C-LEVEL-1, never replaces it) with every clause the phase asked
for, plus an Enforcement-table row set naming the three halves. New pure
`motion/tilt_map.py`: `TiltMapError`, `TiltMap`, `parse_tilt_map`,
`load_tilt_map`, `lookup`, `map_version`. One new entry point,
`levelling.correction_for_pose(offset, tilt_map, pose)` — bit-identical to
`correction_from_offset` when `tilt_map is None`, so non-gating degradation is a
property of the function, not of a caller's `if`. No existing signature or
behaviour changed, so the `tests/ros/test_levelling_frame.py` manifests are
untouched (Phase 2's ingest rewiring is what adds rows there). The stale
`teensy_bridge_node.py:1430` citation in
`tests/hardware/session_phase8_toss_hardware.md:81` is fixed to the sites
verified at HEAD: decode `:349-372`, publish `:1747-1755`.

Two additions beyond the letter of the phase, both additive and reported:
`TiltMap.z_mm` (capture height carried through as provenance — Phase 3's
analyser and the runbook both want it; never keyed on), and `lookup` **raising**
on a non-finite query rather than propagating NaN into the commanded rotation.
Phase 2's loader/ingest must therefore treat `TiltMapError` from `lookup` the
same way it treats a rejected map: degrade to offset-only and log, never crash a
callback. One **substitution**: the `correction_for_pose` tests live in
`test_tilt_map.py` rather than extending `test_levelling.py`, so the entry
point's tests sit beside the map they consume; `test_levelling.py` is unchanged.

**What the independent audit changed before commit** (findings taken, not
argued down):

- **The `< 1e-4 rad` bound was false at its own stated limit.** The draft said
  "below 1e-4 rad for sub-degree tilts" and pinned it with a *single* point 7×
  inside the bound. The exact second-order term is `|level_offset × residual|/2`
  (probe `/tmp/probe_tilt_bound.py`, 2026-08-02, matches the measured matrix
  difference to 5 s.f.): **7.19e-5 rad** at the measured envelope (0.78185° ×
  0.604°, holds) but **1.523e-4 rad** at 1° × 1°, and **1.25e-3 rad** at what
  `MAX_ABS_RESIDUAL_RAD` alone permits. The contract now carries a regime table
  instead of a bare number, and the test sweeps the azimuth ring at both
  magnitudes (the error is a cross product, so a fixed direction measures
  whatever the author happened to pick).
- **`map_version` churned on provenance.** It hashed the whole `grid` block,
  including `z_mm` and `orientation` — contradicting all three documents that
  described it. It now hashes `version` + the two axes + the residual grids,
  **float-normalised**, so YAML's `170` vs `170.0` cannot churn a version that
  the Phase-3 tool re-emits every run.
- **`TiltMap` leaked its invariants.** A hand-built map with a 1-node axis raised
  `ZeroDivisionError` — a type no caller is told to expect — and
  `parse_tilt_map` froze arrays it did not own (`np.asarray` returns the caller's
  object). `__post_init__` now copies, freezes and validates, so the invariant
  travels with the type whatever route builds it.
- **The stale-citation fix was half-done.** `tests/hardware/session_anomaly_fixes.md:93`
  carried the identical `teensy_bridge_node.py:1430` citation for the identical
  claim; fixed in the same commit, and `grep` for that string now returns only
  descriptions *of* the fix.

**Phase 2 must widen the structural guard's vocabulary.**
`tests/ros/test_levelling_frame.py:243` defines
`_APPLY_FUNCS = {'levelling.correct_pose', 'levelling.apply_gravity_correction'}`
and `test_every_application_sits_in_a_declared_ingest_handler` forces any other
callee to be declared `'store'`. `correction_for_pose` is neither: `'store'`
means *once per `/gravity_offset` message*, and this is *once per external
pose*. Declaring it `'store'` would be the cheap fix and would silently retire
the distinction the guard exists to hold. Add a third kind (e.g.
`'build:E3+E4'`) and teach the assertion about it.

**Verification** — `pytest tests/motion/ tests/ros/test_levelling_frame.py -q`
(run 2026-08-02): **967 passed in 281.08 s**; `./run_tests.sh --full` (run
2026-08-02, post-audit-fix tree): **parallel 4501 passed + 3 xfailed in
451.44 s, serial 9 passed in 40.00 s, total 497 s — RESULT: PASS**.

### Phase 2 — trajectory_node integration

- Loader (resolution order above) + all-or-nothing validation; absent → soft
  identity.
- `trajectory/reload_tilt_map` (`std_srvs/Trigger`): re-read, re-validate,
  atomic single-reference swap (the `_on_gravity_offset` pattern); failure
  keeps the previous map and reports it.
- The six ingest sites evaluate the combined offset keyed on the uncorrected
  intent pose, via the new `levelling.*` call shape; structural-test manifest
  rows (`tests/ros/test_levelling_frame.py`) land in the same commit.
- `TrajectoryStatus.tilt_map_loaded` / `.tilt_map_version` (+ publish at the
  existing 5 Hz status site).
- `setup.py` data_files row for `config/tilt_calibration.yaml` (ament
  fallback path).

Tests (`tests/ros/`): loader happy/absent/corrupt paths via `tmp_path` + env
override; reload service swap + failure-keeps-old-map; status fields; keying
regression (map lookup sees the pre-correction pose); double-application
regression (8b tilt-aim pose gets exactly one composition); in-flight rule
(reload mid-plan does not re-frame the live plan — next install only).

Gate: `./run_tests.sh --full` at phase closure. Python 3.8 (`from __future__
import annotations`) throughout `ros_ws/`.

#### Phase 2 — Outcome (2026-08-02)

**Done as specified**, with three decisions Phase 3 depends on and one test the
phase asked for that cannot exist.

- **Loader** — path policy lives in the pure layer:
  `tilt_map.tilt_map_candidates(environ=None) -> Tuple[str, ...]` and
  `tilt_map.resolve_tilt_map_path(environ=None) -> Optional[str]`. Order is
  `$JUGGLEBOT_TILT_CAL` → repo source tree → ament share, and it returns `None`
  rather than raising (`friction_ff_params` fail-fasts; that YAML is a hard
  dependency, this one is a refinement). **The env override is authoritative:
  when set it is the ONLY candidate.** A typo'd override that fell through would
  apply *a different calibration than the operator named* while reporting
  `tilt_map_loaded=True` and a plausible version — strictly worse than applying
  none, since C-LEVEL-2 is non-gating.
- **The source-tree candidate is found by SEARCH, not by a fixed `__file__`
  walk — and the pre-commit audit is what made that necessary.** The first draft
  copied `friction_ff_params.py`'s five-level walk, which from the colcon install
  tree lands on `ros_ws/install/jugglebot`; `setup.py` installs config into
  `share/jugglebot/config/`, so `install/jugglebot/config/` is a directory
  nothing creates. The source-tree candidate could therefore **never exist in
  production**, every load would have resolved the build-time share copy, and
  Phase 3's write-then-reload cycle would have re-applied the previous build's
  calibration under `tilt_map_loaded=true` — the exact trap the inversion exists
  to prevent, and green under every test, because every assertion was about the
  *order* and the order was right. `_find_repo_root` now searches upward for
  `config/hardware_config.yaml` beside `ros_ws/` (correct from both trees,
  `None` for a detached deployment). **Phase 3 gets this for free but must not
  re-introduce a fixed walk of its own** when the tool resolves the file it
  writes — use `tilt_map.resolve_tilt_map_path`.
- **Reload with the file absent UNLOADS the map and returns `success=True`.**
  Reload's contract is "make the node agree with the file". This is load-bearing
  for Phase 3: `--force-uninstall` moves the file aside and reloads precisely to
  reach `tilt_map_loaded == false` before a capture, and keeping a stale
  in-memory map would defeat the capture preconditions silently. Only an
  **invalid** file keeps the previous map (`success=False`, ERROR, version of
  what is still applied named in the response).
- **The structural guard grew a third kind, not a relabel.**
  `tests/ros/test_levelling_frame.py` now declares `store` (per
  `/gravity_offset` message) / `build:` (per external pose — `correction_for_pose`)
  / `apply:` (per external pose — `correct_pose`), with four `build:` rows beside
  the four `apply:` rows. A new `test_every_apply_has_a_build_in_the_same_scope`
  closes the C-LEVEL-2 mirror bug: an ingest that applies the correction without
  building it for its pose is a *working* C-LEVEL-1 application that silently
  ignores the map. B1 (`mpc_bridge_node`, dormant) is the one declared exception
  and is the same revival obligation the contract already carries.
- **The keying regression the phase specified is unwritable, and that is a
  stronger result than the test.** It asked for a map lookup keyed on the
  corrected pose to produce a detectably different residual. It cannot:
  C-LEVEL-1 is rotation-only, so the corrected and uncorrected poses have
  **identical x/y** and key the same cell — keying on the corrected pose is a
  fixed-point iteration that converges in one step, not a wrong answer. What is
  observable, and what is tested instead, is that each external pose is keyed on
  **its own** x/y: `timed_target(hold_after=False)` — one call, two external
  poses — must give the (150, 150) target the corner residual and the neutral
  return the home-node residual, which a correction hoisted per service call
  would not. The property itself (`corrected[:3] == intent[:3]`) is pinned so
  that if the correction is ever allowed to move a position, the keying rule
  stops being free and fails loudly.

Also landed, unasked: an autouse `no_tilt_calibration` fixture in
`tests/ros/conftest.py`. Once Phase 4 commits a real `config/tilt_calibration.yaml`,
every `TrajectoryNode` built in `tests/ros/` would otherwise start applying a
hardware calibration and the whole C-LEVEL-1 E-row family would fail on a
few-thousandths-of-a-radian diff — and the suite's answers would depend on
whether *this machine* had been calibrated. The fixture pins the loader off; the
tests that want a map set the env var themselves.

**Verification** — see the logbook entry's Verification section for the full
(date, command, result) triples. Phase gate: `./run_tests.sh --full`, run on the
final tree (the session crossed midnight, so the entry is dated by the work and
the runs by the clock).

### Phase 3 — Acquisition tool, analyser, runbook

`tests/hardware/tilt_cal_grid.py` (rclpy client, `traj_ramp_battery.py`
family: requests via `/trajectory/go_to_pose`, **never arms, never changes
modes**, fail-closed when unarmed):

- Grid spec: `--x/--y` node lists or `--box ±mm --nodes N` (default 5×5 over
  ±150 mm), `--z` (default 170), `--dwell-s`, `--n-reads` (defaults pinned by
  rung C0), `--lean-gain 0.0`, explicit long `--move-duration-s`.
- Preflight (refuses to start otherwise): `gravity_correction_loaded` true
  **and fresh `level` this session** (operator-confirmed prompt),
  `tilt_map_loaded` **false** (a map loaded during capture would bake itself
  into the data — the tool refuses; `--force-uninstall` moves the file aside
  and reloads first), hand quiescent (tilt reads block the Platform-Teensy
  loop that streams hand moves), can-bridge freshly rebooted
  (`uptime_ms` echoed and logged).
- Per node: go_to_pose → wait `planned_duration_s` + settle → N sequential
  `get_platform_tilt` calls (never concurrent — relay is serialized) →
  residual = mean + mounting offset; NaN/invalid → node marked failed,
  policy `--on-fail={continue,abort}` (abort returns to centre first; never
  park at a raised displaced pose).
- Home-node sanity: |residual(0,0)| ≤ 3×noise-sd or abort (stale level).
- Outputs: per-read CSV → `temp/logs/tilt_cal_grid_<ts>.csv`, `_meta.json`
  (git SHA, uptime first/last), map YAML → `config/tilt_calibration.yaml`,
  then `reload_tilt_map` + status readback = **auto-apply**, then a
  verification pass over M off-node check poses printing residuals and a
  PASS/FAIL verdict (nonzero exit on FAIL).
- `--dry-run` makes zero service calls; prints the node sequence + ETA.

`tools/tilt_cal_analyse.py` (offline, cogging-analyser pattern): heatmap +
quiver of the residual field, node-sd table, map-vs-map diff mode (for rung
C2), HTML/PNG to `temp/reports/`, `--json` for machine consumption.

`tests/hardware/session_tilt_calibration.md` runbook: house format (danger
banner, roles/safety framing verbatim, build gate incl. **both**
`jugglebot_interfaces` and `jugglebot`, can-bridge power-cycle, fresh-`level`
precondition, read-only preflight block, rungs C0–C3 below with numeric
PASS/ABORT, after-rung data obligations).

Tests: pure parts (grid-spec generation, residual math incl. mounting
offset, map-file writer round-trip vs the Phase-1 loader, CSV schema) via
import, no ROS required.

Gate: `./run_tests.sh --full` (mandatory pre-sitting anyway).

#### Phase 3 — Outcome (2026-08-03)

**Done as specified**, plus three CLI additions each forced by a rung this plan
already defines, and one design decision left deliberately unfinished.

- **`tests/hardware/tilt_cal_grid.py`** — pure core (grid spec, residual
  reduction, document assembly, CSV rows) importable with **zero** ROS at module
  import; `rclpy` and the interface packages are imported inside `run()` only.
  Runs under system python3.8 with ROS sourced, not the venv. Writes via
  `tilt_map.tilt_map_candidates()[0]` — **no `__file__` walk anywhere**, per the
  Phase-2 audit finding — and the post-write `tilt_map_version` readback is the
  hard guarantee that the node loaded the file the tool wrote.
- **Three flags beyond the phase's list, each serving a Phase-4 rung that could
  not otherwise be run:**
  - `--no-apply` (**C0**): capture + CSV + meta, assemble and validate the map,
    but write nothing and reload nothing. A read-noise probe must not leave a
    calibration behind, and C0 runs before any threshold exists to verify
    against.
  - `--verify-only` (**C2a**): re-measure the check poses against the
    already-loaded map with no recapture. C2a is *defined* as "re-`level` only,
    NO recapture", so without this the rung is unrunnable. Check poses are
    derived from the loaded map's own axes, which is what makes them identical
    to the capture run's.
  - `--read-gap-s` (**C0**): the SCL3300 runs a **10 Hz internal LPF** and is
    read one sample per trigger, so back-to-back reads resample the same
    filtered state — the mean is fine but the spread collapses, and the per-read
    sd is what gates the home node and sizes θ_acc. Default 0.15 s is just over
    one filter period. Without this the tool would have reported a noise floor
    the sensor does not have.
- **Failed nodes are refused, not interpolated.** `--on-fail continue` finishes
  the sweep (the operator gets the full CSV and learns *which* nodes are bad) but
  **no map is written** and the exit is nonzero. Filling a failed node from its
  neighbours would invent calibration data at exactly the place the machine had
  trouble, and C-LEVEL-2 refuses NaN at load.
- **The home-node gate is two-sided.** `|residual(0,0)|` must be within
  `max(3 × sd, floor)` **and** under an absolute ceiling. One-sided fails one of
  two ways: 3 × sd alone is degenerate on a quiet sensor (the Teensy persistence
  quantum is 1 mrad/axis on its own, so a tolerance of 3e-9 rad would abort a
  good capture — and a gate that fires on good data is a gate the third operator
  disables), while a *noisy* sensor makes 3 × sd wide enough to launder a
  genuinely stale reference.
- **A real CLI bug, found by its own test:** `--x -150,-75,0,75,150` — the
  documented explicit-list form — fails, because argparse only treats a leading
  `-` as a value for bare negative *numbers*. The parser now appends the `=`
  fix to that specific error rather than leaving it to be rediscovered at the
  robot.

**The analyser's outlier threshold is deliberately left provisional, and that is
the honest result.** The first draft flagged a node whose residual departed from
its neighbours' mean. That flags the **entire boundary** of a good capture: an
edge node's neighbours all lie on its interior side, and the residual field is
genuinely curved (the 07-28 table refutes only a *linear* fit). The statistic is
now a **second difference** — a pinned leg does not bend the field, it kinks it —
which correctly ignores a pure gradient. But no *threshold* on it survives
scrutiny: on a probe field with structured curvature, `4 × median` missed a real
0.5° pin while `2 × median` flagged ten good nodes. Rather than tune a number on
one synthetic field and ship it as validated, the flag is a coarse net and the
report leads with the **top/median curvature ratio**, which needs no threshold at
all — probe (2026-08-03, smooth field, 5×5/±150, 8 reads at 2e-5 rad): **1.01
clean, 2.43 at a 0.2° pin, 3.65 at 0.3°, 6.07 at 0.5°**. Rung C1 produces the
first real field and is what should pin `CURVATURE_FLOOR_DEG` /
`CURVATURE_FLAG_MULT`; `--curvature-flag-deg` overrides them meanwhile.

**Tests** live in `tests/motion/test_tilt_cal_grid.py` (55) and
`tests/sim/test_tilt_cal_analyse.py` (21) — 76 together, not `tests/ros/`, following the
precedent that every importer of a `tests/hardware/` script sits in
`tests/motion/` and every importer of a `tools/` script in `tests/sim/`. The
strongest one drives the **real** `state_machine.LevellingHandler` phase and
compares its output to the tool's reduction: a residual is *defined* as that
formula, so an assertion restating it would drift in lockstep with a sign flip —
and a sign flip inverts every node, aiming the machine roughly twice as badly as
no map at all. A second pushes the writer's own output through
`tilt_map.parse_tilt_map`, the same function the node loads with.

**What the independent audit changed before commit** (findings taken, not argued
down):

- **BLOCKING — a second Ctrl-C escaped the return-to-centre guard.** The guard
  caught `Exception`, which does **not** catch `KeyboardInterrupt`. The return
  blocks for up to `--timeout-s` plus the move, so the reflex second interrupt
  landed inside it, propagated out of the `finally`, and skipped the artefact
  writes, the rclpy shutdown **and** the `RETURN TO CENTRE FAILED` message —
  leaving the platform parked at a raised displaced pose in silence, which is
  precisely what the tool's docstring and the runbook both promise cannot
  happen. Now `except BaseException`, with the artefact write and the shutdown
  each in their own `finally`, and a structural test pins the handler type
  because `run()` cannot be exercised without a robot.
- **The contract overstated its own enforcement.** `levelling_frame.md` claimed
  the tool "refuses to start" unless all four capture preconditions hold. Only
  *no map loaded* is machine-checkable; fresh-`level` and hand-quiescent are
  operator confirmations (`--yes` skips them, now with a loud live-capture
  warning) and can-bridge uptime is warn-and-record by deliberate choice. The
  contract now carries a per-precondition enforcement table saying so — a tool
  trusted to enforce what it cannot observe is worse than one that says it
  cannot.
- **The write-target sanity check ran after the four-minute sweep**, despite its
  own docstring claiming otherwise; hoisted into the preflight.
- **A home-node *service* failure under the default `--on-fail continue`
  skipped the stale-level gate** via `continue`, then captured 24 more nodes
  before refusing for the missing home node. Visit 0 is now unconditionally
  fatal — aborting early is the whole reason home is measured first.
- `_meta.json` shipped an empty `nodes` list on the home-gate abort (the
  runbook's headline ABORT criterion) because summaries were built in a trailing
  loop; they are now appended per node.
- Two runbook command errors: C3's `--force-uninstall --dry-run` returns before
  `run()` and so moves nothing, and C2b told the operator to back up the C1 map
  *after* the command that overwrites it. Both rewritten as explicit sequences.
- The SCL3300 "mode 4, 10 Hz LPF" citation pointed at a line range that shows
  the read path but not the mode; now cited separately (`SCL3300.h:144` for the
  library default, which the firmware never overrides).
- Test counts in this Outcome and the logbook were wrong (55/76 claimed against
  54/75 measured) — corrected, and re-measured after the fixes.

**Phase 4 must update the PROVISIONAL defaults in the same commit as the C0
logbook entry** — `--dwell-s`, `--n-reads`, `--read-gap-s`, `--threshold-deg`,
the home-gate floor/ceiling in `tilt_cal_grid.py`, and the two `CURVATURE_*`
constants in `tilt_cal_analyse.py`. They are marked PROVISIONAL in code
precisely so that commit is easy to find.

**Verification** — scoped: `pytest tests/motion/test_tilt_cal_grid.py
tests/sim/test_tilt_cal_analyse.py -q` (run 2026-08-03, post-audit-fix tree):
**76 passed in 1.05 s**. Phase gate: `./run_tests.sh --full` — the (date,
command, result) triple is in the logbook entry's Verification section. No
hardware ran in this phase.

### Phase 4 — Hardware sittings (operator; runbook is the authority)

- **C0 — probe** (probe-first rule: no threshold is asserted in a test
  before this rung pins it): fresh boot; hold centre, two extremity poses,
  and one ~6° tilted pose; ~30 sequential reads each at 2–3 dwell settings.
  Pins: single-read noise sd, `--n-reads`, `--dwell-s`, achievable accuracy
  θ_acc, and sizes orientation-dependence (the tilted-pose reading). Also
  the first hardware exercise of mid-TRAJECTORY tilt reads. PASS: N-read
  mean repeatability ≤ 0.03°; reads return finite at all four poses.
- **C1 — baseline capture + verify**: fresh `level` → capture 5×5/±150 →
  auto-apply → verification at ≥6 off-node check poses. PASS: every check
  residual ≤ **θ_acc (provisional 0.15° until C0 pins it)**; home node ≈ 0;
  map min/max consistent with the 07-28 seed table (same order, same worst
  quadrant). 0.15° ≈ 8 mm at the 0.78 m default toss — well inside the
  30–40 mm basin.
- **C2 — deliberately tilted base (the extreme validation)**: shim the base
  by ~1–2° → re-`level` only (NO recapture) → repeat C1's check poses.
  PASS: residuals still ≤ θ_acc — proves base tilt is absorbed as
  common-mode by `level` while the map stays valid. Then recapture and diff
  (analyser diff mode): PASS if max node delta ≤ max(2×noise, 0.05°) —
  proves map invariance. ABORT if any go_to_pose rejection pattern suggests
  the stacked tilt approaches workspace/feasibility limits.
- **C3 — symptom closure (displaced-toss A/B, optional but recommended)**:
  displaced vertical tosses (the exact symptom geometry) with map off vs on,
  same session, fresh boot, uptime logged. PASS: hardware-clip behaviour
  disappears or measurably reduces with the map on. This rung supplies the
  quantitative symptom record the motivating sittings never got.

Each rung closes with a logbook entry; C2 almost certainly meets a
Discussion trigger (it tests a design hypothesis).

## Testing plan

Pure math in `tests/motion/` (no ROS mocks); node behaviour in `tests/ros/`
(mock-ROS conftest; new service mocked there); tool pure-parts imported
directly. No `serial`/`nightly` markers anywhere — everything here asserts
contracts, none of it measures the machine; the motion/trajectory surface is
hardware-safety and never demoted. Ports ephemeral, files via `tmp_path`.
Cross-node choreography is minimal by design (one client calling two
existing services sequentially), so the Phase-7-style real-ordering-trace
obligation is scoped to the C0/C1 evidence capture (CSV + `_meta.json` are
the trace) rather than an offline checker — revisit if the tool ever grows
an action server.

## Risk register

- **Double-application class**: capture-with-map-loaded (tool refuses;
  preflight), keying on corrected pose (regression test), B1's second copy
  on MPC revival (contract row), `motor_commands.cartesian_to_motor_commands`'
  dead `gravity_correction` parameter (trap — stays unused).
- **Moving-lookup class**: forbidden per-knot evaluation is contract text +
  the per-target property is regression-tested. Residual transit error =
  map-gradient × move length, exact at the terminal hold — throws and reads
  both happen from stationary holds.
- **Extrapolation**: clamp-to-hull only; a wrong-signed edge extrapolation
  aims throws worse than no map.
- **Persistence quantization**: the Teensy-persisted single offset truncates
  at 1 mrad/axis (worst 0.081° combined) — runbook requires a fresh `level`
  before capture and before validation reads so truncation never rides under
  the map during a sitting; across reboots it is a known accuracy-floor term
  (future firmware `lroundf` fix, out of scope).
- **Workspace edges**: ±150 mm corners near stroke clamps; grid nodes stay
  inside the toss planning box, rejections are handled as node failures, and
  the firmware stroke clamp silently pinning a leg would corrupt that node's
  reading — the analyser flags outlier nodes for operator review.
- **SCL3300 unknowns**: noise floor and orientation-dependence unmeasured in
  repo — C0 exists precisely to pin them before any test asserts a
  threshold.
- **Uptime discipline**: capture on a fresh can-bridge boot; `uptime_ms`
  logged first/last in `_meta.json` and the map's `captured` block.
- **Interfaces rebuild**: TrajectoryStatus fields force the two-package
  colcon build; the runbook build gate names both packages (partial rebuild
  = module-scope ImportError takes down all ball-op actions).

## Notes for collaborators

- The map is a **refinement, never a gate**: absence must always degrade to
  exactly today's behaviour. Resist adding a rejection code for it.
- Change `levelling_frame.md` **before** touching `levelling.py` — the
  contract's "no second implementation" clause is the load-bearing part.
- The acquisition tool is request-only by design: arming, modes, and E-STOP
  belong to the operator. Keep it that way.
- `Teensy_code_canbridge/` and `Teensy_code_platform/` live under
  `ros_ws/src/jugglebot/`, not the repo root (CLAUDE.md's sketch is
  approximate here).
- Grid captures are meaningless without a fresh `level` immediately before —
  the residuals are *defined* relative to that reference.
