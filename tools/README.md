# tools/ — Standalone Hardware Test & Debug Utilities

Scripts in this directory bypass ROS2 and talk directly to hardware via python-can. They are designed for bench testing, bring-up, and diagnostics.

## Generators & drift gates (no hardware, no ROS import)

These are the exception to the sentence above: offline generators whose output
is committed and pinned by a test, so the artifact cannot silently rot. Each
regenerates in place; run it and commit the diff when its source of truth moves.

| Script | Generates | Pinned by |
|--------|-----------|-----------|
| `gen_choreography_map.py` | `ros_ws/docs/choreography.md` — publisher/subscriber/service/action graph of the Python nodes (`--check` exits 1 on drift) | `tests/ros/test_choreography_map.py` |
| `gen_gui_fk_golden.py` | `tests/ros/gui_fk_golden.json` — Python-IK golden poses replayed through `ros_ws/gui/js/stewart-fk.js` under node | `tests/ros/test_gui_fk_golden.py` |

## Preview Convention (`--preview`)

**Scope**: these rules bind the *operator-facing hardware harnesses* — the
scripts in this directory and in `tests/hardware/` that command the robot and
whose `--preview` previews a motion sequence before it is run. They do **not**
bind the offline analysis probes under `tools/probes/`, which run headless on the
Jetson where a matplotlib window is unusable; there `--preview` conventionally
prints the raw per-sample window instead (see
`tools/probes/hand_stroke_timeline.py`).

**All test harnesses that accept `--preview` must follow these rules:**

1. **Single window** — `--preview` produces exactly one matplotlib figure containing the full test sequence. Never open a separate window per test.

2. **Unified timeline** — All tests in the suite are stitched end-to-end on a shared time axis. The sequence must be C2-continuous throughout: both *during* and *between* individual tests, all commanded values must be smooth (no step discontinuities in position, velocity, or acceleration).

3. **Return-to-home transitions** — Between tests, generate a proper profiled return-to-home move (not a step command). Highlight these transition segments visually (e.g. light blue `axvspan` shading) so they are clearly distinguishable from test segments. After the final test, return to home and then show the stow segment (highlighted in green).

4. **Test boundary markers** — Mark each test's start/end with vertical dashed lines and annotate with the test name/label.

5. **Scrollbar + zoom** — For sequences longer than ~10 seconds, include a horizontal `matplotlib.widgets.Slider` at the bottom for panning, and bind mouse scroll-wheel to zoom in/out centered on the cursor. See `trajectory_viewer.py:_show_continuous_plots()` for the reference implementation.

6. **5-row plot layout** — Use 5 vertically stacked subplots sharing the x-axis:
   - Row 1: Cartesian Pose (mm / rad) — 6 channels, Cartesian colors
   - Row 2: Cartesian Twist (mm/s / rad/s) — 6 channels, Cartesian colors
   - Row 3: Leg Extensions (mm) — 6 channels, leg colors
   - Row 4: Motor Velocity Feedforward (rev/s) — 6 channels, leg colors
   - Row 5: Torque Feedforward (Nm) — 6 channels, leg colors

   Colors and labels are defined in `trajectory_viewer.py` (`LEG_COLORS`, `CARTESIAN_LABELS`, `CARTESIAN_COLORS`). Import them rather than redefining.

7. **`--dry-run --preview`** — Must work without CAN hardware. Run workspace feasibility checks, then show the preview plot using offline simulation only.

**Reference implementations:**
- `smoother_test.py:preview_all_tests()` — streaming/direct-target preview
- `trajectory_viewer.py:_show_continuous_plots()` — trajectory sequence preview

## Common Patterns

### PlatformTestHarness

`free_platform_test.py` defines `PlatformTestHarness`, the shared base for all multi-leg platform tests. Key usage:

```python
harness = PlatformTestHarness(interface='socketcan', channel='can0')
harness.connect()   # Must call explicitly — __init__ does NOT open the bus
with harness:
    harness.home_all()
    # ... run tests ...
```

Also exports: `LEG_AXES`, `encode_set_input_pos`, `encode_set_controller_mode`, `pose_to_raw_positions`, `error_names`.

### WorkspaceLimits

```python
ws = WorkspaceLimits.from_geometry(geom)  # NOT .from_config()
```

### Shutdown: Always Stow

**Every test harness must end with the platform stowed** (all legs at 0 rev, fully compressed). Never idle axes while the platform is raised — an idled motor provides no holding torque and the platform will drop.

The shutdown sequence is: **return to home → stow (0 rev via TRAP_TRAJ) → idle all axes**.

```python
def stow_platform(harness):
    harness.enter_trap_traj_mode_all(vel_limit=1.5, acc_limit=5.0, dec_limit=5.0)
    for axis_id in LEG_AXES:
        harness.states[axis_id].trajectory_done = False
    for axis_id in LEG_AXES:
        harness.send(encode_set_input_pos(axis_id, 0.0, vel_ff=0, torque_ff=0))
    harness.wait_for_all_trajectories_done(timeout_s=15.0)
```

This applies to:
- Normal completion (after all tests pass/fail)
- Error/exception paths
- Ctrl-C handler (best-effort stow, then idle)

Reference implementations: `tests/hardware/free_platform_test.py` and
`tests/hardware/single_leg_test.py`. (The original references — `smoother_test.py:safe_shutdown()`
and `dynamic_target_test.py:safe_idle_all()` — were pre-MPC harnesses removed with
`tests/archived/` in 67889a6, 2026-04-17; recoverable from git history.)

The `--preview` plot must also show the stow segment at the end of the timeline so the operator sees the full motion the robot will execute.

### Safety

All test harnesses share these safety principles:
- Conservative current limit (50% of rated) set before any motion
- Always stow before idling (see above)
- Mandatory feasibility pre-check before hardware execution
- Interactive confirmation before each test stage
- Heartbeat watchdog on all axes

## Test Harnesses

Every row below names a file that exists today. Paths are given because only one
of these harnesses (`tracking_analyzer.py`) still lives under `tools/` — the rest
are under `tests/hardware/`.

| Script | Phase | Description |
|--------|-------|-------------|
| `tests/hardware/single_leg_test.py` | 2 | Single-axis bench tests (torque, e-stop, encoder, force) |
| `tests/hardware/supported_platform_test.py` | 3A | Position control bench tests on a single supported leg |
| `tests/hardware/free_platform_test.py` | 3B-C | Platform-level tests; defines `PlatformTestHarness` |
| `tools/tracking_analyzer.py` | 6 | Rosbag analysis for ball tracking data |
| `tests/hardware/cogging_bench_test.py` | bench-rig | Velocity-mode bench runner with `--sweep`. Produces friction-vs-velocity sweeps for Stribeck fitting. |
| `tests/hardware/breakaway_ramp_test.py` | bench-rig | Torque-mode ramp from rest. Position-only escape detector measures stiction per starting electrical angle. |
| `tests/hardware/torque_step_test.py` | bench-rig | Bidirectional torque-step terminal-velocity test. Reveals controller-fight overhead in velocity-mode measurements. |
| `tests/hardware/friction_ff_demo.py` | bench-rig | Friction-FF effectiveness demo: pos_step / vel_step / sweep modes with Stribeck FF, stiction-boost, and vel_ff toggles. |

**Pre-MPC harnesses (removed 2026-04-17).** Nine harnesses this table used to
list — `trajectory_test.py`, `inertia_test.py`, `hardening_test.py`,
`dynamic_target_test.py`, `juggling_test.py`, `smoother_test.py`,
`trajectory_viewer.py`, `catch_sim_test.py`, `throw_catch_test.py` — were deleted
with `tests/archived/` in commit 67889a6. They exercised the pre-MPC quintic
path-following controller (`control_loop.py`, archived with them) and were
already excluded from pytest. Their test *scenarios* remain valid references for
future MPC hardware tests; recover any of them with
`git show 67889a6^:tests/archived/<name>`.

### Friction characterisation & FF demo (single-leg bench rig)

Four scripts under `tests/hardware/` form a complete friction-characterisation and feedforward-validation pipeline for a single ODrive leg on an isolated CAN bus. Used during the 2026-04-24 to 2026-04-27 bench-validation phase of the motion-onset dead-time investigation.

**Canonical reference:** [logbook/2026-04-27-friction-feedforward-bench-validation.md](../logbook/2026-04-27-friction-feedforward-bench-validation.md) — the bench-validation arc, withdrawn hypotheses, and final fit parameters.

**Pipeline:**

```bash
# 1. Fit Stribeck friction model from a velocity sweep
python tests/hardware/cogging_bench_test.py --sweep 0.075,0.10,0.20,0.30,0.50,1.0,2.0,5.0
python tools/friction_study_analyse.py --glob 'temp/logs/cogging_<ts>*.csv'

# 2. Measure stiction breakaway per rotor angle
python tests/hardware/breakaway_ramp_test.py --direction both --trials 8
python tools/breakaway_analyse.py temp/logs/breakaway_<ts>.csv

# 3. (optional) Confirm against terminal-velocity in TORQUE mode
python tests/hardware/torque_step_test.py

# 4. Validate friction FF + stiction-boost + vel_ff in POSITION mode
python tests/hardware/friction_ff_demo.py
```

All four scripts share safety conventions: 3 rev stroke cap, brake-resistor-aware acceleration cap (250 rev/s²), heartbeat watchdog, IDLE-on-exit. See each script's `--dry-run` output for current parameter limits.

### Friction-FF analysis tools (`tools/*_analyse.py`)

| Script | Inputs | Outputs |
|--------|--------|---------|
| `cogging_map_analyse.py` | one cogging CSV | iq-vs-electrical-angle map, harmonic R² (n=1..12), position-drift slope |
| `friction_study_analyse.py` | multiple cogging CSVs (a sweep) | Stribeck four-parameter fit (τ_c, τ_s, ω_s, b), tracking-quality plots, fwd-vs-rev electrical-map Pearson R |
| `breakaway_analyse.py` | breakaway summary CSV | per-direction breakaway iq stats, angular-dependence harmonic fits, sanity check that breakaway moved more than half a cogging period |

All three accept `--json <path>` to emit a machine-readable result; plots go to `temp/reports/`.

### Tilt calibration grid (contract C-LEVEL-2)

Pose-dependent gravity-level calibration: `level` measures the platform's tilt
against gravity at **one** pose and applies that offset at **every** pose, so
pose-dependent kinematic error is invisible to it by construction. These two
tools measure it over an (x, y) grid and ship it as an interpolatable residual
map.

| Script | Role |
|--------|------|
| `tests/hardware/tilt_cal_grid.py` | **Operator-run** capture. Drives the grid via `trajectory/go_to_pose` (never arms, never changes modes, never commands the hand), reads the inclinometer N times per node, writes `config/tilt_calibration.yaml`, reloads it in the running node, verifies the applied version, then re-measures off-node check poses. Runs under **system python3.8 with ROS 2 sourced** — not the venv. |
| `tools/tilt_cal_analyse.py` | Offline. Heat maps + quiver of the residual field, per-node sd table, outlier flagging, and a `--diff` mode for map-invariance (rung C2). Accepts `--json`; reports go to `temp/reports/tilt_cal_<ts>/`. |

**Runbook — read it before running either:** `tests/hardware/session_tilt_calibration.md`
(rungs C0–C3 with numeric PASS/ABORT). Contract: `ros_ws/docs/levelling_frame.md`
§ C-LEVEL-2. Plan: `plans/active/tilt-calibration-grid.md`.

`tilt_cal_grid.py` takes `--dry-run` (prints the node order and ETA, makes zero
ROS calls) rather than `--preview`: it commands no continuous motion sequence to
plot — it is a series of discrete rest-to-rest moves — so the five-row preview
layout above does not apply. `tilt_cal_analyse.py` is offline analysis and takes
no preview at all, matching `cogging_map_analyse.py`.

⚠ **Several defaults are PROVISIONAL until rung C0 measures them on hardware**
(`--dwell-s`, `--n-reads`, `--read-gap-s`, `--threshold-deg`, the home-node
gate bounds, and the analyser's curvature-flag constants). The SCL3300's noise
floor is unmeasured in this repo; C0 exists to pin it before any test asserts a
threshold.

### Toss aim calibration — toss_cal_fit.py + toss_cal_analyse.py

Layer **1** of the same stack: `config/toss_calibration.yaml`, contract
C-TOSS-CAL-1 (`plans/active/toss-selftuning.md`). Where the tilt map is
INCLINOMETER-measured and answers *"where is gravity at this pose?"*, the aim map
is BALL-measured and answers *"where does the ball actually go?"* — the residual
that is left after levelling, and that an inclinometer is structurally blind to
(hand-axis misalignment, cup/release asymmetry, platform residual motion).

| Script | Role |
|--------|------|
| `tests/hardware/toss_fit_lib.py` | The pure fit core, importable and hardware-free: partition rule + census, the per-toss reduction, admission guards, the thin-node rule, the two write-refusing gates, the document build. Imported by both tools and by `tests/motion/test_toss_cal_fit.py`. |
| `tests/hardware/toss_cal_fit.py` | **Desk-side**, not operator-at-the-robot: reads a JSONL corpus, writes `config/toss_calibration.yaml`. `--dry-run` / `--no-apply` write nothing; `--reload` (opt-in, needs ROS 2 sourced) calls `toss/reload_calibration` and **reads the version back**. Runs in the venv. |
| `tools/toss_cal_analyse.py` | Offline. Heat map + quiver (drawn in **landing** space), per-node n/sd, anchor series, residual-vs-uptime scatter, `--diff` map-invariance, `--group` A/B scoring on the continuous observables. `--json`; reports go to `temp/reports/toss_cal/`. |

The corpus comes from `tools/probes/toss_record_miner.py`, which mines a rosbag
into `toss_record/1` rows. Plan: `plans/active/toss-selftuning.md`. Build log:
`logbook/2026-08-10-toss-selftuning-build.md`.

⚠ `N_MIN` (8), the 10 % trim, the 1.0 mm node-snap tolerance and the 200 ms
timing-cadence ceiling are **PROVISIONAL** until the first real capture, in the
same convention `tilt_cal_grid.py` uses.

### catch_sim_test.py

Offline catch simulation that exercises the full pure-Python pipeline: `BallTracker` (Kalman filter + marker matching) → `CatchCoordinator` (catch pose + hand commands) → `TrajectoryManager` (async feasibility + trajectory execution). No ROS2 or hardware required.

**3D visualization** (`--viz`): animated Stewart platform with ball trajectory, KF estimate, landing prediction, hand state indicator, landing error chart, and event timeline. Supports pause/step (Space, arrow keys).

### throw_catch_test.py

Throw-catch cycle: platform throws a ball to itself ((-100,0,170) → (100,0,170) mm), repositions during flight, and catches. Validates the full pipeline including hand throw/catch trajectory timing, platform tilt for throw, and hand catch offset. Reuses the `catch_sim_test.py` animated 3D visualizer.

```bash
python tools/throw_catch_test.py           # text-only results
python tools/throw_catch_test.py --viz     # animated 3D playback
python tools/throw_catch_test.py --viz --apex 500   # lower throw (500mm apex)
```

### catch_sim_test.py

Offline catch simulation that exercises the full pure-Python pipeline: `BallTracker` (Kalman filter + marker matching) → `CatchCoordinator` (catch pose + hand commands) → `TrajectoryManager` (async feasibility + trajectory execution). No ROS2 or hardware required.

**3D visualization** (`--viz`): animated Stewart platform with ball trajectory, KF estimate, landing prediction, hand state, leg extensions, and event timeline.

```bash
python tools/catch_sim_test.py --viz                           # Default throw
python tools/catch_sim_test.py --viz --landing 50 -30          # Specific landing XY
python tools/catch_sim_test.py --viz --landing 0 0 --launch 300 -300 900  # Custom launch
python tools/catch_sim_test.py --landing 0 0 --tof 1.0        # Longer flight
python tools/catch_sim_test.py --viz --noise 10                # High mocap noise
python tools/catch_sim_test.py --sweep                         # Noise sweep (text table)

# Playback controls (--viz):
#   SPACE       play/pause       . / ,     step +1/-1
#   ] / [       step +10/-10     0 / 9     first/last frame
```
