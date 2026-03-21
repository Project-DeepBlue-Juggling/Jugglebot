# tools/ — Standalone Hardware Test & Debug Utilities

Scripts in this directory bypass ROS2 and talk directly to hardware via python-can. They are designed for bench testing, bring-up, and diagnostics.

## Preview Convention (`--preview`)

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

Reference: `smoother_test.py:safe_shutdown()`, `dynamic_target_test.py:safe_idle_all()`.

The `--preview` plot must also show the stow segment at the end of the timeline so the operator sees the full motion the robot will execute.

### Safety

All test harnesses share these safety principles:
- Conservative current limit (50% of rated) set before any motion
- Always stow before idling (see above)
- Mandatory feasibility pre-check before hardware execution
- Interactive confirmation before each test stage
- Heartbeat watchdog on all axes

## Test Harnesses

| Script | Phase | Description |
|--------|-------|-------------|
| `single_leg_test.py` | 2 | Single-axis bench tests (torque, e-stop, encoder, force) |
| `supported_platform_test.py` | 3A | Position control bench tests on a single supported leg |
| `free_platform_test.py` | 3B-C | Platform-level tests; defines `PlatformTestHarness` |
| `trajectory_test.py` | 4 | Quintic trajectory hardware validation (T1-T4) |
| `inertia_test.py` | 5 | Inertia feedforward comparison tests (T5-T7) |
| `hardening_test.py` | 6 | Workspace boundary, fault injection, endurance (H1-H6) |
| `dynamic_target_test.py` | 7 | Dynamic target acceptance/tracking (DT1-DT5) |
| `juggling_test.py` | 7+ | Ball-catching juggling pattern tests |
| `smoother_test.py` | — | StreamSmoother validation (S1-S5, direct-target smoothing) |
| `trajectory_viewer.py` | — | 3D Stewart platform viewer + trajectory plot utility |
| `catch_sim_test.py` | 6 | Offline catch simulation (no hardware). Full tracking + coordinator + hand control pipeline with 3D viewer |
| `throw_catch_test.py` | 6 | Throw-catch cycle test: platform throws ball to itself, repositions, catches. Reuses catch_sim_test visualizer |
| `tracking_analyzer.py` | 6 | Rosbag analysis for ball tracking data |

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
