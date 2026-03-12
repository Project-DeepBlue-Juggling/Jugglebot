# tools/ — Standalone Hardware Test & Debug Utilities

Scripts in this directory bypass ROS2 and talk directly to hardware via python-can. They are designed for bench testing, bring-up, and diagnostics.

## Preview Convention (`--preview`)

**All test harnesses that accept `--preview` must follow these rules:**

1. **Single window** — `--preview` produces exactly one matplotlib figure containing the full test sequence. Never open a separate window per test.

2. **Unified timeline** — All tests in the suite are stitched end-to-end on a shared time axis. The sequence must be C2-continuous throughout: both *during* and *between* individual tests, all commanded values must be smooth (no step discontinuities in position, velocity, or acceleration).

3. **Return-to-home transitions** — Between tests, generate a proper profiled return-to-home move (not a step command). Highlight these transition segments visually (e.g. light blue `axvspan` shading) so they are clearly distinguishable from test segments.

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

### Safety

All test harnesses share these safety principles:
- Conservative current limit (50% of rated) set before any motion
- Universal IDLE on completion, error, or Ctrl-C
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
