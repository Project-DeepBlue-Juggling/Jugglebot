# Running the Simulation

This page covers how to install, run, and analyze the MuJoCo simulation.

**Entry point:** `sim/main.py`

## Installation

From the repository root:

```bash
cd sim
python -m venv .venv
source .venv/bin/activate      # Linux/macOS
# .venv\Scripts\activate       # Windows

pip install -r requirements.txt
```

### Dependencies

| Package | Version | Purpose |
|---|---|---|
| `mujoco` | >= 3.0 | Physics simulation |
| `casadi` | >= 3.6 | NLP solver (IPOPT backend) |
| `numpy` | >= 1.24 | Numerics |
| `matplotlib` | >= 3.7 | Plotting (analysis scripts) |
| `pytest` | >= 7.0 | Test runner |
| `pyyaml` | >= 6.0 | Config loading (StewartGeometry) |
| `pyspacemouse` | >= 0.6 | SpaceMouse input (Linux only, optional) |

!!! note
    CasADi bundles IPOPT — no separate IPOPT installation is needed.

## Quick Start

All commands are run from the `sim/` directory.

```bash
# Simplest MPC run: move platform to z+50mm
python main.py --mpc --pose 0,0,50,0,0,0

# Same thing, headless (no viewer window)
python main.py --mpc --pose 0,0,50,0,0,0 --no-viewer --duration 5

# Direct IK command (no MPC — step change, no trajectory planning)
python main.py --pose 0,0,50,0,0,0
```

Every run writes a telemetry CSV to `sim/logs/` and prints a summary of tracking error and solve times.

## Operating Modes

### Static Pose

Command the platform to a fixed pose. With `--mpc`, the controller plans a smooth approach. Without it, the plant receives a step command (useful for checking the MuJoCo model).

```bash
# MPC plans smooth motion to z+50mm
python main.py --mpc --pose 0,0,50,0,0,0

# With tilt: z+80mm, 5deg pitch, -3deg roll
python main.py --mpc --pose 0,0,80,0.087,-0.052,0
```

### Pose Sequence

Multiple poses triggered at specific times. Format: `x,y,z,rx,ry,rz@time` separated by spaces.

```bash
python main.py --mpc --sequence "0,0,50,0,0,0@0.5 0,0,0,0,0,0@2.0"
```

### Scripted Trajectories (T1-T6)

Pre-defined waypoint sequences with arrival times. The MPC plans optimal motion between waypoints.

```bash
python main.py --trajectory T1     # Linear: home -> z+50 -> home
python main.py --trajectory T2     # Circular orbit (80mm radius, z=50)
python main.py --trajectory T3     # Multi-axis translation + tilt
python main.py --trajectory T4     # Speed test (~400ms transit)
python main.py --trajectory T5     # Grand tour (10 workspace poses)
python main.py --trajectory T6     # Extreme tour (large sweeping motions)
```

| Trajectory | Motion | Duration |
|---|---|---|
| T1 | Linear Z translation (home → +50mm → home) | ~3 s |
| T2 | Circular XY orbit at z=50mm, 80mm radius | ~6.5 s |
| T3 | Multi-axis translation + tilt, 3 segments | ~5.5 s |
| T4 | Fast point-to-point (~400ms transit) | ~3 s |
| T5 | Grand tour — 10 poses spanning the workspace | ~14 s |
| T6 | Extreme tour — large sweeping motions | ~14 s |

### Scripted Catch (DT1-DT8, BB1-BB4)

Ball is spawned at a specified position and velocity. The MPC drives the platform to the predicted landing pose while the hand executes a catch trajectory.

```bash
python main.py --catch DT1        # Vertical drop, centred
python main.py --catch DT5        # Off-centre with lateral velocity
python main.py --catch DT8        # Deliberate miss (ball too far)
python main.py --catch BB1        # Ball Butler throw (angled)
```

| Sequence | Description |
|---|---|
| DT1-DT5 | Vertical/angled drops at various positions and speeds |
| DT6-DT7 | Edge cases (timing, velocity) |
| DT8 | Ball miss — tests graceful failure and return home |
| BB1-BB4 | Ball Butler throws (angled trajectories from external launcher) |

### Scripted Throw-Catch (TC1-TC4)

Full throw-catch cycle: the platform throws a ball upward with the hand, then repositions to catch it.

```bash
python main.py --throw-catch TC1   # Basic self-throw-catch
python main.py --throw-catch TC4   # Offset catch position
```

### Interactive Catch

Spawn balls manually with keyboard controls. The MPC and hand coordinator handle the catch autonomously.

```bash
python main.py --interactive-catch

# With Ball Butler launcher (adds T key for machine throws)
python main.py --interactive-catch --bb
```

**Keyboard controls:**

| Key | Action |
|---|---|
| B | Spawn ball (or Ball Butler throw if `--bb`) |
| `` ` `` (grave) | Spawn ball with current preset (when `--bb`) |
| N / M | Next / previous spawn preset |
| 1-5 | Select preset directly |
| PgUp / PgDn | Spawn height +/- 200mm |
| Left / Num0 | Spawn X offset +/- 20mm |
| Num1 / Num3 | Spawn Y offset +/- 20mm |
| Num8 / Num2 | Spawn Vz +/- 200mm/s |
| Num4 / Num6 | Spawn Vx +/- 50mm/s |
| Num7 / Num9 | Spawn Vy +/- 50mm/s |

### Continuous Juggle

Self-throw-catch loop with real-time parameter adjustment. The platform throws a ball, repositions, catches it, and repeats.

```bash
python main.py --juggle
```

**Keyboard controls:**

| Key | Action |
|---|---|
| T | Toggle editing: throw position vs catch position |
| Left / Num0 | Selected pos X +/- 20mm |
| Num1 / Num3 | Selected pos Y +/- 20mm |
| PgUp / PgDn | Ball height +/- 20mm |
| F / G | Flight time -/+ 0.05s |
| B | Reset ball (on drop or force-reset) |

### Interactive Input (SpaceMouse / Keyboard)

Continuous ASAP target from a physical input device. The MPC smoothly tracks the moving target.

```bash
python main.py --spacemouse       # SpaceMouse (Linux only)
python main.py --keyboard         # Keyboard (cross-platform)
```

## Common Options

These flags work with any mode:

| Flag | Default | Effect |
|---|---|---|
| `--no-viewer` | off | Run headless (no MuJoCo window) |
| `--duration N` | varies | Simulation duration in seconds |
| `--dashboard` | off | Start live telemetry dashboard |
| `--dashboard-port N` | 8082 | Dashboard HTTP port |
| `--log-dir PATH` | `sim/logs/` | Telemetry CSV output directory |

### Viewer Controls

Available in all viewer modes:

| Key | Action |
|---|---|
| Space | Pause / unpause |
| Right arrow | Step one frame (while paused) |
| Up / Down arrow | Speed x2 / x0.5 |
| R | Reset speed to 1x |

### Live Dashboard

Add `--dashboard` to any MPC mode to start a browser-based telemetry viewer:

```bash
python main.py --mpc --trajectory T5 --dashboard
```

Then open [http://localhost:8082](http://localhost:8082) in a browser. The dashboard shows real-time plots of platform pose, tracking error, leg extensions, solve time, and hand state. Data is pushed via Server-Sent Events (auto-reconnects if the page is refreshed).

## Telemetry Output

Every run writes a CSV to `sim/logs/` with the naming pattern `{mpc|sim}_{timestamp}.csv`.

### CSV Columns

Each row is one control step (50 Hz / 20ms):

| Group | Columns | Units |
|---|---|---|
| Time | `time` | s |
| Reference | `ref_pose_{x,y,z,rx,ry,rz}`, `ref_twist_{vx,vy,vz,wx,wy,wz}` | mm, rad, mm/s, rad/s |
| Actual | `actual_pose_{x,y,z,rx,ry,rz}`, `actual_twist_{vx,vy,vz,wx,wy,wz}` | mm, rad, mm/s, rad/s |
| Commands | `cmd_ext_{0-5}` | mm |
| Sensors | `actual_ext_{0-5}`, `leg_vel_{0-5}` | mm, mm/s |
| Hand | `hand_cmd_mm`, `hand_pos_mm`, `hand_vel_mmps` | mm, mm/s |
| MPC | `solve_time_ms`, `solve_status`, `cost`, `constraint_violation` | ms, -, -, mm |
| Derived | `tracking_error_mm`, `tracking_error_deg` | mm, deg |

### Analyzing Telemetry

Load a CSV with pandas or numpy for post-hoc analysis:

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('sim/logs/mpc_20260321_143000.csv')

fig, axes = plt.subplots(3, 1, sharex=True)
axes[0].plot(df.time, df.ref_pose_z, '--', label='ref')
axes[0].plot(df.time, df.actual_pose_z, label='actual')
axes[0].set_ylabel('Z (mm)')
axes[0].legend()

axes[1].plot(df.time, df.tracking_error_mm)
axes[1].set_ylabel('Error (mm)')

axes[2].plot(df.time, df.solve_time_ms)
axes[2].set_ylabel('Solve (ms)')
axes[2].set_xlabel('Time (s)')

plt.tight_layout()
plt.show()
```

## Running Tests

The test suite covers MPC, plant, hand trajectories, ball physics, and integration:

```bash
cd sim
python -m pytest tests/ -v
```

### Test Files

| Test File | Coverage |
|---|---|
| `test_mpc_static.py` | MPC static pose tracking (cold start, warm start, convergence) |
| `test_mpc_trajectory.py` | MPC waypoint tracking (T1-T4 trajectories) |
| `test_mpc_dynamic.py` | MPC dynamic targets (catch sequences DT1-DT5) |
| `test_variable_horizon.py` | Variable-resolution horizon and urgency system |
| `test_plant.py` | MuJoCo plant: sensors, commands, coordinate conversions |
| `test_model.py` | MJCF model generation and validation |
| `test_hand.py` | Hand coordinator state machine |
| `test_hand_trajectory.py` | Catch/throw trajectory math (Teensy port) |
| `test_ball.py` | Ball manager: spawn, capture, release |
| `test_ballistics.py` | Parabolic ball flight prediction |
| `test_planner.py` | Throw-catch plan generation |
| `test_throw_trajectory.py` | Throw trajectory integration |
| `test_post_catch.py` | Post-catch behavior (retract, return home) |
| `test_target_interface.py` | TargetSource adapters |
| `test_ball_butler_sim.py` | Ball Butler simulator |

### Running Specific Tests

```bash
# Single test file
python -m pytest tests/test_mpc_static.py -v

# Single test case
python -m pytest tests/test_mpc_static.py::test_cold_start -v

# Only MPC tests
python -m pytest tests/ -v -k "mpc"

# With output (see print statements)
python -m pytest tests/test_mpc_dynamic.py -v -s
```

## Standalone Demo

The `demo_mpc.py` script runs a fixed pose sequence without the full `main.py` infrastructure — useful for quick MPC validation:

```bash
python demo_mpc.py                # with viewer
python demo_mpc.py --no-viewer    # headless
python demo_mpc.py --dashboard    # with live dashboard
```

It commands 7 poses over 10 seconds and prints tracking statistics at the end.
