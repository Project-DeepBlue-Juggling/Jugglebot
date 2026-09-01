# Operations Guide

!!! note "Removed 2026-09-01"
    The MPC chain was **deleted** on 2026-09-01:
    `controller/{mpc,params,runner,hardware_plant,generate_solver}.py`, `run_mpc.py`,
    the ROS2 `motion_bridge_node` / `mpc_bridge_node`, and `sim/main.py`'s `--mpc`
    and `--hardware` modes. Every procedure below that invokes them is **historical**
    and resolves only at git tag **`mpc-final`**. See
    `logbook/2026-09-01-mpc-chain-removed.md`.

This page covers how to work with the motion planner in practice — running tests, tuning ODrive gains, using the test harnesses, and extending the system.

## Running the Motor Guard

### On the Jetson (Production)

```bash
# historical — removed 2026-09-01 (tag mpc-final); sim/main.py has no --hardware mode
# Start the motor guard (separate terminal from ROS2)
python -m jugglebot.motion.motor_guard --rate 500 --log-level INFO

# Start the MPC process (separate terminal)
python sim/main.py --hardware
```

The motor guard starts in `DISABLED` mode. It connects to the bridge's ZeroMQ socket automatically and waits for an `enable` command. The MPC process defaults to `ZmqTargetSource`, which receives targets from `mpc_bridge_node` via ZMQ :5558.

### Offline Testing (No Hardware)

The motion modules have no ROS2 dependency and can be used directly in Python:

```python
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.motor_commands import cartesian_to_motor_commands
import numpy as np

geom = StewartGeometry()
params = DynamicsParams.from_config()

# Compute motor commands for a given pose
pose_6dof = np.array([0, 0, 30, 0, 0, 0])  # 30mm up
twist_6dof = np.zeros(6)
accel_6dof = np.zeros(6)

pos_rev, vel_ff_rps, torque_ff_Nm = cartesian_to_motor_commands(
    pose_6dof, twist_6dof, accel_6dof, geom, params
)
```

## Running the Test Suites

### Offline Tests (No Hardware Required)

```bash
# Motor guard tests (28 tests)
pytest tests/motion/test_motor_guard.py -v

# Kinematics verification (6 tests)
pytest tests/motion/test_kinematics.py -v

# Dynamics tests (13 tests)
pytest tests/motion/test_dynamics.py -v

# MPC tests (59 tests)
pytest tests/sim/ -v
```

### Hardware Test Harnesses

The `tools/` directory contains standalone test harnesses that bypass ROS2 and communicate directly with the ODrives over CAN. These are used for hardware validation and debugging.

!!! warning "Hardware test prerequisites"
    - ODrives must be powered and CAN bus connected
    - Motors must be homed before most tests
    - Current limits should be set conservatively (50% rated = 10A)
    - Always preview trajectories offline before executing on hardware

#### PlatformTestHarness

The base harness class in `free_platform_test.py`:

```python
from tools.free_platform_test import PlatformTestHarness

# Pattern A: Context manager (auto-connects on entry, auto-disconnects on exit)
with PlatformTestHarness() as harness:
    harness.home_all_axes()
    # ... run tests ...

# Pattern B: Explicit connect/disconnect
harness = PlatformTestHarness()
harness.connect()
harness.home_all_axes()
# ... run tests ...
harness.disconnect()
```

!!! danger "CAN bus is NOT opened in `__init__`"
    `PlatformTestHarness` does **not** open the CAN bus in `__init__`. If using Pattern B, you **must** call `harness.connect()` before any hardware operations. The context manager (Pattern A) handles this automatically via `__enter__`. This is a deliberate safety measure.

#### Available Harnesses

| File | Tests | Speed |
|---|---|---|
| `tests/hardware/single_leg_test.py` | Phase 2/3A isolated leg tests | N/A |
| `tests/hardware/free_platform_test.py` | Phase 3C platform tests | Low |
| `tests/hardware/supported_platform_test.py` | Supported platform tests | Low |

!!! note "Removed pre-MPC test harnesses"
    Several harnesses (`trajectory_test.py`, `hardening_test.py`, `dynamic_target_test.py`, `trajectory_viewer.py`) depended on the pre-MPC quintic trajectory system and were deleted with `tests/archived/` in commit 67889a6 (2026-04-17) — there is no `tools/archived/` directory. Their test scenarios remain valid references for future MPC hardware tests; recover a file with `git show 67889a6^:tests/archived/<name>`.

## Motion Preview

### MPC Simulation Preview

The canonical way to preview motion before running on hardware is through the MPC simulation:

```bash
# historical — removed 2026-09-01 (tag mpc-final); sim/main.py has no --mpc or --keyboard mode
# Preview a static pose target
python sim/main.py --mpc --pose 20,0,30,0.05,0,0 --no-viewer --duration 3

# Preview with the 3D viewer
python sim/main.py --mpc --pose 20,0,30,0.05,0,0 --duration 5

# Preview with the telemetry dashboard (http://localhost:8082)
python sim/main.py --dashboard --mpc --pose 20,0,30,0.05,0,0

# Keyboard-controlled interactive preview
python sim/main.py --keyboard --mpc
```

### Joint-Space Analysis

To plot leg extensions for a given pose (without running the MPC):

```python
import matplotlib.pyplot as plt
import numpy as np
from jugglebot.motion.ik_solver import pose_to_leg_lengths, rotvec_to_rot_matrix
from jugglebot.motion.geometry import StewartGeometry

geom = StewartGeometry()

# Sweep Z from 0 to 50 mm
z_values = np.linspace(0, 50, 100)
poses = np.array([[0, 0, z, 0, 0, 0] for z in z_values])

extensions = np.array([
    pose_to_leg_lengths(p[:3], rotvec_to_rot_matrix(p[3:6]), geom)
    for p in poses
])

fig, ax = plt.subplots(figsize=(10, 4))
for i in range(6):
    ax.plot(z_values, extensions[:, i], label=f'Leg {i}')
ax.set_xlabel('Z displacement (mm)')
ax.set_ylabel('Extension (mm)')
ax.legend()
ax.set_title('Leg Extensions vs Z')
plt.tight_layout()
plt.show()
```

### Post-Session Analysis (Rosbag)

When recording is enabled (`ros2 launch jugglebot jugglebot_launch.py record:=true`), rosbag captures topics in MCAP format to `~/Desktop/rosbags/`. Recorded topics include `/leg_lengths_topic` (commanded motor positions, velocities, torques) and `/platform_pose_topic` (commanded Cartesian poses).

To play back and analyze a recording, use the `rosbags` or `mcap` Python packages:

```bash
pip install rosbags
```

!!! note "Motor feedback not yet in rosbag"
    Currently, actual motor encoder positions (feedback) are not published to a ROS2 topic by the bridge — they stay in the IPC layer. This means rosbag captures **commanded** trajectories but not **measured** ones. Adding a `/motor_feedback` topic to the bridge would close this gap. See `MINOR_UPDATES.md` for the proposed implementation.

## ODrive Gain Tuning

The ODrive controllers have three tunable gains set via CAN:

| Gain | Units | Default | Purpose |
|---|---|---|---|
| `pos_gain` | 1/s | 40 | Position error → velocity command |
| `vel_gain` | Nm·s/rev | 0.2 | Velocity error → current command |
| `vel_int_gain` | Nm/rev | 0.32 | Eliminates steady-state velocity error |

### Tuning Procedure

1. **Start with defaults:** The baseline gains (`pos_gain=40`, `vel_gain=0.2`, `vel_int_gain=0.32`) were established during Phase 3A on an isolated leg.

2. **Tune on isolated leg first:** Before tuning on the assembled platform, verify behaviour on a single bench-mounted leg.

3. **Step response:** Command a small position step and observe:
   - **Overshoot:** Reduce `pos_gain` or increase `vel_gain`
   - **Sluggish response:** Increase `pos_gain`
   - **Oscillation:** Reduce `pos_gain` and/or increase `vel_gain`
   - **Audible vibration:** Reduce `vel_gain`

4. **Trajectory tracking:** With gains set, run trajectory tests at low speed. Increasing `vel_gain` improves tracking but increases sensitivity to noise.

5. **Velocity integrator:** Keep `vel_int_gain` low for trajectory tracking (the setpoint updates every cycle). Increase only if steady-state holding accuracy at rest needs improvement.

### Setting Gains via IPC

Gains are set via the `set_gains` mode command through the IPC layer. The CAN node sends `set_pos_gain` and `set_vel_gains` CAN commands to the ODrives.

## Extending the System

### Adding a New Motion Mode

All motion modes route through the MPC. To add a new mode (e.g., joystick, automated sequence):

1. Create a `TargetSource` implementation in `sim/input/` (implements `update(sim_time, state) → TargetCommand`)
2. For hardware: create a ROS2 node that publishes targets, and the `mpc_bridge_node` forwards them to the MPC via ZMQ :5558
3. Register the mode in the orchestrator's state machine (`state_machine.py`)
4. The MPC handles trajectory planning, feasibility, and smoothing automatically — you only provide target poses

### Adding a New Input Source (ROS2 Side)

On the ROS2 side, a new input source publishes target poses that the `mpc_bridge_node` forwards to the MPC:

1. Create a ROS2 node that publishes `PlatformPoseCommand` messages on `platform_pose_topic`
2. Set the `publisher` field in each message to your source name (e.g., `JOYSTICK`)
3. Register the source name in `mpc_bridge_node`'s mode handling (the `_on_control_mode` method)
4. Add the mode to the orchestrator's state machine (`state_machine.py`)

### Adding New Telemetry Fields

1. Add the field to the pre-allocated `_telem_msg` dict in `motor_guard.py` `__init__()`
2. Populate it in `_publish_telemetry()` in `motor_guard.py`
3. Extract it in `_poll_telemetry()` in `motion_bridge_node.py`
4. Publish on a ROS2 topic if needed

### Adding New Safety Checks

1. Add limit constants to `workspace.py` (follow the existing `LEG_SOFT_MARGIN_MM` pattern)
2. Add the check to `check_workspace_limits()` for runtime enforcement
3. Add the check to `_on_mpc_command()` in `motor_guard.py` for command-arrival validation
4. **Critical:** Ensure workspace check margins match the MPC solver's constraints

## Common Patterns

### Previewing Motion via MPC Simulation

The MPC simulation is the canonical way to preview motion offline. Run with `--no-viewer` for headless mode or `--dashboard` for real-time telemetry:

```bash
# historical — removed 2026-09-01 (tag mpc-final); sim/main.py has no --mpc mode
# Quick check: does the MPC reach this pose smoothly?
python sim/main.py --mpc --pose 20,0,30,0.05,0,0 --no-viewer --duration 3

# Full dashboard with telemetry plots
python sim/main.py --dashboard --mpc --pose 20,0,30,0.05,0,0
```

The simulation runs the same MPC solver and `HardwarePlant` code path used on hardware, so motion behavior is representative.

### Computing Feedforward Torques at a Pose

```python
from jugglebot.motion.dynamics import compute_full_feedforward_torques
from jugglebot.motion.ik_solver import rotvec_to_rot_matrix
import numpy as np

pos = np.array([0, 0, 0])
rot = rotvec_to_rot_matrix(np.array([0.05, 0, 0]))  # 0.05 rad X tilt
twist = np.zeros(6)
accel = np.zeros(6)

torques = compute_full_feedforward_torques(pos, rot, twist, accel, geom, params)
print(f"Per-leg gravity torques: {torques}")
print(f"Total: {torques.sum():.4f} Nm")
```

### Checking Workspace at a Pose

```python
from jugglebot.motion.workspace import (
    WorkspaceLimits, check_workspace_limits, compute_condition_number
)
from jugglebot.motion.ik_solver import pose_to_leg_lengths

limits = WorkspaceLimits.from_geometry(geom)
extensions = pose_to_leg_lengths(pos, rot, geom)
cond = compute_condition_number(pos, rot, geom)

check = check_workspace_limits(extensions, cond, limits)
print(f"Status: {check.status}")
print(f"Speed scale: {check.speed_scale}")
if check.violations:
    for v in check.violations:
        print(f"  - {v}")
```

## Troubleshooting

### Motor guard won't start

- Check that the bridge node is running (the motor guard connects to port 5555)
- Check ZeroMQ is installed: `pip install pyzmq msgpack`
- Check for port conflicts: another process may be bound to 5555, 5556, or 5557

### E-stop on first enable

- The heartbeat timeout is 0.5 seconds. If the bridge isn't publishing messages fast enough, the motor guard will E-stop. Ensure the bridge's poll timer is running.
- The MPC staleness timeout is 200 ms. If the MPC process isn't running, the motor guard will E-stop after enable.

### MPC solver fails or returns poor trajectories

- Check the MPC solver status in telemetry — `Solve_Succeeded` or `Solved_To_Acceptable_Level` are normal
- Common causes of solver failure: target pose outside workspace, extreme tilts approaching singularity, target velocity too high
- Check MPC tuning parameters in `controller/params.py` — cost weights and horizon length affect trajectory quality
- Ran `sim/main.py --mpc --pose <target>` to reproduce the issue in simulation before debugging on hardware (historical — both `controller/params.py` and the `--mpc` mode were removed 2026-09-01, tag `mpc-final`)

### Tracking error too high

- Check ODrive gains (may need tuning for loaded platform vs. bench)
- Check that `vel_ff` and `torque_ff` are being sent (not zeros)
- Check for mechanical issues (Leg 2 was consistently the worst tracker in hardware tests — a mechanical issue, not a software one)

### 18ms startup spike

Every hardware run shows an ~18 ms first-sample loop spike. This is a startup transient (JIT, memory allocation) and is not correlated with tracking error. It does not indicate a problem.
