# Operations Guide

This page covers how to work with the motion planner in practice — running tests, tuning ODrive gains, using the test harnesses, and extending the system.

## Running the Control Process

### On the Jetson (Production)

```bash
# Start the control loop (separate terminal from ROS2)
python -m jugglebot.motion.control_loop --rate 500 --log-level INFO
```

The control loop starts in `DISABLED` mode. It will connect to the bridge's ZeroMQ socket automatically and wait for an `enable` command.

### Offline Testing (No Hardware)

The motion modules have no ROS2 dependency and can be used directly in Python:

```python
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.trajectory import (
    make_rest_to_rest, check_feasibility, evaluate
)
import numpy as np

geom = StewartGeometry()
params = DynamicsParams.from_config()

# Create a trajectory
traj = make_rest_to_rest(
    end_pose=np.array([0, 0, 30, 0, 0, 0]),  # 30mm up
    duration=1.0,
    speed_scale=0.5
)

# Check feasibility
result = check_feasibility(traj, geom, params)
print(f"Feasible: {result.feasible}")
if not result.feasible:
    for v in result.violations:
        print(f"  - {v}")

# Evaluate at specific times
for t in np.linspace(0, traj.duration, 50):
    pose, twist, accel = evaluate(traj, t)
    print(f"t={t:.3f}s  z={pose[2]:.1f}mm  vz={twist[2]:.1f}mm/s")
```

## Running the Test Suites

### Offline Tests (No Hardware Required)

```bash
# Kinematics verification (6 tests)
python -m pytest ros_ws/src/jugglebot/jugglebot/motion/tests/test_kinematics.py -v

# Control loop tests (3 tests)
python -m pytest ros_ws/src/jugglebot/jugglebot/motion/tests/test_control_loop.py -v

# Dynamics tests (7 tests)
python -m pytest ros_ws/src/jugglebot/jugglebot/motion/tests/test_dynamics.py -v

# Trajectory tests (7 tests)
python -m pytest ros_ws/src/jugglebot/jugglebot/motion/tests/test_trajectory.py -v

# Async pipeline tests (8 tests)
python -m pytest ros_ws/src/jugglebot/jugglebot/motion/tests/test_async_pipeline.py -v
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
| `tools/single_leg_test.py` | Phase 2/3A isolated leg tests | N/A |
| `tools/free_platform_test.py` | Phase 3C platform tests | Low |
| `tools/trajectory_test.py` | Phase 4 trajectory tracking (T1–T4) | ≤25% |
| `tools/hardening_test.py` | Phase 6 workspace, endurance (H1–H6) | 25–100% |
| `tools/dynamic_target_test.py` | Phase 7 dynamic targets (DT1–DT5) | 100% |

#### Trajectory Viewer

A standalone 3D matplotlib viewer for previewing trajectories before execution:

```bash
# Standalone viewer
python tools/trajectory_viewer.py

# Preview mode (used by trajectory_test.py)
python tools/trajectory_test.py --preview
```

## Trajectory Visualization

### Pre-Execution Preview

Before running a trajectory on hardware, preview it offline. The `tools/trajectory_viewer.py` provides a 3D matplotlib visualization of the Stewart platform moving through a trajectory.

For Cartesian and joint-space analysis, you can plot trajectories programmatically:

```python
import matplotlib.pyplot as plt
import numpy as np
from jugglebot.motion.trajectory import make_rest_to_rest, evaluate, check_feasibility
from jugglebot.motion.ik_solver import pose_to_leg_lengths, rotvec_to_rot_matrix
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams

geom = StewartGeometry()
params = DynamicsParams.from_config()
traj = make_rest_to_rest(np.array([20, 0, 30, 0.05, 0, 0]), duration=1.5)

times = np.linspace(0, traj.duration, 200)
poses = np.array([evaluate(traj, t)[0] for t in times])
twists = np.array([evaluate(traj, t)[1] for t in times])

# Cartesian space
fig, axes = plt.subplots(2, 3, figsize=(12, 6), sharex=True)
labels = ['X (mm)', 'Y (mm)', 'Z (mm)', 'RX (rad)', 'RY (rad)', 'RZ (rad)']
for i, (ax, label) in enumerate(zip(axes.flat, labels)):
    ax.plot(times, poses[:, i])
    ax.set_ylabel(label)
axes[-1, 1].set_xlabel('Time (s)')
plt.suptitle('Cartesian Trajectory')
plt.tight_layout()

# Joint space (leg extensions)
extensions = np.array([
    pose_to_leg_lengths(p[:3], rotvec_to_rot_matrix(p[3:6]), geom)
    for p in poses
])
fig2, ax2 = plt.subplots(figsize=(10, 4))
for i in range(6):
    ax2.plot(times, extensions[:, i], label=f'Leg {i}')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Extension (mm)')
ax2.legend()
ax2.set_title('Leg Extensions')
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

### Adding a New Trajectory Type

To add a new trajectory creation function:

1. Add the function to `trajectory.py` alongside `make_rest_to_rest()`
2. Ensure it returns a `QuinticTrajectory` (via `create_trajectory()`)
3. Always run `check_feasibility()` before submitting to hardware
4. Add an IPC message constructor to `ipc.py` if the bridge needs to pass parameters

### Adding a New Input Source

To add a new control source (e.g., joystick, ball predictor):

1. Create a ROS2 node that publishes `PlatformPoseCommand` messages on `platform_pose_topic`
2. Set the `publisher` field in each message to your source name (e.g., `JOYSTICK`)
3. Register the source name in the bridge's mode handling (the `_on_control_mode` method)
4. The orchestrator publishes mode changes on `control_mode_topic` — add your mode to its state machine

### Adding New Telemetry Fields

1. Add the field to `make_telemetry()` in `ipc.py`
2. Populate it in `_publish_telemetry()` in `control_loop.py`
3. Extract it in `_poll_telemetry()` in `motion_bridge_node.py`
4. Publish on a ROS2 topic if needed

### Adding New Safety Checks

1. Add limit constants to `workspace.py` (follow the existing `LEG_SOFT_MARGIN_MM` pattern)
2. Add the check to `check_workspace_limits()` for runtime enforcement
3. Add a corresponding check to `check_feasibility()` in `trajectory.py` for planning-time enforcement
4. **Critical:** Ensure both use the same margins so planning and runtime agree

## Common Patterns

### Previewing a Trajectory Offline

```python
import matplotlib.pyplot as plt
from jugglebot.motion.trajectory import make_rest_to_rest, evaluate, check_feasibility
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.dynamics import DynamicsParams
import numpy as np

geom = StewartGeometry()
params = DynamicsParams.from_config()

traj = make_rest_to_rest(
    end_pose=np.array([20, 0, 30, 0.05, 0, 0]),
    duration=1.5
)

result = check_feasibility(traj, geom, params)
print(f"Feasible: {result.feasible}")
print(f"Peak vel: {result.peak_leg_vel_rps.max():.2f} rev/s")
print(f"Peak accel: {result.peak_leg_accel_rps2.max():.2f} rev/s²")
print(f"Peak cond: {result.peak_condition_number:.0f}")

# Plot
times = np.linspace(0, traj.duration, 200)
poses = np.array([evaluate(traj, t)[0] for t in times])

fig, axes = plt.subplots(2, 3, figsize=(12, 6))
labels = ['X (mm)', 'Y (mm)', 'Z (mm)', 'RX (rad)', 'RY (rad)', 'RZ (rad)']
for i, (ax, label) in enumerate(zip(axes.flat, labels)):
    ax.plot(times, poses[:, i])
    ax.set_ylabel(label)
    ax.set_xlabel('Time (s)')
plt.tight_layout()
plt.show()
```

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

### Control loop won't start

- Check that the bridge node is running (the control loop connects to port 5555)
- Check ZeroMQ is installed: `pip install pyzmq msgpack`
- Check for port conflicts: another process may be bound to 5555 or 5556

### E-stop on first enable

- The heartbeat timeout is 0.5 seconds. If the bridge isn't publishing messages fast enough, the control loop will E-stop. Ensure the bridge's poll timer is running.

### Trajectory rejected as infeasible

- Check `result.violations` for specific constraint violations
- Common causes: trajectory too fast (reduce speed_scale), poses near workspace boundary, extreme tilts approaching singularity
- Try `find_min_feasible_duration()` to find the shortest feasible duration

### Tracking error too high

- Check ODrive gains (may need tuning for loaded platform vs. bench)
- Check that `vel_ff` and `torque_ff` are being sent (not zeros)
- Check for mechanical issues (Leg 2 was consistently the worst tracker in hardware tests — a mechanical issue, not a software one)

### 18ms startup spike

Every hardware run shows an ~18 ms first-sample loop spike. This is a startup transient (JIT, memory allocation) and is not correlated with tracking error. It does not indicate a problem.
