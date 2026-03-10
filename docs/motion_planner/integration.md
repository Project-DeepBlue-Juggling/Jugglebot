# System Integration

This page describes how the motion planner connects to the rest of the Jugglebot system — the ROS2 bridge, the CAN interface, and how the full system starts up and shuts down.

**Source files:**

- [motion_bridge_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py) — ROS2 bridge
- [can_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/can_node.py) — CAN interface
- [orchestrator_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/orchestrator_node.py) — State machine coordinator

## System Architecture

```
┌───────────────────────────────────────────────────────────┐
│                     ROS2 Process                          │
│                                                           │
│  ┌──────────────┐   ┌──────────────┐   ┌──────────────┐  │
│  │ orchestrator  │   │  spacemouse  │   │    mocap     │  │
│  │   _node.py   │   │  _handler.py │   │ _interface   │  │
│  └──────┬───────┘   └──────┬───────┘   └──────┬───────┘  │
│         │                  │                   │          │
│         ▼                  ▼                   ▼          │
│  ┌─────────────────────────────────────────────────────┐  │
│  │              motion_bridge_node.py                  │  │
│  │                                                     │  │
│  │  ROS2 Topics ←──→ ZeroMQ IPC ←──→ Telemetry Pub   │  │
│  └─────────────────────┬───────────────────────────────┘  │
│                        │ ZeroMQ                           │
│  ┌─────────────────────┴───────────────────────────────┐  │
│  │                  can_node.py                        │  │
│  │                                                     │  │
│  │  leg_lengths_topic → CAN set_input_pos × 6 axes    │  │
│  │  ODrive feedback → motor_feedback / robot_state     │  │
│  └─────────────────────┬───────────────────────────────┘  │
└─────────────────────────┼─────────────────────────────────┘
                          │ CAN Bus
                ┌─────────┴─────────┐
                │  6× ODrive Motor  │
                │   Controllers     │
                └───────────────────┘

┌───────────────────────────────────────────────────────────┐
│                  Control Process (separate)               │
│                                                           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │              control_loop.py (500 Hz)               │  │
│  │                                                     │  │
│  │  trajectory.py  ik_solver.py  dynamics.py           │  │
│  │  workspace.py   conversions.py  geometry.py         │  │
│  └─────────────────────────────────────────────────────┘  │
└───────────────────────────────────────────────────────────┘
```

## The Motion Bridge Node

The bridge translates between the ROS2 topic world and the ZeroMQ IPC layer. It has no motion planning logic of its own — it's a pure adapter.

### ROS2 → Control Process

| ROS2 Topic | Message Type | IPC Topic | Behaviour |
|---|---|---|---|
| `platform_pose_topic` | `PlatformPoseCommand` | `target` | Forward pose to control process. **Gated:** only forwards if `msg.publisher` matches the current active control mode |
| `control_mode_topic` | `String` | `mode` | Map mode strings to enable/disable/estop commands |
| `gravity_offset` | `Float64MultiArray` | `mode` | Forward tilt correction as `set_gravity_offset` |

### Control Process → ROS2

| IPC Topic | ROS2 Topic | Message Type | Content |
|---|---|---|---|
| `telem` | `leg_lengths_topic` | `Float64MultiArray` | 18 values: [6 positions (rev), 6 velocities (rev/s), 6 torques (Nm)] |
| `telem` | `leg_torques_diagnostic` | `Float64MultiArray` | 6 feedforward torques (monitoring only) |

Telemetry is polled at 500 Hz (matching the control loop rate).

### Control Mode Gating

The bridge tracks which control source is currently active (spacemouse, shell, levelling, GUI). When a `PlatformPoseCommand` arrives, the bridge checks `msg.publisher` against the active mode. Only matching messages are forwarded — this prevents one source from interfering with another.

| Mode String | Active Publisher | IPC Action |
|---|---|---|
| `SPACEMOUSE` | spacemouse | `enable` (if mode changed) |
| `SHELL` | shell | `enable` (if mode changed) |
| `LEVELLING` | levelling | `enable` (if mode changed) |
| `GUI` | gui | `enable` (if mode changed) |
| `ERROR` | — | `estop` |
| empty/None | — | `disable` (if was active) |

## The CAN Interface

The CAN node (`can_node.py`) handles all communication with the ODrive motor controllers. The motion planner's output reaches the ODrives through this path:

```
control_loop.py
    → telemetry (pos_rev, vel_ff_rps, torque_ff_Nm)
    → motion_bridge_node.py
    → leg_lengths_topic (18 Float64 values)
    → can_node.py
    → set_input_pos CAN command × 6 axes
    → ODrive controllers
```

### CAN Command: set_input_pos

For each of the 6 axes, the CAN node sends a `set_input_pos` message containing:

| Field | Type | Source |
|---|---|---|
| Position | float32 | Motor position in revolutions |
| Velocity FF | int16 | Motor velocity × 1000 (0.001 rev/s resolution) |
| Torque FF | int16 | Motor torque × 10000 (0.0001 Nm resolution), negated |

The CAN node handles:

- **Leg inversion:** Negates position and velocity for leg sign conventions
- **int16 clamping:** Velocity and torque feedforward values are clamped to [-32768, 32767]
- **Scaling:** Multiplies by the per-axis scale factor (from `protocol_config`) and rounds to integer for the int16 fields

### Motor Feedback

The CAN node receives encoder feedback from each ODrive and publishes it on ROS2 topics. The bridge forwards this to the control process as `motorfb` IPC messages, used for tracking error computation.

### CAN Bus Timing

All 6 axes receive commands within a single control cycle (2 ms). CAN bus throughput was validated during Phase 3 Stage B:

- P99 cycle time: 2.04 ms
- No dropped frames
- All 6 axes receive commands and return feedback within one cycle

## The Orchestrator

The orchestrator (`orchestrator_node.py`) manages the overall robot state machine. It doesn't directly interact with the motion planner, but it controls the **mode** that determines which input source is forwarded to the control process.

State transitions relevant to the motion planner:

```
BOOT → HOMING → IDLE → ACTIVE (SPACEMOUSE | SHELL | ...)
                  ↑              |
                  +──────────────+  (deactivate)
FAULT ← (any state on error)
```

The orchestrator publishes mode changes on `control_mode_topic`, which the bridge translates into enable/disable/estop commands for the control process.

## Startup Sequence

### Using the Launch File (Recommended)

The primary way to start the system is the ROS2 launch file:

```bash
# Standard startup (real hardware)
ros2 launch jugglebot jugglebot_launch.py

# With rosbag recording
ros2 launch jugglebot jugglebot_launch.py record:=true

# With simulator instead of hardware
ros2 launch jugglebot jugglebot_launch.py use_simulator:=true
```

**Launch file:** [`ros_ws/src/jugglebot/launch/jugglebot_launch.py`](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/launch/jugglebot_launch.py)

**Launch arguments:**

| Argument | Default | Purpose |
|---|---|---|
| `use_simulator` | `false` | Use Webots simulator instead of real hardware |
| `record` | `false` | Enable rosbag recording (MCAP format, saved to `~/Desktop/rosbags/`) |

**Nodes started by the launch file:**

| Node / Process | Condition | Purpose |
|---|---|---|
| `can_node` | Real hardware only | CAN bus communication with ODrives |
| `orchestrator_node` | Always | Robot state machine |
| `spacemouse_handler` | Always | SpaceMouse input |
| `sp_ik` | Always | Legacy Stewart platform IK node |
| `rosbridge_websocket` | Always | WebSocket bridge for GUI |
| `rosbag record` | If `record:=true` | Records selected topics in MCAP format |
| Webots simulator | If `use_simulator:=true` | Simulated environment |

!!! note "Motion planner not yet in launch file"
    The `motion_bridge_node` and `control_loop` process are defined in the launch file but are currently **commented out** (lines 123 and 128). Until they are uncommented, the motion planner must be started manually in separate terminals alongside the launch file. This will be integrated once the motion planner is the primary control path.

### Manual Startup (When Not Using Launch File)

For development or when running the motion planner separately:

1. **Launch ROS2 nodes** (via launch file or individually)
2. **Start control process:** `python -m jugglebot.motion.control_loop --rate 500` (separate terminal). Starts in `DISABLED` mode with zero outputs.
3. **IPC connection:** The bridge's BridgeIPC binds on port 5555. The control loop's ControlProcessIPC connects. ZeroMQ handles the connection/reconnection automatically.
4. **Homing:** The orchestrator commands homing via the CAN node. ODrives find their encoder references.
5. **Activate:** The orchestrator transitions to ACTIVE, publishing a mode on `control_mode_topic`. The bridge sends an `enable` command. The control loop transitions to `ENABLED` and seeds the home pose.
6. **Operation:** The active input source (spacemouse, shell, ball predictor) publishes `PlatformPoseCommand` messages. The bridge forwards them. The control loop computes motor commands.

## Shutdown Sequence

On shutdown:

1. The bridge sends a `disable` mode command to the control process
2. The control loop zeros all outputs and cancels any active trajectory
3. The trajectory manager's background thread is shut down (2-second timeout)
4. IPC sockets are closed
5. ROS2 nodes are destroyed

If the control process crashes or is killed, the ODrives hold the last commanded position (position control fail-safe). The bridge's heartbeat watchdog will detect the loss and can trigger an E-stop through the orchestrator.

## Error Handling

### ODrive Faults

If an ODrive reports a fault (overcurrent, encoder error, etc.), the CAN node detects it and publishes on `robot_state`. The orchestrator parses this and publishes `ERROR` on `control_mode_topic`. The bridge sends an `estop` command to the control process. All motor outputs go to zero.

The control process can also receive faults directly via the `fault` mode command, which triggers the same E-stop sequence.

### IPC Loss

If the control process receives no messages for 0.5 seconds, it self-triggers an E-stop (`ipc_heartbeat_lost`). This protects against bridge crashes, network issues, or ROS2 hangs.

### Workspace Violations

If a workspace hard limit is triggered during operation, the control loop cancels the active trajectory and E-stops. This catches situations where external disturbances or modelling errors push the platform beyond safe bounds.

## Dependencies

The motion planner core (`motion/` subpackage) depends only on:

- **numpy** — array math
- **pyzmq** — ZeroMQ bindings (IPC)
- **msgpack** — message serialization

These are Jetson-only dependencies (not needed on Windows dev machines for offline testing, though pyzmq and msgpack are available there too).

The bridge node additionally depends on:

- **rclpy** — ROS2 Python client
- Standard ROS2 message types (`std_msgs`, `geometry_msgs`)
- Custom Jugglebot message types (`PlatformPoseCommand`)
