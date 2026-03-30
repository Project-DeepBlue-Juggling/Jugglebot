# System Integration

This page describes how the motion system connects to the rest of the Jugglebot system — the ROS2 bridges, the MPC solver, the motor guard, the CAN interface, and how the full system starts up and shuts down.

**Source files:**

- [motion_bridge_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py) — ROS2 ↔ motor guard bridge
- [mpc_bridge_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/mpc_bridge_node.py) — ROS2 ↔ MPC target bridge
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
│  ┌──────────────────────────────┐  ┌──────────────────┐  │
│  │    motion_bridge_node.py    │  │ mpc_bridge_node  │  │
│  │  ROS2 ←→ ZMQ :5555/:5556   │  │ ROS2 → ZMQ :5558│  │
│  └──────────────┬──────────────┘  └────────┬─────────┘  │
│                 │                          │             │
│  ┌──────────────┴──────────────────────────┘             │
│  │                                                       │
│  │  ┌─────────────────────────────────────────────────┐  │
│  │  │                  can_node.py                    │  │
│  │  │  leg_lengths_topic → CAN set_input_pos × 6     │  │
│  │  │  ODrive feedback → motor_feedback / robot_state │  │
│  │  └─────────────────────┬───────────────────────────┘  │
│  │                        │ CAN Bus                      │
└──┼────────────────────────┼──────────────────────────────┘
   │                ┌───────┴─────────┐
   │                │  6× ODrive Motor│
   │                │   Controllers   │
   │                └─────────────────┘
   │
┌──┼────────────────────────────────────────────────────────┐
│  │            MPC Process (50 Hz, separate)                │
│  │                                                         │
│  │  ┌─────────────────────────────────────────────────┐    │
│  │  │  sim/main.py --hardware                         │    │
│  │  │                                                 │    │
│  │  │  ZmqTargetSource ← ZMQ :5558 ← mpc_bridge_node │    │
│  │  │  MPCController (CasADi/IPOPT)                   │    │
│  │  │  HardwarePlant → ZMQ :5557 → motor_guard        │    │
│  │  └─────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────┐
│               Motor Guard Process (500 Hz, separate)       │
│                                                            │
│  ┌─────────────────────────────────────────────────────┐   │
│  │              motor_guard.py                         │   │
│  │                                                     │   │
│  │  Quadratic interpolation (50 Hz → 500 Hz)           │   │
│  │  Safety checks (every cycle)                        │   │
│  │  workspace.py   ik_solver.py (cond# only)           │   │
│  └─────────────────────────────────────────────────────┘   │
└───────────────────────────────────────────────────────────┘
```

## The Motion Bridge Node

The motion bridge translates between the ROS2 topic world and the motor guard's ZeroMQ IPC layer. It has no motion planning logic of its own — it's a pure adapter.

### ROS2 → Motor Guard

| ROS2 Topic | Message Type | IPC Topic | Behaviour |
|---|---|---|---|
| `control_mode_topic` | `String` | `mode` | Map mode strings to enable/disable/estop commands |
| `robot_state` | `RobotState` | `motorfb` | Forward motor positions/velocities/currents (first 6 axes = legs) to motor guard for [safety checks](safety.md) |

### Motor Guard → ROS2

| IPC Topic | ROS2 Topic | Message Type | Content |
|---|---|---|---|
| `telem` | `leg_lengths_topic` | `Float64MultiArray` | 18 values: [6 positions (rev), 6 velocities (rev/s), 6 torques (Nm)] |
| `telem` | `leg_torques_diagnostic` | `Float64MultiArray` | 6 feedforward torques (monitoring only) |
| `telem` | `motion/tracking_error` | `Float64MultiArray` | 6 per-leg tracking errors (mm) |
| `telem` | `motion/diagnostics` | `DiagnosticStatus` | Condition number, workspace status, faults |

Telemetry is polled at 500 Hz (matching the motor guard rate).

## The MPC Bridge Node

The MPC bridge forwards target poses from ROS2 input sources to the MPC process via ZMQ. It is mode-gated: only the active source's targets are forwarded.

### ROS2 → MPC Process

| ROS2 Topic | Message Type | ZMQ Topic | Behaviour |
|---|---|---|---|
| `platform_pose_topic` | `PlatformPoseCommand` | `mpctgt` | Forward pose to MPC. **Gated:** only forwards if `msg.publisher` matches the current active mode |
| `catch/dynamic_target` | `DynamicTargetCommand` | `mpctgt` | Forward catch target to MPC. **Gated:** only forwards when mode is CATCH |
| `control_mode_topic` | `String` | `mpcmode` | Forward mode transitions (enable/disable) |
| `gravity_offset` | `Float64MultiArray` | — | Stored as correction rotation matrix; composed into every outgoing target orientation |

### Mode Gating

| Mode String | Active Publisher | Action |
|---|---|---|
| `SPACEMOUSE` | spacemouse | Forward platform_pose_topic, send `mpcmode:spacemouse` |
| `SHELL` | shell | Forward platform_pose_topic, send `mpcmode:shell` |
| `GUI` | gui | Forward platform_pose_topic, send `mpcmode:gui` |
| `CATCH` | catch_coordinator | Forward catch/dynamic_target, send `mpcmode:catch` |
| `LEVELLING` | — | Send `mpcmode:disabled` |
| `ERROR` | — | Send `mpcmode:disabled` |
| empty/None | — | Send `mpcmode:disabled` |

### Catch Target Feedback

When the MPC receives a catch target (via `:5558`), it sends accept/reject feedback to the catch coordinator via a dedicated ZMQ channel on `:5559` (`TargetFeedbackPub` → `TargetFeedbackSub`). This tells the coordinator whether the solver was able to plan a trajectory to the requested catch position and timing.

### Gravity Correction

The `gravity_offset` topic provides tilt correction for base levelling. The MPC bridge stores the correction as a rotation matrix and pre-multiplies it into every outgoing target orientation before reaching the MPC. The MPC sees corrected references transparently — it does not need to know about the gravity correction.

## The CAN Interface

The CAN node (`can_node.py`) handles all communication with the ODrive motor controllers. The motor guard's output reaches the ODrives through this path:

```
motor_guard.py
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

The CAN node receives encoder feedback from each ODrive at ~100 Hz and publishes it on the `/robot_state` topic as a `RobotState` message containing a `MotorStateSingle` per axis. The bridge subscribes to this topic, extracts `pos_estimate`, `vel_estimate`, and `iq_measured` from the first 6 entries (the leg motors), and forwards them to the motor guard as `motorfb` IPC messages.

This feedback is critical for the [motor command safety](safety.md) system — the max-deviation check and overspeed check both require current encoder data. Without current feedback, the motor guard suppresses all motor commands.

### CAN Bus Timing

All 6 axes receive commands within a single control cycle (2 ms). CAN bus throughput was validated during Phase 3 Stage B:

- P99 cycle time: 2.04 ms
- No dropped frames
- All 6 axes receive commands and return feedback within one cycle

## The Orchestrator

The orchestrator (`orchestrator_node.py`) manages the overall robot state machine. It doesn't directly interact with the motion system, but it controls the **mode** that determines which input source is forwarded to the MPC.

State transitions relevant to the motion system:

```
BOOT → HOMING → IDLE → ACTIVE (SPACEMOUSE | SHELL | GUI | CATCH)
                  ↑              |
                  +──────────────+  (deactivate)
FAULT ← (any state on error)
```

The orchestrator publishes mode changes on `control_mode_topic`, which both bridges translate into appropriate commands:

- **motion_bridge_node** → enable/disable/estop for the motor guard
- **mpc_bridge_node** → enable/disable for the MPC process

## Startup Sequence

### Using the Launch File (Recommended)

The primary way to start the system is the ROS2 launch file:

```bash
# Standard startup (real hardware)
ros2 launch jugglebot jugglebot_launch.py

# With rosbag recording
ros2 launch jugglebot jugglebot_launch.py record:=true
```

**Launch file:** [`ros_ws/src/jugglebot/launch/jugglebot_launch.py`](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/launch/jugglebot_launch.py)

**Nodes and processes started by the launch file:**

| Node / Process | Condition | Purpose |
|---|---|---|
| `can_node` | Real hardware only | CAN bus communication with ODrives |
| `orchestrator_node` | Always | Robot state machine |
| `spacemouse_handler` | Always | SpaceMouse input |
| `sp_ik` | Always | Legacy Stewart platform IK node |
| `rosbridge_websocket` | Always | WebSocket bridge for GUI |
| `motion_bridge_node` | Always | Motor guard ↔ ROS2 bridge |
| `mpc_bridge_node` | Always | MPC target ↔ ROS2 bridge |
| `motor_guard` | Always | 500 Hz interpolator + safety |
| `rosbag record` | If `record:=true` | Records selected topics in MCAP format |

### Manual Startup (When Not Using Launch File)

For development or when running processes separately:

1. **Launch ROS2 nodes** (via launch file or individually)
2. **Start motor guard:** `python -m jugglebot.motion.motor_guard --rate 500` (separate terminal). Starts in `DISABLED` mode with zero outputs.
3. **Start MPC process:** `python sim/main.py --hardware` (separate terminal). Defaults to `ZmqTargetSource` for receiving targets from `mpc_bridge_node`.
4. **IPC connection:** ZeroMQ handles connection/reconnection automatically on all ports.
5. **Homing:** The orchestrator commands homing via the CAN node. ODrives find their encoder references.
6. **Activate:** The orchestrator transitions to ACTIVE, publishing a mode on `control_mode_topic`. Both bridges forward enable commands. The MPC begins solving; the motor guard begins interpolating.
7. **Operation:** The active input source (spacemouse, shell, catch coordinator) publishes targets. The MPC bridge forwards them. The MPC plans a trajectory. The motor guard interpolates and sends to hardware.

## Shutdown Sequence

On shutdown:

1. The motion bridge sends a `disable` mode command to the motor guard
2. The MPC bridge sends `disabled` to the MPC process
3. The motor guard zeros all outputs
4. IPC sockets are closed
5. ROS2 nodes are destroyed

If the motor guard crashes or is killed, the ODrives hold the last commanded position (position control fail-safe). The bridge's heartbeat watchdog will detect the loss and can trigger an E-stop through the orchestrator.

If the MPC process crashes, the motor guard detects it via the MPC staleness timeout (200 ms) and triggers an E-stop.

## Error Handling

### ODrive Faults

If an ODrive reports a fault (overcurrent, encoder error, etc.), the CAN node detects it and publishes on `robot_state`. The orchestrator parses this and publishes `ERROR` on `control_mode_topic`. Both bridges propagate the error: the motion bridge sends `estop` to the motor guard, and the MPC bridge sends `disabled` to the MPC process. All motor outputs go to zero.

### IPC Loss

If the motor guard receives no messages for 0.5 seconds, it self-triggers an E-stop (`ipc_heartbeat_lost`). This protects against bridge crashes, network issues, or ROS2 hangs.

### MPC Process Loss

If the motor guard receives no MPC command for 200 ms, it self-triggers an E-stop (`mpc_cmd_staleness`). This catches MPC process crashes, solver hangs, or ZMQ connection loss.

### Workspace Violations

If a workspace hard limit is triggered on an incoming MPC command, the motor guard E-stops. This catches cases where the MPC solver produces an out-of-bounds solution (constraint violation).

## Dependencies

The motor guard (`motion/` subpackage) depends only on:

- **numpy** — array math
- **pyzmq** — ZeroMQ bindings (IPC)
- **msgpack** — message serialization

The MPC process additionally depends on:

- **casadi** — symbolic optimization
- **IPOPT** — nonlinear solver (via CasADi)

The bridge nodes additionally depend on:

- **rclpy** — ROS2 Python client
- Standard ROS2 message types (`std_msgs`, `geometry_msgs`)
- Custom Jugglebot message types (`PlatformPoseCommand`, `DynamicTargetCommand`)
