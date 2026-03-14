# Architecture

This page describes the overall design of the motion planner — how the pieces fit together, why they're structured the way they are, and how data flows from a desired platform pose to motor commands on the CAN bus.

## The Big Picture

The motion planner's job is straightforward: given where the platform should be, compute what each motor should do. But "what each motor should do" is three things, not one:

1. **Position** — where the motor shaft should be (revolutions)
2. **Velocity feedforward** — how fast it should be moving (rev/s)
3. **Torque feedforward** — what force it needs to apply (Nm)

These three values are sent to each ODrive motor controller every 2 ms (500 Hz). The ODrive's own cascaded PID (running at 8 kHz) uses them to drive the motor.

## Why Position Control with Feedforward?

The ODrive implements three cascaded control loops in firmware:

```
Position loop (8 kHz):
    vel_cmd = pos_gain * (target_pos - actual_pos) + vel_ff

Velocity loop (8 kHz):
    iq_cmd = vel_gain * (vel_cmd - actual_vel) + vel_integrator + torque_ff / Kt

Current loop (40+ kHz):
    PWM drive
```

The motion planner provides `target_pos`, `vel_ff`, and `torque_ff`. The ODrive handles everything else. This split has four key advantages:

**The dynamics model is expressed through feedforward.** Gravity compensation, inertia feedforward, Jacobian-based force decomposition — all of this flows through `vel_ff` and `torque_ff`. In a well-tuned system, the feedforward does the heavy lifting; the ODrive's PID only corrects for modelling errors and disturbances.

**The ODrive's feedback runs 16x faster than Python.** Closing a PID at 8 kHz on dedicated hardware will always outperform a 500 Hz Python loop for disturbance rejection and tracking bandwidth.

**Fail-safe on communication loss.** If the Python loop stalls, the ODrive holds the last commanded position. In torque or velocity mode, a stale command would mean the motor keeps applying force or keeps moving — potentially into an end-stop.

**Natural trajectory interface.** The quintic trajectory generator produces position, velocity, and acceleration at each timestep. These map directly to `input_pos`, `vel_ff`, and `torque_ff` (via the dynamics model).

## Process Architecture

The system runs as two separate OS processes connected by ZeroMQ IPC:

```
Process 1: ROS2 Nodes                    Process 2: Control Loop
+----------------------------------+     +---------------------------+
|                                  |     |                           |
|  orchestrator_node.py            |     |  control_loop.py          |
|  can_node.py                     |     |    quintic.py             |
|  motion_bridge_node.py -------IPC----->    feasibility.py          |
|  spacemouse_handler.py           |     |    trajectory_manager.py  |
|  mocap_interface.py        <--IPC------    ik_solver.py            |
|                                  |     |    dynamics.py            |
|                                  |     |    workspace.py           |
+----------------------------------+     +---------------------------+
```

**Why a separate process?** ROS2's executor is designed for message passing, not deterministic timing. The control loop needs consistent 2 ms cycles. Running it in its own process with a simple `time.sleep()` loop gives predictable timing — measured at 500 Hz mean, 0.093 ms p99 jitter on the Jetson.

**Why ZeroMQ?** It's lightweight, has no ROS2 dependency, supports PUB/SUB patterns, and adds minimal latency (median 0.755 ms, well under one control cycle).

## IPC Layer

Communication between the bridge and control process uses two ZeroMQ channels:

```
Bridge (ROS2 side)                     Control Process

  PUB ──tcp://localhost:5555──────► SUB   (commands in)
  SUB ◄──tcp://localhost:5556────── PUB   (telemetry out)
```

Messages are serialized with msgpack (compact binary, faster than JSON).

### Command Channel (5555) — Bridge to Control Process

The control process has **two** SUB sockets on this channel:

| Socket | Topics | Mode | Purpose |
|---|---|---|---|
| Data socket | `target`, `motorfb` | CONFLATE (keep latest only) | High-frequency pose targets and motor feedback — only the most recent matters |
| Mode socket | `mode`, `traj`, `dyntgt` | Ordered (deliver every message) | Critical commands that must not be dropped |

This dual-socket design means a burst of high-frequency target updates won't queue behind each other, while mode commands like "estop" are guaranteed to arrive.

### Telemetry Channel (5556) — Control Process to Bridge

Single PUB socket with CONFLATE on the subscriber side. Carries the computed motor commands plus diagnostics (timing, workspace status, tracking error, fault state).

### Message Types

| Topic | Constructor | Content |
|---|---|---|
| `target` | `make_target_state()` | Platform pose (pos + quaternion), twist, acceleration |
| `mode` | `make_mode_command()` | enable, disable, estop, set_feedforward, set_gravity_offset, fault |
| `traj` | `make_trajectory_command()` | Quintic trajectory boundary conditions + duration |
| `dyntgt` | `make_dynamic_target_command()` | Dynamic target: position, quaternion, velocity, arrival time |
| `motorfb` | `make_motor_feedback()` | Encoder positions, velocities, currents from CAN |
| `telem` | `make_telemetry()` | Motor commands, feedforward torques, timing, workspace status, faults |

See [ipc.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/ipc.py) for the complete message definitions.

## Control Modes

The control loop has three modes:

| Mode | Behaviour |
|---|---|
| `DISABLED` | All outputs zero. No motor commands sent. |
| `ENABLED` | Active control. Computes and publishes motor commands. |
| `ESTOP` | Emergency stop. All outputs zero. Requires explicit re-enable. |

Within `ENABLED`, the loop operates in one of two sub-modes:

**Trajectory mode:** When a trajectory is active (state is EXECUTING or RETURNING), the loop evaluates the quintic polynomials at the current time to get Cartesian pose/twist/acceleration, then maps these to motor commands through IK and dynamics.

**Direct-target mode:** When no trajectory is active, the loop uses the most recent target pose from IPC (e.g., from the spacemouse or shell commands) and computes IK + dynamics directly.

## The Computation Pipeline

Every 2 ms, the control loop runs the same pipeline regardless of sub-mode:

```
1. Cartesian state: (pos, twist, accel)
        |
        v
2. Rotation: rotvec -> rotation matrix (+ gravity correction if set)
        |
        v
3. Position IK: pose -> leg extensions (mm)
        |
        v
4. Unit conversion: extensions (mm) -> motor positions (rev)
        |
        v
5. Velocity IK: twist -> leg velocities (mm/s) -> motor velocities (rev/s)
        |
        v
6. Dynamics: gravity wrench + inertia wrench -> J^{-T} -> leg forces -> motor torques
        |
        v
7. Workspace check: verify extensions and condition number within limits
        |
        v
8. Output: (pos_rev, vel_ff_rps, torque_ff_Nm) x 6 legs
```

Steps 1–6 are detailed in the [Kinematics](kinematics.md) and [Dynamics](dynamics.md) pages. Step 7 is covered in [Workspace Safety](workspace.md). The full loop is described in [Control Loop](control_loop.md).

## Coordinate Frames and Conventions

### Frames

- **Base frame:** Origin at the geometric centre of the base. Z-axis points up. This is the world frame.
- **Platform frame:** Body-fixed to the platform. At the home pose, it is aligned with the base frame, with the platform centre at height `init_height_mm` (574.3 mm) above the base.

### Pose Representation

A platform pose is specified as an **offset from home**:

- **Position:** `[x, y, z]` in mm — displacement of the platform centre from its home position. At home, `pos = [0, 0, 0]` and the absolute platform height is `init_height_mm`.
- **Orientation:** Rotation vector `[rx, ry, rz]` in radians (axis × angle). At home, `rot = [0, 0, 0]`. Valid for tilts up to ~15 degrees. Internally converted to a 3×3 rotation matrix.

### Leg Extensions

- `0 mm` = fully retracted (string fully wound)
- `leg_stroke_mm` (280 mm) = fully extended (string fully unwound)
- Values outside this range are physically impossible — the workspace checker rejects them

### Sign Conventions

- Positive leg force = extension direction (unwinding string)
- Positive motor torque = extension direction
- The CAN node handles ODrive-specific sign inversion — the motion planner never deals with it

### Units

| Quantity | Unit |
|---|---|
| Length / extension | mm |
| Motor position | revolutions |
| Velocity (Cartesian) | mm/s, rad/s |
| Velocity (motor) | rev/s |
| Force | N |
| Torque | Nm |
| Wrench | N and N·mm (mixed due to Jacobian units) |
| Acceleration | mm/s², rad/s² |

The Jacobian maps `[mm/s, mm/s, mm/s, rad/s, rad/s, rad/s]` to `[mm/s × 6]`, so its columns have mixed units. This causes the raw condition number to be ~450 at home — high in absolute terms but normal for this convention. All condition-number thresholds use relative factors (1.5× and 2.0× the home value) to account for this.

## Source Files

| File | Lines | Purpose |
|---|---|---|
| [geometry.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/geometry.py) | ~65 | Platform geometry constants |
| [ik_solver.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py) | ~370 | Kinematics (IK, FK, Jacobian) |
| [conversions.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/conversions.py) | ~100 | Unit conversions |
| [dynamics.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/dynamics.py) | ~360 | Gravity + inertia feedforward |
| [workspace.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/workspace.py) | ~300 | Workspace limits |
| [quintic.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/quintic.py) | ~280 | Quintic polynomial solver + evaluation |
| [motor_commands.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/motor_commands.py) | ~85 | Cartesian → motor command mapping |
| [feasibility.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/feasibility.py) | ~340 | Feasibility checking + convenience constructors |
| [trajectory_manager.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/trajectory_manager.py) | ~530 | Execution state machine + async pipeline |
| [feasibility_worker.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/feasibility_worker.py) | ~380 | Background worker process |
| [ipc.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/ipc.py) | ~375 | ZeroMQ IPC layer |
| [control_loop.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/control_loop.py) | ~690 | Control process |
| [motion_bridge_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py) | ~195 | ROS2 bridge |
