# Architecture

This page describes the overall design of the motion system — how the pieces fit together, why they're structured the way they are, and how data flows from a desired platform pose to motor commands on the CAN bus.

## The Big Picture

The motion system's job is straightforward: given where the platform should be, compute what each motor should do. But "what each motor should do" is three things, not one:

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

The motion system provides `target_pos`, `vel_ff`, and `torque_ff`. The ODrive handles everything else. This split has four key advantages:

**The dynamics model is expressed through feedforward.** Gravity compensation, inertia feedforward, Jacobian-based force decomposition — all of this flows through `vel_ff` and `torque_ff`. In a well-tuned system, the feedforward does the heavy lifting; the ODrive's PID only corrects for modelling errors and disturbances.

**The ODrive's feedback runs 16x faster than Python.** Closing a PID at 8 kHz on dedicated hardware will always outperform a 500 Hz Python loop for disturbance rejection and tracking bandwidth.

**Fail-safe on communication loss.** If the Python process stalls, the ODrive holds the last commanded position. In torque or velocity mode, a stale command would mean the motor keeps applying force or keeps moving — potentially into an end-stop.

**Natural MPC interface.** The MPC solver outputs leg extensions directly (via symbolic IK constraints). These map directly to `input_pos`, and `vel_ff`/`torque_ff` are computed from the predicted trajectory's velocity and acceleration.

## Process Architecture

The system runs as three separate OS processes connected by ZeroMQ IPC:

```
Process 1: ROS2 Nodes
+-------------------------------------------------------+
|                                                       |
|  orchestrator_node.py         spacemouse_handler.py   |
|  can_node.py                  mocap_interface.py      |
|  motion_bridge_node.py  ←--→  ZMQ :5555/:5556        |
|  mpc_bridge_node.py     ←--→  ZMQ :5558              |
|  catch_coordinator_node.py                            |
|  ball_tracker_node.py                                 |
+-------------------------------------------------------+

Process 2: MPC Solver (50 Hz)
+-------------------------------------------------------+
|  sim/main.py --hardware                               |
|                                                       |
|  ZmqTargetSource  ←--- ZMQ :5558 --- mpc_bridge_node |
|  MPCController (CasADi/IPOPT)                         |
|  HardwarePlant ---→ ZMQ :5557 ---→ motor_guard        |
+-------------------------------------------------------+

Process 3: Motor Guard (500 Hz)
+-------------------------------------------------------+
|  motor_guard.py                                       |
|                                                       |
|  Quadratic interpolation (50 Hz → 500 Hz)             |
|  Safety checks (every cycle)                          |
|  workspace.py    ik_solver.py (condition number only) |
+-------------------------------------------------------+
```

**Why three processes?**

- **ROS2 nodes** handle message routing, mode management, and CAN communication. They use ROS2's executor for event-driven callbacks.
- **MPC solver** runs at 50 Hz with its own timing. It stays ROS2-free so the same code runs identically in MuJoCo simulation (on Windows) and on hardware (on Jetson). CasADi/IPOPT solve times vary; a dedicated process prevents solve-time jitter from affecting motor output timing.
- **Motor guard** needs consistent 2 ms cycles. Running it in its own process with a simple `time.sleep()` loop gives predictable timing — measured at 500 Hz mean, 0.093 ms p99 jitter on the Jetson.

**Why ZeroMQ?** It's lightweight, has no ROS2 dependency, supports PUB/SUB patterns, and adds minimal latency (median 0.755 ms, well under one control cycle).

## Signal Flow

### Target → MPC → Motors

All input modes (spacemouse, GUI, shell, catch coordinator) route through the MPC:

```
spacemouse_handler ──┐
GUI (rosbridge)    ──┼──► platform_pose_topic ──► mpc_bridge_node ──► ZMQ :5558
shell commands     ──┘                                                    │
catch_coordinator ────► catch/dynamic_target ──► mpc_bridge_node ──► ZMQ :5558
                                                                          │
                                                                          ▼
                                                                    ZmqTargetSource
                                                                          │
                                                                          ▼
                                                              MPCController.solve()
                                                                          │
                                                                          ▼
                                                              HardwarePlant.command()
                                                                          │
                                                                    ZMQ :5557
                                                                          │
                                                                          ▼
                                                              motor_guard (500 Hz)
                                                                          │
                                                                    ZMQ :5556
                                                                          │
                                                                          ▼
                                                            motion_bridge_node
                                                                          │
                                                              leg_lengths_topic
                                                                          │
                                                                          ▼
                                                                   can_node.py
                                                                          │
                                                                       CAN bus
                                                                          │
                                                                          ▼
                                                                 6× ODrive Motors
```

### Motor Feedback Loop

Motor feedback closes the loop at every MPC tick (50 Hz):

```
Encoders → CAN → can_node → /robot_state (100 Hz)
    → motion_bridge_node → ZMQ :5555 → motor_guard (safety checks)
    → motion_bridge_node → /robot_state → HardwarePlant.get_state()
        → FK (encoder positions → Cartesian pose)
        → J⁻¹ · q̇ (encoder velocities → platform twist)
        → PlantState (actual pose + twist)
            → mpc.solve(actual_state, target)
```

The MPC replans from measured state every 20 ms — it never plans from its own prediction. If a disturbance pushes the platform off-plan, the next solve sees the real position *and velocity* and adapts.

## IPC Layer

Communication uses four ZeroMQ channels:

```
Bridge → Motor Guard:
  PUB ──tcp://localhost:5555──────► SUB   (mode + motor feedback)

Motor Guard → Bridge:
  SUB ◄──tcp://localhost:5556────── PUB   (telemetry: motor commands + diagnostics)

MPC → Motor Guard:
  HardwarePlant PUB ──tcp://localhost:5557──► SUB   (MPC commands)

MPC Bridge → MPC Process:
  PUB ──tcp://localhost:5558──────► SUB   (target poses + mode transitions)

MPC Process → Catch Coordinator:
  PUB ──tcp://localhost:5559──────► SUB   (target accept/reject feedback)
```

Messages are serialized with msgpack (compact binary, faster than JSON).

### Message Types

| Topic | Constructor | Content |
|---|---|---|
| `mode` | `make_mode_command()` | enable, disable, estop |
| `mpccmd` | `make_mpc_command()` | Motor positions (rev), velocities (rev/s), torques (Nm), extensions (mm), pose |
| `motorfb` | `make_motor_feedback()` | Encoder positions, velocities, currents from CAN |
| `telem` | (pre-allocated dict) | Motor commands, torques, timing, workspace status, faults |
| `mpctgt` | `make_mpc_target()` | Target pose (rotation vector), arrival time, twist, source |
| `mpcmode` | `make_mpc_mode()` | MPC enable/disable transitions |

See [ipc.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/ipc.py) for the complete message definitions.

## Control Modes

The motor guard has three modes:

| Mode | Behaviour |
|---|---|
| `DISABLED` | All outputs zero. No motor commands sent. |
| `ENABLED` | Active interpolation. Quadratically extrapolates MPC commands at 500 Hz. |
| `ESTOP` | Emergency stop. All outputs zero. Requires explicit re-enable. |

The MPC process has its own lifecycle managed by the `mpc_bridge_node`:

- Mode transitions on `control_mode_topic` are forwarded to the MPC process via ZMQ :5558
- `ZmqTargetSource.enabled` drives `HardwarePlant.enable()` / `disable()` calls
- While disabled, the MPC idles (no solves, no commands)

## Where IK and Dynamics Live

**IK is inside the MPC solver.** The MPC formulates `q[k] = IK(p[k])` as an equality constraint using CasADi symbolic IK. Every optimization step enforces kinematic consistency. The solver outputs leg extensions directly — no external IK step is needed anywhere in the pipeline.

**Dynamics are computed in HardwarePlant.** When the MPC outputs a command, `HardwarePlant.set_pose()` computes the full Newton-Euler torque feedforward (gravity + platform inertia + reflected motor inertia) via `cartesian_to_motor_commands()`. Platform twist and acceleration are derived from the MPC's predicted trajectory (finite-differencing the first three nodes at 20 ms spacing).

**The motor guard does no IK or dynamics.** It receives pre-computed motor-space commands (position in rev, velocity in rev/s, torque in Nm) and only interpolates + validates.

## Coordinate Frames and Conventions

### Frames

- **Base frame:** Origin at the geometric centre of the base. Z-axis points up. This is the world frame.
- **Platform frame:** Body-fixed to the platform. At the active pose, it is aligned with the base frame, with the platform centre at height `init_height_mm` (574.3 mm) above the base.

### Pose Representation

A platform pose is specified as an **offset from the active pose**:

- **Position:** `[x, y, z]` in mm — displacement of the platform centre from its active position. At the active pose, `pos = [0, 0, 0]` and the absolute platform height is `init_height_mm`.
- **Orientation:** Rotation vector `[rx, ry, rz]` in radians (axis × angle). At the active pose, `rot = [0, 0, 0]`. Valid for tilts up to ~15 degrees. Internally converted to a 3×3 rotation matrix.

### Leg Extensions

- `0 mm` = fully retracted (string fully wound)
- `leg_stroke_mm` (280 mm) = fully extended (string fully unwound)
- Values outside this range are physically impossible — the workspace checker rejects them

### Sign Conventions

- Positive leg force = extension direction (unwinding string)
- Positive motor torque = extension direction
- The CAN node handles ODrive-specific sign inversion — the motion system never deals with it

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

The Jacobian maps `[mm/s, mm/s, mm/s, rad/s, rad/s, rad/s]` to `[mm/s × 6]`, so its columns have mixed units. This causes the raw condition number to be ~450 at the active pose — high in absolute terms but normal for this convention. All condition-number thresholds use relative factors (1.5× and 2.0× the reference value) to account for this.

## Source Files

### Motor Guard + IPC

| File | Lines | Purpose |
|---|---|---|
| [motor_guard.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py) | ~860 | Interpolator + safety monitor |
| [ipc.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/ipc.py) | ~375 | ZeroMQ IPC layer |
| [motion_bridge_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py) | ~195 | ROS2 ↔ ZMQ bridge (motor guard side) |
| [mpc_bridge_node.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/mpc_bridge_node.py) | ~170 | ROS2 ↔ ZMQ bridge (MPC target side) |

### Motion Primitives (used by MPC / HardwarePlant)

| File | Lines | Purpose |
|---|---|---|
| [geometry.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/geometry.py) | ~65 | Platform geometry constants |
| [ik_solver.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py) | ~370 | Kinematics (IK, FK, Jacobian) |
| [conversions.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/conversions.py) | ~100 | Unit conversions |
| [dynamics.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/dynamics.py) | ~360 | Gravity + inertia feedforward |
| [workspace.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/workspace.py) | ~300 | Workspace limits |
| [motor_commands.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/ros_ws/src/jugglebot/jugglebot/motion/motor_commands.py) | ~85 | Cartesian → motor command mapping |

### MPC Solver (top-level package)

| File | Lines | Purpose |
|---|---|---|
| [controller/mpc.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/controller/mpc.py) | — | MPCController class (CasADi/IPOPT) |
| [controller/params.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/controller/params.py) | — | MPCParams tuning parameters |
| [controller/target.py](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/controller/target.py) | — | TargetCommand + TargetSource protocol |

### Archived (replaced by MPC)

The following were archived to `ros_ws/.../archived/` during the MPC-native refactor:

| File | Replaced by |
|---|---|
| `control_loop.py` | `motor_guard.py` |
| `stream_smoother.py` | MPC trajectory smoothing |
| `trajectory_manager.py` | MPC trajectory planning |
| `feasibility.py` | MPC constraints |
| `feasibility_worker.py` | MPC constraints |
| `quintic.py` | MPC trajectory generation |
