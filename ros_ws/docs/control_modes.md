# Control Modes and Command Flow

## Overview

Motor position commands can reach the ODrives through two independent paths.
Only one should be active at a time — the architecture enforces mutual
exclusion through control mode gating.

### Path 1: MPC → motor guard → bridge → CAN node

```
MPC (50 Hz)  →  HardwarePlant  →  motor_guard.py  →  IPC (ZeroMQ)  →  motion_bridge_node.py  →  leg_lengths_topic  →  can_node.py
                    :5557            500 Hz interp        :5556            500 Hz poll              ROS2 topic          CAN bus
```

Used in: `SPACEMOUSE`, `SHELL`, `GUI`, `CATCH` modes.

All input modes route through the MPC, which is the sole motion planner. The
MPC receives targets via the MPC bridge node (:5558), solves at 50 Hz, and
sends leg extension commands via HardwarePlant to the motor guard (:5557).
The motor guard interpolates at 500 Hz (cubic, with jerk estimation), applies
safety checks (workspace limits, overspeed, staleness, deviation), and
publishes motor commands via IPC. The motion bridge node polls these and
publishes to `leg_lengths_topic`. The CAN node subscribes and sends
`set_input_pos` commands (position + vel_ff + torque_ff) to each ODrive.

### Path 2: Direct CAN (CAN node internal)

```
can_node.py  →  _gently_move_to_setpoint()  →  _send_position_target()  →  CAN bus
                    trapezoidal profile              per-axis command
```

Used in: `LEVELLING` (activate/deactivate), activation, deactivation, error
stow.

The CAN node generates its own trapezoidal position profiles internally and
streams them to the ODrives at ~100 Hz. This path does **not** go through
`leg_lengths_topic` and is not subject to the position step limit check
(since the CAN node controls the profile internally).

## Mode lifecycle

### LEVELLING

```
IDLE → LEVELLING:
  1. State machine sets ctx.control_mode = 'LEVELLING'
  2. Orchestrator publishes control_mode_topic = 'LEVELLING'
  3. Motion bridge: does NOT enable motor guard (LEVELLING excluded)
  4. CAN node: activate_or_deactivate service → gentle move to activation height
  5. Settle → read inclinometer → publish gravity offset → persist
  6. CAN node: deactivate → gentle move to 0
  7. State machine transitions to IDLE, control_mode = ''
```

The motor guard stays `DISABLED` throughout. The gravity offset is received by
the MPC bridge node via the `gravity_offset` ROS2 topic and composed into every
outgoing target orientation, so the MPC sees corrected references transparently.

### SPACEMOUSE / SHELL / GUI / CATCH (active modes)

```
IDLE → ACTIVE:
  1. State machine sets ctx.control_mode = 'SPACEMOUSE' (or SHELL/GUI/CATCH)
  2. Orchestrator publishes control_mode_topic
  3. CAN node: activate_or_deactivate service → gentle move to activation height
  4. Motion bridge: sends 'enable' to motor guard
  5. MPC bridge: receives mode, begins forwarding targets to MPC on :5558
  6. MPC solves, HardwarePlant sends commands to motor guard on :5557
  7. Motor guard: interpolates at 500 Hz, publishes telemetry on :5556
  8. Motion bridge: forwards telemetry to leg_lengths_topic
  9. CAN node: receives and sends motor commands
```

### ERROR

```
Any state → FAULT:
  1. Orchestrator publishes control_mode_topic = 'ERROR'
  2. Motion bridge: sends 'estop' to motor guard, sets _motor_guard_enabled = False
  3. MPC bridge: sends 'disabled' mode to MPC on :5558
  4. CAN node: gentle move to 0 (stow)
```

## Safety invariants

1. The motor guard never publishes telemetry unless `mode == ENABLED`
2. The motion bridge never publishes to `leg_lengths_topic` unless
   `_motor_guard_enabled == True`
3. The CAN node rejects any `leg_lengths_topic` command that would step a leg
   by more than `JB_OP_MAX_POSITION_STEP_REV` (0.2 rev) from its current
   encoder position
4. The CAN node's `_gently_move_to_setpoint()` always seeds the ODrive with
   the current encoder position before entering closed-loop, preventing jolts
5. The motor guard E-stops on: MPC staleness (>200ms), motor overspeed,
   max deviation (>0.5 rev from feedback), workspace hard limits, NaN commands

See [safety.md](safety.md) for detailed documentation of each safety layer.
