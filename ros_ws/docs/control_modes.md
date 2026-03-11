# Control Modes and Command Flow

## Overview

Motor position commands can reach the ODrives through two independent paths.
Only one should be active at a time — the architecture enforces mutual
exclusion through control mode gating.

### Path 1: Motion planner (control loop → bridge → CAN node)

```
control_loop.py  →  IPC (ZeroMQ)  →  motion_bridge_node.py  →  leg_lengths_topic  →  can_node.py
   500 Hz              PUB/SUB            500 Hz poll              ROS2 topic          CAN bus
```

Used in: `SPACEMOUSE`, `SHELL`, `GUI` modes.

The standalone control process runs IK + dynamics at 500 Hz and publishes
motor commands (position, velocity feedforward, torque feedforward) via IPC.
The motion bridge node polls these and publishes to `leg_lengths_topic`. The
CAN node subscribes and sends `set_input_pos` commands to each ODrive.

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
  3. Motion bridge: does NOT enable control loop (LEVELLING excluded)
  4. CAN node: activate_or_deactivate service → gentle move to activation height
  5. Settle → read inclinometer → publish gravity offset → persist
  6. CAN node: deactivate → gentle move to 0
  7. State machine transitions to IDLE, control_mode = ''
```

The control loop stays `DISABLED` throughout. The gravity offset is stored in
the control loop via `set_gravity_offset` IPC and will be applied when the
control loop is next enabled.

### SPACEMOUSE / SHELL / GUI (active modes)

```
IDLE → ACTIVE:
  1. State machine sets ctx.control_mode = 'SPACEMOUSE' (or SHELL/GUI)
  2. Orchestrator publishes control_mode_topic
  3. CAN node: activate_or_deactivate service → gentle move to activation height
  4. Motion bridge: sends 'enable' to control loop
  5. Control loop: seeds home pose, begins publishing telemetry
  6. Motion bridge: forwards telemetry to leg_lengths_topic
  7. CAN node: receives and sends motor commands at 500 Hz
```

### ERROR

```
Any state → FAULT:
  1. Orchestrator publishes control_mode_topic = 'ERROR'
  2. Motion bridge: sends 'estop' to control loop, sets _control_loop_enabled = False
  3. CAN node: gentle move to 0 (stow)
```

## Safety invariants

1. The control loop never publishes telemetry unless `mode == ENABLED`
2. The motion bridge never publishes to `leg_lengths_topic` unless
   `_control_loop_enabled == True`
3. The CAN node rejects any `leg_lengths_topic` command that would step a leg
   by more than `JB_OP_MAX_POSITION_STEP_REV` (0.2 rev) from its current
   encoder position
4. The CAN node's `_gently_move_to_setpoint()` always seeds the ODrive with
   the current encoder position before entering closed-loop, preventing jolts

See [safety.md](safety.md) for detailed documentation of each safety layer.
