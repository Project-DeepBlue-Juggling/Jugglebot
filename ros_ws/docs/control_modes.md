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

Used in: `STANDBY`, `SPACEMOUSE`, `GUI`, `CATCH` modes.
`STANDBY` accepts commands from a manually-launched `run_mpc.py` only —
the MPC bridge forwards no ROS2 input-source targets in this mode.

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

### STANDBY (default sub-mode on activation)

```
IDLE → ACTIVE:
  1. CAN node: activate_or_deactivate service → gentle move to activation height
  2. State machine enters ACTIVE with active_mode = STANDBY
  3. Orchestrator publishes control_mode_topic = 'STANDBY'
  4. Motion bridge: sets _motor_guard_enabled = True (gates leg_lengths_topic
     publishing); does NOT send enable to motor guard
  5. MPC bridge: does NOT forward any ROS2 input-source targets
  6. Motor guard: remains DISABLED until a run_mpc.py process enables it
```

In STANDBY the motors are held in closed-loop at the activation pose by the
ODrives (seeded by the CAN node's activation profile).  The platform is
responsive only to a manually-launched MPC process — e.g.:

```bash
python run_mpc.py --pose 0,0,220,0,0,0 --duration 10
```

`HardwarePlant.enable()` sends `disable+enable` to the motor guard on startup
(the leading `disable` clears any lingering ESTOP), and `disable` on shutdown.
No ROS2 input source (spacemouse, GUI, catch) can influence the
platform while in STANDBY — this eliminates ambiguity about command origin
during hardware testing.

### SPACEMOUSE / GUI / CATCH (input-routing sub-modes)

```
ACTIVE:STANDBY → ACTIVE:SPACEMOUSE (or GUI/CATCH):
  1. User clicks sub-mode button in GUI → orchestrator_command published
  2. State machine sets ctx.control_mode = 'SPACEMOUSE' (or other)
  3. Orchestrator publishes control_mode_topic
  4. Motion bridge: _motor_guard_enabled stays True; no mode command sent
  5. MPC bridge: receives mode, begins forwarding targets to MPC on :5558
  6. MPC solves, HardwarePlant (if running) sends commands to motor guard
  7. Motor guard: interpolates at 500 Hz, publishes telemetry on :5556
  8. Motion bridge: forwards telemetry to leg_lengths_topic
  9. CAN node: receives and sends motor commands
```

Returning to STANDBY from any sub-mode:

```
ACTIVE:SPACEMOUSE → ACTIVE:STANDBY:
  1. User clicks Standby button
  2. MPC bridge sends mode=disabled to MPC on :5558
  3. MPC holds at last commanded pose — no snap to activation pose
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

## Operational caveats

**Do not launch two `run_mpc.py` processes simultaneously.** Both publish
commands on tcp://127.0.0.1:5557 and the motor guard will see interleaved
commands from two planners, causing jerky or unsafe motion.  There is no
safeguard currently — enforce manually.

**Ctrl-C `run_mpc.py` before clicking Deactivate.** If you deactivate while
an MPC process is still running, the CAN node drives the platform to STOW
via its direct path while the motor guard is still receiving MPC commands
for the old target.  Within a few hundred ms the motor guard trips
`MAX_DEVIATION` and self-ESTOPs.  This is harmless (motion bridge gates
the topic forwarding in IDLE and the next `run_mpc.py` launch clears ESTOP
via its `disable+enable` cycle), but it leaves the motor guard in a fault
state visibly logged in telemetry.

**Motor-guard enable is owned by `HardwarePlant`.**  Launching `run_mpc.py`
enables the motor guard; Ctrl-C disables it.  The orchestrator's
`control_mode_topic` is a target-routing signal (routes ROS2 input sources
to the MPC via `mpc_bridge_node`); it no longer gates motor_guard
enable/disable.  The only mode message this bridge still sends to the motor
guard is `estop` on `control_mode = 'ERROR'`.
