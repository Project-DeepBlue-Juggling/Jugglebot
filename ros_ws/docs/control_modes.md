# Control Modes and Command Flow

## Overview

Motor position commands can reach the ODrives through two independent paths.
Only one should be active at a time — the architecture enforces mutual
exclusion through control mode gating.

### Path 1: MPC → motor guard → bridge → CAN node — **REMOVED 2026-09-01**

> Not runnable as drawn since 2026-08-01, and most of it no longer exists.
> `can_node` was deleted 2026-07-06 (SocketCAN decommission);
> `jugglebot_launch.py` stopped starting `motor_guard` and `motion_bridge_node`
> on 2026-08-01 (MPC dormancy); and on **2026-09-01 the MPC chain was deleted
> outright** — `run_mpc.py`, `HardwarePlant`, `motion_bridge_node` and
> `mpc_bridge_node` are gone, preserved at git tag **`mpc-final`**
> (`logbook/2026-09-01-mpc-chain-removed.md`). `motor_guard.py` survives as a
> parked fallback with neither feeder nor consumer. Path 2 is the live leg path;
> Path 1 is now a historical record, and there are no longer two live paths to
> keep mutually exclusive.

```
MPC (50 Hz)  →  HardwarePlant  →  motor_guard.py  →  IPC (ZeroMQ)  →  motion_bridge_node.py  →  leg_lengths_topic  →  can_node.py
                    :5557            500 Hz interp        :5556            500 Hz poll              ROS2 topic          CAN bus
```

Was used in: `STANDBY`, `SPACEMOUSE`, `GUI` modes.
`STANDBY` accepted commands from a manually-launched `run_mpc.py` only —
the MPC bridge forwarded no ROS2 input-source targets in this mode. Both
`run_mpc.py` and the MPC bridge were removed 2026-09-01 (tag `mpc-final`).

> **Catching is not a control mode.** There is no `CATCH` mode — it was retired
> 2026-07-20 (`logbook/2026-07-20-reload-action-catch-latch.md`). The reactive catch
> is driven by the **`jugglebot/reload` action** for its duration via a **catch-armed
> latch** on `trajectory_node` (`trajectory/arm_catch`, mirrored on the `catch/armed`
> topic that gates the hand). A reload runs from **ACTIVE + streaming a hold in
> TRAJECTORY**, armed — it raises the latch only for the ball's flight window, which lets
> `catch/dynamic_target` reach `planner.build_catch` and actuate the platform, then
> lowers it and re-centers.

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
  6. Motor guard: remains DISABLED (it had no enabler after the 2026-09-01
     MPC removal — `run_mpc.py` was the only one)
```

In STANDBY the motors are held in closed-loop at the activation pose by the
ODrives (seeded by the CAN node's activation profile).  The platform was
responsive only to a manually-launched MPC process — historically:

```bash
# historical — run_mpc.py was removed 2026-09-01 (tag mpc-final)
python run_mpc.py --pose 0,0,220,0,0,0 --duration 10
```

`HardwarePlant.enable()` sent `disable+enable` to the motor guard on startup
(the leading `disable` cleared any lingering ESTOP), and `disable` on shutdown.
No ROS2 input source (spacemouse, GUI) could influence the platform while in
STANDBY — this eliminated ambiguity about command origin during hardware
testing.  With the MPC chain removed there is no STANDBY command source at all.

### SPACEMOUSE / GUI (input-routing sub-modes)

```
ACTIVE:STANDBY → ACTIVE:SPACEMOUSE (or GUI):
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

> The three caveats below are **historical** — `run_mpc.py`, `HardwarePlant`,
> `motion_bridge_node` and `mpc_bridge_node` were removed 2026-09-01 (git tag
> `mpc-final`; `logbook/2026-09-01-mpc-chain-removed.md`). They are retained as
> the record of how the removed chain behaved. The live :5557 publisher is
> `trajectory_node`, whose single-binder interlock makes a second publisher loud
> rather than silent.

**Two `run_mpc.py` processes must never have run simultaneously.** Both
published commands on tcp://127.0.0.1:5557 and the motor guard would see
interleaved commands from two planners, causing jerky or unsafe motion.  There
was no safeguard — it was enforced manually.

**`run_mpc.py` had to be Ctrl-C'd before clicking Deactivate.** Deactivating
while an MPC process was still running had the CAN node drive the platform to
STOW via its direct path while the motor guard was still receiving MPC commands
for the old target.  Within a few hundred ms the motor guard tripped
`MAX_DEVIATION` and self-ESTOPped.  This was harmless (the motion bridge gated
topic forwarding in IDLE and the next `run_mpc.py` launch cleared ESTOP via its
`disable+enable` cycle), but it left the motor guard in a fault state visibly
logged in telemetry.

**Motor-guard enable was owned by `HardwarePlant`.**  Launching `run_mpc.py`
enabled the motor guard; Ctrl-C disabled it.  The orchestrator's
`control_mode_topic` was a target-routing signal (routing ROS2 input sources
to the MPC via `mpc_bridge_node`); it did not gate motor_guard enable/disable.
The only mode message that bridge still sent to the motor guard was `estop` on
`control_mode = 'ERROR'`.
