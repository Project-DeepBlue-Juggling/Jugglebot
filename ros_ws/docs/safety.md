# Safety Architecture

Motor command safety is enforced at **three independent layers**. Each layer
can prevent dangerous commands on its own — the defence-in-depth design means
a bug in any single component cannot cause the platform to slam.

## 1. Control loop telemetry gating

**File:** `jugglebot/motion/control_loop.py`

The standalone control process only publishes telemetry (which contains motor
position/velocity/torque commands) when its mode is `ENABLED`. In `DISABLED`
or `ESTOP` mode the loop is completely silent — no IPC messages leave the
process.

**Why this matters:** On startup the control loop is `DISABLED` and its
internal position buffer is initialised to zeros. Without gating, those zeros
would be forwarded to the CAN node and interpreted as "retract all legs to 0".

## 2. Motion bridge command gating

**File:** `jugglebot/motion_bridge_node.py`

The ROS2 bridge node tracks a `_control_loop_enabled` flag that mirrors
whether it last sent an `enable` or `disable` IPC command to the control
process. The bridge only publishes to `leg_lengths_topic` (the motor command
topic consumed by the CAN node) when this flag is `True`.

Diagnostic-only topics (`leg_torques_diagnostic`, `motion/tracking_error`,
`motion/diagnostics`) are not gated — they cannot command motors.

### Control modes and the control loop

| Mode | Control loop | Notes |
|------|-------------|-------|
| `SPACEMOUSE` | enabled | Normal operation — motion planner active |
| `SHELL` | enabled | Manual CLI commands through motion planner |
| `GUI` | enabled | GUI-driven pose commands |
| `LEVELLING` | **stays disabled** | Uses CAN node's profiled gentle-move; no IK needed |
| `ERROR` | e-stopped | Immediate e-stop command sent |
| `''` (empty) | disabled | Idle / deactivated |

**LEVELLING** is a special case: the levelling sequence uses the CAN node's
`_gently_move_to_setpoint()` trapezoidal profiles for all platform movement.
The control loop is not involved. The gravity offset computed during levelling
is stored in the control loop via the `set_gravity_offset` IPC command, which
works regardless of enabled/disabled state — the offset is applied on next
enable.

## 3. CAN node position step limit

**File:** `jugglebot/can_node.py`
**Config:** `hardware_config.yaml` → `jugglebot_operational.max_position_step_rev`
**Constant:** `JB_OP_MAX_POSITION_STEP_REV` (default: 0.2 rev ≈ 14 mm)

Last-resort safety net in the CAN node's `_sub_leg_lengths()` callback. Before
sending any `leg_lengths_topic` command to the ODrives, the CAN node compares
each commanded position against the leg's current encoder estimate. If **any**
leg would jump by more than `MAX_POSITION_STEP_REV` from its current position,
the **entire** command is rejected and an error is logged.

This check runs only when encoder feedback is available
(`motors.first_heartbeat_received`). It does not affect the CAN node's own
`_gently_move_to_setpoint()` or `_send_position_target()` calls, which bypass
this subscriber callback entirely.

**Threshold rationale:** The control loop runs at 500 Hz with a maximum motor
velocity of 15 rev/s, giving a worst-case per-cycle step of 0.03 rev. The
0.2 rev threshold provides ~7× headroom for normal operation while catching
catastrophic jumps (e.g., the original bug produced a 2.57 rev step).

### What triggers a rejection

- Stale zero commands from a disabled control loop (the original incident)
- Sign errors or unit mismatches in a new command source
- Race conditions where two command sources fight for motor control
- Any software bug that produces an implausible position target

### Log output on rejection

```
[ERROR] Leg command REJECTED: leg 3 would step 2.574 rev (limit 0.2).
        Commanded=0.0000, actual=2.5741
```
