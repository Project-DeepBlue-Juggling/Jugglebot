# Safety Architecture

Motor command safety is enforced at **three independent layers**. Each layer
can prevent dangerous commands on its own — the defence-in-depth design means
a bug in any single component cannot cause the platform to slam.

## 1. Motor guard telemetry gating

**File:** `jugglebot/motion/motor_guard.py`

The standalone motor guard process only publishes telemetry (which contains motor
position/velocity/torque commands) when its mode is `ENABLED`. In `DISABLED`
or `ESTOP` mode the process is completely silent — no IPC messages leave the
process.

The motor guard also enforces safety checks on every MPC command and every
interpolation cycle:

| Check | Threshold | Action |
|-------|-----------|--------|
| MPC command NaN/Inf | Any non-finite value | Reject command |
| MPC command staleness | > 200 ms | E-stop |
| Motor feedback staleness | > 150 ms | Suppress commands |
| Motor overspeed | > max_vel × 1.1 | E-stop |
| Max deviation from feedback | > 0.5 rev | E-stop |
| Workspace hard limit (leg stroke) | < hard_min or > hard_max | E-stop |
| Workspace hard limit (cond number) | > 2.0× home | E-stop |
| IPC heartbeat timeout | > 500 ms | E-stop |

**Why this matters:** On startup the motor guard is `DISABLED` and produces no
output. Without gating, stale or zero commands would be forwarded to the CAN
node and interpreted as "retract all legs to 0".

## 2. Motion bridge command gating

**File:** `jugglebot/motion_bridge_node.py`

The ROS2 bridge node tracks a `_motor_guard_enabled` flag that mirrors
whether it last sent an `enable` or `disable` IPC command to the motor
guard. The bridge only publishes to `leg_lengths_topic` (the motor command
topic consumed by the CAN node) when this flag is `True`.

Diagnostic-only topics (`leg_torques_diagnostic`, `motion/tracking_error`,
`motion/diagnostics`) are not gated — they cannot command motors.
(`leg_torques_diagnostic` is also published by `trajectory_node` on the
trajectory path — same type/semantic — so it carries the gravity feedforward
regardless of which producer is live; still diagnostic-only, still not gated.)

### Control modes and the motor guard

| Mode | Motor guard | Notes |
|------|------------|-------|
| `SPACEMOUSE` | enabled | MPC plans path from spacemouse targets |
| `GUI` | enabled | MPC plans path from GUI targets |
| `LEVELLING` | **stays disabled** | Uses CAN node's profiled gentle-move; no MPC needed |

There is **no `CATCH` mode** (retired 2026-07-20 —
`logbook/2026-07-20-reload-action-catch-latch.md`). Catching is driven by the
**`jugglebot/reload` action** for its duration via a **catch-armed latch** on
`trajectory_node` (`trajectory/arm_catch`, mirrored on the `catch/armed` topic that
gates the hand); a reload runs within the already-enabled **TRAJECTORY** motor-guard
context, so it needs no dedicated mode row.
| `ERROR` | e-stopped | Immediate e-stop command sent |
| `''` (empty) | disabled | Idle / deactivated |

**LEVELLING** is a special case: the levelling sequence uses the CAN node's
`_gently_move_to_setpoint()` trapezoidal profiles for all platform movement.
The motor guard is not involved. The gravity offset computed during levelling
is received by the MPC bridge node via the `gravity_offset` ROS2 topic and
composed into every outgoing target orientation — the correction applies
automatically to all input modes.

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

**Threshold rationale:** The motor guard interpolates at 500 Hz with a maximum
motor velocity of ~7.3 rev/s, giving a worst-case per-cycle step of ~0.015 rev.
The 0.2 rev threshold provides ~13× headroom for normal operation while catching
catastrophic jumps (e.g., the original bug produced a 2.57 rev step).

### What triggers a rejection

- Stale zero commands from a disabled motor guard (the original incident)
- Sign errors or unit mismatches in a new command source
- Race conditions where two command sources fight for motor control
- Any software bug that produces an implausible position target

### Log output on rejection

```
[ERROR] Leg command REJECTED: leg 3 would step 2.574 rev (limit 0.2).
        Commanded=0.0000, actual=2.5741
```
