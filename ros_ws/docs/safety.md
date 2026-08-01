# Safety Architecture

**Re-framed 2026-08-01** for the MVP topology (the Phase 3 dormancy of
`plans/active/refactor-2026-07.md` stopped launching the MPC-era chain this
file used to describe; the old text is preserved below as the dormant-chain
record). Constants and thresholds are deliberately NOT duplicated here —
each layer points at the single place its numbers live, so this file cannot
drift out of date the way its predecessor did.

## The live leg path and its safety layers

```
trajectory_node → :5557 (ZMQ) → teensy_bridge_node → setpoint_pump (UDP)
   → can-bridge Teensy (500 Hz leg_interp) → CAN → ODrives
```

Defence in depth, four independent layers — each can stop a dangerous
command on its own:

1. **Arming contract (bridge, Jetson side).** The bridge refuses to relay
   motion unless the arming state machine allows it, and a guard E-STOP
   latches until an explicit `CLEAR_ERRORS`. Normative doc:
   `ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md`; enforcement lives in
   `teensy_bridge_node.py` (arming section, ~1954–2245).
2. **Per-frame step clamp (`teensy_link/setpoint_pump.py`).** Each outgoing
   frame's commanded `u0` is gated against the **previously accepted**
   setpoint (not encoder feedback): a leg stepping more than `max_step_rev`
   rejects the frame. Threshold + semantics: the module docstring and
   `DEFAULT_MAX_STEP_REV` in that file. This is the port of the retired CAN
   node's step limit — the rationale it inherited is recorded in the dormant
   section below.
3. **Firmware deviation guard (can-bridge Teensy).** The `MAX_DEVIATION`
   latch E-STOPs when a leg's commanded-vs-feedback deviation crosses the
   configured bound; the latch event is snapshotted for forensics
   (`Teensy_code_canbridge/fault_machine.h` — MAX_DEVIATION latch-event
   comment block; interpolation-side handling in `leg_interp.cpp`). **This is
   the leg-path safety authority in the MVP topology** (owner-confirmed,
   2026-07-31). A HAND fault E-STOPs the legs too.
4. **Bus-health command gate (can-bridge Teensy).** `classify_command_gate()`
   / `jugglebot_commands_allowed()` (`Teensy_code_canbridge/can_buses.h`)
   refuse non-setpoint commands under *sustained* bus confinement (BUS_OFF
   instantly), sized so ordinary error-passive blips never flap the gate —
   see the 2026-07-29 logbook entry for why the gate keys on sustained
   confinement and why `classify_bus_health` stays separate.

Two standing invariants sit above all four layers:

- **All movements use profiled trajectories** — step position commands are
  never sent (CLAUDE.md, Critical Conventions). Profiled stow on shutdown is
  owned by `teensy_bridge_node`.
- **Control modes** gate which producer may command motion; the mode/
  streaming contract (including the catch-armed latch that replaced the
  retired `CATCH` mode on 2026-07-20) is specified in
  `ros_ws/docs/control_modes.md`, not here.

LEVELLING runs through the bridge's profiled gentle moves (relay path) — no
MPC involvement; the gravity offset it measures is applied per the levelling
contract, `ros_ws/docs/levelling_frame.md` (C-LEVEL-1).

## Dormant record — the parked MPC-era chain

> Nothing below runs today. `can_node` was deleted 2026-07-06 (SocketCAN
> decommission); `motor_guard` + `motion_bridge_node` stopped launching
> 2026-08-01 (MPC dormancy). The chain is parked, not deleted, for the MPC
> revival (`controller/`, `run_mpc.py`); this section is its safety design
> record.

**Chain:** `run_mpc.py → motor_guard (500 Hz) → motion_bridge_node →
leg_lengths_topic → can_node → ODrives`.

**Motor guard telemetry gating** (`jugglebot/motion/motor_guard.py`): only
publishes motor commands when `ENABLED`; silent in `DISABLED`/`ESTOP`. Its
per-cycle checks, as last shipped:

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

**Motion bridge command gating** (`jugglebot/motion_bridge_node.py`): only
published to `leg_lengths_topic` while its `_motor_guard_enabled` flag was
true; diagnostic-only topics were never gated. (`leg_torques_diagnostic` is
still published by `trajectory_node` on the live path — diagnostic-only,
unchanged.)

**CAN node position step limit** (deleted `jugglebot/can_node.py`; config
`jugglebot_operational.max_position_step_rev`): rejected any command stepping
a leg more than the configured bound from its encoder estimate. **Ported to
the live path** as `setpoint_pump`'s per-frame clamp (layer 2 above, with the
reference frame deliberately changed to prior-accepted-setpoint). The
original rationale, kept because it still sizes the live clamp's class of
catch: the guard interpolated at 500 Hz with ~7.3 rev/s max motor velocity
(~0.015 rev/cycle worst case), so a generous threshold still catches
catastrophic jumps — the original incident was a 2.57 rev step from stale
zero commands of a disabled motor guard; sign/unit errors and
two-producers-fighting bugs are the same class.
