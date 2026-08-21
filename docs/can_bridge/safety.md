# Safety Mechanisms

**Source files:**

- [`fault_machine.cpp`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.cpp) / [`.h`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/fault_machine.h) — the fault state machine and CAN3 watchdog
- [`leg_interp.cpp`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp) — lead/stroke clamps, NaN/Inf rejection
- [`can_buses.cpp`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/can_buses.cpp) — per-bus health classification
- [`canbridge_config.h`](../../ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h) — every threshold cited below
- [`docs/motion_planner/safety.md`](../motion_planner/safety.md) — the historical Jetson-side (`motor_guard.py`/`can_node.py`) version of this same logic, which the firmware below ports

This firmware is a port of `can_node.py`'s fault-handling
(`_handle_error`/`_watchdog_check`/`_fault_response`) and `motor_guard.py`'s
staleness/overspeed/max-deviation guard, run natively on the Teensy instead
of on the Jetson's Linux scheduler. One behaviour was deliberately hardened
beyond the original: the guard E-STOP (staleness / overspeed / max-deviation)
now **latches** — the legacy code auto-cleared the instant a transient
passed and silently re-armed 500 Hz streaming with no operator involved,
which was judged unsafe.

## Staleness / watchdog layer

Two independent link-liveness detectors and two independent bus-liveness
detectors run concurrently.

| Guard | Timeout | Effect |
|---|---|---|
| MPC command staleness | 250 ms (`MPC_CMD_STALENESS_US`) | **Latches** guard E-STOP (`MPC_STALE`) |
| Motor (encoder) feedback staleness | 150 ms (`MOTOR_FB_STALENESS_US`) | Suppresses output only — deliberately **not** latched, "recoverable" |
| CAN3 leg heartbeat (bus-down detector) | 2.0 s (`CAN_HEARTBEAT_TIMEOUT_US`) | `CAN_BUS_DOWN`, arms the deferred-stow latch |
| Jetson↔Teensy UDP link | 500 ms (10 Hz heartbeat × 5 missed, `JETSON_LINK_TIMEOUT_US`) | `LINK_LOST`, output disabled (not latched) |
| Bus-partner presence (anti-bus-off) | 5.0 s (`BUS_PARTNER_STALENESS_US`) | Withholds *all* TX on that bus pre-emptively — never transmit into a bus with no recently-seen partner |
| Reboot-in-progress suppression | 6.0 s (`REBOOT_WATCHDOG_SUPPRESS_US`) | Prevents a deliberate `REBOOT_ODRIVES` RPC's silence window from being misread as a real CAN loss |

Before the 250 ms MPC-staleness E-STOP actually latches, the interpolator's
own extrapolation ladder already softens the command: Taylor extrapolation
for the first 50 ms of silence, then a velocity ramp to zero over the next
60 ms — so a brief MPC hiccup decays smoothly rather than jerking the leg,
well before the hard stop fires. See [Control Flow](control.md) for the
full ladder.

On total command loss, once output is gated off, the Teensy does not
command a deceleration — the leg ODrives autonomously hold their last
`CLOSED_LOOP` setpoint via their own internal position controller. This
"never command into a fault, let the ODrive hold" pattern is deliberate —
see the CAN-bus-down row below for the reasoning.

## E-STOP / fault enumeration

`FaultState` values: `NONE`, `MPC_STALE`, `LINK_LOST`, `MOTOR_OVERSPEED`,
`MAX_DEVIATION`, `ODRIVE_FATAL`, `CAN_BUS_DOWN`, `MOTOR_FB_STALE`.
Priority when multiple conditions are true simultaneously (highest wins):
`CAN_BUS_DOWN > ODRIVE_FATAL > {latched guard reason} > MOTOR_FB_STALE > LINK_LOST > NONE`.

| Condition | Threshold | Latches? | Clears via |
|---|---|---|---|
| MPC setpoint stale | 250 ms | **Yes** | Explicit `CLEAR_ERRORS` RPC only |
| Motor overspeed (measured, not commanded) | 16.5 rev/s (`MAX_MOTOR_VEL_RPS` = 110% of the 15.0 rev/s trap-vel limit) | **Yes** | Explicit `CLEAR_ERRORS` RPC only |
| Max deviation (commanded `u0` vs. encoder) | 1.0 rev (`MAX_DEVIATION_REV`; raised 0.5 → 1.0 on 2026-07-16 after legitimate tracking lag latched it at vel = 200 mm/s — see the logbook entry of that date) | **Yes** | Explicit `CLEAR_ERRORS` RPC only |
| Active ODrive error (any of 7 axes, legs **and** hand) | — | Self-clears if the axis reports clean; one auto soft-reset attempt (`MAX_SOFT_RESET_ATTEMPTS = 1`); once that budget is spent, requires `CLEAR_ERRORS` | |
| Disarmed while closed-loop (any axis) | — | Unconditionally fatal, same clear path as above | |
| CAN3 bus down (leg heartbeats stale) | 2.0 s | Auto-clears via the deferred-stow sequence once the bus is *confirmed* reconnected; ends in `IDLE`, not resumed motion | |
| Motor feedback stale | 150 ms | Not latched, recoverable | |
| Jetson link lost | 500 ms | Not latched, recoverable | |

The three conditions in the top three rows (stale MPC, overspeed,
max-deviation) share a single sticky **guard E-STOP** latch. `MAX_DEVIATION`
is the "impossible target" catch: if the commanded position diverges from
where the leg actually is by more than 1.0 rev, that's treated as a stale
zero, a sign error, or a runaway command source — not a real target — and
the platform stops rather than chasing it. (The guard watches the *raw* knot
command, before the lead clamp — so it counts command-space lag the executed
command never carries.)

### CAN-bus-down: the deferred-stow safety inversion

CAN3 loss is handled by a specific, hard-won invariant (preserved from an
earlier investigation into a can_node fault-response bug):

1. CAN3 down ⇒ **never** command the dead bus. Output gates off; the leg
   ODrives autonomously hold their last `CLOSED_LOOP` setpoint.
2. On loss detection, **arm** a deferred-stow latch — do not act yet.
3. Execute the stow **only once the bus is confirmed restored** (fresh leg
   heartbeats): a profiled, velocity-limited descent to the retracted pose.
4. If the stow half-completes and the bus drops again, re-arm the latch.
5. If the bus never returns, the terminal fail-safe is simply leg `IDLE` —
   never a blind command into a bus with no listener.

The rationale is that a bus that's merely *slow to reconnect* should not be
treated the same as one that's *permanently gone*; guessing wrong in either
direction (stowing too eagerly vs. never stowing) has a real hardware cost,
so the machine only acts on confirmed state.

## Joint / velocity / current / impossible-target limits

Validated continuously in the 500 Hz command path (see
[Control Flow](control.md#joint-limits-are-enforced-on-the-interpolated-output-not-the-target)
for the clamp details): a lead clamp bounding the commanded position to
within 0.10 rev of the live encoder (0.15 → 0.10 on 2026-07-10), and a stroke clamp to each leg's
measured physical hard-stop range. A non-finite (NaN/Inf) command is
dropped at ingest rather than clamped. The final wire encode additionally
clips position to the ODrive's own absolute range (4.2 rev for legs,
**10.8 rev** for the hand — the operator-measured metal contact; this read
11.1 rev, 0.3 rev *past* metal, until the 2026-08-18 correction, and the tightened
clamp is live on a bridge flashed from `mvp-trajectory-bringup` at **FW 15** or
later. An FW 14 board still passes setpoints up to 0.3 rev past metal, and the two
are wire-identical, so read `link_status/bridge_fw_version` rather than inferring
from a healthy link) as a last-line backstop.

!!! note "Known gap: one-shot RPC paths bypass these checks"
    `SET_ABSOLUTE_POSITION` and `SDO_WRITE` are dispatched straight to the
    CAN bus with only an axis-index and bus-gating check — no `isfinite`
    check, no range clamp. These are rare, one-shot configuration-time
    operations rather than the continuous drive path, so the exposure is
    limited, but a malformed or NaN value in either RPC's argument would
    currently reach the wire unchecked.

## Thermal / overcurrent protection

**Delegated entirely to the ODrive.** The Teensy does no threshold-checking
of its own on `temp_fet`, `temp_motor`, or current — it relays the ODrive's
own fault flags through the generic active-error path above, and forwards
temperature purely as telemetry (an on-change threshold there governs only
*when to report*, not a safety trip).

### Homing

The one place the Teensy itself reads live current for a decision: homing
uses an EMA of measured current to detect the physical hardstop during a
velocity-limited move-to-hardstop. This is a *completion* detector, not an
overcurrent trip — the ODrive's own current limit (configured with
headroom above the detection threshold) is what actually protects the
motor if the leg pushes past the stop.

## CAN bus health monitoring

Two genuinely independent detectors of "is CAN3 down" run in parallel:

- A **heartbeat-based** detector (the fault machine, above) — drives the
  deferred-stow sequence.
- A **register-level** detector reading the FlexCAN controller's bus-off /
  error-passive bits directly — gates *new* commands (RPC config writes,
  platform-Teensy relay reads) separately from the streaming path.

This dual-detector design is why a since-resolved "marginal CAN3" scare
turned out to be benign: the raw error counter was counting harmless
bus-idle phase-flips, not real wire errors. It has since been relabeled and
a generalized presence gate (the `BUS_PARTNER_STALENESS_US` row above) was
added across all three buses so no un-ACKed frame is ever transmitted into
a bus with nothing listening.

## Cross-propagation between subsystems

- A **hand fault does propagate** to an E-STOP of the legs — the same
  active-error scan covers all 7 axes (6 legs + hand), so a hand ODrive
  error trips the platform-wide guard.
- A **stale hand heartbeat alone does not** propagate — the CAN3
  heartbeat/deferred-stow watchdog is scoped to the 6 leg axes only, on the
  reasoning that there is nothing to stow on the hand.
- No path was found where a leg-specific condition (overspeed,
  max-deviation, CAN3-down) propagates into a hand-specific fault beyond
  the platform-wide guard-mode/output-gate effects already covered above —
  hand trajectory commands are gated separately, on bus-liveness and a
  homing interlock, not on the guard E-STOP latch.

!!! note "Known gap: the platform Teensy itself has no dedicated liveness check"
    The physical platform Teensy (distinct from the hand ODrive) is not
    tracked as a discrete entity — its only CAN3 traffic the bridge
    processes is a couple of relay-reply IDs and a traffic-report
    broadcast that isn't decoded. If the platform Teensy froze while the
    leg ODrives kept heartbeating normally, nothing in this firmware would
    detect or react to it.
