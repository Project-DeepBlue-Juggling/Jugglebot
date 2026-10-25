---
title: "Refused DEACTIVATE left the orchestrator in IDLE with the robot ACTIVE: the wire-disarm wait now runs whoever disarmed, and ACTIVE -> IDLE is confirmed by the robot"
type: investigation
date: 2026-10-23
status: in-progress
phase: "two-ball-oval - rung 1 Sitting close-out"
related_plan: two-ball-oval.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - tests/ros/test_state_machine.py
  - tests/ros/test_state_machine_deactivate_confirm.py
  - tests/ros/test_teensy_bridge_node_deactivate.py
  - tests/ros/test_teensy_bridge_node_shutdown_stow.py
  - logbook/INDEX.md
  - ros_ws/docs/choreography.md
subsystem:
  - ros
tags:
  - safety
  - testing
---

# Refused DEACTIVATE: orchestrator said IDLE, robot stayed ACTIVE

## Summary

When the 2026-10-23 Sitting closed, the owner requested IDLE before Ctrl-C. The bridge answered
`DEACTIVATE rejected: DEACTIVATE: ERR_REJECTED`, but the orchestrator still entered IDLE.
The legs meanwhile stayed up in CLOSED_LOOP. The owner says this has happened before. The
launch logs show one earlier case, on 2026-10-19. Two defects combined:

1. **Why the firmware refused (bridge race).** The GUI's IDLE teardown disarms through
   `/set_setpoint_output false` and publishes `deactivate` once `/link_status` reports
   `mpc_active=0`. That field is the *host* flag, and it clears at once. The disarm itself
   is only staged until the next 10 Hz J->T heartbeat. `_run_deactivate_inner` waited for the
   firmware's arm-took bit (T2J bit3) only `if self._mpc_active`. After an external disarm
   that flag was already False, so the wait was skipped. The DEACTIVATE then reached
   firmware that still had `s_mpc_active=1`, and the firmware refused it
   (`leg_deactivate.cpp:136`, `fault_mpc_active()` -> ERR_REJECTED).
2. **Why the machine claimed IDLE (orchestrator).** `ActiveHandler.execute` returned IDLE as
   soon as it got the *command* (`state_machine.py`, the old `if cmd == 'deactivate': return
   RobotState.IDLE`). It fired the deactivate from `on_exit`, and `IdleHandler` waited only
   for `operation_pending` and ignored `operation_result`. A refused deactivate therefore
   ended in IDLE.

## Evidence (launch logs)

| Sitting | disarm (`setpoint output DISARMED`) | `deactivate: ACTIVE -> IDLE` | gap | result |
|---|---|---|---|---|
| 2026-10-19 11:11 | 670.542 | 670.607 | 65 ms | ERR_REJECTED |
| 2026-10-19 retry | 676.708 | 676.807 | 100 ms | stowed |
| 2026-10-23 17:40 | 621.701 | 621.741 | 40 ms | ERR_REJECTED (log line 3842) |
| 2026-10-23 retry | 626.913 | 627.042 | 130 ms | stowed 1.2 s |

(`~/.ros/log/2026-10-19-11-04-03-078330-jetson-3068207/launch.log`,
`~/.ros/log/2026-10-23-17-30-31-087871-jetson-428635/launch.log`.)

In both Sittings the refused attempt came less than one heartbeat period (100 ms) after the
disarm, and the accepted attempt came at least one period after it. No skill goal was
running at 17:40:21: the `oval_1ball` goal had COMPLETED at 17:40:12.99 and `go_home`
(2.0 s) began at 17:40:19.39 and ended about 17:40:21.39. The bridge logged no
`deactivate: TRAP_TRAJ` line, so the firmware refused before any leg moved. The re-ACTIVATE
at 17:40:24.75 reported "platform at active pose, 6 legs (0.0 s)", which confirms the
legs had never left the active pose.

Firmware refusal conditions for DEACTIVATE (`deactivate_request`, `leg_deactivate.cpp:126-145`):
ERR_BUS_DOWN when `!deactivate_allowed()`. ERR_REJECTED when homing or an activate is
active, when a fault stow is pending, when `fault_mpc_active()` holds, or when a deactivate is
already running or latched. ERR_BAD_ARGS when no target leg is present. At 17:40:21 only
`fault_mpc_active()` fits: there was no cold-start move, no fault and no prior deactivate.

## Discussion

**Ruled out.** A skill goal, a Hand stroke or a BB feed still in flight: the goal finished
8.7 s earlier, and none of those would set `s_mpc_active`. `go_home` still running: the
second attempt at 17:40:27.04 landed *during* a go_home (started 26.42) and was accepted,
because the firmware gate is the arm bit, not motion. A firmware defect: the reject is
the documented backstop (ARMING_CONTRACT A3) doing its job. **No firmware change is needed.**

**The class (one level up).** "The orchestrator records a transition the robot has not
confirmed." Every handler-driven operation was checked:
- ACTIVATE: `ActiveHandler` waits for `operation_result` and goes to FAULT on False. Confirmed.
- Arm (`arm_setpoints`): bounded retries, then FAULT. Confirmed.
- LEVELLING phases (including `level_deactivate`): each phase waits for its result; False
  goes to FAULT. Confirmed.
- FAULT exit stow: `_resolve_stow` holds FAULT on a failed stow. Confirmed.
- HOMING: confirmed through the action result.
- **ACTIVE -> IDLE: the only unconfirmed transition.** Fixed here.
- The bridge-side race class ("a cold-start verb fired before the disarm reaches the
  firmware") also covers ACTIVATE and HOME, which share the `fault_mpc_active()` gate.
  Neither path is reachable within 100 ms of a disarm today: both run from IDLE or HOMING,
  long after the ACTIVE teardown. They are left unchanged and noted here.

**Why stay ACTIVE plus retry, and not a bridge fix alone.** The bridge fix removes today's
cause. Without the state-machine fix, though, any future refusal (ERR_BUS_DOWN, a deactivate
already busy, a lost ack) would still produce the same false IDLE. Owner rule: a genuine
refusal keeps the machine ACTIVE; a transient one is retried; the reported state always
reflects the robot.

**Retry safety (hardened on the owner's audit decisions, 2026-10-23).** DEACTIVATE is a
profiled TRAP_TRAJ lower to STOW followed by IDLE, so it moves the legs and drops torque.
The reply codes are not alike:
- **ERR_REJECTED = nothing latched.** `deactivate_request` returns before setting
  `s_start_req`, so a re-send is safe. It is retried up to 3 attempts, 1.0 s apart (more
  than one 10 Hz heartbeat).
- **ERR_TIMEOUT = a start may have latched.** The firmware's `deactivate_request` never
  returns ERR_TIMEOUT itself; on DEACTIVATE it is the client's `RpcTimeout`, meaning the
  request or the ack was lost. The bridge therefore does not report it as a refusal. It
  observes the legs through the same DeactivateMonitor window (20 s, or 6 s at shutdown).
  A descent that runs reports success; nothing moving reports a genuine
  `deactivate FAILED … (DEACTIVATE ack lost; observed, not re-sent)`. It is never re-sent
  blind, and the Hand is not de-energised under legs that never moved.
  - **Resting-position caveat.** If the descent finished before the monitor caught a
    CLOSED_LOOP sample at STOW, the bridge falls back to `_legs_already_stowed`, which
    requires every leg IDLE with |pos| <= `_STOW_POS_MAX_REV` (0.2 rev). A leg resting
    outside that band (foam) still reads as a failure. That is the fail-safe direction:
    the orchestrator's reconcile window then follows the legs out of closed loop to IDLE.
- An accepted deactivate that fails mid-descent, ERR_BUS_DOWN, ERR_BAD_ARGS, and "no reply
  within 30 s" are all genuine failures and are never retried.

**Reconciliation.** A genuine failure, a spent budget or no reply opens a 1 s reconcile
window (`DEACTIVATE_RECONCILE_S`). The machine reaches no verdict until the window ends:
- If the legs are reported out of closed loop (`ctx.legs_closed_loop`, derived from
  `/robot_state.motor_states`) for `LEGS_OUT_TICKS` = 5 consecutive 10 Hz ticks, the robot
  *is* idle and the machine follows it to IDLE.
- Otherwise it stays ACTIVE with the legs closed-loop and setpoint output disarmed (A3
  already disarmed it), and the reason is logged as a WARN.

Two further rules apply:
- **ACTIVE-wide rule.** The same N-tick rule applies throughout armed ACTIVE.
  /robot_state arrives at 100 Hz, so 5 ticks is about 50 consecutive samples; one
  glitched sample cannot trip it. It fires only after the legs have been *seen*
  closed-loop in this ACTIVE visit, so no claim is made without evidence.
- **Re-arm policy (owner).** There is no automatic re-arm: the operator asked for a
  teardown. Streaming mode switches are refused while the wire is disarmed
  (`ctx.wire_armed` False, `/link_status` seen) until the operator re-arms explicitly.
  This applies to auto-arm sessions only: a manual-arm session switches mode before its
  operator arm (runbook Sharp Edge #5).

**Legs-out exits request disarm + Hand IDLE only (second audit, owner decision).** Both
exits to IDLE driven by the legs leaving closed loop must leave the robot safe. These are
the ACTIVE-wide rule and the reconcile-to-IDLE. A plain transition would not be enough: the
wire could stay ARMED and the Hand energised under an IDLE label, and when IdleHandler
blanks the mode the stream stops. That is the SETPOINT_STALE race A4 exists to prevent. A
leg DEACTIVATE is also wrong here. It would re-energise legs that already fell out of closed
loop and lower them a second time.

The chosen request is the new `disarm_and_idle_hand`. It is two fire-and-forget calls to
existing bridge services and carries no leg command:
- `/set_setpoint_output false`, the bridge's disarm;
- `/set_hand_state IDLE`, which is SET_AXIS_STATE on axis 6 only.

The disarm clears the host flag at once and reaches the wire within one heartbeat (~100 ms),
inside the 250 ms SETPOINT_STALE window. Ruled out: a full DEACTIVATE gated on
`_legs_already_stowed`-equivalent evidence. That fallback was only needed if no
disarm-only path existed, and both services already do.

**Late replies (second audit, item 4).** The orchestrator puts no timeout on a service
future. It keeps a single `_pending_future`, and each new tracked dispatch replaces it, so
the old future is never read. A late reply therefore cannot be taken as a *later*
deactivate's answer. A late reply that arrives while no deactivate is pending is ignored:
ActiveHandler reads `operation_result` only in its arm and deactivate phases, and both
clear it before dispatching. One real gap remained: a reconcile-to-IDLE after no reply
left `operation_pending` True, which wedged IdleHandler until the dead call resolved. The
no-reply path now clears it. Request sequence numbers were not needed.

**Reply deadline.** 30 s, not ~15 s. The bridge's own budget for one deactivate is the
monitor's 20 s, plus its 5 s hard deadline, plus a wire wait of up to 1 s. A 15 s deadline
would call a live bridge silent while it is still watching a stalled descent. A test pins
the deadline against `teensy_link.deactivate.DEFAULT_TIMEOUT_S`. While the reply is pending,
only streaming mode switches are discarded (one WARN each). A duplicate `deactivate` is
absorbed, and every other command stays queued. The E-stop button and the Soft E-stop do
not pass through this command queue at all.

**Control-system walk-through (one deactivate cycle, fixed).** Tick N: the `deactivate`
command arrives. ActiveHandler stays ACTIVE, `control_mode` stays STANDBY (A4), and the
orchestrator dispatches the service. On the bridge, the host is already disarmed, so
`_wait_wire_disarmed` polls T2J bit3 every 20 ms. That takes at most one heartbeat
(<= 100 ms, budget 1 s). DEACTIVATE then fires and the descent runs for ~1.3 s while the
trajectory emitter keeps streaming the hold into a disarmed wire (ignored, and the
staleness watchdog is inert). The reply arrives with success; tick N+k returns IDLE.
`on_exit` skips the redundant second request. IdleHandler blanks the mode and the emitter
stops. Nothing new enters the 40 Hz or 500 Hz loops: the only added latency is up to
one heartbeat before a lower that is already slow and profiled.

## Fix

- `teensy_bridge_node.py` `_run_deactivate_inner`:
  - `_wait_wire_disarmed(wire_wait_s)` now runs unconditionally. Before, it ran only inside
    `if self._mpc_active:`.
  - ERR_TIMEOUT now falls through to the DeactivateMonitor (`lost_ack`) instead of
    returning `DEACTIVATE rejected`. The already-stowed fallback still applies, and the Hand
    is not idled when no leg moved.
  - `_run_deactivate` takes a new `wire_wait_s` (default 1.0). `_shutdown_stow` passes 0.0,
    because `on_shutdown` has already settled and waited, so the ~8 s teardown is kept. The
    wire-wait error text now reports `wire_wait_s` instead of a hard-coded 1 s.
  - A monitor loop that exits on the hard deadline with no verdict now returns
    `deactivate FAILED — no verdict (hard deadline)`. It previously fell through to success.
- `state_machine.py` `ActiveHandler`:
  - The confirmed deactivate runs in three phases: `waiting`, then `spacing`, then
    `reconcile`.
  - Constants: `DEACTIVATE_MAX_ATTEMPTS = 3`, `DEACTIVATE_RETRY_SPACING_S = 1.0`,
    `DEACTIVATE_RECONCILE_S = 1.0`, `DEACTIVATE_REPLY_DEADLINE_S = 30.0`,
    `LEGS_OUT_TICKS = 5`.
  - New: the ACTIVE-wide legs-out rule, the disarmed-wire mode-switch refusal, and the
    command filter used during a deactivate.
  - `deactivate_failure_is_retryable` retries only `ERR_REJECTED` and `not ready`.
  - `on_exit` re-requests the deactivate only on a non-confirmed exit (ACTIVE -> FAULT).
    The two legs-out exits request `disarm_and_idle_hand` instead (`_exit_disarm_only`).
  - The no-reply path clears `ctx.operation_pending`.
  - `Context` gains `operation_message` and `operator_notices` (via `notify()`).
  - The IdleHandler A4 comment is reworded.
- `orchestrator_node.py`:
  - Stores the service message in `ctx.operation_message`.
  - Logs each `ctx.operator_notices` entry as a WARN from a single call site (Foxy
    severity rule).
  - The `_COMMANDS_ACCEPTED` comment now notes the discard during a deactivate.
  - Adds the `_hand_state_client` (`set_hand_state`) and dispatches the new
    `disarm_and_idle_hand` request.
- `ARMING_CONTRACT.md`: A4 and choreography step 5 are rewritten to match.

## Verification

- Red/green: `python -m pytest -q -p no:cacheprovider "tests/ros/test_teensy_bridge_node_deactivate.py::test_run_deactivate_waits_for_wire_after_an_external_disarm"`
  (2026-10-23). With the wire wait gated behind `self._mpc_active` again: **1 failed**.
  With the fix: **1 passed**.
- (2026-10-23, `python -m pytest -q -p no:cacheprovider -n 4 --dist loadfile tests/ros/test_state_machine.py
  tests/ros/test_state_machine_deactivate_confirm.py tests/ros/test_orchestrator_node.py
  tests/ros/test_orchestrator_conduit_contract.py tests/ros/test_orchestrator_conduit_integration.py
  tests/ros/test_teensy_bridge_node_deactivate.py tests/ros/test_teensy_bridge_node_shutdown_stow.py
  tests/ros/test_teensy_bridge_node_conduit.py tests/ros/test_teensy_bridge_node_hand.py
  tests/ros/test_teensy_bridge_node_recover_lane.py tests/ros/test_gui_geometry.py
  tests/ros/test_teensy_bridge_node_hand_jam.py`): **537 passed**. This run is after the
  second-audit changes; the first hardening pass gave 535.
- The full gate, `./run_tests.sh`, is run by the orchestrating session before the commit.
  It is not part of this entry's evidence.
- NOT FLOWN. Deploy: `colcon build --packages-select jugglebot` and a full launch restart.
  The flight check is the IDLE teardown from the GUI. Expect no `deactivate failed` line, and
  a `deactivate: ACTIVE -> IDLE` line *after* `deactivate: platform stowed`.

## Open (owner)

- The GUI teardown's verify gate (`ros_ws/gui/js/state-minimap.js`) reads the host
  `mpc_active` KeyValue. `teensy_mpc_active` (T2J bit3) is the firmware's own state. Gating
  on it would close the race GUI-side too. The bridge fix makes this optional.
- DECIDED (owner, 2026-10-23): no automatic re-arm; streaming mode switches are refused
  while the wire is disarmed. Still open: should the published state carry a distinct
  marker (e.g. a GUI badge for "ACTIVE, disarmed")? Today it stays `ACTIVE:<mode>`.
