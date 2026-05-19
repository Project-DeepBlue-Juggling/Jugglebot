---
title: CAN-loss fault-response safety inversion — blanket "ERROR ⇒ stow+IDLE" de-energises healthy holding actuators on reconnect
type: investigation
date: 2026-05-19
status: tuned
#
# Status ladder (for investigation entries):
#   open         — nothing done yet
#   in-progress  — diagnosis done, fix/verification still ongoing
#   tuned        — symptom scoped to this entry addressed + verified, open sibling
#   resolved     — every symptom in scope addressed and verified
#
phase: "hardware-bringup"
related_plan: "hardware-bringup.md"
related_entries:
  - 2026-05-19-findingb-motor-guard-estop-latch-observability
  - 2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap
  - 2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation
sessions:
  - mpc_20260519_200613.csv
files_changed:
  - ros_ws/src/jugglebot/jugglebot/can_node.py
  - tests/ros/test_can_node.py
  - sim/analysis/known_issues.yaml
  - sim/analysis/log_index.json
commits:
  - d8bab95
subsystem:
  - can
  - safety
  - motion
tags:
  - hardware-bringup
  - safety
  - can
  - fault-response
  - safety-inversion
  - investigation
  - contract-gap
  - open-question
---

# CAN-loss fault-response safety inversion — blanket "ERROR ⇒ stow+IDLE" de-energises healthy holding actuators on reconnect

## Summary

On a transient CAN loss during MPC runtime, the platform was **safe
while the bus was down** (the leg ODrives stay `CLOSED_LOOP(8)` and
autonomously hold their last commanded setpoint — proven forensically
in the
[z=30 entry](2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md)
and the
[Finding B entry](2026-05-19-findingb-motor-guard-estop-latch-observability.md):
zero ODrive disarm/error/CLOSED_LOOP-exit across a real CAN loss) but
**de-energised on reconnect**: the orchestrator FAULT → `control_mode='ERROR'`
path drove `can_node._sub_control_mode('ERROR')` →
`_gently_move_to_setpoint(0.0, deactivating=True)`, idling all legs.
An unstable held pose therefore collapses on recovery — a **safety
inversion** (the recovery is more dangerous than the fault).

Root cause (one level up from the instance): a **single blind
fault-response policy** — *"`control_mode=='ERROR'` ⇒ stow+IDLE"* —
applied to a **heterogeneous fault set**. A comms/non-actuator fault
where the ODrives are demonstrably fine and holding demands the
*opposite* of stow.

Fixed by making the fault response **discriminate on actuator
integrity** at one canonical enforcement point, and — per operator
direction at the FIX_PLAN gate — never commanding a stow into a
down bus: any genuine stow is **deferred until the watchdog confirms
the bus restored**.

Status `tuned`: the safety inversion (the symptom in scope) is
addressed and verified; one sibling is intentionally left open (the
frozen-telemetry residual / the deferred >2 s-dropout hardware
validation — see Open Questions, tied to Finding B Recommendation 4).

## Symptoms

Diagnosed session `mpc_20260519_200613.csv` (verdict **FAIL** in
`sim/analysis/log_index.json`), rosbag
`~/Desktop/rosbags/2026-05-19_20-05-46`. The MPC session data itself
is **clean**: telemetry went stale on the unplug and the P4/P5
`telemetry_stale` watchdog fired correctly at 0.524 s > 0.5 s,
clean-exit (Exit 3) — `mpc_20260519_200613.log`. The FAIL is the
**system-level CAN-loss recovery behaviour**, not the MPC: motors
commanded IDLE on the CAN-fault → ERROR path, leaving the platform
de-energised on reconnect.

Operator-observed: the platform holds fine during the disconnect, then
**collapses the instant the cable is replugged**.

## Diagnosis

### Forensic timeline (precondition pinned)

The open precondition before the fix was: *which step actually issues
the reconnect IDLE — the ERROR-mode stow, or a FAULT→BOOT recovery
re-init (`_setup_odrives_steps('IDLE')`)?* Resolved by decoding the
rosbag `2026-05-19_20-05-46_0.mcap` (hand-rolled CDR, throwaway
`/tmp/probe_caninv_decode.py`, **not committed** per the CLAUDE.md
empirical-probe discipline; cross-validated — the decoded
`has_fatal_can_error` edges line up exactly with the `error[]` string,
proving the variable-length parse is correct). Timeline, relative to
bag start:

| t (s) | Event |
|------:|-------|
| ~31.5 | CAN cable physically unplugged (inferred: 2.0 s before watchdog) |
| 33.493 | `has_fatal_can_error 0→1`; `error[]=['Fatal CAN bus issue.']` (watchdog, `_HEARTBEAT_TIMEOUT_S=2.0 s`) |
| 33.501 | `control_mode STANDBY→ERROR`; `orch_state ACTIVE:STANDBY→FAULT` |
| **33.535–33.621** | **legs `current_state` 8→1 (CLOSED_LOOP→IDLE), staggered over 86 ms** |
| 35.411 | `has_fatal_can_error 1→0`; `error[]=[]` (watchdog `attempt_restore_steps` succeeded) |
| 35.500 | `orch_state FAULT→BOOT` |
| 35.601 | `orch_state BOOT→IDLE` (already homed → skips HOMING) |

Throughout, every axis reports `active_errors=0` and `disarm_reason=0`
— **no ODrive ever errored or disarmed**, consistent with the
Finding B Phase-1 candump forensics.

### The reconnect IDLE is the ERROR-mode stow, not BOOT re-init

Two independent proofs:

1. **Timing.** Legs idle 34–120 ms *after* `control_mode='ERROR'`
   (33.501 s) and **~1.9 s before** `FAULT→BOOT` (35.500 s). The BOOT
   path cannot be the cause — it happens after the legs are already
   idle.
2. **Call graph.** `_setup_odrives_steps('IDLE')` is only ever called
   from `_home_robot_steps` (homing). `FaultHandler→BootHandler` with
   an already-homed robot skips straight to `IDLE`
   ([state_machine.py:240](../ros_ws/src/jugglebot/jugglebot/state_machine.py)) —
   `_setup_odrives_steps` is **never entered** in this recovery.
   Definitively ruled out.

The single fault-stow trigger is therefore
[`can_node._sub_control_mode`](../ros_ws/src/jugglebot/jugglebot/can_node.py)'s
`if msg.data == 'ERROR':` branch. The orchestrator's *only* action on
this fault is to publish `control_mode='ERROR'`
([FaultHandler.on_enter](../ros_ws/src/jugglebot/jugglebot/state_machine.py)) —
and MPC-crash / bridge-crash route through the *same* `FaultHandler`,
so this one branch is the whole generalised class.

### Secondary forensic note (sub-path within the ERROR stow)

`has_fatal_odrive_error` stays `0` for the entire event. `_emergency_idle()`
sets `motors.fatal_error=True`, which would surface as
`has_fatal_odrive_error=1`. It never does ⇒ the IDLE was **not** the
`_gentle_move_steps:1534` `fatal_can_error` short-circuit to
`_emergency_idle()`; it was the generator's own `deactivating` IDLE
blast (`for axis in JUGGLEBOT_AXES: encode_set_state(axis,'IDLE')`).
Both sub-paths are downstream of the *same* `_gently_move_to_setpoint(
0.0, deactivating=True)` call — so guarding that one call site covers
both regardless of which sub-path the bus-timing selects.

## Discussion

CLAUDE.md makes the Discussion non-negotiable here on three counts: an
open precondition was resolved forensically (and the resolution
ruled out the *a priori* more-likely candidate), a non-obvious
tradeoff was accepted, and the chosen fix beat a reasonable
alternative for reasons not inferable from the code.

### 1. Why the rejected "defer/suppress `fatal_can_error`" framing was wrong

An earlier framing proposed making `fatal_can_error` *not* fire (or
suppressing it) on a short CAN bounce. The operator rejected it as a
**dishonest hack that overloads the flag**: a CAN loss *is* a CAN
error — `fatal_can_error` reporting it is correct, honest behaviour.
This is the *same* root-cause logic as Finding A's P5 and Finding B's
bridge fix: **fix the consumer that mis-handles an honest signal, not
the honest producer.** Suppressing `fatal_can_error` would also have
blinded every *other* consumer of that flag (the orchestrator's FAULT
classification, the diagnostic path) to a real bus event. The defect
is not that the fault is *reported*; it is that a single response is
applied to faults that demand opposite responses.

### 2. Climbing one level — the class, not the instance

The instance is "CAN loss collapsed the platform on reconnect once."
The class is: **any fault that routes to `control_mode='ERROR'` while
the leg ODrives are healthy and holding triggers a de-energising stow
that is strictly less safe than doing nothing.** CAN loss is one
member; an MPC crash, an mpc/motion-bridge crash, or any orchestrator
FAULT with intact actuators are others (all route through the same
`FaultHandler`). The fix is therefore a **discriminator on actuator
integrity**, not a CAN-specific special-case: one helper
(`_actuators_intact_and_holding`), one policy method
(`_fault_response`), two call sites (the `ERROR` branch and the
unknown-mode catch-all) routed through it — one canonical enforcement
point in the contracts-over-patches sense.

### 3. The discriminator trusts frozen telemetry — why that is correct *here*

`_actuators_intact_and_holding` reads `motors.last_states`, which
during a CAN-down window is the **frozen last-good heartbeat**. The
Finding A / B body of work established that frozen/stale telemetry is
*the* hazard when it is used to drive motion. Here the logic is
**inverted into the safe direction**: frozen-good state is used only
to *withhold* a de-energising action, never to command motion.

- Frozen-good (legs were CLOSED_LOOP, error/disarm-free) ⇒ **hold**.
  The ODrives autonomously keep their setpoint with zero CAN traffic
  (Finding B Phase-1 candump: forensic proof). Platform stays up.
  Safe.
- Any *pre-freeze* degradation (a leg that left CLOSED_LOOP or
  errored before the last decode) is captured *in* that last decode ⇒
  discriminator returns False ⇒ existing stow fail-safe. Safe.

The only uncovered edge is a leg leaving CLOSED_LOOP **during** the
bus-down window (after the last decoded heartbeat); `_sub_control_mode`
will not re-evaluate because `'ERROR'` is de-duped. Finding B's
Phase-1 forensics established the ODrives do **not** spontaneously
leave CLOSED_LOOP on CAN loss (no ODrive-side CAN watchdog
configured), so this edge is **not physically realised on current
hardware**. It is a real residual *assumption*, recorded honestly
(Open Questions; `known_issues.yaml`) and tied to Finding B
Recommendation 4 (the deferred controlled >2 s-dropout hardware
test). This is the accepted, bounded cost of the fix; it is strictly
safer than today's *unconditional* collapse.

### 4. Why defer the stow to reconnect (operator FIX_PLAN modification)

The originally-approved plan stowed immediately in the non-intact
case. The operator amended it: **the stow must only begin AFTER
reconnection.** Root cause this prevents: a profiled stow commanded
into a down bus is best-effort — frames are dropped and the IDLE
lands *non-deterministically whenever the cable happens to come back*
(exactly the "best-effort during the bus-down window, possibly
landing on reconnect" hazard the precondition named). Deferring to
the watchdog's `attempt_restore_steps` success (the *single
deterministic* "bus confirmed rx+tx OK" signal in the code) means the
stow is a clean, fully-delivered profiled move or it does not happen
at all — and if the just-restored bus re-drops mid-profile the stow
re-arms for the next watchdog-confirmed reconnect rather than being
silently abandoned half-delivered (`_stow_pending_on_reconnect` is
cleared and `stowed_due_to_error` set only on a *successful* deferred
move). If the bus never returns, `_watchdog_check`'s existing
restore-**failure** `_emergency_idle()` is the terminal best-effort
fail-safe (unchanged), and the deferred profiled stow is correctly
abandoned (it is impossible without a bus). The deferral execution
point was a design choice not specified by the operator; it is
justified by being the *only* deterministic bus-restored signal —
recorded here so a future reader can challenge that premise rather
than the mechanism.

### 5. The unknown-mode branch was folded into the same policy

The operator also asked to guard the unknown-mode catch-all
(`else: … Stowing`). It had the *identical* blanket-stow defect for
the identical reason (an unexpected/garbled mode string while the
actuators are fine and holding should not collapse the platform).
Routing it through the same `_fault_response` keeps the policy in one
place — resisting the "two nearly-identical stow sites" drift that
contracts-over-patches exists to prevent.

## Fix

Single enforcement point in
[`ros_ws/src/jugglebot/jugglebot/can_node.py`](../ros_ws/src/jugglebot/jugglebot/can_node.py):

| Element | Change |
|---------|--------|
| `__init__` | New `self._stow_pending_on_reconnect = False` latch (beside `stowed_due_to_error`). |
| `_actuators_intact_and_holding()` (new) | True iff all 6 legs `current_state==CLOSED_LOOP` **and** every leg `active_errors==0` and `disarm_reason==0`, read from `motors.last_states`. |
| `_fault_response(reason)` (new) | The policy: intact ⇒ **hold** (no stow, no `stowed_due_to_error`); degraded **and** `fatal_can_error` ⇒ **defer** (`_stow_pending_on_reconnect=True`, no command into a down bus); degraded **and** bus healthy ⇒ stow now (`_gently_move_to_setpoint(0.0, deactivating=True)`, `stowed_due_to_error=True`). |
| `_sub_control_mode` | `ERROR` branch and unknown-mode `else` branch both now call `_fault_response(...)` (was: direct `_gently_move_to_setpoint` + `stowed_due_to_error`). |
| `_watchdog_check` | Restore-**success**: after clearing `fatal_can_error`, if `_stow_pending_on_reconnect` execute the deferred profiled stow now (bus confirmed rx+tx OK). Restore-**failure**: clear the latch (the existing `_emergency_idle()` is the terminal fail-safe; a profiled stow is impossible without a bus). |

The watchdog restore-failure `_emergency_idle()` path is otherwise
**untouched** — genuine unrecoverable CAN loss still fails safe
exactly as before.

## Verification

Per the CLAUDE.md (date, exact command, result) triple rule:

- Regression suite added to
  [tests/ros/test_can_node.py](../tests/ros/test_can_node.py): new
  `TestCanLossFaultResponse` (9 cases: ERROR-intact ⇒ hold;
  ERROR-degraded-bus-down ⇒ defer; `disarm_reason` ⇒ degraded;
  unknown-mode ⇒ same policy; watchdog restore-success ⇒ deferred stow
  executes; restore-failure ⇒ latch cleared + `_emergency_idle`;
  deferred stow runs only after `fatal_can_error` cleared
  (clear-before-move ordering invariant); a failed deferred stow
  re-arms the latch and leaves `stowed_due_to_error` False;
  end-to-end intact→reconnect ⇒ never stows) plus the reframed
  `test_error_mode_degraded_leg_healthy_bus_stows` (was
  `test_error_mode_calls_gentle_move`, now explicit about the
  fail-safe path).
- Isolated, post-`/audit` fixes (`pytest tests/ros/test_can_node.py -q`,
  run 2026-05-19): **80 passed in 2.97 s** (the 9 `TestCanLossFaultResponse`
  cases + the reframed degraded-leg test + the pre-existing
  `tests/ros/test_can_node.py` suite).
- Authoritative full pre-commit gate, post-`/audit` fixes
  (`pytest tests/ -q`, run 2026-05-19): **1427 passed, 1 xfailed in
  434.00 s, exit 0 — fully clean.** 1427 = 1424 prior baseline + 1
  (the documented load-induced `test_hot_loop_allocation_contract`
  tracemalloc flake passed this run rather than flaking) + 2 new
  ordering/re-arm tests. The 1 xfailed is the pre-existing inherited
  T-U-T1a-4 `Restoration_Failed` permanent xfail (unrelated).
- Pre-`/audit` gate (`pytest tests/ -q`, run 2026-05-19):
  **1424 passed, 1 failed, 1 xfailed in 423.35 s** — the single failure
  was the **documented load-induced
  `tests/sim/test_hot_loop_allocation_contract.py` tracemalloc-baseline
  flake** (same class as the z=30 entry's Verification; that test builds
  a `MuJoCoPlant` and never executes a line of `can_node` — structurally
  impossible for this ROS2 change to have caused it). Confirmed by
  isolation re-run (`pytest tests/sim/test_hot_loop_allocation_contract.py
  -q`, run 2026-05-19): **3 passed in 15.88 s**, and the flake did not
  recur in the authoritative post-`/audit` run above.

## Outcome

The safety inversion is **closed and verified**: a CAN loss (or any
FAULT→ERROR) with all legs CLOSED_LOOP and error/disarm-free now
**holds** the pose — the ODrives keep their setpoint autonomously and
the platform no longer collapses on reconnect. A genuine degraded-leg
fault still stows, but **never into a down bus** — the stow is
deferred to the watchdog-confirmed reconnect, or, if the bus never
returns, the existing terminal `_emergency_idle()` fail-safe applies.
The system is now safe **both** during the disconnect and on
recovery. Commit hash backfilled after the COMMIT gate.

## Open Questions

- **Leg leaving CLOSED_LOOP *during* a bus-down window** (Discussion
  §3). Not physically realised on current hardware (no ODrive-side
  CAN watchdog — Finding B Phase-1 forensic proof), but an unvalidated
  assumption. Tied to **Finding B Recommendation 4** — the deferred
  controlled **>2 s** CAN-dropout hardware test, which would also
  exercise can_node's untested
  watchdog/`attempt_restore_steps`/`fatal_can_error` recovery path.
  Recorded in `known_issues.yaml` (`CAN_LOSS_FAULT_INVERSION`).
- **Deferred-stow execution point** (Discussion §4). Placed at the
  watchdog restore-success because it is the only deterministic
  "bus confirmed restored" signal. If a future change introduces an
  earlier authoritative bus-restored signal, the deferral should move
  to it.

## Related

- [2026-05-19-findingb-motor-guard-estop-latch-observability.md](2026-05-19-findingb-motor-guard-estop-latch-observability.md)
  — Phase-1 candump forensic proof that ODrives stay CLOSED_LOOP with
  zero disarm/error across a real CAN loss (the load-bearing premise
  of this fix's discriminator). Shares the deferred >2 s-dropout test
  (Recommendation 4) as this entry's open sibling.
- [2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md](2026-05-18-z30-solve-failure-motor-pos-none-watchdog-gap.md)
  — Finding A / P5: the consumer-side telemetry-validity contract;
  same root-cause logic (fix the consumer that mis-handles an honest
  signal, not the honest producer).
- [controller/PLANT_INTERFACE_CONTRACT.md](../controller/PLANT_INTERFACE_CONTRACT.md)
  — P5 telemetry-validity (orthogonal layer: P5 governs the MPC's view
  of plant state; this entry governs can_node's fault-response policy).
- `ros_ws/src/jugglebot/jugglebot/can_node.py` —
  `_fault_response` / `_actuators_intact_and_holding` (new),
  `_sub_control_mode` (ERROR + unknown-mode branches), `_watchdog_check`
  (deferred-stow execution / latch clear).
- Session `temp/logs/mpc_20260519_200613.{csv,log}`; rosbag
  `~/Desktop/rosbags/2026-05-19_20-05-46/2026-05-19_20-05-46_0.mcap`
  (forensic decoder throwaway `/tmp/probe_caninv_decode.py`, not
  committed).
