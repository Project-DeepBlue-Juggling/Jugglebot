---
title: "FW 53 unit 6a (host): the real-fault exit stows whenever all six Legs are Closed-loop, whatever state the visit began in; the activate and deactivate observers name a returned rise and a held Leg at once instead of timing out; Stow-gate refusals read \"Platform not at Stow\"; /link_status carries legs_held and rig_bench"
type: feature
date: 2026-11-26
status: in-progress
phase: "FW 53 U6a"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - teensy_link/activate.py
  - teensy_link/deactivate.py
  - tests/ros/test_state_machine.py
  - tests/ros/test_teensy_bridge_node_fw53_host.py
  - tests/teensy_link/test_activate.py
  - tests/teensy_link/test_deactivate.py
subsystem: [orchestrator, ros, safety]
---

## Summary

The host side of FW 53 (I-FW-19, no Leg leaves Closed-loop above Stow). Host only; no
firmware change. Four changes:

1. **FAULT exit.** `FaultHandler._real_fault_exit` stows (profiled DEACTIVATE, hold FAULT
   until it succeeds) whenever `ctx.legs_closed_loop` holds: all six Legs Closed-loop with
   no active error. It no longer also requires the visit to have begun in ACTIVE or
   LEVELLING. `_LEGS_POSED_STATES` and `_legs_posed_at_entry` are deleted. Otherwise it
   goes to BOOT, where the one A6/A7 predicate decides as before.
2. **Observers.** `DeactivateMonitor` fails at once on `DIAG_FLAG_LEG_HELD` (reason prefix
   `HELD`). `ActivateMonitor` fails at once on HELD, and on an IDLE seen after Closed-loop:
   `RETURNED` (the last Closed-loop sample is within 0.15 rev of Stow) or `DROPPED` (above
   Stow). A Closed-loop position 0.10 rev below the run's highest extends that axis's budget
   to 15 s from that moment, so a late abort's return is not cut by the rise's own 20 s.
   `_run_activate` tracks the monitor's deadline and leads its message with "activate
   aborted, Platform returned to Stow" or "activate aborted, Legs held Closed-loop above
   Stow".
3. **Refusals.** `_call_rpc` takes `stow_gate=(axis, requested_state)` from
   `teensy_set_axis_state`, `teensy_reboot` and `teensy_sdo_write`. When the request gets
   ERR_REJECTED and `rpc_args.requires_platform_at_stow` covers it, the message ends with
   "Platform not at Stow: " + `STOW_REFUSAL_HINT`. The Armed hint now needs
   `refused_while_armed` for these methods, so a Leg IDLE refused while Armed is called a
   Stow refusal: IDLE is never Armed-gated.
4. **Homing and status.** `_run_home` logs the expected HOME refusal once per axis at INFO
   ("waiting for the previous Leg to read Idle"), then at DEBUG. The retry budget is
   unchanged (see the margin below). `/link_status` gains `legs_held` (Legs carrying
   LEG_HELD, or `none`) and `rig_bench` (0/1).

## Problem

FW 53 changed what an abort leaves behind, and the host read the old behaviour:
- **The FAULT exit stowed only when the visit began in ACTIVE or LEVELLING.** FW 53 can
  leave all six Legs HELD Closed-loop mid-air. A held Platform reached after a visit that
  began anywhere else went to BOOT with the Legs up, and only A7 kept BOOT from claiming
  IDLE.
- **The observers knew nothing of FW 53's outcomes.** `ActivateMonitor` read a returned
  rise (the Legs rise, come back down and go Idle) as nothing, and timed out after 20 s.
  It ignored HELD. `DeactivateMonitor` waited out its 20 s on a frozen descent (fw53-u3
  Finding 2; fw53_design.md section 5).

## Discussion

### Where the orchestrator is during each ladder

`IdleHandler` returns ACTIVE once the target fetch resolves, and `ActiveHandler.on_enter`
requests `activate`. The whole ACTIVATE rise therefore runs in **ACTIVE**, and a failed
activate returns FAULT from `ActiveHandler.execute` with `prev_state == ACTIVE`. The
`level_activate` rise runs in **LEVELLING**, whose failed op returns FAULT with
`prev_state == LEVELLING`. Both were already in `_LEGS_POSED_STATES`, so for the two rises
the old condition did not bite. It bit on visits that began elsewhere, for example a boot
timeout under a posed Platform, or a manual `/activate` from IDLE. Dropping it is the
condition the brief asked for. It also leaves one rule, "all six Closed-loop -> stow",
with no record of history to drift.

### The FAULT exit, walked per FW 53 abort outcome (to the operator's clear_errors)

Every walk starts with the bridge's activate reply (False) and ActiveHandler (or
LevellingHandler) returning FAULT. FAULT disarms (A2) and sets control_mode `ERROR`. If a
guard latch or an ODrive error is present it holds until the operator's `clear_errors`;
otherwise the exit runs on the next tick.

| Outcome (firmware) | What the bridge reports, and when | `legs_closed_loop` at the exit | Exit | Then |
|---|---|---|---|---|
| Mid-rise abort, Platform returned to Stow, Legs Idle | `RETURNED` once every observed Leg reads Idle (the monitor waits for the IDLE, it does not report when the descent starts) | False (six Idle) | BOOT | `is_homed` kept, A7 passes (six Idle, fresh): IDLE. The operator can retry `activate` |
| Pre-rise abort at Stow (Legs IDLEd at the bottom) | `RETURNED` "before the rise" | False | BOOT | IDLE as above |
| Frozen rise, or a pre-rise abort from a posed Platform: six HELD Closed-loop | `HELD` as soon as a Leg shows the bit after a clear one | True | profiled DEACTIVATE, hold FAULT | The firmware accepts it (no ladder busy, HELD cleared on accept), the six descend, BOOT, IDLE. A failed stow holds FAULT, and one retry per `clear_errors` |
| A Leg dropped mid-rise, the other five frozen HELD | `DROPPED` (or `active_errors`) for that Leg, `HELD` for the five | False (mixed) | BOOT | A7 (homed) or A6 (unhomed) holds BOOT with the existing mixed notice: support the Platform, press the E-stop button. The recovery for that case belongs to a later unit (R3/R2) |
| DEACTIVATE aborted above Stow (exit stow or ActiveHandler's) | `HELD` from DeactivateMonitor, within one Diagnostic of the freeze | True if all six still Closed-loop | the exit stow fails, FAULT holds, `clear_errors` retries once | as the row above, or mixed |

**Race: the activate reply versus `/robot_state`.** The bridge replies only after its own
diag cache shows every Leg Idle, and `/robot_state` is published from that cache at 100 Hz.
The FAULT exit runs at least one 10 Hz tick later. If it ever read a stale "all six
Closed-loop", it would request a DEACTIVATE of Legs at Stow. The firmware accepts that (the
ACTIVATE ladder has ended), arrives at once and IDLEs them through its gate. If it read a
mixed set, BOOT re-evaluates A7 every tick and passes once the six read Idle. Both outcomes
are benign.

**Why the monitor waits for the IDLE and does not report the abort when the descent
starts.** The returned rise keeps the ACTIVATE ladder busy for up to 10 s, and
`deactivate_request` answers ERR_REJECTED while `activate_active()`. An early report would
send FAULT into an exit stow the firmware refuses. That stow would fail, FAULT would hold
for a `clear_errors` that should not be needed, and the operator would see an alarming
refusal. Waiting costs at most the return time, and `RETURN_TIMEOUT_S` bounds it.

**Stowing Legs already at Stow.** The orchestrator has no Leg position it trusts in FAULT
(`legs_closed_loop` is drive state only), so the exit decides on drive state alone.
`deactivate_request` (leg_deactivate.cpp:202-232) does not refuse Legs already at Stow:
AXIS_ALL is not Stow-gated, and the MONITOR arrival test passes on the first tick, so the
gate IDLEs them. That costs one ladder, with no motion.

### Tradeoffs accepted

- **The exit stow now also fires after a boot timeout** when heartbeats then arrive showing
  all six Closed-loop (a relaunch under a posed Platform where BOOT first timed out). That
  is a profiled descent the operator did not command. Before this change, BOOT held with
  the A7 "stow it first with /deactivate" notice. A FAULT exit with no errors was already
  automatic for the ACTIVE and LEVELLING origins, and the descent is the safe direction.
  The relaunch case that does not time out still holds in BOOT and names `/deactivate`,
  unchanged.
- **HELD is honoured only after a clear bit** in the same run, the same pattern as the FW 36
  stall bit. The flag is sticky until the next accepted ladder, and the host cache can show
  the previous abort's bit for one tick. If a new freeze set the bit again before the host
  saw it clear, the observer would fall back to its timeout, which is the old behaviour.
- **DROPPED versus RETURNED** depends on the last Closed-loop sample before the IDLE. The
  firmware's Stow gate IDLEs only on an encoder-confirmed Stow, after a decelerating
  approach, so several Closed-loop samples land inside 0.15 rev (the same band as
  DeactivateMonitor's arrival latch). This is unchecked on hardware.

### Homing retry margin (scope item 3)

`_MAX_REJECT_RETRIES` = 20 at `poll_dt` = 0.05 s means at least 1.0 s of sleeps after the
first refusal; every RPC round trip only adds to that. The FW 53 refusal of HOME(Leg k+1)
lasts up to about 100 ms, about 210 ms with one lost heartbeat and about 310 ms with two
(fw53-u4). The margin is 1.0 / 0.31 = 3.2x, which is at least 3x, so the budget is
unchanged. `test_home_retry_budget_keeps_3x_margin_over_the_fw53_refusal` pins it.

## Fix

Listed in the Summary. One control-cycle note: nothing here touches a setpoint, the
40 Hz emitter or a feedforward path. The FAULT exit's only motion is the existing profiled
DEACTIVATE ladder in firmware, and the observers only read.

## Verification

- 2026-11-26, `flock /tmp/jugglebot-tests.lock python -m pytest -q -p no:cacheprovider -n 4 --dist loadfile tests/ros/test_state_machine.py tests/ros/test_state_machine_deactivate_confirm.py tests/ros/test_orchestrator_node.py tests/ros/test_orchestrator_conduit_integration.py tests/ros/test_teensy_bridge_node_*.py tests/teensy_link tests/ros/test_hand_lane_stall_host.py tests/firmware/test_stow_gate_xref.py tests/firmware/test_activate_xref.py tests/firmware/test_hand_stall_guard_xref.py tests/motion/test_hand_stream_bench_stall.py`: **1975 passed in 88.54 s**. This is the brief's scoped set plus the other importers of `teensy_link.activate` and `teensy_link.deactivate`, found by `grep -rln`.
- 2026-11-26, a mutation script (15 single-string mutations, one test file each, `pytest -x`): **15/15 caught**. They cover the FAULT exit restored to posed-only; the ActivateMonitor IDLE-after-Closed-loop branch, return extension and HELD branch, and HELD honoured without a clear bit; the same two HELD mutations in DeactivateMonitor; no `stow_gate` on SET_AXIS_STATE; the Stow hint on any rejection; the Armed hint by method only; `held` dropped from either status builder; the Hand in `legs_held`; no RETURNED summary; and the HOME refusal at WARN.
- Two existing tests encoded the old rule and were rewritten:
  `test_real_fault_not_from_active_exits_to_boot_at_once` became
  `test_real_fault_from_any_state_with_six_closed_loop_stows` (plus a not-all-Closed-loop
  twin), and `test_fault_exit_with_posed_legs_from_idle_does_not_claim_idle` became
  `..._stows_then_claims_idle`.

## Outcome

Host side of FW 53 U6a done on `offline/fw53`. Nothing changes on the robot until FW 53
is flashed and the live checkout is rebuilt. `ARMING_CONTRACT.md` choreography step 6
had one sentence the code made false ("stows the legs when the visit began in ACTIVE or
LEVELLING"); only that sentence is fixed. The full rewrite of that step and of
`INVARIANTS.md` belongs to a later unit (U7).

## Open Questions

- Is the automatic exit stow after a boot timeout (first tradeoff above) what the owner
  wants? Recommendation: yes. The alternative is an operator `/deactivate` from BOOT, which
  A7 already names.
