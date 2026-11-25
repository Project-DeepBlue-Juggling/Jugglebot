---
title: "FW 53 unit 6b (host): the mixed-case recovery R3, after one Leg dropped out by itself with five holding the Platform up the FAULT exit re-engages it where it stands and stows all six, only when its position reference is proven kept (one pure predicate, leg_reference.py); otherwise the E-stop-button remedy with the reason named; a refused reboot no longer clears the homed flag"
type: feature
date: 2026-11-26
status: in-progress
phase: "FW 53 U6b"
related_plan: ""
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/leg_reference.py
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - tests/ros/test_leg_reference.py
  - tests/ros/test_state_machine.py
  - tests/ros/test_orchestrator_node.py
  - tests/ros/test_teensy_bridge_node_coldstart.py
  - tests/firmware/native/test_leg_deactivate.cpp
subsystem: [orchestrator, ros, safety]
---

## Summary

The "mixed case": the Platform is up, one Leg's drive disarmed itself on an error, the other
five hold, the can-bridge guard latched. Until now the FAULT exit sent that to BOOT, which
held and told the operator to support the Platform and press the E-stop button. Owner
decision 2026-11-25: "If the dropped Leg provably kept its reference, then yes: re-enable it
where it is and stow all six together. Otherwise, we can keep with the current remedy."

1. **The predicate.** `jugglebot/leg_reference.py` (pure): `LegDropWatch` records, per Leg,
   the evidence from the moment it stops reading Closed-loop with no error, and
   `reference_unproven(...)` returns None (proven) or the reason it is not.
   `mixed_case_verdict(...)` applies it to every Leg that is not Closed-loop.
2. **The recovery.** `FaultHandler._real_fault_exit`: once the operator's `clear_errors` has
   cleared every error and the guard latch, a mixed set (some Legs Closed-loop, the rest
   Idle) whose every dropped Leg passes the predicate gets the same profiled DEACTIVATE as
   the all-six exit stow, once per FAULT visit. No new command.
3. **Not proven.** Unchanged behaviour (BOOT holds with the A7 notice), plus one notice
   naming each dropped Leg and why its reference is not proven.
4. **`_reboot_odrives`** clears `is_homed` and levelling only after the firmware accepted
   the reboot.

## The predicate as built

A Leg's reference counts as kept only if ALL of these hold (anything else, or any input
missing, is "not proven"):

| # | Evidence | Source | Why |
|---|---|---|---|
| 1 | `is_homed` true | `/robot_state.is_homed` | The bridge drops its local `is_homed` at once on any `MISSING_ESTIMATE` or `INITIALIZING` edge on any homed axis (`reference_loss.py`, `_on_reference_loss`). A Hand-only loss therefore also refuses: conservative. |
| 2 | This orchestrator saw the Leg Closed-loop with no active error, and its length then | `LegDropWatch` | No baseline, no comparison (an orchestrator restarted after the drop never proves it). |
| 3 | The Leg now reads Idle, from fresh states (`LEG_STATES_MAX_AGE_S`) | `ctx.leg_states` | Any other state is a drive starting or calibrating. |
| 4 | No `/robot_state` gap over `LEG_STATES_MAX_AGE_S` since the drop | orchestrator receive clock | A gap could hide a reboot. |
| 5 | No sample since the drop in a state other than Idle or Closed-loop | `motor_states[i].current_state` | A drive boot passes through other states. |
| 6 | A nonzero `disarm_reason` was seen | `motor_states[i].disarm_reason` | An Idle Leg with no reason is unexplained (a reboot, or an outside IDLE). |
| 7 | The `disarm_reason` never went back to 0 before the operator's `clear_errors` | same, sealed by `FaultHandler` at the clear | `disarm_reason` is sticky until CLEAR_ERRORS; a drive reboot zeroes it (inferred: it is drive RAM; unchecked on hardware). |
| 8 | Every error bit seen (`active_errors \| disarm_reason`, ORed over the drop) is in `KEEPS_REFERENCE` | same | Table below. |
| 9 | Every length seen since the drop is within `REF_DRIFT_TOL_REV` = 0.25 rev (about 18 mm at 70.5 mm/rev) of the last Closed-loop length, and finite | `pos_estimate` | A frame shift after a re-seed or reboot is whole turns (the 2026-11-20 Hand jump was 5.00); 0.25 rev leaves 0.75 rev margin under one turn while allowing some sag. A sag past it refuses: conservative. |

Error bits (the repo's `odrive.ERROR_CODES`, ODrive 0.6 `ODriveError`):

| Bit | Name | Class | Reason |
|---|---|---|---|
| 1 | INITIALIZING | loses | drive (re)boot |
| 2 | SYSTEM_LEVEL | unknown | firmware-level fault |
| 4 | TIMING_ERROR | unknown | control loop overrun; estimator runs in the same loop |
| 8 | MISSING_ESTIMATE | loses | no position estimate |
| 16 | BAD_CONFIG | unknown | |
| 32 | DRV_FAULT | unknown | gate driver; may coincide with a brown-out |
| 64 | MISSING_INPUT | unknown | |
| 256 | DC_BUS_OVER_VOLTAGE | keeps | power stage; logic and encoder unaffected |
| 512 | DC_BUS_UNDER_VOLTAGE | unknown | can precede a brown-out reboot (the E-stop button path) |
| 1024 | DC_BUS_OVER_CURRENT | keeps | power stage |
| 2048 | DC_BUS_OVER_REGEN_CURRENT | keeps | power stage |
| 4096 | CURRENT_LIMIT_VIOLATION | keeps | motor current; the encoder is independent |
| 8192 | MOTOR_OVER_TEMP | keeps | thermal |
| 16384 | INVERTER_OVER_TEMP | keeps | thermal |
| 32768 | VELOCITY_LIMIT_VIOLATION | unknown | also the signature of an estimate jump |
| 65536 | POSITION_LIMIT_VIOLATION | unknown | also the signature of a frame shift |
| 16777216 | WATCHDOG_TIMER_EXPIRED | keeps | CAN axis watchdog, communications only |
| 33554432 | ESTOP_REQUESTED | keeps | commanded |
| 67108864 | SPINOUT_DETECTED | unknown | one field case kept continuity (2026-11-20, `reference_loss.py`), but a spinout is a disagreement between the commutation and the measured position, which is exactly the doubt; owner question |
| 134217728 | BRAKE_RESISTOR_DISARMED | keeps | power stage |
| 268435456 | THERMISTOR_DISCONNECTED | keeps | thermal sensor |
| 1073741824 | CALIBRATION_ERROR | loses | calibration sets the encoder offset |
| any other | (not decoded) | unknown | |

"Keeps" is reasoned from the ODrive's structure (the estimator does not depend on the power
stage or the axis state), not measured on this robot.

Inputs NOT obtainable on the orchestrator, therefore treated through the proxies above:
the drive's own heartbeat continuity (the firmware's `HB_STALE`/`HB_SEEN` reach the bridge's
Diagnostic cache and `/link_status` aggregates, not `/robot_state` per axis); a drive boot
counter or uptime (the ODrive publishes none the repo decodes); the Leg's last COMMANDED
length (trajectory_node and the bridge have it; the last measured Closed-loop length is
used, within the guard's tracking tolerance of it); a kinematic plausibility of the sagged
length against the five held lengths (feasible in principle, one Stewart degree of freedom,
but not built: the drift bound is the stricter, simpler test).

## Discussion

**Why the trigger stays the operator's `clear_errors`.** The disarm reason is what makes the
drop explainable, and CLEAR_ERRORS erases it. So the evidence is recorded continuously from
`/robot_state` and sealed at the operator's clear (item 7). The exit already waits for that
clear (a disarm with a Leg still Closed-loop is fatal, `disarm_while_cl`), and the
firmware's SETUP refuses an errored target, so the clear must come first anyway. A new
command would add an operator step with no new information.

**Once per FAULT visit.** A failed recovery is not retried automatically or by a second
`clear_errors`: a Leg that re-faults on re-engage is telling us the drive problem persists,
and re-dispatching a move on that is the blind retry the project rules out. If the failure
left all six Closed-loop (frozen), the existing all-six exit stow takes over on the next
`clear_errors`; if it left a mixed set, the E-stop-button remedy.

**Rejected: an explicit `clear_errors` inside the recovery.** The exit fires only once
every error is clear, so a second clear would be a no-op that costs a slot in the single
tracked operation.

**Rejected: proving the reference from the five held Legs' kinematics.** With five lengths
fixed the sixth moves along one degree of freedom, so a sagged length is not a single
expected value; the geometry would only bound it. The drift bound already refuses any sag
over 18 mm.

## What the firmware does with it (read at d110d44f, pinned by the new native test)

`leg_deactivate.cpp`, DEACTIVATE(AXIS_ALL) with Legs 0, 1, 3, 4, 5 Closed-loop at the pose
and Leg 2 Idle at a sagged length, errors cleared:

- `deactivate_request`: guard not ESTOP (the clear released it), no Homing/ACTIVATE, no
  deferred stow, no armed stream (the bridge disarms first, A3), AXIS_ALL not Stow-gated:
  accepted; HELD bits cleared.
- First tick: the Hand IDLEs (exempt) and leaves the ladder.
- SETUP, one Leg per 10 ms tick, Leg 0 first. For each Leg: an active error aborts the whole
  op (pre-descent abort: the gate refuses IDLE above Stow, nothing is sent, every Leg HELD);
  else `input_pos` = the MEASURED length `axes[i].pos_rev` (Leg 2: its sagged length, not the
  pose), TRAP_TRAJ limits (2.5 rev/s, 30 rev/s^2), POSITION/TRAP_TRAJ, CLOSED_LOOP. For the
  five that are already Closed-loop the seed replaces their held stream setpoint by their
  measured length (a step of their tracking error only). For Leg 2 the engage commands no
  motion: the target is where it stands. What it does physically: the drive starts with an
  empty velocity integrator under its share of the Platform weight, so it sags by its
  proportional-loop error until the integrator fills, then holds (unchecked on hardware:
  the size of that dip). If it was still sagging at the engage the seed is up to one encoder
  sample old (10 ms), so the drive pulls it back by v x 10 ms.
- COMMAND 10 ms (`CL_ENGAGE_SETTLE_US`) after the last SETUP: all six targets STOW (0.0
  rev) in one tick; each ODrive profiles its own Leg from its seed, so the six move
  together (not time-synchronised: different distances, as in every DEACTIVATE).
- MONITOR (`stow_descent_step`): a target whose heartbeat AFTER its CLOSED_LOOP command +
  10 ms reports anything but Closed-loop freezes the others at their stopping points
  (FAILED, HELD; nothing IDLEd); arrival of all six within 0.01 rev of Stow and settled IDLEs
  them through the gate; the 10 s timeout IDLEs only at Stow, else freezes.

## Walk: one recovery, and its failure branches

Nominal (Leg 2 disarms with CURRENT_LIMIT_VIOLATION at the Active pose, 2.19 rev):

1. Leg 2's drive disarms: IDLE, `disarm_reason` = 4096. Five hold their last stream input
   (the guard latches ODRIVE_FATAL and gates output). `LegDropWatch` opens Leg 2's record
   with last Closed-loop length 2.19 rev and the 4096 bit.
2. The bridge publishes `has_fatal_odrive_error` (`disarm_while_cl`); the orchestrator forces
   FAULT. `FaultHandler.on_enter`: real fault, `disarm_setpoints`, control mode ERROR.
3. The operator sends `clear_errors`. `FaultHandler` seals the open records, requests
   `clear_errors`; the bridge sends CLEAR_ERRORS(AXIS_ALL): drive errors, guard latch, HELD.
4. Next `/robot_state`: no errors, no fatal flag; `/link_status`: guard not latched. Leg 2
   reads Idle at, say, 2.10 rev. The exit sees a mixed set; the predicate passes (homed;
   baseline 2.19; Idle, fresh; no gap; no odd state; reason 4096 seen and held to the clear;
   4096 keeps; drift 0.09 rev). One notice: "re-engaging Leg 2 where it stands and stowing
   all six". Request `deactivate`; FAULT holds.
5. Bridge `_run_deactivate`: wire disarmed, not already stowed, DEACTIVATE(AXIS_ALL),
   `DeactivateMonitor` on all six (Leg 2 Idle before its first Closed-loop sample waits,
   it does not fail).
6. Firmware as above: Hand IDLE; SETUP 60 ms; Leg 2 held at 2.10; COMMAND; six descend
   (2.19 rev at 2.5 rev/s, 30 rev/s^2: about 1.0 s); arrival; six IDLE at Stow.
7. `DeactivateMonitor` sees each Leg Closed-loop at Stow then Idle: success. `FaultHandler`
   `_stow_done` -> BOOT -> homed, six Idle and fresh -> IDLE (the homed skip, A7 passes).
   The Platform is at Stow, lowered, not dropped; no Homing needed.

Failure branches (in every one nothing is IDLEd above Stow):

| Branch | Where the Platform ends | What the operator sees |
|---|---|---|
| The clear does not clear Leg 2's error (the fault persists) | Up, five holding, Leg 2 Idle | FAULT holds (the exit needs no errors); the error list names it. A second `clear_errors` is allowed; the record keeps every bit it saw. |
| Not proven (any predicate row fails) | Up, as before | One notice naming the Leg and the failed row, then BOOT's A7 mixed notice: support the Platform, E-stop button. |
| DEACTIVATE refused (ERR_BUS_DOWN guard re-latched, ERR_REJECTED a deferred stow pending, bus down) | Up, unchanged: nothing was sent | The stow failed; FAULT holds. The next `clear_errors` does not retry the mixed recovery (once per visit): BOOT with the E-stop notice. |
| Leg 2 re-faults on re-engage (its post-CL heartbeat reads Idle) | The five frozen at their stopping points inside [seed, Stow], Leg 2 free | `DeactivateMonitor` fails fast (dropped to IDLE before reaching STOW), FAULT holds. Between COMMAND and that heartbeat the five are descending; the exposure is one heartbeat period plus the 10 ms settle (unchecked: the Leg ODrive heartbeat period; at 30 rev/s^2 for 0.1 s about 0.15 rev, 10 mm) then their stopping distance (2.5^2/60 = 0.10 rev). E-stop remedy. |
| A SETUP error (Leg 2 errored again before its tick) | Up: the configured Legs hold their measured seeds, the rest their stream pose; all HELD | Stow failed with HELD; FAULT holds; E-stop remedy. |
| Descent timeout (10 s) above Stow | Frozen where they stand, HELD | Stow failed; the next `clear_errors` finds all six Closed-loop and runs the existing all-six exit stow. |

## Fix

See Summary. Code: `leg_reference.py` (new), `state_machine.py` (`Context.leg_drops`,
`FaultHandler` seal and mixed branch), `orchestrator_node.py` (feeds the watch),
`teensy_bridge_node.py` (`_reboot_odrives`), `ARMING_CONTRACT.md` (the A6 mixed-case
remedy sentence).

Also changed: BOOT's A7 mixed-case notice no longer says "the present fallback until
FW 53"; it says the FAULT exit re-engages a dropped Leg and stows all six only when its
reference is proven kept (one test assertion updated to the new wording).

## Verification

- 2026-11-26, `flock /tmp/jugglebot-tests.lock python -m pytest -q -p no:cacheprovider -n 4
  --dist loadfile tests/ros/test_state_machine.py tests/ros/test_state_machine_deactivate_confirm.py
  tests/ros/test_orchestrator_node.py tests/ros/test_orchestrator_conduit_integration.py
  tests/ros/test_teensy_bridge_node_*.py tests/teensy_link/test_deactivate.py
  tests/ros/test_leg_reference.py tests/motion/test_tilt_cal_grid.py` (the brief's set plus the
  new file and the one other importer of `state_machine`): **1261 passed in 65.13 s**.
- 2026-11-26, `python tests/firmware/native/build.py && ./temp/firmware_native/test_leg_deactivate`:
  **28/28 test cases, 751 assertions pass**; the new case alone (`-tc='R3*'`) 61 assertions.
- 2026-11-26, 18 Python mutations, each by `pytest -x` on its test (script
  in the unit scratchpad, not committed): **18/18 caught** (the mixed branch, the seal, once per visit,
  the verdict ignoring refusals, the orchestrator feed, the lapse/seal logic, SPINOUT added to
  KEEPS, nine predicate rows, the reboot clear unconditional, the ERR_TIMEOUT exception).
- 2026-11-26, 3 native mutations built beside the tree (a scratchpad script, not committed; the
  firmware sources are not edited): SETUP seed fixed to the pose, the SETUP error gate removed,
  the DROPPED branch removed from `stow_descent_step`: **3/3 fail the R3 case**; the unmutated
  control built the same way passes.
- Not run: `./run_tests.sh` (the orchestrator gates at integration). Nothing ran on hardware.

## Before a Sitting trusts this recovery

It re-energises a Leg under a held Platform. Fly it first with a person supporting the
Platform and a hand on the E-stop button, from a low pose:
1. Force the case: with the Platform at a low Active pose, disarm one Leg with an error that
   keeps the reference (e.g. a CAN watchdog expiry on that axis, or an SDO current limit
   breach on a bench-safe setting) and confirm `/robot_state` shows its `disarm_reason` held
   until the `clear_errors`.
2. Watch the re-engage: the dropped Leg's dip under load on CLOSED_LOOP entry (empty
   integrator), and that its first post-CL heartbeat reads Closed-loop.
3. Measure the Leg ODrive heartbeat period: it bounds how far the five descend before a Leg
   that re-faults on engage is seen (the walk's 10 mm estimate assumes 100 ms).
4. Check `REF_DRIFT_TOL_REV` (0.25 rev) against the sag the free Leg actually shows.
