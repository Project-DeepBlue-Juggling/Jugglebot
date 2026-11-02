---
title: "GUI IDLE -> ACTIVE:TRAJECTORY no longer times out: a successful arm reply seeds the orchestrator's wire-armed mirror, so a mode queued during activation is not refused against the pre-arm /link_status"
type: bugfix
date: 2026-11-02
status: complete
phase: "Sitting 2026-11-02 follow-up"
subsystem: orchestrator
related_entries:
  - 2026-11-02-hand-release-step.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - tests/ros/test_state_machine_deactivate_confirm.py
  - logbook/INDEX.md
---

# IDLE -> TRAJECTORY arm race

**What happened.** At the 2026-11-02 17:00 Sitting, the GUI's direct IDLE ->
ACTIVE:TRAJECTORY "consistently timed out"; going via STANDBY worked.

The activation itself finished in about 1.6 s. The orchestrator then refused
the queued `trajectory` command as "setpoint output is DISARMED (wire not
armed)" (launch.log l.380, 285.022). The GUI waited its 15 s `T_MODE` for a
TRAJECTORY that never came, and reported a time-out.

**Why.**
- The GUI plan publishes `trajectory` as soon as `control_mode` reads STANDBY,
  which is before the arm.
- `ActiveHandler` holds that command until the `arm_setpoints` reply. It then
  consumes it on the same tick against the owner-policy check from 1be27071
  (2026-10-23): auto-arm, plus a `/link_status` already seen, plus
  `ctx.wire_armed` false, means refuse.
- `wire_armed` came only from the bridge's 10 Hz `/link_status`. On that tick it
  still held the pre-arm value; the first `mpc_active=1` arrived 19 ms after the
  refusal.
- Whether a publish lands between the arm and the next tick depends only on the
  two free-running 10 Hz timer phases, which are fixed for a launch. So a launch
  either mostly works or mostly fails.
- A survey of launches since 10-16 found two other refusals besides this one:
  2026-10-28 09:49 once, and 2026-11-02 twice.

**Fix.** A successful `arm_setpoints` reply now also sets
`ctx.wire_armed = True`.
- The reply is the wire's state: the bridge returns success only from
  `_arm_setpoint_output_locked`, either "already enabled" or after
  `_start_setpoint_output` -> `_set_mpc_active(True)`.
- The seed is not a latch: any later disarm reaches `wire_armed` through the
  next `/link_status`, so the disarmed-wire refusal still holds.
- A failed arm does not set it, and manual-arm sessions skip the arm phase.
- Nothing commanded changes: not the activation move, the arm pre-checks or the
  40 Hz stream.

Diagnosis and fix by an Opus agent; reviewed in the main session (the bridge's
success path was read to confirm the seed is truthful).

**Verification.**
- New tests in `tests/ros/test_state_machine_deactivate_confirm.py`:
  - `test_mode_queued_during_activation_lands_on_the_arm_tick` replays the
    Sitting;
  - `test_disarm_after_the_arm_still_refuses_mode_switch`;
  - `test_failed_arm_attempt_does_not_seed_wire_armed`.
  Two of these fail on HEAD's `state_machine.py` and all pass with the fix.
- Readers (2026-11-02, the agent's run):
  `pytest -q -n 4 tests/ros/` gave 3533 passed, 2 skipped.
- Gate (2026-11-02, `./run_tests.sh`): **7990 passed, 55 skipped (parallel
  406 s) + 5 passed (serial), RESULT PASS**.

**Not done.**
- The `_on_link_status` docstring in `orchestrator_node.py` (~l.500) still says
  handlers never gate on it, which has been stale since 10-23.
- The GUI does not surface the orchestrator's refusal notice; the operator saw a
  bare 15 s "timed out".

Hardware needs a `colcon build` and a launch restart to pick this up.
