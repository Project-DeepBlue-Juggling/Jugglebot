---
title: "The orchestrator's real-fault exit from ACTIVE now stows the legs (profiled DEACTIVATE) and holds FAULT until the stow completes, so IDLE is never claimed with the legs holding an active pose"
type: investigation
date: 2026-10-06
status: in-progress
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-10-06-skill-stack-r5-sitting-6-catch-high-seat-verdict.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
  - tests/ros/test_state_machine.py
  - tests/ros/test_orchestrator_node.py
---

# The real-fault exit from ACTIVE stows the legs

## Summary

What happened at 2026-10-06 20:25 (launch `2026-10-06-20-19-40`):

- At 718.60 the bridge guard latched MOTOR_FB_STALE on axis 6, because the owner had rebooted the hand ODrive. The orchestrator went ACTIVE → FAULT (forced, guard-only). The forced path's `_cancel_pending_operations` dropped `ActiveHandler.on_exit`'s deactivate, by design.
- At 720.41 the hand's `ODrive error on hand` promoted the fault to real.
- At 721.10 everything had cleared and the machine went FAULT → BOOT → IDLE without a deactivate. The legs were still in CLOSED_LOOP, holding an ACTIVE pose 62 mm off centre. That broke IDLE's invariant that the platform is stowed, and the next `activate` started from there.

The fix: `FaultHandler` now records whether the visit began in ACTIVE. The flag survives a guard-only → real promotion. Once every error and the guard latch have cleared, it requests the profiled `deactivate` if `ctx.legs_closed_loop` (new) and `ctx.is_homed` are both true. FAULT holds until the deactivate succeeds.

- **Stow fails:** FAULT holds. Each operator `clear_errors` allows one retry, so the exit cannot spin.
- **Operator commands during the stow:** they stay queued, so a `clear_errors` cannot take the single tracked slot and have its result read as the stow's.
- **Legs not all closed-loop, or the fault did not start in ACTIVE:** BOOT at once, as before.

The orchestrator derives `legs_closed_loop` from `/robot_state.motor_states`: all six legs must be in `odrive.AXIS_STATES['CLOSED_LOOP']` with `active_errors == 0`. It also logs one `FAULT exit: stowing the legs` line. The contract changed first (`ARMING_CONTRACT.md` choreography step 6 and the A2 row).

## Discussion

This follows the owner's decision of 2026-10-06. The rejected alternative was to stow on the guard-only path too ("any FAULT deactivates"). The owner kept the guard-only resume: that path never disarmed, and it goes back to ACTIVE without re-arming, which is cheaper and is the reason the path exists. Only the real-fault exit, which already ends at BOOT/IDLE, has to make IDLE's invariant true.

## Verification

- 2026-10-06, `python -m pytest tests/ros/test_state_machine.py tests/ros/test_orchestrator_node.py tests/ros/test_orchestrator_conduit_contract.py -q -p no:cacheprovider`: **282 passed in 4.04 s**. This includes 14 new tests: `TestFaultExitStow` (9), `TestLegsClosedLoop` (4) and `TestFaultExitStowThroughTick` (1).
- Full gate (`./run_tests.sh --full`, run 2026-10-07, log `temp/logs/gate_full_r5_plane830_curr15_stow_20261007.log`, on the tree holding this unit, the orchestrator stow unit and the merged box): **parallel 6125 passed, 9 skipped, 1 xfailed in 305.26 s; serial 6 passed in 20.51 s; RESULT PASS, exit 0.** The only edits after that run are these Verification lines; the logbook tests were re-run after them.

## Open Questions

- A real fault during LEVELLING leaves the legs at the level pose in the same way: `LevellingHandler.on_exit` clears requests, and `level_deactivate` never runs. It is outside this change's scope, which covers faults from ACTIVE only.
