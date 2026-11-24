---
title: "BUG-122: the bridge clears is_homed when a Leg or Hand drive loses its position reference (MISSING_ESTIMATE, or a drive reboot seen as INITIALIZING), so a Fault exit goes BOOT -> Homing instead of BOOT -> Idle; the real-fault exit stow no longer needs is_homed"
type: bugfix
date: 2026-11-20
status: in-progress
phase: "BUG-122 (Sitting 2026-11-20 Hand overheat)"
subsystem: [can-bridge, orchestrator, ros]
files_changed:
  - ros_ws/src/jugglebot/jugglebot/reference_loss.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/can/odrive.py
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - tests/ros/test_teensy_bridge_node_reference_loss.py
  - tests/ros/test_state_machine.py
tags:
  - safety
---

## What and why

Sitting 2026-11-20: the Hand drive raised MISSING_ESTIMATE at rest (13:32:16) and its position came back exactly 5.00 turns low. `is_homed` lives in Platform Teensy RAM and was cleared only by REBOOT_ODRIVES or that board losing power, so `clear_errors` went Fault -> BOOT -> Idle with no Homing (`state_machine.py` BootHandler), and the next Feed Catch drove the Hand into its top stop at 70 A for 4.4 s (motor thermistor 88 C). After the E-stop button power-cycled all seven drives (INITIALIZING on every axis), `is_homed` was still true and BOOT went to Idle again.

Fix, host side only (no firmware, threshold or guard change):

- `jugglebot/reference_loss.py` (pure): a per-axis rising edge of MISSING_ESTIMATE or INITIALIZING, in either the active or the disarm mask, on axes 0..6 (Legs and Hand; not the Ball Butler) is one reference-loss event. It remembers that a clear is owed until a write of `is_homed=False` lands, or until a Homing that started after the event writes its result.
- `teensy_bridge_node.py`: every Diagnostic feeds the watch. On an event while homed (or while Homing, or before the boot read), the local view drops to `is_homed=False` at once and one ERROR line names the axis: `homed flag cleared: hand reported MISSING_ESTIMATE (disarm), so its position reference may be lost; re-home required`. A worker thread lands the clear through `_write_is_homed(False)` (the existing relay STATE_WRITE, read-modify-write, so levelling and pose are kept). The 1 Hz `_health_check` retries it until it lands. A boot read that still says homed while a clear is owed is published as not homed.
- `state_machine.py` `_real_fault_exit` and ARMING_CONTRACT step 6: the exit stow no longer requires `is_homed`; see the Discussion.

After `clear_errors` the Fault exit goes BOOT -> Homing automatically (BootHandler `if ctx.is_homed: return IDLE` / `return HOMING`). The operator does not have to ask for it.

## Discussion

Hazards checked before coding:

- **Reboot signal.** INITIALIZING in the drive's active errors. The ODrive sets it at every boot until init completes, and holds it while the DC bus is down. It appears in every logged drive reboot (2026-08-12 powered reboot, 2026-10-06 Hand reboot, 2026-11-20 E-stop button) and never during Homing, encoder search or configure in the 2026-11-18/20 launch logs. Rejected alternatives: `axis_state` UNDEFINED is brief and can fall between Diagnostic frames; the drive's uptime is not relayed; an under-voltage disarm can be a sag without a reboot, and a powered reboot (BUG-56's GUI save) has no under-voltage.
- **Start-up.** A fresh power-on reboots the Platform Teensy too, so the read says not homed and the event is a no-op. Before the boot read lands nothing is written, because that would clobber levelling and pose with defaults; the read then decides. Homing does not reboot drives. An event during a Homing stays owed, so a second Homing is needed.
- **Debounce.** Edge-triggered per axis, because `disarm_reason` is sticky and INITIALIZING persists while the bus is down. One worker at a time. A failed write keeps the latch and the local view stays false. A Homing that starts after the event answers it, so a late retry cannot undo the new home.
- **Legs.** The same class as the Hand: a Leg MISSING_ESTIMATE or reboot clears too.
- **Ordinary faults keep the reference.** SPINOUT_DETECTED (position stayed continuous on 2026-11-20), under-voltage alone, and the can-bridge guard faults such as MAX_DEVIATION (a HeartbeatT2J fault state, not a Diagnostic bit) do not clear it.
- **Exit stow (state machine change).** The 2026-10-06 real-fault exit stow was gated on `legs_closed_loop and is_homed`. A Hand-only loss during Active (the 2026-10-06 Hand-reboot case itself) now clears `is_homed` while the Legs hold an Active pose. Under the old gate that would skip the stow and start Homing with the Legs at pose. The gate is now `legs_closed_loop` alone. A Leg cannot be in CLOSED_LOOP with no active error after MISSING_ESTIMATE or a reboot, so that already proves each Leg's reference. DEACTIVATE only IDLEs the Hand (`leg_deactivate.cpp` `idle_hand_now`).
- **Activate while un-homed.** Idle is reached only from a successful Homing or from BOOT with `is_homed`. Both trigger bits are drive errors, which put `robot_state.error[]` into Fault from any state (`orchestrator_node._tick`). IdleHandler has no `is_homed` check of its own.
- **Skipped on purpose.** The "Hand position jumped or went NaN while Idle" detector needs a threshold, so it is the owner's decision. Also not done: a drive reboot that happens while the bridge node is not running is missed if INITIALIZING has cleared before the node starts. The encoder-search bit is left set, matching the operator's working manual re-home at 13:37:50.

### Port to main and adversarial review (2026-11-24)

The fix was held on `night-shift/2026-11-20-BUG-122` (7748f72e) while main moved to can-bridge FW 49, Platform FW 15 and the Platform IMU gap check. It was cherry-picked onto main as `offline/bug122`. The only code conflict was the import block of `teensy_bridge_node.py`: main's multi-line `teensy_link.platform_imu` import (with `format_upload_end`, `read_gap_check`, `BRIDGE_ARMED_MIN_GAP_10US`) is kept whole and the branch's `ReferenceLossWatch` import sits above it. Every other hunk applied cleanly and was re-read in today's context: the `__init__` fields come before the Diagnostic subscription, so no frame can reach a missing watch.

Spurious-clear walk (could this send the robot to Homing when nothing was lost?):

- **Where the bits come from.** `active_errors` and `disarm_reason` in a Diagnostic are copied verbatim from the drive's `get_error` frame (`can_buses.cpp` `case ODriveCmd::get_error`). Nothing in the can-bridge writes those fields itself. The FW 47 to 49 Hand stall guard commands the Hand IDLE and its latch is cleared by the next ACTIVATE (`leg_activate.cpp`); the BUG-105 vel_ff zeroing writes only an input. Neither raises a drive error bit, so neither can fire the watch.
- **Zero frames.** A can-bridge reboot zeroes its axis table (`axis_state.h` defaults, `telemetry_init`), and a frame before the first `get_error` carries 0/0. A zero frame can only be a falling edge. A falling-then-rising pair (`set, 0, set`) needs the bit really set on both sides, and any nonzero mask on a fresh Leg or Hand Diagnostic is already in `robot_state.error[]`, which forces Fault from every state (`orchestrator_node._tick`). In Idle or Active every watched axis reads 0/0, so a bridge reboot or reconnect there produces no event.
- **First frame.** A set bit on the first frame seen for an axis counts as an edge (deliberate: a loss before this process started is still a loss). A normal relaunch of the ROS stack against healthy drives sees 0/0 and keeps `is_homed`; a relaunch while a drive still carries an uncleared MISSING_ESTIMATE clears it, which is correct.
- **Stale frames.** A Diagnostic with the stale flag repeats the frozen masks, so it cannot make an edge. **NaN** cannot occur: the masks are integers on the wire. Position NaN is not watched (the skipped detector above).
- **Per axis.** `_prev` is keyed by axis; axes 7 and 8 (Ball Butler) are ignored.

Armed-and-moving walk (a loss during an Attempt): the trigger bits are drive errors, so the robot goes Active -> Fault exactly as before this fix; clearing `is_homed` adds nothing to that edge. Mid-Active the only reader of `is_homed` outside the state machine is `trajectory_node`'s seed gate, which only defers a seed that has not happened yet (`trajectory_node.py` `_on_robot_state`), and the bridge has no other gate on it. The local-view drop runs on the receive thread under `_lock` and never blocks; the relay write runs on a worker. So the Attempt in flight ends by the Fault, never by the clear.

Stow gate (does the new gate allow a stow the old one refused?): yes, in exactly the cases where `is_homed` is false while the Legs are posed in CLOSED_LOOP with no active error: (1) a Hand-only loss, the new case; (2) a CAN3-recovery conservative read that failed or read False while the Legs were posed. The old gate sent both straight to BOOT -> Homing with the Legs at an Active pose. A profiled DEACTIVATE from a verified closed-loop pose is the safer of the two, so the change is accepted. No case that used to stow is now refused.

**One defect found and fixed (small).** `_refresh_cold_start_state` (the boot read and both reconnect re-reads) checked only `pending` when applying a reply. A read whose reply left the Platform Teensy before a loss, but was applied after that loss's clear had already landed (pending back to None), would write `is_homed=True` back into the local view; BOOT would then skip Homing, the original hazard. The relay serialises the read and the write, so only a late apply (a descheduled thread) opens it; the CAN3-recovery re-read is exactly the one an E-stop button press can trigger beside the drives' INITIALIZING edges. Fix: snapshot `ReferenceLossWatch.seq` before the read, and a homed reply applied after any newer event is published as not homed. The safe direction only: at worst it forces one extra Homing. Test: `test_read_in_flight_across_a_landed_clear_does_not_restore_homed`.

What the owner sees change: every E-stop button press, drive reboot (including an ODrive GUI save, BUG-56's case) or MISSING_ESTIMATE on any Leg or the Hand now makes the next `clear_errors` go Fault -> BOOT -> **Homing**, which moves the Legs and the Hand to their stops without a further command.

## Verification

- 2026-11-20, on the held branch (7748f72e, before the port to main), `python -m pytest -q -p no:cacheprovider tests/ros/test_teensy_bridge_node_reference_loss.py`: **11 passed in 4.29 s**.
- 2026-11-20, on the held branch (7748f72e, before the port to main), `flock /tmp/jugglebot-tests.lock python -m pytest -q -p no:cacheprovider -n 4 --dist loadfile tests/ros/test_teensy_bridge_node_*.py tests/ros/test_state_machine.py tests/ros/test_state_machine_deactivate_confirm.py tests/ros/test_orchestrator_node.py tests/ros/test_orchestrator_conduit_contract.py tests/ros/test_orchestrator_conduit_integration.py tests/ros/test_hand_capture.py` (every bridge-node test file, the state machine and the orchestrator): **960 passed in 62.87 s**.
- 2026-11-20, on the held branch (7748f72e, before the port to main), `python -m pytest -q -p no:cacheprovider tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py`: **48 passed in 2.81 s**.
- 2026-11-24, on `offline/bug122` (main bc101eaf + this fix), `python -m pytest -q -p no:cacheprovider tests/ros/test_teensy_bridge_node_reference_loss.py`: **12 passed in 6.43 s**; with the new seq check in `_refresh_cold_start_state` replaced by `elif False:`, `test_read_in_flight_across_a_landed_clear_does_not_restore_homed` fails (1 failed, 11 passed), restored after.
- 2026-11-24/25, `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider -n 4 --dist loadfile tests/ros/ tests/firmware/test_odrive_protocol_xref.py tests/firmware/test_hand_stall_guard_xref.py tests/motion/test_tilt_cal_grid.py` (all of `tests/ros/`, a superset of the 2026-11-20 set, plus the other readers of `can/odrive.py` and the bridge node), box load average about 30: **4361 passed, 2 failed, 2 skipped in 3946 s**. The two failures read none of the changed files and are load timing: `test_skill_node_robot_frame.py::test_moved_frame_announcement_landing_is_the_robot_frame_prior` (a mocap sample 78 s stale against a 1.0 s window) and `test_trajectory_tilt_map.py::test_the_acquisition_tool_detects_this_nodes_real_disarmed_marker`. Rerun alone, `python -m pytest -q -p no:cacheprovider tests/ros/test_trajectory_tilt_map.py tests/ros/test_skill_node_robot_frame.py`: **57 passed in 10.94 s**.
- 2026-11-24, `python -m pytest -q -p no:cacheprovider tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py`: 46 passed, 2 failed, both in `test_logbook_index.py` because `logbook/INDEX.md` is main's copy and lacks this entry's row (regenerated at integration).
- Full gate: (filled in by the main session)
