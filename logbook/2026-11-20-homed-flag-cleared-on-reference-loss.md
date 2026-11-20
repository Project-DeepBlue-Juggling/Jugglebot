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

## Verification

- 2026-11-20, `python -m pytest -q -p no:cacheprovider tests/ros/test_teensy_bridge_node_reference_loss.py`: **11 passed in 4.29 s**.
- 2026-11-20, `flock /tmp/jugglebot-tests.lock python -m pytest -q -p no:cacheprovider -n 4 --dist loadfile tests/ros/test_teensy_bridge_node_*.py tests/ros/test_state_machine.py tests/ros/test_state_machine_deactivate_confirm.py tests/ros/test_orchestrator_node.py tests/ros/test_orchestrator_conduit_contract.py tests/ros/test_orchestrator_conduit_integration.py tests/ros/test_hand_capture.py` (every bridge-node test file, the state machine and the orchestrator): **960 passed in 62.87 s**.
- 2026-11-20, `python -m pytest -q -p no:cacheprovider tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_no_legacy_item_ids.py tests/sim/test_docs_links.py`: **48 passed in 2.81 s**.
- Full gate: (filled in by the main session)
