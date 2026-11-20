---
id: BUG-122
kind: bug
title: Hand stays homed after its drive loses the position reference, so the next move rams the top stop
status: in_progress
created: '2026-11-20'
updated: '2026-11-20T04:33:49Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: subsystem
diagnosis: confirmed
area: hand
blocked_by: []
links:
- logbook/2026-11-20-sitting-debrief.md
- bug-odrive-gui-save-resets-the-hand-zero-the-bridge-stays-homed
filed_by: claude
night_skip: owner-session
night_skip_on: '2026-11-21'
night_skip_note: A day Session built and held the fix (state_machine and ARMING_CONTRACT edits, unflown recovery path); the night does not re-gate it.
---

Sitting of 2026-11-20, launch 1 (bag `2026-11-20_12-49-25`). At 13:32:16.53 the Hand drive raised MISSING_ESTIMATE while resting at 5.40 rev; its position read NaN and came back as 0.40 rev, exactly 5.00 turns lost. `is_homed` stayed 1, so `clear_errors` went FAULT, BOOT, IDLE with no Homing (`state_machine.py:347-349`), and the next Feed Catch (polygon_walk, 13:34:22) drove the Hand into its top stop:

68-70 A for 4.4 s, thermistor peak 88.3 C, ended only by the E-stop button. After that full drive power cycle (INITIALIZING on all 7 axes) `is_homed` was still 1 and BOOT went to IDLE again at 13:37:48.

`is_homed` is Platform Teensy RAM, cleared only by REBOOT_ODRIVES (`teensy_bridge_node.py`, beside `_write_is_homed`) or by that board losing power. No drive fault clears it. Same class as BUG-56, whose fix covered only the ACTIVATE ladder.

Fix: clear `is_homed` through the existing relay write when any axis reports MISSING_ESTIMATE or a drive reboot (INITIALIZING after under-voltage), or when the Hand's position goes NaN or jumps while Idle; the FAULT exit then routes to Homing. Defence in depth: the can-bridge refuses ACTIVATE of the Hand after a discontinuity since the last HOME (`leg_activate.cpp:112-117`).

## Sitting

- setup: level
- why: the robot must re-home after a drive loses its position reference, and only a real drive power cycle shows the flag being cleared.
- do:
  - Check out the branch and rebuild both packages (`colcon build --packages-select jugglebot_interfaces jugglebot`).
  - Launch and home; do NOT arm. Leave the Cup empty.
  - Press the E-stop button, wait five seconds, release it.
  - Clear errors from the GUI and watch the state.
- pass: after the clear the robot enters Homing (not Idle) and `/robot_state` shows `is_homed` false until that Homing finishes.
- record: the states the GUI showed after the clear, and the launch log line that clears the homed flag.
- minutes: 5
- branch: night-shift/2026-11-20-BUG-122

## Progress

- 2026-11-20: Fix held on night-shift/2026-11-20-BUG-122 (7748f72e): the bridge clears the homed flag on a rising MISSING_ESTIMATE or INITIALIZING on any Leg or the Hand, retried until the write lands, so clear_errors goes Fault, BOOT, Homing.
- 2026-11-20: It also changes the fault-exit stow condition in state_machine.py and step 6 of ARMING_CONTRACT.md (a Hand-only loss with the Legs posed still stows first), which is why it is held, not merged: a normative contract edit and an unflown recovery path. Scoped tests 960 passed; no full gate or audit yet (they run at land). Every E-stop button press will force a re-home.
