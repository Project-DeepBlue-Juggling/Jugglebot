---
id: FEAT-49
kind: feature
title: Arm a power capture with the fault trigger on the robot Hand to catch a BUG-57 fault
status: in_progress
created: '2026-11-11'
updated: '2026-11-20T04:33:53Z'
owners: []
priority: medium
effort: m
hardware: robot
owner: hands
radius: local
area: hand
blocked_by: []
links:
- map/bug-hand-overshoot-at-apex-1-35-saturated-the-stepped-brake-at.md
- map/feature-measure-the-hand-s-commutation-angle-error-through-a-hard.md
- tools/probes/hand_foc_scope.py
- logbook/2026-11-18-sitting-debrief.md
- logbook/2026-11-20-sitting-debrief.md
- logbook/2026-11-20-hand-foc-scope-keeps-the-forced-window.md
filed_by: claude-debrief
night_skip: sitting
night_skip_on: '2026-11-15'
night_skip_note: After FEAT-45 rungs B and C pass on the bench Hand.
---

Arm `tools/probes/hand_foc_scope.py capture --preset power --trigger fault` on the robot's Hand for a whole Sitting, to catch a BUG-57 SPINOUT at its disarm. It is the one measurement that separates a transient commutation angle error from the other readings (BUG-57, 2026-11-11 section). Blocked on FEAT-45: its bench rungs B and C must pass with the fixed scope tool first.

## Wanted

- The capture records the drive's power channels through the fault: iq and deceleration per amp up to the disarm, and the filtered mechanical power at the trip.
- An angle error shows as a large q-axis flux error and torque per amp collapsing before the disarm while current still flows.

## Risks

- The robot Hand's USB must go through an isolator, and the loaded Jetson must hold about 2100 USB reads per second for the whole Sitting. Check both first.
- Unverified: the fault trigger has never run on a real disarm. On 2026-11-11 a timed-out window was saved as if it were a capture.
- The fixed tool fires only on an edge of `disarm_reason` or `active_errors`, and re-arms without saving on a clean exit from CLOSED_LOOP (it saves one only with `--save-clean`). That has only been tested offline: confirm on the robot that a clean IDLE at Park re-arms and saves nothing, and that a repeat fault with the same code is caught by the `active_errors` edge.

## Sitting

- setup: armed
- why: BUG-57 faults about once in 50 to 400 launches, and only a power capture armed across launches can show whether the torque loss is an angle error.
- do:
  - Do not fly this until FEAT-45 rungs B and C have passed with the fixed scope tool. Then run `live_session` and check CPU load, and confirm the robot Hand's USB runs through the isolator.
  - From the repo root in the venv, with the launch up and the Hand homed: `python tools/probes/hand_foc_scope.py capture --preset power --trigger fault --label robot_fault`. Leave it armed for the whole Sitting and fly the Apex 0.95 to 1.35 rows that produced the faults.
  - Stop at the first SPINOUT after the capture saves. Ctrl-C on the capture is safe.
- pass: a window is saved on a real fault, with the brake inside it and the power channels finite.
- record: the capture path, the launch log line of the SPINOUT, the number of launches flown, and any clean-stop saves or timeouts.
- minutes: 60

## Sitting result (2026-11-18 13:05)

- verdict: inconclusive
- notes (split into bullets):
  - Gave this a bunch of attempts. Until the last one, all attempts either completed or ended due to a missed catch. The final attempt ended with MAX_DEVIATION. (sheet 2026-11-18-robot, step 48) [derived: temp/logs/launch_2026-11-18_4.log and temp/hand_capture/hand_capture_20261118_121427.csv: armed 12:15:41 for 10 Columns Attempts, 0 saved, 1 clean exit.
  - The Hand drive stayed in closed loop with no errors 12:15:59-12:25:22, so no real fault occurred to trigger on (the one MAX_DEVIATION latch is a Teensy guard fault, not a drive fault).]

## Sitting result (2026-11-20 15:11)

- verdict: fail
- notes: A SPINOUT shows up in the `grep` readout (sheet 2026-11-20-robot, step 16) [derived: The trigger fired on the real SPINOUT (temp/logs/launch_2026-11-20_1.log:5298, 13:22:42) but hand_foc_scope.py raised 'scope still recording 3.0 s after the trigger' in scope_download and saved nothing; launch 2 had no drive fault to trigger on. The newest file in temp/hand_foc_scope/ is from 2026-11-13.]

## Progress

- 2026-11-18: Flown at the Sitting of 2026-11-18, verdict unresolved: armed for 10 Columns Attempts, 0 saved; the Hand drive never faulted (the one MAX_DEVIATION latch is a Teensy guard fault and cannot fire the trigger). Arm it in every launch: BUG-111 was a SPINOUT in launch 1, before it was armed. FEAT-45 rungs B and C passed on the bench 2026-11-13.
- 2026-11-20: Failed at the Sitting of 2026-11-20: the trigger fired on a real SPINOUT (13:22:42) but scope_download raised 'scope still recording 3.0 s after the trigger' and threw the stopped window away; nothing saved. Likely cause: the drive's scope does not reach its stop condition once the axis has disarmed (never exercised on the bench).
- 2026-11-20: A guard latch (MAX_DEVIATION) never disarms the drive, so it cannot trigger a capture.
- 2026-11-20: Tool fixed on main (658fd7c0): a scope still recording 3 s after the trigger is force-stopped and its window saved as <label>_<n>_fault_forced with forced_stop in the metadata; one failed capture re-arms, three in a row exit 4. Unknown until a real fault: whether the forced window still holds the fault or only the last 256 ms before the stop. The next Sitting keeps this row.
