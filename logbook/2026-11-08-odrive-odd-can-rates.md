---
title: "All seven Jugglebot ODrives on odd-ms CAN rates (encoder and iq 9, error 19, heartbeat and temperature 99, bus voltage 499); written over USB and verified from flash; the fleet tool's --set gains --include-hand"
type: feature
date: 2026-11-08
status: resolved
phase: "can-bridge clock beat"
related_issues:
  - BUG-82
  - BUG-50
files_changed:
  - config/ODrive config Files/odrive_pro_leg_config.json
  - config/ODrive config Files/odrive_pro_hand_config.json
  - tools/odrive_fleet_reflash.py
  - tests/motion/test_odrive_fleet_reflash.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hand_fan.h
subsystem:
  - can-bridge
  - tools
tags:
  - odrive
  - can
---

## What and why

The encoder dropouts are a clock beat (`logbook/2026-11-06-arrival-pull-and-hand-dropout-probes.md`). Any cyclic ODrive message whose period is an even number of ms is a whole multiple of the can-bridge's 2 ms setpoint tick. Its frame phase drifts into a lossy window 6 to 30 µs wide and stays there, so every frame is lost for 0.6 to 3 s. An odd period alternates between two phases 1 ms apart, so at most every other frame can be lost.

Evidence that 9 ms works, from the 2026-11-07 Sitting:
- Leg 4 (set by the owner before that Sitting): 0 of 2396 `/cache_diag` windows over 25 ms (peak 17.1 ms), while the other five Legs had 15 to 25 windows over 50 ms in the same launch.
- The Hand: clean at 9 ms over two Sittings (`logbook/2026-11-07-sitting-debrief.md`).

The owner chose to move every cyclic message to an odd period in one save per drive (2026-11-08). Each new value is 1 ms faster than before, so no staleness threshold ever sees an older frame than it did:

| Message | Was | Now | What reads it |
|---|---|---|---|
| encoder | 10 | 9 | Leg and Hand feedback, MOTOR_FB_STALE, the lead clamps |
| iq | 10 | 9 | telemetry, the Hand stall guard on the cold-start ladder |
| error | 20 | 19 | `active_errors`, which drives the bridge's Fault decision |
| heartbeat | 100 | 99 | axis state; the stale flag at 2.0 s (Leg 2 had a 2.1 s gap on 2026-11-07) |
| temperature | 100 | 99 | telemetry, the Platform Teensy's Hand fan (timeout 5 s) |
| bus voltage | 500 | 499 | telemetry |

The bridge counts only encoder and heartbeat frames, so the error, iq, temperature and bus-voltage streams are expected to behave the same way but cannot be measured. Added bus load is about 2 %.

## Changes

- Both backup JSONs carry the new rates. A new test pins the rule: every non-zero `*_msg_rate_ms` in both files must be odd.
- `tools/odrive_fleet_reflash.py --set` gains `--include-hand`: it writes the same properties to the Hand drive, last, after backing it up and checking it is Idle along with the Legs. It refuses if the Hand is not on USB, and the summary reminds that the Hand's zero is gone too. The erase path still never touches the Hand. Six new tests cover it.
- Two firmware comments that quoted the old Hand rates were corrected (`canbridge_config.h`, `Teensy_code_platform/hand_fan.h`). Both changes are comment-only, so no flash is needed.

## Verification

- Drives: `python tools/odrive_fleet_reflash.py --include-hand --set ...` (the six rates), run 2026-11-08 13:01. All seven drives (Legs 0 to 5, then the Hand) were backed up, written, saved, rebooted and read back from flash: verify OK on all seven. Leg 4's and the Hand's encoder rates were already 9. Log: `temp/logs/odrive_can_rates_2026-11-08.log`. Backups: `temp/odrive_configs/20261108_130126/`.
- Tests: `pytest -q tests/motion/test_odrive_fleet_reflash.py`, run 2026-11-08: 77 passed. Default gate: see the commit message.

## Before the next launch

Every drive rebooted, so every encoder zero is gone. Check the launch log for `homing: 7 axes homed` before arming. That was enough after the Hand's save on 2026-11-06, without a bridge power-cycle.

At the next debrief, read the `/cache_diag` census and expect every Leg near the Hand's 9 ms numbers. Do the same for heartbeat dropouts, which should fall from 77 episodes per 40 min.
