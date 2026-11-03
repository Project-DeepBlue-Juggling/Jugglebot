---
id: JB-146
kind: bug
title: ODrive GUI save resets the Hand zero, the bridge stays homed, and ACTIVATE parks the Hand into its stop at full current
status: complete
created: '2026-11-02'
updated: '2026-11-03T02:48:50Z'
owners:
- jugglebot-3d
priority: high
effort: s
hardware: robot
owner: decision
radius: normative
diagnosis: confirmed
area: can-bridge
blocked_by: []
links:
- logbook/2026-11-03-sitting-debrief.md
filed_by: jugglebot-3d
---

Saving settings in the ODrive GUI reboots the Hand ODrive, which erases the homed absolute position (`set_absolute_position(-0.1)` at the hard stop lives in ODrive RAM). The can-bridge keeps `is_homed=1`, so nothing re-homes, and the next ACTIVATE parks the Hand at "0.0 rev" in the wrong frame.

## Evidence (2026-11-02 16:52 launch, bag `2026-11-02_16-52-08_Hand-motor-overheat`)

Before levelling the Hand sat idle at 0.49 rev (normally held at 0.30). At LEVELLING it went closed-loop to 0.0, stalled at 0.22 rev against the stop, and the position loop wound up linearly to the 70 A soft limit in 0.55 s. It held about -70 A for 5.5 s until MOTOR_OVER_TEMP. The levelling DEACTIVATE was rejected meanwhile. The owner confirmed the zero reset and re-homed.

## Options

1. The bridge clears the Hand's homed flag when it sees the Hand ODrive reboot (heartbeat state, uptime or a position jump), forcing a re-home.
2. A stall guard in the ACTIVATE / HAND_MOVE_TO ladder: |iq| near the limit with |v| about 0 for more than ~300 ms aborts to IDLE with a named refusal.
3. A procedure note: after any ODrive GUI save, power-cycle the bridge or re-home before ACTIVATE.

Option 2 also covers any other stall. 1 and 2 are firmware (flash).

## Sitting

- setup: level, armed
- why: confirm the FW 36 stall guard (8ffeba66, flashed 2026-11-02) stays silent through every normal activation, and that the GUI's direct IDLE -> TRAJECTORY now works (2a19d9c6).
- do:
  - Level from IDLE.
  - In the GUI, go straight from IDLE to TRAJECTORY.
  - Return to IDLE, then go IDLE -> TRAJECTORY directly once more.
- pass: no STALL line in the launch log, the Hand parks normally each time, and both direct IDLE -> TRAJECTORY requests arm without a time-out.
- record: the launch log's activation lines (look for STALL and for "refused: setpoint output is DISARMED"), and the Hand's peak current during each park from the capture CSV.
- minutes: 5

## Sitting result (2026-11-03)

- verdict: pass
- notes: IDLE > TRAJECTORY passed without issue three times.

## Progress

- 2026-11-02: FW 36 stall guard landed (8ffeba66): 25 A / 0.5 rev/s / 300 ms aborts the Hand to IDLE with a named STALL. Built, not flashed; closes after the flash receipt and a Sitting.
- 2026-11-03: Passed at the Sitting of 2026-11-03: 3 direct IDLE->TRAJECTORY arms in 1.6 s (launch.log L273/335/422), 0 STALL lines, diag bit2 never set in 31,389 rows, park peaks 2.3-2.8 A. A GUI save still resets the Hand zero: re-home after every ODrive GUI save (the owner saved again on 2026-11-03 after the Sitting).
