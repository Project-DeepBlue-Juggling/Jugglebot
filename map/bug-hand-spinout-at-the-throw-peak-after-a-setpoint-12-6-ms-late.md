---
id: BUG-111
kind: bug
title: Hand SPINOUT at the Throw peak after a setpoint 12.6 ms late (2026-11-18)
status: inbox
created: '2026-11-18'
updated: '2026-11-18T02:07:35Z'
owners: []
priority: high
hardware: robot
diagnosis: hypothesis
area: hand
blocked_by: []
links:
- logbook/2026-11-18-sitting-debrief.md
filed_by: sitting-debrief 2026-11-18
---

Sitting of 2026-11-18, launch 1, first Columns Attempt, 10:41:49.256 (temp/logs/launch_2026-11-18_1.log lines 545-575; temp/hand_capture/hand_capture_20261118_103811.csv). Related: BUG-57, FEAT-49, BUG-89. The owner reports nothing physical changed on the Hand since 2026-11-13.

- Hypothesis (the plan was not read from the bag): a Hand setpoint frame arrived 12.6 ms late, so the firmware extrapolated the Hand for about 13.6 ms at the peak of a Throw, just inside the 14 ms Hand C1 stop grace. The command peaked at 137 rev/s against 122-124 rev/s on the 17 earlier Throws, then dropped to 81 in one echo.
- The Hand overshot its command by 1.6 rev and hit the top stop at 10.90 rev (the other Throws topped out at 9.42-9.46). Braking fell to about 500 rev/s^2 at -52 to -58 A, the BUG-57 torque-loss signature, and the drive disarmed with SPINOUT_DETECTED.

Owner decision, not made: HAND_C1_STOP_GRACE_US is 14000 us (canbridge_config.h:547; it was 6000 before 2026-11-05). This event argues for less; the launch 3 stop at line 1339 missed it by 1.3 ms, which argues for more.

Next evidence: FEAT-49 armed in every launch, and the plan Hand velocity from the bag at the next Hand C1-grace promotion (command peak against plan peak).

## Sitting

- setup: level, armed, bb-fed
- why: only another fault can show whether a late setpoint at the Throw peak is the trigger.
- do: fly every launch with the FEAT-49 fault capture armed; no extra Attempts.
- pass: a SPINOUT is captured and the bag shows whether the command peak exceeded the plan peak at a late frame.
- record: the capture path, the launch log line of the SPINOUT, the nearest setpoint drain dropped line and its lag.
- minutes: 5
