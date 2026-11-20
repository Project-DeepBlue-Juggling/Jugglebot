---
id: BUG-111
kind: bug
title: Hand SPINOUT at the Throw peak after a setpoint 12.6 ms late (2026-11-18)
status: inbox
created: '2026-11-18'
updated: '2026-11-20T04:13:00Z'
owners: []
priority: high
effort: m
hardware: robot
owner: decision
radius: subsystem
diagnosis: confirmed
area: hand
blocked_by: []
links:
- logbook/2026-11-18-sitting-debrief.md
- logbook/2026-11-20-sitting-debrief.md
filed_by: sitting-debrief 2026-11-18
night_skip: waiting-trigger
night_skip_on: '2026-11-19'
night_skip_note: Bag review confirmed; options recorded in the body. The owner decides after the current work (2026-11-18).
---

Sitting of 2026-11-18, launch 1, first Columns Attempt, 10:41:49.256 (temp/logs/launch_2026-11-18_1.log lines 545-575; temp/hand_capture/hand_capture_20261118_103811.csv). Related: BUG-57, FEAT-49, BUG-89. The owner reports nothing physical changed on the Hand since 2026-11-13.

## Findings (2026-11-18, bag review, confirmed)

Method: the bag has no Hand knot topic, so the knot plan was rebuilt from the firmware command echo (velocity residual 0.002 rev/s) and replayed through the firmware twin with the real arrival times. The numbers below are the record (the scripts lived in the session scratchpad); the hypothesis was first written in logbook/2026-11-18-sitting-debrief.md.

- The plan never asked for 137 rev/s. Its velocity at the release knot is 126.0 rev/s, the same as four earlier Throws of the Attempt (126.00 to 126.03).
- At the release knot the plan flips from +1126 rev/s^2 to about -2150 rev/s^2. The late frame (seq 2724, 12.6 ms late) was the one carrying that flip.
- The firmware extended the accelerating cubic meanwhile: the two echoes left out of the fit are predicted to four digits, 127.17 rev/s and 136.52 rev/s, where the plan read 123.80 and 102.02.
- The 137 to 81 drop is the promotion of the late frame, not a C1 stop (`hand_c1_stops +0`): a command velocity step of -39 to -45 rev/s in one 2 ms tick.
- The commanded position never exceeded the plan's 9.395 rev. The overshoot to 10.90 rev and the weak braking are the drive's response to that step (the BUG-57 signature; not diagnosed here).
- Lateness alone is the cause. The dropped knot (seq 2723) changes nothing in the twin, and the BUG-110 emitter fix would not have prevented this event.
- The lateness looks bridge-side (inference): emitter max gap 26.1 ms, bridge forward gap 54.7 ms, bridge loop gap 36.9 ms. That is the BUG-89 and BUG-58 stall class.
- The step grows about 3 rev/s per ms of extrapolation across a release knot (-7.7 rev/s at 3 ms, -50.2 at 17 ms).

## What a different Hand C1 stop grace would have done (twin, command only)

| Grace | Peak command | Outcome |
|---|---|---|
| 6 ms | 132.2 rev/s, 8.17 rev | late frame refused, latched hold, Attempt ends, no velocity step |
| 10 ms | 135.7 rev/s, 8.81 rev | the same |
| 14 ms (now) | 136.5 to 138.0 rev/s, 9.395 rev | frame promoted with the step that preceded the SPINOUT |

Neither direction gives a clean Throw: of the three C1 stops on 2026-11-05, two ended in MAX_DEVIATION and one in SPINOUT, and on 2026-11-18 a C1 stop that missed the grace by 1.3 ms ended in MAX_DEVIATION.

## Options (owner, 2026-11-18: recorded, decide after the current work)

- Leave `HAND_C1_STOP_GRACE_US` at 14000 us (agreed 2026-11-18).
- Emit lead 1 for the Hand, so the release knot and its brake are on board before they are due. Owner: prudent, but may not fully resolve it.
- Constant-velocity extrapolation of the Hand in the firmware, in place of extending the accelerating cubic.
- Find why the bridge loop stalls (BUG-112, BUG-89), since that is what made the frame late.

Not checked: which tick promoted the frame; the Hand's physical response in any counterfactual; other late frames near release knots across the five launches.

## Sitting

- setup: level, armed, bb-fed
- why: only another fault can show whether a late setpoint at the Throw peak is the trigger.
- do: fly every launch with the FEAT-49 fault capture armed; no extra Attempts.
- pass: a SPINOUT is captured and the bag shows whether the command peak exceeded the plan peak at a late frame.
- record: the capture path, the launch log line of the SPINOUT, the nearest setpoint drain dropped line and its lag.
- minutes: 5

## Sitting result (2026-11-20 15:11)

- verdict: inconclusive
- notes: A SPINOUT shows up in the `grep` readout (sheet 2026-11-20-robot, step 16) [derived: A SPINOUT occurred (launch_2026-11-20_1.log:5298) but was not captured. The bag's 100 Hz echo shows the command peaks equal to the preceding Throws (9.357 rev, 123.55 rev/s) with a 27.3 ms-late frame dropped 41 ms before the fault line (line 5296); it cannot resolve a 500 Hz excursion and no plan replay was run.]

## Progress

- 2026-11-18: Bag review confirmed it: firmware extrapolation across the release knot of a frame 12.6 ms late produced the 137 rev/s command and a -39 to -45 rev/s step when the frame was promoted. Grace stays at 14 ms (owner); options recorded in the body, to decide after the current work.
- 2026-11-20: Flown at the Sitting of 2026-11-20, verdict unresolved: a SPINOUT occurred at 13:22:42 during the post-release brake of a normal Throw (Hand 0.46 rev above its command at the disarm) but was not captured; the 100 Hz echo shows command peaks equal to the preceding Throws, with a 27.3 ms-late frame dropped 41 ms before. Missing:
- 2026-11-20: the 500 Hz plan replay with the arrival times of seq 50752 and 50753, or a saved capture.
