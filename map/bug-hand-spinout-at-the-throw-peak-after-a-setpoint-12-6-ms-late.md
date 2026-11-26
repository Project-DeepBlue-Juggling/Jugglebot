---
id: BUG-111
kind: bug
title: Hand SPINOUT at the Throw peak after a setpoint 12.6 ms late (2026-11-18)
status: in_progress
created: '2026-11-18'
updated: '2026-11-25T04:58:09Z'
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
night_skip: sitting
night_skip_on: '2026-11-26'
night_skip_note: A day Session built the lead-1 arm (branch sitting/emit-lead-1); the paired A/B is its Sitting section.
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
- why: Emit lead 1 (setpoint frames sent one knot earlier) should remove the late-frame step behind BUG-111's Hand SPINOUT and the stamp-gap STOPs behind BUG-89; only a paired Sitting, same Patterns and time on lead 0 (main) and lead 1 (branch sitting/emit-lead-1), can show it. The nodes fix the lead at import, so each arm needs a checkout, a build and a relaunch; there is no runtime switch.
- do:
  - Arm A, on main (lead 0): launch with the bag recording. Dress rehearsal: grep each node's start-up line "wire budget:", it must read emit_lead_knots=0 WIRE_READ_KNOTS=3 in trajectory_node and skill_node.
  - Arm A rehearsal, one nominate call per Pattern; each must answer accepted and moves nothing: columns,apex_m=0.95,separation_mm=125,dwell_s=0.27; columns_turned,...,columns_angle_deg=45 and =90; oval_1ball_fed,apex_m=1.00,separation_mm=150,dwell_s=0.50; columns_1ball_fed at the usual working point. columns_turned needs mocap on the robot base markers and Ball Butler's Base markers.
  - Arm A flights, 10 minutes of Attempt time per group: Columns, turned Columns (45 degrees), oval_1ball_fed, columns_1ball_fed. Note the clock time of each Attempt. Then Ctrl-C the launch (Cup empty, Platform stowed).
  - Switch to arm B: from ~/Desktop/Jugglebot run git fetch && git checkout sitting/emit-lead-1 && git merge --ff-only origin/sitting/emit-lead-1 && git log --oneline -1 and read the log line (it must start with the branch tip the Session names). Then cd ~/Desktop/Jugglebot/ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot. Then relaunch exactly as arm A.
  - Arm B rehearsal on the loaded Jetson (bag recording and GUI up) before arming: "wire budget:" must read emit_lead_knots=1 WIRE_READ_KNOTS=4 in BOTH trajectory_node and skill_node; then the same nominate calls as arm A, every one accepted. If either line reads otherwise, stop, do not arm, go back to main.
  - Arm B flights: the same four groups, same Patterns and parameters, same Attempt time as arm A. Abort arm B at the first Hand SPINOUT, Guard Fault or E-stop button press: Ctrl-C, then switch back.
  - Switch back (always, even after an abort): Ctrl-C the launch, git checkout main && git pull --ff-only && git log --oneline -1 (go on only if git printed Switched to branch 'main' or Already on main), then colcon build --packages-select jugglebot_interfaces jugglebot, and relaunch only if more flights follow.
- pass: Arm B has zero Hand SPINOUT; in-Attempt stamp-gap STOPs per Attempt-minute at most 1/4 of arm A's; SPLICE_TOO_LATE plus ORIGIN_TOO_LATE ends per Attempt-minute no higher than arm A's; zero REBASE_DISCONTINUOUS and zero Attempt-start Hand resume refusals; Columns Catches per Attempt not below arm A's minus arm A's own spread. Read arm B's lag and frame-age diagnostics as 25 ms different from arm A's by design.
- record: Per arm and Pattern: launch log and bag paths, Attempt minutes, Hand SPINOUT lines, firmware scheduler STOP lines (stamp gap) inside Attempts, Hand C1 stops, LANE_REFUSED ends, count of "setpoint drain dropped" lines, SPLICE_TOO_LATE, ORIGIN_TOO_LATE, REBASED and REBASE_DISCONTINUOUS lines, Attempt-start Hand resume refusals, re-aims accepted and declined with reason, Catches per Attempt, max emit gap, the "wire budget:" line.
- minutes: 150
- branch: sitting/emit-lead-1

## Sitting result (2026-11-20 15:11)

- verdict: inconclusive
- notes: A SPINOUT shows up in the `grep` readout (sheet 2026-11-20-robot, step 16) [derived: A SPINOUT occurred (launch_2026-11-20_1.log:5298) but was not captured. The bag's 100 Hz echo shows the command peaks equal to the preceding Throws (9.357 rev, 123.55 rev/s) with a 27.3 ms-late frame dropped 41 ms before the fault line (line 5296); it cannot resolve a 500 Hz excursion and no plan replay was run.]

## Progress

- 2026-11-18: Bag review confirmed it: firmware extrapolation across the release knot of a frame 12.6 ms late produced the 137 rev/s command and a -39 to -45 rev/s step when the frame was promoted. Grace stays at 14 ms (owner); options recorded in the body, to decide after the current work.
- 2026-11-20: Flown at the Sitting of 2026-11-20, verdict unresolved: a SPINOUT occurred at 13:22:42 during the post-release brake of a normal Throw (Hand 0.46 rev above its command at the disarm) but was not captured; the 100 Hz echo shows command peaks equal to the preceding Throws, with a 27.3 ms-late frame dropped 41 ms before. Missing:
- 2026-11-20: the 500 Hz plan replay with the arrival times of seq 50752 and 50753, or a saved capture.
- 2026-11-25: 500 Hz replay (tools/probes/hand_late_knot_replay.py) confirms both SPINOUTs: a late release-knot frame let the firmware extend the accelerating cubic, then its promotion stepped the velocity command by 34 to 40 rev/s (2026-11-18) and 18 to 38 rev/s (2026-11-20). They are the only 2 of 209 replayed drops at a release knot above 90 rev/s. Emit lead 1 removes both steps.
- 2026-11-25: emit lead 1 designed and phase 1 landed: the planner's wire horizon is 3 + emit_lead_knots, the fresh-REST rebase threshold is the onset budget minus (lead + 1) knots (50 ms at lead 1, not 100), and the stall fence stays 37.5 ms. The configured lead is still 0 and every constant is identical. Next: a lead-1 test tier, the cost measurement, then seeding holds at the wire horizon.
- 2026-11-25: lead-1 arm built as branch sitting/emit-lead-1 (pushed, never merged): 975 planner and node tests and the five-configuration sim gate pass at the configured lead 1. The Sitting section is now the paired A/B (main at lead 0 against the branch at lead 1), owner decision 2026-11-25.
