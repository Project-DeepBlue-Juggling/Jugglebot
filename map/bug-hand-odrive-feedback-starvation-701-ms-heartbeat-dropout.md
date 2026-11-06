---
id: JB-137
kind: bug
title: 'Hand ODrive feedback starvation: 701 ms heartbeat dropout left the encoder 100 ms stale, the 2.0 rev lead clamp pinned the throw to stale+2.0, guard MAX_DEVIATION +3.707 fired correctly (turned-90 at 16/16)'
status: in_progress
created: '2026-11-01'
updated: '2026-11-06T09:45:57Z'
owners: []
priority: high
effort: m
hardware: robot
owner: hands
radius: normative
diagnosis: hypothesis
area: can-bridge
blocked_by: []
links:
- map/bug-teensy-guard-max-deviation-latched-on-the-hand-8-4-rev-mid.md
- map/bug-max-deviation-latched-on-40-90-ms-stale-hand-telemetry.md
- logbook/2026-11-06-sitting-debrief-2.md
filed_by: claude
---

Found at the 2026-11-01 Sitting, turned-90 goal at 16/16 (launch log L4695): a 701 ms hand heartbeat dropout, then GUARD_LATCHED.

## Findings

- The hand encoder was 100 ms stale during the dropout.
- The 2.0 rev lead clamp pinned the throw to stale encoder + 2.0 rev.
- The guard MAX_DEVIATION +3.707 rev fired correctly: the real deviation was real.
- Hypothesis: node 6 loses CAN arbitration to leg traffic. Not tested.
- Different mechanism from JB-98's high-low trip (a bridge stall).

## Wanted

- Encoder age in the census, and a look at hand-node frame loss under leg load.

## Sitting
- setup: level, armed
- why: tests the one-setting cure for the Hand feedback dropouts (a clock beat between the drive's 10 ms encoder frames and the bridge's 2 ms setpoint tick); if it holds, the dropout behind the 2026-11-06 CURRENT_LIMIT_VIOLATION cannot recur on the Hand.
- do:
  - Before the launch: on the Hand drive (odrivetool over USB, or the ODrive GUI) set `axis0.config.can.encoder_msg_rate_ms` from 10 to 9 and save. Change nothing else. The drive reboots on the save, so power-cycle the can-bridge, launch, and home the Hand as after any Hand drive save.
  - Arm and level, start no goal, and leave the robot holding for 20 minutes with the bag recording. The setpoint stream runs during an armed hold, which is all the test needs. There is nothing to watch live.
  - Then one GUI `Self-toss` goal, Apex 1.00, Dwell 0.27, Throws 20, as a check that Throws are unchanged.
  - Stop and note it on any Hand fault or guard latch. To undo: set the value back to 10, save, power-cycle the bridge, home the Hand.
  - Two things not to be misled by. The Hand's backup `config/ODrive config Files/odrive_pro_hand_config.json` still says 10, so re-applying it undoes the trial (it is updated at the debrief if 9 is kept). The launch log's heartbeat-dropout lines count `co-dropped` frames against 100 per second and misread a drive sending 111; ignore that count for the Hand.
- pass: type the result as pending. At the debrief, from the bag's `/cache_diag`: the Hand's `age_max_us_6` never exceeds 25 ms over the 20 minutes, while at least one Leg shows a window over 50 ms (the in-run control, expected every 120 to 270 s per Leg). One Hand window over 50 ms refutes the model. If no Leg shows a window over 50 ms the stream did not run and the Hand result is void. The `Self-toss` goal runs 20/20 with no `Hand lead clamp ENGAGED` line.
- record: the launch log and bag paths, the clock times of arming and of the goal, and whether the setting was left at 9 or put back.
- minutes: 30

## Sitting result (2026-11-06 20:45)

- verdict: pending
- notes: No visible issues to my eye!

## Progress

- 2026-11-02: Sitting 2026-11-02: not involved in any end (hb_stale and lead_clamp masks 0 before all 3 LANE_REFUSED). Sub-hypothesis 'node 6 loses CAN arbitration' WITHDRAWN: 7 hand dropouts all had a fresh encoder (<=24 ms) while the legs reached 99 ms. The 11-01 100 ms stale encoder did not recur: cannot tell.
- 2026-11-03: 2026-11-03 Sitting: 90 deg turned flown with no GUARD_LATCHED, no SPINOUT, no Hand error (L11704-14714), so untestable here. Related new Item JB-160: MAX_DEVIATION (L2528) fired on 40-90 ms stale Hand telemetry without MOTOR_FB_STALE.
- 2026-11-06: Second occurrence, 2026-11-06 P2 Sitting goal 10: the Hand's encoder frames fell to about 10 Hz for 0.6 s while its current frames kept arriving.
- 2026-11-06: The lead clamp held the launch at the stale position + 2.0 rev (Throw reached Apex 0.32 m), then released a 1.2 rev step: iq 81.3 A over the 80 A hard max, CURRENT_LIMIT_VIOLATION. The Hand coasted about 1 mm past the top stop at 8 rev/s or less.
- 2026-11-06: No firmware guard acts in a 30 to 150 ms feedback gap by design. Cause of the frame loss unknown (selective by message; Leg 0 showed the same cadence). The host now warns per episode (`teensy_link/hand_feedback_watch.py`). Owner decisions open: the Hand clamp anchor (port the Legs' FW 31 dead-reckoned anchor, stop, or slew back) and the 70 / 80 A gap against an 11.3 A overshoot.
- 2026-11-06: Evidence: `temp/probes/sitting_debrief_2026-11-06b/unit_hand.md`.
- 2026-11-06: Cause found offline: a clock beat. Each drive's 10 ms encoder schedule drifts into one 6 to 30 us phase of the bridge's 2 ms setpoint tick; recurrence 120 to 270 s per axis, 78 episodes in 5 bags, only while the stream runs. Withdrawn: bus load, RX overflow, poller, encoder or current fault, electrical. 89 short episodes are bag artefacts.
- 2026-11-06: Proposed, not applied: Hand encoder_msg_rate_ms 10 to 9 (worst gap 80-100 ms to about 18 ms). Test: Hand parked, stream 20+ min, age_max_us_6 under about 25 ms; one episode over 50 ms refutes it. Unknown: which side drops the frame; a passive CAN sniffer settles it before claiming entirely stopped. Firmware dither is the alternative.
- 2026-11-06: Owner answers 2026-11-06: the Hand drive hard limit of 80 A is correct as found; the Hand was up (at the top) at the fault; no change to the wiring.
