---
id: JB-195
kind: bug
title: 'Leg encoder dropouts of 80 to 99 ms continue at the clock beat: trial encoder_msg_rate_ms 9 on Leg 4, then all Legs'
status: inbox
created: '2026-11-06'
updated: '2026-11-08T00:13:23Z'
owners: []
priority: medium
effort: s
hardware: robot
owner: hands
radius: subsystem
diagnosis: hypothesis
area: can-bridge
blocked_by: []
links:
- map/bug-hand-odrive-feedback-starvation-701-ms-heartbeat-dropout.md
- logbook/2026-11-06-sitting-debrief-3.md
filed_by: claude
---

The Hand's cure held at the Sitting of 2026-11-06 20:20 (JB-137). All six Legs, still at 10 ms, kept their episodes in the same bag: 4 to 8 episodes over 50 ms in 1303 s, recurring every 111 to 254 s, up to 99.1 ms. The Legs' clamp anchor dead-reckons through such a gap and MOTOR_FB_STALE acts at 150 ms, and no Attempt end has been attributed to one so far (not searched for specifically): this widens a margin, it does not cure a known fault.

Leg 4 goes first because it is the strongest test: 8 consecutive recurrences 124 to 130 s apart, so 20 minutes expects 7.6 episodes and zero by chance is about 5e-4 (the Hand's trial was about 8 %, on count alone). If it passes, all Legs go to 9: six Legs add 67 frames/s, +0.74 to 0.90 % of the bus. The Legs share one backup JSON (`config/ODrive config Files/odrive_pro_leg_config.json`), so it changes only when all six do.

Carried decisions from JB-137, none applied. (1) The Hand lead-clamp anchor: port the Legs' FW 31 dead-reckoned anchor, stop, or slew back; a long dropout did not occur at 9 ms, but the design still allows the step. (2) The 70 / 80 A gap against the 11.3 A overshoot.

Decided 2026-11-06, for the record. (3) Hand gaps of 20 to 33 ms against the 30 ms gate: owner kept 9 on 2026-11-06; 7 ms would make them 14 to 21 ms for +43 frames/s per axis. (4) Hand heartbeat and error frame rates: heartbeat gaps of 700 to 800 ms continue every 156 to 162 s; owner took the recommendation to wait and change one thing per trial.

Still unknown: which side drops the frame (a passive CAN sniffer settles it), why only some crossings collapse into a run at 10 ms, and iq per unit commanded acceleration 13 % above the P2 goals at 9 ms (the two 10 ms sets differ by 19 %, unattributed). Evidence: `temp/probes/sitting_debrief_2026-11-06c/unit_9ms.md`.

## Sitting
- setup: level, armed
- why: tests the Hand's 9 ms cure on the axis where zero episodes by chance is about 5e-4, far stronger than the Hand's own trial, and decides whether all six Legs change.
- do:
  - Before the launch: set Leg 4 only, `axis0.config.can.encoder_msg_rate_ms` from 10 to 9, and save, the way you set the Hand (`tools/odrive_fleet_reflash.py --only 4 --set axis0.config.can.encoder_msg_rate_ms=9` also does this and reads it back from the drive's saved config; it is untried for this setting). The drive reboots on the save and loses its zero.
  - Launch. Check the log says `homing: 7 axes homed` before arming; if it does not, stop and do not arm. The Hand trial ran this way with no bridge power-cycle and no symptom (one run).
  - Before arming, read `/cache_diag`: `enc_frames_4` and `enc_frames_6` should climb by about 111 per s, the others by about 100. Otherwise the setting is not in; stop.
  - Arm and level, start no goal, and hold for 20 minutes with the bag recording. There is nothing to watch live.
  - Stop and note it on any Leg fault or guard latch. To undo: set Leg 4 back to 10, save, and launch again so the homing runs.
- pass: type the result as pending. At the debrief, from the bag's `/cache_diag`: `age_max_us_4` never exceeds 25 ms over the 20 minutes, while at least one other Leg shows a window over 50 ms (the in-run control). One Leg 4 window over 50 ms refutes the cure for the Legs; no window over 50 ms on any other Leg voids the result. Leg 4's frame-loss spacing should fall to about 62 s from 124 to 130 s.
- record: the launch log and bag paths, the clock time of arming, and whether Leg 4 was left at 9 or put back.
- minutes: 30

## Progress

- 2026-11-08: Leg 4 flew at encoder_msg_rate_ms 9 through the whole 2026-11-07 launch (owner set it before; in-run, not the parked hold the row asks for): Leg 4 peak 17.1 ms, 0 of 2396 /cache_diag windows over 25 ms over 40 min; Legs 0-3 and 5 peak 99-100 ms, 20-30 windows over 25 ms; Hand 18.1 ms, 0. Frame-loss spacing not measured. Row and status left to the owner.
