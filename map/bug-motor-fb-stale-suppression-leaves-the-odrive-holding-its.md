---
id: JB-73
kind: bug
title: Hand ODrive state can silently diverge from what the can-bridge commanded
status: inbox
created: '2026-10-27'
updated: '2026-10-27T10:34:35Z'
owners: []
priority: low
effort: s
hardware: bench-leg
owner: decision
radius: normative
diagnosis: confirmed
area: firmware
blocked_by: []
links:
- ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
filed_by: claude
---

During a MOTOR_FB_STALE suppression the can-bridge stops transmitting leg setpoints (`s_output_enabled` gates the TX in `leg_interp.cpp`). The ODrive then holds its LAST setpoint, including its vel_ff. In POSITION/PASSTHROUGH the leg settles at input_pos + vel_ff/pos_gain and stays there until output resumes.

Seen on the bench leg on 2026-10-27 (lead-clamp P2, FW 30 and FW 33 alike), with the gap driver forcing encoder_msg_rate_ms=0. The last vel_ff was -2.0 rev/s at pos_gain 40, so the leg ended 0.05-0.056 rev (3.5-4 mm) past the last transmitted command. The worst case is the LEAD_CLAMP_VELFF_LIMIT_RPS cap of 3.5 rev/s, which gives 0.0875 rev (~6 mm) at pos_gain 40. Runs are in temp/probes/lead_clamp_gap_bench/20261027_21*.

This is bounded (the velocity error goes to 0, so the integrator does not wind) and the recovery slew re-baselines from the encoder, so it is not a runaway. But the suppressed state is meant to be a hold, and it is not one.

Candidate fix: on the suppress edge, send one final setpoint with vel_ff = 0 at the clamped position. This needs a design look against the hand lane and the stow path, which share the gate.

A related reporting quirk: LEG_CMD (`axes[].target_pos_rev`) is written on every tick even while suppressed, so it shows commands that never reached the ODrive. Bench analysers must mask them, as `tools/probes/lead_clamp_gap_bench.py` now does.

## Further instances (JB-34 survey, 2026-10-28)

The class widened: ODrive-side Hand state that drifts from what the can-bridge believes it commanded. The vel_ff hold above is the first instance; the logbook survey added three more.

- A Hand in Closed-loop but in the wrong input mode swallows the stream silently, and no telemetry sees it.
- The Guard measures the Hand's deviation against the raw plan, not the post-slew command (`leg_interp.cpp`).
- RETARGET sends `Set_Traj_Vel_Limit` (0x011) and `Set_Input_Pos` (0x00C) in the same tick; 0x00C wins CAN arbitration, so the move can be planned at the old cruise speed. Never fixed; the 2026-10-05 entry routes around it with the jam anchor.

Source entries: [logbook/2026-09-04-fw17-hand-sitting-unflashed-idle-axis.md](logbook/2026-09-04-fw17-hand-sitting-unflashed-idle-axis.md), [logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md](logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md), [logbook/2026-10-05-skill-stack-r5-sitting-5-launch-limits-and-jam-anchor.md](logbook/2026-10-05-skill-stack-r5-sitting-5-launch-limits-and-jam-anchor.md).
