---
id: BUG-19
kind: bug
title: Hand ODrive state can silently diverge from what the can-bridge commanded
status: in_progress
created: '2026-10-27'
updated: '2026-11-01T05:18:45Z'
owners:
- jugglebot-9d
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
- logbook/2026-11-01-jb73-suppress-edge-hold-flush.md
filed_by: claude
---

During a MOTOR_FB_STALE suppression the can-bridge stops transmitting leg setpoints (`s_output_enabled` gates the TX in `leg_interp.cpp`). The ODrive then holds its LAST setpoint, including its vel_ff. In POSITION/PASSTHROUGH the leg settles at input_pos + vel_ff/pos_gain and stays there until output resumes.

Seen on the bench leg on 2026-10-27 (lead-clamp P2, FW 30 and FW 33 alike), with the gap driver forcing encoder_msg_rate_ms=0. The last vel_ff was -2.0 rev/s at pos_gain 40, so the leg ended 0.05-0.056 rev (3.5-4 mm) past the last transmitted command. The worst case is the LEAD_CLAMP_VELFF_LIMIT_RPS cap of 3.5 rev/s, which gives 0.0875 rev (~6 mm) at pos_gain 40. Runs are in temp/probes/lead_clamp_gap_bench/20261027_21*.

This is bounded (the velocity error goes to 0, so the integrator does not wind) and the recovery slew re-baselines from the encoder, so it is not a runaway. But the suppressed state is meant to be a hold, and it is not one.

Candidate fix: on the suppress edge, send one final setpoint with vel_ff = 0 at the clamped position. This needs a design look against the hand lane and the stow path, which share the gate.

A related reporting quirk: LEG_CMD (`axes[].target_pos_rev`) is written on every tick even while suppressed, so it shows commands that never reached the ODrive. Bench analysers must mask them, as `tools/probes/lead_clamp_gap_bench.py` now does.

## Further instances (IDEA-2 survey, 2026-10-28)

The class widened: ODrive-side Hand state that drifts from what the can-bridge believes it commanded. The vel_ff hold above is the first instance; the logbook survey added three more.

- A Hand in Closed-loop but in the wrong input mode swallows the stream silently, and no telemetry sees it.
- The Guard measures the Hand's deviation against the raw plan, not the post-slew command (`leg_interp.cpp`).
- RETARGET sends `Set_Traj_Vel_Limit` (0x011) and `Set_Input_Pos` (0x00C) in the same tick; 0x00C wins CAN arbitration, so the move can be planned at the old cruise speed. Never fixed; the 2026-10-05 entry routes around it with the jam anchor.

Source entries: [logbook/2026-09-04-fw17-hand-sitting-unflashed-idle-axis.md](logbook/2026-09-04-fw17-hand-sitting-unflashed-idle-axis.md), [logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md](logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md), [logbook/2026-10-05-skill-stack-r5-sitting-5-launch-limits-and-jam-anchor.md](logbook/2026-10-05-skill-stack-r5-sitting-5-launch-limits-and-jam-anchor.md).

## Options (2026-11-01)

1. Suppress-edge flush: on the suppress edge send one final setpoint at the clamped position with vel_ff = 0, for every lane the gate covers (legs, hand, stow). One edge, one frame; needs the design look the body names.
2. Legs only: the flush on the leg lanes, hand and stow untouched until each is looked at. Smaller, leaves the Hand instance open.
3. Report only: leave the hold as it is (bounded, no wind-up, re-baselined on recovery) and fix the LEG_CMD reporting quirk so bench analysers see which commands reached the ODrive. No behaviour change.
4. Fold into the next can-bridge firmware revision with the other three instances (input-mode check, Guard against the post-slew command, RETARGET's 0x011 / 0x00C arbitration order) as one firmware unit with a bench-leg Sitting.

Recommendation: 1 if the owner wants the suppressed state to be a true hold, flown first on the bench leg with the gap driver (`encoder_msg_rate_ms=0`) and the pass criterion "leg within 0.01 rev of the last transmitted command after suppression"; otherwise 3 now and 4 when the firmware is next touched.

A question ticket carries this to the owner; the answer under its `## Answer` re-opens this Item.

## Sitting

- setup: bridge-cycled
- why: confirm FW 35's suppress-edge hold flush leaves the bench leg at the last transmitted command instead of input_pos + vel_ff/pos_gain, and measure the current step it costs.
- do:
  - FW 35 is already flashed (2026-11-01). Power-cycle the bridge after re-cabling to the bench leg and confirm `BRIDGE_IDENTITY` reads FW 35 / PROTOCOL 10.
  - Run `tools/probes/lead_clamp_gap_bench.py` with the 2026-10-27 recipe (2.0 rev/s triangle) and rate-0 gaps (`--gaps start_s:0:0.5`) placed mid-ramp, so vel_ff is non-zero at the suppress edge.
  - Analyse with `--analyse temp/probes/lead_clamp_gap_bench/<run>`.
- pass: in every rate-0 gap the leg settles within 0.01 rev of the last transmitted command (FW 33: 0.05-0.056 rev), MOTOR_FB_STALE self-clears when frames return, no MAX_DEVIATION latch, and the iq step at the suppress edge stays under about 10 A with no visible jolt (else switch to the pre-registered vel_ff ramp, logbook 2026-11-01-jb73-suppress-edge-hold-flush).
- record: per gap the settle offset (rev), the last vel_ff, the ODrive iq step at the edge, and the run directory.
- minutes: 30

## Progress

- 2026-11-01: FW 35 suppress-edge hold flush committed (TKT-20 option 1), not flashed; bench-leg Sitting section added
- 2026-11-01: FW 35 flashed 16:15 (owner asked); BridgeIdentity reads FW 35 / PROTOCOL 10; bench-leg Sitting still to fly
