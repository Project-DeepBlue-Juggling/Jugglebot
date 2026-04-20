---
title: Leg 1 pose-dependent hold-phase twitch at (0,-100,200)
type: investigation
date: 2026-04-19
status: in-progress
phase: post-per-leg-gains-deadband-session
related_issues:
  - 2026-04-18-hold-fighting-motion-onset-jitter.md
  - 2026-04-18-mpc-overhead-spikes-fallback-bursts.md
  - motion-onset-deadtime-investigation.md
sessions:
  - mpc_20260419_134901.csv
  - mpc_20260419_134919.csv
  - mpc_20260419_134931.csv
  - mpc_20260419_134947.csv
  - mpc_20260419_135002.csv
  - mpc_20260419_135030.csv
  - mpc_20260419_135059.csv
  - mpc_20260419_135140.csv
  - mpc_20260419_135251.csv
rosbag: /home/jetson/Desktop/rosbags/2026-04-19_13-48-32
files_changed:
commits:
subsystem:
  - motion
  - mpc
tags:
  - hold-phase
  - pose-dependent
  - gain-tuning
  - kinematic
  - leg-1
---

# Leg 1 pose-dependent hold-phase twitch at (0,-100,200)

## Summary

A 9-move rosbag session on 2026-04-19 confirmed two known issues and surfaced one new one. The user-reported "staggered first move" matched the cold-stiction motion-onset dead-time signature (Move 1 latency 151 ms, dropping to 75-125 ms on warm moves 2-4) exactly as predicted by `plans/active/motion-onset-deadtime-investigation.md`. The user-reported "only non-smooth" final move to (0, -100, 200, 0, 0, 0) ended with 0.237 mm final tracking error, but during the 5-second hold that followed, leg 1 oscillated with 436.9 um standard deviation (1107.5 um peak-to-peak, velocity max 9.68 mm/s) while the other five legs held between 7 and 31 um stdev — a 60.3x asymmetry ratio. This is a new, pose-dependent hold-phase failure mode not covered by Iteration-3 gain tuning (which was performed at neutral poses). Moves 6-9 also showed chronic IPOPT solve-budget overruns (up to 9 consecutive violations, 40.4 ms max) — an already-tracked issue deferred to future IPOPT tuning.

## Symptoms

- User observation: first move "staggered" but smooth; all subsequent moves smooth; final move to (0,-100,200) "non-smooth."
- Move 1 (`mpc_20260419_134901`): 151 ms motion-onset dead-time, all 6 legs synchronized, 0.94 mm first-tick leap. Leg 4 tracking 1.67x median.
- Moves 2-4: onset latencies 75, 100, 100 ms — mechanism "warms up."
- Move 9 (`mpc_20260419_135251`): traversal RMS 10.90 mm peak 64.27 mm during motion to (0,-100,200), but final tracking error 0.237 mm. During the ensuing 5 s hold, leg 1 showed act_std 436.95 um, range 1107.5 um peak-to-peak, vel_abs_max 9.68 mm/s. Other legs 7.25-31.22 um stdev. Asymmetry ratio 60.3x.
- IPOPT overruns cluster on moves 6-9 (3, 5, 9, 7 consecutive `Maximum_CpuTime_Exceeded`; max solve 40.4 ms on move 8).
- User-observed pos_setpoint HF noise (ODriveGUI): diagnose detector confirms 25-31% of cmd energy in 10-20 Hz band during hold on moves 5 and 7 (`mpc_20260419_135002.csv`, `mpc_20260419_135059.csv`). Labelled "MPC numerical noise floor" — below mechanical bandwidth but visible on setpoint trace.
- Move 8 (`mpc_20260419_135140.csv`) to (100,100,200): verdict FAIL. 9 consecutive IPOPT timeouts (max 40.4 ms), leg 0 traversal RMS 2.356 mm (2.3x median), 6.648 mm RMS hold-phase error — largest tracking error in the session group by 3-5x. Reached a kinematically extreme XY corner.

## Diagnosis

Three distinct phenomena:

1. **Motion-onset dead-time (confirmed, tracked elsewhere).** Moves 1-4 reproduce the cold->warm pattern predicted in `plans/active/motion-onset-deadtime-investigation.md`. Move 1 (post-idle) 151 ms; subsequent moves 75-125 ms as the mechanism de-seats backlash / reduces stiction. The user's perception "staggered first move then smooth" maps 1:1 to detector output. No new work needed from this entry — the investigation plan already handles it.

2. **IPOPT solve-budget overruns on longer moves (tracked elsewhere).** 3/5/9/7 consecutive `Maximum_CpuTime_Exceeded` on moves 6-9. Already flagged as a deferred follow-up in `2026-04-18-mpc-overhead-spikes-fallback-bursts.md`. Not the focus of this entry.

3. **NEW: Leg 1 pose-dependent hold twitch.** At the (0, -100, 200) pose, leg 1 hold-phase stats are 436.95 um stdev / 1107.5 um peak-to-peak — 60x the quietest leg. Every prior gain-tuning iteration was performed at neutral or near-neutral poses; this pose puts the platform at an extreme asymmetric offset where some leg is likely near a stroke or geometric condition boundary, and leg 1's PID evidently enters a limit cycle that Iteration-3 tuning does not cover. Mechanism candidates to investigate:
   - (a) leg 1 is near its stroke limit at this pose (kinematic),
   - (b) the Jacobian condition at this pose is poor so small encoder noise on leg 1 translates into a large commanded-velocity signal (control architecture),
   - (c) leg 1's `pos_gain = 30` (from Iteration 3) is too soft for the loading at this pose,
   - (d) MPC weight tuning is over-penalizing a state that can only be changed by leg 1.

   Needs bench-session follow-up: repeat the hold at (0,-100,200), capture per-leg `iq`, and compare leg-1 stats at a 45-deg-rotated variant of the same pose to isolate "pose geometry" from "leg 1 specifically."

4. **NEW: pos_setpoint HF noise visible on ODriveGUI (cross-cutting).** The MPC emits a numerical residual in the 10-20 Hz band that leaks into `cmd_ext` during hold; the diagnose detector quantifies it at 25-31% of command energy on the two long-hold sessions that have it measured. Because this band sits below the mechanical bandwidth the plant does not physically chatter at that frequency, but the operator sees it on the ODriveGUI pos_setpoint trace and it is aesthetically concerning / may have downstream implications for torque smoothness. Mechanism candidates to discuss: IPOPT tolerance vs. step-to-step microscopic differences; regularization weights on u/du in the cost; ref-update cadence vs. MPC-period jitter; `cmd_ext` being the raw decoded MPC output with no output-side filter. Not yet diagnosed; raise in the Discussion section.

5. **Move 8 FAIL at (100,100,200) — possibly related to Move 9 leg-1 twitch.** Both moves 8 and 9 target extreme off-axis poses; both fail IPOPT budget and both show the worst hold-phase stats in the session group. Hypothesis worth raising in Discussion: the same "pose geometry" mechanism behind the leg-1 twitch at (0,-100,200) may also be what blows up IPOPT solve time at (100,100,200) — a poor Jacobian condition at these extreme poses increases the problem difficulty AND amplifies per-leg control sensitivity. If true, the leg-1 twitch investigation and the IPOPT-overrun investigation are not actually separable. Needs confirmation via (a) IPOPT iteration trace at extreme vs. neutral poses, (b) Jacobian condition number across the pose grid.

## Discussion

<!-- To be filled in the discussion gate. -->

## Fix

<!-- To be filled once a fix is proposed and landed. -->

## Outcome

<!-- To be filled once the fix is verified on hardware. -->

## Open Questions

- Is the leg 1 twitch specific to leg 1 (wiring, motor, ODrive instance), or is it the leg whose geometry is worst at this pose (kinematic)? The 45-deg-rotated-pose test disambiguates.
- Does leg 1's `iq` trace show a limit-cycle signature (bang-bang around zero) or a noise-amplification signature (broadband)?
- Is the MPC commanding the twitch (visible in `cmd_ext`) or is it pure ODrive-PID limit cycle (visible only in `act_ext`)?
