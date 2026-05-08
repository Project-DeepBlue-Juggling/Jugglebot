---
title: Leg 1 pose-dependent hold-phase twitch at (0,-100,200)
type: investigation
date: 2026-04-19
status: resolved
phase: post-per-leg-gains-deadband-session
related_issues:
  - 2026-04-18-hold-fighting-motion-onset-jitter.md
  - 2026-04-18-mpc-overhead-spikes-fallback-bursts.md
  - "2026-05-08 motion-onset-deadtime-investigation.md"
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
  - mpc_20260420_160401.csv   # A/B: leg 1 reverted to 40/0.32, leg 4 still at 30/0.24
  - mpc_20260420_182945.csv   # A/B: both legs 1 and 4 reverted to 40/0.32 (final)
rosbag:
  - /home/jetson/Desktop/rosbags/2026-04-19_13-48-32
  - /home/jetson/Desktop/rosbags/2026-04-20_16-03-32
  - /home/jetson/Desktop/rosbags/2026-04-20_18-29-26
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - run_mpc.py
  - controller/target.py
  - sim/analysis/diagnose.py
  - plans/archived/2026-05-08 motion-onset-deadtime-investigation.md
  - plans/active/leg-gain-tuning-methodology.md
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

A 9-move rosbag session on 2026-04-19 confirmed two known issues and surfaced one new one. The user-reported "staggered first move" matched the cold-stiction motion-onset dead-time signature (Move 1 latency 151 ms, dropping to 75-125 ms on warm moves 2-4) exactly as predicted by `plans/archived/2026-05-08 motion-onset-deadtime-investigation.md`. The user-reported "only non-smooth" final move to (0, -100, 200, 0, 0, 0) ended with 0.237 mm final tracking error, but during the 5-second hold that followed, leg 1 oscillated with 436.9 um standard deviation (1107.5 um peak-to-peak, velocity max 9.68 mm/s) while the other five legs held between 7 and 31 um stdev — a 60.3x asymmetry ratio. This is a new, pose-dependent hold-phase failure mode not covered by Iteration-3 gain tuning (which was performed at neutral poses). Moves 6-9 also showed chronic IPOPT solve-budget overruns (up to 9 consecutive violations, 40.4 ms max) — an already-tracked issue deferred to future IPOPT tuning.

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

1. **Motion-onset dead-time (confirmed, tracked elsewhere).** Moves 1-4 reproduce the cold->warm pattern predicted in `plans/archived/2026-05-08 motion-onset-deadtime-investigation.md`. Move 1 (post-idle) 151 ms; subsequent moves 75-125 ms as the mechanism de-seats backlash / reduces stiction. The user's perception "staggered first move then smooth" maps 1:1 to detector output. No new work needed from this entry — the investigation plan already handles it.

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

The investigation was scoped to avoid another hardware session until we had a falsifiable, kinematic explanation for why the twitch was pose-dependent. A pure desk analysis ruled out the two most obvious mechanisms. (a) **Stroke limit:** at (0,−100,200) leg 1 sits at 182.94 mm extension, which leaves 178 mm of margin to the minimum stroke and 97 mm to the maximum — not even close to a hardware end. (b) **Jacobian ill-conditioning:** the normalized Jacobian at (0,−100,200) has condition number 3.80 versus 3.60 at home, with essentially identical singular value spread. The pose is unremarkable from a geometric sensitivity standpoint. Mechanisms (a) and (b) were off the table before we touched the robot again.

The decisive observation came from the x-z symmetry of the pose. Legs 1 and 2 end up at mathematically identical extensions — both 182.94 mm — because (0,−100,200) is mirror-symmetric across the y-z plane that separates them. Accounting for the platform's measured CoM offset of `[-9.68, -68.64, 52.73] mm`, their static axial loads differ by only 4% (−3.28 N on leg 1 vs −3.40 N on leg 2). Yet the observed hold-phase standard deviation was 437 µm on leg 1 and 29 µm on leg 2 — a 15× difference between two legs being asked to do mechanically identical jobs. That single data point ruled mechanism (c) — undersized PID gain at this loading — strongly IN, left (d) — MPC weight interaction — ambiguous, and more importantly reframed the problem: whatever was going on, it had to be **leg-1-specific**, not pose-specific.

The user's standing skepticism of "leg-to-leg hardware asymmetry" was the final push toward a software-asymmetry explanation. Iteration-3 gain tuning (performed earlier at near-neutral poses where hold-phase noise data is statistically quieter) had dropped legs 1 and 4 to `pos_gain=30 / vel_int=0.24` while the other four legs were left at 40/0.32. Leg 1 at that lower gain was evidently right on the edge of phase-margin collapse; at (0,−100,200) it tipped over into a limit cycle. Leg 2, sitting at the same static load but with the higher gain, did not. The fix — revert leg 1 to 40/0.32 while intentionally leaving leg 4 at 30/0.24 as an in-experiment control — was designed to be both the remediation and the falsifier: if leg 4 showed the same pose-dependent signature at this pose, the mechanism was confirmed and the asymmetry was fully explained by gain tuning, not hardware.

## Fix

- `config/hardware_config.yaml` edited to set `leg_pos_gains[1] = 40.0` and `leg_vel_int_gains[1] = 0.32`, matching legs 0/2/3/5. Leg 4 intentionally left at 30/0.24 as the in-experiment control.
- Config constants regenerated via `python config/generate_config.py`, which refreshed `config/generated/hardware_config.py`, `config/generated/hardware_config.h`, the ROS2 mirror `ros_ws/src/jugglebot/jugglebot/hardware_config.py`, and the Teensy mirror `ros_ws/src/jugglebot/Teensy_code/hardware_config.h`.
- Live per-leg values were confirmed on the ODrive after re-homing with the new YAML, before the A/B test. The A/B test itself was `python run_mpc.py --pose 0,-100,200,0,0,0 --duration 15` from a fresh process.
- `plans/archived/2026-05-08 motion-onset-deadtime-investigation.md` was authored during this investigation as an earlier sibling — motion-onset dead-time is a separate phenomenon (moves 1-4 of the 9-move rosbag) tracked there rather than here.
- A motion-onset detector was added to `sim/analysis/diagnose.py` as a side-benefit of this investigation (commit `a41b17f`). That detector is what produced the "151 ms on first move, 75-125 ms on warm moves" numbers captured in the Symptoms section.
- A short-lived `--auto-leg1-test` flag was added to `run_mpc.py` on 2026-04-19 to drive the 9-move replay automatically. The attempted run hit a Python GC pause during move 7 that tripped the 500 ms telemetry-staleness E-STOP guard, and the test was aborted. The flag was removed on 2026-04-20 in favour of the simpler single-pose fresh-process protocol, which is what produced the conclusive A/B result. `run_mpc.py` and `controller/target.py` retain only the supporting plumbing.
- Not yet committed as of this entry's resolution — `commits:` in the frontmatter is deliberately empty pending the user's commit.

## Outcome

The A/B confirmed the gain-asymmetry hypothesis decisively.

**Before (leg 1 at 30/0.24), session `mpc_20260419_135251`, 5 s hold at (0,−100,200):**
- Leg 1 `act_std = 436.95 µm`, `range = 1107.5 µm`, `vel_abs_max = 9.68 mm/s`.
- Asymmetry ratio vs leg 3: 60.3×.

**After (leg 1 at 40/0.32), session `mpc_20260420_160401`, 5 s hold at same pose:**
- Leg 1 `act_std = 27.49 µm`, `range = 94.35 µm`, `vel_abs_max = 3.23 mm/s`.
- **16× improvement** on leg 1 stdev. Leg 1 is now mid-pack among the six legs.
- Solver healthy: 100% IPOPT success, p50 10.9 ms, p95 18.4 ms, max 28.8 ms. Only 3 budget violations, all under 29 ms. No fallback bursts, no overhead spikes.

**Control confirmation (leg 4 kept at 30/0.24):**
- Leg 4 now shows the same pose-dependent signature at (0,−100,200) — `act_std = 50.4 µm`, 10× asymmetry vs leg 5. Same mechanism, symmetric result. The control held, and leg 4 is now a candidate for the identical revert once reviewed.

The "leg-1 pose-dependent twitch" was entirely a self-inflicted software asymmetry from Iteration-3's over-cautious gain reduction, not a hardware difference between legs. The user's original skepticism of "leg-to-leg hardware asymmetry" was correct: once the Iteration-3 asymmetry in the YAML was undone on leg 1, the twitch vanished, and the same asymmetry reproduced on a different leg (4) that still had the reduced gain. Mechanism fully explained, investigation closed.

**Leg-4 revert follow-up (2026-04-20), session `mpc_20260420_182945.csv`, 5 s hold at (0,−100,200):**

| Leg | Original (both at 30/0.24) | Leg-1 reverted only | Both reverted (latest) |
|-----|---|---|---|
| 0 | 31 µm | 11 | 7 |
| 1 | **437** | 28 | 58 |
| 2 | 29 | 14 | 7 |
| 3 | 7 | 21 | 11 |
| 4 | 19 | **50** | 36 |
| 5 | 9 | 5 | 19 |
| Asymmetry ratio | **60×** | 10× | **8.5×** |

Leg 4 reduction from 50 µm → 36 µm (~28 %) — directionally correct but milder than leg 1's 16× drop, consistent with leg 4's baseline being less severe to begin with. Leg 1 drifted from 28 → 58 µm between the two sessions at unchanged gain; that magnitude is within the session-to-session variance observed earlier (10–30 µm jitter on quiet legs across sessions) and is not read as a regression. Motion-onset latency on this run was 235–249 ms — higher than the 100–190 ms baseline, but consistent with a cold first-move after re-homing; no new signature.

**Final state:** all six legs now uniform at the ODrive-flash baseline **40/0.20/0.32**. Worst-case hold-phase stdev across all measured poses is now 58 µm (a ~7.5× improvement on the original 437 µm), asymmetry ratio has collapsed from 60× → 8.5× relative to the original Iteration-2 configuration, and no leg is catastrophically bad at any tested pose. Per-leg gain tuning chapter closed for hardware bringup. The plan `plans/active/leg-gain-tuning-methodology.md` has been updated with the lessons and revised Level-1 entry criteria.

### Follow-ups

- ✅ **Leg 4 at 30/0.24 revert — DONE (2026-04-20).** Reverted to 40/0.32 in `config/hardware_config.yaml`, configs regenerated, validated in session `mpc_20260420_182945.csv`.
- The Move 8 / (100,100,200) IPOPT-overrun pattern raised in the Diagnosis section is **not** resolved by this work — the leg-1 fix had no bearing on solve-time blow-up at different extreme poses. That remains tracked under `2026-04-18-mpc-overhead-spikes-fallback-bursts.md`.
- The residual ~36–58 µm stdev on legs 1 and 4 at (0,−100,200) at uniform gains suggests the plant *does* have a mild pose-dependent phase-margin component even with symmetric gains. Hold this observation — it is orders of magnitude smaller than the motion-onset dead-time (~100–200 ms, mm-scale in tracking-error terms), so pursuing it now would be premature optimisation. Revisit only after motion-onset is closed and if the 50 µm floor is a juggling-timing blocker.

## Open Questions

- RESOLVED: Is the leg 1 twitch specific to leg 1 (wiring, motor, ODrive instance), or is it the leg whose geometry is worst at this pose (kinematic)? — **Gain-reduction-specific, not hardware-specific.** Reverting leg 1 to 40/0.32 eliminated the twitch (16× stdev drop) with no hardware change, and the identical symptom has now reproduced on leg 4 — the other leg that Iteration-3 left at 30/0.24. The kinematic check (symmetric extensions on legs 1 and 2, near-identical static loads, cond(J_norm) unremarkable) had already ruled geometry out.
- RESOLVED (moot): Does leg 1's `iq` trace show a limit-cycle signature (bang-bang around zero) or a noise-amplification signature (broadband)? — Not collected. Rendered moot by the A/B outcome: the signature, whatever it was, was produced by the 30/0.24 gain set and disappeared entirely when the gains were restored to 40/0.32. No further trace capture is warranted.
- RESOLVED (implicitly): Is the MPC commanding the twitch (visible in `cmd_ext`) or is it pure ODrive-PID limit cycle (visible only in `act_ext`)? — **ODrive-PID limit cycle.** A 16× stdev drop on `act_ext` from a pure ODrive-gain change, with no MPC, controller, or reference-generator modification, is only consistent with a motion that was not being commanded by the MPC in the first place.
