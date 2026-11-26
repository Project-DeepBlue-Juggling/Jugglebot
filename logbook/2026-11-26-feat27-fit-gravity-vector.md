---
title: "FEAT-27: the Fit takes gravity's horizontal part in the Mocap world frame (config gravity_world_xy_mmps2, default [0, 0] = today bit for bit, live ROS parameter for the Sitting A/B); every landing consumer traced: the Catch gains, the Learner re-learns a constant shift in about 30 Throws and its Memory is kept, Ball Butler's aim was calibrated on measured crossings and is untouched"
type: feature
date: 2026-11-26
status: in-progress
phase: "FEAT-27 gravity vector, held for a Sitting"
related_plan: ""
related_issues: []
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/tracking/ballistics.py
  - ros_ws/src/jugglebot/jugglebot/tracking/flight_fit.py
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - ros_ws/src/jugglebot/jugglebot/tracking/human_throw_event.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - tools/probes/tracker_bag_replay.py
  - tests/tracking/test_flight_fit_gravity_tilt.py
  - tests/ros/test_ball_tracker_gravity_tilt.py
  - logbook/2026-11-26-feat27-fit-gravity-vector.md
subsystem: [tracking, config]
---

## Summary

The Mocap world frame is tilted 0.55 deg from gravity: measured (+43, +83, -9782) mm/s^2, drag removed, from 1,139 free flights (FEAT-27 map note of 2026-11-26, `temp/probes/feat27-gravity/`). The Tracker's Fit assumed gravity along -z, which puts the landing (-8, -21) mm off in the Robot frame for an early Fit and (-3, -11) mm for the final Fit. This change gives the Fit, its landing and every other landing the Tracker publishes a horizontal gravity vector, `ball_tracking.gravity_world_xy_mmps2`, **default [0, 0]**, which keeps today's Fit bit for bit. The owner switches it on at a Sitting with `ros2 param set /ball_tracker_node gravity_world_xy_mmps2 "[43.0, 83.0]"`. Held on `offline/feat27-gravity` until that Sitting.

## Motivation: the interaction check (done before the build)

### Which landing each consumer reads

| Consumer | What it reads (file:line on main 8afbab7d) | Which Fit and when | Frame |
|---|---|---|---|
| Tracker publish | `matcher._update_landing_prediction` (matcher.py:1137) prefers `FlightFit.landing`, else the Kalman extrapolation (`landing_from_fit` False) | every frame | Mocap world |
| Catch aim and re-aims | `skill_node._on_balls` (skill_node.py:1797) -> `_world_to_schedule` (:1832, yaw-only robot frame, minus the `_mocap_to_schedule_mm` offset) -> `executor._catch_aim` order (executor.py:4696): a converged Fit first, then the Learner's prior, then the Kalman fallback; re-aims in `_resend_live_catch` when the Fit moves at least 10 mm or 10 ms (:2831), at most 2 per Catch (:2845), none inside `CATCH_FREEZE_S` = `LEAD_S` + one knot (:286) | first converged Fit about 0.18 s after release (p50, executor.py:693); the Cup's final position is the last re-aim's Fit, at some age between the first Fit and the freeze (not measured per Catch: unchecked) | schedule = Robot frame minus offset |
| Learner / Memory row | `_consider_landing` (executor.py:7140) stores `best_landing`: the LAST converged Fit before the ball's next release, i.e. the final Fit (all samples down to the 250 mm freeze floor) | final Fit | schedule frame; Memory persists per plant_id across Sittings (`memory.py`, `temp/learn/<plant_id>/memory_v2.csv`) |
| Learner prior aim | `_prior_landing` under `AIM_SCHEDULE_LEARNED`: the Learner's forward prediction, trained on final-Fit rows | indirectly the final Fit | schedule frame |
| Report and records | `report.py:73,113` `landing_err_mm` = `best_landing` minus target, for Throws and Feeds | final Fit | schedule frame |
| Ball Butler Feed Catch | `_install_announced_reload` (skill_node.py:6875): first aim = BB's announced landing (BB solver + affine, no Fit), then re-aims on the Fit as above | as the Catch | schedule frame |
| BB aim calibration | `resources/throw_affine_correction.json`, fitted 2026-10-09 in the BallButler repo by `analyze_local_calibration.extract`, which measures the crossing from a LOCAL free-acceleration fit around the plane ("never a prediction") | not the Tracker's Fit | BB-local |
| Human Throw event | `human_throw_event._evaluate` (human_throw_event.py:216) calls `predict_landing_state` on the Fit's state directly | every revision | Mocap world, then `_world_to_schedule` (skill_node.py:6546) |
| Kalman fallback | matcher.py:1171, ranked BELOW the prior for aiming; refused as a Memory row (needs `from_fit`) | early flight only | Mocap world |
| Offsets | `_frame_offset_check` (skill_node.py:700): the Platform body's Mocap position minus the commanded position at rest, no ball involved; `seat_offset_m` in Memory is the Catch aim itself, not a tuned value; `columns_feed_bb_bias_mm` is (0, 0) | none | |
| Tools that re-fit | `tools/probes/tracker_bag_replay.py` (mirrors the node's constructor), `tools/tracking_analyzer.py`, `tools/probes/human_throw_bench.py`, `tools/probes/tracker_fit_latency/*` | | |

### What changes when the Fit becomes unbiased

- **The Catch gains.** Today the Learner makes the final Fit land on the Site, so the true landing sits about (+3, +11) mm (Robot frame) from where the final Fit says, and the Cup, aimed at a Fit, is that much or more off the ball: (3, 11) mm against a final-Fit re-aim, up to (8, 21) mm against an early one. With the vector the Cup goes to the true landing within about 1.5 mm, and the early and final Fits agree, so fewer re-aims cross the 10 mm threshold. No offset was tuned against the biased Fit: the frame-offset check uses the Platform body at rest, the seat offset follows the aim, the robot frame template comes from the base markers, and the BB bias is zero.
- **The Learner re-learns a constant, and the robot's own Throws do not get worse.** Every Memory row's observed landing carries the final-Fit bias, and so does the learned command. At the switch the commands do not change, so the true landings do not move: the first Throws land exactly where they do today. What jumps is the OBSERVED error, from about 0 to about (+3, +11) mm, and the Learner then walks the command back. A closed-loop run of the live `Learner` (default config: k 16, w0 2, forgetting 0.99; plant identity plus an offset, final-Fit noise sd (4, 5) mm; 400 Throws with the bias, then 120 without; three seeds; `$S/out/feat27-fix/learner_step.py`) gave the TRUE landing error in y: 11.0 mm before the switch, 5-14 mm over Throws 0-4 after, 1.5-4.7 mm over Throws 8-16, -0.6..3.9 mm over 16-24, and under 1.5 mm from Throw 32 on. So about 16 to 30 Throws per (ball, release kind) cohort. Clearing the Memory instead throws away the apex gain K (1.081 against the identity prior's 1, 73.5 mm cold-start apex error, `plant_model.py` doc): keep the Memory. Shifting old rows by a constant is not exact either (the Robot-frame bias depends on each Sitting's yaw) and the Learner does it unaided within a Sitting.
- **Ball Butler.** Its aim correction was fitted on measured crossings, so it is unbiased already; today its Feed `landing_err_mm` in records reads the BB error plus the final-Fit bias, and after the switch reads the BB error alone. The Feed's Catch gains as any Catch does.
- **Records.** `landing_err_mm` before and after the switch are not comparable: the switch moves the recorded landing of the robot's own Throws by about (+3, +11) mm for the first ~30 Throws, then back toward zero as the Learner converges. A Sitting's records should note the switch time.
- **No consumer is harmed.** The Learner transient leaves the true landing unchanged at the switch and improves it after; every aim moves toward the ball.

## Discussion

**Why a horizontal vector and not re-levelling the Mocap frame.** Re-levelling QTM would remove the tilt for every consumer at once, but it moves the world frame, which invalidates the Robot frame template, the Ball Butler base frame and the kinematic calibration (FEAT-27 note). A vector in the Tracker is local, reversible with one parameter, and testable offline.

**Why only the horizontal part.** The measured vertical part reads -9782 against the configured 9806; that discrepancy is a separate open question (and 0.45 mm/s^2 of the tilt's own effect on z is negligible). The z root keeps `GRAVITY_MMPS2`, so the landing TIME is unchanged by this change.

**What follows the vector and what does not.** Everything that produces a landing someone aims at: the Fit's model and residual gate, `FlightFit.landing`, the Human Throw event's landing (it calls `predict_landing_state` itself), and the Kalman fallback's extrapolation (correct physics whatever the filter's internal model, and it is a published landing). Left alone, with the numbers: the announced-ball confirmation gate (the tilt moves a ball 0.5 x 94 x 0.8^2 = 30 mm over a 0.8 s flight against a 200 mm sphere); the parabolic human-Throw test (94 mm/s^2 against its 400 mm/s^2 threshold); the Kalman filter's own process model (94 mm/s^2 over one 3-5 ms frame is about 0.001 mm, far below its measurement noise); `KalmanFilter.predict_landing` (no live caller); the sim (MuJoCo's world is level and the sim never builds a `BallTracker`). Ball Butler's announced landings are BB-solver predictions in the world frame under -z gravity: they are only the first aim of a Feed's Catch and are replaced by the Fit, so they stay.

**Why a live parameter.** The A/B wants both arms on one Sitting without a relaunch, as `detect_human_throws` does. A change applies to fits in flight at their next `fit()` (one step in that flight's estimate), so switch between Attempts.

## Fix

- `ballistics.predict_landing_state(..., g_xy_mmps2=None)`: landing xy plus `0.5 g_xy t^2`, landing velocity plus `g_xy t`; None or (0, 0) takes the old arithmetic untouched.
- `FlightFit(gravity_xy_mmps2=(0, 0))`: fit targets `pos - 0.5 g tau^2` and the residual model include (gx, gy) only when non-zero; `landing()` passes the vector on.
- `BallTracker(gravity_world_xy_mmps2=...)` hands it to every new FlightFit and to the Kalman fallback's landing; `set_gravity_world_xy` updates the tracker and the fits in progress.
- `ball_tracker_node`: parameter `gravity_world_xy_mmps2` (default from config), live via `_on_set_parameters`, refusing anything but two finite numbers; the start-up line prints it.
- `config/hardware_config.yaml` `ball_tracking.gravity_world_xy_mmps2: [0.0, 0.0]` with the measured value, its source and its staleness in the comment; regenerated (`TRACKING_GRAVITY_WORLD_XY_MMPS2`; the firmware headers gain an unused `GRAVITY_WORLD_XY_MMPS2` constant, no firmware reads it). The BallButler tree was clean before and after the generator.
- `tools/probes/tracker_bag_replay.py` passes the config value, so a replay matches the live constructor.

## Verification

- 2026-11-26, `python -m pytest tests/tracking/test_flight_fit_gravity_tilt.py -q`: **12 passed**. The synthetic Hand-Throw-like flight (300 Hz, apex about 1.08 m, the measured (43, 83) tilt) lands within 0.0001 mm with the vector. Without it the landing is off by (-3.7, -7.2) mm for the final Fit and (-14.0, -27.1) mm for a 0.2 s Fit, which equals the least-squares prediction to 0.05 mm. With (0, 0) the fit and landing match a frozen copy of the pre-change arithmetic bit for bit (`np.array_equal`, three noisy seeds with outliers and in-cup samples).
- Shown to fail: on the unchanged tracking modules, all 12 fail (the TypeError on the new keyword). Mutant A (the fit ignores the vector) fails 3; mutant B (the landing ignores it) fails 2; mutant C (the tracker does not hand it to new fits) fails 1. 2026-11-26, same command.
- 2026-11-26, `python -m pytest tests/ros/test_ball_tracker_gravity_tilt.py -q`: **3 passed**; on the unchanged `ball_tracker_node.py`, **3 failed**.
- 2026-11-26, scoped run over the readers found with `grep -rln` (the tracking package's importers and the readers of `hardware_config`): `flock /tmp/jugglebot-tests.lock timeout 900 nice -n 10 python -m pytest tests/tracking tests/ros/test_ball_tracker_*.py tests/ros/test_launch_nodes.py tests/ros/test_skill_node_human_catch_audit.py tests/sim/test_trapped_ball_online.py tests/firmware/test_config_drift.py tests/ros/test_gui_geometry.py -q -p no:cacheprovider -n 2`: **377 passed in 32.68 s**.
- Not run here: the full gate (the orchestrator runs it at integration), `python -m mapkit gen` (so `logbook/INDEX.md` gets this row at integration).

## Outcome

Built and held. At the next launch with the default config nothing changes. The Sitting switch-on procedure is in FEAT-27's `## Sitting` section.
