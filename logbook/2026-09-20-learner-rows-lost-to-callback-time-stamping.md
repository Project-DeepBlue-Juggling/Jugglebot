---
title: "34 of 60 flights left the learner no row — the tracker fitted against the callback clock, not the frame's time; plus the crossing freeze retired, a refused re-send made terminal, the re-send cap a parameter, the Kalman stepping by the measured interval"
type: investigation
date: 2026-09-20
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot_interfaces/msg/MocapDataMulti.msg
  - ros_ws/src/jugglebot/jugglebot/mocap_interface.py
  - ros_ws/src/jugglebot/jugglebot/mocap_node.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tools/probes/tracker_bag_replay.py
  - tools/probes/README.md
  - tests/ros/conftest.py
  - tests/ros/test_ball_tracker_frame_stamp.py
  - tests/ros/test_mocap_interface.py
  - tests/ros/test_mocap_node.py
  - tests/ros/test_skill_node_resend_param.py
  - tests/motion/test_skills_executor.py
  - ros_ws/src/jugglebot/jugglebot/tracking/tests/test_matcher_frame_dt.py
---

## Symptom

Sitting 2026-09-18 16:16 (bag `2026-09-18_16-16-17`, log `temp/logs/launch_r2gate_20260918_1616.log`)
flew 60 throws with no early end and 26 of 26 rows caught — and only 26 rows. 34 outcomes read
`no row: no converged ballistic fit`, all in the chained schedules (1-throw attempts 4/5 rows,
5-throw 1/5, 10-throw 5/10, two 20-throw 8/20 each). The learner ran on 43 % of what it threw.

## Diagnosis — the hypothesis that did not survive, and the one that did

The first reading was the outcome FREEZE: the row froze at the scheduled crossing and the fit often
converges later. Replaying the bag through the real `BallTracker` (`tools/probes/tracker_bag_replay.py`
machinery, `scratchpad/yield_replay.py`) refuted it: **54 of 60** flights had a fitted landing BEFORE the
scheduled crossing in replay. The LIVE `/balls` stream in the same bag carried a fitted landing for only
**33 of 60** flights, and 27 flights had zero fitted samples out of ~650. The live tracker and the
replayed tracker are the same code on the same frames; what differs is the TIME each frame is fitted
against. `ball_tracker_node._on_mocap` stamped every frame with the node clock at callback time; under
sitting load the callback runs late and unevenly, and the batch fit (`tracking/flight_fit.py`, 12 mm
residual gate) sees a jittered time base — at 3 m/s a 10 ms error is 30 mm. Replaying with Gaussian
jitter on the frame times reproduces the live loss exactly:

| jitter σ on frame time | flights with a fitted landing |
|---|---|
| 0 (bag receipt time) | 54 / 60 |
| 3 ms | 55 / 60 |
| 6 ms | 43 / 60 |
| 10 ms | 23 / 60 |
| live, callback-time stamping | 33 / 60 |

The mocap interface already had the fact: QTM stamps every packet, and `mocap_interface` keeps a
smoothed `ros_ns ≈ qtm_us·1000 + offset`. The published `MocapDataMulti` carried no time.

## Discussion

Stamp at the source, once. A frame's markers and its QTM time are one snapshot under the interface's
`data_lock`; the node publishes `stamp` on the ROS clock through the existing offset (zero before the
offset is established) and the tracker fits against it, falling back to the callback clock only when
the stamp is zero. The replay prefers the stamp too, so it replays what the node saw; old bags have no
stamp and fall back to log time, which is why the replay's 54/60 is unchanged by this entry.

Three things the same investigation settled: (1) the crossing freeze and the crossing guard
(`OUTCOME_GUARD_S`) both guarded a Kalman estimate that wobbles at its own crossing; a converged fit is
the same parabola before, at or after it, every announcement mints its own tracker id and the host
correlates per release, and the outcome is the fit's apex — so the row now freezes at the ball's NEXT
release and admits a fit that converged after the landing. (2) A re-send refused on a limit is now
terminal for that catch: 18 of 21 timing-only re-sends on 09-18 were refused `LIMIT_JERK`, most a
second refusal for the same catch, each a 40–130 ms solve; the dive only tightens toward touch-down.
(3) The Kalman predict stepped by the nominal 5 ms while the frames run at 5.2 ms; it now steps by the
measured interval, clamped to [0.4, 4]× nominal (measured: +119 mm/s of velocity lag at the real frame
rate, +1698 mm/s at 8 ms frames, against +2 / +20 mm/s stepping by the measured gap). Only the published
position/velocity and the pre-fit fallback depend on it.

Two probes ran and changed nothing: the Jetson scheduling holes (`scratchpad/stall_probe.py`) are rare on
the 16:16 bag (9 hand-telemetry gaps > 30 ms, max 57.6 ms; 13 setpoint gaps > 45 ms, max 74 ms) and NOT
locked to the bridge's 10 Hz publish (3 of 9 within 12 ms vs 2.2 expected by chance) — general jitter
that only an in-process callback timer could attribute, and harmless since the rate-bound step gate.
And the learner's hyperparameters (`scratchpad/learner_grid_probe.py`: plant `y = 0.085 + 0.984·u`,
residual 49 mm, R² 0.83; 27 configs × 200 seeds × two apexes) are flat — every configuration enters the
42 mm band in 2.7–2.9 throws with 43–50 mm steady error, the plant's own scatter. No re-pin.

## Fix

- `MocapDataMulti.msg`: `builtin_interfaces/Time stamp`; `mocap_interface.latest_frame_ros_ns()` (one
  snapshot with the markers); `mocap_node._publish_mocap_data` fills it at both publish sites;
  `ball_tracker_node._on_mocap` fits against it, one-shot logs on first stamp and on any fallback;
  `tracker_bag_replay._iter_bag` prefers it.
- `executor._consider_landing`: test 2 (crossing guard) retired, test 3 freezes at the next release;
  `_resend_live_catch`: a refused install clears the live catch (sixth fence); docstrings follow.
- `skill_node`: `catch_resend_max` parameter (0..5, clamped with a WARN; 0 disables re-aiming) passed
  as `resend_max_per_catch` to both executor constructions.
- `tracking/matcher.py`: `_frame_dt` and `kf.predict(dt)`.
- Tests: the two freeze tests rewritten to the new rule, the guard test replaced by "a fit at or after
  its crossing is admitted and the last wins", a terminal-refusal test, `test_matcher_frame_dt.py`,
  `test_ball_tracker_frame_stamp.py`, the mocap node/interface stamp tests, the parameter test.

## Verification

(2026-09-20, `pytest tests/motion/test_skills_executor.py ros_ws/src/jugglebot/jugglebot/tracking/tests tests/ros/test_ball_tracker_flight_fit.py tests/ros/test_ball_tracker_gate.py tests/ros/test_skill_node_resend_param.py -q`):
**all pass** (the executor file 116, the tracking package 32, the tracker gate/fit 27, the parameter 5).
(2026-09-20, unit 6, `pytest tests/ros/test_mocap_node.py tests/ros/test_mocap_interface.py tests/ros/test_ball_tracker_frame_stamp.py tests/ros/test_ball_tracker_flight_fit.py tests/ros/test_ball_tracker_gate.py ros_ws/src/jugglebot/jugglebot/tracking/tests -q`): **132 passed in 1.89 s.**
(2026-09-20, `python scratchpad/yield_replay.py ~/Desktop/rosbags/2026-09-18_16-16-17` after the change): **54/60, unchanged** (old bag, no stamps).
(2026-09-20, `./run_tests.sh --full` in a throwaway worktree of the COMMIT — the working tree also carried the contract session's uncommitted work — log `~/Desktop/Jugglebot-verify/temp/logs/verify_amended_full.log`): **PASS — parallel 6321 passed, 9 skipped, 2 xfailed in 288.41 s; serial 6 passed in 18.93 s.**

## Carried

- The live yield after this change is unmeasured until the next sitting: expect rows on ≥ 50 of 60
  flights (the replay's 54 minus the fits that converge only after the next release).
- The working tree also carries the cup-contact contract session's uncommitted work; that session
  owns the admissible-box re-sweep its `feasibility.py` edit requires (the mocked start services refuse
  `gate_hash` mismatch until it lands — not this entry's).
