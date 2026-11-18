---
title: "HumanThrowEvent message and the Tracker's human Throw emitter (PWAY-11 P1); the first event of a high Throw is about 70 mm off"
type: feature
date: 2026-11-18
status: in-progress
phase: "PWAY-11 P1 (offline)"
related_plan: "human-partner-juggling.md"
related_issues: []
files_changed:
  - ros_ws/src/jugglebot/jugglebot/tracking/human_throw_event.py
  - ros_ws/src/jugglebot/jugglebot/tracking/flight_fit.py
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - ros_ws/src/jugglebot_interfaces/msg/HumanThrowEvent.msg
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tools/probes/human_throw_bench.py
  - tests/tracking/test_human_throw_event.py
  - tests/tracking/test_human_throw_bench_events.py
  - tests/ros/test_ball_tracker_human_event.py
  - tests/ros/conftest.py
subsystem: [tracking, ros, tools]
tags: [human-throw, IPC, testing]
---

# HumanThrowEvent: the human Throw message and its emitter

## What

The Tracker now publishes `jugglebot_interfaces/msg/HumanThrowEvent` on `human_throws` for a human Ball once its Fit is usable. It carries the track id, the flight index (`flight_seq`), a revision number, the fitted Landing position and time at the Catch plane (world frame), the Fit's age, sample count, rms and span, the flight's start speed, and a `final` flag. The pure emitter is `tracking/human_throw_event.py` (`HumanThrowEventEmitter`); `ball_tracker_node` steps it once per mocap frame, only while `detect_human_throws` is true. Identity is `(track_id, flight_seq)`; `revision` 0 is the first usable Fit, later revisions refine it, `final` marks a frozen Fit. Source: `handoff-hc-u1.md`, `handoff-hc-u2.md`.

## Why a new message

Three options were compared in the design (`design-human-catch.md` section 1). Reusing the Ball Butler `ThrowAnnouncement` fails because `ball_tracker_node` subscribes to it and mints an announced track per message, so the Tracker's own event would spawn a second track for the same Ball; a re-published announcement on each refinement also looks like a duplicate to `match_announced_track`, and a synthetic `throw_time` passes an estimate off as the identity key. Reading `/balls` in the skill node is the path retired on 2026-10-17: `/balls` cannot tell a refinement from a new flight on a kept id, carries no first state (no phantom rule), no sample count or residual (no usable rule), and the retired consumer rejected candidates silently. A new message lets the Tracker apply both rules once, at the source, in a pure function the bench probe replays verbatim. The cost is one interface change: both ROS packages must be rebuilt before the first launch that uses it.

## The rules (PROPOSED owner values, not decided)

- Phantom rule (D1): a flight whose first-state speed is below 500 mm/s is not served. On the 2026-11-18 slice it rejects all idle phantoms; real Throws lost to it: 0 of 11, 1 of 44, 4 of 82.
- Usable rule (D2): `FlightFit.fit()` valid, rms at most 6 mm, and the Landing from two Fits at least 20 ms apart agreeing within 30 mm and 20 ms. Defaults live in `HumanThrowEventConfig` and are node parameters; U1's single `stable_ms` was split into `stable_gap_ms` and `stable_time_ms` (both 20) in the audit commit `c11137ba`, because one number served two purposes.

## P1 gate (2026-11-18, `human_throw_bench.py --events`, BUG-92 box on, default rules; `p1-gate.md`)

| | 11-18 launch 3 (192 s) | 11-02 passes (91 s) | 11-02 cascade (61 s) |
|---|---|---|---|
| real Throws | 11 | 44 | 82 |
| Throws with at least one event | 11 | 39 | 58 |
| first-event lead p10 / p50 / p90 (s) | 0.71 / 0.75 / 0.80 | 0.38 / 0.73 / 0.85 | 0.40 / 0.60 / 0.77 |
| first-event xy error p50 / p90 (mm) | 70 / 99 | 22 / 50 | 28 / 49 |
| first event within 50 mm and 30 ms | 2/11 | 33/37 | 49/53 |
| last revision at least 0.30 s before the plane, xy p50 (mm) | 14 | 13 | 16 |
| phantom event streams per minute | 0.0 | 0.0 | 1.0 |

Finding: the first event of a high Throw (apex 1.85 to 2.07 m, the 2026-11-18 cascade) is about 70 mm off in xy (p50), early by 3 to 37 ms, because the Fit extrapolates from a short ascent arc. It is corrected within about 0.05 s: the first revision within 50 mm and 30 ms arrives at lead p50 0.70 s for 11 of 11 Throws. D10's first-event criterion (at least 70 % within 50 mm and 30 ms) therefore fails on that bag (2/11) and passes on the other two. The D2 variants (rms 4, 6, 10 mm; agreement 20, 30, 50 mm) barely move the first event, so the owner's choice is about the criterion, not the emitter (options in `p1-gate.md` section 5). Not yet decided.

## Verification

- (2026-11-18, `python -m pytest tests/tracking tests/ros/test_ball_tracker*.py -q`, U1) 134 passed; (same command after U2) 141 passed, 7 of them new in `tests/ros/test_ball_tracker_human_event.py`.
- (2026-11-18, `colcon build --packages-select jugglebot_interfaces --build-base /tmp/hc_build --install-base /tmp/hc_install`) finished, `HumanThrowEvent.msg` and `.idl` generated, 7 min on the loaded box. The `jugglebot` package build was not run.
- The full gate result on the integrated tree is in the commit that files this entry. No node was launched and no hardware was used.

## Open questions

- Owner: D1, D2 values and the D10 first-event criterion (section 2 of the plan).
- Not fixed: no `config/` keys for the node parameters (config was frozen); `missed_frames_to_lose` is 10 in the probe and 15 in the node.
