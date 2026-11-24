---
title: "Live WARN when an unlabelled marker dwells under the Hand: the trapped-Ball finder's geometry moved to a shared module plus an online detector in the Tracker node"
type: feature
date: 2026-11-14
status: complete
phase: night-shift
subsystem: tracking
related_items: [FEAT-50]
files_changed:
  - ros_ws/docs/choreography.md
  - ros_ws/src/jugglebot/jugglebot/tracking/trapped_ball.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - tools/trapped_ball_finder.py
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - tests/sim/test_trapped_ball_online.py
  - tests/ros/test_ball_tracker_trapped_warn.py
  - tools/probes/trapped_ball_replay.py
  - logbook/2026-11-14-trapped-ball-live-warn.md
  - logbook/INDEX.md
---

# Live WARN for a marker dwelling under the Hand

## What and why

BUG-84's three Hand stalls (2026-11-13) were each a Ball trapped under the Cup that the Tracker had dropped; only the raw unlabelled marker on `mocap_data` showed it, found offline by `tools/trapped_ball_finder.py`. `region_of`, `to_platform_frame`, `quat_to_matrix`, `hand_marker_local_z` moved to `tracking/trapped_ball.py` (pure, no ROS), with an `OnlineDwellDetector` fed one frame at a time. The finder imports and re-exports them (its batch `find_dwells` and `tests/sim/test_trapped_ball_finder.py` are untouched). `ball_tracker_node` logs `marker dwelling under the Hand at (x, y, z) mm for N ms, Hand at R rev` at WARN, repeated at 1 Hz while it persists. Log-only: no Hand-path action, the Teensy guard and jam detector stay the authority. The Hand-marker fit, Cup seat and dwell thresholds are `ball_tracking.trapped_*` in `hardware_config.yaml` (generated `TRACKING_TRAPPED_*`).

Wiring: the node gained `rigid_body_poses` (Platform pose; the brief named the `robot_frame` subscription but that is the world-frame fit, not the Platform) and `hand_telemetry` (`pos_meas`) subscriptions; every 6th mocap frame (about 50 Hz, as the tool) is fed. No pose yet, no telemetry or telemetry older than 0.5 s: nothing happens (pose staleness added 2026-11-25). Any detector exception is logged once and swallowed.

## Fixture-filter choice

The batch rule (voxel occupied in more than 15 % of the whole bag) has no online analogue. Online: a 10 mm voxel occupied in more than 90 % of the last 30 s of fed frames is a fixture, and only once 10 s of history exist. Costs: a Ball truly trapped for about 27 s becomes a fixture; during the first 10 s after node start a static marker can still warn. A resting Ball at (-55, 57, 3) is ON_PLATFORM only while the Hand is low (rev 0); with the Hand raised (rev 2.6) `region_of` calls the same marker UNDER_HAND (unchanged finder geometry, pinned in a test), so such a Ball can warn until the filter retires it. Not changed here; decide after the Sitting.

## Not done

- The 0.08 s at-a-stall threshold (`STALL_DWELL_MIN_S`) needs stall detection; the live WARN uses 0.30 s only.
- Sitting confirmation pending: not yet seen on hardware; replayed on bags 2026-11-25 (below).

## Verification

- 2026-11-14, `tools/night_shift_nonet.sh python -m pytest -q tests/sim/test_trapped_ball_finder.py tests/sim/test_trapped_ball_online.py tests/ros/test_ball_tracker_*.py`: 79 passed.
- 2026-11-14, `tools/night_shift_nonet.sh python -m pytest -q tests/firmware/test_config_drift.py tests/ros/test_config_identity.py tests/motion/test_hand_ball_detect_config.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py`: 48 passed.
- 2026-11-14, `tools/night_shift_nonet.sh ./run_tests.sh --full` on 4ae0a89f: 2 failed (`tests/ros/test_choreography_map.py`, the generated topic map lacked the two new subscriptions), 9592 passed; fixed by `python tools/gen_choreography_map.py`.
- New tests fail without the change: detector `DWELL_MIN_S` forced to 0.05 (3 online tests fail), region filter widened past UNDER_HAND (1 fails), node `_check_trapped` call removed (the node WARN test fails).

## 2026-11-25: the five audit findings fixed, replayed on recorded bags

Brought onto main as `offline/feat50` (cherry-pick of the held branch's three code/doc commits; main's INDEX, runbook and map files kept; config and choreography map regenerated clean, no diff). Log-only still: the change touches `trapped_ball.py`, the node's `_check_trapped` and the batch tool; nothing publishes, subscribes to a command topic or reaches the Hand or Platform path. Control-cycle walk: a mocap frame arrives, every 6th is fed, the detector returns events, the node calls `get_logger().warning`; no return value leaves `_check_trapped`.

1. Platform pose staleness: `_trapped_pose_mono` (wall clock at receipt); older than `POSE_STALE_S` (0.5 s, = the Hand's) the detector idles, as for stale Hand telemetry.
2. Prune cost: dead voxels are swept every `PRUNE_PERIOD_S` (5 s of data time) instead of every frame; only markers in a rev-independent zone (`RECORD_Z_MM`, r <= Platform radius) enter the history; `region_of` uses `math.hypot`.
3. Voxel-boundary escape: occupancy is dilated to the 26 neighbours, so a marker jittering across a boundary reads ~100 % on both voxels. Cost: a fixture's footprint is +-1 voxel.
4. Region agreement: the batch tool and the detector now share `cluster_add` / `cluster_region` / `SKIP_REGIONS`; online clusters ON_PLATFORM and UNDER_HAND markers like the batch tool and warns only when the majority region is UNDER_HAND. Deliberate remaining difference: the warning is UNDER_HAND-only; the fixture rule is rolling-window online, whole-bag in batch.
5. Pinned: decimation (`STRIDE`) and the swallowed-exception path (one log line, node survives) have node tests.

Replay (`tools/probes/trapped_ball_replay.py`, detector cost per fed frame = `to_platform_frame` + `update`, decode excluded, frames cached and re-timed 5x, lowest-mean run; box loaded by other agents, so read before/after relative; outputs in the unit's scratchpad `out/feat50/`):

| window / bag | markers per frame | before mean / p99 us | after mean / p99 us |
|---|---|---|---|
| 11-13 11-42-08, 2 min | 8.8 | 347 / 465 | 197 / 312 |
| 11-13 11-42-08, 2 min (event 2) | - | 365 / 480 | 186 / 299 |
| 11-13 13-06-39, 2 min | - | 284 / 486 | 158 / 249 |
| 11-20 13-48-32, 2 min | - | 279 / 410 | 154 / 268 |
| 11-23 14-31-06 clean oval, 8.4 min | 6.6 | 217 / 369 | 130 / 196 |
| 11-20 12-49-25 Columns, 56.9 min (after only) | 6.1 | - | 144 / 235 |

At 49.8 fed frames/s the after cost is 0.6-1.0 % of one core (Columns bag 0.72 %); before it was 1.1-1.8 %. Worst single frame 2-5 ms in the best run, 24-41 ms as the maximum over all runs (load spikes; not attributed, unchecked).

Detections against the known events: event 2 (12:08:30.63) warned at 12:08:30.964 (ball first seen ~12:08:30.61, latency 0.35 s, after the stall, which self-cleared); event 3 (13:15:22.69) warned at 13:15:22.994 (first seen ~.68, 0.31 s). Event 1 (12:02:17.66, marker sat 0.13 s) and the 2026-11-20 13:52:36 event (marker 0.20 s) are NOT detected: both are under the 0.30 s listing threshold, the "Not done" stall-aware 0.08 s bar. False warnings: the rim-pocket Ball at (-55, 57, 3) with the Hand raised is UNDER_HAND (the caveat above): 3 repeats in one episode in the 11-13 11-42-08 window, 1 episode (3 repeats) plus 1 short (0.30 s, (-49, -81, -99)) in the 56.9 min Columns bag (~0.07 warnings per minute), 0 in the clean oval bag, 0 in the 11-20 13-48-32 window; the 11-13 event 2 window also shows one extra 0.31 s warning at (50, 53, 7), Hand 8.3 rev, which the old detector did not raise.

## Verification (2026-11-25)

- 2026-11-25, `flock /tmp/jugglebot-tests.lock nice -n 10 python -m pytest -q -p no:cacheprovider tests/sim/test_trapped_ball_online.py tests/sim/test_trapped_ball_finder.py tests/ros/test_ball_tracker_trapped_warn.py`: 31 passed in 12.42 s.
- 2026-11-25, the same plus tests/ros/test_ball_tracker_*.py, test_launch_nodes, test_perception_console_lines, test_choreography_map, test_config_identity, tests/firmware/test_config_drift.py, tests/tracking/, logbook front-matter, docs-links, glossary: 287 passed, 3 failed before the final test fix; the failures were one test of mine (batch fixture rule, fixed) and the two logbook-index tests (INDEX.md is main's: regenerate at integration).
- New tests fail without the fix (2026-11-25): with the pre-fix `trapped_ball.py` and the pose-staleness line removed, 5 fail (jitter, prune timer, majority region, shared code, stale pose); with STRIDE forced to 1 and the once-flag removed, the decimation and exception tests fail (and the 1 Hz test).

- Default gate on the integrated branch, 2026-11-25 05:07 (bc8ada5d: main dd2328f6 + FEAT-50 + emit-lead P1a), `flock /tmp/jugglebot-tests.lock ./run_tests.sh`: **11213 passed, 59 skipped, 2 xfailed in 738.72 s**, serial tail **7 passed in 29.67 s**; box load average 4 falling to 3.
