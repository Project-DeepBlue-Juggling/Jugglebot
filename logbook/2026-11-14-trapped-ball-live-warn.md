---
title: "Live WARN when an unlabelled marker dwells under the Hand: the trapped-Ball finder's geometry moved to a shared module plus an online detector in the Tracker node"
type: feature
date: 2026-11-14
status: complete
phase: night-shift
subsystem: tracking
related_items: [FEAT-50]
files_changed:
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
  - logbook/2026-11-14-trapped-ball-live-warn.md
  - logbook/INDEX.md
---

# Live WARN for a marker dwelling under the Hand

## What and why

BUG-84's three Hand stalls (2026-11-13) were each a Ball trapped under the Cup that the Tracker had dropped; only the raw unlabelled marker on `mocap_data` showed it, found offline by `tools/trapped_ball_finder.py`. `region_of`, `to_platform_frame`, `quat_to_matrix`, `hand_marker_local_z` moved to `tracking/trapped_ball.py` (pure, no ROS), with an `OnlineDwellDetector` fed one frame at a time. The finder imports and re-exports them (its batch `find_dwells` and `tests/sim/test_trapped_ball_finder.py` are untouched). `ball_tracker_node` logs `marker dwelling under the Hand at (x, y, z) mm for N ms, Hand at R rev` at WARN, repeated at 1 Hz while it persists. Log-only: no Hand-path action, the Teensy guard and jam detector stay the authority. The Hand-marker fit, Cup seat and dwell thresholds are `ball_tracking.trapped_*` in `hardware_config.yaml` (generated `TRACKING_TRAPPED_*`).

Wiring: the node gained `rigid_body_poses` (Platform pose; the brief named the `robot_frame` subscription but that is the world-frame fit, not the Platform) and `hand_telemetry` (`pos_meas`) subscriptions; every 6th mocap frame (about 50 Hz, as the tool) is fed. Missing pose, no telemetry or telemetry older than 0.5 s: nothing happens. Any detector exception is logged once and swallowed.

## Fixture-filter choice

The batch rule (voxel occupied in more than 15 % of the whole bag) has no online analogue. Online: a 10 mm voxel occupied in more than 90 % of the last 30 s of fed frames is a fixture, and only once 10 s of history exist. Costs: a Ball truly trapped for about 27 s becomes a fixture; during the first 10 s after node start a static marker can still warn. A resting Ball at (-55, 57, 3) is ON_PLATFORM only while the Hand is low (rev 0); with the Hand raised (rev 2.6) `region_of` calls the same marker UNDER_HAND (unchanged finder geometry, pinned in a test), so such a Ball can warn until the filter retires it. Not changed here; decide after the Sitting.

## Not done

- The 0.08 s at-a-stall threshold (`STALL_DWELL_MIN_S`) needs stall detection; the live WARN uses 0.30 s only.
- Sitting confirmation pending: not yet seen on hardware or replayed on a bag; the live Tracker CPU cost of the decimated feed (FEAT-30 flags the Tracker) is unmeasured.

## Verification

- 2026-11-14, `tools/night_shift_nonet.sh python -m pytest -q tests/sim/test_trapped_ball_finder.py tests/sim/test_trapped_ball_online.py tests/ros/test_ball_tracker_*.py`: 79 passed.
- 2026-11-14, `tools/night_shift_nonet.sh python -m pytest -q tests/firmware/test_config_drift.py tests/ros/test_config_identity.py tests/motion/test_hand_ball_detect_config.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py tests/sim/test_glossary_avoid_words.py`: 48 passed.
- New tests fail without the change: detector `DWELL_MIN_S` forced to 0.05 (3 online tests fail), region filter widened past UNDER_HAND (1 fails), node `_check_trapped` call removed (the node WARN test fails).
