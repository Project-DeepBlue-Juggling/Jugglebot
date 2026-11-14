---
title: "Human-throw detection drops candidates that start inside the Ball Butler frame box (BUG-92, TKT-71 option 3)"
type: bugfix
date: 2026-11-15
status: complete
phase: "Night Shift"
subsystem: tracking
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py
  - ros_ws/src/jugglebot/jugglebot/ball_tracker_node.py
  - tests/tracking/test_matcher.py
  - tests/ros/test_ball_tracker_frame_stamp.py
  - tests/ros/test_ball_tracker_gate.py
  - logbook/INDEX.md
---

## What and why

With `detect_human_throws` on, static Ball Butler frame markers started `human_throw` tracks: 12 of 124 live tracks and 26 of 30 bench phantoms at the Sitting of 2026-11-13 began on fixed cells (x -730 to -988, z 1665 to 1861 mm). The label filter cannot name them (QTM labels the flying ball `Ball Butler - 1`) and a parabola tolerance cannot help. The owner chose option 3 of TKT-71: a Ball Butler zone in config ("Ball Butler won't move, and I'll never be partner juggling with Jugglebot from near Ball Butler").

- `ball_tracking.human_throw_exclusion_{x,z}_{min,max}_mm` = x [-1028, -690], z [1625, 1901] (measured cells padded 40 mm per side); y unbounded, as the option names x and z only. min > max disables it. Generated with `python config/generate_config.py`.
- `BallTracker(human_throw_exclusion_box=(x_min, x_max, z_min, z_max))`, default None: `_detect_parabolic` does not open a new marker track for an unmatched marker inside the box. Announced balls never pass through that path, so Ball Butler Throws are untouched; nothing changes with `detect_human_throws` off. A track that starts outside and moves in keeps going. The node wires the box from `hw` and logs it in its start-up info line beside `detect_human_throws=`.

## Frame check

The tracker runs in the mocap (world) frame: `_on_mocap` passes `msg.markers[].position` untouched (only the landing plane is converted, via `robot_frame`). The debrief numbers come from raw `/mocap_data` markers (the offline bench) and live `/balls` tracks, both in that frame. The debrief does not name the frame in words, so this is inferred from the data path, not stated by it.

## Verification

- 2026-11-15, `night_shift_nonet.sh python -m pytest tests/tracking/ tests/ros/test_ball_tracker_frame_stamp.py tests/ros/test_ball_tracker_gate.py -q`: 83 passed.
- Without the matcher check (`if False:` in place of the box test), `-k ExclusionBox`: the phantom-inside-box test fails, the other four pass (they are guards: the same marker 100 mm outside the box, a true throw outside, an announced release inside, and the no-box control).
- Not run: `./run_tests.sh` (Night Shift gate owns it).
