---
title: "Bag record passes --include-hidden-topics, so the Juggle action's feedback and status topics it already lists are recorded at last (BUG-144); unflown, a Sitting confirms it"
type: bugfix
date: 2026-11-26
status: complete
phase: "Night Shift 2026-11-26"
related_plan: ""
related_issues: [BUG-144]
files_changed:
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - tests/ros/test_launch_bag_record.py
subsystem: [ros, tools]
tags:
  - night-shift
---

# Bag record includes hidden topics

**What.** `jugglebot_launch.py` lists `/jugglebot/juggle/_action/feedback` and `/jugglebot/juggle/_action/status` in its record command. The launch comment calls them the only record of which Attempt an instant belongs to. 0 of 960 bags hold either (a metadata scan on 2026-11-25, BUG-144). They are hidden topics, because a name token starts with `_`. The record verb drops hidden topics, listed or not, unless `--include-hidden-topics` is passed. The flag is now passed. Foxy's record verb accepts it and hands it to both the all-topics and the topic-list paths (`ros2bag/verb/record.py` lines 77, 130 and 152 in `/opt/ros/foxy`).

**Test.** `tests/ros/test_launch_bag_record.py` reads the launch file's `rosbag_record` command from its AST. It asserts that a command listing a hidden topic also passes the flag. Source scan only: the launch needs real ROS 2.

**Unflown.** Only a bag the launch records can show the two topics arriving. The Item's `## Sitting` section carries that check, so this branch is held for it. The change cannot move the robot: it is the recorder's command line only.

## Verification

- 2026-11-26, `tools/night_shift_nonet.sh timeout 300 python -m pytest tests/ros/test_launch_bag_record.py tests/ros/test_launch_console.py tests/ros/test_bridge_launch_sigterm.py tests/ros/test_choreography_map.py -q -p no:cacheprovider`: **65 passed in 23.12 s**. These are the new test plus the tests that read the launch file.
- Same day, with the launch change stashed: `test_a_record_command_with_hidden_topics_includes_them` **fails**, so the test sees the defect.
