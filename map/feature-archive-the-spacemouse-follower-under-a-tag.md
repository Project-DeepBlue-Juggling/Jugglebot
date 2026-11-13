---
id: FEAT-57
kind: feature
title: Archive the SpaceMouse follower under a tag
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:10:02Z'
owners: []
priority: low
area: skill-stack
blocked_by: []
links: []
filed_by: claude
---

Archive the SpaceMouse follower path under a tag, so it can be brought back later. Owner, 2026-11-14: not used for some time, happy to archive, likely to want it again.

## Findings (2026-11-13 maintainer review)

- `spacemouse_handler` is started unconditionally by the launch (`jugglebot_launch.py:433`).
- In `trajectory_node.py` the follower is `_on_platform_pose` (`:2276`), `_follower_tick` (`:2676`) and the `TargetFollower` it drives (`:685`), about 150 lines plus `motion/trajectory/follower.py` and `chase.py`.
- `trajectory/timed_target` (`_svc_timed_target` `:3798-3937`, `_plan_and_install_timed` `:3738-3797`, about 200 lines) has no caller outside `tests/ros/test_trajectory_node.py` and old hardware notes. `_ros_time_to_perf` (`:3583`) says in its own docstring that it has no production caller. Whether `timed_target` goes too is the owner's call.

## Wanted

- A grep-counted removal: the handler, the launch entry, the follower code and its tests, with an `archive/` tag on the last commit that has them and a note of the tag in the logbook entry.
- Check what else imports `follower.py`, `chase.py` and `_install_continuity_ok` before deleting; the reviewer saw the last one still called at `:5049`.
