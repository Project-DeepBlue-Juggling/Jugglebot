---
id: BUG-85
kind: bug
title: live_session matches node filenames in any process argv, not running nodes
status: complete
created: '2026-11-07'
updated: '2026-11-24T13:07:25Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: subsystem
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- tools/lib/live_session.sh
filed_by: claude
---

tools/lib/live_session.sh uses pgrep -f against a regex of node names, so a peer Claude session's shell command that merely ran sed on teensy_bridge_node.py (and the caller's own guard script containing the pattern) read as a live Sitting on 2026-11-07 23:02 while no ros2 launch or node process existed; the suite and build gates and the Night Shift wait on this detector, so it should match process executables or the launch's own PID file rather than any command line.

## Progress

- 2026-11-25: a default-regex pgrep hit now counts only when its argv has the shape of a node entry point, jugglebot_launch.py or ros2 launch jugglebot; an overridden regex keeps the old match. Still to confirm against a real launch (BUG-129 series). The lock-file design stays with BUG-101.
