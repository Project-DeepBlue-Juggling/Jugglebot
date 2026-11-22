---
id: BUG-85
kind: bug
title: live_session matches node filenames in any process argv, not running nodes
status: inbox
created: '2026-11-07'
updated: '2026-11-07T18:11:53Z'
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
night_skip: frozen-path
night_skip_on: '2026-11-22'
night_skip_note: The fix is in tools/lib/ and its guard test, both on the Night Shift never-list; closes with BUG-101 in a day Session.
---

tools/lib/live_session.sh uses pgrep -f against a regex of node names, so a peer Claude session's shell command that merely ran sed on teensy_bridge_node.py (and the caller's own guard script containing the pattern) read as a live Sitting on 2026-11-07 23:02 while no ros2 launch or node process existed; the suite and build gates and the Night Shift wait on this detector, so it should match process executables or the launch's own PID file rather than any command line.
