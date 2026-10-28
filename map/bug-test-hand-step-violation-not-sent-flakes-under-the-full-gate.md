---
id: JB-79
kind: bug
title: test_hand_step_violation_not_sent flakes under the full gate (assert 2 == 1)
status: inbox
created: '2026-10-28'
updated: '2026-10-28T01:00:25Z'
owners: []
priority: medium
blocked_by: []
links:
- tests/ros/test_teensy_bridge_node_setpoint.py
filed_by: claude
---

Seen 2026-10-28 in ./run_tests.sh (parallel phase) on an agent worktree whose diff touched only ros_ws/gui/js/quick-capture.js, index.html and tests/ros/test_gui_quick_capture.py: tests/ros/test_teensy_bridge_node_setpoint.py::test_hand_step_violation_not_sent failed with assert 2 == 1 (a hand frame counted as sent when it should have been held back). Scoped rerun of the file: 36 passed in 11.6 s. Same shape as JB-64 and JB-66 (a wall-clock wait or an un-injected clock under four-worker load). Hand-safety surface: never demote to nightly or widen a tolerance; find the clock. Log: temp/logs/D_jb78_gate.log in that worktree (scratchpad wt-jb78).
