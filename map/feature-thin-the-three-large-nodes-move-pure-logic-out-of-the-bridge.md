---
id: FEAT-54
kind: feature
title: 'Thin the three large nodes: move pure logic out of the bridge, skill and trajectory nodes'
status: inbox
created: '2026-11-14'
updated: '2026-11-13T23:09:41Z'
owners: []
priority: medium
effort: xl
radius: cross-cutting
blocked_by: []
links: []
filed_by: claude
---

Bring the three large nodes back toward the rule that a node is a thin wrapper over pure modules. From the 2026-11-13 maintainer review: `teensy_bridge_node.py` is 10,666 lines, `skill_node.py` 7,454 and `trajectory_node.py` 5,725, and the reviewers judged more than half of each to be logic.

## Cheapest moves first

- `skill_node.py:148-830`: about 690 lines of functions with no ROS dependence (`_frame_offset_check`, `_platform_attitude`, `_mocap_aim_point_mm`, `_columns_feed_aim_site`, `accept_cup_decision` and others). Move to `motion/skills/`.
- `teensy_bridge_node.py:634-1010` `LagClockNormalizer` (already tested without ROS) and the decoders at `:1011-1290`. Move to `teensy_link/`.
- `_wire_tilt_out` (`skill_node.py:514`) and `_wire_tilt` (`trajectory_node.py:438`) are the two halves of one encoding. Put them together with a round-trip test.

## Larger extractions

- The Feed and Columns state machine in `skill_node.py` (`_install_announced_reload`, `_start_reload`, `_run_columns` and four more, about 1,500 lines) as a pure coordinator taking a clock, an installer and parameters.
- The bridge's windowed monitors (`_latency_monitor_*`, `_note_sched_sample`, `_note_downlink_sample`, about 600 lines), the lane recovery sequence (about 450) and the `_run_*` verbs, whose pure halves already live in `teensy_link/`.
- `trajectory_node._svc_install_segment_impl` (445 lines) and `_cycle_start_state` (315).

## What holds this back

- The reviewer counted about 6,355 private-attribute accesses across `tests/ros/`, 1,545 of them in `test_skill_node.py`, which has 363 tests and no fixtures. Every extraction breaks tests that reach into the node. New tests should go through the extracted module's public functions, and each extraction should move its tests with it.
- `mpc_active` appears about 106 times in the bridge as the name of the wire arm flag, long after the MPC chain was deleted. A rename is its own grep-counted commit.
- The launch file passes `pre_release_hold_s` as `0.000` (`jugglebot_launch.py:290-296`) while the node default reads `SegmentConfig`; a changed default would be silently overridden.
