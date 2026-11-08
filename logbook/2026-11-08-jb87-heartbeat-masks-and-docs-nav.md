---
title: "FEAT-28 slice: bridge heartbeat masks read the generated enum; UDP protocol doc in the docs nav"
type: refactor
date: 2026-11-08
status: complete
phase: night-shift
files_changed:
  - mkdocs.yml
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - tests/ros/test_teensy_bridge_node_cone.py
subsystem:
  - teensy_link
tags:
  - jb-87
  - protocol
---

# FEAT-28 slice: heartbeat masks and docs nav

## What and why
The Item's Split (2026-11-07) names two things for the Night Shift. (1) `docs/teensy-udp-protocol.md` is now in the `mkdocs.yml` nav under CAN Bridge, beside Safety Mechanisms. (2) `_T2J_FLAG_MPC_ACTIVE` and `_T2J_CONE_HEALTH_MASK` in `teensy_bridge_node.py` were hand-copied literals (0x8, 0x30); they are now `int(p.HeartbeatT2JFlags.…)`, so the generated protocol is the single source. Values are unchanged (8, 48), so wire behaviour cannot change; `int()` keeps them plain ints for the bit operations.

FEAT-28 stays open for the action field, the rename and the firmware comment.

## Verification
- New test `test_heartbeat_flag_masks_come_from_generated_protocol` (tests/ros/test_teensy_bridge_node_cone.py) asserts both names equal the enum members and are plain ints. Drift detection confirmed 2026-11-08 by temporarily pointing one constant at a wrong member: test failed, then restored.
- 2026-11-08, `tools/night_shift_nonet.sh pytest -q tests/ros/test_teensy_bridge_node_cone.py tests/sim/test_docs_links.py`: 17 passed in 2.87 s.
