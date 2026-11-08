---
title: "Frame check reports the Platform's tilt and yaw in the robot frame and their change since the previous check (report only); it would have shown the 2026-11-07 bad re-seat as +1.30 deg about y, +2.39 deg yaw"
type: feature
date: 2026-11-08
status: resolved
phase: "frame check"
related_issues:
  - BUG-72
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node_platform_attitude.py
subsystem:
  - skill-stack
tags:
  - mocap
  - observability
---

## What and why

At the 2026-11-07 Sitting a Leg left its base joint unnoticed. The re-seat that followed put the Platform about 14 mm and 2.7 deg off, and the tilt caused four hop misses. The session-start frame check reads position only (limit 25 mm), so it passed (`logbook/2026-11-07-sitting-debrief.md`). The owner chose a report-only line (2026-11-08).

`skill_node` now keeps the mocap Platform body's orientation beside its position, and at each frame check logs one more DEBUG line:

`Platform attitude (mocap body, robot frame, report only): tilt T deg (about x X, about y Y), yaw Z deg; since the previous check this launch: about x dX, about y dY, yaw dZ deg`

- **Refuses nothing.** A failure to measure costs only this line.
- **Not compared with the commanded attitude**, because `trajectory/commanded_position` carries no orientation. The useful number is the change since the previous check in the same launch.
- **Pure function:** `_platform_attitude`.

## Verification

- **Replay of the 2026-11-07 bag** (Base body as the robot frame, through `_platform_attitude`):
    - checks before the bad re-seat changed by 0.10 deg or less;
    - the first check after it read about y +1.30 and yaw +2.39 deg.
- **Tests:** `pytest -q tests/ros/test_skill_node_platform_attitude.py` (2026-11-08): 8 passed. Default gate: see the commit message.
