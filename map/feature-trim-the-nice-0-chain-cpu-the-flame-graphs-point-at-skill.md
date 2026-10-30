---
id: JB-94
kind: feature
title: 'Trim the nice-0 chain CPU the flame graphs point at: skill_node executor (~0.5 core), mocap_node publish path (~0.28), tracker message plumbing (~0.28)'
status: inbox
created: '2026-10-28'
updated: '2026-10-30T06:12:20Z'
owners: []
priority: medium
area: skill-stack
blocked_by: []
links:
- 2026-10-28-jb76-solve-tail-is-environment-starvation-instrumented
parent: two-ball-oval
filed_by: claude (JB-76 21:02 Sitting read-out)
---

py-spy 100 Hz on the 21:02 Sitting (temp/probes/profile_20261028-210241/pyspy_*.svg; shares and file:line in the JB-76 logbook entry, section The Sitting of 2026-10-28 21:02).

## skill_node

- ~79 % of its work is rclpy MultiThreadedExecutor wait-set rebuild + task dispatch per message (executors.py:493-661, :347, thread.py:160-195) driven by the 200-300 Hz rigid_body_poses and balls subscriptions (skill_node.py:1334-1406).
- Fix = move the high-rate subscriptions off the tick executor or take latest-only.
- Needs a control-implication check of the tick threading (ReentrantCallbackGroup skill_node.py:1243).

## mocap_node

- Three full-rate publishes 24 %.
- RigidBodyPose() nested alloc 6.7 % (mocap_node.py:1152).
- bb/markers published when empty 3.5 % (:1179-1185).
- Per-frame robot-frame settled() 4 %.

## ball_tracker_node

- Message take/construct 27 %.
- BallState+now() 12 %.
- 300 Hz publish 8 %.
- Per-frame landing refit 10.6 % (matcher.py:1051 -> flight_fit.py:215).

## Not yet sampled

Bigger than any of these: teensy_bridge_node (79 % every Sitting) and rosbridge (70 % during goals) - sample them first.

## Progress

- 2026-10-30: 2026-10-30 (owner): this is the next CPU lever, ahead of JB-95 (backlog). Re-gate launch-wide: 444 CATCH solves, wall p50 104 / p90 136 / max 194 ms, CPU p50 81 ms, 19 solves over 150 ms.
