---
id: BUG-144
kind: bug
title: Juggle action feedback and status topics are in the bag record list but have never been recorded
status: in_progress
created: '2026-11-25'
updated: '2026-11-25T17:40:44Z'
owners:
- night-shift
priority: medium
effort: xs
hardware: robot
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- ros_ws/src/jugglebot/launch/jugglebot_launch.py
filed_by: claude
---

The record list in jugglebot_launch.py names /jugglebot/juggle/_action/feedback and /jugglebot/juggle/_action/status, but 0 of 960 bags under ~/Desktop/rosbags contain either topic (metadata.yaml scan, 2026-11-25).

Cause: they are hidden topics and the ros2 bag record command does not pass --include-hidden-topics, which Foxy supports. The launch comment calls them the only record of which Attempt an instant belongs to, so every bag since R4 lacks that join.

Fix candidate: add --include-hidden-topics to the record command, then confirm both topics appear in a dress-rehearsal bag.

## Sitting

- setup: level, armed, bridge-cycled
- why: The fix (record passes --include-hidden-topics) can only be shown by a bag the launch actually records.
- do:
  - Launch with recording on, then Boot, Home, Level and Activate and run one Juggle goal (any Attempt the Sitting already flies counts)
  - After the launch ends, run `ros2 bag info` on that launch's bag directory
- pass: The bag's topic list has /jugglebot/juggle/_action/feedback and /jugglebot/juggle/_action/status, each with at least one message
- record: The message counts of both topics and the bag directory name
- minutes: 5
- branch: night-shift/2026-11-26-BUG-144

## Progress

- 2026-11-26: Night Shift 2026-11-26: held on night-shift/2026-11-26-BUG-144: record passes --include-hidden-topics, source-scan test; unblocked by a Sitting bag that lists both action topics
