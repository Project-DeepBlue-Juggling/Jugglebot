---
id: BUG-144
kind: bug
title: Juggle action feedback and status topics are in the bag record list but have never been recorded
status: inbox
created: '2026-11-25'
updated: '2026-11-25T12:02:10Z'
owners: []
priority: medium
blocked_by: []
links:
- ros_ws/src/jugglebot/launch/jugglebot_launch.py
filed_by: claude
---

The record list in jugglebot_launch.py names /jugglebot/juggle/_action/feedback and /jugglebot/juggle/_action/status, but 0 of 960 bags under ~/Desktop/rosbags contain either topic (metadata.yaml scan, 2026-11-25).

Cause: they are hidden topics and the ros2 bag record command does not pass --include-hidden-topics, which Foxy supports. The launch comment calls them the only record of which Attempt an instant belongs to, so every bag since R4 lacks that join.

Fix candidate: add --include-hidden-topics to the record command, then confirm both topics appear in a dress-rehearsal bag.
