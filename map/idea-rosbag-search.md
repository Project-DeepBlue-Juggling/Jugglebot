---
id: IDEA-4
kind: idea
title: Rosbag search
status: complete
created: '2026-10-24'
updated: '2026-11-01T03:06:02Z'
owners: []
priority: low
effort: m
hardware: none
owner: decision
radius: local
area: gui
blocked_by: []
links: []
filed_by: harrison
---

A mechanism to search through rosbags from recordings to find specific information.

## Options (2026-11-01)

1. A CLI, `tools/rosbag_search.py`: open an mcap bag read-only, filter by topic, time window and a field predicate (`--topic /skills/attempt --where "code==HAND_LIMIT_ACC"`), print matching messages one per line with their stamps. Offline, testable with a small fixture bag under `tests/`, effort s to m.
2. A search box in the GUI replay's digest view (the replay already indexes a bag per topic): type a topic and a substring, the trackbar jumps to hits. Effort m, GUI only.
3. Index once, grep forever: a nightly job writes one JSONL per bag (`temp/bag_index/<bag>.jsonl`, one line per message of the few chosen topics) so plain `grep` and `jq` answer most questions; effort s, no new UI.
4. Document the `ros2 bag` plus `mcap` CLI recipes that already answer the usual questions in `docs/`, and leave it there.

Recommendation: 1 if the use is from Sessions (Claude reads bags by path), 2 if the use is at the robot; 3 is the cheapest way to make both grep-able.

A question ticket carries this to the owner; the answer under its `## Answer` re-opens this Item.

## Progress

- 2026-11-01: TKT-25 option 2 as a CONSOLE view (ROS / UDP / CONSOLE) in the GUI, live and in replay, severity filter and search
