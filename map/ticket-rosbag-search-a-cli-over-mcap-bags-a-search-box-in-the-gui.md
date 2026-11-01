---
id: JB-130
kind: ticket
title: 'Rosbag search: a CLI over mcap bags, a search box in the GUI replay, a nightly JSONL index for grep, or documented bag-tool recipes?'
status: complete
created: '2026-11-01'
updated: '2026-11-01T00:32:30Z'
owners: []
priority: low
effort: xs
hardware: none
owner: decision
radius: local
area: tests-tooling
blocked_by: []
links: []
parent: idea-rosbag-search
type: question
---

JB-36 asks for a mechanism to search rosbags from recordings for specific information (one line, low priority, owner decision). Night Shift 2026-11-01 drafted the shapes below into the Item's `## Options (2026-11-01)` section; the choice sets the effort (s to l) and whether it is a tool or a GUI feature.

## Options

1. A CLI, `tools/rosbag_search.py`: open an mcap bag read-only, filter by topic, time window and a field predicate (`--topic /skills/attempt --where "code==HAND_LIMIT_ACC"`), print matching messages one per line with their stamps. Offline, testable with a small fixture bag under `tests/`, effort s to m.
2. A search box in the GUI replay's digest view (the replay already indexes a bag per topic): type a topic and a substring, the trackbar jumps to hits. Effort m, GUI only.
3. Index once, grep forever: a nightly job writes one JSONL per bag (`temp/bag_index/<bag>.jsonl`, one line per message of the few chosen topics) so plain `grep` and `jq` answer most questions; effort s, no new UI.
4. Document the `ros2 bag` plus `mcap` CLI recipes that already answer the usual questions in `docs/`, and leave it there.

Recommendation: 1 if the use is from Sessions (Claude reads bags by path), 2 if the use is at the robot; 3 is the cheapest way to make both grep-able.

## What each answer unblocks

- 1 or 3: a Night Shift unit (hardware none, tests-tooling).
- 2: a GUI unit on the replay side.
- 4: a docs unit; the Item closes with the recipes.

## Answer

I like Option 2. I think the best location for this would be a third button to add to  "ROS | UDP", for "CONSOLE". Would this be possible to add to the live GUI as well? When in the CONSOLE viewer (which should be read-only), it'd be great to be able to filter based on severity level
