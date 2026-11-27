---
title: "human_catch_replay: a transport delay from the event's frame stamp to the policy (PWAY-11 P3 readiness)"
type: feature
date: 2026-11-27
status: complete
phase: "PWAY-11 tooling"
related_plan: "human-partner-juggling.md"
related_issues: []
files_changed:
  - tools/probes/human_catch_replay.py
  - tools/probes/README.md
  - tests/tracking/test_human_catch_replay.py
  - logbook/2026-11-27-human-catch-replay-transport-delay.md
subsystem: [tooling, tracking]
tags: [human_catch, replay, latency, PWAY-11]
---

## What and why

`human_catch_replay.py` let the policy see each Human Throw event at its frame stamp: zero transport delay, which no live node has. The 2026-11-27 lob bag recorded the live `/human_throws` (523 events): the recorder received them a median 43.8 ms (p90 50.8, max 85.4) after `header.stamp`, against 8.5 ms (p90 15.5) for `/mocap_data`, so about 35 ms is spent inside the Tracker's callback or its queue (unchecked which). At the plan window (0.62 s) the node therefore plans from a revision that much older, and near 0.6 s of lead the Fit error falls 1 to 2 mm per 10 ms.

New `--transport-ms MS` (default 0, so earlier reports reproduce): `delay_events` moves each event's `stamp` later and leaves the Fit untouched; the cache stays undelayed and a delayed run writes `<stem>_trNNms_replay.md` so it never overwrites the zero-delay report. Plus the probe's missing README row. No live code changes; nothing changes on the robot.

## Result on the 2026-11-27 lobs (Robot frame, 13:50:30 to 13:54:30)

| run | lobs in reach | planned (all lobs) | planned Cup to crossing p50 / p90 | in reach and inside 35 mm / 20 ms |
|---|---|---|---|---|
| 0 ms (as before) | 14 | 16 | 24 / 48 mm | 7 of 14 |
| `--transport-ms 44` | 14 | 15 | 31 / 52 mm | 6 of 14 |
| the bag's own live events at their receipt time (scratch script, not in the probe) | 14 | 12 | 34 / 48 mm | 5 of 14 |

So the number the 2026-11-27 debrief quoted (28 mm median, Hand frame, zero delay) is about 4 to 6 mm better than what the live node would have planned. The 35 mm / 20 ms "caught" is the probe's model, not a measured Catch.

## Verification

- (2026-11-27, `python -m pytest tests/tracking/test_human_catch_replay.py -q -p no:cacheprovider`) 11 passed. The two new tests fail with the probe reverted to `origin/main` (2 failed, 9 passed: `delay_events` missing), pass restored.
- (2026-11-27, `python -u tools/probes/human_catch_replay.py ~/Desktop/rosbags/2026-11-27_13-49-34 --t-start 13:50:30 --t-end 13:54:30 --frame both --transport-ms 44 --out-dir <scratch>`) rc 0; Robot frame 15 planned, 31 / 52 mm, 6 of 14 in reach inside 35 mm / 20 ms; Hand frame 14 planned, 32 / 52 mm, 5 of 12.
