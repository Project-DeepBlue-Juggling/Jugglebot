---
title: "human_catch_replay bounds the bag read by its time window and fails loudly when it writes nothing (BUG-126)"
type: bugfix
date: 2026-11-20
status: complete
phase: "PWAY-11 tooling"
related_plan: "pattern-progression.md"
related_issues: []
files_changed:
  - tools/probes/human_catch_replay.py
  - tools/probes/human_throw_bench.py
  - tools/probes/tracker_bag_replay.py
  - tests/tracking/test_human_catch_replay.py
  - logbook/2026-11-20-human-catch-replay-time-window.md
  - logbook/INDEX.md
subsystem: [tooling, tracking]
tags: [probe, replay, bugfix]
---

## What and why

`human_catch_replay.py <bag> --t-start .. --t-end ..` on the 2.4 GB bag 2026-11-20_12-49-25 ran 47 min 45 s, printed nothing, wrote no file and exited 0. Three defects found by reading `main()`:

1. The window did not bound the read: `B.load_frames` decoded the whole bag and only then filtered; with `--frames-pkl` the window was ignored. The mcap reader takes start/end times, so `tracker_bag_replay._iter_bag` and `human_throw_bench.load_frames` now take an optional window (default None = unchanged whole-bag read and cache; a windowed read bypasses the cache both ways). A pickle with wall times is windowed by them; clock times on a pickle without wall times are an error, never silently ignored.
2. The stream cache and the report were keyed by the bag name only, so a run with a different window silently reused the previous window's stream (or overwrote its report). Both names now carry the window (`..._125320_125645_replay.md`).
3. No progress output before the long read and no failure path: it now prints the bag, window and frame count to stderr, and exits 1 with a message when no frames, no truth Throws, or no report file result.

The silent-exit itself could not be reproduced from the code (every normal path prints `stream: ...` before returning 0), so the 47-minute run most likely died outside Python (the box was at 3 GB free of 7 GB while the whole bag was held as frames) behind a wrapper that reported 0. Unconfirmed; the loud failure paths and the bounded read remove the exposure either way.

## Verification

- 2026-11-20, `pytest tests/tracking/test_human_catch_replay.py -q`: 8 passed (5 new: bounded read, default unbounded, pickle window with wall times, pickle without, nothing-to-report exits 1).
- 2026-11-20, `pytest tests/tracking tests/ros/test_ball_tracker_gate.py tests/ros/test_launch_nodes.py -q`: 130 passed. Importer grep: `grep -rln "human_throw_bench\|tracker_bag_replay"` (tests/tracking x3, tests/ros x2, tools/probes/tracker_fit_latency x3 which only import, none call `_iter_bag` or `load_frames`).
- 2026-11-20, `pytest tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_index.py tests/sim/test_docs_links.py tests/sim/test_no_legacy_item_ids.py -q`: 48 passed.
