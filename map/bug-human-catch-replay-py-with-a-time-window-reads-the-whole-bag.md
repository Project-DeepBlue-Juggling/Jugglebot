---
id: BUG-126
kind: bug
title: human_catch_replay.py with a time window reads the whole bag, then exits 0 with no output
status: complete
created: '2026-11-20'
updated: '2026-11-20T07:07:53Z'
owners: []
priority: low
effort: s
hardware: none
owner: none
radius: local
diagnosis: hypothesis
area: tests-tooling
blocked_by: []
links:
- logbook/2026-11-20-sitting-debrief.md
- idea-jp-17-three-ball-cascade-with-the-owner-as-the-second-hand
- logbook/2026-11-20-human-catch-replay-time-window.md
filed_by: claude
---

Debrief of 2026-11-20: `python tools/probes/human_catch_replay.py <bag> --t-start .. --t-end ..` on the 2.4 GB bag `2026-11-20_12-49-25` ran 47 min 45 s, printed nothing, wrote no file and exited 0. Slicing `/mocap_data` for the window first and passing `--frames-pkl` took under 4 minutes and produced the report. The time window should bound the bag read, and a run that writes nothing should not exit 0.

## Progress

- 2026-11-20: Fixed on main (43e88504): the time window bounds the bag read and applies to a frames pickle; an empty run exits 1; the stream cache is keyed by window. The 47-minute silent exit was not reproduced (likely killed for memory behind a wrapper). The rerun on the bag took 391 s.
