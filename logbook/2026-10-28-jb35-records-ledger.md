---
title: "JB-35 records ledger: docs/records.md, written live by skill_node at each Attempt's end, shown in a GUI panel"
type: feature
date: 2026-10-28
status: in-progress
phase: "inbox-wall"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/records.py
  - docs/records.md
  - mkdocs.yml
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/gui/gui_server.py
  - ros_ws/gui/js/records-model.js
  - ros_ws/gui/js/records-panel.js
  - ros_ws/gui/css/records-panel.css
  - ros_ws/gui/css/replay.css
  - ros_ws/gui/index.html
  - ros_ws/gui/js/main.js
  - ros_ws/gui/js/state-minimap.js
  - ros_ws/gui/js/juggle-panel.js
  - tests/motion/test_records.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_gui_records_api.py
  - tests/ros/test_gui_records_panel.py
  - tests/ros/js/records_panel_harness.js
subsystem:
  - gui
  - skills
tags:
  - inbox-wall
  - records
---

## Summary

JB-35 built to the owner's 2026-10-28 design. A record is the most Catches in one
continuous Attempt on the robot, per Pattern, beaten only by strictly more Catches; each
beaten record appends a row, so `docs/records.md` is a history, newest last, grouped by
ball count then Pattern id, each heading marked `<!-- pattern: <id> balls: <n> -->`.

- `jugglebot/records.py` (pure, py3.8) renders and parses the whole file (round trip
  byte-identical), `apply_attempt`, `rebuild` from the seed CSV
  (`when_local,pattern,throws,catches,end_code,source`; sim rows refused), `to_json`, the
  CLI, and the end-code categories (completed / stopped / drop / compute / other, map in
  its docstring; literals pinned against `executor.py`). Writes are atomic and only on a
  beaten record; a malformed file raises and is never overwritten.
- `skill_node` parameter `records_path` (default `''` = off); `jugglebot_launch.py` sets
  it to the repo's `docs/records.md`. At the end of each Attempt (Feeds excluded) the node
  offers the Attempt after the Result is built, WARNs on failure, and adds `record_beaten`
  / `record_catches` to the `skills/attempt` juggle_end event. A beaten record dirties the
  live checkout until the Sitting's close-out commit (owner-accepted).
- `GET /api/records` in `gui_server.py` (module loaded by path, server stays ROS-free);
  a top-left panel to the right of the collapsed minimap, refetched on juggle_end and
  flashed on `record_beaten`, hidden in replay; one "Records: <Pattern>" View toggle per
  Pattern, kept in `localStorage`.

Ball counts are the REAL balls: only `columns` is 2; the phantom-ball columns halves and
the fed oval (one Ball Butler ball) are 1. The file ships with every heading and no rows;
the seed CSV lands in a follow-up commit. Reaches the robot after a colcon build.

## Verification

Gate, 2026-10-28, `./run_tests.sh`: **7440 passed, 13 skipped in 335 s, serial tail 5 passed, exit 0**.
The first run surfaced two contract tests the new code had to meet, both fixed: the GUI
wall-clock marker on the panel's flash timer, and `importlib` added to the replay
server's stdlib allowlist (the server loads `records.py` by path). New tests:
`tests/motion/test_records.py` (41), `tests/ros/test_gui_records_api.py` (5),
`tests/ros/test_gui_records_panel.py` (8, node harness), 4 in `test_skill_node.py`.
Panel screenshot from the worktree GUI with a fake ledger:
`temp/screenshots/jb35/records_panel_1600.png` (gitignored).

## Seed (main session, 2026-10-28)

A Sonnet agent mined every launch.log from 2026-09-13 to 2026-10-28 09:56 into
`logbook/artifacts/2026-10-28-records-seed-attempts.csv` (588 Attempts, chronological; 406 from
the skill node summary line that exists from 2026-09-29, 182 reconstructed from per-throw
OUTCOME lines between pre-level markers for the earlier logs, so those are approximate and say
so in their Note; 100 zero-throw refusals and the bare Feed moves dropped; the first Sitting of
2026-09-09..12 left no attempt lines at all). `python -m jugglebot.records rebuild --attempts
logbook/artifacts/2026-10-28-records-seed-attempts.csv --out docs/records.md` wrote the file:
38 record rows. Records after the seed: columns 47/48 (2026-10-22, drop), oval_1ball 35/35
(2026-10-27), oval_1ball_fed 35/35 (2026-10-28), self_toss 24/25 and hop 24/28 (reconstructed,
2026-09-22 / 09-29), columns_1ball 10/10 (2026-10-04), columns_1ball_fed 4/5 (2026-10-13). The
R5 gate night reads 31/32 here because the node tally excludes the feed catch and the Stop
throw (the logbook says 32 consecutive). `seed_note()` carries the provenance into the Note
column (test added). Verification: 2026-10-28, `pytest tests/motion/test_records.py -q`:
**42 passed in 0.43 s**; full gate on the rebased tree in this commit message.
