---
id: BUG-108
kind: bug
title: test_TI1 in test_install_segment goes stale under box load and fails the gate on unrelated branches
status: complete
created: '2026-11-16'
updated: '2026-11-16T17:54:21Z'
owners: []
priority: medium
effort: s
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- logbook/2026-11-17-test-install-segment-stale-clock.md
filed_by: claude
---

`tests/ros/test_install_segment.py::test_TI1_seven_channel_frames_reach_the_wire_and_the_flags_fall` failed the full gate twice on the Night Shift of 2026-11-16 with `robot_state telemetry stale — cannot install a segment` (test line ~1524): on night-shift/2026-11-16-BUG-96 and on night-shift/2026-11-16-BUG-104.

BUG-104 changes only the GUI's `commands.js`, so the branch cannot be the cause. Both suites ran while the box carried extra test load (the orchestrator was re-running that file in parallel).

## Findings

- Alone it passed 3 of 3 (2026-11-16, `night_shift_nonet.sh python -m pytest -q tests/ros/test_install_segment.py -k test_TI1_seven_channel`).
- As a whole file under load it failed 2 of 4 on the BUG-96 branch, once on TI1 and once on `test_a_fresh_throw_installs_through_the_real_worker_process`, and passed 2 of 2 on main. The runs took 44 to 156 s, depending on the load.
- It passed in the BUG-91, BUG-97 and BUG-98 suites tonight and in the 04:00 nightly.

## Wanted

Find which wall-clock freshness window the test leaves to real time (the robot_state staleness check against the node clock), and pin it with the test's own clock so box load cannot make it stale. A gate that fails on load turns a clean branch into a HOLD.
