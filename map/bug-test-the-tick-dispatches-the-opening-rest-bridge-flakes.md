---
id: JB-165
kind: bug
title: 'test_the_tick_dispatches_the_opening_rest_bridge flakes under a loaded full gate: the mocap and status freshness clock is not frozen'
status: complete
created: '2026-11-04'
updated: '2026-11-03T18:29:24Z'
owners: []
priority: medium
effort: xs
hardware: none
owner: none
radius: local
diagnosis: confirmed
area: tests-tooling
blocked_by: []
links:
- logbook/2026-11-04-jb165-observation-clock-injectable.md
filed_by: night-shift-2026-11-04
---

Night Shift 2026-11-04: the JB-159 full gate failed once on `tests/ros/test_skill_node.py::test_the_tick_dispatches_the_opening_rest_bridge_through_the_mocked_client` (`assert 0 == 1`, no install). The test passes 5 of 5 alone and in its whole file (388 passed), and the nightly was GREEN.

## Mechanism (reproduced 2026-11-04)

- `SkillNode._observations` judges mocap (`_MOCAP_STALE_S` 0.5 s) and trajectory-status (`_TRAJ_STATUS_STALE_S` 1.0 s) freshness on the raw `time.perf_counter()`, and stamps both caches and `_last_install_mono` on it too.
- The fixture `_node_with_client` freezes the hand, frame and commanded clocks, but not this one. `_start_pattern` sits between `_freshen` and the tick and takes about 0.4 s on an idle box.
- A probe that ages `perf_counter` by 0.6 s before the tick reproduces the failure: `levelled` and `in_trajectory_mode` go False and the tick ends `ABORTED_MODE_CHANGED` with nothing dispatched. At 0.0 s it dispatches.

## Fix

The JB-141 shape: one injectable `_obs_clock` (production `time.perf_counter`, unchanged) stamps and judges these three caches, and the fixture freezes it. Production behaviour is unchanged.
