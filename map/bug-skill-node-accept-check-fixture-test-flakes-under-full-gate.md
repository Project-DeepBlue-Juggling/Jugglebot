---
id: JB-64
kind: bug
title: skill_node accept-check fixture test flakes under full-gate load
status: complete
created: '2026-10-26'
updated: '2026-10-28T00:46:54Z'
owners: []
priority: medium
blocked_by: []
links:
- tests/ros/test_skill_node.py
filed_by: claude
---

## Symptom

`tests/ros/test_skill_node.py::test_a_slow_test_run_cannot_age_the_fixture_sample_past_the_accept_check` failed in the full gate twice on 2026-10-26, on trees whose diff touched nothing under `ros_ws/src` (map, mapkit and GUI-static work only):
- `./run_tests.sh` at e66cd312: 1 failed, 7334 passed (parallel phase).
- `./run_tests.sh` at e139969a: 1 failed, 7360 passed (parallel phase).
Run alone (`pytest "tests/ros/test_skill_node.py::test_a_slow_test_run_cannot_age_the_fixture_sample_past_the_accept_check" -q`) it passed 3 of 3. Other full gates the same day passed.

## Notes

The test sleeps `_HAND_STATE_STALE_S + 0.1` and expects the clock injection (docstring: probe 2026-10-15) to keep the fixture's seated sample fresh at `_start_pattern`. Under parallel load something still ages it, so the injection does not cover every clock the accept path reads. This is the hardware-safety surface: do not demote it to `nightly` or widen the tolerance; find the clock that is not injected.

## Progress

- 2026-10-28: Unit E: reproduced 0/6 under 8 loaders; refusal is the frame check (perf_counter, 1.0 s window), not the hand clock
- 2026-10-28: Frame check clock injected (SkillNode._frame_clock, frozen by the fixture); 0/6 to 6/6 under eight loaders
