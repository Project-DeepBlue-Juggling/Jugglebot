---
title: "JB-64: the skill_node accept-check fixture test flaked on the frame check's clock, not the hand clock"
type: bugfix
date: 2026-10-28
status: resolved
phase: "JB-64 - skill_node fixture load flake"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - tests/ros/test_skill_node_robot_frame.py
subsystem:
  - skills
  - tests
tags:
  - load-flake
  - frame-check
---

## Summary

`test_a_slow_test_run_cannot_age_the_fixture_sample_past_the_accept_check` sleeps 0.6 s and then starts a columns Pattern. Under load it was refused `REJECTED_FRAME_OFFSET`, with the cup check passing. The 2026-10-15 fix froze only `_hand_clock`. `_node_with_client` seeds a second set of samples, the session-start frame check's Platform and commanded buffers (`_frame_ready`). Those were stamped and judged on the real `time.perf_counter()` against `_FRAME_CHECK_WINDOW_S` (1.0 s), which left the test 0.4 s of margin. Clock reads on the columns accept path:
- hand freshness: `_hand_clock`, already injected
- frame-check stamps and `now`: perf_counter, **not injected** (this bug)
- `trajectory/status` freshness: perf_counter (1.0 s), only consulted by the ladder's `levelled`/`in_trajectory_mode`, which does not refuse a columns start
- robot-frame age: perf_counter, logged and never refused

A sleep of 1.1 s with the fix in place still starts the Pattern, so nothing else on the path ages.

Fix: `SkillNode._frame_clock` (`time.perf_counter` in production, so behaviour is unchanged). It is now the only clock the frame-check group reads: both buffer stamps in `_on_mocap`/`_on_commanded_position` and the `now` passed in `_frame_check`. `_mocap_mono`/`_commanded_pos_mono` stay on perf_counter. The fixture freezes it when `frame=True`. `frame=False` tests keep the real clock and stamp their own samples, and `_frame_ready` stamps on `node._frame_clock()`. The test now sleeps past both windows and asserts no frame refusal, so it would fail without either injection. The stale-sample siblings still refuse.

## Verification

- Recipe used both times: run the single test 6 times in a loop with eight `python -c "while True: pass"` loaders on the 6 cores.
- 2026-10-28, before the fix: **0/6 pass**, each refused for a stale or missing Platform body.
- 2026-10-28, after the fix: **6/6 pass**.
- 2026-10-28, `pytest tests/ros/ -k skill_node -q -n 4 --dist loadfile`: **401 passed**.
- Gate: the pre-commit `./run_tests.sh` triple is in this entry's commit message.
