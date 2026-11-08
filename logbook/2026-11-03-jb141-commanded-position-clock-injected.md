---
title: "The commanded-position freshness reads an injectable clock; the `self_toss` load flake closed (BUG-53)"
type: bugfix
date: 2026-11-03
status: complete
phase: ""
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - logbook/INDEX.md
  - map/bug-test-self-toss-refuse-commanded-position-stale-under-a.md
  - docs/project-map.md
subsystem:
  - skill-stack
  - tests-tooling
tags:
  - load-flake
  - clock-injection
---

# The commanded-position freshness reads an injectable clock (BUG-53)

## What and why

In the Night Shift 2026-11-02 full gate, two `self_toss` tests in `tests/ros/test_skill_node.py` refused with `trajectory/commanded_position is stale` while a MuJoCo unit loaded the box. The diagnosis is confirmed by reading the code, not by inference. `_prelevel_ready` stamped `_commanded_pos_mono` on the real `time.perf_counter()`. `_prelevel` then judged that stamp on the same real clock against `_TRAJ_STATUS_STALE_S` (1.0 s). Any second of load between the fixture and `_start_pattern` refused the goal.

This is the third member of the class, after the hand-state clock (2026-10-15) and the frame-check clock (BUG-14, 2026-10-28), and the fix has the same shape:

- `SkillNode._commanded_clock` (`time.perf_counter` in production, so behaviour is unchanged) is now the only clock the family reads. It stamps the sample in `_on_commanded_position` and supplies ``now`` to both readers of `_live_commanded_position`: `_prelevel` and the magazine start.
- `_node_with_client` freezes it, and `_prelevel_ready` stamps on it.
- No tolerance was widened.

No motion, timing or wire path changes. The production clock and the 1.0 s window are the same objects as before; only where the clock is looked up moved.

## Verification

- Both new tests (`test_commanded_position_freshness_reads_the_injected_clock`, and `test_a_slow_test_run_cannot_age_the_prelevel_sample` with its 1.1 s sleep standing in for load) failed on the pre-change tree on 2026-11-03: `pytest -q tests/ros/test_skill_node.py -k "injected_clock or cannot_age_the_prelevel"` with the production change and the fixture freeze reverted gave 2 failed. The same command on the change gave 2 passed.
- 2026-11-03, `pytest -q -n 4 --dist loadfile tests/ros/test_skill_node.py tests/ros/test_skill_node_magazine.py tests/ros/test_skill_node_reload_only.py tests/ros/test_skill_node_robot_frame.py tests/ros/test_skill_node_resend_param.py` (every test file that uses `_node_with_client` or `_prelevel_ready`): **418 passed in 71.55 s**.
- This entry is committed before the full gate (`./run_tests.sh --full`, the Night Shift merge gate) runs on the branch tip, so it cannot carry that run's count. The gate's date, command and result are recorded in the Night Shift 2026-11-03 report.
