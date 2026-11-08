---
title: "The ladder's mocap and trajectory-status freshness reads an injectable clock; the opening-REST tick load flake closed (BUG-65)"
type: bugfix
date: 2026-11-04
status: complete
phase: ""
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_skill_node.py
  - logbook/2026-11-04-jb165-observation-clock-injectable.md
  - logbook/INDEX.md
  - map/bug-test-the-tick-dispatches-the-opening-rest-bridge-flakes.md
  - docs/project-map.md
subsystem:
  - skill-stack
  - tests-tooling
tags:
  - load-flake
  - clock-injection
---

# BUG-65: the observation clock, the fourth twin

**What.** The BUG-61 full gate (Night Shift 2026-11-04) failed once on `test_the_tick_dispatches_the_opening_rest_bridge_through_the_mocked_client` (`assert 0 == 1`). Alone it passed 5 of 5, and its file passed 388 of 388. `SkillNode._observations` judged mocap (0.5 s) and trajectory-status (1.0 s) freshness on the raw `time.perf_counter()`, and `_start_pattern` (about 0.4 s idle) sits between the fixture's `_freshen` and the tick. A `/tmp` probe that ages `perf_counter` by 0.6 s before the tick reproduced it: `in_trajectory_mode` goes False and the tick ends `ABORTED_MODE_CHANGED` with no install.

**Fix.** One injectable `SkillNode._obs_clock` (production `time.perf_counter`, unchanged) stamps `_traj_status_mono`, `_mocap_mono` and `_last_install_mono` and is the `now` `_observations` judges them with. All three share one clock, so `plan_superseded`'s status-vs-install comparison stays in one domain. `_hand_lane_refused` keeps the raw perf clock: its `_sched_refused_mono` is stamped by the `/link_status` handler on that clock. The fixture `_node_with_client` freezes `_obs_clock`, as it does the hand, frame and commanded clocks. No window was widened, and production behaviour is unchanged.

**Verification.** 2026-11-04. The new `test_a_slow_test_run_cannot_age_the_status_before_the_first_tick` (a 1.1 s sleep stands in for the load) failed with `skill_node.py` stashed and passed with it (`pytest -q tests/ros/test_skill_node.py -k slow_test_run_cannot_age_the_status`: 1 failed, then 1 passed). Then `pytest -q tests/ros/test_skill_node.py tests/ros/test_skill_node_magazine.py tests/ros/test_skill_node_reload_only.py tests/ros/test_skill_node_resend_param.py tests/ros/test_skill_node_robot_frame.py` gave **425 passed in 87.96 s**. Full gate on 58fe9438 (`./run_tests.sh --full`, 2026-11-04): **8156 passed, 53 skipped, plus 8 serial passed**, `EXIT=0`; re-run on the audit tip before the merge.
