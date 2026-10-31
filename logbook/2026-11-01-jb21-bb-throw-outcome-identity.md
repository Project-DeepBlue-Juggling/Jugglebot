---
title: "bb/throw_outcome carries the throw's identity: a release_wall_s suffix, matched by skill_node (JB-21)"
type: bugfix
date: 2026-11-01
status: complete
phase: ""
files_changed:
  - ros_ws/src/jugglebot/jugglebot/throw_identity.py
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - tests/ros/test_throw_identity.py
  - tests/ros/test_ball_butler_node.py
  - tests/ros/test_skill_node.py
  - logbook/INDEX.md
  - map/bug-bb-throw-outcome-carries-no-throw-identity.md
  - docs/project-map.md
subsystem:
  - ball-butler
  - skill-stack
tags:
  - bb-throw-outcome
---

# bb/throw_outcome carries the throw's identity (JB-21)

## What and why

bb-self-cancel finding F2: the `bb/throw_outcome` String did not say which Throw it reported, so a late verdict from attempt N could end attempt N+1 (or spend its once-per-attempt `THROW_ABORTED_NOT_SETTLED` retry). `ball_butler_node._on_throw_result` now appends ` release_wall_s=<%.6f>` (the goal's absolute release instant on the ROS wall clock, which it computed for that throw) to the relayed message; a goal with no release (0) is relayed unchanged. The leading token and everything before the suffix are byte-for-byte as before, so every by-name consumer is untouched. One pure module, `throw_identity.py` (no ROS import, Python 3.8), holds `tag_message`, `parse_release_wall_s`, `release_window` and `verdict_matches`. The only consumer that gates on the topic is `skill_node._on_bb_throw_outcome` (the others: `jugglebot_launch.py` bag list, text in `executor.py`, docs, which only name the topic).

The identity the consumer can know is the release it asked for: `_fire_reload_throw` records `release_window(now_wall, delay_s)` = [now+delay-0.25, now+delay+0.75] s before its blocking `bb/throw_at_target` call (both nodes use the ROS clock, CLOCK_REALTIME; the producer stamps `release_wall_s` = its `now + delay - 44 ms` somewhere inside that call). The window is cleared wherever `_bb_throw_dispatched` is. A verdict with a suffix outside the window is logged at info and dropped before any state is touched (no end, no retry, `_bb_pending_release_mono` kept). No suffix, or no recorded window: today's path exactly.

## Walk-through (one Feed)

Attempt N fires: window W_N around T_N, BB publishes the announcement, the throw releases, `OK` verdict arrives (ignored as always). Attempt N+1 starts and fires at ~T_N + several s: window W_N+1 replaces W_N. A late `THROW_REJECTED_BAD_STATE release_wall_s=T_N` from N reaches `await_announcement` of N+1: T_N is seconds before W_N+1, so it is ignored and N+1 continues to its announcement; before the fix it ended N+1 as `REJECTED_BB(...)`. N+1's own verdict carries ~T_N+1 inside W_N+1 and ends the attempt as before. The same late message cannot consume N+1's NOT_SETTLED retry.

Tradeoff accepted: a window (not exact equality) because the consumer cannot know the producer's `now` or the 44 ms compensation; attempts are seconds apart (minimum lead 0.7 s plus flight), the window is 1 s wide. A NOT_SETTLED retry re-fires and replaces the window.

## Verification

- 2026-11-01, `pytest tests/ros/test_throw_identity.py tests/ros/test_ball_butler_node.py tests/ros/test_skill_node.py -q -p no:randomly`: 459 passed in 62.27 s.
- Same command with `ball_butler_node.py` and `skill_node.py` stashed to HEAD: the 7 new node tests fail (suffix absent / `_bb_throw_release_window` missing); the helper tests need only the new module.
- `./run_tests.sh` not run (unit brief forbids it); the owner's merge gate runs it.
