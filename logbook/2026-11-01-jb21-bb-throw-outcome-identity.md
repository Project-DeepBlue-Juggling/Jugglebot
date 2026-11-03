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

## Audit findings applied (2026-11-03)

The Night Shift audit left two behaviour-affecting findings; the owner decided to apply both (ticket JB-153, option 1).

- **Release window bound and clock read (WARNING).** `WINDOW_AFTER_S` is now 2.5 s (was 0.75 s), and `_fire_reload_throw` re-reads the ROS clock right after `wait_for_service` returns (`window_now`) and centres the window on that instant. Before, the window was centred on a clock read taken before the wait and the service round trip, so a round trip over ~0.8 s dropped a genuine verdict and a genuine NOT_SETTLED was never retried. The window is now `[t-0.25, t+2.5]` around `window_now + delay_s`; throws are whole beats (seconds) apart, so it still separates them.
- **No gap between the inflight flag and the window (NOTE).** `_reload_throw_inflight` is now set True in the same `_reload_lock` block that records `_bb_throw_release_window` (and sets `_bb_throw_dispatched`), so from `_on_bb_throw_outcome`'s view the window exists the moment the flag is True. Every end path already clears both together. The flag is also still set at the executor swaps, as before.
- The rebase onto main also broke two of the branch's own tests (the retry they referred to was retired by JB-25; the window width assert): `..._does_not_spend_the_retry` became `..._does_not_end_the_attempt` and the width assert follows the new bound.

Verification: new tests `test_jb21_window_opens_after_wait_for_service_not_before` and `test_jb21_window_exists_the_moment_inflight_is_true` (tests/ros/test_skill_node.py). 2026-11-03, `pytest -q -p no:cacheprovider tests/ros/test_skill_node.py -k jb21_window` on the pre-fix code: 2 failed (verdict dropped; `inflight` False in flight). After the fix: see the scoped run below.
2026-11-03, `pytest -q -p no:cacheprovider tests/ros/test_skill_node.py tests/ros/test_ball_butler_node.py tests/ros/test_throw_identity.py tests/sim/test_logbook_front_matter.py tests/sim/test_docs_links.py`: 477 passed in 101.92 s. The default gate is the merge gate's.
