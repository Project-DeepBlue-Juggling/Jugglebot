---
title: "BUG-23: test_hand_step_violation_not_sent flaked (assert 2 == 1) because the pump's step gate is a wall-clock velocity bound and the test slept on the real clock; the pump clock is now frozen in the two step-violation tests"
type: bugfix
date: 2026-11-01
status: complete
phase: "night-shift"
related_plan:
  - night-shift.md
files_changed:
  - tests/ros/test_teensy_bridge_node_setpoint.py
  - logbook/INDEX.md
  - map/bug-test-hand-step-violation-not-sent-flakes-under-the-full-gate.md
  - docs/project-map.md
subsystem:
  - tests
  - teensy_link
tags:
  - night-shift
  - flaky-test
  - xdist
---

# BUG-23: the un-injected clock behind `assert 2 == 1`

**What.** Since 2026-09-17 `SetpointPump`'s per-step gate is a velocity bound: it rejects iff `|du| > v_max * max(dt, knot_dt)`, with `dt` read from `time.monotonic` against the last ACCEPTED frame (the pump takes an injectable `clock`). `test_hand_step_violation_not_sent` sent a baseline, did a real `time.sleep(0.02)`, then a hand jump of `max_step_hand_rev + 0.1` = 5.1 rev. The hand budget is 200 rev/s, so any stall of the baseline-to-jump interval past 25.5 ms (the 20 ms sleep plus four-worker load) widens the budget over 5.1 rev and the pump correctly ACCEPTS the frame: `n_after == n_before` reads `2 == 1`. The pump is right and the test was wrong: it depended on wall time while asserting a time-dependent gate. `test_step_violation_not_sent` (legs, 0.9 rev jump, 12 rev/s so a 75 ms stall) is the same defect with 3x more margin; fixed the same way.

**Fix (test only, node and pump untouched, no tolerance widened, no marker).** Both tests set `node._sp_pump._clock = lambda: 100.0` before the baseline. A frozen clock gives `dt = 0`, floored to one knot period, so the budget is exactly the nominal `max_step_*` and the jump is rejected at any load. The baseline-delivery `sleep(0.02)` became `_wait_until(... SETPOINT count == 1)`, removing the second race (a baseline datagram not yet counted by FakeTeensy). The 0.05 s sleep before the post-jump count stays: it only gives a wrongly-sent frame time to show, and `frames_rejected == 1` is the synchronous, load-proof assertion.

**Verification.**
- Reproduction (2026-11-01, original test with `time.sleep(0.02)` raised to `0.03`, `pytest -q -p no:randomly -k hand_step_violation_not_sent` from `tests/ros`): `E assert 2 == 1`, deterministic, 1 failed. Unmodified at idle: 1 passed.
- New form against the old defect (2026-11-01, `time.sleep(0.2)` injected between baseline and jump, the clock line removed): `assert 2 == 1`, 1 failed. With the clock line present and the same 0.2 s stall: 2 passed.
- Under load (2026-11-01, four busy-loop hogs started and stopped by me, 20 repetitions of `pytest -q -p no:randomly -k "test_hand_step_violation_not_sent or test_step_violation_not_sent"` from `tests/ros`): 20 of 20 `2 passed`.
- Scoped (2026-11-01, `pytest -q -p no:randomly -k "hand_step_violation or step_violation or pump_step_gates or link_down_blocks or enabled_sends or default_disabled"` from `tests/ros`): 6 passed in 4.49 s. `./run_tests.sh` not run (unit brief).
