---
title: trajectory/timed_target now takes a relative lead_time_s instead of an absolute ROS-clock arrival_time
type: change
date: 2026-07-16
status: resolved
phase: "MVP trajectory bringup — Phase 5 (timed targets): operator-facing interface"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot_interfaces/srv/TimedTarget.srv
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/conftest.py
  - tests/hardware/session_phase5_timed.md
  - tests/hardware/mvp_bench_runbook.md
commits:
subsystem:
  - ros
tags:
  - interface
  - trajectory
---

# timed_target takes a relative lead_time_s now

## Summary

Operator-requested interface change ahead of the S5 sitting: the
`trajectory/timed_target` service field `builtin_interfaces/Time arrival_time`
(absolute ROS clock) is replaced by `float64 lead_time_s` (relative, measured
from service receipt). The node anchors the absolute arrival internally:
`arrival_perf = time.perf_counter() + lead_time_s` at handler entry — verified
to be the same clock domain `_plan_and_install_timed` and the emitter run on,
so no clock crossing remains anywhere in the service path.

## Why

An absolute ROS-time arrival is effectively untypeable in a manual
`ros2 service call` (the operator would have to read the clock, add a lead,
and format a Time message), and it forces every caller to solve caller↔node
clock agreement. A relative lead is what the S5 protocol actually reasons
about ("arrive 3 s from now"), needs no clock plumbing, and the hard-timing
semantics are unchanged: a too-tight lead still rejects loudly (`TOO_FAST`
with the genuinely achievable `min_duration_s`) — never silently late.

Design notes:

- **Non-positive leads flow through the single feasibility gate** rather than
  a duplicated handler check: `build_timed`'s lead floor rejects them
  `TOO_FAST` with the *achievable* minimum and publishes `target_feedback`.
  Only non-finite values (which sail past floor comparisons) are guarded at
  the handler (`UNREACHABLE`).
- The `/catch/dynamic_target` TOPIC keeps its perf-domain absolute
  `arrival_time` — machine-to-machine from catch_coordinator, correct as-is.
  `TargetFeedback.arrival_time` (the coordinator's correlation key) is also
  unchanged.
- `_ros_time_to_perf` and the 30 s clock-offset refresh plumbing are retained
  but now consumer-less on the service path; their docstrings say so.

## Verification

- `python -m pytest tests/ros/ -q` (run 2026-07-16): **905 passed in 54.02 s**
  (includes the two new reject-path tests:
  `test_timed_target_non_finite_lead_rejected`,
  `test_timed_target_non_positive_lead_rejected_too_fast`).
- Full-suite pre-commit triple: see the commit message (shared gate with the
  same-day lean-planning changes).

## Operator deployment

The interface package changed: **`colcon build --packages-select
jugglebot_interfaces jugglebot`** then a full relaunch (a plain jugglebot-only
rebuild is NOT enough). `session_phase5_timed.md` carries the banner and the
new `lead_time_s` call examples.

## Related

- `tests/hardware/session_phase5_timed.md` — updated S5 protocol of record.
- `logbook/2026-07-16-lean-planning-latency-and-boundary-step.md` — same-day
  trajectory-layer work sharing this commit.
