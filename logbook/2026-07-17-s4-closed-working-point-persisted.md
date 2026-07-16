---
title: S4 closed — the working point (1000, 5000, 30000) + lean_gain 0.6 persisted to YAML, with the srv default that makes the gain actually reachable
type: change
date: 2026-07-17
status: resolved
phase: "MVP trajectory bringup — S4 limit ramp: CLOSED (step-5 persistence)"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.h
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot_interfaces/srv/GoToPose.srv
  - tests/ros/conftest.py
  - tests/ros/test_trajectory_node.py
  - tests/motion/test_chase_clamp.py
  - tests/motion/test_follower_chase.py
  - tests/motion/test_trajectory_follower.py
  - tests/motion/test_trajectory_planner_move.py
  - tests/motion/test_trajectory_shaping.py
  - tests/hardware/mvp_bench_runbook.md
  - tests/hardware/session_phase4_ramp.md
  - plans/active/mvp-trajectory-bringup.md
commits:
subsystem:
  - config
  - ros
tags:
  - trajectory
  - interface
---

# S4 closed: the working point ships

## Summary

Operator decision (2026-07-17, after the 2026-07-16 limits A/B and lean-gain
sweep): persist the S4 working point to YAML — `trajectory_op` session defaults
`(leg_vel, leg_acc, leg_jerk) = (1000, 5000, 30000)` (mm/s, mm/s², mm/s³) and
`lean_gain: 0.6`. This is the S4 ladder's step-5 closure; **S4 is marked PASSED
in the runbook.** The operator chose vel = 1000 (below the A/B's 2000) because
velocity was only 14–23 % utilized in the A/B — it costs nothing and shortens a
runaway command's runway.

## The catch the walkthrough found (why this is not just a YAML edit)

`GoToPose.srv`'s `lean_gain` field had **no default**, and an unset float64
reads 0.0 — which the node treats as an EXPLICIT "force lean OFF" (only a
*negative* value defers to the config gain). So flipping the YAML alone would
have made 0.6 reachable only by callers that explicitly send −1: the config
default would have been **silently unreachable** for every default-constructed
request. The srv's own comment documented this trap.

Fix shipped with the flip: **`float64 lean_gain -1.0`** — the srv field default
is now −1.0, so a default-constructed request defers to the config gain (0.6,
lean ON), while an explicit `0.0` still forces OFF (the A/B baseline arm).
Mirrored in the `tests/ros/conftest.py` mock, and pinned three ways in
`tests/ros/test_trajectory_node.py`:
`test_default_constructed_request_defers_to_shipped_lean_gain` (the
choreography tripwire), `test_go_to_pose_explicit_zero_lean_gain_forces_lean_off`
(the baseline arm), and `test_shipped_trajectory_defaults_are_the_s4_working_point`
(pins the four YAML values AND regex-pins the srv `-1.0` default so the gain
can never be silently orphaned again).

## Consequences the operator should know

- **A relaunch now reverts to the HOT working point**, not the old gentle
  (100, 400, 8000) — the runbook's "relaunch = revert everything" escape hatch
  changed meaning (runbook note updated). Cautious probing needs an explicit
  `set_limits` DOWN after launch.
- **Default service moves are now shaped** (lean 0.6): planning ~1.2–1.5 s per
  move until the shaped-planning efficiency work lands (exploration opened the
  same day), and shaped moves plan ~1.3–1.5× longer durations. `go_home`,
  `hold`, timed targets, and the SpaceMouse follower are unshaped by
  construction — unchanged.
- **The battery instrument's `--lean-gain` default is 0.0** (explicit), so its
  baseline runs stay unshaped unless the flag is passed — correct A/B
  semantics, but note the instrument does NOT pick up the new config default.
- **8 tests re-pinned** to the gentle (100, 400, 8000) limits their geometry
  was designed around — 7 across `tests/motion/` (`test_chase_clamp` ×2,
  `test_follower_chase` ×2, `test_trajectory_follower`,
  `test_trajectory_planner_move`, `test_trajectory_shaping`; via `from_config`
  overrides) and 1 in `tests/ros`
  (`test_pending_stop_on_infeasible_stop_then_retry_converges`, via
  `with_session_limits` — at the hot working point a graceful stop from 2 mm
  inside the boundary is feasible, so the infeasible-stop-retry path never
  engaged). All exercise MECHANISMS, not the shipped limits; each root cause
  was verified as limits-float (not lean) before pinning.
- **Deployment: `colcon build --packages-select jugglebot_interfaces jugglebot`
  + relaunch** (the srv changed again).

## Verification

- Scoped (run 2026-07-17): `pytest tests/motion/ -q` → **675 passed in
  142.05 s**; `pytest tests/ros/test_trajectory_node.py -q` → **118 passed in
  11.25 s**; `pytest tests/sim/ -q` → **890 passed, 1 xfailed in 412.04 s**
  (no lean-gain ripple into the diagnose metadata tests).
- Full suite (`pytest tests/ -q`, run 2026-07-17): **2828 passed, 1 xfailed in
  797.81 s** — the pre-commit gate (+2 = the new defer-choreography and
  shipped-defaults tripwires).
- The srv field-default syntax was verified against this box's actual Foxy
  parser (`rosidl_adapter.parser` → `default= -1.0`); the operator's
  `colcon build --packages-select jugglebot_interfaces jugglebot` is the final
  confirmation.

## Related

- `logbook/2026-07-16-lean-planning-latency-and-boundary-step.md` — the sweep
  + A/B data this decision rests on (Addendum).
- `logbook/2026-07-16-max-deviation-guard-tracking-lag.md` — the guard/envelope
  arc S4 ran inside.
- `tests/hardware/mvp_bench_runbook.md` — S4 marked PASSED; relaunch-semantics
  note updated.
- The shaped-planning efficiency exploration (same day) — the follow-up that
  makes the now-default shaped planning cheap.
