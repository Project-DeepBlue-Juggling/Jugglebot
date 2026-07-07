---
title: MVP Phase 5 — Timed target states (code-complete; hardware deferred)
type: feature
date: 2026-07-08
status: resolved
phase: "5"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot_interfaces/srv/TimedTarget.srv
  - ros_ws/src/jugglebot_interfaces/msg/TargetFeedback.msg
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - tests/motion/test_trajectory_planner_timed.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_catch_coordinator_node.py
  - tests/ros/conftest.py
  - tests/hardware/session_phase5_timed.md
commits:
  - 62e9ea7
  - d40da26
subsystem: motion
tags: [mvp-trajectory-bringup, trajectory, timed-target, catch, supersede, feasibility]
---

## Summary

Phase 5 gives the MVP trajectory generator its **timed target** surface (MVP
goal 3): reach an arbitrary pose+orientation at a nominal velocity at an **absolute
arrival time**, with a too-tight lead **loudly rejected** (never silently slowed to
arrive late). One new pure-Python constructor `planner.build_timed`; one new ROS
service `trajectory/timed_target` (`TimedTarget.srv`); the `catch/dynamic_target`
CATCH-mode path wired through the SAME `build_timed` (+ a reach-freeze window); a new
`trajectory/target_feedback` topic (`TargetFeedback.msg`) that `catch_coordinator_node`
consumes in place of the dormant MPC process's ZMQ :5559 feedback (feasibility
blacklist semantics preserved); and one ROS-clock→perf_counter conversion point in
the node.

The load-bearing decision this phase owns: **the supersede design.** A mid-plan
superseding timed target must install a C2 replan mid-move — the exact TOCTOU class
the Phase-2 install-continuity guard closed on the ~377 ms analytic gate. `build_timed`
is gated by the **fast** `validate_follow` (~1.6 ms/segment), so a supersede installs
within ~2 ms of its seed sample (drift ≪ the pump/firmware step gates), enabling a
clean C2 replan without a `BUSY` restriction. Software complete; the ±25 ms mocap
timed-move battery + supersede demo are **deferred to an operator bench session**.

## Motivation

Goals 1–2 (waypoint moves) and the SpaceMouse follower (goal 2/streaming) landed in
Phases 1–4. Goal 3 is arbitrary `((position),(orientation),arrival_time)` requests —
the primitive the reload catch (Phase 6/7) is built on (`catch/dynamic_target` is a
timed target with zero arrival velocity). The plan requires the catch path to route
through the *same* `build_timed` so nothing bypasses the single feasibility gate, and
requires the Phase-2 temporary `BUSY` restriction on mid-plan supersede to be lifted
(the hardware test demands a mid-plan superseding timed target with a C2 replan).

## Design

### `planner.build_timed` — a fixed-lead reach, always rest-terminating

`build_timed(state0, target_pose, target_twist, duration_s, limits, geom, *,
hold_after=True, neutral_pose=None)`:

- Builds a **reach** quintic from the seed `(pose, twist, accel)` to
  `(target_pose, target_twist, zero-accel)` over the **fixed** lead `duration_s`.
  The segment END is the timing-accuracy-critical knot: the arrival pose error is
  zero by construction, and the emitted knot nearest `t_arrival` is within one 25 ms
  knot.
- The lead is **never stretched.** Unlike `build_move` (which finds the minimal
  feasible duration), a timed arrival is a hard constraint — a too-tight lead is
  rejected `TOO_FAST` with the minimal feasible lead in `min_duration_s`
  (`_min_feasible_timed` stretches the reach with `v1` fixed until the fast gate
  passes). A spatial failure (`WORKSPACE`/`UNREACHABLE`) is re-raised as-is (a longer
  lead cannot reach an out-of-stroke target).
- **Rest-termination is a safety invariant.** The implicit terminal hold snaps twist
  to zero, so a final segment with a nonzero end velocity would be a velocity
  discontinuity ⇒ unbounded leg jerk ⇒ dangerous hardware jerk. So when the arrival
  velocity is nonzero, a decel-to-rest continuation (the audited `build_graceful_stop`
  primitive, C2-joined: v0 = v1, a0 = 0 = the reach's a1) is appended.
- `hold_after`: `True` holds at the target after arriving (the catch path); `False`
  profiles back to `neutral_pose` after arriving and holds there (a "reach out, then
  return" one-shot). Both are rest-terminating & C2.
- **Gated by the fast `validate_follow`**, not the ~377 ms analytic `validate` — the
  reason the supersede works (below). Timed plans are never lean-shaped, so
  `validate_follow` (shaping-blind, it rejects a `_ShapedPlan` loudly) is exactly
  right. The assembled multi-segment plan is re-gated as a whole (the single-gate
  contract: the emitter runs the assembled object).

### `trajectory_node` — service, catch path, feedback, one clock crossing

- `trajectory/timed_target` (`TimedTarget.srv`, TRAJECTORY mode only, else
  `WRONG_MODE`): converts the ROS-clock `arrival_time` → perf via the single
  `_ros_time_to_perf` point, computes `lead = arrival_perf − seed_mono`, and installs
  with `t0 = seed_mono` so the reach end lands at wall time `seed_mono + lead =
  arrival_perf`. **No BUSY restriction** — supersede is the point.
- `catch/dynamic_target` (CATCH mode): `arrival_time` is ALREADY perf-domain (the
  coordinator converts landing_time→perf; perf is system-wide `CLOCK_MONOTONIC` on
  Linux, comparable across the two node processes), so no clock crossing. Each update
  supersedes the prior via a C2 replan — EXCEPT inside the **reach-freeze window**
  (within `JB_TRAJ_CATCH_REACH_FREEZE_S = 0.3 s` of the committed arrival), where late
  target jitter is ignored so the platform holds its committed reach into the catch (a
  parked, non-jittering rim seats the ball; the reload design). Leaving CATCH / a fresh
  seed clears the freeze.
- `trajectory/target_feedback` (`TargetFeedback.msg`: accepted/code/reason/
  arrival_time/source): published on every timed/catch accept+reject.
  `catch_coordinator_node` swaps its dormant ZMQ :5559 `TargetFeedbackSub` for a
  subscription to this topic; the correlation (by `arrival_time`) + feasibility
  blacklist (`report_rejection_with_position`) are unchanged.
- An **install-continuity guard** (same 0.06 rev bound as `go_to_pose`) backstops the
  service→emitter race: re-sample the live state before install, reject `STALE_STATE`
  if the plan's t=0 pose drifted past the bound. With the fast gate the window is
  ~2 ms so a normal supersede passes; a pathological stall is rejected rather than
  jumped.

### The catch z-convention (flagged for Phase-7 verification)

`catch/dynamic_target.target_pos.z` is an **offset from the active pose** (0 = active
— the MPC target convention, confirmed by `controller/mpc.py`'s z bounds relative to
active). The trajectory pose convention is **STOW-relative** (170 = active). So a catch
target's z is lifted by `JB_OP_DEFAULT_ACTIVE_Z_MM` (170); x/y are 0-at-active in both
and map directly. The velocity is frame-offset-invariant, so no z adjustment. **This
frame mapping is to be RE-VERIFIED on hardware before any ball flies** (mirrors the
plan's Phase-7a QTM-frame verification); the feasibility gate loudly rejects an
out-of-stroke z meanwhile, so an error is loud, not dangerous.

## Implementation

New: `planner.build_timed` + `_min_feasible_timed`; `TimedTarget.srv`,
`TargetFeedback.msg` (+ CMakeLists rows); `trajectory_node` clock offset +
`_ros_time_to_perf` + `_plan_and_install_timed` + `_catch_target_from_msg` +
`_on_dynamic_target` + `_svc_timed_target` + `_install_continuity_ok` +
`_publish_target_feedback`, the `catch/dynamic_target` subscription, the
`trajectory/timed_target` service, the `trajectory/target_feedback` publisher, and the
30 s clock-refresh timer; `catch_coordinator_node` feedback-topic swap;
`/trajectory/target_feedback` added to the launch rosbag list; conftest mocks
(`DynamicTargetCommand`, `TargetFeedback`, `BallStateArray`, `TimedTarget` +
`_RosTimeMsg`). Tests: `tests/motion/test_trajectory_planner_timed.py` (12),
`tests/ros/test_trajectory_node.py` (+17), `tests/ros/test_catch_coordinator_node.py`
(7, new). Operator protocol `tests/hardware/session_phase5_timed.md`. **No
`hardware_config.yaml`/codegen change** — the Phase-5 config constants
(`JB_TRAJ_MIN_TIMED_LEAD_S`, `JB_TRAJ_CATCH_REACH_FREEZE_S`) already landed in Phase 1.

## Verification

- Full suite: `pytest tests/ -q` (2026-07-08) = **2164 passed, 1 xfailed in 535.82 s**
  (baseline 2128 passed / 1 xfailed at Phase-4 audit-fix `1c0f9c1`; net **+36** = the
  new tests only: 12 planner-timed + 17 node + 7 catch-coordinator; no regressions).
- ci-deep (deferred from Phase 2): `pytest tests/ -q --hypothesis-profile=ci-deep`
  (2026-07-08) = **<PENDING>**.
- colcon: `colcon build --packages-select jugglebot_interfaces jugglebot`
  (2026-07-08) = **2 packages finished, 0 errors** (`TimedTarget.srv`,
  `TargetFeedback.msg`).
- Production-in-the-loop invariant re-asserted on timed plans (incl. a
  nonzero-velocity arrival + decel): every emitted knot accepted by a real
  `SetpointPump` (`test_every_timed_knot_pump_accepted`,
  `test_timed_target_emitted_frames_pump_accepted`).

## Discussion

### Fork (mine, load-bearing) — the supersede design: fast-gate, not guard+retry

The inherited context named four options for the mid-plan-supersede TOCTOU (predicted
install-time seed / reuse the fast `validate_follow` gate / guard+retry / hybrid) and
asked me to own the choice with its failure modes. **I chose the fast-gate option.**

Concrete failure mode each alternative fails to prevent:
- **guard+retry alone** (the `go_to_pose` Phase-2 approach): the analytic gate runs
  ~377 ms while the emitter streams the OLD plan; the seed is 377 ms stale at install,
  so a mid-move supersede's live state has drifted far past the guard bound → the guard
  **rejects every supersede while moving**. That directly defeats the Phase-5 hardware
  requirement (a mid-plan superseding timed target with a C2 replan). Unusable for
  supersede.
- **predicted install-time seed**: seeds the reach where the plan is *predicted* to be
  after the gate runs. But the gate time varies (GC, scheduler), so a misprediction
  reintroduces the u0 jump — it trades a measurable drift for an unmeasurable one.
- **fast `validate_follow` (~1.6 ms/segment)**: the seed sampled at entry is still
  fresh at install (~2 ms later); the drift over 2 ms at the 280 mm/s ceiling is
  ~0.6 mm ≪ the 0.06 rev guard bound, so the install-continuity guard passes and the
  reach starts C2 off the live state. The plan installs with `t0 = seed_mono`, so the
  arrival lands at `arrival_perf` and the replan is bumpless. This is the one option
  that makes the supersede both *accepted* and *safe*.

Why `validate_follow` is not a bypass of the single-gate contract: it lives in the one
`feasibility` module, returns the same `FeasibilityReport`, uses the same codes/limits/
stroke logic, its knot-step-bound is bit-identical to `validate` (so pump-acceptance is
preserved), and the reach is reached only through `planner`. It is shaping-blind and
rejects a `_ShapedPlan` loudly — timed plans are never shaped, so that is a guard, not
a limitation. `go_to_pose` **keeps** its `BUSY` guard (it uses the analytic gate for
shaped plans); only the fast-gated timed path lifts `BUSY`. That asymmetry is
deliberate and tested (`test_go_to_pose_still_busy_mid_move`) — surfacing a
supersede-capable path without silently changing the shaped path's behaviour.

### Fork (mine) — `hold_after` semantics: hold-at-target vs return-to-neutral

The plan pins `bool hold_after` but not its exact meaning. I chose: **True → hold at
the target; False → after arriving (and coming to rest), profile back to neutral and
hold**. Ruled out: (a) *fly-through* (False = end at the reach with the arrival
velocity, caller supersedes) — an unsuperseded fly-through leaves a nonzero end
velocity that the implicit terminal hold snaps to zero = unbounded jerk = dangerous;
rejected because it can produce an unsafe plan with no wired successor. (b) A no-op
flag — poor form. The chosen semantics is always rest-terminating, always C2, gives the
flag a clear operator meaning ("hold after arriving" vs "come home"), and matches the
catch path (True = hold quiescent at the catch pose). Fly-through/chaining is deferred
to Phase 8+ (self-toss), where a real successor exists.

### Fork (mine) — rest-termination is forced, not optional

Whatever `hold_after`/arrival-velocity is requested, `build_timed` NEVER returns a plan
whose final segment ends with a nonzero velocity. This is the single most safety-
relevant choice in the file: the `TrajectoryPlan` implicit terminal hold returns zero
twist, so a nonzero-velocity final segment is a step discontinuity in commanded leg
velocity — exactly the "dangerous jerky hardware movement" `CLAUDE.md` warns about. A
nonzero arrival velocity therefore ALWAYS gets a decel-to-rest continuation, gated as
part of the assembled plan.

### Why the catch path reuses `build_timed` (not a parallel catch builder yet)

The plan's single-most-load-bearing convention is that everything that moves the
platform flows through `planner` → the gate. The catch reach IS a timed target (arrive
at the catch pose at the landing time, at zero velocity), so it uses `build_timed`
directly + a node-level reach-freeze. The richer `build_catch` (tilt-through-seat,
quiescent settle hold) is Phase 6 — this phase deliberately ships only the reach +
freeze so the timing surface is exercised end-to-end without pulling Phase-6 geometry
forward.

### Clock domains — one crossing, and why the catch path needs none

`perf_counter` is `CLOCK_MONOTONIC` on Linux (system-wide, comparable across the
`trajectory_node` and `catch_coordinator_node` processes and with the emitter thread),
so the catch path — whose `arrival_time` the coordinator already converted to perf —
needs no crossing here. The single crossing is the `timed_target` service's ROS-clock
arrival, converted at `_ros_time_to_perf` (offset = median(perf − ros), refreshed every
30 s, mirroring the coordinator). Keeping exactly one crossing point is the plan's
Architecture requirement and prevents the mistimed-catch class of bugs.

## Open questions

1. **Catch z-convention (Phase 7).** The `+170` active-z lift mapping MPC-offset z →
   trajectory STOW-relative z is by best-supported inference (mpc.py z bounds + the
   coordinator's `centroid − initial_height`), NOT hardware-verified. Verify with an
   aim-only / static-hold check before any ball flies (mirrors Phase-7a's QTM-frame
   task). The feasibility gate makes any error loud (WORKSPACE), not dangerous.
2. **go_to_pose supersede.** Left on `BUSY` (analytic gate for shaped plans). A future
   option: route an *unshaped* (gain 0) go_to_pose supersede through `validate_follow`
   too. Deferred — not a Phase-5 requirement.
3. **`_min_feasible_timed` convergence on a fast moving seed.** Bounded to
   `_MAX_FOLLOW_ITERS` (6) like the follower; a moving seed breaks exact 1/Tⁿ scaling
   so it returns a best-effort minimum. Fine for advertising a retry lead; not a
   guaranteed-tight minimum.

## Related

- Plan: `plans/active/mvp-trajectory-bringup.md` § Phase 5 (+ Architecture: the command
  seam, the feasibility gate, `trajectory_node`).
- Predecessors: `logbook/2026-07-08-mvp-phase3-spacemouse-streaming.md` (the fast
  `validate_follow` gate + `build_follow`/`build_graceful_stop` this phase reuses),
  `logbook/2026-07-07-mvp-phase2-waypoint-moves.md` (the install-continuity guard + the
  `string`-code service convention + the temporary `BUSY` restriction now lifted for
  timed targets).
- Operator session: `tests/hardware/session_phase5_timed.md`.
