---
title: MVP Phase 7 — BB→Jugglebot reload action (software-complete; hardware deferred)
type: feature
date: 2026-07-08
status: resolved
phase: "7"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/ball_butler_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot_interfaces/action/Reload.action
  - ros_ws/src/jugglebot_interfaces/srv/BallButlerThrow.srv
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/src/jugglebot/setup.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - sim/hand/trajectory.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_ball_butler_node.py
  - tests/ros/test_reload_sequencer.py
  - tests/ros/test_reload_coordinator_node.py
  - tests/ros/test_reload_integration.py
  - tests/ros/conftest.py
  - tests/sim/test_hand_trajectory.py
  - tests/hardware/session_phase7_reload.md
commits:
  - 6107d06   # integration gap: catch/dynamic_target -> build_catch
  - 3889444   # Reload.action + BallButlerThrow point-target/aim-only
  - e2c5afe   # reload sequencer FSM + coordinator node + integration test
  - f3cca4c   # CATCH_VEL_RATIO 0.9 -> 0.6 + refreshed gate evidence
  - <pending> # docs (logbook + plan + INDEX)
subsystem: motion
tags: [mvp-trajectory-bringup, reload, catch, ball-butler, action, fsm, tilt-through-seat]
---

## Summary

Phase 7 lands the **software** for the BB→Jugglebot reload (MVP goal 4): Ball Butler
aims at Jugglebot's ACTIVE catch point and throws; Jugglebot (CATCH mode, hand armed by
the existing coordinator) tilts to receive and catches. Exposed as `jugglebot/reload`
(new `Reload.action`). Four pieces:

1. **Integration gap closed** — `trajectory_node`'s `catch/dynamic_target` path now
   builds a tilt-through-seat catch (`planner.build_catch`) instead of the Phase-5
   reach-only `build_timed` that parked the tilted rim at contact.
2. **`reload_sequencer.py`** — a pure-Python FSM (unit-testable, no ROS) that runs the
   preconditions → aim+throw → announcement → catch → confirm sequence with every loud
   reject and abort.
3. **`reload_coordinator_node.py`** — a thin ROS wrapper that orchestrates ONLY: it
   drives the FSM, calls two BB services on its behalf, and never actuates the robot or
   bypasses the feasibility gate.
4. **`BallButlerThrow` point-target extension** — `use_target_point` (aim at a
   caller-supplied world point, skip the QTM lookup) + `aim_only` (speed 0, the 7a
   frame-verification fast-path).

Plus the separable Phase-6 follow-up: `sim/hand/trajectory.py` `CATCH_VEL_RATIO`
0.9→0.6 (the config source of truth) with the reload-gate evidence refreshed.

Hardware is DEFERRED to the staged operator sessions 7a/7b/7c
(`tests/hardware/session_phase7_reload.md`).

## Motivation

Phase 6 sim-gated the catch trajectory; Phase 7 wires it into a hardware-runnable
action. The plan's inherited premise is load-bearing: the **existing** catch path
(mocap → `ball_tracker_node` correlates the announced ball → `catch_coordinator_node`
emits `catch/dynamic_target` and arms the hand) already catches smoothly on hardware.
So the reload coordinator adds no new motion path — it orchestrates the existing
subsystems and lets `trajectory_node`/`planner`/`feasibility.validate` own all platform
motion. The one real gap: the catch path still routed through the Phase-5 `build_timed`
(a reach that parked the tilted rim), so a hardware reload would have caught *without*
the tilt-through-seat the Phase-6 `build_catch` provides.

## Design

### Integration gap — `catch/dynamic_target` → `build_catch`

`_on_dynamic_target` now calls a new `_plan_and_install_catch` (mirroring
`_plan_and_install_timed`) that builds `planner.build_catch`: reach with translational
arrival velocity forced to zero → tilt-through-seat decay (a small residual tilt rate
carried through the seat, decayed to rest over 0.15 s) → literal quiescent hold. Same
fast `validate_follow` gate, same install-continuity guard, same C2-supersede and
pump-acceptance guarantees; the settle-bounded reach-freeze + `FROZEN` feedback
semantics are unchanged (they wrap the plan-and-install call).

**Where the receive tilt comes from (a decide+document fork — see Discussion).** The
hardware-validated `catch_coordinator.compute_catch_orientation` already computes the
tilt collinear with the ball's arrival velocity and sends it in
`DynamicTargetCommand.target_quat`; `_catch_target_from_msg` extracts it into the catch
pose's `(rx, ry)`. `build_catch` reads that pose tilt to aim the through-seat residual
rate. `target_vel` (always zero — a stationary catch) is unused; `build_catch` forces
translational arrival velocity to zero regardless.

### `reload_sequencer.py` — the pure FSM

Phases map to the `Reload` action feedback: CHECKING → AIMING → THROW_PENDING →
BALL_IN_FLIGHT → CATCHING → SETTLING. Loud rejects at CHECKING (WRONG_MODE,
BB_DISCONNECTED, BB_BUSY, MOCAP_STALE, NOT_STREAMING; empty hand → `bb/reload` with a
10 s `RELOADING→IDLE` wait → NO_BALL). CANT_MAKE_LEAD is pre-checked (lead < 2.5 s) AND
covered by BB's own reject surfaced as REJECTED_BB. Aborts: no announcement within
`throw_delay + 0.5 s` → ABORTED_NO_ANNOUNCEMENT; a post-release infeasible catch target
(from `trajectory/target_feedback`) → MISSED_INFEASIBLE_<gate-code>; leaving CATCH mode
mid-sequence → ABORTED_MODE_CHANGED; a BB fault → ABORTED_BB_ERROR. The FSM imports no
ROS and every transition is a pure function of `(now, observations, discrete events)`.

### `reload_coordinator_node.py` — thin orchestrator

Caches the observations the FSM reasons about (BB heartbeat, mocap freshness, streaming,
control mode, tracked-ball CAUGHT status, target feedback), calls `bb/reload` and
`bb/throw_at_target` on the FSM's behalf, and publishes phase feedback / the outcome.
`_build_observations` and `_step_sequence` are the testable seams. The catch point is
`(0, 0, GEOM_INITIAL_HEIGHT_MM + JB_OP_DEFAULT_ACTIVE_Z_MM)` = (0, 0, 744.3) mm world
(`compute_catch_point_mm`); the z-convention is hardware-UNVERIFIED and is exactly what
7a checks with an aim-only (speed 0) command before any ball flies.

### `BallButlerThrow` point-target extension

`use_target_point` skips the QTM rigid-body lookup and aims at
`target_point_global_mm`; `aim_only` commands speed 0 (no ball, no announcement).
Default-zero fields preserve every existing `target_name` caller.

## Implementation

New: `Reload.action`, `reload_sequencer.py`, `reload_coordinator_node.py`, the
`BallButlerThrow` fields + `ball_butler_node` handler branch,
`_plan_and_install_catch` in `trajectory_node`, setup.py + launch wiring, and the
staged operator protocol. Tests: 3 trajectory-node catch, 3 BB-node point-target,
19 sequencer, 10 coordinator-node, 3 announcement→correlation→coordinator integration.
Separable follow-up: `CATCH_VEL_RATIO` 0.9→0.6 + refreshed gate JSONs.

## Verification

- Full suite: `pytest tests/ -q` (2026-07-08) = **2261 passed, 1 xfailed in 541.51 s**
  (baseline 2223 passed / 1 xfailed at Phase-6 audit-fix `bf5b46e`; net **+38** = the
  new tests only: 3 trajectory-node catch + 3 BB-node + 19 sequencer + 10 coordinator
  + 3 integration; no regressions).
- `colcon build --packages-select jugglebot_interfaces jugglebot` (2026-07-08) =
  **2 packages finished, 0 errors** (`Reload.action`, `BallButlerThrow` fields, the new
  node entry point + launch).
- Reload-gate evidence refreshed at `CATCH_VEL_RATIO = 0.6` (deterministic seeded
  reruns, 2026-07-08, `python sim/reload_gate.py --trials 20 --seed {0,100,200,300,400}
  …` per the Phase-6 recipe): all five **CORE PASS** — caught 20/20, feas_viol 0,
  pump_rejects 0, hold travel ≤ 0.02 mm, tilt ≤ 0.01°, sep 0 ms, required leg limits
  156/660/10331 **unchanged**. The deferred vel-match rose (larger designed mismatch):
  nominal worst **0.343 → 0.440** (mean 0.286 → 0.420); sweeps arm+30 **0.647**,
  arm−30 **0.692**, ev+10 **0.404**, ev−10 **0.494**.
- No `hardware_config.yaml` / codegen change this phase → no codegen determinism gate.

## Discussion

### Fork (mine, decide+document) — the receive tilt stays in `catch_coordinator`, not moved to `target_vel`

The task's inherited framing said "tilt-to-receive from the announced arrival VELOCITY
in `DynamicTargetCommand.target_vel`". Ground truth diverges: `target_vel` is **always
zero** (`catch_coordinator.CatchCommand.target_vel = np.zeros(3)  # Stationary catch`),
and the receive tilt is already computed by the hardware-validated
`catch_coordinator.compute_catch_orientation` (the same collinear-catch geometry as
`tilt_geometry.tilt_to_receive`) and shipped in `target_quat`. Two options:

- **(chosen) Use the tilt already in `target_quat`; swap `build_timed → build_catch`
  in `trajectory_node` only.** `build_catch` reads the tilt from the catch pose to aim
  the through-seat rate. This is a LOCAL, reversible change that fully closes the gap
  (reach-only → tilt-through-seat + quiescent hold) with the tilt genuinely collinear
  with the arrival velocity.
- **(rejected) Move tilt computation to `trajectory_node` from `target_vel`.** This
  requires the coordinator to change `DynamicTargetCommand`'s `target_vel`/`target_pos`
  semantics (the pre-computed hand-offset centroid depends on the tilt), i.e. a
  **cross-subsystem contract change** to a hardware-validated node — a HARD-STOP fork
  under the run's fork policy, and one that buys nothing the chosen path doesn't (the
  tilt is already collinear-to-velocity).

Concrete failure mode the choice avoids: a needless wire/contract change to the catch
node that has caught balls smoothly on hardware, risking a regression in the proven
catch geometry for zero functional gain. Residual: `compute_catch_orientation` and
`tilt_geometry.tilt_to_receive` are two implementations of the same geometry — a
pre-existing situation (Phase 6 created `tilt_geometry`), NOT new drift introduced here;
the ROS catch path uses only `compute_catch_orientation`. Consolidating them is a future
cleanup (Open Question 2), not a Phase-7 obligation.

### Fork (mine, decide+document) — the reload coordinator orchestrates only

The coordinator does not switch modes (the operator sets CATCH), does not plan platform
motion (trajectory_node does), and does not arm the hand (catch_coordinator does). It
calls exactly two BB services and watches state. Root cause this prevents: a second
motion path (a coordinator that commanded the platform directly) would bypass
`feasibility.validate` — the single-gate contract the whole plan rests on — and could
also fight the catch_coordinator's hand arming. Keeping it an orchestrator makes the
reload a *composition* of proven paths.

### Fork (mine, decide+document) — synthesized-message integration test, not rosbag replay

The task offered recorded bags OR synthesized messages. I built the
announcement→correlation→coordinator integration test on synthesized messages driving
the **real** `BallTracker` (correlation) + `CatchCoordinator` (policy) engines — the
logic the thin nodes wrap. Concrete reasons: (a) the ~/Desktop/rosbags captures predate
this trajectory/`build_catch` pipeline (their `DynamicTargetCommand` came from the old
MPC catch path), so a replay would validate a dead path; (b) there is no rosbag2 reader
in the ROS-mocked pytest environment; (c) the per-node message seams are covered by the
per-node tests. The test asserts the load-bearing behaviour: a confirmed jugglebot-bound
ball yields a catch command with a receive tilt, a ball for another destination is
ignored (the correlation tag routes the reload), and a vertical arrival is near-level.

### Fork (mine, decide+document) — `BallButlerThrow` gets `aim_only` (a third field)

The plan's § Reload sequence lists two new srv fields; Phase 7a needs a speed-0
aim-only fast-path. I added `aim_only` (default false) rather than a separate service.
Root cause this prevents: a duplicate aim-only service diverging from the throw path's
world→BB-local + aim-correction + IK chain; one handler with a `throw=not aim_only`
branch reuses that chain exactly. Additive + default-zero → existing callers unchanged.

### The `CATCH_VEL_RATIO` follow-up stayed small

The Phase-6 HIGH open question feared a dozen-plus test reconciliations. In fact only
`test_hand_trajectory` imports the constant, and it references it **symbolically**
(`-CATCH_VEL_RATIO * 3.0`), so the tests auto-adjusted; only one stale inline comment
needed touching. The core reload-gate result is ratio-independent (the ball is caught
and held either way); only the *deferred* vel-match metric shifted, and it shifted *up*
(a larger designed first-contact mismatch at 0.6) — reinforcing the Phase-6 conclusion
that the ≤15%-at-first-contact criterion is inconsistent with the hardware-validated
hand, not a trajectory defect.

## Open Questions

1. **Catch z-convention (744.3 mm) is hardware-UNVERIFIED.** 7a's aim-only command
   verifies the QTM-world vs jugglebot-base frame AND this z before any ball flies. If
   7a needs a correction, update `reload_sequencer.compute_catch_point_mm` and re-run
   the software gate.
2. **Two copies of the collinear-catch geometry** (`catch_coordinator.
   compute_catch_orientation` quaternion-based, `tilt_geometry.tilt_to_receive`
   rotvec-based). Pre-existing (not introduced here); a future consolidation to a single
   source of truth. Not a Phase-7 blocker (the ROS path uses only the coordinator's).
3. **Vel-match / `CATCH_VEL_RATIO`** (carried from Phase 6): the ≤15%-first-contact
   metric remains inconsistent with the 0.6 hand design; revisit the *definition*
   (measure over the seat stroke, or against the 0.6 design) with 7b/7c hardware
   evidence — the operative hardware contact guard is 7b's two-consecutive-bounce-out
   abort.
4. **Reach envelope vs offsets under noise** (carried from Phase 6): two nominal gate
   trials still flag > 80 mm reach; caught, not fatal; tighten the offset or widen the
   envelope with hardware evidence in 7c.

## Related

- Plan: `plans/active/mvp-trajectory-bringup.md` § Phase 7, § Reload sequence,
  § The command seam, § Hand-catch smoothness.
- Predecessor: `logbook/2026-07-08-mvp-phase6-catch-trajectory-sim-gate.md`
  (`build_catch`, the reload gate, the `CATCH_VEL_RATIO` finding this phase closes).
- Predecessor: `logbook/2026-07-08-mvp-phase5-timed-targets.md` (the `build_timed`
  catch path + reach-freeze this phase's `build_catch` swap extends).
- Operator protocol: `tests/hardware/session_phase7_reload.md` (7a/7b/7c).
- Gate evidence (refreshed at 0.6): `logbook/artifacts/2026-07-08-mvp-phase6/*.json`.
