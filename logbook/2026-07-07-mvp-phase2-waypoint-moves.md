---
title: MVP Phase 2 — Waypoint moves at very low limits (full gate + build_move)
type: feature
date: 2026-07-07
status: resolved
phase: "2"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/limits.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/state_machine.py
  - ros_ws/src/jugglebot/launch/jugglebot_launch.py
  - ros_ws/src/jugglebot_interfaces/srv/GoToPose.srv
  - ros_ws/src/jugglebot_interfaces/srv/SetTrajectoryLimits.srv
  - ros_ws/src/jugglebot_interfaces/msg/TrajectoryStatus.msg
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - tests/motion/test_trajectory_feasibility.py
  - tests/motion/test_trajectory_planner_move.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_state_machine.py
  - tests/ros/conftest.py
commits:
  - 614820c
  - 1dc9571
  - f38153f
subsystem:
  - motion
  - ros
tags:
  - feature
  - trajectory
  - safety
  - feasibility
---

# MVP Phase 2 — Waypoint moves at very low limits (full gate + build_move)

## Summary

Phase 2 of `mvp-trajectory-bringup.md`: the trajectory generator gains its
**arbitrary-target profiled move surface** and its **full feasibility gate**.
`trajectory/go_to_pose` (a new `GoToPose` service, TRAJECTORY mode only) drives
`planner.build_move`, which runs the complete `feasibility.validate` gate
(reachability / workspace / condition, leg vel/acc/jerk peaks, per-knot step bound)
plus a duration-stretch loop that finds the minimal feasible duration or **loudly
rejects** a too-tight requested one (`TOO_FAST`, with the achievable minimum in the
response). `trajectory/set_limits` (`SetTrajectoryLimits`) is the in-session leg-limit
ramp (each value clamped to its YAML hard ceiling). `trajectory/status` migrates off
the Phase-1 `DiagnosticStatus` stand-in to the typed `TrajectoryStatus` msg, and
`trajectory/diagnostics` publishes the active move's measured leg peaks + emitter
jitter. The orchestrator gains `ActiveMode.TRAJECTORY` and a `'trajectory'` command.

All motion still flows through the single `planner` → `feasibility.validate` gate —
Phase 2 fills in the gate's checks without reshaping the contract or adding any
side-channel motion path.

Software is complete and the full suite is green (count triple in Verification).
**Hardware validation is DEFERRED** to an operator bench session — the waypoint move
battery + one loud-rejection demo at the default low limits (protocol in the plan's
Phase 2 detail; the reusable `tools/probes/traj_stream_probe.py` from Phase 1 shows
the streamed knots read-only).

## Motivation

Phase 1 proved the streaming substrate by holding the ACTIVE pose. Phase 2 is the
first phase that *moves* the platform under command, so it is where the loud-rejection
guarantee earns its keep: an operator asks for a pose + duration, and every request is
either planned smoothly or rejected with a machine code and an achievable duration —
never silently dropped, never mid-stream at the pump. The move envelope starts at the
deliberately tiny default limits (100 mm/s, 400 mm/s², 8000 mm/s³) and is ramped up in
Phase 4; the gate's always-on jerk/accel/vel enforcement is the primary smoothness
mechanism.

## Design

### The full feasibility gate (`feasibility.validate`)

Phase 1 shipped a minimal gate (stroke + workspace + vel/acc caps). Phase 2 completes
it. The function is a **pure predicate over a fully-specified plan** — it never mutates
durations (the stretch loop lives in the planner) — and it computes **all** peaks in
one pass so the planner can read the worst limit ratio off a single report.

Order (first failure wins the `code`):

1. **Geometry**, per dense sample (200/segment): non-finite reject (`UNREACHABLE`);
   leg extensions within the hard stroke margins (`WORKSPACE`); Jacobian condition
   number under the workspace hard bound (`UNREACHABLE`). These early-return — a
   non-finite / out-of-stroke / near-singular pose makes the Jacobian peaks
   meaningless.
2. **Leg kinematic peaks** via the `ik_solver` Jacobian chain: peak leg velocity
   (`LIMIT_VEL`), acceleration (`LIMIT_ACC`), and jerk (`LIMIT_JERK`). Jerk is the
   finite difference of the *analytic* leg acceleration — a second-order-accurate
   stand-in for the third difference of leg position — taken **per segment** (the plan
   is C2, not C3, so jerk is only C0 across joins; a cross-join difference would
   fabricate a spurious spike at the seam).
3. **Knot-step bound** (`STEP_BOUND`): the actual wire `u0` sequence is the plan
   sampled at the 25 ms knot spacing mapped `ext × mm_to_rev`; the max per-knot `|Δu0|`
   must stay under `max_step_rev` with a 20 % margin. This rejects a step-heavy move
   **before motion**, rather than one tick after `mpc_active=1` at the pump.

### `planner.build_move` + the duration-stretch loop

`build_move(state0, target_pose, duration_s, limits, geom)` builds a rest-terminating
quintic from the seed state to the target.

- `duration_s` **None / ≤ 0** → the **minimal feasible** duration from the stretch loop.
- `duration_s` **> 0** → honoured if `≥` the minimal feasible; otherwise
  `TrajectoryInfeasible(TOO_FAST, …, min_duration_s=t_min)`. A too-tight duration is
  *rejected*, never silently stretched — the operator asked for a specific timing.

The stretch loop starts at the `min_move_duration_s` floor and multiplies `T` by
`max(r_vel, √r_acc, ∛r_jerk, r_step) · 1.05`. From a rest start the leg peaks scale
**exactly** as `1/Tⁿ` (the spatial path is fixed; only its timing rescales), so this
factor lands the binding constraint at `limit / 1.05²` in a single stretch — convergence
in ≤ 2 `validate` calls (measured: ≤ 3, asserted in a test). A `WORKSPACE` / `UNREACHABLE`
failure is spatial (a longer `T` traces the same poses) and re-raises immediately.

### `TrajectoryLimits.with_session_limits`

The `set_limits` ramp: a clamped session-limit override that keeps the ceilings /
knot / step / duration fields and can never raise a limit past its YAML ceiling.

### `trajectory_node` (thin wrapper)

`trajectory/go_to_pose` gates on TRAJECTORY mode (`WRONG_MODE` otherwise — a scripted
move from STANDBY/SPACEMOUSE/CATCH is a mode confusion, not a motion command) and on
being seeded (`STALE_STATE`), converts `geometry_msgs/Pose` → `pose_6dof` (quaternion
→ rotvec), and installs the built plan; a reject leaves the held hold plan untouched.
`trajectory/set_limits` swaps the frozen limits reference atomically. Status is the
typed `TrajectoryStatus`; diagnostics publish the last accepted move's peaks + jitter.

### Orchestrator

`ActiveMode.TRAJECTORY` + a `'trajectory'` command in ACTIVE (the orchestrator node
has no command whitelist — the state machine's ACTIVE handler is the single accept
point). STANDBY's docstring now reads "hold at neutral via trajectory_node".

## Implementation

The pose→leg chain is reused verbatim from `motion/ik_solver.py` and
`motion/workspace.py`; Phase 2 adds only the gate's jerk/step passes and the planner's
stretch loop. Empirical head-start (throwaway probe on the pinned stack): at the default
limits a z 170→190 move needs 0.544 s (binding: acc 362.8 ≈ 0.907·400, the exact 1.05²
margin signature); an explicit 0.05 s request for a 20/20/15 mm move raises `TOO_FAST`
with `min_duration_s = 0.629 s`; an out-of-stroke target raises `WORKSPACE` in the first
iteration (not `TOO_FAST` after exhausting iters). The interface package rebuilt clean
(`colcon build --packages-select jugglebot_interfaces jugglebot`, 2026-07-07, 1 min 41 s).

## Verification

(date, command, result triples — re-runnable from the artefact alone)

- **New motion gate + planner tests** (`pytest tests/motion/test_trajectory_feasibility.py
  tests/motion/test_trajectory_planner_move.py -q`, run 2026-07-07) = **33 passed** (23
  feasibility incl. the 12-seed peak-bound property test + 10 planner-move).
- **Node + state-machine tests** (`pytest tests/ros/test_trajectory_node.py
  tests/ros/test_state_machine.py -q`, run 2026-07-07) = both green within the group run
  below; `test_trajectory_node.py` alone = **22 passed in 7.24 s** (was 12; +10).
- **colcon build gate** (`colcon build --packages-select jugglebot_interfaces jugglebot`,
  run 2026-07-07, no venv, ROS Foxy) = **2 packages finished [1 min 41 s]**, 0 errors —
  the three new interfaces (`GoToPose` / `SetTrajectoryLimits` / `TrajectoryStatus`)
  generate cleanly.
- **Full suite** (`pytest tests/ -q`, run 2026-07-07) = **2041 passed, 1 xfailed in
  500.17 s** — 0 failed. Baseline before Phase 2 (`pytest tests/ -q`, 2026-07-07, post
  Phase-1 + audit fixes, c0b31a9): 1996 passed, 1 xfailed. Net **+45 passed**, fully
  accounted for by the new tests and nothing else: 23 (`test_trajectory_feasibility.py`)
  + 10 (`test_trajectory_planner_move.py`) + 10 (`test_trajectory_node.py`) + 2
  (`test_state_machine.py`). No pre-existing test changed count; the 1 xfailed is
  unchanged.
- **Codegen determinism** (2026-07-07): no `config/hardware_config.yaml` change this
  phase (the `JB_TRAJ_*` constants landed in Phase 1); re-running
  `python config/generate_config.py` produced **no** working-tree changes.

### Hardware bench session — 2026-07-09, S2: **PASS**

Operator-run battery `tests/hardware/traj_ramp_battery.py --lean-gain 0.0`, no
`--set-*` flags, so limits stayed at the Phase-1 defaults (100 mm/s, 400 mm/s²,
8000 mm/s³). Evidence extracted read-only from rosbag
`~/Desktop/rosbags/2026-07-09_13-17-56` (`/trajectory/diagnostics`,
`/trajectory/status`, `/control_mode_topic`, `/orchestrator_command`).

- **11/11 feasible moves accepted and executed**, 13:34:07 → 13:34:43, one install per
  move (`move_seq` 1…11), `lean_gain = 0.00` throughout, `plan_kind='move'`.
- **Predicted vs realized leg peaks** (worst case across the 11 moves):

  | | vel (mm/s) | acc (mm/s²) | jerk (mm/s³) |
  |---|---|---|---|
  | gate-predicted | 68.4 | 362.8 | 6 911 |
  | realized | 68.3 | 362.8 | 5 583 |
  | session limit | 100.0 | 400.0 | 8 000 |
  | realized headroom used | 68 % | **91 %** | 70 % |

  Vel and acc track the prediction to ~1 %; realized jerk runs ~20 % under, consistent
  with the gate's deliberately conservative jerk bound. **Acceleration is the binding
  constraint at the Phase-1 defaults** — not jerk. That does not contradict the Phase-4
  ramp ordering (which raises jerk first): jerk binds at the *Phase-6 catch* operating
  point, where the reach is faster and shorter, not at these gentle defaults.
- **Loud rejection, zero motion** — the `duration_s: 0.05` request returned
  `TOO_FAST: requested duration 0.050s < minimal feasible 0.629s for this move at the
  current limits`. `move_seq` held at 11 across the rejection, i.e. **no plan was
  installed**. This is the load-bearing Phase-2 invariant and it held on hardware.
- **Emitter** — session-max `max_emit_gap_ms` 56.60 (vs the 250 ms staleness window);
  higher than Phase 1's 42.27 ms, as expected with the follower/planner work on the tick.
- No pump rejects; no oscillation or audible snap reported by the operator.

**Two operator hazards surfaced (documented, not code defects).** (1) `ros2 topic pub
--once` lost the `trajectory` mode change to the DDS discovery race — no error, no log
line. The operator armed without re-verifying the mode, so all 11 moves came back
`WRONG_MODE` and nothing moved (the mode gate behaving exactly as designed). (2) The
cleanup then issued `deactivate` while still armed. That is a two-sided failure: the
state machine's ACTIVE→IDLE transition is pure software and lands instantly, dropping
`control_mode` to `''` — so the emitter stops and the guard latches `MPC_STALE` inside
250 ms — while the *firmware* rejects the DEACTIVATE because `mpc_active=1`, so the legs
never profile-stow. Observed end state: orchestrator IDLE, platform still at 2.196 rev
(the ACTIVE pose), `fault_state=MPC_STALE`, `mpc_active=0`, `setpoints_rejected=0`,
buses OK. Cleared with `/clear_errors` (a bridge service — the orchestrator only routes
`'clear_errors'` from its FAULT state, so a topic publish from IDLE is discarded), then
re-activated; the retry passed clean. Both are now runbook Sharp Edges #5 and #6, and
the S2 protocol gained an explicit verify-the-mode-before-arming gate.

**Two NOTES for follow-up.**

1. `/link_status` is **not** in the launch's rosbag record list, so the MPC_STALE E-STOP
   at 13:29:47 left no trace in the bag; it was only visible on the live topic. The
   fault channel is invisible to post-hoc analysis — a one-line launch fix, worth doing
   before S4 starts generating bags that matter.
2. The teardown `go_home` installed as `move_seq=12` with realized peaks 0.0 (correct —
   it is a genuine no-op from the neutral pose) but reported **predicted** peaks
   identical to move 11's (44.3 / 303.6 / 6911) rather than zero. `peak_leg_*` therefore
   looks **stale for a zero-distance plan**. Harmless here, but S4's per-step review
   reads predicted-vs-realized headroom straight off these fields, so confirm or fix
   before the ramp.

## Discussion

CLAUDE.md makes the Discussion non-negotiable: several reversible forks were decided
under the autonomous decide+document policy.

### Why `GoToPose.code` is a `string`, not the plan's `int32`

Fork — the plan's Architecture wrote `int32 code` for the service response, but the
feasibility layer's codes are already a **string** enum (`OK`, `LIMIT_ACC`, …) that is
tested and consumed across `feasibility.py` / `planner.py`. Concrete failure modes a
string field prevents: (a) an `int↔string` mapping table is a second source of truth
that silently drifts from `feasibility.py` when a code is added — a `string` field is
the enum verbatim, so there is nothing to drift; (b) an int on the wire is opaque —
`ros2 service call` shows `code: 4` instead of `LIMIT_ACC`, forcing the operator to
keep a decoder ring during a safety-sensitive bench session. The field is brand-new
with no external consumers (this phase is its only producer/consumer; Phase 5's
`TimedTarget.srv` mirrors whatever shape ships here), so the choice is cheap to make
now and expensive to leave inconsistent. Chose `string`; noted here so Phase 5 follows
suit.

### Why the stretch loop starts at the duration floor, not the plan's pose-space pre-size

Fork — the plan lists rest-to-rest pre-size formulas (`peak_vel = 1.875·|Δp|/T`, etc.)
to seed the initial duration. Those are **pose-axis** peaks (mm/s, rad/s), but the gate's
limits are **leg-space** (mm/s of extension), so the pre-size is only an approximation
that still needs the iteration to correct. Starting at `min_move_duration_s` and relying
on the *exact* `1/Tⁿ` leg-space scaling of the stretch factor is simpler and provably
converges in one stretch from a rest start (the binding leg peak lands at `limit/1.05²`
by construction — verified numerically and asserted by `test_stretch_converges_in_few_iters`).
The pose-space pre-size would add a second, inexact duration estimate for no iteration
saving. The closed-form pose-space peak functions are still copied + tested
(`test_closed_form_peaks_match_dense_sampling`) as validated utility math later phases
may use; the gate itself measures true leg-space peaks by dense sampling, which is
strictly more accurate than any pose-space bound.

### Why the condition-number check is defence-in-depth here

Fork — include a Jacobian condition-number reachability check (mapped to `UNREACHABLE`)
even though a sweep found **no** in-stroke pose in the current geometry that exceeds the
workspace hard bound (`cond_hard ≈ 5.59`; poses go out-of-stroke before they go
ill-conditioned, so `WORKSPACE` always fires first). Root cause it guards: the plan's
gate spec names `check_workspace_limits` (which includes the condition bound), and a
future geometry / larger envelope / a spacemouse target pushed to the edge *could* reach
a near-singular but in-stroke pose where the Jacobian peaks are numerically unreliable.
Rejecting there is correct; it costs one `np.linalg.cond` per sample (already computed
for the peaks). Tested via monkeypatch (`test_reject_unreachable_singular`) since no real
pose triggers it — the test documents that the branch is a live backstop, not dead code.

### Why validate computes all peaks (no early return on a limit failure)

Fork — Phase 1's gate early-returned on the first limit failure. Phase 2's does a full
pass and returns all four peaks. Root cause: the duration-stretch loop needs the *worst*
ratio across vel/acc/jerk/step to pick the correct stretch factor in one step; an
early-return report would only carry the first-failing peak, forcing multiple
gate calls (or a second full pass) to find the true binding constraint. Computing all
peaks once is cheaper and makes the stretch exact. Geometry checks still early-return
(a bad pose makes the Jacobian peaks meaningless).

### Why `trajectory/diagnostics` stays `diagnostic_msgs/DiagnosticStatus`

Fork — `trajectory/status` migrated to the typed `TrajectoryStatus.msg`, so the
obvious symmetry would be to type `trajectory/diagnostics` too. Kept it as
`diagnostic_msgs/DiagnosticStatus` deliberately. Root cause it addresses: the
diagnostics payload is an open-ended, evolving bag of measured peaks — Phase 4 adds
per-move peak tracking and a `/diagnose` rosbag summariser, so the field set is not
yet stable. `DiagnosticStatus`'s `KeyValue` list absorbs new keys with **no interface
rebuild** (no `.msg` edit, no `colcon build --packages-select jugglebot_interfaces`,
no downstream regeneration), whereas a typed message would force an interface churn
every time a peak is added. `status` is different — its fields (streaming/mode/
plan_kind/time_remaining/seq) are a fixed contract worth typing. Chose flexibility
for the still-moving diagnostics surface, typing for the settled status surface.

### Why the code landed as two commits (motion layer, then ROS surface)

Fork — the phase could have been one commit. Split into `614820c` (pure-motion:
`feasibility.py` full gate + `planner.build_move` + `limits.with_session_limits`,
all numpy, no ROS) and `1dc9571` (ROS surface: interfaces + node handlers +
orchestrator). Root cause: rollback granularity. The pure-motion layer is
independently testable (`tests/motion/`) and has no ROS/interface dependencies, so a
regression found later in either the math or the ROS wiring can be reverted in
isolation without dragging the other half with it — and `git blame` on a motion-math
line lands on a commit that is *only* motion math, not buried in an interface churn.
The two layers also have different review surfaces (control-system correctness vs
ROS plumbing), which the split keeps legible.

### Deviation carried from Phase 1: armed-mode-exit is still a sharp edge

The Phase-1 note stands: leaving a streaming mode while the bridge is ARMED latches
`MPC_STALE` within 250 ms. Phase 2 adds `ActiveMode.TRAJECTORY` but deliberately does
**not** yet couple mode-exit to an auto-disarm — that structural fix (orchestrator-owned
auto-disarm on ACTIVE sub-mode exit) is scoped with the broader orchestrator-automated
arming, which is explicitly Deferred in the plan. The operator protocol (disarm before
mode change) remains the guard.

## Open questions / next steps

- **Hardware session is the gate** (deferred): the plan's Phase-2 battery — z
  170→190→170, x ±20, y ±20, tilt rx ±3°, then one `duration_s: 0.05` request that must
  reject `TOO_FAST` with `min_duration_s` and zero motion. PASS per move: subjectively
  smooth, `/diagnose --latest` shows leg jerk within limits, no pump rejects, no E-STOP.
- **ci-deep** is nominally due at the end of Phase 2 (Testing Plan). This phase's new
  math tests are deterministic `parametrize` (not hypothesis-driven), so ci-deep adds
  little for the Phase-2 additions specifically; deferred to run alongside Phase 5's
  property-heavy `build_timed` work, or on request.
- Phase 3 (SpaceMouse) adds `follower.py` + the `platform_pose_topic` publisher-field
  gating and gravity-offset composition (verbatim ports from `mpc_bridge_node`).

## Audit fixes (2026-07-07)

A `/audit` of the two Phase-2 commits (`614820c..1dc9571`) returned one BLOCKING,
two WARNING, and six NOTE findings. All were applied in a single follow-up commit
(one code commit + a SHA-backfill follow-up). Findings → fixes, one line each:

**BLOCKING — mid-move `go_to_pose` install step.** The seed is sampled at service
entry, but the full gate takes ~1.5 s (4–5 `validate` passes at ~377 ms each,
measured on this Jetson) while the emitter streams the OLD plan; installing then
jumps `u0` back to the stale seed (measured ~0.083 rev / 25 ms ≈ 575 mm/s transient
that passes both step gates). Three-part fix:
- *Install-continuity guard (permanent)* — `_svc_go_to_pose` re-samples the
  commanded state immediately before install and compares the plan's t=0 leg
  positions against the live ones in `motor_rev` (the pump's units, via the exact
  emitter chain). Drift past `0.25·STEP_BOUND_MARGIN·JB_OP_MAX_POSITION_STEP_REV`
  (≈0.06 rev) → `STALE_STATE` reject, no install.
- *Phase-2 `BUSY` restriction (temporary, documented)* — a `go_to_pose` while a
  move is in flight is rejected `BUSY` (a service-level code beside `_MOVE_MODE`,
  NOT a feasibility-enum member). Moves are accepted only from a hold; interrupting
  an in-flight move (supersede) needs the follower's C2 chaining and is lifted by
  **Phase 3/5**.
- *Validate-perf quick wins* — `build_move` returns its accepting `FeasibilityReport`
  (node drops the redundant re-validate); an explicitly-requested duration is
  validated FIRST and honoured if the gate accepts it (the stretch loop runs only on
  failure, to populate `min_duration_s`); the once-per-geometry `WorkspaceLimits`
  construction is hoisted out of `validate()` into a per-geom cache.

**WARNING — `go_to_pose` skipped telemetry-staleness.** Added a `_robot_state_fresh`
gate after the seeded check: stale telemetry → `STALE_STATE`, 'cannot plan a move'.

**WARNING — STANDBY semantics.** Corrected the `state_machine` STANDBY docstring
(STANDBY *silences* move commands; trajectory_node keeps streaming and holds at the
seeded/last-reached pose, NOT neutral; return-to-neutral is `trajectory/go_home`).
Behaviour: leaving TRAJECTORY mid-move for another streaming mode now installs a
profiled C2 decel-stop (`build_hold` from the moving seed) — if the stop is too
aggressive for the gate at the current limits, it logs loudly and lets the
already-validated move complete (safe) rather than snapping a jerk-violating stop.

**NOTE — jerk endpoint bias.** Documented next to `_MIN_SAMPLES`: the forward-diff
jerk under-measures the endpoint peak by ≤ 1.5 % at 200 samples/segment; accepted
because the jerk limit is session-ramped by feel with large ceiling headroom.

**NOTE — non-finite `duration_s`.** `_svc_go_to_pose` rejects a NaN/Inf `duration_s`
up front (`TOO_FAST`, 'non-finite duration_s') before the `> 0.0` branch.

**NOTE — plan Architecture said `int32 code`.** Corrected the plan's GoToPose line to
`string code` (verbatim feasibility/service code; the TimedTarget "same response
shape" bullet now names `string code`).

**NOTE — Discussion recorded 4+1 of 6 decisions.** Added the two missing bullets:
`trajectory/diagnostics` staying `DiagnosticStatus` (KeyValue flexibility, no
interface rebuild for Phase-4 peak tracking) and the two-code-commit split
(pure-motion layer vs ROS surface, for rollback/blame clarity).

**NOTE — `TOO_FAST` false-reject.** A requested duration the gate accepts but which
sits below the 1.05-margin-padded `t_min` was being rejected. Fixed by the
validate-requested-first restructure above; pinned by
`test_requested_below_padded_tmin_but_gate_accepts_is_honoured` (empirically: a
z 170→190 move, padded `t_min` = 0.5440 s, request 0.5386 s validates OK → now
accepted).

**NOTE — planner spatial-invariance comment.** Annotated `_STRETCHABLE` that the
"spatial path is duration-invariant" argument holds only for a REST seed; a moving
seed re-validates every candidate `T` (which the stretch loop already does).

**Deferred to Phase 3 (not fixed here).** `validate()` at ~377 ms (200-sample gate,
measured 2026-07-07 on the Jetson) is far too slow for the SpaceMouse follower's
per-40 Hz-tick replan; it must drop to low-single-digit ms via a vectorised sampling
chain, a decimated/skipped per-sample condition-number SVD, and/or a follower-scoped
reduced gate. The `WorkspaceLimits` hoist is a first step, not sufficient. Recorded
as an explicit Phase-3 prerequisite in the plan.

**Verification.** Scoped: `pytest tests/motion/ tests/ros/test_trajectory_node.py
tests/ros/test_state_machine.py -q` (run 2026-07-07) = **298 passed**. Full suite:
`pytest tests/ -q` (run 2026-07-07) = **2047 passed, 1 xfailed in 501.12 s**, 0
failed (baseline before these fixes: 2041 passed, 1 xfailed; net **+6** = the six
audit-fix tests only — telemetry-stale reject, BUSY reject, non-finite duration
reject, install-continuity reject, STANDBY-exit profiled stop, and the TOO_FAST
false-reject regression).

## Related

- Plan: [`plans/active/mvp-trajectory-bringup.md`](../plans/active/mvp-trajectory-bringup.md) — Phase 2 detail + the feasibility-gate architecture.
- [2026-07-07-mvp-phase1-streaming-foundation.md](2026-07-07-mvp-phase1-streaming-foundation.md) — the streaming substrate this builds on; the minimal gate this completes.
- [2026-06-29-canbridge-phase0-native-harness.md](2026-06-29-canbridge-phase0-native-harness.md) — format precedent.
