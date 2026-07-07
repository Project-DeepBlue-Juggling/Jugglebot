---
title: MVP Phase 3 — SpaceMouse streaming (fast follower gate + always-valid graceful stop)
type: feature
date: 2026-07-08
status: resolved
phase: "3"
related_plan: mvp-trajectory-bringup.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/segment.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/follower.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/__init__.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/motion/test_trajectory_follower.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/conftest.py
commits:
  - 5cac69b
  - 4bdc688
  - PENDING_DOCS_SHA
subsystem:
  - motion
  - ros
tags:
  - feature
  - trajectory
  - safety
  - feasibility
  - spacemouse
---

# MVP Phase 3 — SpaceMouse streaming (fast follower gate + always-valid graceful stop)

## Summary

Phase 3 of `mvp-trajectory-bringup.md`: continuous **target following** through the
same validated streaming layer. A new pure `TargetFollower` turns a
continuously-updated platform-pose target (SpaceMouse / GUI / shell) into a stream
of C2-continuous plans that the 40 Hz emitter samples. It rests on the two
orchestration prerequisites the Phase-2 audit flagged:

1. **A fast feasibility gate.** The analytic `feasibility.validate` costs ~377 ms on
   this Jetson — three orders of magnitude too slow for a per-40 Hz-tick replan. New
   `feasibility.validate_follow` measures the **identical** leg-space quantities a
   cheaper way (vectorised pose→extension sampling + finite-difference vel/acc/jerk;
   decimated condition SVD) at **~1.5–4 ms**. It is the same gate, not a bypass: same
   module, same `FeasibilityReport`, same codes/limits/stroke logic, **bit-identical
   step-bound** (so the "every emitted frame is pump-accepted" invariant is preserved
   exactly), reached only through `planner`.
2. **An always-valid graceful stop.** New `planner.build_graceful_stop` is a
   duration-stretched decel-to-rest that lengthens its horizon until the gate passes,
   so it **always** produces a valid C2 stop for any gate-limited seed. It replaces
   the Phase-2 STANDBY-exit "catch-and-complete" fallback (a high-velocity mid-move
   exit no longer falls through to letting the move run on to its target) and backs
   the follower's input-loss handling.

`trajectory_node` gains the `platform_pose_topic` subscription (publisher-field mode
gating, verbatim from `mpc_bridge_node`), the `gravity_offset` composition (verbatim
port), a saturation policy (out-of-workspace targets clamped to the nearest reachable
pose with a throttled WARN), and the follower replan integrated into the emitter tick.

All motion still flows through `planner` → a `feasibility` gate; nothing bypasses it.
Software is complete and the full suite is green. **Hardware validation is DEFERRED**
to an operator bench session (gentle flight, saturation shove, mid-flight unplug).

## Motivation

Phase 2 gave the generator arbitrary profiled moves but only *from a hold* — its BLOCKING
audit finding was that the analytic gate is too slow (~377 ms, 4–5 passes ≈ 1.5 s per
move) to run in a streaming loop, so it deferred the per-tick supersede to this phase.
Continuous SpaceMouse following is exactly a per-tick supersede: every 40 Hz tick the
follower must replan from the current commanded state toward the newest target and
install a fresh plan, all inside the 25 ms emit budget (and without threatening the
250 ms staleness E-STOP). That is impossible with a 377 ms gate, so the gate had to
drop to low-single-digit ms — the load-bearing engineering problem of this phase.

## Design

### The fast follower gate (`feasibility.validate_follow`)

**Where the analytic gate's time actually goes** (measured 2026-07-08, throwaway
probe on the pinned stack): of the ~373 ms at 200 samples/segment, the
condition-number SVD is only ~35 ms (~9 %). The dominant cost is the **per-sample
analytic Jacobian chain** in a Python loop — `compute_jacobian` (0.42 ms/sample) and
especially `accel_to_leg_accels` (0.94 ms/sample, which finite-differences the
Jacobian for J̇). So decimating the SVD or reducing samples alone can't reach the
budget (even 12 samples was 24 ms).

**The chosen approach — vectorised finite differences.** `validate_follow` never
touches the analytic Jacobian chain. It:

- samples the segment pose at 300 uniform points in **one** numpy expression
  (`QuinticSegment.eval_pose_batch` — the h-basis on a whole time vector);
- computes leg extensions for all 300 poses batched (`_batched_leg_vectors` — a
  vectorised twin of `ik_solver.compute_leg_vectors`, verified equal to the scalar
  version to 1e-9);
- derives leg **vel/acc/jerk by finite-differencing the sampled extensions** — the
  exact method `validate`'s own step-bound pass already trusts;
- checks stroke on **every** sample (batched); decimates the condition-number SVD to
  12 points (batched, normalised exactly like `workspace.compute_condition_number`);
- reuses `validate`'s **bit-identical** knot-step-bound sampling.

Result: ~1.5 ms for the vectorised core, ~3.7 ms including the scalar knot-step loop.
Accuracy vs the analytic gate (verified across cases): leg **vel/acc/step match to
< 0.1 %**; leg **jerk under-measures by ≤ 1.5 % at 300 samples** — the *same*
endpoint bias `validate` itself already accepts (see its `_MIN_SAMPLES` note) — and is
then inflated by `_FOLLOW_JERK_MARGIN = 1.05` so the follower gate is **strictly
conservative** on the binding constraint (its measured jerk is always ≥ the analytic
peak; over-conservatism only ever rejects a marginal plan, which just keeps the last
valid one — never accepts an over-jerk one).

### `planner.build_follow` — the per-tick follower move

Builds a rest-terminating quintic from the current (moving) commanded state to the
target over `max(min_feasible, horizon_s)`: build at `horizon_s`
(`JB_TRAJ_SPACEMOUSE_HORIZON_S = 0.35 s`, floored at `min_move_duration_s`); if the
fast gate rejects it with a stretchable (vel/acc/jerk/step) failure, stretch the
duration up toward the minimal feasible one and re-gate (≤ 6 iters — a moving seed
breaks the exact `1/Tⁿ` scaling, so a couple of extra passes may be needed; each is a
~1.6 ms `validate_follow`). A spatial `WORKSPACE`/`UNREACHABLE` failure re-raises
immediately (the follower's saturation clamp is what keeps the target reachable).

### `planner.build_graceful_stop` — the always-valid stop

A decel-to-rest **in place** (`p1 == p0`, the seed's own reachable pose), duration
stretched until the fast gate passes. Because it decelerates in place, the only
possible failures are the stretchable vel/acc/jerk/step ones (a longer stop is
monotonically gentler), so it **always converges** for a gate-limited seed — unlike
`build_hold`, which builds one fixed-duration decel and *raises* if that decel is too
aggressive. This is the primitive both "stop now, no matter what" cases need.

### `TargetFollower` (pure) and its policy

Per tick (given the current commanded state and the latest target): **clamp** the
target into the reachable workspace along the `current → target` ray (saturation);
**deadband** against the last target we planned toward (0.5 mm / 0.1°) to stop jitter
churn; else **replan** via `build_follow` — on a gate rejection **keep the last valid
plan** and report the rejection for a throttled WARN (never raises into the hot path).

### `trajectory_node` integration

- The follower replan runs **inside the emitter tick** (drain-to-latest per 40 Hz
  tick, as the plan specifies): a `platform_pose` callback stores the latest
  `(pose, perf_counter)` tuple (atomic single-reference publish, gravity baked in);
  the emitter drains it each tick and calls `_follower_tick` before sampling. The
  ~4–7 ms replan sits comfortably inside the 25 ms budget, so it never threatens the
  emit cadence.
- **Input loss** (no fresh target within `follower_input_loss_s = 0.4 s` — the backstop
  for the SpaceMouse node dying entirely; that node itself publishes an ACTIVE-pose
  hold on unplug) installs an always-valid graceful stop **once**.
- **STANDBY-exit mid-move** now installs `build_graceful_stop` instead of the Phase-2
  `build_hold`-or-let-the-move-complete fallback; generalised to any motion-mode →
  non-motion-mode transition with a move in flight.
- **Gravity offset** and **publisher-field mode gating** are verbatim ports from
  `mpc_bridge_node`.

## Implementation

No ROS interface (`.srv`/`.msg`), config, launch, or setup changes this phase — the
`JB_TRAJ_SPACEMOUSE_HORIZON_S` constant landed in Phase 1, and the follower reuses the
existing `PlatformPoseCommand` / `gravity_offset` topics. So there is **no colcon
build gate and no codegen** for Phase 3 (both confirmed unnecessary, not skipped). The
batched-math primitives were each prototyped and verified equal to their scalar
`ik_solver` / `workspace` counterparts to machine precision (throwaway `/tmp` probes,
not committed) before the gate was written, per the empirical-probe rule.

Two code commits mirror the Phase-2 split: pure-motion layer (`5cac69b`:
`feasibility`/`planner`/`segment`/`follower`/`__init__` + `tests/motion/`), then the
ROS surface (`4bdc688`: `trajectory_node` + `tests/ros/`).

## Verification

(date, command, result triples — re-runnable from the artefact alone)

- **New follower motion tests** (`pytest tests/motion/test_trajectory_follower.py -q`,
  run 2026-07-08) = **23 passed** — batched-helper exactness vs scalar (1e-9),
  `validate_follow` peak agreement + jerk conservatism vs `validate`, the ~ms speed
  guard, `build_follow`/`build_graceful_stop`, the follower clamp/deadband/keep-last
  policy, and the seeded streaming property test (bounded discrete vel/acc/jerk +
  every knot pump-accepted + step-response convergence).
- **Node follower tests** (`pytest tests/ros/test_trajectory_node.py -q`, run
  2026-07-08) = **36 passed** (was 27 at HEAD `8728713`; **+9** follower tests:
  mode/publisher gating both ways, follower-tick install, deadband, input-loss
  graceful stop, node-boundary pump-acceptance over a stream, gravity composition,
  mode-entry reset).
- **Motion + trajectory-node + state-machine subset** (`pytest tests/motion/
  tests/ros/test_trajectory_node.py tests/ros/test_state_machine.py -q`, run
  2026-07-08) = **330 passed** (the `test_motor_guard` return-value warnings are
  pre-existing and unrelated).
- **Full suite** (`pytest tests/ -q`, run 2026-07-08) = **2079 passed, 1 xfailed in
  502.33 s** — 0 failed. Baseline before Phase 3 (`pytest tests/ -q`, 2026-07-07, post
  Phase-2 + audit fixes, `f38153f`): 2047 passed, 1 xfailed. Net **+32 passed**, fully
  accounted for by the new tests and nothing else: 23 (`test_trajectory_follower.py`)
  + 9 (`test_trajectory_node.py`: 27 → 36). No pre-existing test changed count; the 1
  xfailed is unchanged.
- **Codegen determinism** (2026-07-08): no `config/hardware_config.yaml` change this
  phase; no interface change → no colcon build gate. Confirmed both are genuinely
  unnecessary (the follower reuses existing constants/topics), not skipped.

## Discussion

CLAUDE.md makes the Discussion non-negotiable: several reversible forks were decided
under the autonomous decide+document policy, and two touch behaviour near the
workspace edge, so the reasoning matters for future sessions.

### Fork — the fast gate is a vectorised finite-difference gate, not a decimated analytic gate

The task authorised "vectorise the sampling chain, decimate/skip the per-sample
condition-number SVD, and/or a reduced follower-specific check." An empirical profile
killed the obvious first idea (decimate the SVD): the SVD is only ~9 % of the ~373 ms;
the analytic Jacobian chain (`compute_jacobian` + `accel_to_leg_accels`' J̇ finite
difference) is the ~90 %. Sample reduction alone bottoms out at ~24 ms (12 samples) —
still an order of magnitude over budget. Two candidate cures: (a) vectorise the shared
`ik_solver` Jacobian functions, or (b) sidestep the analytic chain entirely with
finite differences of a vectorised extension sampler. I chose (b) because (a) means
rewriting the batched linear algebra of shared, hardware-validated production functions
used by the analytic service-path gate too — high blast radius for a follower-only
speedup — whereas (b) is fully contained in the trajectory package, measures the same
leg-space quantities, and its step-bound check is *bit-identical* to `validate` (so the
pump-acceptance safety invariant is untouched). Concrete failure mode (b) prevents that
(a) risks: a subtle divergence in a batched `compute_jacobian` would corrupt the
*analytic* service-path gate that hardware moves depend on, not just the follower.

### Fork — jerk conservatism (300 samples + ×1.05), not "accept the bias"

Finite-differencing under-measures the true jerk peak (third-difference smoothing).
Concrete failure mode: at the follower's low session jerk limit (8000 mm/s³), a gate
that under-reports jerk by X % would accept a plan whose *true* jerk exceeds the limit
by X % — a real smoothness/safety gap. I pinned the sample count at 300 (bias ≤ 1.5 %,
identical to the endpoint bias `validate` itself already accepts and documents) **and**
inflate the measured jerk by 1.05 so the follower gate is provably never
anti-conservative. vel/acc are left un-inflated (they match to < 0.1 %). Over-rejecting
a marginal plan is benign — the follower keeps its last valid plan — so the asymmetric
margin (tighten only the under-measured axis) is the safe direction.

### Fork — the follower runs in the emitter thread (drain-to-latest per tick), not a callback

The alternative is running the replan in the `platform_pose` subscription callback (per
SpaceMouse message, ~100 Hz) or on a third thread. I ran it inside the 40 Hz emitter
tick because: the plan explicitly says "drain-to-latest per 40 Hz tick"; it naturally
rate-limits replanning to 40 Hz and drains to the newest target for free; the gate is
now cheap enough (~4–7 ms) to fit the 25 ms budget with margin; and it keeps the plan
install on the same thread that already owns `_plan_lock`, avoiding a third thread and
extra cross-thread install races. The one shared read (`_follower_target`) is a single
atomic tuple reference, so a torn read is impossible and a stale read is benign
(next tick catches up).

### Fork (edge-behaviour) — saturation is a ray-clamp to the EXISTING stroke envelope, not a new pose box

This one borders the "envelope/limit semantics" hard-stop, so I was deliberate. The
plan says an out-of-workspace target "clamps to nearest-valid." A true geometric
nearest-valid clamp requires *defining* a workspace box (xy/z/tilt) — i.e. inventing a
new envelope semantic, which is a safety-relevant fork I must not auto-pick. But doing
nothing is worse than a heuristic: `build_follow` on an unreachable target fails the
gate's stroke check at the endpoint, so the platform would **freeze** instead of
tracking toward the shove. My resolution uses **only the existing envelope**: binary-
search the furthest reachable point along the `current → target` ray (via the existing
`check_leg_extensions`), so the platform tracks *up to* the stroke boundary along the
approach direction. This introduces no new limit — it selects a reachable target inside
the envelope the gate already enforces — and every clamped plan is still independently
gated. A true geometric pose-box clamp (a genuinely new envelope shape) is deferred to
the operator (Open Questions). Root cause this respects: the safety enforcement stays
the existing stroke gate; the clamp is only a target-selection heuristic bounded by it.

### Fork (edge-behaviour) — graceful stop is decel-in-place, and cannot stop a super-limit velocity

`build_graceful_stop` keeps `build_hold`'s `p1 == p0` semantics (decelerate to rest at
the seed pose) for consistency, and stretches the horizon. A probe surfaced the physical
subtlety: a decel-in-place from a **super-limit** velocity overshoots the workspace
(excursion ∝ v·T, so a longer stop overshoots *more* while a shorter one violates jerk —
they conflict), so no smooth stop keeps it in stroke. This is physical reality, not a
bug: the follower never produces a super-limit seed (all plans are gate-limited ≤ the
session leg-vel limit, and a limited-velocity stop's excursion stays well in stroke —
verified: vz = 100 mm/s stops in 0.96 s in-stroke; vz = 120 first hits WORKSPACE). So
"always-valid" holds *for gate-limited seeds*, which is all that ever arises. The node
still logs loudly on the (unreachable) super-limit rejection rather than silently
looping. My first test used an unrealistic 250 mm/s and caught exactly this — the test
now uses vz = 60 (fails `build_hold`, stretches to a valid 0.573 s stop).

### Fork — STANDBY-exit replaces catch-and-complete with the always-valid stop

The Phase-2 code, on a high-velocity mid-move STANDBY exit, let the already-validated
move *complete to its target* if a min-duration stop violated the gate. The task
mandates replacing this with the graceful stop. Behavioural change: the platform now
stops in place (over a stretched decel) instead of running on to the old target — which
is more faithful to "STANDBY silences the move." Both are safe (both C2, both
rest-terminating, both gated); the new one no longer runs residual commanded motion the
operator asked to silence.

### Reversible naming/structure forks (decided, low-stakes)

`follower_input_loss_s` is a node **parameter** (0.4 s), not a `JB_TRAJ_*` config
constant — it is node-tuning, not a physical limit, and reversible. Rotation deadband
uses the rotvec-difference norm (small-angle-exact at MVP tilts ≤ 12°, far below a 0.1°
deadband). Two code commits (motion, then ROS) mirror Phase 2 for rollback/blame
granularity.

## Open questions / next steps

- **Hardware session is the gate** (deferred): gentle SpaceMouse flight at the default
  low limits; a hard-shove saturation test (expect: tracks to the workspace edge, then
  a throttled "clamped to nearest reachable" WARN, no runaway); a mid-flight unplug
  (expect: smooth graceful stop / the SpaceMouse node's ACTIVE-pose hold). PASS:
  subjectively smooth throughout, no rejects, clean disconnect. ABORT: any jerk event,
  E-STOP, runaway. The Phase-1 `tools/probes/traj_stream_probe.py` shows the streamed
  knots read-only.
- **True geometric nearest-valid clamp (deferred to the operator).** The ray-clamp
  tracks up to the boundary along the approach direction, which is safe and needs no new
  envelope. A clamp that slides *along* the workspace boundary (rather than stopping at
  the ray intersection) would feel smoother during a sustained edge shove but requires
  defining a workspace-boundary projection — a new envelope semantic worth an explicit
  operator decision, not an autonomous pick.
- **Emitter jitter under follower load** is untested in CI (timing tests are load-flaky).
  The replan is ~4–7 ms of the 25 ms budget; the hardware session's diagnostics (max
  emit gap) are the real check.
- Phase 4 (limit ramp-up + lean A/B) and Phase 5 (timed targets) build on this; the
  follower's per-tick supersede is what lifts the Phase-2 `BUSY` restriction on
  interrupting an in-flight move.

## Related

- Plan: [`plans/active/mvp-trajectory-bringup.md`](../plans/active/mvp-trajectory-bringup.md) — Phase 3 detail + the two orchestration prerequisites.
- [2026-07-07-mvp-phase2-waypoint-moves.md](2026-07-07-mvp-phase2-waypoint-moves.md) — the audit that deferred the fast gate + supersede to this phase; the `validate` gate this parallels.
- [2026-07-07-mvp-phase1-streaming-foundation.md](2026-07-07-mvp-phase1-streaming-foundation.md) — the streaming substrate + the `JB_TRAJ_SPACEMOUSE_HORIZON_S` config this reuses.
