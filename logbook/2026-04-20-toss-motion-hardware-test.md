---
title: "Add `--toss-motion` hardware test and `controller/ballistics` helpers"
type: feature
date: 2026-04-20
status: resolved
phase: "hardware-bringup — platform-only dynamic-motion entry point"
related_plan: "hardware-bringup.md"
related_entries:
  - 2026-04-20-sim-catch-source-k1-k6-migration
  - 2026-04-20-k1-k6-reference-feasibility-resolution
files_changed:
  - controller/__init__.py
  - controller/ballistics.py
  - controller/runner.py
  - controller/toss_motion_source.py
  - run_mpc.py
  - sim/hand/ballistics.py
  - tests/sim/test_toss_geometry.py
commits:
  - d09ed7d
  - 111dea7
subsystem:
  - controller
tags:
  - hardware-bringup
  - ref-events
  - testing
---

# Add `--toss-motion` hardware test and `controller/ballistics` helpers

## Summary

Adds a platform-only back-and-forth ballistic test that runs on hardware by
default (`--toss-motion`) or against `MuJoCoPlant` via `--sim`. Each cycle
computes 6-DOF throw and catch platform poses from the platform's live pose at
cycle start and a user-specified catch position, so chained cycles naturally
become "throw-from-previous-catch". Hand/ball-independent ballistics are lifted
out of `sim/hand/planner.py` into a new hardware-safe `controller/ballistics.py`
module; targets are emitted through the K1–K6 reference-feasibility contract so
peak velocity / acceleration bounds are enforced at the reference layer. The
feature is landed; the first hardware exercise is still pending.

## Motivation

After the sim-side K1–K6 migration
(`2026-04-20-sim-catch-source-k1-k6-migration`), sim is ready for reliable
tossing experiments but there was no hardware entry point for "move the
platform on a throw–catch trajectory without committing to a real catch". The
existing hardware paths each miss the requirement:

- `--pose` — single static target, no chaining.
- `--sequence` — explicit time-stamped poses, no ballistic geometry.
- ZMQ targets — waits for `mpc_bridge_node` to drive.

None of these lets an operator say "throw from here to (50, 0, 170) at a 1.2 m
apex, then ping-pong back — but without the ball or hand". That is the gap this
feature closes, and it is the natural next step on the `hardware-bringup` plan
before hand and ball are reintroduced.

Operator-facing design goals (from discussion with the user):

1. **Throw pose = current platform pose.** The CLI only specifies the *catch*
   position; the throw position is read live from `state.platform_pos_mm`.
   Chained throws fall out for free (catch N = throw N+1).
2. **No explicit tilt.** Platform orientation at throw/catch is derived from
   the ballistics (platform +Z aligned with launch / anti-arrival velocity).
3. **Non-planar throws supported from day 1.** The helper solves the full
   ballistic quadratic so a catch at a different Z from the throw works
   correctly (new `flight_time_from_apex`).
4. **Hardware default.** The hardware path is the default entry; `--sim` is
   the opt-in dry-run.

## Design

### Architecture

Three layers, separated by hardware-safety:

**1. `controller/ballistics.py`** (new, numpy-only, hardware-safe)

- `compute_launch_velocity(throw, catch, flight_time) → v_launch`
- `compute_arrival_velocity(v_launch, flight_time) → v_arrival`
- `flight_time_from_apex(throw, catch, apex_mm) → t` — solves
  ½g·t² − v_z·t + Δz = 0 for the descent root; guards against
  catch-above-apex and non-positive apex.
- `compute_orientation(direction) → rotvec` — platform +Z aligned with the
  direction; raises if tilt > 30°.
- `rodrigues(rotvec) → 3×3 rotmat`.

**2. `controller/toss_motion_source.py`** (new) —
`TossMotionSource(catch_positions, apex_mm, hold_s, v_max_mmps, tau_s, clamp_start_twist_mmps)`.

- Implements the `TargetSource` protocol; threads K1–K6 params through
  `flat_target_to_events` on every emitted reference.
- `APPROACH → TRANSIT → HOLD → DONE` state machine, per cycle.
- Cycle start reads `state.platform_pos_mm` as the throw position — this is
  what makes "throw-from-previous-catch" work without any state-passing
  between cycles.
- `None` entries in the catch list are sentinels resolved to the platform's
  pose on the very first `update()` — used for ping-pong via
  `--catch-pose X,Y,Z --cycles N`.
- The `APPROACH → TRANSIT` gate requires `pos_err < 2 mm`, `rot_err < 0.02 rad`,
  **and both** linear velocity `< 10 mm/s` and angular velocity `< 0.05 rad/s`
  so the TRANSIT quintic's zero start-twist boundary is physically accurate.
- Ref-events cache invalidates on pose OR phase change — required because
  TRANSIT (arrival_time = deadline) and HOLD (arrival_time = None) can share
  the same target pose.
- Docstrings explicitly flag that the transit is **timing-matched but not
  path-matched**: the platform rides the min-jerk quintic for `flight_time`
  seconds while the notional ball traces a parabola.

**3. `run_mpc.py` CLI wiring.**

- New flags: `--toss-motion`, `--catch-pose X,Y,Z`,
  `--sequence-catches "x1,y1,z1 x2,y2,z2 ..."`, `--cycles N`, `--apex 1.2`,
  `--hold-s 1.0`, `--sim`.
- `--sim` swaps `HardwarePlant` for `MuJoCoPlant` and skips the ROS2-side
  hooks (`TargetFeedbackPub`, `SessionMetadataPush`); the sim path warms the
  MuJoCo plant up to ACTIVE (z=170) before handing off so the first
  `TossMotionSource` cycle starts from the hardware precondition.
- Every hardware-only plant method (`set_pose`, `enable`, `disable`, `close`)
  is `hasattr`-guarded so `MuJoCoPlant` dry-runs don't error.
- Pre-flight speed warning: estimates worst-case TRANSIT horizontal speed
  (`max_horizontal / flight_time_from_apex`) and prints a WARN if it exceeds
  `0.85·v_max`, so the operator knows the K1–K6 layer will silently stretch
  the quintic past the ballistic deadline before any physical motion happens.

### Why we didn't use `sim/hand/planner.py::ThrowCatchPlanner` directly

The existing sim-side planner already computes throw/catch poses from
throw/catch positions (same ballistics math). Importing it is technically
hardware-safe (numpy + dataclass, no MuJoCo). Rejected because:

- The planner also enforces hand-speed limits (7 m/s ejection cap) and
  hand-sequence transit-time budgets that are irrelevant for platform-only
  tests and would raise `ValueError` for configurations we consider feasible.
- The planner applies a hand-release-point offset to the platform centroid,
  coupling the pose output to a hand-position assumption we aren't honouring.

Instead, the pure-math helpers were extracted to `controller/ballistics.py`
(clean hardware layer) with `sim/hand/ballistics.py` kept as a compat shim
re-exporting the moved symbols — zero sim-side breakage.

### Safety envelope

Hardware-side `max_leg_vel_mmps = 140 mm/s` (unchanged). From the measured
Jacobian at active pose:

- Platform `vx` → max leg-rate gain 0.37 (X direction most aligned leg).
- Rotational gain ~200 mm/s per rad/s (costly — rotations eat the budget fast).

For a 1.2 m apex / same-height throw with lateral `L`, the combined peak leg
velocity (translation + rotation at quintic midpoint):

- `L = 50 mm`: 43 mm/s — 2.8× margin below `β·v_max = 119`.
- `L = 100 mm`: 86 mm/s — 1.4× margin (at the 30%-safety-rule limit).
- `L = 125 mm`: 108 mm/s — tight, ~10% margin.
- `L = 150 mm`: 129 mm/s — OVER budget; K1–K6 stretches.

Recommended first-hardware ramp: `L = 50 → 75 → 100`. Beyond ~100–125 mm,
hardware `max_leg_vel_mmps` needs to be raised.

## Implementation

See commit `d09ed7d` for the full diff. Key implementation points beyond what
the Design section covers:

- **`flight_time_from_apex`** returns the descent root
  `(v_z + √(v_z² − 2g·Δz)) / g`. When the catch is at exactly the apex height
  (Δz = apex), the discriminant is zero and a single root degenerates to
  `v_z / g`. Asserted by `test_flight_time_catch_at_apex_is_single_root`.
- **Ping-pong via `None` sentinels.** `--catch-pose X,Y,Z --cycles 3` expands
  to `[X, None, X]` — odd cycles return to the captured start pose. The start
  pose is captured once on the first `update()` and reused for every `None`
  thereafter.
- **Cold-start hook in the sim path.** `plant.reset(active_pose)` sets the
  actuator setpoints; `plant.command(active_ext) + plant.step(2.0)` lets the
  joints physically converge to ACTIVE over 2 sim-seconds before the MPC loop
  hands over.

## Verification

### Unit + integration tests

`pytest tests/ -q` → **1033 passed, 52 warnings, 0 failed**. Baseline
pre-commit was 1011 (the K1–K6 sim-migration baseline); +22 new tests in
`tests/sim/test_toss_geometry.py`:

- 6 tests — `flight_time_from_apex` correctness (symmetric throw, Δz > 0,
  Δz < 0, catch at apex, catch-above-apex raises, non-positive apex raises).
- 2 tests — launch/arrival round-trip (symmetric + fully non-planar diagonal
  with Δz ≠ 0).
- 6 tests — orientation geometry (+Z identity, forward-tilt axis direction,
  tilt-limit boundary, zero-vector rejection, Z-alignment recovery, rodrigues
  spot-checks).
- 2 tests — rodrigues identity + quarter-turn Z.
- 1 test — `sim/hand/ballistics.py` re-exports are literally the same
  callables.
- 5 tests — full `TossMotionSource` state machine: first-update anchors
  `ref_events` at `sim_time`; `None` sentinel resolves to initial pose;
  APPROACH does not advance with live angular velocity; full
  `APPROACH → TRANSIT → HOLD → DONE` cycle; phase-transition invalidates
  `ref_events` cache.

### Sim dry-runs (`--sim --no-viewer` on Jetson)

| scenario | stretch? | tracking | solve p95 |
|---|---|---|---|
| L=50 mm, 3 cycles | — | 0.26 mm | 7 ms |
| L=100 mm, 3 cycles | 62% (0.99→1.6 s) | 0.47 mm | 7 ms |
| L=150 mm, 3 cycles | 143% (0.99→2.4 s) | 0.71 mm | 7 ms |
| L=50 + Δz=+50 (non-planar) | — | 0.27 mm | 7 ms |
| L=50 diagonal (35,35,170) | — | 0.28 mm | 7 ms |
| 4-corner sequence | — | 0.13 mm | 7 ms |

K1–K6 stretching fires at `L ≥ 100 mm` as designed (the reference layer
catches infeasibility before IPOPT does). The `L = 150` test also triggered
the new pre-flight warning:

> `WARN: estimated TRANSIT horizontal speed 152 mm/s exceeds 0.85·v_max
> (119 mm/s). K1–K6 will stretch the quintic past the 0.99s ballistic
> deadline; observed transit will run longer than the notional ball's flight
> time.`

### Audit + fixes

Self-audit via `/audit --unstaged` before commit flagged 3 WARNINGs and
5 NOTEs; all 9 applied before the commit landed. Highlights:

- `APPROACH → TRANSIT` gate now checks angular velocity, not just linear.
- Pre-flight speed warning added (prevents operator surprise at v_max=140).
- Ref-events cache now invalidates on phase boundary (TRANSIT → HOLD).
- `pose_6dof_from_state` de-duplicated (imported from `controller.target`).
- `TossMotionSource` re-exported from `controller/__init__.py`.
- Docstrings tightened to distinguish timing-match from path-match.
- Stale `import numpy` cleared from the compat shim.
- Anti-parallel branch in `compute_orientation` annotated as defensive.

## Outcome

Hardware entry point for ballistic platform-only tests is live. Operator can
now run:

```
python run_mpc.py --toss-motion --catch-pose 50,0,170 --cycles 1 --apex 1.2
```

and see the platform make one throw-to-(50,0,170) motion from wherever it is
(expected: started at ACTIVE), tilting to the correct throw/catch orientations
without any hand or ball. Ping-pong, off-axis catches, and chained sequences
via `--sequence-catches` work in sim.

## Updates

### 2026-04-22 — source-driven termination; `--duration` ignored for `--toss-motion` (commit `111dea7`)

First hardware run on 2026-04-22 (`--catch-pose 50,0,170 --cycles 1 --duration 8`)
exposed an ergonomics issue with the wall-clock `--duration` flag:

- A too-short `--duration` would hard-stop the MPC loop mid-cycle (not graceful).
- A too-generous `--duration` left the platform holding at the final catch for
  longer than necessary.
- Neither matches the operator's intent, which is "run the motion you described".
  For a finite-cycle toss plan the motion has a well-defined end.

Fix: the source is now the termination signal, not a wall clock.

- **`controller/runner.py`** polls `getattr(source, 'done', False)` each tick
  and exits the loop cleanly on True. Sources without `.done` (ZMQ, static,
  waypoint) are unaffected because the default is `False`.
- **`controller/toss_motion_source.py`** has a new `final_hold_s` kwarg
  (default `0.0`). On the final cycle's HOLD the hold duration becomes
  `hold_s + final_hold_s` before `.done` flips — gives the operator an
  observation window at the final catch pose before the runner exits.
- **`run_mpc.py`** adds `--final-hold-s` (default `2.0 s`). For
  `--toss-motion`, `--duration` is ignored (prints a one-line NOTE if
  supplied) and the auto runtime is derived from the plan:
  `N × (0.5 s approach + flight_time + hold_s) + final_hold_s + 2 s pad`.
  The pad is a safety ceiling; the loop actually exits via `source.done`.

Verification: `pytest tests/ -q` → 1033 passed, zero regressions. Sim
dry-run with `--duration 900` against a 3-cycle plan prints the ignored-duration
NOTE, computes an 11.5 s ceiling, and the runner logs
`source signalled done — exiting cleanly` at ~8 s of actual motion.

## Open Questions

1. **First hardware run done (2026-04-22, `L = 50 mm`, `apex = 1.2 m`);
   "stepped command" observation pending investigation.** Motion itself was
   smooth but the commanded position was non-monotonic when zoomed out — small
   step-like features that aren't present in the sim traces. Candidates: (a)
   phase-boundary ref-events cache rebuilds emitting a fresh quintic from the
   slightly-different live pose; (b) hardware-side hysteresis / cogging not
   modelled in sim. `/investigate` on the session CSV (2026-04-22) is in
   flight. Ramp is on hold at `L = 50 mm` until that is characterised; next
   steps are `L = 75 → 100 mm` once clean.
2. **`ContinuousThrowCatchSource` / `TossLoopController` still bypasses
   `flat_target_to_events`.** These sim-side paths build `ref_events` directly
   and are not part of the K1–K6 migration or this feature. Not urgent at
   current workloads (Phase A/B up to 1.2 m throws stay under
   `β·v_max = 425 mm/s`) but worth tracking.
3. **Hand centroid offset is ignored for platform-only.** Correct behaviour
   for this feature, but when the hand is reintroduced the throw/catch
   positions will need to be offset by
   `compute_hand_offset_mm(release_pos) * platform_z` so the *hand release
   point* lands at the throw_pos, not the platform centroid. The sim planner
   already does this — porting that offset into the hardware path is a
   follow-up.
4. **Non-planar catches change arrival velocity.** `compute_arrival_velocity`
   already handles this correctly for ballistics, but at catch time the
   platform only tracks the *orientation* matching the arrival velocity — it
   doesn't impart any relative velocity. Fine for no-ball tests; will need an
   `arrival_twist` hook when the ball comes back.
