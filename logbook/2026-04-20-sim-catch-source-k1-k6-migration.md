---
title: "Migrate sim catch sources to K1–K6 reference-feasibility contract and lower sim v_max to 500 mm/s"
type: feature
date: 2026-04-20
status: resolved
phase: "hardware-bringup — sim-side unlock for Phase 6/7"
related_plan: "hardware-bringup.md"
related_entries:
  - 2026-04-20-k1-k6-reference-feasibility-resolution
files_changed:
  - sim/main.py
commits: []
subsystem:
  - sim
  - controller
tags:
  - ref-events
  - hardware-bringup
  - testing
---

# Migrate sim catch sources to K1–K6 reference-feasibility contract and lower sim v_max to 500 mm/s

## Summary

Migrated the three sim catch sources (`CatchTargetSource`, `ThrowCatchTargetSource`,
`InteractiveCatchSource`) in `sim/main.py` to the K1–K6 reference-feasibility
contract landed in `2026-04-20-k1-k6-reference-feasibility-resolution`, and
lowered the sim MPC velocity limit from 1000 mm/s → 500 mm/s for dynamic modes.
This closes the three `TODO(W12)` punts left in sim by the prior K1–K6 entry and
makes sim-first validation of dynamic juggling meaningful: catches that would
silently saturate the MPC solver now surface as clean reference-layer rejections.

## Motivation

The `2026-04-20-k1-k6-reference-feasibility-resolution` entry landed the K1–K6
contract throughout the controller stack and migrated the hardware
`ZmqTargetSource`, but three sim catch sources were explicitly deferred with
`TODO(W12)` markers. All three called `flat_target_to_events(...)` without
`v_max_mmps` / `tau_s` kwargs — tripping the `DeprecationWarning` and skipping
K2/K3 (peak velocity / peak acceleration) enforcement at the reference boundary.
As a result, an infeasible catch quintic in sim would slip past the reference
layer unchecked and only manifest as MPC solver saturation, making sim behavior
drift from hardware behavior on the exact failure mode the contract was designed
to catch.

Separately, the `needs_high_vel` override at `sim/main.py:1324` had bumped the
sim MPC velocity limit to 1000 mm/s during the K1–K6 landing to unblock
aggressive presets. That override was never re-tuned once hardware settled at
`max_leg_vel_mmps=140` and the reference layer started enforcing peak velocity
bounds. Operator direction for the current hardware-bringup phase is "leg speed
as low as possible" with ~30 % margin over measured peak workload — which, per
the sweep in Verification, is 258 mm/s for a 1.2 m vertical throw with 300 mm
lateral travel. 500 mm/s gives ~94 % headroom over that workload while capping
everything short of the deliberately-aggressive TC2 preset (964 mm/s).

## Design

Two coupled changes to `sim/main.py`, both narrow in scope:

**1. Sim v_max lowered from 1000 → 500 mm/s for dynamic modes.**
The `needs_high_vel` override is retained but with the new ceiling. Hardware is
unchanged — `controller/params.py` still carries `max_leg_vel_mmps = 140`. The
cap is documented as scoped to sim dynamic modes only.

**2. Catch sources accept the feasibility contract at the constructor.**
`CatchTargetSource` (scripted DT*/BB* catch sequences), `ThrowCatchTargetSource`
(scripted TC* throw-catch), and `InteractiveCatchSource` (`--interactive-catch`)
now accept `v_max_mmps`, `tau_s`, and `clamp_start_twist_mmps` in their
constructors. Each source passes these kwargs to `flat_target_to_events` along
with `no_stretch=True` + `return_reason=True`, matching the catch-path policy
from `controller/REFERENCE_LAYER_CONTRACT.md`.

**Rejection handling — why "hold current pose" instead of "retain last feasible
events":** the hardware `ZmqTargetSource` caches the last feasible `ref_events`
and keeps emitting them on rejection (continues the last viable trajectory). The
sim sources don't have that caching infrastructure — they rebuild events every
tick. The minimal-surgery design here is to match the existing "coordinator
returned None" fallback pattern: `pose=current_pose`, `ref_events=None`. On
catch rejection the platform holds rather than committing to an infeasible
quintic. If the rejection is spurious and the catch becomes feasible next tick,
the coordinator re-emits and the MPC picks up from the held position; if
rejections are persistent (target genuinely impossible), the platform stays put
and the ball drops, which is the correct failure mode.

**Out of scope.** `ContinuousThrowCatchSource` / `TossLoopController` (used by
`--cycle-time` and `--juggle`) is *not* migrated. That path builds `ref_events`
directly in `TossLoopController._build_ref_events` without going through
`flat_target_to_events`, it was not flagged by the original W12 TODO, and its
peak leg velocities sit comfortably below β·v_max = 425 mm/s for all workloads
tested (worst-case 258 mm/s). Flagged as a follow-up if/when lateral spacing or
cycle times push harder.

## Implementation

Call-site plumbing at `sim/main.py`:

- `sim/main.py:1442` — `InteractiveCatchSource` construction threads
  `mpc.params.max_leg_vel_mmps`, `mpc.params.tau`,
  `mpc.params.max_ref_start_twist_mmps` from the already-built MPC instance.
- `sim/main.py:1469` — `ThrowCatchTargetSource` same plumbing.
- `sim/main.py:1472` — `CatchTargetSource` same plumbing.
- `sim/main.py:1324` — `needs_high_vel` override ceiling changed from 1000 → 500.

Helper `_warn_on_catch_rejection(reason, sim_time, source)` added at
`sim/main.py:~336` — logs at most once per second per source to avoid console
spam during rejection bursts (DT3's natural rejection window is ~225 ms of
persistent rejections as the catch window closes).

All three `TODO(W12)` markers in `sim/main.py` were removed.

## Verification

### Unit + integration tests

`pytest tests/ -v` → **1011 passed, 52 warnings, 0 failed**.

Prior baseline from the K1–K6 landing entry was 1009; +2 represents two tests
added elsewhere since that baseline. Zero regressions introduced by this
migration.

### Preset sweep — 22 presets

Measured peak leg velocity (mm/s) at 40 Hz control rate, steady-state (excluding
first 1 s cold-start), headless `--no-viewer`.

**Pre-migration, v_max=1000:**
- PhaseA (vertical 1.2 m): 18 steady (cold-start 1000)
- PhaseB (1.2 m / 300 mm lateral): 258 steady (cold-start 1000)
- TC1, TC3, TC4: 98 steady
- TC2 (400 mm / 1.15 s flight — aggressive): 964 steady
- DT1–DT8: 0–276 (DT4/6/8 are deliberate-failure fixtures at 0)
- BB1–BB4: 129–229
- T1–T6: 82–201

**Post-migration, v_max=500:**
- PhaseA, PhaseB: unchanged (cold-start now clipped to 500)
- TC1, TC3, TC4, DT1/2/5/7, BB1/2/4, T1–T6: unchanged (all below rate limit)
- TC2: rate-limited at 500 (previously 964 — explicitly out of scope at current
  speed budget)
- DT3, BB3: brief 3–4 tick saturation transients at 500 during post-rejection
  re-engagement (K1–K6 correctly rejects the aggressive catch quintic during
  the final ~200 ms of its catch window; when feasibility recovers, the MPC
  tracks the recovered reference at the rate limit). Catches still succeed with
  `pos_err = 0.0 mm`.

### Tossing smoke tests

- **Phase A** 1.2 m (cycle_time = 1.65 s, vertical only): 8/8 catches, 0 drops,
  final tracking 0.2 mm, solve p95 9.6 ms.
- **Phase B** 1.2 m with 300 mm lateral: 8/8 catches, 0 drops, final tracking
  1.0 mm (rotation 3.4°), solve p95 23.6 ms.

### Rejection path

DT3 (deliberately-aggressive catch): 2 dedup'd warnings printed (first in each
rejection burst), catch succeeds at `t=0.950 s` with `pos_err = 0.0 mm`. No
solver saturation, no E-stop.

## Outcome

Sim is now K1–K6-compliant end-to-end for the catch path. The reference layer
enforces peak velocity and acceleration bounds for every catch target in sim,
matching the hardware behaviour landed on 2026-04-20 commit `e8c5833`.
Sim-first validation of 1.2 m throw-catch sequences is now meaningful: a catch
that saturates the MPC solver in sim (previously silent due to unenforced
K2/K3) will now surface a clean reference-layer rejection.

The sim `v_max = 500 mm/s` cap reflects the "careful, as low as possible"
operator direction for the current hardware-bringup phase. The cap is scoped to
sim dynamic modes only — hardware remains at 140 mm/s per
`controller/params.py`.

This unlocks sim-first validation for Phase 6/7 dynamic-juggling readiness under
the hardware-bringup plan.

## Open Questions

- **TC2 is at the rate limit** at `v_max=500`. Keep as a known out-of-scope
  regression; revisit when the speed envelope is raised toward 1000 mm/s again.
- **`ContinuousThrowCatchSource` / `TossLoopController` path not migrated** —
  builds `ref_events` directly, bypassing `flat_target_to_events`. Current
  workloads (Phase A/B up to 1.2 m throws with 300 mm lateral) stay under
  β·v_max = 425 mm/s so the unmigrated path hasn't bitten yet. Flag for
  migration if/when lateral spacing or cycle times push harder.
- **Hardware is unchanged.** This migration touches only `sim/main.py`.
  Hardware's `ZmqTargetSource` was migrated in
  `2026-04-20-k1-k6-reference-feasibility-resolution`.
