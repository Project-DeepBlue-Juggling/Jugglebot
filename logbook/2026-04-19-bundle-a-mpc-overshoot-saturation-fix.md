---
title: "Bundle A — ref-from-plant-state on move transitions (MPC overshoot-saturation fix)"
type: feature
date: 2026-04-19
status: in-progress
phase: "hardware-bringup — Phase 4 (moderate motions)"
related_plan: "hardware-bringup.md"
related_issues:
  - MPC_OVERSHOOT_SATURATION
related_entries:
  - 2026-04-18-move5-overshoot-stall-and-plant-collapse
sessions: []
files_changed:
  - controller/target.py
  - controller/zmq_target.py
  - run_mpc.py
  - sim/main.py
  - tests/sim/test_target_interface.py
  - tests/sim/test_zmq_target.py
commits: []
subsystem:
  - controller
  - mpc
tags:
  - mpc
  - ref-events
  - overshoot
  - IPC
  - hardware-bringup
  - testing
---

# Bundle A — ref-from-plant-state on move transitions (MPC overshoot-saturation fix)

## Summary

On a new target arriving via ROS2 → `mpc_bridge_node` → ZMQ :5558, `ZmqTargetSource.update()` returned `TargetCommand(ref_events=None)`, causing the MPC loop to fall back to a flat-hold reference (`ref_traj[:] = target_pose`, zero twist). When the plant was carrying residual motion, IPOPT was asked to unwind the overshoot against a stationary ref target, stalling at `ipopt_iter == 0` within the 22 ms CPU cap — the Move-5 signature from [2026-04-18-move5-overshoot-stall-and-plant-collapse](2026-04-18-move5-overshoot-stall-and-plant-collapse.md). This entry lands the performance half (the original "Fix (b)" / "Bundle A" in that investigation): the ZMQ target source now synthesises a warm-up `ReferenceEvent` at `t = t_now` anchored at the live plant pose and (clamped) twist on every distinct new target, so the NLP is never asked to reverse the plant against an accelerating ref. Bundle B (diagnose-only detectors for `MPC_OVERSHOOT_SATURATION` + `PLANT_TELEMETRY_COLLAPSE`) already landed in commit `0dbb4fa`.

## Motivation

From the 2026-04-18 Move-5 entry:

> Move 5 starts from `(8.57, 34.0, 169.3)` carrying residual state from Move 4 — not from Active. Over the first ~15 steps the plant accelerates on the prior cmd, reaching `actual_z ≈ 193 mm` by t=0.74 s. At that instant `ref_z` is only 173 mm (still accelerating from 170 → 220 at 60 mm/s). Plant is **20 mm AHEAD of ref**. [...] IPOPT hits the 24 ms CPU cap without completing a single iteration.

That geometry is exclusive to the ZMQ production path: `StaticTargetSource` and `WaypointTargetSource` already emit a `ReferenceEvent` anchored at the live plant state on first invocation, so sim runs never see the failure mode. The ZMQ path had the opposite behaviour — `ref_events=None` → MPC loop falls back to `ref_traj[:] = target_pose`, zero twist. The flat-hold ref is only correct when the plant is already at rest near the target; in a ROS2 production scenario (spacemouse, trajectory planner, etc.) incoming targets routinely arrive while the plant is still carrying velocity from the previous command, and every one of those transitions re-triggered the Move-5 stall geometry.

The `MPC_OVERSHOOT_SATURATION` entry in `sim/analysis/known_issues.yaml` identifies the ref source — not the solver — as the fix target:

> Fix target is the ref source, not the solver — prepend a warm-up Hermite segment from (current plant pose, current twist) on move transitions so the NLP is never asked to reverse the plant against an accelerating ref.

This is Bundle A.

## Design

Walk-through of one MPC cycle immediately after a new distinct target arrives on :5558:

1. `ZmqTargetSource.poll()` ingests the target. Internal change-detection (`np.array_equal` on pose/arrival_time/twist) sets `_target_dirty = True` only when the ingested target differs from the cached one. Identical retransmissions (spacemouse streams, etc.) leave the cache intact so IPOPT warm-start is preserved.

2. `ZmqTargetSource.update(sim_time, state)` sees the dirty flag, reads the live plant pose (from `state.q_platform` via FK) and platform twist (`state.platform_twist`), and calls `flat_target_to_events(current_pose, current_twist, target_pose, t=sim_time, target_twist=…, arrival_time=…, clamp_start_twist_mmps=v_max_mmps)`. The result is cached and returned in `TargetCommand.ref_events`.

3. The MPC loop's ref builder sees `ref_events` is non-empty and builds a quintic Hermite segment from `(live plant pose, live plant twist)` at `t = t_now` → `(target pose, target twist)` at `t = arrival_time`. `ref[0]` now equals the plant pose, and `twist_ref[0]` equals the (clamped) plant twist.

4. IPOPT sees `p[0] ≈ p_ref[0]` and `v[0] ≈ v_ref[0]`. The first iteration is tractable — no 20 mm reversal against an accelerating ref. The Move-5 saturation geometry disappears.

On subsequent ticks within the same target, `poll()` sees no change and `_target_dirty` stays false. `update()` returns the cached events — the ref trajectory is stable and the solver's warm-start remains valid.

The twist clamp (`clamp_start_twist_mmps = MPCParams.max_leg_vel_mmps`) guards against a noisy FK-derived twist spike becoming the quintic start-velocity boundary condition. Applied element-wise to the linear components (indices 0..2) only; angular components (3..5) pass through unclamped since they scale differently. Clamping limits the worst-case twist magnitude the warm-up segment can carry while preserving direction-of-motion information.

## Implementation

### `controller/target.py`

- `flat_target_to_events(current_pose, current_twist, target_pose, t, *, target_twist=None, arrival_time=None, clamp_start_twist_mmps=None)` — new kwarg. When set, clips linear components (indices 0..2) of `current_twist` element-wise to ±that value. Angular components untouched.
- Defensively copies the input twist before clamping so the caller's array is never mutated.
- `StaticTargetSource` and `WaypointTargetSource` gained an optional `clamp_start_twist_mmps` constructor kwarg that threads through to `flat_target_to_events()` on every call.

### `controller/zmq_target.py`

- `ZmqTargetSource.__init__` gained `v_max_mmps: float | None = None`. Callers pass `MPCParams.max_leg_vel_mmps`.
- New state: `_cached_events: list[ReferenceEvent] | None` and `_target_dirty: bool`.
- `poll()` — change-detection via `np.array_equal` on pose, arrival_time, and target_twist against the cached target. Flag set only when the ingested target is genuinely distinct. Identical retransmissions do not thrash the cache.
- `update(sim_time, state)` — when the flag is set (or cache is empty and a target is available), re-emits `ref_events` via `flat_target_to_events(pose_from_state, state.platform_twist, target, sim_time, target_twist=…, arrival_time=…, clamp_start_twist_mmps=self._v_max_mmps)` and caches. The returned `TargetCommand` always carries `ref_events=self._cached_events`.
- Cache cleared on mode → disabled and in `reset()`.

### `run_mpc.py`

- Both the `StaticTargetSource` path (`--pose`, `--sequence`) and the `ZmqTargetSource` production path now receive `clamp_start_twist_mmps=mpc.params.max_leg_vel_mmps` (resp. `v_max_mmps=…`).

### `sim/main.py`

- The `--hardware` sim-side entry point wires `clamp_start_twist_mmps` / `v_max_mmps` identically to `run_mpc.py` so the sim-plant and hardware paths stay behaviourally equivalent under the ZMQ target source.

## Verification

`pytest tests/ -v`: **959 passed, 51 warnings in 213.91 s**. Baseline was 935 (the previous day's run in the MPC-overhead-spikes entry) → +24 new tests, zero pre-existing regressions. Warnings are the pre-existing `return True` style in `tests/motion/test_motor_guard.py` flagged in the 2026-04-18 overhead-spikes entry — not introduced here.

### New test coverage

**`tests/sim/test_target_interface.py` — `TestFlatTargetTwistClamp` (5 cases):**

- noisy linear twist (±300 mm/s) clipped to ±v_max
- in-band linear twist (±50 mm/s) untouched
- angular components pass through unclamped for all magnitudes
- `clamp_start_twist_mmps=None` disables clamping
- clamp applied internally on a defensive copy — caller's twist array is not mutated

**`tests/sim/test_zmq_target.py` (NEW FILE, 9 cases):**

- `ref_events` is `None` before first target is published
- first target → events anchored at live plant pose+twist; `ref[0] ≈ plant_pose`
- cache reused across identical target retransmissions (warm-start preserved)
- cache rebuilt with fresh plant state on a distinct target
- cache cleared on mode → disabled
- cache cleared in `reset()`
- FK-derived twist spike (e.g. ±400 mm/s) clipped to v_max in the cached events
- `v_max_mmps=None` disables clamping end-to-end
- identical retransmissions do not thrash `_target_dirty`

**`TestOvershootSaturationRegression.test_ref_aligns_with_plant_at_first_solve`** — reproduces the Move-5 geometry in sim: off-Active plant pose `(8.57, 34.0, -0.7)` mm + spoofed +80 mm/s upward platform twist, new target at `z = 50` mm, assert `ref[0]` equals the plant pose and `ref_twist[0]` carries the clamped +80 mm/s. The live hardware CSV `mpc_20260418_015112.csv` is a recording and cannot be replayed against the fix; the regression test reproduces the *geometry* (off-Active pose + residual upward twist + new target) in sim instead.

`MpcTargetIPC` is mocked via `unittest.mock.patch` at the `controller.zmq_target` import site so no ZMQ socket is opened during test.

### Hardware status

Today's (2026-04-19) sessions at 13:49–13:52 — nine runs, `temp/logs/mpc_20260419_13*.csv` — show tracking RMS climbing 0.52 mm → 10.9 mm across the session with `max_consecutive_timeout=5` on the later runs. The trend is directionally consistent with the Bundle A mechanism (off-Active targets + residual twist on move transitions). These sessions predate the fix and do NOT cross the `overshoot_saturation` detector threshold (≥10 consecutive `ipopt_iter==0` steps) — they sit under the mid-severity bar by design. A fresh hardware rerun against the fix is the gating validation step and has not happened yet.

## Outcome

Unit + integration tests green at 959/959. Bundle A is not yet validated on hardware. Status remains `in-progress` until the next hardware session produces a CSV with an off-Active target transition where the `overshoot_saturation` detector does *not* fire. Once verified, this entry closes and `MPC_OVERSHOOT_SATURATION` in `sim/analysis/known_issues.yaml` moves from `active` → `resolved`.

The 2026-04-19 13:49–13:52 sessions will not, on their own, confirm or deny the fix — the tracking degradation they show sits below the `overshoot_saturation` detector threshold (10-consecutive-iter=0), which is a deliberate mid-severity bar. A rerun with a clean off-Active transition (Move-5-style geometry) is required.

## Open Questions

- Bundle B1 (`HardwarePlant` `estop_requested` flag + persistent-stale-contents detection) remains open from the 2026-04-18 Move-5 entry. It is the safety half of the Move-5 decomposition and is independent of this fix.
- Should `WaypointTargetSource` adopt the same change-detection-plus-cache structure as `ZmqTargetSource` for consistency, or is its current "emit fresh events on every `update()`" acceptable given that waypoint sources are driven by sim time rather than external IPC?
- The twist clamp is applied uniformly at `MPCParams.max_leg_vel_mmps`. Should it instead be derived from the Jacobian-mapped linear-velocity envelope at the current pose, which would permit faster warm-up segments near the workspace centre where leg velocities are less kinematically restrictive?
