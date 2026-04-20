---
title: "Bundle A quintic-ref settling lag + ZMQ live-twist feedback trap"
type: investigation
date: 2026-04-19
status: resolved
phase: "hardware-bringup — Phase 4 (moderate motions)"
related_plan: "hardware-bringup.md"
related_entries:
  - 2026-04-20-k1-k6-reference-feasibility-resolution
  - 2026-04-19-bundle-a-mpc-overshoot-saturation-fix
  - 2026-04-19-leg1-pose-dependent-hold-twitch
  - 2026-04-18-move5-overshoot-stall-and-plant-collapse
  - 2026-04-18-mpc-overhead-spikes-fallback-bursts
sessions:
  - mpc_20260419_223727.csv
  - mpc_20260419_223756.csv
  - mpc_20260419_223902.csv
  - mpc_20260419_224454.csv
rosbag:
  - /home/jetson/Desktop/rosbags/2026-04-19_22-36-46
  - /home/jetson/Desktop/rosbags/2026-04-19_22-44-29
files_changed:
  - controller/feasibility.py
  - controller/mpc.py
  - controller/params.py
  - controller/runner.py
  - controller/scheduler.py
  - controller/target.py
  - controller/zmq_target.py
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py
  - run_mpc.py
  - sim/analysis/record_baselines.py
  - sim/main.py
  - tests/sim/test_cold_start_fixes.py
  - tests/sim/test_make_feasible_events.py
  - tests/sim/test_mpc_adversarial_sequences.py
  - tests/sim/test_target_interface.py
  - tests/sim/test_zmq_target.py
commits:
  - e8c5833
  - 831ab6f
subsystem:
  - controller
  - mpc
tags:
  - mpc
  - ref-events
  - quintic
  - overshoot
  - settling
  - hardware-bringup
  - tracking
---

# Bundle A quintic-ref settling lag + ZMQ live-twist feedback trap

## Summary

User reported both most-recent hardware sessions (2026-04-19 22:36 and 22:44) exhibited more oscillation / jerkiness than prior runs. Data analysis shows two distinct mechanisms, both introduced by the same-day uncommitted Bundle A change to `controller/zmq_target.py` and the quintic ref path in `controller/target.py` / `controller/mpc.py`:

1. **Quintic-ref settling lag (both sessions)**: replacing the old flat-hold ref (`ref_events=None`) with a 2-event quintic (`flat_target_to_events` from current plant pose+twist to target over ~0.5 s at `cruise_speed=80 mm/s`) doubles settling time on short moves (4-5 s vs 2 s for a 50 mm z move, head-to-head comparable setups) and degenerates catastrophically on larger moves (25-50 s to converge a 20-50 mm step) because the plant can't match the quintic's peak velocity, IPOPT hits its 22 ms cap while plant is far off-ref, fallback extrapolation does no ref correction, and the cycle repeats. In 22:44 (`mpc_20260419_224454.csv`), 1707 of 6954 solves (~25 %) were `hold_extrap(Maximum_CpuTime_Exceeded)` during what should have been short transitions.

2. **Live-twist feedback trap (223902 only)**: when a ZMQ sender streams targets with a refreshed `arrival_time` every tick (stream-style spacemouse/orchestrator behaviour), `ZmqTargetSource._target_dirty` becomes true every tick (line 184-189), the ref cache rebuilds with `current_twist = state.platform_twist`, and if IPOPT is in fallback/hold_extrap the plant twist is already a runaway value (~130 mm/s) from prior cmd extrapolation. The `clamp_start_twist_mmps = v_max = 140 mm/s` is a no-op because the runaway speed sits below the clamp. The newly-anchored quintic starts at 130 mm/s every tick, perpetuating the runaway. In `mpc_20260419_223902.csv` this drove actual_z to 244 mm against a ref_z of 210 mm, tripped `PLANT_TELEMETRY_COLLAPSE` (Bundle B detector, already landed), and triggered the operator E-stop. The 318 mm/s overspeed is the signature.

Steady-state hold performance is NOT regressed — per-leg act_ext stdev at stable hold (last 5 s) in 22:37 run 2 is 5-11 µm (leg 1 actually improved 10.8 → 4.9 µm thanks to the uncommitted leg-1 gain revert 30→40 in `config/hardware_config.yaml`). The "more jerky" perception traces to longer in-move/settling dynamics, not hold chatter.

## Symptoms

- User reported both most-recent hardware sessions exhibited more oscillation / jerkiness than prior good sessions.
- `mpc_20260419_224454.csv` (22:44, 207 s, `--auto-leg1-test`): pose 4→5 transition (20 mm retreat, should take ~1 s) took 28 s to converge; pose 5→6 (50 mm climb) took 50 s. 1241 consecutive IPOPT solve-budget violations. Worst motion-onset dead-time 1871 ms at t=126 s. Solve mean 16.0 ms p95 26.5 ms max 79.1 ms. Final tracking tail err 14.7 mm.
- `mpc_20260419_223902.csv` (22:38, truncated 4 s): uncommanded overspeed 318 mm/s peak at t=3.1 s (2.3× v_max=140), zero-extension sentinel from t=3.47 s, pilot E-stop with DC_BUS_UNDER_VOLTAGE (err 512) across all 7 motors. Actual_z overshot to 244 mm against ref_z max 210 mm. Solver saturated at ipopt_iter ≈ 0 from the first tick.
- `mpc_20260419_223727.csv` and `mpc_20260419_223756.csv` (22:37, 20 s holds at z=170-220): each clean, tail tracking 0.13 and 0.06 mm respectively, solve mean 10.8-10.9 ms — these establish that the baseline still works when the ZMQ sender is not stream-refreshing.
- Comparison to pre-Bundle-A baseline `mpc_20260419_104332.csv` (10:43, same 170→220 move, ref_events=None flat-hold): settled to <0.1 mm in ~2 s with 0.13 mm small overshoot. 22:37 run 1 with Bundle A quintic ref: settled to <0.1 mm in ~4-5 s with monotonic undershoot — 2× longer settle. The Bundle A quintic trades overshoot for settling time.

## Diagnosis

### Mechanism 1: Quintic-ref settling lag

The Bundle A ref source emits a 2-event quintic via `flat_target_to_events` with default `cruise_speed=80 mm/s` and `response_time=0.3 s`. For a 50 mm z move, `t_arrival = t_now + max(0.3, 50/80) = t_now + 0.625 s`. Peak ref velocity is ~1.875 × average ≈ 150 mm/s — the quintic's cruise phase, right at v_max. During the ramp the MPC tries to track a reference that's accelerating faster than the plant physically can (ODrive bandwidth + mechanical inertia), so the plant lags. By t_arrival the plant is still 5-10 mm behind. After t_arrival the ref pins at (target, 0); the MPC closes the residual gap on closed-loop feedback, which is slow because the horizon is only 400 ms (N=10, τ=40 ms) — too short to plan an aggressive catch-up with the current cost weights. On larger moves (22:44), the plant-ref gap grows large enough that IPOPT can no longer solve within 22 ms, walk-forward fallback takes over, and because fallback does not re-solve the NLP against the *current* plant-ref mismatch it makes no progress closing the gap. Result: 25-50 s convergence on what should be 0.5 s moves.

The pre-Bundle-A behaviour (ref_events=None → MPC fell back to `ref_traj[:] = target_pose, zero twist`) was effectively an infinite-bandwidth step reference. The MPC chased the target as fast as ODrive+plant allowed, with a small overshoot. Faster convergence, slightly noisier endpoint. Bundle A intentionally inverted that tradeoff; the problem is the default `cruise_speed=80 mm/s` is conservative for Phase-4 hardware that is already velocity-constrained at 140 mm/s per leg.

### Mechanism 2: Live-twist feedback trap in ZmqTargetSource

Walk through one MPC cycle during the 223902 event:

1. Prior tick: IPOPT saturated at iter=0, cmd extrapolated at +130 mm/s z-rate (hold_extrap).
2. Plant follows cmd, actual_twist_vz = +130 mm/s.
3. ZMQ sender emits target with refreshed `arrival_time`. `ZmqTargetSource.poll()` sees `arrival != self._arrival_time` (line 187), marks `_target_dirty = True`.
4. `ZmqTargetSource.update()` rebuilds cache: `flat_target_to_events(current_pose, current_twist=[0,0,+130,0,0,0], target_pose, sim_time, target_twist=None→ _ZERO6, arrival_time=sim_time+0.5, clamp_start_twist_mmps=140)`.
5. Clamp at 140 is a no-op — 130 < 140.
6. New cached events: `[(sim_time, current_pose, [0,0,130,0,0,0]), (sim_time+0.5, target_pose, 0)]`.
7. MPC horizon reads ref[0] = (current_pose, +130 mm/s) — this is the evaluated quintic at t=sim_time, which is exactly the starting event.
8. IPOPT is now asked to plan starting from (plant_pose, plant_twist=+130) heading to (target, 0). Unchanged NLP difficulty vs. last tick.
9. IPOPT saturates again, fallback extrapolates at +130 mm/s. Plant keeps climbing. Goto 1.

The `PLANT_TELEMETRY_COLLAPSE` detector (Bundle B, commit 0dbb4fa) correctly captured this exogenous-looking failure. The operator E-stop ended it.

The trap arms only when:
- (a) the sender streams targets with changing `arrival_time` field (stream-style, not one-shot),
- (b) the plant twist is already non-zero, typically from a prior fallback burst.

That's why 223727 and 223756 (same day, same ZMQ path) did not hit it — those sessions apparently received a one-shot target and the cache stayed stable.

### Mechanism 3 (ruled out or non-regression)

- **Stroke limit at Z=220**: false positive in the diagnose flag. The flag reads `min extension 0.0 mm` which turned out to be the zero-extension sentinel values emitted *after* telemetry collapse, not real stroke saturation. Actual_ext at Z=220 is ~200 mm, nowhere near any stroke limit. All six legs are equal length at (0,0,220) by symmetry, so no single leg is loaded asymmetrically.
- **Leg-1 gain revert**: the YAML change (pos_gain 30→40, vel_int_gain 0.24→0.32) was done at 17:56 on 2026-04-19 to test whether Iteration-3's reduced gain was causing leg 1's pose-dependent twitch at (0,-100,200). Observed per-leg hold stdev dropped from 10.8 µm → 4.9 µm on the stable final hold in 22:37 run 2, consistent with the stiffer gain taking effect. This is a POSITIVE, not a regression. Note that leg 4 remains at the reduced 30/0.24 as a control, so the platform is running asymmetric per-leg gains — may warrant a follow-up discussion but is not the cause of the jerkiness.

## Discussion

The `/investigate` fix-proposer (agent invocation on 2026-04-20) ranked three candidates in recommended order: Fix 1 (gate `_target_dirty` on pose/twist only), Fix 2a (raise `cruise_speed` default 80 → 100 mm/s), Fix 3 (tighten start-twist clamp 140 → 60 mm/s via new `MPCParams.max_ref_start_twist_mmps`). User elected to land Fix 1 + Fix 3 only, deferring Fix 2a ("keep speeds low until we get the lurking bugs resolved"). The settling-lag mechanism is therefore unaddressed by this commit; it will need a separate investigation once the overshoot-saturation class of bugs is fully closed out. See [plans/active/hardware-bringup.md](../plans/active/hardware-bringup.md) for phase sequencing.

A deferred fourth candidate (Fix 2b — activate the K1–K6 feasibility layer by threading `v_max_mmps`/`tau_s` through all call sites; the deprecation warning at [controller/target.py:216](../controller/target.py#L216) has been requesting this migration) is also on hold until we've cleaned up the ref-event layer further. The `ZmqTargetSource.update()` call at [controller/zmq_target.py:253-260](../controller/zmq_target.py#L253-L260) does pass `v_max_mmps`/`tau_s` today (so K1–K6 is active for the ZMQ path); the other target sources (`StaticTargetSource`, `AutoSequenceTargetSource`, `WaypointTargetSource`) still need threading.

## Fix

### Fix 1 — Gate `_target_dirty` on (pose, target_twist) only

[controller/zmq_target.py](../controller/zmq_target.py): removed `arrival != self._arrival_time` from the `changed` disjunction in `poll()`. The scalar `self._arrival_time = arrival` assignment remains and is still forwarded via `TargetCommand.arrival_time` on every tick — only the ref-cache invalidation trigger is gated. Added a block comment explaining the failure mode being closed.

### Fix 3 — Decouple clamp from kinematic ceiling; tighten to 60 mm/s

[controller/params.py](../controller/params.py): added `max_ref_start_twist_mmps: float = 60.0` field to `MPCParams`, distinct from `max_leg_vel_mmps = 140.0`. Docstring explains the "runaway sits below the kinematic ceiling" rationale.

[controller/mpc.py](../controller/mpc.py): added `'max_ref_start_twist_mmps'` to `_NLP_HASH_EXCLUDE_PARAMS` — this parameter is a ref-layer runtime bound and does not affect the compiled NLP, so it must not bust the AOT cache on change.

[controller/zmq_target.py](../controller/zmq_target.py): added a dedicated `clamp_start_twist_mmps: float | None = None` kwarg to `ZmqTargetSource.__init__`, stored on `self._clamp_start_twist_mmps`, and used in `update()` at the `flat_target_to_events` call (replacing the previous conflation with `self._v_max_mmps`). Docstring updated to call out the distinction from `v_max_mmps`.

[run_mpc.py](../run_mpc.py): `_src_kwargs` (threaded to `StaticTargetSource` / `AutoSequenceTargetSource`) now uses `clamp_start_twist_mmps=mpc.params.max_ref_start_twist_mmps` (60) while keeping `v_max_mmps=mpc.params.max_leg_vel_mmps` (140) for the K1–K6 layer. The `ZmqTargetSource` construction and the stow-cleanup `AutoSequenceTargetSource` construction both pass `clamp_start_twist_mmps=mpc.params.max_ref_start_twist_mmps` as well.

[sim/main.py](../sim/main.py): the `--hardware` sim-side `StaticTargetSource` and `ZmqTargetSource` paths were updated symmetrically.

### Test coverage added

[tests/sim/test_zmq_target.py](../tests/sim/test_zmq_target.py):

- Fixture updated to pass `clamp_start_twist_mmps=140.0` so pre-existing twist-clamp tests continue to exercise the same behaviour under the decoupled API.
- `TestTwistClamp.test_clamp_independent_of_v_max` — new. Asserts that with `v_max_mmps=140, clamp_start_twist_mmps=60`, a plant twist of 130 mm/s (below kinematic ceiling, characteristic of fallback-driven runaway) is clipped to 60 mm/s in the quintic start event. Explicitly covers the Fix 3 intent.
- `TestTwistClamp.test_clamp_none_disables` — updated to set `clamp_start_twist_mmps=None` directly (previously relied on `v_max_mmps=None` conflation).
- `TestChangeDetection.test_arrival_time_only_change_does_not_rebuild_cache` — new. Enqueues a target at `arrival_time=1.0`, then ten more messages with the same pose but `arrival_time` refreshed each tick. Simulates the 223902 runaway: plant state has `vz=130 mm/s` during the refresh storm. Asserts `tc_b.ref_events is tc_a.ref_events` (cache reused), and that the scalar `arrival_time` IS still updated and propagated via `TargetCommand`.
- `TestChangeDetection.test_pose_change_still_rebuilds_cache` — new regression guard. Ensures the Fix-1 gating doesn't accidentally suppress a genuine pose change.

### Verification

`pytest tests/ -v`: **985 passed, 52 warnings in 233 s**. Baseline was 959 (the 2026-04-19 bundle-a-mpc-overshoot-saturation-fix entry's verification run) → +26 new tests, zero pre-existing regressions. Warnings are the pre-existing `return True` style in `tests/motion/test_motor_guard.py` and the K1–K6 deprecation warning from the target-source call sites that have not yet been migrated — neither is introduced by this entry.

AOT solver regenerated via `python controller/generate_solver.py` (hash `72528f46…`). The regen script has a small post-compile cleanup bug (`FileNotFoundError` unlinking `mpc_gen.c`) that does not affect artefact validity — the `.so` and `.hash` are both correct. Worth a separate tiny fix.

## Outcome

Resolved by the K1–K6 reference-feasibility contract that landed in commit `e8c5833` on 2026-04-20. The contract is a broader generalisation of Fix 1 (gate `_target_dirty` on pose/twist only) and Fix 3 (dedicated `MPCParams.max_ref_start_twist_mmps` clamp) that the fix-proposer originally recommended — both are included inside the larger W1–W11 work. The settling-lag half (originally deferred for a dedicated follow-up since the user wanted to keep speeds low) is addressed by the contract's per-node-IK cold start + walk-forward fallback hardening rather than by raising `cruise_speed`.

Hardware validation on session group `2026-04-20_16-00-31` (commit `831ab6f`) reproduced the 223902 failure geometry under the fix:

| Metric | 2026-04-19 fail | 2026-04-20 fixed | Δ |
| --- | --- | --- | --- |
| p50 solve time (ms) | 26.0 | 10.5 | -59% |
| max consecutive timeouts | 147 | 5 | -97% |
| success rate | 6.2% | 96.2% | +90pp |
| E-stop required | YES | NO | ✓ |

The K1 anchor is directly visible in the telemetry at the target-flip moment (sample 13, t=1.274 s): `ref_pose_z == actual_pose_z == 174.25 mm` exactly — the quintic is now rebuilt from live plant state at the transition rather than stepping away from it.

Baselines were also re-validated: simple holds at z=220 and z=170 (`mpc_20260420_160120.csv` / `mpc_20260420_160159.csv`) match the pre-fix p50 of 10.6 ms with 100% / 99.9% success rates — zero regression from the contract on the clean-hold case. The rapid-succession W7 stress test (four targets in 2 s, `mpc_20260420_160242.csv`) passed with max 1 consecutive timeout — the live-twist-trap mechanism identified in this entry no longer fires.

See [2026-04-20-k1-k6-reference-feasibility-resolution.md](2026-04-20-k1-k6-reference-feasibility-resolution.md) for the K1–K6 work's own narrative and `sim/analysis/known_issues.yaml` for the MPC_OVERSHOOT_SATURATION entry transition from `active` → `fixed`.

## Open Questions

- On the 223902 feedback-trap hypothesis: confirming requires inspection of the raw ZMQ target stream (not in the CSV). Worth adding a lightweight log line in `ZmqTargetSource.poll()` that prints when cache rebuilds, so future sessions carry evidence.
- `cruise_speed=80 mm/s` was inherited from the initial quintic-reference work — probably not tuned for Phase-4 v_max=140 mm/s hardware.
- Leg 4 is still running the Iteration-3 reduced gain (30/0.24) while leg 1 has been reverted to 40/0.32. Asymmetric gains may be fine (it's an A/B test), but worth documenting.
- The 25-50 s transitions in 22:44 also expose that walk-forward fallback is not ref-correcting — it just extrapolates cmd. Longer-term, fallback could be smarter (e.g., blend toward a flat-hold at the current target).
