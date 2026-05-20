---
title: MPC-loop overhead spikes trigger fallback bursts — audible motor "fighting" on every move
type: investigation
date: 2026-04-18
status: tuned
phase: "STANDBY-mode MPC (post-64742f2 fallback walk-forward) — tail-latency attribution"
related_plan: "hardware-bringup.md"
related_issues:
  - MPC_STALENESS
related_entries:
  - 2026-05-20-mpc-warmstart-deadlock-escape
sessions:
  - mpc_20260418_184845.csv
  - mpc_20260418_185014.csv
  - mpc_20260418_222358.csv
  - mpc_20260418_222414.csv
  - mpc_20260419_094723.csv
  - mpc_20260419_094733.csv
  - mpc_20260520_115857.csv
files_changed:
  - controller/hardware_plant.py
  - controller/mpc.py
commits:
  - a89a4dd
  - abc4a8e
  - b5d83df
subsystem:
  - controller
  - mpc
  - telemetry
tags:
  - performance
  - observability
  - control
---

## Summary

On every STANDBY-mode move commanded via `run_mpc.py`, the platform produces an audible small-amplitude bidirectional "fighting" motion driven by the MPC command signal. Root mechanism: single-tick Python-level overhead spikes (observed 33–43 ms on a nominal 5 ms baseline) trigger short MPC-fallback bursts. When a burst exhausts the walk-forward plan, the existing `_handle_failure` endgame freezes `cmd` at `prev_w[6(N-1):6N]` while the platform (τ=40 ms) coasts past; the ODrive PID then recoils at −167 to −220 mm/s. That recoil is what the operator hears. This is not a regression from recent fixes (walk-forward fallback 64742f2, 22 ms IPOPT budget ae51fb7, prime-at-live-pose 31a56c5) — those all work as intended. The mechanism is "below" those fixes, in the Python control plane.

> **See also:** `2026-04-18-hold-fighting-motion-onset-jitter.md` — the sub-LSB command dead-band in `controller/hardware_plant.py` (commit `abc4a8e`) addresses the HF `pos_setpoint` jitter at hold that is characterised in that entry. It is filed here because the MPC-side overhead-spike mechanism is the closest-related "fighting" failure mode, but the primary narrative for the dead-band fix lives in the hold-fighting entry.

## Symptoms

- Audible "fighting" sound on every move (Z ramps, Y holds, diagonals, return-to-home). Present in single-axis and combined-axis commands.
- In `temp/logs/mpc_20260418_184845.csv` (N=600, 15.9 s): overhead_ms p50=5.3, p95=10.8, p99=24.7, max=43.2. solve_time_ms p50=10.9, p95=28.5, p99=34.4, max=54.1. 17/600 ticks (2.8%) overhead > 15 ms; 23 ticks total > 40 ms MPC period.
- Direct trace around step 7 (t=1.05 s): overhead spike to 43.2 ms + solve 35.1 ms = 78.3 ms total (3× period). Steps 8–15 cascade into fallback/hold. Step 14 `actual_ext ≈ 178.7 mm` while cmd frozen at 175.7 mm (3 mm overshoot). Step 15 leg velocities jump to −167 to −220 mm/s reversing direction. 11 over-v_max samples in a 200 ms window.
- Same pattern in `mpc_20260418_185014.csv` and older `mpc_20260418_164119.csv` — pre-existing, not a regression.

## Diagnosis

**Two independent failure modes compound into the fighting:**

1. **Isolated overhead pops** (single-tick, flanked by clean 5–7 ms neighbors). Step 7 = 43 ms with 6–7 ms neighbors. Step 443 = 41 ms with 4–8 ms neighbors. Step 24 = 33 ms overhead with `solve=10.1, fk=1` (solver/FK healthy) — proves the spike is outside the solver. Spike cluster gaps (min=1, median=9, max=346 ticks) match bursty generational-GC signature, not monotonic ZMQ queue growth.
2. **Chronic solve-time overrun** — p95 solve = 28.5 ms > 22 ms IPOPT budget. Once a spike triggers the cascade, IPOPT keeps timing out and the cascade sustains itself.

**Walk-forward endgame recoil** — The existing `_handle_failure` walks `cmd` along `prev_w[6k:6(k+1)]` with `k = min(_fallback_step, N−1)`. When a burst exceeds N−1 = 9 ticks the cursor saturates and `cmd` freezes at `prev_w[6(N-1):6N]`. Plant dynamics (τ=40 ms) keep moving under the frozen command; the PID then has to recoil back. This is the audible event. Fix was rejected in the prior `2026-04-17` investigation because that work was focused on fixing the much larger sawtooth; with walk-forward in place the endgame recoil is now the dominant residual effect.

**Ruled out on CSV evidence:**
- FK convergence (fk_iterations is ~3 on both high- and low-overhead ticks — constant)
- CSV flush (`_FLUSH_EVERY=5000`, only 600 records written)
- Dashboard (not enabled in these sessions)

**Candidates requiring instrumentation:**
- Python GC pause (primary hypothesis — pattern matches)
- ZMQ SUB drain surge
- `cartesian_to_motor_commands` torque-FF compute
- Stdout-tee .log write contention

## Flagged Issues

- [performance] Overhead spike p99=24.7 ms, max=43.2 ms on 5 ms baseline — triggers fallback bursts
- [performance] Chronic solve_time overrun: p95=28.5 ms > 22 ms IPOPT budget
- [control] Walk-forward endgame freezes cmd at `u[N-1]` while plant coasts — produces 3 mm overshoot and −220 mm/s PID recoil

## Discussion

Planned two-part fix:
- **Fix A (instrumentation, commit-worthy):** extend StepRecord with per-segment timings (t_getstate_ms, t_target_ms, t_solve_setup_ms, t_hooks_ms, t_cmd_ms, t_log_ms), gc_ms, zmq_drain_count. Install gc.callbacks to attribute pauses to specific ticks. Print one-line `OH SPIKE ...` breakdown when overhead > 15 ms. Stage 2 remediation (GC disable / ZMQ HWM=1 / CSV background thread) follows the instrumentation data.
- **Fix B (endgame extrapolation, direct remedy):** once `_fallback_step >= N-1`, switch from frozen-cmd to plant-tracking extrapolation: `cmd = q_cur + q_dot * dt0`, rate-limited by `|cmd − prev_u| ≤ v_max·dt`. Requires passing `q_dot` (state.leg_velocities_mmps) into `_handle_failure`.

Rejected alternatives: emitting `_timeout_hint` partial iterate as cmd (safety), reducing horizon N (invalidates AOT solver), adaptive max_cpu_time (complexity not justified at bringup).

## Fix

### Fix A — instrumentation (commit a89a4dd)

Added per-segment `perf_counter` timings to `run_mpc_loop`, a `_GCTracker` that attributes `gc.callbacks` pauses to ticks, a ZMQ drain counter on `HardwarePlant.get_state`, and a stdout `OH SPIKE` breakdown when `overhead_ms > 15`. Extended `StepRecord` with 8 new columns (`t_getstate_ms`, `t_target_ms`, `t_solve_setup_ms`, `t_hooks_ms`, `t_cmd_ms`, `t_log_ms`, `gc_ms`, `zmq_drain_count`). All fields default to 0 so sim and older analysis tools remain compatible.

### Fix B — hold-branch plant-tracking extrapolation (commit a89a4dd)

`_handle_failure` now accepts an optional `q_dot` (measured leg velocities). In the hold branch (fires after `max_consecutive_failures` walk-forward steps), when `q_cur` and `q_dot` are both available it emits `cmd = q_cur + q_dot * dt0` rate-limited to `|cmd − prev_u| ≤ v_max·dt` and returns status `hold_extrap(...)`. The original freeze-at-prev_u behaviour is retained as a fallback for sim/cold paths where `q_dot` is None. Three call sites in `solve()` updated to pass `state.leg_velocities_mmps`. Walk-forward branch and cold_hold branch are unchanged.

### What the instrumentation revealed

- `OH SPIKE` lines across two re-test sessions (`mpc_20260418_222358`, `mpc_20260419_094723`, `mpc_20260419_094733`) consistently show `getstate ≈ overhead` with companion stdout warnings `FK did not converge; using last measured pose`. GC callback count is zero on every spike — **GC was NOT the cause**; the primary hypothesis was wrong.
- FK Newton-Raphson iterates to `max_iter = 50` at ~0.6 ms/iter = 30+ ms per failed call. Failure is correlated with motion onset where the cached `_fk_last_guess` lags the moving plant.
- Example: UP session step 19 `oh=33.4 getstate=31.4 solve=10.2 drain=3` — a minimal-drain tick, so ZMQ queue growth is also ruled out. Pure FK.
- ZMQ drain counts of 9–14 are normal (500 Hz publisher / 40 Hz consumer = 12.5 msgs expected); drain=43–45 on some ticks reflects the MPC running LATE as a consequence of a prior FK spike, not the cause.
- Chronic solve-time overrun (p95 ~20–23 ms vs 18 ms budget) is a separate issue. The AOT solver / IPOPT options tuning would need to address it.
- Fix B activated cleanly (step 1 emitted `hold_extrap(...)` on both 2026-04-19 sessions). The walk-forward endgame recoil is no longer a contributor in the current traces.

### Fix C — FK real-time budget (commit b5d83df)

`controller/hardware_plant.py:609-613`: pass `max_iter=10, tol=1e-4` to `leg_lengths_to_pose(...)`.

Rationale:
- Default `tol=1e-10 mm` is 700 000× below the 0.07 mm encoder LSB — Newton iterates on floating-point noise long past any physically meaningful residual. `tol=1e-4 mm` is still 700× below LSB and is reached in 3 iters on the common path.
- `max_iter=10` caps the divergence case at ~6 ms (was ~30 ms). When FK would have iterated to 50 and thrown, it now throws at 10 — the existing RuntimeError / `_last_measured_pose` fallback is unchanged, so safety semantics are preserved (`_FK_FAIL_ESTOP_THRESHOLD=5` still trips after 5 consecutive failures).
- Call-site scope only — does not modify `ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py`.

Expected effect: eliminates the 30-ms FK-driven overhead spikes; total tick time on FK-failed ticks drops from ~65 ms (2.6× budget) to ~35 ms (1.4× budget), which should stay out of fallback cascades.

### What Fix C does NOT address

Chronic solve_time overrun during motion onset (p95 ~20–23 ms). Deferred — if Fix C alone does not quiet the fighting, next candidates are (a) better warm-start quality via `mpc.predicted_poses_view[0]` as FK initial guess, (b) IPOPT option retuning, (c) horizon reduction (invalidates AOT solver, expensive).

## Outcome

Status flipped `in-progress` → `tuned` on 2026-05-20. All three planned
fixes (A, B, C) landed; the original on-every-move "fighting" symptom is
resolved on hardware. One residual jitter mechanism remains under separate
investigation — see "Residual" below.

### Fixes landed

| Fix | Commit | What it addressed |
|-----|--------|-------------------|
| A — overhead instrumentation | `a89a4dd` (2026-04-18) | Added per-segment timings, `_GCTracker`, `OH SPIKE` stdout. Revealed `getstate ≈ overhead` with zero GC callbacks — falsified the GC hypothesis and pointed at FK Newton-Raphson divergence. |
| B — hold-branch plant-tracking extrapolation | `a89a4dd` (2026-04-18) | `_handle_failure` `hold_extrap` arm emits `cmd = q_cur + q_dot·dt0` (rate-limited) instead of freezing at `prev_w[6(N-1):6N]`. Eliminated the walk-forward endgame recoil that produced −167 to −220 mm/s reversals. |
| C — FK iteration budget cap | `b5d83df` (2026-04-19) | `leg_lengths_to_pose` capped at `max_iter=10, tol=1e-4`. Cuts the divergence-case wall-time from ~30 ms to ~6 ms; the existing RuntimeError → `_last_measured_pose` fallback path is unchanged. `_FK_FAIL_ESTOP_THRESHOLD=5` semantics preserved. |

### Reframing for the chronic-solve-time overrun

This entry's diagnosis section identified two compounding mechanisms:
isolated overhead pops (resolved by Fix C) and a *chronic* solve-time
overrun framed as "p95 solve = 28.5 ms > 22 ms IPOPT budget." That second
framing turned out to be incomplete. The downstream 2026-05-20
investigation showed the chronic phase of the cascade was driven by a
distinct warm-start + `t_ref`-freeze deadlock: once a few solves tripped
the CPU cap, IPOPT was burning the full 22 ms in init / KKT factorisation
on poisoned warm-start vectors and exiting with `iter_count = 0`, and
the runner's `t_ref` froze on the unchanging fallback status —
presenting the solver with the bit-identical NLP every tick.

That mechanism and its fix are documented in
[2026-05-20-mpc-warmstart-deadlock-escape.md](2026-05-20-mpc-warmstart-deadlock-escape.md);
commit `67ae3da` generalises the warm-start contract from
*structural-reference-shift-driven* to *also failure-driven*. Hardware
re-run at z=30 post-fix (`temp/logs/mpc_20260520_115857.csv`, 60 s,
2400 ticks): 99.6 % success, zero chronic phase, two isolated singleton
fallbacks across the full settle.

### Residual under separate investigation

The two singleton fallbacks visible at ticks 22 and 71 of
`temp/logs/mpc_20260520_115857.csv` coincide with two operator-reported
audible/visible "jerk" events during the z=170→z=30 settle. Candidate
mechanism (working hypothesis, to be confirmed/refuted by an offline
production-faithful replay): the walk-forward branch emits
`_prev_w[6:12]` on `_consecutive_failures == 1`, but `_prev_w[6:12]` is
`u[1]` from the *prior* tick's plan — computed as a prediction one step
ahead of the prior tick's state, not a fresh solve against the current
measured state. The rate-limit clamps the step at
`max_leg_vel_mmps · dt0 = 140 × 0.025 = 3.5 mm`, and the recorded data
shows `cmd_ext` discontinuities of 2.1 mm (tick 22) and 3.5 mm (tick 71
— **exactly at the clamp**) which the motor-guard Hermite interpolation
cannot fully smooth. That tick-71 event saturating the rate-limit
exactly is itself evidence that the walk-forward singleton path is the
mechanism producing the jerk — the clamp fired, which only happens when
the emitted `_prev_w[6:12]` step against the current `_prev_u` exceeds
the 3.5 mm budget.

This is structurally distinct from the cascade case that the original
walk-forward fix (`64742f2`) was designed for: a *cascade* rides the
same old plan across multiple ticks (which is continuous by construction);
a *singleton* is followed by a fresh `u[0]` from a healthy solve against
the current state on the very next tick, exposing the prior tick's stale
prediction as a one-tick discontinuity. The Fix B `hold_extrap` arm
addresses the post-cascade case, not the singleton case. Tracked
separately under the walk-forward singleton-emission investigation.
