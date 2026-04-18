---
title: MPC-loop overhead spikes trigger fallback bursts — audible motor "fighting" on every move
type: investigation
date: 2026-04-18
status: in-progress
phase: "STANDBY-mode MPC (post-64742f2 fallback walk-forward) — tail-latency attribution"
related_plan: "hardware-bringup.md"
related_issues:
  - MPC_STALENESS
sessions:
  - mpc_20260418_184845.csv
  - mpc_20260418_185014.csv
files_changed: []
commits: []
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

_To be filled by subsequent /investigate steps._

## Outcome

_To be filled by subsequent /investigate steps._
