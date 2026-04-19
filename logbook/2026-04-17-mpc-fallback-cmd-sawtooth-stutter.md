---
title: MPC fallback-command sawtooth produces visible stutter on off-Active multi-axis moves
type: investigation
date: 2026-04-17
status: in-progress
# --- Context (use what's relevant) ---
phase: "STANDBY-mode multi-axis MPC (post-2.4, pre-4.x combined-axis work)"
related_plan: "hardware-bringup.md"
related_issues:
  - MPC_STALENESS
  - LEG2_TRACKING
sessions:
  - mpc_20260417_184446.csv
  - mpc_20260417_184516.csv
  - mpc_20260417_184532.csv
  - mpc_20260417_184547.csv
  - mpc_20260417_184601.csv
# --- Traceability ---
files_changed:
  - controller/mpc.py
  - run_mpc.py
commits:
  - 64742f2
# --- Classification ---
subsystem:
  - controller
  - mpc
tags:
  - performance
  - safety
  - IPC
---

# MPC fallback-command sawtooth produces visible stutter on off-Active multi-axis moves

## Summary

On off-Active multi-axis MPC moves, the solver's fallback and hold paths (triggered when IPOPT exceeds the 24 ms CPU budget) emit commands that go backwards or freeze relative to the evolving reference. The motor guard faithfully tracks these stale/reversed commands, producing a visible 1–2 s cmd_ext sawtooth and a platform overshoot-recovery cycle — this is the stutter the operator observed. Root mechanism is new (not documented in known_issues.yaml yet); it is triggered on combined X/Y/Z moves where p50 solve time sits already at the 24 ms budget edge.

## Symptoms

- Operator-visible platform stutter during a STANDBY-mode multi-axis move.
- Session 184601 (the stutter session): 74.2% solver timeouts, max 440 consecutive `hold(Maximum_CpuTime_Exceeded)`, p50 25.9 ms > 24 ms budget (p95 31.3 ms, max 65.4 ms), steady-state pos RMS 33 mm, peak 42 mm.
- Two flagged cmd_ext discontinuities at t=1.477 s: leg 2 jumped −10.5 mm and leg 3 jumped −9.4 mm in a single 25 ms sample. 29 additional actual-position jumps (2–8 mm) in the window t=1.27–2.22 s.
- HardwarePlant telemetry-stale E-STOP fired repeatedly (108 times) late in the run (t≈11 s), downstream of the stall — it is a consequence, not the cause.
- Preceding commands 1–3 in the same ROS2 session (pure single-axis Z or Y moves up to 50 mm) were clean: 97.5–100% solver success, < 1 mm final error.
- Command 4 (the combined 174 mm X+Y diagonal in 2.3 s) had only 90% solve success and left the platform 29.6 mm short of target — that off-Active end state became the starting pose for command 5, which is where the stutter appeared.

## Diagnosis

### Stutter mechanism (direct CSV evidence, leg 2 — leg 3 shape is identical)

Steps 15–26 (t=0.75–1.07 s): healthy ramp, all `Solve_Succeeded`, solve time 16–27 ms.
Step 27 (t=1.10 s): first `fallback(Maximum_CpuTime_Exceeded)` — cmd2 drops 151.62 → 149.37 mm (−2.25 mm) on one sample while actual is still climbing. First stutter kick.
Steps 28–36: alternating `Solve_Succeeded` / `fallback`/`Solved_To_Acceptable_Level`, cmd zigzagging forward-and-back ≈ 2–3 mm per sample.
Step 38 (t=1.48 s): `fallback` — cmd 160.58 → 158.74 → 148.27 mm, a **−10.5 mm backwards cmd on a single sample**. Leg 3 same shape (−9.4 mm). This is the largest single-sample stutter and matches the diagnosis flags.
Steps 41–60: `hold(Maximum_CpuTime_Exceeded)` — cmd frozen at 145.32 mm while actual has momentum: act2 coasts 143.7 → 146.2 → 152.3 → 158.8 → 160.0 → 161.9 → **163.3 mm peak** (18 mm past cmd) before settling back to ≈ 145.1 mm. This is the dominant second-long visible stutter.
Steps 60–500: `hold(Maximum_CpuTime_Exceeded)` the whole time, platform parked at ~(60, ?, 184) while ref holds at (33, ?, 197), 37.7 mm unresolved position error.

### Why session 184601 stutters but 184446/184516/184532 don't

Commands 1–3 vary only one axis at a time starting from home. Stewart-platform Jacobian is near-symmetric there, IPOPT converges in a few iterations, solves land well under 24 ms, fallback path almost never fires. Command 5 starts off-Active in all three axes simultaneously (platform at (71.5, −92.4, 170) from unfinished command 4), which is where solve time hits the budget edge consistently — fallback/hold paths fire on nearly every other sample, which is exactly where the sawtooth lives.

### Root cause (new hypothesis)

Both fallback paths in controller/mpc.py emit commands that fail to track the evolving reference:

1. `fallback(Maximum_CpuTime_Exceeded)` salvages a partial IPOPT iterate from the previous warm-start. Against a still-evolving reference that iterate can be behind the current cmd, so the controller outputs a command that moves backward.
2. `hold(Maximum_CpuTime_Exceeded)` pins cmd at the last successful value. The reference keeps moving and the platform has momentum, so actual overshoots past cmd and has to return — a physical overshoot-recovery cycle.

The motor guard is not at fault: it is faithfully interpolating to whatever cmd the MPC emits. The cmd itself is the stutter source.

Secondary contributor: p50 solve time 25.9 ms is already > the 24 ms budget for this operating point. The fallback/hold paths were designed to fire rarely as a safety net. On off-Active multi-axis moves they fire constantly, which is why a mild design compromise becomes a dominant stutter.

### Flagged Issues

- [error] MPC solver saturated: 74.2% timeouts, max 440 consecutive → known issue MPC_STALENESS
- [error] MPC solve budget exceeded: 379 consecutive > 24 ms, max 65.4 ms → same (MPC_STALENESS)
- [error] Command discontinuity leg 2: 10.5 mm at t=1.477 s → unknown pattern (new: MPC_FALLBACK_SAWTOOTH candidate for known_issues.yaml)
- [error] Command discontinuity leg 3: 9.4 mm at t=1.477 s → unknown pattern (same)
- [warning] Leg 2 RMS 2.5× median → known issue LEG2_TRACKING (persistent, mechanical)
- [warning] Steady-state RMS 33 mm → symptom of the above, not independent
- [info] First-sample cold solve 28.4 ms → TIMING_FIRST_SAMPLE, expected

### Known-issue matches

- MPC_STALENESS (active) — direct, via solve-time cascade
- LEG2_TRACKING (persistent, mechanical) — direct
- ZMQ_TELEMETRY_STALE (2026-04-15) — referenced as prior art, but NOT the cause here; telemetry went stale as a downstream consequence of the MPC loop starving its own SUB thread once the sawtooth degenerated into 440 consecutive holds.

### New pattern worth adding to known_issues.yaml

MPC_FALLBACK_SAWTOOTH: cmd_ext jumps backwards > 2 mm between consecutive samples while solver alternates success/fallback/hold, typically on off-Active multi-axis ramps where p50 solve time ≥ budget. Detection signature: count of negative cmd_ext deltas > 2 mm during solver success/fail alternation windows.

## Discussion

Summary of the options weighed (by the /investigate fix-proposer agent, with risks):

1. **Walk forward along `prev_w` during fallback (correctness).** Each consecutive fallback emits `prev_w[6k : 6(k+1)]` — the next planned node from the last successful solve — instead of repeatedly emitting the shifted `prev_w[6:12]`. Rate-limit clamp switches from clipping against `q_cur` to clipping against `self._prev_u`, so the cmd cannot be pulled backward by a momentum-overshot plant. Medium risk. Chosen.

2. **Realign IPOPT budget (tuning).** `run_mpc.py` was overriding `max_cpu_time=0.022` s, contradicting the documented `params.py` default of 0.018 s and the value that `controller/generate_solver.py` was AOT-compiled against. The 22 ms wall-clock was the reason p50 solve = 25.9 ms timed out so often. Low risk. Chosen, stacks on top of (1).

3. **Emit `_timeout_hint` (partial IPOPT iterate) as cmd source.** Rejected. Partial iterates can violate IK equality or rate-limit constraints; commanding them is unsafe without extensive sanity checks. Walk-forward along `prev_w` is a safer realisation of "smarter fallback" because the plan is already feasible by construction.

Three additional bugs the agent surfaced while tracing the mechanism:
- **Same `prev_w[6:12]` emitted every consecutive fallback tick.** `_shift_warm_start(self._prev_w)[:6]` always returns `prev_w[6:12]` and `_prev_w` only refreshes on success — so 440 consecutive fallbacks all emitted one fixed 6-vector. This is the real reason `cmd_ext_2` froze at 145.32 mm for 440 samples, not the `hold(...)` branch.
- **Rate-limit clamp against `q_cur` is wrong during overshoot.** When actual has overshot the held cmd, `q_cur` is ahead in the direction of travel and `np.clip(cmd, q_cur ± max_delta, …)` pulls cmd toward `q_cur`, i.e. toward the overshoot — causing the ~10 mm single-sample backwards kicks.
- **`cmd_vel` collapses to zero after the first fallback.** `self._prev_u = cmd` is assigned inside the fallback branch, so on the next fallback tick `cmd == prev_u` and the feedforward velocity is 0 — losing the motor guard's FF signal during the fallback chain.

Rejected alternatives considered:
- Reducing horizon (N=10 -> N=5) would reduce solve time directly but changes the AOT-compiled solver's structure and costs lookahead. Deferred until we see whether Fix 1 + Fix 2 alone resolve the stutter.
- Adaptive `max_cpu_time` based on ref-change magnitude. Needlessly complex for a bringup issue that turns out to have a straightforward correctness fix.

## Fix

Two file changes, in one commit.

### `controller/mpc.py` — walk-forward fallback

1. Added `self._fallback_step: int = 0` to `__init__` (near the other per-solve state around the `_consecutive_failures` field) with a docstring explaining its purpose.
2. Added `self._fallback_step = 0` to `reset()` alongside the existing `_consecutive_failures = 0` line.
3. Added `self._fallback_step = 0` to the success branch of `solve()`, right after `self._consecutive_failures = 0`, so a successful solve restarts the walk-forward cursor.
4. Added `self._fallback_step = 0` to the exception handler that already clears `_prev_w`, `_prev_lam_*`, and `_timeout_*` — keeps the invariant "if `_prev_w` is None, `_fallback_step` is 0".
5. Rewrote the first branch of `_handle_failure` (the one gated on `_prev_w is not None and _consecutive_failures <= max_consecutive_failures`):
   - Increments `self._fallback_step` first, then takes `k = min(self._fallback_step, N - 1)` so the cursor caps at the last planned node.
   - Emits `cmd = np.clip(self._prev_w[6k : 6(k+1)], margin, stroke - margin)`.
   - Rate-limit clamp now uses `self._prev_u ± max_leg_vel_mmps * dt0` (NOT `q_cur ±`). Prior code: `cmd = np.clip(cmd, q_cur - max_delta, q_cur + max_delta)` (only applied when `q_cur is not None`). New code: `cmd = np.clip(cmd, prev_u - max_delta, prev_u + max_delta)` when `prev_u is not None`.
   - Computes forward-looking `cmd_vel = (prev_w[6*k_next : 6*(k_next+1)] - cmd) / dt0` with `k_next = min(k + 1, N - 1)`, so the motor guard's feedforward signal stays live across the fallback chain instead of collapsing to 0 after tick 1.
   - Adds `diag['fallback_step'] = k` so downstream telemetry/logging can see the walk-forward progression (not yet wired into the CSV — kept in diag dict only).

The existing `hold(...)` branch (fires once `_consecutive_failures > max_consecutive_failures`, default 3) is unchanged and now naturally holds at whatever the last walk-forward cmd was (<= u[3] of the old plan), which is still strictly better than the old behaviour (always holding at u[1]).

### `run_mpc.py` — IPOPT budget realignment

Changed `MPCParams(max_cpu_time=0.022, max_iter=100)` to `MPCParams(max_cpu_time=0.018, max_iter=100)`. The inline comment now documents the 18 ms IPOPT + 5 ms overhead = 23 ms budget inside the 25 ms MPC period, and references this logbook entry so future readers know why the value was pinned back to the `params.py` default.

### Files changed
- `controller/mpc.py` (+40 / -11 approx; state field, three single-line resets, and the fallback-branch rewrite)
- `run_mpc.py` (+6 / -2; one-value change plus expanded comment)

### What was deliberately NOT changed
- `controller/runner.py` `t_ref` freeze during fallback — already correct, no change.
- `controller/hardware_plant.py` staleness thresholds — the 108 telemetry-stale E-STOPs in 184601 are a downstream consequence of the sawtooth; no threshold change needed.
- `controller/telemetry.py` / `StepRecord` — `fallback_step` stays in the `diag` dict for now; wiring it into the CSV column schema is left for a follow-up if it proves useful during hardware validation.
- `sim/analysis/known_issues.yaml` — the `MPC_FALLBACK_SAWTOOTH` entry will be added only once hardware validation confirms the fix works (pattern signature and status `fixed` should not be claimed before that).

## Outcome

<!-- To be filled in during /investigate outcome step. -->
