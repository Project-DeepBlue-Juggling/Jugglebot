---
title: hold_extrap fallback positive-feedback oscillation drove platform into chaotic motion (peak 336.9 mm/s leg velocity, 2.41× soft limit)
type: investigation
date: 2026-05-20
status: in-progress
phase: "hardware-safety / fallback redesign"
related_plan: ""
related_entries:
  - 2026-04-18-mpc-overhead-spikes-fallback-bursts
  - 2026-05-20-mpc-warmstart-deadlock-escape
sessions:
  - mpc_20260520_215909.csv
  - mpc_20260520_215924.csv
  - mpc_20260520_215936.csv
  - mpc_20260520_215950.csv
  - mpc_20260520_220004.csv
  - mpc_20260520_220020.csv
  - mpc_20260520_220030.csv
  - mpc_20260520_220039.csv
session_group: "2026-05-20_21-58-43"
rosbag: "/home/jetson/Desktop/rosbags/2026-05-20_21-58-43"
files_changed: []
commits: []
subsystem:
  - controller
  - mpc
  - safety
  - fallback
tags:
  - safety-critical
  - oscillation
  - positive-feedback
  - hold-extrap
  - hardware-safety
  - transient-bug
  - fallback-chain
---

# hold_extrap fallback positive-feedback oscillation drove platform into chaotic motion

## Summary

The hardware MPC entered a positive-feedback oscillation during a sequence of
lateral pose-target tests on 2026-05-20 21:59-22:00. Solve-time grew 2.5×
across the session (10 ms → 26 ms p50) — well into "every tick exceeds the
22 ms budget" territory by the chaotic moves. CTE rate climbed monotonically
across the 8 recorded sessions (0.5 % → 4.5 % → 5 consec → 21 consec → … →
**98.3 %** in `mpc_20260520_220030.csv`). Once the fallback chain entered
`hold_extrap`, the platform was driven by a positive-feedback loop:
plant velocity → `hold_extrap = q_cur + q_dot · dt0` → cmd → motor PID
accelerates plant → larger plant velocity → wronger emission. Peak measured
leg velocity: **250.6 mm/s in `mpc_20260520_220030.csv`, 336.9 mm/s in
`mpc_20260520_220039.csv`** — 1.79× and 2.41× the 140 mm/s
`max_leg_vel_mmps` MPC-side soft limit. The MPC's velocity constraint is an
NLP soft bound on planned `u_dot`, not an enforced hardware limit; motor_guard
tracks whatever cmd is given. **The platform was borderline-needed-E-stop
chaotic.** Operator did not E-stop; chaos ended only when `run_mpc.py` exited
at the 3 s `--duration` timeout.

The mechanism is `hold_extrap`'s use of measured plant velocity as input.
Plant velocity creates a feedback path from plant → cmd → plant that becomes
unstable when cmd leads plant (the normal control state during transients)
and the rate-limit can't reverse direction faster than the plant's inertia.
Earlier today an offline-probe analysis (see
`tools/probes/replay_hardware_csv.py`) had empirically demonstrated this same
wrong-direction-emission mechanism in a singleton-fallback case at tick 71 of
`mpc_20260520_115857.csv`; this hardware event provided independent
confirmation at hardware scale. The probe analysis is referenced in the
Discussion section below.

## Symptoms

- **CTE-rate progression across 8 sessions**: 0.5 % → 4.5 % → 4.0 % →
  5 consec → 21 consec → 12 consec / 18.3 % → **117 consec / 98.3 %** →
  **53 consec / 81.7 %**.
- **`solve_time` progression (p50)**: 10.4 ms → 16.8 ms → 11.8 ms → 10.9 ms
  → 12.0 ms → 19.1 ms → **26.4 ms** → **26.3 ms**. Solve doubled across the
  session.
- **Oscillation flag fired** in `mpc_20260520_220030.csv`:
  "chatter ratio > 0.5 on legs 3 (AMPLITUDE GROWING — possible instability)".
- **Reference was STATIONARY** in `220030`: `ref_pose_x/y/z = (69.6, 10.6,
  170)` for the entire 4.3 s, yet `cmd_ext` spanned 79.5 mm.
  **Platform commanded a wide-arc motion with no commanded motion in the
  reference.**
- **Peak actual leg velocity**: 250.6 mm/s (`220030`), 336.9 mm/s (`220039`).
  MPC's `max_leg_vel_mmps = 140` is a soft NLP constraint, not enforced at
  the motor_guard / actuator layer.
- **Operator-reported subjective experience**: *"extremely chaotic motion,
  very borderline for me to hit the E-stop. The motion ended when the mpc
  stopped, after 3 sec, but upon trying again, the result was similarly
  bad."*

## Diagnosis

Two compounding mechanisms produced the chaotic motion. They are separable
and should be tracked separately — Mechanism 1 is the **trigger**, Mechanism 2
is the **amplifier** and is the scope of this entry's fix.

### 1. Trigger — cumulative solve-time slowdown (out-of-scope)

A cumulative slowdown over the session brought the `p50` `solve_time` from
10 ms (`mpc_20260520_215909.csv`) to 26 ms (`mpc_20260520_220030.csv`,
`mpc_20260520_220039.csv`). By the chaotic moves, every tick was over the
22 ms budget, so CTEs fired chronically.

Root cause is unknown. Candidates the operator and Claude have surfaced:

- Thermal / DVFS throttling on the Jetson Orin Nano CPU cores.
- Cumulative GC pressure on the hot loop (though the hot-loop zero-allocation
  contract — see `2026-04-23-hot-loop-zero-allocation-contract` — was
  supposed to address this).
- ROS2 / DDS queue backpressure across the long-running launch session.
- Cache / page-cache effects after many minutes of `run_mpc.py` activity.

**Tracked separately under a Jetson load-profiling investigation that the
operator is building tooling for** (live-observer probe in `tools/probes/`
per the updated `tools/probes/README.md`). This entry does not attempt to
fix the trigger; it fixes the amplifier so that the trigger no longer
escalates into a safety event.

### 2. Amplifier — `hold_extrap` positive-feedback loop (this entry's scope)

Once chronic CTE began, the fallback chain entered `hold_extrap` (Fix B from
`2026-04-18-mpc-overhead-spikes-fallback-bursts`, commit `a89a4dd`).
`hold_extrap` computes `cmd = q_cur + q_dot · dt0` — "where the plant will be
one tick from now if it keeps doing what it's doing." This was DESIGNED under
the assumption `q_cur ≈ cmd` (steady-state), so `q_dot · dt0` was treated as
a small noise-tracking term. In transients (cmd leads plant by 15-20 mm,
`q_dot` has meaningful magnitude), `hold_extrap` emits a cmd that tells the
motor PID *"continue your current motion"* instead of *"stop and hold the
target."* This creates a positive-feedback loop:

```
plant velocity → q_dot reading → hold_extrap output → cmd → motor PID drives plant → larger plant velocity
       ↑___________________________________________________________________________________________|
```

Per-tick evidence from `mpc_20260520_220030.csv` (cross-reference
`controller/mpc.py:1724-1746` for the `hold_extrap` implementation):

- **Tick 6** (entry into `hold_extrap`): cmd 175.66, plant 175.69, `q_dot`
  +6.45 — benign.
- **Tick 10**: cmd 174.14, plant 175.16, `q_dot` -40.88 → `hold_extrap`
  follows down.
- **Tick 12-13**: `q_dot` REVERSES sign (+11.83, +25.82) → `hold_extrap`
  reverses cmd (174.77, 175.35).
- **Tick 18-19**: `q_dot` drops to -86 → cmd 172.38, 170.96.
- **Tick 21-22**: `q_dot` reverses again → cmd 173.70, 174.36.
- **Tick 83**: peak `leg_vel_5` = -250.6 mm/s. cmd was being chased by
  oscillating plant velocity.
- **Stale CAN telemetry** visible at ticks 15-16 (identical consecutive
  `leg_vel` readings of +8.61) — when telemetry catches up the plant has
  moved more than predicted, amplifying the next `hold_extrap` step.

### 3. Warm-start escape (commit 67ae3da) is firing but cannot save us

`temp/logs/mpc_20260520_220030.log` shows
`MPC: 100 consecutive failures > max=3 — invalidating warm-start (deadlock
escape)` repeating every tick from #100 to #117. But `solve_time` stayed
24-34 ms (over the 22 ms budget) — so the escape successfully invalidates
the warm-start every tick, but the resulting cold-warm-start solves still
over-run because the NLP is intrinsically hard with a high-velocity
off-trajectory state. **The warm-start fix
(`2026-05-20-mpc-warmstart-deadlock-escape`) is a necessary safety-net but
not sufficient when the platform state is already pathological.**

### 4. No enforced hardware velocity limit (separate finding)

Peak leg velocity 336.9 mm/s exceeds `max_leg_vel_mmps = 140` by 2.41×. The
MPC's velocity constraint is an NLP soft bound on *planned* `u_dot`, not a
hard motor-guard / actuator-side check. **This is worth its own investigation
eventually** — even with `hold_extrap` fixed, an out-of-distribution NLP
output could in principle command an unsafe leg velocity. Tracked separately;
not in scope for this entry.

## Discussion

> **TODO**: expand this section once the Tier 1 fix lands — explain the
> chosen primitive, why cmd-stream over alternatives, what failure modes it
> does and doesn't cover, and the Tier 2 follow-up scope. Tier 2 is
> **intentionally deferred**, not abandoned (see below).

The fix design is in flight via the fix-proposer agent (Tier 1 root fix
proposal). The user has explicitly rejected "minimum-risk patches" in favour
of a root fix that breaks the positive-feedback path. The leading direction
is to replace `hold_extrap`'s plant-state extrapolation
(`q_cur + q_dot · dt0`) with **cmd-stream extrapolation** (e.g.,
`cmd = 2·prev_u − prev_prev_u`) — which has *no* path from plant velocity to
cmd, so the positive-feedback loop is structurally impossible.

A second, deeper architectural fix is **deferred** to a follow-up session:

- **Tier 2 (deferred)**: trajectory-aware fallback using `_prev_w` indexed
  by elapsed time and/or sampling the K1-K6 reference source at the current
  `t_now`. This would let `hold_extrap` (or its successor) follow the
  *intended* trajectory rather than emit a no-motion approximation. The
  singleton-emission jerk investigation (Task 2 of the post-warm-start-fix
  arc) is part of this Tier 2 scope and is **not dropped** — it is
  re-prioritised behind the safety-critical Tier 1 fix this entry tracks.

The probe analysis at `tools/probes/replay_hardware_csv.py` had identified
this exact wrong-direction-emission mechanism offline before today's hardware
event. The probe found a singleton-fallback case at tick 71 of
`mpc_20260520_115857.csv` where `hold_extrap` emitted a cmd opposite the
trajectory direction; this hardware event is the chronic-case version of the
same root cause at much larger amplitude.

Cross-references:

- `logbook/2026-04-18-mpc-overhead-spikes-fallback-bursts.md` — Fix B
  introduced `hold_extrap`. The design assumption *"plant tracks cmd, so
  `q_cur ≈ cmd`"* was true in steady-state but false in transients. This
  entry exposes that hidden assumption.
- `logbook/2026-05-20-mpc-warmstart-deadlock-escape.md` — sibling
  investigation. The warm-start escape (commit `67ae3da`) IS firing every
  tick in the chaotic CSVs but doesn't help because the NLP itself is hard
  once cmd diverges from optimal. Necessary safety-net, not sufficient
  alone.

## Fix

**Tier-1 root fix — linear cmd-stream extrapolation, homogeneous fallback.**
Replaced the walk-forward + `hold_extrap` ladder in `_handle_failure`
([controller/mpc.py:1577-1720](../controller/mpc.py#L1577-L1720)) with a
single homogeneous primitive:

```python
cmd = 2 · _prev_u − _prev_prev_u           # linear cmd-stream extrapolation
cmd = clip(cmd, margin, stroke − margin)   # workspace
cmd = clip(cmd, _prev_u ± v_max·dt0)       # rate-limit against last emitted cmd
cmd_vel = (cmd − _prev_u) / dt0            # derivative of the extrapolation
```

The emission depends **only on the cmd history** (`_prev_u`,
`_prev_prev_u`). It does NOT read `q_dot`. The positive-feedback path
(plant velocity → q_dot read → fallback cmd → motor PID drives plant →
larger plant velocity) is now structurally absent.

Time-bounded: after 500 ms of consecutive failures (matching the
pre-existing W7 staleness threshold), the cascade escalates to
`cold_hold(q_cur)` — a measured-position hold with `cmd_vel = 0`. 500 ms
is the W7 "plan stale" threshold that already lived in the codebase;
re-using it preserves the established staleness contract while
restructuring the escalation target.

**Removed**:
- Walk-forward arm (was at lines 1684-1722) — emitted `_prev_w[6k:6(k+1)]`
  from the last successful plan. Its singleton-emission case (k=1)
  produced the cmd_vel zigzag at the tick-71 jerks recorded in
  [mpc_20260520_115857.csv](../temp/logs/mpc_20260520_115857.csv) — see
  the prior probe analysis in [tools/probes/replay_hardware_csv.py](../tools/probes/replay_hardware_csv.py)
  and the residual flagged in
  [2026-04-18-mpc-overhead-spikes-fallback-bursts.md](2026-04-18-mpc-overhead-spikes-fallback-bursts.md).
- `hold_extrap` arm (was at lines 1724-1746) — emitted `q_cur + q_dot·dt0`,
  the positive-feedback source documented in this entry.
- W7 walk-forward-unsafe block (was at lines 1655-1682) — the 20 mm
  ref-shift check and twist-direction-reversal check are no longer
  needed because cmd-stream extrapolation doesn't read the reference
  during fallback (the property W7 was protecting is now structural).
  The 500 ms staleness check is retained as the time-bound escalation
  trigger (`cascade_too_long`).
- The `'hold'` legacy status (frozen-cmd cold-freeze arm).

**Preserved**:
- The non-finite-`q_cur` sanitisation at fallback entry (Tier-1c bugfix
  from [2026-05-11-tier1c-input-fuzz-bugfix.md](2026-05-11-tier1c-input-fuzz-bugfix.md)
  is unaffected; `q_cur` still defends the cold_hold escalation path
  from NaN propagation).
- `_consecutive_failures` counter increment — still used by the warm-start
  deadlock-escape gate at solve() entry (commit `67ae3da`,
  [2026-05-20-mpc-warmstart-deadlock-escape.md](2026-05-20-mpc-warmstart-deadlock-escape.md)).
- Cold-bootstrap absolute fallback (`cold_hold(q_cur)` when `_prev_u` is
  None) — unchanged.

**New status families** (controller/DIAG_SCHEMA_CONTRACT.md updated):
- `fallback_extrap(...)` — primary arm.
- `fallback_hold(...)` — bootstrap arm when `_prev_prev_u` is None
  (only fires on the literal first failure after a single successful
  solve before `_prev_prev_u` is populated).
- `cold_hold(...)` — absolute fallback (unchanged semantics, now also
  the 500 ms time-bound escalation target).
- `hold_extrap(...)` and `hold(...)` are no longer produced but remain
  documented in DIAG_SCHEMA for historical-CSV compatibility (per the
  contract's "no removal" rule).

### Tests

New test `tests/sim/test_mpc_static.py::TestMPCSolverFailure::test_fallback_no_positive_feedback_growth`
— drives 10 consecutive fallbacks with synthetic non-zero
`leg_velocities_mmps = 100 mm/s` (chaos-arc transient) and asserts:
(a) no `hold_extrap` status emitted, (b) per-tick rate-limit honoured,
(c) step magnitude does not grow tick-over-tick (the safety property the
old `hold_extrap` arm violated).

New test `tests/sim/test_mpc_static.py::TestMPCSolverFailure::test_fallback_extrap_homogeneous_across_cascade`
— short-cascade homogeneity guard.

Updated tests (status string changes; same intent preserved):
- `test_diag_schema_fuzz.py` — two test classes renamed for clarity;
  schema invariant unchanged.
- `test_mpc_input_fuzz.py::_FALLBACK_PREFIXES` — added `fallback_extrap(`
  and `fallback_hold(`; one explicit `startswith('fallback(')` check
  broadened.
- `test_solver_failures.py::test_casadi_exception_routes_through_handler`
  — prefix check broadened.
- `test_solver_failures.py::TestStalenessEscalation` (was
  `TestWalkForwardStaleness`) — expected status changed from
  `hold_extrap` to `cold_hold` (the new 500 ms escalation target).
- `test_solver_failures.py::TestHomogeneousCascadeEmission` (was
  `TestMaxConsecutiveFailuresEscalation`) — rewritten to test the new
  homogeneous behaviour.
- `test_mpc_adversarial_sequences.py::TestScenario13_FallbackSurvivesTargetReversal`
  (was `TestScenario13_WalkForwardOldDir`) — W7-specific
  `hold_extrap`-engagement check replaced with a recovery-only check.

Deleted tests (mechanisms no longer exist):
- `TestWalkForwardRefShiftThreshold` (both `test_above_threshold_escalates_to_hold_extrap`
  and `test_below_threshold_stays_in_fallback`) — the W7 20 mm ref-shift
  escalation logic is removed.

### Offline-probe validation

Replayed [mpc_20260520_220030.csv](../temp/logs/mpc_20260520_220030.csv)
(the chaotic CSV) through the patched MPC via
[tools/probes/replay_hardware_csv.py](../tools/probes/replay_hardware_csv.py).
During the active `fallback_extrap` phase (ticks 3-19, 17 ticks before
the 500 ms time bound fires):
- `cmd_step`: **0.345 mm constant** every tick (smooth continuation of
  the prior cmd-stream slope at constant velocity).
- `d_cmd_vel`: **0.0 mm/s** (linear extrapolation has zero acceleration
  by construction).

Compared to the recorded chaos signature on the same hardware state:
- Recorded `cmd_step` p50 = 1.66 mm, max = 3.5 mm (rate-limit saturated).
- Recorded peak `leg_vel` = 250.6 mm/s (vs 140 mm/s soft limit).

After the 500 ms time bound, the probe escalates to `cold_hold(q_cur)`
and inherits whatever the recorded plant motion was — but that's a probe
artefact (the plant state is replayed, not simulated). In hardware with
the new code, the plant should not reach the chaotic state in the first
place.

### Verification

- **Targeted re-run** of the 5 touched test files
  (`pytest tests/sim/test_solver_failures.py tests/sim/test_mpc_static.py
  tests/sim/test_mpc_input_fuzz.py tests/sim/test_diag_schema_fuzz.py
  tests/sim/test_mpc_adversarial_sequences.py -q`, run 2026-05-21):
  **206/206 pass in 114.84 s** (1 xfailed; ci-fast hypothesis profile).
- **Full suite** (`pytest tests/ -q`, run 2026-05-21 — post-audit-fixes
  pass): **1428/1428 pass in 433.04 s** (1 xfailed; ci-fast).  One
  prior run on the same code had a transient failure of
  `tests/sim/test_hot_loop_allocation_contract.py::test_hot_loop_allocation_contract`
  which the test docstring identifies as load-flaky on a busy Jetson —
  passed cleanly in isolation (`pytest tests/sim/test_hot_loop_allocation_contract.py -v`,
  run 2026-05-21: **3/3 pass in 16.04 s**) AND in two subsequent
  full-suite runs (424.39 s pre-audit; 433.04 s post-audit).

## Outcome

*To be filled after hardware re-run validates the fix. The operator will
re-run the lateral-move sequence that triggered the chaos and assert:*
- *Peak `leg_vel` ≤ 1.2× the 140 mm/s soft limit (was 336.9 mm/s).*
- *No `MAX_DEVIATION` E-stops.*
- *No `hold_extrap` status in the recorded CSVs (the deleted arm).*
- *For every fallback tick: `cmd_vel` magnitude ≤ `v_max = 140 mm/s`.*

*Status flip to `resolved` (if validated) or `tuned` (if a residual
remains) gets handled in the same session as the hardware validation.*

## Open Questions

- What is the root cause of the cumulative `solve_time` slowdown across the
  session? (Tracked under the separate Jetson load-profiling investigation;
  not in scope here, but a complete answer to this safety event requires
  understanding both the trigger and the amplifier.)
- Should `max_leg_vel_mmps` be promoted from an NLP soft bound to a hard
  motor-guard / actuator-side clamp? Peak 336.9 mm/s on a 140 mm/s limit
  argues yes, but this needs its own investigation — clamps in the wrong
  place can hide control-loop bugs.
- Does the cmd-stream extrapolation primitive (Tier 1 direction) handle the
  trajectory-onset case as well as the chronic-CTE case, or does it merely
  trade one failure mode for another? Probe replay against the existing
  hardware CSVs (especially `mpc_20260520_115857.csv` tick 71) should
  answer this before the fix lands on hardware.
