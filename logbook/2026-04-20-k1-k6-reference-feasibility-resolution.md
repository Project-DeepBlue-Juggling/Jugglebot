---
title: "K1–K6 reference-feasibility contract resolves MPC_OVERSHOOT_SATURATION"
type: investigation
date: 2026-04-20
status: resolved
phase: "hardware-bringup — Phase 4 (moderate motions)"
related_plan: "plans/active/hardware-bringup.md"
related_issues:
  - MPC_OVERSHOOT_SATURATION
related_entries:
  - 2026-04-18-move5-overshoot-stall-and-plant-collapse
  - 2026-04-19-bundle-a-mpc-overshoot-saturation-fix
  - 2026-04-19-bundle-a-quintic-ref-settling-and-live-twist-trap
sessions:
  - mpc_20260420_160049.csv   # 223902 reproduction under fix
  - mpc_20260420_160120.csv   # simple z=220 baseline
  - mpc_20260420_160159.csv   # simple z=170 baseline
  - mpc_20260420_160242.csv   # rapid-succession W7 workout
files_changed:
  - controller/feasibility.py
  - controller/target.py
  - controller/zmq_target.py
  - controller/scheduler.py
  - controller/mpc.py
  - controller/runner.py
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py
  - run_mpc.py
  - sim/main.py
  - sim/analysis/record_baselines.py
  - sim/analysis/known_issues.yaml
  - controller/REFERENCE_LAYER_CONTRACT.md
  - tests/sim/test_make_feasible_events.py
  - tests/sim/test_zmq_target.py
  - tests/sim/test_mpc_adversarial_sequences.py
  - tests/sim/test_cold_start_fixes.py
commits:
  - e8c5833   # W1–W11 landing + audit fixes
  - e656673   # contract doc relocation
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

# K1–K6 reference-feasibility contract resolves MPC_OVERSHOOT_SATURATION

## Summary

The 2026-04-19 hardware session `mpc_20260419_223902` produced a violent
mid-frequency oscillation and an operator E-stop on the two-target sequence
`(0,0,220)@0 → (0,0,170)@1.2s`.  Root cause: at the target flip the
reference source (`StaticTargetSource` via `flat_target_to_events`) emitted
a quintic whose peak interior velocity was **177 mm/s** against an actuator
limit of 140 mm/s.  IPOPT could not reconcile the ref vs rate-constraint
within the 22 ms CPU budget → 147 consecutive timeouts → walk-forward
committed to the old upward plan while the flipped target wanted downward
→ divergence → oscillation → E-stop.

The W1–W11 cycle landed a reference-feasibility contract (K1–K6) that
eliminates the whole class of failures at their source: every reference
emitted by every `TargetSource` now satisfies bounded peak velocity and
acceleration by construction, anchors at the live plant state on every
distinct target transition, clamps FK-derived twist spikes, and signals
the MPC to cold-start through structural-shift transitions rather than
warm-starting from a stale plan.

Hardware validation on 2026-04-20 reproduced the **exact** 223902 geometry
cleanly — **5 max-consecutive timeouts** (down from 147), **96.2 % success
rate** (up from 6.2 %), and **no E-stop**.

## Symptoms

From the 2026-04-19 session:
- Platform at (0, 0, 210) with +60 mm/s upward velocity when target flipped from (0, 0, 220) to (0, 0, 170).
- IPOPT reported `Maximum_CpuTime_Exceeded` on 147 consecutive solves (93.8 % of session ticks).
- Walk-forward fallback emitted commands from the old upward plan; plant continued rising to ~229 mm while target was 170 mm.
- Coupled oscillation between fallback commander and ODrive PID; operator E-stopped at CSV t=3.67 s.
- Session diagnostic flagged `MPC_OVERSHOOT_SATURATION` adjacent signature (max_consec_timeout=5 in the 13:49–13:52 session; crossed threshold on 20:00 session with 147 consec).

## Diagnosis

The quintic peak-velocity analysis for the 223902 boundary conditions —
`(p0=210, v0=+60 mm/s, p1=170, v1=0)` over `T=0.5 s` — yields peak
`|v_z| = 177.5 mm/s` (verified closed-form in
[controller/feasibility.py](../controller/feasibility.py) against a 2001-point
dense scan to 6 decimal places).  The reference was physically infeasible
for the actuators from the first tick of the transition; IPOPT was being
asked to minimise the cost of an unreachable target.

Diagnosis narrative, refinements, and Day-2 design discussion:
[2026-04-19-bundle-a-mpc-overshoot-saturation-fix](2026-04-19-bundle-a-mpc-overshoot-saturation-fix.md)
and the prior investigation
[2026-04-18-move5-overshoot-stall-and-plant-collapse](2026-04-18-move5-overshoot-stall-and-plant-collapse.md).

The deep root cause is **not** any specific ref source or the MPC solver,
but the absence of a shared feasibility contract at the
reference-production boundary.  W1–W11 introduces that contract
([controller/REFERENCE_LAYER_CONTRACT.md](../controller/REFERENCE_LAYER_CONTRACT.md))
as six normative invariants (K1–K6) enforced in one place
(`make_feasible_events`), with every built-in `TargetSource` migrated to it
and the MPC given hint-and-hardening channels to react appropriately when
the reference structurally shifts.

## Discussion

The Day-2 architectural review (see Sonnet's fix-proposer outputs, 2026-04-19)
enumerated 14 failure-mode scenarios in this broader class and decomposed
the fix into 11 work items:

| # | Scope |
|---|-------|
| W1 | `make_feasible_events` — K1–K6 enforcement, stretch binary-search |
| W2 | Property + scenario tests (hypothesis-based, 23 cases) |
| W3 | `flat_target_to_events` becomes a compat wrapper |
| W4 | Migrate every source (Static/Waypoint/Auto/Zmq + scheduler defense-in-depth) |
| W5 | `TargetCommand.warm_start_valid` hint + MPC invalidation |
| W6 | Per-node-IK `_cold_start` with 30 % budget guard |
| W7 | Fallback hardening: 20 mm shift / velocity reversal / 500 ms staleness gates |
| W8 | Adversarial MuJoCo sim fixture (15 cases covering all 14 scenarios) |
| W9 | AOT `_nlp_source_hash` extended to include ref-layer source files |
| W10 | `controller/REFERENCE_LAYER_CONTRACT.md` — normative K1–K6 spec |
| W11 | 3-field `TargetFeedback` message for catch rejection / stretch-warning |

All 11 items landed in commit `e8c5833` with the contract-doc relocation
in `e656673`.  Tests: **1009 passed, 0 failed** (baseline pre-cycle was
959; +50 new tests across W2/W5/W8/W11, zero pre-existing regressions).
The adversarial fixture includes the exact 223902 geometry as
`TestScenario02_DirectionReversal.test_mid_ramp_reversal_no_cascade`,
which asserts zero fallback saturation in sim — this is now the
regression floor.

## Fix

Seven work items touched controller code, three touched tests, one touched
docs, and one touched the ROS2 IPC.  The commit message summary lives in
`e8c5833`; this entry captures the hardware-outcome perspective.

Key architectural moves:

1. **Every reference goes through ``make_feasible_events``** — this is the
   single K1–K6 enforcement point.  The function accepts a caller's desired
   events, clamps twists (K6), anchors at live plant state (K1), enforces
   strictly-increasing times (K4) and twist consistency (K5), and binary-
   searches a stretch factor to bring peak velocity (K2) and acceleration
   (K3) within their β·v_max bounds.  β=0.85.
2. **Catch targets set ``no_stretch=True``** — ``make_feasible_events``
   refuses to silently delay a hard-deadline catch; instead returns a
   rejection reason, and ``ZmqTargetSource`` continues the last feasible
   quintic and publishes a ``rejected_infeasible`` feedback message to the
   catch coordinator.
3. **Large structural ref shifts invalidate the warm-start** — sources
   detect >20 mm shift or velocity reversal at the horizon end and set
   ``warm_start_valid=False`` on their ``TargetCommand``.  The MPC clears
   ``_prev_w`` / ``_timeout_hint`` for that tick and cold-starts.
4. **Cold-start is cheaper and closer** — W6 per-node IK seeds give IPOPT
   a near-feasible initial guess; measured `_numerical_ik` p95 is 118 µs
   on Jetson, so (N+1) IK calls ≈ 1.3 ms (6 % of the 22 ms cap).
5. **Fallback hardening skips walk-forward on structural shift** — W7 in
   `_handle_failure` detects large ref-shift / velocity reversal / 500 ms
   staleness and switches to plant-tracking `hold_extrap` instead of
   marching an old plan forward.
6. **AOT .so auto-invalidates on ref-layer changes** — W9 extends the
   `_nlp_source_hash` to cover `target.py` and `feasibility.py`, so a
   refactor of the reference-layer semantics forces an AOT rebuild rather
   than silently drifting.

See [controller/REFERENCE_LAYER_CONTRACT.md](../controller/REFERENCE_LAYER_CONTRACT.md) for the
normative spec; implementations of new `TargetSource` subclasses should
start there.

## Verification

### Unit + integration tests
`pytest tests/ -v` — **1009 passed, 52 warnings, 0 failed** (prior baseline
959 → +50 new).  The adversarial fixture
`tests/sim/test_mpc_adversarial_sequences.py` covers all 14 failure
scenarios from the table against a live `MuJoCoPlant` through `run_mpc_loop`.

### Hardware validation (four sessions, session group 2026-04-20_16-00-31)

| CSV | Scenario | p50 ms | max consec TO | success % | E-stop | Verdict |
|-----|---------|-------:|-------:|-------:|:-----:|---------|
| `mpc_20260420_160049` | **223902 reproduction** | **10.5** | **5** | **96.2 %** | NO | MAJOR WIN |
| `mpc_20260420_160120` | simple z=220 hold 20s | 10.6 | 0 | 100 % | NO | baseline unchanged |
| `mpc_20260420_160159` | simple z=170 hold 20s | 10.7 | 1 | 99.9 % | NO | baseline unchanged |
| `mpc_20260420_160242` | 4-target rapid-succession | 10.7 | 1 | 99.2 % | NO | W7 workout PASSES |

Pre-fix baseline for the 223902 geometry (`mpc_20260419_223902.csv`):
p50 = 26.0 ms, max consec = 147, success = 6.2 %, E-stop required.

Deltas on the exact same geometry:
- **p50 solve time 26.0 → 10.5 ms (−59 %)**
- **max consecutive timeouts 147 → 5 (−97 %)**
- **success rate 6.2 % → 96.2 % (+90 pts)**
- **E-stop: YES → NO**
- `overshoot_saturation` diagnostic signature no longer fires.

At the target-flip moment (CSV sample 13, t=1.274 s) the K1 anchor is
visible directly in the telemetry: `ref_pose_z == actual_pose_z == 174.25`
exactly — the new quintic was rebuilt from the live plant state at that
tick.  Pre-fix, this transition was 147 consecutive fallback ticks in a
row.

## Outcome

`MPC_OVERSHOOT_SATURATION` flipped from `active` → `fixed` in
[sim/analysis/known_issues.yaml](../sim/analysis/known_issues.yaml);
`fixed_date: 2026-04-20`; logbook-entry reference updated to this file.

### Open follow-ups (not introduced by this work, flagged in hardware sessions)

- **Hold-phase steady-state RMS 2 mm** on both simple-hold sessions
  ([160120](../temp/logs/mpc_20260420_160120.csv),
  [160159](../temp/logs/mpc_20260420_160159.csv)) — the known
  [leg-1 pose-dependent hold-twitch](2026-04-19-leg1-pose-dependent-hold-twitch.md)
  investigation (resolved same day for leg 1 via PID revert; leg 4 still
  shows the same signature and is a candidate for the same revert).
- **Motion-onset dead-time 180–253 ms** — stick-slip / backlash
  signature, pre-existing, cross-session consistent.
- **Isolated GC spikes 50–67 ms** — single-sample overhead blips, pre-
  existing, tracked in the 2026-04-18 overhead-spikes investigation.

These are **unrelated to `MPC_OVERSHOOT_SATURATION`** and remain open.

### Global MPC-controller robustness observations

The K1–K6 contract closes the whole class of "infeasible reference →
solver saturation" failures.  What remains on the path to robust dynamic
juggling — each bounded by a specific hardware-observable:

1. **Hold-phase micro-jitter (leg-level)** — per-leg PID gain tuning.
   Investigation arc well-established; leg 1 resolved today, leg 4 next.
2. **Actuator dead-time / stick-slip at motion onset** — 180–253 ms
   consistently across all 2026-04-20 sessions.  Unlikely to be fixable
   in the MPC layer; candidates are static-friction compensation in the
   motor guard's feedforward, or lighter pre-load on the couplers.
3. **GC pauses on the Jetson** — 50–67 ms overhead spikes force
   walk-forward fallbacks even with K1–K6 in place.  Mitigations: pre-
   allocate all per-tick buffers (partially done), disable generational
   GC in the control loop and run explicit collections between trajectories.
4. **Catch-coordinator rejection semantics** — the W11 plumbing is in
   place but the downstream coordinator still treats `accepted=False` as
   a simple blacklist add.  Richer re-planning on `rejected_infeasible`
   (retarget with stretched arrival, notify ball Butler to re-throw) is
   a good next step once catch performance is measured.
5. **Scheduler path K1–K6 enforcement** — currently defense-in-depth
   (`_verify_segment_feasibility` logs warnings; with `strict_feasibility=True`
   raises).  The full catch / toss-loop integration with K1–K6 is
   untested on hardware because we haven't run catch trajectories since
   the contract landed.
