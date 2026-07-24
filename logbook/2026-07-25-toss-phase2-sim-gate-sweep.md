---
title: "Single-ball toss Phase 2: sim/toss_gate.py production-in-the-loop gate — Tier-8a sweep PASSES both binding bands"
type: feature
date: 2026-07-25
status: resolved
phase: "MVP trajectory bringup — Phase 8 / single-ball toss Phase 2 (sim gate + Tier-8a sweep)"
related_plan: single-ball-toss.md
subsystem: sim
tags: [feature, testing, sim]
commits:
  - PENDING
files_changed:
  - sim/toss_gate.py
  - sim/gate_common.py
  - sim/reload_gate.py
  - tests/sim/test_toss_gate.py
---

# Single-ball toss Phase 2: sim toss gate + Tier-8a sweep

## Summary

`sim/toss_gate.py` lands — the production-in-the-loop gate for the Tier-8a
toss, a sibling of `reload_gate.py` sharing a new minimal `sim/gate_common.py`
(extraction grep-verified; `test_reload_gate.py` unmodified and green). The
full sweep (29 points × 10 trials + 20 contact-diagnostic trials, seed 0,
552.9 s wall) **PASSES both binding bands**: the plan's 2–3 m/s band 10/10 at
all 12 points, and the orchestrator-amended T=0.80 s @ z=170 band (the
hardware-relevant flights) 10/10 at four points and 9/10 at one — with zero
feasibility violations and 45,950-frame pump acceptance at 100 % (every emitted
knot accepted, now asserted non-vacuously). The advisory full band FAILS in
the long-flight tail (T ≥ 0.95 s), which is the expected signature of the
placeholder release-noise model — details in Discussion. The adversarial
review caught a BLOCKING gate-honesty hole before the sweep ran: a trial with
the catch-arm path completely dead could still pass `core_clean` (ball falls
into the statically-parked cup) — `core_clean` now requires the catch to have
actually armed.

## Changes

- **`sim/toss_gate.py`** (new): per-trial flow = production
  `planner.build_move` pre-position streamed through `KnotEmitter` + a real
  `SetpointPump` onto the MuJoCo plant (arrival asserted; worst pre-position
  error in the sweep 0.08 mm) → self-announcement via the real
  `motion/trajectory/toss_release` functions (hardware-frame defaults;
  announced-vs-actual deltas recorded, never reconciled — worst announced
  landing error −43.0 ms) → pre-tilt `build_catch` timed to the announced
  landing (second pump; handoff continuity asserted within
  `JB_OP_MAX_POSITION_STEP_REV`) → `HandThrowSequence` stroke with a seeded
  kinematic release at the computed release state (`Ball.release` — the
  mode-correct ejector for the `contact_carry=False` gating plant; see
  Discussion) + 1 % placeholder fractional release-velocity noise → in-plant
  flight observed through `JuggleNoise.observe` + `BallisticEstimator`
  driving the arm-and-forget `HandCatchSequence` (armed ONCE; per-trial
  `catch_armed` recorded and REQUIRED by `core_clean`) → seat/quiescence
  verdict. Gate = both binding bands at ≥⌈0.9 n⌉ AND zero feasibility
  violations AND pump accepted == emitted; empty-band runs gate on the full
  band with a loud `NO BINDING POINTS` banner (no vacuous PASS). JSON report
  to `temp/reports/toss_gate_seed{s}_n{n}.json` with per-point binding flags,
  the `hardware_marginal_flight` (<0.7 s) annotation, and the non-gating
  contact-physics diagnostic column (`Ball.ballistic_release` runs verbatim there in its opt-in kinematic mode via `--diag-release kinematic`; the recorded sweep's diag column ran the default detach mode).
- **`sim/gate_common.py`** (new): viewer trio, travel/tilt helpers, shared
  constants + thresholds extracted from `reload_gate.py` (−79/+17 lines
  there; the only importer of `reload_gate` was its own test — grep-verified
  before moving).
- **`tests/sim/test_toss_gate.py`** (new): 12 CI tests — 4-trial
  module-scoped smoke (verdict fields, caught ⇒ `catch_armed` ∧ finite
  `capture_rel_ms`, pump accepted == emitted > 0, report well-formedness
  incl. `pass_9_of_10` = (core_clean ≥ ⌈0.9 n⌉), `_pass_threshold` integer
  pins, gating plant `contact_carry is False` / diag plant `is True`), plus
  the corner-pose announcement→`build_catch` wire-crossing test deferred
  from Phase 1 (asserts a non-None `Setpoint` per frame — the pump's silent
  `(None, None)` no-position path made the acceptance invariant vacuous
  otherwise, an inherited hole that also exists in `reload_gate`'s test;
  noted as a follow-up there).

## Sweep result (the phase gate)

`python sim/toss_gate.py` (full factored grid, seed 0, run 2026-07-25;
552.9 s wall; report `temp/reports/toss_gate_seed0_n290.json`):

| Point (x, y, z, T) | core_clean | Binding | Verdict |
|---|---|---|---|
| centre + 4 corners @ z=170, T=0.55 | 10/10 ×5 | 2–3 m/s (hw-marginal) | PASS |
| centre + 4 corners @ z=170, T=0.60 | 10/10 ×5 | 2–3 m/s (hw-marginal) | PASS |
| (0,0,140) and (0,0,200), T=0.60 | 10/10 ×2 | 2–3 m/s (hw-marginal) | PASS |
| centre + 3 corners @ z=170, T=0.80 | 10/10 ×4 | T=0.80 @ z170 | PASS |
| (−60,60,170), T=0.80 | 9/10 | T=0.80 @ z170 | PASS |
| (0,0,140/200), T=0.80 | 9–10/10 | advisory | pass |
| all points, T=0.95 | 7–10/10 | advisory | 3 points <9 |
| all points, T=1.10 | 5–8/10 | advisory | 5 points <9 |

Headline: **PASS** (both binding bands) — `core_clean` 262/290 overall,
`caught` 262, `accepted` 290/290, `feas_viol` 0, `pump_rejects` 0, pump
frames accepted == emitted; worst hold travel 0.02 mm / tilt 0.00°; required
leg envelope vel 51 mm/s, acc 111 mm/s², jerk 803 mm/s³ (far inside session
limits). Velocity-match is a deferred criterion (as in `reload_gate`; worst
0.479). Contact-diagnostic column (non-gating, detach mode): 8/20
seat-caught, landing error worst 4.2 mm — aim is fine; the seat/carry is the
knife-edge, consistent with the Rung-2b finding that the sim contact model,
not the throw, is the low-fidelity element.

## Discussion

**The advisory full-band failure is signal, not noise.** Every failing point
is at T ≥ 0.95 s (throw speed 4.7–5.4 m/s); the binding bands (T ≤ 0.80)
pass 169/170. Under the placeholder fractional velocity noise (1 %, the
plan's explicit placeholder until Phase-5 T0 measures real scatter), lateral
landing drift scales as v_err·T ∝ v·T — the tall tosses drift 40–95 mm
(worst landing error 90.7 mm) against the 80 mm-class reach envelope. Three
readings: (a) the gate criterion the plan pinned (2–3 m/s) plus the
amendment band pass robustly; (b) IF the real hand's release scatter is
fractional-velocity-like at ~1 %, tall tosses (T2's 1.1 s rung) will be
envelope-limited on hardware — T0's measured noise magnitude re-runs this
gate before any hardware catch attempt, exactly as the plan pre-registered;
(c) nothing was tuned to make the pass happen — the noise model and
thresholds are the spec's, and the failing tail is reported, not trimmed.

**Why `Ball.release`, when the plan names `Ball.ballistic_release`.** The
locked decision is the *kinematic release mechanism*; `ballistic_release`'s
own contract (manager.py) requires a contact-carried ball and names
`release` as the kinematic-hold-mode analogue. The gating plant is
`contact_carry=False` — chosen over carry mode for four failure-mode
reasons (sim-contact fidelity confound per the plan's own "the sim contact
model is the low-fidelity element", the untested carry-mode seat/verdict
combination, carry-slosh artifacts, and column comparability with
`reload_gate`) — so the gating column uses the mode-correct ejector while
the plan-named `ballistic_release` runs verbatim in the contact-diagnostic
column, in its contract mode. Mechanism preserved; function name deviation
documented here rather than silently absorbed.

**The amendment band exists because the plan's binding band is
hardware-marginal.** Phase 1's control analysis found the firmware kind-1
windup-budget race makes flights <0.7 s a silent-drop risk on hardware; the
plan's 2–3 m/s criterion binds entirely at T ≤ 0.61 s. Gating additionally
on T=0.80 @ z=170 (the flights hardware T1 will actually fly) strengthens
the gate; both bands are reported separately in the JSON so neither
dilutes the other.

**What the adversarial review caught before the sweep ran.** (1) BLOCKING,
empirically probed: `core_clean` was earnable with the catch-arm path dead —
the ball fell into the statically-parked cup and the capture model latched
it; any regression silently killing the arm path (the exact tracker-analogue
seam this gate exists to exercise) would have gate-passed. `core_clean` now
requires `catch_armed`. (2) A vacuous PASS on empty binding bands
(single-point debug runs exited 0 with a PASS banner at 0/10 caught).
(3) The pump-acceptance invariant was vacuous: `SetpointPump.build` has a
silent no-position `(None, None)` path that increments nothing — the tests
now assert a non-None `Setpoint` per frame and the harness asserts
accepted == emitted. The same vacuity exists in `reload_gate`'s test —
follow-up noted. (4) A `_summarise` crash path that lost the entire run's
report if any point's release state was invalid. All fixed pre-sweep.

**Fidelity deltas recorded, not hidden** (module docstring lists all):
announced-landing timing error −43 ms class (the announcement is built from
commanded release; the real release differs — hardware analog is T0's
latency slot); release-frame z delta ≈ +45 mm (hardware-frame release plane
vs the sim hand's throw-end); the z-sweep tracker-plane skew (~7.6 ms at
z=200 — the gate models a better tracker than production's fixed-plane
predictor, flagged on the advisory z rows only; binding rows are at z=170
where it is zero).

## Verification

- Scoped (2026-07-25, Jetson venv):
  `pytest tests/sim/test_toss_gate.py tests/sim/test_reload_gate.py -q` →
  **22 passed in 20.68 s** (12 + 10; `test_reload_gate.py` UNMODIFIED —
  the `gate_common` extraction is behaviour-neutral).
- Full sweep: quoted above with the (date, command, result) triple.
- ci-deep (Phase 2 exit gate) —
  `pytest tests/ -q --hypothesis-profile=ci-deep`, run 2026-07-25 against
  the final post-audit tree: **3330 passed, 3 xfailed, 198 warnings in
  3802.33 s (1:03:22)** (+12 toss-gate tests over the Phase-1 3318
  baseline; same 3 xfails).
- `/audit` (pre-commit, 2026-07-25): no unfixed findings — 2 BLOCKING
  narrative-number errors this entry originally carried (pump frames
  "4405" vs the JSON's 45,950; binding-band arithmetic "129/130" vs
  169/170), 2 WARNINGs (a miscounted advisory row — 3 not 4 points <9 at
  T=0.95; the diag column's 8/20 result mis-attributed to
  `ballistic_release`, which never executed in the recorded detach-mode
  run), and 4 NOTEs (band membership now purely geometric so a
  fully-rejected binding point cannot vacate its band; empty-points runs
  can never PASS; two stale docstring line refs; a plan cross-reference
  for the `Ball.release` delta). All fixed-and-landed pre-commit; the
  audit independently re-verified every other quoted number against the
  sweep JSON and the `gate_common` extraction symbol-by-symbol.
