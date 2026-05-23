---
title: Hand trajectory generator overhaul — Phase 2 jerk-limited profile family design + offline Python reference
type: feature
date: 2026-05-23
status: resolved
phase: "hand-trajectory-generator-overhaul — Phase 2"
related_plan: "hand-trajectory-generator-overhaul.md"
related_entries:
  - 2026-05-22-hand-generator-phase1-characterisation
  - 2026-05-23-catch-vel-ratio-port-reconciled
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - tools/probes/hand_jerk_limited_prototype.py
  - tools/probes/README.md
  - tests/sim/test_hand_jerk_limited_prototype.py
  - logbook/2026-05-23-hand-generator-phase2-jerk-limited-design.md
  - logbook/INDEX.md
  - plans/active/hand-trajectory-generator-overhaul.md
commits:
  - 89dc933
  - 2d57f27
subsystem:
  - hand
  - teensy
tags:
  - design
  - trajectory
  - profile-family
  - jerk-limited
---

# Phase 2 — jerk-limited profile family design + offline Python reference

## Summary

Phase 2 selects and specifies the new throw/catch profile family — a
**symmetric 3-segment quintic-linear-quintic** profile with C2 (acceleration-
continuous) joins — and produces a complete offline Python reference
implementation that Phase 3 will port into firmware + sim port in lockstep.

The chosen design:

* Three segments per event: **accel-quintic → cruise (linear) → decel-quintic**,
  symmetric (`T_a = T_d`).
* Reparameterised inputs: `(cruise_vel, duration, stroke_lo, stroke_hi)`
  per the plan §4 Phase 2.
* Bounded jerk by construction; analytic peak `|a| = 1.5·v/T_a` and peak
  `|j| = 6·v/T_a²`.
* New `hardware_config.yaml` field `teensy_trajectory.max_event_hand_accel_rps2 = 6000.0`
  (the binding bound), anchored on the Phase 1 measurement.
* Time-budget slack (`T > T_min`) monotonically reduces **both** peak accel
  and peak jerk — verified by a sweep test.

The prototype lives at `tools/probes/hand_jerk_limited_prototype.py`
(committed reference, not yet wired into `sim/hand/`); an 18-test offline
validation suite at `tests/sim/test_hand_jerk_limited_prototype.py` (98
test items after parametrisation) pins every invariant Phase 3 must
reproduce sample-for-sample.

## Motivation

Phase 1 characterised the legacy generator's failure modes:
piecewise-constant acceleration → step discontinuities up to 191 m/s² per
segment boundary (unbounded jerk), and a backwards `peak accel ∝ v²` /
`duration ∝ 1/v` scaling that pinned the generator to a single fast-or-
nothing operating point. Phase 2's job is to choose the family that closes
both — bounded jerk *and* a sensible `(v, T)` tradeoff surface — without
giving up the on-Teensy real-time generation constraint.

## Design

### Choice of family

Three candidate families were considered:

| Family | Jerk profile | Per-segment cost | Within-ramp smoothness | Reaches v=7 at A_peak = 6000 rev/s² ? |
|--------|--------------|------------------|------------------------|---------------------------------------|
| Legacy (piecewise-constant accel) | Dirac deltas at every boundary (**unbounded**) | Trivial | N/A | Yes — the status quo. |
| **3-quintic (chosen)** | Linear in τ within each ramp; piecewise-continuous at cruise joins | Cubic-polynomial eval per sample | Continuous (linear) | **No** — v_max ≈ 6.31 m/s at this bound. |
| 7-segment S-curve | Piecewise-CONSTANT across 7 sub-segments | Cubic-polynomial eval per sub-segment | Constant within each (discontinuous between) | Yes — preserves v=7. |
| Septic (C3 at joins) | Continuous everywhere | 7th-order polynomial per ramp | Continuous everywhere | Same envelope as quintic at chosen bound. |

The user selected **3-quintic at the legacy-matching 6000 rev/s² bound**,
explicitly accepting the v_max drop from 7.0 to ~6.3 m/s in exchange for
the within-ramp jerk smoothness the quintic delivers. The decision was
made knowing the math (raised in-session as the
"physical intuition disagrees with framing" check) — the alternative paths
(raise the bound to ~7500 rev/s² to preserve v=7, or switch to S-curve)
were rejected on, respectively, hardware-headroom uncertainty and the
S-curve's piecewise-constant jerk shape.

### The math

**Accel quintic** — from rest at `stroke_lo` to cruise velocity `v` over
ramp duration `T_a`, with BCs `(pos, vel, accel) = (stroke_lo, 0, 0) →
(stroke_lo + d_a, v, 0)`. With `τ = t / T_a` and `q(τ) = p(t) / d_a`:

  q(τ) = 2·τ³ − τ⁴
  q'(τ) = 6·τ² − 4·τ³            (peak 2 at τ=1)
  q''(τ) = 12·τ − 12·τ²           (peak 3 at τ=1/2)
  q'''(τ) = 12 − 24·τ              (peak ±12 at τ=0, 1)

Distance covered: `d_a = v · T_a / 2` (the symmetric-velocity-profile
identity).

In real units:

  |a|_max = (d_a / T_a²) · 3 = (v·T_a/2 / T_a²) · 3 = **1.5 · v / T_a**
  |j|_max = (d_a / T_a³) · 12 = **6 · v / T_a²**

**Decel quintic** — time-reverse: `r(τ) = 2·τ − 2·τ³ + τ⁴`. Same peak
magnitudes, mirrored sign pattern.

**Cruise** — linear, jerk = 0, accel = 0; trivially C2 with the quintics
because the quintic BCs match `(vel = v, accel = 0)`.

### Feasibility envelope

Given total duration `T`, stroke `s`, cruise velocity `v` (all signed, same
sign):

  T_a = T − s/v                    (from the symmetric-ramp identity)
  T_c = 2·s/v − T                  (cruise duration)

Constraints: `T_a ≥ T_a_min = 1.5·v/A_peak` (peak-accel) and `T_c ≥ 0`
(cruise non-negative). The first gives `T ≥ T_min = s/v + 1.5·v/A_peak`;
the second `T ≤ T_max = 2·s/v`. Both hold iff `v² ≤ s · A_peak / 1.5`,
which gives the **v_max** ceiling. At the configured `max_event_hand_accel_rps2 = 6000`
(≈ 189.77 m/s²) and full effective stroke 0.315 m: **v_max ≈ 6.313 m/s**.

### Time-budget slack

When `T > T_min`, `T_a > T_a_min`, and:

  peak |a| = 1.5·v / T_a  →  *decreases as T grows*  (∝ 1/T_a)
  peak |j| = 6·v / T_a²   →  *decreases as T grows*  (∝ 1/T_a²)

Both bounded quantities respond monotonically to slack — exactly the
"spend slack on lower jerk" semantics the plan §1 calls for. Empirical
sweep at `v = 3.0 m/s` (where the envelope is wide): T_min = 0.129 s gives
peak |a| = 189.8 m/s² and peak |j| = 32 011 m/s³; mid-envelope T = 0.169 s
gives peak |a| = 69.9 m/s² (−63 %) and peak |j| = 4 346 m/s³ (−86 %).
Both monotone-decreasing across the sweep (`test_tu6_*`).

### Jerk discontinuities — the honest accounting

The quintic is **C2 globally** (position + velocity + acceleration
continuous everywhere) but **not C3**: jerk steps to zero at each cruise
join. Per half-event:

* Within each quintic ramp: jerk continuous and bounded (linear in τ;
  peak magnitude `6·v/T_a²` at τ = 0 and τ = 1).
* At each cruise join: jerk drops from `±6·v/T_a²` to `0` — **a single
  step-discontinuity** of magnitude equal to the in-ramp peak.

This is fewer jerk steps than the 7-segment S-curve (which has six per
half-event, each of magnitude `J_max`) and they are at predictable,
well-defined points — making them easier to handle in any downstream
filtering or controller-side compensation.

### `max_event_hand_accel_rps2 = 6000` — provenance

Anchored on the Phase 1 measurement: the legacy piecewise-constant
generator at v = 7 m/s reaches peak |a| ≈ 191 m/s² ≈ 6054 rev/s². The
configured `6000 rev/s²` (≈ 189.77 m/s²) sits a hair below today's
measured peak — preserving today's known-good hardware operating point.
Phase 4's hardware bring-up tunes empirically.

## Implementation

* **`config/hardware_config.yaml`** — added `max_event_hand_accel_rps2: 6000.0`
  under `teensy_trajectory`, with a comment recording the Phase 1
  provenance. Regenerated `config/generated/hardware_config.{py,h}` and the
  ros_ws + BallButler mirrored headers via `python config/generate_config.py`.
* **`tools/probes/hand_jerk_limited_prototype.py`** (new, ~500 LOC) —
  `JerkLimitedProfile` class, `feasibility()`, `solve_min_time()`,
  `make_throw()` / `make_catch()` factories, `sample_grid()` helper, and a
  `main()` that emits a 4-profile time-series PNG + JSON summary to
  `temp/probes/`. Reads `MAX_EVENT_HAND_ACCEL_RPS2` from the generated
  config module — single source of truth, regression-tested.
* **`tests/sim/test_hand_jerk_limited_prototype.py`** (new) — 18 test
  functions; with parametrisation across the velocity sweep the suite
  collects 98 items. Pins every invariant of the plan §5 T-U1..T-U7 list,
  the feasibility envelope (above v_max raises ValueError), and a
  determinism guard.

No production code in `sim/hand/`, `Trajectory.h`, `ros_ws/.../can_node.py`
or any CAN payload encoder was modified — Phase 2 stays offline, per the
plan's phase split.

## Verification

* Phase 1's inaugural baseline: 1443 passed + 1 xfailed (SHA `7d16e2b`).
* After Phase 1 + the catch_vel_ratio reconciliation: 1455 passed + 1 xfailed.
* **After Phase 2** (this entry):
  * Scoped: `pytest tests/sim/test_hand_jerk_limited_prototype.py -q`,
    run 2026-05-23 → **98 passed** in 0.29 s.
  * Full suite: `pytest tests/ -q`, run 2026-05-23 → **1553 passed,
    1 xfailed** in 452.84 s. Delta over the pre-Phase-2 baseline (1455 + 1xf)
    is exactly +98, matching the new test items — no regressions.
  * Hot-loop allocation contract: `pytest
    tests/sim/test_hot_loop_allocation_contract.py -q`, run 2026-05-23 →
    **3 passed** in 16.22 s (Phase 2 touches no hot-loop code; expected).
* Prototype is deterministic — `test_determinism_bit_identical_sampling`
  asserts re-evaluation produces identical tuples.

## Decision deferred to Phase 3 — CAN payload recommendation

The plan §4 Phase 2 gates one decision: whether the new `(v, T,
stroke_lo, stroke_hi)` parameterisation can be derived on-Teensy from the
existing `0x6D0` payload (`event_delay, event_vel, traj_type`), or whether
new CAN fields are required.

**Recommendation for Phase 3: extend `0x6D0` with two stroke fields,
derive `T` on-Teensy from `event_delay`.** Specifically:

* Keep `event_vel` → `cruise_vel`.
* Derive `T` on-Teensy as `max(T_min(v), event_delay − SAFETY_GAP)`. The
  existing `event_delay` already tells the Teensy when the event happens;
  the natural mapping is "use all the time you have, minus the gap". This
  preserves the offline optimiser's control over `T` (via `event_delay`)
  without a new field.
* Add two new payload fields (uint16 each, scaled mm): `stroke_lo_mm`,
  `stroke_hi_mm`. These cannot be derived on-Teensy because their values
  come from the offline trajectory optimiser's pose plan.
* If the new fields don't fit the 8-byte CAN frame, switch this command to
  CAN-FD (the Teensy 4.0 supports it) or use a second frame.

The detailed wire-format decision is properly Phase 3's — but the prototype
is built around exactly these four inputs, so any Phase-3 wire format that
delivers them works.

## Discussion

### Why quintic over 7-segment S-curve

The 7-seg S-curve reaches v_max = 7 m/s at the chosen 6000 rev/s² bound
(it sustains the peak accel; no `5.77×` or `1.5×` overhead). The quintic
costs ~10 % of top-end velocity capability at the same bound. The user
chose the quintic anyway because:

* The within-ramp jerk is *continuous* (linear in τ), not stepped — the
  motor sees a smooth jerk ramp through every quintic, with discontinuities
  only at the cruise joins (one per join, predictable).
* Phase 4 hardware capability will likely allow nudging the accel bound up
  if v=7 capability becomes necessary; the prototype reads the bound from
  config so no code change is needed for that tuning.
* The juggling demo (`bb-led-two-ball-juggle-demo.md`) operates well below
  v=7 anyway — the lost capability is at the top of an envelope the demo
  does not approach.

### Why I should have raised the v_max tradeoff in the first question round

The original Phase 2 question set framed the family choice on "smoothness
of jerk" alone and labelled the 6000 rev/s² option "match current
capability — preserves today's max-velocity capability exactly". That
description was true for the piecewise-constant family but false for the
quintic, where the same bound costs ~10 % of v_max. The user's selection
was made on the wrong framing; I re-engaged with the math before writing
code and the user reconfirmed quintic, accepting the cost knowingly. Lesson
recorded for future design questions: when a "preserves capability"
description is family-conditional, name the family AND the resulting v_max.

### Why the offline reference lives in `tools/probes/`

Phase 3's lockstep firmware+port rewrite is the moment the new profile
becomes production. Until then, importing the reference into `sim/hand/`
would either (a) silently replace the existing port (regressing
hardware-faithful sim behaviour against the still-installed legacy
firmware) or (b) coexist as parallel implementations (drift risk). Keeping
it under `tools/probes/` makes Phase 3 the single, atomic flip — the
reference becomes the production implementation by being ported into both
production sites simultaneously, and `tools/probes/` keeps a snapshot of
the reference as it stood at Phase 3's commit for future audit.

### Tradeoffs accepted

* **v_max drop 7.0 → 6.31 m/s** at the chosen 6000 rev/s² bound. Explicit
  decision; revisited if Phase 4 measurement raises the bound.
* **Symmetric ramps** — does not model the legacy generator's
  `inertia_ratio = 0.747` loaded/unloaded asymmetry. The asymmetric model
  was offered (option D in the second question round) and rejected — the
  symmetric profile is simpler, the modelled-out asymmetry is small in the
  velocity regime the demo uses, and Phase 4 hardware data can drive a
  reintroduction if the symmetric profile underperforms.
* **Jerk discontinuity at cruise joins** — single step per join, magnitude
  equals the in-ramp peak. Bounded; not eliminated. A C3 family (septic at
  ramps with jerk-BCs of 0 at the cruise joins) would close this — left as
  a possible Phase 5 if Phase 4 measurement suggests it.

## Open Questions

* **Hardware peak-accel tuning (Phase 4).** Is `6000 rev/s²` the right
  operating point, or does Phase 4 bench data justify raising to ~7500
  (recovering v_max = 7) or lowering for margin?
* **CAN payload finalisation (Phase 3).** The recommendation above (extend
  `0x6D0` with two stroke fields, derive `T` from `event_delay`) needs
  Phase 3 wire-format confirmation — including the CAN-FD vs second-frame
  decision if the new fields overflow 8 bytes.

## Related

* Phase 1 (characterisation): [2026-05-22-hand-generator-phase1-characterisation.md](2026-05-22-hand-generator-phase1-characterisation.md)
* `catch_vel_ratio` reconciliation: [2026-05-23-catch-vel-ratio-port-reconciled.md](2026-05-23-catch-vel-ratio-port-reconciled.md)
* Plan: `plans/active/hand-trajectory-generator-overhaul.md` (Phase 2 →
  COMPLETE; Phase 3 cleared to start)
* Prototype: `tools/probes/hand_jerk_limited_prototype.py`
* Tests: `tests/sim/test_hand_jerk_limited_prototype.py`
