---
title: Hand trajectory generator overhaul — Phase 1 empirical characterisation of the current generator
type: feature
date: 2026-05-22
status: resolved
phase: "hand-trajectory-generator-overhaul — Phase 1"
related_plan: "hand-trajectory-generator-overhaul.md"
related_entries:
  - 2026-04-23-hot-loop-zero-allocation-contract
  - 2026-05-20-walk-forward-singleton-emission-jerk
files_changed:
  - tools/probes/hand_profile_probe.py
  - tools/probes/README.md
  - tests/sim/test_hand_profile_probe.py
  - logbook/2026-05-22-hand-generator-phase1-characterisation.md
  - logbook/INDEX.md
  - plans/active/hand-trajectory-generator-overhaul.md
commits:
  - <pending>
subsystem:
  - hand
  - teensy
tags:
  - characterisation
  - trajectory
  - probe
  - testing
---

# Phase 1 — Characterise the current hand throw/catch generator

## Summary

Phase 1 of the hand-trajectory-generator overhaul builds a reusable
characterisation probe — `tools/probes/hand_profile_probe.py` — that drives
the *real* current generator (`sim/hand/trajectory.py`, the Python port of
Teensy `Trajectory.h`) across the `0.3–7.0 m/s` event-velocity sweep and
quantifies the two limitations the overhaul targets:

1. **Unbounded jerk.** `calcThrow`/`calcCatch` build three-segment profiles
   with piecewise-CONSTANT acceleration. Acceleration *steps* discontinuously
   at every segment boundary — measured at up to **191 m/s² per step** for a
   `7 m/s` throw — so jerk is a Dirac spike there.
2. **Backwards velocity scaling.** Peak acceleration is an exact `v²` power
   law (`3.908·v²` m/s² for the throw) and motion duration an exact `1/v`
   law (`0.614/v` s). Peak acceleration is hardware-bounded, so this scaling
   is inverted — it cannot deliver fast low throws *and* slow high throws.

The probe also resolved the long-standing **`catch_vel_ratio` divergence**:
the firmware runs **0.6** (authoritative); the Python port hardcodes **0.9**.

## Motivation

The overhaul (`plans/active/hand-trajectory-generator-overhaul.md`) replaces
the throw/catch builders with a jerk-limited, time-budget-parameterised
profile family. Phase 2's profile design and Phase 4's hardware jerk-
reduction claim both need a quantified **baseline** of the current generator:
exact acceleration-step magnitudes, the realised peak acceleration as a
function of `v`, and the duration scaling law. Phase 1 produces that baseline
as a committed, reproducible probe so later phases can re-run it for an
apples-to-apples before/after comparison.

## Design

Design decisions were surfaced to the user before any code was written:

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Velocity sweep | metrics at `0.3–7.0 m/s` step `0.1` (68 pts); time-series plots at `{0.5, 1.0, 3.0, 5.0, 7.0}` m/s | dense enough to fit the `v²`/`1/v` laws cleanly without 68 unreadable plots |
| Jerk computation | analytic accel-step **and** finite-difference at 500 Hz | the analytic step is the sample-rate-independent ground truth; the FD spike shows it is sample-rate-dependent, i.e. unbounded |
| Output format | PNG plots + per-velocity CSV + summary JSON | JSON is what the unit tests assert against |
| `makeSmoothMove` | characterised as the jerk-bounded reference point | gives Phase 2 a concrete jerk/accel target to design against |

The probe drives the **real port classes** (`HandThrowTrajectory`,
`HandCatchTrajectory`, `HandSmoothMove`) — `sample()` is the production code
path. Velocity/acceleration/jerk are forward-finite-differenced from the
sampled positions; the per-segment analytic accelerations are read from the
constructed objects. A unit test (`test_probe_drives_the_real_generator`)
asserts the probe's class identities are the port's, so the probe can never
silently drift into a re-implementation.

## Implementation

- **`tools/probes/hand_profile_probe.py`** (new, committed per the
  `tools/probes/` convention). Self-contained `sys.path` setup; outputs to
  `temp/probes/` (gitignored). Pure characterisation functions
  (`characterise_throw`, `characterise_catch`, `characterise_smooth_move`,
  `run_probe`, `fit_power_law`) are importable by tests with no file IO.
- **`tools/probes/README.md`** — added a third probe family,
  "Characterisation harnesses", and the `hand_profile_probe.py` row.
- **`tests/sim/test_hand_profile_probe.py`** (new, 12 tests) — asserts the
  headline findings: acceleration discontinuity at every boundary, the `v²`
  and `1/v` scaling laws (exact-fit residual `< 1e-12`), the smooth-move
  bounded reference, determinism, and the `catch_vel_ratio` divergence.

No production code was modified — Phase 1 is characterisation-only.
`Trajectory.h`, `sim/hand/trajectory.py` and `hardware_config.yaml` are read
but not touched (their edits belong to Phases 2–3).

## Verification

**Inaugural baseline** (this is the plan's first phase — no prior baseline):

- Start SHA: `7d16e2b2823dac6d24dc3efdcad890be7cbd5c9a`
  (`feat(demo): juggling-demo trajectory player core — Phase 2 (partial)`).
- Baseline suite: `pytest tests/ -q`, run 2026-05-22 → **1443 passed,
  1 xfailed** in 417.00 s (ci-fast profile).

**After Phase 1:**

- Scoped verification of this phase's change: `pytest
  tests/sim/test_hand_profile_probe.py -q`, run 2026-05-22 → **12 passed**
  in 1.32 s. Phase 1 adds exactly these 12 test items.
- Full suite: `pytest tests/ -q`, run 2026-05-22 → **1461 passed, 1 xfailed**
  in 436.71 s — zero failures, zero regressions. *Caveat:* the shared
  working tree also carried an actively-developing parallel session's
  Phase 2 work for `bb-led-two-ball-juggle-demo` (`sim/ball/`,
  `sim/plant/mujoco_plant.py`, `sim/model/jugglebot.xml`,
  `tests/sim/test_multiball.py`). The full-suite delta over baseline
  (+18 = 1461 − 1443) is therefore *not* solely Phase 1; Phase 1's
  contribution is exactly the +12 `test_hand_profile_probe.py` items
  (confirmed: collection drops to 1450 with that file removed, so the
  remaining +6 are the parallel session's). Only Phase-1-owned files were
  staged for this phase's commit.
- Hot-loop allocation contract: `pytest
  tests/sim/test_hot_loop_allocation_contract.py -q`, run 2026-05-22 →
  **3 passed** in 16.67 s (stayed green throughout).
- Probe determinism: CSV, JSON and all three PNGs are **bit-identical** on
  re-run (verified by `md5sum` across two runs).
- Mutation check: fitting the `v²` data with exponent 3 (or the `1/v` data
  with exponent −2) raises the max relative residual from `~1e-16` to
  `~0.95` — well past the test threshold of `1e-12` — confirming the
  scaling-law tests fail if the asserted law is wrong.

### Characterisation results

**Throw** (`HandThrowTrajectory`):

| Quantity | Result | Fit residual |
|----------|--------|--------------|
| Peak acceleration | `3.908 · v²` m/s² | `2.9e-16` |
| Motion duration | `0.614 / v` s | `1.8e-16` |
| Accel step @ `v=0.3` | `0.26` / `0.35` m/s² | — |
| Accel step @ `v=7.0` | `143.0` / `191.5` m/s² | — |
| Peak accel @ `v=7.0` | `191.5` m/s² = `6054` rev/s² = **60.5× the smooth-move limit** (100 rev/s²) | — |
| FD jerk @ `v=7.0` (500 Hz) | `78 845` m/s³ | — |

**Catch** (`HandCatchTrajectory`, **port `catch_vel_ratio = 0.9`**):

| Quantity | Result (port 0.9) | On hardware (firmware 0.6) |
|----------|-------------------|----------------------------|
| Peak acceleration | `3.341 · v²` m/s² | `1.485 · v²` m/s² (`× (0.6/0.9)²`) |
| Motion duration | `0.665 / v` s | `0.998 / v` s (`× 0.9/0.6`) |
| Peak accel @ `v=7.0` | `163.7` m/s² = `5176` rev/s² | `72.8` m/s² = `2300` rev/s² |
| FD jerk @ `v=7.0` (500 Hz) | `55 923` m/s³ | `22 509` m/s³ (measured at ratio 0.6 — see note) |
| Total duration | `motion_duration + 0.10 s` fixed `END_PROFILE_HOLD` tail | same |

> **Note on the firmware-ratio column.** The *analytic* quantities scale
> cleanly: peak acceleration by `ratio²`, motion duration by `1/ratio`.
> Finite-difference jerk does **not** — it is sample-rate-dependent (see
> Discussion), and at ratio 0.6 the motion duration changes so the 500 Hz
> grid lands on different profile points. The `22 509 m/s³` figure was
> therefore *measured directly* by re-running the probe with
> `CATCH_VEL_RATIO = 0.6`, not scaled from the port value.

**SmoothMove reference** (`HandSmoothMove`, full effective stroke): peak
acceleration `100.0` rev/s² (exactly its design limit), FD jerk `42.3` m/s³
— **three orders of magnitude below the throw**. This is the jerk-bounded
target Phase 2's profile family must meet.

## Discussion

### Why a committed probe over a one-off script

The probe is a reusable characterisation harness, not a one-off: Phase 4
will re-run it (or a sibling) to measure the *new* generator's jerk against
this exact baseline, and the unit tests import its pure functions directly.
Per CLAUDE.md's reusable-probe rule and `tools/probes/README.md`, that means
committed to `tools/probes/` with a header docstring and a README entry,
outputs to gitignored `temp/probes/`. A `/tmp/probe_*.py` would have rotted
the moment Phase 4 needed it.

### Why analytic accel-steps *and* finite-difference jerk

The current generator's acceleration is piecewise-constant, so its jerk is
mathematically a sum of Dirac deltas — there is no finite "jerk value" to
measure. Two complementary quantities were therefore recorded:

- The **analytic acceleration-step magnitude** at each boundary — exact,
  sample-rate-independent, and the honest headline number.
- The **finite-difference jerk on the 500 Hz grid** — what a real 500 Hz
  controller actually sees. Its magnitude (`78 845` m/s³ for the throw)
  *depends on the sample rate* (it would double at 1 kHz); that
  rate-dependence is itself the proof that the underlying jerk is unbounded.

Reporting only the FD jerk would have invited the false reading that jerk is
a finite ~80 000 m/s³ quantity. Reporting only the analytic step would have
lost the controller-relevant figure. Both, with the relationship explained,
is the rigorous choice.

### Why the fits use motion duration, not total duration

The throw timeline is pure accel→vel-hold→decel, so its total duration is an
exact `1/v` law. The catch timeline appends a fixed `END_PROFILE_HOLD = 0.10 s`
tail, so its *total* duration is `1/v + const` and does **not** fit a pure
power law (residual `~0.45`). The probe fits `motion_duration_s`
(accel+vel+decel only) for both, which is pure `1/v` for each, and records
the constant tail separately. `test_total_catch_duration_is_not_pure_inverse_v`
documents this distinction so a future reader does not "fix" the fit by
forcing the wrong quantity through it.

### The `catch_vel_ratio` divergence — resolved

`config/hardware_config.yaml` → `config/generated/hardware_config.h` sets
`CATCH_VEL_RATIO = 0.6f`. `Trajectory.h` reads `TeensyTraj::CATCH_VEL_RATIO`,
so **the Teensy firmware compiles and runs 0.6**. The Python port
(`sim/hand/trajectory.py:36`) hardcodes `CATCH_VEL_RATIO = 0.9`, introduced
2026-03-21 in commit `6859a9c` ("Ineedtopushmoreoften...") — roughly three
weeks *after* the YAML already held `0.6` (commit `b8795cd`, 2026-03-01).

The port exists to mirror the firmware. The firmware is the source of truth.
**Therefore 0.6 is the authoritative value and the port at 0.9 is the
divergent, incorrect copy.** This is precisely the silent port/firmware
divergence the overhaul plan's lockstep rule guards against — surfaced here
because the port hardcodes constants instead of reading the generated config
module.

Phase 1 does **not** fix it: editing the port is out of characterisation
scope. It is filed below for Phase 3 (the lockstep firmware+port rewrite),
which must reconcile the port to 0.6 — *unless* 0.9 was a deliberate design
intent, in which case the YAML changes to 0.9 and the Teensy is re-flashed.
That is a decision for the user / Phase 3 author, not Phase 1.

**Cross-effect on the BB-led two-ball juggle demo:** the concurrent demo
session used `0.6` in its Phase 1 feasibility arithmetic. Since `0.6` *is*
the true hardware value, **the demo's feasibility table is correct and needs
no revisiting** — the divergence does not propagate into that effort.

### Tradeoffs accepted

- The probe reads private attributes (`_throwA`, `_t_acc`, …) of the port
  objects for the analytic figures. This couples the probe to the port's
  internal layout, but those attributes are stable (the port is the
  measurement subject and Phase 3 rewrites it in lockstep anyway), and it is
  far cheaper than re-deriving the segment math — which would risk exactly
  the re-implementation drift Phase 1 must avoid.
- Forward (not central) finite differencing was chosen for simplicity and
  determinism; it loses one sample per derivative. Peak-magnitude detection
  is unaffected, and the analytic steps are the authoritative figures
  regardless.

## Open Questions

- **`catch_vel_ratio` reconciliation (Phase 3).** The port must be brought
  to the firmware value `0.6` in the lockstep rewrite — or, if `0.9` was a
  deliberate catch-impact-reduction choice, the YAML moves to `0.9` and the
  Teensy is re-flashed. Needs a user/Phase-3 decision on design intent.
- **Profile family (Phase 2).** The `v²`/`1/v` baseline is now quantified;
  Phase 2 chooses 7-segment S-curve vs piecewise-quintic against it.

## Related

- Plan: `plans/active/hand-trajectory-generator-overhaul.md` (Phase 1)
- Probe: `tools/probes/hand_profile_probe.py`; outputs in `temp/probes/`
- Tests: `tests/sim/test_hand_profile_probe.py`
- `plans/active/bb-led-two-ball-juggle-demo.md` §6 — also tracks the
  `catch_vel_ratio` open item (resolved here as 0.6).
