---
title: catch_vel_ratio — Python port reconciled to firmware-authoritative 0.6
type: bugfix
date: 2026-05-23
status: resolved
phase: "hand-trajectory-generator-overhaul — Phase 1 follow-up"
related_plan: "hand-trajectory-generator-overhaul.md"
related_entries:
  - 2026-05-22-hand-generator-phase1-characterisation
files_changed:
  - sim/hand/trajectory.py
  - tools/probes/hand_profile_probe.py
  - tests/sim/test_hand_profile_probe.py
  - docs/sim_mpc/hand_and_ball.md
  - controller/demo/pattern.py
  - plans/active/bb-led-two-ball-juggle-demo.md
  - logbook/2026-05-23-catch-vel-ratio-port-reconciled.md
  - logbook/INDEX.md
  - plans/active/hand-trajectory-generator-overhaul.md
commits:
  - 23f086d
  - d2e0962
subsystem:
  - hand
tags:
  - bugfix
  - port-firmware-lockstep
---

# `catch_vel_ratio` — port reconciled to firmware

## Summary

Reconcile the Python port's hardcoded `CATCH_VEL_RATIO = 0.9` to the
firmware-authoritative `0.6` (`config/hardware_config.yaml` →
`config/generated/hardware_config.h`). This closes the open item Phase 1 of
the hand-generator overhaul surfaced
([2026-05-22-hand-generator-phase1-characterisation.md](2026-05-22-hand-generator-phase1-characterisation.md)).

## Discussion

The Teensy firmware reads `CATCH_VEL_RATIO` from the generated header
(`hardware_config.h` is built from `hardware_config.yaml`); the YAML has
held `0.6` since `b8795cd` (2026-03-01). The Python port
(`sim/hand/trajectory.py:36`) hardcoded `0.9`, introduced 2026-03-21 in
commit `6859a9c` — three weeks after the YAML was already authoritative.

The port exists to mirror the firmware (its module docstring opens with
"Python port of Teensy Trajectory.h"). The firmware is the source of truth.
**Therefore the port at 0.9 was the divergent, incorrect copy; 0.6 is the
correct value.** The user confirmed 0.6 is intended (no plan to re-flash the
Teensy with a different value).

This is a tiny, atomic fix surfaced by — but independent of — the rest of
the overhaul. It is landed standalone, before the Phase 2 profile-family
design, so the offline reference work in Phase 2 starts from a port that
already agrees with the firmware. Deferring it to the Phase 3 lockstep
rewrite would have meant another phase of probe/test/sim runs producing
catch numbers that did not reflect what hardware actually executes.

### What changes downstream of the fix

- `HandCatchTrajectory` now produces a catch velocity of `-0.6 · event_vel`
  instead of `-0.9 · event_vel` (impact-velocity reduction drops from 90 % to
  60 %, matching hardware). At `v = 7 m/s`: catch hold velocity goes from
  `-6.3 m/s` (port-only) to `-4.2 m/s` (matches firmware).
- Analytic catch peak acceleration scales by `(0.6/0.9)² = 0.444`. Probe now
  reports `1.485 · v²` m/s² (matching the firmware figures already cited in
  the Phase 1 logbook results table); was `3.341 · v²` under the divergent
  port.
- Catch motion duration scales by `0.9/0.6 = 1.5`. Probe now reports
  `0.998 / v` s; was `0.665 / v` s.
- The probe's WARNING line and JSON note are now in their non-divergent
  branch — `divergent: false`, `"Port matches firmware (config/hardware_config.h)."`
- The existing `tests/sim/test_hand_trajectory.py` catch assertions use
  `CATCH_VEL_RATIO` symbolically (importing it from the port), so they adapt
  automatically with no edits required. (A latent confirmation that 0.6 was
  the intended value: line 317's comment reads `vC = -CATCH_VEL_RATIO * 3.0
  # -1.8 m/s`, which is `-0.6 · 3.0`, not `-2.7`. The test author already
  expected 0.6.)

### What stays the same

The fix is value-only. No call-site, signature, or sequencing change. The
overall throw-flight-catch tempo on hardware is unchanged because hardware
already ran 0.6.

### Why this is not yet Phase 3

The lockstep firmware+port rewrite in Phase 3 replaces the *algorithm*
(piecewise-constant-acceleration → jerk-limited family). This change is a
*constant value* correction, orthogonal to algorithm choice. Bundling them
would couple two independently-reviewable changes and obscure the rollback
unit. Per the project's "fix surfaced bugs in the same session when
diagnosis is clear" rule, the fix lands now.

## Verification

- Inaugural-this-session SHA: `5a45c44` (Phase 1 SHA-backfill).
- Hand tests: `pytest tests/sim/test_hand_trajectory.py
  tests/sim/test_hand_profile_probe.py tests/sim/test_hand.py -q`, run
  2026-05-23 → **58 passed** in 2.19 s.
- Full suite: `pytest tests/ -q`, run 2026-05-23 → **1455 passed, 1 xfailed**
  in 445.49 s (no regressions; same 1443-baseline + 12 Phase-1 tests = 1455).
- Probe stdout no longer prints the divergence WARNING (verified manually).
- The probe's catch summary now matches the firmware predictions already
  recorded in the Phase 1 logbook results table (`1.485 · v²` m/s²;
  `0.998 / v` s).

## Related

- Phase 1 logbook: [2026-05-22-hand-generator-phase1-characterisation.md](2026-05-22-hand-generator-phase1-characterisation.md) — where the divergence was characterised.
- Plan: `plans/active/hand-trajectory-generator-overhaul.md` §6 — the
  catch_vel_ratio open item, now closed.

### Cross-effort closeouts (same change set)

`grep`-driven sweep for stale references to the divergence; closed in this
commit alongside the port value change:

- `docs/sim_mpc/hand_and_ball.md:25` — constants table now reads
  `CATCH_VEL_RATIO | 0.6 | Hand velocity = 60% of ball speed` (was `0.9`).
- `controller/demo/pattern.py:30-33` — NOTE block describing the
  divergence and a contingency-recompute clause is replaced with a closeout
  pointer to this entry. `CATCH_DUR_COEF = 0.998` is unchanged (already
  computed for the firmware/0.6 case).
- `plans/active/bb-led-two-ball-juggle-demo.md` §6 — open-item bullet
  rewritten to mirror the CLOSED wording in
  `hand-trajectory-generator-overhaul.md` §6.
