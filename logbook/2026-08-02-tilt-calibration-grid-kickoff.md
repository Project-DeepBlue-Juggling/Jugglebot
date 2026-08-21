---
title: "Tilt-calibration-grid kickoff: architecture scan + plan created (pose-dependent level reference)"
type: feature
date: 2026-08-02
status: resolved
phase: "Tilt calibration grid — kickoff"
related_plan: tilt-calibration-grid.md
subsystem: [motion]
tags: [docs, levelling, calibration]
files_changed:
  - plans/archived/tilt-calibration-grid.md
  - plans/active/INDEX.md
---

# Tilt-calibration-grid kickoff: architecture scan + plan created

## Summary

Owner report: on every sitting where Jugglebot threw vertically to a
non-(0, 0, 170) pose, many throws clipped platform hardware before landing in
the hand — the ball leaves slightly "backwards" (+rx). A 9-agent architecture
scan (2026-08-02) mapped the C-LEVEL-1 levelling chain end to end and verified
the design seam. New plan `plans/archived/tilt-calibration-grid.md`: a 2-D (x, y)
residual tilt map layered on the single-offset `level` routine — captured by
driving a `go_to_pose` grid and reading the SCL3300 through the ungated
`get_platform_tilt` service, persisted as committed machine-written
`config/tilt_calibration.yaml`, auto-applied via a reload service, evaluated
once per external-pose ingest keyed on the uncorrected intent pose, clamped to
the calibrated hull, non-gating. No firmware change anywhere in the plan.
Validation includes a deliberately tilted base (rung C2) to prove base tilt is
absorbed as common-mode by `level` while the map stays invariant.

## Discussion

Trigger (a) — the motivating premise was reframed mid-investigation. The
motivating sittings are unlogged: no repo artifact records a phase-8 sitting
after 2026-07-27, and that sitting's recorded rim bounce-outs were attributed
to BB warm-up arrival drift, not platform tilt. The scan surfaced the actual
recorded seed evidence instead: the extremity-tilt table in
`logbook/2026-07-28-anomaly-fixes-validation-sitting.md` (0.041° at (60, 0) →
0.604° at (150, −150), non-linear in (x, y), revisit repeatability 15–40×
smaller than the effect) — which itself nominated an inclinometer sweep as the
discriminator. Owner decision (2026-08-02): build the tool canonically rather
than reconstruct the unlogged sittings; rung C3 (displaced-toss A/B, map
off/on) captures the symptom quantitatively instead.

Design verdicts with non-obvious rationale live in the plan's Architecture
section: per-target evaluation at ingest (a per-knot/emitter lookup is
disqualified three ways: knot/velocity desync, feasibility-gate escape, wire
steps below every guard threshold); layered-not-replacing (the map must be
invariant under base tilt for the C2 validation to mean anything); loader
resolution order source-tree-before-ament-share (deliberate inversion of the
friction_ff order — the calibration tool rewrites the source-tree file at
runtime, and an ament-copy-first order silently serves stale calibration until
the next colcon build).

## Verification

Docs-only commit (plan + INDEX + this entry; no *.py/*.yaml touched). The
tests that read the changed paths: tests/sim/test_plans_index.py (plan ↔ INDEX
bidirectional pin) and tests/sim/test_logbook_front_matter.py (this entry's
frontmatter) — run scoped before commit; the (date, command, result) triple is
cited in the commit message.
