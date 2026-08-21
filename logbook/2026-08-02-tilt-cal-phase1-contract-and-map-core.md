---
title: Tilt calibration Phase 1 — C-LEVEL-2 contract amendment + the pure map core (parse/validate/bilinear + the single application entry point)
type: feature
date: 2026-08-02
status: resolved
phase: "tilt-calibration-grid Phase 1"
related_plan: tilt-calibration-grid.md
files_changed:
  - ros_ws/docs/levelling_frame.md
  - ros_ws/src/jugglebot/jugglebot/motion/tilt_map.py
  - ros_ws/src/jugglebot/jugglebot/motion/levelling.py
  - tests/motion/test_tilt_map.py
  - tests/hardware/session_phase8_toss_hardware.md
  - tests/hardware/session_anomaly_fixes.md
  - plans/archived/tilt-calibration-grid.md
subsystem:
  - motion
---

# Tilt calibration Phase 1 — contract + map core

**Why**: `level` measures the platform's tilt against gravity at *one* pose and
applies that offset everywhere, so every pose-dependent kinematic error is
invisible to it by construction — 0.041° at (60, 0) growing to 0.604° at
(150, −150) on the 07-28 extremity table, which is ~25 mm of landing error at
the 0.6 m toss against a ~30–40 mm cup basin. Phase 1 lands the contract and the
pure core; no node touches it yet (Phase 2).

**Doc first, as the plan requires.** `ros_ws/docs/levelling_frame.md` gains
**C-LEVEL-2**, which composes with and never replaces C-LEVEL-1: residual
semantics (measured at commanded-level orientation against a *fresh* level
reference; home node ≈ 0 by construction), additive rotation-vector composition
through the existing single Rodrigues with its second-order term stated **as a
regime table, not a bare bound** (see below), keying on the **uncorrected
intent** pose, evaluation **at ingest per target
only** (per-knot lookup forbidden, with its three independent disqualifiers —
Hermite u0/u1/u2-vs-declared-velocity desync, escaping `feasibility.validate`,
and stepping the wire below both the 0.3 rev pump gate and the 1.0 rev
`MAX_DEVIATION_REV` so correct and buggy look identical), clamp-to-hull,
all-or-nothing load validation, the `tilt_map_loaded` / `tilt_map_version`
observability fields, explicitly **non-gating** semantics, capture
preconditions, the inherited in-flight rule, the map artifact
(`config/tilt_calibration.yaml`, schema v1) and a B1 revival-obligation row for
the dormant `mpc_bridge_node`.

**Code**: new pure `motion/tilt_map.py` (`TiltMapError`, `TiltMap`,
`parse_tilt_map`, `load_tilt_map`, `lookup`, `map_version`) and one new entry
point `levelling.correction_for_pose(offset, tilt_map, pose)`. With
`tilt_map=None` it is bit-identical to `correction_from_offset` — the non-gating
degradation is a property of the function, not of a caller's `if`. No existing
signature or behaviour changed.

**Also** (docs hygiene, flagged by every scan scout): the stale
`teensy_bridge_node.py:1430` citation now names the real sites, verified at HEAD
— decode `:349-372`, publish `:1747-1755` — in **both** runbooks carrying it,
`tests/hardware/session_phase8_toss_hardware.md:81` and
`tests/hardware/session_anomaly_fixes.md:93`. The plan named only the first; the
audit found the second, and a half-fixed citation is worse than an unfixed one
(two runbooks, two citations, one fact).

**What the pre-commit audit corrected — one withdrawn claim, worth the space.**
The draft contract asserted the additive-composition error is "< 1e-4 rad for
sub-degree tilts" and pinned it with a *single* worked point 7× inside the
bound. It is false at its own boundary. The exact second-order term is
`|level_offset × residual| / 2` (probe `/tmp/probe_tilt_bound.py`, run
2026-08-02, matching the measured matrix difference to 5 s.f.): **7.19e-5 rad**
at the measured envelope (0.78185° offset × 0.604° residual, orthogonal — holds
with 28% margin), **1.523e-4 rad** at 1° × 1°, and **1.25e-3 rad** at what
`MAX_ABS_RESIDUAL_RAD` alone permits. Physical impact today is nil (1.5e-4 rad
is 0.36 mm at 41.9 mm/°); the defect was the false normative claim, and a test
whose name promised a regime it never swept. The contract now carries the regime
table and the closed form, and the test sweeps the azimuth ring at both
magnitudes — the error is a cross product, so a fixed direction measures
whatever the author happened to pick. Three smaller audit fixes landed with it:
`map_version` hashed `grid.z_mm`/`orientation` (provenance) and so churned on a
re-emit, contradicting its own docstring — it now hashes the axes + residuals,
float-normalised; `TiltMap` enforced its invariants only in `parse_tilt_map`, so
a hand-built 1-node axis raised `ZeroDivisionError` instead of `TiltMapError`,
and `parse_tilt_map` froze arrays it did not own — `__post_init__` now copies,
freezes and validates; and the same stale `:1430` citation survived in
`tests/hardware/session_anomaly_fixes.md:93`, fixed here too so the grep count
reaches zero.

**Verification** — `pytest tests/motion/test_tilt_map.py tests/motion/test_levelling.py -q`
(run 2026-08-02): **80 passed in 2.35 s**; `./run_tests.sh --full` (run
2026-08-02, post-audit-fix tree): **parallel 4501 passed + 3 xfailed in
451.44 s, serial 9 passed in 40.00 s, total 497 s — RESULT: PASS**.
