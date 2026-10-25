---
title: R6 close-out - MPC-era residue deleted, docs/motion_planner rewritten, two-ball-skill-stack plan archived completed
type: refactor
date: 2026-10-13
status: resolved
phase: "two-ball-skill-stack — R6 (close-out)"
related_plan: two-ball-skill-stack.md
related:
  - 2026-10-13-skill-stack-r5-sitting-8-gate-met.md
  - 2026-09-01-mpc-chain-removed.md
  - 2026-09-09-skill-stack-r0-dead-layer-deletion.md
files_changed:
  - controller/target.py
  - controller/__init__.py
  - controller/README.md
  - controller/REFERENCE_LAYER_CONTRACT.md
  - sim/gate_common.py
  - sim/skills_gate.py
  - docs/index.md
  - docs/motion_planner/
  - docs/analysis/
  - mkdocs.yml
  - CLAUDE.md
  - plans/archived/two-ball-skill-stack.md
  - plans/active/INDEX.md
  - plans/archived/INDEX.md
  - plans/active/skill-stack-dress-rehearsals-and-residue.md
  - logbook/INDEX.md
  - tests/ (docstring path re-points, 15 files; tests/hardware/ sitting sheets, 13 files)
  - tools/probes/ (path re-points, 4 files)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/ (path re-points, __init__.py, INVARIANTS.md, executor.py comment)
  - ros_ws/src/jugglebot/jugglebot/ball_possession.py (comment path re-point)
  - config/hardware_config.yaml (comment path re-point; generated artifacts unchanged)
  - plans/archived/cup-contact-contract.md (path re-point)
  - docs/can_bridge/safety.md (link to the deleted page re-pointed)
subsystem:
  - skill-stack
  - docs
tags:
  - close-out
  - archival
---

# R6 close-out

## What
- **Deletions.** `controller/target.py` 953 -> ~475 lines: `TargetSource` protocol, `AutoSequence`/`Static`/`WaypointTargetSource`, `TargetCommand`, `sample_ref_fn`, `compute_stretch_ms`, `is_warm_start_invalidating` and unused imports. `sim/gate_common.py` lost `travel_mm` and `tilt_change_deg`; `sim/skills_gate.py` lost the `--throw-noise-frac` flag. Pre-counts were non-zero; post-deletion code references are zero (prose-only hits remain in deletion notes, history, and `controller/REFERENCE_LAYER_CONTRACT.md`). Kept: `ReferenceEvent`, `make_feasible_events`, `flat_target_to_events`, `pose_6dof_from_state` (now has no importer - a candidate for the next sweep).
- **Docs.** `docs/index.md` and `docs/motion_planner/{index,architecture,control_loop}.md` rewritten; `trajectory`, `safety`, `operations`, `integration` pages deleted; `kinematics`, `dynamics`, `workspace`, `results` and `docs/analysis/{index,diagnosis,logbook}.md` carry a "Status (2026-10-13)" historical banner; `mkdocs.yml` nav pruned (Motor Guard parked, Validation Results historical).
- **Archival.** Plan `git mv`'d to `plans/archived/` (name unchanged), frontmatter `status: completed`, `archived: 2026-10-14`, `completed: 2026-10-13`; Archival note and `residual_xfails:` added; both INDEXes updated; every `plans/active/two-ball-skill-stack.md` path reference outside `logbook/` re-pointed to `plans/archived/` (the audit extended the first sweep of 23 to the dated sitting sheets under `tests/hardware/`, `config/hardware_config.yaml`'s comment — regenerated, no generated artifact changed — and the two dated code comments; `logbook/` entries are history and keep the old path; the Platform firmware comment `Teensy_code_platform.ino:143` waits for the next firmware build). No box-gated file was edited.

## Why
R5's gate was met at sitting 8, which is R6's trigger by the plan's own definition. The census found R5 itself left nothing dead, so the deletion is the MPC-era residue R0 missed (a `controller/target.py` that the 2026-09-09 sweep kept as live), not R5 debris. Residue and rehearsals are re-homed in the active dress-rehearsals plan, not left in the archived one.

## Verification
- Deletion unit (2026-10-13): `pytest -q -p no:cacheprovider tests/sim/test_make_feasible_events.py tests/sim/test_multi_event.py tests/sim/test_skills_gate.py tests/motion/test_trajectory_quintic.py` -> 78 passed in 125.09 s.
- Docs units (2026-10-13): relative-link check of the rewritten pages, no missing targets; mkdocs strict build not run (not installed here).
- Scoped (2026-10-14): `pytest -q -p no:cacheprovider tests/sim/test_plans_index.py tests/sim/test_logbook_front_matter.py tests/sim/test_logbook_search.py` (run 2026-10-14) -> 112 passed in 0.84 s
- Full gate: `./run_tests.sh --full`, run 2026-10-14 00:18-00:24 (`temp/logs/gate_r6_final_20261014.log`): **PASS — 6992 passed, 9 skipped, 1 xfailed in 354.37 s (parallel), serial 7 passed in 27.46 s**

## Open
Residue plan: `plans/active/skill-stack-dress-rehearsals-and-residue.md`. `controller/REFERENCE_LAYER_CONTRACT.md` was re-framed the same night (K1-K6 unchanged; the MPC-era producer, stretch, W5 and diagnosis sections marked historical, code at tag `mpc-final`); the audit then found that `make_feasible_events` itself has no live caller outside its tests (the sim throw loop imports only `ReferenceEvent`) and `pose_6dof_from_state` none at all — recorded in the contract and the residue plan as an owner decision, not deleted.
