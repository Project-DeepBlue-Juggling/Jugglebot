---
title: Plans-board cleanup — 5 archived, 6 parked, 2 prompts deleted, 4 probes archived
type: refactor
date: 2026-08-15
status: resolved
phase: "plans-board hygiene"
subsystem:
  - docs
tags:
  - planning
  - cleanup
---

# Plans-board cleanup — 5 archived, 6 parked, 2 prompts deleted, 4 probes archived

**What/why.** After the 2026-07→08 development burst the active-plans board held
21 files, most no longer driving work. Owner-ratified triage (2026-08-14, full
survey of all 21 by content, not mtime) restores an honest board of 10 rows
(8 truly active + 1 reference + 1 draft):

- **Archived** (to `plans/archived/`, each with an Archival note):
  *fk-convergence-tolerance* (both phases DONE; FK-1/FK-2 PASS 2026-07-27;
  FK-3/FK-4 structurally unscoreable), *hand-ball-sensor* (Phases 0–6 shipped
  and in production as the possession source; Phase 7 steps 4–5 are bench
  tuning re-homed to `tests/hardware/session_hand_ball_sensor.md`),
  *tilt-calibration-grid* (map v2026-08-10-3bf7964f committed + validated;
  leg-0 SPINOUT already homed in the 2026-08-10 C0-blockers entry + the
  2026-08-12 fleet-reflash tool), *teensy-can-offload* (legs cut over,
  socketcan decommissioned; residual rows live in
  `ros_ws/docs/can-node-teensy-parity.md`; U5 tail gated on dormant MPC),
  *bb-led-two-ball-juggle-demo* (superseded by
  `bb-online-juggle-tilt-rearchitecture` Rung 3; its Phase 4 targeted the
  deleted `can_node` and the parked `motor_guard`).
- **Parked** (status `parked` in INDEX + a banner; work remains but nothing is
  schedulable): *accel-ff-inertia* (firmware stale-hold decay unwritten;
  premise re-derivation owned by the bridge arc), *learned-ff-residuals*
  (gates G-A/G-B/G-C all open), *catch-reach-degenerate-overshoot* (all phases
  DONE; residual seat-rate A/B blocked by catch-robustness § Constraints),
  *hand-trajectory-generator-overhaul* (12 weeks cold, self-declared
  non-prerequisite), *levelling-frame-contract* (one short sitting + one paper
  decision; int16 truncation defect noted in the banner), *refactor-2026-07*
  (ERR_TIMEOUT closed; remainder is owner decisions + unscheduled refactors;
  § Standing coordination rules remains live process text).
- **Deleted** (owner convention: completed prompts are deleted):
  `PROMPT-anomaly-fixes-orchestration.md` (all 11 rows DONE 2026-07-27; Item
  D/E narratives verified mirrored in `2026-07-29-hand-post-release-decel.md`
  and single-ball-toss § Phase E) and
  `PROMPT-single-ball-toss-software-run.md` (all Done-means satisfied
  2026-07-25). Untracked one-shot phase runners `canbridge-phase-runner.js`
  (dangling pointer to an archived plan), `sitting-analysis.js` (hardcoded to
  the 2026-07-27 sitting) and `anomaly-phase-runner.js` (bound to the deleted
  prompt) removed; `mvp-phase-runner.js` committed (live plan, same shape as
  the tracked `bb-tilt-phases.js`).
- **Probes archived** (to `tools/probes/archived/`, README ledger rows added):
  `canbridge_relay_probe.py`, `canbridge_version_probe.py`,
  `canbridge_reboot_latch_probe.py` (cold-start parity arc, complete),
  `canhub_500hz_deadline_gate.py` (Tier-2 validation arc, complete) — all
  verified zero live references before the move.

**Verification.** See the (date, command, result) triple in the commit-time
gate run recorded below; `tests/sim/test_plans_index.py` pins the INDEX ↔
`plans/active/` correspondence and passes with the new board.

- 2026-08-15, `./run_tests.sh --full`: **RESULT: PASS** (parallel phase 497 s
  rc=0; serial phase 9/9 in 41.61 s; total 542 s) — run on the final combined
  tree immediately pre-commit; `--full` chosen because the change touches a file
  under `sim/`. Independent corroboration: the 04:04 nightly
  (`./run_tests.sh --full --hypothesis-profile=ci-deep`, 2026-08-15) was GREEN
  5574/5577 (3 xfailed) on the same tree before the audit fixes.
- 2026-08-15, `/audit --unstaged`: ISSUES FOUND (5 WARNING / 6 NOTE, none
  blocking; the control-path change audited clean) — all 12 findings fixed
  pre-commit: the dissolved chaining limitation reframed in the operative
  runbook and the `toss_session`/`reload_coordinator_node` docstrings, a full
  batch repoint of every living reference to the five archived plans and two
  deleted prompts (grep-to-zero outside `logbook/` + `plans/archived/`), the
  hand-sensor step-numbering and widening-date and line-ref corrections, the
  new-msg-fields note in deployment row B, the R6 tier-8a capture interaction
  documented in catch-robustness, and the field-less-publisher test arm made
  to exercise real attribute absence.
