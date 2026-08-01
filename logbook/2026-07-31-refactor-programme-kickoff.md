---
title: Refactor programme kickoff — 13-agent review verdict, owner decisions, and the Phase 0 dead-weight sweep
type: refactor
date: 2026-07-31
status: resolved
phase: "Refactor programme Phase 0 — review + dead-weight sweep"
files_changed:
  - plans/active/refactor-2026-07.md
  - attic/README.md
  - attic/ros-jugglebot-archived/
  - docs/sim_mpc/index.md
  - tools/probes/README.md
  - ros_ws/src/jugglebot/jugglebot/bb_calibration.py
  - ros_ws/src/jugglebot/jugglebot/tracking/kalman.py
  - .gitignore
subsystem:
  - repo
---

# Refactor programme kickoff + Phase 0 sweep

**What**: a 13-agent codebase review (6 subsystem mappers, 3 pain-point
analysts, synthesis, 3-lens red team; all findings evidence-cited and
spot-verified) assessed whether refactoring is warranted. Verdict: the
load-bearing architecture is healthy; the drag is package boundaries, dead
weight, god-files, and process ceremony. The full programme — owner
decisions of 2026-07-31 included (SHA-backfill retirement, short-form
logbook default, 4am nightly runner, teensy_link → repo root, MPC
operational removal with parked code, build-frozen config + drift-warning
contract) — lives in `plans/active/refactor-2026-07.md`. CAN3 verdict
stress-tested and upheld as hardware (~85–90%); the legitimate software
residue is scheduled as plan Phase 7.

**Phase 0 sweep landed here**: `jugglebot/archived/` (30 files, 16,311
lines — verified excluded from `setup.py` packages and import-free) moved
to `attic/ros-jugglebot-archived/`; deleted 8 unreferenced
`sim/viz/reference_*.png` (~3.5 MB), `sim/sweep_speed_ratio.py`+result
PNGs (tree listing in `docs/sim_mpc/index.md` updated),
`sim/tools/motor_command_verification.png`, empty `sim/tests/` (its stale
`.gitignore` rule dropped too; `MUJOCO_LOG.TXT` turned out to be already
ignored); removed untracked `sim/controller/`+`tests/archived/` pycache
remnants and the dead untracked `ros_ws/src/yasmin` (5.7 MB, zero
references); the three "lifted/ported from archived/" docstrings
(`bb_calibration.py`, `tracking/kalman.py`, `sim/ball_butler/sim.py`) and
the `archived/` path references in `ros_ws/docs/levelling_frame.md` +
`docs/motion_planner/trajectory.md` re-pointed at the attic;
`tools/probes/README.md` gained the missing
`canbridge_reboot_latch_probe.py` row.

**Why attic instead of delete**: logbook entries reference these paths and
the code carries archaeology active subsystems still consult; the attic
keeps references live while removing ~32% of the ROS package's Python
lines (16,311 of 50,557 by raw `wc -l`, run 2026-07-31) from every grep,
index, and session context load (policy note in `attic/README.md`).

**Also landed**: a `flock` suite mutex in `run_tests.sh` — two live
gate-vs-gate collisions with a parallel session on 2026-08-01 (the race
window between "is a suite running?" and launch) motivated queuing full
gates on `/tmp/jugglebot-run_tests.lock`; scoped passthrough runs stay
lock-free.

**Verification**: colcon install provably unaffected (`setup.py:22-28`
packages list never included `archived`); repo-wide import grep for every
removed/moved artifact = zero live references (run 2026-07-31, this
session). Full gate (run through the new mutex): `./run_tests.sh`, run
2026-08-01: **parallel 4292 passed + 3 xfailed in 434.35 s; serial 9
passed in 39.08 s; total 478 s — RESULT: PASS**.
