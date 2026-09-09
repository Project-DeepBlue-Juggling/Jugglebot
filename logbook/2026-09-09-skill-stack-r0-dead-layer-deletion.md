---
title: "R0 dead-layer deletion — offline juggle demo, MPC-era sim sources (partial), historical MPC docs; MPC telemetry analysis traced and kept in full"
type: refactor
date: 2026-09-09
status: resolved
related_plan: two-ball-skill-stack.md
files_changed:
  - sim/juggle_demo.py (deleted)
  - sim/juggle_planner/juggle_optimizer.py (deleted)
  - sim/juggle_planner/player.py (deleted)
  - sim/juggle_planner/timeline.py (deleted)
  - sim/juggle_planner/trajectory.py (deleted)
  - sim/juggle_planner/pattern.py (deleted)
  - sim/JUGGLE_DEMO.md (deleted)
  - sim/juggle_planner/__init__.py (rewritten)
  - sim/juggle_planner/juggle_planner.py (docstring cross-ref fix)
  - tests/sim/test_demo_juggle_sim.py (deleted)
  - tests/sim/test_demo_juggle_optimizer.py (deleted)
  - tests/sim/test_demo_timeline.py (deleted)
  - tests/sim/test_demo_sim_playback.py (deleted)
  - tests/sim/test_demo_trajectory.py (deleted)
  - tests/sim/test_sim_import_style.py (removed stale frozenset entry)
  - controller/scheduler.py (deleted)
  - controller/zmq_target.py (deleted)
  - controller/toss_motion_source.py (deleted)
  - controller/catch_optimizer.py (deleted)
  - controller/SCHEDULER_CONTRACT.md (deleted)
  - controller/__init__.py (dropped exports)
  - controller/README.md (table + removed-section note)
  - controller/hermite.py (2 stale docstring cross-refs fixed)
  - controller/PLANT_INTERFACE_CONTRACT.md (3 dead-link/stale-prose fixes)
  - controller/REFERENCE_LAYER_CONTRACT.md (2 dead-reference fixes)
  - controller/telemetry.py (docstring fix)
  - sim/hand/scheduled_coordinator.py (deleted)
  - sim/input/continuous_throw_catch.py (deleted)
  - sim/input/interactive_catch.py (deleted)
  - sim/input/keyboard.py (deleted)
  - sim/input/spacemouse.py (deleted)
  - sim/input/zmq_target.py (deleted)
  - tests/sim/test_scheduler.py (deleted)
  - tests/sim/test_scheduler_contract.py (deleted)
  - tests/sim/test_zmq_corruption.py (deleted)
  - tests/sim/test_zmq_target.py (deleted)
  - tests/sim/test_catch_optimizer.py (deleted)
  - tests/sim/test_toss_geometry.py (TossMotionSource block removed, ballistics tests kept)
  - tests/sim/_zmq_test_harness.py (deleted — orphaned by test_zmq_target.py/test_zmq_corruption.py deletion)
  - tests/sim/conftest.py (comment fix)
  - tests/ros/_bridge_harness.py (comment fix)
  - docs/sim_mpc/ (9 files, deleted)
  - mkdocs.yml (nav block removed)
  - docs/index.md (Simulation MPC section replaced with a historical note)
  - docs/can_bridge/control.md (dead module reference fixed)
  - DOCUMENTATION_GUIDE.md (stale example dir name fixed)
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py (comment fix)
  - sim/ball/manager.py (2 comment fixes)
  - sim/juggle_online.py (3 comment fixes)
  - CLAUDE.md (controller bullet)
  - plans/active/two-ball-skill-stack.md (§ 6 ledger rows updated)
  - logbook/2026-09-09-skill-stack-r0-dead-layer-deletion.md
  - logbook/INDEX.md
subsystem:
  - sim
  - motion
tags:
  - refactor
  - dead-code
---

# R0 dead-layer deletion (skill-stack)

## What / why

First code-deletion unit of the two-ball-skill-stack arc (`plans/active/
two-ball-skill-stack.md` § 6). Four clusters, census-then-delete per § 0's
Rigor value (grep before deleting, count to zero after).

## Deletion ledger

**Cluster A — offline juggle demo.** Deleted in full: `sim/juggle_demo.py`,
`sim/juggle_planner/{juggle_optimizer,player,timeline,trajectory,pattern}.py`,
`sim/JUGGLE_DEMO.md`, and their 5 dedicated test files. Importers before:
each other only (0 outside the cluster). Kept `sim/juggle_planner/
juggle_planner.py` (the CasADi reference `tools/probes/
capture_cup_cycle_refs.py` and the sim rung scripts import) — its `__init__.py`
rewritten to describe the surviving module only.

**Cluster B — MPC-era sim sources (PARTIAL — census missed live importers).**
The plan's own census listed `controller/{scheduler,target,zmq_target,
toss_motion_source,catch_optimizer}.py`, `sim/hand/`, `sim/input/` with only
`controller/__init__.py` and each-other as importers. A fresh grep found three
it missed:

- `sim/toss_gate.py` and `sim/reload_gate.py` (protected `*_gate.py` rung
  scripts) import `sim.hand.trajectory`; `sim/ball_butler/sim.py` imports
  `sim.hand.coordinator` directly. **Kept**: `sim/hand/{ballistics,
  coordinator,planner,trajectory}.py` (planner.py is a transitive dependency
  of the two items below). Deleted: `sim/hand/scheduled_coordinator.py` (the
  only sim/hand module with zero importers outside itself).
- `sim/viz/reference_plot.py` (protected `sim/viz/`, never itself imported or
  tested, but a real standalone debug script) lazily imports `sim.input.
  toss_loop.TossLoopController`. **Kept**: `sim/input/toss_loop.py` and its
  dependency `sim/input/sim_control.py`.
- `tests/sim/test_ball_butler_sim.py` (the live test suite for the protected
  `sim/ball_butler/sim.py`) imports `_ball_landing`/`_compute_catch_target`
  from `sim/input/scripted.py`. **Kept**: `sim/input/scripted.py` whole,
  rather than extracting the two pure-math helpers — extraction is a design
  decision out of scope for a dead-layer deletion rung.
  `toss_loop.py` also imports `controller.target` (`ReferenceEvent`), so
  **kept** `controller/target.py` too (with the same reasoning it survives
  in `controller/__init__.py`'s export list).

Deleted: `controller/{scheduler,zmq_target,toss_motion_source,
catch_optimizer}.py` + `controller/SCHEDULER_CONTRACT.md`; `sim/hand/
scheduled_coordinator.py`; `sim/input/{continuous_throw_catch,
interactive_catch,keyboard,spacemouse,zmq_target}.py`; their 5 dedicated test
files plus `tests/sim/_zmq_test_harness.py` (orphaned — its only consumers,
`test_zmq_target.py`/`test_zmq_corruption.py`, are gone; the two files that
named it in a docstring, `tests/sim/conftest.py` and `tests/ros/
_bridge_harness.py`, got comment fixes, not deletion). `test_toss_geometry.py`
kept its ballistics tests, only the `TossMotionSource` state-machine block
(5 tests) was removed with the module.

**Cluster C — MPC telemetry analysis (traced, kept in full).** `sim/
analysis/diagnose.py` performs live rosbag (MCAP) analysis
(`analyse_rosbag()`) integrated with MPC-CSV analysis — the pilot-E-stop
attribution rule cross-references rosbag timing against CSV state, and this
is the engine behind the `/diagnose` skill used on real hardware sessions.
Not separable into an "MPC-CSV half" without gutting the rosbag path; nothing
deleted here. `controller/telemetry.py` is required by `sim/viz/telemetry.py`
— a re-export shim (`"""Re-export shim — canonical location is
controller.telemetry."""`), and `sim/viz/` is protected. `sim/analysis/
logbook_search.py` is an unrelated documentation-search tool, also kept.
`tools/motion_onset_cogging_study.py` mines `temp/logs/mpc_*.csv` only (no
rosbag path) — MPC-CSV-specific and now dormant, but outside this cluster's
named scope (not `sim/analysis/` or `controller/telemetry.py`), so left
untouched.

**Cluster D — historical MPC docs.** Deleted `docs/sim_mpc/` (9 files) and
its `mkdocs.yml` nav block. `docs/index.md`'s matching section (not itself
named in the brief, but the same dead-link problem) replaced with a short
historical note pointing at `logbook/2026-09-01-mpc-chain-removed.md` and the
`mpc-final` tag.

## Coherence fixes

`controller/__init__.py`, `controller/README.md`, `CLAUDE.md`'s `controller/`
bullet, `mkdocs.yml`, `docs/index.md`, `DOCUMENTATION_GUIDE.md`'s example
dir list, and the two normative contract docs (`PLANT_INTERFACE_CONTRACT.md`,
`REFERENCE_LAYER_CONTRACT.md`) all had dangling references to the deleted
modules/docs fixed. Several protected/live files carried stale comments
naming `sim/juggle_demo.py` or `controller/zmq_target.py` as if still
present (`controller/telemetry.py`, `sim/ball/manager.py`,
`sim/juggle_online.py`, `ros_ws/.../motion/ipc.py`, `docs/can_bridge/
control.md`) — corrected to past tense with the deletion date. Plans and
logbook entries under `plans/archived/` and `logbook/` were left untouched
(history, per convention). One in-scope parked-plan mention
(`plans/parked/refactor-2026-07.md:336`, a "DONE 2026-08-01" changelog line
about the original `controller/demo/` → `sim/juggle_planner/` move) was also
left as historical narrative rather than edited — same treatment as
logbook/archived, on the judgment that a completed changelog entry from a
different, earlier action shouldn't be rewritten for a later, unrelated
deletion.

`plans/active/two-ball-skill-stack.md` § 6 rows updated: A and D marked
**done 2026-09-09**; B marked **done 2026-09-09 (partial)** naming the
surviving modules; C marked **traced 2026-09-09, kept in full** with the
rosbag-integration reason.

## Verification

- Smoke: `python -c "import sim.main"` — clean, no output (2026-09-09).
- Smoke: `python sim/main.py --no-viewer --duration 0.5 --pose 0,0,50,0,0,0`
  (2026-09-09) — clean exit, tracking error 0.032 mm / 0.0027 deg.
- Smoke: all Cluster-B kept modules import cleanly in one process
  (`sim.toss_gate`, `sim.reload_gate`, `sim.ball_butler.sim`,
  `sim.input.toss_loop`, `sim.input.scripted`, `sim.viz.reference_plot`,
  `controller`) — 2026-09-09.
- Gate (`./run_tests.sh --full`, run 2026-09-09): **parallel 6780 passed, 9
  skipped, 1 xfailed in 353.52 s; serial 4 passed, 6790 deselected in
  25.64 s; total 385 s — RESULT: PASS (exit 0)**. No test failures — nothing
  needed deleting-and-re-running.
