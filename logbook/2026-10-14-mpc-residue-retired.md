---
title: MPC-era residue retired - juggle cluster, sim island, K1-K6 enforcement, IPC symbols, diagnose solver panels
type: refactor
date: 2026-10-14
status: resolved
phase: "two-ball-skill-stack — residue (post-R6)"
related_plan: skill-stack-dress-rehearsals-and-residue.md
related:
  - 2026-10-13-skill-stack-r6-close-out.md
  - 2026-09-01-mpc-chain-removed.md
  - 2026-09-09-skill-stack-r0-dead-layer-deletion.md
files_changed:
  - sim/juggle_{bb_catch,catch,selfcatch,throw}.py, sim/juggle_planner/ (deleted), tests/sim/test_juggle_* and test_demo_juggle_planner.py (deleted)
  - sim/hand/, sim/input/, sim/tools/, sim/viz/{reference_plot,horizon}.py (deleted), 5 tests/sim files deleted
  - controller/target.py, controller/REFERENCE_LAYER_CONTRACT.md, tests/sim/test_make_feasible_events.py, tests/sim/test_multi_event.py (deleted)
  - controller/{__init__,README,ballistics,feasibility,hermite,plant,telemetry,PLANT_INTERFACE_CONTRACT}.py/md (MPC wording)
  - ros_ws/src/jugglebot/jugglebot/motion/ipc.py (878 -> ~560 lines)
  - ros_ws/src/jugglebot/jugglebot/ (rename sweep, 31 files)
  - sim/analysis/ (diagnose, compare, compare_sessions, plot_*, report_html, known_issues.yaml, logbook_search), .claude/commands/diagnose.md
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md, CLAUDE.md, pyproject.toml, config/hardware_config.yaml (comment)
  - tools/probes/archived/ (8 probes moved)
subsystem:
  - skill-stack
  - cleanup
tags:
  - dead-code
---

# MPC-era residue retired

## What
- **Juggle cluster (unit H).** Deleted `sim/juggle_{bb_catch,catch,selfcatch,throw}.py`, `sim/juggle_planner/`, five `tests/sim/test_juggle_*` files and `test_demo_juggle_planner.py` (the strict xfail's test went with them). 8 probes `git mv`'d to `tools/probes/archived/`; `juggle_cup_bandlimit` / `juggle_tilt_bandlimit` stay (plant-only). KEPT: `sim/juggle_noise.py` (live importers `sim/skills_gate.py:135`, `throw_outcome_bag_probe.py:125`; misnamed) and `sim/juggle_tilt.py` (+ test; oracle for `tests/motion/test_cup_realize.py`).
- **Sim island (U1).** Deleted `sim/hand/`, `sim/input/`, `sim/tools/`, `sim/viz/{reference_plot,horizon}.py` and their tests. KEPT: `sim/ball_butler/sim.py` - the census called it dead, but the orchestrator's check found it is the bit-identical twin of the BB throw solver (`tests/ros/test_throw_ballistics.py::TestPitchCoreTwin`); `BallSpawn` moved into it. `test_ballistics.py` and `test_ball_butler_sim.py` re-pointed, not deleted.
- **K1-K6 (unit I).** `controller/target.py`, `REFERENCE_LAYER_CONTRACT.md` and the two feasibility tests deleted; MPC hits in `controller/` 45 -> 15, all dated history. KEPT: `controller/feasibility.py` (the quintic test's oracle). The six K1-K6 rows of `motion/skills/INVARIANTS.md` are marked RETIRED 2026-10-14 (their enforcement point and tests deleted; I-PLAN-1/2 and the splice rule are the live twins); CLAUDE.md's architecture tree and `controller/` bullet, `pyproject.toml`'s nightly-tier text and one `config/hardware_config.yaml` comment were updated by the orchestrator (generated artifacts unchanged).
- **IPC symbols (U2/U3).** `ipc.py` lost `make_mpc_target`, `make_mpc_mode`, `make_target_feedback`, the target/feedback addresses and topics, `MpcBridgeIPC`, `MpcTargetIPC`, `TargetFeedbackPub/Sub`; MPC hits there ~60 -> 9. Rename sweep over 31 files; old names zero except history notes. KEPT: the wire names (`make_mpc_command`, `b'mpccmd'`, `MPC_COMMAND_ADDR`, `MpcCommandPub`, `mpc_active`), `motor_guard.py` (retained deliberately).
- **Diagnose (U4).** MPC-only analyses, five `known_issues` entries and the solver panels deleted; generic tracking/oscillation/rosbag analysis kept. New CLI: `diagnose.py [csv_path] [--rosbag P] [--ros-log-dir D] [--json] [--plots] [--no-html]` (`--budget-ms` removed, CSV optional, flag source `mpc` -> `telemetry`).

## Why
Owner, 2026-10-14: the xfail's test is not needed, so delete the cluster; remove the K1-K6 enforcement now that nothing live calls it; all references to MPC in live code should be gone.

## Verification
- G-adjacent units, 2026-10-14: U1 scoped list incl. `TestPitchCoreTwin` -> 157 passed in 270.96 s; `sim/main.py --no-viewer` smoke OK.
- U2, 2026-10-14, `pytest -q tests/teensy_link tests/motion/{test_hardening,test_trajectory_emitter,test_motor_guard,test_motor_guard_friction_ff}.py tests/ros/{test_install_segment,test_teensy_bridge_node*,test_trajectory_node*}.py` -> 1280 passed, 1 failed (`test_install_segment::test_a_fresh_throw_installs_through_the_real_worker_process`, "solver did not answer by the wire deadline", worker-process timing, not investigated).
- I, 2026-10-14, `pytest -q -p no:cacheprovider tests/motion/test_trajectory_quintic.py tests/motion/test_cup_cycle.py tests/sim/test_toss_geometry.py tests/sim/test_plant_interface_contract.py tests/sim/test_telemetry_roundtrip.py` -> 135 passed in 69.26 s.
- U3, 2026-10-14, scoped run over teensy_link, firmware/test_hermite_xref, motion/motor_guard*, trajectory_emitter, leg_torque_ff, hardening, ros/install_segment + bridge_node* + trajectory_node -> 1391 passed, 1 failed (the byte-pinned V5 fixture still carried the old topic bytes), 1 skipped in 188 s; after the fixture fix `pytest -q -p no:cacheprovider tests/motion/test_trajectory_emitter.py tests/teensy_link` -> 467 passed in 20 s.
- U4, 2026-10-14, `pytest -q -p no:cacheprovider tests/sim/{test_diagnose_oscillation,test_diagnose_trajectory,test_compare_sessions,test_logbook_search,test_telemetry_roundtrip,test_logbook_front_matter}.py` -> 70 passed, 4 skipped; rosbag smoke exit 0.
- Unit H and I handoffs record grep proofs, not pytest counts.
- Full gate: `./run_tests.sh --full`, run 2026-10-14 09:34-09:39 on the combined tree of both commits (`temp/logs/gate_wave2_final_20261014.log`): **PASS — 6623 passed, 9 skipped in 286.46 s (parallel), serial 7 passed in 27.21 s**

## Open
- Deliberately left: `mpc_active` (firmware wire), `MPC_DT` in `xref.py`, `MPC_CMD_STALENESS_S` and `mpc_age` in motor_guard, dated history lines. Live launch must restart after a colcon rebuild (both ZMQ ends renamed).
- `sim/juggle_noise.py` / `sim/juggle_tilt.py` rename (misnamed) not done.
- `/diagnose` command's CLI shape changed; `sim/analysis/diagnose.py:1886` regex on `TargetFeedbackPub:` is a dead pattern; `docs/analysis/logbook.md` keeps `mpc` subsystem-tag examples (legitimate).
- Stale prose not touched: `cup_cycle.py` docstrings, `sim/viz/recording.py`, `test_juggle_tilt.py` comment, `sim/analysis/known_issues.yaml:160`, `ros_ws/docs/hand_command_continuity.md:323`, `tests/hardware/session_cadence_ladder.md:922`, `.claude/commands/commit.md:48`.
- The one failure in U2's scoped run (`tests/ros/test_install_segment.py::test_a_fresh_throw_installs_through_the_real_worker_process`, "solver did not answer by the wire deadline") was load: it failed alone at load average 8.3 on the six-core Jetson while four agent units ran tests concurrently, and passed alone (1.68 s) at load 2.1 once they finished (2026-10-14, `pytest -q -p no:cacheprovider tests/ros/test_install_segment.py::test_a_fresh_throw_installs_through_the_real_worker_process` -> 1 passed); the full gate below ran it green. Its 175 ms fresh-origin budget is real and the test is not `serial`-marked; a second load failure would be the trigger to mark it.
