---
title: "MPC chain removed — the dormant solver, its hardware path and its sim modes are deleted, not parked"
type: refactor
date: 2026-09-01
status: resolved
phase: "post unified-7dof-planner Phase 1"
related_plan: unified-7dof-planner.md
files_changed:
  - controller/
  - run_mpc.py
  - sim/main.py
  - ros_ws/src/jugglebot/jugglebot/
  - tests/sim/
  - CLAUDE.md
  - plans/parked/refactor-2026-07.md
subsystem:
  - mpc
  - sim
  - ros
  - testing
---

# MPC chain removed

## What and why

`plans/parked/refactor-2026-07.md` Phase 3 parked the MPC chain on 2026-08-01
**with a revival path**, on the owner's "remove for now, bring it back later".
The unified 7-DoF planner landed 2026-09-01 (`663d8cb`, `c1477e6`) and **is**
the lower-rate replanner that parking preserved the option for, so the revival
scenario is superseded rather than deferred. Owner authorised outright deletion
2026-09-01; git history and the tag **`mpc-final`** on the pre-removal commit
preserve the final implementation.

Two secondary forces: the battery was the sole `--full` RED
(`tests/sim/test_solver_failures.py::TestWarmStartIntegrity`, Hypothesis DB
replay at `controller/mpc.py:1041`), and nothing had touched `mpc.py`,
`params.py` or `runner.py` since 2026-05-21 — **zero commits to any of the
three since 2026-06-01**.

## Deleted

`controller/{mpc,params,runner,hardware_plant,hardware_hooks,hot_loop_contract,generate_solver}.py`,
`controller/generated/` (`mpc_gen.so`/`.hash`),
`controller/{HOT_LOOP_CONTRACT,DIAG_SCHEMA_CONTRACT}.md`,
`controller/teensy_link.py` (the aliasing shim, due for deletion after 2026-09
and with no consumer but its own test),
`run_mpc.py`,
`ros_ws/.../{motion_bridge_node,mpc_bridge_node}.py` + their `setup.py` entry
points, `sim/{demo_mpc.py,hand/feasibility.py,analysis/record_baselines.py}`,
`tools/probes/replay_hardware_csv.py`, and **24 test files** (the MPC battery, the
HardwarePlant battery, `tests/sim/helpers.py`, `tests/sim/_hardware_plant_stub.py`,
`tests/ros/test_motion_bridge_node.py`, `tests/teensy_link/test_compat_shim.py`).

Two of those 24 are operator harnesses and went for a **safety** reason, not
tidiness: `tests/hardware/th_t2b1_publisher_kill_test.py` and
`th_t2a1_can_unplug_test.py` instrumented the telemetry-stale ESTOP cascade that
lived in `hardware_plant.py`. Both had already PASSED and been recorded
(`logbook/2026-05-18-hardware-bringup-t2b1-t2a1-cascade-validation.md`), and no
live process emits those log lines any more. T-H-T2a-1 is a MEDIUM-HIGH-hazard
procedure — it has the operator physically unplug CAN and the platform freewheels
under gravity. A hazardous procedure that can no longer observe the cascade it
exists to measure is worse than dead code, so it went rather than sat. Their
shared helper `tests/hardware/_th_test_common.py` **stays** — `tilt_cal_grid.py`,
`toss_cal_grid.py` and `rescore_artifact.py` still use it.

`sim/main.py` lost its MPC half: **1585 → 297 lines**. Gone are `--mpc`,
`--hardware` and the eight modes that forced MPC on (`--catch`, `--juggle`,
`--interactive-catch`, `--throw-catch`, `--cycle-time`, `--keyboard`,
`--spacemouse`, `--trajectory`) plus every `TargetSource` adapter. What remains
is the direct pose-command simulator: `--pose`, `--sequence`, `--dashboard`,
`--no-viewer`, `--duration`.

## Kept, and why

- **`controller/{ballistics,catch_optimizer,feasibility,hermite,plant,scheduler,target,telemetry,toss_motion_source,zmq_target}.py`** —
  live sim and hardware consumers that never went through the solver. The
  package no longer imports CasADi at all.
- **`motion/motor_guard.py`** — it predates and outlives the MPC. It is the
  validated Python twin the `hermite_xref` firmware trust chain drives
  (`tools/probes/teensy_link_profiling/hermite_xref/xref.py`) and its safety
  tests run per-commit. Now a parked fallback with neither feeder nor consumer.
- **`controller/zmq_target.py`** — its ROS publisher (`mpc_bridge_node`) is
  gone, so :5558 has no producer, but the decode/clamp path is
  publisher-agnostic and independently tested. Flagged as a residual, not
  deleted: removing it would cascade into `ros_ws/.../motion/ipc.py`, a shared
  module with live classes.
- **`ros_ws/.../motion/ipc.py` residuals, and one exception.** The module keeps
  live classes (`MpcCommandPub` feeds the :5557 funnel from `trajectory_node`),
  so it stays, and its now-producerless address constants (`MPC_TARGET_ADDR`
  :5558, `MPC_FEEDBACK_ADDR` :5559) are annotated rather than removed — both
  still have live *consumers* worth keeping decodable. The **:5560
  session-metadata channel is the exception and was deleted**:
  `SessionMetadataPush` / `SessionMetadataPull` / `SESSION_ADDR` /
  `make_session_start` carried the telemetry-CSV filename from `sim/main.py
  --mpc` to `mpc_bridge_node` for ROS2 log correlation, and **both** endpoints
  went with the chain, leaving no producer and no consumer and no test. A
  tombstone comment marks the site; the code is at `mpc-final`.
- **`PLANT_INTERFACE_CONTRACT.md`**, **`REFERENCE_LAYER_CONTRACT.md`**,
  **`SCHEDULER_CONTRACT.md`** — the contracts still bind `MuJoCoPlant`, the
  reference layer and the scheduler. `PLANT_INTERFACE_CONTRACT.md` carries a
  dated header saying every `HardwarePlant` reference in it is now historical
  and its line links resolve only at `mpc-final`.
- **`sim/analysis/diagnose.py`** and its `known_issues.yaml` — a log analyser.
  The archived `mpc_*.csv` telemetry it reads did not change when its producer
  went away, so the MPC-era patterns are retained deliberately and annotated.
- **`docs/sim_mpc/`** (nine mkdocs pages) — retained as the fullest prose
  description of the removed design, under a `REMOVED` banner and a nav label
  that says so. Every code reference in them resolves at `mpc-final`.

Three test files kept with only their MPC half cut, because their subject
survives: `test_multi_event.py` (TossLoopController ref-event construction),
`test_plant_interface_contract.py` (P1–P4 against MuJoCoPlant),
`test_zmq_corruption.py` (msgpack/ZMQ frame corruption — the methodology still
governs the live toss path).

One rule got **stricter** as a side effect. `mpc_bridge_node._on_platform_pose`
was obligation B1 in `ros_ws/docs/levelling_frame.md` § "Revival obligations" —
a *second* copy of the C-LEVEL-1 levelling application that would have had to be
given the tilt map before the MPC could ship again, or the two pose paths would
disagree about where level is. Deleting the node discharges the obligation, and
B1 was the levelling manifest's single declared exception: with it gone,
`test_every_apply_has_a_build_in_the_same_scope` now runs unconditionally.

## Scope note

The authorising brief's known-keep list named only
`controller.{ballistics,target,telemetry,scheduler,plant}` as live sim imports.
The dependency sweep found that **`sim/main.py --mpc` also reached `mpc.py`,
`params.py`, `runner.py` and `hardware_plant.py`**, and that `sim/hand/__init__.py`
eagerly imported `FeasibilityChecker` → `controller.mpc`, so *any* `import
sim.hand` pulled in CasADi. Removing the solver therefore had to take the sim's
MPC modes with it. That was treated as in scope — `controller/HOT_LOOP_CONTRACT.md`
itself classified `sim/main.py --mpc` as part of the dormant chain, and the live
sim gates (`sim/{toss,reload,cycle}_gate.py`) have no MPC imports — but it is a
larger amputation than the brief's file list implied and is recorded here for
that reason.

## Verification

All runs 2026-09-01, on the Jetson under `~/Desktop/PDJ_venv/venv`.

| command | result |
|---|---|
| `python -m pytest tests/ -q --collect-only` | **6425 collected, 0 errors** |
| `python -m pytest tests/sim/ -q` | **1030 passed, 2 xfailed** in 666.06 s |
| `python -m pytest tests/ros/ -q` | **2622 passed, 1 skipped** in 362.45 s |
| `python -m pytest tests/motion/ -q` | **2123 passed, 3 skipped** in 331.76 s |
| `python -m pytest tests/teensy_link/ tests/firmware/ -q` | **644 passed** in 27.71 s |
| `python -m pytest tests/ -q -k logbook` | **34 passed, 6391 deselected** in 6.86 s |
| `python -m pytest tests/ros/test_levelling_frame.py tests/ros/test_choreography_map.py tests/ros/test_trajectory_node.py -q` | **211 passed** in 27.54 s |
| `python -m pytest tests/sim/test_zmq_corruption.py tests/sim/test_zmq_target.py tests/sim/test_catch_optimizer.py -q -m ""` | **44 passed** (the nightly-marked survivors) |

Four structural checks beyond the suite:

- `python -c "import controller, sim.hand, sim.main"` → OK, and
  **`casadi` is not in `sys.modules`** — the solver dependency is off the
  import path entirely, where before *any* `import sim.hand` pulled it in.
- `python -m compileall -q controller sim tools ros_ws/src/jugglebot/jugglebot`
  → exit 0.
- Residual references to the deleted symbols across `tests/`, measured
  2026-09-01:

  ```
  grep -rnE 'run_mpc|motion_bridge|mpc_bridge|hardware_plant|HardwarePlant|hot_loop_contract|generate_solver|mpc_gen|demo_mpc' tests/ --include='*.py'
  ```
  → **22 hits in 12 files**, every one prose — a docstring, a comment, or one
  surviving test-function name (`test_hardware_plant_honours_config_term_flags`
  in `tests/motion/test_leg_torque_ff.py`, which asserts config-flag behaviour
  that outlived its namesake). Most carry a dated removal note; none is an
  import, a call, or a subprocess invocation. The import-scoped sweep confirms
  that:

  ```
  grep -rnE 'import (run_mpc|controller\.(mpc|params|runner|teensy_link))' tests/ --include='*.py'
  ```
  → **0 hits**.
- `python tools/gen_choreography_map.py` regenerated `ros_ws/docs/choreography.md`
  (`NOT_LAUNCHED_NODES` is now empty), pinned green by `test_choreography_map.py`.

- Pre-commit gate (2026-09-01, `./run_tests.sh --full`, after the audit fixes):
  **6416 passed, 4 skipped, 2 xfailed in 423.54 s** (parallel) + 3 passed
  (serial, 17.62 s), total 447 s, **PASS — fully green**. The
  `TestWarmStartIntegrity` RED is gone with its subject, and the full tier is
  ~2.5 min faster (447 s vs 604 s pre-removal).
