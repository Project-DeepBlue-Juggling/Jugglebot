# `controller/`

Pure-Python motion primitives shared by the MuJoCo simulation (`sim/`) and
the ROS2 hardware stack (`ros_ws/`). No ROS2 imports, no CasADi.

| module | what it is |
|---|---|
| `ballistics.py` | Free-flight solutions; throw/catch pose geometry, `TILT_LIMIT_RAD` |
| `catch_optimizer.py` | `CatchHeightOptimizer`, catch orientation/pose from ball state |
| `feasibility.py` | Segment feasibility for the K1–K6 reference contract |
| `hermite.py` | Quintic / Hermite interpolation with accel and jerk integrals |
| `plant.py` | `PlantInterface` / `PlantState` — the plant contract |
| `scheduler.py` | `EventScheduler` — catch/throw event sequencing |
| `target.py` | `TargetSource` / `TargetCommand` / `ReferenceEvent` |
| `telemetry.py` | `TelemetryLogger`, `StepRecord`, CSV round-trip |
| `toss_motion_source.py` | Toss motion generation |
| `zmq_target.py` | `ZmqTargetSource` — ZMQ target ingest |

Normative documents: `PLANT_INTERFACE_CONTRACT.md`,
`REFERENCE_LAYER_CONTRACT.md`, `SCHEDULER_CONTRACT.md`.

## Removed: the MPC chain (2026-09-01)

`mpc.py`, `params.py`, `runner.py`, `hardware_plant.py`, `hardware_hooks.py`,
`hot_loop_contract.py`, `generate_solver.py`, `generated/`, plus the root
`run_mpc.py`, the ROS2 `motion_bridge_node` / `mpc_bridge_node`, `sim/main.py`'s
`--mpc` modes and the `HOT_LOOP_CONTRACT.md` / `DIAG_SCHEMA_CONTRACT.md`
normative docs were deleted outright. The chain had been operationally dormant
since 2026-08-01 (`plans/parked/refactor-2026-07.md` Phase 3, which parked it
with a revival path), and the unified 7-DoF planner that landed 2026-09-01
(`plans/active/unified-7dof-planner.md`) is the lower-rate replanner that
parking preserved the option for — so the revival path is retired, not deferred.

The final implementation is preserved at git tag **`mpc-final`**; the removal is
recorded in `logbook/2026-09-01-mpc-chain-removed.md`.

Docstrings in the modules above still name `MPCParams` fields (e.g.
`max_leg_vel_mmps`, `tau`) as the historical source of a numeric argument.
Those are provenance notes, not live imports — the values now come from the
caller.
