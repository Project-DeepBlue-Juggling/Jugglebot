# Motion Planner Overview

!!! info "Scope: Stewart Platform Only"
    This motion planner controls the **Stewart platform** — the 6-leg parallel mechanism. It does not cover the Ball Butler subsystem or the linear throw axis, which are controlled independently. The throw axis is treated as a static payload for dynamics purposes (its mass is included in the platform inertia model).

The motion system is responsible for commanding the Stewart platform's six leg actuators. The MPC solver (50 Hz) plans trajectories for all input modes; the motor guard (500 Hz) interpolates between MPC commands and validates them against motor feedback before forwarding to the ODrive motor controllers over CAN bus.

## Design Principles

**Pure Python, no ROS2 in the core.** The motion modules (`motion/`, `controller/`) have zero ROS2 dependency. This keeps the motor guard deterministic and testable without a ROS2 environment. Bridge nodes translate between ROS2 topics and ZeroMQ IPC.

**MPC is the sole motion planner.** All input modes (spacemouse, GUI, shell, catch coordinator) set target poses that the MPC tracks. The MPC outputs pre-computed motor commands — the motor guard does not compute IK, dynamics, or trajectories.

**Position control with feedforward.** The ODrive controllers run cascaded PID at 8 kHz. The system provides three values per leg each cycle:

| Field | Source | Purpose |
|---|---|---|
| `input_pos` | MPC solver (symbolic IK constraints) | Where the leg should be |
| `vel_ff` | HardwarePlant (finite-difference of MPC extensions) | How fast it should be moving |
| `torque_ff` | HardwarePlant (Newton-Euler dynamics model) | What force is needed (gravity + inertia) |

The dynamics model is expressed through feedforward, not feedback. The ODrive's PID only corrects for modelling errors and disturbances.

**Safety first.** There is only one robot. The motor guard validates every command against motor feedback (max-deviation, overspeed, workspace limits). The system fails safe — a stale position command holds position rather than continuing to apply force.

## Module Map

```
controller/                    MPC solver (portable, no ROS2, no MuJoCo)
├── mpc.py                     MPCController (CasADi/IPOPT, symbolic IK)
├── params.py                  Tuning parameters
└── target.py                  TargetCommand + TargetSource protocol

motion/                        Motor guard + motion primitives
├── motor_guard.py             500 Hz interpolator + safety monitor
├── ipc.py                     ZeroMQ PUB/SUB inter-process communication
├── geometry.py                Load Stewart platform dimensions from config
├── ik_solver.py               Position/velocity/acceleration inverse kinematics + FK
├── conversions.py             mm <-> rev, force <-> torque unit conversions
├── dynamics.py                Gravity wrench, inertia wrench, feedforward torques
├── workspace.py               Workspace limit checking, singularity monitoring
└── motor_commands.py          Cartesian → motor command mapping
```

External:

- `motion_bridge_node.py` — ROS2 node that bridges motor guard ↔ CAN via IPC
- `mpc_bridge_node.py` — ROS2 node that bridges input targets → MPC via IPC
- `controller/hardware_plant.py` — Converts MPC output to motor commands with feedforward
- `tools/*.py` — Standalone hardware test harnesses that bypass ROS2

## Data Flow Summary

```
                          ROS2 Input Sources
                     (spacemouse, GUI, shell, catch)
                              |
                    mpc_bridge_node.py  (ZMQ :5558)
                              |
                    +---------+---------+
                    | MPC solver (50 Hz)|
                    | CasADi/IPOPT      |
                    | symbolic IK       |
                    +---------+---------+
                              |
                    HardwarePlant (vel_ff, torque_ff)
                              |
                        ZMQ :5557
                              |
                    +---------+---------+
                    | motor_guard (500Hz)|
                    | quadratic interp  |
                    | safety checks     |
                    +---------+---------+
                              |
                    (pos, vel_ff, torque_ff)
                              |
                    motion_bridge_node  (ZMQ :5556)
                              |
                         can_node.py
                              |
                        CAN bus → ODrives
```

## Quick Links

| Topic | Page | Key Question Answered |
|---|---|---|
| [Architecture](architecture.md) | How the system is structured | How do the pieces fit together? |
| [Kinematics](kinematics.md) | Geometry, IK, FK, Jacobian | How does a platform pose become leg lengths? |
| [Dynamics](dynamics.md) | Gravity, inertia, feedforward | How are motor torques computed? |
| [Trajectory](trajectory.md) | Quintic polynomials, feasibility | How were smooth motions planned? (historical — MPC now handles this) |
| [Workspace](workspace.md) | Limits, singularity avoidance | How does the system stay safe? |
| [Motor Command Safety](safety.md) | Max-deviation, feedback gating, fault checks | How are dangerous commands prevented? |
| [Motor Guard](control_loop.md) | 500 Hz process, modes, IPC | What happens every 2 ms? |
| [Integration](integration.md) | ROS2 bridges, CAN, startup | How does it connect to the rest of the robot? |
| [Operations](operations.md) | Testing, tuning, extending | How do I work with this system? |
| [Results](results.md) | Hardware validation data | How well does it actually work? |

## Development History

The motion planner was developed in 8 phases, each with explicit exit criteria and graduated hardware exposure. The full development narrative, including all test results and decisions, is preserved in [`MOTION_PLANNER_PLAN.md`](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/MOTION_PLANNER_PLAN.md) at the repository root. The subsequent MPC-native refactor is documented in [`MPC_NATIVE_REFACTOR_PLAN.md`](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/MPC_NATIVE_REFACTOR_PLAN.md). This documentation describes the system as it exists now; the plan documents record how it got here.
