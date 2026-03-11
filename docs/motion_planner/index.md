# Motion Planner Overview

!!! info "Scope: Stewart Platform Only"
    This motion planner controls the **Stewart platform** — the 6-leg parallel mechanism. It does not cover the Ball Butler subsystem or the linear throw axis, which are controlled independently. The throw axis is treated as a static payload for dynamics purposes (its mass is included in the platform inertia model).

The motion planner is the subsystem responsible for commanding the Stewart platform's six leg actuators. Given a desired platform pose (position + orientation), it computes motor positions, velocity feedforward, and torque feedforward for each leg, and sends these commands to the ODrive motor controllers over CAN bus at 500 Hz.

## Design Principles

**Pure Python, no ROS2 in the core.** The motion planning modules (`motion/`) have zero ROS2 dependency. This keeps the control loop deterministic and testable without a ROS2 environment. A separate bridge node translates between ROS2 topics and the control process.

**Position control with feedforward.** The ODrive controllers run cascaded PID at 8 kHz. The motion planner provides three values per leg each cycle:

| Field | Source | Purpose |
|---|---|---|
| `input_pos` | Position IK | Where the leg should be |
| `vel_ff` | Velocity IK via Jacobian | How fast it should be moving |
| `torque_ff` | Dynamics model | What force is needed (gravity + inertia) |

The dynamics model is expressed through feedforward, not feedback. The ODrive's PID only corrects for modelling errors and disturbances.

**Safety first.** There is only one robot. Every trajectory is feasibility-checked before execution. Workspace limits are enforced every control cycle. The system fails safe — a stale position command holds position rather than continuing to apply force.

## Module Map

```
motion/
├── geometry.py       Load Stewart platform dimensions from config
├── ik_solver.py      Position/velocity/acceleration inverse kinematics + FK
├── conversions.py    mm <-> rev, force <-> torque unit conversions
├── dynamics.py       Gravity wrench, inertia wrench, feedforward torques
├── workspace.py      Workspace limit checking, singularity monitoring
├── trajectory.py     Quintic trajectory generation, feasibility, manager
├── ipc.py            ZeroMQ PUB/SUB inter-process communication
└── control_loop.py   500 Hz standalone control process
```

External:

- `motion_bridge_node.py` — ROS2 node that bridges topics to/from the control process via IPC
- `tools/*.py` — Standalone hardware test harnesses that bypass ROS2

## Data Flow Summary

```
                          ROS2 World
                              |
                    motion_bridge_node.py
                       (ZeroMQ IPC)
                              |
                  +-----------+-----------+
                  |    control_loop.py    |
                  |      (500 Hz)        |
                  |                       |
                  |  trajectory.py        |
                  |  ik_solver.py         |
                  |  dynamics.py          |
                  |  workspace.py         |
                  +-----------+-----------+
                              |
                    (pos, vel_ff, torque_ff)
                              |
                       motion_bridge_node
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
| [Trajectory](trajectory.md) | Quintic polynomials, feasibility | How are smooth motions planned? |
| [Workspace](workspace.md) | Limits, singularity avoidance | How does the system stay safe? |
| [Motor Command Safety](safety.md) | Slew limiter, feedback gating, fault checks | How are dangerous commands prevented? |
| [Control Loop](control_loop.md) | 500 Hz process, modes, IPC | What happens every 2 ms? |
| [Integration](integration.md) | ROS2 bridge, CAN, startup | How does it connect to the rest of the robot? |
| [Operations](operations.md) | Testing, tuning, extending | How do I work with this system? |
| [Results](results.md) | Hardware validation data | How well does it actually work? |

## Development History

The motion planner was developed in 8 phases, each with explicit exit criteria and graduated hardware exposure. The full development narrative, including all test results and decisions, is preserved in [`MOTION_PLANNER_PLAN.md`](https://github.com/Project-DeepBlue-Juggling/Jugglebot/blob/refactor/MOTION_PLANNER_PLAN.md) at the repository root. This documentation describes the system as it exists now; the plan document records how it got here.
