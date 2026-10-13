# Jugglebot Documentation

Jugglebot is a 6-DoF Stewart platform robot with ball-catching capabilities. It uses string-driven linear actuators controlled by ODrive motor controllers, with a Jetson running ROS2 for high-level coordination and a skill-stack planner whose segments the can-bridge Teensy interpolates at 500 Hz.

## Documentation Sections

### [Motion Planner](motion_planner/index.md) — Stewart Platform

The motion planner controls the **Stewart platform** (the 6-leg parallel mechanism). It does not cover the Ball Butler subsystem or the linear throw axis, which have their own independent control paths.

The live chain is `skill_node` -> `trajectory/install_segment` -> `trajectory_node` -> ZMQ :5557 -> `teensy_bridge_node` -> can-bridge Teensy.

- **[Overview](motion_planner/index.md)** — the skill stack, status and history
- **[Architecture](motion_planner/architecture.md)** — processes, IPC, CAN side, timing, invariants
- **[Kinematics](motion_planner/kinematics.md)** — Stewart platform geometry, inverse/forward kinematics, Jacobian
- **[Dynamics](motion_planner/dynamics.md)** — Gravity compensation, inertia feedforward, force decomposition
- **[Workspace Safety](motion_planner/workspace.md)** — Limit enforcement, singularity avoidance
- **[Motor Guard (parked)](motion_planner/control_loop.md)** — the parked fallback and Hermite xref twin
- **[Validation Results](motion_planner/results.md)** — historical hardware test outcomes

### [CAN Bridge](can_bridge/index.md) — Teensy 4.1 Hardware Offload

A dedicated Teensy 4.1 hosts CAN bus communication and the leg setpoint
interpolator, moving both off the Jetson's non-real-time Linux scheduler.
It talks to the Jetson over UDP/Ethernet and owns three isolated CAN buses
(legs + hand, Ball Butler, catching cone).

- **[Overview](can_bridge/index.md)** — topology, time-sync, current production status
- **[Control Flow](can_bridge/control.md)** — the 40 Hz→500 Hz leg interpolation ladder and the hand's separate relay path
- **[Safety Mechanisms](can_bridge/safety.md)** — staleness watchdogs, the E-STOP fault machine, and CAN bus health monitoring

### History

The CasADi/IPOPT MPC was removed 2026-09-01 (git tag `mpc-final`); the FSM stack was deleted 2026-09-24 (tag `fsm-final`).

### [Hardware Analysis](analysis/index.md) -- Diagnosis & Engineering Logbook

Tools for diagnosing hardware test sessions, comparing before/after results, and maintaining a structured engineering logbook with full traceability from code to investigation.

- **[Diagnosis Tools](analysis/diagnosis.md)** -- Session analysis engine, interactive Plotly reports, session comparison, known issues catalog
- **[Engineering Logbook](analysis/logbook.md)** -- Structured investigation records, prior-art search, `/investigate` pipeline, commit traceability

## Repository Structure

```
Jugglebot/
├── config/                     # hardware_config.yaml -> generated constants
├── controller/                 # shared motion primitives (ballistics, feasibility, hermite, plant)
├── teensy_link/                # can-bridge UDP transport (root package, not installed into ROS)
├── ros_ws/src/jugglebot/       # ROS2 package: skill_node, trajectory_node, teensy_bridge_node, mocap_node, motion/, tracking/
├── sim/                        # MuJoCo simulation
├── logbook/                    # Engineering logbook (investigation entries)
├── plans/                      # active / parked / archived plans
├── tools/                      # utilities and probes
└── docs/                       # This documentation
```
