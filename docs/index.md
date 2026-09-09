# Jugglebot Documentation

Jugglebot is a 6-DoF Stewart platform robot with ball-catching capabilities. It uses string-driven linear actuators controlled by ODrive motor controllers, with a Jetson running ROS2 for high-level coordination and a standalone Python process for real-time motion control.

## Documentation Sections

### [Motion Planner](motion_planner/index.md) — Stewart Platform

The motion planner controls the **Stewart platform** (the 6-leg parallel mechanism). It does not cover the Ball Butler subsystem or the linear throw axis, which have their own independent control paths.

Complete technical documentation for the motion planning and control subsystem, covering:

- **[Architecture](motion_planner/architecture.md)** — System design, data flow, and control philosophy
- **[Kinematics](motion_planner/kinematics.md)** — Stewart platform geometry, inverse/forward kinematics, Jacobian
- **[Dynamics](motion_planner/dynamics.md)** — Gravity compensation, inertia feedforward, force decomposition
- **[Trajectory Planning](motion_planner/trajectory.md)** — Quintic polynomial generation, feasibility checking, dynamic targets (historical — MPC now handles trajectory planning)
- **[Workspace Safety](motion_planner/workspace.md)** — Limit enforcement, singularity avoidance, fault detection
- **[Motor Command Safety](motion_planner/safety.md)** — Max-deviation check, motor feedback gating, defense-in-depth fault checks
- **[Motor Guard](motion_planner/control_loop.md)** — The 500 Hz interpolator + safety monitor, operating modes, IPC layer
- **[System Integration](motion_planner/integration.md)** — ROS2 bridge, CAN interface, full system startup
- **[Operations Guide](motion_planner/operations.md)** — Running tests, tuning gains, extending the system
- **[Validation Results](motion_planner/results.md)** — Summary of hardware test outcomes

### [CAN Bridge](can_bridge/index.md) — Teensy 4.1 Hardware Offload

A dedicated Teensy 4.1 hosts CAN bus communication and the leg setpoint
interpolator, moving both off the Jetson's non-real-time Linux scheduler.
It talks to the Jetson over UDP/Ethernet and owns three isolated CAN buses
(legs + hand, Ball Butler, catching cone).

- **[Overview](can_bridge/index.md)** — topology, time-sync, current production status
- **[Control Flow](can_bridge/control.md)** — the 40 Hz→500 Hz leg interpolation ladder and the hand's separate relay path
- **[Safety Mechanisms](can_bridge/safety.md)** — staleness watchdogs, the E-STOP fault machine, and CAN bus health monitoring

### Simulation MPC (REMOVED 2026-09-01, historical)

The CasADi/IPOPT receding-horizon MPC described here was deleted 2026-09-01
— dormant since 2026-08-01 and superseded by the unified 7-DoF planner. The
final implementation is preserved at git tag `mpc-final`; see
`logbook/2026-09-01-mpc-chain-removed.md`. Its documentation pages
(`docs/sim_mpc/`) were deleted 2026-09-09 (R0 dead-layer deletion,
`plans/active/two-ball-skill-stack.md` § 6) along with the mkdocs nav
section that linked them.

### [Hardware Analysis](analysis/index.md) -- Diagnosis & Engineering Logbook

Tools for diagnosing hardware test sessions, comparing before/after results, and maintaining a structured engineering logbook with full traceability from code to investigation.

- **[Diagnosis Tools](analysis/diagnosis.md)** -- Session analysis engine, interactive Plotly reports, session comparison, known issues catalog
- **[Engineering Logbook](analysis/logbook.md)** -- Structured investigation records, prior-art search, `/investigate` pipeline, commit traceability

## Repository Structure

```
Jugglebot/
├── config/                     # Code-generated protocol & hardware constants
│   ├── jugglebot_protocol.yaml
│   ├── hardware_config.yaml
│   └── generate_config.py
├── ros_ws/src/jugglebot/       # ROS2 workspace
│   └── jugglebot/
│       ├── can/                # CAN bus interface (bus, odrive, ball_butler)
│       ├── motion/             # Motion planner (pure Python, no ROS2)
│       ├── can_node.py         # ROS2 CAN interface node
│       ├── orchestrator_node.py
│       └── state_machine.py
├── controller/                 # Reference/telemetry layer (portable, no ROS2, no MuJoCo)
│   └── target.py               # TargetCommand + TargetSource protocol
├── sim/                        # MuJoCo simulation
│   ├── plant/                  # MuJoCo + hardware plant interfaces
│   ├── hand/                   # Hand coordination, catch/throw trajectories
│   ├── input/                  # Input adapters (scripted, interactive, ZMQ)
│   ├── viz/                    # Telemetry logging, horizon renderer, dashboard
│   ├── model/                  # MJCF model + generator
│   ├── analysis/               # Hardware diagnosis, comparison, reporting
│   └── main.py                 # Entry point + 50 Hz MPC loop
├── logbook/                    # Engineering logbook (investigation entries)
├── tools/                      # Standalone hardware test harnesses
├── docs/                       # This documentation
└── MOTION_PLANNER_PLAN.md      # Historical development plan & test log
```
