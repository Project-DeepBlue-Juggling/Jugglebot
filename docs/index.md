# Jugglebot Documentation

Jugglebot is a 6-DoF Stewart platform robot with ball-catching capabilities. It uses string-driven linear actuators controlled by ODrive motor controllers, with a Jetson running ROS2 for high-level coordination and a standalone Python process for real-time motion control.

## Documentation Sections

### [Motion Planner](motion_planner/index.md) — Stewart Platform

The motion planner controls the **Stewart platform** (the 6-leg parallel mechanism). It does not cover the Ball Butler subsystem or the linear throw axis, which have their own independent control paths.

Complete technical documentation for the motion planning and control subsystem, covering:

- **[Architecture](motion_planner/architecture.md)** — System design, data flow, and control philosophy
- **[Kinematics](motion_planner/kinematics.md)** — Stewart platform geometry, inverse/forward kinematics, Jacobian
- **[Dynamics](motion_planner/dynamics.md)** — Gravity compensation, inertia feedforward, force decomposition
- **[Trajectory Planning](motion_planner/trajectory.md)** — Quintic polynomial generation, feasibility checking, dynamic targets
- **[Workspace Safety](motion_planner/workspace.md)** — Limit enforcement, singularity avoidance, fault detection
- **[Motor Command Safety](motion_planner/safety.md)** — Slew rate limiter, motor feedback gating, defense-in-depth fault checks
- **[Control Loop](motion_planner/control_loop.md)** — The 500 Hz control process, operating modes, IPC layer
- **[System Integration](motion_planner/integration.md)** — ROS2 bridge, CAN interface, full system startup
- **[Operations Guide](motion_planner/operations.md)** — Running tests, tuning gains, extending the system
- **[Validation Results](motion_planner/results.md)** — Summary of hardware test outcomes

### [Simulation MPC](sim_mpc/index.md) — CasADi Nonlinear MPC

The simulation MPC is a parallel development path that replaces the production quintic trajectory planner with a **receding-horizon nonlinear MPC** using CasADi/IPOPT. It runs in the MuJoCo simulation environment (`sim/`) and models actuator dynamics, IK constraints, and variable-resolution horizons for ball catching.

- **[NLP Formulation](sim_mpc/nlp_formulation.md)** — Decision variables, cost function, IK constraints, warm-starting
- **[Variable Horizon](sim_mpc/variable_horizon.md)** — Fine/coarse timestep schedule, urgency ramp, feasibility checking
- **[Control Loop](sim_mpc/control_loop.md)** — 50 Hz loop, target source adapters, hand coordination
- **[Plant Interface](sim_mpc/plant.md)** — MuJoCo plant, coordinate conventions, ball management
- **[Tuning Guide](sim_mpc/tuning.md)** — Parameters, weights, solver options, common scenarios

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
│       ├── state_machine.py
│       └── motion_bridge_node.py
├── sim/                        # MuJoCo simulation + MPC controller
│   ├── controller/             # CasADi NMPC (mpc.py, params.py)
│   ├── plant/                  # MuJoCo plant interface
│   ├── hand/                   # Hand coordination, catch/throw trajectories
│   ├── input/                  # Input adapters (scripted, interactive, keyboard)
│   ├── viz/                    # Telemetry logging, horizon renderer, dashboard
│   ├── model/                  # MJCF model + generator
│   └── main.py                 # Entry point + 50 Hz control loop
├── tools/                      # Standalone hardware test harnesses
├── docs/                       # This documentation
└── MOTION_PLANNER_PLAN.md      # Historical development plan & test log
```
