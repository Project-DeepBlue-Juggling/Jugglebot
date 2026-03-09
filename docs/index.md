# Jugglebot Documentation

Jugglebot is a 6-DoF Stewart platform robot with ball-catching capabilities. It uses string-driven linear actuators controlled by ODrive motor controllers, with a Jetson running ROS2 for high-level coordination and a standalone Python process for real-time motion control.

## Documentation Sections

### [Motion Planner](motion_planner/index.md)

Complete technical documentation for the motion planning and control subsystem, covering:

- **[Architecture](motion_planner/architecture.md)** — System design, data flow, and control philosophy
- **[Kinematics](motion_planner/kinematics.md)** — Stewart platform geometry, inverse/forward kinematics, Jacobian
- **[Dynamics](motion_planner/dynamics.md)** — Gravity compensation, inertia feedforward, force decomposition
- **[Trajectory Planning](motion_planner/trajectory.md)** — Quintic polynomial generation, feasibility checking, dynamic targets
- **[Workspace Safety](motion_planner/workspace.md)** — Limit enforcement, singularity avoidance, fault detection
- **[Control Loop](motion_planner/control_loop.md)** — The 500 Hz control process, operating modes, IPC layer
- **[System Integration](motion_planner/integration.md)** — ROS2 bridge, CAN interface, full system startup
- **[Operations Guide](motion_planner/operations.md)** — Running tests, tuning gains, extending the system
- **[Validation Results](motion_planner/results.md)** — Summary of hardware test outcomes

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
├── tools/                      # Standalone hardware test harnesses
├── docs/                       # This documentation
└── MOTION_PLANNER_PLAN.md      # Historical development plan & test log
```
