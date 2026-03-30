# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Jugglebot is a Stewart platform robot that catches and throws balls. The codebase has three main subsystems that share a common config layer:

1. **ROS2 hardware stack** (`ros_ws/`) — runs on Jetson Orin Nano (Ubuntu 20.04, ROS2 Foxy, Python 3.8)
2. **MuJoCo simulation** (`sim/`) — standalone Python 3.11+, no ROS2 dependency
3. **MPC controller** (`controller/`) — pure Python with CasADi, imported by both sim and hardware

## Architecture

```
config/hardware_config.yaml  ← single source of truth for all physical parameters
config/generate_config.py    ← generates .py/.h/.js constants → config/generated/ + consumer dirs
controller/                  ← CasADi NMPC solver, shared by sim and hardware
sim/                         ← MuJoCo simulation (plant/, hand/, ball/, input/, viz/)
ros_ws/src/jugglebot/        ← ROS2 package (can/, motion/, tracking/, nodes)
tests/                       ← all tests (ros/, sim/, motion/, hardware/, archived/)
tools/                       ← standalone utilities (tracking_analyzer)
```

**Key architectural boundaries:**
- `ros_ws/.../motion/` and `controller/` are pure Python — no ROS2 imports allowed
- ROS2 nodes (`*_node.py`) are thin wrappers; business logic lives in pure-Python modules
- `sim/plant/interface.py` defines `PlantInterface` — implemented by `MuJoCoPlant` (sim) and `HardwarePlant` (real robot via ZMQ IPC)
- IPC between processes uses ZeroMQ PUB/SUB on tcp://localhost:5556 (telemetry) and :5557 (commands), msgpack serialization
- `motor_guard.py` is the safety-critical 500 Hz control loop; MPC runs at 50 Hz in a separate thread

## Commands

### Config generation (run from repo root)
```bash
python config/generate_config.py
```

### Simulation
```bash
# Basic
python sim/main.py
python sim/main.py --mpc --pose 0,0,50,0,0,0
python sim/main.py --keyboard --mpc
python sim/main.py --no-viewer --duration 5

# With dashboard (http://localhost:8082)
python sim/main.py --dashboard --mpc --pose 0,0,50,0,0,0

# Docker (GPU required)
cd sim && docker compose up
```

### Tests
```bash
# All automated tests (from repo root)
pytest tests/ -v

# By category
pytest tests/ros/ -v                          # ROS2 node tests (conftest mocks ROS2)
pytest tests/sim/ -v                          # MPC + simulation tests
pytest tests/motion/ -v                       # motion module tests (kinematics, dynamics, motor_guard)
pytest tests/ros/test_can_node.py -v          # single file
pytest tests/sim/test_mpc_static.py -v        # single file

# Hardware test harnesses (standalone scripts, require real robot)
python tests/hardware/free_platform_test.py --test all
python tests/hardware/single_leg_test.py --test all
```

### ROS2 (on Jetson)
```bash
cd ros_ws && colcon build --packages-select jugglebot
source install/setup.bash
ros2 launch jugglebot jugglebot_launch.py
```

### Documentation
```bash
mkdocs serve   # local preview at http://localhost:8000
```

## Critical Conventions

- **Python 3.8 compatibility** in `ros_ws/`: always use `from __future__ import annotations` for modern type hints
- **Config codegen**: after editing `hardware_config.yaml` or `protocol_config.yaml`, run `python config/generate_config.py` — never hand-edit generated files
- `tests/conftest.py` sets up shared paths; `tests/ros/conftest.py` injects mock ROS2 modules for Windows
- Stewart platform uses mixed mm/rad units throughout; condition numbers are naturally high (~450-650) due to this
- Jacobian convention: J maps `[vx,vy,vz,wx,wy,wz]` to `[q_dot_1..q_dot_6]`
- Force decomposition: `f = J^{-T} * W` (use `np.linalg.solve(J.T, W)`), NOT `J^T * W`
- All movements must use profiled trajectories — never command step position changes
- CAN encoding must match `can_node.py`: negate, scale by appropriate value (check protocol_config), int16, clamp
