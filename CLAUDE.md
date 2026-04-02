# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Jugglebot is a real-time robotics control codebase (MPC planner, MuJoCo simulation, ODrive PID). Incorrect velocity, feedforward, or timing changes can cause dangerous jerky hardware movement. Always verify physics/control implications of changes, not just test passage.

Jugglebot is a Stewart platform robot that catches and throws balls. The codebase has three main subsystems that share a common config layer:

1. **ROS2 hardware stack** (`ros_ws/`) — runs on Jetson Orin Nano (Ubuntu 20.04, ROS2 Foxy, Python 3.8)
2. **MuJoCo simulation** (`sim/`) — standalone Python 3.11+, no ROS2 dependency
3. **MPC controller** (`controller/`) — pure Python with CasADi, imported by both sim and hardware

## Architecture

```
config/hardware_config.yaml  ← single source of truth for all physical parameters
config/generate_config.py    ← generates .py/.h/.js constants → config/generated/ + consumer dirs
controller/                  ← MPC runtime: solver, plant abstractions, telemetry, MPC loop, hardware plant
run_mpc.py                   ← hardware MPC entry point (uses controller/)
sim/                         ← MuJoCo simulation (plant/, hand/, ball/, input/, viz/)
ros_ws/src/jugglebot/        ← ROS2 package (can/, motion/, tracking/, nodes)
tests/                       ← all tests (ros/, sim/, motion/, hardware/, archived/)
tools/                       ← standalone utilities (tracking_analyzer)
logbook/                     ← engineering logbook (investigation entries, INDEX.md)
plans/active/                ← in-progress plans and implementation reports
plans/archived/              ← completed or superseded plans
```

**Engineering logbook & planning:**
- All code changes are logged in `logbook/` — see `logbook/README.md` for the full guide
- `/investigate` — hardware diagnosis-to-fix pipeline; `/log` — log non-hardware changes; `/logbook` — browse/search entries
- `/archive-plan` — move completed plans from `plans/active/` to `plans/archived/` (with critical review)
- Commits include `Logbook-Entry: <slug>` trailers for traceability from `git blame` to logbook entries

**Key architectural boundaries:**
- `ros_ws/.../motion/` and `controller/` are pure Python — no ROS2 imports allowed
- ROS2 nodes (`*_node.py`) are thin wrappers; business logic lives in pure-Python modules
- `controller/plant.py` defines `PlantInterface` — implemented by `MuJoCoPlant` (sim) and `HardwarePlant` (real robot via ZMQ IPC)
- IPC between processes uses ZeroMQ PUB/SUB on tcp://localhost:5556 (telemetry) and :5557 (commands), msgpack serialization
- `motor_guard.py` is the safety-critical 500 Hz control loop; MPC runs at 40 Hz in a separate process

## Environment

On the Jetson, always use the project virtualenv for all Python commands (tests, sim, scripts):
```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
```
The system `python3` (3.8.10) lacks MuJoCo and other project dependencies. The venv at `~/Desktop/PDJ_venv/venv/` has everything installed.

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

### Hardware MPC (on Jetson)
```bash
# Hold at active pose
python run_mpc.py --pose 0,0,170,0,0,0 --duration 10

# Production: receive targets from ROS2 via mpc_bridge_node
python run_mpc.py

# With live dashboard
python run_mpc.py --dashboard
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

## Workflow Rules

- **Grep before refactoring**: before renaming or refactoring any symbol, grep the entire codebase for all references first. List every file and line number, count total occurrences. After making changes, verify the count drops to zero. A partial find-and-replace is not acceptable.
- **Analyze control-system implications before changes**: before implementing changes to MPC, feedforward, or timing code, analyze the control-system implications first. What happens to the feedforward path? Could this cause discontinuities, oscillation, or timing issues at 40 Hz? Walk through one MPC cycle step-by-step with the proposed change.
- **TodoWrite checklist for multi-file tasks**: for tasks involving changes to multiple files, create a TodoWrite checklist before starting. List every file that needs changes, every test that needs updating, and a final verification step. Check off each item as you complete it. Do not declare the task done until all items are checked.
- **Run tests after code changes**: after any code changes, run the full test suite (`pytest tests/ -v`) and ensure all tests pass before considering the task complete. Report the test count and results.

## Critical Conventions

- **Python 3.8 compatibility** in `ros_ws/`: always use `from __future__ import annotations` for modern type hints
- **Config codegen**: after editing `hardware_config.yaml` or `protocol_config.yaml`, run `python config/generate_config.py` — never hand-edit generated files
- `tests/conftest.py` sets up shared paths; `tests/ros/conftest.py` injects mock ROS2 modules for Windows
- Stewart platform uses mixed mm/rad units; Jacobian is normalized by `plat_radius_mm` before numeric work (condition number ~3-8 at home, not the raw ~450)
- Jacobian convention: J maps `[vx,vy,vz,wx,wy,wz]` to `[q_dot_1..q_dot_6]`
- Force decomposition: `f = J^{-T} * W` (use `np.linalg.solve(J.T, W)`), NOT `J^T * W`
- All movements must use profiled trajectories — never command step position changes
- CAN encoding must match `can_node.py`: negate, scale by appropriate value (check protocol_config), int16, clamp
- Log files and temporary artifacts are in `temp/`, not `/tmp/`
