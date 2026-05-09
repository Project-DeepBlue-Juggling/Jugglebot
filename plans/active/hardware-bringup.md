---
title: MPC Hardware Bringup
created: 2026-03-28
status: active
---

# MPC Hardware Bringup Plan

## Context

The MPC-based motion planner has only ever run in simulation. The hardware has previously been controlled by a simple quintic path-following controller (`control_loop.py`, now archived). We're bringing the new MPC pipeline online for the first time:

- `motor_guard.py` — 500 Hz safety-critical interpolator (replaces `control_loop.py`)
- `mpc_bridge_node.py` — ROS2 bridge for MPC target commands
- `controller/mpc.py` — CasADi NMPC solver
- `controller/hardware_plant.py` — ZMQ plant connecting MPC to motor guard

These are wired into `jugglebot_launch.py` and `setup.py` but have **never been deployed to the Jetson**.

The hardware can destroy itself with high-jerk movements or movements past limits. Physical E-stop button is available.

### Command pipeline
```
MPC (50 Hz) → HardwarePlant (:5557) → MotorGuard (500 Hz) → MotionBridge → CAN → ODrives (8 kHz)
```

### Coordinate frame
- `--pose X,Y,Z,RX,RY,RZ` — Z is **STOW-relative** in mm
- **Active position = z=170mm** (NOT z=0)
- `--pose 0,0,170,0,0,0` = hold at Active (zero motion)
- `--pose 0,0,220,0,0,0` = 50mm above Active

---

## Phase 0: Pre-Deployment Verification (Desktop, No Jetson) — COMPLETE (2026-03-30)

### 0.1 Run all tests
```bash
pytest tests/ -v
```
**Go:** All tests pass. **Abort:** Any motor_guard, workspace, or conversion test failure.

### 0.2 Verify conversion factors offline
Manually check from `hardware_config.py`:
- Active in rev: `154.5mm × 0.01418 rev/mm ≈ 2.19 rev` — must match `JB_OP_ACTIVATE_POSITION_REVS`
- Stroke max in rev: `280mm × 0.01418 ≈ 3.97 rev` — must be < `LEG_MOTOR_MAX_POSITION_REVS = 4.2`
- Slew rate in mm/s: `9.5 rev/s / 0.01418 ≈ 670 mm/s` — must be < MPC velocity limit (700 mm/s)
- MAX_DEVIATION_REV = 0.5 rev ≈ 35mm — reasonable runaway threshold

**Go:** All values physically consistent. **Abort:** Any inconsistency.

### 0.3 Sim regression
```bash
cd sim
python main.py --mpc --pose 0,0,190,0,0,0 --no-viewer --duration 5
python main.py --mpc --pose 20,0,170,0,0,0 --no-viewer --duration 5
python main.py --mpc --pose 0,0,170,0.05,0,0 --no-viewer --duration 5
```
**Go:** Smooth convergence, no oscillation, extensions in [5,275]mm, solve time < 19ms.

---

## Phase 1: Jetson Deployment (No Motors) — COMPLETE (2026-03-30)

### 1.1 Install dependencies
```bash
pip3 install casadi pyzmq msgpack
python3 -c "import casadi; import zmq; import msgpack; print('OK')"
```

### 1.2 Build ROS2 workspace
```bash
cd ~/ros_ws && colcon build --packages-select jugglebot && source install/setup.bash
# Verify new executables exist:
which motor_guard && which mpc_bridge_node
```
If Python 3.8 syntax errors occur with `X | None` annotations: they should be safe due to `from __future__ import annotations`, but verify each new module imports cleanly:
```bash
python3 -c "from jugglebot.motion.motor_guard import MotorGuard; print('OK')"
python3 -c "from jugglebot.mpc_bridge_node import main; print('OK')"
```

### 1.3 Motor guard standalone smoke test
```bash
motor_guard --rate 500
# Should start, sit in DISABLED, print workspace limits. Ctrl-C after 10s.
```
**Go:** Starts cleanly, no crashes.

### 1.4 MPC solver timing on Jetson — PASS (2026-03-30)
```bash
cd ~/Desktop/Jugglebot/sim
PYTHONPATH=..:../ros_ws/src/jugglebot:../config/generated python3 -c "
from controller.mpc import MPCController
from controller.params import MPCParams
from plant.interface import PlantState
from jugglebot.motion.geometry import StewartGeometry
import jugglebot.hardware_config as hw
import numpy as np, time

geom = StewartGeometry()
params = MPCParams()
active_rev = np.array(hw.JB_OP_ACTIVATE_POSITION_REVS)
active_ext = active_rev / geom.mm_to_rev

mpc = MPCController(
    params=params,
    base_nodes=geom.base_nodes,
    plat_nodes=geom.plat_nodes,
    init_height_mm=geom.init_height_mm,
    init_leg_lengths_mm=geom.init_leg_lengths_mm,
    active_extensions_mm=active_ext,
)

state = PlantState(
    leg_extensions_mm=active_ext,
    leg_velocities_mmps=np.zeros(6),
    platform_pos_mm=np.array([0., 0., 170.]),
    platform_rot=np.zeros(3),
    platform_twist=np.zeros(6),
    time=0.0,
)
target = np.array([0., 0., 170., 0., 0., 0.])

t0 = time.perf_counter()
cmd, vel, diag = mpc.solve(state, target)
t1 = time.perf_counter()
print(f'Cold solve: {(t1-t0)*1000:.1f} ms')

times = []
for _ in range(10):
    t0 = time.perf_counter()
    cmd, vel, diag = mpc.solve(state, target)
    times.append((time.perf_counter()-t0)*1000)
print(f'Warm: mean={np.mean(times):.1f}ms, max={np.max(times):.1f}ms, p99={np.percentile(times,99):.1f}ms')
"
```

**Result:** Cold solve 8.7 ms, warm mean 2.1 ms, max 2.7 ms — well within 19 ms budget.

**Go:** Warm solve < 15ms mean, < 18ms max. **Abort:** > 18ms consistently.

### 1.5 Full launch IPC test (motors OFF / CAN disconnected) — PASS (2026-03-30)
```bash
ros2 launch jugglebot jugglebot_launch.py
# In another terminal:
ros2 node list  # should include mpc_bridge_node
ps aux | grep motor_guard  # should show the process
```
**Go:** All nodes start. No ZMQ port conflicts. Motor guard in DISABLED.

---

## Phase 2: Zero-Motion Hardware Test (Motors On, No Commanded Movement) — COMPLETE (2026-03-30)

**From this point: HAND ON PHYSICAL E-STOP AT ALL TIMES.**

### 2.1 Activate robot via proven path
```bash
# ROS2 launch running
ros2 topic pub --once /orchestrator_command std_msgs/msg/String "data: 'activate'"
```
Confirm normal activation (all 6 legs to Active ~2.19 rev). This uses the existing proven quintic path, NOT the MPC.

**Go:** All motors at Active position, healthy. **Abort:** Any motor fault or homing failure.

### 2.2 Verify motor guard receives feedback and enters SHELL cleanly
The `/motion/diagnostics` topic only publishes once the motor guard is ENABLED
**and** has received at least one MPC command.  In DISABLED mode, no telemetry
flows.  Instead, verify via motor guard logs in the launch terminal.

```bash
ros2 topic pub --once /orchestrator_command std_msgs/msg/String "data: 'shell'"
```

**What to expect in launch terminal:**
- `Motor guard DISABLED` then `Motor guard ENABLED (source=SHELL)`
- Periodic loop timing logs (mean ~2 ms, jitter < 1 ms)
- `cond#` and `ws=ok` — confirms motor feedback is flowing and FK is working
- **No E-stop** — the staleness watchdog only triggers after the motor guard
  has received its *first* MPC command (`_has_mpc_cmd` flag).  Before that,
  the guard sits ENABLED but sends no motor commands — safe by design.

**CRITICAL CHECK:** Motors must NOT move. If they do, hit physical E-stop.

**Go:** Motor guard ENABLED, loop timing healthy, cond# reasonable (~2-5), no motor movement.

### 2.4 MPC hold at Active position
```bash
# Clear the E-stop from 2.3 — deactivate and re-activate:
ros2 topic pub --once /orchestrator_command std_msgs/msg/String "data: 'deactivate'"
# Wait for legs to return to STOW
ros2 topic pub --once /orchestrator_command std_msgs/msg/String "data: 'activate'"
# Wait for activation complete

# Start MPC holding at Active (z=170mm = current position)
cd ~/Jugglebot/sim
python3 main.py --hardware --mpc --pose 0,0,170,0,0,0 --duration 300
```

The startup sequence:
1. `HardwarePlant.__init__()` binds PUB on :5557, connects SUB on :5556
2. `MPCController.__init__()` builds CasADi NLP (takes a few seconds)
3. `plant.enable()` sends enable to motor guard (starts 200ms staleness clock)
4. 50ms sleep
5. First MPC solve + command (must arrive within ~150ms of enable)

**What to expect:**
- Robot holds perfectly still at Active position
- Tracking error < 2mm on all legs
- MPC solve time < 15ms
- Motor currents steady (gravity compensation only)

**Monitor:**
- `ros2 topic echo /motion/tracking_error` — should be near zero
- `ros2 topic echo /motion/diagnostics` — workspace_status: ok
- Console output from `main.py` shows solve times

**What to look for:**
- ANY movement = extension convention mismatch. The slew limiter prevents a step, but you'll see slow drift.
- Oscillation in motor currents = ODrive PID fighting MPC commands
- Tracking error growing = integration drift in motor guard

**Abort:** Any visible movement. Tracking error > 5mm. Motor guard E-stop.

**Go:** 60+ seconds of perfect stillness. Tracking error < 2mm. Solve time < 15ms.

**Result (2026-03-30):** PASS — zero platform movement, tracking error ≤0.04mm, MPC staleness E-stop fired correctly on exit.

---

## Phase 3: Tiny Motions (5mm Scale) — COMPLETE (2026-03-30)

### 3.1 Z+5mm
```bash
python3 main.py --hardware --mpc --pose 0,0,175,0,0,0 --duration 30
```
**Expect:** Platform rises ~5mm smoothly. Peak leg velocity < 50mm/s. Settles in < 1s.

### 3.2 Return to Active
```bash
python3 main.py --hardware --mpc --pose 0,0,170,0,0,0 --duration 30
```

### 3.3 X+5mm (lateral — tests differential leg motion)
```bash
python3 main.py --hardware --mpc --pose 5,0,170,0,0,0 --duration 30
```

### 3.4 Tiny tilt (0.02 rad ≈ 1.1°)
```bash
python3 main.py --hardware --mpc --pose 0,0,170,0.02,0,0 --duration 30
```

**Go for all:** Smooth motion. No overshoot > 1mm. Settles within 1s. No oscillation.

**Result (2026-03-30):** PASS — controlled platform movement, no overshoot > 1 mm, MPC staleness E-stop fired correctly on exit.

---

## Phase 4: Moderate Motions (20-50mm Scale)

### 4.1 Z+20mm
```bash
python3 main.py --hardware --mpc --pose 0,0,190,0,0,0 --duration 30
```

### 4.2 Z+50mm
```bash
python3 main.py --hardware --mpc --pose 0,0,220,0,0,0 --duration 30
```

### 4.3 Combined translation + tilt
```bash
python3 main.py --hardware --mpc --pose 20,20,190,0.05,0.05,0 --duration 30
```
Monitor condition number — should stay well below soft limit.

### 4.4 Sequence of poses
```bash
python3 main.py --hardware --mpc --sequence "0,0,190,0,0,0@0.5 0,0,220,0,0,0@3 0,0,170,0,0,0@6" --duration 15
```
Tests multi-target transitions.

**Go:** All smooth. Peak velocity < 500mm/s. Tracking error < 5mm during motion.

---

## Phase 5: Tau Calibration

### 5.1 Record step response
```bash
python3 main.py --hardware --mpc --pose 0,0,220,0,0,0 --duration 15
# CSV saved to temp/logs/mpc_YYYYMMDD_HHMMSS.csv
```

### 5.2 Estimate tau from CSV
The CSV contains `cmd_ext_0..5` (MPC commands) and `actual_ext_0..5` (motor feedback). For each leg:
```
tau ≈ dt / median((actual[k+1] - actual[k]) / (cmd[k] - actual[k]))
```
Filter to samples where `|cmd - actual| > 0.5mm` (ignore steady state).

Use existing analysis tools if available:
```bash
python3 -m analysis.compare analysis/baselines/T1_baseline.csv logs/mpc_LATEST.csv --estimate-tau
```

### 5.3 Update tau if needed
If measured tau differs from 30ms by > 20%, update `controller/params.py`:
```python
tau: float = 0.027  # calibrated from hardware (was 0.03)
```
Then re-run Phase 4 tests and compare tracking improvement.

**Go:** Tau calibrated. Step response overshoot < 5%.

---

## Phase 6: Dynamic Trajectories

### Pre-requisite: Raise velocity limits for dynamic motion

Before running trajectories faster than ~280 mm/s, raise both limits:

1. **MPC velocity limit** — change `max_leg_vel_mmps` in `controller/params.py` from 280 to 700
2. **ODrive velocity limit** — publish `SetMotorVelCurrLimitsMessage` with `legs_vel_limit=10.0` via `motion_bridge_node` (or add this to the MPC bridge startup sequence)

Until these are raised, all trajectories are capped at ~280 mm/s leg speed. T1 and slow T2 should work fine at this limit; T4 (fast transit) will not.

### 6.1 T1 trajectory (gentle Z sinusoid)
```bash
python3 main.py --hardware --mpc --trajectory T1 --duration 30
```
**Expect:** Tracking error < 5mm. Sim baseline is ~0.48mm, so 10x margin is generous.

### 6.2 T2 trajectory (circular orbit)
```bash
python3 main.py --hardware --mpc --trajectory T2 --duration 30
```
**Expect:** Tracking error < 10mm. Sustained multi-axis motion.

### 6.3 T3 trajectory (multi-axis translation + tilt)
```bash
python3 main.py --hardware --mpc --trajectory T3 --duration 30
```

### 6.4 T4 trajectory (fast transit — most aggressive)
```bash
python3 main.py --hardware --mpc --trajectory T4 --duration 15
```
**Expect:** Tracking error < 10mm. Peak velocity approaching 400-500mm/s.

**Go:** All trajectories execute cleanly. No E-stops. Tracking within 10× sim baseline.

---

## Phase 7: Production Path & Catch Readiness

### 7.1 Full production path (ZMQ targets from mpc_bridge)
```bash
# Terminal 1: ROS2 launch
ros2 launch jugglebot jugglebot_launch.py

# Terminal 2: MPC process (receives targets from bridge)
cd ~/Jugglebot/sim
python3 main.py --hardware --mpc --duration 3600

# Terminal 3: Activate → SHELL mode
ros2 topic pub --once /orchestrator_command std_msgs/msg/String "data: 'activate'"
# Wait for activation
ros2 topic pub --once /orchestrator_command std_msgs/msg/String "data: 'shell'"
```
Send small pose commands via ROS2 topics through the mpc_bridge_node.

### 7.2 Mode transitions
Test SHELL → SPACEMOUSE → SHELL, SHELL → deactivate → activate → SHELL.
Verify clean transitions without motor jerks.

### 7.3 Catch trajectories (simulated ball events)
```bash
python3 main.py --hardware --mpc --catch DT1 --duration 30
```
Start with slowest catch scenario. Progressively try DT2→DT5.

### 7.4 Ball Butler integration
Only after all prior phases pass — real ball throws from Ball Butler.

---

## Critical Files

| File | Role |
|------|------|
| `ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py` | 500 Hz safety loop — all safety checks |
| `controller/hardware_plant.py` | MPC ↔ motor guard ZMQ bridge |
| `controller/params.py` | MPC tuning (tau, velocity limits, horizon) |
| `sim/main.py` | Entry point for `--hardware --mpc` mode |
| `ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py` | ROS2 ↔ motor guard IPC bridge |
| `ros_ws/src/jugglebot/jugglebot/mpc_bridge_node.py` | ROS2 target commands → MPC |
| `ros_ws/src/jugglebot/jugglebot/motion/ipc.py` | ZMQ message protocol |

## Safety Parameters Reference

| Parameter | Value | Consequence if wrong |
|-----------|-------|---------------------|
| Active Z | 170mm (STOW-relative) | Wrong hold position → unexpected movement |
| mm_to_rev | ~0.01418 per leg | All position commands wrong |
| tau | 30ms (default) | Oscillation (too low) or sluggishness (too high) |
| Stroke limits | [5, 275]mm hard | Physical damage if exceeded |
| MPC vel limit | 280 mm/s (bringup) / 700 mm/s (production) | Motor/cable damage if exceeded |
| ODrive vel limit | 4.0 rps (bringup) / 10.0 rps (production) | Extra hardware speed cap |
| Slew rate | 9.5 rev/s | Excess jerk if too high |
| MAX_DEVIATION | 0.5 rev (~35mm) | E-stop if cmd diverges from actual |
| MPC staleness | 200ms | E-stop if MPC stops sending |
| Motor FB staleness | 150ms | Commands suppressed |

## Verification

After each phase, check:
1. No motor guard E-stops (unless expected, as in Step 2.3)
2. Tracking error within phase-specific thresholds
3. Condition number below soft limit (1.5× home, ~5-12 with normalized Jacobian)
4. Motor currents reasonable (< 5A during normal operation)
5. CSV telemetry log saved and reviewable
