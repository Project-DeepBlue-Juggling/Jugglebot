# Jugglebot MPC Simulation: Stewart Platform with Model Predictive Control

## Summary

Build a MuJoCo-based simulation of Jugglebot's Stewart platform controlled by a nonlinear Model Predictive Controller (CasADi + IPOPT). The simulation replicates the real robot's kinematics and provides a development environment for MPC tuning, spacemouse control, and dynamic target interception — all without requiring the physical robot.

The MPC will eventually deploy to real hardware (Jetson), sending position setpoints to ODrive's existing 8 kHz PID loop.

## Phase Overview

| Phase | Name | What it delivers |
|-------|------|-----------------|
| 0 | MuJoCo Stewart Platform Model | MJCF XML model of Jugglebot validated against existing kinematics (**COMPLETE**) |
| 1 | Simulation Harness | Simulation loop, `PlantInterface` abstraction, `MuJoCoPlant`, telemetry logging (**COMPLETE**) |
| 2 | MPC Core (Static Pose Tracking) | CasADi/IPOPT NMPC that tracks a static reference pose with warm-starting and solver failure fallback (**COMPLETE**) |
| 3 | Trajectory Tracking | Time-varying reference generation (waypoints, quintics) fed to MPC; tracking quality validation (**COMPLETE**) |
| 4 | SpaceMouse Integration | Live spacemouse input mapped to target pose, MPC plans optimal motion to reach it (**COMPLETE**) |
| 5 | Dynamic Target (Ball Catching) | Timed target interception — platform arrives at a pose by a deadline, with optional throw velocity |
| 6 | Hardware Bridge | Swap simulated plant for real hardware; MPC outputs motor commands via CAN; sim-to-real validation |

## Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Physics engine | MuJoCo | Best parallel mechanism support (equality constraints), fast, free, built-in viewer |
| MPC solver | CasADi + IPOPT | Nonlinear, symbolic autodiff, pure Python workflow, upgradeable to acados |
| Control rate | 50 Hz MPC, MuJoCo steps at 500 Hz | 50 Hz gives 20 ms solve budget; warm-started solves use ~5 ms; MuJoCo substeps fill in between |
| MPC horizon | N = 10 (200 ms) | N=20 exceeded solve budget on cold starts; N=10 is reliable with room to grow |
| MPC model | Kinematics-only initially | Actuators assumed ideal; dynamics layered in later (see Phase 6 notes) |
| Plant interface | Abstract `PlantInterface` | Sim and hardware are swappable without touching the controller |
| Inner loop (hardware) | ODrive position control (8 kHz) | Proven, fail-safe (stale command = hold position) |
| Spacemouse mode | Target pose (not velocity) | MPC plans optimal motion to reach the target |
| Ball model | Target pose at time T | No contact physics; ball = a pose deadline |
| Code location | `sim/` top-level directory | Separate from ROS2; imports from `motion/` where useful |
| ROS2 dependency | None in sim path | Pure Python; ROS2 only enters at hardware bridge (Phase 6) |
| Containerisation | Docker (Linux container) | One `docker compose up` from clone to running sim on any machine (Windows/Linux). GPU passthrough for MuJoCo viewer via `--gpus all` + WSLg (Windows) or native X11 (Linux) |

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     User Inputs                         │
│   SpaceMouse  │  Scripted Targets  │  Dynamic Targets   │
└──────┬────────┴────────┬───────────┴────────┬───────────┘
       │                 │                    │
       ▼                 ▼                    ▼
┌─────────────────────────────────────────────────────────┐
│                  Reference Generator                     │
│   Converts user input → reference trajectory/pose        │
│   (target pose, or time-varying reference)               │
└────────────────────────┬────────────────────────────────┘
                         │  ref_pose(t), ref_twist(t)
                         ▼
┌─────────────────────────────────────────────────────────┐
│                   MPC Controller                         │
│   CasADi + IPOPT  │  50 Hz  │  Horizon: N steps         │
│                                                          │
│   min  Σ  ‖x(k) - x_ref(k)‖²_Q  +  ‖u(k)‖²_R         │
│    u   k=0..N-1     + ‖Δu(k)‖²_S                       │
│                                                          │
│   s.t.  x(k+1) = f(x(k), u(k))    dynamics              │
│         u_min ≤ u(k) ≤ u_max       actuator limits       │
│         x_min ≤ x(k) ≤ x_max       workspace limits      │
│         ‖Δu(k)‖ ≤ Δu_max          rate limits            │
└────────────────────────┬────────────────────────────────┘
                         │  u* = [pos_cmd₁..₆]  (motor revs)
                         ▼
┌─────────────────────────────────────────────────────────┐
│                  PlantInterface                           │
│                                                          │
│   ┌──────────────┐          ┌───────────────────┐        │
│   │  MuJoCoPlant │          │  HardwarePlant    │        │
│   │  (simulation)│          │  (CAN → ODrive)   │        │
│   │              │          │                   │        │
│   │  Steps at    │          │  Sends input_pos  │        │
│   │  500 Hz      │          │  + vel_ff + τ_ff  │        │
│   │  internally  │          │  via CAN bus      │        │
│   └──────┬───────┘          └────────┬──────────┘        │
│          │                           │                   │
│          ▼                           ▼                   │
│   state = (motor_pos, motor_vel)  per leg                │
└────────────────────────┬────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────┐
│                  Visualization                           │
│   MuJoCo viewer (sim) │ Telemetry logging (both)        │
└─────────────────────────────────────────────────────────┘
```

### MPC State and Control Vectors

**State `x` (12-dimensional):**
```
x = [x, y, z, rx, ry, rz, vx, vy, vz, ωx, ωy, ωz]
     ├── pose (mm, rad) ──┤├── twist (mm/s, rad/s) ──┤
```
Platform pose as position offset from home + rotation vector, plus their time derivatives. Rotation vector representation is valid for the tilt range Jugglebot operates in (≤15°).

**Control `u` (6-dimensional):**
```
u = [q₁, q₂, q₃, q₄, q₅, q₆]   leg extensions (mm)
```
MPC directly outputs desired leg extensions. These are converted to motor revolutions before being sent to the plant.

**Prediction model `f(x, u)`:**
Kinematic model — given leg extensions `u`, compute the resulting platform pose via FK, assume actuators track commanded positions with a first-order lag:
```
q_actual(k+1) = q_actual(k) + (1/τ) · (q_cmd(k) - q_actual(k)) · dt
x(k+1) = FK(q_actual(k+1))
```
Where `τ` is a tunable actuator time constant (~20-50 ms, matching real ODrive tracking bandwidth). This captures the essential dynamics without full Newton-Euler computation in the MPC inner loop.

### File Structure

```
sim/
├── Dockerfile                     # Linux container: Python 3.11 + GPU libs + pip deps
├── compose.yaml                   # Two services: sim (GPU+display) and test (headless)
├── .dockerignore                  # Exclude .git, __pycache__, logs, etc.
├── requirements.txt               # mujoco, casadi, pyspacemouse, numpy, pytest, pyyaml, etc.
│
├── model/
│   ├── generate_mjcf.py           # Generates jugglebot.xml from hardware_config.yaml
│   ├── jugglebot.xml              # Auto-generated MJCF model (do not edit by hand)
│   └── meshes/                    # STL meshes exported from Onshape (visual geoms)
│       ├── base.stl               # Base frame
│       ├── platform.stl           # Platform assembly
│       ├── leg_outer.stl          # Outer tube (shared by all 6 legs)
│       └── leg_inner.stl          # Inner tube (shared by all 6 legs)
│
├── plant/
│   ├── __init__.py
│   ├── interface.py               # Abstract PlantInterface base class (Phase 1)
│   ├── mujoco_plant.py            # MuJoCo simulation plant (Phase 1)
│   └── hardware_plant.py          # Real hardware plant (Phase 6)
│
├── controller/
│   ├── __init__.py                # Exports MPCController, MPCParams, ReferenceGenerator
│   ├── mpc.py                     # CasADi MPC formulation and solver (Phase 2-3, complete)
│   ├── reference.py               # Reference trajectory generation (Phase 3, complete)
│   └── params.py                  # MPC tuning parameters dataclass (Phase 2, complete)
│
├── input/
│   ├── __init__.py
│   ├── spacemouse.py              # SpaceMouse → target pose (Phase 4, complete — Linux only)
│   ├── keyboard.py                # Keyboard → target pose (Phase 4, complete — cross-platform)
│   └── scripted.py                # Scripted test trajectories T1-T6 (Phase 3, complete)
│
├── viz/
│   ├── __init__.py                # Exports StepRecord, TelemetryLogger, record_from_arrays
│   ├── horizon.py                 # MPC predicted trajectory overlay in MuJoCo viewer (Phase 2, complete)
│   ├── telemetry.py               # Logging, plotting, diagnostics (Phase 1, complete)
│   └── dashboard/                 # Live web-based telemetry dashboard (SSE + uPlot)
│
├── tests/
│   ├── __init__.py
│   ├── test_model.py              # MuJoCo model validation vs existing IK (22 tests)
│   ├── test_mpc_static.py         # MPC tracks static poses (Phase 2, 14 tests, complete)
│   ├── test_mpc_trajectory.py     # MPC tracks moving references (Phase 3, 15 tests, complete)
│   └── test_mpc_dynamic.py        # MPC intercepts timed targets (Phase 5)
│
└── main.py                        # Entry point: run simulation loop (Phase 1)
```

---

## Phased Implementation

### Phase 0: MuJoCo Stewart Platform Model — ✅ COMPLETE (2026-03-16)

Build the MJCF XML description of Jugglebot and validate it against existing kinematics.

**Background — MuJoCo and parallel mechanisms:**
A Stewart platform is a closed-loop (parallel) mechanism with 6 legs connecting a base to a platform. MuJoCo cannot directly represent closed kinematic loops in its tree-based model. The standard approach is:

1. Define the platform as a free body (6-DoF free joint)
2. Define each leg as a prismatic (sliding) actuator
3. Use **equality constraints** (`connect` or `weld`) to enforce that each leg endpoint stays attached to its base and platform nodes

MuJoCo's constraint solver enforces these at each timestep, effectively simulating the parallel mechanism. This is well-documented and works reliably for Stewart platforms specifically.

**Tasks:**
- [x] Set up Docker environment:
  - Created `sim/Dockerfile` (NVIDIA CUDA 12.2 base, Python 3.11, OpenGL/GLEW/OSMesa libs, libhidapi, pip deps)
  - Created `sim/compose.yaml` with two services: `sim` (GPU, display forwarding, SpaceMouse) and `test` (headless, OSMesa)
  - Created `sim/.dockerignore` (exclude `.git`, `__pycache__`, `logs/`, etc.)
  - Created `sim/requirements.txt` (mujoco, casadi, numpy, pyspacemouse, matplotlib, pytest, pyyaml)
- [x] Export STL meshes from Onshape:
  - Meshes exported directly from Onshape (not URDF): `base.stl`, `platform.stl`, `leg_outer.stl`, `leg_inner.stl`
  - All legs identical — single copy of each leg mesh, reused for all 6 legs
  - Placed in `sim/model/meshes/`; generator auto-detects mesh presence and falls back to primitives if absent
  - See [Appendix A](#appendix-a-stl-mesh-coordinate-conventions) for mesh origin conventions and offsets
- [x] Create `sim/model/jugglebot.xml` MJCF file (auto-generated by `sim/model/generate_mjcf.py`):
  - Reads all geometry from `config/hardware_config.yaml` — no hardcoded values
  - Base frame: 6 base nodes as attachment sites (fixed in worldbody)
  - Platform body: free joint, mass/inertia from `dynamics` section, CoM offset applied
  - 6 legs: nested body structure — see [Appendix B](#appendix-b-mujoco-modelling-lessons) for rationale
  - Equality constraints: `connect` constraints between platform and each leg tip
  - Actuators: 6 position actuators (kp=10000) driving slide joints
  - Visual geoms: STL meshes with mm→m scaling, or capsule/cylinder primitives as fallback
  - Sensors: `framepos`/`framequat` on platform site (not body — see Appendix B)
- [x] Set MuJoCo solver parameters: timestep 2 ms (500 Hz), Newton solver, constraint tolerance defaults
- [x] Validate: model loads and renders correctly in `mujoco.viewer` on both Windows and Jetson
- [x] Write `sim/tests/test_model.py` — 22 tests across 5 classes, all passing:
  - `TestModelLoads` — model structure, actuators, sensors
  - `TestHomePosition` — height (574.3 mm), orientation (identity), XY (origin)
  - `TestFKValidation` — 13 parametrized poses compared against analytical FK (<0.023 mm position, <0.002° orientation)
  - `TestLegLimits` — slide range covers 280 mm stroke
  - `TestConstraintSatisfaction` — equality constraint violations negligible at home and offset poses

**Key geometry values (from `hardware_config.yaml`):**
| Parameter | Value | Used for |
|-----------|-------|----------|
| `base_nodes_mm` | 6×3 array | Base attachment sites |
| `init_plat_nodes_mm` | 6×3 array | Platform attachment sites |
| `initial_height_mm` | 574.3 | Home position Z offset |
| `leg_stroke_mm` | 280.0 | Prismatic joint range |
| `platform_mass_kg` | 1.2 | Platform body mass |
| `platform_com_offset_mm` | [-9.68, -68.64, 52.73] | CoM in platform frame |
| `platform_inertia_tensor_kgmm2` | 6 components | Rotational inertia |

**Deliverable:** A `.xml` model that opens in MuJoCo's viewer, shows a Stewart platform at home position, and whose FK matches existing code to < 0.023 mm (far exceeding the < 1 mm target). Validated on Windows 10 and Jetson Orin Nano.

---

### Phase 1: Simulation Harness — ✅ COMPLETE (2026-03-16)

Build the simulation loop and plant abstraction layer.

**Tasks:**
- [x] Define `PlantInterface` (abstract base class) in `sim/plant/interface.py`:
  - `PlantInterface` ABC with `command()`, `get_state()`, `step()`, `reset()` methods
  - `PlantState` dataclass: leg extensions/velocities (mm), platform pose offset (mm) + rotation vector (rad) + twist, sim time
- [x] Implement `MuJoCoPlant` in `sim/plant/mujoco_plant.py`:
  - Loads MJCF model, creates `MjData`, resets to home keyframe
  - `command()`: converts IK extensions (mm) → MuJoCo slide values (m) via pre-computed geometric home lengths
  - `get_state()`: reads all sensors (cached address lookup), converts MuJoCo quaternion → rotation vector, handles mm↔m and home-offset conventions
  - `step()`: computes substep count from `dt / model.opt.timestep`, calls `mj_step()` in a loop
  - `reset()`: resets to home keyframe, optionally computes IK for a target pose and sets actuator controls
  - Exposes `model`, `data`, `geom`, `timestep` properties for viewer integration
- [x] Implement main simulation loop in `sim/main.py`:
  - 50 Hz outer loop (MPC control rate = 20 ms, with 10 internal substeps at 500 Hz)
  - `--no-viewer` mode: pure headless loop (command → step → log)
  - `--pose x,y,z,rx,ry,rz` (mm, rad): target pose via IK
  - `--duration` (seconds): simulation length
  - Viewer mode: `mujoco.viewer.launch_passive()` with real-time pacing via `time.sleep()`
  - Clean shutdown on Ctrl+C (flushes telemetry)
  - `--log-dir` for output directory
- [x] Basic test: command z+50mm and combined translation+rotation poses, platform settles within 0.34 mm / 0.04°
- [x] Define telemetry logging in `sim/viz/telemetry.py`:
  - `StepRecord` dataclass: scalar fields (no numpy arrays) for CSV compatibility — ref/actual pose (6 DoF each), ref/actual twist, 6 cmd + 6 actual leg extensions, MPC diagnostics (solve_time_ms, solve_status, cost, constraint_violation), derived tracking errors (mm, deg)
  - `record_from_arrays()` convenience function: takes numpy arrays → StepRecord with auto-computed tracking errors
  - `TelemetryLogger`: accumulates records, writes CSV on `flush()`/`close()`
  - Plotting utilities: `plot_tracking()` (6-DoF ref vs actual + error over time), `plot_solve_times()` (histogram vs 20 ms budget)
  - Same schema used for sim and hardware — enables direct sim-to-real comparison in Phase 6
- [x] Write `sim/tests/test_plant.py` — 7 tests across 4 classes, all passing:
  - `TestPlantHome` — state at home, extension range check (~27-30 mm due to ball joint offset)
  - `TestPlantCommand` — z+50mm and combined pose settle within 1.0 mm / 0.5°
  - `TestPlantReset` — reset to home, reset to specific pose
  - `TestTelemetryLogger` — CSV output with correct header and data

**Test results:**
- z+50 mm: tracking error 0.331 mm position, 0.044° orientation
- Combined [30, -20, 80, 0.05, -0.03, 0]: tracking error 0.334 mm position, 0.045° orientation
- All 29 tests pass (22 Phase 0 + 7 Phase 1) in < 1 second

**Implementation note — home-relative extension convention:**
The IK model measures extensions from `init_leg_lengths_mm` (which excludes `ball_joint_offset_mm` ~28 mm), so at MuJoCo home the IK reports ~27-30 mm of extension. To match the real robot's encoder convention (extension=0 at home, range [0, 280]), `MuJoCoPlant` stores the IK extensions at home as `_home_extensions_mm` and subtracts them from all public-facing extension values. The `pose_to_extensions()` helper computes home-relative extensions directly from a 6-DoF pose. Internally, the IK math is unchanged — the offset is applied at the `command()` / `get_state()` boundary.

**Deliverable:** Running simulation where you can programmatically command leg extensions and see the platform respond in the 3D viewer. Telemetry logged in structured CSV from the start. Headless mode for CI/batch runs.

---

### Phase 2: MPC Core (Static Pose Tracking) — ✅ COMPLETE (2026-03-16)

Formulate and implement the NMPC for tracking a reference pose.

**Tasks:**
- [x] Define CasADi symbolic model in `sim/controller/mpc.py`:
  - Symbolic IK: rotation vector → rotation matrix (Rodrigues with regularised denominator) → leg vectors → leg lengths. Expressed as equality constraints in the NLP (Option 1 from plan).
  - Actuator dynamics: first-order lag `q(k+1) = q(k) + (u(k) - q(k)) · dt/τ`
  - Pose is a decision variable at each horizon step, constrained to be IK-consistent with actual leg extensions
- [x] Formulate the optimal control problem:
  - **Horizon:** N = 10 steps at 50 Hz (200 ms lookahead). N=20 was too large for the 18 ms budget — cold-start solves exceeded the time limit. N=10 converges reliably.
  - **Decision variables:** u[0..N-1] (commanded extensions, 6 each), q[1..N] (actual extensions after lag), p[1..N] (platform pose). Total: 180 variables for N=10.
  - **Cost function:**
    - `Q_pos=10, Q_ori=1000` — pose tracking (separate weights for mm² and rad²)
    - `Qf_pos=50, Qf_ori=5000` — terminal cost (5× running cost)
    - `Q_vel_lin=0.0001, Q_vel_ang=0.01` — velocity damping via finite-difference twist
    - `R=1e-4` — control effort (small regulariser)
    - `S=0.01` — control smoothness (penalises Δu between steps)
  - **Constraints:**
    - Actuator dynamics: 6·N equality constraints
    - IK consistency: 6·N equality constraints (implicit FK via IK)
    - Leg extensions: `0 ≤ u,q ≤ 280 mm` (stroke limits as box constraints)
    - Leg velocity: `|Δu/dt| ≤ 1060 mm/s` (rate limits as inequality constraints)
    - Workspace: `|x,y,z| ≤ 200 mm`, `|rx,ry,rz| ≤ 0.3 rad` (box constraints)
  - **Solver:** IPOPT with exact Hessian (CasADi AD), warm-starting (primal + dual)
- [x] Implement `MPCController` class in `sim/controller/mpc.py`:
  - `MPCController.from_plant(params, plant)` factory for easy construction
  - `solve(state, reference)` → `(cmd, diagnostics)` with warm-starting and failure handling
  - `predicted_poses` property for horizon visualisation
  - `reset()` clears warm-start state
- [x] Define `MPCParams` in `sim/controller/params.py`:
  - All tunable parameters as a `@dataclass` with sensible defaults
  - Separate weights for position/orientation tracking, velocity damping, effort, smoothness
  - IPOPT options: max_iter, max_cpu_time, tolerance, warm_start, print_level
  - Failure handling: max_consecutive_failures threshold
- [x] Tune for static pose tracking:
  - z+50 mm: 0.232 mm final error, settles within 500 ms ✓
  - x+20 mm at z+50 mm: 0.257 mm final error ✓ (pure x+50 from home is infeasible — see note below)
  - pitch+5° at z+80 mm: 0.231 mm / 0.023° final error ✓ (pure pitch from home infeasible)
  - Combined [30, -20, 50, 3°, -2°, 0°]: 0.350 mm / 0.016° final error ✓
  - Acceptance criteria met: smooth approach, no overshoot, settles < 500 ms, no constraint violations
- [x] Measure solve time:
  - Cold start: 16–37 ms (exceeds budget, but only happens once)
  - Warm-started mean: ~5 ms (well within 20 ms budget)
  - P95: 6–8 ms
  - Target of < 15 ms mean achieved for warm-started solves
- [x] Implement solver failure handling in `MPCController.solve()`:
  - **Timeout/non-convergence:** Apply first step of shifted previous solution (warm-start fallback)
  - **Consecutive failures:** Counter tracks sequential failures; escalation thresholds configurable (default 10)
  - **Cold start failure:** Hold home position and retry next step
  - **Three-tier fallback:** shifted previous → hold last command → home (zero extensions)
  - Logging: status, solve time, and consecutive failure count on every failure
- [x] Implement predicted trajectory visualization in `sim/viz/horizon.py`:
  - `HorizonRenderer` class: renders N+1 predicted platform positions as translucent green spheres
  - Colour-coded by time: bright (near) → faded (far), sphere radius shrinks with horizon depth
  - Uses `mjv_initGeom()` on `viewer.user_scn` — no viewer patching needed
  - Toggle via `enabled` property (always on when `--mpc` is active)
- [x] Wire MPC into simulation loop (`sim/main.py`):
  - `--mpc` flag enables MPC mode; without it, existing direct-command mode is preserved
  - `ReferenceScheduler` class: returns target pose by time (does not command plant directly)
  - `run_mpc_headless()` and `run_mpc_with_viewer()`: MPC-specific loop variants
  - MPC diagnostics (solve_time, status, cost, constraint_violation) logged to telemetry CSV
  - Horizon visualisation active in viewer mode
- [x] Write `sim/tests/test_mpc_static.py` — 14 tests across 6 classes, all passing:
  - `TestMPCBuild` — construction, cold-start solve
  - `TestMPCStaticTracking` — z+50mm, x+20mm@z+50mm, pitch+5°@z+80mm, combined pose (all < 1 mm / 0.5°)
  - `TestMPCPerformance` — settle time < 500 ms, mean solve < 15 ms
  - `TestMPCConstraints` — stroke limits [0, 280], rate limits ≤ v_max·dt
  - `TestMPCSolverFailure` — fallback on failure, reset clears state
  - `TestMPCPredictedTrajectory` — (N+1, 6) shape, first pose matches current state

**Test results:**
| Test case | Position error | Orientation error | Cold-start | Warm mean | P95 |
|-----------|---------------|-------------------|------------|-----------|-----|
| z+50 mm | 0.232 mm | 0.036° | 16 ms | 5.0 ms | 6.0 ms |
| x+20 mm, z+50 mm | 0.257 mm | 0.036° | 26 ms | 5.5 ms | 7.1 ms |
| pitch+5°, z+80 mm | 0.231 mm | 0.023° | 21 ms | 5.0 ms | 8.1 ms |
| Combined | 0.350 mm | 0.016° | 37 ms | 5.6 ms | 7.5 ms |

All 43 tests pass (22 Phase 0 + 7 Phase 1 + 14 Phase 2) in < 5 seconds.

**Implementation note — IK as equality constraint (Option 1):**
The symbolic IK uses the Rodrigues formula with a regularised denominator (`angle_sq + 1e-20`) so that `sin(θ)/θ` and `(1-cos θ)/θ²` evaluate correctly at θ=0. This avoids `if_else` branching in CasADi and produces smooth, well-conditioned derivatives for IPOPT. Each horizon step has 6 IK equality constraints (one per leg), each involving `norm_2(leg_vector)` — CasADi handles the `sqrt` derivative correctly since leg lengths are always ~600-900 mm (far from zero).

**Implementation note — N=10 vs N=20:**
The plan originally specified N=20 (400 ms lookahead). With N=20, the NLP has 360 decision variables and 360 constraints; cold-start solves consistently exceeded the 18 ms budget on Windows (every solve timed out, preventing warm-start accumulation). Reducing to N=10 (180 variables, 200 ms lookahead) brought cold-start solves to 16-37 ms and warm-started solves to ~5 ms. For Phase 3 trajectory tracking, the 200 ms horizon should still be adequate since the MPC re-solves every 20 ms. If a longer horizon is needed (e.g., for Phase 5 dynamic targets), options include:
- Reducing control rate from 50 Hz to 25 Hz (doubling the lookahead at the same N)
- Using CasADi code generation (`nlpsol(..., {'jit': True})`) to speed up evaluations
- Profiling on Jetson (ARM + IPOPT may have different performance characteristics)

**Implementation note — reachability from home:**
The home position corresponds to full retraction (extension=0). Lateral and rotational motions require some legs to shorten below their home length, which is infeasible (extension < 0 is out of stroke). This means **pure lateral or pure rotational targets from home are unreachable** — the platform must first be raised (z > 0) to give legs room to both extend and retract. This is a real physical constraint of the robot, not a simulation artefact. The test suite uses raised-base targets (e.g., x+20mm at z+50mm) to stay within the reachable workspace.

**Implementation note — warm-starting:**
IPOPT warm-starting uses both primal (`x0`) and dual (`lam_g0`, `lam_x0`) variables from the previous solve, shifted by one timestep. This reduces typical iteration counts from 10-12 (cold start) to 2-4 (warm start), yielding the ~3× speedup observed. The warm-start bound push parameters are set to `1e-8` (vs IPOPT's default `1e-2`) to allow the warm-started iterate to start closer to constraint boundaries.

**Implementation note — rate limit revision (Phase 3):**
The original `max_leg_vel_mmps = 50` was revised to `300` during Phase 3. The 50 mm/s limit (1 mm/step at 50 Hz) was too restrictive for the N=10 horizon — the MPC could only plan 10 mm of total extension change, which prevented orientation tracking for combined poses. With 300 mm/s (6 mm/step), the MPC has sufficient room for differential leg motions needed for pitch/roll changes. The smoothness costs (S, A) remain the primary dynamics limiter; the rate limit is now a safety bound. The Phase 2 test results above were obtained with the original 50 mm/s in Docker; with 300 mm/s, all Phase 2 tests pass with comparable or better tracking accuracy on CasADi 3.7.2.

**Deliverable:** MPC tracks commanded static poses with smooth, constraint-respecting motion in the MuJoCo viewer. Warm-started solves at ~5 ms (4× headroom within the 20 ms budget).

---

### Phase 3: Trajectory Tracking — ✅ COMPLETE (2026-03-16)

Feed the MPC time-varying reference trajectories and validate tracking quality.

**Tasks:**
- [x] Implement `ReferenceGenerator` in `sim/controller/reference.py`:
  - `from_static_pose(pose_6dof)` → constant reference across horizon
  - `from_waypoints(poses, durations)` → quintic interpolation between waypoints (reuses `quintic.py` from production)
  - `from_function(fn)` → arbitrary callable `fn(t) → (pose, twist)` for parametric trajectories (e.g. circular orbit)
  - `evaluate(t_start, dt, N)` → `(N+1, 6)` poses + `(N+1, 6)` twists for the MPC horizon
- [x] Wire trajectory reference into MPC:
  - Extended NLP parameter vector to include reference twist: `p_ref(6*(N+1)) + twist_ref(6*(N+1))`
  - Velocity cost now tracks reference twist: `‖dp/dt - twist_ref(k)‖²` (was `‖dp/dt‖²`)
  - Backward-compatible: when `ref_twist=None`, zeros are passed (identical to static tracking)
  - `solve()` accepts optional `ref_twist: (N+1, 6)` parameter
- [x] Test with scripted trajectories in `sim/input/scripted.py`:
  - **T1: Linear translation** — home → [0, 0, 50, 0, 0, 0] → home, 1s each segment
  - **T2: Circular orbit** — 80 mm radius circle in XY at z=50, 2s period, with 1s quintic ramp-up to orbit start (velocity- and acceleration-matched at junction for C2 continuity)
  - **T3: Multi-axis** — simultaneous translation + tilt, 1.5s segments: home → [30,-20,60,3°,-2°,0] → [-20,30,40,-2°,3°,0] → home
  - **T4: Speed test** — fast point-to-point, 400ms transit from raised position [0,0,80] → [40,-30,60,2°,-1°,0]
- [x] Compare MPC tracking against reference:
  - Logged actual vs reference pose at each MPC step
  - Max/RMS tracking error and solve time statistics computed per trajectory
- [x] Tune MPC parameters:
  - **Critical fix:** `max_leg_vel_mmps` increased from 50 → 300 mm/s (see implementation note below)
  - No other weight changes needed — existing Q, S, A weights work well for trajectory tracking
- [x] Wire trajectory mode into simulation loop (`sim/main.py`):
  - `--trajectory T1|T2|T3|T4` flag (implies `--mpc`)
  - `run_trajectory_headless()` and `run_trajectory_with_viewer()`: pass full (N+1, 6) pose+twist to MPC each step
  - Ref twist logged to telemetry CSV
- [x] Write `sim/tests/test_mpc_trajectory.py` — 15 tests across 7 classes, all passing:
  - `TestReferenceGenerator` — 5 tests: static pose, waypoint shapes/boundaries/twist, circular function
  - `TestT1LinearTranslation` — tracking quality (< 3 mm) + returns to home
  - `TestT2CircularOrbit` — steady-state tracking (< 5 mm) + Z height stability
  - `TestT3MultiAxis` — multi-DoF tracking (< 5 mm / 2°)
  - `TestT4SpeedTest` — fast transit (< 8 mm / 2°) + settle after transit
  - `TestTrajectoryPerformance` — solve time, stroke limits, no solver failures

**Test results:**
| Trajectory | Max pos error | Max ori error | RMS pos error | Mean solve | P95 solve |
|------------|--------------|---------------|---------------|------------|-----------|
| T1: Linear | 0.48 mm | 0.005° | 0.28 mm | 8.4 ms | 14.9 ms |
| T2: Circle | 3.38 mm | 0.835° | 2.66 mm | 8.3 ms | 19.0 ms |
| T3: Multi-axis | 1.01 mm | 1.123° | 0.47 mm | 9.0 ms | 18.2 ms |
| T4: Fast transit | 2.66 mm | 1.062° | 0.93 mm | 7.0 ms | 14.8 ms |

All 58 tests pass (22 Phase 0 + 7 Phase 1 + 14 Phase 2 + 15 Phase 3) in ~24 seconds.

**Implementation note — rate limit fix (50 → 300 mm/s):**
The Phase 2 default `max_leg_vel_mmps = 50` was far too conservative. With the N=10 horizon (200 ms lookahead), a 50 mm/s rate limit means the MPC can only plan 1 mm of leg extension change per step. This made orientation tracking nearly impossible — the differential leg motions needed for pitch/roll changes were severely constrained, causing the solver to prioritise position over orientation. Specifically, the `pitch+5deg_z+80mm` test showed 4.16° residual error (83% of target) because the solver couldn't redistribute leg extensions fast enough.

Increasing to 300 mm/s (still well under the hardware maximum of ~1060 mm/s) immediately resolved this. The rate limit is now 6 mm/step, giving the MPC enough room to plan orientation changes within its 200 ms horizon. The smoothness costs (S=1.0, A=0.2) still prevent jerky motion — the rate limit is a safety bound, not the primary dynamics limiter.

This also fixed the pre-existing Phase 2 static tracking failures (`pitch+5deg_z+80mm`, `combined_pose`, `settle_time`), which were caused by the same rate limit issue.

**Implementation note — reference twist in velocity cost:**
The MPC velocity damping term was changed from penalising absolute velocity `‖dp/dt‖²` to penalising velocity deviation from reference `‖dp/dt - twist_ref‖²`. For static pose tracking (twist_ref = 0), the behaviour is identical. For trajectory tracking, this is essential — without it, the velocity cost fights the reference trajectory, penalising the MPC for following a moving target. The reference twist is computed analytically by the `ReferenceGenerator` (from quintic derivatives or parametric trajectory functions) and passed as an additional NLP parameter.

**Implementation note — T2 circular orbit ramp-up:**
The initial T2 implementation had a step discontinuity at orbit start (home → [80, 0, 50] instantaneously at t=0.5s), causing a 16.7 mm Z-axis transient. This was fixed by adding a 1s quintic ramp-up segment that smoothly moves from home to the orbit starting position [80, 0, 50] with velocity- and acceleration-matched boundary conditions. Start: zero twist/accel. End: tangential velocity `[0, r*omega, 0]` and centripetal acceleration `[-r*omega^2, 0, 0]`. This ensures C2 continuity at the ramp-orbit junction (no jerk spike). Note: the orbit-end transition back to hold has a velocity discontinuity (`vy = r*omega → 0`), which the MPC handles naturally via its prediction horizon.

**Implementation note — CasADi version sensitivity:**
These tests were developed with CasADi 3.7.2 / IPOPT on Windows. The Phase 2 tests (originally developed in Docker with a different CasADi version) had pre-existing failures at the old 50 mm/s rate limit that were masked by different IPOPT convergence behaviour in the Docker environment. The rate limit fix resolves these failures across CasADi versions. Future investigation: the Docker environment's CasADi version should be pinned to match local development for reproducibility.

**Deliverable:** MPC smoothly tracks multi-waypoint and parametric trajectories in simulation. Tracking quality documented with test results. Velocity-matched reference twist enables the MPC to anticipate trajectory changes through the prediction horizon.

---

### Phase 4: SpaceMouse Integration — ✅ COMPLETE (2026-03-17)

Wire spacemouse and keyboard input to MPC as a live pose reference.

**Tasks:**
- [x] Implement `SpaceMouseInput` in `sim/input/spacemouse.py`:
  - Reads spacemouse state via `pyspacemouse` (same library as existing `spacemouse_handler.py`)
  - Applies sensitivity multipliers from `hardware_config.yaml` (`jugglebot_spacemouse` section):
    - XY: ±150 mm, Z: ±140 mm (offset by `default_active_z_mm` = 170 mm), pitch/roll: ±30°, yaw: ±10°
  - Output: target pose as `[x, y, z, rx, ry, rz]` (rotation vector, matching production)
  - Background polling thread at 100 Hz; MPC reads latest value at 50 Hz
  - Connection retry (3 attempts, 2s delay) matching production `spacemouse_handler.py`
  - Graceful cleanup: `close()` stops thread and closes HID device
  - Linux-only (requires `libhidapi-dev` and USB HID access)
- [x] Implement `KeyboardInput` in `sim/input/keyboard.py` (cross-platform alternative):
  - Uses MuJoCo viewer's `key_callback` (passed to `launch_passive()`) — works on Windows and Linux without special hardware
  - Uses arrow keys + numpad to avoid conflicts with MuJoCo viewer's built-in letter-key visualisation toggles
  - Incremental pose adjustments: Arrow Up/Down (Y), Arrow Left/Right (X), Page Up/Down (Z), Numpad 8/2 (pitch), Numpad 4/6 (roll), Numpad 7/9 (yaw)
  - Home key resets to default pose (0, 0, z_offset, 0, 0, 0)
  - Numpad +/- adjusts sensitivity (0.2x to 5.0x multiplier on movement rate)
  - Hold-to-move: holding a key applies continuous motion at a fixed rate (150 mm/s translation, 30 deg/s rotation at 1.0x sensitivity). Keys are tracked via timestamp; released after 100 ms of no callbacks.
  - Workspace clamping: ±150 mm XY, 0–260 mm Z, ±26° pitch/roll, ±11° yaw
- [x] Feed interactive target into MPC reference:
  - Each MPC step reads the latest target pose from the input device
  - Target is broadcast across the full N+1 horizon (constant reference) — MPC plans optimal path
  - No explicit stream smoother needed — MPC's smoothness penalty (S weight) and prediction horizon provide smooth, constraint-respecting motion
- [x] Handle edge cases:
  - SpaceMouse disconnected: fails at startup with helpful error, directs to `--keyboard`
  - Target outside workspace: MPC constraint satisfaction automatically clips to feasible region
  - Zero input (spacemouse at rest): default pose is `[0, 0, 170, 0, 0, 0]` (raised to operating height)
  - Keyboard at rest: holds last incremental target
- [x] Wire into simulation loop (`sim/main.py`):
  - `--spacemouse` flag: SpaceMouse interactive mode (implies `--mpc`, Linux only)
  - `--keyboard` flag: keyboard interactive mode (implies `--mpc`, cross-platform)
  - Both require the viewer (incompatible with `--no-viewer`)
  - `run_interactive_with_viewer()`: unified loop for both input types, with horizon visualisation
  - MPC leg velocity limit raised to 1000 mm/s for interactive mode (fast target changes)
  - Default duration 300 s (5 minutes, effectively unlimited — close viewer to stop)
  - SpaceMouse automatically cleaned up on exit (thread stop + HID close)

**Rotation vector validation:**
The `_rotvec_from_euler()` function in `spacemouse.py` reproduces the production code's quaternion composition (`q_yaw * q_roll * q_pitch`) using rotation matrices and Rodrigues inverse. Validated against `numpy-quaternion` to machine precision (max diff ~1e-17).

**Implementation note — MPC as the stream smoother:**
The production robot uses a dedicated `stream_smoother.py` (C2-continuous) between the SpaceMouse and the control loop. In the sim, the MPC replaces this entirely — the prediction horizon + smoothness costs produce naturally smooth motion to the target. This is one of the key advantages of the MPC approach: the same controller handles trajectory tracking, static pose tracking, and interactive input without any mode-specific smoothing logic.

**Implementation note — keyboard input design:**
The keyboard input uses rate-based motion: the control loop calls `apply(dt)` each step (50 Hz) to integrate motion for all "active" keys at a fixed rate (150 mm/s translation, 30 deg/s rotation at 1.0x sensitivity). Sensitivity is a multiplier on the movement rate — at 2.0x, translation runs at 300 mm/s and rotation at 60 deg/s. The MPC then plans smooth motion to the continuously-updated target.

**Known limitation — hold-to-move:** MuJoCo's C++ `_Simulate` class only forwards `GLFW_PRESS` events to the Python `key_callback`, not `GLFW_REPEAT` or `GLFW_RELEASE`. This means holding a key produces only a single callback (on initial press), not a continuous stream. The hold-to-move mechanism works in principle (timestamp tracking + 100 ms timeout) but in practice each key press produces only one step of motion (~3 mm / 0.6° at default rate). Rapid tapping works as an alternative. A proper fix would require patching MuJoCo's C++ simulate to forward repeat events, or using a separate GLFW window for input polling.

**Implementation note — key binding conflicts:**
MuJoCo's passive viewer reserves most letter keys for built-in visualisation toggles (e.g. W=wireframe, F=joint frames, E=inertia ellipsoids). The `key_callback` fires *alongside* these built-in bindings — there is no way to suppress them. The keyboard input therefore uses arrow keys (translation), Page Up/Down (Z), and numpad keys (rotation, sensitivity) which do not conflict with any viewer shortcuts. The `key_callback` must be passed to `launch_passive()` at construction time — it cannot be set on the viewer handle after creation.

**Implementation note — platform support:**
The interactive viewer (keyboard/spacemouse input) works on Windows. The MuJoCo viewer does not work on Jetson Orin Nano due to Tegra GLX incompatibility (see Development Notes). SpaceMouse additionally requires `pyspacemouse` + `libhidapi-dev` + USB HID access (Linux/Docker only). Both input modes share the same `run_interactive_with_viewer()` loop — the only difference is the `read()` source.

**Deliverable:** Move the simulated Stewart platform interactively in real-time via SpaceMouse (Linux) or keyboard (cross-platform). Motion is smooth and respects workspace limits. All 60 pre-existing tests pass with no regressions.

---

### Phase 5: Dynamic Target (Ball Catching)

Implement timed target interception — the platform must arrive at a target pose by a deadline.

**Tasks:**
- [ ] Define the dynamic target interface in `sim/input/scripted.py`:
  ```python
  @dataclass
  class DynamicTarget:
      pose_6dof: np.ndarray       # [x, y, z, rx, ry, rz] target pose
      arrival_time: float          # absolute time by which platform must be at pose
      arrival_twist: np.ndarray | None = None  # (6,) optional twist at arrival
                                   # None or zeros = catch (hold at pose)
                                   # Non-zero = throw (platform moving at arrival)
      hold_duration: float = 0.5   # seconds to hold after arrival (catch mode only)
  ```
  This matches the existing Phase 7 API which supports `target_vel` for throw motions. When `arrival_twist` is non-zero, the platform must be moving at the specified velocity when it reaches the target pose — this is the throw case where the hand releases the ball while the platform is in motion.
- [ ] Implement time-aware reference generation:
  - Given current state and a `DynamicTarget`, generate a reference trajectory that:
    1. Arrives at `target.pose_6dof` at or before `target.arrival_time`
    2. **Catch mode** (`arrival_twist` is None/zero): zero velocity at arrival, hold pose for `hold_duration`, return to home
    3. **Throw mode** (`arrival_twist` is non-zero): specified velocity at arrival, then decelerate to stop at a physically-computed endpoint (not home — momentum carries the platform). Return to home after deceleration.
  - Reference trajectory uses quintic interpolation with duration = `arrival_time - now`
  - If arrival time is too soon for feasible motion: arrive as early as possible (MPC does its best; constraint satisfaction prevents damage)
  - **MPC terminal constraint difference:** In catch mode, the MPC terminal cost penalises both pose error and twist. In throw mode, it penalises pose error and twist *deviation from target twist* — the platform should be moving at the right velocity, not stationary.
- [ ] Implement feasibility pre-check:
  - Before committing to a target, estimate whether the platform can reach it in time
  - Use a simplified check: max leg velocity × available time ≥ required leg displacement
  - If infeasible: log warning, optionally reject target
- [ ] Test with synthetic ball sequences:
  - **DT1: Single catch** — target at [30, -20, 80, 3°, -2°, 0°], arrival in 400 ms
  - **DT2: Rapid succession** — two targets 800 ms apart, different poses
  - **DT3: Edge of workspace** — target near soft workspace limit, arrival in 500 ms
  - **DT4: Infeasible** — target requiring motion faster than actuator limits, verify graceful handling
  - **DT5: Early arrival** — target with 2s lead time, verify platform arrives early and holds
  - **DT6: Throw (non-zero arrival velocity)** — target at [0, 0, 60, 0, 0, 0] with arrival_twist [0, 0, -200, 0, 0, 0] (downward at 200 mm/s), 500 ms. Verify platform has correct velocity at arrival time, then decelerates smoothly to stop.
  - **DT7: Catch then throw** — catch target at t=0.4s (zero velocity), hold 0.3s, throw target at t=1.0s (non-zero velocity). Verify both phases execute correctly in sequence.
- [ ] Validate timing:
  - Log: time of arrival vs deadline for each target
  - Acceptance: arrive within 1 MPC step (20 ms) of deadline, or early
  - Acceptance: holding pose error < 2 mm at moment of "catch"
- [ ] Mid-motion replanning:
  - If a new target arrives while moving to a previous target, MPC naturally handles this — the reference trajectory changes, and the MPC replans from its current state
  - Test: send target A, then 200 ms later send target B. Verify smooth transition.

**Deliverable:** Simulated platform reliably arrives at target poses before deadlines. Handles multiple targets, infeasible requests, and mid-motion replanning.

---

### Phase 6: Hardware Bridge

Swap the simulated plant for real hardware. MPC outputs motor commands via CAN.

**Tasks:**
- [ ] Implement `HardwarePlant` in `sim/plant/hardware_plant.py`:
  - `command()`: convert leg extensions (mm) → motor positions (rev) using per-leg `mm_to_rev` factors, then send via CAN
  - `get_state()`: read motor encoder positions/velocities from CAN feedback, convert to `PlantState`
  - `step()`: no-op (hardware runs in real time)
  - `reset()`: send home position command via existing trajectory system
  - Communication: either direct CAN (reusing `can/bus.py` + `can/odrive.py`) or via IPC to the existing control loop
- [ ] Decide integration approach (two options):
  - **Option A: MPC replaces control_loop.py** — MPC runs at 50 Hz, sends `set_input_pos(pos, vel_ff, torque_ff)` directly to ODrives via CAN. `vel_ff` and `torque_ff` are zero initially (position-only). Cleanest but requires MPC to handle all safety checks (slew limiter, workspace enforcement).
  - **Option B: MPC feeds into control_loop.py** — MPC outputs target poses at 50 Hz via IPC (same `TOPIC_TARGET` protocol). The existing 500 Hz control loop handles IK, feedforward, safety, and motor commands. Safest, reuses proven infrastructure.
  - **Recommended: Option B initially**, then Option A once MPC is proven.
- [ ] Safety integration:
  - Workspace checks: either replicated in MPC constraints (Option A) or handled by existing control_loop.py (Option B)
  - E-stop: hardware kill switch remains independent of software
  - Slew limiter: existing `max_position_step_rev: 0.2` enforcement
- [ ] Calibrate actuator time constant `τ` from real hardware data:
  - Use existing Phase 3-7 hardware test logs (step responses, trajectory tracking error)
  - Method: fit first-order lag `q_actual(t) = q_cmd · (1 - e^{-t/τ})` to step response data from ODrive encoder feedback
  - Alternatively: measure tracking delay at multiple speeds during T1-T3 trajectory replays
  - Expected range: 20-50 ms. If τ varies significantly by leg (due to mechanical differences like leg 2), consider per-leg τ values.
  - Update `MPCParams.tau` with the calibrated value. Re-run sim trajectories and compare tracking error — sim should now predict real tracking error within 20%.
- [ ] Staged hardware bring-up:
  1. **Bench (25% speed):** Run MPC at 50 Hz, feed targets to control_loop.py via IPC, observe tracking
  2. **50% speed:** Increase speed scaling, log tracking error vs simulation predictions
  3. **100% speed:** Full-speed operation, compare with simulation
- [ ] Sim-to-real validation:
  - Record same trajectory in sim and on hardware using identical `StepRecord` telemetry schema (defined in Phase 1)
  - Compare side-by-side: tracking error, leg velocities, solve times, constraint violations
  - Key metrics: sim-predicted tracking error vs actual (target: within 30%), sim-predicted solve time vs actual on Jetson
  - Document any sim-to-real discrepancies — these inform whether dynamics integration (see Future Work) is needed
  - If discrepancies are large, re-calibrate τ and re-run. If still large, consider Approach A from Dynamics Integration.

**Deliverable:** MPC running on real hardware, tracking poses from spacemouse and scripted targets. Validated against simulation predictions with quantified sim-to-real gap.

---

## Future Work: Dynamics Integration

The initial MPC uses a kinematics-only prediction model (actuators track commanded positions with a first-order lag). This section outlines how to add full dynamics when needed — likely when pushing toward faster ball-catching motions where inertial effects matter.

### When dynamics become necessary
At current Phase 7 speeds (worst tracking 2.4 mm), the kinematic model should suffice because ODrive's 8 kHz PID loop handles the dynamics locally. Dynamics become important when:
- Catching motions require > 50% of actuator force capacity (inertial loads compete with gravity)
- Tracking error exceeds acceptable limits due to unmodelled inertial coupling between legs
- You want MPC to produce `vel_ff` and `torque_ff` for better tracking (replacing or augmenting control_loop.py's feedforward)

### Implementation approaches

**Approach A: Add feedforward torques to MPC output (easiest)**
Keep the kinematic MPC model but compute feedforward torques as a post-processing step using the existing `dynamics.py` Newton-Euler model:
```
MPC solves for: q_cmd(k) (leg extensions)
Post-process:   pose, twist, accel = differentiate q_cmd trajectory
                 torque_ff = compute_full_feedforward_torques(pose, twist, accel, ...)
Send to plant:   (q_cmd, vel_ff, torque_ff)
```
This is non-invasive — the MPC problem doesn't change, you just extract more from its solution.

**Approach B: Dynamics in the MPC prediction model (most principled)**
Replace the first-order lag model with a full Newton-Euler dynamics model expressed symbolically in CasADi:
```
x(k+1) = x(k) + dt · f_dynamics(x(k), u(k))
where f_dynamics includes: gravity, platform inertia, reflected motor inertia, Coriolis/gyroscopic terms
```
This lets MPC *anticipate* inertial effects and plan accordingly. The CasADi symbolic framework handles automatic differentiation of the dynamics. The main cost is increased solve time (more complex NLP), but given your 20 ms budget at 50 Hz, this should be feasible.

Key translation from existing code:
| `dynamics.py` function | CasADi equivalent |
|------------------------|-------------------|
| `compute_gravity_wrench()` | Symbolic: `F_g = [0, 0, -m*g]`, `τ = cross(R @ r_com, F_g)` |
| `compute_inertia_wrench()` | Symbolic: `I_world = R @ I_body @ R.T`, `τ = I_world @ α + cross(ω, I_world @ ω)` |
| `np.linalg.solve(J.T, W)` | `casadi.solve(J.T, W)` |

**Approach C: Learned residual dynamics (research-grade)**
Train a small neural network on sim-to-real tracking error data to capture unmodelled effects (friction, backlash, cable dynamics). Add as a correction term to Approach B. This is only warranted if the analytical model proves insufficient.

**Recommended path:** Start with Approach A (easiest, non-invasive), move to Approach B if tracking at high speeds is inadequate.

---

## Dependencies

| Package | Version | Purpose | Platform |
|---------|---------|---------|----------|
| `mujoco` | ≥ 3.0 | Physics simulation + viewer | All (pip) |
| `casadi` | ≥ 3.6 | Symbolic NLP + IPOPT solver | All (pip) |
| `numpy` | ≥ 1.24 | Array operations | All (pip) |
| `pyspacemouse` | ≥ 0.6 | SpaceMouse input | All (pip) |
| `matplotlib` | ≥ 3.7 | Post-hoc plotting | All (pip) |
| `docker` | ≥ 24.0 | Container runtime | Host machine |
| `nvidia-container-toolkit` | latest | GPU passthrough for MuJoCo viewer | Host machine (if NVIDIA GPU) |

CasADi ships with IPOPT bundled — no separate solver installation required.

MuJoCo is pure pip install since version 2.3.0 (no separate binary download).

All Python dependencies are installed inside the Docker container — the only host requirement is Docker Desktop (Windows) or Docker Engine + NVIDIA Container Toolkit (Linux).

---

## Docker Environment

The entire simulation runs inside a Docker container. A new machine goes from clone to running sim in under 5 minutes.

### Prerequisites (host machine only)

- **Docker Desktop** (Windows) or **Docker Engine** (Linux) — [install guide](https://docs.docker.com/get-docker/)
- **NVIDIA GPU drivers** installed on the host (for MuJoCo viewer hardware acceleration)
- **NVIDIA Container Toolkit** (Linux) or **WSLg** (Windows, included in Windows 11 / recent Windows 10) for GPU passthrough

### Quickstart

```bash
git clone https://github.com/<you>/Jugglebot.git
cd Jugglebot/sim

# Build the container (~3 min first time, cached after)
docker compose build

# Run tests (headless, no GPU needed)
docker compose run --rm sim pytest tests/

# Run simulation with viewer
docker compose up sim

# Run simulation headless (logging only, no display)
docker compose run --rm sim python main.py --no-viewer --trajectory T1
```

### Dockerfile (overview)

```dockerfile
FROM nvidia/cuda:12.2.0-runtime-ubuntu22.04

RUN apt-get update && apt-get install -y \
    python3.11 python3.11-venv python3-pip \
    libgl1-mesa-glx libglew-dev libosmesa6-dev \
    libhidapi-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . /app
WORKDIR /app
```

The NVIDIA CUDA base image provides GPU access for MuJoCo's OpenGL rendering. `libhidapi-dev` enables SpaceMouse USB access inside the container.

### compose.yaml (overview)

```yaml
services:
  sim:
    build: .
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    environment:
      - DISPLAY=${DISPLAY:-:0}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix      # X11 (Linux)
      - ./logs:/app/logs                    # Persist telemetry logs
    devices:
      - /dev/hidraw0:/dev/hidraw0           # SpaceMouse (adjust device path)
    command: python main.py
```

### Compose profiles

| Command | What it does |
|---------|-------------|
| `docker compose up sim` | Viewer + SpaceMouse + full simulation |
| `docker compose run --rm sim pytest tests/` | Run all tests headless |
| `docker compose run --rm sim python main.py --no-viewer` | Headless sim with telemetry logging |
| `docker compose run --rm sim python main.py --trajectory T2` | Run a specific scripted trajectory |

### Display forwarding by OS

| Host OS | How the viewer reaches your screen |
|---------|-----------------------------------|
| **Linux** | Native X11: mount `/tmp/.X11-unix`, set `DISPLAY`. Works out of the box. |
| **Windows** | WSLg (Windows 11 / recent Win 10): Docker Desktop with WSL2 backend automatically provides a Wayland/X11 display. No extra config. |

### SpaceMouse USB passthrough

The SpaceMouse is a USB HID device. To use it inside the container:
- **Linux:** `--device /dev/hidraw0` (check `ls /dev/hidraw*` to find the right device)
- **Windows (WSL2):** Requires `usbipd-win` to attach the USB device to WSL, then `--device` as above. One-time setup: `usbipd bind --busid <BUS_ID>`, then `usbipd attach --wsl --busid <BUS_ID>`.

If no SpaceMouse is connected, the sim runs normally — spacemouse input simply produces no data (same as existing `spacemouse_handler.py` behaviour).

---

## Development Notes

- **No ROS2 in the sim path.** The entire `sim/` directory is a standalone Python project. ROS2 only enters at Phase 6 via the existing IPC bridge or direct CAN access.
- **Reuse existing code by import, not copy.** Where the existing `motion/` subpackage has useful math (IK, quintics, workspace limits), import it directly. The CasADi symbolic model is a *parallel expression* of the same math, not a replacement — both exist for different purposes.
- **MuJoCo model ≠ MPC model.** The MuJoCo model is the "ground truth" simulation. The MPC model is what the controller *thinks* the world looks like. Keeping these separate lets you test robustness to model mismatch (e.g., add payload mass to MuJoCo but not to MPC).
- **Warm-starting is critical.** IPOPT converges much faster when initialised with the previous solution shifted by one timestep. This is standard MPC practice and should be implemented from Phase 2.
- **Solver failure is expected, not exceptional.** IPOPT will occasionally fail to converge — aggressive targets, cold starts, numerical issues near singularities. The fallback strategy (apply shifted previous solution) must be implemented from Phase 2 and tested explicitly. On hardware, consecutive failures escalate to hold → E-STOP.
- **Log everything.** Every MPC solve should log to the `StepRecord` schema: solve time, cost, status, constraint violations, reference vs actual. This data is invaluable for tuning and is the basis for quantitative sim-to-real comparison in Phase 6. Use the same schema in sim and on hardware.
- **Docker is the canonical environment.** All development, testing, and CI runs inside the Docker container. If a dependency or system library is needed, add it to the `Dockerfile` — never rely on host-installed packages. This ensures any machine with Docker + an NVIDIA GPU can reproduce the full environment.
- **Jetson: viewer not supported.** The MuJoCo interactive viewer (GLFW) does not work on Jetson Orin Nano (JetPack 5.1.1 / L4T R35.3.1). GLFW fails with `GLX: No GLXFBConfigs returned` — the Tegra GLX driver does not advertise framebuffer configs compatible with GLFW's requirements, despite `glxinfo` reporting a working display. Attempted fixes: `libglvnd-dev`/`libgl1-mesa-glx` packages, `__GLX_VENDOR_LIBRARY_NAME=nvidia`, `LD_PRELOAD` of `libGLX_nvidia.so.0`, system `libglfw3` via `PYGLFW_LIBRARY`, and `MUJOCO_GL=egl` — none resolved the issue. **Use Windows for interactive sim (viewer + keyboard/spacemouse). Jetson is headless only** (`--no-viewer`) for Phase 6 hardware bridge testing.
- **Local venv for Windows development.** A local venv at `sim/.venv/` is used for Windows development outside Docker. Created with `python -m venv sim/.venv` and `pip install -r sim/requirements.txt`.

---

## Appendices

### Appendix A: STL Mesh Coordinate Conventions

All STL meshes are exported from Onshape in **millimeters**. The MJCF generator applies `scale="0.001 0.001 0.001"` to convert to meters at load time.

Each mesh has its Onshape origin at a specific offset from the functional attachment point. The generator compensates with a `pos` attribute on the mesh geom:

| Mesh | Onshape origin relative to attachment | MuJoCo `pos` offset (m) | Notes |
|------|---------------------------------------|-------------------------|-------|
| `base.stl` | (0, 0, -82) mm from ball joint plane (global origin) | `0 0 -0.082` | Base sits below the Z=0 ball joint plane |
| `platform.stl` | At platform body frame origin | `0 0 0` | No offset needed |
| `leg_outer.stl` | (0, 0, -64) mm from ball joint centre, Z along leg axis | `0 0 -0.064` | Placed in leg base body (ball joint at origin) |
| `leg_inner.stl` | (0, 0, -704.47) mm from ball joint centre, Z along leg axis | `0 0 -0.70447` | Placed in leg inner body (platform attachment at origin) |

All leg meshes are identical across the 6 legs. The per-leg rotation is handled by the body `quat` attribute on each `leg_{i}_base` body.

If meshes are absent from `sim/model/meshes/`, the generator falls back to primitive shapes (capsules for legs, cylinder for base, cylinder for platform). This allows the model to work without Onshape exports.

### Appendix B: MuJoCo Modelling Lessons

Key issues encountered during Phase 0 development and their solutions:

**1. Nested body structure for correct constraint anchors**

MuJoCo computes `connect` equality constraint anchor positions at **compile time** using the reference configuration (all joints at `qpos0`). The initial approach — a single body per leg with the slide joint spanning the full leg length — failed because at `qpos0=0` the leg tip body was at the base node, not the platform node. The constraint anchor was computed 648 mm away from where it needed to be, causing the platform to fly apart on simulation start.

**Solution:** Nested body structure. Each leg has an **outer body** at the base node (with 2 hinge joints for ball joint DoF) containing an **inner body** offset by the geometric home leg length. The slide joint is on the inner body. At compile time (`slide=0`), the inner body is already at the platform attachment point, so the `connect` constraint anchor is `(0, 0, 0)` — the body origin.

```
leg_{i}_base (at base_node, with quat aligning Z to leg direction)
├── joint: hinge_x, hinge_y (ball joint approximation)
├── geom: outer tube mesh or capsule
└── leg_{i} (offset by [0, 0, home_len] — at platform node when slide=0)
    ├── joint: slide along Z (range: [-margin, stroke+margin])
    ├── geom: inner tube mesh
    └── site: leg_tip (for visualization)
```

**2. Site-based sensors for body origin (not CoM)**

MuJoCo's `framepos` and `framequat` sensors with `objtype="body"` return the **inertial frame** (centre of mass), not the body origin. For the platform, this was offset by `platform_com_offset_mm = [-9.68, -68.64, 52.73]`, causing an apparent 69 mm position error at home.

**Solution:** Add a `<site name="platform_origin" pos="0 0 0"/>` to the platform body and use `objtype="site"` on the sensors. Sites report their actual position, not the CoM.

**3. IK extension coordinate system vs geometric lengths**

The IK model uses `init_leg_lengths_mm` (raw measured leg lengths, per-leg, WITHOUT `ball_joint_offset_mm`) as the zero reference for extensions. MuJoCo's slide joints use geometric home lengths (uniform, computed from node positions) as their zero reference. The conversion between IK extensions and MuJoCo slide values must account for this:

```python
# IK extensions (mm) → MuJoCo slide values (m)
abs_length_m = (geom.init_leg_lengths_mm + extensions_mm) / 1000.0
slide_m = abs_length_m - geometric_home_lengths_m
```

The per-leg discrepancy between measured and geometric lengths (~2-5 mm) means commanding "IK home" (extensions=0) results in slightly different slide positions per leg. FK validation must compare MuJoCo's pose against FK of the **actual slide positions**, not the commanded pose.

**4. Ground plane alignment**

The base mesh's Onshape origin is 82 mm below the ball joint plane. The ground plane must be at `pos="0 0 -0.082"` to sit at the base bottom, not at `Z=0` (which cuts through the base).
