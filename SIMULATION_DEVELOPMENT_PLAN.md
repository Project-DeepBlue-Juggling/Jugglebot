# Jugglebot MPC Simulation: Stewart Platform with Model Predictive Control

## Summary

Build a MuJoCo-based simulation of Jugglebot's Stewart platform controlled by a nonlinear Model Predictive Controller (CasADi + IPOPT). The simulation replicates the real robot's kinematics and provides a development environment for MPC tuning, spacemouse control, and dynamic target interception — all without requiring the physical robot.

The MPC will eventually deploy to real hardware (Jetson), sending position setpoints to ODrive's existing 8 kHz PID loop.

## Key Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Physics engine | MuJoCo | Best parallel mechanism support (equality constraints), fast, free, built-in viewer |
| MPC solver | CasADi + IPOPT | Nonlinear, symbolic autodiff, pure Python workflow, upgradeable to acados |
| Control rate | 50 Hz MPC, MuJoCo steps at 500 Hz | 50 Hz gives 20 ms solve budget (generous for 6-DoF NMPC); MuJoCo substeps fill in between |
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
│   ├── __init__.py
│   ├── mpc.py                     # CasADi MPC formulation and solver (Phase 2)
│   ├── reference.py               # Reference trajectory generation (Phase 3)
│   └── params.py                  # MPC tuning parameters (Q, R, S, N, etc.) (Phase 2)
│
├── input/
│   ├── __init__.py
│   ├── spacemouse.py              # SpaceMouse → target pose (Phase 4)
│   └── scripted.py                # Scripted target sequences for testing (Phase 3)
│
├── viz/
│   ├── __init__.py
│   ├── horizon.py                 # MPC predicted trajectory overlay in MuJoCo viewer (Phase 2)
│   └── telemetry.py               # Logging, plotting, diagnostics (Phase 1)
│
├── tests/
│   ├── __init__.py
│   ├── test_model.py              # MuJoCo model validation vs existing IK (22 tests)
│   ├── test_mpc_static.py         # MPC tracks static poses (Phase 2)
│   ├── test_mpc_trajectory.py     # MPC tracks moving references (Phase 3)
│   └── test_mpc_dynamic.py        # MPC intercepts timed targets (Phase 5)
│
└── main.py                        # Entry point: run simulation loop (Phase 1)
```

---

## Phased Implementation

### Phase 0: MuJoCo Stewart Platform Model — COMPLETE (2026-03-16)

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

### Phase 1: Simulation Harness

Build the simulation loop and plant abstraction layer.

**Tasks:**
- [ ] Define `PlantInterface` (abstract base class) in `sim/plant/interface.py`:
  ```python
  class PlantInterface(ABC):
      @abstractmethod
      def command(self, leg_extensions_mm: np.ndarray) -> None:
          """Send 6 leg extension commands."""

      @abstractmethod
      def get_state(self) -> PlantState:
          """Read current platform state."""

      @abstractmethod
      def step(self, dt: float) -> None:
          """Advance simulation by dt (no-op for hardware)."""

      @abstractmethod
      def reset(self, pose_6dof: np.ndarray | None = None) -> None:
          """Reset to home or specified pose."""
  ```
  Where `PlantState` is:
  ```python
  @dataclass
  class PlantState:
      leg_extensions_mm: np.ndarray    # (6,) actual leg positions
      leg_velocities_mmps: np.ndarray  # (6,) actual leg velocities
      platform_pos_mm: np.ndarray      # (3,) [x, y, z] offset from home
      platform_rot: np.ndarray         # (3,) rotation vector (rad)
      platform_twist: np.ndarray       # (6,) [vx,vy,vz,wx,wy,wz]
      time: float                      # simulation time (s)
  ```
- [ ] Implement `MuJoCoPlant` in `sim/plant/mujoco_plant.py`:
  - Load MJCF model, create `mujoco.MjData`
  - `command()`: set actuator controls (position targets)
  - `get_state()`: read joint positions/velocities, compute platform pose from MuJoCo body state
  - `step()`: call `mujoco.mj_step()` for N substeps (e.g. 10 substeps of 0.2 ms per 2 ms control step)
  - `reset()`: `mujoco.mj_resetData()`, optionally set initial pose via IK
- [ ] Implement main simulation loop in `sim/main.py`:
  - 50 Hz outer loop (MPC control rate)
  - Each MPC step: read state → compute control → command plant → step plant by 20 ms (with internal 500 Hz substeps)
  - MuJoCo viewer running in parallel (passive viewer, updated each outer step)
  - Clean shutdown on Ctrl+C
  - Command-line args: `--no-viewer` (headless), `--pose x,y,z,rx,ry,rz` (initial command)
- [ ] Basic test: command a static pose offset (e.g. z+50mm), watch the platform move in the viewer
- [ ] Define telemetry logging schema in `sim/viz/telemetry.py`:
  - Structured per-step log record matching the existing IPC telemetry where possible:
    ```python
    @dataclass
    class StepRecord:
        time: float                      # simulation time (s)
        # Reference
        ref_pose: np.ndarray             # (6,) [x,y,z,rx,ry,rz] reference
        ref_twist: np.ndarray            # (6,) reference twist
        # Actual
        actual_pose: np.ndarray          # (6,) actual platform pose
        actual_twist: np.ndarray         # (6,) actual platform twist
        # Actuators
        cmd_extensions_mm: np.ndarray    # (6,) commanded leg extensions
        actual_extensions_mm: np.ndarray # (6,) actual leg extensions
        # MPC diagnostics
        solve_time_ms: float             # IPOPT wall-clock solve time
        solve_status: str                # "converged", "max_iter", "failed", etc.
        cost: float                      # optimal cost value
        constraint_violation: float      # max constraint violation
        # Derived
        tracking_error_mm: float         # position tracking error norm
        tracking_error_deg: float        # orientation tracking error norm
    ```
  - Write to CSV or Parquet file (one row per MPC step)
  - Same schema used for sim and hardware — enables direct sim-to-real comparison in Phase 6
  - Plotting utilities: time-series of each DoF (ref vs actual), solve time histogram, tracking error over time

**Deliverable:** Running simulation where you can programmatically command leg extensions and see the platform respond in the 3D viewer. Telemetry logged in a structured format from the start.

---

### Phase 2: MPC Core (Static Pose Tracking)

Formulate and implement the NMPC for tracking a reference pose.

**Tasks:**
- [ ] Define CasADi symbolic model in `sim/controller/mpc.py`:
  - Symbolic FK: given 6 leg extensions, compute platform pose (translation + rotation vector). This re-expresses the math from `ik_solver.py` using CasADi symbolic variables so that automatic differentiation works.
  - Actuator dynamics: first-order lag model `q(k+1) = q(k) + (q_cmd - q) / τ · dt`
  - Full prediction model: `x(k+1) = [FK_pos(q(k+1)), FK_rot(q(k+1)), (x_pos(k+1) - x_pos(k))/dt, ...]`
- [ ] Formulate the optimal control problem:
  - **Horizon:** N = 20 steps at 50 Hz (400 ms lookahead)
  - **Cost function:**
    - `Q` — pose tracking: penalise `‖x(k) - x_ref(k)‖²`. Separate weights for position (mm) and orientation (rad) to handle unit mismatch.
    - `R` — control effort: penalise `‖u(k)‖²` (leg extensions)
    - `S` — control smoothness: penalise `‖u(k) - u(k-1)‖²` (prevents jerky leg commands)
    - Terminal cost `Q_f` on final state (heavier weight to ensure convergence)
  - **Constraints:**
    - Leg extensions: `0 ≤ q_i ≤ 280 mm` (stroke limits)
    - Leg velocity: `|Δq_i / dt| ≤ v_max` (from `mm_to_rev` × 15 rev/s ≈ 1060 mm/s per leg)
    - Workspace: condition number of Jacobian < 900 (hard limit). Implemented as a penalty rather than a hard constraint initially, since `cond(J)` is expensive to differentiate symbolically. Can be hardened later.
  - **Solver:** IPOPT with exact Hessian (CasADi provides via AD), warm-starting from previous solution
- [ ] Implement `MPCController` class:
  ```python
  class MPCController:
      def __init__(self, params: MPCParams):
          """Build CasADi problem (once at startup)."""

      def solve(self, current_state: PlantState,
                reference: np.ndarray) -> np.ndarray:
          """Solve MPC, return optimal leg extension command."""

      def set_reference_trajectory(self,
                refs: list[np.ndarray], times: list[float]):
          """Set time-varying reference for the horizon."""
  ```
- [ ] Define `MPCParams` in `sim/controller/params.py`:
  - Q, R, S weight matrices
  - N (horizon length)
  - dt (control timestep = 0.02 s)
  - τ (actuator time constant)
  - Constraint bounds
  - IPOPT options (max iterations, tolerance, warm-start)
- [ ] Tune for static pose tracking:
  - Step from home to [0, 0, 50, 0, 0, 0] (z +50 mm)
  - Step to [50, 0, 0, 0, 0, 0] (x +50 mm)
  - Step to [0, 0, 0, 5°, 0, 0] (pitch +5°)
  - Acceptance: smooth approach with no overshoot, settle within 500 ms, no constraint violations
- [ ] Measure solve time: target < 15 ms per step on Windows dev machine (well within 20 ms budget)
- [ ] Implement solver failure handling in `MPCController.solve()`:
  - **Timeout:** Set IPOPT `max_cpu_time` to 18 ms (90% of 20 ms budget). If IPOPT hits the limit, it returns the best iterate found so far.
  - **Non-convergence:** If IPOPT returns a non-optimal status (max iterations, infeasible, numerical error):
    1. Apply the first control step from the **previous** solution shifted by one timestep (warm-start fallback). This is always available after the first successful solve.
    2. Log the failure: status code, iteration count, cost, max constraint violation.
    3. Increment a consecutive-failure counter. If failures persist for > 10 consecutive steps (200 ms), trigger a controlled stop (ramp to hold position over 500 ms) and raise an alarm.
  - **Cold start (first solve):** No warm-start available. If the first solve fails, hold the home position and retry next step.
  - **Hardware (Phase 6) escalation:** On hardware, 5 consecutive failures → command hold position, 10 consecutive → trigger E-STOP via existing fault mechanism. These thresholds are conservative and tunable.
- [ ] Implement predicted trajectory visualization in `sim/viz/horizon.py`:
  - After each MPC solve, extract the predicted state trajectory `x*(0..N)` from the solution
  - Render as translucent ghost platforms (or simpler: a 3D line trace of the platform centre) in the MuJoCo viewer using `mjv_initGeom()` custom geoms
  - Toggle on/off via keyboard shortcut in the viewer (e.g. 'H' for horizon)
  - Useful for debugging: see whether the MPC is "planning ahead" correctly, especially during trajectory tracking and dynamic targets
  - Colour-code by time: near-future = bright, far-future = faded

**CasADi FK implementation note:**
The symbolic FK needs to solve "given leg lengths, what is the platform pose?" This is the inverse of the IK (which is trivial — just Euclidean distance). Options:
1. **Implicit FK via IK constraint:** Define IK symbolically (platform pose → leg lengths = known commanded values) and let CasADi/IPOPT solve the implicit equation as part of the NLP. This avoids implementing iterative FK in CasADi.
2. **Explicit FK Newton-Raphson in CasADi:** Unroll a fixed number of Newton steps symbolically. Fragile and complex.
3. **Simplified model:** Use a linear approximation `Δx ≈ J⁻¹ · Δq` around the current operating point, updated each MPC call. Fastest to solve but least accurate at large displacements.

**Recommended approach: Option 1** — express IK as an equality constraint in the NLP. CasADi's symbolic IK is straightforward (it's just 6 Euclidean distance equations), and IPOPT naturally handles the implicit relationship. The platform pose becomes a decision variable constrained to be consistent with the leg lengths.

**Deliverable:** MPC tracks commanded static poses with smooth, constraint-respecting motion in the MuJoCo viewer.

---

### Phase 3: Trajectory Tracking

Feed the MPC time-varying reference trajectories and validate tracking quality.

**Tasks:**
- [ ] Implement `ReferenceGenerator` in `sim/controller/reference.py`:
  - `from_static_pose(pose_6dof)` → constant reference across horizon
  - `from_waypoints(poses, durations)` → quintic interpolation between waypoints (can reuse math from `quintic.py`)
  - `from_trajectory(traj)` → sample existing `QuinticTrajectory` objects at MPC timesteps
  - `evaluate(t_start, dt, N)` → array of N reference states for the MPC horizon
- [ ] Wire trajectory reference into MPC:
  - Each MPC solve receives N future reference states (not just one)
  - Cost function uses per-step reference: `‖x(k) - x_ref(k)‖²_Q`
  - This is the key advantage of MPC over feedforward: it "sees ahead" in the reference
- [ ] Test with scripted trajectories in `sim/input/scripted.py`:
  - **T1: Linear translation** — home → [0, 0, 50, 0, 0, 0] → home, 1s each leg
  - **T2: Circular orbit** — 80 mm radius circle in XY at z=50, 2s period
  - **T3: Multi-axis** — simultaneous translation + tilt, 1.5s
  - **T4: Speed test** — fast point-to-point at the limits of feasible motion (~300 ms transit)
- [ ] Compare MPC tracking against reference:
  - Log: actual pose vs reference pose at each timestep
  - Compute: max tracking error (mm position, deg orientation), RMS error, settling time
  - Visualise: time-series plots of each DoF (reference vs actual)
- [ ] Tune MPC weights if needed:
  - If tracking is sluggish: increase Q (tracking weight)
  - If motion is jerky: increase S (smoothness weight)
  - If approaching limits: verify constraint satisfaction in logs

**Deliverable:** MPC smoothly tracks multi-waypoint trajectories in simulation. Tracking quality documented with plots.

---

### Phase 4: SpaceMouse Integration

Wire spacemouse input to MPC as a pose reference.

**Tasks:**
- [ ] Implement `SpaceMouseInput` in `sim/input/spacemouse.py`:
  - Read spacemouse state via `pyspacemouse` (same library as existing `spacemouse_handler.py`)
  - Apply sensitivity multipliers from `hardware_config.yaml` (`jugglebot_spacemouse` section):
    - XY: ±150 mm, Z: ±140 mm (offset by `default_active_z_mm` = 170 mm), pitch/roll: ±30°, yaw: ±10°
  - Output: target pose as `[x, y, z, rx, ry, rz]`
  - Update rate: 100 Hz (spacemouse poll rate), but MPC only reads latest value at 50 Hz
- [ ] Feed spacemouse target into MPC reference:
  - `ReferenceGenerator.from_static_pose(spacemouse_target)` — MPC plans optimal path to current target
  - The MPC horizon naturally provides smooth motion planning: even if the target changes abruptly, the MPC output is smooth because of the smoothness penalty (S weight) and the prediction horizon
  - No explicit stream smoother needed — MPC replaces it
- [ ] Handle edge cases:
  - SpaceMouse disconnected: hold last target
  - Target outside workspace: MPC constraint satisfaction automatically clips to feasible region (log a warning)
  - Zero input (spacemouse at rest): MPC holds current pose (tracking cost → 0 when at reference)
- [ ] Cross-platform validation:
  - Test Docker container on Windows (Docker Desktop + WSLg) and Linux
  - Verify MuJoCo viewer renders correctly via display forwarding on both
  - Verify SpaceMouse USB passthrough works (Linux: `--device`, Windows: `usbipd-win`)
  - Test on Jetson (native, not Docker — Jetson runs the hardware path in Phase 6)

**Deliverable:** Move the simulated Stewart platform with the spacemouse in real-time. Motion is smooth and respects workspace limits even with aggressive spacemouse input.

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
- **Jetson requires `MUJOCO_GL=egl`.** The Jetson Orin Nano uses NVIDIA's EGL (not GLX). Set `export MUJOCO_GL=egl` before launching the MuJoCo viewer. For headless testing, `MUJOCO_GL=osmesa` with `libosmesa6-dev` also works.
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
