# Jugglebot Codebase Overhaul: Targeted Rewrite of the ROS2 Layer

## Implementation Notes

- **Working on `refactor` branch** — we can modify files in place, no need for a parallel package. The `main` branch remains untouched as a safety net.

## Context

The Jugglebot ROS2 workspace has grown organically around specific behaviors (catching, volley testing, hoop sinker, etc.) resulting in 18 Python nodes (~12,900 lines), scattered constants, monolithic files (`can_interface.py` at 2,351 lines), duplicated nodes (ball_butler + volley_testing are 95% identical), and no motion planner. The system going forward has a fundamentally different shape: 4-6 focused nodes centered around a **motion planner** that doesn't exist today.

**Approach: Targeted rewrite of the ROS2 Python layer.** Write new nodes from scratch, directly in the existing `jugglebot` package on the `refactor` branch, while preserving the Teensy firmware (already solid), the YAML+codegen system (recently refactored and excellent), and lifting battle-tested protocol code from the existing CAN interface. This is not a refactor of 18 nodes — it's replacing them with 4-6 well-designed ones.

### Why rewrite over refactor?
- The current architecture was designed around behaviors (catch thrown ball, catch dropped ball, etc.) that will be subsumed by a motion planner
- 12 of 18 nodes would be deleted or completely rewritten anyway
- The monolithic `can_interface.py` (2,351 lines) is harder to surgically refactor than to rewrite with extracted protocol modules
- We're on the `refactor` branch, so we can freely replace files without risk to the working `main` branch

### Why keep ROS2?
- **rosbag** recording (14 topics) is irreplaceable for debugging a physical robot
- **tf2** for mocap coordinate transforms is already in use
- **Ecosystem tooling** (`ros2 topic echo`, `rqt_graph`) matters on the Jetson
- The Python GIL concern is manageable: hard real-time runs on Teensys (1kHz interrupt-driven), the Jetson only needs soft real-time at 200-500Hz which Python+ROS2 handles adequately
- Treat ROS2 as communication/recording infrastructure, not an application framework — all core logic lives in plain Python classes with thin ROS2 wrappers

---

## Proposed Architecture

### Node Map (4 core + 2 optional)

```
                    ┌─────────────────┐
                    │   Orchestrator   │  BOOT → HOMING → IDLE → ACTIVE → FAULT
                    │   Node           │  Monitors /robot_state, coordinates lifecycle
                    └────────┬────────┘
                             │
              ┌──────────────┼──────────────┐
              │              │              │
    ┌─────────▼───┐  ┌──────▼───────┐  ┌───▼───────────┐
    │ CAN          │  │ Motion       │  │ Mocap         │
    │ Interface    │  │ Planner      │  │ Interface     │
    │ Node         │  │ Node         │  │ Node          │
    └──────────────┘  └──────────────┘  └───────────────┘

    Optional:
    ┌──────────────────┐  ┌──────────────────┐
    │ Spacemouse       │  │ Diagnostic CLI   │
    │ Handler Node     │  │ (terminal tool)  │
    └──────────────────┘  └──────────────────┘
```

### Data Flow

```
Spacemouse/CLI ──► /platform_pose_cmd ──► Motion Planner ──► /leg_lengths ──► CAN Node ──► CAN bus
                                               ▲
QTM (200Hz) ──► Mocap Node ──► /mocap_data ────┘
                             ──► /rigid_body_poses (tf2)

CAN bus ──► CAN Node ──► /robot_state ──► Orchestrator (state machine)
                       ──► /hand_telemetry
                       ──► /bb/heartbeat
```

### Node Responsibilities

**1. CAN Interface Node** (`can_node.py` + `can/` subpackage)
- Bus I/O at 1kHz poll rate
- ODrive protocol: encode commands, decode heartbeats/encoders/temps
- Ball Butler protocol: heartbeat parsing, throw/reload/reset commands
- Time-sync broadcast, inclinometer relay
- Motor state tracking (9 axes), error detection

Split into focused modules:
- `can/bus.py` (~150 lines) — lifecycle, send/recv, reconnection
- `can/odrive.py` (~300 lines) — ODrive message encode/decode, heartbeat, error parsing
- `can/ball_butler.py` (~100 lines) — BB heartbeat, throw commands
- `can/motor_state.py` (~150 lines) — per-axis state tracking
- `can_node.py` (~300 lines) — ROS2 wrapper: pubs, subs, services

**2. Motion Planner Node** (`motion_planner_node.py` + `motion/` subpackage)
The central new piece. Accepts pose commands, validates them, generates smooth trajectories, runs IK, and publishes leg targets.

Core modules:
- `motion/ik_solver.py` (~200 lines) — extracted from current `sp_ik.py`, pure math
- `motion/trajectory.py` (~300 lines) — trajectory generation with time profiles, pre-computes duration so impossible commands can be rejected before execution
- `motion/workspace.py` (~150 lines) — leg extension limits, position bounds, scaffolded for future force/stability analysis
- `motion/geometry.py` (~100 lines) — robot geometry loaded from `hardware_config.yaml`
- `motion_planner_node.py` (~400 lines) — ROS2 wrapper, process management

Initial planner scope:
- Smooth trajectory interpolation (trapezoidal or quintic profiles)
- Pre-compute trajectory duration; reject infeasible commands before execution
- Enforce leg extension limits (initial_length ± stroke)
- Enforce platform acceleration/velocity limits
- **Scaffold** for future additions: force estimation, tip-over prevention, dynamic stability
- The `motion/workspace.py` module is the designated home for all constraint validation — starting with kinematics, extensible to dynamics

**3. Mocap Interface Node** (`mocap_node.py`)
Largely preserved from current `MocapInterface` class (524 lines). Minor cleanup:
- Remove Ball Butler marker publishing logic (simplify). Only complete this after carefully reading through how the ball butler location calibration works. We must maintain that process to ensure that BB can be calibrated. (this process is currently handled by /ball_butler_node)
- Keep QTM async connection, clock sync, rigid body parsing, tf2 broadcast

**4. Orchestrator Node** (`orchestrator_node.py` + `state_machine.py`)
Extensible state machine (~200 lines) using a registry pattern:

Initial states:
- BOOT: wait for all 7 axis heartbeats
- HOMING: trigger encoder search + homing via CAN node services. This is also a good time to automatically run the Ball Butler calibration sequence.
- IDLE: robot ready, waiting for activation command
- ACTIVE: robot activated, accepts pose commands (sub-modes: spacemouse, shell, etc.)
- FAULT: error detected, safe shutdown, configurable timeout for transient errors

**Extensibility design**: Each state is a class implementing `enter()`, `execute()`, `exit()` methods. New states are added by:
1. Adding a value to the `RobotState` enum
2. Writing a state handler class
3. Registering it with `state_machine.register(RobotState.NEW_STATE, NewStateHandler())`

This means adding states like CATCHING, CALIBRATING, etc. later requires zero changes to the state machine framework — just new handler classes.

Error severity classification (transient vs fatal) built in from the start.

**5. Spacemouse Handler** (optional, ~180 lines)
Nearly unchanged from current. Publishes to `/platform_pose_cmd`.

**6. Diagnostic CLI** (optional, not a ROS2 node)
Terminal tool for sending pose commands, querying state, triggering homing. Replaces the SHELL active state.

---

## What to Keep, Adapt, and Discard

### KEEP (unchanged)
| Component | Why |
|-----------|-----|
| Ball Butler Teensy firmware (`Ball Butler/ball_butler_main/`) | Already excellent: interrupt-driven, hard real-time, clean state machine |
| Platform Teensy firmware (`ros_ws/src/jugglebot/Teensy_code/`) | Stable CAN router, inclinometer, time-sync |
| YAML+codegen (`config/protocol_config.yaml`, `config/generate_config.py`) | Recently refactored, single source of truth |
| `jugglebot_interfaces` package (`.msg`, `.srv` definitions) | Stable. Prune unused, add new, but package stays |

### ADAPT (lift core logic into new modules)
| Component | What to extract |
|-----------|----------------|
| `can_interface.py` (2,351 lines) | ODrive encode/decode (~400 lines), BallButlerHeartbeat dataclass, bus reconnection logic, motor state tracking |
| `sp_ik.py` | IK math (pose → leg lengths) becomes `motion/ik_solver.py` |
| `mocap_interface.py` (524 lines) | QTM async connection, clock sync, rigid body parsing — class mostly preserved |
| `robot_geometry.py` | **Obviated** — all geometry data now in `hardware_config.yaml`. Archived. IK solver will import directly from `hardware_config.py`. |
| `spacemouse_handler.py` (178 lines) | Minor changes (new topic name, remove control_mode gating) |
| `can_interface_node.py` (806 lines) | Control mode switching logic (lines 280-325), activate/deactivate sequences — important domain knowledge to preserve |

### DISCARD (archived, do not port)
22 files moved to `ros_ws/src/jugglebot/jugglebot/archived/` — see Appendix A for full list.

---

## Phased Implementation

### Phase 0: Preparation — DONE (2026-02-19)
- [x] Write `CODEBASE_REWRITE_PLAN.md` at project root; delete obsolete `CONFIG_REFACTOR_PLAN.md` and `JUGGLEBOT_REFACTOR_PLAN.md`
- [x] Create `hardware_config.yaml` in `config/` — all 17 sections with descriptive comments sourced from original files
- [x] Extend `config/generate_config.py` to generate `hardware_config.h` and `hardware_config.py` from the new YAML
  - Generator outputs to `config/generated/` and copies to all consumer directories (BB firmware, Teensy firmware, ROS2 package)
  - Derived constants computed: `GRAVITY_MMPS2`, `CATCH_HEIGHT_THROWN/DROPPED_MM`, `INIT_LEG_LENGTHS_WITH_OFFSET_MM`, `TEENSY_LINEAR_GAIN`, `BB_LINEAR_GAIN`, `BB_MAX_THROW_SAMPLES`, `BB_MAX_TRAJ_FRAMES`
  - Verify: `python config/generate_config.py` succeeds and produces matching values
- [x] Restructure the existing `jugglebot` package in place — created `can/`, `motion/`, `archived/` subdirectories with `__init__.py`
- [x] Archive discarded nodes into `archived/` directory (22 files moved via `git mv`, including `robot_geometry.py`)
- [x] Create `config/compute_geometry.py` — standalone script that recomputes base/platform node positions from parametric inputs. Run with `--update` to regenerate `hardware_config.yaml` if platform dimensions change. Preserves the `build_platform()` algorithm from the archived `robot_geometry.py`.

**Target package structure** (in-place replacement):
```
ros_ws/src/jugglebot/
  jugglebot/
    can/
      __init__.py
      bus.py
      odrive.py
      ball_butler.py
      motor_state.py
    motion/
      __init__.py
      ik_solver.py
      trajectory.py
      workspace.py
      geometry.py
    state_machine.py
    can_node.py
    motion_planner_node.py
    mocap_node.py
    orchestrator_node.py
    spacemouse_node.py
    archived/              # Old nodes kept for reference
      can_interface.py
      can_interface_node.py
      yasmin_state_machine.py
      catch_thrown_ball_node.py
      ... etc
  launch/
    jugglebot_launch.py    # Rewritten for new nodes
  setup.py                 # Updated entry points
  package.xml
```

### Phase 1: CAN Interface Node — DONE (2026-02-19)
- [x] Extract ODrive encode/decode from `can_interface.py` into `can/odrive.py`
- [x] Extract BB protocol into `can/ball_butler.py`
- [x] Extract motor state tracking into `can/motor_state.py`
- [x] Write `can/bus.py` for bus lifecycle and reconnection
- [x] Write `can_node.py` as ROS2 wrapper
- [x] **Critical**: Port the control mode switching logic from `can_interface_node.py` lines 280-325 and the activate/deactivate sequences
- [x] **Test**: Verify heartbeat reception, motor state reporting, and basic commands against real hardware

### Phase 2: State Machine + Orchestrator — DONE (2026-02-20)
- [x] Write `state_machine.py` (~200 lines of logic, plain Python, extensible registry pattern)
- [x] Implement initial states: BOOT, HOMING, IDLE, ACTIVE, FAULT
  - ACTIVE state supports sub-modes (spacemouse, shell) from the start
- [x] Write `orchestrator_node.py` with startup sequence: wait for heartbeats → encoder search → homing → IDLE
- [x] Implement error severity classification (transient vs fatal) from day one
- [x] FAULT state with safe motor disable via 'ERROR' control mode
- [x] **Test**: Full power-on → homing → IDLE → ACTIVE (spacemouse) sequence on real hardware

### Phase 3: Motion Planner — IN PROGRESS
#### Motion Planner Phase 1 (Kinematic Foundation) — DONE (2026-02-20)
- [x] Extract IK math from `sp_ik.py` into `motion/ik_solver.py` (position IK, velocity IK via Jacobian, acceleration IK with bias term, numerical FK)
- [x] Load geometry from generated `hardware_config.py` via `motion/geometry.py`
- [x] Implement `motion/workspace.py`: leg extension limits, condition number, reachability, singularity mapping
- [x] Implement `motion/conversions.py`: leg force ↔ motor torque, mm ↔ rev conversions
- [x] Add `set_input_torque` (0x0E) to protocol config + `can/odrive.py`

#### Motion Planner Phase 2 (Control Process & IPC) — DONE (2026-02-25)
- [x] Implement `motion/ipc.py`: ZeroMQ PUB/SUB IPC layer with msgpack serialization
- [x] Implement `motion/control_loop.py`: standalone fixed-rate control process with timing instrumentation and heartbeat watchdog
- [x] Write `motion_bridge_node.py`: ROS2 ↔ IPC bridge (subscribes to pose commands, publishes leg lengths)
- [x] Phase 1 verification tests (6 tests, all PASS) — `motion/tests/test_kinematics.py`
- [x] Phase 2 verification tests (3 tests) — `motion/tests/test_control_loop.py`
- [x] Standalone single-leg test harness (`tools/single_leg_test.py`) — all 4 bench tests PASS on hardware (2026-02-25)
- [x] Loop timing + IPC latency tests

#### Motion Planner Phase 3+ (Dynamics, Trajectory, Hardening) — IN PROGRESS
- [x] Implement `motion/trajectory.py`: smooth trajectory generation with pre-computed durations
  - Quintic polynomial solver respecting per-leg velocity/acceleration limits
  - Duration estimation: compute exact time required, reject infeasible commands before starting
- [x] Gravity compensation & static feedforward (Phase 3 of MOTION_PLANNER_PLAN)
- [x] Full inertia feedforward & dynamic compensation (Phase 5 of MOTION_PLANNER_PLAN)
- [x] Hardening & operational readiness (Phase 6 of MOTION_PLANNER_PLAN)
  - Workspace limits, singularity monitoring, fault detection — completed 2026-03-01
  - **Post-incident safety hardening (2026-03-11):** After a broken actuator caused by a step discontinuity, added defense-in-depth motor command safety: slew rate limiter (9.5 rev/s against actual motor feedback), motor feedback gating (no feedback → no commands), feedback staleness check (100ms), motor overspeed fault, tracking error fault (10mm), sustained slew fault (0.5s → ESTOP), lead-time gate (300ms minimum for dynamic targets), and bridge motor feedback forwarding. 8 offline tests in `test_safety.py`. See [docs/motion_planner/safety.md](docs/motion_planner/safety.md) for full documentation.
- [ ] Absorb `gently_move_platform_to_setpoint()` as a trajectory-planned move
- [ ] Absorb platform leveling as a planner method
- [ ] **Test**: Spacemouse control through new pipeline. A/B compare with old system

#### Mocap Gravity Alignment Check (Placeholder)
Platform levelling includes a placeholder for verifying mocap alignment with gravity.
When mocap integration is complete, update the LEVELLING state to:
- Read platform rigid body pose from `rigid_body_poses` topic (body name: "platform")
- Extract platform z-axis from rotation matrix
- Compute angle between platform z and world z ([0,0,1])
- Log WARNING if angle > 1.0 deg (indicates mocap global frame misaligned with gravity)

### Phase 4: Mocap Integration
- [ ] Lift `MocapInterface` class with minor cleanup into new `mocap_node.py`
- [ ] Keep tf2 static broadcast, QTM clock sync
- [ ] **Keep BB marker subscription** (`bb/markers`) — needed for calibration. Do NOT remove.
- [ ] Implement BB calibration position publisher: after the `bb/calibrate` service completes, the mocap node (or a dedicated calibration module) must collect BB fiducial marker trajectories during the calibration motion, fit rotation axes via 3D circle fitting to determine the BB's global position and yaw offset, and publish the result so the GUI and aiming code can use it. The archived `ball_butler_node.py` (see `archived/ball_butler_node.py`) contains the full algorithm: 5 markers → circle fit → weighted axis intersection → `bb_mocap_position` [x,y,z] + `bb_yaw_offset_rad`. The key outputs are:
  - BB global position (mm): where the yaw rotation axis intersects the marker Z-plane
  - BB yaw offset (rad): angular offset between BB's local frame and the mocap global frame
  - These were previously stored as ROS2 parameters on the `ball_butler_node`; in the new architecture they should be published on a latched topic (e.g. `bb/calibration_result`) so the GUI and any future aiming node can subscribe
- [ ] **Test**: Verify `/mocap_data` and `/rigid_body_poses` publish correctly

### Phase 5: Integration + Polish
- [x] Write new `jugglebot_launch.py` for the new node set
  - Added `motion_bridge_node` and `control_loop` to launch description
  - `control_loop` registered as console_scripts entry point in `setup.py`, launched via `ExecuteProcess`
  - Fixed ZeroMQ bind addresses (`localhost` → `127.0.0.1`) for Jetson compatibility
  - Added `/motion/tracking_error`, `/motion/motor_feedback`, `/motion/diagnostics` topics + rosbag recording
- [ ] Full system test: power on → homing → ACTIVE (spacemouse control) → shutdown
- [ ] Verify rosbag recording
- [ ] Clean up `archived/` directory (remove or keep as reference)
- [x] ~~Remove YASMIN vendored package (`ros_ws/src/yasmin/`)~~ — DONE (2026-03-12)
- [ ] Remove rosbridge dependency if no longer needed

### Phase 6 (Future): Advanced Features
- [ ] **Ball prediction → motion planner integration**: Connect the ball predictor to the motion planner's dynamic target API (`submit_dynamic_target` / `request_dynamic_target`). The predictor outputs `(target_pos, target_quat, target_vel, arrival_time)` and the planner handles feasibility checking, trajectory generation, mid-motion replanning (C2 continuity), and auto-return-to-home. Key sub-tasks:
  - [ ] Ball predictor node: processes mocap ball tracking data → predicts intercept point/time/velocity
  - [ ] Bridge between ball predictor output and motion planner IPC (`TOPIC_DYN_TARGET` / `make_dynamic_target_command()`)
  - [ ] Mode sequencing in orchestrator: IDLE → TRACKING → catching → return-to-home → IDLE
  - [ ] Timeout handling: if no feasible target arrives within deadline, return to IDLE
  - [ ] Graceful target updates: re-plan from current state when prediction improves as ball approaches
  - [ ] Validation with synthetic ball trajectories before live predictor
  - [ ] End-to-end timing accuracy test (planned arrival vs actual)
- [ ] Re-add Ball Butler aiming/coordination (depends on Phase 4 BB calibration position publisher — `bb/calibration_result` provides the `bb_mocap_position` and `bb_yaw_offset_rad` needed by `global_to_bb_frame()` for aim calculations)
- [ ] Add force estimation + stability analysis to `motion/workspace.py`
- [ ] Pose correction (feedforward from mocap)

---

## Line Count Comparison

| | Current | Proposed |
|---|---------|----------|
| ROS2 nodes | 18 entry points | 4-6 entry points |
| Python lines | ~12,900 | ~2,500-3,000 |
| State machine | 11+ states (legacy framework + 4 vendored packages) | 5 initial states, extensible (~200 lines) |
| CAN interface | 1 monolithic file (2,351 lines) | 4 focused modules (~800 lines) |
| Dependencies | ROS2 + rosbridge | ROS2 only |
| Scattered constants | 8+ files | Single `hardware_config.yaml` |

---

## Key Risks and Mitigations

| Risk | Mitigation |
|------|-----------|
| CAN protocol subtleties lost in translation | **Extract** low-level encode/decode code verbatim, don't reimagine it. Port `attempt_to_restore_can_connection()` directly. |
| Motion planner is the hardest new piece | Start with simplest planner (IK + rate limiting + leg limits). The current system has NO planner — even a basic one is an improvement. |
| Loss of rosbag compatibility | Keep same topic names where possible. Document any name changes. |
| Regression during rewrite | Working on `refactor` branch — `main` branch preserved as fallback. Old nodes archived, not deleted. |
| Spacemouse control regression | Phase 3 explicitly includes A/B testing between old and new pipelines. |
| Control mode switching logic lost | Read `can_interface_node.py` lines 280-450 carefully in Phase 1 — this encodes which axes need which ODrive states for each control mode. |

---

## Critical Source Files Reference

Files to extract from (now in `archived/`):
- `can_interface.py` (2,351 lines) — ODrive protocol, BB heartbeat, motor state
- `can_interface_node.py` (806 lines) — control mode switching, activate/deactivate
- `sp_ik.py` — IK math (still active, not yet archived)
- `mocap_interface.py` (524 lines) — QTM interface (still active)
- `spacemouse_handler.py` (178 lines) — minor adaptation needed (still active)

Config infrastructure:
- `config/protocol_config.yaml` + `config/hardware_config.yaml` — sources of truth
- `config/generate_config.py` — generates C++/Python from both YAMLs, copies to all consumers

---

## Verification Plan

After each phase:
- **Phase 0**: `python config/generate_config.py` succeeds, generated files match current values
- **Phase 1**: CAN node receives heartbeats from all 9 axes, can send commands, BB heartbeat publishes on `/bb/heartbeat`
- **Phase 2**: Robot completes BOOT → HOMING → IDLE on power-on. Error injection triggers FAULT correctly
- **Phase 3**: Spacemouse moves the platform smoothly. Infeasible pose commands are rejected with clear feedback. Trajectory durations match expectations
- **Phase 4**: `/mocap_data` publishes at 200Hz, tf2 transforms correct
- **Phase 5**: `ros2 launch jugglebot jugglebot_launch.py record:=true` — full session with rosbag recording

---

## Appendix A: Phase 0 Completion Notes (2026-02-19)

### Config files
- `config/hardware_config.yaml` — 17 self-documented sections centralizing all hardware/geometry constants. See the file itself for structure and values.
- `config/generate_config.py` — generates `hardware_config.h` / `hardware_config.py` (and `protocol_config.h` / `protocol_config.py`), computes derived constants, and copies to all consumer directories.
- `config/compute_geometry.py` — recomputes base/platform node positions from parametric inputs. Run with `--update` to regenerate `hardware_config.yaml`.

### Discrepancies resolved during constant extraction
- **`hand_stroke`**: Unified Teensy value from 0.358 m to 0.355 m (matching geometric measurement). BB value remains 0.28 m (different hardware).
- **`initial_height_mm`**: Canonical value is 574.3 mm (experimentally calibrated). Stale 565.0 mm value was in archived catch nodes.
- **Gravity**: Canonical value is 9.806 m/s^2 everywhere. Several Python files had 9.81.
- **`ball_check_sample_interval_ms`**: Firmware uses 100 ms, not 250 ms.

### Files archived (22 total)
Moved to `ros_ws/src/jugglebot/jugglebot/archived/` via `git mv`:
- 15 discarded nodes, 2 monolithic files to rewrite (`can_interface.py`, `can_interface_node.py`), 1 obviated (`robot_geometry.py`), 4 supporting modules

### Known issues to fix during rewrite
- **Time arithmetic bug** in `catch_thrown_ball_node.py`: manual sec/nanosec subtraction can underflow
- **Infinite service wait loops** in multiple nodes: add maximum retry count
- **BB reload sequence** in `StateMachine.cpp` has 12 sub-states: consider collapsing where axes can move in parallel

---

## Appendix B: Phase 1 Completion Notes (2026-02-19)

### Architecture: monolith → 4 focused modules + ROS2 node

The 2,351-line `can_interface.py` and 806-line `can_interface_node.py` have been replaced by:

| File | Lines | Purpose |
|------|-------|---------|
| `can/motor_state.py` | ~90 | Thread-safe per-axis motor state tracking |
| `can/odrive.py` | ~230 | ODrive CAN protocol encode/decode (struct.pack, no cantools/DBC) |
| `can/ball_butler.py` | ~100 | Ball Butler heartbeat parsing + command encoding |
| `can/bus.py` | ~140 | CAN bus lifecycle, send/recv, time sync, reconnection |
| `can_node.py` | ~620 | ROS2 node with all pubs/subs/services/actions |
| **Total** | **~1,180** | Down from ~3,157 (old combined total) |

### Key design decisions

1. **Dropped cantools/DBC dependency**: The old code used `cantools` with an `ODrive_Pro.dbc` file for encoding outgoing commands. Since every ODrive CAN message is a simple little-endian struct, encoding via `struct.pack` is simpler, faster, and eliminates a dependency. The DBC file (`resources/ODrive_Pro.dbc`) is still present in the package but no longer loaded at runtime.

2. **Explicit leg inversion**: The ODrive uses negative motor positions for leg extension. Previously, this inversion was buried inside `send_position_target()` and `_handle_encoder_estimates()`. Now: `odrive.py` is a pure protocol module (no Jugglebot-specific conventions); leg inversion is done explicitly in `can_node.py`'s `_send_position_target()` and `_handle_encoder()` methods.

3. **python-can import alias**: Since the `jugglebot.can` subpackage shadows the `can` namespace, `can_node.py` imports `import can as python_can` to disambiguate.

4. **Class-level handler dispatch table**: `can_node.py` uses a class-level dict `_odrive_handlers` mapping command IDs to unbound methods. These are called as `handler(self, axis_id, data)` — an intentional Python pattern that avoids per-instance dict construction and keeps the dispatch table compact.

5. **Generator-based state machines for async operations**: All long-running operations (encoder search, homing, activate/deactivate, reboot, tilt reading, encoder status check) are implemented as Python generators. Each `yield` is a state transition: `yield None` for tight CAN polling, `yield <seconds>` for timed waits with CAN processing. A single `_run_to_completion(gen)` driver processes CAN messages between yield points, ensuring the bus never goes silent during multi-second operations. Generators compose naturally via `yield from` (e.g. `_home_robot_steps` delegates to `_home_motor_steps` for each axis). This is pure Python — no ROS2 multi-threaded executor, no asyncio — and the generators can be trivially driven by a timer instead of synchronously when services are later converted to action servers.

### What was ported verbatim
- **Error handling logic** (`_handle_error`): The complete error classification from `can_interface.py` lines 1585-1722 is preserved — soft-error clearing, fatal error detection, undervoltage handling, per-axis per-error throttled logging.
- **Control mode switching** (`_sub_control_mode`): Exact port of `can_interface_node.py` lines 280-325 — which modes need legs-only vs all-axes in CLOSED_LOOP.
- **Activate/deactivate** (`_gently_move_to_setpoint`): Full port of the gentle setpoint approach with velocity limiting, trajectory completion wait, and proper state transitions.
- **Homing sequence** (`_home_robot`, `_run_motor_until_current`): Motor-by-motor homing with current-limit detection and exponential moving average.
- **Ball Butler heartbeat** (`BallButlerHeartbeat` dataclass): Verbatim, including CAN frame layout and resolution constants.
- **Teensy communication**: State persistence (`_update_teensy_state`), tilt reading (`_get_tilt_reading`), hand trajectory commands, time sync broadcast.

### What was dropped
- `matplotlib` plotting of hand trajectory data (`_plot_hand_traj_data`) — debug-only, caused the node to freeze
- `cantools` / DBC file dependency — replaced with direct `struct.pack`
- Unused `ament_index_python` import for DBC file lookup
- Callback registration pattern in CANInterface — replaced with direct method calls

### Bugs fixed from old code
- **Infinite tilt reading loop**: `_get_tilt_reading` now has a max retry count (5) and returns a default on failure, instead of looping forever.
- **False target-reached on startup**: ROS2 message `float32` fields default to `0.0` (never `None`), so the old `None` guard was dead code. Added `_encoder_data_received` tracking to prevent false positives before real encoder data arrives.
- **Trajectory-done check includes hand axis**: `_gently_move_to_setpoint` now only waits for legs (0-5) trajectory completion, not the hand (axis 6) which may not have received a command.
- **Stale state in error handler**: `_handle_error` now calls `get_states()` to refresh the snapshot immediately after updating error fields, ensuring the error classification logic sees current data.
- **Blocking sleep in error recovery**: `_handle_error` had a `time.sleep(0.5)` after soft-error clearing. Removed — the handler is reactive (each incoming CAN error message re-triggers it), so waiting is unnecessary.

### Safety hardening (post-audit)
- **Emergency idle on generator exceptions (3a)**: `_run_to_completion()` now catches unhandled exceptions with `try/except`, sends all Jugglebot axes to IDLE via `_emergency_idle()`, and sets `fatal_error = True` before re-raising. Prevents motors being left in CLOSED_LOOP with no position command after unexpected errors.
- **CAN bus disconnection detection (3c)**: Heartbeat watchdog added via `MotorStateTracker.record_heartbeat()` timestamps. A 1 Hz timer (`_watchdog_check`) triggers after 2s without any axis heartbeat. On detection: attempts `bus.attempt_restore()` (3 internal retries); sets `fatal_can_error` on failure. During generator operations, `_pump()` performs a lightweight staleness check and sets the flag so generators can abort promptly.  Watchdog only activates after the first heartbeat is received (avoids false alarms before ODrives power on).
- **Heartbeat gate on ODrive configuration (3d)**: `_setup_odrives()` converted to generator `_setup_odrives_steps()` that waits for heartbeats from all Jugglebot axes (5s timeout) before sending configuration commands. Prevents fire-and-forget configuration against rebooting or unresponsive hardware.
- **Homing cleanup on failure**: `_home_motor_steps()` now sends IDLE to the specific axis on fatal error, preventing motors from being left in velocity mode. Also checks `fatal_can_error` alongside `fatal_error`.
- **Reboot watchdog suppression**: `_reboot_odrives_steps()` resets heartbeat tracking before the 10s wait, preventing false watchdog triggers during ODrive reboot.

### Items for investigation during Phase 2+
- **~~`time.sleep()` blocking the executor~~** — **RESOLVED**: All long-running operations now use generator-based state machines driven by `_run_to_completion()`. The driver processes CAN messages between yield points (1ms polling during timed waits, immediate re-entry for tight polls). Service callbacks still block the single-threaded executor for the duration of the operation, but CAN traffic flows continuously. The remaining `time.sleep()` calls are 1-5ms CAN bus pacing delays between consecutive sends to avoid buffer overflow — these are intentional and harmless. If true non-blocking services are needed later, the generators can be driven by a timer instead of synchronously (the generator protocol supports this without code changes to the generators themselves).
- **Shallow-copy thread safety**: `get_states()` does a shallow list copy — readers and writers share the same `MotorStateSingle` objects. This works in CPython due to GIL-atomic attribute access but is technically a race condition. Not worth fixing unless moving to a multi-threaded executor.
- **~~DBC file cleanup~~** - **RESOLVED**: The `resources/ODrive_Pro.dbc` file has been deleted.
- **rigid_body_poses subscriber**: The old `can_interface_node.py` had a subscriber for `/rigid_body_poses` to check that the QTM base is at the origin. This was a safety check that has been deferred — it should be re-added in the orchestrator node or mocap node (Phase 4).

---

## Appendix C: Phase 2 Completion Notes (2026-02-20)

### Architecture: Legacy framework → lightweight registry-based state machine

The 11+ state legacy framework (4 vendored sub-packages, Blackboard, MonitorState, ServiceState, ActionState) was replaced by:

| File | Lines | Purpose |
|------|-------|---------|
| `state_machine.py` | ~320 | Pure Python state machine: enum, context, handler base class, 5 handlers, factory |
| `orchestrator_node.py` | ~310 | ROS2 node: bridges state machine to services/topics/actions |
| **Total** | **~630** | Down from ~1,500+ (legacy SM + 4 vendored packages + node wrapper) |

### Key design decisions

1. **Pure Python state machine (no ROS2 dependency)**: `state_machine.py` imports only `time` and `enum`. All ROS2 interaction is in the orchestrator node. This means the state machine is testable without a running ROS2 system — just instantiate a `Context`, a `StateMachine`, and call `tick()`.

2. **Registry pattern for extensibility**: Each state is a `StateHandler` subclass with `on_enter`/`execute`/`on_exit` methods. Adding a new state (e.g., CATCHING, CALIBRATING) requires:
   - Add a value to `RobotState` enum
   - Write a handler class
   - Call `sm.register(RobotState.NEW_STATE, handler)`
   No changes to the state machine framework or orchestrator node.

3. **Async-first orchestrator**: All CAN node interactions are non-blocking. Service calls use `call_async()`, action calls use `send_goal_async()`. The orchestrator polls futures in its 10 Hz tick. This avoids blocking the executor during multi-second operations (homing takes 30+ seconds, activation ~5 seconds).

4. **Request/response pattern between handlers and orchestrator**: State handlers express intent by setting `ctx.request` (e.g., `'encoder_search'`, `'activate'`, `'deactivate'`). The orchestrator translates these to ROS2 service/action calls. Handlers check `ctx.operation_result` for completion. This keeps domain logic in the handlers and ROS2 plumbing in the orchestrator.

5. **Error detection via /robot_state typed fields**: The orchestrator monitors the CAN node's `/robot_state` topic. Error classification uses typed boolean fields (`has_fatal_odrive_error`, `has_fatal_can_error`, `has_undervoltage`) rather than string parsing. The `string[] error` field is retained for human-readable logging. Any non-empty error list triggers FAULT.

6. **Force-transition with cleanup**: When errors are detected, the orchestrator force-transitions to FAULT and cancels any pending async operations. This prevents stale operation results from leaking into future states.

### State flow

```
BOOT ──────────────> HOMING ──────────────> IDLE <══════════> ACTIVE
  ▲  all heartbeats    │  encoder_search     ▲   activate      │
  │  (if !homed→HOMING │  home_motors         │   deactivate    │
  │   if homed→IDLE)   │  bb_calibrate(opt)   │                 │
  │                    │                      │                 │
  ╰────────────────────╰──────────────────────╰─────────────────╯
  │                           │                                 │
  │ errors cleared            ▼          errors detected        │
  ╰──────────────────── FAULT  <────────────────────────────────╯
                              │     (from any state)
```

### What changed from the old architecture

| Aspect | Old | New |
|--------|-----|-----|
| Framework | Legacy framework + 4 vendored packages | ~90 lines of StateMachine class |
| States | 7+ states (Boot, EncoderSearch, Homing, StandbyIdle, GenericActive x8, LevelPlatform, Fault) | 5 states (BOOT, HOMING, IDLE, ACTIVE, FAULT) |
| Async ops | ServiceState/ActionState (blocking, multi-threaded) | Non-blocking futures polled in tick loop |
| Error detection | Separate RobotStateSynchronizer thread with mutex | Single-threaded: orchestrator checks /robot_state each tick |
| Command topics | 2 topics (standby_command, active_command) | 1 topic (orchestrator_command) |
| ACTIVE sub-modes | 8 separate GenericActiveState instances | Single ActiveHandler with ActiveMode enum |
| State machine comms | Blackboard (shared dict) | Context object (typed attributes) |

### HOMING sequence details

The HomingHandler runs three phases in sequence:
1. **encoder_search**: Calls `encoder_search` Trigger service on CAN node. Drives all leg motors to find encoder index pulses.
2. **home**: Calls `home_motors` HomeMotors action on CAN node. Drives each motor to its end-stop using current-limit detection, then sets absolute position.
3. **bb_calibrate** (optional): Calls `bb/calibrate` Trigger service. If Ball Butler is not connected (service not available), this phase is skipped automatically.

If encoder search was already complete (checked via /robot_state), the handler skips directly to homing.

### ROS2 interface

**Topics published:**
- `control_mode_topic` (String) — tells CAN node which axis configuration to use
- `orchestrator_state` (String) — current state name for monitoring

**Topics subscribed:**
- `robot_state` (RobotState) — heartbeats, homing status, errors from CAN node
- `orchestrator_command` (String) — user commands: `activate`, `deactivate`, `spacemouse`, `shell`, `home`, `clear_errors`

**Service clients:**
- `encoder_search` (Trigger) — encoder index search
- `activate_or_deactivate` (ActivateOrDeactivate) — platform activation/deactivation
- `odrive_command` (ODriveCommandService) — error clearing
- `bb/calibrate` (Trigger) — Ball Butler calibration

**Action clients:**
- `home_motors` (HomeMotors) — full homing sequence

### Items for investigation during Phase 3+

- **~~Error string matching is fragile~~** — **RESOLVED**: Added typed boolean fields to `RobotState.msg` (`has_fatal_odrive_error`, `has_fatal_can_error`, `has_undervoltage`). The CAN node sets these directly from its internal motor state tracker flags. The orchestrator reads the booleans instead of parsing error strings. The `string[] error` field is retained for human-readable logging and rosbag inspection. If new error categories are added in the future, the pattern is: add a `bool` field to the message, set it in `can_node.py:_publish_robot_state()`, and read it in `orchestrator_node.py:_on_robot_state()` — the coupling is now explicit and compile-time visible.

- **rigid_body_poses origin check**: Still deferred from Phase 1. The old code validated that the QTM base rigid body was at the origin (within 5 mm, 2 degrees) before allowing activation. This should be added to the ACTIVE handler's activation sequence or to the mocap node in Phase 4.

- **~~Shutdown race condition~~** — **RESOLVED**: Replaced `spin_until_future_complete()` with fire-and-forget ERROR mode publish. The CAN node handles ERROR mode by stowing the platform and idling all axes, achieving the same safety outcome without the race condition. The CAN node's own `on_shutdown()` also stows as a belt-and-suspenders safety measure.

- **Single pending operation**: The orchestrator tracks only one pending service call and one pending action at a time. This is sufficient for the current sequential flow (HOMING phases run one at a time, ACTIVE has one activate/deactivate). If future states need concurrent operations, the tracking would need to be extended.

- **~~FAULT timeout not implemented~~** — **RESOLVED**: Removed unused `FAULT_TIMEOUT_S` constant and `ctx.fault_entry_time` from state_machine.py. The 'ERROR' control mode immediately triggers CAN node to stow and IDLE all axes, which is sufficient. If an ODrive reboot timeout is needed later, it can be added to FaultHandler.

### Post-Phase 2 fixes (2026-02-20)

Seven issues identified during Phase 2 code review, all resolved:

1. **Pending command single-slot drop** (state_machine.py, orchestrator_node.py): Commands arriving between 10 Hz ticks could silently overwrite each other, and unrecognized commands persisted in the buffer indefinitely. Fix: `_on_command()` now warns when overwriting an unconsumed command. Each state handler consumes commands eagerly — `consume_command()` is called unconditionally, and only recognized commands trigger actions. Unrecognized commands are silently discarded (already logged on receipt by the orchestrator).

2. **No BOOT timeout** (state_machine.py, orchestrator_node.py): BootHandler waited indefinitely for ODrive heartbeats. Fix: Added `BOOT_TIMEOUT_S = 30.0` constant. BootHandler transitions to FAULT after timeout, setting `ctx.boot_timed_out`. FaultHandler respects this flag — stays in FAULT until heartbeats arrive (auto-recovery) or operator sends `clear_errors`. The orchestrator logs a clear error message on first boot timeout occurrence.

3. **Missing `__init__` on HomingHandler** (state_machine.py): `self._phase` was set in `on_enter` but never declared in `__init__`, risking `AttributeError` if `execute` were called directly (e.g., in tests). Fix: Added `__init__` with `self._phase = 'encoder_search'`. (ActiveHandler already had `__init__`.)

4. **Dead control mode names** (can_node.py): `_sub_control_mode` handled 6 legacy mode names (`STANDBY_ACTIVE`, `CATCH_THROWN_BALL_NODE`, `LEVEL_PLATFORM_NODE`, `HOOP_SINKER`, `CATCH_FROM_BALL_BUTLER`, `CALIBRATE_PLATFORM`) that the new orchestrator never publishes. Fix: Removed all dead mode names. Only valid modes remain: `''`, `'ERROR'`, `'SPACEMOUSE'`, `'SHELL'`.

5. **Unused `_check_encoder_search_status`** (can_node.py): Method and its generator `_encoder_status_steps` were never called — the orchestrator relies on the CAN node's `encoder_search_complete` flag set during the encoder search service. Fix: Removed both methods. The SDO response handler and `encoder_search_feedback` field on MotorStateTracker remain (protocol-level, may be useful for future diagnostics).

6. **Unused `FAULT_TIMEOUT_S` and `fault_entry_time`** (state_machine.py): Constant and context field were defined but never read. Fix: Removed both. See "FAULT timeout not implemented" item above.

7. **Shutdown race condition** (orchestrator_node.py): `on_shutdown()` called `rclpy.spin_until_future_complete()` which could fail if the node was partially destroyed. Fix: Replaced with fire-and-forget ERROR mode publish. See "Shutdown race condition" item above.

---

## Appendix D: Phase 3 Completion Notes — Motion Planner Phases 1-2 (2026-02-20)

### Architecture: pure Python motion subpackage + standalone control process

The `motion/` subpackage is pure Python + numpy with no ROS2 dependency. The real-time control process runs as a standalone Python process communicating with ROS2 via ZeroMQ IPC. A thin bridge node translates between ROS2 topics and IPC messages.

| File | Lines | Purpose |
|------|-------|---------|
| `motion/geometry.py` | ~65 | `StewartGeometry` class — loads all platform constants from `hardware_config.py` |
| `motion/ik_solver.py` | ~270 | Position/velocity/acceleration IK, analytical Jacobian, numerical FK (Newton-Raphson), rotation utilities |
| `motion/workspace.py` | ~130 | Leg extension checking, condition number, reachability, singularity mapping |
| `motion/conversions.py` | ~70 | Leg force ↔ motor torque, extension ↔ rev, velocity conversions |
| `motion/ipc.py` | ~190 | ZeroMQ PUB/SUB IPC with msgpack; `ControlProcessIPC` + `BridgeIPC` classes |
| `motion/control_loop.py` | ~230 | Fixed-rate standalone control process with timing instrumentation, heartbeat watchdog |
| `motion_bridge_node.py` | ~170 | ROS2 bridge: subscriptions → IPC → publishers |
| `motion/tests/test_kinematics.py` | ~260 | 6 Phase 1 verification tests |
| `motion/tests/test_control_loop.py` | ~170 | 3 Phase 2 verification tests |
| `motion/__init__.py` | ~40 | Public API exports |

### Key design decisions

1. **No ROS2 in motion/**: All kinematic, conversion, and IPC code is plain Python. This keeps the control loop independent of ROS2 lifecycle and threading, and makes everything testable without a running ROS2 system.

2. **ZeroMQ PUB/SUB IPC**: The control process subscribes on `tcp://localhost:5555` (targets and mode commands from the bridge) and publishes on `tcp://localhost:5556` (telemetry back to the bridge). Messages use msgpack serialization with topic-prefix framing (e.g., `b'target'`, `b'mode'`, `b'telem'`).

3. **Heartbeat watchdog**: The control loop E-stops if no IPC messages arrive for 500 ms, preventing runaway operation if the bridge node dies.

4. **Jacobian convention**: `J` maps `[vx, vy, vz, wx, wy, wz]` → `[q_dot_1..q_dot_6]`. This is a 6×6 matrix where row `i` corresponds to leg `i` and columns 0-2 are translational, 3-5 are rotational.

5. **Torque passthrough**: Added `set_input_torque` (command 0x0E) to `protocol_config.yaml` and `encode_set_input_torque()` to `can/odrive.py`. This enables direct torque control of ODrives, replacing the TRAP_TRAJ position commands for future dynamics-based control.

6. **Spool radius derivation**: `spool_radius_mm = 1.0 / (2π × mm_to_rev)` — derived from the `GEOM_MM_TO_REV` constant in `hardware_config.py`, yielding ~11 mm. This is the fixed geometric ratio for force ↔ torque conversion.

### Phase 1 verification results (all PASS)

| Test | Result | Error | Notes |
|------|--------|-------|-------|
| Regression vs sp_ik.py | PASS | 0.00e+00 mm | Exact match with legacy implementation at 20 poses |
| Numerical Jacobian | PASS | 1.11e-06 | Analytical vs finite-difference at 30 random poses |
| Round-trip twist integration | PASS | 2.19e-05 mm/s | Twist → leg velocities → integrate → verify pose |
| Bias term (Jdot · twist) | PASS | 1.71e-06 mm/s² | Analytical vs numerical Jacobian time-derivative |
| FK round-trip | PASS | 0.00e+00 | IK → FK (Newton-Raphson) → compare at 20 poses |
| Singularity map | INFO | — | 929/1944 poses reachable; cond(J) range 449-644 |

### Phase 2 verification results — software-only

Software-only tests run on Windows dev machine. **1 of 3 passed; 2 skipped** due to missing Jetson-only dependencies (`pyzmq`, `msgpack`). The skipped tests are Phase 2 exit gates for the IPC layer and must pass on the Jetson before the control loop is used in production.

| Test | Result | Notes |
|------|--------|-------|
| Loop timing | SKIP — **must pass on Jetson** | Requires pyzmq/msgpack. Exit gate: p99 jitter < 2× nominal period |
| IPC latency | SKIP — **must pass on Jetson** | Requires pyzmq/msgpack. Exit gate: round-trip < 1 control cycle |
| Force conversion (round-trip) | PASS (<1e-14) | Round-trip force/torque conversion; spool radii ~11 mm |

### Phase 2 verification results — hardware bench tests (2026-02-25)

All four isolated-leg bench tests passed on ODrive axis 0 using `tools/single_leg_test.py` (standalone, no ROS2). Current limit set to 50% of rated (10A). Leg bench-mounted, not connected to platform.

| Test | Result | Notes |
|------|--------|-------|
| Torque passthrough smoke test | PASS | 0.075 Nm to overcome friction. 4.15 rev over 2s, clean stop on IDLE |
| Emergency stop (single leg) | PASS | IDLE in 60-88 ms (4 runs), no errors, no residual velocity |
| Encoder sign convention | PASS | +torque → +encoder (retraction). Opposite for -torque. `can_node.py` inversion correct |
| Force conversion (multi-weight) | PASS | 4 weights (1.25-2.75 kg), R^2=0.994, \|Kt\|=0.0624 vs datasheet 0.0637 Nm/A = 2.0% discrepancy |

### Remaining before Phase 3 hardware

1. Install dependencies on Jetson: `pip install pyzmq msgpack`
2. Run `python -m jugglebot.motion.tests.test_control_loop` — loop timing and IPC latency must pass (not blocking Phase 3 Stage A which uses standalone harness). Note: in production, `control_loop` is launched via the `control_loop` console_scripts entry point (not `python -m`)

### Findings for future phases

1. **Singularity map condition numbers (449-644)**: Raw `cond(J)` reflects mixed mm/rad units in the Jacobian. The platform is not near-singular — the numbers are simply not comparable to "well-conditioned ≈ 1". For Phase 6 runtime monitoring, normalize the Jacobian (e.g., characteristic length scaling) to make condition numbers interpretable.

2. **Rotation perturbation convention**: Jacobian columns 3-5 correspond to world-frame angular velocity, not rotation vector components. Numerical validation must perturb as `exp(skew(δ·eᵢ)) · R`, not `rotvec + δ·eᵢ`. This distinction matters for non-zero rotation states.

3. **Motor friction threshold** (2026-02-25): Axis 0 requires ~0.075 Nm to overcome static friction (bench-mounted, unloaded). Friction offset is ~0.27 A from multi-weight force test intercept. Relevant for Phase 3 feedforward tuning.

4. **Motor Kt validated** (2026-02-25): Multi-weight calibration measured Kt = 0.0624 Nm/A (slope of iq vs tau at 4 loads, R^2=0.994). Within 2.0% of datasheet Kt = 60/(2π×150) = 0.0637 Nm/A. Confirms spool radius derivation from `mm_to_rev` is correct.

### Post-Phase 3 additions (2026-02-25)

1. **`encode_set_input_torque()` added to `can/odrive.py`**: The function was documented as existing in Appendix D but was not actually present. Added: encodes a float32 torque (Nm) for ODrive command 0x0E (`set_input_torque`). Used by the standalone test harness and will be used by the control loop for torque-mode operation.

2. **Standalone single-leg test harness**: `tools/single_leg_test.py` — bypasses ROS2, talks directly to one ODrive via python-can. Implements all four Phase 2 bench tests plus multi-weight Kt calibration. See `tools/README.md`.
