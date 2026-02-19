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

### Why drop YASMIN?
- Current state machine: 11+ states across YASMIN framework (4 vendored sub-packages)
- New state machine starts with 5 states (BOOT → HOMING → IDLE → ACTIVE → FAULT), easily extensible
- The replacement is a lightweight, **extensible** state machine (~150-200 lines) using an `Enum` + registry pattern. Adding a new state means defining a class and registering it — no framework changes needed. YASMIN's Blackboard, MonitorState, ServiceState, and ActionState abstractions add complexity without proportional value

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
- [ ] **Test**: Verify heartbeat reception, motor state reporting, and basic commands against real hardware

### Phase 2: State Machine + Orchestrator
- [ ] Write `state_machine.py` (~150-200 lines, plain Python, extensible registry pattern, no YASMIN)
- [ ] Implement initial states: BOOT, HOMING, IDLE, ACTIVE, FAULT
  - ACTIVE state supports sub-modes (spacemouse, shell) from the start
- [ ] Write `orchestrator_node.py` with startup sequence: wait for heartbeats → encoder search → homing → IDLE
- [ ] Implement error severity classification (transient vs fatal) from day one
- [ ] FAULT state with configurable timeout + safe motor disable
- [ ] **Test**: Full power-on → homing → IDLE → ACTIVE (spacemouse) sequence on real hardware

### Phase 3: Motion Planner
- [ ] Extract IK math from `sp_ik.py` into `motion/ik_solver.py`
- [ ] Load geometry from generated `hardware_config.yaml`
- [ ] Implement `motion/workspace.py`: leg extension limits, position bounds (scaffold for force/stability)
- [ ] Implement `motion/trajectory.py`: smooth trajectory generation with pre-computed durations
  - Trapezoidal velocity profiles respecting per-leg velocity/acceleration limits
  - Duration estimation: compute exact time required, reject infeasible commands before starting
  - Rate-limited pose updates
- [ ] Write `motion_planner_node.py`: subscribe to `/platform_pose_cmd`, validate, plan trajectory, publish `/leg_lengths`
- [ ] Absorb `gently_move_platform_to_setpoint()` as a trajectory-planned move
- [ ] Absorb platform leveling as a planner method
- [ ] **Test**: Spacemouse control through new pipeline. A/B compare with old system

### Phase 4: Mocap Integration
- [ ] Lift `MocapInterface` class with minor cleanup into new `mocap_node.py`
- [ ] Keep tf2 static broadcast, QTM clock sync
- [ ] Remove Ball Butler marker logic if not needed yet
- [ ] **Test**: Verify `/mocap_data` and `/rigid_body_poses` publish correctly

### Phase 5: Integration + Polish
- [ ] Write new `jugglebot_launch.py` for the new node set
- [ ] Full system test: power on → homing → ACTIVE (spacemouse control) → shutdown
- [ ] Verify rosbag recording
- [ ] Clean up `archived/` directory (remove or keep as reference)
- [ ] Remove YASMIN vendored package (`ros_ws/src/yasmin/`) and rosbridge dependency if no longer needed

### Phase 6 (Future): Advanced Features
- [ ] Re-add ball catching as a motion planner process
- [ ] Re-add Ball Butler aiming/coordination
- [ ] Add force estimation + stability analysis to `motion/workspace.py`
- [ ] Pose correction (feedforward from mocap)

---

## Line Count Comparison

| | Current | Proposed |
|---|---------|----------|
| ROS2 nodes | 18 entry points | 4-6 entry points |
| Python lines | ~12,900 | ~2,500-3,000 |
| State machine | 11+ states (YASMIN + 4 vendored packages) | 5 initial states, extensible (~200 lines) |
| CAN interface | 1 monolithic file (2,351 lines) | 4 focused modules (~800 lines) |
| Dependencies | ROS2 + YASMIN + rosbridge | ROS2 only |
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

### Items for investigation during Phase 2+
- **~~`time.sleep()` blocking the executor~~** — **RESOLVED**: All long-running operations now use generator-based state machines driven by `_run_to_completion()`. The driver processes CAN messages between yield points (1ms polling during timed waits, immediate re-entry for tight polls). Service callbacks still block the single-threaded executor for the duration of the operation, but CAN traffic flows continuously. The remaining `time.sleep()` calls are 1-5ms CAN bus pacing delays between consecutive sends to avoid buffer overflow — these are intentional and harmless. If true non-blocking services are needed later, the generators can be driven by a timer instead of synchronously (the generator protocol supports this without code changes to the generators themselves).
- **Shallow-copy thread safety**: `get_states()` does a shallow list copy — readers and writers share the same `MotorStateSingle` objects. This works in CPython due to GIL-atomic attribute access but is technically a race condition. Not worth fixing unless moving to a multi-threaded executor.
- **DBC file cleanup**: The `resources/ODrive_Pro.dbc` file is no longer loaded but is still in the package. Could be removed after confirming no other code references it, or kept as documentation.
- **rigid_body_poses subscriber**: The old `can_interface_node.py` had a subscriber for `/rigid_body_poses` to check that the QTM base is at the origin. This was a safety check that has been deferred — it should be re-added in the orchestrator node (Phase 2) or mocap node (Phase 4).
