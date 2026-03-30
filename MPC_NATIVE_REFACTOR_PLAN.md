# MPC-Native Architecture Refactor Plan

## Motivation

The current motion pipeline routes MPC commands through `control_loop.py`, a 500 Hz
process originally designed to compute IK + dynamics from Cartesian poses.  For
MPC pass-through, this intermediate layer:

- **Introduces a slew limiter mismatch**: the per-cycle budget (9.5 rev/s × 2 ms =
  0.019 rev ≈ 1.35 mm) systematically truncates 50 Hz MPC commands that arrive as
  20 ms position steps, causing position lag and vel_ff/position conflicts.
- **Adds unnecessary computation**: IK, dynamics, stream smoother, and trajectory
  manager are all bypassed in MPC mode.  The forward-once gate and telemetry gate
  are workarounds for the architectural mismatch.
- **Runs at the wrong rate**: 500 Hz loop with 50 Hz useful work and 450 Hz of
  early returns.

### Target state

A single motion planner (the MPC) handles all motion commands.  Spacemouse, GUI,
shell, and catch modes simply set the target pose; the MPC plans the path.  The
former control loop becomes `motor_guard.py` — a 500 Hz process that linearly
interpolates between 50 Hz MPC commands, validates them against motor feedback,
and forwards them to the CAN layer.

ODrives stay in PASSTHROUGH mode (POS_FILTER is incompatible with vel_ff/torque_ff
— see Phase 1 findings).

---

## Phase Summary

| Phase | Description | Status |
|-------|-------------|--------|
| **1** | Exploratory Hardware Testing | COMPLETE |
| 1A | POS_FILTER investigation | CANCELLED |
| 1B | Linear interpolation for MPC commands | PROVISIONAL (superseded by Phase 2) |
| 1C | Step limit increase | DONE (permanent) |
| **2** | control_loop.py → motor_guard.py | DONE (offline) |
| 2A | Create `motor_guard.py` | DONE |
| 2B | Update IPC topology | DONE |
| 2C | Update launch configuration | DONE |
| 2D | Update bridge node | DONE |
| 2E | Update HardwarePlant | DONE |
| 2F | Migrate tests | DONE |
| **3** | Unify All Input Modes Through MPC | DONE |
| 3A | Target-setting interface | DONE |
| 3B–3E | All input modes → MPC target | DONE |
| 3F | Extract controller + hardware MPC entry | DONE |
| **4** | Cleanup | IN PROGRESS |
| 4A | Remove dead code | DONE |
| 4B | Update documentation | NOT STARTED |
| 4C | Update config generation | NOT STARTED |

---

## Architecture Overview

### Current (MPC pass-through)

```
                          :5555                         :5556
Spacemouse/GUI ──► Bridge ──────► control_loop (500 Hz) ──────► Bridge ──► CAN node ──► ODrive
                          ◄──────  IK, dynamics, smoother       ◄──────        PASSTHROUGH
MPC (50 Hz) ──► HardwarePlant ──────────────────────────────►
                               :5557  (pass-through bolt-on)
```

### Target (MPC-native)

```
                   ROS2 topics          :5558                       :5556
Spacemouse/GUI ─────────────► mpc_bridge ──────► MPC (50 Hz) ──► HardwarePlant ──────► motor_guard ──► Bridge ──► CAN node ──► ODrive
Catch coordinator ──────────►                                                   ◄──────                ◄──────        PASSTHROUGH
                                                                                 :5555 (motor feedback)
```

Key changes:
- MPC is the sole motion planner for all modes
- `control_loop.py` → `motor_guard.py` (interpolator + safety monitor)
- ODrive stays PASSTHROUGH (vel_ff + torque_ff required; POS_FILTER drops them)
- Bridge node simplified (no target/trajectory/smoother IPC)

### Repository structure

The MPC solver lives in a top-level `controller/` package (extracted from
`sim/controller/` in Phase 3F), importable by both `sim/` (MuJoCo development)
and `ros_ws/` (hardware).  The solver has no ROS2 dependency — only CasADi/IPOPT
and numpy.

```
jugglebot/
├── controller/              ← MPC solver (portable, no ROS2, no MuJoCo)
│   ├── __init__.py
│   ├── mpc.py               ← MPCController class
│   ├── params.py            ← MPCParams tuning
│   └── ...
├── sim/                     ← MuJoCo simulation (imports controller/)
│   ├── plant/
│   │   ├── mujoco_plant.py
│   │   ├── hardware_plant.py
│   │   └── interface.py     ← PlantInterface, PlantState
│   ├── main.py              ← sim + hardware entry point
│   └── ...
├── ros_ws/src/jugglebot/    ← ROS2 nodes (imports controller/)
│   └── jugglebot/
│       ├── mpc_bridge_node.py  ← thin ROS2↔MPC bridge (target topics → ZMQ)
│       ├── motion/
│       │   ├── motor_guard.py  ← safety monitor (replaces control_loop.py)
│       │   ├── ipc.py
│       │   ├── geometry.py
│       │   └── ...
│       └── ...
└── config/                  ← codegen configs (unchanged)
```

The extraction was completed in Phase 3F (`git mv sim/controller controller`).
Sim files add both `_sim_dir` and `_repo_root` to `sys.path`; `controller/mpc.py`
uses a `TYPE_CHECKING` guard for `PlantState` (duck-typed at runtime).

### Where IK and feedback live

**IK is inside the MPC solver.**  The MPC formulates `q[k] = IK(p[k])` as an
equality constraint using CasADi symbolic IK (`mpc.py:_build_symbolic_ik()`).
Every optimization step enforces kinematic consistency.  The solver outputs leg
extensions directly — no external IK step is needed anywhere in the pipeline.
Non-MPC modes (spacemouse, GUI) feed target *poses* to the MPC, which internally
solves IK as part of the optimization.

**Motor feedback closes the loop at every MPC tick.**  `HardwarePlant.get_state()`
reads encoder positions *and velocities* from telemetry, converts to Cartesian
pose via FK (forward kinematics) and platform twist via `J⁻¹ · q̇` (Jacobian
inverse applied to measured leg velocities), and returns the actual platform
state.  The MPC replans from measured state every 20 ms — it never plans from
its own prediction.  If a disturbance pushes the platform off-plan, the next
solve sees the real position *and velocity* and adapts.

```
Feedback loop (20 ms cycle):
  Encoders → CAN → bridge → ZMQ → HardwarePlant.get_state() → PlantState
      → mpc.solve(actual_state, target) → leg extensions
          → HardwarePlant.command() → motor_guard → bridge → CAN → ODrives
```

The motor_guard refactor does not touch this feedback path — it only simplifies
what happens between `mpc.solve()` output and the CAN node.

---

## Phase 1: Exploratory Hardware Testing — COMPLETE

**Goal**: Understand what's needed for smooth MPC→hardware motion.

**Outcome**: Phase 1 was an exploratory investigation.  The permanent config changes
(1A, 1C) are kept.  The `control_loop.py` changes (1B) are provisional — they will
be superseded by `motor_guard.py` in Phase 2.  The key findings from hardware testing
inform the rest of the plan.

### 1A. POS_FILTER investigation — CANCELLED

**Status**: Investigated and rejected.

POS_FILTER mode does NOT accept vel_ff or torque_ff as inputs (per ODrive docs).
This means we would lose:
- **vel_ff**: velocity hint that helps the ODrive interpolate between 50 Hz
  position steps, critical for smooth PASSTHROUGH tracking
- **torque_ff**: gravity compensation feedforward that reduces PID effort,
  essential for high-performance motion at the platform's limits

**Decision**: Stay with PASSTHROUGH mode.  `POS_FILTER: 3` was added to
`protocol_config.yaml` for protocol completeness but is not used at runtime.

### 1B. Linear interpolation for MPC commands — PROVISIONAL

**Status**: Implemented in `control_loop.py` for hardware testing.  Will be
superseded by `motor_guard.py` in Phase 2.

**Problem discovered during testing**: The original forward-once gate sent each
50 Hz MPC command to the ODrive exactly once.  In PASSTHROUGH mode, vel_ff
persists as a constant velocity push for the full 20 ms between updates.  The
ODrive's position PID settles in ~4 ms; vel_ff keeps pushing for the remaining
~16 ms → overshoot → MPC corrects → oscillation (3-4 cycles before settling).

**Solution**: Replace the forward-once gate with a linear interpolator that
re-sends every 500 Hz cycle:

```python
# Latch base position + velocity on new MPC command
if self._mpc_cmd_new:
    self._mpc_base_pos_rev = mpc_motor_rev
    self._mpc_base_vel_rps = leg_velocities_to_motor_velocities(vel_mm_s, geom)
    self._mpc_base_timestamp = t_mpc_cmd

# Every cycle: extrapolate from last MPC command
dt_since_cmd = t_now - self._mpc_base_timestamp
pos = self._mpc_base_pos_rev + self._mpc_base_vel_rps * dt_since_cmd
```

This upsamples 50 Hz MPC commands to 500 Hz smooth ramps.  vel_ff is used
correctly as the *slope* of the ramp — the ODrive sees small incremental
position steps with matching velocity feedforward.  torque_ff (gravity comp)
passes through unchanged.

**Architectural note**: This interpolation role is the one piece of "control
authority" that cannot be removed from the intermediate process.  When
`control_loop.py` becomes `motor_guard.py` in Phase 2, the interpolator must
be preserved.  It's not planning — it's upsampling.

### 1C. Step limit increase — DONE (permanent)

**File**: `config/hardware_config.yaml`

`JB_OP_MAX_POSITION_STEP_REV`: 0.2 → 0.3 rev.  At 50 Hz MPC with max velocity
(9.5 rev/s), one step = 0.19 rev — the old 0.2 limit had only 5% margin.
0.3 gives 58% headroom for timing jitter.

### Phase 1 Findings (inform all subsequent phases)

1. **POS_FILTER is incompatible with feedforward.**  Per ODrive docs,
   POS_FILTER mode ignores vel_ff and torque_ff inputs.  PASSTHROUGH is the
   only viable mode for high-performance motion.

2. **500 Hz interpolation is architecturally required.**  The motor_guard
   cannot be a simple forward-on-arrival safety monitor.  It must linearly
   interpolate between 50 Hz MPC commands at 500 Hz so that vel_ff acts as a
   ramp slope (not a persistent push that causes overshoot).

3. **MPC trajectory quality matters.**  Hardware testing revealed non-monotonic
   motion (e.g. platform initially moving away from target before correcting).
   This is an MPC trajectory planning issue — cost weights, horizon length,
   or reference trajectory construction — not a pipeline issue.  Best addressed
   with offline MPC tuning + comprehensive offline tests before hardware.

4. **Actuator model τ is unchanged.**  Without POS_FILTER, there is no
   additional filter lag.  The MPC's τ=30 ms calibration remains valid.

5. **Scaling factors are correct.**  `INPUT_SCALE_LEG_VEL = 1000` and
   `INPUT_SCALE_LEG_TOR = 10000` match the ODrive firmware config exactly.
   Sign inversion via `_leg_sign()` is applied to position, vel_ff, and
   torque_ff uniformly.

### Phase 1 Changes Summary

| File | Change | Status |
|------|--------|--------|
| `config/protocol_config.yaml` | Added `POS_FILTER: 3` to `input_modes` | Permanent (informational) |
| `config/hardware_config.yaml` | `max_position_step_rev: 0.2` → `0.3` | Permanent |
| `config/generate_config.py` output | Regenerated all `.py` and `.h` files | Permanent |
| `control_loop.py` `__init__()` | Added interpolation state variables | Provisional (Phase 2 supersedes) |
| `control_loop.py` `_compute()` | Replaced forward-once gate with linear interpolator | Provisional (Phase 2 supersedes) |
| `control_loop.py` `_slew_limit()` | Standard per-cycle budget (interpolated steps are small) | Provisional (Phase 2 supersedes) |
| `control_loop.py` constants | Added `MPC_SLEW_DT_S = 0.02` (unused, retained for reference) | Provisional (Phase 2 supersedes) |

---

## Phase 2: control_loop.py → motor_guard.py — DONE (offline)

**Goal**: Build `motor_guard.py` from scratch as a clean replacement for
`control_loop.py`.  Strip to interpolation + safety monitoring.  Test
extensively offline before hardware.

### 2A. Create `motor_guard.py` — DONE

**File**: `ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py` (new, replaces
`control_loop.py`)

The motor guard is a standalone process that:
1. Receives MPC commands on :5557 (pos_rev, vel_ff_rps, torque_ff_Nm)
2. Receives motor feedback from bridge on :5555
3. Validates each command against safety checks
4. **Quadratically interpolates** between 50 Hz MPC commands at 500 Hz (upgraded from linear in 4A fix; see Phase 1B for history)
5. Forwards interpolated commands on :5556 to the bridge
6. E-stops if any safety check fails

**Safety checks to keep** (ported from control_loop.py):

| Check | Description | Action |
|-------|-------------|--------|
| Motor feedback staleness | No feedback for > 150 ms | Suppress commands |
| MPC command staleness | No MPC command for > 200 ms | E-stop |
| Motor overspeed | Any motor > MAX_MOTOR_VEL_RPS | E-stop |
| Max deviation | Commanded pos too far from actual | E-stop |
| Workspace limits | Leg extension or cond# out of bounds | E-stop (hard), log (soft) |
| IPC heartbeat | No messages from bridge for > 500 ms | E-stop |
| NaN/Inf check | Any non-finite value in command | Reject command |

**Safety checks to remove**:

| Check | Reason |
|-------|--------|
| Per-cycle slew limiter | Replaced by max-deviation check; interpolator handles smoothing |
| Tracking error threshold | Informational only (was logging, not E-stopping).  Motor guard can still log it but doesn't need the 500 Hz tracking error computation |

**Computation to remove**:

| Component | Reason |
|-----------|--------|
| IK (pose_to_leg_lengths) | MPC outputs leg extensions directly |
| Dynamics (gravity_to_motor_torques) | HardwarePlant.set_pose() computes torque_ff |
| StreamSmoother | MPC produces smooth trajectories |
| TrajectoryManager | MPC IS the trajectory planner |
| cartesian_to_motor_commands() | Not needed — MPC provides motor-space commands |
| Workspace speed scaling | MPC respects its own constraints |
| Direct-target mode | Replaced by MPC target-setting |
| Trajectory mode | Replaced by MPC |
| Jacobian/condition number computation | Keep for telemetry/monitoring only |

**Class structure**:

```python
class MotorGuard:
    """Interpolator + safety monitor between MPC and motor hardware.

    Receives pre-computed motor commands from the MPC (via HardwarePlant),
    linearly interpolates between 50 Hz MPC updates at 500 Hz, validates
    against motor feedback, and forwards approved commands to the bridge.

    Does NOT compute IK, dynamics, or trajectories — the MPC handles all
    motion planning.
    """

    def __init__(self, rate_hz=500, geom=None, ipc=None):
        # 500 Hz for linear interpolation of 50 Hz MPC commands.
        # Safety checks run every cycle; interpolated output sent every cycle.
        ...

    def run(self):
        while self._running:
            self._process_ipc()       # Read MPC commands + motor feedback
            self._check_safety()      # Staleness, overspeed, heartbeat
            self._interpolate_and_send()  # Interpolate + forward to bridge
            self._sleep_remainder()

    def _on_mpc_command(self, msg):
        # Validate: NaN, workspace, max-deviation
        # If OK: latch as new interpolation base (pos, vel, timestamp)
        # If bad: E-stop or reject

    def _on_motor_feedback(self, msg):
        # Update feedback state for safety checks

    def _interpolate_and_send(self):
        # Linearly extrapolate from last MPC command:
        #   pos = base_pos + vel × dt_since_cmd
        # Send interpolated (pos, vel_ff, torque_ff) to bridge every cycle

    def _check_safety(self):
        # Runs every cycle regardless of new commands:
        # - Motor feedback staleness
        # - MPC command staleness
        # - Motor overspeed
        # - IPC heartbeat
```

**Rate**: 500 Hz (2 ms cycle).  Required for smooth linear interpolation of
50 Hz MPC commands — each cycle sends an incremental position update to the
ODrive with matching vel_ff.  Safety checks (overspeed, staleness, heartbeat)
run every cycle at negligible cost.

#### 2A Implementation Notes

**Implemented** (2026-03-23):

The `MotorGuard` class is ~420 lines (vs control_loop.py's ~1230 lines).  Key
design decisions during implementation:

1. **Reuses `ControlProcessIPC` as-is.**  The motor guard subscribes to the
   same ZMQ sockets (`:5555` for mode/motor feedback, `:5557` for MPC commands)
   and publishes telemetry on `:5556`.  No IPC class changes needed for 2A --
   the unused topics (TARGET, TRAJECTORY, DYN_TARGET) are simply never matched
   in `_process_ipc()`.  IPC cleanup is deferred to Phase 2B.

2. **Max-deviation check runs at command arrival, not every cycle.**  The
   deviation between the incoming MPC position and the *current* motor feedback
   is checked once when the command arrives (in `_on_mpc_command()`), not every
   500 Hz cycle.  Rationale: the interpolator only extrapolates from the MPC
   command, so per-cycle drift is bounded by vel_ff × dt (small).  Checking at
   arrival catches the dangerous case: a corrupted or stale MPC command that
   jumps far from the actual motor position.

3. **Workspace check also runs at command arrival.**  The MPC provides
   `ext_mm` and `pose_6dof` for workspace/condition-number checks.  These are
   evaluated once per MPC command (50 Hz), not every interpolation cycle.
   This is sufficient because the MPC plans within workspace limits; the check
   is a safety net, not the primary constraint enforcer.

4. **`_trigger_estop()` is the single E-stop path.**  All safety violations
   funnel through one method that sets mode, zeros outputs, resets MPC state,
   and records the fault reason.  This eliminates the scattered E-stop logic
   that existed in control_loop.py.

5. **No `multiprocessing` dependency.**  The motor guard doesn't spawn any
   child processes (no feasibility worker, no trajectory manager).  The
   `main()` entry point is simpler -- no `set_start_method()` needed.

6. **`MAX_DEVIATION_REV = 0.5` (~36 mm at standard spool radius).**  This is
   generous enough to tolerate normal MPC step sizes (0.19 rev at max velocity)
   plus interpolation lag, but tight enough to catch a crashed MPC sending
   garbage positions.  May need tuning during Phase 2 hardware tests.

7. **Bounded extrapolation + stroke clamp** (2026-03-24).  The linear
   interpolator (`_interpolate_and_send`) originally extrapolated without
   any time or position bound — at max velocity (9.5 rev/s), a missed MPC
   command could drift ~1.9 rev (~136 mm) before the 200 ms staleness E-stop.
   Fixed with a two-phase approach: normal linear extrapolation for 40 ms
   (`MAX_EXTRAP_DT_S`), then velocity decays linearly to zero over 60 ms
   (`EXTRAP_DECAY_DT_S`).  Position follows a parabolic coast-down during
   decay (C0-continuous in velocity — no step discontinuity).  Worst-case
   travel at max velocity: ~0.665 rev (~47.5 mm).  Additionally, every
   interpolation cycle clamps `_commanded_pos_rev` against stroke hard limits
   (in rev-space) as a backstop — logs a warning if clamping activates, but
   does not E-stop (the MPC should recover on the next command).  5 new
   tests added to `test_motor_guard.py` (tests 20–24).

8. **Stroke clamp zeroes feedforward** (2026-03-24).  When `np.clip` activates
   on a leg at the stroke limit, `vel_ff` and `torque_ff` are now zeroed for
   that leg.  Previously only position was clamped — the ODrive received "hold
   at limit" (position) + "keep moving" (vel_ff) + gravity/inertia push
   (torque_ff), causing the velocity loop to fight the position loop with
   current spikes and potential oscillation.  Test 24 (`test_stroke_clamp`)
   extended with feedforward assertions for both positive and negative clamp
   directions.

**Items to watch during subsequent phases**:

- **`send_dynamic_feedback()` removed.**  The motor guard doesn't send dynamic
  target feedback (no feasibility pipeline).  If the bridge node still
  subscribes to `TOPIC_DYN_FEEDBACK`, it will simply never receive messages.
  This should be cleaned up in Phase 2B/2D.

- **Gravity correction (`set_gravity_offset`).**  RESOLVED.  Moved from
  motion_bridge → motor guard (where it was ignored) to the mpc_bridge_node,
  which composes the correction into every outgoing target orientation via
  rotation-matrix multiplication.  The MPC sees corrected references
  transparently.  The motor guard no longer receives this command.

- **Home pose seeding.**  The control_loop.py seeded the home pose on enable
  so the first telemetry had valid positions.  The motor guard waits for the
  first MPC command instead -- no output is sent until the MPC provides one.
  This means the bridge won't receive motor commands during the window between
  enable and first MPC command.  Verify this doesn't cause issues with the
  orchestrator's activation flow.

- **`set_feedforward` / `set_smoother_limits` commands.**  These mode commands
  are now dead in motor_guard context (no smoother, no local feedforward
  toggle).  They should be removed from the bridge and orchestrator in Phase
  2D/Phase 4.

- **Tracking error is computed but not gated.**  The motor guard computes
  tracking error for telemetry (informational) but does not log warnings or
  E-stop on large errors.  The max-deviation check at command arrival provides
  the safety equivalent.  If per-cycle tracking monitoring is needed, it can
  be added later with minimal effort.

### 2B. Update IPC topology — DONE

**File**: `ipc.py`

`ControlProcessIPC` renamed to `MotorGuardIPC` with `ControlProcessIPC = MotorGuardIPC`
backward-compat alias (control_loop.py still imports the old name).

**Removed** (constants, message constructors, sockets, methods):
- Constants: `TOPIC_TARGET`, `TOPIC_TRAJECTORY`, `TOPIC_DYN_TARGET`, `TOPIC_DYN_FEEDBACK`
- Message constructors: `make_target_state()`, `make_trajectory_command()`,
  `make_dynamic_target_command()`, `make_dynamic_target_feedback()`
- `MotorGuardIPC`: removed `_sub_target` socket (was CONFLATE sub for pose targets),
  removed `TOPIC_TRAJECTORY` and `TOPIC_DYN_TARGET` subscriptions from `_sub_mode`,
  removed `send_dynamic_feedback()` method
- `BridgeIPC`: removed `_sub_dyn_fb` socket (was sub for dynamic target feedback),
  removed `send_target()`, `send_trajectory_command()`, `send_dynamic_target()`,
  `recv_dynamic_feedback()` methods
- Updated docstrings and architecture diagram in module docstring

**Kept**: `TOPIC_MODE`, `TOPIC_MPC_CMD`, `TOPIC_MOTOR_FB`, `TOPIC_TELEMETRY`,
`make_mode_command()`, `make_telemetry()`, `make_motor_feedback()`, `make_mpc_command()`,
`_pack()`, `_unpack()`

`motor_guard.py` updated to import `MotorGuardIPC` instead of `ControlProcessIPC`.

#### 2B Implementation Notes

**Socket reduction**: `MotorGuardIPC` now has 4 SUB sockets (was 4, but
different).  The old `_sub_target` socket is gone — there are no
direct-target consumers.  A new `_sub_mpc_mode` socket was added on `:5557`
(non-CONFLATE) so that `HardwarePlant` enable/disable/estop messages are
never dropped by CONFLATE on `_sub_mpc_cmd`.  The `_sub_mode` socket now
only subscribes to `TOPIC_MODE` (was also subscribing to `TOPIC_TRAJECTORY`
and `TOPIC_DYN_TARGET`).

**BridgeIPC simplification**: `BridgeIPC` now has 1 PUB + 1 SUB socket (was
1 PUB + 2 SUB).  The `_sub_dyn_fb` socket for dynamic target feedback is gone.
The bridge can only send mode commands and motor feedback, and receive telemetry.

**Items to watch in subsequent phases**:

- **`motion_bridge_node.py` still imports removed symbols.**  It imports
  `make_target_state`, `make_dynamic_target_command` and calls `send_target()`,
  `send_dynamic_target()` on BridgeIPC — all now removed.  The bridge node
  **will fail to import** until Phase 2D cleans it up.  This is expected:
  2D is the bridge cleanup phase.

- ~~**`catch_coordinator_node.py` has a `_FeedbackIPC` class**~~ — **RESOLVED**.
  `_FeedbackIPC` replaced with `TargetFeedbackSub` from `ipc.py` (`:5559`).
  The catch coordinator now receives accept/reject feedback from the MPC
  process via a dedicated ZMQ channel.  MPC-side publishing is a TODO.

- **`control_loop.py` still imports `TOPIC_TARGET`, `TOPIC_TRAJECTORY`,
  `TOPIC_DYN_TARGET`** — these constants are gone.  `control_loop.py` is
  the deprecated predecessor being replaced; it will fail to import until
  archived in Phase 4A.  The `ControlProcessIPC` backward-compat alias
  keeps its class import working.

- **Test files** `test_control_loop.py` and `test_mpc_passthrough.py` import
  removed symbols (`make_target_state`, `TOPIC_TARGET`, etc.).  These tests
  are for the old control_loop and will be migrated/removed in Phase 2F.

### 2C. Update launch configuration — DONE

**File**: `ros_ws/src/jugglebot/launch/jugglebot_launch.py`

Changed `ExecuteProcess` from `control_loop` to `motor_guard`:
```python
motor_guard = ExecuteProcess(
    cmd=[os.path.join(pkg_lib_dir, 'motor_guard'), '--rate', '500'],
    output='screen',
)
```

Variable name also changed from `control_loop` to `motor_guard` throughout
the launch file (declaration + LaunchDescription assembly).

**File**: `ros_ws/src/jugglebot/setup.py`

Added `motor_guard` entry point; kept `control_loop` as deprecated alias:
```python
'motor_guard = jugglebot.motion.motor_guard:main',
'control_loop = jugglebot.motion.motor_guard:main',  # deprecated alias
```

Both entry points now resolve to `motor_guard:main`.  The deprecated alias
can be removed in Phase 4A after `control_loop.py` is archived.

### 2D. Update bridge node — DONE

**File**: `motion_bridge_node.py`

Simplifications:
- Remove `_sub_platform_pose` (was feeding direct-target mode via :5555)
- Remove trajectory-related IPC sending
- Remove dynamic-target IPC sending
- Remove smoother-limit IPC sending
- Keep: motor feedback forwarding (:5555), telemetry receiving (:5556),
  mode command forwarding (:5555), CAN node publishing

The bridge node's telemetry polling rate should stay at 500 Hz to match the
motor guard's interpolation rate.

#### 2D Implementation Notes

**Implemented** (2026-03-23):

**Removed** (imports, subscriptions, callbacks, state):
- Imports: `make_dynamic_target_command`, `make_target_state`,
  `DynamicTargetCommand`, `PlatformPoseCommand`
- Subscriptions: `platform_pose_topic`, `smoother_limits`, `catch/dynamic_target`
- Callbacks: `_on_pose_command()`, `_on_smoother_limits()`,
  `_on_catch_dynamic_target()`
- State: `_active_publisher` (publisher gating logic for pose commands)

**Updated**:
- All "control loop" / "control process" references → "motor guard"
- `_control_loop_enabled` → `_motor_guard_enabled`
- Diagnostics name: `motion/control_loop` → `motion/motor_guard`,
  hardware_id: `motion_planner` → `motor_guard`
- Removed `traj_state` and `traj_progress` from diagnostics publishing
  (motor guard doesn't produce these fields)
- Removed `gravity_offset` subscription from motion_bridge_node — the
  gravity correction now lives in mpc_bridge_node (see Phase 4 cleanup).

**Items to watch in subsequent phases**:

- ~~**`catch_coordinator_node.py` still has `_FeedbackIPC`**~~ — **RESOLVED**.
  See Phase 3E notes.  `_FeedbackIPC` replaced with `TargetFeedbackSub`.

- ~~**`gravity_offset` forwarding** sends `set_gravity_offset` to the motor
  guard, which logs "Unknown mode command" and ignores it.~~ — **RESOLVED**.
  Gravity correction moved to mpc_bridge_node: subscribes to `gravity_offset`
  topic, stores a correction rotation matrix, and composes it into every
  outgoing target orientation.  motion_bridge_node no longer forwards this.

### 2E. Update HardwarePlant — DONE

**File**: `sim/plant/hardware_plant.py`

Minor changes:
- `set_pose()` continues computing torque_ff from dynamics (gravity compensation).
- The `enable(source='MPC')` flow remains the same.

**Decision: vel_ff computation**

With PASSTHROUGH, vel_ff is critical for smooth inter-sample interpolation.
The finite-difference in `_on_mpc_command()` within motor_guard is the right place
to compute it (it has access to consecutive commands).  Remove the redundant
computation in HardwarePlant.command().  The motor_guard computes vel_ff from
consecutive extensions and includes it in the forwarded telemetry.

#### 2E Implementation Notes

**Implemented** (2026-03-23):

- **Removed vel_ff computation** from `command()`.  Deleted
  `_ff_vel_mm_s`, `_prev_ext_mm`, `_prev_cmd_time` state variables and
  the finite-difference code.  `command()` no longer sends `vel_mm_s` in
  the MPC command message.  Motor guard computes vel_ff from consecutive
  extensions independently.  (**Update 2026-03-24**: `command()` now sends
  `vel_mm_s` again, computed as `(cmd - q_init) / control_dt` using the
  deterministic MPC period.  Also sends `acc_mm_s2` for quadratic
  interpolation — see 4A fix.)

- **`set_pose()` now accepts `twist_6dof` and `accel_6dof`** (2026-03-24) —
  computes full Newton-Euler torque feedforward (gravity + platform inertia +
  reflected motor inertia) via `cartesian_to_motor_commands()`.  Previously
  `accel_6dof` was hardcoded to zeros (gravity-only).  See 4A fix.

- **All docstrings updated** — "control loop" → "motor guard" throughout.

- **`get_state()` now returns real platform twist** (2026-03-24) — previously
  returned `np.zeros(6)` regardless of motor velocity.  Now computes
  `platform_twist = solve(J, vel_mmps)` where `J` is the Jacobian at the
  FK-solved pose and `vel_mmps` is from real ODrive encoder velocity feedback.
  Falls back to zeros on singular Jacobian or missing motor data.  Cost:
  one `compute_jacobian()` + one 6×6 `linalg.solve` (~10–15 µs), negligible
  at 100 Hz telemetry rate.  Note: the mixed mm/rad Jacobian (cond ~449–644)
  may amplify noise into angular twist components — Jacobian normalization
  is tracked separately.

### 2F. Migrate tests — DONE

**Files**: `tests/test_control_loop.py`, `tests/test_mpc_passthrough.py`,
`tests/test_safety.py`, `tests/test_spacemouse_pipeline.py`

| Test file | Action |
|-----------|--------|
| `test_control_loop.py` | Rename to `test_motor_guard.py`.  Keep loop timing test (at 500 Hz).  Remove IK-specific tests. |
| `test_mpc_passthrough.py` | Refactor: motor_guard IS MPC pass-through by default.  Keep command forwarding tests, staleness tests. |
| `test_safety.py` | Port all safety tests to motor_guard.  Replace slew-limiter tests with max-deviation tests.  Keep overspeed, staleness, feedback tests. |
| `test_spacemouse_pipeline.py` | Remove (spacemouse now routes through MPC, tested at MPC level). |

#### 2F Implementation Notes

**Implemented** (2026-03-23):

Created `test_motor_guard.py` — comprehensive 18-test suite covering:

| # | Test | Source |
|---|------|--------|
| 1 | Loop timing (500 Hz jitter) | test_control_loop.py |
| 2 | IPC latency (ZMQ round-trip) | test_control_loop.py |
| 3 | Force conversion (round-trip) | test_control_loop.py |
| 4 | MPC enable | test_mpc_passthrough.py |
| 5 | MPC command flow | test_mpc_passthrough.py |
| 6 | MPC staleness E-stop | test_mpc_passthrough.py |
| 7 | Disable clears state | test_mpc_passthrough.py |
| 8 | Workspace hard limit E-stop | test_mpc_passthrough.py |
| 9 | Torque passthrough | test_mpc_passthrough.py |
| 10 | Zeros when no feedforward | test_mpc_passthrough.py |
| 11 | No feedback suppresses | test_safety.py |
| 12 | Stale feedback suppresses | test_safety.py |
| 13 | Motor overspeed E-stop | test_safety.py |
| 14 | Max deviation E-stop | NEW (replaces slew limiter) |
| 15 | NaN rejection | NEW |
| 16 | Vel_ff finite-difference | NEW |
| 17 | IPC heartbeat E-stop | NEW |
| 18 | Interpolation output | NEW |

**Tests NOT ported** (dead in motor_guard context):
- Slew clamp / transparency / sustained fault (test_safety 1-3) — replaced
  by max-deviation check (#14)
- Tracking error fault (test_safety 7) — motor guard doesn't E-stop on
  tracking error (informational only)
- Lead-time gate (test_safety 8) — no trajectory manager in motor guard
- Mode exclusivity target/trajectory (test_mpc_passthrough 4-5) — no
  target/trajectory modes exist
- Spacemouse pipeline tests — spacemouse routes through MPC

**Old test files kept for now** — they reference `control_loop.py` which
still exists.  They will be archived in Phase 4A along with `control_loop.py`.

All 18 tests pass on Windows (2026-03-23):
- Loop timing: mean=2.849ms, p99 jitter=1.078ms (gate: <4.0ms)
- IPC latency: median=0.614ms, 100/100 delivered
- All safety/command tests: PASS

---

## Phase 3: Unify All Input Modes Through MPC — NOT STARTED

**Goal**: Spacemouse, GUI, shell commands, and catch coordinator all set target
poses that the MPC tracks.

### 3A. Target-setting interface — DONE

**File**: `sim/controller/target.py` (will move to `controller/target.py` in 3F-step1)

The `TargetSource` protocol and `TargetCommand` dataclass, previously inline in
`sim/main.py`, are now formalized in `controller/target.py`.

#### 3A Implementation Notes

**Implemented** (2026-03-23):

1. **Created `sim/controller/target.py`** — the canonical MPC-level target
   interface with zero sim-specific dependencies (no MuJoCo, no hand/ball types).
   Contains:
   - `TargetCommand` dataclass: `target_pose` (6,), `arrival_time` (float|None),
     `target_twist` (6,)|None — the three fields consumed by `mpc.solve()`.
   - `TargetSource` protocol: `update(sim_time, state) → TargetCommand`.
     `state` typed as `Any` to avoid a dependency on `plant.interface` —
     concrete implementations type-narrow to `PlantState`.

2. **Updated `sim/controller/__init__.py`** — re-exports `TargetCommand` and
   `TargetSource` alongside `MPCController` and `MPCParams`.

3. **Updated `sim/main.py`** — the inline `TargetSource` protocol and
   `TargetCommand` dataclass are replaced by imports from `controller.target`.
   The sim-level `TargetCommand` subclasses the base, adding `hand_cmd` and
   `ball_spawn` fields for sim-specific control loop handling (hand trajectory
   sequencing, ball spawning).  All existing `TargetSource` adapters
   (`StaticTargetSource`, `WaypointTargetSource`, `CatchTargetSource`, etc.)
   continue to work unchanged — they return the sim-level `TargetCommand`
   which IS-A base `TargetCommand`.

4. **Removed unused imports** from `sim/main.py`: `Protocol`, `runtime_checkable`
   (now provided by `controller.target`).

**All tests pass** (2026-03-23):
- 16 target interface tests (test_target_interface.py)
- 37 MPC tests (test_mpc_static, test_mpc_dynamic, test_mpc_trajectory)
- 18 motor guard tests (test_motor_guard.py)

**Design decisions**:

- **File location: `sim/controller/` not top-level `controller/`.**  Since
  `sim/` is on `sys.path` and all existing code imports `from controller.xxx`,
  the file goes in `sim/controller/target.py`.  When 3F-step1 does the
  extraction (`git mv sim/controller controller`), this file moves with it
  automatically.  No import changes needed.

- **`state` parameter typed as `Any`.**  The `TargetSource` protocol uses
  `Any` for the `state` parameter rather than `PlantState`.  This avoids a
  dependency from `controller/` → `plant/` (which lives in `sim/`).  The
  `@runtime_checkable` decorator only checks method existence, not parameter
  types.  Concrete implementations type their `state` parameter as
  `PlantState` for IDE support.

- **Dataclass inheritance for sim fields.**  `sim/main.py` defines
  `TargetCommand(_BaseTargetCommand)` adding `hand_cmd` and `ball_spawn`.
  Python 3.8+ dataclass inheritance works correctly here because all child
  fields have defaults and the parent's trailing fields also have defaults.
  The `isinstance(sim_tc, base_TargetCommand)` check returns `True`.

**Items to watch in subsequent phases**:

- **3F-step1 extraction** is simplified: `controller/` directory already has
  `target.py` alongside `mpc.py`, `params.py`, `__init__.py`.  The `git mv`
  moves everything together.

- **ROS2 consumers** (3B-3E) will import from `controller.target` directly.
  They don't need the sim-level `TargetCommand` subclass — the base class
  with 3 MPC fields is sufficient.  Hand commands on hardware go through
  separate ROS2 topics to the CAN node, not through the MPC target.

- **`PlantState` dependency** — when `controller/` becomes a top-level package
  (3F-step1), `mpc.py` already imports `from plant.interface import PlantState`.
  This needs resolution: either move `PlantState` to `controller/` or adjust
  `sys.path`.  This is a 3F-step1 concern, not 3A.

### 3B–3E. All input modes → MPC target — DONE

**Architecture decision**: A single `mpc_bridge_node.py` handles ALL input
modes (spacemouse, GUI, shell, catch) rather than separate adapters per mode.
The bridge is mode-gated: it tracks `control_mode_topic` and only forwards
targets from the active source.

**New flow** (all modes):
```
spacemouse_handler ──┐
GUI (rosbridge)    ──┼──► platform_pose_topic ──► mpc_bridge_node ──► ZMQ :5558 ──► MPC process
shell commands     ──┘                                                                  ↓
catch_coordinator ────► catch/dynamic_target ──► mpc_bridge_node ──► ZMQ :5558 ──► MPC process
                                                                                        ↓
                                                                              HardwarePlant ──► motor_guard
```

**New IPC channel**: `tcp://127.0.0.1:5558` (MPC bridge PUB → MPC process SUB)
- `TOPIC_MPC_TARGET` (b'mpctgt'): target poses (CONFLATE — latest wins)
- `TOPIC_MPC_MODE` (b'mpcmode'): mode transitions (non-CONFLATE — every message)

Two SUB sockets on the MPC side (same pattern as MotorGuardIPC) prevent
CONFLATE from silently dropping mode messages when a target arrives in
the same inter-poll window.

**Message format** (`make_mpc_target()`):
```python
{
    'type': 'mpc_target',
    'target_pose': [x, y, z, rx, ry, rz],   # mm / rad (rotation vector)
    'arrival_time': float | absent,           # perf_counter deadline, or absent = ASAP
    'target_twist': [vx,vy,vz,wx,wy,wz] | absent,  # mm/s / rad/s, or absent = hold
    'source': 'spacemouse' | 'gui' | 'shell' | 'catch',
}
```

**Input mode mapping**:

| Mode | ROS2 topic | Message type | arrival_time | target_twist |
|------|-----------|--------------|--------------|--------------|
| SPACEMOUSE | `platform_pose_topic` | PlatformPoseCommand | None (ASAP) | None (hold) |
| GUI | `platform_pose_topic` | PlatformPoseCommand | None (ASAP) | None (hold) |
| SHELL | `platform_pose_topic` | PlatformPoseCommand | None (ASAP) | None (hold) |
| CATCH | `catch/dynamic_target` | DynamicTargetCommand | Absolute time | Linear velocity |

**Quaternion → rotation vector** conversion uses the production
`quat_to_rot_matrix()` + `rot_matrix_to_rotvec()` from `ik_solver.py`
to ensure consistency with the rest of the motion pipeline.

**Mode gating**:
- `platform_pose_topic` messages are only forwarded when `msg.publisher`
  matches the active mode (SPACEMOUSE/GUI/SHELL)
- `catch/dynamic_target` messages are only forwarded when mode is CATCH
- On mode transition to inactive, `make_mpc_mode('disabled')` is sent
- On mode transition to active, `make_mpc_mode(mode.lower())` is sent

#### 3B–3E Implementation Notes

**Implemented** (2026-03-23):

1. **`ipc.py`** — added target channel:
   - `MPC_TARGET_ADDR = 'tcp://127.0.0.1:5558'`
   - `TOPIC_MPC_TARGET`, `TOPIC_MPC_MODE` topic prefixes
   - `make_mpc_target()`, `make_mpc_mode()` message constructors
   - `MpcBridgeIPC` class (PUB side, used by mpc_bridge_node)
   - `MpcTargetIPC` class (SUB side, used by ZmqTargetSource in MPC process)

2. **`mpc_bridge_node.py`** — new ROS2 node (~170 lines):
   - Subscribes to `control_mode_topic`, `platform_pose_topic`,
     `catch/dynamic_target`
   - Mode-gated forwarding: only the active source's targets are published
   - Quaternion → rotation vector via `ik_solver.py` functions
   - Entry point: `mpc_bridge_node = jugglebot.mpc_bridge_node:main`

3. **`sim/input/zmq_target.py`** — new `ZmqTargetSource` (~170 lines):
   - Implements `TargetSource` protocol (`update(sim_time, state)`)
   - Two SUB sockets: CONFLATE for targets, non-CONFLATE for mode
   - Exposes `mode`, `enabled`, `has_target`, `source` properties
   - Default pose `[0, 0, default_z_mm, 0, 0, 0]` before first target
   - `poll()` drains ZMQ; `update()` calls `poll()` internally
   - Clears stale targets on disable so previous session data isn't reused

4. **`setup.py`** — added `mpc_bridge_node` entry point

5. **`jugglebot_launch.py`** — added `mpc_bridge_node` to launch description

**Design decisions**:

- **Separate node, not merged with motion_bridge_node.**  The motion bridge
  handles motor_guard ↔ CAN communication (500 Hz polling).  The MPC bridge
  handles target forwarding (event-driven, no polling).  Merging would
  couple unrelated concerns and make both harder to test.

- **ZMQ rather than ROS2 for the MPC target channel.**  The MPC process
  stays ROS2-free (identical code for sim and hardware).  ZMQ CONFLATE
  gives deterministic latest-wins semantics.  The MPC loop owns its own
  timing with no ROS2 executor interference.

- **`ZmqTargetSource` in `sim/input/` (not `controller/`).**  It's an
  input source alongside `spacemouse.py` and `keyboard.py`.  It has a
  ZMQ + IPC dependency that would be inappropriate inside `controller/`.

- **Mode included in target messages as `source` field, not embedded.**
  Mode transitions are sent separately on `TOPIC_MPC_MODE` so they are
  never dropped by CONFLATE.  The `source` field on targets is
  informational (logging/diagnostics), not authoritative for gating.

**Items to watch in subsequent phases**:

- **MPC process lifecycle (3F-step2)**: `ZmqTargetSource.enabled` must
  drive `HardwarePlant.enable()` / `disable()` calls in the MPC loop.
  The source provides the state; the loop acts on it.

- **Shell command node**: Shell mode expects `platform_pose_topic` with
  `publisher='SHELL'`.  No shell command node exists yet — when created,
  it just needs to publish `PlatformPoseCommand` and the bridge handles
  the rest automatically.

- **Catch feedback**: The old `TOPIC_DYN_FEEDBACK` (accept/reject) is
  gone.  Replaced with a lightweight ZMQ feedback channel:
  - `MPC_FEEDBACK_ADDR` (`:5559`): dedicated PUB/SUB channel from the MPC
    process directly to the catch coordinator node.
  - `TOPIC_TARGET_FB` / `make_target_feedback()`: message carrying
    `arrival_time`, `accepted` (bool), `source`, and optional `violations`.
  - `TargetFeedbackPub` (MPC side) / `TargetFeedbackSub` (coordinator side)
    IPC classes in `ipc.py`.
  - `catch_coordinator_node.py`: `_FeedbackIPC` removed; replaced with
    `TargetFeedbackSub` import.  The existing `_poll_feedback()` and
    `report_acceptance()`/`report_rejection_with_position()` logic is
    unchanged — it just receives from the new channel.
  - `sim/main.py`: `TargetFeedbackPub` instantiated in hardware mode;
    `_send_target_feedback()` helper called after each `_mpc_solve()` in
    `run_mpc_headless()`.  Feedback sent once per unique catch target
    (deduplicated by arrival_time, 50 ms threshold).  "Accepted" =
    `Solve_Succeeded` or `Solved_To_Acceptable_Level`; "Rejected" =
    any other solver status, with status string in `violations`.

### 3F. Extract MPC to top-level `controller/` + ROS2 bridge — DONE

**Step 1: Extract `sim/controller/` → `controller/`**

Move the MPC solver out of the sim tree:
```
git mv sim/controller controller
```

Update imports in `sim/main.py`, `sim/tests/`, etc. from
`from sim.controller.mpc import ...` → `from controller.mpc import ...`.

The solver has zero ROS2 or MuJoCo dependencies (only CasADi, IPOPT, numpy).
Both `sim/` and `ros_ws/` import from `controller/` — the repo root is already
on `sys.path` in both environments.

**Step 2: Create `mpc_bridge_node.py`**

A thin ROS2 node (`ros_ws/src/jugglebot/jugglebot/mpc_bridge_node.py`) that:
- Subscribes to ROS2 target topics (spacemouse, GUI, catch coordinator)
- Translates them to `TargetCommand` and publishes via ZMQ to the MPC process
- Subscribes to MPC telemetry for diagnostics publishing

The MPC process itself stays ROS2-free.  On the Jetson it runs as a standalone
process (like the motor guard), receiving targets via ZMQ and sending commands
to the motor guard via the existing `:5557` path.

This preserves the MPC's testability: on Windows, `sim/main.py` drives it with
MuJoCo; on Jetson, `mpc_bridge_node.py` drives it with ROS2 target sources.
The solver code is identical in both cases.

#### 3F Implementation Notes

**Implemented** (2026-03-23):

**Step 1: `git mv sim/controller controller`** — controller package now lives at
the repo root, importable by both `sim/` and `ros_ws/` code.

1. **`PlantState` import resolved via `TYPE_CHECKING` guard.**
   `controller/mpc.py` used `from plant.interface import PlantState`, which breaks
   when `controller/` moves out of `sim/`.  Since `mpc.py` uses
   `from __future__ import annotations`, type annotations are strings at runtime —
   `PlantState` is only needed at type-checking time.  Moved the import into a
   `TYPE_CHECKING` block:
   ```python
   from typing import TYPE_CHECKING
   if TYPE_CHECKING:
       from plant.interface import PlantState
   ```
   At runtime, `mpc.py` accesses `state.platform_pos_mm`, `state.platform_rot`,
   `state.leg_extensions_mm`, `state.time` by duck typing — no `PlantState` class
   needed.

2. **`sys.path` updated in 10 files.**  All sim files that import from `controller`
   now add `_repo_root = os.path.dirname(_sim_dir)` to `sys.path` in addition to
   `_sim_dir`.  This is the minimal change — existing `from plant.xxx` imports still
   resolve via `_sim_dir`, and `from controller.xxx` resolves via `_repo_root`.

   Files updated: `sim/main.py`, `sim/demo_mpc.py`, `sim/analysis/record_baselines.py`,
   `sim/hand/feasibility.py`, `sim/input/zmq_target.py` (comment only — relies on
   caller), `sim/tests/test_mpc_static.py`, `test_mpc_dynamic.py`,
   `test_mpc_trajectory.py`, `test_variable_horizon.py`, `test_target_interface.py`.

**Step 2: `mpc_bridge_node.py`** — already completed in Phase 3B–3E.  No additional
work needed.

**Step 3: Hardware MPC entry point via `ZmqTargetSource`.**

3. **`sim/main.py --hardware` now defaults to `ZmqTargetSource`.**  When `--hardware`
   is specified without an explicit target (no `--pose`, `--sequence`, etc.), the MPC
   loop receives targets from `mpc_bridge_node` via ZMQ :5558.  This is the standard
   production path where all ROS2 input modes route through the bridge.

4. **Enable/disable lifecycle managed by the MPC loop.**  `run_mpc_headless()` now
   checks `source.enabled` each cycle (when the source supports it).  Mode transitions
   from `mpc_bridge_node` drive `plant.enable()` / `plant.disable()` calls:
   - `disabled → active`: call `plant.enable()` (activates motor guard)
   - `active → disabled`: call `plant.disable()` (deactivates motor guard)
   - While disabled: idle-sleep at `CONTROL_DT` (no MPC solve, no commands)

   Non-lifecycle sources (sim targets without `.enabled`) behave exactly as before —
   always active, no lifecycle checks.

5. **`getattr` guards for sim-specific `TargetCommand` fields.**  The base
   `TargetCommand` (from `controller/target.py`) has only `target_pose`,
   `arrival_time`, `target_twist`.  The sim subclass adds `ball_spawn` and
   `hand_cmd`.  Both `run_mpc_headless()` and `run_mpc_with_viewer()` now use
   `getattr(tc, 'ball_spawn', None)` and `getattr(tc, 'hand_cmd', None)` so
   ZmqTargetSource (which returns the base class) works without AttributeError.

**All tests pass** (2026-03-23):
- 16 target interface tests
- 14 MPC static tests
- 13 MPC dynamic tests
- 12 MPC trajectory tests
- 20 variable horizon tests
- 18 motor guard tests
- Total: 93 tests, all PASS

**Items to watch during subsequent phases**:

- **Motor guard staleness during mode transitions.**  When the MPC loop transitions
  from active → disabled, it calls `plant.disable()`.  The motor guard should accept
  the disable command and stop expecting MPC commands (no staleness E-stop).  When
  transitioning back to active, `plant.enable()` reactivates.  Verify this lifecycle
  works correctly on hardware — the motor guard's `_on_mode_command('disable')` must
  clear the MPC staleness timer.

- **CasADi on Jetson.**  The controller package requires CasADi + IPOPT.  These need
  to be installed on the Jetson Orin Nano (Ubuntu 20.04, Python 3.8).  CasADi provides
  `pip` wheels; IPOPT may need system-level installation (`apt install coinor-libipopt-dev`).
  Verify solver performance on ARM64 — the MPC must solve in <20 ms to maintain 50 Hz.

- **`PlantState` type checking.**  The `TYPE_CHECKING` guard means IDE support for
  `PlantState` in `mpc.py` depends on the type checker having `sim/` on its path
  (for `from plant.interface import PlantState`).  This may require a `pyrightconfig.json`
  or similar at the repo root with `extraPaths: ["sim"]`.  Not a runtime issue — only
  affects IDE autocompletion and type checking.

- **`sim/controller/` removal.**  After `git mv`, `sim/controller/` no longer exists.
  If any script or tool has a hardcoded reference to `sim/controller/`, it will fail.
  The `__pycache__` directory at `controller/__pycache__/` may contain stale bytecode
  from when the package lived in `sim/` — consider clearing it on first Jetson deploy.

---

## Phase 4: Cleanup — IN PROGRESS

### 4A. Remove dead code — DONE

After all modes route through MPC, the following became dead code:

| File/Component | Status |
|----------------|--------|
| `stream_smoother.py` | Dead — MPC handles smoothing |
| `trajectory_manager.py` | Dead — MPC handles trajectory planning |
| `feasibility.py` | Dead — MPC constraints handle feasibility |
| `feasibility_worker.py` | Dead — no async feasibility pipeline |
| `quintic.py` | Dead — MPC doesn't use quintics |
| `control_loop.py` | Replaced by `motor_guard.py` |
| `test_spacemouse_pipeline.py` | Dead |
| `test_dynamic_target.py` | Dead |
| `test_trajectory.py` | Dead |
| `test_control_loop.py` | Dead — tests archived control_loop |
| `test_mpc_passthrough.py` | Dead — tests MPC passthrough through archived control_loop |
| `test_safety.py` | Dead — tests archived control_loop safety |
| `test_stream_smoother.py` | Dead — tests archived stream_smoother |
| `test_async_pipeline.py` | Dead — tests archived feasibility_worker |
| `helpers.py` (test helper) | Dead — only used by archived test files |

**Action**: `git mv` to `archived/` — DONE (2026-03-23).

#### 4A Implementation Notes

**Implemented** (2026-03-23):

1. **Archived 6 source files** from `motion/` to `archived/`:
   `control_loop.py`, `stream_smoother.py`, `trajectory_manager.py`,
   `feasibility.py`, `feasibility_worker.py`, `quintic.py`

2. **Archived 9 test files** from `motion/tests/` to `archived/`:
   `test_control_loop.py`, `test_mpc_passthrough.py`, `test_safety.py`,
   `test_spacemouse_pipeline.py`, `test_stream_smoother.py`,
   `test_dynamic_target.py`, `test_trajectory.py`, `test_async_pipeline.py`,
   `helpers.py` (renamed to `test_helpers.py` in archived/)

3. **Removed `ControlProcessIPC` backward-compat alias** from `ipc.py`
   (line 362).  Only consumer was `control_loop.py` (now archived).

4. **Removed deprecated `control_loop` entry point** from `setup.py`.
   Only the `motor_guard` entry point remains.

5. **Cleaned up `test_hardening.py`** — removed 3 dead tests that depended
   on archived modules:
   - Test 8 (trajectory progress) — used TrajectoryManager
   - Test 9 (trajectory progress cancel) — used TrajectoryManager
   - Test 11 (feasibility/workspace consistency) — used quintic + feasibility

   Remaining 9 tests renumbered (1-9).  Removed unused imports:
   `DynamicsParams`, `compute_jacobian`, `extensions_mm_to_revs`,
   `check_feasibility`, `create_trajectory`, `evaluate`,
   `TrajectoryManager`, `TrajectoryState`.

6. **Cleaned up `test_dynamics.py`** — removed test 14 (torque profile
   preview) which depended on archived `quintic.py` and `feasibility.py`.
   Remaining 13 tests (1-13) are all live dynamics tests.

7. **Updated `motion/__init__.py`** — module listing now reflects only
   live modules: `geometry.py`, `ik_solver.py`, `workspace.py`,
   `conversions.py`, `dynamics.py`, `motor_commands.py`, `ipc.py`,
   `motor_guard.py`.

8. **Added NaN/Inf validation to `_on_motor_feedback()`** (motor_guard.py
   line 487-489).  Audit found that MPC commands were validated for
   NaN/Inf (line 325) but motor feedback was not.  Because `NaN > threshold`
   evaluates to `False` in numpy, corrupt feedback silently bypassed
   overspeed and max-deviation E-stop checks.  Fix: early return if any
   of pos/vel/cur are non-finite.  Added test 16 (NaN feedback rejection)
   to `test_motor_guard.py`.

9. **Added NaN/Inf validation to `ZmqTargetSource.poll()`** (sim/input/zmq_target.py).
   Audit found that MPC targets arriving via ZMQ were not validated before
   being passed to `mpc.solve()`.  A corrupted ROS2 message would produce
   NaN in the CasADi parameter vector, consuming the full solve budget
   before failing.  Fix: reject any target message where `target_pose`,
   `target_twist`, or `arrival_time` contains non-finite values.  The
   previous valid target is retained (MPC continues tracking last-known-good).

**All remaining tests pass** (2026-03-23):
- 19 motor guard tests: PASS
- 13 dynamics tests: PASS
- 8/9 hardening tests: PASS (test 2 has a pre-existing failure unrelated
  to this refactor — workspace check returns HARD_LIMIT at home pose)
- 16 target interface tests: PASS
- 59 MPC sim tests (static + dynamic + trajectory + variable horizon): PASS

**Items to watch**:

- ~~**`tools/` directory**~~: RESOLVED (2026-03-30).  9 broken tool scripts
  archived to `tools/archived/` via `git mv` (preserves history).  Two are
  fully obsolete (`smoother_test.py` — StreamSmoother dead, `dynamic_target_test.py`
  — TrajectoryManager dead).  Seven have reusable test scenarios but need
  MPC-based trajectory generation: `trajectory_test.py`, `trajectory_viewer.py`,
  `hardening_test.py`, `inertia_test.py`, `juggling_test.py`, `catch_sim_test.py`,
  `throw_catch_test.py`.  Working tools unaffected: `free_platform_test.py`,
  `single_leg_test.py`, `supported_platform_test.py`, `tracking_analyzer.py`.

- **`test_hardening.py` test 2 pre-existing failure**: The workspace check
  at home pose returns `HARD_LIMIT` instead of `OK`.  This predates the
  refactor — the home pose leg extensions (~26-29 mm) may be triggering
  the hard minimum (5 mm) on a different axis, or `StewartGeometry` may
  have changed since the test was written.  Worth investigating separately.

- **`config/hardware_config.yaml`**: Contains `feasibility_worker_max_restarts: 3`
  which references the now-archived feasibility worker.  This constant is
  harmless (unused at runtime) but could be cleaned up in Phase 4C.

#### 4A Fix: Double enable from two independent bridges (2026-03-24)

**Problem**: Both `motion_bridge_node.py` (port :5555) and `HardwarePlant.enable()`
(port :5557) send enable commands to the motor guard.  The ordering is
nondeterministic across poll cycles.  If MPC's enable arrived first, the
bridge's `disable`+`enable` sequence would reset `_mpc_prev_ext_mm`, causing
zero `vel_ff` on the next MPC command.

**Fix**: Made enable-when-already-ENABLED a no-op (debug log only, no state
reset).  The bridge's `disable` step already clears all MPC state via
`_reset_mpc_state()`.  All three possible message orderings now converge to
the same correct end state: guard ENABLED with clean MPC state.

**Lifecycle authority**: The ROS2 motion bridge on :5555 is the primary
authority for motor guard enable/disable/estop.  `HardwarePlant.enable()` on
:5557 is retained as a fallback for direct hardware mode (no ROS2 bridge).

**Files changed**:

| File | Change |
|------|--------|
| `motor_guard.py:460-466` | Removed "MPC takeover" branch; enable-when-ENABLED is now a no-op |
| `test_motor_guard.py` | +2 tests (25: enable idempotent, 26: bridge disable+enable clears state) |
| `hardware_plant.py` | Docstring clarification on `enable()` scope |
| `ipc.py` | Updated architecture docstring with lifecycle authority note |

#### 4A Fix: Remove remaining dead code from motor_guard + ipc (2026-03-24)

Audit found three categories of dead code left over from the `control_loop.py` era:

1. **`_mpc_cmd_new` flag** (motor_guard.py) — set in `_on_mpc_command()` and reset
   in `_on_mode_command()`/`_reset_mpc_state()`, but never read.  Was the
   forward-once gate in control_loop.py.  Removed from 4 locations.

2. **`slew_limited` and `workspace_clamped` params** (ipc.py `make_telemetry()`) —
   always defaulted to `False`; motor guard never set them.  The slew limiter
   and workspace speed-scaling were control_loop.py concepts; the motor guard
   E-stops on hard limits instead of clamping.  Removed parameters, docstring
   entries, and body code.

3. **`traj_state` and `traj_progress` params** (ipc.py `make_telemetry()`) —
   never set by motor guard; leftover from `TrajectoryManager` integration.
   Removed parameters, docstring entries, and body code.  Updated
   `test_hardening.py` test 8 to remove dead assertions for these fields.

**Note**: The GUI (`panels.js:531-537`) still reads `traj_state` and
`traj_progress` from telemetry, but since motor guard never populates them,
the GUI already shows "idle" with no progress bar.  No functional change.

**Files changed**:

| File | Change |
|------|--------|
| `motor_guard.py` | Removed `_mpc_cmd_new` (4 locations: init, `_on_mpc_command`, `_on_mode_command`, `_reset_mpc_state`) |
| `ipc.py` | Removed `traj_state`, `traj_progress`, `slew_limited`, `workspace_clamped` from `make_telemetry()` |
| `test_hardening.py` | Removed `traj_state`/`traj_progress` from test 8 assertions |

#### 4A Fix: Telemetry hot-path allocation reduction (2026-03-24)

Audit item 10: `_publish_telemetry()` was constructing a new dict via
`make_telemetry()` every 500 Hz cycle, calling `.tolist()` on 8 numpy arrays
(~56 small Python objects/cycle), and double-converting via `list()` inside
`make_telemetry()`.

**Changes**:

1. **Pre-allocated telemetry dict**: `_telem_msg` created once in `__init__`,
   updated in-place each cycle.  Eliminates dict construction, key insertion,
   and `make_telemetry()` function call overhead.

2. **Collapsed duplicate torque keys**: `cmd_torques` and `ff_torques` carried
   identical data (`_commanded_torque_ff_Nm`).  Unified to `leg_torques`,
   following the `leg_pos`/`leg_vel` naming convention.  Removed `pd_torques`
   (never populated by motor guard).

3. **Eliminated double list conversion**: motor_guard previously called
   `.tolist()` then `make_telemetry()` wrapped it in `list()` again.
   Now `.tolist()` writes directly into the pre-allocated dict.

**Files changed**:

| File | Change |
|------|--------|
| `motor_guard.py` | Pre-allocated `_telem_msg` dict; `_publish_telemetry()` updates in-place; removed `make_telemetry` import |
| `motor_guard.py` | `_publish_fault_telemetry()`: `cmd_torques` → `leg_torques` |
| `ipc.py` | `make_telemetry()`: `commanded_torques`/`ff_torques`/`pd_torques` → `leg_torques`; `cmd_torques` key → `leg_torques` |
| `motion_bridge_node.py` | `cmd_torques` → `leg_torques`; feedforward diagnostic now reads from unified `leg_torques` |
| `test_motor_guard.py` | `cmd_torques` → `leg_torques` |
| `test_hardening.py` | Updated `make_telemetry()` calls + assertions for new signature |

#### 4A Fix: Quadratic interpolation + inertia feedforward (2026-03-24)

**Audit item 4**: 50 Hz position ripple at MPC command boundaries.

**Root cause analysis**: The audit originally attributed the ripple to nonlinear
spool geometry (velocity in extension-mm/s vs position in motor-rev space).
Investigation showed this is **incorrect** — `mm_to_rev` is a constant per-leg
scalar (~0.0142 rev/mm), not position-dependent.  The spool geometry is linear.

The actual cause is **constant-velocity extrapolation of an accelerating
trajectory**.  When the motor guard linearly interpolates `pos(t) = base + vel·dt`,
the extrapolated position at the end of an MPC interval differs from the next
MPC command by `a·dt²` (the second difference of consecutive extensions), where
`a` is leg acceleration and `dt = 20 ms`.  At catch-speed accelerations
(~10,000 mm/s²), this step is ~4 mm (~0.057 rev, ~460 encoder counts) — easily
visible to the ODrive PID, creating 50 Hz current ripple.

**Fix (two parts)**:

1. **Quadratic interpolation in motor_guard** (eliminates position ripple):
   HardwarePlant now computes leg acceleration from three consecutive commanded
   extensions: `acc = (u_curr - 2·u_prev + u_prev_prev) / dt²`, using the
   deterministic MPC control period (immune to ZMQ jitter).  This is sent via
   a new `acc_mm_s2` field in the IPC message.  The motor guard converts to
   rev/s² and extrapolates:

   ```
   pos(t) = base_pos + vel·dt + ½·acc·dt²
   vel_ff(t) = vel + acc·dt
   ```

   For constant-acceleration segments this is exact (zero step).  Residual error
   is O(jerk·dt³) — at catch-speed jerk (~30,000 mm/s³), this is ~0.24 mm,
   negligible.  The `vel_ff` sent to the ODrive is now time-varying within each
   MPC interval (smooth ramp vs staircase), which improves PID tracking.

   The extrapolation decay path (MPC late > 40 ms) uses the boundary velocity
   `vel + acc·MAX_EXTRAP_DT_S` as the starting velocity for the linear
   coast-down, and the boundary position includes the quadratic term.  Behaviour
   is continuous at the boundary.

   Backward-compatible: if `acc_mm_s2` is absent from the IPC message, the motor
   guard defaults to zeros (pure linear interpolation, same as before).

2. **Real platform acceleration for torque feedforward** (enables inertia FF):
   `main.py` now computes platform twist and acceleration by finite-differencing
   the MPC's `predicted_poses` trajectory:

   ```
   twist = (poses[1] - poses[0]) / dt0
   twist_next = (poses[2] - poses[1]) / dt1
   accel = (twist_next - twist) / (0.5 · (dt0 + dt1))
   ```

   Both are passed to `HardwarePlant.set_pose(pose, twist, accel)`, which feeds
   them into `cartesian_to_motor_commands()`.  The Newton-Euler dynamics model
   now computes the full torque feedforward: gravity + platform inertia
   (`F = m·a_com`, `τ = I·α + ω×Iω`) + reflected motor inertia
   (`τ_motor = J_rotor · q̈`).  Previously `accel_6dof` was hardcoded to zeros,
   so only gravity compensation was active.

   This uses the first three nodes of the MPC predicted trajectory (all fine-tier
   at 20 ms spacing), giving well-conditioned finite differences.  Falls back to
   gravity-only when `predicted_poses` has fewer than 3 nodes (first MPC solve).

**Files changed**:

| File | Change |
|------|--------|
| `ipc.py` | Added `acc_mm_s2` parameter to `make_mpc_command()` |
| `hardware_plant.py` | Added extension history tracking (`_prev_cmd_ext_mm`, `_prev_prev_cmd_ext_mm`); `command()` computes and sends `acc_mm_s2`; `set_pose()` accepts `accel_6dof` |
| `motor_guard.py` | Added `_mpc_base_accel_rps2` state; parses `acc_mm_s2` from IPC; `_interpolate_and_send()` uses quadratic extrapolation with time-varying `vel_ff` |
| `main.py` | Computes platform twist + acceleration from `mpc.predicted_poses/times`; passes to `set_pose()` |

**All existing tests pass** (28 motor guard + 238 sim tests).  Existing tests
use zero acceleration by default, so quadratic interpolation degenerates to
linear — no test changes needed.

**Items to investigate**:

- **Rotation vector finite difference**: Differencing rotation vectors is valid
  for small tilts (≤15°, per Phase 4 notes).  At larger tilts the rotation
  vector space isn't linear — twist/accel estimates would drift.  Workspace
  limits constrain tilt well below this, but worth verifying if the catch
  envelope is expanded.

- **Acceleration sign flip on oscillatory targets**: If the MPC alternates
  acceleration direction (e.g. tracking a fast-moving target with overshoot),
  the three-point finite difference may lag by one step.  The MPC's jerk
  penalty should prevent sharp sign changes, but worth monitoring in
  hardware telemetry.

- **Torque feedforward magnitude at catch speeds**: Phase 5 hardware tests
  showed inertia feedforward reduced PID effort by 1.5–2.8% at moderate speeds
  (stiction-dominated).  At catch speeds where inertia forces are larger
  relative to stiction, the improvement should be more significant.  This is
  now testable with the real acceleration data flowing through.

- **First-command transient**: The first two MPC commands have no acceleration
  (insufficient history).  The third command onward has valid acceleration.
  At startup the platform is stationary so this is benign, but a mode
  transition during motion (e.g. switching from spacemouse to catch) will
  have a 2-step gap where acceleration is zero.  The twist/accel from
  `predicted_poses` still provides torque feedforward during this gap.

### 4B. Update documentation

- `docs/motion_planner/control_loop.md` → rename/rewrite for motor_guard
- `docs/motion_planner/architecture.md` → update signal flow diagrams
- `SIMULATION_DEVELOPMENT_PLAN.md` → update Phase 6 hardware bridge section
- Memory files → update project structure

### 4C. Update config generation

If any generated config references control_loop paths or constants, update
`config/generate_config.py` accordingly.

---

## Implementation Order

```
Phase 1: Exploratory hardware testing   ← COMPLETE
  1A: POS_FILTER investigation          ← CANCELLED (vel_ff/torque_ff not supported)
  1B: Linear interpolation              ← PROVISIONAL (in control_loop.py, superseded by Phase 2)
  1C: Step limit increase               ← DONE (permanent: 0.2 → 0.3 rev)
          ↓
Phase 2: motor_guard.py                 ← DONE (offline)
  2A: motor_guard.py                    ← DONE: interpolator + safety monitor
  2B: IPC simplification                ← DONE: removed unused topics
  2C: Launch config                     ← DONE: points at motor_guard
  2D: Bridge cleanup                    ← DONE: removed dead IPC paths
  2E: HardwarePlant                     ← DONE: removed redundant vel_ff
  2F: Test migration                    ← DONE: 18 tests, all pass
          ↓
     [OFFLINE TESTS — verify interpolation, safety, IPC]
          ↓
     [HARDWARE TESTS — static hold → slow moves → fast moves]
          ↓
Phase 3: Unified input modes            ← DONE
  3A: Target-setting interface        ← DONE: TargetCommand + TargetSource in controller/target.py
  3B-E: All input modes → MPC target  ← DONE: mpc_bridge_node.py + ZmqTargetSource
  3F-step1: Extract controller/       ← DONE: git mv sim/controller controller + sys.path updates
  3F-step2: mpc_bridge_node.py        ← DONE (completed in 3B-E)
  3F-step3: Hardware MPC entry point  ← DONE: --hardware defaults to ZmqTargetSource, lifecycle mgmt
          ↓
     [HARDWARE TESTS — spacemouse/GUI through MPC]
          ↓
Phase 4: Cleanup                        ← IN PROGRESS
  4A: Remove dead code                  ← DONE: 15 files archived, 4 files cleaned up
```

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| MPC solver failure with no fallback | Motor guard holds last-good interpolated position.  MPC's own failure handling (hold last command, up to 10 failures) provides first line of defense. |
| Losing safety coverage during refactor | Phase 2 builds motor_guard with comprehensive offline tests before hardware.  Never remove a check without its replacement being tested. |
| Spacemouse latency through MPC | MPC solves in <18 ms.  Total latency: spacemouse → ROS2 → MPC → motor_guard → bridge → CAN ≈ 40-60 ms.  Acceptable for manual control (human reaction time ~200 ms). |
| CAN node step-limit rejects MPC steps | Resolved in Phase 1C: `JB_OP_MAX_POSITION_STEP_REV` increased from 0.2 → 0.3 rev (58% headroom). |
| vel_ff overshoot in PASSTHROUGH mode | Resolved by 500 Hz linear interpolation (Phase 1B finding).  motor_guard uses vel_ff as ramp slope, not persistent push. |

---

## Open Questions

1. ~~**MPC actuator model τ**~~: Resolved — POS_FILTER cancelled, no additional
   filter lag.  The existing τ=30 ms calibration remains valid.

2. ~~**Motor guard rate**~~: Resolved — 500 Hz required for linear interpolation
   of 50 Hz MPC commands.  Same rate as current control loop but trivial per-cycle
   work (one multiply-add + safety checks, no IK/dynamics).

3. **Hand motor**: Currently PASSTHROUGH with its own trajectory controller in
   firmware. This motor does NOT need to be touched by the MPC controller.

4. **Gravity levelling**: RESOLVED.  The gravity correction is now applied in
   `mpc_bridge_node.py` — the correction rotation matrix is composed into
   every outgoing target orientation before reaching the MPC.

5. **MPC trajectory quality**: Hardware testing revealed non-monotonic motion
   (e.g. tilting away from target before translating toward it).  This is an
   MPC tuning issue (cost weights, reference trajectory construction, horizon
   length) — not a pipeline issue.  Should be addressed with offline MPC tests
   before Phase 2 hardware testing.
