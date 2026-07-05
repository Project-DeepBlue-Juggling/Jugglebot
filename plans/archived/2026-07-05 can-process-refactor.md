---
title: CAN Process Extraction Refactor
created: 2026-03-31
status: active
---

# CAN Process Extraction Refactor

## Context

### Why this refactor exists

The current CAN interface (`can_node.py`, ~1,680 lines) is a ROS2 node that owns the CAN bus, handles ODrive protocol encoding/decoding, homing sequences, Ball Butler commands, error recovery, and motor command dispatch -- all within the ROS2 executor. This works, but creates a problem for the MPC-native motion pipeline:

**Motor commands traverse 3 ROS2 scheduling hops to reach the CAN bus:**

```
MotorGuard (500 Hz, own process)
  --> ZMQ :5556 telemetry (commanded pos/vel/torque)
    --> MotionBridgeNode (ROS2 timer, 500 Hz poll)
      --> ROS2 Float64MultiArray publish
        --> CAN Node (ROS2 subscription callback)
          --> _send_position_target() --> CAN bus --> ODrives
```

Each ROS2 hop adds 0.5-5ms of scheduling jitter. The feedback path has the same issue in reverse (encoder heartbeat --> CAN node --> ROS2 publish --> motion bridge --> ZMQ --> motor guard --> ZMQ --> HardwarePlant). During Phase 3.1 hardware bringup, the feedback path exhibited 80-140ms of startup latency -- partly pipeline fill time, partly ROS2 scheduling.

Additionally, the CAN node's 0.3 rev step-limit safety check silently rejects commands with **no feedback to the motor guard**. If a command is rejected, the motor guard continues interpolating forward, unaware that the motors haven't moved. This created a positive-feedback cascade during the Phase 3.1 oscillation incident (see `plans/archived/2026-03-30 mpc-oscillation-analysis.md`).

### What this refactor achieves

1. **Deterministic motor command latency** -- Motor guard sends commands directly to the CAN process via ZMQ, bypassing ROS2 entirely. Expected latency: <1ms (vs 2-10ms current).
2. **Deterministic encoder feedback** -- CAN process sends motor feedback directly to motor guard via ZMQ. Eliminates ROS2 from the feedback loop.
3. **Command rejection visibility** -- CAN process can NACK rejected commands back to motor guard via ZMQ, breaking the positive-feedback cascade.
4. **Clean separation of concerns** -- CAN bus I/O is deterministic and time-critical; ROS2 is for orchestration and monitoring. Mixing them in one node conflates two different reliability requirements.

### When to do this

**After** MPC bringup Phases 3-6 are complete (basic motions, tau calibration, dynamic trajectories). **Before** Phase 7 (catch trajectories), where 700 mm/s leg speeds make timing margins much tighter. The velocity feedforward fix and tracking clamp (`plans/archived/2026-03-30 mpc-oscillation-analysis.md`) are sufficient for Phases 3-6.

---

## Architecture

### Current

```
                          ROS2 Executor
                    +--------------------------+
MPC (40 Hz)         |                          |
  --> HardwarePlant |                          |
    --> ZMQ :5557 --+--> MotorGuard (500 Hz)   |   (own process)
                    |      |                   |
                    |      v ZMQ :5556         |
                    |    MotionBridgeNode ------+--> leg_lengths_topic --> CAN Node --> CAN bus
                    |      ^                   |                           |
                    |      | ZMQ :5555         |                           v
                    |    MotionBridgeNode <-----+-- robot_state <-------- CAN bus
                    +--------------------------+
```

### Proposed

```
                          ROS2 Executor
                    +--------------------------+
MPC (40 Hz)         |                          |
  --> HardwarePlant |                          |
    --> ZMQ :5557 --+--> MotorGuard (500 Hz)   |   (own process)
                    |      |                   |
                    |      v ZMQ :5560 (NEW)   |
                    |    CAN Process (own proc) |   (own process, NO ROS2)
                    |      |                   |
                    |      v ZMQ :5561 (NEW)   |
                    |    MotorGuard <-----------+   (encoder feedback)
                    |                          |
                    |    CAN Process            |
                    |      | ZMQ :5562 (PUB)   |
                    |      v                   |
                    |    CAN Bridge Node -------+--> robot_state, bb/heartbeat, ...
                    |      ^ ZMQ :5563 (REQ)   |
                    |      |                   |
                    |    CAN Bridge Node <------+-- services (home, activate, ...)
                    +--------------------------+
```

**Key changes:**
- Motor commands: MotorGuard --> ZMQ :5560 --> CAN Process --> CAN bus (1 hop)
- Motor feedback: CAN bus --> CAN Process --> ZMQ :5561 --> MotorGuard (1 hop)
- ROS2 bridge is telemetry/orchestration only -- not in the command critical path
- CAN process owns the CAN bus exclusively; no other process touches python-can

### New ZMQ Ports

| Port | Direction | Protocol | Purpose |
|------|-----------|----------|---------|
| :5560 | MotorGuard --> CAN Process | PUB/SUB | Motor commands (pos, vel_ff, torque_ff) |
| :5561 | CAN Process --> MotorGuard | PUB/SUB | Encoder feedback (pos, vel, current) |
| :5562 | CAN Process --> CAN Bridge | PUB/SUB | Robot state, BB heartbeat, events |
| :5563 | CAN Bridge --> CAN Process | REQ/REP | Orchestration commands (home, activate, ...) |

Existing ports (:5555, :5556) remain for MotionBridge --> MotorGuard mode commands and MotorGuard --> MotionBridge telemetry (diagnostics, workspace status). The MotionBridge no longer carries motor commands or motor feedback.

---

## Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| 1 | CAN Process: bus + motor commands + encoder feedback | NOT STARTED | | Medium | Motor command latency, encoder feedback loop |
| 2 | MotorGuard: direct ZMQ to CAN process | NOT STARTED | | Medium | End-to-end command path without ROS2 |
| 3 | CAN Process: homing, gentle move, encoder search | NOT STARTED | | High | Generator-based blocking operations in new process |
| 4 | CAN Process: Ball Butler, Teensy comms, error handling | NOT STARTED | | Medium | Full CAN protocol coverage |
| 5 | CAN Bridge Node: thin ROS2 wrapper | NOT STARTED | | Low | ROS2 service/topic interface preserved |
| 6 | MotionBridge simplification + launch file update | NOT STARTED | | Low | Clean transition, old code archived |

---

## Implementation Phases

### Phase 1: CAN Process Core (Bus + Motor Commands + Feedback) — NOT STARTED

**New file:** `ros_ws/src/jugglebot/jugglebot/can/can_process.py`

**Scope:**
- Standalone Python process (no ROS2 imports)
- Opens CAN bus (`CANBus` from `can/bus.py`)
- Subscribes to ZMQ :5560 for motor commands from motor guard
- Publishes encoder feedback on ZMQ :5561
- Polls CAN bus at 1 kHz (same as current `_poll_can_bus` timer)
- Dispatches ODrive heartbeat, encoder, error messages
- Maintains `MotorStateTracker` for encoder positions
- Applies the 0.3 rev step-limit check (with NACK on rejection)
- Sends `encode_set_input_pos()` per leg on command receipt

**IPC protocol (motor commands, :5560):**
```python
# MotorGuard --> CAN Process
{
    'type': 'motor_cmd',
    'pos_rev': [6 floats],         # absolute motor revolutions (ODrive convention)
    'vel_ff_rps': [6 floats],      # velocity feedforward (rev/s, pre-scaled)
    'torque_ff_Nm': [6 floats],    # torque feedforward (Nm, pre-scaled)
    'seq': int,                    # sequence number for drop detection
}
```

**IPC protocol (encoder feedback, :5561):**
```python
# CAN Process --> MotorGuard
{
    'type': 'motor_feedback',
    'pos': [6 floats],       # encoder position (rev, leg-sign-corrected)
    'vel': [6 floats],       # encoder velocity (rev/s, leg-sign-corrected)
    'cur': [6 floats],       # measured current (A)
    'seq_ack': int,           # last accepted command seq (for NACK detection)
}
```

**IPC protocol (command rejection, :5561):**
```python
# CAN Process --> MotorGuard (on same feedback channel)
{
    'type': 'cmd_rejected',
    'seq': int,               # rejected command's sequence number
    'reason': str,            # 'step_limit', 'nan', 'bus_error'
    'leg_steps_rev': [6],     # actual step sizes that triggered rejection
}
```

**Critical details:**
- Leg sign convention (`_leg_sign`) must be applied identically to current `can_node.py`
- Feedforward scaling (vel * 1000, torque * 10000 for legs) stays in CAN process
- int16 clamping for vel_ff and torque_ff stays in CAN process
- Position clipping to [0, max_rev] per axis stays in CAN process
- CAN bus send pacing (5ms between legs) may need adjustment for latency

**Dependencies:** `can/bus.py`, `can/odrive.py`, `can/motor_state.py`, `config/generated/hardware_config.py`, `config/generated/protocol_config.py`

### Phase 2: Motor Guard Direct Link — NOT STARTED

**Modified file:** `ros_ws/src/jugglebot/jugglebot/motion/motor_guard.py`

**Changes:**
- Add ZMQ PUB socket on :5560 (motor commands to CAN process)
- Add ZMQ SUB socket on :5561 (encoder feedback from CAN process)
- On each 500 Hz cycle, after interpolation: publish motor command on :5560
- Process encoder feedback from :5561 instead of from MotionBridge (via :5555)
- Handle `cmd_rejected` messages: reset interpolation base to current encoder position, zero velocity/acceleration feedforward, increment rejection counter

**Motor guard reaction to NACK:**
```
1. Receive cmd_rejected on :5561
2. Reset _mpc_base_pos_rev = _motor_fb_pos_rev (snap to reality)
3. Zero _mpc_base_vel_rps, _mpc_base_accel_rps2, _mpc_base_jerk_rps3
4. Increment _rejection_count
5. If _rejection_count > N consecutive: trigger soft fault
6. On next accepted command: reset _rejection_count
```

**Backward compatibility:** Motor guard should accept feedback from EITHER the old path (:5555 via MotionBridge) OR the new path (:5561 via CAN process), selected by a `--can-direct` flag or auto-detected by which socket receives data first. This allows incremental rollout.

### Phase 3: Homing, Gentle Move, Encoder Search — NOT STARTED

**Modified file:** `ros_ws/src/jugglebot/jugglebot/can/can_process.py`

**Scope:** Move the generator-based blocking operations into the CAN process:
- `_home_motor_steps()` -- velocity control until stall detection (EMA current monitoring)
- `_gentle_move_steps()` -- trapezoidal profiling at 100 Hz for activate/deactivate
- `_encoder_search_steps()` -- encoder index search on all 6 legs
- `_setup_odrives_steps()` -- ODrive configuration (mode, gains, limits)

**IPC protocol (orchestration commands, :5563 REQ/REP):**
```python
# CAN Bridge --> CAN Process (request)
{
    'type': 'command',
    'cmd': 'home' | 'activate' | 'deactivate' | 'encoder_search' |
           'clear_errors' | 'reboot' | 'set_vel_curr_limits' |
           'set_hand_state' | 'set_hand_gains' | 'set_hand_traj_cmd' |
           'smooth_move_hand',
    'params': { ... },   # command-specific parameters
}

# CAN Process --> CAN Bridge (response)
{
    'type': 'response',
    'success': bool,
    'message': str,       # error description on failure
    'data': { ... },      # command-specific return data (e.g., tilt reading)
}
```

**Critical: Generator pump loop.** The current `_run_to_completion()` pumps CAN polling + ROS2 timers while generators yield. In the CAN process, the main loop must continue polling CAN and processing motor commands while a homing/activate generator is running. This requires a cooperative scheduling approach:
- Main loop: poll CAN, process ZMQ commands, advance active generator
- Generator yields sleep duration; main loop continues other work during the wait
- Only one blocking operation at a time (home, activate, deactivate)
- Motor commands from motor guard continue to be forwarded during blocking ops (or explicitly paused, matching current behavior)

### Phase 4: Ball Butler, Teensy, Error Handling — NOT STARTED

**Modified file:** `ros_ws/src/jugglebot/jugglebot/can/can_process.py`

**Scope:**
- Ball Butler throw/reload/reset/calibrate commands (via :5563 REQ/REP)
- Ball Butler heartbeat parsing and forwarding (via :5562 PUB)
- Teensy time sync (100 Hz `broadcast_time()`)
- Teensy hand trajectory commands, smooth hand move
- Teensy tilt reading (request/response via CAN + :5563)
- Teensy state persistence (homing/levelling state)
- ODrive error handling (the multi-phase error logic from `_handle_error`)
- CAN watchdog and bus restore (non-blocking generator)
- CAN traffic reporting

**IPC protocol (robot state, :5562 PUB):**
```python
# CAN Process --> CAN Bridge (periodic, 100 Hz)
{
    'type': 'robot_state',
    'motor_states': [9x {pos, vel, iq_set, iq_meas, fet_temp, motor_temp, bus_v, bus_i, state, ...}],
    'has_fatal_error': bool,
    'has_fatal_can_error': bool,
    'has_undervoltage': bool,
    'firmware_validated': bool,
    'is_homed': bool,
    'levelling_complete': bool,
    'encoder_search_complete': bool,
    'errors': [str, ...],
}

# CAN Process --> CAN Bridge (periodic, 10 Hz)
{
    'type': 'bb_heartbeat',
    'connected': bool,
    'ball_in_hand': bool,
    'state': str,
    'yaw_deg': float,
    'pitch_deg': float,
    'hand_mm': float,
}

# CAN Process --> CAN Bridge (event-driven)
{
    'type': 'throw_announcement',
    'landing_pos_mm': [x, y, z],
    'landing_vel_mps': [vx, vy, vz],
    'landing_time_s': float,
}
```

### Phase 5: CAN Bridge Node (Thin ROS2 Wrapper) — NOT STARTED

**New file:** `ros_ws/src/jugglebot/jugglebot/can_bridge_node.py`

**Scope:** Translates between ROS2 services/topics and ZMQ messages to/from CAN process. No CAN bus access, no protocol logic, no state machines.

**ROS2 interface (preserved from current can_node.py):**

Services (proxied to CAN process via :5563 REQ/REP):
- `encoder_search` (Trigger)
- `odrive_command` (ODriveCommandService) -- clear_errors, reboot
- `get_platform_tilt` (GetTiltReadingService)
- `activate_or_deactivate` (ActivateOrDeactivate)
- `set_hand_state` (SetString)
- `set_hand_traj_cmd` (SetHandTrajCmd)
- `smooth_move_hand` (SetFloat)
- `set_hand_gains` (SetHandGains)
- `bb/send_throw_command` (SendBallButlerCommand)
- `bb/reload`, `bb/reset`, `bb/calibrate` (Trigger)

Action servers:
- `home_motors` (HomeMotors) -- proxied with progress feedback via :5562

Publishers (populated from :5562 PUB state):
- `robot_state` (RobotState) -- 100 Hz
- `can_traffic` (CanTrafficReportMessage)
- `hand_telemetry` (HandTelemetryMessage) -- 500 Hz
- `platform_target_reached` (LegsTargetReachedMessage) -- 10 Hz
- `bb/heartbeat` (BallButlerHeartbeatMsg) -- 10 Hz
- `throw_announcements` (ThrowAnnouncement) -- event-driven

Subscribers (forwarded to CAN process via :5563):
- `set_motor_vel_curr_limits` (SetMotorVelCurrLimitsMessage)
- `control_mode_topic` (String) -- forwarded to CAN process for hand state management
- `set_level_state` (Float64MultiArray)
- `bb/calibration_result` (BallButlerCalibrationResult)

**The bridge does NOT subscribe to `leg_lengths_topic`** -- motor commands now flow directly from motor guard to CAN process. The MotionBridgeNode no longer publishes to `leg_lengths_topic` either.

### Phase 6: Cleanup and Transition — NOT STARTED

**Changes:**
- **MotionBridgeNode** -- Remove `leg_lengths_topic` publisher (motor commands no longer flow through it). Keep mode commands (:5555) and telemetry (:5556) for diagnostics.
- **Launch file** -- Add CAN process (`ExecuteProcess`), replace `can_node` with `can_bridge_node`, keep `motion_bridge_node` (simplified role).
- **Archive** `can_node.py` -- Move to `archived/` with a note. Keep available as fallback until CAN process is fully validated.
- **Update `setup.py`** -- Add `can_process` and `can_bridge_node` entry points.

---

## Testing Plan

The CAN interface is safety-critical. A bug here can destroy hardware (motors driven past limits, uncontrolled motion, loss of E-stop). Testing must be exhaustive.

### Unit Tests (offline, no hardware)

**T-U1: CAN Process message dispatch**
- Mock CAN bus (no real socketcan)
- Inject ODrive heartbeat, encoder, error, iq, temp, bus_vc frames
- Verify correct decode and motor state tracker updates
- Verify correct leg sign convention on all 6 legs + hand

**T-U2: Motor command encoding**
- Send motor_cmd via ZMQ :5560
- Verify CAN process produces correct `encode_set_input_pos()` frames
- Verify leg sign inversion applied to position, vel_ff, torque_ff
- Verify feedforward scaling: vel * 1000, torque * 10000 (legs); vel * 100, torque * 100 (hand)
- Verify int16 clamping on vel_ff and torque_ff
- Verify position clipping to [0, max_rev] per axis

**T-U3: Step-limit rejection**
- Send motor_cmd with step > 0.3 rev from current encoder position
- Verify entire command rejected (all 6 legs)
- Verify `cmd_rejected` message sent on :5561 with correct seq and step sizes
- Verify command with step <= 0.3 rev is accepted

**T-U4: Step-limit gating on encoder data**
- Send motor_cmd before any encoder data received
- Verify command is accepted (step check is gated, matching current can_node behavior)
- Then inject encoder data, send command with large step
- Verify now rejected

**T-U5: Encoder feedback forwarding**
- Inject encoder estimate CAN frames for all 6 legs
- Verify motor_feedback ZMQ message on :5561 with correct positions, velocities
- Verify leg sign convention matches current can_node.py output

**T-U6: Sequence number tracking**
- Send commands with seq 0, 1, 2, 3
- Verify seq_ack in feedback messages tracks last accepted seq
- Reject command seq=4, verify seq_ack stays at 3

**T-U7: NaN/Inf rejection**
- Send motor_cmd with NaN in position
- Verify rejected with reason='nan'

**T-U8: Error handling (multi-phase)**
- Inject error CAN frame with active_errors bitmask
- Verify fatal_error flag set in robot_state
- Inject error frame with disarm_reason + CLOSED_LOOP state
- Verify fatal_error set
- Inject clear (all zeros)
- Verify fatal_error cleared

**T-U9: Undervoltage recovery**
- Inject undervoltage error (bit 9) only
- Verify soft reset attempted (clear_errors sent)
- Verify does NOT set fatal_error if soft reset succeeds

**T-U10: Homing sequence (mocked CAN)**
- Start home command via :5563
- Mock encoder feedback showing motor moving
- Mock iq_measured approaching current limit (EMA)
- Verify stall detected, set_absolute_position sent
- Verify all 7 axes homed (6 legs + hand)
- Verify response on :5563: success=True

**T-U11: Homing timeout**
- Start home command, mock motor not stalling
- Verify timeout fires at 30s
- Verify response: success=False, message includes timeout

**T-U12: Gentle move profile**
- Start activate command via :5563
- Capture all CAN position commands sent during gentle move
- Verify trapezoidal velocity profile shape (accel, cruise, decel)
- Verify position tolerance convergence (< 0.01 rev)
- Verify velocity tolerance convergence (< 0.1 rps)

**T-U13: Encoder search sequence**
- Start encoder_search via :5563
- Mock axis state transitions: IDLE --> ENCODER_INDEX_SEARCH --> IDLE (procedure_result=0)
- Verify all 6 legs searched
- Verify SDO read for commutation_mapper.pos_abs (endpoint 488)

**T-U14: Ball Butler throw encoding**
- Send throw command via :5563 with yaw, pitch, speed, delay
- Verify CAN frame matches `encode_throw_command()` output exactly
- Verify throw announcement computed (landing prediction)

**T-U15: Ball Butler state commands**
- Send reload, reset, calibrate via :5563
- Verify correct CAN ID used for each (0x7D2, 0x7D3, 0x7D4)
- Verify empty payload (dlc=0)

**T-U16: Ball Butler heartbeat parsing**
- Inject BB heartbeat CAN frames with various states
- Verify correct decode: ball_in_hand, state, yaw_deg, pitch_deg, hand_mm
- Verify bb_heartbeat published on :5562

**T-U17: CAN watchdog timeout**
- Start CAN process with mocked bus
- Stop injecting heartbeats
- Verify fatal_can_error set after 2.0s
- Verify bus restore attempted

**T-U18: CAN bus restore sequence**
- Trigger watchdog timeout
- Mock bus restore success (reconnect, heartbeats resume)
- Verify fatal_can_error cleared
- Verify motor commands resume

**T-U19: Time sync broadcast**
- Verify CAN process sends time sync frame on CAN_ID_SHARED_TIME_SYNC (0x7DD) at ~100 Hz
- Verify payload format: `struct.pack('<II', sec, usec)`

**T-U20: Teensy state persistence**
- Send state update via :5563 (is_homed, levelling_complete, tilt offsets)
- Verify CAN frame sent on CAN_ID_PLATFORM_STATE_UPDATE (0x6E0)

**T-U21: Concurrent motor commands during homing**
- Start homing via :5563
- Send motor_cmd on :5560 during homing
- Verify motor commands are either queued or rejected with appropriate reason (homing active)
- Verify homing completes normally

**T-U22: Graceful shutdown**
- Send SIGTERM / shutdown signal to CAN process
- Verify gentle stow initiated (if motors were active)
- Verify CAN bus closed
- Verify ZMQ sockets closed

**T-U23: Motor guard NACK handling**
- Inject cmd_rejected into motor guard's :5561 subscription
- Verify interpolation base reset to current encoder position
- Verify velocity/acceleration feedforward zeroed
- Verify next interpolation starts from encoder position (not stale MPC command)

**T-U24: CAN Bridge service proxying**
- Call each ROS2 service on CAN Bridge Node
- Verify correct ZMQ REQ sent to CAN process on :5563
- Verify response correctly translated to ROS2 service response
- Test timeout handling (CAN process doesn't respond within 5s)

**T-U25: CAN Bridge state publishing**
- Inject robot_state on :5562
- Verify ROS2 RobotState message published with correct field mapping
- Verify all 9 motor states present (6 legs + hand + 2 BB)
- Verify error flags mapped correctly

### Integration Tests (Jetson, CAN bus connected, motors OFF)

**T-I1: CAN bus connectivity**
- Start CAN process standalone
- Verify CAN bus opens on can0 at 1 Mbps
- Verify heartbeats received from all Jugglebot axes (0-6)
- Verify encoder estimates decoded with correct sign convention

**T-I2: ZMQ connectivity**
- Start CAN process + motor guard
- Verify motor guard receives encoder feedback on :5561
- Verify motor guard can send commands on :5560 (commands rejected since motors are idle -- that's correct)

**T-I3: CAN Bridge ROS2 interface**
- Start CAN process + CAN Bridge Node
- Verify `ros2 service list` shows all expected services
- Verify `ros2 topic echo /robot_state` shows motor states updating at 100 Hz
- Compare output field-for-field with current can_node.py output

**T-I4: Error flag propagation**
- Clear errors via CAN Bridge service
- Verify error flags reset in robot_state topic
- Inject a fault (e.g., disable power briefly)
- Verify has_fatal_error propagates to robot_state within 100ms

**T-I5: End-to-end latency measurement**
- Instrument CAN process: timestamp when motor_cmd received on :5560
- Instrument CAN process: timestamp when CAN frame sent to bus
- Measure delta over 10,000 commands
- **Pass criteria:** p50 < 0.5ms, p99 < 2ms

**T-I6: Feedback latency measurement**
- Instrument CAN process: timestamp when encoder CAN frame received
- Instrument motor guard: timestamp when motor_feedback received on :5561
- Measure delta over 10,000 samples
- **Pass criteria:** p50 < 0.5ms, p99 < 2ms

### Hardware Tests (Jetson, motors ON)

**HAND ON E-STOP FOR ALL HARDWARE TESTS.**

**T-H1: Homing via CAN process**
- Start full stack (CAN process + CAN Bridge + motor guard + orchestrator)
- Issue `activate` command via orchestrator
- Verify homing completes (all 7 axes)
- Verify final positions match expected home positions (0.1 rev legs, -0.1 rev hand)
- Compare homing duration with baseline from current can_node.py (should be similar)

**T-H2: Activate / deactivate cycle**
- After homing, issue `activate`
- Verify all legs reach ACTIVATE_POSITION_REVS
- Issue `deactivate`
- Verify all legs return to 0.0 rev and enter IDLE
- Repeat 3 times -- no faults, no position drift

**T-H3: MPC hold test (replicate Phase 2.4)**
- Activate + enter SHELL mode
- Run `python3 main.py --hardware --mpc --pose 0,0,170,0,0,0 --duration 60`
- **Pass criteria:** Zero platform movement, tracking error < 0.1mm (matching Phase 2.4 baseline)

**T-H4: MPC 5mm step (replicate Phase 3.1)**
- Run `python3 main.py --hardware --mpc --pose 0,0,175,0,0,0 --duration 30`
- **Pass criteria:** Smooth rise, no oscillation, tracking error < 2mm, zero CAN rejections

**T-H5: MPC sequence (Phase 3 full)**
- Run Phase 3.2 through 3.4 (return to Active, X+5mm, tiny tilt)
- **Pass criteria:** All smooth, no E-stops

**T-H6: Command rejection NACK verification**
- Temporarily lower step limit to 0.01 rev (will reject any motion)
- Issue a small MPC motion
- Verify motor guard receives cmd_rejected messages
- Verify motor guard resets interpolation (no runaway)
- Restore step limit

**T-H7: CAN disconnect recovery**
- During MPC hold, briefly disconnect CAN USB adapter
- Verify motor guard detects feedback loss (staleness)
- Verify CAN process detects heartbeat loss (watchdog)
- Reconnect adapter
- Verify CAN process restores bus
- Verify motor guard resumes operation

**T-H8: E-stop behavior**
- During MPC motion, press physical E-stop
- Verify motors disarm within 100ms
- Verify CAN process reports error state
- Verify motor guard enters E-stop
- Clear E-stop, verify recovery path works

**T-H9: Ball Butler throw (if BB hardware available)**
- Issue throw command via CAN Bridge service
- Verify CAN frame sent with correct encoding
- Verify throw announcement published with landing prediction

**T-H10: Endurance test**
- Run MPC with dynamic trajectories for 30 minutes
- Monitor: CAN error count, command rejection count, feedback latency, motor guard loop jitter
- **Pass criteria:** Zero CAN errors, zero rejections, p99 feedback latency < 2ms

### Regression Tests

**T-R1: Bit-exact CAN frame comparison**
- For every CAN frame type produced by CAN process, capture the raw bytes
- Compare against frames produced by current can_node.py with identical inputs
- Must be bit-identical for: set_input_pos, set_state, set_controller_mode, set_vel_curr_limits, set_absolute_position, clear_errors, all BB commands, time sync
- This catches sign convention errors, scaling errors, endianness errors

**T-R2: Robot state message comparison**
- Run CAN process + CAN Bridge alongside current can_node.py (on separate test rig or via CAN replay)
- Compare RobotState messages field-by-field
- All numeric fields must match within float32 epsilon

**T-R3: Homing sequence comparison**
- Record all CAN frames during homing with current can_node.py
- Record all CAN frames during homing with CAN process
- Compare: same sequence of commands, same axis ordering, same current limits, same stall detection threshold

**T-R4: All existing tests pass**
- `pytest tests/ -v` must pass with zero regressions
- `pytest tests/ros/ -v` specifically -- these test ROS2 node behavior

---

## Notes for Collaborators

### Safety-critical invariants (must be preserved exactly)

1. **Leg sign convention:** ODrive negative = extension, Jugglebot positive = extension. The `_leg_sign()` function in can_node.py (line 61) must be replicated exactly. A sign error here will drive motors in the wrong direction. Applied to: position commands, velocity feedforward, torque feedforward, encoder position readback, encoder velocity readback.

2. **Feedforward scaling:** Legs use INPUT_SCALE_LEG_VEL (1000) and INPUT_SCALE_LEG_TOR (10000). Hand uses INPUT_SCALE_HAND_VEL (100) and INPUT_SCALE_HAND_TOR (100). These convert float rev/s and Nm to int16 CAN fields. Wrong scaling = wrong motor behavior. The scaling constants live in `protocol_config.py`.

3. **Position clipping:** Legs [0, 4.2] rev, Hand [0, 11.1] rev. Exceeding these can physically damage the platform. The `odrive.clip_position()` function handles this.

4. **Step limit gating:** The 0.3 rev step check only activates after real encoder data has been received (`_encoder_data_received` flags). Before that, commands pass through unchecked. This prevents false rejections at startup when encoder positions are zero. The CAN process must replicate this gating.

5. **Homing current thresholds:** Leg stall = 5.0A (EMA, weight 0.7). Hand stall = 8.0A. Headroom = 3.0A added to velocity/current limits during homing. These values are from hardware_config and must not be changed without physical testing.

6. **CAN send pacing:** The current can_node sends one leg command per CAN frame with implicit pacing from the ROS2 callback. The CAN process must not flood the bus with 6 frames simultaneously -- space them by at least 0.5ms or use the bus send queue. At 1 Mbps with ~111 bits per frame, 6 frames = ~0.67ms bus time.

### Architecture decisions

7. **Single bus owner:** Only the CAN process touches python-can. The CAN Bridge Node, motor guard, and MPC process never import `can`. This eliminates race conditions on the bus.

8. **Generator-to-async migration:** The current homing/gentle-move generators use `_run_to_completion()` which pumps timers in a blocking loop. In the CAN process, these become cooperative coroutines in the main event loop. Each generator `yield sleep_duration` is replaced with a timestamp check in the main loop. Only one blocking operation runs at a time.

9. **Startup ordering:** CAN process must be running and have received heartbeats before motor guard is enabled. The launch file should start CAN process first, wait for a "ready" signal (e.g., a ZMQ message or file-based flag), then start motor guard.

10. **Shutdown ordering:** On SIGTERM, CAN process should: (a) stop accepting new commands, (b) gentle-stow if motors active, (c) send IDLE to all axes, (d) close CAN bus, (e) close ZMQ sockets. Timeout: 15s for stow, then emergency IDLE.

### Files affected

| File | Change |
|------|--------|
| `can/can_process.py` | **NEW** -- standalone CAN process |
| `can_bridge_node.py` | **NEW** -- thin ROS2 wrapper |
| `motion/motor_guard.py` | Add ZMQ :5560/:5561, NACK handling |
| `motion/ipc.py` | Add new port constants, message constructors |
| `motion_bridge_node.py` | Remove leg_lengths_topic publisher |
| `jugglebot_launch.py` | Add CAN process, replace can_node with can_bridge_node |
| `setup.py` | Add entry points |
| `can_node.py` | **ARCHIVED** (kept as fallback) |

### Rollback plan

Keep `can_node.py` fully functional and importable. Add a `--legacy-can` flag to the launch file that starts can_node.py instead of can_process.py + can_bridge_node.py. Motor guard's `--can-direct` flag controls which feedback path it uses. This allows instant rollback if the new architecture has issues during hardware testing.
