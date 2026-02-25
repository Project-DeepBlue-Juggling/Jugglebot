# Codebase Robustness Issues

Audit of the refactored CAN subsystem, state machine, orchestrator, and motion subpackage performed 2026-02-25. Issues ranked by severity.

---

## CRITICAL — Could cause hardware damage or unpredictable motor behavior

### 1. `_handle_error` logic can suppress fatal errors

**File**: `ros_ws/src/jugglebot/jugglebot/can_node.py` lines 229-272

The error handler has sequential branches where a fatal error can be set and then immediately un-set within the same invocation. The `no_active` flag is computed across *all* Jugglebot axes, but lines 268-271 can clear `fatal_error` entirely based on a single axis's undervoltage disarm — even when other axes still have active errors.

The flow:
1. Line 265: `fatal_error = True` (because `not no_active`)
2. Line 268-271: if `disarm_reason` has undervoltage AND `no_active` → clear `fatal_error` and call `_clear_errors()`

The contradiction: `no_active` was False at line 264 (which set the fatal flag), so line 268 should never trigger in the same call. But across *successive* calls from different axes, the interleaving can produce an inconsistent state where fatal errors from one axis are cleared by a clean report from another.

- **Worst case**: Fatal motor errors silently cleared; motors remain in CLOSED_LOOP with an unresolved fault.
- **Most likely**: Works fine because undervoltage events typically affect all axes simultaneously. A single-axis failure during undervoltage recovery could leave the system inconsistent.
- **Fix**: Restructure as: accumulate all error evidence first, then make a single determination. Never unconditionally clear `fatal_error` — only clear when *all axes* are confirmed clean.

---

### 2. `_gentle_move_steps` enters CLOSED_LOOP on legs that may have uncleared errors

**File**: `ros_ws/src/jugglebot/jugglebot/can_node.py` lines 957-960

When legs are not in CLOSED_LOOP, the generator unconditionally sends `CLOSED_LOOP` to all legs without checking `motors.fatal_error` or per-axis error state. If a leg has an active error or disarm reason, the ODrive will reject the state change or immediately fault.

This code path is reachable during error conditions: `_sub_control_mode` handles `'ERROR'` mode by calling `_gently_move_to_setpoint(0.0, deactivating=True)`, which reaches this generator.

- **Worst case**: After an error clears on some axes but not all, a stow attempt forces a faulted axis into CLOSED_LOOP, triggering a fault cascade.
- **Most likely**: The orchestrator's FAULT handling usually prevents reaching this during normal error flows. But the `_sub_control_mode` → ERROR path bypasses that protection.
- **Fix**: Add `fatal_error` / `fatal_can_error` check before sending CLOSED_LOOP commands. If errors are present, skip to IDLE-all instead of attempting a gentle move.

---

### 3. Homing motor loop has no maximum time budget

**File**: `ros_ws/src/jugglebot/jugglebot/can_node.py` lines 924-935

`_home_motor_steps` loops indefinitely waiting for the EMA of `iq_measured` to exceed the current limit. The generator only checks `fatal_error` and `fatal_can_error`, not elapsed time. If the motor jams mechanically without exceeding the current limit, or if current readings are erratic, this loop runs forever — blocking the single-threaded executor.

- **Worst case**: Motor stalls without exceeding current limit (friction changes, cable catch, wrong homing direction); node hangs permanently.
- **Most likely**: Works in practice because current rises quickly at end-stops. Environmental changes could make this unreliable.
- **Fix**: Add a deadline: `deadline = time.time() + HOMING_MOTOR_TIMEOUT_S` with a configurable timeout (e.g., 30s per motor). Return False and send IDLE on timeout.

---

## HIGH — Could cause system lockup or incorrect state

### 4. CAN bus failure during ACTIVE leaves motors in CLOSED_LOOP with no stow

**Files**: `ros_ws/src/jugglebot/jugglebot/state_machine.py` lines 278-280, `ros_ws/src/jugglebot/jugglebot/orchestrator_node.py` lines 120-123

When errors arrive during ACTIVE, the orchestrator calls `force_transition(FAULT)` which calls `ActiveHandler.on_exit()` → sets `ctx.request = 'deactivate'`. Then `_cancel_pending_operations()` clears `ctx.request = None`. The deactivation never dispatches.

FaultHandler then sets `control_mode = 'ERROR'`, which triggers the CAN node's `_sub_control_mode` to stow. But if the CAN bus is the thing that failed (`fatal_can_error`), `bus.send()` silently returns when `_bus is None` — so the stow command never reaches the hardware.

- **Worst case**: CAN bus fails, motors remain in CLOSED_LOOP with their last position command, no stow happens.
- **Most likely**: Mitigated by ODrive hardware watchdog timers (they fault to IDLE if they stop receiving heartbeats). But there's a window where motors are live with no commanded updates.
- **Fix**: In `_watchdog_check`, after `attempt_restore` fails, call `_emergency_idle()` as a best-effort last attempt before the bus is closed. Even if it fails, it makes the intent explicit.

---

### 5. `MotorStateTracker.update()` assumes axis IDs are contiguous 0-indexed array indices

**File**: `ros_ws/src/jugglebot/jugglebot/can/motor_state.py` lines 63-70

`update()` uses `axis_id` directly as an index into `self._states[axis_id]`. This works because `ALL_AXES = [0,1,2,3,4,5,6,7,8]` and `NUM_AXES = 9` — the IDs happen to be contiguous and 0-based. If `protocol_config` ever assigns non-contiguous or non-zero-based node IDs, this writes to the wrong slot or raises `IndexError`.

- **Worst case**: Encoder data from one axis updates the wrong axis's state; wrong leg moves.
- **Most likely**: Fine as long as axis IDs stay 0-8. But this is a hidden assumption.
- **Fix**: Add `assert ALL_AXES == list(range(NUM_AXES)), "Axis IDs must be contiguous 0-indexed"` in `__init__`, or switch to a dict-based lookup.

---

### 6. Single-slot command buffer can silently lose user intent

**File**: `ros_ws/src/jugglebot/jugglebot/orchestrator_node.py` lines 101-108

If two commands arrive within one 10 Hz tick (100ms window), the first is overwritten. A warning is logged, but the command is lost. The GUI or CLI could easily send rapid sequences (e.g., "activate" then immediately "spacemouse").

- **Worst case**: User sends "activate" then "spacemouse" — only "spacemouse" reaches the state machine, which discards it in IDLE (not recognized). Robot never activates.
- **Most likely**: Usually fine for human-driven commands. Automated test scripts or rapid GUI interactions could hit this.
- **Fix**: Use a small queue (e.g., `collections.deque(maxlen=4)`) and process all queued commands per tick.

---

## MEDIUM — Affects observability or edge-case correctness

### 7. `_tilt_to_quat` accumulates quaternions without normalization

**File**: `ros_ws/src/jugglebot/jugglebot/can_node.py` lines 1111-1123

`_last_tilt_offset` is updated by multiplying new rotations onto it every time a tilt reading is processed. This quaternion is never normalized, so numerical drift accumulates over time.

- **Worst case**: After thousands of tilt readings, quaternion norm drifts from 1.0, producing distorted orientation readings.
- **Most likely**: Drift is slow enough to not matter for typical session durations.
- **Fix**: Normalize `self._last_tilt_offset` after each multiplication: `result = result.normalized()` (numpy-quaternion supports this).

---

### 8. ZMQ `CONFLATE=1` on control process SUB can drop mode commands

**File**: `ros_ws/src/jugglebot/jugglebot/motion/ipc.py` line 151

The control process SUB socket has `CONFLATE=1`, which keeps only the most recent message **per socket** (not per topic). Since `TOPIC_TARGET`, `TOPIC_MODE`, and `TOPIC_MOTOR_FB` all come through the same SUB socket, a burst of motor feedback or target messages could overwrite a pending mode command.

- **Worst case**: An `estop` command gets overwritten by a motor feedback message and is never processed.
- **Most likely**: The IPC heartbeat watchdog (500ms) provides a backstop. But a delayed E-stop is concerning.
- **Fix**: Either remove `CONFLATE` (and accept queue growth risk), or use separate SUB sockets per topic, or at minimum use a separate socket for mode commands.

---

### 9. Error strings in `last_known_state` cleared optimistically

**File**: `ros_ws/src/jugglebot/jugglebot/can_node.py` lines 174-178

After `fetch_all`, if `fatal_error` and `fatal_can_error` are both False, the error list is cleared — even if errors were *just* cleared by `_clear_errors()` and the ODrives haven't confirmed the clear yet. Error strings disappear from monitoring before hardware actually confirms resolution.

- **Worst case**: Misleading rosbag data — errors appear resolved before they actually are.
- **Most likely**: Mostly cosmetic. The typed boolean flags are the source of truth for the orchestrator.
- **Fix**: Only clear the error list when heartbeat confirms `active_errors == 0` and `disarm_reason == 0` for all axes.

---

### 10. `_clear_errors` directly accesses `motors._states` bypassing the lock

**File**: `ros_ws/src/jugglebot/jugglebot/can_node.py` lines 375-376

`_clear_errors` iterates `self.motors._states` and sets `state.disarm_reason = 0` directly, bypassing the `_lock` and the `update()` API.

- **Worst case**: Under CPython's GIL this is safe for simple attribute writes. But it violates the encapsulation and would break under a multi-threaded executor.
- **Fix**: Add a `clear_disarm_reasons()` method to `MotorStateTracker` that acquires the lock.

---

## LOW — Design debt / cleanup

### 11. Generator pattern blocks executor for multi-second operations

**Files**: `ros_ws/src/jugglebot/jugglebot/can_node.py` (acknowledged in CODEBASE_REWRITE_PLAN.md Appendix B)

During homing (~30s), encoder search (~10s), or activation (~5s), the ROS2 executor is blocked. No other service calls can be processed; the node appears unresponsive to external callers. The keepalive schedule inside `_pump()` maintains CAN traffic and publishing, but subscriber callbacks and new service requests queue up.

This is a **known limitation**. If diagnostic tools or additional nodes need to query the CAN node during homing, they will time out.

---

### 12. No validation of incoming leg length commands

**File**: `ros_ws/src/jugglebot/jugglebot/can_node.py` lines 591-596

`_sub_leg_lengths` directly forwards positions to `_send_position_target` without checking that the array has exactly 6 elements or filtering NaN/Inf values. `clip_position` in `odrive.py` clips individual positions but doesn't reject clearly invalid inputs.

- **Fix**: Add `len(positions) == 6` check and NaN/Inf filtering before dispatching.

---

### 13. `on_shutdown` stow can block indefinitely

**File**: `ros_ws/src/jugglebot/jugglebot/can_node.py` lines 1134-1143

During shutdown, `_gently_move_to_setpoint` runs the full generator (up to `GENTLE_MOVE_TIMEOUT_S`). If the CAN bus is degraded, this blocks shutdown for the entire timeout period. The `main()` function has no timeout wrapper around `on_shutdown`.

- **Fix**: Add a maximum shutdown duration or skip the gentle stow if `fatal_can_error` is set.

---

## Summary

| # | Severity | Module | Issue |
|---|----------|--------|-------|
| 1 | **CRITICAL** | can_node | Error handler can suppress fatal errors |
| 2 | **CRITICAL** | can_node | Gentle move enters CLOSED_LOOP ignoring errors |
| 3 | **CRITICAL** | can_node | Homing motor loop has no timeout |
| 4 | HIGH | orchestrator + can_node | CAN bus failure → no stow possible |
| 5 | HIGH | motor_state | Axis ID = array index assumption |
| 6 | HIGH | orchestrator | Single-slot command buffer drops commands |
| 7 | MEDIUM | can_node | Tilt quaternion accumulates without normalization |
| 8 | MEDIUM | ipc | ZMQ CONFLATE can drop mode commands |
| 9 | MEDIUM | can_node | Error strings cleared optimistically |
| 10 | MEDIUM | can_node | Direct access to `_states` bypasses lock |
| 11 | LOW | can_node | Generator blocks executor (known limitation) |
| 12 | LOW | can_node | No validation of leg length array |
| 13 | LOW | can_node | Shutdown stow can block indefinitely |

**Overall assessment**: The architecture is solid and well-documented. Issues 1-3 should be fixed before hardware testing. Issues 4-8 will surface during real-world fault conditions. Issues 9-13 are code hygiene items for long-term maintainability.
