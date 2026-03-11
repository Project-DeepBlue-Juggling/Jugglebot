# Minor Updates — Proposed Code Changes

These are non-urgent improvements identified during documentation review. Each is independent and can be tackled in any order.

---

## ~~1. Improve `torque_ff` CAN Encoding Resolution~~ ✅ DONE (2026-03-10)

**Change:** Increased `leg_tor` scale from 1000 (0.001 Nm/count) to 10000 (0.0001 Nm/count), giving 10× better resolution with max ±3.2 Nm.

**Files modified:**

- `config/protocol_config.yaml` — `leg_tor: 10000.0`
- `config/ODrive config Files/odrive_pro_leg_config.json` — `"input_torque_scale": 10000`
- `config/generate_config.py` — no changes needed (already propagates `input_scales`)
- Generated files regenerated: `protocol_config.{h,py}` copied to all consumers
- `tools/single_leg_test.py`, `tools/free_platform_test.py`, `tools/supported_platform_test.py` — updated inline comments
- `docs/motion_planner/dynamics.md`, `docs/motion_planner/integration.md` — updated encoding docs
- `MOTION_PLANNER_PLAN.md` — updated risk table and observation #25

**Note:** All consumers (`can_node.py`, tool harnesses, firmware) already referenced the generated constant `INPUT_SCALE_LEG_TOR` / `InputScale::leg_tor` — no code logic changes were required. ODrive legs must be reconfigured with the updated `odrive_pro_leg_config.json` before hardware use.

---

## ~~2. Add `end_boundary_state()` Convenience Function~~ ✅ DONE (2026-03-10)

**Problem:** `evaluate(traj, t)` at `t >= t_end` returns the end **pose** with **zero** twist and acceleration (hold behaviour). This is correct for motor commands but wrong when reading boundary conditions for trajectory planning (e.g., the return-to-home splice). The bug this caused was found and fixed during Phase 7, but the pattern is still error-prone.

**Proposed change:** Add a convenience function or method that makes the intent explicit:

```python
def end_boundary_state(traj: QuinticTrajectory) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the trajectory's end boundary conditions (pose, twist, accel).

    Unlike evaluate(traj, t_end), this returns the ORIGINAL boundary twist
    and acceleration, not zeros.
    """
    return (
        traj.end_state[:6],
        traj.end_state[6:12],
        traj.end_state[12:18],
    )
```

**Files to modify:**

- `ros_ws/src/jugglebot/jugglebot/motion/trajectory.py` — add function
- Any code currently accessing `traj.end_state` directly could optionally use this instead for clarity

**Verification:** Add a unit test confirming `end_boundary_state()` returns nonzero twist when the trajectory has nonzero end velocity, while `evaluate(traj, t_end + 0.1)` returns zero twist.

---

## ~~3. Re-implement Deferred Start for Dynamic Targets~~ REVERTED (2026-03-11)

Deferred start was re-implemented (2026-03-10) but caused three categories of failure during hardware validation:

1. **GIL contention**: The background `find_min_feasible_duration()` binary search (~1-2s on Jetson) starved the main control loop, causing position jumps when the loop resumed.
2. **Quintic overshoot**: Long-duration quintics starting from mid-motion states (nonzero velocity/acceleration) overshoot massively, causing feasibility rejections for trajectories that would have been fine at shorter durations.
3. **Unnecessary complexity**: The deferred start logic added significant branching in `submit_dynamic_target()`, `commit_async_trajectory()`, and `_bg_check_feasibility()`.

**Resolution:** Removed deferred start entirely. The platform now always uses the full requested duration (`arrival_time - t_now`) for its trajectory. For far-future targets this produces a very slow, gentle motion — which is well-behaved and trivially feasible. `DEFERRED_START_BUFFER_S`, the related branching, and 3 unit tests (13, 15, 16) were removed. The `deferred_start_test.py` hardware test suite was deleted.

---

## 4. Trajectory Visualization Tooling

**Problem:** No unified way to (a) preview trajectories with Cartesian + joint-space plots before hardware execution, or (b) analyze commanded vs. measured trajectories from rosbag recordings.

### 4a. Pre-Execution Preview Function

Add a standalone `preview_trajectory()` function that generates a standard set of plots:

```python
def preview_trajectory(traj, geom, params=None, show=True):
    """Plot Cartesian poses, twists, leg extensions, velocities, and optionally torques."""
```

**Files to create/modify:**

- `tools/trajectory_viewer.py` — add `preview_trajectory()` alongside existing 3D viewer
- Or create `tools/trajectory_plots.py` as a separate plotting module

### 4b. Rosbag Playback Analysis

Create a script that reads MCAP rosbag files and plots:

- Commanded Cartesian poses (from `/platform_pose_topic`)
- Commanded motor positions/velocities/torques (from `/leg_lengths_topic`)
- Measured motor positions (from `/motor_feedback` — see item #5 below)

**Files to create:**

- `tools/rosbag_analyzer.py` — reads MCAP, plots commanded vs measured

**Dependencies:** `rosbags` or `mcap` Python packages for MCAP file reading.

---

## ~~5. Publish Tracking Error and Motor Feedback to ROS2~~ ✅ DONE (2026-03-11)

**Change:** Added two new ROS2 publishers in `motion_bridge_node.py` that extract diagnostic fields from IPC telemetry and publish them as `Float64MultiArray` topics.

| New Topic | Message Type | Content |
|---|---|---|
| `/motion/tracking_error` | `Float64MultiArray` | 6 per-leg tracking errors (mm) |
| `/motion/diagnostics` | `DiagnosticStatus` | Named key-value pairs: `cond_number`, `workspace_status`, `workspace_speed_scale`, `traj_state`, `traj_progress`, `fault_state`. Level field reflects OK/WARN/ERROR. |

Motor feedback (encoder positions/velocities/currents) was **not** added as a separate topic because `/robot_state` already publishes per-motor `pos_estimate`, `vel_estimate`, and `iq_measured` via `MotorStateSingle[]` at 100 Hz from the CAN node.

**Files modified:**

- `ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py` — added 2 publishers, extraction logic in `_poll_telemetry()`
- `ros_ws/src/jugglebot/launch/jugglebot_launch.py` — added new topics to rosbag recording list

**Note:** Diagnostics uses `diagnostic_msgs/DiagnosticStatus` with named `KeyValue` pairs for readability. The `level` field is set automatically: OK (normal), WARN (soft workspace limit), ERROR (hard limit or fault). Added `diagnostic_msgs` dependency to `package.xml`.

**Verification:** Run system, verify topics appear in `ros2 topic list`, verify data flows into rosbag.

---

## 6. CAN Bus Load Measurement (bits/second)

**Problem:** The GUI shows CAN messages/second (counted by the Teensy) but not bits/second, which is the more meaningful metric for bus utilization.

**Background:**

- CAN bus runs at 1 Mbps (from `protocol_config.yaml`)
- A standard CAN frame with 8 data bytes is approximately 111 bits (SOF + arbitration + control + data + CRC + ACK + EOF + stuff bits)
- Theoretical max throughput: ~9,000 frames/second
- Stuff bits are data-dependent (worst case adds ~20%), but a fixed approximation is within ~10%

**Proposed change:** Add a `BITS_PER_CAN_FRAME_APPROX` constant (e.g., 111 for 8-byte payloads) and multiply by the existing message counter to get bits/second. Report both metrics.

**Files to modify:**

- Teensy firmware (wherever `msgs_per_sec` is computed) — add `bits_per_sec = msgs_per_sec * BITS_PER_CAN_FRAME_APPROX`
- `config/jugglebot_protocol.yaml` — define the constant
- GUI code that displays the metric — add bits/sec and utilization %

**Verification:** Check that reported bits/sec makes sense relative to 1 Mbps. At typical operation (~120 frames/sec for 6 axes × 2 directions × ~10 Hz), expect ~13,000 bits/sec (~1.3% utilization).

---

## ~~7. Update `connect()` Documentation for Context Manager~~ ✅ DONE (2026-03-10)

**Change:** Updated the `operations.md` PlatformTestHarness section to show both the context manager pattern (Pattern A) and explicit connect/disconnect pattern (Pattern B). Also fixed the danger box wording and corrected `shutdown()` → `disconnect()` (the actual method name).

**Files modified:**

- `docs/motion_planner/operations.md` — updated code example and danger box
