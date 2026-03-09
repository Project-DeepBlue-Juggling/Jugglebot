# Minor Updates — Proposed Code Changes

These are non-urgent improvements identified during documentation review. Each is independent and can be tackled in any order.

---

## 1. Improve `torque_ff` CAN Encoding Resolution

**Problem:** The `torque_ff` field is encoded as int16 with a fixed 0.001 Nm/count scale. Typical per-leg feedforward torques are 0.01–0.1 Nm (~10–100 counts), giving coarse resolution. The maximum representable value is ±32.767 Nm — far beyond what the system ever needs.

**Proposed change:** Increase resolution by reducing the scale factor (e.g., 0.0001 Nm/count → 10× better resolution, max ±3.2 Nm). Define the scale as a named constant in `config/jugglebot_protocol.yaml` so `generate_config.py` propagates it to all consumers.

**Files to modify:**

- `config/jugglebot_protocol.yaml` — add `INPUT_TORQUE_SCALE: 0.0001` (or similar)
- `config/generate_config.py` — ensure the new constant is included in generated outputs
- `ros_ws/src/jugglebot/jugglebot/can_node.py` — use the generated constant for int16 encoding instead of hardcoded `* 1000`
- Any Teensy/Ball Butler firmware that decodes `torque_ff` from CAN — use the same generated constant
- `tools/*.py` — any test harnesses that encode `torque_ff` directly (e.g., `hardening_test.py`, `trajectory_test.py`)

**Verification:** Run existing hardware tests and verify feedforward torques match expected values with the new encoding.

---

## 2. Add `end_boundary_state()` Convenience Function

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

## 3. Re-implement Deferred Start for Dynamic Targets

**Problem:** Deferred start was implemented in commit `a078b5c` (`DEFERRED_START_BUFFER_S = 2.0`) then removed in commit `1695946` ("Fix small jumps glitch") because it caused position discontinuities at the hold-to-move transition. Currently, far-future targets produce unnecessarily slow trajectories over the full duration.

**Proposed change:** Re-implement deferred start with proper splice continuity. The key issue was that the original implementation re-sampled the current state at the deferred `t_start` time, which could produce a discontinuity if a trajectory was active. The fix: when deferring, create the trajectory from the current state (at `t_now`) but set `t_start` in the future. The `evaluate()` method already handles `t < t_start` by holding at the start pose — so as long as the start state matches the current hold/trajectory state at the deferred start time, the transition will be smooth.

**Algorithm:**

1. Compute `min_feasible = find_min_feasible_duration(...)` for the target
2. If `arrival_time - t_now > min_feasible + DEFERRED_START_BUFFER_S`:
   - `motion_duration = min_feasible + DEFERRED_START_BUFFER_S`
   - `t_start = arrival_time - motion_duration`
   - Sample state at `t_start` from active trajectory (if executing) or hold pose (if idle)
   - Create trajectory with that state as start, `t_start` set in the future
3. The `evaluate()` hold-before-t_start mechanism keeps the platform in place until motion begins

**Key consideration:** When deferring from an active trajectory, the sampled state at `t_start` must exactly match what `evaluate()` will return at that future time. For IDLE/COMPLETE states this is trivial (hold pose). For EXECUTING, it requires evaluating the active trajectory at the future `t_start` — which is valid as long as the active trajectory hasn't completed by then.

**Files to modify:**

- `ros_ws/src/jugglebot/jugglebot/motion/trajectory.py` — `submit_dynamic_target()` and `request_dynamic_target()`
- `ros_ws/src/jugglebot/jugglebot/motion/tests/test_dynamic_target.py` — update test 13

**Verification:** Offline test with far-future target: verify the platform holds, then moves at moderate speed. Hardware test: confirm no position discontinuity at the hold-to-move transition.

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

## 5. Publish Tracking Error and Motor Feedback to ROS2

**Problem:** `tracking_error_mm` is computed in the control loop and included in IPC telemetry, but the bridge does not publish it to any ROS2 topic. It is lost at the IPC→ROS2 boundary and cannot be captured by rosbag. Same applies to motor feedback (encoder positions), condition number, and workspace status.

**Proposed change:** Add ROS2 publishers in `motion_bridge_node.py` for diagnostic data:

| New Topic | Message Type | Content |
|---|---|---|
| `/motion/tracking_error` | `Float64MultiArray` | 6 per-leg tracking errors (mm) |
| `/motion/motor_feedback` | `Float64MultiArray` | 18 values: 6 positions + 6 velocities + 6 currents |
| `/motion/diagnostics` | `Float64MultiArray` | condition number, workspace status, workspace speed scale |

**Files to modify:**

- `ros_ws/src/jugglebot/jugglebot/motion_bridge_node.py` — add publishers, extract fields from telemetry in `_poll_telemetry()`
- `ros_ws/src/jugglebot/launch/jugglebot_launch.py` — add new topics to rosbag recording list

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

## 7. Update `connect()` Documentation for Context Manager

**Problem:** The docs say "always call `connect()` explicitly" but `PlatformTestHarness` supports a context manager pattern (`with harness:`) which calls `connect()` automatically via `__enter__()`.

**Proposed change:** Update the `operations.md` danger box to mention both patterns:

```python
# Pattern A: Context manager (auto-connects on entry, auto-disconnects on exit)
with PlatformTestHarness() as harness:
    harness.home_all_axes()
    # ...

# Pattern B: Explicit connect/disconnect
harness = PlatformTestHarness()
harness.connect()
# ...
harness.shutdown()
```

**Files to modify:**

- `docs/motion_planner/operations.md` — update the danger box and code example

**Note:** This is a docs-only change but is listed here because the user asked for it to be included in this document.
