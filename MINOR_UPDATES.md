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

## ~~3. Re-implement Deferred Start for Dynamic Targets~~ ✅ DONE (2026-03-10)

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

**Implementation notes:**

- `DEFERRED_START_BUFFER_S = 2.0` constant added to `trajectory.py`
- `submit_dynamic_target()`: calls `find_min_feasible_duration()` to decide deferral; samples state at deferred `t_start` for splice continuity
- `request_dynamic_target()` / `_bg_check_feasibility()`: bg thread also computes `min_feasible` alongside the existing feasibility check
- `commit_async_trajectory()`: applies deferred start using `min_feasible` from the bg result
- Test 13 updated: now verifies deferred start (hold phase → moderate-speed motion → on-time arrival)
- Test 15 added: short-duration target starts immediately (no deferral)
- Test 16 added: zero discontinuity at hold-to-move transition (the critical safety test)
- All 16 dynamic target tests PASS (test 10 has pre-existing 7e-6 rad quat precision issue, unrelated)

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

## ~~7. Update `connect()` Documentation for Context Manager~~ ✅ DONE (2026-03-10)

**Change:** Updated the `operations.md` PlatformTestHarness section to show both the context manager pattern (Pattern A) and explicit connect/disconnect pattern (Pattern B). Also fixed the danger box wording and corrected `shutdown()` → `disconnect()` (the actual method name).

**Files modified:**

- `docs/motion_planner/operations.md` — updated code example and danger box
