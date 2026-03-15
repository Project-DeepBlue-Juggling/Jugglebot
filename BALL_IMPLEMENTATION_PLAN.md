# Ball Tracking & Catch Coordination — Implementation Plan

## Context

Phase 6 of the Jugglebot rewrite connects ball perception to the motion planner's dynamic target API. The platform needs to detect balls (from Ball Butler throws and human throws), predict where they'll land, and command the platform to catch them.

The motion planner's dynamic target API is already built and hardware-tested (Phase 7 of motion planner). It accepts `(target_pos, target_quat, target_vel, arrival_time)` and handles feasibility checking, trajectory generation, mid-motion replanning with C2 continuity, and auto-decelerate-to-stop. This phase produces the inputs that API consumes.

---

## Architecture

Three cleanly separated concerns — perception, policy, and execution:

```
Mocap (200Hz markers)  ──►  Ball Tracker Node  ──►  /balls (BallStateArray)
BB heartbeat (throw)   ──┘     (perception)              │
                                                          ▼
                                              Catch Coordinator Node  ──► dynamic_target IPC
                                                 (control policy)              │
                                                                               ▼
                                                                    Motion Planner (already built)
```

### New Components

| Component | Location | Pattern | Purpose |
|-----------|----------|---------|---------|
| `tracking/` subpackage | `jugglebot/tracking/` | Pure Python (no ROS2) | Kalman filter, ballistic prediction, marker matching, ball lifecycle |
| `ball_tracker_node.py` | `jugglebot/ball_tracker_node.py` | Thin ROS2 wrapper | Subscribes `/mocap_data` + `/throw_announcements`, publishes `/balls` |
| `catch_coordinator.py` | `jugglebot/catch_coordinator.py` | Pure Python | Policy: which ball to catch, catch pose computation, blacklist management |
| `catch_coordinator_node.py` | `jugglebot/catch_coordinator_node.py` | ROS2 wrapper | Subscribes `/balls`, sends dynamic targets via IPC to motion planner |

### Package Structure

```
ros_ws/src/jugglebot/jugglebot/
  tracking/
    __init__.py
    ball.py              # Ball dataclass, BallStatus enum, TrackingConfidence enum
    kalman.py            # 6D Kalman filter (pos + vel) with gravity model
    ballistics.py        # Parabolic prediction: current state → landing state at landing_z
    matcher.py           # Marker-to-ball matching, parabolic motion detection
    tests/
      __init__.py
      test_kalman.py
      test_ballistics.py
      test_matcher.py
      test_lifecycle.py
  ball_tracker_node.py   # ROS2 wrapper for tracking/
  catch_coordinator.py   # Pure Python catch policy
  catch_coordinator_node.py  # ROS2 wrapper for coordinator
```

---

## Ball Object

### Data Model

```python
from enum import IntEnum
from dataclasses import dataclass, field
import numpy as np

class BallStatus(IntEnum):
    TO_BE_THROWN = 0   # Announced by thrower, not yet thrown
    IN_FLIGHT    = 1   # Ball is airborne (time has passed throw_time, or parabolic motion detected)
    CAUGHT       = 2   # Ball landed on platform (inferred from timeout + marker disappearance)
    DROPPED      = 3   # Ball missed / bounced off
    UNKNOWN      = 4   # Lost track, no resolution

class TrackingConfidence(IntEnum):
    ANNOUNCED = 0      # Only from throw announcement, no mocap confirmation yet
    CONFIRMED = 1      # Matched to mocap marker, Kalman filter is being updated

@dataclass
class Ball:
    id: int                              # Monotonic per session, never reused
    status: BallStatus                   # Current lifecycle state
    tracking: TrackingConfidence         # Whether mocap has confirmed this ball
    source: str                          # "ball_butler", "human_throw"
    destination: str                     # Robot name (e.g. "jugglebot"), empty = unassigned

    # Current state (Kalman-filtered if CONFIRMED, predicted from announcement if ANNOUNCED)
    position: np.ndarray                 # [x, y, z] mm, in base frame
    velocity: np.ndarray                 # [vx, vy, vz] mm/s
    timestamp: float                     # ROS2 time (seconds) of last state update

    # Predicted landing state (ballistic projection to landing_z plane)
    landing_position: np.ndarray         # [x, y, z] mm
    landing_velocity: np.ndarray         # [vx, vy, vz] mm/s
    landing_time: float                  # ROS2 time (seconds), absolute

    # Internal tracking state (not published in ROS2 message)
    kalman_state: object = None          # KalmanFilter instance (None if ANNOUNCED-only)
    frames_tracked: int = 0              # Number of mocap frames matched to this ball
    last_seen: float = 0.0               # ROS2 time of last mocap marker match
    throw_time: float = 0.0              # Expected throw time (from announcement)
```

### Status Lifecycle

```
ANNOUNCED BALLS (Ball Butler path):
  ThrowAnnouncement received       →  TO_BE_THROWN  (tracking=ANNOUNCED)
  now >= throw_time                 →  IN_FLIGHT     (tracking=ANNOUNCED)
  Mocap marker matched              →  IN_FLIGHT     (tracking=CONFIRMED)
  landing_time + 200ms, marker gone →  CAUGHT
  Marker reappears on new path      →  DROPPED
  Timeout, no resolution            →  UNKNOWN

UNANNOUNCED BALLS (human throw path):
  Parabolic marker detected (3+ frames)  →  IN_FLIGHT  (tracking=CONFIRMED)
  landing_time + 200ms, marker gone      →  CAUGHT
  Marker reappears on new path           →  DROPPED
  Timeout, no resolution                 →  UNKNOWN
```

**Key insight**: The transition from `TO_BE_THROWN` to `IN_FLIGHT` does NOT require mocap confirmation. Once `now >= throw_time`, the ball is in flight regardless of whether mocap has seen it. This handles balls that are partially or fully occluded during flight. The `tracking` field separately tracks whether mocap has confirmed the ball.

### Ball Cleanup

Balls in terminal states (`CAUGHT`, `DROPPED`, `UNKNOWN`) are retained for a configurable duration (e.g. 2 seconds) for downstream consumers to observe the final state, then removed from the active list.

---

## Kalman Filter

### Existing Implementation

A working Kalman filter exists at `jugglebot/archived/kalman_filter.py` (230 lines). Lift and clean up for `tracking/kalman.py`.

### Specification

- **State**: 6D — `[x, y, z, vx, vy, vz]` (mm, mm/s)
- **Measurement**: 3D position `[x, y, z]` from mocap markers
- **Process model**: constant velocity + gravity control input
  - Gravity: `-9810.0 mm/s²` applied via control matrix `B` (canonical value from `hardware_config`)
  - `B` includes `0.5·dt²` position coupling and `dt` velocity coupling
- **Process noise Q**: `eye(6) * process_noise` (default `1.0`)
- **Measurement noise R**: `eye(3) * measurement_noise` (default `3.0`)
- **Initial covariance**: `eye(6) * 500.0` (large initial uncertainty)

### Initialization Modes

1. **From announcement**: `initialize_with_state([x, y, z, vx, vy, vz])` using ThrowAnnouncement's `initial_position` and `initial_velocity`. High initial covariance since BB position estimate may have error.

2. **From mocap detection**: Initialize from first 2-3 marker positions, with velocity estimated by finite difference. Tighter initial covariance on position, wider on velocity.

**Note**: ThrowAnnouncement publication must be implemented. The archived `ball_butler_node.py` published announcements; in the new architecture, `can_node.py` needs to detect the BB THROWING state transition and publish ThrowAnnouncement on `/throw_announcements`. Verify this exists or implement it during this phase.

### Landing Prediction

`predict_landing_state(ground_z)` solves the quadratic:
```
z + vz·t + 0.5·(-9810)·t² = ground_z
```

Returns `(landing_pos_xy, landing_vel_xyz, time_to_land)` or `None` if no real positive solution.

**Landing Z plane**: Configurable, default = `GEOM_INITIAL_HEIGHT_MM + 160.0` = 734.3 mm. This will become adaptive in the future (set by the motion planner based on hand position), but a fixed offset is sufficient for initial implementation.

---

## Marker-to-Ball Matching

### Parabolic Motion Detection (for human throws)

1. Buffer the last 4 frames of each unlabelled marker using frame-to-frame nearest-neighbour tracking
2. For each candidate with 3+ consecutive frames, compute acceleration via finite difference:
   ```
   a_measured = (v[n] - v[n-1]) / dt
   a_expected = [0, 0, -9810]  # mm/s²
   ```
3. If `||a_measured - a_expected|| < threshold` (e.g. 3000 mm/s²), flag as parabolic → create Ball with `source="human_throw"`

### Announced Ball Matching

When a new ball is confirmed as parabolic (or even just a new marker near predicted trajectory):

1. For each `TO_BE_THROWN` or `IN_FLIGHT` ball with `tracking=ANNOUNCED`:
   - Propagate the announcement's initial state to current time using ballistic equations
   - Compare predicted position with observed marker position
   - If distance < adaptive threshold → match

2. **Adaptive threshold**: Base = 100mm, scales up with:
   - Time since throw (trajectory divergence accumulates)
   - Ball speed (faster balls have more uncertainty)
   - The archived `ball_prediction_node.py` `_try_match_pending()` has tuned values to reference

3. If no announcement matches → ball is `source="human_throw"`

### Frame-to-Frame Association

Each tracking cycle, existing `CONFIRMED` balls need to be matched to current mocap markers:
- Use nearest-neighbour with gating: predicted position (from Kalman predict step) vs observed markers
- Gate radius: e.g. 50mm (ball shouldn't move more than this in 5ms at typical velocities)
- Unmatched balls: increment a "missed frames" counter. After N missed frames (e.g. 10 = 50ms), if past landing_time → transition to CAUGHT/UNKNOWN

---

## Catch Coordinator

### Responsibilities

1. **Filter**: Only consider balls with `destination == "jugglebot"` (or own robot name)
2. **Select**: Pick the best catchable ball (earliest landing time, within workspace bounds)
3. **Compute catch pose**: Position from `landing_position`, orientation from `landing_velocity`
4. **Submit dynamic target**: Convert to motion planner format and send via IPC
5. **Update**: Re-submit on each ball state update (planner handles C2 splice replanning)
6. **Manage blacklist**: Track consecutive feasibility rejections per ball

### Catch Pose Computation

**Position**: `[landing_x, landing_y, catch_z - initial_height]` (convert to platform frame)

**Orientation**: Quaternion such that platform normal aligns with incoming ball velocity direction.

From archived `catch_thrown_ball_node.py`:
```python
reference = [0, 0, -1]            # Platform Z-axis (downward)
v_norm = landing_velocity / ||landing_velocity||
angle = arccos(dot(reference, v_norm))

if angle > catch_angle_limit (30°):
    return None  # Uncatchable angle

axis = cross(reference, v_norm)
axis = axis / ||axis||
quat = Rotation.from_rotvec(angle * axis).as_quat()  # scipy: [x, y, z, w]
# Convert to [w, x, y, z] for dynamic target API
```

**Important**: The dynamic target API expects quaternion as `[w, x, y, z]`. The scipy `as_quat()` returns `[x, y, z, w]`. The coordinator must reorder.

### Feasibility Blacklist

The coordinator maintains an internal blacklist of ball IDs it has given up on:

- After **3 consecutive** feasibility rejections for a ball → add to blacklist, stop submitting dynamic targets
- The ball's `status` and `destination` fields are **unchanged** — the tracker is pure perception with no feedback channel
- **Re-evaluation escape hatch**: If a blacklisted ball's `landing_position` shifts by more than 50mm from the position recorded at blacklist time → remove from blacklist and re-evaluate. This handles early Kalman filter uncertainty causing an initially-infeasible prediction that later converges to a feasible landing point.
- Blacklist entries are cleaned up when the corresponding ball transitions out of `IN_FLIGHT`

### How Feasibility Rejection is Detected

The coordinator sends a dynamic target via IPC. The motion planner's `TrajectoryManager` performs background feasibility checking:
- `request_dynamic_target()` returns `False` if lead time < 300ms (`MIN_LEAD_TIME_S`)
- `poll_pending_result()` returns `{'accepted': False}` if trajectory fails workspace/vel/accel/jerk/cond checks

The coordinator needs feedback on whether its target was accepted. Options:
1. **Telemetry polling**: The control loop publishes telemetry including trajectory state — coordinator can infer acceptance
2. **Dedicated feedback topic**: Control loop publishes accept/reject on a new IPC topic
3. **Timeout-based**: If no trajectory state change within N ms of submission, assume rejected

Option 2 is cleanest. This requires a small addition to `motion/ipc.py` and `motion/control_loop.py`.

---

## Clock Domain & Time Conversion

### The Problem

Three clock domains are in play:
- **ROS2 time** (`get_clock().now()`): Used by all ROS2 nodes, ball tracker, mocap timestamps
- **perf_counter**: Used by the motion control process for trajectory timing
- **QTM time**: Mocap system clock, converted to ROS2 time by `mocap_interface.py` (exponential smoothing, <10ms accuracy)

The dynamic target API's `arrival_time` parameter is in **perf_counter** domain (see `motion/ipc.py` line 213).

### Solution

The catch coordinator node must convert ROS2 landing_time → perf_counter arrival_time.

**Approach**: Measure the offset `perf_counter() - ros2_time_seconds()` at startup (average over a few samples). This offset is stable within a process lifetime (both are monotonic-ish on Linux; ROS2 uses wall clock by default but the offset changes slowly).

```python
# At startup:
offsets = []
for _ in range(10):
    t_perf = time.perf_counter()
    t_ros = node.get_clock().now().nanoseconds / 1e9
    offsets.append(t_perf - t_ros)
self._ros_to_perf_offset = np.median(offsets)

# At submission time:
arrival_perf = ball.landing_time + self._ros_to_perf_offset
```

This gives sub-millisecond accuracy, well within the 10ms requirement.

---

## ROS2 Message Updates

### Existing Messages (keep as-is)

The existing `BallState.msg` already has the right fields for publishing tracked balls:
```
std_msgs/Header header
uint32 id
string source
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
string target_id              # → will use for destination
geometry_msgs/Point target_position
geometry_msgs/Point landing_position
geometry_msgs/Vector3 landing_velocity
builtin_interfaces/Time time_at_land
```

### New Fields Needed

Add to `BallState.msg`:
```
uint8 status                  # BallStatus enum (0=TO_BE_THROWN, 1=IN_FLIGHT, 2=CAUGHT, 3=DROPPED, 4=UNKNOWN)
uint8 tracking                # TrackingConfidence enum (0=ANNOUNCED, 1=CONFIRMED)
string destination            # Robot this ball is heading to (empty = unassigned)
```

**Note**: The existing `target_id` field could be repurposed for `destination`, but its semantics are different (target_id was "what the throw was aimed at", destination is "who should catch it"). Keep both — `target_id` comes from the ThrowAnnouncement, `destination` is set by the tracker/coordinator.

### Existing Messages Used As-Is

- `ThrowAnnouncement.msg` — published by can_node when BB enters THROWING state
- `BallStateArray.msg` — `BallState[] balls`, published on `/balls`
- `MocapDataMulti.msg` / `MocapDataSingle.msg` — marker positions from mocap node
- `BallButlerHeartbeat.msg` — BB state, yaw/pitch/hand positions

---

## Data Flow — Detailed

### Ball Butler Throw Path

1. Ball Butler CAN heartbeat transitions to `THROWING` state
2. `can_node.py` detects state change, publishes `ThrowAnnouncement` on `/throw_announcements`
   - `initial_position`: BB position in global frame (from calibration)
   - `initial_velocity`: computed from throw parameters (yaw, pitch, speed)
   - `throw_time`: when the ball will leave the hand
   - `landing_position`, `landing_velocity`, `landing_time`: pre-computed from ballistics
3. `ball_tracker_node.py` receives announcement:
   - Creates `Ball(status=TO_BE_THROWN, tracking=ANNOUNCED, source="ball_butler")`
   - Initializes Kalman filter from `initial_position` and `initial_velocity`
   - Sets `throw_time` from announcement
   - Landing state initialized from announcement's pre-computed values
4. Each tracking cycle (`~200 Hz`):
   - If `now >= throw_time` and `status == TO_BE_THROWN` → transition to `IN_FLIGHT`
   - Kalman filter predict step (propagate state with gravity)
   - If mocap marker matches (spatial proximity to predicted position):
     - `tracking = CONFIRMED`
     - Kalman filter update step with measured position
     - Landing prediction recomputed from filtered state
   - If no match: state propagates from last known (landing prediction still valid, just not refined)
5. Published on `/balls` at mocap rate

### Human Throw Path

1. `matcher.py` monitors all unlabelled mocap markers
2. Detects parabolic motion (3-4 consecutive frames with gravity-consistent acceleration)
3. Creates `Ball(status=IN_FLIGHT, tracking=CONFIRMED, source="human_throw")`
   - `destination` assigned via trajectory analysis: if the ball's predicted landing position is within the platform's reachable workspace, set `destination = "jugglebot"`. Otherwise leave empty.
   - Landing state computed from current Kalman filter estimate
4. Same tracking cycle as above

### Catch Path

1. `catch_coordinator_node.py` receives `BallStateArray` on `/balls`
2. `catch_coordinator.py` processes:
   - Filter: only balls with `destination == "jugglebot"` and `status == IN_FLIGHT`
   - Skip blacklisted ball IDs (unless landing_position has shifted >50mm from blacklist snapshot)
   - Select best candidate: earliest `landing_time` that passes preliminary feasibility (within XY range, sufficient lead time)
3. Compute catch pose:
   - Position: `[landing_x, landing_y, catch_z - initial_height]` in platform frame
   - Orientation: quaternion normal to `landing_velocity` (see algorithm above)
   - Velocity: `[0, 0, 0]` (platform should be stationary at catch — ball velocity is absorbed by compliance, not matched)
4. Convert to dynamic target:
   ```python
   target_pos = [landing_x, landing_y, catch_z_offset]
   target_quat = [w, x, y, z]  # from catch orientation computation
   target_vel = [0, 0, 0]      # stationary catch
   arrival_time = landing_time + ros_to_perf_offset  # clock domain conversion
   ```
5. Send via IPC: `bridge.send_dynamic_target(make_dynamic_target_command(...))`
6. On each subsequent `/balls` update: re-send with refined prediction
   - Motion planner handles C2 splice replanning automatically
   - If planner rejects (infeasible), increment rejection counter for blacklist
7. Hand positioning: command hand height via separate topic/IPC based on `landing_time` and `landing_velocity`. The hand only moves vertically (up/down relative to the platform) — there is no open/close mechanism.

---

## Key Constants & Parameters

| Parameter | Value | Source |
|-----------|-------|--------|
| Gravity | -9810.0 mm/s² | `hardware_config.py` (GRAVITY_MMPS2) |
| Mocap rate | 200 Hz (dt = 5ms) | `hardware_config.py` (TRACKING_MOCAP_DT_S) |
| Platform initial height | 574.3 mm | `hardware_config.py` (GEOM_INITIAL_HEIGHT_MM) |
| Landing Z offset | 160.0 mm | Configurable, will become adaptive |
| Landing Z plane | 734.3 mm | initial_height + offset (default) |
| KF process noise | 1.0 | Tunable, from archived implementation |
| KF measurement noise | 3.0 | Tunable, from archived implementation |
| KF initial covariance | 500.0 | Large initial uncertainty |
| Parabolic detection frames | 3-4 | Minimum frames to confirm parabolic motion |
| Parabolic accel threshold | ~3000 mm/s² | Tolerance on gravity-consistency check |
| Marker match gate radius | 50 mm | Frame-to-frame association for tracked balls |
| Announced match base threshold | 100 mm | Spatial proximity for announcement matching |
| Catch angle limit | 30° | Max platform tilt angle for catch |
| Caught grace period | 200 ms | Timeout after expected landing before declaring caught |
| Missed frames to lose track | 10 (50ms) | Mocap frames without match before ball considered lost |
| Terminal state retention | 2 seconds | How long CAUGHT/DROPPED/UNKNOWN balls stay in list |
| Blacklist rejection threshold | 3 consecutive | Feasibility rejections before giving up |
| Blacklist re-eval threshold | 50 mm | Landing position shift to re-evaluate blacklisted ball |
| Min lead time (planner) | 300 ms | `MIN_LEAD_TIME_S` in trajectory_manager.py |
| Splice lead time (planner) | 200 ms | `SPLICE_LEAD_TIME_S` in trajectory_manager.py |

---

## Existing Code to Reuse

| What | Where | Notes |
|------|-------|-------|
| Kalman filter | `archived/kalman_filter.py` (230 lines) | Lift to `tracking/kalman.py`, clean up |
| Landing prediction | `archived/kalman_filter.py` `predict_landing_state()` | Quadratic solver for ballistic landing |
| Ball prediction node logic | `archived/ball_prediction_node.py` (470 lines) | Matching logic, PendingBall/TrackedBall pattern |
| Catch orientation | `archived/catch_thrown_ball_node.py` `calculate_catch_orientation()` | Velocity-to-quaternion, angle limit check |
| Preliminary feasibility | `archived/catch_thrown_ball_node.py` `check_preliminary_catch_feasibility()` | XY range, time-to-land checks |
| Dynamic target IPC | `motion/ipc.py` `make_dynamic_target_command()` | Already built, use directly |
| IPC bridge | `motion/ipc.py` `BridgeIPC.send_dynamic_target()` | Already built |
| ROS2 messages | `jugglebot_interfaces/msg/BallState.msg` etc. | Extend with status/tracking/destination fields |

---

## Implementation Considerations

### Quaternion Convention

- **scipy** `Rotation.as_quat()` returns `[x, y, z, w]`
- **Dynamic target API** expects `[w, x, y, z]`
- **ROS2 Quaternion.msg** uses fields `x, y, z, w`
- The catch coordinator must be explicit about conversions. Suggest a helper function.

### Coordinate Frames

- **Mocap markers**: Base frame (world frame shifted by `GEOM_INITIAL_HEIGHT_MM` on Z)
- **Ball positions**: Same base frame (mm)
- **Dynamic target `target_pos`**: Platform offset from home position (mm). This is relative to the home/center position, NOT absolute world coordinates.
  - `target_pos = [landing_x, landing_y, landing_z - initial_height]` approximately, but verify against how the motion planner defines "home"
- **Landing Z plane**: `GEOM_INITIAL_HEIGHT_MM + 160.0` mm (734.3 mm). Configurable; will become adaptive in the future.

### Thread Safety

The ball tracker node processes mocap data at 200Hz. If the ROS2 executor is multi-threaded:
- The `Ball` objects and active ball list must be thread-safe
- Use a lock or a copy-on-publish pattern
- The archived code was single-threaded; the new design should be too (single-threaded executor) unless there's a performance reason

### Performance

- 200Hz mocap → 200Hz tracking loop → 200Hz publish. Each cycle:
  - Kalman predict: O(1) per ball (6x6 matrix ops)
  - Kalman update: O(1) per ball
  - Matching: O(N·M) where N=active balls, M=markers. Typically N<5, M<20 → negligible
  - Landing prediction: O(1) per ball (quadratic solve)
- Total per cycle: well under 1ms. No performance concerns.

---

## Testing Plan

### Offline Tests (`tracking/tests/`)

1. **test_kalman.py**
   - Filter convergence on synthetic parabolic trajectory (known initial state, verify position/velocity converge)
   - Measurement noise rejection (add Gaussian noise to positions, verify filter smooths)
   - Gravity model correctness (predict without measurements, compare to analytical ballistic trajectory)
   - Landing prediction accuracy (known throw parameters → predicted landing vs analytical)

2. **test_ballistics.py**
   - Simple vertical throw: known v₀, verify time-to-land and landing velocity
   - Angled throw: verify XY landing position matches analytical solution
   - Edge cases: ball already below landing plane, ball moving upward (not yet at apex)

3. **test_matcher.py**
   - Parabolic detection: synthetic marker stream with 1 parabolic + N static markers → correctly identifies ball
   - Announced matching: create announced ball, feed markers near predicted trajectory → matched
   - Announced matching: feed markers far from predicted trajectory → not matched, becomes human_throw
   - Frame-to-frame association: tracked ball with occasional missed frames → maintains identity

4. **test_lifecycle.py**
   - TO_BE_THROWN → IN_FLIGHT transition at throw_time (time-based, no mocap needed)
   - IN_FLIGHT → CAUGHT (marker disappears after landing_time + grace)
   - IN_FLIGHT → DROPPED (marker reappears on new trajectory)
   - IN_FLIGHT → UNKNOWN (timeout with no resolution)
   - Terminal state cleanup after retention period

5. **test_coordinator.py**
   - Catch pose computation: known landing velocity → correct quaternion orientation
   - Angle limit: steep approach angle → returns None (uncatchable)
   - Blacklist: 3 consecutive rejections → ball blacklisted, no more submissions
   - Blacklist re-eval: landing position shifts >50mm → removed from blacklist
   - Multi-ball: selects earliest feasible ball

### Integration Tests (with mocap + BB hardware)

1. BB throw → ball tracked → landing prediction within ±50mm of actual landing
2. Human throw → detected within 4 frames (~20ms) → prediction converges
3. Catch coordinator sends dynamic target → planner accepts → platform moves to catch position
4. Occluded ball: BB throw where ball is partially occluded → still transitions to IN_FLIGHT via time-based transition, landing prediction from announcement is used

### End-to-End

1. Full catch sequence: BB throw → track → predict → platform moves → catch
2. Timing accuracy: platform arrives within ±20ms of predicted landing time
3. Human throw catch: throw ball by hand → detected → caught
4. Infeasible ball: throw to unreachable location → blacklisted after 3 rejections → platform stays home

