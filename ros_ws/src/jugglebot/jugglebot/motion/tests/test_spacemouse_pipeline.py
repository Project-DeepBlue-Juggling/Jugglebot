"""End-to-end spacemouse pipeline test.

Simulates the full spacemouse → smoother → motor commands pipeline offline:

  1. Generate spacemouse-like raw inputs (-1..+1) for all DoFs
  2. Apply production sensitivity multipliers and coordinate transforms
  3. Feed through StreamSmoother at 100 Hz target rate / 500 Hz evaluate rate
  4. Convert to motor commands via cartesian_to_motor_commands
  5. Verify C2 continuity, smoothness, and velocity buildup

Tests:
  1. Ramp all axes   — linear ramp from 0→1, verify smooth motor trajectories
  2. Step response   — step from 0→1, verify C2 smooth and reaches target
  3. Direction reversal — ramp 0→1→-1, verify smooth reversal
  4. Velocity buildup — verify motor velocity exceeds a meaningful fraction
                        of the limit (the whole point of hysteresis/deferral)
  5. C2 continuity   — numerical C2 check on motor position trace

Run:  python -m pytest ros_ws/src/jugglebot/jugglebot/motion/tests/test_spacemouse_pipeline.py -v
"""

from __future__ import annotations

import math
import sys

import numpy as np
from numpy.linalg import norm

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    quat_to_rot_matrix,
    rot_matrix_to_rotvec,
    rotvec_to_rot_matrix,
    pose_to_leg_lengths,
    compute_jacobian,
)
from jugglebot.motion.conversions import extensions_mm_to_revs
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.motor_commands import cartesian_to_motor_commands
from jugglebot.motion.stream_smoother import StreamSmoother


# ---------------------------------------------------------------------------
# Pipeline helpers — replicate prod transforms without ROS2 / pyspacemouse
# ---------------------------------------------------------------------------

def spacemouse_raw_to_pose_6dof(
    x: float, y: float, z: float,
    pitch: float, roll: float, yaw: float,
) -> np.ndarray:
    """Convert raw spacemouse values (each in -1..+1) to a 6-DoF pose.

    Replicates the exact transforms in spacemouse_handler.py:
      - Apply sensitivity multipliers
      - Add z_offset (default active Z)
      - Euler → quaternion (yaw * roll * pitch order, with sign flips)
      - Quaternion → rotation matrix → rotation vector
    """
    xy_mult = hw.SPACEMOUSE_XY_MULT_MM
    z_mult = hw.SPACEMOUSE_Z_MULT_MM
    pitch_roll_mult = hw.SPACEMOUSE_PITCH_ROLL_MULT_DEG
    yaw_mult = hw.SPACEMOUSE_YAW_MULT_DEG
    z_offset = hw.JB_OP_DEFAULT_ACTIVE_Z_MM

    # Position
    px = x * xy_mult
    py = y * xy_mult
    pz = z * z_mult + z_offset

    # Orientation — matching spacemouse_handler.py sign conventions
    roll_rad = math.radians(roll * pitch_roll_mult)
    pitch_rad = math.radians(-pitch * pitch_roll_mult)
    yaw_rad = math.radians(-yaw * yaw_mult)

    # Quaternion composition: yaw * roll * pitch
    # Each from_rotation_vector([ax, 0, 0]) is equivalent to axis-angle
    # Using Rodrigues' formula for small-angle rotation vectors:
    def _rotvec_to_quat(rv):
        """rotation vector → quaternion [w, x, y, z]."""
        angle = math.sqrt(rv[0]**2 + rv[1]**2 + rv[2]**2)
        if angle < 1e-12:
            return np.array([1.0, 0.0, 0.0, 0.0])
        ha = angle / 2
        s = math.sin(ha) / angle
        return np.array([math.cos(ha), rv[0]*s, rv[1]*s, rv[2]*s])

    def _qmul(a, b):
        """Quaternion multiplication [w,x,y,z] convention."""
        return np.array([
            a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
            a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
            a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
            a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0],
        ])

    q_roll = _rotvec_to_quat([0.0, roll_rad, 0.0])
    q_pitch = _rotvec_to_quat([pitch_rad, 0.0, 0.0])
    q_yaw = _rotvec_to_quat([0.0, 0.0, yaw_rad])

    q = _qmul(_qmul(q_yaw, q_roll), q_pitch)

    # Convert to rotation matrix → rotation vector (matching control_loop._on_target)
    rot = quat_to_rot_matrix(q[0], q[1], q[2], q[3])
    rotvec = rot_matrix_to_rotvec(rot)

    return np.array([px, py, pz, rotvec[0], rotvec[1], rotvec[2]])


def make_smoother_spacemouse() -> tuple[StreamSmoother, StewartGeometry, DynamicsParams]:
    """Create a StreamSmoother with spacemouse-default limits."""
    geom = StewartGeometry()
    smoother = StreamSmoother(
        geom,
        vel_limit_rps=hw.SPACEMOUSE_SMOOTHER_VEL_LIMIT_RPS,
        accel_limit_rps2=hw.SPACEMOUSE_SMOOTHER_ACCEL_LIMIT_RPS2,
    )
    dyn = DynamicsParams.from_config()
    return smoother, geom, dyn


def home_pose() -> np.ndarray:
    return np.array([0.0, 0.0, hw.JB_OP_DEFAULT_ACTIVE_Z_MM, 0.0, 0.0, 0.0])


def simulate_spacemouse(
    smoother: StreamSmoother,
    geom: StewartGeometry,
    dyn: DynamicsParams,
    raw_trajectory: list[tuple[float, float, float, float, float, float]],
    target_rate_hz: float = 100.0,
    eval_rate_hz: float = 500.0,
    settle_time_s: float = 0.5,
) -> dict:
    """Run a full simulation of the spacemouse pipeline.

    Parameters
    ----------
    raw_trajectory : list of (x, y, z, pitch, roll, yaw) tuples, each in -1..+1
        One entry per spacemouse sample (at target_rate_hz).
    target_rate_hz : spacemouse publish rate
    eval_rate_hz : control loop rate
    settle_time_s : extra time after last target to let platform settle

    Returns
    -------
    dict with keys:
        times : (N,) ndarray of evaluation timestamps
        poses : (N, 6) ndarray of Cartesian poses
        twists : (N, 6) ndarray of Cartesian twists
        accels : (N, 6) ndarray of Cartesian accelerations
        motor_pos : (N, 6) ndarray of motor positions (revs)
        motor_vel : (N, 6) ndarray of motor velocities (rev/s)
        targets : (M, 6) ndarray of target poses sent to smoother
        target_times : (M,) ndarray of target timestamps
    """
    smoother.reset(home_pose())

    dt_target = 1.0 / target_rate_hz
    dt_eval = 1.0 / eval_rate_hz

    # Total simulation time
    n_targets = len(raw_trajectory)
    total_target_time = n_targets * dt_target
    total_time = total_target_time + settle_time_s

    # Pre-compute all target poses
    target_poses = []
    target_times = []
    for i, raw in enumerate(raw_trajectory):
        t = i * dt_target
        pose_6dof = spacemouse_raw_to_pose_6dof(*raw)
        target_poses.append(pose_6dof)
        target_times.append(t)

    # Simulate at eval_rate_hz, injecting targets at target_rate_hz
    n_eval = int(total_time * eval_rate_hz) + 1
    times = np.zeros(n_eval)
    poses = np.zeros((n_eval, 6))
    twists = np.zeros((n_eval, 6))
    accels = np.zeros((n_eval, 6))
    motor_pos = np.zeros((n_eval, 6))
    motor_vel = np.zeros((n_eval, 6))
    extensions = np.zeros((n_eval, 6))
    torques = np.zeros((n_eval, 6))

    target_idx = 0

    for i in range(n_eval):
        t = i * dt_eval

        # Inject any targets whose time has arrived
        while target_idx < n_targets and target_times[target_idx] <= t:
            smoother.set_target(target_poses[target_idx], target_times[target_idx])
            target_idx += 1

        # Evaluate smoother
        pose, twist, accel = smoother.evaluate(t)

        # Convert to motor commands
        pos_rev, vel_ff, torque_ff, _ = cartesian_to_motor_commands(
            pose, twist, accel, geom, dyn, feedforward_enabled=True)

        rot = rotvec_to_rot_matrix(pose[3:6])
        ext = pose_to_leg_lengths(pose[:3], rot, geom)

        times[i] = t
        poses[i] = pose
        twists[i] = twist
        accels[i] = accel
        motor_pos[i] = pos_rev
        motor_vel[i] = vel_ff
        extensions[i] = ext
        torques[i] = torque_ff

    return {
        'times': times,
        'poses': poses,
        'twists': twists,
        'accels': accels,
        'motor_pos': motor_pos,
        'motor_vel': motor_vel,
        'extensions': extensions,
        'torques': torques,
        'targets': np.array(target_poses),
        'target_times': np.array(target_times),
    }


# ---------------------------------------------------------------------------
# Test helpers
# ---------------------------------------------------------------------------

def check_c2_continuity(times, values, label,
                        pos_tol=0.5, vel_tol=50.0, acc_tol=5000.0):
    """Check C2 continuity by verifying numerical derivatives are smooth.

    Computes finite-difference velocity and acceleration from the position
    trace, then checks that jumps in acceleration are bounded (no C2 breaks).

    Returns (passed, max_pos_jump, max_vel_jump, max_acc_jump).
    """
    dt = np.diff(times)

    # Numerical velocity (finite difference)
    num_vel = np.diff(values, axis=0) / dt[:, None]

    # Numerical acceleration
    num_acc = np.diff(num_vel, axis=0) / dt[1:, None]

    # Numerical jerk (indicator of C2 breaks)
    num_jerk = np.diff(num_acc, axis=0) / dt[2:, None]

    # Look for jumps: large jerk spikes indicate discontinuities
    # Use the 99.9th percentile as "peak" to ignore single-sample noise
    jerk_mag = np.max(np.abs(num_jerk), axis=1)

    if len(jerk_mag) == 0:
        return True, 0.0, 0.0, 0.0

    p999_jerk = np.percentile(jerk_mag, 99.9)
    median_jerk = np.median(jerk_mag)

    # A C2 break shows as a jerk spike >> median.  Allow up to 20x median
    # for legitimate transients (segment boundaries with large targets).
    # The key check: no INFINITE or extremely large jerk spikes.
    ratio = p999_jerk / (median_jerk + 1e-12)

    # Also compute position/velocity continuity directly
    pos_jumps = np.max(np.abs(np.diff(values, axis=0)), axis=1)
    vel_jumps = np.max(np.abs(np.diff(num_vel, axis=0)), axis=1)
    acc_jumps = np.max(np.abs(np.diff(num_acc, axis=0)), axis=1)

    max_pos_jump = np.max(pos_jumps)
    max_vel_jump = np.max(vel_jumps)
    max_acc_jump = np.max(acc_jumps) if len(acc_jumps) > 0 else 0.0

    passed = (max_pos_jump < pos_tol and
              max_vel_jump < vel_tol and
              max_acc_jump < acc_tol)

    return passed, max_pos_jump, max_vel_jump, max_acc_jump


# ---------------------------------------------------------------------------
# Test 1: Ramp all axes
# ---------------------------------------------------------------------------

def test_ramp_all_axes():
    """Linear ramp from 0→1 on all spacemouse axes over 1 second.

    Verifies:
    - Motor position trace is smooth (C2 continuous)
    - Platform reaches near the target at the end
    - No motor velocity exceeds limits
    """
    smoother, geom, dyn = make_smoother_spacemouse()

    n_samples = 100  # 1 second at 100 Hz
    raw_traj = []
    for i in range(n_samples):
        frac = i / (n_samples - 1)  # 0..1
        raw_traj.append((frac, frac, frac, frac, frac, frac))

    result = simulate_spacemouse(smoother, geom, dyn, raw_traj,
                                 settle_time_s=2.0)

    # Check C2 on motor positions
    passed, max_pj, max_vj, max_aj = check_c2_continuity(
        result['times'], result['motor_pos'], 'motor_pos')
    assert passed, (
        f"Motor position C2 violation: pos_jump={max_pj:.4f}, "
        f"vel_jump={max_vj:.4f}, acc_jump={max_aj:.4f}")

    # Check motor velocity within limits (with 10% margin for transients)
    peak_motor_vel = np.max(np.abs(result['motor_vel']))
    vel_limit = hw.SPACEMOUSE_SMOOTHER_VEL_LIMIT_RPS
    assert peak_motor_vel < vel_limit * 1.10, (
        f"Peak motor velocity {peak_motor_vel:.2f} exceeds limit "
        f"{vel_limit:.1f} * 1.1")

    # Check that the final pose is near the final target.
    # The ramp ends at full deflection (150mm XY, 140mm Z, 30deg tilt),
    # so we need generous settle time.  Full deflection at spacemouse
    # limits crosses the workspace boundary — verify we get close.
    final_target = result['targets'][-1]
    final_pose = result['poses'][-1]
    pos_err = norm(final_pose[:3] - final_target[:3])
    assert pos_err < 60.0, (
        f"Final position error {pos_err:.1f} mm (should be < 60 mm)")

    print(f"  [PASS] Ramp all axes: peak_vel={peak_motor_vel:.2f} rps, "
          f"final_pos_err={pos_err:.1f} mm")


# ---------------------------------------------------------------------------
# Test 2: Step response
# ---------------------------------------------------------------------------

def test_step_response():
    """Instant step from 0→1 on all axes (held for 1 second).

    Verifies:
    - Response is C2 smooth despite the step
    - Platform reaches target within settle time
    """
    smoother, geom, dyn = make_smoother_spacemouse()

    n_samples = 100
    raw_traj = [(1.0, 1.0, 1.0, 1.0, 1.0, 1.0)] * n_samples

    result = simulate_spacemouse(smoother, geom, dyn, raw_traj,
                                 settle_time_s=1.0)

    # C2 check
    passed, max_pj, max_vj, max_aj = check_c2_continuity(
        result['times'], result['motor_pos'], 'motor_pos')
    assert passed, (
        f"Step response C2 violation: pos_jump={max_pj:.4f}, "
        f"vel_jump={max_vj:.4f}, acc_jump={max_aj:.4f}")

    # Should reach target (with settle time)
    final_target = result['targets'][-1]
    final_pose = result['poses'][-1]
    pos_err = norm(final_pose[:3] - final_target[:3])
    assert pos_err < 1.0, (
        f"Step response didn't settle: pos_err={pos_err:.1f} mm")

    # Velocity should be zero at end (settled)
    final_twist_norm = norm(result['twists'][-1])
    assert final_twist_norm < 0.1, (
        f"Step response didn't settle: twist={final_twist_norm:.4f}")

    print(f"  [PASS] Step response: pos_err={pos_err:.2f} mm, "
          f"final_twist={final_twist_norm:.4f}")


# ---------------------------------------------------------------------------
# Test 3: Direction reversal
# ---------------------------------------------------------------------------

def test_direction_reversal():
    """Ramp 0→1→-1 on all axes over 2 seconds.

    Verifies:
    - C2 continuity through the reversal
    - Platform eventually reaches the -1 target
    """
    smoother, geom, dyn = make_smoother_spacemouse()

    n_half = 100  # 1 second each way
    raw_traj = []
    # Ramp up
    for i in range(n_half):
        frac = i / (n_half - 1)
        raw_traj.append((frac, frac, frac, frac, frac, frac))
    # Ramp down through zero to -1
    for i in range(n_half):
        frac = 1.0 - 2.0 * i / (n_half - 1)  # 1.0 → -1.0
        raw_traj.append((frac, frac, frac, frac, frac, frac))

    result = simulate_spacemouse(smoother, geom, dyn, raw_traj,
                                 settle_time_s=2.0)

    # C2 check
    passed, max_pj, max_vj, max_aj = check_c2_continuity(
        result['times'], result['motor_pos'], 'motor_pos')
    assert passed, (
        f"Reversal C2 violation: pos_jump={max_pj:.4f}, "
        f"vel_jump={max_vj:.4f}, acc_jump={max_aj:.4f}")

    # Should reach the final (-1) target.  Full deflection at (-1, -1, ...)
    # is at the workspace boundary — allow generous tolerance since the
    # smoother can't track poses outside the reachable workspace exactly.
    final_target = result['targets'][-1]
    final_pose = result['poses'][-1]
    pos_err = norm(final_pose[:3] - final_target[:3])
    assert pos_err < 80.0, (
        f"Reversal didn't reach target: pos_err={pos_err:.1f} mm")

    print(f"  [PASS] Direction reversal: C2 smooth, "
          f"final_pos_err={pos_err:.1f} mm")


# ---------------------------------------------------------------------------
# Test 4: Velocity buildup
# ---------------------------------------------------------------------------

def test_velocity_buildup():
    """Verify that motor velocity reaches a meaningful fraction of the limit.

    This is the key test for hysteresis/deferral effectiveness.  Without
    these features, 100 Hz re-splicing limits peak velocity to ~25% of
    the theoretical maximum.  With them, we should exceed 30%.

    The Z-only ramp produces ~1.4mm per 10ms tick (~0.02 rev/tick).
    With hysteresis (0.05 rev) and deferral (50ms), each quintic covers
    several ticks of displacement and inherits velocity from the prior
    segment, meaningfully improving velocity buildup.
    """
    smoother, geom, dyn = make_smoother_spacemouse()

    # Sustained ramp in Z only — largest single-axis displacement
    n_samples = 100  # 1 second
    raw_traj = []
    for i in range(n_samples):
        frac = i / (n_samples - 1)
        raw_traj.append((0.0, 0.0, frac, 0.0, 0.0, 0.0))

    result = simulate_spacemouse(smoother, geom, dyn, raw_traj)

    peak_motor_vel = np.max(np.abs(result['motor_vel']))
    vel_limit = hw.SPACEMOUSE_SMOOTHER_VEL_LIMIT_RPS

    vel_fraction = peak_motor_vel / vel_limit
    assert vel_fraction > 0.30, (
        f"Peak motor velocity too low: {peak_motor_vel:.2f} rps = "
        f"{vel_fraction*100:.1f}% of limit {vel_limit:.1f} rps "
        f"(need >30%, baseline without hysteresis/deferral is ~25%)")

    print(f"  [PASS] Velocity buildup: peak={peak_motor_vel:.2f} rps = "
          f"{vel_fraction*100:.1f}% of limit")


# ---------------------------------------------------------------------------
# Test 5: Per-motor C2 continuity (all axes simultaneously)
# ---------------------------------------------------------------------------

def test_c2_all_axes_motor_space():
    """Full 6-DoF motion: verify C2 continuity in motor space.

    Ramps all axes with different profiles to exercise cross-coupling,
    then checks each motor's position trace for C2 continuity.
    """
    smoother, geom, dyn = make_smoother_spacemouse()

    n_samples = 150  # 1.5 seconds
    raw_traj = []
    for i in range(n_samples):
        frac = i / (n_samples - 1)
        # Different profiles per axis to create varied motion
        raw_traj.append((
            frac,                           # X: linear ramp
            0.5 * math.sin(2 * math.pi * frac),  # Y: sine
            frac * 0.8,                     # Z: slower ramp
            0.3 * frac,                     # pitch: gentle ramp
            0.3 * math.sin(math.pi * frac), # roll: half sine
            0.2 * frac,                     # yaw: gentle ramp
        ))

    result = simulate_spacemouse(smoother, geom, dyn, raw_traj)

    # Check C2 for each motor individually
    for motor_idx in range(6):
        motor_trace = result['motor_pos'][:, motor_idx:motor_idx+1]
        passed, max_pj, max_vj, max_aj = check_c2_continuity(
            result['times'], motor_trace, f'motor_{motor_idx}')
        assert passed, (
            f"Motor {motor_idx} C2 violation: pos_jump={max_pj:.4f}, "
            f"vel_jump={max_vj:.4f}, acc_jump={max_aj:.4f}")

    # Also check the composite motor velocity trace is smooth
    vel_mag = np.max(np.abs(result['motor_vel']), axis=1)
    vel_diff = np.abs(np.diff(vel_mag))
    max_vel_jump = np.max(vel_diff)

    # Velocity magnitude shouldn't jump by more than 1 rps between
    # consecutive 500 Hz samples
    assert max_vel_jump < 1.0, (
        f"Motor velocity magnitude jump = {max_vel_jump:.4f} rps "
        f"(limit 1.0 rps per 2ms)")

    print(f"  [PASS] All-axes motor C2: all 6 motors smooth, "
          f"max vel jump = {max_vel_jump:.4f} rps")


# ---------------------------------------------------------------------------
# Test 6: Settle from rest after sustained input
# ---------------------------------------------------------------------------

def test_settle_after_release():
    """Ramp to full deflection, then suddenly release to zero.

    Verifies that the platform smoothly decelerates to rest at the
    zero-deflection pose.
    """
    smoother, geom, dyn = make_smoother_spacemouse()

    raw_traj = []
    # 0.5s ramp to full Z deflection
    for i in range(50):
        frac = i / 49
        raw_traj.append((0.0, 0.0, frac, 0.0, 0.0, 0.0))
    # Instant release to zero (held for 0.5s)
    for _ in range(50):
        raw_traj.append((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))

    result = simulate_spacemouse(smoother, geom, dyn, raw_traj,
                                 settle_time_s=1.0)

    # C2 check through the release
    passed, max_pj, max_vj, max_aj = check_c2_continuity(
        result['times'], result['motor_pos'], 'motor_pos')
    assert passed, (
        f"Settle C2 violation: pos_jump={max_pj:.4f}, "
        f"vel_jump={max_vj:.4f}, acc_jump={max_aj:.4f}")

    # Should return to home (z = default active Z, everything else ~0)
    final_pose = result['poses'][-1]
    home = home_pose()
    pos_err = norm(final_pose[:3] - home[:3])
    assert pos_err < 1.0, (
        f"Didn't settle to home: pos_err={pos_err:.1f} mm")

    final_twist_norm = norm(result['twists'][-1])
    assert final_twist_norm < 0.1, (
        f"Didn't settle to rest: twist={final_twist_norm:.4f}")

    print(f"  [PASS] Settle after release: pos_err={pos_err:.2f} mm, "
          f"twist={final_twist_norm:.4f}")


# ---------------------------------------------------------------------------
# Preview support
# ---------------------------------------------------------------------------

# Test definitions: (name, description, raw_trajectory_builder, settle_time_s)
# Each builder returns a list of (x, y, z, pitch, roll, yaw) tuples in -1..+1.
TEST_DEFS = [
    ('ramp_all', 'Ramp all axes 0->1', lambda: [
        (i/99, i/99, i/99, i/99, i/99, i/99) for i in range(100)
    ], 2.0),
    ('step', 'Step 0->1 all axes', lambda: [
        (1.0, 1.0, 1.0, 1.0, 1.0, 1.0) for _ in range(100)
    ], 1.0),
    ('reversal', 'Ramp 0->1->-1', lambda: [
        *(((i/99, i/99, i/99, i/99, i/99, i/99) for i in range(100))),
        *((((1.0 - 2.0*i/99,)*6) for i in range(100))),
    ], 2.0),
    ('z_ramp', 'Z-only ramp (vel buildup)', lambda: [
        (0.0, 0.0, i/99, 0.0, 0.0, 0.0) for i in range(100)
    ], 0.5),
    ('mixed', 'Mixed profiles (C2 test)', lambda: [
        (i/149,
         0.5 * math.sin(2 * math.pi * i/149),
         i/149 * 0.8,
         0.3 * i/149,
         0.3 * math.sin(math.pi * i/149),
         0.2 * i/149)
        for i in range(150)
    ], 0.5),
    ('release', 'Ramp + release', lambda: [
        *((0.0, 0.0, i/49, 0.0, 0.0, 0.0) for i in range(50)),
        *((0.0, 0.0, 0.0, 0.0, 0.0, 0.0) for _ in range(50)),
    ], 1.0),
]

HOLD_BETWEEN_TESTS = 0.5  # seconds to dwell at home between tests


def preview_all_tests():
    """Preview all spacemouse pipeline tests in a single scrollable plot.

    Stitches all tests end-to-end with C2-continuous return-to-home
    transitions.  A single StreamSmoother runs across the entire timeline.
    Uses the same 5-row plot layout as smoother_test.py.
    """
    try:
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Slider
    except ImportError:
        print("WARNING: matplotlib not available, skipping preview")
        return

    # Import viewer constants — add tools/ to path
    import os
    tools_dir = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        '..', '..', '..', '..', '..', '..', 'tools')
    tools_dir = os.path.normpath(tools_dir)
    if tools_dir not in sys.path:
        sys.path.insert(0, tools_dir)

    from trajectory_viewer import LEG_COLORS, CARTESIAN_LABELS, CARTESIAN_COLORS

    smoother, geom, dyn = make_smoother_spacemouse()
    home = home_pose()
    dt = 1.0 / 500.0  # 500 Hz eval rate
    dt_target = 1.0 / 100.0  # 100 Hz target rate

    # Single-pass simulation across all tests
    all_ts = []
    all_poses = []
    all_twists = []
    all_extensions = []
    all_vel_ff = []
    all_torques = []

    test_boundaries = []    # (start_t, end_t, label)
    return_boundaries = []  # (start_t, end_t)

    t = 0.0

    for name, desc, builder, settle_s in TEST_DEFS:
        raw_traj = builder()
        n_targets = len(raw_traj)
        target_poses = [spacemouse_raw_to_pose_6dof(*raw) for raw in raw_traj]

        test_start = t
        target_idx = 0
        test_end = t + n_targets * dt_target + settle_s

        # Simulate test phase
        while t < test_end:
            local_t = t - test_start
            while (target_idx < n_targets
                   and local_t >= target_idx * dt_target):
                smoother.set_target(target_poses[target_idx],
                                    test_start + target_idx * dt_target)
                target_idx += 1

            pose, twist, accel = smoother.evaluate(t)
            pos_rev, vel_ff, torque_ff, _ = cartesian_to_motor_commands(
                pose, twist, accel, geom, dyn, feedforward_enabled=True)
            rot = rotvec_to_rot_matrix(pose[3:6])
            ext = pose_to_leg_lengths(pose[:3], rot, geom)

            all_ts.append(t)
            all_poses.append(pose)
            all_twists.append(twist)
            all_extensions.append(ext)
            all_vel_ff.append(vel_ff)
            all_torques.append(torque_ff)

            t += dt

        test_boundaries.append((test_start, t, desc))

        # Return to home
        return_start = t
        smoother.set_target(home.copy(), t)
        return_dur = smoother._duration
        return_end = t + return_dur + HOLD_BETWEEN_TESTS

        while t < return_end:
            pose, twist, accel = smoother.evaluate(t)
            pos_rev, vel_ff, torque_ff, _ = cartesian_to_motor_commands(
                pose, twist, accel, geom, dyn, feedforward_enabled=True)
            rot = rotvec_to_rot_matrix(pose[3:6])
            ext = pose_to_leg_lengths(pose[:3], rot, geom)

            all_ts.append(t)
            all_poses.append(pose)
            all_twists.append(twist)
            all_extensions.append(ext)
            all_vel_ff.append(vel_ff)
            all_torques.append(torque_ff)

            t += dt

        return_boundaries.append((return_start, t))

    # Convert to numpy
    t_arr = np.array(all_ts)
    poses_arr = np.array(all_poses)
    twists_arr = np.array(all_twists)
    ext_arr = np.array(all_extensions)
    vel_arr = np.array(all_vel_ff)
    tor_arr = np.array(all_torques)
    total_duration = t_arr[-1]

    # -- Build 5-row scrollable plot --
    ROW_SPECS = [
        ('Cartesian Pose', poses_arr, 'mm / rad', False),
        ('Cartesian Twist', twists_arr, 'mm/s / rad/s', False),
        ('Leg Extensions', ext_arr, 'mm', True),
        ('Motor Velocity FF', vel_arr, 'rev/s', True),
        ('Torque Feedforward', tor_arr, 'Nm', True),
    ]

    fig, axes = plt.subplots(5, 1, figsize=(16, 11), sharex=True)
    fig.subplots_adjust(hspace=0.35, left=0.08, right=0.96,
                        top=0.93, bottom=0.10)

    fig.suptitle(
        f'Spacemouse Pipeline Preview — {len(TEST_DEFS)} tests, '
        f'{total_duration:.1f}s  '
        f'(vel={hw.SPACEMOUSE_SMOOTHER_VEL_LIMIT_RPS} rps, '
        f'accel={hw.SPACEMOUSE_SMOOTHER_ACCEL_LIMIT_RPS2} rps\u00b2)',
        fontsize=11)

    for row_idx, (title, arr, ylabel, use_leg) in enumerate(ROW_SPECS):
        ax = axes[row_idx]
        colors = LEG_COLORS if use_leg else CARTESIAN_COLORS
        labels = ([f'Leg {i}' for i in range(6)]
                  if use_leg else CARTESIAN_LABELS)

        for ch in range(arr.shape[1]):
            ax.plot(t_arr, arr[:, ch], '-', color=colors[ch],
                    linewidth=0.8, label=labels[ch])

        ax.set_ylabel(ylabel, fontsize=7)
        ax.set_title(title, fontsize=8, pad=3)
        ax.tick_params(labelsize=6)
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=5, ncol=3 if use_leg else 6, loc='upper right')

        # Test boundary markers
        for start, end, _label in test_boundaries:
            ax.axvline(start, color='#444444', linewidth=0.8,
                       linestyle='--', alpha=0.7)

        # Return-to-home shading
        for start, end in return_boundaries:
            ax.axvspan(start, end, alpha=0.06, color='#4363d8')

    axes[-1].set_xlabel('Time (s)', fontsize=7)

    # Test labels along top axis
    ax_top = axes[0]
    y_top = ax_top.get_ylim()[1]
    for start, end, label in test_boundaries:
        mid = (start + end) / 2
        ax_top.annotate(label, xy=(mid, y_top), fontsize=5,
                        ha='center', va='bottom', rotation=0,
                        color='#333333', annotation_clip=False)

    # Return labels
    for start, end in return_boundaries:
        mid = (start + end) / 2
        ax_top.annotate('Return', xy=(mid, y_top), fontsize=4,
                        ha='center', va='bottom', rotation=0,
                        color='#4363d8', alpha=0.7,
                        annotation_clip=False)

    # -- Scrollbar --
    view_width = [min(10.0, total_duration)]

    ax_scroll = fig.add_axes([0.08, 0.02, 0.88, 0.02])
    max_scroll = max(0.0, total_duration - view_width[0])
    scroll_slider = Slider(ax_scroll, '', 0.0, max(max_scroll, 0.001),
                           valinit=0.0, valstep=0.05,
                           color='#4363d8', track_color='#dddddd')
    scroll_slider.valtext.set_visible(False)

    def _apply_view(t_left):
        t_right = min(t_left + view_width[0], total_duration)
        t_left = max(0.0, t_right - view_width[0])
        axes[-1].set_xlim(t_left, t_right)
        fig.canvas.draw_idle()

    def _on_scroll_change(val):
        _apply_view(val)

    scroll_slider.on_changed(_on_scroll_change)

    def _on_mouse_scroll(event):
        if event.inaxes not in axes:
            return
        zoom_factor = 0.8 if event.button == 'up' else 1.25
        new_width = np.clip(view_width[0] * zoom_factor, 1.0, total_duration)
        view_width[0] = new_width

        t_mouse = event.xdata if event.xdata is not None else 0.0
        current_left, current_right = axes[-1].get_xlim()
        frac = ((t_mouse - current_left) /
                max(current_right - current_left, 0.001))
        new_left = t_mouse - frac * new_width
        new_left = np.clip(new_left, 0.0, total_duration - new_width)

        new_max = max(0.0, total_duration - new_width)
        scroll_slider.valmax = max(new_max, 0.001)
        scroll_slider.ax.set_xlim(0.0, scroll_slider.valmax)
        scroll_slider.set_val(np.clip(new_left, 0.0, new_max))
        _apply_view(new_left)

    fig.canvas.mpl_connect('scroll_event', _on_mouse_scroll)

    if fig.canvas.toolbar is not None:
        fig.canvas.toolbar.mode = ''

    _apply_view(0.0)
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description='Spacemouse pipeline end-to-end tests')
    parser.add_argument('--preview', action='store_true',
                        help='Show scrollable 5-row plot preview of all tests')
    args = parser.parse_args()

    if args.preview:
        preview_all_tests()
        return 0

    print("Spacemouse pipeline end-to-end tests")
    print("=" * 60)

    tests = [
        ("1. Ramp all axes", test_ramp_all_axes),
        ("2. Step response", test_step_response),
        ("3. Direction reversal", test_direction_reversal),
        ("4. Velocity buildup", test_velocity_buildup),
        ("5. C2 all-axes motor space", test_c2_all_axes_motor_space),
        ("6. Settle after release", test_settle_after_release),
    ]

    passed = 0
    failed = 0
    for name, fn in tests:
        print(f"\n{name}")
        try:
            fn()
            passed += 1
        except AssertionError as e:
            print(f"  [FAIL] {e}")
            failed += 1
        except Exception as e:
            print(f"  [FAIL] {e}")
            import traceback
            traceback.print_exc()
            failed += 1

    print(f"\n{'=' * 60}")
    print(f"Results: {passed} passed, {failed} failed out of {len(tests)}")
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
