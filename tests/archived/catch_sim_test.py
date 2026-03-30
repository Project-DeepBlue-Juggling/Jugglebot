#!/usr/bin/env python3
"""Simulated catch sequence — exercises the full pure-Python pipeline.

Simulates a Ball Butler throw with configurable mocap noise, runs it through
the BallTracker (Kalman filter + marker matching), CatchCoordinator
(catch pose + hand command generation), and TrajectoryManager (feasibility-
checked quintic trajectories) — the same pipeline the real robot uses.

No ROS2 or hardware required. Uses only the pure-Python tracking,
coordinator, and motion modules.

Usage:
    python tools/catch_sim_test.py
    python tools/catch_sim_test.py --viz
    python tools/catch_sim_test.py --viz --noise 5.0 --delay 0.08
    python tools/catch_sim_test.py --human-throw
    python tools/catch_sim_test.py --sweep
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np

# ── Path setup (same pattern as other tools/) ────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_SCRIPT_DIR)
_CONFIG_DIR = os.path.join(_PROJECT_ROOT, 'config', 'generated')
_ROS_PKG_DIR = os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'jugglebot')

sys.path.insert(0, _CONFIG_DIR)
sys.path.insert(0, _ROS_PKG_DIR)

import hardware_config as hw
from jugglebot.tracking.matcher import BallTracker
from jugglebot.tracking.ball import Ball, BallStatus, TrackingConfidence
from jugglebot.tracking.ballistics import GRAVITY_MMPS2, predict_landing_state
from jugglebot.catch_coordinator import CatchCoordinator, CatchCommand


# ── Constants ────────────────────────────────────────────────────────
MOCAP_DT = 0.005          # 200 Hz mocap
LANDING_Z = (hw.GEOM_INITIAL_HEIGHT_MM
             + hw.JB_OP_DEFAULT_ACTIVE_Z_MM
             + hw.HAND_CATCH_OFFSET_MM)
INITIAL_HEIGHT = hw.GEOM_INITIAL_HEIGHT_MM      # 574.3 mm

# Hand motor constants
HAND_PRIME_REV = hw.JB_OP_HAND_CATCH_PRIME_REV       # 9.858 rev
HAND_MAX_REV = hw.GEOM_HAND_MOTOR_MAX_POSITION_REVS  # 11.1 rev
HAND_STROKE_MM = hw.GEOM_HAND_STROKE_MM              # 355 mm
HAND_MM_PER_REV = HAND_STROKE_MM / HAND_MAX_REV      # ~31.98 mm/rev
HAND_SMOOTH_ACCEL_RPS2 = hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2  # 100 rps²
HAND_SPOOL_CIRC_M = 2.0 * math.pi * hw.TEENSY_TRAJ_HAND_SPOOL_RADIUS_M  # m/rev
HAND_BOTTOM_OFFSET_MM = hw.GEOM_HAND_AXIS_BOTTOM_OFFSET_MM  # -129 mm from centroid

# Ball Butler position (typical calibrated position).
# BB sits beside the platform, elevated above the catch plane; close
# enough that a moderate throw lands within the Stewart platform's XY
# workspace (~±80 mm).
BB_POSITION = np.array([40.0, -80.0, 1500.0])  # mm, base frame


# ── Hand trajectory math (mirrors Trajectory.h) ──────────────────

# Teensy trajectory constants (from hardware_config)
_TOTAL_STROKE = hw.TEENSY_TRAJ_HAND_STROKE_M - 2.0 * hw.TEENSY_TRAJ_STROKE_MARGIN_M
_INERTIA_RATIO = hw.TEENSY_TRAJ_INERTIA_RATIO
_CATCH_VEL_RATIO = hw.TEENSY_TRAJ_CATCH_VEL_RATIO
_CATCH_VEL_HOLD_PCT = hw.TEENSY_TRAJ_CATCH_VEL_HOLD_PCT
_LINEAR_GAIN = hw.TEENSY_LINEAR_GAIN  # rev/m


def _calc_catch_params(event_vel_mps: float) -> dict:
    """Compute catch trajectory parameters from ball speed.

    Mirrors Trajectory.h calcThrow() + calcCatch(). The Teensy uses
    event_vel as v_throw to generate a 3-segment catch profile:
      accel (t_acc) → velocity hold (t_vel) → decel (t_dec)

    The hand starts at x3 (top of stroke, at rest), accelerates downward
    for t_acc seconds, then holds constant velocity vC for t_vel seconds
    (ball is caught at the midpoint of this hold), then decelerates to rest.

    Returns dict with segment durations, accelerations, and positions in rev.
    """
    v_throw = event_vel_mps
    x3_m = _TOTAL_STROKE  # top of throw stroke

    # calcCatch — same algebra as Trajectory.h
    vC = -_CATCH_VEL_RATIO * v_throw
    irC = 1.0 / _INERTIA_RATIO
    vel_hold_m = _CATCH_VEL_HOLD_PCT * _TOTAL_STROKE
    accel_stroke_m = _TOTAL_STROKE - vel_hold_m

    t_acc = -(2.0 / (irC + 1.0)) * accel_stroke_m / vC
    t_vel = -vel_hold_m / vC
    t_dec = t_acc * irC

    catchA = vC / t_acc       # m/s² (negative = downward)
    catchD = -catchA / irC    # m/s² (positive = braking)

    x5_m = x3_m + 0.5 * catchA * t_acc * t_acc
    x6_m = x5_m + vC * t_vel

    return {
        't_acc': t_acc,
        't_vel': t_vel,
        't_dec': t_dec,
        'catchA': catchA,
        'catchD': catchD,
        'vC': vC,
        'x3_rev': x3_m * _LINEAR_GAIN,
        'x5_rev': x5_m * _LINEAR_GAIN,
        'x6_rev': x6_m * _LINEAR_GAIN,
    }


# ── Hand motor simulator ───────────────────────────────────────────

class HandSim:
    """Hand motor position simulator for offline catch sim.

    Replicates the Teensy's Trajectory.h catch timing:
      1. Smooth-move to prime position (triangular velocity profile).
      2. Catch trajectory: 3-segment (accel → vel-hold → decel) timed so
         the hand is at x5 (catch point) moving at vC when the ball arrives.
         The hand starts moving t_acc seconds BEFORE the event time.
    """

    def __init__(self):
        self.position_rev: float = 0.0
        self._vel_rps: float = 0.0

        # Smooth-move state
        self._move_start_time: float = -1.0
        self._move_start_rev: float = 0.0
        self._move_target_rev: float = 0.0
        self._move_duration: float = 0.0
        self._move_peak_vel: float = 0.0

        # Catch trajectory state (mirrors Trajectory.h 3-segment catch)
        self._catch_armed: bool = False
        self._catch_start_time: float = -1.0  # when hand starts moving (t4)
        self._catch_params: dict = {}

    def command_prime(self, target_rev: float, start_time: float):
        """Start a smooth-move to the prime position."""
        distance = abs(target_rev - self.position_rev)
        if distance < 0.001:
            return
        self._move_start_time = start_time
        self._move_start_rev = self.position_rev
        self._move_target_rev = target_rev
        # Triangular profile: total_time = 2 * sqrt(distance / accel)
        self._move_duration = 2.0 * math.sqrt(distance / HAND_SMOOTH_ACCEL_RPS2)
        self._move_peak_vel = HAND_SMOOTH_ACCEL_RPS2 * (self._move_duration / 2.0)

    def command_catch(self, event_time: float, event_vel_mps: float):
        """Arm the catch trajectory to fire at event_time.

        The hand will start moving t_acc seconds BEFORE event_time,
        matching the Teensy's Trajectory.h timing. At event_time the hand
        is at x5 (catch position) moving at constant velocity vC.
        """
        self._catch_params = _calc_catch_params(event_vel_mps)
        self._catch_armed = True
        # Hand starts moving t_acc before the ball arrives (matches Teensy
        # makeCatch() time shift: first sample at -(t5-t4) = -t_acc)
        self._catch_start_time = event_time - self._catch_params['t_acc']

    def step(self, t: float):
        """Update hand position to time t."""
        # Catch trajectory: 3-segment (accel → vel-hold → decel)
        if self._catch_armed and t >= self._catch_start_time:
            p = self._catch_params
            tau = t - self._catch_start_time  # time since hand started moving

            t_acc = p['t_acc']
            t_vel = p['t_vel']
            t_dec = p['t_dec']

            if tau < t_acc:
                # Segment 1: accelerate from rest at x3
                pos_rev = p['x3_rev'] + 0.5 * p['catchA'] * _LINEAR_GAIN * tau * tau
            elif tau < t_acc + t_vel:
                # Segment 2: constant velocity hold (ball caught at midpoint)
                dt = tau - t_acc
                pos_rev = p['x5_rev'] + p['vC'] * _LINEAR_GAIN * dt
            elif tau < t_acc + t_vel + t_dec:
                # Segment 3: decelerate to rest
                dt = tau - t_acc - t_vel
                pos_rev = (p['x6_rev']
                           + p['vC'] * _LINEAR_GAIN * dt
                           + 0.5 * p['catchD'] * _LINEAR_GAIN * dt * dt)
            else:
                # Hold at end position
                dt_end = t_dec
                pos_rev = (p['x6_rev']
                           + p['vC'] * _LINEAR_GAIN * dt_end
                           + 0.5 * p['catchD'] * _LINEAR_GAIN * dt_end * dt_end)

            self.position_rev = max(0.0, pos_rev)
            self._move_start_time = -1.0  # cancel smooth-move
            return

        # Smooth-move to prime
        if self._move_start_time >= 0 and t >= self._move_start_time:
            elapsed = t - self._move_start_time
            if elapsed >= self._move_duration:
                self.position_rev = self._move_target_rev
                self._vel_rps = 0.0
                return
            half = self._move_duration / 2.0
            dist = self._move_target_rev - self._move_start_rev
            sign = 1.0 if dist > 0 else -1.0
            if elapsed <= half:
                # Accelerating
                self.position_rev = (self._move_start_rev
                                     + sign * 0.5 * HAND_SMOOTH_ACCEL_RPS2
                                     * elapsed * elapsed)
            else:
                # Decelerating
                t_decel = elapsed - half
                mid_pos = (self._move_start_rev
                           + sign * 0.5 * HAND_SMOOTH_ACCEL_RPS2 * half * half)
                self.position_rev = (mid_pos
                                     + sign * self._move_peak_vel * t_decel
                                     - sign * 0.5 * HAND_SMOOTH_ACCEL_RPS2
                                     * t_decel * t_decel)


# ── Throw simulation ────────────────────────────────────────────────

@dataclass
class ThrowParams:
    """Parameters for a simulated throw."""
    initial_position: np.ndarray
    initial_velocity: np.ndarray  # mm/s
    throw_time: float             # Absolute time (s)
    source: str = "ball_butler"
    destination: str = "jugglebot"


def make_bb_throw(
    throw_time: float = 0.5,
    speed_mps: float = 1.0,
    yaw_deg: float = 90.0,
    pitch_deg: float = 80.0,
) -> ThrowParams:
    """Create a Ball Butler throw aimed at the platform.

    Default: BB at z=1500mm (well above catch plane at 734mm), 80mm to
    the side.  Thrown at 1.0 m/s, pitch=80 (nearly vertical), yaw=90
    toward platform.  Ball arcs up then descends through the catch
    plane with ~0.6s flight time, landing within the Stewart platform's
    XY workspace (~±60 mm from center).
    """
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    speed = speed_mps * 1000.0  # mm/s

    vx = speed * math.cos(pitch) * math.cos(yaw)
    vy = speed * math.cos(pitch) * math.sin(yaw)
    vz = speed * math.sin(pitch)

    return ThrowParams(
        initial_position=BB_POSITION.copy(),
        initial_velocity=np.array([vx, vy, vz]),
        throw_time=throw_time,
    )


def make_throw_to_landing(
    landing_xy: tuple[float, float] = (0.0, 0.0),
    throw_time: float = 0.5,
    flight_time: float = 0.6,
    launch_pos: Optional[np.ndarray] = None,
) -> ThrowParams:
    """Create a throw that lands at a specific XY on the catch plane.

    Back-computes the initial velocity so the ball follows a ballistic
    arc and crosses the catch plane (LANDING_Z) at the desired XY
    after ``flight_time`` seconds.

    Args:
        landing_xy: (x, y) in mm on the catch plane (base frame).
        throw_time: Absolute time of throw (s).
        flight_time: Desired time of flight (s). Longer = gentler arc.
        launch_pos: [x, y, z] launch position in mm. Defaults to BB_POSITION.
                    Lower Z or larger XY offset → steeper approach angles.
    """
    p0 = launch_pos if launch_pos is not None else BB_POSITION
    lx, ly = landing_xy
    t = flight_time

    # Solve for v0 from: landing = p0 + v0*t + 0.5*g_vec*t^2
    #   v0 = (landing - p0 - 0.5*g_vec*t^2) / t
    vx = (lx - p0[0]) / t
    vy = (ly - p0[1]) / t
    vz = (LANDING_Z - p0[2] + 0.5 * GRAVITY_MMPS2 * t * t) / t

    return ThrowParams(
        initial_position=p0.copy(),
        initial_velocity=np.array([vx, vy, vz]),
        throw_time=throw_time,
    )


def make_human_throw(throw_time: float = 0.2) -> ThrowParams:
    """Create a human throw — no announcement, detected by parabolic motion."""
    return ThrowParams(
        initial_position=np.array([200.0, 300.0, 1200.0]),
        initial_velocity=np.array([-150.0, -250.0, 1500.0]),
        throw_time=throw_time,
        source="human_throw",
        destination="",
    )


def ballistic_position(pos0: np.ndarray, vel0: np.ndarray, dt: float) -> np.ndarray:
    """Compute ballistic position after dt seconds."""
    return np.array([
        pos0[0] + vel0[0] * dt,
        pos0[1] + vel0[1] * dt,
        pos0[2] + vel0[2] * dt - 0.5 * GRAVITY_MMPS2 * dt * dt,
    ])


# ── Per-frame recording for visualization ────────────────────────────

@dataclass
class SimFrame:
    """One frame of simulation state for visualization."""
    time: float
    # Ball
    ball_true_pos: Optional[np.ndarray] = None    # True ballistic position
    ball_kf_pos: Optional[np.ndarray] = None      # Kalman filter estimate
    ball_status: Optional[BallStatus] = None
    ball_tracking: Optional[TrackingConfidence] = None
    # Landing prediction
    landing_pred_pos: Optional[np.ndarray] = None  # Predicted landing XYZ
    # Coordinator raw target (where the coordinator wants the platform)
    platform_target_pos: Optional[np.ndarray] = None  # [x,y,z] platform frame
    platform_target_quat: Optional[np.ndarray] = None  # [w,x,y,z]
    # Trajectory-planned actual pose (what the real robot would execute)
    platform_actual_pos: Optional[np.ndarray] = None   # [x,y,z] platform frame
    platform_actual_quat: Optional[np.ndarray] = None  # [w,x,y,z]
    traj_state: Optional[str] = None                    # TrajectoryState name
    # Leg extensions (from IK)
    leg_extensions_mm: Optional[np.ndarray] = None  # (6,) extensions, 0=retracted
    # Hand
    hand_armed: bool = False
    hand_primed: bool = False
    hand_position_rev: float = 0.0


# ── Simulation results ──────────────────────────────────────────────

@dataclass
class CatchSimResult:
    """Results from a simulated catch sequence."""
    throw_params: ThrowParams = None
    true_landing_pos: np.ndarray = None
    true_landing_vel: np.ndarray = None
    true_landing_time: float = 0.0

    first_detection_time: float = 0.0
    first_detection_delay_ms: float = 0.0
    tracking_confirmed: bool = False
    landing_pos_errors_mm: List[float] = field(default_factory=list)
    landing_time_errors_ms: List[float] = field(default_factory=list)

    first_catch_command_time: float = 0.0
    catch_commands_issued: int = 0
    final_target_pos: Optional[np.ndarray] = None
    final_target_pos_error_mm: float = 0.0
    hand_armed: bool = False
    hand_event_vel_mps: float = 0.0
    hand_prime_rev: float = 0.0
    hand_event_delay_s: float = 0.0

    ball_status: BallStatus = BallStatus.UNKNOWN

    # Timeline for visualization
    frames: List[SimFrame] = field(default_factory=list)



def _block_wait_for_async_commit(traj_mgr, timeout_s: float = 2.0) -> bool:
    """Block-wait for the feasibility worker and submit directly.

    In the real 500 Hz control loop the worker returns results between
    cycles and commits via a splice re-check.  In this offline sim we
    have exact sim-time evaluation, so we submit the feasibility-checked
    trajectory directly — identical polynomial, no splice delay.
    """
    import time as _time

    deadline = _time.perf_counter() + timeout_s
    while _time.perf_counter() < deadline:
        try:
            result = traj_mgr.poll_pending_result()
        except Exception:
            return False
        if result is not None:
            if not result.get('accepted'):
                return False
            traj_mgr.submit(result['traj'])
            if result.get('needs_decel'):
                traj_mgr._pending_decel = True
                traj_mgr._start_decel_precompute(result['traj'])
            return True
        _time.sleep(0.002)
    return False


# ── Main simulation ─────────────────────────────────────────────────

def run_catch_simulation(
    throw: ThrowParams,
    noise_std_mm: float = 2.0,
    mocap_delay_s: float = 0.1,
    announced: bool = True,
    seed: int = 42,
) -> CatchSimResult:
    """Run a full simulated catch sequence.

    Integrates the real trajectory planning pipeline: CatchCoordinator
    outputs are fed through TrajectoryManager (with async feasibility
    checking) so the simulated platform motion matches what the real
    robot would execute.
    """
    from scipy.spatial.transform import Rotation
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.dynamics import DynamicsParams
    from jugglebot.motion.trajectory_manager import TrajectoryManager
    from jugglebot.motion.ik_solver import pose_to_leg_lengths, rotvec_to_rot_matrix

    rng = np.random.RandomState(seed)
    result = CatchSimResult(throw_params=throw)

    # Compute true landing
    landing = predict_landing_state(
        throw.initial_position, throw.initial_velocity, LANDING_Z)
    if landing is None:
        print("ERROR: Ball never reaches landing plane")
        return result
    true_lpos, true_lvel, ttl = landing
    true_landing_time = throw.throw_time + ttl
    result.true_landing_pos = true_lpos
    result.true_landing_vel = true_lvel
    result.true_landing_time = true_landing_time

    tracker = BallTracker(dt=MOCAP_DT, landing_z=LANDING_Z)
    coordinator = CatchCoordinator(
        robot_name="jugglebot",
        initial_height_mm=INITIAL_HEIGHT,
        landing_z_offset_mm=hw.JB_OP_DEFAULT_ACTIVE_Z_MM + hw.HAND_CATCH_OFFSET_MM,
        hand_catch_offset_mm=hw.HAND_CATCH_OFFSET_MM,
        catch_angle_limit_deg=30.0,
    )

    # ── Trajectory planner (same as real control loop) ────────────
    geom = StewartGeometry()
    dynamics_params = DynamicsParams.from_config()

    # Synthetic clock so TrajectoryManager uses sim time, not wall clock.
    # The mutable list allows the closure to track the current sim time.
    sim_clock = [0.0]
    traj_mgr = TrajectoryManager(
        geom, dynamics_params, clock=lambda: sim_clock[0])

    # Set hold pose to active pose (platform at default active Z, level)
    active_pose = np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM),
                            0.0, 0.0, 0.0])
    traj_mgr.set_hold_pose(active_pose)

    ball_id = None
    if announced:
        ball_id = tracker.handle_announcement(
            initial_position=throw.initial_position,
            initial_velocity=throw.initial_velocity,
            throw_time=throw.throw_time,
            source=throw.source,
            destination=throw.destination,
        )

    hand_sim = HandSim()

    sim_start = 0.0
    sim_end = true_landing_time + 0.5
    t = sim_start
    first_detection = False
    hand_armed_for = None
    hand_primed = False
    hand_is_armed = False
    current_target_pos = None
    current_target_quat = None
    last_traj_submit_time = -1.0  # Throttle trajectory submissions

    try:
        while t < sim_end:
            sim_clock[0] = t
            frame = SimFrame(time=t)

            # Generate marker positions
            markers = []
            time_since_throw = t - throw.throw_time

            if time_since_throw > 0:
                true_pos = ballistic_position(
                    throw.initial_position, throw.initial_velocity,
                    time_since_throw)
                frame.ball_true_pos = true_pos.copy()

                if (time_since_throw >= mocap_delay_s
                        and true_pos[2] > LANDING_Z - 50.0):
                    noisy_pos = true_pos + rng.randn(3) * noise_std_mm
                    markers.append(noisy_pos)

            # Process frame
            balls = tracker.process_frame(markers, t)

            # Find our ball and record its state
            tracked_ball = None
            for b in balls:
                if ball_id is not None and b.id == ball_id:
                    tracked_ball = b
                elif ball_id is None and b.status == BallStatus.IN_FLIGHT:
                    tracked_ball = b

            if tracked_ball is not None:
                frame.ball_kf_pos = tracked_ball.position.copy()
                frame.ball_status = tracked_ball.status
                frame.ball_tracking = tracked_ball.tracking
                if tracked_ball.landing_time > 0:
                    frame.landing_pred_pos = tracked_ball.landing_position.copy()

            # Track first detection
            if not first_detection:
                for b in balls:
                    if b.tracking == TrackingConfidence.CONFIRMED:
                        first_detection = True
                        result.first_detection_time = t
                        result.first_detection_delay_ms = (
                            (t - throw.throw_time) * 1000.0)
                        result.tracking_confirmed = True
                        break

            # Collect landing prediction errors
            for b in balls:
                if b.status == BallStatus.IN_FLIGHT and b.landing_time > 0:
                    pos_err = np.linalg.norm(b.landing_position - true_lpos)
                    time_err = (b.landing_time - true_landing_time) * 1000.0
                    result.landing_pos_errors_mm.append(pos_err)
                    result.landing_time_errors_ms.append(time_err)

            # ── Run coordinator ───────────────────────────────────
            cmd = coordinator.update(balls, t)
            if cmd is not None:
                result.catch_commands_issued += 1
                if result.catch_commands_issued == 1:
                    result.first_catch_command_time = t
                    hand_primed = True
                    # Prime the hand motor
                    hand_sim.command_prime(HAND_PRIME_REV, t)

                current_target_pos = cmd.target_pos.copy()
                current_target_quat = cmd.target_quat.copy()
                result.final_target_pos = current_target_pos.copy()

                # Feed command into trajectory planner.
                #
                # Throttle: only submit every 50ms sim-time to avoid
                # superseding in-flight requests before the background
                # worker finishes.  The real 500 Hz loop submits every
                # frame but the worker takes ~50ms wall-time per check;
                # here sim-time outruns wall-time so we must pace.
                if (t - last_traj_submit_time) >= 0.050:
                    queued = traj_mgr.request_dynamic_target(
                        target_pos=cmd.target_pos,
                        target_quat=cmd.target_quat,
                        target_vel=cmd.target_vel,
                        arrival_time=cmd.landing_time,
                        t_now=t,
                    )
                    if queued:
                        last_traj_submit_time = t
                        # Block-wait for the worker to finish this
                        # request and commit the trajectory directly
                        # (bypassing the splice path since we have
                        # exact sim-time evaluation).
                        _block_wait_for_async_commit(
                            traj_mgr, timeout_s=2.0)

                if cmd.arm_hand and hand_armed_for != cmd.ball_id:
                    event_delay = cmd.landing_time - t
                    if event_delay >= 0.3:
                        hand_is_armed = True
                        result.hand_armed = True
                        result.hand_event_vel_mps = cmd.event_vel_mps
                        result.hand_event_delay_s = event_delay
                        result.hand_prime_rev = hw.JB_OP_HAND_CATCH_PRIME_REV
                        hand_armed_for = cmd.ball_id
                        # Arm the catch trajectory on the hand motor sim
                        hand_sim.command_catch(cmd.landing_time,
                                               cmd.event_vel_mps)

            # ── Update hand motor ─────────────────────────────────
            hand_sim.step(t)

            # ── Evaluate trajectory (get actual platform pose) ────
            traj_mgr.evaluate(t)
            pose_6dof = traj_mgr.current_pose_6dof  # [x,y,z,rx,ry,rz]

            # Convert rotation vector → quaternion [w,x,y,z] for viz
            rotvec = pose_6dof[3:6]
            if np.linalg.norm(rotvec) > 1e-12:
                quat_xyzw = Rotation.from_rotvec(rotvec).as_quat()
                actual_quat = np.array([quat_xyzw[3], quat_xyzw[0],
                                        quat_xyzw[1], quat_xyzw[2]])
            else:
                actual_quat = np.array([1.0, 0.0, 0.0, 0.0])

            frame.platform_actual_pos = pose_6dof[:3].copy()
            frame.platform_actual_quat = actual_quat
            frame.traj_state = traj_mgr.state.name

            # Compute leg extensions via IK
            rot_matrix = rotvec_to_rot_matrix(rotvec)
            frame.leg_extensions_mm = pose_to_leg_lengths(
                pose_6dof[:3], rot_matrix, geom)

            frame.platform_target_pos = current_target_pos
            frame.platform_target_quat = current_target_quat
            frame.hand_primed = hand_primed
            frame.hand_armed = hand_is_armed
            frame.hand_position_rev = hand_sim.position_rev

            for b in balls:
                if ball_id is not None and b.id == ball_id:
                    result.ball_status = b.status

            result.frames.append(frame)
            t += MOCAP_DT

    finally:
        traj_mgr.shutdown()

    # Compute final target position error
    if result.final_target_pos is not None:
        true_target = np.array([
            true_lpos[0], true_lpos[1], true_lpos[2] - INITIAL_HEIGHT])
        result.final_target_pos_error_mm = float(
            np.linalg.norm(result.final_target_pos - true_target))

    return result


# ── 3D Visualization ────────────────────────────────────────────────

def visualize_catch(result: CatchSimResult, fps: int = 30,
                    slowdown: float = 3.0):
    """Animate the catch sequence in a 3D matplotlib viewer.

    Args:
        result: Completed simulation result with frames.
        fps: Animation frame rate.
        slowdown: Playback speed divisor (3.0 = 3x slower than real-time).
    """
    import matplotlib.pyplot as plt
    from jugglebot.motion.geometry import StewartGeometry
    from scipy.spatial.transform import Rotation

    geom = StewartGeometry()
    frames = result.frames
    if not frames:
        print("No frames to visualize")
        return

    # Subsample frames for target FPS
    sim_duration = frames[-1].time - frames[0].time
    anim_duration = sim_duration * slowdown
    n_anim_frames = max(1, int(anim_duration * fps))
    frame_indices = np.linspace(0, len(frames) - 1, n_anim_frames, dtype=int)

    # ── Figure layout ────────────────────────────────────────────
    # 3D viewer takes full left column; charts stacked on right:
    #   error (small) | legs | hand | timeline
    fig = plt.figure(figsize=(16, 10))
    fig.patch.set_facecolor('#1a1a2e')

    gs = fig.add_gridspec(4, 2, width_ratios=[2, 1],
                          height_ratios=[1, 1.5, 1, 0.8],
                          wspace=0.25, hspace=0.40,
                          left=0.05, right=0.97, top=0.93, bottom=0.06)

    ax3d = fig.add_subplot(gs[:, 0], projection='3d')
    ax_err = fig.add_subplot(gs[0, 1])
    ax_legs = fig.add_subplot(gs[1, 1])
    ax_hand = fig.add_subplot(gs[2, 1])
    ax_status = fig.add_subplot(gs[3, 1])

    # ── 3D viewport ──────────────────────────────────────────────
    ax3d.set_facecolor('#16213e')
    ax3d.set_xlim([-600, 600])
    ax3d.set_ylim([-600, 600])
    ax3d.set_zlim([0, 1500])
    ax3d.set_xlabel('X (mm)', fontsize=8, color='#aaa')
    ax3d.set_ylabel('Y (mm)', fontsize=8, color='#aaa')
    ax3d.set_zlabel('Z (mm)', fontsize=8, color='#aaa')
    ax3d.set_box_aspect([1, 1, 1.2])
    ax3d.tick_params(labelsize=6, colors='#888')

    # Draw catch plane
    cp_x = np.array([-500, 500, 500, -500, -500])
    cp_y = np.array([-500, -500, 500, 500, -500])
    cp_z = np.full(5, LANDING_Z)
    ax3d.plot(cp_x, cp_y, cp_z, '--', color='#334466', linewidth=0.8, alpha=0.5)

    # Draw base ring
    base_order = list(range(6)) + [0]
    ax3d.plot(geom.base_nodes[base_order, 0],
              geom.base_nodes[base_order, 1],
              geom.base_nodes[base_order, 2],
              'o-', color='#555555', linewidth=2, markersize=4)

    # True trajectory (full arc, drawn once)
    tp = result.throw_params
    tof = result.true_landing_time - tp.throw_time
    t_arc = np.linspace(0, tof, 200)
    arc = np.array([ballistic_position(tp.initial_position, tp.initial_velocity, dt)
                    for dt in t_arc])
    ax3d.plot(arc[:, 0], arc[:, 1], arc[:, 2], '-', color='#ffffff',
              linewidth=0.6, alpha=0.3, label='True trajectory')

    # True landing marker
    if result.true_landing_pos is not None:
        lp = result.true_landing_pos
        ax3d.plot([lp[0]], [lp[1]], [lp[2]], 'x', color='#ff4444',
                  markersize=12, markeredgewidth=2, label='True landing')

    # BB position marker
    ax3d.plot([tp.initial_position[0]], [tp.initial_position[1]],
              [tp.initial_position[2]], 's', color='#ffaa00', markersize=8,
              label='Ball Butler')

    # Animated artists
    ball_true_dot, = ax3d.plot([], [], [], 'o', color='#00ff88', markersize=10,
                                zorder=10, label='Ball (true)')
    ball_kf_dot, = ax3d.plot([], [], [], 'o', color='#00aaff', markersize=7,
                              zorder=9, label='Ball (KF)')
    landing_pred_dot, = ax3d.plot([], [], [], 'D', color='#ffaa00', markersize=8,
                                   markeredgewidth=1.5, markeredgecolor='#ff6600',
                                   zorder=8, label='Landing pred')

    # Platform ring
    plat_line, = ax3d.plot([], [], [], 'o-', color='#4488ff', linewidth=2.5,
                            markersize=4, label='Platform')
    leg_lines = []
    leg_colors = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4']
    for i in range(6):
        line, = ax3d.plot([], [], [], '-', color=leg_colors[i], linewidth=1.5)
        leg_lines.append(line)

    # Hand: vertical line (stroke range) + sphere (hand tip)
    hand_line, = ax3d.plot([], [], [], '-', color='#ff44ff', linewidth=2,
                            alpha=0.5, label='Hand stroke')
    hand_sphere, = ax3d.plot([], [], [], 'o', color='#ff44ff', markersize=12,
                              zorder=11, label='Hand')

    # Ghost marker for coordinator's raw target (where it *wants* to go)
    target_ghost, = ax3d.plot([], [], [], '+', color='#ff6666', markersize=14,
                               markeredgewidth=2, zorder=7, alpha=0.7,
                               label='Coord target')

    ax3d.legend(loc='upper right', fontsize=6, ncol=2, facecolor='#1a1a2e',
                edgecolor='#333', labelcolor='#ccc')

    # Info text
    info_text = ax3d.text2D(0.02, 0.97, '', transform=ax3d.transAxes,
                             fontsize=8, fontfamily='monospace', color='#cccccc',
                             verticalalignment='top')

    # ── Shared x-axis range for right panels ────────────────────
    t_min = frames[0].time
    t_max = frames[-1].time

    # ── Error plot ───────────────────────────────────────────────
    ax_err.set_facecolor('#16213e')
    ax_err.set_title('Landing Prediction Error', fontsize=9, color='#ccc')
    ax_err.set_ylabel('Error (mm)', fontsize=8, color='#aaa')
    ax_err.tick_params(labelsize=6, colors='#888')
    ax_err.tick_params(axis='x', labelbottom=False)  # shared with timeline below
    ax_err.grid(True, alpha=0.2)

    # Pre-plot full error history
    err_times = []
    err_vals = []
    for f in frames:
        if f.landing_pred_pos is not None and result.true_landing_pos is not None:
            err = np.linalg.norm(f.landing_pred_pos - result.true_landing_pos)
            err_times.append(f.time)
            err_vals.append(err)
    if err_times:
        ax_err.plot(err_times, err_vals, '-', color='#555555', linewidth=0.8,
                    alpha=0.4)
    err_line, = ax_err.plot([], [], '-', color='#ffaa00', linewidth=1.5)
    err_cursor = ax_err.axvline(0, color='#ff4444', linewidth=1, alpha=0.7)
    ax_err.set_xlim(t_min, t_max)
    if err_vals:
        ax_err.set_ylim(0, max(err_vals) * 1.1 + 1)

    # ── Leg motor positions plot ────────────────────────────────
    leg_stroke = hw.GEOM_LEG_STROKE_MM  # 280 mm
    ax_legs.set_facecolor('#16213e')
    ax_legs.set_title('Leg Extensions', fontsize=9, color='#ccc')
    ax_legs.set_ylabel('Extension (mm)', fontsize=8, color='#aaa')
    ax_legs.tick_params(labelsize=6, colors='#888')
    ax_legs.tick_params(axis='x', labelbottom=False)
    ax_legs.grid(True, alpha=0.2)
    ax_legs.set_xlim(t_min, t_max)
    ax_legs.set_ylim(-10, leg_stroke + 10)

    # Stroke limit reference lines
    ax_legs.axhline(0, color='#ff4444', linewidth=1.1, linestyle='--', alpha=0.7)
    ax_legs.axhline(leg_stroke, color='#ff4444', linewidth=1.1, linestyle='--',
                     alpha=0.7)
    ax_legs.text(t_max, 0, ' 0 mm', fontsize=6, color='#ff6666', va='center')
    ax_legs.text(t_max, leg_stroke, f' {leg_stroke:.0f} mm',
                  fontsize=6, color='#ff6666', va='center')

    # Pre-compute leg extension data for all frames
    leg_times = [f.time for f in frames if f.leg_extensions_mm is not None]
    leg_data = np.array([f.leg_extensions_mm for f in frames
                         if f.leg_extensions_mm is not None])

    # Pre-plot full traces as ghost
    for i in range(6):
        if len(leg_data) > 0:
            ax_legs.plot(leg_times, leg_data[:, i], '-', color=leg_colors[i],
                          linewidth=0.6, alpha=0.25)

    # Animated traces (one per leg, matching 3D colours)
    leg_trace_lines = []
    for i in range(6):
        line, = ax_legs.plot([], [], '-', color=leg_colors[i], linewidth=1.5,
                              label=f'Leg {i+1}')
        leg_trace_lines.append(line)
    ax_legs.legend(loc='upper right', fontsize=5, ncol=3, facecolor='#1a1a2e',
                    edgecolor='#333', labelcolor='#ccc')
    legs_cursor = ax_legs.axvline(0, color='#ff4444', linewidth=1, alpha=0.7)

    # ── Hand motor position plot ─────────────────────────────────
    ax_hand.set_facecolor('#16213e')
    ax_hand.set_title('Hand Motor Position', fontsize=9, color='#ccc')
    ax_hand.set_ylabel('Position (rev)', fontsize=8, color='#aaa')
    ax_hand.tick_params(labelsize=6, colors='#888')
    ax_hand.tick_params(axis='x', labelbottom=False)
    ax_hand.grid(True, alpha=0.2)
    ax_hand.set_xlim(t_min, t_max)
    ax_hand.set_ylim(-0.5, HAND_MAX_REV + 0.5)

    # Stroke limit reference lines
    ax_hand.axhline(0, color='#ff4444', linewidth=1.1, linestyle='--', alpha=0.7)
    ax_hand.axhline(HAND_MAX_REV, color='#ff4444', linewidth=1.1,
                     linestyle='--', alpha=0.7)
    ax_hand.text(t_max, 0, ' 0 rev', fontsize=6, color='#ff6666', va='center')
    ax_hand.text(t_max, HAND_MAX_REV, f' {HAND_MAX_REV:.1f} rev',
                  fontsize=6, color='#ff6666', va='center')
    # Prime position reference line
    ax_hand.axhline(HAND_PRIME_REV, color='#884488', linewidth=1,
                     linestyle='--', alpha=0.5)
    ax_hand.text(t_min, HAND_PRIME_REV, f'prime ({HAND_PRIME_REV:.1f}) ',
                  fontsize=6, color='#884488', va='bottom', ha='left')

    # Pre-plot full hand position trace (ghost)
    hand_times = [f.time for f in frames]
    hand_revs = [f.hand_position_rev for f in frames]
    ax_hand.plot(hand_times, hand_revs, '-', color='#555555', linewidth=0.8,
                 alpha=0.4)
    # Animated trace
    hand_trace_line, = ax_hand.plot([], [], '-', color='#ff44ff', linewidth=1.5)
    hand_cursor = ax_hand.axvline(0, color='#ff4444', linewidth=1, alpha=0.7)

    # ── Status timeline ──────────────────────────────────────────
    ax_status.set_facecolor('#16213e')
    ax_status.set_title('Schedule', fontsize=9, color='#ccc')
    ax_status.set_xlabel('Time (s)', fontsize=8, color='#aaa')
    ax_status.tick_params(labelsize=6, colors='#888')
    ax_status.set_xlim(t_min, t_max)
    ax_status.set_ylim(-0.5, 4.5)
    ax_status.set_yticks([0, 1, 2, 3])
    ax_status.set_yticklabels(['Ball', 'Track', 'Plat', 'Hand'],
                               fontsize=6, color='#aaa')
    ax_status.grid(True, alpha=0.15, axis='x')

    # Pre-draw status bars
    _draw_timeline_bars(ax_status, frames, result)
    timeline_cursor = ax_status.axvline(0, color='#ff4444', linewidth=1.5,
                                         alpha=0.8)

    # Legend — collect all colour/label pairs across the 4 rows
    from matplotlib.patches import Patch
    legend_entries = [
        Patch(facecolor='#ffaa00', alpha=0.6, label='Announced'),
        Patch(facecolor='#00ff88', alpha=0.6, label='In flight / Confirmed'),
        Patch(facecolor='#4488ff', alpha=0.6, label='Caught / Executing'),
        Patch(facecolor='#ff4444', alpha=0.6, label='Dropped'),
        Patch(facecolor='#555555', alpha=0.6, label='Idle / Unknown'),
        Patch(facecolor='#884488', alpha=0.6, label='Primed'),
        Patch(facecolor='#ff44ff', alpha=0.6, label='Armed'),
    ]
    ax_status.legend(handles=legend_entries, loc='upper right', fontsize=5,
                      ncol=2, facecolor='#1a1a2e', edgecolor='#333',
                      labelcolor='#ccc')

    fig.suptitle('Catch Simulation', fontsize=13, color='#eeeeee',
                 fontweight='bold')

    # Unbind matplotlib defaults that clash with 3D rotation
    import matplotlib as _mpl
    for _k in ('keymap.back', 'keymap.forward', 'keymap.home'):
        _mpl.rcParams[_k] = []

    # Controls help text
    controls_text = fig.text(
        0.5, 0.015,
        'SPACE: play/pause  |  . / ,  step +1/-1  |  '
        '] / [  step +10/-10  |  0 / 9  first/last frame',
        ha='center', fontsize=7, color='#666666', fontfamily='monospace')

    # Pause indicator
    pause_text = ax3d.text2D(0.98, 0.97, '', transform=ax3d.transAxes,
                              fontsize=10, fontfamily='monospace', color='#ff6666',
                              ha='right', va='top', fontweight='bold')

    # ── Render a single frame ────────────────────────────────────
    # Pre-compute full error trace so stepping backward works
    err_trace_full_t = []
    err_trace_full_v = []
    for f in frames:
        if f.landing_pred_pos is not None and result.true_landing_pos is not None:
            err = np.linalg.norm(f.landing_pred_pos - result.true_landing_pos)
            err_trace_full_t.append(f.time)
            err_trace_full_v.append(err)

    def render_frame(anim_idx):
        """Render a single animation frame by index."""
        fi = frame_indices[anim_idx]
        f = frames[fi]

        # Ball true position
        if f.ball_true_pos is not None:
            ball_true_dot.set_data_3d([f.ball_true_pos[0]],
                                       [f.ball_true_pos[1]],
                                       [f.ball_true_pos[2]])
        else:
            ball_true_dot.set_data_3d([], [], [])

        # Ball KF position
        if f.ball_kf_pos is not None:
            ball_kf_dot.set_data_3d([f.ball_kf_pos[0]],
                                     [f.ball_kf_pos[1]],
                                     [f.ball_kf_pos[2]])
        else:
            ball_kf_dot.set_data_3d([], [], [])

        # Landing prediction
        if f.landing_pred_pos is not None:
            landing_pred_dot.set_data_3d([f.landing_pred_pos[0]],
                                          [f.landing_pred_pos[1]],
                                          [f.landing_pred_pos[2]])
        else:
            landing_pred_dot.set_data_3d([], [], [])

        # Platform pose — render at trajectory-planned actual pose
        if f.platform_actual_pos is not None and f.platform_actual_quat is not None:
            q = f.platform_actual_quat
            rot = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
            pos = f.platform_actual_pos

            platform_centre = pos + geom.init_height_vec
            plat_world = platform_centre + (rot @ geom.plat_nodes.T).T

            order = list(range(6)) + [0]
            plat_line.set_data_3d(plat_world[order, 0],
                                   plat_world[order, 1],
                                   plat_world[order, 2])

            for i in range(6):
                leg_lines[i].set_data_3d(
                    [geom.base_nodes[i, 0], plat_world[i, 0]],
                    [geom.base_nodes[i, 1], plat_world[i, 1]],
                    [geom.base_nodes[i, 2], plat_world[i, 2]])

            # Hand: render as a line (stroke) + sphere (tip) based on
            # actual motor position from the hand simulator.
            # Hand axis bottom is offset from platform centroid along local Z.
            hand_ext_mm = f.hand_position_rev * HAND_MM_PER_REV
            hand_base = platform_centre + rot @ np.array([0, 0, HAND_BOTTOM_OFFSET_MM])
            hand_tip = hand_base + rot @ np.array([0, 0, max(hand_ext_mm, 0.0)])
            hand_color = '#ff44ff' if f.hand_armed else '#884488'
            if hand_ext_mm > 0.5:
                hand_line.set_data_3d([hand_base[0], hand_tip[0]],
                                       [hand_base[1], hand_tip[1]],
                                       [hand_base[2], hand_tip[2]])
                hand_line.set_color(hand_color)
            else:
                hand_line.set_data_3d([], [], [])
            hand_sphere.set_data_3d([hand_tip[0]],
                                     [hand_tip[1]],
                                     [hand_tip[2]])
            hand_sphere.set_color(hand_color)
        else:
            home_pos = np.zeros(3)
            platform_centre = home_pos + geom.init_height_vec
            plat_world = platform_centre + geom.plat_nodes

            order = list(range(6)) + [0]
            plat_line.set_data_3d(plat_world[order, 0],
                                   plat_world[order, 1],
                                   plat_world[order, 2])
            for i in range(6):
                leg_lines[i].set_data_3d(
                    [geom.base_nodes[i, 0], plat_world[i, 0]],
                    [geom.base_nodes[i, 1], plat_world[i, 1]],
                    [geom.base_nodes[i, 2], plat_world[i, 2]])
            hand_line.set_data_3d([], [], [])
            hand_sphere.set_data_3d([], [], [])

        # Ghost marker: coordinator's raw target position (in base frame)
        if f.platform_target_pos is not None:
            tp = f.platform_target_pos + geom.init_height_vec
            target_ghost.set_data_3d([tp[0]], [tp[1]], [tp[2]])
        else:
            target_ghost.set_data_3d([], [], [])

        # Error trace — show up to current time
        cut = 0
        for i, et in enumerate(err_trace_full_t):
            if et <= f.time:
                cut = i + 1
        err_line.set_data(err_trace_full_t[:cut], err_trace_full_v[:cut])

        # Leg extension traces — show up to current time
        leg_cut = 0
        for i, lt in enumerate(leg_times):
            if lt <= f.time:
                leg_cut = i + 1
        for i in range(6):
            if leg_cut > 0:
                leg_trace_lines[i].set_data(leg_times[:leg_cut],
                                             leg_data[:leg_cut, i])
            else:
                leg_trace_lines[i].set_data([], [])

        # Hand motor trace — show up to current time
        hand_cut = 0
        for i, ht in enumerate(hand_times):
            if ht <= f.time:
                hand_cut = i + 1
        hand_trace_line.set_data(hand_times[:hand_cut], hand_revs[:hand_cut])

        err_cursor.set_xdata([f.time, f.time])
        legs_cursor.set_xdata([f.time, f.time])
        hand_cursor.set_xdata([f.time, f.time])
        timeline_cursor.set_xdata([f.time, f.time])

        # Info text
        status_str = f.ball_status.name if f.ball_status else "---"
        tracking_str = f.ball_tracking.name if f.ball_tracking else "---"
        hand_str = "ARMED" if f.hand_armed else ("PRIMED" if f.hand_primed else "IDLE")
        traj_str = f.traj_state or "---"
        hand_mm = f.hand_position_rev * HAND_MM_PER_REV
        def _quat_to_euler_str(q):
            """Convert [w,x,y,z] quat to 'rx, ry, rz' degrees string."""
            r = Rotation.from_quat([q[1], q[2], q[3], q[0]])
            e = r.as_euler('xyz', degrees=True)
            return f"{e[0]:+.1f}, {e[1]:+.1f}, {e[2]:+.1f}"

        info = (
            f"t = {f.time:.3f}s  [{anim_idx+1}/{n_anim_frames}]\n"
            f"Ball: {status_str} ({tracking_str})\n"
            f"Hand: {hand_str} {f.hand_position_rev:.1f}rev ({hand_mm:.0f}mm)  Traj: {traj_str}\n"
        )
        if f.platform_actual_pos is not None:
            p = f.platform_actual_pos
            info += f"Platform pos:  [{p[0]:+.0f}, {p[1]:+.0f}, {p[2]:+.0f}] mm"
            if f.platform_actual_quat is not None:
                info += f"  rot: [{_quat_to_euler_str(f.platform_actual_quat)}] deg"
            info += "\n"
        if f.platform_target_pos is not None:
            p = f.platform_target_pos
            info += f"Platform tgt:  [{p[0]:+.0f}, {p[1]:+.0f}, {p[2]:+.0f}] mm"
            if f.platform_target_quat is not None:
                info += f"  rot: [{_quat_to_euler_str(f.platform_target_quat)}] deg"
        info_text.set_text(info)

        fig.canvas.draw_idle()

    # ── Playback controller ──────────────────────────────────────
    state = {'idx': 0, 'playing': True, 'timer': None}

    def _advance():
        """Timer callback: advance one frame while playing."""
        if not state['playing']:
            return
        state['idx'] = (state['idx'] + 1) % n_anim_frames
        render_frame(state['idx'])
        pause_text.set_text('')

    def _step(delta: int):
        """Step by delta frames (positive or negative)."""
        state['idx'] = max(0, min(n_anim_frames - 1, state['idx'] + delta))
        render_frame(state['idx'])

    def _on_key(event):
        if event.key == ' ':
            state['playing'] = not state['playing']
            if state['playing']:
                pause_text.set_text('')
            else:
                pause_text.set_text('PAUSED')
                fig.canvas.draw_idle()

        elif event.key == '.':
            state['playing'] = False
            pause_text.set_text('PAUSED')
            _step(1)
        elif event.key == ',':
            state['playing'] = False
            pause_text.set_text('PAUSED')
            _step(-1)
        elif event.key == ']':
            state['playing'] = False
            pause_text.set_text('PAUSED')
            _step(10)
        elif event.key == '[':
            state['playing'] = False
            pause_text.set_text('PAUSED')
            _step(-10)
        elif event.key == '0':
            state['playing'] = False
            pause_text.set_text('PAUSED')
            state['idx'] = 0
            render_frame(0)
        elif event.key == '9':
            state['playing'] = False
            pause_text.set_text('PAUSED')
            state['idx'] = n_anim_frames - 1
            render_frame(state['idx'])

    fig.canvas.mpl_connect('key_press_event', _on_key)

    # Render first frame
    render_frame(0)

    # Start playback timer
    interval_ms = 1000.0 / fps
    timer = fig.canvas.new_timer(interval=int(interval_ms))
    timer.add_callback(_advance)
    timer.start()
    state['timer'] = timer

    plt.show()


def _draw_timeline_bars(ax, frames: List[SimFrame], result: CatchSimResult):
    """Draw colored horizontal bars for the event timeline."""
    # Ball status (row 0)
    _draw_status_spans(ax, frames, row=0, key='ball_status', color_map={
        BallStatus.TO_BE_THROWN: ('#ffaa00', 'Announced'),
        BallStatus.IN_FLIGHT: ('#00ff88', 'In flight'),
        BallStatus.CAUGHT: ('#4488ff', 'Caught'),
        BallStatus.DROPPED: ('#ff4444', 'Dropped'),
        BallStatus.UNKNOWN: ('#888888', 'Unknown'),
    })

    # Tracking (row 1)
    _draw_status_spans(ax, frames, row=1, key='ball_tracking', color_map={
        TrackingConfidence.ANNOUNCED: ('#ffaa00', 'Announced'),
        TrackingConfidence.CONFIRMED: ('#00ff88', 'Confirmed'),
    })

    # Platform trajectory state (row 2)
    _draw_status_spans(ax, frames, row=2, key='traj_state', color_map={
        'IDLE': ('#555555', 'Idle'),
        'EXECUTING': ('#4488ff', 'Executing'),
        'RETURNING': ('#ffaa00', 'Returning'),
        'COMPLETE': ('#00ff88', 'Complete'),
    })

    # Hand (row 3)
    primed_times = [f.time for f in frames if f.hand_primed and not f.hand_armed]
    armed_times = [f.time for f in frames if f.hand_armed]
    if primed_times:
        ax.barh(3, primed_times[-1] - primed_times[0] + MOCAP_DT,
                left=primed_times[0], height=0.6, color='#884488', alpha=0.6,
                edgecolor='none')
    if armed_times:
        ax.barh(3, armed_times[-1] - armed_times[0] + MOCAP_DT,
                left=armed_times[0], height=0.6, color='#ff44ff', alpha=0.6,
                edgecolor='none')
        ax.text(armed_times[0], 3.35, 'Armed', fontsize=6, color='#ff88ff')

    # Throw time and landing time markers
    tp = result.throw_params
    ax.axvline(tp.throw_time, color='#ffaa00', linewidth=1, linestyle='--', alpha=0.5)
    ax.text(tp.throw_time, 3.7, 'throw', fontsize=6, color='#ffaa00', ha='center')
    if result.true_landing_time > 0:
        ax.axvline(result.true_landing_time, color='#ff4444', linewidth=1,
                   linestyle='--', alpha=0.5)
        ax.text(result.true_landing_time, 3.7, 'land', fontsize=6,
                color='#ff4444', ha='center')


def _draw_status_spans(ax, frames, row, key, color_map):
    """Draw colored spans for a status field across the timeline."""
    if not frames:
        return
    current_val = None
    span_start = frames[0].time
    for f in frames:
        val = getattr(f, key, None)
        if val != current_val:
            if current_val is not None and current_val in color_map:
                color, label = color_map[current_val]
                ax.barh(row, f.time - span_start, left=span_start,
                        height=0.6, color=color, alpha=0.6, edgecolor='none')
            current_val = val
            span_start = f.time
    # Final span
    if current_val is not None and current_val in color_map:
        color, label = color_map[current_val]
        ax.barh(row, frames[-1].time - span_start + MOCAP_DT, left=span_start,
                height=0.6, color=color, alpha=0.6, edgecolor='none')


# ── Print results ───────────────────────────────────────────────────

def print_result(result: CatchSimResult, label: str = ""):
    """Print simulation results in a human-readable format."""
    if label:
        print(f"\n{'=' * 60}")
        print(f"  {label}")
        print(f"{'=' * 60}")

    tp = result.throw_params
    print(f"\nThrow: source={tp.source}, dest='{tp.destination}'")
    print(f"  Initial pos: [{tp.initial_position[0]:.0f}, "
          f"{tp.initial_position[1]:.0f}, {tp.initial_position[2]:.0f}] mm")
    speed = np.linalg.norm(tp.initial_velocity) / 1000.0
    print(f"  Initial speed: {speed:.2f} m/s")
    print(f"  Throw time: {tp.throw_time:.3f} s")

    if result.true_landing_pos is not None:
        print(f"\nTrue landing:")
        print(f"  Position: [{result.true_landing_pos[0]:.1f}, "
              f"{result.true_landing_pos[1]:.1f}, {result.true_landing_pos[2]:.1f}] mm")
        tof = result.true_landing_time - tp.throw_time
        print(f"  Time of flight: {tof:.3f} s (landing at t={result.true_landing_time:.3f} s)")
        speed_at_land = np.linalg.norm(result.true_landing_vel) / 1000.0
        print(f"  Speed at landing: {speed_at_land:.2f} m/s")

    print(f"\nTracking:")
    print(f"  Confirmed: {'YES' if result.tracking_confirmed else 'NO'}")
    if result.tracking_confirmed:
        print(f"  First detection: {result.first_detection_delay_ms:.0f} ms after throw")

    if result.landing_pos_errors_mm:
        errs = result.landing_pos_errors_mm
        print(f"  Landing prediction error (pos):")
        print(f"    First: {errs[0]:.1f} mm  |  Final: {errs[-1]:.1f} mm  |  "
              f"Min: {min(errs):.1f} mm")

    if result.landing_time_errors_ms:
        terrs = result.landing_time_errors_ms
        print(f"  Landing prediction error (time):")
        print(f"    First: {terrs[0]:.1f} ms  |  Final: {terrs[-1]:.1f} ms  |  "
              f"Min (abs): {min(abs(e) for e in terrs):.1f} ms")

    print(f"\nCatch coordinator:")
    if tp.destination == "":
        print(f"  (No destination -- coordinator correctly ignores unassigned balls)")
    print(f"  Commands issued: {result.catch_commands_issued}")
    if result.catch_commands_issued > 0:
        lead_time = result.true_landing_time - result.first_catch_command_time
        print(f"  First command: {lead_time * 1000.0:.0f} ms before landing")
        print(f"  Final target error: {result.final_target_pos_error_mm:.1f} mm")

    print(f"\nHand control:")
    if result.hand_armed:
        print(f"  Hand armed: YES")
        print(f"  Prime position: {result.hand_prime_rev:.3f} rev")
        print(f"  Event delay: {result.hand_event_delay_s:.3f} s")
        print(f"  Event velocity: {result.hand_event_vel_mps:.2f} m/s")
    else:
        print(f"  Hand armed: NO")

    print(f"\nFinal ball status: {result.ball_status.name}")


def run_noise_sweep(throw: ThrowParams, noise_levels: list):
    """Run multiple simulations at different noise levels."""
    print(f"\n{'=' * 60}")
    print(f"  Noise sweep: {len(noise_levels)} levels")
    print(f"{'=' * 60}")
    print(f"\n{'Noise':>8s} | {'Pos err':>8s} | {'Time err':>9s} | "
          f"{'Detect':>7s} | {'Commands':>8s} | {'Hand':>5s}")
    print(f"{' (mm)':>8s} | {'(mm)':>8s} | {'(ms)':>9s} | "
          f"{'(ms)':>7s} | {'':>8s} | {'':>5s}")
    print("-" * 60)

    for noise in noise_levels:
        result = run_catch_simulation(throw, noise_std_mm=noise)
        final_pos_err = result.landing_pos_errors_mm[-1] if result.landing_pos_errors_mm else float('inf')
        final_time_err = result.landing_time_errors_ms[-1] if result.landing_time_errors_ms else float('inf')
        detect = f"{result.first_detection_delay_ms:.0f}" if result.tracking_confirmed else "N/A"
        hand = "YES" if result.hand_armed else "NO"

        print(f"{noise:8.1f} | {final_pos_err:8.1f} | {final_time_err:9.1f} | "
              f"{detect:>7s} | {result.catch_commands_issued:8d} | {hand:>5s}")


# ── Entry point ──────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Simulated catch sequence test",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--noise", type=float, default=2.0,
                        help="Mocap noise std dev in mm (default: 2.0)")
    parser.add_argument("--delay", type=float, default=0.1,
                        help="Mocap detection delay in seconds (default: 0.1)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="Throw speed in m/s (default: 1.0)")
    parser.add_argument("--yaw", type=float, default=90.0,
                        help="BB yaw angle in degrees (default: 90.0)")
    parser.add_argument("--pitch", type=float, default=80.0,
                        help="BB pitch angle in degrees (default: 80.0)")
    parser.add_argument("--landing", type=float, nargs=2, metavar=('X', 'Y'),
                        help="Landing position X Y in mm (base frame). "
                             "Back-computes throw to hit this point.")
    parser.add_argument("--launch", type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help="Launch position X Y Z in mm (base frame). "
                             "Overrides BB_POSITION. Use with --landing for "
                             "steeper angles (e.g. --launch 300 -300 900).")
    parser.add_argument("--tof", type=float, default=0.6,
                        help="Time of flight for --landing mode (default: 0.6s)")
    parser.add_argument("--human-throw", action="store_true",
                        help="Simulate a human throw (no announcement)")
    parser.add_argument("--sweep", action="store_true",
                        help="Run noise sweep from 0.5 to 20 mm")
    parser.add_argument("--viz", action="store_true",
                        help="Show 3D animated visualization")
    parser.add_argument("--fps", type=int, default=30,
                        help="Visualization FPS (default: 30)")
    parser.add_argument("--slowdown", type=float, default=3.0,
                        help="Playback slowdown factor (default: 3.0)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Random seed (default: 42)")
    args = parser.parse_args()

    if args.human_throw:
        throw = make_human_throw()
        announced = False
        label = "Human throw (no announcement)"
    elif args.landing is not None:
        lx, ly = args.landing
        launch = np.array(args.launch) if args.launch else None
        throw = make_throw_to_landing(
            landing_xy=(lx, ly), flight_time=args.tof, launch_pos=launch)
        announced = True
        src = f"from [{launch[0]:.0f},{launch[1]:.0f},{launch[2]:.0f}]" if launch is not None else "from BB"
        label = f"Throw {src} to landing ({lx:.0f}, {ly:.0f}) mm, TOF={args.tof:.2f}s"
    else:
        throw = make_bb_throw(
            speed_mps=args.speed,
            yaw_deg=args.yaw,
            pitch_deg=args.pitch,
        )
        announced = True
        label = (f"Ball Butler throw: {args.speed:.1f} m/s, "
                 f"yaw={args.yaw:.0f}, pitch={args.pitch:.0f}")

    if args.sweep:
        run_noise_sweep(throw, [0.5, 1.0, 2.0, 5.0, 10.0, 20.0])
    else:
        result = run_catch_simulation(
            throw,
            noise_std_mm=args.noise,
            mocap_delay_s=args.delay,
            announced=announced,
            seed=args.seed,
        )
        print_result(result, label)

        if args.viz:
            visualize_catch(result, fps=args.fps, slowdown=args.slowdown)


if __name__ == "__main__":
    main()
