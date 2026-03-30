#!/usr/bin/env python3
"""Throw-catch cycle test: Jugglebot throws and catches the same ball.

Ball is thrown from (-100, 0, 170) to (100, 0, 170) mm in platform frame,
reaching 1m height above the throw/catch plane. The platform tilts to aim
the throw, then repositions to catch the ball using the full pipeline:
BallTracker + CatchCoordinator + TrajectoryManager + HandSim.

No ROS2 or hardware required. Uses the same visualizer as catch_sim_test.

Usage:
    python tools/throw_catch_test.py
    python tools/throw_catch_test.py --viz
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Optional

import numpy as np

# ── Path setup ────────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_SCRIPT_DIR)
_CONFIG_DIR = os.path.join(_PROJECT_ROOT, 'config', 'generated')
_ROS_PKG_DIR = os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'jugglebot')

sys.path.insert(0, _CONFIG_DIR)
sys.path.insert(0, _ROS_PKG_DIR)
sys.path.insert(0, _SCRIPT_DIR)  # for catch_sim_test imports

import hardware_config as hw
from jugglebot.tracking.matcher import BallTracker
from jugglebot.tracking.ball import BallStatus, TrackingConfidence
from jugglebot.tracking.ballistics import GRAVITY_MMPS2, predict_landing_state
from jugglebot.catch_coordinator import CatchCoordinator

# Reuse data structures and visualizer from catch_sim_test
from catch_sim_test import (
    SimFrame, CatchSimResult, ThrowParams,
    ballistic_position, visualize_catch,
    MOCAP_DT, INITIAL_HEIGHT, HAND_BOTTOM_OFFSET_MM,
    _block_wait_for_async_commit,
)

# ── Constants ────────────────────────────────────────────────────
_TOTAL_STROKE = hw.TEENSY_TRAJ_HAND_STROKE_M - 2.0 * hw.TEENSY_TRAJ_STROKE_MARGIN_M
_LINEAR_GAIN = hw.TEENSY_LINEAR_GAIN
_INERTIA_RATIO = hw.TEENSY_TRAJ_INERTIA_RATIO
_CATCH_VEL_RATIO = hw.TEENSY_TRAJ_CATCH_VEL_RATIO
_CATCH_VEL_HOLD_PCT = hw.TEENSY_TRAJ_CATCH_VEL_HOLD_PCT
_THROW_VEL_HOLD_PCT = hw.TEENSY_TRAJ_THROW_VEL_HOLD_PCT
HAND_SMOOTH_ACCEL_RPS2 = hw.TEENSY_TRAJ_MAX_SMOOTH_MOVE_HAND_ACCEL_RPS2


# ── Throw/catch ballistics ────────────────────────────────────────

def compute_throw_ballistics(
    throw_pos_plat: np.ndarray,
    catch_pos_plat: np.ndarray,
    apex_height_mm: float,
) -> dict:
    """Compute throw velocity vector and platform tilt for a parabolic arc."""
    from scipy.spatial.transform import Rotation

    throw_base = throw_pos_plat.copy()
    throw_base[2] += INITIAL_HEIGHT
    catch_base = catch_pos_plat.copy()
    catch_base[2] += INITIAL_HEIGHT

    dz_throw_to_apex = apex_height_mm + (catch_base[2] - throw_base[2]) / 2.0
    v_z = math.sqrt(2.0 * GRAVITY_MMPS2 * dz_throw_to_apex)
    t_up = v_z / GRAVITY_MMPS2
    dz_apex_to_catch = dz_throw_to_apex + (throw_base[2] - catch_base[2])
    t_down = math.sqrt(2.0 * dz_apex_to_catch / GRAVITY_MMPS2)
    flight_time = t_up + t_down

    dx = catch_base[0] - throw_base[0]
    dy = catch_base[1] - throw_base[1]
    v_x = dx / flight_time
    v_y = dy / flight_time
    v_throw_mps = math.sqrt(v_x**2 + v_y**2 + v_z**2) / 1000.0

    throw_vel = np.array([v_x, v_y, v_z])
    v_hat = throw_vel / np.linalg.norm(throw_vel)

    z_axis = np.array([0.0, 0.0, 1.0])
    cross = np.cross(z_axis, v_hat)
    cross_norm = np.linalg.norm(cross)
    if cross_norm > 1e-9:
        axis = cross / cross_norm
        angle = math.acos(np.clip(np.dot(z_axis, v_hat), -1.0, 1.0))
        tilt_rotvec = axis * angle
    else:
        tilt_rotvec = np.zeros(3)

    R = Rotation.from_rotvec(tilt_rotvec).as_matrix()
    release_offset = np.array([0, 0, HAND_BOTTOM_OFFSET_MM + hw.HAND_THROW_POS_M * 1000])
    release_pos_base = throw_base + R @ release_offset

    vel_hold = _THROW_VEL_HOLD_PCT * _TOTAL_STROKE
    accel_stroke = _TOTAL_STROKE - vel_hold
    t_acc_throw = 2.0 / (_INERTIA_RATIO + 1.0) * accel_stroke / v_throw_mps
    t_vel_throw = vel_hold / v_throw_mps
    t2 = t_acc_throw + t_vel_throw

    landing_z = catch_base[2] + hw.HAND_CATCH_OFFSET_MM

    return {
        'throw_vel_world': throw_vel,
        'tilt_rotvec': tilt_rotvec,
        'v_throw_mps': v_throw_mps,
        'flight_time_s': flight_time,
        'release_pos_base': release_pos_base,
        't2_s': t2,
        'landing_z': landing_z,
        'throw_base': throw_base,
        'catch_base': catch_base,
    }


# ── Hand trajectory math (mirrors Trajectory.h) ──────────────────

def _calc_throw_params(v_throw_mps: float) -> dict:
    vt = v_throw_mps
    vel_hold = _THROW_VEL_HOLD_PCT * _TOTAL_STROKE
    accel_stroke = _TOTAL_STROKE - vel_hold

    t_acc = 2.0 / (_INERTIA_RATIO + 1.0) * accel_stroke / vt
    t_vel = vel_hold / vt
    t_dec = t_acc * _INERTIA_RATIO

    throwA = vt / t_acc
    throwD = -throwA / _INERTIA_RATIO

    x1_m = 0.5 * throwA * t_acc**2
    x2_m = x1_m + vt * t_vel
    x3_m = x2_m + vt * t_dec + 0.5 * throwD * t_dec**2

    return {
        't_acc': t_acc, 't_vel': t_vel, 't_dec': t_dec,
        'throwA': throwA, 'throwD': throwD,
        'x1_rev': x1_m * _LINEAR_GAIN,
        'x2_rev': x2_m * _LINEAR_GAIN,
        'x3_rev': x3_m * _LINEAR_GAIN,
    }


def _calc_catch_params(event_vel_mps: float) -> dict:
    x3_m = _TOTAL_STROKE
    vC = -_CATCH_VEL_RATIO * event_vel_mps
    irC = 1.0 / _INERTIA_RATIO
    vel_hold_m = _CATCH_VEL_HOLD_PCT * _TOTAL_STROKE
    accel_stroke_m = _TOTAL_STROKE - vel_hold_m

    t_acc = -(2.0 / (irC + 1.0)) * accel_stroke_m / vC
    t_vel = -vel_hold_m / vC
    t_dec = t_acc * irC

    catchA = vC / t_acc
    catchD = -catchA / irC
    x5_m = x3_m + 0.5 * catchA * t_acc**2
    x6_m = x5_m + vC * t_vel

    return {
        't_acc': t_acc, 't_vel': t_vel, 't_dec': t_dec,
        'catchA': catchA, 'catchD': catchD, 'vC': vC,
        'x3_rev': x3_m * _LINEAR_GAIN,
        'x5_rev': x5_m * _LINEAR_GAIN,
        'x6_rev': x6_m * _LINEAR_GAIN,
    }


# ── Hand motor simulator (throw + catch) ─────────────────────────

class HandSim:
    """Hand motor simulator supporting both throw and catch trajectories."""

    def __init__(self):
        self.position_rev: float = 0.0
        self._move_start_time: float = -1.0
        self._move_start_rev: float = 0.0
        self._move_target_rev: float = 0.0
        self._move_duration: float = 0.0
        self._move_peak_vel: float = 0.0
        self._throw_armed: bool = False
        self._throw_start_time: float = -1.0
        self._throw_params: dict = {}
        self._catch_armed: bool = False
        self._catch_start_time: float = -1.0
        self._catch_params: dict = {}

    def command_throw(self, start_time: float, v_throw_mps: float):
        self._throw_params = _calc_throw_params(v_throw_mps)
        self._throw_armed = True
        self._throw_start_time = start_time

    @property
    def throw_release_time(self) -> float:
        if not self._throw_armed:
            return -1.0
        return self._throw_start_time + self._throw_params['t_acc'] + self._throw_params['t_vel']

    def command_catch(self, event_time: float, event_vel_mps: float):
        self._catch_params = _calc_catch_params(event_vel_mps)
        self._catch_armed = True
        self._catch_start_time = event_time - self._catch_params['t_acc']

    def step(self, t: float):
        if self._catch_armed and t >= self._catch_start_time:
            self._eval_catch(t)
            return
        if self._throw_armed and t >= self._throw_start_time:
            self._eval_throw(t)
            return
        if self._move_start_time >= 0 and t >= self._move_start_time:
            self._eval_smooth_move(t)
            return

    def _eval_throw(self, t: float):
        p = self._throw_params
        tau = t - self._throw_start_time
        t_acc, t_vel, t_dec = p['t_acc'], p['t_vel'], p['t_dec']
        if tau < t_acc:
            pos_rev = 0.5 * p['throwA'] * _LINEAR_GAIN * tau**2
        elif tau < t_acc + t_vel:
            dt = tau - t_acc
            vt_rev = p['throwA'] * _LINEAR_GAIN * t_acc
            pos_rev = p['x1_rev'] + vt_rev * dt
        elif tau < t_acc + t_vel + t_dec:
            dt = tau - t_acc - t_vel
            vt_rev = p['throwA'] * _LINEAR_GAIN * t_acc
            pos_rev = p['x2_rev'] + vt_rev * dt + 0.5 * p['throwD'] * _LINEAR_GAIN * dt**2
        else:
            pos_rev = p['x3_rev']
        self.position_rev = max(0.0, pos_rev)

    def _eval_catch(self, t: float):
        p = self._catch_params
        tau = t - self._catch_start_time
        t_acc, t_vel, t_dec = p['t_acc'], p['t_vel'], p['t_dec']
        if tau < t_acc:
            pos_rev = p['x3_rev'] + 0.5 * p['catchA'] * _LINEAR_GAIN * tau**2
        elif tau < t_acc + t_vel:
            dt = tau - t_acc
            pos_rev = p['x5_rev'] + p['vC'] * _LINEAR_GAIN * dt
        elif tau < t_acc + t_vel + t_dec:
            dt = tau - t_acc - t_vel
            pos_rev = p['x6_rev'] + p['vC'] * _LINEAR_GAIN * dt + 0.5 * p['catchD'] * _LINEAR_GAIN * dt**2
        else:
            dt_end = t_dec
            pos_rev = p['x6_rev'] + p['vC'] * _LINEAR_GAIN * dt_end + 0.5 * p['catchD'] * _LINEAR_GAIN * dt_end**2
        self.position_rev = max(0.0, pos_rev)

    def _eval_smooth_move(self, t: float):
        elapsed = t - self._move_start_time
        if elapsed >= self._move_duration:
            self.position_rev = self._move_target_rev
            return
        half = self._move_duration / 2.0
        dist = self._move_target_rev - self._move_start_rev
        sign = 1.0 if dist > 0 else -1.0
        if elapsed <= half:
            self.position_rev = self._move_start_rev + sign * 0.5 * HAND_SMOOTH_ACCEL_RPS2 * elapsed**2
        else:
            t_d = elapsed - half
            mid_pos = self._move_start_rev + sign * 0.5 * HAND_SMOOTH_ACCEL_RPS2 * half**2
            self.position_rev = mid_pos + sign * self._move_peak_vel * t_d - sign * 0.5 * HAND_SMOOTH_ACCEL_RPS2 * t_d**2


# ── Main simulation ──────────────────────────────────────────────

def run_throw_catch(
    throw_pos_plat: np.ndarray = np.array([-100.0, 0.0, 170.0]),
    catch_pos_plat: np.ndarray = np.array([100.0, 0.0, 170.0]),
    apex_height_mm: float = 1000.0,
    noise_std_mm: float = 2.0,
    seed: int = 42,
) -> CatchSimResult:
    """Run a full throw-catch cycle simulation.

    Returns a CatchSimResult compatible with catch_sim_test's visualizer.
    """
    from scipy.spatial.transform import Rotation
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.dynamics import DynamicsParams
    from jugglebot.motion.trajectory_manager import TrajectoryManager
    from jugglebot.motion.ik_solver import pose_to_leg_lengths, rotvec_to_rot_matrix

    rng = np.random.RandomState(seed)

    # ── Compute throw ballistics ──────────────────────────────────
    bals = compute_throw_ballistics(throw_pos_plat, catch_pos_plat, apex_height_mm)
    landing_z = bals['landing_z']

    # Platform needs ~0.8s to reach the throw position from active pose.
    # Give it 1.0s, then throw starts once it's settled.
    THROW_START_TIME = 1.0
    release_time = THROW_START_TIME + bals['t2_s']
    ball_landing_time = release_time + bals['flight_time_s']
    sim_end = ball_landing_time + 0.5

    ball_initial_pos = bals['release_pos_base']
    ball_initial_vel = bals['throw_vel_world']

    # Compute true landing state for result tracking.
    # The ball is released BELOW the landing plane going UP, so
    # predict_landing_state would pick the immediate upward crossing.
    # Propagate past the apex first so only the downward crossing remains.
    t_apex = ball_initial_vel[2] / GRAVITY_MMPS2  # time to apex
    apex_pos = ballistic_position(ball_initial_pos, ball_initial_vel, t_apex + 0.01)
    apex_vel = ball_initial_vel.copy()
    apex_vel[2] -= GRAVITY_MMPS2 * (t_apex + 0.01)
    true_landing = predict_landing_state(apex_pos, apex_vel, landing_z)
    true_lpos, true_lvel, true_ttl = true_landing
    true_landing_time = release_time + t_apex + 0.01 + true_ttl

    print("=== Throw-Catch Ballistics ===")
    print(f"  Throw: {throw_pos_plat} -> Catch: {catch_pos_plat} (platform frame, mm)")
    print(f"  Apex: {apex_height_mm:.0f} mm, Flight: {bals['flight_time_s']*1000:.0f} ms, "
          f"Tilt: {np.degrees(np.linalg.norm(bals['tilt_rotvec'])):.2f} deg")
    print(f"  Hand throw speed: {bals['v_throw_mps']:.3f} m/s")
    print(f"  Timeline: throw@{THROW_START_TIME:.2f}s, release@{release_time:.3f}s, "
          f"land@{ball_landing_time:.3f}s")
    print()

    # ── Build result object (catch_sim_test compatible) ────────────
    throw_params = ThrowParams(
        initial_position=ball_initial_pos.copy(),
        initial_velocity=ball_initial_vel.copy(),
        throw_time=release_time,
        source="self",
        destination="jugglebot",
    )
    result = CatchSimResult(throw_params=throw_params)
    result.true_landing_pos = true_lpos
    result.true_landing_vel = true_lvel
    result.true_landing_time = true_landing_time

    # ── Set up trajectory planner ─────────────────────────────────
    geom = StewartGeometry()
    dynamics_params = DynamicsParams.from_config()
    sim_clock = [0.0]
    traj_mgr = TrajectoryManager(geom, dynamics_params, clock=lambda: sim_clock[0])

    active_pose = np.array([0.0, 0.0, float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM),
                            0.0, 0.0, 0.0])
    traj_mgr.set_hold_pose(active_pose)

    throw_pose = np.array([
        throw_pos_plat[0], throw_pos_plat[1], throw_pos_plat[2],
        bals['tilt_rotvec'][0], bals['tilt_rotvec'][1], bals['tilt_rotvec'][2],
    ])

    # ── Set up tracker and coordinator ────────────────────────────
    tracker = BallTracker(dt=MOCAP_DT, landing_z=landing_z)
    coordinator = CatchCoordinator(
        robot_name="jugglebot",
        initial_height_mm=INITIAL_HEIGHT,
        landing_z_offset_mm=hw.JB_OP_DEFAULT_ACTIVE_Z_MM + hw.HAND_CATCH_OFFSET_MM,
        hand_catch_offset_mm=hw.HAND_CATCH_OFFSET_MM,
        catch_angle_limit_deg=30.0,
    )

    hand_sim = HandSim()

    # Submit move-to-throw trajectory (platform needs ~0.8s for 100mm lateral move)
    queued = traj_mgr.request_dynamic_target(
        target_pos=throw_pose[:3],
        target_quat=Rotation.from_rotvec(throw_pose[3:]).as_quat()[[3, 0, 1, 2]],
        target_vel=np.zeros(3),
        arrival_time=THROW_START_TIME - 0.05,
        t_now=0.0,
    )
    if queued:
        accepted = _block_wait_for_async_commit(traj_mgr)
        if not accepted:
            print("WARNING: Move-to-throw trajectory rejected by feasibility checker")
    else:
        print("WARNING: Move-to-throw trajectory not queued")

    hand_sim.command_throw(THROW_START_TIME, bals['v_throw_mps'])

    # ── Simulation state ──────────────────────────────────────────
    ball_released = False
    ball_id = None
    hand_catch_armed = False
    hand_primed = False
    last_traj_submit_time = -1.0
    catch_target_submitted = False
    current_target_pos = None
    current_target_quat = None

    # ── Simulation loop ───────────────────────────────────────────
    t = 0.0
    try:
        while t < sim_end:
            sim_clock[0] = t
            frame = SimFrame(time=t)

            # ── Ball release ──────────────────────────────────────
            if not ball_released and t >= release_time:
                ball_released = True
                ball_id = tracker.handle_announcement(
                    initial_position=ball_initial_pos,
                    initial_velocity=ball_initial_vel,
                    throw_time=release_time,
                    source="self",
                    destination="jugglebot",
                )
                print(f"  Ball released at t={release_time:.3f}s")

            # ── Generate markers ──────────────────────────────────
            markers = []
            if ball_released:
                dt_flight = t - release_time
                if dt_flight > 0:
                    true_pos = ballistic_position(ball_initial_pos, ball_initial_vel, dt_flight)
                    frame.ball_true_pos = true_pos.copy()
                    if dt_flight > 0.02 and true_pos[2] > landing_z - 50.0:
                        noisy_pos = true_pos + rng.randn(3) * noise_std_mm
                        markers.append(noisy_pos)

            # ── Run tracker ───────────────────────────────────────
            balls = tracker.process_frame(markers, t)

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
            if not result.tracking_confirmed:
                for b in balls:
                    if b.tracking == TrackingConfidence.CONFIRMED:
                        result.tracking_confirmed = True
                        result.first_detection_time = t
                        result.first_detection_delay_ms = (t - release_time) * 1000.0
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

                current_target_pos = cmd.target_pos.copy()
                current_target_quat = cmd.target_quat.copy()
                result.final_target_pos = current_target_pos.copy()

                if not catch_target_submitted and (t - last_traj_submit_time) >= 0.050:
                    queued = traj_mgr.request_dynamic_target(
                        target_pos=cmd.target_pos,
                        target_quat=cmd.target_quat,
                        target_vel=cmd.target_vel,
                        arrival_time=cmd.landing_time,
                        t_now=t,
                    )
                    if queued:
                        last_traj_submit_time = t
                        accepted = _block_wait_for_async_commit(traj_mgr)
                        if accepted:
                            catch_target_submitted = True
                            print(f"  Catch target accepted at t={t:.3f}s, "
                                  f"pos={cmd.target_pos.astype(int)}")

                if cmd.arm_hand and not hand_catch_armed:
                    event_delay = cmd.landing_time - t
                    if event_delay >= 0.15:
                        hand_sim.command_catch(cmd.landing_time, cmd.event_vel_mps)
                        hand_catch_armed = True
                        result.hand_armed = True
                        result.hand_event_vel_mps = cmd.event_vel_mps
                        result.hand_event_delay_s = event_delay
                        result.hand_prime_rev = hw.JB_OP_HAND_CATCH_PRIME_REV
                        print(f"  Hand catch armed at t={t:.3f}s, "
                              f"vel={cmd.event_vel_mps:.2f} m/s, "
                              f"delay={event_delay:.3f}s")

            # ── Update hand ───────────────────────────────────────
            hand_sim.step(t)

            # ── Evaluate trajectory ───────────────────────────────
            traj_mgr.evaluate(t)
            pose_6dof = traj_mgr.current_pose_6dof

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

            rot_matrix = rotvec_to_rot_matrix(rotvec)
            frame.leg_extensions_mm = pose_to_leg_lengths(pose_6dof[:3], rot_matrix, geom)

            frame.platform_target_pos = current_target_pos
            frame.platform_target_quat = current_target_quat
            frame.hand_primed = hand_primed
            frame.hand_armed = hand_catch_armed
            frame.hand_position_rev = hand_sim.position_rev

            for b in balls:
                if ball_id is not None and b.id == ball_id:
                    result.ball_status = b.status

            result.frames.append(frame)
            t += MOCAP_DT

    finally:
        traj_mgr.shutdown()

    # ── Summary ───────────────────────────────────────────────────
    if result.final_target_pos is not None:
        true_target = np.array([true_lpos[0], true_lpos[1], true_lpos[2] - INITIAL_HEIGHT])
        result.final_target_pos_error_mm = float(
            np.linalg.norm(result.final_target_pos - true_target))

    print()
    print("=== Results ===")
    print(f"  Catch commands issued: {result.catch_commands_issued}")
    print(f"  Target pos error: {result.final_target_pos_error_mm:.1f} mm")
    if result.landing_pos_errors_mm:
        final_err = result.landing_pos_errors_mm[-1]
        print(f"  Final landing pred error: {final_err:.1f} mm")

    return result


# ── Entry point ───────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Throw-catch cycle test")
    parser.add_argument('--viz', action='store_true', help='Show animated 3D visualization')
    parser.add_argument('--apex', type=float, default=1000.0,
                        help='Ball apex height above throw plane (mm)')
    parser.add_argument('--noise', type=float, default=2.0,
                        help='Mocap noise std dev (mm)')
    args = parser.parse_args()

    result = run_throw_catch(
        throw_pos_plat=np.array([-100.0, 0.0, 170.0]),
        catch_pos_plat=np.array([100.0, 0.0, 170.0]),
        apex_height_mm=args.apex,
        noise_std_mm=args.noise,
    )

    if args.viz:
        visualize_catch(result)


if __name__ == '__main__':
    main()
