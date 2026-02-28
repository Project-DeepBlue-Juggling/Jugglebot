#!/usr/bin/env python3
"""3D Stewart platform trajectory viewer.

Renders a wireframe Stewart platform using matplotlib and can display:
  - Static poses
  - Animated single trajectories
  - Chained test sequences (preview before hardware execution)

Geometry is loaded from the project's hardware_config via StewartGeometry.

Usage:
  python tools/trajectory_viewer.py --home
  python tools/trajectory_viewer.py --pose 0 0 20 0 0 0
  python tools/trajectory_viewer.py --demo
  python tools/trajectory_viewer.py --test-sequence

Requirements:
  pip install numpy matplotlib
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

# ---------------------------------------------------------------------------
# Path setup (same pattern as free_platform_test.py)
# ---------------------------------------------------------------------------
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_SCRIPT_DIR)
_CONFIG_DIR = os.path.join(_PROJECT_ROOT, 'config', 'generated')
_ROS_PKG_DIR = os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'jugglebot')

sys.path.insert(0, _CONFIG_DIR)
sys.path.insert(0, _ROS_PKG_DIR)

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    rotvec_to_rot_matrix,
    pose_to_leg_lengths,
)
from jugglebot.motion.dynamics import DynamicsParams
from jugglebot.motion.workspace import compute_condition_number
from jugglebot.motion.trajectory import (
    create_trajectory,
    evaluate,
    check_feasibility,
    cartesian_to_motor_commands,
)

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Line3DCollection  # noqa: F401

# Leg colours (6 distinct colours for visual identification)
LEG_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4']


# ---------------------------------------------------------------------------
# StewartPlatformRenderer
# ---------------------------------------------------------------------------

class StewartPlatformRenderer:
    """Renders a wireframe Stewart platform in a matplotlib 3D axes."""

    def __init__(self, geom: StewartGeometry = None, ax=None):
        self.geom = geom or StewartGeometry()
        self._owns_fig = ax is None

        if ax is None:
            self.fig = plt.figure(figsize=(10, 8))
            self.ax = self.fig.add_subplot(111, projection='3d')
        else:
            self.fig = ax.get_figure()
            self.ax = ax

        self._setup_artists()
        self._set_axis_limits()

    def _setup_artists(self):
        """Create the matplotlib artists that will be updated each frame."""
        ax = self.ax
        g = self.geom

        # Base ring: closed polygon connecting base nodes (dark gray)
        base_order = list(range(6)) + [0]
        bx = g.base_nodes[base_order, 0]
        by = g.base_nodes[base_order, 1]
        bz = g.base_nodes[base_order, 2]
        self._base_line, = ax.plot(bx, by, bz, 'o-',
                                   color='#555555', linewidth=2,
                                   markersize=5, label='Base')

        # Base node labels
        for i in range(6):
            ax.text(g.base_nodes[i, 0] * 1.08,
                    g.base_nodes[i, 1] * 1.08,
                    g.base_nodes[i, 2],
                    f'B{i}', fontsize=7, color='#555555', ha='center')

        # Platform ring (updated per frame)
        self._plat_line, = ax.plot([], [], [], 'o-',
                                   color='#2060c0', linewidth=2.5,
                                   markersize=5, label='Platform')

        # Legs (updated per frame)
        self._leg_lines = []
        for i in range(6):
            line, = ax.plot([], [], [], '-',
                            color=LEG_COLORS[i], linewidth=1.5,
                            label=f'Leg {i}' if i < 3 else None)
            self._leg_lines.append(line)

        # Info text (updated per frame)
        self._info_text = ax.text2D(0.02, 0.95, '', transform=ax.transAxes,
                                    fontsize=8, fontfamily='monospace',
                                    verticalalignment='top')

        # Title text for trajectory labels
        self._title_text = ax.text2D(0.5, 1.02, '', transform=ax.transAxes,
                                     fontsize=11, fontweight='bold',
                                     ha='center', va='bottom')

        ax.legend(loc='upper right', fontsize=7, ncol=2)

    def _set_axis_limits(self):
        """Set fixed viewport limits."""
        ax = self.ax
        ax.set_xlim([-500, 500])
        ax.set_ylim([-500, 500])
        ax.set_zlim([0, 900])
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        ax.set_box_aspect([1, 1, 0.9])

    def update_pose(self, pose_6dof: np.ndarray, label: str = ''):
        """Update the platform to a new pose.

        Parameters
        ----------
        pose_6dof : (6,) array — [x, y, z, rx, ry, rz] in mm, rad
        label : str — optional title label
        """
        pose = np.asarray(pose_6dof, dtype=np.float64)
        pos = pose[:3]
        rotvec = pose[3:6]
        rot = rotvec_to_rot_matrix(rotvec)

        g = self.geom

        # Platform nodes in world frame (matches ik_solver.compute_leg_vectors)
        platform_centre = pos + g.init_height_vec
        plat_world = platform_centre + (rot @ g.plat_nodes.T).T  # (6, 3)

        # Platform ring
        order = list(range(6)) + [0]
        self._plat_line.set_data_3d(
            plat_world[order, 0],
            plat_world[order, 1],
            plat_world[order, 2])

        # Legs
        for i in range(6):
            self._leg_lines[i].set_data_3d(
                [g.base_nodes[i, 0], plat_world[i, 0]],
                [g.base_nodes[i, 1], plat_world[i, 1]],
                [g.base_nodes[i, 2], plat_world[i, 2]])

        # Info text
        extensions = pose_to_leg_lengths(pos, rot, g)
        cond = compute_condition_number(pos, rot, g)
        tilt_deg = np.degrees(np.linalg.norm(rotvec))

        info = (
            f"Pose: [{pos[0]:+.1f}, {pos[1]:+.1f}, {pos[2]:+.1f}] mm  "
            f"tilt: {tilt_deg:.1f} deg\n"
            f"Ext:  [{', '.join(f'{e:.1f}' for e in extensions)}] mm\n"
            f"Cond: {cond:.0f}"
        )
        self._info_text.set_text(info)

        if label:
            self._title_text.set_text(label)

        return (self._plat_line, *self._leg_lines, self._info_text,
                self._title_text)


# ---------------------------------------------------------------------------
# Static pose display
# ---------------------------------------------------------------------------

def show_pose(pose_6dof: np.ndarray,
              geom: StewartGeometry = None,
              title: str = None):
    """Display the Stewart platform at a single static pose."""
    renderer = StewartPlatformRenderer(geom)
    renderer.update_pose(pose_6dof, label=title or '')
    if title:
        renderer.fig.suptitle(title)
    plt.show()


# ---------------------------------------------------------------------------
# Single trajectory animation
# ---------------------------------------------------------------------------

def animate_trajectory(traj, geom: StewartGeometry = None,
                       dynamics_params: DynamicsParams = None,
                       fps: int = 30, speed_multiplier: float = 1.0):
    """Animate a single quintic trajectory on the Stewart platform."""
    geom = geom or StewartGeometry()
    renderer = StewartPlatformRenderer(geom)

    n_frames = max(1, int(traj.duration * fps / speed_multiplier))
    interval_ms = 1000.0 / fps

    def update(frame_idx):
        t = traj.t_start + (frame_idx / n_frames) * traj.duration
        pose, twist, accel = evaluate(traj, t)
        progress = frame_idx / n_frames * 100
        renderer.update_pose(pose, label=f'Progress: {progress:.0f}%')
        return ()

    anim = FuncAnimation(renderer.fig, update, frames=n_frames,
                         interval=interval_ms, blit=False, repeat=True)
    plt.show()
    return anim


# ---------------------------------------------------------------------------
# Test sequence preview
# ---------------------------------------------------------------------------

def build_test_trajectories(geom: StewartGeometry = None,
                            speed_scale: float = 0.25):
    """Build the standard Phase 4 hardware test trajectories.

    Returns a list of (label, QuinticTrajectory) tuples.
    """
    geom = geom or StewartGeometry()
    zeros6 = np.zeros(6)

    def rest_to_rest(end_pose, duration, start_pose=None):
        sp = np.asarray(start_pose, dtype=float) if start_pose is not None \
            else zeros6.copy()
        ep = np.asarray(end_pose, dtype=float)
        return create_trajectory(
            start_pose=sp, start_twist=zeros6, start_accel=zeros6,
            end_pose=ep, end_twist=zeros6, end_accel=zeros6,
            duration=duration, t_start=0.0, speed_scale=speed_scale)

    trajs = []

    # --- T1: Small move from home ---
    trajs.append(('T1: Home -> +10mm Z', rest_to_rest([0, 0, 10, 0, 0, 0], 1.0)))
    trajs.append(('T1: +10mm Z -> Home', rest_to_rest([0, 0, 0, 0, 0, 0], 1.0,
                                                        start_pose=[0, 0, 10, 0, 0, 0])))

    # --- T2: Graduated distance/tilt ---
    trajs.append(('T2a: Home -> +20mm Z',
                  rest_to_rest([0, 0, 20, 0, 0, 0], 1.0)))
    trajs.append(('T2a: +20mm Z -> Home',
                  rest_to_rest([0, 0, 0, 0, 0, 0], 1.0,
                               start_pose=[0, 0, 20, 0, 0, 0])))

    rx5 = np.deg2rad(5)
    trajs.append(('T2b: Home -> 5 deg tilt X',
                  rest_to_rest([0, 0, 0, rx5, 0, 0], 1.0)))
    trajs.append(('T2b: 5 deg tilt X -> Home',
                  rest_to_rest([0, 0, 0, 0, 0, 0], 1.0,
                               start_pose=[0, 0, 0, rx5, 0, 0])))

    trajs.append(('T2c: Home -> 20mm Z + 5 deg',
                  rest_to_rest([0, 0, 20, rx5, 0, 0], 1.5)))
    trajs.append(('T2c: 20mm Z + 5 deg -> Home',
                  rest_to_rest([0, 0, 0, 0, 0, 0], 1.5,
                               start_pose=[0, 0, 20, rx5, 0, 0])))

    trajs.append(('T2d: Home -> +40mm Z',
                  rest_to_rest([0, 0, 40, 0, 0, 0], 1.5)))
    trajs.append(('T2d: +40mm Z -> Home',
                  rest_to_rest([0, 0, 0, 0, 0, 0], 1.5,
                               start_pose=[0, 0, 40, 0, 0, 0])))

    # --- T3: Multi-pose sequence ---
    rx3 = np.deg2rad(3)
    ry2 = np.deg2rad(-2)
    sequence = [
        ('T3: Home -> 30mm Z',          [0, 0, 30, 0, 0, 0]),
        ('T3: -> 20,-10,20mm',          [20, -10, 20, 0, 0, 0]),
        ('T3: -> -15,15,10mm + 3deg X', [-15, 15, 10, rx3, 0, 0]),
        ('T3: -> 0,0,50mm + 3X -2Y',   [0, 0, 50, rx3, ry2, 0]),
        ('T3: -> Home',                 [0, 0, 0, 0, 0, 0]),
    ]
    prev_pose = zeros6.copy()
    for label, end in sequence:
        ep = np.array(end, dtype=float)
        trajs.append((label, rest_to_rest(ep, 1.5, start_pose=prev_pose)))
        prev_pose = ep.copy()

    # --- T4: Speed scale verification ---
    trajs.append(('T4: Home -> 30mm Z @ 25%',
                  rest_to_rest([0, 0, 30, 0, 0, 0], 1.0)))
    trajs.append(('T4: 30mm Z -> Home @ 25%',
                  rest_to_rest([0, 0, 0, 0, 0, 0], 1.0,
                               start_pose=[0, 0, 30, 0, 0, 0])))

    # T4 comparison at 12.5% (will be shown as a slower version)
    trajs.append(('T4: Home -> 30mm Z @ 12.5%',
                  create_trajectory(
                      start_pose=zeros6, start_twist=zeros6,
                      start_accel=zeros6,
                      end_pose=np.array([0, 0, 30, 0, 0, 0], dtype=float),
                      end_twist=zeros6, end_accel=zeros6,
                      duration=1.0, t_start=0.0, speed_scale=0.125)))
    trajs.append(('T4: 30mm Z -> Home @ 12.5%',
                  create_trajectory(
                      start_pose=np.array([0, 0, 30, 0, 0, 0], dtype=float),
                      start_twist=zeros6, start_accel=zeros6,
                      end_pose=zeros6, end_twist=zeros6, end_accel=zeros6,
                      duration=1.0, t_start=0.0, speed_scale=0.125)))

    return trajs


def preview_test_sequence(trajs: list = None,
                          geom: StewartGeometry = None,
                          dynamics_params: DynamicsParams = None,
                          fps: int = 30,
                          hold_s: float = 0.5):
    """Animate a chained sequence of trajectories for pre-hardware preview.

    Parameters
    ----------
    trajs : list of (label, QuinticTrajectory) tuples.
        If None, builds the standard Phase 4 test set.
    geom : StewartGeometry (auto-created if None)
    dynamics_params : DynamicsParams (auto-created if None)
    fps : animation frame rate
    hold_s : pause between trajectories (seconds)
    """
    geom = geom or StewartGeometry()
    dynamics_params = dynamics_params or DynamicsParams.from_config()
    if trajs is None:
        trajs = build_test_trajectories(geom)

    # Run feasibility on all trajectories first
    print(f"\nPre-flight feasibility check ({len(trajs)} trajectories):")
    all_feasible = True
    for label, traj in trajs:
        result = check_feasibility(traj, geom, dynamics_params)
        status = 'OK' if result.feasible else 'FAIL'
        peak_vel = np.max(result.peak_leg_vel_rps)
        peak_cond = result.peak_condition_number
        print(f"  [{status}] {label}: "
              f"dur={traj.duration:.2f}s  "
              f"peak_vel={peak_vel:.2f} rev/s  "
              f"cond={peak_cond:.0f}")
        if not result.feasible:
            all_feasible = False
            for v in result.violations:
                print(f"        VIOLATION: {v}")

    if not all_feasible:
        print("\nWARNING: Some trajectories are infeasible!")

    # Build frame schedule
    # Each trajectory contributes its duration in frames, plus hold_s
    schedule = []  # list of (traj_idx, t_within_traj | None for hold)
    for traj_idx, (label, traj) in enumerate(trajs):
        n_traj_frames = max(1, int(traj.duration * fps))
        for f in range(n_traj_frames):
            t = traj.t_start + (f / n_traj_frames) * traj.duration
            schedule.append((traj_idx, t, label))
        # Hold frames at end pose
        n_hold = int(hold_s * fps)
        for _ in range(n_hold):
            schedule.append((traj_idx, traj.t_start + traj.duration, label))

    total_frames = len(schedule)
    renderer = StewartPlatformRenderer(geom)
    interval_ms = 1000.0 / fps

    total_duration = sum(t.duration for _, t in trajs) + hold_s * len(trajs)
    renderer.fig.suptitle(
        f'Phase 4 Trajectory Test Sequence  '
        f'({len(trajs)} moves, {total_duration:.1f}s total)',
        fontsize=12)

    # Progress bar text
    progress_text = renderer.ax.text2D(0.02, 0.02, '', transform=renderer.ax.transAxes,
                                       fontsize=8, fontfamily='monospace')

    def update(frame_idx):
        traj_idx, t, label = schedule[frame_idx]
        _, traj = trajs[traj_idx]
        pose, twist, accel = evaluate(traj, t)
        renderer.update_pose(pose, label=label)
        pct = frame_idx / total_frames * 100
        progress_text.set_text(
            f'Move {traj_idx + 1}/{len(trajs)}  |  '
            f'Overall: {pct:.0f}%')
        return ()

    anim = FuncAnimation(renderer.fig, update, frames=total_frames,
                         interval=interval_ms, blit=False, repeat=True)
    plt.show()
    return anim


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='3D Stewart platform trajectory viewer',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python tools/trajectory_viewer.py --home            Show home pose
  python tools/trajectory_viewer.py --pose 0 0 20 0 0 0   Show offset pose
  python tools/trajectory_viewer.py --demo            Animate a 10mm Z demo
  python tools/trajectory_viewer.py --test-sequence   Animate full Phase 4 tests
        """)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--home', action='store_true',
                       help='Display the platform at home pose')
    group.add_argument('--pose', nargs=6, type=float, metavar='V',
                       help='Display pose [x y z rx ry rz] (mm, rad)')
    group.add_argument('--demo', action='store_true',
                       help='Animate a demo trajectory (10mm Z)')
    group.add_argument('--test-sequence', action='store_true',
                       help='Animate the full Phase 4 test sequence')

    parser.add_argument('--speed-scale', type=float, default=0.25,
                        help='Speed scale for trajectories (default: 0.25)')
    parser.add_argument('--fps', type=int, default=30,
                        help='Animation frame rate (default: 30)')

    args = parser.parse_args()
    geom = StewartGeometry()

    if args.home:
        show_pose(np.zeros(6), geom, title='Home Pose')

    elif args.pose:
        pose = np.array(args.pose)
        show_pose(pose, geom, title=f'Pose: {pose}')

    elif args.demo:
        zeros6 = np.zeros(6)
        traj = create_trajectory(
            start_pose=zeros6, start_twist=zeros6, start_accel=zeros6,
            end_pose=np.array([0, 0, 10, 0, 0, 0], dtype=float),
            end_twist=zeros6, end_accel=zeros6,
            duration=1.0, t_start=0.0, speed_scale=args.speed_scale)
        animate_trajectory(traj, geom, fps=args.fps)

    elif args.test_sequence:
        trajs = build_test_trajectories(geom, speed_scale=args.speed_scale)
        preview_test_sequence(trajs, geom, fps=args.fps)


if __name__ == '__main__':
    main()
