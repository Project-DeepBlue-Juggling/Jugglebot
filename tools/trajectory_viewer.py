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
    twist_to_leg_velocities,
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


# Cartesian DoF labels and colours
CARTESIAN_LABELS = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
CARTESIAN_COLORS = ['#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4', '#42d4f4']


# ---------------------------------------------------------------------------
# Trajectory sampling helper
# ---------------------------------------------------------------------------

def sample_trajectory(traj, geom: StewartGeometry,
                      dynamics_params: DynamicsParams,
                      n_samples: int = 200):
    """Sample a trajectory at uniform time steps and compute all plot data.

    Returns a dict with keys: t, pose, twist, accel, extensions_mm,
    leg_vel_mm_s, torque_ff_Nm.  Each value is an (n_samples, 6) array
    except t which is (n_samples,).
    """
    ts = np.linspace(traj.t_start, traj.t_start + traj.duration, n_samples)
    poses = np.empty((n_samples, 6))
    twists = np.empty((n_samples, 6))
    accels = np.empty((n_samples, 6))
    extensions = np.empty((n_samples, 6))
    leg_vels = np.empty((n_samples, 6))
    torques = np.empty((n_samples, 6))

    for i, t in enumerate(ts):
        pose, twist, accel = evaluate(traj, t)
        poses[i] = pose
        twists[i] = twist
        accels[i] = accel

        pos = pose[:3]
        rot = rotvec_to_rot_matrix(pose[3:6])
        extensions[i] = pose_to_leg_lengths(pos, rot, geom)
        leg_vels[i] = twist_to_leg_velocities(twist, pos, rot, geom)
        torques[i] = cartesian_to_motor_commands(
            pose, twist, accel, geom, dynamics_params)[2]

    return dict(
        t=ts, pose=poses, twist=twists, accel=accels,
        extensions_mm=extensions, leg_vel_mm_s=leg_vels,
        torque_ff_Nm=torques,
    )


# ---------------------------------------------------------------------------
# TrajectoryPlotPanel — 2D plots alongside the 3D viewer
# ---------------------------------------------------------------------------

class TrajectoryPlotPanel:
    """Manages a column of trajectory plots with an animated time cursor.

    Create once, then call ``load_trajectory()`` to populate data and
    ``set_cursor(t)`` each animation frame to move the vertical line.
    """

    # (row_title, data_key, y_label, use_leg_colors)
    ROW_SPECS = [
        ('Cartesian Pose',     'pose',          'mm / rad',   False),
        ('Cartesian Twist',    'twist',          'mm·s\u207b\u00b9 / rad·s\u207b\u00b9', False),
        ('Leg Extensions',     'extensions_mm',  'mm',         True),
        ('Leg Velocity',       'leg_vel_mm_s',   'mm/s',       True),
        ('Torque Feedforward', 'torque_ff_Nm',   'Nm',         True),
    ]

    def __init__(self, axes: list):
        """Initialise with a list of 5 matplotlib Axes (one per row)."""
        self.axes = axes
        self._cursor_lines = []
        self._data_lines = {}  # keyed by (row_idx, channel)
        self._setup_axes()

    def _setup_axes(self):
        for row_idx, (title, _key, ylabel, use_leg) in enumerate(self.ROW_SPECS):
            ax = self.axes[row_idx]
            ax.set_ylabel(ylabel, fontsize=7)
            ax.set_title(title, fontsize=8, pad=3)
            ax.tick_params(labelsize=6)
            ax.grid(True, alpha=0.3)

            # Create 6 data lines
            colors = LEG_COLORS if use_leg else CARTESIAN_COLORS
            labels = [f'Leg {i}' for i in range(6)] if use_leg else CARTESIAN_LABELS
            for ch in range(6):
                line, = ax.plot([], [], '-', color=colors[ch],
                                linewidth=0.8, label=labels[ch])
                self._data_lines[(row_idx, ch)] = line

            ax.legend(fontsize=5, ncol=3, loc='upper right')

            # Cursor line (vertical)
            vline = ax.axvline(0, color='#333333', linewidth=0.8,
                               linestyle='--', visible=False)
            self._cursor_lines.append(vline)

        # Only bottom axis gets an x label
        self.axes[-1].set_xlabel('Time (s)', fontsize=7)

    def load_trajectory(self, data: dict, label: str = ''):
        """Populate all plots from a sample_trajectory() result dict."""
        t = data['t']
        for row_idx, (_title, key, _ylabel, _use_leg) in enumerate(self.ROW_SPECS):
            arr = data[key]  # (n, 6)
            ax = self.axes[row_idx]
            for ch in range(6):
                line = self._data_lines[(row_idx, ch)]
                line.set_data(t, arr[:, ch])
            ax.relim()
            ax.autoscale_view()
            self._cursor_lines[row_idx].set_visible(True)

    def set_cursor(self, t: float):
        """Move the vertical cursor to time ``t``."""
        for vline in self._cursor_lines:
            vline.set_xdata([t, t])


# ---------------------------------------------------------------------------
# Standalone trajectory preview (static plots, no animation)
# ---------------------------------------------------------------------------

def preview_trajectory(traj, geom: StewartGeometry = None,
                       dynamics_params: DynamicsParams = None,
                       n_samples: int = 200, show: bool = True):
    """Plot Cartesian poses, twists, leg extensions, velocities, and torques.

    Generates a static 5-row plot summarising the entire trajectory.
    Useful for quick inspection without the 3D animation.

    Parameters
    ----------
    traj : QuinticTrajectory
    geom : StewartGeometry (auto-created if None)
    dynamics_params : DynamicsParams (auto-created if None)
    n_samples : int — number of uniform time samples
    show : bool — call plt.show() immediately
    """
    geom = geom or StewartGeometry()
    dynamics_params = dynamics_params or DynamicsParams.from_config()
    data = sample_trajectory(traj, geom, dynamics_params, n_samples=n_samples)

    fig, axes = plt.subplots(5, 1, figsize=(10, 10), sharex=True)
    fig.subplots_adjust(hspace=0.4, left=0.10, right=0.95,
                        top=0.94, bottom=0.06)
    panel = TrajectoryPlotPanel(list(axes))
    panel.load_trajectory(data)
    # Hide cursors for static plot
    for vline in panel._cursor_lines:
        vline.set_visible(False)
    fig.suptitle(f'Trajectory Preview  (dur={traj.duration:.2f}s, '
                 f'scale={traj.speed_scale:.0%})', fontsize=11)
    if show:
        plt.show()
    return fig


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
                       fps: int = 30, speed_multiplier: float = 1.0,
                       show_plots: bool = False):
    """Animate a single quintic trajectory on the Stewart platform.

    Parameters
    ----------
    show_plots : bool
        If True, display trajectory plots alongside the 3D viewer with
        an animated cursor line tracking the current time.
    """
    geom = geom or StewartGeometry()
    dynamics_params = dynamics_params or DynamicsParams.from_config()

    if show_plots:
        fig = plt.figure(figsize=(18, 9))
        gs = fig.add_gridspec(5, 2, width_ratios=[1.2, 1], wspace=0.35,
                              hspace=0.45, left=0.06, right=0.97,
                              top=0.93, bottom=0.06)
        ax3d = fig.add_subplot(gs[:, 0], projection='3d')
        plot_axes = [fig.add_subplot(gs[r, 1]) for r in range(5)]
        renderer = StewartPlatformRenderer(geom, ax=ax3d)
        panel = TrajectoryPlotPanel(plot_axes)
        data = sample_trajectory(traj, geom, dynamics_params)
        panel.load_trajectory(data)
    else:
        renderer = StewartPlatformRenderer(geom)
        panel = None

    n_frames = max(1, int(traj.duration * fps / speed_multiplier))
    interval_ms = 1000.0 / fps

    def update(frame_idx):
        t = traj.t_start + (frame_idx / n_frames) * traj.duration
        pose, twist, accel = evaluate(traj, t)
        progress = frame_idx / n_frames * 100
        renderer.update_pose(pose, label=f'Progress: {progress:.0f}%')
        if panel is not None:
            panel.set_cursor(t)
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
                          hold_s: float = 0.5,
                          show_plots: bool = False):
    """Animate a chained sequence of trajectories for pre-hardware preview.

    Parameters
    ----------
    trajs : list of (label, QuinticTrajectory) tuples.
        If None, builds the standard Phase 4 test set.
    geom : StewartGeometry (auto-created if None)
    dynamics_params : DynamicsParams (auto-created if None)
    fps : animation frame rate
    hold_s : pause between trajectories (seconds)
    show_plots : bool
        If True, display trajectory plots alongside the 3D viewer.
        Plots update to show the current trajectory's data with a
        vertical cursor tracking animation progress.
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

    # Pre-sample all trajectories for the plot panel
    if show_plots:
        sampled = []
        for _label, traj in trajs:
            sampled.append(sample_trajectory(traj, geom, dynamics_params))

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

    if show_plots:
        fig = plt.figure(figsize=(18, 9))
        gs = fig.add_gridspec(5, 2, width_ratios=[1.2, 1], wspace=0.35,
                              hspace=0.45, left=0.06, right=0.97,
                              top=0.93, bottom=0.06)
        ax3d = fig.add_subplot(gs[:, 0], projection='3d')
        plot_axes = [fig.add_subplot(gs[r, 1]) for r in range(5)]
        renderer = StewartPlatformRenderer(geom, ax=ax3d)
        panel = TrajectoryPlotPanel(plot_axes)
        # Load the first trajectory's data
        panel.load_trajectory(sampled[0], label=trajs[0][0])
    else:
        renderer = StewartPlatformRenderer(geom)
        panel = None

    interval_ms = 1000.0 / fps

    total_duration = sum(t.duration for _, t in trajs) + hold_s * len(trajs)
    renderer.fig.suptitle(
        f'Trajectory Test Sequence  '
        f'({len(trajs)} moves, {total_duration:.1f}s total)',
        fontsize=12)

    # Progress bar text
    progress_text = renderer.ax.text2D(0.02, 0.02, '', transform=renderer.ax.transAxes,
                                       fontsize=8, fontfamily='monospace')

    prev_traj_idx = [0]  # mutable for closure

    def update(frame_idx):
        traj_idx, t, label = schedule[frame_idx]
        _, traj = trajs[traj_idx]
        pose, twist, accel = evaluate(traj, t)
        renderer.update_pose(pose, label=label)
        pct = frame_idx / total_frames * 100
        progress_text.set_text(
            f'Move {traj_idx + 1}/{len(trajs)}  |  '
            f'Overall: {pct:.0f}%')
        if panel is not None:
            # Reload plot data when trajectory changes
            if traj_idx != prev_traj_idx[0]:
                panel.load_trajectory(sampled[traj_idx], label=label)
                prev_traj_idx[0] = traj_idx
            panel.set_cursor(t)
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
  python tools/trajectory_viewer.py --demo --preview  Demo with trajectory plots
  python tools/trajectory_viewer.py --test-sequence   Animate full Phase 4 tests
  python tools/trajectory_viewer.py --test-sequence --preview  With trajectory plots
  python tools/trajectory_viewer.py --phase5-sequence --speed-scale 0.5  Phase 5 tests
  python tools/trajectory_viewer.py --phase7-sequence --speed-scale 0.3  Phase 7 tests
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
    group.add_argument('--phase5-sequence', action='store_true',
                       help='Animate the Phase 5 inertia feedforward tests')
    group.add_argument('--phase7-sequence', action='store_true',
                       help='Animate the Phase 7 dynamic target tests')

    parser.add_argument('--preview', action='store_true',
                        help='Show trajectory plots alongside 3D viewer')
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
        animate_trajectory(traj, geom, fps=args.fps,
                           show_plots=args.preview)

    elif args.test_sequence:
        trajs = build_test_trajectories(geom, speed_scale=args.speed_scale)
        preview_test_sequence(trajs, geom, fps=args.fps,
                              show_plots=args.preview)

    elif args.phase5_sequence:
        from inertia_test import build_phase5_test_trajectories
        trajs = build_phase5_test_trajectories(geom,
                                                speed_scale=args.speed_scale)
        preview_test_sequence(trajs, geom, fps=args.fps,
                              show_plots=args.preview)

    elif args.phase7_sequence:
        from dynamic_target_test import build_phase7_test_trajectories
        params = DynamicsParams.from_config()
        trajs = build_phase7_test_trajectories(geom,
                                                speed_scale=args.speed_scale)
        preview_test_sequence(trajs, geom, params, fps=args.fps,
                              show_plots=args.preview)


if __name__ == '__main__':
    main()
