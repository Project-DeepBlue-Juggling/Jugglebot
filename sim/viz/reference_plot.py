"""Plot the toss-loop reference trajectory in Cartesian and joint space.

Usage (standalone):
    python sim/viz/reference_plot.py [--cycle-time 1.2] [--lateral-spacing 300] [--platform-event-speed-ratio 0.5]

Also importable:
    from sim.viz.reference_plot import plot_reference
    plot_reference(cycle_time=1.2, lateral_spacing_mm=300, ratio=0.5)
"""
from __future__ import annotations

import argparse
import sys
import os

import numpy as np

# Single path bootstrap (repo root, ros_ws pkg, config/generated);
# see sim/_paths.py.  Runnable entry scripts only — library modules under
# sim/ never touch sys.path.
_repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
if _repo_root not in sys.path:
    sys.path.insert(0, _repo_root)
from sim._paths import bootstrap_paths  # noqa: E402
bootstrap_paths()


def _rodrigues(rv: np.ndarray) -> np.ndarray:
    """Rotation vector -> 3x3 matrix (Rodrigues)."""
    angle = np.linalg.norm(rv)
    if angle < 1e-10:
        return np.eye(3)
    K = np.array([
        [0, -rv[2], rv[1]],
        [rv[2], 0, -rv[0]],
        [-rv[1], rv[0], 0],
    ])
    return (np.eye(3)
            + (np.sin(angle) / angle) * K
            + ((1 - np.cos(angle)) / (angle * angle)) * K @ K)


def _numerical_ik(pose_6dof, base_nodes, plat_nodes, init_height_vec,
                   init_leg_lengths):
    """Compute STOW-relative leg extensions for a 6-DOF pose."""
    pos = pose_6dof[:3]
    R = _rodrigues(pose_6dof[3:])
    exts = np.empty(6)
    for i in range(6):
        plat_w = pos + init_height_vec + R @ plat_nodes[i]
        leg_vec = plat_w - base_nodes[i]
        exts[i] = np.linalg.norm(leg_vec) - init_leg_lengths[i]
    return exts


def _load_geometry():
    """Load Stewart geometry from hardware_config (no MuJoCo needed)."""
    from jugglebot.motion.geometry import StewartGeometry
    geom = StewartGeometry()
    return (
        geom.base_nodes,
        geom.plat_nodes,
        np.array([0.0, 0.0, geom.init_height_mm]),
        geom.init_leg_lengths_mm,
    )


def plot_reference(
    cycle_time: float = 1.2,
    hold_ratio: float = 0.4,
    lateral_spacing_mm: float = 300.0,
    ratio: float = 0.5,
    n_samples: int = 500,
):
    """Sample and plot the toss-loop reference for 2 cycles.

    Shows 4 subplots:
      1. Cartesian position (X, Y, Z) with event markers
      2. Cartesian orientation (rx, ry, rz)
      3. Leg extensions (6 legs)
      4. Leg velocities (finite-difference)
    """
    import matplotlib.pyplot as plt
    from sim.input.toss_loop import TossLoopController, _quintic_interp

    ctrl = TossLoopController(
        cycle_time=cycle_time,
        hold_ratio=hold_ratio,
        lateral_spacing_mm=lateral_spacing_mm,
        platform_event_speed_ratio=ratio,
    )

    # Make plans for 2 cycles
    throw_time_1 = 1.0
    plan1 = ctrl._make_plan(throw_time_1, 20.0, 0.5)
    if plan1 is None:
        print("Plan 1 failed!")
        return

    ctrl._position_idx += 1
    throw_time_2 = throw_time_1 + cycle_time
    plan2 = ctrl._make_plan(throw_time_2, 20.0, 0.5)
    if plan2 is None:
        print("Plan 2 failed!")
        return

    # Timing
    air_time = ctrl._air_time
    hold_time = ctrl._hold_time
    catch_time_1 = throw_time_1 + air_time
    catch_time_2 = throw_time_2 + air_time
    zero = np.zeros(6)

    def tw(target):
        return target.arrival_twist if target.arrival_twist is not None else zero

    # Sample the quintic Hermite ASAP target (what the MPC actually sees
    # as flat reference — ref_events are disabled because their leg
    # velocities exceed the 300 mm/s MPC rate limit).
    #
    # Segment definitions: (start_time, end_time, p0, v0, p1, v1)
    segments = [
        # Approach: rest -> throw1
        (0.5, throw_time_1,
         np.zeros(6), zero,
         plan1.throw_target.pose_6dof, tw(plan1.throw_target)),
        # Flight 1: throw1 -> catch1
        (throw_time_1, catch_time_1,
         plan1.throw_target.pose_6dof, tw(plan1.throw_target),
         plan1.catch_target.pose_6dof, tw(plan1.catch_target)),
        # Hold 1: catch1 -> throw2 (end vel matches flight2 start for C2)
        (catch_time_1, throw_time_2,
         plan1.catch_target.pose_6dof, tw(plan1.catch_target),
         plan2.throw_target.pose_6dof, tw(plan2.throw_target)),
        # Flight 2: throw2 -> catch2
        (throw_time_2, catch_time_2,
         plan2.throw_target.pose_6dof, tw(plan2.throw_target),
         plan2.catch_target.pose_6dof, tw(plan2.catch_target)),
    ]

    all_t = []
    all_poses = []
    for seg_idx, (t_start, t_end, p0, v0, p1, v1) in enumerate(segments):
        duration = t_end - t_start
        n_seg = max(2, int(n_samples * duration / 3.0))
        # Exclude the first point of all segments except the first to avoid
        # duplicates at segment boundaries (which create zero-velocity blips
        # in the finite-difference leg velocity computation).
        start_idx = 1 if seg_idx > 0 else 0
        ts = np.linspace(0, duration, n_seg)
        for t in ts[start_idx:]:
            pose, _ = _quintic_interp(p0, v0, zero, p1, v1, zero, duration, t)
            all_t.append(t_start + t)
            all_poses.append(pose)

    t_arr = np.array(all_t)
    poses = np.array(all_poses)

    # Load geometry for IK
    try:
        base_nodes, plat_nodes, init_h_vec, init_lens = _load_geometry()
        has_geom = True
    except Exception as e:
        print(f"  Could not load geometry ({e}), skipping joint-space plots")
        has_geom = False

    # Compute leg extensions
    if has_geom:
        extensions = np.array([
            _numerical_ik(p, base_nodes, plat_nodes, init_h_vec, init_lens)
            for p in poses
        ])
        # Finite-difference leg velocities (guard against zero dt at segment joins)
        dt = np.diff(t_arr)
        dext = np.diff(extensions, axis=0)
        mask = dt > 1e-9
        leg_vel = np.zeros_like(dext)
        leg_vel[mask] = dext[mask] / dt[mask, None]
        t_vel = (t_arr[:-1] + t_arr[1:]) / 2.0

    # Event times for markers
    events = {
        'throw1': throw_time_1,
        'catch1': catch_time_1,
        'throw2': throw_time_2,
        'catch2': catch_time_2,
    }

    # --- Plot ---
    n_rows = 4 if has_geom else 2
    fig, axes = plt.subplots(n_rows, 1, figsize=(14, 3.5 * n_rows), sharex=True)

    # Row 1: Cartesian position
    ax = axes[0]
    ax.plot(t_arr, poses[:, 0], label='X')
    ax.plot(t_arr, poses[:, 1], label='Y')
    ax.plot(t_arr, poses[:, 2], label='Z')
    ax.set_ylabel('Position (mm)')
    ax.set_title(f'Reference Trajectory — cycle={cycle_time}s, '
                 f'lateral={lateral_spacing_mm}mm, ratio={ratio}')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Row 2: Cartesian orientation
    ax = axes[1]
    ax.plot(t_arr, np.degrees(poses[:, 3]), label='rx')
    ax.plot(t_arr, np.degrees(poses[:, 4]), label='ry')
    ax.plot(t_arr, np.degrees(poses[:, 5]), label='rz')
    ax.set_ylabel('Orientation (deg)')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    if has_geom:
        # Row 3: Leg extensions
        ax = axes[2]
        for i in range(6):
            ax.plot(t_arr, extensions[:, i], label=f'Leg {i}', alpha=0.8)
        ax.set_ylabel('Extension (mm)')
        ax.legend(loc='upper right', ncol=3, fontsize=8)
        ax.grid(True, alpha=0.3)

        # Row 4: Leg velocities
        ax = axes[3]
        for i in range(6):
            ax.plot(t_vel, leg_vel[:, i], label=f'Leg {i}', alpha=0.8)
        ax.set_ylabel('Leg velocity (mm/s)')
        ax.set_xlabel('Time (s)')
        ax.legend(loc='upper right', ncol=3, fontsize=8)
        ax.grid(True, alpha=0.3)

    # Add event markers to all axes
    colors = {'throw1': 'green', 'catch1': 'red',
              'throw2': 'green', 'catch2': 'red'}
    for ax in axes:
        for name, t_ev in events.items():
            ax.axvline(t_ev, color=colors[name], linestyle='--', alpha=0.5,
                       linewidth=1)
        # Label at top
        for name, t_ev in events.items():
            axes[0].annotate(name, xy=(t_ev, 1.0), xycoords=('data', 'axes fraction'),
                             ha='center', va='bottom', fontsize=8,
                             color=colors[name], fontweight='bold')

    plt.tight_layout()

    # Save if path given, otherwise show
    save_path = os.environ.get('REFERENCE_PLOT_SAVE')
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Saved to {save_path}")
        plt.close(fig)
    else:
        plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot toss-loop reference trajectory')
    parser.add_argument('--cycle-time', type=float, default=1.2)
    parser.add_argument('--hold-ratio', type=float, default=0.4)
    parser.add_argument('--lateral-spacing', type=float, default=300.0)
    parser.add_argument('--platform-event-speed-ratio', type=float, default=0.5)
    args = parser.parse_args()

    plot_reference(
        cycle_time=args.cycle_time,
        lateral_spacing_mm=args.lateral_spacing,
        ratio=args.platform_event_speed_ratio,
        hold_ratio=args.hold_ratio,
    )
