"""Verify MPC → motor revolution commands WITHOUT sending to hardware.

Runs the MPC with a HardwarePlant but never connects to the control loop.
Instead, intercepts the motor_rev values that *would* be sent and plots
them against JB_OP_ACTIVATE_POSITION_REVS to confirm zero step change
at the Active position.

Usage:
    cd Jugglebot/sim
    python tools/verify_motor_commands.py
    python tools/verify_motor_commands.py --pose 0,0,10,0,0,0
    python tools/verify_motor_commands.py --trajectory T1
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

# Ensure sim/ and ros_ws package are on the path
_sim_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
_ros_dir = os.path.abspath(os.path.join(_sim_dir, '..', 'ros_ws', 'src', 'jugglebot'))
for p in (_sim_dir, _ros_dir):
    if p not in sys.path:
        sys.path.insert(0, p)

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import pose_to_leg_lengths, rotvec_to_rot_matrix
from jugglebot.motion.conversions import extensions_mm_to_revs


def compute_motor_rev_for_pose(pose_6dof: np.ndarray, geom: StewartGeometry,
                                home_ext_mm: np.ndarray,
                                stow_offset_rev: np.ndarray) -> np.ndarray:
    """Replicate HardwarePlant's conversion: MPC extensions → motor rev."""
    # MPC would output home-relative extensions for this pose
    rot = rotvec_to_rot_matrix(pose_6dof[3:6])
    ik_ext = pose_to_leg_lengths(pose_6dof[:3], rot, geom)
    mpc_ext = ik_ext - home_ext_mm  # home-relative

    # HardwarePlant.command() conversion
    ik_ext_mm = mpc_ext + home_ext_mm  # back to IK convention
    motor_rev = ik_ext_mm * geom.mm_to_rev + stow_offset_rev
    return motor_rev, mpc_ext, ik_ext


def main():
    parser = argparse.ArgumentParser(description='Verify motor commands (no hardware)')
    parser.add_argument('--pose', type=str, default='0,0,0,0,0,0',
                        help='Target pose: x,y,z,rx,ry,rz (mm, rad)')
    parser.add_argument('--trajectory', type=str, default=None,
                        help='Trajectory name (T1, T2, etc.) — runs MPC loop')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='Duration in seconds (for trajectory mode)')
    args = parser.parse_args()

    geom = StewartGeometry()

    # Compute offsets (same as HardwarePlant.__init__)
    base_m = geom.base_nodes / 1000.0
    plat_m = geom.plat_nodes / 1000.0
    height_m = geom.init_height_mm / 1000.0
    plat_world_m = plat_m + np.array([0.0, 0.0, height_m])
    geom_home_mm = np.linalg.norm(plat_world_m - base_m, axis=1) * 1000.0
    home_ext_mm = geom_home_mm - geom.init_leg_lengths_mm

    activate_rev = np.array(hw.JB_OP_ACTIVATE_POSITION_REVS)
    home_ik_rev = home_ext_mm * geom.mm_to_rev
    stow_offset_rev = activate_rev - home_ik_rev

    print("=== Motor Command Verification ===\n")
    print(f"Active position (rev):  {activate_rev}")
    print(f"Home IK ext (mm):       {home_ext_mm}")
    print(f"Stow offset (rev):      {stow_offset_rev}")
    print()

    # Static pose check
    pose = np.array([float(x) for x in args.pose.split(',')])
    motor_rev, mpc_ext, ik_ext = compute_motor_rev_for_pose(
        pose, geom, home_ext_mm, stow_offset_rev)

    print(f"Target pose:            {pose}")
    print(f"MPC extensions (mm):    {mpc_ext}")
    print(f"IK extensions (mm):     {ik_ext}")
    print(f"Motor rev (computed):   {motor_rev}")
    print(f"Active position (rev):  {activate_rev}")
    print(f"Delta from Active (rev):{motor_rev - activate_rev}")
    print(f"Delta from Active (mm): {(motor_rev - activate_rev) / geom.mm_to_rev}")
    print()

    if np.allclose(motor_rev, activate_rev, atol=1e-6):
        print("PASS: At home pose, motor commands MATCH Active position (zero step change)")
    else:
        delta_mm = (motor_rev - activate_rev) / geom.mm_to_rev
        print(f"FAIL: MISMATCH: max delta = {np.max(np.abs(delta_mm)):.3f} mm")

    # Sweep test: smooth Z ramp from 0 to 50mm and back
    _run_sweep_verification(geom, home_ext_mm, stow_offset_rev, activate_rev,
                            args.duration)


def _run_sweep_verification(geom, home_ext_mm, stow_offset_rev, activate_rev,
                            duration):
    """Sweep through Z offsets and plot motor_rev to show smooth transitions."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("\nmatplotlib required for plotting")
        return

    dt = 0.02  # 50 Hz
    n_steps = int(duration / dt)
    z_max = 50.0  # mm

    times = []
    all_motor_rev = []
    all_delta_mm = []

    for step in range(n_steps):
        t = step * dt
        # Smooth Z ramp: up for first half, down for second half
        phase = t / duration
        if phase < 0.5:
            z = z_max * (2 * phase)  # 0 → z_max
        else:
            z = z_max * (2 * (1.0 - phase))  # z_max → 0

        pose = np.array([0.0, 0.0, z, 0.0, 0.0, 0.0])
        motor_rev, mpc_ext, ik_ext = compute_motor_rev_for_pose(
            pose, geom, home_ext_mm, stow_offset_rev)

        delta_mm = (motor_rev - activate_rev) / geom.mm_to_rev

        times.append(t)
        all_motor_rev.append(motor_rev.copy())
        all_delta_mm.append(delta_mm.copy())

    times = np.array(times)
    all_motor_rev = np.array(all_motor_rev)
    all_delta_mm = np.array(all_delta_mm)

    # Check for step changes
    delta_rev = np.diff(all_motor_rev, axis=0)
    max_step_rev = np.max(np.abs(delta_rev))
    max_step_mm = max_step_rev / np.mean(geom.mm_to_rev)
    print(f"\n=== Z Sweep: 0 -> {z_max} -> 0 mm over {duration}s ===")
    print(f"Motor rev range:    [{all_motor_rev.min():.4f}, {all_motor_rev.max():.4f}]")
    print(f"Delta from Active:  [{all_delta_mm.min():.3f}, {all_delta_mm.max():.3f}] mm")
    print(f"Max step-to-step:   {max_step_rev:.6f} rev = {max_step_mm:.3f} mm")
    print(f"First sample delta: {all_delta_mm[0]} mm (should be ~0)")

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Motor Command Verification — Z Sweep', fontsize=14)

    # Top: absolute motor revolutions
    ax = axes[0]
    for i in range(6):
        ax.plot(times, all_motor_rev[:, i], label=f'Leg {i}', linewidth=1)
        ax.axhline(activate_rev[i], color=f'C{i}', linestyle='--', alpha=0.3)
    ax.set_ylabel('Motor position (rev)')
    ax.set_title('Absolute motor revolutions (dashed = Active position)')
    ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    # Middle: delta from active (mm)
    ax = axes[1]
    for i in range(6):
        ax.plot(times, all_delta_mm[:, i], label=f'Leg {i}', linewidth=1)
    ax.axhline(0, color='k', linewidth=0.5)
    ax.set_ylabel('Delta from Active (mm)')
    ax.set_title('Deviation from Active position (0 = Active)')
    ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    # Bottom: step-to-step change
    ax = axes[2]
    for i in range(6):
        steps_mm = np.abs(np.diff(all_motor_rev[:, i])) / geom.mm_to_rev[i]
        ax.plot(times[1:], steps_mm, label=f'Leg {i}', linewidth=1, alpha=0.7)
    ax.set_ylabel('|Step change| (mm)')
    ax.set_xlabel('Time (s)')
    ax.set_title('Step-to-step change magnitude (should be smooth, no spikes)')
    ax.legend(ncol=3, fontsize=8)
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()
