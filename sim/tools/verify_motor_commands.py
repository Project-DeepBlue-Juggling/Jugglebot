"""Verify MPC → motor revolution commands WITHOUT sending to hardware.

After the STOW-zero refactor, the conversion is direct:
    motor_rev = IK_extension_mm × mm_to_rev

This tool computes motor_rev for various poses and compares against
JB_OP_ACTIVATE_POSITION_REVS to confirm zero step change at the Active
position (default_active_z).

Usage:
    cd Jugglebot/sim
    python tools/verify_motor_commands.py
    python tools/verify_motor_commands.py --pose 0,0,0,0,0,0
"""

from __future__ import annotations

import argparse
import os
import sys

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

import jugglebot.hardware_config as hw
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import pose_to_leg_lengths, rotvec_to_rot_matrix


def compute_motor_rev_for_pose(pose_6dof: np.ndarray,
                                geom: StewartGeometry) -> tuple[np.ndarray, np.ndarray]:
    """Compute motor revolutions for a pose using direct conversion.

    Returns (motor_rev, ik_ext_mm).
    """
    rot = rotvec_to_rot_matrix(pose_6dof[3:6])
    ik_ext = pose_to_leg_lengths(pose_6dof[:3], rot, geom)
    motor_rev = ik_ext * geom.mm_to_rev
    return motor_rev, ik_ext


def main():
    parser = argparse.ArgumentParser(description='Verify motor commands (no hardware)')
    parser.add_argument('--pose', type=str, default=None,
                        help='Target pose: x,y,z,rx,ry,rz (mm, rad). '
                             'Default: Active position [0,0,default_z,0,0,0]')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='Duration in seconds (for sweep mode)')
    args = parser.parse_args()

    geom = StewartGeometry()
    activate_rev = np.array(hw.JB_OP_ACTIVATE_POSITION_REVS)

    # Compute active extensions for reference
    active_ext_mm = activate_rev / geom.mm_to_rev

    print("=== Motor Command Verification (STOW-Zero) ===\n")
    print(f"Active position (rev):  {activate_rev}")
    print(f"Active extensions (mm): {active_ext_mm}")
    print(f"mm_to_rev:              {geom.mm_to_rev}")
    print()

    # Determine test pose
    if args.pose is not None:
        pose = np.array([float(x) for x in args.pose.split(',')])
        label = f"Custom pose {pose}"
    else:
        # Default: Active position [0, 0, default_active_z, 0, 0, 0]
        default_z = float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)
        pose = np.array([0.0, 0.0, default_z, 0.0, 0.0, 0.0])
        label = f"Active position (z={default_z}mm)"

    motor_rev, ik_ext = compute_motor_rev_for_pose(pose, geom)

    print(f"Target pose:            {pose}")
    print(f"IK extensions (mm):     {ik_ext}")
    print(f"Motor rev (computed):   {motor_rev}")
    print(f"Active position (rev):  {activate_rev}")
    print(f"Delta from Active (rev):{motor_rev - activate_rev}")
    print(f"Delta from Active (mm): {(motor_rev - activate_rev) / geom.mm_to_rev}")
    print()

    if args.pose is None:
        # At Active position, motor commands should match activate_rev exactly
        if np.allclose(motor_rev, activate_rev, atol=1e-6):
            print("PASS: At Active pose, motor commands MATCH JB_OP_ACTIVATE_POSITION_REVS")
        else:
            delta_mm = (motor_rev - activate_rev) / geom.mm_to_rev
            print(f"FAIL: MISMATCH: max delta = {np.max(np.abs(delta_mm)):.6f} mm")
    else:
        print(f"(Custom pose — no pass/fail for Active position match)")

    # Sweep test: smooth Z ramp from 0 to 50mm and back
    _run_sweep_verification(geom, activate_rev, args.duration)


def _run_sweep_verification(geom, activate_rev, duration):
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
        motor_rev, ik_ext = compute_motor_rev_for_pose(pose, geom)

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
    print(f"First sample delta: {all_delta_mm[0]} mm (STOW offset from Active)")

    # Plot
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    fig.suptitle('Motor Command Verification — Z Sweep (STOW-Zero)', fontsize=14)

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
