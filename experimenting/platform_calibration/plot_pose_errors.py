#!/usr/bin/env python3
"""
Visualise pose-tracking errors in a .mcap recording.

Usage
-----
    python plot_pose_errors.py  run_raw.mcap

Outputs
-------
* Matplotlib window with:
    • XY scatter of commanded poses coloured by position-error magnitude
    • Histograms of translational errors  (dx, dy, dz)
    • Histograms of rotational  errors  (droll, dpitch, dyaw)
* Console summary (mean, RMS, max) for every channel
"""

import argparse, sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from ament_index_python import get_package_share_directory

# ───── helper functions ────────────────────────────────────────────────────────
def quat_mult(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2])

def load_poses(bag: Path):
    """
    Return matched arrays (cmd_pos, cmd_quat, meas_pos, meas_quat).

    Pairing rule (same as KD-tree build):
        cmd  → platform moves → first subsequent settled  → keep pair
        cmd  → next cmd arrives before a settled         → drop cmd
    """
    typestore = get_typestore(Stores.ROS2_FOXY)

    # Register custom jugglebot messages once
    pkg_share = get_package_share_directory('jugglebot_interfaces')
    msg_dir = Path(pkg_share) / 'msg'
    for msg_file in msg_dir.glob('*.msg'):
        # Ignore any message with type "PlatformPoseMessage"
        if msg_file.stem == 'PlatformPoseMessage':
            continue
        typestore.register(
            get_types_from_msg(
                msg_file.read_text(encoding='utf-8'),
                f"jugglebot_interfaces/msg/{msg_file.stem}"
            )
        )

    cmd_pos, cmd_quat, meas_pos, meas_quat = [], [], [], []

    with Reader(bag) as reader:
        for con, _, raw in reader.messages():
            topic = con.topic

            if topic == '/settled_platform_poses':
                msg = typestore.deserialize_cdr(raw, con.msgtype)
                # Expecting a PoseArray message with at least two poses
                if len(msg.poses) < 2:
                    continue  # skip if not enough poses

                # First pose: command, second pose: measured
                cmd_pose = msg.poses[0]
                meas_pose = msg.poses[1]

                cmd_p = np.array([cmd_pose.position.x, cmd_pose.position.y, cmd_pose.position.z], dtype=np.float64)
                cmd_q = np.array([cmd_pose.orientation.x, cmd_pose.orientation.y,
                                  cmd_pose.orientation.z, cmd_pose.orientation.w], dtype=np.float64)
                meas_p = np.array([meas_pose.position.x, meas_pose.position.y, meas_pose.position.z], dtype=np.float64)
                meas_q = np.array([meas_pose.orientation.x, meas_pose.orientation.y,
                                   meas_pose.orientation.z, meas_pose.orientation.w], dtype=np.float64)

                cmd_pos.append(cmd_p);   cmd_quat.append(cmd_q)
                meas_pos.append(meas_p); meas_quat.append(meas_q)


    if len(cmd_pos) == 0:
        sys.exit("❌  No valid (command, settled) pairs found in this bag!")

    return (np.vstack(cmd_pos),
            np.vstack(cmd_quat),
            np.vstack(meas_pos),
            np.vstack(meas_quat))

def compute_errors(cmd_p, cmd_q, meas_p, meas_q):
    """
    Compute translational and rotational errors for arrays of poses.

    Returns a dict with:
        dp     (N,3)  translational errors [mm]
        mag_p  (N,)   translational magnitude [mm]
        rpy    (N,3)  roll-pitch-yaw errors [deg]
        mag_r  (N,)   rotational magnitude [deg]
    """
    # --- translational ------------------------------------------------------
    dp = cmd_p - meas_p
    mag_p = np.linalg.norm(dp, axis=1)

    # --- rotational ---------------------------------------------------------
    rot_cmd  = R.from_quat(cmd_q)          # vectorised
    rot_meas = R.from_quat(meas_q)
    rot_err  = rot_cmd * rot_meas.inv()    # element-wise relative rotation

    rpy = rot_err.as_euler('xyz', degrees=True)   # (N,3)  deg
    mag_r = np.linalg.norm(rpy, axis=1)           # magnitude in deg

    return dict(dp=dp, mag_p=mag_p, rpy=rpy, mag_r=mag_r)

def print_stats(label, arr):
    mean = arr.mean()
    rms  = np.sqrt((arr**2).mean())
    mx   = np.max(np.abs(arr))
    print(f"{label:<10}  mean={mean:8.3f}   rms={rms:8.3f}   max={mx:8.3f}")

def visualise(cmd_p, errs):
    """Draw a single 3-panel figure."""
    fig, axes = plt.subplots(3, 1, figsize=(10, 12))
    plt.subplots_adjust(hspace=0.32)

    # ── Panel 1: XY scatter coloured by ‖Δp‖ ────────────────────────────────
    sc = axes[0].scatter(cmd_p[:,0], cmd_p[:,1], c=errs['mag_p'],
                         cmap='viridis', s=15)
    # Set color limits so colorbar always starts at 0
    sc.set_clim(vmin=0)
    cb = plt.colorbar(sc, ax=axes[0], label='‖Δp‖  (mm)')
    axes[0].set_aspect('equal')
    axes[0].set_xlabel('Command X  [mm]')
    axes[0].set_ylabel('Command Y  [mm]')
    axes[0].set_title('Spatial distribution of position error magnitude')
    axes[0].grid(True)

    # ── Panel 2: Histograms translational ───────────────────────────────────
    axes[1].hist(errs['dp'][:,0], bins=40, alpha=0.7, label='Δx')
    axes[1].hist(errs['dp'][:,1], bins=40, alpha=0.7, label='Δy')
    axes[1].hist(errs['dp'][:,2], bins=40, alpha=0.7, label='Δz')
    axes[1].set_title('Translational errors')
    axes[1].set_xlabel('Error  [mm]')
    axes[1].set_ylabel('Count')
    axes[1].legend()
    axes[1].grid(True)

    # ── Panel 3: Histograms rotational ──────────────────────────────────────
    axes[2].hist(errs['rpy'][:,0], bins=40, alpha=0.7, label='Δroll')
    axes[2].hist(errs['rpy'][:,1], bins=40, alpha=0.7, label='Δpitch')
    axes[2].hist(errs['rpy'][:,2], bins=40, alpha=0.7, label='Δyaw')
    axes[2].set_title('Rotational errors')
    axes[2].set_xlabel('Error  [deg]')
    axes[2].set_ylabel('Count')
    axes[2].legend()
    axes[2].grid(True)

    fig.suptitle('Stewart-platform pose-tracking errors')
    plt.tight_layout(rect=[0,0,1,0.96])
    plt.show()

# ───── main ───────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Plot pose errors from an .mcap recording.')
    parser.add_argument('bag', type=Path, help='.mcap file with /platform_pose_topic & /settled_platform_pose')
    args = parser.parse_args()

    cmd_p, cmd_q, meas_p, meas_q = load_poses(args.bag)
    errs = compute_errors(cmd_p, cmd_q, meas_p, meas_q)

    # Console summary
    print("\n===  Pose-error statistics  ===")
    print_stats('Δx (mm)',   errs['dp'][:,0])
    print_stats('Δy (mm)',   errs['dp'][:,1])
    print_stats('Δz (mm)',   errs['dp'][:,2])
    print_stats('‖Δp‖ (mm)', errs['mag_p'])
    print_stats('Δroll (°)', errs['rpy'][:,0])
    print_stats('Δpitch(°)', errs['rpy'][:,1])
    print_stats('Δyaw (°)',  errs['rpy'][:,2])
    print_stats('‖Δθ‖ (°)',  errs['mag_r'])

    # Plot
    visualise(cmd_p, errs)

# ────────────────────────────────────────────────────────────────────────────
# Entry-points – both CLI *and* inline VS Code run support
# ────────────────────────────────────────────────────────────────────────────
def run_on_file(bag_path: str):
    """Convenience helper so you can call from VS Code like a normal function."""
    bag = Path(bag_path).expanduser()
    if not bag.exists():
        sys.exit(f"❌  Bag file not found: {bag}")
    cmd_p, cmd_q, meas_p, meas_q = load_poses(bag)
    errs = compute_errors(cmd_p, cmd_q, meas_p, meas_q)

    print(f"\n===  Pose-error statistics for {bag.name}  ===")
    print_stats('Δx (mm)',   errs['dp'][:,0])
    print_stats('Δy (mm)',   errs['dp'][:,1])
    print_stats('Δz (mm)',   errs['dp'][:,2])
    print_stats('‖Δp‖ (mm)', errs['mag_p'])
    print_stats('Δroll (°)', errs['rpy'][:,0])
    print_stats('Δpitch(°)', errs['rpy'][:,1])
    print_stats('Δyaw (°)',  errs['rpy'][:,2])
    print_stats('‖Δθ‖ (°)',  errs['mag_r'])

    # Print all poses for which the position error is greater than 10 mm OR orientation error is greater than 1.5 degrees
    threshold_pos = 10.0 # mm
    threshold_ori = 1.5  # degrees
    for i in range(len(errs['mag_p'])):
        if errs['mag_p'][i] > threshold_pos or errs['mag_r'][i] > threshold_ori:
            print(f"Pose {i}:")
            print(f"  Position Error:     {errs['mag_p'][i]:8.2f} mm")
            print(f"  Orientation Error:  {errs['mag_r'][i]:8.2f} degrees")
            print(f"  Commanded Position: [{cmd_p[i][0]:8.2f}, {cmd_p[i][1]:8.2f}, {cmd_p[i][2]:8.2f}] mm")
            print(f"  Commanded Orient.:  [{cmd_q[i][0]: .4f}, {cmd_q[i][1]: .4f}, {cmd_q[i][2]: .4f}, {cmd_q[i][3]: .4f}] (x, y, z, w)")
            print(f"  Measured  Position: [{meas_p[i][0]:8.2f}, {meas_p[i][1]:8.2f}, {meas_p[i][2]:8.2f}] mm")
            print(f"  Measured  Orient.:  [{meas_q[i][0]: .4f}, {meas_q[i][1]: .4f}, {meas_q[i][2]: .4f}, {meas_q[i][3]: .4f}] (x, y, z, w)")
            print("-" * 70)
    
    print(f"Total number of poses: {len(errs['mag_p'])}")
    print(f"Number of poses with position error > {threshold_pos} mm or orientation error > {threshold_ori} degrees: {len([i for i in range(len(errs['mag_p'])) if errs['mag_p'][i] > threshold_pos or errs['mag_r'][i] > threshold_ori])}")


    visualise(cmd_p, errs)

if __name__ == '__main__':
    #
    # ── Option A: run from a terminal  ────────────────────────────────────
    #     python plot_pose_errors.py  my_session.mcap
    #
    if len(sys.argv) > 1:
        main()                 # use argparse / CLI path-parsing
    else:
        #
        # ── Option B: press ▶ in VS Code  ───────────────────────────────
        #     Just edit BAG_FILE below and hit Run.
        #
        folder_dir= "/home/jetson/Desktop/Jugglebot/experimenting/platform_calibration"
        file_name = "testrun100"
        bag_file = folder_dir + "/" + file_name 
        run_on_file(bag_file)