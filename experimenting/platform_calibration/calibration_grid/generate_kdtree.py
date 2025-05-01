#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Merge one or more .mcap calibration recordings, build a KD-tree, and save
everything as a .joblib snapshot (plus optional CSV).

Usage
-----
    python gen_kdtree.py run1.mcap run2.mcap -o platform_calib
    # The later files on the command line win when poses overlap.
"""
# ────────────────────────────────────────────────────────────────────────────────
from __future__ import annotations
import argparse, csv, sys
from pathlib import Path
from typing import Dict, List, Tuple

import joblib, numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import KDTree

# ───── Tuning constants ────────────────────────────────────────────────────────
ROT_SCALE      = 100.0     # 1 rad rotation counts as 180 mm in the metric
KD_LEAF_SIZE   = 40
POS_RES_MM     = 0.5       # positional key resolution for de-duplication
ANG_RES_RAD    = np.deg2rad(1.0)  # rotational key resolution (1 deg)

# ═══════════════════════════════════════════════════════════════════════════════
# Quaternion helpers (unchanged)
# ═══════════════════════════════════════════════════════════════════════════════
def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)

def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ], dtype=np.float64)

# ═══════════════════════════════════════════════════════════════════════════════
# 1. Load & merge bags
# ═══════════════════════════════════════════════════════════════════════════════
def make_key(pos: np.ndarray, quat: np.ndarray) -> Tuple[int, ...]:
    """Quantise pose to a hashable integer key for de-duplication."""
    rv = R.from_quat(quat).as_rotvec()             # radians
    quantised = np.hstack([
        np.round(pos / POS_RES_MM),
        np.round(rv / ANG_RES_RAD),
    ]).astype(int)
    return tuple(quantised.tolist())

def load_bags(bag_paths: List[Path],
              pkg_interfaces: str = 'jugglebot_interfaces'
              ) -> Tuple[List[np.ndarray], ...]:
    """Return merged (cmd_pos, cmd_quat, meas_pos, meas_quat) lists."""
    typestore = get_typestore(Stores.ROS2_FOXY)

    # register custom msgs once
    msg_dir = Path(
        __import__('ament_index_python.packages',
                   fromlist=['get_package_share_directory'])
        .get_package_share_directory(pkg_interfaces)
    ) / 'msg'
    for m in msg_dir.glob('*.msg'):
        typestore.register(get_types_from_msg(m.read_text(), f'{pkg_interfaces}/msg/{m.stem}'))

    # Dict keyed by quantised pose; value = (cmd_p, cmd_q, meas_p, meas_q)
    merged: Dict[Tuple[int, ...], Tuple[np.ndarray, ...]] = {}

    for bag in bag_paths:
        with Reader(bag) as reader:
            cmd_p, cmd_q = None, None   # cache until we see matching measurement
            for con, _, raw in reader.messages():
                topic = con.topic
                if topic == '/platform_pose_topic':
                    msg = typestore.deserialize_cdr(raw, con.msgtype).pose_stamped.pose
                    cmd_p = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=np.float64)
                    cmd_q = np.array([msg.orientation.x, msg.orientation.y,
                                      msg.orientation.z, msg.orientation.w], dtype=np.float64)
                elif topic == '/settled_platform_pose' and cmd_p is not None:
                    msg = typestore.deserialize_cdr(raw, con.msgtype).pose
                    meas_p = np.array([msg.position.x, msg.position.y, msg.position.z], dtype=np.float64)
                    meas_q = np.array([msg.orientation.x, msg.orientation.y,
                                       msg.orientation.z, msg.orientation.w], dtype=np.float64)
                    k = make_key(cmd_p, cmd_q)
                    merged[k] = (cmd_p, cmd_q, meas_p, meas_q)  # later bags overwrite
                    cmd_p, cmd_q = None, None                  # reset until next cmd

    # unzip dict → lists
    cmd_pos, cmd_quat, meas_pos, meas_quat = [], [], [], []
    for tup in merged.values():
        cp, cq, mp, mq = tup
        cmd_pos.append(cp);   cmd_quat.append(cq)
        meas_pos.append(mp);  meas_quat.append(mq)

    print(f'✔ Merged {sum(1 for _ in merged)} unique poses from {len(bag_paths)} bags.')
    return cmd_pos, cmd_quat, meas_pos, meas_quat

# ═══════════════════════════════════════════════════════════════════════════════
# 2. As before: compute errors, build KD-tree, save
# ═══════════════════════════════════════════════════════════════════════════════
def compute_errors(cmd_pos, cmd_quat, meas_pos, meas_quat):
    N = len(cmd_pos)
    errors   = np.zeros((N, 7), dtype=np.float64)
    features = np.zeros((N, 6), dtype=np.float64)

    for i in range(N):
        errors[i, :3] = cmd_pos[i] - meas_pos[i]
        errors[i, 3:] = quat_mult(cmd_quat[i], quat_conj(meas_quat[i]))
        rotvec = R.from_quat(cmd_quat[i]).as_rotvec()
        features[i, :3] = cmd_pos[i]
        features[i, 3:] = rotvec * ROT_SCALE
    return errors, features

def build_tree(feat: np.ndarray) -> KDTree:
    return KDTree(feat, leaf_size=KD_LEAF_SIZE)

def save_outputs(prefix: Path,
                 tree,
                 features,
                 errors,
                 cmd_quat: List[np.ndarray],
                 csv_dump=True):
    """Save KDTree+features+errors (joblib) + optional readable CSV."""
    joblib.dump({'tree': tree, 'features': features, 'errors': errors},
                prefix.with_suffix('.joblib'), compress=3)
    
    if csv_dump:
        with prefix.with_suffix('.csv').open('w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['cmd_x','cmd_y','cmd_z','cmd_qx','cmd_qy','cmd_qz','cmd_qw',
                         'err_x','err_y','err_z','err_qx','err_qy','err_qz','err_qw'])
            for i in range(len(errors)):
                wr.writerow(
                    list(features[i, :3]) +            # cmd_x, cmd_y, cmd_z
                    list(cmd_quat[i]) +                # cmd_qx, cmd_qy, cmd_qz, cmd_qw
                    list(errors[i])                    # error_x, error_y, error_z, error_qx, error_qy, error_qz, error_qw
                )

# ═══════════════════════════════════════════════════════════════════════════════
# 3. CLI
# ═══════════════════════════════════════════════════════════════════════════════
def main():
    p = argparse.ArgumentParser(description='Build KD-tree from one or many .mcap bags.')
    p.add_argument('bags', type=Path, nargs='+', help='Input .mcap file(s); later ones win on overlap')
    p.add_argument('-o','--out', type=Path, default=Path('platform_calib'),
                   help='Output file prefix (default: platform_calib)')
    p.add_argument('--no-csv', action='store_true', help='Skip CSV dump')
    args = p.parse_args()

    cmd_p, cmd_q, meas_p, meas_q = load_bags(args.bags)
    errs, feats = compute_errors(cmd_p, cmd_q, meas_p, meas_q)
    tree = build_tree(feats)
    save_outputs(args.out, tree, feats, errs, cmd_q, csv_dump=not args.no_csv)

    print(f'KD-tree saved to {args.out.with_suffix(".joblib")} '
          f'({feats.shape[0]} samples, ROT_SCALE={ROT_SCALE} mm/rad)')

if __name__ == '__main__':
    main()