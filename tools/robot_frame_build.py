#!/usr/bin/env python3
"""Build ``ros_ws/src/jugglebot/resources/jugglebot_robot_frame.json`` - the robot-base
marker template (``Base - 1..7`` in the ROBOT frame) that ``jugglebot.robot_frame`` fits.

The robot frame IS the kinematic-calibration QTM global frame (kincal capture
2026-09-27 14:32, ``temp/logs/kincal_sweep_20260927_143217.csv`` header
'frame: Base = QTM global'). Owner decision 2026-10-19: the markers have NOT moved, and the
~0.66 mm shape difference between the kincal bag and the 2026-10-16 bags is the QTM
reconstruction's per-wand-calibration scatter. So:

  --shape-bag      the FRESHEST bag: the template SHAPE is learned from it (live fit ~0.03 mm)
  --reference-bag  the kincal bag: the shape is REGISTERED once into the kincal frame
                   (``robot_frame.register_template``); the registered template is written
  z_axis_world_at_build = R[:, 2] of a RobotFrameTracker fed the registered template against
  the SHAPE bag's frames (the tilt-from-build cap's reference).

The registration residual (~0.66 mm) is expected and is recorded in provenance, not an error.

Needs the PDJ venv (mcap_ros2):
    ~/Desktop/PDJ_venv/venv/bin/python -I tools/robot_frame_build.py \\
        [--shape-bag B] [--reference-bag B] [--out F] [--every N] [--dry-run]
"""
from __future__ import annotations

import argparse
import datetime
import glob
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, 'ros_ws', 'src', 'jugglebot')
sys.path.insert(0, PKG)

from jugglebot import robot_frame as rf  # noqa: E402

DEFAULT_OUT = os.path.join(PKG, 'resources', 'jugglebot_robot_frame.json')
BAG_ROOT = os.path.expanduser('~/Desktop/rosbags')
DEFAULT_SHAPE = os.path.join(BAG_ROOT, '2026-10-16_22-01-50')
DEFAULT_REF = os.path.join(BAG_ROOT, '2026-09-27_14-31-13')
FRAME_SENTENCE = ("frame: the kinematic-calibration QTM global frame (kincal capture 2026-09-27 "
                  "14:32, temp/logs/kincal_sweep_20260927_143217.csv header 'frame: Base = QTM global')")


def _mcap(bag):
    d = bag if os.path.isdir(bag) else os.path.join(BAG_ROOT, bag)
    return sorted(glob.glob(os.path.join(d, '*.mcap')))[0]


def read_bag(bag, every=1, max_frames=12000):
    """([(stamp_ns, labelled rows)] every Nth /mocap_data frame, mean QTM 'Base' body pose or None)."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    frames, pos, quat, i = [], [], [], 0
    with open(_mcap(bag), 'rb') as f:
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for _s, ch, _m, msg in reader.iter_decoded_messages(topics=['/mocap_data', '/rigid_body_poses']):
            if ch.topic == '/mocap_data':
                if i >= max_frames:
                    break
                if i % every == 0:
                    rows = [(mk.label, mk.position.x, mk.position.y, mk.position.z, mk.residual)
                            for mk in msg.markers]
                    frames.append((msg.stamp.sec * 10 ** 9 + msg.stamp.nanosec, rows))
                i += 1
            else:
                for b in msg.bodies:
                    if b.name == 'Base':
                        p, q = b.pose.pose.position, b.pose.pose.orientation
                        pos.append((p.x, p.y, p.z))
                        quat.append((q.x, q.y, q.z, q.w))
    body = None
    if pos:
        q = np.mean(np.array(quat) * np.sign(np.array(quat)[:, 3:4] + 1e-12), axis=0)
        q /= np.linalg.norm(q)
        body = {'n': len(pos), 'position_mm': [float(v) for v in np.mean(pos, axis=0)],
                'quat_xyzw': [float(v) for v in q],
                'rotation_deg': float(np.degrees(2 * np.arccos(min(1.0, abs(q[3])))))}
    return frames, body


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--shape-bag', default=DEFAULT_SHAPE)
    ap.add_argument('--reference-bag', default=DEFAULT_REF)
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--every', type=int, default=1, help='use every Nth /mocap_data frame')
    ap.add_argument('--max-frames', type=int, default=12000, help='/mocap_data frames read per bag')
    ap.add_argument('--dry-run', action='store_true')
    a = ap.parse_args(argv)

    s_frames, s_body = read_bag(a.shape_bag, a.every, a.max_frames)
    r_frames, r_body = read_bag(a.reference_bag, a.every, a.max_frames)
    shape = rf.learn_robot_frame_template([r for _, r in s_frames], source=os.path.basename(a.shape_bag.rstrip('/')))
    ref = rf.learn_robot_frame_template([r for _, r in r_frames], source=os.path.basename(a.reference_bag.rstrip('/')))
    tpl, reg_rms, per = rf.register_template(shape, ref)
    print(f'shape {len(s_frames)} frames, reference {len(r_frames)} frames; '
          f'registration RMS {reg_rms:.3f} mm; per marker '
          + ', '.join(f'{k[-1]}:{v:.2f}' for k, v in per.items()))

    # z axis at build: the tracker over the SHAPE bag's frames with the registered template.
    trk = rf.RobotFrameTracker(tpl)
    for ns, rows in s_frames:
        trk.feed(rows, ns)
    cur = trk.current()
    if cur is None:
        sys.exit('no OK fit over the shape bag - cannot set z_axis_world_at_build')
    z = cur.R[:, 2]
    prov = dict(tpl.provenance)
    prov.update({
        'built': datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
                 + ' by tools/robot_frame_build.py',
        'frame': FRAME_SENTENCE,
        'shape_bag': os.path.basename(a.shape_bag.rstrip('/')),
        'reference_bag': os.path.basename(a.reference_bag.rstrip('/')),
        'every': a.every, 'max_frames': a.max_frames,
        'shape_learn': shape.provenance, 'reference_learn': ref.provenance,
        'qtm_base_body_mean_pose': {'shape_bag': s_body, 'reference_bag': r_body},
        'registration_note': ('registration RMS ~0.66 mm is EXPECTED: per-wand-calibration QTM '
                              'reconstruction scatter between the two bags (markers have not moved)'),
        'fit_over_shape_bag': {'rms_mm_median': cur.rms_mm, 'n_matched': cur.n_matched,
                               'origin_mm': [float(v) for v in cur.t]},
    })
    out = rf.RobotFrameTemplate(labels=tpl.labels, points_mm=tpl.points_mm, source=tpl.source,
                                provenance=prov, z_axis_world_at_build=z)
    print(f'z_axis_world_at_build {np.round(z, 6).tolist()}; tracker RMS {cur.rms_mm:.3f} mm')
    for lb, p in zip(out.labels, out.points_mm):
        sd = shape.provenance['per_marker'][lb]
        print(f'  {lb}: ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}) mm  shape-bag n={sd["n"]} sd={np.round(sd["sd_mm"], 3).tolist()}')
    if a.dry_run:
        return 0
    rf.write_robot_frame_template(a.out, out)
    print(f'written {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
