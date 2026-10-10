#!/usr/bin/env python3
"""Build or update ``ros_ws/src/jugglebot/resources/bb_base_frame.json`` — the BB
base-marker frame (as-built marker coordinates + pooled BB-in-base constants).

Two sources:

  --bag BAG.mcap      learn the as-built base template from the bag's unlabelled
                      markers (/mocap_data) and take one BB-in-base record per
                      ACCEPTED sweep the node published in it (/bb/calibration_result,
                      success; each paired with the base pose over its CALIBRATING
                      window from /bb/heartbeat). The published values are the
                      deployed gauge (the node's own numbers, not an offline replay).
  --from-state FILE   fold mocap_node's runtime pool (bb_base_frame_state.json)
                      into the resource's records (the as-built template is kept).

Needs the PDJ venv for --bag (mcap_ros2):
    ~/Desktop/PDJ_venv/venv/bin/python -I tools/bb_base_frame_build.py --bag B.mcap [--out F] [--dry-run]

See logbook/2026-10-10-bb-base-marker-frame.md and jugglebot/bb_base_frame.py.
"""
from __future__ import annotations

import argparse
import datetime
import itertools
import json
import math
import os
import re
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG = os.path.join(REPO, 'ros_ws', 'src', 'jugglebot')
sys.path.insert(0, PKG)

from jugglebot import bb_base_frame as bf          # noqa: E402
from jugglebot.bb_calibration import load_marker_template  # noqa: E402

DEFAULT_OUT = os.path.join(PKG, 'resources', 'bb_base_frame.json')
MARKER_TEMPLATE = os.path.join(PKG, 'resources', 'bb_marker_template.json')
CALIBRATING = 5     # BallButlerStates.CALIBRATING


def read_bag(path):
    """(unlabelled frames [(stamp_s, (m,3))], heartbeats [(t, state)], accepted results)."""
    from mcap_ros2.reader import read_ros2_messages
    frames, hb, res = [], [], []
    last = None
    for rm in read_ros2_messages(path, topics=['/mocap_data', '/bb/heartbeat',
                                               '/bb/calibration_result']):
        t = rm.log_time_ns * 1e-9
        m = rm.ros_msg
        topic = rm.channel.topic
        if topic == '/mocap_data':
            ns = m.stamp.sec * 1_000_000_000 + m.stamp.nanosec
            if ns <= 0 or ns == last:
                continue
            last = ns
            U = np.array([(k.position.x, k.position.y, k.position.z)
                          for k in m.markers if not k.label]).reshape(-1, 3)
            frames.append((ns * 1e-9, U))
        elif topic == '/bb/heartbeat':
            hb.append((t, int(m.state)))
        elif m.success:
            stamp = re.search(r'accepted (\S+?)\)', m.message)
            res.append(dict(t=t, yaw_deg=math.degrees(m.yaw_offset_rad),
                            sigma_deg=float(m.yaw_offset_std_deg),
                            pos=np.array([m.position_mm.x, m.position_mm.y, m.position_mm.z]),
                            accepted_at=stamp.group(1) if stamp else ''))
    return frames, hb, res


def windows(hb):
    """[(t_start, t_end)] of the CALIBRATING runs (mocap_node's start/end edges)."""
    out, start, prev = [], None, None
    for t, s in hb:
        if s == CALIBRATING and start is None:
            start = t
        if prev == CALIBRATING and s != CALIBRATING and start is not None:
            out.append((start, t))
            start = None
        prev = s
    return out


def build_from_bag(bag, pin_deg, repeat_deg, pin_sd):
    frames, hb, results = read_bag(bag)
    print(f'{len(frames)} distinct mocap frames, {len(hb)} heartbeats, {len(results)} accepted results')
    nominal = bf.nominal_base_template()
    track = bf.track_base_frame(frames, nominal)
    T = bf.learn_as_built(track)
    track = bf.track_base_frame(frames, T)       # re-pose with the as-built template
    whole = bf.estimate_base_pose(None, T, track=track)
    print(whole.summary())
    wins = windows(hb)
    recs = []
    for r in results:
        # The window this result closed: the last one that ended before it.
        w = [w for w in wins if w[1] <= r['t']]
        if not w:
            print(f'  result at {r["t"]:.1f} has no window — skipped')
            continue
        t0, t1 = w[-1]
        sel = (track.t >= t0 - 0.5) & (track.t <= t1 + 0.5)
        sub = bf.BaseFrameTrack(t=track.t[sel], R=track.R[sel], origin=track.origin[sel],
                                rms_mm=track.rms_mm[sel], n_matched=track.n_matched[sel],
                                points=track.points[sel], n_frames_in=int(sel.sum()))
        base = bf.estimate_base_pose(None, T, track=sub)
        kappa, p_b = bf.bb_in_base(base, r['yaw_deg'], r['pos'])
        recs.append(bf.KappaRecord(kappa_deg=kappa, sigma_deg=r['sigma_deg'],
                                   axis_point_b_mm=tuple(float(v) for v in p_b), pin_deg=pin_deg,
                                   accepted_at=r['accepted_at'], yaw_offset_deg=r['yaw_deg'],
                                   base_heading_deg=base.heading_deg,
                                   source=f'{os.path.basename(bag)} sweep {len(recs) + 1}'))
        print(f'  sweep {len(recs)}: φ {r["yaw_deg"]:+.4f}° h {base.heading_deg:.4f}° '
              f'κ {kappa:+.4f}° p_b ({p_b[0]:.3f}, {p_b[1]:.3f}, {p_b[2]:.3f})')
    full = np.all(np.isfinite(track.points[:, :, 0]), axis=1)
    loc = []
    for Q in track.points[full]:
        R, o, _ = bf.kabsch(T, Q)
        loc.append((Q - o) @ R)
    sd = np.std(loc, axis=0)
    D = np.linalg.norm(track.points[full][:, :, None] - track.points[full][:, None], axis=3)
    dN = np.linalg.norm(nominal[:, None] - nominal[None], axis=2)
    pairs = {}
    for a, b in itertools.combinations(range(4), 2):
        k = bf.BASE_MARKER_NAMES[a] + bf.BASE_MARKER_NAMES[b]
        pairs[k] = {'as_built_mm': round(float(D[:, a, b].mean()), 3),
                    'nominal_mm': round(float(dN[a, b]), 2)}
    as_built = {
        'markers': [{'id': n, 'xyz_mm': [round(float(v), 4) for v in T[i]],
                     'frame_sd_mm': [round(float(v), 4) for v in sd[i]]}
                    for i, n in enumerate(bf.BASE_MARKER_NAMES)],
        'pair_distances_mm': pairs,
        'plane_tilt_deg': round(whole.tilt_deg, 3),
        'plane_normal_world': [round(float(v), 5) for v in whole.R[:, 2]],
        'built_from': f'{os.path.basename(bag)}: generalised Procrustes over {int(full.sum())} '
                      f'frames with all four markers (of {track.n_frames_in})',
    }
    return as_built, recs, whole


def resource_json(as_built, recs, pin_deg, repeat_deg, pin_sd, provenance):
    pooled = bf.pool_kappa(recs, pin_deg, repeat_deg)
    return {
        'schema': 1,
        'description': ('Ball Butler base-marker frame: four coplanar markers fastened to BB\'s '
                        'shelf (no QTM label; identified by geometry), their as-built '
                        'coordinates, and BB\'s pose in that frame pooled over accepted sweeps. '
                        'mocap_node composes this sitting\'s base pose with these constants '
                        '(bb_pose_source base_frame / auto). jugglebot/bb_base_frame.py.'),
        'frame': ('Base frame, mm: origin C (the L\'s corner); z the marker plane\'s normal, '
                  'pointing up; x along C->E with its normal component removed; y = z x x '
                  '(S on +y). Heading = angle of the posed x axis projected on world xy.'),
        'nominal': {'cs_mm': bf.NOMINAL_CS_MM, 'cm_mm': bf.NOMINAL_CM_MM,
                    'ce_mm': bf.NOMINAL_CE_MM,
                    'note': 'owner\'s nominal (3D-printed); identification tolerance '
                            f'{bf.BASE_MATCH_TOL_MM} mm per pair distance'},
        'as_built': as_built,
        'bb_in_base': {
            'kappa_deg': None if pooled is None else round(pooled.kappa_deg, 5),
            'kappa_sd_deg': None if pooled is None else round(pooled.sd_deg, 5),
            'kappa_se_deg': None if pooled is None else round(pooled.se_deg, 5),
            'n_sweeps': 0 if pooled is None else pooled.n,
            'axis_point_mm': None if pooled is None else [round(float(v), 4) for v in pooled.axis_point_b_mm],
            'axis_point_sd_mm': None if pooled is None else [round(float(v), 4) for v in pooled.axis_point_sd_mm],
            'meaning': ('kappa = published yaw offset (pinned gauge, about world z) - base heading; '
                        'axis_point = BB\'s axis point in the base frame. Summary only: mocap_node '
                        'pools the records (each moved to the current pin).'),
            'records': [r.to_dict() for r in recs],
        },
        'gauge': {
            'pinned_yaw_offset_deg_at_build': pin_deg,
            'pin_uncertainty_deg': pin_sd,
            'note': ('kappa is measured from the PUBLISHED (pinned) sweep offsets, so at the '
                     'build sitting the base-derived offset equals the constellation\'s: same '
                     'gauge, the frame throw_affine_correction.json was fitted in. Each record '
                     'carries the pin it was measured under and pooling moves it to the marker '
                     'template\'s CURRENT gauge.pinned_yaw_offset_deg, so a landing-derived '
                     're-pin (BallButler settle_yaw_gauge.py: add delta to the pin) moves kappa '
                     'by the same delta with no re-sweep. The pin\'s own uncertainty is common '
                     'to every sitting and is not in kappa_se_deg.'),
        },
        'provenance': provenance,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--bag')
    src.add_argument('--from-state')
    ap.add_argument('--out', default=DEFAULT_OUT)
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--note', default='', help='free text added to provenance.note')
    a = ap.parse_args(argv)
    tpl = load_marker_template(MARKER_TEMPLATE)
    pin, rep, pin_sd = tpl.pinned_yaw_offset_deg, tpl.repeatability_deg, tpl.pin_uncertainty_deg
    now = datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ')
    if a.bag:
        as_built, recs, whole = build_from_bag(a.bag, pin, rep, pin_sd)
        prov = {'built': f'{now} by tools/bb_base_frame_build.py --bag {os.path.basename(a.bag)}',
                'base_pose_at_build': whole.to_dict()}
    else:
        with open(a.out) as f:
            old = json.load(f)
        state = bf.load_base_state(a.from_state)
        if state is None:
            sys.exit(f'{a.from_state} does not exist')
        as_built = old['as_built']
        recs = state['records']
        prov = dict(old.get('provenance', {}))
        prov['updated'] = (f'{now} by tools/bb_base_frame_build.py --from-state '
                           f'{os.path.basename(a.from_state)} ({len(recs)} records)')
    if a.note:
        prov['note'] = a.note
    out = resource_json(as_built, recs, pin, rep, pin_sd, prov)
    b = out['bb_in_base']
    print(f'kappa {b["kappa_deg"]}° SD {b["kappa_sd_deg"]} SE {b["kappa_se_deg"]} n {b["n_sweeps"]}; '
          f'axis point {b["axis_point_mm"]} mm (SD {b["axis_point_sd_mm"]})')
    if a.dry_run:
        return 0
    with open(a.out, 'w') as f:
        json.dump(out, f, indent=2)
        f.write('\n')
    print(f'written {a.out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
