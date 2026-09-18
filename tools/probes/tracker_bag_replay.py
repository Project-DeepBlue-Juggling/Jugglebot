#!/usr/bin/env python3
"""Replay a rosbag's `/mocap_data` + `/throw_announcements` through the REAL tracker.

Why this exists
---------------
The 2026-09-15 sitting threw 13 balls, the operator watched every one of them
fly, and the tracker CONFIRMED none: `ball_tracker_node` forwarded only markers
whose mocap label was EMPTY, and QTM's AIM model had claimed the flying ball as
`Ball Butler - 1`. That defect was invisible to every offline test in the repo,
because every one of them feeds synthetic *unlabelled* markers — the tests and
the node agreed with each other and both disagreed with the mocap system.

This probe closes that gap by replaying the bag's own frames, labels included,
through the SAME `BallTracker` the node builds from the SAME generated config,
frame by frame in bag order, using each frame's own bag timestamp as
`current_time`. Nothing here re-implements matching, gating or prediction: if
this probe says a ball confirms, the node confirms it too.

What it reports, per announced ball
-----------------------------------
- `t_conf`   first CONFIRMED time, relative to the announcement's `throw_time`
- `t_land`   first time a landing estimate exists while CONFIRMED, same datum
- `land_err` the tracker's FIRST landing_time minus the announcement's. Near
  zero by construction — at `t_conf` the filter is still essentially the
  announcement's own seed state — so it is a sanity check, not a measurement.
- `dl_*`     the landing estimate as it stands at the CATCH executor's own
  deadline, `announced_landing - CATCH_DEADLINE_WINDOW_S - lead_s`, and the
  only column that says whether the tracker has overtaken the announcement's
  (on 2026-09-15, ~25 % fast) prior. **Read it as an UPPER bound on what the
  tracker knows when a catch is aimed (2026-09-18):** the aim is now ordered
  (`executor._catch_aim`, converged fit → schedule prior → unfitted landing),
  so a catch WITH a schedule prior is aimed at its own SCHEDULED dispatch
  instant — earlier than this deadline — and then re-aimed from later fits
  until the freeze; only a catch with no prior in the schedule still waits
  this long. A `dl_*` that has not converged therefore means the DISPATCH
  runs on the prior, not that the catch never sees the fit.
- `pos_err`  |tracker landing_position - announced landing_position| (mm, xy)
- `status`   the ball's final `BallStatus`

Pass criterion (2026-09-15, R3 apex ladder): every announced ball CONFIRMED
within 0.25 s of release WITH a landing estimate. The catch executor's deadline
is `landing_time - 0.278 s - lead_s`, i.e. ~0.47-0.49 s before landing at the R3
operating point, so a confirmation later than that is a `NO_LANDING` in the air.

Landing-estimate accuracy (`--validate-landing`, added 2026-09-17)
--------------------------------------------------------------------
Independently of the CONFIRM/landing-existence check above, this mode scores
HOW ACCURATE the tracker's landing_time is, against an offline gravity-fixed
parabola fit of the raw `/mocap_data` ball marker (same method as
`late_catch_probe.py` / `flight_truth_probe.py`: track the marker nearest the
announced ballistic path frame-to-frame, then a two-pass fit over samples
z > 980 mm with a 12 mm residual gate — a code path independent of
`tracking/flight_fit.py`, so this is a real check, not a circular one). For
each announced ball it reports two errors (tracker estimate minus truth
crossing at `landing_z`):
  - `last_err_ms`  the LAST in-flight landing estimate (closest the tracker
    ever gets before the ball leaves IN_FLIGHT)
  - `dl_err_ms`    the estimate at the catch executor's own dispatch instant
    (see `dl_*` above)
Pass criterion (this brief, 2026-09-17): |bias| <= 10 ms and sd <= 15 ms at
the last in-flight sample; |bias| <= 20 ms at dispatch. Writes
`temp/probes/tracker_fit_validation_<tag>.md` per bag when given `--out-md`,
or aggregates several bags into one report — see `--validate-landing --help`
usage below.

Usage
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    source /opt/ros/foxy/setup.bash
    python tools/probes/tracker_bag_replay.py ~/Desktop/rosbags/2026-09-15_18-51-37

    # tighten/loosen the criterion, or point at one throw
    python tools/probes/tracker_bag_replay.py <bag> --confirm-deadline-s 0.25

    # landing-estimate accuracy across one or more bags -> one aggregated .md
    python tools/probes/tracker_bag_replay.py --validate-landing \\
        ~/Desktop/rosbags/2026-09-17_18-45-10 ~/Desktop/rosbags/2026-09-17_18-50-45 \\
        --out-md temp/probes/tracker_fit_validation_20260917.md

Writes `temp/probes/tracker_bag_replay_<bagname>.csv` plus a `.md` table.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import math
import os
import sys
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO / 'ros_ws' / 'src' / 'jugglebot'))

import jugglebot.hardware_config as hw                     # noqa: E402
from jugglebot.motion.skills import sites                  # noqa: E402
from jugglebot.tracking.ball import BallStatus, TrackingConfidence  # noqa: E402
from jugglebot.tracking.ballistics import GRAVITY_MMPS2     # noqa: E402
from jugglebot.tracking.matcher import (                    # noqa: E402
    BallTracker, parse_label_prefixes)


def make_tracker() -> BallTracker:
    """A BallTracker wired EXACTLY as `ball_tracker_node.BallTrackerNode` wires it.

    Any divergence here makes the whole probe a lie, so this stays a
    line-for-line mirror of that constructor call.
    """
    return BallTracker(
        dt=hw.TRACKING_MOCAP_DT_S,
        landing_z=float(sites.CATCH_CUP_Z_MM),
        match_threshold_base_mm=hw.TRACKING_MATCH_THRESHOLD_BASE_MM,
        parabolic_min_frames=hw.TRACKING_MIN_MATCHES_TO_CONFIRM,
        missed_frames_to_lose=10,
        max_frames_without_measurement=hw.TRACKING_MAX_FRAMES_WITHOUT_MEASUREMENT,
        process_noise=hw.TRACKING_PROCESS_NOISE,
        measurement_noise=hw.TRACKING_MEASUREMENT_NOISE,
        announced_gate_mm=hw.TRACKING_ANNOUNCED_GATE_MM,
        excluded_label_prefixes=parse_label_prefixes(
            hw.TRACKING_EXCLUDED_LABEL_PREFIXES),
        detect_human_throws=hw.TRACKING_DETECT_HUMAN_THROWS,
        flight_fit_min_samples=hw.TRACKING_FLIGHT_FIT_MIN_SAMPLES,
        flight_fit_residual_mm=hw.TRACKING_FLIGHT_FIT_RESIDUAL_MM,
        flight_fit_freeze_above_plane_mm=hw.TRACKING_FLIGHT_FIT_FREEZE_ABOVE_PLANE_MM,
    )


def _find_mcap(bag: Path) -> Path:
    if bag.is_file():
        return bag
    cands = sorted(bag.glob('*.mcap'))
    if not cands:
        raise SystemExit(f"no .mcap under {bag}")
    return cands[0]


def _iter_bag(mcap_path: Path, topics):
    """Yield (topic, log_time_s, decoded_msg) in bag order."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    with open(mcap_path, 'rb') as fh:
        reader = make_reader(fh, decoder_factories=[DecoderFactory()])
        for _schema, channel, message, decoded in reader.iter_decoded_messages(
                topics=list(topics)):
            yield channel.topic, message.log_time * 1e-9, decoded


def _stamp(t) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


class _Record:
    """Per-announced-ball outcome accumulator."""

    def __init__(self, ball_id, throw_time, ann_landing_time, ann_landing_pos, source,
                 init_pos=None, init_vel=None):
        self.ball_id = ball_id
        self.throw_time = throw_time
        self.ann_landing_time = ann_landing_time
        self.ann_landing_pos = ann_landing_pos
        self.source = source
        self.init_pos = init_pos          # announced initial position (truth-fit gate)
        self.init_vel = init_vel          # announced initial velocity (truth-fit gate)
        self.t_confirmed = None
        self.t_landing_est = None
        self.landing_time_est = None
        self.landing_pos_est = None
        self.final_status = None
        self.frames_tracked = 0
        # State as of the CATCH executor's deadline
        self.dl_landing_time = None
        self.dl_landing_pos = None
        self.dl_tracking = None
        # LAST landing estimate seen while the ball was still IN_FLIGHT —
        # the closest-to-truth number the tracker ever publishes, used by
        # `--validate-landing` (2026-09-17).
        self.last_landing_time = None
        self.last_landing_pos = None


# `executor.CATCH_DEADLINE_WINDOW_S` (the R2/R3 transit window) plus the
# measured per-skill dispatch lead on the 2026-09-15 sitting (0.196-0.211 s).
CATCH_DEADLINE_WINDOW_S = 0.278
CATCH_LEAD_S = 0.200


def replay(mcap_path: Path, confirm_deadline_s: float, buffer_raw: bool = False):
    """Replay one bag through the real tracker.

    `buffer_raw`: also buffer every `/mocap_data` frame's (label, x, y, z)
    tuples, returned as a fourth value `mocap_frames` — used by
    `--validate-landing` for an INDEPENDENT ground-truth fit. Off by default
    (it roughly doubles memory use on a long bag) since the CONFIRM/landing
    existence report doesn't need it.
    """
    tracker = make_tracker()
    records = {}          # tracker ball id -> _Record
    order = []
    n_frames = 0
    n_markers = 0
    n_eligible = 0
    mocap_frames = [] if buffer_raw else None

    for topic, t_bag, msg in _iter_bag(
            mcap_path, ('/mocap_data', '/throw_announcements')):
        if topic == '/throw_announcements':
            throw_time = _stamp(msg.throw_time)
            landing_time = _stamp(msg.landing_time)
            init_pos = np.array([msg.initial_position.x,
                                 msg.initial_position.y,
                                 msg.initial_position.z])
            init_vel = np.array([msg.initial_velocity.x,
                                 msg.initial_velocity.y,
                                 msg.initial_velocity.z])
            land_pos = np.array([msg.landing_position.x,
                                 msg.landing_position.y,
                                 msg.landing_position.z])
            land_vel = np.array([msg.landing_velocity.x,
                                 msg.landing_velocity.y,
                                 msg.landing_velocity.z])
            # The node substitutes wall clock for a zero throw_time; the bag's
            # own log time is this replay's wall clock.
            if throw_time < 1.0:
                throw_time = t_bag
            bid = tracker.handle_announcement(
                initial_position=init_pos,
                initial_velocity=init_vel,
                throw_time=throw_time,
                source=msg.thrower_name or 'ball_butler',
                destination=msg.target_id or '',
                landing_position=land_pos if landing_time > 0 else None,
                landing_velocity=land_vel if landing_time > 0 else None,
                landing_time=landing_time if landing_time > 0 else None,
            )
            rec = _Record(bid, throw_time, landing_time, land_pos,
                          msg.thrower_name or 'ball_butler',
                          init_pos=init_pos, init_vel=init_vel)
            records[bid] = rec
            order.append(bid)
            continue

        # /mocap_data — every marker, with its label, exactly as the node now does
        n_frames += 1
        positions = []
        labels = []
        for mk in msg.markers:
            positions.append(np.array([mk.position.x, mk.position.y, mk.position.z]))
            labels.append(mk.label or '')
        n_markers += len(positions)
        n_eligible += len(tracker.eligible_markers(positions, labels))
        if buffer_raw:
            mocap_frames.append((t_bag, [
                (labels[i], p[0], p[1], p[2]) for i, p in enumerate(positions)]))

        balls = tracker.process_frame(positions, t_bag, labels)

        for ball in balls:
            rec = records.get(ball.id)
            if rec is None:
                continue
            if rec.t_confirmed is None and ball.tracking == TrackingConfidence.CONFIRMED:
                rec.t_confirmed = t_bag - rec.throw_time
            if (rec.t_confirmed is not None and rec.t_landing_est is None
                    and ball.landing_time and ball.landing_time > 0
                    and ball.frames_tracked >= 1):
                rec.t_landing_est = t_bag - rec.throw_time
                rec.landing_time_est = float(ball.landing_time)
                rec.landing_pos_est = np.array(ball.landing_position, dtype=float)
            if (rec.ann_landing_time
                    and t_bag <= rec.ann_landing_time - CATCH_DEADLINE_WINDOW_S
                                                      - CATCH_LEAD_S):
                # Latest sample at or before the executor's deadline
                rec.dl_tracking = ball.tracking
                if ball.landing_time and ball.landing_time > 0:
                    rec.dl_landing_time = float(ball.landing_time)
                    rec.dl_landing_pos = np.array(ball.landing_position, dtype=float)
            if ball.status == BallStatus.IN_FLIGHT and ball.landing_time and ball.landing_time > 0:
                # Overwritten every frame — the final value is the LAST
                # in-flight sample's landing estimate (`--validate-landing`).
                rec.last_landing_time = float(ball.landing_time)
                rec.last_landing_pos = np.array(ball.landing_position, dtype=float)
            rec.frames_tracked = max(rec.frames_tracked, int(ball.frames_tracked))
            rec.final_status = ball.status

    rows = []
    for bid in order:
        r = records[bid]
        land_err = (r.landing_time_est - r.ann_landing_time
                    if (r.landing_time_est and r.ann_landing_time) else None)
        pos_err = (float(np.linalg.norm(r.landing_pos_est[:2] - r.ann_landing_pos[:2]))
                   if r.landing_pos_est is not None else None)
        dl_err = (r.dl_landing_time - r.ann_landing_time
                  if (r.dl_landing_time and r.ann_landing_time) else None)
        dl_pos = (float(np.linalg.norm(r.dl_landing_pos[:2] - r.ann_landing_pos[:2]))
                  if r.dl_landing_pos is not None else None)
        ok = (r.t_confirmed is not None
              and r.t_confirmed <= confirm_deadline_s
              and r.t_landing_est is not None)
        rows.append(dict(
            ball_id=bid,
            source=r.source,
            throw_time=f'{r.throw_time:.3f}',
            t_conf_s=(f'{r.t_confirmed:.3f}' if r.t_confirmed is not None else ''),
            t_land_est_s=(f'{r.t_landing_est:.3f}' if r.t_landing_est is not None else ''),
            land_time_err_s=(f'{land_err:+.3f}' if land_err is not None else ''),
            land_xy_err_mm=(f'{pos_err:.0f}' if pos_err is not None else ''),
            dl_conf=(r.dl_tracking.name if r.dl_tracking is not None else ''),
            dl_land_err_s=(f'{dl_err:+.3f}' if dl_err is not None else ''),
            dl_land_xy_err_mm=(f'{dl_pos:.0f}' if dl_pos is not None else ''),
            frames_tracked=r.frames_tracked,
            final_status=(r.final_status.name if r.final_status is not None else ''),
            pass_=('PASS' if ok else 'FAIL'),
        ))
    stats = dict(frames=n_frames, markers=n_markers, eligible=n_eligible,
                 announced=len(order))
    return rows, stats, records, order, mocap_frames


def _truth_crossing_for_ball(mocap_frames, mocap_times, throw_time, pos0, vel0,
                              landing_z):
    """Independent ground-truth landing crossing for one throw.

    Same method as `late_catch_probe.py` / `flight_truth_probe.py`: track the
    marker nearest the announced ballistic path frame-to-frame (continuity
    gated once a track is established), then a two-pass gravity-fixed
    parabola fit over samples z > 980 mm with a 12 mm residual gate. This is
    a code path INDEPENDENT of `tracking/flight_fit.py` — it does not import
    it or reuse any of its logic — so a bag that validates here validates the
    fit for real, not circularly.

    Returns (t_cross_abs, rms_mm, n) or (None, None, None).
    """
    x0, y0, z0v = float(pos0[0]), float(pos0[1]), float(pos0[2])
    vz0 = float(vel0[2])
    i0 = bisect.bisect_left(mocap_times, throw_time - 0.05)
    i1 = bisect.bisect_left(mocap_times, throw_time + 1.6)
    pts = []
    last = None
    for k in range(i0, i1):
        t, frame = mocap_frames[k]
        best = None
        for lab, x, y, z in frame:
            if lab.startswith('Platform') or lab.startswith('Base'):
                continue
            if last is None:
                dtau = t - throw_time
                zp = (z0v + vz0 * dtau - 0.5 * GRAVITY_MMPS2 * dtau * dtau
                      if dtau > 0 else z0v)
                ok = (abs(x - x0) < 150.0 and abs(y - y0) < 150.0
                      and abs(z - zp) < 250.0 and t < throw_time + 0.45)
            else:
                dt = t - last[0]
                ok = (math.hypot(x - last[1], y - last[2]) < 60 + 1500 * dt
                      and abs(z - last[3]) < 60 + 4500 * dt)
            if ok and (best is None or z > best[3]):
                best = (t, x, y, z)
        if best is None:
            continue
        last = best
        pts.append(best)

    if len(pts) < 20:
        return None, None, None

    T = np.array([p[0] for p in pts])
    Z = np.array([p[3] for p in pts])
    sel = Z > 980.0
    z0f = v0f = res = None
    for _ in range(4):
        if sel.sum() < 10:
            return None, None, None
        tau = T[sel] - throw_time
        zz = Z[sel] + 0.5 * GRAVITY_MMPS2 * tau * tau
        A = np.vstack([np.ones_like(tau), tau]).T
        (z0f, v0f), *_ = np.linalg.lstsq(A, zz, rcond=None)
        res = Z - (z0f + v0f * (T - throw_time)
                   - 0.5 * GRAVITY_MMPS2 * (T - throw_time) ** 2)
        sel = (Z > 980.0) & (np.abs(res) < 12.0)
    if sel.sum() < 10:
        return None, None, None

    disc = v0f * v0f + 2 * GRAVITY_MMPS2 * (z0f - landing_z)
    if disc < 0:
        return None, None, None
    t_cross_rel = (v0f + math.sqrt(disc)) / GRAVITY_MMPS2
    rms = float(np.sqrt(np.mean(res[sel] ** 2)))
    return throw_time + t_cross_rel, rms, int(sel.sum())


def validate_landing(mcap_path: Path, landing_z: float):
    """Score landing-estimate ACCURACY for one bag against the offline truth fit.

    Returns a list of per-ball dicts (last_err_ms, dl_err_ms, truth_n,
    truth_rms_mm, ...) — see module docstring `--validate-landing`.
    """
    _, _, records, order, mocap_frames = replay(
        mcap_path, confirm_deadline_s=0.25, buffer_raw=True)
    mocap_times = [f[0] for f in mocap_frames]

    out = []
    for bid in order:
        r = records[bid]
        if r.init_pos is None or r.init_vel is None:
            continue
        t_truth, rms, n_truth = _truth_crossing_for_ball(
            mocap_frames, mocap_times, r.throw_time, r.init_pos, r.init_vel,
            landing_z)
        last_err_ms = (None if (t_truth is None or r.last_landing_time is None)
                       else (r.last_landing_time - t_truth) * 1000.0)
        dl_err_ms = (None if (t_truth is None or r.dl_landing_time is None)
                     else (r.dl_landing_time - t_truth) * 1000.0)
        out.append(dict(
            bag=mcap_path.stem,
            ball_id=bid,
            source=r.source,
            truth_ok=(t_truth is not None),
            truth_n=n_truth,
            truth_rms_mm=rms,
            last_err_ms=last_err_ms,
            dl_err_ms=dl_err_ms,
            final_status=(r.final_status.name if r.final_status is not None else ''),
        ))
    return out


def _validate_landing_main(args):
    """`--validate-landing`: aggregate one or more bags into one report."""
    landing_z = float(sites.CATCH_CUP_Z_MM)
    out_md = Path(args.out_md) if args.out_md else (
        _REPO / 'temp' / 'probes' / 'tracker_fit_validation.md')
    out_md.parent.mkdir(parents=True, exist_ok=True)

    all_rows = []
    per_bag_lines = []
    bag_names = []
    for bag_arg in args.bags:
        mcap_path = _find_mcap(Path(os.path.expanduser(bag_arg)))
        bag_names.append(mcap_path.stem)
        rows = validate_landing(mcap_path, landing_z)
        all_rows.extend(rows)
        n = sum(1 for r in rows if r['truth_ok'])
        per_bag_lines.append(f"- `{mcap_path.stem}`: {len(rows)} announced, "
                              f"{n} with a truth fit")
        print(f"{mcap_path.stem}: {len(rows)} announced, {n} with a truth fit")

    def _stats(key):
        vals = np.array([r[key] for r in all_rows
                          if r[key] is not None], dtype=float)
        if vals.size == 0:
            return None, None, 0
        return float(np.mean(vals)), float(np.std(vals)), int(vals.size)

    last_bias, last_sd, last_n = _stats('last_err_ms')
    dl_bias, dl_sd, dl_n = _stats('dl_err_ms')

    last_pass = (last_bias is not None and abs(last_bias) <= 10.0 and last_sd <= 15.0)
    dl_pass = (dl_bias is not None and abs(dl_bias) <= 20.0)

    cols = ['bag', 'ball_id', 'source', 'truth_ok', 'truth_n', 'truth_rms_mm',
            'last_err_ms', 'dl_err_ms', 'final_status']
    lines = ['| ' + ' | '.join(cols) + ' |',
             '|' + '|'.join(['---'] * len(cols)) + '|']
    for r in all_rows:
        def fmt(v):
            if v is None:
                return ''
            if isinstance(v, float):
                return f'{v:.2f}'
            return str(v)
        lines.append('| ' + ' | '.join(fmt(r[c]) for c in cols) + ' |')

    summary = (
        f"# Tracker landing-estimate validation ({', '.join(bag_names)})\n\n"
        f"landing_z={landing_z:.1f} mm  criterion: |bias|<=10ms AND sd<=15ms at last "
        f"in-flight sample; |bias|<=20ms at dispatch\n\n"
        + '\n'.join(per_bag_lines) + '\n\n'
        f"Last in-flight sample: bias={('n/a' if last_bias is None else f'{last_bias:+.2f} ms')}  "
        f"sd={('n/a' if last_sd is None else f'{last_sd:.2f} ms')}  n={last_n}  "
        f"-> {'PASS' if last_pass else 'FAIL'}\n\n"
        f"Dispatch instant: bias={('n/a' if dl_bias is None else f'{dl_bias:+.2f} ms')}  "
        f"sd={('n/a' if dl_sd is None else f'{dl_sd:.2f} ms')}  n={dl_n}  "
        f"-> {'PASS' if dl_pass else 'FAIL'}\n\n"
    )
    md = summary + '\n'.join(lines) + '\n'
    out_md.write_text(md)
    print(summary)
    print(f"wrote {out_md}")
    return 0 if (last_pass and dl_pass) else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', nargs='?', help='rosbag directory or .mcap file')
    ap.add_argument('bags', nargs='*', default=[],
                    help='(with --validate-landing) additional bags to aggregate')
    ap.add_argument('--confirm-deadline-s', type=float, default=0.25,
                    help='a ball must CONFIRM within this many seconds of release '
                         '(default 0.25 — the R3 catch deadline is ~0.47 s before '
                         'landing, so anything later is a NO_LANDING in the air)')
    ap.add_argument('--out-dir', default=None,
                    help='default temp/probes/ under the repo root')
    ap.add_argument('--validate-landing', action='store_true',
                    help='score landing-estimate accuracy against an offline '
                         'ground-truth parabola fit instead of the CONFIRM check')
    ap.add_argument('--out-md', default=None,
                    help='(--validate-landing) output path for the aggregated report')
    args = ap.parse_args(argv)

    if args.validate_landing:
        args.bags = ([args.bag] if args.bag else []) + list(args.bags)
        if not args.bags:
            ap.error('--validate-landing needs at least one bag')
        return _validate_landing_main(args)

    if not args.bag:
        ap.error('bag is required')
    mcap_path = _find_mcap(Path(os.path.expanduser(args.bag)))
    rows, stats, _records, _order, _mocap = replay(mcap_path, args.confirm_deadline_s)

    out_dir = Path(args.out_dir) if args.out_dir else (_REPO / 'temp' / 'probes')
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = mcap_path.stem
    csv_path = out_dir / f'tracker_bag_replay_{tag}.csv'
    md_path = out_dir / f'tracker_bag_replay_{tag}.md'

    cols = list(rows[0].keys()) if rows else ['ball_id']
    with open(csv_path, 'w', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        w.writerows(rows)

    n_pass = sum(1 for r in rows if r['pass_'] == 'PASS')
    header = (f"bag={mcap_path}\n"
              f"mocap frames={stats['frames']}  markers={stats['markers']}  "
              f"eligible after label exclusion={stats['eligible']}\n"
              f"announced balls={stats['announced']}  "
              f"PASS={n_pass}/{len(rows)} "
              f"(confirm within {args.confirm_deadline_s:.2f} s of release "
              f"AND a landing estimate)\n"
              f"excluded_label_prefixes={parse_label_prefixes(hw.TRACKING_EXCLUDED_LABEL_PREFIXES)}  "
              f"announced_gate_mm={hw.TRACKING_ANNOUNCED_GATE_MM}  "
              f"detect_human_throws={bool(hw.TRACKING_DETECT_HUMAN_THROWS)}\n")
    lines = ['| ' + ' | '.join(cols) + ' |',
             '|' + '|'.join(['---'] * len(cols)) + '|']
    for r in rows:
        lines.append('| ' + ' | '.join(str(r[c]) for c in cols) + ' |')
    md = header + '\n' + '\n'.join(lines) + '\n'
    md_path.write_text(md)

    print(header)
    print('\n'.join(lines))
    print(f"\nwrote {csv_path}\nwrote {md_path}")
    return 0 if (rows and n_pass == len(rows)) else 1


if __name__ == '__main__':
    raise SystemExit(main())
