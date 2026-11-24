#!/usr/bin/env python3
"""Replay a bag's marker stream through the LIVE trapped-marker detector (FEAT-50).

Feeds ``tracking/trapped_ball.OnlineDwellDetector`` exactly as ``ball_tracker_node``
does (every STRIDE-th ``/mocap_data`` frame, unlabelled markers only, Platform pose
from ``/rigid_body_poses``, Hand rev from ``/hand_telemetry``) and reports:
  * every WARN it would have logged (bag time, dwell duration, position, Hand rev),
  * the detector's own cost per fed frame (mean / p50 / p99 / max, microseconds;
    time of ``to_platform_frame`` + ``update`` only, decode excluded; the frames are
    extracted once and re-timed ``--repeats`` times, the lowest-mean run is reported),
  * the implied CPU at the live feed rate (fed frames/s x mean cost).
Offline and read-only. The Tracker is load-sensitive: run under ``nice`` and read the
numbers as relative (before/after) rather than absolute.

    python tools/probes/trapped_ball_replay.py BAG [--detector-file OLD_trapped_ball.py]
        [--window T0 T1] [--out temp/probes/trapped_replay_<bag>.json]
``--detector-file`` loads an alternative copy of trapped_ball.py (e.g. the pre-fix
one) for a before/after cost comparison on identical input.
"""
from __future__ import annotations

import argparse
import glob
import importlib.util
import json
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, "ros_ws", "src", "jugglebot"))


def _load(path):
    if not path:
        from jugglebot.tracking import trapped_ball
        return trapped_ball
    spec = importlib.util.spec_from_file_location("trapped_ball_alt", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _extract(bag, window, stride):
    """Stream the bag once; return the fed frames [(t, local_xyz_array, rev)] exactly as the node builds them."""
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
    from jugglebot.tracking import trapped_ball as tb0
    df = DecoderFactory()
    dec = {}
    pose = None
    rev = None
    n_mocap = 0
    frames = []
    kw = {}
    if window:
        kw = {"start_time": int(window[0] * 1e9), "end_time": int(window[1] * 1e9)}
    f = sorted(glob.glob(os.path.join(os.path.expanduser(bag), "*.mcap")))[0]
    with open(f, "rb") as fh:
        rd = make_reader(fh)
        for schema, ch, msg in rd.iter_messages(
                topics=["/mocap_data", "/rigid_body_poses", "/hand_telemetry"], **kw):
            top = ch.topic
            if top == "/mocap_data":
                n_mocap += 1
                if (n_mocap - 1) % stride or pose is None or rev is None:
                    continue
            d = dec.get(schema.id)
            if d is None:
                d = dec[schema.id] = df.decoder_for(ch.message_encoding, schema)
            m = d(msg.data)
            t = msg.log_time / 1e9
            if top == "/hand_telemetry":
                rev = float(m.pos_meas)
            elif top == "/rigid_body_poses":
                for b in m.bodies:
                    if b.name == "Platform":
                        p, q = b.pose.pose.position, b.pose.pose.orientation
                        pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            else:
                world = [[k.position.x, k.position.y, k.position.z] for k in m.markers if not k.label]
                frames.append((t, np.array(world) if world else None, pose, rev))
    return frames


def replay(bag, tb, window=None, repeats=1):
    frames = _extract(bag, window, tb.STRIDE)
    runs = []
    events = []
    for r in range(max(1, repeats)):
        det = tb.OnlineDwellDetector()
        costs = []
        ev_run = []
        for t, world, pose, rev in frames:
            a = time.perf_counter_ns()
            local = tb.to_platform_frame(world, pose) if world is not None else []
            ev = det.update(t, local, rev)
            costs.append(time.perf_counter_ns() - a)
            for e in ev:
                ev_run.append({"t": t, **{k: v for k, v in e.items() if k != "t"}})
        runs.append(np.array(costs, float) / 1e3)
        events = ev_run
    n = len(frames)
    span = (frames[-1][0] - frames[0][0]) if n > 1 else 0.0
    best = min(runs, key=lambda c: c.mean()) if n else None
    mean_markers = float(np.mean([0 if w is None else len(w) for _t, w, _p, _r in frames])) if n else 0.0
    return {"bag": os.path.basename(os.path.normpath(bag)), "fed_frames": n, "span_s": span,
            "fed_hz": n / span if span else 0.0, "mean_unlabelled_markers": mean_markers,
            "repeats": len(runs), "events": events,
            "cost_us_best_run": {"mean": float(best.mean()), "p50": float(np.percentile(best, 50)),
                                 "p99": float(np.percentile(best, 99)), "max": float(best.max())} if n else None,
            "worst_frame_us_over_all_runs": float(max(c.max() for c in runs)) if n else None,
            "cpu_pct_of_one_core_at_feed_rate": float(best.mean() * 1e-6 * n / span * 100) if span else 0.0}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--detector-file", default=None)
    ap.add_argument("--window", nargs=2, type=float, default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--repeats", type=int, default=5, help="re-time the detector this many times on the cached frames")
    a = ap.parse_args(argv)
    res = replay(a.bag, _load(a.detector_file), tuple(a.window) if a.window else None, a.repeats)
    out = a.out or os.path.join(REPO, "temp", "probes", "trapped_replay_%s.json" % res["bag"])
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as fh:
        json.dump(res, fh, indent=1, default=float)
    print(json.dumps({k: v for k, v in res.items() if k != "events"}, indent=1))
    for e in res["events"]:
        print("WARN t=%.3f dur=%.2fs at (%.0f,%.0f,%.0f) rev %.2f" % (e["t"], e["dur_s"], *e["local_mean"], e["hand_rev"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
