"""Per-frame trace of named balls through the decimated (live-faithful) replay.

Same inputs as probe_decim.py (bag, bagrate.py JSON, lag_s); additionally traces ball ids in TRACE
(edit the set) and stops at STOP: per frame, time since throw, status, tracking, frames without
measurement, fit samples, distance from the nearest marker to the CONFIRMED filter, filter z.
Recipe (repo root, project venv):
  python tools/probes/tracker_fit_latency/probe_trace.py ~/Desktop/rosbags/2026-10-16_10-50-42 temp/probes/tracker_fit_latency/rate_S16.json 0.0 temp/probes/tracker_fit_latency/trace_S16.json
Result 2026-10-16 (pre-fix matcher): S16 balls 4/15/18: nearest marker to the filter jumps to 63-118 mm
after confirmation, frames_without_measurement climbs for the rest of the flight, fit holds 0-2 samples."""
import sys, json, os, bisect
from pathlib import Path
import numpy as np
REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / 'tools' / 'probes')); sys.path.insert(0, str(REPO / 'ros_ws' / 'src' / 'jugglebot'))
import tracker_bag_replay as tbr
from jugglebot.tracking.ball import TrackingConfidence, BallStatus
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
bag, ratef, lag, out = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
r = json.load(open(ratef)); mt = r['mocap_t']; pub = [b[0] for b in r['balls']]
sel = {}   # mocap log_time -> publish time
for p in pub:
    i = bisect.bisect_right(mt, p - lag) - 1
    if i >= 0 and mt[i] not in sel: sel[mt[i]] = p
tr = tbr.make_tracker(); rec = {}
TRACE = {4,15,16,18,20,24,25,28}; STOP = 28; TR = {}
mc = tbr._find_mcap(Path(bag)); dec = DecoderFactory(); ds = {}
with open(mc, 'rb') as fh:
    for schema, ch, msg in make_reader(fh).iter_messages(topics=['/mocap_data', '/throw_announcements']):
        t_bag = msg.log_time * 1e-9
        if ch.topic == '/mocap_data' and t_bag not in sel: continue
        d = ds.get(schema.id) or ds.setdefault(schema.id, dec.decoder_for(ch.message_encoding, schema))
        m = d(msg.data)
        if ch.topic == '/throw_announcements':
            tt = tbr._stamp(m.throw_time); lt = tbr._stamp(m.landing_time)
            v = lambda q: np.array([q.x, q.y, q.z])
            bid = tr.handle_announcement(v(m.initial_position), v(m.initial_velocity), tt if tt > 1 else t_bag,
                source=m.thrower_name or 'ball_butler', destination=m.target_id or '',
                landing_position=v(m.landing_position) if lt > 0 else None,
                landing_velocity=v(m.landing_velocity) if lt > 0 else None, landing_time=lt if lt > 0 else None)
            rec[bid] = dict(id=bid, source=m.thrower_name, throw_time=tt, ann_landing=lt, F=None, t_conf=None)
            continue
        pos = [np.array([k.position.x, k.position.y, k.position.z]) for k in m.markers]
        lab = [k.label or '' for k in m.markers]
        p = sel[t_bag]
        tf = tbr._frame_time(m, t_bag)
        for b in tr.process_frame(pos, tf, lab):
            if b.id in TRACE:
                f = tr._flight_fits.get(b.id)
                fr = f.fit() if f is not None and len(f) >= 12 else None
                kfp = tr._filters[b.id].position if b.id in tr._filters else None
                near = min((float(np.linalg.norm(x - kfp)) for x in pos), default=-1) if kfp is not None else -1
                TR.setdefault(b.id, []).append((round(tf - b.throw_time, 4), b.status.name[:4], b.tracking.name[:4], b.frames_without_measurement,
                    len(f) if f is not None else -1, (fr.n if fr else None), round(near, 1), round(float(kfp[2]) if kfp is not None else 0), bool(b.landing_from_fit)))
            if b.id == STOP and b.status != BallStatus.IN_FLIGHT and b.status != BallStatus.TO_BE_THROWN: raise SystemExit(json.dump(TR, open(out, 'w')))
            q = rec.get(b.id)
            if q is None: continue
            if q['t_conf'] is None and b.tracking == TrackingConfidence.CONFIRMED: q['t_conf'] = p
            if q['F'] is None and b.landing_from_fit and b.status == BallStatus.IN_FLIGHT: q['F'] = p
json.dump(list(rec.values()), open(out, 'w'))
jb = [q for q in rec.values() if q['source'] == 'jugglebot']
lf = np.array([(q['ann_landing'] - q['F']) * 1e3 for q in jb if q['F']])
print('lag', lag, 'frames', len(sel), 'jugglebot', len(jb), 'landing-F p10/50/90', np.percentile(lf, [10, 50, 90]).round(0), 'never', sum(q['F'] is None for q in jb),
      'all-src never', sum(q['F'] is None for q in rec.values()), 'of', len(rec))
