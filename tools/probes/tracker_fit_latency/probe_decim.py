"""Live-faithful A/B: replay ONLY the /mocap_data frames the live node could have processed.

For each live /balls publish time p (from bagrate.py's JSON), feed the newest /mocap_data frame
logged at or before p - lag_s; F is the publish time p of the first landing_from_fit frame.
Recipe (repo root, project venv):
  python tools/probes/tracker_fit_latency/bagrate.py ~/Desktop/rosbags/2026-10-16_10-50-42 temp/probes/tracker_fit_latency/rate_S16.json
  python tools/probes/tracker_fit_latency/probe_decim.py ~/Desktop/rosbags/2026-10-16_10-50-42 temp/probes/tracker_fit_latency/rate_S16.json 0.0 temp/probes/tracker_fit_latency/dec_S16_0.json
(same for S8 = ~/Desktop/rosbags/2026-10-13_23-12-50_2-Ball-32-Catches). Run it before and after the matcher fix for the before/after table.
Result 2026-10-16 (pre-fix matcher): S16 landing - F p10/p50/p90 157/547/631 ms, 70/212 never
(live 142/540/614, 77); S8 509/638/678 ms, 17/145 never (live 566/633/671, 25)."""
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
        for b in tr.process_frame(pos, tbr._frame_time(m, t_bag), lab):
            q = rec.get(b.id)
            if q is None: continue
            if q['t_conf'] is None and b.tracking == TrackingConfidence.CONFIRMED: q['t_conf'] = p
            if q['F'] is None and b.landing_from_fit and b.status == BallStatus.IN_FLIGHT: q['F'] = p
json.dump(list(rec.values()), open(out, 'w'))
jb = [q for q in rec.values() if q['source'] == 'jugglebot']
lf = np.array([(q['ann_landing'] - q['F']) * 1e3 for q in jb if q['F']])
print('lag', lag, 'frames', len(sel), 'jugglebot', len(jb), 'landing-F p10/50/90', np.percentile(lf, [10, 50, 90]).round(0), 'never', sum(q['F'] is None for q in jb),
      'all-src never', sum(q['F'] is None for q in rec.values()), 'of', len(rec))
