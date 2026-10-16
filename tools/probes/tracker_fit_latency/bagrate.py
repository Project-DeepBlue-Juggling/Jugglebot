"""Rates of /mocap_data vs /balls in a bag, and the /balls publish times.

Recipe (repo root, project venv):
  python tools/probes/tracker_fit_latency/bagrate.py ~/Desktop/rosbags/2026-10-16_10-50-42 temp/probes/tracker_fit_latency/rate_S16.json
Inputs: a bag directory (S16 = ~/Desktop/rosbags/2026-10-16_10-50-42, S8 = ~/Desktop/rosbags/2026-10-13_23-12-50_2-Ball-32-Catches). Output: JSON {mocap_t, balls}, the
input of probe_decim.py.
Result 2026-10-16: mocap 286.8 Hz (S8) / 292.0 Hz (S16); in-flight /balls published for p50
0.45 (S8) / 0.33 (S16) of the mocap frames."""
import sys, json, os
from pathlib import Path
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

mc = sorted(Path(os.path.expanduser(sys.argv[1])).glob('*.mcap'))[0]
dec = DecoderFactory()
mocap_t = []
balls = []   # (log_t, header_stamp, [(throw_time_ns, from_fit, tracking, status, id)])
with open(mc, 'rb') as fh:
    rd = make_reader(fh)
    decoders = {}
    for schema, ch, msg in rd.iter_messages(topics=['/mocap_data', '/balls']):
        if ch.topic == '/mocap_data':
            mocap_t.append(msg.log_time * 1e-9)
            continue
        d = decoders.get(schema.id)
        if d is None:
            d = decoders[schema.id] = dec.decoder_for(ch.message_encoding, schema)
        m = d(msg.data)
        hs = None
        lst = []
        for b in m.balls:
            hs = b.header.stamp.sec + b.header.stamp.nanosec * 1e-9
            lst.append((b.throw_time.sec * 10**9 + b.throw_time.nanosec, bool(b.landing_from_fit),
                        int(b.tracking), int(b.status), int(b.id)))
        balls.append((msg.log_time * 1e-9, hs, lst))
mocap_t = np.array(mocap_t)
bt = np.array([b[0] for b in balls])
print('mocap msgs', len(mocap_t), 'span', mocap_t[-1] - mocap_t[0], 'rate', len(mocap_t) / (mocap_t[-1] - mocap_t[0]))
print('balls msgs', len(bt), 'rate', len(bt) / (bt[-1] - bt[0]))
dm = np.diff(mocap_t); db = np.diff(bt)
print('mocap gap p50/p99/max ms', np.percentile(dm, [50, 99, 100]) * 1e3)
print('balls gap p50/p99/max ms', np.percentile(db, [50, 99, 100]) * 1e3)
json.dump(dict(mocap_t=mocap_t.tolist(), balls=balls), open(sys.argv[2], 'w'))
