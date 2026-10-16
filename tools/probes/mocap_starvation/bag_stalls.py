"""/mocap_data publish stalls in a bag, and what the frames published after each stall carry.

Recipe (repo root, project venv):
  python tools/probes/mocap_starvation/bag_stalls.py ~/Desktop/rosbags/2026-10-16_16-10-37 [min_gap_s=0.1]
For every publish (log_time) gap >= min_gap_s it prints: the gap, the QTM-stamp span the
gap's neighbours cover (stamp gap = frames lost to the node, not the stream), and the size
and publish duration of the catch-up burst (frames whose stamp is > 20 ms older than their
publish time). A node-side executor stall shows as: log gap ~= stamp gap ~= stall length,
followed by a catch-up burst of up to a few hundred frames over ~0.3 s (the 150 queued
frames, FRAME_QUEUE_MAXLEN, plus those arriving while the drain catches up; 242 in 314 ms at 16:10). A receive-
side stall (H3) would show a stamp gap with NO preceding publish gap at 300 Hz cadence.
Used by logbook 2026-10-16-mocap-frame-queue-starvation."""
import sys, os
from pathlib import Path
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

bag = Path(os.path.expanduser(sys.argv[1]))
min_gap = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1
mc = sorted(bag.glob('*.mcap'))[0]
dec = DecoderFactory(); decoders = {}
log_t, stamp_t = [], []
with open(mc, 'rb') as fh:
    for schema, ch, msg in make_reader(fh).iter_messages(topics=['/mocap_data']):
        d = decoders.get(schema.id) or decoders.setdefault(schema.id, dec.decoder_for(ch.message_encoding, schema))
        m = d(msg.data)
        log_t.append(msg.log_time * 1e-9)
        stamp_t.append(m.stamp.sec + m.stamp.nanosec * 1e-9 if hasattr(m, 'stamp') else np.nan)
log_t = np.array(log_t); stamp_t = np.array(stamp_t)
span = log_t[-1] - log_t[0]
print(f'{mc.name}: {len(log_t)} /mocap_data msgs over {span:.1f} s = {len(log_t)/span:.1f} Hz')
lag = log_t - stamp_t
print(f'publish - QTM stamp: p50 {np.nanpercentile(lag,50)*1e3:.1f} ms, p99 {np.nanpercentile(lag,99)*1e3:.1f} ms')
dl = np.diff(log_t)
for i in np.where(dl >= min_gap)[0]:
    j = i + 1; k = j
    while k < len(log_t) and lag[k] > 0.020 + np.nanpercentile(lag, 50): k += 1
    sg = stamp_t[j] - stamp_t[i]
    print(f'  t={log_t[i]:.2f}  publish gap {dl[i]*1e3:7.1f} ms  stamp gap {sg*1e3:7.1f} ms '
          f'(~{max(0, round(sg*300)-1)} frames never published)  catch-up burst {k-j} frames in '
          f'{(log_t[k-1]-log_t[j])*1e3 if k > j else 0:.1f} ms')
# Steady state outside stalls: frames per second histogram
sec = np.floor(log_t - log_t[0]).astype(int)
counts = np.bincount(sec)
print('per-second /mocap_data counts: p5/p50/p95', np.percentile(counts[1:-1], [5, 50, 95]),
      ' seconds < 250:', int((counts[1:-1] < 250).sum()), 'of', len(counts) - 2)
