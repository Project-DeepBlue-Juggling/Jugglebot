"""Executor-stall length vs mocap_node's logged queue-overflow drops, from launch logs.

Recipe (repo root, project venv; reads logs only, no ROS needed):
  python tools/probes/mocap_starvation/log_stalls.py [~/.ros/log/<stamp>/launch.log ...]
Default: glob ~/.ros/log/2026-10-1[1-6]-*/launch.log.
Per log, the time base is the epoch stamp that prefixes each launch.log line.
  * sweep end  = mocap_node DEBUG line "Calibration sweep complete ... Processing..."
  * 1 Hz line  = mocap_node DEBUG "qtm clock sync: ..." (the executor timer line the
                 logbook entry used; a gap in it is an executor stall)
  * stall T    = first 1 Hz line after a sweep end minus the sweep end (the executor is
                 inside _finalize_calibration until then). Entry definition; the "largest
                 gap within 20 s" variant gives the same value on every sweep checked.
  * predicted  = 300*T - 150 (300 Hz x stall, minus the 150-frame FRAME_QUEUE_MAXLEN)
  * logged     = N from "N dropped by mocap_node" in WARNs 0-20 s after that sweep end
                 (summed if there are several; each WARN is charged to the LATEST sweep end before it).
Only WARNs with N > 0 count as node-overflow WARNs (the WARN also fires for pure QTM gaps).
Every such WARN is then classified within/outside 20 s of ANY sweep end.
Result 2026-10-16 (logbook 2026-10-16-mocap-frame-queue-starvation): see the entry's
Diagnosis 1-2; the run's actual numbers are recorded in its Verification rows.
Used by that entry."""
import glob, os, re, sys

WIN = 20.0
TOL = 15
a = sys.argv[1:] or sorted(glob.glob(os.path.expanduser('~/.ros/log/2026-10-1[1-6]-*/launch.log')))
files = [os.path.expanduser(p) for p in a]
ts_re = re.compile(r'^(\d+\.\d+) ')
drop_re = re.compile(r'(\d+) dropped by mocap_node')
n_warn = n_in = 0
outside = []
n_sweeps = n_ok = 0
for f in files:
    sweeps, ones, warns = [], [], []
    with open(f, errors='replace') as fh:
        for line in fh:
            m = ts_re.match(line)
            if not m or '[mocap_node' not in line:
                continue
            t = float(m.group(1))
            if 'Calibration sweep complete' in line and 'Processing' in line:
                sweeps.append(t)
            elif 'qtm clock sync:' in line:
                ones.append(t)
            elif '[WARN]' in line:
                d = drop_re.search(line)
                if d:
                    warns.append((t, int(d.group(1))))
    warns = [w for w in warns if w[1] > 0]  # node-overflow WARNs only (N > 0)
    n_warn += len(warns)
    for t, n in warns:
        if any(0 <= t - s <= WIN for s in sweeps):
            n_in += 1
        else:
            outside.append((f, t, n))
    rows = []
    for s in sweeps:
        nxt = [o for o in ones if o > s]
        T = nxt[0] - s if nxt else float('nan')
        logged = sum(n for t, n in warns if 0 <= t - s <= WIN
                     and max(x for x in sweeps if x <= t) == s)
        rows.append((s, T, 300 * T - 150, logged))
    if rows:
        print(os.path.basename(os.path.dirname(f)))
        for s, T, p, lg in rows:
            ok = lg == 0 and p < 150 or abs(p - lg) <= TOL
            n_sweeps += lg > 0
            n_ok += lg > 0 and abs(p - lg) <= TOL
            print('  sweep@%.1f  T=%.2f s  predicted=%5.0f  logged=%5d  %s'
                  % (s, T, p, lg, 'ok' if ok else 'MISMATCH'))
print('\nsweeps with a logged overflow: %d, predicted within %d frames: %d' % (n_sweeps, TOL, n_ok))
print('node-overflow WARNs: %d across %d logs; within %.0f s of a sweep end: %d; outside: %d'
      % (n_warn, len(files), WIN, n_in, len(outside)))
for f, t, n in outside:
    print('  outside: %s t=%.1f dropped=%d' % (os.path.basename(os.path.dirname(f)), t, n))
