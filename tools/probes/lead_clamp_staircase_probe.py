#!/usr/bin/env python3
"""lead_clamp_staircase_probe.py — the 2026-10-24 leg lead-clamp staircase, its anchor-age
distribution, and a replay of today's clamp against the age-aware options.

Motivation: logbook/2026-10-24-leg-lead-clamp-age-aware-p1-firmware.md (plan
plans/archived/leg-lead-clamp-age-aware.md). The A/B oval sitting showed /leg_cmd_executed
FROZEN for 10-20 ms then stepping ~10 mm while the target velocity was > 0.5 rev/s: the
firmware clamp anchors to the last encoder frame (pos_rev), the encoder frames arrive
irregularly (median ~10 ms, up to ~100 ms), and a moving leg travels more than 0.10 rev during such a gap.

Reads the A/B review's decoded pickles (temp/probes/oval_carry_2026-10-24_*/bag.pkl) for
/robot_state, /leg_cmd_executed, /leg_setpoint_echo; decodes /cache_diag and /link_status from
the mcap (cached under temp/probes/lead_clamp_age_aware/).

Run (venv):
  python tools/probes/lead_clamp_staircase_probe.py                  # report -> temp/probes/lead_clamp_age_aware/
  python tools/probes/lead_clamp_staircase_probe.py --emit-fixture   # (re)write tools/probes/data/lead_clamp_staircase_2026-10-24.json
  python tools/probes/lead_clamp_staircase_probe.py --fixture-only   # twin replay of the committed fixture, no bags needed
  python tools/probes/lead_clamp_staircase_probe.py --emit-fixture-0815   # (re)write ..._2026-08-15.json (P2)
  python tools/probes/lead_clamp_staircase_probe.py --scan <bag.mcap>     # P3 gate read-back of any Sitting bag

P2 (2026-10-25) adds the archived leg-bus-frame-drops plan's 2026-08-15 episodes (FW 14,
bag 2026-08-15_10-08-06, decoded straight from the mcap into temp/probes/lead_clamp_p2_0815/):
ax1 (leg 1, a staircase — the oracle) and ax4 (leg 4, ~100 ms gaps that never engage the
clamp — a no-harm window). ax5 (a motionless hold: no command travel to staircase) and ax2
(cache age 18 ms, below the 30 ms census threshold) carry nothing to replay and are not cut.
CAVEAT shared by every window: 'a fresh frame' = /robot_state pos_estimate changed, which is
only trustworthy while the leg moves (a still leg reads as one long gap).

The fixture is a MEASUREMENT RECORD (three windows); regenerate only on purpose.
`replay_window(window, mode)` drives the Python twin (teensy_interp.TeensyLegInterp) at 500 Hz
over a fixture window and is the oracle tests/firmware/test_leg_clamp_age_aware_twin.py uses.
"""
import os, sys, pickle, json, argparse
import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, os.path.join(REPO, 'tools', 'probes', 'teensy_link_profiling', 'hermite_xref'))
FIXTURE = os.path.join(REPO, 'tools', 'probes', 'data', 'lead_clamp_staircase_2026-10-24.json')
MM_PER_REV = 70.7
# The three main windows: (name, bag key, leg, t0, t1) — episode bounds from this probe's own scan.
WINDOWS = [
    ('S1A_leg1', '10-44-21', 1, 1792799742.51, 1792799742.90),
    ('S1B_leg2', '10-44-21', 2, 1792799921.77, 1792799922.61),
    ('S2_leg1', '11-34-29', 1, 1792802191.80, 1792802192.05),
]
BAGS = {
    '10-44-21': ('/home/jetson/Desktop/rosbags/2026-10-24_10-44-21/2026-10-24_10-44-21_0.mcap',
                 f'{REPO}/temp/probes/oval_carry_2026-10-24_10-44-21/bag.pkl'),
    '11-34-29': ('/home/jetson/Desktop/rosbags/2026-10-24_11-34-29/2026-10-24_11-34-29_0.mcap',
                 f'{REPO}/temp/probes/oval_carry_2026-10-24_11-34-29/bag.pkl'),
}
FIXTURE_0815 = os.path.join(REPO, 'tools', 'probes', 'data', 'lead_clamp_staircase_2026-08-15.json')
WINDOWS_0815 = [
    ('ax1_leg1', '10-08-06', 1, 1786752543.59, 1786752544.12),
    ('ax4_leg4', '10-08-06', 4, 1786752548.30, 1786752548.75),
]
BAGS['10-08-06'] = ('/home/jetson/Desktop/rosbags/2026-08-15_10-08-06/2026-08-15_10-08-06_0.mcap',
                    f'{REPO}/temp/probes/lead_clamp_p2_0815/bag.pkl')
OUT = f'{REPO}/temp/probes/lead_clamp_age_aware'
LEAD = 0.10
CAPS = [0.100, 0.150]
VEL_MIN = 0.5          # rev/s: the review's "frozen while target_vel > 0.5" signature


def tod(m):
    if hasattr(m, '__slots__'):
        return {k: tod(getattr(m, k)) for k in m.__slots__}
    if isinstance(m, (list, tuple)):
        return [tod(x) for x in m]
    return m


def load_diag(mcap, cache):
    if os.path.exists(cache):
        return pickle.load(open(cache, 'rb'))
    from mcap_ros2.reader import read_ros2_messages
    d = {'/cache_diag': [], '/link_status': []}
    for r in read_ros2_messages(mcap, topics=list(d)):
        kv = {v.key: v.value for v in r.ros_msg.values}
        d[r.channel.topic].append((r.log_time_ns, kv))
    os.makedirs(os.path.dirname(cache), exist_ok=True)
    pickle.dump(d, open(cache, 'wb'))
    return d


def arrays(d):
    rs = d['/robot_state']
    t = np.array([x[0] for x in rs]) * 1e-9
    pos = np.array([[ms['pos_estimate'] for ms in x[1]['motor_states'][:6]] for x in rs])
    vel = np.array([[ms['vel_estimate'] for ms in x[1]['motor_states'][:6]] for x in rs])
    iqs = np.array([[ms['iq_setpoint'] for ms in x[1]['motor_states'][:6]] for x in rs])
    lce = d['/leg_cmd_executed']
    lt = np.array([x[0] for x in lce]) * 1e-9
    lpos = np.array([x[1]['position'][:6] for x in lce])
    lvel = np.array([x[1]['velocity'][:6] for x in lce])
    echo = d['/leg_setpoint_echo']
    et = np.array([x[0] for x in echo]) * 1e-9
    epos = np.array([x[1]['data'][:6] for x in echo])
    return dict(t=t, pos=pos, vel=vel, iqs=iqs, lt=lt, lpos=lpos, lvel=lvel, et=et, epos=epos)


def staircase_episodes(A):
    """Frozen /leg_cmd_executed target while |target_vel| > VEL_MIN → runs → episodes."""
    eps = []
    for leg in range(6):
        p, v, t = A['lpos'][:, leg], A['lvel'][:, leg], A['lt']
        frozen = np.r_[False, (np.abs(np.diff(p)) < 1e-7) & (np.abs(v[1:]) > VEL_MIN)]
        # runs of frozen samples
        runs = []
        k = 0
        n = len(frozen)
        while k < n:
            if frozen[k]:
                j = k
                while j + 1 < n and frozen[j + 1]:
                    j += 1
                runs.append((k, j))
                k = j + 1
            else:
                k += 1
        # merge runs < 0.3 s apart into episodes
        cur = None
        for (a, b) in runs:
            if cur and t[a] - t[cur[1]] < 0.3:
                cur[1] = b; cur[2].append((a, b))
            else:
                if cur: eps.append((leg, cur))
                cur = [a, b, [(a, b)]]
        if cur: eps.append((leg, cur))
    out = []
    for leg, (a, b, runs) in eps:
        if len(runs) < 3:
            continue
        t0, t1 = A['lt'][runs[0][0] - 1], A['lt'][b]
        steps = [A['lpos'][r[1] + 1, leg] - A['lpos'][r[1], leg] for r in runs if r[1] + 1 < len(A['lt'])]
        out.append(dict(leg=leg, t0=float(t0), t1=float(t1), dur=float(t1 - t0), n_runs=len(runs),
                        n_frozen=int(sum(r[1] - r[0] + 1 for r in runs)),
                        step_med_mm=float(np.median(np.abs(steps)) * 70.7) if steps else 0.0,
                        step_max_mm=float(np.max(np.abs(steps)) * 70.7) if steps else 0.0))
    return out


def fresh_samples(A, leg):
    """Times/values where /robot_state pos_estimate for `leg` changed (a fresh encoder frame)."""
    p = A['pos'][:, leg]
    ch = np.r_[True, np.diff(p) != 0.0]
    return A['t'][ch], p[ch], A['vel'][ch, leg]


def interval_stats(iv):
    iv = iv * 1e3
    return dict(n=int(len(iv)), median=float(np.median(iv)), p95=float(np.percentile(iv, 95)),
                p99=float(np.percentile(iv, 99)), max=float(np.max(iv)))


def cache_age_stats(diag, t0=None, t1=None):
    rows = []
    for ns, kv in diag['/cache_diag']:
        t = ns * 1e-9
        if t0 is not None and not (t0 - 1.0 <= t <= t1 + 1.0):
            continue
        rows.append([int(kv.get(f'age_max_us_{i}', -1)) for i in range(6)] +
                    [int(kv.get(f'age_min_us_{i}', -1)) for i in range(6)])
    r = np.array(rows, float)
    if not len(r):
        return None
    mx = r[:, :6] / 1e3
    return dict(n=int(len(r)),
                age_max_ms_median=[float(x) for x in np.median(mx, 0)],
                age_max_ms_p95=[float(x) for x in np.percentile(mx, 95, 0)],
                age_max_ms_max=[float(x) for x in mx.max(0)],
                age_min_ms_median=[float(x) for x in np.median(r[:, 6:] / 1e3, 0)])


def echo_shift(A, leg, t0, t1):
    """Lag (s) of /leg_cmd_executed behind /leg_setpoint_echo on a window, by brute-force xcorr."""
    m = (A['lt'] >= t0) & (A['lt'] <= t1)
    lt, lp = A['lt'][m], A['lpos'][m, leg]
    best = None
    for s in np.arange(0.0, 0.120, 0.002):
        e = np.interp(lt - s, A['et'], A['epos'][:, leg])
        r = np.sqrt(np.mean((e - lp) ** 2))
        if best is None or r < best[1]:
            best = (float(s), float(r))
    return best


def replay(A, leg, t0, t1, cap_s, pad=0.4):
    """Per-sample replay of today's clamp, X and Y against the fresh-encoder trace.

    raw command = echo shifted by the window's fitted lag (clean-sample residual reported).
    Age is quantised by the ~10 ms telemetry; this is a design-sizing replay, P2 runs the
    500 Hz twin.
    """
    ft, fp, fv = fresh_samples(A, leg)
    shift, _ = echo_shift(A, leg, t0 - 2.0, t0 - 0.2)       # fit the lag on the clean approach
    m = (A['lt'] >= t0 - pad) & (A['lt'] <= t1 + pad)
    lt, rec = A['lt'][m], A['lpos'][m, leg]
    raw = np.interp(lt - shift, A['et'], A['epos'][:, leg])
    idx = np.searchsorted(ft, lt, side='right') - 1
    idx[idx < 0] = 0
    fb, vel, age = fp[idx], fv[idx], lt - ft[idx]
    agec = np.minimum(age, cap_s)
    today = fb + np.clip(raw - fb, -LEAD, LEAD)
    fbx = fb + vel * agec
    optx = fbx + np.clip(raw - fbx, -LEAD, LEAD)
    band = LEAD + np.abs(vel) * agec
    opty = fb + np.clip(raw - fb, -band, band)
    # Option Z: dead-reckoned anchor = fb + (raw now - raw at the frame), shift frozen past the cap
    raw_at_frame = np.interp(ft[idx] - shift, A['et'], A['epos'][:, leg])
    dr = raw - raw_at_frame
    # freeze the shift at its value when age crossed the cap
    raw_at_cap = np.interp(ft[idx] + cap_s - shift, A['et'], A['epos'][:, leg])
    dr = np.where(age <= cap_s, dr, raw_at_cap - raw_at_frame)
    fbz = fb + dr
    optz = fbz + np.clip(raw - fbz, -LEAD, LEAD)

    def met(e, anchor):
        dev = e - raw
        return dict(pin_frac=float(np.mean(np.abs(dev) > 1e-3)),
                    max_dev_mm=float(np.max(np.abs(dev)) * 70.7),
                    max_step_mm=float(np.max(np.abs(np.diff(e))) * 70.7),
                    anchor_shift_max_mm=float(np.max(np.abs(anchor - fb)) * 70.7))
    clean = age < 0.03
    return dict(shift_ms=shift * 1e3,
                oracle_today_vs_recorded_mm=float(np.sqrt(np.mean((today - rec) ** 2)) * 70.7),
                raw_vs_recorded_clean_mm=float(np.sqrt(np.mean((raw[clean] - rec[clean]) ** 2)) * 70.7),
                recorded_max_step_mm=float(np.max(np.abs(np.diff(rec))) * 70.7),
                age_max_ms=float(age.max() * 1e3),
                vel_abs_max_rps=float(np.abs(vel).max()),
                today=met(today, fb), X=met(optx, fbx), Y=met(opty, fb + np.sign(raw - fb) * (band - LEAD)), Z=met(optz, fbz))


def report():
    os.makedirs(OUT, exist_ok=True)
    report = {}
    for name, (mcap, pkl) in BAGS.items():
        if name == '10-08-06':
            continue                 # the P2 fixture-only bag (no /cache_diag decode, FW 14)
        d = pickle.load(open(pkl, 'rb'))
        diag = load_diag(mcap, f'{OUT}/diag_{name}.pkl')
        A = arrays(d)
        eps = staircase_episodes(A)
        # whole-bag encoder interval stats per leg (fresh /robot_state samples)
        iv_all = {}
        for leg in range(6):
            ft, _, _ = fresh_samples(A, leg)
            iv_all[leg] = interval_stats(np.diff(ft))
        bag = dict(episodes=[], encoder_interval_ms_per_leg=iv_all,
                   cache_age_whole_bag=cache_age_stats(diag))
        for e in eps:
            leg, t0, t1 = e['leg'], e['t0'], e['t1']
            ft, _, _ = fresh_samples(A, leg)
            inwin = np.diff(ft)[(ft[1:] >= t0) & (ft[1:] <= t1)]
            e['encoder_interval_ms_in_episode'] = interval_stats(inwin) if len(inwin) else None
            e['cache_age_in_episode'] = cache_age_stats(diag, t0, t1)
            # accel sizing from fresh samples around the episode
            mm = (ft >= t0 - 0.5) & (ft <= t1 + 0.5)
            _, fp, fv = fresh_samples(A, leg)
            if mm.sum() > 2:
                a = np.diff(fv[mm]) / np.diff(ft[mm])
                e['accel_abs_max_rps2'] = float(np.abs(a).max())
            m = (A['t'] >= t0) & (A['t'] <= t1)
            e['iq_sp_abs_max_A'] = float(np.abs(A['iqs'][m, leg]).max()) if m.any() else None
            e['replay'] = {f'cap_{int(c*1e3)}ms': replay(A, leg, t0, t1, c) for c in CAPS}
            bag['episodes'].append(e)
        report[name] = bag
    json.dump(report, open(f'{OUT}/probe_report.json', 'w'), indent=1)
    # human summary
    for name, bag in report.items():
        print(f'\n===== bag {name}: {len(bag["episodes"])} staircase episodes')
        print(' encoder interval ms (whole bag, fresh /robot_state): ',
              {k: (round(v['median'], 1), round(v['p99'], 1), round(v['max'], 1)) for k, v in bag['encoder_interval_ms_per_leg'].items()})
        ca = bag['cache_age_whole_bag']
        if ca:
            print(' /cache_diag age_max_us per leg (ms): median', [round(x, 1) for x in ca['age_max_ms_median']],
                  'p95', [round(x, 1) for x in ca['age_max_ms_p95']], 'max', [round(x, 1) for x in ca['age_max_ms_max']],
                  '| age_min median', [round(x, 2) for x in ca['age_min_ms_median']])
        for e in bag['episodes']:
            print(f"\n  leg {e['leg']}  {e['t0']:.2f}-{e['t1']:.2f} ({e['dur']:.2f} s) runs={e['n_runs']} frozen={e['n_frozen']} "
                  f"step med/max {e['step_med_mm']:.1f}/{e['step_max_mm']:.1f} mm  iq_sp max {e['iq_sp_abs_max_A']} A "
                  f"accel max {e.get('accel_abs_max_rps2', 0):.1f} rev/s2")
            print('    enc interval in episode:', e['encoder_interval_ms_in_episode'])
            print('    cache_diag in episode:', e['cache_age_in_episode'] and
                  {'age_max_ms_max': [round(x, 1) for x in e['cache_age_in_episode']['age_max_ms_max']]})
            for cap, r in e['replay'].items():
                print(f"    replay {cap}: shift {r['shift_ms']:.0f} ms | oracle today-vs-recorded rms {r['oracle_today_vs_recorded_mm']:.2f} mm, "
                      f"raw-vs-recorded clean rms {r['raw_vs_recorded_clean_mm']:.2f} mm | recorded max step {r['recorded_max_step_mm']:.1f} mm | "
                      f"age max {r['age_max_ms']:.0f} ms |v| max {r['vel_abs_max_rps']:.2f}")
                for k in ('today', 'X', 'Y', 'Z'):
                    print(f"      {k:5s} pin {r[k]['pin_frac']:.2f}  max|emit-raw| {r[k]['max_dev_mm']:.1f} mm  max step {r[k]['max_step_mm']:.1f} mm  anchor shift max {r[k]['anchor_shift_max_mm']:.1f} mm")
    print(f'\nwrote {OUT}/probe_report.json')


def decode_bag(mcap, pkl, topics=('/robot_state', '/leg_cmd_executed', '/leg_setpoint_echo')):
    """mcap -> the {topic: [(log_time_ns, msg_dict)]} pickle arrays() reads (cached)."""
    if os.path.exists(pkl):
        return pickle.load(open(pkl, 'rb'))
    from mcap_ros2.reader import read_ros2_messages
    d = {t: [] for t in topics}
    for r in read_ros2_messages(mcap, topics=list(topics)):
        d[r.channel.topic].append((r.log_time_ns, tod(r.ros_msg)))
    os.makedirs(os.path.dirname(pkl), exist_ok=True)
    pickle.dump(d, open(pkl, 'wb'))
    return d


def emit_fixture(path=FIXTURE, windows=WINDOWS, recipe='--emit-fixture'):
    """Per window: echo knots (t, u[6]) t0-0.6..t1+0.4, fresh encoder samples (t,pos,vel) and the
    recorded /leg_cmd_executed (t,pos,vel) for the window's leg. Bag receive times, epoch seconds."""
    out = {'recipe': f'python tools/probes/lead_clamp_staircase_probe.py {recipe}',
           'bags': {w[1]: os.path.basename(BAGS[w[1]][0]) for w in windows}, 'windows': []}
    pk = {}
    for name, bag, leg, t0, t1 in windows:
        if bag not in pk:
            pk[bag] = arrays(decode_bag(*BAGS[bag]))
        A = pk[bag]
        lo, hi = t0 - 0.6, t1 + 0.4
        me = (A['et'] >= lo) & (A['et'] <= hi)
        ft, fp, fv = fresh_samples(A, leg)
        mf = (ft >= lo - 0.5) & (ft <= hi)
        ml = (A['lt'] >= lo) & (A['lt'] <= hi)
        r = lambda x, n=7: [round(float(v), n) for v in x]
        out['windows'].append(dict(
            name=name, bag=bag, leg=leg, t0=t0, t1=t1,
            echo=[[round(float(t), 6)] + r(u) for t, u in zip(A['et'][me], A['epos'][me])],
            enc=[[round(float(a), 6), round(float(b), 7), round(float(c), 5)] for a, b, c in zip(ft[mf], fp[mf], fv[mf])],
            executed=[[round(float(a), 6), round(float(b), 7), round(float(c), 5)]
                      for a, b, c in zip(A['lt'][ml], A['lpos'][ml, leg], A['lvel'][ml, leg])]))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    json.dump(out, open(path, 'w'), separators=(',', ':'))
    print(f'wrote {path}: {os.path.getsize(path)/1e3:.0f} KB')


def load_fixture(path=FIXTURE):
    return json.load(open(path))


def replay_window(w, mode, tick_dt=0.002, pre=0.4, post=0.3):
    """Drive TeensyLegInterp(leg_clamp=mode) at 500 Hz over fixture window  20:00:11 up 43 days,  8:51,  8 users,  load average: 0.79, 1.09, 1.60
USER     TTY      FROM             LOGIN@   IDLE   JCPU   PCPU WHAT
jetson   pts/0    tmux(1952).%0    10Sep26  8:25m  5.78s  5.78s -zsh
jetson   pts/1    tmux(1952).%1    10Sep26 48:27  38:37  38:35  claude
jetson   pts/2    tmux(1952).%2    10Sep26 19:26  33:57  33:44  claude
jetson   pts/3    tmux(1952).%3    10Sep26  7:52   3:16   3:14  claude
jetson   pts/5    192.168.20.20    Fri09    8:22m  0.68s  0.00s tmux attach -t ROS2
jetson   pts/6    192.168.20.20    Fri09    7:30   0.68s  0.00s tmux attach -t General_1
jetson   pts/8    192.168.20.20    Fri09    7:52   0.68s  0.00s tmux attach -t General_2
jetson   pts/7    192.168.20.20    Fri09   33:05   0.69s  0.00s tmux attach -t Claude.

    RECIPE: latch the 40 Hz echo knots at their recorded times (u0=k, u1=k+1, u2=k+2, v0 = chord
    (u1-u0)/T as xref.py does); every tick feed fb / fb_ts from the leg's fresh encoder samples
    (sample-and-hold, stamp = the sample's /robot_state receive time); the other five legs are fed
    their own raw command at age 0 (never clamp). Returns arrays over the ticks of [t0-pre, t1+post].
    """
    from teensy_interp import TeensyLegInterp, SEGMENT_T_S
    leg = w['leg']
    echo = w['echo']
    enc = w['enc']
    te = [e[0] for e in echo]
    knots = [e[1:] for e in echo]
    interp = TeensyLegInterp([-100.0] * 6, [100.0] * 6, leg_clamp=mode)
    n = int(round((w['t1'] + post - (w['t0'] - pre)) / tick_dt))
    T = np.zeros(n); EM = np.zeros(n); EV = np.zeros(n); RAW = np.zeros(n)
    FB = np.zeros(n); AGE = np.zeros(n); ENG = np.zeros(n, bool); REF = np.zeros(n)
    k_lat = -1                       # last latched knot index
    j = 0                            # encoder sample index
    for s in range(n):
        t = w['t0'] - pre + s * tick_dt
        while k_lat + 3 < len(echo) and te[k_lat + 1] <= t:
            k_lat += 1               # latch knots in order; needs k+2 to exist
            k = k_lat
            if k + 2 >= len(echo):
                k_lat -= 1
                break
            u0, u1, u2 = knots[k], knots[k + 1], knots[k + 2]
            v0 = [(u1[i] - u0[i]) / SEGMENT_T_S for i in range(6)]
            interp.latch_setpoint(u0, v0, [0.0] * 6, [0.0] * 6, te[k], u1=u1, u2=u2)
        if k_lat < 0:
            continue
        while j + 1 < len(enc) and enc[j + 1][0] <= t:
            j += 1
        fb = [interp.cmd_pos[i] for i in range(6)]   # other legs: age 0, never clamp
        fts = [t] * 6
        fb[leg], fts[leg] = enc[j][1], enc[j][0]
        _, _, _ = interp.tick(t, fb, fts)
        T[s] = t; EM[s] = interp.cmd_pos[leg]; EV[s] = interp.cmd_vel[leg]; RAW[s] = interp.raw_pos[leg]
        FB[s] = fb[leg]; AGE[s] = t - fts[leg]; ENG[s] = interp.leg_clamped[leg]
        REF[s] = interp._leg_cmd_ref[leg]
    ok = T > 0
    return dict(t=T[ok], emit=EM[ok], vel=EV[ok], raw=RAW[ok], fb=FB[ok], age=AGE[ok], engaged=ENG[ok],
                ref=REF[ok], interp=interp)


def twin_summary(w):
    """Per mode: max emitted step (mm), clamp engagement over stale-gap ticks, and (firmware) the
    engagement on gap ticks whose frame-time tracking error was < 0.10 rev."""
    from teensy_interp import LEG_ANCHOR_STALE_S, MOTOR_FB_STALENESS_S, MAX_LEAD_REV_FW
    res = {}
    for mode in ('firmware_fw30', 'firmware_fw31', 'firmware'):
        r = replay_window(w, mode)
        gap = r['age'] > LEG_ANCHOR_STALE_S
        err_frame = np.abs(r['ref'] - r['fb'])
        sub = gap & (r['age'] <= MOTOR_FB_STALENESS_S) & (err_frame < MAX_LEAD_REV_FW)
        res[mode] = dict(max_step_mm=float(np.max(np.abs(np.diff(r['emit']))) * MM_PER_REV),
                         gap_ticks=int(gap.sum()),
                         gap_engaged_frac=float(r['engaged'][gap].mean()) if gap.any() else 0.0,
                         subband_gap_ticks=int(sub.sum()),
                         subband_gap_engaged=int(r['engaged'][sub].sum()),
                         max_emit_minus_raw_mm=float(np.max(np.abs(r['emit'] - r['raw'])) * MM_PER_REV))
    return res


def fixture_report(fx=None):
    fx = fx or load_fixture()
    if os.path.exists(FIXTURE_0815):
        fx = dict(fx, windows=fx['windows'] + load_fixture(FIXTURE_0815)['windows'])
    print('\n== twin replay of the committed fixture (500 Hz, sample-and-hold encoder) ==')
    out = {}
    for w in fx['windows']:
        r = twin_summary(w)
        out[w['name']] = r
        for mode, m in r.items():
            print(f"  {w['name']:9s} {mode:14s} max step {m['max_step_mm']:6.2f} mm | gap ticks {m['gap_ticks']:4d} "
                  f"engaged {m['gap_engaged_frac']*100:5.1f} % | sub-band gap ticks {m['subband_gap_ticks']:4d} "
                  f"engaged {m['subband_gap_engaged']:3d} | max|emit-raw| {m['max_emit_minus_raw_mm']:.1f} mm")
    os.makedirs(OUT, exist_ok=True)
    json.dump(out, open(f'{OUT}/twin_replay.json', 'w'), indent=1)
    return out


def _clamp_census(kv):
    """'clamp a b c d e f | stale a b c d e f' (the /link_status leg_clamp_diag row) -> (clamp, stale) or None."""
    v = kv.get('leg_clamp_diag', '')
    if not v.startswith('clamp '):
        return None
    c, st = v[len('clamp '):].split(' | stale ')
    return np.array([int(x) for x in c.split()]), np.array([int(x) for x in st.split()])


def scan(mcap):
    """P3 gate read-back: staircase episodes on /leg_cmd_executed + the bridge's lead-clamp census over the bag.

    The census is cumulative since bridge boot, so it is DIFFERENCED first-to-last; a counter that went
    backwards means the bridge rebooted mid-bag and the delta is reported per boot segment.
    """
    stem = os.path.splitext(os.path.basename(mcap))[0]
    cache = f'{REPO}/temp/probes/lead_clamp_p3_scan/{stem}'
    A = arrays(decode_bag(mcap, f'{cache}/bag.pkl'))
    diag = load_diag(mcap, f'{cache}/diag.pkl')
    eps = staircase_episodes(A)
    print(f'bag {mcap}')
    print(f'/leg_cmd_executed {len(A["lt"])} samples over {A["lt"][-1] - A["lt"][0]:.1f} s')
    print(f'staircase episodes (>= 3 frozen runs while |target vel| > {VEL_MIN} rev/s): {len(eps)}')
    for e in eps:
        print(f'  leg {e["leg"]}  t0 {e["t0"]:.2f}  dur {e["dur"]:.2f} s  runs {e["n_runs"]}  '
              f'step med {e["step_med_mm"]:.2f} / max {e["step_max_mm"]:.2f} mm')
    rows = [(t, _clamp_census(kv)) for t, kv in diag['/link_status']]
    rows = [(t, c) for t, c in rows if c is not None]
    fws = sorted({kv.get('bridge_fw_version', '?') for _, kv in diag['/link_status']})
    print(f'bridge_fw_version rows seen: {fws}')
    if not rows:
        print('lead-clamp census: NEVER SEEN on /link_status (bridge older than FW 31, or no bridge)')
        return eps
    seg0 = rows[0]
    for (t_prev, c_prev), (t, c) in zip(rows, rows[1:]):
        if np.any(c[0] < c_prev[0]) or np.any(c[1] < c_prev[1]):
            _print_census(seg0, (t_prev, c_prev), 'segment before a bridge reboot')
            seg0 = (t, c)
    _print_census(seg0, rows[-1], 'census delta')
    return eps


def _print_census(a, b, label):
    dc, ds = b[1][0] - a[1][0], b[1][1] - a[1][1]
    print(f'{label} over {(b[0] - a[0]) * 1e-9:.1f} s (per leg 0..5): '
          f'clamp {" ".join(map(str, dc))} | stale {" ".join(map(str, ds))}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--emit-fixture', action='store_true', help='write tools/probes/data/lead_clamp_staircase_2026-10-24.json from the bag pickles')
    ap.add_argument('--emit-fixture-0815', action='store_true', help='write tools/probes/data/lead_clamp_staircase_2026-08-15.json from the 2026-08-15_10-08-06 mcap')
    ap.add_argument('--fixture-only', action='store_true', help='only replay the committed fixture through the twin')
    ap.add_argument('--scan', metavar='MCAP', help='P3 gate read-back of a Sitting bag; nothing else runs')
    a = ap.parse_args()
    if a.scan:
        scan(a.scan)
        return
    if a.emit_fixture:
        emit_fixture()
    if a.emit_fixture_0815:
        emit_fixture(FIXTURE_0815, WINDOWS_0815, '--emit-fixture-0815')
    if not a.fixture_only:
        report()
    fixture_report()


if __name__ == '__main__':
    main()
