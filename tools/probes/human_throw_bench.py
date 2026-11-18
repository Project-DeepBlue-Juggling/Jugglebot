#!/usr/bin/env python3
"""Score the ball tracker's human-throw detection against a ground truth cut from the same bag.

Why this exists
---------------
IDEA-16 (three-ball cascade with the owner as the second Hand) needs the robot to see a
human Throw. `BallTracker` has a parabolic detector for that (`detect_human_throws`, off by
default), but nothing had measured it: on 2026-09-15 it spawned 158 phantom tracks on a
static marker and on 2026-09-30 it registered no lobs. On 2026-11-02 the owner flew two
human-only Sittings with the robot moved aside (one-ball passes to where the robot lives,
then a three-ball cascade with one hand there). This probe is the benchmark the detector is
tuned against. Logbook: `logbook/2026-11-02-jb113-human-throw-bench.md`.

What it does, per bag
---------------------
1. **Ground truth, independent of the tracker.** Links every eligible `/mocap_data`
   marker (the tracker's own label exclusion) frame to frame with a gravity-aware
   constant-velocity prediction (so crossing balls in a cascade keep their identity),
   cuts free-flight windows where a fixed-g parabola fits (z within `--fit-tol-mm`,
   xy within 1.7x that), merges the pieces of one flight, and keeps rising flights as
   Throws. Each Throw gets its release, its descending crossing of the Catch plane
   (`--plane-z`, default the tracker's own landing plane) and its releasing hand
   (k-means on release xy, `--hands`).
2. **The real tracker, human detection on.** Replays the same frames through
   `tracker_bag_replay.make_tracker()` (the node's constructor, line for line) with
   `detect_human_throws` forced on. A tracker ball serves a Throw when at least five of
   its samples inside the flight lie within 30 mm of the truth parabola (a track carried
   over from an earlier flight counts: the catch would use it). A ball that serves no
   Throw is a **phantom**.
3. **Scores**: Throws served by a tracker ball, the lead before the plane at which its
   landing estimate first comes within 50 mm and 30 ms of the truth, the tracker's landing
   estimate error (xy mm, time ms) as it stands `--deadline-s` before the plane crossing,
   phantoms per minute, and two pattern-prior reads for the owner's design question:
   - landing scatter at the plane per hand-to-hand direction, and each landing against
     the mean of the previous five in its direction (what a running pattern mean predicts);
   - a **hand-region gate** what-if: a track passes when its state 30 ms into the flight
     (a phantom: its first state), carried back
     along a free-flight parabola to the median release height, lands within
     `--gate-r-mm` (xy) of a release-cluster centre. That gate is the cheapest pattern
     prior; this says how many phantoms it rejects and how many real tracks it keeps.

Strictly offline, read-only. Output: a timestamped markdown report under
`temp/probes/human_throw_bench/`.

Usage
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/human_throw_bench.py \\
        ~/Desktop/rosbags/2026-11-02_20-09-16_Human-single-3-tosses-near-Jugglebot \\
        ~/Desktop/rosbags/2026-11-02_21-04-06_Human-3-ball-juggling-near-Jugglebot

Limits
------
- Throws only the robot will see matter, but the probe scores every Throw in the volume.
- A flight the linker loses (a long occlusion) is absent from the truth, so the
  detection rate is over Throws the truth found, not every Throw flown, and the same
  lost flights count as phantoms: the phantom rate is an UPPER bound. The report splits
  phantoms by whether a truth release lies within 1.5 s (2026-11-02 cascade: the last
  12 s, mostly reverse cascade, are nearly absent from the truth).
- The hand clustering assumes `--hands` stationary hands; a moving juggler smears it.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO / 'tools' / 'probes'))
sys.path.insert(0, str(_REPO / 'ros_ws' / 'src' / 'jugglebot'))

import tracker_bag_replay as R                              # noqa: E402
from jugglebot.motion.skills import sites                   # noqa: E402

G = 9810.0
LINK_GATE_MM = 30.0
LINK_MAX_GAP_S = 0.03
MIN_SEG_S = 0.20
MIN_SEG_DZ_MM = 100.0
MERGE_GAP_S = 0.05
MERGE_JUMP_MM = 40.0
MIN_THROW_VZ = 500.0
GAP_S = 1.5          # a phantom this far from every truth release sits in a truth gap


def load_frames(bag: Path, with_wall: bool = False):
    """[(t, [xyz arrays], [labels])] of eligible markers, in bag order.

    `with_wall` appends the bag log time (epoch s) as a fourth element, for the
    `--t-start/--t-end` clock slices."""
    cache = _REPO / 'temp' / 'probes' / 'human_throw_bench' / 'frames' / f'{bag.name}.pkl'
    if with_wall and cache.exists():     # decoding a bag takes ~10 min on the loaded box
        import pickle
        with open(cache, 'rb') as fh:
            return pickle.load(fh)
    mc = R._find_mcap(bag)
    excl = R.make_tracker()
    frames = []
    for _topic, t_bag, msg in R._iter_bag(mc, ('/mocap_data',)):
        t = R._frame_time(msg, t_bag)
        pos = [np.array([m.position.x, m.position.y, m.position.z]) for m in msg.markers]
        lab = [m.label or '' for m in msg.markers]
        el = excl.eligible_markers(pos, lab)
        ids = {id(p) for p in el}
        keep = [i for i, p in enumerate(pos) if id(p) in ids]
        row = (t, [pos[i] for i in keep], [lab[i] for i in keep])
        frames.append(row + (t_bag,) if with_wall else row)
    if with_wall:
        import pickle
        cache.parent.mkdir(parents=True, exist_ok=True)
        with open(cache, 'wb') as fh:
            pickle.dump(frames, fh, protocol=4)
    return frames


def link(frames):
    """Greedy global nearest-neighbour linking against a gravity-aware prediction."""
    done, live = [], []
    for t, pos, _lab in frames:
        preds = []
        for tr in live:
            dt = t - tr['t'][-1]
            p = tr['p'][-1]
            if len(tr['t']) >= 2 and tr['t'][-1] > tr['t'][-2]:
                v = (tr['p'][-1] - tr['p'][-2]) / (tr['t'][-1] - tr['t'][-2])
                p = p + v * dt + np.array([0.0, 0.0, -0.5 * G * dt * dt])
            preds.append(p)
        pairs = sorted(
            (float(np.linalg.norm(pos[j] - preds[i])), i, j)
            for i in range(len(live)) for j in range(len(pos)))
        ti, tj, nxt = set(), set(), []
        for d, i, j in pairs:
            if d > LINK_GATE_MM:
                break
            if i in ti or j in tj:
                continue
            ti.add(i), tj.add(j)
            live[i]['t'].append(t), live[i]['p'].append(pos[j])
        for i, tr in enumerate(live):
            if i in ti or t - tr['t'][-1] <= LINK_MAX_GAP_S:
                nxt.append(tr)
            else:
                done.append(tr)
        for j, p in enumerate(pos):
            if j not in tj:
                nxt.append({'t': [t], 'p': [p]})
        live = nxt
    return done + live


def fit(T, P):
    tt = T - T[0]
    A = np.vstack([np.ones(len(T)), tt]).T
    zc = P[:, 2] + 0.5 * G * tt ** 2
    cz = np.linalg.lstsq(A, zc, rcond=None)[0]
    cxy = np.linalg.lstsq(A, P[:, :2], rcond=None)[0]
    return cz, cxy, np.abs(A @ cz - zc).max(), np.abs(A @ cxy - P[:, :2]).max()


def flights(tracks, tol):
    segs = []
    for tr in tracks:
        T, P = np.array(tr['t']), np.array(tr['p'])
        n = len(T)
        if n < 40 or np.ptp(P[:, 2]) < MIN_SEG_DZ_MM:
            continue
        i = 0
        while i < n - 40:
            j = i + 40
            _, _, rz, rxy = fit(T[i:j], P[i:j])
            if rz > tol or rxy > 1.7 * tol:
                i += 5
                continue
            while j < n:
                _, _, rz, rxy = fit(T[i:j + 5], P[i:j + 5])
                if rz > tol or rxy > 1.7 * tol:
                    break
                j = min(n, j + 5)
            if T[j - 1] - T[i] >= MIN_SEG_S and np.ptp(P[i:j, 2]) > MIN_SEG_DZ_MM:
                segs.append((T[i:j], P[i:j]))
            i = j
    segs.sort(key=lambda s: s[0][0])
    merged = []
    for T, P in segs:
        if merged:
            mT, mP = merged[-1]
            if 0 <= T[0] - mT[-1] < MERGE_GAP_S and np.linalg.norm(P[0] - mP[-1]) < MERGE_JUMP_MM:
                merged[-1] = (np.concatenate([mT, T]), np.vstack([mP, P]))
                continue
        merged.append((T, P))
    return merged


def throws(merged, plane_z):
    out = []
    for T, P in merged:
        cz, cxy, _, _ = fit(T, P)
        if cz[1] <= MIN_THROW_VZ:
            continue
        disc = cz[1] ** 2 - 2 * G * (plane_z - cz[0])
        tc = (cz[1] + np.sqrt(disc)) / G if disc > 0 else np.nan
        reaches = bool(np.isfinite(tc) and tc <= T[-1] - T[0] + 0.15)
        out.append(dict(t_rel=T[0], t_end=T[-1], p_rel=P[0], vz=cz[1], cz=cz, cxy=cxy,
                        apex=cz[0] + cz[1] ** 2 / (2 * G),
                        t_cross=T[0] + tc if reaches else np.nan,
                        xy_cross=cxy[0] + cxy[1] * tc if reaches else None))
    return out


def kmeans(X, k, iters=50):
    c = X[np.argsort(X[:, 0])[np.linspace(0, len(X) - 1, k).astype(int)]].copy()
    for _ in range(iters):
        lab = np.argmin(((X[:, None, :] - c[None]) ** 2).sum(-1), axis=1)
        c = np.array([X[lab == i].mean(0) if np.any(lab == i) else c[i] for i in range(k)])
    order = np.argsort(c[:, 0])
    remap = {int(o): i for i, o in enumerate(order)}
    return c[order], np.array([remap[int(v)] for v in lab])


def human_throw_box():
    """The BUG-92 Ball Butler frame box, computed exactly as `ball_tracker_node` does."""
    hw = R.hw
    hx = (float(hw.TRACKING_HUMAN_THROW_EXCLUSION_X_MIN_MM), float(hw.TRACKING_HUMAN_THROW_EXCLUSION_X_MAX_MM))
    hz = (float(hw.TRACKING_HUMAN_THROW_EXCLUSION_Z_MIN_MM), float(hw.TRACKING_HUMAN_THROW_EXCLUSION_Z_MAX_MM))
    return (hx[0], hx[1], hz[0], hz[1]) if hx[0] <= hx[1] and hz[0] <= hz[1] else None


def replay(frames, emitters=None, box=False, flights_seen=None, events_out=None):
    """Replay through the real tracker. The default call is the original replay (no BUG-92
    box, no emitter). `emitters` ({name: HumanThrowEventEmitter}) are stepped on every frame
    with the tracker's own balls and Fits and their events appended to `events_out[name]`;
    `box` applies the node's BUG-92 exclusion box; `flights_seen` ({(id, seq): dict}) records
    every flight's start speed whether or not an emitter accepts it."""
    R.hw.TRACKING_DETECT_HUMAN_THROWS = True
    trk = R.make_tracker()
    assert trk.detect_human_throws
    if box:
        trk.human_throw_exclusion_box = human_throw_box()
    hist = {}
    log = events_out if events_out is not None else {}
    for k in (emitters or {}):
        log.setdefault(k, [])
    for t, pos, lab in frames:
        balls = trk.process_frame(pos, t, lab)
        for b in balls:
            h = hist.setdefault(b.id, {'t0': t, 'p0': np.array(b.position, float),
                                       'v0': np.array(b.velocity, float), 'trace': []})
            h['trace'].append((t, np.array(b.position, float), float(b.landing_time or 0.0),
                               np.array(b.landing_position, float), np.array(b.velocity, float)))
        if emitters:
            fits = trk._flight_fits
            for name, em in emitters.items():
                log[name].extend(em.step(balls, fits, t))
            if flights_seen is not None:
                for b in balls:
                    f = fits.get(b.id)
                    if f is None or b.source != 'human_throw':
                        continue
                    d = flights_seen.setdefault((b.id, f.flight_seq), {'speed': None, 'start': None})
                    if d['speed'] is None and f.start_speed_mm_s is not None:
                        d['speed'] = float(f.start_speed_mm_s)
                    if d['start'] is None and f.flight_start_t is not None:
                        d['start'] = float(f.flight_start_t)
    return hist


def truth_at(x, t):
    tt = t - x['t_rel']
    return np.array([*(x['cxy'][0] + x['cxy'][1] * tt), x['cz'][0] + x['cz'][1] * tt - 0.5 * G * tt * tt])


def origin(p, v, z_hand):
    """Back-extrapolate a state along a free-flight parabola to z_hand; None if it never was
    there (a static or falling start)."""
    a, b, c = -0.5 * G, v[2], p[2] - z_hand
    disc = b * b - 4 * a * c
    if disc < 0 or v[2] <= 0:
        return None
    taus = [(-b + s * np.sqrt(disc)) / (2 * a) for s in (1, -1)]
    neg = [tau for tau in taus if tau <= 0]
    if not neg:
        return None
    tau = max(neg)
    return p[:2] + v[:2] * tau


def serve_map(hist, th):
    """{throw index: [(first serving sample time, tracker id, serving samples)]}.

    A tracker ball serves a Throw when >= 5 of its samples inside the flight lie within
    MATCH_MM of the truth parabola; detection = its first such sample (a carried-over
    track counts, which is what the catch would use)."""
    serve = {}
    for bid, h in hist.items():
        for k, x in enumerate(th):
            near = [s for s in h['trace'] if x['t_rel'] <= s[0] <= x['t_end']
                    and np.linalg.norm(s[1] - truth_at(x, s[0])) <= 30.0]
            if len(near) >= 5:
                serve.setdefault(k, []).append((near[0][0], bid, near))
    return serve


def analyse(bag, args, plane_z):
    frames = load_frames(bag)
    t0 = frames[0][0]
    span = frames[-1][0] - t0
    th = throws(flights(link(frames), args.fit_tol_mm), plane_z)
    if len(th) >= args.hands:
        centres, hand = kmeans(np.array([x['p_rel'][:2] for x in th]), args.hands)
    else:
        centres, hand = np.zeros((0, 2)), np.zeros(len(th), int)
    for x, h in zip(th, hand):
        x['hand'] = int(h)
    hist = replay(frames)

    serve = serve_map(hist, th)
    served_ids = {bid for v in serve.values() for _, bid, _ in v}
    phantoms = [(bid, h) for bid, h in hist.items() if bid not in served_ids]

    rows = []
    for k, x in enumerate(th):
        r = dict(k=k, x=x, det=None, err_xy=None, err_t=None, bid=None, lead=None, s0=None)
        if k in serve:
            t_det, bid, near = min(serve[k], key=lambda e: e[0])
            r['det'], r['bid'] = t_det - x['t_rel'], bid
            # the serving state 30 ms into the flight (the gate's input for a carried-over track)
            r['s0'] = next((s for s in near if s[0] >= x['t_rel'] + 0.03), near[-1])
            if np.isfinite(x['t_cross']):
                good = [s for s in near if s[2] > 0
                        and np.linalg.norm(s[3][:2] - x['xy_cross']) <= 50.0
                        and abs(s[2] - x['t_cross']) <= 0.030]
                if good:
                    r['lead'] = x['t_cross'] - good[0][0]
                cut = x['t_cross'] - args.deadline_s
                est = [s for s in near if s[0] <= cut and s[2] > 0]
                if est:
                    r['err_xy'] = float(np.linalg.norm(est[-1][3][:2] - x['xy_cross']))
                    r['err_t'] = (est[-1][2] - x['t_cross']) * 1000.0
        rows.append(r)

    z_hand = float(np.median([x['p_rel'][2] for x in th])) if th else 0.0

    def in_gate(p, v):
        o = origin(p, v, z_hand)
        return (o is not None and len(centres) > 0
                and np.min(np.linalg.norm(centres - o, axis=1)) <= args.gate_r_mm)

    return dict(bag=bag, span=span, rows=rows, centres=centres, phantoms=phantoms,
                hist=hist, in_gate=in_gate, t0=t0, z_hand=z_hand)


def report(res, args, plane_z):
    L = []
    rows, span, t0 = res['rows'], res['span'], res['t0']
    n = len(rows)
    det = [r for r in rows if r['det'] is not None]
    cross = [r for r in rows if np.isfinite(r['x']['t_cross'])]
    good = [r for r in cross if r['err_xy'] is not None and r['err_xy'] <= 50 and abs(r['err_t']) <= 30]
    leads = [r['lead'] for r in cross if r['lead'] is not None]
    L.append(f"## {res['bag'].name}\n")
    L.append(f"- span {span:.1f} s; Throws in the truth: **{n}** ({len(cross)} cross the plane z={plane_z:.0f} mm descending)")
    L.append("- hand centres (release xy, mm): " + ', '.join(
        f"{chr(65 + i)} ({c[0]:.0f}, {c[1]:.0f})" for i, c in enumerate(res['centres'])))
    L.append(f"- served by a tracker ball (5+ samples within 30 mm of the flight): **{len(det)}/{n}**; "
             f"{sum(1 for r in det if r['det'] <= 0.0)} of them by a track already on the ball at release")
    if leads:
        L.append(f"- first landing estimate within 50 mm and 30 ms: {len(leads)}/{len(cross)} Throws; "
                 f"lead before the plane median {np.median(leads):.2f} s, p10 {np.percentile(leads, 10):.2f} s, "
                 f"**{sum(1 for v in leads if v >= args.deadline_s)} with {args.deadline_s:.2f} s or more**, "
                 f"{sum(1 for v in leads if v >= 0.5)} with 0.5 s or more")
    L.append(f"- landing estimate within 50 mm and 30 ms at {args.deadline_s:.2f} s before the plane: "
             f"**{len(good)}/{len(cross)}**")
    ph = res['phantoms']
    rel = [r['x']['t_rel'] for r in rows]
    covered = [(bid, h) for bid, h in ph if any(abs(h['t0'] - t) <= GAP_S for t in rel)]
    L.append(f"- phantoms: **{len(ph)}** ({len(ph) / span * 60:.1f} per minute, an upper bound); "
             f"{len(covered)} start within {GAP_S:.1f} s of a truth release "
             f"({len(covered) / span * 60:.1f} per minute); the rest fall in stretches the truth "
             f"does not cover and may be real flights it lost")
    gp = sum(1 for _, h in ph if res['in_gate'](h['p0'], h['v0']))
    gnone = sum(1 for _, h in ph if origin(h['p0'], h['v0'], res['z_hand']) is None)
    gd = sum(1 for r in det if res['in_gate'](r['s0'][1], r['s0'][4]))
    L.append(f"- hand-region gate (track state 30 ms into the flight, or a phantom's first state, back-extrapolated to the median release height "
             f"{res['z_hand']:.0f} mm lands within {args.gate_r_mm:.0f} mm of a hand centre): "
             f"passes {gp}/{len(ph)} phantoms ({gnone} of the rejected never rise: v_z <= 0 or no "
             f"parabola reaches release height) and {gd}/{len(det)} served Throws")
    L.append('')
    L.append('| direction | n | landing mean (mm) | sd x, y (mm) | vs mean of previous 5: median, p90 (mm) | flight to plane (s) |')
    L.append('|---|---|---|---|---|---|')
    for a in range(len(res['centres'])):
        for b in range(len(res['centres'])):
            S = [r['x'] for r in cross if r['x']['hand'] == a
                 and int(np.argmin(np.linalg.norm(res['centres'] - r['x']['xy_cross'], axis=1))) == b]
            if len(S) < 3:
                continue
            xy = np.array([s['xy_cross'] for s in S])
            tf = np.array([s['t_cross'] - s['t_rel'] for s in S])
            e = [np.linalg.norm(xy[i] - xy[i - 5:i].mean(0)) for i in range(5, len(S))]
            es = f"{np.median(e):.0f}, {np.percentile(e, 90):.0f}" if e else '-'
            L.append(f"| {chr(65 + a)} to {chr(65 + b)} | {len(S)} | ({xy[:, 0].mean():.0f}, {xy[:, 1].mean():.0f}) "
                     f"| {xy[:, 0].std():.0f}, {xy[:, 1].std():.0f} | {es} | {tf.min():.2f}-{tf.max():.2f} |")
    L.append('')
    L.append('| # | t (s) | hand | release xyz (mm) | apex (mm) | to plane (s) | served from (ms after release) | good-estimate lead (s) | est err xy (mm), t (ms) |')
    L.append('|---|---|---|---|---|---|---|---|---|')
    for r in rows:
        x = r['x']
        tp = x['t_cross'] - x['t_rel'] if np.isfinite(x['t_cross']) else float('nan')
        d = f"{r['det'] * 1000:.0f}" if r['det'] is not None else 'MISSED'
        ld = f"{r['lead']:.2f}" if r['lead'] is not None else '-'
        e = f"{r['err_xy']:.0f}, {r['err_t']:+.0f}" if r['err_xy'] is not None else '-'
        p = x['p_rel']
        L.append(f"| {r['k']} | {x['t_rel'] - t0:.2f} | {chr(65 + x['hand'])} | ({p[0]:.0f}, {p[1]:.0f}, {p[2]:.0f}) "
                 f"| {x['apex']:.0f} | {tp:.2f} | {d} | {ld} | {e} |")
    L.append('')
    L.append('Phantoms (t s, first position mm): ' + '; '.join(
        f"{h['t0'] - t0:.2f} ({h['p0'][0]:.0f}, {h['p0'][1]:.0f}, {h['p0'][2]:.0f})" for _, h in ph))
    L.append('')
    return L


# ----------------------------------------------------------------------------------------
# --events: score the human Throw EVENT stream (PWAY-11 P1 gate, design-human-catch.md s.5)
# ----------------------------------------------------------------------------------------

def _pct(v, q):
    v = [x for x in v if x is not None and np.isfinite(x)]
    return float(np.percentile(v, q)) if v else float('nan')


def _fmt(v, f='{:.2f}'):
    return f.format(v) if np.isfinite(v) else '-'


def match_flight(track_id, flight_start, th, serve_ids, pad_s=0.10):
    """Index of the truth Throw a flight (track id, first admitted sample time) belongs to, or
    None: the tracker id serves that Throw and the flight starts between `pad_s` before its
    release and its end. Independent of the landing the emitter reported (no circularity)."""
    best = None
    for k, x in enumerate(th):
        if track_id in serve_ids.get(k, ()) and x['t_rel'] - pad_s <= flight_start <= x['t_end']:
            d = abs(flight_start - x['t_rel'])
            if best is None or d < best[0]:
                best = (d, k)
    return None if best is None else best[1]


def score_events(events, th, serve_ids, span_s, deadline_s=0.30, gap_s=GAP_S):
    """Score one event stream against the truth Throws. Pure: no tracker needed.

    `events` are `HumanThrowEvent`s (any object with track_id, flight_seq, revision, stamp,
    flight_start, landing_position, landing_time, final, start_speed_mm_s); `th` the truth
    Throws from `throws()`; `serve_ids` {throw index: ids of tracker balls serving it}.
    A stream is (track_id, flight_seq). Returns per-Throw rows, phantom streams and a summary."""
    streams = {}
    for e in events:
        streams.setdefault((e.track_id, e.flight_seq), []).append(e)
    for v in streams.values():
        v.sort(key=lambda e: (e.stamp, e.revision))
    by_throw, phantoms = {}, []
    rel = [x['t_rel'] for x in th]
    for key, evs in streams.items():
        k = match_flight(key[0], evs[0].flight_start, th, serve_ids)
        if k is None:
            idle = not any(abs(evs[0].flight_start - t) <= gap_s for t in rel)
            phantoms.append(dict(key=key, t=evs[0].stamp, idle=idle, n=len(evs),
                                 speed=evs[0].start_speed_mm_s))
        else:
            by_throw.setdefault(k, []).append(evs)
    rows = []
    for k, x in enumerate(th):
        r = dict(k=k, served=False, lead=None, err_xy=None, err_t=None, n_rev=0, frozen_only=False,
                 last_err_xy=None, last_err_t=None, seq=None)
        cross = bool(np.isfinite(x['t_cross']))
        if k in by_throw:
            evs = min(by_throw[k], key=lambda v: v[0].stamp)
            e0 = evs[0]
            r.update(served=True, n_rev=len(evs), seq=e0.flight_seq,
                     frozen_only=bool(e0.revision == 0 and e0.final))
            if cross:
                r['lead'] = x['t_cross'] - e0.stamp
                r['err_xy'] = float(np.linalg.norm(np.asarray(e0.landing_position)[:2] - x['xy_cross']))
                r['err_t'] = (e0.landing_time - x['t_cross']) * 1000.0
                pre = [e for e in evs if e.stamp <= x['t_cross'] - deadline_s]
                if pre:
                    e = pre[-1]
                    r['last_err_xy'] = float(np.linalg.norm(np.asarray(e.landing_position)[:2] - x['xy_cross']))
                    r['last_err_t'] = (e.landing_time - x['t_cross']) * 1000.0
        r['cross'] = cross
        rows.append(r)
    cross_rows = [r for r in rows if r['cross']]
    served = [r for r in rows if r['served']]
    sc = [r for r in cross_rows if r['served']]
    good = [r for r in sc if abs(r['err_t']) <= 30.0 and r['err_xy'] <= 50.0]
    lastgood = [r for r in cross_rows if r['last_err_xy'] is not None
                and abs(r['last_err_t']) <= 30.0 and r['last_err_xy'] <= 50.0]
    ph_idle = [p for p in phantoms if p['idle']]
    summ = dict(
        n_throws=len(th), n_cross=len(cross_rows), n_served=len(served), n_served_cross=len(sc),
        lead_p10=_pct([r['lead'] for r in sc], 10), lead_p50=_pct([r['lead'] for r in sc], 50),
        lead_p90=_pct([r['lead'] for r in sc], 90),
        err_xy_p50=_pct([r['err_xy'] for r in sc], 50), err_xy_p90=_pct([r['err_xy'] for r in sc], 90),
        err_t_abs_p50=_pct([abs(r['err_t']) for r in sc], 50),
        err_t_abs_p90=_pct([abs(r['err_t']) for r in sc], 90),
        n_good_first=len(good), n_good_last=len(lastgood),
        n_last=sum(1 for r in cross_rows if r['last_err_xy'] is not None),
        last_xy_p50=_pct([r['last_err_xy'] for r in cross_rows], 50),
        last_xy_p90=_pct([r['last_err_xy'] for r in cross_rows], 90),
        last_t_p50=_pct([abs(r['last_err_t']) for r in cross_rows if r['last_err_t'] is not None], 50),
        last_t_p90=_pct([abs(r['last_err_t']) for r in cross_rows if r['last_err_t'] is not None], 90),
        n_frozen_only=sum(1 for r in served if r['frozen_only']),
        n_phantom=len(phantoms), n_phantom_idle=len(ph_idle),
        n_phantom_juggle=len(phantoms) - len(ph_idle),
        phantom_per_min=len(phantoms) / span_s * 60.0 if span_s > 0 else float('nan'),
        n_events=len(events))
    return dict(rows=rows, phantoms=phantoms, summary=summ)


def classify_flights(flights_seen, th, serve_ids, min_speed=500.0):
    """One dict per (track id, flight_seq) the tracker produced: its start speed, the truth
    Throw it belongs to (or None) and whether the D1 rule would reject it."""
    out = []
    for (tid, seq), d in sorted(flights_seen.items()):
        if d['start'] is None:
            continue
        k = match_flight(tid, d['start'], th, serve_ids)
        out.append(dict(id=tid, seq=seq, speed=d['speed'], k=k, start=d['start'],
                        rejected=bool(d['speed'] is not None and d['speed'] < min_speed)))
    return out


def _fit_only_config(base):
    from jugglebot.tracking.human_throw_event import HumanThrowEventConfig as C
    return C(min_start_speed_mm_s=base.min_start_speed_mm_s, rms_mm=1e9)


def make_emitter(name, cfg):
    """The D2-i 'fit valid only' variant cannot be expressed in the config (`stable_ms` is
    both the minimum age gap and the landing-time agreement, so 0 demands identical times and
    a huge value never fires): the probe skips the stability test in a subclass instead."""
    from jugglebot.tracking.human_throw_event import HumanThrowEventEmitter as E
    if name == 'D2-i fit-only':
        class FitOnly(E):
            def _stable(self, *a, **k):
                return True
        return FitOnly(cfg)
    return E(cfg)


def d2_variants(base=None):
    """Emitter configs the P1 gate compares. 'default' is the PROPOSED D1/D2 set."""
    from jugglebot.tracking.human_throw_event import HumanThrowEventConfig as C
    base = base or C()
    v = {'default': base,
         'D2-i fit-only': _fit_only_config(base),
         'D2-iii 20mm/10ms': C(min_start_speed_mm_s=base.min_start_speed_mm_s, stable_mm=20.0,
                               stable_ms=10.0),
         'no-speed-rule': C(min_start_speed_mm_s=0.0),
         'speed 300': C(min_start_speed_mm_s=300.0)}
    for rms in (4.0, 6.0, 10.0):
        for agr in (20.0, 30.0, 50.0):
            nm = f'rms{rms:g}/agree{agr:g}'
            v[nm] = C(min_start_speed_mm_s=base.min_start_speed_mm_s, rms_mm=rms, stable_mm=agr)
    return v


def _parse_clock(v, wall0, t_first):
    """A `--t-start/--t-end` value -> frame time. `HH:MM:SS[.f]` is the local clock of the bag
    log time; a bare number is seconds from the first frame."""
    if v is None:
        return None
    if ':' in str(v):
        h, m, sec = str(v).split(':')
        base = time.localtime(wall0)
        mid = time.mktime((base.tm_year, base.tm_mon, base.tm_mday, int(h), int(m), 0,
                           0, 0, base.tm_isdst))
        return t_first + (mid + float(sec) - wall0)
    return t_first + float(v)


def analyse_events(bag, args, plane_z):
    import pickle
    frames4 = load_frames(bag, with_wall=True)
    t_first, wall0 = frames4[0][0], frames4[0][3]
    lo = _parse_clock(args.t_start, wall0, t_first)
    hi = _parse_clock(args.t_end, wall0, t_first)
    frames = [f[:3] for f in frames4 if (lo is None or f[0] >= lo) and (hi is None or f[0] <= hi)]
    del frames4
    t0 = frames[0][0]
    span = frames[-1][0] - t0
    th = throws(flights(link(frames), args.fit_tol_mm), plane_z)
    variants = d2_variants()
    ems = {n: make_emitter(n, c) for n, c in variants.items()}
    flights_seen, ev = {}, {}
    hist = replay(frames, emitters=ems, box=not args.no_box, flights_seen=flights_seen, events_out=ev)
    serve = serve_map(hist, th)
    serve_ids = {k: {bid for _, bid, _ in v} for k, v in serve.items()}
    scores = {n: score_events(ev[n], th, serve_ids, span, args.deadline_s) for n in variants}
    fl = classify_flights(flights_seen, th, serve_ids)
    rejections = {n: dict(ems[n].rejections) for n in variants}
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    tag = bag.name + ('_nobox' if args.no_box else '')
    ev_dump = {n: [dict(vars(e), landing_position=list(map(float, e.landing_position)),
                        landing_velocity=list(map(float, e.landing_velocity))) for e in ev[n]]
               for n in ('default',)}
    with open(out / f"{tag}_events.pkl", 'wb') as fh:
        pickle.dump(dict(events=ev_dump, truth=[{k: v for k, v in x.items()} for x in th],
                         flights=fl, t0=t0, span=span), fh)
    return dict(bag=bag, t0=t0, span=span, th=th, scores=scores, flights=fl,
                rejections=rejections, variants=variants, box=not args.no_box,
                first_throw=min((x['t_rel'] for x in th), default=None))


def report_events(res, args, plane_z):
    L = []
    s = res['scores']['default']['summary']
    sp = res['span']
    L.append(f"## {res['bag'].name} (events; BUG-92 box {'ON' if res['box'] else 'OFF'}; span {sp:.1f} s; "
             f"truth Throws {s['n_throws']}, crossing {s['n_cross']})\n")
    hdr = ('| variant | Throws served (any / crossing) | frozen-only first events | first lead p10 / p50 / p90 (s) '
           '| first err xy p50 / p90 (mm) | first err t p50 / p90 (ms) | first good (50 mm, 30 ms) '
           '| last-before-0.30 s err xy p50 / p90 (mm), t p50 / p90 (ms), good | phantom streams (idle / juggling), per min |')
    L += [hdr, '|' + '---|' * 9]
    for n, sc in res['scores'].items():
        m = sc['summary']
        L.append(f"| {n} | {m['n_served']}/{m['n_throws']} ({m['n_served_cross']}/{m['n_cross']}) | {m['n_frozen_only']} "
                 f"| {_fmt(m['lead_p10'])} / {_fmt(m['lead_p50'])} / {_fmt(m['lead_p90'])} "
                 f"| {_fmt(m['err_xy_p50'], '{:.0f}')} / {_fmt(m['err_xy_p90'], '{:.0f}')} "
                 f"| {_fmt(m['err_t_abs_p50'], '{:.0f}')} / {_fmt(m['err_t_abs_p90'], '{:.0f}')} "
                 f"| {m['n_good_first']}/{m['n_served_cross']} "
                 f"| {_fmt(m['last_xy_p50'], '{:.0f}')} / {_fmt(m['last_xy_p90'], '{:.0f}')}, "
                 f"{_fmt(m['last_t_p50'], '{:.0f}')} / {_fmt(m['last_t_p90'], '{:.0f}')}, {m['n_good_last']}/{m['n_last']} "
                 f"| {m['n_phantom']} ({m['n_phantom_idle']} / {m['n_phantom_juggle']}), {m['phantom_per_min']:.1f} |")
    L.append('')
    L.append('Rejections by reason (default emitter; phantom_start_speed counts flights, the rest evaluations): '
             + ', '.join(f"{k} {v}" for k, v in sorted(res['rejections']['default'].items())))
    fl = res['flights']
    for seq_name, sel in (('flight_seq 0', [f for f in fl if f['seq'] == 0]),
                          ('flight_seq >= 1', [f for f in fl if f['seq'] >= 1])):
        real = [f for f in sel if f['k'] is not None]
        fake = [f for f in sel if f['k'] is None]
        rs = [f['speed'] for f in real]
        L.append(f"- {seq_name}: {len(sel)} flights; matched to a real Throw {len(real)} "
                 f"(start speed p10/p50/p90 {_fmt(_pct(rs, 10), '{:.0f}')}/{_fmt(_pct(rs, 50), '{:.0f}')}/{_fmt(_pct(rs, 90), '{:.0f}')} mm/s, "
                 f"{sum(1 for f in real if f['rejected'])} below 500 = real Throws the D1 rule would reject); "
                 f"unmatched {len(fake)} ({sum(1 for f in fake if f['rejected'])} rejected by D1, "
                 f"{sum(1 for f in fake if not f['rejected'])} pass D1 and are phantom-event candidates)")
    L.append('')
    L.append('Default emitter, per Throw (lead = plane crossing minus first event stamp):\n')
    L.append('| # | t (s) | seq | events | first lead (s) | first err xy (mm), t (ms) | frozen-only | last-before-0.30 s err xy, t |')
    L.append('|---|---|---|---|---|---|---|---|')
    for r, x in zip(res['scores']['default']['rows'], res['th']):
        if not r['served']:
            L.append(f"| {r['k']} | {x['t_rel'] - res['t0']:.2f} | - | 0 | NO EVENT | - | - | - |")
            continue
        e = f"{r['err_xy']:.0f}, {r['err_t']:+.0f}" if r['err_xy'] is not None else '-'
        la = f"{r['last_err_xy']:.0f}, {r['last_err_t']:+.0f}" if r['last_err_xy'] is not None else '-'
        L.append(f"| {r['k']} | {x['t_rel'] - res['t0']:.2f} | {r['seq']} | {r['n_rev']} | "
                 f"{_fmt(r['lead']) if r['lead'] is not None else '-'} | {e} | {'yes' if r['frozen_only'] else ''} | {la} |")
    L.append('')
    ph = res['scores']['default']['phantoms']
    L.append('Phantom streams (default): ' + '; '.join(
        f"id {p['key'][0]}/{p['key'][1]} t {p['t'] - res['t0']:.1f} {'IDLE' if p['idle'] else 'near-truth'} v0 {p['speed']:.0f}"
        for p in ph))
    L.append('')
    return L


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bags', nargs='+', help='rosbag directories (or .mcap files)')
    ap.add_argument('--plane-z', type=float, default=None,
                    help='Catch plane z, mm (default: the tracker landing plane)')
    ap.add_argument('--deadline-s', type=float, default=0.30,
                    help='score the landing estimate this long before the plane crossing')
    ap.add_argument('--hands', type=int, default=2, help='release clusters (stationary hands)')
    ap.add_argument('--fit-tol-mm', type=float, default=6.0, help='parabola fit tolerance, z')
    ap.add_argument('--gate-r-mm', type=float, default=200.0, help='hand-region gate radius, xy')
    ap.add_argument('--events', action='store_true',
                    help='score the Human Throw EVENT stream (HumanThrowEventEmitter, PWAY-11 P1 gate): '
                         'real tracker with the BUG-92 box, proposed D1/D2 plus the D2 sweep')
    ap.add_argument('--t-start', default=None,
                    help='--events slice start: seconds from the first frame, or local HH:MM:SS of the bag log time')
    ap.add_argument('--t-end', default=None, help='--events slice end (same forms)')
    ap.add_argument('--no-box', action='store_true', help='--events: leave the BUG-92 exclusion box OFF')
    ap.add_argument('--out-dir', default=str(_REPO / 'temp' / 'probes' / 'human_throw_bench'))
    args = ap.parse_args(argv)
    plane_z = args.plane_z if args.plane_z is not None else float(sites.CATCH_CUP_Z_MM)
    L = [f"# Human-throw bench {time.strftime('%Y-%m-%d %H:%M:%S')}\n",
         f"`python tools/probes/human_throw_bench.py {' '.join(str(b) for b in args.bags)}` "
         f"(plane {plane_z:.0f} mm, deadline {args.deadline_s:.2f} s, fit tol {args.fit_tol_mm:.0f} mm)\n"]
    for b in args.bags:
        if args.events:
            L += report_events(analyse_events(Path(b).expanduser(), args, plane_z), args, plane_z)
        else:
            L += report(analyse(Path(b).expanduser(), args, plane_z), args, plane_z)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"human_throw_bench{'_events' if args.events else ''}_{time.strftime('%Y%m%d_%H%M%S')}.md"
    path.write_text('\n'.join(L))
    print('\n'.join(l for l in L if args.events or not l.startswith('| ') or l.startswith('| direction') or ' to ' in l))
    print(f"\nreport: {path}")


if __name__ == '__main__':
    main()
