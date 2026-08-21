#!/usr/bin/env python3
"""Is a mocap object's tracked point BIASED, or is a FORCE acting on it?

    plans/active/critical-point-ilc.md § Phase 1, entry condition E-1
    logbook/2026-08-12-critical-point-ilc-g3-and-phase0.md § 0c

THE ONE MEASUREMENT THAT SEPARATES THEM
---------------------------------------
Write a tracked ballistic arc as a function of ``tau = t - t_apex``. The arc's
HEIGHT is an even function of tau; that is the whole lever.

* a **measurement bias** that depends on the object's position — the tracked
  point is the centroid of the visible reflective cap, and which part is visible
  depends on viewing geometry and occlusion — enters a LATERAL channel as an
  **EVEN** function of tau, because ``z`` is even. A per-branch line fit then
  reads ``v_true + <db/dz . vz>`` ascending and ``v_true - <db/dz . |vz|>``
  descending: an antisymmetric branch split whose MEAN is the truth.
* an **aerodynamic force** cannot do that. Drag flips with ``v``; Magnus
  (``omega x v``) flips with ``vz`` at the apex. Both put their DISPLACEMENT
  contribution in the **ODD** channel. No aerodynamic term is even in tau except
  a steady room draft, which this probe bounds separately.
* a **rotating asymmetry** (a discrete marker on a spinning ball) is a smooth
  function of TIME with a per-arc phase, so its even/odd split is arbitrary and
  its SHAPE varies arc to arc. It also shows up as periodicity in the residual.

So, per arc::

    E(tau) = (p(+tau) + p(-tau)) / 2       -> the EVEN part: b(z), in mm
    O(tau) = (p(+tau) - p(-tau)) / (2 tau) -> the ODD part: true lateral velocity

``E(tau) - E(0)`` IS the bias curve ``b(z) - b(z_apex)``, in millimetres, with no
model of the camera geometry required.

WHAT THE PROBE THEN DOES WITH IT
--------------------------------
1. **The parity verdict** — even amplitude and its cross-arc repeatability
   (pooled ``b(z)`` profile, sd, pairwise r) against the odd/cubic term,
   restated as EQUIVALENT LATERAL ACCELERATIONS so the two are comparable.
2. **The held-out test, which is the load-bearing one** — correct each arc with
   a bias profile fitted on OTHER arcs (leave-one-out, and cross-session) and
   re-measure the branch delta. A bias that transfers is a property of the room;
   one that does not is a property of the fit.
3. **The whole-arc leak** — ``cov(tau, E)/var(tau)``, exactly zero for coverage
   symmetric about the apex. This is what licenses a whole-arc estimator where a
   per-branch one is unusable, and it is the quantity
   ``toss_record_miner.COVERAGE_ASYM_MAX_S`` gates.
4. **Controls** — a rig control (ordinary point markers on the same captures), a
   blob-split census (does the tracked object reconstruct as ONE marker?), a
   periodogram of the detrended residual, and an aerodynamic scale bound derived
   from the arc's own VERTICAL odd channel, which is pure drag.

Promoted from ``temp/probes/e1_artefact/probe_e1_artefact.py`` (2026-08-13) —
the E-1 verdict's numbers had no committed provenance while it lived under a
gitignored path. Underpins ``toss_record_miner.mine_arc``'s whole-arc lateral
estimators and the ``coverage_asym_s`` gate, and the E-1 cases in
``toss_record_miner.self_check``.

    python tools/probes/mocap_parity_bias.py --self-check
    python tools/probes/mocap_parity_bias.py --bag 2026-08-10_16-30-44
    python tools/probes/mocap_parity_bias.py --bag A --bag B --bag C --plot

Strictly offline and read-only: opens ``.mcap`` files, nothing else. Exit 0 on
success, 1 on a failed self-check or an unreadable bag.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import pickle
import sys
import warnings
from collections import defaultdict
from datetime import datetime

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))          # tools/probes
_REPO = os.path.dirname(os.path.dirname(_HERE))             # repo root
for _p in (_HERE, _REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import toss_record_miner as M                                    # noqa: E402

DEFAULT_ROOT = os.path.expanduser('~/Desktop/rosbags')
OUT_DIR = os.path.join(_REPO, 'temp', 'probes')
CACHE_DIR = os.path.join(OUT_DIR, 'mocap_parity_cache')

#: The three mocap-bearing bags the E-1 verdict was taken on. A default, not a
#: constraint — any bag with ``/mocap_data`` + ``/throw_announcements`` works.
DEFAULT_BAGS = ('2026-08-10_16-30-44', '2026-08-12_17-45-44',
                '2026-08-12_19-02-52')

#: Ball radius, mm. NOT in ``hardware_config.yaml`` (checked: only
#: ``ball_joint_offset`` and the tracker's match thresholds are there). 35 mm is
#: the figure the repo states twice — ``logbook/2026-06-27-throw-aim-band-limit-
#: and-closed-loop-catch.md`` and ``ros_ws/docs/levelling_frame.md`` § cup
#: tolerance. Every "bounded by a ball radius" claim is against THIS assumed
#: number; the blob-split census is the attempt to measure it from the data.
BALL_RADIUS_MM = 35.0
#: Assumed, and the aero bounds scale linearly with it — reported, never hidden.
BALL_MASS_KG = 0.100

#: Association gate: a marker further than this from the current arc estimate is
#: not this object. 100 mm is the tracker's own ``match_threshold_base_mm``.
ASSOC_GATE_MM = 100.0
#: Rows closer than this in time are the same mocap frame (200 Hz nominal).
FRAME_EPS_S = 1e-4
#: Interpolation across a mocap gap wider than this is refused.
MAX_INTERP_GAP_S = 0.030
#: Minimum |tau| for the odd part — ``O = dp/(2 tau)`` blows up at the apex.
MIN_TAU_S = 0.05
#: |tau| above which the odd part is read as "the lateral velocity".
ODD_READ_TAU_S = 0.15
#: Height-grid pitch for the pooled ``b(z)`` profile, mm.
Z_GRID_MM = 25.0
#: Cache read window half-width, mm — WIDER than the miner's lateral gate,
#: because the censuses need the LABELLED rig markers the miner discards.
WIDE_MM = 900.0
#: Companion-marker search radius for the blob-split census, mm.
COMPANION_MM = 140.0


# ═════════════════════════════════════════════════════════════════════════════
# Bag cache — the heavy decode, paid once
# ═════════════════════════════════════════════════════════════════════════════

def _stamp_s(t) -> float:
    return float(t.sec) + float(t.nanosec) * 1e-9


def build_cache(bag: str, root: str = DEFAULT_ROOT) -> dict:
    """Decode one bag into the light structures this analysis needs.

    Deliberately not ``toss_record_miner.read_bag``: that routes LABELLED
    markers into ``fixture_cells`` and throws their positions away, and both
    censuses below need them (a labelled rig marker beside the arc answers a
    different question from an unlabelled companion inside a ball diameter).
    The lateral gate is widened to :data:`WIDE_MM` for the same reason.
    """
    path = os.path.join(root, bag)
    files = sorted(glob.glob(os.path.join(path, '*.mcap')))
    if not files:
        raise IOError('no .mcap in {}'.format(path))
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    anns, hand_offsets = [], []
    for f in files:
        with open(f, 'rb') as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _sch, ch, msg, dec in reader.iter_decoded_messages(
                    topics=['/hand_telemetry', '/throw_announcements']):
                t = msg.log_time / 1e9
                if ch.topic == '/hand_telemetry':
                    s = _stamp_s(dec.timestamp)
                    if s > 0:
                        hand_offsets.append(t - s)
                    continue
                anns.append({
                    'thrower': str(dec.thrower_name),
                    'target': str(getattr(dec, 'target_id', '') or ''),
                    'throw_time': _stamp_s(dec.throw_time),
                    'landing_time': _stamp_s(dec.landing_time),
                    'landing_position': [float(dec.landing_position.x),
                                         float(dec.landing_position.y),
                                         float(dec.landing_position.z)],
                    'initial_position': [float(dec.initial_position.x),
                                         float(dec.initial_position.y),
                                         float(dec.initial_position.z)],
                    'initial_velocity': [float(dec.initial_velocity.x),
                                         float(dec.initial_velocity.y),
                                         float(dec.initial_velocity.z)],
                    'predicted_tof_sec': float(
                        getattr(dec, 'predicted_tof_sec', 0.0) or 0.0),
                    't_bag': t})
    hand_offsets.sort()
    off = (hand_offsets[len(hand_offsets) // 2] if hand_offsets else 0.0)

    # ONE pass over /mocap_data, gated to the union of the landing windows — a
    # sitting carries ~137k frames and nothing here looks outside them.
    #
    # The window is the MINER's OUTER one (``_WINDOW_PRE_S`` = 6 s, not the
    # ``PRE_S`` = 1.2 s the arc fits use), and that is a correctness requirement
    # rather than generosity: :func:`resting_census` looks 2 s BEFORE the throw,
    # which for a ~0.9 s flight is ``landing − 2.9 s``. Caching at ±1.2 s leaves
    # it 0.25 s of overlap per announcement and it silently reports an eighth of
    # the frames — measured, when this probe was promoted: 1849 frames and 6
    # split announcements against the correct 14862 and 7.
    windows = [(a['landing_time'] + off - M._WINDOW_PRE_S,
                a['landing_time'] + off + M._WINDOW_POST_S)
               for a in anns if a['landing_time'] > 0.0]
    if not windows:
        raise IOError('{}: no announcement carries a landing time'.format(bag))
    lo = min(w[0] for w in windows)
    hi = max(w[1] for w in windows)

    def inside(t):
        return lo <= t <= hi and any(a <= t <= b for a, b in windows)

    markers, frames = [], 0
    for f in files:
        with open(f, 'rb') as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _sch, _ch, msg, dec in reader.iter_decoded_messages(
                    topics=['/mocap_data']):
                t = msg.log_time / 1e9
                if not inside(t):
                    continue
                frames += 1
                for mk in dec.markers:
                    px, py = float(mk.position.x), float(mk.position.y)
                    if abs(px) > WIDE_MM or abs(py) > WIDE_MM:
                        continue
                    markers.append((t, px, py, float(mk.position.z),
                                    str(mk.label or '')))
    markers.sort()
    return {'bag': bag, 'announcements': anns, 'log_minus_stamp_s': off,
            'markers': markers, 'n_frames': frames, 'wide_mm': WIDE_MM}


def load_cache(bag: str, *, root: str = DEFAULT_ROOT,
               refresh: bool = False) -> dict:
    """Pickled :func:`build_cache`. ``read_bag``-scale work runs once per bag."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    path = os.path.join(CACHE_DIR, bag + '.pkl')
    if os.path.exists(path) and not refresh:
        with open(path, 'rb') as fh:
            return pickle.load(fh)
    data = build_cache(bag, root=root)
    with open(path, 'wb') as fh:
        pickle.dump(data, fh, protocol=2)
    return data


def as_bagdata(cached: dict, lateral_mm: float = None):
    """The miner's ``BagData``, reconstructed EXACTLY as ``read_bag`` builds it.

    Same routing (labelled -> ``fixture_cells`` bucketed by second, unlabelled ->
    ``mocap``) and the same lateral gate, so :func:`track_for_toss` selects the
    rows the mined record's own fits see and the two cannot describe different
    markers.
    """
    lateral = M.LATERAL_MM if lateral_mm is None else lateral_mm
    data = M.BagData()
    data.announcements = cached['announcements']
    data.log_minus_stamp_s = cached['log_minus_stamp_s']
    for t, x, y, z, label in cached['markers']:
        if abs(x) > lateral or abs(y) > lateral:
            continue
        if label:
            data.fixture_cells.setdefault(
                int(math.floor(t)), set()).add(M._cell(x, y, z))
        else:
            data.mocap.append((t, x, y, z))
    data.mocap.sort()
    return data


# ═════════════════════════════════════════════════════════════════════════════
# Track extraction
# ═════════════════════════════════════════════════════════════════════════════

def _frames(rows):
    """Group ``(t, x, y, z)`` rows into per-frame lists keyed by time."""
    out = defaultdict(list)
    for t, x, y, z in rows:
        out[round(t / FRAME_EPS_S)].append((t, x, y, z))
    return out


def track_for_toss(data, ann, plane_mm):
    """One toss -> the single-marker track, miner-faithful in selection.

    Selection is ``M.ball_rows`` + ``M.branches`` — the same rows the mined
    record's fits see. What this ADDS is per-frame ASSOCIATION: the miner fits
    all candidate rows at once and trims, while a residual analysis needs exactly
    one point per frame, so the nearest row to the current ballistic estimate is
    kept and the fit repeated.

    -> a track dict, or ``{'skipped': reason}``. Never raises on a bad arc: a
    census over a sitting must be able to report *why* an arc was unusable.
    """
    land_bag = float(ann['landing_time']) + data.log_minus_stamp_s
    ref = M.command_reference(ann, plane_mm)
    rows_all = M.ball_rows(data, land_bag)
    asc, desc = M.branches(rows_all, float(ref['release_plane_mm']), plane_mm)
    rows = sorted(set(asc) | set(desc))
    if len(rows) < 2 * M.RELEASE_FIT_MIN_SAMPLES:
        return {'skipped': 'few_rows({})'.format(len(rows))}
    fit = M.fit_ballistic(rows)
    if fit is None:
        return {'skipped': 'no_arc_fit'}
    track, ambiguous = rows, 0
    for _ in range(3):
        pos, vel, t_ref = fit[0], fit[1], fit[2]
        keep, ambiguous = [], 0
        for _key, cand in sorted(_frames(rows).items()):
            best, bestd, n_in = None, ASSOC_GATE_MM, 0
            for t, x, y, z in cand:
                p = M.ballistics_bc.position_at(pos, vel, t - t_ref)
                d = math.sqrt((x - p[0]) ** 2 + (y - p[1]) ** 2
                              + (z - p[2]) ** 2)
                if d <= ASSOC_GATE_MM:
                    n_in += 1
                if d < bestd:
                    best, bestd = (t, x, y, z), d
            if best is not None:
                keep.append(best)
            if n_in > 1:
                ambiguous += 1
        if len(keep) < 2 * M.RELEASE_FIT_MIN_SAMPLES:
            return {'skipped': 'few_associated({})'.format(len(keep))}
        # No trimming on the refit: a trim would delete the very residual
        # structure this probe exists to characterise.
        refit = M.fit_ballistic(keep, trim_floor_mm=1e9)
        if refit is None:
            return {'skipped': 'no_refit'}
        track, fit = keep, refit
    pos, vel, t_ref, n, rms, se = fit
    return _track(np.array(track, dtype=float), vel, t_ref,
                  release_plane_mm=float(ref['release_plane_mm']),
                  fit_rms_mm=float(rms), n=int(n),
                  fit_se_mms=[float(v) for v in se],
                  fit_vel=[float(v) for v in vel],
                  n_candidate_rows=len(rows), n_ambiguous_frames=int(ambiguous))


def _track(rows, vel, t_ref, **extra):
    """The track dict, with the apex instant taken from the FITTED arc.

    The fitted apex, not the highest sample: parity is measured about it, and a
    sample-max apex is off by up to half a frame times the local curvature — on
    a 200 Hz capture that is small, but it is a bias, and it enters the even/odd
    split as an odd term, i.e. exactly where the force would be.
    """
    out = {'rows': rows, 't_ref': float(t_ref),
           't_apex': float(t_ref) + float(vel[2]) / M.GRAVITY_MMS2}
    out.update(extra)
    return out


def branch_velocities(tr):
    """Per-branch whole-branch ballistic fits, and the DELTA that is the artefact.

    The SAME estimator the miner used before E-1 (``M.fit_ballistic``), applied
    to the associated track — so the delta reported here is the one that was
    contaminating the mined direction channels, not a re-derivation of it.
    """
    rows, ta = tr['rows'], tr['t_apex']
    asc = [tuple(r) for r in rows if r[0] <= ta]
    desc = [tuple(r) for r in rows if r[0] >= ta]
    out = {}
    for name, rr in (('asc', asc), ('desc', desc)):
        f = M.fit_ballistic(rr, trim_floor_mm=1e9)
        if f is None:
            return None
        pos, vel, t_ref, n, rms, se = f
        del pos
        out[name] = {'vel': [float(v) for v in vel], 'n': int(n),
                     'rms_mm': float(rms), 't_ref': float(t_ref),
                     'se_mms': [float(v) for v in se]}
    out['delta_mms'] = [out['desc']['vel'][i] - out['asc']['vel'][i]
                        for i in range(3)]
    return out


# ═════════════════════════════════════════════════════════════════════════════
# The parity decomposition — THE discriminator
# ═════════════════════════════════════════════════════════════════════════════

def _interp(ts, vals, t):
    """Linear interpolation, REFUSING across a gap wider than the tolerance."""
    i = int(np.searchsorted(ts, t))
    if i <= 0 or i >= len(ts):
        return None
    if ts[i] - ts[i - 1] > MAX_INTERP_GAP_S:
        return None
    w = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
    return vals[i - 1] + w * (vals[i] - vals[i - 1])


def parity(tr):
    """Even/odd split of each axis about the apex. ``None`` if too few pairs.

    -> per axis, arrays over ``|tau|``::

        even_<a>  (p(+tau) + p(-tau))/2, referenced to the apex end  [mm]
        odd_<a>   (p(+tau) - p(-tau))/(2 tau)                        [mm/s]

    plus ``tau_s`` and the height ``z_mm`` each mirrored pair sits at.

    Under a pure ballistic truth the even part of a LATERAL axis is exactly the
    height-locked bias ``b(z(tau))`` up to a constant: the one aerodynamic term
    that is even in tau is a steady draft, and :func:`force_bounds` says how much
    wind that would take.
    """
    rows = tr['rows']
    ts = rows[:, 0]
    ta = tr['t_apex']
    out = {'tau_s': [], 'z_mm': []}
    for name in ('x', 'y', 'z'):
        out['even_' + name] = []
        out['odd_' + name] = []
    for t, x, y, z in rows:
        tau = t - ta
        if tau >= -MIN_TAU_S:
            continue                       # ascending side drives every pair
        vals = []
        for k in (1, 2, 3):
            v = _interp(ts, rows[:, k], ta - tau)      # the descending partner
            if v is None:
                vals = None
                break
            vals.append(v)
        if vals is None:
            continue
        out['tau_s'].append(-tau)
        out['z_mm'].append(0.5 * (z + vals[2]))
        for k, name, here in ((1, 'x', x), (2, 'y', y), (3, 'z', z)):
            mirror = vals[k - 1]
            out['even_' + name].append(0.5 * (here + mirror))
            out['odd_' + name].append((mirror - here) / (2.0 * (-tau)))
    for key in list(out):
        out[key] = np.asarray(out[key], dtype=float)
    if len(out['tau_s']) < 10:
        return None
    order = np.argsort(out['tau_s'])
    for key in list(out):
        out[key] = out[key][order]
    for name in ('x', 'y', 'z'):
        ref = float(np.median(out['even_' + name][:5]))
        out['even_' + name] = out['even_' + name] - ref
    return out


def poly_fit(tr, degree=3):
    """The parity argument in polynomial form, with standard errors.

    A height-linear position bias adds a QUADRATIC term in tau and no cubic
    (``z`` is quadratic in tau); a Magnus force about a horizontal axis adds a
    CUBIC and no quadratic. Reported per axis as coefficient and SE, so
    "significant" is a number rather than an impression.
    """
    rows = tr['rows']
    tau = rows[:, 0] - tr['t_apex']
    out = {}
    for k, name in ((1, 'x'), (2, 'y')):
        v = rows[:, k]
        design = np.stack([tau ** d for d in range(degree + 1)], axis=1)
        coef = np.linalg.lstsq(design, v, rcond=None)[0]
        resid = v - design.dot(coef)
        dof = max(len(v) - (degree + 1), 1)
        cov = (float((resid ** 2).sum()) / dof) * np.linalg.pinv(
            design.T.dot(design))
        out[name] = {'coef': [float(c) for c in coef],
                     'se': [float(s) for s in
                            np.sqrt(np.clip(np.diag(cov), 0.0, None))],
                     'rms_mm': float(np.sqrt((resid ** 2).mean()))}
    return out


def spectrum(tr, model_degree=2):
    """Periodogram of the lateral residual after removing a low-order polynomial.

    A rotating asymmetry at ``f`` leaves power at ``f``. Reported WITH the
    resolution caveat the arc length forces: ``df = 1/T``, so anything below
    ``1/T`` is not resolvable and is degenerate with the polynomial just removed.
    """
    rows = tr['rows']
    tau = rows[:, 0] - tr['t_apex']
    span = float(tau[-1] - tau[0])
    out = {'T_s': span, 'f_min_resolvable_hz': 1.0 / span if span > 0 else None}
    if span <= 0:
        return out
    dt = float(np.median(np.diff(rows[:, 0])))
    grid = np.arange(tau[0], tau[-1], dt)
    for k, name in ((1, 'x'), (2, 'y')):
        v = rows[:, k]
        design = np.stack([tau ** d for d in range(model_degree + 1)], axis=1)
        resid = v - design.dot(np.linalg.lstsq(design, v, rcond=None)[0])
        regular = np.interp(grid, tau, resid)
        win = np.hanning(len(regular))
        amp = np.abs(np.fft.rfft((regular - regular.mean()) * win)) \
            * 4.0 / len(regular)
        freq = np.fft.rfftfreq(len(regular), dt)
        keep = freq >= 1.0 / span
        if not keep.any():
            continue
        i = int(np.argmax(amp[keep]))
        out[name] = {'resid_rms_mm': float(np.sqrt((resid ** 2).mean())),
                     'peak_hz': float(freq[keep][i]),
                     'peak_amp_mm': float(amp[keep][i]),
                     'median_amp_mm': float(np.median(amp[keep]))}
    return out


def leak_into_whole_arc(tr, par):
    """How much of the even bias survives a WHOLE-ARC linear velocity fit.

    The even part contributes to a least-squares slope only through
    ``cov(tau, E)/var(tau)``, and with coverage symmetric about the apex that
    covariance is exactly zero because ``E`` is even. This measures the actual
    leak on the actual samples — the number that licenses a whole-arc estimator
    where a per-branch one is unusable.

    ``coverage_asym_s`` is the FIRST MOMENT of the coverage, ``mean(tau)``, and
    is the same quantity ``toss_record_miner`` mines per row. Note what it is
    not: a zero first moment does not prove a symmetric distribution, so a small
    ``coverage_asym_s`` bounds the gross-truncation case and not the shape case.
    Both are reported; the summary prints them side by side for exactly that
    reason.
    """
    tau = tr['rows'][:, 0] - tr['t_apex']
    out = {'coverage_asym_s': float(tau.mean()),
           'span_pre_s': float(-tau.min()), 'span_post_s': float(tau.max())}
    var = float(((tau - tau.mean()) ** 2).sum())
    for name in ('x', 'y'):
        e = _bias_at(par['tau_s'], par['even_' + name], tau)
        cov = float(((tau - tau.mean()) * (e - e.mean())).sum())
        out['leak_v' + name + '_mms'] = cov / var if var > 0 else None
    return out


def _bias_at(taus, vals, tau_query):
    """The bias profile evaluated at ``|tau|``, clamped outside its span."""
    return np.interp(np.abs(tau_query), taus, vals, left=vals[0],
                     right=vals[-1])


def corrected_branch_delta(tr, par):
    """Branch delta after subtracting THIS arc's OWN even part.

    Near-tautological by construction — the branch delta lives entirely in the
    even part — and reported anyway as the numerical closure: if a residual delta
    survived here, the parity decomposition would not be the whole story. The
    load-bearing version is the HELD-OUT one in :func:`aggregate`, where the
    correction comes from other arcs entirely.
    """
    rows = tr['rows'].copy()
    tau = rows[:, 0] - tr['t_apex']
    for k, name in ((1, 'x'), (2, 'y')):
        rows[:, k] = rows[:, k] - _bias_at(par['tau_s'], par['even_' + name],
                                           tau)
    br = branch_velocities({'rows': rows, 't_apex': tr['t_apex']})
    return None if br is None else br['delta_mms']


# ═════════════════════════════════════════════════════════════════════════════
# The censuses and the controls
# ═════════════════════════════════════════════════════════════════════════════

def companion_census(cached, tr, radius_mm=COMPANION_MM):
    """Frames near the arc carrying a SECOND marker — the blob-split signature.

    Run against the WIDE cache (labelled markers included) so a fragment the
    miner's fixture filter deleted is still seen: that filter drops any
    unlabelled row sitting where a LABELLED marker has been, which is exactly
    what a strut-adjacent fragment looks like.

    LABELLED and UNLABELLED are split because they answer different questions. A
    labelled rig marker at 75 mm is furniture the object flew past and can never
    enter the track; an UNLABELLED companion inside a ball diameter IS the
    bisection signature, and its separation is the only in-data measurement of
    the tracked object's optical scale.
    """
    by_frame = defaultdict(list)
    for t, x, y, z, label in cached['markers']:
        by_frame[round(t / FRAME_EPS_S)].append((t, x, y, z, label))
    out = {'n_frames': int(len(tr['rows'])), 'radius_mm': radius_mm,
           'unlab_seps_mm': [], 'lab_seps_mm': [],
           'n_frames_unlab_companion': 0, 'n_frames_lab_companion': 0,
           'n_frames_unlab_within_diameter': 0}
    for t, x, y, z in tr['rows']:
        unlab, lab = [], []
        for _tt, xx, yy, zz, label in by_frame.get(round(t / FRAME_EPS_S), ()):
            d = math.sqrt((xx - x) ** 2 + (yy - y) ** 2 + (zz - z) ** 2)
            if d < 1e-6 or d > radius_mm:
                continue                      # the track point itself, or far
            (lab if label else unlab).append(d)
        if unlab:
            out['n_frames_unlab_companion'] += 1
            out['unlab_seps_mm'].extend(unlab)
            if any(d <= 2 * BALL_RADIUS_MM for d in unlab):
                out['n_frames_unlab_within_diameter'] += 1
        if lab:
            out['n_frames_lab_companion'] += 1
            out['lab_seps_mm'].extend(lab)
    out['unlab_seps_mm'].sort()
    out['lab_seps_mm'].sort()
    return out


def resting_census(cached, ann, off, radius_mm=COMPANION_MM):
    """The same census on the object at REST, before release.

    The extreme form of the split — one held ball reconstructing as two markers —
    is observable at the bottom of the stroke, where the platform occludes
    cameras. Centred on the announced cup, NOT on the origin: these are displaced
    tosses, and a census centred at (0, 0) finds zero markers on every one of
    them and reports "no resting track", which is false.
    """
    t_throw = float(ann['throw_time']) + off
    lo, hi = t_throw - 2.0, t_throw - 0.05
    by_frame = defaultdict(list)
    for t, x, y, z, label in cached['markers']:
        if lo <= t <= hi:
            by_frame[round(t / FRAME_EPS_S)].append((t, x, y, z, label))
    if not by_frame:
        return None
    cup_x, cup_y, cup_z = [float(v) for v in ann['landing_position']]
    counts, seps = [], []
    for _key, rows in sorted(by_frame.items()):
        cand = [r for r in rows
                if not r[4] and math.hypot(r[1] - cup_x, r[2] - cup_y) < 120.0
                and abs(r[3] - cup_z) < 260.0]
        counts.append(len(cand))
        for i in range(len(cand)):
            for j in range(i + 1, len(cand)):
                d = math.sqrt(sum((cand[i][k] - cand[j][k]) ** 2
                                  for k in (1, 2, 3)))
                if d <= radius_mm:
                    seps.append((d, float(cand[i][0] - t_throw)))
    return {'n_frames': len(counts),
            'frames_with_0': sum(1 for c in counts if c == 0),
            'frames_with_1': sum(1 for c in counts if c == 1),
            'frames_with_2plus': sum(1 for c in counts if c >= 2),
            'pair_separations_mm': sorted(s for s, _t in seps)}


def rig_control(cached, pairs=(('Platform - 3', 'Platform - 4'),
                               ('Platform - 1', 'Platform - 5'),
                               ('Base - 3', 'Base - 6'))):
    """CONTROL: how well does this capture reconstruct ORDINARY point markers?

    Inter-marker distances on rigid bodies over the whole cached window. If the
    volume itself were warped at the tens-of-mm scale the tracked ball shows,
    these would wander by the same order. They do not — which is what makes "the
    ball is a fully-taped SPHERE, not a point marker" the load-bearing difference
    rather than "the mocap is bad".
    """
    by_frame = defaultdict(dict)
    for t, x, y, z, label in cached['markers']:
        if label:
            by_frame[round(t / FRAME_EPS_S)][label] = (x, y, z)
    out = {}
    for a, b in pairs:
        ds = [float(np.linalg.norm(np.array(mk[a]) - np.array(mk[b])))
              for mk in by_frame.values() if a in mk and b in mk]
        if len(ds) > 100:
            out[a + '|' + b] = {'n': len(ds), 'mean_mm': float(np.mean(ds)),
                                'sd_mm': float(np.std(ds))}
    return out


def force_bounds(odd_z_mms, *, v0_mms=4900.0, tau_eval_s=0.35,
                 radius_m=BALL_RADIUS_MM / 1000.0, mass_kg=BALL_MASS_KG,
                 rho=1.20, cd=0.5, target_accel_mms2=220.0):
    """Bound the object's aerodynamic scale FROM THE DATA, not from assumptions.

    Vertical drag is ODD about the apex (the force flips with ``vz``), so it
    lands entirely in the vertical ODD channel :func:`parity` already measures —
    a channel with NO ballistic content at all, since a drag-free arc is exactly
    symmetric. So ``odd_z`` IS a measurement of ``k = 0.5 rho Cd A / m``, and
    every other aerodynamic force on the same object is bounded by the same
    ``rho A / m`` it shares.

    -> the implied ``k``, its ratio to nominal, the spin a Magnus force would
    need to reach ``target_accel_mms2`` at that bound, and the steady WIND that
    would be needed — the one aerodynamic term that is even in tau, hence the
    only one that could mimic a height-locked bias at all.
    """
    g = M.GRAVITY_MMS2 / 1000.0
    area = math.pi * radius_m ** 2
    k_nom = 0.5 * rho * cd * area / mass_kg          # 1/m

    def odd_z_raw(k):
        dt = 1e-4
        z, vz, t = 0.0, v0_mms / 1000.0, 0.0
        hist = []
        while t < 1.4 and (z > -1.0 or vz > 0):
            hist.append((t, z))
            vz += (-g - k * abs(vz) * vz) * dt
            z += vz * dt
            t += dt
        arr = np.array(hist)
        t_ap = arr[int(np.argmax(arr[:, 1])), 0]
        za = np.interp(t_ap - tau_eval_s, arr[:, 0], arr[:, 1])
        zd = np.interp(t_ap + tau_eval_s, arr[:, 0], arr[:, 1])
        return (zd - za) / (2 * tau_eval_s) * 1000.0        # mm/s

    # The k = 0 arc is EXACTLY symmetric, so whatever odd_z_raw(0) returns is
    # pure integration error and is subtracted. Without this the nominal drag
    # signal — a few mm/s — is swamped by it, and at dt = 1 ms it even flips
    # sign: the first cut of this function reported -0.45 mm/s for a quantity
    # that is +3.7.
    base = odd_z_raw(0.0)
    nominal = odd_z_raw(k_nom) - base
    implied = k_nom * (odd_z_mms / nominal) if nominal else None
    out = {'k_nominal_per_m': k_nom,
           'odd_z_predicted_at_k_nominal_mms': nominal,
           'odd_z_measured_mms': odd_z_mms,
           'implied_k_per_m': implied,
           'aero_scale_vs_nominal': (implied / k_nom) if implied else None,
           'assumed_mass_kg': mass_kg, 'assumed_radius_m': radius_m,
           'target_accel_mms2': target_accel_mms2}
    #  Magnus at low spin ratio S = omega R / v has C_L ~ S, so
    #  a = (k/Cd) . omega . R . v.
    v = v0_mms / 1000.0
    for tag, kk in (('nominal', k_nom), ('implied', implied)):
        if not kk:
            continue
        per_rev = (kk / cd) * (2 * math.pi) * radius_m * v * 1000.0
        out['magnus_mms2_per_rev_s_' + tag] = per_rev
        out['spin_rev_s_needed_' + tag] = target_accel_mms2 / per_rev
        # A steady draft does NOT flip with vz, so it is the only aerodynamic
        # term with the same parity as a position bias. This is the number that
        # closes the "could it be air?" question.
        out['wind_mps_needed_' + tag] = (target_accel_mms2 / 1000.0) / (kk * v)
    return out


# ═════════════════════════════════════════════════════════════════════════════
# Per-bag analysis and aggregation
# ═════════════════════════════════════════════════════════════════════════════

def analyse_bag(bag, *, root=DEFAULT_ROOT, plane_mm=None, refresh=False,
                robot='jugglebot'):
    """-> one record per self-toss announcement, tracked or skipped."""
    cached = load_cache(bag, root=root, refresh=refresh)
    data = as_bagdata(cached)
    plane = M.DEFAULT_PLANE_MM if plane_mm is None else plane_mm
    out = []
    for i, ann in enumerate(M.self_tosses(cached['announcements'], robot)):
        rec = {'bag': bag, 'index': i, 'throw_time': ann['throw_time'],
               'cup_xyz': ann['landing_position'],
               'cmd_release_vel': ann['initial_velocity']}
        # The resting census runs on EVERY announcement, tracked or not: it asks
        # about the object sitting in the cup, which is there either way.
        rec['resting'] = resting_census(cached, ann,
                                        cached['log_minus_stamp_s'])
        tr = track_for_toss(data, ann, plane)
        if 'rows' not in tr:
            rec['skipped'] = tr.get('skipped', 'no_track')
            out.append(rec)
            continue
        par = parity(tr)
        if par is None:
            rec['skipped'] = 'too_few_mirrored_pairs'
            out.append(rec)
            continue
        rec.update({
            'n_track': int(len(tr['rows'])),
            'n_candidate_rows': tr['n_candidate_rows'],
            'n_ambiguous_frames': tr['n_ambiguous_frames'],
            'fit_rms_mm': tr['fit_rms_mm'],
            't_apex': tr['t_apex'],
            'apex_z_mm': float(tr['rows'][:, 3].max()),
            'release_plane_mm': tr['release_plane_mm'],
            'arc_span_s': float(tr['rows'][-1, 0] - tr['rows'][0, 0]),
            'whole_arc_vel_mms': list(tr['fit_vel']),
            'lat_mean_mm': [float(tr['rows'][:, 1].mean()),
                            float(tr['rows'][:, 2].mean())],
            'branch': branch_velocities(tr),
            'poly': poly_fit(tr),
            'spectrum': spectrum(tr),
            'companion': companion_census(cached, tr),
            'parity': {k: [float(v) for v in par[k]] for k in par},
            'leak': leak_into_whole_arc(tr, par),
            'corrected_delta_mms': corrected_branch_delta(tr, par),
        })
        far = par['tau_s'] > ODD_READ_TAU_S
        for name in ('x', 'y', 'z'):
            rec['odd_' + name + '_mms'] = (
                float(np.median(par['odd_' + name][far])) if far.any() else None)
        for key, col in (('_rows_t', 0), ('_rows_x', 1), ('_rows_y', 2),
                         ('_rows_z', 3)):
            rec[key] = [float(v) for v in tr['rows'][:, col]]
        out.append(rec)
    return out


def _profile_on_grid(rec, key, grid, axis='z'):
    """One arc's even profile resampled onto a shared grid; NaN outside its span."""
    par = rec['parity']
    x = np.asarray(par['z_mm' if axis == 'z' else 'tau_s'], float)
    v = np.asarray(par[key], float)
    order = np.argsort(x)
    x, v = x[order], v[order]
    out = np.full(len(grid), np.nan)
    inside = (grid >= x[0]) & (grid <= x[-1])
    out[inside] = np.interp(grid[inside], x, v)
    return out


def _pairwise_r(profiles, min_overlap=8):
    rs = []
    for i in range(len(profiles)):
        for j in range(i + 1, len(profiles)):
            a, b = profiles[i], profiles[j]
            ok = np.isfinite(a) & np.isfinite(b)
            if ok.sum() < min_overlap:
                continue
            aa, bb = a[ok] - a[ok].mean(), b[ok] - b[ok].mean()
            den = math.sqrt(float((aa ** 2).sum() * (bb ** 2).sum()))
            if den > 0:
                rs.append(float(aa.dot(bb) / den))
    return rs


def apply_donor_profile(rec, donors, zgrid):
    """Correct ONE arc with the donors' pooled ``b(z)`` and re-measure the delta.

    This is the held-out test, and it is the load-bearing one: a bias fitted on
    OTHER arcs is a claim about the room, and it either transfers or it does not.
    """
    prof = {}
    for name in ('x', 'y'):
        prof[name] = np.nanmean(np.vstack(
            [_profile_on_grid(u, 'even_' + name, zgrid, 'z') for u in donors]),
            axis=0)
    rows = np.array([rec['_rows_t'], rec['_rows_x'], rec['_rows_y'],
                     rec['_rows_z']], dtype=float).T
    for name, k in (('x', 1), ('y', 2)):
        good = np.isfinite(prof[name])
        if good.sum() < 5:
            return None
        rows[:, k] = rows[:, k] - np.interp(
            rows[:, 3], zgrid[good], prof[name][good],
            left=prof[name][good][0], right=prof[name][good][-1])
    br = branch_velocities({'rows': rows, 't_apex': rec['t_apex']})
    return None if br is None else [float(v) for v in br['delta_mms'][:2]]


def aggregate(records):
    """The cross-arc verdict: repeatability, held-out transfer, leak, controls."""
    ts = [r for r in records if 'parity' in r]
    out = {'n_tracks': len(ts), 'n_announcements': len(records)}
    if not ts:
        return out
    # A shared height grid, inset from both ends so every arc contributes over
    # its whole width rather than through an extrapolation.
    z_lo = max(r['release_plane_mm'] + 80.0 for r in ts)
    z_hi = min(r['apex_z_mm'] - 80.0 for r in ts)
    zgrid = np.arange(z_lo, z_hi, Z_GRID_MM)
    tgrid = np.arange(MIN_TAU_S, min(max(r['parity']['tau_s']) for r in ts),
                      0.01)
    out['z_grid_mm'] = [float(zgrid[0]), float(zgrid[-1])]
    for name in ('x', 'y'):
        pz = [_profile_on_grid(r, 'even_' + name, zgrid, 'z') for r in ts]
        pt = [_profile_on_grid(r, 'even_' + name, tgrid, 'tau') for r in ts]
        # A grid column outside EVERY arc's span is all-NaN, and nanmean/nanstd
        # warn on it. That is the correct value (no data at that height), not a
        # defect, so the warning is suppressed rather than the NaN avoided —
        # suppressed NARROWLY, because a nan appearing anywhere else in this
        # aggregate would be a real bug.
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            mean_z = np.nanmean(np.vstack(pz), axis=0)
            sd_z = np.nanstd(np.vstack(pz), axis=0)
            sd_tau = np.nanmedian(np.nanstd(np.vstack(pt), axis=0))
        out['even_' + name] = {
            'pooled_profile_grid_mm': [float(v) for v in zgrid],
            'profile_z_mean_mm': [float(v) for v in mean_z],
            'profile_z_sd_mm': [float(v) for v in sd_z],
            'amplitude_mm': float(np.nanmax(mean_z) - np.nanmin(mean_z)),
            'sd_median_mm': float(np.nanmedian(sd_z)),
            'sd_median_over_tau_mm': float(sd_tau),
            'pairwise_r_z_median': float(np.median(_pairwise_r(pz))),
            'pairwise_r_tau_median': float(np.median(_pairwise_r(pt))),
        }
    # HELD OUT: leave-one-out, and (when the corpus spans sittings) cross-bag.
    held = {'raw': [], 'loo': [], 'cross_bag': []}
    for i, rec in enumerate(ts):
        held['raw'].append([float(v) for v in rec['branch']['delta_mms'][:2]])
        for mode, donors in (
                ('loo', [u for j, u in enumerate(ts) if j != i]),
                ('cross_bag', [u for u in ts if u['bag'] != rec['bag']])):
            if not donors:
                continue
            res = apply_donor_profile(rec, donors, zgrid)
            if res is not None:
                held[mode].append(res)
    out['held_out'] = {
        k: {'median_abs_dvx': float(np.median([abs(r[0]) for r in v])),
            'median_abs_dvy': float(np.median([abs(r[1]) for r in v])),
            'p90_abs_dvy': float(np.percentile([abs(r[1]) for r in v], 90)),
            'n': len(v)}
        for k, v in held.items() if v}
    # Velocity estimates, three ways, so the artefact and its fix are side by
    # side in one table.
    for name, key in (('x', 0), ('y', 1)):
        for tag, pull in (('whole_arc_v', lambda r: r['whole_arc_vel_mms'][key]),
                          ('branch_asc_v',
                           lambda r: r['branch']['asc']['vel'][key]),
                          ('branch_desc_v',
                           lambda r: r['branch']['desc']['vel'][key]),
                          ('branch_delta_v',
                           lambda r: r['branch']['delta_mms'][key])):
            vals = [pull(r) for r in ts]
            out[tag + name + '_mms'] = {
                'median': float(np.median(vals)), 'sd': float(np.std(vals)),
                'min': float(min(vals)), 'max': float(max(vals))}
    for name in ('x', 'y', 'z'):
        vals = [r['odd_' + name + '_mms'] for r in ts
                if r.get('odd_' + name + '_mms') is not None]
        if vals:
            out['odd_' + name + '_mms'] = {
                'median': float(np.median(vals)), 'sd': float(np.std(vals))}
    # The two parities as EQUIVALENT LATERAL ACCELERATIONS — the comparison the
    # verdict rests on. For p = ... + c2 tau^2 + c3 tau^3:
    #   even/quadratic -> a_even = 2 c2   (NO aerodynamic force has this parity)
    #   odd/cubic      -> a = c.vz with c = -6 c3 / g, i.e. |c| v0 at |vz| = v0
    v0 = 4400.0
    for name in ('x', 'y'):
        c2 = [r['poly'][name]['coef'][2] for r in ts]
        c3 = [r['poly'][name]['coef'][3] for r in ts]
        out['equiv_accel_' + name] = {
            'even_quadratic_mms2_median': float(np.median([2 * c for c in c2])),
            'odd_cubic_equiv_mms2_median': float(np.median(
                [abs(6.0 * c / M.GRAVITY_MMS2) * v0 for c in c3])),
            'odd_cubic_sign_consistent': float(np.mean(
                [1.0 if c < 0 else 0.0 for c in c3])),
            'quad_over_se_median': float(np.median(
                [abs(r['poly'][name]['coef'][2]) / r['poly'][name]['se'][2]
                 for r in ts])),
            'cubic_over_se_median': float(np.median(
                [abs(r['poly'][name]['coef'][3]) / r['poly'][name]['se'][3]
                 for r in ts])),
        }
        pk = [r['spectrum'][name]['peak_amp_mm'] for r in ts
              if name in r['spectrum']]
        hz = [r['spectrum'][name]['peak_hz'] for r in ts if name in r['spectrum']]
        if pk:
            out['spectrum_' + name] = {
                'peak_amp_mm_median': float(np.median(pk)),
                'peak_hz_median': float(np.median(hz)),
                'peak_hz_sd': float(np.std(hz)),
                'f_min_resolvable_hz_median': float(np.median(
                    [r['spectrum']['f_min_resolvable_hz'] for r in ts])),
            }
    # The whole-arc estimator's exposure, and its driver.
    for name in ('x', 'y'):
        vals = [abs(r['leak']['leak_v' + name + '_mms']) for r in ts]
        out['leak_v' + name] = {'median_abs_mms': float(np.median(vals)),
                                'max_abs_mms': float(max(vals))}
    asym = [r['leak']['coverage_asym_s'] for r in ts]
    out['coverage_asym_s'] = {'median_abs': float(np.median(np.abs(asym))),
                              'max_abs': float(max(abs(v) for v in asym)),
                              'values': [float(v) for v in asym]}
    # Is the bias locked to the ROOM? Same profile at different object positions
    # within one sitting, and the same position across sittings.
    def _stats(members, key):
        """The group's bias at the LOWEST height it actually covers.

        Not blindly ``zgrid[0]``: the shared grid spans every arc's common
        range, and a group whose arcs start higher has NaN there. Reporting the
        NaN would put ``+nan`` in a comparison table that exists to be read
        across groups, so the height is chosen per group and REPORTED with the
        number — two groups quoted at different heights are not comparable, and
        the reader has to be able to see that.
        """
        prof = np.vstack([_profile_on_grid(r, key, zgrid, 'z')
                          for r in members])
        with np.errstate(invalid='ignore'):
            finite = np.where(np.isfinite(prof).all(axis=0))[0]
        if not finite.size:
            return {'n': len(members), 'b_mm': None, 'sem_mm': None,
                    'at_z_mm': None}
        col = int(finite[0])
        return {'n': len(members), 'at_z_mm': float(zgrid[col]),
                'b_mm': float(prof[:, col].mean()),
                'sem_mm': float(prof[:, col].std() / math.sqrt(len(members)))}

    by_bag_cup = defaultdict(list)
    for r in ts:
        by_bag_cup[(r['bag'], round(r['cup_xyz'][0]),
                    round(r['cup_xyz'][1]))].append(r)
    out['by_bag_and_cup'] = {
        '{}|{},{}'.format(*k): {'y': _stats(v, 'even_y'),
                                'x': _stats(v, 'even_x')}
        for k, v in sorted(by_bag_cup.items())}
    by_cup = defaultdict(list)
    for r in ts:
        by_cup[(round(r['cup_xyz'][0]), round(r['cup_xyz'][1]))].append(r)
    out['same_cup_across_bags'] = {
        '{},{}'.format(*cup): {b: _stats([r for r in v if r['bag'] == b],
                                         'even_y')
                               for b in sorted({r['bag'] for r in v})}
        for cup, v in sorted(by_cup.items())
        if len({r['bag'] for r in v}) > 1}
    out['aero_from_vertical'] = force_bounds(
        float(np.median([abs(r['odd_z_mms']) for r in ts
                         if r.get('odd_z_mms') is not None])))
    out['companion'] = {
        'frames_total': int(sum(r['companion']['n_frames'] for r in ts)),
        'frames_unlab': int(sum(r['companion']['n_frames_unlab_companion']
                                for r in ts)),
        'frames_unlab_within_diameter': int(sum(
            r['companion']['n_frames_unlab_within_diameter'] for r in ts)),
        'frames_lab': int(sum(r['companion']['n_frames_lab_companion']
                              for r in ts)),
        'ambiguous_frames': int(sum(r['n_ambiguous_frames'] for r in ts))}
    rest = [r['resting'] for r in records if r.get('resting')]
    seps = sorted(s for r in rest for s in r['pair_separations_mm'])
    out['resting'] = {
        'n_announcements': len(rest),
        'n_with_a_split': sum(1 for r in rest if r['frames_with_2plus'] > 3),
        'split_frame_frac': sorted(
            round(r['frames_with_2plus'] / max(r['n_frames'], 1), 3)
            for r in rest if r['frames_with_2plus'] > 3),
        'frames': int(sum(r['n_frames'] for r in rest)),
        'frames_1_marker': int(sum(r['frames_with_1'] for r in rest)),
        'frames_2plus': int(sum(r['frames_with_2plus'] for r in rest)),
        'n_pairs': len(seps),
        'sep_percentiles_mm': ([float(v) for v in
                                np.percentile(seps, [0, 25, 50, 75, 100])]
                               if seps else None),
        'sep_hist': {tag: int(sum(1 for s in seps if lo <= s < hi))
                     for tag, lo, hi in (('0-40', 0, 40), ('40-70', 40, 70),
                                         ('70-100', 70, 100),
                                         ('100-140', 100, 140))}}
    return out


# ═════════════════════════════════════════════════════════════════════════════
# Reporting
# ═════════════════════════════════════════════════════════════════════════════

def summary_text(records, agg) -> str:
    ts = [r for r in records if 'parity' in r]
    lines = []
    add = lines.append
    add('mocap parity-bias diagnostic')
    add('=' * 72)
    add('corpus: {} tracked arcs of {} announcements, bags {}'
        .format(len(ts), len(records), ', '.join(sorted({r['bag']
                                                         for r in records}))))
    if not ts:
        return '\n'.join(lines) + '\nNO TRACKED ARCS — nothing to report.\n'
    add('')
    add('THE ARTEFACT: per-branch velocity delta (desc - asc)')
    for name in ('x', 'y'):
        d = agg['branch_delta_v' + name + '_mms']
        add('  v_{}: median {:+.1f} mm/s  sd {:.1f}  range [{:+.1f}, {:+.1f}]'
            .format(name, d['median'], d['sd'], d['min'], d['max']))
        add('        asc {:+.1f}   desc {:+.1f}   WHOLE-ARC {:+.1f} mm/s'
            .format(agg['branch_asc_v' + name + '_mms']['median'],
                    agg['branch_desc_v' + name + '_mms']['median'],
                    agg['whole_arc_v' + name + '_mms']['median']))
    add('')
    add('PARITY ABOUT THE APEX (the discriminator)')
    for name in ('x', 'y'):
        e = agg['even_' + name]
        add('  EVEN part b_{}(z): amplitude {:.1f} mm, cross-arc sd {:.2f} mm, '
            'pairwise r {:.3f}'.format(name, e['amplitude_mm'],
                                       e['sd_median_mm'],
                                       e['pairwise_r_z_median']))
        q = agg['equiv_accel_' + name]
        add('    equivalent lateral accel: EVEN {:+.1f} mm/s^2 (quad/se {:.1f}) '
            '| ODD cubic {:.1f} mm/s^2 (cubic/se {:.1f}), sign consistent on '
            '{:.0f}% of arcs'.format(q['even_quadratic_mms2_median'],
                                     q['quad_over_se_median'],
                                     q['odd_cubic_equiv_mms2_median'],
                                     q['cubic_over_se_median'],
                                     100 * q['odd_cubic_sign_consistent']))
    add('  object radius assumed {:.0f} mm -> the even part is {:.0f}% of it'
        .format(BALL_RADIUS_MM,
                100 * agg['even_y']['amplitude_mm'] / BALL_RADIUS_MM))
    add('')
    add('HELD-OUT CORRECTION (delta after subtracting a profile fitted on OTHER '
        'arcs)')
    for key in ('raw', 'loo', 'cross_bag'):
        h = agg['held_out'].get(key)
        if h is None:
            continue
        add('  {:10s} n={:<3d} |dv_y| median {:6.1f}  p90 {:6.1f}   |dv_x| '
            'median {:5.1f}'.format(key, h['n'], h['median_abs_dvy'],
                                    h['p90_abs_dvy'], h['median_abs_dvx']))
    add('')
    add('WHOLE-ARC ESTIMATOR LEAK, and its driver')
    add('  leak v_y median {:.2f} mm/s  max {:.2f};  v_x median {:.2f}  max '
        '{:.2f}'.format(agg['leak_vy']['median_abs_mms'],
                        agg['leak_vy']['max_abs_mms'],
                        agg['leak_vx']['median_abs_mms'],
                        agg['leak_vx']['max_abs_mms']))
    ca = agg['coverage_asym_s']
    add('  |coverage_asym_s| median {:.4f} s  max {:.4f} s   (miner refuses '
        'past {:.2f} s)'.format(ca['median_abs'], ca['max_abs'],
                                M.COVERAGE_ASYM_MAX_S))
    add('  NOTE: coverage_asym_s is the FIRST MOMENT — a zero first moment does '
        'not prove symmetric coverage, so the gate bounds gross truncation and '
        'the residual leak above is a standing uncertainty.')
    add('')
    add('AERO SCALE, from the data (vertical odd channel = pure drag)')
    a = agg['aero_from_vertical']
    add('  odd_z measured {:.1f} mm/s vs {:.1f} predicted at nominal drag '
        '(k={:.4f}/m) -> aero <= {:.1f}x nominal'
        .format(a['odd_z_measured_mms'], a['odd_z_predicted_at_k_nominal_mms'],
                a['k_nominal_per_m'], a['aero_scale_vs_nominal']))
    add('  to reach {:.0f} mm/s^2 laterally: Magnus needs {:.1f} rev/s '
        '(nominal) / {:.1f} rev/s (at that bound) and lands in the ODD channel; '
        'a steady draft needs {:.1f} m/s'.format(
            a['target_accel_mms2'], a['spin_rev_s_needed_nominal'],
            a['spin_rev_s_needed_implied'], a['wind_mps_needed_implied']))
    add('')
    add('SPIN: periodicity in the lateral residual after a quadratic')
    for name in ('x', 'y'):
        s = agg.get('spectrum_' + name)
        if s:
            add('  {}: peak {:.2f} mm at {:.2f} Hz median (arc resolves f >= '
                '{:.2f} Hz only)'.format(name, s['peak_amp_mm_median'],
                                         s['peak_hz_median'],
                                         s['f_min_resolvable_hz_median']))
    add('')
    add('BLOB-SPLIT CENSUS')
    c = agg['companion']
    add('  in flight: {} arc frames, {} with an unlabelled companion <= {:.0f} '
        'mm, {} within an object diameter; {} beside a LABELLED rig marker'
        .format(c['frames_total'], c['frames_unlab'], COMPANION_MM,
                c['frames_unlab_within_diameter'], c['frames_lab']))
    r = agg['resting']
    add('  at rest (2 s pre-release), ALL announcements: {} frames, {} with one '
        'marker, {} with two ({:.0f} %)'.format(
            r['frames'], r['frames_1_marker'], r['frames_2plus'],
            100.0 * r['frames_2plus'] / max(r['frames'], 1)))
    add('  {} of {} announcements show a persistent split; per-arc split-frame '
        'fraction {}'.format(r['n_with_a_split'], r['n_announcements'],
                             r['split_frame_frac']))
    if r['sep_percentiles_mm']:
        add('  separations mm (p0/p25/p50/p75/p100): '
            + ' / '.join('{:.1f}'.format(v) for v in r['sep_percentiles_mm'])
            + '   histogram ' + json.dumps(r['sep_hist']))
    add('')
    add('POSITION LOCK — bias at the lowest height each group covers, by bag '
        'and object xy')
    for key, v in sorted(agg['by_bag_and_cup'].items()):
        if v['y']['b_mm'] is None:
            add('  {:34s} n={}  (no common height)'.format(key, v['y']['n']))
            continue
        add('  {:34s} n={}  at z={:.0f}  b_y {:+.1f} +- {:.1f} mm   '
            'b_x {:+.1f} +- {:.1f}'
            .format(key, v['y']['n'], v['y']['at_z_mm'], v['y']['b_mm'],
                    v['y']['sem_mm'], v['x']['b_mm'], v['x']['sem_mm']))
    for cup, bags in sorted(agg['same_cup_across_bags'].items()):
        add('  SAME position {} across sittings: {}'.format(
            cup, ', '.join(
                '{} n={} b_y {} at z={}'.format(
                    b, s['n'],
                    'n/a' if s['b_mm'] is None else '{:+.1f}'.format(s['b_mm']),
                    'n/a' if s['at_z_mm'] is None
                    else '{:.0f}'.format(s['at_z_mm']))
                for b, s in sorted(bags.items()))))
    add('')
    add('RIG CONTROL — ordinary point markers on the same captures')
    for k, v in sorted(agg.get('rig_control', {}).items()):
        add('  {:34s} n={:6d}  {:8.2f} mm  sd {:.2f} mm'
            .format(k, v['n'], v['mean_mm'], v['sd_mm']))
    add('')
    add('BIAS PROFILE b(z), pooled (mm, relative to the apex end)')
    zg = agg['even_y']['pooled_profile_grid_mm']
    for i in range(0, len(zg), 2):
        add('  z={:6.0f}   b_x {:+6.2f} +-{:.2f}   b_y {:+6.2f} +-{:.2f}'
            .format(zg[i], agg['even_x']['profile_z_mean_mm'][i],
                    agg['even_x']['profile_z_sd_mm'][i],
                    agg['even_y']['profile_z_mean_mm'][i],
                    agg['even_y']['profile_z_sd_mm'][i]))
    return '\n'.join(lines) + '\n'


def write_plot(records, agg, path):
    """The four panels the verdict is read off. Optional — needs matplotlib."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    ts = [r for r in records if 'parity' in r]
    bags = sorted({r['bag'] for r in ts})
    colours = {b: 'C{}'.format(i % 10) for i, b in enumerate(bags)}
    fig, ax = plt.subplots(2, 2, figsize=(13, 9))
    for rec in ts:
        p = rec['parity']
        for col, name in ((0, 'y'), (1, 'x')):
            ax[0][col].plot(p['even_' + name], p['z_mm'], lw=0.8,
                            color=colours[rec['bag']], alpha=0.7)
    for col, name in ((0, 'y'), (1, 'x')):
        ax[0][col].set_xlabel('even part of {} = b_{}(z) rel. apex [mm]'
                              .format(name, name))
        ax[0][col].set_ylabel('height [mm]')
        ax[0][col].grid(alpha=0.3)
        ax[0][col].axvline(0, color='k', lw=0.6)
    for rec in ts:
        tau = np.asarray(rec['_rows_t']) - rec['t_apex']
        c = rec['poly']['y']['coef']
        resid = np.asarray(rec['_rows_y']) - (c[0] + c[1] * tau + c[2] * tau ** 2
                                              + c[3] * tau ** 3)
        ax[1][0].plot(tau, resid, lw=0.6, color=colours[rec['bag']], alpha=0.6)
    ax[1][0].set_xlabel('tau = t - t_apex [s]')
    ax[1][0].set_ylabel('y residual after a cubic in tau [mm]')
    ax[1][0].grid(alpha=0.3)
    ax[1][1].plot(range(len(ts)), [r['branch']['delta_mms'][1] for r in ts],
                  'o-', label='raw branch delta v_y')
    ax[1][1].plot(range(len(ts)), [r['corrected_delta_mms'][1] for r in ts],
                  's-', label='after removing the arc\'s own even part')
    ax[1][1].set_xlabel('arc #')
    ax[1][1].set_ylabel('v_y(desc) - v_y(asc) [mm/s]')
    ax[1][1].legend(fontsize=8)
    ax[1][1].grid(alpha=0.3)
    fig.suptitle('the lateral branch artefact is an even (height-locked) '
                 'position bias of {:.1f} mm, not a force'
                 .format(agg['even_y']['amplitude_mm']))
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    return path


# ═════════════════════════════════════════════════════════════════════════════
# Self-check — synthetic arcs with a KNOWN answer, both signs
# ═════════════════════════════════════════════════════════════════════════════

def synth_track(*, bias_per_mm=0.0, odd_accel_c=0.0, apex_shift_mm=0.0,
                truncate_post_s=None, vy=-25.0, dt=0.005, plane_mm=809.08):
    """A synthetic tracked arc with a KNOWN injected defect.

    * ``bias_per_mm`` — a height-locked lateral bias ``b(z) = k.(z - z_release)``
      applied to y. EVEN in tau by construction, because z is.
    * ``odd_accel_c`` — a lateral acceleration ``a_y = c.vz``, i.e. Magnus about
      a horizontal axis. Integrating twice about the apex gives
      ``dy = -c.g.tau^3/6``: purely ODD, and the negative control this probe
      must not mistake for a bias.
    * ``apex_shift_mm`` — raises the release point, so donors and recipients in
      the held-out test do not share an arc.
    * ``truncate_post_s`` — cuts the descending branch, which is how coverage
      asymmetry is manufactured on demand.
    """
    rel_plane = (plane_mm - float(M.hw.HAND_CATCH_OFFSET_MM)
                 + M.HAND_THROW_OFFSET_MM)
    rel_pos = np.array([150.0, -120.0, rel_plane + apex_shift_mm])
    rel_vel = np.array([60.0, float(vy), 4436.0])
    arc = M.synth_arc(rel_pos, rel_vel, plane_mm=plane_mm, t_release=1000.0,
                      dt=dt)
    fit = M.fit_ballistic(arc, trim_floor_mm=1e9)
    t_apex = fit[2] + float(fit[1][2]) / M.GRAVITY_MMS2
    rows = []
    for t, x, y, z in arc:
        tau = t - t_apex
        if truncate_post_s is not None and tau > truncate_post_s:
            continue
        rows.append((t, x,
                     y + bias_per_mm * (z - rel_plane)
                     - odd_accel_c * M.GRAVITY_MMS2 * tau ** 3 / 6.0,
                     z))
    arr = np.array(rows, dtype=float)
    refit = M.fit_ballistic([tuple(r) for r in arr], trim_floor_mm=1e9)
    return _track(arr, refit[1], refit[2], release_plane_mm=rel_plane,
                  fit_vel=[float(v) for v in refit[1]],
                  fit_rms_mm=float(refit[4]), n=int(refit[3]),
                  n_candidate_rows=len(arr), n_ambiguous_frames=0,
                  truth_vy_mms=float(vy))


def self_check() -> int:
    """Bag-free acceptance. Every case has a two-sided twin.

    The point of a two-sided design here is specific: a probe that only ever sees
    real data with a real bias cannot tell "my decomposition works" from "the
    data happens to look like that". So a KNOWN bias is injected and must be
    recovered, and a KNOWN force is injected and must NOT be reported as a bias.
    """
    fails, n = [], [0]

    def check(name, got, want):
        n[0] += 1
        if got != want:
            fails.append('{}: got {!r}, want {!r}'.format(name, got, want))

    # ── 1. A clean arc: no bias, no force ────────────────────────────────────
    clean = synth_track()
    par = parity(clean)
    check('a clean arc yields a parity decomposition', par is not None, True)
    check('a clean arc has a flat EVEN part',
          bool(np.max(np.abs(par['even_y'])) < 1.0), True)
    br = branch_velocities(clean)
    check('a clean arc has no branch delta',
          bool(abs(br['delta_mms'][1]) < 1.0), True)
    far = par['tau_s'] > ODD_READ_TAU_S
    check('the ODD part of a clean arc IS the true lateral velocity',
          bool(abs(float(np.median(par['odd_y'][far])) - clean['truth_vy_mms'])
               < 1.0), True)

    # ── 2. A KNOWN height-locked bias must be recovered as EVEN ──────────────
    k = 0.02                                   # mm of y bias per mm of height
    biased = synth_track(bias_per_mm=k)
    pb = parity(biased)
    bb = branch_velocities(biased)
    check('a height-locked bias SPLITS the branch velocities',
          bool(bb['delta_mms'][1] < -50.0), True)
    # The even part at the lowest mirrored height must equal k.(z - z_apex), and
    # this is an EXACT prediction, not a tolerance around a measurement.
    z_lo = float(pb['z_mm'].max()) if pb['z_mm'][0] > pb['z_mm'][-1] \
        else float(pb['z_mm'].min())
    z_hi = float(np.median(pb['z_mm'][:5]))
    predicted = k * (z_lo - z_hi)
    measured = float(pb['even_y'][int(np.argmin(np.abs(pb['z_mm'] - z_lo)))])
    check('the EVEN part reproduces the injected b(z) to under 1 mm',
          bool(abs(measured - predicted) < 1.0), True)
    # The whole-arc estimate is NOT exact — it keeps the coverage leak, measured
    # here at 1.5 mm/s on a 22 mm bias. What matters is the RATIO: the branch
    # estimates sit ~44 mm/s off the truth, so the whole-arc fit is ~30x closer.
    # Asserting the ratio rather than an absolute is what makes this case fail
    # if a lateral component ever goes back to a branch fit.
    err_whole = abs(biased['fit_vel'][1] - biased['truth_vy_mms'])
    err_branch = min(abs(bb[side]['vel'][1] - biased['truth_vy_mms'])
                     for side in ('asc', 'desc'))
    check('the whole-arc fit returns the TRUE lateral velocity to ~1 mm/s',
          bool(err_whole < 2.0), True)
    check('...an order better than EITHER branch does',
          bool(err_branch > 10.0 * err_whole), True)
    pol = poly_fit(biased)['y']
    check('a bias is significant in the QUADRATIC term',
          bool(abs(pol['coef'][2]) / pol['se'][2] > 10.0), True)
    check('...and NOT in the cubic',
          bool(abs(pol['coef'][3]) / pol['se'][3] < 3.0), True)

    # ── 3. A KNOWN force must NOT be reported as a bias (the control) ────────
    # c chosen to put ~200 mm/s^2 of lateral acceleration at |vz| = 4400 mm/s,
    # i.e. the same scale as the artefact the real corpus shows.
    forced = synth_track(odd_accel_c=200.0 / 4400.0)
    pf = parity(forced)
    check('a lateral FORCE leaves the EVEN part flat',
          bool(np.max(np.abs(pf['even_y'])) < 1.0), True)
    polf = poly_fit(forced)['y']
    check('a force is significant in the CUBIC term',
          bool(abs(polf['coef'][3]) / polf['se'][3] > 10.0), True)
    check('...and NOT in the quadratic',
          bool(abs(polf['coef'][2]) / polf['se'][2] < 3.0), True)
    bf = branch_velocities(forced)
    check('a force does NOT split the branches the way a bias does',
          bool(abs(bf['delta_mms'][1]) < 10.0), True)

    # ── 4. HELD OUT: a profile fitted on a DIFFERENT arc must transfer ───────
    recs = []
    for i, shift in enumerate((0.0, 60.0, -40.0)):
        tr = synth_track(bias_per_mm=k, apex_shift_mm=shift)
        p = parity(tr)
        rec = {'bag': 'synthetic', 'index': i, 'cup_xyz': [150.0, -120.0, 0.0],
               't_apex': tr['t_apex'], 'apex_z_mm': float(tr['rows'][:, 3].max()),
               'release_plane_mm': tr['release_plane_mm'],
               'parity': {kk: [float(v) for v in p[kk]] for kk in p},
               'branch': branch_velocities(tr)}
        for key, col in (('_rows_t', 0), ('_rows_x', 1), ('_rows_y', 2),
                         ('_rows_z', 3)):
            rec[key] = [float(v) for v in tr['rows'][:, col]]
        recs.append(rec)
    z_lo = max(r['release_plane_mm'] + 80.0 for r in recs)
    z_hi = min(r['apex_z_mm'] - 80.0 for r in recs)
    zgrid = np.arange(z_lo, z_hi, Z_GRID_MM)
    raw = abs(recs[0]['branch']['delta_mms'][1])
    corrected = abs(apply_donor_profile(recs[0], recs[1:], zgrid)[1])
    check('a bias profile fitted on OTHER arcs collapses the branch delta',
          bool(corrected < raw / 5.0), True)
    # The negative twin: donors carrying NO bias must not "fix" anything, or the
    # held-out test would be measuring the correction machinery, not the room.
    donors_clean = []
    for shift in (60.0, -40.0):
        tr = synth_track(apex_shift_mm=shift)
        p = parity(tr)
        donors_clean.append({
            'parity': {kk: [float(v) for v in p[kk]] for kk in p},
            'release_plane_mm': tr['release_plane_mm'],
            'apex_z_mm': float(tr['rows'][:, 3].max())})
    null = abs(apply_donor_profile(recs[0], donors_clean, zgrid)[1])
    check('...while UNBIASED donors leave it alone', bool(null > raw * 0.9),
          True)

    # ── 5. Coverage asymmetry drives the whole-arc leak ──────────────────────
    sym = leak_into_whole_arc(biased, pb)
    cut = synth_track(bias_per_mm=k, truncate_post_s=0.15)
    lop = leak_into_whole_arc(cut, parity(cut))
    check('symmetric coverage leaks almost nothing into the whole-arc fit',
          bool(abs(sym['leak_vy_mms']) < 2.0), True)
    check('a truncated branch is CONFESSED by coverage_asym_s',
          bool(abs(lop['coverage_asym_s']) > M.COVERAGE_ASYM_MAX_S), True)
    # Measured on these arcs: leak 1.40 -> 3.30 mm/s for coverage_asym 0.008 ->
    # -0.152 s. The leak grows, but only 2.4x for a 20x asymmetry — which is the
    # numerical statement of the honest limit in `leak_into_whole_arc`'s
    # docstring: the first moment is a truncation guard, not a leak predictor.
    check('...and a truncated branch really does leak more',
          bool(abs(lop['leak_vy_mms']) > 2.0 * abs(sym['leak_vy_mms'])), True)
    check('the probe and the miner share ONE coverage definition',
          bool(abs(sym['coverage_asym_s']
                   - (biased['t_ref'] - biased['t_apex'])) < 1e-12), True)

    # ── 6. The force bound is derived, not asserted ──────────────────────────
    fb = force_bounds(9.4)
    check('a measured vertical odd channel implies a finite drag scale',
          bool(1.0 < fb['aero_scale_vs_nominal'] < 10.0), True)
    check('...an order more spin than the ball carries (~0.5 rev/s) would be '
          'needed for Magnus to reach the observed lateral scale',
          bool(fb['spin_rev_s_needed_implied'] > 3.0), True)
    # The calibration round-trips: feed it the odd_z the NOMINAL drag predicts
    # and it must return exactly nominal. A sign or normalisation slip in
    # `odd_z_raw`'s integration-error subtraction fails here and nowhere else.
    rt = force_bounds(fb['odd_z_predicted_at_k_nominal_mms'])
    check('and the drag calibration round-trips to exactly nominal',
          bool(abs(rt['aero_scale_vs_nominal'] - 1.0) < 1e-9), True)

    for f in fails:
        print('FAIL  {}'.format(f))
    print('{}/{} self-check cases pass'.format(n[0] - len(fails), n[0]))
    return 1 if fails else 0


# ═════════════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════════════

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--bag', action='append', default=[],
                    help='bag directory under --root (repeatable); default: '
                         'the three mocap-bearing E-1 bags')
    ap.add_argument('--root', default=DEFAULT_ROOT)
    ap.add_argument('--robot', default='jugglebot')
    ap.add_argument('--plane', type=float, default=None,
                    help='fit plane in GLOBAL mm (default: the 8a cup plane, '
                         '{:.1f})'.format(M.DEFAULT_PLANE_MM))
    ap.add_argument('--refresh-cache', action='store_true',
                    help='re-decode the bags instead of using the pickles')
    ap.add_argument('--plot', action='store_true',
                    help='also write the four-panel PNG (needs matplotlib)')
    ap.add_argument('--self-check', action='store_true',
                    help='synthetic acceptance, no bag needed')
    args = ap.parse_args(argv)

    if args.self_check:
        return self_check()

    bags = args.bag or list(DEFAULT_BAGS)
    os.makedirs(OUT_DIR, exist_ok=True)
    records = []
    for bag in bags:
        try:
            records.extend(analyse_bag(bag, root=args.root,
                                       plane_mm=args.plane,
                                       refresh=args.refresh_cache,
                                       robot=args.robot))
        except (IOError, OSError) as exc:
            print('ERROR  {}: {}'.format(bag, exc))
            return 1
    agg = aggregate(records)
    if records:
        agg['rig_control'] = rig_control(
            load_cache(bags[0], root=args.root))

    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = os.path.join(OUT_DIR, 'mocap_parity_bias_{}'.format(stamp))
    text = summary_text(records, agg)
    with open(base + '.txt', 'w') as fh:
        fh.write(text)
    print(text)
    for rec in records:
        if 'parity' not in rec:
            print('  {} #{}: SKIP {}'.format(rec['bag'][-8:], rec['index'],
                                             rec.get('skipped', 'no track')))
    # BEFORE the raw rows are dropped — they are what the residual panel plots.
    if args.plot and any('parity' in r for r in records):
        print('wrote {}'.format(write_plot(records, agg, base + '.png')))
    # The per-sample rows are ~150 kB per arc and reproducible from the bag;
    # everything a later pass needs is already in `parity` and `aggregate`.
    for rec in records:
        for key in ('_rows_t', '_rows_x', '_rows_y', '_rows_z'):
            rec.pop(key, None)
    with open(base + '.json', 'w') as fh:
        json.dump({'arcs': records, 'aggregate': agg, 'bags': bags,
                   'object_radius_mm_assumed': BALL_RADIUS_MM,
                   'object_mass_kg_assumed': BALL_MASS_KG}, fh)
    print('wrote {}.txt and {}.json'.format(base, base))
    return 0


if __name__ == '__main__':
    sys.exit(main())
