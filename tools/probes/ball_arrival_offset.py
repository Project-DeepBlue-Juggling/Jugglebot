#!/usr/bin/env python3
"""Where did each thrown ball actually ARRIVE? — the matched-offset scorer.

WHAT IT DOES
------------
For every throw announcement in a rosbag, reads the raw **mocap** marker stream
(``/mocap_data``, unlabelled markers) and reports where that ball crossed a
horizontal plane on its way in:

    x@plane, y@plane   least-squares fit of x(z) and y(z) over the DESCENDING
                       samples in [plane, plane + band], evaluated at the plane

plus a REPORT-only floor census (did a new ball come to rest on the floor after
this attempt?) as an independent cross-check on the catch/drop outcome.

WHY IT EXISTS
-------------
The 2026-07-27 validation sitting could not decide its own headline question.
Three reload attempts bounced out of a stationary 10.8 deg rim; the competing
explanation was a measured 26-39 mm Ball-Butler warm-up drift in arrival ``x``
that plateaued around the sixth throw. The two cannot be separated from that
capture because it contains **no throws at a large +x offset with a non-zero
seat rate**. The A/B that settles it compares bounce-out rates **at matched
arrival offsets** — which requires the arrival offset to be a number the session
produces, and today it is not: the sitting's own gap list records *"Nothing asks
for the ball's measured arrival point... it appears in no log line"*, and the
number that made the drop analysis possible was computed by hand, once, and is
not re-runnable.

This is that instrument. Same estimator applied to both arms of the A/B is what
makes "matched offsets" mean something.

Plan: ``plans/active/catch-reach-degenerate-overshoot.md`` (follow-on phase C).
Protocol: ``tests/hardware/session_anomaly_fixes.md`` SECTION SEAT-EXP, rows
``SEAT-EXP-4`` / ``SEAT-EXP-6`` / ``SEAT-EXP-7.1``.
Evidence base: ``logbook/2026-07-28-anomaly-fixes-validation-sitting.md``.
Consuming test: ``tests/motion/test_ball_arrival_offset_probe.py``.

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate

    # score every Ball-Butler throw in a capture
    python tools/probes/ball_arrival_offset.py \
        --bag ~/Desktop/rosbags/2026-07-27_15-39-38

    # the A/B: attempts 1-8 are the burn-in (discarded), 9-20 and 21-32 the arms
    python tools/probes/ball_arrival_offset.py --bag <BAG> \
        --discard 8 --group CONTROL=9-20 --group EXPERIMENT=21-32

    # instrument acceptance, no bag needed
    python tools/probes/ball_arrival_offset.py --self-check

Strictly offline and read-only: it opens ``.mcap`` files and nothing else. No
ROS2 needed. CSV output goes to ``temp/probes/`` (timestamped, never clobbering).

THE ESTIMATOR, AND WHY IT IS DEFINED THIS WAY
---------------------------------------------
* **Unlabelled markers only.** The ball carries no rigid body, so it is only ever
  an unlabelled marker. Labelled markers (``Platform - n``, ``Base - n``, ...)
  are excluded outright.
* **A lateral gate** (``--lateral``, default 300 mm) removes the room's fixed
  reflectors. The 2026-07-27 capture has a permanent one at
  ``(-537.1, 412.9, 1163.7)`` that sits squarely in the arrival band and would
  otherwise dominate every fit.
* **Descending branch only.** Samples are cut at the FIRST sample below the
  plane, so nothing after contact enters the fit. This matters more than it
  looks: a bounce-out's post-contact excursion is large (measured up to 70 mm in
  ``x`` on ball 6) and in the opposite direction on different balls, so a fit
  that includes it biases each track by a different amount **in a way that
  correlates with the outcome being scored**. That is the one bias an
  outcome-comparison instrument must not have.
* **A fit, not the nearest sample.** Mocap drops the ball for tens of ms at a
  time (ball 11 was lost at ``z = 1111`` and never re-acquired on the way in), so
  a nearest-sample reading is undefined exactly when a track is worst. A fit
  degrades gracefully and reports ``n``; ``n < MIN_SAMPLES`` (5) is flagged
  ``SPARSE`` rather than silently trusted.
* **``--plane`` defaults to 1000 mm**, matching the number the sitting analysis
  published, so the two are commensurable. The band above it is 300 mm by
  default: long enough for ~10 samples at 5 m/s, short enough that the track's
  curvature (the ball is on a ballistic arc, so x(z) is not exactly linear)
  contributes under a millimetre.

VALIDATED AGAINST THE SITTING (2026-07-28)
------------------------------------------
Two-sided acceptance, because an instrument that only ever agrees with itself is
not validated:

* **ACCEPT / absolute calibration.** Scored at ``--plane 1050`` against the three
  entry points the sitting published by hand for the three bounce-outs
  (``logbook/2026-07-28-anomaly-fixes-validation-sitting.md`` -> "The three
  drops"): ball 6 ``(-6.6, 16.2)`` vs published ``(-7.2, 16.1)``; ball 9
  ``(-18.5, 3.9)`` vs ``(-18.8, 3.6)``; ball 11 ``(-21.0, 7.6)`` vs
  ``(-21.5, 7.4)``. Worst disagreement **0.6 mm** — and ball 11's is the
  extrapolated one (3 samples, all above ``z = 1100``).
  **SCOPE THAT 0.6 mm CAREFULLY — it is not this estimator's accuracy.** It is
  agreement with ONE published quantity (bounce-out entry points, at
  ``--plane 1050``). Against the sitting's published ``x@z1000`` series
  ``+6.7 / -21.6 / -10.7 / -14.5 / -27.0 / -36.3 / -30.1`` this probe reads
  ``+2.8 / -8.9 / -11.5 / -11.5 / -27.3 / -36.1 / -26.9`` — attempt 2 differs by
  **12.7 mm** and attempts 1/4/7 by 3.0-3.9 mm. Two defensible estimators on the
  same sparse tracks disagree by an order more than 0.6 mm, so do NOT carry
  "0.6 mm" as a licence to treat a few-mm block-mean gap as physically real.
  That disagreement is systematic-between-estimators, not random-within-this-one
  (this probe is deterministic and both A/B arms are scored by it, so it cancels
  within a sitting) — which is the whole reason the decision rule is
  within-sitting and ``SEAT-EXP-6`` is expressed against its own standard error.
* **The population statistic reproduces.** The eleven non-bounce-out attempts
  that have a usable track (attempts 4-18) have mean ``x@1000`` **-35.11 mm**
  against the published catches' **-34.9 mm**.
* **The warm-up drift reproduces**, which is the effect the protocol's discard
  rule is sized against: attempts 1-6 read ``+2.8, -8.9, -11.5, -11.5, -27.3,
  -36.1`` mm and 7-18 sit on a plateau (mean **-38.9**, sd **9.0**, n=8).
* **Two of the four un-scorable attempts are independently expected.** Four
  announcements (8, 10, 16, 17) yield NO TRACK; the sitting records that two of
  its eighteen announcements aborted ``THROW_ABORTED_NOT_SETTLED`` so no ball
  ever left the Butler. The exact index mapping is not established here — goal
  numbering and announcement numbering differ — so this is corroboration, not a
  claim.
* **One number does NOT reproduce, deliberately recorded here rather than
  buried.** The sitting's decisive marginality is quoted as *"attempt 3 dropped
  at ``x = -10.7`` and attempt 4 caught at ``x = -14.5``, 3.8 mm apart"*. This
  estimator reads them at **-11.507** and **-11.525** — the same offset to within
  **0.02 mm**. The published pair came from an ad-hoc fit whose window is not
  recorded, and attempt 3 is the sparsest track in the capture (**3** samples,
  extrapolated from above ``z = 1100``), so it is the value most sensitive to
  that choice. The qualitative reading is unchanged and if anything sharper: a
  drop and a catch at the SAME measured arrival offset is a capture-basin edge
  sitting exactly on the warm-up excursion, which is precisely why the seat
  hypothesis and the aim hypothesis cannot be separated from that capture. It
  also means **no absolute number from the old hand analysis should be used as a
  gate** — the SEAT-EXP decision rule is within-sitting, both arms scored by this
  probe, for exactly this reason.

FLOOR CENSUS — REPORT ONLY, ON PURPOSE
--------------------------------------
The sitting scored its catch numerator by a mocap floor census because the
operator had not been asked ball-by-ball. That worked, but it is fragile: a ball
already resting on the floor is re-seen in every later window, the room has a
permanent floor reflector at ``(549, -401, 61)``, and 4 of 13 catches showed no
marker at the cup at all (pose-dependent occlusion), so cup occupancy alone
under-counted catches as 9/16. This probe counts only **new** resting clusters
(ones with no marker within ``--floor-radius`` before the attempt began) and
reports them. The SEAT-EXP protocol makes the operator's per-ball eye verdict the
primary outcome and uses this as the cross-check, never the other way round.

**Its measured failure mode, so nobody has to rediscover it.** On the reference
capture the census reads ``6, 2, 0`` new clusters for the three known
bounce-outs, i.e. it both over-counts (one rolling ball crosses several 150 mm
cells) and UNDER-counts (attempt 3's ball came to rest at ``(-8.9, 338.5)``,
**111 mm** from where attempt 2's ball had already stopped — inside the default
``--floor-radius``, so it merges and reads 0). It is a coarse corroborator and is
labelled REPORT everywhere it appears. It is not tuned to make that capture come
out right; a radius picked to separate those two balls would be fitted to one
session and would fail on the next.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import glob
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))

#: Default plane (mm) the arrival offset is quoted at. 1000 mm is what the
#: 2026-07-27 sitting analysis published, so re-scores stay commensurable.
PLANE_MM = 1000.0
#: Fit band above the plane (mm).
BAND_MM = 300.0
#: Lateral gate (mm) that removes the room's fixed reflectors.
LATERAL_MM = 300.0
#: Below this many descending samples a track is reported SPARSE.
MIN_SAMPLES = 5
#: Floor census: a marker below this height is "on the floor".
FLOOR_Z_MM = 120.0
#: Two floor markers within this distance are the same resting ball.
FLOOR_RADIUS_MM = 150.0


class Track:
    """One throw's measured arrival."""

    __slots__ = ('index', 'thrower', 'land_t', 'x', 'y', 'n', 'z_lo', 'z_hi',
                 'new_floor')

    def __init__(self, index, thrower, land_t):
        self.index = index
        self.thrower = thrower
        self.land_t = land_t
        self.x = self.y = None
        self.n = 0
        self.z_lo = self.z_hi = None
        self.new_floor = 0

    @property
    def ok(self):
        return self.x is not None

    @property
    def sparse(self):
        return self.ok and self.n < MIN_SAMPLES

    @property
    def radial(self):
        return None if not self.ok else math.hypot(self.x, self.y)


def fit_plane_crossing(samples, plane, band):
    """-> (x_at_plane, y_at_plane, n, z_lo, z_hi) or None.

    ``samples`` is an iterable of ``(t, x, y, z)`` for ONE throw window, in any
    order. Only the descending run before the first sub-plane sample is used.

    The five-tuple is the SHIPPED shape and stays that way; callers that also
    need the fit residual (the toss-record corpus's G2 track-quality guard) call
    :func:`fit_plane_crossing_full`, which this delegates to. One fit, two views
    — a second least-squares here would be a second place for the band selection
    to drift.
    """
    got = fit_plane_crossing_full(samples, plane, band)
    return None if got is None else got[:5]


def fit_plane_crossing_full(samples, plane, band):
    """-> (x_at_plane, y_at_plane, n, z_lo, z_hi, rms_mm, t_lo, t_hi) or None.

    ``rms_mm`` is the RMS of the fitted residual over the band rows, taken as
    ``hypot(dx, dy)`` per row — one number for "did x(z) and y(z) actually look
    like straight lines?". Added 2026-08-10 for the toss-record miner
    (``plans/active/toss-selftuning.md`` § 3.6.2 guard G2: fit RMS <= 3 mm); the
    2-sample degenerate case reports 0.0, which is arithmetically true and is why
    G2 pairs the RMS with a minimum sample count rather than trusting it alone.
    """
    rows = sorted(samples)
    cut = len(rows)
    for i, (_t, _x, _y, z) in enumerate(rows):
        if z < plane:
            cut = i
            break
    band_rows = [r for r in rows[:cut] if plane <= r[3] <= plane + band]
    n = len(band_rows)
    if n < 2:
        return None
    zs = [r[3] for r in band_rows]
    z_mean = sum(zs) / n
    szz = sum((z - z_mean) ** 2 for z in zs)
    if szz <= 0.0:
        return None
    out = []
    resid_sq = [0.0] * n
    for col in (1, 2):
        v_mean = sum(r[col] for r in band_rows) / n
        slope = sum((r[3] - z_mean) * (r[col] - v_mean) for r in band_rows) / szz
        out.append(v_mean + slope * (plane - z_mean))
        for i, r in enumerate(band_rows):
            resid_sq[i] += (r[col] - (v_mean + slope * (r[3] - z_mean))) ** 2
    rms = math.sqrt(sum(resid_sq) / n)
    ts = [r[0] for r in band_rows]
    return (out[0], out[1], n, min(zs), max(zs), rms, min(ts), max(ts))


def trend(values):
    """-> (n, slope_mm_per_throw, stderr) for ``x`` against attempt ORDER.

    The burn-in gate (``SEAT-EXP-2.1``) asks "has the Ball-Butler's aim stopped
    walking?", and the obvious phrasing of that — *successive-throw change under
    10 mm*, which is how the sitting first proposed it — **fires on correct
    behaviour**. Measured on the 2026-07-27 plateau, successive changes run
    ``0.3, 15.6, 0.8, 11.5, 10.0, 9.0, 6.3`` mm: three of the seven exceed 10 mm
    on a Butler that has demonstrably settled, because the throw-to-throw scatter
    is ~9-11 mm sd. A plateau is the absence of a TREND, not the absence of
    scatter.

    So the gate fits a line instead. On the same data: burn-in throws 1-6 read
    **-7.13 mm/throw**, the plateau reads **-1.75 mm/throw** (n=8) — the two are
    cleanly separated by a 3 mm/throw threshold.

    ``stderr`` is returned because it is the honest limit of the gate and the
    reason it is a pre-filter rather than the protection: at the measured
    11.4 mm sd it is **2.72 mm/throw at n=6** and **1.76 at n=8**, so a real
    residual drift of a few mm/throw is simply invisible here. That is what
    ``SEAT-EXP-6`` (matched block means) exists to catch after the fact.
    """
    n = len(values)
    if n < 3:
        return n, None, None
    x_mean = (n - 1) / 2.0
    y_mean = sum(values) / n
    sxx = sum((i - x_mean) ** 2 for i in range(n))
    slope = sum((i - x_mean) * (v - y_mean)
                for i, v in enumerate(values)) / sxx
    resid = [v - (y_mean + slope * (i - x_mean)) for i, v in enumerate(values)]
    if n <= 2:
        return n, slope, None
    s2 = sum(r * r for r in resid) / (n - 2)
    return n, slope, math.sqrt(s2 / sxx)


def _cluster_key(x, y, radius):
    return (round(x / radius), round(y / radius))


def read_bag(bag_dir, thrower, plane, band, lateral, pre_s, post_s,
             floor_z, floor_radius):
    """-> [Track], read from ``bag_dir``'s ``.mcap`` files.

    ``mcap_ros2`` is imported HERE so ``--self-check`` (and the consuming test)
    can exercise the estimator without the bag reader installed.
    """
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as exc:                                   # pragma: no cover
        sys.exit(f"ERROR: mcap_ros2 not importable ({exc}).\n"
                 "       activate the project venv "
                 "(source ~/Desktop/PDJ_venv/venv/bin/activate)")
    paths = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))
    if not paths:
        sys.exit(f'ERROR: no .mcap files under {bag_dir}')

    tracks = []
    for path in paths:
        for m in read_ros2_messages(path, topics=['/throw_announcements']):
            a = m.ros_msg
            if thrower != 'any' and str(a.thrower_name) != thrower:
                continue
            lt = a.landing_time.sec + a.landing_time.nanosec * 1e-9
            tracks.append(Track(len(tracks) + 1, str(a.thrower_name), lt))
    if not tracks:
        sys.exit(f'ERROR: no /throw_announcements from thrower {thrower!r} '
                 f'in {bag_dir}')

    windows = [[] for _ in tracks]
    floor_before = [set() for _ in tracks]
    floor_after = [set() for _ in tracks]
    lo = min(t.land_t for t in tracks) - max(pre_s, 5.0)
    hi = max(t.land_t for t in tracks) + max(post_s, 5.0)
    for path in paths:
        for m in read_ros2_messages(path, topics=['/mocap_data']):
            t = m.log_time_ns * 1e-9
            if t < lo or t > hi:
                continue
            for mk in m.ros_msg.markers:
                if mk.label:
                    continue
                px, py, pz = (float(mk.position.x), float(mk.position.y),
                              float(mk.position.z))
                for i, tr in enumerate(tracks):
                    if pz < floor_z:
                        if t < tr.land_t:
                            floor_before[i].add(
                                _cluster_key(px, py, floor_radius))
                        elif t <= tr.land_t + post_s:
                            floor_after[i].add(
                                _cluster_key(px, py, floor_radius))
                        continue
                    if abs(px) > lateral or abs(py) > lateral:
                        continue
                    if tr.land_t - pre_s <= t <= tr.land_t + post_s:
                        windows[i].append((t, px, py, pz))

    for i, tr in enumerate(tracks):
        got = fit_plane_crossing(windows[i], plane, band)
        if got:
            tr.x, tr.y, tr.n, tr.z_lo, tr.z_hi = got
        tr.new_floor = len(floor_after[i] - floor_before[i])
    return tracks


def parse_group(spec):
    """``NAME=lo-hi`` -> ``(name, lo, hi)`` (1-based, inclusive)."""
    name, _, rng = spec.partition('=')
    lo, _, hi = rng.partition('-')
    try:
        return name.strip(), int(lo), int(hi)
    except ValueError:
        sys.exit(f'ERROR: bad --group {spec!r}; want NAME=lo-hi, e.g. A=7-18')


def stats(values):
    """-> (n, mean, sd) with the sample sd (n-1); sd is None for n < 2."""
    n = len(values)
    if n == 0:
        return 0, None, None
    mean = sum(values) / n
    if n < 2:
        return n, mean, None
    var = sum((v - mean) ** 2 for v in values) / (n - 1)
    return n, mean, math.sqrt(var)


def print_table(tracks, plane, discard):
    print(f'arrival offsets at z = {plane:.0f} mm '
          f'(least-squares fit of x(z), y(z) over the descending band)\n')
    print(f'{"#":>3} {"thrower":>12} {"x_mm":>9} {"y_mm":>9} {"radial":>8} '
          f'{"n":>3} {"z_fit":>15} {"floor":>6}  note')
    for tr in tracks:
        note = []
        if tr.index <= discard:
            note.append('WARM-UP (discarded)')
        if not tr.ok:
            note.append('NO TRACK')
        elif tr.sparse:
            note.append(f'SPARSE (n<{MIN_SAMPLES}) — offset is extrapolated')
        if tr.new_floor:
            note.append(f'{tr.new_floor} new floor arrival(s)')
        if tr.ok:
            zf = f'{tr.z_lo:.0f}..{tr.z_hi:.0f}'
            print(f'{tr.index:>3} {tr.thrower:>12} {tr.x:9.1f} {tr.y:9.1f} '
                  f'{tr.radial:8.1f} {tr.n:>3} {zf:>15} {tr.new_floor:>6}  '
                  + '; '.join(note))
        else:
            print(f'{tr.index:>3} {tr.thrower:>12} {"—":>9} {"—":>9} '
                  f'{"—":>8} {tr.n:>3} {"—":>15} {tr.new_floor:>6}  '
                  + '; '.join(note))


def print_groups(tracks, groups, discard):
    if not groups:
        return
    print('\n── blocks ──')
    summary = []
    for name, lo, hi in groups:
        sel = [t for t in tracks
               if lo <= t.index <= hi and t.index > discard and t.ok]
        n, mean, sd = stats([t.x for t in sel])
        summary.append((name, lo, hi, n, mean, sd, sel))
        sd_s = 'n/a' if sd is None else f'{sd:.1f}'
        mean_s = 'n/a' if mean is None else f'{mean:.1f}'
        print(f'  {name}: attempts {lo}-{hi}  scored n={n}  '
              f'x mean {mean_s} mm  sd {sd_s} mm')
    if len(summary) == 2:
        (_n1, _l1, _h1, n1, m1, s1, _a) = summary[0]
        (_n2, _l2, _h2, n2, m2, s2, _b) = summary[1]
        if n1 and n2:
            gap = abs(m1 - m2)
            print(f'\n  MATCHED-ARRIVAL CHECK: block means differ by '
                  f'{gap:.1f} mm')
            pooled = None
            if s1 is not None and s2 is not None:
                pooled = math.sqrt(((n1 - 1) * s1 ** 2 + (n2 - 1) * s2 ** 2)
                                   / max(n1 + n2 - 2, 1))
                print(f'  pooled within-block sd {pooled:.1f} mm')
            if pooled:
                # The gap must be judged against the standard error of the
                # DIFFERENCE OF MEANS, not against the single-observation sd.
                # A fixed 5 mm gate is not a statement about matching: at the
                # measured 9-11 mm sd and n=12/arm, se_diff is 3.7-4.7 mm, so
                # two blocks flown at IDENTICAL aim exceed 5 mm about 17-28 % of
                # the time. That is a criterion that fires on correct behaviour
                # — the same defect this file's burn-in gate was rewritten to
                # remove — and it would score a good sitting INCONCLUSIVE.
                se_diff = pooled * math.sqrt(1.0 / n1 + 1.0 / n2)
                thresh = max(5.0, 2.0 * se_diff)
                print(f'  se(difference of means) {se_diff:.2f} mm  '
                      f'-> gap = {gap / se_diff:.2f} se ({gap / pooled:.2f} '
                      f'single-observation sd)')
                print(f'  SEAT-EXP-6 wants gap <= max(5.00, 2*se) = '
                      f'{thresh:.2f} mm  ->  '
                      f'{"MATCHED" if gap <= thresh else "NOT MATCHED"}')


def write_csv(tracks, plane, band, bag_dir):
    out_dir = os.path.join(_REPO, 'temp', 'probes')
    os.makedirs(out_dir, exist_ok=True)
    stamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    path = os.path.join(out_dir, f'ball_arrival_offset_{stamp}.csv')
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['attempt', 'thrower', 'landing_time_s', 'x_at_plane_mm',
                    'y_at_plane_mm', 'radial_mm', 'n_samples', 'z_fit_lo_mm',
                    'z_fit_hi_mm', 'new_floor_arrivals', 'plane_mm', 'band_mm',
                    'bag'])
        for t in tracks:
            w.writerow([
                t.index, t.thrower, f'{t.land_t:.6f}',
                '' if t.x is None else f'{t.x:.3f}',
                '' if t.y is None else f'{t.y:.3f}',
                '' if t.x is None else f'{t.radial:.3f}',
                t.n,
                '' if t.z_lo is None else f'{t.z_lo:.1f}',
                '' if t.z_hi is None else f'{t.z_hi:.1f}',
                t.new_floor, f'{plane:.1f}', f'{band:.1f}', bag_dir])
    print(f'\nwrote {path}')
    return path


# ── instrument acceptance ────────────────────────────────────────────────────

def _synthetic(x0, y0, vx, vy, v_down, z_start, dt=0.005, n=120, seed_noise=0.0):
    """A ballistic descent sampled at ``dt``, in the probe's (t, x, y, z) shape."""
    g = 9810.0
    out = []
    for i in range(n):
        t = i * dt
        out.append((t,
                    x0 + vx * t + seed_noise * ((i % 3) - 1),
                    y0 + vy * t,
                    z_start - v_down * t - 0.5 * g * t * t))
    return out


def self_check():
    """-> 0 if every case passes, 1 otherwise. Prints one line per case.

    Two-sided by construction: cases that must ACCEPT a good track and cases
    that must REFUSE / FLAG a bad one. A one-sided instrument that only ever
    says "fine" cannot be trusted to score an experiment.
    """
    bad = []

    def check(name, ok, detail):
        print(f'{"OK " if ok else "BAD"}  {name}\n     {detail}')
        if not ok:
            bad.append(name)

    # 1. ACCEPT — a clean descent's offset is recovered to sub-mm.
    #    The ball passes x = -30, y = +12 exactly at z = 1000.
    plane = 1000.0
    v_down = 5000.0
    t_cross = 0.0
    for _ in range(60):                      # solve z(t) = plane from z0 = 1600
        f = 1600.0 - v_down * t_cross - 0.5 * 9810.0 * t_cross ** 2 - plane
        fp = -v_down - 9810.0 * t_cross
        t_cross -= f / fp
    vx, vy = 200.0, -80.0
    rows = _synthetic(-30.0 - vx * t_cross, 12.0 - vy * t_cross, vx, vy,
                      v_down, 1600.0)
    got = fit_plane_crossing(rows, plane, BAND_MM)
    err = None if got is None else math.hypot(got[0] + 30.0, got[1] - 12.0)
    check('a clean descent recovers its plane crossing',
          got is not None and err < 1.0,
          f'x {got[0]:.3f} (want -30), y {got[1]:.3f} (want +12), '
          f'n {got[2]}, err {err:.3f} mm' if got else 'no fit')

    # 2. FLAG — post-contact samples must NOT enter the fit. Same track, with a
    #    violent bounce-out grafted on. This is the bias that would correlate
    #    with the outcome being scored, so it is the single most important thing
    #    this instrument must not do.
    #
    #    The graft MUST land back INSIDE the fit band (z >= plane), which is what
    #    a ball deflected upward off the rim actually does — ball 11 of the
    #    reference sitting reappeared 345 mm above its free-fall prediction. An
    #    earlier draft grafted at z = 700..739, i.e. BELOW the plane, where the
    #    band filter drops the samples whether or not the descending-branch cut
    #    exists: that made this case pass unchanged against a probe with the cut
    #    deleted (verified 2026-07-28 — a mutant with `cut = len(rows)` scored
    #    8/8 PASS, exit 0). A self-check that cannot fail on the one bias the
    #    header calls disqualifying is worse than no self-check, because
    #    SEAT-EXP-4 gates the whole scoring pass on it.
    bounced = list(rows)
    last_t = rows[-1][0]
    for i in range(1, 40):
        bounced.append((last_t + i * 0.005, 400.0, -400.0, plane + 3.0 * i))
    got_b = fit_plane_crossing(bounced, plane, BAND_MM)
    # The naive estimator this case exists to reject: same samples, but with
    # every sub-plane row stripped so the descending-branch cut can never fire.
    # That is exactly what `fit_plane_crossing` would compute without its cut.
    naive = fit_plane_crossing([r for r in bounced if r[3] >= plane],
                               plane, BAND_MM)
    moved = None if (naive is None or got_b is None) else abs(naive[0] - got_b[0])
    check('post-contact excursion is excluded from the fit',
          got_b is not None and abs(got_b[0] - got[0]) < 1e-9
          and moved is not None and moved > 1.0,
          f'with a bounce-out grafted back into the band: x {got_b[0]:.3f} '
          f'(unchanged from {got[0]:.3f}); the naive fit that kept those '
          f'samples reads {naive[0]:.3f} — {moved:.1f} mm away, so the cut is '
          f'load-bearing' if (got_b and naive) else 'no fit')

    # 3. FLAG — a sparse, high-only track still fits, and is REPORTED sparse
    #    rather than silently trusted. Ball 11 of the reference sitting is
    #    exactly this shape: mocap lost it at z = 1111 and never re-acquired it.
    high = [r for r in rows if r[3] > 1100.0]
    got_h = fit_plane_crossing(high, plane, BAND_MM)
    check('a sparse high-only track extrapolates and is countable as sparse',
          got_h is not None and abs(got_h[0] + 30.0) < 3.0
          and got_h[2] < MIN_SAMPLES + 20,
          f'x {got_h[0]:.3f} from n={got_h[2]} samples in '
          f'z {got_h[3]:.0f}..{got_h[4]:.0f}' if got_h else 'no fit')

    # 4. REFUSE — a track with nothing in the band yields no number at all,
    #    rather than a fabricated one.
    check('a track that never enters the band yields NO offset',
          fit_plane_crossing([r for r in rows if r[3] > 1500.0], plane,
                             BAND_MM) is None,
          'fit_plane_crossing returned None, as it must')

    # 5. ACCEPT — the block statistics are the ones the decision rule uses.
    n, mean, sd = stats([-10.0, -20.0, -30.0])
    check('block stats use the SAMPLE sd (n-1)',
          n == 3 and abs(mean + 20.0) < 1e-12 and abs(sd - 10.0) < 1e-12,
          f'n {n}, mean {mean:.3f}, sd {sd:.6f} (want 10.0)')

    # 6. REFUSE — a single-sample block reports no sd rather than 0.0, which
    #    would read as "perfectly repeatable" in the decision rule.
    n1, mean1, sd1 = stats([-12.0])
    check('a single-sample block reports NO sd (not zero)',
          n1 == 1 and sd1 is None, f'n {n1}, mean {mean1}, sd {sd1}')

    # 7. TWO-SIDED on real data — the burn-in gate must REFUSE the 2026-07-27
    #    warm-up and ACCEPT its plateau, using the same threshold. These are the
    #    measured x@1000 values from that capture (see the header's validation
    #    block), so this case fails loudly if the estimator or the trend fit ever
    #    stops separating the two regimes the gate is built on.
    burn_in = [2.8, -8.9, -11.5, -11.5, -27.3, -36.1]
    plateau = [-26.9, -27.2, -42.8, -42.0, -53.5, -43.5, -34.5, -40.8]
    _n_b, s_b, _e_b = trend(burn_in)
    _n_p, s_p, se_p = trend(plateau)
    check('the burn-in trend gate separates warm-up from plateau (real data)',
          abs(s_b) > 3.0 and abs(s_p) <= 3.0,
          f'2026-07-27 warm-up slope {s_b:+.2f} mm/throw (must exceed 3.00), '
          f'plateau {s_p:+.2f} (must not), plateau stderr {se_p:.2f}')

    # 8. REFUSE — the criterion the sitting FIRST proposed ("successive-throw
    #    change < 10 mm") fires on that same correct plateau. Pinned so nobody
    #    re-adopts it from the logbook prose without meeting the counterexample.
    worst = max(abs(plateau[i + 1] - plateau[i])
                for i in range(len(plateau) - 1))
    check('the rejected successive-change criterion would fire on a good plateau',
          worst > 10.0,
          f'largest successive change on a SETTLED Butler is {worst:.1f} mm — '
          f'a "< 10 mm successive change" gate would refuse it')

    print('\nSELF-CHECK: ' + ('PASS' if not bad
                              else f'FAIL ({len(bad)} case(s))'))
    return 1 if bad else 0


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Measured ball arrival offsets from a rosbag (mocap).')
    ap.add_argument('--bag', help='rosbag directory containing .mcap files')
    ap.add_argument('--thrower', default='ball_butler',
                    help="thrower_name to score, or 'any' (default ball_butler)")
    ap.add_argument('--plane', type=float, default=PLANE_MM,
                    help=f'plane height in mm (default {PLANE_MM:.0f})')
    ap.add_argument('--band', type=float, default=BAND_MM,
                    help=f'fit band above the plane in mm (default {BAND_MM:.0f})')
    ap.add_argument('--lateral', type=float, default=LATERAL_MM,
                    help='reject markers beyond this |x| or |y| in mm')
    ap.add_argument('--pre', type=float, default=1.0,
                    help='window start, seconds before announced landing')
    ap.add_argument('--post', type=float, default=3.0,
                    help='window end, seconds after announced landing')
    ap.add_argument('--discard', type=int, default=0,
                    help='mark the first N attempts as warm-up and exclude '
                         'them from block statistics')
    ap.add_argument('--group', action='append', default=[],
                    help='NAME=lo-hi block of attempts (repeatable)')
    ap.add_argument('--trend-window', type=int, default=8,
                    help='fit the burn-in trend over the last N scored '
                         'attempts (default 8; at 6 the slope stderr is '
                         '2.7 mm/throw, at 8 it is 1.8)')
    ap.add_argument('--floor-z', type=float, default=FLOOR_Z_MM)
    ap.add_argument('--floor-radius', type=float, default=FLOOR_RADIUS_MM)
    ap.add_argument('--csv', action='store_true',
                    help='also write a CSV under temp/probes/')
    ap.add_argument('--self-check', action='store_true',
                    help='instrument acceptance; no bag needed')
    args = ap.parse_args(argv)

    if args.self_check:
        return self_check()
    if not args.bag:
        ap.error('--bag is required (or use --self-check)')

    bag = os.path.expanduser(args.bag)
    tracks = read_bag(bag, args.thrower, args.plane, args.band, args.lateral,
                      args.pre, args.post, args.floor_z, args.floor_radius)
    print_table(tracks, args.plane, args.discard)
    print_groups(tracks, [parse_group(g) for g in args.group], args.discard)
    scored = [t for t in tracks if t.ok and t.index > args.discard]
    n, mean, sd = stats([t.x for t in scored])
    print(f'\n{len(tracks)} announcement(s); {n} scored past the warm-up; '
          f'x mean {"n/a" if mean is None else f"{mean:.1f}"} mm, '
          f'sd {"n/a" if sd is None else f"{sd:.1f}"} mm')

    # The PLATEAU statistic, over the same slice the trend is fitted to.
    # SEAT-EXP-2.2 (the sitting's go/no-go) asks whether THE PLATEAU sits in the
    # marginal band, and the all-scored mean above is not that number: it still
    # carries the warm-up excursion the burn-in exists to outlast, and that
    # excursion is toward +x, i.e. toward passing the gate. On the 2026-07-27
    # reference capture the all-scored mean reads -28.8 mm while the plateau is
    # -38.9 mm — a 10.1 mm optimistic bias, two thirds of the gate's whole
    # margin (band edge -15 mm, basin edge -12 mm). Printing only the biased
    # number would let a Butler that has genuinely plateaued outside the band
    # pass the go/no-go and burn the sitting on a 0-vs-0 result.
    window = [t.x for t in scored][-args.trend_window:]
    pn, pmean, psd = stats(window)
    print(f'PLATEAU (last {pn} scored): x mean '
          f'{"n/a" if pmean is None else f"{pmean:.1f}"} mm, '
          f'sd {"n/a" if psd is None else f"{psd:.1f}"} mm'
          '   <- SEAT-EXP-2.2 reads THIS line, not the all-scored mean above')
    tn, slope, se = trend(window)
    if slope is None:
        print(f'BURN-IN TREND: not computable ({tn} scored track(s); '
              f'needs 3)')
    else:
        se_s = 'n/a' if se is None else f'{se:.2f}'
        print(f'BURN-IN TREND over the last {tn} scored: '
              f'slope {slope:+.2f} mm/throw (stderr {se_s}). '
              f'SEAT-EXP-2.1 wants |slope| <= 3.00; the stderr is the gate\'s '
              f'detection floor, not a formality')
    if args.csv:
        write_csv(tracks, args.plane, args.band, bag)
    return 0 if scored else 1


if __name__ == '__main__':
    sys.exit(main())
