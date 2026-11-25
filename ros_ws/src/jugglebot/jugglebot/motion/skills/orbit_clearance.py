"""Ball-to-ball clearance of JP-14 Centre and orbit (``centre_orbit``), offline.

Pure geometry on the REAL compiled Schedule (no solve, milliseconds): both
balls' nominal centre paths on a 1 ms grid.  Used by the probe
``tools/probes/centre_orbit_clearance.py`` and by the logbook's odd-count numbers.  Nominal paths only: landing scatter (sd
about 20 mm per axis on the robot) eats the margin.  Logbook:
``2026-11-25-jp14-centre-and-orbit-compile.md``.
"""
from __future__ import annotations

import dataclasses
import math
import numpy as np

import jugglebot.hardware_config as hw
from jugglebot.motion.skills import nominate as nm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills.segments import CATCH, THROW

G_MM_S2 = 9810.0
BALL_DIAMETER_MM = 2.0 * float(hw.JUGGLING_BALL_RADIUS_MM)


def flights_of(sched):
    """``[(ball, t_release, xy_from, t_catch, xy_to, throw_index)]`` read off
    the Schedule's own skills (launch THROW, carried throws)."""
    f = float(sched.flight_s)
    out = []
    k = 0
    for sk in sched.skills:
        if sk.kind == THROW:
            out.append((sk.ball_id, sk.t_abs_s, sk.site.cup_mm[:2],
                        sk.t_abs_s + f, sk.target.cup_mm[:2], k))
            k += 1
        elif sk.kind == CATCH and sk.then_throw is not None:
            tt = sk.then_throw
            out.append((sk.ball_id, tt.t_release_abs_s,
                        tt.release_site_or(sk.site).cup_mm[:2],
                        tt.t_release_abs_s + f, tt.target.cup_mm[:2], k))
            k += 1
    out.sort(key=lambda r: r[1])
    return out


def ball_path(flights, ball, t, f, apex_mm, inflight_xy, t_first_catch):
    """(T, 3) nominal centre path of ``ball`` on grid ``t`` (Catch plane z=0)."""
    pos = np.full((len(t), 3), np.nan)
    mine = [r for r in flights if r[0] == ball]
    v0 = 4.0 * apex_mm / f          # symmetric arc: z(f / 2) = apex
    segs = []
    if ball == 1 and inflight_xy is not None:
        # the pre-existing flight (the Feed) lands at the first CATCH
        segs.append((t_first_catch - f, inflight_xy, t_first_catch,
                     inflight_xy))
    segs += [(r[1], r[2], r[3], r[4]) for r in mine]
    for i, (tr, a, tc, b) in enumerate(segs):
        m = (t >= tr) & (t <= tc)
        s = t[m] - tr
        lam = s / f
        pos[m, 0] = a[0] + (b[0] - a[0]) * lam
        pos[m, 1] = a[1] + (b[1] - a[1]) * lam
        pos[m, 2] = v0 * s - 0.5 * G_MM_S2 * s * s
        if i + 1 < len(segs):     # held in the cup until the next release
            m2 = (t > tc) & (t < segs[i + 1][0])
            pos[m2] = [b[0], b[1], 0.0]
    return pos


def clearance(radius_mm, n, apex_m=0.95, dwell_s=0.27, stop=False,
              columns=False, n_throws=None):
    """``stop=False``: the steady lap, up to the Stop's release (the Stop
    is aimed to land ON the held ball, so the two balls close by design, as
    in Columns).  ``stop=True``: the whole Schedule.  ``columns=True``: the
    Columns control at separation ``radius_mm`` (one Columns lap = ``2 n +
    1`` Throws), the shape every centre-orbit transit repeats.
    ``n_throws`` None = one closed lap plus the Stop (``2 n + 2``, EVEN: Ball
    2's Throw onto Ball 1 at the centre); an ODD count (refused by the
    Pattern, measured here only) ends on Ball 1's Stop onto Ball 2 at O1."""
    cfg = nm.NominationConfig(
        pattern='columns' if columns else sc.CENTRE_ORBIT, apex_m=apex_m,
        separation_mm=radius_mm, dwell_s=dwell_s, orbit_n=n,
        n_throws=(2 * n + 1) if columns else (2 * n + 2 if n_throws is None
                                              else n_throws))
    odd = (not columns and n_throws is not None and int(n_throws) % 2 != 0)
    pat = nm.build_pattern(dataclasses.replace(cfg, n_throws=int(n_throws) + 1)
                           if odd else cfg)
    if odd:
        # an ODD count is REFUSED by Pattern (ORBIT_ODD_THROW_COUNT, owner
        # 2026-11-25); it is built here as the next even count and re-counted
        # ONLY to measure how close the refused Stop would have been
        object.__setattr__(pat, 'n_throws', int(n_throws))
    sched = sc.compile_columns(pat, t0_abs_s=10.0)
    f = float(sched.flight_s)
    fl = flights_of(sched)
    first_catch_b = min(s.t_abs_s for s in sched.skills
                        if s.kind == CATCH and s.ball_id == 1)
    t = np.arange(10.0, fl[-1][3] + 1e-9, 0.001)
    pa = ball_path(fl, 0, t, f, apex_m * 1000.0, None, None)
    pb = ball_path(fl, 1, t, f, apex_m * 1000.0, pat.sites[1].cup_mm[:2],
                   first_catch_b)
    ok = ~(np.isnan(pa[:, 0]) | np.isnan(pb[:, 0]))
    if not stop:
        ok &= t < fl[-1][1]
    d = np.linalg.norm(pa - pb, axis=1)
    d[~ok] = np.inf
    i = int(np.argmin(d))
    dxy = np.linalg.norm(pa[:, :2] - pb[:, :2], axis=1)
    dxy[~ok] = np.inf
    j = int(np.argmin(dxy))
    beat = (t[i] - 10.0) / float(sched.beat_s)
    throw = max((r[5] for r in fl if r[1] <= t[i]), default=0)
    return dict(kind=('columns' if columns else 'orbit')
                + ('+stop' if stop else ''), radius_mm=radius_mm, n=n, apex_m=apex_m, dwell_s=dwell_s,
                chord_mm=2.0 * radius_mm * math.sin(math.pi / n),
                chord_to_centre_mm=radius_mm * math.cos(math.pi / n),
                min_3d_mm=float(d[i]), t_min_s=float(t[i] - 10.0),
                beat_at_min=beat, throw_at_min=int(throw),
                ball_a_z_mm=float(pa[i, 2]), ball_b_z_mm=float(pb[i, 2]),
                min_xy_mm=float(dxy[j]),
                margin_3d_mm=float(d[i]) - BALL_DIAMETER_MM)



