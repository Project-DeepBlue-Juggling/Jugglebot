#!/usr/bin/env python3
"""Offline levelling-frame verdict for a recorded session's rosbag.

WHAT IT CHECKS
--------------
Forward-kinematically reconstructs the **commanded** platform pose from every
``/leg_setpoint_echo`` vector (the *accepted* leg setpoints echoed by
``teensy_bridge_node``) and reports the commanded-tilt **plateau structure** of
the session: every window where the commanded ``rx``/``ry`` sits still, with its
start time, duration and value in degrees.

Given the session's levelling offset (``--offset TILT_X TILT_Y``, radians, from
the orchestrator's ``Gravity offset published: [...]`` log line) it turns that
structure into a verdict on contract **C-LEVEL-1**
(``ros_ws/docs/levelling_frame.md``):

    park_rx / park_ry   the longest plateau — where the platform RESTS
                        -> PASS iff within --tol of (-tilt_x, -tilt_y) in deg.
                           This is the whole fix in one number: before the
                           2026-07-25 migration the platform parked at plan-frame
                           rx = 0 while corrected surfaces asked for -0.7788 deg;
                           after it, every surface parks in the same frame.
                           THE PARK IS THE ONLY GATED QUANTITY.
    settle_rx           the DEEPEST plateau (furthest from the park in the
                        correction's own direction), expected at
                        park_rx * (1 + overshoot/|tilt|) because ``build_catch``
                        ramps a tilt-through-seat residual — see THE CAVEAT.
    peak_above_park     max(rx) - park_rx over the window. REPORTED, NEVER gated
                        — see THE SECOND CAVEAT. Post-fix this quantity equals the
                        platform's PHYSICAL peak tilt against gravity, and it is
                        LARGER than the pre-fix reading of the same name.
    implied_lead_s      peak_above_park / the closed-form arrival-twist slope. A
                        cross-check on the through-seat model: it should match the
                        toss's actual catch lead. REPORTED, never gated.

THE CAVEAT — READ BEFORE SCORING A SESSION
------------------------------------------
    *** SUPERSEDED FOR CAPTURES RECORDED AFTER 2026-07-26. ***
    C-CATCH-1 (``ros_ws/docs/catch_arrival_contract.md``) landed and closed BOTH
    caveats below: ``build_catch`` now takes the GRAVITY-REFERENCED receive tilt
    as its own argument, so a gravity-level catch gets a ZERO arrival twist. On a
    post-fix capture the commanded ``rx`` IS flat across a catch goal, the settle
    lands exactly on the target (0.0000 deg off gravity, not 0.3008), and
    ``peak_above_park`` reads the requested displacement rather than a swing. The
    two caveats are kept verbatim because they are what made C-CATCH-1 necessary,
    and because they remain the correct reading of every capture from the
    C-LEVEL-1-only era (2026-07-25 to 2026-07-26) — including the reference bag
    this probe's self-check is built on. Score the catch reach itself with
    ``tools/probes/catch_reach_replay.py``, which prints the pre/post delta.

The commanded ``rx`` will **NOT** be flat across a catch goal, and the plan's
original "flat to +-0.05 deg across the whole goal" criterion is unreachable.
``planner.build_catch`` reads ``catch_pose[3:5]`` as "the receive tilt" and ramps
a residual rate through the seat along it. That premise holds only while the
commanded frame IS the gravity frame. With a correction loaded, a *gravity-level*
catch arrives as a non-zero plan-frame tilt — the correction itself — so the
through-seat always engages and settles the platform

    0.5 * 0.07 rad/s * 0.15 s = 0.005250 rad = 0.3008 deg

**off gravity-level, in the correction's own direction**, at ball contact. Closed
form for the 2026-07-25 offset: settle = -1.078408 deg rx / -0.095775 deg ry,
which is *exactly* what that session's bag recorded (-1.0784 / -0.0958). The
un-levelled 15:04:35 baseline reads flat for the mirror reason: with no
correction the level catch target has ``tmag == 0``, which disables the
through-seat entirely.

So the 0.3008 deg residual — and the ~16 mm catch error it implies at 3.93 m/s
over 0.8 s — is **not** closed by C-LEVEL-1. It is a separate frame defect in
``build_catch``'s through-seat aim, pinned as a characterisation test in
``tests/ros/test_levelling_frame.py::test_catch_through_seat_aims_off_the_gravity_referenced_receive_tilt``
— **closed 2026-07-26 by C-CATCH-1**, which does change commanded motion at ball
contact on every catch including the shipping reload path (settle delta there:
rx +0.021086 deg, ry +0.004297 deg; seat aim rotated 4.0997 deg).

THE SECOND CAVEAT — WHY ``peak_above_park`` IS NOT A GATE
---------------------------------------------------------
The same through-seat produces the *visible swing* — "the platform slowly tilts
back, then forward" — and **C-LEVEL-1 does not remove it.** The reach carries a
specified non-zero ARRIVAL TWIST (``rate * tdir``), so even a reach with zero net
tilt displacement must excurt and come back. With ``p0 == p1`` the quintic reduces
exactly to ``v1 * T * phi(s)``, ``phi = -4s^3 + 7s^4 - 3s^5``, ``|phi|`` maximal
``16/81 = 0.1975309`` at ``s = 2/3``. So

    peak_above_park  =  (16/81) * rate * |tdir_x| * lead
                     =  0.789132 deg per second of catch lead
                        (2026-07-25 offset: rate 0.07 rad/s, |tdir_x| 0.996079)

**exactly linear in the catch lead** — verified against the production planner at
leads 0.8 / 1.2 / 2.0 / 3.7 / 5.0 s to 4 dp (2026-07-26).

Two consequences, both of which the first version of this probe got wrong:

* Post-fix ``peak_above_park`` is **larger** than pre-fix, not smaller. Pre-fix the
  park sat 0.7788 deg high, so subtracting it hid part of the swing. Measured on
  the reference bag's 3.70 s lead: pre-fix ``+2.3204`` (plan frame, park 0), post-fix
  ``+2.9198``. Physically the platform improves only from a ``+3.0992`` peak tilt
  against gravity to ``+2.9198`` — 0.18 deg of a 2.9 deg swing.
* Any threshold on it is a threshold on the *catch lead*. A gate at the pre-fix
  ``+3.099`` fires on a perfectly healthy system at any lead >= 3.93 s, and sits
  only 0.18 deg away at the lead the reference session actually ran. It is
  therefore REPORTED and never gated; the frame question is the park plateau, which
  is lead-independent.

Removing the swing means changing ``build_catch``'s arrival twist — see
``plans/parked/catch-reach-degenerate-overshoot.md``, not this plan. **That is
what C-CATCH-1 did on 2026-07-26**: on a post-fix capture ``peak_above_park`` for
a level catch is the requested displacement (0 for a fully level goal), not
``0.789132 * lead``. The linear-in-the-lead model above still describes every
C-LEVEL-1-era capture, which is what this instrument is for.

WHY THIS EXISTS
---------------
``trajectory_node`` applied the levelling correction on some pose-ingest paths
and not others, so "level" meant two different things inside one node: the toss's
positioning ``go_to_pose`` parked at plan-frame ``rx = 0`` while the pre-tilt
``catch/dynamic_target`` asked for ``rx = -0.7788 deg``, and ``build_catch``
planned a min-jerk reach between the two frames arriving 0.7 s before release.
Five tosses, reliably reproduced, on 2026-07-25 15:17:48. What that cost is the
PARK: the platform rested 0.78 deg off gravity and the two surfaces disagreed
about where "level" was. It is not the swing (see THE SECOND CAVEAT).

SELF-CHECK
----------
``--self-check`` scores three synthetic sessions built through the PRODUCTION
planner — pre-fix (must FAIL), post-fix (must PASS), and post-fix contaminated by
a long pre-``go_home`` ACTIVATE hold (must FAIL *with the ambiguity note*). An
instrument validated only against the broken shape will score a working fix as a
failure at the bench, which is worse than having no instrument. Guarded in CI by
``tests/motion/test_levelling_probe.py``.

RELATED
-------
* Plan:     ``plans/parked/levelling-frame-contract.md`` (Phases 1-2, 4)
* Contract: ``ros_ws/docs/levelling_frame.md``
* Tests:    ``tests/ros/test_levelling_frame.py``, ``tests/motion/test_levelling.py``
* Runbook:  ``tests/hardware/session_anomaly_fixes.md`` section LVL

SAFETY / SCOPE
--------------
Strictly offline and read-only: it opens ``.mcap`` files and nothing else. Never
starts a node, never opens a socket, never touches hardware. Does NOT need ROS2 —
``mcap_ros2`` decodes the schemas embedded in the bag itself.

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    # newest bag, structure only (no verdict without --offset):
    python tools/probes/levelling_tilt_bag_check.py
    # the verdict, with the session's published offset:
    python tools/probes/levelling_tilt_bag_check.py \
        --bag ~/Desktop/rosbags/2026-07-26_HH-MM-SS \
        --offset 0.013592347421588673 0.001207157476773584 --json
    # prove the instrument itself still reads both shapes correctly:
    python tools/probes/levelling_tilt_bag_check.py --self-check

Score a window that starts AFTER the session's first ``go_home`` (``--t0``): the
post-ACTIVATE hold is commanded at plan-frame ``rx ~ 0`` and is *correctly*
uncorrected (a seed is never corrected — C-LEVEL-1's second half), so a long one
can win the "longest plateau" vote and read as the pre-fix frame. The probe says
so when it detects the shape, but ``--t0`` removes the ambiguity outright.

Exit code 0 = PASS (or structure-only, no ``--offset``); 1 = the park plateau is
outside ``--tol`` of the correction, or the bag could not be read.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import time

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'))

try:
    import numpy as np
except ImportError:                                              # pragma: no cover
    sys.exit("ERROR: numpy not importable — activate the project venv "
             "(source ~/Desktop/PDJ_venv/venv/bin/activate)")

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import leg_lengths_to_pose, rot_matrix_to_rotvec

DEFAULT_ROOT = os.path.expanduser('~/Desktop/rosbags')
OUT_DIR = os.path.join(_REPO, 'temp', 'probes')
SETPOINT_ECHO = '/leg_setpoint_echo'

#: ``planner._CATCH_TILT_OVERSHOOT_FRAC * <the session's seat rate>
#: * build_catch(tilt_decay_s=0.15)`` — the tilt-through-seat overshoot, in rad.
#: Kept as a literal rather than imported so this probe stays a *record* of what
#: the numbers were when the session was scored; if the planner constants move,
#: the mismatch should be visible, not silently absorbed. They have since moved:
#: ``planner._CATCH_TILT_THROUGH_RATE_RADPS`` ships at **0.0** as of 2026-07-26, so
#: the 0.07 below is history, and deliberately so — this probe scores a capture
#: recorded before that.
THROUGH_SEAT_OVERSHOOT_RAD = 0.5 * 0.07 * 0.15

#: The arrival twist magnitude the 2026-07-25 session ran at. Formerly
#: ``planner._CATCH_TILT_THROUGH_RATE_RADPS``, which now ships at 0.0; same
#: literal-not-imported reasoning as above, and line 559 passes it EXPLICITLY so
#: the rebuild stays faithful to the recording rather than to today's default.
THROUGH_SEAT_RATE_RADPS = 0.07

#: ``max |phi|`` of the quintic's arrival-twist basis ``phi = -4s^3+7s^4-3s^5``,
#: attained at ``s = 2/3``. With ``p0 == p1`` (a zero-net-displacement reach) the
#: whole tilt excursion IS ``v1 * T * phi(s)``, so the peak is exactly linear in
#: the catch lead. Verified against ``planner.build_catch`` at leads 0.8 / 1.2 /
#: 2.0 / 3.7 / 5.0 s to 4 dp, 2026-07-26.
ARRIVAL_TWIST_BASIS_PEAK = 16.0 / 81.0

#: Pre-fix reference: bag 2026-07-25_15-17-48, toss #4 (ball 11). PLAN-FRAME
#: values — the park sat 0.7788 deg off gravity, which is the defect.
PRE_FIX_PARK_DEG = 0.0
PRE_FIX_PEAK_DEG = 2.3204
PRE_FIX_SETTLE_DEG = -1.0784
#: That session's correction (``-tilt_x`` in degrees). The PHYSICAL tilt against
#: gravity is the plan-frame value MINUS this, because commanding this value is
#: what "level" means once the correction is loaded.
PRE_FIX_CORRECTION_RX_DEG = -0.77878414
#: The pre-fix PHYSICAL peak tilt against gravity — the like-for-like number to
#: compare a post-fix ``peak_above_park`` against, since post-fix the park IS
#: gravity-level. NOT the pre-fix ``peak_above_park``, which was +2.3204.
PRE_FIX_PHYSICAL_PEAK_DEG = PRE_FIX_PEAK_DEG - PRE_FIX_CORRECTION_RX_DEG
#: The catch lead that reproduces the reference bag's peak, from the linear model.
PRE_FIX_LEAD_S = 3.70


# ---------------------------------------------------------------------------
# bag discovery (mirrors fk_convergence_bag_check.py)
# ---------------------------------------------------------------------------

def _bag_dirs(root):
    out = []
    for name in sorted(os.listdir(root)):
        path = os.path.join(root, name)
        if os.path.isdir(path) and glob.glob(os.path.join(path, '*.mcap')):
            out.append(path)
    return out


def _newest_bag(root):
    bags = _bag_dirs(root)
    if not bags:
        return None
    return max(bags, key=lambda b: max(os.path.getmtime(m)
                                       for m in glob.glob(os.path.join(b, '*.mcap'))))


# ---------------------------------------------------------------------------
# FK reconstruction of the COMMANDED pose
# ---------------------------------------------------------------------------

def reconstruct(bag_dir, geom, t0=None, t1=None):
    """-> (t_rel seconds, poses Nx6 [x,y,z,rx,ry,rz] mm/rad, n_fk_failures).

    ``mcap_ros2`` is imported HERE, not at module scope, so the scoring half of
    this probe (:func:`score_series`) can be exercised — by ``--self-check`` and by
    ``tests/motion/test_levelling_probe.py`` — without the bag reader installed.
    """
    try:
        from mcap_ros2.reader import read_ros2_messages
    except ImportError as exc:                                   # pragma: no cover
        sys.exit(f"ERROR: mcap_ros2 not importable ({exc}).\n"
                 "       activate the project venv "
                 "(source ~/Desktop/PDJ_venv/venv/bin/activate)")
    times, revs = [], []
    for path in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        for m in read_ros2_messages(path, topics=[SETPOINT_ECHO]):
            data = list(m.ros_msg.data)
            if len(data) >= 6:
                times.append(m.log_time_ns * 1e-9)
                revs.append(data[:6])
    if not times:
        return None, None, 0
    times = np.asarray(times, dtype=float)
    times -= times[0]
    revs = np.asarray(revs, dtype=float)
    if t0 is not None:
        keep = times >= t0
        times, revs = times[keep], revs[keep]
    if t1 is not None:
        keep = times <= t1
        times, revs = times[keep], revs[keep]
    exts = revs / np.asarray(geom.mm_to_rev, dtype=float)

    poses, keep_t, failures = [], [], 0
    for t, ext in zip(times, exts):
        try:
            pos, rot, _J = leg_lengths_to_pose(ext, geom)
        except RuntimeError:
            failures += 1
            continue
        poses.append(np.concatenate([pos, rot_matrix_to_rotvec(rot)]))
        keep_t.append(t)
    if not poses:
        return None, None, failures
    return np.asarray(keep_t), np.asarray(poses), failures


# ---------------------------------------------------------------------------
# plateau detection
# ---------------------------------------------------------------------------

def plateaus(times, values_deg, tol_deg, min_s):
    """Maximal runs where ``values_deg`` stays inside a ``tol_deg`` band.

    Greedy: extend the run while max-min of the run stays under the band. A
    plateau is the physically meaningful unit here — the platform holds a
    commanded tilt between profiled moves — and it is robust to the sample rate
    and to where the operator started recording, unlike a fixed time window.
    """
    out = []
    i = 0
    n = len(values_deg)
    while i < n:
        lo = hi = values_deg[i]
        j = i
        while j + 1 < n:
            nlo, nhi = min(lo, values_deg[j + 1]), max(hi, values_deg[j + 1])
            if nhi - nlo > tol_deg:
                break
            lo, hi, j = nlo, nhi, j + 1
        dur = float(times[j] - times[i])
        if dur >= min_s:
            out.append(dict(t_start=float(times[i]), t_end=float(times[j]),
                            duration_s=dur,
                            value_deg=float(np.mean(values_deg[i:j + 1])),
                            span_deg=float(hi - lo), n=int(j - i + 1)))
        i = j + 1
    return out


# ---------------------------------------------------------------------------
# the check
# ---------------------------------------------------------------------------

def score_series(times, poses, offset=None, tol_deg=0.05, plateau_tol_deg=0.02,
                 plateau_min_s=0.3, row=None):
    """Score a commanded-pose series. The whole verdict, with no bag I/O.

    Split out from :func:`check_bag` so both shapes the instrument must read —
    the one it must FLAG and the one it must ACCEPT — are exercisable without a
    recorded session (``--self-check``, ``tests/motion/test_levelling_probe.py``).
    """
    row = dict(row or {})
    row.setdefault('error', None)
    row.setdefault('verdict', None)
    rx_deg = np.degrees(poses[:, 3])
    ry_deg = np.degrees(poses[:, 4])
    row['n'] = int(len(times))
    row['duration_s'] = float(times[-1] - times[0])

    pl_rx = plateaus(times, rx_deg, plateau_tol_deg, plateau_min_s)
    row['plateaus_rx'] = pl_rx
    if not pl_rx:
        row['error'] = ('no rx plateau of >= '
                        f'{plateau_min_s} s within {plateau_tol_deg} deg — the '
                        'platform never held still; widen --plateau-tol or check '
                        'the window')
        return row

    park = max(pl_rx, key=lambda p: p['duration_s'])
    # ry over exactly the park window, so the two axes are read at the same instant.
    mask = (times >= park['t_start']) & (times <= park['t_end'])
    park_ry = float(np.mean(ry_deg[mask]))

    tilt_x, tilt_y = (None, None) if offset is None else (float(offset[0]),
                                                          float(offset[1]))
    exp_rx = None if tilt_x is None else -np.degrees(tilt_x)
    # The catch settle is the plateau FURTHEST FROM THE PARK IN THE CORRECTION'S
    # OWN DIRECTION, not the last one: after C-LEVEL-1 the session ends parked at
    # the corrected neutral, so "last" would report the park again. The direction
    # has to come from the offset rather than a hard-coded ``min()``: ``min()``
    # picks the most-negative plateau, which is the settle only while the
    # correction tilts rx negative (true for this machine's mount, not in general
    # — for ``tilt_x < 0`` it would return the park and report a spurious
    # settle_err). Falls back to ``min()`` when no offset was supplied.
    if exp_rx is None or exp_rx == 0.0:
        settle = min(pl_rx, key=lambda p: p['value_deg'])
    else:
        sgn = 1.0 if exp_rx < 0 else -1.0
        settle = min(pl_rx, key=lambda p: sgn * p['value_deg'])

    row['park_rx_deg'] = park['value_deg']
    row['park_ry_deg'] = park_ry
    row['park_duration_s'] = park['duration_s']
    row['settle_rx_deg'] = settle['value_deg']
    row['peak_rx_deg'] = float(np.max(rx_deg))
    row['peak_above_park_deg'] = float(np.max(rx_deg) - park['value_deg'])
    row['pos_span_mm'] = [float(np.ptp(poses[:, k])) for k in range(3)]

    if offset is None:
        row['verdict'] = 'STRUCTURE-ONLY'
        return row

    exp_ry = -np.degrees(tilt_y)
    tmag = float(np.hypot(tilt_x, tilt_y))
    scale = (1.0 + THROUGH_SEAT_OVERSHOOT_RAD / tmag) if tmag > 1e-12 else 1.0
    row['expected_park_rx_deg'] = exp_rx
    row['expected_park_ry_deg'] = exp_ry
    row['expected_settle_rx_deg'] = exp_rx * scale
    row['park_err_rx_deg'] = row['park_rx_deg'] - exp_rx
    row['park_err_ry_deg'] = row['park_ry_deg'] - exp_ry
    row['settle_err_rx_deg'] = row['settle_rx_deg'] - row['expected_settle_rx_deg']

    # peak_above_park, interpreted: with p0 == p1 the reach IS v1*T*phi(s), so the
    # peak is exactly linear in the catch lead. Inverting it gives the lead the
    # capture implies, which is the honest cross-check on the through-seat model —
    # unlike a fixed threshold, which is really a threshold on the lead.
    slope = 0.0
    if tmag > 1e-12:
        slope = float(np.degrees(ARRIVAL_TWIST_BASIS_PEAK
                                 * THROUGH_SEAT_RATE_RADPS * abs(tilt_x) / tmag))
    row['arrival_twist_deg_per_s'] = slope
    row['implied_lead_s'] = (row['peak_above_park_deg'] / slope
                             if slope > 1e-12 else None)

    ok = (abs(row['park_err_rx_deg']) <= tol_deg
          and abs(row['park_err_ry_deg']) <= tol_deg)
    row['verdict'] = 'PASS' if ok else 'FAIL'

    # A FAIL whose park reads ~0 has two very different causes and the runbook
    # sends the operator somewhere different for each. Say which one this is.
    row['park_ambiguous'] = False
    if not ok and abs(row['park_rx_deg']) <= tol_deg:
        others = [p for p in pl_rx
                  if abs(p['value_deg'] - exp_rx) <= tol_deg
                  and p['duration_s'] >= plateau_min_s]
        if others:
            row['park_ambiguous'] = True
            row['park_ambiguous_note'] = (
                f"{len(others)} other plateau(s) totalling "
                f"{sum(p['duration_s'] for p in others):.1f} s sit within "
                f"{tol_deg} deg of the expected park. The longest plateau may be "
                f"the pre-go_home ACTIVATE hold, which is commanded at plan-frame "
                f"rx ~ 0 and is CORRECTLY uncorrected (a seed is never corrected). "
                f"Re-run with --t0 set after the session's first go_home before "
                f"scoring this as the pre-fix frame.")
    return row


def check_bag(bag_dir, geom, offset=None, tol_deg=0.05, plateau_tol_deg=0.02,
              plateau_min_s=0.3, t0=None, t1=None):
    row = dict(bag=os.path.basename(bag_dir), error=None, verdict=None)
    try:
        times, poses, failures = reconstruct(bag_dir, geom, t0, t1)
    except Exception as exc:                                     # noqa: BLE001
        row['error'] = f'{type(exc).__name__}: {exc}'
        return row
    if times is None:
        row['error'] = (f'no usable {SETPOINT_ECHO} vectors '
                        f'(legacy sqlite3 bag, or topic not recorded)')
        return row
    row['fk_failures'] = int(failures)
    return score_series(times, poses, offset=offset, tol_deg=tol_deg,
                        plateau_tol_deg=plateau_tol_deg,
                        plateau_min_s=plateau_min_s, row=row)


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------

def _print_row(row, tol_deg):
    print(f"\nbag: {row['bag']}")
    if row['error']:
        print(f"  ERROR: {row['error']}")
        return
    print(f"  samples {row['n']}  span {row['duration_s']:.1f} s  "
          f"FK failures {row.get('fk_failures', 0)}")
    print(f"  commanded position span (x,y,z) mm = "
          f"{['%.2f' % v for v in row['pos_span_mm']]}")
    print(f"  {'t_start':>9} {'dur_s':>7} {'rx_deg':>10} {'span_deg':>9}")
    for p in row['plateaus_rx']:
        print(f"  {p['t_start']:>9.2f} {p['duration_s']:>7.2f} "
              f"{p['value_deg']:>10.4f} {p['span_deg']:>9.4f}")
    print(f"  park  rx {row['park_rx_deg']:+.4f} deg  ry {row['park_ry_deg']:+.4f} deg "
          f"(longest plateau, {row['park_duration_s']:.1f} s)")
    print(f"  settle rx {row['settle_rx_deg']:+.4f} deg (plateau furthest from the "
          f"park in the correction's direction — the catch settle)")
    print(f"  peak rx {row['peak_rx_deg']:+.4f} deg "
          f"({row['peak_above_park_deg']:+.4f} above park)")
    if row['verdict'] == 'STRUCTURE-ONLY':
        print("  VERDICT: STRUCTURE-ONLY — pass --offset TILT_X TILT_Y (rad, from "
              "the orchestrator's 'Gravity offset published' line) to score it")
        return
    print(f"  expected park   rx {row['expected_park_rx_deg']:+.4f} "
          f"ry {row['expected_park_ry_deg']:+.4f} deg  "
          f"(err {row['park_err_rx_deg']:+.4f} / {row['park_err_ry_deg']:+.4f}, "
          f"tol +-{tol_deg})   <- THE GATE")
    print(f"  expected settle rx {row['expected_settle_rx_deg']:+.4f} deg  "
          f"(err {row['settle_err_rx_deg']:+.4f}) — the through-seat overshoot, "
          f"NOT closed by C-LEVEL-1")
    # REPORT-only, in ONE convention: post-fix the park IS gravity-level, so
    # peak_above_park IS the physical peak tilt against gravity. The like-for-like
    # pre-fix number is therefore the pre-fix PHYSICAL peak (+3.099), not the
    # pre-fix peak_above_park (+2.3204) — those differ by exactly the correction.
    lead = row.get('implied_lead_s')
    lead_txt = 'n/a' if lead is None else f'{lead:.2f} s'
    basis = ('= the PHYSICAL peak tilt vs gravity' if row['verdict'] == 'PASS'
             else '(plan-frame: the park is NOT gravity-level in this capture)')
    print(f"  peak above park {row['peak_above_park_deg']:+.4f} deg {basis}. "
          f"REPORT ONLY, never a gate:")
    print(f"      arrival-twist model {row['arrival_twist_deg_per_s']:.4f} deg per "
          f"second of catch lead  =>  implied lead {lead_txt}")
    print(f"      pre-fix, same convention: {PRE_FIX_PHYSICAL_PEAK_DEG:+.4f} deg at "
          f"a {PRE_FIX_LEAD_S:.2f} s lead (plan-frame {PRE_FIX_PEAK_DEG:+.4f} from a "
          f"{PRE_FIX_PARK_DEG:+.4f} park). C-LEVEL-1 does NOT remove this swing.")
    if row.get('park_ambiguous'):
        print(f"  NOTE: {row['park_ambiguous_note']}")
    print(f"  VERDICT: {row['verdict']}")


# ---------------------------------------------------------------------------
# self-check — the instrument read against BOTH shapes
# ---------------------------------------------------------------------------

SELF_CHECK_OFFSET = (0.013592347421588673, 0.001207157476773584)


def synth_series(offset, *, corrected_park, lead_s=3.70, park_s=6.0,
                 settle_s=2.0, tail_s=8.0, activate_hold_s=0.0, dt=0.025):
    """A synthetic commanded-pose series, built through the PRODUCTION planner.

    ``corrected_park=True`` is the post-C-LEVEL-1 shape (positioning and catch
    agree); ``False`` reproduces the 2026-07-25 defect (park at plan-frame
    ``rx = 0``, catch target corrected). ``activate_hold_s`` prepends a hold at the
    UNCORRECTED activate pose — the post-ACTIVATE, pre-``go_home`` window, which is
    correctly uncorrected and can out-last the real park.

    **Both shapes are C-LEVEL-1-era records, and this probe scores them as such.**
    The arrival rate is REQUESTED explicitly (``tilt_through_rate_radps``) and the
    seat is aimed at the plan-frame tilt (``receive_tilt=corrected[3:5]``), which
    is what ``build_catch`` did before ``ros_ws/docs/catch_arrival_contract.md``
    (C-CATCH-1) landed. Left implicit, the shipping planner would now aim the seat
    at the gravity-referenced receive tilt and bound the manufactured rate, so the
    ``corrected_park=True`` case would carry NO through-seat at all — the synthetic
    series would stop resembling the sessions this instrument exists to score, and
    its FLAG/ACCEPT discrimination would be scoring physics no capture contains.
    A post-C-CATCH-1 capture's catch reach is monotone with no swing; score that
    with ``tools/probes/catch_reach_replay.py``, which reports the delta directly.

    Returns ``(times, poses)`` in the same units :func:`reconstruct` produces.
    """
    from jugglebot.motion import levelling
    from jugglebot.motion.trajectory import planner
    from jugglebot.motion.trajectory.limits import TrajectoryLimits
    import jugglebot.hardware_config as hw

    geom = StewartGeometry()
    limits = TrajectoryLimits.from_config(hw)
    R = levelling.correction_from_offset(*offset)
    neutral = np.array([0.0, 0.0, 170.0, 0.0, 0.0, 0.0])
    corrected = levelling.correct_pose(neutral, R)
    park = corrected if corrected_park else neutral
    plan, _report = planner.build_catch(
        (park, np.zeros(6), np.zeros(6)), corrected, float(lead_s), limits, geom,
        settle_hold_s=float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S),
        receive_tilt=corrected[3:5],
        tilt_through_rate_radps=THROUGH_SEAT_RATE_RADPS)

    times, poses, t = [], [], 0.0

    def hold(pose, dur):
        nonlocal t
        for _ in range(int(round(dur / dt))):
            times.append(t)
            poses.append(np.asarray(pose, dtype=float))
            t += dt

    hold(neutral, activate_hold_s)
    hold(park, park_s)
    n = int(round(plan.total_duration / dt))
    for k in range(n):
        times.append(t)
        poses.append(np.asarray(plan.state_at(k * dt)[0], dtype=float))
        t += dt
    hold(plan.final_pose, settle_s)
    hold(park, tail_s)
    return np.asarray(times), np.asarray(poses)


def self_check(tol_deg=0.05):
    """Score the three shapes the instrument must distinguish. -> exit code."""
    cases = (
        ('post-fix (the shape it must ACCEPT)',
         dict(corrected_park=True), 'PASS', False),
        ('pre-fix  (the shape it must FLAG)',
         dict(corrected_park=False), 'FAIL', False),
        ('post-fix + a 30 s pre-go_home ACTIVATE hold',
         dict(corrected_park=True, activate_hold_s=30.0), 'FAIL', True),
    )
    bad = 0
    for label, kwargs, want_verdict, want_ambiguous in cases:
        times, poses = synth_series(SELF_CHECK_OFFSET, **kwargs)
        row = score_series(times, poses, offset=SELF_CHECK_OFFSET,
                           tol_deg=tol_deg, row=dict(bag=f'SYNTH {label}'))
        got_ambiguous = bool(row.get('park_ambiguous'))
        ok = row['verdict'] == want_verdict and got_ambiguous == want_ambiguous
        bad += 0 if ok else 1
        print(f"\n{'OK  ' if ok else 'BAD '} {label}")
        print(f"     want verdict {want_verdict}, ambiguity-note "
              f"{want_ambiguous}  ->  got {row['verdict']}, {got_ambiguous}")
        print(f"     park rx {row['park_rx_deg']:+.4f} (expected "
              f"{row['expected_park_rx_deg']:+.4f})   settle rx "
              f"{row['settle_rx_deg']:+.4f}   peak above park "
              f"{row['peak_above_park_deg']:+.4f}   implied lead "
              f"{row['implied_lead_s']:.2f} s")
    print(f"\nSELF-CHECK: {'PASS' if bad == 0 else f'FAIL ({bad} case(s))'}")
    return 0 if bad == 0 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', help='bag directory (default: newest under --root)')
    ap.add_argument('--root', default=DEFAULT_ROOT, help='rosbag root directory')
    ap.add_argument('--offset', nargs=2, type=float, metavar=('TILT_X', 'TILT_Y'),
                    help='the session levelling offset in RADIANS — without it '
                         'the probe reports structure but renders no verdict')
    ap.add_argument('--tol', type=float, default=0.05,
                    help='park-plateau tolerance in degrees (default 0.05)')
    ap.add_argument('--plateau-tol', type=float, default=0.02,
                    help='band width that defines a plateau, degrees')
    ap.add_argument('--plateau-min-s', type=float, default=0.3,
                    help='shortest run counted as a plateau, seconds')
    ap.add_argument('--t0', type=float, help='window start, s from bag start')
    ap.add_argument('--t1', type=float, help='window end, s from bag start')
    ap.add_argument('--json', action='store_true',
                    help=f'also write a JSON summary to {OUT_DIR}/')
    ap.add_argument('--self-check', action='store_true',
                    help='score three synthetic sessions built through the '
                         'production planner (post-fix must PASS, pre-fix must '
                         'FAIL, ACTIVATE-contaminated must FAIL with the note) '
                         'and exit — no bag needed')
    args = ap.parse_args(argv)

    if args.self_check:
        return self_check(tol_deg=args.tol)

    bag = args.bag
    if bag is None:
        if not os.path.isdir(args.root):
            sys.exit(f"ERROR: --root {args.root} is not a directory")
        bag = _newest_bag(args.root)
        if bag is None:
            sys.exit(f"ERROR: no bag directories with *.mcap under {args.root}")
    if not os.path.isdir(bag):
        sys.exit(f"ERROR: --bag {bag} is not a directory")

    geom = StewartGeometry()
    row = check_bag(bag, geom, offset=args.offset, tol_deg=args.tol,
                    plateau_tol_deg=args.plateau_tol,
                    plateau_min_s=args.plateau_min_s, t0=args.t0, t1=args.t1)
    _print_row(row, args.tol)

    if args.json:
        os.makedirs(OUT_DIR, exist_ok=True)
        out = os.path.join(
            OUT_DIR, f"levelling_tilt_{time.strftime('%Y%m%d_%H%M%S')}.json")
        with open(out, 'w', encoding='utf-8') as fh:
            json.dump(row, fh, indent=2)
        print(f"\nJSON -> {out}")

    return 0 if row['verdict'] in ('PASS', 'STRUCTURE-ONLY') else 1


if __name__ == '__main__':
    sys.exit(main())
