#!/usr/bin/env python3
"""Offline FK-convergence verdict for a recorded session's rosbag.

WHAT IT CHECKS
--------------
Replays every ``/robot_state`` encoder vector and every ``/leg_setpoint_echo``
commanded vector from a rosbag through the PRODUCTION
``jugglebot.motion.ik_solver.leg_lengths_to_pose`` and reports, per topic:

    n            samples decoded
    def_raise    RuntimeErrors under the production default criterion
                 -> **this is the verdict: it must be 0**
    hist_raise   RuntimeErrors under the historical absolute-only criterion
                 (``rtol=0, stall_ceiling_mm=0``)
                 -> **must be > 0 for the verdict to mean anything**; it is
                    how many vectors in this session actually reached the
                    round-off floor.  If it is 0 the session never exercised
                    the failure region and a `def_raise == 0` PASS is vacuous.
    hist_res     residual range of those historical failures (mm)
    worst_res    worst residual accepted under the default (mm)
    max_iters    worst Newton iteration count under the default

``hist_raise`` is the load-bearing cross-check.  Without it a clean session
(one whose poses all happen to land under 1e-10 mm) reads identically to a
genuinely fixed one.

``hist_raise`` is a LOWER BOUND, not a reproduction of the deployed old code.
The knobs restore the historical *acceptance test*; they cannot switch off the
post-loop honest-residual re-check, which is unconditional.  So a vector whose
final Newton step happens to land under 1e-10 mm is counted as passing here
where the pre-2026-07-25 code raised.  Do not quote ``hist_raise`` as "what the
old code did on this session" — quote it as "at least this many vectors reached
the round-off floor", which is all the vacuity check needs.

WHY THIS EXISTS
---------------
``leg_lengths_to_pose``'s convergence test was a bare absolute residual in mm.
The residual is ``|leg_vector| - init_leg_length``, a difference of two
650-870 mm quantities, so its achievable floor scales with the absolute leg
length and the Jacobian conditioning — a fixed 1e-10 mm is not reachable
everywhere in the workspace.  Consequence on hardware: 26 consecutive
``seed FK failed ... not streaming until a valid state`` ERRORs in 286 ms on
2026-07-24 09:08:55, one more on 2026-07-25 15:24:29, and — measured with this
probe — **514 of 28953 ``/robot_state`` vectors (1.775 %) and 37 of 10453
``/leg_setpoint_echo`` vectors (0.354 %) of the 2026-07-25 15:17:48 session**
unreconstructable at the shipped default.  Every one of those rejected poses
was accurate to <= 4.60e-13 mm.

The failure is **spatially clustered, not uniformly rare**, which is why it
presents as a burst rather than as background noise.  Jittering the encoder by
+-1 dead-band (8.6e-3 mm) around the 2026-07-24 burst pose makes **96.8 %** of
neighbouring vectors fail; around the 2026-07-25 15:24:29 seed pose, 70.2 %.
At ``z=50`` or ``z=110`` flat: 0 %.  So whether the robot happens to be parked
inside a floor-limited pocket is what decides whether seeding works at all, and
re-seeding every 10 ms inside such a pocket is retrying where ~70-97 % of
samples fail — recovery needs the encoder to drift OUT of the pocket.

RELATED
-------
* Plan:  ``plans/archived/fk-convergence-tolerance.md`` (Phases 0-1)
* Test:  ``tests/motion/test_fk_convergence.py`` — its two hardware fixtures
  were extracted with this probe's decode path (bags
  ``2026-07-25_15-22-50`` ``/robot_state`` @1784957069.071425 and
  ``2026-07-25_15-17-48`` ``/leg_setpoint_echo`` sample 2166).
* Contract: the *FK CONVERGENCE CRITERION* block in
  ``ros_ws/src/jugglebot/jugglebot/motion/ik_solver.py``.

SAFETY / SCOPE
--------------
Strictly offline and read-only: it opens ``.mcap`` files and nothing else.
Never starts a node, never opens a socket, never touches hardware.  Does NOT
need ROS2 — ``mcap_ros2`` decodes the schemas embedded in the bag itself.

USAGE
-----
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/fk_convergence_bag_check.py                 # newest bag
    python tools/probes/fk_convergence_bag_check.py --bag ~/Desktop/rosbags/2026-07-25_15-17-48
    python tools/probes/fk_convergence_bag_check.py --all --match 2026-07-25
    python tools/probes/fk_convergence_bag_check.py --json          # -> temp/probes/

Exit code 0 = every checked topic had ``def_raise == 0``; 1 = at least one
FK reconstruction still fails (or a bag could not be read).  A vacuous PASS
(``hist_raise == 0`` everywhere) is reported loudly and exits 0 with a
``VACUOUS`` banner — it is not a failure, but it is not evidence either.
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

try:
    from mcap_ros2.reader import read_ros2_messages
except ImportError as exc:                                       # pragma: no cover
    sys.exit(f"ERROR: mcap_ros2 not importable ({exc}).\n"
             "       activate the project venv "
             "(source ~/Desktop/PDJ_venv/venv/bin/activate)")

from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import leg_lengths_to_pose, pose_to_leg_lengths

DEFAULT_ROOT = os.path.expanduser('~/Desktop/rosbags')
OUT_DIR = os.path.join(_REPO, 'temp', 'probes')

#: The pre-2026-07-25 ACCEPTANCE TEST, expressed through the back-compatible
#: knobs — not the pre-2026-07-25 control flow.  See the ``hist_raise`` note in
#: the module docstring: the post-loop re-check has no off switch, so this is a
#: lower bound on what the deployed old code rejected.
HISTORICAL = dict(rtol=0.0, stall_ceiling_mm=0.0)

ROBOT_STATE = '/robot_state'
SETPOINT_ECHO = '/leg_setpoint_echo'


# ---------------------------------------------------------------------------
# bag discovery
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
# vector extraction
# ---------------------------------------------------------------------------

def _extract(bag_dir, geom):
    """-> {topic: (n_samples, ndarray of leg extensions in mm)}."""
    rs, echo = [], []
    for path in sorted(glob.glob(os.path.join(bag_dir, '*.mcap'))):
        for m in read_ros2_messages(path, topics=[ROBOT_STATE, SETPOINT_ECHO]):
            if m.channel.topic == ROBOT_STATE:
                states = getattr(m.ros_msg, 'motor_states', None)
                if states is not None and len(states) >= 6:
                    rs.append([float(states[i].pos_estimate) for i in range(6)])
            else:
                data = list(m.ros_msg.data)
                if len(data) >= 6:
                    echo.append(data[:6])
    out = {}
    for topic, rows in ((ROBOT_STATE, rs), (SETPOINT_ECHO, echo)):
        if rows:
            out[topic] = np.asarray(rows, dtype=float) / geom.mm_to_rev
    return out


# ---------------------------------------------------------------------------
# the check
# ---------------------------------------------------------------------------

def _check_vectors(exts, geom):
    n = int(exts.shape[0])
    def_raise = hist_raise = 0
    hist_res = []
    worst_res = 0.0
    max_iters = 0
    for ext in exts:
        try:
            pos, rot, _ = leg_lengths_to_pose(ext, geom)
            max_iters = max(max_iters, leg_lengths_to_pose.last_iterations)
            worst_res = max(worst_res,
                            float(np.max(np.abs(pose_to_leg_lengths(pos, rot, geom)
                                                - ext))))
        except RuntimeError:
            def_raise += 1
        try:
            leg_lengths_to_pose(ext, geom, **HISTORICAL)
        except RuntimeError as exc:
            hist_raise += 1
            try:
                hist_res.append(float(str(exc).split('max residual: ')[1]
                                      .split(' mm')[0]))
            except (IndexError, ValueError):
                pass
    return dict(n=n, def_raise=def_raise, hist_raise=hist_raise,
                hist_res_min=min(hist_res) if hist_res else None,
                hist_res_max=max(hist_res) if hist_res else None,
                worst_res=worst_res, max_iters=max_iters)


def check_bag(bag_dir, geom):
    row = dict(bag=os.path.basename(bag_dir), topics={}, error=None)
    try:
        vectors = _extract(bag_dir, geom)
    except Exception as exc:                                     # noqa: BLE001
        row['error'] = f'{type(exc).__name__}: {exc}'
        return row
    if not vectors:
        row['error'] = (f'no {ROBOT_STATE} or {SETPOINT_ECHO} vectors '
                        f'(legacy sqlite3 bag, or neither topic recorded)')
        return row
    for topic, exts in sorted(vectors.items()):
        row['topics'][topic] = _check_vectors(exts, geom)
    return row


# ---------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------

_HDR = ('{:<26} {:<19} {:>7} {:>9} {:>10} {:>21} {:>11} {:>9}'
        .format('bag', 'topic', 'n', 'def_rai', 'hist_rai',
                'hist_res min..max mm', 'worst_res', 'max_it'))


def _print_row(row):
    if row['error']:
        print(f"{row['bag']:<26} ERROR: {row['error']}")
        return
    for topic, r in sorted(row['topics'].items()):
        if r['hist_res_min'] is None:
            band = '-'
        else:
            band = f"{r['hist_res_min']:.2e}..{r['hist_res_max']:.2e}"
        flag = '' if r['def_raise'] == 0 else '  <== FAIL'
        print('{:<26} {:<19} {:>7} {:>9} {:>10} {:>21} {:>11.2e} {:>9}{}'
              .format(row['bag'], topic, r['n'], r['def_raise'],
                      r['hist_raise'], band, r['worst_res'], r['max_iters'],
                      flag))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', help='bag directory (default: newest under --root)')
    ap.add_argument('--root', default=DEFAULT_ROOT, help='rosbag root directory')
    ap.add_argument('--all', action='store_true', help='sweep every bag under --root')
    ap.add_argument('--match', help='with --all: only bags whose name contains this')
    ap.add_argument('--json', action='store_true',
                    help=f'also write a JSON summary to {OUT_DIR}/')
    args = ap.parse_args(argv)

    if not os.path.isdir(args.root):
        sys.exit(f"ERROR: --root {args.root} is not a directory")

    if args.all:
        bags = _bag_dirs(args.root)
        if args.match:
            bags = [b for b in bags if args.match in os.path.basename(b)]
    elif args.bag:
        bags = [os.path.expanduser(args.bag)]
    else:
        newest = _newest_bag(args.root)
        if newest is None:
            sys.exit(f"ERROR: no bags with .mcap files under {args.root}")
        bags = [newest]
    if not bags:
        sys.exit("ERROR: no bags matched")

    geom = StewartGeometry()
    print(_HDR)
    print('-' * len(_HDR))
    rows = []
    t0 = time.time()
    for bag in bags:
        row = check_bag(bag, geom)
        rows.append(row)
        _print_row(row)

    failed = sum(1 for r in rows if r['error']
                 or any(t['def_raise'] for t in r['topics'].values()))
    exercised = sum(1 for r in rows
                    if any(t['hist_raise'] for t in r['topics'].values()))
    print('-' * len(_HDR))
    print(f"{len(rows)} bag(s) in {time.time() - t0:.1f} s")
    if failed:
        print(f"VERDICT: FAIL — {failed} bag(s) still have FK reconstructions "
              f"that raise at the production default (or could not be read)")
    elif exercised == 0:
        print("VERDICT: VACUOUS — no bag contained a vector that the historical "
              "absolute-only criterion rejected, so a clean result here is not "
              "evidence the fix works.  Check a session with more workspace "
              "coverage.")
    else:
        print(f"VERDICT: PASS — 0 FK reconstruction failures at the production "
              f"default, and {exercised} bag(s) contained vectors the "
              f"historical criterion rejected (so the check was meaningful)")

    if args.json:
        os.makedirs(OUT_DIR, exist_ok=True)
        out = os.path.join(OUT_DIR,
                           f'fk_convergence_bag_check_'
                           f'{time.strftime("%Y%m%d_%H%M%S")}.json')
        with open(out, 'w') as fh:
            json.dump(dict(root=args.root, rows=rows), fh, indent=2)
        print(f"wrote {out}")

    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
