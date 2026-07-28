#!/usr/bin/env python3
"""Map the Tier-8b displaced-throw A→B frontier through the PRODUCTION planner.

WHAT IT ANSWERS
---------------
For a displaced self-toss (throw from A, catch at B, `single-ball-toss` Tier 8b),
four questions that decide the shipped displacement cap and the hardware ladder:

1. **How far can the deferred A→B catch reach actually go?** Not per the
   closed-form quintic bound in `toss_sequencer.reach_displacement_limit_mm` —
   per `planner.build_catch` itself, seeded at the real swing-compensated
   pre-tilt pose at A and targeted at the real receive-tilted catch pose at B,
   over the flight time as the lead. Swept across 8 directions × flight × radius.
2. **Is the closed-form bound conservative or optimistic?** Both, it turns out,
   on either side of T ≈ 0.75 s — which is why the cap must not lean on it.
3. **Where does `MAX_TILT_DEG` actually bind?** (Never, inside the workspace.)
4. **Does the DEGENERATE case plan?** B == A — zero displacement, level aim —
   is the operator's "8b subsumes 8a" expectation and the case that has never
   flown on hardware. It must produce a valid `build_move`, a valid
   `build_catch`, and a release state BITWISE equal to the Tier-8a one.

WHY IT EXISTS
-------------
The 2026-07-28 operator decision (d) ordered displaced throws across the full
±150 mm workspace. The old 70 mm cap was justified as *the intersection of the
80 mm reach envelope with the bb Rung-2a clean box* — and contract **C-REACH-1**
(`ros_ws/docs/catch_reach_envelope.md`) removed the envelope half, because the
envelope bounds unrequested drift and was never meant to cap requested reach.
That left the cap needing its own evidence rather than an inherited one.

Hardware evidence stops at 70 mm (11/11, the 2026-07-27 T4 rungs — see
`logbook/2026-07-28-anomaly-fixes-validation-sitting.md`). The sim gate
(`sim/toss_gate.py --tier 8b`) measures whether the ball is CAUGHT, which is a
physics question. This probe measures whether the platform motion is even
PLANNABLE, which is a kinematics question with an exact answer — and it runs in
seconds rather than the gate's tens of minutes.

WHAT IT UNDERPINS
-----------------
- `jugglebot_operational.toss_max_displacement_mm = 150.0` and every number in
  its YAML comment.
- `toss_sequencer.TOSS_MAX_DISPLACEMENT_MM`'s comment block and
  `reach_displacement_limit_mm`'s "conservative below / optimistic above"
  paragraph.
- `tests/ros/test_toss_sequencer.py::test_displacement_rejected` (which legs
  exercise the cap vs the closed-form bound) and
  `::test_reach_displacement_limit_closed_form` (the 0.669 s crossover).
- `ThrowTiltInfeasible`'s "still a contract, not a live constraint" claim.
- `tests/hardware/session_anomaly_fixes.md` § SECTION DISP — the ladder's
  flight-time guidance per rung comes from table A.

It is committed rather than left in `/tmp` because the cap is operator-adjustable
and the bench ladder re-runs this to re-derive the frontier whenever the session
limits (`trajectory_limits.leg_*`) move — the frontier is a function of them.

RUNNING
-------
    source ~/Desktop/PDJ_venv/venv/bin/activate
    python tools/probes/displaced_reach_frontier.py             # tables to stdout
    python tools/probes/displaced_reach_frontier.py --json      # + temp/probes/

Read-only: it plans trajectories in-process and actuates nothing.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
for _p in (os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import jugglebot.hardware_config as hw                       # noqa: E402
from jugglebot.motion.geometry import StewartGeometry        # noqa: E402
from jugglebot.motion.trajectory import planner              # noqa: E402
from jugglebot.motion.trajectory import tilt_geometry        # noqa: E402
from jugglebot.motion.trajectory import toss_release         # noqa: E402
from jugglebot.motion.trajectory.feasibility import (        # noqa: E402
    TrajectoryInfeasible,
)
from jugglebot.motion.trajectory.limits import TrajectoryLimits   # noqa: E402

GEOM = StewartGeometry()
LIMITS = TrajectoryLimits.from_config(hw)
Z_ACTIVE = float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)
SETTLE_S = float(hw.JB_TRAJ_CATCH_SETTLE_HOLD_S)
# The ballistics-side gravity (never the tracker's 9810) — the same source
# motion/trajectory/ballistics_bc uses.
GRAVITY_MMS2 = 9806.0

_R2 = math.sqrt(0.5)
DIRECTIONS = (('+x', 1.0, 0.0), ('NE', _R2, _R2), ('+y', 0.0, 1.0),
              ('NW', -_R2, _R2), ('-x', -1.0, 0.0), ('SW', -_R2, -_R2),
              ('-y', 0.0, -1.0), ('SE', _R2, -_R2))
FLIGHTS_S = (0.55, 0.60, 0.65, 0.70, 0.80, 0.95, 1.10)
RADII_MM = (70.0, 100.0, 125.0, 150.0, 175.0, 200.0, 225.0, 250.0)


def _rest_state(pose6):
    return (np.asarray(pose6, dtype=float), np.zeros(6), np.zeros(6))


def closed_form_limit_mm(flight_s: float) -> float:
    """`toss_sequencer.reach_displacement_limit_mm`, re-derived here from the
    SAME session limits so a probe run and the FSM cannot disagree silently."""
    t = float(flight_s)
    return min(LIMITS.leg_vel_mmps * t / 1.875,
               LIMITS.leg_acc_mmps2 * t * t / 5.7735,
               LIMITS.leg_jerk_mmps3 * t ** 3 / 60.0)


def reach_verdict(a_xy, b_xy, flight_s):
    """Plan the production deferred A→B reach. Returns ('OK'|<reject code>, info).

    Reproduces the production chain exactly: `compute_release_state_tilted` for
    the swing-compensated pre-tilt seed at A and the aim, then the receive tilt
    from the ARRIVAL velocity (launch velocity minus g·T, as the tracker sees
    it), then the swing-compensated catch centroid at B — which is what
    `catch_coordinator.predicted_catch_command` produces — and finally
    `planner.build_catch` with the flight as the lead.
    """
    try:
        rs = toss_release.compute_release_state_tilted(
            (float(b_xy[0]), float(b_xy[1]), Z_ACTIVE), float(flight_s),
            throw_site_xy_mm=(float(a_xy[0]), float(a_xy[1])))
    except toss_release.ThrowTiltInfeasible as exc:
        return 'TILT_CLAMP', {'required_deg': exc.required_deg}
    v_arrival = np.asarray(rs.launch_vel_mms, dtype=float).copy()
    v_arrival[2] -= GRAVITY_MMS2 * float(flight_s)
    rx, ry = tilt_geometry.tilt_to_receive(v_arrival)
    shift = tilt_geometry.cup_lateral_shift_mm(
        rx, ry, cup_z_mm=float(rs.catch_point_global_mm[2]))
    catch_pose = np.array([float(b_xy[0]) - shift[0],
                           float(b_xy[1]) - shift[1], Z_ACTIVE, rx, ry, 0.0])
    info = {
        'throw_tilt_deg': math.degrees(math.hypot(rs.tilt_rx, rs.tilt_ry)),
        'event_vel_mps': float(rs.event_vel_mps),
        'displacement_mm': float(rs.displacement_mm),
    }
    try:
        _plan, report = planner.build_catch(
            _rest_state(rs.pretilt_pose_stow), catch_pose, float(flight_s),
            LIMITS, GEOM, settle_hold_s=SETTLE_S, receive_tilt=(rx, ry))
    except TrajectoryInfeasible as exc:
        return exc.code, info
    info['peak_leg_vel_mmps'] = float(report.peak_leg_vel_mmps)
    info['peak_leg_acc_mmps2'] = float(report.peak_leg_acc_mmps2)
    info['peak_leg_jerk_mmps3'] = float(report.peak_leg_jerk_mmps3)
    return 'OK', info


def frontier_table(a_xy=(0.0, 0.0)):
    """Rows of {flight, closed_form_mm, per-radius accepted-direction counts}."""
    rows = []
    for T in FLIGHTS_S:
        cells = {}
        for r in RADII_MM:
            ok, codes = 0, {}
            for _name, ux, uy in DIRECTIONS:
                verdict, _info = reach_verdict(
                    a_xy, (a_xy[0] + r * ux, a_xy[1] + r * uy), T)
                if verdict == 'OK':
                    ok += 1
                else:
                    codes[verdict] = codes.get(verdict, 0) + 1
            cells[r] = {'accepted': ok, 'n': len(DIRECTIONS), 'codes': codes}
        rows.append({'flight_s': T,
                     'closed_form_mm': closed_form_limit_mm(T),
                     'radii': cells})
    return rows


def degenerate_check(sites):
    """B == A at each site: the move plans, the catch plans, and the release
    state is BITWISE the Tier-8a one."""
    out = []
    for a in sites:
        seed = np.array([a[0], a[1], Z_ACTIVE, 0.0, 0.0, 0.0])
        try:
            move_plan, _rep = planner.build_move(
                _rest_state(seed), seed.copy(), 0.0, LIMITS, GEOM)
            move = {'ok': True, 'duration_s': float(move_plan.total_duration),
                    'segments': len(move_plan.segments)}
        except TrajectoryInfeasible as exc:
            move = {'ok': False, 'code': exc.code}
        row = {'site_mm': list(a), 'degenerate_move': move, 'flights': {}}
        for T in (0.55, 0.70, 0.80, 1.10):
            rs = toss_release.compute_release_state_tilted(
                (a[0], a[1], Z_ACTIVE), T, throw_site_xy_mm=a)
            ref = toss_release.compute_release_state((a[0], a[1], Z_ACTIVE), T)
            verdict, _info = reach_verdict(a, a, T)
            row['flights'][f'{T:.2f}'] = {
                'reach': verdict,
                'tilt_exactly_level': bool(rs.tilt_rx == 0.0 and rs.tilt_ry == 0.0),
                'bitwise_equal_to_8a': bool(
                    np.array_equal(rs.launch_vel_mms, ref.launch_vel_mms)
                    and np.array_equal(rs.release_pos_global_mm,
                                       ref.release_pos_global_mm)),
                'displacement_mm': float(rs.displacement_mm),
            }
        out.append(row)
    return out


def tilt_table():
    """Required from-vertical aim (deg) and event_vel (m/s) vs displacement."""
    rows = []
    for r in (0.0, 70.0, 100.0, 150.0, 200.0, 250.0, 300.0, 320.0):
        cells = {}
        for T in FLIGHTS_S:
            try:
                rs = toss_release.compute_release_state_tilted(
                    (r, 0.0, Z_ACTIVE), T, throw_site_xy_mm=(0.0, 0.0))
                cells[f'{T:.2f}'] = {
                    'aim_deg': math.degrees(math.hypot(rs.tilt_rx, rs.tilt_ry)),
                    'event_vel_mps': float(rs.event_vel_mps)}
            except toss_release.ThrowTiltInfeasible as exc:
                cells[f'{T:.2f}'] = {'clamp_required_deg': exc.required_deg}
        rows.append({'displacement_mm': r, 'flights': cells})
    return rows


def _print_frontier(rows, title):
    print('=' * 78)
    print(title)
    print('=' * 78)
    header = f"{'T':>5} {'cf_lim':>8} " + ' '.join(
        f'{int(r):>6}' for r in RADII_MM)
    print(header)
    for row in rows:
        cells = []
        for r in RADII_MM:
            c = row['radii'][r]
            mark = '' if not c['codes'] else '*'
            cells.append(f"{c['accepted']}/{c['n']}{mark}")
        print(f"{row['flight_s']:5.2f} {row['closed_form_mm']:8.1f} "
              + ' '.join(f'{c:>6}' for c in cells))
    print('  (* = at least one direction rejected; cf_lim = the closed-form '
          'quintic bound the FSM gates on)')


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    p.add_argument('--json', action='store_true',
                   help='also write the full result to temp/probes/')
    args = p.parse_args(argv)

    centre = frontier_table((0.0, 0.0))
    _print_frontier(centre, 'A. A→B reach frontier from A = (0, 0), 8 directions')

    print()
    offsets = [(150.0, 0.0), (-150.0, 0.0), (0.0, 150.0), (150.0, 150.0),
               (-150.0, -150.0), (150.0, -150.0)]
    print('=' * 78)
    print('B. Off-centre throw sites A at the shipped cap '
          f'({hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM:.0f} mm), directions that stay '
          'inside the ±150 box')
    print('=' * 78)
    cap = float(hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM)
    off_rows = []
    for a in offsets:
        for T in (0.70, 0.80, 0.95):
            cells = []
            for name, ux, uy in DIRECTIONS:
                b = (a[0] + cap * ux, a[1] + cap * uy)
                if abs(b[0]) > 150.0 or abs(b[1]) > 150.0:
                    cells.append(f'{name}:box')   # refused by TOSS_XY_LIMIT_MM
                    continue
                verdict, _info = reach_verdict(a, b, T)
                cells.append(f'{name}:{verdict}')
                off_rows.append({'a_mm': list(a), 'b_mm': list(b),
                                 'flight_s': T, 'verdict': verdict})
            print(f'  A={str(a):>16} T={T:.2f}  ' + '  '.join(cells))

    print()
    deg = degenerate_check([(0.0, 0.0), (150.0, 0.0), (-150.0, 150.0),
                            (150.0, -150.0), (75.0, 75.0)])
    print('=' * 78)
    print('C. DEGENERATE B == A (the "8b subsumes 8a" case)')
    print('=' * 78)
    for row in deg:
        flags = []
        for T, f in row['flights'].items():
            flags.append(f"T={T}:{f['reach']}"
                         f"{'/level' if f['tilt_exactly_level'] else '/TILTED'}"
                         f"{'/bitwise8a' if f['bitwise_equal_to_8a'] else '/DIFFERS'}")
        mv = row['degenerate_move']
        print(f"  A=B={str(tuple(row['site_mm'])):>16} "
              f"move={'OK %.3f s' % mv['duration_s'] if mv['ok'] else mv['code']}"
              f"  " + '  '.join(flags))

    print()
    tilts = tilt_table()
    print('=' * 78)
    print(f'D. Required aim vs displacement (MAX_TILT_DEG = '
          f'{tilt_geometry.MAX_TILT_DEG})')
    print('=' * 78)
    print(f"{'d_mm':>6} " + ' '.join(f'T={T:.2f}' for T in FLIGHTS_S))
    for row in tilts:
        cells = []
        for T in FLIGHTS_S:
            c = row['flights'][f'{T:.2f}']
            cells.append(f"CLAMP{c['clamp_required_deg']:5.1f}"
                         if 'clamp_required_deg' in c
                         else f"{c['aim_deg']:4.2f}d/{c['event_vel_mps']:4.2f}")
        print(f"{row['displacement_mm']:6.0f} "
              + ' '.join(f'{c:>11}' for c in cells))

    if args.json:
        out_dir = os.path.join(_REPO, 'temp', 'probes')
        os.makedirs(out_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        path = os.path.join(out_dir, f'displaced_reach_frontier_{stamp}.json')
        with open(path, 'w') as fh:
            json.dump({
                'session_limits': {
                    'leg_vel_mmps': LIMITS.leg_vel_mmps,
                    'leg_acc_mmps2': LIMITS.leg_acc_mmps2,
                    'leg_jerk_mmps3': LIMITS.leg_jerk_mmps3},
                'shipped_cap_mm': cap,
                'max_tilt_deg': float(tilt_geometry.MAX_TILT_DEG),
                'frontier_from_centre': [
                    {'flight_s': r['flight_s'],
                     'closed_form_mm': r['closed_form_mm'],
                     'radii': {str(k): v for k, v in r['radii'].items()}}
                    for r in centre],
                'off_centre_at_cap': off_rows,
                'degenerate': deg,
                'aim_table': tilts,
            }, fh, indent=2)
        print(f'\nwrote {path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
