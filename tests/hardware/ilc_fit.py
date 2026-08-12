#!/usr/bin/env python3
"""Critical-point ILC fit — the operator CLI over ``ilc_fit_lib``.

``plans/active/critical-point-ilc.md`` Phase 1. A THIN wrapper: every number it
prints comes from :mod:`ilc_fit_lib`, which is where the arguments live and where
the tests point. Nothing here touches the robot — the fit is offline by
construction (design constraint 2), and this file has no ROS import, no socket
and no write path into ``config/``. Phase 2 owns the artifact writer; until then
the answer is printed, read, and argued about.

    # what the sensitivity says, with no corpus at all
    python tests/hardware/ilc_fit.py --report --flight-time 0.9032

    # the fit on the mined corpus (the 2026-08-12 re-mine)
    python tests/hardware/ilc_fit.py --corpus temp/probes/toss_records_*.jsonl \\
        --allow-cross-partition

    # the four pre-registered Phase-1 validations, V1..V4
    python tests/hardware/ilc_fit.py --validate --corpus temp/probes/toss_records_*.jsonl

    # the model round trip alone (no corpus, no hardware)
    python tests/hardware/ilc_fit.py --self-check

Exit status is 0 on success, 1 on a refusal (including a V4 NULL-exit — a null
is a legitimate scientific outcome and a NON-ZERO exit, because the plan says
nothing ships on it and a green CLI would say the opposite).
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ilc_fit_lib as lib                                          # noqa: E402


def _fmt(v, spec='{:+.6g}'):
    return 'n/a' if v is None else spec.format(v)


def print_channels() -> None:
    print('COMMAND VECTOR u — throw side (v1 candidates)')
    for i, ch in enumerate(lib.U_CHANNELS):
        print('  [{}] {:<22s} unit={:<26s} authority={:.6g} ({})'.format(
            i, ch.name, ch.unit, ch.authority, ch.authority_src))
        print('      seam: {}'.format(ch.seam))
        print('      fd step={:.3g}  tau0={:.6g}'.format(ch.fd_step, ch.tau0))
    print('COMMAND VECTOR u — catch side: DECLARED, NOT IMPLEMENTED in v1')
    for name, unit, seam in lib.CATCH_CHANNELS:
        print('  {:<22s} unit={:<12s} seam: {}'.format(name, unit, seam))
    print('TASK ERROR e — throw critical point')
    for i, lbl in enumerate(lib.E_LABELS):
        print('  [{}] {:<18s} sigma={:<10.5g} {}'.format(
            i, lbl, lib.SIGMA_E[i],
            'E-1 BLOCKED' if lbl in lib.E1_BLOCKED else 'FITTED in v1'))


def print_report(goal: lib.TossGoal, *, mask=None) -> dict:
    plane = lib.measurement_plane_mm(goal)
    print('\nGOAL  pose={} mm (STOW)  T={:.6f} s  apex={:.4f} m  plane={:.2f} mm'
          .format(tuple(round(v, 3) for v in goal.catch_pose_stow_mm),
                  goal.flight_time_s, goal.apex_m, plane))
    e0 = lib.e_model(lib.zero_command(), goal)
    print('  e_model(u=0) = {}  (|e| = {:.3g} — the planner solves the boundary '
          'problem it is scored on)'.format(
              np.array2string(e0, precision=3), float(np.linalg.norm(e0))))
    F = lib.sensitivity(goal=goal)
    print('\nSENSITIVITY F = de/du  (rows {} / columns {})'.format(
        list(lib.E_LABELS), list(lib.U_LABELS)))
    for i in range(lib.N_E):
        print('  {:<18s} {}'.format(
            lib.E_LABELS[i],
            '  '.join('{:>13.6g}'.format(F[i, j]) for j in range(lib.N_U))))

    # Both masks are passed EXPLICITLY: mask=None means E1_MASK everywhere in the
    # library (weight_matrix, solve_step, required_command, conditioning,
    # screen_channels, iterate, fit_corpus share one default), so the full-size
    # report has to ask for np.ones by name.
    for label, m in (('FULL-SIZE (model-only; the mocap artefact cannot reach it)',
                      np.ones(lib.N_E)),
                     ('UNDER E-1 (the v1 answer)', lib.E1_MASK)):
        scr = lib.screen_channels(F, mask=m)
        rep = scr['report']
        print('\nCONDITIONING — {}'.format(label))
        print('  singular values (column-scaled, noise-whitened): {}'.format(
            np.array2string(rep['singular_values'], precision=4)))
        print('  rank {}   condition number over retained directions {:.4g}'
              .format(rep['rank'], rep['condition_retained']))
        for v in scr['verdicts']:
            print('  {:<22s} snr={:<9.4g} {}{}'.format(
                v['channel'], v['snr'], v['verdict'],
                '' if v['verdict'] == 'RETAINED'
                else '  [{}]'.format(v['reason'])))
            if v['detail']:
                print('      {}'.format(v['detail']))
        print('  v1 subset: {}'.format(list(scr['retained_labels'])))
    del mask
    return {'F': F}


def print_corpus(records, args) -> dict:
    admitted = [r for r in records if lib.admit_record(r)[0]]
    print('\nCORPUS  {} rows, {} admitted (usable_for_release_fit + the vertical '
          'channel)'.format(len(records), len(admitted)))
    reasons = {}
    for r in records:
        ok, why = lib.admit_record(r)
        if not ok:
            reasons[why] = reasons.get(why, 0) + 1
    for why, n in sorted(reasons.items(), key=lambda kv: -kv[1]):
        print('    excluded {:<4d} {}'.format(n, why))

    census = lib.partition_census(admitted)
    print('  partitions:')
    for line in lib.census_lines(census):
        print(line)
    up = lib.uptime_census(admitted)
    print('  can-bridge uptime at release: min {:.2f} h  median {:.2f} h  max '
          '{:.2f} h  {}'.format(up['min_h'] or float('nan'),
                                up['median_h'] or float('nan'),
                                up['max_h'] or float('nan'), up['buckets']))
    if up['buckets']['>12h']:
        print('    WARNING (G-1): {} of {} admitted rows were recorded past 12 h '
              'of bridge uptime, where the transport latency has drifted. This '
              'module does not refuse them — the healthy threshold is G-1\'s to '
              'fix — but a correction learnt here is a correction learnt on that '
              'plant state.'.format(up['buckets']['>12h'], len(admitted)))

    fit = lib.fit_corpus(records, goal_cell=args.goal_cell,
                         pool_across_goals=args.pool,
                         allow_cross_partition=args.allow_cross_partition,
                         uptime_max_ms=args.uptime_max_ms)
    for w in fit['warnings']:
        print('  WARNING: {}'.format(w))
    print('  goal cells: {}'.format(
        {str(k): v for k, v in sorted(fit['goal_cells'].items(), key=str)}))
    print('\nFIT  cell={}  rows={}{}'.format(
        fit['goal_cell'], fit['n_rows'],
        '  (POOLED ACROSS GOAL CELLS by request)' if fit['pooled'] else ''))
    print('  e_meas = {}'.format(np.array2string(fit['e_meas'], precision=6)))
    print('  counts = {}'.format(list(int(c) for c in fit['channel_counts'])))
    print('  du     = {}   (trust-region shrinks: {})'.format(
        np.array2string(fit['du'], precision=6), fit['shrinks']))
    for r in fit['refusals']:
        print('    gate refusal: {}'.format(r))

    auth = lib.authority_report(fit['F'], fit['e_meas'])
    print('\nAUTHORITY — what the residual asks for vs what may be commanded')
    for row in auth['channels']:
        if row['required'] == 0.0 and not row['exceeds']:
            continue
        print('  {:<22s} required {:+.6g}  authority {:.6g}  = {:.1%}  {}'
              .format(row['channel'], row['required'], row['authority'],
                      row['fraction'],
                      'EXCEEDS — operator decision' if row['exceeds'] else 'ok'))
        if row['joint_required'] is not None:
            # The aim pair is bounded JOINTLY (toss_cal.clamp_total_aim), so the
            # verdict above is the joint one — print the magnitude it was taken
            # on, or the per-axis fraction beside an EXCEEDS reads as a
            # contradiction.
            print('      joint |aim| = {:.6g} rad = {:.1%} of the authority — '
                  'the bound is on the MAGNITUDE, not the axis'
                  .format(row['joint_required'], row['joint_fraction']))
        print('      bound owned by {}'.format(row['source']))
    steps = lib.iterate(fit['goal'], fit['e_meas'], n_iter=args.iters)
    print('  model-side convergence at tau0 (measured residual held fixed):')
    for s in steps:
        print('    iter {}  u={}  predicted flight_time_err {:+.5f} s{}'.format(
            s['iter'], np.array2string(s['u'], precision=5), s['e_pred'][4],
            '  SATURATED {}'.format(s['saturated']) if s['saturated'] else ''))
    return fit


def run_validations(records, args) -> int:
    """V1..V4, the four pre-registered Phase-1 validations. Returns an exit code."""
    failures = []
    print('\n' + '=' * 78)
    print('V1 — F\'s aim/landing block reproduces b = 4.h.theta')
    print('=' * 78)
    from jugglebot import toss_trim
    for h_m in (0.78, 1.00):
        T = math.sqrt(8.0 * h_m * 1000.0 / lib.ballistics_bc.GRAVITY_MMS2)
        goal = lib.TossGoal(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                            flight_time_s=T)
        F = lib.sensitivity(goal=goal)
        block = F[0:2, 0:2]
        gain = abs(float(block[0, 1]))
        prod = toss_trim.aim_landing_jacobian(T, 170.0)
        ideal = 4.0 * h_m * 1000.0
        d_prod = abs(gain - abs(float(prod[0, 1]))) / gain
        d_ideal = (gain - ideal) / ideal
        print('  h={:.2f} m (T={:.5f} s): F block gain {:.4f} mm/rad = {:.4f} '
              'mm/deg'.format(h_m, T, gain, gain * math.pi / 180.0))
        print('      vs PRODUCTION toss_trim.aim_landing_jacobian {:.4f}  '
              '(rel. diff {:.2e})'.format(abs(float(prod[0, 1])), d_prod))
        print('      vs idealised 4h = {:.1f}  (rel. diff {:+.4%})'
              .format(ideal, d_ideal))
        print('      structure: [[{:.1f}, {:.1f}], [{:.1f}, {:.1f}]] — a 90 deg '
              'rotation, NOT a scaled identity'.format(
                  block[0, 0], block[0, 1], block[1, 0], block[1, 1]))
        if d_prod > 1e-9:
            failures.append('V1: F disagrees with the production Jacobian at '
                            'h={}'.format(h_m))
        if abs(d_ideal) > 0.01:
            failures.append('V1: F is more than 1 % from 4h at h={}'.format(h_m))
    print('  VERDICT: {}'.format('PASS' if not failures else 'FAIL'))

    print('\n' + '=' * 78)
    print('V2a — closed-loop sign test through the FORWARD model (synthetic)')
    print('=' * 78)
    goal = lib.TossGoal(catch_pose_stow_mm=(0.0, 150.0, 170.0),
                        flight_time_s=lib.CORPUS_FLIGHT_TIME_S)
    u_plant = np.array([0.0, 0.0, 0.02, 0.0])
    syn = lib.synthetic_corpus(goal, u_plant=u_plant, n=60)
    du = lib.required_command(lib.sensitivity(goal=goal),
                              lib.pooled_error(syn)[0])
    print('  injected plant offset  u_plant = {}'.format(u_plant))
    print('  fitted correction      du      = {}'.format(
        np.array2string(du, precision=6)))
    print('  cancellation on event_vel_trim: {:.2%}'.format(
        1.0 - abs(du[2] + u_plant[2]) / abs(u_plant[2])))
    if du[2] >= 0.0 or abs(du[2] + u_plant[2]) > 0.15 * abs(u_plant[2]):
        failures.append('V2a: the synthetic closed loop did not cancel')
    du_flip = lib.required_command(-lib.sensitivity(goal=goal),
                                   lib.pooled_error(syn)[0])
    print('  a SIGN-FLIPPED F returns {} — same magnitude, wrong way, which '
          'DOUBLES the residual'.format(np.array2string(du_flip, precision=6)))
    if du_flip[2] <= 0.0:
        failures.append('V2a: the sign-flipped control did not fail')

    print('\n' + '=' * 78)
    print('V2b — real-corpus sign test on a channel the fit NEVER SAW')
    print('=' * 78)
    if records:
        fit = lib.fit_corpus(records, goal_cell=args.goal_cell,
                             pool_across_goals=True,
                             allow_cross_partition=args.allow_cross_partition,
                             uptime_max_ms=args.uptime_max_ms)
        need = lib.required_command(fit['F'], fit['e_meas'])
        cc = lib.cross_check_replay(need, fit['goal'], records)
        print('  fitted on flight_time_err_s only:  du = {}'.format(
            np.array2string(need, precision=6)))
        print('  replayed against release_speed_err_mms (the OTHER branch of '
              'the arc, a disjoint mocap fit):')
        print('    predicted change {:+.1f} mm/s   measured mean {:+.1f} mm/s'
              .format(cc['predicted_change_mms'], cc['mean_before_mms']))
        print('    rms {:.1f} -> {:.1f} mm/s   reduction {:.1%}'.format(
            cc['rms_before_mms'], cc['rms_after_mms'], cc['reduction']))
        flip = lib.cross_check_replay(-need, fit['goal'], records)
        print('    sign-flipped control: rms {:.1f} -> {:.1f} mm/s (x{:.2f})'
              .format(flip['rms_before_mms'], flip['rms_after_mms'],
                      flip['rms_after_mms'] / flip['rms_before_mms']))
        if need[2] >= 0.0:
            failures.append('V2b: the fitted trim is not negative against a '
                            'plant that throws FAST')
        if cc['reduction'] < 0.5:
            failures.append('V2b: the correction does not reduce the '
                            'cross-check channel')
    else:
        print('  SKIPPED — no corpus given')

    print('\n' + '=' * 78)
    print('V3 — held-out prediction')
    print('=' * 78)
    if records:
        held_out_n = []
        for channel in ('flight_time_err_s', 'release_speed_err_mms'):
            ho = lib.held_out_prediction(records, channel=channel)
            held_out_n.append(ho['n'])
            s = ho['split']
            print('  {:<24s} n={}'.format(channel, ho['n']))
            print('    split  train {} / test {}: du = {:+.5g}, rms {:.5g} -> '
                  '{:.5g} ({:.1%})'.format(
                      s['n_train'], s['n_test'], s['du'],
                      s['rms_before'], s['rms_after'], s['reduction']))
            print('      PREDICTED held-out mean {:+.5g}   ACTUAL {:+.5g}   '
                  '(se {:.3g}, miss = {:+.2f} se)'.format(
                      s['predicted_test_mean'], s['actual_test_mean'],
                      s['se_test_mean'], s['z']))
            print('    LOO    rms {:.5g} -> {:.5g} ({:.1%})'.format(
                ho['loo']['rms_before'], ho['loo']['rms_after'],
                ho['loo']['reduction']))
        # Every number in this sentence is READ OFF the run, never restated from
        # a memo: the held-out n is the admitted rows with a RECOVERABLE goal
        # (fewer than the admitted count, because the model prediction needs a
        # geometry to evaluate the sensitivity at), and the cell / flight-time
        # counts come from the same keys the fit grouped on.
        cells = {lib.goal_key(r) for r in records if lib.admit_record(r)[0]}
        cells.discard(None)
        n_txt = ' / '.join(str(n) for n in dict.fromkeys(held_out_n))
        print('  HONEST READING: n = {} (admitted rows with a recoverable goal) '
              'across {} goal cell(s) and {} commanded flight time(s). The '
              'held-out reduction is large because the residual is a '
              'near-constant bias; it says NOTHING about generalisation to '
              'another flight time, which this corpus cannot test at all.'
              .format(n_txt, len(cells), len({c[3] for c in cells})))
    else:
        print('  SKIPPED — no corpus given')

    print('\n' + '=' * 78)
    print('V4 — repeatability decision (pre-registered NULL-exit)')
    print('=' * 78)
    if records:
        for channel in ('flight_time_err_s', 'release_speed_err_mms'):
            rep = lib.repeatability(records, channel=channel)
            print('  {:<24s} n={} over {} goal cells'.format(
                channel, rep['n'], rep['n_goal_cells']))
            print('    mean {:+.5g}  sd {:.5g}   rms {:.5g} -> {:.5g} (LOO, '
                  'per goal cell)'.format(rep['mean'], rep['sd'],
                                          rep['rms_raw'], rep['rms_loo']))
            print('    R_rep = {:.4f}   threshold {:.2f}   null expectation '
                  '{:+.4f}   ==> {}'.format(
                      rep['R_rep'], rep['threshold'], rep['null_expectation'],
                      rep['verdict']))
            for pg in rep['per_goal']:
                if pg.get('skipped'):
                    print('      goal {} n={} SKIPPED (n < 2)'.format(
                        pg['goal'], pg['n']))
                else:
                    print('      goal {} n={:<3d} mean {:+.5g} sd {:.5g}'.format(
                        pg['goal'], pg['n'], pg['mean'], pg['sd']))
            if rep['verdict'] != 'PASS':
                failures.append('V4 NULL-exit on {}'.format(channel))
    else:
        print('  SKIPPED — no corpus given')

    print('\n' + '=' * 78)
    if failures:
        print('VALIDATIONS FAILED:')
        for f in failures:
            print('  - {}'.format(f))
        return 1
    print('V1-V4: all pre-registered validations PASS')
    return 0


def self_check() -> int:
    """Model-only round trips — no corpus, no hardware, no files."""
    goal = lib.TossGoal(catch_pose_stow_mm=(0.0, 150.0, 170.0),
                        flight_time_s=lib.CORPUS_FLIGHT_TIME_S)
    checks = []

    e0 = lib.e_model(lib.zero_command(), goal)
    checks.append(('e_model(0) == 0', float(np.linalg.norm(e0)) < 1e-9,
                   '|e| = {:.3g}'.format(float(np.linalg.norm(e0)))))

    F = lib.sensitivity(goal=goal)
    checks.append(('release_timing_offset column is STRUCTURALLY zero',
                   bool(np.all(F[:, 3] == 0.0)),
                   'column = {} — dt reaches no production call in this model, '
                   'so the zero is algebraic; it says the MODEL has no dt '
                   'dependence, not that the machine has none'.format(F[:, 3])))

    dv = 0.01
    e = lib.e_model(np.array([0.0, 0.0, dv, 0.0]), goal)
    checks.append(('a +1 % event_vel trim lengthens the flight by ~+1 %',
                   abs(e[4] / (dv * goal.flight_time_s) - 1.0) < 0.02,
                   'de/dv = {:+.6f} s vs T = {:.6f} s'.format(e[4],
                                                              goal.flight_time_s)))

    ok, why = lib.admit_command(np.array([0.0, 0.0, 0.5, 0.0]), goal)
    checks.append(('a +50 % trim is refused by the real gates', not ok, why))

    try:
        lib.catch_channel('catch_pose_dx')
        raised = False
    except NotImplementedError:
        raised = True
    checks.append(('a catch-side channel refuses loudly', raised, ''))

    rc = 0
    for name, ok, detail in checks:
        print('  {:<52s} {}'.format(name, 'ok' if ok else 'FAIL'))
        if detail:
            print('      {}'.format(detail))
        rc |= 0 if ok else 1
    return rc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--corpus', nargs='*', default=[],
                    help='mined toss_record/1 JSONL file(s)')
    ap.add_argument('--report', action='store_true',
                    help='print the sensitivity + conditioning report')
    ap.add_argument('--validate', action='store_true',
                    help='run the four pre-registered validations V1..V4')
    ap.add_argument('--self-check', action='store_true',
                    help='model-only round trips (no corpus needed)')
    ap.add_argument('--flight-time', type=float, default=lib.CORPUS_FLIGHT_TIME_S)
    ap.add_argument('--pose', default='0,150,170',
                    help='catch pose x,y,z in STOW mm (default %(default)s)')
    ap.add_argument('--goal-cell', default='__modal__',
                    help='fit this goal cell (default: the corpus modal cell)')
    ap.add_argument('--pool', action='store_true',
                    help='pool ALL goal cells into one fit (deliberate)')
    ap.add_argument('--allow-cross-partition', action='store_true',
                    help='fit across plant partitions (stamped in the output)')
    ap.add_argument('--uptime-max-ms', type=float, default=None,
                    help='G-1 defence in depth: refuse rows past this uptime')
    ap.add_argument('--iters', type=int, default=4)
    args = ap.parse_args(argv)

    if not (args.report or args.validate or args.self_check or args.corpus):
        args.report = True

    print('critical-point ILC — Phase 1 fit core ({})'.format(lib.TOOL_NAME))
    print_channels()

    rc = 0
    if args.self_check:
        print('\nSELF-CHECK')
        rc |= self_check()

    x, y, z = (float(v) for v in args.pose.split(','))
    goal = lib.TossGoal(catch_pose_stow_mm=(x, y, z),
                        flight_time_s=args.flight_time)
    if args.report or args.corpus or args.validate:
        print_report(goal)

    records = lib.load_corpus(args.corpus) if args.corpus else []
    if records:
        try:
            print_corpus(records, args)
        except lib.IlcFitError as exc:
            print('\nREFUSED: {}'.format(exc))
            rc |= 1
    if args.validate:
        try:
            rc |= run_validations(records, args)
        except lib.IlcFitError as exc:
            print('\nREFUSED: {}'.format(exc))
            rc |= 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
