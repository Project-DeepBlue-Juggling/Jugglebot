#!/usr/bin/env python3
"""Critical-point ILC fit — the operator CLI over ``ilc_fit_lib``.

``plans/active/critical-point-ilc.md`` Phases 1 and 2. A THIN wrapper: every
number it prints comes from :mod:`ilc_fit_lib`, which is where the arguments live
and where the tests point. Nothing here touches the robot — the fit is offline by
construction (design constraint 2), and this file has no ROS import and no
socket.

Phase 2 added the one write path: ``--write-artifact``, which emits the
production ``config/toss_ilc.yaml`` schema through
:mod:`jugglebot.motion.toss_ilc` (never a second copy of it) and **refuses on any
refusal, saturation or unprovable provenance**.

    # what the sensitivity says, with no corpus at all
    python tests/hardware/ilc_fit.py --report --flight-time 0.9032

    # the fit on the mined corpus (the 2026-08-12 re-mine)
    python tests/hardware/ilc_fit.py --corpus temp/probes/toss_records_*.jsonl \\
        --allow-cross-partition

    # the four pre-registered Phase-1 validations, V1..V4
    python tests/hardware/ilc_fit.py --validate --corpus temp/probes/toss_records_*.jsonl

    # the model round trip alone (no corpus, no hardware)
    python tests/hardware/ilc_fit.py --self-check

    # PHASE 2: emit the production artifact (refuses unless the fit is clean)
    python tests/hardware/ilc_fit.py --corpus temp/probes/toss_records_*.jsonl \\
        --write-artifact config/toss_ilc.yaml

Exit status is 0 on success, 1 on a refusal (including a V4 NULL-exit — a null
is a legitimate scientific outcome and a NON-ZERO exit, because the plan says
nothing ships on it and a green CLI would say the opposite).
"""

from __future__ import annotations

import argparse
import datetime
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import ilc_fit_lib as lib                                          # noqa: E402

# The PRODUCTION schema, imported rather than restated. A writer carrying its own
# idea of the artifact layout is a second schema, and two schemas drift silently:
# the file still loads, it just means something else. Same argument
# ``toss_record`` makes for living in the installed package.
from jugglebot.motion import toss_ilc                              # noqa: E402


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
        if lib.DEFAULT_MASK[i]:
            role = 'FITTED'
        elif lbl in lib.MONITOR_CHANNELS:
            role = 'MONITOR ONLY (decision 6 — never drives the aim)'
        else:
            role = 'MASKED OUT'
        print('  [{}] {:<18s} sigma={:<10.5g} {}'.format(
            i, lbl, lib.SIGMA_E[i], role))
    print('  default mask {} — land_err is MONITOR-ONLY since 2026-08-21 (C3, '
          'owner decision 6); the 08-13..08-21 answer is mask=FULL_MASK {} and '
          'the pre-E-1 one is mask=E1_MASK {}'.format(
              list(lib.DEFAULT_MASK), list(lib.FULL_MASK), list(lib.E1_MASK)))
    print('  lateral channels additionally require a mined coverage_asym_s '
          'within {:.2f} s (ilc_fit_lib.lateral_admissible)'
          .format(lib.COVERAGE_ASYM_MAX_S))


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

    # All three masks are passed EXPLICITLY. mask=None means DEFAULT_MASK
    # everywhere in the library (weight_matrix, solve_step, required_command,
    # conditioning, screen_channels, iterate, fit_corpus share one default), and
    # since owner decision 6 that is [0, 0, 1, 1, 1] — land_err demoted to a
    # monitor. Naming all three here keeps each historical answer one line away
    # instead of one archaeology session away, and lets an operator SEE that the
    # demotion did not screen the aim channels out (the question the decision had
    # to answer): the aim SNR falls 2.9207 -> 1.9330 and stays far above 1.0.
    for label, m in (('THE DEFAULT since 2026-08-21 — land_err MONITOR-ONLY (C3)',
                      lib.DEFAULT_MASK),
                     ('FULL-SIZE (HISTORICAL — the 2026-08-13..08-21 answer)',
                      lib.FULL_MASK),
                     ('UNDER E-1 (HISTORICAL — the pre-2026-08-13 v1 answer)',
                      lib.E1_MASK)):
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
        print('  retained subset: {}'.format(list(scr['retained_labels'])))
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
    # The E-1 admission, reported as a census on the ADMITTED rows rather than
    # left to be inferred from a channel count: a corpus glob happily matches a
    # pre-2026-08-13 mine, whose lateral channels are the artefact.
    lat_ok, lat_why = 0, {}
    for r in admitted:
        ok, why = lib.lateral_admissible(r)
        if ok:
            lat_ok += 1
        else:
            lat_why[why.split(' —')[0]] = lat_why.get(why.split(' —')[0], 0) + 1
    print('  lateral channels admissible on {}/{} admitted rows (E-1 '
          'coverage_asym_s gate)'.format(lat_ok, len(admitted)))
    for why, n in sorted(lat_why.items(), key=lambda kv: -kv[1]):
        print('    lateral-refused {:<4d} {}'.format(n, why))

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
    print('  mask   = {}  (effective, after intersecting with the counts: {})'
          .format(list(fit['mask']), list(fit['mask_effective'])))
    print('  du     = {}   (trust-region shrinks: {})'.format(
        np.array2string(fit['du'], precision=6), fit['shrinks']))
    for r in fit['refusals']:
        print('    gate refusal: {}'.format(r))

    ev = fit['evidence']
    print('\nEVIDENCE GATE (toss_trim SE gate + CUSUM, per channel){}'
          .format('' if fit['evidence_applied']
                  else '  — REPORTED, NOT APPLIED (require_evidence=False)'))
    for c in ev['channels']:
        print('  {:<18s} n={:<3d} mean={:+12.6g} se={:<12.6g} |mean|/se={:6.2f} '
              ' {}{}'.format(c['channel'], c['n'], c['mean'], c['se'],
                             c['significance'], c['verdict'],
                             '' if c['requested'] else '  [monitor]'))
        if c['detail']:
            print('      {}'.format(c['detail']))
    if ev['frozen']:
        print('  FROZEN: a FITTED channel shifted mid-cell. The accumulated '
              'command is HELD at u_prev on that channel — never zeroed.')
    if ev.get('frozen_monitor'):
        print('  NOTE: a MONITOR channel shifted mid-cell. Nothing was being '
              'learnt from it, so no command is frozen — but a plane residual '
              'that moves inside one cell is a plant statement worth reading.')

    dis = fit['disagreement']
    print('\nC3 CHANNEL DISAGREEMENT (monitor — land_err minus arrival_dir x the '
          'model gain)')
    if dis['n']:
        print('  n={}  mean ({:+.2f}, {:+.2f}) mm   sd ({:.2f}, {:.2f}) mm'
              .format(dis['n'], dis['mean_mm'][0], dis['mean_mm'][1],
                      dis['sd_mm'][0], dis['sd_mm'][1]))
        print('  The aim update is driven by arrival_dir ALONE (decision 6). A y '
              'residual holding the measured b(z) profile shape CONFIRMS the '
              'centroid-bias model; a catch-rate plateau at converged aim '
              're-opens C3.')
    else:
        print('  no row in this cell can answer both channels')

    auth = lib.authority_report(fit['F'], fit['e_meas'],
                                mask=fit['mask_effective'])
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


class WriteRefused(RuntimeError):
    """The artifact writer's refusal. Distinct from :class:`lib.IlcFitError` so a
    caller can tell "the fit could not be computed" from "the fit was computed
    and must not be shipped"."""


def _accumulate(fit, u_prev):
    """``u_next = u_prev + du`` — the ACCUMULATED command this cell should carry.

    **The artifact stores the accumulated ``u``, not the last step**, because the
    loop is iterative by construction: ``fit_corpus`` returns a step bounded by
    the trust region ``tau`` (0.040 on ``event_vel_trim``), and the measured
    corpus asks for −0.1076, so the correction is reached over Phase 3's ``k <= 3``
    iterations rather than in one write. Each hardware sitting mines a fresh
    corpus, re-fits the residual *that already carries the applied correction*,
    and adds a bounded step to it — the plan's "corrections accumulate into a
    per-goal keyed artifact".

    ``ilc_fit_lib.propose_step`` validated ``u_prev_assumed_zero + du`` against
    the real gates. The ACCUMULATED command is a different vector, so it is
    re-validated by ``_cell_refusals`` through the same ``admit_command`` —
    design constraint 3 says every emitted command passes the exact gates, and
    "the step was admissible" is not "the sum is".

    The converse also holds and is gated separately: "the sum is admissible" is
    not "the seed was". ``_prev_u`` runs the same gates on ``u_prev`` itself
    before it reaches this function, and the prior artifact's provenance is
    checked against this write's declarations before either — see
    :func:`write_artifact`.
    """
    du = np.asarray(fit['du'], dtype=float)
    return np.asarray(u_prev, dtype=float).reshape(lib.N_U) + du


def _cell_refusals(fit, u_next, goal) -> list:
    """Every reason this cell's fit must NOT be written, in one pass.

    ``goal`` is **THIS CELL's** geometry, never the fit's. For a per-cell fit the
    two are the same object; for a POOLED fit they are not, and the difference is
    load-bearing rather than pedantic: one ``du`` is written to every cell the
    corpus visited, and those cells differ in pose and in flight time. Every gate
    :func:`lib.admit_command` runs is goal-dependent — the sequencer's
    flight-time band and workspace box, ``compute_release_state_tilted``'s tilt
    feasibility, and the bridge's ``event_vel`` acceptance band, which is the one
    that actually bites: a ``-0.10`` trim that clears the 0.3 m/s floor at
    T = 0.90 s is a different absolute speed at T = 1.10 s. Validating a pooled
    cell against the MODAL cell's geometry is therefore a gate that can pass for
    a command the machine would reject at the cell it is being written to.

    The bar is "the fit is clean", not "the fit produced a number", and these are
    the ways ``ilc_fit_lib`` says otherwise:

    * ``shrinks > 0`` / a non-empty ``refusals`` list — the trust region had to be
      halved because the REAL production gates refused the solved step. The step
      that survived is a fraction of the one the residual asked for, and shipping
      it as though it were the answer hides that the machine refused the answer.
      (Saturating AT ``tau`` is not this: that is the loop working as designed.)
    * ``any_exceeds`` — the correction the residual asks for is past an
      authority. That is an operator decision (it is exactly the one Gate 1 had
      to take for ``event_vel_trim``), not something a writer may quietly
      saturate into a file.
    * the accumulated command fails ``admit_command`` — the exact production
      gates, run on the vector that will actually be commanded.
    * a non-zero ``release_timing_offset`` column — REFUSED from v1; a non-zero
      value there would mean the model gained a dt dependence nobody screened.
    * an all-zero correction — an artifact cell that commands nothing is not the
      same object as no cell: it reports ``ilc_applied`` and changes nothing,
      which is the state hardest to tell from a broken apply seam.
    """
    out = []
    if fit['shrinks']:
        out.append('the trust region was halved {}x by the REAL gates — the '
                   'step that survived is not the step the residual asked for'
                   .format(fit['shrinks']))
    for refusal in fit['refusals']:
        out.append('gate refusal: {}'.format(refusal))
    # F at THIS CELL's geometry too. On a per-cell fit this reproduces `fit['F']`
    # exactly (same function, same goal); on a pooled one it is the sensitivity
    # the residual would actually be inverted through at the cell being written,
    # so the authority verdict is about the command this cell will carry.
    F_cell = lib.sensitivity(goal=goal)
    auth = lib.authority_report(F_cell, fit['e_meas'],
                                mask=fit['mask_effective'])
    for row in auth['channels']:
        if row['exceeds']:
            out.append('{} needs {:+.6g}, past its {:.6g} authority ({}) — '
                       'widening a bound is an operator decision, not a write'
                       .format(row['channel'], row['required'],
                               row['authority'], row['source']))
    u_next = np.asarray(u_next, dtype=float)
    ok, why = lib.admit_command(u_next, goal)
    if not ok:
        out.append('the ACCUMULATED command {} is refused by the real gates: {}'
                   .format(np.array2string(u_next, precision=6), why))
    if float(u_next[3]) != 0.0:
        out.append('release_timing_offset came back {:+.6g}, but it is REFUSED '
                   'from v1 and its column of F is structurally zero — a '
                   'non-zero value means the model changed underneath the screen'
                   .format(float(u_next[3])))
    if not np.any(u_next[:3]):
        out.append('the correction is all zeros — an artifact cell that commands '
                   'nothing reports ilc_applied and changes nothing, which is '
                   'indistinguishable from a broken apply seam')
    return out


def _corpus_field(rows, name):
    """The single value of ``name`` across ``rows``, or ``None`` when they
    disagree; ``(value, distinct)`` so the caller can report the disagreement."""
    seen = []
    for rec in rows:
        value = rec.get(name)
        if value not in seen:
            seen.append(value)
    return (seen[0] if len(seen) == 1 else None), seen


#: Every record field the fit, the guards or the partition rule reads — the
#: PROJECTION that :func:`emit_fixture` commits. Deliberately a hand-maintained
#: list rather than "every key on the row": a fixture is evidence, and evidence
#: whose provenance nobody can audit because it is 166 columns wide is not much
#: better than no fixture. Pinned complete by
#: ``test_the_committed_fixture_reproduces_the_headline_numbers`` — if the fit
#: grows a field this list lacks, the fixture's answers stop matching and the
#: suite says so.
FIXTURE_FIELDS = (
    # identity + partition (ilc_fit_lib.PARTITION_KEYS, load_corpus)
    'toss_uid', 'schema', 'tilt_map_version', 'bridge_fw_version',
    'platform_fw_version', 'cmd_flight_time_s', 'land_plane_mm',
    'toss_cal_loaded', 'toss_cal_applied', 'uptime_ms_at_release',
    # the miner's own gates
    'usable_for_release_fit', 'usable_for_lateral_fit', 'usable_for_aim_fit',
    'usable_for_speed_fit', 'usable_for_timing_fit', 'excluded_reason',
    # the E channels and what recovers the goal
    'flight_time_err_s', 'land_err_mm', 'arrival_dir_err_rad',
    'land_xy_global_mm', 'coverage_asym_s', 'release_speed_err_mms',
    # C7
    'toss_tier', 'throw_site_xy_mm',
    # toss_trim's G1-G11
    'label', 't_departure_raw_ros', 'ball_track_confirmed',
    'throw_stroke_seen', 'fit_sparse', 'fit_rms_mm', 'n_fit',
    'retry_of', 'reload_settle', 'total_aim_rad', 'map_aim_rad',
    'trim_aim_rad', 'ilc_aim_rad', 'apex_height_m', 'achieved_flight_s_mocap',
    'flight_time_s', 'goal_catch_xyz_stow_mm', 'dip_below_x3_rev',
    'stroke_peak_rev', 'trunc', 'gravity_correction_loaded',
    'tilt_map_applied', 'announce_throw_time_ros',
    'ball_held_stamp_wall_anchored', 'rimshot', 'sensor_poll_dt_ms_median',
)

_FIXTURE_HEADER = '''\
"""Measured ILC corpus fixture — DO NOT hand-edit.

The {n} rows of the 2026-08-12 corpus that ``ilc_fit_lib.admit_record`` admits,
projected onto :data:`ilc_fit.FIXTURE_FIELDS` — every field the fit, the
``toss_trim`` guards or the partition rule reads, and nothing else.

**Why this file exists (contradiction C8 of the ILC-primary fold-in).** Every
corpus-backed number this arc reports — V2b's 86.8 % out-of-channel
cancellation, V3's 84.9 / 86.4 % leave-one-out, V4's R_rep 0.9858 / 0.9790, the
C3 disagreement, D2's sigma — was measured against ``temp/probes/*.jsonl``.
``temp/`` is gitignored and per-worktree, so on a clean checkout those tests
SKIP and the headline numbers have no committed provenance at all. A skipped
test is not a passing test, and a number nobody can re-derive is not a
measurement.

Regenerate with:
    python tests/hardware/ilc_fit.py \\
        --corpus temp/probes/toss_records_*.jsonl --emit-fixture \\
        tests/hardware/ilc_corpus_fixture.py

Source bags: {bags}
Mined:       {mines}
Partitions:  {parts}

Consumed by tests/motion/test_ilc_fit.py. This is a PROJECTION, not the corpus:
it cannot re-run the miner, and it carries no mocap samples. What it can do is
answer every question the fit asks, which is what the V2b/V3/V4 assertions
actually need.
"""

from __future__ import annotations

#: The admitted rows, newest-mine-per-bag, in ``load_corpus`` order.
ROWS = (
'''


def emit_fixture(records, args, path: str) -> int:
    """Write the committed corpus fixture (C8). Returns a shell exit code."""
    admitted = [r for r in records
                if lib.admit_record(r, uptime_max_ms=args.uptime_max_ms)[0]]
    if not admitted:
        print('\nFIXTURE REFUSED: no admitted rows to cut a fixture from')
        return 1
    bags = sorted({str(r.get('_source_file') or '?') for r in admitted})
    # The mine stamp is the source FILE's, by `load_corpus`'s own convention —
    # `_mine_stamp` is what resolved the newest-mine-per-bag choice, so quoting
    # anything else here would name a different file from the one that won.
    mines = sorted({lib._mine_stamp(b) for b in bags if b != '?'})
    census = lib.partition_census(admitted)
    lines = [_FIXTURE_HEADER.format(
        n=len(admitted),
        bags=', '.join(os.path.basename(b) for b in bags),
        mines=', '.join(mines),
        parts='; '.join(lib.census_lines(census)) or '(none)')]
    for rec in admitted:
        lines.append('    {')
        for name in FIXTURE_FIELDS:
            if name in rec:
                lines.append('        {!r}: {!r},'.format(name, rec[name]))
        lines.append('    },')
    lines.append(')\n')
    with open(path, 'w') as fh:
        fh.write('\n'.join(lines))
    print('\nFIXTURE: wrote {} rows x {} fields to {}'
          .format(len(admitted), len(FIXTURE_FIELDS), path))
    return 0


def _artifact_provenance(rows, args) -> dict:
    """Derive ``requires.tilt_map_version`` / ``requires.toss_cal_version`` from
    the corpus, or REFUSE.

    **Unknown is never assumed.** A mined-only corpus (every bag predating
    ``/toss/record``) carries ``toss_cal_version = None``, and stamping ``''``
    there would declare "fitted with NO aim map applied" on evidence that says
    only "nobody wrote it down". The artifact's whole dormancy gate rests on
    these two strings, so a guess here is a gate that fails open forever.

    The two ``--declare-*`` flags are the operator's escape hatch, and using
    either is STAMPED in ``captured`` rather than laundered into the requires
    block: the artifact then records that its provenance was asserted rather than
    measured, which is the difference a later reader needs.
    """
    out = {}
    declared = {'tilt_map_version': args.declare_tilt_map,
                'toss_cal_version': args.declare_toss_cal}
    flag = {'tilt_map_version': '--declare-tilt-map',
            'toss_cal_version': '--declare-toss-cal'}
    stamps = {}

    for name in ('tilt_map_version', 'toss_cal_version'):
        if declared[name] is not None:
            value = '' if declared[name].upper() == 'NONE' else declared[name]
            out[name] = value
            stamps[name + '_source'] = 'operator-declared'
            continue
        if name == 'toss_cal_version':
            applied, seen_applied = _corpus_field(rows, 'toss_cal_applied')
            if applied is False:
                out[name] = ''
                stamps[name + '_source'] = 'corpus (toss_cal_applied=false)'
                continue
            if applied is None:
                raise WriteRefused(
                    'the corpus does not agree on toss_cal_applied (saw {}) — '
                    'an artifact fitted across an aim map coming and going '
                    'cannot declare which one it sits on top of. Split the '
                    'corpus, or assert it with --declare-toss-cal <version|NONE>'
                    .format(seen_applied))
        value, seen = _corpus_field(rows, name)
        if value is None or value == '':
            raise WriteRefused(
                'the corpus does not supply {} (saw {}) — this string is the '
                'artifact\'s dormancy gate, and guessing it makes the gate fail '
                'open forever. Assert it with {} <version|NONE> if you know it '
                'out of band'.format(name, seen, flag[name]))
        out[name] = str(value)
        stamps[name + '_source'] = 'corpus'
    out['_stamps'] = stamps
    return out


def write_artifact(records, args) -> int:
    """``--write-artifact``: emit ``config/toss_ilc.yaml`` from a clean fit.

    All-or-nothing across every cell: one refused cell refuses the whole write.
    A partial artifact would ship corrections for the goals that fitted cleanly
    and silently omit the ones that did not — which reads, at the machine, as
    "those goals need no correction".

    The document is round-tripped through the PRODUCTION loader
    (``toss_ilc.parse_toss_ilc``) before a byte reaches the disk, so the file
    that is written is by construction a file this build can load. Writing first
    and validating later would leave an unloadable artifact in ``config/`` on
    every failure.
    """
    if not records:
        raise WriteRefused('--write-artifact needs a --corpus to fit')

    admitted = [r for r in records
                if lib.admit_record(r, uptime_max_ms=args.uptime_max_ms)[0]]
    if not admitted:
        raise WriteRefused('no admitted rows in this corpus')

    groups = lib.group_by_goal(admitted)
    cells = sorted(k for k in groups if k is not None)
    if not cells:
        raise WriteRefused('no admitted row carries a recoverable goal cell')

    # The provenance THIS write will stamp, derived BEFORE anything is fitted,
    # because ``--from-artifact`` has to be checked against it (below) and a
    # seeded accumulation whose premises are unprovable must not cost a fit
    # first.
    provenance = _artifact_provenance(admitted, args)
    stamps = provenance.pop('_stamps')

    previous = None
    if args.from_artifact:
        # The ITERATION seam. Without it every write would restart from zero and
        # the trust region would cap the correction at tau forever — the loop
        # would look convergent and never converge.
        previous = toss_ilc.load_toss_ilc(args.from_artifact)
        print('  iterating on {} [{}], {} cell(s)'.format(
            args.from_artifact, previous.version, previous.n_cells))
        # THE PRIOR IS NOT TRUSTED BECAUSE IT PARSED. `parse_toss_ilc` proves the
        # document is well-formed and inside the per-cell authorities; it does
        # NOT prove the artifact was fitted on the plant this write is fitting.
        # Accumulation is `u_prev + du`, so an unchecked prior imports its whole
        # command vector into a file that will then be stamped with THIS corpus's
        # provenance — a correction learnt under aim map A shipped under a
        # document declaring aim map B, which is exactly the double-count the
        # dormancy gate exists to prevent, laundered past it by the writer.
        #
        # The check is the PRODUCTION gate (`TossIlc.provenance_mismatch`), run
        # against the strings this document will carry, never a second copy of
        # the comparison: the writer and the machine must agree about what
        # "matches" means, or an artifact refuses at the robot after shipping.
        mismatch = previous.provenance_mismatch(
            tilt_map_version=provenance['tilt_map_version'],
            toss_cal_version=provenance['toss_cal_version'])
        if mismatch:
            raise WriteRefused(
                '--from-artifact {} cannot be accumulated onto: {}. The prior '
                'correction was fitted on a different plant state, and '
                'u_prev + du would ship it under THIS corpus\'s provenance '
                '(tilt_map={!r}, toss_cal={!r}) — the double-count the dormancy '
                'gate exists to prevent, moved to where the gate cannot see it. '
                'Re-fit from zero, or restore the layers it was fitted under.'
                .format(args.from_artifact, mismatch,
                        provenance['tilt_map_version'],
                        provenance['toss_cal_version']))

    def _prev_u(cell, goal):
        """This cell's seeded ``u_prev``, VALIDATED at this cell's own goal.

        ``_cell_refusals`` re-validates the SUM, which is necessary and not
        sufficient: ``admit_command`` is not monotone in ``u``, so an
        inadmissible ``u_prev`` can perfectly well add to an admissible
        ``u_prev + du`` (the D7 clamp bounds a MAGNITUDE — a prior aim past the
        authority plus a ``du`` pointing back inside it lands in bounds, and the
        machine would have refused the prior every toss it flew). Design
        constraint 3 says every emitted command passes the exact gates; the
        seed is one of the commands the emitted vector is made of, so it is
        gated where it enters rather than inferred from the sum.
        """
        if previous is None:
            return lib.zero_command()
        got = toss_ilc.lookup(previous, cell[0], cell[1], cell[2], cell[3])
        u = lib.zero_command()
        u[0] = got.correction.aim_rx
        u[1] = got.correction.aim_ry
        u[2] = got.correction.event_vel_trim
        ok, why = lib.admit_command(u, goal)
        if not ok:
            raise WriteRefused(
                'the SEED from --from-artifact {} for cell {} is {} and the '
                'real gates refuse it: {}. An inadmissible prior is not made '
                'admissible by adding a step to it — re-fit this cell from '
                'zero.'.format(args.from_artifact, list(cell),
                               np.array2string(u, precision=6), why))
        if not got.hit:
            print('    cell {}: no prior cell — seeding from ZERO'
                  .format(list(cell)))
        return u

    def _cell_goal(fit, cell):
        """THIS cell's own geometry out of the fit that produced ``du``."""
        goal = fit['cell_goals'].get(cell)
        if goal is None:
            raise WriteRefused(
                'cell {} has no goal in the fit that produced its correction — '
                'the corpus rows that named it were dropped before the fit ran, '
                'so nothing measured this cell'.format(list(cell)))
        return goal

    print('\nWRITE-ARTIFACT  {} goal cell(s): {}'.format(
        len(cells), [list(c) for c in cells]))

    written = []
    refusals = []
    if args.pool:
        # ONE fit, deliberately pooled, written to every cell the corpus visited.
        # Defensible on the 2026-08-12 corpus (the three cells' vertical means are
        # not resolved above their own standard errors) and it is stamped in the
        # document, because "one number for three poses" is a claim a later reader
        # must be able to see rather than infer.
        #
        # The FIT is pooled; the ADMISSION is not. Each cell is gated at its own
        # geometry — see `_cell_refusals` for why the modal cell's is the wrong
        # one to gate a different pose and flight time against.
        fit = lib.fit_corpus(records, goal_cell=args.goal_cell,
                             pool_across_goals=True,
                             allow_cross_partition=args.allow_cross_partition,
                             uptime_max_ms=args.uptime_max_ms)
        for cell in cells:
            goal = _cell_goal(fit, cell)
            u_next = _accumulate(fit, _prev_u(cell, goal))
            why = _cell_refusals(fit, u_next, goal)
            if why:
                refusals.extend('POOLED -> cell {}: {}'.format(list(cell), w)
                                for w in why)
            else:
                written.append((cell, u_next[:3], fit['evidence']))
    else:
        for cell in cells:
            fit = lib.fit_corpus(records, goal_cell=cell,
                                 allow_cross_partition=args.allow_cross_partition,
                                 uptime_max_ms=args.uptime_max_ms)
            goal = _cell_goal(fit, cell)
            u_next = _accumulate(fit, _prev_u(cell, goal))
            why = _cell_refusals(fit, u_next, goal)
            if why:
                refusals.extend('cell {}: {}'.format(list(cell), w) for w in why)
            else:
                written.append((cell, u_next[:3], fit['evidence']))

    if refusals:
        raise WriteRefused(
            'the fit is not clean, so NOTHING is written:\n  {}'
            .format('\n  '.join(refusals)))

    doc = toss_ilc.build_document(
        # `or 0.0` normalises -0.0 to 0.0: they compare equal, but the apply
        # seam's byte-identical branch reads `!= 0.0` and a `-0.0` in a committed
        # YAML is a distraction an operator would have to reason about.
        [(key, [float(v) or 0.0 for v in u]) for key, u, _ev in written],
        tilt_map_version=provenance['tilt_map_version'],
        toss_cal_version=provenance['toss_cal_version'],
        date=datetime.date.today().isoformat(),
        tool=lib.TOOL_NAME,
        captured=dict(stamps, n_admitted=len(admitted),
                      n_goal_cells=len(cells),
                      pooled=bool(args.pool),
                      corpus=[os.path.basename(p) for p in args.corpus]),
        fit={'sigma_e': [float(v) for v in lib.SIGMA_E],
             'rho_damping': float(lib.RHO_DAMPING),
             'channels': list(toss_ilc.ILC_CHANNELS)})

    # Round-trip through the PRODUCTION loader BEFORE writing (see the docstring).
    parsed = toss_ilc.parse_toss_ilc(doc)

    path = os.path.abspath(args.write_artifact)
    parent = os.path.dirname(path)
    if parent and not os.path.isdir(parent):
        raise WriteRefused('{} does not exist'.format(parent))
    with open(path, 'w') as handle:
        yaml.safe_dump(doc, handle, sort_keys=False, default_flow_style=False)

    print('  version   {}'.format(parsed.version))
    print('  requires  tilt_map={!r} toss_cal={!r} estimator={!r} model={!r}'
          .format(parsed.requires_tilt_map_version,
                  parsed.requires_toss_cal_version,
                  parsed.requires_estimator_version,
                  parsed.requires_model_config_identity))
    for key, u, ev in written:
        print('  cell {}  aim_rx={:+.6g} aim_ry={:+.6g} event_vel_trim={:+.6g}'
              .format(list(key), float(u[0]), float(u[1]), float(u[2])))
        # A cell whose evidence gate withheld a channel carried u_prev THROUGH
        # on that channel (freeze-never-zero) rather than learning anything, and
        # with no --from-artifact that is a written ZERO. Silence here would let
        # "I wrote an artifact" read as "I learned something", which is the one
        # misreading an all-or-nothing writer cannot afford.
        for chan, verdict in ev['gated']:
            print('      HELD  {:<16s} {} — no step written on this channel; '
                  'the accumulated value is carried through, never zeroed'
                  .format(chan, verdict))
    print('  WROTE {}'.format(path))
    print('  It ships DORMANT: jugglebot_operational.toss_ilc_enabled is the '
          'gate, it defaults false, and arming it is a reviewed config commit + '
          '`python config/generate_config.py` + '
          '`colcon build --packages-select jugglebot`.')
    return 0


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
        need = lib.required_command(fit['F'], fit['e_meas'],
                                    mask=fit['mask_effective'])
        cc = lib.cross_check_replay(need, fit['goal'], records)
        print('  fitted on {}:  du = {}'.format(
            [lib.E_LABELS[i] for i in range(lib.N_E)
             if fit['mask_effective'][i]],
            np.array2string(need, precision=6)))
        print('  NOTE: release_speed_err_mms is NOT among them — it is the '
              'held-back channel this replay scores against.')
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


def build_parser() -> argparse.ArgumentParser:
    """The CLI surface, exposed so tests parse the SAME flags an operator types.

    Split out of :func:`main` when ``--write-artifact`` landed: a test that
    hand-builds an args namespace drifts from the parser silently, and the one
    entry point that writes a file into ``config/`` is the last place to accept
    that."""
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
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
    ap.add_argument('--write-artifact', metavar='PATH', default=None,
                    help='Phase 2: emit the production config/toss_ilc.yaml '
                         'from this fit. Refuses on any gate refusal, trust-'
                         'region shrink, authority exceedance or unprovable '
                         'provenance.')
    ap.add_argument('--declare-tilt-map', metavar='VERSION|NONE', default=None,
                    help='assert requires.tilt_map_version when the corpus '
                         'cannot supply it (stamped as operator-declared)')
    ap.add_argument('--declare-toss-cal', metavar='VERSION|NONE', default=None,
                    help='assert requires.toss_cal_version when the corpus '
                         'cannot supply it (stamped as operator-declared)')
    ap.add_argument('--emit-fixture', metavar='PATH', default=None,
                    help='project the admitted rows onto FIXTURE_FIELDS and '
                         'write them as a committed Python fixture (C8) — the '
                         'only route by which a clean checkout can re-derive '
                         'this arc\'s corpus-backed numbers')
    ap.add_argument('--from-artifact', metavar='PATH', default=None,
                    help='ITERATE: seed each cell from this existing artifact, '
                         'so the write is u_prev + du rather than du. Without '
                         'it every write restarts from zero and the trust '
                         'region caps the correction at tau forever. The prior '
                         'is REFUSED unless its provenance matches this write\'s '
                         'declarations and every seeded command passes the real '
                         'gates at its own cell.')
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

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
    if args.emit_fixture:
        try:
            rc |= emit_fixture(records, args, args.emit_fixture)
        except (lib.IlcFitError, OSError) as exc:
            print('\nFIXTURE REFUSED: {}'.format(exc))
            rc |= 1
    if args.write_artifact:
        try:
            rc |= write_artifact(records, args)
        except (WriteRefused, lib.IlcFitError, toss_ilc.TossIlcError,
                OSError) as exc:
            # A non-zero exit, deliberately: a refusal that exits green would
            # tell a script the artifact is on disk when it is not.
            print('\nWRITE REFUSED: {}'.format(exc))
            rc |= 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
