#!/usr/bin/env python3
"""Fit ``config/toss_calibration.yaml`` from a per-toss corpus — contract
**C-TOSS-CAL-1**, build phase 2c of ``plans/active/toss-selftuning.md``.

    # what would be written, node by node — writes NOTHING, makes no ROS call
    python tests/hardware/toss_cal_fit.py --corpus temp/probes/toss_records_*.jsonl --dry-run

    # fit + validate + write, then print the reload command
    python tests/hardware/toss_cal_fit.py --corpus temp/probes/*.jsonl

    # A/B score two arms of a ladder on the CONTINUOUS observables
    python tests/hardware/toss_cal_fit.py --corpus c.jsonl --group A=1-20 --group B=21-40

**This tool is entirely desk-side.** It reads JSONL, writes YAML, and prints. It
sends no goal, actuates nothing, and does not import ``rclpy`` unless ``--reload``
is passed — the flag exists so the operator can close the § 3.8 loop (call
``toss/reload_calibration`` and **read the version back**, which is the hard
guarantee the node loaded the file this tool wrote) without a second terminal.

Every number, gate and refusal lives in :mod:`toss_fit_lib`, which is pure and
importable; this file is argument parsing, ordering and printing. That split is
what lets ``tests/motion/test_toss_cal_fit.py`` drive the REAL fit — including
the closed-loop sign test that injects a known aim bias into a replayed corpus
and asserts the fit recovers it — instead of a restated copy of it.

ORDERING, AND WHY THE REFUSALS COME FIRST
-----------------------------------------
Preflight refusals are hoisted ahead of any work (the tilt-cal audit found a
write-target check running *after* a four-minute sweep). In order: resolve the
write target; load and partition the corpus; refuse a cross-partition fit; fit;
run the two guards that refuse a write (flat field, ball-actually-flew);
self-validate through the production loader; only then write.

EXIT CODES
----------
``0`` fit written (or ``--dry-run`` / ``--no-apply`` completed);
``1`` a refusal — partition crossing, a guard, a validation failure, or an
unreadable corpus. Non-zero always means *nothing was written*.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import subprocess
import sys
from datetime import datetime
from typing import Any, Dict, List, Optional, Sequence, Tuple

_HERE = os.path.dirname(os.path.abspath(__file__))              # tests/hardware
_REPO = os.path.dirname(os.path.dirname(_HERE))                 # repo root
for _p in (_HERE, _REPO, os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np                                              # noqa: E402
import yaml                                                     # noqa: E402

import toss_fit_lib as fit                                      # noqa: E402
from jugglebot.motion import toss_cal                           # noqa: E402

DEFAULT_BOX_MM = 150.0
DEFAULT_NODES = 3
DEFAULT_Z_MM = 170.0


# ── helpers ──────────────────────────────────────────────────────────────────


def build_axis(box_mm: float, nodes: int, name: str) -> List[float]:
    """``nodes`` points spanning ``±box_mm`` inclusively, origin included.

    An even ``nodes`` has no centre point, and the map is home-referenced, so it
    is refused here rather than discovered by :func:`toss_fit_lib.home_index`
    after the corpus has been loaded.
    """
    if nodes < 3 or nodes % 2 == 0:
        raise fit.TossFitError(
            '--{}-nodes must be odd and >= 3 (got {}) — the map is '
            'HOME-REFERENCED, so the grid must contain (0, 0)'
            .format(name, nodes))
    if box_mm <= 0.0:
        raise fit.TossFitError('--box-mm must be positive (got {})'.format(box_mm))
    step = 2.0 * float(box_mm) / (nodes - 1)
    return [round(-float(box_mm) + i * step, 6) for i in range(nodes)]


def parse_node_list(text: str, name: str) -> List[float]:
    values = [float(v) for v in text.replace(',', ' ').split()]
    if len(values) < 2:
        raise fit.TossFitError('--{} needs at least 2 nodes'.format(name))
    if any(b <= a for a, b in zip(values, values[1:])):
        raise fit.TossFitError(
            '--{} must be strictly increasing, got {}'.format(name, values))
    return values


def parse_group(spec: str) -> Tuple[str, int, int]:
    """``NAME=lo-hi`` (1-based, inclusive) — mirrors ``ball_arrival_offset.py``."""
    try:
        name, span = spec.split('=', 1)
        lo, hi = span.split('-', 1)
        return name, int(lo), int(hi)
    except ValueError:
        raise fit.TossFitError(
            'bad --group {!r}; want NAME=lo-hi, e.g. A=7-18'.format(spec))


def git_sha() -> str:
    try:
        out = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'],
                                      cwd=_REPO, stderr=subprocess.DEVNULL)
        return out.decode().strip()
    except Exception:
        return 'unknown'


def expand(patterns: Sequence[str]) -> List[str]:
    paths: List[str] = []
    for pattern in patterns:
        hits = sorted(glob.glob(pattern))
        if not hits:
            if os.path.exists(pattern):
                hits = [pattern]
            else:
                raise fit.TossFitError(
                    'no corpus file matches {!r}'.format(pattern))
        paths.extend(hits)
    seen, out = set(), []
    for path in paths:
        if path not in seen:
            seen.add(path)
            out.append(path)
    return out


def _deg(rad: float) -> float:
    return math.degrees(float(rad))


# ── printing ─────────────────────────────────────────────────────────────────


def print_nodes(x_mm, y_mm, fits, anchor_rad, previous_doc, gain_mm_per_rad):
    """Node-by-node table: the shipped (home-referenced) aim, its n/sd, and the
    delta against the previous map. This is what ``--dry-run`` prints."""
    print('\nNODES  (shipped = home-referenced;  mm at the capture height uses '
          'gain {:.1f} mm/rad)'.format(gain_mm_per_rad))
    print('  {:>4} {:>4} {:>8} {:>8} {:>5} {:>10} {:>10} {:>9} {:>9} {}'.format(
        'x', 'y', 'rx[deg]', 'ry[deg]', 'n', 'sd_rx[deg]', 'sd_ry[deg]',
        'Lx[mm]', 'Ly[mm]', 'state'))
    prev_rx = ((previous_doc or {}).get('aim_rad') or {}).get('rx')
    prev_ry = ((previous_doc or {}).get('aim_rad') or {}).get('ry')
    for iy in range(len(y_mm)):
        for ix in range(len(x_mm)):
            f = fits[(ix, iy)]
            rx = f.rx - anchor_rad[0]
            ry = f.ry - anchor_rad[1]
            # The landing shift this node commands: J·aim, with J the production
            # Jacobian (a 90 deg rotation), so Lx comes from ry and Ly from -rx.
            Lx = gain_mm_per_rad * ry
            Ly = -gain_mm_per_rad * rx
            state = f.source if f.stale else 'fit'
            if f.stale:
                state = 'STALE({})'.format(f.source)
            delta = ''
            if prev_rx and prev_ry:
                try:
                    d = math.hypot(rx - float(prev_rx[iy][ix]),
                                   ry - float(prev_ry[iy][ix]))
                    delta = '  Δ{:.4f}°'.format(_deg(d))
                except (IndexError, TypeError, ValueError):
                    delta = '  Δ?'
            print('  {:>4.0f} {:>4.0f} {:>8.4f} {:>8.4f} {:>5d} {:>10} {:>10} '
                  '{:>9.1f} {:>9.1f} {}{}'.format(
                      x_mm[ix], y_mm[iy], _deg(rx), _deg(ry), f.n,
                      '-' if not math.isfinite(f.sd_rx) else '{:.4f}'.format(_deg(f.sd_rx)),
                      '-' if not math.isfinite(f.sd_ry) else '{:.4f}'.format(_deg(f.sd_ry)),
                      Lx, Ly, state, delta))


def print_summary(rows, excluded, sigma, r_eff, speed, timing, anchors):
    print('\nCORPUS  {} records in partition'.format(len(rows)))
    if excluded:
        print('  excluded from the aim fit:')
        for reason, n in sorted(excluded.items(), key=lambda kv: -kv[1]):
            print('    {:<32} {}'.format(reason, n))
    print('  sigma_L (per-axis, admitted) : {}'.format(
        '-' if sigma is None else '{:.1f} mm'.format(sigma)))
    print('  R_eff from the catch rate    : {}   (geometric radius {:.1f} mm)'
          .format('-' if r_eff is None else '{:.1f} mm'.format(r_eff),
                  fit.hw.GEOM_HAND_RADIUS_MM))
    if sigma is not None:
        # The pre-registered discriminators of plan section 6, evaluated here so
        # the operator reads them off the artefact instead of remembering them.
        if sigma > 30.0:
            print('  DISCRIMINATOR: sigma_L > 30 mm — STOP. No calibration '
                  'converges inside a sitting at that scatter; the fault is '
                  'upstream and this map would be fitting a broken plant.')
        elif sigma >= 18.0:
            print('  DISCRIMINATOR: sigma_L >= 18 mm — the throw SCATTERS. '
                  'Pursue variance reduction (catch-robustness Phase 0, settle '
                  'discipline, Layer 1.5); this map is worth ~5-8 catch-rate '
                  'points and no more.')
        elif sigma <= 12.0:
            print('  DISCRIMINATOR: sigma_L <= 12 mm — if catches still fail, '
                  'the SEAT is the fault, not the aim; the catch-knob ladder is '
                  'the right work.')
    print('\nANCHOR SERIES ({} home visits)'.format(len(anchors)))
    for visit in anchors:
        print('  t={:.1f}  n={:<3d} aim ({:+.4f}, {:+.4f})°'.format(
            visit.t_s, visit.n, _deg(visit.rx), _deg(visit.ry)))
    print('\nSPEED   n={} k_v={} se={} applies={}'.format(
        speed['n'],
        '-' if speed['k_v'] is None else '{:.4f}'.format(speed['k_v']),
        '-' if speed['se'] is None else '{:.4f}'.format(speed['se']),
        speed['applies']))
    print('TIMING  n={} tau={} se={} sd={} applies={} uptime_trend={}'.format(
        timing['n'],
        '-' if timing['tau_ms'] is None else '{:.1f} ms'.format(timing['tau_ms']),
        '-' if timing['se_ms'] is None else '{:.1f} ms'.format(timing['se_ms']),
        '-' if timing['sd_ms'] is None else '{:.1f} ms'.format(timing['sd_ms']),
        timing['applies'],
        '-' if timing['uptime_trend_ms_per_h'] is None
        else '{:+.1f} ms/h'.format(timing['uptime_trend_ms_per_h'])))
    if timing['excluded']:
        print('  timing exclusions: {}'.format(
            ', '.join('{}={}'.format(k, v)
                      for k, v in sorted(timing['excluded'].items()))))


# ── the run ──────────────────────────────────────────────────────────────────


def run(args) -> int:                                            # noqa: C901
    paths = expand(args.corpus)
    print('CORPUS FILES')
    for path in paths:
        print('  {}'.format(path))

    target = args.out or fit.write_target_path()
    print('\nWRITE TARGET  {}'.format(target))
    if not args.dry_run and not args.no_apply:
        parent = os.path.dirname(os.path.abspath(target))
        if not os.path.isdir(parent) or not os.access(parent, os.W_OK):
            raise fit.TossFitError(
                'write target directory is not writable: {}'.format(parent))

    records = fit.load_corpus(paths)
    if not records:
        raise fit.TossFitError('the corpus is empty')

    if args.group:
        groups = []
        for spec in args.group:
            name, lo, hi = parse_group(spec)
            groups.append((name, records[lo - 1:hi]))
        print('\nA/B SCORING (continuous observables — never the label, F4)')
        for row in fit.score_groups(records, groups):
            print('  {:<12} n={:<4d} admitted={:<4d} mean_err={} sigma={} '
                  'R_eff={} catch={}'.format(
                      row['name'], row['n_records'], row['n_admitted'],
                      '-' if row['mean_err_mm'] is None
                      else '({:+.1f}, {:+.1f}) mm'.format(*row['mean_err_mm']),
                      '-' if row['sigma_mm'] is None
                      else '{:.1f} mm'.format(row['sigma_mm']),
                      '-' if row['r_eff_mm'] is None
                      else '{:.1f} mm'.format(row['r_eff_mm']),
                      '-' if row['catch_rate'] is None
                      else '{:.0%} of {}'.format(row['catch_rate'],
                                                 row['n_labelled'])))

    rows, census, key, warnings = fit.select_partition(
        records, allow_cross=args.allow_cross_partition,
        key=None if args.partition_index is None
        else sorted(fit.partition_census(records).items(),
                    key=lambda kv: (-kv[1], str(kv[0])))[args.partition_index][0])
    print('\nPARTITION CENSUS ({} partitions)'.format(len(census)))
    for line in fit.census_lines(census):
        print(line)
    for warning in warnings:
        print('  WARN: {}'.format(warning))

    if args.grid_x:
        x_mm = parse_node_list(args.grid_x, 'grid-x')
    else:
        x_mm = build_axis(args.box_mm, args.nodes, 'x')
    if args.grid_y:
        y_mm = parse_node_list(args.grid_y, 'grid-y')
    else:
        y_mm = build_axis(args.box_mm, args.nodes, 'y')
    fit.home_index(x_mm, y_mm)          # refuses a grid without the origin

    previous = None
    previous_doc = None
    prev_path = args.previous or toss_cal.resolve_toss_cal_path()
    if prev_path and os.path.exists(prev_path):
        try:
            with open(prev_path, 'r') as handle:
                previous_doc = yaml.safe_load(handle)
            previous = toss_cal.parse_toss_cal(previous_doc)
            print('\nPREVIOUS MAP  {}  version {}'.format(prev_path,
                                                          previous.version))
        except (OSError, ValueError, toss_cal.TossCalError) as exc:
            print('\nPREVIOUS MAP  {} is unreadable ({}) — thin nodes have '
                  'nothing to carry'.format(prev_path, exc))
            previous, previous_doc = None, None

    fits, admitted, excluded = fit.fit_nodes(
        rows, x_mm, y_mm, n_min=args.n_min, previous=previous)

    # The ball-actually-flew guard runs HERE, before the anchor, and the order is
    # the point: on a corpus where the mocap could not see the ball (the
    # reference sitting: 0/31 usable, plan section 10) the anchor estimate also
    # fails, but its message says "no admitted toss at home" while this one says
    # "nothing flew anywhere, and here is where to look". The operator should be
    # handed the root cause, not the first symptom to raise.
    flew_ok, flew_msg = fit.ball_flew_verdict(fits, previous)
    if not flew_ok:
        print('\nEXCLUSIONS')
        for reason, n in sorted(excluded.items(), key=lambda kv: -kv[1]):
            print('  {:<32} {}'.format(reason, n))
        raise fit.TossFitError(flew_msg)

    anchors = fit.anchor_visits(rows, x_mm, y_mm)
    anchor = fit.anchor_estimate(anchors)

    z_mm = args.z_mm
    flights = [fit.achieved_flight_s(r) for r in rows]
    flights = [f for f in flights if f]
    T_ref = float(np.median(flights)) if flights else 0.7977
    J = fit.aim_landing_jacobian(T_ref, z_mm)
    gain = float(np.linalg.norm(J[:, 1]))

    sigma = fit.sigma_land_mm(rows)
    r_eff = fit.r_eff_mm(rows)
    speed = fit.speed_fit(rows)
    timing = fit.timing_fit(rows)

    print_summary(rows, excluded, sigma, r_eff, speed, timing, anchors)
    print_nodes(x_mm, y_mm, fits, anchor[0], previous_doc, gain)
    print('\nANCHOR (absolute home residual — phase 2e warm-start PRIOR, not a '
          'correction)\n  aim ({:+.4f}, {:+.4f})°  n={}  se ({}, {})°'.format(
              _deg(anchor[0][0]), _deg(anchor[0][1]), anchor[2],
              '-' if not math.isfinite(anchor[1][0]) else '{:.4f}'.format(_deg(anchor[1][0])),
              '-' if not math.isfinite(anchor[1][1]) else '{:.4f}'.format(_deg(anchor[1][1]))))

    # ── the second guard that refuses a write (plan section 3.8) ─────────────
    print('\nGUARD ball-actually-flew: PASS — {}'.format(flew_msg))
    is_flat, flat_msg = fit.flat_field_verdict(fits, anchor[0])
    print('GUARD flat-field: {}'.format('FLAT' if is_flat else 'PASS — ' + flat_msg))
    if is_flat and not args.allow_flat_field:
        raise fit.TossFitError(flat_msg)

    captured = {
        'date': datetime.now().strftime('%Y-%m-%d'),
        'git_sha': git_sha(),
        'tool': fit.TOOL_NAME,
        'args': ' '.join(sys.argv[1:]),
        'uptime_ms_first': _minmax(rows, 'uptime_ms_at_release', min),
        'uptime_ms_last': _minmax(rows, 'uptime_ms_at_release', max),
        'base_condition': args.base_condition,
        'partition_census': [
            {'n': n, **{name: (None if v is None else str(v))
                        for name, v in zip(fit.PARTITION_KEYS, k)}}
            for k, n in sorted(census.items(), key=lambda kv: -kv[1])],
        'partition_fitted': (None if key is None else
                             {name: (None if v is None else str(v))
                              for name, v in zip(fit.PARTITION_KEYS, key)}),
        'cross_partition': bool(args.allow_cross_partition and len(census) > 1),
        'flat_field_override': bool(is_flat and args.allow_flat_field),
        'source_files': [{'path': os.path.relpath(p, _REPO),
                          'sha256': fit.sha256_of(p)} for p in paths],
    }
    requires = {
        'tilt_map_version': _one_of(rows, 'tilt_map_version', args.tilt_map_version),
        'level_offset_rad': _pair_of(rows, 'level_offset_rad'),
        'estimator_version': toss_cal.ESTIMATOR_VERSION,
        'estimator': fit.default_estimator_block(),
    }
    jacobian = {
        'S': [[round(float(J[r][c] / gain), 9) for c in range(2)]
              for r in range(2)],
        'gain_mm_per_rad': round(gain, 4),
        'source': ('production-model (toss_release.aim_target_offset_mm, '
                   'finite-differenced) — SC-0 measures it on hardware'),
        'reference_flight_time_s': round(T_ref, 6),
    }
    stats_extra = {
        'sigma_L_mm': None if sigma is None else round(sigma, 3),
        'R_eff_mm': None if r_eff is None else round(r_eff, 3),
        'n_admitted': sum(len(v) for v in admitted.values()),
        'n_records': len(rows),
        'excluded': dict(sorted(excluded.items())),
        'timing': {k: v for k, v in timing.items() if k != 'excluded'},
    }
    speed_block = {'k_v': None if speed['k_v'] is None else round(speed['k_v'], 6),
                   'se': None if speed['se'] is None else round(speed['se'], 6),
                   'n': speed['n'], 'applies': speed['applies']}

    doc = fit.build_map_document(x_mm, y_mm, z_mm, fits, anchor,
                                 captured=captured, requires=requires,
                                 jacobian=jacobian, speed=speed_block,
                                 stats_extra=stats_extra)
    parsed = fit.validate_map_document(doc)
    print('\nVALIDATED through the production loader — version {}'.format(
        parsed.version))

    if previous_doc:
        diff = fit.diff_documents(previous_doc, doc)
        if diff['shape_match']:
            print('MAP-vs-MAP  max node delta {:.4f}°'.format(
                _deg(diff['max_delta_rad'])))
        else:
            print('MAP-vs-MAP  previous map is on a different grid — no diff')

    if args.json:
        with open(args.json, 'w') as handle:
            json.dump({'version': parsed.version, 'document': _jsonable(doc)},
                      handle, indent=2, sort_keys=False)
        print('wrote {}'.format(args.json))

    if args.dry_run:
        print('\n--dry-run: NOTHING WRITTEN. Version this fit would ship: {}'
              .format(parsed.version))
        return 0
    if args.no_apply:
        print('\n--no-apply: captured and validated, NOTHING WRITTEN. Version: '
              '{}'.format(parsed.version))
        return 0

    with open(target, 'w') as handle:
        handle.write(fit.dump_map_yaml(doc))
    print('\nWROTE {}  version {}'.format(target, parsed.version))

    if args.reload:
        return _reload_and_readback(parsed.version)
    print('Reload it on the running node and READ THE VERSION BACK:\n'
          '  ros2 service call /toss/reload_calibration std_srvs/srv/Trigger\n'
          '  ros2 topic echo /toss/calibration_status --once\n'
          '  # expect toss_cal_version == {}'.format(parsed.version))
    return 0


def _jsonable(value):
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def _minmax(rows, field, fn):
    vals = [r.get(field) for r in rows if r.get(field) is not None]
    return fn(vals) if vals else None


def _one_of(rows, field, override):
    """The single value of ``field`` across the partition, or the override.

    A partition is by construction single-valued in the partition keys, so a
    disagreement here means the caller crossed partitions deliberately; the
    fitted map then records the override or ``''`` rather than picking a winner.
    """
    if override is not None:
        return override
    values = {r.get(field) for r in rows if r.get(field) is not None}
    if len(values) == 1:
        return str(next(iter(values)))
    return ''


def _pair_of(rows, field):
    for row in rows:
        value = row.get(field)
        if (isinstance(value, (list, tuple)) and len(value) == 2
                and all(isinstance(v, (int, float)) for v in value)):
            return [float(value[0]), float(value[1])]
    return None


def _reload_and_readback(expected_version: str) -> int:
    """Call ``toss/reload_calibration`` and read ``toss_cal_version`` back.

    ``rclpy`` is imported HERE, not at module scope, so importing this file (the
    tests do) never needs ROS on the path — and so the desk-side default path
    cannot accidentally require a running graph.
    """
    try:
        import rclpy                                            # noqa: F401
        from rclpy.node import Node
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
    except ImportError as exc:
        print('--reload needs a sourced ROS 2 environment: {}'.format(exc))
        return 1
    rclpy.init()
    try:
        node = Node('toss_cal_fit_reload')
        client = node.create_client(Trigger, '/toss/reload_calibration')
        if not client.wait_for_service(timeout_sec=5.0):
            print('REFUSED: /toss/reload_calibration is not available')
            return 1
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        response = future.result()
        print('reload: success={} message={}'.format(
            getattr(response, 'success', None),
            getattr(response, 'message', None)))
        seen: Dict[str, Any] = {}

        def _on_status(msg):
            try:
                seen.update(json.loads(msg.data))
            except ValueError:
                pass
        node.create_subscription(String, '/toss/calibration_status', _on_status, 1)
        deadline = 5.0
        while deadline > 0.0 and 'toss_cal_version' not in seen:
            rclpy.spin_once(node, timeout_sec=0.2)
            deadline -= 0.2
        got = seen.get('toss_cal_version')
        if got != expected_version:
            print('READBACK MISMATCH: node reports {!r}, this tool wrote {!r} — '
                  'the node is NOT running the map that was just written.'
                  .format(got, expected_version))
            return 1
        print('READBACK OK: node reports {} (applied={})'.format(
            got, seen.get('toss_cal_applied')))
        return 0
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    corpus = parser.add_argument_group('corpus')
    corpus.add_argument('--corpus', action='append', required=True,
                        help='toss_record JSONL file or glob (repeatable)')
    corpus.add_argument('--allow-cross-partition', action='store_true',
                        help='fit across partitions; the census is stamped in')
    corpus.add_argument('--partition-index', type=int, default=None,
                        help='fit partition N of the census (0 = largest)')
    corpus.add_argument('--group', action='append', default=[],
                        help='A/B scoring span, NAME=lo-hi (1-based, inclusive)')

    grid = parser.add_argument_group('grid')
    grid.add_argument('--box-mm', type=float, default=DEFAULT_BOX_MM)
    grid.add_argument('--nodes', type=int, default=DEFAULT_NODES,
                      help='odd, >= 3 (the grid must contain the origin)')
    grid.add_argument('--grid-x', default=None, help='explicit x nodes, mm')
    grid.add_argument('--grid-y', default=None, help='explicit y nodes, mm')
    grid.add_argument('--z-mm', type=float, default=DEFAULT_Z_MM)

    fitting = parser.add_argument_group('fit')
    fitting.add_argument('--n-min', type=int, default=fit.N_MIN,
                         help='admitted tosses to RE-FIT a node (PROVISIONAL)')
    fitting.add_argument('--previous', default=None,
                         help='previous map YAML (default: the resolved one)')
    fitting.add_argument('--tilt-map-version', default=None,
                         help='override requires.tilt_map_version')
    fitting.add_argument('--base-condition', default='',
                         help='free text stamped into captured.base_condition')

    workflow = parser.add_argument_group('workflow')
    workflow.add_argument('--dry-run', action='store_true',
                          help='print the node-by-node diff, write nothing')
    workflow.add_argument('--no-apply', action='store_true',
                          help='fit + validate, write nothing')
    workflow.add_argument('--allow-flat-field', action='store_true',
                          help='ship a map smaller than the tilt map\'s own '
                               'accuracy floor (stamped into captured)')
    workflow.add_argument('--out', default=None, help='write target override')
    workflow.add_argument('--json', default=None, help='also emit JSON here')
    workflow.add_argument('--reload', action='store_true',
                          help='call toss/reload_calibration and read the '
                               'version back (needs a sourced ROS 2)')
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return run(args)
    except fit.TossFitError as exc:
        print('\nREFUSED: {}'.format(exc))
        return 1
    except OSError as exc:
        print('\nREFUSED: {}'.format(exc))
        return 1


if __name__ == '__main__':
    sys.exit(main())
