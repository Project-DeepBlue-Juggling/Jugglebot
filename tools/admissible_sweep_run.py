#!/usr/bin/env python3
"""Drive a full admissible-box re-sweep: every job family once (round a), then a
ROTATING SAMPLE of families again (round b) and a byte-for-byte compare (JB-150).

Replaces the uncommitted ``temp/sweeps/sweep_h4500.sh`` runner. The default
family set ``committed`` is 55 families (carry-ring-zvd P1c, JB-102): the 33
pre-ZVD families (the 26 of ``sweep_h4500.sh`` / ``--set h4500``, the four
columns-apex families above 0.95 (C-HAND-5), three one-ball oval dwells) of
which 22 are pose, plus a ``<name>_zvd`` twin (``--carry-zvd true``) of every
pose family; ``h70`` is the 30-family pre-oval set (49 with its twins). The leg and Hand limits default to the committed box's
own ``limits`` stamp, so a re-sweep at NEW limits (a working-point change)
must name them: ``--leg-vel 600 --leg-acc 8000 --leg-jerk 200000``. Round b used to
re-run ALL families (about half the wall time); rounds a and b have been
byte-identical on every sweep so far, so b now re-runs a sample (default 3) that
rotates with the gate_hash: ``start = int(gate_hash, 16) % n`` over the families
in the fixed ORDER, then every ``n // N``-th family from there. Different gated
code gives a different triple; ``--sample all`` restores the full round b.

Nothing is merged unless the sampled compare passes. The sampled names and the
verdict go to stdout (tee it), to ``<out>/round_b_verdict.json``, and are NOT
written into the box YAML (the loader's schema has no metadata block and a
mixed-in key would re-shape the committed file).

Usage (venv; run from a tree whose gated files are frozen)::

    python tools/admissible_sweep_run.py --out temp/sweeps/<name> [--sample all|N]
        [--leg-vel V --leg-acc A --leg-jerk J --hand-acc H] [--workers 6]
        [--set committed|h4500] [--list-sample]

Owner norm (2026-11-03): every box sweep is round a plus the sampled round b;
``--sample all`` only with a recorded a/b mismatch as the reason.

BLAS is pinned to one thread here and again inside admissible_sweep.py.
"""
from __future__ import annotations

import argparse
import concurrent.futures as cf
import json
import os
import subprocess
import sys
from typing import Dict, List, Optional, Sequence, Tuple

for _v in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[_v] = '1'

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SAMPLE = 3
_BOX = os.path.join(_ROOT, 'config', 'generated', 'admissible_box.yaml')
_OVAL = ['--site-pairs', 'oval', '--oval-separation-mm', '150', '--oval-apex', '0.95',
         '--single-apex-halfwidth', '0.025']
_OVALX = ['--site-pairs', 'oval', '--oval-separation-mm', '150', '--oval-apex',
          '1.00', '1.05', '1.10', '--single-apex-halfwidth', '0.025']


def _col(apexes: Sequence[str]) -> List[str]:
    return (['--site-pairs', 'columns', '--single-apex'] + list(apexes)
            + ['--single-apex-halfwidth', '0.025', '--separation-mm', '125'])


def families(which: str = 'committed') -> List[Tuple[str, List[str]]]:
    """(name, CLI args after the common/hand-acc ones), in MERGE ORDER."""
    def mode(m, name, args):
        return (name + '_' + m, ['--planner-mode', m] + args)
    out: List[Tuple[str, List[str]]] = []
    d27 = ['--dwell-s', '0.27']
    for m in ('level', 'pose'):
        base = [
            ('single', d27 + ['--site-pairs', 'single', '--single-apex',
                              '0.5', '0.6', '0.7', '0.8', '0.9']),
            ('hop', d27 + ['--site-pairs', 'hop', '--single-apex', '0.80', '0.85',
                           '0.90', '0.95', '--single-apex-halfwidth', '0.025',
                           '--hop-separation-mm', '250']),
            ('columns', d27 + _col(['0.85', '0.90', '0.95'])),
        ]
        base += [('oval%s' % d.replace('.', ''), ['--dwell-s', d] + _OVAL)
                 for d in ('0.65', '0.55', '0.50', '0.45', '0.40', '0.35')]
        base += [('ovalx%s' % d.replace('.', ''), ['--dwell-s', d] + _OVALX)
                 for d in ('0.50', '0.45')]
        out += [mode(m, n, a) for n, a in base]
    # merge order of the committed box: level block, then pose block, per sweep_h4500
    order = ['single_level', 'hop_level', 'columns_level', 'oval065_level', 'oval055_level',
             'oval050_level', 'oval045_level', 'oval040_level', 'oval035_level',
             'single_pose', 'hop_pose', 'columns_pose', 'oval065_pose', 'oval055_pose',
             'oval050_pose', 'oval045_pose', 'oval040_pose', 'oval035_pose',
             'ovalx050_level', 'ovalx045_level', 'ovalx050_pose', 'ovalx045_pose']
    by = dict(out)
    res = [(n, by[n]) for n in order]
    for a in ('45', '90'):
        res.append(('turned%s_pose' % a, ['--planner-mode', 'pose'] + d27 + [
            '--site-pairs', 'columns', '--angle-deg', a, '--separation-mm', '125',
            '--single-apex', '0.85', '0.90', '0.95', '--single-apex-halfwidth', '0.025']))
    for pair in (('1.05', '0.95'), ('1.00', '0.90')):
        res.append(('hl_100_' + ''.join(pair).replace('.', ''), ['--planner-mode', 'pose']
                    + d27 + ['--site-pairs', 'columns', '--apex-pair', pair[0], pair[1],
                             '--separation-mm', '100', '--single-apex-halfwidth', '0.025']))
    if which in ('committed', 'h70'):
        for i, ap in enumerate((['1.00', '1.05', '1.10'], ['1.15', '1.20', '1.25'],
                                ['1.30', '1.35', '1.40'], ['1.45', '1.50']), 1):
            res.append(('columnsx%d_pose' % i, ['--planner-mode', 'pose'] + d27 + _col(ap)))
        # Two-ball-oval P4 (2026-11-03; committed set only, h70 stays the JB-144 30):
        # the one-ball oval ladder at the P4 working point, pose planner only (level is CUP_SEATING_CONE-refused at 0.40 at any
        # leg limit, plan amendment 52-58). `p4_headroom_cells.py` plans all three
        # dwells at apex 1.00 / 1.05 at 600/8000/200k with the Hand at 4500.
        for d in ('0.40', '0.35', '0.30') if which == 'committed' else ():
            res.append(('ovalx%s_pose' % d.replace('.', ''),
                        ['--planner-mode', 'pose', '--dwell-s', d] + _OVALX))
    elif which != 'h4500':
        raise ValueError('unknown family set %r' % which)
    # carry-ring-zvd P1c (JB-102): the ZVD ring rows exist only in the pose
    # planner. Every family states its flag explicitly (never the yaml default,
    # so a yaml flip cannot silently re-label a sweep); level families run once
    # (stamped false, the flag is inert there); every pose family runs TWICE,
    # flag false in place and its `<name>_zvd` twin (flag true) appended after
    # the existing order, so a merge keeps the pre-ZVD cells first.
    out_res = [(n, a + ['--carry-zvd', 'false']) for n, a in res]
    out_res += [(n + '_zvd', a + ['--carry-zvd', 'true']) for n, a in res
                if a[:2] == ['--planner-mode', 'pose']]
    return out_res


def family_plan(fams: Sequence[Tuple[str, List[str]]]) -> Dict[Tuple[str, str], List[str]]:
    """Family names grouped by (planner_mode, carry_zvd), read off each
    family's own args -- the dry listing of what the sweep will stamp."""
    plan: Dict[Tuple[str, str], List[str]] = {}
    for name, args in fams:
        mode = args[args.index('--planner-mode') + 1]
        zvd = args[args.index('--carry-zvd') + 1]
        plan.setdefault((mode, zvd), []).append(name)
    return plan


def round_b_sample(names: Sequence[str], gate_hash: str, n: Optional[int]) -> List[str]:
    """The round-b families: all of them when ``n`` is None or >= len(names),
    else ``n`` names spread ``len//n`` apart from ``int(gate_hash,16) % len``.
    Pure and deterministic: a function of (names, gate_hash, n) only."""
    k = len(names)
    if n is None or n >= k:
        return list(names)
    if n < 1:
        raise ValueError('--sample must be >= 1 or "all"')
    start = int(gate_hash, 16) % k
    step = k // n
    return [names[(start + i * step) % k] for i in range(n)]


def committed_limits(path: str = _BOX) -> Dict[str, str]:
    """The committed box's own ``limits`` stamp, as CLI strings (``%g``)."""
    import yaml
    with open(path) as fh:
        lim = yaml.safe_load(fh)['limits']
    return {'leg_vel': '%g' % lim['leg_vel_mmps'], 'leg_acc': '%g' % lim['leg_acc_mmps2'],
            'leg_jerk': '%g' % lim['leg_jerk_mmps3'], 'hand_acc': '%g' % lim['hand_acc_rps2']}


def _limit_args(lim: Dict[str, str]) -> List[str]:
    return ['--leg-vel', lim['leg_vel'], '--leg-acc', lim['leg_acc'],
            '--leg-jerk', lim['leg_jerk'], '--hand-acc', lim['hand_acc']]


def _run_job(name: str, args: List[str], lim: Dict[str, str], out_dir: str, rnd: str) -> Tuple[str, int]:
    path = os.path.join(out_dir, 'box_%s_%s.yaml' % (name, rnd))
    log = os.path.join(out_dir, 'logs', '%s_%s.log' % (name, rnd))
    cmd = ([sys.executable, os.path.join(_ROOT, 'tools', 'admissible_sweep.py')]
           + _limit_args(lim) + args + ['--out', path])
    with open(log, 'w') as fh:
        rc = subprocess.call(cmd, stdout=fh, stderr=subprocess.STDOUT, cwd=_ROOT)
    return name, rc


def _run_round(fams, rnd, lim, out_dir, workers) -> None:
    with cf.ThreadPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(_run_job, n, a, lim, out_dir, rnd) for n, a in fams]
        bad = [n for n, rc in (f.result() for f in futs) if rc != 0]
    if bad:
        raise SystemExit('round %s: job(s) FAILED (see logs/): %s' % (rnd, ', '.join(bad)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', required=True, help='output directory')
    box_lim = committed_limits()
    ap.add_argument('--leg-vel', default=box_lim['leg_vel'])
    ap.add_argument('--leg-acc', default=box_lim['leg_acc'])
    ap.add_argument('--leg-jerk', default=box_lim['leg_jerk'])
    ap.add_argument('--hand-acc', default=box_lim['hand_acc'])
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--set', dest='fam_set', default='committed', choices=('committed', 'h70', 'h4500'))
    ap.add_argument('--sample', default=str(DEFAULT_SAMPLE),
                    help='round-b families: N (default 3) or "all"')
    ap.add_argument('--list-sample', action='store_true',
                    help='print the round-b sample for the live gate_hash and exit')
    args = ap.parse_args(argv)
    sys.path.insert(0, os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot'))
    from jugglebot.motion.skills import admissible as adm
    fams = families(args.fam_set)
    lim = {'leg_vel': args.leg_vel, 'leg_acc': args.leg_acc,
           'leg_jerk': args.leg_jerk, 'hand_acc': args.hand_acc}
    print('limits: %s' % ' '.join(_limit_args(lim)), flush=True)
    names = [n for n, _ in fams]
    ghash = adm.gate_hash()
    nsamp = None if args.sample == 'all' else int(args.sample)
    sample = round_b_sample(names, ghash, nsamp)
    print('gate_hash %s; round-b sample (%s of %d): %s'
          % (ghash, args.sample, len(names), ', '.join(sample)), flush=True)
    plan = family_plan(fams)
    for (mode, zvd), fnames in sorted(plan.items()):
        print('plan: planner_mode=%s carry_zvd=%s: %d families' % (mode, zvd, len(fnames)))
    print('plan: %d families total' % len(fams))
    if args.list_sample:
        # Cells per planner of the COMMITTED box (the pose count doubles under
        # the flag; the zvd cells are expected to be a subset where RING refuses).
        boxes = adm.load(_BOX)
        by_mode: Dict[str, int] = {}
        for b in boxes:
            by_mode[str(b.planner_mode)] = by_mode.get(str(b.planner_mode), 0) + 1
        print('plan: committed box cells by planner_mode %s; upper bound after the '
              'sweep: level %d + pose %d x 2 = %d' % (
                  dict(sorted(by_mode.items())), by_mode.get('level', 0),
                  by_mode.get('pose', 0),
                  by_mode.get('level', 0) + 2 * by_mode.get('pose', 0)))
        return 0
    out = os.path.abspath(args.out)
    os.makedirs(os.path.join(out, 'logs'), exist_ok=True)
    _run_round(fams, 'a', lim, out, args.workers)
    print('round a done', flush=True)
    _run_round([f for f in fams if f[0] in sample], 'b', lim, out, args.workers)
    print('round b done', flush=True)
    verdicts: Dict[str, str] = {}
    for n in sample:
        fa = os.path.join(out, 'box_%s_a.yaml' % n)
        fb = os.path.join(out, 'box_%s_b.yaml' % n)
        same = open(fa, 'rb').read() == open(fb, 'rb').read()
        verdicts[n] = 'IDENTICAL' if same else 'DIFFERS'
        print('%s %s' % (n, verdicts[n]), flush=True)
    ok = all(v == 'IDENTICAL' for v in verdicts.values())
    with open(os.path.join(out, 'round_b_verdict.json'), 'w') as fh:
        json.dump({'gate_hash': ghash, 'sample_arg': args.sample, 'families_total': len(names),
                   'sampled': sample, 'verdicts': verdicts,
                   'status': 'PASS' if ok else 'FAIL'}, fh, indent=1)
    print('CMP_STATUS %s' % ('0' if ok else '1'), flush=True)
    if not ok:
        print('ROUND B MISMATCH: sweep is NOT deterministic; nothing merged', file=sys.stderr)
        return 1
    import yaml
    parts = [yaml.safe_load(open(os.path.join(out, 'box_%s_a.yaml' % n))) for n in names]
    for key in ('swept_at', 'gate_hash', 'limits'):
        assert all(p[key] == parts[0][key] for p in parts), key
    got = parts[0]['limits']
    assert (got['leg_vel_mmps'], got['leg_acc_mmps2'], got['leg_jerk_mmps3'],
            got['hand_acc_rps2']) == tuple(float(lim[k]) for k in (
                'leg_vel', 'leg_acc', 'leg_jerk', 'hand_acc')), got
    merged = dict(parts[0])
    merged['boxes'] = [b for p in parts for b in p['boxes']]
    mpath = os.path.join(out, 'merged.yaml')
    with open(mpath, 'w') as fh:
        fh.write(yaml.safe_dump(merged, sort_keys=False))
    print('cells %d; gate_hash %s; loader accepts %d cells; live gate_hash %s'
          % (len(merged['boxes']), merged['gate_hash'], len(adm.load(mpath)), adm.gate_hash()))
    print('review the per-cell diff, then: cp %s config/generated/admissible_box.yaml' % mpath)
    return 0


if __name__ == '__main__':
    sys.exit(main())
