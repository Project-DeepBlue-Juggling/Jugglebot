#!/usr/bin/env python3
"""Drive a full admissible-box re-sweep: every job family once (round a), then a
ROTATING SAMPLE of families again (round b) and a byte-for-byte compare (JB-150).

Replaces the uncommitted ``temp/sweeps/sweep_h4500.sh`` runner. Round a is the
same invocations as that script (26 families = the committed box; ``--set h70``
adds the four extra columns-apex families of the 29-family sweep), so its
outputs are byte-identical to the old path for the same commit. Round b used to
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

    python tools/admissible_sweep_run.py --out temp/sweeps/h4500 [--sample all|N]
        [--hand-acc 4500] [--workers 6] [--set committed|h70] [--list-sample]

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
_COMMON = ['--leg-vel', '350', '--leg-acc', '5000', '--leg-jerk', '200000']
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
    if which == 'h70':
        for i, ap in enumerate((['1.00', '1.05', '1.10'], ['1.15', '1.20', '1.25'],
                                ['1.30', '1.35', '1.40'], ['1.45', '1.50']), 1):
            res.append(('columnsx%d_pose' % i, ['--planner-mode', 'pose'] + d27 + _col(ap)))
    elif which != 'committed':
        raise ValueError('unknown family set %r' % which)
    return res


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


def _run_job(name: str, args: List[str], hand_acc: str, out_dir: str, rnd: str) -> Tuple[str, int]:
    path = os.path.join(out_dir, 'box_%s_%s.yaml' % (name, rnd))
    log = os.path.join(out_dir, 'logs', '%s_%s.log' % (name, rnd))
    cmd = ([sys.executable, os.path.join(_ROOT, 'tools', 'admissible_sweep.py')]
           + _COMMON + ['--hand-acc', hand_acc] + args + ['--out', path])
    with open(log, 'w') as fh:
        rc = subprocess.call(cmd, stdout=fh, stderr=subprocess.STDOUT, cwd=_ROOT)
    return name, rc


def _run_round(fams, rnd, hand_acc, out_dir, workers) -> None:
    with cf.ThreadPoolExecutor(max_workers=workers) as ex:
        futs = [ex.submit(_run_job, n, a, hand_acc, out_dir, rnd) for n, a in fams]
        bad = [n for n, rc in (f.result() for f in futs) if rc != 0]
    if bad:
        raise SystemExit('round %s: job(s) FAILED (see logs/): %s' % (rnd, ', '.join(bad)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--out', required=True, help='output directory')
    ap.add_argument('--hand-acc', default='4500')
    ap.add_argument('--workers', type=int, default=6)
    ap.add_argument('--set', dest='fam_set', default='committed', choices=('committed', 'h70'))
    ap.add_argument('--sample', default=str(DEFAULT_SAMPLE),
                    help='round-b families: N (default 3) or "all"')
    ap.add_argument('--list-sample', action='store_true',
                    help='print the round-b sample for the live gate_hash and exit')
    args = ap.parse_args(argv)
    sys.path.insert(0, os.path.join(_ROOT, 'ros_ws', 'src', 'jugglebot'))
    from jugglebot.motion.skills import admissible as adm
    fams = families(args.fam_set)
    names = [n for n, _ in fams]
    ghash = adm.gate_hash()
    nsamp = None if args.sample == 'all' else int(args.sample)
    sample = round_b_sample(names, ghash, nsamp)
    print('gate_hash %s; round-b sample (%s of %d): %s'
          % (ghash, args.sample, len(names), ', '.join(sample)), flush=True)
    if args.list_sample:
        return 0
    out = os.path.abspath(args.out)
    os.makedirs(os.path.join(out, 'logs'), exist_ok=True)
    _run_round(fams, 'a', args.hand_acc, out, args.workers)
    print('round a done', flush=True)
    _run_round([f for f in fams if f[0] in sample], 'b', args.hand_acc, out, args.workers)
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
    assert parts[0]['limits']['hand_acc_rps2'] == float(args.hand_acc)
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
