#!/usr/bin/env python3
"""The Columns golden: every field of every Schedule a matrix of Columns
compiles produces, serialised EXACTLY (plan ``learner-generalisation`` P4,
test T-R1).

Written ONCE, on the tree BEFORE ``compile_columns`` became a wrapper over
``compile_beats`` (2026-11-06, branch p4-beat-events at 0b60a7a6), so the
fixture is today's compiler's output and the refactor has to reproduce it bit
for bit.  The fixture is never regenerated to make a failing check pass: if
``--check`` fails after a compiler edit, the compiler is wrong, not the
fixture.

Serialisation (one rule per type, no tolerance anywhere):

- a float (Python or numpy) -> ``float.hex()``;
- a numpy array -> a list of hex floats;
- a :class:`Site` -> ``{"site": name, "cup_mm": [hex, hex, hex]}``;
- a dataclass (``Skill``, ``ThenThrow``, ``LandingPrior``, ``Schedule``) ->
  ``{"__type__": class name, <every dataclasses.fields() name>: value}``, so a
  field added later is a visible difference, not a silent omission (the one
  deliberate exception, ``_SKIPPED``, is listed with its reason; a field
  added later whose None is the old compile is written only when set,
  ``_ABSENT_WHEN_NONE``);
- None -> null, a bool -> a bool, an int -> an int, a str -> a str, a tuple /
  list -> a list.

A case that raises ``ValueError`` today is recorded as ``{"error": message}``
with the exact message.

Usage::

    python tools/probes/columns_golden.py --check   # exit 1 on any difference
    python tools/probes/columns_golden.py --write   # ONLY on the pre-refactor tree

The fixture: ``tests/fixtures/columns_schedule_golden_2026-11-06.json``, read
by ``tests/motion/test_skills_beats.py`` (which imports this file by path and
recompiles every case through the public compilers).
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import os
import sys

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(
    __file__))))
for _p in (os.path.join(_REPO, 'ros_ws', 'src', 'jugglebot'),
           os.path.join(_REPO, 'config', 'generated'), _REPO):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402

from jugglebot.motion.skills import schedule as sc  # noqa: E402
from jugglebot.motion.skills import sites as st  # noqa: E402

FIXTURE = os.path.join(_REPO, 'tests', 'fixtures',
                       'columns_schedule_golden_2026-11-06.json')

#: (class name, field name) pairs the field walk deliberately skips.
#: ``Schedule.source`` (P4, D-P4-1) is the BeatSource a Schedule was compiled
#: from: a recipe for recompiling, not compiled output, and it carries the
#: event stream whose own equality the beats tests pin separately
#: (``compile_columns(...).source.events == columns_events(...)``).  It is
#: ``compare=False`` on the dataclass for the same reason.
_SKIPPED = {('Schedule', 'source')}

#: (class name, field name) pairs ADDED after the fixture was written whose
#: ``None`` is exactly the pre-change compile: written only when NOT None, so
#: every recorded case still pins them None (a non-None value is a visible
#: difference) without regenerating the fixture.
#: ``Skill.receive_tilt_offset`` (IDEA-22, 2026-11-12): the turned-Columns
#: 90 deg Catch tilt-back; ``Pattern.catch_tilt_offset`` defaults None, so
#: every case here, the 45 and 90 deg turned pairs included, compiles None.
_ABSENT_WHEN_NONE = {('Skill', 'receive_tilt_offset')}


def ser(v):
    """The exact serialisation (module docstring)."""
    if v is None or isinstance(v, (bool, np.bool_)):
        return None if v is None else bool(v)
    if isinstance(v, (float, np.floating)):
        return float(v).hex()
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, str):
        return v
    if isinstance(v, np.ndarray):
        return [float(x).hex() for x in v.reshape(-1)]
    if isinstance(v, st.Site):
        return {'site': v.name, 'cup_mm': ser(v.cup_mm)}
    if dataclasses.is_dataclass(v) and not isinstance(v, type):
        name = type(v).__name__
        out = {'__type__': name}
        for f in dataclasses.fields(v):
            if (name, f.name) in _SKIPPED:
                continue
            val = getattr(v, f.name)
            if val is None and (name, f.name) in _ABSENT_WHEN_NONE:
                continue
            out[f.name] = ser(val)
        return out
    if isinstance(v, (tuple, list)):
        return [ser(x) for x in v]
    raise TypeError('columns_golden cannot serialise %r (%s)' % (v, type(v)))


def _h(x: float) -> str:
    return float(x).hex()


def _f(s) -> float:
    return float.fromhex(s) if isinstance(s, str) else float(s)


# ── the matrix ──────────────────────────────────────────────────────────────

_T0_SMALL = 10.0
_T0_WALL = 1789263419.5          # the R2 sitting's wall clock (2026-09-13)
_FEED_T_LAND = 1789263419.8125   # exactly representable, wall-clock sized
_FEED_VEL = (310.0, -120.0, -4150.0)
_BB_VEL = (4800.0 * np.sin(np.radians(25.0)), 0.0,
           -4800.0 * np.cos(np.radians(25.0)))


def _inputs(kind='columns', n=4, clock='t0_small', phantom=(), apex=0.9,
            apex_b=None, dwell=0.27, sep=100.0, angle=0.0, launch_s=None,
            rest_tail_s=None):
    inp = {'kind': kind, 'n_throws': n, 'clock': clock,
           'phantom_balls': list(phantom), 'apex_m': _h(apex),
           'apex_m_ball_b': None if apex_b is None else _h(apex_b),
           'dwell_s': _h(dwell), 'separation_mm': _h(sep),
           'angle_deg': _h(angle),
           'launch_s': None if launch_s is None else _h(launch_s)}
    # Added with the 2026-11-06 audit cases: the key exists ONLY when given, so
    # the first 201 cases' recorded inputs are byte-identical to the first write.
    if rest_tail_s is not None:
        inp['rest_tail_s'] = _h(rest_tail_s)
    return inp


def matrix():
    """The case list (``name`` -> inputs), in a fixed order."""
    cases = []

    def add(name, **kw):
        cases.append((name, _inputs(**kw)))

    clocks = ('t0_small', 't0_wall', 'feed', 'feed_hop')
    # Core axes at the reference geometry: n x clock x phantom.
    for n in (1, 2, 3, 4, 5, 8, 20):
        for clock in clocks:
            for ph in ((), (0,), (1,)):
                add('core n%d %s ph%s' % (n, clock, ''.join(map(str, ph))),
                    n=n, clock=clock, phantom=ph)
    # Apex / dwell: equal Apexes and high-low pairs at two dwells.
    for dwell in (0.27, 0.35):
        for apex, apex_b in ((0.8, None), (0.9, None), (1.05, None),
                             (1.05, 0.95), (0.90, 1.00)):
            for n in (2, 3, 8):
                for clock in ('t0_wall', 'feed_hop'):
                    add('apex %s/%s d%s n%d %s'
                        % (apex, apex_b, dwell, n, clock),
                        n=n, clock=clock, apex=apex, apex_b=apex_b,
                        dwell=dwell)
            add('apex %s/%s d%s n5 feed ph1' % (apex, apex_b, dwell),
                n=5, clock='feed', apex=apex, apex_b=apex_b, dwell=dwell,
                phantom=(1,))
    # An explicitly equal apex_m_ball_b (must compile as plain Columns).
    add('apex equal-b 0.9/0.9 n4 t0_wall', n=4, clock='t0_wall', apex=0.9,
        apex_b=0.9)
    # Geometry: two separations and a turned pair.
    for sep, angle in ((100.0, 0.0), (125.0, 0.0), (125.0, 45.0),
                       (100.0, 90.0)):
        for n in (3, 8):
            for clock in ('t0_small', 'feed_hop'):
                for apex, apex_b in ((0.9, None), (1.05, 0.95)):
                    add('geom s%s a%s n%d %s %s/%s'
                        % (sep, angle, n, clock, apex, apex_b),
                        n=n, clock=clock, sep=sep, angle=angle, apex=apex,
                        apex_b=apex_b)
    # A non-default launch window.
    add('launch 0.6 n4 t0_wall', n=4, clock='t0_wall', launch_s=0.6)
    # Today's refusals, recorded with their exact messages.
    add('err dwell>=flight', n=4, clock='t0_small', apex=0.3, dwell=0.6)
    add('err highlow short transit', n=4, clock='t0_small', apex=1.05,
        apex_b=0.5, dwell=0.27)
    add('err highlow no transit', n=4, clock='t0_small', apex=1.05,
        apex_b=0.3, dwell=0.27)
    add('err dwell window floor', n=4, clock='t0_small', dwell=0.05)
    add('err launch window floor', n=4, clock='t0_small', launch_s=0.05)
    add('err n0', n=0, clock='t0_small')
    add('err both t0 and feed', n=4, clock='both')
    add('err neither t0 nor feed', n=4, clock='neither')
    add('err hop without feed', n=4, clock='t0_hop')
    # The two wrappers (columns_1ball only).
    for n in (4, 5):
        add('reload_columns n%d' % n, kind='reload_columns', n=n,
            clock='t0_wall', phantom=(1,))
        add('rest_columns n%d' % n, kind='rest_columns', n=n,
            clock='t0_wall', phantom=(1,))
    # ── added after the P4 audit (2026-11-06): axes the first matrix held
    # at one value. APPENDED, so the first 201 cases keep their order; written
    # by this generator run against the PRE-CHANGE compiler (a detached
    # checkout of 0b60a7a6), like the first 201.
    for rest in (0.25, 0.45):
        add('rest_tail %s n4 t0_wall' % rest, n=4, clock='t0_wall',
            rest_tail_s=rest)
        add('rest_tail %s n5 feed_hop' % rest, n=5, clock='feed_hop',
            rest_tail_s=rest)
        add('rest_tail %s n5 feed ph1 highlow' % rest, n=5, clock='feed',
            phantom=(1,), apex=1.05, apex_b=0.95, rest_tail_s=rest)
    for dwell in (0.30, 0.45):
        for n in (3, 8):
            for clock in ('t0_wall', 'feed_hop'):
                add('dwell %s n%d %s a1.05' % (dwell, n, clock), n=n,
                    clock=clock, apex=1.05, dwell=dwell)
        add('dwell %s n8 feed highlow' % dwell, n=8, clock='feed', apex=1.05,
            apex_b=0.95, dwell=dwell)
    for n in (6, 7, 12):
        for ph in ((), (0,), (1,)):
            add('more n%d t0_wall ph%s' % (n, ''.join(map(str, ph)) or '-'),
                n=n, clock='t0_wall', phantom=ph)
    add('launch 0.5 rest 0.4 dwell 0.33 n9 feed_hop', n=9, clock='feed_hop',
        launch_s=0.5, rest_tail_s=0.4, dwell=0.33)
    names = [c[0] for c in cases]
    assert len(names) == len(set(names)), 'duplicate case names'
    return cases


# ── compiling one case ──────────────────────────────────────────────────────

def pattern_of(inp) -> sc.Pattern:
    sites = st.columns_sites(_f(inp['separation_mm']), _f(inp['angle_deg']))
    kw = dict(sites=sites, apex_m=_f(inp['apex_m']), dwell_s=_f(inp['dwell_s']),
              n_throws=int(inp['n_throws']),
              phantom_balls=tuple(inp['phantom_balls']))
    if inp['apex_m_ball_b'] is not None:
        kw['apex_m_ball_b'] = _f(inp['apex_m_ball_b'])
    if inp['launch_s'] is not None:
        kw['launch_s'] = _f(inp['launch_s'])
    if inp.get('rest_tail_s') is not None:
        kw['rest_tail_s'] = _f(inp['rest_tail_s'])
    return sc.Pattern(**kw)


def _feed(pattern: sc.Pattern) -> sc.LandingPrior:
    return sc.LandingPrior(pos_mm=pattern.sites[1].catch_site_mm(),
                           vel_mm_s=np.array(_FEED_VEL),
                           t_land_abs_s=_FEED_T_LAND)


def compile_case(inp) -> sc.Schedule:
    """Compile one case through the PUBLIC compilers (raises what they raise)."""
    pattern = pattern_of(inp)
    kind = inp['kind']
    if kind == 'reload_columns':
        site_a = pattern.sites[0]
        return sc.compile_reload_columns(
            site_a.catch_site_mm(), np.array(_BB_VEL), _T0_WALL + 3.0,
            sc.FLOOR_LIFT_S, pattern, _T0_WALL)
    if kind == 'rest_columns':
        return sc.compile_rest_columns(pattern, pattern.sites[1],
                                       sc.FLOOR_LIFT_S, _T0_WALL)
    assert kind == 'columns', kind
    clock = inp['clock']
    if clock == 't0_small':
        return sc.compile_columns(pattern, t0_abs_s=_T0_SMALL)
    if clock == 't0_wall':
        return sc.compile_columns(pattern, t0_abs_s=_T0_WALL)
    if clock == 't0_hop':
        return sc.compile_columns(pattern, t0_abs_s=_T0_WALL, hop_entry=True)
    if clock == 'feed':
        return sc.compile_columns(pattern, feed=_feed(pattern))
    if clock == 'feed_hop':
        return sc.compile_columns(pattern, feed=_feed(pattern),
                                  hop_entry=True)
    if clock == 'both':
        return sc.compile_columns(pattern, t0_abs_s=_T0_WALL,
                                  feed=_feed(pattern))
    if clock == 'neither':
        return sc.compile_columns(pattern)
    raise AssertionError('unknown clock %r' % (clock,))


def result_of(inp):
    """The serialised result of one case: the Schedule, or the ValueError."""
    try:
        return {'schedule': ser(compile_case(inp))}
    except ValueError as exc:
        return {'error': str(exc)}


def build():
    return {'cases': [{'name': name, 'inputs': inp, 'result': result_of(inp)}
                      for name, inp in matrix()]}


def dump(doc) -> str:
    """One case per line: small, and a diff names the case."""
    lines = ['{"cases": [']
    cases = doc['cases']
    for k, c in enumerate(cases):
        lines.append(json.dumps(c, sort_keys=True, separators=(',', ':'))
                     + (',' if k < len(cases) - 1 else ''))
    lines.append(']}')
    return '\n'.join(lines) + '\n'


def first_difference(want, got, path=''):
    """The path and the two values of the first difference, or None."""
    if type(want) is not type(got):
        return path, want, got
    if isinstance(want, dict):
        for k in sorted(set(want) | set(got)):
            if k not in want or k not in got:
                return '%s.%s' % (path, k), want.get(k, '<absent>'), \
                    got.get(k, '<absent>')
            d = first_difference(want[k], got[k], '%s.%s' % (path, k))
            if d is not None:
                return d
        return None
    if isinstance(want, list):
        if len(want) != len(got):
            return path + '.<len>', len(want), len(got)
        for i, (a, b) in enumerate(zip(want, got)):
            d = first_difference(a, b, '%s[%d]' % (path, i))
            if d is not None:
                return d
        return None
    return None if want == got else (path, want, got)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--write', action='store_true')
    g.add_argument('--check', action='store_true')
    args = ap.parse_args(argv)
    doc = build()
    if args.write:
        with open(FIXTURE, 'w') as fh:
            fh.write(dump(doc))
        print('wrote %d cases to %s' % (len(doc['cases']), FIXTURE))
        return 0
    with open(FIXTURE) as fh:
        want = json.load(fh)
    if len(want['cases']) != len(doc['cases']):
        print('case count differs: fixture %d, now %d'
              % (len(want['cases']), len(doc['cases'])))
        return 1
    for w, g_ in zip(want['cases'], doc['cases']):
        d = first_difference(w, g_)
        if d is not None:
            print('DIFFERENT case %r at %s: fixture %r, now %r'
                  % (w['name'], d[0], d[1], d[2]))
            return 1
    print('OK: %d cases identical' % (len(doc['cases']),))
    return 0


if __name__ == '__main__':
    sys.exit(main())
