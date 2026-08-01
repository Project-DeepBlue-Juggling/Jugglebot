# -*- coding: utf-8 -*-
"""JS-vs-Python kinematics golden-vector check for the GUI FK solver.

``ros_ws/gui/js/stewart-fk.js`` is the repo's only *duplicate* kinematics
implementation: a hand-port of ``jugglebot.motion.ik_solver``'s IK plus its own
Newton-Raphson FK.  It was pinned only by a manual browser page
(``ros_ws/gui/test_fk.html``), so a divergence showed up as a wrong 3D platform
pose on the dashboard with nothing red anywhere.

Python is ground truth.  ``tools/gen_gui_fk_golden.py`` writes
``tests/ros/gui_fk_golden.json`` from the production Python IK; this test

1. re-derives the JSON in memory and fails if the committed file drifted
   (the Python-side pin — it holds even with no node installed), and
2. replays the same vectors through the JS implementation under node and
   compares IK, FK-from-extensions and FK-from-motor-revs.

**How the JS is loaded.** The GUI sources are ES modules living in a directory
with no ``package.json``, so node parses ``.js`` there as CommonJS and rejects
``export``.  The fixture copies ``stewart-fk.js`` and ``geometry-config.js``
*verbatim* into ``tmp_path`` next to a ``{"type": "module"}`` package.json and
the harness.  No source rewriting — the bytes under test are the shipped bytes.
``stewart-fk.js`` imports nothing but ``geometry-config.js`` (no THREE.js), and
:func:`test_stewart_fk_has_no_unexpected_imports` fails loudly if that changes,
rather than letting node emit a cryptic resolution error.

**Tolerances** (measured on node v22.22.3, 2026-08-01, 25 vectors; each is
>= 10x the observed worst case):

===========================  ============  ============
quantity                     observed max  tolerance
===========================  ============  ============
IK leg extensions            1.14e-13 mm   1e-9 mm
rotvec -> rotation matrix    1.11e-16      1e-12
FK position                  4.64e-07 mm   5e-6 mm
FK rotation matrix           3.36e-09      1e-7
FK rotation vector           3.38e-09 rad  1e-7 rad
===========================  ============  ============

IK is pure arithmetic on identical generated constants, so it agrees to
round-off.  FK cannot: the JS solver stops at ``max|residual| < 1e-6 mm`` with a
finite-difference Jacobian, and the pose error is that residual mapped through
J^-1 — sub-micron, i.e. ~5 nm here.  A genuine port bug (a swapped node, a
sign, a rev/mm inversion) is millimetre-scale and blows every one of these
gates by six orders of magnitude.

``FK_POS_TOL_MM`` is the tightest margin here, so its headroom is *derived*, not
just observed.  Raising ``stewart-fk.js``'s ``const tol = 1e-6`` in a sandbox
copy and re-running these vectors (node v22.22.3, 2026-08-01):

===================  =====================
solver residual tol  max FK position error
===================  =====================
1e-5 mm              5.294e-06 mm  (0.53x)
1e-6 mm (shipped)    4.637e-07 mm  (0.46x)
1e-7 mm              7.958e-09 mm  (0.08x)
===================  =====================

So the error is *bounded* by ~0.53x the residual tolerance (it falls faster than
linearly below that, because Newton's last step overshoots the stopping test).
At the shipped 1e-6 the algorithmic ceiling is therefore ~5.3e-7 mm and
``FK_POS_TOL_MM = 5e-6`` is ~10x that ceiling — not 10x one lucky sample.  A
libm/ULP difference on another node build can at worst change how far the final
iterate undershoots the stopping test, which stays inside the ceiling.  Do not
loosen this gate to accommodate a failure; re-derive the ceiling first.
"""

from __future__ import annotations

import glob
import json
import os
import re
import shutil
import subprocess

import pytest

from tools.gen_gui_fk_golden import DEFAULT_OUT, render

_TESTS_ROS_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.dirname(os.path.dirname(_TESTS_ROS_DIR))
_GUI_JS_DIR = os.path.join(_REPO_ROOT, 'ros_ws', 'gui', 'js')
_HARNESS = os.path.join(_TESTS_ROS_DIR, 'js', 'fk_harness.js')

#: GUI modules copied into the sandbox.  stewart-fk.js imports only the first.
_JS_SOURCES = ('stewart-fk.js', 'geometry-config.js')

IK_EXT_TOL_MM = 1e-9
ROT_MATRIX_TOL = 1e-12
FK_POS_TOL_MM = 5e-6
FK_ROT_TOL = 1e-7
FK_ROTVEC_TOL_RAD = 1e-7

_IMPORT_RE = re.compile(r"""from\s+['"](\./[^'"]+)['"]""")


# ---------------------------------------------------------------------------
# node discovery (never assume PATH)
# ---------------------------------------------------------------------------

def _find_node():
    """Absolute path to a usable node binary, or ``None``."""
    found = shutil.which('node') or shutil.which('nodejs')
    if found:
        return found
    patterns = (
        os.path.expanduser('~/.nvm/versions/node/*/bin/node'),
        '/usr/local/bin/node',
        '/usr/bin/node',
        '/opt/node*/bin/node',
    )
    candidates = []
    for pattern in patterns:
        candidates.extend(glob.glob(pattern))
    # Highest version first, sorted NUMERICALLY: a lexicographic sort puts
    # nvm's "v9.x" above "v22.x" and would pick an ancient interpreter (and
    # then skip) on a box that has a modern one installed.
    for candidate in sorted(candidates, key=_path_version, reverse=True):
        if os.access(candidate, os.X_OK):
            return candidate
    return None


def _path_version(path):
    """Version tuple parsed out of an nvm-style path; (0,) when absent."""
    match = re.search(r'/v(\d+)\.(\d+)\.(\d+)/', path)
    return tuple(int(g) for g in match.groups()) if match else (0, 0, 0)


_NODE = _find_node()


def _node_major(binary):
    try:
        out = subprocess.check_output([binary, '--version'],
                                      stderr=subprocess.STDOUT).decode()
    except Exception:                                    # pragma: no cover
        return 0
    match = re.match(r'v(\d+)\.', out.strip())
    return int(match.group(1)) if match else 0


_NODE_MAJOR = _node_major(_NODE) if _NODE else 0

#: ESM + `node:fs` specifiers need >= 14.18.  Skip (never fail) otherwise so
#: the suite stays portable to boxes with no node.
requires_node = pytest.mark.skipif(
    _NODE is None or _NODE_MAJOR < 14,
    reason='node >= 14 not found (looked on PATH, ~/.nvm, /usr/local, /usr, /opt)')


# ---------------------------------------------------------------------------
# Golden file (Python side — runs everywhere)
# ---------------------------------------------------------------------------

def _golden():
    with open(DEFAULT_OUT, 'r', encoding='utf-8') as handle:
        return json.load(handle)


def _walk_numbers(node, path=''):
    """Yield ``(path, value)`` for every leaf in a nested JSON document."""
    if isinstance(node, dict):
        for key in sorted(node):
            for item in _walk_numbers(node[key], path + '.' + str(key)):
                yield item
    elif isinstance(node, list):
        for index, value in enumerate(node):
            for item in _walk_numbers(value, path + '[%d]' % index):
                yield item
    else:
        yield path, node


def test_golden_file_matches_python_ground_truth():
    """The committed vectors are what today's Python IK produces.

    Compared as parsed structure with a tight relative tolerance, NOT as bytes.
    This assertion is deliberately outside ``requires_node`` so it guards the
    golden file on every box — including the Win10 clone that runs ``tests/ros/``
    under the mocked-ROS conftest and skips the JS half.  A byte compare would
    pin numpy's float repr there too, so a different numpy/BLAS build could
    redden the gate on a last-bit difference, with an unreadable 25-vector JSON
    diff as the only clue.  1e-12 relative is ~4 orders tighter than the
    loosest kinematics gate in this file, so a real IK change still fails.
    """
    with open(DEFAULT_OUT, 'r', encoding='utf-8') as handle:
        committed = json.load(handle)
    fresh = json.loads(render())

    stale = ('tests/ros/gui_fk_golden.json is stale relative to '
             'jugglebot.motion.ik_solver — regenerate with '
             'python tools/gen_gui_fk_golden.py (and re-check the JS side): ')

    committed_leaves = dict(_walk_numbers(committed))
    fresh_leaves = dict(_walk_numbers(fresh))
    assert sorted(committed_leaves) == sorted(fresh_leaves), (
        stale + 'shape changed (%r)'
        % sorted(set(committed_leaves) ^ set(fresh_leaves))[:10])

    for key, want in fresh_leaves.items():
        got = committed_leaves[key]
        if isinstance(want, float) or isinstance(got, float):
            assert abs(got - want) <= 1e-12 * max(1.0, abs(want)), (
                stale + '%s: committed %r vs fresh %r' % (key, got, want))
        else:
            assert got == want, (
                stale + '%s: committed %r vs fresh %r' % (key, got, want))


def test_golden_file_spans_the_workspace():
    doc = _golden()
    vectors = doc['vectors']
    assert len(vectors) >= 24, len(vectors)
    stroke = doc['geometry']['leg_stroke_mm']

    all_ext = [e for v in vectors for e in v['extensions_mm']]
    assert min(all_ext) > 0.0 and max(all_ext) < stroke
    # Real span, not a cluster around the active pose.
    assert min(all_ext) < 0.15 * stroke
    assert max(all_ext) > 0.85 * stroke

    labels = [v['label'] for v in vectors]
    assert len(set(labels)) == len(labels)
    for axis in ('roll', 'pitch', 'yaw'):
        assert any(axis in label for label in labels), axis
    assert any(any(abs(c) > 1e-9 for c in v['rotvec_rad']) for v in vectors)
    assert any(abs(v['pos_mm'][0]) > 1e-9 for v in vectors)
    assert any(abs(v['pos_mm'][1]) > 1e-9 for v in vectors)


def test_motor_revs_match_extensions_through_mm_to_rev():
    """The revs column is extensions * mm_to_rev — not its reciprocal.

    Pins the direction of the conversion in the golden file itself, so the JS
    comparison cannot be satisfied by two implementations sharing an inversion.
    """
    doc = _golden()
    mm_to_rev = doc['geometry']['mm_to_rev']
    for vector in doc['vectors']:
        for ext, rev, k in zip(vector['extensions_mm'],
                               vector['motor_revs'], mm_to_rev):
            assert abs(rev - ext * k) < 1e-12
            # 1 mm is well under 1 rev on this drive — a flipped factor is ~70x.
            assert abs(rev) < abs(ext) or abs(ext) < 1e-9


def test_stewart_fk_has_no_unexpected_imports():
    """The sandbox copy set still covers everything stewart-fk.js imports."""
    with open(os.path.join(_GUI_JS_DIR, 'stewart-fk.js'), 'r',
              encoding='utf-8') as handle:
        source = handle.read()
    imported = {os.path.basename(m) for m in _IMPORT_RE.findall(source)}
    unexpected = imported - set(_JS_SOURCES)
    assert not unexpected, (
        'stewart-fk.js gained relative import(s) %r — add them to _JS_SOURCES '
        '(or write a shim) so the node harness keeps testing the real file'
        % sorted(unexpected))
    # A bare (non-relative) import would be an npm/CDN package — THREE.js is the
    # one the GUI actually uses, and no shim can honestly stand in for it.
    bare = re.findall(r"""(?:from|import)\s+['"]([^'".][^'"]*)['"]""", source)
    assert not bare, (
        'stewart-fk.js gained non-relative import(s) %r — the node harness '
        'cannot resolve them without npm; shim or defer explicitly' % bare)


# ---------------------------------------------------------------------------
# JS side
# ---------------------------------------------------------------------------

@pytest.fixture(scope='module')
def js_results(tmp_path_factory):
    """Run the GUI kinematics under node over the golden vectors.

    Sandbox comes from ``tmp_path_factory`` (module-scope compatible), not
    ``tempfile.mkdtemp``: pytest reaps its own tmp root, so a killed xdist worker
    cannot leak a ``/tmp/gui_fk_*`` tree — and CLAUDE.md's runtime-artifact
    convention points away from ``/tmp`` anyway.
    """
    if _NODE is None or _NODE_MAJOR < 14:
        pytest.skip('node unavailable')

    sandbox = str(tmp_path_factory.mktemp('gui_fk'))
    for name in _JS_SOURCES:
        shutil.copy(os.path.join(_GUI_JS_DIR, name),
                    os.path.join(sandbox, name))
    shutil.copy(_HARNESS, os.path.join(sandbox, 'fk_harness.js'))
    with open(os.path.join(sandbox, 'package.json'), 'w',
              encoding='utf-8') as handle:
        handle.write('{"type": "module"}\n')

    # No eager rmtree: pytest's tmp root keeps the last few runs, so a harness
    # failure leaves the exact sandbox on disk to re-run node against by hand.
    proc = subprocess.run(
        [_NODE, os.path.join(sandbox, 'fk_harness.js'), DEFAULT_OUT],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)
    if proc.returncode != 0:
        pytest.fail('node harness failed (exit %d) in %s\nstderr:\n%s'
                    % (proc.returncode, sandbox,
                       proc.stderr.decode('utf-8', 'replace')))
    return json.loads(proc.stdout.decode('utf-8'))


def _paired(js_results):
    golden = _golden()['vectors']
    results = js_results['results']
    assert len(results) == len(golden)
    for expected, actual in zip(golden, results):
        assert expected['label'] == actual['label']
        yield expected, actual


@requires_node
def test_js_ik_matches_python(js_results):
    """poseToLegLengths() == pose_to_leg_lengths() to round-off."""
    for expected, actual in _paired(js_results):
        for leg, (want, got) in enumerate(zip(expected['extensions_mm'],
                                              actual['ik_extensions_mm'])):
            assert abs(want - got) < IK_EXT_TOL_MM, (
                '%s leg %d: python %.12f mm vs js %.12f mm'
                % (expected['label'], leg, want, got))


@requires_node
def test_js_rotvec_to_matrix_matches_python(js_results):
    for expected, actual in _paired(js_results):
        for row_want, row_got in zip(expected['rot_matrix'], actual['rot_matrix']):
            for want, got in zip(row_want, row_got):
                assert abs(want - got) < ROT_MATRIX_TOL, expected['label']


@requires_node
@pytest.mark.parametrize('channel', ['fk_from_extensions', 'fk_from_revs'])
def test_js_fk_recovers_the_golden_pose(js_results, channel):
    """solveFK()/legLengthsToPose() invert the Python IK back to the pose."""
    for expected, actual in _paired(js_results):
        fk = actual[channel]
        assert fk['converged'], '%s: JS FK did not converge (%s)' % (
            expected['label'], channel)

        for axis, (want, got) in enumerate(zip(expected['pos_mm'], fk['pos_mm'])):
            assert abs(want - got) < FK_POS_TOL_MM, (
                '%s [%s] pos[%d]: python %.9f mm vs js %.9f mm'
                % (expected['label'], channel, axis, want, got))

        for row_want, row_got in zip(expected['rot_matrix'], fk['rot_matrix']):
            for want, got in zip(row_want, row_got):
                assert abs(want - got) < FK_ROT_TOL, (
                    '%s [%s] rotation matrix' % (expected['label'], channel))

        for axis, (want, got) in enumerate(zip(expected['rotvec_rad'],
                                               fk['rotvec_rad'])):
            assert abs(want - got) < FK_ROTVEC_TOL_RAD, (
                '%s [%s] rotvec[%d]: python %.12f vs js %.12f'
                % (expected['label'], channel, axis, want, got))


@requires_node
def test_js_fk_entry_points_agree(js_results):
    """``solveFK(extensions)`` and ``legLengthsToPose(revs)`` land on one pose.

    Both are solved from the same cold warm-start (``resetFKState()`` between
    them — ``prevState`` is module-level in stewart-fk.js, so without the reset a
    vector's answer would depend on which vector ran before it), so the only
    difference is the ``revs / MM_TO_REV`` round trip. Measured worst case
    2.3e-9 mm on node v22.22.3, 2026-08-01; the gate is 100x that and two
    orders tighter than :data:`FK_POS_TOL_MM`, so it stays sensitive to a
    conversion-path regression that the golden comparison alone would absorb.
    """
    for expected, actual in _paired(js_results):
        ext = actual['fk_from_extensions']
        rev = actual['fk_from_revs']
        for want, got in zip(ext['pos_mm'], rev['pos_mm']):
            assert abs(want - got) < 2.3e-7, expected['label']
