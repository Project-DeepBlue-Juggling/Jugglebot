"""One import root for sim/ — the enforcement point (Phase 6, refactor-2026-07).

Contract (normative text: ``sim/_paths.py``'s module docstring):

  1. sim code is imported as ``sim.*``, never bare (``from plant.x import ...``).
     Bare style plus ``sim/`` on ``sys.path`` loads a file under two module
     identities in one interpreter; a ``patch()`` against one then silently
     patches nothing — the class this contract closes.
  2. ``sys.path`` mutation lives in exactly one place, ``sim/_paths.py``.
     Library modules under ``sim/`` never touch it; runnable entry scripts
     call ``bootstrap_paths()``.

These tests are cheap greps: they fail the moment a new bare-style import or a
new hand-rolled path block lands, which is the only thing that keeps the
conversion from rotting back.
"""
from __future__ import annotations

import ast
import os

import pytest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))

#: Top-level names that live under sim/ and must never be imported bare.
#: ``juggle_planner`` is here even though no bare import of it has ever
#: existed: it arrived under ``sim/`` in the same phase (from
#: ``controller/demo/``), several still-active plan documents spell it
#: ``from juggle_planner.timeline import ...``, and a direct script run always
#: has ``sim/`` on ``sys.path`` — so without this entry the newest package is
#: the one hole through which the dual-identity class could re-open.
#: ``tools`` is deliberately absent: ``tools.*`` is a real repo-root package.
_SIM_TOP_LEVEL = frozenset({
    'analysis', 'ball', 'ball_butler', 'demo_mpc', 'gate_common', 'hand',
    'input', 'juggle_bb_catch', 'juggle_catch', 'juggle_demo', 'juggle_noise',
    'juggle_online', 'juggle_planner', 'juggle_selfcatch', 'juggle_throw',
    'juggle_tilt', 'model', 'plant', 'reload_gate', 'toss_gate', 'viz',
})

#: Directories scanned.  attic/ is frozen history; ros_ws is a separate root.
#: ``teensy_link`` is scanned since it became a repo-root package (2026-08-01,
#: refactor Phase 4) — relocating a package out of ``controller/`` must not
#: silently drop it out of the bare-import scan.  Only the bare-import rule
#: reaches it: the sys.path-mutation rule below is explicitly ``sim/``-only, and
#: ``teensy_link/protocol.py``'s insert is the deliberate, long-standing way
#: ``config/generated/udp_protocol`` is reached from ROS nodes and standalone
#: scripts, so it is out of that rule's scope by design, not by omission.
_SCAN_DIRS = ('sim', 'tests', 'tools', 'controller', 'teensy_link')

#: The one file allowed to mutate sys.path for sim, plus the test suite's own
#: conftest bootstrap (pytest has no entry script to call bootstrap_paths()).
_PATH_MUTATION_ALLOWED = frozenset({
    os.path.join('sim', '_paths.py'),
})


def _python_files():
    for top in _SCAN_DIRS:
        for root, dirs, files in os.walk(os.path.join(_REPO_ROOT, top)):
            dirs[:] = [d for d in dirs
                       if d not in ('__pycache__', 'node_modules', '.git')]
            for name in sorted(files):
                if name.endswith('.py'):
                    path = os.path.join(root, name)
                    yield os.path.relpath(path, _REPO_ROOT), path
    yield 'run_mpc.py', os.path.join(_REPO_ROOT, 'run_mpc.py')


def _bare_sim_imports(tree):
    """Yield (lineno, offending_root) for bare-style sim imports."""
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.level:          # relative import — always fine
                continue
            root = (node.module or '').split('.')[0]
            if root in _SIM_TOP_LEVEL:
                yield node.lineno, root
        elif isinstance(node, ast.Import):
            for alias in node.names:
                root = alias.name.split('.')[0]
                if root in _SIM_TOP_LEVEL:
                    yield node.lineno, root


def test_no_bare_style_sim_imports_anywhere():
    """Every import of sim code goes through the single ``sim.*`` root."""
    offenders = []
    for rel, path in _python_files():
        with open(path, encoding='utf-8') as fh:
            src = fh.read()
        try:
            tree = ast.parse(src, filename=rel)
        except SyntaxError:                                # pragma: no cover
            pytest.fail('%s does not parse' % rel)
        for lineno, root in _bare_sim_imports(tree):
            offenders.append('%s:%d imports bare `%s` (use `sim.%s`)'
                             % (rel, lineno, root, root))
    assert not offenders, (
        'bare-style sim imports create a second module identity:\n  '
        + '\n  '.join(offenders))


def test_sim_library_modules_do_not_mutate_sys_path():
    """Only ``sim/_paths.py`` and entry scripts touch ``sys.path`` under sim/.

    An entry script is identified by its call to ``bootstrap_paths()`` — the
    single blessed mutation.  Anything else re-deriving roots by hand is how
    the four roots drifted apart in the first place (``sim/viz/reference_plot``
    had a fifth, wrong, root; ``tools/probes/juggle_fastcatch`` was missing
    ``config/generated`` and only worked because the plant self-bootstrapped).
    """
    offenders = []
    for rel, path in _python_files():
        if not rel.startswith('sim' + os.sep):
            continue
        if rel in _PATH_MUTATION_ALLOWED:
            continue
        with open(path, encoding='utf-8') as fh:
            src = fh.read()
        if 'sys.path' not in src:
            continue
        tree = ast.parse(src, filename=rel)
        uses_bootstrap = any(
            isinstance(n, ast.Call) and getattr(n.func, 'id', None) == 'bootstrap_paths'
            for n in ast.walk(tree))
        mutations = [
            n.lineno for n in ast.walk(tree)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
            and n.func.attr in ('insert', 'append')
            and isinstance(n.func.value, ast.Attribute)
            and n.func.value.attr == 'path'
            and getattr(n.func.value.value, 'id', None) == 'sys'
        ]
        if not mutations:
            continue
        if uses_bootstrap and len(mutations) == 1:
            continue        # the entry-script preamble: repo root, then bootstrap
        offenders.append('%s: %d sys.path mutation(s) at line(s) %s'
                         % (rel, len(mutations), mutations))
    assert not offenders, (
        'sys.path mutation belongs in sim/_paths.py:\n  ' + '\n  '.join(offenders))


def test_bootstrap_installs_every_import_capability():
    """Every capability the old four-root blocks provided survives.

    ``sim.*`` + ``controller.*`` off the repo root, ``jugglebot.*`` off the
    ros_ws package root, ``hardware_config`` / ``protocol_config`` off
    config/generated.  The fourth (``sim/``) root is retired by design — see
    ``sim/_paths.py`` — so this test also pins that it stays retired.
    """
    import sys

    from sim import _paths

    assert _paths.PATH_ROOTS == (
        _REPO_ROOT,
        os.path.join(_REPO_ROOT, 'ros_ws', 'src', 'jugglebot'),
        os.path.join(_REPO_ROOT, 'config', 'generated'),
    )
    assert _paths.LEGACY_SIM_ROOT not in _paths.PATH_ROOTS

    _paths.bootstrap_paths()
    for root in _paths.PATH_ROOTS:
        assert root in sys.path
    before = list(sys.path)
    _paths.bootstrap_paths()                       # idempotent
    assert sys.path == before


def test_no_dual_module_identity_in_the_test_session():
    """A sim module resolves to exactly one object under pytest.

    ``tests/sim/conftest.py`` deliberately leaves ``sim/`` off ``sys.path``,
    so the legacy spelling is not merely unused — it is unimportable, and a
    bare import that sneaks in fails loudly at collection instead of quietly
    minting a twin.  (Direct script runs are different: Python always puts the
    script's own directory on ``sys.path``, so ``python sim/main.py`` has
    ``sim/`` live no matter what the bootstrap does.  There, the guarantee
    rests on the import-style contract above, not on path hygiene.)
    """
    import importlib
    import sys

    # `tests/sim/__init__.py` makes tests/sim a REGULAR package also named
    # `sim`, and tests/conftest.py puts tests/ on sys.path — so if tests/ ever
    # precedes the repo root, `import sim` short-circuits onto the test
    # directory and every sim.* import in the suite resolves somewhere else.
    # It works today only by pytest's insertion order; pin it so a change to
    # that order fails here rather than 40 confusing collection errors later.
    import sim as _sim_pkg
    assert os.path.dirname(os.path.abspath(_sim_pkg.__file__)) == os.path.join(
        _REPO_ROOT, 'sim'), (
        '`import sim` resolved to %s, not the real package — tests/sim is '
        'shadowing it' % _sim_pkg.__file__)

    for canonical, legacy in (('sim.hand.ballistics', 'hand.ballistics'),
                              ('sim.viz.telemetry', 'viz.telemetry'),
                              ('sim.plant.interface', 'plant.interface')):
        mod = importlib.import_module(canonical)
        assert sys.modules[canonical] is mod
        assert legacy not in sys.modules, (
            '%s is live under BOTH names — dual identity' % canonical)
        with pytest.raises(ImportError):
            importlib.import_module(legacy)
