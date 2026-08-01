"""The ``controller.teensy_link`` compat shim must alias, never duplicate.

``teensy_link`` moved from ``controller/teensy_link/`` to the repo root on
2026-08-01 (refactor-2026-07 Phase 4). ``controller/teensy_link.py`` keeps the
old import path alive until 2026-09.

The failure this pins is silent and safety-relevant. A re-export shim (``from
teensy_link import *``) would leave the two paths holding two distinct class
objects for ``RpcError``/``RpcTimeout``, so a ``raise`` through one identity
would pass straight through an ``except`` written against the other — the RPC
handler in the can-bridge node stops exactly that escape. Nothing in the
ordinary test suite would notice: both names exist, both are exception
classes, and the mismatch only shows up when a real RPC fails.

Each test re-imports in a **fresh interpreter** because module identity is
process-global state: once one ordering has run, ``sys.modules`` is warm and a
later ordering can no longer take the path that would have minted a twin.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[2]


def _run(body: str) -> None:
    """Execute ``body`` in a fresh interpreter with the repo root on sys.path."""
    src = f"import sys; sys.path.insert(0, {str(_REPO_ROOT)!r})\n{body}"
    proc = subprocess.run([sys.executable, "-c", src],
                          capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, (
        f"subprocess failed:\n--- stdout ---\n{proc.stdout}\n"
        f"--- stderr ---\n{proc.stderr}")


# Ordering matters: whichever path is imported FIRST is the one that could mint
# the twin for the other. Both directions must land on one object.
@pytest.mark.parametrize("first,second", [
    ("controller.teensy_link.rpc", "teensy_link.rpc"),
    ("teensy_link.rpc", "controller.teensy_link.rpc"),
])
def test_rpc_submodule_is_one_module_object(first, second):
    _run(f"""
import importlib
a = importlib.import_module({first!r})
b = importlib.import_module({second!r})
assert a is b, ('module twin: %r is not %r' % (a, b))
""")


@pytest.mark.parametrize("first,second", [
    ("controller.teensy_link.rpc", "teensy_link.rpc"),
    ("teensy_link.rpc", "controller.teensy_link.rpc"),
])
def test_exception_classes_have_one_identity(first, second):
    """``except RpcError`` written against either path must catch the other."""
    _run(f"""
import importlib
a = importlib.import_module({first!r})
b = importlib.import_module({second!r})
for name in ('RpcError', 'RpcTimeout'):
    assert getattr(a, name) is getattr(b, name), name + ' has two identities'

# The behavioural form of the same claim: raise through one, catch the other.
# RpcTimeout subclasses RpcError, so the cross-path catch also proves the
# inheritance edge survives the alias.
try:
    raise a.RpcTimeout(method=0x0051, retries=3, timeout=0.05)
except b.RpcError:
    pass
try:
    raise b.RpcError(method=0x0051, status=4, message='probe')
except a.RpcError:
    pass
""")


def test_package_and_every_submodule_are_aliased():
    """Not just ``rpc`` — every submodule the package ships, enumerated live so
    a newly added module cannot quietly escape the alias loop."""
    _run("""
import importlib, pkgutil
import teensy_link
import controller.teensy_link as shim
assert shim is teensy_link, 'package itself is not aliased'
names = [m.name for m in pkgutil.iter_modules(teensy_link.__path__)]
assert names, 'no submodules found — enumeration is broken, not the alias'
for n in names:
    new = importlib.import_module('teensy_link.' + n)
    old = importlib.import_module('controller.teensy_link.' + n)
    assert new is old, 'submodule twin: ' + n
""")


def test_symbols_reexported_by_the_package_keep_identity():
    """The package-level re-exports (``from teensy_link import RpcError``) are
    the form most call sites actually use."""
    _run("""
from controller.teensy_link import RpcError as old_err, RpcTimeout as old_to
from controller.teensy_link import TeensyLinkClient as old_client
from teensy_link import RpcError as new_err, RpcTimeout as new_to
from teensy_link import TeensyLinkClient as new_client
assert old_err is new_err and old_to is new_to and old_client is new_client
""")


def test_top_level_teensy_link_is_the_repo_root_package():
    """``import teensy_link`` in the test session must reach the REAL package.

    The move claims the top-level name ``teensy_link``, which
    ``tests/teensy_link/__init__.py`` already claims too — that directory is a
    REGULAR package of the same name, and ``tests/conftest.py`` inserts
    ``tests/`` at ``sys.path[0]`` (it is the last insert). Reproduced live: with
    ``sys.path = [<repo>/tests, <repo>]``, ``import teensy_link`` resolves to
    ``tests/teensy_link/__init__.py`` and ``import teensy_link.setpoint_pump``
    raises ``ModuleNotFoundError: No module named 'teensy_link.setpoint_pump'``
    — a message that names the submodule, not the shadowing, so the ~40
    simultaneous collection errors it produces all point at the wrong thing.

    It is green today only because pytest's default prepend import mode puts the
    rootdir back at ``sys.path[0]`` before importing each test module. That is an
    accident of pytest internals, not a decision, so pin it: an import-mode
    change or a conftest reorder fails HERE, once, with this message. Same guard
    ``tests/sim/test_sim_import_style.py`` installs for ``sim``, whose test
    directory collides the same way.
    """
    import teensy_link

    resolved = Path(teensy_link.__file__).resolve().parent
    assert resolved == _REPO_ROOT / "teensy_link", (
        "`import teensy_link` resolved to %s, not the repo-root package — "
        "tests/teensy_link is shadowing it" % teensy_link.__file__)


def test_shim_declares_its_removal_date_and_the_canonical_path():
    """The shim is temporary; keep the deletion trigger and the replacement
    import legible to whoever reads it in September."""
    text = (_REPO_ROOT / "controller" / "teensy_link.py").read_text()
    assert "2026-09" in text, "shim lost its removal date"
    assert "TEMPORARY" in text, "shim no longer announces that it is temporary"
    assert "from teensy_link import" in text, (
        "shim no longer shows the canonical import new code should use")
