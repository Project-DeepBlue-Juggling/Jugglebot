"""``teensy_link.protocol``'s re-export list is complete against the generator.

WHY THIS EXISTS
---------------
``teensy_link/protocol.py`` is the stable import path for the generated wire
module.  Its ``from udp_protocol import (...)`` list is **hand-maintained**, and
until 2026-08-16 nothing checked it — so it drifted silently:
``HEARTBEAT_CONE_HEALTH_SHIFT`` had been generated for weeks and was simply
missing from the list, with every test green.  The failure mode is quiet by
construction: a consumer writes ``p.SOMETHING``, gets ``AttributeError`` at
*runtime* in whatever context first needed it, and the generator, the C++ header
and the markdown spec all agree the symbol exists.

WHAT IS ASSERTED, AND WHAT IS DELIBERATELY NOT
----------------------------------------------
Every name a *consumer* has business importing must be on the facade: every
``CONSTANTS`` entry, every ``ENUMS`` name, every ``MESSAGES`` payload class and
its ``<SCREAMING>_SIZE``, and every ``RPC_ARGS`` struct.  Driven off the
generator spec, exactly like ``test_udp_protocol_xlang.py``, so a new wire object
lands here the moment it is specified rather than when someone happens to need
it.

NOT asserted: that the facade is a *mirror*.  The per-message ``*_FMT`` strings
and the per-arg ``ARG_*_SIZE`` constants are struct-packing internals — no caller
should reach for them through this path, and demanding them here would turn a
curated facade into a re-export of ``dir()``.  Tests that genuinely need them
import ``udp_protocol`` directly (see ``test_clap_send.py``).

Pure stdlib + the pure host package.  No ROS 2, no compile, no hardware.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

from teensy_link import protocol as p

_REPO_ROOT = Path(__file__).resolve().parents[2]
_GEN_SCRIPT = _REPO_ROOT / "config" / "generate_udp_protocol.py"


@pytest.fixture(scope="module")
def gen():
    spec = importlib.util.spec_from_file_location("gen_udp_protocol_reexport", _GEN_SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod          # Python 3.8 dataclass-annotation gotcha
    spec.loader.exec_module(mod)
    return mod


def _missing(names) -> list:
    return sorted(n for n in names if not hasattr(p, n))


def test_every_wire_constant_is_re_exported(gen):
    missing = _missing(name for name, *_rest in gen.CONSTANTS)
    assert not missing, (
        f"generated constants absent from teensy_link/protocol.py: {missing}. "
        "Add them to the `from udp_protocol import (...)` list — the list is "
        "hand-maintained and nothing else notices.")


def test_every_enum_is_re_exported(gen):
    missing = _missing(gen.ENUMS.keys())
    assert not missing, f"generated enums absent from teensy_link/protocol.py: {missing}"


def test_every_payload_struct_and_its_size_is_re_exported(gen):
    missing = _missing(m.name for m in gen.MESSAGES)
    assert not missing, f"payload structs absent from teensy_link/protocol.py: {missing}"
    missing_sizes = _missing(f"{gen._screaming(m.name)}_SIZE" for m in gen.MESSAGES)
    assert not missing_sizes, (
        f"payload SIZE constants absent from teensy_link/protocol.py: {missing_sizes}")


def test_every_rpc_arg_struct_is_re_exported(gen):
    missing = _missing(a.name for a in gen.RPC_ARGS)
    assert not missing, f"RPC arg structs absent from teensy_link/protocol.py: {missing}"


def test_the_guard_would_have_caught_the_drift_it_was_written_for(gen):
    """A regression witness, not a tautology.

    ``HEARTBEAT_CONE_HEALTH_SHIFT`` is the symbol that was generated but absent
    from the facade with no test noticing; it is in ``CONSTANTS``, so the first
    test above covers it.  Naming it explicitly means a future reader can see
    what this file was written for without reading the git history.
    """
    assert "HEARTBEAT_CONE_HEALTH_SHIFT" in {n for n, *_r in gen.CONSTANTS}
    assert p.HEARTBEAT_CONE_HEALTH_SHIFT == 4
