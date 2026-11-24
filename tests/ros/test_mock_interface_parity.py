"""BUG-116 slice 1: the mock ROS interface classes in ``conftest.py`` must carry
the same field names (and, where the mock declares one, the same type class) as
the real ``.msg`` / ``.srv`` / ``.action`` definitions in
``ros_ws/src/jugglebot_interfaces/``.

WHY: the mocks are plain dataclasses / classes, so a node test that writes a
field the real message does not have passes silently and fails only on the
robot. The real definitions are parsed straight from the text files, so this
needs no built ROS install.

Scope: ``jugglebot_interfaces`` only (geometry_msgs / std_msgs have no source
text in the repo). Constants (``NAME=value`` lines) are not compared. A mock
field whose type is ``object``/``None``/a nested default is compared by NAME
only. The known drift is pinned in ``KNOWN_DRIFT`` below, so the test passes
today and fails on any NEW drift, and also fails when a listed case is fixed
(a stale allow-list entry must be deleted).

Slice 2 (strict ``__slots__`` / ``__setattr__`` on the mocks) is NOT done here.
"""
from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

import pytest

# Importing conftest's module is not needed: pytest has already loaded it, and it
# registers the mocks in sys.modules unconditionally.
from tests.ros import conftest as _conftest  # noqa: F401  (ensures the mocks are registered)

IFACE = Path(__file__).resolve().parents[2] / 'ros_ws' / 'src' / 'jugglebot_interfaces'

# (kind, interface, section) -> {"mock_only": {...}, "real_only": {...},
#                                "type_mismatch": {...}}
# Each entry carries a comment saying what it is and whether node code touches it.
KNOWN_DRIFT: dict = {}

_PRIMS = {
    'bool': 'bool', 'byte': 'int', 'char': 'int', 'int8': 'int', 'uint8': 'int',
    'int16': 'int', 'uint16': 'int', 'int32': 'int', 'uint32': 'int',
    'int64': 'int', 'uint64': 'int', 'float32': 'float', 'float64': 'float',
    'string': 'str',
}


def _parse_fields(text: str) -> dict:
    """``name -> category`` ('bool'|'int'|'float'|'str'|'list'|'nested')."""
    out = {}
    for raw in text.splitlines():
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        toks = line.split()
        if len(toks) < 2 or '=' in line.split(None, 2)[1] or (
                len(toks) >= 3 and toks[2].startswith('=')) or '=' in toks[1]:
            continue  # constant definition
        typ, name = toks[0], toks[1]
        if '[' in typ:
            out[name] = 'list'
        else:
            out[name] = _PRIMS.get(typ, 'nested')
    return out


def _split_sections(path: Path) -> list:
    parts = [[]]
    for line in path.read_text().splitlines():
        if line.strip() == '---':
            parts.append([])
        else:
            parts[-1].append(line)
    return ['\n'.join(p) for p in parts]


def _real(kind: str, name: str) -> dict:
    ext = {'msg': 'msg', 'srv': 'srv', 'action': 'action'}[kind]
    secs = _split_sections(IFACE / kind / f'{name}.{ext}')
    names = {'msg': ['msg'], 'srv': ['Request', 'Response'],
             'action': ['Goal', 'Result', 'Feedback']}[kind]
    return {n: _parse_fields(secs[i]) if i < len(secs) else {}
            for i, n in enumerate(names)}


def _cat_of_value(v) -> str | None:
    if isinstance(v, bool):
        return 'bool'
    if isinstance(v, int):
        return 'int'
    if isinstance(v, float):
        return 'float'
    if isinstance(v, str):
        return 'str'
    if isinstance(v, (list, tuple)):
        return 'list'
    return None  # None / object / nested: name-only


_ANN = {'bool': 'bool', 'int': 'int', 'float': 'float', 'str': 'str', 'list': 'list'}


def _mock_fields(cls) -> dict:
    """``name -> category or None`` for a mock message/section class."""
    if dataclasses.is_dataclass(cls):
        out = {}
        for f in dataclasses.fields(cls):
            # conftest has no `from __future__ import annotations`, so f.type is
            # the class; a string annotation (forward ref) is handled too.
            t = f.type if isinstance(f.type, str) else getattr(f.type, '__name__', '')
            t = t.strip("'\"")
            out[f.name] = _ANN.get(t)
        return out
    inst = cls()
    return {k: _cat_of_value(v) for k, v in vars(inst).items()}


def _mock_sections(kind: str, mock) -> dict:
    if kind == 'msg':
        return {'msg': _mock_fields(mock)}
    names = ['Request', 'Response'] if kind == 'srv' else ['Goal', 'Result', 'Feedback']
    return {n: _mock_fields(getattr(mock, n)) for n in names}


def _cases():
    out = []
    for kind in ('msg', 'srv', 'action'):
        mod = sys.modules[f'jugglebot_interfaces.{kind}']
        for name in sorted(n for n in vars(mod) if not n.startswith('_')):
            out.append((kind, name))
    return out


def _diff(kind: str, name: str) -> dict:
    mock = getattr(sys.modules[f'jugglebot_interfaces.{kind}'], name)
    real = _real(kind, name)
    got = _mock_sections(kind, mock)
    drift = {}
    for sec, rf in real.items():
        mf = got[sec]
        d = {
            'mock_only': sorted(set(mf) - set(rf)),
            'real_only': sorted(set(rf) - set(mf)),
            'type_mismatch': sorted(
                n for n in set(mf) & set(rf)
                if mf[n] is not None and mf[n] != rf[n]
                # float-typed ROS field defaulted with an int literal is fine
                and not (rf[n] == 'float' and mf[n] == 'int')),
        }
        d = {k: v for k, v in d.items() if v}
        if d:
            drift[sec] = d
    return drift


def test_every_mock_has_a_real_definition():
    missing = [(k, n) for k, n in _cases()
               if not (IFACE / k / f'{n}.{k}').is_file()]
    assert missing == KNOWN_DRIFT.get('__no_real_file__', [])


@pytest.mark.parametrize('kind,name', _cases())
def test_mock_matches_real_interface(kind, name):
    if not (IFACE / kind / f'{name}.{kind}').is_file():
        pytest.skip('covered by test_every_mock_has_a_real_definition')
    assert _diff(kind, name) == KNOWN_DRIFT.get((kind, name), {})


# Real interfaces that have NO mock (nothing under tests/ros imports them through
# the mocked module). A new real interface lands here deliberately or gets a mock.
UNMOCKED = {
    ('action', 'LevelPlatform'),
    ('msg', 'Target'),
    ('srv', 'GetRobotGeometry'),
}


def test_unmocked_real_interfaces_are_the_known_set():
    mocked = set(_cases())
    real = {(k, p.stem) for k in ('msg', 'srv', 'action')
            for p in (IFACE / k).glob(f'*.{k}')}
    assert real - mocked == UNMOCKED


def test_parser_sanity():
    """Pin the parser on a message with constants, comments and arrays."""
    f = _real('srv', 'InstallSegment')
    assert 'KIND_THROW' not in f['Request']
    assert f['Request']['site_mm'] == 'list'
    assert f['Request']['carry_zvd'] == 'bool'
    assert f['Response']['margin'] == 'float'
    assert _real('msg', 'BallButlerHeartbeat')['msg']['hand_pos_mm'] == 'float'
