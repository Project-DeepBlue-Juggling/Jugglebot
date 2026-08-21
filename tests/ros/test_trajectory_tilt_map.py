"""`trajectory_node`'s half of contract C-LEVEL-2 — `ros_ws/docs/levelling_frame.md`.

Phase 2 of `plans/archived/tilt-calibration-grid.md`: the loader, the
`trajectory/reload_tilt_map` service, the two observability fields, and the
rewiring of the six C-LEVEL-1 ingest sites onto `levelling.correction_for_pose`.

The *enumeration* of those six sites is guarded structurally in
`test_levelling_frame.py` (AST manifest, three kinds: store / build / apply).
This file is the behavioural half — it drives the surfaces and asserts on the
**installed plan**, for the same reason C-LEVEL-1's tests do: the placement
decision's whole justification is *plan == emitted == gated*, so a test that
reads a message field or a log line is not testing what ships.

**The map used here is deliberately steep and asymmetric.** Real residuals are
0.04–0.6°; these reach 0.86° per axis and 1.18° combined at the corner node,
arranged so that no two grid nodes share a residual and `tx` and `ty` have
different gradients. A symmetric or gentle map
would let a lookup keyed on the wrong pose — the neutral instead of the target,
the held pose instead of the request — return a number close enough to pass.
Home node `(0, 0)` is exactly zero, which is the physical property the
acquisition tool asserts at capture time (the home node is the pose `level`
itself measured).

**One test the plan asked for cannot exist as written, and this is where that is
recorded.** Phase 2's brief wanted a keying regression proving the lookup sees
the *pre-correction* pose "so keying on the corrected pose would produce a
detectably different residual". It cannot: C-LEVEL-1 is rotation-only
(`levelling.correct_pose` copies `pose[0:3]` through untouched), so the corrected
and uncorrected poses have **identical x/y** and both key the map to the same
cell. The distinction is unobservable by construction, which is a stronger
guarantee than a test — and the contract already says so in § "Keying" ("Position
is never touched by the correction … so the intent pose's x/y *are* the commanded
x/y"). What IS observable, and what `test_lookup_is_keyed_on_each_target_pose`
and `test_one_call_two_external_poses_get_two_different_residuals` pin instead,
is that the lookup keys on **each external pose's own x/y** rather than on the
platform's held pose or on a single correction hoisted per service call.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import pytest
import yaml

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from jugglebot_interfaces.msg import MotorStateSingle, RobotState
from jugglebot_interfaces.srv import GoToPose, TimedTarget

import jugglebot.hardware_config as hw
from jugglebot.motion import levelling, tilt_map
from jugglebot.trajectory_node import TrajectoryNode

# The C-LEVEL-2 acquisition tool. Imported with the `tests/hardware/` sys.path
# idiom (`tests/motion/test_tilt_cal_grid.py` uses the same one) so the two
# seam tests at the bottom of this file can drive the tool's guards with THIS
# node's real responses — the tool's own unit tests can only assert against a
# literal copy of the marker, which would not notice this node rewording it.
_HW_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'hardware')
if _HW_DIR not in sys.path:
    sys.path.insert(0, _HW_DIR)

import tilt_cal_grid as tcg  # noqa: E402

# The offset published during the 2026-07-25 15:17:48 self-toss session — the
# same capture record `test_levelling_frame.py` pins its E rows against.
_SESSION_OFFSET = (0.013592347421588673, 0.001207157476773584)
_ACTIVATE_REV = list(hw.JB_OP_ACTIVATE_POSITION_REVS)

_X_MM = [-150.0, 0.0, 150.0]
_Y_MM = [-150.0, 0.0, 150.0]
# Indexed [iy][ix]. Every node distinct; the two axes carry different gradients;
# the home node (0, 0) — [1][1] — is exactly zero.
_TX = [[-0.0100, -0.0040, 0.0020],
       [-0.0080, 0.0000, 0.0085],
       [0.0010, 0.0060, 0.0140]]
_TY = [[0.0120, 0.0070, -0.0030],
       [0.0055, 0.0000, -0.0062],
       [-0.0020, -0.0090, -0.0150]]


# ── fixtures / helpers ────────────────────────────────────────


def _map_doc(tx=None, ty=None, date='2026-08-02', version=1):
    return {
        'version': version,
        'captured': {'date': date, 'git_sha': 'abc1234', 'tool': 'unit-test',
                     'uptime_ms_first': 1000, 'uptime_ms_last': 2000,
                     'level_offset_rad': list(_SESSION_OFFSET),
                     'base_condition': 'bench, unshimmed'},
        'grid': {'z_mm': 170.0, 'orientation': 'level',
                 'x_mm': list(_X_MM), 'y_mm': list(_Y_MM)},
        'residual_rad': {'tx': [list(row) for row in (tx or _TX)],
                         'ty': [list(row) for row in (ty or _TY)]},
    }


def _write_map(path, doc):
    path.write_text(yaml.safe_dump(doc))
    return str(path)


class _CapturePub:
    def __init__(self):
        self.frames = []

    def send(self, msg):
        self.frames.append(msg)

    def close(self):
        pass


class _RecordingLogger:
    """`conftest.MockLogger` swallows messages; this one keeps them per level.

    The loader's *level* is load-bearing, not decoration: an absent map is INFO
    (every uncalibrated machine is in that state), a dropped map is WARN, and an
    invalid one is ERROR. Asserting only "it logged something" would let the
    absent case shout at every operator who has not run a capture yet.
    """

    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, msg, **_kw):
        self.infos.append(str(msg))

    def warning(self, msg, **_kw):
        self.warnings.append(str(msg))

    warn = warning

    def error(self, msg, **_kw):
        self.errors.append(str(msg))

    fatal = error

    def debug(self, msg, **_kw):
        pass


def _logger_for(node):
    """Swap a recording logger into the node (MockNode.get_logger → _logger)."""
    rec = _RecordingLogger()
    node._logger = rec
    return rec


def _robot_state(pos_rev=None):
    pos_rev = pos_rev if pos_rev is not None else _ACTIVATE_REV
    rs = RobotState()
    rs.motor_states = [MotorStateSingle(pos_estimate=float(pos_rev[i]))
                       for i in range(6)] + [MotorStateSingle()]
    rs.is_homed = True
    return rs


def _node(mode='TRAJECTORY', offset=_SESSION_OFFSET):
    node = TrajectoryNode(command_pub_factory=_CapturePub, start_emitter=False)
    node._on_robot_state(_robot_state())
    node._on_control_mode(String(data=mode))
    if offset is not None:
        node._on_gravity_offset(Float64MultiArray(data=list(offset)))
    return node


def _mapped_node(monkeypatch, tmp_path, doc=None, name='tilt_calibration.yaml',
                 **kw):
    """A node built with `$JUGGLEBOT_TILT_CAL` pointed at a written map."""
    path = _write_map(tmp_path / name, doc if doc is not None else _map_doc())
    monkeypatch.setenv('JUGGLEBOT_TILT_CAL', path)
    return _node(**kw), path


def _quat(rotvec):
    """`geometry_msgs/Quaternion` for a rotation vector.

    Built with KEYWORDS. `Quaternion`'s positional order is (x, y, z, w) while
    every rotvec→quat helper in this repo returns (w, x, y, z) — splatting one
    into the other silently produces a ~180° rotation, which `go_to_pose` then
    accepts as a perfectly valid (and completely wrong) target.
    """
    w, x, y, z = _rotvec_to_quat(rotvec)
    return Quaternion(w=w, x=x, y=y, z=z)


def _go_to_pose_req(x=0.0, y=0.0, z=170.0, duration_s=2.0, rotvec=(0.0, 0.0, 0.0)):
    req = GoToPose.Request()
    req.pose = Pose(position=Point(x=x, y=y, z=z), orientation=_quat(rotvec))
    req.duration_s = duration_s
    req.lean_gain = 0.0
    return req


def _rotvec_to_quat(rotvec):
    """(w, x, y, z) — the wire's orientation encoding."""
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    axis = np.asarray(rotvec, dtype=float) / theta
    s = float(np.sin(theta / 2.0))
    return (float(np.cos(theta / 2.0)), float(axis[0] * s),
            float(axis[1] * s), float(axis[2] * s))


def _expected_rotvec(x, y, target_rotvec=(0.0, 0.0, 0.0), tilt_map_obj=None,
                     offset=_SESSION_OFFSET):
    """Ground truth: offset (+ residual at (x, y)) → one Rodrigues → compose."""
    tilt_x, tilt_y = float(offset[0]), float(offset[1])
    if tilt_map_obj is not None:
        rx, ry = tilt_map.lookup(tilt_map_obj, x, y)
        tilt_x += rx
        tilt_y += ry
    R = levelling.correction_from_offset(tilt_x, tilt_y)
    return levelling.apply_gravity_correction(np.asarray(target_rotvec), R)


def _run_plan_out(node):
    """Fast-forward past the active plan so the next install is accepted."""
    node._plan_t0 = node._plan_t0 - (node._active_plan.total_duration + 0.05)


# ══════════════════════════════════════════════════════════════
# Loader — happy / absent / invalid
# ══════════════════════════════════════════════════════════════


def test_loader_absent_file_is_soft_identity():
    """No calibration file ⇒ no map, no complaint, exactly C-LEVEL-1 behaviour.

    Absence is the state every machine is in until an operator has run a
    capture, so it must cost nothing: no ERROR, no gate, and a `go_to_pose` that
    lands on the same rotvec it landed on before this phase existed.
    """
    node = _node()                       # conftest pins $JUGGLEBOT_TILT_CAL off
    assert node._tilt_map is None
    assert node._tilt_map_loaded is False
    assert node._tilt_map_version == ''

    assert node._svc_go_to_pose(_go_to_pose_req(x=120.0, y=-120.0),
                                GoToPose.Response()).accepted is True
    assert np.allclose(np.asarray(node._active_plan.final_pose)[3:6],
                       _expected_rotvec(120.0, -120.0), atol=1e-12)


def test_loader_absent_file_logs_info_not_error(monkeypatch, tmp_path):
    """PLAIN absence — no env var, no file — is INFO. It must cost nothing.

    `monkeypatch.delenv` is load-bearing and not tidying. The directory fixture
    pins "no map" by pointing `$JUGGLEBOT_TILT_CAL` at a path that does not
    exist, which is a *different* state (see the test below): the operator named
    a file and is not getting it. Without the delenv this test asserted INFO
    about the override branch while its name claimed the plain one — it could
    never have caught a regression in either.

    Absence is manufactured at the RESOLVER seam — `tilt_map_candidates`, which
    `resolve_tilt_map_path` reaches through a module-global lookup, so patching
    the one attribute pins both. Patching `_SRC_TILT_YAML` alone (what this test
    did until 2026-08-10) is no longer enough, and the way it broke is the
    reason to say so here: `config/tilt_calibration.yaml` is now a committed
    file, `setup.py` installs it whenever it exists, and so every `colcon build`
    populates the THIRD candidate, `share/jugglebot/config/`. The source-tree
    candidate was blanked and the ament one silently answered in its place —
    this became a happy-path test wearing an absence test's name, and the two
    "nothing was logged loudly" assertions above it still passed.

    Scope: the candidate CHAIN is not this test's subject (it belongs to
    `test_loader_uses_the_real_candidate_chain_when_no_override_is_set` and to
    tests/motion/test_tilt_map.py). What this test owns is the loader's log
    LEVEL when resolution comes back empty with no override set.
    """
    absent = str(tmp_path / 'never-captured' / 'tilt_calibration.yaml')
    monkeypatch.delenv('JUGGLEBOT_TILT_CAL', raising=False)
    monkeypatch.setattr(tilt_map, 'tilt_map_candidates',
                        lambda environ=None: (absent,))
    # The pin must actually bite: if resolution still finds something, every
    # assertion below is about the wrong branch.
    assert tilt_map.resolve_tilt_map_path() is None
    node = TrajectoryNode(command_pub_factory=_CapturePub, start_emitter=False)
    # The constructor already ran the loader against a logger we cannot see, so
    # re-run it against one we can — the code path is identical.
    rec = _logger_for(node)
    ok, message = node._load_tilt_map()
    assert ok is True
    assert rec.errors == []
    assert rec.warnings == []
    assert any('no tilt calibration map found' in line for line in rec.infos)
    assert 'no tilt calibration map found' in message
    assert absent in message                 # the INFO names what it tried


def test_loader_warns_when_the_env_override_points_at_nothing(monkeypatch,
                                                              tmp_path):
    """A typo'd `$JUGGLEBOT_TILT_CAL` degrades LOUDLY, naming the path.

    `tilt_map_candidates`' docstring has always promised this degradation is
    loud, and the reasoning is sound — the override is authoritative, so a typo
    silently runs the whole session uncalibrated while the operator believes
    their named map is applied. But the log level was INFO, which at the default
    level is indistinguishable from the uncalibrated machine's normal boot. The
    promise and the behaviour disagreed; this pins the behaviour.

    Plain absence stays INFO (the test above): it is the state every machine is
    in before its first capture and must not shout at anyone.
    """
    missing = str(tmp_path / 'typo.yaml')
    monkeypatch.setenv('JUGGLEBOT_TILT_CAL', missing)
    node = TrajectoryNode(command_pub_factory=_CapturePub, start_emitter=False)
    rec = _logger_for(node)
    ok, message = node._load_tilt_map()
    # Still non-gating: the load SUCCEEDS and the node runs. Loud, not fatal.
    assert ok is True
    assert node._tilt_map is None and node._tilt_map_loaded is False
    assert rec.errors == []
    assert rec.infos == []
    assert len(rec.warnings) == 1
    # The path is what makes the warning actionable — a typo is invisible
    # without seeing the string the operator actually set.
    assert missing in rec.warnings[0]
    assert 'JUGGLEBOT_TILT_CAL' in rec.warnings[0]
    assert missing in message


def test_loader_happy_path_loads_and_versions(monkeypatch, tmp_path):
    node, path = _mapped_node(monkeypatch, tmp_path)
    assert node._tilt_map is not None
    assert node._tilt_map_loaded is True
    assert node._tilt_map.shape == (3, 3)
    assert node._tilt_map_path == path
    # The published version is the map's own identity, hashed over the applied
    # numbers only — so it is reproducible from the document alone.
    assert node._tilt_map_version == tilt_map.map_version(_map_doc())
    assert node._tilt_map_version.startswith('2026-08-02-')


def test_loader_env_override_is_authoritative(monkeypatch, tmp_path):
    """A typo'd `$JUGGLEBOT_TILT_CAL` loads NOTHING — it does not fall through.

    Falling through to the source tree would apply *a different calibration than
    the operator named* while reporting `tilt_map_loaded=True` and a plausible
    version string. C-LEVEL-2 is non-gating, so loading nothing costs only
    precision; loading the wrong map costs a mis-aimed throw with a green status.
    """
    real = _write_map(tmp_path / 'real.yaml', _map_doc())
    monkeypatch.setenv('JUGGLEBOT_TILT_CAL', str(tmp_path / 'typo.yaml'))
    assert tilt_map.resolve_tilt_map_path() is None
    assert tilt_map.tilt_map_candidates() == (str(tmp_path / 'typo.yaml'),)
    node = _node()
    assert node._tilt_map is None and node._tilt_map_loaded is False
    # ...and the file it would have found without the override is right there.
    monkeypatch.setenv('JUGGLEBOT_TILT_CAL', real)
    assert tilt_map.resolve_tilt_map_path() == real


def test_loader_uses_the_real_candidate_chain_when_no_override_is_set(
        monkeypatch, tmp_path):
    """Drive the node through the ACTUAL resolution chain, override removed.

    Every other node-level test here sets `$JUGGLEBOT_TILT_CAL`, and the override
    is authoritative — so `tilt_map_candidates` short-circuits to one entry and
    the source-tree/ament-share walk is never executed through the node at all.
    That gap is how a source-tree candidate that could never exist in the colcon
    install tree passed a full suite: every assertion was about ORDER, and the
    order was right. This test is the join between the two halves.
    """
    monkeypatch.delenv('JUGGLEBOT_TILT_CAL', raising=False)
    path = _write_map(tmp_path / 'tilt_calibration.yaml', _map_doc())
    monkeypatch.setattr(tilt_map, '_SRC_TILT_YAML', path)

    node = _node()
    assert node._tilt_map_loaded is True
    assert node._tilt_map_path == path
    assert node._tilt_map_version == tilt_map.map_version(_map_doc())


@pytest.mark.parametrize('mangle,needle', [
    ('version', 'schema version'),
    ('axis', 'strictly increasing'),
    ('shape', 'indexed [iy][ix]'),
    ('nan', 'non-finite'),
    ('bound', 'sanity bound'),
    ('yaml', 'not valid YAML'),
])
def test_loader_invalid_map_is_refused_whole_and_loudly(
        monkeypatch, tmp_path, mangle, needle):
    """All-or-nothing: any validation failure ⇒ no map, ERROR, flag stays False.

    Six different corruptions, because "the loader rejects a bad map" is only
    worth asserting if it holds for the ones an operator or a half-written file
    actually produces — a truncated write (`yaml`), a stale schema (`version`),
    a transposed grid (`shape`), a failed capture node (`nan`), a units error
    (`bound`) and a hand-edited axis (`axis`).
    """
    doc = _map_doc()
    path = tmp_path / 'tilt_calibration.yaml'
    if mangle == 'version':
        doc['version'] = 2
    elif mangle == 'axis':
        doc['grid']['x_mm'] = [0.0, 0.0, 150.0]
    elif mangle == 'shape':
        doc['residual_rad']['tx'] = [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]]
    elif mangle == 'nan':
        doc['residual_rad']['ty'][2][1] = float('nan')
    elif mangle == 'bound':
        doc['residual_rad']['tx'][0][0] = 0.9
    if mangle == 'yaml':
        path.write_text('version: 1\ngrid: [unclosed\n')
    else:
        _write_map(path, doc)
    monkeypatch.setenv('JUGGLEBOT_TILT_CAL', str(path))

    node = TrajectoryNode(command_pub_factory=_CapturePub, start_emitter=False)
    assert node._tilt_map is None
    assert node._tilt_map_loaded is False
    assert node._tilt_map_version == ''

    rec = _logger_for(node)
    ok, message = node._load_tilt_map()
    assert ok is False
    assert len(rec.errors) == 1
    assert 'REJECTED' in rec.errors[0]
    assert needle in message


def test_an_invalid_map_still_lets_the_node_command_poses(monkeypatch, tmp_path):
    """Non-gating, at the machine: a rejected map degrades to C-LEVEL-1, not to
    a node that will not move."""
    path = _write_map(tmp_path / 'bad.yaml', {'version': 1})
    monkeypatch.setenv('JUGGLEBOT_TILT_CAL', path)
    node = _node()
    assert node._tilt_map_loaded is False
    assert node._gravity_correction_loaded is True
    assert node._svc_go_to_pose(_go_to_pose_req(x=120.0, y=-120.0),
                                GoToPose.Response()).accepted is True
    assert np.allclose(np.asarray(node._active_plan.final_pose)[3:6],
                       _expected_rotvec(120.0, -120.0), atol=1e-12)


# ══════════════════════════════════════════════════════════════
# Reload service
# ══════════════════════════════════════════════════════════════


def test_reload_swaps_the_map_and_reports_the_version(monkeypatch, tmp_path):
    """The Phase-3 tool's auto-apply step: write the file, call reload, read the
    version back out of the response."""
    path = tmp_path / 'tilt_calibration.yaml'
    monkeypatch.setenv('JUGGLEBOT_TILT_CAL', str(path))
    node = _node()
    assert node._tilt_map_loaded is False

    doc = _map_doc()
    _write_map(path, doc)
    resp = node._svc_reload_tilt_map(Trigger.Request(), Trigger.Response())

    assert resp.success is True
    assert node._tilt_map_loaded is True
    assert node._tilt_map_version == tilt_map.map_version(doc)
    assert node._tilt_map_version in resp.message, (
        'the acquisition tool reads the applied version out of this message')
    assert 'grid=3x3' in resp.message


def test_reload_failure_keeps_the_previous_map(monkeypatch, tmp_path):
    """A half-written or corrupt successor must never displace a good map.

    The tool rewrites `config/tilt_calibration.yaml` in place, so the interesting
    failure is a reload that lands on a truncated file. Keeping the previous map
    means the machine keeps throwing with the calibration it was validated with
    instead of silently dropping to offset-only mid-session.
    """
    path = tmp_path / 'tilt_calibration.yaml'
    node, _ = _mapped_node(monkeypatch, tmp_path)
    good_version = node._tilt_map_version
    good_map = node._tilt_map
    assert node._tilt_map_loaded is True

    path.write_text('version: 1\nresidual_rad: {tx: [[nan]]}\n')
    rec = _logger_for(node)
    resp = node._svc_reload_tilt_map(Trigger.Request(), Trigger.Response())

    assert resp.success is False
    assert node._tilt_map is good_map, 'the previous map was displaced'
    assert node._tilt_map_version == good_version
    assert node._tilt_map_loaded is True
    assert good_version in resp.message, 'the response must name what is still applied'
    assert len(rec.errors) == 1

    # ...and the machine still applies the OLD map, not offset-only.
    assert node._svc_go_to_pose(_go_to_pose_req(x=150.0, y=150.0),
                                GoToPose.Response()).accepted is True
    assert np.allclose(np.asarray(node._active_plan.final_pose)[3:6],
                       _expected_rotvec(150.0, 150.0, tilt_map_obj=good_map),
                       atol=1e-12)


def test_reload_with_the_file_removed_unloads_the_map(monkeypatch, tmp_path):
    """Absence ⇒ UNLOAD, and the call reports success.

    This is the Phase-3 tool's `--force-uninstall` path: it moves the map aside
    and reloads precisely to reach `tilt_map_loaded == false` before a capture,
    because a capture run with a map loaded bakes the map into its own successor
    (the double-application class arriving through the data). Keeping a stale
    in-memory map here would defeat that silently, in the one workflow that most
    needs it — so absence is a legitimate outcome of the operation, not a failure
    of it. Reload's contract is "make this node agree with the file".
    """
    path = tmp_path / 'tilt_calibration.yaml'
    node, _ = _mapped_node(monkeypatch, tmp_path)
    assert node._tilt_map_loaded is True

    path.unlink()
    rec = _logger_for(node)
    resp = node._svc_reload_tilt_map(Trigger.Request(), Trigger.Response())

    assert resp.success is True
    assert node._tilt_map is None
    assert node._tilt_map_loaded is False
    assert node._tilt_map_version == ''
    # Dropping a LIVE map is worth a WARN — losing a calibration you had is not
    # the same event as never having had one.
    assert len(rec.warnings) == 1
    assert 'UNLOADED' in rec.warnings[0]

    assert node._svc_go_to_pose(_go_to_pose_req(x=150.0, y=150.0),
                                GoToPose.Response()).accepted is True
    assert np.allclose(np.asarray(node._active_plan.final_pose)[3:6],
                       _expected_rotvec(150.0, 150.0), atol=1e-12)


def test_reload_service_is_registered_as_a_trigger():
    node = _node()
    svc = node._services['trajectory/reload_tilt_map']
    assert svc.srv_type is Trigger


# ══════════════════════════════════════════════════════════════
# Observability
# ══════════════════════════════════════════════════════════════


def test_status_reports_both_tilt_map_states(monkeypatch, tmp_path):
    node = _node()
    node._publish_status()
    status = node.status_pub.published[-1]
    assert status.tilt_map_loaded is False
    assert status.tilt_map_version == '', (
        'no version may be published when no map is loaded (a non-empty '
        'version with gravity_correction_loaded=false is the loaded-but-'
        'dormant state and is legitimate; empty is only for NO map)')

    path = _write_map(tmp_path / 'tilt_calibration.yaml', _map_doc())
    monkeypatch.setenv('JUGGLEBOT_TILT_CAL', path)
    assert node._svc_reload_tilt_map(
        Trigger.Request(), Trigger.Response()).success is True
    node._publish_status()
    status = node.status_pub.published[-1]
    assert status.tilt_map_loaded is True
    assert status.tilt_map_version == tilt_map.map_version(_map_doc())


def test_the_tilt_map_does_not_gate_anything(monkeypatch, tmp_path):
    """C-LEVEL-2 § "why it is not a gate": the map is a refinement, and
    `gravity_correction_loaded` — the flag the toss actually gates on — is
    completely independent of it in both directions."""
    node, _ = _mapped_node(monkeypatch, tmp_path, offset=None)
    node._publish_status()
    status = node.status_pub.published[-1]
    assert status.tilt_map_loaded is True
    assert status.gravity_correction_loaded is False, (
        'a loaded map must never imply the machine knows where gravity is')

    node._on_gravity_offset(Float64MultiArray(data=list(_SESSION_OFFSET)))
    node._publish_status()
    assert node.status_pub.published[-1].gravity_correction_loaded is True
    assert node.status_pub.published[-1].tilt_map_loaded is True


# ══════════════════════════════════════════════════════════════
# Ingest — keying, and exactly one composition
# ══════════════════════════════════════════════════════════════


@pytest.mark.parametrize('x,y', [
    (150.0, 150.0),      # a corner node
    (-150.0, 0.0),       # an edge node
    (0.0, 0.0),          # the home node — residual is exactly zero here
    (75.0, -37.5),       # mid-cell, both axes interpolated
])
def test_go_to_pose_applies_the_residual_at_its_own_xy(monkeypatch, tmp_path, x, y):
    node, _ = _mapped_node(monkeypatch, tmp_path)
    assert node._svc_go_to_pose(_go_to_pose_req(x=x, y=y),
                                GoToPose.Response()).accepted is True
    end = np.asarray(node._active_plan.final_pose)
    assert np.allclose(end[:3], [x, y, 170.0], atol=1e-9), \
        'C-LEVEL-1 is rotation-only; the map must not move a position'
    assert np.allclose(end[3:6],
                       _expected_rotvec(x, y, tilt_map_obj=node._tilt_map),
                       atol=1e-12)


def test_a_loaded_map_is_DORMANT_until_the_node_is_levelled(monkeypatch,
                                                            tmp_path):
    """Map loaded, no `/gravity_offset` yet ⇒ ingest behaves as C-LEVEL-1 unlevelled.

    **This is the state every boot is in** once `config/tilt_calibration.yaml`
    is committed: the map loads in `__init__`, and the offset only arrives when
    the operator runs `level`. Before this was pinned, the map term was composed
    onto `_gravity_offset`'s `(0.0, 0.0)` **placeholder** — and that placeholder
    means "we do not know the tilt", not "the tilt is zero". A residual is
    *defined* as what is left over after a fresh `level` reference is applied
    (C-LEVEL-2 § "What the map is"), so applying one unreferenced commands a
    rotation relative to nothing. It is not a smaller error than no map; it is a
    differently-wrong one, and no contract clause described it.

    The decision is to gate the map on the offset: identity in, identity out —
    exactly what the machine did before this phase existed, and what the rest of
    the stack already assumes an unlevelled node does (the toss's
    `REJECTED_NOT_LEVELLED` keys on `gravity_correction_loaded` alone).
    """
    node, _ = _mapped_node(monkeypatch, tmp_path, offset=None)
    assert node._gravity_correction_loaded is False
    # The map is LOADED and observable — dormant, not unloaded. An operator
    # inspecting trajectory/status must still see which calibration is in place.
    assert node._tilt_map is not None
    assert node._tilt_map_loaded is True
    assert node._tilt_map_version != ''
    assert node._active_tilt_map() is None

    # A node at a corner where the map's residual is large (0.86°/axis): if the
    # map leaked through, this endpoint could not be the identity.
    assert node._svc_go_to_pose(_go_to_pose_req(x=150.0, y=150.0),
                                GoToPose.Response()).accepted is True
    end = np.asarray(node._active_plan.final_pose)
    assert np.allclose(end[3:6], [0.0, 0.0, 0.0], atol=1e-12), \
        'an unlevelled node must command the requested rotation unchanged'


def test_the_dormant_map_starts_applying_the_moment_level_lands(monkeypatch,
                                                                tmp_path):
    """Dormancy is not a latch — the offset arriving is enough, with no reload.

    The pair to the test above, and the half that keeps the gate from being a
    silent permanent disable: an operator who runs `level` after boot must get
    their calibration without knowing that a reload was ever needed.
    """
    node, _ = _mapped_node(monkeypatch, tmp_path, offset=None)
    assert node._active_tilt_map() is None

    node._on_gravity_offset(Float64MultiArray(data=list(_SESSION_OFFSET)))
    assert node._active_tilt_map() is node._tilt_map

    assert node._svc_go_to_pose(_go_to_pose_req(x=150.0, y=150.0),
                                GoToPose.Response()).accepted is True
    end = np.asarray(node._active_plan.final_pose)
    assert np.allclose(end[3:6],
                       _expected_rotvec(150.0, 150.0,
                                        tilt_map_obj=node._tilt_map),
                       atol=1e-12)


def test_dormancy_is_announced_once_not_at_ingest_rate(monkeypatch, tmp_path):
    """A dormant map WARNs once — silence here would be the whole bug again.

    `tilt_map_loaded=true` while the map is not being applied is exactly the
    kind of state this project refuses to leave silent. But the announcement
    lives on the ingest path, so it must be latched: an unlevelled follower
    stream would otherwise log at 40 Hz.
    """
    node, _ = _mapped_node(monkeypatch, tmp_path, offset=None)
    rec = _logger_for(node)
    for _ in range(5):
        node._active_tilt_map()
    assert len(rec.warnings) == 1
    assert 'DORMANT' in rec.warnings[0]
    assert node._tilt_map_version in rec.warnings[0]


def test_loader_warns_when_the_map_was_captured_off_the_active_plane(
        monkeypatch, tmp_path):
    """A map captured at z=200 applied in a z=170 session says so, loudly.

    `z_mm` is provenance: excluded from the version hash by design, and never a
    lookup key — so the residual grid is applied at EVERY height regardless of
    where it was measured. That is fine for the ±few mm an operator's `--z`
    might drift and wrong for a deliberate `--z 200` capture, and the version
    string cannot carry the difference (by design). The load is the only place
    an operator can be told. WARN, never refuse: C-LEVEL-2 is non-gating, and an
    off-plane map is a precision question, not a safety one.
    """
    doc = _map_doc()
    doc['grid']['z_mm'] = 200.0
    node, _ = _mapped_node(monkeypatch, tmp_path, doc=doc)
    rec = _logger_for(node)
    ok, _message = node._load_tilt_map()

    assert ok is True
    assert node._tilt_map_loaded is True, 'non-gating: the map still loads'
    assert len(rec.warnings) == 1
    # BOTH numbers, or the operator cannot tell which way the mismatch runs.
    assert '200.0' in rec.warnings[0]
    assert str(float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)) in rec.warnings[0]


def test_a_map_captured_at_the_active_plane_is_silent(monkeypatch, tmp_path):
    """The common case must not warn, or the warning above gets ignored."""
    node, _ = _mapped_node(monkeypatch, tmp_path)   # _map_doc uses z_mm=170.0
    rec = _logger_for(node)
    node._load_tilt_map()
    assert rec.warnings == []


def test_a_pose_outside_the_hull_clamps_at_ingest(monkeypatch, tmp_path):
    """Outside the calibrated grid the lookup clamps to the nearest hull point.

    Asserted through `_pose_from_msg` rather than through an installed plan
    deliberately: a query far enough outside ±150 mm to make extrapolation
    obvious is also outside the leg workspace, so `go_to_pose` would reject it
    and the test would pass for the wrong reason. The ingest converter is where
    the clamp lives, so that is where it is measured.

    A wrong-signed edge extrapolation aims a throw WORSE than no map at all, and
    the grid's edge is exactly where the field is least linear — the 2026-07-28
    extremity table refutes a 2-gain fit.
    """
    node, _ = _mapped_node(monkeypatch, tmp_path)
    far = node._pose_from_msg(Pose(position=Point(x=900.0, y=-900.0, z=170.0),
                                   orientation=Quaternion()))
    corner = _expected_rotvec(150.0, -150.0, tilt_map_obj=node._tilt_map)
    assert np.allclose(far[3:6], corner, atol=1e-15), \
        'a far query must return the nearest hull NODE, never an extrapolation'
    assert np.allclose(far[:3], [900.0, -900.0, 170.0], atol=1e-12)


def test_lookup_is_keyed_on_each_target_pose_not_the_held_pose(
        monkeypatch, tmp_path):
    """Move to a corner, then request another: the second move's residual is the
    SECOND target's, not the pose the platform is holding.

    A lookup hoisted to "wherever we are now" would be indistinguishable at the
    first move and wrong at every one after it — and it is the natural shape a
    future emitter-side or state-seeded implementation would take.
    """
    node, _ = _mapped_node(monkeypatch, tmp_path)
    assert node._svc_go_to_pose(_go_to_pose_req(x=-150.0, y=-150.0),
                                GoToPose.Response()).accepted is True
    _run_plan_out(node)
    assert node._svc_go_to_pose(_go_to_pose_req(x=150.0, y=150.0),
                                GoToPose.Response()).accepted is True

    end = np.asarray(node._active_plan.final_pose)[3:6]
    assert np.allclose(
        end, _expected_rotvec(150.0, 150.0, tilt_map_obj=node._tilt_map),
        atol=1e-12)
    # The two corners' residuals genuinely differ, or this proves nothing.
    other = _expected_rotvec(-150.0, -150.0, tilt_map_obj=node._tilt_map)
    assert float(np.degrees(np.linalg.norm(end - other))) > 0.5


def test_one_call_two_external_poses_get_two_different_residuals(
        monkeypatch, tmp_path):
    """E4 + E6: `timed_target(hold_after=False)` carries a displaced target AND
    the neutral return, in ONE service call.

    This is the C-LEVEL-2 shape of the E6 class. A correction built once per
    *call* — the obvious optimisation, and what a `self._correction` hoisted out
    of the ingest converters would produce — gives both poses the target's
    residual. The reach would then be aimed correctly and the machine would park
    the platform at home carrying a 150 mm-corner calibration, which is 1.18° off
    gravity at the pose the next throw leaves from.
    """
    node, _ = _mapped_node(monkeypatch, tmp_path)
    req = TimedTarget.Request()
    req.pose = Pose(position=Point(x=150.0, y=150.0, z=178.0),
                    orientation=Quaternion())
    req.velocity_mm_s = Vector3()
    req.lead_time_s = 3.0
    req.hold_after = False
    assert node._svc_timed_target(req, TimedTarget.Response()).accepted is True

    plan = node._active_plan
    reach_end = np.asarray(plan.state_at(plan.segments[0].duration)[0])[3:6]
    final = np.asarray(plan.final_pose)[3:6]

    assert np.allclose(
        reach_end, _expected_rotvec(150.0, 150.0, tilt_map_obj=node._tilt_map),
        atol=1e-9), 'the reach target must carry the residual at (150, 150)'
    assert np.allclose(
        final, _expected_rotvec(0.0, 0.0, tilt_map_obj=node._tilt_map),
        atol=1e-9), 'the neutral return must carry the HOME node residual'
    # At the home node the residual is exactly zero, so the return is the plain
    # C-LEVEL-1 correction — and it differs measurably from the target's.
    assert np.allclose(final, _expected_rotvec(0.0, 0.0), atol=1e-15)
    assert float(np.degrees(np.linalg.norm(final - reach_end))) > 0.4


def test_go_home_uses_the_home_node_residual(monkeypatch, tmp_path):
    """E5. The neutral constant is at (0, 0), where a valid capture's residual is
    ≈ 0 by construction — so `go_home` under a map is byte-identical to
    `go_home` without one, and the stored constant is still not mutated."""
    node, _ = _mapped_node(monkeypatch, tmp_path)
    assert node._svc_go_home(Trigger.Request(), Trigger.Response()).success is True
    end = np.asarray(node._active_plan.final_pose)
    assert np.allclose(end[3:6], _expected_rotvec(0.0, 0.0), atol=1e-15)
    assert np.allclose(node._neutral_pose,
                       [0.0, 0.0, hw.JB_OP_DEFAULT_ACTIVE_Z_MM, 0.0, 0.0, 0.0],
                       atol=1e-15)


def test_platform_pose_follower_target_carries_the_residual(monkeypatch, tmp_path):
    """E1 — the SpaceMouse/GUI stream. The emitter thread reads this tuple, so
    the residual has to be in it before the follower ever sees the pose."""
    from geometry_msgs.msg import PoseStamped
    from jugglebot_interfaces.msg import PlatformPoseCommand

    node, _ = _mapped_node(monkeypatch, tmp_path, mode='SPACEMOUSE')
    msg = PlatformPoseCommand()
    msg.pose_stamped = PoseStamped()
    msg.pose_stamped.pose = Pose(position=Point(x=-150.0, y=150.0, z=170.0),
                                 orientation=Quaternion())
    msg.publisher = 'SPACEMOUSE'
    node._on_platform_pose(msg)

    target, _stamp = node._follower_target
    assert np.allclose(target[:3], [-150.0, 150.0, 170.0], atol=1e-12)
    assert np.allclose(
        target[3:6],
        _expected_rotvec(-150.0, 150.0, tilt_map_obj=node._tilt_map), atol=1e-12)


def test_catch_target_carries_the_residual_and_the_receive_tilt_does_not(
        monkeypatch, tmp_path):
    """E2, and C-CATCH-1's boundary.

    One wire message, two quantities: the commanded pose (which must carry the
    offset AND the residual) and the gravity-referenced receive tilt (which must
    carry NEITHER — it is what the ball is physically doing, and the map is a
    property of the machine's geometry, not of the ball's arrival).
    """
    from jugglebot_interfaces.msg import DynamicTargetCommand
    import time as _time

    node, _ = _mapped_node(monkeypatch, tmp_path)
    wire = np.array([0.03, -0.12])
    msg = DynamicTargetCommand()
    msg.target_pos = Point(x=150.0, y=-150.0, z=172.0)
    msg.target_vel = Vector3()
    msg.target_quat = _quat((wire[0], wire[1], 0.0))
    msg.arrival_time = _time.perf_counter() + 1.2

    target, _twist, receive_tilt = node._catch_target_from_msg(msg)
    assert np.allclose(receive_tilt, wire, atol=1e-12), (
        'the receive tilt is the WIRE orientation — no offset, no residual')
    assert np.allclose(
        target[3:6],
        _expected_rotvec(150.0, -150.0, target_rotvec=(wire[0], wire[1], 0.0),
                         tilt_map_obj=node._tilt_map), atol=1e-12)


def test_tilted_aim_pose_gets_exactly_one_composition(monkeypatch, tmp_path):
    """The double-application regression, at a Tier-8b tilt-aim geometry.

    A displaced, deliberately tilted `go_to_pose` (5.7° aim at the workspace
    corner — the largest tilt the toss path commands) must come out as ONE
    Rodrigues built from `offset + residual`, composed onto the request once.
    The two failure modes this separates are both silent in leg-space telemetry:
    the residual added twice (a `correction_for_pose` at ingest AND a stored map
    correction downstream), and the correction composed twice.
    """
    node, _ = _mapped_node(monkeypatch, tmp_path)
    aim = (0.09, -0.04, 0.0)             # |aim| = 5.63°
    assert node._svc_go_to_pose(
        _go_to_pose_req(x=150.0, y=-150.0, rotvec=aim),
        GoToPose.Response()).accepted is True

    end = np.asarray(node._active_plan.final_pose)[3:6]
    once = _expected_rotvec(150.0, -150.0, target_rotvec=aim,
                            tilt_map_obj=node._tilt_map)
    assert np.allclose(end, once, atol=1e-12)

    # Both wrong answers, and both are close enough to pass a loose tolerance.
    residual = np.asarray(tilt_map.lookup(node._tilt_map, 150.0, -150.0))
    doubled_residual = levelling.apply_gravity_correction(
        np.asarray(aim),
        levelling.correction_from_offset(
            *(np.asarray(_SESSION_OFFSET) + 2.0 * residual)))
    doubled_correction = levelling.apply_gravity_correction(
        once, levelling.correction_from_offset(
            *(np.asarray(_SESSION_OFFSET) + residual)))
    assert not np.allclose(end, doubled_residual, atol=1e-6)
    assert not np.allclose(end, doubled_correction, atol=1e-6)
    # ...and the size of getting it wrong, so this is not a vacuous inequality:
    # applying the residual twice mis-aims by EXACTLY one residual, 0.2066° at
    # this node — 8.7 mm of landing displacement at the 41.9 mm/° conversion for
    # a 0.6 m toss, against a ~30–40 mm cup basin.
    assert float(np.degrees(np.linalg.norm(once - doubled_residual))) == \
        pytest.approx(float(np.degrees(np.linalg.norm(residual))), rel=0.02)
    assert float(np.degrees(np.linalg.norm(residual))) == pytest.approx(
        0.2066, abs=1e-3)


def test_uncorrected_intent_and_corrected_pose_share_their_xy(monkeypatch, tmp_path):
    """Why the "keyed on the pre-correction pose" regression cannot be written as
    a behavioural difference — pinned as a property instead of left as prose.

    `levelling.correct_pose` copies `pose[0:3]` through untouched, so the
    corrected pose's x/y ARE the intent pose's x/y and both key the map to the
    same cell. Keying on the corrected pose is therefore not a *wrong answer*, it
    is a fixed-point iteration that happens to converge in one step. If this
    property ever breaks — if the correction is ever allowed to move a position —
    the keying rule stops being free and C-LEVEL-2 § "Keying" has to be rewritten
    before anything else is.
    """
    node, _ = _mapped_node(monkeypatch, tmp_path)
    intent = np.array([137.0, -88.0, 170.0, 0.09, -0.04, 0.0])
    correction = levelling.correction_for_pose(
        node._gravity_offset, node._tilt_map, intent)
    corrected = levelling.correct_pose(intent, correction)
    assert np.array_equal(corrected[:3], intent[:3])
    assert (tilt_map.lookup(node._tilt_map, corrected[0], corrected[1])
            == tilt_map.lookup(node._tilt_map, intent[0], intent[1]))


# ══════════════════════════════════════════════════════════════
# In-flight rule
# ══════════════════════════════════════════════════════════════


def test_reload_does_not_reframe_a_live_plan(monkeypatch, tmp_path):
    """C-LEVEL-2 inherits C-LEVEL-1's in-flight rule and must not restate it.

    A map that lands mid-plan does not re-frame the executing plan; the next
    install picks it up. This falls out of ingest-side placement — the residual
    is baked into the plan's endpoint at build time and nothing re-reads the map
    for an installed plan. The alternative steps `u0` by the full map delta on
    the next 25 ms knot, below every guard threshold on the wire.
    """
    path = tmp_path / 'tilt_calibration.yaml'
    node, _ = _mapped_node(monkeypatch, tmp_path)
    first_version = node._tilt_map_version

    assert node._svc_go_to_pose(_go_to_pose_req(x=150.0, y=150.0, duration_s=4.0),
                                GoToPose.Response()).accepted is True
    end_before = np.asarray(node._active_plan.final_pose).copy()

    # A visibly different calibration lands mid-flight.
    flipped = [[-v for v in row] for row in _TX]
    _write_map(path, _map_doc(tx=flipped, date='2026-08-03'))
    assert node._svc_reload_tilt_map(
        Trigger.Request(), Trigger.Response()).success is True
    assert node._tilt_map_version != first_version

    assert np.array_equal(np.asarray(node._active_plan.final_pose), end_before), \
        'a live plan was re-framed by a tilt-map reload'

    # The NEXT install picks the new map up.
    _run_plan_out(node)
    assert node._svc_go_to_pose(_go_to_pose_req(x=150.0, y=150.0),
                                GoToPose.Response()).accepted is True
    assert np.allclose(
        np.asarray(node._active_plan.final_pose)[3:6],
        _expected_rotvec(150.0, 150.0, tilt_map_obj=node._tilt_map), atol=1e-12)
    assert not np.allclose(np.asarray(node._active_plan.final_pose)[3:6],
                           end_before[3:6], atol=1e-6)


# ══════════════════════════════════════════════════════════════
# Degrade path — a lookup that raises must not kill a callback
# ══════════════════════════════════════════════════════════════


def test_non_finite_request_degrades_to_offset_only_and_warns_once(
        monkeypatch, tmp_path):
    """`tilt_map.lookup` RAISES on a non-finite query rather than returning NaN.

    A NaN x on `go_to_pose` is an upstream defect, and before this phase it flowed
    through the rotation-only correction untouched and was rejected downstream by
    `feasibility.validate`. The map must not convert that into a dead callback —
    C-LEVEL-2 is non-gating, so the pose degrades to the offset-only correction
    and the failure stays exactly where it was.

    WARN once per loaded map: the fault is a property of the client, so a
    NaN-emitting one would otherwise log at the full ingest rate — the same
    burial the seed gate exists to prevent (4091 ERROR lines in 41 s, 2026-07-15).
    """
    node, _ = _mapped_node(monkeypatch, tmp_path)
    rec = _logger_for(node)

    pose = Pose(position=Point(x=float('nan'), y=0.0, z=170.0),
                orientation=Quaternion())
    out = node._pose_from_msg(pose)                 # must not raise
    assert np.isnan(out[0])
    assert np.allclose(out[3:6], _expected_rotvec(0.0, 0.0), atol=1e-15), (
        'the degraded pose must carry the plain C-LEVEL-1 correction')
    assert len(rec.warnings) == 1
    assert 'gravity offset only' in rec.warnings[0]

    node._pose_from_msg(pose)
    node._pose_from_msg(Pose(position=Point(x=0.0, y=float('inf'), z=170.0),
                             orientation=Quaternion()))
    assert len(rec.warnings) == 1, 'the degrade WARN must not repeat per pose'

    # A reload re-arms the warning: a new map deserves a fresh report.
    assert node._svc_reload_tilt_map(
        Trigger.Request(), Trigger.Response()).success is True
    node._pose_from_msg(pose)
    assert len(rec.warnings) == 2


def test_degrade_path_leaves_the_map_installed(monkeypatch, tmp_path):
    """One bad pose must not unload the calibration for every later pose."""
    node, _ = _mapped_node(monkeypatch, tmp_path)
    node._pose_from_msg(Pose(position=Point(x=float('nan'), y=0.0, z=170.0),
                             orientation=Quaternion()))
    assert node._tilt_map is not None and node._tilt_map_loaded is True
    assert node._svc_go_to_pose(_go_to_pose_req(x=150.0, y=150.0),
                                GoToPose.Response()).accepted is True
    assert np.allclose(
        np.asarray(node._active_plan.final_pose)[3:6],
        _expected_rotvec(150.0, 150.0, tilt_map_obj=node._tilt_map), atol=1e-12)


def test_every_ingest_survives_a_non_finite_pose(monkeypatch, tmp_path):
    """The degrade path is per-site, so assert it per site.

    Each of the four ingest converters wraps its own `correction_for_pose` call
    (that per-site shape is what makes dropping the map at one of six ingests a
    structural-manifest failure in `test_levelling_frame.py`), which means the
    `except` is duplicated four times and a missing one is a crash in a ROS
    callback rather than a test failure. E5/E6 reads the built-in neutral, which
    is finite by construction, so only three are reachable — the fourth is
    covered structurally.
    """
    from geometry_msgs.msg import PoseStamped
    from jugglebot_interfaces.msg import (DynamicTargetCommand,
                                          PlatformPoseCommand)
    import time as _time

    node, _ = _mapped_node(monkeypatch, tmp_path, mode='SPACEMOUSE')

    # E3/E4
    node._pose_from_msg(Pose(position=Point(x=float('nan'), y=0.0, z=170.0),
                             orientation=Quaternion()))
    # E1
    msg = PlatformPoseCommand()
    msg.pose_stamped = PoseStamped()
    msg.pose_stamped.pose = Pose(
        position=Point(x=0.0, y=float('inf'), z=170.0),
        orientation=Quaternion())
    msg.publisher = 'SPACEMOUSE'
    node._on_platform_pose(msg)
    target, _stamp = node._follower_target
    assert np.allclose(target[3:6], _expected_rotvec(0.0, 0.0), atol=1e-15)

    # E2
    dyn = DynamicTargetCommand()
    dyn.target_pos = Point(x=float('nan'), y=0.0, z=170.0)
    dyn.target_vel = Vector3()
    dyn.target_quat = Quaternion()
    dyn.arrival_time = _time.perf_counter() + 1.2
    catch_target, _twist, _seat = node._catch_target_from_msg(dyn)
    assert np.allclose(catch_target[3:6], _expected_rotvec(0.0, 0.0), atol=1e-15)

    # E5/E6 — finite by construction, and still map-corrected.
    neutral = node._corrected_neutral_pose()
    assert np.allclose(neutral[3:6], _expected_rotvec(0.0, 0.0), atol=1e-15)


# ══════════════════════════════════════════════════════════════
# The seam between this node and the C-LEVEL-2 acquisition tool
# ══════════════════════════════════════════════════════════════


def test_the_acquisition_tool_detects_this_nodes_real_disarmed_marker():
    """Anti-drift: the tool's detector is driven by the node's ACTUAL response.

    `tests/hardware/tilt_cal_grid.py` aborts a capture when an accepted
    `go_to_pose` response says the wire is disarmed — because an accepted move
    on a disarmed wire moves nothing, and a whole capture then writes and
    "verifies" an all-zeros calibration that verifies precisely because the
    platform never moved (the 2026-07-15 silent-no-op class).

    The tool's own unit tests assert against a *literal* copy of the marker, so
    they would stay green if this node reworded it — and the tool would go
    silently blind in the one situation it exists to catch. This test is the
    join: the real node produces the real message, and the real detector is
    asked about it. It lives here rather than in `tests/motion/` because only
    this directory can construct a `TrajectoryNode`.
    """
    node = _node()
    assert node._wire_armed is False, \
        'precondition: a node that has seen no /link_status is disarmed'
    response = node._svc_go_to_pose(_go_to_pose_req(x=100.0, y=0.0),
                                    GoToPose.Response())
    assert response.accepted is True, \
        'precondition: the disarmed wire does NOT reject — that is the trap'
    assert tcg.response_reports_disarmed(response.message) is True

    # ...and the armed case must not trip it, or the tool aborts every capture.
    node._on_link_status(DiagnosticStatus(
        values=[KeyValue(key='mpc_active', value='1')]))
    assert node._wire_armed is True
    _run_plan_out(node)
    armed = node._svc_go_to_pose(_go_to_pose_req(x=-100.0, y=0.0),
                                 GoToPose.Response())
    assert armed.accepted is True
    assert tcg.response_reports_disarmed(armed.message) is False


def test_the_tool_reads_the_same_mpc_active_encoding_this_node_does():
    """Both sides key on `/link_status` `mpc_active == '1'`, a STRING.

    The bridge publishes `str(int(bool))`. A tool that compared against `1`,
    `True` or `'true'` would silently never see an armed wire (refusing every
    capture) or never see a disarmed one (the blocking bug). Pinning both
    consumers to one fixture keeps them from drifting apart.
    """
    node = _node()
    for value, expected in (('1', True), ('0', False)):
        node._on_link_status(DiagnosticStatus(
            values=[KeyValue(key='mpc_active', value=value)]))
        assert node._wire_armed is expected
        ok, _ = tcg.wire_armed_verdict({'mpc_active': value,
                                        'fault_state': 'NONE',
                                        'bridge_link': 'UP'})
        assert ok is expected
