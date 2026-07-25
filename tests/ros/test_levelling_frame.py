"""Bypass guard for contract C-LEVEL-1 — `ros_ws/docs/levelling_frame.md`.

The gravity-levelling correction must be applied **exactly once per external
pose** and to **no** derived pose. The bug this file exists to prevent is not a
wrong number: it is an *incomplete enumeration* of the surfaces a pose can enter
through. Four of the six external ingest surfaces were uncorrected before
2026-07-25, and the sixth (`timed_target(hold_after=False)`'s neutral return
target) was found only by the Phase 0 audit — a per-surface behavioural test
could never have caught it, because that surface was *never exercised*.

So this file has two halves, and the structural half is the load-bearing one:

**Structural** (`ast.parse` only — the sources are read, never imported, so this
half needs no ROS mocking at all): freeze the set of planner entries and the set
of `levelling` call sites as data, keyed on the **pose-bearing arguments** of
each call. A new planner entry, a new pose-bearing argument on an existing entry,
a planner entry in a new module, or a `levelling` application inside a new
function all fail until the author adds a row and *declares* its E/D
classification.

**Behavioural**: with a non-identity correction installed, drive each surface and
assert on the **installed plan** — never on a message field or a log string —
because the placement decision's whole justification is *plan == emitted ==
gated*, and count the applications per row, because a final-rotvec equality check
cannot distinguish "corrected once" from "corrected, re-corrected, and
accidentally un-corrected".

Why the manifest key is what it is
----------------------------------
`build_timed(state, target, twist, lead, limits, geom, hold_after=…,
neutral_pose=neutral)` is **one** call node carrying **two** independent external
poses (E4's target and E6's neutral). A key of (enclosing function, *the* target
argument) is byte-identical with and without `neutral_pose=`, so it is
structurally blind to the entire E6 class. `build_catch` accepts `neutral_pose`
too, so a one-line future change to `_plan_and_install_catch` would add a
*seventh* external ingest inside an existing call — under the collapsed key every
assertion here would still pass while the post-catch return parked at plan-frame
`rx = 0`. Hence: pose-bearing arguments, **plural**, per entry.

The manifest also spans more than one module, and the module set is discovered
rather than hard-coded: `follower.py` owns two planner entries and
`mpc_bridge_node.py` owns a `levelling` call site, so an AST pass over
`trajectory_node.py` alone could never equal the thing it freezes.

Never key on line numbers: a line-numbered manifest becomes a maintenance tax and
gets deleted the first time an unrelated edit shifts the file — which would
silently remove the whole guard.
"""

from __future__ import annotations

import ast
import os
from collections import Counter
from typing import NamedTuple

import numpy as np
import pytest

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from jugglebot_interfaces.msg import (
    DynamicTargetCommand,
    MotorStateSingle,
    PlatformPoseCommand,
    RobotState,
)
from jugglebot_interfaces.srv import GoToPose, TimedTarget

import jugglebot.hardware_config as hw
from jugglebot.motion import levelling
from jugglebot.motion.ik_solver import leg_lengths_to_pose, rot_matrix_to_rotvec
from jugglebot.trajectory_node import TrajectoryNode

# The offset published during the 2026-07-25 15:17:48 self-toss session.
_SESSION_OFFSET = (0.013592347421588673, 0.001207157476773584)
_ACTIVATE_REV = list(hw.JB_OP_ACTIVATE_POSITION_REVS)

_PKG_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    'ros_ws', 'src', 'jugglebot', 'jugglebot')


# ══════════════════════════════════════════════════════════════════
# Structural half — the AST manifest
# ══════════════════════════════════════════════════════════════════

# Pose-bearing arguments per planner entry, as (manifest label, keyword name,
# positional index). ``None`` for the index means keyword-only at the call sites
# we care about. Entries with an empty tuple carry no pose at all (they are
# seeded from the live state) — listing them is deliberate: an omitted callee
# would silently drop out of the manifest instead of failing.
#
# ``self._plan_and_install_timed`` / ``_plan_and_install_catch`` are included
# even though they are not `planner` functions, because they are where the node's
# EXTERNAL pose arguments are still visible. Inside them the argument is a bare
# local (``target``, ``neutral``), which carries no evidence of the frame; at the
# call site E6 reads ``self._corrected_neutral_pose()``, so reverting it to the
# uncorrected ``self._neutral_pose`` fires this test immediately.
_POSE_BEARING_ARGS = {
    'planner.build_move': (('target_pose', 'target_pose', 1),),
    'planner.build_follow': (('target_pose', 'target_pose', 1),),
    'planner.build_timed': (('target_pose', 'target_pose', 1),
                            ('neutral_pose', 'neutral_pose', None)),
    'planner.build_catch': (('catch_pose', 'catch_pose', 1),
                            ('neutral_pose', 'neutral_pose', None)),
    'planner.build_return_to_neutral': (('neutral_pose', 'neutral_pose', 1),),
    'planner.build_hold': (),
    'planner.build_graceful_stop': (),
    'self._follower.follow': (('target_pose', 'target_pose', 1),),
    'self._plan_and_install_timed': (('target_pose', None, 0),
                                     ('neutral_pose', 'neutral', None)),
    'self._plan_and_install_catch': (('catch_pose', None, 0),),
}

# Every planner entry in the live ROS package, with its C-LEVEL-1 classification.
#   E = external pose ⇒ corrected exactly once at ingest
#   D = derived (measurement or an existing plan) ⇒ never corrected
# Duplicated rows are intentional where one function makes the same call twice —
# the comparison is a multiset, so losing or gaining a call fires.
_PLANNER_MANIFEST = (
    # file, enclosing scope, callee, pose args, class, note
    ('motion/trajectory/follower.py', 'TargetFollower.follow',
     'planner.build_follow', (('target_pose', 'chase.pose'),), 'D',
     'D8 — the chased pose, derived from the already-corrected E1 target'),
    ('motion/trajectory/follower.py', 'TargetFollower.follow',
     'planner.build_graceful_stop', (), 'D',
     'A2 stop, seeded from the live state'),
    ('trajectory_node.py', 'TrajectoryNode._build_descent_plan',
     'planner.build_follow', (('target_pose', 'target'),), 'D',
     'D3/D4 — target is the FK of the live encoder'),
    ('trajectory_node.py', 'TrajectoryNode._build_descent_plan',
     'planner.build_follow', (('target_pose', 'target'),), 'D',
     'D3/D4 — the decel-then-descend fallback, same measured target'),
    ('trajectory_node.py', 'TrajectoryNode._build_descent_plan',
     'planner.build_graceful_stop', (), 'D', 'in-place decel before the descent'),
    ('trajectory_node.py', 'TrajectoryNode._follower_tick',
     'planner.build_graceful_stop', (), 'D', 'D7 — follower input-loss stop'),
    ('trajectory_node.py', 'TrajectoryNode._follower_tick',
     'self._follower.follow', (('target_pose', 'target_tuple[0]'),), 'E',
     'E1 — already corrected in _on_platform_pose; the follower must not re-correct'),
    ('trajectory_node.py', 'TrajectoryNode._install_graceful_stop',
     'planner.build_graceful_stop', (), 'D', 'D5 — mode-exit / arm-latch-edge stop'),
    ('trajectory_node.py', 'TrajectoryNode._on_dynamic_target',
     'self._plan_and_install_catch', (('catch_pose', 'target'),), 'E',
     'E2 — corrected in _catch_target_from_msg'),
    ('trajectory_node.py', 'TrajectoryNode._plan_and_install_catch',
     'planner.build_catch', (('catch_pose', 'catch_pose'),), 'E',
     'E2 continued. NOTE: adding hold_after=False + neutral_pose here would create '
     'a SEVENTH external ingest — this row is what makes that visible'),
    ('trajectory_node.py', 'TrajectoryNode._plan_and_install_timed',
     'planner.build_timed',
     (('target_pose', 'target'), ('neutral_pose', 'neutral')), 'E',
     'E4 + E6 — ONE call, TWO external poses; both corrected at their call site'),
    ('trajectory_node.py', 'TrajectoryNode._retry_pending_stop',
     'planner.build_graceful_stop', (), 'D', 'D6 — pending-stop retry'),
    ('trajectory_node.py', 'TrajectoryNode._seed_hold_from',
     'planner.build_hold', (), 'D', 'D1 — FK telemetry seed'),
    ('trajectory_node.py', 'TrajectoryNode._svc_go_home',
     'planner.build_return_to_neutral',
     (('neutral_pose', 'self._corrected_neutral_pose()'),), 'E',
     'E5 — the neutral constant, corrected AT USE'),
    ('trajectory_node.py', 'TrajectoryNode._svc_go_to_pose',
     'planner.build_move', (('target_pose', 'target'),), 'E',
     'E3 — corrected in _pose_from_msg'),
    ('trajectory_node.py', 'TrajectoryNode._svc_hold',
     'planner.build_hold', (), 'D', 'D2 — seed only'),
    ('trajectory_node.py', 'TrajectoryNode._svc_timed_target',
     'self._plan_and_install_timed',
     (('target_pose', 'target'),
      ('neutral_pose', 'None if hold_after else self._corrected_neutral_pose()')),
     'E', 'E4 + E6 at the surface — the neutral is corrected AT USE, never at '
          'construction (a stored corrected constant double-applies on the next '
          'offset)'),
)

# Every call into `jugglebot.motion.levelling` from the live ROS package.
# ``correct_pose`` / ``apply_gravity_correction`` are APPLICATIONS and may appear
# only in an E-row ingest handler. ``correction_from_offset`` /
# ``identity_correction`` only STORE the correction.
_LEVELLING_MANIFEST = (
    ('mpc_bridge_node.py', 'MpcBridgeNode.__init__',
     'levelling.identity_correction', 'store'),
    ('mpc_bridge_node.py', 'MpcBridgeNode._on_gravity_offset',
     'levelling.correction_from_offset', 'store'),
    ('mpc_bridge_node.py', 'MpcBridgeNode._on_platform_pose',
     'levelling.correct_pose', 'apply:B1'),
    ('trajectory_node.py', 'TrajectoryNode.__init__',
     'levelling.identity_correction', 'store'),
    ('trajectory_node.py', 'TrajectoryNode._catch_target_from_msg',
     'levelling.correct_pose', 'apply:E2'),
    ('trajectory_node.py', 'TrajectoryNode._corrected_neutral_pose',
     'levelling.correct_pose', 'apply:E5+E6'),
    ('trajectory_node.py', 'TrajectoryNode._on_gravity_offset',
     'levelling.correction_from_offset', 'store'),
    ('trajectory_node.py', 'TrajectoryNode._on_platform_pose',
     'levelling.correct_pose', 'apply:E1'),
    ('trajectory_node.py', 'TrajectoryNode._pose_from_msg',
     'levelling.correct_pose', 'apply:E3+E4'),
)

_APPLY_FUNCS = frozenset(
    {'levelling.correct_pose', 'levelling.apply_gravity_correction'})

# Modules whose members must be reached by ATTRIBUTE access, never bound directly
# by `from <module> import name`. Both AST manifests key on the dotted source name
# (`planner.build_*`, `levelling.*`), and the behavioural counting fixture patches
# `levelling.correct_pose` on the module object — a direct binding defeats all
# three at once, silently. The live package already uses attribute access
# exclusively, so the allow-list below is empty and costs nothing to hold.
_ATTRIBUTE_ONLY_MODULES = frozenset({
    'jugglebot.motion.levelling',
    'jugglebot.motion.trajectory.planner',
})
_DIRECT_IMPORT_ALLOWED: frozenset = frozenset()

# Node state that feeds a planner entry from a DIFFERENT scope than the one that
# wrote it. `_on_platform_pose` corrects E1 and stores it here; `_follower_tick`
# reads it and calls `self._follower.follow`. So a new ingest can reach the
# planner without making any call this file's manifests can see — it only has to
# assign this attribute. Frozen as (scope, source text) so a second writer fails.
_FOLLOWER_TARGET_WRITERS = (
    ('TrajectoryNode.__init__', 'self._follower_target = None'),
    ('TrajectoryNode._on_control_mode', 'self._follower_target = None'),
    ('TrajectoryNode._seed_hold_from', 'self._follower_target = None'),
    ('TrajectoryNode._on_platform_pose',
     'self._follower_target = (target, time.perf_counter())'),
)


def _dotted_name(node):
    """`self._follower.follow` / `planner.build_move` → the dotted source name."""
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if not isinstance(node, ast.Name):
        return None
    parts.append(node.id)
    return '.'.join(reversed(parts))


class _Collector(ast.NodeVisitor):
    """Collect planner entries and `levelling` call sites from one module."""

    def __init__(self, src: str, rel: str):
        self._src = src
        self._rel = rel
        self._scope: list = []
        self.planner: list = []
        self.levelling: list = []
        self.unknown_planner: list = []
        self.direct_imports: list = []
        self.follower_target_writes: list = []

    def _visit_scope(self, node):
        self._scope.append(node.name)
        self.generic_visit(node)
        self._scope.pop()

    visit_ClassDef = _visit_scope
    visit_FunctionDef = _visit_scope
    visit_AsyncFunctionDef = _visit_scope

    def visit_ImportFrom(self, node):
        if node.module in _ATTRIBUTE_ONLY_MODULES:
            for alias in node.names:
                self.direct_imports.append(
                    (self._rel, node.module, alias.name))
        self.generic_visit(node)

    def visit_Assign(self, node):
        for tgt in node.targets:
            if isinstance(tgt, ast.Attribute) and tgt.attr == '_follower_target':
                seg = ast.get_source_segment(self._src, node)
                self.follower_target_writes.append(
                    ('.'.join(self._scope), ' '.join((seg or '').split())))
        self.generic_visit(node)

    def visit_Call(self, node):
        name = _dotted_name(node.func)
        where = '.'.join(self._scope)
        if name in _POSE_BEARING_ARGS:
            args = []
            for label, kwarg, index in _POSE_BEARING_ARGS[name]:
                seg = None
                if index is not None and len(node.args) > index:
                    seg = ast.get_source_segment(self._src, node.args[index])
                if kwarg is not None:
                    for kw in node.keywords:
                        if kw.arg == kwarg:
                            seg = ast.get_source_segment(self._src, kw.value)
                if seg is not None:
                    args.append((label, ' '.join(seg.split())))
            self.planner.append((self._rel, where, name, tuple(args)))
        elif name is not None and name.startswith('planner.build_'):
            # A planner entry this manifest has never heard of: fail loudly rather
            # than key it with zero pose arguments and quietly under-constrain it.
            self.unknown_planner.append((self._rel, where, name))
        elif name is not None and name.startswith('levelling.'):
            self.levelling.append((self._rel, where, name))
        self.generic_visit(node)


class _Scan(NamedTuple):
    planner: list
    levelling: list
    unknown_planner: list
    direct_imports: list
    follower_target_writes: list


def _walk_package():
    """Parse every live module in the ROS package.

    ``archived/`` is excluded deliberately: it holds two dead copies of the old
    inline transform, neither has a ``console_scripts`` entry and neither is
    imported by any live module. Deleting archived code is a separate concern —
    but counting it as a call site would be simply wrong.
    """
    planner, lev, unknown, direct, follower = [], [], [], [], []
    for root, dirs, files in os.walk(_PKG_DIR):
        dirs[:] = [d for d in dirs if d not in ('__pycache__', 'archived')]
        for fname in sorted(files):
            if not fname.endswith('.py'):
                continue
            path = os.path.join(root, fname)
            with open(path, 'r', encoding='utf-8') as fh:
                src = fh.read()
            collector = _Collector(src, os.path.relpath(path, _PKG_DIR)
                                   .replace(os.sep, '/'))
            collector.visit(ast.parse(src))
            planner += collector.planner
            lev += collector.levelling
            unknown += collector.unknown_planner
            direct += collector.direct_imports
            follower += collector.follower_target_writes
    return _Scan(planner, lev, unknown, direct, follower)


@pytest.fixture(scope='module')
def ast_scan():
    return _walk_package()


def test_no_unknown_planner_entry(ast_scan):
    """A `planner.build_*` this manifest does not know about is a hard failure.

    Otherwise a new planner function would be keyed with an empty pose-argument
    tuple and the guard would under-constrain the very entry it was added for.
    """
    unknown = ast_scan.unknown_planner
    assert unknown == [], (
        "unrecognised planner entry — add it to _POSE_BEARING_ARGS with its "
        f"pose-bearing arguments, then to _PLANNER_MANIFEST: {unknown}")


def test_planner_entry_module_set_is_frozen(ast_scan):
    """Freeze the FILE set, not just the entries.

    An AST pass over `trajectory_node.py` alone would not fire on a planner entry
    added inside `follower.py` (which already owns two), nor on `_svc_go_home`'s
    `build_return_to_neutral` being refactored down into a motion helper.
    """
    planner = ast_scan.planner
    assert {row[0] for row in planner} == {row[0] for row in _PLANNER_MANIFEST}


def test_planner_entry_manifest_is_exhaustive(ast_scan):
    """Every planner entry, keyed on its POSE-BEARING arguments (plural).

    Fails on: a new planner entry; a pose-bearing argument added to an existing
    entry (the E6 class — one call, two external poses); a planner entry in a new
    module; a change to the *source text* of a pose argument, which is what makes
    reverting `self._corrected_neutral_pose()` to `self._neutral_pose` fire here.
    """
    planner = ast_scan.planner
    found = Counter(planner)
    expected = Counter((f, w, c, a) for f, w, c, a, _cls, _note in _PLANNER_MANIFEST)
    assert found == expected, (
        "planner-entry manifest drift — declare the new entry's E/D "
        f"classification in _PLANNER_MANIFEST.\nmissing: {expected - found}\n"
        f"unexpected: {found - expected}")


def test_levelling_call_site_module_set_is_frozen(ast_scan):
    lev = ast_scan.levelling
    assert {row[0] for row in lev} == {row[0] for row in _LEVELLING_MANIFEST}


def test_levelling_call_sites_are_exactly_the_enumerated_ingests(ast_scan):
    """The reverse error: a correction applied inside a DERIVED path.

    That is the mirror bug — an FK-seeded hold already reads the corrected frame,
    so correcting it again doubles the tilt (−0.7788° becomes −1.5576°).
    """
    lev = ast_scan.levelling
    found = Counter(lev)
    expected = Counter((f, w, c) for f, w, c, _kind in _LEVELLING_MANIFEST)
    assert found == expected, (
        "levelling call-site drift.\n"
        f"missing: {expected - found}\nunexpected: {found - expected}")


def test_planner_and_levelling_are_reached_by_attribute_access_only(ast_scan):
    """A direct `from … import name` un-instruments this whole file at once.

    Both manifests above key on the dotted SOURCE name, so a bare `build_move(…)`
    or `correct_pose(…)` matches nothing and is silently dropped — including by
    `test_no_unknown_planner_entry`, whose contract is "a planner entry this
    manifest has never heard of is a hard failure". The behavioural counting
    fixture fails the same way: `monkeypatch.setattr(levelling, 'correct_pose', …)`
    cannot rebind a name already bound into a consumer module's globals, so a
    correction applied through a direct binding counts **zero** applications.

    The two failure modes that then pass green are the two this contract exists to
    prevent: a seventh ingest that omits the correction, and the mirror bug (a
    derived pose corrected a second time, commanding −1.5576° where the platform
    sits at −0.7788°). The live package uses attribute access exclusively, so the
    allow-list is empty; if a direct import is ever genuinely wanted, teach
    `_Collector` the bound name FIRST, then add it here.
    """
    offenders = [row for row in ast_scan.direct_imports
                 if (row[1], row[2]) not in _DIRECT_IMPORT_ALLOWED]
    assert offenders == [], (
        "a direct `from <module> import <name>` defeats both AST manifests and "
        f"the behavioural counting fixture: {offenders}")


def test_follower_target_writers_are_frozen(ast_scan):
    """The one plan-feeding surface a new ingest can reach without any tracked call.

    `_on_platform_pose` corrects E1 and stores it in `self._follower_target`;
    `_follower_tick`, a different scope, reads it and calls the planner. A second
    absolute-pose topic (a GUI pose channel, a re-enabled ZMQ path, a chase topic)
    whose handler assigned this attribute without calling `levelling.correct_pose`
    would add no planner call and no levelling call — so every other assertion in
    this file stays green while the follower commands plan-frame poses and
    `catch/dynamic_target` commands gravity-frame ones. That is the identical "two
    meanings of level in one node" defect, reintroduced under a passing guard.

    Freezing the writer set is the cheap close. It is NOT a general "no new node
    state" guard — see `ros_ws/docs/levelling_frame.md` § "What this guard
    deliberately does not cover".
    """
    assert (Counter(ast_scan.follower_target_writes)
            == Counter(_FOLLOWER_TARGET_WRITERS)), (
        "a new writer of self._follower_target — if it carries an EXTERNAL pose "
        "it must apply levelling.correct_pose and be declared in "
        "_LEVELLING_MANIFEST; then add it here.\n"
        f"found: {ast_scan.follower_target_writes}")


def test_every_application_sits_in_a_declared_ingest_handler():
    """`correct_pose` / `apply_gravity_correction` may appear only in an E/B row."""
    for _f, where, callee, kind in _LEVELLING_MANIFEST:
        if callee in _APPLY_FUNCS:
            assert kind.startswith('apply:'), (
                f"{where} applies the correction but is not declared an ingest")
        else:
            assert kind == 'store', f"{where}: unexpected kind {kind}"


def test_no_node_reimplements_the_transform():
    """No live module may carry a second copy of the sign convention or of the
    old per-node application method.

    Two implementations of a normative transform is how the contract drifts, and
    the composition is not commutative, so a re-derived copy is a *silent* frame
    error rather than a loud one. Both copies that existed before 2026-07-25 are
    named here by their literal text.

    Scope note: this is a cheap belt-and-braces check on the two exact forms that
    were duplicated, not a general "nobody may compose rotations" guard — the AST
    manifests above are the real structural guard. It deliberately does NOT match
    ``teensy_bridge_node._tilt_to_quat``'s ``[-tilt_x, 0, 0]``, which is a
    different transform (the inclinometer reading → ``RobotState.pose_offset_quat``,
    byte-identical to ``can_node``'s) and has nothing to do with the correction.
    """
    offenders = []
    for root, dirs, files in os.walk(_PKG_DIR):
        dirs[:] = [d for d in dirs if d not in ('__pycache__', 'archived')]
        for fname in sorted(files):
            if not fname.endswith('.py'):
                continue
            path = os.path.join(root, fname)
            rel = os.path.relpath(path, _PKG_DIR).replace(os.sep, '/')
            if rel == 'motion/levelling.py':
                continue          # the one implementation
            with open(path, 'r', encoding='utf-8') as fh:
                for lineno, line in enumerate(fh, 1):
                    code = line.split('#')[0]
                    if '-tilt_x, -tilt_y' in code or '_apply_gravity_correction' in code:
                        offenders.append(f"{rel}:{lineno}")
    assert offenders == [], (
        f"a second implementation of the levelling transform: {offenders}")


# ══════════════════════════════════════════════════════════════════
# Behavioural half — drive the surfaces, assert on the PLAN
# ══════════════════════════════════════════════════════════════════


class _CapturePub:
    def __init__(self):
        self.frames = []

    def send(self, msg):
        self.frames.append(msg)

    def close(self):
        pass


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


def _expected_rotvec(target_rotvec=(0.0, 0.0, 0.0)):
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    return levelling.apply_gravity_correction(np.asarray(target_rotvec), R)


def _go_to_pose_req(x=0.0, y=0.0, z=170.0, duration_s=0.0):
    req = GoToPose.Request()
    req.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion())
    req.duration_s = duration_s
    req.lean_gain = 0.0
    return req


def _timed_req(x=0.0, y=0.0, z=178.0, lead_s=2.5, hold_after=True):
    req = TimedTarget.Request()
    req.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion())
    req.velocity_mm_s = Vector3()
    req.lead_time_s = lead_s
    req.hold_after = hold_after
    return req


def _dynamic_target(node, x=0.0, y=0.0, z=170.0, lead_s=1.2):
    import time as _time
    msg = DynamicTargetCommand()
    msg.target_pos = Point(x=x, y=y, z=z)
    msg.target_vel = Vector3()
    msg.target_quat = Quaternion()          # identity orientation on the wire
    msg.arrival_time = _time.perf_counter() + lead_s
    return msg


def _platform_pose(x=0.0, y=0.0, z=170.0, publisher='SPACEMOUSE'):
    from geometry_msgs.msg import PoseStamped
    msg = PlatformPoseCommand()
    msg.pose_stamped = PoseStamped()
    msg.pose_stamped.pose = Pose(position=Point(x=x, y=y, z=z),
                                 orientation=Quaternion())
    msg.publisher = publisher
    return msg


def _arm(node, want: bool):
    req = SetBool.Request()
    req.data = want
    return node._svc_arm_catch(req, SetBool.Response())


def _plan_end_rotvec(node):
    plan = node._active_plan
    return np.asarray(plan.state_at(plan.total_duration)[0])[3:6]


# ── E rows: the correction reaches the installed PLAN ────────────


def test_E3_go_to_pose_identity_lands_corrected_in_the_plan():
    node = _node()
    assert node._svc_go_to_pose(_go_to_pose_req(z=175.0, duration_s=2.0),
                                GoToPose.Response()).accepted is True
    assert np.allclose(_plan_end_rotvec(node), _expected_rotvec(), atol=1e-12)


def test_E4_timed_target_identity_lands_corrected_in_the_plan():
    node = _node()
    assert node._svc_timed_target(_timed_req(z=178.0),
                                  TimedTarget.Response()).accepted is True
    plan = node._active_plan
    # The reach segment's endpoint is the target; hold_after=True keeps it there.
    assert np.allclose(_plan_end_rotvec(node), _expected_rotvec(), atol=1e-12)
    assert plan.total_duration > 0.0


def test_E5_go_home_targets_the_corrected_neutral():
    node = _node()
    resp = node._svc_go_home(Trigger.Request(), Trigger.Response())
    assert resp.success is True
    end = np.asarray(node._active_plan.final_pose)
    assert np.allclose(end[:3], node._neutral_pose[:3], atol=1e-12)
    assert np.allclose(end[3:6], _expected_rotvec(), atol=1e-12)
    # ...and the stored constant is NOT mutated (correct at use, not at
    # construction — a corrected stored constant double-applies on the next offset)
    assert np.allclose(node._neutral_pose, [0.0, 0.0, hw.JB_OP_DEFAULT_ACTIVE_Z_MM,
                                            0.0, 0.0, 0.0], atol=1e-15)


def test_E6_timed_target_hold_after_false_returns_to_the_corrected_neutral():
    """The path Phase 0 found. It has no production client, which is exactly why
    only a structural guard could have caught it."""
    node = _node()
    assert node._svc_timed_target(_timed_req(x=12.0, z=185.0, lead_s=3.0,
                                             hold_after=False),
                                  TimedTarget.Response()).accepted is True
    end = np.asarray(node._active_plan.final_pose)
    assert np.allclose(end[:3], node._neutral_pose[:3], atol=1e-2)
    assert np.allclose(end[3:6], _expected_rotvec(), atol=1e-9)


def test_E1_platform_pose_target_is_corrected():
    node = _node(mode='SPACEMOUSE')
    node._on_platform_pose(_platform_pose(z=170.0))
    target, _stamp = node._follower_target
    assert np.allclose(target[3:6], _expected_rotvec(), atol=1e-12)
    assert np.allclose(target[:3], [0.0, 0.0, 170.0], atol=1e-12)


def test_E2_catch_target_is_corrected_in_the_plan():
    node = _node()
    _arm(node, True)
    node._on_dynamic_target(_dynamic_target(node, z=172.0))
    assert node._active_plan.kind == 'move'
    reach_end = np.asarray(node._active_plan.state_at(
        node._active_plan.segments[0].duration)[0])
    assert np.allclose(reach_end[3:6], _expected_rotvec(), atol=1e-9)


# ── D rows: derived poses are NEVER corrected (the mirror bug) ───


def test_D1_fk_seed_is_not_corrected():
    """C-LEVEL-1's second half, made executable.

    Seed from leg lengths that correspond to an ALREADY-corrected hold; the
    seeded plan's rotvec must equal the FK rotvec exactly. Routing the seed
    through the correction would read −1.5576° where the platform sits at
    −0.7788°.
    """
    node = _node()
    # Legs physically parked at the corrected neutral.
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    corrected = levelling.correct_pose(node._neutral_pose, R)
    from jugglebot.motion.ik_solver import pose_to_leg_lengths, rotvec_to_rot_matrix
    lengths = pose_to_leg_lengths(corrected[:3],
                                  rotvec_to_rot_matrix(corrected[3:6]), node._geom)
    pos_rev = np.asarray(lengths, dtype=float) * np.asarray(node._geom.mm_to_rev)

    node._on_robot_state(_robot_state(pos_rev=pos_rev))
    assert node._seed_hold_from(pos_rev) is True

    ext_mm = pos_rev / np.asarray(node._geom.mm_to_rev)
    fk_pos, fk_rot, _J = leg_lengths_to_pose(ext_mm, node._geom)
    fk_rotvec = rot_matrix_to_rotvec(fk_rot)
    seeded = np.asarray(node._active_plan.state_at(0.0)[0])
    assert np.allclose(seeded[3:6], fk_rotvec, atol=1e-12), \
        "an FK seed was re-corrected — the mirror bug"
    # And it is NOT the double-corrected value.
    doubled = levelling.apply_gravity_correction(fk_rotvec, R)
    assert not np.allclose(seeded[3:6], doubled, atol=1e-6)


def test_D2_hold_service_is_not_corrected():
    node = _node()
    before = np.asarray(node._current_state()[0])[3:6].copy()
    assert node._svc_hold(Trigger.Request(), Trigger.Response()).success is True
    assert np.allclose(np.asarray(node._active_plan.final_pose)[3:6], before,
                       atol=1e-12)


def test_D3_reseed_from_measured_is_not_corrected():
    node = _node()
    measured = node._measured_pose()
    assert measured is not None
    assert node._svc_reseed_from_measured(
        Trigger.Request(), Trigger.Response()).success is True
    assert np.allclose(np.asarray(node._active_plan.final_pose)[3:6],
                       measured[3:6], atol=1e-9)


# ── Application counting, per E/D ROW (not per surface) ──────────


class _CountingCorrect:
    """Wrap `levelling.correct_pose` and count applications."""

    def __init__(self, real):
        self._real = real
        self.count = 0

    def __call__(self, pose, correction):
        self.count += 1
        return self._real(pose, correction)


@pytest.fixture
def counted(monkeypatch):
    counter = _CountingCorrect(levelling.correct_pose)
    monkeypatch.setattr(levelling, 'correct_pose', counter)
    return counter


@pytest.mark.parametrize('surface,rows', [
    ('go_to_pose', 1),          # E3
    ('timed_target_hold', 1),   # E4
    ('timed_target_return', 2),  # E4 + E6 — ONE surface, TWO external poses
    ('go_home', 1),             # E5
    ('platform_pose', 1),       # E1
    ('dynamic_target', 1),      # E2
])
def test_external_surface_applies_the_correction_exactly_once_per_row(
        surface, rows, counted):
    """A final-rotvec equality check cannot distinguish "corrected once" from
    "corrected, re-corrected, and accidentally un-corrected". A count can.

    "Exactly one per surface" is unimplementable here for the same reason the
    manifest key had to widen: `timed_target(hold_after=False)` is one surface
    carrying two independent external poses.
    """
    node = _node(mode='SPACEMOUSE' if surface == 'platform_pose' else 'TRAJECTORY')
    counted.count = 0                       # ignore construction-time calls
    if surface == 'go_to_pose':
        assert node._svc_go_to_pose(_go_to_pose_req(z=175.0, duration_s=2.0),
                                    GoToPose.Response()).accepted is True
    elif surface == 'timed_target_hold':
        assert node._svc_timed_target(_timed_req(),
                                      TimedTarget.Response()).accepted is True
    elif surface == 'timed_target_return':
        assert node._svc_timed_target(
            _timed_req(x=12.0, z=185.0, lead_s=3.0, hold_after=False),
            TimedTarget.Response()).accepted is True
    elif surface == 'go_home':
        assert node._svc_go_home(Trigger.Request(),
                                 Trigger.Response()).success is True
    elif surface == 'platform_pose':
        node._on_platform_pose(_platform_pose())
    elif surface == 'dynamic_target':
        _arm(node, True)
        counted.count = 0                   # arm_catch itself must apply nothing
        node._on_dynamic_target(_dynamic_target(node, z=172.0))
    assert counted.count == rows


@pytest.mark.parametrize('surface', ['hold', 'reseed_from_measured', 'seed',
                                     'arm_catch', 'graceful_stop'])
def test_derived_surface_applies_the_correction_exactly_zero_times(
        surface, counted):
    node = _node()
    counted.count = 0
    if surface == 'hold':
        node._svc_hold(Trigger.Request(), Trigger.Response())
    elif surface == 'reseed_from_measured':
        node._svc_reseed_from_measured(Trigger.Request(), Trigger.Response())
    elif surface == 'seed':
        node._seed_hold_from(_ACTIVATE_REV)
    elif surface == 'arm_catch':
        _arm(node, True)
    elif surface == 'graceful_stop':
        node._install_graceful_stop('test')
    assert counted.count == 0


# ── The regression this whole plan exists to fix ────────────────


def _positioned_then_catch(node):
    """`go_to_pose(identity)` → run it out → `catch/dynamic_target(identity)` at
    the same xyz. Returns (held rotvec before the catch, installed catch plan)."""
    assert node._svc_go_to_pose(_go_to_pose_req(z=170.0, duration_s=1.0),
                                GoToPose.Response()).accepted is True
    node._plan_t0 = node._plan_t0 - (node._active_plan.total_duration + 0.05)
    held = np.asarray(node._current_state()[0])[3:6].copy()
    _arm(node, True)
    node._on_dynamic_target(_dynamic_target(node, x=0.0, y=0.0, z=170.0))
    return held, node._active_plan


def test_positioning_and_catch_land_in_the_same_frame():
    """The 2026-07-25 anomaly, as an executable regression.

    Before the fix, `go_to_pose(identity)` parked the platform at plan-frame
    `rx = 0` while `catch/dynamic_target(identity)` asked for plan-frame
    `rx = −0.7788°`, so `build_catch` planned a min-jerk **reach between two
    frames** — the tilt the operator saw before every toss. After the fix the
    reach's NET tilt displacement is exactly zero: the two surfaces agree.

    The assertion is on the reach segment's net displacement rather than on a
    flat rx trace, because `build_catch` deliberately ramps a tilt-through-seat
    residual whenever the commanded tilt is non-zero — a *separate* effect that
    this contract does not close (see
    ``test_catch_through_seat_still_aims_off_the_plan_frame_tilt``). Net reach
    displacement isolates the frame question exactly, and it is the quantity that
    was −0.7788° on the day.
    """
    node = _node()
    held, plan = _positioned_then_catch(node)
    reach_end = np.asarray(plan.state_at(plan.segments[0].duration)[0])[3:6]
    assert np.allclose(reach_end, held, atol=1e-12), (
        "the catch reach commanded a net tilt displacement of "
        f"{np.degrees(np.linalg.norm(reach_end - held)):.4f}° from the pose the "
        "positioning move parked at — the positioning and catch surfaces are in "
        "different frames, which is the 2026-07-25 anomaly")


def test_the_frame_regression_would_fire_without_the_fix():
    """Prove the assertion above is not vacuous.

    Reproduces the PRE-fix geometry directly through the planner: an uncorrected
    positioning park (`rx = 0`, what `go_to_pose` produced before Phase 2) with
    the corrected catch target. The reach's net tilt displacement is then the
    full correction — exactly what the previous test forbids.
    """
    from jugglebot.motion.trajectory import planner as _planner
    node = _node()
    R = levelling.correction_from_offset(*_SESSION_OFFSET)
    uncorrected_park = np.asarray(node._neutral_pose, dtype=float)
    catch_target = levelling.correct_pose(uncorrected_park, R)

    plan, _report = _planner.build_catch(
        (uncorrected_park, np.zeros(6), np.zeros(6)), catch_target, 1.2,
        node._limits, node._geom,
        settle_hold_s=node._catch_settle_hold_s, hold_after=True)
    reach_end = np.asarray(plan.state_at(plan.segments[0].duration)[0])[3:6]
    net = float(np.linalg.norm(reach_end - uncorrected_park[3:6]))
    assert net == pytest.approx(0.01364585, rel=1e-6), (
        "the pre-fix geometry must show the full correction as a net reach "
        "displacement, or the regression test above proves nothing")


def test_the_reach_excursion_across_the_plan_is_the_arrival_twist_alone():
    """Sample ACROSS the reach, not only at its endpoints — and say what remains.

    The plan requires an across-plan sample twice over ("the 2026-07-25 signature
    was a mid-plan excursion to +2.32°, so an endpoint-only assertion would have
    passed while the platform tilted"). The net-displacement assertion above is
    endpoint-only by construction, so this is its across-plan half.

    What it pins is deliberately NOT "flat": `build_catch` specifies a non-zero
    ARRIVAL TWIST, so even a reach with `p0 == p1` must swing out and come back.
    With those boundary conditions the quintic collapses exactly to
    `v1·T·φ(s)`, `φ = −4s³ + 7s⁴ − 3s⁵`, `|φ|` maximal `16/81` at `s = 2/3` —
    so the excursion is **linear in the catch lead** and this contract cannot
    remove it.

    Two numbers, at the ~1.2 s lead the fixture uses (the reach duration is a
    live-clock lead, so everything below is asserted as a **rate per second of
    lead** — 0.789132 °/s for this offset direction — rather than as a fixed
    excursion):

      * post-fix peak, plan frame:      +0.16817°  (was +0.38397° pre-fix)
      * post-fix peak above the park:   +0.94696°  (was +0.38397° pre-fix)

    The second RISES, and that is not a regression: pre-fix the park itself sat
    0.7788° high, so measuring from it hid part of the swing. Physically the peak
    tilt against gravity falls (the first column plus 0.7788° in each case:
    1.16275° → 0.94696°). This is why `tools/probes/levelling_tilt_bag_check.py`
    reports `peak_above_park` and never gates on it, and why the runbook's
    operator pre-brief says the visible tilt REMAINS. Removing it means changing
    `build_catch`'s arrival twist — `plans/active/catch-reach-degenerate-overshoot.md`.
    """
    from jugglebot.motion.trajectory import planner as _planner
    node = _node()
    held, plan = _positioned_then_catch(node)
    reach = plan.segments[0]
    ts = np.linspace(0.0, reach.duration, 24001)
    rx = np.degrees(np.array(
        [np.asarray(plan.state_at(float(t))[0])[3] for t in ts]))
    held_rx = float(np.degrees(held[0]))

    # Closed form, derived from the planner's own constants rather than pasted:
    # |phi|max = 16/81 at s = 2/3, times the arrival rate's rx component, times
    # the lead. Nothing here depends on the correction MAGNITUDE — only on its
    # direction — which is why the swing does not shrink as the machine is
    # levelled better.
    tdir_x = abs(held[0]) / float(np.hypot(held[0], held[1]))
    slope_deg_per_s = np.degrees((16.0 / 81.0)
                                 * _planner._CATCH_TILT_THROUGH_RATE_RADPS
                                 * tdir_x)
    assert slope_deg_per_s == pytest.approx(0.789132, abs=1e-6)

    # The peak measured from where the platform rests, per second of catch lead.
    assert (float(np.max(rx)) - held_rx) / reach.duration == pytest.approx(
        slope_deg_per_s, rel=1e-4)
    # The excursion is one-sided: the reach never goes BELOW the park it starts
    # and ends at, so the whole swing is the arrival twist's overshoot.
    assert float(np.min(rx)) == pytest.approx(held_rx, abs=1e-9)
    # ...and it peaks at s = 2/3 of the reach, the closed-form phi extremum.
    assert float(ts[int(np.argmax(rx))] / reach.duration) == pytest.approx(
        2.0 / 3.0, abs=1e-3)

    # And the direction of the two effects, against the PRE-fix geometry at the
    # identical lead: the plan-frame peak FALLS, the peak above the park RISES.
    uncorrected = np.asarray(node._neutral_pose, dtype=float)
    pre_plan, _rep = _planner.build_catch(
        (uncorrected, np.zeros(6), np.zeros(6)),
        np.asarray(plan.segments[0].p1, dtype=float), reach.duration,
        node._limits, node._geom, settle_hold_s=node._catch_settle_hold_s)
    pre_rx = np.degrees(np.array(
        [np.asarray(pre_plan.state_at(float(t))[0])[3] for t in ts]))
    assert float(np.max(rx)) < float(np.max(pre_rx))          # plan frame: better
    assert (float(np.max(rx)) - held_rx
            > float(np.max(pre_rx)) - float(np.degrees(uncorrected[3])))


def test_catch_through_seat_still_aims_off_the_plan_frame_tilt():
    """CHARACTERISATION — a defect this contract surfaces but does NOT close.

    `build_catch` reads ``catch_pose[3:5]`` as "the receive tilt" and ramps a
    tilt-through-seat residual along it (`_CATCH_TILT_THROUGH_RATE_RADPS`, decayed
    over `tilt_decay_s`, overshooting by `_CATCH_TILT_OVERSHOOT_FRAC·rate·decay`).
    That premise holds only while the commanded frame IS the gravity frame. Once a
    levelling correction is loaded, a *gravity-level* catch (identity receive tilt
    on the wire) arrives here as a non-zero plan-frame tilt — the correction
    itself — so the through-seat engages and ramps the platform **0.3008° off
    gravity-level exactly at ball contact**, in the correction's own direction.

    This is pre-existing, not introduced by C-LEVEL-1: `catch/dynamic_target` was
    already corrected before 2026-07-25. It is pinned here because it is the
    quantity the 2026-07-25 bag actually recorded:

      * closed form settle = −1.078408° rx, −0.095775° ry
      * bag observed       = −1.0784° rx, −0.0958° ry (flat, release −0.5 s → +2 s)
      * residual vs gravity 0.3008° ⇒ 16.5 mm drift at 3.93 m/s over 0.8 s,
        against the 16 mm tracker-measured catch error

    So the plan's Context table mis-attributes the −1.08° and the 16 mm to the
    frame plumbing. Fixing THIS is what would take the catch error to ~0; it needs
    an operator decision because it changes commanded motion at ball contact on
    every catch, including the shipping reload path.

    The un-levelled 2026-07-25 15:04 baseline is flat for the same reason in
    reverse: with no correction, a level catch target has `tmag == 0`, which
    disables the through-seat entirely.
    """
    from jugglebot.motion.trajectory import planner as _planner
    node = _node()
    held, plan = _positioned_then_catch(node)

    tmag = float(np.hypot(held[0], held[1]))
    tdir = held[:2] / tmag
    overshoot = (_planner._CATCH_TILT_OVERSHOOT_FRAC
                 * _planner._CATCH_TILT_THROUGH_RATE_RADPS * 0.15)
    expected_settle = held[:2] + overshoot * tdir

    final = np.asarray(plan.final_pose)[3:5]
    assert np.allclose(final, expected_settle, atol=1e-12)
    assert np.degrees(final[0]) == pytest.approx(-1.078408, abs=1e-6)
    assert np.degrees(final[1]) == pytest.approx(-0.095775, abs=1e-6)
    # ...and that settle is 0.3008° OFF gravity-level, not on it.
    assert np.degrees(np.linalg.norm(final - held[:2])) == pytest.approx(
        0.300803, abs=1e-5)


def test_without_a_correction_nothing_changes():
    """Identity correction ⇒ every surface behaves exactly as before Phase 2.

    This is what makes the migration safe to land un-levelled, and it is why the
    pre-existing `timed_target(hold_after=False)` neutral-return test stays green.
    """
    node = _node(offset=None)
    assert node._svc_go_home(Trigger.Request(), Trigger.Response()).success is True
    assert np.allclose(np.asarray(node._active_plan.final_pose),
                       node._neutral_pose, atol=1e-12)


def test_in_flight_plan_keeps_its_frame_when_a_new_offset_arrives():
    """C-LEVEL-1's in-flight rule.

    An emitter-side correction would instead step `u0` by the full offset delta
    on the very next 25 ms knot — 2.77 mm / 0.039 rev for this offset, an
    instantaneous ~111 mm/s leg transient that the pump's 0.3 rev step gate would
    NOT catch. Ingest-side placement gives this rule for free: the correction is
    baked into the plan's endpoint at build time and nothing re-reads it.
    """
    node = _node()
    assert node._svc_go_to_pose(_go_to_pose_req(z=175.0, duration_s=2.0),
                                GoToPose.Response()).accepted is True
    end_before = np.asarray(node._active_plan.final_pose).copy()

    node._on_gravity_offset(Float64MultiArray(data=[0.05, -0.03]))
    assert np.array_equal(np.asarray(node._active_plan.final_pose), end_before), \
        "a live plan was re-framed by a new gravity offset"

    # The NEXT install picks the new correction up.
    node._plan_t0 = node._plan_t0 - (node._active_plan.total_duration + 0.05)
    assert node._svc_go_home(Trigger.Request(), Trigger.Response()).success is True
    R_new = levelling.correction_from_offset(0.05, -0.03)
    assert np.allclose(np.asarray(node._active_plan.final_pose)[3:6],
                       levelling.apply_gravity_correction(np.zeros(3), R_new),
                       atol=1e-12)


def test_B1_mpc_bridge_forwards_a_corrected_msgpack_safe_target():
    """`mpc_bridge_node`'s only pose surface, after its duplicate copy was deleted.

    Constructed WITHOUT `__init__` deliberately: `MpcBridgeIPC.__init__` binds a
    ZMQ PUB on :5558, and a test that binds a production socket can collide with
    a live process on the same machine. Only the ingest path is under test here.

    The `float()` conversion is asserted because the payload is msgpack-encoded
    on the wire and `correct_pose` returns a numpy array — a numpy scalar would
    raise at serialisation time, in the bridge, at runtime, with no test between
    here and the robot (this node has no other coverage and is currently dropped
    from the launch).
    """
    from jugglebot.mpc_bridge_node import MpcBridgeNode
    from geometry_msgs.msg import PoseStamped

    sent = []

    class _FakeIPC:
        def send_target(self, msg):
            sent.append(msg)

    node = MpcBridgeNode.__new__(MpcBridgeNode)
    node._current_mode = 'SPACEMOUSE'
    node._ipc = _FakeIPC()
    node._gravity_correction = levelling.correction_from_offset(*_SESSION_OFFSET)

    msg = PlatformPoseCommand()
    msg.pose_stamped = PoseStamped()
    msg.pose_stamped.pose = Pose(position=Point(x=3.0, y=-4.0, z=170.0),
                                 orientation=Quaternion())
    msg.publisher = 'SPACEMOUSE'
    node._on_platform_pose(msg)

    assert len(sent) == 1
    pose = sent[0]['target_pose']
    assert all(type(v) is float for v in pose), \
        "target_pose must be plain floats — msgpack cannot encode numpy scalars"
    assert np.allclose(pose[:3], [3.0, -4.0, 170.0], atol=1e-12)
    assert np.allclose(pose[3:6], _expected_rotvec(), atol=1e-12)


def test_correction_never_moves_the_catch_reach_envelope():
    """Both sides of the envelope test are position-only, so a pure tilt
    correction cannot move the margin. Verified rather than argued, because the
    swing-compensated pre-tilt POSITION that Tier 8b ships is computed
    request-side and passes through `correct_pose` untouched."""
    node = _node()
    _arm(node, True)
    centre = np.asarray(node._catch_envelope_center, dtype=float)
    assert np.allclose(centre, [0.0, 0.0, hw.JB_OP_DEFAULT_ACTIVE_Z_MM], atol=1e-9)

    msg = _dynamic_target(node, x=25.0, y=-12.0, z=175.0)
    target, _twist = node._catch_target_from_msg(msg)
    assert np.array_equal(target[:3], [25.0, -12.0, 175.0])
    node_off = _node(offset=None)
    target_uncorrected, _ = node_off._catch_target_from_msg(msg)
    assert np.array_equal(target[:3], target_uncorrected[:3])
