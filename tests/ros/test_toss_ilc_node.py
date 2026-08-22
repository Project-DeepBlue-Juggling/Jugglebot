"""Node seams for the LAYER-3 critical-point ILC correction — Phase 2 of
``plans/active/critical-point-ilc.md``.

The artifact itself is covered in ``tests/motion/test_toss_ilc.py`` (pure, no
ROS). Here we test the seven things only the NODE can be asked, and they are the
phase's acceptance gates:

1. **BYTE-IDENTICAL OFF**, in TWO arms, because one of them cannot cover the
   other. ``jugglebot_operational.toss_ilc_enabled`` ships false, and with a real
   correction loaded and commanding non-zero in every channel (a) the emitted
   toss cycle is bit-for-bit the cycle a node with no artifact at all emits, and
   (b) the aim block is bit-for-bit what the PRE-Phase-2 arithmetic produced.
   Arm (a) catches layer 3 being wired outside the flag gate; only arm (b)
   catches the composition rewrite itself moving a number, because both of (a)'s
   nodes run the same Phase-2 composition. **Both are run with an aim map AND a
   session trim loaded** — without them ``_toss_aim_for_goal`` returns at its
   all-zero early guard and neither arm executes the code it is named after.
2. **ONE apply point**, structurally — the D4 manifest shape, applied to layer 3.
3. **The clamp REFUSES layer 3; it never truncates it** (plan risk 5).
4. **Provenance-mismatch dormancy**: loaded, NOT applied, loud, zero correction —
   including when the provenance verdict cannot be COMPUTED at all, which is a
   mismatch and never an exception on the goal-build path.
5. **A key MISS is exactly zero** and the machine says so.
6. **The apply-seam speed gates still gate** — ``validate_event_vel`` AND
   ``throw_envelope.evaluate`` (contract C-HAND-3, added 2026-08-21 for
   contradiction C2) — and a refusal costs the trim, never the goal.
7. **The record carries what was APPLIED**, post-gate, in both channels.

ROS 2 is mocked by ``tests/ros/conftest.py``; the map fixture and the monkeypatch
seams are ``test_toss_calibration.py``'s, deliberately reused — two copies of "a
node whose calibration resolves to this document" is two places for a fixture to
drift from the loader it stands in for.
"""

from __future__ import annotations

import ast
import dataclasses
import math
import os
import sys

import numpy as np
import pytest
import yaml

import jugglebot.hardware_config as hw
from jugglebot import toss_trim
from jugglebot.motion import toss_cal, toss_ilc
from jugglebot.motion.trajectory import toss_release as tr
from jugglebot.reload_coordinator_node import ReloadCoordinatorNode
from jugglebot.toss_sequencer import TIER_8A

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_toss_calibration import _cal_doc, _TILT_V                # noqa: E402

import jugglebot.reload_coordinator_node as _rcn                   # noqa: E402
_PKG_DIR = os.path.dirname(os.path.abspath(_rcn.__file__))

_POSE = (0.0, 150.0, 170.0)
_FLIGHT = 0.9
#: The cell ``_POSE`` at ``_FLIGHT`` quantises onto — asserted, not assumed, by
#: ``test_the_fixture_pose_is_on_the_cell_it_claims`` below.
_KEY = [0.0, 150.0, 170.0, 0.9]


def _ilc_doc(*, aim_rx=0.0011, aim_ry=-0.0023, dv=-0.10, key=None,
             tilt_version=_TILT_V, cal_version='', **over):
    """A valid artifact commanding a NON-ZERO correction in all three channels.

    All three non-zero deliberately: an OFF-path test that only zeroed the aim
    would pass on a build that had wired the velocity trim outside the gate, and
    vice versa.
    """
    doc = {
        'version': 1,
        'captured': {'date': '2026-08-13', 'tool': 'test'},
        'requires': {'tilt_map_version': tilt_version,
                     'toss_cal_version': cal_version,
                     'estimator_version': toss_ilc.ESTIMATOR_VERSION,
                     'model_config_identity': toss_ilc.model_config_identity()},
        'units': {'aim': 'rad', 'event_vel_trim': 'dimensionless'},
        'key': {'pose_cell_mm': toss_ilc.POSE_CELL_MM,
                'pose_z_cell_mm': toss_ilc.POSE_Z_CELL_MM,
                'flight_time_cell_s': toss_ilc.FLIGHT_TIME_CELL_S},
        'cells': [{'key': list(key or _KEY), 'aim_rx': aim_rx,
                   'aim_ry': aim_ry, 'event_vel_trim': dv}],
    }
    doc.update(over)
    return doc


def _node(monkeypatch, tmp_path, *, ilc_doc=None, cal_doc=None, enabled=False,
          live_tilt=_TILT_V):
    """A node whose layer-1 and layer-3 artifacts resolve to the given documents.

    Both are resolved through the production resolvers (monkeypatched at the
    module seam, exactly as ``test_toss_calibration._node_with_map`` does), so
    the node's own load path runs rather than a hand-injected object.
    """
    monkeypatch.setattr(hw, 'JB_OP_TOSS_ILC_ENABLED', enabled)
    if cal_doc is None:
        monkeypatch.setattr(toss_cal, 'resolve_toss_cal_path', lambda *a: None)
    else:
        path = tmp_path / 'toss_calibration.yaml'
        path.write_text(yaml.safe_dump(cal_doc, sort_keys=False))
        monkeypatch.setattr(toss_cal, 'resolve_toss_cal_path',
                            lambda *a: str(path))
    monkeypatch.setattr(toss_cal, 'toss_cal_candidates', lambda *a: ('<test>',))
    ilc_path = tmp_path / 'toss_ilc.yaml'
    if ilc_doc is not None:
        ilc_path.write_text(yaml.safe_dump(ilc_doc, sort_keys=False))
    # Resolve exactly as production does — "the first candidate that EXISTS" —
    # over a tmp path, so `_reload_ilc` below exercises the real absent/present
    # transition instead of a lambda that always answers the same way.
    monkeypatch.setattr(toss_ilc, 'resolve_toss_ilc_path',
                        lambda *a: (str(ilc_path) if ilc_path.exists()
                                    else None))
    monkeypatch.setattr(toss_ilc, 'toss_ilc_candidates', lambda *a: ('<test>',))
    node = ReloadCoordinatorNode()
    with node._lock:
        node._tilt_map_loaded = bool(live_tilt)
        node._tilt_map_version = live_tilt
    # __init__ loaded before the tilt version was known; reload so both
    # provenance verdicts are computed against it (exactly what a live
    # /trajectory/status arrival does, minus the message plumbing).
    node._load_toss_cal()
    return node


def _reload_ilc(node, tmp_path, doc):
    """Re-stamp the artifact and reload it through the node's own loader.

    Needed wherever the artifact's ``requires.toss_cal_version`` has to name the
    LIVE aim map, whose version is a hash the fixture cannot know until the map
    has been loaded."""
    (tmp_path / 'toss_ilc.yaml').write_text(yaml.safe_dump(doc, sort_keys=False))
    node._load_toss_ilc()
    return node


def _live_cal_version(node):
    with node._lock:
        return node._toss_cal_version


def _build(node, monkeypatch, pose=_POSE, flight=_FLIGHT, tier=TIER_8A):
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', tier)
    return node._build_toss_cycle(pose, flight, 5.0, 0.9)


def _warnings(node):
    out = []
    node.get_logger().warning = out.append
    return out


def _record(node, outcome='CAUGHT'):
    class _R:
        pass
    result = _R()
    result.outcome = outcome
    result.success = True
    result.catch_error_mm = float('nan')
    result.achieved_flight_s = float('nan')
    result.catch_event_dt_s = float('nan')
    return node._toss_record_fields(result)


class _StubTrim:
    """A session trim that always ESTIMATES the same non-zero aim.

    Non-zero on purpose everywhere it is used below: since C4 (2026-08-21) it
    commands nothing, so the value's job is to prove that the zero in
    ``trim_aim_rad`` is structural rather than an artefact of an empty
    estimator — a stub returning ``(0, 0)`` would make every C4 assertion below
    pass on a build that had never been demoted.
    """

    def __init__(self, rx=math.radians(0.05), ry=math.radians(-0.03)):
        self._aim = (float(rx), float(ry))

    def aim(self):
        return self._aim

    def snapshot(self):
        """The five keys ``_toss_record_fields`` reads off a live trim. Constant,
        so the record's trim columns are identical between the two arms of the
        byte-identity comparison and any difference it reports is layer 3's."""
        return {'n': 4, 'state': 'ACTIVE', 'reason': '',
                'speed_k_v': 1.0, 'tau_ms': 0.0}


def _plain(value):
    """A hashable/comparable plain-Python form of any field value.

    ``==`` on a numpy array returns an ARRAY, which is truthy-ambiguous — a
    fingerprint comparison that hit one would raise rather than fail, or worse,
    pass through an ``all()`` that never saw it. ``NaN`` is mapped to a sentinel
    for the opposite reason: ``nan != nan``, so the record's several
    not-yet-known columns would report as differences on every run and the real
    signal would be buried in them. Two ``NaN``s ARE the same declaration here —
    "this cycle did not produce this number".
    """
    if isinstance(value, np.ndarray):
        return tuple(_plain(float(v)) for v in value.reshape(-1))
    if isinstance(value, (list, tuple)):
        return tuple(_plain(v) for v in value)
    if isinstance(value, dict):
        return tuple(sorted((str(k), _plain(v)) for k, v in value.items()))
    if isinstance(value, float) and math.isnan(value):
        return 'NaN'
    return value


#: Fingerprint keys that are NOT commands and are NOT deterministic. Kept tiny
#: and each one justified: an unexplained exclusion list is exactly how the
#: acceptance test this replaces lost its teeth, so anything added here has to
#: be something two builds of the SAME node would also disagree on.
#:
#: The first two are clock reads (``perf_counter`` minus the ROS clock, sampled
#: per record), so they move between any two calls. The last two are the node's
#: wall-clock IDENTITY strings — ``session_id`` is stamped at construction and
#: ``toss_uid`` is derived from it — so two nodes built either side of a second
#: boundary disagree by construction. None of the four is a command, and every
#: one of them differs between two runs of the SAME build.
_NON_DETERMINISTIC_KEYS = frozenset({
    'record.perf_minus_ros_s',
    'record.perf_minus_ros_inst_s',
    'record.session_id',
    'record.toss_uid',
})

#: Fingerprint keys that MUST differ between "artifact loaded, flag off" and
#: "no artifact at all". They are the OBSERVABILITY of the artifact's presence —
#: the one thing that is genuinely different about the two nodes — so they are
#: asserted to differ rather than skipped. If they ever stopped differing, the
#: byte-identity comparison would be running against two identical nodes and
#: would prove nothing.
_ARTIFACT_PRESENCE_KEYS = frozenset({'aim.ilc_loaded', 'aim.ilc_version'})


def _fields_of(obj, prefix):
    """Every PUBLIC dataclass field of ``obj``, flattened and prefixed.

    Reflection rather than a hand-written list, deliberately: a hand-written
    list is a subset that silently stops covering the thing it names the moment
    a field is added, which is precisely how the acceptance test this replaces
    became vacuous. Private (``_``-prefixed) fields are excluded because the
    sequencer stamps ``perf_counter()`` into several of them at ``start()``, so
    they differ between two runs of the SAME build and would make the
    fingerprint meaningless rather than strict.
    """
    if obj is None:
        return {prefix: None}
    out = {}
    for f in dataclasses.fields(obj):
        if f.name.startswith('_'):
            continue
        out['{}.{}'.format(prefix, f.name)] = _plain(getattr(obj, f.name))
    return out


def _cycle_fingerprint(node, seq):
    """EVERY number one built toss cycle puts in front of the machine.

    The FSM's public goal parameters (which carry ``event_vel_mps`` — the number
    ``_dispatch_toss_throw`` actually sends), BOTH release states (the
    announcement's and the COMMANDED one, which are different objects the moment
    an aim applies), the whole aim block, the POSITIONING target and the whole
    per-toss record. Compared with ``==`` and nothing else: this is the
    byte-identity claim, and ``approx`` would admit exactly the sub-ulp drift the
    claim is about.
    """
    fp = _fields_of(seq, 'seq')
    with node._lock:
        fp.update(_fields_of(node._toss_release_state, 'release'))
        fp.update(_fields_of(node._toss_release_cmd, 'release_cmd'))
        fp['release_cmd_is_release'] = (node._toss_release_cmd
                                        is node._toss_release_state)
        for key, value in sorted(node._toss_aim.items()):
            fp['aim.{}'.format(key)] = _plain(value)
        fp['platform_target'] = _plain(node._toss_platform_target_mm)
        fp['landing_global'] = _plain(node._toss_landing_global_mm)
    for key, value in sorted(_record(node).items()):
        fp['record.{}'.format(key)] = _plain(value)
    return fp


def test_the_fixture_pose_is_on_the_cell_it_claims():
    """Guards every test below from passing vacuously: if ``_POSE``/``_FLIGHT``
    did not quantise onto ``_KEY``, the "applied" tests would silently become
    key-miss tests and still assert zeros in the OFF cases."""
    assert list(toss_ilc.goal_key(_POSE[0], _POSE[1], _POSE[2], _FLIGHT)) == _KEY


# ══════════════════════════════════════════════════════════════════════════════
# 1. BYTE-IDENTICAL OFF
# ══════════════════════════════════════════════════════════════════════════════

def test_the_shipped_flag_is_false_at_the_node():
    assert ReloadCoordinatorNode._toss_ilc_enabled() is False
    assert hw.JB_OP_TOSS_ILC_ENABLED is False


def test_a_disabled_ilc_leaves_the_release_state_untouched(monkeypatch,
                                                           tmp_path):
    """THE Phase-2 acceptance gate, half one. With a valid artifact loaded and
    commanding a correction in all three channels, the flag alone withholds it:
    the COMMANDED release is the same OBJECT as the announcement release, so the
    disabled path costs not one floating-point operation."""
    node = _node(monkeypatch, tmp_path, ilc_doc=_ilc_doc(), enabled=False)
    assert node._toss_ilc is not None, 'the artifact must be LOADED to prove it '\
        'is the FLAG that withholds it, not an empty table'
    _build(node, monkeypatch)
    with node._lock:
        assert node._toss_release_cmd is node._toss_release_state
        aim = dict(node._toss_aim)
    assert aim['ilc_enabled'] is False
    assert aim['ilc_loaded'] is True
    assert aim['ilc_applied'] is False
    assert aim['aim_rad'] == (0.0, 0.0)
    assert aim['ilc_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_vel_trim'] == 0.0
    assert node._publishers['catch/pretilt_hold'].published == []


def _aimed_node(monkeypatch, tmp_path, *, ilc_doc, enabled=False):
    """A node on the branch Phase 2 actually rewrote: a LOADED aim map AND an
    active session trim, so ``_toss_aim_for_goal`` runs its composition arm.

    This fixture is the whole point of the two tests below. With no map and no
    trim the method returns its literal all-zero ``block`` at the early
    ``map_aim == (0,0) and trim_aim == (0,0) and ilc == 0`` guard, so every
    number compared afterwards is a default that no build could have moved — the
    comparison passes on any implementation, including a broken one. The
    arithmetic Phase 2 changed (``base_offset``, ``trim_offset_mm`` re-expressed
    as a difference against it, and ``ilc_offset_mm``) lives past that guard and
    is only reachable with a non-zero map or trim.
    """
    node = _node(monkeypatch, tmp_path,
                 cal_doc=_cal_doc(math.radians(0.2), math.radians(-0.15)),
                 ilc_doc=ilc_doc, enabled=enabled)
    node._params[_rcn._TOSS_TRIM_PARAM] = True
    node._toss_trim = _StubTrim()
    if ilc_doc is not None:
        # The artifact's requires.toss_cal_version has to name the LIVE map,
        # whose version is a hash the fixture cannot know until it has loaded —
        # otherwise the artifact is DORMANT and the test would prove that the
        # dormancy gate works, not that the FLAG withholds.
        requires = dict(ilc_doc['requires'],
                        toss_cal_version=_live_cal_version(node))
        _reload_ilc(node, tmp_path, dict(ilc_doc, requires=requires))
    return node


def test_the_disabled_cycle_is_BIT_IDENTICAL_to_a_node_with_no_artifact(
        monkeypatch, tmp_path):
    """THE Phase-2 acceptance gate, half two — and the one that cannot be argued
    with. Two nodes, one with a loaded artifact whose provenance MATCHES and
    which commands a real correction in all three channels, one with no artifact
    at all. Every number either cycle puts in front of the machine is compared
    with ``==``, not ``approx``, and the comparison is over the WHOLE
    fingerprint (:func:`_cycle_fingerprint`) rather than a hand-picked list.

    Two things make this non-vacuous, and the version this replaced had neither:

    * **the composition path actually runs** (:func:`_aimed_node`) — a map and a
      trim are loaded, so ``_toss_aim_for_goal`` gets past its all-zero early
      return and executes the arithmetic Phase 2 rewrote. The previous fixture
      had no map and no trim, so both arms returned the same literal default
      dict and the assertions could not have failed;
    * **the field set is reflected, not listed** — every public dataclass field
      of the FSM and of both release states, every key of the aim block, every
      key of the record. A hand-written list stops covering the object the moment
      a field is added, silently.

    ``event_vel_mps`` is in there and is the number the hand is dispatched at
    (``_dispatch_toss_throw`` sends ``seq.event_vel_mps``), so it is the one that
    would move first if the velocity trim had been wired outside the flag gate.
    """
    with_art = _aimed_node(monkeypatch, tmp_path, ilc_doc=_ilc_doc(),
                           enabled=False)
    assert with_art._toss_ilc is not None, 'the artifact must be LOADED to ' \
        'prove it is the FLAG that withholds it, not an empty table'
    fp_a = _cycle_fingerprint(with_art, _build(with_art, monkeypatch))
    # The fixture must have reached the branch under test, or this test is the
    # vacuous one it replaces.
    assert fp_a['aim.offset_mm'] != (0.0, 0.0)
    assert fp_a['aim.map_offset_mm'] != (0.0, 0.0)
    # Layer 2 contributes nothing since C4, so `trim_offset_mm` is no longer the
    # witness that the composition arm ran — the MAP's offset is. The trim is
    # still loaded, and its monitor read-out is in the fingerprint, so the two
    # arms are still compared with a live estimator underneath them.
    assert fp_a['aim.trim_offset_mm'] == (0.0, 0.0)
    monkeypatch.undo()

    bare = tmp_path / 'bare'
    bare.mkdir(exist_ok=True)
    without = _aimed_node(monkeypatch, bare, ilc_doc=None, enabled=False)
    assert without._toss_ilc is None
    fp_b = _cycle_fingerprint(without, _build(without, monkeypatch))

    assert set(fp_a) == set(fp_b)
    differing = {k: (fp_a[k], fp_b[k]) for k in fp_a
                 if k not in _NON_DETERMINISTIC_KEYS and fp_a[k] != fp_b[k]}
    # Exactly the artifact-presence observability, and NOTHING that reaches the
    # machine. Equality both ways: a shrinking difference set would mean the two
    # arms had stopped being different nodes.
    assert set(differing) == _ARTIFACT_PRESENCE_KEYS, differing


def _pre_phase2_aim_block(node, catch_pose, flight):
    """The PRE-Phase-2 ``_toss_aim_for_goal``, transcribed verbatim from the
    commit that preceded this phase, evaluated against ``node``'s live state.

    A second copy of production code in a test is normally the wrong shape. It is
    the right one here because this is an ORACLE, and an oracle that called the
    function under test would assert nothing. The claim being checked is *"with
    layer 3 inactive, the numbers this method produces are the numbers it
    produced before layer 3 existed"* — and the only way to check it is to hold
    the old arithmetic beside the new.

    The comparison the other test makes (Phase-2 node with an artifact vs
    Phase-2 node without one) cannot reach this: both arms run the SAME Phase-2
    composition, so a change in that composition moves both fingerprints
    together and cancels. This is the arm that would catch it.
    """
    with node._lock:
        cal = node._toss_cal
        live_tilt = node._tilt_map_version
        version = node._toss_cal_version
        trim = node._toss_trim
    reason = cal.provenance_mismatch(live_tilt) if cal is not None else ''
    block = {
        'aim_rad': (0.0, 0.0),
        'offset_mm': (0.0, 0.0),
        'map_aim_rad': (0.0, 0.0),
        'map_offset_mm': (0.0, 0.0),
        'trim_aim_rad': (0.0, 0.0),
        'trim_offset_mm': (0.0, 0.0),
        'trim_monitor_aim_rad': (0.0, 0.0),
        'trim_authority': toss_trim.AIM_AUTHORITY,
        'clamp_hits': [],
        'loaded': bool(cal is not None),
        'applied': bool(cal is not None and not reason),
        'trim_enabled': False,
        'version': version,
    }
    map_aim = (0.0, 0.0)
    if cal is not None and not reason:
        map_aim = toss_cal.lookup(cal, float(catch_pose[0]),
                                  float(catch_pose[1]))
    # C4 (2026-08-21): layer 2's aim commands nothing. The oracle carries the
    # demotion because the CLAIM it checks is about layer 3 — "with layer 3
    # inactive the numbers are the pre-Phase-2 numbers" — and leaving layer 2's
    # authority in would make it fail for a reason that has nothing to do with
    # layer 3. What it still holds independently, and is the whole point, is the
    # pre-Phase-2 SHAPE: `trim_offset_mm = offset − map_offset` rather than the
    # `base_offset − map_offset` the live code computes.
    trim_aim = (0.0, 0.0)
    enabled = bool(node.get_parameter(_rcn._TOSS_TRIM_PARAM).value)
    block['trim_enabled'] = enabled
    block['trim_authority'] = toss_trim.AIM_AUTHORITY
    if trim is not None:
        monitor = trim.aim()
        block['trim_monitor_aim_rad'] = (float(monitor[0]), float(monitor[1]))
    if map_aim == (0.0, 0.0) and trim_aim == (0.0, 0.0):
        return block
    rx, ry, hits = toss_cal.clamp_total_aim(map_aim[0] + trim_aim[0],
                                            map_aim[1] + trim_aim[1])
    offset = tr.aim_target_offset_mm(rx, ry, float(flight),
                                     float(catch_pose[2]))
    map_offset = tr.aim_target_offset_mm(map_aim[0], map_aim[1], float(flight),
                                         float(catch_pose[2]))
    block['aim_rad'] = (float(rx), float(ry))
    block['offset_mm'] = (float(offset[0]), float(offset[1]))
    block['map_aim_rad'] = (float(map_aim[0]), float(map_aim[1]))
    block['map_offset_mm'] = (float(map_offset[0]), float(map_offset[1]))
    block['trim_aim_rad'] = (float(trim_aim[0]), float(trim_aim[1]))
    block['trim_offset_mm'] = (float(offset[0]) - float(map_offset[0]),
                               float(offset[1]) - float(map_offset[1]))
    block['clamp_hits'] = list(hits)
    return block


def test_the_disabled_cycle_is_BIT_IDENTICAL_to_the_PRE_PHASE_2_arithmetic(
        monkeypatch, tmp_path):
    """THE Phase-2 acceptance gate, half three — the arm the two-node comparison
    structurally cannot cover.

    Phase 2 rewrote the aim composition even for goals layer 3 never touches:
    ``trim_offset_mm`` is now ``base_offset − map_offset`` where it used to be
    ``offset − map_offset``, and a new ``ilc_offset_mm`` is derived from the same
    pair. With layer 3 inactive the code takes ``base_offset = offset`` (the
    literal ``if ilc_aim == (0.0, 0.0)`` branch), so the two expressions are the
    SAME float — but "should be" is the claim, and this is where it is checked
    against the old arithmetic rather than against another copy of the new.

    Run with a map AND a trim loaded so every one of those terms is non-zero, and
    with a valid artifact present and the flag OFF — the shipped state.
    """
    node = _aimed_node(monkeypatch, tmp_path, ilc_doc=_ilc_doc(), enabled=False)
    _build(node, monkeypatch)
    with node._lock:
        live = dict(node._toss_aim)
    expected = _pre_phase2_aim_block(node, _POSE, _FLIGHT)

    # Every key the pre-Phase-2 block carried must be bit-identical. The layer-3
    # keys are additive and are asserted to be the inert values instead — an
    # explicit list rather than "the ones the oracle happens not to have",
    # because the comparison loop above iterates the ORACLE's keys and a new
    # layer-3 key would otherwise be checked by nothing at all.
    for key, want in sorted(expected.items()):
        assert live[key] == want, (key, live[key], want)
    assert live['offset_mm'] != (0.0, 0.0), 'the composition path must have run'
    assert live['map_offset_mm'] != (0.0, 0.0)
    assert live['ilc_aim_rad'] == (0.0, 0.0)
    assert live['ilc_offset_mm'] == (0.0, 0.0)
    assert live['ilc_vel_trim'] == 0.0
    assert live['ilc_applied'] is False
    assert live['ilc_refused'] == ''
    # The C1 session component's keys (2026-08-21). Inert on this path for a
    # reason worth stating: the flag is OFF, and the flag has to withhold the
    # WHOLE layer — a build that gated only the cell lookup would command a
    # learned common mode with the feature disabled.
    assert live['ilc_spatial_aim_rad'] == (0.0, 0.0)
    assert live['ilc_session_aim_rad'] == (0.0, 0.0)
    assert live['ilc_session_applied'] is False
    assert live['ilc_session_n'] == 0
    assert set(live) - set(expected) == {
        'ilc_aim_rad', 'ilc_spatial_aim_rad', 'ilc_session_aim_rad',
        'ilc_session_applied', 'ilc_session_reason', 'ilc_session_n',
        'ilc_offset_mm', 'ilc_vel_trim', 'ilc_enabled', 'ilc_loaded',
        'ilc_applied', 'ilc_version', 'ilc_key', 'ilc_hit', 'ilc_refused',
    }, 'a NEW block key must be added to the oracle or to this inert list'


def test_an_absent_artifact_is_silent(monkeypatch, tmp_path):
    """Absence is the pre-Phase-2 machine AND the Phase-3 A/B's baseline arm
    (``$JUGGLEBOT_TOSS_ILC`` pointed at a path that is not there), so it must
    cost nothing and say nothing."""
    node = _node(monkeypatch, tmp_path, ilc_doc=None, enabled=True)
    warnings = _warnings(node)
    _build(node, monkeypatch)
    with node._lock:
        assert node._toss_release_cmd is node._toss_release_state
        assert node._toss_aim['ilc_loaded'] is False
    assert warnings == []


def test_an_invalid_artifact_loads_nothing_and_never_gates_a_toss(monkeypatch,
                                                                  tmp_path):
    """Layer 3 is a refinement, never a gate: a rejected artifact costs the
    correction, not the throw."""
    bad = _ilc_doc()
    bad['cells'][0]['event_vel_trim'] = -0.9        # past the ILC authority
    node = _node(monkeypatch, tmp_path, ilc_doc=bad, enabled=True)
    assert node._toss_ilc is None
    seq = _build(node, monkeypatch)
    assert seq.tilt_clamp_exceeded is False
    with node._lock:
        assert node._toss_release_cmd is node._toss_release_state
        assert node._toss_aim['ilc_vel_trim'] == 0.0


# ══════════════════════════════════════════════════════════════════════════════
# 2. ONE apply point (the D4 manifest, applied to layer 3)
# ══════════════════════════════════════════════════════════════════════════════

def _calls_in_package(targets):
    found = []
    for root, dirs, files in os.walk(_PKG_DIR):
        dirs[:] = [d for d in dirs if d not in ('__pycache__', 'archived')]
        for fname in sorted(files):
            if not fname.endswith('.py'):
                continue
            with open(os.path.join(root, fname), encoding='utf-8') as handle:
                tree = ast.parse(handle.read(), filename=fname)
            for node in ast.walk(tree):
                if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                for child in ast.walk(node):
                    if isinstance(child, ast.Call) and \
                            _dotted(child.func) in targets:
                        found.append((fname, node.name, _dotted(child.func)))
    return sorted(set(found))


def _dotted(node):
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
    return '.'.join(reversed(parts))


def test_the_ilc_artifact_is_looked_up_in_exactly_one_scope():
    """Design constraint 2, structurally. The correction is evaluated ONCE PER
    TOSS GOAL and nowhere else — not in the 40 Hz emitter, not per Hermite knot,
    not in ``catch_coordinator``, never on the reload path.

    Why structural and not behavioural: a second lookup site is a perfectly
    working program. Every behavioural assertion about "is the correction
    applied" still passes; the only symptom is that the pose the platform is
    commanded to and the pose the record says it was commanded to drift apart —
    which is the plan's risk 5 arriving through the back door.
    """
    sites = _calls_in_package({'toss_ilc.lookup'})
    assert sites == [('reload_coordinator_node.py', '_toss_aim_for_goal',
                      'toss_ilc.lookup')], sites


def test_the_one_lookup_scope_is_reached_from_exactly_one_place():
    callers = _calls_in_package({'self._toss_aim_for_goal'})
    assert callers == [('reload_coordinator_node.py', '_build_toss_cycle',
                        'self._toss_aim_for_goal')], callers


def test_nothing_else_in_the_package_imports_the_ilc_loader():
    """One owner. ``reload_coordinator_node`` holds layer 3 for the same reason
    it holds layer 1: both rewrite a GOAL. A second importer inside the package
    is a second owner, and two owners of one calibration is how "applied" and
    "loaded" drift apart."""
    importers = set()
    for root, dirs, files in os.walk(_PKG_DIR):
        dirs[:] = [d for d in dirs if d not in ('__pycache__', 'archived')]
        for fname in sorted(files):
            if not fname.endswith('.py'):
                continue
            rel = os.path.relpath(os.path.join(root, fname),
                                  _PKG_DIR).replace(os.sep, '/')
            if rel == 'motion/toss_ilc.py':
                continue
            with open(os.path.join(root, fname), encoding='utf-8') as handle:
                tree = ast.parse(handle.read(), filename=rel)
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    names = [a.name for a in node.names]
                elif isinstance(node, ast.ImportFrom):
                    names = [(node.module or '')] + [
                        (node.module or '') + '.' + a.name for a in node.names]
                else:
                    continue
                if any(n.endswith('toss_ilc') or '.toss_ilc' in n
                       for n in names):
                    importers.add(rel)
    assert importers == {'reload_coordinator_node.py'}, sorted(importers)


def test_the_event_vel_trim_is_applied_at_exactly_one_place():
    """The velocity channel does not ride the aim lookup — it multiplies the
    commanded ``event_vel`` after the release state exists — so it needs its own
    single-site guard. Anything else that scaled ``event_vel`` would be a second
    authority over the number the hand is dispatched at."""
    path = os.path.join(_PKG_DIR, 'reload_coordinator_node.py')
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    # A string-literal scan rather than a Subscript scan: on Python 3.8 a
    # subscript's slice is wrapped in ``ast.Index`` and on 3.9+ it is not, and a
    # guard that silently matched nothing on the Jetson's 3.8 would be worse than
    # no guard at all. The name appears only as a dict key, so the literal IS the
    # site.
    scopes = set()
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        for child in ast.walk(node):
            if isinstance(child, ast.Constant) and \
                    child.value == 'ilc_vel_trim':
                scopes.add(node.name)
    assert scopes == {'_toss_aim_for_goal', '_build_toss_cycle',
                      '_toss_record_fields'}, scopes


# ══════════════════════════════════════════════════════════════════════════════
# 3. Applied — the composition, and the record
# ══════════════════════════════════════════════════════════════════════════════

def test_an_enabled_hit_composes_into_the_total_aim(monkeypatch, tmp_path):
    """Layer 3 is summed with layers 1 and 2 BEFORE the single D7 clamp, so the
    existing clamp stays the final authority over the whole commanded aim rather
    than a fourth bound being added after it."""
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(cal_version=''), enabled=True)
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
        cmd = node._toss_release_cmd
        base = node._toss_release_state
    assert aim['ilc_applied'] is True
    assert aim['ilc_hit'] is True
    assert tuple(aim['ilc_key']) == tuple(_KEY)
    assert aim['ilc_aim_rad'] == pytest.approx((0.0011, -0.0023))
    assert aim['aim_rad'] == pytest.approx((0.0011, -0.0023))
    assert aim['map_aim_rad'] == (0.0, 0.0)
    # The virtual target really moved: the commanded release is a NEW state
    # carrying the tilt, not the announcement's.
    assert cmd is not base
    assert float(cmd.tilt_rx) == pytest.approx(0.0011, abs=1e-9)
    assert float(cmd.tilt_ry) == pytest.approx(-0.0023, abs=1e-9)


def test_the_three_layers_sum_and_the_offsets_split_cleanly(monkeypatch,
                                                            tmp_path):
    """map + trim + ilc, with the mm reports being DIFFERENCES of commanded
    offsets rather than three independent conversions — so they sum to the total
    the machine actually commanded even when a clamp has bound."""
    node = _node(monkeypatch, tmp_path, cal_doc=_cal_doc(math.radians(0.2), 0.0),
                 ilc_doc=_ilc_doc(), enabled=True)
    # The artifact's requires.toss_cal_version has to name the LIVE aim map,
    # whose version is a hash the fixture cannot know until the map has loaded.
    _reload_ilc(node, tmp_path,
                _ilc_doc(aim_rx=math.radians(0.1), aim_ry=0.0, dv=0.0,
                         cal_version=_live_cal_version(node)))
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['ilc_applied'] is True
    assert aim['aim_rad'][0] == pytest.approx(math.radians(0.3), rel=1e-9)
    assert aim['map_aim_rad'][0] == pytest.approx(math.radians(0.2), rel=1e-9)
    assert aim['ilc_aim_rad'][0] == pytest.approx(math.radians(0.1), rel=1e-9)
    # The mm reports partition the commanded total exactly. Note they land on the
    # y axis: the aim→landing Jacobian is a 90° ROTATION, not a scaled identity,
    # so asserting on [0] would pass on a build that had quietly made it one.
    total = tr.aim_target_offset_mm(math.radians(0.3), 0.0, _FLIGHT, _POSE[2])
    assert (aim['map_offset_mm'][1] + aim['trim_offset_mm'][1]
            + aim['ilc_offset_mm'][1]) == pytest.approx(float(total[1]),
                                                        rel=1e-9)
    assert aim['ilc_offset_mm'][1] != 0.0


def test_the_event_vel_trim_scales_the_dispatched_speed(monkeypatch, tmp_path):
    """``k_v − 1``, applied as the multiply ``ilc_fit_lib`` models — the fit and
    the machine must scale the same magnitude in the same place, or the
    sensitivity ``F`` describes a different command than the one that flies.

    The assertion is on ``seq.event_vel_mps`` because that is what
    ``_dispatch_toss_throw`` sends; asserting on ``release_cmd`` would pass on a
    build that computed the trim and never dispatched it.
    """
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(aim_rx=0.0, aim_ry=0.0, dv=-0.10,
                                  cal_version=''), enabled=True)
    seq = _build(node, monkeypatch)
    nominal = tr.compute_release_state(_POSE, _FLIGHT)
    assert seq.event_vel_mps == pytest.approx(
        float(nominal.event_vel_mps) * 0.90, rel=1e-12)
    with node._lock:
        # A velocity-only correction leaves the AIM path bit-identical: the
        # commanded release is still the announcement's object.
        assert node._toss_release_cmd is node._toss_release_state
        assert node._toss_aim['aim_rad'] == (0.0, 0.0)
        assert node._toss_aim['ilc_vel_trim'] == -0.10


def test_the_record_carries_the_APPLIED_ilc_contribution(monkeypatch, tmp_path):
    """Post-gate, in both channels, and the dispatched ``event_vel`` beside them
    — so a future fit can reconstruct exactly what the machine flew."""
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(cal_version=''), enabled=True)
    node._open_toss_record(action='toss', goal_id='dead', cycle_index=1,
                           catch_pose=_POSE, throw_delay=5.0, vel_scale=0.9,
                           raw_goal={}, flight=_FLIGHT)
    seq = _build(node, monkeypatch)
    row = _record(node)
    assert row['ilc_aim_rad'] == pytest.approx([0.0011, -0.0023])
    assert row['ilc_vel_trim'] == pytest.approx(-0.10)
    assert row['total_aim_rad'] == pytest.approx([0.0011, -0.0023])
    assert row['event_vel_mps'] == pytest.approx(float(seq.event_vel_mps),
                                                 rel=1e-12)


def test_a_goal_that_never_built_records_explicit_zeros(monkeypatch, tmp_path):
    """The REJECTED_BAD_GOAL path commanded exactly zero learned correction, and
    the corpus must be able to prove that rather than read a null as "unknown" —
    the same present-and-explicit discipline the map and trim fields follow."""
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(cal_version=''), enabled=True)
    node._open_toss_record(action='toss', goal_id='dead', cycle_index=1,
                           catch_pose=_POSE, throw_delay=5.0, vel_scale=0.9,
                           raw_goal={}, flight=None)
    row = _record(node, outcome='REJECTED_BAD_GOAL')
    assert row['ilc_aim_rad'] == [0.0, 0.0]
    assert row['ilc_vel_trim'] == 0.0


# ══════════════════════════════════════════════════════════════════════════════
# 4. The clamp REFUSES layer 3 (plan risk 5)
# ══════════════════════════════════════════════════════════════════════════════

def test_a_binding_total_clamp_REFUSES_the_ilc_aim_rather_than_truncating_it(
        monkeypatch, tmp_path):
    """THE risk-5 gate. A map saturated at its own 1.0° authority plus any ILC
    aim exceeds D7's total, and the answer is to drop layer 3 WHOLE, loudly.

    Root cause, not convention: a partially truncated correction is not the
    correction that was solved for, so applied-u and recorded-u desynchronise and
    the next fit learns against a command the machine never flew. It is also
    ``ilc_fit_lib.admit_command``'s own rule — the fit refuses a step the clamp
    would truncate and shrinks its trust region instead — and the two halves of
    one loop must agree about what "refused" means.
    """
    node = _node(monkeypatch, tmp_path,
                 cal_doc=_cal_doc(toss_cal.TOTAL_MAX_RAD, 0.0),
                 ilc_doc=_ilc_doc(), enabled=True)
    _reload_ilc(node, tmp_path,
                _ilc_doc(aim_rx=0.001, aim_ry=0.0, dv=0.0,
                         cal_version=_live_cal_version(node)))
    warnings = _warnings(node)
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['ilc_refused'] == 'total_aim'
    assert aim['ilc_aim_rad'] == (0.0, 0.0), 'a REFUSAL is not a truncation'
    # And the commanded aim is EXACTLY the map's own — bit for bit what a build
    # without layer 3 would have flown.
    assert aim['aim_rad'][0] == toss_cal.TOTAL_MAX_RAD
    assert aim['aim_rad'][1] == 0.0
    assert aim['clamp_hits'] == []
    assert any('REFUSED' in w for w in warnings), warnings


def test_the_refusal_records_zero_so_the_next_fit_sees_the_truth(monkeypatch,
                                                                 tmp_path):
    node = _node(monkeypatch, tmp_path,
                 cal_doc=_cal_doc(toss_cal.TOTAL_MAX_RAD, 0.0),
                 ilc_doc=_ilc_doc(), enabled=True)
    _reload_ilc(node, tmp_path,
                _ilc_doc(aim_rx=0.001, aim_ry=0.0, dv=0.0,
                         cal_version=_live_cal_version(node)))
    node._open_toss_record(action='toss', goal_id='dead', cycle_index=1,
                           catch_pose=_POSE, throw_delay=5.0, vel_scale=0.9,
                           raw_goal={}, flight=_FLIGHT)
    _build(node, monkeypatch)
    row = _record(node)
    assert row['ilc_aim_rad'] == [0.0, 0.0]
    assert row['total_aim_rad'][0] == pytest.approx(toss_cal.TOTAL_MAX_RAD)


def test_the_D7_clamp_can_no_longer_BIND_without_layer_3(monkeypatch, tmp_path):
    """**What C4 did to the clamp, stated honestly rather than left implied.**

    Its predecessor —
    ``test_a_clamp_that_binds_on_map_plus_trim_ALONE_still_truncates`` — pinned
    that layer 1/2 keep their long-standing TRUNCATION semantics when layer 3
    contributes nothing, and it drove the over-authority sum with a saturated map
    plus a saturated trim. With layer 2 at zero authority that sum no longer
    exists, and it cannot be reconstructed from layer 1 alone: ``parse_toss_cal``
    refuses any node past ``TOTAL_MAX_RAD`` **on the magnitude**, the same
    quantity and the same number ``clamp_total_aim`` bounds, so every loadable
    map is already inside the clamp.

    So the truncation branch is now unreachable without layer 3, and the two
    halves of that are what this asserts: a map saturated exactly AT the
    authority is commanded verbatim with no clamp hit, and a map past it is
    refused at PARSE — loudly, before a byte is applied — rather than silently
    truncated at apply. The truncation code stays because layer 1 keeps its
    semantics on paper; what changed is that nothing can reach it, which is a
    fact a future reader should find written down rather than rediscover.
    """
    cal = _cal_doc(toss_cal.TOTAL_MAX_RAD, 0.0)
    node = _node(monkeypatch, tmp_path, cal_doc=cal, ilc_doc=None, enabled=True)
    node._params['toss_trim_enabled'] = True
    node._toss_trim = _StubTrim(toss_trim.TRIM_MAX_RAD, 0.0)
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['clamp_hits'] == []
    assert math.hypot(*aim['aim_rad']) == pytest.approx(toss_cal.TOTAL_MAX_RAD,
                                                        rel=1e-12)
    # ... and the saturated MONITOR trim contributed nothing to the sum the clamp
    # saw — this is the C4 demotion from the clamp's side.
    assert aim['trim_aim_rad'] == (0.0, 0.0)
    assert aim['trim_monitor_aim_rad'][0] == pytest.approx(
        toss_trim.TRIM_MAX_RAD)

    # The other half: past the authority the LOADER refuses, so there is nothing
    # left for the apply-time clamp to truncate.
    with pytest.raises(toss_cal.TossCalError):
        toss_cal.parse_toss_cal(_cal_doc(toss_cal.TOTAL_MAX_RAD * 1.01, 0.0))


# ══════════════════════════════════════════════════════════════════════════════
# 5. Provenance dormancy, and the key miss
# ══════════════════════════════════════════════════════════════════════════════

@pytest.mark.parametrize('doc_over,fragment', [
    ({'tilt_version': '2020-01-01-00000000'}, 'tilt_map_version'),
    ({'cal_version': '2020-01-01-ffffffff'}, 'toss_cal_version'),
])
def test_a_provenance_mismatch_is_LOADED_but_DORMANT_and_loud(monkeypatch,
                                                              tmp_path,
                                                              doc_over,
                                                              fragment):
    """Design constraint 5. Loaded, valid, and DOING NOTHING — with a WARN,
    because a silently neutered correction is a number nobody can explain later
    from the console alone."""
    node = _node(monkeypatch, tmp_path, ilc_doc=_ilc_doc(**doc_over),
                 enabled=True)
    assert node._toss_ilc is not None
    warnings = _warnings(node)
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
        assert node._toss_release_cmd is node._toss_release_state
    assert aim['ilc_loaded'] is True
    assert aim['ilc_applied'] is False
    assert aim['ilc_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_vel_trim'] == 0.0
    assert any('DORMANT' in w and fragment in w for w in warnings), warnings
    assert node._toss_ilc_status_snapshot()['ilc_applied'] is False


def test_a_provenance_check_that_CANNOT_BE_COMPLETED_fails_CLOSED(monkeypatch,
                                                                   tmp_path):
    """**An unanswerable provenance question is a mismatch, not an exception.**

    ``provenance_mismatch`` is not a string compare: it calls
    ``model_config_identity()``, which reads six generated ``hardware_config``
    constants through ``getattr`` and floats them. An install tree that predates
    a codegen — exactly the skew ``_toss_ilc_enabled``'s ``getattr(..., False)``
    already fails closed on — raises ``AttributeError`` there, and this call sits
    on the GOAL-BUILD path. Unguarded, it does not cost the correction; it kills
    ``_build_toss_cycle`` and with it the whole toss, for a layer that is
    documented as *a refinement, never a gate*.

    So the failure is fail-CLOSED and matches ``provenance_mismatch``'s own rule:
    *"I cannot verify what is underneath me" is not "the right thing is
    underneath me"*. Zero correction, dormant, loud — and the goal still flies.

    Driven at the raising call itself rather than by deleting a constant from
    ``hardware_config``: several of those names are bound as default arguments at
    import time elsewhere in the release chain, so deleting one would produce a
    different failure at a different place and prove nothing about this seam.
    """
    node = _node(monkeypatch, tmp_path, ilc_doc=_ilc_doc(cal_version=''),
                 enabled=True)
    # CONTROL: with the identity computable this artifact APPLIES, so the
    # assertions below are about the raise and not about a mismatch that was
    # there all along.
    _build(node, monkeypatch)
    with node._lock:
        assert node._toss_aim['ilc_applied'] is True

    def _boom():
        raise AttributeError("module 'jugglebot.hardware_config' has no "
                             "attribute 'HAND_CATCH_OFFSET_MM'")

    monkeypatch.setattr(toss_ilc, 'model_config_identity', _boom)
    warnings = _warnings(node)
    seq = _build(node, monkeypatch)          # MUST NOT raise

    assert seq is not None
    with node._lock:
        aim = dict(node._toss_aim)
        assert node._toss_release_cmd is node._toss_release_state
    assert aim['ilc_loaded'] is True
    assert aim['ilc_applied'] is False
    assert aim['ilc_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_vel_trim'] == 0.0
    assert any('DORMANT' in w and 'AttributeError' in w for w in warnings), \
        warnings

    # The status topic agrees, and is itself survivable: an operator debugging
    # this needs the read-back more than usual, and it also rides the
    # toss/reload_calibration Trigger response.
    snapshot = node._toss_ilc_status_snapshot()
    assert snapshot['ilc_applied'] is False
    assert 'AttributeError' in snapshot['dormant_reason']
    assert snapshot['model_config_identity'].startswith('UNAVAILABLE')


def test_a_DORMANT_aim_map_makes_the_ilc_dormant_too(monkeypatch, tmp_path):
    """The asymmetry against layer 2, and it is deliberate. The session trim was
    measured THIS goal against THIS layer 0, so a dormant layer 1 does not
    invalidate it. Layer 3's provenance explicitly records which aim map was
    being APPLIED underneath it, so a layer 1 that stops applying is a recorded
    premise of the fit going false."""
    node = _node(monkeypatch, tmp_path, cal_doc=_cal_doc(math.radians(0.2), 0.0),
                 ilc_doc=_ilc_doc(), enabled=True)
    _reload_ilc(node, tmp_path,
                _ilc_doc(cal_version=_live_cal_version(node)))
    _build(node, monkeypatch)
    with node._lock:
        assert node._toss_aim['ilc_applied'] is True

    # Now make layer 1 dormant by moving the live tilt map underneath it. Layer 1
    # stops applying, so layer 3's recorded premise is false and it must stop too.
    with node._lock:
        node._tilt_map_version = '2020-01-01-00000000'
    node._load_toss_cal()
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['applied'] is False
    assert aim['ilc_applied'] is False
    assert aim['ilc_aim_rad'] == (0.0, 0.0)
    # The status topic and the apply seam derive "which aim map is underneath"
    # in two places (one holds the lock already, one takes it); they must agree,
    # or the console would report a correction the machine is not commanding.
    assert node._toss_ilc_status_snapshot()['ilc_applied'] is False


def test_a_key_MISS_is_exactly_zero_and_the_machine_says_so(monkeypatch,
                                                            tmp_path):
    """No hull clamp, no nearest neighbour. An operator who expected a correction
    needs to learn that this goal quantised somewhere the fit never visited —
    silence would read as "applied, and it made no difference"."""
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(key=[-150.0, -150.0, 170.0, 0.9],
                                  cal_version=''), enabled=True)
    infos = []
    node.get_logger().info = infos.append
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
        assert node._toss_release_cmd is node._toss_release_state
    assert aim['ilc_applied'] is True, 'the artifact APPLIES; this goal missed'
    assert aim['ilc_hit'] is False
    assert aim['ilc_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_vel_trim'] == 0.0
    assert any('no cell for goal' in str(m) for m in infos), infos


# ══════════════════════════════════════════════════════════════════════════════
# 6. The apply-seam speed gates — validate_event_vel AND throw_envelope
# ══════════════════════════════════════════════════════════════════════════════

def test_a_trim_that_breaks_the_throw_envelope_REFUSES_the_trim_not_the_goal(
        monkeypatch, tmp_path):
    """**C2, at the seam that had no envelope gate at all.**

    Before this, ``_build_toss_cycle`` checked only ``validate_event_vel`` — the
    bridge's [0.3, 7.0] m/s wire band, which "bounds nothing physical". A trim
    that cleared the wire band and broke contract C-HAND-3 sailed through here
    and was then refused by ``TossSequencer`` CHECKING as
    ``REJECTED_THROW_ENVELOPE`` — **the whole goal died for a refinement**, which
    is exactly what layer 3's "a refinement, never a gate" rule forbids.

    Driven by a REAL artifact rather than by injection, and that is the point: at
    T = 1.10 s the admissible ``k_v − 1`` is ``+0.043``
    (``tools/probes/ilc_speed_band.py``, 2026-08-21) while the artifact's
    parse-time ceiling is still ±0.15, so a ``+0.10`` cell is a perfectly legal
    artifact that this machine cannot fly at this flight time. That gap between
    "legal to persist" and "flyable here" is why the gate has to be at the apply
    seam and not only at parse.
    """
    flight = 1.10
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(aim_rx=0.0, aim_ry=0.0, dv=+0.10,
                                  key=[0.0, 150.0, 170.0, flight],
                                  cal_version=''), enabled=True)
    warnings = _warnings(node)
    seq = _build(node, monkeypatch, flight=flight)
    nominal = tr.compute_release_state(_POSE, flight)
    # The wire band would have said yes — so the wire band is not what saved it.
    assert tr.validate_event_vel(float(nominal.event_vel_mps) * 1.10)
    assert seq.event_vel_mps == float(nominal.event_vel_mps)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['ilc_vel_trim'] == 0.0
    assert 'event_vel' in aim['ilc_refused']
    assert any('REJECTED_THROW_ENVELOPE' in w for w in warnings), warnings


def test_an_untrimmable_goal_never_has_its_verdict_flipped_by_layer_3(
        monkeypatch, tmp_path):
    """A goal the machine cannot fly UNTRIMMED gets no trim, admissible or not.

    Root cause: if layer 3 could apply a trim that rescued an out-of-envelope
    goal, then whether a goal flies at all would depend on whether an artifact
    happened to be loaded — and "byte-identical with layer 3 off" would stop
    being true of the machine's *verdict*, only of its arithmetic. The goal is
    then refused by the FSM for its own reason, with the record honestly carrying
    ``ilc_vel_trim = 0.0``.
    """
    # T = 1.25 s is past MAX_FLIGHT_TIME_S = 1.1485 s, so the untrimmed 6.134 m/s
    # is refused (DECEL_AUTHORITY). −0.12 brings it to 5.398 m/s, which the
    # envelope ADMITS — so without the third check this trim would rescue a goal
    # the machine had already refused.
    flight = 1.25
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(aim_rx=0.0, aim_ry=0.0, dv=-0.12,
                                  key=[0.0, 150.0, 170.0, flight],
                                  cal_version=''), enabled=True)
    warnings = _warnings(node)
    seq = _build(node, monkeypatch, flight=flight)
    nominal = tr.compute_release_state(_POSE, flight)
    assert seq.event_vel_mps == float(nominal.event_vel_mps)
    with node._lock:
        assert node._toss_aim['ilc_vel_trim'] == 0.0
    assert any('UNTRIMMED goal is itself outside the throw envelope' in w
               for w in warnings), warnings


def test_an_out_of_band_event_vel_REFUSES_the_trim_not_the_goal(monkeypatch,
                                                                tmp_path):
    """Layer 3 is a refinement, never a gate. Without this branch the FSM would
    mint ``REJECTED_EVENT_VEL`` and the whole goal would die for a refinement.

    **Driven by injection, and that is the honest form for THIS gate.** The wire
    band really is unreachable through the command vector — every trim big
    enough to reach [0.3, 7.0] is refused by the envelope first (the test above
    is the one that exercises a real refusal). So the only way to exercise this
    particular defence-in-depth path is to make the wire gate say no.
    """
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(aim_rx=0.0, aim_ry=0.0, dv=-0.10,
                                  cal_version=''), enabled=True)
    monkeypatch.setattr(_rcn, 'validate_event_vel', lambda *a, **k: False)
    warnings = _warnings(node)
    seq = _build(node, monkeypatch)
    nominal = tr.compute_release_state(_POSE, _FLIGHT)
    assert seq.event_vel_mps == float(nominal.event_vel_mps)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['ilc_vel_trim'] == 0.0
    assert 'event_vel' in aim['ilc_refused']
    assert any('REFUSED' in w for w in warnings), warnings


# ══════════════════════════════════════════════════════════════════════════════
# 7. Observability
# ══════════════════════════════════════════════════════════════════════════════

def test_the_status_topic_reports_enabled_loaded_and_applied_separately(
        monkeypatch, tmp_path):
    """Three booleans, not two: an operator debugging "why did nothing change?"
    has to be able to tell a disabled build from a missing file from a dormant
    artifact.

    ``ilc_applied`` folds the flag in, so it means exactly what a non-zero
    ``ilc_aim_rad`` in the record means — *this machine is commanding the learned
    correction*. A topic saying "applied" beside a record of zeros would be the
    same loaded-vs-applied confusion in a new place. ``dormant_reason`` is what
    separates "the flag is off" from "the provenance does not match".
    """
    import json
    node = _node(monkeypatch, tmp_path, ilc_doc=_ilc_doc(), enabled=False)
    node._publish_toss_cal_status()
    published = json.loads(
        node._publishers['toss/calibration_status'].published[-1].data)
    ilc = published['ilc']
    assert ilc['ilc_enabled'] is False
    assert ilc['ilc_loaded'] is True
    assert ilc['ilc_applied'] is False     # the FLAG is off …
    assert ilc['dormant_reason'] == ''     # … and the provenance is fine
    assert ilc['ilc_cells'] == 1
    assert ilc['ilc_version'] == node._toss_ilc.version
    assert ilc['estimator_version'] == toss_ilc.ESTIMATOR_VERSION
    # The existing keys are untouched — layer 3 rides the same latched topic
    # under a nested key rather than claiming a second one.
    assert published['toss_cal_loaded'] is False


def test_the_status_topic_separates_a_dormant_artifact_from_a_disabled_one(
        monkeypatch, tmp_path):
    """The pair that would otherwise be indistinguishable: both report
    ``ilc_applied = false`` and only ``dormant_reason`` says which."""
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(tilt_version='2020-01-01-00000000'),
                 enabled=True)
    snapshot = node._toss_ilc_status_snapshot()
    assert snapshot['ilc_enabled'] is True
    assert snapshot['ilc_loaded'] is True
    assert snapshot['ilc_applied'] is False
    assert 'tilt_map_version' in snapshot['dormant_reason']


def test_the_dormancy_warning_is_emitted_once_not_once_per_goal(monkeypatch,
                                                                tmp_path):
    """A dormant artifact at 10 goals/min would otherwise bury the console — the
    4091-ERRORs-in-41 s failure mode the aim map's own dormancy log guards
    against."""
    node = _node(monkeypatch, tmp_path,
                 ilc_doc=_ilc_doc(tilt_version='2020-01-01-00000000'),
                 enabled=True)
    warnings = _warnings(node)
    for _ in range(5):
        _build(node, monkeypatch)
    assert len([w for w in warnings if 'DORMANT' in w]) == 1, warnings


# ══════════════════════════════════════════════════════════════════════════════
# 8. C4 — BOTH layers enabled, and no double-count in either direction
# ══════════════════════════════════════════════════════════════════════════════
#
# The digest names two failure narratives. This section owns the COMPOSITION one
# (the machine over-aims by the ILC's contribution while the trim reports
# CONVERGED); ``tests/motion/test_toss_trim.py`` owns the ARITHMETIC one (a
# converged ILC makes the monitor demand a second copy of its own correction, and
# a trim that acted on that demand would drive the artifact to zero).
#
# Both are latent-only-because-both-flags-are-false in the shipped build, which
# is exactly why they need a test: the whole point of the fold-in is that layer 3
# now ships ON.


def test_both_layers_live_the_machine_does_NOT_over_aim_by_the_trim(monkeypatch,
                                                                     tmp_path):
    """**C4, narrative 1.** Map, trim and ILC all live and all non-zero.

    The commanded total is ``map + ilc``, to the last bit — the trim is not in
    it. The failure this closes is not a rounding difference: at the corpus
    operating point the ILC contribution is worth ~36–42 mm of landing shift
    against a 35 mm capture radius, so a machine that added a converged trim on
    top of a converged ILC would miss the cup while both estimators reported
    success.

    The counterfactual is asserted, not just the value: the total that WOULD have
    been commanded under the old composition is computed here and shown to differ
    — otherwise a stub trim that happened to read zero would pass this.
    """
    node = _aimed_node(monkeypatch, tmp_path,
                       ilc_doc=_ilc_doc(aim_rx=math.radians(0.10),
                                        aim_ry=math.radians(-0.05), dv=0.0),
                       enabled=True)
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
        trim = node._toss_trim

    monitor = trim.aim()
    assert monitor != (0.0, 0.0), 'the fixture trim must be non-zero'
    assert aim['ilc_applied'] is True
    assert aim['ilc_aim_rad'] != (0.0, 0.0)

    # What was commanded: map + ilc, exactly.
    assert aim['aim_rad'][0] == pytest.approx(
        aim['map_aim_rad'][0] + aim['ilc_aim_rad'][0], abs=1e-15)
    assert aim['aim_rad'][1] == pytest.approx(
        aim['map_aim_rad'][1] + aim['ilc_aim_rad'][1], abs=1e-15)
    assert aim['trim_aim_rad'] == (0.0, 0.0)

    # What the OLD composition would have commanded, and by how much it differs.
    would_have = (aim['aim_rad'][0] + monitor[0], aim['aim_rad'][1] + monitor[1])
    assert would_have != aim['aim_rad']
    over_aim_mm = tr.aim_target_offset_mm(monitor[0], monitor[1], _FLIGHT,
                                          _POSE[2])
    assert math.hypot(float(over_aim_mm[0]), float(over_aim_mm[1])) > 1.0, (
        'the counterfactual must be a real displacement, not a rounding error')

    # ... and the trim's estimate is still on the record, under its own key.
    assert aim['trim_monitor_aim_rad'] == pytest.approx(monitor)
    assert aim['trim_authority'] == 'MONITOR'


def test_the_trim_keeps_OBSERVING_while_it_commands_nothing(monkeypatch,
                                                            tmp_path):
    """Zero authority, full visibility — the half of decision 1 that is easy to
    lose in a refactor.

    A demotion implemented by never feeding the estimator would satisfy every
    assertion above and destroy the thing the demotion was for: the trim reduces
    ``land_err_mm`` while layer 3's aim update is driven by ``arrival_dir``
    (decision 6), so their divergence per toss is the standing validation that
    C3's by-decision resolution was right. If the monitor stops being fed, that
    validation silently stops existing.
    """
    node = _aimed_node(monkeypatch, tmp_path, ilc_doc=_ilc_doc(), enabled=True)
    observed = []

    class _Recording(_StubTrim):
        def observe(self, rec):
            observed.append(rec)
            return toss_trim.AimObservation('uid', False, 'no_mocap_fit',
                                            None, None)

        def console_lines(self, **kwargs):
            self.applied_flag = kwargs.get('applied')
            return ['TRIM stub']

    node._toss_trim = _Recording()
    node._open_toss_record(action='toss', goal_id='dead', cycle_index=1,
                           catch_pose=_POSE, throw_delay=5.0, vel_scale=0.9,
                           raw_goal={}, flight=_FLIGHT)
    _build(node, monkeypatch)
    node._toss_trim_observe({'toss_uid': 'u1'}, node._toss_trim_snapshot())
    assert observed == [{'toss_uid': 'u1'}]
    # ... and the console says MONITOR, never APPLIED: an operator who reads a
    # converging trim as a converging CORRECTION goes looking for it in the
    # commanded aim and finds nothing.
    assert node._toss_trim.applied_flag is False


# ══════════════════════════════════════════════════════════════════════════════
# 9. C1 — the session common mode at the apply seam, and C5's composition
# ══════════════════════════════════════════════════════════════════════════════
#
# Owner decision 2 of the 2026-08-21 fold-in. The loader's half (schema, bounds,
# the transposed evidence gate) is in `tests/motion/test_toss_ilc.py` section 9;
# this section owns what only the NODE can be asked — that the two layer-3
# components COMPOSE, that a D7 clamp hit drops BOTH, that the common mode is not
# keyed on a cell hit, and that it dies with the goal.

_ANCHOR = {'aim_rad': [math.radians(0.55), math.radians(-0.24)], 'n': 7,
           'se_rad': [0.0005, 0.0005]}


def _anchored(**over):
    """`_ilc_doc` plus an anchor that CLEARS the evidence gate.

    9.6 mrad at n = 7 — the measured per-cell |aim| is 9.1-10.5 mrad, so this is
    the real operating magnitude rather than a number chosen to pass.
    """
    doc = _ilc_doc(**over)
    doc['anchor'] = dict(_ANCHOR)
    return doc


def _goal(node, monkeypatch, goal_id='g1', **kw):
    """Start a GOAL the way production does — open the record context,
    `_toss_trim_begin` (which seeds the layer-3 session component), then build
    one cycle.

    Going through the real begin hook matters: it is the single place the
    component is created, and a test that hand-installed one would pass on a
    build where the hook had been dropped.
    """
    node._open_toss_record(action='toss', goal_id=goal_id, cycle_index=1,
                           catch_pose=kw.get('pose', _POSE), throw_delay=5.0,
                           vel_scale=0.9, raw_goal={},
                           flight=kw.get('flight', _FLIGHT))
    node._toss_trim_begin(goal_id=goal_id)
    return _build(node, monkeypatch, **kw)


def test_the_commanded_aim_is_the_SUM_of_BOTH_layer_3_components(monkeypatch,
                                                                  tmp_path):
    """**C5's composition, and the arithmetic C1 rests on.**

    ``clamp_total_aim(map + ilc_spatial + ilc_session)``, and the record carries
    the total plus its two parts. The total is what every consumer that subtracts
    what layer 3 applied must read — ``toss_trim.ilc_aim_rad``'s C4 subtraction
    above all — so it stays ``ilc_aim_rad`` and the split rides beside it.

    Both parts are non-zero and DIFFERENT here on purpose: equal parts, or a zero
    part, would let a build that dropped one of them pass.
    """
    node = _node(monkeypatch, tmp_path, ilc_doc=None, enabled=True)
    _reload_ilc(node, tmp_path, _anchored())
    _goal(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)

    spatial = aim['ilc_spatial_aim_rad']
    session = aim['ilc_session_aim_rad']
    assert spatial != (0.0, 0.0) and session != (0.0, 0.0)
    assert spatial != session
    assert aim['ilc_session_applied'] is True
    assert aim['ilc_session_reason'] == toss_ilc.SESSION_APPLIED
    assert aim['ilc_session_n'] == _ANCHOR['n']

    for i in (0, 1):
        assert aim['ilc_aim_rad'][i] == pytest.approx(
            spatial[i] + session[i], abs=1e-15)
        # No map in this fixture, so the commanded total IS layer 3's total.
        assert aim['aim_rad'][i] == pytest.approx(aim['ilc_aim_rad'][i],
                                                  abs=1e-15)
    # ... and the record carries all three, so a corpus can attribute the aim.
    row = _record(node)
    assert row['ilc_aim_rad'] == [pytest.approx(v) for v in aim['ilc_aim_rad']]
    assert row['ilc_spatial_aim_rad'] == [pytest.approx(v) for v in spatial]
    assert row['ilc_session_aim_rad'] == [pytest.approx(v) for v in session]
    assert row['ilc_session_applied'] is True
    assert row['ilc_session_n'] == _ANCHOR['n']


def test_the_session_common_mode_applies_on_a_cell_MISS(monkeypatch, tmp_path):
    """**The asymmetry that is the whole point of splitting the two.**

    A miss contributes exactly zero SPATIAL residual — ``toss_ilc.lookup``
    interpolates nothing, and that rule is untouched. It does NOT zero the common
    mode.

    Root cause for the difference: the no-interpolation rule exists because a
    sparse command-vector table must not invent a value BETWEEN its cells. The
    common mode is by construction not a function of the cell — it measured
    consistent at 9.1-10.5 mrad across all three goal cells — so applying it at an
    unvisited goal is applying a constant, not interpolating a field. It is also
    exactly what layer 2 has always done with its own common mode: the session
    trim applies at every goal, whether or not the map has a node there.

    The tradeoff, accepted and recorded rather than hidden: at a goal far outside
    the fitted region this extrapolates a constant measured inside it. It is
    bounded by ``ILC_AIM_MAX_RAD``, gated on evidence, and written to every record
    as ``ilc_session_aim_rad`` so the assumption is visible in the corpus.
    """
    node = _node(monkeypatch, tmp_path, ilc_doc=None, enabled=True)
    _reload_ilc(node, tmp_path, _anchored(key=[300.0, 300.0, 170.0, 0.9]))
    _goal(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['ilc_hit'] is False
    assert aim['ilc_spatial_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_session_applied'] is True
    assert aim['ilc_session_aim_rad'] != (0.0, 0.0)
    assert aim['ilc_aim_rad'] == aim['ilc_session_aim_rad']
    # The velocity trim is per CELL and stays exactly zero on a miss: the common
    # mode is an AIM quantity — a level() draw is an orientation, and it does not
    # touch release speed.
    assert aim['ilc_vel_trim'] == 0.0


def test_a_D7_clamp_hit_drops_BOTH_layer_3_components(monkeypatch, tmp_path):
    """**Drop-the-whole-layer, and "whole" means both parts.**

    Dropping only the spatial residual and keeping the common mode (or the
    reverse) would fly a correction no fit ever solved for: the cells are
    referenced TO the anchor, so ``spatial`` alone is a residual about a baseline
    the machine is not applying, and ``session`` alone is a baseline with its
    residual removed. Half a decomposition is not a smaller correction, it is a
    different one — which is the same argument that makes a TRUNCATED correction
    a refusal rather than a smaller step (risk 5).
    """
    cal = _cal_doc(toss_cal.TOTAL_MAX_RAD * 0.95, 0.0)
    node = _node(monkeypatch, tmp_path, cal_doc=cal, ilc_doc=None, enabled=True)
    doc = _anchored(aim_rx=math.radians(0.3), aim_ry=0.0, dv=0.0)
    doc['requires'] = dict(doc['requires'],
                           toss_cal_version=_live_cal_version(node))
    _reload_ilc(node, tmp_path, doc)
    _goal(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)

    assert aim['ilc_refused'] == 'total_aim'
    assert aim['ilc_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_spatial_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_session_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_session_applied'] is False
    # The goal still flies the MAP aim, unchanged and unclamped — layer 3 is a
    # refinement and never a gate.
    assert aim['aim_rad'][0] == pytest.approx(toss_cal.TOTAL_MAX_RAD * 0.95)
    assert aim['clamp_hits'] == []
    # ... and the record says zero, so the next fit sees what actually flew.
    row = _record(node)
    assert row['ilc_aim_rad'] == [0.0, 0.0]
    assert row['ilc_spatial_aim_rad'] == [0.0, 0.0]
    assert row['ilc_session_aim_rad'] == [0.0, 0.0]


def test_a_DORMANT_artifact_contributes_NO_session_prior(monkeypatch, tmp_path):
    """The prior is gated by the SAME dormancy verdict the cells are, decided
    once and in one place.

    Root cause, and it is layer 2's own audit fix of 2026-08-11 one layer up: a
    prior carried in from an artifact fitted under a different levelling layer is
    the D3 double-count arriving through the back door, with none of the fence's
    warning. "I cannot verify what is underneath me" is not "the right thing is
    underneath me".
    """
    node = _node(monkeypatch, tmp_path, ilc_doc=_anchored(tilt_version='other'),
                 enabled=True)
    _goal(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['ilc_applied'] is False
    assert aim['ilc_session_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_session_applied'] is False
    assert aim['aim_rad'] == (0.0, 0.0)


def test_the_flag_withholds_the_session_prior_as_well_as_the_cells(monkeypatch,
                                                                    tmp_path):
    """``JB_OP_TOSS_ILC_ENABLED`` off must withhold the WHOLE layer. A build
    that gated only the cell lookup would command a learned correction with the
    feature flag off, which is the one thing the flag exists to make
    impossible."""
    node = _node(monkeypatch, tmp_path, ilc_doc=_anchored(), enabled=False)
    _goal(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['ilc_enabled'] is False
    assert aim['ilc_session_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_aim_rad'] == (0.0, 0.0)
    assert aim['aim_rad'] == (0.0, 0.0)


def test_the_session_component_is_DISCARDED_at_goal_end(monkeypatch, tmp_path):
    """RAM only, one per goal. ``_toss_trim_end`` drops it — and drops it FIRST,
    before the trim's best-effort proposal write, so a full disk cannot leave a
    stale common mode alive into the next goal.

    A component that survived its goal would be applying one sitting's ``level()``
    draw to a sitting that never took it, which is C1 with the persistence moved
    from the file into the process.
    """
    node = _node(monkeypatch, tmp_path, ilc_doc=_anchored(), enabled=True)
    node._toss_trim_begin(goal_id='g1')
    with node._lock:
        first = node._toss_ilc_session
    assert first is not None and first.applied is True

    node._toss_trim_end()
    with node._lock:
        assert node._toss_ilc_session is None

    # A new goal seeds a FRESH one from the artifact — never the old object.
    node._toss_trim_begin(goal_id='g2')
    with node._lock:
        second = node._toss_ilc_session
    assert second is not None and second is not first
    assert second.aim() == first.aim()


def test_a_session_level_draw_never_reaches_a_PERSISTED_cell(monkeypatch,
                                                              tmp_path):
    """**THE C1 gate, in the shape the build ladder asks for.**

    Two goals in one node — the stand-in for two sessions, since the component's
    lifetime IS the goal — with the persisted artifact untouched between them.
    Whatever the common mode commands, it commands from the anchor block and the
    cells are byte-identical before and after. There is no path from an applied
    common mode back into a cell, because there is no write path at all.
    """
    node = _node(monkeypatch, tmp_path, ilc_doc=_anchored(), enabled=True)
    with node._lock:
        before = dict(node._toss_ilc.cells)
        version = node._toss_ilc_version

    seen = []
    for goal_id in ('g1', 'g2'):
        _goal(node, monkeypatch, goal_id=goal_id)
        with node._lock:
            seen.append(dict(node._toss_aim)['ilc_session_aim_rad'])
        node._toss_trim_end()

    assert seen[0] != (0.0, 0.0) and seen[0] == seen[1], (
        'the prior is deterministic — a difference here would mean the '
        'component had learnt something, which it has no path to do')
    with node._lock:
        assert dict(node._toss_ilc.cells) == before
        assert node._toss_ilc_version == version
    # ... and the commanded common mode is nowhere in any persisted cell.
    for correction in before.values():
        assert correction.aim_rad != seen[0]


def test_a_thin_anchor_leaves_the_cells_commanding_alone(monkeypatch, tmp_path):
    """The gate refusing the prior must not refuse the SPATIAL residual with it.

    They are separate evidence: the cells were fitted from admitted tosses at
    that goal, the anchor from between-session repeats, and a thin anchor says
    nothing about the cells. Coupling them would make a fresh machine — which by
    definition has one session of anchor evidence — command no correction at all.
    """
    doc = _anchored()
    doc['anchor'] = dict(_ANCHOR, n=1)
    node = _node(monkeypatch, tmp_path, ilc_doc=doc, enabled=True)
    _goal(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['ilc_session_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_session_reason'] == toss_ilc.SESSION_INSUFFICIENT_EVIDENCE
    assert aim['ilc_session_n'] == 1
    assert aim['ilc_spatial_aim_rad'] != (0.0, 0.0)
    assert aim['ilc_aim_rad'] == aim['ilc_spatial_aim_rad']


def test_an_artifact_with_NO_anchor_is_the_pre_C1_machine_bit_for_bit(
        monkeypatch, tmp_path):
    """Absent anchor ⇒ zero session component ⇒ the composition is exactly what
    it was before C1 landed. Asserted against a live goal rather than by
    inspection, because "adds zero" and "is not in the expression" have to be the
    same thing here and only the arithmetic can say so."""
    node = _node(monkeypatch, tmp_path, ilc_doc=_ilc_doc(), enabled=True)
    _goal(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    assert aim['ilc_session_aim_rad'] == (0.0, 0.0)
    assert aim['ilc_session_reason'] == toss_ilc.SESSION_NO_ANCHOR
    assert aim['ilc_aim_rad'] == aim['ilc_spatial_aim_rad']
    assert aim['ilc_aim_rad'] != (0.0, 0.0), 'the cell arm must have run'


def test_the_session_component_is_seeded_in_exactly_one_scope():
    """D4's manifest shape, applied to the C1 component: it is constructed in ONE
    place (``_toss_ilc_session_begin``) and read in ONE place
    (``_toss_aim_for_goal``).

    Two construction sites is how the two per-goal RAM components get out of
    step — one goal starts a trim and not a common mode — and two read sites is
    how a goal ends up commanding two different values of a quantity that is
    defined to be constant for its whole life.
    """
    constructs = _calls_in_package({'toss_ilc.IlcSessionCommonMode'})
    assert sorted({scope for _f, scope, _c in constructs}) == [
        '_toss_ilc_session_begin'], constructs
    reads = _calls_in_package({'session.aim'})
    assert sorted({scope for _f, scope, _c in reads}) == [
        '_toss_aim_for_goal'], reads
