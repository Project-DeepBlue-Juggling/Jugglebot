"""Node seams for the toss AIM calibration — contract C-TOSS-CAL-1, phase 2b.

The loader itself is covered in ``tests/motion/test_toss_cal.py`` (pure, no ROS)
and the aim geometry in ``tests/motion/test_toss_release.py``. Here we test the
four things only the NODE can be asked:

1. **Absent map ⇒ today's machine, bit for bit.** No new rejection code, no new
   topic traffic, the commanded release IS the announcement release — the same
   object, so the disabled path costs not one floating-point operation.
2. **``catch/pretilt_hold`` on ANY non-zero commanded aim, tier-independent**
   — the D3 rule these tests were written for. NOTE that the hold became
   UNCONDITIONAL on 2026-08-22 (census E5, a cadence fix): it is raised for a
   level 8a too, so it is no longer EVIDENCE that an aim is commanded. These
   tests remain valid because each asserts the commanded release state
   alongside the topic, and because their scope is the goal BUILD rather than
   the PREPARE tick where the hold is published.
   (D3). Without it the stock announcement pre-tilt levels the platform back
   ≥ 1 s before release while every log line still reports the aim as applied.
3. **The announcement stays the UNCORRECTED landing** (D4) — the aim moves the
   commanded pose and ``event_vel`` and nothing else.
4. **Provenance dormancy** (D3): a ``requires.tilt_map_version`` mismatch ⇒
   loaded, NOT applied, loud, and the goal is aimed vertically.

ROS 2 is mocked by ``tests/ros/conftest.py``; service calls are monkeypatch-seamed
in the ``test_toss_coordinator.py`` pattern.
"""

from __future__ import annotations

import json
import math

import numpy as np
import pytest
import yaml

import jugglebot.hardware_config as hw
from jugglebot.motion import toss_cal
from jugglebot.motion.trajectory import toss_release as tr
from jugglebot.reload_coordinator_node import ReloadCoordinatorNode
from jugglebot.toss_sequencer import TIER_8A, TIER_8B

_TILT_V = '2026-08-10-deadbeef'


# ── Fixtures ──────────────────────────────────────────────────────────────────

def _cal_doc(rx, ry, *, tilt_version=_TILT_V, x=(-150.0, 0.0, 150.0),
             y=(-150.0, 0.0, 150.0)):
    """A CONSTANT-field map: every node carries the same aim, so a lookup at any
    goal pose returns it exactly and the test's expectation needs no
    interpolation arithmetic of its own."""
    grid = [[float(rx)] * len(x) for _ in y]
    grid_y = [[float(ry)] * len(x) for _ in y]
    return {
        'version': 1,
        'captured': {'date': '2026-08-11', 'tool': 'test'},
        'requires': {'tilt_map_version': tilt_version,
                     'estimator_version': toss_cal.ESTIMATOR_VERSION},
        'units': {'aim': 'rad'},
        'grid': {'z_mm': 170.0, 'x_mm': list(x), 'y_mm': list(y)},
        'aim_rad': {'rx': grid, 'ry': grid_y},
    }


def _node_with_map(monkeypatch, tmp_path, doc, *, live_tilt=_TILT_V):
    """A node whose aim map resolves to ``doc`` (or none, when ``doc`` is None),
    with the live tilt-map version already latched."""
    if doc is None:
        monkeypatch.setattr(toss_cal, 'resolve_toss_cal_path', lambda *a: None)
    else:
        path = tmp_path / 'toss_calibration.yaml'
        path.write_text(yaml.safe_dump(doc, sort_keys=False))
        monkeypatch.setattr(toss_cal, 'resolve_toss_cal_path',
                            lambda *a: str(path))
    monkeypatch.setattr(toss_cal, 'toss_cal_candidates', lambda *a: ('<test>',))
    node = ReloadCoordinatorNode()
    with node._lock:
        node._tilt_map_loaded = bool(live_tilt)
        node._tilt_map_version = live_tilt
    # __init__ loaded before the tilt version was known; reload so the
    # provenance verdict is computed against it (this is exactly what a live
    # /trajectory/status arrival does, minus the message plumbing).
    node._load_toss_cal()
    return node


def _build(node, pose=(0.0, 150.0, 170.0), flight=0.8, tier=TIER_8A,
           monkeypatch=None):
    if monkeypatch is not None:
        monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', tier)
    # `_build_toss_cycle` returns (seq, TossCycleState) since Phase B1;
    # this helper keeps handing back the sequencer, and every caller
    # reads the cycle state off `node._toss_committed`.
    return node._build_toss_cycle(pose, flight, 5.0, 0.9)[0]


def _status(node):
    pub = node._publishers['toss/calibration_status']
    return json.loads(pub.published[-1].data)


# ── 1. Absent map ⇒ exactly today's machine ───────────────────────────────────

def test_absent_map_leaves_the_release_state_untouched(monkeypatch, tmp_path):
    """The 2b headline gate. With no map the COMMANDED release is the same
    OBJECT as the announcement release — not merely equal. An absent calibration
    must cost not one extra floating-point operation, so "no map" cannot differ
    from today even by an ULP."""
    node = _node_with_map(monkeypatch, tmp_path, None)
    _build(node, monkeypatch=monkeypatch)
    with node._lock:
        assert node._toss_committed.release_cmd is node._toss_committed.release_state
        assert node._toss_committed.release_state.__class__ is tr.ReleaseState
        assert node._toss_committed.aim['aim_rad'] == (0.0, 0.0)
        assert node._toss_committed.aim['offset_mm'] == (0.0, 0.0)
    expected = tr.compute_release_state((0.0, 150.0, 170.0), 0.8)
    got = node._toss_committed.release_state
    assert np.array_equal(got.release_pos_global_mm,
                          expected.release_pos_global_mm)
    assert np.array_equal(got.launch_vel_mms, expected.launch_vel_mms)
    assert got.event_vel_mps == expected.event_vel_mps


def test_absent_map_adds_no_rejection_code_and_no_pretilt_hold(monkeypatch,
                                                               tmp_path):
    """C-TOSS-CAL-1 is a refinement, never a gate: absence is silent.

    The scope here is the GOAL BUILD, not the whole cycle: nothing about an
    absent map may reach a topic while the release state is being resolved. The
    PREPARE tick is a separate question and its answer changed on 2026-08-22 —
    ``catch/pretilt_hold`` is raised there UNCONDITIONALLY now (census E5), for a
    cadence reason that has nothing to do with the map. Asserting emptiness here
    still pins what this test is about; see
    ``test_toss_coordinator.py::test_pretilt_hold_is_raised_on_every_cycle_
    including_a_level_one`` for the PREPARE-tick behaviour."""
    node = _node_with_map(monkeypatch, tmp_path, None)
    seq = _build(node, monkeypatch=monkeypatch)
    assert seq.tilt_clamp_exceeded is False
    assert node._release_is_tilted(node._toss_commanded_release()) is False
    assert node._publishers['catch/pretilt_hold'].published == []
    status = _status(node)
    assert status['toss_cal_loaded'] is False
    assert status['toss_cal_applied'] is False
    assert status['toss_cal_version'] == ''


def test_a_loaded_but_all_zero_map_is_still_the_identity_path(monkeypatch,
                                                              tmp_path):
    """A flat-field map (§ 3.8 refuses to WRITE one, but an operator can hand-
    edit one to zero as a rollback) must take the same identity path as no map
    at all — the aim is zero, so the tilted branch is never entered.

    Same scope caveat as the test above: this is the goal BUILD. The PREPARE
    tick raises ``catch/pretilt_hold`` unconditionally since census E5."""
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(0.0, 0.0))
    _build(node, monkeypatch=monkeypatch)
    with node._lock:
        assert node._toss_committed.release_cmd is node._toss_committed.release_state
        # loaded, valid, commanding 0
        assert node._toss_committed.aim['applied'] is True
    assert node._publishers['catch/pretilt_hold'].published == []


# ── 2. pretilt_hold on ANY non-zero aim, tier-independent (D3) ────────────────

def _prepare_tick(node, seq, now):
    """Drive the ACTION_PREPARE_CATCH branch of _step_toss_sequence directly —
    the tick that raises the holds. The FSM's own path to it is covered in
    test_toss_coordinator.py; here only the hold decision is under test."""
    from jugglebot.toss_sequencer import ACTION_PREPARE_CATCH, TossDecision
    decision = TossDecision(done=False, phase='PREPARING',
                            action=ACTION_PREPARE_CATCH, result=None)
    node._build_toss_observations = lambda _now, _state=None: None
    seq.step = lambda _now, _obs: decision
    node._step_toss_sequence(seq, now)


def test_pretilt_hold_is_raised_for_a_tier_8a_aim(monkeypatch, tmp_path):
    """THE structural gate. Today 8a never touches catch/pretilt_hold and gets
    away with it because its pre-tilt target equals the pose already held. The
    moment a toss commands an aim tilt the hold becomes MANDATORY: without it
    ``catch_coordinator._on_throw_announcement``'s stock pre-tilt completes the
    un-tilt to level ≥ 1 s BEFORE release, so the aim is reverted while every
    log line reports it as applied. Silent-wrong is the expensive class here."""
    aim = math.radians(0.4)
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(aim, 0.0))
    seq = _build(node, monkeypatch=monkeypatch)
    assert seq.tier == TIER_8A
    assert node._release_is_tilted(node._toss_commanded_release()) is True
    _prepare_tick(node, seq, 100.0)
    published = [m.data for m in node._publishers['catch/pretilt_hold'].published]
    assert published == [True]
    assert node._toss_committed.pretilt_hold_raised is True


def test_pretilt_hold_is_raised_for_every_non_zero_aim_direction(monkeypatch,
                                                                 tmp_path):
    """Structural over every axis and sign — an implementation that keyed on
    ``rx`` alone, or on a magnitude with a deadband, would pass a single-axis
    test and silently drop half the field."""
    for rx, ry in ((1e-5, 0.0), (-1e-5, 0.0), (0.0, 1e-5), (0.0, -1e-5),
                   (math.radians(0.3), math.radians(-0.3))):
        node = _node_with_map(monkeypatch, tmp_path, _cal_doc(rx, ry))
        seq = _build(node, monkeypatch=monkeypatch)
        _prepare_tick(node, seq, 100.0)
        assert [m.data for m in
                node._publishers['catch/pretilt_hold'].published] == [True], (
            f'aim ({rx}, {ry}) did not raise catch/pretilt_hold')


def test_the_hold_is_released_at_the_terminal_for_an_aimed_8a_goal(monkeypatch,
                                                                   tmp_path):
    """The teardown is keyed on the cycle state's ``pretilt_hold_raised``, so it
    follows the raise automatically — but a level goal must still never touch
    the topic."""
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(math.radians(0.4), 0.0))
    seq = _build(node, monkeypatch=monkeypatch)
    _prepare_tick(node, seq, 100.0)
    node._recenter = lambda: None                # the ladder is not under test
    node._toss_recenter()
    assert [m.data for m in
            node._publishers['catch/pretilt_hold'].published] == [True, False]

    level = _node_with_map(monkeypatch, tmp_path, None)
    level._recenter = lambda: None
    level._toss_recenter()
    assert level._publishers['catch/pretilt_hold'].published == []


# ── 3. The aim reaches the platform, and only the platform (D4) ───────────────

def test_the_commanded_orientation_carries_the_aim(monkeypatch, tmp_path):
    """The re-keyed ``_position_platform_for_toss`` branch: a tilted commanded
    release sends the tilt quaternion, a level one sends identity."""
    aim_rx, aim_ry = math.radians(0.35), math.radians(-0.20)
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(aim_rx, aim_ry))
    seq = _build(node, monkeypatch=monkeypatch)
    sent = {}
    node._go_to_pose_cli.wait_for_service = lambda timeout_sec=None: True

    def _call_async(req):
        sent['req'] = req
        return 'future'
    node._go_to_pose_cli.call_async = _call_async
    node._wait_future = lambda _f: None          # NO_RESPONSE; we only want the req
    node._position_platform_for_toss(seq)

    cmd = node._toss_commanded_release()
    assert cmd.tilt_rx == pytest.approx(aim_rx, abs=1e-9)
    assert cmd.tilt_ry == pytest.approx(aim_ry, abs=1e-9)
    want = ReloadCoordinatorNode._tilt_quaternion(cmd.tilt_rx, cmd.tilt_ry)
    got = sent['req'].pose.orientation
    assert (got.w, got.x, got.y, got.z) == (want.w, want.x, want.y, want.z)
    # ... and the commanded POSITION is the swing-compensated pre-tilt pose, so
    # the CUP still lands on the nominated B rather than a cup-swing away.
    pre = cmd.pretilt_pose_stow
    assert (sent['req'].pose.position.x,
            sent['req'].pose.position.y) == (pytest.approx(float(pre[0])),
                                             pytest.approx(float(pre[1])))
    assert abs(float(pre[0]) - 0.0) < 1.2        # cup swing ≤ 1.13 mm at 1°
    assert abs(float(pre[1]) - 150.0) < 1.2


def test_the_announcement_landing_stays_uncorrected(monkeypatch, tmp_path):
    """D4. The announcement is the PREDICTION of where the ball goes, and after
    a correct aim correction that is B — so it must not move. Keeping it
    uncorrected leaves the correlation→catch path, the receive-tilt computation
    and the possession plausibility bound bitwise unchanged; only the commanded
    pre-tilt pose and event_vel change."""
    pose = (0.0, 150.0, 170.0)
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(math.radians(0.8),
                                                          math.radians(0.5)))
    _build(node, pose=pose, monkeypatch=monkeypatch)
    plain = tr.compute_release_state(pose, 0.8)
    ann = node._toss_committed.release_state
    assert np.array_equal(ann.catch_point_global_mm,
                          plain.catch_point_global_mm)
    assert np.array_equal(ann.launch_vel_mms, plain.launch_vel_mms)
    assert np.array_equal(ann.release_pos_global_mm, plain.release_pos_global_mm)
    with node._lock:
        assert node._toss_committed.landing_global_mm == tuple(
            float(v) for v in plain.catch_point_global_mm)
    # The COMMANDED state is the one that moved.
    cmd = node._toss_commanded_release()
    assert cmd is not ann
    assert not np.array_equal(cmd.launch_vel_mms, plain.launch_vel_mms)
    # event_vel grows by 1/cos(aim) only — 0.94 deg total here.
    ratio = cmd.event_vel_mps / plain.event_vel_mps
    assert 1.0 < ratio < 1.0002


def test_the_dispatched_event_vel_is_the_commanded_one(monkeypatch, tmp_path):
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(math.radians(0.8), 0.0))
    seq = _build(node, monkeypatch=monkeypatch)
    assert seq.event_vel_mps == pytest.approx(
        node._toss_commanded_release().event_vel_mps, abs=1e-12)
    assert tr.validate_event_vel(seq.event_vel_mps)


def test_the_positioning_crosscheck_target_matches_the_commanded_pose(
        monkeypatch, tmp_path):
    """One source for the go_to_pose command and the mocap arrival cross-check —
    else an operator who configures a platform body for an aimed sitting gets
    measured-pretilt vs target-B and a spurious ABORTED_POSITION_FAILED."""
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(math.radians(0.6), 0.0))
    _build(node, monkeypatch=monkeypatch)
    cmd = node._toss_commanded_release()
    with node._lock:
        target = node._toss_committed.platform_target_mm
    assert target == pytest.approx(tuple(float(v)
                                         for v in cmd.pretilt_pose_stow[:3]))


def test_the_record_declares_what_was_applied(monkeypatch, tmp_path):
    """§ 3.3's "most important block". The applied bias is recorded per toss,
    which is what makes the phase-2c fit a converging FIXED POINT rather than a
    one-shot measurement — captures do not have to run with the map
    uninstalled."""
    aim_rx, aim_ry = math.radians(0.5), math.radians(-0.25)
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(aim_rx, aim_ry))
    node._open_toss_record(action='toss', goal_id='abcd1234efgh',
                           cycle_index=1, catch_pose=(0.0, 150.0, 170.0),
                           throw_delay=5.0, vel_scale=0.9,
                           raw_goal={'throw_height_m': 0.0,
                                     'throw_delay_s': 0.0,
                                     'catch_vel_scale': 0.0},
                           flight=0.8)
    _build(node, monkeypatch=monkeypatch)
    from jugglebot.toss_sequencer import TossResult
    row = node._toss_record_fields(TossResult(True, 'CAUGHT'))
    assert row['map_aim_rad'] == pytest.approx([aim_rx, aim_ry], abs=1e-12)
    assert row['total_aim_rad'] == pytest.approx([aim_rx, aim_ry], abs=1e-12)
    assert row['trim_aim_rad'] == [0.0, 0.0]          # phase 2e
    assert row['toss_cal_loaded'] is True
    assert row['toss_cal_applied'] is True
    assert row['toss_cal_version'] == toss_cal.map_version(
        _cal_doc(aim_rx, aim_ry))
    assert row['clamp_hits'] == []
    # The commanded tilt matches the map, and the mm report field matches the
    # offset that was actually commanded (no re-derivation to drift from).
    assert row['aim_tilt_rx_rad'] == pytest.approx(aim_rx, abs=1e-9)
    assert row['aim_tilt_ry_rad'] == pytest.approx(aim_ry, abs=1e-9)
    want = tr.aim_target_offset_mm(aim_rx, aim_ry, 0.8, 170.0)
    assert row['map_aim_mm_at_h'] == pytest.approx(list(want), abs=1e-12)
    # catch_point stays B's cup point — the quantity the miner recomputes from
    # goal_catch_xyz_stow_mm and fails loud on mismatch (§ 7 R1).
    assert row['catch_point_global_mm'] == pytest.approx(
        list(tr.compute_release_state((0.0, 150.0, 170.0), 0.8)
             .catch_point_global_mm))


# ── 4. Provenance dormancy (D3) ───────────────────────────────────────────────

def test_a_provenance_mismatch_loads_but_does_not_apply(monkeypatch, tmp_path):
    """The map is LOADED and observable, and commands NOTHING. Root cause: an
    aim residual fitted under tilt map A double-counts tilt map B's delta. This
    is the loaded-vs-applied distinction stated on the wire from day one."""
    doc = _cal_doc(math.radians(0.4), 0.0, tilt_version='2026-01-01-00000000')
    node = _node_with_map(monkeypatch, tmp_path, doc, live_tilt=_TILT_V)
    status = _status(node)
    assert status['toss_cal_loaded'] is True
    assert status['toss_cal_applied'] is False
    assert 'tilt_map_version' in status['dormant_reason']
    assert status['toss_cal_version'] == toss_cal.map_version(doc)

    _build(node, monkeypatch=monkeypatch)
    with node._lock:
        assert node._toss_committed.release_cmd is node._toss_committed.release_state
        assert node._toss_committed.aim['aim_rad'] == (0.0, 0.0)
        assert node._toss_committed.aim['loaded'] is True
        assert node._toss_committed.aim['applied'] is False
    assert node._publishers['catch/pretilt_hold'].published == []


def test_the_dormancy_warning_is_loud_once_per_pairing(monkeypatch, tmp_path):
    """Loud, but once. A dormant map at 10 goals/min would otherwise bury every
    other console line — the 4091-ERROR-lines-in-41 s failure mode."""
    doc = _cal_doc(math.radians(0.4), 0.0, tilt_version='2026-01-01-00000000')
    node = _node_with_map(monkeypatch, tmp_path, doc)
    warnings = []
    node.get_logger().warning = warnings.append
    for _ in range(4):
        _build(node, monkeypatch=monkeypatch)
    dormant = [w for w in warnings if 'DORMANT' in w]
    assert len(dormant) == 1, warnings
    assert 'tilt_map_version' in dormant[0]


def test_an_unknown_live_tilt_map_version_is_dormant_not_applied(monkeypatch,
                                                                 tmp_path):
    """Fail closed. "I cannot verify which levelling layer is underneath me" is
    not "the right one is underneath me"."""
    node = _node_with_map(monkeypatch, tmp_path,
                          _cal_doc(math.radians(0.4), 0.0), live_tilt='')
    assert _status(node)['toss_cal_applied'] is False
    _build(node, monkeypatch=monkeypatch)
    with node._lock:
        assert node._toss_committed.aim['applied'] is False


def test_a_tilt_map_reload_re_evaluates_dormancy_live(monkeypatch, tmp_path):
    """The aim map's applied-ness is a function of the live tilt-map version, so
    a tilt-map reload can flip a dormant aim map without touching this node's
    own reload service — and the status topic must say so."""
    node = _node_with_map(monkeypatch, tmp_path,
                          _cal_doc(math.radians(0.4), 0.0), live_tilt='')
    assert _status(node)['toss_cal_applied'] is False

    class _Status:
        streaming = True
        gravity_correction_loaded = True
        tilt_map_loaded = True
        tilt_map_version = _TILT_V
    node._on_traj_status(_Status())
    assert _status(node)['toss_cal_applied'] is True
    _build(node, monkeypatch=monkeypatch)
    assert node._release_is_tilted(node._toss_commanded_release()) is True


# ── Invalid / absent handling on the reload service ───────────────────────────

def test_an_invalid_file_keeps_the_previous_map(monkeypatch, tmp_path):
    """All or nothing, and the PREVIOUS map survives: a half-trusted aim map is
    indistinguishable at the machine from a correct one until a ball misses."""
    path = tmp_path / 'toss_calibration.yaml'
    good = _cal_doc(math.radians(0.4), 0.0)
    path.write_text(yaml.safe_dump(good, sort_keys=False))
    monkeypatch.setattr(toss_cal, 'resolve_toss_cal_path', lambda *a: str(path))
    monkeypatch.setattr(toss_cal, 'toss_cal_candidates', lambda *a: (str(path),))
    node = ReloadCoordinatorNode()
    with node._lock:
        node._tilt_map_version = _TILT_V
    node._load_toss_cal()
    assert _status(node)['toss_cal_applied'] is True
    version = _status(node)['toss_cal_version']

    path.write_text('version: 1\nunits: {aim: mm}\n')
    ok, message = node._load_toss_cal()
    assert ok is False
    assert 'REJECTED' in message
    assert _status(node)['toss_cal_version'] == version
    assert _status(node)['toss_cal_applied'] is True


def test_a_removed_file_unloads_the_map(monkeypatch, tmp_path):
    """Reload's contract is "make this node's map agree with the FILE". Keeping
    a stale in-memory map after the operator deliberately removed it would
    defeat ``--force-uninstall``, silently, in the one workflow that most needs
    it: a capture run with a map loaded bakes the map into its own successor."""
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(math.radians(0.4), 0.0))
    assert _status(node)['toss_cal_loaded'] is True
    monkeypatch.setattr(toss_cal, 'resolve_toss_cal_path', lambda *a: None)
    ok, _ = node._load_toss_cal()
    assert ok is True
    assert _status(node)['toss_cal_loaded'] is False
    assert _status(node)['toss_cal_version'] == ''


def test_the_reload_service_is_a_trigger_and_reads_back_the_version(monkeypatch,
                                                                    tmp_path):
    """``std_srvs/Trigger`` so adding it needs no jugglebot_interfaces rebuild
    (D10). The response carries the loaded version + the applied verdict — the
    hard guarantee that the node loaded the file the phase-2c tool wrote."""
    from std_srvs.srv import Trigger
    doc = _cal_doc(math.radians(0.4), 0.0)
    node = _node_with_map(monkeypatch, tmp_path, doc)
    svc = node._services['toss/reload_calibration']
    assert svc.srv_type is Trigger

    class _Resp:
        success = None
        message = ''
    resp = node._svc_reload_toss_calibration(object(), _Resp())
    assert resp.success is True
    assert toss_cal.map_version(doc) in resp.message
    assert 'applied=True' in resp.message


def test_the_status_topic_is_latched_and_a_plain_string(monkeypatch, tmp_path):
    """A typed status field would force a jugglebot_interfaces rebuild, and a
    partial two-package colcon build takes down every ball-op action (D10). It
    could not live on TrajectoryStatus in any case: trajectory_node publishes
    that message and does not own this map, so it cannot know whether it is
    applied. TRANSIENT_LOCAL because the map changes only on load and reload —
    an unlatched topic would be silent exactly when the operator asks."""
    from std_msgs.msg import String
    node = _node_with_map(monkeypatch, tmp_path, None)
    pub = node._publishers['toss/calibration_status']
    assert pub.msg_type is String
    assert json.loads(pub.published[-1].data)['estimator_version'] == \
        toss_cal.ESTIMATOR_VERSION


# ── Tier 8b keeps working, and composes ───────────────────────────────────────

def test_tier_8b_is_unchanged_with_no_map(monkeypatch, tmp_path):
    """The re-key must not perturb the displaced toss: 8b's own tilt is
    non-zero, so it takes exactly the same branches it did before 2b."""
    node = _node_with_map(monkeypatch, tmp_path, None)
    with node._lock:
        node._commanded_pos_mm = (0.0, 0.0, 170.0)
        node._commanded_pos_mono = 1e9
    monkeypatch.setattr(node, '_live_commanded_position', lambda _now: (0.0, 0.0, 170.0))
    seq = _build(node, pose=(100.0, 0.0, 170.0), tier=TIER_8B,
                 monkeypatch=monkeypatch)
    expected = tr.compute_release_state_tilted((100.0, 0.0, 170.0), 0.8,
                                               throw_site_xy_mm=(0.0, 0.0))
    cmd = node._toss_commanded_release()
    assert cmd is node._toss_committed.release_state
    assert cmd.tilt_rx == expected.tilt_rx and cmd.tilt_ry == expected.tilt_ry
    assert node._release_is_tilted(cmd) is True
    _prepare_tick(node, seq, 100.0)
    assert [m.data for m in
            node._publishers['catch/pretilt_hold'].published] == [True]


def test_tier_8b_composes_the_aim_on_top_of_its_displacement(monkeypatch,
                                                             tmp_path):
    """The aim is a property of the PLANT, not of the tier, so it applies
    uniformly. The composition is in DISPLACEMENT (the virtual target moves by
    the aim's own ballistic offset), which is the physically additive quantity —
    lateral displacements add, tilts only approximately do."""
    aim = math.radians(0.5)
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(aim, 0.0))
    monkeypatch.setattr(node, '_live_commanded_position', lambda _now: (0.0, 0.0, 170.0))
    _build(node, pose=(100.0, 0.0, 170.0), tier=TIER_8B, monkeypatch=monkeypatch)
    plain = tr.compute_release_state_tilted((100.0, 0.0, 170.0), 0.8,
                                            throw_site_xy_mm=(0.0, 0.0))
    cmd = node._toss_commanded_release()
    # The announcement half is the UNCORRECTED displaced state (D4).
    assert np.array_equal(node._toss_committed.release_state.catch_point_global_mm,
                          plain.catch_point_global_mm)
    # The commanded half carries the extra aim, and it is additive in the
    # target, so the extra tilt is close to (not exactly) the map's aim.
    assert cmd.tilt_rx > plain.tilt_rx
    assert (cmd.tilt_rx - plain.tilt_rx) == pytest.approx(aim, rel=0.05)
