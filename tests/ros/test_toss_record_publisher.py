"""The `/toss/record` declaration — the node half of the per-toss record.

Node under test: ``reload_coordinator_node._publish_toss_record`` and friends.
Schema: ``jugglebot/toss_record.py``. Plan:
``plans/active/toss-selftuning.md`` §§ 3.3, 3.4, D10.
Logbook: ``logbook/2026-08-10-toss-selftuning-build.md`` § Phase 2a.

THE PROPERTY THESE TESTS PROTECT
--------------------------------
This is an **instrument with zero control authority**. Its whole value depends on
being unable to affect the cycle it observes — the node it lives in owns the hand,
the catch latch and the abort ladder, and a teardown that stalls because a disk
filled up is strictly worse than a lost measurement. So the tests below are less
about "does it record the right thing" (``tests/motion/test_toss_record.py`` owns
that) and more about **what happens when it fails**: an unencodable record, a
publish that raises, a belt that cannot write. All three must leave the cycle
untouched and cost at most one WARN.

The second property is census integrity. Exactly one record per cycle terminal,
including ``REJECTED_BAD_GOAL`` — and that row must carry ITS own identity, not
the previous goal's, or a corpus's rejected rows silently attribute themselves to
the wrong toss.
"""

from __future__ import annotations

import json
import os

import pytest

from jugglebot import toss_record as tr
from jugglebot.reload_coordinator_node import ReloadCoordinatorNode
import jugglebot.reload_coordinator_node as rcn
from jugglebot.toss_sequencer import TossResult


def _node():
    return ReloadCoordinatorNode()


def _open(node, **kw):
    kw.setdefault('action', 'toss')
    kw.setdefault('goal_id', 'deadbeefcafef00d')
    kw.setdefault('cycle_index', 1)
    kw.setdefault('catch_pose', (0.0, 150.0, 170.0))
    kw.setdefault('throw_delay', 5.0)
    kw.setdefault('vel_scale', 0.9)
    kw.setdefault('raw_goal', {'throw_height_m': 0.0, 'throw_delay_s': 0.0,
                               'catch_vel_scale': 0.0})
    # `flight` present == a cycle was built. The bad-goal path opens the context
    # WITHOUT it, and the node uses that to refuse to attribute the previous
    # goal's resolved state to this rejection — see _open_bad_goal below.
    kw.setdefault('flight', 0.8)
    node._open_toss_record(**kw)
    with node._lock:
        node._catch_vel_scale = float(kw['vel_scale'])


def _open_bad_goal(node, **kw):
    """The REJECTED_BAD_GOAL context: opened before the numerics gate, so no
    flight time was ever resolved and no cycle was ever built."""
    kw['flight'] = None
    _open(node, **kw)


def _records(node):
    pub = node._publishers['toss/record']
    return [tr.decode(m.data) for m in pub.published]


# ── The wire ──────────────────────────────────────────────────────────────────

def test_the_topic_is_published_as_a_plain_string():
    """``std_msgs/String`` carrying JSON, deliberately NOT a typed message: a
    schema tweak on a typed message needs a two-package colcon build, which is
    exactly the partial-build ImportError class the runbook's build gate exists
    to prevent (D10)."""
    from std_msgs.msg import String
    node = _node()
    pub = node._publishers['toss/record']
    assert pub.msg_type is String


def test_one_record_per_terminal_and_it_validates():
    node = _node()
    _open(node)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    rows = _records(node)
    assert len(rows) == 1
    assert tr.validate(rows[0]) == ()
    assert rows[0]['schema'] == tr.SCHEMA
    assert rows[0]['outcome'] == 'CAUGHT'
    assert rows[0]['success'] is True


def test_the_record_rides_the_single_authoritative_outcome_line():
    """``_log_toss_outcome`` is documented as "exactly one per goal". Hooking the
    record there rather than at each caller is what makes the census complete
    without a second list of terminals to keep in sync."""
    node = _node()
    _open(node)
    for outcome in ('CAUGHT', 'MISSED', 'ABORTED_TIMEOUT'):
        node._log_toss_outcome(TossResult(outcome == 'CAUGHT', outcome))
    assert [r['outcome'] for r in _records(node)] == [
        'CAUGHT', 'MISSED', 'ABORTED_TIMEOUT']


def test_a_nan_diagnostic_encodes_as_null():
    """``catch_error_mm`` is NaN on every non-caught toss. JSON has no NaN, and a
    bare ``NaN`` token is not portable JSON — the encoder maps it to null, and
    ``allow_nan=False`` turns a miss into a hard error rather than a corpus
    nobody can parse."""
    node = _node()
    _open(node)
    node._log_toss_outcome(TossResult(False, 'MISSED'))
    payload = node._publishers['toss/record'].published[0].data
    assert 'NaN' not in payload
    assert json.loads(payload)['catch_error_mm_fsm'] is None


# ── Census integrity ──────────────────────────────────────────────────────────

def test_a_rejected_bad_goal_declares_its_OWN_identity():
    """The bug this shape prevents: ``_execute_toss`` logs the bad-goal terminal
    BEFORE building the cycle, so a context opened only after the build would let
    a rejected goal inherit the PREVIOUS goal's ``toss_uid``. A census whose
    rejected rows are attributed to the wrong toss is worse than one with no
    rejected rows."""
    node = _node()
    _open(node, goal_id='1111111111111111', cycle_index=1)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    _open_bad_goal(node, goal_id='2222222222222222', cycle_index=1)
    node._log_toss_outcome(TossResult(False, 'REJECTED_BAD_GOAL(throw_height_m)'))
    rows = _records(node)
    assert rows[0]['goal_id'] == '1111111111111111'
    assert rows[1]['goal_id'] == '2222222222222222'
    assert rows[0]['toss_uid'] != rows[1]['toss_uid']


def test_a_rejected_bad_goal_records_NO_resolved_state():
    """The subtler half of the same bug. A bad goal is refused BEFORE the cycle
    is built, so no flight time, no release state and no catch-vel scale were
    ever resolved — but the node's per-goal caches still hold the PREVIOUS
    cycle's. Reading them would attribute the last toss's physics to this
    rejection, and it would look completely plausible in the corpus.

    Raw goal fields survive, because those are what the operator actually asked
    for and they are the only honest thing a rejection can report."""
    node = _node()
    _open(node, goal_id='1111111111111111', vel_scale=0.9)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    _open_bad_goal(node, goal_id='2222222222222222', vel_scale=-1.0,
                   raw_goal={'throw_height_m': float('nan'),
                             'throw_delay_s': 5.0, 'catch_vel_scale': -1.0})
    node._log_toss_outcome(TossResult(False, 'REJECTED_BAD_GOAL(throw_height_m)'))
    bad = _records(node)[1]
    assert bad['flight_time_s'] is None
    assert bad['event_vel_mps'] is None
    assert bad['catch_point_global_mm'] is None
    assert bad['goal_catch_vel_scale'] is None
    assert bad['catch_knobs']['catch_vel_scale'] is None
    # ...but the request itself is on the record.
    assert bad['goal_throw_delay_s_raw'] == pytest.approx(5.0)
    assert bad['goal_throw_height_m_raw'] is None      # NaN encodes as null
    assert bad['outcome'] == 'REJECTED_BAD_GOAL(throw_height_m)'


def test_the_uid_is_session_goal_cycle():
    node = _node()
    _open(node, goal_id='abcdef0123456789', cycle_index=7)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    row = _records(node)[0]
    assert row['toss_uid'] == '{}-abcdef01-7'.format(node._session_id)
    assert row['session_id'] == node._session_id
    assert row['cycle_index'] == 7


def test_opening_a_new_cycle_clears_the_previous_announcement():
    """The join key is per-CYCLE. A stale ``announce_throw_time_ros`` surviving
    into the next cycle would join that cycle's mined half to the previous
    cycle's declaration — silently, and within tolerance."""
    node = _node()
    _open(node)
    with node._lock:
        node._toss_record_announce = (1000.0, 1000.8)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    _open(node, cycle_index=2)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    rows = _records(node)
    assert rows[0]['announce_throw_time_ros'] == pytest.approx(1000.0)
    assert rows[1]['announce_throw_time_ros'] is None


# ── Phase 2a applies NOTHING, and says so ─────────────────────────────────────

def test_the_calibration_block_is_explicitly_zero_not_absent():
    """A null would read as "unknown"; zero reads as "no calibration layer
    exists yet". The corpus must be able to prove that about its own baseline —
    2b's map and 2e's trim are scored against these rows."""
    node = _node()
    _open(node)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    row = _records(node)[0]
    assert row['map_aim_rad'] == [0.0, 0.0]
    assert row['trim_aim_rad'] == [0.0, 0.0]
    assert row['total_aim_rad'] == [0.0, 0.0]
    assert row['toss_cal_loaded'] is False
    assert row['toss_cal_applied'] is False


def test_the_layer_1_5_block_degrades_to_an_empty_read_set():
    """Schema in 2a, reads in 2d. A toss with NO dwell reads — a single ``Toss``,
    which has no dwell at all, or a session dwell too tight for one read — records
    ``dwell_tilt_n = 0`` and nulls for the statistics. That is a LEGAL record for
    good: § 3.10's degrade-never-delay rule makes it the routine outcome at the
    shipped 6.0 s cadence, so analysers must tolerate it rather than treat it as
    corruption.

    ``dwell_tilt_n`` and ``dwell_tilt_degraded`` are NOT null, because "how many
    reads did this toss get?" has a definite answer for every toss — and a null
    there would let a fit silently treat "no reads" as "not measured yet"."""
    node = _node()
    _open(node)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    row = _records(node)[0]
    assert row['dwell_tilt_n'] == 0
    assert row['dwell_tilt_degraded'] is False
    for name in ('dwell_tilt_rad', 'dwell_tilt_sd_rad', 'dwell_tilt_span_s',
                 'dwell_tilt_last_read_to_release_s'):
        assert row[name] is None, name


def test_both_clock_offsets_are_carried():
    """``_announcement_landing_perf`` documents an OPEN reconciliation question:
    the FSM uses a single instantaneous ``perf - ros`` read while the rest of the
    stack uses the median-filtered one. Carrying both numbers on every record
    turns that argument into a measurement — and the FSM keeps its own read, so
    nothing about the toss changes."""
    node = _node()
    _open(node)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    row = _records(node)[0]
    assert row['perf_minus_ros_s'] is not None
    assert row['perf_minus_ros_inst_s'] is not None


def test_the_catch_knobs_are_the_gains_the_toss_installs():
    """Recording the ODrive defaults here would be a record of numbers the
    machine was not running: the toss installs the SOFT-CATCH set at PREPARE."""
    node = _node()
    _open(node, vel_scale=0.9)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    knobs = _records(node)[0]['catch_knobs']
    assert knobs['catch_vel_scale'] == pytest.approx(0.9)
    assert knobs['hand_pos_gain'] == pytest.approx(
        rcn._TOSS_SOFT_CATCH_GAINS['pos_gain'])
    assert knobs['hand_vel_gain'] == pytest.approx(
        rcn._TOSS_SOFT_CATCH_GAINS['vel_gain'])


# ── Failure is silent, always ─────────────────────────────────────────────────

def test_an_unencodable_record_costs_one_warn_and_nothing_else():
    """The instrument must never be able to raise into a terminal path. The
    cycle has already finished; the only correct behaviour is to say so and
    return."""
    node = _node()
    _open(node)
    node._toss_record_fields = lambda result: (_ for _ in ()).throw(
        ValueError('boom'))
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))     # must not raise
    assert _records(node) == []


def test_a_publish_that_raises_does_not_escape():
    node = _node()
    _open(node)

    def _boom(msg):
        raise RuntimeError('rmw is unhappy')

    node._publishers['toss/record'].publish = _boom
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))     # must not raise


def test_a_belt_that_cannot_write_warns_once_per_goal_then_goes_quiet(monkeypatch):
    """A full disk is a lost measurement, never a stalled abort ladder — and a
    per-cycle WARN storm during a 72-toss capture is its own denial of service on
    the operator's attention."""
    node = _node()
    _open(node)

    def _boom(*a, **kw):
        raise OSError('no space left on device')

    monkeypatch.setattr(rcn.os, 'makedirs', _boom)
    warnings = []
    node.get_logger().warning = warnings.append
    for _ in range(5):
        node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    # The belt write runs on the record WORKER since 2026-08-22 (census B6), so
    # the assertion has to wait for it. The drain is the CONTRACT — every caller
    # that reads what the worker produces drains first — and this test is one of
    # them, standing in for the operator reading the file afterwards.
    assert node._toss_records_drain()
    assert len(warnings) == 1
    assert 'belt' in warnings[0]
    # The declarations still went out: the bag is the canonical sink and the
    # belt is the fallback, not the other way round.
    assert len(_records(node)) == 5


def test_the_belt_writes_the_same_bytes_as_the_topic(monkeypatch, tmp_path):
    """Same encoder, same line — so a belt row and a bag row are the same record
    and a corpus can be assembled from either without a second parser."""
    node = _node()
    monkeypatch.setattr(rcn, '_RECORD_BELT_DIR', str(tmp_path))
    _open(node)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    assert node._toss_records_drain()          # census B6: the belt is off-thread
    path = os.path.join(str(tmp_path),
                        'toss_records_{}.jsonl'.format(node._session_id))
    with open(path) as fh:
        lines = [ln.rstrip('\n') for ln in fh if ln.strip()]
    assert lines == [node._publishers['toss/record'].published[0].data]


def test_the_belt_is_skipped_outside_a_repo_checkout(monkeypatch):
    """``_RECORD_BELT_DIR`` is None for a deployment outside the repo. The belt
    simply does not run; it must not invent a path."""
    node = _node()
    monkeypatch.setattr(rcn, '_RECORD_BELT_DIR', None)
    called = []
    monkeypatch.setattr(rcn.os, 'makedirs',
                        lambda *a, **kw: called.append(a))
    node._belt_toss_record('{}')
    assert called == []


def test_the_belt_dir_is_resolved_by_marker_not_by_a_fixed_walk(
        real_record_belt_dir):
    """A fixed number of ``dirname`` hops cannot be right in both trees: this
    module runs from ``ros_ws/src`` under pytest and from the colcon install tree
    in production. That is the tilt-cal Phase-2 finding, which produced a path
    that could not exist in production while looking fine in the source tree.

    Reads ``real_record_belt_dir`` — the PRODUCTION-resolved constant captured
    before ``tests/ros/conftest.py``'s autouse isolation fixture redirects the
    sink into a tmp dir. Asserting on ``rcn._RECORD_BELT_DIR`` directly would
    pass vacuously against whatever the fixture chose.
    """
    assert rcn._REPO_ROOT is not None, 'running inside the repo checkout'
    assert os.path.isdir(os.path.join(rcn._REPO_ROOT, 'ros_ws'))
    assert os.path.exists(os.path.join(rcn._REPO_ROOT, 'config',
                                       'hardware_config.yaml'))
    assert real_record_belt_dir == os.path.join(rcn._REPO_ROOT, 'temp', 'logs')
    assert real_record_belt_dir.endswith(os.path.join('temp', 'logs'))
    # The isolation fixture really did redirect the live constant, so no test in
    # this suite can write a production-shaped artefact into temp/logs.
    assert rcn._RECORD_BELT_DIR != real_record_belt_dir


# ── Provenance ────────────────────────────────────────────────────────────────

def test_the_tilt_map_identity_travels_with_every_toss():
    """D3/G5: an aim residual fitted under tilt map A double-counts tilt map B's
    delta, so the map identity has to be ON the record, not looked up later."""
    node = _node()
    with node._lock:
        node._tilt_map_loaded = True
        node._tilt_map_version = '2026-08-10-3bf7964f'
        node._gravity_correction_loaded = True
    _open(node)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    row = _records(node)[0]
    assert row['tilt_map_version'] == '2026-08-10-3bf7964f'
    assert row['tilt_map_applied'] is True


def test_tilt_map_applied_needs_BOTH_flags():
    """``tilt_map_loaded`` alone is not "applied" — the correction only reaches
    the plant while ``gravity_correction_loaded`` is also true. Recording the
    conjunction is the loaded-vs-applied distinction the tilt-cal review had to
    go back and fix in four documents."""
    node = _node()
    with node._lock:
        node._tilt_map_loaded = True
        node._gravity_correction_loaded = False
    _open(node)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    assert _records(node)[0]['tilt_map_applied'] is False


def test_uptime_and_firmware_are_left_to_the_miner():
    """A deliberate 2a deviation from the schema's ``D+M``: the coordinator does
    not subscribe to ``/link_status``, and adding a subscription to the node that
    owns the hand, the latch and the abort ladder to obtain a pure covariate is
    the surface D10 argues to keep out. ``/link_status`` is in the bag at 5 Hz,
    so the miner recovers the number to ~200 ms."""
    node = _node()
    _open(node)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    row = _records(node)[0]
    assert row['uptime_ms_at_release'] is None
    assert row['bridge_fw_version'] is None
    assert 'link_status' not in node._subscriptions


# ══ 2d — the session-policy and exclusion fields ═════════════════════════════

def test_the_resolved_reload_policy_is_recorded_not_the_raw_field():
    """RESOLVED, never raw: ``on_empty_cup`` has already been through the
    whitelist and ``max_reloads`` through the config default, so the corpus
    records the policy the machine RAN rather than the string the operator typed.
    A corpus that stored the raw field could not tell a session that reloaded
    from one whose typo silently resolved to STOP."""
    from jugglebot.toss_session import ON_EMPTY_CUP_RELOAD, TossSessionSequencer
    node = _node()
    session = TossSessionSequencer(num_throws=3, dwell_time_s=8.0,
                                   throw_delay_s=5.0,
                                   on_empty_cup=ON_EMPTY_CUP_RELOAD,
                                   max_reloads=3)
    _open(node, action='toss_continuous', session=session, cycle_index=2)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    row = _records(node)[0]
    assert row['goal_on_empty_cup'] == 'RELOAD'
    assert row['goal_max_reloads'] == 3
    # G10 is an explicit boolean on every session cycle — "was this cycle after a
    # reload?" has a definite answer, and a null would let a fit include it.
    assert row['reload_settle'] is False
    assert row['retry_of'] is None


def test_the_two_exclusion_flags_reach_the_record():
    from jugglebot.toss_session import TossSessionSequencer
    node = _node()
    session = TossSessionSequencer(num_throws=3, dwell_time_s=8.0,
                                   throw_delay_s=5.0)
    _open(node, action='toss_continuous', session=session, cycle_index=1)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    _open(node, action='toss_continuous', session=session, cycle_index=2,
          reload_settle=True, retry=True)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    row = _records(node)[1]
    assert row['reload_settle'] is True
    assert row['retry_of'] == _records(node)[0]['toss_uid']


def test_a_single_toss_carries_no_session_policy_fields():
    """A plain ``Toss`` has no session, so the reload policy has no referent —
    and a null is the honest value, not a default nobody chose."""
    node = _node()
    _open(node)
    node._log_toss_outcome(TossResult(True, 'CAUGHT'))
    row = _records(node)[0]
    assert row['goal_on_empty_cup'] is None
    assert row['goal_max_reloads'] is None
    assert row['reload_settle'] is None
