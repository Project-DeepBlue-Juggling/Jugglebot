"""Node seams for the LAYER-2 session trim — build phase **2e** of
``plans/active/toss-selftuning.md``.

The estimator itself is covered in ``tests/motion/test_toss_trim.py`` (pure, no
ROS). Here we test the six things only the NODE can be asked:

1. **``toss_trim_enabled`` defaults FALSE**, and a disabled trim leaves the
   commanded release the SAME OBJECT as the announcement release — i.e. phase
   2b's bitwise-identity gate still holds with 2e landed, which is the "zero
   trim ⇒ bitwise today" acceptance criterion.
2. **Layer 2's aim has ZERO authority** over the commanded aim
   (``toss_trim.AIM_AUTHORITY``, owner decision 1 of the 2026-08-21 ILC-primary
   fold-in, contradiction C4) — and its estimate is still reported, under its
   own key. D7's *re-clamped at apply* rule is untouched and now arbitrates
   ``map + ilc``; a BINDING clamp is exercised in ``test_toss_ilc_node.py``.
3. **``catch/pretilt_hold`` is raised UNCONDITIONALLY** — since 2026-08-22
   (census E5) it is no longer keyed on the aim at all: at the cadence rungs
   CCN's pre-tilt arrival clamps to the landing itself, so even a level 8a would
   command a reach arriving at contact. The tests here therefore assert the
   COMMANDED RELEASE STATE directly (``_release_is_tilted``) rather than using
   the hold as a proxy for it — the hold stopped being evidence about the aim.
4. **The trim is read exactly ONCE per goal**, in ``_build_toss_cycle``, and
   observed from exactly one place — the D4 manifest shape, applied to layer 2.
5. **The record carries what was COMMANDED and what was KNOWN separately**, so a
   corpus can tell "no bias" from "no permission".
6. **The proposal is written at goal end and is not a calibration** (premise P1).

ROS 2 is mocked by ``tests/ros/conftest.py``; the map fixture and the
monkeypatch seams are ``test_toss_calibration.py``'s, deliberately reused.
"""

from __future__ import annotations

import ast
import math
import os
import sys

import numpy as np
import pytest
import yaml

import jugglebot.hardware_config as hw
from jugglebot import toss_trim
from jugglebot.motion import toss_cal
from jugglebot.motion.trajectory import toss_release as tr
from jugglebot.reload_coordinator_node import ReloadCoordinatorNode
from jugglebot.toss_sequencer import TIER_8A

# The map fixture and the monkeypatch seams are 2b's, reused DELIBERATELY: two
# copies of "a node whose aim map resolves to this document" is two places for a
# fixture to drift from the loader it is standing in for.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_toss_calibration import _cal_doc, _node_with_map   # noqa: E402,F401

_POSE = (0.0, 150.0, 170.0)
_FLIGHT = 0.8
import jugglebot.reload_coordinator_node as _rcn                # noqa: E402
_PKG_DIR = os.path.dirname(os.path.abspath(_rcn.__file__))


def _build(node, monkeypatch, pose=_POSE, flight=_FLIGHT, tier=TIER_8A):
    monkeypatch.setattr(hw, 'JB_OP_TOSS_TIER', tier)
    return node._build_toss_cycle(pose, flight, 5.0, 0.9)


class _StubTrim:
    """A trim with a KNOWN commanded value — the estimator is not under test
    here, the seam is. Mirrors the real class's read surface exactly."""

    def __init__(self, aim=(0.0, 0.0)):
        self._aim = (float(aim[0]), float(aim[1]))
        self.reads = 0
        self.observed = []

    def aim(self):
        self.reads += 1
        return self._aim

    def observe(self, rec):
        self.observed.append(rec)
        return toss_trim.AimObservation('uid', False, 'no_mocap_fit', None, None)

    def snapshot(self):
        return {'n': 3, 'state': 'ACTIVE', 'reason': 'because',
                'speed_k_v': 1.02, 'tau_ms': 25.0}

    def console_lines(self, **kwargs):
        return ['TRIM stub']

    def proposal(self, **meta):
        doc = {'schema': toss_trim.TRIM_SCHEMA, 'trim': {'c_rad': list(self._aim)}}
        doc.update({str(k): v for k, v in meta.items()})
        return doc


# ── 1. default OFF, and the identity path ─────────────────────────────────────

def test_the_trim_parameter_defaults_false(monkeypatch, tmp_path):
    """DEFAULT FALSE, deliberately. The trim costs ~1 mm of aim when there is
    nothing to correct (measured — ``toss_trim.SE_GATE``), it has never been
    validated on hardware, and its measurement channel is not live yet. It still
    LEARNS while disabled; only its output is withheld."""
    node = _node_with_map(monkeypatch, tmp_path, None)
    assert node.get_parameter('toss_trim_enabled').value is False


def test_a_disabled_trim_leaves_the_release_state_untouched(monkeypatch,
                                                            tmp_path):
    """THE 2e acceptance gate: zero trim ⇒ today's machine, bit for bit.

    Composes with 2b's identity gate rather than restating it — the COMMANDED
    release is the same OBJECT as the announcement release, so a disabled trim
    costs not one floating-point operation. The stub commands a real trim, so
    this proves the PARAMETER is what withholds it, not an accidentally empty
    estimator."""
    node = _node_with_map(monkeypatch, tmp_path, None)
    node._toss_trim = _StubTrim((toss_trim.TRIM_MAX_RAD, 0.0))
    _build(node, monkeypatch)
    with node._lock:
        assert node._toss_release_cmd is node._toss_release_state
        assert node._toss_aim['aim_rad'] == (0.0, 0.0)
        assert node._toss_aim['trim_aim_rad'] == (0.0, 0.0)
        assert node._toss_aim['trim_enabled'] is False
    assert node._publishers['catch/pretilt_hold'].published == []
    expected = tr.compute_release_state(_POSE, _FLIGHT)
    got = node._toss_release_state
    assert np.array_equal(got.launch_vel_mms, expected.launch_vel_mms)
    assert got.event_vel_mps == expected.event_vel_mps


def test_an_enabled_but_warming_up_trim_is_still_the_identity_path(monkeypatch,
                                                                   tmp_path):
    """A REAL trim that has learnt nothing commands exactly zero, and zero must
    take the identity branch — which is the state every live session is in until
    a mined arrival lands in the record (``toss_trim``'s module docstring)."""
    node = _node_with_map(monkeypatch, tmp_path, None)
    _enable_trim(node)
    node._toss_trim_begin(goal_id='abc')
    _build(node, monkeypatch)
    with node._lock:
        assert node._toss_release_cmd is node._toss_release_state
        assert node._toss_aim['trim_enabled'] is True
        assert node._toss_aim['aim_rad'] == (0.0, 0.0)


def _enable_trim(node, value=True):
    """Set ``toss_trim_enabled`` through the mocked parameter store.

    ``tests/ros/conftest.py``'s ``MockNode`` implements ``declare_parameter`` /
    ``get_parameter`` over a plain dict and has no ``set_parameters``; writing
    the dict directly is what every other parameterised node test here does, and
    it exercises the SAME read path the node uses (``get_parameter(...).value``).
    """
    node._params['toss_trim_enabled'] = bool(value)


# ── 2. the TOTAL clamp, at apply ──────────────────────────────────────────────

def test_the_layer_2_aim_has_ZERO_authority_over_the_commanded_aim(monkeypatch,
                                                                   tmp_path):
    """**C4 / owner decision 1, 2026-08-21: the trim's aim is MONITOR-ONLY.**

    Its predecessor —
    ``test_the_total_is_reclamped_at_apply_over_map_plus_trim`` — pinned the
    opposite: a map saturated at 1.0° PLUS a trim saturated at 0.15° summed to
    1.15° and the D7 apply-time clamp cut it back. That composition is gone,
    because two converging estimators of one quantity double-count in both
    directions (the machine over-aims by the ILC's contribution while the trim
    reports CONVERGED; and a converged trim makes the ILC unlearn itself to
    zero). D7's *re-clamped AT APPLY* rule is untouched and is still the single
    enforcement point — it now arbitrates ``map + ilc``, and
    ``tests/ros/test_toss_ilc_node.py`` is where a BINDING total clamp is
    exercised.

    So the strongest available statement is this one: a trim saturated at its own
    authority moves the commanded aim by exactly nothing, and does not even reach
    the clamp.
    """
    node = _node_with_map(monkeypatch, tmp_path,
                          _cal_doc(toss_cal.TOTAL_MAX_RAD, 0.0))
    _enable_trim(node)
    node._toss_trim = _StubTrim((toss_trim.TRIM_MAX_RAD, 0.0))
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    # The commanded total is the MAP alone, to the last bit — a saturated trim
    # did not push it into the clamp because it was never added.
    assert aim['aim_rad'][0] == pytest.approx(toss_cal.TOTAL_MAX_RAD, rel=1e-12)
    assert aim['clamp_hits'] == []
    assert aim['trim_aim_rad'] == (0.0, 0.0)
    # ... and the estimate is still REPORTED, under its own key. Zero authority,
    # full visibility: this is the number whose divergence from the ILC's is the
    # standing validation of the C3 by-decision resolution.
    assert aim['trim_monitor_aim_rad'][0] == pytest.approx(
        toss_trim.TRIM_MAX_RAD)
    assert aim['trim_authority'] == toss_trim.AIM_AUTHORITY == 'MONITOR'


def test_the_trims_mm_report_is_the_difference_of_the_commanded_offsets(
        monkeypatch, tmp_path):
    """``trim_aim_mm_at_h`` is ``offset(total) − offset(map)``, not
    ``offset(trim)`` computed on its own: what the trim MOVED is the virtual
    target, and reporting the independent value would credit it with a
    displacement the machine did not command.

    Under monitor-only (C4) that difference is **structurally** zero rather than
    incidentally zero, which is the strongest form of the property this test was
    always about — and it is worth keeping precisely because a future edit that
    "helpfully" reported ``offset(trim_monitor_aim_rad)`` here would put a
    never-commanded displacement into the corpus. The same difference-of-offsets
    rule one layer further out (total − (map+trim)) is what layer 3's report
    uses, and ``tests/ros/test_toss_ilc_node.py::
    test_the_three_layers_sum_and_the_offsets_split_cleanly`` exercises it with a
    non-zero value."""
    node = _node_with_map(monkeypatch, tmp_path,
                          _cal_doc(toss_cal.TOTAL_MAX_RAD, 0.0))
    _enable_trim(node)
    node._toss_trim = _StubTrim((toss_trim.TRIM_MAX_RAD, 0.0))
    _build(node, monkeypatch)
    with node._lock:
        aim = dict(node._toss_aim)
    # The total is clamped back onto the map's own value, so the trim moved the
    # target by nothing at all — and says so.
    assert aim['trim_offset_mm'][0] == pytest.approx(0.0, abs=1e-9)
    assert aim['trim_offset_mm'][1] == pytest.approx(0.0, abs=1e-9)
    # ... while an UNCLAMPED trim reports a real displacement.
    node2 = _node_with_map(monkeypatch, tmp_path, _cal_doc(0.0, 0.0))
    _enable_trim(node2)
    node2._toss_trim = _StubTrim((toss_trim.TRIM_MAX_RAD, 0.0))
    _build(node2, monkeypatch)
    with node2._lock:
        aim2 = dict(node2._toss_aim)
    want = tr.aim_target_offset_mm(toss_trim.TRIM_MAX_RAD, 0.0, _FLIGHT,
                                   _POSE[2])
    assert aim2['trim_offset_mm'][0] == pytest.approx(float(want[0]), rel=1e-9)


def test_the_virtual_target_is_NOT_moved_by_the_monitor_only_trim(monkeypatch,
                                                                  tmp_path):
    """The mirror of the old ``..._is_built_from_the_TOTAL_aim``, and the more
    important direction now.

    That test's root cause — *a layer reported as applied but never actually
    flown is the silent-wrong class D3 exists to close* — is exactly why the
    demotion had to change the RECORD as well as the composition. A build that
    left ``trim_aim_rad`` carrying the estimate while quietly not composing it
    would be that silent-wrong, inverted. So: a live trim with no map at all
    leaves the commanded release the SAME OBJECT as the announcement release,
    and the record says the trim commanded nothing.
    """
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(0.0, 0.0))
    _enable_trim(node)
    trim_aim = (math.radians(0.12), math.radians(-0.05))
    node._toss_trim = _StubTrim(trim_aim)
    _build(node, monkeypatch)
    with node._lock:
        cmd = node._toss_release_cmd
        base = node._toss_release_state
        aim = dict(node._toss_aim)
    assert cmd is base                     # not one floating-point operation
    assert aim['aim_rad'] == (0.0, 0.0)
    assert aim['trim_aim_rad'] == (0.0, 0.0)
    assert aim['trim_monitor_aim_rad'] == pytest.approx(trim_aim)


# ── 3. pretilt_hold for any NON-ZERO COMMANDED aim (D3) ───────────────────────

def test_pretilt_hold_is_raised_for_a_map_only_aim(monkeypatch, tmp_path):
    """D3 is about ANY non-zero **commanded** aim: without the hold, the stock
    announcement pre-tilt completes an un-tilt to level ≥ 1 s before release and
    reverts the aim while every log line reports it as applied.

    Keyed on the MAP since 2026-08-21. The predecessor
    (``test_pretilt_hold_is_raised_for_a_trim_only_aim``) used the session trim
    as its non-zero source, which no longer commands anything — so the test is
    re-pointed at a layer that does rather than deleted, and the companion below
    asserts the trim's own silence.
    """
    from jugglebot.toss_sequencer import ACTION_PREPARE_CATCH, TossDecision
    node = _node_with_map(monkeypatch, tmp_path,
                          _cal_doc(math.radians(0.12), 0.0))
    seq = _build(node, monkeypatch)
    assert node._release_is_tilted(node._toss_commanded_release()) is True
    decision = TossDecision(done=False, phase='PREPARING',
                            action=ACTION_PREPARE_CATCH, result=None)
    node._build_toss_observations = lambda _now: None
    seq.step = lambda _now, _obs: decision
    node._step_toss_sequence(seq, 100.0)
    published = [m.data for m in node._publishers['catch/pretilt_hold'].published]
    assert published == [True]


def test_a_trim_only_aim_leaves_the_COMMANDED_release_level(monkeypatch, tmp_path):
    """The C4 demotion seen from the FSM's side: a saturated monitor-only trim
    leaves the goal on the byte-identical LEVEL path. ``_release_is_tilted`` keys
    on the commanded release state rather than on any layer's value, so this
    follows from the composition and is not a second rule that could drift.

    ⚠ **This test no longer asserts anything about ``catch/pretilt_hold``**, and
    the reason is a deliberate widening, not a regression. Until 2026-08-22 the
    hold was the OBSERVABLE for "is the commanded release tilted", because the
    two were keyed together. Census E5 unkeyed them: the hold is now raised
    UNCONDITIONALLY, because at the cadence rungs CCN's pre-tilt arrival clamps
    to the landing itself and even a level 8a would command a reach arriving at
    contact. So the hold stopped being evidence about the aim, and this test
    asserts the aim directly — which is what it was always about."""
    from jugglebot.toss_sequencer import ACTION_PREPARE_CATCH, TossDecision
    node = _node_with_map(monkeypatch, tmp_path, None)
    _enable_trim(node)
    node._toss_trim = _StubTrim((toss_trim.TRIM_MAX_RAD, 0.0))
    seq = _build(node, monkeypatch)
    assert node._release_is_tilted(node._toss_commanded_release()) is False
    decision = TossDecision(done=False, phase='PREPARING',
                            action=ACTION_PREPARE_CATCH, result=None)
    node._build_toss_observations = lambda _now: None
    seq.step = lambda _now, _obs: decision
    node._step_toss_sequence(seq, 100.0)
    # The hold IS raised — for the cadence reason (E5), not for an aim reason.
    published = [m.data for m in node._publishers['catch/pretilt_hold'].published]
    assert published == [True]


# ── 4. read ONCE per goal, observed from ONE place (D4, layer 2) ──────────────

def _calls_in_package(targets):
    """Every (file, enclosing scope, dotted call) in the package matching
    ``targets`` — the AST manifest shape ``test_toss_cal.py`` uses for D4."""
    found = []
    for root, dirs, files in os.walk(_PKG_DIR):
        dirs[:] = [d for d in dirs if d not in ('__pycache__', 'archived')]
        for fname in sorted(files):
            if not fname.endswith('.py'):
                continue
            path = os.path.join(root, fname)
            with open(path, encoding='utf-8') as handle:
                tree = ast.parse(handle.read(), filename=fname)
            for node in ast.walk(tree):
                if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    continue
                for child in ast.walk(node):
                    if not isinstance(child, ast.Call):
                        continue
                    dotted = _dotted(child.func)
                    if dotted in targets:
                        found.append((fname, node.name, dotted))
    return sorted(set(found))


def _dotted(node):
    parts = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
    return '.'.join(reversed(parts))


def test_the_trim_is_read_in_exactly_one_scope():
    """D4, one layer down. The trim is evaluated ONCE PER GOAL and nowhere else
    — not in the 40 Hz emitter, not per Hermite knot, not on the reload path.

    A second read is a perfectly working program: every behavioural assertion
    about "is the trim applied" still passes, and the only symptom is that two
    evaluations of a *mutating* estimator disagree — so the pose the platform is
    commanded to and the pose the record says it was commanded to drift apart.
    The trim mutates between cycles, which makes this stricter than the map's
    version of the same rule, not looser."""
    sites = _calls_in_package({'trim.aim'})
    assert sites == [('reload_coordinator_node.py', '_toss_aim_for_goal',
                      'trim.aim')], sites


def test_the_trim_is_fed_from_exactly_one_place():
    """One ingest point. The declaration is minted exactly once per cycle
    terminal and already carries every field the § 3.6.2 guards read, so feeding
    the estimator anywhere else would either double-count a toss or feed it a
    different object from the one the offline replay consumes.

    The SCOPE moved on 2026-08-22 (census B6) — from ``_publish_toss_record``,
    which runs on the cycle thread, to ``_toss_records_run_one``, which runs on
    the record worker. The invariant is unchanged and is if anything stronger:
    the worker is a SINGLE thread draining a FIFO, so "one ingest point" now also
    means "one update at a time" for a MUTATING estimator. What the move required
    is that the trim's context be snapshotted at SUBMIT time — see
    ``_toss_trim_snapshot`` and the test below."""
    sites = [s for s in _calls_in_package({'trim.observe'})
             if s[0] != 'toss_trim.py']       # toss_trim.replay is the OFFLINE
                                              # counterpart and feeds its own
                                              # fresh estimator, not the node's
    assert sites == [('reload_coordinator_node.py', '_toss_trim_observe',
                      'trim.observe')], sites
    callers = _calls_in_package({'self._toss_trim_observe'})
    assert callers == [('reload_coordinator_node.py', '_toss_records_run_one',
                        'self._toss_trim_observe')], callers


def test_the_aim_scope_is_still_reached_from_one_place_only():
    """The 2b invariant, re-checked with 2e landed: ``_toss_aim_for_goal`` is
    still called only from ``_build_toss_cycle``, so "once per scope" is still
    "once per goal" for BOTH layers."""
    callers = _calls_in_package({'self._toss_aim_for_goal'})
    assert callers == [('reload_coordinator_node.py', '_build_toss_cycle',
                        'self._toss_aim_for_goal')], callers


# ── 5. the record: COMMANDED vs KNOWN ─────────────────────────────────────────

def _record(node, outcome='CAUGHT', success=True):
    class _R:
        pass
    result = _R()
    result.outcome = outcome
    result.success = success
    result.catch_error_mm = float('nan')
    result.achieved_flight_s = float('nan')
    return node._toss_record_fields(result)


def test_the_record_separates_what_was_commanded_from_what_was_known(
        monkeypatch, tmp_path):
    """A corpus has to be able to tell "this session had no bias" from "this
    session had no permission". ``trim_aim_rad`` is what was COMMANDED (zero
    while the parameter is false); ``trim_state`` / ``trim_source_n`` describe
    the estimator, which keeps learning either way."""
    node = _node_with_map(monkeypatch, tmp_path, _cal_doc(0.0, 0.0))
    node._toss_trim = _StubTrim((toss_trim.TRIM_MAX_RAD, 0.0))
    node._open_toss_record(action='toss', goal_id='dead', cycle_index=1,
                           catch_pose=_POSE, throw_delay=5.0, vel_scale=0.9,
                           raw_goal={}, flight=_FLIGHT)
    _build(node, monkeypatch)
    row = _record(node)
    assert row['trim_aim_rad'] == [0.0, 0.0]        # commanded: nothing
    assert row['trim_state'] == 'ACTIVE'            # known: an active estimator
    assert row['trim_source_n'] == 3
    assert row['trim_reset_reason'] == 'because'
    assert row['speed_bias_applied'] == pytest.approx(1.02)
    assert row['timing_bias_applied_ms'] == pytest.approx(25.0)


def test_the_record_carries_map_trim_monitor_and_total_separately(monkeypatch,
                                                                   tmp_path):
    """Four keys, and the corpus must be able to tell them apart: what the map
    commanded, what layer 2 commanded (structurally nothing), what layer 2
    ESTIMATED, and what was actually flown."""
    node = _node_with_map(monkeypatch, tmp_path,
                          _cal_doc(math.radians(0.2), 0.0))
    _enable_trim(node)
    node._toss_trim = _StubTrim((math.radians(0.1), 0.0))
    node._open_toss_record(action='toss', goal_id='dead', cycle_index=1,
                           catch_pose=_POSE, throw_delay=5.0, vel_scale=0.9,
                           raw_goal={}, flight=_FLIGHT)
    _build(node, monkeypatch)
    row = _record(node)
    assert row['map_aim_rad'][0] == pytest.approx(math.radians(0.2))
    assert row['trim_aim_rad'] == [0.0, 0.0]                  # commanded
    assert row['trim_monitor_aim_rad'][0] == pytest.approx(   # estimated
        math.radians(0.1))
    assert row['trim_authority'] == 'MONITOR'
    # The TOTAL is the map alone — 0.2°, not 0.3°. This is the assertion the
    # predecessor got wrong the moment layer 2 lost its authority, and it is the
    # one that would catch a partial revert.
    assert row['total_aim_rad'][0] == pytest.approx(math.radians(0.2))
    # The mm report lands on the **y** axis, not x: the aim→landing Jacobian is
    # a 90° ROTATION (a +rx tilt moves the ball in −y), not a scaled identity.
    # Asserting on [0] here would pass on a build that had quietly made it one.
    assert row['map_aim_mm_at_h'][0] == pytest.approx(0.0, abs=1e-9)
    assert row['map_aim_mm_at_h'][1] != 0.0
    assert row['trim_aim_mm_at_h'][1] == pytest.approx(0.0, abs=1e-12)
    # The mm report fields still sum to the commanded total offset, exactly.
    total = tr.aim_target_offset_mm(math.radians(0.2), 0.0, _FLIGHT, _POSE[2])
    assert (row['map_aim_mm_at_h'][1] + row['trim_aim_mm_at_h'][1]
            == pytest.approx(float(total[1]), rel=1e-9))


def test_a_goal_with_no_trim_object_records_nulls_not_zeros(monkeypatch,
                                                            tmp_path):
    """No estimator ⇒ ``trim_state`` is NULL, not ``'WARMUP'``. "There was no
    trim" and "the trim had nothing yet" are different facts and the corpus must
    keep them apart — the same present-and-null discipline ``blank_record``
    applies everywhere else."""
    node = _node_with_map(monkeypatch, tmp_path, None)
    node._toss_trim = None
    node._open_toss_record(action='toss', goal_id='dead', cycle_index=1,
                           catch_pose=_POSE, throw_delay=5.0, vel_scale=0.9,
                           raw_goal={}, flight=_FLIGHT)
    _build(node, monkeypatch)
    row = _record(node)
    assert row['trim_state'] is None
    assert row['trim_source_n'] is None
    assert row['trim_aim_rad'] == [0.0, 0.0]        # commanded: still explicit


# ── 6. the proposal, and premise P1 ───────────────────────────────────────────

def test_the_proposal_is_written_at_goal_end_and_is_not_a_calibration(
        monkeypatch, tmp_path):
    """PREMISE P1, enforced at the node. The goal's trim dies at
    ``_toss_trim_end``; its only trace is a document in ``temp/logs/`` that the
    map loader structurally REFUSES, so a mistaken ``cp`` into ``config/`` fails
    loudly instead of installing an unreviewed online estimate as a
    calibration."""
    import jugglebot.reload_coordinator_node as rcn
    monkeypatch.setattr(rcn, '_RECORD_BELT_DIR', str(tmp_path))
    node = _node_with_map(monkeypatch, tmp_path, None)
    node._toss_trim = _StubTrim((math.radians(0.1), 0.0))
    node._toss_trim_end()
    assert node._toss_trim is None, 'the trim must not outlive its goal'
    written = sorted(p for p in os.listdir(str(tmp_path))
                     if p.endswith('_trim_proposal.yaml'))
    assert len(written) == 1, written
    doc = yaml.safe_load((tmp_path / written[0]).read_text())
    assert doc['schema'] == toss_trim.TRIM_SCHEMA
    assert doc['toss_trim_enabled'] is False
    with pytest.raises(toss_cal.TossCalError):
        toss_cal.parse_toss_cal(doc)


def test_a_proposal_write_failure_never_reaches_the_goal(monkeypatch, tmp_path):
    """Instrument-grade failure handling: a full disk costs the proposal, never
    a teardown. The same rule the record belt already follows."""
    import jugglebot.reload_coordinator_node as rcn
    monkeypatch.setattr(rcn, '_RECORD_BELT_DIR',
                        str(tmp_path / 'nope' / '\x00bad'))
    node = _node_with_map(monkeypatch, tmp_path, None)
    node._toss_trim = _StubTrim((math.radians(0.1), 0.0))
    node._toss_trim_end()          # must not raise
    assert node._toss_trim is None


def test_every_toss_goal_starts_a_fresh_trim_and_ends_it():
    """Both goal handlers, structurally. A trim carried across a re-``level``, a
    map reload or an operator's coffee break estimates a quantity that changed
    underneath it, and a trim never ended leaks its proposal into the next
    goal's file."""
    path = os.path.join(_PKG_DIR, 'reload_coordinator_node.py')
    with open(path, encoding='utf-8') as handle:
        tree = ast.parse(handle.read())
    starts, ends = set(), set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.FunctionDef):
            continue
        for child in ast.walk(node):
            if isinstance(child, ast.Call):
                dotted = _dotted(child.func)
                if dotted == 'self._toss_trim_begin':
                    starts.add(node.name)
                elif dotted == 'self._toss_trim_end':
                    ends.add(node.name)
    assert starts == {'_execute_toss', '_execute_toss_continuous'}, starts
    assert ends == {'_execute_toss', '_execute_toss_continuous'}, ends


def test_the_trim_is_warm_started_from_the_maps_anchor(monkeypatch, tmp_path):
    """``anchor.aim_rad`` is the absolute residual measured at the home anchor,
    which § 3.2 refuses to ship as a correction because it is ``level``-noise
    contaminated — but it is exactly the right PRIOR, and ``n₀ = 4`` lets one
    node's worth of data override it."""
    doc = _cal_doc(0.0, 0.0)
    doc['anchor'] = {'aim_rad': [math.radians(0.08), math.radians(-0.03)]}
    doc['speed'] = {'k_v': 1.03}
    node = _node_with_map(monkeypatch, tmp_path, doc)
    node._toss_trim_begin(goal_id='abc')
    trim = node._toss_trim
    assert isinstance(trim, toss_trim.SessionTrim)
    snap = trim.snapshot()
    assert snap['prior_rad'][0] == pytest.approx(math.radians(0.08))
    assert snap['prior_rad'][1] == pytest.approx(math.radians(-0.03))
    assert trim.speed_gain() == pytest.approx(1.03)


def test_a_DORMANT_map_contributes_NO_prior(monkeypatch, tmp_path):
    """AUDIT FIX 2026-08-11. The identical map, DORMANT — its ``requires``
    names a tilt-map version the machine is not running.

    Layer 1 already refuses to apply it (D3: an aim residual fitted under one
    levelling layer double-counts another's delta). The prior was walking in
    behind that fence: ``_toss_trim_begin`` read ``anchor.aim_rad`` and
    ``speed.k_v`` off the same distrusted map and seeded the estimator with them
    at n₀ = 4 — the same double-count, at a quarter of the authority and with
    none of the warning. Note this is NOT a retreat from "layer 2 does not
    depend on layer 1's dormancy": the trim's own MEASUREMENT is taken this goal
    against this layer 0 and is untouched. Only the borrowed number is refused.
    """
    doc = _cal_doc(0.0, 0.0, tilt_version='2026-01-01-00000000')
    doc['anchor'] = {'aim_rad': [math.radians(0.08), math.radians(-0.03)]}
    doc['speed'] = {'k_v': 1.03}
    node = _node_with_map(monkeypatch, tmp_path, doc)
    assert node._toss_cal is not None                 # loaded…
    assert node._toss_cal_status_snapshot()['toss_cal_applied'] is False  # …dormant
    warnings = []
    node.get_logger().warning = warnings.append
    node._toss_trim_begin(goal_id='abc')
    trim = node._toss_trim
    assert isinstance(trim, toss_trim.SessionTrim)
    snap = trim.snapshot()
    assert snap['prior_rad'][0] == pytest.approx(0.0)
    assert snap['prior_rad'][1] == pytest.approx(0.0)
    assert trim.speed_gain() == pytest.approx(1.0)
    # And it says so — a silently neutered prior is a number nobody can explain
    # later from the console alone.
    assert any('DORMANT' in w for w in warnings), warnings


def test_the_console_trim_block_is_emitted_once_per_cycle(monkeypatch,
                                                          tmp_path):
    node = _node_with_map(monkeypatch, tmp_path, None)
    node._toss_trim = _StubTrim((0.0, 0.0))
    node._open_toss_record(action='toss', goal_id='dead', cycle_index=1,
                           catch_pose=_POSE, throw_delay=5.0, vel_scale=0.9,
                           raw_goal={}, flight=_FLIGHT)
    _build(node, monkeypatch)
    node._toss_trim_observe({'cycle_index': 1}, node._toss_trim_snapshot())
    assert len(node._toss_trim.observed) == 1