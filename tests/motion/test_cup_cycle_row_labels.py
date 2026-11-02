"""Row labels in the ``UNVERIFIED`` refusal (JB-104 / JB-126, 2026-11-01).

``cup_cycle._verify`` refuses a QP point whose equality residual or constraint
violation is above the bar.  The reason stays ``'UNVERIFIED'`` (infeasible and
degenerate are deliberately not distinguished; no caller branches on it), but
the message now NAMES the worst equality row and the worst inequality row so
the operator sees what bound.  The labels are derived lazily from layout
bookkeeping recorded at assembly (``_Program.eq_marks`` / ``ineq_names`` /
``ineq_widths`` / ``held_knots``), which no numeric code reads.

Each labelled block is checked here against the ASSEMBLED matrices, not
against the labeller's own arithmetic: the label's claim (axis, knot, kind) is
parsed back out and compared with the row's actual support in ``Aeq`` / ``C``.
"""

from __future__ import annotations

import re

import numpy as np
import pytest

from jugglebot.motion.trajectory import cup_cycle as cc

from tests.motion.test_cup_capture import _cfg as _cap_cfg, _cup_case
from tests.motion.test_cup_cycle import _grid_case

AX = {'x': 0, 'y': 1, 'z': 2}


def _throw_catch_program(cfg=None):
    case = _grid_case(0.8, 0.8, False)
    state0 = cc.CupState(case['pos0'], case['vel0'], case['acc0'], None)
    throw = cc.ThrowEvent(1, case['period_s'], case['throw_pos'],
                          case['throw_target'], case['flight_s'])
    catch = cc.CatchEvent(0, case['catch_time_s'], case['catch_pos'],
                          case['catch_vel'])
    return cc._assemble(state0, throw, catch, case['period_s'],
                        cc.CupCycleConfig() if cfg is None else cfg)


def _capture_program():
    s0, catch, period, settle, _ = _cup_case((20.0, -15.0), 15, capture=True)
    return cc._assemble(s0, None, catch, period, _cap_cfg(), settle_site=settle)


def _support(row, n):
    """``{axis: [knot, ...]}`` of the nonzero jerk variables of one row."""
    nz = np.flatnonzero(np.abs(row[:3 * n]) > 1e-12)
    out = {}
    for j in nz:
        out.setdefault(int(j // n), []).append(int(j % n))
    return out


def _eq_labels(prog):
    return [cc._eq_label(prog, i) for i in range(prog.Aeq.shape[0])]


def _ineq_labels(prog):
    return [cc._ineq_label(prog, i) for i in range(prog.C.shape[1])]


def test_no_row_of_a_real_program_falls_back_to_an_index():
    for prog in (_throw_catch_program(), _capture_program()):
        for lab in _eq_labels(prog) + _ineq_labels(prog):
            assert 'row' not in lab.split(':')[0], lab      # "... row i of m"


def test_release_and_rest_equalities_name_axis_and_derivative():
    prog = _throw_catch_program()
    n = prog.n_steps
    seen = set()
    for i, lab in enumerate(_eq_labels(prog)):
        m = re.match(r'equality %d: release \(throw\) (\w+) along (\w)$' % i, lab)
        if not m:
            continue
        qty, a = m.group(1), AX[m.group(2)]
        row = prog.Aeq[i, a * n:(a + 1) * n]
        assert _support(prog.Aeq[i], n) == {a: list(range(n))}
        d1, d2 = np.diff(row), np.diff(row, 2)
        if qty == 'acceleration':
            assert np.allclose(d1, 0.0)           # jerk integrates to acc: flat
        elif qty == 'velocity':
            assert not np.allclose(d1, 0.0) and np.allclose(d2, 0.0)
        else:
            assert qty == 'position' and not np.allclose(d2, 0.0)
        seen.add((qty, a))
    assert seen == {(q, a) for q in ('position', 'velocity', 'acceleration')
                    for a in range(3)}
    assert any('catch-position' in lab for lab in _eq_labels(prog))


def test_capture_program_labels_slaving_capture_and_rest_rows():
    prog = _capture_program()
    n, labs = prog.n_steps, _eq_labels(prog)
    assert prog.capture_k and len(prog.held_knots)
    slav = [(i, l) for i, l in enumerate(labs) if 'held-axis slaving' in l]
    assert len(slav) == 2 * len(prog.held_knots)
    for i, lab in slav:
        m = re.match(r'equality %d: held-axis slaving (\w) jerk, knot (\d+)$' % i, lab)
        a, k = AX[m.group(1)], int(m.group(2))
        # slaving row: unit on that lateral jerk and -kappa on the z jerk, same knot
        assert _support(prog.Aeq[i], n) == {a: [k], 2: [k]}
    cap = [(i, l) for i, l in enumerate(labs) if 'capture-move' in l]
    assert len(cap) == 6
    for i, lab in cap:
        a = AX[re.search(r'capture-move \w+ (\w) at', lab).group(1)]
        assert a in _support(prog.Aeq[i], n)
        assert 'knot %d' % prog.capture_k in lab
    # z-only rest and catch-position rows on a held axis
    assert any('terminal rest position along z' in l for l in labs)
    assert any('catch-position z' in l for l in labs)


def test_box_rows_are_labelled_with_the_axis_knot_and_side_they_bound():
    for prog in (_throw_catch_program(), _capture_program()):
        n, hit = prog.n_steps, set()
        for i, lab in enumerate(_ineq_labels(prog)):
            col = prog.C[:, i]
            m = re.match(r'inequality %d: knot (\d+) (\w) jerk (upper|lower) bound$'
                         % i, lab)
            if m:
                k, a, side = int(m.group(1)), AX[m.group(2)], m.group(3)
                sup = _support(col, n)
                if prog.capture_k and a < 2 and k < prog.capture_k:
                    assert sup == {a: [k], 2: [k]}      # j_e box: -kappa on j_z
                else:
                    assert sup == {a: [k]}
                assert (col[a * n + k] < 0) == (side == 'upper')   # -BR / +BR
                hit.add('jerk')
                continue
            m = re.match(r'inequality %d: knot (\d+) (\w) position (upper|lower) '
                         'bound$' % i, lab)
            if m:
                k, a, side = int(m.group(1)), AX[m.group(2)], m.group(3)
                sup = _support(col, n)
                assert sup == {a: list(range(k))}       # Ap[k] reaches jerks 0..k-1
                assert (col[a * n] < 0) == (side == 'upper')
                hit.add('pos')
        assert hit == {'jerk', 'pos'}
    prog = _throw_catch_program()
    assert any('catch-runway floor' in l for l in _ineq_labels(prog))


def test_held_knots_are_skipped_in_the_jerk_labels():
    prog = _capture_program()
    labs = _ineq_labels(prog)
    held = set(prog.held_knots)
    named = {(int(m.group(1)), m.group(2)) for l in labs
             for m in [re.search(r'knot (\d+) (\w) jerk', l)] if m}
    assert held and not any((k, a) in named for k in held for a in 'xy')


def test_unknown_layout_falls_back_to_the_row_index():
    import dataclasses
    prog = _throw_catch_program()
    bare = dataclasses.replace(prog, eq_marks=(), ineq_names=(), ineq_widths=())
    assert cc._eq_label(bare, 3) == 'equality row 3 of %d' % prog.Aeq.shape[0]
    assert cc._ineq_label(bare, 5) == 'inequality row 5 of %d' % prog.C.shape[1]
    off = dataclasses.replace(prog, ineq_widths=tuple(
        w + 1 for w in prog.ineq_widths))
    assert cc._ineq_label(off, 0).startswith('inequality row 0 of')


def test_verify_message_names_the_worst_equality_and_inequality_rows():
    prog = _throw_catch_program()
    good, active, iters = cc._solve_qp(prog, None, 300, 1e-9, 1e-7)
    cc._verify(prog, good, 1e-7, active, iters)
    # (1) break one chosen equality row by ~1, leave inequalities slack-ish
    i = 4
    d = prog.Aeq[i] / (prog.Aeq[i] @ prog.Aeq[i])
    with pytest.raises(cc.CupCycleInfeasible) as ei:
        cc._verify(prog, good + d, 1e-7, active, iters)
    assert ei.value.reason == 'UNVERIFIED'
    msg = str(ei.value)
    assert msg.startswith('QP solution failed verification')
    assert cc._eq_label(prog, i) in msg and 'residual 1.000e+00' in msg
    # (2) push one inequality column far past its bound
    j = int(np.argmin(prog.C.T @ good - prog.bc)) + 0
    t = 50.0
    with pytest.raises(cc.CupCycleInfeasible) as ei:
        cc._verify(prog, good - t * prog.C[:, j], 1e-7, active, iters)
    assert ei.value.reason == 'UNVERIFIED'
    assert 'worst rows:' in str(ei.value)
    v = -(prog.C.T @ (good - t * prog.C[:, j]) - prog.bc)
    assert cc._ineq_label(prog, int(v.argmax())) in str(ei.value)
    # (3) non-finite x: refused, nothing nameable, no crash
    with pytest.raises(cc.CupCycleInfeasible) as ei:
        cc._verify(prog, good * np.nan, 1e-7, active, iters)
    assert ei.value.reason == 'UNVERIFIED' and 'worst rows' not in str(ei.value)


def _pose_programs(z_band_mm):
    """The ``_assemble`` programs of one production pose STEADY window."""
    import jugglebot.hardware_config as hw
    from jugglebot.motion import unified_cycle as uc
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.trajectory.limits import TrajectoryLimits
    from tests.motion import test_pose_planner as tp
    limits = TrajectoryLimits.from_config(hw).with_sitting_limits(
        leg_vel_mmps=350.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3900.0)
    progs, orig = [], cc._assemble

    def spy(*a, **k):
        p = orig(*a, **k)
        progs.append(p)
        return p
    cc._assemble = spy
    try:
        tp._folded(0.50, uc.PLANNER_POSE, limits, StewartGeometry(),
                   pose_cfg=cc.PoseConfig(z_band_mm=z_band_mm))
    finally:
        cc._assemble = orig
    return [p for p in progs if 'cone' in p.ineq_names]


def _max_knot(col, n, lo, hi):
    nz = np.flatnonzero(np.abs(col[lo:hi]) > 1e-12)
    return int(nz.max() % n) if nz.size else -1


def test_pose_blocks_are_labelled_and_match_their_support():
    prog = _pose_programs(30.0)[-1]
    n = prog.n_steps
    assert prog.n_dz == n
    labs = _ineq_labels(prog)
    kinds = set()
    for i, lab in enumerate(labs):
        col = prog.C[:, i]
        m = re.match(r'inequality %d: knot (\d+) tilt-cone (side|floor)' % i, lab)
        if m:       # acceleration at knot k reaches jerks 0..k-1 on the cup blocks
            assert _max_knot(col, n, 0, 3 * n) == int(m.group(1)) - 1
            assert not np.any(col[3 * n:])
            kinds.add('cone')
        m = re.match(r'inequality %d: knot (\d+) platform-z band (lower|upper)$'
                     % i, lab)
        if m:
            k, side = int(m.group(1)), m.group(2)
            assert not np.any(col[:3 * n])                 # dz block only
            assert _max_knot(col, n, 3 * n, 4 * n) == k - 1
            assert (col[3 * n] > 0) == (side == 'lower')   # D col 2(k-1) is +Ap
            kinds.add('band')
        if 'touch-down skid' in lab:
            assert _max_knot(col, n, 0, 3 * n) == prog.catch_k
            kinds.add('skid')
    assert kinds == {'cone', 'band', 'skid'}
    assert not any(l.startswith('inequality row') for l in labs)
    eq = _eq_labels(prog)
    tail = eq[-3:]
    assert [l.split(': ')[1] for l in tail] == [
        'platform-z terminal position', 'platform-z terminal velocity',
        'platform-z terminal acceleration']
    for i in range(prog.Aeq.shape[0] - 3, prog.Aeq.shape[0]):
        assert not np.any(prog.Aeq[i, :3 * n]) and np.any(prog.Aeq[i, 3 * n:])
