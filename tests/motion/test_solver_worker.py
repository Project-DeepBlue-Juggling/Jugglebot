"""``motion/skills/solver_worker`` — the segment solve in its own process
(unit I1, 2026-10-13; design ``w3_unitI_design.md`` § 6 b-d).

What the mocked ``tests/ros`` suite cannot see and this file can: the
``posix_spawn`` + fd-3 + ``PYTHONPATH`` inheritance, real pickling of every
field of a real ``PlanRecord``/``InstallResult``, signal handling, and the
death/restart budget.  The worker is module-scoped (spawn to READY ~0.33 s).
All resources are ephemeral (a socketpair per worker); parallel-safe.
"""

from __future__ import annotations

import dataclasses
import math
import os
import pickle
import signal
import time

import numpy as np
import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion import unified_cycle as uc
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as si
from jugglebot.motion.skills import solver_worker as sw
from jugglebot.motion.trajectory import cup_realize as cr
from jugglebot.motion.trajectory.limits import TrajectoryLimits

FLIGHT_S = 0.857
LAUNCH_S = 0.4
LAND_VEL = np.array([0.0, 0.0, -FLIGHT_S / 2.0 * 9806.0])
FAR_S = 1e9     # a deadline that never binds


@pytest.fixture(scope='module')
def geom():
    return StewartGeometry()


@pytest.fixture(scope='module')
def limits():
    return TrajectoryLimits.from_config(hw).with_session_limits(
        leg_vel_mmps=300.0, leg_acc_mmps2=5000.0, leg_jerk_mmps3=200000.0,
        hand_acc_rps2=3500.0)


@pytest.fixture(scope='module')
def sites():
    return si.columns_sites(100.0)


def _rest_state(cup_mm) -> uc.CycleState:
    cfg = cr.RealizeConfig()
    slider_mm = float(cup_mm[2]) - cfg.cup_z_base_mm
    rev = (slider_mm - cfg.slider_rev_zero_mm) / 1000.0 * cr.HAND_REV_PER_M
    pose = np.array([cup_mm[0], cup_mm[1], cfg.active_z_mm, 0.0, 0.0, 0.0])
    return uc.CycleState.at_rest(pose, rev, cfg)


def _throw_req(seq, t0, limits, geom, sites, deadline=FAR_S):
    p1, _p2 = sites
    return sw.SegmentSolveRequest(
        seq=seq, record=None, seed_rest=_rest_state(p1.rest_site_mm()),
        kind=sg.THROW,
        terminal=sg.ThrowTerminal(site_mm=p1.throw_site_mm(),
                                  target_mm=p1.catch_site_mm(),
                                  flight_s=FLIGHT_S, t_release_s=t0 + LAUNCH_S),
        t_now_s=t0, lead_s=ex.LEAD_S, cfg=None, limits=limits, geom=geom,
        warm_start=None, reserve_fresh_lead=False, deadline_s=deadline)


def _catch_req(seq, t0, record, warm, limits, geom, sites, deadline=FAR_S):
    _p1, p2 = sites
    return sw.SegmentSolveRequest(
        seq=seq, record=record, seed_rest=None, kind=sg.CATCH,
        terminal=sg.CatchTerminal(landing_mm=p2.catch_site_mm(),
                                  landing_vel_mm_s=LAND_VEL,
                                  t_land_s=t0 + LAUNCH_S + FLIGHT_S,
                                  rest_site_mm=p2.rest_site_mm()),
        t_now_s=t0 + LAUNCH_S, lead_s=ex.LEAD_S, cfg=None, limits=limits,
        geom=geom, warm_start=warm, reserve_fresh_lead=False,
        deadline_s=deadline)


def _t0() -> float:
    # In the future on the perf clock, so the worker's lateness checks read
    # "early" however long the solve takes.
    return time.perf_counter() + 3600.0


@pytest.fixture(scope='module')
def solver():
    s = sw.WorkerSegmentSolver()
    assert s.wait_ready(20.0), s.state
    yield s
    s.close()


def _plan_equal(a, b):
    names = [n for c in type(a).__mro__ for n in getattr(c, '__slots__', ())]
    assert 'pose' in names and 'hand_rev' in names
    for name in names:
        va, vb = getattr(a, name), getattr(b, name)
        if isinstance(va, np.ndarray):
            assert np.array_equal(va, vb), name
        else:
            assert va == vb, name


# ---------------------------------------------------------------------------
# (1) round trip == in-process, field by field
# ---------------------------------------------------------------------------

def test_a_real_throw_then_catch_round_trips_identically_to_in_process(
        solver, limits, geom, sites):
    t0 = _t0()
    rq = _throw_req(1, t0, limits, geom, sites)
    rep = solver.solve(rq)
    assert rep.seq == 1 and rep.result.accepted, rep.result.message
    new_ref, res_ref, seg_ref = ex.install_segment(
        None, rq.seed_rest, rq.kind, rq.terminal, rq.t_now_s, limits=limits,
        geom=geom)
    assert rep.result.accepted == res_ref.accepted
    for name in ('code', 'splice_k', 't0_s', 'event_t_s', 'seeded_post_release'):
        assert getattr(rep.result, name) == getattr(res_ref, name), name
    _plan_equal(rep.new_record.plan, new_ref.plan)
    assert rep.new_record.t0_s == new_ref.t0_s
    assert rep.release_t_s == seg_ref.release_t_s
    assert rep.t_inst_s > 0.0 and math.isfinite(rep.t_inst_s)
    assert rep.solve_s > 0.0

    rq2 = _catch_req(2, t0, rep.new_record, rep.warm_start, limits, geom, sites)
    rep2 = solver.solve(rq2)
    assert rep2.seq == 2 and rep2.result.accepted, rep2.result.message
    new2, res2, seg2 = ex.install_segment(
        new_ref, None, rq2.kind, rq2.terminal, rq2.t_now_s, limits=limits,
        geom=geom, warm_start=seg_ref.warm_start)
    assert res2.accepted
    assert rep2.result.splice_k == res2.splice_k
    _plan_equal(rep2.new_record.plan, new2.plan)
    assert rep2.release_t_s == seg2.release_t_s
    assert rep2.warm_start is not None


# ---------------------------------------------------------------------------
# (2) deadline semantics (worker side, driven in-thread over a socketpair)
# ---------------------------------------------------------------------------

def test_the_worker_skips_a_request_whose_deadline_passed(limits, geom, sites):
    import socket
    import threading
    from multiprocessing.connection import Connection
    a, b = socket.socketpair()
    th = threading.Thread(target=sw.worker_main, args=(b.detach(),), daemon=True)
    th.start()
    conn = Connection(a.detach())
    tag = pickle.loads(conn.recv_bytes())
    assert tag[0] == 'READY'
    rq = _throw_req(7, _t0(), limits, geom, sites,
                    deadline=time.perf_counter() - 0.5)
    conn.send_bytes(pickle.dumps(('SOLVE', rq)))
    rep = pickle.loads(conn.recv_bytes())
    assert rep.seq == 7 and not rep.result.accepted
    assert rep.result.code == sw.SKIPPED_LATE
    assert rep.solve_s == 0.0 and rep.new_record is None   # nothing was solved
    conn.send_bytes(pickle.dumps(('STOP',)))
    th.join(5.0)
    assert not th.is_alive()


def test_the_client_refuses_a_passed_deadline_without_sending(solver, limits,
                                                               geom, sites):
    rq = _throw_req(8, _t0(), limits, geom, sites,
                    deadline=time.perf_counter() - 0.01)
    rep = solver.solve(rq)
    assert rep.result.code == sw.SKIPPED_LATE and not rep.result.accepted


def test_a_rest_that_misses_its_deadline_is_unavailable_not_a_timing_code(
        solver, limits, geom, sites):
    p1, _ = sites
    rq = dataclasses.replace(
        _throw_req(9, _t0(), limits, geom, sites),
        kind=sg.REST, terminal=sg.RestTerminal(rest_site_mm=p1.rest_site_mm(),
                                               t_rest_s=_t0() + 0.5),
        deadline_s=time.perf_counter() - 0.01)
    assert solver.solve(rq).result.code == sw.SOLVER_UNAVAILABLE


# ---------------------------------------------------------------------------
# (3) a stale reply is dropped by seq
# ---------------------------------------------------------------------------

def test_a_stale_reply_is_dropped_by_seq(solver, limits, geom, sites):
    t0 = _t0()
    # Abandon a real solve: the deadline lands inside it, so the client gives
    # up (SKIPPED_LATE) while the worker is still working.
    gone = solver.solve(_throw_req(100, t0, limits, geom, sites,
                                   deadline=time.perf_counter() + 0.004))
    assert gone.seq == 100 and gone.result.code == sw.SKIPPED_LATE
    nxt = solver.solve(_throw_req(101, t0, limits, geom, sites))
    assert nxt.seq == 101 and nxt.result.accepted, nxt.result.message
    assert solver.state == sw.READY and not solver._inflight


# ---------------------------------------------------------------------------
# (4) STOP and EOF exit 0; (5) SIGINT ignored; (6) READY content
# ---------------------------------------------------------------------------

def _wait_exit(pid, timeout=5.0):
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        done, status = os.waitpid(pid, os.WNOHANG)
        if done:
            return os.WEXITSTATUS(status) if os.WIFEXITED(status) else -1
        time.sleep(0.01)
    raise AssertionError('worker did not exit')


def test_stop_makes_the_worker_exit_zero():
    s = sw.WorkerSegmentSolver()
    assert s.wait_ready(20.0)
    pid = s.pid
    s._conn.send_bytes(pickle.dumps(('STOP',)))
    assert _wait_exit(pid) == 0
    s._pid = None
    s.close()


def test_eof_makes_the_worker_exit_zero():
    s = sw.WorkerSegmentSolver()
    assert s.wait_ready(20.0)
    pid = s.pid
    s._conn.close()
    assert _wait_exit(pid) == 0
    s._pid = None
    s._conn = None
    s.close()


def test_close_stops_the_worker_cleanly():
    s = sw.WorkerSegmentSolver()
    assert s.wait_ready(20.0)
    pid = s.pid
    s.close()
    assert s.state == sw.DOWN
    with pytest.raises(ProcessLookupError):
        os.kill(pid, 0)         # reaped: no zombie left


def test_sigint_is_ignored_by_the_worker(solver, limits, geom, sites):
    pid = solver.pid
    os.kill(pid, signal.SIGINT)
    time.sleep(0.1)
    assert solver.pid == pid
    rep = solver.solve(_throw_req(200, _t0(), limits, geom, sites))
    assert rep.result.accepted, rep.result.message


def test_ready_carries_the_executor_file_and_the_blas_thread_count(solver):
    pid, exfile, blas = solver.ready_info
    assert pid == solver.pid
    assert os.path.realpath(exfile) == os.path.realpath(ex.__file__)
    assert blas is None or isinstance(blas, int)


def test_a_worker_on_the_wrong_tree_is_down_and_unavailable(limits, geom, sites):
    logs = []
    s = sw.WorkerSegmentSolver(
        expected_executor_file='/nonexistent/other/tree/executor.py',
        log=lambda lvl, msg: logs.append((lvl, msg)))
    try:
        assert s.wait_ready(20.0) is False
        assert s.state == sw.DOWN
        rep = s.solve(_throw_req(1, _t0(), limits, geom, sites))
        assert rep.result.code == sw.SOLVER_UNAVAILABLE
        assert [lvl for lvl, _ in logs].count('error') == 1
    finally:
        s.close()


# ---------------------------------------------------------------------------
# (7) death -> UNAVAILABLE -> ONE restart per episode
# ---------------------------------------------------------------------------

def test_a_killed_worker_is_unavailable_then_restarted_then_down_on_a_second_death(
        limits, geom, sites):
    logs = []
    s = sw.WorkerSegmentSolver(log=lambda lvl, msg: logs.append((lvl, msg)))
    try:
        assert s.wait_ready(20.0)
        first = s.pid
        os.kill(first, signal.SIGKILL)
        time.sleep(0.05)
        rep = s.solve(_throw_req(1, _t0(), limits, geom, sites))
        assert rep.result.code == sw.SOLVER_UNAVAILABLE
        assert s.pid is not None and s.pid != first        # a replacement
        assert s.state == sw.STARTING
        assert s.solve(_throw_req(2, _t0(), limits, geom, sites)
                       ).result.code == sw.SOLVER_UNAVAILABLE   # not READY yet
        assert s.wait_ready(20.0)
        ok = s.solve(_throw_req(3, _t0(), limits, geom, sites))
        assert ok.result.accepted, ok.result.message        # episode ends

        # Fresh episode: the answered solve restored the restart budget.
        os.kill(s.pid, signal.SIGKILL)
        time.sleep(0.05)
        assert s.solve(_throw_req(4, _t0(), limits, geom, sites)
                       ).result.code == sw.SOLVER_UNAVAILABLE
        assert s.wait_ready(20.0)
        # ... and a second death with NO reply in between stays DOWN.
        os.kill(s.pid, signal.SIGKILL)
        time.sleep(0.05)
        rep = s.solve(_throw_req(5, _t0(), limits, geom, sites))
        assert rep.result.code == sw.SOLVER_UNAVAILABLE
        assert s.state == sw.DOWN and s.pid is None
        assert 'failed twice' in rep.result.message or 'failed twice' in \
            s.solve(_throw_req(6, _t0(), limits, geom, sites)).result.message
        assert [lvl for lvl, _ in logs].count('error') == 1
    finally:
        s.close()


# ---------------------------------------------------------------------------
# (8) picklability, in-process (no spawn)
# ---------------------------------------------------------------------------

def test_request_and_reply_are_picklable_and_round_trip_exactly(limits, geom,
                                                                sites):
    t0 = _t0()
    rq = _throw_req(1, t0, limits, geom, sites)
    rq2 = pickle.loads(pickle.dumps(rq, protocol=pickle.HIGHEST_PROTOCOL))
    assert rq2.kind == rq.kind and rq2.t_now_s == rq.t_now_s
    rep = sw.InProcessSegmentSolver().solve(rq)
    assert rep.result.accepted, rep.result.message
    rep2 = pickle.loads(pickle.dumps(rep, protocol=pickle.HIGHEST_PROTOCOL))
    _plan_equal(rep2.new_record.plan, rep.new_record.plan)
    assert rep2.t_inst_s == math.inf
    # The CATCH carries the chained record and the QP warm start.
    rq3 = _catch_req(2, t0, rep.new_record, rep.warm_start, limits, geom, sites)
    pickle.loads(pickle.dumps(rq3, protocol=pickle.HIGHEST_PROTOCOL))
    rep3 = sw.InProcessSegmentSolver().solve(rq3)
    assert rep3.result.accepted, rep3.result.message
    pickle.loads(pickle.dumps(rep3, protocol=pickle.HIGHEST_PROTOCOL))


def test_the_in_process_double_looks_up_the_solver_and_clock_at_call_time(
        monkeypatch, limits, geom, sites):
    seen = {}

    def fake(record, seed, kind, terminal, t_now, **kw):
        seen['t_install_s'] = kw['t_install_s']
        return None, ex.InstallResult(False, 'X', 'fake', 0.0), None

    monkeypatch.setattr(ex, 'install_segment', fake)
    rep = sw.InProcessSegmentSolver().solve(_throw_req(1, 5.0, limits, geom, sites))
    assert rep.result.code == 'X'
    assert seen['t_install_s'] is time.perf_counter



def test_a_worker_that_never_reports_ready_is_failed_then_down(
        monkeypatch, tmp_path, limits, geom, sites):
    """Alive but silent (import hang, wrong interpreter): STARTING times out
    into the same one-restart-then-DOWN budget.  The "interpreter" is a script
    that sleeps and never speaks; the timeout is shortened for the test."""
    fake = tmp_path / 'silent_python'
    fake.write_text('#!/bin/sh\nexec sleep 60\n')
    fake.chmod(0o755)
    monkeypatch.setattr(sw, 'READY_TIMEOUT_S', 0.3)
    logs = []
    s = sw.WorkerSegmentSolver(python=str(fake),
                               log=lambda lvl, msg: logs.append(lvl))
    try:
        first = s.pid
        assert s.state == sw.STARTING
        time.sleep(0.4)
        s.poll_health()                     # times out -> ONE restart
        assert s.state == sw.STARTING and s.pid not in (None, first)
        time.sleep(0.4)
        s.poll_health()                     # budget spent -> DOWN
        assert s.state == sw.DOWN and s.pid is None
        rep = s.solve(_throw_req(1, _t0(), limits, geom, sites))
        assert rep.result.code == sw.SOLVER_UNAVAILABLE
        assert 'failed twice' in rep.result.message
        assert logs.count('error') == 1
    finally:
        s.close()
