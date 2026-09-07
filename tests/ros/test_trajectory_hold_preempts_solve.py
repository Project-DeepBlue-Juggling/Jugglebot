"""``trajectory/hold`` must be able to preempt an in-flight ``plan_cycle`` solve.

WHAT THIS DEFENDS, and it is the last line before the supersede cliff.

When a chained STEADY window's solve is running out of time, the coordinator's
final move is to give up at ``supersede_deadline − 0.160 s`` and HOLD. Under
plain ``rclpy.spin`` that hold queued BEHIND the very solve it was cancelling:
every service shared one mutually-exclusive callback group, so ``_svc_hold``
could not execute until ``_svc_plan_cycle`` returned — 387 ms idle, **1.06 s
loaded** — and it landed after the cliff. By then the emitter had sampled
``tau = duration − dt`` and shipped the release segment with ``v1 = 0`` against a
true 93 rev/s: inside ``MAX_LEAD_HAND_REV`` and ``MAX_DEVIATION_HAND_REV``, so no
firmware guard fires and the only symptom is a throw that went somewhere else
(``unified_cycle.latest_supersede_time_s`` carries the measurement).

The client-side alternative — have the coordinator give up a whole solve earlier
— raises the practical dwell floor to ~1.15 s and still cannot guarantee the
margin under load, so the closure is server-side and it is two pieces:

1. the hold service on its own ``ReentrantCallbackGroup`` under a
   ``MultiThreadedExecutor``, so it can run WHILE a solve is in flight;
2. an install epoch, so the solve that was still running cannot then install over
   the hold and re-arm the throw the operator just cancelled.

Both halves are needed and neither is sufficient: without (1) the hold is late,
without (2) the hold is silently undone a few hundred milliseconds later.

ROS 2 is mocked by ``tests/ros/conftest.py``. The concurrency here is REAL
threads against the real callbacks, which is the right level: the executor's job
is only to decide WHICH thread runs a callback, and what these tests pin is that
the callbacks are safe to run that way and that the wiring asks for it.
"""

from __future__ import annotations

import threading
import time

import numpy as np
import pytest

from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

from jugglebot_interfaces.srv import PlanCycle
from jugglebot import trajectory_node as tn
from jugglebot.motion.trajectory.cycle_plan import CyclePlan

from tests.ros.test_unified_cycle_integration import (
    _cycle_node,
    _launch_req,
    _refresh,
    _steady_chain_req,
)


#: Bound on how long the hold's own work may take while a solve runs beside it.
#:
#: DERIVED from measurement, not chosen. A hold is ``build_hold`` + ``_install``
#: — no QP, no ``validate_cycle``. MEASURED (2026-09-07, this Jetson, three runs
#: each, ``/tmp/t_lat.py``): **0.8 / 0.8 / 0.8 ms alone**, and **4.4 / 1.0 /
#: 1.0 ms while a real MODE_EXTEND solve ran concurrently on another thread** —
#: so GIL contention with a numpy-heavy solve costs at most ~3.6 ms here. numpy
#: releases the GIL inside its own C kernels and a cycle solve is a dense run of
#: SMALL calls, which is the property that could have made this bad; it does not,
#: at this size.
#:
#: The comparison that matters is not "hold alone" but "hold now vs hold after
#: the whole solve", and the same runs measured the solve at **307-314 ms** (and
#: 1.06 s loaded, per the coordinator's own numbers). 0.100 s is ~23x the worst
#: measured hold and still 3x under the solve it preempts — loose enough to
#: survive a loaded parallel worker, tight enough that a regression to
#: "the hold waits for the solve" fails it by a factor of three.
_HOLD_LATENCY_BOUND_S = 0.100


def _mode_and_seed(node):
    """A node in TRAJECTORY mode with a plan installed, ready to hold."""
    _refresh(node)
    return node


# ═════════════════════════════════════════════════════════════════════════════
# The wiring — pinned so a refactor cannot silently return to plain spin
# ═════════════════════════════════════════════════════════════════════════════

def test_only_the_hold_service_is_reentrant():
    """The hold is on its OWN reentrant group; everything else keeps its group.

    The narrowness is the safety argument. Moving every service to a reentrant
    group would let ``plan_cycle`` interleave with the timers and subscriptions it
    shares state with, and nothing here is written for that. Only the hold needs
    to run during a solve, so only the hold moves — and this test fails if a
    later refactor widens it.
    """
    node = _cycle_node()
    services = node._services
    hold = services['trajectory/hold']
    assert isinstance(hold.callback_group, ReentrantCallbackGroup)
    # Everything else stays on the node default (recorded as None by the mock),
    # i.e. mutually exclusive with the timers and subscriptions.
    for name, svc in services.items():
        if name == 'trajectory/hold':
            continue
        assert svc.callback_group is None, name
    # Named explicitly, because this is the pair whose serialisation the shared
    # state depends on: plan_cycle must NOT be reentrant.
    assert services['trajectory/plan_cycle'].callback_group is None


def test_main_runs_a_multi_threaded_executor_not_plain_spin(monkeypatch):
    """A reentrant group is inert under ``rclpy.spin`` — the executor must match.

    This is the half a refactor is most likely to undo, because ``spin`` looks
    like the simpler call and the callback group keeps compiling. Under plain
    ``spin`` the hold queues behind the solve exactly as before and the cliff is
    back, silently.
    """
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    built = {}

    class _Spy(MultiThreadedExecutor):
        def __init__(self, *a, **kw):
            super().__init__(*a, **kw)
            built['threads'] = kw.get('num_threads')

        def spin(self):
            built['spun'] = True

    monkeypatch.setattr('jugglebot.trajectory_node.MultiThreadedExecutor', _Spy)
    monkeypatch.setattr(rclpy, 'init', lambda *a, **k: None)
    monkeypatch.setattr(rclpy, 'shutdown', lambda *a, **k: None)
    monkeypatch.setattr(rclpy, 'spin',
                        lambda *a, **k: pytest.fail('plain spin is back'),
                        raising=False)

    tn.main()
    assert built.get('spun') is True
    # At least two: one thread to run the serialized group, one free for the hold.
    assert built.get('threads', 0) >= 2


# ═════════════════════════════════════════════════════════════════════════════
# The behaviour — a real hold against a real in-flight solve
# ═════════════════════════════════════════════════════════════════════════════

def test_a_hold_preempts_an_in_flight_solve(monkeypatch):
    """The hold installs WHILE the solve is running, and the solve then loses.

    Both halves in one test, because either alone is a false comfort: a hold that
    lands on time but is overwritten 300 ms later has not cancelled anything.

    The solve is made to block at a known point — after it has captured its epoch
    and started planning, before it installs — so "the hold ran while the solve
    was in flight" is a fact about ordering and not about how fast the box was.
    The hold then runs on this thread (the executor's second one, in production),
    and the solve is released.

    MEASURED (2026-09-07, this Jetson, three runs, `/tmp/t_lat.py`, with a REAL
    concurrent solve and no artificial block): hold **4.4 / 1.0 / 1.0 ms** against
    a solve of **314 / 311 / 308 ms**, every one of which came back
    `SUPERSEDED_BY_HOLD`. Alone the hold is 0.8 ms, so the GIL contention this
    change introduces costs at most ~3.6 ms — against the 307-1060 ms of waiting
    it removes.
    """
    node = _mode_and_seed(_cycle_node())
    # A cycle is installed and streaming, the shape a ring holds against.
    assert node._svc_plan_cycle(_steady_chain_req(),
                                PlanCycle.Response()).accepted is True
    cycle_plan = node._active_plan
    assert isinstance(cycle_plan, CyclePlan)

    solving = threading.Event()
    release = threading.Event()
    real_plan_cycle = tn.uc.plan_cycle

    def _blocking_plan_cycle(*a, **kw):
        # Block INSIDE the solve — after `_plan_cycle_extend` has already read
        # `self._cycle` and captured its origin. That is where a real solve is
        # when a hold lands, and it is the case the epoch exists for: the
        # NO_CYCLE check has already passed, so clearing the record is not what
        # stops this install.
        solving.set()
        assert release.wait(timeout=10.0), 'the hold never released the solve'
        return real_plan_cycle(*a, **kw)

    monkeypatch.setattr(tn.uc, 'plan_cycle', _blocking_plan_cycle)

    out = {}

    def _run_solve():
        req = _launch_req()
        req.mode = req.MODE_EXTEND
        req.kind = req.KIND_STEADY
        req.period_s = 1.4
        req.catch_frac = 0.6 / 1.4
        out['resp'] = node._svc_plan_cycle(req, PlanCycle.Response())
        out['returned_at'] = time.perf_counter()

    # Re-stamp immediately before the request: the chain install above cost a
    # real ~400 ms solve, which is most of the node's 0.5 s telemetry-staleness
    # bound, and a STALE_STATE refusal would never reach the dispatch at all.
    _refresh(node)
    worker = threading.Thread(target=_run_solve, daemon=True)
    worker.start()
    assert solving.wait(timeout=10.0), (
        'the solve never reached the planner: %s' % (out.get('resp') and
                                                     out['resp'].message,))

    # THE HOLD, while the solve is provably still in flight.
    _refresh(node)
    t0 = time.perf_counter()
    hold_resp = node._svc_hold(Trigger.Request(), Trigger.Response())
    hold_latency = time.perf_counter() - t0
    assert hold_resp.success is True, hold_resp.message
    hold_plan = node._active_plan
    assert hold_plan is not cycle_plan
    assert hold_latency < _HOLD_LATENCY_BOUND_S, hold_latency
    # The solve was STILL IN FLIGHT when the hold installed. Asserted on the
    # worker rather than by comparing timestamps: `held_at < returned_at` is true
    # by construction here (the worker cannot return until `release` is set below),
    # so it would have passed even if the hold had queued behind the solve.
    assert worker.is_alive()

    # Only now is the solve allowed to finish.
    release.set()
    worker.join(timeout=20.0)
    assert not worker.is_alive()

    # ...and the solve does NOT install over it. The continuity guard alone would
    # not have caught this: right after a hold the hold ramp and the plan it
    # replaced agree well inside the 0.06 rev bound at the live tau, so the
    # joined plan would have passed and silently re-armed the cancelled throw.
    resp = out['resp']
    assert resp.accepted is False, resp.message
    assert resp.code == tn._SUPERSEDED_BY_HOLD
    assert 'hold' in resp.message
    assert node._active_plan is hold_plan
    assert not isinstance(node._active_plan, CyclePlan)


def test_the_epoch_guard_and_not_the_continuity_check_is_what_refuses():
    """Non-vacuity: the plan the epoch refuses would have passed the continuity gate.

    Stated as a separate measurement because it is the whole reason the epoch
    exists. If a post-hold cycle install were already refused ``STALE_STATE``,
    the epoch would be ceremony; it is not. A hold is a profiled decel from the
    live commanded state, so immediately after it the hold ramp and the plan it
    replaced still agree at the live ``tau`` — well inside the guard's 0.06 rev
    leg bound — and the joined plan installs cleanly.
    """
    node = _mode_and_seed(_cycle_node())
    assert node._svc_plan_cycle(_steady_chain_req(),
                                PlanCycle.Response()).accepted is True
    plan, meta, t0 = node._cycle

    # Hold, exactly as the coordinator's give-up path does.
    assert node._svc_hold(Trigger.Request(), Trigger.Response()).success is True
    hold_plan = node._active_plan

    # The cycle plan the in-flight solve was about to install is STILL continuous
    # against what the hold put on the wire...
    tau_now = time.perf_counter() - t0
    assert node._install_continuity_ok(plan, tau_now) is True
    # ...so nothing but the epoch stands between it and the wire.
    epoch_now = node._install_epoch
    assert node._install(plan, t0=t0, require_epoch=epoch_now - 1) is False
    assert node._active_plan is hold_plan
    # And a caller carrying the CURRENT epoch is unaffected — the guard refuses
    # stale solves, not every install.
    assert node._install(plan, t0=t0, require_epoch=epoch_now) is True
    assert node._active_plan is plan


def test_the_hold_swaps_the_plan_and_bumps_the_epoch_under_ONE_acquisition():
    """The bump must not be a SECOND ``_plan_lock`` block after the install.

    The window it left was small and fatal. A solve that had already passed
    ``_hold_superseded`` and ``_install_continuity_ok`` — which passes right after
    a hold, as the sibling test measures — sits blocked on ``_plan_lock`` inside
    ``_install(..., require_epoch=epoch0)``. With the bump in a separate
    acquisition it could take the lock between the hold's swap and the hold's
    bump, read the OLD epoch, and put the release-terminal plan back on the wire.
    The hold then bumped and returned ``success=True``: the operator's cancel
    silently undone and the throw re-armed, with a success response to say so.

    Driven by parking a thread on ``_plan_lock`` for the whole hold and releasing
    it after: whatever the parked install sees, it must see a state in which the
    swap and the bump have BOTH happened.
    """
    node = _mode_and_seed(_cycle_node())
    assert node._svc_plan_cycle(_steady_chain_req(),
                                PlanCycle.Response()).accepted is True
    cycle_plan, _meta, t0 = node._cycle
    epoch0 = node._install_epoch

    parked = threading.Event()
    go = threading.Event()
    out = {}

    def _parked_install():
        # Hold the lock so the hold's install must wait for it, then let go and
        # immediately attempt the stale install — the losing side of the race.
        with node._plan_lock:
            parked.set()
            assert go.wait(timeout=10.0)
        out['installed'] = node._install(cycle_plan, t0=t0,
                                         require_epoch=epoch0)

    worker = threading.Thread(target=_parked_install, daemon=True)
    worker.start()
    assert parked.wait(timeout=10.0)
    go.set()
    assert node._svc_hold(Trigger.Request(),
                          Trigger.Response()).success is True
    hold_plan = node._active_plan
    worker.join(timeout=10.0)
    assert not worker.is_alive()

    # The stale install was refused, whichever order the two threads got the
    # lock in: if it ran first the hold overwrote it and bumped; if it ran
    # second it saw the bumped epoch. Either way the hold is what is streaming.
    assert node._install_epoch == epoch0 + 1
    assert node._active_plan is hold_plan
    assert not isinstance(node._active_plan, CyclePlan)
    if out['installed']:
        # It won the lock first — then the hold's own install came after it.
        assert node._active_plan is not cycle_plan


def test_a_hold_racing_the_cycle_bookkeeping_leaves_no_stale_record():
    """``self._cycle`` is written INSIDE the install's lock block, not after it.

    Written on the next line instead, a hold landing in that window sets
    ``self._cycle = None`` (a non-cycle install clears the record) and the caller
    immediately wrote it straight back — leaving the node believing it holds a
    cycle whose plan is not on the wire. The next EXTEND would then chain onto a
    release that is never going to happen.

    Driven at ``_install`` directly, because the window being closed is one line
    wide and a timing test could not reliably land in it.
    """
    node = _mode_and_seed(_cycle_node())
    assert node._svc_plan_cycle(_steady_chain_req(),
                                PlanCycle.Response()).accepted is True
    cycle = node._cycle
    assert cycle is not None

    # A hold clears the record...
    assert node._svc_hold(Trigger.Request(), Trigger.Response()).success is True
    assert node._cycle is None
    # ...and a stale cycle install cannot put it back, because the record travels
    # WITH the install and the install is refused.
    plan, meta, t0 = cycle
    assert node._install(plan, t0=t0, require_epoch=node._install_epoch - 1,
                         cycle=cycle, reset_replans=True) is False
    assert node._cycle is None
    assert not isinstance(node._active_plan, CyclePlan)


def test_a_hold_racing_the_cycle_unpack_does_not_raise():
    """A cleared ``_cycle`` between check and unpack would take the node down.

    ``_plan_cycle_extend`` / ``_plan_cycle_replan`` / ``_on_dynamic_target`` all
    used to test ``self._cycle is not None`` and then unpack it. With the hold
    reentrant, a hold in between makes that unpack a ``TypeError`` — which
    ``_svc_plan_cycle``'s except ladder does not catch (it catches
    ``CycleInfeasible``, ``ValueError`` and ``TrajectoryInfeasible``), so Foxy's
    MultiThreadedExecutor re-raises, ``executor.spin()`` unwinds into ``main()``'s
    finally, the emitter thread stops and the Teensy setpoint watchdog E-STOPs.
    A refusal is the correct outcome; a crash is not.

    Each path is driven with the record cleared at the moment the handler reads
    it, which is exactly what a concurrent hold does.
    """
    node = _mode_and_seed(_cycle_node())
    assert node._svc_plan_cycle(_steady_chain_req(),
                                PlanCycle.Response()).accepted is True

    for mode, kind in ((PlanCycle.Request.MODE_EXTEND,
                        PlanCycle.Request.KIND_STEADY),
                       (PlanCycle.Request.MODE_REPLAN,
                        PlanCycle.Request.KIND_LANDING)):
        node._cycle = None                       # what a concurrent hold leaves
        _refresh(node)
        req = _launch_req()
        req.mode, req.kind = mode, kind
        req.period_s, req.catch_frac = 1.4, 0.6 / 1.4
        resp = node._svc_plan_cycle(req, PlanCycle.Response())
        assert resp.accepted is False
        assert resp.code == 'NO_CYCLE', resp.message

    # The dynamic_target path takes its own snapshot and must fall through to the
    # LEGACY reach rather than unpacking None.
    node._cycle = None
    node._catch_armed = False
    _refresh(node)
    from tests.ros.test_unified_cycle_integration import _dyn_msg
    node._on_dynamic_target(_dyn_msg(x=5.0))     # no exception is the assertion


def test_a_move_service_cannot_install_over_a_hold_either():
    """The epoch guard covers every planning install, not just the cycle ones.

    `go_to_pose`, `go_home`, `timed_target` and the catch install were all
    serialized behind the hold before the reentrant group existed. Each now takes
    hundreds of ms during which a hold can land, and a move installed over a hold
    undoes the operator's cancel exactly as a cycle would. `go_to_pose` stands for
    the group here; the wiring is identical in the other three.
    """
    node = _mode_and_seed(_cycle_node())
    assert node._svc_hold(Trigger.Request(), Trigger.Response()).success is True
    hold_plan = node._active_plan
    stale = node._install_epoch - 1
    # A plan built before that hold cannot install over it.
    assert node._install(hold_plan, require_epoch=stale) is False
    assert node._active_plan is hold_plan
    # The four handlers all capture the epoch at entry — pinned by source so a
    # new planning service cannot quietly join without it.
    import inspect
    for fn in (node._svc_go_home, node._svc_go_to_pose,
               node._plan_and_install_timed, node._plan_and_install_catch):
        src = inspect.getsource(fn)
        assert 'epoch0 = self._install_epoch' in src, fn.__name__
        assert 'require_epoch=epoch0' in src, fn.__name__


def test_the_hold_epoch_moves_only_on_a_hold():
    """Only the callback that can interleave bumps the epoch.

    Every other install runs on the serialized group, so it cannot land during a
    solve and cannot invalidate one. Bumping on those too would refuse installs
    for a race that cannot happen — and a guard that fires when nothing is wrong
    is how guards get switched off.
    """
    node = _mode_and_seed(_cycle_node())
    start = node._install_epoch
    assert node._svc_plan_cycle(_launch_req(),
                                PlanCycle.Response()).accepted is True
    assert node._install_epoch == start          # a cycle install does not bump
    node._install_graceful_stop('test')
    assert node._install_epoch == start          # nor does a graceful stop
    assert node._svc_hold(Trigger.Request(), Trigger.Response()).success is True
    assert node._install_epoch == start + 1      # the hold does
