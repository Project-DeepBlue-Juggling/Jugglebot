"""UH-7a — the unified STEADY ring, on the COORDINATOR side.

The chaining PRIMITIVES were written and tested long before anything called
them: ``unified_cycle.extend``, ``MODE_EXTEND``, ``KIND_STEADY``, the supersede
deadline and the ``CHAIN_SKEW`` guard all shipped commented "Phase 5's UH-7
ring". What was missing was the *shape* (chain a STEADY, not a LANDING) and the
*release hand-off* (the FSM is TOLD when the plan throws instead of deriving
``now + throw_delay_s`` from whenever the START_CYCLE poll happened). This file
pins the shape and the hand-off, end to end on the coordinator, and it pins the
three ways they can go wrong.

**What "end to end" means here, and what it does not.** The session sequencer,
``_build_toss_cycle``, ``_tick_unified_launch``, ``_tick_unified_extend``, the
request builder, the fail-safe ladder and the beat arithmetic are all REAL. Two
things are stubbed: ``trajectory/plan_cycle`` (a fake planner that answers with
the same release/deadline arithmetic the real service computes, so no QP runs
and the requests are inspectable), and ``_run_toss_cycle``'s FSM ladder, which is
replaced by a tick-by-tick replay of the unified seams on the coordinator's own
40 ms grid. The FSM's own gates are covered separately at the bottom of this
file, driven directly.

⚠ The mocked-ROS harness (``tests/ros/conftest.py``) makes ``time.sleep`` a
no-op, so every wait in here is bounded by an iteration count as well as by a
deadline: a "wait until t" loop against a real deadline would be a CPU spin.
"""

from __future__ import annotations

import types

import pytest

import jugglebot.hardware_config as hw
from jugglebot import reload_coordinator_node as rcn
from jugglebot.motion import unified_cycle as uc
from jugglebot.toss_sequencer import TossResult, TossSequencer
from jugglebot import toss_session as tsess
from jugglebot.toss_session import TossSessionSequencer
from jugglebot_interfaces.srv import PlanCycle

from tests.ros.test_toss_continuous_node import (
    _Clock,
    _ContGoalHandle,
    _ready_node,
    _stamp,
)
from tests.ros.test_toss_sequencer import _obs


FLIGHT = float(hw.JB_OP_TOSS_FLIGHT_TIME_DEFAULT_S)     # 0.8 s
DWELL = 1.2                                             # beat 2.0 s
DELAY = 1.0                                             # spin-up margin
BEAT = FLIGHT + DWELL
KNOT_DT_S = 0.025                                       # the emitter's knot grid
#: How long after the plan's catch instant this harness lets the verdict land.
#: 0.30 s sits inside the measured 0.202-0.442 s band the rest of the suite uses.
VERDICT_LATENCY_S = 0.30


# ── the fake planner ──────────────────────────────────────────────────────────

class _Planner:
    """A stand-in for ``trajectory/plan_cycle`` that answers with the SAME
    arithmetic the real service does, and records every request.

    Three properties are reproduced because the ring's correctness depends on
    them and on nothing else the QP does:

    * ``t_release_mono`` is **the first release still AHEAD of now**, not
      ``releases[0]``. That search is why a chained cycle is handed its own
      release rather than a spent one, and reproducing it is what makes the
      hand-off assertions mean anything;
    * ``supersede_deadline_mono`` is ``t0 + duration − dt`` for a plan that ends
      at a release, and ``0.0`` for one that ends at rest — the release-terminal
      cliff, which is exactly what the extend races;
    * an EXTEND keeps the ORIGIN (``t0``) and appends its window, so the joined
      plan's releases accumulate on one clock.
    """

    def __init__(self, clock, *, refuse_kinds=(), unacked_kinds=()):
        self.clock = clock
        self.requests = []
        self.t0 = None
        self.releases = []          # absolute perf instants
        self.end = None             # absolute instant the plan runs out
        self.release_terminal = False
        # Keyed by (mode, kind) OR by bare kind, so a test can refuse "every
        # STEADY" or "only the EXTENDED one" — the difference matters, because
        # refusing the launch's own chained window means the ring never starts
        # and there is nothing for the fall-back to fall back FROM.
        self.refuse_kinds = set(refuse_kinds)
        self.unacked_kinds = set(unacked_kinds)
        #: Seconds of fake-clock solve latency an EXTEND future takes to resolve.
        #: 0.0 answers on the dispatching tick; the measured MODE_EXTEND is
        #: 354-387 ms, i.e. ~9-10 coordinator ticks, and the whole point of the
        #: non-blocking conversion is that the FSM keeps ticking through them.
        self.extend_latency_s = 0.0
        #: Futures handed out by `client`, newest last — a test that abandons one
        #: resolves it here to prove the answer is DISCARDED rather than acted on.
        self.futures = []
        #: Seconds of fake-clock latency the BLOCKING seam (`__call__`) takes
        #: before it answers — the LAUNCH's own solve. 2026-09-09: 1.15-3.3 s
        #: at a 3.6 s beat, against a cycle-1 schedule that had assumed 0.606.
        self.launch_latency_s = 0.0
        #: The `timeout_s` each blocking call was given, in order.
        self.timeouts = []

    # ── the two faces of this stub ───────────────────────────────────────────
    # `__call__` is the BLOCKING seam (`_call_plan_cycle`), still used by the
    # LAUNCH and the floor lift. `client` is the ASYNC seam the extend now uses.
    # Both run the same state machine, so a test never has to know which path a
    # request took to reason about the plan it produced.

    def client(self):
        """A stand-in for ``node._plan_cycle_cli``."""
        planner = self

        class _Fut:
            def __init__(self, req, due):
                self.req, self.due, self._resp = req, due, None

            def done(self):
                return float(planner.clock.t) >= self.due

            def result(self):
                # Computed at RESOLUTION time, not at dispatch: the real service
                # installs when it runs, so the release instants and the
                # supersede deadline belong to the instant it answered.
                if self._resp is None:
                    self._resp = planner._serve(self.req)[0]
                return self._resp

        class _Cli:
            ready = True

            def service_is_ready(_self):
                return _Cli.ready

            def wait_for_service(_self, timeout_sec=None):
                return _Cli.ready

            def call_async(_self, req):
                planner.requests.append(req)
                key = (req.mode, req.kind)
                # An UNACKED request is a future that NEVER resolves — which is
                # what "dispatched and did not answer" actually looks like on the
                # wire, and is what the poll's `_SERVICE_WAIT_S` bound is for. It
                # is not a refusal answered late.
                due = (float('inf')
                       if (key in planner.unacked_kinds
                           or req.kind in planner.unacked_kinds)
                       else float(planner.clock.t)
                       + float(planner.extend_latency_s))
                fut = _Fut(req, due)
                planner.futures.append(fut)
                return fut

        return _Cli()

    # The signature `_call_plan_cycle` presents: (req) -> (response, dispatched)
    def __call__(self, req, **kw):
        self.requests.append(req)
        self.timeouts.append(kw.get('timeout_s'))
        if self.launch_latency_s > 0.0:
            self.clock.sleep(self.launch_latency_s)
        return self._serve(req)

    def _serve(self, req):
        key = (req.mode, req.kind)
        if key in self.unacked_kinds or req.kind in self.unacked_kinds:
            return None, True
        if key in self.refuse_kinds or req.kind in self.refuse_kinds:
            return self._resp(accepted=False, code='INFEASIBLE',
                              message='REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: '
                                      'stubbed refusal)'), True
        now = float(self.clock.t)
        if req.mode == PlanCycle.Request.MODE_NEW:
            self.t0 = now
            self.releases = []
            self.end = now
            if req.kind == PlanCycle.Request.KIND_LAUNCH:
                self.end = now + float(req.period_s)
                self.releases.append(self.end)
                self.release_terminal = True
            if bool(req.chain):
                self.end = self.end + float(req.chain_period_s)
                if req.chain_kind == PlanCycle.Request.KIND_STEADY:
                    self.releases.append(self.end)
                    self.release_terminal = True
                else:
                    self.release_terminal = False
        elif req.mode == PlanCycle.Request.MODE_EXTEND:
            if self.t0 is None:
                return self._resp(accepted=False, code='NO_CYCLE',
                                  message='REJECTED_CYCLE_PLAN(NO_CYCLE)'), True
            self.end = self.end + float(req.period_s)
            if req.kind == PlanCycle.Request.KIND_STEADY:
                self.releases.append(self.end)
                self.release_terminal = True
            else:
                self.release_terminal = False
        else:                                    # SETTLE lifts, replans, …
            return self._resp(accepted=True), True
        return self._resp(accepted=True), True

    def _resp(self, *, accepted, code='', message=''):
        now = float(self.clock.t)
        ahead = [r for r in self.releases if r > now]
        t_rel = ahead[0] if ahead else (self.releases[-1] if self.releases
                                        else 0.0)
        return types.SimpleNamespace(
            accepted=bool(accepted), code=code, message=message,
            t_release_mono=float(t_rel),
            # The catch side has no "still ahead" search in the real service
            # either (§ 4.4); the ring never reads it, and answering 0.0 here is
            # what stops a test from accidentally depending on one.
            t_catch_mono=0.0,
            release_terminal=bool(self.release_terminal),
            supersede_deadline_mono=(float(self.end) - KNOT_DT_S
                                     if self.release_terminal else 0.0),
            duration_s=float((self.end or 0.0) - (self.t0 or 0.0)),
            plan_wall_ms=180.0,
            release_vel_mm_s=[0.0, 0.0, 3900.0],
            hand_peak_vel_rps=0.0, hand_peak_acc_rps2=0.0)


def _dispatched(node, monkeypatch, *, resolve=None):
    """Record what the extend DISPATCHES, without answering it.

    The extend is non-blocking now, so "was a request sent" is a question about
    ``_plan_cycle_cli.call_async`` rather than about ``_call_plan_cycle``.
    ``resolve`` is the response each future eventually carries (``None`` ⇒ the
    future never resolves, which is what a dispatched-and-unanswered request
    actually looks like and is the input the poll's ``_SERVICE_WAIT_S`` bound is
    written against).
    """
    sent = []

    class _Fut:
        def done(self):
            return resolve is not None

        def result(self):
            return resolve

    class _Cli:
        def service_is_ready(self):
            return True

        def wait_for_service(self, timeout_sec=None):
            return True

        def call_async(self, req):
            sent.append(req)
            return _Fut()

    monkeypatch.setattr(node, '_plan_cycle_cli', _Cli())
    return sent


def _cycle_requests(planner):
    """Only the LAUNCH/STEADY/LANDING chain requests — the floor lift's
    ``KIND_SETTLE`` window and the warm-up are session-start plumbing, not the
    ring, and a test that had to skip them by index would break the first time
    one moved."""
    return [r for r in planner.requests
            if r.kind in (PlanCycle.Request.KIND_LAUNCH,
                          PlanCycle.Request.KIND_STEADY,
                          PlanCycle.Request.KIND_LANDING)]


# ── the harness ───────────────────────────────────────────────────────────────

def _unified_goal(*, num_throws=3, dwell=DWELL, delay=DELAY, **kw):
    gh = _ContGoalHandle(num_throws=num_throws, dwell=dwell, delay=delay, **kw)
    gh.request.unified_cycle = True
    return gh


def _ring_node(monkeypatch, clock, planner, *, lifts=None):
    """A node with every unified session precondition satisfied and the planner
    stubbed. Returns the node."""
    monkeypatch.setattr(rcn, 'time', clock)
    monkeypatch.setattr(rcn.hw, 'JB_OP_UNIFIED_CYCLE_ENABLED', True,
                        raising=False)
    # The pipeline is forced off by the session's own single resolution under
    # unified, but pinning it here says which machine this file drives rather
    # than inheriting it from the shipped YAML (the 2026-08-28 lesson).
    monkeypatch.setattr(rcn.hw, 'JB_OP_TOSS_PIPELINE_ENABLED', False,
                        raising=False)
    node = _ready_node(clock)
    # The hand seed is part of the fixture, not an incidental default:
    # `_ready_node` parks it at the homed 0.0 rev, which is ~10 mm BELOW the
    # planner's cup floor, and every LAUNCH would then be refused HAND_BELOW_FLOOR
    # before it solved. Seating it AT the floor is what a machine parked ready
    # actually looks like; the refusal itself is `tests/ros/test_unified_launch_floor.py`'s.
    with node._lock:
        node._hand_pos_meas = uc.hand_rev_for_cup_z(uc.SETTLE_CUP_Z_MM)
    monkeypatch.setattr(node, '_unified_warm_planner', lambda: 0.0)
    # Both seams, one state machine: the LAUNCH and the floor lift still go
    # through the blocking `_call_plan_cycle`, the EXTEND through the client.
    monkeypatch.setattr(node, '_call_plan_cycle', planner)
    monkeypatch.setattr(node, '_plan_cycle_cli', planner.client())
    if lifts is not None:
        monkeypatch.setattr(
            node, '_unified_floor_lift',
            lambda why: (lifts.append(why), '')[1])
    else:
        monkeypatch.setattr(node, '_unified_floor_lift', lambda why: '')
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: 'held (stub)')
    return node


def _install_ring_replay(node, monkeypatch, clock, *, outcomes,
                         max_ticks=4000):
    """Replace ``_run_toss_cycle`` with a tick-by-tick replay of the UNIFIED
    seams, and record what each cycle did.

    This is NOT ``_stub_cycles`` with extra steps: that helper skips straight to
    the verdict, which is exactly where the ring's plumbing lives. Here the two
    polled seams (``_tick_unified_launch`` at the top of a tick,
    ``_tick_unified_extend`` on every later one) run for real, on the
    coordinator's own ``_PACE_PERIOD_S`` grid, against the fake clock. What is
    NOT replayed is the FSM's own ladder — its gates are driven directly further
    down this file — so a trace row describes what the NODE did in a cycle, not
    what the sequencer decided.
    """
    trace = []
    pending = list(outcomes)

    def fake_run(seq, *, deadline_s, cancel_now_fn, feedback_fn, state=None):
        st = node._toss_committed if state is None else state
        row = {
            'seq': seq,
            'chained': bool(seq.chained),
            'release_at_perf': float(seq.release_at_perf),
            't_release': float(seq.t_release),
            'announced_at': None, 'plan_release': None,
            'extend_at': None, 'live_catch': None, 'extend_deadline': None,
            # How many FSM ticks the cycle took AFTER dispatching its extend.
            # Under the blocking wait this was zero by construction — the thread
            # was inside the service call — so it is the direct measurement of
            # what the non-blocking conversion bought.
            'ticks_after_extend': 0,
            'pending_at_verdict': False,
            'reject': '',
        }
        # ── PREPARE → ANNOUNCE: the FSM arms the deferral, the node's polled
        # seam does the planning (or, on a chain, only the announcing).
        st.unified_launch_pending = True
        for _ in range(max_ticks):
            if not st.unified_launch_pending:
                break
            node._tick_unified_launch(seq, st, clock.t)
            if st.unified_launch_pending:
                clock.sleep(rcn._PACE_PERIOD_S)
        else:                                          # pragma: no cover
            pytest.fail('the launch seam never resolved')
        if st.unified_reject:
            row['reject'] = st.unified_reject
            trace.append(row)
            return TossResult(False, st.unified_reject), 'fsm'
        row['announced_at'] = clock.t
        plan = st.unified_plan
        assert plan is not None, 'an accepted launch left no plan on the cycle'
        t_rel = float(plan.t_release_mono)
        row['plan_release'] = t_rel
        row['live_catch'] = t_rel + float(seq.flight_time_s)
        row['extend_deadline'] = float(
            getattr(plan, 'supersede_deadline_mono', 0.0) or 0.0)
        # ── the flight and the settle, ticked to the verdict ──
        stop = row['live_catch'] + VERDICT_LATENCY_S
        for _ in range(max_ticks):
            if clock.t >= stop:
                break
            was = st.unified_extended
            node._tick_unified_extend(seq, st, clock.t)
            if st.unified_extended and not was:
                row['extend_at'] = clock.t
            elif row['extend_at'] is not None:
                row['ticks_after_extend'] += 1
            clock.sleep(rcn._PACE_PERIOD_S)
        else:                                          # pragma: no cover
            pytest.fail('the cycle never reached its verdict')
        row['pending_at_verdict'] = node._toss_unified_extend is not None
        _stamp(node, clock.t)
        trace.append(row)
        result = pending.pop(0)
        if not result.success:
            # A MISS routes through the FSM's SAFE_ABORT terminal, which is the
            # node's `_toss_safe_abort` — the single seam that decides what a
            # unified teardown owes. Replaying it is the whole point of the
            # MISS test below.
            node._toss_safe_abort(st, settle=False)
        # A CAUGHT cycle's still-in-flight extend is settled by the SESSION
        # LOOP, immediately after `_run_toss_cycle` returns — which is outside
        # this replacement, so it is production's own call and not the harness's.
        if st.unified_reject:
            result = TossResult(False, st.unified_reject)
        return result, 'fsm'

    monkeypatch.setattr(node, '_run_toss_cycle', fake_run)
    return trace


def _caught():
    return TossResult(True, 'CAUGHT', 2.0, FLIGHT)


def _missed():
    return TossResult(False, 'MISSED', float('nan'), float('nan'))


# ═════════════════════════════════════════════════════════════════════════════
# 1. THE RING, as the coordinator asks for it
# ═════════════════════════════════════════════════════════════════════════════

def test_a_three_throw_ring_asks_LAUNCH_chain_STEADY_then_EXTEND_STEADY_then_EXTEND_LANDING(
        monkeypatch):
    """THE shape, stated as the exact request sequence.

    Three throws is the smallest ring that exercises every rung — the launch's
    chained window, an extend that must chain ANOTHER steady, and an extend that
    must chain the landing — and the indexing between them is where a ring goes
    wrong. Each window carries the NEXT cycle's catch and ends at the cycle after
    that one's release, so the count that chooses STEADY-vs-LANDING is one index
    further along at the extend than it is at the launch. Getting that off by one
    either throws a ball with no window planned to catch it (too few STEADYs) or
    leaves a terminal release nobody throws (too many).
    """
    clock = _Clock()
    planner = _Planner(clock)
    node = _ring_node(monkeypatch, clock, planner)
    _install_ring_replay(node, monkeypatch, clock,
                         outcomes=[_caught()] * 3)
    result = node._execute_toss_continuous(_unified_goal(num_throws=3))
    assert result.outcome == 'COMPLETED', result.outcome

    reqs = _cycle_requests(planner)
    assert [(r.mode, r.kind) for r in reqs] == [
        (PlanCycle.Request.MODE_NEW, PlanCycle.Request.KIND_LAUNCH),
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_STEADY),
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_LANDING),
    ]
    launch = reqs[0]
    # The launch chains a STEADY, at the SESSION's beat, with the catch a flight
    # in — the window cycle 1 catches in and cycle 2 releases from.
    assert launch.chain is True
    assert launch.chain_kind == PlanCycle.Request.KIND_STEADY
    assert launch.period_s == pytest.approx(rcn._UNIFIED_LAUNCH_WINDOW_S)
    assert launch.chain_period_s == pytest.approx(BEAT)
    assert launch.chain_catch_frac == pytest.approx(FLIGHT / BEAT)
    steady = reqs[1]
    assert steady.period_s == pytest.approx(BEAT)
    assert steady.catch_frac == pytest.approx(FLIGHT / BEAT)
    landing = reqs[2]
    # A LANDING catches and then STOPS: one flight plus a launch window's worth
    # of decel, which is the shortest shape that does both.
    assert landing.period_s == pytest.approx(
        FLIGHT + rcn._UNIFIED_LAUNCH_WINDOW_S)
    assert landing.catch_frac == pytest.approx(
        FLIGHT / (FLIGHT + rcn._UNIFIED_LAUNCH_WINDOW_S))


def test_one_throw_asks_for_exactly_the_pre_ring_request(monkeypatch):
    """``num_throws == 1`` is BIT-IDENTICAL to the shape the 2026-09-04 ladder
    flew nine clean rows on, and that is deliberate rather than incidental.

    A one-throw session owes no second release, so there is nothing for a STEADY
    to hand on and the LANDING is not a fallback — it is the right answer. Making
    the ring's arrival cost the single toss its proven request would be trading a
    validated shape for an unvalidated one at the exact moment there is no reason
    to.
    """
    clock = _Clock()
    planner = _Planner(clock)
    node = _ring_node(monkeypatch, clock, planner)
    _install_ring_replay(node, monkeypatch, clock,
                         outcomes=[_caught()])
    result = node._execute_toss_continuous(_unified_goal(num_throws=1))
    assert result.outcome == 'COMPLETED', result.outcome
    reqs = _cycle_requests(planner)
    assert len(reqs) == 1, 'a single toss chained something'
    launch = reqs[0]
    assert (launch.mode, launch.kind) == (PlanCycle.Request.MODE_NEW,
                                          PlanCycle.Request.KIND_LAUNCH)
    assert launch.chain is True
    assert launch.chain_kind == PlanCycle.Request.KIND_LANDING
    assert launch.chain_period_s == pytest.approx(
        FLIGHT + rcn._UNIFIED_LAUNCH_WINDOW_S)
    assert launch.chain_catch_frac == pytest.approx(
        FLIGHT / (FLIGHT + rcn._UNIFIED_LAUNCH_WINDOW_S))


# ═════════════════════════════════════════════════════════════════════════════
# 2. THE RELEASE HAND-OFF
# ═════════════════════════════════════════════════════════════════════════════

def test_a_chained_cycle_is_TOLD_its_release_by_the_plan(monkeypatch):
    """The plan is the authority on when the hand throws.

    Cycle 1 derives ``now + throw_delay_s`` because it really does decide when to
    throw — its LAUNCH has not been planned yet. Every later cycle does NOT: its
    release is already a knot the emitter is streaming toward, and deriving a
    second opinion from whenever the START_CYCLE poll happened would put the
    FSM's landing schedule, the hand ball sensor's asymmetric arrival band and
    the release-grace deadline a poll's lateness off the ball. So the chained
    cycle is started with ``release_at_perf`` taken verbatim from the extend
    response, and the beat between consecutive planned releases is then the
    session's own — exactly, not to within a tick.
    """
    clock = _Clock()
    planner = _Planner(clock)
    node = _ring_node(monkeypatch, clock, planner)
    trace = _install_ring_replay(node, monkeypatch, clock,
                                 outcomes=[_caught()] * 3)
    assert node._execute_toss_continuous(
        _unified_goal(num_throws=3)).outcome == 'COMPLETED'

    assert [row['chained'] for row in trace] == [False, True, True]
    # Cycle 1: derived, so no absolute release was handed in.
    assert trace[0]['release_at_perf'] == 0.0
    for row in trace[1:]:
        # …and every later cycle's FSM release IS the plan's, to the float.
        assert row['release_at_perf'] == row['t_release']
        assert row['release_at_perf'] == row['plan_release']
    # The beat the machine actually flew, release to release.
    releases = [row['plan_release'] for row in trace]
    gaps = [b - a for a, b in zip(releases, releases[1:])]
    assert gaps == pytest.approx([BEAT] * len(gaps), abs=1e-9), gaps


def test_the_beat_is_hoisted_once_and_both_sides_read_the_same_number():
    """A4 — one number, two readers, one identity.

    The session schedules the next cycle from ``next_release_at`` (landing +
    dwell) while the coordinator asks the planner for a window of
    ``_unified_beat_s`` (the session's ``beat_s``). Those are the two sides the
    CHAIN_SKEW guard compares every beat, so they must be the same cadence
    stated twice rather than two cadences that happen to agree: release → release
    is landing → release plus a flight, which is the identity below.
    """
    session = TossSessionSequencer(
        num_throws=3, dwell_time_s=DWELL, throw_delay_s=DELAY,
        flight_time_s=FLIGHT, ilc_speed_trim_possible=False)
    assert session.beat_s == pytest.approx(FLIGHT + DWELL)
    landing = 1234.5
    assert (session.next_release_at(landing) - landing
            == pytest.approx(session.beat_s - session.flight_time_s))
    # The dwell DEFAULT is substituted before the beat is taken, so a goal that
    # left dwell unset gets the beat the session will actually run — not a beat
    # built from the 0.0 sentinel.
    unset = TossSessionSequencer(
        num_throws=3, dwell_time_s=0.0, throw_delay_s=DELAY,
        flight_time_s=FLIGHT, ilc_speed_trim_possible=False)
    assert unset.beat_s == pytest.approx(
        FLIGHT + float(hw.JB_OP_TOSS_SESSION_DWELL_DEFAULT_S))


def test_the_coordinators_beat_is_the_sessions_beat(monkeypatch):
    """``_unified_beat_s`` READS the hoisted number rather than re-adding it.

    Two additions of the same two floats is the shape that produces a skew nobody
    can account for — and on this path a skew is not a rounding curiosity, it is
    the CHAIN_SKEW refusal that stops the session."""
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    session = TossSessionSequencer(
        num_throws=3, dwell_time_s=DWELL, throw_delay_s=DELAY,
        flight_time_s=FLIGHT, ilc_speed_trim_possible=False)
    node._toss_session_ref = session
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY,
                        unified=True)
    assert node._unified_beat_s(seq) == session.beat_s


def test_a_chain_the_session_disagrees_with_is_REFUSED_not_announced(
        monkeypatch):
    """The cross-check, and why its tolerance is one TICK.

    With the hand-off in force the two numbers are the same float, so the honest
    tolerance is the jitter of the tick that scheduled them and nothing more.
    Sizing it at the extend lead instead would admit 0.60 s of disagreement about
    when the ball leaves — most of a flight — and the FSM's landing schedule is
    what the hand ball sensor's 0.087 s arrival band is cut from, so a skew that
    large mints MISSED on a caught ball. Refusing is the only honest answer: a
    disagreement means the session re-scheduled under the plan.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY,
                        unified=True)
    seq.start(clock.t)
    seq._prepare_dispatched = True
    node._toss_unified_live = True
    state = node._toss_committed
    state.unified_launch_pending = True
    skew = rcn._UNIFIED_CHAIN_SKEW_TOL_S * 2.0
    node._toss_unified_chain = types.SimpleNamespace(
        t_release_mono=seq.t_release + skew, release_vel_mm_s=[0, 0, 3900.0],
        plan_wall_ms=1.0, duration_s=6.0, t_catch_mono=0.0,
        release_terminal=True, supersede_deadline_mono=seq.t_release + 10.0)
    monkeypatch.setattr(
        node, '_call_plan_cycle',
        lambda req: pytest.fail('a skewed chain fell through to a NEW launch'))
    node._tick_unified_launch(seq, state, clock.t)
    assert 'CHAIN_SKEW' in state.unified_reject
    assert node._publishers['throw_announcements'].published == []
    # …and a skew INSIDE the tick is noise, not a disagreement.
    state.unified_reject = ''
    state.unified_launch_pending = True
    node._toss_unified_chain = types.SimpleNamespace(
        t_release_mono=seq.t_release + rcn._UNIFIED_CHAIN_SKEW_TOL_S * 0.5,
        release_vel_mm_s=[0, 0, 3900.0], plan_wall_ms=1.0, duration_s=6.0,
        t_catch_mono=0.0, release_terminal=True,
        supersede_deadline_mono=seq.t_release + 10.0)
    node._tick_unified_launch(seq, state, clock.t)
    assert state.unified_reject == ''
    assert len(node._publishers['throw_announcements'].published) == 1


# ═════════════════════════════════════════════════════════════════════════════
# 3. WHEN THE EXTEND FIRES
# ═════════════════════════════════════════════════════════════════════════════

def test_every_extend_lands_after_the_live_catch_and_before_the_deadline(
        monkeypatch):
    """The trigger, from both ends.

    AFTER the live catch, because a replan re-aims that catch through the same
    single ``_cycle`` install an extend replaces — asking earlier is two writers
    on one record, and asking after it is free (the whole dwell is ahead).
    BEFORE ``deadline − lead``, because past the deadline the emitter reads the
    release segment's endpoint from the plan's terminal HOLD and commands the
    throw to a stop, with no firmware guard to say so.

    ⚠ The supersede deadline arms for the FIRST time on this shape. Its alarm
    firing is a PLUMBING defect report, not an acceptance signal — which is why
    this asserts the ordering on every cycle rather than on average.
    """
    clock = _Clock()
    planner = _Planner(clock)
    node = _ring_node(monkeypatch, clock, planner)
    trace = _install_ring_replay(node, monkeypatch, clock,
                                 outcomes=[_caught()] * 3)
    assert node._execute_toss_continuous(
        _unified_goal(num_throws=3)).outcome == 'COMPLETED'
    extending = [row for row in trace if row['extend_at'] is not None]
    assert len(extending) == 2, 'a 3-throw ring chains exactly two windows'
    for row in extending:
        assert row['extend_at'] >= row['live_catch'], (
            'the extend fired %.3f s BEFORE the live catch — inside the window '
            'a catch-side replan can still re-install'
            % (row['live_catch'] - row['extend_at'],))
        assert row['extend_at'] <= (row['extend_deadline']
                                    - rcn._UNIFIED_EXTEND_LEAD_S
                                    + rcn._PACE_PERIOD_S), (
            'the extend fired only %.3f s before the supersede deadline, short '
            'of its %.2f s lead'
            % (row['extend_deadline'] - row['extend_at'],
               rcn._UNIFIED_EXTEND_LEAD_S))
    # The LAST cycle owes nothing: its window is the LANDING, which is
    # rest-terminal and has no deadline to race.
    assert trace[-1]['extend_at'] is None
    assert trace[-1]['extend_deadline'] == 0.0


def test_the_deadline_backstop_fires_even_if_the_catch_instant_never_arrives(
        monkeypatch):
    """The second clause of the trigger, alone.

    A plan whose catch instant sits past its own supersede deadline (a bench
    call, a flight longer than the window) would never satisfy the catch clause —
    and the cliff would arrive anyway. The backstop is what makes the ask bounded
    on both ends rather than conditional on a catch happening.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY,
                        unified=True)
    seq.start(clock.t)
    state = node._toss_committed
    deadline = clock.t + 5.0
    state.unified_plan = types.SimpleNamespace(
        # The catch would land a full second PAST the deadline.
        t_release_mono=deadline + 1.0 - FLIGHT + 1.0,
        release_terminal=True, supersede_deadline_mono=deadline)
    sent = _dispatched(node, monkeypatch)
    node._tick_unified_extend(seq, state, deadline - rcn._UNIFIED_EXTEND_LEAD_S
                              - 0.01)
    assert sent == [], 'the backstop fired before its own lead'
    node._tick_unified_extend(seq, state, deadline - rcn._UNIFIED_EXTEND_LEAD_S
                              + 0.01)
    assert len(sent) == 1
    assert sent[0].mode == PlanCycle.Request.MODE_EXTEND


def test_every_extend_reports_its_achieved_lead_solve_and_depth(monkeypatch):
    """C3 — the three numbers an operator judges a ring by, on one line.

    A ring is otherwise INVISIBLE: ``cycle_active`` is a type test on the
    installed plan and stays true across the whole chain, so nothing on
    ``/trajectory/status`` distinguishes throw 2 from throw 12. The ACHIEVED lead
    (deadline minus install instant) is the one that says whether the chain is
    healthy — it is what ``_UNIFIED_EXTEND_LEAD_S`` budgets, and a ring whose
    achieved lead drifts toward zero is one slow solve away from the supersede
    cliff. The solve time says why it is drifting; the depth says how far the
    ring has run.
    """
    clock = _Clock()
    planner = _Planner(clock)
    node = _ring_node(monkeypatch, clock, planner)
    lines = []
    monkeypatch.setattr(node.get_logger(), 'info',
                        lambda msg, *a: lines.append(str(msg)))
    _install_ring_replay(node, monkeypatch, clock,
                         outcomes=[_caught()] * 3)
    assert node._execute_toss_continuous(
        _unified_goal(num_throws=3)).outcome == 'COMPLETED'
    chained = [ln for ln in lines if 'chained (depth' in ln]
    assert len(chained) == 2, chained
    assert 'STEADY chained (depth 2)' in chained[0], chained[0]
    # The LANDING makes the standing plan rest-terminal, so the ring's depth
    # goes back to 0 — "no chain is owed" and "the chain is 3 deep" must not
    # read the same on the console.
    assert 'LANDING chained (depth 0)' in chained[1], chained[1]
    for ln in chained:
        assert 'solve' in ln and 'before the superseded deadline' in ln


# ═════════════════════════════════════════════════════════════════════════════
# 4. A CHAINED CYCLE COMMANDS NOTHING
# ═════════════════════════════════════════════════════════════════════════════

def test_a_chained_cycle_neither_lifts_the_floor_nor_plans_a_launch(
        monkeypatch):
    """C1 — the per-cycle floor lift is a NO-OP on a chained cycle, not a refusal.

    The lift exists to put the hand inside the planner's cup box before a
    ``MODE_NEW`` LAUNCH is SEEDED from it. A chained cycle seeds nothing: its
    window was planned from the standing plan's terminal release state, which the
    QP already gated, and the hand is mid-carry between a catch and the next
    throw. Reading the floor there measures a machine in motion against a park
    height, and ACTING on the reading would command a ``KIND_SETTLE`` window over
    a plan that is streaming — one interval, two owners.
    """
    clock = _Clock()
    planner = _Planner(clock)
    lifts = []
    node = _ring_node(monkeypatch, clock, planner, lifts=lifts)
    _install_ring_replay(node, monkeypatch, clock,
                         outcomes=[_caught()] * 3)
    assert node._execute_toss_continuous(
        _unified_goal(num_throws=3)).outcome == 'COMPLETED'
    assert lifts == ['session start', 'cycle 1'], lifts
    # …and no chained cycle asked for a NEW plan of any kind.
    news = [r for r in planner.requests
            if r.mode == PlanCycle.Request.MODE_NEW]
    assert len(news) == 1 and news[0].kind == PlanCycle.Request.KIND_LAUNCH


def test_a_chained_cycle_commands_no_positioning_move(monkeypatch):
    """The platform is MID-PLAN, so POSITIONING must not reach the service.

    Not "it happens to be already there": a ``go_to_pose`` here is either refused
    BUSY — the cycle then dies REJECTED_POSITION with a ball in the cup — or
    ACCEPTED, in which case it SUPERSEDES the cycle plan and the release the FSM
    is about to announce stops existing. The FSM path is otherwise unchanged: the
    phase still runs and arrival is declared through the same
    ``note_position_noop`` seam a co-located chain uses.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    node._toss_unified_live = True
    monkeypatch.setattr(
        node._go_to_pose_cli, 'wait_for_service',
        lambda timeout_sec=None: pytest.fail(
            'a chained cycle reached trajectory/go_to_pose'))
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY,
                        unified=True, chained=True)
    seq.start(clock.t)
    seq.step(clock.t, _obs(clock.t, hand_parked=False))     # -> POSITIONING
    state = node._toss_committed
    state.chained = True
    # FAIL-CLOSED the other way on purpose: the cached decision SAYS "command the
    # move", and the chained branch must still command nothing. That is what
    # makes this a structural rule rather than a lucky tolerance.
    state.positioning_move = True
    node._position_platform_for_toss(seq, state)
    # Arrival IS declared — through the no-op seam, with nothing commanded. The
    # FSM path below POSITIONING is byte-identical to an accepted move, which is
    # the property that keeps the reach-envelope declaration and the mocap
    # cross-check window intact on a chained cycle.
    assert seq._position_result == (True, 0.0, 'ALREADY_THERE')


def test_the_builder_forces_the_positioning_decision_off_for_a_chained_cycle(
        monkeypatch):
    """…and it is forced at the SINGLE decision seam.

    The CHECKING lead floor (``pre_dispatch_budget_s``) is charged off this same
    boolean, so deciding "no move" in POSITIONING while the budget was charged
    for one is the accept-vs-runtime split the 2026-08-23 single-decision rule
    closed. One decision, one place, both readers.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    node._toss_unified_live = True
    # A catch pose the platform is NOT at, so the honest answer would be True.
    seq, state = node._build_toss_cycle(
        (120.0, 0.0, 170.0), FLIGHT, DELAY, 0.0,
        delay_is_cadence=True, release_at_perf=clock.t + 3.0, chained=True)
    assert state.positioning_move is False
    assert state.chained is True
    assert seq.chained is True
    assert seq.release_at_perf == pytest.approx(clock.t + 3.0)
    assert seq.t_release == pytest.approx(clock.t + 3.0)


def test_a_chained_cycle_is_not_refused_for_a_hand_outside_the_park_band():
    """The park band describes a kind-0 stroke no chained cycle dispatches.

    The gate asks whether the hand sits in the BOTTOM band because a kind-0 throw
    stroke commands ABSOLUTE positions from 0 rev. Under a chain there is no
    kind-0 stroke — the release is a knot on a streaming plan — and the hand is
    off the band BY CONSTRUCTION: a cup held at the 830 mm catch height is
    4.755 rev, 9.5x the 0.5 rev band. So the un-branched gate would refuse every
    chained cycle for a hazard that is not there.

    ⚠ It is NOT keyed on ``unified`` alone. Cycle 1 of a unified session plans a
    real LAUNCH from the live hand state, and there the band is exactly the right
    question — so an un-chained unified cycle is still refused.
    """
    obs = _obs(0.0, hand_parked=False)
    chained = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                            flight_time_s=FLIGHT, throw_delay_s=DELAY,
                            unified=True, chained=True, release_at_perf=10.0)
    chained.start(0.0)
    decision = chained.step(0.0, obs)
    assert not decision.done, decision.result
    assert decision.action == rcn.TOSS_ACTION_POSITION_PLATFORM
    plain = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                          flight_time_s=FLIGHT, throw_delay_s=DELAY,
                          unified=True)
    plain.start(0.0)
    refused = plain.step(0.0, obs)
    assert refused.done and 'HAND_NOT_PARKED' in refused.result.outcome


# ═════════════════════════════════════════════════════════════════════════════
# 5. THE FAIL-SAFE LADDER
# ═════════════════════════════════════════════════════════════════════════════

def test_a_refused_STEADY_extend_ends_the_ring_on_a_LANDING_not_a_cliff(
        monkeypatch):
    """Rung 2: the ring ends on a LANDING, catches the ball, and STOPS.

    A LANDING is not a retry of the same request — it is a strictly easier one (a
    window that ends at rest owes no terminal release and no next beat), so a
    planner that cannot serve the beat can still very often bring the machine
    down. It costs no HOLD, because the plan that ends the ring settles the cup
    on its own.

    **And then the session STOPS rather than degrading** (owner, 2026-09-07).
    Continuing is physically safe — the LANDING parks the cup at the floor and
    the next cycle would plan a genuine `MODE_NEW` LAUNCH from a stopped machine
    — but the beat would silently revert to the settle-plus-relaunch cadence for
    the rest of the sitting. These are MEASUREMENT sessions; a corpus with two
    cadences in it is a corpus with two machines in it, and the operator's
    correct response to a refused window is to raise `dwell_time_s`, which is
    only obvious if the session says so by name.

    One more cycle runs, deliberately: its ball is airborne when the refusal is
    known, and the fall-back window is what catches it.
    """
    clock = _Clock()
    # Refuse only the EXTENDED steady: the launch's own chained window must
    # install, or the ring never starts and there is nothing to fall back FROM.
    planner = _Planner(clock, refuse_kinds={
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_STEADY)})
    node = _ring_node(monkeypatch, clock, planner)
    holds = []
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (holds.append(1), 'held (stub)')[1])
    trace = _install_ring_replay(node, monkeypatch, clock,
                                 outcomes=[_caught()] * 3)
    result = node._execute_toss_continuous(_unified_goal(num_throws=3))
    node._toss_last_session_outcome = result.outcome
    kinds = [(r.mode, r.kind) for r in _cycle_requests(planner)]
    assert kinds[:3] == [
        (PlanCycle.Request.MODE_NEW, PlanCycle.Request.KIND_LAUNCH),
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_STEADY),
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_LANDING),
    ], kinds
    assert holds == [], 'a landing that installed does not need a hold'
    # The ring is TRUNCATED, and says so — carrying the planner's own refusal.
    outcome = node._toss_last_session_outcome
    assert outcome.startswith(tsess.OUTCOME_STOPPED_CHAIN_REFUSED), outcome
    assert 'LIMIT_JERK' in outcome, outcome
    # Two throws flew, not the three that were asked for, and the session did
    # NOT report COMPLETED for a ring it cut short.
    assert len(trace) == 2, [row['chained'] for row in trace]


def test_the_ring_flies_with_a_REALISTIC_solve_latency(monkeypatch):
    """The conversion's own claim: the FSM keeps ticking through the solve.

    The measured MODE_EXTEND is 354-387 ms — nine or ten coordinator ticks — and
    under the blocking wait every one of them was a tick the cycle did not take.
    Those are the ticks straddling the CATCH, i.e. exactly the ones a miss would
    be seen in, so the stall was not a latency cost: it was a window in which the
    coordinator could not have issued the hold that stops the chained stroke.

    Here the whole ring flies with a 0.40 s solve and the choreography is
    unchanged — same three requests, same exact beat.
    """
    clock = _Clock()
    planner = _Planner(clock)
    planner.extend_latency_s = 0.40
    node = _ring_node(monkeypatch, clock, planner)
    trace = _install_ring_replay(node, monkeypatch, clock,
                                 outcomes=[_caught()] * 3)
    assert node._execute_toss_continuous(
        _unified_goal(num_throws=3)).outcome == 'COMPLETED'
    assert [(r.mode, r.kind) for r in _cycle_requests(planner)] == [
        (PlanCycle.Request.MODE_NEW, PlanCycle.Request.KIND_LAUNCH),
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_STEADY),
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_LANDING),
    ]
    releases = [row['plan_release'] for row in trace]
    gaps = [b - a for a, b in zip(releases, releases[1:])]
    assert gaps == pytest.approx([BEAT] * len(gaps), abs=1e-9), gaps
    # The FSM ticked THROUGH the solve rather than around it: the extend was
    # dispatched at the catch and the cycle went on being stepped until its
    # verdict, ~10 ticks later.
    for row in trace[:-1]:
        assert row['ticks_after_extend'] >= 5, row


def test_a_MISS_during_a_PENDING_extend_holds_on_that_tick_and_discards_it(
        monkeypatch):
    """THE reason the extend is non-blocking, stated as a test.

    The extend is dispatched at the catch instant and takes ~0.4 s to answer. A
    miss is discovered inside that window. Blocking, the coordinator could not
    have seen it — it would have been sitting in a service wait, and at a dwell
    near the floor the hold would then arrive after the chained stroke had begun,
    over an empty cup. Polling, the miss is seen on its own tick and the hold
    goes out on that tick.

    And the extend's answer, when it finally lands, is DISCARDED. That is not
    "ignored because it failed": it may be an ACCEPT — the service installs on
    accept and replies afterwards — so the client both refuses to act on it (no
    ``_toss_unified_chain``, no live chain, no release handed to a cycle that
    will never run) AND holds a second time, because an install that landed after
    the first hold has superseded it. See ``_settle_abandoned_chain`` for why
    server-side arrival order is not relied on: ``trajectory/hold`` is registered
    BEFORE ``trajectory/plan_cycle``, so a same-wait-set pair is serviced in the
    adverse order.
    """
    clock = _Clock()
    planner = _Planner(clock)
    planner.extend_latency_s = 0.60          # still in flight at the verdict
    node = _ring_node(monkeypatch, clock, planner)
    holds = []
    monkeypatch.setattr(
        node, '_hold_after_unacked_plan',
        lambda: (holds.append(clock.t), 'held (stub)')[1])
    trace = _install_ring_replay(node, monkeypatch, clock,
                                 outcomes=[_missed(), _caught(), _caught()])
    result = node._execute_toss_continuous(_unified_goal(num_throws=3))
    assert result.outcome == 'STOPPED_ON_MISS', result.outcome
    row = trace[0]
    assert row['extend_at'] is not None, 'the extend never went out'
    assert row['pending_at_verdict'], (
        'the extend had already answered — this test is not exercising the '
        'window it was written for')
    # The hold went out while the request was still in flight…
    assert holds and holds[0] >= row['extend_at']
    # …and the answer was discarded: nothing re-armed the chain.
    assert node._toss_unified_chain is None
    assert node._toss_unified_release_ahead == 0.0
    assert node._toss_unified_extend is None
    # The abandoned ACCEPT was drained and answered with a SECOND hold, because
    # an install that landed after the first one superseded it.
    assert len(holds) == 2, holds


def test_a_lost_chain_keeps_the_cycles_own_CAUGHT_verdict(monkeypatch):
    """The ring failed; the throw did not.

    ``unified_reject`` still carries the detail for the record and the bench, but
    the per-cycle RELABEL is skipped once the session terminal is armed. Left in,
    it wrote ``REJECTED_CHAIN_LOST(...)`` over the outcome of a cycle that caught
    its ball — so the corpus recorded a rejection for a catch — and, with
    ``success`` still True, handed ``note_cycle_result`` a REJECTED_* string it
    reads as a successful throw. The arming alone was then the only thing
    stopping the session, which is a single point of failure for a hazard that
    has two.
    """
    clock = _Clock()
    planner = _Planner(clock, unacked_kinds={
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_STEADY)})
    node = _ring_node(monkeypatch, clock, planner)
    outcomes = []
    real_note = None

    def spy_result(result, *a, **k):
        outcomes.append((bool(result.success), str(result.outcome)))
        return real_note(result, *a, **k)

    def wire(session):
        nonlocal real_note
        real_note = session.note_cycle_result
        monkeypatch.setattr(session, 'note_cycle_result', spy_result)

    real_build = node._build_toss_cycle
    monkeypatch.setattr(node, '_build_toss_cycle',
                        lambda *a, **k: (wire(node._toss_session_ref)
                                         if real_note is None else None,
                                         real_build(*a, **k))[1])
    _install_ring_replay(node, monkeypatch, clock, outcomes=[_caught()] * 3)
    result = node._execute_toss_continuous(_unified_goal(num_throws=3))
    assert outcomes == [(True, 'CAUGHT')], outcomes
    assert result.outcome.startswith(tsess.OUTCOME_STOPPED_CHAIN_LOST)
    assert result.catches_confirmed == 1
    # …and the detail is not lost — it moved to the SESSION outcome, which is
    # where a stop belongs. (`unified_reject` still carries it on the cycle state
    # for the record and the bench, but the cycle teardown clears that before
    # `note_cycle_result` runs, so the session string is the durable copy.)
    assert 'NO_WINDOW' in result.outcome, result.outcome
    assert 'UNACKED' in result.outcome, result.outcome


def test_a_stale_toss_session_is_reported_LOUDLY_not_swallowed(monkeypatch):
    """The arming is what stops the session, so a missing hook is an ERROR.

    ``except (AttributeError, TypeError): pass`` would have swallowed a
    ``note_chain_refused`` that a stale ``colcon``-installed ``toss_session``
    does not have — reinstating, silently, the exact bug the arming replaced: a
    lost chain reporting COMPLETED. The only way to reach it is a build older
    than this node, and an operator who is TOLD that fixes it in one command.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    errors = []
    monkeypatch.setattr(node.get_logger(), 'error',
                        lambda msg, *a: errors.append(str(msg)))
    monkeypatch.setattr(node, '_hold_after_unacked_plan', lambda: 'held (stub)')
    state = node._toss_committed
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY, unified=True)
    seq.start(clock.t)
    # (a) no hook at all — the stale install.
    node._toss_session_ref = types.SimpleNamespace()
    node._unified_chain_failed(seq, state, True, '', ('refused', True), 0.0)
    assert any('STALE INSTALL' in e for e in errors), errors
    assert state.unified_chain_stop_armed is False
    # (b) the one-argument stale signature.
    errors.clear()
    state.unified_reject = ''
    node._toss_session_ref = types.SimpleNamespace(
        note_chain_refused=lambda detail: None)
    node._unified_chain_failed(seq, state, True, '', ('refused', True), 0.0)
    assert any('STALE INSTALL' in e for e in errors), errors
    assert state.unified_chain_stop_armed is False
    # (c) the real thing arms, and says so.
    errors.clear()
    state.unified_reject = ''
    node._toss_session_ref = TossSessionSequencer(
        num_throws=3, dwell_time_s=DWELL, throw_delay_s=DELAY,
        flight_time_s=FLIGHT, ilc_speed_trim_possible=False)
    node._unified_chain_failed(seq, state, True, '', ('refused', True), 0.0)
    assert state.unified_chain_stop_armed is True
    assert not any('STALE INSTALL' in e for e in errors), errors


@pytest.mark.parametrize('miss_on,label', [(0, 'cycle 1'), (1, 'cycle N-1')])
def test_a_MISS_HOLDS_on_EVERY_cycle_that_still_has_a_release_ahead(
        monkeypatch, miss_on, label):
    """H1 — the predicate is "is a release still ahead", not `release_terminal`.

    `release_terminal` means "the plan's LAST KNOT is a release", which is a
    different question and answers FALSE for the ring's last two cycles: the
    final LANDING window is installed a whole beat BEFORE the release it starts
    at, so from that install until R_N there is a full stroke ahead and the flag
    calls the machine safe. A MISS on cycle N-1 took the `if not live` early
    return, issued NO hold, and let the plan throw over an empty cup ~1.2 s after
    the session had already reported STOPPED_ON_MISS.

    The `cycle 1` row is the case that always worked; it is kept as the control,
    because a fix that only moved the hole would still pass it.
    """
    clock = _Clock()
    planner = _Planner(clock)
    node = _ring_node(monkeypatch, clock, planner)
    holds = []
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (holds.append(clock.t), 'held (stub)')[1])
    outcomes = [_caught()] * 3
    outcomes[miss_on] = _missed()
    _install_ring_replay(node, monkeypatch, clock, outcomes=outcomes)
    result = node._execute_toss_continuous(_unified_goal(num_throws=3))
    assert result.outcome == 'STOPPED_ON_MISS', result.outcome
    assert holds, (
        'a MISS on %s left the standing plan to throw over an empty cup' % label)
    assert node._toss_unified_release_ahead == 0.0


def test_no_hold_is_owed_once_the_last_release_has_fired(monkeypatch):
    """…and the predicate is not simply "always hold".

    After R_N the ring's last stroke has happened: the LANDING window is carrying
    the cup through its catch and on to the floor, and a hold there would replace
    a settle that was going to happen anyway. The hold is owed for a stroke that
    has NOT fired, and for nothing else — otherwise every clean ring would end by
    superseding its own landing.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    holds = []
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (holds.append(1), 'held (stub)')[1])
    node._toss_unified_release_ahead = clock.t - 0.001      # R_N just fired
    assert node._hold_live_unified_chain('teardown') is False
    assert holds == []
    # …and one tick the other way, it is owed.
    node._toss_unified_release_ahead = clock.t + 0.001
    assert node._hold_live_unified_chain('teardown') is True
    assert holds == [1]


def test_a_cancel_inside_the_last_cycle_before_its_release_still_HOLDS(
        monkeypatch):
    """The same hole, reached through the operator's own stop button (H1).

    `_safe_toss_on_early_exit` is the cancel / timeout / shutdown path, and the
    session `finally` is the belt behind it. Both read the same predicate, so
    both were blind for the whole of the ring's last cycle — the window in which
    the operator is most likely to press stop and least likely to expect one more
    throw.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    node._toss_unified_live = True
    holds = []
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (holds.append(1), 'held (stub)')[1])
    # The state the LANDING extend leaves behind: rest-terminal plan, release
    # R_N still a beat away.
    node._toss_unified_release_ahead = clock.t + BEAT
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY,
                        unified=True, chained=True,
                        release_at_perf=clock.t + BEAT)
    seq.start(clock.t)
    node._safe_toss_on_early_exit(seq, node._toss_committed)
    assert holds == [1], 'a cancel before the last release commanded no hold'


def test_the_reload_interlude_holds_the_chain_before_it_moves_anything(
        monkeypatch):
    """M3 — every SESSION-loop action that commands motion holds first.

    A chained cycle can mint REJECTED_NO_BALL in CHECKING and terminalise with
    ACTION_NONE — no SAFE_ABORT, so no hold — while the previous cycle's window
    is streaming with a release still ahead. With `on_empty_cup RELOAD` the
    session then drives the platform (go_home, recentre) under that plan: one
    interval with two owners, and the plan still throws at R_{k+1}.

    Pinned at the guard rather than inside the interlude, because the finding is
    a CLASS — "the loop is about to command motion while a stroke is pending" —
    and the interlude is only the instance that was found.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    order = []
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (order.append('hold'), 'held (stub)')[1])
    monkeypatch.setattr(node, '_go_home',
                        lambda *a, **k: (order.append('go_home'), True)[1])
    node._toss_unified_release_ahead = clock.t + BEAT
    node._hold_chain_before_session_motion('the reload interlude')
    assert order == ['hold'], order
    assert node._toss_unified_release_ahead == 0.0
    # Idempotent: a second action in the same teardown costs nothing.
    node._hold_chain_before_session_motion('go_home')
    assert order == ['hold'], order


def test_a_STEADY_that_never_ANSWERS_gives_up_BEFORE_the_cliff(monkeypatch):
    """H2 — the ladder is bounded by the DEADLINE, not by the client wait.

    A wedged ``plan_cycle`` used to be declared UNACKED only at
    ``sent_at + _SERVICE_WAIT_S`` (2.0 s), while the whole carry a ring has
    between its catch and its supersede deadline is ``dwell - dt`` — 1.175 s at
    the shipped beat 2.0. So the fall-back and the hold BOTH landed ~0.8 s past
    the cliff: the emitter had already read the release segment's endpoint from
    the plan's terminal hold and commanded a 93.011 rev/s stroke to 0.0, which is
    10.90 mm of slider error and invisible to every firmware guard.
    ``_SERVICE_WAIT_S`` is a client-patience number and knows nothing about this
    plan's cliff.

    Everything the ladder does now happens before that deadline. And with only
    0.16 s of margin left, the LANDING rung is deliberately SKIPPED rather than
    attempted: a window that cannot arrive in time is not a fall-back, it is the
    hold's budget spent on nothing.
    """
    clock = _Clock()
    planner = _Planner(clock, unacked_kinds={
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_STEADY)})
    node = _ring_node(monkeypatch, clock, planner)
    holds = []
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (holds.append(clock.t), 'held (stub)')[1])
    trace = _install_ring_replay(node, monkeypatch, clock,
                                 outcomes=[_caught()] * 3)
    result = node._execute_toss_continuous(_unified_goal(num_throws=3))
    deadline = trace[0]['extend_deadline']
    assert deadline > 0.0
    # Every rung was DISPATCHED before the deadline…
    for req_t in [row['extend_at'] for row in trace if row['extend_at']]:
        assert req_t < deadline, (req_t, deadline)
    # …and the hold — the rung that always works — landed in front of the cliff,
    # with the margin it was sized for.
    assert holds, 'the machine was never held'
    assert holds[0] <= deadline, (holds[0], deadline)
    assert deadline - holds[0] == pytest.approx(rcn._UNIFIED_HOLD_MARGIN_S,
                                                abs=rcn._PACE_PERIOD_S)
    # The LANDING was skipped: at the give-up instant there was not enough left
    # for a whole MODE_EXTEND plus the hold behind it.
    assert [(r.mode, r.kind) for r in _cycle_requests(planner)] == [
        (PlanCycle.Request.MODE_NEW, PlanCycle.Request.KIND_LAUNCH),
        (PlanCycle.Request.MODE_EXTEND, PlanCycle.Request.KIND_STEADY),
    ]
    # …and the session STOPS by name, without stealing cycle 1's catch.
    assert result.outcome.startswith(tsess.OUTCOME_STOPPED_CHAIN_LOST), (
        result.outcome)
    assert result.catches_confirmed == 1, result.catches_confirmed


def test_the_steady_to_landing_to_hold_ladder_runs_in_order(monkeypatch):
    """The whole ladder, at its own seam, with each rung's refusal in place.

    Rung 3 COMMANDS, and that is the point: on a rest-terminal plan "hold the
    last good plan" is true — the emitter parks the cup and nothing else happens.
    On a RELEASE-terminal one it is false in the way that matters, because the
    standing plan's last act is a throw stroke. Doing nothing there means
    throwing again, with no window planned to catch what leaves.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    node._toss_unified_live = True
    node._toss_session_ref = TossSessionSequencer(
        num_throws=3, dwell_time_s=DWELL, throw_delay_s=DELAY,
        flight_time_s=FLIGHT, ilc_speed_trim_possible=False)
    node._toss_session_ref.start(clock.t)
    node._toss_session_ref.step(clock.t)          # -> cycle_index 1
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY,
                        unified=True)
    seq.start(clock.t)
    state = node._toss_committed
    state.unified_plan = types.SimpleNamespace(
        t_release_mono=clock.t, release_terminal=True,
        supersede_deadline_mono=clock.t + BEAT)
    node._toss_unified_release_ahead = clock.t + BEAT
    holds = []
    refused = types.SimpleNamespace(
        accepted=False, code='INFEASIBLE',
        message='REJECTED_CYCLE_INFEASIBLE(LIMIT_JERK: no)')
    asked = _dispatched(node, monkeypatch, resolve=refused)
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (holds.append(clock.t), 'held (stub)')[1])
    # Tick 1 dispatches the STEADY; tick 2 reads its refusal and dispatches the
    # LANDING; tick 3 reads THAT refusal and runs the hold. Three ticks is the
    # ladder's real shape now that each rung's answer arrives on a later tick,
    # and driving it tick by tick is what shows the FSM was never blocked.
    for k in range(3):
        node._tick_unified_extend(seq, state, clock.t + FLIGHT
                                  + k * rcn._PACE_PERIOD_S)
    assert [r.kind for r in asked] == [PlanCycle.Request.KIND_STEADY,
                                       PlanCycle.Request.KIND_LANDING], asked
    assert len(holds) == 1, 'the machine was not held'
    assert holds[0] < clock.t + BEAT, 'the hold landed past the deadline'
    assert rcn._OUTCOME_CHAIN_LOST in state.unified_reject
    assert 'LIMIT_JERK' in state.unified_reject     # both refusals are carried
    assert 'held' in state.unified_reject           # …and the hold's verdict


def test_an_unavailable_service_is_not_asked_twice(monkeypatch):
    """A service that is not up means no request ever left this node.

    Two things are pinned. The readiness check is ``service_is_ready()``, not a
    2 s ``wait_for_service`` — a blocking readiness wait inside a live cycle is
    the same defect the non-blocking call fixed, one layer up. And the ladder
    then drops STRAIGHT to the hold: a second request to a service that is not
    there is not a fail-safe, it is a duplicate, and it would spend
    ``_SERVICE_WAIT_S`` of a budget the supersede deadline is already eating."""
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    node._toss_unified_live = True
    node._toss_session_ref = TossSessionSequencer(
        num_throws=3, dwell_time_s=DWELL, throw_delay_s=DELAY,
        flight_time_s=FLIGHT, ilc_speed_trim_possible=False)
    node._toss_session_ref.start(clock.t)
    node._toss_session_ref.step(clock.t)
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY, unified=True)
    seq.start(clock.t)
    state = node._toss_committed
    state.unified_plan = types.SimpleNamespace(
        t_release_mono=clock.t, release_terminal=True,
        supersede_deadline_mono=clock.t + BEAT)
    holds = []
    asked = _dispatched(node, monkeypatch)
    node._plan_cycle_cli.service_is_ready = lambda: False
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (holds.append(1), 'held (stub)')[1])
    node._tick_unified_extend(seq, state, clock.t + FLIGHT)
    assert asked == [], 'a request left the node with the service down'
    assert len(holds) == 1
    assert rcn._OUTCOME_CHAIN_LOST in state.unified_reject


def test_a_MISS_mid_chain_HOLDS_before_anything_else_and_chains_no_steady(
        monkeypatch):
    """The chained stroke must never fire over an empty cup.

    Every sentence in ``_unified_hold_after_abort`` about "not calling go_home is
    the whole mechanism" is true of a REST-terminal plan and false of a
    release-terminal one: the standing plan's last act is a throw stroke. A MISS
    is precisely the state in which the cup is empty, so the teardown owes a
    ``trajectory/hold`` — and it owes it BEFORE ``catch/armed`` False, which is
    the one place this ladder departs from ``_safe_abort``'s ordering rule. The
    stroke is the hazard with a clock on it; the retry tick is not.

    ⚠ **The extend fired during the miss cycle is not a bug and must not be
    "fixed" by making the extend evidence-triggered.** The chain is asked for at
    the plan's own catch INSTANT, which is strictly earlier than any verdict —
    the arrival band alone is 0.087 s wide on the early side, and a MISS produces
    no catch evidence at all, so a ring that waited for one would simply never
    chain. What makes the miss safe is not withholding the window; it is
    STOPPING the plan that carries it. So the assertion here is about what
    happens AFTER the verdict: nothing is asked for, and the hold has run.
    """
    clock = _Clock()
    planner = _Planner(clock)
    node = _ring_node(monkeypatch, clock, planner)
    order = []
    asked_at_hold = []
    monkeypatch.setattr(
        node, '_hold_after_unacked_plan',
        lambda: (order.append('hold'),
                 asked_at_hold.append(len(planner.requests)),
                 'held (stub)')[2])
    monkeypatch.setattr(node, '_publish_catch_armed',
                        lambda armed: order.append(('armed', bool(armed))))
    monkeypatch.setattr(node, '_arm_catch', lambda armed: (
        order.append(('arm_catch', bool(armed))), True)[1])
    _install_ring_replay(node, monkeypatch, clock,
                         outcomes=[_missed(), _caught(), _caught()])
    result = node._execute_toss_continuous(_unified_goal(num_throws=3))
    assert result.outcome == 'STOPPED_ON_MISS', result.outcome
    # The hold is the FIRST thing the teardown does — before catch/armed False.
    assert order[0] == 'hold', order
    assert ('armed', False) in order
    # …and nothing was asked for after it: the ring is dead, cycle 2 never ran.
    assert len(planner.requests) == asked_at_hold[0], (
        'a window was requested AFTER the miss was known')
    assert node._toss_unified_chain is None
    assert node._toss_unified_release_ahead == 0.0


@pytest.mark.parametrize('prepared', [True, False])
def test_an_early_exit_holds_a_live_chain_whether_or_not_the_cycle_ARMED(
        monkeypatch, prepared):
    """A cancel/timeout/shutdown INSIDE a chained cycle stops the plan.

    ``seq.prepared`` — the gate the rest of this ladder runs behind — asks "did
    THIS cycle arm anything". A chained cycle that has not reached PREPARE has
    armed nothing, and the un-gated reading of that is "nothing to safe". It is
    wrong here in the one case that matters most: the previous cycle's plan is
    still carrying a release toward this cycle, the operator has just asked the
    machine to STOP, and a stroke is scheduled on the emitter's own clock. So the
    chain hold sits outside the gate, and this pins both sides of it.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    node._toss_unified_live = True
    node._toss_unified_release_ahead = clock.t + BEAT
    holds = []
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (holds.append(1), 'held (stub)')[1])
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY,
                        unified=True, chained=True, release_at_perf=clock.t + 2)
    seq.start(clock.t)
    seq._prepare_dispatched = prepared
    node._safe_toss_on_early_exit(seq, node._toss_committed)
    assert holds == [1], 'the streaming chain was left to throw again'
    assert node._toss_unified_release_ahead == 0.0


def test_the_session_terminal_is_the_belt_for_a_chain_no_cycle_tore_down(
        monkeypatch):
    """The between-cycles exits reach no cycle terminal at all.

    The session ceiling, the F3 stall watchdog and the between-cycles cancel are
    each commented "nothing is armed and nothing is airborne" — true of the
    shipped rest-terminal shape, and false of a ring, where the standing plan is
    one window from a throw. The ``finally`` is the belt, and it is idempotent:
    on the ordinary path a cycle ladder has already held and cleared the flag, so
    it costs one lock acquisition.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    holds = []
    monkeypatch.setattr(node, '_hold_after_unacked_plan',
                        lambda: (holds.append(1), 'held (stub)')[1])
    # A session that cannot even build a cycle: the goal is cancelled between
    # cycles, so no `_toss_safe_abort` runs — but a chain is standing.
    def raise_the_chain(seq, **kw):
        node._toss_unified_release_ahead = clock.t + BEAT
        raise RuntimeError('boom')

    monkeypatch.setattr(node, '_run_toss_cycle', raise_the_chain)
    with pytest.raises(RuntimeError):
        node._execute_toss_continuous(_unified_goal(num_throws=3))
    assert holds == [1]
    assert node._toss_unified_release_ahead == 0.0
    # …and a session that never raised one pays nothing.
    holds.clear()
    node2 = _ring_node(monkeypatch, clock, _Planner(clock))
    monkeypatch.setattr(node2, '_hold_after_unacked_plan',
                        lambda: (holds.append(1), 'held (stub)')[1])
    _install_ring_replay(node2, monkeypatch, clock, outcomes=[_caught()])
    node2._execute_toss_continuous(_unified_goal(num_throws=1))
    assert holds == []


def test_a_REAL_chained_sequencer_announces_and_releases_at_the_floor_dwell():
    """B1(d) — the accept floor is one the machine can actually fly.

    Driven through the REAL ``TossSequencer``, not the ring replay: the whole
    finding was that the FSM's own runtime guard refused a cadence the accept
    gate had admitted, so a test that stubs the FSM cannot see it.

    The cycle is started where a chained cycle really starts — the catch, plus
    the worse of the verdict band and a settled extend — at exactly the accept
    floor's dwell, and it must reach ANNOUNCE and then DISPATCH_THROW without
    ``ABORTED_CANT_MAKE_RELEASE``. Before the fix the release-window guard
    charged the kind-0 dispatch budget (0.281 s at this flight) that a chained
    cycle never spends, so EVERY chained cycle aborted here, mid-ring, on a beat
    the machine had promised to fly.
    """
    floor, _binding = rcn._unified_chain_dwell_floor_s()
    catch = 1000.0
    # The instant `note_cycle_result` can be reached: the catch, then the LATER
    # of the verdict band and the extend the settle waits out — the same `max`
    # the floor is derived from.
    start = catch + max(rcn.TOSS_CATCH_CONFIRM_WINDOW_S,
                        rcn._UNIFIED_EXTEND_LEAD_S)
    release = catch + floor              # R_{k+1} = C_k + dwell
    seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                        flight_time_s=FLIGHT, throw_delay_s=DELAY,
                        unified=True, chained=True, release_at_perf=release)
    seq.start(start)
    assert seq.t_release == pytest.approx(release)
    now, actions = start, []
    for _ in range(40):                          # bounded: no real clock here
        obs = _obs(now, hand_parked=False)       # mid-carry, as a chain is
        decision = seq.step(now, obs)
        actions.append(decision.action)
        assert not decision.done, (
            'the chained cycle terminalised %s at t-%.3f s from its release'
            % (decision.result.outcome, release - now))
        if decision.action == rcn.TOSS_ACTION_POSITION_PLATFORM:
            seq.note_position_noop(now)          # the chained no-op seam
        elif decision.action == rcn.TOSS_ACTION_PREPARE_CATCH:
            seq.note_prepare_result(True)
        elif decision.action == rcn.TOSS_ACTION_ANNOUNCE:
            seq.note_announcement()
        elif decision.action == rcn.TOSS_ACTION_DISPATCH_THROW:
            break
        now += rcn._PACE_PERIOD_S
    assert rcn.TOSS_ACTION_ANNOUNCE in actions, actions
    assert actions[-1] == rcn.TOSS_ACTION_DISPATCH_THROW, actions
    assert now < release, (
        'the dispatch landed AFTER the release the plan had committed to')


def test_the_chained_release_guard_is_a_tick_not_a_kind0_windup():
    """R1 (owner decision 4): NEITHER path fronts a dispatch-side windup any
    more — ``set_hand_traj_cmd`` and the stroke it needed a prelude/gap/windup
    for are retired, so ``_dispatch_toss`` issues no RPC on either path and
    ``min_event_delay_for_throw_s`` is a flat zero. The PLAIN floor is that
    zero directly; the CHAINED floor is one tick (``NODE_LOOP_PERIOD_S`), for
    the announcement it still has to get out before the ball leaves — so the
    two floors are no longer "windup vs tick", they are "zero vs one tick",
    with chained now the (slightly) LARGER of the two — the inverse of the
    pre-R1 relationship, where the reactive windup made plain the larger one.
    """
    plain = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                          flight_time_s=FLIGHT, throw_delay_s=DELAY)
    chained = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                            flight_time_s=FLIGHT, throw_delay_s=DELAY,
                            chained=True)
    assert plain.release_window_floor_s == plain.min_event_delay_for_throw_s
    assert plain.release_window_floor_s == 0.0          # R1: no more windup
    assert chained.release_window_floor_s == rcn._PACE_PERIOD_S
    assert chained.release_window_floor_s > plain.release_window_floor_s


# ═════════════════════════════════════════════════════════════════════════════
# 6. THE ACCEPT-TIME BEAT FLOOR
# ═════════════════════════════════════════════════════════════════════════════

def test_cycle_1_adopts_the_plans_release_and_waits_for_the_plan_it_asked_for(
        monkeypatch):
    """The 2026-09-09 hand-off, and the two cliffs it closes.

    Cycle 1 is started on a DERIVED release (``now + throw_delay_s``) because
    its LAUNCH does not exist yet, while the plan's release lands at
    ``install + 0.6`` whatever the solve cost. On hardware a 171-knot
    LAUNCH+STEADY at a 3.6 s beat solved in 1.15 s: the FSM minted
    ``ABORTED_NO_RELEASE`` 0.4 s before the plan's release with the plan
    streaming, and three of six launches crossed the flat 2.0 s client wait
    and were answered by a hold that cancelled the solve. So (1) once the
    LAUNCH answers, the sequencer ADOPTS the plan's own instant —
    ``t_release``, ``release_at_perf`` and the landing schedule move together —
    and (2) the blocking wait is sized by the plan the request produces, knot
    for knot, never below the flat wait.

    The old accept-time "cycle-1 floor" (0.866 s, a lateness bound built from a
    75-knot solve measurement) is gone with the lateness; the session's own
    ``REJECTED_THROW_DELAY`` floor is the one delay floor left.
    """
    clock = _Clock()
    planner = _Planner(clock)
    # A solve that outlives the derived schedule AND its 0.5 s grace — the
    # pre-fix FSM minted ABORTED_NO_RELEASE before the plan's release.
    planner.launch_latency_s = DELAY + 0.9
    node = _ring_node(monkeypatch, clock, planner)
    dwell = 3.0                                  # beat 3.8 s: a ~177-knot install
    trace = _install_ring_replay(node, monkeypatch, clock,
                                 outcomes=[_caught()] * 3)
    result = node._execute_toss_continuous(
        _unified_goal(num_throws=3, dwell=dwell, delay=DELAY))
    assert result.outcome == 'COMPLETED', result.outcome
    first = trace[0]
    seq = first['seq']
    # (1) the hand-off: the FSM runs on the PLAN's release, not the placeholder
    # it was started on — and the placeholder really was left behind.
    assert first['plan_release'] > first['t_release'] + 0.5, first
    assert seq.t_release == pytest.approx(first['plan_release'])
    assert seq.release_at_perf == pytest.approx(first['plan_release'])
    assert seq.landing_perf == pytest.approx(first['plan_release'] + FLIGHT)
    # (2) the wait is the plan's: launch window + the chained STEADY (the
    # beat) at the budget's per-knot rate, above the flat wait at this beat…
    expected = rcn._unified_first_install_wait_s(FLIGHT + dwell)
    assert planner.timeouts[0] == pytest.approx(expected)
    assert expected > rcn._SERVICE_WAIT_S
    assert expected == pytest.approx(
        ((rcn._UNIFIED_LAUNCH_WINDOW_S + FLIGHT + dwell) / KNOT_DT_S + 1.0)
        * rcn._UNIFIED_PLAN_BUDGET_S / rcn._UNIFIED_PLAN_BUDGET_KNOTS)
    # …and a short beat keeps the flat wait: nothing that fit before waits less.
    assert rcn._unified_first_install_wait_s(FLIGHT + 0.8) == pytest.approx(
        rcn._SERVICE_WAIT_S)
    assert not hasattr(rcn, '_unified_cycle1_delay_floor_s')


def test_a_dwell_under_the_extend_lead_is_REJECTED_BEAT_TOO_SHORT(monkeypatch):
    """A3 — refused before anything is armed, lifted or commanded.

    A chained window is planned INSIDE the previous one: requested once the live
    catch has passed, installed before the standing plan's supersede deadline.
    The carry between those two instants IS the dwell (less one knot), so a dwell
    under one extend lead asks the chain to receive a window it has no time to
    receive — and the failure is not a slow beat, it is the LANDING fail-safe
    firing on every cycle and the session ending after one throw with the
    operator reading a planner refusal instead of a cadence one.

    The message names all three things the house style requires: the requested
    value, the limit it broke, and the knobs that move it.
    """
    clock = _Clock()
    planner = _Planner(clock)
    node = _ring_node(monkeypatch, clock, planner)
    monkeypatch.setattr(
        node, '_build_toss_cycle',
        lambda *a, **k: pytest.fail('a cycle was built under the beat floor'))
    floor, binding = rcn._unified_chain_dwell_floor_s()
    # The floor is the LARGER of the two requirements, and at the shipped
    # constants it is the NEXT CYCLE'S LEAD, not the extend carry — which is the
    # whole B1 finding: gating on the carry alone admitted beats on which every
    # chained cycle then aborted CANT_MAKE_RELEASE mid-ring.
    assert floor == pytest.approx(0.800, abs=5e-4), floor
    assert binding == "the next cycle's lead"
    short = floor - 0.05
    result = node._execute_toss_continuous(
        _unified_goal(num_throws=3, dwell=short))
    assert result.outcome.startswith(rcn._OUTCOME_BEAT_TOO_SHORT), result.outcome
    assert 'dwell' in result.outcome
    assert '{:.3f}'.format(short) in result.outcome
    assert '{:.3f}'.format(floor) in result.outcome
    assert binding in result.outcome            # WHICH term bound
    assert 'dwell_time_s' in result.outcome
    assert 'throw_height_m' in result.outcome
    # …and both terms are shown, so the operator can see the other one coming.
    assert '{:.3f}'.format(rcn._UNIFIED_EXTEND_LEAD_S) in result.outcome
    assert '{:.3f}'.format(rcn.TOSS_CATCH_CONFIRM_WINDOW_S) in result.outcome
    # Nothing was planned, and nothing was latched: the refusal is above the
    # hand-source assertion and above the floor lift.
    assert planner.requests == []


def test_the_beat_floor_is_LAYER_B_and_fires_before_the_sessions_dwell_gate(
        monkeypatch):
    """Ordering, and it is the reason the gate is at Layer B at all.

    A 0.5 s dwell breaks the session FSM's own ``REJECTED_DWELL`` floor too. The
    operator must read the UNIFIED reason, because that is the one that names the
    mechanism they have to act on — the chain has no carry to plan in — rather
    than a legacy plumbing floor derived from a kind-0 dispatch budget this path
    never spends.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    session = TossSessionSequencer(
        num_throws=3, dwell_time_s=0.5, throw_delay_s=DELAY,
        flight_time_s=FLIGHT, ilc_speed_trim_possible=False)
    assert 'REJECTED_DWELL' in str(session._checking_reject())   # non-vacuous
    result = node._execute_toss_continuous(_unified_goal(num_throws=3,
                                                         dwell=0.5))
    assert result.outcome.startswith(rcn._OUTCOME_BEAT_TOO_SHORT)


def test_the_beat_floor_leaves_a_LEGACY_goal_alone(monkeypatch):
    """The floor is a property of the CHAIN, so a goal with no chain never meets
    it. A legacy session with the same dwell is refused by its own name — since
    R1 (2026-09-11) that name is REJECTED_STROKE_ENGINE_RETIRED at accept (the
    device the legacy branch dispatched to is deleted), which fires before the
    session FSM's REJECTED_DWELL ever could; the beat floor is never consulted."""
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    goal = _ContGoalHandle(num_throws=3, dwell=0.5, delay=DELAY)
    goal.request.unified_cycle = False
    result = node._execute_toss_continuous(goal)
    assert result.outcome.startswith('REJECTED_STROKE_ENGINE_RETIRED'), result.outcome
    assert rcn._OUTCOME_BEAT_TOO_SHORT not in result.outcome


@pytest.mark.parametrize('dwell', [float('nan'), -1.0])
def test_a_malformed_dwell_still_belongs_to_the_sessions_own_gate(monkeypatch,
                                                                  dwell):
    """A NaN or a sign typo is ``REJECTED_DWELL``'s, at the session FSM.

    Stealing them would name the wrong knob for the wrong reason: the beat floor
    is a statement about how much carry a chain has, and a negative dwell is not
    a short carry — it is a goal that was typed wrong.
    """
    clock = _Clock()
    node = _ring_node(monkeypatch, clock, _Planner(clock))
    result = node._execute_toss_continuous(
        _unified_goal(num_throws=3, dwell=dwell))
    assert rcn._OUTCOME_BEAT_TOO_SHORT not in result.outcome
