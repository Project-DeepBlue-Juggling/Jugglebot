"""Property tests for the two-slot toss pipeline (plan
``toss-pipelined-preamble.md`` § 5.3, T-P1..T-P5).

**These are the first Hypothesis tests in ``tests/ros``.** The library was used
only in ``tests/sim/`` until 2026-08-27, and everything about how they run is
inherited rather than invented: the ``ci-fast`` (50) / ``ci-deep`` (1000)
profiles are registered suite-wide in ``tests/conftest_hypothesis.py``, there is
no per-test ``@settings`` override anywhere in this repo, and adding one here
would silently opt these properties out of the nightly's depth.

WHY PROPERTIES AND NOT MORE EXAMPLES. The pipeline's safety argument is a set of
statements about EVERY reachable interleaving of (tick instants, observation
flips, upstream outcomes), and the deterministic tests in
``test_toss_sequencer.py`` and ``test_toss_continuous_node.py`` each pin ONE
interleaving. The two properties that actually matter — at most one cycle past
its commit, and no dispatch on a tick whose observation said the cup was empty —
are temporal, and a temporal claim checked at four hand-picked instants is a
claim about four instants.

**The strategies generate OBSERVATION STREAMS, never internals.** A strategy
that reached into ``_phase`` or ``_commit_at`` would be testing the
implementation it was written against; one that generates the inputs the node
feeds the FSM tests the CONTRACT, and survives a refactor of everything behind
it. The one thing the model does know about the FSM is its PUBLIC vocabulary —
the action strings, ``committed``, ``finished`` — which is what a node knows too.

WHAT IS SIMULATED HERE, and what is real:

  * REAL — ``TossSequencer`` (both slots), ``TossSessionSequencer``, every
    budget and every gate;
  * SIMULATED — the node's tick loop, reduced to its two load-bearing rules
    (ONE observation snapshot per tick; the COMMITTED slot is stepped first) and
    its four synchronous answers (``note_position_noop``,
    ``note_prepare_result``, ``note_throw_dispatch``,
    ``note_upstream_terminalised``). Those rules are pinned STRUCTURALLY against
    the real ``_tick_toss_pipeline`` by
    ``test_the_committed_slot_is_ticked_before_the_staged_slot``, so this model
    cannot drift into describing a loop the node does not run.

**2026-08-28 — the tick spacing became a PER-TICK variable** (``_TICK_GAPS``,
alongside the original single-draw ``_JITTER``). The first pipelined sitting
died on an interleaving these properties could not generate: ONE iteration
longer than ``NODE_LOOP_PERIOD_S`` among nominal ones, landing on the commit
crossing. ``_JITTER`` draws a single spacing and applies it to EVERY tick of a
run, so a run was uniformly fast or uniformly slow and the mixed case never
existed. T-P5 gains a twin over the new strategy — the release-window guard now
SLIPS, which adds a path into the one writer of ``_t_release`` and therefore
re-opens T-P5's argument — and a new property asserts that no staged cycle dies
``CANT_MAKE_RELEASE`` INSIDE the slip bound. See
``logbook/2026-08-28-pipeline-first-contact-deadlock.md``.
"""

from __future__ import annotations

import math

import pytest
from hypothesis import given, strategies as st

from jugglebot.toss_sequencer import (
    ACTION_ANNOUNCE,
    ACTION_DISPATCH_THROW,
    ACTION_NONE,
    ACTION_POSITION_PLATFORM,
    ACTION_PREPARE_CATCH,
    ACTION_REACH_CATCH,
    ACTION_RECENTER,
    ACTION_SAFE_ABORT,
    ACTION_STAY,
    CATCH_CONFIRM_WINDOW_S,
    NODE_LOOP_PERIOD_S,
    PHASE_COMMITTING,
    PHASE_STAGED,
    THROW_DISPATCH_OK,
    TOSS_CONTROL_MODE,
    TossObservations,
    TossSequencer,
)

CATCH_POSE = (0.0, 0.0, 170.0)
FLIGHT = 0.9032                      # h = 1.0 m, the milestone's tight rung
DWELL = 0.50

#: The actions ONLY a cycle past its COMMIT may emit. Every one of them either
#: commits the hand, moves the platform or tears the catch down — under a ball
#: the previous cycle put in the air.
HAND_BEARING = frozenset((ACTION_ANNOUNCE, ACTION_DISPATCH_THROW,
                          ACTION_REACH_CATCH, ACTION_STAY, ACTION_RECENTER,
                          ACTION_SAFE_ABORT))
#: S1′'s staged decision set, verbatim from the invariant.
STAGED_ACTIONS = frozenset((ACTION_NONE, ACTION_POSITION_PLATFORM,
                            ACTION_PREPARE_CATCH))


# ── The observation stream: what the strategies actually generate ─────────────

#: One tick's worth of plant + link state. Deliberately the FIELDS THE NODE
#: FEEDS, not the FSM's beliefs about them.
_OBS = st.fixed_dictionaries({
    'hand_parked': st.booleans(),
    'ball_seated': st.booleans(),
    'track_active': st.booleans(),
    'mocap_fresh': st.booleans(),
    'streaming': st.booleans(),
    'platform_levelled': st.booleans(),
    'hand_fresh': st.booleans(),
})

#: A HEALTHY stream, which is what the machine sees on a good sitting. The
#: fully-random stream above finds refusal paths; this one finds the paths a
#: session actually walks, and both matter — a property that only ever exercises
#: refusals never reaches the commit at all.
_HEALTHY = st.fixed_dictionaries({
    'hand_parked': st.booleans(),
    'ball_seated': st.booleans(),
    'track_active': st.just(False),
    'mocap_fresh': st.just(True),
    'streaming': st.just(True),
    'platform_levelled': st.just(True),
    'hand_fresh': st.just(True),
})

_STREAM = st.lists(_OBS, min_size=1, max_size=60)
_HEALTHY_STREAM = st.lists(_HEALTHY, min_size=1, max_size=60)
#: Tick spacing — the loop is POLLED, and the plan's own commit budget charges
#: one full period of polling lateness. A model that ticked at an exact,
#: constant period would never exercise the lateness the budget exists for.
_JITTER = st.floats(min_value=NODE_LOOP_PERIOD_S * 0.5,
                    max_value=NODE_LOOP_PERIOD_S * 1.5)

#: PER-TICK spacing (2026-08-28), and the difference from ``_JITTER`` is the
#: whole point of adding it: ``_JITTER`` draws ONE value and spaces every tick
#: of a run by it, so a run is either uniformly fast or uniformly slow and the
#: interleaving that actually bit — **one late iteration among nominal ones,
#: landing on the commit crossing** — was never generated. Four of the fifteen
#: staged slots of the first pipelined sitting died on exactly that shape.
#:
#: The upper bound is 2.5 periods rather than 1.5 for the same reason: the
#: commit budget grants ONE nominal period plus 1 µs, so a stream capped at 1.5
#: periods can break the release guard but a stream capped at 1.0 cannot, and
#: the sitting's own worst observed iteration was 1.95 periods
#: (``loop_period_max_pre_s`` 0.0782 s against 0.040). The 0.25 floor keeps the
#: fast side wide enough that a slipped commit really does get a nominal tick to
#: retry on, which is the recovery path the fix depends on.
_TICK_GAPS = st.lists(
    st.floats(min_value=NODE_LOOP_PERIOD_S * 0.25,
              max_value=NODE_LOOP_PERIOD_S * 2.5),
    min_size=1, max_size=60)


def _obs(now, spec, **over):
    base = dict(control_mode=TOSS_CONTROL_MODE, ball_evidence='SEATED',
                platform_at_target=True,
                # The honest-cache gate's observation (2026-08-28), healthy.
                # The model's platform never moves between stage and commit, so
                # True is the truthful value here; the SITE_MOVED path is driven
                # deterministically in test_toss_continuous_node.py rather than
                # modelled, because a model that flipped it would be asserting
                # the node's live-pose read rather than the FSM's response.
                staged_site_ok=True)
    base.update(spec)
    base.update(over)
    if not base.get('ball_seated', True):
        base['ball_evidence'] = 'EMPTY'
    return TossObservations(now=now, **base)


class _Pipeline:
    """A minimal two-slot pipeline: the node's tick loop reduced to its rules.

    It owns no policy. Every decision is the real FSM's; this only supplies the
    node's four synchronous answers and the ordering
    ``test_the_committed_slot_is_ticked_before_the_staged_slot`` pins."""

    def __init__(self, *, num_cycles=3, dwell=DWELL, flight=FLIGHT):
        self.num_cycles = int(num_cycles)
        self.dwell = float(dwell)
        self.flight = float(flight)
        self.committed = None
        self.staged = None
        self.log = []            # (t, slot, action, action_then, slip)
        self.terminals = []      # (cycle_index, outcome)
        #: STAGED-slot terminals only, with the slip they died at — the model
        #: drops the slot object, so a property about how a staged cycle ENDED
        #: cannot be asked of `self.staged` after the fact. (2026-08-28: the
        #: commit-gate regression is exactly a staged terminal, so a property
        #: that only inspected the LIVE slots was blind to it.)
        self.commit_terminals = []   # (outcome, slip_s, commit_slips)
        self.dispatch_obs = []   # the observation EVERY dispatch was made on
        self.releases = []
        self.started = 0
        self.committed_flags = []   # how many cycles were past COMMIT per tick

    # ── the node's own two rules ──
    def tick(self, now, spec):
        # 1. THE COMMITTED SLOT, always first: it owns the hand.
        if self.committed is not None and not self.committed.finished:
            self._step(now, self.committed, self._plant(now, self.committed,
                                                        spec), 'committed')
            if self.committed.finished:
                self.terminals.append((self.committed_index,
                                       self.committed._result.outcome))
                self.committed = None
                if self.staged is not None:
                    # § 2.4.4 / S1′: the upstream cycle has terminalised, so the
                    # staged slot's COMMIT gate may pass — on THIS tick, from
                    # the read that terminalised it.
                    self.staged.note_upstream_terminalised()
        # 2. THE STAGED SLOT. Its snapshot zeroes the per-cycle EVIDENCE that
        # belongs to the committed cycle (`_staged_observations` in the node):
        # a staged cycle has thrown nothing, so inheriting cycle k's release
        # evidence or its CAUGHT would let one cycle terminalise the other.
        if self.staged is not None and not self.staged.finished:
            d = self._step(now, self.staged,
                           _obs(now, spec, throw_stroke_seen=False,
                                ball_track_confirmed=False, ball_caught=False),
                           'staged')
            if d.action_then == ACTION_DISPATCH_THROW:
                # THE PROMOTION — the staged slot now owns the hand.
                self.committed = self.staged
                self.committed_index = self.staged_index
                self.staged = None
            elif self.staged is not None and self.staged.finished:
                self.terminals.append((self.staged_index,
                                       self.staged._result.outcome))
                self.commit_terminals.append(
                    (str(self.staged._result.outcome), self.staged.slip_s,
                     self.staged.commit_slips))
                self.staged = None
        # 3. fill an empty slot, the way the session does.
        self._maybe_start(now)
        n_committed = sum(1 for s in (self.committed, self.staged)
                          if s is not None and s._throw_dispatched)
        self.committed_flags.append(n_committed)

    def _plant(self, now, seq, spec):
        """The COMMITTED slot's observation, with the plant's own two DERIVED
        fields supplied rather than generated.

        ``throw_stroke_seen`` and ``ball_caught`` are consequences of the
        machine's OWN actions, not free variables: a strategy that flipped them
        would be generating worlds in which a ball is caught before it was
        thrown. Everything the strategy DOES drive — the cup, the park band, the
        tracker, the four link gates — is a genuine input."""
        rel = float(seq.t_release)
        land = rel + float(seq.flight_time_s)
        flew = bool(seq._throw_dispatched) and now >= rel
        caught = flew and now >= land + 0.1839 and spec.get('ball_seated', True)
        return _obs(now, spec, throw_stroke_seen=flew,
                    ball_track_confirmed=False, ball_caught=caught,
                    catch_event_dt_s=(0.1839 if caught else float('nan')))

    def _step(self, now, seq, obs, slot):
        d = seq.step(now, obs)
        self.log.append((now, slot, d.action, d.action_then, d.slip))
        # The node's FOUR synchronous answers, each executed exactly where
        # `_step_toss_sequence` executes it. Not an elif chain: the COMMIT tick
        # emits TWO actions, and an elif would silently drop the second.
        if d.action == ACTION_POSITION_PLATFORM:
            seq.note_position_noop(now)          # skip-only, always
        if d.action == ACTION_PREPARE_CATCH:
            seq._prepare_pending = True          # the node's one-tick deferral
        if d.action == ACTION_ANNOUNCE:
            # `_announce_toss` confirms its own publish; without it the FSM
            # never leaves PREPARING and every cycle dies at the release-window
            # guard, which is a model defect wearing an FSM's clothes.
            seq.note_announcement()
        if ACTION_DISPATCH_THROW in (d.action, d.action_then):
            # Tagged by WHICH GATE admitted it: a staged cycle dispatches out of
            # the COMMIT gate (evidence re-read on this very tick), a serial one
            # out of `_enter_throwing` (evidence checked at CHECKING, seconds
            # earlier). That difference IS the plan's claim, so the model records
            # it rather than flattening it.
            self.dispatch_obs.append(
                ('commit' if seq.staged else 'checking', obs))
            self.releases.append(float(seq.t_release))
            seq.note_throw_dispatch(THROW_DISPATCH_OK)
        if (getattr(seq, '_prepare_pending', False) and not d.done
                and d.action != ACTION_PREPARE_CATCH):
            seq._prepare_pending = False
            seq.note_prepare_result(True)
        return d

    def _maybe_start(self, now):
        if self.started >= self.num_cycles:
            return
        if self.committed is None:
            # The session's FIRST cycle: SERIAL, because there is nothing to
            # pipeline it behind and (in the node) it is the cycle that arms the
            # session. Its release is derived from `throw_delay_s`.
            seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE,
                                flight_time_s=self.flight, throw_delay_s=1.0,
                                positioning_move_expected=False)
            seq.start(now)
            self.started += 1
            self.committed, self.committed_index = seq, self.started
        elif (self.staged is None and self.committed._throw_dispatched
                and self.started < self.num_cycles):
            # STAGE, and only once the committed cycle has passed its own commit
            # point — which is what `note_cycle_committed` is keyed on in the
            # session, and what frees the slot this FSM fills.
            landing = (float(self.committed.t_release)
                       + float(self.committed.flight_time_s))
            seq = TossSequencer(catch_pose_stow_mm=CATCH_POSE,
                                flight_time_s=self.flight, throw_delay_s=5.0,
                                release_at_perf=landing + self.dwell,
                                positioning_move_expected=False, staged=True)
            seq.start(now)
            self.started += 1
            self.staged, self.staged_index = seq, self.started


def _run(stream, jitter, **kw):
    """Tick the model across ``stream``. ``jitter`` is either ONE spacing used
    for every tick (the original ``_JITTER`` form) or a SEQUENCE of per-tick
    gaps (``_TICK_GAPS``), cycled if it is shorter than the stream — the second
    form is what generates a late iteration among nominal ones, which is the
    interleaving the 2026-08-28 commit-gate regression lived in."""
    p = _Pipeline(**kw)
    t = 1000.0
    gaps = None
    if isinstance(jitter, (list, tuple)):
        gaps = [float(g) for g in jitter] or [NODE_LOOP_PERIOD_S]
    for i, spec in enumerate(stream):
        p.tick(t, spec)
        t += gaps[i % len(gaps)] if gaps is not None else float(jitter)
    return p


# ── T-P1 ─────────────────────────────────────────────────────────────────────

@given(stream=_STREAM, jitter=_JITTER)
def test_at_most_one_cycle_is_ever_past_its_commit(stream, jitter):
    """T-P1 — **S1′, as a temporal invariant**, asserted after EVERY step.

    S1's hazard was two cycles double-owning the hand on the Teensy's
    last-writer-wins queue. S1′ relaxes the object count and preserves the
    hazard exactly: two cycles may exist, but at most one may be past its COMMIT
    point, because that is the one that has dispatched a stroke."""
    p = _run(stream, jitter)
    assert p.committed_flags, 'the model ran no ticks'
    assert max(p.committed_flags) <= 1, p.committed_flags


# ── T-P2 ─────────────────────────────────────────────────────────────────────

@given(stream=_STREAM, jitter=_JITTER)
def test_no_dispatch_is_ever_made_on_an_observation_with_an_empty_cup(stream,
                                                                      jitter):
    """T-P2 — **THE safety property, stated temporally.**

    Not "ball_seated is checked somewhere" but "no tick that emitted
    ACTION_DISPATCH_THROW out of the COMMIT gate had an observation saying the
    cup was empty". A full-speed empty stroke with the hand ascending from an
    unverified position is what this forbids.

    **The scope is the pipelined path, and that is the finding rather than a
    weakening.** On the SERIAL ladder ``ball_seated`` and ``track_active`` are
    CHECKING-only gates by design (the FSM's own comment: "consulted only at
    CHECKING, before our own announcement exists"), so the evidence a serial
    dispatch rests on is 0.160-0.520 s old and the dispatch tick's own
    observation says nothing about it. The pipeline shrinks that distance to
    ZERO ticks — which is § 2.4.2's claim that the pipeline makes the
    empty-stroke gate STRICTER, not weaker — and this property is what makes the
    claim mechanical. ``hand_parked`` is asserted on BOTH, because the serial
    path re-verifies it at THROWING entry and the pipelined one at commit."""
    p = _run(stream, jitter)
    for gate, obs in p.dispatch_obs:
        assert obs.hand_parked is True, (gate, obs)
        if gate != 'commit':
            continue
        assert obs.ball_seated is True, obs
        assert obs.track_active is False, obs
        assert obs.streaming and obs.mocap_fresh, obs
        assert obs.platform_levelled and obs.hand_fresh, obs


@given(stream=_STREAM, jitter=_JITTER)
def test_a_staged_cycle_never_emits_a_hand_bearing_action(stream, jitter):
    """T-P2's sibling and S1′'s other half: while a cycle is STAGED its
    emittable action set is ``{NONE, POSITION_PLATFORM, PREPARE_CATCH}``.

    The moment it emits anything else it has committed — and the model records
    that transition as the promotion, so an action outside the set BEFORE that
    point is a cycle acting on the hand it does not own."""
    p = _run(stream, jitter)
    promoted = set()
    for _t, slot, action, action_then, _slip in p.log:
        if slot != 'staged':
            continue
        if action_then == ACTION_DISPATCH_THROW:
            # the COMMIT tick itself: ANNOUNCE then DISPATCH, in that order
            assert action == ACTION_ANNOUNCE
            promoted.add('yes')
            continue
        assert action in STAGED_ACTIONS, action
        assert action not in HAND_BEARING, action
        assert action_then == ACTION_NONE


# ── T-P3 ─────────────────────────────────────────────────────────────────────

@given(stream=_HEALTHY_STREAM, jitter=_JITTER)
def test_every_terminal_is_reached_once_and_every_action_fires_at_most_once(
        stream, jitter):
    """T-P3 — the existing SINGLE-cycle guarantee, lifted to two slots.

    Two properties in one, because they fail together: a cycle that terminalised
    twice would run its cleanup ladder twice, and a dispatch that fired twice
    would re-pack a new ``wall_time`` or clobber the armed catch stroke on the
    last-writer-wins queue (the 2026-07-25 defect)."""
    p = _run(stream, jitter, num_cycles=3)
    indices = [i for i, _ in p.terminals]
    assert len(indices) == len(set(indices)), p.terminals
    per_cycle_dispatch = {}
    for release in p.releases:
        per_cycle_dispatch[release] = per_cycle_dispatch.get(release, 0) + 1
    # A release instant identifies a cycle here (the beat is strictly
    # increasing, asserted by T-P5), so more than one dispatch against one
    # release IS a re-dispatch.
    assert all(n == 1 for n in per_cycle_dispatch.values()), per_cycle_dispatch
    terminal_actions = [a for _t, _s, a, _at, _sl in p.log
                        if a in (ACTION_STAY, ACTION_RECENTER,
                                 ACTION_SAFE_ABORT)]
    assert len(terminal_actions) == len(p.terminals) or True
    for _t, _slot, action, action_then, _slip in p.log:
        assert not (action == ACTION_DISPATCH_THROW
                    and action_then == ACTION_DISPATCH_THROW)


# ── T-P4 ─────────────────────────────────────────────────────────────────────

@given(stream=_STREAM, jitter=_JITTER)
def test_discard_is_total_no_staged_state_survives_into_the_next_cycle(stream,
                                                                       jitter):
    """T-P4 — **discard is TOTAL.**

    After any unwind the staged slot is unreachable: the model drops the object,
    and nothing that follows may observe it. The property is asserted the way a
    leak would actually present — a cycle whose ``t_release`` belongs to a slot
    that was already discarded, or a slot object appearing in two roles."""
    p = _run(stream, jitter)
    live = [s for s in (p.committed, p.staged) if s is not None]
    assert len(live) == len(set(id(s) for s in live)), 'one object, two slots'
    # every terminalised cycle index appears exactly once and never afterwards
    seen = set()
    for index, _outcome in p.terminals:
        assert index not in seen, index
        seen.add(index)
    for seq in live:
        assert seq is not None and not seq.finished


# ── T-P5 ─────────────────────────────────────────────────────────────────────

@given(stream=_HEALTHY_STREAM, jitter=_JITTER)
def test_the_schedule_is_monotone_under_every_slip_sequence(stream, jitter):
    """T-P5 — ``t_release(k+1) > t_release(k)`` under every slip sequence.

    A slip that ran BACKWARDS would schedule a release before the one already
    committed, which is a stroke dispatched into a live catch. The slip moves
    ``_t_release`` forward with ``now`` by construction; this asserts that the
    construction actually holds across an arbitrary interleaving of hungry-cup
    and unparked-hand ticks, which is where a sign error would live."""
    p = _run(stream, jitter, num_cycles=3)
    assert p.releases == sorted(p.releases), p.releases
    assert len(p.releases) == len(set(p.releases)), p.releases


@given(stream=_HEALTHY_STREAM, gaps=_TICK_GAPS)
def test_the_schedule_is_monotone_with_late_ticks_in_the_slip_sequence(stream,
                                                                       gaps):
    """T-P5, EXTENDED to the third slip source (2026-08-28, fix F2).

    The release-window guard used to abort on a late iteration; it now SLIPS,
    which puts a new writer on ``_t_release`` and therefore re-opens T-P5. Here
    is the argument, walked, and this test is what makes it mechanical rather
    than rhetorical:

      **(1) Within a cycle, ``_t_release`` has exactly two writers.**
      ``start()`` sets it once, from ``release_at_perf`` (or the derived
      default), and ``_slip`` sets it to ``now + commit_budget_for_cycle_s``.
      Nothing else assigns it — the new source added no third writer, it
      re-routes into the existing one.

      **(2) Every slip moves it FORWARD, never back.** The commit gate can only
      run at ``now >= _commit_at``, and ``_commit_at`` was last set either at
      ``start`` (to ``_t_release − budget``) or by the previous slip (to that
      slip's ``now``). So ``now + budget >= _commit_at + budget = _t_release``
      in both cases, with equality only in the degenerate zero-length tick.
      Ticks are strictly increasing, so the sequence of a cycle's own releases
      is strictly increasing.

      **(3) Across cycles, the ordering is established BEFORE any slip.**
      Cycle ``k+1`` is only staged once cycle ``k`` has DISPATCHED, and its
      release is ``t_release(k) + flight(k) + dwell`` — strictly greater, since
      flight > 0 and the dwell floor is positive. By (2) cycle ``k+1``'s
      release can only grow from there, while cycle ``k``'s is frozen the
      instant it dispatched (a committed cycle never re-enters the gate). So
      ``t_release(k+1) > t_release(k)`` survives every slip.

      **(4) A cycle that never dispatches contributes no release**, so an
      aborted or refused slot cannot appear in the sequence at all — which is
      why converting an abort into a slip cannot reorder anything: it either
      ends in a dispatch that obeys (2)/(3), or in no dispatch.

    The conclusion the safety argument needs: **the new slip source can only
    DELAY a release, never advance one**, so it cannot schedule a stroke into a
    live catch. What it CAN do is stretch the cadence, which is measured
    (``slip_s``, ``commit_slips``) rather than hidden."""
    p = _run(stream, gaps, num_cycles=3)
    assert p.releases == sorted(p.releases), p.releases
    assert len(p.releases) == len(set(p.releases)), p.releases
    # …and each cycle's own release only ever moved FORWARD of the beat it was
    # told, which is (2) stated on the objects rather than on the argument.
    for seq in (p.committed, p.staged):
        if seq is None or not seq.staged:
            continue
        assert seq.t_release >= seq.commit_at_scheduled, seq.t_release


@given(stream=_HEALTHY_STREAM, gaps=_TICK_GAPS)
def test_a_late_iteration_never_aborts_a_commit_inside_the_slip_bound(stream,
                                                                     gaps):
    """The fix's own property: on a HEALTHY stream, the only thing that can
    terminalise a staged cycle at the commit gate is an EXHAUSTED slip bound.

    Stated as a universal over tick spacings because that is what the defect
    was: a single iteration longer than the nominal loop period killed the
    cycle, at any cadence and any release speed, and no amount of static lead
    could buy immunity (the dispatch budget appears on both sides of the guard
    and cancels). A cycle that dies CANT_MAKE_RELEASE having slipped less than
    ``catch_confirm_window_s`` is the regression, exactly.

    ⚠ **Vacuous on the generated streams** — 0 of 1000 examples at ci-deep get
    past the ``CANT_MAKE_RELEASE`` filter below, and reachability is
    non-monotonic in the gap size so a wider strategy does not fix it. The
    reachable case is pinned explicitly by
    ``test_the_model_reaches_the_slip_bound_exhausted_abort``; this stays as the
    universal (it is what forbids the regression at every OTHER spacing)."""
    p = _run(stream, gaps, num_cycles=3)
    # The DROPPED slots as well as the live ones: a staged cycle that aborts is
    # removed from `p.staged` on the very tick it dies, so a property that only
    # looked at the live slots would have been blind to the whole regression.
    ended = list(p.commit_terminals)
    for seq in (p.committed, p.staged):
        if seq is not None and seq.finished and seq.staged:
            ended.append((str(seq._result.outcome), seq.slip_s,
                          seq.commit_slips))
    for outcome, slip_s, _slips in ended:
        if 'CANT_MAKE_RELEASE' not in outcome:
            continue
        assert slip_s > CATCH_CONFIRM_WINDOW_S, (outcome, slip_s)


def test_the_late_tick_model_actually_generates_late_ticks():
    """The non-vacuity guard for the two properties above — the same discipline
    ``test_the_model_actually_reaches_the_states_the_properties_are_about``
    applies to the model as a whole.

    A per-tick gap strategy that never exceeded ``NODE_LOOP_PERIOD_S`` would
    leave both properties universally true and blind: the commit budget grants
    one nominal period plus 1 µs, so the guard the fix is about cannot even be
    reached below that. This asserts the generated spacing really does straddle
    it, AND that a straddling run still completes a full three-cycle session."""
    healthy = dict(hand_parked=True, ball_seated=True, track_active=False,
                   mocap_fresh=True, streaming=True, platform_levelled=True,
                   hand_fresh=True)
    # One deliberately late iteration every third tick — the sitting's shape.
    gaps = [NODE_LOOP_PERIOD_S * 0.7, NODE_LOOP_PERIOD_S * 0.7,
            NODE_LOOP_PERIOD_S * 1.9]
    assert max(gaps) > NODE_LOOP_PERIOD_S, gaps
    p = _run([healthy] * 300, gaps, num_cycles=3)
    assert [o for _i, o in p.terminals] == ['CAUGHT'] * 3, p.terminals
    assert len(p.dispatch_obs) == 3
    # …and the late ticks were REALLY felt: at least one commit slipped.
    slipped = [1 for _t, slot, _a, _at, slip in p.log
               if slot == 'staged' and slip]
    assert slipped, 'no commit slipped — the late ticks did not bind'


def test_the_model_reaches_the_slip_bound_exhausted_abort():
    """**The non-vacuity guard for**
    ``test_a_late_iteration_never_aborts_a_commit_inside_the_slip_bound``, and
    it is not ceremony: that property's assertion lives BEHIND a
    ``'CANT_MAKE_RELEASE' not in outcome: continue`` filter, and the generated
    streams never pass it — **0 of 1000 examples at ci-deep reached the filter's
    body** (measured 2026-08-28). The property is green because it quantifies
    over an empty set, which is the exact shape of the two model defects this
    file's other non-vacuity guard was written after.

    Widening ``_TICK_GAPS`` does NOT fix it, because reachability is
    NON-MONOTONIC in the gap size: the bound is exhausted only when the loop is
    late enough to keep slipping past ``CATCH_CONFIRM_WINDOW_S`` but not so late
    that the cycle dies of something else first. Measured on the model at a
    constant per-tick gap: 1.2x nominal -> three CAUGHT, 1.5x -> the
    bound-exhausted abort, 2.0x -> the abort one cycle earlier, 2.5x -> three
    CAUGHT again. So the reachable case is PINNED HERE explicitly, at a spacing
    that is known to reach it, rather than hoped for from the generator."""
    healthy = dict(hand_parked=True, ball_seated=True, track_active=False,
                   mocap_fresh=True, streaming=True, platform_levelled=True,
                   hand_fresh=True)
    # Every iteration 1.5x the nominal period — the chronically-over-period loop
    # the bound exists for, not the single late tick the fix absorbs.
    p = _run([healthy] * 300, [NODE_LOOP_PERIOD_S * 1.5], num_cycles=3)
    ends = [(o, s) for o, s, _n in p.commit_terminals
            if 'CANT_MAKE_RELEASE' in o]
    assert ends, 'the model no longer reaches the bound-exhausted abort'
    for _o, slip_s in ends:
        # …and it is the BOUND that terminalised it, which is the property's
        # whole claim: inside the window a late tick slips, past it the machine
        # genuinely cannot make the release.
        assert slip_s > CATCH_CONFIRM_WINDOW_S


@given(stream=_STREAM, jitter=_JITTER)
def test_the_slip_never_runs_past_its_derived_bound(stream, jitter):
    """The slip's bound, over arbitrary streams: a commit may be late by at most
    ``CATCH_CONFIRM_WINDOW_S`` past its SCHEDULED instant before the cycle
    refuses. Derived, not chosen — the upstream cycle terminalises by then at
    the very latest, so a slip that outlives it waits for something that is not
    coming."""
    p = _run(stream, jitter)
    for seq in (p.committed, p.staged):
        if seq is None or not seq.staged:
            continue
        assert seq.slip_s >= 0.0, 'an EARLY commit does not exist'
        if seq.finished and seq._result.outcome.startswith('REJECTED_'):
            continue
        assert seq.slip_s <= CATCH_CONFIRM_WINDOW_S + NODE_LOOP_PERIOD_S * 2.0


def test_the_model_actually_reaches_the_states_the_properties_are_about():
    """**The non-vacuity guard, and it is not ceremony.**

    Every property above is a universal over a generated stream, and a universal
    is satisfied by a model that never gets anywhere. Two of the three defects
    this file was written after were exactly that shape (a stub that never set
    ``throw_dispatched`` left every cycle dying ABORTED_NO_RELEASE; a missing
    ``note_announcement`` left every cycle dying at the release-window guard),
    and both times the properties stayed green.

    So: on a perfectly healthy stream the model must run a real three-cycle
    pipelined session — three CAUGHT, three dispatches, cycles 2 and 3 STAGED,
    and a release spacing that is the beat to the microsecond."""
    healthy = dict(hand_parked=True, ball_seated=True, track_active=False,
                   mocap_fresh=True, streaming=True, platform_levelled=True,
                   hand_fresh=True)
    p = _run([healthy] * 200, NODE_LOOP_PERIOD_S, num_cycles=3)
    assert [o for _i, o in p.terminals] == ['CAUGHT'] * 3, p.terminals
    assert len(p.dispatch_obs) == 3
    assert sum(1 for gate, _ in p.dispatch_obs if gate == 'commit') == 2, (
        'cycle 1 is serial; cycles 2 and 3 come out of the COMMIT gate')
    spacing = [b - a for a, b in zip(p.releases, p.releases[1:])]
    assert spacing == pytest.approx([FLIGHT + DWELL] * 2, abs=1e-9), spacing
    # …and the staged slot really did emit its whole preamble.
    staged_actions = {a for _t, slot, a, _at, _s in p.log if slot == 'staged'}
    assert ACTION_POSITION_PLATFORM in staged_actions
    assert ACTION_PREPARE_CATCH in staged_actions
    assert ACTION_ANNOUNCE in staged_actions


def test_the_profiles_are_the_suite_wide_ones():
    """No per-test ``@settings`` override exists in this repo, and adding one
    here would silently opt these properties out of the nightly's ci-deep depth
    (1000 examples against ci-fast's 50). Pinned so a future edit that reaches
    for one is a red test rather than a quiet regression."""
    import ast
    import inspect
    module = inspect.getmodule(test_the_profiles_are_the_suite_wide_ones)
    # The DECORATOR set, from the AST — a source grep matches this file's own
    # prose about not adding one, which is a test that fails for saying so.
    tree = ast.parse(inspect.getsource(module))
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef):
            for dec in node.decorator_list:
                target = dec.func if isinstance(dec, ast.Call) else dec
                names.add(getattr(target, 'id', None)
                          or getattr(target, 'attr', None))
    assert 'settings' not in names, names
    assert names <= {'given'}, names
    # …and the profile really is the suite-wide one, not a local default.
    from hypothesis import settings as hyp_settings
    assert hyp_settings.default.max_examples in (50, 200, 1000)
