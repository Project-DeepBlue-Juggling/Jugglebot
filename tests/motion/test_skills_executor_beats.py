"""P4 (learner-generalisation, D-P4-4): ``SkillExecutor.request_beat_change``
-- a per-beat change of a RUNNING Columns stream, applied by swapping the
executor's Schedule for its recompile (``schedule.columns_change`` +
``Schedule.with_events``) from the first changeable Throw on.

Harness: the REAL compiler (``compile_columns``, 125 mm, Apex 0.95, Dwell
0.27, 12 Throws on a wall clock at t0 = 100 s), a recording installer that
accepts everything, a scripted always-OK command-time checker (so the P3a
early command runs and fills ``_early_by_idx``, the state the first-changeable
rule must respect), and a perfect analytic tracker that reads the executor's
LIVE Schedule (a changed Throw lands where the candidate says).  Ticks at the
skill node's 40 Hz.

What is pinned: the first-changeable rule against the dispatch state on both
sides of the early-command window (+ BEAT_CHANGE_GUARD_S), the swap leaves
every dispatched Skill and its result alone and the rest dispatch in order at
the candidate's instants with the new command, the pattern runs to its Stop
and closing REST; each refusal by name with nothing changed; the compiler's
``BEAT_*`` message prefix contract; the beat_checker; the Learner context's
dwell (deliverable C); the survivor tail clearing ``source`` (B); the
pending-outcome re-derivation and the note clearing; the identity band.
"""

from __future__ import annotations

import dataclasses
import re

import pytest

from jugglebot.motion.skills import admissible as adm
from jugglebot.motion.skills import executor as ex
from jugglebot.motion.skills import learner as lr
from jugglebot.motion.skills import plant_model as pm
from jugglebot.motion.skills import schedule as sc
from jugglebot.motion.skills import segments as sg
from jugglebot.motion.skills import sites as st
from jugglebot.motion.trajectory import ballistics_bc as bal

_T0 = 100.0
_N = 12
_APEX = 0.95
_DWELL = 0.27
_DT = 0.025                    # the skill node's 40 Hz tick


def _columns(n=_N, apex=_APEX, dwell=_DWELL):
    return sc.compile_columns(
        sc.Pattern(sites=st.columns_sites(125.0), apex_m=apex, dwell_s=dwell,
                   n_throws=n), _T0)


class _Installer:
    def __init__(self):
        self.calls = []

    def __call__(self, kind, terminal, t_now_s, ball_id=None):
        self.calls.append((kind, terminal, t_now_s, ball_id))
        return ex.InstallResult(True, 'OK', 'ok', 0.0, splice_k=0)


class _OkChecker:
    def __call__(self, kind, terminal, t_abs_s, *, ball_id):
        return adm.CommandVerdict(True, 0.5, 'OK', True)


class _Tracker:
    """Perfect fits off the executor's LIVE Schedule: ball ``b`` lands at
    its next CATCH's site and instant, arriving along the flight of the
    Throw that put it there."""

    def __init__(self):
        self.x = None
        self.t = 0.0

    def __call__(self, ball_id):
        sched = self.x.schedule
        for k, sk in enumerate(sched.skills):
            if (sk.kind != sg.CATCH or sk.ball_id != ball_id
                    or float(sk.t_abs_s) <= self.t - 0.05):
                continue
            prev = ex._previous_release(sched, k, ball_id)
            if prev is None:
                flight = sched.flight_for(ball_id)
                rel = sk.site.throw_site_mm()
            else:
                flight = sc.flight_s(float(prev[2][1]))
                rel = prev[0].throw_site_mm()
            pos = sk.site.catch_site_mm()
            vel = bal.arrival_velocity(bal.launch_velocity(rel, pos, flight),
                                       flight)
            return ex.Landing(pos_mm=pos, vel_mm_s=vel,
                              t_land_abs_s=float(sk.t_abs_s), from_fit=True)
        return None


def _accept(candidate, first_idx):
    """A trivial planner walk that admits every candidate (the executor
    refuses BEAT_NO_CHECKER without one)."""
    return None


def _executor(sched=None, **kw):
    """``beat_checker`` defaults to :func:`_accept`; pass ``None`` to build
    one with no planner walk bound."""
    kw.setdefault('beat_checker', _accept)
    sched = _columns() if sched is None else sched
    inst = _Installer()
    trk = _Tracker()
    x = ex.SkillExecutor(sched, inst, tracker=trk, checker=_OkChecker(),
                         early_command=True, **kw)
    trk.x = x
    return x, inst, trk


def _run(x, trk, t_from, t_to):
    """Tick on the 40 Hz grid from ``t_from`` while ``t < t_to``; returns
    the next tick instant."""
    t = t_from
    while t < t_to - 1e-12:
        trk.t = t
        x.tick(t)
        t += _DT
    return t


def _run_to_end(x, trk, t):
    t_end = max(sk.t_abs_s for sk in x.schedule.skills) + 4.0
    while t < t_end and not x.done:
        trk.t = t
        x.tick(t)
        t += _DT
    return t


def _snapshot(x):
    return (x.schedule, set(x.dispatched), list(x.results),
            dict(x._u_cache), set(x._learned_u), dict(x._ctx_cache),
            dict(x._early_by_idx), x._beat_from_idx, x._identity_beat,
            len(x.notices), x.n_throws)


def _assert_unchanged(x, snap):
    (sched, disp, results, u, learned, ctx, early, frm, ib, nn, nt) = snap
    assert x.schedule is sched
    assert x.dispatched == disp and x.results == results
    assert x._u_cache == u and x._learned_u == learned
    assert x._ctx_cache == ctx and x._early_by_idx == early
    assert x._beat_from_idx == frm and x._identity_beat == ib
    assert len(x.notices) == nn and x.n_throws == nt


def _start():
    return _T0 - 1.0


# ---------------------------------------------------------------------------
# the first changeable Throw, against the dispatch state, by hand
# ---------------------------------------------------------------------------

_J = 6


def _opens(sched, j):
    return (float(sched.skills[j].dispatch_s()) - ex.EARLY_COMMAND_MAX_S
            - ex.BEAT_CHANGE_GUARD_S)


@pytest.mark.parametrize('offset_s, want_j', [(-0.001, _J), (+0.001, _J + 1)])
def test_the_change_applies_from_exactly_the_first_changeable_throw(
        offset_s, want_j):
    """Requested 1 ms before Skill 6's guarded early window opens -> Throw 6
    (Skill 5 is already dispatched, Skill 6 untouched); 1 ms after it ->
    Throw 7, although Skill 6 has NO early entry yet (its window proper opens
    BEAT_CHANGE_GUARD_S later): the guard, not the cache, decides."""
    x, inst, trk = _executor()
    old = x.schedule
    t_req = _opens(old, _J) + offset_s
    t = _run(x, trk, _start(), t_req)
    assert _J - 1 in x.dispatched and _J not in x.dispatched
    assert _J not in x._early_by_idx and _J not in x._u_cache
    assert x.dispatched == set(range(_J))
    results_before = list(x.results)
    early_before = dict(x._early_by_idx)
    res = x.request_beat_change(t_req, apex_m=0.85)
    assert res.ok, res
    assert res.code == ex.BEAT_CHANGE_OK and res.from_throw == want_j
    cand = x.schedule
    assert cand is not old and cand.source is not None
    assert x.results == results_before
    assert all(a is b for a, b in zip(x.results, results_before))
    assert x._early_by_idx == early_before
    for k in range(want_j):
        assert ex._same_value(cand.skills[k], old.skills[k])
    assert cand.skills[want_j].then_throw.y_d[1] == pytest.approx(0.85)
    assert old.skills[want_j].then_throw.y_d[1] == pytest.approx(_APEX)
    assert x._beat_from_idx == want_j
    t = _run_to_end(x, trk, t)
    assert not x.attempt_ended, x.end_message
    assert x.dispatched == set(range(len(cand.skills)))
    order = [i for i, _s, _r in x.results]
    assert order == list(range(len(cand.skills)))
    assert len(inst.calls) == len(x.results)          # no re-sends
    for (i, sk, r), (kind, term, t_now, _b) in zip(x.results, inst.calls):
        assert r.accepted
        want = cand.skills[i] if i >= _J else old.skills[i]
        assert sk is want or ex._same_value(sk, want)
        d_s = float(cand.skills[i].dispatch_s())
        assert d_s <= t_now + 1e-9 < d_s + _DT + 1e-9
    # the folded CATCH carrying Throw j: the installer got the NEW Apex
    _k, term_j, _t, _b = inst.calls[want_j]
    assert term_j.then_throw.flight_s == pytest.approx(sc.flight_s(0.85))
    if want_j > _J:
        _k, term_6, _t, _b = inst.calls[_J]
        assert term_6.then_throw.flight_s == pytest.approx(sc.flight_s(_APEX))
    assert cand.skills[-1].kind == sg.REST
    assert cand.skills[-3].then_throw.shadow_landing          # the Stop


def test_inside_the_early_window_the_throw_is_no_longer_changeable():
    x, _inst, trk = _executor()
    d6 = float(x.schedule.skills[_J].dispatch_s())
    t_req = d6 - 0.1
    _run(x, trk, _start(), t_req)
    assert _J in x._early_by_idx and _J not in x.dispatched
    assert x.first_changeable_throw(t_req) == _J + 1


@pytest.mark.parametrize('kw, apexes, dwell', [
    (dict(apex_m=0.85), (0.85, 0.85), _DWELL),
    (dict(dwell_s=0.30), (_APEX, _APEX), 0.30),
    (dict(apex_m=0.85, dwell_s=0.30), (0.85, 0.85), 0.30),
    (dict(apex_m=0.95, apex_m_ball_b=0.85), (0.95, 0.85), None),
])
def test_apex_dwell_and_both_run_to_the_stop(kw, apexes, dwell):
    x, inst, trk = _executor()
    old = x.schedule
    t_req = _opens(old, _J) - 0.001
    t = _run(x, trk, _start(), t_req)
    res = x.request_beat_change(t_req, **kw)
    assert res.ok and res.from_throw == _J, res
    cand = x.schedule
    ev = cand.source.events
    for i in range(_J, _N):
        assert ev[i].apex_m == pytest.approx(apexes[ev[i].ball_id])
        sk = cand.skills[i]
        assert sk.then_throw.y_d[1] == pytest.approx(apexes[sk.ball_id])
    if dwell is not None:
        # past the transition Throw every carried dwell is the new one
        for i in range(_J + 2, _N):
            sk = cand.skills[i]
            assert (sk.then_throw.t_release_abs_s - sk.t_abs_s
                    == pytest.approx(dwell, abs=1e-9))
    t = _run_to_end(x, trk, t)
    assert not x.attempt_ended, x.end_message
    assert x.dispatched == set(range(len(cand.skills)))
    for i in range(_J, _N):
        _k, term, _t, _b = inst.calls[i]
        assert term.then_throw is not None or cand.skills[i].then_throw is None
        if cand.skills[i].then_throw is not None:
            assert term.then_throw.flight_s == pytest.approx(
                sc.flight_s(apexes[cand.skills[i].ball_id]))


# ---------------------------------------------------------------------------
# refusals: one each, nothing changed
# ---------------------------------------------------------------------------

def _mid(x, trk):
    t_req = _opens(x.schedule, _J) - 0.001
    _run(x, trk, _start(), t_req)
    return t_req


def test_refused_not_supported_without_a_source():
    x, _i, trk = _executor(dataclasses.replace(_columns(), source=None))
    t = _mid(x, trk)
    snap = _snapshot(x)
    res = x.request_beat_change(t, apex_m=0.85)
    assert (res.ok, res.code, res.from_throw) == (
        False, ex.BEAT_NOT_SUPPORTED, None)
    _assert_unchanged(x, snap)


def test_refused_no_checker_without_a_planner_walk():
    """Fail-safe by default: no ``beat_checker`` -> every change refused
    BEAT_NO_CHECKER, nothing changed, and the Attempt runs as compiled."""
    x, _i, trk = _executor(beat_checker=None)
    t = _mid(x, trk)
    snap = _snapshot(x)
    res = x.request_beat_change(t, apex_m=0.85)
    assert (res.ok, res.code, res.from_throw) == (
        False, ex.BEAT_NO_CHECKER, None)
    assert 'beat_checker=' in res.detail
    _assert_unchanged(x, snap)
    _run_to_end(x, trk, t)
    assert not x.attempt_ended
    assert all(sk.then_throw.y_d[1] == _APEX
               for sk in x.schedule.skills if sk.then_throw is not None)


@pytest.mark.parametrize('how', ['ended', 'end_code', 'stop', 'closing'])
def test_refused_closing(how):
    x, _i, trk = _executor()
    t = _mid(x, trk)
    if how == 'ended':
        x.attempt_ended, x.end_code = True, 'STOPPED'
    elif how == 'end_code':
        x.end_code = ex.MISSED_CATCH
    elif how == 'stop':
        x.stop_terminals(t)
    else:
        x._closing_state = 'waiting'
    snap = _snapshot(x)
    res = x.request_beat_change(t, apex_m=0.85)
    assert not res.ok and res.code == ex.BEAT_CLOSING, res
    _assert_unchanged(x, snap)


@pytest.mark.parametrize('kw', [dict(), dict(apex_m=_APEX),
                                dict(dwell_s=_DWELL),
                                dict(apex_m=_APEX, dwell_s=_DWELL)])
def test_refused_no_change(kw):
    x, _i, trk = _executor()
    t = _mid(x, trk)
    snap = _snapshot(x)
    res = x.request_beat_change(t, **kw)
    assert not res.ok and res.code == ex.BEAT_NO_CHANGE, res
    _assert_unchanged(x, snap)


def test_refused_too_late():
    x, _i, trk = _executor()
    stop_carrier = _N - 1
    t = _opens(x.schedule, stop_carrier) + 0.001
    _run(x, trk, _start(), t)
    snap = _snapshot(x)
    res = x.request_beat_change(t, apex_m=0.85)
    assert not res.ok and res.code == ex.BEAT_TOO_LATE, res
    assert res.from_throw is None
    _assert_unchanged(x, snap)


@pytest.mark.parametrize('kw, code', [
    (dict(dwell_s=0.9), sc.BEAT_NO_TRANSIT),
    (dict(dwell_s=0.01), sc.BEAT_DWELL_TOO_SHORT),
    (dict(apex_m=0.3), sc.BEAT_DWELL_TOO_SHORT),
])
def test_refused_by_the_compiler_with_its_code(kw, code):
    x, _i, trk = _executor()
    t = _mid(x, trk)
    snap = _snapshot(x)
    res = x.request_beat_change(t, **kw)
    assert not res.ok and res.code == code, res
    assert res.detail.startswith(code + ':') and res.from_throw == _J
    _assert_unchanged(x, snap)


@pytest.mark.parametrize('kw', [dict(apex_m=float('nan')),
                                dict(dwell_s=-0.1), dict(apex_m_ball_b=0.0),
                                # not numbers at all (audit 2026-11-06): these
                                # raised out of the request instead of refusing
                                dict(apex_m='high'), dict(dwell_s=''),
                                dict(apex_m_ball_b=object())])
def test_refused_invalid_values(kw):
    x, _i, trk = _executor()
    t = _mid(x, trk)
    snap = _snapshot(x)
    res = x.request_beat_change(t, **kw)
    assert not res.ok and res.code == sc.BEAT_STREAM_INVALID, res
    _assert_unchanged(x, snap)


def test_the_compilers_beat_errors_start_with_their_code():
    """The prefix contract ``request_beat_change`` parses: every ValueError
    columns_change / with_events raises for a beat starts ``BEAT_<NAME>: ``
    and ``<NAME>`` is a schedule.py constant."""
    sched = _columns()
    ev = sched.source.events
    changed = sc.columns_change(ev, 4, apex_m=0.85)
    cases = [lambda: sc.columns_change(ev, 5, dwell_s=0.9),
             lambda: sc.columns_change(ev, 5, dwell_s=0.01),
             lambda: sc.columns_change(ev, 5, apex_m=0.3),
             lambda: sc.columns_change(ev, 1, apex_m=0.85),
             lambda: sc.columns_change(ev, _N, apex_m=0.85),
             lambda: sc.columns_change(changed, 6, apex_m=0.9),
             lambda: dataclasses.replace(sched, source=None).with_events(ev)]
    codes = set()
    for case in cases:
        with pytest.raises(ValueError) as info:
            case()
        msg = str(info.value)
        m = re.match(r'^(BEAT_[A-Z_]+): ', msg)
        assert m, msg
        assert getattr(sc, m.group(1)) == m.group(1)
        assert ex._beat_code(msg) == m.group(1)
        codes.add(m.group(1))
    assert len(codes) >= 4
    assert ex._beat_code('apex_m must be > 0') == ex.BEAT_COMPILE_REFUSED


def test_a_second_change_while_the_stream_re_centres_is_refused_by_the_compiler():
    x, _i, trk = _executor()
    t = _mid(x, trk)
    assert x.request_beat_change(t, apex_m=0.85).ok
    t2 = _opens(x.schedule, _J + 1) - 0.001        # first changeable = J+1
    _run(x, trk, t, t2)
    assert x.first_changeable_throw(t2) == _J + 1
    snap = _snapshot(x)
    res = x.request_beat_change(t2, apex_m=0.90)
    assert not res.ok and res.code == sc.BEAT_CHANGE_RANGE, res
    _assert_unchanged(x, snap)


def _patch_with_events(monkeypatch, edit):
    real = sc.Schedule.with_events

    def fake(self, events):
        cand = real(self, events)
        return dataclasses.replace(cand, skills=edit(cand.skills))
    monkeypatch.setattr(sc.Schedule, 'with_events', fake)


@pytest.mark.parametrize('edit', [
    lambda sk: (sk[0], dataclasses.replace(
        sk[1], window_s=sk[1].window_s + 1e-6)) + tuple(sk[2:]),
    lambda sk: tuple(sk[:-1]),
], ids=['skill-1-moved', 'count'])
def test_refused_prefix_changed(monkeypatch, edit):
    x, _i, trk = _executor()
    t = _mid(x, trk)
    _patch_with_events(monkeypatch, edit)
    snap = _snapshot(x)
    res = x.request_beat_change(t, apex_m=0.85)
    assert not res.ok and res.code == ex.BEAT_PREFIX_CHANGED, res
    _assert_unchanged(x, snap)


def test_the_beat_checker_gets_the_candidate_and_the_first_changed_index():
    seen = []

    def checker(cand, first):
        seen.append((cand, first))
        return None
    x, _i, trk = _executor(beat_checker=checker)
    t = _mid(x, trk)
    res = x.request_beat_change(t, apex_m=0.85)
    assert res.ok
    (cand, first), = seen
    assert cand is x.schedule and first == _J == res.from_throw
    assert cand.skills[first].then_throw.y_d[1] == pytest.approx(0.85)


@pytest.mark.parametrize('answer, code', [
    ('LIMIT_JERK at Skill 9 (212000 > 200000)', ex.BEAT_PLANNER_REFUSED),
    (RuntimeError('walker crashed'), ex.BEAT_CHECKER_ERROR),
])
def test_the_beat_checker_refuses(answer, code):
    def checker(cand, first):
        if isinstance(answer, Exception):
            raise answer
        return answer
    x, _i, trk = _executor(beat_checker=checker)
    t = _mid(x, trk)
    snap = _snapshot(x)
    res = x.request_beat_change(t, apex_m=0.85)
    assert not res.ok and res.code == code, res
    assert ('walker crashed' in res.detail if code == ex.BEAT_CHECKER_ERROR
            else res.detail == answer)
    _assert_unchanged(x, snap)
    t = _run_to_end(x, trk, t)                   # and the Attempt runs on
    assert not x.attempt_ended
    assert x.dispatched == set(range(len(x.schedule.skills)))


def test_a_refused_change_then_an_accepted_one():
    x, _i, trk = _executor()
    t = _mid(x, trk)
    old = x.schedule
    assert x.request_beat_change(t, dwell_s=0.9).code == sc.BEAT_NO_TRANSIT
    assert x.schedule is old
    res = x.request_beat_change(t, apex_m=0.85)
    assert res.ok and res.from_throw == _J and x.schedule is not old


def test_two_accepted_changes_far_enough_apart():
    x, inst, trk = _executor()
    t = _mid(x, trk)
    assert x.request_beat_change(t, apex_m=0.85).from_throw == _J
    j2 = _J + 4
    t2 = _opens(x.schedule, j2) - 0.001
    _run(x, trk, t, t2)
    assert x.first_changeable_throw(t2) == j2
    res = x.request_beat_change(t2, apex_m=1.0)
    assert res.ok and res.from_throw == j2, res
    assert x._beat_from_idx == _J
    _run_to_end(x, trk, t2)
    assert not x.attempt_ended
    cand = x.schedule
    assert x.dispatched == set(range(len(cand.skills)))
    for i in range(2, _N):
        want = _APEX if i < _J else 0.85 if i < j2 else 1.0
        assert cand.skills[i].then_throw.y_d[1] == pytest.approx(want)
        assert inst.calls[i][1].then_throw.flight_s == pytest.approx(
            sc.flight_s(want))


# ---------------------------------------------------------------------------
# B: the survivor tail clears the source
# ---------------------------------------------------------------------------

def test_a_change_after_a_drop_is_refused_not_supported():
    x, _i, trk = _executor()
    t = _mid(x, trk)
    y_idx, y_skill = x._survivor_catch(0)
    x._install_survivor_tail(y_idx, y_skill, 0)
    assert x.schedule.source is None
    snap = _snapshot(x)
    res = x.request_beat_change(t, apex_m=0.85)
    assert not res.ok and res.code == ex.BEAT_NOT_SUPPORTED, res
    _assert_unchanged(x, snap)


# ---------------------------------------------------------------------------
# index-keyed state after the swap
# ---------------------------------------------------------------------------

def test_notes_at_or_past_the_first_changed_skill_are_cleared():
    x, _i, trk = _executor()
    t = _mid(x, trk)
    x._identity_refused |= {('catch', 1), ('catch', _J), ('catch', 10),
                            ('throw', 3)}
    x._resend_notes |= {(1, 'X'), (_J, 'AIM-LATERAL-CLAMPED'), (11, 'Y')}
    assert x.request_beat_change(t, apex_m=0.85).from_throw == _J
    assert x._identity_refused == {('catch', 1), ('throw', 3)}
    assert (1, 'X') in x._resend_notes
    assert not [key for key in x._resend_notes if key[0] >= _J]


def test_pending_outcomes_re_derive_their_next_release():
    """Throw J-2's row closes its window at its ball's NEXT release, Throw
    J, which a Dwell change moves: re-derived from the candidate."""
    x, _i, trk = _executor(on_experience=lambda row: None,
                           observer=lambda b, t: ex.CAUGHT_EVIDENCE)
    t = _mid(x, trk)
    old = x.schedule
    pend = next(p for p in x._pending_outcomes
                if p.t_release_s == old.skills[_J - 2].then_throw
                .t_release_abs_s)
    assert pend.t_next_release_s == old.skills[_J].then_throw.t_release_abs_s
    assert x.request_beat_change(t, dwell_s=0.30).ok
    new_rel = x.schedule.skills[_J].then_throw.t_release_abs_s
    assert abs(new_rel - old.skills[_J].then_throw.t_release_abs_s) > 0.02
    assert pend.t_next_release_s == new_rel
    assert pend.t_seat_close_s == ex._seat_close(
        x.schedule, pend.ball_id, pend.t_land_scheduled_s, new_rel)


def test_the_identity_band_follows_a_shorter_beat():
    x, _i, trk = _executor()
    t = _mid(x, trk)
    assert x._identity_beat_s() == x.schedule.beat_s      # as compiled
    assert x.request_beat_change(t, apex_m=0.75).ok
    new_beat = sc.beat_s(sc.flight_s(0.75), _DWELL)
    assert new_beat < x.schedule.beat_s
    assert x._identity_beat_s() <= new_beat + 1e-9
    assert x._identity_beat_s() > 0.5 * new_beat


# ---------------------------------------------------------------------------
# C: the Learner context's dwell; reports; the operator line
# ---------------------------------------------------------------------------

def test_an_unchanged_stream_keeps_dwell_for_bit_for_bit():
    x, _i, trk = _executor()
    _run_to_end(x, trk, _start())
    assert x._ctx_cache
    for k, ctx in x._ctx_cache.items():
        assert ctx.dwell_s == x.schedule.dwell_for(x.schedule.skills[k].ball_id)


def _learner():
    return lr.Learner(model=pm.PlantModel(), config=lr.LearnerConfig())


def test_rows_after_a_dwell_change_carry_the_skills_own_dwell():
    rows = []
    x, _i, trk = _executor(learner=_learner(), on_experience=rows.append,
                           observer=lambda b, t: ex.CAUGHT_EVIDENCE)
    t = _mid(x, trk)
    assert x.request_beat_change(t, dwell_s=0.30).ok
    _run_to_end(x, trk, t)
    assert not x.attempt_ended, x.end_message
    cand = x.schedule
    for k, ctx in x._ctx_cache.items():
        sk = cand.skills[k]
        if k < _J:
            assert ctx.dwell_s == cand.dwell_for(sk.ball_id) == _DWELL
        else:
            assert ctx.dwell_s == (sk.then_throw.t_release_abs_s - sk.t_abs_s)
    assert x._ctx_cache[_J + 2].dwell_s == pytest.approx(0.30, abs=1e-9)
    assert x._ctx_cache[_J + 1].dwell_s != pytest.approx(0.30, abs=1e-6)
    assert rows
    late = [r for r in rows if r.dwell_s == pytest.approx(0.30, abs=1e-9)]
    early = [r for r in rows if r.dwell_s == _DWELL]
    assert late and early, sorted((r.dwell_s for r in rows))


def test_reports_and_the_operator_line_after_a_change():
    rows = []
    x, _i, trk = _executor(on_experience=rows.append,
                           observer=lambda b, t: ex.CAUGHT_EVIDENCE)
    t = _mid(x, trk)
    res = x.request_beat_change(t, apex_m=0.85, dwell_s=0.30)
    assert res.ok
    trans = x.schedule.source.events[_J + 1].dwell_s
    want = ('%.3f BEAT CHANGE from throw %d: apex 0.950 -> 0.850 m, dwell '
            '0.270 -> 0.300 s (transition dwell %.4f s on throw %d)'
            % (t, _J, trans, _J + 1))
    assert res.detail == want
    assert x.notices[-1] == ('INFO', want)
    _run_to_end(x, trk, t)
    assert x.n_throws == _N
    assert x.reports
    assert all(r.n_throws == _N for r in x.reports)
    assert sorted(r.throw_no for r in x.reports) == sorted(
        {r.throw_no for r in x.reports})


# ── audit 2026-11-06: a state conflict is a named refusal BEFORE the swap ────

@pytest.mark.parametrize('poison', ['resend_counts', 'u_cache', 'n_throws'])
def test_a_state_conflict_is_refused_by_name_and_changes_nothing(poison):
    """`first_changeable_throw` rules these out, so they are reached here by
    poisoning the executor: state held at or past the first changed Skill,
    or a Throw count that would change. Before the audit they were asserts
    AFTER `self.schedule` had been swapped (an AssertionError out of the
    request with the Schedule already replaced); now the request refuses
    `BEAT_STATE_CONFLICT` first and nothing is touched."""
    x, _i, trk = _executor()
    t = _mid(x, trk)
    j = x.first_changeable_throw(t)
    last = len(x.schedule.skills) - 1
    if poison == 'resend_counts':
        x._resend_counts[last] = 1
    elif poison == 'u_cache':
        x._u_cache[last] = ((0.0, 0.0), 0.9)
    else:
        x.n_throws += 1
    # `first_changeable_throw` itself would now answer "too late" (it requires
    # every Skill from j on untouched): pin its answer so the request reaches
    # the second line of defence under test.
    x.first_changeable_throw = lambda _t: j
    snap = _snapshot(x)
    res = x.request_beat_change(t, apex_m=1.0)
    assert not res.ok and res.code == ex.BEAT_STATE_CONFLICT, res
    _assert_unchanged(x, snap)
