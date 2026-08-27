"""THE cadence contract, pinned: **a goal the gates accept cannot abort.**

``tools/probes/cadence_rung_check.py`` is the probe; this file is the gate.  It
imports the probe rather than re-deriving anything, for the reason
``tools/probes/README.md`` gives for committing replay harnesses at all: a check
that lives only in a probe is a check nobody runs.

What it pins, in order of how much it costs to get wrong:

1. **accept-implies-flies over the whole ``(T, dwell, delay, aim)`` grid.**
   ``ABORTED_CANT_MAKE_RELEASE`` is minted in ``_step_preparing`` — the catch
   latch is UP, the announcement is out, the hand is committed — and its cleanup
   retracts the hand under a seated ball.  No goal the SESSION accepts may reach
   it from static arithmetic.  Until 2026-08-23 three published rungs of
   ``tests/hardware/session_cadence_ladder.md`` did exactly that, every cycle.
2. **the published ladder still flies** — every rung, chained and first-cycle,
   with a layer-3 artifact loaded and without.
3. **the pre-audit ladder still reds** — the regression the probe was written to
   find has to stay findable, or the probe has lost it.
4. **the PIPELINED model reproduces the plan's own floor table** (2026-08-27,
   Phase B0 / probe P3 of ``plans/active/toss-pipelined-preamble.md``), and its
   grid holds the same accept-implies-flies contract the serial one does.

⚠ **The pipelined half pins a MODEL, not shipped code.** Neither
``commit_budget_s`` nor the pipelined branch of ``required_dwell_s`` exists in
``ros_ws/`` yet; workstream B4 lands both, and when it does these numbers become
the acceptance it is measured against (``probe.commit_budget_s.__doc__`` carries
the reconciliation obligation). Every number below came out of the probe first
and was then typed here — the house's probe-before-test rule, ``CLAUDE.md``
"Empirical probe before writing tests".

Runtime note: the grid sweep drives two real FSMs at the FSM loop's measured
0.040 s period — ``toss_sequencer.NODE_LOOP_PERIOD_S``, imported by the probe,
never restated — over ~1500 grid points, which is a few seconds. Worth it for the
one class of failure that reaches the hand.

⚠ **It said "the real 0.02 s node tick" until 2026-08-26, and the probe advanced
by a local literal of that value** (audit finding B3). That is the SLEEP at the
bottom of ``_run_toss_cycle``, not what an iteration costs, and it is half what
``pre_dispatch_budget_s`` charges the same ladder. A probe whose clock runs at
half the rate the gate is charged at grants the machine lead it never has: driven
with the pre-D3 floors, the corrected probe reports **252** accept-implies-flies
violations where the old one reported **0** — including the exact
``ABORTED_CANT_MAKE_RELEASE`` at ``+0.120 s`` that bag ``2026-08-26_14-25-16``
produced twice with the hand committed.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[2]
_PROBE = _REPO / 'tools' / 'probes' / 'cadence_rung_check.py'


def _load_probe():
    """Import the probe by PATH — it lives outside any package, deliberately
    (``tools/probes/README.md``: probes are standalone scripts, not a library)."""
    spec = importlib.util.spec_from_file_location('cadence_rung_check', _PROBE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault('cadence_rung_check', mod)
    spec.loader.exec_module(mod)
    return mod


probe = _load_probe()


def _milestone_row(h_m):
    """``(flight, pipelined floor, milestone dwell)`` at one apex height —
    one row of the plan's § 2.7 table, from the probe."""
    T = probe.flight_for_height(h_m)
    return T, probe.pipelined_required_dwell(T), probe.milestone_dwell(T)


def test_accept_implies_flies_over_the_whole_grid():
    """THE contract. Empty list or the machine can abort with the hand
    committed."""
    violations = probe.grid_violations()
    assert violations == [], '\n'.join(
        'T={:.4f} dwell={:.4f} delay={:.4f} {} ilc_trim={}: {}'.format(
            T, dwell, delay, 'first' if aimed else 'chained', ilc, why)
        for T, dwell, delay, aimed, ilc, why in violations)


def test_every_published_rung_flies_chained_and_on_its_first_cycle():
    """The ladder is an operator runbook: an operator arms these numbers on a
    machine with a ball in the cup. Each rung is checked four ways — the session
    accept gate, the chained cycle (census-B1 skip), the first cycle (which
    commands the pre-positioning move and is granted the extra lead the node
    grants it), each with a layer-3 artifact loaded and without."""
    for name, h, dwell, delay in probe.LADDER:
        T = probe.flight_for_height(h)
        v = probe.ts.vertical_event_vel_mps(T)
        granted = max(float(delay),
                      probe.ts.min_throw_delay_for_release_s(v, True))
        for ilc in (False, True):
            assert probe.session_accepts(T, dwell, delay, ilc_trim=ilc) is None, (
                '{} rejected at the session gate (ilc_trim={})'.format(name, ilc))
        assert probe.run_cycle(T, delay, aimed=False)[0] == 'FLIES', (
            '{} does not fly on a chained cycle'.format(name))
        assert probe.run_cycle(T, granted, aimed=True)[0] == 'FLIES', (
            '{} does not fly on its first cycle'.format(name))


def test_the_pre_audit_ladder_still_reproduces_its_failure():
    """The 2026-08-22 finding, kept findable.

    Three rungs of the 78daf4b ladder could not throw a ball. What CHANGED on
    2026-08-23 is where they die: the accept-time floor now models the whole
    pre-dispatch sequence, so they are refused BEFORE anything is armed
    (``REJECTED_*``, nothing moved) instead of aborting at ``cycle_start +
    0.06 s`` with the catch latch up. That is the fix, and a test that only
    asserted "they still fail" would not see the difference."""
    failures = 0
    for name, h, dwell, delay in probe.LADDER_PRE_AUDIT:
        T = probe.flight_for_height(h)
        session = probe.session_accepts(T, dwell, delay)
        first = probe.run_cycle(T, delay, aimed=True)[0]
        if session is not None or first != 'FLIES':
            failures += 1
            # Whatever refuses it must refuse it EARLY.
            assert first == 'FLIES' or first.startswith('REJECTED_'), (
                '{} still dies mid-sequence: {}'.format(name, first))
    assert failures >= 3, (
        'the pre-audit ladder no longer reproduces its own finding')


def test_the_probe_reports_the_frontier_the_runbook_publishes():
    """The runbook's § 2.0 frontier table and this probe must agree, because the
    operator reads one and the gate runs the other.

    Pinned at the band FLOOR, which is where the frontier lives (it is monotone
    across the band). The two columns MEET there: the throw envelope refuses a
    negative speed trim at ``T = 0.4949``, so a loaded ILC artifact costs
    nothing at the fastest point."""
    T = float(probe.FLIGHT_TIME_MIN_S)
    disarmed = probe.fastest_at(T, ilc_trim=False)
    armed = probe.fastest_at(T, ilc_trim=True)
    # 54.3 / 0.4168 / 0.6101 until 2026-08-26. Owner decision D3 charges the
    # pre-dispatch sequence in the loop's measured PERIOD instead of its sleep
    # (+0.080 s on every delay floor), which costs 6.8 % of the frontier — the
    # cadence the old floor advertised was one the machine could not make, and
    # bag 2026-08-26_14-25-16 aborted two cycles proving it. The way back is a
    # cheaper tick, not a smaller floor.
    assert 60.0 / disarmed[2] == pytest.approx(50.6, abs=0.1)
    assert 60.0 / armed[2] == pytest.approx(50.6, abs=0.1)
    assert disarmed[0] == pytest.approx(0.4968, abs=5e-4)
    assert disarmed[1] == pytest.approx(0.6901, abs=5e-4)


# ── The PIPELINED model (plan B § 2.7 / § 6.2) ───────────────────────────────


def test_the_commit_budget_is_one_loop_period_not_four():
    """THE arithmetic the whole plan rests on.

    ``pre_dispatch_budget_s`` charges the chained preamble ``(1 + 3) x
    NODE_LOOP_PERIOD_S``; the pipeline moves three of those four ticks into the
    PREVIOUS cycle's flight and satisfies the armed->announce gap with the
    session-scoped latch (invariant S6) instead of with a tick. So the commit
    budget must be **exactly three loop periods smaller** than the serial delay
    floor — not "about", and not by a hand-typed 0.120."""
    for h in probe.MILESTONE_HEIGHTS:
        T = probe.flight_for_height(h)
        v = probe.ts.vertical_event_vel_mps(T)
        dispatch = probe.hand_stroke.min_throw_event_delay_s(v)
        assert probe.commit_budget_s(v) == pytest.approx(
            dispatch + probe.NODE_LOOP_PERIOD_S
            + probe.FLOOR_REPRESENTATION_SLACK_S, abs=1e-12)
        serial = probe.ts.min_throw_delay_for_release_s(v, False)
        assert serial - probe.commit_budget_s(v) == pytest.approx(
            3.0 * probe.NODE_LOOP_PERIOD_S, abs=1e-9)


def test_the_pipelined_floor_table_reproduces_the_plans_section_2_7():
    """§ 2.7's table, row for row, from the shipped session properties plus the
    one modelled term. The two milestone rows are the ones the plan is FOR:
    ``h = 1.0`` clears by 17.9 ms and ``h = 1.3`` by 101.8 ms, and the two rows
    below the milestone band do NOT clear — which is § 7's whole reason for
    putting ``h = 0.5`` out of scope rather than pretending it is reachable."""
    expected = {0.50: (0.4941, 0.3075), 0.80: (0.4390, 0.3890),
                1.00: (0.4170, 0.4349), 1.30: (0.3941, 0.4958)}
    for h, (floor, milestone) in expected.items():
        _T, got_floor, got_milestone = _milestone_row(h)
        assert got_floor == pytest.approx(floor, abs=5e-4), h
        assert got_milestone == pytest.approx(milestone, abs=5e-4), h
    assert _milestone_row(1.00)[2] - _milestone_row(1.00)[1] == pytest.approx(
        0.0179, abs=5e-4)
    assert _milestone_row(1.30)[2] - _milestone_row(1.30)[1] == pytest.approx(
        0.1018, abs=5e-4)
    for h in (0.50, 0.80):
        _T, floor, milestone = _milestone_row(h)
        assert milestone < floor, (
            'h={} is out of scope precisely because it does not clear'.format(h))


def test_the_pipelined_floor_is_below_the_serial_one_at_every_milestone_height():
    """The point of the exercise, stated as an inequality rather than as two
    tables a reader has to subtract by eye. The gap is the three loop periods
    the preamble stops spending on the critical path."""
    for h in probe.MILESTONE_HEIGHTS:
        T = probe.flight_for_height(h)
        serial = probe.required_dwell(T, probe.session_floor_delay(T))
        assert probe.pipelined_required_dwell(T) < serial, h


def test_every_section_6_2_rung_is_admitted_and_commits():
    """The operator runbook's pipelined rungs, checked the way the published
    ladder is: the session accept gate, then the modelled commit. P5's 13 ms of
    clearance is the razor edge the runbook flags — it is asserted POSITIVE
    here, so a floor that grows past it fails loudly instead of shipping."""
    for name, h, dwell, delay in probe.PIPELINED_LADDER:
        T = probe.flight_for_height(h)
        assert delay == 0.0, (
            '{}: the pipelined release is absolute; throw_delay is not the '
            'cadence lever'.format(name))
        assert probe.pipelined_session_accepts(T, dwell) is None, name
        verdict, _slip, _at, _rel = probe.commit_tick(T, dwell)
        assert verdict == 'FLIES', '{} -> {}'.format(name, verdict)
        assert dwell > probe.pipelined_required_dwell(T), name


def test_the_predicted_commit_slip_matches_the_plans_section_1_4():
    """§ 1.4's prediction, tested rather than asserted: the seat edge costs
    NOTHING at ``h = 1.3`` (the commit instant is already past it) and ~45/65 ms
    at ``h = 1.0``. This is the number rung PIPE-1 of the hardware ladder scores
    against, so it has to come from the same expression the runbook quotes."""
    seat = probe.MEASURED_SEAT_EDGE_MEDIAN_S
    slips = {}
    for name, h, dwell, _delay in probe.PIPELINED_LADDER:
        T = probe.flight_for_height(h)
        slips[name] = probe.commit_tick(T, dwell, seat_edge_s=seat)[1]
    for name in ('P0', 'P1', 'P2', 'P3'):
        assert slips[name] == pytest.approx(0.0, abs=1e-9), name
    assert slips['P4'] == pytest.approx(0.0446, abs=5e-4)
    assert slips['P5'] == pytest.approx(0.0646, abs=5e-4)


def test_pipelined_accept_implies_flies_over_the_whole_grid():
    """THE contract, pipelined. ``ABORTED_CANT_MAKE_RELEASE`` costs the same
    under the pipeline as it does today — latch up, announcement out, hand
    committed — so no dwell the session accepts may reach it, and none may force
    a slip on the park gate the floor exists to cover."""
    violations = probe.pipelined_grid_violations()
    assert violations == [], '\n'.join(
        'T={:.4f} dwell={:.4f} ilc_trim={}: {}'.format(T, dwell, ilc, why)
        for T, dwell, ilc, why in violations)


def test_the_pipelined_grid_reds_when_the_commit_budget_forgets_the_polled_tick():
    """The pipelined half's own regression case, and it is the reason the
    contract is worth a test at all.

    The commit tick is POLLED: the iteration crossing ``commit_at`` may be a
    full loop period late, and that lateness comes straight off the lead the
    release-window guard measures. Charge zero for it — machine still polling,
    budget pretending it does not — and the grid finds hundreds of dwells the
    session accepts and the commit then aborts. A probe that cannot show that
    has lost the finding, exactly as ``LADDER_PRE_AUDIT`` would."""
    counter = probe.pipelined_grid_violations(budget_loop_s=0.0)
    assert len(counter) > 100, (
        'the +1 loop period in commit_budget_s is no longer load-bearing — '
        'either the model changed or the counter-check stopped biting')
    assert all('ABORTED_CANT_MAKE_RELEASE' in why for _T, _d, _i, why in counter)


def test_a_loop_period_of_zero_is_refused_rather_than_looping_forever():
    """``commit_tick`` polls; a period of zero is a machine whose clock never
    advances, and the model must say so instead of spinning. (It did spin, once,
    2026-08-27 — the counter-check above was first written as ``loop_period_s=0``
    and hung the probe.)"""
    T = probe.flight_for_height(1.00)
    with pytest.raises(ValueError):
        probe.commit_tick(T, 0.45, loop_period_s=0.0)


def test_the_pipelined_frontier_is_faster_than_the_serial_one():
    """The cadence the pipeline buys, at the band floor where the frontier
    lives. 50.6 -> 56.3 throws/min is the three returned loop periods showing up
    as an operator-visible number."""
    T = float(probe.FLIGHT_TIME_MIN_S)
    serial_period = probe.fastest_at(T, ilc_trim=False)[2]
    dwell, period = probe.fastest_pipelined_at(T, ilc_trim=False)
    assert 60.0 / serial_period == pytest.approx(50.6, abs=0.1)
    assert 60.0 / period == pytest.approx(56.3, abs=0.1)
    assert dwell == pytest.approx(0.5701, abs=5e-4)


# ── B4's probe reconciliation (2026-08-27) ───────────────────────────────────


def test_the_probe_imports_the_shipped_commit_budget_rather_than_modelling_it():
    """**THE reconciliation this module's header demanded**, and the obligation
    ``commit_budget_s.__doc__`` carried through Phase B0.

    The probe MODELLED ``commit_budget_s`` while it did not exist in
    ``ros_ws/``; B4 landed ``toss_sequencer.commit_budget_s`` and the probe now
    forwards to it. Pinned as an IDENTITY, not an approximation, because the
    defect class this closes is a probe and a shipped floor drifting apart —
    which is the 2026-08-22 audit's finding (the session's mirror and the
    cycle's gate were two expressions of one floor) wearing a different hat.

    Asserted three ways so a re-fork cannot slip through any of them: the same
    float at every milestone speed, the same object identity through
    ``__wrapped__``-free forwarding, and the same behaviour under the override
    and loop-period arguments the sweeps drive it with."""
    from jugglebot import toss_sequencer as ts
    assert probe.commit_budget_s is not ts.commit_budget_s, (
        'the probe keeps a thin wrapper so its docstring can carry the history')
    for h in probe.MILESTONE_HEIGHTS:
        T = probe.flight_for_height(h)
        v = probe.ts.vertical_event_vel_mps(T)
        for loop in (ts.NODE_LOOP_PERIOD_S, 0.025, 0.070):
            assert probe.commit_budget_s(v, loop_period_s=loop) == (
                ts.commit_budget_s(v, loop_period_s=loop))
        assert probe.commit_budget_s(v, 0.35) == ts.commit_budget_s(v, 0.35)


def test_the_probes_pipelined_floor_is_the_shipped_required_dwell():
    """The other half of the reconciliation: the probe's ``floor`` column and a
    ``pipelined=True`` session's ``required_dwell_s`` are ONE number.

    The forwarded ``commit_budget_s`` closes the BUDGET; this closes the FLOOR
    built on it. Without it the probe could still print a § 2.7 table the accept
    gate does not enforce — which is precisely how three published cadence rungs
    shipped."""
    for h in (0.50, 0.80, 1.00, 1.30):
        T = probe.flight_for_height(h)
        for ilc in (False, True):
            terms = probe.pipelined_terms(T, ilc_trim=ilc)
            assert terms['floor'] == pytest.approx(terms['shipped_floor'],
                                                   abs=1e-12), (h, ilc)


def test_the_pipelined_floor_table_reproduces_from_the_shipped_gate():
    """§ 2.7's table, from the SHIPPED ``required_dwell_s`` rather than from the
    probe's own expression — B4's acceptance, stated as the plan states it.

    These are the same four rows ``test_the_pipelined_floor_table_reproduces_
    the_plans_section_2_7`` asserts off the model; asserting them again off the
    gate is what makes "the model and the machine agree" a fact rather than an
    intention."""
    from jugglebot.toss_session import TossSessionSequencer
    expected = {0.50: 0.4941, 0.80: 0.4390, 1.00: 0.4170, 1.30: 0.3941}
    for h, floor in expected.items():
        T = probe.flight_for_height(h)
        session = TossSessionSequencer(num_throws=5, dwell_time_s=9.0,
                                       throw_delay_s=5.0, flight_time_s=T,
                                       ilc_speed_trim_possible=False,
                                       pipelined=True)
        assert session.required_dwell_s == pytest.approx(floor, abs=5e-4), h


def test_the_flag_false_decision_stream_is_the_pre_b4_one_over_the_whole_grid():
    """**T-U13 / T-G1 — the acceptance the flag ships on.**

    With ``toss_pipeline_enabled`` false the SERIAL grid must be exactly what it
    was before B4: the probe drives the REAL ``TossSequencer`` at the real loop
    period over ~1500 grid points, and every one of them must reach the same
    verdict. The new FSM fields (``staged``, ``action_then``, ``slip``, the two
    new phases) are inert at their defaults, so "identical" here is a statement
    about the whole decision stream and not only about the terminal.

    This is the rollback § 9.5 level 1 promises, made checkable: one YAML key
    plus a colcon build, and the machine is the one the ladder validated."""
    assert probe.grid_violations() == []
    # …and the SERIAL ladder still flies, four ways, exactly as before.
    for name, h, dwell, delay in probe.LADDER:
        T = probe.flight_for_height(h)
        v = probe.ts.vertical_event_vel_mps(T)
        granted = max(float(delay),
                      probe.ts.min_throw_delay_for_release_s(v, True))
        for ilc in (False, True):
            assert probe.session_accepts(T, dwell, delay,
                                         ilc_trim=ilc) is None, name
        assert probe.run_cycle(T, delay, aimed=False)[0] == 'FLIES', name
        assert probe.run_cycle(T, granted, aimed=True)[0] == 'FLIES', name
    # …while the pre-audit ladder still REDS: the regression stays findable, so
    # a green grid is evidence rather than an absence of coverage.
    reds = sum(1 for name, h, dwell, delay in probe.LADDER_PRE_AUDIT
               if (probe.session_accepts(probe.flight_for_height(h), dwell,
                                         delay) is not None
                   or probe.run_cycle(probe.flight_for_height(h), delay,
                                      aimed=True)[0] != 'FLIES'))
    assert reds >= 3


def test_no_serial_decision_moves_when_a_cycle_is_merely_capable_of_staging():
    """The other direction of the same acceptance, at the FSM: constructing a
    cycle with the pipeline's new fields at their defaults produces the
    identical decision stream to one constructed without them at all.

    It matters because ``staged`` has a ``__post_init__`` belt that can force
    itself False, and a belt that ALSO perturbed the serial path would make the
    flag a behaviour change rather than a switch."""
    from jugglebot.toss_sequencer import TossSequencer, TossObservations
    from jugglebot import toss_sequencer as ts

    def stream(**kw):
        seq = TossSequencer(catch_pose_stow_mm=(0.0, 0.0, 170.0),
                            flight_time_s=0.8, throw_delay_s=5.0,
                            positioning_move_expected=False, **kw)
        seq.start(0.0)
        out = []
        t = 0.0
        for _ in range(10):
            obs = TossObservations(
                now=t, control_mode=ts.TOSS_CONTROL_MODE, streaming=True,
                mocap_fresh=True, platform_levelled=True, hand_fresh=True,
                hand_parked=True, ball_seated=True, ball_evidence='SEATED',
                platform_at_target=True)
            d = seq.step(t, obs)
            out.append((round(t, 6), d.phase, d.action, d.action_then, d.slip,
                        d.done))
            if d.action == ts.ACTION_POSITION_PLATFORM:
                seq.note_position_noop(t)
            elif d.action == ts.ACTION_PREPARE_CATCH:
                seq.note_prepare_result(True)
            elif d.action == ts.ACTION_ANNOUNCE:
                seq.note_announcement()
            t += ts.NODE_LOOP_PERIOD_S
        return out

    assert stream() == stream(staged=False)
