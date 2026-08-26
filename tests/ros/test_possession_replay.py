"""Production-faithful offline replay of bag ``2026-08-26_14-25-16`` through the
SENSOR-ONLY possession verdict — the acceptance for owner decision **D1**
(2026-08-26).

WHY A REPLAY AND NOT A UNIT TEST. The unit surface is already covered
(``test_ball_possession.py``); what a unit test cannot show is whether the new
verdict, driven by a real cup stream at the FSM's own deadline and under the real
cadence clamps, reproduces what a human watching the sitting counted. It has to
reproduce it EXACTLY — 23 CAUGHT / 4 MISSED — because the cup called 31/31 by eye
on that sitting (these 27 adjudicated cycles plus four that never put a ball up:
2x ``ABORTED_CANT_MAKE_RELEASE``, 2x ``REJECTED_NO_BALL``), and a rule that is
"mostly right" about possession is a rule that mints phantom reloads.

WHAT IS PRODUCTION HERE, and what is not:

  * PRODUCTION — ``HandBallSensorSource`` (constructed from the generated config
    exactly as the node constructs it), ``merge_possession``, ``arrival_blind``,
    ``arrival_boundary_t``, ``CATCH_CONFIRM_WINDOW_S``, and the schedule clamps
    ``reload_coordinator_node._set_toss_next_cycle_perf`` latches;
  * FIXTURE — the cup stream and the per-cycle schedule, cut from the bag by
    ``tools/probes/possession_replay.py --emit-fixture``;
  * THIS FILE — only the assertions.

The bag is machine-local and gitignored; ``tests/ros/toss_verdict_replay_fixtures.py``
is what carries the evidence across a fresh clone.

Contract: ``ros_ws/docs/ball_possession_contract.md`` (C-POSSESS-1 § 3.2).
Entry: ``logbook/2026-08-26-possession-verdicts-become-sensor-only.md``.
"""

from __future__ import annotations

import importlib.util
import os

import pytest

from jugglebot.toss_session import TossSessionSequencer
from tests.ros import toss_verdict_replay_fixtures as fx

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _probe():
    """Import the emitting probe as a library. The replay itself lives THERE, not
    here, so the numbers this file asserts and the numbers the probe prints at the
    bench can never be two different computations — the same reason
    ``possession_verdict_bag_check.py`` owns its own scoring."""
    spec = importlib.util.spec_from_file_location(
        'possession_replay',
        os.path.join(_REPO, 'tools', 'probes', 'possession_replay.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(scope='module')
def replayed():
    """-> ``{(run, cycle): (verdict, blind, catch_dt)}`` for every fixture row."""
    probe = _probe()
    out = {}
    for row in fx.CYCLES:
        stream = probe.expand_stream(row['segments'], row['step_s'])
        out[(row['run'], row['cycle_index'])] = probe.replay_cycle(row, stream)
    return out


#: The rows the CUP adjudicated — i.e. a ball flew and the cup said something
#: about it. The other four rows never reach a possession verdict in the FSM at
#: all (two ABORTED_CANT_MAKE_RELEASE, two REJECTED_NO_BALL, all four minted
#: before BALL_IN_FLIGHT), and they are asserted separately below.
def _adjudicated():
    return [r for r in fx.CYCLES if r['sensor_label'] in ('CAUGHT', 'MISSED')]


def test_the_fixture_is_the_sitting_we_think_it_is():
    """Premise guard. Every assertion below is about one sitting; if the fixture
    is regenerated from a different bag the counts would move silently."""
    assert fx.REFERENCE_BAG == '2026-08-26_14-25-16'
    assert len(fx.CYCLES) == 31
    assert len(_adjudicated()) == 27


def test_the_replayed_census_is_the_cup_census_exactly(replayed):
    """**THE D1 ACCEPTANCE.** 23 CAUGHT / 4 MISSED, against the shipped
    tracker-primary code's 11 / 16.

    Not "close to" and not "at least": every one of the 27 adjudicated cycles must
    land on the cup's own label. The cup called 31/31 outcomes correctly on this
    sitting by the operator's eye (these 27 plus four unadjudicated no-release /
    no-ball rows), so any disagreement here is the new rule being wrong, not the
    ground truth being noisy."""
    verdicts = {(r['run'], r['cycle_index']): replayed[(r['run'],
                                                        r['cycle_index'])][0]
                for r in _adjudicated()}
    caught = sum(1 for v in verdicts.values() if v == 'CAUGHT')
    missed = sum(1 for v in verdicts.values() if v.startswith('MISSED'))
    assert (caught, missed) == (23, 4)
    for row in _adjudicated():
        key = (row['run'], row['cycle_index'])
        assert verdicts[key].startswith(row['sensor_label']), key
    # …and the shipped code's own census, for the delta this test exists to show.
    shipped_caught = sum(1 for r in _adjudicated()
                         if r['fsm_outcome'] == 'CAUGHT')
    shipped_missed = sum(1 for r in _adjudicated()
                         if r['fsm_outcome'].startswith('MISSED'))
    assert (shipped_caught, shipped_missed) == (11, 16)


def test_every_false_missed_flips_to_caught(replayed):
    """The 15 genuine catches the tracker-primary path threw away.

    Twelve of them carried no confirmed tracker track at all — the tracker vetoed
    by SILENCE, because the FSMs only asked the possession question on a tracker
    ``CAUGHT``. Three had a tracker CAUGHT that arrived 0.615-0.830 s past the
    scheduled landing, i.e. past the 0.560 s confirm window. Every one of the 15
    has a cup arrival edge inside that window."""
    flips = [r for r in _adjudicated()
             if r['sensor_label'] == 'CAUGHT'
             and r['fsm_outcome'].startswith('MISSED')]
    assert len(flips) == 15
    edges = []
    for row in flips:
        verdict, blind, dt = replayed[(row['run'], row['cycle_index'])]
        assert verdict == 'CAUGHT', (row['run'], row['cycle_index'])
        assert blind is False
        assert dt is not None
        edges.append(dt)
    # The reason the confirm window did NOT need to shrink (the pre-registered
    # H1a this investigation refuted): the cup edges sit at +0.143..+0.303 s
    # against a 0.560 s ceiling. The budget was never being spent on the sensor
    # band — it was being spent on tracker latency.
    from jugglebot.toss_sequencer import CATCH_CONFIRM_WINDOW_S
    assert min(edges) == pytest.approx(0.143, abs=2e-3)
    assert max(edges) == pytest.approx(0.303, abs=2e-3)
    assert max(edges) < 0.55 * CATCH_CONFIRM_WINDOW_S


def test_every_false_caught_flips_to_missed(replayed):
    """The 3 catches the tracker MINTED over an empty cup. One of them drove a
    phantom reload — the machine asked BallButler to throw a ball at a cup it had
    just wrongly recorded as loaded."""
    flips = [r for r in _adjudicated()
             if r['sensor_label'] == 'MISSED' and r['fsm_outcome'] == 'CAUGHT']
    assert len(flips) == 3
    for row in flips:
        verdict, blind, dt = replayed[(row['run'], row['cycle_index'])]
        assert verdict == 'MISSED'
        # A positive OBSERVATION of non-arrival, not blindness: the distinction
        # decides whether the terminal names the throw or the sensor.
        assert blind is False
        assert dt is None


def test_the_one_genuine_drop_stays_missed(replayed):
    """Run 5 cycle 3 — the sitting's only cycle where the ball genuinely did not
    reach the cup AND the shipped code agreed. A rule that flipped everything
    would pass the two tests above and fail this one, which is why it is asserted
    on its own."""
    row = next(r for r in fx.CYCLES if (r['run'], r['cycle_index']) == (5, 3))
    assert row['sensor_label'] == 'MISSED'
    assert row['fsm_outcome'] == 'MISSED'
    verdict, blind, dt = replayed[(5, 3)]
    assert verdict == 'MISSED'
    assert blind is False
    assert dt is None


def test_the_four_unadjudicated_rows_never_reach_a_verdict():
    """The four rows the cup did not label, stated so a future reader does not
    read them as replay failures.

    Two are ``ABORTED_CANT_MAKE_RELEASE`` (the D3 aborts — the ball never left the
    cup) and two are ``REJECTED_NO_BALL``. All four terminalise BEFORE
    ``BALL_IN_FLIGHT``, and ``ball_caught`` is read only in ``_step_in_flight``
    and ``_step_settling``, so no possession verdict is ever consulted for them.

    That matters more than it looks: run 12 cycle 2 replays CAUGHT, off an
    operator hand-reload 0.92 s after a landing that never happened. If a future
    edit ever moved the possession read earlier — into CHECKING, say — that row is
    the one that would mint a catch out of a reload."""
    unadjudicated = [r for r in fx.CYCLES
                     if r['sensor_label'] not in ('CAUGHT', 'MISSED')]
    assert len(unadjudicated) == 4
    assert sorted(r['fsm_outcome'] for r in unadjudicated) == [
        'ABORTED_CANT_MAKE_RELEASE', 'ABORTED_CANT_MAKE_RELEASE',
        'REJECTED_NO_BALL', 'REJECTED_NO_BALL']
    from jugglebot.toss_sequencer import (
        PHASE_BALL_IN_FLIGHT, PHASE_CATCHING, PHASE_SETTLING)
    import inspect
    from jugglebot import toss_sequencer
    src = inspect.getsource(toss_sequencer.TossSequencer)
    # The read sites, pinned structurally: `obs.ball_caught` may appear only in
    # the two in-flight/settling handlers.
    for handler in ('_step_checking', '_step_positioning', '_step_preparing',
                    '_step_throwing'):
        body = src.split('def {}('.format(handler))[1].split('\n    def ')[0]
        assert 'ball_caught' not in body, handler
    assert PHASE_BALL_IN_FLIGHT and PHASE_CATCHING and PHASE_SETTLING


# ── The schedule arithmetic the corrected verdicts restore ────────────────────

def _release_spacing(dwell_s, throw_delay_s, flight_s, caught):
    """Drive the PRODUCTION session FSM through two cycles and return the
    release-to-release spacing it schedules.

    The session is what owns the cadence: ``note_cycle_result`` sets
    ``_next_cycle_at``, the node adds ``throw_delay_s`` to reach the release, and
    a not-successful result additionally floors the start at ``landing +
    miss_cleanup_s``. Driving the real FSM rather than restating that arithmetic
    is the point — the bug this test is about was a verdict feeding the arithmetic,
    not the arithmetic itself."""
    probe = TossSessionSequencer(num_throws=5, dwell_time_s=dwell_s,
                                 throw_delay_s=throw_delay_s,
                                 flight_time_s=flight_s, catch_vel_scale=0.9)
    probe.start(0.0)
    # The sitting's own delay where it is still legal, the LIVE floor where it is
    # not. Run 9's rung asked for 0.45 s and the D3 floor is 0.4575 s at this
    # flight, so that rung needs 7.5 ms more lead than it was given — and the
    # spacing this function measures is `flight + dwell`, which does not depend on
    # the delay at all, so raising it to the floor changes nothing this test
    # asserts. Stated rather than hidden: it is the one published rung D3 moves.
    throw_delay_s = max(float(throw_delay_s), probe.min_throw_delay_s)
    # stop_on_miss FALSE so the MISSED arm reaches the scheduling hook at all —
    # with the shipped default the session simply stops, which is a different
    # (and correct) behaviour, not the cadence this measures.
    s = TossSessionSequencer(num_throws=5, dwell_time_s=dwell_s,
                             throw_delay_s=throw_delay_s, stop_on_miss=False,
                             flight_time_s=flight_s, catch_vel_scale=0.9)
    s.start(0.0)
    from jugglebot.toss_sequencer import TossResult
    d = s.step(0.0)
    assert d.action == 'start_cycle', d
    t_release = 0.0 + throw_delay_s
    landing = t_release + flight_s
    result = (TossResult(True, 'CAUGHT', 4.0, flight_s) if caught
              else TossResult(False, 'MISSED', float('nan'), float('nan')))
    s.note_cycle_result(result, t_release, landing)
    return (s.next_cycle_at + throw_delay_s) - t_release


@pytest.mark.parametrize('run,dwell,delay,want', [(7, 1.5, 0.9, 2.298),
                                                  (9, 0.65, 0.45, 1.448)])
def test_the_corrected_verdicts_restore_a_uniform_release_cadence(
        run, dwell, delay, want):
    """**The operator-visible half of the defect.** Every false MISSED also
    charged the next cycle ``DEFAULT_SESSION_MISS_CLEANUP_S``, so the sitting's
    spacing was not merely mislabelled — it was physically irregular.

    Measured release-to-release spacing in the bag:

      run 7 (dwell 1.5)   4.310 / 2.303 / 2.335 / 4.319 s
      run 9 (dwell 0.65)  3.861 / 3.868 / 3.853 / 3.866 s   (every cycle missed)

    With the corrected verdicts every cycle is CAUGHT and the spacing collapses to
    ``flight + dwell``, uniform: **2.298 s** and **1.448 s**. The MISSED spacing is
    asserted too, so this pins the mechanism (the cleanup floor) and not just the
    happy number."""
    flight = fx.CYCLES[[c['run'] for c in fx.CYCLES].index(run)]['flight_time_s']
    caught = _release_spacing(dwell, delay, flight, caught=True)
    assert caught == pytest.approx(want, abs=1e-3)
    assert caught == pytest.approx(flight + dwell, abs=1e-9)
    # The floor is what the false MISSEDs bought, and it is still there for a
    # REAL miss — 2.80 s past the landing since D3 (2.60 s on the sitting).
    from jugglebot.toss_session import DEFAULT_SESSION_MISS_CLEANUP_S
    s = TossSessionSequencer(num_throws=5, dwell_time_s=dwell,
                             throw_delay_s=delay, flight_time_s=flight,
                             catch_vel_scale=0.9)
    s.start(0.0)
    charged_delay = max(delay, s.min_throw_delay_s)
    missed = _release_spacing(dwell, delay, flight, caught=False)
    assert missed == pytest.approx(
        flight + DEFAULT_SESSION_MISS_CLEANUP_S + charged_delay, abs=1e-9)
    assert missed > caught
