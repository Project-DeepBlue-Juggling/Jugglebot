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
    exactly as ``tools/probes/possession_replay.py`` constructs it), the module's
    own ``merge_possession``, ``arrival_blind``, ``arrival_boundary_t`` and
    ``ARRIVAL_BAND_MAX_S`` (C-POSSESS-1, INVARIANTS.md § 5);
  * FIXTURE — the cup stream and the per-cycle schedule, cut from the bag by
    ``tools/probes/possession_replay.py --emit-fixture``;
  * THIS FILE — only the assertions.

The bag is machine-local and gitignored; ``tests/ros/toss_verdict_replay_fixtures.py``
is what carries the evidence across a fresh clone.

Contract: ``ros_ws/docs/ball_possession_contract.md`` (C-POSSESS-1 § 3.2).
Entry: ``logbook/2026-08-26-possession-verdicts-become-sensor-only.md``.

R4 NOTE (2026-09-24, U6b Cluster A follow-up, `census_fsm_deletion.md`): this
file used to also pin the FSM's OWN cadence arithmetic (``TossSessionSequencer``
release spacing, the B4 pipelined-verdict clamp) and a source-introspection
check on ``TossSequencer``'s phase handlers. All of that died with the FSM
(``toss_sequencer.py`` / ``toss_session.py`` deleted under `fsm-final`) and had
no live analogue to port to — the skill stack's schedule is absolute wall-clock
with no cadence clamp and no phase-handler structure to introspect (plan § 0,
`REJECTED_RELEASE_SCHEDULE` / `REJECTED_CANT_MAKE_LEAD` rows, INVARIANTS.md
§ 8). What survives below is the part that pins ``ball_possession``'s own
sensor-only verdict logic against the recorded bag — a live claim, not an FSM
one.
"""

from __future__ import annotations

import importlib.util
import os

import pytest

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
    from jugglebot.ball_possession import ARRIVAL_BAND_MAX_S
    assert min(edges) == pytest.approx(0.143, abs=2e-3)
    assert max(edges) == pytest.approx(0.303, abs=2e-3)
    assert max(edges) < 0.55 * ARRIVAL_BAND_MAX_S


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

    Two are ``ABORTED_CANT_MAKE_RELEASE`` and two are ``REJECTED_NO_BALL`` — all
    four terminalise before the ball ever leaves the cup, so no possession
    verdict is ever consulted for them.

    That matters more than it looks: run 12 cycle 2 replays CAUGHT, off an
    operator hand-reload 0.92 s after a landing that never happened. If a future
    edit ever moved the possession read earlier than release, that row is the
    one that would mint a catch out of a reload.

    R4 NOTE (2026-09-24): this used to ALSO pin the read site structurally, via
    `inspect.getsource` on the now-deleted `toss_sequencer.TossSequencer`
    (`ball_caught` may appear only in `_step_in_flight`/`_step_settling`). No
    live analogue is driven here — the skill stack's outcome capture
    (`motion/skills/executor.py::SkillExecutor._advance_release_evidence` /
    `_consider_landing`) is release-gated by construction (INVARIANTS.md
    `ABORTED_NO_RELEASE` row), not by a phase-handler boundary to introspect —
    so this test keeps only the DATA claim (the four rows carry no verdict),
    which is a live fact about the fixture and the replay, not about FSM
    source."""
    unadjudicated = [r for r in fx.CYCLES
                     if r['sensor_label'] not in ('CAUGHT', 'MISSED')]
    assert len(unadjudicated) == 4
    assert sorted(r['fsm_outcome'] for r in unadjudicated) == [
        'ABORTED_CANT_MAKE_RELEASE', 'ABORTED_CANT_MAKE_RELEASE',
        'REJECTED_NO_BALL', 'REJECTED_NO_BALL']
