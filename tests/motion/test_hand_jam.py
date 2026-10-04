"""Hand-jam detector + recovery step machine (motion/hand_jam.py).

The invariant under test: a stalled hand at the clamp under a descending
command is relieved before anything else and never pushed through.

The detector is pinned against the committed 2026-10-02 fixture (decoded from
bag 2026-10-02_18-11-14; t is seconds relative to T = 1790929469.795, the
bridge's guard-latch report) and against one synthetic trace per row of the
design's false-positive table. The step machine is driven against a small
plant: a hand that TRAP_TRAJs toward its target and an optional ball that
stops a downward move (a pinch) and falls once the hand is raised far enough.
"""
from __future__ import annotations

import csv
import dataclasses
import math
import pathlib

import pytest

from jugglebot.motion.hand_jam import (
    HandSample, Intent, IntentKind, IntentResult, JamConfig, JamDetector,
    JamRecovery, Outcome, RecoveryObs, Verdict, planned_steps)

FIXTURE = pathlib.Path(__file__).parent / 'fixtures' / 'hand_jam_20261002.csv'


def _fixture_samples(limit=50.0):
    out = []
    with FIXTURE.open() as f:
        rows = [r for r in csv.reader(f) if r and not r[0].startswith('#')]
    for r in rows[1:]:
        t, pc, vf, pm, vm, iq, st, er, age = map(float, r)
        out.append(HandSample(t, pc, vf, pm, vm, iq, int(st), int(er), age, limit))
    return out


def _fires(det, samples):
    return [(s.t, v) for s in samples for v in [det.step(s)] if v is not Verdict.NONE]


def _s(t, pos_cmd, vel_ff, pos_meas, vel_meas, iq, state=8, err=0, age=0.005,
       limit=50.0):
    return HandSample(t, pos_cmd, vel_ff, pos_meas, vel_meas, iq, state, err, age, limit)


def _descent_into(stop_rev, *, dur=1.0, dt=0.01, start=6.0, v=-20.0, lag=0.15,
                  iq_stall=-50.0, state=8, err=0, age=0.005, release_after=None,
                  rest=0.307):
    """A streamed REST descent whose hand STOPS at ``stop_rev`` (ball, snag,
    frozen encoder...), with the firmware's 2.0 rev lead clamp on the sent
    command. ``release_after`` frees the hand that long after the stop."""
    out, t_stop = [], None
    for i in range(int(dur / dt)):
        t = i * dt
        raw = max(start + v * t, rest)
        free = max(raw + lag, rest)
        held = t_stop is not None and (release_after is None
                                       or t - t_stop < release_after)
        if free <= stop_rev and t_stop is None:
            t_stop, held = t, True
        meas = stop_rev if held else free
        vel_meas = 0.05 if held else (v if raw > rest else 0.0)
        iq = (iq_stall if held and t - t_stop >= 0.03 else -5.0)
        cmd = max(raw, meas - 2.0)
        out.append(_s(t, cmd, v if raw > rest else 0.0, meas, vel_meas, iq,
                      state, err, age))
    return out


def _hand_move_to_trace(target, *, start, v, stop=None, dur=1.0, dt=0.01,
                        moving_iq=-5.0, iq_stall=-50.0, state=8, err=0, age=0.01):
    """A HAND_MOVE_TO-shaped move (2026-10-04, D2): ``pos_cmd`` is the SINGLE
    target write the whole trace (never changes, unlike the streamed REST
    lane's ``_descent_into``), and ``vel_ff_cmd`` is always 0 (HAND_MOVE_TO
    carries no feedforward — the bridge writes it that way,
    ``teensy_bridge_node.teensy_hand_move_to``). The hand moves from
    ``start`` at velocity ``v`` and STOPS at ``stop`` if given (a ball,
    snag...) short of ``target`` — maintaining the lag ``target`` keeps
    below (or above) the stop, same as ``_descent_into``'s ``stop_rev`` vs.
    ``rest``; with no ``stop`` it just moves until the trace ends."""
    out, pos, stuck = [], start, False
    for i in range(int(dur / dt)):
        t = i * dt
        if not stuck:
            pos += v * dt
            if stop is not None and ((v < 0 and pos <= stop) or (v > 0 and pos >= stop)):
                pos, stuck = stop, True
        vel = 0.0 if stuck else v
        iq = iq_stall if stuck else moving_iq
        out.append(_s(t, target, 0.0, pos, vel, iq, state, err, age))
    return out


# ── the event fixture ────────────────────────────────────────────────────────

def test_fixture_fires_hand_jam_at_least_80ms_before_the_guard_latch():
    fires = _fires(JamDetector(JamConfig()), _fixture_samples())
    assert len(fires) == 1, fires
    t, v = fires[0]
    assert v is Verdict.HAND_JAM
    assert t <= -0.080, f'fired at T{t:+.3f} s — later than 80 ms before the latch'


def test_fixture_out_of_band_reports_hand_stall():
    # Same event with the band moved off the stall: P5 false -> HAND_STALL.
    cfg = JamConfig(band_rev=(3.7, 9.0))
    fires = _fires(JamDetector(cfg), _fixture_samples())
    assert [v for _, v in fires] == [Verdict.HAND_STALL]
    assert fires[0][0] <= -0.080


def test_fixture_with_the_draft_1rps_threshold_fires_too_late():
    # Why stall_vel_rps is 1.5: the -1.243 rev/s creep sample at T-0.148
    # resets a 1.0 threshold's sustain (see JamConfig's docstring).
    fires = _fires(JamDetector(JamConfig(stall_vel_rps=1.0)), _fixture_samples())
    assert fires and fires[0][0] > -0.080


def test_detector_fires_once_then_latches_until_reset():
    det = JamDetector(JamConfig())
    samples = _fixture_samples()
    assert len(_fires(det, samples + [
        _s(s.t + 1.0, s.pos_cmd, s.vel_ff_cmd, s.pos_meas, s.vel_meas, s.iq_meas)
        for s in samples])) == 1
    assert det.fired is Verdict.HAND_JAM
    det.reset()
    assert det.fired is Verdict.NONE and len(_fires(det, samples)) == 1


# ── synthetic: must fire ─────────────────────────────────────────────────────

def test_silent_stall_below_the_latch_threshold_fires():
    # Stopped at 2.5 rev: raw REST target 0.307 - 2.5 = -2.19 never reaches
    # the 2.5 rev guard, so the firmware never latches. The detector must.
    tr = _descent_into(2.5)
    assert min(s.pos_cmd - s.pos_meas for s in tr) > -2.5    # no guard latch
    assert [v for _, v in _fires(JamDetector(JamConfig()), tr)] == [Verdict.HAND_JAM]


def test_snag_and_fingers_fire_by_design():
    # A string snag / carriage bind / fingers look identical while descending:
    # the detector fires, and the RAISE discriminates (see the recovery tests).
    assert _fires(JamDetector(JamConfig()), _descent_into(3.0))


# ── synthetic: the false-positive table, none may fire ──────────────────────

def _never(trace, cfg=None):
    fires = _fires(JamDetector(cfg or JamConfig()), trace)
    assert fires == [], fires


def test_fp_normal_descent_and_catch():
    tr = []
    for i in range(100):
        t = i * 0.01
        cmd = max(6.0 - 20.0 * t, 0.307)
        moving = cmd > 0.307
        # a hard catch: the clamp is hit for 40 ms at the bottom, lag <= 0.2
        iq = -50.0 if 0.27 <= t < 0.31 else -5.0
        tr.append(_s(t, cmd, -20.0 if moving else 0.0, cmd + (0.18 if moving else 0.0),
                     -20.0 if moving else 0.0, iq))
    _never(tr)


@pytest.mark.parametrize('pos', [0.307, 0.0])
def test_fp_settled_at_rest_or_park(pos):
    _never([_s(i * 0.01, pos, 0.0, pos - 0.005, 0.02, -1.0) for i in range(200)])


def test_fp_top_stop_at_the_stroke_clip():
    tr = [_s(i * 0.01, 10.701, 5.0 if i < 20 else 0.0, 10.69, 0.0, 50.0)
          for i in range(200)]
    _never(tr)


def test_fp_encoder_freeze_with_odrive_errors():
    _never(_descent_into(3.0, err=0x200))


def test_fp_stale_diagnostic():
    # Above the new bound (1.5 s, 2026-10-04): a dead diagnostic feed is
    # still a false positive this must reject.
    _never(_descent_into(3.0, age=2.0))


def test_fires_with_a_diagnostic_age_inside_the_new_bound():
    # Below 1.5 s but above the design draft's 0.05 s: this is exactly the
    # gap the draft bound closed off (see test_fires_through_the_
    # one_hertz_diagnostic_cadence for the cadence that produces it live).
    assert [v for _, v in _fires(JamDetector(JamConfig()), _descent_into(3.0, age=1.0))
           ] == [Verdict.HAND_JAM]


@pytest.mark.parametrize('state,iq', [(1, 0.0), (8, 0.0)])
def test_fp_hand_odrive_fault_idle_or_no_current(state, iq):
    _never(_descent_into(3.0, state=state, iq_stall=iq))


def test_fp_undervoltage_lag_without_the_clamp():
    _never(_descent_into(3.0, iq_stall=-30.0))


def test_fp_ball_landing_on_the_cup_shorter_than_the_sustain():
    _never(_descent_into(3.0, release_after=0.12))   # clamp for 90 ms only


def test_fires_through_the_one_hertz_diagnostic_cadence():
    """D1 (2026-10-04, R5 sitting 4): the can-bridge firmware
    (``Teensy_code_canbridge/telemetry.cpp``, ``diag_changed`` /
    ``DIAG_FORCE_PERIOD_US`` = 1e6) sends axis 6's DIAGNOSTIC on-change or
    forced at 1 Hz, so in a steady clamp stall (iq constant -> no on-change
    send) the diagnostic age ramps 0 -> ~1 s every second instead of staying
    near zero. ``max_age_s`` 0.05 (the design draft) is then true for <= 50
    ms per second, so the 100 ms sustain can never complete; 1.5 s survives
    the whole cycle. Pins the defect class, not just the live symptom."""
    base = _descent_into(2.5, dur=3.0)
    t_stall = next(s.t for s in base if s.iq_meas == -50.0)
    tr = [dataclasses.replace(
              s, age_s=((s.t - t_stall) % 1.0) if s.t >= t_stall else s.age_s)
         for s in base]
    assert [v for _, v in _fires(JamDetector(JamConfig()), tr)] == [Verdict.HAND_JAM]
    assert _fires(JamDetector(JamConfig(max_age_s=0.05)), tr) == []


def test_fp_hand_move_to_descent_fires_by_measured_velocity():
    """D2 (2026-10-04): HAND_MOVE_TO writes the command cache ONCE (the
    target, 0.5 rev — well below the 2.5 rev stop, so the lag persists for
    as long as the hand is stuck); P1's measured-descent alternative is what
    lets the detector witness the rest of the move through the encoder
    instead."""
    tr = _hand_move_to_trace(0.5, start=5.0, v=-2.0, stop=2.5, dur=1.5)
    assert [v for _, v in _fires(JamDetector(JamConfig()), tr)] == [Verdict.HAND_JAM]


def test_fp_hand_move_to_ascent_into_a_stop_never_fires():
    # Same shape, opposite direction (target above the stop, so there is no
    # lag once stuck — P3 false — and vel_meas is never negative, so the
    # new alternative never opens a false-positive class for an ascent).
    _never(_hand_move_to_trace(4.0, start=0.5, v=2.0, stop=2.5, dur=1.5))


def test_fp_hand_move_to_soft_catch_moving_low_current_never_fires():
    # Still moving into the cup (vel_meas magnitude above stall_vel_rps) at
    # ordinary cruise current when the trace ends -- P2 stays false.
    _never(_hand_move_to_trace(2.5, start=5.0, v=-2.0, dur=0.3, moving_iq=-5.0))


def test_telemetry_gap_restarts_the_sustain():
    tr = _descent_into(2.5)
    t_fire = _fires(JamDetector(JamConfig()), tr)[0][0]
    # Drop every sample in a 60 ms window inside the sustain: no longer
    # continuous, so the fire moves later (it must not count the gap).
    gapped = [s for s in tr if not (t_fire - 0.08 < s.t < t_fire - 0.02)]
    t2 = _fires(JamDetector(JamConfig()), gapped)[0][0]
    assert t2 > t_fire


# ── config ───────────────────────────────────────────────────────────────────

@pytest.mark.parametrize('kw', [
    {'band_rev': [0.2, 3.6]},          # below park + the lower-stall margin
    {'band_rev': [3.0, 2.0]},
    {'band_rev': [1.0]},
    {'relief_curr_a': 2.0},            # below the carriage gravity hold
    {'relief_curr_a': 45.0},           # not a relief
    {'sustain_s': 0.5},                # >= the P1 descent window
    {'iq_frac': 1.5},
    {'raise_rev': [2.5, 1.0]},
    {'dwell_s': 0.05},
    {'stall_vel_rps': float('nan')},
    {'enabled': 'yes'},
    {'no_such': 1.0},
])
def test_config_rejects(kw):
    with pytest.raises(ValueError):
        JamConfig.from_values(**kw)


def test_config_defaults_validate_and_accept_ros_lists():
    cfg = JamConfig.from_values(band_rev=[1.0, 3.6], raise_rev=[1.0, 2.5], enabled=True)
    assert cfg.band_rev == (1.0, 3.6) and cfg.raise_rev == (1.0, 2.5)
    JamConfig().validate()


# ── recovery step machine ────────────────────────────────────────────────────

class _Plant:
    """Axis 6 under HAND_MOVE_TO: constant-velocity TRAP to the target.
    ``ball`` is the pinch height; the ball falls once the hand has been at
    or above ``ball + clear`` (``clear=inf``: wedged). ``snag`` blocks UP."""

    def __init__(self, pos, *, ball=None, clear=0.5, snag=False, unknown=False):
        self.pos, self.vel = pos, 0.0
        self.ball, self.clear, self.snag, self.unknown = ball, clear, snag, unknown
        self.target = None
        self.done, self.ok, self.status = False, False, ''

    def start(self, target, vel):
        if self.unknown:
            self.done, self.ok, self.status = True, False, 'ERR_UNKNOWN_METHOD'
            return
        self.target, self.v = target, vel
        self.done, self.ok, self.status = False, False, ''

    def tick(self, dt):
        if self.target is None or self.done:
            self.vel = 0.0
            return
        d = self.target - self.pos
        step = math.copysign(min(abs(d), self.v * dt), d)
        if step > 0 and self.snag:
            step = 0.0
        if self.ball is not None and step < 0 and self.pos + step < self.ball:
            step = min(0.0, self.ball - self.pos)
        self.pos += step
        self.vel = step / dt
        if self.ball is not None and self.pos >= self.ball + self.clear:
            self.ball = None                       # it fell off the lip
        if abs(self.target - self.pos) < 1e-9:
            self.done, self.ok, self.status = True, True, 'OK'


def _drive(rec, plant, *, latched=True, armed=True, fail=(), bb_until=0.0,
           t_end=30.0, dt=0.01, nan_after=None):
    """Run ``rec`` to its END against ``plant``; return the executed intents
    (with the time each ran)."""
    st = {'latched': latched, 'armed': armed, 'restore_fail': 'restore' in fail}
    log, t = [], 0.0
    while t < t_end:
        pos = math.nan if (nan_after is not None and t >= nan_after) else plant.pos
        obs = RecoveryObs(t=t, pos=pos, vel=plant.vel, latched=st['latched'],
                          armed=st['armed'], bb_pending=t < bb_until,
                          move_done=plant.done, move_ok=plant.ok,
                          move_status=plant.status)
        it = rec.next(obs)
        if it.kind is IntentKind.END:
            break
        if it.kind is IntentKind.WAIT:
            plant.tick(dt)
            t += dt
            continue
        log.append((t, it))
        ok = True
        if it.kind is IntentKind.SET_CURRENT:
            ok = not (it.purpose in fail or (it.purpose == 'restore' and st['restore_fail']))
        elif it.kind is IntentKind.CONVERGE_CLEAR:
            ok = 'clear' not in fail
            st['latched'] = not ok
        elif it.kind is IntentKind.DISARM:
            ok = 'disarm' not in fail
            st['armed'] = not ok
        elif it.kind is IntentKind.MOVE:
            plant.start(it.target_rev, it.vel_rps)
        rec.result(IntentResult(ok, 'OK' if ok else 'ERR', '' if ok else 'boom'))
    return log


def _kinds(log):
    return [f'{i.kind.value}:{i.purpose}' if i.purpose else i.kind.value for _, i in log]


def _rec(stall=2.85, kind=Verdict.HAND_JAM, cfg=None, resume=False):
    return JamRecovery(cfg or JamConfig(), kind=kind, stall_pos=stall,
                       restore_curr_a=50.0, iq_at_trigger=-50.1, resume=resume)


def test_recovered_sequence_relief_first_restore_last():
    rec = _rec()
    log = _drive(rec, _Plant(2.85, ball=2.85, clear=0.5))
    assert _kinds(log) == ['HOLD_RECOVERING', 'SET_CURRENT:relief', 'CONVERGE_CLEAR',
                           'DISARM', 'MOVE:raise1', 'MOVE:lower', 'SET_CURRENT:restore']
    assert log[1][1].curr_a == 10.0 and log[-1][1].curr_a == 50.0
    assert rec.outcome is Outcome.HAND_JAM_RECOVERED and not rec.holds_recovering


def test_disarm_is_confirmed_before_any_move():
    log = _drive(_rec(), _Plant(2.85))
    k = _kinds(log)
    assert k.index('DISARM') < k.index('MOVE:raise1')
    rec = _rec()
    k = _kinds(_drive(rec, _Plant(2.85), fail=('disarm',)))
    assert not any(x.startswith('MOVE') for x in k)
    assert rec.outcome is Outcome.HAND_JAM_UNRECOVERED and 'disarm' in rec.reason


def test_not_latched_skips_the_clear():
    k = _kinds(_drive(_rec(), _Plant(2.85), latched=False))
    assert 'CONVERGE_CLEAR' not in k and k[:2] == ['HOLD_RECOVERING', 'SET_CURRENT:relief']


def test_relief_failure_never_moves_and_never_clears():
    rec = _rec()
    k = _kinds(_drive(rec, _Plant(2.85), fail=('relief',)))
    assert k == ['HOLD_RECOVERING', 'SET_CURRENT:relief', 'SET_CURRENT:relief']
    assert rec.outcome is Outcome.HAND_JAM_UNRECOVERED and rec.holds_recovering


def test_raise_target_is_clamped():
    rec = _rec(stall=9.6, cfg=JamConfig(band_rev=(1.0, 9.9)))
    log = _drive(rec, _Plant(9.6))
    raise1 = [i for _, i in log if i.purpose == 'raise1'][0]
    assert raise1.target_rev == 10.0
    assert _rec(stall=2.85).raise_target(2.85, 0) == pytest.approx(3.85)


def test_lower_stall_twice_ends_unrecovered_raised_and_never_restores():
    rec = _rec()
    plant = _Plant(2.85, ball=2.85, clear=math.inf)        # wedged: never falls
    k = _kinds(_drive(rec, plant))
    assert k == ['HOLD_RECOVERING', 'SET_CURRENT:relief', 'CONVERGE_CLEAR', 'DISARM',
                 'MOVE:raise1', 'MOVE:lower', 'MOVE:raise2', 'MOVE:lower',
                 'MOVE:final_raise']
    assert 'SET_CURRENT:restore' not in k
    assert rec.outcome is Outcome.HAND_JAM_UNRECOVERED and rec.holds_recovering
    assert rec.lower_stalls == 2
    assert plant.pos == pytest.approx(2.85 + 2.5)          # left raised off the ball


def test_attempt_two_clears_a_ball_that_r1_does_not():
    rec = _rec()
    k = _kinds(_drive(rec, _Plant(2.85, ball=2.85, clear=2.0)))
    assert k[-4:] == ['MOVE:lower', 'MOVE:raise2', 'MOVE:lower', 'SET_CURRENT:restore']
    assert rec.outcome is Outcome.HAND_JAM_RECOVERED


def test_firmware_without_hand_move_to_is_relief_only_unrecovered():
    rec = _rec()
    k = _kinds(_drive(rec, _Plant(2.85, unknown=True)))
    assert k[-1] == 'MOVE:raise1' and 'SET_CURRENT:restore' not in k
    assert rec.outcome is Outcome.HAND_JAM_UNRECOVERED
    assert rec.reason == 'firmware has no HAND_MOVE_TO'


def test_snag_raise_that_does_not_track_stops():
    rec = _rec()
    plant = _Plant(2.85, snag=True)
    log = _drive(rec, plant)
    k = _kinds(log)
    # Abort to hold at measured: a retargeting move to where the hand IS,
    # never a wait on the firmware's IDLE-ing timeout; no lower, no restore.
    assert k[-2:] == ['MOVE:raise1', 'MOVE:hold']
    assert log[-1][1].target_rev == pytest.approx(2.85)
    assert rec.outcome is Outcome.HAND_JAM_UNRECOVERED and 'did not track' in rec.reason
    assert 'held at measured' in rec.reason


def test_hand_stall_out_of_band_is_relief_only():
    rec = _rec(stall=5.0, kind=Verdict.HAND_STALL)
    k = _kinds(_drive(rec, _Plant(5.0)))
    assert k == ['HOLD_RECOVERING', 'SET_CURRENT:relief']
    assert rec.outcome is Outcome.HAND_STALL_RELIEVED and rec.holds_recovering


def test_restore_not_acknowledged_is_unrecovered():
    rec = _rec()
    _drive(rec, _Plant(2.85, ball=2.85), fail=('restore',))
    assert rec.outcome is Outcome.HAND_JAM_UNRECOVERED and 'restore' in rec.reason


def test_telemetry_loss_mid_raise_ends_unrecovered():
    rec = _rec()
    _drive(rec, _Plant(2.85), latched=False, nan_after=0.05)
    assert rec.outcome is Outcome.HAND_JAM_UNRECOVERED and 'telemetry' in rec.reason


def test_abort_reports_unrecovered_and_keeps_the_hold():
    rec = _rec()
    rec.abort('exception in the jam worker')
    assert rec.outcome is Outcome.HAND_JAM_UNRECOVERED and rec.holds_recovering
    assert rec.next(RecoveryObs(t=0, pos=1, vel=0)).kind is IntentKind.END


def test_rearmed_wire_refuses_the_move():
    rec = _rec()
    rec.next(RecoveryObs(t=0, pos=2.85, vel=0))
    rec.result(IntentResult(True))                         # hold
    rec.next(RecoveryObs(t=0, pos=2.85, vel=0))
    rec.result(IntentResult(True))                         # relief
    rec.next(RecoveryObs(t=0, pos=2.85, vel=0))
    rec.result(IntentResult(True))                         # disarm (not latched)
    it = rec.next(RecoveryObs(t=0, pos=2.85, vel=0, armed=True))
    assert it.kind is IntentKind.END and 're-armed' in rec.reason


def test_resume_starts_at_the_lower():
    rec = _rec(stall=5.35, resume=True)
    k = _kinds(_drive(rec, _Plant(5.35), latched=False))
    assert k == ['HOLD_RECOVERING', 'SET_CURRENT:relief', 'DISARM', 'MOVE:lower',
                 'SET_CURRENT:restore']
    assert rec.outcome is Outcome.HAND_JAM_RECOVERED


def test_resumed_lower_that_stalls_raises_and_stays():
    rec = _rec(stall=5.35, resume=True)
    k = _kinds(_drive(rec, _Plant(5.35, ball=2.85, clear=math.inf), latched=False))
    assert k[-2:] == ['MOVE:lower', 'MOVE:final_raise']
    assert rec.outcome is Outcome.HAND_JAM_UNRECOVERED


def test_lower_waits_for_a_pending_ball_butler_throw_bounded():
    log = _drive(_rec(), _Plant(2.85), bb_until=2.5)
    t_lower = [t for t, i in log if i.purpose == 'lower'][0]
    assert t_lower >= 2.5
    log = _drive(_rec(), _Plant(2.85), bb_until=100.0)
    raise_done = [t for t, i in log if i.purpose == 'raise1'][0] + 0.4   # 1 rev @ 2.5
    t_lower = [t for t, i in log if i.purpose == 'lower'][0]
    assert t_lower <= raise_done + 0.6 + 5.0 + 0.05                    # dwell + cap


def test_next_without_result_is_a_protocol_error():
    rec = _rec()
    rec.next(RecoveryObs(t=0, pos=2.85, vel=0))
    with pytest.raises(RuntimeError):
        rec.next(RecoveryObs(t=0, pos=2.85, vel=0))


def test_planned_steps_text():
    txt = '\n'.join(planned_steps(JamConfig(), stall_pos=2.85, latched=True, armed=True,
                                  restore_curr_a=50.0, vel_limit=1000.0))
    assert 'relief first' in txt and '+3.850' in txt and 'restore LAST' in txt
    assert isinstance(Intent(IntentKind.WAIT), Intent)
