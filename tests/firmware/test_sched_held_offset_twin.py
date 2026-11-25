"""can-bridge FW 52 (BUG-99 D3): the twin's held-offset lane decision.

``teensy_interp.TeensyInterp.sched_on_setpoint`` mirrors ``leg_interp.cpp``
``sched_lane_for``: unsynced, a stream whose scheduled group is active with the
output enabled stays scheduled on the held offset until the anchor is
``TIME_OFFSET_HOLD_MAX_US`` old, then is dropped; otherwise it is demoted.
"""
from __future__ import annotations

import os
import re
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_TWIN = os.path.join(_ROOT, 'tools', 'probes', 'teensy_link_profiling', 'hermite_xref')
if _TWIN not in sys.path:
    sys.path.insert(0, _TWIN)

import teensy_interp as ti  # noqa: E402

_Z = [0.5] * 7


def _frame(tw, t_origin, **kw):
    return tw.sched_on_setpoint(t_origin, t_origin - 20_000, _Z, _Z, _Z, [0.0] * 7, [0.0] * 7, [0.0] * 7, **kw)


def _twin():
    return ti.TeensyLegInterp([0.0] * 6, [10.0] * 6)


def test_hold_constant_matches_the_firmware():
    h = open(os.path.join(_ROOT, 'ros_ws/src/jugglebot/Teensy_code_canbridge/time_base.h'), encoding='utf-8').read()
    m = re.search(r"TIME_OFFSET_HOLD_MAX_US = (\d+)'(\d+)'(\d+)ULL", h)
    assert m and int(''.join(m.groups())) == ti.TIME_OFFSET_HOLD_MAX_US


def test_unsynced_with_an_active_group_is_held_then_dropped():
    tw = _twin()
    tw.sched_out_en = True
    tw._sg[0].active = True
    assert _frame(tw, 1_000_000, time_synced=False, anchor_age_us=100_000_000) == 'sched'
    assert tw.sched_held_offset == 1 and tw.sched_demoted == 0
    assert _frame(tw, 1_025_000, time_synced=False, anchor_age_us=ti.TIME_OFFSET_HOLD_MAX_US) == 'future'
    assert tw.sched_demoted == 0


def test_unsynced_without_a_group_or_output_is_demoted():
    tw = _twin()
    tw.sched_out_en = True
    assert _frame(tw, 1_000_000, time_synced=False) == 'demoted'
    tw._sg[0].active = True
    tw.sched_out_en = False
    assert _frame(tw, 1_025_000, time_synced=False) == 'demoted'
    assert tw.sched_held_offset == 0


# ── FW 52 (BUG-99 D3) part 2: the legacy -> scheduled entry check, mirrored ──────
# The twin's legacy Hand lane plays a constant 10 rev/s (Mode 1, v0 = v1), then a
# scheduled frame promotes into the inactive Hand group. Continuous: accepted
# (PLAY). Offset by v x 24 ms (the FW 51 arrival-vs-stamp step, 0.24 rev, past
# SCHED_RESUME_TOL_POS_HAND_REV = 0.05): refused into a LATCHED C2 stop from the
# legacy state, counted in sched_refused.
_V = 10.0
_T0 = 1.0                      # legacy latch, s
_TS_US = 1_010_000             # the scheduled stamp, on a tick (2 ms grid)


def _hand_legacy_then_sched(offset_rev, out_en=True, hand_slewing=False):
    tw = _twin()
    tw.sched_out_en = out_en
    tw.hand_recover_slewing = hand_slewing
    tw.latch_hand(0.0, _V, _T0, u1=_V * 0.025, u2=_V * 0.050, v1=_V)
    t_us = 1_000_000
    while t_us < _TS_US:
        tw.tick_hand(t_us * 1e-6, fb_rev=0.0)
        t_us += 2000
    p_at = _V * (_TS_US * 1e-6 - _T0) + offset_rev
    u0 = [0.5] * 6 + [p_at]
    u1 = [0.5] * 6 + [p_at + _V * 0.025]
    u2 = [0.5] * 6 + [p_at + _V * 0.050]
    vv = [0.0] * 6 + [_V]
    assert tw.sched_on_setpoint(_TS_US, _TS_US - 20_000, u0, u1, u2, vv, vv, vv,
                                has_hand=True) == 'sched'
    tw.tick_hand(_TS_US * 1e-6, fb_rev=0.0)
    return tw


def test_legacy_record_constant_matches_the_firmware():
    src = open(os.path.join(_ROOT, 'ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp'),
               encoding='utf-8').read()
    assert 'LEGACY_REC_MAX_AGE_US = 4 * (int64_t)INTERP_PERIOD_US;' in src
    cfg = open(os.path.join(_ROOT, 'ros_ws/src/jugglebot/Teensy_code_canbridge/canbridge_config.h'),
               encoding='utf-8').read()
    rate = int(re.search(r'INTERP_RATE_HZ\s*=\s*(\d+)', cfg).group(1))
    assert 'INTERP_PERIOD_US    = 1000000u / INTERP_RATE_HZ;' in cfg
    assert ti.LEGACY_REC_MAX_AGE_US == 4 * (1_000_000 // rate)


def test_a_continuous_legacy_to_scheduled_entry_is_accepted():
    tw = _hand_legacy_then_sched(0.0)
    assert tw.sched_phase(1) == 1 and tw.sched_refused == 0
    assert not tw._sg[1].latched


def test_a_discontinuous_entry_is_refused_into_a_latched_stop_from_the_legacy_state():
    tw = _hand_legacy_then_sched(_V * 0.024)
    assert tw.sched_phase(1) == 2 and tw._sg[1].latched
    assert tw.sched_refused == 1
    g = tw._sg[1]
    assert abs(g.ev[ti.HAND_AXIS] - _V) < 1e-9                 # the stop starts from the legacy v
    assert abs(g.ep[ti.HAND_AXIS] - _V * (_TS_US * 1e-6 - _T0)) < 1e-6   # ... and its p + v dt
    assert tw.hand_base_pos == _V * (_TS_US * 1e-6 - _T0) + _V * 0.024  # the refused frame as base


def test_the_entry_check_is_skipped_with_the_output_off_or_under_a_recovery_slew():
    assert _hand_legacy_then_sched(_V * 0.024, out_en=False).sched_phase(1) == 1
    assert _hand_legacy_then_sched(_V * 0.024, hand_slewing=True).sched_phase(1) == 1
