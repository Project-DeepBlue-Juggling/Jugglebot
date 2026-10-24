"""The Python twin of the can-bridge's frame-age-aware LEG lead clamp (FW 31).

Plan ``plans/active/leg-lead-clamp-age-aware.md``; design in
``logbook/2026-10-24-leg-lead-clamp-age-aware-p1-firmware.md``.  The firmware clamp
anchored to the last encoder frame (``pos_rev``); frames arrive irregularly (median
~10 ms, up to ~100 ms) and a moving leg travels more than the 0.10 rev band during such a gap, so the clamp
PINNED the command to the stale anchor and then stepped it ~10 mm when the next frame
landed (the 2026-10-24 staircase).  Form Z: while a frame is aged the anchor is
``fb + (raw_cmd_now - raw_cmd_at_the_frame)``, frozen once the age passes the
150 ms staleness cap; at age 0 it is bit-identical to FW 30.

``TeensyLegInterp(leg_clamp=...)``: ``motor_guard`` (default; what the xref pins),
``firmware_fw30`` (clamp as flashed through FW 30), ``firmware`` (FW 31).

REPLAY ORACLE RECIPE (test_replay_oracle_*): the committed fixture
``tools/probes/data/lead_clamp_staircase_2026-10-24.json`` (regenerate with
``python tools/probes/lead_clamp_staircase_probe.py --emit-fixture``) holds, per window,
the 40 Hz ``/leg_setpoint_echo`` knots, the leg's fresh encoder samples and the recorded
``/leg_cmd_executed``.  ``replay_window`` latches the knots at their recorded times
(u0=k, u1=k+1, u2=k+2, v0 = chord like xref.py), ticks at 500 Hz, and feeds fb / fb_ts by
sample-and-hold of the fresh encoder samples.  CAVEAT: the recorded encoder trace carries
the lag that TODAY's clamp built (the leg was pinned behind its command for 10-20 ms
at a time), so the frame-time tracking error is often > 0.10 rev; FW 31 cannot read
perfectly clean on it -- a frame that lands with the leg already > band behind its command
must still be clamped, and that clamp steps.  The claim asserted is therefore: no clamp
engagement on gap ticks whose frame-time error was inside the band, and a large drop of the
staircase step.
"""

from __future__ import annotations

import os
import re
import sys

import numpy as np
import pytest
from hypothesis import given, settings, strategies as st

_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "tools", "probes", "teensy_link_profiling", "hermite_xref"))
sys.path.insert(0, os.path.join(_REPO, "tools", "probes"))

import teensy_interp as ti  # noqa: E402
from teensy_interp import TeensyLegInterp, NUM_LEGS  # noqa: E402

xref = pytest.importorskip("xref")
_CFG = os.path.join(_REPO, "ros_ws", "src", "jugglebot", "Teensy_code_canbridge",
                    "canbridge_config.h")
_FIXTURE = os.path.join(_REPO, "tools", "probes", "data", "lead_clamp_staircase_2026-10-24.json")
_LO, _HI = [-100.0] * NUM_LEGS, [100.0] * NUM_LEGS
_DT = 0.002


# ── 1. constants pinned to the firmware header ───────────────────────────────
def _hdr(name):
    text = open(_CFG).read()
    m = re.search(rf"constexpr\s+(?:float|uint32_t)\s+{name}\s*=\s*([0-9.eE+-]+)[fu]?\s*;", text)
    assert m, f"{name} not found in canbridge_config.h"
    return float(m.group(1))


def test_leg_clamp_constants_match_the_firmware():
    assert ti.MAX_LEAD_REV_FW == _hdr("MAX_LEAD_REV")
    assert ti.LEAD_CLAMP_VELFF_LIMIT_RPS == _hdr("LEAD_CLAMP_VELFF_LIMIT_RPS")
    assert ti.MOTOR_FB_STALENESS_S == pytest.approx(_hdr("MOTOR_FB_STALENESS_US") * 1e-6)
    assert ti.LEG_ANCHOR_STALE_S == pytest.approx(_hdr("LEG_ANCHOR_STALE_US") * 1e-6)
    assert ti.MAX_LEAD_REV == 0.15          # motor_guard's value, deliberately not the firmware's


# ── 2. the default mode did not move ─────────────────────────────────────────
def test_motor_guard_mode_is_untouched_by_the_mode_switch():
    assert TeensyLegInterp(_LO, _HI).leg_clamp == "motor_guard"
    max_dpos, max_dvel = xref.run_synthetic_xref()
    assert max_dpos < 1e-6 and max_dvel < 1e-6


# ── 3. age-0 identity (property) ─────────────────────────────────────────────
_rev = st.floats(min_value=-3.0, max_value=3.0, allow_nan=False, allow_infinity=False)
_vec = st.lists(_rev, min_size=NUM_LEGS, max_size=NUM_LEGS)


@settings(deadline=None)
@given(u0=_vec, u1=_vec, u2=_vec, off=st.lists(st.floats(-0.4, 0.4), min_size=NUM_LEGS, max_size=NUM_LEGS),
       dts=st.lists(st.floats(0.0, 0.2), min_size=1, max_size=6), with_u=st.integers(0, 2))
def test_age_zero_firmware_equals_fw30_and_never_wider(u0, u1, u2, off, dts, with_u):
    """At age 0 FW 31 == FW 30 bit for bit, and the band is never wider than 0.10 rev."""
    a = TeensyLegInterp(_LO, _HI, leg_clamp="firmware_fw30")
    b = TeensyLegInterp(_LO, _HI, leg_clamp="firmware")
    v0 = [(u1[i] - u0[i]) / ti.SEGMENT_T_S for i in range(NUM_LEGS)]
    kw = {}
    if with_u >= 1:
        kw["u1"] = u1
    if with_u >= 2:
        kw["u2"] = u2
    for m in (a, b):
        m.latch_setpoint(u0, v0, [0.0] * NUM_LEGS, [0.0] * NUM_LEGS, 100.0, **kw)
    for dt in sorted(dts):
        t = 100.0 + dt
        fb = [u0[i] + off[i] for i in range(NUM_LEGS)]
        pa, va, _ = a.tick(t, fb)
        pb, vb, _ = b.tick(t, fb, [t] * NUM_LEGS)
        assert pa == pb and va == vb          # bit-identical, not approx
        for i in range(NUM_LEGS):
            assert abs(pb[i] - fb[i]) <= ti.MAX_LEAD_REV_FW + 1e-12


# ── 4. dead-reckoned coast ───────────────────────────────────────────────────
_V = 2.0                     # rev/s, constant-velocity leg
_LAG = 0.05                  # rev, the encoder trails the command by this (inside the band)


def _coast(mode, gap_s, t0=10.0003, tail=0.05):
    """Stream a constant-velocity command; the leg's frame stops at t0 for gap_s, then returns."""
    m = TeensyLegInterp(_LO, _HI, leg_clamp=mode)
    T = ti.SEGMENT_T_S
    pos = lambda t: _V * t
    rows = []
    last_k = -1
    n = int(round((t0 + gap_s + tail - 9.0) / _DT))
    fb_ts_f, fb_f = 0.0, 0.0
    for s in range(n):
        t = 9.0 + s * _DT
        k = int(np.floor(t / T + 1e-9))
        if k != last_k:
            last_k = k
            u = [[pos((k + j) * T)] * NUM_LEGS for j in range(3)]
            m.latch_setpoint(u[0], [_V] * NUM_LEGS, [0.0] * NUM_LEGS, [0.0] * NUM_LEGS, k * T,
                             u1=u[1], u2=u[2], v1=[_V] * NUM_LEGS)
        if t < t0 or t >= t0 + gap_s:       # frames every 10 ms outside the gap
            fb_ts_f = t - ((t - 9.0) % 0.010) if t < t0 else t
            fb_f = pos(fb_ts_f) - _LAG
        pos_out, vel_out, _ = m.tick(t, [fb_f] * NUM_LEGS, [fb_ts_f] * NUM_LEGS)
        rows.append((t, pos_out[0], vel_out[0], pos(t), fb_f, fb_ts_f, m.leg_anchor[0], m.leg_clamped[0]))
    return m, rows


def test_coast_fw30_pins_and_steps_while_firmware_emits_the_raw_command():
    gap = 0.12
    m30, r30 = _coast("firmware_fw30", gap)
    m31, r31 = _coast("firmware", gap)
    t0 = 10.0003
    gap_rows = lambda rows: [r for r in rows if t0 + 0.0005 < r[0] < t0 + gap]
    # FW 30: pinned at fb + 0.10 once the raw command is 0.10 ahead of the frozen frame.
    pinned = [r for r in gap_rows(r30) if r[7]]
    assert len(pinned) > 30 and all(abs(r[1] - (r[4] + 0.10)) < 1e-9 for r in pinned)
    # ... and steps when the frame returns (> one band's worth of catch-up).
    step30 = max(abs(r30[i + 1][1] - r30[i][1]) for i in range(len(r30) - 1))
    assert step30 > 0.10
    # FW 31: raw command on every gap tick, no pin, census = ticks with age > 30 ms.
    assert all(not r[7] and abs(r[1] - r[3]) < 1e-9 for r in gap_rows(r31))
    assert m31.leg_lead_clamp_ticks[0] == 0
    expect_stale = sum(1 for r in r31 if r[0] - r[5] > ti.LEG_ANCHOR_STALE_S)
    assert m31.leg_stale_anchor_ticks[0] == expect_stale > 40
    # continuity at the frame return: no step larger than one tick's travel.
    step31 = max(abs(r31[i + 1][1] - r31[i][1]) for i in range(len(r31) - 1))
    assert step31 <= _V * _DT * 1.01 + 1e-9


def test_coast_past_the_cap_freezes_the_shift_and_a_frame_resets_it():
    gap = 0.40
    m, rows = _coast("firmware", gap, tail=0.0)
    t0 = 10.0003
    late = [r for r in rows if r[0] - r[5] > ti.MOTOR_FB_STALENESS_S + 1e-9]
    assert len(late) > 50
    assert max(r[6] for r in late) - min(r[6] for r in late) == 0.0      # anchor frozen past the cap
    assert late[-1][7] and abs(late[-1][1] - (late[-1][6] + 0.10)) < 1e-9  # clamp re-engaged at the band
    # the frame returns: shift resets, anchor == fb exactly.
    m2, rows2 = _coast("firmware", gap, tail=0.02)
    last = rows2[-1]
    assert m2._leg_dr[0] == 0.0 and last[6] == last[4]


# ── 5. replay oracle on the committed 2026-10-24 windows ─────────────────────
def _oracle():
    import lead_clamp_staircase_probe as probe
    return probe, probe.load_fixture(_FIXTURE)


@pytest.mark.parametrize("name,min_cut", [("S1A_leg1", 0.40), ("S1B_leg2", 0.25), ("S2_leg1", 0.40)])
def test_replay_oracle_staircase_fw30_vs_fw31(name, min_cut):
    """S1B (leg 2) is held to 25 %, not 40 %: 2 of its 3 largest remaining FW 31 steps land on
    frames whose frame-time error is 0.23-0.30 rev, i.e. the recorded leg really was > band
    behind its command (the lag today's clamp built), so the clamp must engage there."""
    probe, fx = _oracle()
    w = next(x for x in fx["windows"] if x["name"] == name)
    r = probe.twin_summary(w)
    f30, f31 = r["firmware_fw30"], r["firmware"]
    print(f"\n{name}: fw30 max step {f30['max_step_mm']:.2f} mm, gap engaged {f30['gap_engaged_frac']*100:.1f} % | "
          f"fw31 max step {f31['max_step_mm']:.2f} mm, gap engaged {f31['gap_engaged_frac']*100:.1f} %, "
          f"sub-band gap ticks {f31['subband_gap_ticks']} engaged {f31['subband_gap_engaged']}")
    assert f30["max_step_mm"] >= 8.0                     # the staircase reproduces (0.113 rev)
    assert f30["gap_engaged_frac"] >= 0.20
    assert f31["max_step_mm"] <= (1.0 - min_cut) * f30["max_step_mm"]
    assert f31["subband_gap_ticks"] > 50                 # the claim below is not vacuous
    assert f31["subband_gap_engaged"] == 0
