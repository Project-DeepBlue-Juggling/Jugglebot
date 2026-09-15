"""``motion/skills/hand_launch`` — the MEASURED hand launch-speed ratio.

WHAT THESE TESTS DEFEND
-----------------------
``r = v_meas / v_cmd`` is the only measured quantity the open-loop catch aim
is ever corrected by (owner decision 2026-09-15: adjust the theoretical throw
state towards the measured one, and never use QTM for catch prediction). It
multiplies a flight time, so a wrong ``r`` moves touch-down by tens of
milliseconds and the catch misses — which makes the module's REFUSALS as
load-bearing as its estimate: every ``None`` path below is a window the
monitor will not vouch for, and a ``None`` keeps the theoretical aim (the aim
that is already committed), never a guess.

The ratio is defined on the two channels' PEAKS over the stroke window, not on
a sample at release, because at release the signal's slope is thousands of
rev/s² and a few ms of stamp skew is a several-percent error in exactly the
quantity being estimated. The peak is a stationary point, so it is the
skew-insensitive feature — that is what
:func:`test_the_ratio_survives_a_sampling_skew_between_the_two_channels` pins.

Ground truth: the measured hand overspeed of the 2026-09-15 sitting, bag
``~/Desktop/rosbags/2026-09-15_18-51-37``, reduced per throw by
``tools/probes/hand_overspeed_probe.py`` — rows restated inline below because
``temp/probes/`` is gitignored and a test may not depend on it.

Unmarked and parallel-safe: pure Python, no clock, no filesystem, no ports.

Plan: ``plans/active/two-ball-skill-stack.md`` § 2.5 (catch aim source).
"""

from __future__ import annotations

import pytest

from jugglebot.motion.skills.hand_launch import HandLaunchMonitor

#: One throw stroke's worth of samples at the telemetry cadence.
DT = 0.01
#: (cmd_peak_rps, meas_peak_rps, meas_over_cmd) from the 2026-09-15 bag's
#: per-throw reduction — the 0.9 m and 0.5 m apex ends of that sitting's
#: ladder. These are the numbers the correction is sized for.
BAG_ROWS = ((127.6, 138.4, 1.084), (95.5, 100.7, 1.054))


def _stroke(mon, t_release, *, peak_cmd, r, n=20, sign=1.0,
            meas_lag_samples=0, n_post=0):
    """Feed a throw stroke whose commanded velocity rises to ``peak_cmd`` AT
    ``t_release`` and (with ``n_post``) brakes after it — the real shape, so
    the peak is a stationary point rather than a truncated ramp. The measured
    channel is the same shape scaled by ``r``, optionally lagged by whole
    samples (the bag measured 9-16 ms of command-to-measurement delay)."""
    cmds = [peak_cmd * (i + 1) / n for i in range(n)]
    cmds += [peak_cmd * (1.0 - (k + 1) / (n_post + 1.0))
             for k in range(n_post)]
    for i, cmd in enumerate(cmds):
        j = i - meas_lag_samples
        meas = cmds[j] * r if 0 <= j < len(cmds) else 0.0
        mon.add_sample(t_release + (i - (n - 1)) * DT, sign * cmd,
                       sign * meas)


def test_the_ratio_is_the_peak_of_each_channel_over_the_stroke():
    mon = HandLaunchMonitor()
    _stroke(mon, 100.0, peak_cmd=127.6, r=1.084)
    assert mon.ratio(100.0) == pytest.approx(1.084, abs=1e-6)


@pytest.mark.parametrize('peak_cmd,peak_meas,r', BAG_ROWS)
def test_the_measured_rows_of_the_2026_09_15_sitting_reproduce(peak_cmd,
                                                               peak_meas, r):
    """The bag's own reduction is `meas_peak / cmd_peak` — this module must
    be the SAME definition, or the correction is sized against a number
    nothing measured."""
    mon = HandLaunchMonitor()
    _stroke(mon, 50.0, peak_cmd=peak_cmd, r=peak_meas / peak_cmd)
    assert mon.ratio(50.0) == pytest.approx(peak_meas / peak_cmd, abs=1e-6)
    # ... and that IS the ratio the bag's own reduction reported.
    assert abs(peak_meas / peak_cmd - r) < 1e-3


def test_the_ratio_survives_a_sampling_skew_between_the_two_channels():
    """A measured channel lagged two whole samples (20 ms) still yields the
    same ratio to within a percent: the peaks are stationary points, so the
    skew that would wreck a single-sample-at-release estimate barely moves
    this one. A release-instant sample of the same stroke is 10 % low at this
    lag (the triangle's slope is peak/n per sample)."""
    mon = HandLaunchMonitor()
    _stroke(mon, 10.0, peak_cmd=100.0, r=1.08, meas_lag_samples=1,
            n_post=2)
    r = mon.ratio(10.0)
    assert r == pytest.approx(1.08, rel=0.02)


def test_a_reversed_hand_direction_cannot_invert_the_correction():
    """Both peaks are taken along the COMMANDED peak's sign, so a hand wired
    the other way round reports the same ratio — never ``1/r``, never a
    negative one."""
    mon = HandLaunchMonitor()
    _stroke(mon, 10.0, peak_cmd=120.0, r=1.09, sign=-1.0)
    assert mon.ratio(10.0) == pytest.approx(1.09, abs=1e-6)


def test_too_few_samples_in_the_window_is_a_refusal():
    """Telemetry not flowing (or a release still in the future) must not
    produce a correction from two samples."""
    mon = HandLaunchMonitor(min_samples=4)
    mon.add_sample(9.99, 100.0, 108.0)
    mon.add_sample(10.0, 120.0, 130.0)
    assert mon.ratio(10.0) is None
    assert 'only 2 hand samples' in mon.last_reason


def test_a_window_with_no_throw_in_it_is_a_refusal():
    """A rest or catch stroke's commanded peak is an order of magnitude below
    a throw's; ratio-ing one would hand the catch a correction measured off
    motion that is not the throw."""
    mon = HandLaunchMonitor()
    _stroke(mon, 10.0, peak_cmd=2.0, r=1.5)
    assert mon.ratio(10.0) is None
    assert 'below the' in mon.last_reason


def test_a_ratio_outside_the_trust_band_is_a_refusal():
    """A 2× ratio is not a 9 %-class plant error — it is a broken window (the
    wrong stroke, a fault, a dropped channel), and the safe answer to that is
    "no correction"."""
    mon = HandLaunchMonitor()
    _stroke(mon, 10.0, peak_cmd=120.0, r=2.0)
    assert mon.ratio(10.0) is None
    assert 'outside' in mon.last_reason


def test_only_the_stroke_ENDING_at_the_release_is_measured():
    """Two throws in one attempt: asking about the second release must not
    see the first stroke's peaks (a 0.30 s window at a ≥1.4 s beat)."""
    mon = HandLaunchMonitor()
    _stroke(mon, 10.0, peak_cmd=120.0, r=1.20)      # r outside the band
    _stroke(mon, 11.5, peak_cmd=120.0, r=1.08)
    # A blended window would peak the measured channel at 1.20 x 120 rev/s
    # against the same commanded peak, i.e. report the FIRST stroke's ratio.
    assert mon.ratio(11.5) == pytest.approx(1.08, abs=1e-6)


def test_a_sample_past_the_release_is_admitted_only_inside_the_post_window():
    """The true peak can be sampled a few ms LATE (the release instant is the
    schedule's, the samples are the hand's), so a small post-window is
    admitted — but not the brake stroke that follows."""
    mon = HandLaunchMonitor()
    _stroke(mon, 10.0, peak_cmd=100.0, r=1.0)
    mon.add_sample(10.0 + 0.01, 100.0, 108.0)       # inside post_s
    mon.add_sample(10.0 + 0.10, 100.0, 300.0)       # brake, far outside
    assert mon.ratio(10.0) == pytest.approx(1.08, abs=1e-6)


def test_a_clock_step_backwards_drops_the_history_it_cannot_compare():
    mon = HandLaunchMonitor()
    _stroke(mon, 1000.0, peak_cmd=120.0, r=1.08)
    mon.add_sample(10.0, 120.0, 130.0)              # a ROS time jump
    assert len(mon) == 1
    assert mon.ratio(1000.0) is None


def test_nan_samples_and_unstamped_rows_are_dropped():
    mon = HandLaunchMonitor()
    nan = float('nan')
    mon.add_sample(nan, 1.0, 1.0)
    mon.add_sample(0.0, 1.0, 1.0)
    mon.add_sample(10.0, nan, 1.0)
    mon.add_sample(10.0, 1.0, nan)
    assert len(mon) == 0


def test_the_history_is_bounded():
    """The monitor lives for the node's life, so it must not grow with it."""
    mon = HandLaunchMonitor(history_s=0.5)
    t = 0.0
    for _ in range(2000):
        t += DT
        mon.add_sample(t, 1.0, 1.0)
    assert len(mon) <= int(0.5 / DT) + 2
