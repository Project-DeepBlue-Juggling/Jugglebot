"""Headless integration test for the interactive BB -> Jugglebot catch tool
(``sim.juggle_bb_catch``).

The tool is operator-driven in the viewer, but its headless path runs scripted
throws so the catch can be exercised without a display. This pins the DEFAULT
preset — the real Ball Butler solution to ``(0, 0, catch_z)`` under the §3 noise
(2% BB throw + 0.5 mm tracking, on) — as caught, held, and seated within a small
in-cup offset, across several seeds, deterministically.

The preset throw reuses the validated Rung-1 catch (``SingleCatchRunner``, the
phase-matched fast-capable seat) via the additive, default-off
``SingleCatchConfig.manual_launch_mm`` field; the catch controller / contact / seat
are unchanged. These drive the full MuJoCo contact-carry catch, so the seed set is
kept lean. Measured (2026-07-02): the preset seats ~5.6 mm in-cup on seeds 0-2.
"""
import pytest

from sim.juggle_bb_catch import BBCatchConfig, run


def _throw(seed, **kw):
    """Run one headless scripted throw; return its result dict."""
    res = run(BBCatchConfig(headless=True, throws=1, seed=seed, realtime_rate=0.0, **kw))
    assert len(res) == 1
    return res[0]


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_preset_throw_is_caught_and_seated(seed):
    """The default preset (real-BB solution to (0,0,catch_z), §3 noise, seeded) is
    caught, held, and seated within ~20 mm of the cup centre, across >= 3 seeds."""
    r = _throw(seed)
    assert r['caught'] and r['held'], f"seed {seed}: not caught+held ({r})"
    assert r['in_off_mm'] <= 20.0, f"seed {seed}: seated {r['in_off_mm']:.1f} mm off (> 20)"


def test_headless_runs_multiple_throws():
    """``--throws N`` runs N distinct (per-seed) scripted throws, each a clean catch."""
    res = run(BBCatchConfig(headless=True, throws=3, seed=0))
    assert len(res) == 3
    for i, r in enumerate(res):
        assert r['caught'] and r['held'], f"throw {i}: not caught+held ({r})"
        assert r['in_off_mm'] <= 20.0, f"throw {i}: seated {r['in_off_mm']:.1f} mm off"


def test_preset_throw_is_deterministic_per_seed():
    """The §3 noise is the only RNG and the seed is threaded through, so a given seed
    reproduces the same catch result exactly (offset, reach, landing)."""
    assert _throw(1) == _throw(1)


def test_full_flight_preset_is_also_caught():
    """The optional ``--full-flight`` spawn (release-point, whole arc) still catches
    the demo preset cleanly (the far/high demo BB clears the cup on the rising arc)."""
    r = _throw(0, full_flight=True)
    assert r['caught'] and r['held']
    assert r['in_off_mm'] <= 20.0
