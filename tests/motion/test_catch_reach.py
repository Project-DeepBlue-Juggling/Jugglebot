"""``motion/trajectory/catch_reach`` — the pre-throw verdict on the DEFERRED A→B
catch reach, and the identity that keeps the probe honest.

WHY THIS MODULE EXISTS AT ALL (and why its tests are here rather than in
``tests/ros/``): Tier 8b pre-positions the platform at the swing-compensated
pre-tilt pose at the throw site A and DEFERS the A→B translation to
``t_release``. So ``go_to_pose`` never judges the nominated catch pose B, the
FSM's z band is a scalar on ``B.z``, and its closed-form bound is a scalar on
``|B − A|`` — none of them bound ``|B|``. Measured 2026-08-29 on this tree,
``B = (250, 0)`` at ``T = 0.80 s`` was ADMITTED pre-throw and then refused
``WORKSPACE`` by the planner mid-flight, with the ball in the air. This module
is the gate that closes that, and this file pins it as a KINEMATICS question,
which is what it is.

THE IDENTITY, pinned last and load-bearing: ``tools/probes/
displaced_reach_frontier.py`` publishes the frontier every toss document quotes,
and the machine now gates on it. Those two must be ONE body — a probe measuring
a frontier the machine does not gate on is worse than no probe — so the last
test asserts the probe reaches this module rather than carrying a copy.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

import jugglebot.hardware_config as hw
from jugglebot.motion.trajectory import catch_reach
from jugglebot.motion.trajectory.limits import TrajectoryLimits

_REPO = Path(__file__).resolve().parents[2]
_PROBE = _REPO / 'tools' / 'probes' / 'displaced_reach_frontier.py'

Z_ACTIVE = float(hw.JB_OP_DEFAULT_ACTIVE_Z_MM)


def _load_probe():
    """Import the probe by PATH — it lives outside any package, deliberately
    (``tools/probes/README.md``: probes are standalone scripts, not a library)."""
    spec = importlib.util.spec_from_file_location('displaced_reach_frontier',
                                                  _PROBE)
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault('displaced_reach_frontier', mod)
    spec.loader.exec_module(mod)
    return mod


# ── the verdicts themselves ───────────────────────────────────────────────────

@pytest.mark.parametrize('a_xy,b_xy,flight_s', [
    ((0.0, 0.0), (0.0, 0.0), 0.80),        # degenerate — "8b subsumes 8a"
    ((0.0, 0.0), (70.0, 0.0), 0.80),       # the 11/11 hardware-validated rung
    ((0.0, 0.0), (100.0, 0.0), 0.80),
    ((0.0, 0.0), (0.0, -100.0), 0.80),     # the ring is not +x-only
    ((70.0, 70.0), (0.0, 0.0), 0.95),      # an OFF-CENTRE throw site
])
def test_a_reach_inside_the_frontier_is_ok(a_xy, b_xy, flight_s):
    """Small and moderate centred reaches PLAN. This is the half that matters
    for not breaking the capability: a gate that refused these would refuse the
    displaced throws Tier 8b exists for."""
    assert catch_reach.catch_reach_feasible(a_xy, b_xy, flight_s) == 'OK'


@pytest.mark.parametrize('b_xy,flight_s', [
    ((250.0, 0.0), 0.80),                  # inside the 256 mm closed-form bound
    ((500.0, 0.0), 1.00),                  # exactly ON the 500 mm bound
    ((300.0, 300.0), 1.00),
])
def test_a_far_reach_is_refused_workspace(b_xy, flight_s):
    """THE MEASURED WINDOW (2026-08-29). Every case here clears the FSM's
    surviving pre-throw ladder — the closed-form ``|B − A|`` bound admits them,
    the z band is untouched at ``z = 170``, and the aim stays under the 12°
    ceiling — and every one is refused ``WORKSPACE`` by ``build_catch``.

    Before this gate existed that verdict arrived at ``t_release`` with the ball
    already airborne: a miss and a dead catch, not a refusal. ``WORKSPACE``
    specifically (not merely "not OK") because the code becomes the outcome's
    subcode and routes the operator — a drift to ``UNREACHABLE`` or a limit code
    would be a real change in what refused, and worth a failing test."""
    assert catch_reach.catch_reach_feasible((0.0, 0.0), b_xy, flight_s) == \
        'WORKSPACE'


def test_the_verdict_form_carries_the_numbers_behind_the_code():
    """``catch_reach_verdict`` is the ``(code, info)`` form the probe surveys
    with; ``catch_reach_feasible`` is its code alone. The two must agree by
    construction — the second calls the first — and the info must carry enough
    to explain a verdict without re-planning it."""
    code, info = catch_reach.catch_reach_verdict((0.0, 0.0), (100.0, 0.0), 0.80)
    assert code == 'OK'
    assert code == catch_reach.catch_reach_feasible((0.0, 0.0), (100.0, 0.0),
                                                   0.80)
    assert info['displacement_mm'] == pytest.approx(100.0, abs=1e-6)
    assert 0.0 < info['throw_tilt_deg'] < 12.0
    assert info['event_vel_mps'] > 0.0
    # The sampled leg-space peaks the frontier tables print — present only on a
    # plan that actually built, which is what makes them a plan-happened marker.
    assert info['peak_leg_vel_mmps'] > 0.0
    assert info['peak_leg_jerk_mmps3'] > 0.0

    far_code, far_info = catch_reach.catch_reach_verdict(
        (0.0, 0.0), (250.0, 0.0), 0.80)
    assert far_code == 'WORKSPACE'
    # A refusal short-circuits before the sampled pass, so it carries the aim
    # numbers and NOT the peaks — the absence is the evidence of where it died.
    assert 'peak_leg_vel_mmps' not in far_info


def test_live_session_limits_move_the_verdict():
    """The gate fronts for the feasibility gate that runs at ``t_release``, so
    it must judge against the LIVE session limits — the same argument the
    closed-form bound is built on. A ramp-DOWN must be able to refuse a reach
    the YAML default admits, or a soft session would sail past this gate and
    refuse mid-flight, which is the exact failure it exists to prevent."""
    a, b, T = (0.0, 0.0), (100.0, 0.0), 0.80
    assert catch_reach.catch_reach_feasible(a, b, T) == 'OK'
    crawl = TrajectoryLimits.from_config(
        hw, leg_vel_mmps=20.0, leg_acc_mmps2=50.0, leg_jerk_mmps3=100.0)
    assert catch_reach.catch_reach_feasible(a, b, T, limits=crawl) != 'OK'


def test_the_catch_z_defaults_to_the_active_plane():
    """``catch_z_mm=None`` means the ACTIVE plane, and passing that plane
    explicitly must be the SAME call — the node passes ``catch_pose[2]``
    verbatim, so a default that drifted from it would make the production gate
    judge a different pose than the probe's frontier."""
    assert (catch_reach.catch_reach_verdict((0.0, 0.0), (100.0, 0.0), 0.80)
            == catch_reach.catch_reach_verdict((0.0, 0.0), (100.0, 0.0), 0.80,
                                               catch_z_mm=Z_ACTIVE))


# ── THE identity: one body, two callers ───────────────────────────────────────

def test_the_frontier_probe_delegates_to_this_module():
    """The probe's ``reach_verdict`` IS this module — not a copy of it.

    Asserted two ways because either alone is escapable: the probe's function
    must reach ``catch_reach.catch_reach_verdict`` (patched here, so a probe
    carrying its own body would never call it), and its verdicts must equal the
    module's across the window that matters. A drift between them would mean the
    frontier the toss documents quote is not the frontier the machine gates on.
    """
    probe = _load_probe()
    seen = []
    real = catch_reach.catch_reach_verdict

    def _spy(*args, **kwargs):
        seen.append((args, tuple(sorted(kwargs))))
        return real(*args, **kwargs)

    original = catch_reach.catch_reach_verdict
    catch_reach.catch_reach_verdict = _spy
    try:
        verdict, _info = probe.reach_verdict((0.0, 0.0), (250.0, 0.0), 0.80)
    finally:
        catch_reach.catch_reach_verdict = original
    assert seen, 'the probe did not reach the shared helper'
    assert verdict == 'WORKSPACE'

    for a, b, T in (((0.0, 0.0), (100.0, 0.0), 0.80),
                    ((0.0, 0.0), (250.0, 0.0), 0.80),
                    ((0.0, 0.0), (0.0, 0.0), 0.60),
                    ((70.0, 70.0), (-70.0, -70.0), 0.95)):
        assert probe.reach_verdict(a, b, T)[0] == \
            catch_reach.catch_reach_feasible(a, b, T, catch_z_mm=probe.Z_ACTIVE,
                                             limits=probe.LIMITS,
                                             geom=probe.GEOM,
                                             settle_hold_s=probe.SETTLE_S)


def test_the_probe_does_not_carry_its_own_gravity():
    """``GRAVITY_MMS2`` is the ballistics-side 9806, never the tracker's 9810,
    and the probe re-exports the module's rather than declaring one. Two copies
    of a constant that appears in a ballistic inverse is exactly how a probe
    starts quoting a frontier the machine does not have."""
    probe = _load_probe()
    assert probe.GRAVITY_MMS2 is catch_reach.GRAVITY_MMS2
    assert catch_reach.GRAVITY_MMS2 == 9806.0
