"""Tests for ZmqTargetSource — Bundle A of MPC_OVERSHOOT_SATURATION fix.

Validates that on every *distinct* target arrival, the source emits a
quintic ``ref_events`` anchored at the live plant pose + (clamped) twist
at the current sim_time.  This is the fix target described in
sim/analysis/known_issues.yaml (MPC_OVERSHOOT_SATURATION) and
logbook/2026-04-18-move5-overshoot-stall-and-plant-collapse.md.

The ZMQ IPC is mocked out so tests run without a live ZMQ socket.
"""

from __future__ import annotations

from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from controller.plant import PlantState


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

class FakeIPC:
    """Drop-in replacement for ``MpcTargetIPC`` that serves queued messages."""

    def __init__(self, *args, **kwargs):
        self._queue: list[tuple[bytes, dict]] = []
        self.closed = False

    def recv_all(self):
        out, self._queue = self._queue, []
        return out

    def close(self):
        self.closed = True

    # Test-only helper to enqueue messages the way the real bridge would
    def enqueue(self, topic: bytes, msg: dict):
        self._queue.append((topic, msg))


def _state(time_s: float, pose3: tuple, twist6: tuple) -> PlantState:
    """Build a PlantState with minimal fields for target-source testing."""
    return PlantState(
        leg_extensions_mm=np.zeros(6),
        leg_velocities_mmps=np.zeros(6),
        platform_pos_mm=np.array(pose3, dtype=float),
        platform_rot=np.zeros(3),
        platform_twist=np.array(twist6, dtype=float),
        time=time_s,
    )


@pytest.fixture
def source_with_ipc():
    """Instantiate ZmqTargetSource with IPC patched at its import site.

    ``v_max_mmps`` (the kinematic ceiling threaded to K1–K6) and
    ``clamp_start_twist_mmps`` (the FK-derived start-twist clipping bound)
    are distinct parameters as of the 2026-04-20 live-twist-trap fix.
    Default the fixture to ``v_max_mmps=140`` and ``clamp=140`` so the
    pre-existing twist-clamp tests continue to exercise the same behaviour
    under the new API.  Tests that care about the split override either.
    """
    with patch('controller.zmq_target.MpcTargetIPC', FakeIPC):
        from controller.zmq_target import ZmqTargetSource
        src = ZmqTargetSource(
            default_z_mm=170.0,
            v_max_mmps=140.0,
            clamp_start_twist_mmps=140.0,
        )
        yield src, src._ipc


# ---------------------------------------------------------------------------
# Topics (match jugglebot.motion.ipc)
# ---------------------------------------------------------------------------

from jugglebot.motion.ipc import TOPIC_MPC_TARGET, TOPIC_MPC_MODE



# NIGHTLY TIER — the MPC is operationally dormant (plans/active/refactor-2026-07.md
# Phase 3: jugglebot_launch.py no longer starts motor_guard/motion_bridge_node; the
# leg path is trajectory_node -> teensy_bridge_node -> the Teensy MAX_DEVIATION
# guard). The code is parked, not deleted, so this battery is parked with it: it
# runs nightly via tools/nightly_suite.sh and on `./run_tests.sh --full`, which is
# mandatory before any hardware sitting. Promotion back to per-commit is step 4 of
# the MPC revival.
pytestmark = pytest.mark.nightly


# ---------------------------------------------------------------------------
# Behaviour: ref_events emission and caching
# ---------------------------------------------------------------------------

class TestRefEventsEmission:
    def test_no_events_until_first_target(self, source_with_ipc):
        src, _ipc = source_with_ipc
        tc = src.update(0.0, _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0)))
        assert tc.ref_events is None

    def test_events_anchored_at_plant_pose_on_first_target(self, source_with_ipc):
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 220.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        state = _state(5.0, (12.3, -4.1, 169.0), (30.0, 0.0, 80.0, 0, 0, 0))
        tc = src.update(sim_time=5.0, state=state)

        assert tc.ref_events is not None and len(tc.ref_events) == 2
        # First event = synthetic warm-up at t=t_now with plant pose/twist
        ev0 = tc.ref_events[0]
        assert ev0.time == pytest.approx(5.0)
        np.testing.assert_allclose(ev0.pose[:3], [12.3, -4.1, 169.0])
        np.testing.assert_allclose(ev0.twist[:3], [30.0, 0.0, 80.0])
        # Final event lands on target
        np.testing.assert_allclose(tc.ref_events[-1].pose[:3], [0.0, 0.0, 220.0])

    def test_same_target_reuses_cache(self, source_with_ipc):
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        state1 = _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0))
        tc1 = src.update(0.0, state1)

        # Subsequent poll delivers the *same* target; plant has drifted
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        state2 = _state(0.1, (0, 0, 175), (0, 0, 40, 0, 0, 0))
        tc2 = src.update(0.1, state2)

        # Same events object — IPOPT warm-start is preserved across ticks
        assert tc2.ref_events is tc1.ref_events

    def test_distinct_target_rebuilds_with_fresh_state(self, source_with_ipc):
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        state1 = _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0))
        tc1 = src.update(0.0, state1)
        # W4b: ZmqTargetSource now returns the same TargetCommand each
        # tick (mutated in place).  Snapshot ref_events BEFORE the next
        # update so we can compare list identities.
        events1 = tc1.ref_events

        # Distinct target arrives while plant has drifted and is moving
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 220.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        state2 = _state(0.5, (0, 0, 185), (0, 0, 60, 0, 0, 0))
        tc2 = src.update(0.5, state2)

        assert tc2.ref_events is not events1
        ev0 = tc2.ref_events[0]
        # Rebuilt quintic starts at the *live* plant pose+twist at t=0.5
        assert ev0.time == pytest.approx(0.5)
        np.testing.assert_allclose(ev0.pose[:3], [0.0, 0.0, 185.0])
        np.testing.assert_allclose(ev0.twist[:3], [0.0, 0.0, 60.0])

    def test_disable_clears_events(self, source_with_ipc):
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        src.update(0.0, _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0)))
        assert src._cached_events is not None

        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'disabled'})
        src.poll()
        assert src._cached_events is None
        assert src.has_target is False


# ---------------------------------------------------------------------------
# Behaviour: twist clamp
# ---------------------------------------------------------------------------

class TestTwistClamp:
    def test_spiked_fk_twist_is_clipped(self, source_with_ipc):
        src, ipc = source_with_ipc  # v_max_mmps=140
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 220.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        # Simulate an FK twist spike (e.g. encoder glitch) of 500 mm/s
        state = _state(0.0, (0, 0, 170), (500, -500, 500, 0, 0, 0))
        tc = src.update(0.0, state)

        np.testing.assert_allclose(
            tc.ref_events[0].twist[:3], [140.0, -140.0, 140.0]
        )

    def test_clamp_none_disables(self, source_with_ipc):
        """clamp_start_twist_mmps=None ⇒ no clamping of start twist."""
        with patch('controller.zmq_target.MpcTargetIPC', FakeIPC):
            from controller.zmq_target import ZmqTargetSource
            src = ZmqTargetSource(
                default_z_mm=170.0,
                v_max_mmps=140.0,
                clamp_start_twist_mmps=None,
            )
            ipc = src._ipc
            ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
            ipc.enqueue(TOPIC_MPC_TARGET, {
                'target_pose': [0.0, 0.0, 220.0, 0.0, 0.0, 0.0],
                'source': 'gui',
            })
            state = _state(0.0, (0, 0, 170), (500, 0, 0, 0, 0, 0))
            tc = src.update(0.0, state)
            np.testing.assert_allclose(
                tc.ref_events[0].twist[:3], [500.0, 0.0, 0.0]
            )

    def test_clamp_independent_of_v_max(self):
        """Clamp value is independent of the kinematic v_max.

        The 2026-04-20 fix introduces ``max_ref_start_twist_mmps = 60`` as
        a distinct, tighter bound than ``max_leg_vel_mmps = 140``.  A
        fallback-driven plant runaway at ~130 mm/s sits below the kinematic
        ceiling but must NOT be honoured as a new ref's start twist.  The
        clamp must kick in at 60 even though v_max is 140.
        """
        with patch('controller.zmq_target.MpcTargetIPC', FakeIPC):
            from controller.zmq_target import ZmqTargetSource
            src = ZmqTargetSource(
                default_z_mm=170.0,
                v_max_mmps=140.0,              # kinematic ceiling
                clamp_start_twist_mmps=60.0,   # tighter clamp
            )
            ipc = src._ipc
            ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
            ipc.enqueue(TOPIC_MPC_TARGET, {
                'target_pose': [0.0, 0.0, 220.0, 0.0, 0.0, 0.0],
                'source': 'gui',
            })
            # Simulate a fallback-driven runaway at 130 mm/s (< v_max, so a
            # v_max-valued clamp would be a no-op — but the 60 mm/s clamp
            # must clip it).
            state = _state(0.0, (0, 0, 170), (0, 0, 130, 0, 0, 0))
            tc = src.update(0.0, state)
            np.testing.assert_allclose(
                tc.ref_events[0].twist[:3], [0.0, 0.0, 60.0]
            )


# ---------------------------------------------------------------------------
# Behaviour: change detection on repeated identical targets
# ---------------------------------------------------------------------------

class TestChangeDetection:
    def test_identical_retransmissions_do_not_thrash_cache(self, source_with_ipc):
        """Spacemouse-like streams replay identical targets; cache must survive."""
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'spacemouse'})
        target = [0.0, 0.0, 200.0, 0.0, 0.0, 0.0]
        ipc.enqueue(TOPIC_MPC_TARGET, {'target_pose': target, 'source': 'spacemouse'})

        state = _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0))
        tc_a = src.update(0.0, state)

        # Ten more identical transmissions arriving between solves
        for _ in range(10):
            ipc.enqueue(TOPIC_MPC_TARGET,
                        {'target_pose': target, 'source': 'spacemouse'})
        tc_b = src.update(0.025, state)

        assert tc_a.ref_events is tc_b.ref_events

    def test_arrival_time_only_change_does_not_rebuild_cache(self, source_with_ipc):
        """A stream sender refreshing only ``arrival_time`` must NOT trigger
        a cache rebuild.

        Rebuilding on each tick re-reads the live plant twist as the
        quintic's start-velocity boundary.  When IPOPT is in fallback and
        cmd is extrapolating, the plant is in runaway motion — and using
        that runaway twist as the new ref's start velocity self-seeds the
        next tick, producing positive-feedback overspeed (observed on
        mpc_20260419_223902.csv: actual_z reached 244 mm against ref_z of
        210, peak 318 mm/s, PLANT_TELEMETRY_COLLAPSE detector tripped).

        The scalar ``self._arrival_time`` is still updated and still
        forwarded via ``TargetCommand.arrival_time`` — only the ref-cache
        invalidation trigger is gated on pose/twist.
        """
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'spacemouse'})
        target = [0.0, 0.0, 200.0, 0.0, 0.0, 0.0]

        # Initial target with arrival=1.0
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': target,
            'arrival_time': 1.0,
            'source': 'spacemouse',
        })
        state_rest = _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0))
        tc_a = src.update(0.0, state_rest)
        events_a = tc_a.ref_events
        assert events_a is not None

        # Simulate plant entering runaway mid-move: new ticks arrive with
        # the same pose but a freshly refreshed arrival_time; plant twist
        # is non-zero.  Pre-fix, each such tick would rebuild the cache
        # and embed the runaway twist into the new quintic's start.
        state_runaway = _state(0.025, (0, 0, 175), (0, 0, 130, 0, 0, 0))
        for i in range(1, 11):
            ipc.enqueue(TOPIC_MPC_TARGET, {
                'target_pose': target,
                'arrival_time': 1.0 + i * 0.025,  # refreshed every tick
                'source': 'spacemouse',
            })
        tc_b = src.update(0.025, state_runaway)

        # Cache must be the same object — no rebuild fired.
        assert tc_b.ref_events is events_a

        # arrival_time IS still updated and propagated (scalar channel).
        assert src._arrival_time == pytest.approx(1.0 + 10 * 0.025)
        assert tc_b.arrival_time == pytest.approx(1.0 + 10 * 0.025)

    def test_pose_change_still_rebuilds_cache(self, source_with_ipc):
        """Regression check: after the arrival-time gating fix, a genuine
        pose change must still rebuild the cache against the live plant
        state."""
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'spacemouse'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'arrival_time': 1.0, 'source': 'spacemouse',
        })
        tc_a = src.update(0.0, _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0)))
        # W4b: snapshot the ref_events list reference before the next
        # update overwrites the pre-allocated TargetCommand's attribute.
        events_a = tc_a.ref_events

        # New pose
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 210.0, 0.0, 0.0, 0.0],
            'arrival_time': 1.0, 'source': 'spacemouse',
        })
        tc_b = src.update(0.025, _state(0.025, (0, 0, 175), (0, 0, 50, 0, 0, 0)))

        assert tc_b.ref_events is not events_a


# ---------------------------------------------------------------------------
# Regression: Move-5 overshoot geometry — the MPC's ref[0] must match plant
# state at the first solve after a new target arrives with residual motion.
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# W11 — catch rejection feedback + stretch-warning publication
# ---------------------------------------------------------------------------

class _FakeFeedbackPub:
    """Captures TargetFeedback messages for assertion — no ZMQ."""

    def __init__(self):
        self.sent: list[dict] = []
        self.closed = False

    def send(self, msg: dict):
        self.sent.append(msg)

    def close(self):
        self.closed = True


class TestW11CatchRejection:
    """A catch target that would need stretching must be rejected — not stretched.

    The W11 contract: make_feasible_events(no_stretch=True) returns a reason
    string; ZmqTargetSource keeps the previously-cached events and publishes
    a 'rejected_infeasible' feedback message.  Catch coordinator consumes
    accepted=False via its existing path.
    """

    def _make_src_with_fb(self):
        fb = _FakeFeedbackPub()
        with patch('controller.zmq_target.MpcTargetIPC', FakeIPC):
            from controller.zmq_target import ZmqTargetSource
            src = ZmqTargetSource(
                default_z_mm=170.0, v_max_mmps=140.0, tau_s=0.04,
                clamp_start_twist_mmps=60.0,
                feedback_pub=fb,
            )
        return src, src._ipc, fb

    def test_catch_reject_publishes_rejected_infeasible(self):
        src, ipc, fb = self._make_src_with_fb()
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'catch'})
        # Catch target: plant falling (-80 mm/s), want to rise to z=260 in 0.2s
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 260.0, 0.0, 0.0, 0.0],
            'target_twist': [0.0, 0.0, 120.0, 0.0, 0.0, 0.0],
            'arrival_time': 0.2,
            'source': 'catch',
        })
        state = _state(0.0, (0, 0, 200), (0, 0, -80, 0, 0, 0))
        tc = src.update(0.0, state)
        # No previous feasible cache → cached_events remains None after rejection
        assert src._cached_events is None
        # Rejection feedback was published
        assert len(fb.sent) == 1
        msg = fb.sent[0]
        assert msg['feedback_type'] == 'rejected_infeasible'
        assert msg['accepted'] is False
        assert msg['arrival_time'] == 0.2
        assert 'peak_exceeds_no_stretch_allowed' in msg['reason']
        assert msg['source'] == 'catch'

    def test_catch_reject_preserves_last_feasible_events(self):
        """A rejected catch after a previously-feasible target keeps the old quintic."""
        src, ipc, fb = self._make_src_with_fb()
        # First: a feasible catch at reasonable speed
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'catch'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'target_twist': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'arrival_time': 1.0,
            'source': 'catch',
        })
        state = _state(0.0, (0, 0, 190), (0, 0, 0, 0, 0, 0))
        tc1 = src.update(0.0, state)
        feasible_events = tc1.ref_events
        assert feasible_events is not None

        # Now: an infeasible catch
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 280.0, 0.0, 0.0, 0.0],
            'target_twist': [0.0, 0.0, 150.0, 0.0, 0.0, 0.0],
            'arrival_time': 1.1,
            'source': 'catch',
        })
        state2 = _state(0.1, (0, 0, 192), (0, 0, -100, 0, 0, 0))
        tc2 = src.update(0.1, state2)

        # Cache UNCHANGED (same events object)
        assert src._cached_events is feasible_events
        # Rejection sent
        assert any(m['feedback_type'] == 'rejected_infeasible' for m in fb.sent)

    def test_catch_reject_dedup_by_arrival_time(self):
        """Same infeasible arrival retransmitted → only one rejection message."""
        src, ipc, fb = self._make_src_with_fb()
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'catch'})
        for _ in range(5):
            ipc.enqueue(TOPIC_MPC_TARGET, {
                'target_pose': [0.0, 0.0, 260.0, 0.0, 0.0, 0.0],
                'target_twist': [0.0, 0.0, 120.0, 0.0, 0.0, 0.0],
                'arrival_time': 0.2,
                'source': 'catch',
            })
            src.update(0.0, _state(0.0, (0, 0, 200), (0, 0, -80, 0, 0, 0)))
        rejects = [m for m in fb.sent
                   if m.get('feedback_type') == 'rejected_infeasible']
        assert len(rejects) == 1, f"dedupe: expected 1, got {len(rejects)}"

    def test_stuck_infeasible_catch_no_per_tick_recompute(self):
        """A stuck infeasible catch (no prior feasible cache) MUST NOT re-fire
        ``flat_target_to_events`` on every tick.  Without the
        ``_last_rejected_request`` fingerprint the entry guard's
        ``_cached_events is None`` clause re-runs the full K1–K6 +
        binary-search stretch attempt at 40 Hz indefinitely.
        """
        from unittest.mock import patch as _patch
        src, ipc, fb = self._make_src_with_fb()
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'catch'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 260.0, 0.0, 0.0, 0.0],
            'target_twist': [0.0, 0.0, 120.0, 0.0, 0.0, 0.0],
            'arrival_time': 0.2,
            'source': 'catch',
        })
        state = _state(0.0, (0, 0, 200), (0, 0, -80, 0, 0, 0))
        # Wrap the real flat_target_to_events so we can count its invocations
        # but still get the real rejection behaviour.
        from controller import zmq_target as _zt
        real = _zt.flat_target_to_events
        with _patch.object(_zt, 'flat_target_to_events',
                           wraps=real) as spy:
            for _ in range(10):
                src.update(0.0, state)
        assert spy.call_count == 1, (
            f"stuck infeasible catch re-fired flat_target_to_events "
            f"{spy.call_count} times across 10 ticks (expected 1)"
        )


class TestW11StretchWarning:
    """Non-catch targets that need stretching are stretched silently at the
    solver level but announced via a 'stretch_warning' feedback message.
    The existing catch-coordinator consumer reads accepted=True on this
    message (the catch *will* still arrive, just later), so backward-compat
    is preserved."""

    def _make_src_with_fb(self):
        fb = _FakeFeedbackPub()
        with patch('controller.zmq_target.MpcTargetIPC', FakeIPC):
            from controller.zmq_target import ZmqTargetSource
            src = ZmqTargetSource(
                default_z_mm=170.0, v_max_mmps=140.0, tau_s=0.04,
                clamp_start_twist_mmps=60.0,
                feedback_pub=fb,
            )
        return src, src._ipc, fb

    def test_non_catch_stretch_publishes_warning(self):
        src, ipc, fb = self._make_src_with_fb()
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            # Infeasible without stretch, but gui-source = no_stretch=False
            'target_pose': [0.0, 0.0, 170.0, 0.0, 0.0, 0.0],
            'target_twist': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'arrival_time': 0.3,  # reach 170 from 210 in 0.3s with vz=+60
            'source': 'gui',
        })
        state = _state(0.0, (0, 0, 210), (0, 0, 60, 0, 0, 0))
        tc = src.update(0.0, state)
        # Stretched events
        assert tc.ref_events is not None
        assert tc.ref_events[-1].time > 0.3 + 1e-6
        # Stretch-warning sent with accepted=True (backward compat)
        warns = [m for m in fb.sent if m.get('feedback_type') == 'stretch_warning']
        assert len(warns) == 1
        assert warns[0]['accepted'] is True
        assert warns[0]['stretch_ms'] > 1.0

    def test_no_warning_when_no_stretch_needed(self):
        src, ipc, fb = self._make_src_with_fb()
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 175.0, 0.0, 0.0, 0.0],
            'arrival_time': 1.5,
            'source': 'gui',
        })
        src.update(0.0, _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0)))
        assert not any(m.get('feedback_type') == 'stretch_warning'
                       for m in fb.sent)


# ---------------------------------------------------------------------------
# W5 — warm_start_valid invalidation on structural ref shift
# ---------------------------------------------------------------------------

class TestW5WarmStartInvalidation:
    """Source detects large ref-horizon-end shifts and flags the MPC
    to cold-start instead of warm-starting from the stale plan."""

    def test_small_shift_keeps_warm_start_valid(self, source_with_ipc):
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        # Target 1
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 195.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        tc1 = src.update(0.0, _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0)))
        # First target → no prev ref → warm_start_valid defaults True
        assert tc1.warm_start_valid is True

        # Target 2 — only 5 mm shift (below 20 mm threshold)
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        tc2 = src.update(0.1, _state(0.1, (0, 0, 175), (0, 0, 40, 0, 0, 0)))
        assert tc2.warm_start_valid is True

    def test_large_shift_invalidates_warm_start(self, source_with_ipc):
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        # Target 1 at z=200
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        src.update(0.0, _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0)))

        # Target 2 at z=250 — 50 mm shift, above 20 mm threshold
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 250.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        tc2 = src.update(0.1, _state(0.1, (0, 0, 175), (0, 0, 40, 0, 0, 0)))
        assert tc2.warm_start_valid is False

    def test_invalidation_is_one_tick_latch(self, source_with_ipc):
        """After one invalidation, subsequent ticks (same target) are valid."""
        src, ipc = source_with_ipc
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        src.update(0.0, _state(0.0, (0, 0, 170), (0, 0, 0, 0, 0, 0)))

        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 250.0, 0.0, 0.0, 0.0],
            'source': 'gui',
        })
        tc_a = src.update(0.1, _state(0.1, (0, 0, 175), (0, 0, 0, 0, 0, 0)))
        assert tc_a.warm_start_valid is False

        # Next tick, no new target — warm_start trivially valid
        tc_b = src.update(0.125, _state(0.125, (0, 0, 180), (0, 0, 0, 0, 0, 0)))
        assert tc_b.warm_start_valid is True


class TestW11TerminalHold:
    """After the last feasible quintic's terminal time, ZmqTargetSource
    emits a hold event at the terminal pose (Day-2 decision: 'continuance of
    the last feasible move; once that's done, hold position')."""

    def _make_src_with_fb(self):
        fb = _FakeFeedbackPub()
        with patch('controller.zmq_target.MpcTargetIPC', FakeIPC):
            from controller.zmq_target import ZmqTargetSource
            src = ZmqTargetSource(
                default_z_mm=170.0, v_max_mmps=140.0, tau_s=0.04,
                clamp_start_twist_mmps=60.0,
                feedback_pub=fb,
            )
        return src, src._ipc, fb

    def test_terminal_hold_after_quintic_expires(self):
        src, ipc, fb = self._make_src_with_fb()
        ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'catch'})
        ipc.enqueue(TOPIC_MPC_TARGET, {
            'target_pose': [0.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            'arrival_time': 0.5,
            'source': 'catch',
        })
        src.update(0.0, _state(0.0, (0, 0, 190), (0, 0, 0, 0, 0, 0)))
        # Now advance past the quintic's terminal time
        tc_late = src.update(
            1.5, _state(1.5, (0, 0, 200), (0, 0, 0, 0, 0, 0)))
        assert tc_late.ref_events is not None
        # Hold event at terminal pose with zero twist
        assert len(tc_late.ref_events) == 1
        np.testing.assert_allclose(
            tc_late.ref_events[0].pose[:3], [0.0, 0.0, 200.0], atol=1e-6,
        )
        np.testing.assert_allclose(
            tc_late.ref_events[0].twist, np.zeros(6), atol=1e-6,
        )


class TestOvershootSaturationRegression:
    """Reproduce the Move-5 initial condition and verify Bundle A's invariant.

    Move 5 (CSV mpc_20260418_015112) started from (8.57, 34.0, 169.3) with
    residual upward motion carried over from Move 4; a new target of
    (0, 0, 220) arriving in that state produced 50 consecutive ipopt_iter==0
    steps because the solver was asked to unwind a 20 mm lead against an
    accelerating ref within 22 ms.

    The Bundle A fix anchors the ref at the live plant pose+twist at the
    moment the target arrives, so ref[0] aligns with the plant and the NLP
    is never given the "unwind vs accelerating ref" geometry.
    """

    def test_ref_aligns_with_plant_at_first_solve(self):
        from plant.mujoco_plant import MuJoCoPlant
        from controller.mpc import MPCController
        from controller.params import MPCParams

        plant = MuJoCoPlant()
        plant.reset(pose_6dof=np.array([8.57, 34.0, -0.7, 0, 0, 0]))
        # Let MuJoCo settle the ctrl into qpos so get_state() is consistent
        plant.step(0.025)
        state = plant.get_state()

        # Seed a residual-motion plant state by spoofing platform_twist —
        # the geometry is what matters for this regression test, not the
        # physics of how the twist was acquired.
        state.platform_twist = np.array([0.0, 0.0, 80.0, 0.0, 0.0, 0.0])

        with patch('controller.zmq_target.MpcTargetIPC', FakeIPC):
            from controller.zmq_target import ZmqTargetSource
            src = ZmqTargetSource(default_z_mm=170.0, v_max_mmps=140.0)
            src._ipc.enqueue(TOPIC_MPC_MODE, {'mode': 'gui'})
            src._ipc.enqueue(TOPIC_MPC_TARGET, {
                'target_pose': [0.0, 0.0, 50.0, 0.0, 0.0, 0.0],
                'source': 'gui',
            })
            tc = src.update(sim_time=state.time, state=state)

        # Build a one-shot MPC and inspect ref_traj[0] — it must match the
        # plant pose (the Bundle A invariant).  Without the fix, ref_events
        # would be None and ref_traj[0] would be the target pose — a 50 mm
        # step away from the plant.
        mpc = MPCController.from_plant(
            MPCParams(max_cpu_time=2.0, max_iter=500, prime_solver=False),
            plant,
        )
        ref, twist, _ = mpc._build_reference(
            state, tc.target_pose, ref_events=tc.ref_events, t_now=state.time,
        )
        plant_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])

        # ref[0] is anchored at plant pose (synthetic warm-up event)
        np.testing.assert_allclose(ref[0], plant_pose, atol=1e-6)
        # ref_twist[0] carries the (clamped) plant twist, not zero
        np.testing.assert_allclose(twist[0, :3], [0.0, 0.0, 80.0], atol=1e-6)
