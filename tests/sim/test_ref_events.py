"""Tests for event-based MPC reference trajectories.

Validates:
  - _build_reference_from_events: Hermite interpolation, backward extrapolation, hold
  - ReferenceEvent / sample_ref_fn: dataclass and convenience adapter
  - MPC solve with ref_events: velocity tracking
  - Backward compatibility: ref_events=None is identical to legacy path
"""

from __future__ import annotations

import numpy as np
import pytest

from controller.mpc import (
    MPCController, _hermite_interp, _build_reference_from_events,
    _quintic_interp_events,
)
from controller.params import MPCParams
from controller.target import ReferenceEvent, sample_ref_fn
from plant.mujoco_plant import MuJoCoPlant

CONTROL_DT = 0.02  # 50 Hz


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _create_mpc(plant, **param_overrides):
    defaults = dict(max_cpu_time=2.0, max_iter=500)
    defaults.update(param_overrides)
    params = MPCParams(**defaults)
    return MPCController.from_plant(params, plant)


def _run_mpc_with_events(plant, mpc, ref_events_fn, duration_s):
    """Run MPC loop with event-based references.

    ref_events_fn(state) -> list[ReferenceEvent] is called each step to
    provide time-varying references.
    """
    n_steps = int(duration_s / CONTROL_DT)
    states = []
    for _ in range(n_steps):
        state = plant.get_state()
        events = ref_events_fn(state)
        target_pose = events[0].pose if events else np.zeros(6)
        cmd, cmd_vel, diag = mpc.solve(
            state, target_pose, ref_events=events)
        plant.command(cmd, vel_mm_s=cmd_vel)
        plant.step(CONTROL_DT)
        pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        states.append({
            'time': state.time,
            'pose': pose.copy(),
            'twist': state.platform_twist.copy(),
            'ext': state.leg_extensions_mm.copy(),
            'cmd': cmd.copy(),
            'diag': diag,
        })
    return states


@pytest.fixture
def plant():
    return MuJoCoPlant()


@pytest.fixture
def mpc(plant):
    return _create_mpc(plant)


# ---------------------------------------------------------------------------
# Unit tests: _hermite_interp
# ---------------------------------------------------------------------------

class TestHermiteInterp:
    """Unit tests for the cubic Hermite interpolation helper."""

    def test_endpoints_match(self):
        """Hermite interpolation returns exact values at endpoints."""
        p0 = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        p1 = np.array([10.0, 0.0, 80.0, 0.0, 0.0, 0.0])
        v0 = np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        v1 = np.array([0.0, 0.0, -100.0, 0.0, 0.0, 0.0])
        t0, t1 = 1.0, 2.0

        pose_at_t0, twist_at_t0 = _hermite_interp(t0, t0, p0, v0, t1, p1, v1)
        pose_at_t1, twist_at_t1 = _hermite_interp(t1, t0, p0, v0, t1, p1, v1)

        np.testing.assert_allclose(pose_at_t0, p0, atol=1e-10)
        np.testing.assert_allclose(pose_at_t1, p1, atol=1e-10)
        np.testing.assert_allclose(twist_at_t0, v0, atol=1e-10)
        np.testing.assert_allclose(twist_at_t1, v1, atol=1e-10)

    def test_midpoint_between_static_poses(self):
        """For two static poses (zero twist), midpoint is the average."""
        p0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        p1 = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        v0 = v1 = np.zeros(6)

        pose_mid, _ = _hermite_interp(0.5, 0.0, p0, v0, 1.0, p1, v1)

        np.testing.assert_allclose(pose_mid, p1 * 0.5, atol=1e-10)

    def test_linear_motion(self):
        """With matching linear velocities, interpolation is linear."""
        p0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        vel = np.array([100.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        p1 = p0 + vel * 1.0  # 1 second at constant velocity
        t0, t1 = 0.0, 1.0

        for t in [0.0, 0.25, 0.5, 0.75, 1.0]:
            pose, twist = _hermite_interp(t, t0, p0, vel, t1, p1, vel)
            expected_pose = p0 + vel * t
            np.testing.assert_allclose(pose, expected_pose, atol=1e-10)
            np.testing.assert_allclose(twist, vel, atol=1e-10)


# ---------------------------------------------------------------------------
# Unit tests: _build_reference_from_events
# ---------------------------------------------------------------------------

class TestBuildReferenceFromEvents:
    """Tests for the event-based reference builder."""

    def _default_cumulative(self):
        return MPCParams().cumulative_times

    def test_single_event_hold(self):
        """Single event in the past: all nodes hold at event pose."""
        pose = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        events = [ReferenceEvent(time=0.5, pose=pose)]
        ct = self._default_cumulative()
        t_now = 1.0  # well past the event

        ref, twist, _accel = _build_reference_from_events(events, t_now, ct, len(ct))

        # All nodes after the event should hold at event pose
        for k in range(len(ct)):
            np.testing.assert_allclose(ref[k], pose, atol=1e-10)
            np.testing.assert_allclose(twist[k], np.zeros(6), atol=1e-10)

    def test_single_event_with_twist_extrapolates(self):
        """Single event with twist: nodes after event extrapolate linearly."""
        pose = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        tw = np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0])
        events = [ReferenceEvent(time=1.0, pose=pose, twist=tw)]
        ct = self._default_cumulative()
        t_now = 1.0  # exactly at event time

        ref, twist_out, _accel = _build_reference_from_events(events, t_now, ct, len(ct))

        # Node 0 is at t_now = event.time, so it should be exactly at pose
        np.testing.assert_allclose(ref[0], pose, atol=1e-10)

        # Later nodes extrapolate: pose + twist * dt
        for k in range(1, len(ct)):
            dt = ct[k]
            expected = pose + tw * dt
            np.testing.assert_allclose(ref[k], expected, atol=1e-8)
            np.testing.assert_allclose(twist_out[k], tw, atol=1e-10)

    def test_backward_extrapolation(self):
        """Nodes before first event use linear backward extrapolation."""
        pose = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        tw = np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0])
        events = [ReferenceEvent(time=2.0, pose=pose, twist=tw)]
        ct = self._default_cumulative()
        t_now = 0.5  # all nodes are before the event

        ref, twist_out, _accel = _build_reference_from_events(events, t_now, ct, len(ct))

        # All nodes should be backward-extrapolated
        for k in range(len(ct)):
            t_k = t_now + ct[k]
            dt_back = t_k - 2.0  # negative
            expected = pose + tw * dt_back
            np.testing.assert_allclose(ref[k], expected, atol=1e-8)
            np.testing.assert_allclose(twist_out[k], tw, atol=1e-10)

    def test_two_events_hermite(self):
        """Two events: nodes between them use cubic Hermite interpolation."""
        p0 = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        p1 = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        v0 = np.zeros(6)
        v1 = np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0])
        events = [
            ReferenceEvent(time=0.0, pose=p0, twist=v0),
            ReferenceEvent(time=1.0, pose=p1, twist=v1),
        ]
        ct = self._default_cumulative()
        t_now = 0.0

        ref, twist_out, _accel = _build_reference_from_events(events, t_now, ct, len(ct))

        # Node 0 (t=0) is at first event
        np.testing.assert_allclose(ref[0], p0, atol=1e-10)
        np.testing.assert_allclose(twist_out[0], v0, atol=1e-10)

        # Intermediate nodes should be between p0 and p1 (z between 60 and 100)
        for k in range(1, len(ct)):
            t_k = ct[k]
            if t_k < 1.0:
                # Between events: z should be between 60 and 100
                assert 55.0 <= ref[k][2] <= 105.0, \
                    f"Node {k} z={ref[k][2]:.1f} out of range"
            else:
                # After event 1: extrapolated from p1 with v1
                dt_fwd = t_k - 1.0
                expected_z = p1[2] + v1[2] * dt_fwd
                np.testing.assert_allclose(ref[k][2], expected_z, atol=1e-6)

    def test_continuity_at_event_boundary(self):
        """Reference is continuous (no jumps) at event times."""
        p0 = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        p1 = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        v0 = np.array([0.0, 0.0, -50.0, 0.0, 0.0, 0.0])
        v1 = np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0])
        events = [
            ReferenceEvent(time=0.0, pose=p0, twist=v0),
            ReferenceEvent(time=0.5, pose=p1, twist=v1),
        ]
        # Use a fine uniform schedule to get a node close to t=0.5
        params = MPCParams(dt_schedule=MPCParams.uniform_schedule(20, 0.05))
        ct = params.cumulative_times

        ref, tw, _accel = _build_reference_from_events(events, 0.0, ct, len(ct))

        # Find the node closest to t=0.5 from below and above
        idx_before = np.searchsorted(ct, 0.5) - 1
        idx_after = idx_before + 1
        if idx_after < len(ct):
            # The jump between adjacent nodes should be small
            jump = np.linalg.norm(ref[idx_after] - ref[idx_before])
            assert jump < 20.0, \
                f"Discontinuity at event boundary: {jump:.1f} mm"


# ---------------------------------------------------------------------------
# Unit tests: sample_ref_fn
# ---------------------------------------------------------------------------

class TestSampleRefFn:
    """Tests for the ref_fn → ReferenceEvent adapter."""

    def test_basic_sampling(self):
        """sample_ref_fn produces correct number of events at correct times."""
        ct = MPCParams().cumulative_times
        t_now = 5.0

        def ref_fn(t):
            pose = np.array([0.0, 0.0, t * 10.0, 0.0, 0.0, 0.0])
            twist = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
            return pose, twist

        events = sample_ref_fn(ref_fn, t_now, ct)

        assert len(events) == len(ct)
        for i, ev in enumerate(events):
            expected_t = t_now + ct[i]
            assert abs(ev.time - expected_t) < 1e-12
            np.testing.assert_allclose(ev.pose[2], expected_t * 10.0, atol=1e-8)
            np.testing.assert_allclose(ev.twist[2], 10.0, atol=1e-12)


# ---------------------------------------------------------------------------
# Integration tests: MPC solve with ref_events
# ---------------------------------------------------------------------------

class TestMPCSolveWithEvents:
    """Integration tests for the MPC with event-based references."""

    def test_single_event_matches_legacy(self, plant, mpc):
        """A single-event reference at active pose produces similar behavior to
        legacy flat reference (both should hold near active pose)."""
        state = plant.get_state()
        active_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])

        # Legacy solve (no events)
        cmd_legacy, _, diag_legacy = mpc.solve(state, active_pose)

        # Reset MPC state for fair comparison
        mpc_events = _create_mpc(plant)
        state = plant.get_state()
        events = [ReferenceEvent(time=state.time, pose=active_pose)]
        cmd_events, _, diag_events = mpc_events.solve(
            state, active_pose, ref_events=events)

        # Both should produce similar commands (at active pose, both are near-zero change)
        np.testing.assert_allclose(cmd_legacy, cmd_events, atol=2.0,
                                   err_msg="Event-based and legacy commands differ significantly")

    def test_two_event_trajectory_tracking(self, plant):
        """MPC tracks a two-event trajectory through time."""
        mpc = _create_mpc(plant, max_leg_vel_mmps=1000.0)
        state = plant.get_state()
        active_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])
        target_z = 50.0  # mm above active pose
        target_pose = active_pose.copy()
        target_pose[2] += target_z

        t_start = state.time

        def make_events(st):
            """Two events: start at active pose, arrive at target_pose in 0.5s."""
            return [
                ReferenceEvent(
                    time=t_start,
                    pose=active_pose.copy(),
                    twist=np.zeros(6),
                ),
                ReferenceEvent(
                    time=t_start + 0.5,
                    pose=target_pose.copy(),
                    twist=np.zeros(6),
                ),
            ]

        states = _run_mpc_with_events(plant, mpc, make_events, duration_s=1.0)

        # Platform should be near target after 1.0s
        final_pose = states[-1]['pose']
        pos_err = np.linalg.norm(final_pose[:3] - target_pose[:3])
        assert pos_err < 3.0, \
            f"Position error {pos_err:.1f} mm after 1.0s (threshold 3.0 mm)"

    def test_velocity_at_event_with_twist(self, plant):
        """MPC achieves nonzero velocity when the reference encodes it.

        This is the key test for Issue 2: with event-based references that
        encode velocity, the MPC should actually track it — unlike the
        legacy flat reference where velocity tracking was structurally defeated.

        Uses default Q_vel_lin=0.05 and Qf_vel_lin=0.5 (tuned in R3).
        """
        mpc = _create_mpc(plant, max_leg_vel_mmps=1000.0)
        state = plant.get_state()
        active_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])

        # Target: arrive at z=active+30mm with vz=-100mm/s at t=0.8s
        target_pose = active_pose.copy()
        target_pose[2] += 30.0
        target_vz = -100.0  # mm/s downward
        target_twist = np.array([0.0, 0.0, target_vz, 0.0, 0.0, 0.0])

        t_start = state.time
        t_event = t_start + 0.8

        def make_events(st):
            """Approach from above: backward extrapolation gives higher z at earlier nodes."""
            return [
                ReferenceEvent(
                    time=t_event,
                    pose=target_pose.copy(),
                    twist=target_twist.copy(),
                ),
            ]

        states = _run_mpc_with_events(plant, mpc, make_events, duration_s=1.0)

        # Check velocity near the event time
        window = [s for s in states if abs(s['time'] - t_event) < 0.15]
        assert len(window) > 0, "No states near event time"

        vz_values = [s['twist'][2] for s in window]
        min_vz = min(vz_values)

        # With event-based references and tuned velocity weights (R3),
        # the MPC should achieve meaningful downward velocity.  The target
        # is -100 mm/s; actuator lag (τ=30ms) limits tracking but the
        # direction and magnitude should be clearly present.
        assert min_vz < -25.0, (
            f"Expected meaningful downward velocity near event, "
            f"got min vz={min_vz:.1f} mm/s (target: {target_vz} mm/s)"
        )


class TestColdStartWithEvents:
    """Tests for cold start with time-varying references."""

    def test_cold_start_uses_per_node_refs(self, plant):
        """Cold start interpolates toward each node's reference, not just final."""
        mpc = _create_mpc(plant)
        state = plant.get_state()
        active_pose = np.concatenate([state.platform_pos_mm, state.platform_rot])

        # Two-event reference: different targets at early vs late nodes
        target_near = active_pose.copy()
        target_near[2] += 10.0
        target_far = active_pose.copy()
        target_far[2] += 50.0

        events = [
            ReferenceEvent(time=state.time + 0.05, pose=target_near),
            ReferenceEvent(time=state.time + 1.0, pose=target_far),
        ]

        # First solve forces cold start (no previous solution)
        cmd, _, diag = mpc.solve(state, active_pose, ref_events=events)

        assert diag['status'] in ('Solve_Succeeded', 'Solved_To_Acceptable_Level'), \
            f"Cold start with events failed: {diag['status']}"


# ---------------------------------------------------------------------------
# Unit tests: quintic interpolation and C2 upgrade
# ---------------------------------------------------------------------------

class TestQuinticInterpEvents:
    """Tests for the quintic Hermite interpolation wrapper."""

    def test_endpoints_match(self):
        """Quintic interpolation returns exact values at endpoints."""
        p0 = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        p1 = np.array([10.0, 0.0, 80.0, 0.0, 0.0, 0.0])
        v0 = np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        v1 = np.array([0.0, 0.0, -100.0, 0.0, 0.0, 0.0])
        a0 = np.array([0.0, 0.0, 5.0, 0.0, 0.0, 0.0])
        a1 = np.array([0.0, 0.0, -10.0, 0.0, 0.0, 0.0])
        t0, t1 = 1.0, 2.0

        pose_0, twist_0, accel_0 = _quintic_interp_events(
            t0, t0, p0, v0, a0, t1, p1, v1, a1)
        pose_1, twist_1, accel_1 = _quintic_interp_events(
            t1, t0, p0, v0, a0, t1, p1, v1, a1)

        np.testing.assert_allclose(pose_0, p0, atol=1e-10)
        np.testing.assert_allclose(pose_1, p1, atol=1e-10)
        np.testing.assert_allclose(twist_0, v0, atol=1e-10)
        np.testing.assert_allclose(twist_1, v1, atol=1e-10)
        np.testing.assert_allclose(accel_0, a0, atol=1e-10)
        np.testing.assert_allclose(accel_1, a1, atol=1e-10)

    def test_degenerate_interval(self):
        """Zero-duration interval returns endpoint values."""
        p0 = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
        p1 = np.array([10.0, 0.0, 80.0, 0.0, 0.0, 0.0])
        v1 = np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        a1 = np.array([0.0, 0.0, -5.0, 0.0, 0.0, 0.0])

        pose, twist, accel = _quintic_interp_events(
            1.0, 1.0, p0, np.zeros(6), np.zeros(6),
            1.0, p1, v1, a1)

        np.testing.assert_allclose(pose, p1, atol=1e-10)
        np.testing.assert_allclose(twist, v1, atol=1e-10)
        np.testing.assert_allclose(accel, a1, atol=1e-10)


class TestQuinticReferenceBuilder:
    """Tests for quintic upgrade in _build_reference_from_events."""

    def _default_cumulative(self):
        return MPCParams().cumulative_times

    def test_asymmetric_velocity_overshoot(self):
        """Quintic (with accel) has less overshoot than cubic on asymmetric velocities.

        This is the key regression test: throw at 250 mm/s, catch at 0 mm/s.
        Cubic Hermite overshoots significantly; quintic with zero-accel
        boundaries should stay much closer to the straight-line path.
        """
        p0 = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        p1 = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        v0 = np.array([0.0, 0.0, -250.0, 0.0, 0.0, 0.0])  # throw velocity
        v1 = np.zeros(6)  # catch at rest
        a_zero = np.zeros(6)

        # Cubic events (no accel)
        events_cubic = [
            ReferenceEvent(time=0.0, pose=p0.copy(), twist=v0.copy()),
            ReferenceEvent(time=0.5, pose=p1.copy(), twist=v1.copy()),
        ]
        # Quintic events (with accel=0 at both ends)
        events_quintic = [
            ReferenceEvent(time=0.0, pose=p0.copy(), twist=v0.copy(), accel=a_zero.copy()),
            ReferenceEvent(time=0.5, pose=p1.copy(), twist=v1.copy(), accel=a_zero.copy()),
        ]

        # Dense uniform schedule for fine sampling
        params = MPCParams(dt_schedule=MPCParams.uniform_schedule(50, 0.01))
        ct = params.cumulative_times

        ref_cubic, _, _ = _build_reference_from_events(
            events_cubic, 0.0, ct, len(ct))
        ref_quintic, _, _ = _build_reference_from_events(
            events_quintic, 0.0, ct, len(ct))

        # Measure overshoot: z should stay between 60 and 100
        cubic_min_z = ref_cubic[:, 2].min()
        quintic_min_z = ref_quintic[:, 2].min()

        cubic_overshoot = max(0, 60.0 - cubic_min_z)
        quintic_overshoot = max(0, 60.0 - quintic_min_z)

        # Quintic overshoot should be much less than cubic
        assert quintic_overshoot < cubic_overshoot * 0.3 + 1.0, (
            f"Quintic overshoot {quintic_overshoot:.1f} mm not sufficiently "
            f"better than cubic {cubic_overshoot:.1f} mm"
        )

        # Quintic peak velocity should be bounded by the boundary velocities.
        # Cubic can produce velocity peaks above the max boundary velocity (250)
        # due to overshoot, while quintic with zero-accel boundaries decelerates
        # monotonically.  Check that quintic doesn't exceed boundary speed + margin.
        quintic_vel = np.abs(ref_quintic[1:, 2] - ref_quintic[:-1, 2]) / 0.01
        assert quintic_vel.max() < 260.0, (
            f"Quintic peak vel {quintic_vel.max():.0f} mm/s exceeds "
            f"boundary velocity 250 mm/s by too much"
        )

    def test_fallback_to_cubic_without_accel(self):
        """Events without accel produce identical results to cubic path."""
        p0 = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        p1 = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        v0 = np.array([0.0, 0.0, -50.0, 0.0, 0.0, 0.0])
        v1 = np.zeros(6)

        events = [
            ReferenceEvent(time=0.0, pose=p0, twist=v0),
            ReferenceEvent(time=1.0, pose=p1, twist=v1),
        ]
        ct = self._default_cumulative()

        ref, twist_out, _ = _build_reference_from_events(events, 0.0, ct, len(ct))

        # Manually compute expected cubic at each node
        for k in range(len(ct)):
            t_k = ct[k]
            if t_k <= 0.0:
                expected_pose, expected_twist = p0.copy(), v0.copy()
            elif t_k >= 1.0:
                dt_fwd = t_k - 1.0
                expected_pose = p1 + v1 * dt_fwd
                expected_twist = v1.copy()
            else:
                expected_pose, expected_twist = _hermite_interp(
                    t_k, 0.0, p0, v0, 1.0, p1, v1)

            np.testing.assert_allclose(ref[k], expected_pose, atol=1e-10,
                                       err_msg=f"Pose mismatch at node {k}")
            np.testing.assert_allclose(twist_out[k], expected_twist, atol=1e-10,
                                       err_msg=f"Twist mismatch at node {k}")

    def test_quadratic_extrapolation_continuity(self):
        """Quadratic extrapolation is C2-continuous at the event boundary."""
        pose = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        twist = np.array([0.0, 0.0, -200.0, 0.0, 0.0, 0.0])
        accel = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])

        events = [ReferenceEvent(time=1.0, pose=pose, twist=twist, accel=accel)]

        # Fine schedule to sample close to the event time
        params = MPCParams(dt_schedule=MPCParams.uniform_schedule(100, 0.01))
        ct = params.cumulative_times
        t_now = 0.5  # nodes span [0.5, 1.5]

        ref, tw_out, acc_out = _build_reference_from_events(
            events, t_now, ct, len(ct))

        # Find node just before and just after event time=1.0
        times = t_now + ct
        idx_before = np.searchsorted(times, 1.0) - 1
        idx_after = idx_before + 1

        # Position should be continuous
        pos_jump = np.linalg.norm(ref[idx_after] - ref[idx_before])
        assert pos_jump < 5.0, f"Position discontinuity: {pos_jump:.3f} mm"

        # Velocity should be continuous
        vel_jump = np.linalg.norm(tw_out[idx_after] - tw_out[idx_before])
        assert vel_jump < 10.0, f"Velocity discontinuity: {vel_jump:.3f} mm/s"

        # Acceleration should be continuous (key C2 test)
        acc_jump = np.linalg.norm(acc_out[idx_after] - acc_out[idx_before])
        assert acc_jump < 20.0, f"Acceleration discontinuity: {acc_jump:.3f} mm/s²"

    def test_accel_reference_output(self):
        """Accel reference is populated from quintic interpolation."""
        p0 = np.array([0.0, 0.0, 100.0, 0.0, 0.0, 0.0])
        p1 = np.array([0.0, 0.0, 60.0, 0.0, 0.0, 0.0])
        v0 = np.array([0.0, 0.0, -250.0, 0.0, 0.0, 0.0])
        v1 = np.zeros(6)
        a_zero = np.zeros(6)

        events = [
            ReferenceEvent(time=0.0, pose=p0, twist=v0, accel=a_zero),
            ReferenceEvent(time=0.5, pose=p1, twist=v1, accel=a_zero),
        ]
        ct = self._default_cumulative()

        _, _, accel_traj = _build_reference_from_events(events, 0.0, ct, len(ct))

        # Accel at endpoints should be zero (boundary condition)
        np.testing.assert_allclose(accel_traj[0], a_zero, atol=1e-8)

        # Mid-trajectory accel should be nonzero (quintic generates curvature)
        mid_nodes = [k for k in range(len(ct)) if 0.1 < ct[k] < 0.4]
        assert len(mid_nodes) > 0, "No mid-trajectory nodes found"
        mid_accel_norm = max(np.linalg.norm(accel_traj[k]) for k in mid_nodes)
        assert mid_accel_norm > 10.0, (
            f"Expected nonzero mid-trajectory accel, got max norm={mid_accel_norm:.2f}"
        )
