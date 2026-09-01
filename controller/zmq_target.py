"""ZMQ-based target source: ingests ``TargetCommand`` messages off :5558.

Presents a ZMQ subscription as a ``TargetSource``. Generic ingest plumbing —
it decodes and rate-limits whatever a publisher puts on :5558 and returns the
latest command per control step.

NO LIVE PUBLISHER as of 2026-09-01: the ROS2 ``mpc_bridge_node`` that fed
:5558 (spacemouse / GUI / catch-coordinator topics → ``TargetCommand``) was
removed with the MPC chain — dormant since 2026-08-01, superseded by the
unified 7-DoF planner. Kept because the decode/clamp path is publisher-
agnostic and still covered by ``tests/sim/test_zmq_target.py``; the deleted
producer is preserved at git tag ``mpc-final`` (see
``logbook/2026-09-01-mpc-chain-removed.md``).

Usage::

    source = ZmqTargetSource(default_z_mm=170.0)
    # ...
    while running:
        source.poll()           # drain ZMQ (non-blocking)
        if source.enabled:
            tc = source.update(t, state)
            cmd, diag = mpc.solve(state, tc.target_pose, ...)
        else:
            time.sleep(0.02)    # idle until enabled
"""

from __future__ import annotations

import logging
from typing import Any, List

import numpy as np

from .target import (
    ReferenceEvent,
    TargetCommand,
    pose_6dof_from_state,
    is_warm_start_invalidating,
    compute_stretch_ms,
    flat_target_to_events,
)

# Import IPC from the production motion package.
# Callers must ensure repo root is on sys.path (for controller/) and
# ros_ws/src/jugglebot is on sys.path (for jugglebot.motion.ipc).
from jugglebot.motion.ipc import (
    MPC_TARGET_ADDR,
    TOPIC_MPC_TARGET,
    TOPIC_MPC_MODE,
    MpcTargetIPC,
)

logger = logging.getLogger(__name__)


class ZmqTargetSource:
    """Reads MPC targets from ZMQ and provides them via the TargetSource protocol.

    Two ZMQ SUB sockets on :5558:
      - CONFLATE for targets (latest wins — the MPC always tracks the
        freshest intent, regardless of how many arrived between solves)
      - Non-CONFLATE for mode (every enable/disable transition is delivered)

    Parameters
    ----------
    target_addr : str
        ZMQ address to connect to (default :5558).
    default_z_mm : float
        Default Z height (mm) for the active pose returned before any target
        has been received.  Should match ``default_active_z_mm`` from
        ``hardware_config.yaml`` (typically 170.0).
    v_max_mmps : float or None
        Per-leg kinematic velocity ceiling (mm/s) used by the K1–K6
        feasibility layer (``make_feasible_events``) for K2 (interior peak
        velocity) and K6 (event-endpoint twist clamp).  Threaded into
        ``flat_target_to_events`` in ``update()`` together with ``tau_s``;
        when both are set the K1–K6 path runs, otherwise the legacy path
        runs with a one-shot ``DeprecationWarning``.  Pass
        ``MPCParams.max_leg_vel_mmps``.
    clamp_start_twist_mmps : float or None
        Upper bound on the FK-derived start twist used as the quintic ref's
        start-velocity boundary.  Distinct from ``v_max_mmps``: the kinematic
        ceiling is too permissive for this role, because a fallback-driven
        plant runaway sits below the ceiling yet must NOT be honoured as a
        new ref's start velocity (it would self-seed the next tick's
        quintic → positive feedback → PLANT_TELEMETRY_COLLAPSE).  Pass
        ``MPCParams.max_ref_start_twist_mmps`` (60 mm/s for Phase 4).
        ``None`` disables clamping.  See logbook
        2026-04-19-bundle-a-quintic-ref-settling-and-live-twist-trap.md.
    """

    def __init__(self,
                 target_addr: str = MPC_TARGET_ADDR,
                 default_z_mm: float = 170.0,
                 v_max_mmps: float | None = None,
                 tau_s: float | None = None,
                 clamp_start_twist_mmps: float | None = None,
                 feedback_pub: Any = None):
        self._ipc = MpcTargetIPC(target_addr)

        self._mode = 'disabled'
        self._default_pose = np.array([
            0.0, 0.0, default_z_mm, 0.0, 0.0, 0.0])
        self._v_max_mmps = v_max_mmps
        self._tau_s = tau_s
        self._clamp_start_twist_mmps = clamp_start_twist_mmps
        # W11: feedback publisher for catch rejection / stretch warnings.
        # Injected rather than constructed internally so tests can stay ZMQ-free.
        self._feedback_pub = feedback_pub
        self._last_rejected_arrival: float | None = None
        # Fingerprint of the most recent request that K1–K6 rejected.  When
        # the cache is empty AND the current request matches this, skip the
        # recompute — the coordinator must publish a *distinct* target
        # (different pose, arrival, or source) before we re-run the
        # polynomial-root math.  Without this, a stuck infeasible catch
        # re-fires the full feasibility pass at 40 Hz indefinitely (the
        # message-side dedupe in _publish_rejection only suppresses output,
        # not the underlying compute).
        self._last_rejected_request: tuple | None = None

        # Latest target state (updated by poll())
        self._target_pose = self._default_pose.copy()
        self._arrival_time: float | None = None
        self._target_twist: np.ndarray | None = None
        self._source = ''
        self._has_target = False

        # Cached ref_events (quintic) rebuilt only when a *distinct* target
        # arrives.  Caching is critical for IPOPT warm-start stability —
        # rebuilding the quintic every tick would present a new optimization
        # landscape to the solver on every solve.  See logbook
        # 2026-04-18-move5-overshoot-stall-and-plant-collapse.md.
        self._cached_events: List[ReferenceEvent] | None = None
        self._target_dirty = False

        # W5: track the previously-emitted horizon-end state so we can detect
        # structurally-large reference shifts and hint IPOPT to cold-start.
        self._prev_ref_horizon_end_pose: np.ndarray | None = None
        self._prev_ref_horizon_end_twist: np.ndarray | None = None
        # One-tick latch: True when the cache was just rebuilt AND the new
        # ref differed structurally from the old ref.  Cleared after the
        # next update() so downstream sees warm_start_valid=False for one
        # solve only.
        self._warm_start_valid_hint: bool = True

        # W4b: pre-allocated TargetCommand.  ``update`` returns the same
        # instance every tick with fields mutated in place.  Also pre-
        # allocate the single-event terminal-hold list used when the
        # cached quintic expires (K1–K6 "hold at terminal pose" policy).
        self._tc = TargetCommand(
            target_pose=self._target_pose,
            arrival_time=None,
            target_twist=None,
            ref_events=None,
            warm_start_valid=True,
        )
        self._terminal_hold_event = ReferenceEvent(
            time=0.0,
            pose=np.empty(6),
            twist=np.zeros(6),
            accel=np.zeros(6),
        )
        self._terminal_hold_list: List[ReferenceEvent] = [
            self._terminal_hold_event,
        ]
        # W4b: cache of ``self._target_pose.tobytes()``.  Invalidated on
        # the dirty bit in poll() and in the rejection-fingerprint
        # comparison below.  Without this the fingerprint tuple
        # materialises fresh 48-byte bytes every tick just to compare
        # against _last_rejected_request.
        self._target_pose_bytes: bytes | None = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def mode(self) -> str:
        """Current mode: 'spacemouse', 'gui', 'catch', or 'disabled'."""
        return self._mode

    @property
    def enabled(self) -> bool:
        """True when the MPC should be solving and commanding."""
        return self._mode != 'disabled'

    @property
    def has_target(self) -> bool:
        """True after the first target has been received in the current session."""
        return self._has_target

    @property
    def source(self) -> str:
        """Source identifier of the latest target ('spacemouse', 'catch', etc.)."""
        return self._source

    # ------------------------------------------------------------------
    # Polling
    # ------------------------------------------------------------------

    def poll(self) -> None:
        """Non-blocking: drain all pending ZMQ messages and update state.

        Call this once per MPC cycle (or let ``update()`` call it for you).
        Mode messages are processed first so the enabled/disabled state is
        always up-to-date before the latest target is applied.
        """
        messages = self._ipc.recv_all()
        for topic, msg in messages:
            if topic == TOPIC_MPC_MODE:
                new_mode = msg.get('mode', 'disabled')
                if new_mode != self._mode:
                    logger.info("ZmqTargetSource: mode %s → %s",
                                self._mode, new_mode)
                    self._mode = new_mode
                    if new_mode == 'disabled':
                        # Clear target state on disable so a stale target
                        # from the previous session isn't reused.
                        self._has_target = False
                        self._cached_events = None
                        self._target_dirty = False
                        self._last_rejected_request = None
            elif topic == TOPIC_MPC_TARGET:
                pose = msg.get('target_pose')
                # Missing / wrong-type / wrong-length target_pose — log
                # warning naming the field, mirror the non-finite case
                # below.  Pre-fix the entire branch was silently skipped
                # with no diagnostic.  Non-sequence types (int / float /
                # str) raised ``TypeError`` on ``len(pose)`` and aborted
                # the MPC tick — same failure-class as Bug A's
                # uncaught msgpack errors, one level higher.  See Plan 2
                # Phase 6 bugfix Bug B
                # (logbook 2026-05-11-tier2c-zmq-recv-resilience-bugfix.md).
                if (pose is None
                        or not isinstance(pose, (list, tuple))
                        or len(pose) != 6):
                    logger.warning(
                        "ZmqTargetSource: rejected target with "
                        "missing/wrong-type/wrong-length 'target_pose': "
                        "%r (source=%s)",
                        pose, msg.get('source', ''))
                    continue
                pose_arr = np.array(pose, dtype=float)
                if not np.all(np.isfinite(pose_arr)):
                    logger.warning(
                        "ZmqTargetSource: rejected target with "
                        "non-finite pose: %s (source=%s)",
                        pose_arr, msg.get('source', ''))
                    continue
                twist = msg.get('target_twist')
                twist_arr = (
                    np.array(twist, dtype=float)
                    if twist is not None else None)
                if twist_arr is not None and not np.all(
                        np.isfinite(twist_arr)):
                    logger.warning(
                        "ZmqTargetSource: rejected target with "
                        "non-finite twist: %s (source=%s)",
                        twist_arr, msg.get('source', ''))
                    continue
                arrival = msg.get('arrival_time')
                if arrival is not None and not np.isfinite(arrival):
                    logger.warning(
                        "ZmqTargetSource: rejected target with "
                        "non-finite arrival_time: %s (source=%s)",
                        arrival, msg.get('source', ''))
                    continue
                # Only mark the cache dirty when something actually
                # changed — spacemouse-like streams emit many ticks
                # with identical content, and rebuilding the quintic
                # every time would defeat IPOPT warm-start.
                #
                # arrival_time is INTENTIONALLY excluded from the dirty
                # check.  Stream senders that refresh arrival_time every
                # tick would otherwise rebuild the cache each tick, and
                # each rebuild re-reads the live plant twist as the
                # quintic's start-velocity boundary.  If the plant is in
                # a fallback-driven runaway (cmd extrapolation during an
                # IPOPT-failure fallback arm — Tier-1 fallback_extrap,
                # logbook 2026-05-20-hold-extrap-positive-feedback-
                # chaotic-motion.md), the runaway twist self-seeds the
                # next ref → positive feedback → PLANT_TELEMETRY_COLLAPSE
                # (see logbook 2026-04-19-bundle-a-quintic-ref-settling-
                # and-live-twist-trap.md).  The scalar arrival_time is
                # still forwarded on every TargetCommand below.
                changed = (
                    not self._has_target
                    or not np.array_equal(pose_arr, self._target_pose)
                    or not _twist_equal(twist_arr, self._target_twist)
                )
                self._target_pose = pose_arr
                self._arrival_time = arrival
                self._target_twist = twist_arr
                self._source = msg.get('source', '')
                self._has_target = True
                if changed:
                    self._target_dirty = True
                    # W4b: invalidate cached tobytes() view.
                    self._target_pose_bytes = None

    # ------------------------------------------------------------------
    # TargetSource protocol
    # ------------------------------------------------------------------

    def update(self, sim_time: float, state: Any) -> TargetCommand:
        """Return the latest target command.

        Satisfies the ``TargetSource`` protocol.  Calls ``poll()``
        internally so callers don't need to drain ZMQ separately.

        If no target has been received yet, returns the default active pose
        (``[0, 0, default_z_mm, 0, 0, 0]``) with arrival_time=None
        (converge ASAP).

        Once a target has been received, emits a quintic ``ref_events``
        anchored at the live plant pose and (clamped) twist at ``sim_time``,
        ending at the target.  Re-anchoring on every *distinct* target (not
        every tick) ensures the MPC is never asked to unwind the plant
        against a ref that starts from a stale snapshot — the
        MPC_OVERSHOOT_SATURATION mechanism — while still preserving IPOPT
        warm-start between solves tracking the same target.
        """
        self.poll()

        # W4b: decide recompute without building the fingerprint tuple in
        # the common case.  When not dirty and cache is populated, we
        # don't need to look at _last_rejected_request at all — skip the
        # tobytes() + tuple allocation entirely.
        recompute = False
        request_key = None
        if self._has_target:
            if self._target_dirty:
                recompute = True
            elif self._cached_events is None:
                # Only now do we need the fingerprint to decide whether
                # to retry a previously-rejected request.
                if self._target_pose_bytes is None:
                    self._target_pose_bytes = self._target_pose.tobytes()
                request_key = (
                    self._target_pose_bytes,
                    self._arrival_time,
                    self._source,
                )
                if request_key != self._last_rejected_request:
                    recompute = True
        if recompute:
            # Ensure request_key is populated for the rejection-memo path
            # below.  When we entered ``recompute=True`` via target_dirty
            # (not cache-miss), the fingerprint was skipped above.
            if request_key is None:
                if self._target_pose_bytes is None:
                    self._target_pose_bytes = self._target_pose.tobytes()
                request_key = (
                    self._target_pose_bytes,
                    self._arrival_time,
                    self._source,
                )
            current_pose = pose_6dof_from_state(state)
            current_twist = np.asarray(state.platform_twist, dtype=float)
            # Catch path: no_stretch=True → reject rather than silently
            # delay a hard-deadline catch.  Non-catch paths: stretch (K1–K6
            # default) and emit a stretch-warning via target feedback (W11).
            no_stretch = (self._source == 'catch')
            # W11: pre-capture the requested arrival_time so we can compute
            # stretch_ms when the feasibility layer pushes it out.
            requested_arrival = self._arrival_time
            new_events, reason = flat_target_to_events(
                current_pose, current_twist,
                self._target_pose, sim_time,
                target_twist=self._target_twist,
                arrival_time=self._arrival_time,
                clamp_start_twist_mmps=self._clamp_start_twist_mmps,
                v_max_mmps=self._v_max_mmps,
                tau_s=self._tau_s,
                no_stretch=no_stretch,
                return_reason=True,
            )

            if reason is not None and no_stretch:
                # Catch rejection — DON'T overwrite the cached events.  The
                # platform continues the last feasible trajectory (per the
                # decision recorded in the Day-2 plan: "continuance of the
                # last feasible move; once that's done, hold position").
                # Publish feedback so the catch coordinator can re-plan or
                # abort.  Cache-intact so warm-start stays healthy while the
                # coordinator retargets.
                self._publish_rejection(requested_arrival, reason)
                # Clear _target_dirty so we don't re-run feasibility every
                # tick.  The next distinct catch target will re-set it via
                # poll()'s `changed` check; identical retransmits are deduped
                # at the IPC level (CONFLATE socket + content-equality in
                # poll()).  The rejection itself is deduped by arrival_time
                # inside _publish_rejection so the coordinator isn't spammed.
                # Also fingerprint this rejected request so the
                # `_cached_events is None` clause of the entry guard above
                # doesn't keep re-firing the polynomial-root math at 40 Hz.
                self._target_dirty = False
                self._last_rejected_request = request_key
            else:
                # W5: detect large structural shift before overwriting cache.
                new_horizon_end_pose = new_events[-1].pose
                new_horizon_end_twist = new_events[-1].twist
                invalidated = is_warm_start_invalidating(
                    self._prev_ref_horizon_end_pose,
                    self._prev_ref_horizon_end_twist,
                    new_horizon_end_pose,
                    new_horizon_end_twist,
                )
                self._cached_events = new_events
                self._target_dirty = False
                # Successful plan — clear any stale rejection fingerprint so
                # a future identical-but-now-feasible request is allowed to
                # retry (e.g. plant pose has shifted enough to satisfy K2/K3).
                self._last_rejected_request = None
                self._prev_ref_horizon_end_pose = new_horizon_end_pose.copy()
                self._prev_ref_horizon_end_twist = new_horizon_end_twist.copy()
                self._warm_start_valid_hint = not invalidated
                if reason is None and self._feedback_pub is not None:
                    # Non-catch caller, successful plan — detect stretch-warn.
                    self._maybe_publish_stretch_warning(
                        requested_arrival, new_events,
                    )
        # When we did NOT rebuild the cache this tick, warm_start is
        # trivially valid (same ref as last tick).
        warm_start_valid_out = self._warm_start_valid_hint
        # Latch: reset to True for subsequent ticks until next rebuild.
        self._warm_start_valid_hint = True

        # W11: once the cached quintic has expired in real time, hold at
        # its terminal pose (catch-rejection continuity — see Day-2 plan).
        # W4b: mutate the pre-allocated terminal-hold event + list in
        # place rather than allocating a fresh single-event list per
        # tick.  Twist/accel stay at the init-time zeros; only pose and
        # time need updating.
        ref_events_out = self._cached_events
        if ref_events_out is not None and len(ref_events_out) > 0:
            if sim_time > ref_events_out[-1].time + 1e-6:
                terminal = ref_events_out[-1]
                np.copyto(self._terminal_hold_event.pose, terminal.pose)
                self._terminal_hold_event.time = sim_time
                # Preserve the zeros already in twist/accel buffers from
                # __init__; no need to re-zero.
                ref_events_out = self._terminal_hold_list

        # W4b: mutate the pre-allocated TargetCommand in place.  Fields
        # that are ndarray references (target_pose, target_twist) are
        # assigned by REFERENCE — consumers don't mutate, and the
        # .copy() defensiveness of the pre-W4b code had no call site
        # justification (grep-verified on every TargetCommand consumer).
        self._tc.target_pose = self._target_pose
        self._tc.arrival_time = self._arrival_time
        self._tc.target_twist = self._target_twist
        self._tc.ref_events = ref_events_out
        self._tc.warm_start_valid = warm_start_valid_out
        return self._tc

    # ------------------------------------------------------------------
    # W11: feedback publication helpers
    # ------------------------------------------------------------------

    def _publish_rejection(self, arrival_time: float | None,
                           reason: str) -> None:
        """Publish a ``rejected_infeasible`` TargetFeedback for a catch target.

        Dedupes by arrival_time to avoid spamming the coordinator when a
        catch persistently retransmits the same infeasible target.
        """
        if self._feedback_pub is None or arrival_time is None:
            logger.info(
                "ZmqTargetSource: catch rejected (no feedback_pub or "
                "arrival_time): %s", reason,
            )
            return
        if (self._last_rejected_arrival is not None
                and abs(arrival_time - self._last_rejected_arrival) < 0.05):
            return
        self._last_rejected_arrival = arrival_time
        try:
            from jugglebot.motion.ipc import make_target_feedback
            self._feedback_pub.send(make_target_feedback(
                arrival_time, False, self._source,
                feedback_type='rejected_infeasible',
                reason=reason,
            ))
            logger.info("ZmqTargetSource: published rejection: %s", reason)
        except Exception as exc:
            logger.warning(
                "ZmqTargetSource: feedback publish failed (%s): %s",
                type(exc).__name__, exc,
            )

    def _maybe_publish_stretch_warning(
        self,
        requested_arrival: float | None,
        realised_events: list,
    ) -> None:
        """Publish a ``stretch_warning`` when K2/K3 stretched the arrival.

        Only fires when the stretch is >1 ms (noise floor) so the coordinator
        isn't notified of irrelevant sub-ms adjustments.  Delegates the
        proposed-vs-realised arithmetic to ``compute_stretch_ms`` so the
        semantics live in a single place (``controller/target.py``).
        """
        if (self._feedback_pub is None or requested_arrival is None
                or not realised_events):
            return
        # Synthesize a one-element "proposed" list whose terminal event time
        # is the requested arrival — compute_stretch_ms compares only the
        # terminal times.  Pose/twist/accel are unread by the helper.
        proposed_terminal = ReferenceEvent(
            time=requested_arrival,
            pose=realised_events[-1].pose,
            twist=realised_events[-1].twist,
            accel=realised_events[-1].accel,
        )
        stretch_ms = compute_stretch_ms([proposed_terminal], realised_events)
        if stretch_ms <= 1.0:
            return
        try:
            from jugglebot.motion.ipc import make_target_feedback
            self._feedback_pub.send(make_target_feedback(
                requested_arrival, True, self._source,
                feedback_type='stretch_warning',
                stretch_ms=float(stretch_ms),
            ))
        except Exception as exc:
            logger.warning(
                "ZmqTargetSource: stretch-warn publish failed (%s): %s",
                type(exc).__name__, exc,
            )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Reset to default state (no target, mode unchanged)."""
        self._target_pose = self._default_pose.copy()
        self._arrival_time = None
        self._target_twist = None
        self._source = ''
        self._has_target = False
        self._cached_events = None
        self._target_dirty = False
        self._last_rejected_arrival = None
        self._last_rejected_request = None

    def close(self) -> None:
        """Close ZMQ sockets."""
        self._ipc.close()


def _twist_equal(a: np.ndarray | None, b: np.ndarray | None) -> bool:
    """Compare two optional twist arrays for change-detection."""
    if a is None and b is None:
        return True
    if a is None or b is None:
        return False
    return np.array_equal(a, b)
