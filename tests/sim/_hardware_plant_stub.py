"""Test helper: build a HardwarePlant with mocked ZMQ for hermetic testing.

Used by ``tests/sim/test_plant_interface_contract.py`` and
``tests/sim/test_hardware_plant_failure_paths.py`` to exercise the real
``HardwarePlant`` class — including its hot-loop in-place mutation, FK
warm-start, watchdog cascades, and feedforward paths — without requiring
a live ZMQ socket topology in CI.

Mirrors the patching pattern in
``tests/sim/test_hot_loop_allocation_contract.py:_build_hardware_fixture``
and is the canonical "construct a real HardwarePlant in-memory" helper
this project uses for contract-level testing.

Usage::

    from tests.sim._hardware_plant_stub import build_hardware_plant_stub

    with build_hardware_plant_stub() as plant:
        # plant is a real controller.hardware_plant.HardwarePlant
        # with patched ZMQ; safe to call get_state(), command(), etc.
        ...

Phase 4 (Plan 2 — Tier 2a) adds failure-injection helpers for the
HardwarePlant safety chain.  Each helper patches at a BOUNDARY of the
production code, not at a private counter — per Plan 2 Working Note #1
("Drive real failures, not mocked ones"):

* ``inject_fk_failure(n_ticks)`` — context manager; patch
  ``controller.hardware_plant.leg_lengths_to_pose`` so the next N calls
  raise ``RuntimeError``.  Drives the real divergence path through
  ``get_state()`` (the ``RuntimeError`` is the same shape Newton-Raphson
  surfaces on real non-convergence at ``hardware_plant.py:580–602``).
* ``inject_singular_jacobian()`` — context manager; patch
  ``controller.hardware_plant.compute_jacobian`` to return a
  rank-deficient matrix AND force the FK-supplied Jacobian to be
  unavailable.  Drives the real ``LinAlgError`` path through the
  twist-solve at ``hardware_plant.py:737–740``.
* ``install_telemetry_pump(plant, *, target_pose=None, motor_vel=None,
  frame_mutator=None)`` — swap the plant's ``_sub.recv_multipart`` pump
  with a programmable per-tick frame.  Default returns byte-identical
  ``motor_pos`` (the publisher-alive-but-dead signature the detector
  at ``hardware_plant.py:648–683`` catches); the ``frame_mutator``
  hook supports per-tick variation (e.g. the 1-ULP bit-different
  variant T-U-T2a-7 uses to verify the bit-exact ``np.array_equal``
  compare).
"""

from __future__ import annotations

import contextlib
import msgpack
from unittest.mock import MagicMock, patch

import numpy as np


# Default target pose used to seed the synthetic telemetry — matches the
# hot-loop test's TARGET_POSE so the stub's encoder readouts represent a
# plant already at a credible production operating point.  Callers that
# want a different operating point can pass ``target_pose=...``.
_DEFAULT_TARGET_POSE = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])


class _FramePump:
    """recv_multipart replacement: alternates one telemetry frame, then Again.

    One frame per ``get_state()`` call — the drain loop in get_state reads
    one frame and breaks on the next Again.  Mirrors
    test_hot_loop_allocation_contract.py:_FramePump.
    """

    __slots__ = ('_frame', '_yield_frame', '_again_cls')

    def __init__(self, frame, again_cls):
        self._frame = frame
        self._yield_frame = True
        self._again_cls = again_cls

    def __call__(self, *args, **kwargs):
        if self._yield_frame:
            self._yield_frame = False
            return self._frame
        self._yield_frame = True
        raise self._again_cls()


@contextlib.contextmanager
def build_hardware_plant_stub(
    *,
    target_pose: np.ndarray | None = None,
    control_dt: float = 0.025,
):
    """Construct a ``HardwarePlant`` with patched ZMQ + synthetic telemetry.

    Yields a real ``controller.hardware_plant.HardwarePlant`` instance.
    On exit, restores the patched ``zmq`` and ``time.sleep`` modules.

    Parameters
    ----------
    target_pose : (6,) ndarray, optional
        Pose used to seed the synthetic motor-position telemetry.  The
        stub's ``get_state()`` will read encoder values consistent with
        the platform sitting at this pose.  Defaults to z=50 mm hold.
    control_dt : float
        ``HardwarePlant.__init__`` ``control_dt`` kwarg.  Defaults to
        the production 40 Hz period.
    """
    import zmq as _zmq_real
    import msgpack as _msgpack

    # Pre-compute motor positions at target_pose via MuJoCoPlant's IK so
    # the synthetic telemetry represents a plant at the requested pose.
    # Done once at fixture build — outside the contract-test measurement
    # window.
    if target_pose is None:
        target_pose = _DEFAULT_TARGET_POSE
    from plant.mujoco_plant import MuJoCoPlant as _SimPlant
    _sim_tmp = _SimPlant()
    target_ext_mm = _sim_tmp.pose_to_extensions(target_pose)
    target_motor_rev = target_ext_mm * _sim_tmp.geom.mm_to_rev
    del _sim_tmp

    telem_dict = {
        'motor_pos': [float(v) for v in target_motor_rev],
        'motor_vel': [0.0] * 6,
    }
    telem_payload = _msgpack.packb(telem_dict, use_bin_type=True)
    telem_frame = [b'telemetry', telem_payload]

    pump = _FramePump(telem_frame, _zmq_real.Again)

    with patch('controller.hardware_plant.zmq') as mock_zmq, \
            patch('controller.hardware_plant.time.sleep'):
        mock_ctx = MagicMock()
        mock_pub = MagicMock()
        mock_sub = MagicMock()
        mock_ctx.socket.side_effect = [mock_pub, mock_sub]
        mock_zmq.Context.return_value = mock_ctx
        mock_zmq.Again = _zmq_real.Again
        mock_zmq.NOBLOCK = 0

        from controller.hardware_plant import HardwarePlant
        plant = HardwarePlant(control_dt=control_dt)

    # Replace MagicMock hot methods with plain callables.  MagicMock
    # records every call in ``call_args_list`` whose backing list grows
    # per tick — irrelevant for correctness tests but matches the
    # hot-loop fixture's pattern so behaviour is consistent across both.
    plant._pub.send_multipart = lambda *a, **k: None
    plant._sub.recv_multipart = pump

    try:
        yield plant
    finally:
        plant.close()


# ---------------------------------------------------------------------
# Phase 4 (Plan 2 Tier 2a) — failure-injection helpers
# ---------------------------------------------------------------------

def inject_fk_failure(n_ticks: int):
    """Context manager: force the next ``n_ticks`` FK calls to raise
    ``RuntimeError``, then resume real FK.

    Patches ``controller.hardware_plant.leg_lengths_to_pose`` (the module-
    level binding the production code uses at ``hardware_plant.py:580``)
    rather than the bound method on the plant instance — this is the
    BOUNDARY of the FK contract; the production code's ``except
    RuntimeError`` at ``:602`` is the surface under test.  Real
    Newton-Raphson surfaces non-convergence by raising RuntimeError, so
    patching at this boundary drives the genuine fail-path through
    ``get_state()``.

    Parameters
    ----------
    n_ticks : int
        Number of consecutive FK calls to make raise.  Each subsequent
        call delegates to the real ``leg_lengths_to_pose`` and counts
        as a success (resetting ``_fk_fail_count``).

    Usage::

        with build_hardware_plant_stub() as plant:
            plant.get_state()  # one successful FK so _fk_ever_succeeded
            with inject_fk_failure(n_ticks=5):
                for _ in range(5):
                    plant.get_state()  # 5 real RuntimeError paths
            # estop_requested == True (5 >= _FK_FAIL_ESTOP_THRESHOLD)
    """
    import controller.hardware_plant as _hp_mod
    real_fk = _hp_mod.leg_lengths_to_pose
    call_count = {'n': 0}

    def _fail_or_pass(*args, **kwargs):
        call_count['n'] += 1
        if call_count['n'] <= n_ticks:
            raise RuntimeError(
                f"_hardware_plant_stub: forced FK non-convergence "
                f"(call {call_count['n']} of {n_ticks})"
            )
        return real_fk(*args, **kwargs)

    # Preserve the ``last_iterations`` attribute the production code
    # reads at hardware_plant.py:595 — without it, the success branch
    # crashes with AttributeError.
    _fail_or_pass.last_iterations = 0
    return patch.object(_hp_mod, 'leg_lengths_to_pose',
                        side_effect=_fail_or_pass)


@contextlib.contextmanager
def inject_singular_jacobian():
    """Context manager: force ``compute_jacobian`` to return a
    rank-deficient (singular) 6×6 matrix AND null out the
    FK-supplied Jacobian path.

    The production code at ``hardware_plant.py:719`` prefers the
    Jacobian returned by ``leg_lengths_to_pose`` when available
    (``if fk_jacobian is not None: J = fk_jacobian``).  To exercise
    the singular ``np.linalg.solve`` path, we BOTH return a singular
    matrix from ``compute_jacobian`` AND have ``leg_lengths_to_pose``
    return ``fk_jacobian=None`` so the production code falls through
    to the ``else: J = compute_jacobian(...)`` branch at ``:721–724``.

    The singular matrix is constructed by zeroing one column of an
    identity matrix — minimal rank-deficient surgery; ``np.linalg.solve``
    raises ``LinAlgError`` deterministically.

    Note that ``state.platform_twist.fill(0.0)`` is unconditional at
    ``hardware_plant.py:715`` (before the try/except), so the zero-twist
    behaviour is structurally guaranteed.  The test value of this
    injection is observing the ONCE-only ``logger.warning`` at ``:739``
    and the ``_jacobian_singular_warned`` flag flip at ``:740``.

    Caller MUST also arrange for ``motor_vel`` to be non-zero in the
    telemetry stream, since the twist-solve branch at ``:716`` is
    gated on ``np.any(vel_mmps != 0.0)``.
    """
    import controller.hardware_plant as _hp_mod
    real_fk = _hp_mod.leg_lengths_to_pose

    def _fk_drops_jacobian(*args, **kwargs):
        pos_offset, rot_matrix, _fk_J = real_fk(*args, **kwargs)
        return pos_offset, rot_matrix, None

    _fk_drops_jacobian.last_iterations = 0

    singular = np.eye(6)
    singular[0, 0] = 0.0  # rank 5 → singular

    with patch.object(_hp_mod, 'leg_lengths_to_pose',
                      side_effect=_fk_drops_jacobian), \
         patch.object(_hp_mod, 'compute_jacobian', return_value=singular):
        yield


def install_telemetry_pump(plant, *, target_pose=None, motor_vel=None,
                           frame_mutator=None):
    """Replace the plant's ZMQ pump with a fresh one keyed off the
    given ``target_pose`` / ``motor_vel`` / per-tick ``frame_mutator``.

    Mirrors the ``_FramePump`` pattern but exposes a programmable
    per-tick mutator for the frozen-motor + recovery scenarios that
    need to vary the telemetry contents across ticks.

    Parameters
    ----------
    plant : HardwarePlant
        Plant whose ``_sub.recv_multipart`` will be replaced.
    target_pose : (6,) ndarray, optional
        Pose used to seed ``motor_pos``.  Defaults to z=50 mm hold.
    motor_vel : (6,) ndarray, optional
        Initial ``motor_vel`` payload.  Defaults to all-zero.
    frame_mutator : callable(tick_idx, motor_pos_list, motor_vel_list),
                    optional
        Called once per tick *before* msgpack-encoding the telemetry
        frame; may mutate the lists in place.  Useful for frozen-motor
        injection (no mutation) vs varying-by-1-LSB sequences.
    """
    import zmq as _zmq

    if target_pose is None:
        target_pose = np.array([0.0, 0.0, 50.0, 0.0, 0.0, 0.0])
    if motor_vel is None:
        motor_vel = [0.0] * 6
    else:
        motor_vel = [float(v) for v in motor_vel]

    # Compute motor_rev at the requested pose (same path as
    # ``build_hardware_plant_stub``).
    from plant.mujoco_plant import MuJoCoPlant as _SimPlant
    _sim_tmp = _SimPlant()
    target_ext_mm = _sim_tmp.pose_to_extensions(target_pose)
    target_motor_rev = [float(v) for v in
                        target_ext_mm * _sim_tmp.geom.mm_to_rev]
    del _sim_tmp

    state = {'tick': 0, 'yield_frame': True}

    def pump(*_args, **_kwargs):
        if state['yield_frame']:
            state['yield_frame'] = False
            state['tick'] += 1
            mp = list(target_motor_rev)
            mv = list(motor_vel)
            if frame_mutator is not None:
                frame_mutator(state['tick'], mp, mv)
            payload = msgpack.packb(
                {'motor_pos': mp, 'motor_vel': mv}, use_bin_type=True)
            return [b'telemetry', payload]
        state['yield_frame'] = True
        raise _zmq.Again()

    plant._sub.recv_multipart = pump
    return pump
