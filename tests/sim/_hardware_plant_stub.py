"""Test helper: build a HardwarePlant with mocked ZMQ for hermetic testing.

Used by ``tests/sim/test_plant_interface_contract.py`` (and forthcoming
Phase 6 tests) to exercise the real ``HardwarePlant`` class — including
its hot-loop in-place mutation, FK warm-start, and feedforward paths —
without requiring a live ZMQ socket topology in CI.

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
"""

from __future__ import annotations

import contextlib
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
