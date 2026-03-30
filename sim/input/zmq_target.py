"""ZMQ-based target source for hardware MPC operation.

Receives MPC target commands from the MPC bridge node via ZeroMQ and
presents them as a ``TargetSource`` for the MPC control loop.

The bridge node (``mpc_bridge_node.py``) converts ROS2 topics (spacemouse,
GUI, shell, catch coordinator) into ``TargetCommand`` messages and publishes
them on :5558.  This source reads them and returns the latest each MPC step.

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
from typing import Any

import numpy as np

from controller.target import TargetCommand

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
    """

    def __init__(self,
                 target_addr: str = MPC_TARGET_ADDR,
                 default_z_mm: float = 170.0):
        self._ipc = MpcTargetIPC(target_addr)

        self._mode = 'disabled'
        self._default_pose = np.array([
            0.0, 0.0, default_z_mm, 0.0, 0.0, 0.0])

        # Latest target state (updated by poll())
        self._target_pose = self._default_pose.copy()
        self._arrival_time: float | None = None
        self._target_twist: np.ndarray | None = None
        self._source = ''
        self._has_target = False

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def mode(self) -> str:
        """Current mode: 'spacemouse', 'gui', 'shell', 'catch', or 'disabled'."""
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
            elif topic == TOPIC_MPC_TARGET:
                pose = msg.get('target_pose')
                if pose is not None and len(pose) == 6:
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
                    self._target_pose = pose_arr
                    self._arrival_time = arrival
                    self._target_twist = twist_arr
                    self._source = msg.get('source', '')
                    self._has_target = True

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
        """
        self.poll()
        return TargetCommand(
            target_pose=self._target_pose.copy(),
            arrival_time=self._arrival_time,
            target_twist=(self._target_twist.copy()
                          if self._target_twist is not None else None),
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

    def close(self) -> None:
        """Close ZMQ sockets."""
        self._ipc.close()
