"""Importable factories for ``MpcLoopHooks`` callbacks.

Hooks are wired by the top-level entry point (``run_mpc.py``) into
``run_mpc_loop`` and run once per tick on the 40 Hz hot loop.  Before
this module existed they were plain closures defined inline inside
``main()`` — unimportable, which prevented the W3 contract test from
exercising the production code directly.  When the `_on_pre_command`
augmented-assign closure bug slipped through W3 during W6 prep
(commit 149070d), the fact that the test was a local replica of the
hook rather than the real thing was the root cause of the
coverage gap.  This module closes that gap by exposing each hook as
a factory that returns an allocation-compliant closure with its
pre-allocated buffers captured in the enclosing scope.

See ``controller/HOT_LOOP_CONTRACT.md`` for the hook pattern and the
"augmented-assign in closures" gotcha these factories defend against.
"""

from __future__ import annotations

import numpy as np


def make_feedforward_pre_command_hook():
    """Return an ``on_pre_command`` hook wired for feedforward dynamics.

    The returned callable has the signature expected by
    ``MpcLoopHooks.on_pre_command``:
    ``(plant, mpc, tc, cmd, cmd_vel, diag) -> None``.

    Behaviour:

    * No-op when the plant does not have a ``set_pose`` attribute
      (e.g. ``MuJoCoPlant`` — sim dry-run has no feedforward channel).
    * Otherwise: derive twist and acceleration from the MPC's predicted
      poses using two-point finite differences, then call
      ``plant.set_pose(poses[0], twist_6dof=twist, accel_6dof=accel)``.

    Allocation contract:

    Three ``(6,)`` buffers (``twist``, ``twist_next``, ``accel``) are
    pre-allocated by this factory and captured in the closure.  The
    returned hook mutates them in place via ``np.subtract(..., out=buf)``
    and ``np.divide(buf, scalar, out=buf)`` — never a bare augmented-
    assignment like ``buf /= scalar``.  The augmented-assignment form
    would bind the buffer name as a local of the inner function (Python
    semantic rule: any assignment-like target makes the name a local),
    which would shadow the free-variable binding from the closure and
    cause an ``UnboundLocalError`` on the earlier ``out=buf`` read.
    This was caught live during W6 prep (commit 149070d); the
    factory-with-explicit-np.divide pattern below is what prevents
    it from recurring.
    """
    _twist_buf = np.empty(6)
    _twist_next_buf = np.empty(6)
    _accel_buf = np.empty(6)

    def _on_pre_command(plant_, mpc_, tc, cmd, cmd_vel, diag):
        """hot-loop body — see controller/HOT_LOOP_CONTRACT.md."""
        if not hasattr(plant_, 'set_pose'):
            return
        poses = mpc_.predicted_poses_view
        times = mpc_.predicted_times_view
        if poses is not None:
            dt0 = times[1] - times[0]
            dt1 = times[2] - times[1]
            # twist = (poses[1] - poses[0]) / dt0, in place
            np.subtract(poses[1], poses[0], out=_twist_buf)
            np.divide(_twist_buf, dt0, out=_twist_buf)
            # twist_next = (poses[2] - poses[1]) / dt1, in place
            np.subtract(poses[2], poses[1], out=_twist_next_buf)
            np.divide(_twist_next_buf, dt1, out=_twist_next_buf)
            # accel = (twist_next - twist) / (0.5 * (dt0 + dt1)), in place
            np.subtract(_twist_next_buf, _twist_buf, out=_accel_buf)
            np.divide(_accel_buf, 0.5 * (dt0 + dt1), out=_accel_buf)
            plant_.set_pose(poses[0], twist_6dof=_twist_buf, accel_6dof=_accel_buf)

    return _on_pre_command
