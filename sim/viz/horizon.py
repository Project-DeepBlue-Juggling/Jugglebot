"""MPC predicted trajectory visualisation in the MuJoCo viewer.

Renders the MPC's predicted platform trajectory as a series of translucent
spheres colour-coded by time: bright green (near future) → faded (far future).
"""

from __future__ import annotations

import numpy as np
import mujoco


class HorizonRenderer:
    """Draws MPC horizon predictions in the MuJoCo viewer's user scene."""

    def __init__(self, init_height_mm: float, enabled: bool = True):
        self._init_height_m = init_height_mm / 1000.0
        self._enabled = enabled
        self._poses: np.ndarray | None = None
        self._times: np.ndarray | None = None

    def update(
        self,
        predicted_poses: np.ndarray | None,
        cumulative_times: np.ndarray | None = None,
    ) -> None:
        """Update with the latest predicted trajectory from the MPC.

        Parameters
        ----------
        predicted_poses : (N+1, 6) array or None
        cumulative_times : (N+1,) array of cumulative horizon times (seconds),
            starting at 0.  When provided, the colour fade and sphere size use
            time-proportional fractions instead of linear index fractions.
            This gives a visually accurate representation of the variable-
            resolution horizon (fine steps are close together, coarse steps
            are spread out).
        """
        self._poses = predicted_poses
        self._times = cumulative_times

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, value: bool) -> None:
        self._enabled = value

    def render(self, viewer) -> None:
        """Add horizon geoms to *viewer.user_scn*.  Call before viewer.sync()."""
        viewer.user_scn.ngeom = 0

        if not self._enabled or self._poses is None:
            return

        n = len(self._poses)
        max_geoms = min(n, viewer.user_scn.maxgeom)

        for k in range(max_geoms):
            pose = self._poses[k]

            # Pose offset (mm) → world position (m)
            pos_m = np.array([
                pose[0] / 1000.0,
                pose[1] / 1000.0,
                pose[2] / 1000.0 + self._init_height_m,
            ])

            # Colour: green, fading with horizon depth.
            # Use time-proportional fraction when cumulative times are
            # available so that coarse-tier spheres (representing longer
            # intervals) are visually distinct from fine-tier ones.
            if self._times is not None and len(self._times) == n and self._times[-1] > 0:
                t_frac = float(self._times[k] / self._times[-1])
            else:
                t_frac = k / max(1, n - 1)
            alpha = float(max(0.15, 1.0 - 0.85 * t_frac))
            rgba = np.array([0.2, 0.9, 0.2, alpha], dtype=np.float32)

            # Sphere radius shrinks slightly toward the end
            r = 0.006 * (1.0 - 0.4 * t_frac)
            size = np.array([r, 0.0, 0.0])

            geom = viewer.user_scn.geoms[viewer.user_scn.ngeom]
            mujoco.mjv_initGeom(
                geom,
                mujoco.mjtGeom.mjGEOM_SPHERE,
                size,
                pos_m,
                np.eye(3).flatten(),
                rgba,
            )
            viewer.user_scn.ngeom += 1
