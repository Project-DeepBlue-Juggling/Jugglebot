"""MPC predicted trajectory visualisation in the MuJoCo viewer.

Renders the MPC's predicted platform trajectory as a series of translucent
spheres colour-coded by time: bright green (near future) → faded (far future).
"""

import numpy as np
import mujoco


class HorizonRenderer:
    """Draws MPC horizon predictions in the MuJoCo viewer's user scene."""

    def __init__(self, init_height_mm: float, enabled: bool = True):
        self._init_height_m = init_height_mm / 1000.0
        self._enabled = enabled
        self._poses: np.ndarray | None = None

    def update(self, predicted_poses: np.ndarray | None) -> None:
        """Update with the latest predicted trajectory from the MPC."""
        self._poses = predicted_poses

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

            # Colour: green, fading with horizon depth
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
