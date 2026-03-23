"""Stewart platform geometry — loads and holds all platform geometry constants.

Pure Python + numpy, no ROS2 dependency.  All constants are loaded from the
generated hardware_config module (source of truth: config/hardware_config.yaml).
"""

import numpy as np
import jugglebot.hardware_config as hw


class StewartGeometry:
    """Immutable container for Jugglebot Stewart platform geometry.

    All length units are millimetres unless a suffix says otherwise.
    Angles are radians in the internal representation.
    """

    def __init__(self):
        # ------------------------------------------------------------------
        # Node positions (shape: 6×3)
        # ------------------------------------------------------------------
        self.base_nodes = np.array(hw.GEOM_BASE_NODES_MM, dtype=np.float64)
        self.plat_nodes = np.array(hw.GEOM_INIT_PLAT_NODES_MM, dtype=np.float64)

        # ------------------------------------------------------------------
        # Heights and lengths
        # ------------------------------------------------------------------
        self.init_height_mm = float(hw.GEOM_INITIAL_HEIGHT_MM)
        self.init_leg_lengths_mm = np.array(hw.GEOM_INIT_LEG_LENGTHS_MM,
                                            dtype=np.float64)
        # Legacy: ball_joint_offset_mm is now 0.0, so this equals init_leg_lengths_mm.
        # Kept for backward compatibility with GUI and catch coordinator consumers.
        self.leg_lengths_with_offset_mm = np.array(
            hw.INIT_LEG_LENGTHS_WITH_OFFSET_MM, dtype=np.float64)
        self.ball_joint_offset_mm = float(hw.GEOM_BALL_JOINT_OFFSET_MM)

        # ------------------------------------------------------------------
        # Stroke and conversion
        # ------------------------------------------------------------------
        self.leg_stroke_mm = float(hw.GEOM_LEG_STROKE_MM)
        self.mm_to_rev = np.array(hw.GEOM_MM_TO_REV, dtype=np.float64)

        # Effective spool radius per leg (mm), derived from mm_to_rev:
        #   1 revolution = 2π × r_spool  ⟹  mm_to_rev = 1 / (2π × r_spool_mm)
        self.spool_radius_mm = 1.0 / (2.0 * np.pi * self.mm_to_rev)

        # ------------------------------------------------------------------
        # Platform dimensional parameters (for reference / recomputation)
        # ------------------------------------------------------------------
        self.base_radius_mm = float(hw.GEOM_BASE_RADIUS_MM)
        self.plat_radius_mm = float(hw.GEOM_PLAT_RADIUS_MM)

        # ------------------------------------------------------------------
        # Motor position limits (revolutions)
        # ------------------------------------------------------------------
        self.leg_motor_max_rev = float(hw.GEOM_LEG_MOTOR_MAX_POSITION_REVS)
        self.hand_motor_max_rev = float(hw.GEOM_HAND_MOTOR_MAX_POSITION_REVS)

    @property
    def num_legs(self) -> int:
        return 6

    @property
    def init_height_vec(self) -> np.ndarray:
        """Initial platform centre position in base frame: [0, 0, init_height]."""
        return np.array([0.0, 0.0, self.init_height_mm])
