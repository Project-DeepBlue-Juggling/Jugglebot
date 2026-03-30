"""Validate MuJoCo Stewart platform model against existing IK/FK.

Tests:
  - Model loads without error
  - Home position: platform at expected height with correct orientation
  - FK validation: command known leg extensions → read platform pose → compare
    against ik_solver.py FK for 10+ test poses across the workspace
  - Leg extension limits match leg_stroke_mm (280 mm)

Acceptance criteria:
  - Position error < 1 mm
  - Orientation error < 0.5 deg
"""

import os
import numpy as np
import pytest

import mujoco
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    leg_lengths_to_pose,
    rotvec_to_rot_matrix,
    rot_matrix_to_rotvec,
    quat_to_rot_matrix,
)

# ---- Paths ----
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
MODEL_PATH = os.path.join(_REPO_ROOT, 'sim', 'model', 'jugglebot.xml')

# ---- Thresholds ----
POS_TOL_MM = 1.0     # position error tolerance
ORI_TOL_DEG = 0.5    # orientation error tolerance
SETTLE_TIME_S = 2.0  # simulation time to let the platform settle


@pytest.fixture(scope='module')
def geom():
    """Load Stewart platform geometry."""
    return StewartGeometry()


@pytest.fixture(scope='module')
def model():
    """Load MuJoCo model."""
    return mujoco.MjModel.from_xml_path(os.path.abspath(MODEL_PATH))


@pytest.fixture
def data(model):
    """Create fresh MuJoCo data for each test."""
    d = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, d, 0)  # reset to 'home' keyframe
    mujoco.mj_forward(model, d)
    return d


def _get_sensor_data(model, data, name):
    """Read sensor data by name."""
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    adr = model.sensor_adr[sensor_id]
    dim = model.sensor_dim[sensor_id]
    return data.sensordata[adr:adr + dim].copy()


def _get_platform_pose_from_mujoco(model, data):
    """Extract platform pose from MuJoCo state.

    Returns:
        pos_offset_mm: (3,) platform offset from home in mm
        rot: (3,3) rotation matrix
    """
    # Platform position from sensor (in meters)
    pos_m = _get_sensor_data(model, data, 'platform_pos')
    # Platform quaternion from sensor (w, x, y, z)
    quat = _get_sensor_data(model, data, 'platform_quat')

    # Convert to mm and compute offset from home
    initial_height_m = 0.5743  # from hardware_config
    pos_offset_mm = np.array([
        pos_m[0] * 1000.0,
        pos_m[1] * 1000.0,
        (pos_m[2] - initial_height_m) * 1000.0,
    ])

    rot = quat_to_rot_matrix(quat[0], quat[1], quat[2], quat[3])
    return pos_offset_mm, rot


def _get_slide_positions_m(model, data):
    """Read all 6 slide joint positions in meters."""
    return np.array([_get_sensor_data(model, data, f'slide_pos_{i}')[0]
                     for i in range(6)])


def _slide_m_to_extensions_mm(slide_m, geom):
    """Convert MuJoCo slide values (m) to IK extensions (mm).

    Inverse of _extensions_mm_to_slide_m.
    IK extension = absolute_length - init_leg_lengths
    absolute_length = geom_home + slide (in mm)
    """
    base_nodes = geom.base_nodes / 1000.0
    plat_nodes = geom.plat_nodes / 1000.0
    height_m = geom.init_height_mm / 1000.0

    plat_world = plat_nodes + np.array([0, 0, height_m])
    leg_vecs = plat_world - base_nodes
    geom_home_lengths_m = np.linalg.norm(leg_vecs, axis=1)

    # absolute_length = geom_home + slide (m)
    abs_length_mm = (geom_home_lengths_m + slide_m) * 1000.0
    return abs_length_mm - geom.init_leg_lengths_mm


def _extensions_mm_to_slide_m(extensions_mm, geom):
    """Convert IK extensions (mm) to MuJoCo slide joint values (m).

    The IK defines: extension = absolute_leg_length - init_leg_lengths_mm.
    After STOW-zero refactor, init_leg_lengths_mm IS the geometric home
    length, so slide ≈ extension / 1000 (small rounding differences only).

    MuJoCo slide = 0 means legs at geometric home length (computed from nodes).
    So:  slide = (absolute_leg_length - geom_home) / 1000
              = (init_leg_lengths_mm + extension - geom_home_mm) / 1000
    """
    # Geometric home lengths from node positions
    base_nodes = geom.base_nodes / 1000.0
    plat_nodes = geom.plat_nodes / 1000.0
    height_m = geom.init_height_mm / 1000.0

    plat_world = plat_nodes + np.array([0, 0, height_m])
    leg_vecs = plat_world - base_nodes
    geom_home_lengths_m = np.linalg.norm(leg_vecs, axis=1)

    # IK absolute leg length = init_leg_lengths_mm + extension
    abs_length_m = (geom.init_leg_lengths_mm + extensions_mm) / 1000.0

    # MuJoCo slide = absolute length - geometric home length
    return abs_length_m - geom_home_lengths_m


def _command_and_settle(model, data, ctrl_values, settle_s=SETTLE_TIME_S):
    """Set actuator controls and step simulation until settled."""
    for i in range(6):
        data.ctrl[i] = ctrl_values[i]

    n_steps = int(settle_s / model.opt.timestep)
    for _ in range(n_steps):
        mujoco.mj_step(model, data)


def _compare_poses(pos_mujoco_mm, rot_mujoco, pos_fk_mm, rot_fk, test_name=''):
    """Compare MuJoCo pose to FK pose, return (pos_err_mm, ori_err_deg)."""
    pos_err = np.linalg.norm(pos_mujoco_mm - pos_fk_mm)
    # Orientation error: rotation between the two orientations
    R_diff = rot_mujoco.T @ rot_fk
    rotvec_diff = rot_matrix_to_rotvec(R_diff)
    ori_err_deg = np.degrees(np.linalg.norm(rotvec_diff))

    print(f"  [{test_name}] pos_err={pos_err:.4f} mm, ori_err={ori_err_deg:.4f} deg")
    print(f"    MuJoCo pos: {pos_mujoco_mm}")
    print(f"    FK pos:     {pos_fk_mm}")
    return pos_err, ori_err_deg


# ===========================================================================
# Tests
# ===========================================================================


class TestModelLoads:
    """Basic model loading and structure tests."""

    def test_load_model(self, model):
        """Model loads without error."""
        assert model is not None
        assert model.nq > 0  # has joints

    def test_has_actuators(self, model):
        """Model has 6 leg actuators + hand actuator."""
        assert model.nu == 7

    def test_has_sensors(self, model):
        """Model has the expected sensors."""
        for i in range(6):
            assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,
                                     f'slide_pos_{i}') >= 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,
                                 'platform_pos') >= 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR,
                                 'platform_quat') >= 0


class TestHomePosition:
    """Platform is at correct home position after reset."""

    def test_home_height(self, model, data):
        """Platform Z at home matches initial_height_mm."""
        pos_m = _get_sensor_data(model, data, 'platform_pos')
        height_mm = pos_m[2] * 1000.0
        assert abs(height_mm - 574.3) < 0.1, \
            f"Home height {height_mm:.2f} mm, expected 574.3 mm"

    def test_home_orientation(self, model, data):
        """Platform orientation at home is identity (no tilt)."""
        quat = _get_sensor_data(model, data, 'platform_quat')
        # Should be [1, 0, 0, 0] (identity)
        assert abs(quat[0] - 1.0) < 0.001
        assert np.linalg.norm(quat[1:]) < 0.001

    def test_home_xy(self, model, data):
        """Platform XY at home is (0, 0)."""
        pos_m = _get_sensor_data(model, data, 'platform_pos')
        assert abs(pos_m[0]) < 0.0001  # < 0.1 mm
        assert abs(pos_m[1]) < 0.0001


class TestFKValidation:
    """Command leg extensions, compare MuJoCo platform pose against IK solver FK."""

    # Test poses: [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    # All poses must have sufficient z (or small xy) to keep all leg extensions
    # positive (STOW-relative).  At STOW height (z=0), large xy translations
    # compress some legs below the hard stop (~-7mm).
    TEST_POSES = [
        # Pure translations
        ([0, 0, 0, 0, 0, 0], 'home'),
        ([0, 0, 50, 0, 0, 0], 'z+50mm'),
        ([0, 0, 100, 0, 0, 0], 'z+100mm'),
        ([50, 0, 50, 0, 0, 0], 'x+50_z+50'),
        ([0, 50, 50, 0, 0, 0], 'y+50_z+50'),
        ([-30, 40, 50, 0, 0, 0], 'xyz_offset'),
        # Pure rotations (at z=30 to stay above STOW)
        ([0, 0, 30, 5, 0, 0], 'pitch+5deg'),
        ([0, 0, 30, 0, 5, 0], 'roll+5deg'),
        ([0, 0, 30, 0, 0, 3], 'yaw+3deg'),
        # Combined
        ([0, 0, 50, 3, 0, 0], 'z50_pitch3'),
        ([30, -20, 80, 3, -2, 0], 'dynamic_target'),
        ([0, 0, 100, 5, 5, 0], 'high_tilt'),
        ([50, 50, 50, 2, 2, 2], 'all_dof'),
    ]

    @pytest.mark.parametrize('pose_deg,name', TEST_POSES,
                             ids=[p[1] for p in TEST_POSES])
    def test_fk_matches(self, model, data, geom, pose_deg, name):
        """MuJoCo platform pose matches analytical FK for the actual leg positions.

        This tests whether MuJoCo's parallel mechanism constraint solver produces
        the same platform pose as the analytical FK for the same leg extensions.
        """
        pose = np.array(pose_deg, dtype=float)
        pos_mm = pose[:3]
        rot_vec = np.radians(pose[3:])
        rot = rotvec_to_rot_matrix(rot_vec)

        # Get leg extensions from IK
        extensions_mm = pose_to_leg_lengths(pos_mm, rot, geom)

        # Check extensions are within stroke limits
        assert np.all(extensions_mm >= -1.0), \
            f"Extensions below zero: {extensions_mm}"
        assert np.all(extensions_mm <= geom.leg_stroke_mm + 1.0), \
            f"Extensions above stroke: {extensions_mm}"

        # Convert to MuJoCo slide joint values (meters)
        slide_targets_m = _extensions_mm_to_slide_m(extensions_mm, geom)

        # Command and settle
        _command_and_settle(model, data, slide_targets_m)

        # Read platform pose from MuJoCo
        pos_mj_mm, rot_mj = _get_platform_pose_from_mujoco(model, data)

        # Read actual slide positions and convert to IK extensions
        actual_slides_m = _get_slide_positions_m(model, data)
        actual_ext_mm = _slide_m_to_extensions_mm(actual_slides_m, geom)

        # Compute analytical FK from the actual leg extensions
        pos_fk, rot_fk = leg_lengths_to_pose(actual_ext_mm, geom)

        # Compare MuJoCo pose against analytical FK
        pos_err, ori_err = _compare_poses(pos_mj_mm, rot_mj, pos_fk, rot_fk, name)

        assert pos_err < POS_TOL_MM, \
            f"Position error {pos_err:.4f} mm > {POS_TOL_MM} mm for {name}"
        assert ori_err < ORI_TOL_DEG, \
            f"Orientation error {ori_err:.4f} deg > {ORI_TOL_DEG} deg for {name}"


class TestLegLimits:
    """Verify leg extension limits match hardware config."""

    def test_slide_range(self, model, geom):
        """Slide joint range covers at least leg_stroke_mm = 280 mm."""
        for i in range(6):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                         f'leg_{i}_slide')
            jnt_range = model.jnt_range[joint_id]
            stroke_m = jnt_range[1] - jnt_range[0]
            stroke_mm = stroke_m * 1000.0
            # Range includes a small margin on each end for measurement discrepancies
            assert stroke_mm >= geom.leg_stroke_mm, \
                f"Leg {i} stroke {stroke_mm:.2f} mm < required {geom.leg_stroke_mm} mm"
            # But shouldn't be excessively large
            assert stroke_mm <= geom.leg_stroke_mm + 20, \
                f"Leg {i} stroke {stroke_mm:.2f} mm unexpectedly large"


class TestConstraintSatisfaction:
    """Verify equality constraints remain satisfied during operation."""

    @staticmethod
    def _max_equality_violation(model, data):
        """Return the max violation across only equality constraint rows.

        data.efc_pos includes both equality and contact constraints.
        We filter to equality rows only so that contact penetration
        depths (from ball-hand collision) don't pollute the check.
        """
        if data.efc_pos.size == 0:
            return 0.0
        # efc_type tells us which rows are equality (type 0 in MuJoCo)
        eq_mask = data.efc_type == mujoco.mjtConstraint.mjCNSTR_EQUALITY
        if not np.any(eq_mask):
            return 0.0
        return float(np.max(np.abs(data.efc_pos[eq_mask])))

    def test_constraints_at_home(self, model, data):
        """Constraint violations at home position are negligible."""
        # Step a few times to let constraints settle
        for _ in range(100):
            mujoco.mj_step(model, data)
        max_violation = self._max_equality_violation(model, data)
        print(f"  Max constraint violation at home: {max_violation:.2e} m")
        assert max_violation < 0.001, \
            f"Constraint violation {max_violation:.2e} m too large at home"

    def test_constraints_at_offset(self, model, data, geom):
        """Constraint violations remain small after moving to offset pose."""
        pos_mm = np.array([30.0, -20.0, 50.0])
        rot = rotvec_to_rot_matrix(np.radians(np.array([3.0, -2.0, 0.0])))
        extensions_mm = pose_to_leg_lengths(pos_mm, rot, geom)
        slide_targets = _extensions_mm_to_slide_m(extensions_mm, geom)

        _command_and_settle(model, data, slide_targets, settle_s=3.0)

        max_violation = self._max_equality_violation(model, data)
        print(f"  Max constraint violation at offset: {max_violation:.2e} m")
        assert max_violation < 0.001, \
            f"Constraint violation {max_violation:.2e} m too large at offset"
