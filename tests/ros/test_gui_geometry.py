"""test_gui_geometry.py — Verify GUI geometry constants match hardware_config.yaml.

Reads geometry-config.js and compares every constant against the YAML source of truth.
This prevents drift between the GUI and the Python/C++ config.
"""

import json
import re
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent.parent
GUI_JS_PATH = ROOT / 'ros_ws' / 'gui' / 'js' / 'geometry-config.js'
GENERATED_PY = ROOT / 'config' / 'generated' / 'hardware_config.py'

# hardware_config.yaml may be in the worktree or the main repo root
# (worktrees only contain files tracked by their branch)
_candidates = [
    ROOT / 'config' / 'hardware_config.yaml',
    ROOT.parent.parent.parent / 'config' / 'hardware_config.yaml',  # worktree → main repo
]
YAML_PATH = next((p for p in _candidates if p.exists()), _candidates[0])


@pytest.fixture(scope='module')
def yaml_config():
    """Load hardware_config.yaml."""
    with open(YAML_PATH) as f:
        return yaml.safe_load(f)


@pytest.fixture(scope='module')
def js_source():
    """Read geometry-config.js as text."""
    return GUI_JS_PATH.read_text()


def _extract_js_number(js_text, name):
    """Extract a numeric constant from JS: 'export const NAME = 123.4;'"""
    pattern = rf'export\s+const\s+{name}\s*=\s*([0-9.eE+-]+)\s*;'
    m = re.search(pattern, js_text)
    assert m, f'Could not find {name} in geometry-config.js'
    return float(m.group(1))


def _extract_js_array(js_text, name):
    """Extract a flat numeric array: 'export const NAME = [1.0, 2.0, ...];'"""
    pattern = rf'export\s+const\s+{name}\s*=\s*\[([\s\S]*?)\];'
    m = re.search(pattern, js_text)
    assert m, f'Could not find array {name} in geometry-config.js'
    body = m.group(1)
    # Remove comments and whitespace, parse as numbers
    numbers = re.findall(r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?', body)
    return [float(x) for x in numbers]


def _extract_js_2d_array(js_text, name):
    """Extract a 2D array: 'export const NAME = [[1,2,3], [4,5,6], ...];'"""
    pattern = rf'export\s+const\s+{name}\s*=\s*\[([\s\S]*?)\];'
    m = re.search(pattern, js_text)
    assert m, f'Could not find 2D array {name} in geometry-config.js'
    body = m.group(1)
    # Find each sub-array
    rows = re.findall(r'\[([^\]]+)\]', body)
    result = []
    for row in rows:
        nums = re.findall(r'[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?', row)
        result.append([float(x) for x in nums])
    return result


# ---- Scalar constants ----


class TestScalarConstants:
    def test_initial_height(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['initial_height_mm']
        js_val = _extract_js_number(js_source, 'INITIAL_HEIGHT_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_base_radius(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['base_radius_mm']
        js_val = _extract_js_number(js_source, 'BASE_RADIUS_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_plat_radius(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['plat_radius_mm']
        js_val = _extract_js_number(js_source, 'PLAT_RADIUS_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_leg_stroke(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['leg_stroke_mm']
        js_val = _extract_js_number(js_source, 'LEG_STROKE_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_ball_joint_offset(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['ball_joint_offset_mm']
        js_val = _extract_js_number(js_source, 'BALL_JOINT_OFFSET_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_arm_radius(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['arm_radius_mm']
        js_val = _extract_js_number(js_source, 'ARM_RADIUS_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_arm_height(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['arm_height_from_platform_mm']
        js_val = _extract_js_number(js_source, 'ARM_HEIGHT_FROM_PLATFORM_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_hand_stroke(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['hand_stroke_mm']
        js_val = _extract_js_number(js_source, 'HAND_STROKE_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_hand_radius(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['hand_radius_mm']
        js_val = _extract_js_number(js_source, 'HAND_RADIUS_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_leg_motor_max_pos(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['leg_motor_max_position_revs']
        js_val = _extract_js_number(js_source, 'LEG_MOTOR_MAX_POS_REVS')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_hand_motor_max_pos(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['hand_motor_max_position_revs']
        js_val = _extract_js_number(js_source, 'HAND_MOTOR_MAX_POS_REVS')
        assert js_val == pytest.approx(yaml_val, abs=0.01)


# ---- Array constants ----


class TestArrayConstants:
    def test_init_leg_lengths(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['init_leg_lengths_mm']
        js_val = _extract_js_array(js_source, 'INIT_LEG_LENGTHS_MM')
        assert len(js_val) == 6
        for i in range(6):
            assert js_val[i] == pytest.approx(yaml_val[i], abs=0.01), f'leg length {i}'

    def test_mm_to_rev(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['mm_to_rev']
        js_val = _extract_js_array(js_source, 'MM_TO_REV')
        assert len(js_val) == 6
        for i in range(6):
            assert js_val[i] == pytest.approx(yaml_val[i], abs=1e-8), f'mm_to_rev {i}'

    def test_base_nodes(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['base_nodes_mm']
        js_val = _extract_js_2d_array(js_source, 'BASE_NODES_MM')
        assert len(js_val) == 6
        for i in range(6):
            assert len(js_val[i]) == 3
            for j in range(3):
                assert js_val[i][j] == pytest.approx(yaml_val[i][j], abs=0.01), \
                    f'base_node[{i}][{j}]'

    def test_init_plat_nodes(self, yaml_config, js_source):
        yaml_val = yaml_config['jugglebot_geometry']['init_plat_nodes_mm']
        js_val = _extract_js_2d_array(js_source, 'INIT_PLAT_NODES_MM')
        assert len(js_val) == 6
        for i in range(6):
            assert len(js_val[i]) == 3
            for j in range(3):
                assert js_val[i][j] == pytest.approx(yaml_val[i][j], abs=0.01), \
                    f'plat_node[{i}][{j}]'


# ---- Derived constants cross-check ----


class TestDerivedConstants:
    def test_init_leg_lengths_with_offset(self, yaml_config, js_source):
        """Verify INIT_LEG_LENGTHS_WITH_OFFSET = INIT_LEG_LENGTHS + BALL_JOINT_OFFSET."""
        init_lengths = yaml_config['jugglebot_geometry']['init_leg_lengths_mm']
        offset = yaml_config['jugglebot_geometry']['ball_joint_offset_mm']
        expected = [l + offset for l in init_lengths]

        js_val = _extract_js_array(js_source, 'INIT_LEG_LENGTHS_WITH_OFFSET_MM')
        assert len(js_val) == 6
        for i in range(6):
            assert js_val[i] == pytest.approx(expected[i], abs=0.01), \
                f'leg_with_offset[{i}]'

    def test_base_node_radius(self, yaml_config):
        """Base nodes should lie on a circle of base_radius_mm."""
        radius = yaml_config['jugglebot_geometry']['base_radius_mm']
        for i, node in enumerate(yaml_config['jugglebot_geometry']['base_nodes_mm']):
            r = (node[0]**2 + node[1]**2)**0.5
            assert r == pytest.approx(radius, abs=0.5), f'base node {i} radius'

    def test_plat_node_radius(self, yaml_config):
        """Platform nodes should lie on a circle of plat_radius_mm."""
        radius = yaml_config['jugglebot_geometry']['plat_radius_mm']
        for i, node in enumerate(yaml_config['jugglebot_geometry']['init_plat_nodes_mm']):
            r = (node[0]**2 + node[1]**2)**0.5
            assert r == pytest.approx(radius, abs=0.5), f'plat node {i} radius'


# ---- File structure validation ----


class TestGUIFileStructure:
    """Verify all expected GUI files exist."""

    GUI_DIR = ROOT / 'ros_ws' / 'gui'

    EXPECTED_FILES = [
        'gui_server.py',
        'index.html',
        'favicon.svg',
        'css/theme.css',
        'css/viewer.css',
        'css/panels.css',
        'js/main.js',
        'js/ros-bridge.js',
        'js/geometry-config.js',
        'js/stewart-fk.js',
        'js/viewer.js',
        'js/stewart-model.js',
        'js/ball-butler-model.js',
        'js/panels.js',
        'js/commands.js',
        'lib/roslib.min.js',
    ]

    @pytest.mark.parametrize('filepath', EXPECTED_FILES)
    def test_file_exists(self, filepath):
        full = self.GUI_DIR / filepath
        assert full.exists(), f'Missing GUI file: {filepath}'


# ---- Ball Butler geometry constants ----


class TestBallButlerGeometry:
    def test_bb_yaw_s_offset(self, yaml_config, js_source):
        yaml_val = yaml_config['ball_butler_geometry']['yaw_s_offset_mm']
        js_val = _extract_js_number(js_source, 'BB_YAW_S_OFFSET_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_bb_pitch_d_offset(self, yaml_config, js_source):
        yaml_val = yaml_config['ball_butler_geometry']['pitch_d_offset_mm']
        js_val = _extract_js_number(js_source, 'BB_PITCH_D_OFFSET_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_bb_release_l_position(self, yaml_config, js_source):
        yaml_val = yaml_config['ball_butler_geometry']['release_l_position_mm']
        js_val = _extract_js_number(js_source, 'BB_RELEASE_L_POSITION_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_bb_pitch_z_offset(self, yaml_config, js_source):
        yaml_val = yaml_config['ball_butler_geometry']['pitch_z_offset_mm']
        js_val = _extract_js_number(js_source, 'BB_PITCH_Z_OFFSET_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.01)

    def test_bb_hand_stroke(self, yaml_config, js_source):
        yaml_val = yaml_config['ball_butler_trajectory']['hand_stroke_m'] * 1000
        js_val = _extract_js_number(js_source, 'BB_HAND_STROKE_MM')
        assert js_val == pytest.approx(yaml_val, abs=0.1)


# ---- Legacy file removal validation ----


class TestLegacyFilesRemoved:
    """Verify legacy GUI files have been removed."""

    GUI_DIR = ROOT / 'ros_ws' / 'gui'

    LEGACY_FILES = [
        'jugglebot_gui.html',
        '3dplotter.js',
        'convex_hull_points.json',
        'convex_hull_points_big.json',
        'package.json',
        'package-lock.json',
    ]

    @pytest.mark.parametrize('filepath', LEGACY_FILES)
    def test_legacy_removed(self, filepath):
        full = self.GUI_DIR / filepath
        assert not full.exists(), f'Legacy file still present: {filepath}'
