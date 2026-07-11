"""test_gui_geometry.py — Verify GUI geometry constants match hardware_config.yaml.

Reads geometry-config.js and compares every constant against the YAML source of truth.
This prevents drift between the GUI and the Python/C++ config.

Also pins other JS↔Python string contracts (regex-level tripwires), e.g. the
DiagnosticStatus KeyValue names shared by teensy_bridge_node and can-traffic.js.
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
        'js/can-traffic.js',
        'js/state-minimap.js',
        'css/state-minimap.css',
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


# ---- CAN traffic panel ↔ teensy_bridge_node KeyValue contract ----


BRIDGE_NODE_PY = (ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'jugglebot'
                  / 'teensy_bridge_node.py')
CAN_TRAFFIC_JS = ROOT / 'ros_ws' / 'gui' / 'js' / 'can-traffic.js'


@pytest.fixture(scope='module')
def bridge_py():
    """Read teensy_bridge_node.py as text."""
    return BRIDGE_NODE_PY.read_text()


@pytest.fixture(scope='module')
def can_traffic_js():
    """Read can-traffic.js as text."""
    return CAN_TRAFFIC_JS.read_text()


def _extract_method_source(py_text, name):
    """Slice one method body out of the node source (up to the next
    top-of-class ``def`` at 4-space indent)."""
    m = re.search(rf'\n    def {name}\(self\):(.*?)\n    def ', py_text, re.S)
    assert m, f'Could not find method {name} in teensy_bridge_node.py'
    return m.group(1)


def _keyvalue_keys(method_src):
    """All KeyValue(key='...') names published by a method body."""
    return set(re.findall(r"KeyValue\(key='([^']+)'", method_src))


def _strip_js_comments(js_text):
    """Crudely drop /* */ and // comments so key extraction only sees code.

    (can-traffic.js documents hypothetical future keys — e.g. the protocol-v4
    'can3'/'bus3_health' slots — in comments; those must not count as
    consumed.  Crude = no string-literal awareness, fine for this tripwire.)
    """
    js_text = re.sub(r'/\*.*?\*/', '', js_text, flags=re.S)
    return re.sub(r'//[^\n]*', '', js_text)


class TestCanTrafficKeyValueContract:
    """Pin the KeyValue-name contract: teensy_bridge_node (producer) ↔
    can-traffic.js (consumer).

    HONESTY NOTE — this is a string-level tripwire in the file's regex style,
    not a behavioural test: it regex-extracts the key names each side
    mentions and asserts consumer ⊆ producer.  It cannot prove the messages
    are published or parsed correctly; it catches silent renames on either
    side — the failure mode that leaves the panel permanently '--'/UNKNOWN
    with no error anywhere.  Each extraction is shape-asserted first so a JS
    refactor that breaks a regex fails loudly instead of passing vacuously.
    """

    def _consumed_profile_keys(self, js):
        # can-traffic.js builds profile keys as `${bus.slot}_<suffix>` from
        # the BUSES registry (slot: null ⇒ bus not on the uplink).
        js = _strip_js_comments(js)
        slots = set(re.findall(r"slot: '(\w+)'", js))
        assert slots == {'can1', 'can2'}, \
            f'BUSES slot extraction went stale: {slots}'
        suffixes = set(re.findall(r"kv\[`\$\{bus\.slot\}_(\w+)`\]", js))
        assert suffixes == {'rx', 'tx', 'util_pct'}, \
            f'profile-suffix extraction went stale: {suffixes}'
        return {f'{slot}_{suffix}' for slot in slots for suffix in suffixes}

    def _consumed_link_status_keys(self, js):
        js = _strip_js_comments(js)
        health_keys = set(re.findall(r"healthKey: '(\w+)'", js))
        assert health_keys == {'bus1_health', 'bus2_health'}, \
            f'healthKey extraction went stale: {health_keys}'
        direct = set(re.findall(r'\bkv\.(\w+)\b', js))
        assert 'bridge_link' in direct, \
            f'direct kv.<key> extraction went stale: {direct}'
        return health_keys | direct

    def test_profile_consumer_subset_of_producer(self, bridge_py, can_traffic_js):
        produced = _keyvalue_keys(
            _extract_method_source(bridge_py, '_publish_profile'))
        assert {'can1_rx', 'can1_tx', 'can2_rx', 'can2_tx',
                'can1_util_pct', 'can2_util_pct'} <= produced, \
            f'producer extraction went stale: {produced}'
        consumed = self._consumed_profile_keys(can_traffic_js)
        assert consumed <= produced, \
            f'can-traffic.js consumes profile keys the bridge never ' \
            f'publishes: {consumed - produced}'

    def test_link_status_consumer_subset_of_producer(self, bridge_py,
                                                     can_traffic_js):
        produced = _keyvalue_keys(
            _extract_method_source(bridge_py, '_publish_link_status'))
        assert {'bus1_health', 'bus2_health', 'bridge_link'} <= produced, \
            f'producer extraction went stale: {produced}'
        consumed = self._consumed_link_status_keys(can_traffic_js)
        assert consumed <= produced, \
            f'can-traffic.js consumes link_status keys the bridge never ' \
            f'publishes: {consumed - produced}'


# ---- State-minimap tripwires ----


STATE_MINIMAP_JS = ROOT / 'ros_ws' / 'gui' / 'js' / 'state-minimap.js'


@pytest.fixture(scope='module')
def minimap_js():
    """Read state-minimap.js as text."""
    return STATE_MINIMAP_JS.read_text()


class TestStateMinimapTripwires:
    """Crude, clearly-labelled string-level tripwires for the minimap.

    HONESTY NOTE — these are regex/substring tripwires, not behavioural
    tests (the behavioural gate is the synthetic-stack GUI probe harness).
    They pin (a) the asset wiring, (b) the SAFETY-CRITICAL teardown order
    (disarm strictly before deactivate — mvp_bench_runbook.md Sharp Edge
    #6: DEACTIVATE while mpc_active=1 latches MPC_STALE and leaves the
    legs un-stowed), and (c) the /link_status KeyValue names the minimap
    consumes against the teensy_bridge_node producer.
    """

    def test_index_links_minimap_assets(self):
        html = (ROOT / 'ros_ws' / 'gui' / 'index.html').read_text()
        assert 'css/state-minimap.css' in html, \
            'index.html no longer links css/state-minimap.css'
        assert 'id="state-minimap"' in html, \
            'index.html no longer contains the #state-minimap mount div'

    def test_teardown_disarms_before_deactivate(self, minimap_js):
        m = re.search(r'function teardownSteps\b(.*?)\n\}', minimap_js, re.S)
        assert m, 'teardownSteps() not found in state-minimap.js'
        body = m.group(1)
        i_disarm = body.find('set_setpoint_output')
        i_deact = body.find("'deactivate'")
        assert i_disarm != -1, 'teardownSteps() lost its disarm step'
        assert i_deact != -1, 'teardownSteps() lost its deactivate step'
        assert i_disarm < i_deact, \
            'teardown must disarm (set_setpoint_output false) BEFORE ' \
            'deactivate — runbook Sharp Edge #6'

    def test_teardown_go_home_before_disarm(self, minimap_js):
        m = re.search(r'function teardownSteps\b(.*?)\n\}', minimap_js, re.S)
        assert m, 'teardownSteps() not found in state-minimap.js'
        body = m.group(1)
        i_home = body.find('trajectory/go_home')
        i_disarm = body.find('set_setpoint_output')
        assert i_home != -1 and i_disarm != -1
        assert i_home < i_disarm, \
            'teardown order is go_home -> disarm -> deactivate (runbook:202-203)'

    def test_minimap_link_status_keys_subset_of_producer(self, bridge_py,
                                                         minimap_js):
        produced = _keyvalue_keys(
            _extract_method_source(bridge_py, '_publish_link_status'))
        # Derive the consumed set from the JS source (mirrors
        # TestCanTrafficKeyValueContract) so a future `kv.<new_key>` consumer
        # cannot silently escape the consumer ⊆ producer check; the shape
        # assertion makes a regex/refactor breakage fail loudly instead of
        # passing vacuously.
        consumed = set(re.findall(r'\bkv\.(\w+)\b',
                                  _strip_js_comments(minimap_js)))
        assert {'fault_state', 'mpc_active', 'bridge_link'} <= consumed, \
            f'kv.<key> extraction went stale: {consumed}'
        assert consumed <= produced, \
            f'state-minimap.js consumes link_status keys the bridge never ' \
            f'publishes: {consumed - produced}'
