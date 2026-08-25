"""test_gui_geometry.py — Verify GUI geometry constants match hardware_config.yaml.

Reads geometry-config.js and compares every constant against the YAML source of truth.
This prevents drift between the GUI and the Python/C++ config.

Also pins other JS↔Python string contracts (regex-level tripwires), e.g. the
DiagnosticStatus KeyValue names shared by teensy_bridge_node and can-traffic.js.
"""

import json
import math
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
        yaml_val = yaml_config['jugglebot_geometry']['hand_motor_hard_stop_revs']
        js_val = _extract_js_number(js_source, 'HAND_MOTOR_HARD_STOP_REVS')
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
        'js/udp-traffic.js',
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


# ---- Chart display-unit conversions (telemetry-charts.js consumes these) ----


def _mm_per_rev_from_yaml(traj_cfg):
    """mm/rev for a spool axis, derived the way compute_derived() does.

    compute_derived emits a rev/m gain::

        gain = linear_gain_factor / (pi * hand_spool_radius_m * 2)

    and the GUI wants its reciprocal in mm::

        mm_per_rev = 1000 / gain

    Written out from the YAML here (not read back from the emitted file, which
    would make the test a tautology) so a spool-radius or gain-factor edit that
    never reaches the GUI fails this test.
    """
    gain_rev_per_m = (traj_cfg['linear_gain_factor']
                      / (math.pi * traj_cfg['hand_spool_radius_m'] * 2.0))
    return 1000.0 / gain_rev_per_m


class TestChartUnitConstants:
    """Pin the four constants telemetry-charts.js converts motor revs with.

    Why this matters more than a normal drift pin: the charts apply these at
    INGESTION, to measured AND commanded alike, so a wrong factor does not look
    wrong — the traces stay on top of each other and the operator reads a
    plausible, silently mis-scaled number off the axis.  The 3D hand had
    exactly that bug (it used the LEG factor, ~2.2x off) and it survived
    unnoticed until someone measured the model.
    """

    def test_hand_mm_per_rev(self, yaml_config, js_source):
        expected = _mm_per_rev_from_yaml(yaml_config['teensy_trajectory'])
        js_val = _extract_js_number(js_source, 'HAND_MM_PER_REV')
        assert js_val == pytest.approx(expected, rel=1e-6)

    def test_bb_hand_mm_per_rev(self, yaml_config, js_source):
        expected = _mm_per_rev_from_yaml(yaml_config['ball_butler_trajectory'])
        js_val = _extract_js_number(js_source, 'BB_HAND_MM_PER_REV')
        assert js_val == pytest.approx(expected, rel=1e-6)

    def test_hand_and_bb_hand_gains_differ(self, js_source):
        """The two spools are NOT the same radius (0.00521 vs 0.0052493 m).

        Guards the copy-paste failure where one constant is emitted twice: the
        difference is only ~4 %, small enough to look right on a chart and
        large enough to matter over a 280 mm stroke.
        """
        hand = _extract_js_number(js_source, 'HAND_MM_PER_REV')
        bb_hand = _extract_js_number(js_source, 'BB_HAND_MM_PER_REV')
        assert hand != pytest.approx(bb_hand, rel=1e-4)

    def test_bb_pitch_affine_constants(self, js_source):
        """deg = 90 + 360*rev — owned by BB firmware (PitchAxis.h), mirrored
        here and in teensy_bridge_node's _publish_bb_axis_estimates docstring.

        Not YAML-derived (this repo does not hold the BB firmware), so the pin
        is against the contract itself.
        """
        assert _extract_js_number(js_source, 'BB_PITCH_DEG_PER_REV') == pytest.approx(360.0)
        assert _extract_js_number(js_source, 'BB_PITCH_DEG_OFFSET') == pytest.approx(90.0)

    def test_bb_pitch_maps_onto_the_configured_range(self, yaml_config, js_source):
        """Sanity: the affine map has to land the barrel on the CONFIGURED range.

        Endpoints derive from ball_butler_pitch.deg_min/deg_max in the YAML —
        not hardcoded literals, which would drift silently if the config moved.
        rev 0 must be the deg_max (vertical) end, and the configured travel
        must be under a quarter motor turn (12-90 deg = 0.2167 rev) — a wrong
        per_rev (180, or a rad/deg slip) breaks the span check.
        """
        per_rev = _extract_js_number(js_source, 'BB_PITCH_DEG_PER_REV')
        offset = _extract_js_number(js_source, 'BB_PITCH_DEG_OFFSET')
        rng = yaml_config['ball_butler_pitch']
        assert offset == pytest.approx(rng['deg_max'], abs=0.1)
        span_rev = (rng['deg_max'] - rng['deg_min']) / per_rev
        assert 0.0 < span_rev < 0.25

    def test_hand_gain_spans_the_physical_stroke(self, yaml_config, js_source):
        """Sanity: hard stop x mm/rev must land inside the physical stroke.

        10.8 rev x 31.63 mm/rev = 341.6 mm against a 344.75 mm stroke — a
        wrong-axis factor (the leg's 70.5 mm/rev) would give 762 mm and fail.
        """
        mm_per_rev = _extract_js_number(js_source, 'HAND_MM_PER_REV')
        hard_stop = yaml_config['jugglebot_geometry']['hand_motor_hard_stop_revs']
        stroke = yaml_config['jugglebot_geometry']['hand_stroke_mm']
        travel = hard_stop * mm_per_rev
        assert 0.9 * stroke < travel <= stroke


# ---- Generated-copy identity ----


class TestGeneratedCopyIdentity:
    """The two geometry-config.js copies must be the same file.

    tests/firmware/test_config_drift.py pins each copy against a FRESH
    generator render, which implies identity transitively; this pins it
    directly, so it still fails if the generator itself cannot be imported
    (the drift module skips wholesale in that case) and it names the actual
    failure — the GUI is served from ros_ws/gui/js/ while every reader of the
    config tree looks at config/generated/.
    """

    def test_copies_are_byte_identical(self):
        generated = ROOT / 'config' / 'generated' / 'geometry-config.js'
        delivered = GUI_JS_PATH
        assert generated.exists(), f'missing {generated}'
        assert delivered.exists(), f'missing {delivered}'
        assert generated.read_bytes() == delivered.read_bytes(), (
            'config/generated/geometry-config.js and ros_ws/gui/js/'
            'geometry-config.js differ — re-run python config/generate_config.py')


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
UDP_TRAFFIC_JS = ROOT / 'ros_ws' / 'gui' / 'js' / 'udp-traffic.js'
PANELS_CSS = ROOT / 'ros_ws' / 'gui' / 'css' / 'panels.css'
CANBRIDGE_FW = ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'Teensy_code_canbridge'
TILT_CAL_GRID_PY = ROOT / 'tests' / 'hardware' / 'tilt_cal_grid.py'


@pytest.fixture(scope='module')
def udp_traffic_js():
    """Read udp-traffic.js as text."""
    return UDP_TRAFFIC_JS.read_text()


@pytest.fixture(scope='module')
def panels_css():
    """Read panels.css as text."""
    return PANELS_CSS.read_text()


@pytest.fixture(scope='module')
def bridge_py():
    """Read teensy_bridge_node.py as text."""
    return BRIDGE_NODE_PY.read_text()


@pytest.fixture(scope='module')
def can_traffic_js():
    """Read can-traffic.js as text."""
    return CAN_TRAFFIC_JS.read_text()


@pytest.fixture(scope='module')
def tilt_cal_grid_py():
    """Read the offline capture harness as text.

    It holds the THIRD copy of the latency_monitor rank table (the node and
    udp-traffic.js hold the other two), so it belongs in the same pin.
    """
    return TILT_CAL_GRID_PY.read_text()


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
        assert slots == {'can1', 'can2', 'can3'}, \
            f'BUSES slot extraction went stale: {slots}'
        suffixes = set(re.findall(r"kv\[`\$\{bus\.slot\}_(\w+)`\]", js))
        assert suffixes == {'rx', 'tx', 'util_pct'}, \
            f'profile-suffix extraction went stale: {suffixes}'
        return {f'{slot}_{suffix}' for slot in slots for suffix in suffixes}

    def _consumed_link_status_keys(self, js):
        js = _strip_js_comments(js)
        health_keys = set(re.findall(r"healthKey: '(\w+)'", js))
        assert health_keys == {'bus1_health', 'bus2_health', 'bus3_health'}, \
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


class TestUdpDiagKeyValueContract:
    """Pin the KeyValue-name contract: ``_publish_udp_diag`` (producer) ↔
    ``udp-traffic.js`` (consumer).

    Same honesty note as ``TestCanTrafficKeyValueContract`` above: this is a
    string-level tripwire, not a behavioural test.  It catches the failure mode
    that leaves the UDP view permanently '--' with no error anywhere — a rename
    on either side of a shared key.

    The per-type keys are built from an f-string on BOTH sides (producer:
    ``f'rx_{name}'`` over ``MsgType``; consumer: a regex over the arriving key
    set), so the producer set is RECONSTRUCTED here from the same MsgType
    inventory, with the f-string shapes shape-asserted so a producer switching
    to ids — or to lower-case names, which would collide with the aggregate
    row names — fails loudly instead of passing vacuously.
    """

    #: Direction/kind prefixes shared by the producer and the consumer regex.
    #: 'gap' left with the column it fed (2026-08-25): a per-type gap tally over
    #: a SHARED wire sequence counter measured type interleaving, not loss
    #: (logbook/2026-08-25-udp-gap-column-artifact.md).  Both ends move together
    #: or this class fails — that is the point of reconstructing the key set
    #: from the same tuple on both sides.
    PREFIXES = ('rx', 'tx')

    def _produced_keys(self, bridge_py):
        from teensy_link import MsgType

        src = _extract_method_source(bridge_py, '_publish_udp_diag')
        for prefix in self.PREFIXES:
            assert f"f'{prefix}_{{name}}'" in src or f"f'{prefix}_{{t.name}}'" in src, \
                f'producer no longer builds {prefix}_<TYPE_NAME> keys by ' \
                f'MsgType NAME (ids, or a renamed loop variable, would break ' \
                f'the consumer regex and the rosbag contract)'
        per_type = {f'{prefix}_{t.name}'
                    for prefix in self.PREFIXES for t in MsgType}
        literals = _keyvalue_keys(src)
        assert {'rx_frames', 'tx_frames', 'crc_errors'} <= literals, \
            f'aggregate extraction went stale: {literals}'
        return per_type | literals

    def _consumed_keys(self, js):
        from teensy_link import MsgType

        js = _strip_js_comments(js)
        # Per-type keys: the consumer matches them by pattern.  Pin the pattern
        # itself (prefix alternation + the UPPER-case name class that keeps
        # per-type rows distinguishable from the lower-case aggregates).
        m = re.search(r'PER_TYPE_KEY_RE = /\^\(([a-z|]+)\)_\(\[A-Z\]', js)
        assert m, 'PER_TYPE_KEY_RE extraction went stale'
        prefixes = tuple(m.group(1).split('|'))
        assert prefixes == self.PREFIXES, \
            f'consumer prefixes drifted from the producer: {prefixes}'
        per_type = {f'{prefix}_{t.name}' for prefix in prefixes for t in MsgType}

        # Aggregates: the explicit list plus every `newest.kv.<key>` /
        # `kv.<key>` read, so a future direct read cannot escape the check.
        agg = re.search(r'AGGREGATE_KEYS = \[(.*?)\]', js, re.S)
        assert agg, 'AGGREGATE_KEYS extraction went stale'
        listed = set(re.findall(r"'([a-z_]+)'", agg.group(1)))
        assert {'rx_frames', 'crc_errors', 'drain_capped'} <= listed, \
            f'AGGREGATE_KEYS extraction went stale: {listed}'
        direct = set(re.findall(r'\bkv\.([a-z_]+)\b', js))
        # bridge_link / heartbeat_age_ms come from link_status, not udp_diag —
        # they are pinned by TestCanTrafficKeyValueContract's link_status test.
        # latency_monitor is link_status too, pinned by
        # TestUdpPanelLatencyMonitor below; rtt_us is clock_diag, pinned by
        # TestUdpPanelAnchorRtt.
        direct -= {'bridge_link', 'heartbeat_age_ms', 'latency_monitor',
                   'rtt_us'}
        return per_type | listed | direct

    def test_udp_diag_consumer_subset_of_producer(self, bridge_py,
                                                  udp_traffic_js):
        produced = self._produced_keys(bridge_py)
        consumed = self._consumed_keys(udp_traffic_js)
        assert consumed <= produced, \
            f'udp-traffic.js consumes udp_diag keys the bridge never ' \
            f'publishes: {sorted(consumed - produced)}'

    def test_rate_column_is_msgs_per_second_never_hz(self, udp_traffic_js):
        """The unit must not read 'Hz'.

        The ROS view's Hz is a BROWSER-RECEIVED rate, capped at 5 for spied
        topics by ros-bridge.js's 200 ms throttle; this view is the true wire
        rate.  Two different quantities in one panel must not share a unit
        string, or an operator reads a saturated 5 as a wire rate.
        """
        js = _strip_js_comments(udp_traffic_js)
        # Scoped to the RENDERED column headers (the text between <th> tags),
        # not the whole file: the module legitimately says "1 Hz" about the
        # publish CADENCE, and a blanket ban would forbid explaining exactly
        # the distinction this test exists to protect.
        headers = re.findall(r'>([^<>]*)</th>', js)
        # 3, not 4: the per-type 'gaps' column was removed 2026-08-25 — that
        # counter is an artifact of the shared wire seq counter, so the panel
        # stopped rendering it (plans/archived/udp-channel-health.md).
        assert len(headers) >= 3, f'<th> extraction went stale: {headers}'
        rate_headers = [h for h in headers if 'msg/s' in h]
        assert len(rate_headers) == 2, \
            f'expected an rx and a tx msg/s column, got {headers}'
        for h in headers:
            assert 'Hz' not in h, \
                f"UDP column header {h!r} labels a rate 'Hz' — that unit " \
                f"belongs to the ROS view's browser-received rate"

    def test_seq_gaps_also_rides_link_status_beside_uptime_ms(self, bridge_py):
        """The gap counter must be readable on the SAME message as uptime_ms.

        A Teensy restart costs exactly one gap — its TX seq statics have no
        reset path (``udp_link.cpp:41-42``) — and nothing in the counter itself
        distinguishes that from a real dropped frame.  ``uptime_ms`` going
        backwards is the only separating signal, and it lives on /link_status,
        so a bag that carries the gaps at 1 Hz on /udp_diag and the uptime at
        10 Hz on /link_status cannot pair them tick-for-tick.  Both on
        /link_status is what makes the distinction reconstructable offline.
        """
        src = _extract_method_source(bridge_py, '_publish_link_status')
        keys = _keyvalue_keys(src)
        assert {'seq_gaps', 'uptime_ms'} <= keys, \
            'seq_gaps and uptime_ms must both be published on /link_status ' \
            f'so a peer reboot can be told from real loss; got {sorted(keys)}'

    def test_window_presets_never_include_one_second(self, udp_traffic_js):
        """/udp_diag publishes at 1 Hz, so a 1 s window holds ONE sample and
        can produce no rate at all — the CAN panel's documented lesson."""
        m = re.search(r'WINDOW_PRESETS = \[([\d,\s]+)\]', udp_traffic_js)
        assert m, 'WINDOW_PRESETS extraction went stale'
        presets = [int(x) for x in m.group(1).split(',') if x.strip()]
        assert presets == [5, 10, 30, 60], f'preset drift: {presets}'

    def test_main_routes_udp_diag_and_link_status(self, main_js):
        """Both feeds must reach the module: the counters, and the
        bridge_link gate that keeps a dead uplink from rendering as zeros."""
        assert "ros.subscribe('udp_diag'" in main_js, \
            'main.js no longer subscribes to udp_diag'
        assert 'udpTrafficOnDiag(msg)' in main_js, \
            'main.js no longer routes udp_diag to udp-traffic.js'
        m = re.search(r'function onLinkStatus\(msg\)\s*\{(.*?)\n\}', main_js, re.S)
        assert m, 'onLinkStatus not found in main.js'
        assert 'udpTrafficOnLinkStatus(msg)' in m.group(1), \
            'the UDP view lost its bridge_link gate — a dead uplink would ' \
            'render as a plausible table of zeros'


class TestUdpPanelLatencyMonitor:
    """Pin the latency verdict the UDP footer renders against the node source.

    Same honesty note as the two contract classes above: string-level
    tripwires, not behavioural tests.

    What they protect is the reason latency is on this panel at all.  The
    five-week uptime-lag arc (2026-07-18 → 2026-08-15, can-bridge FW 14) was a
    LATENCY failure that every loss counter read clean through, and the token
    the node publishes is the alarmed surface that arc left behind.  The rank
    order is CAUSAL, not severity, and the panel's "worst in session" is a max
    over it — so a token this GUI does not know, or a rank reordered on one
    side only, would leave the footer quietly naming the wrong condition: an
    operator pointed at the clamp duty when the ring leak upstream of it is
    what needs fixing.
    """

    def _node_tokens(self, bridge_py):
        """``{token: rank}`` as teensy_bridge_node.py defines them."""
        names = dict(re.findall(r"^(LATENCY_MONITOR_\w+) = '(\w+)'\s*$",
                                bridge_py, re.M))
        assert len(names) == 4, \
            f'token-constant extraction went stale: {names}'
        m = re.search(r'_LATENCY_MONITOR_RANK = \{(.*?)\n\}', bridge_py, re.S)
        assert m, '_LATENCY_MONITOR_RANK extraction went stale'
        ranks = re.findall(r'(LATENCY_MONITOR_\w+):\s*(\d+)', m.group(1))
        assert len(ranks) == 4, f'rank extraction went stale: {ranks}'
        return {names[n]: int(r) for n, r in ranks}

    def _js_tokens(self, udp_traffic_js):
        js = _strip_js_comments(udp_traffic_js)
        m = re.search(r'const LATENCY_MONITOR_RANK = \{(.*?)\n\};', js, re.S)
        assert m, 'LATENCY_MONITOR_RANK extraction went stale in udp-traffic.js'
        pairs = re.findall(r'(\w+):\s*(\d+)', m.group(1))
        assert len(pairs) == 4, f'JS rank extraction went stale: {pairs}'
        return {k: int(v) for k, v in pairs}

    def _harness_tokens(self, tilt_cal_grid_py):
        """``{token: rank}`` as the offline capture harness defines them.

        Its table mixes one NAME reference with three string literals, so both
        forms are resolved here rather than assumed.
        """
        ok = re.search(r"^LATENCY_MONITOR_OK = '(\w+)'\s*$",
                       tilt_cal_grid_py, re.M)
        assert ok, 'LATENCY_MONITOR_OK extraction went stale in tilt_cal_grid.py'
        m = re.search(r'^_LATENCY_MONITOR_RANK = \{(.*?)\}\s*$',
                      tilt_cal_grid_py, re.M | re.S)
        assert m, ('_LATENCY_MONITOR_RANK extraction went stale in '
                   'tilt_cal_grid.py')
        pairs = re.findall(r"(?:'(\w+)'|LATENCY_MONITOR_OK):\s*(\d+)",
                           m.group(1))
        assert len(pairs) == 4, f'harness rank extraction went stale: {pairs}'
        return {(name or ok.group(1)): int(rank) for name, rank in pairs}

    def test_tokens_and_rank_order_match_the_node(self, bridge_py,
                                                  udp_traffic_js):
        assert self._js_tokens(udp_traffic_js) == self._node_tokens(bridge_py)

    def test_all_three_rank_copies_agree(self, bridge_py, udp_traffic_js,
                                         tilt_cal_grid_py):
        """The node, the GUI and the offline harness each carry the table.

        Three copies exist because three runtimes need it and none can import
        the others (the harness must stay off ``rclpy``, the GUI is a browser
        module).  A reorder in one of them makes two tools name a DIFFERENT
        most-upstream cause from the same session, which is worse than either
        being wrong on its own — the operator has no way to tell which reading
        to act on.
        """
        node = self._node_tokens(bridge_py)
        assert self._js_tokens(udp_traffic_js) == node
        assert self._harness_tokens(tilt_cal_grid_py) == node

    def test_unknown_token_outranks_every_known_cause(self, udp_traffic_js,
                                                      tilt_cal_grid_py,
                                                      bridge_py):
        """An unrecognised token comes from a NEWER producer than the reader.

        Its place in the causal chain is unknowable to the reader, so it must
        never be reduced away — not as OK (silently "fine"), and not slotted
        between two named causes it might well sit upstream of.  Both readers
        that tolerate an unknown token rank it above the whole table, and the
        two constants must agree or worst-in-session means two things.
        """
        js = _strip_js_comments(udp_traffic_js)
        m = re.search(r'const LATENCY_MONITOR_UNKNOWN_RANK = (\d+);', js)
        assert m, 'LATENCY_MONITOR_UNKNOWN_RANK extraction went stale in JS'
        js_unknown = int(m.group(1))

        h = re.search(r'^_LATENCY_MONITOR_UNKNOWN_RANK = (\d+)\s*$',
                      tilt_cal_grid_py, re.M)
        assert h, ('_LATENCY_MONITOR_UNKNOWN_RANK extraction went stale in '
                   'tilt_cal_grid.py')
        assert js_unknown == int(h.group(1)), \
            'the GUI and the offline harness disagree on the unknown-token rank'
        assert js_unknown > max(self._node_tokens(bridge_py).values()), \
            'an unknown token no longer outranks every NAMED cause'

        # And the fallback actually USES the constant — a stray literal in
        # latencyRank would leave this pin green while the panel ranked an
        # unknown token below a named one.
        fn = re.search(r'function latencyRank\(token\)\s*\{(.*?)\n\}', js, re.S)
        assert fn, 'latencyRank extraction went stale'
        assert 'LATENCY_MONITOR_UNKNOWN_RANK' in fn.group(1), \
            'latencyRank no longer falls back to LATENCY_MONITOR_UNKNOWN_RANK'

    def test_node_publishes_the_token_on_link_status(self, bridge_py):
        """Unconditionally, and on link_status — the panel's only source."""
        src = _extract_method_source(bridge_py, '_publish_link_status')
        assert "KeyValue(key='latency_monitor'" in src, \
            'the node stopped publishing latency_monitor on /link_status — ' \
            'the UDP footer has no other source for it'

    def test_panel_reads_the_token_from_link_status(self, udp_traffic_js):
        js = _strip_js_comments(udp_traffic_js)
        m = re.search(r'export function udpTrafficOnLinkStatus\(msg\)\s*\{'
                      r'(.*?)\n\}', js, re.S)
        assert m, 'udpTrafficOnLinkStatus extraction went stale'
        assert 'kv.latency_monitor' in m.group(1), \
            'the UDP panel no longer reads the latency verdict'

    def test_ok_is_quiet_and_anything_else_is_prominent(self, udp_traffic_js,
                                                        panels_css):
        """Both render classes must exist in the stylesheet.

        A class named only in JS renders as unstyled body text — which for the
        alarm half means a RING_LEAK verdict that reads exactly like an OK one.
        """
        js = _strip_js_comments(udp_traffic_js)
        m = re.search(r'function latencyHtml\(\)\s*\{(.*?)\n\}', js, re.S)
        assert m, 'latencyHtml extraction went stale'
        body = m.group(1)
        for cls in ('udp-latency-ok', 'udp-latency-alarm'):
            assert cls in body, f'latencyHtml no longer applies .{cls}'
            assert f'.{cls}' in panels_css, f'panels.css has no .{cls} rule'


class TestUdpPanelAnchorRtt:
    """Pin the one live latency NUMBER the panel shows: clock_diag's rtt_us.

    String-level tripwires again.  ``rtt_us`` is the time-sync anchor's round
    trip (1–3 ms healthy) and is the only latency in milliseconds anything in
    this system publishes — it is what exonerated the transport during the
    uptime-lag arc, reading flat 1–3 ms at 63 h while the tracking lag was
    ~240 ms.  A silent rename or a dropped route puts a permanent '--' in the
    footer with no error anywhere.
    """

    def test_node_publishes_rtt_us_on_clock_diag(self, bridge_py):
        src = _extract_method_source(bridge_py, '_publish_clock_diag')
        assert "KeyValue(key='rtt_us'" in src, \
            'the node stopped publishing rtt_us on /clock_diag'

    def test_panel_reads_rtt_us(self, udp_traffic_js):
        js = _strip_js_comments(udp_traffic_js)
        m = re.search(r'export function udpTrafficOnClockDiag\(msg\)\s*\{'
                      r'(.*?)\n\}', js, re.S)
        assert m, 'udpTrafficOnClockDiag extraction went stale'
        assert 'kv.rtt_us' in m.group(1), \
            'the UDP panel no longer reads the anchor RTT'

    def test_main_routes_clock_diag(self, main_js):
        assert "ros.subscribe('clock_diag'" in main_js, \
            'main.js no longer subscribes to clock_diag'
        m = re.search(r'function onClockDiag\(msg\)\s*\{(.*?)\n\}', main_js, re.S)
        assert m, 'onClockDiag not found in main.js'
        assert 'udpTrafficOnClockDiag(msg)' in m.group(1), \
            'clock_diag no longer reaches the UDP panel'


class TestUdpNominalRates:
    """Pin the UDP panel's nominal-rate table against the FIRMWARE constants.

    The panel colours a msg/s cell when a steady flow drifts outside its
    nominal band, which makes those nominals a reference an operator trusts
    without re-deriving.  A firmware retune (TELEM_RATE_HZ, the hand poller's
    CHECK_INTERVAL_MS, HEARTBEAT_HZ) would leave that reference quietly wrong
    — an amber cell on a healthy link, or worse a quiet one on a sick link.
    This is the extract-both-sides-and-compare idiom the rest of this file
    uses, so the retune fails the suite instead.
    """

    @staticmethod
    def _table_block(js):
        """The NOMINAL_RATES literal, comments INTACT (a test below reads
        them: an uncommented number is a number nobody can re-derive)."""
        m = re.search(r'const NOMINAL_RATES = \{(.*?)\n\};', js, re.S)
        assert m, 'NOMINAL_RATES extraction went stale'
        return m.group(1)

    @classmethod
    def _entries(cls, js):
        """``{key: hz}`` from the JS table."""
        block = _strip_js_comments(cls._table_block(js))
        pairs = re.findall(r'(\w+):\s*\{\s*hz:\s*(\d+)', block)
        assert pairs, f'NOMINAL_RATES entry extraction went stale: {block!r}'
        return {k: int(v) for k, v in pairs}

    @staticmethod
    def _firmware_nominals():
        """The three firmware constants the table mirrors, as msg/s."""
        cfg = (CANBRIDGE_FW / 'canbridge_config.h').read_text()
        # The ACTIVE branch, not the BENCH_SYSID_BUILD override above it.
        m = re.search(r'#else\s*\n\s*constexpr uint32_t TELEM_RATE_HZ\s*=\s*(\d+)',
                      cfg)
        assert m, 'TELEM_RATE_HZ extraction went stale (the #else branch)'
        telem_hz = int(m.group(1))

        hw = (CANBRIDGE_FW / 'hardware_config.h').read_text()
        # Scoped to JBBallDetect: BBBallDetect has a CHECK_INTERVAL_MS too, and
        # it is a different robot's sensor.
        ns = re.search(r'namespace JBBallDetect \{(.*?)\n\}', hw, re.S)
        assert ns, 'JBBallDetect namespace extraction went stale'
        m = re.search(r'CHECK_INTERVAL_MS\s*=\s*(\d+)u?;', ns.group(1))
        assert m, 'JBBallDetect::CHECK_INTERVAL_MS extraction went stale'
        poll_ms = int(m.group(1))
        assert 1000 % poll_ms == 0, \
            f'poll interval {poll_ms} ms no longer divides a second evenly — ' \
            f'the JS nominal cannot stay an integer msg/s'

        proto = (CANBRIDGE_FW / 'udp_protocol.h').read_text()
        m = re.search(r'constexpr uint8_t HEARTBEAT_HZ\s*=\s*(\d+)u?;', proto)
        assert m, 'JbUdp::HEARTBEAT_HZ extraction went stale'
        return telem_hz, 1000 // poll_ms, int(m.group(1))

    def test_nominals_match_the_firmware_constants(self, udp_traffic_js):
        telem_hz, hand_hz, hb_hz = self._firmware_nominals()
        entries = self._entries(udp_traffic_js)
        assert entries == {
            # telemetry_step()'s unconditional back-to-back trio
            'rx_TELEMETRY': telem_hz,
            'rx_BB_AXIS_ESTIMATES': telem_hz,
            'rx_LEG_CMD': telem_hz,
            'rx_HAND_SENSOR': hand_hz,
            'rx_HEARTBEAT_T2J': hb_hz,
            'tx_HEARTBEAT_J2T': hb_hz,
            # SETPOINT has no firmware constant — it is the Jetson's own
            # trajectory stream rate, and it is bimodal (see below).
            'tx_SETPOINT': 40,
        }, 'the JS nominal table drifted from the firmware'

    def test_every_entry_names_its_firmware_source(self, udp_traffic_js):
        """The comments are the whole reason the numbers are auditable."""
        block = self._table_block(udp_traffic_js)
        for token in ('TELEM_RATE_HZ', 'canbridge_config.h',
                      'CHECK_INTERVAL_MS', 'hardware_config.h',
                      'HEARTBEAT_HZ', 'udp_protocol.h',
                      'telemetry.cpp'):
            assert token in block, \
                f'NOMINAL_RATES no longer cites {token} — a nominal nobody ' \
                f'can trace back to firmware is a number nobody can trust'

    def test_setpoint_stays_bimodal(self, udp_traffic_js):
        """0 and ~40 are BOTH healthy, so only an overshoot may be flagged.

        Policing the low side would paint the panel amber every time nothing
        is streaming, which is most of a session.
        """
        block = _strip_js_comments(self._table_block(udp_traffic_js))
        m = re.search(r'tx_SETPOINT:\s*\{([^}]*)\}', block)
        assert m, 'tx_SETPOINT entry extraction went stale'
        assert 'bimodal: true' in m.group(1), \
            'tx_SETPOINT lost its bimodal flag — an idle link would flag'

    def test_event_driven_types_have_no_nominal(self, udp_traffic_js):
        """A nominal on an event-driven type is a permanent false alarm."""
        entries = self._entries(udp_traffic_js)
        for name in ('RPC_REQUEST', 'RPC_RESPONSE', 'CMD_RESULT',
                     'HAND_CMD_ECHO', 'CONE_FRAME', 'PLATFORM_FRAME',
                     'DIAGNOSTIC'):
            for prefix in ('rx', 'tx'):
                assert f'{prefix}_{name}' not in entries, \
                    f'{prefix}_{name} is event-driven and cannot have a nominal'

    def test_table_keys_are_real_msgtypes(self, udp_traffic_js):
        """A typo'd key would simply never match and never flag anything."""
        from teensy_link import MsgType

        names = {t.name for t in MsgType}
        for key in self._entries(udp_traffic_js):
            prefix, _, name = key.partition('_')
            assert prefix in ('rx', 'tx'), f'{key} has no direction prefix'
            assert name in names, f'{key} names no MsgType'

    def test_the_deviation_class_is_rendered_and_styled(self, udp_traffic_js,
                                                        panels_css):
        js = _strip_js_comments(udp_traffic_js)
        assert 'rateDeviates(' in js, 'the conformance check is gone'
        assert 'udp-rate-off' in js, 'no cell can be marked as deviating'
        assert '.udp-rate-off' in panels_css, \
            'panels.css has no .udp-rate-off rule — a deviating cell would ' \
            'render exactly like a conforming one'


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


# ---- Ball Butler calibration stale-latch reset ----


MAIN_JS = ROOT / 'ros_ws' / 'gui' / 'js' / 'main.js'
PANELS_JS = ROOT / 'ros_ws' / 'gui' / 'js' / 'panels.js'


@pytest.fixture(scope='module')
def main_js():
    """Read main.js as text."""
    return MAIN_JS.read_text()


@pytest.fixture(scope='module')
def panels_js():
    """Read panels.js as text."""
    return PANELS_JS.read_text()


class TestBBCalibrationStaleLatchReset:
    """Regression tripwires for the BB "Calibrated" stale-latch bug.

    The Calibrated indicator is set by a *latched event* (bb/calibration_result,
    published by mocap_node only on a calibration attempt) — nothing re-publishes
    it to signal that a ROS session ended. In a long-lived GUI tab that outlives
    ROS relaunches the previous session's "Calibrated" therefore persists into a
    fresh, uncalibrated session. The fix resets the indicator on the GUI↔rosbridge
    drop (onConnectionStateChange 'disconnected'), the same axis used for the
    cone catch-counter re-baseline. These are string-level tripwires, not
    behavioural tests — they pin that the reset call survives future refactors.
    """

    def test_panels_exports_reset_bb_calibration(self, panels_js):
        assert re.search(r'export\s+function\s+resetBBCalibration\b', panels_js), \
            'panels.js no longer exports resetBBCalibration()'

    def test_reset_bb_calibration_shows_not_calibrated(self, panels_js):
        m = re.search(r'export\s+function\s+resetBBCalibration\b(.*?)\n\}',
                      panels_js, re.S)
        assert m, 'resetBBCalibration() body not found'
        body = m.group(1)
        assert 'Not Calibrated' in body, \
            'resetBBCalibration() must reset the text to "Not Calibrated"'
        assert 'uncalibrated' in body, \
            'resetBBCalibration() must set the dot to the uncalibrated class'

    def test_main_imports_reset_bb_calibration(self, main_js):
        assert 'resetBBCalibration' in main_js, \
            'main.js no longer imports/uses resetBBCalibration'

    def test_disconnect_branch_resets_bb_calibration(self, main_js):
        # Isolate the 'disconnected' case body inside onConnectionStateChange.
        m = re.search(r"case 'disconnected':(.*?)break;", main_js, re.S)
        assert m, "onConnectionStateChange 'disconnected' branch not found"
        body = m.group(1)
        assert 'resetBBCalibration()' in body, \
            'the disconnect branch must call resetBBCalibration() so a fresh ' \
            'session does not inherit the previous session\'s "Calibrated"'
        # Pin that the cone reset (yesterday's parallel fix) is still colocated —
        # both resets share the same stale-latch rationale and the same axis.
        assert 'setCatchingConeDisconnected()' in body, \
            'the disconnect branch lost the cone re-baseline'


# ---- Ball-in-hand pill ↔ HandTelemetryMessage field-name contract ----


HAND_TELEM_MSG = (ROOT / 'ros_ws' / 'src' / 'jugglebot_interfaces' / 'msg'
                  / 'HandTelemetryMessage.msg')


@pytest.fixture(scope='module')
def hand_telem_msg():
    """Read HandTelemetryMessage.msg as text."""
    return HAND_TELEM_MSG.read_text()


class TestBallPillFieldNameContract:
    """Pin the tri-state field names shared by the .msg definition (producer)
    and the GUI ball-in-hand pill (consumer).

    HONESTY NOTE — a string-level tripwire in this file's regex style, not a
    behavioural test. It cannot prove the pill renders correctly; it catches
    the silent-rename failure mode: rosbridge delivers plain JSON, so a field
    renamed in the .msg leaves ``msg.ball_held_valid`` permanently
    ``undefined`` in panels.js — which coerces to false, and the pill sits at
    UNKNOWN forever with no error anywhere. That is indistinguishable at a
    glance from a genuinely dead sensor, during a session whose whole point is
    reading this pill (Phase 7 step 2's live surface).
    """

    FIELDS = ['ball_held', 'ball_held_raw', 'ball_held_valid']

    @pytest.mark.parametrize('field', FIELDS)
    def test_field_in_msg_definition(self, field, hand_telem_msg):
        assert re.search(rf'^bool {field}\b', hand_telem_msg, re.M), \
            f'HandTelemetryMessage.msg no longer declares "bool {field}"'

    @pytest.mark.parametrize('field', FIELDS)
    def test_field_consumed_by_panels_js(self, field, panels_js):
        assert re.search(rf'\bmsg\.{field}\b', panels_js), \
            f'panels.js no longer reads msg.{field} — the pill would render ' \
            f'from an undefined field (silently UNKNOWN forever)'

    def test_ball_held_raw_is_distinguishable(self, panels_js):
        # ball_held is a prefix of the other two, so the parametrised checks
        # above would pass on `msg.ball_held_raw` alone. Pin the bare read.
        assert re.search(r'\bmsg\.ball_held\b(?!_)', panels_js), \
            'panels.js no longer reads the bare msg.ball_held verdict'


# ---- ODrive error-table drift pins (F3) ----


ODRIVE_ERRORS_JS = ROOT / 'ros_ws' / 'gui' / 'js' / 'odrive-errors.js'
ODRIVE_PY = (ROOT / 'ros_ws' / 'src' / 'jugglebot' / 'jugglebot'
             / 'can' / 'odrive.py')
SUPPORTED_PLATFORM_PY = ROOT / 'tests' / 'hardware' / 'supported_platform_test.py'
SINGLE_LEG_PY = ROOT / 'tests' / 'hardware' / 'single_leg_test.py'
FREE_PLATFORM_PY = ROOT / 'tests' / 'hardware' / 'free_platform_test.py'
TILT_CAL_GRID_PY = ROOT / 'tests' / 'hardware' / 'tilt_cal_grid.py'

# Harness copies whose vendored DECODER (not just the table) is pinned against
# the canonical one, executed for real. All three now honour the contract:
# single_leg_test.py's copy was the last one dropping unknown bits (a bare
# comprehension over ERROR_CODES), fixed 2026-08-25 with the rest of the class.
VENDORED_DECODERS = [
    (SUPPORTED_PLATFORM_PY, 'supported_platform_test.py'),
    (FREE_PLATFORM_PY, 'free_platform_test.py'),
    (SINGLE_LEG_PY, 'single_leg_test.py'),
]

# Harnesses whose REPORT sites must read both masks. Same list, different
# property: a harness can decode perfectly and still print only active_errors.
BOTH_MASK_HARNESSES = [
    (FREE_PLATFORM_PY, 'free_platform_test.py'),
    (SUPPORTED_PLATFORM_PY, 'supported_platform_test.py'),
    (SINGLE_LEG_PY, 'single_leg_test.py'),
]


def _extract_py_dict(py_text, name, source_label, nested=False):
    """Extract a module-level dict literal by AST, without importing.

    Parsing beats importing here: ``can/odrive.py`` pulls the generated config
    (and, transitively on some boxes, python-can), while the hardware harnesses
    open CAN interfaces from ``__main__`` argument parsing. The pin only needs
    the literal.

    ``nested=True`` walks the whole tree instead of the module body, for a copy
    that lives inside a ``try: import … except ImportError:`` fallback — which
    is precisely the copy an import-based pin can never see (see
    ``test_tilt_cal_grid_fallback_table_matches_python``).
    """
    import ast
    tree = ast.parse(py_text)
    for node in (ast.walk(tree) if nested else tree.body):
        targets = (node.targets if isinstance(node, ast.Assign)
                   else [node.target] if isinstance(node, ast.AnnAssign) else [])
        for t in targets:
            if isinstance(t, ast.Name) and t.id == name:
                return ast.literal_eval(node.value)
    raise AssertionError(f'Could not find dict {name} in {source_label}')


def _load_error_names(py_text, source_label):
    """Compile a file's ERROR_CODES + error_names WITHOUT importing the module.

    Same reason as ``_extract_py_dict`` — importing a harness opens a CAN bus
    — but this lifts the DECODER too, so the vendored copies can be pinned on
    BEHAVIOUR (what an operator actually reads) and not only on the table. The
    two definitions are self-contained: ``error_names`` closes over nothing but
    ``ERROR_CODES``, in every copy.
    """
    import ast
    tree = ast.parse(py_text)
    wanted = []
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == 'error_names':
            wanted.append(node)
        elif isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'ERROR_CODES'
                for t in node.targets):
            wanted.append(node)
    assert len(wanted) == 2, (
        f'Expected exactly one ERROR_CODES and one error_names in '
        f'{source_label}; the AST walk found {len(wanted)} node(s)')
    wanted.sort(key=lambda n: n.lineno)
    module = ast.Module(body=wanted, type_ignores=[])
    namespace = {}
    exec(compile(ast.fix_missing_locations(module), f'<{source_label}>',
                 'exec'), namespace)
    return namespace['error_names']


def _extract_js_int_string_map(js_text, name):
    """Extract 'export const NAME = { 0x1: 'A', 2: 'B', };' as {int: str}."""
    pattern = rf'export\s+const\s+{name}\s*=\s*\{{([\s\S]*?)\}}\s*;'
    m = re.search(pattern, js_text)
    assert m, f'Could not find map {name} in odrive-errors.js'
    body = m.group(1)
    pairs = re.findall(r'(0[xX][0-9a-fA-F]+|\d+)\s*:\s*[\'"]([A-Z0-9_]+)[\'"]', body)
    assert pairs, f'{name} entry extraction went stale (0 pairs parsed)'
    return {int(k, 0): v for k, v in pairs}


@pytest.fixture(scope='module')
def authoritative_error_codes():
    """jugglebot.can.odrive.ERROR_CODES — the one true table."""
    table = _extract_py_dict(ODRIVE_PY.read_text(), 'ERROR_CODES', 'can/odrive.py')
    # Shape assertions so a refactor that breaks the AST walk fails loudly
    # instead of pinning every copy against an empty dict.
    assert len(table) >= 20, f'ERROR_CODES extraction went stale: {table}'
    assert table[0x4000000] == 'SPINOUT_DETECTED'
    assert table[512] == 'DC_BUS_UNDER_VOLTAGE'
    return table


class TestODriveErrorTablePins:
    """Pin every hand-mirrored copy of ERROR_CODES against can/odrive.py.

    WHY THIS EXISTS. There is no codegen for this table, so each consumer that
    cannot import the ROS package carries a copy, and a copy drifts silently:
    the decode does not fail, it returns the WRONG NAME (or none) for a fault
    the operator is standing next to. That is not hypothetical — before this
    pin, ``supported_platform_test.py`` mapped 0x1000000 to 'ESTOP_REQUESTED'
    (it is WATCHDOG_TIMER_EXPIRED) and stopped at 0x10000, so SPINOUT_DETECTED
    — the 2026-08-10 leg-0 fault — decoded to nothing at all on that bench.

    ``tilt_cal_grid``'s DECODER is pinned separately, in
    ``tests/motion/test_tilt_cal_grid.py``, next to its own decoder tests —
    it imports the tool for real, so it pins behaviour better than an AST lift
    could. Its vendored FALLBACK table is pinned here instead, for the reason
    in that test's docstring.
    """

    def test_gui_js_table_matches_python(self, authoritative_error_codes):
        js_table = _extract_js_int_string_map(
            ODRIVE_ERRORS_JS.read_text(), 'ODRIVE_ERROR_CODES')
        assert js_table == authoritative_error_codes, \
            'odrive-errors.js has drifted from jugglebot.can.odrive.ERROR_CODES'

    def test_supported_platform_harness_table_matches_python(
            self, authoritative_error_codes):
        table = _extract_py_dict(SUPPORTED_PLATFORM_PY.read_text(),
                                 'ERROR_CODES', 'supported_platform_test.py')
        assert table == authoritative_error_codes

    def test_single_leg_harness_table_matches_python(
            self, authoritative_error_codes):
        table = _extract_py_dict(SINGLE_LEG_PY.read_text(),
                                 'ERROR_CODES', 'single_leg_test.py')
        assert table == authoritative_error_codes

    def test_free_platform_harness_table_matches_python(
            self, authoritative_error_codes):
        """The Stage-C harness copy — unpinned until 2026-08-25, and drifted.

        It stopped at 0x1000000 and called that bit 'ESTOP_REQUESTED' (it is
        WATCHDOG_TIMER_EXPIRED), missing the five bits above it — so on the one
        bench where the platform stands free, a watchdog expiry named the wrong
        cause and SPINOUT_DETECTED named nothing.
        """
        table = _extract_py_dict(FREE_PLATFORM_PY.read_text(),
                                 'ERROR_CODES', 'free_platform_test.py')
        assert table == authoritative_error_codes

    def test_tilt_cal_grid_fallback_table_matches_python(
            self, authoritative_error_codes):
        """The one copy an import-based pin structurally cannot see.

        ``tilt_cal_grid`` prefers the real ``jugglebot.can.odrive.ERROR_CODES``
        and vendors a literal only in the ``except ImportError`` arm (its
        package ``__init__`` needs jugglebot_interfaces + python-can, absent on
        an off-robot checkout). So ``tests/motion/test_tilt_cal_grid.py``'s
        ``tcg.ODRIVE_ERROR_CODES == odrive.ERROR_CODES`` is a TAUTOLOGY on every
        box where the import succeeds — this Jetson included — and the fallback
        literal could drift untouched until the day it is the only table there
        is, i.e. off-robot forensics on a dumped snapshot. Lifting the literal
        by AST pins it regardless of what the import does here.

        Its decoder (``decode_error_bits``) is NOT in VENDORED_DECODERS: it
        takes a different name, a different table name, and renders residue as
        ``UNKNOWN_BITS_0x%08X`` rather than ``UNKNOWN(0x…)`` — a JSON-forensics
        label, pinned behaviourally in tests/motion. It already honours both
        halves of the contract (residue surfaced; both masks decoded and
        printed at every dump site), so nothing here needs to change it.
        """
        table = _extract_py_dict(TILT_CAL_GRID_PY.read_text(),
                                 'ODRIVE_ERROR_CODES', 'tilt_cal_grid.py',
                                 nested=True)
        assert table == authoritative_error_codes

    def test_js_unknown_bit_contract_is_stated(self):
        """Regex tripwire (honesty note: it cannot execute the JS).

        Python's ``error_names`` surfaces undecodable bits as one aggregate
        ``UNKNOWN(0x…)`` entry rather than dropping them — a firmware code this
        build has never seen must still reach the operator. Pin that the JS
        mirror still implements the same escape hatch; losing it is a silent
        divergence that only shows up on the day it matters.
        """
        js = ODRIVE_ERRORS_JS.read_text()
        assert 'UNKNOWN(0x' in js, \
            'odrive-errors.js dropped the UNKNOWN(0x…) fallback'
        assert re.search(r'\bexport\s+function\s+errorNames\s*\(', js), \
            'odrive-errors.js no longer exports errorNames()'

    @pytest.mark.parametrize('path,label', VENDORED_DECODERS,
                             ids=[label for _, label in VENDORED_DECODERS])
    def test_vendored_decoder_matches_canonical(self, path, label):
        """Executed, not grepped: the vendored decode must equal canonical.

        The table pins above compare dicts, which says nothing about what the
        decoder DOES with them. Each copy is lifted out by AST and run
        (importing the harness would open a CAN bus), so a copy that keeps the
        right table but decodes it differently still fails here.
        """
        canonical = _load_error_names(ODRIVE_PY.read_text(), 'can/odrive.py')
        vendored = _load_error_names(path.read_text(), label)
        masks = [0, 0x200, 0x1000000, 0x2000000, 0x4000000,
                 0x200 | 0x4000000, 0xFFFFFFFF]
        for mask in masks:
            assert vendored(mask) == canonical(mask), \
                f'{label} decodes 0x{mask:X} differently from can/odrive.py'

    @pytest.mark.parametrize('path,label', VENDORED_DECODERS,
                             ids=[label for _, label in VENDORED_DECODERS])
    def test_vendored_decoder_keeps_unknown_bits(self, path, label):
        """The unknown-bit contract, for the Python copies (cf. the JS one).

        A bench harness that drops a bit it does not recognise reports 'no
        errors' for a fault the ODrive IS raising — the worst possible output
        for the one operator standing next to a free-standing platform. Bit 7
        (0x80) is unassigned in every published table, same probe as
        tests/ros/test_odrive.py.
        """
        vendored = _load_error_names(path.read_text(), label)
        assert vendored(0) == []
        assert vendored(0x80) == ['UNKNOWN(0x80)'], \
            f'{label} silently drops unknown error bits'
        assert vendored(0x200 | 0x80) == ['DC_BUS_UNDER_VOLTAGE',
                                          'UNKNOWN(0x80)']

    @pytest.mark.parametrize('path,label', BOTH_MASK_HARNESSES,
                             ids=[label for _, label in BOTH_MASK_HARNESSES])
    def test_harness_reports_both_masks(self, path, label):
        """String tripwire: every bench report reads disarm_reason too.

        All three harnesses unpacked disarm_reason into their AxisState from
        the start and printed it at NO report site — free_platform at nine
        sites, supported_platform at eleven, single_leg at four. active_errors
        self-heals; disarm_reason is sticky until CLEAR_ERRORS — so the
        2026-08-10 leg-0 signature (SPINOUT_DETECTED surviving only in the
        disarm mask) printed nothing at all on any bench. Same 'BOTH MASKS,
        ALWAYS' rule teensy_bridge_node._decode_axis_errors carries.

        Honesty note: string-level, in this file's house style. It cannot prove
        a given print statement renders both masks; it pins the two-mask
        decoder, the AxisState property that feeds it both, and the absence of
        the one-mask idiom that made the class silent.
        """
        src = path.read_text()
        assert 'def decode_axis_errors(' in src, \
            f'{label} lost its two-mask decoder'
        assert 'decode_axis_errors(self.active_errors, self.disarm_reason)' \
            in src, f'{label}: fault_summary no longer feeds it both masks'
        assert not re.search(r'error_names\([^)]*active_errors', src), \
            (f'a {label} report site decodes active_errors alone again — '
             f'use fault_summary (both masks)')

    @pytest.mark.parametrize('path,label', BOTH_MASK_HARNESSES,
                             ids=[label for _, label in BOTH_MASK_HARNESSES])
    def test_harness_waits_for_a_post_clear_error_frame(self, path, label):
        """The other half of the both-masks fix, and it is load-bearing.

        Gating on the STICKY mask is only safe if the mask is known fresh.
        Each harness clears errors then immediately asserts a clean bus
        (require_no_errors*), but the cached masks are only ever as new as the
        last Get_Error frame — so without a bounded wait for a POST-clear frame
        a latch from an EARLIER session reads as a live fault and aborts the
        very test that just cleared it. The counter is the freshness proof;
        ERROR_FRESH_TIMEOUT_S bounds it, so a bus not broadcasting Get_Error
        degrades to the old behaviour instead of hanging.
        """
        src = path.read_text()
        assert 'ERROR_FRESH_TIMEOUT_S' in src, \
            f'{label} lost the bounded post-CLEAR_ERRORS wait'
        assert 'error_frame_count' in src, \
            f'{label} lost the Get_Error frame counter the wait watches'

    def test_main_js_consumes_the_decoder(self):
        """The table is only useful if the event log actually decodes with it —
        pin the import so a refactor cannot leave a dead module behind a
        passing drift test."""
        main_js = (ROOT / 'ros_ws' / 'gui' / 'js' / 'main.js').read_text()
        assert re.search(r"from\s+'\./odrive-errors\.js'", main_js), \
            'main.js no longer imports the ODrive error decoder'

    def test_fault_headline_names_the_error(self):
        """String tripwire: the event-log headline says ODRIVE ERROR, loudly.

        The row is read at a glance and the decode only lives in the hover
        title, so a headline that degrades back to a bare axis label ('ODrive:
        hand') costs the operator the one word that says what broke.
        """
        main_js = (ROOT / 'ros_ws' / 'gui' / 'js' / 'main.js').read_text()
        assert 'ODRIVE ERROR: ' in main_js, \
            'main.js event-log fault headline lost its ODRIVE ERROR prefix'


class TestBBCalibrateMocapGate:
    """The Calibrate button's mocap precondition (F4/Q4).

    The calibrate COMMAND path (button -> bb/calibrate -> BB firmware sweeps)
    and the calibrate DATA path (QTM -> mocap_node -> the circle fit) share no
    edge. With QTM down, pressing Calibrate used to run a full physical sweep
    that produced nothing — or, with a partly-visible constellation, a
    plausible BB pose every subsequent throw was then aimed with.
    teensy_bridge_node now refuses such a call outright (QTM_STALE /
    BB_MARKERS_NOT_VISIBLE, no RPC dispatched); this button rule is the cheaper
    half of the same gate, so the operator learns why BEFORE pressing.

    String-level tripwires, in the house style for this file — there is no JS
    runtime in this suite. They pin that the rule and its tooltip survive a
    refactor, not that the DOM ends up in a particular state.
    """

    def test_panels_tracks_mocap_connected_state(self, panels_js):
        assert re.search(r'\blet\s+mocapConnected\b', panels_js), \
            'panels.js no longer keeps a mocapConnected flag for the gate'

    def test_set_mocap_connected_updates_the_flag_and_regates(self, panels_js):
        m = re.search(r'export\s+function\s+setMocapConnected\b(.*?)\n\}',
                      panels_js, re.S)
        assert m, 'setMocapConnected() body not found'
        body = m.group(1)
        assert 'mocapConnected' in body, \
            'setMocapConnected() must record the state the gate reads'
        assert 'applyBBCalibrateGate()' in body, \
            'setMocapConnected() must re-evaluate the Calibrate gate — this ' \
            'flag flips on the mocap_data watchdog, so nothing else would'

    def test_calibrate_gate_disables_on_mocap_disconnect_with_a_reason(self, panels_js):
        m = re.search(r'function\s+applyBBCalibrateGate\b(.*?)\n\}',
                      panels_js, re.S)
        assert m, 'applyBBCalibrateGate() body not found'
        body = m.group(1)
        assert '!mocapConnected' in body, \
            'the Calibrate gate no longer considers the mocap connection'
        assert 'disabled = true' in body, \
            'the Calibrate gate no longer disables the button'
        # A greyed button with no explanation is the failure mode this is meant
        # to prevent, not a milder version of it.
        assert re.search(r"title\s*=\s*'Mocap disconnected", body), \
            'the Calibrate gate must name mocap as the blocking reason in the tooltip'

    def test_calibrate_gate_checks_bb_connection_first(self, panels_js):
        """A live re-enable path, closed.

        setBBDisconnected() correctly disables the button — but lastBBState is a
        last-value cache it did not invalidate, so it still held 1 from the
        final IDLE heartbeat. mocap_data keeps arriving after Ball Butler goes
        away, and every frame calls setMocapConnected(true) ->
        applyBBCalibrateGate(), which saw bbIdle && mocapConnected and RE-ENABLED
        Calibrate on a Ball Butler that is not there. Not a corner case: it
        happens within milliseconds of every BB disconnect.
        """
        m = re.search(r'function\s+applyBBCalibrateGate\b(.*?)\n\}',
                      panels_js, re.S)
        assert m, 'applyBBCalibrateGate() body not found'
        body = m.group(1)
        assert 'bbConnected' in body, \
            'the Calibrate gate no longer considers whether BB is connected'
        assert re.search(r"!bbConnected", body), \
            'the gate must test !bbConnected explicitly'
        # FIRST, before the IDLE and mocap branches: an absent BB is the most
        # specific reason and must be the one the tooltip names.
        assert body.index('!bbConnected') < body.index('!mocapConnected'), \
            'the BB-connection branch must precede the mocap branch'

    def test_panels_tracks_bb_connected_state(self, panels_js):
        assert re.search(r'\blet\s+bbConnected\b', panels_js), \
            'panels.js no longer keeps a bbConnected flag for the gate'

    def test_bb_disconnect_invalidates_both_gate_inputs(self, panels_js):
        """setBBDisconnected must clear BOTH pieces of state the gate reads.
        Leaving lastBBState at its final IDLE value is what let the next mocap
        frame re-enable the button."""
        m = re.search(r'export\s+function\s+setBBDisconnected\b(.*?)\n\}\n',
                      panels_js, re.S)
        assert m, 'setBBDisconnected() body not found'
        body = m.group(1)
        assert re.search(r'bbConnected\s*=\s*false', body), \
            'setBBDisconnected() must clear bbConnected'
        assert re.search(r'lastBBState\s*=\s*-1', body), \
            'setBBDisconnected() must invalidate the cached BB state'

    def test_bb_heartbeat_sets_the_connection_flag(self, panels_js):
        """…and the connect edge must set it again, or Calibrate never re-enables."""
        m = re.search(r'export\s+function\s+updateBBPanel\b(.*?)\n\}\n',
                      panels_js, re.S)
        assert m, 'updateBBPanel() body not found'
        assert re.search(r'bbConnected\s*=\s*true', m.group(1)), \
            'updateBBPanel() must set bbConnected on the connection edge'

    def test_calibrate_gate_leaves_the_reset_button_alone(self, panels_js):
        """BB-ERROR reset has nothing to do with mocap and must stay pressable
        with QTM down."""
        m = re.search(r'function\s+applyBBCalibrateGate\b(.*?)\n\}',
                      panels_js, re.S)
        body = m.group(1)
        assert "classList.contains('bb-reset-mode')" in body, \
            'the Calibrate gate must bail out in RESET mode'

    def test_heartbeat_handler_delegates_to_the_gate(self, panels_js):
        """The BB-IDLE half of the rule must go through the same helper, or the
        next heartbeat re-enables a button the mocap rule just disabled."""
        m = re.search(r'export\s+function\s+updateBBPanel\b(.*?)\n\}\n',
                      panels_js, re.S)
        assert m, 'updateBBPanel() body not found'
        assert 'applyBBCalibrateGate()' in m.group(1), \
            'updateBBPanel() no longer routes the Calibrate enable rule ' \
            'through applyBBCalibrateGate()'
