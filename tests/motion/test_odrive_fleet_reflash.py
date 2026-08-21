"""CI guard for ``tools/odrive_fleet_reflash.py``.

The script's hardware path can only run with seven ODrives attached, but the
logic that decides *what gets written where* is pure and is exactly the part
whose failure is expensive: a wrong node ID flashed to NVM, a hand config
flashed to a leg, or an ambiguous fleet silently resolved by guesswork.

These tests pin that logic, plus the real
``config/ODrive config Files/odrive_pro_leg_config.json`` as a fixture — so a
future edit to that snapshot that breaks the node-ID path or switches the
commutation encoder fails here rather than on the bench.
"""

from __future__ import annotations

import importlib.util
import json
import math
import os

import pytest

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_SCRIPT_PATH = os.path.join(_PROJECT_ROOT, 'tools', 'odrive_fleet_reflash.py')
_LEG_CONFIG = os.path.join(
    _PROJECT_ROOT, 'config', 'ODrive config Files', 'odrive_pro_leg_config.json')
_HAND_CONFIG = os.path.join(
    _PROJECT_ROOT, 'config', 'ODrive config Files', 'odrive_pro_hand_config.json')


def _load_script():
    """Import the tool by path — it is a script, not an installed module."""
    spec = importlib.util.spec_from_file_location('odrive_fleet_reflash', _SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def tool():
    return _load_script()


@pytest.fixture(scope='module')
def leg_config():
    with open(_LEG_CONFIG) as handle:
        return json.load(handle)


# ---------------------------------------------------------------------------
# Importability
# ---------------------------------------------------------------------------

def test_module_imports_without_odrive_package(tool):
    """The odrive import is lazy, so the module loads on a box without hardware."""
    assert hasattr(tool, 'main')
    assert hasattr(tool, 'OdriveApi')


def test_fleet_ids_come_from_generated_config(tool):
    import sys
    sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'config', 'generated'))
    import protocol_config as proto

    assert tool.LEG_NODE_IDS == tuple(proto.NODE_ID_LEGS)
    assert tool.PROTECTED_NODE_IDS == (proto.NODE_ID_JUGGLEBOT_HAND,)
    # The hand must never be in the write set.
    assert proto.NODE_ID_JUGGLEBOT_HAND not in tool.LEG_NODE_IDS


# ---------------------------------------------------------------------------
# flatten_config
# ---------------------------------------------------------------------------

def test_flatten_nested(tool):
    flat = tool.flatten_config({'a': {'b': {'c': 1}}, 'd': 2})
    assert flat == {'a.b.c': 1, 'd': 2}


def test_flatten_is_idempotent_on_flat_input(tool):
    """odrivetool writes flat backups; the repo snapshot is nested. Both work."""
    flat = {'axis0.config.can.node_id': 3, 'can.config.baud_rate': 1000000}
    assert tool.flatten_config(flat) == flat


def test_flatten_real_leg_config(tool, leg_config):
    flat = tool.flatten_config(leg_config)
    assert flat['axis0.config.can.node_id'] == 0  # snapshot came off leg 0
    assert flat['axis0.config.motor.pole_pairs'] == 7
    assert len(flat) > 300


# ---------------------------------------------------------------------------
# patch_node_id — the value that must never be wrong
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('node_id', [0, 1, 2, 3, 4, 5])
def test_patch_node_id_sets_only_the_node_id(tool, leg_config, node_id):
    patched = tool.patch_node_id(leg_config, node_id)
    flat_before = tool.flatten_config(leg_config)
    flat_after = tool.flatten_config(patched)

    assert flat_after['axis0.config.can.node_id'] == node_id
    differing = [
        key for key in flat_before
        if not tool.values_equal(flat_before[key], flat_after[key])
    ]
    assert differing == ([] if node_id == 0 else ['axis0.config.can.node_id'])


def test_patch_node_id_does_not_mutate_the_input(tool, leg_config):
    """Shape-agnostic on purpose: the snapshot's shape depends on which
    odrivetool captured it (this one writes flat, older ones nested), and the
    fixture is the real file."""
    before = json.dumps(leg_config, sort_keys=True, allow_nan=True)
    tool.patch_node_id(leg_config, 5)
    assert json.dumps(leg_config, sort_keys=True, allow_nan=True) == before


def test_patch_node_id_handles_flat_shape(tool):
    patched = tool.patch_node_id({'axis0.config.can.node_id': 0, 'other': 1}, 4)
    assert patched['axis0.config.can.node_id'] == 4


def test_patch_node_id_rejects_a_file_without_the_path(tool):
    with pytest.raises(KeyError):
        tool.patch_node_id({'axis0': {'config': {}}}, 2)


def test_patch_node_id_coerces_to_int(tool, leg_config):
    value = tool.flatten_config(tool.patch_node_id(leg_config, 3.0))[
        'axis0.config.can.node_id']
    assert value == 3
    assert isinstance(value, int)


def test_set_config_value_works_on_both_shapes(tool):
    nested = tool.set_config_value({'a': {'b': 1}}, 'a.b', 7)
    assert nested == {'a': {'b': 7}}
    flat = tool.set_config_value({'a.b': 1}, 'a.b', 7)
    assert flat == {'a.b': 7}


def test_set_config_value_rejects_an_absent_path(tool):
    with pytest.raises(KeyError):
        tool.set_config_value({'a': {'b': 1}}, 'a.c', 7)


# ---------------------------------------------------------------------------
# values_equal / diff_flat — the verify pass
# ---------------------------------------------------------------------------

def test_nan_equals_nan(tool):
    """The snapshot carries NaN (e.g. commutation_encoder_bandwidth); a plain
    == comparison would report every NaN property as a failed write."""
    assert tool.values_equal(float('nan'), float('nan'))
    assert tool.values_equal(float('inf'), float('inf'))
    assert not tool.values_equal(float('nan'), 0.0)


def test_values_equal_is_exact_not_tolerant(tool):
    assert not tool.values_equal(1.0, 1.0000001)


def test_diff_flat_reports_mismatch_and_absence(tool):
    rows = tool.diff_flat(
        actual={'a': 1, 'b': 2},
        expected={'a': 1, 'b': 3, 'c': 4},
    )
    assert [row[0] for row in rows] == ['b', 'c']
    assert rows[0] == ('b', 2, 3)
    assert rows[1][2] == 4


def test_diff_flat_ignores_extra_properties_on_the_drive(tool):
    """A drive exposes config the snapshot does not carry — not a mismatch."""
    assert tool.diff_flat({'a': 1, 'extra': 9}, {'a': 1}) == []


def test_diff_flat_clean_when_drive_matches_real_snapshot(tool, leg_config):
    flat = tool.flatten_config(tool.patch_node_id(leg_config, 4))
    assert tool.diff_flat(dict(flat), flat) == []
    # NaN-carrying properties really are present, so the test above has teeth.
    assert any(isinstance(v, float) and math.isnan(v) for v in flat.values())


# ---------------------------------------------------------------------------
# check_target_is_leg_config — wrong-file guard
# ---------------------------------------------------------------------------

def test_real_leg_config_passes_the_leg_check(tool, leg_config):
    assert tool.check_target_is_leg_config(tool.flatten_config(leg_config)) == []


def test_real_hand_config_is_rejected_as_a_leg_target(tool):
    """Flashing the hand snapshot to a leg would leave it commutating off
    ONBOARD_ENCODER0, which a leg does not have wired."""
    with open(_HAND_CONFIG) as handle:
        hand = json.load(handle)
    problems = tool.check_target_is_leg_config(tool.flatten_config(hand))
    assert problems and 'HAND' in problems[0]


def test_missing_commutation_encoder_is_reported(tool):
    assert tool.check_target_is_leg_config({'axis0.config.can.node_id': 0})


# ---------------------------------------------------------------------------
# validate_fleet — when the serial -> node ID map cannot be trusted
# ---------------------------------------------------------------------------

def _healthy_fleet():
    return {'S0': 0, 'S1': 1, 'S2': 2, 'S3': 3, 'S4': 4, 'S5': 5, 'S6': 6}


def test_healthy_fleet_validates(tool):
    errors, warnings = tool.validate_fleet(_healthy_fleet())
    assert errors == []
    assert warnings == []


def test_duplicate_node_ids_are_an_error(tool):
    """The post-interrupt state: an erased drive reverts to the default node ID
    and collides with leg 0. Guessing which is which is how you flash leg 3's
    node ID onto leg 0."""
    fleet = _healthy_fleet()
    fleet['S3'] = 0
    errors, _ = tool.validate_fleet(fleet)
    assert errors
    assert '--map' in errors[0]


def test_missing_leg_is_an_error(tool):
    fleet = _healthy_fleet()
    del fleet['S4']
    errors, _ = tool.validate_fleet(fleet)
    assert any('[4]' in err for err in errors)


def test_unknown_node_id_is_an_error(tool):
    """A BallButler drive (node 7/8) on the tree must stop the run, not be
    silently reflashed with a Jugglebot leg config."""
    fleet = _healthy_fleet()
    fleet['S7'] = 7
    errors, _ = tool.validate_fleet(fleet)
    assert any('7' in err and 'unrecognised' in err for err in errors)


def test_absent_hand_is_only_a_warning(tool):
    fleet = _healthy_fleet()
    del fleet['S6']
    errors, warnings = tool.validate_fleet(fleet)
    assert errors == []
    assert warnings


# ---------------------------------------------------------------------------
# resolve_targets — what actually gets written
# ---------------------------------------------------------------------------

def test_default_targets_are_the_legs_only(tool):
    targets = tool.resolve_targets(_healthy_fleet(), None)
    assert targets == list(tool.LEG_NODE_IDS)
    assert 6 not in targets


def test_only_selects_a_subset(tool):
    assert tool.resolve_targets(_healthy_fleet(), '3') == [3]
    assert tool.resolve_targets(_healthy_fleet(), '5,0,2') == [0, 2, 5]


def test_only_refuses_the_protected_node(tool):
    with pytest.raises(SystemExit) as excinfo:
        tool.resolve_targets(_healthy_fleet(), '6')
    assert 'protected' in str(excinfo.value)


def test_only_refuses_a_node_that_is_not_attached(tool):
    fleet = _healthy_fleet()
    del fleet['S2']
    with pytest.raises(SystemExit):
        tool.resolve_targets(fleet, '2')


def test_only_rejects_non_integers(tool):
    with pytest.raises(SystemExit):
        tool.resolve_targets(_healthy_fleet(), 'leg3')


def test_only_deduplicates(tool):
    """A drive listed twice would be processed twice, and the second pass would
    run against the handle the first pass's reboot killed."""
    assert tool.resolve_targets(_healthy_fleet(), '2,2,4') == [2, 4]


# ---------------------------------------------------------------------------
# Calibration motion envelope — the guard against commanding more travel
# than the leg physically has
# ---------------------------------------------------------------------------

def test_travel_uses_the_repo_spool_geometry(tool):
    import sys
    sys.path.insert(0, os.path.join(_PROJECT_ROOT, 'config', 'generated'))
    import hardware_config as hw

    assert tool.LEG_STROKE_MM == hw.GEOM_LEG_STROKE_MM
    for node_id, mm_to_rev in enumerate(hw.GEOM_MM_TO_REV):
        assert tool.calibration_travel_mm(1.0, node_id) == pytest.approx(1.0 / mm_to_rev)


def test_snapshot_scan_distance_exceeds_the_stroke(tool, leg_config):
    """The live guard, against the real file: the snapshot's default 8 rev scan
    is about twice the leg stroke, so a default run must NOT be allowed to
    actuate without an explicit override."""
    scan_rev = tool.flatten_config(leg_config)[tool.CALIB_SCAN_DISTANCE_PATH]
    problems = tool.check_calibration_travel(scan_rev, tool.LEG_NODE_IDS)
    assert len(problems) == len(tool.LEG_NODE_IDS)
    assert tool.calibration_travel_mm(scan_rev, 0) > 2 * tool.LEG_STROKE_MM / 1.1


def test_short_scan_passes_the_travel_check(tool):
    assert tool.check_calibration_travel(3.0, tool.LEG_NODE_IDS) == []


def test_travel_check_reports_every_offending_leg(tool):
    problems = tool.check_calibration_travel(8.0, [1, 4])
    assert len(problems) == 2
    assert 'node 1' in problems[0] and 'node 4' in problems[1]


def test_travel_check_is_boundary_exclusive(tool):
    """Exactly-the-stroke is allowed; a hair over is not."""
    stroke_rev = tool.LEG_STROKE_MM / tool.MM_PER_REV[0]
    assert tool.check_calibration_travel(stroke_rev, [0]) == []
    assert tool.check_calibration_travel(stroke_rev * 1.01, [0])


def test_travel_rejects_an_unknown_node(tool):
    with pytest.raises(IndexError):
        tool.calibration_travel_mm(1.0, 99)


def test_describe_calibration_motion_names_the_motion_and_the_numbers(tool):
    lines = tool.describe_calibration_motion(8.0, 2.0, [0, 1])
    blob = '\n'.join(lines)
    assert 'MOVES THE MOTOR' in blob
    assert 'node 0' in blob and 'node 1' in blob
    assert '564' in blob  # 8 rev of leg-0 spool travel, mm


# ---------------------------------------------------------------------------
# Calibration state machine and outcome
# ---------------------------------------------------------------------------

def test_axis_state_constants_match_the_odrive_enums(tool):
    """The state machine drives raw ints so the module imports without the
    odrive package; those ints have to be the real ones."""
    from odrive.enums import AxisState, ProcedureResult

    assert tool.AXIS_STATE_IDLE == int(AxisState.IDLE)
    assert tool.AXIS_STATE_FULL_CALIBRATION_SEQUENCE == int(
        AxisState.FULL_CALIBRATION_SEQUENCE)
    assert tool.PROCEDURE_RESULT_SUCCESS == int(ProcedureResult.SUCCESS)
    assert tool.PROCEDURE_RESULT_BUSY == int(ProcedureResult.BUSY)


def test_calibration_outcome_ok_only_on_clean_success(tool):
    outcome = tool.CalibrationOutcome()
    assert not outcome.ok  # never started

    outcome.started = True
    outcome.result = tool.PROCEDURE_RESULT_SUCCESS
    assert outcome.ok

    outcome.active_errors = 0x20  # DRV_FAULT
    assert not outcome.ok, 'SUCCESS with a latched error is not a pass'

    outcome.active_errors = 0
    outcome.timed_out = True
    assert not outcome.ok

    outcome.timed_out = False
    outcome.result = 15  # NOT_CONVERGING
    assert not outcome.ok


def test_calibration_owned_paths_cover_what_the_procedure_writes(tool):
    """These are excluded from the post-calibration verify, so the set has to
    contain the values calibration is expected to overwrite."""
    for path in (
        'axis0.config.motor.phase_resistance',
        'axis0.config.motor.phase_inductance',
        'axis0.commutation_mapper.config.index_offset',
        'axis0.commutation_mapper.config.index_offset_valid',
    ):
        assert path in tool.CALIBRATION_OWNED_PATHS


def test_drive_result_ok_requires_calibration_to_have_passed(tool):
    res = tool.DriveResult(2, 'SERIAL')
    res.status = 'done'
    res.node_id_after = 2
    assert res.ok, 'no calibration requested -> config-only success'

    outcome = tool.CalibrationOutcome()
    outcome.started = True
    outcome.result = 15  # NOT_CONVERGING
    res.calibration = outcome
    assert not res.ok

    outcome.result = tool.PROCEDURE_RESULT_SUCCESS
    assert res.ok


# ---------------------------------------------------------------------------
# Conflicting-process preflight
# ---------------------------------------------------------------------------

def test_detects_the_ros_stack_and_odrivetool(tool):
    hits = tool.find_conflicting_processes([
        '/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch jugglebot jugglebot_launch.py',
        'python3 /home/jetson/Desktop/Jugglebot/ros_ws/install/jugglebot/lib/jugglebot/teensy_bridge_node',
        'odrivetool',
        '/usr/bin/gnome-terminal',
    ])
    assert sorted(pattern for pattern, _ in hits) == [
        'jugglebot_launch', 'odrivetool', 'teensy_bridge_node']


def test_ignores_its_own_process(tool):
    assert tool.find_conflicting_processes(
        ['python tools/odrive_fleet_reflash.py --dry-run']) == []


def test_clean_box_has_no_conflicts(tool):
    assert tool.find_conflicting_processes(['/sbin/init', 'sshd: jetson@pts/0']) == []


def test_read_proc_cmdlines_returns_something_on_linux(tool):
    cmdlines = tool.read_proc_cmdlines()
    assert isinstance(cmdlines, list)
    if os.path.isdir('/proc/1'):
        assert cmdlines


# ---------------------------------------------------------------------------
# fleet_map round trip
# ---------------------------------------------------------------------------

def test_fleet_map_round_trip(tool, tmp_path):
    path = tmp_path / 'fleet_map.json'
    path.write_text(json.dumps({
        'recorded': '2026-08-12T12:00:00',
        'drives': [
            {'serial': 'AAA', 'node_id': 0, 'role': 'leg'},
            {'serial': 'BBB', 'node_id': 5, 'role': 'leg'},
            {'serial': 'CCC', 'node_id': 6, 'role': 'protected'},
        ],
    }))
    assert tool.load_fleet_map(str(path)) == {'AAA': 0, 'BBB': 5, 'CCC': 6}


# ---------------------------------------------------------------------------
# CLI surface
# ---------------------------------------------------------------------------

def test_defaults_point_at_the_leg_snapshot(tool):
    args = tool.parse_args([])
    assert args.config == tool.DEFAULT_TARGET_CONFIG
    assert os.path.exists(args.config)
    assert args.expect == len(tool.LEG_NODE_IDS) + len(tool.PROTECTED_NODE_IDS) == 7
    assert args.dry_run is False
    assert args.force is False
    assert args.ignore_restore_errors is False


def test_calibration_is_on_by_default_but_gated(tool):
    args = tool.parse_args([])
    assert args.no_calibration is False, 'calibration is part of the requested flow'
    assert args.yes is False, 'and it asks once before it actuates'
    assert args.pause_between_legs is False
    assert args.calib_timeout == tool.CALIB_TIMEOUT_S
    assert args.calib_scan_distance is None


def test_calib_scan_distance_override_is_parsed(tool):
    assert tool.parse_args(['--calib-scan-distance', '2.5']).calib_scan_distance == 2.5


def test_missing_target_config_exits_before_touching_hardware(tool, tmp_path):
    with pytest.raises(SystemExit) as excinfo:
        tool.main(['--config', str(tmp_path / 'nope.json')])
    assert 'not found' in str(excinfo.value)


def test_hand_config_target_exits_before_touching_hardware(tool, tmp_path, capsys):
    """No OdriveApi construction, no discovery — the refusal is a preflight."""
    with pytest.raises(SystemExit) as excinfo:
        tool.main(['--config', _HAND_CONFIG, '--backup-dir', str(tmp_path)])
    assert 'refusing to flash' in str(excinfo.value)
