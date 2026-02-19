"""conftest.py — Mock injection and shared fixtures for CAN interface tests.

Injects fake ROS2 modules into sys.modules at import time so that
``import jugglebot.*`` resolves on Windows without a ROS2 installation.
"""

import sys
import struct
import types
from dataclasses import dataclass, field
from unittest.mock import MagicMock

# ════════════════════════════════════════════════════════════════
# Phase 1: Inject mock ROS2 modules BEFORE any jugglebot imports
# ════════════════════════════════════════════════════════════════
# Must happen at module level — Python caches imports, so the mocks
# must be in sys.modules before conftest or test files trigger
# ``from jugglebot_interfaces.msg import MotorStateSingle``.


def _create_mock_module(name, attrs=None):
    """Create a fake module and register it in sys.modules."""
    mod = types.ModuleType(name)
    if attrs:
        for k, v in attrs.items():
            setattr(mod, k, v)
    sys.modules[name] = mod
    return mod


# ── ROS2 message mocks (dataclasses for field-name safety) ────


@dataclass
class MotorStateSingle:
    active_errors: int = 0
    disarm_reason: int = 0
    current_state: int = 0
    procedure_result: int = 0
    trajectory_done: bool = False
    pos_estimate: float = 0.0
    vel_estimate: float = 0.0
    iq_setpoint: float = 0.0
    iq_measured: float = 0.0
    fet_temp: float = 0.0
    motor_temp: float = 0.0
    bus_voltage: float = 0.0
    bus_current: float = 0.0


@dataclass
class BallButlerHeartbeatMsg:
    ball_in_hand: bool = False
    state: int = 0
    state_data: int = 0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    hand_pos_mm: float = 0.0


@dataclass
class CanTrafficReportMessage:
    received_count: int = 0
    report_interval: int = 0


@dataclass
class HandTelemetryMessage:
    timestamp: object = None
    pos_cmd: float = 0.0
    vel_ff_cmd: float = 0.0
    tor_ff_cmd: float = 0.0
    pos_meas: float = 0.0
    vel_meas: float = 0.0
    iq_meas: float = 0.0


@dataclass
class LegsTargetReachedMessage:
    leg0_has_arrived: bool = False
    leg1_has_arrived: bool = False
    leg2_has_arrived: bool = False
    leg3_has_arrived: bool = False
    leg4_has_arrived: bool = False
    leg5_has_arrived: bool = False


@dataclass
class RobotState:
    timestamp: object = None
    motor_states: list = field(default_factory=list)
    error: list = field(default_factory=list)
    encoder_search_complete: bool = False
    is_homed: bool = False
    levelling_complete: bool = False
    pose_offset_rad: list = field(default_factory=list)
    pose_offset_quat: object = None


@dataclass
class SetMotorVelCurrLimitsMessage:
    legs_vel_limit: float = 0.0
    legs_curr_limit: float = 0.0
    hand_vel_limit: float = 0.0
    hand_curr_limit: float = 0.0


@dataclass
class SetTrapTrajLimitsMessage:
    trap_vel_limit: float = 0.0
    trap_acc_limit: float = 0.0
    trap_dec_limit: float = 0.0


# ── geometry_msgs mock ────────────────────────────────────────


@dataclass
class Quaternion:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    w: float = 1.0


# ── std_msgs mocks ────────────────────────────────────────────


@dataclass
class Float64MultiArray:
    data: list = field(default_factory=list)


@dataclass
class MockString:
    data: str = ''


# ── std_srvs mock ─────────────────────────────────────────────


class _TriggerRequest:
    pass


class _TriggerResponse:
    def __init__(self):
        self.success = False
        self.message = ''


class Trigger:
    Request = _TriggerRequest
    Response = _TriggerResponse


# ── Service / action type mocks ───────────────────────────────
# Each service needs a Request and Response class with the right fields.


def _make_service(req_fields=None, resp_fields=None):
    """Build a lightweight service type with Request/Response inner classes."""
    class Request:
        def __init__(self):
            for name, default in (req_fields or {}).items():
                setattr(self, name, default)
    class Response:
        def __init__(self):
            for name, default in (resp_fields or {}).items():
                setattr(self, name, default)
    class Service:
        pass
    Service.Request = Request
    Service.Response = Response
    return Service


ActivateOrDeactivate = _make_service(
    req_fields={'command': ''},
    resp_fields={'success': False, 'message': ''},
)
GetTiltReadingService = _make_service(
    resp_fields={'tilt_xy': [], 'tilt_quat': None},
)
ODriveCommandService = _make_service(
    req_fields={'command': ''},
    resp_fields={'success': False, 'message': ''},
)
SendBallButlerCommand = _make_service(
    req_fields={'yaw_angle_rad': 0.0, 'pitch_angle_rad': 0.0,
                'throw_speed': 0.0, 'throw_time': 0.0},
    resp_fields={'success': False, 'message': ''},
)
SetFloat = _make_service(
    req_fields={'data': 0.0},
    resp_fields={'success': False, 'message': ''},
)
SetHandTrajCmd = _make_service(
    req_fields={'event_delay': 0.0, 'event_vel': 0.0, 'traj_type': 0},
    resp_fields={'success': False, 'message': ''},
)
SetString = _make_service(
    req_fields={'data': ''},
    resp_fields={'success': False, 'message': ''},
)


# ── HomeMotors action mock ────────────────────────────────────


class _HomeMotorsResult:
    def __init__(self):
        self.success = False


class _HomeMotorsFeedback:
    pass


class HomeMotors:
    class Goal:
        pass
    Result = _HomeMotorsResult
    Feedback = _HomeMotorsFeedback


# ── rclpy mock ────────────────────────────────────────────────


class MockLogger:
    """Logger that accepts throttle_duration_sec kwarg like rclpy loggers."""
    def info(self, msg, **kw): pass
    def warning(self, msg, **kw): pass
    def error(self, msg, **kw): pass
    def fatal(self, msg, **kw): pass
    def debug(self, msg, **kw): pass


class MockClock:
    class _Now:
        def to_msg(self):
            return None
    def now(self):
        return self._Now()


class MockPublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class MockNode:
    """Stand-in for rclpy.node.Node."""
    def __init__(self, name='test_node'):
        self._name = name
        self._logger = MockLogger()
        self._clock = MockClock()
        self._publishers = {}

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def create_service(self, *a, **kw):
        return MagicMock()

    def create_subscription(self, *a, **kw):
        return MagicMock()

    def create_publisher(self, msg_type, topic, qos):
        pub = MockPublisher()
        self._publishers[topic] = pub
        return pub

    def create_timer(self, *a, **kw):
        return MagicMock()

    def destroy_node(self):
        pass


class MockActionServer:
    """Stand-in for rclpy.action.ActionServer."""
    def __init__(self, node, action_type, name, callback):
        self._callback = callback


# ── Register all mock modules ─────────────────────────────────

_create_mock_module('jugglebot_interfaces')
_create_mock_module('jugglebot_interfaces.msg', {
    'MotorStateSingle': MotorStateSingle,
    'BallButlerHeartbeat': BallButlerHeartbeatMsg,
    'CanTrafficReportMessage': CanTrafficReportMessage,
    'HandTelemetryMessage': HandTelemetryMessage,
    'LegsTargetReachedMessage': LegsTargetReachedMessage,
    'RobotState': RobotState,
    'SetMotorVelCurrLimitsMessage': SetMotorVelCurrLimitsMessage,
    'SetTrapTrajLimitsMessage': SetTrapTrajLimitsMessage,
})
_create_mock_module('jugglebot_interfaces.srv', {
    'ActivateOrDeactivate': ActivateOrDeactivate,
    'GetTiltReadingService': GetTiltReadingService,
    'ODriveCommandService': ODriveCommandService,
    'SendBallButlerCommand': SendBallButlerCommand,
    'SetFloat': SetFloat,
    'SetHandTrajCmd': SetHandTrajCmd,
    'SetString': SetString,
})
_create_mock_module('jugglebot_interfaces.action', {
    'HomeMotors': HomeMotors,
})

_create_mock_module('geometry_msgs')
_create_mock_module('geometry_msgs.msg', {'Quaternion': Quaternion})

_create_mock_module('std_msgs')
_create_mock_module('std_msgs.msg', {
    'Float64MultiArray': Float64MultiArray,
    'String': MockString,
})

_create_mock_module('std_srvs')
_create_mock_module('std_srvs.srv', {'Trigger': Trigger})

# rclpy
mock_rclpy = _create_mock_module('rclpy')
mock_rclpy.ok = MagicMock(return_value=True)
mock_rclpy.init = MagicMock()
mock_rclpy.shutdown = MagicMock()
mock_rclpy.spin_once = MagicMock()

_create_mock_module('rclpy.node', {'Node': MockNode})
_create_mock_module('rclpy.action', {'ActionServer': MockActionServer})


# ════════════════════════════════════════════════════════════════
# Phase 2: Pytest fixtures (available after mock injection)
# ════════════════════════════════════════════════════════════════

import pytest  # noqa: E402 — must come after mock injection


@pytest.fixture
def motor_state_single():
    """Fresh MotorStateSingle mock instance."""
    return MotorStateSingle()


@pytest.fixture
def sample_heartbeat_data():
    """Valid 7+ byte ODrive heartbeat CAN frame.

    Layout: 4 bytes axis_error (uint32=0), byte4=current_state (8=CLOSED_LOOP),
    byte5=procedure_result (0=SUCCESS), byte6=traj_done flags (0x01=done),
    byte7=padding.
    """
    return struct.pack('<IBBBB', 0, 8, 0, 0x01, 0)


@pytest.fixture
def sample_error_data():
    """Valid 8-byte ODrive error frame: active_errors=512 (undervoltage), disarm=0."""
    return struct.pack('<II', 512, 0)


@pytest.fixture
def sample_encoder_data():
    """Valid 8-byte encoder estimate: pos=1.5 rev, vel=0.25 rev/s."""
    return struct.pack('<ff', 1.5, 0.25)


@pytest.fixture
def sample_bb_heartbeat_data():
    """Valid 8-byte Ball Butler heartbeat frame.

    ball_in_hand=True, state=IDLE(1), state_data=0,
    yaw_enc=1000, pitch_enc=500, hand_enc=200.
    """
    state_byte = (1 << 1) | 0x01  # state=1 (IDLE), ball_in_hand=True
    return struct.pack('<BBHHH', state_byte, 0, 1000, 500, 200)
