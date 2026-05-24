"""conftest.py — ROS2 mock injection and fixtures for CAN/node tests.

Injects fake ROS2 modules into sys.modules at import time so that
``import jugglebot.*`` resolves on Windows without a ROS2 installation.
Path setup is handled by the parent tests/conftest.py.
"""

import struct
import sys
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

    def get_fields_and_field_types(self):
        """Mimic ROS2 message introspection for field-by-field copy."""
        import dataclasses as _dc
        return {f.name: str(f.type) for f in _dc.fields(self)}


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
class CatchEventMsg:
    header: object = field(default_factory=lambda: MagicMock())
    catch_time: object = None
    sequence: int = 0
    time_synced: bool = False
    retrigger_suppressed: bool = False


@dataclass
class CatchingConeHeartbeatMsg:
    connected: bool = False
    state: int = 0
    state_data: int = 0
    sync_rms_us: int = 0
    last_catch_seq: int = 0
    ms_since_last_catch: int = 0
    time_synced: bool = False
    have_any_catch: bool = False


@dataclass
class CatchTimingResultMsg:
    header: object = field(default_factory=lambda: MagicMock())
    matched: bool = False
    thrower_name: str = ""
    predicted_landing_time: object = None
    actual_catch_time: object = None
    timing_error_ms: float = 0.0


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
    has_fatal_odrive_error: bool = False
    has_fatal_can_error: bool = False
    has_undervoltage: bool = False
    encoder_search_complete: bool = False
    is_homed: bool = False
    firmware_validated: bool = False
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


@dataclass
class BallButlerCalibrationResult:
    position_mm: object = None
    yaw_offset_rad: float = 0.0
    yaw_offset_std_deg: float = 0.0
    axis_tilt_deg: float = 0.0
    success: bool = False
    message: str = ""


@dataclass
class ThrowAnnouncement:
    header: object = field(default_factory=lambda: MagicMock())
    thrower_name: str = ""
    initial_position: object = None
    initial_velocity: object = None
    target_id: str = ""
    throw_time: object = None
    predicted_tof_sec: float = 0.0
    landing_position: object = None
    landing_velocity: object = None
    landing_time: object = None


@dataclass
class _MockPoseInner:
    """Just enough of geometry_msgs/Pose for the director's body.pose.pose.position access."""
    position: object = None


@dataclass
class _MockPoseStamped:
    pose: object = field(default_factory=_MockPoseInner)


@dataclass
class RigidBodyPose:
    name: str = ""
    pose: object = field(default_factory=_MockPoseStamped)


@dataclass
class RigidBodyPoses:
    header: object = field(default_factory=lambda: MagicMock())
    bodies: list = field(default_factory=list)


# ── geometry_msgs mock ────────────────────────────────────────


@dataclass
class Point:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


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
                'throw_speed': 0.0, 'throw_time': 0.0,
                'suppress_announcement': False},
    resp_fields={'success': False, 'message': ''},
)
ThrowAtTarget = _make_service(
    req_fields={'target_name': '', 'throw_delay_s': 0.0},
    resp_fields={
        'success': False, 'message': '',
        'yaw_rad': 0.0, 'pitch_rad': 0.0, 'throw_speed_mps': 0.0,
        'throw_delay_s': 0.0, 'predicted_tof_s': 0.0,
        'target_position_global_mm': None,
        'target_position_bb_local_mm': None,
    },
)
SetFloat = _make_service(
    req_fields={'data': 0.0},
    resp_fields={'success': False, 'message': ''},
)
SetHandGains = _make_service(
    req_fields={'pos_gain': 35.0, 'vel_gain': 0.007, 'vel_integrator_gain': 0.07},
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


class MockTime:
    """Stand-in for rclpy.time.Time — supports to_msg() and Duration addition."""
    def __init__(self, nanoseconds=0):
        self._ns = nanoseconds
    @property
    def nanoseconds(self):
        return self._ns
    def to_msg(self):
        return self._ns
    def __add__(self, other):
        if isinstance(other, MockDuration):
            return MockTime(self._ns + other._ns)
        return NotImplemented


class MockDuration:
    """Stand-in for rclpy.time.Duration."""
    def __init__(self, seconds=0.0, nanoseconds=0):
        self._ns = int(seconds * 1e9) + nanoseconds


class MockClock:
    def now(self):
        return MockTime()


class MockPublisher:
    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _MockParameter:
    def __init__(self, value):
        self.value = value

    def get_parameter_value(self):
        return self

    @property
    def double_value(self):
        return float(self.value)

    @property
    def string_value(self):
        return str(self.value)

    @property
    def double_array_value(self):
        return list(self.value) if self.value is not None else []


class MockNode:
    """Stand-in for rclpy.node.Node."""
    def __init__(self, name='test_node'):
        self._name = name
        self._logger = MockLogger()
        self._clock = MockClock()
        self._publishers = {}
        self._clients = {}
        self._params = {}

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def declare_parameter(self, name, default_value):
        self._params[name] = default_value
        return _MockParameter(default_value)

    def get_parameter(self, name):
        return _MockParameter(self._params.get(name))

    def create_service(self, *a, **kw):
        return MagicMock()

    def create_client(self, srv_type, name):
        client = MockServiceClient()
        client.srv_name = name
        self._clients[name] = client
        return client

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


class MockActionClient:
    """Stand-in for rclpy.action.ActionClient."""
    def __init__(self, node, action_type, name):
        self._action_type = action_type
        self._name = name
        self._server_ready = True

    def server_is_ready(self):
        return self._server_ready

    def send_goal_async(self, goal):
        future = MockFuture()
        return future


class MockFuture:
    """Stand-in for rclpy Future."""
    def __init__(self):
        self._done = False
        self._result = None
        self._exception = None

    def done(self):
        return self._done

    def result(self):
        if self._exception:
            raise self._exception
        return self._result

    def set_result(self, result):
        self._result = result
        self._done = True

    def set_exception(self, exc):
        self._exception = exc
        self._done = True


class MockServiceClient:
    """Stand-in for rclpy service client."""
    def __init__(self):
        self._ready = True
        self.srv_name = 'mock_service'

    def service_is_ready(self):
        return self._ready

    def call_async(self, request):
        future = MockFuture()
        return future

    def wait_for_service(self, timeout_sec=None):
        return self._ready


# ── Register all mock modules ─────────────────────────────────

_create_mock_module('jugglebot_interfaces')
_create_mock_module('jugglebot_interfaces.msg', {
    'MotorStateSingle': MotorStateSingle,
    'BallButlerCalibrationResult': BallButlerCalibrationResult,
    'BallButlerHeartbeat': BallButlerHeartbeatMsg,
    'CanTrafficReportMessage': CanTrafficReportMessage,
    'CatchEvent': CatchEventMsg,
    'CatchingConeHeartbeat': CatchingConeHeartbeatMsg,
    'CatchTimingResult': CatchTimingResultMsg,
    'HandTelemetryMessage': HandTelemetryMessage,
    'LegsTargetReachedMessage': LegsTargetReachedMessage,
    'RigidBodyPose': RigidBodyPose,
    'RigidBodyPoses': RigidBodyPoses,
    'RobotState': RobotState,
    'SetMotorVelCurrLimitsMessage': SetMotorVelCurrLimitsMessage,
    'SetTrapTrajLimitsMessage': SetTrapTrajLimitsMessage,
    'ThrowAnnouncement': ThrowAnnouncement,
})
_create_mock_module('jugglebot_interfaces.srv', {
    'ActivateOrDeactivate': ActivateOrDeactivate,
    'GetTiltReadingService': GetTiltReadingService,
    'ODriveCommandService': ODriveCommandService,
    'SendBallButlerCommand': SendBallButlerCommand,
    'SetFloat': SetFloat,
    'SetHandGains': SetHandGains,
    'SetHandTrajCmd': SetHandTrajCmd,
    'SetString': SetString,
    'ThrowAtTarget': ThrowAtTarget,
})
_create_mock_module('jugglebot_interfaces.action', {
    'HomeMotors': HomeMotors,
})

_create_mock_module('geometry_msgs')
_create_mock_module('geometry_msgs.msg', {
    'Point': Point,
    'Quaternion': Quaternion,
    'Vector3': Vector3,
})

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
mock_rclpy.time = types.SimpleNamespace(Duration=MockDuration, Time=MockTime)

_create_mock_module('rclpy.node', {'Node': MockNode})
_create_mock_module('rclpy.action', {
    'ActionServer': MockActionServer,
    'ActionClient': MockActionClient,
})
_create_mock_module('rclpy.time', {
    'Time': MockTime,
    'Duration': MockDuration,
})

# rclpy.qos — minimal mock for DurabilityPolicy, ReliabilityPolicy, QoSProfile
class _MockDurabilityPolicy:
    TRANSIENT_LOCAL = 1
    VOLATILE = 2

class _MockReliabilityPolicy:
    RELIABLE = 1
    BEST_EFFORT = 2

class _MockQoSProfile:
    def __init__(self, depth=10, durability=None, reliability=None, **kw):
        self.depth = depth
        self.durability = durability
        self.reliability = reliability

_create_mock_module('rclpy.qos', {
    'QoSProfile': _MockQoSProfile,
    'DurabilityPolicy': _MockDurabilityPolicy,
    'ReliabilityPolicy': _MockReliabilityPolicy,
})


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
