"""conftest.py — ROS2 mock injection and fixtures for CAN/node tests.

Injects fake ROS2 modules into sys.modules at import time so that
``import jugglebot.*`` resolves on Windows without a ROS2 installation.
Path setup is handled by the parent tests/conftest.py.

Injection is two-tier:

* **Unconditional** — ``rclpy``, ``jugglebot_interfaces``, ``geometry_msgs``,
  ``std_msgs``, ``std_srvs``. These have no non-ROS2 implementation, so a mock
  is the only option and the mock is always used, Jetson included.
  (``sensor_msgs`` was listed here until 2026-08-11 but has never actually been
  injected: ``teensy_bridge_node``'s two ``JointState`` publishers bind the REAL
  Foxy package. That is fine on the Jetson and load-bearing for the
  ``MockHeader`` stub below — the real ``JointState`` constructs a
  ``std_msgs.msg.Header``, which DOES resolve to the mock — but it does mean the
  bridge test family cannot import on a box with no ROS 2 install.)
* **Fallback only** — ``diagnostic_msgs`` and ``ament_index_python``
  (registered at the bottom of this file, guarded by ``try: import``). Both
  exist for real on the Jetson and are used for real there; the stubs exist
  so this module's opening claim holds on a box without ROS2. Until
  2026-08-02 that claim was false: neither was mocked, and five modules
  import them.
"""

import os
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
class MsgTime:
    """builtin_interfaces/Time stand-in for message FIELDS (not rclpy.time.Time).

    Default-constructed to zero exactly like the real message, so a producer
    that leaves a stamp untouched is distinguishable from one that wrote a
    value — the "never seen ⇒ zeroed stamp" contract on HandTelemetryMessage
    is asserted against these zeros.
    """
    sec: int = 0
    nanosec: int = 0


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
    connected: bool = False
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
    # Hand ball-present sensor (tri-state: ball_held is meaningless unless
    # ball_held_valid). Mirrors msg/HandTelemetryMessage.msg defaults.
    ball_held: bool = False
    ball_held_raw: bool = False
    ball_held_valid: bool = False
    ball_held_stamp: object = field(default_factory=MsgTime)


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
    # Mirrors msg/RobotState.msg. The defaults are the real message's defaults
    # AND the honest boot state: nothing read yet, so nothing known.
    platform_fw_version: int = 0
    platform_fw_version_read: bool = False


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
    # ``Point`` is defined further down this file; the lambda defers the lookup
    # to construction time. A real message default-constructs its Point, and
    # mocap_node._publish_calibration_result writes position_mm.x/.y/.z — a
    # bare ``None`` default made the success path unconstructable under the mock.
    position_mm: object = field(default_factory=lambda: Point())
    yaw_offset_rad: float = 0.0
    yaw_offset_std_deg: float = 0.0
    axis_tilt_deg: float = 0.0
    success: bool = False
    message: str = ""


@dataclass
class MocapDataSingle:
    position: object = field(default_factory=lambda: Point())
    residual: float = 0.0
    label: str = ""


@dataclass
class MocapDataMulti:
    markers: list = field(default_factory=list)
    aligned: bool = False


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
class TrajectoryStatus:
    streaming: bool = False
    mode: str = ''
    plan_kind: str = ''
    plan_time_remaining_s: float = 0.0
    seq: int = 0
    max_emit_gap_ms: float = 0.0
    last_rejection: str = ''
    # Mirrors msg/TrajectoryStatus.msg. Default False = the real message's
    # default too, and the fail-closed reading for the toss's
    # REJECTED_NOT_LEVELLED gate.
    gravity_correction_loaded: bool = False
    # C-LEVEL-2 observability (ros_ws/docs/levelling_frame.md). Defaults mirror
    # the real message again: no map loaded, empty version. The pair reports
    # LOADED, not applied — a loaded map is dormant until
    # gravity_correction_loaded is also true. Nothing gates on either — the
    # tilt map is a refinement, so a consumer that refuses to act without one
    # is a bug.
    tilt_map_loaded: bool = False
    tilt_map_version: str = ''
    # LIVE session leg limits (2026-08-14): the values the feasibility gate is
    # enforcing right now (YAML working point as ramped by set_limits). 0.0 =
    # absent, mirroring the real message — consumers fall back to the
    # YAML-default module copies.
    leg_vel_limit_mmps: float = 0.0
    leg_acc_limit_mmps2: float = 0.0
    leg_jerk_limit_mmps3: float = 0.0


# ── jugglebot_interfaces DynamicTargetCommand / TargetFeedback (Phase 5) ──


@dataclass
class DynamicTargetCommand:
    target_pos: 'Point' = field(default_factory=lambda: Point())
    target_quat: 'Quaternion' = field(default_factory=lambda: Quaternion())
    target_vel: 'Vector3' = field(default_factory=lambda: Vector3())
    arrival_time: float = 0.0


@dataclass
class TargetFeedback:
    accepted: bool = False
    code: str = ''
    reason: str = ''
    arrival_time: float = 0.0
    source: str = ''


@dataclass
class BallStateArray:
    balls: list = field(default_factory=list)


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


@dataclass
class Pose:
    position: Point = field(default_factory=Point)
    orientation: Quaternion = field(default_factory=Quaternion)


@dataclass
class PoseStamped:
    header: object = field(default_factory=lambda: MagicMock())
    pose: Pose = field(default_factory=Pose)


@dataclass
class Transform:
    translation: Vector3 = field(default_factory=Vector3)
    rotation: Quaternion = field(default_factory=Quaternion)


@dataclass
class TransformStamped:
    """geometry_msgs/TransformStamped stand-in — mocap_node's static world →
    platform_start TF. Added 2026-08-21 with tests/ros/test_mocap_node.py: the
    module-level ``from geometry_msgs.msg import TransformStamped`` in
    mocap_node.py made that node unimportable under the mock until now, which
    is why the node had no direct test coverage at all.
    """
    header: object = field(default_factory=lambda: MagicMock())
    child_frame_id: str = ""
    transform: Transform = field(default_factory=Transform)


# ── jugglebot_interfaces PlatformPoseCommand (SpaceMouse/GUI target) ──


@dataclass
class PlatformPoseCommand:
    pose_stamped: PoseStamped = field(default_factory=PoseStamped)
    publisher: str = ''


# ── std_msgs mocks ────────────────────────────────────────────


@dataclass
class Float64MultiArray:
    data: list = field(default_factory=list)


@dataclass
class MockFloat64:
    data: float = 0.0


@dataclass
class Float32MultiArray:
    data: list = field(default_factory=list)


@dataclass
class MockString:
    data: str = ''


@dataclass
class MockBool:
    data: bool = False


@dataclass
class MockHeader:
    """std_msgs/Header stand-in (fields: stamp, frame_id).

    Load-bearing for the REAL ``sensor_msgs`` messages. ``sensor_msgs`` is NOT
    mocked here (it is the real Foxy package on the Jetson), and the real
    ``JointState.__init__`` does ``from std_msgs.msg import Header`` — which
    resolves to THIS module, because std_msgs IS mocked unconditionally. Without
    a Header stub, ``JointState()`` raises ImportError, so no test could
    construct one and the node's two JointState publishers
    (``bb/axis_estimates`` since 2026-07, ``leg_cmd_executed`` from 2026-08-11)
    were unreachable from the suite. The real ``header`` setter asserts
    ``isinstance(value, Header)`` against this same stub, so the pairing is
    consistent.
    """
    stamp: MsgTime = field(default_factory=MsgTime)
    frame_id: str = ''


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


class _SetBoolRequest:
    def __init__(self):
        self.data = False


class _SetBoolResponse:
    def __init__(self):
        self.success = False
        self.message = ''


class SetBool:
    Request = _SetBoolRequest
    Response = _SetBoolResponse


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
BallButlerThrow = _make_service(
    req_fields={'target_name': '', 'throw_delay_s': 0.0,
                # Phase-7 point-target extension (default-zero preserves name callers).
                'target_point_global_mm': None, 'use_target_point': False,
                'aim_only': False},
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
GoToPose = _make_service(
    # lean_gain default -1.0 mirrors the srv FIELD DEFAULT (since 2026-07-17):
    # a default-constructed request DEFERS to the config gain (JB_TRAJ_LEAN_GAIN,
    # ships 0.6 = lean ON); an explicit 0.0 forces lean OFF (see GoToPose.srv).
    req_fields={'pose': None, 'duration_s': 0.0, 'lean_gain': -1.0},
    resp_fields={'accepted': False, 'code': '', 'message': '',
                 'planned_duration_s': 0.0, 'min_duration_s': 0.0},
)
SetTrajectoryLimits = _make_service(
    req_fields={'leg_vel_limit_mmps': 0.0, 'leg_acc_limit_mmps2': 0.0,
                'leg_jerk_limit_mmps3': 0.0},
    resp_fields={'success': False, 'message': '',
                 'applied_vel_limit_mmps': 0.0, 'applied_acc_limit_mmps2': 0.0,
                 'applied_jerk_limit_mmps3': 0.0},
)


class TimedTarget:
    """Timed target service mock (Phase 5): relative lead_time_s per Request
    (seconds from service receipt — the node anchors the absolute arrival at
    handler entry on perf_counter)."""
    class Request:
        def __init__(self):
            self.pose = None
            self.velocity_mm_s = Vector3()
            self.lead_time_s = 0.0
            self.hold_after = False
    Response = _make_service(
        resp_fields={'accepted': False, 'code': '', 'message': '',
                     'planned_duration_s': 0.0, 'min_duration_s': 0.0}).Response


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


# ── BallButlerThrowCmd action mock ──────────────────


class _BallButlerThrowCmdGoal:
    def __init__(self):
        self.yaw_angle_rad = 0.0
        self.pitch_angle_rad = 0.0
        self.throw_speed = 0.0
        self.throw_time = 0.0
        self.suppress_announcement = False


class _BallButlerThrowCmdResult:
    def __init__(self):
        self.success = False
        self.outcome = 0
        self.detail0 = 0
        self.detail1 = 0
        self.message = ''


class _BallButlerThrowCmdFeedback:
    pass


class BallButlerThrowCmd:
    Goal = _BallButlerThrowCmdGoal
    Result = _BallButlerThrowCmdResult
    Feedback = _BallButlerThrowCmdFeedback


# ── Reload action mock (Phase 7) ──────────────────


class _ReloadGoal:
    def __init__(self):
        self.throw_delay_s = 0.0
        self.catch_vel_scale = 0.0


class _ReloadResult:
    def __init__(self):
        self.success = False
        self.outcome = ''
        self.catch_error_mm = float('nan')


class _ReloadFeedback:
    def __init__(self):
        self.phase = ''


class Reload:
    Goal = _ReloadGoal
    Result = _ReloadResult
    Feedback = _ReloadFeedback


# ── Toss action mock (Phase 8) ──────────────────


class _TossGoal:
    def __init__(self):
        self.catch_position = Point()
        self.flight_time_s = 0.0
        self.throw_delay_s = 0.0
        self.catch_vel_scale = 0.0


class _TossResult:
    def __init__(self):
        self.success = False
        self.outcome = ''
        self.catch_error_mm = float('nan')
        self.achieved_flight_s = float('nan')


class _TossFeedback:
    def __init__(self):
        self.phase = ''


class Toss:
    Goal = _TossGoal
    Result = _TossResult
    Feedback = _TossFeedback


# ── TossContinuous action mock (Phase F) ──────────────────


class _TossContinuousGoal:
    def __init__(self):
        self.catch_position = Point()
        self.throw_height_m = 0.0
        self.num_throws = 0
        self.dwell_time_s = 0.0
        self.throw_delay_s = 0.0
        self.catch_vel_scale = 0.0
        # MIRRORS the .action IDL default `bool stop_on_miss true`, and the
        # default is LOAD-BEARING (operator decision (c), 2026-07-28): an
        # omitted field must mean STOP, never CONTINUE. Pinned equal to the
        # .action file by
        # tests/ros/test_toss_session.py::test_stop_on_miss_wire_default_is_true.
        self.stop_on_miss = True
        # MIRRORS `string on_empty_cup "STOP"` — the same doctrine one level on:
        # an omitted or unreadable field must never start an autonomous BB
        # reload. Pinned equal to the .action file by
        # tests/ros/test_toss_session.py::test_on_empty_cup_wire_default_is_stop.
        self.on_empty_cup = 'STOP'
        # 0 => the config default (JB_OP_TOSS_SESSION_MAX_RELOADS, 3).
        self.max_reloads = 0


class _TossContinuousResult:
    def __init__(self):
        self.success = False
        self.outcome = ''
        self.throws_completed = 0
        self.catches_confirmed = 0
        self.per_cycle_outcomes = []
        self.per_cycle_catch_error_mm = []
        self.per_cycle_flight_s = []
        self.per_cycle_dwell_s = []
        self.reloads_used = 0


class _TossContinuousFeedback:
    def __init__(self):
        self.cycle_index = 0
        self.phase = ''
        self.catches_confirmed = 0


class TossContinuous:
    Goal = _TossContinuousGoal
    Result = _TossContinuousResult
    Feedback = _TossContinuousFeedback


# ── rclpy mock ────────────────────────────────────────────────


class MockLogger:
    """Logger that accepts throttle_duration_sec kwarg like rclpy loggers."""
    def info(self, msg, **kw): pass
    def warning(self, msg, **kw): pass
    def warn(self, msg, **kw): pass   # rclpy loggers expose both warn() and warning()
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
        self._services = {}
        self._subscriptions = {}
        self._clients = {}
        # Action server/client registries (for the orchestrator-conduit drift-guard): real rclpy hides an
        # action's sub-services from node.services/node.clients (they live in a
        # Waitable), and the mock ActionServer/ActionClient register nothing on the
        # node. Recording (name, type) here lets a contract test introspect the action
        # surface deterministically, offline, under the mock.
        self._action_servers = {}
        self._action_clients = {}
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
        # Record the service (name, type) so tests can assert the production service
        # surface AND its type — e.g. the leg/hand /teensy/* rename + the orchestrator conduit
        # (name, type) contract the orchestrator's clients depend on. srv_type is the
        # 1st positional (or 'srv_type' kwarg); srv_name the 2nd (or 'srv_name').
        srv_type = a[0] if len(a) >= 1 else kw.get('srv_type')
        name = a[1] if len(a) >= 2 else kw.get('srv_name')
        svc = MagicMock()
        svc.srv_name = name
        svc.srv_type = srv_type
        # Record the callback_group (default None) so a contract test can pin the
        # group a service was registered in — e.g. that the armed /clear_errors reroute
        # shares the /recover ReentrantCallbackGroup, not the node default (F1).
        svc.callback_group = kw.get('callback_group')
        if name is not None:
            self._services[name] = svc
        return svc

    def create_client(self, srv_type, name, **kw):
        client = MockServiceClient()
        client.srv_name = name
        client.srv_type = srv_type
        self._clients[name] = client
        return client

    def create_subscription(self, *a, **kw):
        # Record the subscription (topic, msg_type) for the same reason as
        # create_service. msg_type is the 1st positional (or 'msg_type'); topic the
        # 2nd (or 'topic').
        msg_type = a[0] if len(a) >= 1 else kw.get('msg_type')
        name = a[1] if len(a) >= 2 else kw.get('topic')
        sub = MagicMock()
        sub.topic_name = name
        sub.msg_type = msg_type
        if name is not None:
            self._subscriptions[name] = sub
        return sub

    def create_publisher(self, msg_type, topic, qos, **kw):
        pub = MockPublisher()
        pub.topic_name = topic
        pub.msg_type = msg_type
        self._publishers[topic] = pub
        return pub

    def create_timer(self, *a, **kw):
        return MagicMock()

    def destroy_node(self):
        pass


class GoalResponse:
    """Stand-in for rclpy.action.GoalResponse."""
    REJECT = 1
    ACCEPT = 2


class CancelResponse:
    """Stand-in for rclpy.action.CancelResponse."""
    REJECT = 1
    ACCEPT = 2


class MockActionServer:
    """Stand-in for rclpy.action.ActionServer.

    Accepts both the simple positional-callback form (HomeMotors:
    ``ActionServer(node, type, name, execute_cb)``) and the keyword form the
    bb/throw action uses (execute/goal/cancel callbacks + callback_group).
    """
    def __init__(self, node, action_type, name, execute_callback=None,
                 goal_callback=None, cancel_callback=None, callback_group=None):
        self._action_type = action_type
        self._name = name
        self._execute_callback = execute_callback
        self._goal_callback = goal_callback
        self._cancel_callback = cancel_callback
        self._callback_group = callback_group
        # Conduit drift-guard: expose the action (name, type) and register on the node
        # so a contract test can introspect it (real rclpy hides these in a Waitable).
        self.action_name = name
        self.action_type = action_type
        if node is not None and hasattr(node, '_action_servers'):
            node._action_servers[name] = self


class MockActionClient:
    """Stand-in for rclpy.action.ActionClient."""
    def __init__(self, node, action_type, name):
        self._action_type = action_type
        self._name = name
        self._server_ready = True
        # Conduit drift-guard: expose (name, type) + register on the node.
        self.action_name = name
        self.action_type = action_type
        if node is not None and hasattr(node, '_action_clients'):
            node._action_clients[name] = self

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

    def add_done_callback(self, cb):
        # Mock futures never resolve on their own; if already done, fire now.
        if self._done:
            cb(self)


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
    'MocapDataMulti': MocapDataMulti,
    'MocapDataSingle': MocapDataSingle,
    'RigidBodyPose': RigidBodyPose,
    'RigidBodyPoses': RigidBodyPoses,
    'RobotState': RobotState,
    'SetMotorVelCurrLimitsMessage': SetMotorVelCurrLimitsMessage,
    'SetTrapTrajLimitsMessage': SetTrapTrajLimitsMessage,
    'ThrowAnnouncement': ThrowAnnouncement,
    'TrajectoryStatus': TrajectoryStatus,
    'PlatformPoseCommand': PlatformPoseCommand,
    'DynamicTargetCommand': DynamicTargetCommand,
    'TargetFeedback': TargetFeedback,
    'BallStateArray': BallStateArray,
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
    'BallButlerThrow': BallButlerThrow,
    'GoToPose': GoToPose,
    'SetTrajectoryLimits': SetTrajectoryLimits,
    'TimedTarget': TimedTarget,
})
_create_mock_module('jugglebot_interfaces.action', {
    'HomeMotors': HomeMotors,
    'BallButlerThrowCmd': BallButlerThrowCmd,
    'Reload': Reload,
    'Toss': Toss,
    'TossContinuous': TossContinuous,
})

_create_mock_module('geometry_msgs')
_create_mock_module('geometry_msgs.msg', {
    'Point': Point,
    'Quaternion': Quaternion,
    'Vector3': Vector3,
    'Pose': Pose,
    'PoseStamped': PoseStamped,
    'Transform': Transform,
    'TransformStamped': TransformStamped,
})

_create_mock_module('std_msgs')
_create_mock_module('std_msgs.msg', {
    'Float64MultiArray': Float64MultiArray,
    'Float32MultiArray': Float32MultiArray,
    'Float64': MockFloat64,
    'String': MockString,
    'Bool': MockBool,
    'Header': MockHeader,
})

_create_mock_module('std_srvs')
_create_mock_module('std_srvs.srv', {'Trigger': Trigger, 'SetBool': SetBool})


# ── tf2_ros mock (UNCONDITIONAL, like rclpy) ──────────────────────────
# The REAL Foxy tf2_ros cannot be used here no matter what the box has
# installed: its first import line is `from rclpy.duration import Duration`,
# and this file replaces `rclpy` with a plain module (not a package), so the
# submodule import raises ModuleNotFoundError before tf2_ros finishes loading.
# mocap_node.py imports tf2_ros at module level, which is why that node was
# unimportable — and therefore untested — until 2026-08-21. Only
# StaticTransformBroadcaster is used in this tree.
class _MockStaticTransformBroadcaster:
    """Stand-in for tf2_ros.StaticTransformBroadcaster.

    Mirrors the real constructor's one observable effect — it creates a
    /tf_static publisher on the node — so a test can assert the static TF was
    broadcast without a running ROS graph.
    """

    def __init__(self, node, qos=None):
        self._node = node
        self.sent = []
        self.pub_tf = node.create_publisher(object, '/tf_static', 1)

    def sendTransform(self, transform):
        self.sent.append(transform)


_create_mock_module('tf2_ros', {
    'StaticTransformBroadcaster': _MockStaticTransformBroadcaster,
    'TransformBroadcaster': _MockStaticTransformBroadcaster,
})

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
    'GoalResponse': GoalResponse,
    'CancelResponse': CancelResponse,
})


# ── FALLBACK stubs: diagnostic_msgs / ament_index_python ──────────────
# Everything above is injected UNCONDITIONALLY — those packages have no
# non-ROS2 implementation, so a mock is the only option. These two are
# different, and are registered only when the real package is absent:
#
#   * ``diagnostic_msgs`` is DELIBERATELY the real package on the Jetson.
#     tests/ros/test_teensy_bridge_node_read.py's docstring says so
#     explicitly, and the real message classes are stricter than any stub
#     (see the ``level`` note below). Shadowing them unconditionally would
#     silently lower fidelity on the box that is authoritative for ROS
#     behaviour, to buy portability nothing on that box needs.
#   * ``ament_index_python`` is already handled where it matters:
#     ``motion/friction_ff_params.py`` wraps its import in try/except and
#     falls back to the source tree, and tests/motion/test_friction_ff_params.py
#     pins that fallback by simulating the ImportError. Only
#     ``ball_butler_node.py``'s TOP-LEVEL import is unguarded — its call
#     site (``:317``) is already inside try/except — so the stub exists
#     purely so that module can be imported.
#
# Together these make tests/ros/ importable without a ROS2 installation
# (this file's opening docstring claims exactly that, and until 2026-08-02
# the claim was false for the four modules importing these two packages:
# teensy_bridge_node, trajectory_node, orchestrator_node, motion_bridge_node,
# plus ball_butler_node).

try:  # pragma: no cover - exercised on the Jetson, where the real package wins
    import diagnostic_msgs.msg  # noqa: F401
except ImportError:
    class DiagnosticStatus:
        """Stand-in for diagnostic_msgs/DiagnosticStatus.

        The severity constants are ``bytes``, NOT ints — Foxy declares
        ``level`` as ``byte``, so the real class asserts
        ``bytes`` of length 1 and ``DiagnosticStatus(level=1)`` raises.
        Production assigns ``status.level = DiagnosticStatus.OK`` and tests
        compare against the same constants, so int constants here would
        compare equal to each other but never to a real-package value —
        divergence that only shows up when someone moves a test between
        platforms. Values verified against the installed Foxy package.
        """
        OK = b'\x00'
        WARN = b'\x01'
        ERROR = b'\x02'
        STALE = b'\x03'

        def __init__(self, level=b'\x00', name='', message='',
                     hardware_id='', values=None):
            self.level = level
            self.name = name
            self.message = message
            self.hardware_id = hardware_id
            self.values = values if values is not None else []

    class KeyValue:
        """Stand-in for diagnostic_msgs/KeyValue (fields: key, value)."""

        def __init__(self, key='', value=''):
            self.key = key
            self.value = value

    _create_mock_module('diagnostic_msgs')
    _create_mock_module('diagnostic_msgs.msg', {
        'DiagnosticStatus': DiagnosticStatus,
        'KeyValue': KeyValue,
    })

try:  # pragma: no cover - exercised on the Jetson, where the real package wins
    import ament_index_python.packages  # noqa: F401
except ImportError:
    class PackageNotFoundError(KeyError):
        """Real one subclasses KeyError -> LookupError; callers catch broadly."""

    def _get_package_share_directory(package_name, *_args, **_kwargs):
        # Raise rather than invent a path: without an ament index there IS no
        # share directory, and every call site in this repo is already inside
        # try/except and falls back to the source tree. Returning a plausible
        # but wrong path would turn a handled absence into a silent mislookup.
        raise PackageNotFoundError(package_name)

    _create_mock_module('ament_index_python')
    _create_mock_module('ament_index_python.packages', {
        'get_package_share_directory': _get_package_share_directory,
        'get_package_share_path': _get_package_share_directory,
        'PackageNotFoundError': PackageNotFoundError,
    })


class _MockReentrantCallbackGroup:
    pass


class _MockMutuallyExclusiveCallbackGroup:
    pass


_create_mock_module('rclpy.callback_groups', {
    'ReentrantCallbackGroup': _MockReentrantCallbackGroup,
    'MutuallyExclusiveCallbackGroup': _MockMutuallyExclusiveCallbackGroup,
})


class _MockMultiThreadedExecutor:
    def __init__(self, *a, **kw):
        self._nodes = []
    def add_node(self, node):
        self._nodes.append(node)
    def spin(self):
        pass
    def shutdown(self):
        pass


_create_mock_module('rclpy.executors', {
    'MultiThreadedExecutor': _MockMultiThreadedExecutor,
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

# A path that cannot exist, used to pin the C-LEVEL-2 tilt-map loader OFF.
_NO_TILT_CAL = os.path.join(
    os.sep, 'nonexistent', 'jugglebot-tests', 'tilt_calibration.yaml')


@pytest.fixture(autouse=True)
def no_tilt_calibration(monkeypatch):
    """Every `tests/ros` node starts with NO tilt calibration map loaded.

    `trajectory_node` loads `config/tilt_calibration.yaml` in `__init__`
    (contract C-LEVEL-2, `ros_ws/docs/levelling_frame.md`), resolving the repo
    **source tree** before the ament share dir. That file is now REAL: the first
    machine-written capture was committed on 2026-08-10 (3df256b), which is the
    day this fixture stopped being anticipatory and started being the only thing
    holding the line. Without it, every TrajectoryNode built in this directory
    would silently apply a hardware calibration, and every test that pins an
    exact corrected rotvec (`test_levelling_frame.py::_expected_rotvec` and its
    whole E-row family) would fail with a diff of a few thousandths of a radian
    and no obvious cause.

    Both non-override candidates now resolve, so a test cannot manufacture
    absence by blanking one of them: `setup.py` installs the YAML whenever it
    exists, so `colcon build` populates `share/jugglebot/config/` too. Absence
    has to be pinned at `tilt_map_candidates` itself — see
    `test_trajectory_tilt_map.py::test_loader_absent_file_logs_info_not_error`,
    which learned that the expensive way.

    Worse than the noise is what it would mean: the suite's answers would depend
    on whether this particular machine had been calibrated, so the same commit
    would pass on one checkout and fail on another. A unit test must not read the
    robot's measurements. Tests that WANT a map set `$JUGGLEBOT_TILT_CAL`
    themselves (the env override is authoritative, so pointing it at a real file
    wins) — see `test_trajectory_tilt_map.py`.
    """
    # Read the env-var NAME from the module under test, never a literal: a rename
    # would otherwise leave this fixture green while it pins nothing at all, and
    # the failure it exists to prevent returns with no test naming it.
    from jugglebot.motion.tilt_map import TILT_MAP_ENV
    monkeypatch.setenv(TILT_MAP_ENV, _NO_TILT_CAL)


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


# ── Toss-record sink isolation (parallel safety + operator artefact hygiene) ──
# ``reload_coordinator_node`` resolves BOTH of its file sinks — the JSONL record
# belt and the end-of-goal trim proposal — to ``<repo>/temp/logs`` at import.
# That is right in production and wrong under pytest, for two separate reasons:
#
#  1. it is a FIXED SHARED PATH, and the gate runs four xdist workers over one
#     working tree (CLAUDE.md "New tests are parallel by default");
#  2. worse, a test-written ``toss_records_*.jsonl`` is byte-indistinguishable
#     from a real session's and lands in exactly the directory the toss-cal
#     capture workflow mines. Measured 2026-08-11 before this fixture: 468
#     synthetic ``toss_records_*.jsonl`` + 175 ``*_trim_proposal.yaml`` had
#     accumulated in the operator's ``temp/logs`` from ordinary suite runs.
#
# Patch-if-already-imported: the module is only in ``sys.modules`` for the test
# files that actually exercise it, so no test pays an import it did not ask for
# and a mock-configuration change cannot turn this into a collection error.
# A test that patches ``_RECORD_BELT_DIR`` itself still wins (its monkeypatch is
# applied later); a test that needs the PRODUCTION-resolved value asks for the
# ``real_record_belt_dir`` fixture below.

_RCN_MODULE = 'jugglebot.reload_coordinator_node'


@pytest.fixture
def real_record_belt_dir():
    """The production-resolved ``_RECORD_BELT_DIR``, before isolation redirects it.

    ``_isolate_toss_record_sinks`` DEPENDS on this fixture, which is what forces
    pytest to instantiate it first — so this always reads the unpatched constant
    even though the isolation fixture is autouse.
    """
    rcn = sys.modules.get(_RCN_MODULE)
    return None if rcn is None else getattr(rcn, '_RECORD_BELT_DIR', None)


@pytest.fixture(autouse=True)
def _isolate_toss_record_sinks(tmp_path, monkeypatch, real_record_belt_dir):
    """Redirect the record belt / trim proposal sinks into this test's tmp dir."""
    rcn = sys.modules.get(_RCN_MODULE)
    if rcn is None or real_record_belt_dir is None:
        # Not imported, or a deployment outside the repo checkout — in the
        # latter case the production code writes nothing at all.
        return
    monkeypatch.setattr(rcn, '_RECORD_BELT_DIR',
                        str(tmp_path / 'record_sinks'))
