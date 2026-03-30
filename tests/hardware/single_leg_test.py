#!/usr/bin/env python3
"""Standalone single-leg test harness for Jugglebot bench tests.

Bypasses ROS2 entirely and talks directly to ODrive axes via python-can.
Designed to run on the Jetson (socketcan) or a USB-CAN adapter.

Phase 2 tests (isolated leg, CAN/motor fundamentals):
  1. Torque passthrough smoke test
  2. Emergency stop test (single leg)
  3. Encoder sign check
  4. Force conversion validation

Phase 3 Stage A tests (isolated leg, position control with feedforward):
  5. Position control smoke test (set_input_pos, no feedforward)
  6. Velocity feedforward comparison (ramp with/without vel_ff)
  7. Gravity torque_ff validation (static hold with known weight)

Safety:
  - Uses conservative current limit (50% of rated = 10A)
  - All tests send IDLE on completion, error, or Ctrl-C
  - Heartbeat watchdog detects ODrive disconnection
  - Every test is gated on heartbeat reception before commanding

Usage:
  python tools/single_leg_test.py [--interface socketcan] [--channel can0] [--axis 0]
  python tools/single_leg_test.py --test phase3       # Phase 3 tests only (default)
  python tools/single_leg_test.py --test phase2       # Phase 2 tests only
  python tools/single_leg_test.py --test all          # run all tests sequentially
  python tools/single_leg_test.py --test pos_smoke    # run just one test
  python tools/single_leg_test.py --test vel_ff       # run just one test
  python tools/single_leg_test.py --test torque_ff    # run just one test

Requirements:
  pip install python-can numpy
"""

from __future__ import annotations

import argparse
import signal
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

import can
import numpy as np

# ---------------------------------------------------------------------------
# Add project root to path so we can import config modules directly
# ---------------------------------------------------------------------------
import os

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
_CONFIG_DIR = os.path.join(_PROJECT_ROOT, 'config', 'generated')
_ROS_PKG_DIR = os.path.join(_PROJECT_ROOT, 'ros_ws', 'src', 'jugglebot')

sys.path.insert(0, _CONFIG_DIR)
sys.path.insert(0, _ROS_PKG_DIR)
import protocol_config as proto  # noqa: E402
import hardware_config as hw     # noqa: E402

# ---------------------------------------------------------------------------
# Protocol constants (inline from protocol_config, no ROS2 package import)
# ---------------------------------------------------------------------------

COMMANDS = proto.ODRIVE_COMMANDS
AXIS_STATES = proto.ODRIVE_STATES
CONTROL_MODES = proto.ODRIVE_CONTROL_MODES
INPUT_MODES = proto.ODRIVE_INPUT_MODES

# Error code bitmask -> name (subset relevant for testing)
ERROR_CODES = {
    1: "INITIALIZING", 2: "SYSTEM_LEVEL", 4: "TIMING_ERROR",
    8: "MISSING_ESTIMATE", 16: "BAD_CONFIG", 32: "DRV_FAULT",
    64: "MISSING_INPUT", 256: "DC_BUS_OVER_VOLTAGE",
    512: "DC_BUS_UNDER_VOLTAGE", 1024: "DC_BUS_OVER_CURRENT",
    2048: "DC_BUS_OVER_REGEN_CURRENT", 4096: "CURRENT_LIMIT_VIOLATION",
    8192: "MOTOR_OVER_TEMP", 16384: "INVERTER_OVER_TEMP",
    32768: "VELOCITY_LIMIT_VIOLATION", 65536: "POSITION_LIMIT_VIOLATION",
    16777216: "WATCHDOG_TIMER_EXPIRED", 33554432: "ESTOP_REQUESTED",
    67108864: "SPINOUT_DETECTED", 134217728: "BRAKE_RESISTOR_DISARMED",
    268435456: "THERMISTOR_DISCONNECTED", 1073741824: "CALIBRATION_ERROR",
}

# Safety: 50% of rated leg current
SAFE_CURRENT_LIMIT_A = hw.ODRIVE_LEG_CURR_LIMIT_A * 0.5  # 10A

# Motor torque constant from Phase 2 multi-weight bench test (R^2=0.994)
KT_MEASURED = 0.0624  # Nm/A (measured), vs 0.0637 Nm/A datasheet

# ODrive leg feedforward int16 scaling — from protocol_config (source of truth).
# Must match the ODrive firmware config: axis0.config.can.input_vel_scale / input_torque_scale.
# int16_value = real_value * scale
LEG_VEL_FF_SCALE = proto.INPUT_SCALE_LEG_VEL   # 1000 (0.001 rev/s per LSB)
LEG_TOR_FF_SCALE = proto.INPUT_SCALE_LEG_TOR   # 10000 (0.0001 Nm per LSB)

# Spool geometry for force conversion (standard Jugglebot legs, from config)
MM_TO_REV = np.array(hw.GEOM_MM_TO_REV, dtype=np.float64)
SPOOL_RADIUS_MM = 1.0 / (2.0 * np.pi * MM_TO_REV)  # per-leg, ~11 mm

# Test leg measured geometry — this bench leg is NOT a standard Jugglebot leg.
# One motor revolution extends the leg by 71.5708 mm (measured).
TEST_LEG_MM_PER_REV = 71.5708
TEST_LEG_MM_TO_REV = 1.0 / TEST_LEG_MM_PER_REV          # ~0.01397 rev/mm
TEST_LEG_SPOOL_RADIUS_MM = TEST_LEG_MM_PER_REV / (2.0 * np.pi)  # ~11.39 mm


def arb_id(axis_id: int, command_name: str) -> int:
    """Compute CAN arbitration ID: (node_id << 5) | command_id."""
    return (axis_id << 5) | COMMANDS[command_name]


# ===========================================================================
# CAN message encoding (standalone -- mirrors can/odrive.py without imports)
# ===========================================================================

def encode_set_state(axis_id: int, state: str) -> can.Message:
    data = struct.pack('<I', AXIS_STATES[state]) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_requested_state'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_controller_mode(axis_id: int, control_mode: str,
                                input_mode: str) -> can.Message:
    data = struct.pack('<II', CONTROL_MODES[control_mode],
                       INPUT_MODES[input_mode])
    return can.Message(arbitration_id=arb_id(axis_id, 'set_controller_mode'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_input_torque(axis_id: int, torque: float) -> can.Message:
    data = struct.pack('<f', torque) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_input_torque'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_vel_curr_limits(axis_id: int, vel_limit: float,
                                curr_limit: float) -> can.Message:
    data = struct.pack('<ff', vel_limit, curr_limit)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_vel_curr_limits'),
                       data=data, dlc=8, is_extended_id=False)


def encode_clear_errors(axis_id: int) -> can.Message:
    return can.Message(arbitration_id=arb_id(axis_id, 'clear_errors'),
                       dlc=8, is_extended_id=False, data=bytes(8))


def encode_set_input_pos(axis_id: int, position: float,
                          vel_ff: int = 0, torque_ff: int = 0) -> can.Message:
    data = struct.pack('<fhh', position, vel_ff, torque_ff)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_input_pos'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_input_vel(axis_id: int, velocity: float,
                          torque_ff: float = 0.0) -> can.Message:
    data = struct.pack('<ff', velocity, torque_ff)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_input_vel'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_absolute_position(axis_id: int, position: float) -> can.Message:
    """Set the absolute encoder position (used after homing)."""
    data = struct.pack('<f', position) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_absolute_position'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_pos_gain(axis_id: int, pos_gain: float) -> can.Message:
    """Set position gain (ODrive pos_gain parameter)."""
    data = struct.pack('<f', pos_gain) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_pos_gain'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_vel_gains(axis_id: int, vel_gain: float,
                          vel_int_gain: float) -> can.Message:
    """Set velocity gain and velocity integrator gain."""
    data = struct.pack('<ff', vel_gain, vel_int_gain)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_vel_gains'),
                       data=data, dlc=8, is_extended_id=False)


# ===========================================================================
# CAN message decoding
# ===========================================================================

def decode_heartbeat(data: bytes):
    """Returns (current_state, procedure_result, trajectory_done)."""
    return data[4], data[5], bool(data[6] & 0x01)


def decode_error(data: bytes):
    """Returns (active_errors, disarm_reason) as bitmasks."""
    active_errors = struct.unpack_from('<I', data, 0)[0]
    disarm_reason = struct.unpack_from('<I', data, 4)[0]
    return active_errors, disarm_reason


def decode_encoder_estimate(data: bytes):
    """Returns (pos_estimate, vel_estimate) in rev and rev/s."""
    return struct.unpack_from('<ff', data)


def decode_iq(data: bytes):
    """Returns (iq_setpoint, iq_measured) in amps."""
    return struct.unpack_from('<ff', data)


def error_names(bitmask: int) -> list:
    return [name for code, name in ERROR_CODES.items() if bitmask & code]


# ===========================================================================
# Live state tracker for a single axis
# ===========================================================================

@dataclass
class AxisState:
    """Tracks live state of a single ODrive axis from CAN heartbeats."""
    axis_state: int = 0          # AXIS_STATES enum value
    procedure_result: int = 0
    trajectory_done: bool = False
    active_errors: int = 0
    disarm_reason: int = 0
    pos_rev: float = 0.0         # encoder position (Jugglebot convention: positive = extension)
    pos_rev_raw: float = 0.0     # raw ODrive encoder value (positive = retraction)
    vel_rps: float = 0.0         # encoder velocity (Jugglebot convention)
    vel_rps_raw: float = 0.0     # raw ODrive encoder velocity
    iq_setpoint: float = 0.0     # current setpoint in A
    iq_measured: float = 0.0     # current measured in A
    last_heartbeat: float = 0.0  # time.time() of last heartbeat
    heartbeat_count: int = 0     # total heartbeats received

    @property
    def has_errors(self) -> bool:
        return self.active_errors != 0

    @property
    def state_name(self) -> str:
        for name, val in AXIS_STATES.items():
            if val == self.axis_state:
                return name
        return f"UNKNOWN({self.axis_state})"

    @property
    def is_idle(self) -> bool:
        return self.axis_state == AXIS_STATES['IDLE']

    @property
    def is_closed_loop(self) -> bool:
        return self.axis_state == AXIS_STATES['CLOSED_LOOP']


# ===========================================================================
# Test harness core
# ===========================================================================

class SingleLegTestHarness:
    """Standalone CAN interface for single-leg ODrive bench testing.

    Architecture:
      - Owns a python-can Bus instance directly (no ROS2)
      - Polls CAN messages in a tight loop during all operations
      - Tracks axis state from heartbeats and encoder estimates
      - All commands go through send() which includes a short inter-message delay
      - idle_axis() is the universal safety action -- called on completion,
        error, Ctrl-C, and heartbeat timeout
    """

    CAN_SEND_DELAY_S = 0.002  # 2ms between consecutive sends (CAN pacing)
    HEARTBEAT_TIMEOUT_S = 2.0  # seconds without heartbeat = fault

    def __init__(self, axis_id: int = 0,
                 interface: str = 'socketcan', channel: str = 'can0'):
        self.axis_id = axis_id
        self.state = AxisState()
        self._bus: Optional[can.Bus] = None
        self._interface = interface
        self._channel = channel
        self._shutdown_requested = False

        # Map command IDs to handler methods
        self._handlers = {
            COMMANDS['heartbeat_message']: self._handle_heartbeat,
            COMMANDS['get_error']: self._handle_error,
            COMMANDS['get_encoder_estimate']: self._handle_encoder,
            COMMANDS['get_iq']: self._handle_iq,
        }

    # -- Lifecycle --------------------------------------------------

    def connect(self):
        """Open the CAN bus and wait for the first heartbeat."""
        print(f"Connecting to CAN bus: {self._channel} ({self._interface})")
        self._bus = can.Bus(channel=self._channel, interface=self._interface,
                            bitrate=proto.CAN_BAUD_RATE)
        # Flush stale messages
        while self._bus.recv(timeout=0):
            pass

        print(f"Waiting for heartbeat from axis {self.axis_id}...")
        deadline = time.time() + 5.0
        while time.time() < deadline:
            self._poll(timeout=0.1)
            if self.state.heartbeat_count > 0:
                print(f"  Heartbeat received! State: {self.state.state_name}, "
                      f"errors: {self.state.active_errors}")
                return
        raise TimeoutError(
            f"No heartbeat from axis {self.axis_id} after 5 seconds. "
            f"Is the ODrive powered on and connected?")

    def disconnect(self):
        """Safely idle the axis and close the CAN bus."""
        if self._bus is not None:
            try:
                self.idle_axis()
            except Exception:
                pass
            try:
                self._bus.shutdown()
            except Exception:
                pass
            self._bus = None
            print("CAN bus closed.")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    # -- CAN I/O --------------------------------------------------─

    def send(self, msg: can.Message):
        """Send a CAN message with inter-message pacing."""
        self._bus.send(msg)
        time.sleep(self.CAN_SEND_DELAY_S)

    def _poll(self, timeout: float = 0.0):
        """Read and dispatch all pending CAN messages."""
        while True:
            msg = self._bus.recv(timeout=timeout)
            if msg is None:
                break
            timeout = 0  # subsequent reads are non-blocking
            self._dispatch(msg)

    def _dispatch(self, msg: can.Message):
        """Route a CAN message to the appropriate handler."""
        axis_id = msg.arbitration_id >> 5
        cmd_id = msg.arbitration_id & 0x1F

        if axis_id != self.axis_id:
            return  # ignore other axes

        handler = self._handlers.get(cmd_id)
        if handler:
            handler(msg.data)

    def _handle_heartbeat(self, data: bytes):
        state, result, traj_done = decode_heartbeat(data)
        self.state.axis_state = state
        self.state.procedure_result = result
        self.state.trajectory_done = traj_done
        self.state.last_heartbeat = time.time()
        self.state.heartbeat_count += 1

    def _handle_error(self, data: bytes):
        errors, disarm = decode_error(data)
        self.state.active_errors = errors
        self.state.disarm_reason = disarm

    def _handle_encoder(self, data: bytes):
        pos, vel = decode_encoder_estimate(data)
        # Store raw values for position commands sent to ODrive
        self.state.pos_rev_raw = pos
        self.state.vel_rps_raw = vel
        # Leg inversion (matches can_node.py): positive = extension
        self.state.pos_rev = -pos
        self.state.vel_rps = -vel

    def _handle_iq(self, data: bytes):
        setpoint, measured = decode_iq(data)
        self.state.iq_setpoint = setpoint
        self.state.iq_measured = measured

    # -- Axis control ----------------------------------------------

    def idle_axis(self):
        """Send the axis to IDLE state (universal safety action)."""
        self.send(encode_set_state(self.axis_id, 'IDLE'))
        print(f"  Axis {self.axis_id} -> IDLE")

    def clear_errors(self):
        """Clear errors on the axis."""
        self.send(encode_clear_errors(self.axis_id))
        time.sleep(0.1)
        self._poll()
        print(f"  Errors cleared. Active errors: {self.state.active_errors}")

    def set_safe_limits(self):
        """Apply conservative velocity and current limits."""
        self.send(encode_set_vel_curr_limits(
            self.axis_id,
            vel_limit=hw.ODRIVE_LEG_VEL_LIMIT_RPS,
            curr_limit=SAFE_CURRENT_LIMIT_A))
        print(f"  Limits set: vel={hw.ODRIVE_LEG_VEL_LIMIT_RPS} rev/s, "
              f"curr={SAFE_CURRENT_LIMIT_A} A")

    def _wait_for_closed_loop(self, timeout_s: float = 2.0):
        """Poll CAN until axis reports CLOSED_LOOP, or raise on timeout.

        Retries the CLOSED_LOOP request after 500ms if still IDLE — handles
        CAN bus contention when multiple ODrive axes are connected.
        """
        retried = False
        deadline = time.time() + timeout_s
        retry_at = time.time() + 0.5
        while time.time() < deadline:
            self._poll(timeout=0.02)
            if self.state.is_closed_loop:
                return
            if not retried and time.time() >= retry_at and self.state.is_idle:
                self.send(encode_set_state(self.axis_id, 'CLOSED_LOOP'))
                retried = True
        raise RuntimeError(
            f"Failed to enter CLOSED_LOOP after {timeout_s}s. "
            f"State: {self.state.state_name}, "
            f"errors: {error_names(self.state.active_errors)}")

    def enter_torque_mode(self):
        """Put axis into TORQUE control with PASSTHROUGH input."""
        self.send(encode_set_controller_mode(
            self.axis_id, 'TORQUE', 'PASSTHROUGH'))
        time.sleep(0.05)  # allow controller mode to take effect
        # Send zero torque before entering closed loop
        self.send(encode_set_input_torque(self.axis_id, 0.0))
        time.sleep(0.05)
        self.send(encode_set_state(self.axis_id, 'CLOSED_LOOP'))
        self._wait_for_closed_loop()
        print(f"  Axis in TORQUE/PASSTHROUGH, CLOSED_LOOP")

    def enter_position_mode(self):
        """Put axis into POSITION control with PASSTHROUGH input.

        Sends the current encoder position as the initial setpoint before
        entering CLOSED_LOOP, so the leg holds its current position.
        """
        self._poll()  # ensure encoder is fresh
        pos_now_raw = self.state.pos_rev_raw
        self.send(encode_set_controller_mode(
            self.axis_id, 'POSITION', 'PASSTHROUGH'))
        time.sleep(0.05)  # allow controller mode to take effect
        # Send current position as initial setpoint (prevents jump on entry)
        self.send(encode_set_input_pos(self.axis_id, pos_now_raw))
        time.sleep(0.05)
        self.send(encode_set_state(self.axis_id, 'CLOSED_LOOP'))
        self._wait_for_closed_loop()
        print(f"  Axis in POSITION/PASSTHROUGH, CLOSED_LOOP "
              f"(holding at {pos_now_raw:.4f} rev raw)")

    def home_axis(self):
        """Home the axis by driving to end-stop, then set absolute position.

        Mirrors the homing sequence in can_node.py:
          1. Enter VELOCITY/VEL_RAMP mode
          2. Drive at homing speed toward the retracted end-stop
          3. Monitor current with EMA until it exceeds the limit
          4. Set absolute position to HOMING_LEG_ABS_POS_REV (0.1 rev)
          5. Return to IDLE

        After homing: raw encoder 0.1 = fully retracted, increasing raw = more
        retracted (past the stop). Inverted encoder -0.1 = fully retracted,
        more negative = more retracted.
        """
        print(f"  Homing axis {self.axis_id}...")

        self.clear_errors()
        self.set_safe_limits()

        # Enter velocity mode
        self.send(encode_set_controller_mode(
            self.axis_id, 'VELOCITY', 'VEL_RAMP'))
        time.sleep(0.05)
        self.send(encode_set_vel_curr_limits(
            self.axis_id,
            vel_limit=abs(hw.HOMING_LEG_SPEED_RPS * 2),
            curr_limit=hw.HOMING_LEG_CURRENT_LIMIT_A + hw.HOMING_LEG_CURRENT_HEADROOM_A))
        time.sleep(0.05)
        self.send(encode_set_state(self.axis_id, 'CLOSED_LOOP'))
        self._wait_for_closed_loop()

        # Drive toward end-stop
        self.send(encode_set_input_vel(self.axis_id, hw.HOMING_LEG_SPEED_RPS))

        # Let the velocity ramp up before monitoring current — VEL_RAMP needs
        # time to accelerate, and there may be brief inrush transients.
        print(f"  Ramping up (1s grace period)...")
        self.poll_for(1.0)

        avg = 0.0
        deadline = time.time() + hw.HOMING_MOTOR_TIMEOUT_S
        while time.time() < deadline:
            self._poll(timeout=0.01)
            iq = self.state.iq_measured
            avg = avg * hw.HOMING_EMA_WEIGHT + iq * (1.0 - hw.HOMING_EMA_WEIGHT)
            if abs(avg) >= hw.HOMING_LEG_CURRENT_LIMIT_A:
                print(f"  End-stop detected (avg current: {avg:.2f} A)")
                break
        else:
            self.idle_axis()
            raise RuntimeError(
                f"Homing timeout after {hw.HOMING_MOTOR_TIMEOUT_S}s")

        self.idle_axis()
        time.sleep(0.2)
        self._poll()

        # Set absolute position to 0 (fully compressed = origin)
        self.send(encode_set_absolute_position(self.axis_id, 0.0))
        time.sleep(0.1)
        self._poll()

        print(f"  Homed! Encoder set to 0 rev (fully compressed)")
        print(f"  Current position: raw={self.state.pos_rev_raw:.4f}, "
              f"inverted={self.state.pos_rev:.4f} rev")

    def check_heartbeat_fresh(self) -> bool:
        """Check that we've received a heartbeat recently."""
        if self.state.heartbeat_count == 0:
            return False
        age = time.time() - self.state.last_heartbeat
        return age < self.HEARTBEAT_TIMEOUT_S

    def poll_for(self, duration_s: float, interval_s: float = 0.01):
        """Poll CAN messages for a fixed duration, checking heartbeat health."""
        deadline = time.time() + duration_s
        while time.time() < deadline:
            self._poll(timeout=interval_s)
            if not self.check_heartbeat_fresh():
                raise RuntimeError("Heartbeat timeout -- ODrive may be disconnected!")

    def flush_and_resync(self):
        """Drain stale CAN buffer and wait for a fresh heartbeat.

        Call this after any blocking operation (e.g. input()) that may have
        let the socketcan receive buffer overflow, dropping recent heartbeats.
        """
        # Drain all stale buffered messages
        while self._bus.recv(timeout=0):
            pass
        # Wait for a fresh heartbeat (up to 1 second)
        deadline = time.time() + 1.0
        hb_before = self.state.heartbeat_count
        while time.time() < deadline:
            self._poll(timeout=0.05)
            if self.state.heartbeat_count > hb_before:
                return
        raise RuntimeError("No fresh heartbeat after flush -- ODrive may be disconnected!")

    def require_no_errors(self):
        """Assert the axis has no active errors."""
        self._poll()
        if self.state.has_errors:
            names = error_names(self.state.active_errors)
            raise RuntimeError(f"Axis has errors: {names}")

    # -- Utility: measure encoder at rest --------------------------

    def read_encoder(self, settle_time: float = 0.3) -> tuple:
        """Read encoder after settling. Returns (pos_rev, vel_rps)."""
        self.poll_for(settle_time)
        return self.state.pos_rev, self.state.vel_rps


# ===========================================================================
# Test implementations
# ===========================================================================

def test_smoke(harness: SingleLegTestHarness):
    """Test 1: Single-leg torque passthrough smoke test.

    From MOTION_PLANNER_PLAN Phase 2:
    "Command a constant small torque on the isolated leg, verify the leg
    moves and encoder feedback is received correctly. Verify the leg stops
    cleanly on mode switch to idle."

    Procedure:
      1. Clear errors, set safe current limits
      2. Enter TORQUE/PASSTHROUGH mode
      3. Record initial encoder position
      4. Command a small constant torque (0.08 Nm) for 2 seconds
      5. Verify encoder position changed (leg moved)
      6. Switch to IDLE
      7. Wait 1 second, verify velocity is near zero (leg stopped)
    """
    print("\n" + "=" * 60)
    print("TEST 1: Torque passthrough smoke test")
    print("=" * 60)
    TEST_TORQUE_NM = 0.08  # very small -- just enough to move an unloaded leg
    TEST_DURATION_S = 2.0

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    pos_before, _ = harness.read_encoder()
    print(f"  Initial position: {pos_before:.4f} rev")

    harness.enter_torque_mode()

    # Command constant torque
    print(f"  Commanding {TEST_TORQUE_NM} Nm for {TEST_DURATION_S}s...")
    deadline = time.time() + TEST_DURATION_S
    while time.time() < deadline:
        harness.send(encode_set_input_torque(harness.axis_id, TEST_TORQUE_NM))
        harness.poll_for(0.05)

    pos_after, vel_during = harness.read_encoder(settle_time=0.1)
    print(f"  Position after torque: {pos_after:.4f} rev "
          f"(delta: {pos_after - pos_before:+.4f} rev)")

    # Switch to IDLE
    harness.idle_axis()
    time.sleep(1.0)
    harness.poll_for(0.5)
    vel_after_idle = harness.state.vel_rps
    print(f"  Velocity after IDLE: {vel_after_idle:.4f} rev/s")

    # Verify
    moved = abs(pos_after - pos_before) > 0.01  # >0.01 rev = ~0.7mm
    stopped = abs(vel_after_idle) < 0.05  # <0.05 rev/s

    if moved and stopped:
        print("  PASS: Leg moved under torque and stopped cleanly on IDLE")
        return True
    else:
        if not moved:
            print(f"  FAIL: Leg did not move (delta={pos_after - pos_before:.4f} rev)")
        if not stopped:
            print(f"  FAIL: Leg did not stop (vel={vel_after_idle:.4f} rev/s)")
        return False


def test_estop(harness: SingleLegTestHarness):
    """Test 2: Emergency stop test (single leg).

    From MOTION_PLANNER_PLAN Phase 2:
    "Verify that the control process correctly idles the leg on IPC loss,
    process crash, or explicit stop command. Test each failure mode
    individually."

    Since this is the standalone harness (no IPC layer), we test:
      a) Explicit IDLE command stops the leg during torque application
      b) Verify axis transitions to IDLE state within 100ms
      c) Verify no errors after e-stop
    """
    print("\n" + "=" * 60)
    print("TEST 2: Emergency stop test")
    print("=" * 60)
    TEST_TORQUE_NM = 0.08  # same torque as smoke test to ensure leg is moving before e-stop  

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    harness.enter_torque_mode()

    # Command torque to get the leg moving
    print(f"  Commanding {TEST_TORQUE_NM} Nm to get leg moving...")
    for _ in range(20):  # ~1 second
        harness.send(encode_set_input_torque(harness.axis_id, TEST_TORQUE_NM))
        harness.poll_for(0.05)

    vel_before_stop = harness.state.vel_rps
    print(f"  Velocity before stop: {vel_before_stop:.4f} rev/s")

    # EMERGENCY: send IDLE
    t_stop = time.time()
    harness.idle_axis()

    # Poll until we see IDLE state
    while time.time() - t_stop < 1.0:
        harness._poll(timeout=0.01)
        if harness.state.is_idle:
            break

    stop_latency_ms = (time.time() - t_stop) * 1000
    print(f"  IDLE confirmed in {stop_latency_ms:.1f} ms")

    time.sleep(0.5)
    harness.poll_for(0.3)

    harness.require_no_errors()
    vel_after = harness.state.vel_rps

    idle_confirmed = harness.state.is_idle
    fast_enough = stop_latency_ms < 100
    no_residual = abs(vel_after) < 0.05

    if idle_confirmed and fast_enough and no_residual:
        print("  PASS: E-stop confirmed -- IDLE within 100ms, no errors, "
              "velocity settled to zero")
        return True
    else:
        if not idle_confirmed:
            print(f"  FAIL: Axis not in IDLE (state={harness.state.state_name})")
        if not fast_enough:
            print(f"  FAIL: IDLE took {stop_latency_ms:.1f}ms (>100ms)")
        if not no_residual:
            print(f"  FAIL: Residual velocity {vel_after:.4f} rev/s")
        return False


def test_encoder_sign(harness: SingleLegTestHarness):
    """Test 3: Encoder direction and sign convention check.

    From MOTION_PLANNER_PLAN Phase 2:
    "Command a small positive torque pulse, verify encoder position increases
    in the expected direction. Repeat for negative torque."

    Convention: Jugglebot legs use inverted motor positions -- negative motor
    position = leg extension. So positive torque should cause negative encoder
    movement (leg extending), and negative torque should cause positive encoder
    movement (leg retracting).

    Note: On a bench-mounted leg with no load, the relationship may vary
    depending on orientation. This test documents the measured sign convention
    so it can be verified against the kinematic model.
    """
    print("\n" + "=" * 60)
    print("TEST 3: Encoder sign convention check")
    print("=" * 60)
    PULSE_TORQUE_NM = 0.08  # same small torque as smoke test to ensure measurable movement without overshooting
    PULSE_DURATION_S = 1.0

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    results = {}
    for direction, torque in [("positive", PULSE_TORQUE_NM),
                               ("negative", -PULSE_TORQUE_NM)]:
        print(f"\n  --- {direction.upper()} torque pulse ({torque:+.3f} Nm) ---")

        harness.enter_torque_mode()
        pos_before, _ = harness.read_encoder()
        print(f"  Position before: {pos_before:.4f} rev")

        deadline = time.time() + PULSE_DURATION_S
        while time.time() < deadline:
            harness.send(encode_set_input_torque(harness.axis_id, torque))
            harness.poll_for(0.05)

        harness.idle_axis()
        time.sleep(0.5)
        pos_after, _ = harness.read_encoder()
        delta = pos_after - pos_before
        print(f"  Position after:  {pos_after:.4f} rev (delta: {delta:+.4f} rev)")

        results[direction] = delta

        # Let the leg settle before next pulse
        harness.poll_for(1.0)

    pos_delta = results["positive"]
    neg_delta = results["negative"]
    opposite_signs = (pos_delta > 0) != (neg_delta > 0)
    both_moved = abs(pos_delta) > 0.005 and abs(neg_delta) > 0.005

    print(f"\n  Summary:")
    print(f"    Positive torque -> encoder delta: {pos_delta:+.4f} rev")
    print(f"    Negative torque -> encoder delta: {neg_delta:+.4f} rev")
    print(f"    Expected: positive torque = negative encoder (extension)")

    if pos_delta < 0:
        print(f"    Convention MATCHES Jugglebot model "
              f"(positive torque -> negative encoder = extension)")
    else:
        print(f"    Convention INVERTED vs Jugglebot model "
              f"(positive torque -> positive encoder). "
              f"can_node.py leg inversion handles this.")

    if both_moved and opposite_signs:
        print("  PASS: Both directions move, signs are opposite (consistent)")
        return True
    else:
        if not both_moved:
            print(f"  FAIL: Insufficient movement "
                  f"(pos_delta={pos_delta:.4f}, neg_delta={neg_delta:.4f})")
        if not opposite_signs:
            print(f"  FAIL: Signs are NOT opposite -- something is wrong")
        return False


def _collect_iq_samples(harness: SingleLegTestHarness,
                        duration_s: float = 3.0) -> np.ndarray:
    """Collect iq_measured samples for a fixed duration while position-holding."""
    samples = []
    deadline = time.time() + duration_s
    while time.time() < deadline:
        harness._poll(timeout=0.02)
        samples.append(harness.state.iq_measured)
    return np.array(samples)


def test_force_conversion(harness: SingleLegTestHarness):
    """Test 4: Force conversion validation (multi-weight calibration).

    From MOTION_PLANNER_PLAN Phase 2:
    "Apply a known static load to the isolated leg (e.g., hang a calibrated
    weight) and command zero torque. Measure the motor current required to
    hold position via PD. Compare the measured current to the predicted
    current from the geometric force-to-torque conversion."

    This test uses multiple weights to fit Kt from the slope of
    iq_measured vs predicted_torque. This is more robust than a single
    measurement because:
      - The slope cancels out constant offsets (friction, PD bias)
      - Kt is determined from the data rather than assumed from a datasheet
      - Linearity confirms the spool geometry model is correct

    Procedure:
      1. Enter position hold at current position
      2. Prompt user to attach/change weights, measuring iq at each
      3. Fit a line: iq = (1/Kt) * tau_predicted + offset
      4. Report Kt, offset (friction), R^2 (linearity), and discrepancy
    """
    print("\n" + "=" * 60)
    print("TEST 4: Force conversion validation (multi-weight)")
    print("=" * 60)

    spool_radius_m = SPOOL_RADIUS_MM[harness.axis_id] / 1000.0
    print(f"\n  Spool radius (axis {harness.axis_id}): "
          f"{SPOOL_RADIUS_MM[harness.axis_id]:.2f} mm")
    print(f"\n  This test measures motor current at multiple known loads.")
    print(f"  The leg should be oriented so the weight hangs vertically.")
    print(f"  You will be prompted to add/change weights between measurements.")
    print(f"  Enter at least 3 different weights for a good fit.")
    print(f"  Type 'done' when finished, or 'skip' to skip entirely.\n")

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    # Enter position hold once for the entire test
    # Use raw encoder value for ODrive position commands (ODrive convention)
    pos_now_raw = harness.state.pos_rev_raw
    harness.read_encoder()  # ensure encoder is fresh
    pos_now_raw = harness.state.pos_rev_raw
    print(f"  Current position: {pos_now_raw:.4f} rev (raw ODrive)")
    print(f"  Entering POSITION/PASSTHROUGH to hold position...\n")

    harness.send(encode_set_controller_mode(
        harness.axis_id, 'POSITION', 'PASSTHROUGH'))
    time.sleep(0.01)
    harness.send(encode_set_input_pos(harness.axis_id, pos_now_raw))
    time.sleep(0.01)
    harness.send(encode_set_state(harness.axis_id, 'CLOSED_LOOP'))
    harness._wait_for_closed_loop()

    # Collect measurements at multiple weights
    measurements = []  # list of (mass_kg, predicted_torque_Nm, iq_mean, iq_std)
    point_num = 0

    while True:
        point_num += 1
        weight_str = input(
            f"  [{point_num}] Enter mass in kg "
            f"(or 'done'/'skip'): ").strip().lower()
        harness.flush_and_resync()

        if weight_str == 'skip' and len(measurements) == 0:
            print("  Skipping force conversion test.")
            harness.idle_axis()
            return None

        if weight_str == 'done':
            break

        try:
            mass_kg = float(weight_str)
        except ValueError:
            print(f"  Invalid input, try again.")
            point_num -= 1
            continue

        force_N = mass_kg * hw.GRAVITY_MPS2
        predicted_torque = force_N * spool_radius_m

        # Let the system settle with the new weight, then measure
        print(f"    Settling (2s)...", end='', flush=True)
        harness.poll_for(2.0)
        print(f" Measuring (3s)...", end='', flush=True)
        iq_samples = _collect_iq_samples(harness, duration_s=3.0)
        iq_mean = float(np.mean(iq_samples))
        iq_std = float(np.std(iq_samples))
        measurements.append((mass_kg, predicted_torque, iq_mean, iq_std))
        print(f" done.")
        print(f"    {mass_kg:.3f} kg -> tau={predicted_torque:.4f} Nm, "
              f"iq={iq_mean:.4f} A (std={iq_std:.4f})")

    harness.idle_axis()

    if len(measurements) < 2:
        print(f"\n  Need at least 2 measurements for a fit "
              f"(got {len(measurements)}). Cannot determine Kt.")
        if len(measurements) == 1:
            m = measurements[0]
            print(f"  Single point: {m[0]:.3f} kg, tau={m[1]:.4f} Nm, "
                  f"iq={m[2]:.4f} A")
        return False

    # Extract arrays for fitting
    torques = np.array([m[1] for m in measurements])
    iqs = np.array([m[2] for m in measurements])

    # Linear fit: iq = slope * tau + intercept
    # slope = 1/Kt, intercept = friction/bias offset
    coeffs = np.polyfit(torques, iqs, 1)
    slope = coeffs[0]       # 1/Kt  (A/Nm)
    intercept = coeffs[1]   # bias current (A)

    # R^2
    iq_predicted = np.polyval(coeffs, torques)
    ss_res = np.sum((iqs - iq_predicted) ** 2)
    ss_tot = np.sum((iqs - np.mean(iqs)) ** 2)
    r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

    print(f"\n  " + "-" * 56)
    print(f"  {'Mass (kg)':>10s}  {'Tau pred (Nm)':>14s}  "
          f"{'iq mean (A)':>12s}  {'iq std (A)':>11s}")
    print(f"  " + "-" * 56)
    for m in measurements:
        print(f"  {m[0]:10.3f}  {m[1]:14.4f}  {m[2]:12.4f}  {m[3]:11.4f}")
    print(f"  " + "-" * 56)

    print(f"\n  Linear fit: iq = {slope:.2f} * tau + ({intercept:.4f})")
    print(f"  R^2 = {r_squared:.4f}")

    if abs(slope) > 1e-6:
        kt_measured = 1.0 / slope
        print(f"\n  Measured Kt = 1/slope = {kt_measured:.4f} Nm/A")
        print(f"  Bias current (intercept) = {intercept:.4f} A "
              f"(friction / PD offset)")

        # Compare against datasheet if user wants
        print(f"\n  To validate the spool geometry conversion:")
        print(f"    Compare measured Kt ({kt_measured:.4f} Nm/A) against "
              f"motor datasheet Kt.")
        print(f"    For Kv=150 rpm/V: Kt = 60/(2*pi*150) = 0.0637 Nm/A")
        kt_datasheet = 60.0 / (2.0 * np.pi * 150.0)
        discrepancy_pct = abs(abs(kt_measured) - kt_datasheet) / kt_datasheet * 100
        print(f"    Discrepancy: {discrepancy_pct:.1f}%"
              f"{'  (< 10% -- PASS)' if discrepancy_pct < 10 else '  (> 10% -- investigate)'}")
    else:
        print(f"\n  WARNING: slope near zero -- current does not vary with load.")
        print(f"  The motor may not be loaded by the weights (check orientation).")
        return False

    if r_squared > 0.95 and len(measurements) >= 3:
        print(f"\n  PASS: Linear fit is good (R^2={r_squared:.4f}), "
              f"Kt = {kt_measured:.4f} Nm/A")
        return True
    elif r_squared > 0.95:
        print(f"\n  FIT OK but only {len(measurements)} points -- "
              f"add more for confidence")
        return True
    else:
        print(f"\n  WARNING: Poor linearity (R^2={r_squared:.4f}). "
              f"Check setup: vertical orientation, taut string, "
              f"consistent attachment point.")
        return False


# ===========================================================================
# Phase 3 Stage A: Position control with feedforward tests
# ===========================================================================

def test_pos_smoke(harness: SingleLegTestHarness):
    """Stage A1: Position control smoke test.

    From MOTION_PLANNER_PLAN Phase 3 Stage A:
    "Command a series of position setpoints via set_input_pos (no vel_ff or
    torque_ff yet). Verify the leg moves to each target accurately and holds
    without oscillation."

    Procedure:
      1. Enter POSITION/PASSTHROUGH mode, holding current position
      2. Command a sequence of relative position steps: +20, +20, -30, -10 mm
         (net displacement = 0, returns to start)
      3. At each step: wait for settling (2s), measure position error
      4. Verify accuracy < 1 mm at each setpoint
      5. Verify net return accuracy < 0.5 mm

    All positions are commanded in raw ODrive convention (negative = extension).
    No vel_ff or torque_ff — pure position PID.
    """
    print("\n" + "=" * 60)
    print("STAGE A1: Position control smoke test")
    print("=" * 60)

    mm_to_rev_axis = TEST_LEG_MM_TO_REV
    SETTLE_TIME_S = 2.0
    ACCURACY_MM = 1.0     # per-step accuracy
    RETURN_MM = 0.5       # net return accuracy

    # Steps in mm (positive = extension = negative raw)
    steps_mm = [20.0, 20.0, -30.0, -10.0]

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    harness.enter_position_mode()

    start_raw = harness.state.pos_rev_raw
    start_mm = harness.state.pos_rev / mm_to_rev_axis
    print(f"  Start position: {start_raw:.4f} rev raw ({start_mm:.2f} mm)")

    cumulative_mm = 0.0
    errors_mm = []
    all_pass = True

    for i, step_mm in enumerate(steps_mm, 1):
        cumulative_mm += step_mm
        # Extension = positive mm = negative raw rev
        target_raw = start_raw - cumulative_mm * mm_to_rev_axis
        print(f"\n  Step {i}: {step_mm:+.0f} mm -> "
              f"target {target_raw:.4f} rev raw")

        harness.send(encode_set_input_pos(harness.axis_id, target_raw))
        harness.poll_for(SETTLE_TIME_S)

        actual_raw = harness.state.pos_rev_raw
        error_rev = abs(target_raw - actual_raw)
        error_mm = error_rev / mm_to_rev_axis
        errors_mm.append(error_mm)
        print(f"    Actual: {actual_raw:.4f} rev raw, "
              f"error: {error_mm:.3f} mm", end='')
        if error_mm > ACCURACY_MM:
            print(f"  EXCEEDS {ACCURACY_MM} mm")
            all_pass = False
        else:
            print(f"  OK")

    # Check net return
    final_mm = harness.state.pos_rev / mm_to_rev_axis
    return_err = abs(final_mm - start_mm)
    print(f"\n  Return error: {return_err:.3f} mm "
          f"(start: {start_mm:.2f}, final: {final_mm:.2f})")

    harness.idle_axis()

    good_return = return_err < RETURN_MM
    if all_pass and good_return:
        print(f"  PASS: All steps within {ACCURACY_MM} mm, "
              f"return within {RETURN_MM} mm")
        return True
    else:
        if not all_pass:
            print(f"  FAIL: Step errors: "
                  f"{[f'{e:.3f}' for e in errors_mm]}")
        if not good_return:
            print(f"  FAIL: Return error {return_err:.3f} mm "
                  f"exceeds {RETURN_MM} mm")
        return False


def test_vel_ff(harness: SingleLegTestHarness):
    """Stage A2: Velocity feedforward comparison.

    From MOTION_PLANNER_PLAN Phase 3 Stage A:
    "Command a slow position ramp with and without vel_ff. Compare tracking
    error during the ramp. With vel_ff, the ODrive's position loop should
    anticipate the motion rather than reacting to position error."

    Procedure:
      1. Enter POSITION/PASSTHROUGH mode
      2. Phase A: ramp 40 mm at 10 mm/s with vel_ff=0, measure tracking error
      3. Return to start position, settle
      4. Phase B: same ramp with vel_ff set, measure tracking error
      5. Compare RMS errors -- vel_ff should reduce tracking lag

    Ramp direction: extension (negative raw).
    vel_ff int16 = int(-speed_rps * LEG_VEL_FF_SCALE) for extension direction.
    """
    print("\n" + "=" * 60)
    print("STAGE A2: Velocity feedforward comparison")
    print("=" * 60)

    mm_to_rev_axis = TEST_LEG_MM_TO_REV
    RAMP_DISTANCE_MM = 40.0
    RAMP_SPEED_MM_S = 10.0
    RAMP_DURATION_S = RAMP_DISTANCE_MM / RAMP_SPEED_MM_S  # 4 seconds
    LOOP_DT = 0.01  # 100 Hz command rate

    ramp_speed_rps = RAMP_SPEED_MM_S * mm_to_rev_axis
    vel_ff_int16 = int(-ramp_speed_rps * LEG_VEL_FF_SCALE)  # negative = extension
    vel_ff_int16 = max(-32767, min(32767, vel_ff_int16))

    print(f"  Ramp: {RAMP_DISTANCE_MM:.0f} mm at {RAMP_SPEED_MM_S:.0f} mm/s "
          f"({RAMP_DURATION_S:.1f}s)")
    print(f"  Speed: {ramp_speed_rps:.4f} rev/s, vel_ff int16: {vel_ff_int16}")

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    harness.enter_position_mode()
    start_raw = harness.state.pos_rev_raw
    print(f"  Start position: {start_raw:.4f} rev raw")

    def run_ramp(use_vel_ff: bool) -> float:
        """Execute a linear position ramp. Returns RMS tracking error in mm."""
        vff = vel_ff_int16 if use_vel_ff else 0
        errors_mm = []

        t_start = time.time()
        while True:
            harness._poll(timeout=LOOP_DT)
            elapsed = time.time() - t_start
            if elapsed >= RAMP_DURATION_S:
                break

            # Linear ramp: extension = increasingly negative raw
            progress_rev = ramp_speed_rps * elapsed
            target_raw = start_raw - progress_rev

            harness.send(encode_set_input_pos(
                harness.axis_id, target_raw, vel_ff=vff))

            # Tracking error in mm
            actual_extension_rev = -(harness.state.pos_rev_raw - start_raw)
            actual_mm = actual_extension_rev / mm_to_rev_axis
            target_mm = progress_rev / mm_to_rev_axis
            error_mm = target_mm - actual_mm
            errors_mm.append(error_mm)

        arr = np.array(errors_mm) if errors_mm else np.zeros(1)
        return float(np.sqrt(np.mean(arr ** 2)))

    # Phase A: ramp without vel_ff
    print(f"\n  Phase A: Ramp without vel_ff...")
    rms_no_ff = run_ramp(use_vel_ff=False)
    print(f"    Tracking error RMS: {rms_no_ff:.3f} mm")

    # Return to start
    print(f"  Returning to start...")
    harness.send(encode_set_input_pos(harness.axis_id, start_raw))
    harness.poll_for(3.0)

    # Phase B: ramp with vel_ff
    print(f"  Phase B: Ramp with vel_ff (int16={vel_ff_int16})...")
    rms_with_ff = run_ramp(use_vel_ff=True)
    print(f"    Tracking error RMS: {rms_with_ff:.3f} mm")

    # Return to start and idle
    harness.send(encode_set_input_pos(harness.axis_id, start_raw))
    harness.poll_for(2.0)
    harness.idle_axis()

    # Compare
    if rms_no_ff > 0.01:
        improvement_pct = (1.0 - rms_with_ff / rms_no_ff) * 100
    else:
        improvement_pct = 0.0

    print(f"\n  Summary:")
    print(f"    RMS error (no vel_ff):   {rms_no_ff:.3f} mm")
    print(f"    RMS error (with vel_ff): {rms_with_ff:.3f} mm")
    print(f"    Improvement:             {improvement_pct:.1f}%")

    if rms_with_ff <= rms_no_ff:
        print(f"  PASS: vel_ff reduced tracking error by {improvement_pct:.1f}%")
        return True
    elif rms_with_ff < rms_no_ff * 1.1:
        print(f"  PASS: vel_ff tracking within 10% of baseline")
        return True
    else:
        print(f"  FAIL: vel_ff INCREASED tracking error — check sign convention")
        return False


def test_torque_ff(harness: SingleLegTestHarness):
    """Stage A3: Gravity torque_ff validation.

    From MOTION_PLANNER_PLAN Phase 3 Stage A:
    "With the leg oriented vertically and a known mass attached, command a
    static position hold with and without torque_ff. Measure the ODrive's
    motor current in both cases."

    This test validates the torque_ff feedforward field end-to-end:
    weight -> force -> spool geometry -> motor torque -> int16 encoding.

    Procedure:
      1. Prompt user for attached weight mass
      2. Enter POSITION/PASSTHROUGH, hold current position
      3. Phase A: hold with torque_ff=0, measure baseline iq (3s)
      4. Compute gravity torque from weight and spool geometry
      5. Determine torque_ff sign from measured iq direction
      6. Validate magnitude: compare computed torque/Kt vs measured iq
      7. Phase B: hold with torque_ff enabled, measure iq (3s)
      8. Verify position holding not degraded

    Uses KT_MEASURED = 0.0624 Nm/A from Phase 2 bench test results.
    """
    print("\n" + "=" * 60)
    print("STAGE A3: Gravity torque_ff validation")
    print("=" * 60)

    spool_radius_m = TEST_LEG_SPOOL_RADIUS_MM / 1000.0
    mm_to_rev_axis = TEST_LEG_MM_TO_REV

    print(f"\n  This test validates the torque_ff feedforward field by comparing")
    print(f"  ODrive current draw with and without gravity compensation.")
    print(f"\n  Orient the leg vertically and attach a known weight.")
    weight_str = input(f"\n  Enter weight in kg (or 'skip'): ").strip().lower()
    harness.flush_and_resync()

    if weight_str == 'skip':
        print("  Skipping torque_ff test.")
        return None

    try:
        weight_kg = float(weight_str)
    except ValueError:
        print("  Invalid input. Skipping.")
        return None

    force_N = weight_kg * hw.GRAVITY_MPS2
    gravity_torque_Nm = force_N * spool_radius_m
    predicted_iq = gravity_torque_Nm / KT_MEASURED

    print(f"  Weight: {weight_kg:.3f} kg")
    print(f"  Force:  {force_N:.3f} N")
    print(f"  Torque: {gravity_torque_Nm:.4f} Nm")
    print(f"  Predicted iq (torque/Kt): {predicted_iq:.4f} A")

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    harness.enter_position_mode()

    # Extend the leg so the motor is actually holding the weight, not the
    # end-stop.  Stay in POSITION/PASSTHROUGH — no IDLE gap that would let
    # the weight pull the leg back to the compressed position.
    EXTEND_MM = 40.0
    extend_rev = EXTEND_MM * mm_to_rev_axis
    hold_raw = harness.state.pos_rev_raw - extend_rev  # negative = extension
    print(f"\n  Extending {EXTEND_MM:.0f} mm to clear end-stop "
          f"(target: {hold_raw:.4f} rev raw)...")
    harness.send(encode_set_input_pos(harness.axis_id, hold_raw))
    harness.poll_for(3.0)  # generous settle time with load
    actual_raw = harness.state.pos_rev_raw
    print(f"  Reached: {actual_raw:.4f} rev raw "
          f"(error: {abs(hold_raw - actual_raw) / mm_to_rev_axis:.2f} mm)")

    # Phase A: baseline hold (no torque_ff)
    print(f"\n  Phase A: Hold with no torque_ff (5s)...")
    print(f"    Settling (2s)...", end='', flush=True)
    harness.poll_for(2.0)
    print(f" Measuring (3s)...", end='', flush=True)
    iq_baseline = _collect_iq_samples(harness, duration_s=3.0)
    iq_baseline_mean = float(np.mean(iq_baseline))
    iq_baseline_rms = float(np.sqrt(np.mean(iq_baseline ** 2)))
    pos_baseline = harness.state.pos_rev_raw
    print(f" done.")
    print(f"    iq mean: {iq_baseline_mean:.4f} A, "
          f"RMS: {iq_baseline_rms:.4f} A")

    # Determine torque_ff sign from measured iq direction
    if abs(iq_baseline_mean) < 0.01:
        print(f"  WARNING: Baseline iq very small ({iq_baseline_mean:.4f} A). "
              f"Is the weight attached and leg vertical?")
        torque_ff_Nm = gravity_torque_Nm  # best guess
    else:
        torque_ff_Nm = abs(gravity_torque_Nm) * np.sign(iq_baseline_mean)

    # Validate magnitude
    discrepancy_pct = (abs(abs(iq_baseline_mean) - predicted_iq)
                       / max(predicted_iq, 0.001) * 100)

    print(f"\n  Magnitude check:")
    print(f"    Predicted iq: {predicted_iq:.4f} A")
    print(f"    Measured iq:  {abs(iq_baseline_mean):.4f} A")
    print(f"    Discrepancy:  {discrepancy_pct:.1f}%")

    magnitude_ok = discrepancy_pct < 25.0
    if not magnitude_ok:
        print(f"    WARNING: Discrepancy > 25% — "
              f"check weight/orientation/Kt")

    # Encode torque_ff as int16
    torque_ff_int16 = int(torque_ff_Nm * LEG_TOR_FF_SCALE)
    torque_ff_int16 = max(-32767, min(32767, torque_ff_int16))
    print(f"\n  torque_ff: {torque_ff_Nm:.4f} Nm, "
          f"int16: {torque_ff_int16}")

    # Phase B: hold with torque_ff
    print(f"\n  Phase B: Hold with torque_ff (5s)...")
    print(f"    Settling (2s)...", end='', flush=True)
    settle_deadline = time.time() + 2.0
    while time.time() < settle_deadline:
        harness.send(encode_set_input_pos(
            harness.axis_id, hold_raw, torque_ff=torque_ff_int16))
        harness._poll(timeout=0.05)
    print(f" Measuring (3s)...", end='', flush=True)
    iq_ff_samples = []
    measure_deadline = time.time() + 3.0
    while time.time() < measure_deadline:
        harness.send(encode_set_input_pos(
            harness.axis_id, hold_raw, torque_ff=torque_ff_int16))
        harness._poll(timeout=0.02)
        iq_ff_samples.append(harness.state.iq_measured)
    iq_ff = np.array(iq_ff_samples)
    iq_ff_mean = float(np.mean(iq_ff))
    iq_ff_rms = float(np.sqrt(np.mean(iq_ff ** 2)))
    pos_ff = harness.state.pos_rev_raw
    print(f" done.")
    print(f"    iq mean: {iq_ff_mean:.4f} A, RMS: {iq_ff_rms:.4f} A")

    harness.idle_axis()
    time.sleep(0.5)
    harness._poll()

    # Analysis
    pos_drift_mm = abs(pos_ff - pos_baseline) / mm_to_rev_axis
    iq_reduction = abs(iq_baseline_mean) - abs(iq_ff_mean)

    print(f"\n  Summary:")
    print(f"    Baseline iq mean: {iq_baseline_mean:.4f} A")
    print(f"    With FF iq mean:  {iq_ff_mean:.4f} A")
    print(f"    iq reduction:     {iq_reduction:.4f} A")
    if abs(iq_baseline_mean) > 0.01:
        print(f"    iq reduction:     "
              f"{iq_reduction / abs(iq_baseline_mean) * 100:.1f}%")
    print(f"    Position drift:   {pos_drift_mm:.3f} mm")
    print(f"    Magnitude check:  {discrepancy_pct:.1f}% discrepancy")

    # Pass criteria
    position_ok = pos_drift_mm < 1.0
    if magnitude_ok and position_ok:
        print(f"  PASS: Gravity torque validated "
              f"({discrepancy_pct:.1f}% discrepancy, "
              f"position drift {pos_drift_mm:.3f} mm)")
        return True
    else:
        if not magnitude_ok:
            print(f"  FAIL: Torque magnitude discrepancy "
                  f"{discrepancy_pct:.1f}% exceeds 25%")
        if not position_ok:
            print(f"  FAIL: Position drifted {pos_drift_mm:.3f} mm "
                  f"with torque_ff")
        return False


# ===========================================================================
# Main entry point
# ===========================================================================

TESTS = {
    'smoke': ('Torque passthrough smoke test', test_smoke),
    'estop': ('Emergency stop test', test_estop),
    'encoder': ('Encoder sign check', test_encoder_sign),
    'force': ('Force conversion validation', test_force_conversion),
    'pos_smoke': ('Position control smoke test (Stage A1)', test_pos_smoke),
    'vel_ff': ('Velocity feedforward test (Stage A2)', test_vel_ff),
    'torque_ff': ('Gravity torque_ff test (Stage A3)', test_torque_ff),
}

TEST_GROUPS = {
    'phase2': ['smoke', 'estop', 'encoder', 'force'],
    'phase3': ['pos_smoke', 'vel_ff', 'torque_ff'],
    'all': list(TESTS.keys()),
}


def main():
    parser = argparse.ArgumentParser(
        description='Standalone single-leg bench test harness for Jugglebot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  smoke       Torque passthrough smoke test
  estop       Emergency stop test (single leg)
  encoder     Encoder direction and sign convention check
  force       Force conversion validation (requires known weight)
  pos_smoke   Phase 3 Stage A1: Position control smoke test
  vel_ff      Phase 3 Stage A2: Velocity feedforward comparison
  torque_ff   Phase 3 Stage A3: Gravity torque_ff validation (requires weight)

Test groups:
  phase2      Run Phase 2 tests (smoke, estop, encoder, force)
  phase3      Run Phase 3 tests (pos_smoke, vel_ff, torque_ff) [DEFAULT]
  all         Run all tests sequentially

Safety:
  Current limit is set to {curr}A (50% of rated {rated}A).
  All tests send IDLE on completion, error, or Ctrl-C.
        """.format(curr=SAFE_CURRENT_LIMIT_A,
                   rated=hw.ODRIVE_LEG_CURR_LIMIT_A))

    parser.add_argument('--interface', default='socketcan',
                        help='python-can interface (default: socketcan)')
    parser.add_argument('--channel', default='can0',
                        help='CAN channel name (default: can0)')
    parser.add_argument('--axis', type=int, default=0,
                        choices=[0, 1, 2, 3, 4, 5],
                        help='ODrive axis ID to test (default: 0)')
    parser.add_argument('--test', default='phase3',
                        choices=list(TESTS.keys()) + list(TEST_GROUPS.keys()),
                        help='Which test or group to run (default: phase3)')
    parser.add_argument('--home', action='store_true',
                        help='Home the axis before running tests (drive to end-stop, set zero)')

    args = parser.parse_args()

    # Install Ctrl-C handler for clean shutdown
    harness_ref = [None]
    original_handler = signal.getsignal(signal.SIGINT)

    def sigint_handler(sig, frame):
        print("\n\n  *** Ctrl-C: Emergency IDLE ***")
        if harness_ref[0] is not None:
            try:
                harness_ref[0].idle_axis()
            except Exception:
                pass
        signal.signal(signal.SIGINT, original_handler)
        sys.exit(1)

    signal.signal(signal.SIGINT, sigint_handler)

    # Run tests
    harness = SingleLegTestHarness(
        axis_id=args.axis,
        interface=args.interface,
        channel=args.channel)
    harness_ref[0] = harness

    print(f"\nJugglebot Single-Leg Test Harness")
    print(f"  Axis:      {args.axis}")
    print(f"  Interface: {args.interface}")
    print(f"  Channel:   {args.channel}")
    print(f"  Current limit: {SAFE_CURRENT_LIMIT_A} A "
          f"(50% of {hw.ODRIVE_LEG_CURR_LIMIT_A} A)")

    if args.test in TEST_GROUPS:
        tests_to_run = TEST_GROUPS[args.test]
    else:
        tests_to_run = [args.test]

    results = {}
    try:
        with harness:
            if args.home:
                harness.home_axis()
            for test_name in tests_to_run:
                label, func = TESTS[test_name]
                try:
                    result = func(harness)
                    results[test_name] = result
                except Exception as e:
                    print(f"\n  ERROR in {label}: {e}")
                    results[test_name] = False
                    harness.idle_axis()
                    # Continue to next test after error
                    time.sleep(1.0)
    except TimeoutError as e:
        print(f"\nFATAL: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\nFATAL: {e}")
        sys.exit(1)

    # Summary
    print("\n" + "=" * 60)
    print("RESULTS SUMMARY")
    print("=" * 60)
    for test_name in tests_to_run:
        label, _ = TESTS[test_name]
        result = results.get(test_name)
        if result is True:
            status = "PASS"
        elif result is False:
            status = "FAIL"
        elif result is None:
            status = "SKIP"
        else:
            status = "UNKNOWN"
        print(f"  {label:45s} {status}")

    all_pass = all(r is True or r is None for r in results.values())
    if all_pass:
        print("\nAll tests passed (or skipped).")
    else:
        print("\nSome tests FAILED. Review output above.")
        sys.exit(1)


if __name__ == '__main__':
    main()
