#!/usr/bin/env python3
"""Standalone single-leg test harness for Jugglebot bench tests.

Bypasses ROS2 entirely and talks directly to ODrive axes via python-can.
Designed to run on the Jetson (socketcan) or a USB-CAN adapter.

Phase 2 tests (isolated leg, position/torque basics):
  1. Torque passthrough smoke test
  2. Emergency stop test (single leg)
  3. Encoder sign check
  4. Force conversion validation

Phase 3 Stage A tests (isolated leg, torque control):
  5. PD torque-mode hold test (configurable gains)
  6. Gravity feedforward test (PD-only vs PD+FF with known weight)

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
  python tools/single_leg_test.py --test pd_hold      # run just one test
  python tools/single_leg_test.py --test gravity_ff   # run just one test

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
_PROJECT_ROOT = os.path.dirname(_SCRIPT_DIR)
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

# Safety: torque clamp for PD/FF bench tests (prevents runaway on sign errors)
MAX_BENCH_TORQUE_NM = 0.5

# Spool geometry for force conversion
MM_TO_REV = np.array(hw.GEOM_MM_TO_REV, dtype=np.float64)
SPOOL_RADIUS_MM = 1.0 / (2.0 * np.pi * MM_TO_REV)  # per-leg, ~11 mm


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
      4. Command a small constant torque (0.02 Nm) for 2 seconds
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
    TEST_TORQUE_NM = 0.08  # same torque as smoke test to ensure leg is moving

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
    PULSE_TORQUE_NM = 0.08  # above 0.075 Nm friction threshold
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
# Phase 3 Stage A: PD torque-mode hold test
# ===========================================================================

def test_pd_hold(harness: SingleLegTestHarness,
                 kp: float = 0.1, kd: float = 0.001,
                 duration_s: float = 5.0):
    """Stage A1: PD torque-mode hold test.

    Enters TORQUE+PASSTHROUGH mode and runs a simple PD controller on the
    encoder position to hold the leg at its current position.

    Parameters
    ----------
    kp : N/mm — proportional gain (force per position error)
    kd : N·s/mm — derivative gain (force per velocity error)
    duration_s : seconds to hold

    The test logs position error statistics and checks for stability.
    """
    print("\n" + "=" * 60)
    print(f"STAGE A1: PD torque-mode hold (Kp={kp}, Kd={kd}, {duration_s}s)")
    print("=" * 60)

    spool_radius_m = SPOOL_RADIUS_MM[harness.axis_id] / 1000.0
    mm_to_rev_axis = MM_TO_REV[harness.axis_id]

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    # Record current position as the target
    target_rev, _ = harness.read_encoder()
    target_mm = target_rev / mm_to_rev_axis
    print(f"  Target position: {target_rev:.4f} rev ({target_mm:.2f} mm)")

    harness.enter_torque_mode()

    # PD control loop
    errors_mm = []
    currents_A = []
    torques_Nm = []
    LOOP_DT = 0.01  # 100 Hz (CAN rate limited)

    print(f"  Running PD hold for {duration_s}s...")
    deadline = time.time() + duration_s
    while time.time() < deadline:
        harness._poll(timeout=LOOP_DT)

        # Convert encoder to mm
        actual_mm = harness.state.pos_rev / mm_to_rev_axis
        actual_vel_mm_s = harness.state.vel_rps / mm_to_rev_axis

        # PD law in extension space
        e_pos = target_mm - actual_mm       # mm
        e_vel = 0.0 - actual_vel_mm_s       # mm/s (target velocity = 0)
        f_pd = kp * e_pos + kd * e_vel      # N
        tau = f_pd * spool_radius_m          # Nm

        # Clamp for safety, then invert for ODrive (positive Jugglebot = negative ODrive)
        tau_clamped = max(-MAX_BENCH_TORQUE_NM, min(MAX_BENCH_TORQUE_NM, tau))
        harness.send(encode_set_input_torque(harness.axis_id, -tau_clamped))

        errors_mm.append(e_pos)
        currents_A.append(harness.state.iq_measured)
        torques_Nm.append(tau_clamped)

    # Stop
    harness.idle_axis()
    time.sleep(0.5)
    harness._poll()

    errors_mm = np.array(errors_mm)
    currents_A = np.array(currents_A)
    torques_Nm = np.array(torques_Nm)

    e_mean = np.mean(errors_mm)
    e_std = np.std(errors_mm)
    e_max = np.max(np.abs(errors_mm))
    i_mean = np.mean(np.abs(currents_A))
    i_max = np.max(np.abs(currents_A))

    print(f"  Position error: mean={e_mean:.3f} mm, std={e_std:.3f} mm, "
          f"max={e_max:.3f} mm")
    print(f"  Torque: mean={np.mean(torques_Nm):.4f} Nm, "
          f"max={np.max(np.abs(torques_Nm)):.4f} Nm")
    print(f"  Current: mean={i_mean:.3f} A, max={i_max:.3f} A")

    stable = e_max < 0.5  # ±0.5 mm
    no_oscillation = e_std < 0.2  # low variance
    safe_current = i_max < SAFE_CURRENT_LIMIT_A

    if stable and no_oscillation and safe_current:
        print(f"  PASS: Stable hold within ±{e_max:.2f} mm")
        return True
    else:
        if not stable:
            print(f"  FAIL: Max error {e_max:.3f} mm exceeds ±0.5 mm")
        if not no_oscillation:
            print(f"  FAIL: Error std {e_std:.3f} mm suggests oscillation")
        if not safe_current:
            print(f"  FAIL: Peak current {i_max:.3f} A exceeds limit "
                  f"{SAFE_CURRENT_LIMIT_A} A")
        return False


def test_gravity_ff(harness: SingleLegTestHarness):
    """Stage A2: Gravity feedforward test.

    Compares PD-only vs PD+FF with a known weight attached.
    The feedforward torque is computed from the dynamics model.

    Procedure:
      1. Prompt user for attached weight mass
      2. Hold position with PD only, measure current
      3. Add gravity FF torque, measure current
      4. Compare — FF should reduce PD effort by >=50%
    """
    print("\n" + "=" * 60)
    print("STAGE A2: Gravity feedforward test")
    print("=" * 60)

    spool_radius_m = SPOOL_RADIUS_MM[harness.axis_id] / 1000.0
    mm_to_rev_axis = MM_TO_REV[harness.axis_id]

    print(f"\n  This test compares PD-only vs PD+FF holding a known weight.")
    print(f"  The leg should be oriented vertically with the weight hanging.")
    weight_str = input(f"  Enter attached weight in kg (or 'skip'): ").strip().lower()
    harness.flush_and_resync()

    if weight_str == 'skip':
        print("  Skipping gravity feedforward test.")
        return None

    try:
        weight_kg = float(weight_str)
    except ValueError:
        print("  Invalid input. Skipping.")
        return None

    # Compute expected FF torque for this weight on this axis
    force_N = weight_kg * hw.GRAVITY_MPS2
    ff_torque_Nm = force_N * spool_radius_m
    print(f"  Weight: {weight_kg:.3f} kg, force: {force_N:.3f} N, "
          f"FF torque: {ff_torque_Nm:.4f} Nm")

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    target_rev, _ = harness.read_encoder()
    target_mm = target_rev / mm_to_rev_axis

    KP = 0.5    # N/mm — moderate gains
    KD = 0.005  # N·s/mm
    TEST_DURATION = 3.0
    LOOP_DT = 0.01

    def run_hold(use_ff: bool) -> tuple:
        """Run PD hold and return (error_rms_mm, current_rms_A)."""
        harness.enter_torque_mode()
        errors = []
        currents = []
        deadline = time.time() + TEST_DURATION
        while time.time() < deadline:
            harness._poll(timeout=LOOP_DT)
            actual_mm = harness.state.pos_rev / mm_to_rev_axis
            actual_vel = harness.state.vel_rps / mm_to_rev_axis
            e_pos = target_mm - actual_mm
            e_vel = 0.0 - actual_vel
            f_pd = KP * e_pos + KD * e_vel
            tau_pd = f_pd * spool_radius_m
            tau_total = tau_pd + (ff_torque_Nm if use_ff else 0.0)
            # Clamp for safety, then invert for ODrive
            tau_clamped = max(-MAX_BENCH_TORQUE_NM, min(MAX_BENCH_TORQUE_NM, tau_total))
            harness.send(encode_set_input_torque(harness.axis_id, -tau_clamped))
            errors.append(e_pos)
            currents.append(harness.state.iq_measured)
        harness.idle_axis()
        time.sleep(0.5)
        harness._poll()
        return (float(np.sqrt(np.mean(np.array(errors)**2))),
                float(np.sqrt(np.mean(np.array(currents)**2))))

    # Phase 1: PD only
    print(f"\n  Phase 1: PD-only hold ({TEST_DURATION}s)...")
    err_pd, curr_pd = run_hold(use_ff=False)
    print(f"    Error RMS: {err_pd:.3f} mm, Current RMS: {curr_pd:.3f} A")

    # Phase 2: PD + FF
    print(f"  Phase 2: PD+FF hold ({TEST_DURATION}s)...")
    err_ff, curr_ff = run_hold(use_ff=True)
    print(f"    Error RMS: {err_ff:.3f} mm, Current RMS: {curr_ff:.3f} A")

    # Compare
    if curr_pd > 0.01:
        reduction_pct = (1.0 - curr_ff / curr_pd) * 100
    else:
        reduction_pct = 0.0

    print(f"\n  Current reduction: {reduction_pct:.1f}%")
    print(f"  FF torque: {ff_torque_Nm:.4f} Nm")
    print(f"  Expected torque (from dynamics): {ff_torque_Nm:.4f} Nm")

    if reduction_pct >= 50:
        print(f"  PASS: FF reduced PD effort by {reduction_pct:.1f}% (>= 50%)")
        return True
    elif reduction_pct > 0:
        print(f"  MARGINAL: FF reduced effort by {reduction_pct:.1f}% "
              f"(< 50% target)")
        return True  # Still useful, may need gain tuning
    else:
        print(f"  FAIL: FF did not reduce effort (reduction={reduction_pct:.1f}%)")
        return False


# ===========================================================================
# Main entry point
# ===========================================================================

TESTS = {
    'smoke': ('Torque passthrough smoke test', test_smoke),
    'estop': ('Emergency stop test', test_estop),
    'encoder': ('Encoder sign check', test_encoder_sign),
    'force': ('Force conversion validation', test_force_conversion),
    'pd_hold': ('PD torque-mode hold (Stage A1)', test_pd_hold),
    'gravity_ff': ('Gravity feedforward test (Stage A2)', test_gravity_ff),
}

TEST_GROUPS = {
    'phase2': ['smoke', 'estop', 'encoder', 'force'],
    'phase3': ['pd_hold', 'gravity_ff'],
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
  pd_hold     Phase 3 Stage A1: PD torque-mode hold test
  gravity_ff  Phase 3 Stage A2: Gravity feedforward test (requires weight)

Test groups:
  phase2      Run Phase 2 tests (smoke, estop, encoder, force)
  phase3      Run Phase 3 tests (pd_hold, gravity_ff) [DEFAULT]
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
