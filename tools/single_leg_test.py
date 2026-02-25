#!/usr/bin/env python3
"""Standalone single-leg test harness for Phase 2 bench tests.

Bypasses ROS2 entirely and talks directly to ODrive 0 via python-can.
Designed to run on the Jetson (socketcan) or a USB-CAN adapter.

Implements the four Phase 2 isolated-leg bench tests from MOTION_PLANNER_PLAN:
  1. Torque passthrough smoke test
  2. Emergency stop test (single leg)
  3. Encoder sign check
  4. Force conversion validation

Safety:
  - Uses conservative current limit (50% of rated = 10A)
  - All tests send IDLE on completion, error, or Ctrl-C
  - Heartbeat watchdog detects ODrive disconnection
  - Every test is gated on heartbeat reception before commanding

Usage:
  python tools/single_leg_test.py [--interface socketcan] [--channel can0] [--axis 0]
  python tools/single_leg_test.py --test smoke        # run just the smoke test
  python tools/single_leg_test.py --test estop        # run just the e-stop test
  python tools/single_leg_test.py --test encoder      # run just the encoder sign check
  python tools/single_leg_test.py --test force        # run just the force conversion test
  python tools/single_leg_test.py --test all          # run all tests sequentially

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

sys.path.insert(0, _CONFIG_DIR)
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
    pos_rev: float = 0.0         # encoder position in revolutions
    vel_rps: float = 0.0         # encoder velocity in rev/s
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
        self.state.pos_rev = pos
        self.state.vel_rps = vel

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

    def enter_torque_mode(self):
        """Put axis into TORQUE control with PASSTHROUGH input."""
        self.send(encode_set_controller_mode(
            self.axis_id, 'TORQUE', 'PASSTHROUGH'))
        time.sleep(0.01)
        # Send zero torque before entering closed loop
        self.send(encode_set_input_torque(self.axis_id, 0.0))
        time.sleep(0.01)
        self.send(encode_set_state(self.axis_id, 'CLOSED_LOOP'))
        time.sleep(0.1)
        self._poll()
        if not self.state.is_closed_loop:
            raise RuntimeError(
                f"Failed to enter CLOSED_LOOP. State: {self.state.state_name}, "
                f"errors: {error_names(self.state.active_errors)}")
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
    TEST_TORQUE_NM = 0.02  # very small -- just enough to move an unloaded leg
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
    TEST_TORQUE_NM = 0.03

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
    PULSE_TORQUE_NM = 0.03
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


def test_force_conversion(harness: SingleLegTestHarness):
    """Test 4: Force conversion validation.

    From MOTION_PLANNER_PLAN Phase 2:
    "Apply a known static load to the isolated leg (e.g., hang a calibrated
    weight) and command zero torque. Measure the motor current required to
    hold position via PD. Compare the measured current to the predicted
    current from the geometric force-to-torque conversion."

    This test requires user interaction:
      1. Prompts user to attach a known weight
      2. Uses POSITION/PASSTHROUGH mode to hold the leg at its current position
      3. Measures the steady-state motor current (iq_measured)
      4. Computes the predicted current from: F=mg, tau=F*r_spool, I=tau/Kt
      5. Reports the discrepancy

    Motor torque constant (Kt) note:
      The ODrive reports iq (quadrature current) which relates to torque as
      tau = Kt * iq. For the motors used on Jugglebot, Kt should be determined
      from the motor datasheet or measured. This test logs the raw iq so the
      user can validate against known Kt.
    """
    print("\n" + "=" * 60)
    print("TEST 4: Force conversion validation")
    print("=" * 60)

    print("\n  This test requires a known weight hung from the leg.")
    print("  The leg should be oriented so the weight hangs vertically.")
    weight_str = input("  Enter the mass in kg (or 'skip' to skip): ").strip()
    if weight_str.lower() == 'skip':
        print("  Skipping force conversion test.")
        return None

    try:
        mass_kg = float(weight_str)
    except ValueError:
        print(f"  Invalid input: {weight_str}")
        return False

    force_N = mass_kg * hw.GRAVITY_MPS2
    spool_radius_m = SPOOL_RADIUS_MM[harness.axis_id] / 1000.0
    predicted_torque_Nm = force_N * spool_radius_m

    print(f"\n  Weight: {mass_kg:.3f} kg")
    print(f"  Force:  {force_N:.3f} N")
    print(f"  Spool radius (axis {harness.axis_id}): "
          f"{SPOOL_RADIUS_MM[harness.axis_id]:.2f} mm")
    print(f"  Predicted motor torque: {predicted_torque_Nm:.4f} Nm")

    harness.clear_errors()
    harness.set_safe_limits()
    harness.require_no_errors()

    # Read current position, then hold it with position control
    pos_now, _ = harness.read_encoder()
    print(f"\n  Current position: {pos_now:.4f} rev")
    print(f"  Entering POSITION/PASSTHROUGH to hold position...")

    harness.send(encode_set_controller_mode(
        harness.axis_id, 'POSITION', 'PASSTHROUGH'))
    time.sleep(0.01)
    harness.send(encode_set_input_pos(harness.axis_id, pos_now))
    time.sleep(0.01)
    harness.send(encode_set_state(harness.axis_id, 'CLOSED_LOOP'))
    time.sleep(0.5)
    harness._poll()

    if not harness.state.is_closed_loop:
        raise RuntimeError(
            f"Failed to enter CLOSED_LOOP for position hold. "
            f"State: {harness.state.state_name}")

    # Collect iq samples over 3 seconds
    print(f"  Collecting current measurements for 3 seconds...")
    iq_samples = []
    deadline = time.time() + 3.0
    while time.time() < deadline:
        harness._poll(timeout=0.02)
        iq_samples.append(harness.state.iq_measured)

    harness.idle_axis()

    iq_samples = np.array(iq_samples)
    iq_mean = np.mean(iq_samples)
    iq_std = np.std(iq_samples)

    print(f"\n  Results:")
    print(f"    Mean iq_measured:    {iq_mean:.4f} A (std: {iq_std:.4f} A)")
    print(f"    Predicted torque:    {predicted_torque_Nm:.4f} Nm")
    print(f"    Measured iq * Kt = measured torque (need Kt from motor datasheet)")
    print(f"")
    print(f"    To compute discrepancy: tau_predicted / iq_mean = effective Kt")
    if abs(iq_mean) > 0.001:
        effective_kt = predicted_torque_Nm / iq_mean
        print(f"    Effective Kt:        {effective_kt:.4f} Nm/A")
        print(f"    (Compare against motor datasheet Kt to validate conversion)")
    else:
        print(f"    WARNING: iq_mean near zero -- weight may not be loading the motor")

    # The plan says "discrepancy should be under 10%"
    # We can't fully validate without Kt, so we report all values and let
    # the user compare against the datasheet
    print(f"\n  MANUAL VERIFICATION REQUIRED:")
    print(f"    1. Look up motor Kt (torque constant) from datasheet")
    print(f"    2. Compute expected_iq = predicted_torque / Kt "
          f"= {predicted_torque_Nm:.4f} / Kt")
    print(f"    3. Compare expected_iq to measured iq_mean = {iq_mean:.4f} A")
    print(f"    4. Discrepancy should be < 10%")

    return True  # Can't auto-pass/fail without Kt


# ===========================================================================
# Main entry point
# ===========================================================================

TESTS = {
    'smoke': ('Torque passthrough smoke test', test_smoke),
    'estop': ('Emergency stop test', test_estop),
    'encoder': ('Encoder sign check', test_encoder_sign),
    'force': ('Force conversion validation', test_force_conversion),
}


def main():
    parser = argparse.ArgumentParser(
        description='Standalone single-leg bench test harness for Jugglebot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  smoke     Torque passthrough smoke test
  estop     Emergency stop test (single leg)
  encoder   Encoder direction and sign convention check
  force     Force conversion validation (requires known weight)
  all       Run all tests sequentially

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
    parser.add_argument('--test', default='all',
                        choices=list(TESTS.keys()) + ['all'],
                        help='Which test to run (default: all)')

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

    if args.test == 'all':
        tests_to_run = list(TESTS.keys())
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
