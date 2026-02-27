#!/usr/bin/env python3
"""Standalone supported-platform test harness for Jugglebot Phase 3 Stage B.

Bypasses ROS2 entirely and talks directly to all 6 ODrive leg axes via
python-can.  Designed to run on the Jetson (socketcan) or a USB-CAN adapter.

Phase 3 Stage B tests (supported platform — 6 legs simultaneously):
  B1. Six-leg CAN coordination (exit gate)
  B2. Direction and sign convention check (all legs)

Prerequisites:
  - Robot fully assembled with all 6 legs on the CAN bus
  - Platform mechanically supported (blocks / clamps / sling) so legs are
    NOT load-bearing.  If all legs go IDLE, nothing falls.
  - Legs should be homed before running tests (use --home flag)

Safety:
  - Uses conservative current limit (50% of rated = 10A)
  - All tests send IDLE to all 6 axes on completion, error, or Ctrl-C
  - Heartbeat watchdog detects ODrive disconnection on any axis
  - B1 is an exit gate: if it fails, B2 is skipped

Usage:
  python tools/supported_platform_test.py --home --test all
  python tools/supported_platform_test.py --test can_coord   # B1 only
  python tools/supported_platform_test.py --test direction    # B2 only
  python tools/supported_platform_test.py --home              # Home all, then default tests

Requirements:
  pip install python-can numpy
"""

from __future__ import annotations

import argparse
import math
import signal
import struct
import sys
import time
from dataclasses import dataclass
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
# Protocol lookups (local references for readability)
# ---------------------------------------------------------------------------
COMMANDS = proto.ODRIVE_COMMANDS
AXIS_STATES = proto.ODRIVE_STATES
CONTROL_MODES = proto.ODRIVE_CONTROL_MODES
INPUT_MODES = proto.ODRIVE_INPUT_MODES

LEG_AXES = list(range(6))  # [0, 1, 2, 3, 4, 5]
NUM_LEGS = 6

# Safety: 50% of rated leg current
SAFE_CURRENT_LIMIT_A = hw.ODRIVE_LEG_CURR_LIMIT_A * 0.5  # 10A

# Per-leg spool geometry from hardware_config (standard Jugglebot legs)
MM_TO_REV = np.array(hw.GEOM_MM_TO_REV, dtype=np.float64)  # 6 values

# ODrive leg feedforward int16 scaling
LEG_VEL_FF_SCALE = proto.INPUT_SCALE_LEG_VEL   # 1000
LEG_TOR_FF_SCALE = proto.INPUT_SCALE_LEG_TOR   # 1000


# ---------------------------------------------------------------------------
# CAN encoding helpers (standalone — same as single_leg_test.py)
# ---------------------------------------------------------------------------

def arb_id(axis_id: int, command_name: str) -> int:
    """Compute CAN arbitration ID: (node_id << 5) | command_id."""
    return (axis_id << 5) | COMMANDS[command_name]


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


def encode_set_vel_curr_limits(axis_id: int, vel_limit: float,
                               curr_limit: float) -> can.Message:
    data = struct.pack('<ff', vel_limit, curr_limit)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_vel_curr_limits'),
                       data=data, dlc=8, is_extended_id=False)


def encode_clear_errors(axis_id: int) -> can.Message:
    return can.Message(arbitration_id=arb_id(axis_id, 'clear_errors'),
                       data=b'\x00' * 8, dlc=8, is_extended_id=False)


def encode_set_absolute_position(axis_id: int, position: float) -> can.Message:
    data = struct.pack('<f', position) + b'\x00' * 4
    return can.Message(arbitration_id=arb_id(axis_id, 'set_absolute_position'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_pos_gain(axis_id: int, pos_gain: float) -> can.Message:
    data = struct.pack('<f', pos_gain) + b'\x00' * 4
    return can.Message(arbitration_id=arb_id(axis_id, 'set_pos_gain'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_vel_gains(axis_id: int, vel_gain: float,
                         vel_int_gain: float) -> can.Message:
    data = struct.pack('<ff', vel_gain, vel_int_gain)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_vel_gains'),
                       data=data, dlc=8, is_extended_id=False)


# ---------------------------------------------------------------------------
# CAN decoding helpers
# ---------------------------------------------------------------------------

def decode_heartbeat(data: bytes):
    state = data[4]
    result = data[5]
    traj_done = bool(data[6] & 0x01)
    return state, result, traj_done


def decode_error(data: bytes):
    active_errors = struct.unpack_from('<I', data, 0)[0]
    disarm_reason = struct.unpack_from('<I', data, 4)[0]
    return active_errors, disarm_reason


def decode_encoder_estimate(data: bytes):
    pos, vel = struct.unpack('<ff', data)
    return pos, vel


def decode_iq(data: bytes):
    setpoint, measured = struct.unpack('<ff', data)
    return setpoint, measured


ERROR_CODES = {
    0x01: 'INITIALIZING',
    0x02: 'SYSTEM_LEVEL',
    0x04: 'TIMING_ERROR',
    0x08: 'MISSING_ESTIMATE',
    0x10: 'BAD_CONFIG',
    0x20: 'DRV_FAULT',
    0x40: 'MISSING_INPUT',
    0x100: 'DC_BUS_OVER_VOLTAGE',
    0x200: 'DC_BUS_UNDER_VOLTAGE',
    0x400: 'DC_BUS_OVER_CURRENT',
    0x800: 'DC_BUS_OVER_REGEN_CURRENT',
    0x1000: 'CURRENT_LIMIT_VIOLATION',
    0x2000: 'MOTOR_OVER_TEMP',
    0x4000: 'INVERTER_OVER_TEMP',
    0x8000: 'VELOCITY_LIMIT_VIOLATION',
    0x10000: 'POSITION_LIMIT_VIOLATION',
    0x1000000: 'ESTOP_REQUESTED',
}


def error_names(bitmask: int) -> list:
    return [name for bit, name in ERROR_CODES.items() if bitmask & bit]


# ---------------------------------------------------------------------------
# AxisState — per-axis state tracking
# ---------------------------------------------------------------------------

@dataclass
class AxisState:
    axis_state: int = 0
    procedure_result: int = 0
    trajectory_done: bool = False
    active_errors: int = 0
    disarm_reason: int = 0
    pos_rev: float = 0.0         # Inverted encoder (positive = extension)
    pos_rev_raw: float = 0.0     # Raw ODrive encoder
    vel_rps: float = 0.0         # Inverted velocity
    vel_rps_raw: float = 0.0     # Raw ODrive velocity
    iq_setpoint: float = 0.0
    iq_measured: float = 0.0
    last_heartbeat: float = 0.0
    heartbeat_count: int = 0
    last_encoder_time: float = 0.0  # Timestamp of last encoder message

    @property
    def has_errors(self) -> bool:
        return self.active_errors != 0

    @property
    def state_name(self) -> str:
        for name, val in AXIS_STATES.items():
            if val == self.axis_state:
                return name
        return f'UNKNOWN({self.axis_state})'

    @property
    def is_idle(self) -> bool:
        return self.axis_state == AXIS_STATES['IDLE']

    @property
    def is_closed_loop(self) -> bool:
        return self.axis_state == AXIS_STATES['CLOSED_LOOP']


# ---------------------------------------------------------------------------
# PlatformTestHarness — multi-axis CAN interface for 6-leg platform tests
# ---------------------------------------------------------------------------

class PlatformTestHarness:
    """Standalone CAN interface for 6-leg platform testing.

    Manages all 6 leg axes (0-5) simultaneously.  No ROS2 dependency.
    """

    CAN_SEND_DELAY_S = 0.002  # 2ms between consecutive sends
    HEARTBEAT_TIMEOUT_S = 2.0  # seconds without heartbeat = fault

    def __init__(self, interface: str = 'socketcan', channel: str = 'can0'):
        self._interface = interface
        self._channel = channel
        self._bus: Optional[can.Bus] = None
        self._shutdown_requested = False

        # Per-axis state
        self.states: dict[int, AxisState] = {
            axis_id: AxisState() for axis_id in LEG_AXES
        }

        # Handler dispatch table (same handlers for all axes)
        self._handler_cmd_ids = {
            COMMANDS['heartbeat_message']: self._handle_heartbeat,
            COMMANDS['get_error']: self._handle_error,
            COMMANDS['get_encoder_estimate']: self._handle_encoder,
            COMMANDS['get_iq']: self._handle_iq,
        }

    # -- Lifecycle ----------------------------------------------------------

    def connect(self):
        """Open the CAN bus and wait for heartbeats from all 6 axes."""
        print(f"Connecting to CAN bus: {self._channel} ({self._interface})")
        self._bus = can.Bus(channel=self._channel, interface=self._interface,
                            bitrate=proto.CAN_BAUD_RATE)
        # Flush stale messages
        while self._bus.recv(timeout=0):
            pass

        print(f"Waiting for heartbeats from all {NUM_LEGS} leg axes...")
        deadline = time.time() + 10.0
        while time.time() < deadline:
            self._poll(timeout=0.1)
            received = [
                axis_id for axis_id in LEG_AXES
                if self.states[axis_id].heartbeat_count > 0
            ]
            if len(received) == NUM_LEGS:
                print(f"  All {NUM_LEGS} axes reporting:")
                for axis_id in LEG_AXES:
                    s = self.states[axis_id]
                    print(f"    Axis {axis_id}: {s.state_name}, "
                          f"errors: {s.active_errors}")
                return
            # Progress indicator
            missing = [a for a in LEG_AXES if a not in received]
            if len(received) > 0 and time.time() % 2 < 0.1:
                print(f"  Received: {received}, waiting for: {missing}")

        missing = [a for a in LEG_AXES
                   if self.states[a].heartbeat_count == 0]
        raise TimeoutError(
            f"No heartbeat from axes {missing} after 10 seconds. "
            f"Are all ODrives powered on and connected?")

    def disconnect(self):
        """Safely idle all axes and close the CAN bus."""
        if self._bus is not None:
            try:
                self.idle_all()
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

    # -- CAN I/O ------------------------------------------------------------

    def send(self, msg: can.Message):
        """Send a CAN message with inter-message pacing."""
        self._bus.send(msg)
        time.sleep(self.CAN_SEND_DELAY_S)

    def send_no_delay(self, msg: can.Message):
        """Send a CAN message without pacing delay (for tight timing loops)."""
        self._bus.send(msg)

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

        if axis_id not in self.states:
            return  # ignore non-leg axes

        handler = self._handler_cmd_ids.get(cmd_id)
        if handler:
            handler(axis_id, msg.data)

    def _handle_heartbeat(self, axis_id: int, data: bytes):
        state, result, traj_done = decode_heartbeat(data)
        s = self.states[axis_id]
        s.axis_state = state
        s.procedure_result = result
        s.trajectory_done = traj_done
        s.last_heartbeat = time.time()
        s.heartbeat_count += 1

    def _handle_error(self, axis_id: int, data: bytes):
        errors, disarm = decode_error(data)
        s = self.states[axis_id]
        s.active_errors = errors
        s.disarm_reason = disarm

    def _handle_encoder(self, axis_id: int, data: bytes):
        pos, vel = decode_encoder_estimate(data)
        s = self.states[axis_id]
        s.pos_rev_raw = pos
        s.vel_rps_raw = vel
        # Leg inversion (matches can_node.py): positive = extension
        s.pos_rev = -pos
        s.vel_rps = -vel
        s.last_encoder_time = time.time()

    def _handle_iq(self, axis_id: int, data: bytes):
        setpoint, measured = decode_iq(data)
        s = self.states[axis_id]
        s.iq_setpoint = setpoint
        s.iq_measured = measured

    # -- Multi-axis control -------------------------------------------------

    def idle_all(self):
        """Send all 6 axes to IDLE state."""
        for axis_id in LEG_AXES:
            self.send(encode_set_state(axis_id, 'IDLE'))
        print(f"  All {NUM_LEGS} axes -> IDLE")

    def clear_all_errors(self):
        """Clear errors on all 6 axes."""
        for axis_id in LEG_AXES:
            self.send(encode_clear_errors(axis_id))
        time.sleep(0.1)
        self._poll()
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            if s.has_errors:
                print(f"  WARNING: Axis {axis_id} still has errors: "
                      f"{error_names(s.active_errors)}")
        print(f"  Errors cleared on all axes.")

    def set_safe_limits_all(self):
        """Apply conservative velocity and current limits to all 6 axes."""
        for axis_id in LEG_AXES:
            self.send(encode_set_vel_curr_limits(
                axis_id,
                vel_limit=hw.ODRIVE_LEG_VEL_LIMIT_RPS,
                curr_limit=SAFE_CURRENT_LIMIT_A))
        print(f"  Limits set on all axes: vel={hw.ODRIVE_LEG_VEL_LIMIT_RPS} rev/s, "
              f"curr={SAFE_CURRENT_LIMIT_A} A")

    def require_no_errors_all(self):
        """Assert no axis has active errors."""
        self._poll()
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            if s.has_errors:
                names = error_names(s.active_errors)
                raise RuntimeError(
                    f"Axis {axis_id} has errors: {names}")

    def all_heartbeats_fresh(self) -> bool:
        """Check that all 6 axes have recent heartbeats."""
        now = time.time()
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            if s.heartbeat_count == 0:
                return False
            if now - s.last_heartbeat >= self.HEARTBEAT_TIMEOUT_S:
                return False
        return True

    def poll_for(self, duration_s: float, interval_s: float = 0.01):
        """Poll CAN messages for a fixed duration, checking heartbeat health."""
        deadline = time.time() + duration_s
        while time.time() < deadline:
            self._poll(timeout=interval_s)
            if not self.all_heartbeats_fresh():
                stale = [a for a in LEG_AXES
                         if time.time() - self.states[a].last_heartbeat
                         >= self.HEARTBEAT_TIMEOUT_S]
                raise RuntimeError(
                    f"Heartbeat timeout on axes {stale} -- "
                    f"ODrive(s) may be disconnected!")

    def flush_and_resync(self):
        """Drain stale CAN buffer and wait for fresh heartbeats from all axes."""
        while self._bus.recv(timeout=0):
            pass
        deadline = time.time() + 2.0
        hb_before = {a: self.states[a].heartbeat_count for a in LEG_AXES}
        while time.time() < deadline:
            self._poll(timeout=0.05)
            all_fresh = all(
                self.states[a].heartbeat_count > hb_before[a]
                for a in LEG_AXES
            )
            if all_fresh:
                return
        stale = [a for a in LEG_AXES
                 if self.states[a].heartbeat_count <= hb_before[a]]
        raise RuntimeError(
            f"No fresh heartbeat from axes {stale} after flush.")

    def _wait_for_closed_loop(self, axis_id: int, timeout_s: float = 2.0):
        """Poll CAN until a specific axis reports CLOSED_LOOP."""
        retried = False
        deadline = time.time() + timeout_s
        retry_at = time.time() + 0.5
        while time.time() < deadline:
            self._poll(timeout=0.02)
            if self.states[axis_id].is_closed_loop:
                return
            if (not retried and time.time() >= retry_at
                    and self.states[axis_id].is_idle):
                self.send(encode_set_state(axis_id, 'CLOSED_LOOP'))
                retried = True
        s = self.states[axis_id]
        raise RuntimeError(
            f"Axis {axis_id} failed to enter CLOSED_LOOP after {timeout_s}s. "
            f"State: {s.state_name}, errors: {error_names(s.active_errors)}")

    def enter_position_mode_all(self):
        """Put all 6 axes into POSITION/PASSTHROUGH, holding current positions."""
        self._poll()  # refresh encoder readings

        for axis_id in LEG_AXES:
            pos_now_raw = self.states[axis_id].pos_rev_raw
            self.send(encode_set_controller_mode(
                axis_id, 'POSITION', 'PASSTHROUGH'))
            time.sleep(0.05)
            self.send(encode_set_input_pos(axis_id, pos_now_raw))
            time.sleep(0.05)
            self.send(encode_set_state(axis_id, 'CLOSED_LOOP'))

        # Wait for all to enter CLOSED_LOOP
        for axis_id in LEG_AXES:
            self._wait_for_closed_loop(axis_id)

        print(f"  All {NUM_LEGS} axes in POSITION/PASSTHROUGH, CLOSED_LOOP:")
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            print(f"    Axis {axis_id}: holding at {s.pos_rev_raw:.4f} rev raw "
                  f"({s.pos_rev / MM_TO_REV[axis_id]:.2f} mm)")

    def home_axis(self, axis_id: int):
        """Home a single axis by driving to end-stop, then set absolute position.

        Same sequence as single_leg_test.py, adapted for multi-axis dispatch.
        """
        print(f"  Homing axis {axis_id}...")

        # Clear errors + set safe limits on this axis
        self.send(encode_clear_errors(axis_id))
        time.sleep(0.1)
        self.send(encode_set_vel_curr_limits(
            axis_id,
            vel_limit=hw.ODRIVE_LEG_VEL_LIMIT_RPS,
            curr_limit=SAFE_CURRENT_LIMIT_A))

        # Enter velocity mode
        self.send(encode_set_controller_mode(
            axis_id, 'VELOCITY', 'VEL_RAMP'))
        time.sleep(0.05)
        self.send(encode_set_vel_curr_limits(
            axis_id,
            vel_limit=abs(hw.HOMING_LEG_SPEED_RPS * 2),
            curr_limit=hw.HOMING_LEG_CURRENT_LIMIT_A
            + hw.HOMING_LEG_CURRENT_HEADROOM_A))
        time.sleep(0.05)
        self.send(encode_set_state(axis_id, 'CLOSED_LOOP'))
        self._wait_for_closed_loop(axis_id)

        # Drive toward end-stop (positive raw = retraction)
        self.send(encode_set_input_vel(axis_id, hw.HOMING_LEG_SPEED_RPS))

        # Grace period for velocity ramp-up
        print(f"    Ramping up (1s grace period)...")
        self.poll_for(1.0)

        # Monitor current with EMA
        avg = 0.0
        deadline = time.time() + hw.HOMING_MOTOR_TIMEOUT_S
        while time.time() < deadline:
            self._poll(timeout=0.01)
            iq = self.states[axis_id].iq_measured
            avg = avg * hw.HOMING_EMA_WEIGHT + iq * (1.0 - hw.HOMING_EMA_WEIGHT)
            if abs(avg) >= hw.HOMING_LEG_CURRENT_LIMIT_A:
                print(f"    End-stop detected (avg current: {avg:.2f} A)")
                break
        else:
            self.send(encode_set_state(axis_id, 'IDLE'))
            raise RuntimeError(
                f"Homing timeout on axis {axis_id} after "
                f"{hw.HOMING_MOTOR_TIMEOUT_S}s")

        self.send(encode_set_state(axis_id, 'IDLE'))
        time.sleep(0.2)
        self._poll()

        # Set absolute position to 0 (fully compressed = origin)
        self.send(encode_set_absolute_position(axis_id, 0.0))
        time.sleep(0.1)
        self._poll()

        print(f"    Homed! Encoder set to 0 rev (fully compressed)")
        print(f"    Current position: raw={self.states[axis_id].pos_rev_raw:.4f}, "
              f"inverted={self.states[axis_id].pos_rev:.4f} rev")

    def home_all(self):
        """Home all 6 axes sequentially (one at a time for safety)."""
        print(f"\n  Homing all {NUM_LEGS} axes sequentially...")
        for axis_id in LEG_AXES:
            self.home_axis(axis_id)
        print(f"  All {NUM_LEGS} axes homed successfully.")


# ===========================================================================
# Test B1: Six-leg CAN coordination
# ===========================================================================

def test_can_coordination(harness: PlatformTestHarness):
    """Stage B1: Six-leg CAN coordination test (EXIT GATE).

    From MOTION_PLANNER_PLAN Phase 3 Stage B:
    "Command time-varying position profiles to all six legs simultaneously
    at the full update rate. Verify CAN bus throughput is sufficient (no
    dropped frames), all legs receive commands within the same control cycle,
    and encoder feedback from all six legs is received within one cycle."

    Procedure:
      1. Enter POSITION/PASSTHROUGH on all 6 legs
      2. Run 1000 command cycles at 200 Hz (5 seconds)
      3. Each cycle: cosine position profile (10 mm amplitude, 1 Hz)
         Cosine starts at trough (home position), only extends.
      4. Track timing and encoder response completeness
      5. Report statistics

    Pass criteria:
      - Zero cycles with dropped encoder responses
      - p99 cycle time < target period (5 ms)
      - No axis errors
    """
    print("\n" + "=" * 60)
    print("STAGE B1: Six-leg CAN coordination test (EXIT GATE)")
    print("=" * 60)

    TARGET_RATE_HZ = 200
    TARGET_PERIOD_S = 1.0 / TARGET_RATE_HZ  # 5 ms
    NUM_CYCLES = 1000                        # 5 seconds at 200 Hz
    AMPLITUDE_MM = 10.0                      # Per-leg oscillation amplitude
    FREQ_HZ = 1.0                            # Cosine frequency
    # Max time to wait for encoder responses within a cycle (ms)
    ENCODER_POLL_TIMEOUT_S = 0.003           # 3 ms polling window

    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.require_no_errors_all()
    harness.enter_position_mode_all()

    # Record starting positions (raw) for each axis
    start_raw = {}
    for axis_id in LEG_AXES:
        start_raw[axis_id] = harness.states[axis_id].pos_rev_raw

    # Pre-compute per-leg amplitude in revolutions (raw convention: negative = extension)
    amp_rev = {}
    for axis_id in LEG_AXES:
        amp_rev[axis_id] = AMPLITUDE_MM * MM_TO_REV[axis_id]

    # Timing and completeness tracking
    cycle_times = []
    send_durations = []
    dropped_cycles = []  # list of (cycle_idx, missing_axes)
    encoder_counts = {axis_id: 0 for axis_id in LEG_AXES}

    print(f"\n  Running {NUM_CYCLES} cycles at {TARGET_RATE_HZ} Hz "
          f"({NUM_CYCLES / TARGET_RATE_HZ:.1f}s)...")
    print(f"  Cosine profile: {AMPLITUDE_MM} mm amplitude, {FREQ_HZ} Hz")
    print(f"  Target cycle period: {TARGET_PERIOD_S * 1000:.1f} ms")

    t_start = time.time()

    for cycle in range(NUM_CYCLES):
        t_cycle_start = time.time()
        t_elapsed = t_cycle_start - t_start

        # Compute cosine position for this cycle
        # pos_mm = amplitude * (1 - cos(2π·f·t)) / 2  →  range [0, amplitude]
        # At t=0: pos_mm = 0 (most compressed = home)
        phase = 2.0 * math.pi * FREQ_HZ * t_elapsed
        pos_frac = (1.0 - math.cos(phase)) / 2.0  # 0..1

        # Record which axes had encoder data before this cycle
        enc_before = {
            axis_id: harness.states[axis_id].last_encoder_time
            for axis_id in LEG_AXES
        }

        # Send position commands to all 6 axes (minimal delay)
        t_send_start = time.time()
        for axis_id in LEG_AXES:
            # Extension = positive mm = negative raw rev
            target_raw = start_raw[axis_id] - pos_frac * amp_rev[axis_id]
            harness.send_no_delay(
                encode_set_input_pos(axis_id, target_raw))
        t_send_end = time.time()
        send_durations.append(t_send_end - t_send_start)

        # Poll for encoder responses
        poll_deadline = t_send_end + ENCODER_POLL_TIMEOUT_S
        while time.time() < poll_deadline:
            harness._poll(timeout=0.0005)
            # Check if all axes have fresh encoder data
            all_fresh = all(
                harness.states[a].last_encoder_time > enc_before[a]
                for a in LEG_AXES
            )
            if all_fresh:
                break

        # Record which axes responded
        missing = []
        for axis_id in LEG_AXES:
            if harness.states[axis_id].last_encoder_time > enc_before[axis_id]:
                encoder_counts[axis_id] += 1
            else:
                missing.append(axis_id)
        if missing:
            dropped_cycles.append((cycle, missing))

        # Check for errors
        for axis_id in LEG_AXES:
            if harness.states[axis_id].has_errors:
                harness.idle_all()
                names = error_names(harness.states[axis_id].active_errors)
                raise RuntimeError(
                    f"Axis {axis_id} error during B1 at cycle {cycle}: {names}")

        t_cycle_end = time.time()
        cycle_times.append(t_cycle_end - t_cycle_start)

        # Sleep for remainder of cycle period
        remaining = TARGET_PERIOD_S - (t_cycle_end - t_cycle_start)
        if remaining > 0:
            time.sleep(remaining)

        # Progress indicator every 200 cycles
        if (cycle + 1) % 200 == 0:
            print(f"    Cycle {cycle + 1}/{NUM_CYCLES} "
                  f"(dropped so far: {len(dropped_cycles)})")

    # Return to start positions
    for axis_id in LEG_AXES:
        harness.send(encode_set_input_pos(axis_id, start_raw[axis_id]))
    harness.poll_for(1.0)
    harness.idle_all()

    # -- Statistics ---------------------------------------------------------
    cycle_arr = np.array(cycle_times) * 1000  # ms
    send_arr = np.array(send_durations) * 1000  # ms

    print(f"\n  Results:")
    print(f"    Total cycles:        {NUM_CYCLES}")
    print(f"    Dropped cycles:      {len(dropped_cycles)}")
    print(f"    Cycle time (ms):     mean={cycle_arr.mean():.2f}, "
          f"p95={np.percentile(cycle_arr, 95):.2f}, "
          f"p99={np.percentile(cycle_arr, 99):.2f}, "
          f"max={cycle_arr.max():.2f}")
    print(f"    Send burst (ms):     mean={send_arr.mean():.3f}, "
          f"max={send_arr.max():.3f}")
    print(f"    Encoder responses per axis:")
    for axis_id in LEG_AXES:
        pct = 100.0 * encoder_counts[axis_id] / NUM_CYCLES
        print(f"      Axis {axis_id}: {encoder_counts[axis_id]}/{NUM_CYCLES} "
              f"({pct:.1f}%)")

    if dropped_cycles:
        # Show first few dropped cycles
        print(f"\n    First dropped cycles (up to 10):")
        for cycle_idx, missing_axes in dropped_cycles[:10]:
            print(f"      Cycle {cycle_idx}: missing axes {missing_axes}")

    # -- Pass/Fail ----------------------------------------------------------
    p99_ok = np.percentile(cycle_arr, 99) < TARGET_PERIOD_S * 1000
    no_drops = len(dropped_cycles) == 0

    if no_drops and p99_ok:
        print(f"\n  PASS: Zero dropped frames, "
              f"p99 cycle time {np.percentile(cycle_arr, 99):.2f} ms "
              f"< {TARGET_PERIOD_S * 1000:.1f} ms target")
        return True
    else:
        if not no_drops:
            print(f"\n  FAIL: {len(dropped_cycles)} cycles with dropped "
                  f"encoder responses")
        if not p99_ok:
            print(f"\n  FAIL: p99 cycle time {np.percentile(cycle_arr, 99):.2f} ms "
                  f">= {TARGET_PERIOD_S * 1000:.1f} ms target")
        return False


# ===========================================================================
# Test B2: Direction and sign convention check
# ===========================================================================

def test_direction(harness: PlatformTestHarness):
    """Stage B2: Direction and sign convention check (all legs).

    From MOTION_PLANNER_PLAN Phase 3 Stage B:
    "Command a small position increment on each leg in sequence. Verify each
    leg moves in the direction that would shorten/extend it as expected by
    the kinematic model."

    Procedure:
      1. Enter POSITION/PASSTHROUGH on all 6 legs (hold current position)
      2. For each leg, command +5 mm extension, verify direction and magnitude
      3. Command back to start, verify return accuracy

    Pass criteria (per leg):
      - Direction: encoder moved positive (extension)
      - Magnitude: within 2 mm of commanded 5 mm
      - Return: within 1 mm of start

    Overall pass: all 6 legs pass.
    """
    print("\n" + "=" * 60)
    print("STAGE B2: Direction and sign convention check (all legs)")
    print("=" * 60)

    STEP_MM = 5.0          # Extension step size
    SETTLE_TIME_S = 2.0    # Time to wait for settling
    DIR_TOLERANCE_MM = 2.0 # Maximum error for direction/magnitude check
    RETURN_TOLERANCE_MM = 1.0  # Maximum error for return check

    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.require_no_errors_all()
    harness.enter_position_mode_all()

    all_pass = True
    leg_results = {}

    for axis_id in LEG_AXES:
        print(f"\n  --- Leg {axis_id} ---")
        mm_to_rev = MM_TO_REV[axis_id]

        # Record starting position
        harness._poll()
        start_pos_rev = harness.states[axis_id].pos_rev  # inverted
        start_raw = harness.states[axis_id].pos_rev_raw
        start_mm = start_pos_rev / mm_to_rev
        print(f"    Start: {start_raw:.4f} rev raw "
              f"({start_mm:.2f} mm extension)")

        # Command +5 mm extension
        # Extension = positive inverted = negative raw
        target_mm = start_mm + STEP_MM
        target_raw = start_raw - STEP_MM * mm_to_rev
        print(f"    Commanding +{STEP_MM} mm -> target {target_raw:.4f} rev raw")

        harness.send(encode_set_input_pos(axis_id, target_raw))
        harness.poll_for(SETTLE_TIME_S)

        # Read actual position
        actual_pos_rev = harness.states[axis_id].pos_rev
        actual_mm = actual_pos_rev / mm_to_rev
        displacement_mm = actual_mm - start_mm
        error_mm = abs(displacement_mm - STEP_MM)

        direction_ok = displacement_mm > 0
        magnitude_ok = error_mm < DIR_TOLERANCE_MM

        status = "OK" if (direction_ok and magnitude_ok) else "FAIL"
        print(f"    Actual displacement: {displacement_mm:.3f} mm "
              f"(error: {error_mm:.3f} mm) [{status}]")
        if not direction_ok:
            print(f"    DIRECTION ERROR: leg moved {displacement_mm:.3f} mm "
                  f"(expected positive)")

        # Command back to start
        print(f"    Returning to start...")
        harness.send(encode_set_input_pos(axis_id, start_raw))
        harness.poll_for(SETTLE_TIME_S)

        # Check return accuracy
        return_pos_rev = harness.states[axis_id].pos_rev
        return_mm = return_pos_rev / mm_to_rev
        return_err_mm = abs(return_mm - start_mm)
        return_ok = return_err_mm < RETURN_TOLERANCE_MM
        print(f"    Return error: {return_err_mm:.3f} mm "
              f"[{'OK' if return_ok else 'FAIL'}]")

        leg_pass = direction_ok and magnitude_ok and return_ok
        leg_results[axis_id] = leg_pass
        if not leg_pass:
            all_pass = False

        # Check for errors after each leg
        if harness.states[axis_id].has_errors:
            names = error_names(harness.states[axis_id].active_errors)
            print(f"    WARNING: Axis {axis_id} errors: {names}")
            all_pass = False
            leg_results[axis_id] = False

    harness.idle_all()

    # -- Summary ------------------------------------------------------------
    print(f"\n  Per-leg results:")
    for axis_id in LEG_AXES:
        status = "PASS" if leg_results[axis_id] else "FAIL"
        print(f"    Leg {axis_id}: {status}")

    if all_pass:
        print(f"\n  PASS: All {NUM_LEGS} legs moved in the correct direction "
              f"with correct magnitude")
        return True
    else:
        failed = [a for a, p in leg_results.items() if not p]
        print(f"\n  FAIL: Legs {failed} did not pass direction/magnitude check")
        return False


# ===========================================================================
# Test registry and CLI
# ===========================================================================

TESTS = {
    'can_coord': ('Six-leg CAN coordination (B1 — exit gate)',
                  test_can_coordination),
    'direction': ('Direction & sign convention (B2)',
                  test_direction),
}

TEST_GROUPS = {
    'all': ['can_coord', 'direction'],
}


def main():
    parser = argparse.ArgumentParser(
        description='Supported-platform test harness for Jugglebot (Stage B)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  can_coord   Stage B1: Six-leg CAN coordination (EXIT GATE)
  direction   Stage B2: Direction & sign convention check (all legs)

Test groups:
  all         Run B1 then B2 (B2 skipped if B1 fails) [DEFAULT]

Prerequisites:
  - Robot fully assembled, platform mechanically supported
  - All 6 leg ODrives powered and on the CAN bus
  - Use --home to home all legs before testing

Safety:
  Current limit is set to {curr}A (50% of rated {rated}A).
  All tests send IDLE to all 6 axes on completion, error, or Ctrl-C.
  Platform MUST be mechanically supported.
        """.format(curr=SAFE_CURRENT_LIMIT_A,
                   rated=hw.ODRIVE_LEG_CURR_LIMIT_A))

    parser.add_argument('--interface', default='socketcan',
                        help='python-can interface (default: socketcan)')
    parser.add_argument('--channel', default='can0',
                        help='CAN channel name (default: can0)')
    parser.add_argument('--test', default='all',
                        choices=list(TESTS.keys()) + list(TEST_GROUPS.keys()),
                        help='Which test or group to run (default: all)')
    parser.add_argument('--home', action='store_true',
                        help='Home all 6 axes before running tests')

    args = parser.parse_args()

    # Install Ctrl-C handler for clean shutdown
    harness_ref = [None]
    original_handler = signal.getsignal(signal.SIGINT)

    def sigint_handler(sig, frame):
        print("\n\n  *** Ctrl-C: Emergency IDLE (all axes) ***")
        if harness_ref[0] is not None:
            try:
                harness_ref[0].idle_all()
            except Exception:
                pass
        signal.signal(signal.SIGINT, original_handler)
        sys.exit(1)

    signal.signal(signal.SIGINT, sigint_handler)

    # Run tests
    harness = PlatformTestHarness(
        interface=args.interface,
        channel=args.channel)
    harness_ref[0] = harness

    print(f"\nJugglebot Supported-Platform Test Harness (Stage B)")
    print(f"  Interface: {args.interface}")
    print(f"  Channel:   {args.channel}")
    print(f"  Current limit: {SAFE_CURRENT_LIMIT_A} A "
          f"(50% of {hw.ODRIVE_LEG_CURR_LIMIT_A} A)")
    print(f"\n  *** Platform MUST be mechanically supported ***")

    if args.test in TEST_GROUPS:
        tests_to_run = TEST_GROUPS[args.test]
    else:
        tests_to_run = [args.test]

    results = {}
    try:
        with harness:
            if args.home:
                harness.home_all()

            for test_name in tests_to_run:
                label, func = TESTS[test_name]
                try:
                    result = func(harness)
                    results[test_name] = result
                except Exception as e:
                    print(f"\n  ERROR in {label}: {e}")
                    results[test_name] = False
                    try:
                        harness.idle_all()
                    except Exception:
                        pass
                    time.sleep(1.0)

                # B1 is an exit gate: if it fails, skip remaining tests
                if test_name == 'can_coord' and results.get('can_coord') is False:
                    print("\n  *** B1 FAILED — skipping remaining tests ***")
                    for remaining in tests_to_run[tests_to_run.index(test_name) + 1:]:
                        results[remaining] = None  # SKIP
                    break

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
        print(f"  {label:50s} {status}")

    all_pass = all(r is True or r is None for r in results.values())
    if all_pass:
        print("\nAll tests passed (or skipped).")
    else:
        print("\nSome tests FAILED. Review output above.")
        sys.exit(1)


if __name__ == '__main__':
    main()
