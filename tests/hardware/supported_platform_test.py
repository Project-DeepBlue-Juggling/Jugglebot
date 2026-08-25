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
_PROJECT_ROOT = os.path.dirname(os.path.dirname(_SCRIPT_DIR))
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
LEG_TOR_FF_SCALE = proto.INPUT_SCALE_LEG_TOR   # 10000 (0.0001 Nm per LSB)


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


def encode_set_traj_vel_limit(axis_id: int, vel_limit: float) -> can.Message:
    data = struct.pack('<f', vel_limit) + bytes(4)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_traj_vel_limit'),
                       data=data, dlc=8, is_extended_id=False)


def encode_set_traj_acc_limits(axis_id: int, acc_limit: float,
                               dec_limit: float) -> can.Message:
    data = struct.pack('<ff', acc_limit, dec_limit)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_traj_acc_limits'),
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


# Vendored from jugglebot.can.odrive.ERROR_CODES (this harness deliberately
# imports no ROS workspace).  PINNED by tests/ros/test_gui_geometry.py::
# TestODriveErrorTablePins — the copy must stay byte-equivalent to the
# authoritative table.  Before that pin existed this copy had drifted: it
# stopped at 0x10000 and mislabelled 0x1000000 (WATCHDOG_TIMER_EXPIRED) as
# 'ESTOP_REQUESTED', so a bench watchdog expiry printed the wrong cause and
# SPINOUT_DETECTED — the 2026-08-10 leg-0 fault — printed nothing at all.
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
    0x1000000: 'WATCHDOG_TIMER_EXPIRED',
    0x2000000: 'ESTOP_REQUESTED',
    0x4000000: 'SPINOUT_DETECTED',
    0x8000000: 'BRAKE_RESISTOR_DISARMED',
    0x10000000: 'THERMISTOR_DISCONNECTED',
    0x40000000: 'CALIBRATION_ERROR',
}


def error_names(bitmask: int) -> list:
    """Decode an ODrive error bitmask into names.

    Unknown bits survive as ONE aggregate UNKNOWN(0x…) entry rather than being
    silently dropped — a firmware code this build's table has never seen must
    still reach the operator standing next to the robot.  Same contract as
    jugglebot.can.odrive.error_names (and the GUI's odrive-errors.js).
    """
    known_mask = 0
    names = []
    for bit, name in ERROR_CODES.items():
        if bitmask & bit:
            names.append(name)
        known_mask |= bit
    unknown = bitmask & ~known_mask
    if unknown:
        names.append(f'UNKNOWN(0x{unknown:X})')
    return names


def decode_axis_errors(active: int, disarm: int) -> str:
    """'active=[NAME,...] disarm=[NAME,...] 0x<active>/0x<disarm>'.

    BOTH MASKS, ALWAYS — same shape and same reason as
    teensy_bridge_node._decode_axis_errors.  The 2026-08-10 leg-0 spinout had
    active_errors == 0 and the truth (SPINOUT_DETECTED) only in disarm_reason:
    active_errors self-heals, disarm_reason is sticky until CLEAR_ERRORS.  A
    bench report that reads one mask is blind to exactly the class of fault
    that already bit this robot — and this harness unpacked disarm_reason into
    AxisState from the start while printing it at none of its report sites.
    The raw hex pair rides along so a pasted log line stays decodable even
    against a table that later gains a bit.
    """
    return (f'active=[{",".join(error_names(active))}] '
            f'disarm=[{",".join(error_names(disarm))}] '
            f'0x{active:X}/0x{disarm:X}')


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
    error_frame_count: int = 0   # Get_Error frames seen (freshness, not a fault)

    @property
    def has_errors(self) -> bool:
        """Active-error predicate (the raw ODrive bit)."""
        return self.active_errors != 0

    @property
    def has_fault(self) -> bool:
        """EITHER mask is set — the predicate every report site gates on.

        has_errors alone is blind to a self-healed fault: SPINOUT_DETECTED
        (the 2026-08-10 leg-0 event) clears out of active_errors but stays in
        the sticky disarm_reason until CLEAR_ERRORS.  Gating the bench reports
        on active_errors only meant that fault printed nothing at all.

        Safe against stale latches because every test clears errors and then
        waits for a post-clear Get_Error frame (see clear_all_errors) before
        anything is gated on this.
        """
        return self.has_errors or self.disarm_reason != 0

    @property
    def fault_summary(self) -> str:
        """Both masks, decoded — see decode_axis_errors."""
        return decode_axis_errors(self.active_errors, self.disarm_reason)

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
    ERROR_FRESH_TIMEOUT_S = 0.5   # bounded wait for a post-CLEAR_ERRORS frame

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
                          f"{s.fault_summary}")
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
        s.error_frame_count += 1

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
        # Bounded, best-effort wait for a POST-clear Get_Error frame on every
        # axis before anything is judged.  The cached masks are only ever as
        # new as the last frame, and disarm_reason is STICKY: without this wait
        # a latch from an earlier session reads as a live fault and aborts the
        # very test that just cleared it (require_no_errors_all runs seconds
        # later in every Stage-B test).  If the ODrives are not broadcasting
        # Get_Error at all the counters never move, the loop simply expires and
        # both masks stay 0 — identical behaviour to before.
        #
        # The baseline is snapshotted HERE, after the drain above, not before
        # the CLEAR_ERRORS send: a frame already in flight when the clear went
        # out carries a PRE-clear mask, and counting it would satisfy the wait
        # with the very staleness the wait exists to exclude.  The drain
        # absorbs those, so the frame this loop waits for provably arrived at
        # least 0.1 s after the clear was sent.
        frames_before = {a: self.states[a].error_frame_count for a in LEG_AXES}
        deadline = time.time() + self.ERROR_FRESH_TIMEOUT_S
        while time.time() < deadline:
            self._poll(timeout=0.02)
            if all(self.states[a].error_frame_count > frames_before[a]
                   for a in LEG_AXES):
                break
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            if s.has_fault:
                print(f"  WARNING: Axis {axis_id} still faulted: "
                      f"{s.fault_summary}")
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
        """Assert no axis is faulted (either mask)."""
        self._poll()
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            if s.has_fault:
                raise RuntimeError(
                    f"Axis {axis_id} has faults: {s.fault_summary}")

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
            f"State: {s.state_name}, {s.fault_summary}")

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

    def enter_trap_traj_mode_all(self,
                                vel_limit: float = None,
                                acc_limit: float = 10.0,
                                dec_limit: float = 10.0):
        """Put all 6 axes into POSITION/TRAP_TRAJ with gentle limits.

        Uses trapezoidal trajectory mode for smooth, ramped movements.
        The trajectory_done flag on each axis indicates when the move completes.
        """
        if vel_limit is None:
            vel_limit = hw.JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS  # 2.5 rev/s
        self._poll()  # refresh encoder readings

        for axis_id in LEG_AXES:
            pos_now_raw = self.states[axis_id].pos_rev_raw
            self.send(encode_set_controller_mode(
                axis_id, 'POSITION', 'TRAP_TRAJ'))
            time.sleep(0.05)
            # Set trajectory limits
            self.send(encode_set_traj_vel_limit(axis_id, vel_limit))
            self.send(encode_set_traj_acc_limits(axis_id, acc_limit, dec_limit))
            # Hold current position before entering CLOSED_LOOP
            self.send(encode_set_input_pos(axis_id, pos_now_raw))
            time.sleep(0.05)
            self.send(encode_set_state(axis_id, 'CLOSED_LOOP'))

        # Wait for all to enter CLOSED_LOOP
        for axis_id in LEG_AXES:
            self._wait_for_closed_loop(axis_id)

        print(f"  All {NUM_LEGS} axes in POSITION/TRAP_TRAJ, CLOSED_LOOP:")
        print(f"    Trap limits: vel={vel_limit} rev/s, "
              f"acc={acc_limit} rev/s², dec={dec_limit} rev/s²")
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            print(f"    Axis {axis_id}: holding at {s.pos_rev_raw:.4f} rev raw "
                  f"({s.pos_rev / MM_TO_REV[axis_id]:.2f} mm)")

    def wait_for_trajectory_done(self, axis_id: int, timeout_s: float = 10.0):
        """Poll until trajectory_done flag is set on the specified axis."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self._poll(timeout=0.02)
            if self.states[axis_id].trajectory_done:
                return True
            if self.states[axis_id].has_fault:
                raise RuntimeError(
                    f"Axis {axis_id} fault while waiting for trajectory: "
                    f"{self.states[axis_id].fault_summary}")
            if not self.all_heartbeats_fresh():
                raise RuntimeError("Heartbeat timeout during trajectory wait")
        return False  # timeout

    def home_axis(self, axis_id: int, homing_current_lim: float = 4.0):
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
            if abs(avg) >= homing_current_lim:
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

    The ODrives broadcast encoder estimates at ~100 Hz (10 ms period).  At a
    200 Hz command rate the test cannot expect encoder updates from every axis
    in every 5 ms cycle.  Instead, the test validates:

      1. Command throughput:  we can send 6 position frames every cycle
         without exceeding the target period.
      2. Encoder liveness:    every axis maintains its ~100 Hz broadcast
         rate — no axis goes silent for more than MAX_ENCODER_GAP_MS.
      3. No errors:           no axis enters an error state during the run.

    Procedure:
      1. Enter POSITION/PASSTHROUGH on all 6 legs
      2. Run 1000 command cycles at 200 Hz (5 seconds)
      3. Each cycle: cosine position profile (10 mm amplitude, 1 Hz)
         Cosine starts at trough (homed position), only extends.
      4. Track per-axis encoder timestamps and command cycle timing
      5. Report statistics

    Pass criteria:
      - p99 command-cycle time < target period (5 ms)
      - Max encoder gap per axis < MAX_ENCODER_GAP_MS (50 ms)
      - Per-axis encoder rate >= MIN_ENCODER_RATE_HZ (80 Hz)
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
    MAX_ENCODER_GAP_MS = 50.0                # Max gap between encoder reads
    MIN_ENCODER_RATE_HZ = 80.0               # Minimum acceptable per-axis rate

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

    # Timing tracking
    cycle_times = []
    send_durations = []

    # Per-axis encoder tracking: list of timestamps for each encoder message
    encoder_timestamps: dict[int, list[float]] = {a: [] for a in LEG_AXES}
    # Snapshot the initial encoder times so we can detect new messages
    last_enc_time = {a: harness.states[a].last_encoder_time for a in LEG_AXES}

    print(f"\n  Running {NUM_CYCLES} cycles at {TARGET_RATE_HZ} Hz "
          f"({NUM_CYCLES / TARGET_RATE_HZ:.1f}s)...")
    print(f"  Cosine profile: {AMPLITUDE_MM} mm amplitude, {FREQ_HZ} Hz")
    print(f"  Target cycle period: {TARGET_PERIOD_S * 1000:.1f} ms")
    print(f"  ODrive encoder broadcast: ~100 Hz (expect ~50% per-cycle hit rate)")

    t_start = time.time()

    for cycle in range(NUM_CYCLES):
        t_cycle_start = time.time()
        t_elapsed = t_cycle_start - t_start

        # Compute cosine position for this cycle
        # pos_mm = amplitude * (1 - cos(2π·f·t)) / 2  →  range [0, amplitude]
        # At t=0: pos_mm = 0 (most compressed = homed)
        phase = 2.0 * math.pi * FREQ_HZ * t_elapsed
        pos_frac = (1.0 - math.cos(phase)) / 2.0  # 0..1

        # Send position commands to all 6 axes (minimal delay)
        t_send_start = time.time()
        for axis_id in LEG_AXES:
            # Extension = positive mm = negative raw rev
            target_raw = start_raw[axis_id] - pos_frac * amp_rev[axis_id]
            harness.send_no_delay(
                encode_set_input_pos(axis_id, target_raw))
        t_send_end = time.time()
        send_durations.append(t_send_end - t_send_start)

        # Poll for any available CAN messages (non-blocking drain)
        harness._poll(timeout=0.0)

        # Record any new encoder timestamps
        for axis_id in LEG_AXES:
            t_enc = harness.states[axis_id].last_encoder_time
            if t_enc > last_enc_time[axis_id]:
                encoder_timestamps[axis_id].append(t_enc)
                last_enc_time[axis_id] = t_enc

        # Check for errors
        for axis_id in LEG_AXES:
            if harness.states[axis_id].has_fault:
                harness.idle_all()
                raise RuntimeError(
                    f"Axis {axis_id} fault during B1 at cycle {cycle}: "
                    f"{harness.states[axis_id].fault_summary}")

        t_cycle_end = time.time()
        cycle_times.append(t_cycle_end - t_cycle_start)

        # Sleep for remainder of cycle period
        remaining = TARGET_PERIOD_S - (t_cycle_end - t_cycle_start)
        if remaining > 0:
            time.sleep(remaining)

        # Progress indicator every 200 cycles
        if (cycle + 1) % 200 == 0:
            print(f"    Cycle {cycle + 1}/{NUM_CYCLES}")

    t_end = time.time()
    test_duration = t_end - t_start

    # Return to start positions
    for axis_id in LEG_AXES:
        harness.send(encode_set_input_pos(axis_id, start_raw[axis_id]))
    harness.poll_for(1.0)
    harness.idle_all()

    # -- Statistics ---------------------------------------------------------
    cycle_arr = np.array(cycle_times) * 1000  # ms
    send_arr = np.array(send_durations) * 1000  # ms

    print(f"\n  Results:")
    print(f"    Test duration:       {test_duration:.2f} s")
    print(f"    Total cycles:        {NUM_CYCLES}")
    print(f"    Cycle time (ms):     mean={cycle_arr.mean():.2f}, "
          f"p95={np.percentile(cycle_arr, 95):.2f}, "
          f"p99={np.percentile(cycle_arr, 99):.2f}, "
          f"max={cycle_arr.max():.2f}")
    print(f"    Send burst (ms):     mean={send_arr.mean():.3f}, "
          f"max={send_arr.max():.3f}")

    # Per-axis encoder analysis
    all_pass = True

    print(f"\n    Per-axis encoder feedback:")
    for axis_id in LEG_AXES:
        ts = encoder_timestamps[axis_id]
        count = len(ts)
        rate = count / test_duration if test_duration > 0 else 0

        if count >= 2:
            gaps = np.diff(ts) * 1000  # ms
            max_gap = gaps.max()
            mean_gap = gaps.mean()
        else:
            max_gap = float('inf')
            mean_gap = float('inf')

        rate_ok = rate >= MIN_ENCODER_RATE_HZ
        gap_ok = max_gap < MAX_ENCODER_GAP_MS

        status = "OK" if (rate_ok and gap_ok) else "FAIL"
        print(f"      Axis {axis_id}: {count} msgs ({rate:.1f} Hz), "
              f"gap mean={mean_gap:.1f} ms, max={max_gap:.1f} ms  [{status}]")

        if not rate_ok:
            print(f"        FAIL: rate {rate:.1f} Hz < {MIN_ENCODER_RATE_HZ} Hz minimum")
            all_pass = False
        if not gap_ok:
            print(f"        FAIL: max gap {max_gap:.1f} ms >= "
                  f"{MAX_ENCODER_GAP_MS} ms threshold")
            all_pass = False

    # -- Pass/Fail ----------------------------------------------------------
    p99 = np.percentile(cycle_arr, 99)
    p99_ok = p99 < TARGET_PERIOD_S * 1000

    if not p99_ok:
        print(f"\n    FAIL: p99 cycle time {p99:.2f} ms >= "
              f"{TARGET_PERIOD_S * 1000:.1f} ms target")
        all_pass = False

    if all_pass:
        print(f"\n  PASS: CAN throughput verified")
        print(f"    Command cycle p99: {p99:.2f} ms < {TARGET_PERIOD_S * 1000:.1f} ms")
        print(f"    All axes maintaining encoder feedback with no gaps > "
              f"{MAX_ENCODER_GAP_MS} ms")
        return True
    else:
        print(f"\n  FAIL: CAN coordination test did not meet all criteria")
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
      1. Enter POSITION/TRAP_TRAJ on all 6 legs (gentle ramped movements)
      2. For each leg, command +5 mm extension, wait for trajectory_done
      3. Command back to start, wait for trajectory_done

    Pass criteria (per leg):
      - Direction: encoder moved positive (extension = negative raw)
      - Magnitude: within 2 mm of commanded 5 mm
      - Return: within 1 mm of start

    Diagnostics: prints exact CAN arbitration IDs, raw values, and encoder
    readings to help isolate any per-axis direction issues.

    Overall pass: all 6 legs pass.
    """
    print("\n" + "=" * 60)
    print("STAGE B2: Direction and sign convention check (all legs)")
    print("=" * 60)

    STEP_MM = 5.0          # Extension step size
    DIR_TOLERANCE_MM = 2.0 # Maximum error for direction/magnitude check
    RETURN_TOLERANCE_MM = 1.0  # Maximum error for return check
    TRAJ_TIMEOUT_S = 10.0  # Max time to wait for trajectory_done

    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.require_no_errors_all()
    harness.enter_trap_traj_mode_all(vel_limit=2.5, acc_limit=10.0,
                                     dec_limit=10.0)

    all_pass = True
    leg_results = {}

    for axis_id in LEG_AXES:
        print(f"\n  --- Leg {axis_id} ---")
        mm_to_rev = MM_TO_REV[axis_id]

        # Record starting position (raw ODrive values)
        harness._poll()
        start_raw = harness.states[axis_id].pos_rev_raw
        start_inverted = harness.states[axis_id].pos_rev
        start_mm = start_inverted / mm_to_rev
        print(f"    Start raw: {start_raw:.4f} rev")

        # Compute target: +5 mm extension = negative raw delta
        step_rev = STEP_MM * mm_to_rev  # always positive
        target_raw = start_raw - step_rev  # extension = more negative raw
        cmd_arb_id = arb_id(axis_id, 'set_input_pos')

        print(f"    Target raw: {target_raw:.4f} rev "
              f"(CAN arb_id=0x{cmd_arb_id:02X}, "
              f"delta={-step_rev:.4f} rev)")

        # Send position command and wait for trajectory completion
        harness.send(encode_set_input_pos(axis_id, target_raw))
        traj_ok = harness.wait_for_trajectory_done(axis_id, TRAJ_TIMEOUT_S)
        if not traj_ok:
            print(f"    WARNING: trajectory_done timeout after {TRAJ_TIMEOUT_S}s")

        # Small extra settle time after trajectory_done
        harness.poll_for(0.2)

        # Read actual position
        after_raw = harness.states[axis_id].pos_rev_raw
        after_inverted = harness.states[axis_id].pos_rev
        after_mm = after_inverted / mm_to_rev
        displacement_mm = after_mm - start_mm
        raw_displacement = after_raw - start_raw
        error_mm = abs(displacement_mm - STEP_MM)

        direction_ok = displacement_mm > 0
        magnitude_ok = error_mm < DIR_TOLERANCE_MM

        # Diagnostic output
        print(f"    After move raw: {after_raw:.4f} rev "
              f"(expected {target_raw:.4f})")
        print(f"    Raw displacement: {raw_displacement:.4f} rev "
              f"(expected {-step_rev:.4f})")
        print(f"    Extension displacement: {displacement_mm:.3f} mm "
              f"(expected {STEP_MM:.1f} mm, error: {error_mm:.3f} mm)")

        status = "OK" if (direction_ok and magnitude_ok) else "FAIL"
        print(f"    Direction & magnitude: [{status}]")
        if not direction_ok:
            print(f"    DIRECTION ERROR: raw moved {raw_displacement:+.4f} rev "
                  f"(expected negative for extension)")

        # Command back to start
        print(f"    Returning to start (raw={start_raw:.4f})...")
        harness.send(encode_set_input_pos(axis_id, start_raw))
        traj_ok = harness.wait_for_trajectory_done(axis_id, TRAJ_TIMEOUT_S)
        if not traj_ok:
            print(f"    WARNING: return trajectory_done timeout")
        harness.poll_for(0.2)

        # Check return accuracy
        return_raw = harness.states[axis_id].pos_rev_raw
        return_inverted = harness.states[axis_id].pos_rev
        return_mm = return_inverted / mm_to_rev
        return_err_mm = abs(return_mm - start_mm)
        return_ok = return_err_mm < RETURN_TOLERANCE_MM
        print(f"    Return raw: {return_raw:.4f} rev "
              f"(started at {start_raw:.4f}, err: {return_err_mm:.3f} mm) "
              f"[{'OK' if return_ok else 'FAIL'}]")

        leg_pass = direction_ok and magnitude_ok and return_ok
        leg_results[axis_id] = leg_pass
        if not leg_pass:
            all_pass = False

        # Check for errors after each leg
        if harness.states[axis_id].has_fault:
            print(f"    WARNING: Axis {axis_id} faulted: "
                  f"{harness.states[axis_id].fault_summary}")
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
# Test B3: Multi-leg position hold (supported)
# ===========================================================================

def test_position_hold(harness: PlatformTestHarness):
    """Stage B3: Multi-leg position hold (supported).

    From MOTION_PLANNER_PLAN Phase 3 Stage B:
    "Command all six legs to hold their current positions using the unloaded
    baseline ODrive gains from Stage A (or lower). Verify all legs respond,
    no leg oscillates, and the system is stable."

    Procedure:
      1. Set ODrive baseline gains (pos_gain=40, vel_gain=0.2, vel_int_gain=0.32)
      2. Enter POSITION/PASSTHROUGH on all 6 legs (hold current position)
      3. Sample encoder positions at ~10 Hz for 10 seconds
      4. Track per-leg max deviation from start position

    Pass criteria (per leg):
      - Max deviation < 1.0 mm from start position
      - No errors on any axis throughout the hold
      - All heartbeats remain fresh
    """
    print("\n" + "=" * 60)
    print("STAGE B3: Multi-leg position hold (supported)")
    print("=" * 60)

    HOLD_DURATION_S = 10.0
    SAMPLE_INTERVAL_S = 0.1  # ~10 Hz
    MAX_DEVIATION_MM = 1.0

    # Stage A baseline gains
    POS_GAIN = 40.0
    VEL_GAIN = 0.2
    VEL_INT_GAIN = 0.32

    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.require_no_errors_all()

    # Set baseline ODrive gains on all axes
    print(f"  Setting ODrive gains: pos_gain={POS_GAIN}, vel_gain={VEL_GAIN}, "
          f"vel_int_gain={VEL_INT_GAIN}")
    for axis_id in LEG_AXES:
        harness.send(encode_set_pos_gain(axis_id, POS_GAIN))
        harness.send(encode_set_vel_gains(axis_id, VEL_GAIN, VEL_INT_GAIN))

    harness.enter_position_mode_all()

    # Record starting positions
    harness._poll()
    start_mm = {}
    for axis_id in LEG_AXES:
        s = harness.states[axis_id]
        start_mm[axis_id] = s.pos_rev / MM_TO_REV[axis_id]

    # Sample loop
    max_dev_mm = {axis_id: 0.0 for axis_id in LEG_AXES}
    all_pass = True
    errors_seen = {axis_id: False for axis_id in LEG_AXES}
    n_samples = int(HOLD_DURATION_S / SAMPLE_INTERVAL_S)

    print(f"  Holding position for {HOLD_DURATION_S}s "
          f"({n_samples} samples at {1/SAMPLE_INTERVAL_S:.0f} Hz)...")

    for i in range(n_samples):
        harness.poll_for(SAMPLE_INTERVAL_S)

        for axis_id in LEG_AXES:
            s = harness.states[axis_id]
            current_mm = s.pos_rev / MM_TO_REV[axis_id]
            dev = abs(current_mm - start_mm[axis_id])
            if dev > max_dev_mm[axis_id]:
                max_dev_mm[axis_id] = dev

            if s.has_fault and not errors_seen[axis_id]:
                errors_seen[axis_id] = True
                print(f"    ERROR at sample {i}: Axis {axis_id}: "
                      f"{s.fault_summary}")

    harness.idle_all()

    # -- Results ------------------------------------------------------------
    print(f"\n  --- Hold results ({HOLD_DURATION_S}s) ---")
    for axis_id in LEG_AXES:
        dev = max_dev_mm[axis_id]
        ok = dev < MAX_DEVIATION_MM and not errors_seen[axis_id]
        status = "OK" if ok else "FAIL"
        print(f"    Leg {axis_id}: max dev {dev:.3f} mm [{status}]")
        if not ok:
            all_pass = False
            if errors_seen[axis_id]:
                print(f"      Errors detected during hold")

    if all_pass:
        print(f"\n  PASS: All {NUM_LEGS} legs held position within "
              f"{MAX_DEVIATION_MM} mm for {HOLD_DURATION_S}s")
        return True
    else:
        failed = [a for a in LEG_AXES
                  if max_dev_mm[a] >= MAX_DEVIATION_MM or errors_seen[a]]
        print(f"\n  FAIL: Legs {failed} exceeded deviation or had errors")
        return False


# ===========================================================================
# Test B4: Emergency stop (six legs)
# ===========================================================================

def test_estop(harness: PlatformTestHarness):
    """Stage B4: Emergency stop test (six legs).

    From MOTION_PLANNER_PLAN Phase 3 Stage B:
    "Trigger each failure mode (IPC loss, process crash, explicit stop) and
    verify all six legs idle simultaneously and cleanly."

    Since this standalone harness has no IPC layer, we test explicit IDLE:
      1. Enter POSITION/PASSTHROUGH on all 6 legs (holding position)
      2. Send IDLE to all 6 axes simultaneously
      3. Verify all axes transition to IDLE within 200ms
      4. Verify no errors and all velocities settle to zero

    Pass criteria:
      - All 6 axes reach IDLE within 200ms
      - No errors on any axis after stop
      - All velocities < 0.05 rev/s after settling
    """
    print("\n" + "=" * 60)
    print("STAGE B4: Emergency stop test (six legs)")
    print("=" * 60)

    MAX_STOP_LATENCY_MS = 200.0
    MAX_RESIDUAL_VEL = 0.05  # rev/s

    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.require_no_errors_all()
    harness.enter_position_mode_all()

    # Verify all axes are in CLOSED_LOOP
    for axis_id in LEG_AXES:
        if not harness.states[axis_id].is_closed_loop:
            raise RuntimeError(
                f"Axis {axis_id} not in CLOSED_LOOP before e-stop test")

    print(f"  All axes in CLOSED_LOOP, holding position.")
    print(f"  Sending IDLE to all {NUM_LEGS} axes...")

    # Send IDLE to all axes as fast as possible (no inter-message delay)
    t_stop = time.time()
    for axis_id in LEG_AXES:
        harness.send_no_delay(encode_set_state(axis_id, 'IDLE'))

    # Poll until all axes report IDLE
    idle_times = {}
    deadline = time.time() + 1.0  # absolute timeout

    while time.time() < deadline:
        harness._poll(timeout=0.005)
        for axis_id in LEG_AXES:
            if axis_id not in idle_times and harness.states[axis_id].is_idle:
                idle_times[axis_id] = time.time()
        if len(idle_times) == NUM_LEGS:
            break

    # Calculate per-axis latency
    print(f"\n  Per-axis IDLE latency:")
    all_pass = True
    max_latency_ms = 0.0

    for axis_id in LEG_AXES:
        if axis_id in idle_times:
            latency_ms = (idle_times[axis_id] - t_stop) * 1000
            max_latency_ms = max(max_latency_ms, latency_ms)
            ok = latency_ms < MAX_STOP_LATENCY_MS
            print(f"    Axis {axis_id}: {latency_ms:.1f} ms [{'OK' if ok else 'FAIL'}]")
            if not ok:
                all_pass = False
        else:
            print(f"    Axis {axis_id}: TIMEOUT (never reached IDLE) [FAIL]")
            all_pass = False

    # Settle and check for errors / residual velocity
    time.sleep(0.5)
    harness._poll()

    print(f"\n  Post-stop checks:")
    for axis_id in LEG_AXES:
        s = harness.states[axis_id]
        vel = abs(s.vel_rps_raw)
        idle_ok = s.is_idle
        # Both masks, as everywhere else: a stop that trips a fault which then
        # self-heals out of active_errors still has to be reported.  The clear
        # at the top of this test zeroed both masks and waited for a post-clear
        # frame, so anything set here was set BY the e-stop — and it is named,
        # not just counted, so the operator can judge it on the spot.
        err_ok = not s.has_fault
        vel_ok = vel < MAX_RESIDUAL_VEL

        issues = []
        if not idle_ok:
            issues.append(f"state={s.state_name}")
        if not err_ok:
            issues.append(s.fault_summary)
        if not vel_ok:
            issues.append(f"vel={vel:.4f} rev/s")

        if issues:
            print(f"    Axis {axis_id}: FAIL ({', '.join(issues)})")
            all_pass = False
        else:
            print(f"    Axis {axis_id}: IDLE, no errors, vel={vel:.4f} rev/s [OK]")

    if all_pass:
        print(f"\n  PASS: All {NUM_LEGS} axes reached IDLE within "
              f"{max_latency_ms:.1f} ms, no errors, velocities settled")
        return True
    else:
        print(f"\n  FAIL: E-stop did not meet all criteria")
        return False


# ===========================================================================
# Test B5: Feedforward dry run (supported, analytical only)
# ===========================================================================

def test_feedforward_dry_run(harness: PlatformTestHarness):
    """Stage B5: Feedforward dry run (supported, analytical only).

    From MOTION_PLANNER_PLAN Phase 3 Stage B:
    "Enable gravity torque_ff while the platform is still supported. Log the
    commanded feedforward torques for each leg at the active pose. Verify they
    are in the expected direction and roughly the expected magnitude (compare
    to platform_mass x g / 6 as a sanity check, adjusted for CoM offset
    geometry). This is an analytical/sanity check only — do not attempt to
    measure current reduction, because motor stiction (~0.075 Nm per leg) is
    ~4x larger than the per-leg gravity torque (~0.018 Nm) and masks the
    feedforward effect."

    Procedure:
      1. Compute gravity feedforward torques at active pose using dynamics model
      2. Sanity-check direction (all positive) and total magnitude vs m*g estimate
      3. Apply feedforward via set_input_pos torque_ff, hold 3s
      4. Remove feedforward, hold 1s, verify no transient errors

    Pass criteria:
      - All computed torques positive (extension direction = supporting weight)
      - Total torque sum within 50% of total expected (m*g * mean_spool_radius)
      - No errors during feedforward hold or removal

    Note: per-leg torques vary significantly due to CoM offset [-14.5, -67, 54] mm.
    We validate direction and total, not per-leg uniformity.
    """
    print("\n" + "=" * 60)
    print("STAGE B5: Feedforward dry run (supported, analytical only)")
    print("=" * 60)

    # Import motion modules (available because _ROS_PKG_DIR is on sys.path)
    from jugglebot.motion.geometry import StewartGeometry
    from jugglebot.motion.dynamics import DynamicsParams, gravity_to_motor_torques

    FF_HOLD_S = 3.0
    FF_REMOVE_HOLD_S = 1.0
    TOTAL_MAGNITUDE_TOLERANCE = 0.50  # 50% tolerance on total torque sum

    # -- Analytical computation ---------------------------------------------
    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    pos_active = np.zeros(3)
    rot_active = np.eye(3)

    torques_Nm = gravity_to_motor_torques(pos_active, rot_active, geom, params)

    # Sanity estimates
    total_weight_N = params.mass_kg * params.gravity_mps2
    force_per_leg_N = total_weight_N / NUM_LEGS
    spool_radius_m = geom.spool_radius_mm / 1000.0  # (6,) array
    mean_spool_radius_m = float(np.mean(spool_radius_m))
    expected_total_torque_Nm = total_weight_N * mean_spool_radius_m

    print(f"\n  Gravity feedforward at active pose:")
    print(f"    Platform mass: {params.mass_kg} kg, g: {params.gravity_mps2} m/s²")
    print(f"    Total weight: {total_weight_N:.3f} N")
    print(f"    Mean spool radius: {mean_spool_radius_m*1000:.2f} mm")
    print(f"    Expected total torque: ~{expected_total_torque_Nm:.4f} Nm "
          f"(m*g*r_spool)")

    all_pass = True
    torque_ff_ints = []

    print(f"\n  Per-leg computed torques (CoM offset: "
          f"{list(params.com_offset_mm)} mm):")
    for i, axis_id in enumerate(LEG_AXES):
        t = torques_Nm[i]
        ff_int = int(round(t * LEG_TOR_FF_SCALE))
        torque_ff_ints.append(ff_int)
        sign_ok = t > 0
        sign_str = "+" if sign_ok else "NEGATIVE"
        print(f"    Leg {axis_id}: {t:.4f} Nm (ff_int16 = {ff_int}) [{sign_str}]")
        if not sign_ok:
            print(f"      SIGN ERROR: expected positive (extension)")
            all_pass = False

    # Check total magnitude
    total_torque_Nm = float(np.sum(torques_Nm))
    total_ratio = total_torque_Nm / expected_total_torque_Nm
    total_ok = abs(total_ratio - 1.0) < TOTAL_MAGNITUDE_TOLERANCE

    print(f"\n  Total torque: {total_torque_Nm:.4f} Nm "
          f"(expected ~{expected_total_torque_Nm:.4f} Nm, "
          f"ratio = {total_ratio:.2f}) "
          f"[{'OK' if total_ok else 'FAIL'}]")
    if not total_ok:
        print(f"    MAGNITUDE: total ratio {total_ratio:.2f} outside "
              f"[{1-TOTAL_MAGNITUDE_TOLERANCE:.1f}, "
              f"{1+TOTAL_MAGNITUDE_TOLERANCE:.1f}]")
        all_pass = False

    # -- Apply feedforward on hardware -------------------------------------
    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.require_no_errors_all()
    harness.enter_position_mode_all()

    print(f"\n  Applying feedforward to all axes ({FF_HOLD_S}s hold)...")
    for axis_id in LEG_AXES:
        pos_raw = harness.states[axis_id].pos_rev_raw
        harness.send(encode_set_input_pos(
            axis_id, pos_raw, vel_ff=0, torque_ff=torque_ff_ints[axis_id]))

    # Hold with feedforward, monitoring for errors
    ff_errors = False
    deadline = time.time() + FF_HOLD_S
    while time.time() < deadline:
        harness._poll(timeout=0.05)
        for axis_id in LEG_AXES:
            if harness.states[axis_id].has_fault:
                print(f"    ERROR during ff hold: Axis {axis_id}: "
                      f"{harness.states[axis_id].fault_summary}")
                ff_errors = True
        if ff_errors:
            break

    if ff_errors:
        all_pass = False
        print(f"    FAIL: Errors during feedforward hold")
    else:
        print(f"    All axes stable, no errors.")

    # Remove feedforward
    print(f"  Removing feedforward ({FF_REMOVE_HOLD_S}s hold)...")
    for axis_id in LEG_AXES:
        pos_raw = harness.states[axis_id].pos_rev_raw
        harness.send(encode_set_input_pos(axis_id, pos_raw, vel_ff=0, torque_ff=0))

    remove_errors = False
    deadline = time.time() + FF_REMOVE_HOLD_S
    while time.time() < deadline:
        harness._poll(timeout=0.05)
        for axis_id in LEG_AXES:
            if harness.states[axis_id].has_fault:
                print(f"    ERROR during ff removal: Axis {axis_id}: "
                      f"{harness.states[axis_id].fault_summary}")
                remove_errors = True
        if remove_errors:
            break

    if remove_errors:
        all_pass = False
        print(f"    FAIL: Errors during feedforward removal")
    else:
        print(f"    All axes stable, no errors.")

    harness.idle_all()

    if all_pass:
        print(f"\n  PASS: Feedforward values are sane and apply without errors")
        return True
    else:
        print(f"\n  FAIL: Feedforward dry run did not meet all criteria")
        return False


# ===========================================================================
# Test registry and CLI
# ===========================================================================

TESTS = {
    'can_coord':  ('Six-leg CAN coordination (B1 — exit gate)',
                   test_can_coordination),
    'direction':  ('Direction & sign convention (B2)',
                   test_direction),
    'hold':       ('Multi-leg position hold (B3)',
                   test_position_hold),
    'estop':      ('Emergency stop, six legs (B4)',
                   test_estop),
    'feedforward': ('Feedforward dry run (B5)',
                    test_feedforward_dry_run),
}

TEST_GROUPS = {
    'all':     ['can_coord', 'direction', 'hold', 'estop', 'feedforward'],
    'stage_b': ['can_coord', 'direction', 'hold', 'estop', 'feedforward'],
}


def main():
    parser = argparse.ArgumentParser(
        description='Supported-platform test harness for Jugglebot (Stage B)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  can_coord    Stage B1: Six-leg CAN coordination (EXIT GATE)
  direction    Stage B2: Direction & sign convention check (all legs)
  hold         Stage B3: Multi-leg position hold (supported)
  estop        Stage B4: Emergency stop, six legs
  feedforward  Stage B5: Feedforward dry run (analytical)

Test groups:
  all / stage_b   Run B1-B5 in order (B2+ skipped if B1 fails) [DEFAULT]

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
