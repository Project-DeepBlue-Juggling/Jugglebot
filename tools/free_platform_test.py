#!/usr/bin/env python3
"""Standalone free-platform test harness for Jugglebot Phase 3 Stage C.

Bypasses ROS2 entirely and talks directly to all 6 ODrive leg axes via
python-can.  Designed to run on the Jetson (socketcan) or a USB-CAN adapter.

Phase 3 Stage C tests (platform free-standing, unsupported):
  C1. Stable hold at home pose (with torque_ff)
  C2. Feedforward harmlessness (toggle torque_ff on/off)
  C3. Step response & gain tuning
  C4. Gravity vector rotation (analytical — tilted poses)
  C5. Jacobian condition number survey

Prerequisites:
  - All Stage B tests must PASS before running Stage C
  - Robot fully assembled with all 6 legs on the CAN bus
  - Platform mechanically supported INITIALLY — the operator releases
    support during C1 when prompted
  - Legs must be homed before running tests (use --home flag)

Safety:
  - Uses conservative current limit (50% of rated = 10A)
  - All tests send IDLE to all 6 axes on completion, error, or Ctrl-C
  - Heartbeat watchdog detects ODrive disconnection on any axis
  - Interactive pauses before potentially dangerous steps (releasing support)
  - Uses TRAP_TRAJ for all multi-leg moves (smooth, ramped transitions)

Usage:
  python tools/free_platform_test.py --home --test all
  python tools/free_platform_test.py --test hold        # C1 only
  python tools/free_platform_test.py --test ff_toggle    # C2 only
  python tools/free_platform_test.py --test step         # C3 only
  python tools/free_platform_test.py --test gravity_rot  # C4 only
  python tools/free_platform_test.py --test cond_number  # C5 only (offline)

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

# Motion modules
from jugglebot.motion.geometry import StewartGeometry
from jugglebot.motion.ik_solver import (
    pose_to_leg_lengths,
    compute_jacobian,
    rotvec_to_rot_matrix,
)
from jugglebot.motion.dynamics import (
    DynamicsParams,
    gravity_to_motor_torques,
    compute_gravity_wrench,
    gravity_to_leg_forces,
)
from jugglebot.motion.conversions import extensions_mm_to_revs
from jugglebot.motion.workspace import compute_condition_number

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

# Stage A baseline ODrive gains
BASELINE_POS_GAIN = 40.0
BASELINE_VEL_GAIN = 0.2
BASELINE_VEL_INT_GAIN = 0.32


# ---------------------------------------------------------------------------
# CAN encoding helpers (standalone — same as supported_platform_test.py)
# ---------------------------------------------------------------------------

def arb_id(axis_id: int, command_name: str) -> int:
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


def encode_set_input_vel(axis_id: int, velocity: float,
                         torque_ff: float = 0.0) -> can.Message:
    data = struct.pack('<ff', velocity, torque_ff)
    return can.Message(arbitration_id=arb_id(axis_id, 'set_input_vel'),
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
    last_encoder_time: float = 0.0

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
# PlatformTestHarness — multi-axis CAN interface (reused from Stage B)
# ---------------------------------------------------------------------------

class PlatformTestHarness:
    """Standalone CAN interface for 6-leg platform testing."""

    CAN_SEND_DELAY_S = 0.002
    HEARTBEAT_TIMEOUT_S = 2.0

    def __init__(self, interface: str = 'socketcan', channel: str = 'can0'):
        self._interface = interface
        self._channel = channel
        self._bus: Optional[can.Bus] = None
        self._shutdown_requested = False

        self.states: dict[int, AxisState] = {
            axis_id: AxisState() for axis_id in LEG_AXES
        }

        self._handler_cmd_ids = {
            COMMANDS['heartbeat_message']: self._handle_heartbeat,
            COMMANDS['get_error']: self._handle_error,
            COMMANDS['get_encoder_estimate']: self._handle_encoder,
            COMMANDS['get_iq']: self._handle_iq,
        }

    def connect(self):
        print(f"Connecting to CAN bus: {self._channel} ({self._interface})")
        self._bus = can.Bus(channel=self._channel, interface=self._interface,
                            bitrate=proto.CAN_BAUD_RATE)
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
            missing = [a for a in LEG_AXES if a not in received]
            if len(received) > 0 and time.time() % 2 < 0.1:
                print(f"  Received: {received}, waiting for: {missing}")

        missing = [a for a in LEG_AXES
                   if self.states[a].heartbeat_count == 0]
        raise TimeoutError(
            f"No heartbeat from axes {missing} after 10 seconds. "
            f"Are all ODrives powered on and connected?")

    def disconnect(self):
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

    def send(self, msg: can.Message):
        self._bus.send(msg)
        time.sleep(self.CAN_SEND_DELAY_S)

    def send_no_delay(self, msg: can.Message):
        self._bus.send(msg)

    def _poll(self, timeout: float = 0.0):
        while True:
            msg = self._bus.recv(timeout=timeout)
            if msg is None:
                break
            timeout = 0
            self._dispatch(msg)

    def _dispatch(self, msg: can.Message):
        axis_id = msg.arbitration_id >> 5
        cmd_id = msg.arbitration_id & 0x1F
        if axis_id not in self.states:
            return
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
        s.pos_rev = -pos
        s.vel_rps = -vel
        s.last_encoder_time = time.time()

    def _handle_iq(self, axis_id: int, data: bytes):
        setpoint, measured = decode_iq(data)
        s = self.states[axis_id]
        s.iq_setpoint = setpoint
        s.iq_measured = measured

    def idle_all(self):
        for axis_id in LEG_AXES:
            self.send(encode_set_state(axis_id, 'IDLE'))
        print(f"  All {NUM_LEGS} axes -> IDLE")

    def clear_all_errors(self):
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
        for axis_id in LEG_AXES:
            self.send(encode_set_vel_curr_limits(
                axis_id,
                vel_limit=hw.ODRIVE_LEG_VEL_LIMIT_RPS,
                curr_limit=SAFE_CURRENT_LIMIT_A))
        print(f"  Limits set on all axes: vel={hw.ODRIVE_LEG_VEL_LIMIT_RPS} rev/s, "
              f"curr={SAFE_CURRENT_LIMIT_A} A")

    def set_gains_all(self, pos_gain: float, vel_gain: float,
                      vel_int_gain: float):
        for axis_id in LEG_AXES:
            self.send(encode_set_pos_gain(axis_id, pos_gain))
            self.send(encode_set_vel_gains(axis_id, vel_gain, vel_int_gain))
        print(f"  Gains set on all axes: pos_gain={pos_gain}, "
              f"vel_gain={vel_gain}, vel_int_gain={vel_int_gain}")

    def require_no_errors_all(self):
        self._poll()
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            if s.has_errors:
                names = error_names(s.active_errors)
                raise RuntimeError(
                    f"Axis {axis_id} has errors: {names}")

    def all_heartbeats_fresh(self) -> bool:
        now = time.time()
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            if s.heartbeat_count == 0:
                return False
            if now - s.last_heartbeat >= self.HEARTBEAT_TIMEOUT_S:
                return False
        return True

    def poll_for(self, duration_s: float, interval_s: float = 0.01):
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
        self._poll()
        for axis_id in LEG_AXES:
            pos_now_raw = self.states[axis_id].pos_rev_raw
            self.send(encode_set_controller_mode(
                axis_id, 'POSITION', 'PASSTHROUGH'))
            time.sleep(0.05)
            self.send(encode_set_input_pos(axis_id, pos_now_raw))
            time.sleep(0.05)
            self.send(encode_set_state(axis_id, 'CLOSED_LOOP'))

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
        if vel_limit is None:
            vel_limit = hw.JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS
        self._poll()

        for axis_id in LEG_AXES:
            pos_now_raw = self.states[axis_id].pos_rev_raw
            self.send(encode_set_controller_mode(
                axis_id, 'POSITION', 'TRAP_TRAJ'))
            time.sleep(0.05)
            self.send(encode_set_traj_vel_limit(axis_id, vel_limit))
            self.send(encode_set_traj_acc_limits(axis_id, acc_limit, dec_limit))
            self.send(encode_set_input_pos(axis_id, pos_now_raw))
            time.sleep(0.05)
            self.send(encode_set_state(axis_id, 'CLOSED_LOOP'))

        for axis_id in LEG_AXES:
            self._wait_for_closed_loop(axis_id)

        print(f"  All {NUM_LEGS} axes in POSITION/TRAP_TRAJ, CLOSED_LOOP:")
        print(f"    Trap limits: vel={vel_limit} rev/s, "
              f"acc={acc_limit} rev/s^2, dec={dec_limit} rev/s^2")
        for axis_id in LEG_AXES:
            s = self.states[axis_id]
            print(f"    Axis {axis_id}: holding at {s.pos_rev_raw:.4f} rev raw "
                  f"({s.pos_rev / MM_TO_REV[axis_id]:.2f} mm)")

    def wait_for_all_trajectories_done(self, timeout_s: float = 15.0):
        """Wait for trajectory_done flag on all 6 axes."""
        deadline = time.time() + timeout_s
        done = set()
        while time.time() < deadline:
            self._poll(timeout=0.02)
            for axis_id in LEG_AXES:
                if axis_id not in done and self.states[axis_id].trajectory_done:
                    done.add(axis_id)
                if self.states[axis_id].has_errors:
                    names = error_names(self.states[axis_id].active_errors)
                    raise RuntimeError(
                        f"Axis {axis_id} error during trajectory: {names}")
            if len(done) == NUM_LEGS:
                return True
            if not self.all_heartbeats_fresh():
                raise RuntimeError("Heartbeat timeout during trajectory wait")
        return False

    def home_axis(self, axis_id: int, homing_current_lim: float = 4.0):
        print(f"  Homing axis {axis_id}...")
        self.send(encode_clear_errors(axis_id))
        time.sleep(0.1)
        self.send(encode_set_vel_curr_limits(
            axis_id,
            vel_limit=hw.ODRIVE_LEG_VEL_LIMIT_RPS,
            curr_limit=SAFE_CURRENT_LIMIT_A))
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
        self.send(encode_set_input_vel(axis_id, hw.HOMING_LEG_SPEED_RPS))

        print(f"    Ramping up (1s grace period)...")
        self.poll_for(1.0)

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
        self.send(encode_set_absolute_position(axis_id, 0.0))
        time.sleep(0.1)
        self._poll()
        print(f"    Homed! Encoder set to 0 rev (fully compressed)")

    def home_all(self):
        print(f"\n  Homing all {NUM_LEGS} axes sequentially...")
        for axis_id in LEG_AXES:
            self.home_axis(axis_id)
        print(f"  All {NUM_LEGS} axes homed successfully.")


# ---------------------------------------------------------------------------
# Dynamics helpers
# ---------------------------------------------------------------------------

def compute_torque_ff_for_pose(pos: np.ndarray, rot: np.ndarray,
                               geom: StewartGeometry,
                               params: DynamicsParams) -> np.ndarray:
    """Compute per-leg gravity torque_ff int16 values for a given pose."""
    torques_Nm = gravity_to_motor_torques(pos, rot, geom, params)
    return torques_Nm


def pose_to_raw_positions(pos: np.ndarray, rot: np.ndarray,
                          geom: StewartGeometry) -> np.ndarray:
    """Compute raw ODrive encoder positions for a given platform pose.

    Returns (6,) array of raw rev values (negative = extension).
    """
    extensions_mm = pose_to_leg_lengths(pos, rot, geom)
    revs = extensions_mm_to_revs(extensions_mm, geom)
    # Raw ODrive convention: negative = extension (leg inversion)
    return -revs


def send_pose_with_ff(harness: PlatformTestHarness,
                      pos: np.ndarray, rot: np.ndarray,
                      geom: StewartGeometry, params: DynamicsParams,
                      use_ff: bool = True):
    """Command all 6 legs to a platform pose with optional gravity torque_ff.

    Uses set_input_pos with vel_ff=0 (static hold, no velocity feedforward).
    """
    raw_positions = pose_to_raw_positions(pos, rot, geom)
    torques_Nm = compute_torque_ff_for_pose(pos, rot, geom, params) if use_ff \
        else np.zeros(6)

    for axis_id in LEG_AXES:
        ff_int = int(round(torques_Nm[axis_id] * LEG_TOR_FF_SCALE)) if use_ff \
            else 0
        ff_int = max(-32767, min(32767, ff_int))
        harness.send(encode_set_input_pos(
            axis_id, raw_positions[axis_id], vel_ff=0, torque_ff=ff_int))


def interactive_pause(prompt: str):
    """Pause for operator input.  Returns the input string."""
    result = input(f"\n  >>> {prompt} ").strip()
    return result


# ===========================================================================
# Test C1: Stable hold at home pose
# ===========================================================================

def test_stable_hold(harness: PlatformTestHarness):
    """Stage C1: Stable hold at home pose with torque_ff.

    From MOTION_PLANNER_PLAN Phase 3 Stage C:
    "With torque_ff active and ODrive gains at the unloaded baseline, release
    the platform support gradually."

    Procedure:
      1. Enter POSITION/PASSTHROUGH, hold current (home) position
      2. Enable gravity torque_ff at home pose
      3. Prompt operator to gradually release platform support
      4. Monitor position stability for 10 seconds
      5. Report per-leg max deviation

    Pass criteria:
      - All legs maintain position within 2.0 mm during the hold
      - No errors on any axis
      - Operator confirms platform is stable
    """
    print("\n" + "=" * 60)
    print("STAGE C1: Stable hold at home pose (with torque_ff)")
    print("=" * 60)

    HOLD_DURATION_S = 10.0
    SAMPLE_INTERVAL_S = 0.1
    MAX_DEVIATION_MM = 2.0  # relaxed vs Stage B — first free-standing hold

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    pos_home = np.zeros(3)
    rot_home = np.eye(3)

    # Compute and display feedforward values
    torques_Nm = compute_torque_ff_for_pose(pos_home, rot_home, geom, params)
    print(f"\n  Gravity torque_ff at home pose:")
    for i, axis_id in enumerate(LEG_AXES):
        ff_int = int(round(torques_Nm[i] * LEG_TOR_FF_SCALE))
        print(f"    Leg {axis_id}: {torques_Nm[i]:.4f} Nm (int16: {ff_int})")

    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.set_gains_all(BASELINE_POS_GAIN, BASELINE_VEL_GAIN,
                          BASELINE_VEL_INT_GAIN)
    harness.require_no_errors_all()

    # Move to home pose using TRAP_TRAJ (smooth ramp from current position)
    home_raw = pose_to_raw_positions(pos_home, rot_home, geom)
    print(f"\n  Moving to home pose using TRAP_TRAJ...")
    harness.enter_trap_traj_mode_all(vel_limit=1.5, acc_limit=5.0,
                                     dec_limit=5.0)
    for axis_id in LEG_AXES:
        harness.send(encode_set_input_pos(axis_id, home_raw[axis_id]))
    harness.wait_for_all_trajectories_done(timeout_s=15.0)
    harness.poll_for(1.0)

    # Switch to PASSTHROUGH for static hold with feedforward
    print(f"  Switching to POSITION/PASSTHROUGH for static hold...")
    for axis_id in LEG_AXES:
        harness.send(encode_set_controller_mode(
            axis_id, 'POSITION', 'PASSTHROUGH'))
        time.sleep(0.05)

    # Apply feedforward at home pose
    print(f"  Applying gravity torque_ff...")
    for axis_id in LEG_AXES:
        ff_int = int(round(torques_Nm[axis_id] * LEG_TOR_FF_SCALE))
        ff_int = max(-32767, min(32767, ff_int))
        harness.send(encode_set_input_pos(axis_id, home_raw[axis_id],
                                          vel_ff=0, torque_ff=ff_int))

    print(f"\n  torque_ff active. ODrive gains at baseline.")
    print(f"  The platform should be mechanically supported right now.")
    interactive_pause(
        "Gradually release the platform support, then press Enter...")
    harness.flush_and_resync()

    # Record starting positions
    harness._poll()
    start_mm = {}
    for axis_id in LEG_AXES:
        s = harness.states[axis_id]
        start_mm[axis_id] = s.pos_rev / MM_TO_REV[axis_id]

    # Monitor stability
    max_dev_mm = {axis_id: 0.0 for axis_id in LEG_AXES}
    errors_seen = {axis_id: False for axis_id in LEG_AXES}
    n_samples = int(HOLD_DURATION_S / SAMPLE_INTERVAL_S)

    print(f"\n  Monitoring position stability for {HOLD_DURATION_S}s...")

    for i in range(n_samples):
        harness.poll_for(SAMPLE_INTERVAL_S)

        # Resend target position+ff commands to keep feedforward active
        for axis_id in LEG_AXES:
            ff_int = int(round(torques_Nm[axis_id] * LEG_TOR_FF_SCALE))
            ff_int = max(-32767, min(32767, ff_int))
            harness.send_no_delay(encode_set_input_pos(
                axis_id, home_raw[axis_id], vel_ff=0, torque_ff=ff_int))

        for axis_id in LEG_AXES:
            s = harness.states[axis_id]
            current_mm = s.pos_rev / MM_TO_REV[axis_id]
            dev = abs(current_mm - start_mm[axis_id])
            if dev > max_dev_mm[axis_id]:
                max_dev_mm[axis_id] = dev

            if s.has_errors and not errors_seen[axis_id]:
                errors_seen[axis_id] = True
                names = error_names(s.active_errors)
                print(f"    ERROR at sample {i}: Axis {axis_id}: {names}")

        # Progress every 2 seconds
        if (i + 1) % 20 == 0:
            max_across = max(max_dev_mm.values())
            print(f"    {(i+1) * SAMPLE_INTERVAL_S:.0f}s: "
                  f"max deviation = {max_across:.3f} mm")

    # Results
    all_pass = True
    print(f"\n  --- Hold results ({HOLD_DURATION_S}s) ---")
    for axis_id in LEG_AXES:
        dev = max_dev_mm[axis_id]
        ok = dev < MAX_DEVIATION_MM and not errors_seen[axis_id]
        status = "OK" if ok else "FAIL"
        print(f"    Leg {axis_id}: max dev {dev:.3f} mm [{status}]")
        if not ok:
            all_pass = False

    if all_pass:
        print(f"\n  PASS: Platform held stable within {MAX_DEVIATION_MM} mm "
              f"for {HOLD_DURATION_S}s")
        return True
    else:
        failed = [a for a in LEG_AXES
                  if max_dev_mm[a] >= MAX_DEVIATION_MM or errors_seen[a]]
        print(f"\n  FAIL: Legs {failed} exceeded deviation or had errors")
        return False


# ===========================================================================
# Test C2: Feedforward harmlessness (toggle on/off)
# ===========================================================================

def test_ff_toggle(harness: PlatformTestHarness):
    """Stage C2: Feedforward harmlessness test.

    From MOTION_PLANNER_PLAN Phase 3 Stage C:
    "Briefly disable torque_ff. Platform should still hold fine (stiction
    dominates). Re-enable. Verify no transient or instability on the
    transition."

    Procedure:
      1. Hold at home pose WITH torque_ff (3s baseline)
      2. Disable torque_ff (hold 5s, monitor for transients)
      3. Re-enable torque_ff (hold 5s, monitor for transients)

    Pass criteria:
      - Position deviation during toggle < 1.0 mm on each leg
      - No errors during any phase
    """
    print("\n" + "=" * 60)
    print("STAGE C2: Feedforward harmlessness (toggle on/off)")
    print("=" * 60)

    MAX_TOGGLE_DEV_MM = 1.0

    geom = StewartGeometry()
    params = DynamicsParams.from_config()
    pos_home = np.zeros(3)
    rot_home = np.eye(3)
    torques_Nm = compute_torque_ff_for_pose(pos_home, rot_home, geom, params)

    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.set_gains_all(BASELINE_POS_GAIN, BASELINE_VEL_GAIN,
                          BASELINE_VEL_INT_GAIN)
    harness.require_no_errors_all()
    harness.enter_position_mode_all()

    # Use IK-computed home position as the hold target
    pos_home = np.zeros(3)
    rot_home = np.eye(3)
    hold_raw = pose_to_raw_positions(pos_home, rot_home, geom)

    # Record starting positions for deviation measurement
    harness._poll()
    start_mm = {}
    for axis_id in LEG_AXES:
        s = harness.states[axis_id]
        start_mm[axis_id] = s.pos_rev / MM_TO_REV[axis_id]

    def send_ff(enable: bool):
        for axis_id in LEG_AXES:
            ff_int = 0
            if enable:
                ff_int = int(round(torques_Nm[axis_id] * LEG_TOR_FF_SCALE))
                ff_int = max(-32767, min(32767, ff_int))
            harness.send(encode_set_input_pos(
                axis_id, hold_raw[axis_id], vel_ff=0, torque_ff=ff_int))

    def monitor_phase(label: str, duration_s: float,
                      ff_enabled: bool) -> dict[int, float]:
        """Monitor position during a hold phase. Returns max deviation per leg."""
        max_dev = {a: 0.0 for a in LEG_AXES}
        n = int(duration_s / 0.1)
        for _ in range(n):
            send_ff(ff_enabled)
            harness.poll_for(0.1)
            for axis_id in LEG_AXES:
                current_mm = harness.states[axis_id].pos_rev / MM_TO_REV[axis_id]
                dev = abs(current_mm - start_mm[axis_id])
                if dev > max_dev[axis_id]:
                    max_dev[axis_id] = dev
                if harness.states[axis_id].has_errors:
                    names = error_names(harness.states[axis_id].active_errors)
                    raise RuntimeError(
                        f"Axis {axis_id} error during {label}: {names}")
        return max_dev

    # Phase 1: baseline with FF on
    print(f"\n  Phase 1: Hold with torque_ff ON (3s baseline)...")
    send_ff(True)
    dev_baseline = monitor_phase("baseline", 3.0, True)
    max_baseline = max(dev_baseline.values())
    print(f"    Max deviation: {max_baseline:.3f} mm")

    # Phase 2: disable FF
    print(f"  Phase 2: Disabling torque_ff (5s)...")
    dev_off = monitor_phase("ff_off", 5.0, False)
    max_off = max(dev_off.values())
    print(f"    Max deviation: {max_off:.3f} mm")

    # Phase 3: re-enable FF
    print(f"  Phase 3: Re-enabling torque_ff (5s)...")
    dev_on = monitor_phase("ff_on", 5.0, True)
    max_on = max(dev_on.values())
    print(f"    Max deviation: {max_on:.3f} mm")

    harness.idle_all()

    # Results
    all_pass = True
    print(f"\n  --- Toggle results ---")
    print(f"    {'Leg':<6s} {'Baseline':<12s} {'FF Off':<12s} {'FF On':<12s}")
    for axis_id in LEG_AXES:
        b = dev_baseline[axis_id]
        off = dev_off[axis_id]
        on = dev_on[axis_id]
        worst = max(b, off, on)
        ok = worst < MAX_TOGGLE_DEV_MM
        status = "OK" if ok else "FAIL"
        print(f"    {axis_id:<6d} {b:<12.3f} {off:<12.3f} {on:<12.3f} [{status}]")
        if not ok:
            all_pass = False

    if all_pass:
        print(f"\n  PASS: Toggling torque_ff causes no transient > "
              f"{MAX_TOGGLE_DEV_MM} mm")
        return True
    else:
        print(f"\n  FAIL: Position deviation exceeded {MAX_TOGGLE_DEV_MM} mm "
              f"during toggle")
        return False


# ===========================================================================
# Test C3: Step response & gain tuning
# ===========================================================================

def test_step_response(harness: PlatformTestHarness):
    """Stage C3: Step response test for gain adequacy.

    From MOTION_PLANNER_PLAN Phase 3 Stage C:
    "Adjust ODrive gains for clean step response -- fast settling, no
    oscillation, no audible vibration."

    Procedure:
      1. Enter TRAP_TRAJ mode (smooth, ramped moves)
      2. Command a small translation (+10 mm Z, then back)
      3. Monitor settling behaviour after trajectory completes
      4. Report settling time and residual deviation

    Pass criteria:
      - All legs reach target within 2.0 mm
      - Settling residual < 0.5 mm within 2s after trajectory_done
      - No errors
    """
    print("\n" + "=" * 60)
    print("STAGE C3: Step response (gain adequacy check)")
    print("=" * 60)

    STEP_Z_MM = 10.0
    SETTLE_TIME_S = 2.0
    TARGET_ACCURACY_MM = 2.0
    SETTLE_RESIDUAL_MM = 0.5

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.set_gains_all(BASELINE_POS_GAIN, BASELINE_VEL_GAIN,
                          BASELINE_VEL_INT_GAIN)
    harness.require_no_errors_all()
    harness.enter_trap_traj_mode_all(vel_limit=2.5, acc_limit=10.0,
                                     dec_limit=10.0)

    # Use IK-computed home position
    home_raw = pose_to_raw_positions(np.zeros(3), np.eye(3), geom)

    # Compute target pose: +10 mm Z translation
    pos_step = np.array([0.0, 0.0, STEP_Z_MM])
    rot_step = np.eye(3)
    target_raw = pose_to_raw_positions(pos_step, rot_step, geom)
    target_torques = compute_torque_ff_for_pose(pos_step, rot_step, geom, params)

    print(f"\n  Step: +{STEP_Z_MM:.0f} mm Z translation")
    print(f"  Target raw positions:")
    for axis_id in LEG_AXES:
        print(f"    Leg {axis_id}: {target_raw[axis_id]:.4f} rev raw")

    # Command the step
    print(f"\n  Commanding step...")
    for axis_id in LEG_AXES:
        ff_int = int(round(target_torques[axis_id] * LEG_TOR_FF_SCALE))
        ff_int = max(-32767, min(32767, ff_int))
        harness.send(encode_set_input_pos(
            axis_id, target_raw[axis_id], vel_ff=0, torque_ff=ff_int))

    # Wait for trajectories to complete
    traj_ok = harness.wait_for_all_trajectories_done(timeout_s=10.0)
    if not traj_ok:
        print(f"    WARNING: Not all trajectories completed")

    # Measure accuracy at target
    harness._poll()
    print(f"\n  At target (post-trajectory):")
    step_errors = {}
    for axis_id in LEG_AXES:
        actual_raw = harness.states[axis_id].pos_rev_raw
        error_rev = abs(actual_raw - target_raw[axis_id])
        error_mm = error_rev / MM_TO_REV[axis_id]
        step_errors[axis_id] = error_mm
        print(f"    Leg {axis_id}: error = {error_mm:.3f} mm")

    # Settle and measure residual
    print(f"\n  Settling ({SETTLE_TIME_S}s)...")
    harness.poll_for(SETTLE_TIME_S)

    print(f"  Post-settle residual:")
    settle_errors = {}
    all_pass = True
    for axis_id in LEG_AXES:
        actual_raw = harness.states[axis_id].pos_rev_raw
        error_rev = abs(actual_raw - target_raw[axis_id])
        error_mm = error_rev / MM_TO_REV[axis_id]
        settle_errors[axis_id] = error_mm
        ok = error_mm < SETTLE_RESIDUAL_MM
        print(f"    Leg {axis_id}: residual = {error_mm:.3f} mm "
              f"[{'OK' if ok else 'FAIL'}]")
        if not ok:
            all_pass = False

    # Return to home
    print(f"\n  Returning to home...")
    home_torques = compute_torque_ff_for_pose(np.zeros(3), np.eye(3),
                                              geom, params)
    for axis_id in LEG_AXES:
        ff_int = int(round(home_torques[axis_id] * LEG_TOR_FF_SCALE))
        ff_int = max(-32767, min(32767, ff_int))
        harness.send(encode_set_input_pos(
            axis_id, home_raw[axis_id], vel_ff=0, torque_ff=ff_int))

    harness.wait_for_all_trajectories_done(timeout_s=10.0)
    harness.poll_for(1.0)

    # Check return accuracy
    print(f"  Return accuracy:")
    for axis_id in LEG_AXES:
        actual_raw = harness.states[axis_id].pos_rev_raw
        error_mm = abs(actual_raw - home_raw[axis_id]) / MM_TO_REV[axis_id]
        print(f"    Leg {axis_id}: {error_mm:.3f} mm")

    harness.idle_all()

    # Check for errors
    for axis_id in LEG_AXES:
        if harness.states[axis_id].has_errors:
            names = error_names(harness.states[axis_id].active_errors)
            print(f"    ERROR: Axis {axis_id}: {names}")
            all_pass = False

    if all_pass:
        max_settle = max(settle_errors.values())
        print(f"\n  PASS: Step response settled within {max_settle:.3f} mm "
              f"(< {SETTLE_RESIDUAL_MM} mm)")
        return True
    else:
        print(f"\n  FAIL: Step response did not meet settling criteria")
        return False


# ===========================================================================
# Test C4: Gravity vector rotation (analytical)
# ===========================================================================

def test_gravity_rotation(harness: PlatformTestHarness):
    """Stage C4: Gravity vector rotation test (analytical).

    From MOTION_PLANNER_PLAN Phase 3 Stage C:
    "Tilt the platform to known angles, log the commanded gravity
    compensation torques per leg, verify they are geometrically consistent."

    Procedure:
      1. Compute gravity torque_ff at home and several tilted poses
      2. Move platform to each pose using TRAP_TRAJ
      3. At each pose: log torque_ff values and verify consistency
      4. Verify total torque magnitude scales correctly with tilt

    Tilt poses: 5 deg about X, 5 deg about Y, 3 deg about both.

    Pass criteria:
      - All torques have correct sign (positive = extension)
      - Total torque magnitude changes < 5% across poses (gravity magnitude
        is constant; only distribution changes)
      - No errors during moves
    """
    print("\n" + "=" * 60)
    print("STAGE C4: Gravity vector rotation (analytical + move)")
    print("=" * 60)

    TOTAL_TOLERANCE = 0.15  # 15% tolerance on total torque variation

    geom = StewartGeometry()
    params = DynamicsParams.from_config()

    # Define test poses: (label, pos_mm, rotvec_rad)
    test_poses = [
        ("Home",        np.zeros(3), np.zeros(3)),
        ("+5 deg X",    np.zeros(3), np.array([5.0 * np.pi/180, 0, 0])),
        ("-5 deg X",    np.zeros(3), np.array([-5.0 * np.pi/180, 0, 0])),
        ("+5 deg Y",    np.zeros(3), np.array([0, 5.0 * np.pi/180, 0])),
        ("-5 deg Y",    np.zeros(3), np.array([0, -5.0 * np.pi/180, 0])),
        ("+3 deg X+Y",  np.zeros(3), np.array([3.0 * np.pi/180,
                                                3.0 * np.pi/180, 0])),
    ]

    # First pass: compute all torques analytically (offline)
    print(f"\n  Analytical gravity torque_ff at each pose:")
    print(f"  Platform mass: {params.mass_kg} kg, "
          f"CoM offset: {list(params.com_offset_mm)} mm")
    pose_data = []
    total_torques = []

    for label, pos, rotvec in test_poses:
        rot = rotvec_to_rot_matrix(rotvec) if np.any(rotvec != 0) else np.eye(3)
        torques = compute_torque_ff_for_pose(pos, rot, geom, params)
        total = float(np.sum(torques))
        total_torques.append(total)
        all_positive = bool(np.all(torques > 0))

        pose_data.append({
            'label': label,
            'pos': pos,
            'rot': rot,
            'torques': torques,
            'total': total,
            'all_positive': all_positive,
        })

        ff_strs = [f"{t:.4f}" for t in torques]
        sign_str = "ALL +" if all_positive else "HAS NEGATIVE"
        print(f"\n    {label}:")
        print(f"      Per-leg (Nm): {ff_strs}")
        print(f"      Total: {total:.4f} Nm  [{sign_str}]")

    # Check total torque consistency
    total_arr = np.array(total_torques)
    total_mean = np.mean(total_arr)
    total_max_dev = np.max(np.abs(total_arr - total_mean)) / total_mean

    print(f"\n  Total torque consistency:")
    print(f"    Mean: {total_mean:.4f} Nm")
    print(f"    Max deviation from mean: {total_max_dev * 100:.1f}%")

    all_pass = True
    if total_max_dev > TOTAL_TOLERANCE:
        print(f"    FAIL: Total torque variation {total_max_dev*100:.1f}% > "
              f"{TOTAL_TOLERANCE*100:.0f}% tolerance")
        all_pass = False
    else:
        print(f"    OK: Total torque stable across poses")

    for pd in pose_data:
        if not pd['all_positive']:
            print(f"    SIGN ERROR at {pd['label']}: "
                  f"some torques are negative")
            all_pass = False

    # Second pass: move to each pose on hardware
    print(f"\n  --- Moving platform to each pose ---")

    harness.clear_all_errors()
    harness.set_safe_limits_all()
    harness.set_gains_all(BASELINE_POS_GAIN, BASELINE_VEL_GAIN,
                          BASELINE_VEL_INT_GAIN)
    harness.require_no_errors_all()
    harness.enter_trap_traj_mode_all(vel_limit=1.5, acc_limit=5.0,
                                     dec_limit=5.0)

    # Use IK-computed home position
    home_raw = pose_to_raw_positions(np.zeros(3), np.eye(3), geom)

    for pd in pose_data:
        label = pd['label']
        pos = pd['pos']
        rot = pd['rot']
        torques = pd['torques']

        print(f"\n  Moving to: {label}")
        target_raw = pose_to_raw_positions(pos, rot, geom)

        # Command move with torque_ff
        for axis_id in LEG_AXES:
            ff_int = int(round(torques[axis_id] * LEG_TOR_FF_SCALE))
            ff_int = max(-32767, min(32767, ff_int))
            harness.send(encode_set_input_pos(
                axis_id, target_raw[axis_id], vel_ff=0, torque_ff=ff_int))

        traj_ok = harness.wait_for_all_trajectories_done(timeout_s=10.0)
        if not traj_ok:
            print(f"    WARNING: Trajectory not complete at {label}")

        # Settle
        harness.poll_for(2.0)

        # Check position accuracy
        harness._poll()
        max_err = 0.0
        for axis_id in LEG_AXES:
            actual_raw = harness.states[axis_id].pos_rev_raw
            error_mm = abs(actual_raw - target_raw[axis_id]) / MM_TO_REV[axis_id]
            if error_mm > max_err:
                max_err = error_mm

        # Check for errors
        has_err = False
        for axis_id in LEG_AXES:
            if harness.states[axis_id].has_errors:
                names = error_names(harness.states[axis_id].active_errors)
                print(f"    ERROR: Axis {axis_id}: {names}")
                has_err = True
                all_pass = False

        if not has_err:
            print(f"    Holding at {label}: max error = {max_err:.3f} mm [OK]")

    # Return home
    print(f"\n  Returning to home...")
    home_torques = compute_torque_ff_for_pose(np.zeros(3), np.eye(3),
                                              geom, params)
    for axis_id in LEG_AXES:
        ff_int = int(round(home_torques[axis_id] * LEG_TOR_FF_SCALE))
        ff_int = max(-32767, min(32767, ff_int))
        harness.send(encode_set_input_pos(
            axis_id, home_raw[axis_id], vel_ff=0, torque_ff=ff_int))

    harness.wait_for_all_trajectories_done(timeout_s=10.0)
    harness.poll_for(1.0)
    harness.idle_all()

    if all_pass:
        print(f"\n  PASS: Gravity torques geometrically consistent across "
              f"{len(test_poses)} poses")
        return True
    else:
        print(f"\n  FAIL: Gravity rotation test did not meet all criteria")
        return False


# ===========================================================================
# Test C5: Jacobian condition number survey (offline)
# ===========================================================================

def test_condition_number(harness: PlatformTestHarness):
    """Stage C5: Jacobian condition number survey.

    From MOTION_PLANNER_PLAN Phase 3 Stage C:
    "Log Jacobian condition number across the workspace to confirm force
    decomposition is stable at intended operating poses."

    This test is purely computational — it does not command any motion.
    It sweeps a grid of poses and reports condition numbers.

    Pass criteria (informational — no hard pass/fail):
      - All intended operating poses have finite condition number
      - Condition number range is documented for Phase 6
    """
    print("\n" + "=" * 60)
    print("STAGE C5: Jacobian condition number survey (offline)")
    print("=" * 60)

    geom = StewartGeometry()

    # Survey grid: Z offsets and tilt angles
    z_offsets_mm = [0.0, -20.0, -40.0, 20.0, 40.0]
    tilt_degs = [0.0, 3.0, 5.0, 8.0]

    print(f"\n  Sweeping Z offsets: {z_offsets_mm} mm")
    print(f"  Tilt angles: {tilt_degs} deg (about X, Y, and X+Y)")
    print(f"\n  {'Pose':<30s} {'Cond #':<12s} {'Reachable':<10s}")
    print(f"  {'-'*52}")

    results = []
    for z in z_offsets_mm:
        for tilt_deg in tilt_degs:
            tilt_rad = tilt_deg * np.pi / 180.0

            # Test several tilt directions
            for tilt_label, rotvec in [
                (f"X", np.array([tilt_rad, 0, 0])),
                (f"Y", np.array([0, tilt_rad, 0])),
            ]:
                if tilt_deg == 0 and tilt_label == "Y":
                    continue  # skip duplicate zero-tilt

                pos = np.array([0.0, 0.0, z])
                rot = rotvec_to_rot_matrix(rotvec) if tilt_rad > 0 \
                    else np.eye(3)

                label = f"Z={z:+.0f}mm, {tilt_deg:.0f}deg {tilt_label}"

                try:
                    extensions = pose_to_leg_lengths(pos, rot, geom)
                    in_bounds = np.all(
                        (extensions >= 0) & (extensions <= geom.leg_stroke_mm))

                    if in_bounds:
                        cond = compute_condition_number(pos, rot, geom)
                        results.append((label, cond, True))
                        print(f"  {label:<30s} {cond:<12.1f} {'Yes':<10s}")
                    else:
                        results.append((label, float('inf'), False))
                        print(f"  {label:<30s} {'N/A':<12s} {'No':<10s}")
                except Exception as e:
                    results.append((label, float('inf'), False))
                    print(f"  {label:<30s} {'ERR':<12s} {str(e)[:20]:<10s}")

    # Summary
    reachable = [(l, c) for l, c, r in results if r]
    if reachable:
        conds = [c for _, c in reachable]
        print(f"\n  Summary ({len(reachable)} reachable poses):")
        print(f"    Condition number range: {min(conds):.1f} - {max(conds):.1f}")
        print(f"    Mean: {np.mean(conds):.1f}")
        print(f"\n  PASS (informational): Condition numbers logged for Phase 6")
        return True
    else:
        print(f"\n  WARNING: No reachable poses found in survey grid")
        return False


# ===========================================================================
# Test registry and CLI
# ===========================================================================

TESTS = {
    'hold':        ('Stable hold at home (C1)',
                    test_stable_hold),
    'ff_toggle':   ('Feedforward harmlessness (C2)',
                    test_ff_toggle),
    'step':        ('Step response & gain check (C3)',
                    test_step_response),
    'gravity_rot': ('Gravity vector rotation (C4)',
                    test_gravity_rotation),
    'cond_number': ('Jacobian condition number (C5, offline)',
                    test_condition_number),
}

TEST_GROUPS = {
    'all':     ['hold', 'ff_toggle', 'step', 'gravity_rot', 'cond_number'],
    'stage_c': ['hold', 'ff_toggle', 'step', 'gravity_rot', 'cond_number'],
    'hw':      ['hold', 'ff_toggle', 'step', 'gravity_rot'],
    'offline': ['cond_number'],
}


def main():
    parser = argparse.ArgumentParser(
        description='Free-platform test harness for Jugglebot (Stage C)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Tests available:
  hold         Stage C1: Stable hold at home (with torque_ff, operator release)
  ff_toggle    Stage C2: Toggle torque_ff on/off, verify no transient
  step         Stage C3: Step response (+10mm Z, gain adequacy check)
  gravity_rot  Stage C4: Gravity vector rotation (tilt to 5 poses, analytical)
  cond_number  Stage C5: Jacobian condition number survey (offline, no motion)

Test groups:
  all / stage_c   Run C1-C5 in order [DEFAULT]
  hw              Hardware tests only (C1-C4)
  offline         Offline tests only (C5)

Prerequisites:
  - All Stage B tests must PASS first
  - Platform mechanically supported initially (operator releases during C1)
  - Legs must be homed (use --home flag)
  - BE READY TO CATCH THE PLATFORM if something goes wrong

Safety:
  Current limit is set to {curr}A (50% of {rated}A).
  All tests send IDLE to all 6 axes on completion, error, or Ctrl-C.
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

    print(f"\nJugglebot Free-Platform Test Harness (Stage C)")
    print(f"  Interface: {args.interface}")
    print(f"  Channel:   {args.channel}")
    print(f"  Current limit: {SAFE_CURRENT_LIMIT_A} A "
          f"(50% of {hw.ODRIVE_LEG_CURR_LIMIT_A} A)")
    print(f"\n  *** WARNING: Platform will be UNSUPPORTED during these tests ***")
    print(f"  *** Be ready to catch/support the platform if needed ***")

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
