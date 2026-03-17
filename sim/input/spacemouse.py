"""SpaceMouse input — maps 3Dconnexion SpaceMouse axes to a target pose.

Reads raw SpaceMouse state via ``pyspacemouse`` and converts it to a 6-DoF
target pose ``[x, y, z, rx, ry, rz]`` (mm, rad) using sensitivity multipliers
from ``hardware_config.yaml``.

Requires ``libhidapi-dev`` and USB HID access (Linux desktop, not Jetson —
the MuJoCo viewer does not work on Jetson due to Tegra GLX incompatibility).

Usage::

    inp = SpaceMouseInput()
    if inp.connected:
        pose = inp.read()  # (6,) target pose
"""

from __future__ import annotations

import math
import os
import threading
import time

import numpy as np
import yaml


def _load_spacemouse_config() -> dict:
    """Load spacemouse configuration from hardware_config.yaml."""
    candidates = [
        os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'hardware_config.yaml'),
        os.path.join('config', 'hardware_config.yaml'),
    ]
    for path in candidates:
        path = os.path.abspath(path)
        if os.path.isfile(path):
            with open(path) as f:
                config = yaml.safe_load(f)
            sm = config['jugglebot_spacemouse']
            op = config['jugglebot_operational']
            return {
                'xy_mult_mm': sm['xy_mult_mm'],
                'z_mult_mm': sm['z_mult_mm'],
                'pitch_roll_mult_deg': sm['pitch_roll_mult_deg'],
                'yaw_mult_deg': sm['yaw_mult_deg'],
                'default_active_z_mm': op['default_active_z_mm'],
            }
    raise FileNotFoundError("Cannot find config/hardware_config.yaml")


def _rotvec_from_euler(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """Convert individual Euler angles to a rotation vector (Rodrigues).

    Matches the production code's composition order: yaw * roll * pitch.
    Uses small-angle approximation for the rotation vector since the
    SpaceMouse tilt range is <=30 deg where the error is negligible.
    """
    # Build rotation matrices for each axis
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)

    # R_roll (about Y), R_pitch (about X), R_yaw (about Z)
    # Match production: q_roll = from_rotation_vector([0, roll, 0])
    #                   q_pitch = from_rotation_vector([pitch, 0, 0])
    #                   q_yaw = from_rotation_vector([0, 0, yaw])
    #                   q = q_yaw * q_roll * q_pitch
    R_roll = np.array([
        [cr, 0, sr],
        [0, 1, 0],
        [-sr, 0, cr],
    ])
    R_pitch = np.array([
        [1, 0, 0],
        [0, cp, -sp],
        [0, sp, cp],
    ])
    R_yaw = np.array([
        [cy, -sy, 0],
        [sy, cy, 0],
        [0, 0, 1],
    ])

    R = R_yaw @ R_roll @ R_pitch

    # Extract rotation vector via Rodrigues inverse
    angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0)))
    if angle < 1e-10:
        return np.zeros(3)
    # Skew-symmetric extraction
    rx = R[2, 1] - R[1, 2]
    ry = R[0, 2] - R[2, 0]
    rz = R[1, 0] - R[0, 1]
    k = angle / (2.0 * math.sin(angle))
    return np.array([rx * k, ry * k, rz * k])


class SpaceMouseInput:
    """Reads a 3Dconnexion SpaceMouse and produces a target pose.

    The target pose is an absolute pose ``[x, y, z, rx, ry, rz]`` (mm, rad)
    offset from home.  SpaceMouse at rest → ``[0, 0, z_offset, 0, 0, 0]``
    where ``z_offset = default_active_z_mm`` from hardware config.

    Parameters
    ----------
    poll_rate_hz : float
        How often to poll the SpaceMouse hardware (default 100 Hz).
    max_retries : int
        Connection attempts before giving up.
    """

    def __init__(self, poll_rate_hz: float = 100.0, max_retries: int = 3):
        self._config = _load_spacemouse_config()
        self._poll_interval = 1.0 / poll_rate_hz

        self._target_pose = np.array([
            0.0, 0.0, self._config['default_active_z_mm'],
            0.0, 0.0, 0.0,
        ])
        self._lock = threading.Lock()
        self._connected = False
        self._running = False
        self._thread: threading.Thread | None = None

        # Attempt connection
        import pyspacemouse
        for attempt in range(1, max_retries + 1):
            self._connected = pyspacemouse.open()
            if self._connected:
                print(f"SpaceMouse connected (attempt {attempt}/{max_retries})")
                break
            if attempt < max_retries:
                print(f"SpaceMouse not found (attempt {attempt}/{max_retries}), "
                      f"retrying in 2s...")
                time.sleep(2.0)

        if not self._connected:
            print("SpaceMouse: failed to connect — input disabled")
            return

        # Start background polling thread
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    @property
    def connected(self) -> bool:
        return self._connected

    def read(self) -> np.ndarray:
        """Return the current target pose (6,) — thread-safe snapshot."""
        with self._lock:
            return self._target_pose.copy()

    def close(self) -> None:
        """Stop polling and close the SpaceMouse connection."""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._connected:
            import pyspacemouse
            pyspacemouse.close()
            self._connected = False

    def _poll_loop(self) -> None:
        """Background thread: read SpaceMouse at poll_rate_hz."""
        import pyspacemouse

        xy_mult = self._config['xy_mult_mm']
        z_mult = self._config['z_mult_mm']
        pr_mult = self._config['pitch_roll_mult_deg']
        yaw_mult = self._config['yaw_mult_deg']
        z_offset = self._config['default_active_z_mm']

        while self._running:
            state = pyspacemouse.read()
            if state is not None:
                # Position (mm)
                x = state.x * xy_mult
                y = state.y * xy_mult
                z = state.z * z_mult + z_offset

                # Orientation (match production: negate pitch and yaw)
                roll_rad = math.radians(state.roll * pr_mult)
                pitch_rad = math.radians(-state.pitch * pr_mult)
                yaw_rad = math.radians(-state.yaw * yaw_mult)

                rot_vec = _rotvec_from_euler(roll_rad, pitch_rad, yaw_rad)

                pose = np.array([x, y, z, rot_vec[0], rot_vec[1], rot_vec[2]])

                with self._lock:
                    self._target_pose = pose

            time.sleep(self._poll_interval)
