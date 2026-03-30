"""Keyboard input — alternative to SpaceMouse.

Uses MuJoCo viewer's key callback to move the platform interactively.
Works on Windows (and Linux with a working GLX stack — not Jetson).

The key_callback must be passed to ``mujoco.viewer.launch_passive()`` at
construction time — it cannot be installed after the viewer is created.
Use ``KeyboardInput.key_callback`` as the callback argument.

Each key press moves the target by a fixed step.  The step size depends
on sensitivity and the control rate: ``pos_rate * sensitivity * dt`` per
callback.  At 1.0x sensitivity and 50 Hz, each press moves ~3 mm / 0.6°.

Note: MuJoCo's C++ simulate only forwards GLFW_PRESS to the Python
key_callback (not GLFW_REPEAT or GLFW_RELEASE), so holding a key
produces only one movement step.  Tap repeatedly for larger moves, or
increase sensitivity with Numpad +.

Controls (displayed in terminal on startup):
    Arrow Up/Down    — Y forward/backward
    Arrow Left/Right — X left/right
    Page Up/Down     — Z up/down
    Numpad 8/2       — Pitch forward/backward
    Numpad 4/6       — Roll left/right
    Numpad 7/9       — Yaw left/right
    Home             — Reset to default pose
    Numpad +/-       — Increase/decrease sensitivity

Sensitivity scales the step size (e.g. 2.0x = twice as far per tap).

Note: letter keys (WASD etc.) conflict with MuJoCo viewer's built-in
visualisation toggles, so we use arrow/numpad keys instead.
"""

from __future__ import annotations

import os
import threading
import time

import numpy as np
import yaml


# GLFW key codes (from glfw3.h) for non-letter keys
_GLFW_KEY_RIGHT       = 262
_GLFW_KEY_LEFT        = 263
_GLFW_KEY_DOWN        = 264
_GLFW_KEY_UP          = 265
_GLFW_KEY_PAGE_UP     = 266
_GLFW_KEY_PAGE_DOWN   = 267
_GLFW_KEY_HOME        = 268
_GLFW_KEY_KP_0        = 320
_GLFW_KEY_KP_1        = 321
_GLFW_KEY_KP_2        = 322
_GLFW_KEY_KP_3        = 323
_GLFW_KEY_KP_4        = 324
_GLFW_KEY_KP_5        = 325
_GLFW_KEY_KP_6        = 326
_GLFW_KEY_KP_7        = 327
_GLFW_KEY_KP_8        = 328
_GLFW_KEY_KP_9        = 329
_GLFW_KEY_KP_ADD      = 334
_GLFW_KEY_KP_SUBTRACT = 333

# How long (seconds) after the last callback before a key is considered released.
# OS key repeat sends events at ~30 Hz once active, so 100 ms bridges any gaps
# while still stopping promptly after the user lifts their finger.
_KEY_RELEASE_TIMEOUT = 0.10


def _load_keyboard_config() -> dict:
    """Load workspace limits from hardware_config.yaml for clamping."""
    candidates = [
        os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'hardware_config.yaml'),
        os.path.join('config', 'hardware_config.yaml'),
    ]
    for path in candidates:
        path = os.path.abspath(path)
        if os.path.isfile(path):
            with open(path) as f:
                config = yaml.safe_load(f)
            op = config['jugglebot_operational']
            return {
                'default_active_z_mm': op['default_active_z_mm'],
            }
    raise FileNotFoundError("Cannot find config/hardware_config.yaml")


class KeyboardInput:
    """Keyboard-driven pose input using MuJoCo viewer key callbacks.

    Pass ``keyboard_input.key_callback`` to ``launch_passive(key_callback=...)``
    when creating the viewer.

    Call ``apply(dt)`` every control loop step (before ``read()``) to
    integrate held-key motion at the correct rate.

    Parameters
    ----------
    pos_rate_mmps : float
        Translation rate (mm/s) at 1.0x sensitivity.
    rot_rate_dps : float
        Rotation rate (deg/s) at 1.0x sensitivity.
    """

    # Arrow keys + numpad → (axis, direction)
    _KEY_MAP = {
        _GLFW_KEY_UP:        ('y', +1),    # Arrow Up    → Y+
        _GLFW_KEY_DOWN:      ('y', -1),    # Arrow Down  → Y-
        _GLFW_KEY_LEFT:      ('x', -1),    # Arrow Left  → X-
        _GLFW_KEY_RIGHT:     ('x', +1),    # Arrow Right → X+
        _GLFW_KEY_PAGE_UP:   ('z', +1),    # Page Up     → Z+
        _GLFW_KEY_PAGE_DOWN: ('z', -1),    # Page Down   → Z-
        _GLFW_KEY_KP_8:      ('rx', +1),   # Numpad 8    → Pitch+
        _GLFW_KEY_KP_2:      ('rx', -1),   # Numpad 2    → Pitch-
        _GLFW_KEY_KP_4:      ('ry', -1),   # Numpad 4    → Roll-
        _GLFW_KEY_KP_6:      ('ry', +1),   # Numpad 6    → Roll+
        _GLFW_KEY_KP_7:      ('rz', +1),   # Numpad 7    → Yaw+
        _GLFW_KEY_KP_9:      ('rz', -1),   # Numpad 9    → Yaw-
    }

    _AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2, 'rx': 3, 'ry': 4, 'rz': 5}

    def __init__(self, pos_rate_mmps: float = 150.0, rot_rate_dps: float = 30.0):
        config = _load_keyboard_config()
        self._z_offset = config['default_active_z_mm']

        self._pos_rate = pos_rate_mmps            # mm/s at 1x sensitivity
        self._rot_rate = np.radians(rot_rate_dps)  # rad/s at 1x sensitivity
        self._sensitivity = 1.0

        self._default_pose = np.array([
            0.0, 0.0, self._z_offset, 0.0, 0.0, 0.0
        ])
        self._target_pose = self._default_pose.copy()
        self._lock = threading.Lock()

        # Track when each movement key was last seen (monotonic time)
        self._key_last_seen: dict[int, float] = {}
        self._reset_requested = False

        # Workspace clamps (conservative, MPC constraints will also clip)
        self._pos_limits = np.array([150.0, 150.0, 260.0])  # mm
        self._rot_limits = np.array([0.45, 0.45, 0.20])     # rad (~26°, 26°, 11°)

        self._print_help()

    def apply(self, dt: float) -> None:
        """Integrate held-key motion for one control step.

        Call this once per control loop iteration (typically at 50 Hz)
        *before* calling ``read()``.

        Parameters
        ----------
        dt : float
            Time since last call (seconds).  Typically ``1/50 = 0.02``.
        """
        now = time.monotonic()

        with self._lock:
            if self._reset_requested:
                self._target_pose = self._default_pose.copy()
                self._reset_requested = False
                self._key_last_seen.clear()
                return

            # Accumulate increments from all currently-held keys
            expired = []
            for key, last_t in self._key_last_seen.items():
                if now - last_t > _KEY_RELEASE_TIMEOUT:
                    expired.append(key)
                    continue

                mapping = self._KEY_MAP.get(key)
                if mapping is None:
                    continue

                axis, direction = mapping
                idx = self._AXIS_INDEX[axis]

                if idx < 3:
                    step = self._pos_rate * self._sensitivity * direction * dt
                else:
                    step = self._rot_rate * self._sensitivity * direction * dt

                self._target_pose[idx] += step

            # Remove expired keys
            for key in expired:
                del self._key_last_seen[key]

            # Clamp to workspace
            self._clamp()

    def read(self) -> np.ndarray:
        """Return the current target pose (6,) — thread-safe."""
        with self._lock:
            return self._target_pose.copy()

    @property
    def sensitivity(self) -> float:
        """Movement rate multiplier (e.g. 2.0 = 300 mm/s and 60 deg/s)."""
        return self._sensitivity

    def key_callback(self, key: int) -> None:
        """MuJoCo viewer key callback — pass to launch_passive(key_callback=...)."""
        now = time.monotonic()

        # Sensitivity adjustment via numpad +/-
        if key == _GLFW_KEY_KP_ADD:
            self._sensitivity = min(5.0, self._sensitivity * 1.5)
            print(f"  Sensitivity: {self._sensitivity:.1f}x "
                  f"({self._pos_rate * self._sensitivity:.0f} mm/s, "
                  f"{np.degrees(self._rot_rate) * self._sensitivity:.0f} deg/s)")
            return
        if key == _GLFW_KEY_KP_SUBTRACT:
            self._sensitivity = max(0.2, self._sensitivity / 1.5)
            print(f"  Sensitivity: {self._sensitivity:.1f}x "
                  f"({self._pos_rate * self._sensitivity:.0f} mm/s, "
                  f"{np.degrees(self._rot_rate) * self._sensitivity:.0f} deg/s)")
            return

        # Reset
        if key == _GLFW_KEY_HOME:
            with self._lock:
                self._reset_requested = True
            print("  Reset to default pose")
            return

        # Movement keys — record timestamp (held until timeout)
        if key in self._KEY_MAP:
            with self._lock:
                self._key_last_seen[key] = now

    def _clamp(self) -> None:
        """Clamp target pose to workspace limits (caller holds lock)."""
        for i in range(3):
            lo = -self._pos_limits[i]
            hi = self._pos_limits[i]
            if i == 2:  # Z: 0 to limit (relative to active pose)
                lo = 0.0
            self._target_pose[i] = np.clip(self._target_pose[i], lo, hi)
        for i in range(3):
            self._target_pose[3 + i] = np.clip(
                self._target_pose[3 + i],
                -self._rot_limits[i],
                self._rot_limits[i],
            )

    @staticmethod
    def _print_help() -> None:
        print("\n--- Keyboard Controls (focus MuJoCo viewer window) ---")
        print("  Arrow Up/Down   : Y forward/back")
        print("  Arrow Left/Right: X left/right")
        print("  Page Up/Down    : Z up/down")
        print("  Numpad 8/2      : Pitch")
        print("  Numpad 4/6      : Roll")
        print("  Numpad 7/9      : Yaw")
        print("  Home            : Reset to default pose")
        print("  Numpad +/-      : Sensitivity (step size multiplier)")
        print("  Tap repeatedly for larger moves; Numpad + for bigger steps")
        print("------------------------------------------------------\n")
