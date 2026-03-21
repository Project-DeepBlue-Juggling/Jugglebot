"""Pause / step / speed controls for the MuJoCo viewer.

Provides ``SimController`` — a reusable component for viewer keyboard
bindings that control simulation playback.  Used standalone in non-
interactive MPC modes, and composed into ``InteractiveCatchController``
for interactive modes.
"""

from __future__ import annotations


class SimController:
    """Pause / step / speed controls for the viewer.

    Keyboard bindings (active in the MuJoCo viewer window):
        Space       pause / unpause
        Right arrow step one frame (while paused)
        Up arrow    speed up (2x)
        Down arrow  slow down (0.5x)
        R           reset speed to 1x

    Pass ``sim_ctrl.key_callback`` to ``launch_passive(key_callback=...)``.
    """

    # GLFW key codes
    _KEY_SPACE = 32
    _KEY_RIGHT = 262
    _KEY_UP = 265
    _KEY_DOWN = 264
    _KEY_R = 82

    def __init__(self):
        self.paused = False
        self.speed = 1.0
        self._step_once = False

    def key_callback(self, keycode):
        if keycode == self._KEY_SPACE:
            self.paused = not self.paused
            state = "PAUSED" if self.paused else "RUNNING"
            print(f"  [{state}]  speed={self.speed:.2f}x")
        elif keycode == self._KEY_RIGHT:
            if self.paused:
                self._step_once = True
        elif keycode == self._KEY_UP:
            self.speed = min(self.speed * 2.0, 16.0)
            print(f"  speed={self.speed:.2f}x")
        elif keycode == self._KEY_DOWN:
            self.speed = max(self.speed / 2.0, 0.0625)
            print(f"  speed={self.speed:.2f}x")
        elif keycode == self._KEY_R:
            self.speed = 1.0
            print(f"  speed={self.speed:.2f}x")

    def should_step(self) -> bool:
        """Return True if the sim should advance one step this iteration."""
        if not self.paused:
            return True
        if self._step_once:
            self._step_once = False
            return True
        return False

    @property
    def sleep_factor(self) -> float:
        """Multiply the normal sleep duration by this to control speed."""
        return 1.0 / self.speed
