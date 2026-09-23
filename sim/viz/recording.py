"""Offscreen MuJoCo video recording + viewer key codes, shared by the sim demos.

Extracted from ``sim/juggle_online.py`` at R4 (2026-09-24, U6b Cluster C): that
module's ``OnlineJuggleRunner``/``OnlineJuggleConfig``/``realize``/
``_flight_and_speed``/``run``/``main`` (the FSM-era "online juggling" demo
loop) were deleted along with the rest of the FSM under ``fsm-final`` — its
only importer was ``tools/probes/juggle_online_debug.py``, also deleted in
the same commit. But three LIVE sim demos (``sim/juggle_bb_catch.py``,
``sim/juggle_throw.py``, ``sim/juggle_selfcatch.py``) import the other half —
``VideoRecorder``, ``build_record_camera`` and the six ``_KEY_*`` GLFW key
codes — verbatim, with a "do NOT re-implement it" comment at each call site.
This module is that surviving half, moved intact (no behaviour change) so the
three demos keep working without resurrecting the FSM demo loop around it.

No ROS2 dependency; ``mujoco`` + (for ``VideoRecorder.__init__``) a system
``ffmpeg`` on PATH.
"""
from __future__ import annotations

from pathlib import Path

import mujoco

# GLFW key codes for the MuJoCo passive viewer's key_callback (shared by every
# sim/juggle_*.py demo runner so they all bind the same viewer controls).
_KEY_SPACE = 32
_KEY_C = 67                # print current free-camera angle as --cam-* flags
_KEY_LEFT_BRACKET = 91     # `[`  — slower
_KEY_RIGHT_BRACKET = 93    # `]`  — faster
_KEY_RIGHT_ARROW = 262     # step one tick when paused
_KEY_LEFT_ARROW = 263      # accepted but no-op (sim can't run backwards)


def build_record_camera(model, cfg):
    """Fixed free camera for ``--record``: the model's default free camera with
    any user-supplied ``cam_*`` field overridden (None inherits the default).

    ``cfg`` is duck-typed — any object with ``cam_azimuth``/``cam_elevation``/
    ``cam_distance``/``cam_lookat`` attributes (each ``None`` or a value) works;
    every caller passes its own demo-specific config dataclass.
    """
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam)
    if cfg.cam_azimuth is not None:
        cam.azimuth = float(cfg.cam_azimuth)
    if cfg.cam_elevation is not None:
        cam.elevation = float(cfg.cam_elevation)
    if cfg.cam_distance is not None:
        cam.distance = float(cfg.cam_distance)
    if cfg.cam_lookat is not None:
        cam.lookat[:] = [float(v) for v in cfg.cam_lookat]
    return cam


class VideoRecorder:
    """Render the live MuJoCo scene offscreen and stream it to ffmpeg.

    One frame per control tick from a FIXED free camera; raw RGB is piped to
    the system ``ffmpeg`` (H.264 mp4) over stdin — needs only ``mujoco`` + an
    ``ffmpeg`` on PATH (no imageio-ffmpeg). Shares the plant's live model/data
    so the drawn frame is exactly the post-step state.
    """

    def __init__(self, model, data, path, *, width, height, fps, cam):
        import subprocess
        self._data = data
        self._cam = cam
        # The MJCF's default offscreen framebuffer is 640x480; rendering larger
        # raises. Grow the offscreen buffer to fit (no effect on physics or the
        # onscreen viewer) before building the renderer.
        if width > model.vis.global_.offwidth:
            model.vis.global_.offwidth = width
        if height > model.vis.global_.offheight:
            model.vis.global_.offheight = height
        self._renderer = mujoco.Renderer(model, height=height, width=width)
        self.path = path
        self.frames = 0
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        cmd = [
            'ffmpeg', '-y', '-loglevel', 'error',
            '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', f'{width}x{height}', '-r', str(fps), '-i', '-',
            '-an', '-c:v', 'libx264', '-pix_fmt', 'yuv420p',
            '-crf', '18', '-preset', 'medium', str(path),
        ]
        try:
            self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        except FileNotFoundError as exc:
            self._renderer.close()
            raise RuntimeError(
                "ffmpeg not found on PATH — required for --record") from exc

    def capture(self) -> None:
        self._renderer.update_scene(self._data, camera=self._cam)
        frame = self._renderer.render()        # (h, w, 3) uint8, C-contiguous
        self._proc.stdin.write(frame.tobytes())
        self.frames += 1

    def close(self) -> None:
        if self._proc is not None:
            try:
                self._proc.stdin.close()
                self._proc.wait()
            finally:
                self._proc = None
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
