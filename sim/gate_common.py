"""Shared machinery for the sim gate harnesses (``reload_gate``, ``toss_gate``).

Behaviour-free, mechanically-shared pieces extracted from ``sim/reload_gate.py``
(2026-07-25, single-ball-toss Phase 2): the sim/cup geometry constants, the
acceptance thresholds, the hold-quiescence metrics, and the optional passive
MuJoCo viewer hook. ``reload_gate`` re-imports and aliases these so its public
surface and CLI stay byte-identical; ``toss_gate`` imports them directly.

Deliberately dependency-light: numpy + time only (``mujoco.viewer`` is
lazy-imported inside :class:`ViewerHook`, so headless machines never touch it).
No production (``jugglebot.*``) imports — the gates own those.
"""

from __future__ import annotations

import time

import numpy as np

# ── Sim/cup geometry constants (measured against sim/model/jugglebot.xml) ──────
# cup_opening_world_z = CUP_Z_BASE_MM + hand_pos_mm + (platform_z_stow − Z_ACTIVE).
# Probed 2026-07-08: neutral (z=170) + hand=0 → 659.6 mm; +1 mm hand → +1 mm cup;
# +1 mm platform z → +1 mm cup (see the reload-gate logbook Design section).
CUP_Z_BASE_MM = 659.6
Z_ACTIVE_MM = 170.0                       # STOW-relative neutral platform z
KNOT_DT_S = 0.025
NEUTRAL_POSE = np.array([0.0, 0.0, Z_ACTIVE_MM, 0.0, 0.0, 0.0])

# Ball radius (mm) — from sim/model/jugglebot.xml ball_geom size (0.035 m).
# Informational: the ball-cup capture instant (contact→hold) is a coupled function
# of this radius + the co-moving cup, empirically ~+45 ms past centre-arrival; the
# hand hold is anchored via `capture_offset_s`, not a closed-form BALL_R/|v| lead.
BALL_R_MM = 35.0

# ── Acceptance thresholds (plan § Hand-catch smoothness) ──
VEL_MATCH_FRAC = 0.15          # |v_hand − v_ball| ≤ 15 % of |v_ball| at contact
HOLD_TRAVEL_MM = 5.0           # platform centroid travel over the hold window
HOLD_TILT_DEG = 1.0            # platform tilt change over the hold window
SEPARATION_MS = 10.0           # max post-contact ball–cup separation
REACH_ENVELOPE_MM = 80.0       # sim-established reliable reach


def travel_mm(samples) -> float:
    """Max centroid excursion (mm) from the first sample of a position window."""
    if len(samples) < 2:
        return 0.0
    arr = np.asarray(samples)
    return float(np.max(np.linalg.norm(arr - arr[0], axis=1)))


def tilt_change_deg(rot_samples) -> float:
    """Max rot-vec excursion (deg) from the first sample of a rotation window."""
    if len(rot_samples) < 2:
        return 0.0
    arr = np.asarray(rot_samples)
    return float(np.degrees(np.max(np.linalg.norm(arr - arr[0], axis=1))))


class ViewerHook:
    """Optional interactive MuJoCo viewer for watching gate runs.

    Purely observational: attaches a passive viewer to the plant's model/data
    (which survive ``plant.reset`` — the plant resets MjData in place) and paces
    the sim to ``speed``× real time with wall-clock sleeps OUTSIDE the physics,
    so gate math, seeding, and PASS/FAIL are bit-identical to a headless run.
    Closing the viewer window stops the run cleanly via ``ViewerClosed``.

    Requires a display (``pip install mujoco`` ships the viewer). Lazy-imported
    so headless machines never touch it.
    """

    def __init__(self, plant, speed: float = 1.0):
        import mujoco.viewer  # lazy: needs a display only when actually used
        self._viewer = mujoco.viewer.launch_passive(plant.model, plant.data)
        self._speed = max(1e-3, float(speed))
        self._plant = plant
        self._wall0 = time.time()
        self._sim0 = float(plant.data.time)

    def sync(self):
        if not self._viewer.is_running():
            raise ViewerClosed()
        self._viewer.sync()
        # Pace sim time to speed× wall time (sleep the surplus, never rush).
        target_wall = self._wall0 + (float(self._plant.data.time) - self._sim0) / self._speed
        lag = target_wall - time.time()
        if lag > 0:
            time.sleep(lag)

    def close(self):
        try:
            self._viewer.close()
        except Exception:
            pass


class ViewerClosed(Exception):
    """Raised by ViewerHook.sync() when the operator closes the viewer window."""


def attach_viewer(gate, viewer_speed, tag: str) -> bool:
    """Attach an interactive viewer to a constructed gate (``--viewer``).
    Returns True on success; on failure (headless machine, no GLFW) prints the
    reason and returns False so the caller proceeds headless. ``tag`` names the
    calling gate in the console prefix (e.g. ``'reload_gate'``)."""
    if viewer_speed is None:
        return False
    try:
        gate.viewer = ViewerHook(gate.plant, speed=viewer_speed)
        print(f"[{tag}] viewer attached ({viewer_speed:g}x real time) — "
              "pacing sleeps sit outside the physics; results are bit-identical "
              "to a headless run. Close the window to stop.")
        return True
    except Exception as e:  # no display / viewer backend unavailable
        print(f"[{tag}] viewer unavailable ({e}) — continuing headless.")
        return False
