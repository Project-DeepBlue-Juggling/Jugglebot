"""Sim-playback validation for the juggling-demo player pipeline (Phase 2).

Drives MuJoCoPlant open-loop with the TrajectoryPlayer (platform only — no
balls, no hand) and confirms the platform physically traces the analytic
oval. This is the Phase 2 validation gate of
plans/archived/2026-08-15 bb-led-two-ball-juggle-demo.md.

The playback is deliberately run at 1/4 tempo: the point of this test is to
validate the player -> IK -> plant.command pipeline and the trajectory
shape, not MuJoCo's open-loop actuator tracking at full speed (full-tempo
tracking is a hardware concern, Phase 4). The leg-jerk metric is computed
separately, at the real control rate, so it reflects the true trajectory.
"""
import numpy as np

from sim.juggle_planner.pattern import JugglePattern
from sim.juggle_planner.trajectory import build_analytic_oval
from sim.juggle_planner.player import TrajectoryPlayer
from jugglebot.motion.geometry import StewartGeometry
from sim.plant.mujoco_plant import MuJoCoPlant

_CONTROL_DT = 0.025
_SLOWDOWN = 4.0          # play at 1/4 tempo — see module docstring


def test_player_drives_platform_through_the_oval():
    pattern = JugglePattern()
    traj = build_analytic_oval(pattern)
    geom = StewartGeometry()
    player = TrajectoryPlayer(traj, geom=geom, control_dt=_CONTROL_DT)

    plant = MuJoCoPlant(control_dt=_CONTROL_DT)
    # Command toward the THROW keyframe; the platform settles onto it over
    # the first ~10 steps (which the tracking-error assertion below skips).
    plant.reset(traj.eval(0.0)[0])

    n_steps = int(round(1.5 * pattern.platform_period_s
                         * _SLOWDOWN / _CONTROL_DT))
    cmd_xyz, plant_xyz = [], []
    t_rel = 0.0
    for _ in range(n_steps):
        cmd = player.command_at(t_rel)
        plant.command(cmd.ext_mm)
        plant.step(_CONTROL_DT)
        state = plant.get_state()
        assert np.all(np.isfinite(state.platform_pos_mm))
        cmd_xyz.append(cmd.pose[:3].copy())
        plant_xyz.append(state.platform_pos_mm.copy())
        t_rel += _CONTROL_DT / _SLOWDOWN

    cmd_xyz = np.array(cmd_xyz)
    plant_xyz = np.array(plant_xyz)

    # The platform demonstrably traverses the oval in x.
    x_range = plant_xyz[:, 0].max() - plant_xyz[:, 0].min()
    assert x_range > 0.7 * pattern.separation_mm, (
        f"platform x-range {x_range:.1f} mm — expected to traverse the oval "
        f"(separation {pattern.separation_mm:.0f} mm)")

    # Open-loop tracking error (skip the first 10 settling steps).
    err = np.linalg.norm(plant_xyz[10:] - cmd_xyz[10:], axis=1)
    assert err.mean() < 15.0, f"mean tracking error {err.mean():.2f} mm"


def test_commanded_leg_jerk_is_finite_and_bounded():
    """Realised leg jerk of the commanded stream, at the real control rate."""
    pattern = JugglePattern()
    traj = build_analytic_oval(pattern)
    player = TrajectoryPlayer(traj, geom=StewartGeometry(),
                              control_dt=_CONTROL_DT)

    n = int(round(pattern.platform_period_s / _CONTROL_DT))
    ext = np.array([player.command_at(k * _CONTROL_DT).ext_mm
                    for k in range(n)])

    # Third central finite difference, periodic (the trajectory loops).
    jerk = (np.roll(ext, -2, axis=0) - 2 * np.roll(ext, -1, axis=0)
            + 2 * np.roll(ext, 1, axis=0) - np.roll(ext, 2, axis=0)
            ) / (2 * _CONTROL_DT ** 3)

    assert np.all(np.isfinite(jerk))
    peak = np.abs(jerk).max()
    # Generous bound — Phase 2 validates the pipeline produces a
    # bounded-jerk command stream; the offline optimiser will minimise it.
    assert peak < 5.0e5, f"peak commanded leg jerk {peak:.0f} mm/s^3"
