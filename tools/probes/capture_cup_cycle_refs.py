"""Capture CasADi/IPOPT reference cup cycles for the production QP planner.

WHAT THIS CAPTURES
------------------
Runs the REAL ``sim/juggle_planner/juggle_planner.py::plan_cup_cycle`` — the
CasADi/IPOPT program — over a pinned set of six cup cycles and freezes its
pos/vel/acc/jerk/t/catch_k/takeoff_vel output to an ``.npz``. That file is the
**parity authority** for
``ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py``, the numpy-only
QP port, whose T-U2 test asserts < 1 mm / < 10 mm/s against it.

The fixture exists because the reference cannot be recomputed where the test
runs. ``motion/trajectory/`` is pure-Python / numpy-only / Python 3.8 by
architectural rule and CasADi is not importable there; the ROS test environment
has no CasADi at all. Freezing the reference to a committed file is what lets
the parity gate run on every commit instead of only where CasADi lives.

MOTIVATING LOGBOOK ENTRY
    ``logbook/2026-08-30-unified-7dof-planner-phase0-probes.md`` § Decision 1 —
    the Phase 0 probe that measured the QP port at 1.91e-4 mm / 2.47e-3 mm/s
    against these very cases, bound the port to a Goldfarb-Idnani dual active
    set, and caught IPOPT falsely refusing a feasible cycle. This file is the
    promotion of that entry's throwaway ``/tmp/probe_qp_ref.py``.

UNDERPINS
    ``tests/motion/test_cup_cycle.py::test_qp_matches_casadi_reference`` (T-U2)
    and its ``plan_window`` parity twin.

VENV ONLY — CasADi 3.7.2 lives in the project virtualenv, NOT in the system
Python 3.8 the ROS stack runs on. Every invocation below must use the venv
interpreter explicitly::

    # survey run — timestamped .npz + a table, into temp/probes/
    /home/jetson/Desktop/PDJ_venv/venv/bin/python \
        tools/probes/capture_cup_cycle_refs.py

    # REGENERATE THE COMMITTED FIXTURE (the only writer of tools/probes/data/)
    /home/jetson/Desktop/PDJ_venv/venv/bin/python \
        tools/probes/capture_cup_cycle_refs.py --emit-fixture

``--emit-fixture`` is the explicit-output regeneration flag ``tools/probes/
README.md`` mandates for committed reference data: a fixture nobody can
re-baseline is only half a live reference. Re-run it (and re-run the T-U2 test)
whenever ``plan_cup_cycle``'s formulation, ``PlannerConfig``'s defaults, or the
case set below changes.

Cases (a)/(b) are the two fixture families from
``tests/sim/test_demo_juggle_planner.py``; (c)-(f) are grid points chosen so the
BOX constraints are active in the reference solution on every case — an
equality-only solve would hit parity on an unconstrained case and tell us
nothing about the hard part of the problem.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import numpy as np                                              # noqa: E402

from sim.juggle_planner.juggle_planner import (                 # noqa: E402
    GRAVITY, PlannerConfig, plan_cup_cycle, takeoff_velocity)

#: The committed fixture the T-U2 parity test loads.
FIXTURE_PATH = os.path.join(REPO_ROOT, 'tools', 'probes', 'data',
                            'cup_cycle_qp_refs.npz')
#: Survey runs (no ``--emit-fixture``) land here, timestamped.
OUTPUT_DIR = os.path.join(REPO_ROOT, 'temp', 'probes')


def _takeoff(throw_pos, target, flight_s):
    return takeoff_velocity(np.array(throw_pos, float), np.array(target, float),
                            flight_s)


def build_cases():
    """The pinned cycle set. Every parameter is a literal — no config reads.

    Pinning matters: a fixture whose inputs drift with a YAML edit stops being a
    parity reference and becomes a moving target.
    """
    cases = []

    def add(name, throw_pos, throw_target, flight, period, catch_time,
            catch_pos, catch_vel, z_min=0.45, z_max=1.10, detach_axis=None,
            pos0=None, vel0=None, acc0=None):
        v_take = _takeoff(throw_pos, throw_target, flight)
        cases.append(dict(
            name=name, throw_pos=throw_pos, throw_target=throw_target,
            flight_s=flight, period_s=period, catch_time_s=catch_time,
            catch_pos=catch_pos,
            catch_vel=(list(catch_vel) if catch_vel is not None
                       else list(v_take + GRAVITY * flight)),
            z_min_m=z_min, z_max_m=z_max,
            detach_axis=(list(detach_axis) if detach_axis is not None else None),
            pos0=(pos0 if pos0 is not None else throw_pos),
            vel0=(vel0 if vel0 is not None else list(v_take)),
            acc0=(acc0 if acc0 is not None else list(GRAVITY))))

    # (a) tests/sim/test_demo_juggle_planner.py::plan (:50-62)
    add("fixture_level", [0.10, 0.0, 0.80], [-0.10, 0.0, 0.80], 1.03, 0.625,
        0.405, [-0.10, 0.0, 0.80], None)
    # (b) the same file's _tilted_plan (:114-129). Its axis is
    # cup_axis(*tilt_to_throw(v_take)), which equals v_take/|v_take| to 4.6e-16
    # (verified) — the literal unit vector is used so this probe stays free of
    # the ros_ws tilt_geometry import.
    v_tilted = _takeoff([0.0, 0.0, 0.85], [0.12, 0.0, 0.70], 0.60)
    add("fixture_tilted", [0.0, 0.0, 0.85], [0.12, 0.0, 0.70], 0.60, 0.60,
        0.24, [-0.05, 0.0, 0.70], [0.0, 0.0, -2.0], z_min=0.60, z_max=1.00,
        detach_axis=list(v_tilted / np.linalg.norm(v_tilted)))
    # (c)-(f) grid: period 0.5-1.0 s, displaced catch sites inside the boxes.
    add("grid_p050_lat", [0.05, 0.02, 0.80], [-0.05, -0.02, 0.80], 0.85, 0.500,
        0.320, [-0.05, -0.02, 0.76], [0.10, -0.05, -4.00])
    add("grid_p075_diag", [-0.06, 0.08, 0.78], [0.08, -0.06, 0.80], 1.10, 0.750,
        0.500, [0.08, -0.06, 0.74], [-0.15, 0.12, -5.00])
    add("grid_p100_tall", [0.10, 0.0, 0.90], [-0.10, 0.0, 0.90], 1.30, 1.000,
        0.620, [-0.10, 0.04, 0.85], [0.10, -0.05, -6.00])
    add("grid_p065_tilted", [0.02, -0.02, 0.85], [-0.08, 0.06, 0.72], 0.65,
        0.650, 0.300, [0.06, 0.03, 0.75], [-0.25, 0.18, -4.50],
        z_min=0.60, z_max=1.10)
    v_grid = _takeoff([0.02, -0.02, 0.85], [-0.08, 0.06, 0.72], 0.65)
    cases[-1]["detach_axis"] = list(v_grid / np.linalg.norm(v_grid))
    return cases


def solve_case(case):
    """Run the CasADi/IPOPT reference on one case."""
    cfg = PlannerConfig()
    cfg.z_min_m, cfg.z_max_m = case["z_min_m"], case["z_max_m"]
    return plan_cup_cycle(
        pos0=np.array(case["pos0"], float),
        vel0=np.array(case["vel0"], float),
        acc0=np.array(case["acc0"], float),
        throw_pos=np.array(case["throw_pos"], float),
        throw_target=np.array(case["throw_target"], float),
        flight_s=case["flight_s"],
        catch_time_s=case["catch_time_s"],
        catch_pos=np.array(case["catch_pos"], float),
        catch_vel=np.array(case["catch_vel"], float),
        period_s=case["period_s"], cfg=cfg,
        detach_axis=(np.array(case["detach_axis"], float)
                     if case["detach_axis"] else None))


def _boxes_active(case, plan):
    """Did the reference solution saturate any box? (see the module docstring)"""
    return bool(abs(abs(plan.jerk[:, :2]).max() - 300.0) < 1e-3
                or abs(abs(plan.jerk[:, 2]).max() - 6000.0) < 1e-3
                or abs(plan.pos[:, :2]).max() > 0.15 - 1e-6
                or plan.pos[:, 2].min() < case["z_min_m"] + 1e-6
                or plan.pos[:, 2].max() > case["z_max_m"] - 1e-6)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--emit-fixture', action='store_true',
                    help='overwrite the COMMITTED fixture at %s' % FIXTURE_PATH)
    ap.add_argument('--out', default=None,
                    help='explicit output path (overrides both defaults)')
    args = ap.parse_args(argv)

    cases = build_cases()
    out = {}
    solved = []
    print("%-18s %6s %8s  %s" % ("case", "n", "ipopt_s", "note"))
    for case in cases:
        t0 = time.perf_counter()
        try:
            plan = solve_case(case)
        except RuntimeError as exc:
            print("%-18s %6s %8s  INFEASIBLE %s"
                  % (case["name"], "-", "-", str(exc).splitlines()[-1][:60]))
            continue
        wall_s = time.perf_counter() - t0
        name = case["name"]
        out[name + "/pos"] = plan.pos
        out[name + "/vel"] = plan.vel
        out[name + "/acc"] = plan.acc
        out[name + "/jerk"] = plan.jerk
        out[name + "/t"] = plan.t
        out[name + "/catch_k"] = np.array(plan.catch_k)
        out[name + "/takeoff_vel"] = plan.takeoff_vel
        case["ipopt_s"] = wall_s
        solved.append(case)
        print("%-18s %6d %8.3f  boxes_active=%s"
              % (name, plan.jerk.shape[0], wall_s, _boxes_active(case, plan)))

    out["params_json"] = np.array(json.dumps(solved))
    if args.out is not None:
        path = args.out
    elif args.emit_fixture:
        path = FIXTURE_PATH
    else:
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        path = os.path.join(OUTPUT_DIR, 'cup_cycle_qp_refs_%s.npz'
                            % time.strftime('%Y%m%d_%H%M%S'))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez(path, **out)
    print("wrote %s (%d cases)" % (path, len(solved)))
    return 0 if len(solved) == len(cases) else 1


if __name__ == '__main__':
    sys.exit(main())
