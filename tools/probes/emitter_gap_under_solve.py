"""Does a unified ``plan_cycle`` solve starve the 40 Hz setpoint emitter?

Reproduces the 2026-09-06 UH-3 hardware event offline. On the bench the
can-bridge Teensy latched ``fault_state=MPC_STALE`` TWICE, each time during a
``trajectory/plan_cycle`` MODE_NEW + KIND_SETTLE solve, and never during the two
attempts that refused BEFORE solving. The firmware threshold is
``MPC_CMD_STALENESS_US = 250000`` (250 ms with no accepted Setpoint frame while
``s_mpc_active``); the Teensy console measured 804 ms and 324 ms holes, and
``/trajectory/status.cycle_plan_wall_ms`` measured the two solves at 2158.89 ms
and 2021.16 ms.

WHAT THE ROBOT SAID (2026-09-06 14:08-14:11, operator, launch UP, robot
ACTIVATED, nothing moving — the measurement the E-STOP fix was gated on):

    as-is, inside a RECORDED session   solve 245 / 21 / 214 ms   gap 0 ms
    --threads 1, same session          solve 223 / 20 / 186 ms   gap 0 ms
    as-is, FRESH unrecorded session    solve 247 / 76 / 215 ms   gap 0 ms

**Every run survives.** The 12:08 latches did NOT reproduce: under real session
load the solve is 190-250 ms, not 2 s, and at that length the emitter is not
starved at all (gap 0 ms, i.e. no tick ever missed its 25 ms deadline). So BLAS
thread count and rosbag recording are both RULED OUT as the actor — neither
moved the number.

A COLD first solve was the next candidate and it is RULED OUT TOO, by
measurement: in a fresh process the first solve costs 191.0 / 189.8 / 185.4 ms
against a second at 193.2 / 183.5 / 178.9 ms, and with
``trajectory_node._warm_planner_once`` ahead of it 183.2 / 181.2 / 183.3 ms — a
5-7 ms penalty, ~3 %, nothing like 2 s. (The warm-up still ships, as ~185 ms of
cheap insurance at start-up. It is not the explanation.) So the sitting's own two
2.1 s solves stay OPEN, and CPU oversubscription (``--load``, below) is the only
class that reproduces a multi-second solve at all.

Rep 2 of the recorded run refused ``HAND_STROKE: hand position -0.002 rev
outside [0.000, 9.959] rev; at t=0.019s``, and chasing it found a real defect:
this probe never waits for a window to finish, so rep 2 seeded from rep 1's
STILL-RUNNING plan (hand +0.010434 rev ascending at +0.2617 rev/s), and
``_cycle_start_state``'s MOVING branch handed the planner FREE FALL as the
boundary condition of a window over a live carry with no ball in the air. The
Hermite then dipped 0.011953 rev below knot 0 — 0.38 mm, 120x the dive tolerance
— and was correctly refused. Fixed the same day (the moving branch now samples
the active cycle's own cup acceleration at tau); the timing measurement is
unaffected either way, being post-solve.

WHAT IT ESTABLISHED OFFLINE (2026-09-06, this box, 6 cores, numpy 1.24.4 +
OpenBLAS) — the CPU-contention mechanism, reproduced with ``--load``:

* The solve is ~16 ms on an IDLE box and is thousands of SMALL numpy calls —
  no BLAS hotspot, nothing that releases the GIL for long.
* It is savagely sensitive to CPU contention: ~2.2 s at 2-way oversubscription,
  ~5 s at 3-way, ~6 s at 6-way. The robot's measured 2.16 s therefore says the
  live session was running ~2-way oversubscribed, not that the planner is slow.
* The emitter gap tracks the solve at 9-46 %: 198 ms at a 2209 ms solve (just
  UNDER the 250 ms threshold), ~2000 ms at a 4900 ms solve. The hardware's
  804 ms at 2158 ms sits inside that band. **At the robot's own operating point
  the gap straddles the threshold**, which is why it latched twice rather than
  always or never.
* ``sys.setswitchinterval`` is NOT the lever — 0.005 / 0.001 / 0.0002 all leave
  the gap at 2.4-3.4 s. The starvation is OS CPU scheduling, not the GIL's
  switch interval. Any fix has to reduce the solve's CPU demand, raise the
  emitter's scheduling priority, or move the work off the box.

Everything on the path is production code: a real ``TrajectoryNode`` with its
REAL emitter thread and a REAL ``MpcCommandPub`` on an ephemeral port. The
emitter's own ``_max_emit_gap_s`` diagnostic is the measurement.

A rep may end in a REFUSAL rather than an accept, and that does not weaken the
measurement: every refusal on this path is POST-solve, so the full QP +
``validate_cycle`` has already run and been paid for, which is exactly the
window the emitter is starved in. It is the same shape as the hardware event,
where both latches also ended in a refusal. (Until 2026-09-06 the refusal here
was always ``STALE_STATE`` — knot 0 carried a banking tilt the held pose did not
— which ``unified_cycle._start_tilt_for``'s seed pin closed the same day.)

Run:  source ~/Desktop/PDJ_venv/venv/bin/activate

      # The three arms the 14:08 measurement used. Run 1 and 2 with the launch
      # UP and the robot ACTIVATED; run 3 after relaunching with no rosbag.
      python tools/probes/emitter_gap_under_solve.py --reps 3
      python tools/probes/emitter_gap_under_solve.py --reps 3 --threads 1
      python tools/probes/emitter_gap_under_solve.py --reps 3   # fresh, no bag

      # The offline contention sweep that produced the mechanism above.
      python tools/probes/emitter_gap_under_solve.py --load 2
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

REPO = Path('/home/jetson/Desktop/Jugglebot')

ap = argparse.ArgumentParser()
ap.add_argument('--threads', type=int, default=None,
                help='cap OpenBLAS/OMP threads before numpy is imported')
ap.add_argument('--switch-interval', type=float, default=None,
                help='sys.setswitchinterval() value')
ap.add_argument('--reps', type=int, default=3)
ap.add_argument('--load', type=int, default=0,
                help='spawn N CPU-burning subprocesses as background load')
ap.add_argument('--z-mm', type=float, default=690.0)
ap.add_argument('--dx', type=float, default=10.0)
ap.add_argument('--period', type=float, default=1.4)
args = ap.parse_args()

# MUST precede the numpy import.
if args.threads is not None:
    for v in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
              'NUMEXPR_NUM_THREADS'):
        os.environ[v] = str(args.threads)

# NB: do NOT put REPO/'tests' on sys.path — `tests/teensy_link/` shadows the
# real top-level `teensy_link` package. Import the test modules by package path
# (`tests.ros.conftest`) off the repo root instead.
sys.path.insert(0, str(REPO / 'ros_ws' / 'src' / 'jugglebot'))
sys.path.insert(0, str(REPO))

# tests/ros/conftest.py injects the mock ROS2 modules; import it the way pytest
# would so the production node imports resolve.
import tests.ros.conftest  # noqa: F401,E402  (side-effect: installs ROS2 mocks)

import numpy as np  # noqa: E402
import threading  # noqa: E402

if args.switch_interval is not None:
    sys.setswitchinterval(args.switch_interval)

from std_msgs.msg import String  # noqa: E402
from jugglebot_interfaces.msg import MotorStateSingle, RobotState  # noqa: E402
from jugglebot_interfaces.srv import PlanCycle, SetTrajectoryLimits  # noqa: E402
from jugglebot.trajectory_node import TrajectoryNode  # noqa: E402

_SESSION_VEL, _SESSION_ACC, _SESSION_JERK = 250.0, 3000.0, 150000.0


def _robot_state(hand_rev):
    from tests.ros.test_trajectory_node import _ACTIVATE_REV
    rs = RobotState()
    rs.motor_states = [MotorStateSingle(pos_estimate=float(_ACTIVATE_REV[i]))
                       for i in range(6)]
    rs.motor_states.append(MotorStateSingle(pos_estimate=float(hand_rev)))
    rs.is_homed = True
    return rs


def build_node(hand_rev):
    """A seeded TRAJECTORY-mode node with the REAL emitter thread RUNNING."""
    node = TrajectoryNode(start_emitter=True)
    node._on_robot_state(_robot_state(hand_rev))
    node._on_control_mode(String(data='TRAJECTORY'))
    req = SetTrajectoryLimits.Request()
    req.leg_vel_limit_mmps = _SESSION_VEL
    req.leg_acc_limit_mmps2 = _SESSION_ACC
    req.leg_jerk_limit_mmps3 = _SESSION_JERK
    node._svc_set_limits(req, SetTrajectoryLimits.Response())
    return node


def carry_req(cup, dx, z_mm, period):
    r = PlanCycle.Request()
    r.mode = r.MODE_NEW
    r.kind = r.KIND_SETTLE
    r.period_s = float(period)
    r.throw_site_mm = [0.0, 0.0, 0.0]
    r.throw_target_mm = [0.0, 0.0, 0.0]
    r.flight_s = 0.0
    r.catch_site_mm = [0.0, 0.0, 0.0]
    r.catch_vel_mm_s = [0.0, 0.0, 0.0]
    r.catch_frac = 0.0
    r.settle_site_mm = [cup[0] + dx, cup[1], float(z_mm)]
    r.banking_enabled = True
    r.lead_s = 0.0
    r.chain = False
    r.chain_kind = 0
    r.chain_period_s = 0.0
    r.chain_catch_frac = 0.0
    return r


def _spawn_load(n):
    """N CPU-burning subprocesses — a proxy for a live session's box load."""
    import subprocess
    procs = []
    for _ in range(n):
        procs.append(subprocess.Popen(
            [sys.executable, '-c',
             'import numpy as np\n'
             'a=np.random.rand(220,220)\n'
             'while True: a@a\n'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
    time.sleep(2.0)
    return procs


def main():
    procs = _spawn_load(args.load) if args.load else []
    # -0.031 rev is the value the bench's knot 0 actually carried on 2026-09-06.
    node = build_node(-0.031)
    print('threads=%s switch_interval=%.4f s' %
          (args.threads or 'default(all 6 cores)', sys.getswitchinterval()))

    # Let the emitter settle into its cadence before measuring.
    time.sleep(1.0)
    node._max_emit_gap_s = 0.0
    time.sleep(1.0)
    print('IDLE baseline max emitter gap: %.1f ms  (period %.1f ms)'
          % (node._max_emit_gap_s * 1e3, 25.0))

    cup = [0.0, 0.0, 678.7]
    results = []
    for i in range(args.reps):
        node._on_robot_state(_robot_state(-0.031))   # refresh the freshness window
        node._max_emit_gap_s = 0.0
        req = carry_req(cup, args.dx, args.z_mm, args.period)
        t0 = time.perf_counter()
        resp = node._svc_plan_cycle(req, PlanCycle.Response())
        wall = time.perf_counter() - t0
        # Let the emitter tick a few more times so a gap that STRADDLES the
        # return is caught too.
        time.sleep(0.3)
        gap = node._max_emit_gap_s
        results.append((wall, gap))
        print('  rep %d: solve %.0f ms  |  MAX EMITTER GAP %.0f ms  |  '
              'accepted=%s code=%s'
              % (i + 1, wall * 1e3, gap * 1e3, resp.accepted, resp.code))
        if resp.message:
            print('         %s' % (resp.message[:150],))

    worst = max(g for _w, g in results)
    print('\nWORST emitter gap over %d reps: %.0f ms' % (len(results), worst * 1e3))
    print('Firmware MPC_CMD_STALENESS_US threshold: 250 ms')
    print('VERDICT: %s' % ('LATCHES (gap > 250 ms)' if worst > 0.250
                           else 'survives (gap <= 250 ms)'))
    node._emit_stop.set()
    time.sleep(0.2)
    for p in procs:
        p.kill()


if __name__ == '__main__':
    main()
