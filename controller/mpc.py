"""CasADi-based NMPC for Stewart platform pose tracking.

Implements the "IK as equality constraint" approach from the development plan:
the platform pose at each horizon step is a decision variable, constrained to
be kinematically consistent with the leg extensions via the Stewart platform IK
equations expressed symbolically in CasADi.

Decision variables (per horizon, N steps):
    u[0..N-1]  (6 each) : commanded leg extensions (mm, STOW-relative)
    q[1..N]    (6 each) : actual leg extensions after actuator lag
    p[1..N]    (6 each) : platform pose [x, y, z, rx, ry, rz] (mm, rad)

Parameters (set each solve):
    p_init  (6)        : current platform pose
    q_init  (6)        : current actual leg extensions
    u_prev  (6)        : previous commanded extensions (smoothness term)
    p_ref   (6*(N+1))  : reference pose at each horizon step

Constraints:
    Actuator dynamics:  q[k+1] = q[k] + (u[k] - q[k]) · α_k,  α_k = 1 - exp(-dt_k/τ)
    IK consistency:     ‖leg_vec_i(p[k])‖ - init_len_i = q[k]_i   (q IS IK extension)
    Rate limits:        |u[k] - u[k-1]| ≤ v_max · dt_between
"""

from __future__ import annotations

import hashlib
import json
import logging
import math
import time as _time
from dataclasses import asdict
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

try:
    import casadi as cs
except ImportError:
    cs = None


from typing import TYPE_CHECKING

from .hermite import quintic_interp, quintic_interp_with_accel
from .params import MPCParams
from .target import ReferenceEvent

if TYPE_CHECKING:
    from typing import List
    from .plant import PlantState


# ---------------------------------------------------------------------------
# Hermite interpolation for time-varying references
# ---------------------------------------------------------------------------

def _hermite_interp(
    t: float,
    t0: float, p0: np.ndarray, v0: np.ndarray,
    t1: float, p1: np.ndarray, v1: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Cubic Hermite interpolation between two events.

    Returns (pose, twist) at time *t* in [t0, t1], matching pose and twist
    exactly at both endpoints.

    Parameters
    ----------
    t : float
        Query time.
    t0, t1 : float
        Endpoint times (t0 < t1).
    p0, p1 : (6,) ndarray
        Endpoint poses.
    v0, v1 : (6,) ndarray
        Endpoint twists (derivatives of pose w.r.t. time).

    Returns
    -------
    pose : (6,) ndarray
    twist : (6,) ndarray
    """
    dt = t1 - t0
    if dt < 1e-9:
        # Degenerate interval — return endpoint to avoid division by zero.
        return p1.copy(), v1.copy()
    s = (t - t0) / dt          # normalised time in [0, 1]
    s2 = s * s
    s3 = s2 * s

    # Hermite basis functions
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2

    pose = h00 * p0 + h10 * (dt * v0) + h01 * p1 + h11 * (dt * v1)

    # Derivatives of basis functions w.r.t. s, divided by dt for d/dt
    dh00 = (6.0 * s2 - 6.0 * s) / dt
    dh10 = (3.0 * s2 - 4.0 * s + 1.0)        # * dt / dt cancels
    dh01 = (-6.0 * s2 + 6.0 * s) / dt
    dh11 = (3.0 * s2 - 2.0 * s)               # * dt / dt cancels

    twist = dh00 * p0 + dh10 * v0 + dh01 * p1 + dh11 * v1

    return pose, twist


def _quintic_interp_events(
    t: float,
    t0: float, p0: np.ndarray, v0: np.ndarray, a0: np.ndarray,
    t1: float, p1: np.ndarray, v1: np.ndarray, a1: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Quintic Hermite interpolation between two events (absolute time).

    Drop-in companion to ``_hermite_interp`` when acceleration boundary
    conditions are available.  Uses the quintic basis from ``hermite.py``
    which matches pos, vel, and accel at both endpoints (C2-continuous).

    Returns (pose, twist, accel).
    """
    dt = t1 - t0
    if dt < 1e-9:
        return p1.copy(), v1.copy(), a1.copy()
    return quintic_interp_with_accel(p0, v0, a0, p1, v1, a1, dt, t - t0)


def _build_reference_from_events(
    events: List[ReferenceEvent],
    t_now: float,
    cumulative_times: np.ndarray,
    n_nodes: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build (N+1, 6) reference arrays from a list of timed events.

    Uses quintic Hermite interpolation (C2) between events that provide
    acceleration, falling back to cubic Hermite (C1) otherwise.  Before
    the first event, uses quadratic backward extrapolation when accel is
    available (linear otherwise).  After the last event, quadratic forward
    extrapolation (linear fallback).

    Parameters
    ----------
    events : list[ReferenceEvent]
        Sorted by time (ascending).  Must have at least one event.
    t_now : float
        Current absolute time (``state.time``).
    cumulative_times : (N+1,) ndarray
        Relative times at each horizon node (starting at 0).
    n_nodes : int
        Number of horizon nodes (N+1).

    Returns
    -------
    ref_traj : (N+1, 6) ndarray
        Per-node reference poses.
    twist_traj : (N+1, 6) ndarray
        Per-node reference twists.
    accel_traj : (N+1, 6) ndarray
        Per-node reference accelerations.  Populated from quintic
        interpolation when available, finite-differenced from twist
        otherwise.
    """
    ref_traj = np.empty((n_nodes, 6))
    twist_traj = np.empty((n_nodes, 6))
    accel_traj = np.zeros((n_nodes, 6))
    accel_filled = np.zeros(n_nodes, dtype=bool)  # tracks authoritative accel

    n_ev = len(events)
    # Pre-extract arrays for fast access
    ev_times = np.array([e.time for e in events])
    ev_poses = np.array([e.pose for e in events])
    ev_twists = np.array([
        e.twist if e.twist is not None else np.zeros(6) for e in events
    ])
    ev_accels = [e.accel for e in events]  # None or (6,) per event

    if __debug__ and n_ev > 1 and np.any(np.diff(ev_times) < 0):
        raise ValueError("ref_events must be sorted by time (ascending)")

    ev_idx = 0  # reuse search position across nodes (times are monotonic)

    for k in range(n_nodes):
        t_k = t_now + cumulative_times[k]

        if t_k <= ev_times[0]:
            # Before first event: extrapolate backward
            dt_back = t_k - ev_times[0]
            a0 = ev_accels[0]
            if a0 is not None:
                # Quadratic extrapolation (C2 at boundary)
                ref_traj[k] = (ev_poses[0] + ev_twists[0] * dt_back
                               + 0.5 * a0 * dt_back * dt_back)
                twist_traj[k] = ev_twists[0] + a0 * dt_back
                accel_traj[k] = a0
                accel_filled[k] = True
            else:
                # Linear extrapolation (C1 at boundary)
                ref_traj[k] = ev_poses[0] + ev_twists[0] * dt_back
                twist_traj[k] = ev_twists[0]

        elif t_k >= ev_times[-1]:
            # After last event: extrapolate forward
            dt_fwd = t_k - ev_times[-1]
            a_last = ev_accels[-1]
            if a_last is not None:
                ref_traj[k] = (ev_poses[-1] + ev_twists[-1] * dt_fwd
                               + 0.5 * a_last * dt_fwd * dt_fwd)
                twist_traj[k] = ev_twists[-1] + a_last * dt_fwd
                accel_traj[k] = a_last
                accel_filled[k] = True
            else:
                ref_traj[k] = ev_poses[-1] + ev_twists[-1] * dt_fwd
                twist_traj[k] = ev_twists[-1]

        else:
            # Between events: interpolate
            # Advance ev_idx to find the bracketing interval
            while ev_idx < n_ev - 2 and ev_times[ev_idx + 1] < t_k:
                ev_idx += 1
            i = ev_idx
            if ev_times[i + 1] - ev_times[i] < 1e-6:
                # Degenerate interval: snap to endpoint
                ref_traj[k] = ev_poses[i + 1]
                twist_traj[k] = ev_twists[i + 1]
                if ev_accels[i + 1] is not None:
                    accel_traj[k] = ev_accels[i + 1]
                    accel_filled[k] = True
            elif ev_accels[i] is not None and ev_accels[i + 1] is not None:
                # Quintic Hermite (C2) — both endpoints have accel
                pose, twist, accel = _quintic_interp_events(
                    t_k,
                    ev_times[i], ev_poses[i], ev_twists[i], ev_accels[i],
                    ev_times[i + 1], ev_poses[i + 1], ev_twists[i + 1],
                    ev_accels[i + 1],
                )
                ref_traj[k] = pose
                twist_traj[k] = twist
                accel_traj[k] = accel
                accel_filled[k] = True
            else:
                # Cubic Hermite (C1) — fallback when accel missing
                pose, twist = _hermite_interp(
                    t_k,
                    ev_times[i], ev_poses[i], ev_twists[i],
                    ev_times[i + 1], ev_poses[i + 1], ev_twists[i + 1],
                )
                ref_traj[k] = pose
                twist_traj[k] = twist

    # Fill accel gaps via finite-difference for nodes where no authoritative
    # source (quintic interpolation, quadratic extrapolation) provided accel.
    for k in range(n_nodes):
        if not accel_filled[k] and k > 0:
            dt_k = cumulative_times[k] - cumulative_times[k - 1]
            if dt_k > 1e-9:
                accel_traj[k] = (twist_traj[k] - twist_traj[k - 1]) / dt_k

    return ref_traj, twist_traj, accel_traj


# ---------------------------------------------------------------------------
# CasADi symbolic helpers
# ---------------------------------------------------------------------------

def _casadi_skew(v):
    """3-vector → 3×3 skew-symmetric matrix (CasADi SX)."""
    return cs.vertcat(
        cs.horzcat(0, -v[2], v[1]),
        cs.horzcat(v[2], 0, -v[0]),
        cs.horzcat(-v[1], v[0], 0),
    )


def _casadi_rodrigues(rv):
    """Rotation vector → 3×3 rotation matrix (CasADi SX).

    Uses Rodrigues formula with a regularised denominator (ε = 1e-20) so that
    sin(θ)/θ and (1-cos θ)/θ² evaluate correctly at θ = 0.
    """
    angle_sq = cs.dot(rv, rv)
    reg_sq = angle_sq + 1e-20          # always > 0
    angle = cs.sqrt(reg_sq)

    sinc = cs.sin(angle) / angle       # → 1  as θ → 0
    cosc = (1.0 - cs.cos(angle)) / reg_sq  # → 0.5 as θ → 0

    K = _casadi_skew(rv)
    return cs.SX.eye(3) + sinc * K + cosc * (K @ K)


def _build_symbolic_ik(base_nodes, plat_nodes, init_height_vec, init_leg_lengths):
    """Build CasADi Function:  pose (6,) → IK extensions (6,).

    The IK extension for leg *i* is:
        ext_i = ‖platform_centre + R(rv) · plat_node_i − base_node_i‖ − init_len_i
    """
    pose = cs.SX.sym('pose', 6)
    pos = pose[:3]
    rv = pose[3:]
    R = _casadi_rodrigues(rv)

    height = cs.DM(init_height_vec)
    exts = []
    for i in range(6):
        plat_w = pos + height + R @ cs.DM(plat_nodes[i])
        leg_vec = plat_w - cs.DM(base_nodes[i])
        exts.append(cs.norm_2(leg_vec) - float(init_leg_lengths[i]))

    return cs.Function('ik', [pose], [cs.vertcat(*exts)])


# ---------------------------------------------------------------------------
# MPC Controller
# ---------------------------------------------------------------------------

class MPCController:
    """Nonlinear MPC for Stewart platform pose tracking.

    Build the CasADi NLP once at construction; each :meth:`solve` call
    updates parameters and warm-starts from the previous solution.
    """

    def __init__(
        self,
        params: MPCParams,
        base_nodes: np.ndarray,
        plat_nodes: np.ndarray,
        init_height_mm: float,
        init_leg_lengths_mm: np.ndarray,
        active_extensions_mm: np.ndarray,
    ):
        if cs is None:
            raise ImportError("CasADi is required for MPC — pip install casadi")

        self._params = params
        self._active_ext = np.asarray(active_extensions_mm, dtype=float)
        self._init_height_mm = init_height_mm

        # Geometry (plain arrays for CasADi)
        self._base_nodes = np.asarray(base_nodes, dtype=float)
        self._plat_nodes = np.asarray(plat_nodes, dtype=float)
        self._init_height_vec = np.array([0.0, 0.0, init_height_mm])
        self._init_leg_lengths = np.asarray(init_leg_lengths_mm, dtype=float)

        self._build_problem()

        # Warm-start state — _prev_w is the last *successful* solution,
        # used both for warm-start and for fallback commanding in
        # _handle_failure.  _timeout_hint is a partial (unconverged)
        # solution from a CpuTime timeout — used ONLY as the initial
        # guess for the next solve, never for commanding.
        self._prev_w: np.ndarray | None = None
        self._prev_lam_g: np.ndarray | None = None
        self._prev_lam_x: np.ndarray | None = None
        self._timeout_hint: np.ndarray | None = None
        self._timeout_lam_g: np.ndarray | None = None
        self._timeout_lam_x: np.ndarray | None = None
        self._prev_u: np.ndarray | None = None
        self._prev_prev_u: np.ndarray | None = None
        self._consecutive_failures: int = 0
        self._prev_solve_time: float | None = None
        self._last_actual_dt: float | None = None
        # _predicted_poses is pre-allocated in _build_problem(); don't reset it.
        self._predicted_poses_valid = False
        self._last_ref_traj: np.ndarray | None = None
        self._last_twist_traj: np.ndarray | None = None
        self._last_accel_traj: np.ndarray | None = None

        if params.prime_solver:
            self._prime_solver()

    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------

    @classmethod
    def from_plant(cls, params: MPCParams, plant) -> MPCController:
        """Create controller from a plant instance.

        Computes ``active_extensions_mm`` — the IK extensions at the Active
        position (default_active_z) — for the effort cost reference.
        """
        geom = plant.geom

        # Compute IK extensions at the Active pose [0, 0, default_z, 0, 0, 0]
        import jugglebot.hardware_config as hw
        active_rev = np.array(hw.JB_OP_ACTIVATE_POSITION_REVS)
        active_ext = active_rev / geom.mm_to_rev

        return cls(
            params=params,
            base_nodes=geom.base_nodes,
            plat_nodes=geom.plat_nodes,
            init_height_mm=geom.init_height_mm,
            init_leg_lengths_mm=geom.init_leg_lengths_mm,
            active_extensions_mm=active_ext,
        )

    # ------------------------------------------------------------------
    # Ahead-of-time compiled NLP loading
    # ------------------------------------------------------------------

    _AOT_SO_PATH = Path(__file__).resolve().parent / 'generated' / 'mpc_gen.so'
    _AOT_HASH_PATH = Path(__file__).resolve().parent / 'generated' / 'mpc_gen.hash'

    # Params that do NOT affect the symbolic NLP CasADi compiles into the
    # .so.  They are either IPOPT runtime options, orchestration toggles,
    # or numeric bounds that the solver receives as `lbx`/`ubx`/`lbg`/`ubg`
    # data at each call.  Excluding them from the hash keeps the AOT
    # binary valid across common tuning changes (``max_cpu_time``,
    # ``max_leg_vel_mmps``, ``stroke_mm``, etc.) — ONLY cost weights,
    # horizon structure, and geometry invalidate the compiled code.
    _NLP_HASH_EXCLUDE_PARAMS = frozenset({
        # Runtime IPOPT options
        'max_iter', 'max_cpu_time', 'tol', 'print_level', 'warm_start',
        # Orchestration toggles
        'max_consecutive_failures', 'prime_solver', 'use_aot_solver',
        # Numeric bounds (applied at runtime via lbw/ubw/lbg/ubg arrays,
        # not baked into the compiled callbacks)
        'stroke_mm', 'stroke_margin_mm', 'max_leg_vel_mmps',
    })

    def _nlp_source_hash(self) -> str:
        """SHA256 of everything that affects the generated NLP structure.

        Used by ``controller/generate_solver.py`` to stamp the .so and by
        ``_load_nlpsol`` to detect stale binaries.  Anything that changes
        the decision-variable layout, constraint structure, cost terms, or
        geometry must flow through here.  Runtime solver options (max_iter,
        max_cpu_time, prime_solver, etc.) are deliberately excluded — they
        are passed as IPOPT ``opts`` at load time, not baked into the .so.
        """
        h = hashlib.sha256()
        mpc_py = Path(__file__).resolve()
        h.update(mpc_py.read_bytes())

        structural_params = {
            k: v for k, v in asdict(self._params).items()
            if k not in self._NLP_HASH_EXCLUDE_PARAMS
        }
        h.update(json.dumps(structural_params,
                            sort_keys=True, default=str).encode())
        h.update(np.ascontiguousarray(self._base_nodes).tobytes())
        h.update(np.ascontiguousarray(self._plat_nodes).tobytes())
        h.update(np.float64(self._init_height_mm).tobytes())
        h.update(np.ascontiguousarray(self._init_leg_lengths).tobytes())
        h.update(np.ascontiguousarray(self._active_ext).tobytes())
        h.update(cs.__version__.encode())
        return h.hexdigest()

    def _load_nlpsol(self, nlp: dict, opts: dict):
        """Construct the CasADi IPOPT solver, preferring an AOT .so if present.

        Policy:
          * ``params.use_aot_solver == False`` → always build in-process
            (e.g. ``FeasibilityChecker``'s coarse-horizon one-shot MPC)
          * ``.so`` present + hash matches   → load precompiled (production)
          * ``.so`` present + hash mismatch  → ``RuntimeError`` (refuse to
            silently run with stale C code)
          * ``.so`` absent                   → in-process build (dev/test)
        """
        if not self._params.use_aot_solver:
            return cs.nlpsol('mpc', 'ipopt', nlp, opts)
        so_path = self._AOT_SO_PATH
        hash_path = self._AOT_HASH_PATH
        if so_path.exists():
            expected = self._nlp_source_hash()
            stored = hash_path.read_text().strip() if hash_path.exists() else ''
            if stored != expected:
                raise RuntimeError(
                    f"{so_path} is stale (hash mismatch).  Regenerate with: "
                    f"python controller/generate_solver.py\n"
                    f"  expected: {expected[:16]}...\n"
                    f"  found:    {stored[:16] if stored else '<missing>'}..."
                )
            logger.info("MPC: loading AOT-compiled solver from %s", so_path)
            return cs.nlpsol('mpc', 'ipopt', str(so_path), opts)
        logger.info("MPC: no AOT solver found, building in-process "
                    "(run `python controller/generate_solver.py` to "
                    "precompile)")
        return cs.nlpsol('mpc', 'ipopt', nlp, opts)

    # ------------------------------------------------------------------
    # NLP construction (called once)
    # ------------------------------------------------------------------

    def _build_problem(self):
        N = self._params.N
        dt_schedule = self._params.dt_schedule
        tau = self._params.tau
        stroke = self._params.stroke_mm
        max_leg_vel = self._params.max_leg_vel_mmps

        sym_ik = _build_symbolic_ik(
            self._base_nodes, self._plat_nodes,
            self._init_height_vec, self._init_leg_lengths,
        )

        # ---- Parameters (change every solve) ------------------------------
        # p_init(6) + q_init(6) + u_prev(6) + u_prev_prev(6)
        # + p_ref(6*(N+1)) + twist_ref(6*(N+1)) + accel_ref(6*(N+1))
        # + vel_weights(4): [Q_vel_lin, Q_vel_ang, Qf_vel_lin, Qf_vel_ang]
        # + accel_weights(2): [Q_accel_lin, Q_accel_ang]
        n_param = 24 + 18 * (N + 1) + 6
        P = cs.SX.sym('P', n_param)

        p_init = P[:6]
        q_init = P[6:12]
        u_prev_sym = P[12:18]
        u_prev_prev_sym = P[18:24]
        ref_offset = 24
        p_ref = [P[ref_offset + k * 6: ref_offset + (k + 1) * 6] for k in range(N + 1)]
        twist_offset = ref_offset + 6 * (N + 1)
        twist_ref = [P[twist_offset + k * 6: twist_offset + (k + 1) * 6] for k in range(N + 1)]
        accel_offset = twist_offset + 6 * (N + 1)
        accel_ref = [P[accel_offset + k * 6: accel_offset + (k + 1) * 6] for k in range(N + 1)]
        weights_offset = accel_offset + 6 * (N + 1)
        Qvl_sym = P[weights_offset]       # Q_vel_lin (per-solve)
        Qva_sym = P[weights_offset + 1]   # Q_vel_ang
        Qfvl_sym = P[weights_offset + 2]  # Qf_vel_lin
        Qfva_sym = P[weights_offset + 3]  # Qf_vel_ang
        Qal_sym = P[weights_offset + 4]   # Q_accel_lin
        Qaa_sym = P[weights_offset + 5]   # Q_accel_ang

        # ---- Decision variables -------------------------------------------
        # Layout: u[0..N-1](6N)  |  q[1..N](6N)  |  p[1..N](6N)
        n_w = 18 * N
        W = cs.SX.sym('W', n_w)

        u = [W[k * 6: (k + 1) * 6] for k in range(N)]
        q = [q_init] + [W[6 * N + k * 6: 6 * N + (k + 1) * 6] for k in range(N)]
        p = [p_init] + [W[12 * N + k * 6: 12 * N + (k + 1) * 6] for k in range(N)]

        # ---- Cost ---------------------------------------------------------
        active_ext_dm = cs.DM(self._active_ext)
        Qp = self._params.Q_pos
        Qo = self._params.Q_ori
        Qfp = self._params.Qf_pos
        Qfo = self._params.Qf_ori
        R_w = self._params.R
        S_w = self._params.S
        A_w = self._params.A

        J = 0.0

        for k in range(1, N + 1):
            err = p[k] - p_ref[k]
            dt_k = dt_schedule[k - 1]  # dt of the interval ending at node k

            if k < N:
                J += Qp * cs.dot(err[:3], err[:3])
                J += Qo * cs.dot(err[3:], err[3:])
            else:
                # Terminal cost (heavier)
                J += Qfp * cs.dot(err[:3], err[:3])
                J += Qfo * cs.dot(err[3:], err[3:])

            # Velocity tracking (finite-difference twist vs reference twist)
            # Velocity weights are NLP parameters (set per-solve) so that
            # event-based callers can use heavier weights without degrading
            # flat-reference callers.
            dp = (p[k] - p[k - 1]) / dt_k
            twist_err = dp - twist_ref[k]
            if k < N:
                J += Qvl_sym * cs.dot(twist_err[:3], twist_err[:3])
                J += Qva_sym * cs.dot(twist_err[3:], twist_err[3:])
            else:
                # Terminal velocity cost (heavier — match twist at horizon end)
                J += Qfvl_sym * cs.dot(twist_err[:3], twist_err[:3])
                J += Qfva_sym * cs.dot(twist_err[3:], twist_err[3:])

            # Acceleration tracking (finite-difference accel vs reference)
            # Requires two consecutive velocity estimates, so starts at k=2.
            if k >= 2:
                dt_km1 = dt_schedule[k - 2]  # interval ending at node k-1
                vel_k = dp  # already computed above: (p[k] - p[k-1]) / dt_k
                vel_km1 = (p[k - 1] - p[k - 2]) / dt_km1
                # Backward difference at node k, consistent with how vel_k
                # itself is computed.  Avoids systematic bias at the
                # fine→coarse tier boundary where dt_k ≠ dt_km1.
                accel_fd = (vel_k - vel_km1) / dt_k
                accel_err = accel_fd - accel_ref[k]
                J += Qal_sym * cs.dot(accel_err[:3], accel_err[:3])
                J += Qaa_sym * cs.dot(accel_err[3:], accel_err[3:])

        for k in range(N):
            # -- dt_between: effective interval for the u[k-1] → u[k] transition --
            #
            # Rate limits use dt_schedule[k-1] (the *previous* step's duration)
            # because the control loop emits at the fine rate: the physical time
            # available for the transition u[k-1]→u[k] is the interval that
            # *produced* u[k-1], not the one u[k] controls.  For k=0 (transition
            # from the external u_prev), this is one fine control step.
            #
            # For smoothness and acceleration costs, the tier boundary (k = first
            # coarse step) is special: the transition straddles a fine step on one
            # side and a coarse step on the other.  Using the fine dt alone would
            # over-penalise this edge (12.5× heavier than neighboring coarse
            # steps); using the coarse dt would under-penalise it.  The geometric
            # mean sqrt(dt_fine * dt_coarse) splits the penalty evenly in
            # log-space between the two tiers, giving a balanced ~3.5× ratio.
            dt_rate = dt_schedule[0] if k == 0 else dt_schedule[k - 1]
            if k == 0:
                dt_smooth = dt_schedule[0]
            elif dt_schedule[k - 1] != dt_schedule[k]:
                # Tier boundary: geometric mean of the two neighboring intervals
                dt_smooth = math.sqrt(dt_schedule[k - 1] * dt_schedule[k])
            else:
                dt_smooth = dt_schedule[k - 1]

            # Control effort (relative to Active position to avoid bias)
            u_dev = u[k] - active_ext_dm
            J += R_w * cs.dot(u_dev, u_dev)
            # Smoothness: penalise command rate ||du/dt||², integrated over dt.
            # = S/dt * ||du||²  (normalised so coarse and fine steps are comparable)
            du_k = u[k] - (u_prev_sym if k == 0 else u[k - 1])
            J += (S_w / dt_smooth) * cs.dot(du_k, du_k)
            # Acceleration smoothness: A/dt · ||ddu||².
            # The physically "correct" jerk penalty would be A/dt³, but that
            # makes coarse-tier jerk ~2000× cheaper than fine-tier (dt ratio
            # cubed).  A/dt gives a 12.5× ratio, keeping coarse steps from
            # jerking freely while still penalising fine-tier oscillation more.
            if k == 0:
                du_prev = u_prev_sym - u_prev_prev_sym
            elif k == 1:
                du_prev = u[0] - u_prev_sym
            else:
                du_prev = u[k - 1] - u[k - 2]
            ddu = du_k - du_prev
            J += (A_w / dt_smooth) * cs.dot(ddu, ddu)

        # ---- Constraints --------------------------------------------------
        g_list = []

        # 1. Actuator dynamics  (6·N equality)
        # Use exact exponential decay: alpha_k = 1 - exp(-dt_k / tau).
        # Forward Euler (dt/tau) is unstable when dt/tau > 2 (coarse steps
        # have dt/tau = 0.25/0.03 = 8.33).  Exact form is unconditionally
        # stable: alpha ∈ (0, 1) for any positive dt.
        for k in range(N):
            alpha_k = 1.0 - math.exp(-dt_schedule[k] / tau)
            q_pred = q[k] + (u[k] - q[k]) * alpha_k
            g_list.append(q[k + 1] - q_pred)

        # 2. IK consistency  (6·N equality) — q IS IK extension, direct equality
        for k in range(1, N + 1):
            ik_ext = sym_ik(p[k])
            g_list.append(ik_ext - q[k])

        # 3. Rate limits  (6·N inequality)
        for k in range(N):
            du = u[k] - (u_prev_sym if k == 0 else u[k - 1])
            g_list.append(du)

        g = cs.vertcat(*g_list)

        # ---- Constraint bounds --------------------------------------------
        n_eq = 12 * N       # actuator + IK
        n_ineq = 6 * N      # rate limits
        n_g = n_eq + n_ineq

        lbg = np.zeros(n_g)
        ubg = np.zeros(n_g)
        # Per-step rate limits: |du| <= v_max * dt_rate
        # du = u[k] - u[k-1] spans the interval ending at u[k].
        # For k=0: the interval is one control step (dt_schedule[0]).
        # For k>0: the interval is dt_schedule[k-1] (the previous step's duration).
        # Rate limits use the *physical* interval (not the geometric mean used
        # for smoothness cost) because they are hard constraints on actuator speed.
        for k in range(N):
            dt_rate = dt_schedule[0] if k == 0 else dt_schedule[k - 1]
            v_max_dt_k = max_leg_vel * dt_rate
            lbg[n_eq + k * 6: n_eq + (k + 1) * 6] = -v_max_dt_k
            ubg[n_eq + k * 6: n_eq + (k + 1) * 6] = v_max_dt_k

        # ---- Variable bounds ----------------------------------------------
        lbw = np.full(n_w, -np.inf)
        ubw = np.full(n_w, np.inf)

        margin = self._params.stroke_margin_mm
        for k in range(N):
            # u bounds [margin, stroke - margin] — match motor_guard hard limits
            lbw[k * 6: (k + 1) * 6] = margin
            ubw[k * 6: (k + 1) * 6] = stroke - margin
            # q bounds [0, stroke] — actual leg extensions (after actuator lag)
            lbw[6 * N + k * 6: 6 * N + (k + 1) * 6] = 0.0
            ubw[6 * N + k * 6: 6 * N + (k + 1) * 6] = stroke
            # p bounds (workspace limits — generous, stroke limits are the real safety net)
            bp = 12 * N + k * 6
            lbw[bp: bp + 2] = -200.0       # x, y (mm)
            ubw[bp: bp + 2] = 200.0
            lbw[bp + 2] = -50.0            # z (mm) — can't go below active pose much
            ubw[bp + 2] = 300.0            # z (mm) — stroke limit caps at ~275
            lbw[bp + 3: bp + 6] = -0.3     # rx, ry, rz (rad ≈ ±17°)
            ubw[bp + 3: bp + 6] = 0.3

        # ---- Solver -------------------------------------------------------
        nlp = {'x': W, 'f': J, 'g': g, 'p': P}
        opts = {
            'ipopt.max_iter': self._params.max_iter,
            'ipopt.max_cpu_time': self._params.max_cpu_time,
            'ipopt.tol': self._params.tol,
            'ipopt.print_level': self._params.print_level,
            'ipopt.sb': 'yes',
            'print_time': 0,
            # Adaptive barrier reduces iterations by 20-40% on coupled NLPs
            # (tau=0.065 creates tight actuator-IK coupling).
            'ipopt.mu_strategy': 'adaptive',
            # Early termination: accept 10x-relaxed solution after 3
            # consecutive "acceptable" iterations.  For a 25ms re-solve
            # loop, 1e-3 tolerance is perfectly adequate.
            'ipopt.acceptable_tol': 1e-3,
            'ipopt.acceptable_iter': 3,
        }
        if self._params.warm_start:
            opts['ipopt.warm_start_init_point'] = 'yes'
            opts['ipopt.warm_start_bound_push'] = 1e-8
            opts['ipopt.warm_start_mult_bound_push'] = 1e-8
            # Match barrier init to tolerance so warm-started solves skip
            # unnecessary barrier reduction phases.
            opts['ipopt.mu_init'] = 1e-4

        self._n_w = n_w
        self._n_param = n_param
        self._N = N
        self._dt_schedule = dt_schedule
        # Load the NLP.  Prefer an ahead-of-time compiled .so when one is
        # available and matches the current source+params hash; otherwise
        # fall back to an in-process build.  See
        # ``controller/generate_solver.py`` for the AOT generator.
        self._solver = self._load_nlpsol(nlp, opts)
        self._cumulative_times = self._params.cumulative_times
        self._lbw = lbw
        self._ubw = ubw
        self._lbg = lbg
        self._ubg = ubg

        # Pre-allocated buffers to avoid per-solve allocations
        par = self._params
        self._p_cur_buf = np.empty(6)
        self._p_param_buf = np.empty(n_param)
        self._vel_weights_normal = np.array([
            par.Q_vel_lin, par.Q_vel_ang,
            par.Qf_vel_lin, par.Qf_vel_ang,
        ])
        self._vel_weights_boosted = np.array([
            par.Q_vel_lin_events, par.Q_vel_ang_events,
            par.Qf_vel_lin_events, par.Qf_vel_ang_events,
        ])
        self._accel_weights = np.array([par.Q_accel_lin, par.Q_accel_ang])
        self._predicted_poses = np.zeros((N + 1, 6))

    # ------------------------------------------------------------------
    # Solve
    # ------------------------------------------------------------------

    def solve(
        self,
        state: PlantState,
        target_pose: np.ndarray,
        *,
        ref_events: List[ReferenceEvent] | None = None,
        boost_vel_weights: bool = False,
        t_now: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray, dict]:
        """Solve MPC for one step.

        Parameters
        ----------
        state : PlantState
            Current plant state.
        target_pose : (6,) ndarray
            Fallback target pose ``[x, y, z, rx, ry, rz]`` (mm, rad).
            Used only when ``ref_events`` is None (safety fallback).
        ref_events : list[ReferenceEvent] or None
            Time-varying reference trajectory.  The MPC builds per-node
            references via quintic Hermite interpolation (C2) between
            events with acceleration, falling back to cubic Hermite (C1)
            for events without.  All sources should provide this via
            ``flat_target_to_events()`` or equivalent.
        boost_vel_weights : bool
            If True, use the heavier event-based velocity tracking weights
            (``Q_vel_lin_events`` etc.).  Should be True only for sources
            with kinematically consistent twist references (toss loop,
            scheduler).  Default False uses conservative weights that
            don't penalize motion toward a target.
        t_now : float or None
            Reference-clock time to sample events at.  ``None`` uses
            ``state.time``.  The runner passes a separately-maintained
            ``t_ref`` that is frozen while the solver is in a fallback
            class so the reference does not run away from the platform.

        Returns
        -------
        cmd : (6,) ndarray
            STOW-relative leg extension command (mm).  At Active position,
            cmd ≈ [154.5, ...] per leg.
        cmd_vel : (6,) ndarray
            Forward-looking command velocity (mm/s): ``(cmd - q_cur) / dt_fine``.
            Analytically cleaner than backward-differencing against measured
            state because it uses the same ``q_cur`` the MPC optimized against
            and doesn't include tracking error noise.
        diag : dict
            Diagnostics: solve_time_ms, status, cost, constraint_violation.
        """
        target = np.asarray(target_pose, dtype=float)

        ref_traj, twist_traj, accel_traj = self._build_reference(
            state, target, ref_events=ref_events, t_now=t_now)
        self._last_ref_traj = ref_traj
        self._last_twist_traj = twist_traj
        self._last_accel_traj = accel_traj

        # Current state — fill pre-allocated buffer instead of concatenate
        p_cur = self._p_cur_buf
        p_cur[:3] = state.platform_pos_mm
        p_cur[3:] = state.platform_rot
        q_cur = state.leg_extensions_mm.copy()
        u_prev = self._prev_u if self._prev_u is not None else q_cur.copy()
        u_prev_prev = self._prev_prev_u if self._prev_prev_u is not None else u_prev.copy()

        # Velocity weights: boosted for trajectory-aware sources (toss loop,
        # scheduler) where the twist reference is kinematically consistent.
        # Conservative for convergence sources (interactive, catch) where
        # twist_ref is zero and boosting would penalize motion toward target.
        vel_weights = (self._vel_weights_boosted if boost_vel_weights
                       else self._vel_weights_normal)
        accel_weights = self._accel_weights

        # Parameter vector — fill pre-allocated buffer in-place
        N1_6 = (self._N + 1) * 6
        p_param = self._p_param_buf
        p_param[:6] = p_cur
        p_param[6:12] = q_cur
        p_param[12:18] = u_prev
        p_param[18:24] = u_prev_prev
        off = 24
        p_param[off:off + N1_6] = ref_traj.ravel()
        off += N1_6
        p_param[off:off + N1_6] = twist_traj.ravel()
        off += N1_6
        p_param[off:off + N1_6] = accel_traj.ravel()
        off += N1_6
        p_param[off:off + 4] = vel_weights
        off += 4
        p_param[off:off + 2] = accel_weights

        # Initial guess — prefer last successful solution, then timeout
        # hint (partial/unconverged), then cold-start as last resort.
        if self._prev_w is not None:
            w0 = self._shift_warm_start(self._prev_w)
        elif self._timeout_hint is not None:
            w0 = self._shift_warm_start(self._timeout_hint)
        else:
            w0 = self._cold_start(p_cur, q_cur, ref_traj)

        # Solve — track wall-clock dt for accurate feedforward velocity
        t0 = _time.perf_counter()
        actual_dt = (t0 - self._prev_solve_time) if self._prev_solve_time is not None else None
        self._prev_solve_time = t0
        self._last_actual_dt = actual_dt
        try:
            kw = dict(
                x0=w0, p=p_param,
                lbx=self._lbw, ubx=self._ubw,
                lbg=self._lbg, ubg=self._ubg,
            )
            # Dual variable warm-start: prefer successful, fall back to timeout
            lam_g = self._prev_lam_g if self._prev_lam_g is not None else self._timeout_lam_g
            lam_x = self._prev_lam_x if self._prev_lam_x is not None else self._timeout_lam_x
            if lam_g is not None:
                kw['lam_g0'] = lam_g
            if lam_x is not None:
                kw['lam_x0'] = lam_x

            sol = self._solver(**kw)
            solve_ms = (_time.perf_counter() - t0) * 1000.0

            stats = self._solver.stats()
            ret = stats.get('return_status', 'unknown')
            iter_count = stats.get('iter_count', -1)
            success = ret in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')

            # Timeout produces a suboptimal but often usable primal guess.
            # Save it as a warm-start HINT for the next solve so it doesn't
            # cold-start (which would also timeout, creating a cascade).
            # Stored separately from _prev_w so _handle_failure never uses
            # an unconverged solution for commanding — only for initial guess.
            if not success and ret == 'Maximum_CpuTime_Exceeded':
                w_partial = np.asarray(sol['x']).ravel()
                if np.all(np.isfinite(w_partial)):
                    self._timeout_hint = w_partial
                    self._timeout_lam_g = np.asarray(sol['lam_g']).ravel()
                    self._timeout_lam_x = np.asarray(sol['lam_x']).ravel()

            if success:
                self._consecutive_failures = 0
                # Clear timeout hint — no longer needed once we have a
                # converged solution.
                self._timeout_hint = None
                self._timeout_lam_g = None
                self._timeout_lam_x = None
                w_opt = np.asarray(sol['x']).ravel()
                if not np.all(np.isfinite(w_opt)):
                    logger.warning("MPC solve returned non-finite primal solution")
                    return self._handle_failure(
                        solve_ms, 'non_finite_solution', q_cur)
                self._prev_w = w_opt
                self._prev_lam_g = np.asarray(sol['lam_g']).ravel()
                self._prev_lam_x = np.asarray(sol['lam_x']).ravel()

                cmd = w_opt[:6].copy()
                cmd_next = w_opt[6:12].copy()   # u[1]: next-step command
                cmd_next2 = w_opt[12:18].copy()  # u[2]: step after next
                prev_u = self._prev_u  # read BEFORE overwrite
                self._prev_prev_u = prev_u.copy() if prev_u is not None else u_prev.copy()
                self._prev_u = cmd
                self._extract_predicted_poses(w_opt, p_cur)

                # Compute true constraint violation (distance outside bounds).
                # Includes both g-constraints (dynamics, IK, rate limits) and
                # x-bounds (stroke, workspace) for complete diagnostics.
                g_vals = np.asarray(sol['g']).ravel()
                g_viol = np.maximum(
                    np.maximum(0.0, g_vals - self._ubg),
                    np.maximum(0.0, self._lbg - g_vals),
                )
                x_vals = w_opt
                x_viol = np.maximum(
                    np.maximum(0.0, x_vals - self._ubw),
                    np.maximum(0.0, self._lbw - x_vals),
                )
                violation = np.maximum(g_viol, x_viol)

                # Forward-looking command velocity for motor guard
                # interpolation.  Uses the MPC's own planned trajectory:
                # (u[1] - u[0]) / dt_fine, which is the derivative of the
                # command sequence the MPC just optimized.  Forward-looking
                # (not backward-differenced), so direction changes are
                # anticipated rather than reacted to.  Uses the nominal
                # dt_schedule[0] because the MPC optimized for this interval
                # regardless of wall-clock jitter.
                dt0 = self._params.dt_schedule[0]
                cmd_vel = (cmd_next - cmd) / dt0

                return cmd, cmd_vel, {
                    'solve_time_ms': solve_ms,
                    'status': ret,
                    'iter_count': iter_count,
                    'cost': float(sol['f']),
                    'constraint_violation': float(np.max(violation)),
                    'cmd_next_mm': cmd_next,
                    'cmd_next2_mm': cmd_next2,
                }
            else:
                return self._handle_failure(solve_ms, ret, q_cur)

        except Exception as exc:
            solve_ms = (_time.perf_counter() - t0) * 1000.0
            # Clear warm-start to prevent corruption cascade — the exception
            # may have left CasADi's internal state inconsistent, so reusing
            # _prev_w could cause repeated failures.  Forces cold-start on
            # the next solve.
            self._prev_w = None
            self._prev_lam_g = None
            self._prev_lam_x = None
            self._timeout_hint = None
            self._timeout_lam_g = None
            self._timeout_lam_x = None
            return self._handle_failure(solve_ms, f'exception: {exc}', q_cur)

    # ------------------------------------------------------------------
    # Warm-start helpers
    # ------------------------------------------------------------------

    def _cold_start(self, p_cur, q_cur, ref_traj):
        """Generate initial guess for the first solve (or after reset).

        Strategy:
          * Pose guess (``p`` block): use ``ref_traj`` directly — each node's
            optimal pose is the reference pose at that node (the optimiser
            only needs to tolerate tracking and dynamics constraints).
          * q / u guess: a single numerical IK call for the final reference
            pose gives ``q_final``; intermediate nodes are a linear q-space
            interpolation from ``q_cur`` toward ``q_final``.  The IK
            equality constraints will be driven to feasibility by IPOPT;
            the guess only needs to be close.

        Why one IK call instead of N:
          * N IK calls per cold-start risk FK non-convergence — observed 5
            in a 15s run during solver saturation (2026-04-17-130811).
          * A monotonic q-space sweep is a cleaner seed than N
            independently-IK'd poses (which may overshoot individual nodes).
        """
        N = self._N
        margin = self._params.stroke_margin_mm
        stroke = self._params.stroke_mm
        w0 = np.zeros(self._n_w)

        q_final = np.clip(self._numerical_ik(ref_traj[N]), 0.0, stroke)

        for k in range(N):
            alpha = (k + 1) / N
            # p: use the node's reference pose directly
            w0[12 * N + k * 6: 12 * N + (k + 1) * 6] = ref_traj[k + 1]
            # q: linear interpolate in joint space from q_cur toward q_final
            q_k = q_cur * (1.0 - alpha) + q_final * alpha
            w0[6 * N + k * 6: 6 * N + (k + 1) * 6] = q_k
            # u: clamp q_k to command bounds
            w0[k * 6: (k + 1) * 6] = np.clip(q_k, margin, stroke - margin)
        return w0

    def _numerical_ik(self, pose_6dof: np.ndarray) -> np.ndarray:
        """Compute STOW-relative leg extensions for a pose using numpy IK."""
        pos = pose_6dof[:3]
        rv = pose_6dof[3:]
        # Rodrigues rotation
        angle = np.linalg.norm(rv)
        if angle < 1e-10:
            R = np.eye(3)
        else:
            K = np.array([[0, -rv[2], rv[1]], [rv[2], 0, -rv[0]], [-rv[1], rv[0], 0]])
            R = np.eye(3) + np.sin(angle) / angle * K + (1 - np.cos(angle)) / (angle * angle) * K @ K
        exts = np.empty(6)
        for i in range(6):
            plat_w = pos + self._init_height_vec + R @ self._plat_nodes[i]
            leg_vec = plat_w - self._base_nodes[i]
            exts[i] = np.linalg.norm(leg_vec) - self._init_leg_lengths[i]
        return exts

    def _prime_solver(self) -> None:
        """Invoke the solver once at construction to pay cold-start costs upfront.

        The first call into any CasADi nlpsol instance pays one-time costs
        that are independent of the problem data: function-table warmup,
        page faults on the IPOPT shared libraries, BLAS/LAPACK init, and
        (for JIT backends) C compilation of the callback functions.  Doing
        this in ``__init__`` keeps that cost out of the operator-visible
        control loop — the first real solve then runs at steady-state
        latency (~3-15ms) instead of 27-100ms.

        The solve itself uses neutral parameters (home pose, flat reference
        at home) and may legitimately time out within the configured
        ``max_cpu_time``.  That is fine — the warmup work happens during
        the first callback evaluations regardless of whether IPOPT
        converges.  Outputs are discarded; no warm-start state is seeded,
        which would otherwise bias the first real solve toward home.
        """
        N = self._N
        home_pose = np.zeros(6)
        home_q = np.zeros(6)
        # Flat reference: all nodes hold at home pose with zero twist/accel.
        ref_traj = np.tile(home_pose, (N + 1, 1))
        twist_traj = np.zeros((N + 1, 6))
        accel_traj = np.zeros((N + 1, 6))

        # Pack parameters using the same layout as solve().
        par = self._params
        vel_weights = np.array([
            par.Q_vel_lin, par.Q_vel_ang,
            par.Qf_vel_lin, par.Qf_vel_ang,
        ])
        accel_weights = np.array([par.Q_accel_lin, par.Q_accel_ang])
        N1_6 = (self._N + 1) * 6
        p_param = np.empty(self._n_param)
        p_param[:6] = home_pose
        p_param[6:12] = home_q
        p_param[12:18] = home_q  # u_prev
        p_param[18:24] = home_q  # u_prev_prev
        off = 24
        p_param[off:off + N1_6] = ref_traj.ravel(); off += N1_6
        p_param[off:off + N1_6] = twist_traj.ravel(); off += N1_6
        p_param[off:off + N1_6] = accel_traj.ravel(); off += N1_6
        p_param[off:off + 4] = vel_weights; off += 4
        p_param[off:off + 2] = accel_weights

        # Cold-start the decision vector the same way solve() would.
        w0 = self._cold_start(home_pose, home_q, ref_traj)

        # One solver call.  Exceptions are swallowed — a timeout or numeric
        # issue here is acceptable; the goal is warmup, not convergence.
        try:
            self._solver(
                x0=w0, p=p_param,
                lbx=self._lbw, ubx=self._ubw,
                lbg=self._lbg, ubg=self._ubg,
            )
        except Exception as exc:
            logger.info("Prime solve completed with exception (expected): %s", exc)

    def _shift_warm_start(self, prev_w):
        """Shift previous optimal solution by one timestep.

        Each of the three blocks (u, q, p) is shifted left by one node
        (6 elements), with the last node repeated.  Vectorised: one
        bulk copy per block instead of N-1 scalar-loop iterations.
        """
        N = self._N
        w0 = np.empty(self._n_w)

        # For each block (u, q, p): shift left by 6, repeat last node
        for base in (0, 6 * N, 12 * N):
            end = base + N * 6
            # Copy [1:] → [0:-1]  (shift left by one node)
            w0[base: end - 6] = prev_w[base + 6: end]
            # Repeat last node
            w0[end - 6: end] = prev_w[end - 6: end]

        return w0

    # ------------------------------------------------------------------
    # Reference construction (target-based interface)
    # ------------------------------------------------------------------

    def _build_reference(
        self,
        state: PlantState,
        target_pose: np.ndarray,
        ref_events: List[ReferenceEvent] | None = None,
        t_now: float | None = None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Construct (N+1, 6) reference arrays for the MPC horizon.

        Quintic Hermite (C2) interpolation between events with acceleration,
        cubic Hermite (C1) fallback for events without acceleration.

        If ``ref_events`` is None or empty, falls back to holding at
        ``target_pose`` with zero twist and acceleration.  All sources
        should emit ``ref_events`` via ``flat_target_to_events()`` or
        equivalent; this fallback is a safety net only.
        """
        sample_t = state.time if t_now is None else t_now
        if ref_events is not None and len(ref_events) > 0:
            return _build_reference_from_events(
                ref_events, sample_t,
                self._cumulative_times, self._N + 1,
            )

        # Safety fallback: hold at target_pose
        N = self._N
        ref_traj = np.empty((N + 1, 6))
        ref_traj[:] = target_pose
        twist_traj = np.zeros((N + 1, 6))
        accel_traj = np.zeros((N + 1, 6))
        return ref_traj, twist_traj, accel_traj

    # ------------------------------------------------------------------
    # Failure handling
    # ------------------------------------------------------------------

    def _handle_failure(self, solve_ms, status_str, q_cur=None):
        """Fallback strategy on solver failure.

        Returns (cmd, cmd_vel, diag) matching solve()'s return signature.
        """
        self._consecutive_failures += 1
        logger.warning(
            "MPC solve failed (%d consecutive): %s (%.1f ms)",
            self._consecutive_failures, status_str, solve_ms,
        )

        diag = {
            'solve_time_ms': solve_ms,
            'status': f'fallback({status_str})',
            'cost': 0.0,
            'constraint_violation': 0.0,
            'cmd_next_mm': None,
            'cmd_next2_mm': None,
        }

        stroke = self._params.stroke_mm
        margin = self._params.stroke_margin_mm
        dt0 = self._params.dt_schedule[0]

        if (self._prev_w is not None
                and self._consecutive_failures <= self._params.max_consecutive_failures):
            # Apply first step of shifted previous solution
            shifted = self._shift_warm_start(self._prev_w)
            cmd = np.clip(shifted[:6], margin, stroke - margin)
            # Rate-limit against current state to prevent jumps when the
            # plant has drifted since the last successful solve.
            if q_cur is not None:
                max_delta = self._params.max_leg_vel_mmps * dt0
                cmd = np.clip(cmd, q_cur - max_delta, q_cur + max_delta)
            prev_u = self._prev_u
            self._prev_prev_u = prev_u.copy() if prev_u is not None else cmd.copy()
            ff_dt = max(self._last_actual_dt, dt0 * 0.5) if self._last_actual_dt is not None else dt0
            cmd_vel = (cmd - prev_u) / ff_dt if prev_u is not None else np.zeros(6)
            self._prev_u = cmd
            return cmd, cmd_vel, diag

        if self._prev_u is not None:
            # Hold last command — Δu = 0 so prev_prev = prev
            self._prev_prev_u = self._prev_u.copy()
            diag['status'] = f'hold({status_str})'
            return self._prev_u.copy(), np.zeros(6), diag

        # Absolute fallback: hold at current position if known, otherwise
        # margin-safe minimum.  Using q_cur prevents the dangerous ~150mm
        # command jump that occurs when the solver times out on first solve
        # while the platform is at Active (~154mm).
        diag['status'] = f'cold_hold({status_str})'
        if q_cur is not None:
            cmd = np.clip(q_cur.copy(), margin, stroke - margin)
            self._prev_u = cmd
            return cmd, np.zeros(6), diag
        return np.full(6, margin), np.zeros(6), diag

    # ------------------------------------------------------------------
    # Post-solve extraction
    # ------------------------------------------------------------------

    def _extract_predicted_poses(self, w_opt, p_cur):
        """Store N+1 predicted platform poses from the solution."""
        N = self._N
        self._predicted_poses[0] = p_cur
        self._predicted_poses[1:] = w_opt[12 * N: 18 * N].reshape(N, 6)
        self._predicted_poses_valid = True

    # ------------------------------------------------------------------
    # Public properties
    # ------------------------------------------------------------------

    @property
    def predicted_poses(self) -> np.ndarray | None:
        """(N+1, 6) predicted platform poses from last solve, or None.

        Returns a copy — safe for external consumers and tests.
        For hot-path use (control loop), prefer ``predicted_poses_view``.
        """
        if not self._predicted_poses_valid:
            return None
        return self._predicted_poses.copy()

    @property
    def predicted_poses_view(self) -> np.ndarray | None:
        """(N+1, 6) predicted poses — non-copying read-only view.

        Returns the internal array directly.  Callers MUST NOT modify it.
        Use this in the control loop hot path to avoid per-step allocation.
        """
        if not self._predicted_poses_valid:
            return None
        return self._predicted_poses

    @property
    def predicted_times(self) -> np.ndarray:
        """(N+1,) cumulative times from current step (0 to horizon_s).

        Returns a copy — safe for external consumers and tests.
        For hot-path use, prefer ``predicted_times_view``.
        """
        return self._cumulative_times.copy()

    @property
    def predicted_times_view(self) -> np.ndarray:
        """(N+1,) cumulative times — non-copying read-only view.

        The underlying array is immutable (writeable=False).
        """
        return self._cumulative_times

    @property
    def last_ref_traj(self) -> np.ndarray | None:
        """(N+1, 6) reference trajectory from last solve, or None."""
        if self._last_ref_traj is None:
            return None
        return self._last_ref_traj.copy()

    @property
    def last_twist_traj(self) -> np.ndarray | None:
        """(N+1, 6) reference twist trajectory from last solve, or None."""
        if self._last_twist_traj is None:
            return None
        return self._last_twist_traj.copy()

    @property
    def last_accel_traj(self) -> np.ndarray | None:
        """(N+1, 6) reference accel trajectory from last solve, or None."""
        if self._last_accel_traj is None:
            return None
        return self._last_accel_traj.copy()

    @property
    def consecutive_failures(self) -> int:
        return self._consecutive_failures

    @property
    def params(self) -> MPCParams:
        return self._params

    def reset(self):
        """Clear all warm-start state (call after plant reset)."""
        self._prev_w = None
        self._prev_lam_g = None
        self._prev_lam_x = None
        self._timeout_hint = None
        self._timeout_lam_g = None
        self._timeout_lam_x = None
        self._prev_u = None
        self._prev_prev_u = None
        self._consecutive_failures = 0
        self._predicted_poses_valid = False
        self._last_ref_traj = None
        self._last_twist_traj = None
        self._last_accel_traj = None
