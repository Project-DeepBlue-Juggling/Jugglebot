"""CasADi-based NMPC for Stewart platform pose tracking.

Implements the "IK as equality constraint" approach from the development plan:
the platform pose at each horizon step is a decision variable, constrained to
be kinematically consistent with the leg extensions via the Stewart platform IK
equations expressed symbolically in CasADi.

Decision variables (per horizon, N steps):
    u[0..N-1]  (6 each) : commanded leg extensions (mm, home-relative)
    q[1..N]    (6 each) : actual leg extensions after actuator lag
    p[1..N]    (6 each) : platform pose [x, y, z, rx, ry, rz] (mm, rad)

Parameters (set each solve):
    p_init  (6)        : current platform pose
    q_init  (6)        : current actual leg extensions
    u_prev  (6)        : previous commanded extensions (smoothness term)
    p_ref   (6*(N+1))  : reference pose at each horizon step

Constraints:
    Actuator dynamics:  q[k+1] = q[k] + (u[k] - q[k]) · dt/τ
    IK consistency:     ‖leg_vec_i(p[k])‖ - init_len_i = q[k]_i + home_ext_i
    Rate limits:        |u[k] - u[k-1]| ≤ v_max · dt
"""

from __future__ import annotations

import logging
import time as _time

import numpy as np

logger = logging.getLogger(__name__)

try:
    import casadi as cs
except ImportError:
    cs = None


from plant.interface import PlantState
from .params import MPCParams


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
        home_extensions_mm: np.ndarray,
    ):
        if cs is None:
            raise ImportError("CasADi is required for MPC — pip install casadi")

        self._params = params
        self._home_ext = np.asarray(home_extensions_mm, dtype=float)
        self._init_height_mm = init_height_mm

        # Geometry (plain arrays for CasADi)
        self._base_nodes = np.asarray(base_nodes, dtype=float)
        self._plat_nodes = np.asarray(plat_nodes, dtype=float)
        self._init_height_vec = np.array([0.0, 0.0, init_height_mm])
        self._init_leg_lengths = np.asarray(init_leg_lengths_mm, dtype=float)

        self._build_problem()

        # Warm-start state
        self._prev_w: np.ndarray | None = None
        self._prev_lam_g: np.ndarray | None = None
        self._prev_lam_x: np.ndarray | None = None
        self._prev_u: np.ndarray | None = None
        self._prev_prev_u: np.ndarray | None = None
        self._consecutive_failures: int = 0
        self._predicted_poses: np.ndarray | None = None

    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------

    @classmethod
    def from_plant(cls, params: MPCParams, plant) -> MPCController:
        """Create controller from a MuJoCoPlant instance."""
        geom = plant.geom
        return cls(
            params=params,
            base_nodes=geom.base_nodes,
            plat_nodes=geom.plat_nodes,
            init_height_mm=geom.init_height_mm,
            init_leg_lengths_mm=geom.init_leg_lengths_mm,
            home_extensions_mm=plant.home_extensions_mm,
        )

    # ------------------------------------------------------------------
    # NLP construction (called once)
    # ------------------------------------------------------------------

    def _build_problem(self):
        N = self._params.N
        dt = self._params.dt
        tau = self._params.tau
        stroke = self._params.stroke_mm
        v_max_dt = self._params.max_leg_vel_mmps * dt

        sym_ik = _build_symbolic_ik(
            self._base_nodes, self._plat_nodes,
            self._init_height_vec, self._init_leg_lengths,
        )

        # ---- Parameters (change every solve) ------------------------------
        # p_init(6) + q_init(6) + u_prev(6) + u_prev_prev(6)
        # + p_ref(6*(N+1)) + twist_ref(6*(N+1))
        n_param = 24 + 12 * (N + 1)
        P = cs.SX.sym('P', n_param)

        p_init = P[:6]
        q_init = P[6:12]
        u_prev_sym = P[12:18]
        u_prev_prev_sym = P[18:24]
        ref_offset = 24
        p_ref = [P[ref_offset + k * 6: ref_offset + (k + 1) * 6] for k in range(N + 1)]
        twist_offset = ref_offset + 6 * (N + 1)
        twist_ref = [P[twist_offset + k * 6: twist_offset + (k + 1) * 6] for k in range(N + 1)]

        # ---- Decision variables -------------------------------------------
        # Layout: u[0..N-1](6N)  |  q[1..N](6N)  |  p[1..N](6N)
        n_w = 18 * N
        W = cs.SX.sym('W', n_w)

        u = [W[k * 6: (k + 1) * 6] for k in range(N)]
        q = [q_init] + [W[6 * N + k * 6: 6 * N + (k + 1) * 6] for k in range(N)]
        p = [p_init] + [W[12 * N + k * 6: 12 * N + (k + 1) * 6] for k in range(N)]

        # ---- Cost ---------------------------------------------------------
        Qp = self._params.Q_pos
        Qo = self._params.Q_ori
        Qfp = self._params.Qf_pos
        Qfo = self._params.Qf_ori
        Qvl = self._params.Q_vel_lin
        Qva = self._params.Q_vel_ang
        R_w = self._params.R
        S_w = self._params.S
        A_w = self._params.A

        J = 0.0

        for k in range(1, N + 1):
            err = p[k] - p_ref[k]
            if k < N:
                J += Qp * cs.dot(err[:3], err[:3])
                J += Qo * cs.dot(err[3:], err[3:])
            else:
                # Terminal cost (heavier)
                J += Qfp * cs.dot(err[:3], err[:3])
                J += Qfo * cs.dot(err[3:], err[3:])

            # Velocity tracking (finite-difference twist vs reference twist)
            dp = (p[k] - p[k - 1]) / dt
            twist_err = dp - twist_ref[k]
            J += Qvl * cs.dot(twist_err[:3], twist_err[:3])
            J += Qva * cs.dot(twist_err[3:], twist_err[3:])

        for k in range(N):
            # Control effort
            J += R_w * cs.dot(u[k], u[k])
            # Smoothness (velocity penalty)
            du_k = u[k] - (u_prev_sym if k == 0 else u[k - 1])
            J += S_w * cs.dot(du_k, du_k)
            # Acceleration smoothness (penalises change in Δu)
            if k == 0:
                du_prev = u_prev_sym - u_prev_prev_sym
            elif k == 1:
                du_prev = u[0] - u_prev_sym
            else:
                du_prev = u[k - 1] - u[k - 2]
            ddu = du_k - du_prev
            J += A_w * cs.dot(ddu, ddu)

        # ---- Constraints --------------------------------------------------
        g_list = []

        # 1. Actuator dynamics  (6·N equality)
        for k in range(N):
            q_pred = q[k] + (u[k] - q[k]) * (dt / tau)
            g_list.append(q[k + 1] - q_pred)

        # 2. IK consistency  (6·N equality)
        home_ext_dm = cs.DM(self._home_ext)
        for k in range(1, N + 1):
            ik_ext = sym_ik(p[k])
            g_list.append(ik_ext - (q[k] + home_ext_dm))

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
        lbg[n_eq:] = -v_max_dt
        ubg[n_eq:] = v_max_dt

        # ---- Variable bounds ----------------------------------------------
        lbw = np.full(n_w, -np.inf)
        ubw = np.full(n_w, np.inf)

        for k in range(N):
            # u bounds [0, stroke]
            lbw[k * 6: (k + 1) * 6] = 0.0
            ubw[k * 6: (k + 1) * 6] = stroke
            # q bounds [0, stroke]
            lbw[6 * N + k * 6: 6 * N + (k + 1) * 6] = 0.0
            ubw[6 * N + k * 6: 6 * N + (k + 1) * 6] = stroke
            # p bounds (workspace limits — generous, stroke limits are the real safety net)
            bp = 12 * N + k * 6
            lbw[bp: bp + 2] = -200.0       # x, y (mm)
            ubw[bp: bp + 2] = 200.0
            lbw[bp + 2] = -50.0            # z (mm) — can't go below home much
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
        }
        if self._params.warm_start:
            opts['ipopt.warm_start_init_point'] = 'yes'
            opts['ipopt.warm_start_bound_push'] = 1e-8
            opts['ipopt.warm_start_mult_bound_push'] = 1e-8

        self._solver = cs.nlpsol('mpc', 'ipopt', nlp, opts)
        self._n_w = n_w
        self._N = N
        self._lbw = lbw
        self._ubw = ubw
        self._lbg = lbg
        self._ubg = ubg

    # ------------------------------------------------------------------
    # Solve
    # ------------------------------------------------------------------

    def solve(
        self,
        state: PlantState,
        reference: np.ndarray,
        ref_twist: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict]:
        """Solve MPC for one step.

        Parameters
        ----------
        state : PlantState
            Current plant state.
        reference : (6,) or (N+1, 6) ndarray
            Static target pose, or per-step reference trajectory.
        ref_twist : (N+1, 6) ndarray or None
            Per-step reference twist for velocity tracking.
            If None, zero twist is used (static pose tracking).

        Returns
        -------
        cmd : (6,) ndarray
            Home-relative leg extension command (mm).
        diag : dict
            Diagnostics: solve_time_ms, status, cost, constraint_violation.
        """
        N = self._N

        ref = np.asarray(reference, dtype=float)
        if ref.ndim == 1:
            ref_traj = np.tile(ref, (N + 1, 1))
        else:
            ref_traj = ref

        if ref_twist is not None:
            twist_traj = np.asarray(ref_twist, dtype=float)
        else:
            twist_traj = np.zeros((N + 1, 6))

        # Current state
        p_cur = np.concatenate([state.platform_pos_mm, state.platform_rot])
        q_cur = state.leg_extensions_mm.copy()
        u_prev = self._prev_u if self._prev_u is not None else q_cur.copy()
        u_prev_prev = self._prev_prev_u if self._prev_prev_u is not None else u_prev.copy()

        # Parameter vector
        p_param = np.concatenate([
            p_cur, q_cur, u_prev, u_prev_prev,
            ref_traj.ravel(), twist_traj.ravel(),
        ])

        # Initial guess
        if self._prev_w is not None:
            w0 = self._shift_warm_start(self._prev_w)
        else:
            w0 = self._cold_start(p_cur, q_cur, ref_traj)

        # Solve
        t0 = _time.perf_counter()
        try:
            kw = dict(
                x0=w0, p=p_param,
                lbx=self._lbw, ubx=self._ubw,
                lbg=self._lbg, ubg=self._ubg,
            )
            if self._prev_lam_g is not None:
                kw['lam_g0'] = self._prev_lam_g
            if self._prev_lam_x is not None:
                kw['lam_x0'] = self._prev_lam_x

            sol = self._solver(**kw)
            solve_ms = (_time.perf_counter() - t0) * 1000.0

            stats = self._solver.stats()
            ret = stats.get('return_status', 'unknown')
            success = ret in ('Solve_Succeeded', 'Solved_To_Acceptable_Level')

            if success:
                self._consecutive_failures = 0
                w_opt = np.asarray(sol['x']).ravel()
                self._prev_w = w_opt
                self._prev_lam_g = np.asarray(sol['lam_g']).ravel()
                self._prev_lam_x = np.asarray(sol['lam_x']).ravel()

                cmd = w_opt[:6].copy()
                self._prev_prev_u = self._prev_u.copy() if self._prev_u is not None else u_prev.copy()
                self._prev_u = cmd
                self._extract_predicted_poses(w_opt, p_cur)

                # Compute true constraint violation (distance outside bounds)
                g_vals = np.asarray(sol['g']).ravel()
                violation = np.maximum(
                    np.maximum(0.0, g_vals - self._ubg),
                    np.maximum(0.0, self._lbg - g_vals),
                )

                return cmd, {
                    'solve_time_ms': solve_ms,
                    'status': ret,
                    'cost': float(sol['f']),
                    'constraint_violation': float(np.max(violation)),
                }
            else:
                return self._handle_failure(solve_ms, ret)

        except Exception as exc:
            solve_ms = (_time.perf_counter() - t0) * 1000.0
            return self._handle_failure(solve_ms, f'exception: {exc}')

    # ------------------------------------------------------------------
    # Warm-start helpers
    # ------------------------------------------------------------------

    def _cold_start(self, p_cur, q_cur, ref_traj):
        """Generate IK-consistent initial guess by interpolating toward the reference.

        Interpolates pose linearly from current to final reference, then computes
        matching leg extensions via numerical IK so the initial guess satisfies
        the IK equality constraints approximately.
        """
        N = self._N
        w0 = np.zeros(self._n_w)
        for k in range(N):
            alpha = (k + 1) / N
            # p: interpolate from current pose toward final reference
            p_k = p_cur * (1.0 - alpha) + ref_traj[-1] * alpha
            w0[12 * N + k * 6: 12 * N + (k + 1) * 6] = p_k
            # q: compute IK-consistent extensions for this pose
            q_k = self._numerical_ik(p_k)
            w0[6 * N + k * 6: 6 * N + (k + 1) * 6] = q_k
            # u: command the same extensions (no lag in initial guess)
            w0[k * 6: (k + 1) * 6] = q_k
        return w0

    def _numerical_ik(self, pose_6dof: np.ndarray) -> np.ndarray:
        """Compute home-relative leg extensions for a pose using numpy IK."""
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
            exts[i] = np.linalg.norm(leg_vec) - self._init_leg_lengths[i] - self._home_ext[i]
        return exts

    def _shift_warm_start(self, prev_w):
        """Shift previous optimal solution by one timestep."""
        N = self._N
        w0 = np.zeros(self._n_w)

        # u: [u1*, u2*, ..., u_{N-1}*, u_{N-1}*]
        for k in range(N - 1):
            w0[k * 6: (k + 1) * 6] = prev_w[(k + 1) * 6: (k + 2) * 6]
        w0[(N - 1) * 6: N * 6] = prev_w[(N - 1) * 6: N * 6]

        # q: [q2*, q3*, ..., qN*, qN*]
        for k in range(N - 1):
            src = 6 * N + (k + 1) * 6
            dst = 6 * N + k * 6
            w0[dst: dst + 6] = prev_w[src: src + 6]
        w0[6 * N + (N - 1) * 6: 6 * N + N * 6] = prev_w[6 * N + (N - 1) * 6: 6 * N + N * 6]

        # p: [p2*, p3*, ..., pN*, pN*]
        for k in range(N - 1):
            src = 12 * N + (k + 1) * 6
            dst = 12 * N + k * 6
            w0[dst: dst + 6] = prev_w[src: src + 6]
        w0[12 * N + (N - 1) * 6: 12 * N + N * 6] = prev_w[12 * N + (N - 1) * 6: 12 * N + N * 6]

        return w0

    # ------------------------------------------------------------------
    # Failure handling
    # ------------------------------------------------------------------

    def _handle_failure(self, solve_ms, status_str):
        """Fallback strategy on solver failure."""
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
        }

        if (self._prev_w is not None
                and self._consecutive_failures <= self._params.max_consecutive_failures):
            # Apply first step of shifted previous solution
            shifted = self._shift_warm_start(self._prev_w)
            cmd = shifted[:6].copy()
            self._prev_prev_u = self._prev_u.copy() if self._prev_u is not None else cmd.copy()
            self._prev_u = cmd
            return cmd, diag

        if self._prev_u is not None:
            # Hold last command — Δu = 0 so prev_prev = prev
            self._prev_prev_u = self._prev_u.copy()
            diag['status'] = f'hold({status_str})'
            return self._prev_u.copy(), diag

        # Absolute fallback: home
        diag['status'] = f'cold_hold({status_str})'
        return np.zeros(6), diag

    # ------------------------------------------------------------------
    # Post-solve extraction
    # ------------------------------------------------------------------

    def _extract_predicted_poses(self, w_opt, p_cur):
        """Store N+1 predicted platform poses from the solution."""
        N = self._N
        poses = np.zeros((N + 1, 6))
        poses[0] = p_cur
        for k in range(N):
            poses[k + 1] = w_opt[12 * N + k * 6: 12 * N + (k + 1) * 6]
        self._predicted_poses = poses

    # ------------------------------------------------------------------
    # Public properties
    # ------------------------------------------------------------------

    @property
    def predicted_poses(self) -> np.ndarray | None:
        """(N+1, 6) predicted platform poses from last solve, or None."""
        return self._predicted_poses

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
        self._prev_u = None
        self._prev_prev_u = None
        self._consecutive_failures = 0
        self._predicted_poses = None
