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
    Actuator dynamics:  q[k+1] = q[k] + (u[k] - q[k]) · α_k,  α_k = 1 - exp(-dt_k/τ)
    IK consistency:     ‖leg_vec_i(p[k])‖ - init_len_i = q[k]_i + home_ext_i
    Rate limits:        |u[k] - u[k-1]| ≤ v_max · dt_between
"""

from __future__ import annotations

import logging
import math
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
        # + p_ref(6*(N+1)) + twist_ref(6*(N+1)) + urgency(N)
        n_param = 24 + 12 * (N + 1) + N
        P = cs.SX.sym('P', n_param)

        p_init = P[:6]
        q_init = P[6:12]
        u_prev_sym = P[12:18]
        u_prev_prev_sym = P[18:24]
        ref_offset = 24
        p_ref = [P[ref_offset + k * 6: ref_offset + (k + 1) * 6] for k in range(N + 1)]
        twist_offset = ref_offset + 6 * (N + 1)
        twist_ref = [P[twist_offset + k * 6: twist_offset + (k + 1) * 6] for k in range(N + 1)]
        urgency_offset = twist_offset + 6 * (N + 1)
        urgency = [P[urgency_offset + k] for k in range(N)]  # multiplier for nodes 1..N

        # ---- Decision variables -------------------------------------------
        # Layout: u[0..N-1](6N)  |  q[1..N](6N)  |  p[1..N](6N)
        n_w = 18 * N
        W = cs.SX.sym('W', n_w)

        u = [W[k * 6: (k + 1) * 6] for k in range(N)]
        q = [q_init] + [W[6 * N + k * 6: 6 * N + (k + 1) * 6] for k in range(N)]
        p = [p_init] + [W[12 * N + k * 6: 12 * N + (k + 1) * 6] for k in range(N)]

        # ---- Cost ---------------------------------------------------------
        home_ext_dm = cs.DM(self._home_ext)
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
            dt_k = dt_schedule[k - 1]  # dt of the interval ending at node k
            urg_k = urgency[k - 1]     # urgency multiplier for this node

            if k < N:
                J += Qp * urg_k * cs.dot(err[:3], err[:3])
                J += Qo * urg_k * cs.dot(err[3:], err[3:])
            else:
                # Terminal cost (heavier)
                J += Qfp * urg_k * cs.dot(err[:3], err[:3])
                J += Qfo * urg_k * cs.dot(err[3:], err[3:])

            # Velocity tracking (finite-difference twist vs reference twist)
            dp = (p[k] - p[k - 1]) / dt_k
            twist_err = dp - twist_ref[k]
            J += Qvl * urg_k * cs.dot(twist_err[:3], twist_err[:3])
            J += Qva * urg_k * cs.dot(twist_err[3:], twist_err[3:])

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

            # Control effort (relative to home extensions to avoid bias at elevated poses)
            u_dev = u[k] - home_ext_dm
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

        for k in range(N):
            # u bounds [0, stroke] — full physical range
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
        self._dt_schedule = dt_schedule
        self._cumulative_times = self._params.cumulative_times
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
        target_pose: np.ndarray,
        *,
        arrival_time: float | None = None,
        target_twist: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict]:
        """Solve MPC for one step.

        Parameters
        ----------
        state : PlantState
            Current plant state.
        target_pose : (6,) ndarray
            Target pose ``[x, y, z, rx, ry, rz]`` (mm, rad).
        arrival_time : float or None
            Absolute time by which the platform should reach the target pose.
            None = "as soon as possible" (drive to target as fast as
            constraints allow).
        target_twist : (6,) ndarray or None
            Desired twist at arrival.  None = zero (hold at target).

        Returns
        -------
        cmd : (6,) ndarray
            Home-relative leg extension command (mm).
        diag : dict
            Diagnostics: solve_time_ms, status, cost, constraint_violation.
        """
        N = self._N

        target = np.asarray(target_pose, dtype=float)
        tw = None
        if target_twist is not None:
            tw = np.asarray(target_twist, dtype=float)
        ref_traj, twist_traj = self._build_reference(
            state, target, arrival_time, tw)
        urgency = self._compute_urgency(state.time, arrival_time)

        # Current state
        p_cur = np.concatenate([state.platform_pos_mm, state.platform_rot])
        q_cur = state.leg_extensions_mm.copy()
        u_prev = self._prev_u if self._prev_u is not None else q_cur.copy()
        u_prev_prev = self._prev_prev_u if self._prev_prev_u is not None else u_prev.copy()

        # Parameter vector
        p_param = np.concatenate([
            p_cur, q_cur, u_prev, u_prev_prev,
            ref_traj.ravel(), twist_traj.ravel(),
            urgency,
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
        margin = self._params.stroke_margin_mm
        stroke = self._params.stroke_mm
        t_cumulative = self._cumulative_times  # (N+1,) array starting at 0
        t_total = t_cumulative[-1]
        w0 = np.zeros(self._n_w)
        for k in range(N):
            alpha = t_cumulative[k + 1] / t_total
            # p: interpolate from current pose toward final reference
            p_k = p_cur * (1.0 - alpha) + ref_traj[-1] * alpha
            w0[12 * N + k * 6: 12 * N + (k + 1) * 6] = p_k
            # q: compute IK-consistent extensions (full physical range)
            q_k = np.clip(self._numerical_ik(p_k), 0.0, stroke)
            w0[6 * N + k * 6: 6 * N + (k + 1) * 6] = q_k
            # u: clamp to command bounds [margin, stroke - margin]
            u_k = np.clip(q_k, margin, stroke - margin)
            w0[k * 6: (k + 1) * 6] = u_k
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
    # Reference construction (target-based interface)
    # ------------------------------------------------------------------

    def _build_reference(
        self,
        state: PlantState,
        target_pose: np.ndarray,
        arrival_time: float | None,
        target_twist: np.ndarray | None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Construct (N+1, 6) reference arrays from a target pose + timing.

        All reference nodes are set to the target pose.  The MPC finds the
        optimal feasible path via its cost function and constraints — no
        intermediate waypoints are imposed.  The urgency system
        (``_compute_urgency``) provides timing incentive for timed targets.

        Twist reference is zero before the deadline (let MPC plan velocity)
        and ``target_twist`` at/after the deadline.
        """
        N = self._N
        ref_traj = np.empty((N + 1, 6))
        twist_traj = np.zeros((N + 1, 6))

        tw = target_twist if target_twist is not None else np.zeros(6)

        ref_traj[:] = target_pose

        if arrival_time is not None:
            time_budget = arrival_time - state.time
            if time_budget <= 0:
                # Past deadline: apply target twist everywhere
                twist_traj[:] = tw
            else:
                # Apply target twist only at/past the deadline
                t_nodes = self._cumulative_times
                for k in range(N + 1):
                    if t_nodes[k] >= time_budget:
                        twist_traj[k] = tw

        return ref_traj, twist_traj

    def _compute_urgency(
        self,
        t_now: float,
        arrival_time: float | None,
    ) -> np.ndarray:
        """Compute per-node urgency multipliers for the tracking cost.

        Returns (N,) array of multipliers for nodes 1..N.

        - ASAP mode (arrival_time is None): uniform 1.0.
        - Timed mode: ``urgency_base`` far from the deadline, ramping
          linearly to ``urgency_max`` over the ``urgency_ramp_s`` window.
          The low base lets the MPC choose its own path to the target
          without heavy cost pressure on nodes that are far from feasible.
          The terminal cost (Qf) and ramp ensure accurate arrival.
        """
        N = self._N
        if arrival_time is None:
            return np.ones(N)

        ramp_s = self._params.urgency_ramp_s
        urg_base = self._params.urgency_base
        urg_max = self._params.urgency_max
        t_nodes = self._cumulative_times  # (N+1,) relative from t_now

        urgency = np.full(N, urg_base)
        for k in range(N):
            t_k = t_now + t_nodes[k + 1]  # absolute time of node k+1
            time_to_deadline = max(arrival_time - t_k, 0.0)
            ramp = max(0.0, 1.0 - time_to_deadline / ramp_s) if ramp_s > 0 else 1.0
            urgency[k] = urg_base + (urg_max - urg_base) * ramp

        return urgency

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

        stroke = self._params.stroke_mm
        margin = self._params.stroke_margin_mm

        if (self._prev_w is not None
                and self._consecutive_failures <= self._params.max_consecutive_failures):
            # Apply first step of shifted previous solution
            shifted = self._shift_warm_start(self._prev_w)
            cmd = np.clip(shifted[:6], margin, stroke - margin)
            self._prev_prev_u = self._prev_u.copy() if self._prev_u is not None else cmd.copy()
            self._prev_u = cmd
            return cmd, diag

        if self._prev_u is not None:
            # Hold last command — Δu = 0 so prev_prev = prev
            self._prev_prev_u = self._prev_u.copy()
            diag['status'] = f'hold({status_str})'
            return self._prev_u.copy(), diag

        # Absolute fallback: hold at margin-safe position (lowest feasible)
        diag['status'] = f'cold_hold({status_str})'
        return np.full(6, margin), diag

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
        if self._predicted_poses is None:
            return None
        return self._predicted_poses.copy()

    @property
    def predicted_times(self) -> np.ndarray:
        """(N+1,) cumulative times from current step (0 to horizon_s)."""
        return self._cumulative_times.copy()

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
