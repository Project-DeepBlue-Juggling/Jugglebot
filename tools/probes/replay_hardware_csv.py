#!/usr/bin/env python
"""Production-faithful MPC replay of a recorded hardware CSV.

Mirrors ``controller/runner.py``'s hardware loop:
  - ``StaticTargetSource(schedule=[(0.0, target)], **src_kwargs)`` with
    the same kwargs ``run_mpc.py`` uses.
  - Per tick: ``tc = source.update(t_ref, state); mpc.solve(state,
    tc.target_pose, ref_events=tc.ref_events,
    boost_vel_weights=tc.boost_vel_weights, t_now=t_ref,
    warm_start_valid=tc.warm_start_valid)``.
  - ``t_ref`` advances ONLY on success-class statuses (mirrors
    ``runner.py:585-587``).
  - State per tick comes from the recorded CSV's ``actual_*`` columns.

Two modes:

  --mode single (default)
      Single MPC instance, baseline production behaviour.

  --mode compare
      Three MPC instances in parallel, each with its own
      ``_handle_failure`` override.  Used to A/B/C the fallback
      candidates investigated in the walk-forward singleton-emission
      arc.  Each instance carries its own state (``_prev_u``,
      ``_prev_w``, ``_consecutive_failures``) so the rate-limit clamps
      at the next solve are computed against each variant's actual
      prior emission, not the baseline's.

      Variants:
        baseline   — production ``MPCController`` unchanged.
        option_z   — singleton (``_consecutive_failures == 0`` before
                     this call) fires the ``hold_extrap`` arm directly;
                     ``2..max_consecutive_failures`` uses walk-forward;
                     ``>max`` keeps the existing ``hold_extrap``.
        unified    — all fallback ticks emit linear extrapolation of
                     the recent cmd stream (``cmd = 2·_prev_u −
                     _prev_prev_u``), workspace-clamped, rate-limited,
                     time-bounded at 300 ms (after which it escalates
                     to ``hold_extrap``).  Does not use ``_prev_w``.

Logbook entry: ``logbook/2026-05-20-walk-forward-singleton-emission-jerk.md``
(lands alongside the fix this probe motivates).

Usage:
    # Baseline-only repro
    python tools/probes/replay_hardware_csv.py \\
        --src temp/logs/mpc_20260520_115857.csv \\
        --target 0,0,30,0,0,0 --label singleton_repro

    # Three-way comparison
    python tools/probes/replay_hardware_csv.py \\
        --src temp/logs/mpc_20260520_115857.csv \\
        --target 0,0,30,0,0,0 --label compare_singleton \\
        --mode compare

Output:
    temp/probes/<label>.csv          per-tick replay log
    temp/probes/<label>.summary.json summary stats
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent.parent
for p in (REPO, REPO / "sim", REPO / "ros_ws" / "src" / "jugglebot"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

CONTROL_DT = 1.0 / 40.0
WARMUP_TICKS = 40
PIN_CORE = 5
_SUCCESS = ("Solve_Succeeded", "Solved_To_Acceptable_Level")
_FALLBACK_KEYWORDS = ("fallback", "hold", "cold_hold", "extrap", "unified")
UNIFIED_MAX_FALLBACK_S = 0.3  # 300 ms time bound for variant U


def _build_diag(solve_ms: float, status: str, fallback_step: int) -> dict:
    return {
        "solve_time_ms": solve_ms,
        "status": status,
        "iter_count": 0,
        "cost": 0.0,
        "constraint_violation": 0.0,
        "cmd_next_mm": None,
        "cmd_next2_mm": None,
        "fallback_step": fallback_step,
    }


def _sanitize(q_cur, q_dot, prev_u, margin):
    """Mirror base ``_handle_failure``'s NaN sanitization."""
    if q_cur is not None and not np.all(np.isfinite(q_cur)):
        q_cur = q_cur.copy()
        bad = ~np.isfinite(q_cur)
        q_cur[bad] = prev_u[bad] if prev_u is not None else margin
    if q_dot is not None and not np.all(np.isfinite(q_dot)):
        q_dot = q_dot.copy()
        q_dot[~np.isfinite(q_dot)] = 0.0
    return q_cur, q_dot


def _make_variant(base_class, variant: str):
    """Return an ``MPCController`` subclass implementing the given variant.

    Variants override ``_handle_failure`` only; the success path and all
    other state management are inherited from the base class unchanged.
    """
    import numpy as np

    if variant == "baseline":
        return base_class

    if variant == "option_z":
        class OptionZMPC(base_class):
            __doc__ = "Singleton fallback fires hold_extrap; cascades use walk-forward."

            def _handle_failure(self, solve_ms, status_str, q_cur=None, q_dot=None):
                if (self._consecutive_failures == 0
                        and q_cur is not None and q_dot is not None
                        and self._prev_u is not None):
                    # Singleton case — fire hold_extrap directly instead
                    # of walk-forward, since the prior plan's u[1] was
                    # referenced against stale state and produces the
                    # cmd_vel zigzag against tick N+1's fresh solve.
                    self._consecutive_failures += 1
                    stroke = self._params.stroke_mm
                    margin = self._params.stroke_margin_mm
                    dt0 = self._params.dt_schedule[0]
                    max_delta = self._params.max_leg_vel_mmps * dt0
                    q_cur, q_dot = _sanitize(q_cur, q_dot, self._prev_u, margin)
                    prev_u = self._prev_u
                    cmd = np.clip(q_cur + q_dot * dt0, margin, stroke - margin)
                    cmd = np.clip(cmd, prev_u - max_delta, prev_u + max_delta)
                    self._prev_prev_u = prev_u.copy()
                    self._prev_u = cmd
                    return cmd, q_dot.copy(), _build_diag(
                        solve_ms, f"hold_extrap_singleton({status_str})", 0)
                return super()._handle_failure(solve_ms, status_str, q_cur, q_dot)
        return OptionZMPC

    if variant == "unified":
        class UnifiedFallbackMPC(base_class):
            __doc__ = ("Homogeneous fallback: linear cmd-stream extrapolation, "
                       "workspace-clamped, rate-limited, time-bounded at 300 ms "
                       "(escalates to hold_extrap after).")

            def _handle_failure(self, solve_ms, status_str, q_cur=None, q_dot=None):
                self._consecutive_failures += 1
                stroke = self._params.stroke_mm
                margin = self._params.stroke_margin_mm
                dt0 = self._params.dt_schedule[0]
                max_delta = self._params.max_leg_vel_mmps * dt0
                q_cur, q_dot = _sanitize(q_cur, q_dot, self._prev_u, margin)

                # Time bound — escalate to hold_extrap if extrapolation
                # budget exhausted (caps the duration of pure cmd-stream
                # extrapolation in case the cascade is symptomatic of a
                # larger problem).
                elapsed = self._consecutive_failures * dt0
                if (elapsed > UNIFIED_MAX_FALLBACK_S
                        and q_cur is not None and q_dot is not None
                        and self._prev_u is not None):
                    prev_u = self._prev_u
                    cmd = np.clip(q_cur + q_dot * dt0, margin, stroke - margin)
                    cmd = np.clip(cmd, prev_u - max_delta, prev_u + max_delta)
                    self._prev_prev_u = prev_u.copy()
                    self._prev_u = cmd
                    return cmd, q_dot.copy(), _build_diag(
                        solve_ms, f"unified_escalate({status_str})",
                        self._consecutive_failures)

                # Linear cmd-stream extrapolation: cmd = 2·prev_u − prev_prev_u.
                # Falls back gracefully when history is incomplete.
                prev_u = self._prev_u
                prev_prev_u = self._prev_prev_u
                if prev_u is not None and prev_prev_u is not None:
                    cmd = 2.0 * prev_u - prev_prev_u
                elif prev_u is not None:
                    cmd = prev_u.copy()
                elif q_cur is not None:
                    cmd = q_cur.copy()
                else:
                    cmd = np.full(6, margin)

                # Workspace + rate limit.
                cmd = np.clip(cmd, margin, stroke - margin)
                if prev_u is not None:
                    cmd = np.clip(cmd, prev_u - max_delta, prev_u + max_delta)

                # Implied cmd_vel — derivative of the extrapolation.
                if prev_u is not None:
                    cmd_vel = (cmd - prev_u) / dt0
                else:
                    cmd_vel = np.zeros(6)

                self._prev_prev_u = prev_u.copy() if prev_u is not None else cmd.copy()
                self._prev_u = cmd

                return cmd, cmd_vel, _build_diag(
                    solve_ms, f"unified_extrap({status_str})",
                    self._consecutive_failures)
        return UnifiedFallbackMPC

    raise ValueError(f"Unknown variant: {variant!r}")


def _load_states(csv_path: Path) -> list[dict]:
    """Per-tick recorded plant state + recorded cmd."""
    rows = []
    with open(csv_path) as f:
        r = csv.DictReader(f)
        for row in r:
            g = lambda c, d=0.0: float(row[c]) if row.get(c) not in (None, "") else d
            rows.append(
                dict(
                    pos=np.array([g("actual_pose_x"), g("actual_pose_y"), g("actual_pose_z")]),
                    rot=np.array([g("actual_pose_rx"), g("actual_pose_ry"), g("actual_pose_rz")]),
                    twist=np.array([
                        g("actual_twist_vx"), g("actual_twist_vy"), g("actual_twist_vz"),
                        g("actual_twist_wx"), g("actual_twist_wy"), g("actual_twist_wz"),
                    ]),
                    ext=np.array([g(f"actual_ext_{i}") for i in range(6)]),
                    vel=np.array([g(f"leg_vel_{i}") for i in range(6)]),
                    rec_cmd=np.array([g(f"cmd_ext_{i}") for i in range(6)]),
                    rec_status=row.get("solve_status", "n/a"),
                    rec_solve_ms=g("solve_time_ms"),
                    rec_t_ref=g("t_ref_s"),
                )
            )
    return rows


def _build_mpc(variant: str, max_cpu_time: float, max_iter: int):
    from controller.mpc import MPCController
    from controller.params import MPCParams
    from plant.mujoco_plant import MuJoCoPlant

    cls = _make_variant(MPCController, variant)
    params = MPCParams(max_cpu_time=max_cpu_time, max_iter=max_iter)
    return cls.from_plant(params, MuJoCoPlant())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--src", required=True,
                    help="Path to recorded mpc_*.csv to replay.")
    ap.add_argument("--target", default="0,0,30,0,0,0",
                    help="Static target the source holds.")
    ap.add_argument("--label", required=True,
                    help="Output label (temp/probes/<label>.{csv,summary.json}).")
    ap.add_argument("--mode", choices=["single", "compare"], default="single",
                    help="single = one variant only (controlled by --variant); "
                         "compare = baseline + option_z + unified in one process "
                         "(NB: parallel-instance load perturbs CPU budget — prefer "
                         "running each --variant separately for fair A/B/C).")
    ap.add_argument("--variant", choices=["baseline", "option_z", "unified"],
                    default="baseline",
                    help="Variant to run in --mode single. Ignored in --mode compare.")
    ap.add_argument("--max-ticks", type=int, default=None,
                    help="Optional cap on ticks for smoke runs.")
    args = ap.parse_args()

    src_path = Path(args.src).resolve()
    if not src_path.exists():
        print(f"ERROR: --src not found: {src_path}", file=sys.stderr)
        return 2

    out_dir = REPO / "temp" / "probes"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_csv = out_dir / f"{args.label}.csv"
    out_summary = out_dir / f"{args.label}.summary.json"

    if PIN_CORE is not None:
        try:
            os.sched_setaffinity(0, {PIN_CORE})
        except Exception as exc:
            print(f"WARN sched_setaffinity({PIN_CORE}): {exc}", file=sys.stderr)

    from controller.plant import PlantState
    from controller.target import StaticTargetSource

    variants = [args.variant] if args.mode == "single" else ["baseline", "option_z", "unified"]

    t0 = time.perf_counter()
    mpcs = {v: _build_mpc(v, max_cpu_time=0.022, max_iter=100) for v in variants}
    build_s = time.perf_counter() - t0

    target = np.array([float(v) for v in args.target.split(",")])
    sources: dict = {}
    for v in variants:
        src_kwargs = dict(
            clamp_start_twist_mmps=mpcs[v].params.max_ref_start_twist_mmps,
            v_max_mmps=mpcs[v].params.max_leg_vel_mmps,
            tau_s=mpcs[v].params.tau,
        )
        sources[v] = StaticTargetSource(schedule=[(0.0, target)], **src_kwargs)

    states = _load_states(src_path)
    if args.max_ticks is not None:
        states = states[: args.max_ticks]
    n = len(states)

    print(f"[{args.label}] src={src_path.name} variants={variants} ticks={n} "
          f"target={target.tolist()} build={build_s:.2f}s pin=cpu{PIN_CORE}",
          flush=True)

    # Per-variant tracking.
    t_refs: dict = {v: None for v in variants}
    prev_cmds: dict = {v: None for v in variants}

    rows: list[dict] = []

    loop0 = time.perf_counter()
    for i, s in enumerate(states):
        # Single shared PlantState — all variants see the same recorded state.
        # PlantState fields aliased read-only across variants per Plant-Interface
        # P1/P2 contract (HardwarePlant.get_state writes once; consumers read).
        st = PlantState(
            leg_extensions_mm=s["ext"], leg_velocities_mmps=s["vel"],
            platform_pos_mm=s["pos"], platform_rot=s["rot"],
            platform_twist=s["twist"], time=i * CONTROL_DT,
        )

        per_variant: dict = {}
        for v in variants:
            mpc = mpcs[v]
            src = sources[v]
            if t_refs[v] is None:
                t_refs[v] = st.time

            tc = src.update(t_refs[v], st)

            ts = time.perf_counter()
            try:
                cmd, cmd_vel, diag = mpc.solve(
                    st, tc.target_pose,
                    ref_events=tc.ref_events,
                    boost_vel_weights=tc.boost_vel_weights,
                    t_now=t_refs[v],
                    warm_start_valid=tc.warm_start_valid,
                )
                status = diag.get("status", "n/a")
                it = int(diag.get("iter_count", 0))
                sms = float(diag.get("solve_time_ms", 0.0))
                fbk = int(diag.get("fallback_step", -1))
                cmd_arr = np.asarray(cmd, dtype=float).copy()
            except Exception as exc:
                status = f"PROBE_EXC:{type(exc).__name__}:{exc}"
                it = -1
                sms = (time.perf_counter() - ts) * 1000.0
                fbk = -1
                cmd_arr = np.full(6, np.nan)

            is_fallback = any(kw in status for kw in _FALLBACK_KEYWORDS)
            if not is_fallback:
                t_refs[v] += CONTROL_DT

            # Per-variant cmd step (continuity).
            if prev_cmds[v] is not None:
                step = cmd_arr - prev_cmds[v]
                max_abs_step = float(np.nanmax(np.abs(step)))
            else:
                max_abs_step = 0.0
            prev_cmds[v] = cmd_arr

            per_variant[v] = dict(
                status=status, iter=it, solve_ms=sms,
                fallback_step=fbk,
                cmd=cmd_arr, max_abs_step=max_abs_step,
            )

        # Snapshot row — wide format with per-variant columns.
        row: dict = {
            "tick": i,
            "t_wall_s": round(time.perf_counter() - loop0, 4),
            "rec_status": s["rec_status"],
            "rec_solve_ms": round(s["rec_solve_ms"], 3),
            "rec_t_ref_s": round(s["rec_t_ref"], 4),
        }
        for v in variants:
            pv = per_variant[v]
            row[f"{v}_status"] = pv["status"]
            row[f"{v}_iter"] = pv["iter"]
            row[f"{v}_solve_ms"] = round(pv["solve_ms"], 3)
            row[f"{v}_fb_step"] = pv["fallback_step"]
            row[f"{v}_max_abs_step_mm"] = round(pv["max_abs_step"], 4)
            for j in range(6):
                row[f"{v}_cmd_{j}"] = round(float(pv["cmd"][j]), 4)
        for j in range(6):
            row[f"rec_cmd_{j}"] = round(float(s["rec_cmd"][j]), 4)
        rows.append(row)

    header = list(rows[0].keys())
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header)
        w.writeheader()
        w.writerows(rows)

    # Summary stats.
    def _is_fb(status: str) -> bool:
        return any(kw in status for kw in _FALLBACK_KEYWORDS)

    summary: dict = {
        "label": args.label,
        "src": str(src_path),
        "mode": args.mode,
        "variants": variants,
        "ticks": n,
        "build_s": round(build_s, 2),
        "out_csv": str(out_csv),
    }
    rec_fb_ticks = [r["tick"] for r in rows if _is_fb(r["rec_status"])]
    summary["recorded_fb_ticks"] = rec_fb_ticks

    for v in variants:
        statuses = [r[f"{v}_status"] for r in rows]
        cmd_steps = np.array([r[f"{v}_max_abs_step_mm"] for r in rows])

        # Compute implied cmd_vel zigzag: max abs change in step from tick i
        # to tick i+1 (i.e. acceleration magnitude in mm per dt²).
        cmd_axes = np.array([[r[f"{v}_cmd_{j}"] for j in range(6)] for r in rows])
        cmd_vels = np.diff(cmd_axes, axis=0) / CONTROL_DT  # (n-1, 6) mm/s
        cmd_accels = np.diff(cmd_vels, axis=0) / CONTROL_DT  # (n-2, 6) mm/s² per axis
        # Per-tick max-abs cmd_vel change in mm/s (Δ in one tick).
        d_cmd_vel = np.diff(cmd_vels, axis=0)
        d_cmd_vel_max = np.abs(d_cmd_vel).max(axis=1) if d_cmd_vel.size else np.array([])

        # Identify fallback ticks for this variant.
        fb_ticks = [r["tick"] for r in rows if _is_fb(r[f"{v}_status"])]
        succ_ticks = [r["tick"] for r in rows if r[f"{v}_status"] in _SUCCESS]

        # cmd_step at each fallback tick + neighbours (for boundary analysis).
        fb_table = []
        for tk in fb_ticks:
            r = rows[tk]
            fb_table.append({
                "tick": tk,
                "status": r[f"{v}_status"],
                "fb_step": r[f"{v}_fb_step"],
                "cmd_step_at": r[f"{v}_max_abs_step_mm"],
                "cmd_step_prev": rows[tk - 1][f"{v}_max_abs_step_mm"] if tk >= 1 else None,
                "cmd_step_next": rows[tk + 1][f"{v}_max_abs_step_mm"] if tk + 1 < len(rows) else None,
                "d_cmd_vel_at": float(d_cmd_vel_max[tk - 1]) if (tk - 1) >= 0 and (tk - 1) < len(d_cmd_vel_max) else None,
                "d_cmd_vel_next": float(d_cmd_vel_max[tk]) if tk < len(d_cmd_vel_max) else None,
            })

        # Workspace excursion: max |cmd - margin| / |cmd - (stroke - margin)| envelope.
        params = mpcs[v]._params
        margin = params.stroke_margin_mm
        stroke = params.stroke_mm
        cmd_min = cmd_axes.min()
        cmd_max = cmd_axes.max()
        workspace_clear_low = float(cmd_min - margin)
        workspace_clear_high = float((stroke - margin) - cmd_max)

        # Tracking error proxy: deviation of cmd from recorded cmd over steady-state.
        steady = slice(WARMUP_TICKS, None)
        rec_cmd = np.array([[r[f"rec_cmd_{j}"] for j in range(6)] for r in rows])
        delta_vs_rec = np.abs(cmd_axes - rec_cmd).max(axis=1)
        delta_steady = delta_vs_rec[WARMUP_TICKS:]

        summary[f"{v}"] = {
            "n_success": len(succ_ticks),
            "n_fallback": len(fb_ticks),
            "fallback_ticks": fb_ticks if len(fb_ticks) <= 30 else
                              [f"{len(fb_ticks)} ticks, first/last: {fb_ticks[0]}..{fb_ticks[-1]}"],
            "cmd_step_max_mm": round(float(cmd_steps.max()), 4),
            "cmd_step_p99_mm": round(float(np.percentile(cmd_steps, 99)), 4),
            "cmd_step_at_fb_ticks": fb_table,
            "d_cmd_vel_max_mm_per_s": round(float(d_cmd_vel_max.max()), 2)
                                       if d_cmd_vel_max.size else None,
            "d_cmd_vel_p99_mm_per_s": round(float(np.percentile(d_cmd_vel_max, 99)), 2)
                                       if d_cmd_vel_max.size else None,
            "workspace_margin_low_mm": round(workspace_clear_low, 3),
            "workspace_margin_high_mm": round(workspace_clear_high, 3),
            "delta_vs_recorded_max_mm_steady": round(float(delta_steady.max()), 4)
                                                if delta_steady.size else None,
            "delta_vs_recorded_p95_mm_steady": round(float(np.percentile(delta_steady, 95)), 4)
                                                if delta_steady.size else None,
        }

    with open(out_summary, "w") as f:
        json.dump(summary, f, indent=2)
    print("SUMMARY " + json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
