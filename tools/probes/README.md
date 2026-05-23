# tools/probes/ — Reusable Empirical-Probe Harnesses

Probes fall into three families:

1. **Offline drivers** — scripts that drive the MPC / controller stack in
   a production-faithful way from recorded telemetry, to reproduce
   hardware failures, characterise edge cases, or empirically confirm
   threshold values before they get encoded in tests.
2. **Live observers** — scripts that record system state alongside a live
   session (real robot, real ROS2 stack, real MPC) so a later analysis
   pass can attribute failures to hardware bottlenecks (CPU saturation,
   thermal / DVFS throttling, memory bandwidth, etc).
3. **Characterisation harnesses** — scripts that drive a production code
   path across a pinned parameter sweep to quantify its behaviour
   (scaling laws, discontinuities, limits) before a redesign. No hardware,
   no recorded telemetry — the production code is the subject under test.

**These probes are committed to the repo** — distinct from one-off
exploratory probes which still go to `/tmp/probe_*.py`. The split is
codified in `CLAUDE.md`'s "Empirical probe before writing tests" rule.

## Why committed probes exist

Logbook entries and test docstrings reference probes by path. When the
probe lives in `/tmp/`, any reboot wipes the reference. Reusable
harnesses — anything a future investigation is likely to re-run or
adapt — go here so the references stay live.

## Probe conventions

- **Outputs**: probes write CSV / JSON / report files to `temp/probes/`
  (auto-gitignored under the umbrella `temp/` rule). Never write
  outputs inside `tools/`.
- **Header docstring**: each probe's top-of-file docstring states what
  the probe replays / characterises, the logbook entry that motivated
  it, and the test (if any) whose empirical recipe it underpins.
- **Self-contained**: probes set up `sys.path` themselves so they run
  with `python tools/probes/<name>.py ...` from the repo root without
  needing the project venv to be on `PYTHONPATH`.
- **Offline drivers — no hardware**: read recorded telemetry CSVs from
  `temp/logs/` and drive the MPC stack via Python imports, not ROS2 /
  ZeroMQ. Reproducibility against the pinned dependency stack is the
  whole point.
- **Live observers — hardware-allowed, side-effect-free**: may launch
  alongside a real session, but must only *observe* (read `/proc`,
  spawn `tegrastats` / `pidstat`, attach `py-spy`). Never command the
  motors, never send ROS2 messages — that's `tests/hardware/` territory.

## Available probes

### Offline drivers

| Probe | Purpose | Motivating logbook entry |
|-------|---------|--------------------------|
| `replay_hardware_csv.py` | Production-faithful replay of a recorded `mpc_*.csv` through `MPCController` exactly as `controller/runner.py` does. Reproduces solver-failure scenarios (CTE singletons, cascades, walk-forward emission discontinuities) offline against the pinned dependency stack. | `logbook/2026-05-20-walk-forward-singleton-emission-jerk.md` (Task 2 of post-warm-start-fix arc; logbook landing alongside the fix) |

### Live observers

| Probe | Purpose | Motivating logbook entry |
|-------|---------|--------------------------|
| `profile_session.py` | Records `tegrastats` (CPU/EMC/thermal/power), `pidstat` (per-process CPU / context switches / IO wait), `mpstat` (per-core utilisation), optional `py-spy` flame graphs per process name (`--py-spy-targets run_mpc.py,can_node`), optional `candump` CAN trace (`--candump can0`), plus a pre-flight snapshot (nvpmodel / governors / max freqs / ptrace_scope). Auto-analyses into a `report.md` that flags CPU saturation, DVFS throttling, thermal warnings, EMC saturation, memory pressure, top processes, CAN frame rate / top arbitration IDs, and cross-references can_node CPU% against CAN traffic to estimate per-frame dispatch cost. Subcommands: `start` / `stop` / `record` (Ctrl-C-able blocking mode) / `analyze`. | `logbook/2026-05-22-mpc-compute-bound-jetson-profiling.md` — the investigation that built this harness and used it to show the MPC is compute-bound on the Jetson Orin Nano |

### Characterisation harnesses

| Probe | Purpose | Motivating logbook entry |
|-------|---------|--------------------------|
| `hand_jerk_limited_prototype.py` | Offline Python reference for the Phase 2 jerk-limited throw/catch profile family (symmetric 3-segment quintic-linear-quintic, C2 globally). Exposes `JerkLimitedProfile`, `feasibility()`, `solve_min_time()`, `make_throw()` / `make_catch()`, and validation helpers (`boundary_residuals()`, `sample_grid()`). Reads `max_event_hand_accel_rps2` from the generated config module — single source of truth. Phase 3's lockstep `Trajectory.h` + `sim/hand/trajectory.py` rewrite must reproduce this reference sample-for-sample. Outputs PNG + JSON to `temp/probes/`. | `logbook/2026-05-23-hand-generator-phase2-jerk-limited-design.md` — Phase 2 of the hand-trajectory-generator overhaul |
| `hand_profile_probe.py` | Characterises the *current* hand throw/catch generator (`sim/hand/trajectory.py`, the port of Teensy `Trajectory.h`). Drives the real `HandThrowTrajectory` / `HandCatchTrajectory` / `HandSmoothMove` classes across the `0.3–7.0 m/s` event-velocity sweep; quantifies the piecewise-constant-acceleration discontinuities (analytic accel-step magnitudes + finite-difference 500 Hz jerk), the `peak accel ∝ v²` and `duration ∝ 1/v` scaling laws, and contrasts against the jerk-bounded `HandSmoothMove` reference. Records the `catch_vel_ratio` port-vs-firmware divergence. Outputs CSV / JSON / PNG to `temp/probes/`. | `logbook/2026-05-22-hand-generator-phase1-characterisation.md` — Phase 1 of the hand-trajectory-generator overhaul |
