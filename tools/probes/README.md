# tools/probes/ — Reusable Empirical-Probe Harnesses

Probes fall into two families:

1. **Offline drivers** — scripts that drive the MPC / controller stack in
   a production-faithful way from recorded telemetry, to reproduce
   hardware failures, characterise edge cases, or empirically confirm
   threshold values before they get encoded in tests.
2. **Live observers** — scripts that record system state alongside a live
   session (real robot, real ROS2 stack, real MPC) so a later analysis
   pass can attribute failures to hardware bottlenecks (CPU saturation,
   thermal / DVFS throttling, memory bandwidth, etc).

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
| `juggle_cup_bandlimit.py` | Lock-in Bode characterisation of the juggle demo's cup-tracking: the Stewart platform (lateral cup motion) and the hand slider (vertical cup stroke), with static-gain / workspace checks at the operating height. Grounds the online cup planner's per-axis jerk limits — slider is fast + accurate (do the throw stroke there), platform tracks only smooth lateral motion (-3 dB ~5 Hz) within a ~150 mm workspace. Headless MuJoCo plant, no hardware. | `logbook/2026-06-27-throw-aim-band-limit-and-closed-loop-catch.md` (and the online-replanning successor) |
| `juggle_online_debug.py` | Debug harness for the online re-planning juggle runner (`sim/juggle_online.py`): per-tick cup/ball trajectory log + a separation analysis (cup z-vel/accel vs ball z-vel through the first throw, flagging `a_cup < -g`). `--stiff-always` forces stiff contact for the A/B test that showed the throw blocker is **contact cohesion** (the cup drags the ball), not a bounded slider — stiff makes the ball fly free (catches 1), exposing the second issue (the **slam** to ~16.8 m/s from the carry pre-roll leaving the cup low). Headless MuJoCo plant, no hardware. | `logbook/2026-06-27-online-replanning-architecture-and-cup-bandlimit.md` (throw-separation re-diagnosis) |
| `juggle_tilt_bandlimit.py` | Platform-tilt (orientation rx/ry) tracking characterisation for the throw-aim re-architecture: static-hold accuracy (no droop), the lever arm (~1.66 mm/deg lateral cup shift + second-order vertical cross-coupling), leg headroom at high tilt (>45 mm margin even at 24 deg — tilt is not stroke-limited at z=170), and a low-frequency tilt Bode around the 1.6 Hz cycle freq (~0.99/3deg, better than lateral translation). Grounds the tilted-detach planner — tilt aims the throw, the slider supplies the speed. Headless MuJoCo plant, no hardware. | `logbook/2026-06-29-platform-tilt-tracking-characterisation.md` (Phase 0 of `plans/active/bb-online-juggle-tilt-rearchitecture.md`) |
