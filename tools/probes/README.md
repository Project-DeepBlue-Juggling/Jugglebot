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
| `teensy_can3_telem.py` | Live observer for the can-bridge: brings up the UDP link RX-only (10 Hz J→T heartbeats with `flags=0` + a `TIME_OF_DAY` responder — it cannot command anything) and prints all 7 Jugglebot axes pos/vel + per-axis diagnostics (state, `active_errors`, bus voltage, FET temp) as decoded by the can-bridge off CAN3. Confirms link health, time-sync anchoring, and CAN3 ODrive decoding during bring-up. (The motor-commanding counterpart — encoder index search — lives in `tests/hardware/teensy_encoder_search_bench.py`, per the never-command-motors rule below.) | `plans/active/teensy-can-offload.md` (Phase 5/6 CAN3 bring-up) |

### Can-bridge Teensy (`teensy_link_profiling/`)

Offline validation + bench/profiling tools for the new can-bridge Teensy firmware.
See `teensy_link_profiling/README.md`.

| Tool | Purpose | Motivating plan |
|------|---------|-----------------|
| `teensy_link_profiling/hermite_xref/xref.py` | Offline cross-check: the firmware's 500 Hz interpolator (`leg_interp.cpp`, via the line-for-line Python port `teensy_interp.py`) vs. the real `motor_guard.py`. Proves 0.0 rev divergence over synthetic + recorded `mpc_*.csv`. | `plans/active/teensy-can-offload.md` Phase 7 |
| `teensy_link_profiling/jetson/profile_monitor.py` | Live observer: ingests the 1 Hz PROFILE UDP frame (per-task CPU, CAN1/CAN2 util, UDP RTT/jitter, interp deadline-misses, heap) → CSV + matplotlib plots in `temp/probes/teensy_link_profiling/`. | `plans/active/teensy-can-offload.md` (profiling) |
| `teensy_link_profiling/jetson/setpoint_stub.py` | Bench exerciser / Phase-4 stub client: sends a synthetic 40 Hz setpoint stream + heartbeats, answers the time-of-day RPC, prints uplink — exercises the firmware data path without the real MPC. | `plans/active/teensy-can-offload.md` Phase 4 |
