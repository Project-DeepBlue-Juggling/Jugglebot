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
| `juggle_catch_offset.py` | Rung-1 single-catch characterisation sweep: drives `sim/juggle_catch.py::run_single_catch` (BB-reload catch = translate-to-reach + tilt-to-receive + lever-arm realisation, under the §3 noise) across a grid of nominal landing placements × seeds, reporting the clean-catch rate and the in-cup seat-offset distribution. Grounds the catch realisation constants and the test thresholds (`tests/sim/test_juggle_catch.py`). Headless MuJoCo plant, no hardware. Writes `temp/probes/juggle_catch_offset.csv`. | `logbook/2026-06-30-rung1-clean-single-catch.md` (Phase 1 / Rung 1 of `plans/active/bb-online-juggle-tilt-rearchitecture.md`) |
| `juggle_throw_accuracy.py` | Rung-2a single-throw open-loop accuracy sweep: drives `sim/juggle_throw.py::run_single_throw` (tilt-aimed throw = carry up the tilted cup axis → detach along it → fly → measure landing, under §3 tracking noise) across a grid of target placements × cadences × seeds, reporting the clean-separation rate, the landing error, and the noisy "observed landing" the catch reaches for. The gate question (Rung-1 framing): is the throw scatter inside the catch's ~60-80 mm reliable reach? Found the column + 50 mm ring all land within reach (≤33 mm), the ±100 mm ring exposes a directional separation/aim asymmetry. Grounds `tests/sim/test_juggle_throw.py`. Headless MuJoCo plant, no hardware. Writes `temp/probes/juggle_throw_accuracy.csv`. | `logbook/2026-06-30-rung2a-single-ball-tilt-throw.md` (Phase 2 / Rung 2a of `plans/active/bb-online-juggle-tilt-rearchitecture.md`) |
| `juggle_selfcatch_loopgain.py` | Rung-2b single-ball self-catch loop-gain sweep (the MAKE-OR-BREAK gate evidence): drives `sim/juggle_selfcatch.py::run_self_catch` (compose the Rung-2a tilt-aimed throw with the Rung-1 catch into a single-ball toss→catch→toss loop, re-planned each cycle under §3 tracking noise). **(1) COLUMN** (`oscillate=False`) — three recover/geometry variants (faithful plan_cup_cycle + stationary column, drift in-place, ad-hoc axial retract), reports the per-cycle **reach trend**. Finding: does NOT sustain — diverges within 0–3 cycles (reach amplifies ~8 → ~210 mm past the ~60–80 mm reliable reach). Tilt ~0 (degenerate). **(2) OSCILLATION** (`oscillate=True`, the operator's OPTION-1 re-plan) — shuttle a single ball A↔B so every throw is lateral and tilt ENGAGES (~1.4 deg); axes x/y/diagonal × separations 20–70 mm (x swept at 20/30/40/50/60/70 mm), reports the per-cycle **in-cup-offset trend + landing-error trend**. Finding: STILL a BREAK — diverges within 1–4 cycles (max sustained 4 of ≥ 10, at x-20) WITH tilt engaged (in-cup seat offset stays small, but the LANDING amplifies, e.g. x-40 seed 1: 3.7 → 89 → 728 mm; loop gain > 1). Root cause: the tilt-aimed throw is chaotically sensitive to the throw-ORIGIN pose (dLanding/dOrigin ~4), a contact-detach knife-edge tilt does not address. Grounds `tests/sim/test_juggle_selfcatch.py`. Headless MuJoCo plant, no hardware. Writes `temp/probes/juggle_selfcatch_loopgain.csv`. | `logbook/2026-07-01-rung2b-selfcatch-column-divergence.md` + `logbook/2026-07-01-rung2b-oscillation-tilt-engaged-diverges.md` (Phase 3 / Rung 2b of `plans/active/bb-online-juggle-tilt-rearchitecture.md`) |
