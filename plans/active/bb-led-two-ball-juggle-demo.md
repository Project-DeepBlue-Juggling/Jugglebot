---
title: Ball-Butler-initiated two-ball oval juggling demo
created: 2026-05-22
status: active
---

# Ball-Butler-initiated two-ball oval juggling demo

## 1. Context

### Problem / motivation

A demonstrable, repeatable two-ball juggling pattern is wanted for a project
video. Online MPC control of the Stewart platform is the intended long-term
controller, but it is not yet reliable enough for a sustained dynamic pattern,
and the open MPC questions (session-long solve-time creep, fallback behaviour)
are out of scope here. The demo therefore needs a control approach that is
**deterministic, repeatable, and smooth** without depending on a real-time
solver.

Both halves of the task have already been demonstrated on hardware
independently: a Ball-Butler-to-Jugglebot catch (2026-02-18) and an unassisted
two-ball column pattern (2024-08-13). Both ran on a trivial controller —
per-leg position targets issued at 100 Hz with no kinematic, inertial, or jerk
shaping — which produced functional but visibly janky motion. The hardware is
proven capable; what is new is *combining* the two and making the platform
motion *smooth*.

### What this change achieves

- A scripted, open-loop demo: Ball Butler primes the pattern with one throw,
  then Jugglebot sustains a two-ball one-hand oval pattern for 30+ catches.
- Smooth, jerk-minimised Stewart-platform motion via a trajectory optimised
  **once, offline** — no runtime solver, so platform compute load is
  negligible.
- A reusable open-loop playback path (`TrajectoryPlayer`) that re-uses the
  existing safety-critical 500 Hz `motor_guard` and the inverse-kinematics and
  feedforward stack, with the MPC solver removed from the loop.

### Scope and explicit non-goals

- **In scope:** offline trajectory optimisation, the runtime player, the
  master-timeline orchestration, sim validation, hardware bring-up.
- **Not in scope (parallel side-quest):** the hand-trajectory-generator
  overhaul (`Trajectory.h`: piecewise-constant-acceleration → jerk-limited,
  parameterised by release velocity / time budget / stroke portion). That work
  has its own plan document, `plans/active/hand-trajectory-generator-overhaul.md`
  (to be created), and proceeds independently. This demo is built against the
  *current* hand generator; the overhaul, when landed, improves hand smoothness
  and lowers the achievable throw height but is not a prerequisite.
- **Not in scope:** any change to the MPC control path. The MPC code is left
  untouched; the demo is additive.

### Prerequisites and timing

- Branch `demo/bb-led-two-ball-juggle` (off `refactor` at `155f9b1`).
- Phase 1 (feasibility study) is complete — see §4 Phase 1.
- The demo is throwaway-grade: it is not expected to survive into the
  long-term MPC architecture, and code quality is traded against time-to-result
  accordingly. It must still be safe.

### Related work

- Feasibility study: §4 Phase 1 of this document.
- Hand generator: `ros_ws/src/jugglebot/Teensy_code/Trajectory.h`,
  `sim/hand/trajectory.py` (Python port).
- Reference-feasibility contract (not enforced here — MPC is bypassed):
  `controller/REFERENCE_LAYER_CONTRACT.md`.

## 2. Architecture

### Current architecture (MPC path)

```
ROS2 stack ──targets──> mpc_bridge_node ──ZMQ:5558──> run_mpc.py
                                                         │  MPC solve @ 40 Hz
                                                         │  (controller/runner.py)
                                                         ▼
                                              HardwarePlant.command()
                                                         │  ZMQ:5557 (TOPIC_MPC_CMD)
                                                         ▼
                                          motor_guard @ 500 Hz  (cubic interp + safety)
                                                         │  CAN
                                                         ▼
                                                  6 leg ODrives

hand:  ROS2 service set_hand_traj_cmd     ─> can_node ─> CAN 0x6D0 ─> platform Teensy
BB:    ROS2 service bb/send_throw_command ─> can_node ─> CAN 0x7D0 ─> Ball Butler
clock: can_node bus.broadcast_time() ─> CAN 0x7DD @ 100 Hz ─> both Teensys + Ball Butler
```

### Proposed architecture (demo path)

```
OFFLINE (run once, any machine, no time pressure):
  pattern spec ──> juggle_optimizer.py (CasADi) ──> trajectory file (.npz)
  minimise Σ leg jerk²; periodic boundary conditions; throw/catch constraints

RUNTIME (juggle_demo orchestrator — negligible compute, no solver):
  master timeline  (absolute wall-times on the shared CAN clock 0x7DD)
        │
        ├── platform:  TrajectoryPlayer @ 40 Hz
        │       pose(t) = trajectory.eval(t_wall − t0)  → IK → leg extensions
        │       → HardwarePlant.set_pose() + .command()
        │              │ ZMQ:5557
        │              ▼
        │       motor_guard @ 500 Hz (cubic interp + safety) ─CAN─> 6 leg ODrives
        │
        ├── hand events ──> set_hand_traj_cmd service ──> can_node ─CAN 0x6D0─> Teensy
        │
        └── BB event(s) ──> bb/send_throw_command service ─> can_node ─CAN 0x7D0─> Ball Butler
```

### What changes vs what stays the same

| Element | Status in demo |
|---------|----------------|
| `motor_guard` (500 Hz interp + safety) | **Unchanged** — reused as-is |
| `can_node`, CAN protocol, hand/BB services | **Unchanged** — reused as-is |
| `HardwarePlant` / `MuJoCoPlant` | **Unchanged** — reused as-is |
| Inverse kinematics (`motion/motor_commands.py`, `pose_to_leg_lengths`) | **Unchanged** — reused as-is |
| MPC solver (`controller/mpc.py`, `runner.py`) | **Not invoked** — removed from the loop, code untouched |
| 40 Hz command source | **New** — `TrajectoryPlayer` replaces the MPC |
| Reference layer (K1–K6, `make_feasible_events`) | **Not invoked** — the offline optimiser owns feasibility |
| `hardware_config.yaml` leg velocity/accel limits | **Modified** — raised from conservative test values (Phase 4) |

### No new IPC ports

The demo introduces **no new ZMQ ports**. The platform command path reuses
`HardwarePlant`'s existing publish to `ZMQ:5557` (`TOPIC_MPC_CMD`). Hand and
Ball Butler commands reuse the existing ROS2 services on `can_node`. The shared
wall clock is the existing CAN `0x7DD` broadcast (`bus.broadcast_time()`,
`<II>` sec+usec, 100 Hz).

### New module layout

Pure-Python demo logic (no ROS2 imports) lives under `controller/demo/`:

| File | Role |
|------|------|
| `controller/demo/pattern.py` | Oval pattern geometry + tempo spec; derives throw/catch points, apex, period from feasibility math |
| `controller/demo/juggle_optimizer.py` | Offline CasADi optimiser → periodic platform trajectory file (run once) |
| `controller/demo/trajectory.py` | `JuggleTrajectory` — loads the trajectory file, `eval(t) → (pose, twist, accel)`, priming/steady/exit segments |
| `controller/demo/player.py` | `TrajectoryPlayer` — runtime: wall-time → leg-extension command + feedforward via IK |
| `controller/demo/timeline.py` | `MasterTimeline` — absolute-time hand/BB event schedule; abort handling |

The hardware orchestrator is a thin ROS2 wrapper in `ros_ws/` that imports the
above (matching the project's "ROS2 nodes are thin wrappers, business logic in
pure-Python modules" boundary).

## 3. Implementation Phase Summary

| Phase | Scope | Status | Date | Risk | Validates |
|-------|-------|--------|------|------|-----------|
| 1 | Tempo & geometry feasibility study | COMPLETE | 2026-05-22 | Low | Pattern closes at a sane throw height |
| 2 | Offline trajectory optimiser & player core | IN PROGRESS | 2026-05-22 | Med | Optimised platform trajectory is smooth, periodic, jerk-bounded; sim plant tracks it open-loop |
| 3 | Full sim juggle demo | NOT STARTED | | Med | 30+ catches in MuJoCo with sim hand model + BallButlerSim + master timeline |
| 4 | Hardware bring-up (open-loop) | NOT STARTED | | High | CAN/ROS2 integration; leg-limit raise; first hardware catches |
| 5 | Hardware robustness & polish | NOT STARTED | | High | Sustained 30+ catches; abort hardening; optional QTM correction |

Phases are incremental: each leaves a testable, deployable intermediate state.
No phase depends on a later one. The hand-generator overhaul is **not** a phase
of this plan — it is a parallel plan and may land at any time.

## 4. Implementation Phases (detailed)

### Phase 1: Tempo & geometry feasibility study — COMPLETE (2026-05-22)

**Outcome:** the two-ball one-hand pattern closes at high throws. Operating
point fixed for the first cut.

**Hand timing model** (derived from `Trajectory.h` + `hardware_config.yaml`,
effective stroke 0.315 m, `inertia_ratio` 0.747, `catch_vel_ratio` 0.6), with
throw speed `v` in m/s:

- Throw stroke duration: `t_throw ≈ 0.614 / v`
- Catch stroke duration: `t_catch ≈ 0.998 / v`
- Flight time (symmetric): `t_flight = 2v/g`
- Apex above hand: `h = v²/2g`

**Tempo constraint** (one THROW point, one CATCH point, hand alternates
throw→catch→throw→catch); conservative platform transit window:

`transit_window = (t_flight − t_throw − 1.5·t_catch) / 2`

| Apex `h` | `v` (m/s) | `t_flight` | transit window |
|----------|-----------|-----------|----------------|
| 1.5 m | 5.42 | 1.11 s | 0.36 s |
| **1.3 m** | **5.05** | **1.03 s** | **0.31 s** |
| 1.0 m | 4.43 | 0.90 s | 0.21 s |
| 0.83 m | 4.04 | 0.82 s | 0.15 s |
| 0.53 m | 3.22 | 0.66 s | 0 s |

Floors (current hand generator): ~0.53 m hard, ~0.85 m practical. With a
peak-acceleration-bounded hand generator (the parallel side-quest), both
`t_flight` and `t_event` scale linearly with `v`, making tempo feasibility
nearly scale-invariant — the overhaul is what unlocks lower throws later.

**Operating point fixed:**

| Parameter | Value |
|-----------|-------|
| Throw apex above hand | 1.3 m (ceiling clearance ~1.5 m) |
| Throw speed `v` | ~5.05 m/s (hand limit 7.0 m/s) |
| Pattern period `P` | ~1.25 s |
| THROW↔CATCH separation `d` | ≤ 300 mm available; minimise toward ball-collision clearance (~100 mm) |
| Run length | scripted stop at 30–40 catches (~19–25 s of juggling) |

**Binding constraints identified:** (a) hand catch stroke duration
(`catch_vel_ratio` is the lever — out of scope here, owned by the side-quest);
(b) platform leg jerk — leg *velocity* is **not** binding (configured
`leg_vel_limit_rps: 4.0` is a conservative test value; legs bench-tested to
3.4 m/s), so the optimiser objective is jerk minimisation.

### Phase 2: Offline trajectory optimiser & player core — IN PROGRESS (started 2026-05-22)

**Progress (2026-05-22).** The player core has landed: `controller/demo/pattern.py`
(oval geometry + tempo, reproducing the Phase 1 numbers),
`controller/demo/trajectory.py` (`JuggleTrajectory` — periodic C2 evaluator —
plus `build_analytic_oval`, the un-optimised baseline), and
`controller/demo/player.py` (`TrajectoryPlayer`). Validated by
`tests/sim/test_demo_trajectory.py` (13 unit tests) and
`tests/sim/test_demo_sim_playback.py` (2 tests — open-loop MuJoCo playback +
commanded leg-jerk metric). The CasADi jerk-minimising optimiser
(`juggle_optimizer.py`) is the remaining Phase 2 deliverable; until it lands,
`build_analytic_oval` is the trajectory source.

**New/modified files:**
- `controller/demo/pattern.py` (new)
- `controller/demo/juggle_optimizer.py` (new)
- `controller/demo/trajectory.py` (new)
- `controller/demo/player.py` (new)
- `temp/demo/juggle_trajectory.npz` (generated artefact, gitignored)

**Scope:**

`pattern.py` encodes the oval geometry as a parametric spec: THROW point,
CATCH point, apex 1.3 m, period `P`, the per-ball phase offset `P/2`, and the
event instants (throw-release, catch-contact) within the period.

`juggle_optimizer.py` solves, once, offline, a CasADi NLP for the **periodic
steady-state** platform trajectory plus a **priming transient** and an **exit
transient**:

- Decision variables: platform pose `[x,y,z,rx,ry,rz](t)` over one period,
  represented as quintic-Hermite knots or direct-collocation nodes.
- Objective: minimise `Σ_legs ∫ jerk² dt` — jerk evaluated in **leg space**
  (the nonlinear IK means pose-space-smooth ≠ leg-space-smooth).
- Periodic boundary conditions: pose, twist, accel at `t=0` equal those at
  `t=P`.
- Hard constraints at the throw instant: hand axis vertical (platform level)
  for a clean release; platform horizontal velocity equals the ball's required
  horizontal launch velocity (sets where the ball lands).
- Hard constraints at the catch instant: platform level; platform horizontal
  velocity ≈ ball arrival horizontal velocity (soft, matched catch).
- Orientation banking is **free** during the carry phase (constrained back to
  level at throw/catch) — the optimiser leans the platform if banking lowers
  leg jerk.
- Inequality constraints: leg position within stroke, leg velocity and
  acceleration within (raised) limits, platform tilt within `TILT_LIMIT_RAD`
  (30°, `controller/ballistics.py:22`).
- Output: a resampled spline written to `temp/demo/juggle_trajectory.npz`
  (priming segment, periodic core, exit segment).

`trajectory.py` (`JuggleTrajectory`): loads the `.npz`, exposes
`eval(t) → (pose_6dof, twist_6dof, accel_6dof)`. For `t` in the steady-state
window it evaluates `core(((t − t_prime) mod P))`; before/after it evaluates
the priming/exit segments.

`player.py` (`TrajectoryPlayer`): the runtime object. Per call with wall-time
`t`: evaluate `pose, twist, accel` at `t`, `t+dt`, `t+2·dt`; run IK to leg
extensions; produce the command tuple `(ext_mm, vel_mm_s, cmd_next_mm,
cmd_next2_mm)`. Because the trajectory is fully known, `cmd_next`/`cmd_next2`
are *exact* future leg extensions — `motor_guard` then performs exact cubic
Hermite interpolation rather than extrapolation.

**Validation in this phase:** drive `MuJoCoPlant` open-loop with the player
(platform only, no balls, no hand). Confirm the platform traces the optimised
oval and measure realised leg jerk against the optimiser's predicted jerk.

**Critical details:**
- Pose convention `[x,y,z,rx,ry,rz]`, mm and rad; leg extensions mm,
  STOW-relative (`PlantState.leg_extensions_mm`).
- The optimiser and the runtime player must use the **same IK** as the plant
  (`motion/motor_commands.py`); a divergent IK produces a trajectory the plant
  cannot follow.
- The runtime player must be NumPy-only (no CasADi import) so it can be
  imported by a Python 3.8 ROS2 node in Phase 4.

**Dependencies:** Phase 1 operating point. CasADi (already a `controller/`
dependency). No hardware.

### Phase 3: Full sim juggle demo — NOT STARTED

**New/modified files:**
- `controller/demo/timeline.py` (new)
- `sim/main.py` (modified — add a `--juggle-demo` mode) **or**
  `sim/juggle_demo.py` (new) — Decision required, see §6.
- `controller/demo/player.py` (extended — abort path)

**Scope:**

`timeline.py` (`MasterTimeline`): builds the absolute-time event schedule from
the pattern spec — the BB priming throw(s), and every hand throw/catch event —
each tagged with an absolute wall-time and its parameters (`traj_type`,
`event_vel`). Exposes `due_events(t_wall) → list[Event]` and an `abort(t_wall)`
that truncates the schedule and returns the exit transient.

Sim wiring: the demo loop runs `MuJoCoPlant` + `TrajectoryPlayer` (platform) +
the existing `sim/hand/` model (`HandCatchTrajectory`/`HandThrowTrajectory`/
`HandThrowSequence`) + `BallButlerSim` (`sim/ball_butler/sim.py`,
`throw_at_jugglebot()`), all in one process at 40 Hz. Ball spawn/capture use
the existing `BallManager` (`sim/ball/manager.py`: `spawn`, `check_capture`,
`release`).

**Sequence implemented:** Jugglebot starts holding ball 2 →
`BallButlerSim` throw of ball 1 → Jugglebot throws ball 2 → catches ball 1 →
throws ball 1 → catches ball 2 → … sustained for 30+ catches → exit transient.

**Critical details:**
- The sim hand model is the faithfulness reference for the hardware hand. The
  sim `catch_vel_ratio` **must** match `hardware_config.yaml` — see §6 open
  item (sim port currently reads 0.9; config reads 0.6).
- Catch tolerance: the passive conical hand collects off-centre balls; sim
  capture detection (`BallManager.check_capture`) is the success oracle.
- The hand offset above the platform centroid is position-dependent
  (`compute_hand_offset_mm`); the timeline's catch-pose height must account
  for it.

**Dependencies:** Phase 2 (`JuggleTrajectory`, `TrajectoryPlayer`). No hardware.

### Phase 4: Hardware bring-up (open-loop) — NOT STARTED

**New/modified files:**
- `ros_ws/src/jugglebot/jugglebot/juggle_demo_node.py` (new — thin ROS2
  wrapper) **or** `run_juggle_demo.py` (new — standalone) — Decision required,
  see §6.
- `config/hardware_config.yaml` (modified — raise leg velocity/accel limits)
- regenerated `config/generated/*` artefacts

**Scope:**

The hardware orchestrator runs the Phase 2/3 pure-Python demo logic against
real hardware: `HardwarePlant` for the platform (publishes to `motor_guard` on
`ZMQ:5557`), and the `can_node` ROS2 services `set_hand_traj_cmd` and
`bb/send_throw_command` for the discrete hand and BB events.

The leg velocity/acceleration limits in `hardware_config.yaml`
(`leg_vel_limit_rps`, and the trapezoidal/accel limits the platform path uses)
are raised from the conservative test values toward bench-validated headroom
(reference: 3.4 m/s bench result). Procedure: edit YAML → run
`python config/generate_config.py` → stage regenerated artefacts → test.
The raise is incremental and reviewed, not a single jump to the bench maximum.

Bring-up order, each a stop point:
1. Platform-only: play the optimised trajectory with no balls and no hand
   commands; confirm smooth motion and that `motor_guard` raises no safety
   faults.
2. Platform + hand, no balls: add the scheduled hand throw/catch strokes;
   confirm hand/platform timing.
3. Full demo, low catch count: add the BB priming throw and real balls; target
   the first ~5 catches.

**IPC / message formats used (all existing):**
- Platform command (`HardwarePlant.command` → `ZMQ:5557`):
  `{'type': 'mpc_command', 'ext_mm': [6], 'pose_6dof': [6], 'motor_rev': [6],
  'vel_mm_s': [6], 'torque_Nm': [6], 'acc_mm_s2': [6], 'cmd_next_mm': [6],
  'cmd_next2_mm': [6]}`
- Hand command (`set_hand_traj_cmd` service → CAN `0x6D0`): fields
  `event_delay` (s), `event_vel` (m/s), `traj_type` (`0`=catch, `1`=throw,
  `2`=return, `3`=smooth-move). Encoded as byte 0 `traj_type`, bytes 1–2
  `round(event_vel·100)` uint16 LE, bytes 3–6 `(time.time()+event_delay)·1000
  & 0xFFFF`, byte 7 `0`.
- BB command (`bb/send_throw_command` service → CAN `0x7D0`): fields
  `yaw_angle_rad`, `pitch_angle_rad`, `throw_speed` (m/s), `throw_time`
  (delay s). Encoded `struct.pack('<hHHH', yaw·32768/π, pitch·65536/π,
  speed·10000, (time.time()+delay)·1000 & 0xFFFF)`.
- Shared clock: CAN `0x7DD`, `struct.pack('<II', sec, usec)`, 100 Hz.

**Critical details:**
- The hand Teensy holds one active trajectory; events are sent one-ahead, not
  batch-queued. The orchestrator must send each hand event with enough lead
  (the Teensy executes at the embedded absolute wall-time).
- Ball Butler requires ~100–300 ms throw lead time or it rejects the command
  (`StateMachine.cpp` lead-time check); the BB priming event must be issued
  early.
- All scheduling uses absolute epoch wall-time, consistent with the `0x7DD`
  broadcast — no relative delays accumulate.
- Abort: a ROS2 topic/service (or keypress) triggers `MasterTimeline.abort()`
  → player runs the exit transient to a stow pose → `plant.disable()`. A
  `motor_guard` E-stop remains the independent hardware safety layer.

**Dependencies:** Phases 2–3. Real robot, Ball Butler, E-stop ready.

### Phase 5: Hardware robustness & polish — NOT STARTED

**New/modified files:**
- `controller/demo/pattern.py`, `juggle_optimizer.py` (tuning re-runs)
- `controller/demo/timeline.py` (timing offsets)
- optional: `controller/demo/qtm_correction.py` (new)

**Scope:**
- Tune the operating point and event timing offsets to reach a sustained
  30–40 catches open-loop.
- Harden the abort path; verify a clean stow from any point in the cycle.
- If open-loop drift exceeds the passive-cone catch tolerance over 30 catches,
  add an **optional** QTM-mocap correction: detect each ball's apex and nudge
  the *next* catch-event time. This is a fallback, not a baseline requirement.
- Lower the throw height as far as the current hand generator allows (toward
  the ~0.85 m practical floor), if desired for the video.

**Dependencies:** Phase 4. Real robot. QTM only if the open-loop result is
insufficient.

## 5. Testing Plan

### Unit tests (offline, no hardware) — `tests/sim/`

| ID | Validates | Pass criteria |
|----|-----------|---------------|
| T-U1 | `JuggleTrajectory.eval` periodicity | `eval(t_prime)` and `eval(t_prime+P)` agree in pose, twist, accel to < 1e-9 |
| T-U2 | C2 continuity at every internal knot and at the priming/steady and steady/exit joins | position, velocity, acceleration continuous to < 1e-6 |
| T-U3 | Throw-instant constraint | at each throw event: platform tilt = 0 (hand axis vertical) to < 1e-4 rad; horizontal velocity matches the pattern spec |
| T-U4 | Catch-instant constraint | at each catch event: platform level; horizontal velocity matches ball arrival velocity within tolerance |
| T-U5 | IK round-trip | `ik(pose)` then forward-kinematics returns `pose` within 1e-6 mm / rad across the trajectory |
| T-U6 | Optimiser output respects limits | leg position, velocity, acceleration, and platform tilt all within configured bounds at every sample |
| T-U7 | `TrajectoryPlayer` exact lookahead | `cmd_next_mm`/`cmd_next2_mm` equal `ik(eval(t+dt))`/`ik(eval(t+2dt))` exactly |
| T-U8 | `MasterTimeline` event ordering | events strictly time-ordered; per-ball phase offset = `P/2`; throw and catch alternate |
| T-U9 | `MasterTimeline.abort` | after abort, `due_events` returns no future throws; exit transient ends at the stow pose with zero twist and accel |
| T-U10 | Player NumPy-only | importing `controller/demo/player.py` does not import `casadi` |
| T-U11 | Hand command encoding | `event_vel`/`traj_type`/wall-time encode to the exact `0x6D0` byte layout (regression against `_send_hand_traj_cmd`) |
| T-U12 | BB command encoding | yaw/pitch/speed/time encode to the exact `0x7D0` `<hHHH>` layout |

### Integration tests (sim, safe) — `tests/sim/`

| ID | Validates | Pass criteria |
|----|-----------|---------------|
| T-I1 | Player drives `MuJoCoPlant` open-loop (Phase 2) | platform pose tracks `eval(t)` within sim tolerance for a full period |
| T-I2 | Realised leg jerk vs optimiser prediction | measured per-leg jerk within 10% of the optimiser's predicted value |
| T-I3 | Full sim juggle (Phase 3) | ≥ 30 consecutive catches detected by `BallManager.check_capture` |
| T-I4 | Master-clock scheduling accuracy in sim | each hand/BB event fires within one 40 Hz tick (25 ms) of its scheduled wall-time |
| T-I5 | Priming transient | from the start pose, ball 1 (BB) and ball 2 (Jugglebot) mesh into the steady-state phase offset without a drop |
| T-I6 | Abort mid-pattern | `abort()` at an arbitrary cycle phase yields a smooth exit to stow; no leg velocity/accel limit exceeded |
| T-I7 | Catch-tolerance margin | inject ±30 mm ball-position scatter (BB repeatability); catch success ≥ 90% |

### Hardware tests (real actuators, E-stop ready) — `tests/hardware/`

| ID | Validates | Pass criteria |
|----|-----------|---------------|
| T-H1 | Leg-limit raise | after the `hardware_config.yaml` raise, a platform-only trajectory run produces no `motor_guard` overspeed/deviation fault |
| T-H2 | Platform-only trajectory (Phase 4 step 1) | platform traces the optimised oval; subjectively smooth; no fault |
| T-H3 | Platform + hand, no balls (Phase 4 step 2) | hand throw/catch strokes fire at the correct platform poses and times |
| T-H4 | Full demo, low count (Phase 4 step 3) | ≥ 5 consecutive catches |
| T-H5 | Sustained run (Phase 5) | ≥ 30 consecutive catches |
| T-H6 | Abort on hardware | abort command produces a clean stow and `plant.disable()` from any cycle phase |
| T-H7 | Endurance | a 60 s run completes without drift-induced drop or thermal/timing fault |

### Regression tests

| ID | Validates | Pass criteria |
|----|-----------|---------------|
| T-R1 | MPC path unaffected | `pytest tests/sim/test_mpc_static.py` passes unchanged (the demo adds files, does not modify the MPC path) |
| T-R2 | `motor_guard` unchanged | `motor_guard` source is byte-identical to `refactor`; existing `motor_guard` tests pass |
| T-R3 | Config regeneration | after the leg-limit raise, `python config/generate_config.py` produces no diff beyond the intended limit fields |
| T-R4 | Full suite | `pytest tests/ -q` passes; count cited with the (date, command, result) triple |

## 6. Notes for Collaborators

### Open items / decisions required

- **`catch_vel_ratio` discrepancy — CLOSED (port reconciled 2026-05-23).**
  `config/hardware_config.yaml:377` sets `catch_vel_ratio: 0.6` (firmware
  authoritative). The sim port `sim/hand/trajectory.py` was reconciled from
  the divergent `0.9` to `0.6` on 2026-05-23 — see
  `logbook/2026-05-23-catch-vel-ratio-port-reconciled.md`. The Phase 1
  feasibility math (which already used 0.6) and `CATCH_DUR_COEF = 0.998`
  in `controller/demo/pattern.py` need no revisiting.
- **Hardware orchestrator form.** Two viable patterns for Phase 4:
  (A) a ROS2 node `juggle_demo_node` in `ros_ws/` — natural access to the
  `can_node` hand/BB services, runs the 40 Hz platform timer, imports the pure-
  Python player; risk: `controller/` runtime code must be Python 3.8-safe.
  (B) a standalone process `run_juggle_demo.py` (Python 3.11, like `run_mpc.py`)
  driving the platform via ZMQ, plus a minimal mechanism for the discrete
  hand/BB events. Pattern A is recommended (the discrete events need the ROS2
  services and the runtime player is NumPy-only by Phase 2 design). Decision
  required before §4 Phase 4.
- **Sim entry point.** Add a `--juggle-demo` mode to `sim/main.py` vs a
  standalone `sim/juggle_demo.py`. Decision required before §4 Phase 3.

### Safety-critical invariants

- **Zero platform velocity is not required at the throw, but zero platform
  tilt is.** The columns/oval throw is vertical only if the hand axis is
  vertical at release; the platform's *horizontal velocity* at release is
  deliberately non-zero (it sets the ball's arc) and is an optimiser
  constraint, not an error. Getting the tilt wrong sends balls off-axis and
  the open-loop pattern walks off within a few catches.
- **`motor_guard` is the safety layer and is not modified.** Its overspeed,
  deviation (`MAX_DEVIATION_REV = 0.5`), staleness (`MPC_CMD_STALENESS_S =
  0.25`), and workspace checks remain the last line of defence. The player
  must publish at ≥ 40 Hz or `motor_guard` will fault on command staleness.
- **Leg sign convention.** `can_node._leg_sign` inverts leg axes (ODrive
  negative = extension, Jugglebot positive = extension). The demo does not
  touch this path; the player emits STOW-relative extensions in mm exactly as
  the MPC did.
- **Absolute wall-time only.** Every hand and BB event carries an absolute
  epoch timestamp consistent with the `0x7DD` broadcast. Relative delays must
  not be chained, or timing error accumulates across 30 catches.
- **Ball Butler lead time.** A BB throw command issued with less than
  ~100–300 ms lead is *rejected* (not delayed). The priming event must be
  scheduled early.
- **Raising leg limits is a reviewed change.** `leg_vel_limit_rps` and the
  accel limits are raised incrementally with `config/generate_config.py`
  re-run and staged artefacts; never a single jump to the bench maximum, and
  never a hand-edit of generated files.

### Architecture decisions

- **MPC bypassed, not deleted.** The demo is additive; the MPC path is left
  untouched so the long-term controller work is unaffected and the demo
  branch stays trivially revertible.
- **Optimise offline, play online.** All optimisation is a single offline
  CasADi solve; the runtime player is a pure spline evaluation. This removes
  solver compute from the loop entirely — the open question of MPC solve-time
  is irrelevant to the demo.
- **Jerk is minimised in leg space, not pose space.** The nonlinear IK means
  a pose-smooth trajectory is not leg-smooth; the actuators are what must move
  gently.
- **Open-loop by design.** With a hardware-synced clock and repeatable BB
  (~2–3 cm), feedback is unnecessary for the first cut. QTM correction is a
  Phase 5 fallback only.
- **`motor_guard` reused for the 500 Hz path.** The player supplies exact
  `cmd_next`/`cmd_next2`, so `motor_guard` interpolates (cubic Hermite) rather
  than extrapolates — the smoothest available 40→500 Hz path.

### Startup / shutdown ordering

Startup: `can_node` (CAN + `0x7DD` clock + hand/BB services) → `motor_guard`
→ orchestrator. The orchestrator enables `HardwarePlant`, waits for telemetry,
then starts the master timeline (which issues the BB priming throw first).

Shutdown: abort or completion → player exit transient to stow →
`plant.disable()` → `plant.close()`. On a `motor_guard` E-stop, `disable()` is
skipped to preserve the fault state, matching `run_mpc.py` behaviour.

### Files affected

| File | Action |
|------|--------|
| `controller/demo/pattern.py` | Create |
| `controller/demo/juggle_optimizer.py` | Create |
| `controller/demo/trajectory.py` | Create |
| `controller/demo/player.py` | Create |
| `controller/demo/timeline.py` | Create |
| `controller/demo/qtm_correction.py` | Create (optional, Phase 5) |
| `ros_ws/src/jugglebot/jugglebot/juggle_demo_node.py` | Create (Phase 4, pattern A) |
| `sim/main.py` or `sim/juggle_demo.py` | Modify / create (Phase 3) |
| `config/hardware_config.yaml` | Modify (leg limits, Phase 4) |
| `config/generated/*` | Regenerate (Phase 4) |
| `tests/sim/test_juggle_*.py` | Create |
| `tests/hardware/juggle_demo_test.py` | Create |
| MPC path (`controller/mpc.py`, `runner.py`, `motor_guard.py`) | Untouched |

### Rollback plan

The demo is additive and on a dedicated branch. Rollback of any phase:

- Phases 2–3: new files only — delete `controller/demo/` and the sim entry
  point; nothing else is affected.
- Phase 4: revert the `hardware_config.yaml` leg-limit change and re-run
  `config/generate_config.py`; remove `juggle_demo_node.py`. The MPC path and
  `motor_guard` were never modified, so the robot returns to its pre-demo
  state with a single config revert.
- The entire effort can be abandoned by deleting branch
  `demo/bb-led-two-ball-juggle` with no impact on `refactor`.
