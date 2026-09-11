---
title: Two-ball skill stack — schedule-driven throw/catch skills, one hand master, memory-based learning
created: 2026-09-09
status: active
owner: Harrison
last_updated: 2026-09-09
related_logbook:
  - 2026-09-09-two-ball-skill-stack-kickoff.md
related_config:
  - config/hardware_config.yaml → jugglebot_operational.unified_cycle_enabled (retires at R4)
  - config/hardware_config.yaml → jugglebot_operational.toss_ilc_enabled (retires at R3)
  - config/hardware_config.yaml → trajectory_op.leg_jerk_limit_mmps3 (the ramp lever, sized at R2)
  - config/generated/admissible_box.yaml (NEW at R2, machine-written)
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py::plan_window
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cycle_plan.py::CyclePlan
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py::validate_cycle
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/emitter.py::KnotEmitter
  - teensy_link/setpoint_pump.py::SetpointPump
  - ros_ws/src/jugglebot/Teensy_code_canbridge/leg_interp.cpp
  - ros_ws/src/jugglebot/jugglebot/tracking/matcher.py::BallTracker
---

# Two-ball skill stack

Reference: Lee, Wang, Atkeson, Rizzi, Rojas, *Rapid On-Robot Learning for
Dynamic Manipulation Skills: Robot Juggling*, arXiv:2608.26800v2 (the PDF is in
`Related Research/`). Branch `skill-stack`, worktree `~/Desktop/Jugglebot-skills`.

## 0. Values — normative for every unit of work on this plan

Every agent briefed on any rung below reads this section first and checks its
own diff against it before reporting. A finding against a value is a defect.

**Simplicity.**
- One path per function. No fallback modes, no dual masters, no legacy branch
  kept alive beside its replacement. A feature is not done until the thing it
  replaces is deleted, in the same rung.
- Closed forms and small pure functions over frameworks. A new module past
  ~500 lines, or a new configuration knob, carries a one-line justification in
  the rung's logbook entry naming the *measured* need it answers.
- Every refusal names one physical fact. A code that cannot be explained to
  the operator in one sentence is two codes or none.

**Elegance.**
- Commands are parameterised as outcomes. The learner's command is the landing
  the operator can see; the prior is therefore the identity and nothing needs
  differentiating.
- One data structure per concept — `Site`, `Skill`, `Segment`, `Experience`,
  `Schedule` — and no second copy of any constant or timing (a "timing twin" is
  how `JB_OP_HAND_CATCH_PRIME_REV` drifted 3.2 mm for its whole life).
- Each invariant is stated once, in its contract document, and enforced at one
  point.

**Coherence.**
- One clock (the shared CAN wall clock), one frame convention per boundary
  (§ 2.3), one planner, one memory, one schedule.
- The same orchestrator code drives the MuJoCo plant and the robot. Sim is a
  plant behind `PlantInterface`, never a second implementation.
- ROS nodes are shells: subscribe, call a pure function, publish. Logic lives
  in `motion/` and `teensy_link/`, which import no ROS.

**Robustness.**
- Failures are allowed; damage is not. A missed catch ends the attempt through
  a planned REST segment. The firmware deviation guard is the authority nothing
  on the Jetson overrides.
- Safety by construction: limits inside the planner and an offline-swept
  admissible command box, not a post-hoc gate on the beat path. The exhaustive
  gate stays as an offline certifier and a cheap runtime assert.
- **Every segment ends at rest at its site.** The next skill splices into the
  tail; the tail is the abort path, planned in advance, and it is what the
  machine does if the next skill never arrives.
- The schedule never delays. Perception loss ends the attempt. Determinism: no
  solve, no blocking I/O, no adaptation on the 40 Hz emitter thread; learning
  runs once per throw, at skill onset, on the orchestrator thread.

**Rigor.**
- Every rung has a gate with a (date, command, result) triple before the next
  rung starts, and a dress rehearsal on the loaded Jetson (launch up, bag
  recording, GUI up) before any powered sitting.
- A contract lands as three parts: the normative sentence, one enforcement
  point, one failing test.
- Grep before deleting, count to zero after. Probe before a threshold test.
  One logbook entry per change; `/audit --unstaged` once per rung, at its end.
- Every number carries provenance: bag, command, date.

## 1. Context

### 1.1 What the paper contributes, and what transfers

| Idea | Paper | Transfer |
|---|---|---|
| Regularised memory-based learner | kNN in (state, target) space, weighted local linear fit regularised to a prior, one damped step; prior = identity because the command is the desired landing | **Direct.** Replaces the critical-point ILC, the trim/cal/record stack and their artifacts |
| Orchestrator on a global wall clock | Skills dispatched at absolute times; overruns never delay the schedule; planning once at skill onset | **Direct.** Replaces the FSM choreography and the planner-owned ring beat |
| Mutually reachable set | Offline per-joint polytope constraining transition states; no runtime gate | **In shape.** An offline-swept admissible box on the *command*, plus limits inside the QP; the per-knot gate leaves the beat path |
| Catch-at-rest skills joined by a jerk-limited generator | Each skill a Ruckig segment between (q, q̇, q̈) states | **Not for the platform.** The platform is band-limited and jerk-bound, so it moves continuously through catch and throw; the cup QP's whole-segment shape stays |

### 1.2 What this machine dictates

Two balls in one hand, columns: throws alternate sites at a beat β; a ball
thrown at 0 lands at t_f and is thrown again at 2β after a dwell d, so
β = (t_f + d)/2 and the empty-hand transit between sites is (t_f − d)/2.
Dwell d below is the legacy stroke-coefficient model (catch 0.998/v, throw
0.614/v); the streamed hand may do better and R2 measures it.

| Apex | v | t_f | d | β | transit | current minimum beat |
|---|---|---|---|---|---|---|
| 0.5 m | 3.13 m/s | 0.64 s | 0.51 s | 0.58 s | 0.06 s | 1.44 s |
| 1.0 m | 4.43 m/s | 0.90 s | 0.36 s | 0.63 s | 0.27 s | 1.70 s |
| 1.3 m | 5.05 m/s | 1.03 s | 0.32 s | 0.68 s | 0.36 s | 1.83 s |
| 1.5 m | 5.42 m/s | 1.11 s | 0.30 s | 0.70 s | 0.40 s | 1.91 s |

The current minimum is the 0.800 s chain floor plus one flight (UH-7a,
2026-09-09): 2.5–3× the needed beat *by construction* of the ring, not by
hardware. Facts that size the work:

- The cup QP is cheap; the gate is not. UH-3 2026-09-06: plan 213–222 ms of
  which `qp` 7–11 ms and `val` 191–202 ms; `validate_cycle` 2.44 ms/knot flat.
- The planner caps flight at 0.80 s with the centroid pinned; the hand ladder
  is measured safe to 4.436 m/s (1.0 m apex, n = 14). The cap is a formulation
  limit and R2 lifts it.
- A 100 mm site-to-site transit in 0.27–0.41 s sits above the 30 000 mm/s³
  session jerk limit as a rest-to-rest quintic and well under the 200 000
  ceiling; the continuous cup trajectory lowers the peak. R2 sizes it with the
  real QP and gate and the owner ramps limits accordingly (decision 4).
- The plant's known errors — a +11 % launch-speed excess (≈ +470 mm/s) and a
  +8.5 mrad +y aim bias — are exactly what an identity-prior learner corrects
  in the first few throws.
- Perception must see every flight: on 2026-09-06 QTM bound the ball to a stale
  rigid body on 5 of 7 throws until the cone body was disabled. That
  precondition is hard.

### 1.3 Owner decisions (2026-09-09)

1. **Columns first**, the oval later. Vertical throws are what the machine and
   its corpus already do, and columns keep the balls out of each other's arc.
2. **Retire the Platform Teensy stroke engine at R1**, first, not last. One hand
   master: the can-bridge streamed lane.
3. **Hollow out the existing nodes in place** on branch `skill-stack`. The FSM
   stack is deleted when the skill stack catches two balls (R4/R5), under a git
   tag `fsm-final`, the way the MPC chain went (`mpc-final`).
4. **Hardware limits are open to ramping.** Every ramp is a logged measurement;
   `leg-gain-tuning-methodology.md` stays the procedure.
5. **The ILC is replaced outright** by the memory-based learner.
6. **Resets are cheap**: the Ball Butler reload is the reset, and the learning
   loop assumes it.
7. **R1 decisions (2026-09-11)**: the hand deviation guard boots ARMED (an
   observing guard on a single-master lane is no guard; `hand7 observe` is a
   bench verb that lasts one armed session — the disarm edge re-arms; the
   first trip is observed COLD via a 3 rev gap re-entry);
   ACTIVATE parks the hand at 0 rev, the clip floor (homing unchanged, the
   0.1 rev first-frame residual of FW 17 row 13 is gone by construction, the
   operator energise step retires from the launch-up path); the Ball Butler
   reload's reactive catch is deleted with the stroke engine and operator
   placement is R3's reset (no LANDING port); the legacy kind-0 toss branch's
   device leaves at R1 and the branch is refused at goal accept (one code: the
   stroke engine is deleted); the FSM itself still goes at R4 under `fsm-final`.

### 1.4 Relationship to other plans

Superseded and archived 2026-09-09 (each carries an archival note naming what
survives): `unified-7dof-planner.md`, `critical-point-ilc.md`,
`toss-selftuning.md`, `toss-pipelined-preamble.md`, `single-ball-toss.md`,
`catch-robustness.md`, `bb-online-juggle-tilt-rearchitecture.md`,
`mvp-trajectory-bringup.md`, `hand-trajectory-generator-overhaul.md`,
`catch-reach-degenerate-overshoot.md`, `inertia-ratio-reconciliation.md`.
`hand-geometry-correction.md` was archived 2026-09-11 with its measurement absorbed
into R1 (32.567 mm/rev; see its archival note). Retained active:
`leg-gain-tuning-methodology.md`, `leg-bus-frame-drops.md`,
`odrive-config-drift-assertion.md`, `bridge-clock-frequency-discipline.md`.
Parked plans are untouched.

## 2. Architecture

### 2.1 Layers

```
Pattern (columns: sites P1, P2; apex h; beat β)                                  motion/skills/schedule.py
   │ compile → Schedule: absolute wall-clock list of Skill(kind, ball, site, t_abs)
   ▼
skill_node (the paper's Orchestrator + Skill Executor; one thread, no solve on the emitter)
   THROW  x ← (site, seat offset of the caught ball); u ← Learner(x, y_d, Memory); u ← clip(AdmissibleBox)
          → install_segment(THROW, ball, site, u, t_abs)
   CATCH  terminal ← tracker landing (pos, vel, t_land) for the ball, re-sent on each update until t_land − 0.1 s
          → install_segment(CATCH, ball, site, landing, t_land)
   REST   → install_segment(REST, site)   — the way out of every attempt
   Outcome: observed landing per throw → Experience(x, u, y) appended to Memory
   │ trajectory/install_segment (srv)
   ▼
trajectory_node — plans FROM ITS LIVE STATE (the only holder of it), installs at the splice knot
   plan_segment(kind, seed, terminal) = cup QP → tilt schedule → decompose → CyclePlan, rest-terminal
   runtime assert: vectorised validate_cycle on the new segment; levelling frame; IK; 40 Hz emitter   (unchanged)
   │ ZMQ :5557
   ▼
teensy_bridge_node → SetpointPump → UDP Setpoint v7 (v6 minus the hand RPCs)                          (unchanged)
   ▼
can-bridge Teensy: 500 Hz Hermite × 7, MAX_DEVIATION guard, fault machine — THE safety authority      (unchanged)
   ▼
leg ODrives 0–5 + hand ODrive 6 (single master after R1)

Perception: QTM 200 Hz → mocap_node → ball_tracker_node (Kalman, landing prediction) → /balls        (unchanged)
Possession: hand ball sensor tri-state, contract C-POSSESS-1                                           (unchanged)
Reset:      Ball Butler reload = a CATCH skill whose terminal comes from an external ThrowAnnouncement  (R4)
```

### 2.2 Data types (pinned; module `motion/skills/`, pure Python)

- `Site(name, cup_mm)` — cup-opening position, xy platform frame, z global
  (the `CycleGoals` convention).
- `Skill(kind ∈ {THROW, CATCH, REST}, ball_id, site, t_abs_s)`; THROW carries
  `y_d = (landing_xy_m relative to the target site, flight_s)`; CATCH carries
  the tracked ball id.
- `Segment` — a `CyclePlan` (7 channels, one clock) plus its splice knot, its
  event mark (release or catch instant, takeoff velocity) and its rest site.
  Built by the existing chain `plan_window → tilt_schedule → decompose →
  CyclePlan.from_realized`; **always rest-terminal** (a THROW is release then
  settle; a CATCH is catch then runway to rest — the existing LANDING kind).
- `Experience(x, u, y, t_abs_s, ball_id, caught)` with x ∈ R⁴ = (site xy,
  seat offset xy of the ball just caught), u ∈ R³ = commanded (landing xy,
  flight), y ∈ R³ = observed (landing xy, flight). SI units.
- `Memory` — append-only rows under `temp/learn/<plant_id>/memory.csv`; a
  malformed row is dropped with a warning, never fatal; kNN query in (x, y_d).
- `AdmissibleBox` — per (site pair, apex band) bounds on u, stamped with the
  limits it was swept under and the gate's git hash; the node refuses to start
  on a limits mismatch.

### 2.3 Frames, units, clock

Sites and goals: millimetres, xy platform frame, z global (as `CycleGoals`).
The cup QP and the learner: SI metres and seconds, so the paper's γ and η carry
over as starting points. Emitted plans: the PLAN frame, stow-relative, with the
levelling correction applied exactly once in `unified_cycle._realize`'s
successor (contract C-LEVEL-1 row E8). Time: CAN wall-clock seconds everywhere
in the schedule; converted once, at the ROS boundary, by `clock_offset.py`.

### 2.4 The beat and the splice

For columns at apex h: t_f = 2·√(2h/g), β = (t_f + d)/2. One cycle of the
schedule (ball A at P1, ball B at P2): THROW(A, P1, 0), CATCH(B, P2, β − d),
THROW(B, P2, β), CATCH(A, P1, 2β − d), THROW(A, P1, 2β), … A skill is
dispatched at `t_abs − lead` (lead ≈ 0.10 s: plan ≤ 20 ms plus a two-knot
install margin). Its segment is planned from the live state at the splice knot
(≥ 2 knots ahead of the emit cursor, for the firmware's u1/u2 lookahead) and
replaces the stream from there. A CATCH is re-sent on each tracker update until
t_land − 0.1 s (bounded to the tracker's rate). If a segment is infeasible at
its scheduled time the skill is refused with the gate's code and the attempt
ends: the previous segment's rest tail is already streaming.

### 2.5 The learner (pinned)

Given target y_d, state x and memory D = {(xᵢ, uᵢ, yᵢ)}:

1. Neighbours: the k = 12 nearest in dᵢ² = ‖xᵢ − x‖²/h_x² + ‖yᵢ − y_d‖²/h_y²,
   weights wᵢ = exp(−dᵢ²). Fewer than k_min = 3 rows → u = y_d (the prior).
2. ū = Σ wᵢ uᵢ / Σ wᵢ.
3. Θ* = [C D d] = (Y W Zᵀ + γ Θ₀)(Z W Zᵀ + γ I)⁻¹ with Z = [δx; δu; 1],
   Θ₀ = [0, I, ū], γ = 0.001 (paper eq. S19).
4. u* = ū + (DᵀD + η I)⁻¹ Dᵀ (y_d − d), η = 0.3 (paper eq. 21).
5. u = clip(u*, AdmissibleBox); an empty box at this site refuses the skill.
6. After the throw, the observed y is appended as one row. No observation → no
   row, and the attempt ends (paper § 2A).

k, k_min, h_x, h_y, γ, η are R3 probe outputs recorded with provenance; the
values above are the starting points. ~150 lines plus tests, no new dependency.

### 2.6 Safety by construction

- **Inside the QP** (exists): jerk boxes, workspace box, catch runway.
- **AdmissibleBox** (R2): `tools/admissible_sweep.py` runs the full
  `validate_cycle` over a grid of (site pair, apex, landing offset, flight) at
  the session limits and writes the box of u for which every segment passes
  with margin; regenerated whenever limits change. This is the paper's MRS,
  applied to the command rather than the joint state.
- **Runtime assert** (R2): a vectorised `validate_cycle` on the new segment
  only, target < 10 ms for a 40-knot segment; a failure refuses the skill.
- **Firmware** (unchanged): `MAX_DEVIATION`, the hand clip, the lead clamp,
  the fault machine. The hand E-STOP band arming policy is an R1 owner decision.

### 2.7 Perception and outcome

`/balls` carries the landing prediction (`landing_position`,
`landing_velocity`, `time_at_land`) per tracked ball. The CATCH terminal is the
latest prediction; the THROW outcome y is the tracker's estimate of the
catch-plane crossing, captured by a probe on existing bags at R3 before the
capture code is written. The possession sensor supplies `caught`. Sitting
preconditions: the cone rigid body disabled, the Ball Butler reflectors masked.

## 3. Implementation Phase Summary

The **Status** column is the one source of truth for where each rung stands;
`Gate` states the acceptance criteria only. (Rung = phase; the column is named
`Rung` for the R0–R6 language used throughout this plan.)

| Rung | Name | Builds | Deletes | Gate | Status |
|---|---|---|---|---|---|
| R0 | Board and substrate | invariant checklist; census-backed dead-layer deletion | dead clusters (§ 6) | `./run_tests.sh --full` green; grep counts zero | ✅ **DONE** — checklist landed 2026-09-10, deletion done 2026-09-09 (`429c660`, `3bfec0b`) |
| R1 | One hand master | can-bridge FW 21 (lane follows `HAS_HAND`, guard boots ARMED, ACTIVATE parks the hand at 0 rev), Platform FW 7 (no stroke engine), PROTOCOL_VERSION 7, `hand_mm_per_rev` measured key, lockstep runbook `tests/hardware/session_skill_stack_r1_flash.md` (completed) | `Trajectory.h`, `hand_source`, `hand_ops`, `HAND_TRAJ_CMD`/`HAND_SOURCE_SET`, `SetHandTrajCmd.srv`, `hand_stroke.py` twin, the legacy kind-0 toss device (its FSM branch refused at accept until R4) | bench ladder re-passes on the FW 21 / Platform 7 pair; a streamed self-toss caught with no latch step | ✅ **DONE 2026-09-11** (`1e2c0c9`, `c52dc27`) — flashed, sat, one streamed self-toss caught with no latch step; a levelling-frame tilt snap found + fixed (`_unified_prelevel`); multi-throw chaining + live guard cold-trip → R2 (`logbook/2026-09-11-skill-stack-r1-sitting-prelevel.md`, `…-one-hand-master.md`) |
| R2 | Skills, schedule, stream (sim) | `motion/skills/`, `install_segment`, vectorised gate, admissible sweep, apex ≥ 1.0 m, `sim/skills_gate.py` | `PlanCycle` modes, ring machinery | 20 columns cycles in sim, no drops; plan < 50 ms on the loaded Jetson | ⬜ **NOT STARTED** (next; R1-carried items folded in — see the R2 section) |
| R3 | Learner + single site | `learner.py`, `memory.py`, outcome capture | ILC/trim/cal/record stack, `toss_ilc_enabled` | in-band within 5 throws from cold, sim and hardware; 10 consecutive catches | ⬜ **NOT STARTED** |
| R4 | Two sites, one ball, BB reset | alternating schedule, reload as a CATCH skill, `Juggle.action`, GUI surface | FSM stack (tag `fsm-final`), `catch_coordinator`, `catch_reach`, old sim gates | 10 consecutive alternating catches; BB reload → catch → throw chain | ⬜ **NOT STARTED** |
| R5 | Two-ball columns | Start/Stop phases, limits ramp as sized at R2 | — | five consecutive cycles, then 30 catches; learning curve logged | ⬜ **NOT STARTED** |
| R6 | Close-out | docs, memory, archival | whatever R5 left dead | plan archived `completed` | ⬜ **NOT STARTED** |

## 4. Implementation Phases (detailed)

Each rung is several agent units of under ~80 tool calls; every unit ends with
the rung's tests passing or a handoff file in the scratchpad.

### R0 — Board and substrate

- **Build.** `motion/skills/INVARIANTS.md`: the invariants ported from
  `REFERENCE_LAYER_CONTRACT.md` (K1–K6), `ARMING_CONTRACT.md`, C-POSSESS-1,
  C-LEVEL-1 (E8), the hand guards (FW 17 rows 12–21) and the emitter/pump
  frame rules — one sentence each, with the enforcement point and the pinning
  test named. Anything in the FSM stack that is an *invariant* rather than an
  implementation is listed here before the stack is deleted.
- **Delete** the census-dead clusters in § 6 with their tests and probes. Each
  cluster is one unit: grep the importers, delete, run the gate, count to zero.
- **Gate.** `./run_tests.sh --full` green on `skill-stack`; the deletion ledger
  in the logbook entry lists every file with its importer count before and
  after.

### R1 — One hand master

- **Firmware, can-bridge.** Delete the `hand_source` latch and both modes:
  the hand lane is always active while a `HAS_HAND` frame is latched. Delete
  `hand_ops` and the `HAND_TRAJ_CMD` / `HAND_SOURCE_SET` RPCs and the 0x6D0
  forward. Fix homing so axis 6 leaves `leg_homing.cpp:195` in the streamed
  lane's controller mode. **PROTOCOL_VERSION 6 → 7**: removing message types
  is a wire change and darkness on skew is the intended failure. Native tests
  (`tests/firmware/native/`) updated in the same unit.
- **Firmware, Platform Teensy.** Delete `Trajectory.h`, the 0x6D0 decode, the
  0x0C9 hand-encoder cache and the stroke-engine constants. Retain the
  inclinometer, time-sync slave and the 0x6E0 cold-start state. Flash over CAN
  (`pio run -e teensy40 -t upload`, launch down).
- **Host.** Delete `SetHandTrajCmd.srv`, the `hand_ops` client path in
  `teensy_link/` and `teensy_bridge_node`, `motion/trajectory/hand_stroke.py`
  (the rev↔mm gain moves to one named constant in `hardware_config`), and the
  stroke-engine coupling in `throw_envelope.py` (its physical limits — end stop,
  regen, torque, the measured coast ladder — survive as inputs to the
  admissible sweep). The hand geometry measurement lands here as the new
  constant's value: `hand_mm_per_rev: 32.567` (two owner readings, 2026-09-06 and
  2026-09-11, agreeing to 0.01 %), replacing `linear_gain_factor` and
  `hand_spool_radius_m`; the runbook confirms it on the streamed lane.
- **Owner decisions.** The hand E-STOP band arming policy (observe-first vs
  armed) — row 18 of the hand ladder was closed on thermal grounds, so the trip
  has never been observed on hardware. Whether homing parks the hand at 0 rev.
- **Gate — MET 2026-09-11** (`logbook/2026-09-11-skill-stack-r1-sitting-prelevel.md`).
  Bench ladder rows re-passed on the flashed FW 21 / Platform 7 pair; a streamed
  self-toss flew and was caught through the unified path with no latch step (the
  latch is deleted). The sitting surfaced and closed a levelling-frame seam: the
  banking launch is built gravity-referenced but was seeded from the machine's
  level-to-base rest, snapping the whole correction angle into knot 0 (5× leg-jerk
  inflation — refused the session-start floor lift at 152 k, inflated the launch to
  the ceiling). Fixed by pre-levelling the platform (`_unified_prelevel`, a
  corrected `go_to_pose`) before the session's first lift, reusing the per-cycle
  positioning's own E3 machinery. **Two items carried to R2:** the chained
  multi-throw solve is not warm-started and overruns the launch lead
  (`ABORTED_NO_RELEASE` on `num_throws>1` — the UH-7a rung); and a live bench-driver
  cold-trip of the ARMED hand guard needs an explicit affordance (`hand_stream_bench`
  clamps `--gap-delta` at 1.5 and its belt caps below the 2.5 firmware band, so the
  trip is proven per-commit in `test_fault_machine.cpp`, not live).

### R2 — Skills, schedule, stream (sim first)

- **Build.** `motion/skills/{sites,schedule,segments,admissible}.py`.
  `segments.plan_segment(kind, seed, terminal, cfg)` wraps `plan_window` and
  returns a rest-terminal `Segment` (THROW = release + settle; CATCH = the
  LANDING kind; REST = settle). `trajectory/install_segment` replaces
  `PlanCycle`'s three modes: trajectory_node plans from its live state and
  installs at the splice knot with the existing continuity guard. A new thin
  `skill_node.py` runs the schedule loop (one timer on the CAN clock, the
  `/balls` subscription, the possession subscription) and is the node
  `reload_coordinator_node.py` is hollowed into at R4. `validate_cycle` is
  vectorised (target < 10 ms per 40-knot segment, pinned by a timing test in
  the `serial` tier). `tools/admissible_sweep.py` writes
  `config/generated/admissible_box.yaml`. The plannable flight is raised to
  ≥ 0.90 s (the release-height lever: `unified_z_float_enabled` or a higher
  catch-prime) with the sim gate as the arbiter.
- **Sizing probe** (before any threshold is written down): with the real QP
  and gate, sweep apex ∈ {1.0, 1.3, 1.5} m × separation ∈ {80, 100} mm × leg
  jerk limit ∈ {30k, 60k, 100k} and record which combinations produce a
  feasible columns schedule with the measured dwell. The owner picks the
  session limits for R4/R5 from that table.
- **Sim.** `sim/skills_gate.py` drives `skill_node`'s pure core against the
  MuJoCo plant with the kinematic-capture authority (learner off, sites
  perfect) under the noise model; `sim/cycle_gate.py` and `sim/unified_gate.py`
  are deleted once it passes.
- **Gate.** 20 consecutive columns cycles in sim at apex 1.0 m, separation
  100 mm, zero drops on five seeds; per-skill plan < 50 ms measured on the
  Jetson with the launch up and a bag recording; the sweep completes in < 5 min.
- **Carried from the R1 sitting (2026-09-11).** (a) Warm-start the chained launch
  solve — a cold chained solve ballooned to ~2.2 s against the 1.8 s launch lead
  and aborted `ABORTED_NO_RELEASE` on `num_throws>1`; the schedule/segment install
  must not pay the cold cost inside the beat. (b) A live bench-driver cold-trip
  affordance for the ARMED hand guard: `hand_stream_bench` clamps `--gap-delta` at
  1.5 and its belt caps at 2.0 rev below the 2.5 firmware band, so the trip can only
  be commanded through a new opt-in that raises both together; until then the
  ARMED trip is proven per-commit in `test_fault_machine.cpp`, not live.

### R3 — Learner, then single-site hardware

- **Build.** `motion/skills/{learner,memory}.py` per § 2.5; the outcome capture
  (tracker crossing → `Experience`) designed from the bag probe; a
  `tests/motion/test_learner.py` that pins the closed form against a NumPy
  reference and the paper's identity-prior limit (no rows ⇒ u = y_d).
- **Sim validation.** Inject the measured plant errors (+11 % launch speed,
  +8.5 mrad aim) into the MuJoCo throw; from a cold memory the landing error
  must enter the cup band within 5 throws on 5 seeds, and the memory must be
  monotone in error over the next 20.
- **Hardware.** Single site: THROW(P1) → CATCH(P1) → REST, repeated; the
  first rung a learner ever runs on this machine. The runsheet
  `tests/hardware/session_skills_r3.md` is dress-rehearsed on the loaded
  Jetson with every refusal reported at once.
- **Delete.** `motion/toss_ilc.py`, `toss_trim.py`, `motion/toss_cal.py`,
  `toss_record.py`, `tests/hardware/ilc_fit*.py`, `toss_fit_lib.py`,
  `toss_cal_*.py`, `config/toss_ilc.yaml`, `config/toss_calibration.yaml`, the
  `toss_ilc_enabled` flag, and the probes that mined them.
- **Gate.** Hardware: in-band within 5 throws from a cold memory, then 10
  consecutive catches at one site; the learning curve (landing error vs throw
  index) in the logbook with the bag id.

### R4 — Two sites, one ball, the Ball Butler reset

- **Build.** The alternating schedule; the aim compensation for a moving
  release (the QP's ballistic constraint already includes cup velocity);
  reload re-cut as a CATCH skill whose terminal is the BB `ThrowAnnouncement`
  landing; `Juggle.action` (pattern, apex, separation, num_cycles) replacing
  `Toss`, `TossContinuous` and the reload FSM; the GUI goal surface reduced to
  pattern + start/stop; the session limits chosen at R2 applied.
- **Delete** (tag `fsm-final` first): `toss_sequencer.py`, `toss_session.py`,
  `reload_sequencer.py`, `catch_coordinator.py` + node, `catch_reach.py`, the
  ring machinery in `unified_cycle.py` (`extend`, `replan_tail`, `_concat_*`,
  `_gate_joined`, supersede), `PlanCycle.srv`, the `unified_cycle_enabled`
  and `toss_*` flags, and the FSM tests. `reload_coordinator_node.py` becomes
  `skill_node.py`'s shell.
- **Gate.** 10 consecutive catches alternating P1/P2 on hardware; a BB reload
  → catch → throw → catch chain from the GUI with one button.

### R5 — Two-ball columns

- **Build.** Start phase (hand holds A; B arrives from the Ball Butler as a
  CATCH at P2, or is placed) and Stop phase (the last two CATCHes end in REST
  with both balls held or one held and one delivered to the cone). The
  learning loop runs across resets with the memory retained.
- **Gate.** Five consecutive cycles (the paper's criterion), then 30 catches;
  the consecutive-catch count per attempt logged as the learning curve.

### R6 — Close-out

Delete what R5 left dead, review `docs/` (the MPC-era motion-planner pages),
update memory, archive this plan `completed` with the (date, command, result)
triple of the final gate.

## 5. Testing Plan

- **Unit (offline).** Schedule compilation (β, t_abs, ordering, the lead);
  segment rest-terminality and C2 continuity at the splice knot; learner closed
  form vs reference and the prior limit; admissible box round-trip and the
  limits-mismatch refusal; the runtime assert's timing (`serial`).
- **Integration (real processes, no actuators).** `skill_node` ↔
  `trajectory_node` over the real services with a scripted `/balls` stream;
  the emitter's 40 Hz cadence unchanged during a solve (`max_emit_gap_ms` p50
  ≈ 26 ms, max < 40 ms, the UH-6 numbers); a refused skill leaves the rest
  tail streaming.
- **Sim.** `sim/skills_gate.py` per rung, under the hardware-faithful noise
  model, the kinematic-capture authority.
- **Hardware.** One runsheet per rung under `tests/hardware/`, dress-rehearsed
  on the loaded Jetson, every refusal reported at once, bag id and boot banners
  recorded. `./run_tests.sh --full` before every sitting.
- **Regression.** Marker vocabulary stays `serial` + `nightly`; deleted code
  takes its tests with it in the same commit; the wire fixtures for the leg
  lanes stay byte-pinned through R1's version bump.

## 6. Deletion ledger (census 2026-09-09, non-test importers)

| Cluster | Files | Importers | Rung |
|---|---|---|---|
| Offline juggle demo | `sim/juggle_demo.py`, `sim/juggle_planner/{juggle_optimizer,player,timeline,trajectory,pattern}.py` | each other only | R0 — **done 2026-09-09** |
| MPC-era sim sources | `controller/{scheduler,target,zmq_target,toss_motion_source,catch_optimizer}.py`, `controller/SCHEDULER_CONTRACT.md`, `sim/hand/`, `sim/input/` (trace `sim/main.py` first: it imports only `sim.plant` and `sim.viz.telemetry`) | `controller/__init__.py`, each other | R0 — **done 2026-09-09 (partial — census missed live importers; `controller/target.py`, `sim/hand/{ballistics,coordinator,planner,trajectory}.py`, `sim/input/{toss_loop,sim_control,scripted}.py` kept, see logbook)** |
| MPC telemetry analysis | `controller/telemetry.py`, `sim/analysis/`, `/diagnose` (trace: keep any rosbag path) | `sim/analysis`, `sim/juggle_demo.py` | R0, trace first — **traced 2026-09-09, kept in full: `sim/analysis/diagnose.py` does live rosbag (MCAP) analysis integrated with MPC-CSV analysis; `controller/telemetry.py` required by the protected `sim/viz/telemetry.py` shim; see logbook** |
| Historical MPC docs | `docs/sim_mpc/` + its mkdocs nav | none | R0 — **done 2026-09-09** |
| Phase-runner workflows | `.claude/workflows/*.js` | none | **done** |
| Stroke engine + latch | `Teensy_code_platform/Trajectory.h`, `hand_source.*`, `hand_ops.*`, `hand_stroke.py`, `SetHandTrajCmd.srv` (+ `tools/probes/{hand_stroke_timeline,cadence_rung_check,ilc_speed_band}.py`, `ros_ws/docs/hand_decel_feedforward.md`, the kind-0 timing model in `toss_sequencer`/`toss_session`, the reactive arm in `catch_coordinator_node`; `reload_coordinator_node`'s non-unified goals are REFUSED at accept, the branch itself dies at R4) | R1 list | R1 — **done 2026-09-11** (deleted, importers re-pointed to the generated `HAND_REV_PER_M` / `HAND_HOMED_REST_FLOOR_REV` / `HOMING_HAND_{SETTLE,PARK}_BAND_REV`; grep-to-zero on the code, dated retirement notes remain) |
| Learning stack | `toss_ilc.py`, `toss_trim.py`, `toss_cal.py`, `toss_record.py`, `ilc_fit*.py`, yaml artifacts | `reload_coordinator_node.py` | R3 |
| FSM choreography | `toss_sequencer.py` (3 577), `toss_session.py` (2 127), `reload_sequencer.py`, `catch_coordinator*.py`, `catch_reach.py`, ring half of `unified_cycle.py` (2 711), `PlanCycle.srv`, old sim gates | `reload_coordinator_node.py` (12 955) | R4 |

Retained deliberately: `motor_guard.py` (the hermite xref twin, per-commit
safety tests), the tilt map and levelling, the tracker, the transport, the
firmware guards, the Ball Butler and cone stacks.

## 7. Notes for Collaborators

- **Briefing.** Extract the rung's section, § 0, § 2.2–2.6 and only the files
  the unit touches into a scratchpad file; never hand an agent this whole plan.
  Sonnet by default; Opus only for the segment/QP work in R2, the firmware in
  R1 and the learner probe in R3, one per unit, justified in the brief.
- **Session re-entry.** `cd ~/Desktop/Jugglebot-skills && claude`. Read first:
  this plan § 0–2, the rung in flight, `motion/skills/INVARIANTS.md` (from R0),
  the latest logbook entry whose `related_plan:` is this file, and
  `tests/hardware/session_skills_r<N>.md` if a sitting is next. The worktree has
  its own memory namespace; this plan and the logbook are the re-entry
  vehicle, and the main-tree memory file `project_two_ball_skill_stack.md` is
  the pointer.
- **Parallel lines.** `mvp-trajectory-bringup` continues as the firmware
  validation line; the `hand-geometry-correction` branch and worktree are a record
  only (archived 2026-09-11, measurement absorbed at R1) — remove the worktree once
  R1 lands. `git fetch && git status -sb` before every push.
- **Risks, ranked.** (1) The tracker's per-throw landing observation is the
  learner's food; a blind flight is a lost row and the QTM preconditions are
  non-negotiable. (2) The transit sizing may force a jerk ramp beyond what the
  legs track cleanly; the R2 probe and the tuning methodology decide, not a
  guess. (3) Hollowing 13 k lines of coordinator in place can leave orphaned
  behaviour; the R0 invariant checklist is the net, and `fsm-final` is the
  rollback. (4) The first streamed catch without the stroke engine changes the
  contact softness; R1's bench rows and R3's single-site rung see it before
  two balls are in the air.
- **Rollback.** Each rung is its own commit series on `skill-stack`; the FSM
  stack is recoverable at `fsm-final`, the stroke engine at the R1 commit's
  parent, and `main`/`mvp-trajectory-bringup` are untouched.
