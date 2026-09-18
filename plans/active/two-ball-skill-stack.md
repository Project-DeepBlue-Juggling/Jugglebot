---
title: Two-ball skill stack — schedule-driven throw/catch skills, one hand master, memory-based learning
created: 2026-09-09
status: active
owner: Harrison
last_updated: 2026-09-13
related_logbook:
  - 2026-09-09-two-ball-skill-stack-kickoff.md
  - 2026-09-11-skill-stack-r1-one-hand-master.md
  - 2026-09-11-skill-stack-r1-sitting-prelevel.md
  - 2026-09-12-skill-stack-r2-skills-schedule-stream.md
  - 2026-09-13-skill-stack-r2-plan-gate-runsheet.md
  - 2026-09-13-skill-stack-r2-gate-sittings.md
  - 2026-09-13-skill-stack-r3-learner-single-site.md
related_config:
  - config/hardware_config.yaml → jugglebot_operational.unified_cycle_enabled (retires at R4)
  - config/hardware_config.yaml → jugglebot_operational.toss_ilc_enabled (retires at R3)
  - config/hardware_config.yaml → trajectory_op.leg_jerk_limit_mmps3 (the ramp lever, sized at R2)
  - config/generated/admissible_box.yaml (machine-written by tools/admissible_sweep.py since R2)
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py::plan_window
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cycle_plan.py::CyclePlan
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py::validate_cycle (vectorised at R2)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/{sites,schedule,segments,executor,admissible}.py (R2)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/{learner,memory}.py (R3)
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py::state_at_knot / splice_at (R2)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py (R2)
  - ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv (R2)
  - sim/skills_gate.py (R2)
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

**Physical operating point (current, provenance-dated).**
- Launch leg session limits (`jugglebot_launch.py` via `config/hardware_config.yaml`
  `trajectory_op`): **300 mm/s / 5000 mm/s² / 150 000 mm/s³** (owner decision,
  2026-09-16, "300/5000/150000 are safe enough" — supersedes the S4 point
  1000/5000/30000; this is the R2/R3 operating point and the admissible
  sweep's own limits, flown clean through the R3 apex ladder,
  `logbook/2026-09-16-apex-ladder-k07-ab-result.md`).
- Hand torque feedforward gain (`teensy_bridge_node` param
  `hand_torque_ff_gain`, wire `hand_ff_gain`): **K = 0.7**, set by
  `jugglebot_launch.py` since 2026-09-16 (node's own declared default stays
  0.0 as the bare-launch fail-safe). Closed the R3 apex-ladder A/B measuring
  hand meas/cmd overspeed 1.05–1.22× (mean up to 1.13×) (K=0) → 1.00–1.03× (K=0.7) and ball
  apex ratio ~1.25× → ~1.08×, at peak current well under the 48 A live
  cutoff (max 39.9 A). The pre-registered "K=0.7 → ratio ≤ 1.00" criterion
  was **not** met literally; K=0.7 is adopted anyway (owner decision) —
  residual overspeed is the learner's job, not a further hand-tuning
  target. See `logbook/2026-09-16-apex-ladder-k07-ab-result.md` for why not
  K=1.0 yet.
- Owner decision, 2026-09-16 (verbal, no logged session — provenance is the
  owner's BallButler experience): **columns/oval site separation target for R4 =
  250 mm** (BallButler's own columns ran at ~180 mm). The current
  `separation_mm` code default (100 mm) and the R2/R3 swept admissible boxes
  stay as-is until R4 re-sweeps at 250 mm — this is a forward target, not a
  change to make now.

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
  **Measured at R2 (2026-09-12, `tools/probes/skills_sizing_sweep.py`): the
  HAND binds before the legs.** At 1.0 m the steady columns window needs
  3756–3833 rev/s² against the 3500 session cap at every separation, banking
  setting, z-float setting and chain depth; 1.3/1.5 m need a 900/940 mm
  release and then cannot stop inside the 1004 mm stroke top. Feasible cells
  begin at 0.8–0.9 m, all with leg jerk at the 200 000 ceiling; the owner's
  R2 operating point is the 0.9 m row of the R2 section.
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
5. **The ILC is replaced outright** by the memory-based learner. **Its
   command and outcome are the same physical quantity** (amended 2026-09-18):
   `u = (landing offset x, y [m], apex above the catch plane [m])` and
   `y` the same three, all read off the tracker's converged gravity-fixed fit.
   The learner does not learn a time — a flight time is measured from the
   commanded release knot, which the physical release lags by 0.02–0.14 s
   throw to throw, through a crossing estimate itself extrapolated to ±40 ms,
   and those two biases drove the plant ~20 % low in apex while the metric
   read on target. The schedule's flight time is DERIVED from the commanded
   apex (`schedule.flight_s`) and is not learnable.
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
  `y_d = (landing_xy_m relative to the target site, apex_m)` (an apex since
  2026-09-18, § 0 item 5); CATCH carries the tracked ball id.
- `Segment` — a `CyclePlan` (7 channels, one clock) plus its splice knot, its
  event mark (release or catch instant, takeoff velocity) and its rest site.
  Built by the existing chain `plan_window → tilt_schedule → decompose →
  CyclePlan.from_realized`; **always rest-terminal** (a THROW is release then
  settle; a CATCH is catch then runway to rest — the existing LANDING kind).
  **Amended at R2 (owner, 2026-09-12):** a CATCH skill MAY carry the next
  same-site throw (`Skill.then_throw`); its segment is then the existing STEADY
  kind (catch at t_land, release at t_release) plus a SETTLE tail — still
  rest-terminal. Measured: the pinned split form (LANDING, then a THROW spliced
  one knot after touch-down) refuses at every cell of a 480-cell grid because
  the runway decelerates the hand toward rest and the throw must undo it with
  `dwell − lead` left (260k mm/s³ at the operating point); the whole-window
  form passes at 178k of 200k. This is § 1.1's last row made concrete: the
  platform moves continuously through catch and throw.
- `Experience(x, u, y, t_abs_s, ball_id, caught)` with x ∈ R⁴ = (site xy,
  seat offset xy of the ball just caught), u ∈ R³ = commanded (landing xy,
  apex above the catch plane), y ∈ R³ = the same three observed — the apex
  from the fit's crossing speed, `h = v_z²/2g` (2026-09-18, § 0 item 5). SI
  units. The CSV names those columns `u2_apex_m`/`y2_apex_m` and a
  pre-2026-09-18 flight-time file is REFUSED on load, not reinterpreted.
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

**Measured at R3** (owner decision 2, 2026-09-13; probe `probe_learner.py` +
`probe2_out/`, scratchpad): k = 16, k_min = 2, h_x = 0.01 m, h_y = (0.05 m,
0.05 m, 0.10 m — an APEX bandwidth since 2026-09-18; it was 0.2 s, which was
1.5× the whole explored command range), γ = 1e-2, η = 0.2 — SI units only, γ does not carry to this
plan's mm-frame constants elsewhere. These supersede the k = 12 / k_min = 3 /
γ = 0.001 / η = 0.3 starting points above. Centring (paper eq. S18): δx is
about the query state x; δu is about the weighted mean ū from step 2, not
about y_d. Probe finding: at h_y = 0.02 the start point never learns — 0.02 is
below the cold-start landing error, so its neighbourhood weight underflows to
zero and the identity prior holds regardless of k_min; this is why the adopted
h_y is larger. A non-finite u (NaN/inf, from an underflowed weight sum) raises
and the skill is refused rather than commanding an unclipped trajectory (a
defect found and fixed this rung, pinned by `test_learner.py`).

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
`landing_velocity`, `time_at_land`) per tracked ball, predicted at the 830 mm
catch plane (`sites.CATCH_CUP_Z_MM`; moved from 809.08 mm at R3 — the FSM's
catch plane moves with it, decision 5). The CATCH terminal is the latest
prediction. **Landed at R3:** the THROW outcome y is the tracker's last
estimate of the catch-plane crossing, taken outside a 0.012 s guard
(`OUTCOME_GUARD_S`, `executor.py`) around the scheduled landing instant so a
stale near-crossing sample is never captured; flight is the crossing instant
minus the throw's scheduled release; `caught` is the possession sensor's
evidence read SEATED at ANY tick inside a window around the **OBSERVED**
landing — `[min(t_sched, t_obs) - CAUGHT_LEAD_S, finalise_at]`, where
`finalise_at` is `CAUGHT_WINDOW_S` = 0.35 s past the later of the two, bounded
by `CAUGHT_LAND_DEFER_CAP_S` = 0.35 s (`executor._outcome_window`, amended
2026-09-16; it was one sample 0.15 s after the SCHEDULED landing, which read an
empty cup on 4 of 5 real catches — see § R3's dated paragraph).
Sitting preconditions: the cone rigid body disabled, the Ball Butler
reflectors masked.

**The landing observation FREEZES at the crossing (2026-09-16, same day).**
The window above is also the window the row's LANDING was refreshed in, and at
the chained operating point it reached past the ball's NEXT release: the 16:22
sitting wrote observed flights of 2.2317 / 2.2310 / 1.1554 s for an 0.8569 s
command (`armB-090` attempt 1), the learner obeyed them down to a 0.686 s
command and the operator saw "very low throws". A landing estimate is now
admitted by one gate (`executor._consider_landing`) only while the ball is
still in the air — it must come from a CONVERGED ballistic fit
(`Landing.from_fit`, 2026-09-18: 16 of the 22 rows written on 2026-09-17 had
none), post-date the release, be sampled strictly BEFORE
the crossing it predicts (`OUTCOME_GUARD_S`, no longer an `abs()` test), lie
inside `memory.APEX_RATIO_BAND` = (0.25, 2.56) × the COMMANDED APEX — the old
flight band (0.5, 1.6) squared, since apex goes as the square of release
speed (2026-09-18) — and
arrive before `min(t_sched, t_obs)`; the last survivor stands and nothing after
the landing can replace it. `finalise_at` is additionally bounded to
`t_next_release − OUTCOME_NEXT_RELEASE_EPS_S` (0.010 s, `_next_release`). The
band is one definition in `memory.py` and is enforced on `Memory` load and
append as well; a row with no admissible landing is still no row. `caught=False`
rows continue to feed the memory (plan § 2.5 step 6 / paper § 2A — the command
learner models where the ball LANDED, not whether it was kept). Replay of both
2026-09-16 sittings (`tools/probes/outcome_landing_replay.py`): of 43 written
rows the new rule refuses 21 and admits 22 at y/u = 0.974–1.528, against
0.979–2.985 as written. ⚠ open: the tracker's in-flight estimate ran ~one beat **CLOSED same day: it was the host, not the tracker — `_tracker` served the NEXT announced id; correlation is now per RELEASE (`ball_possession.flight_in_progress`, entry `2026-09-16-tracker-correlation-follows-the-flight-in-progress`), replay 43/43 rows admitted. Interim guard the same day: `learner_lateral_authority_mm` = 0 (the learner corrects flight only) until the banking small-offset defect (`logbook/2026-09-16-banking-saturates-on-small-lateral-offsets.md`: cup_realize `tilt_to_receive` saturates to its 12° clamp for ANY nonzero lateral residual during the pre-catch dive) is fixed at its root.**
late on every chained throw but the last, so a chained sitting now yields one
row per attempt until that is diagnosed. Entry:
[2026-09-16-outcome-landing-frozen-at-the-crossing](../../logbook/2026-09-16-outcome-landing-frozen-at-the-crossing.md).


**Catch aim source (owner decision 2026-09-15; SUPERSEDED in its default by
2026-09-18 below): the catch does NOT depend on the tracker.** At the 2026-09-15 sitting all 13 self-tosses ended
`NO_LANDING` — mocap never produced a marker for the flying ball, so the
catch was never aimed. A catch is now aimed at the landing its ball's
previous release was *commanded* to achieve (`executor._predicted_landing`,
the same construction § 2.5's learner treats as its command), dispatched at
its own scheduled instant; `skill_node`'s `catch_aim_source` parameter
selects `schedule` (the live default until 2026-09-18), `schedule_hand` (that landing re-flown
with the MEASURED hand launch-speed ratio `r = v_meas/v_cmd` from
`/hand_telemetry`, `motion/skills/hand_launch.py` — at 0.9 m the sitting's
r ≈ 1.086 is ~74 ms of late arrival) or `tracker` (the pre-2026-09-15 path,
kept because the sim gate's refine surface is built on it). QTM is not used
for catch prediction for the time being. Outcome capture is unchanged and
still tracker-sourced: no observation, no memory row — so with mocap blind
the learner simply stays at its identity prior. Entry:
[2026-09-15-open-loop-catch-from-throw-state](../../logbook/2026-09-15-open-loop-catch-from-throw-state.md).

**Tracker un-blinded (2026-09-15, same day, after the decision above).** The
reason mocap "never produced a marker" was not mocap: `ball_tracker_node`
forwarded only markers whose QTM `label` was EMPTY, and QTM's AIM model had
labelled the flying ball `Ball Butler - 1` on all 13 throws (711→1491 mm on
armA-050, 715→2002 mm on armA-090 — the only label in the frame that moves).
The node now forwards **every** marker with its label; the matcher excludes
only the robot's own rigid bodies (`Platform`, `Base` —
`ball_tracking.excluded_label_prefixes`), by rigid-body MEMBERSHIP and never by
label shape, since the ball itself wore a Ball-Butler label. An ANNOUNCED ball
is CONFIRMED by the eligible marker nearest its **analytic** ballistic expected
position within `ball_tracking.announced_gate_mm` = 200 mm — a sphere, not a
±200 mm box, because three of the four platform markers sit 204.6–220.7 mm from
the cup and fall inside the box. The old 880 mm height floor is retired (the
ball sits in the cup at 717–742 mm at the throw instant) and the human-throw
parabolic path is off by default (`detect_human_throws`). Replaying the
sitting's own bag through the real matcher
(`tools/probes/tracker_bag_replay.py`) goes **0/13 → 13/13** CONFIRMED within
9 ms of release, with the deadline landing estimate +0.042…+0.128 s past the
announcement — i.e. the tracker corrects the plant's ~25 %-fast throw — and
2026-09-13's chained bag is unchanged at 1/3. **`catch_aim_source` is untouched
and still defaults to `schedule`**: the open-loop catch is now a choice rather
than a necessity, and whether to move it back to `tracker` for the ladder is an
open owner decision — **ANSWERED 2026-09-18, next paragraph.** Entry:
[2026-09-15-tracker-all-markers-gated-to-expected-ball](../../logbook/2026-09-15-tracker-all-markers-gated-to-expected-ball.md).

**The catch is aimed from the tracker's converged fit, with the schedule as
its prior (owner decision 2026-09-18).** The release instant slips
0.019–0.137 s from its knot, throw to throw (2026-09-17, `flight_truth3`);
that is not a plant gain the learner can absorb, it is a disturbance, and the
only way to know it is to watch the ball. Since `ce6d603` the tracker confirms
every flight (22/22 on 2026-09-17) with a converged gravity-fixed fit,
typically by the apex, so `catch_aim_source` now defaults to `tracker` and the
aim is ONE ordered rule (`executor._catch_aim`), for catch-with-throw and
standalone catches alike: **the converged fit if there is one, else the
schedule's commanded landing (`_predicted_landing`), else an unfitted Kalman
crossing** — which ranks below the prior because its crossing runs 0.06–0.20 s
late and grows later through the descent, and is only ever reached by a ball
this schedule never threw (columns' very first catch). No step of that order
waits: the 2026-09-15 lesson holds, a catch that waits for perception is a
catch that does not happen, and `NO_LANDING` now survives only for the catch
with neither a prior nor any landing by the deadline. Later fits then refine
the committed catch (`_resend_live_catch`), unchanged in its two timing fences
and newly gated on three worth-it facts: only a `from_fit` landing, only a
move beyond 10 mm / 0.010 s (the catch's own timing cliff is ~20 ms wide — the
ball seated +0.015 s after the scheduled landing on the two catches that
bounced and +0.104 s on the four that seated smoothly, 2026-09-17 — and the
retired 1 mm / 2 ms pair was inside the tracker's noise), and at most
`resend_max_per_catch` = 2 re-aims, because one re-solve costs 25–130 ms of
orchestrator time on the loaded Jetson (six `SPLICE_TOO_LATE` at the
2026-09-17 23:49 sitting) and a jittering estimate must not spend the splice
budget of the catch it is refining. Each fence that turns a real candidate
away logs one line with the delta it turned away. The learner's outcome row is
unaffected — it reads the fit directly (§ 2.5). This is the paper's
arrangement (p6: replan the catch from vision until 0.1 s before contact; its
one open-loop catch class is the one that never converged, p8).

**The hand-park REFUSAL is retired; the SEED is the enforcement point (owner
decision, 2026-09-16).** The ladder row `REJECTED_HAND_NOT_PARKED` — added at
R3 Unit B as the proxy for latch L2's failure class ("the plan's hand seed is
not where the hand actually is") — is **gone**, along with the `hand_at_seed` /
`hand_at_park` observations and the `fresh_origin` argument that gated it. At
the 2026-09-16 sitting it refused **nine** consecutive schedules at skill 0
(the opening REST) on a hand MEASURED at 0.0000 rev: an ended attempt had
installed a hold at +0.5639 rev, and the bridge's `pos_cmd` echo — which is
event-driven off the streamed lane and is never touched by the firmware-internal
ACTIVATE park — stayed frozen at the held value, so the tracking-error half saw
a phantom 0.56 rev error. The runsheet's own DEACTIVATE/ACTIVATE recovery for
that code could not clear it, and the sitting ended.

The invariant survives one level down, as a CORRECTION rather than a refusal:
`trajectory_node._cycle_start_state` now reconciles a fresh-origin window's
COMMANDED hand seed against the MEASURED hand whenever the machine is at rest
and the two disagree by more than `_SEED_HAND_RECONCILE_TOL_REV` = 0.05 rev
(the firmware's own `SCHED_RESUME_TOL_POS_HAND_REV`), seeding knot 0 from the
encoder and logging one WARN naming both values. **The opening REST then
carries the hand home** — it is a SETTLE aimed at `SETTLE_CUP_Z_MM`, so it
already plans the hand from its seed to the settle clamp (0.3071 rev, inside
the 0.5 rev park band). MEASURED (2026-09-16 probe, R3 limits): the 1.5 s
`FLOOR_LIFT_S` REST plans CLEAN from every seed in the hand's 9.959 rev stroke,
peaking at 9.76 rev/s — so no seed needs the window stretched and none is
refused. `REJECTED_HAND_STALE` is KEPT and is now load-bearing (the seed is
reconciled against the encoder). `_install_continuity_ok`'s 1.0 rev hand bound
is the deliberate outer limit on the reconciliation. Entry:
[2026-09-16-hand-park-refusal-retired-rest-homes-the-hand](../../logbook/2026-09-16-hand-park-refusal-retired-rest-homes-the-hand.md).

**The outcome verdict is a WINDOW around the OBSERVED landing, and the legacy
catch re-aim is refused on a skill-stack plan (2026-09-16).** The R3 sitting
caught 5/5 singles and the learner was told it had caught 1: `caught` was one
possession sample at `t_land_scheduled + 0.15 s`, and the plant's ~8 % fast
throw put the arrival +0.06..+0.20 s later than that, debounce on top. The
verdict now LATCHES on any SEATED reading inside
`[min(t_sched, t_obs) − CAUGHT_LEAD_S, finalise_at]`, where `finalise_at`
follows the OBSERVED landing bounded by `CAUGHT_LAND_DEFER_CAP_S` = 0.35 s.
MEASURED (`tools/probes/caught_window_bag_probe.py`): the landing→SEATED delay
is **not one-signed** — +43..+192 ms on 09-16 (plus one +282 ms bobble) and
−39..−48 ms on all 22 throws of 09-15 — so `CAUGHT_LEAD_S` = 0.10 and
`CAUGHT_WINDOW_S` = **0.35** (owner ruling: a catch that SETTLES LATE is a
catch — the +282 ms armA-050 arrival was re-thrown, not dropped, and that
attempt is logged "worked"; at 0.25 s it scored False). Replay of the sitting's
five rows: **all five read `caught=True`**, matching what the operator recorded. Separately: the sitting's 160 `REPLAN_WINDOW` ERRORs were NOT the
executor probing the planner (every `install_segment` was accepted first call) —
they are the FSM-era `catch/dynamic_target` chain, woken by the stack's own
throw announcement, re-aiming a skill-stack plan 24 knots before a committed
release. `trajectory_node` now records which install path owns the active
`CyclePlan` and refuses a `dynamic_target` against a segment-owned one
`SEGMENT_OWNED` before any solve, with `REPLAN_WINDOW` demoted to a throttled
WARN. `_cycle_stroke_floor`'s exact-zero knife-edge (flagged unfixed above) is
closed. ⚠ `SPLICE_TOO_LATE` on armA-060 skill 4 is analysed and OPEN: zero
refused replans ran in the 0.49 s before that 0.147 s solve (median 64 ms), so
the spam was not its cause. Entry:
[2026-09-16-outcome-window-and-computed-catch-deferral](../../logbook/2026-09-16-outcome-window-and-computed-catch-deferral.md).

## 3. Implementation Phase Summary

The **Status** column is the one source of truth for where each rung stands;
`Gate` states the acceptance criteria only. (Rung = phase; the column is named
`Rung` for the R0–R6 language used throughout this plan.)

| Rung | Name | Builds | Deletes | Gate | Status |
|---|---|---|---|---|---|
| R0 | Board and substrate | invariant checklist; census-backed dead-layer deletion | dead clusters (§ 6) | `./run_tests.sh --full` green; grep counts zero | ✅ **DONE** — checklist landed 2026-09-10, deletion done 2026-09-09 (`429c660`, `3bfec0b`) |
| R1 | One hand master | can-bridge FW 21 (lane follows `HAS_HAND`, guard boots ARMED, ACTIVATE parks the hand at 0 rev), Platform FW 7 (no stroke engine), PROTOCOL_VERSION 7, `hand_mm_per_rev` measured key, lockstep runbook `tests/hardware/session_skill_stack_r1_flash.md` (completed) | `Trajectory.h`, `hand_source`, `hand_ops`, `HAND_TRAJ_CMD`/`HAND_SOURCE_SET`, `SetHandTrajCmd.srv`, `hand_stroke.py` twin, the legacy kind-0 toss device (its FSM branch refused at accept until R4) | bench ladder re-passes on the FW 21 / Platform 7 pair; a streamed self-toss caught with no latch step | ✅ **DONE 2026-09-11** (`1e2c0c9`, `c52dc27`) — flashed, sat, one streamed self-toss caught with no latch step; a levelling-frame tilt snap found + fixed (`_unified_prelevel`); multi-throw chaining + live guard cold-trip → R2 (`logbook/2026-09-11-skill-stack-r1-sitting-prelevel.md`, `…-one-hand-master.md`) |
| R2 | Skills, schedule, stream (sim) | `motion/skills/{sites,schedule,segments,executor,admissible}.py`, `unified_cycle.state_at_knot`/`splice_at`, `InstallSegment.srv` + `trajectory/install_segment`, `skill_node.py`, vectorised `validate_cycle`, `tools/admissible_sweep.py`, `sim/skills_gate.py`, `hand_stream_bench --trip-guard` | `sim/cycle_gate.py`, `sim/unified_gate.py` (+ their tests); the per-sample `validate_cycle` loop. **`PlanCycle` and the ring policy stay for the FSM until R4** (owner, 2026-09-12 — see the R2 section) | 20 columns cycles in sim at the owner's operating point (0.9 m / 100 mm — re-sized at R2), no drops, five seeds; plan < 50 ms on the loaded Jetson | ✅ **DONE** — sim gate MET 2026-09-12 (20/20 × 5 seeds, 0 drops); **hardware gate MET 2026-09-13** on the third no-motion sitting (rows 15/16 PASS all five gates, worst solve 47.9 / 49.0 ms, handoff margin 73–74 ms; non-gating row 17 failed G1/G3 under two extra busy cores) — `4d49e04`, `40371fe` (`logbook/2026-09-12-skill-stack-r2-skills-schedule-stream.md`, `…/2026-09-13-skill-stack-r2-gate-sittings.md`) |
| R3 | Learner + single site | `learner.py`, `memory.py`, outcome capture | ILC/trim/cal/record stack, `toss_ilc_enabled` | in-band within 5 throws from cold, sim and hardware; 10 consecutive catches | 🟡 **SIM MET, HARDWARE OUTSTANDING (2026-09-13)** — landed: the learner + memory, the single-site chained schedule, outcome capture, the precondition ladder (pre-level, floor lift) and a working `skill_node` shell. **Sim criterion MET 2026-09-13**: policies A and B, seeds 0–4, in-band by throw 3 (A) / 5 (B), monotone, 0 drops, repeat runs bit-identical. ⚠ **Sim criterion RE-OPENED 2026-09-14 in xy only**: the dense apex-scoped re-sweep's (P1, P1) 0.9 m box admits x 0…+40 mm, y 0 (the old ±40 × ±30 mm box claimed offsets the chained catch fails at 90 % margin at 0.77/0.81 s flights, item (k)); on it policies A and B, seeds 0–4, land flight in band by throw 3 / 5 with 0 drops but never enter the xy band (the sim's +8.5 mrad aim error is +y) — restoring xy authority needs item (k) resolved. **Sitting 1 (2026-09-13 evening) did not reach the gate**: 5/5 single throws caught but untracked (a plain THROW never announced — fixed), 2–3/5 chained, two hand-axis `MAX_DEVIATION` latches (an ended attempt's plan kept throwing; an opening REST from an un-parked hand), the BB reload retired at R1 — five Jetson-side fixes landed 2026-09-14; ⚠ **the plant throws ~25 % fast (apex 1.38 m for 0.9 m) and the learner's box cannot reach it — owner decision on the hand acceleration ceiling before sitting 2** (`logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md`). Outstanding: sitting 2, `tests/hardware/session_skills_r3.md`. `logbook/2026-09-13-skill-stack-r3-learner-single-site.md`. commits `b403964` (learner + memory), `c737ec9` (planner blend floor), `baab782` (skill path + learning-stack deletion). **apex ladder CLOSED 2026-09-16 (K=0.7 adopted; hand 1.05–1.22× (mean up to 1.13×) → 1.00–1.03×; ball apex 1.25× → 1.08×), entry `logbook/2026-09-16-apex-ladder-k07-ab-result.md`; `tests/hardware/session_skills_r3_apex_ladder.md` §6** ⚠ **Sittings 2026-09-17 (37 throws, 35 caught, gate NOT claimed): every catch mistimed because the tracker's Kalman landing ran +0.05..+0.13 s late and the learner converged onto it (true flight 30–90 ms short of the aim, hand late, HELD/EMPTY/HELD gaps, 10 `caught=False` for 2 drops) — FIXED: ballistic batch-fit landing (`tracking/flight_fit.py`, last-in-flight bias −5 ms), `CAUGHT_WINDOW_S` 0.70; guard chain (SETPOINT_STALE off a 64 ms Jetson hiccup at a displacement gate; MAX_DEVIATION ×2 from the un-parked hand on the recovery slew) — FIXED: rate-bound step gate, `/recover` parks the hand; `temp/learn/jugglebot` quarantined, NEXT sitting cold. `logbook/2026-09-17-late-catches-are-a-late-tracker.md`** **2026-09-18, for the next sitting: the learner's command and outcome are now the same physical quantity at a fixed horizon — a landing xy plus an APEX, all three off the converged fit (§ 0 item 5, § 2.5) — and the catch is aimed from that fit with the schedule as its prior (§ 2.7). Both changes remove the SAME bias in two places: the release-instant slip the 09-17 sitting measured at 0.019–0.137 s. The line to watch in the OUTCOME log is `seat=` — the contact phase, +0.104 s on every smooth catch and +0.015 s on the bouncers.** |
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
- **Carried from the R1 sitting (2026-09-11) — both RESOLVED at R2 (2026-09-12).**
  (a) Warm-start the chained launch solve: **measured, and not a cold-solve
  problem** — in a fresh process the cold penalty on the chained LAUNCH+STEADY is
  ~8 ms (385.6 vs 377.9 ms) and the QP warm start saves ~3 ms of a ~14 ms QP;
  93 % of the 380 ms was three `validate_cycle` passes on an 83-knot chain, and
  the sitting's 2.2 s was that under load. Resolved structurally: the vectorised
  gate (4.7 ms per 40-knot segment) and per-segment shapes; the QP `SolverState`
  is carried on the `Segment` (no cache). (b) `hand_stream_bench --trip-guard`:
  one opt-in that lifts `--gap-delta` to 2.6–3.5 rev and the belt to |gap|+1.0
  together, so the firmware trips first; runbook row 18 names it.
- **Owner decisions (2026-09-12), asked before code was written.**
  1. *Sizing.* No cell of this section's grid (apex ≥ 1.0 m, jerk ≤ 100k, hand
     3500) plans — the hand binds (§ 1.2). Adopted for the sim gate and the R4/R5
     session: **apex 0.9 m (t_f 0.857 s), separation 100 mm, hand acc 3500
     (unchanged), legs 300 / 5000 / 200 000; dwell 0.30 s, beat 0.578 s, transit
     0.278 s** (leg peaks 293 / 4315 / 179k; hand 3458 of 3500). The 1.0 m row
     needs the hand at its 3900 C-HAND-2 ceiling with 2 % margin and was declined.
     The "plannable flight ≥ 0.90 s" target of this section is superseded by
     0.857 s at the adopted apex; the release-height lever was probed (z-float
     does not move the peak hand acceleration; a higher release trades launch
     acceleration for post-release stopping room) and left at 860 mm.
  2. *`PlanCycle` scope.* R2 lands `InstallSegment` + `install_segment` as the
     skill stack's ONE path; `PlanCycle` and the coordinator's ring keep flying
     for the FSM until R4 hollows it (this section's original "deletes PlanCycle
     modes, ring machinery" contradicted § 4 R4 and § 6). The ring PRIMITIVES
     (`extend`, `_concat_plans`, `_gate_joined`, `_seam_check`) are reused by
     `splice_at` and are not dead; R4's list is refreshed below.
  3. *Parity.* The vectorised gate's verdict and reasons are identical to the
     scalar one and every peak within 1e-9 relative, on a battery that includes
     near-threshold plans (`tests/motion/test_validate_cycle_vectorised.py`).
  4. *Splice continuity.* The existing `_install_continuity_ok` at the live plan
     time plus `replan_tail`'s seam/detach rules, and ONE new refusal
     `SPLICE_TOO_LATE` (the splice knot must stay ahead of the knots the wire has
     read, checked against a clock read AFTER the solve). Lead = 4 knots.
  5. *Warm start* and *cold trip* as in the carried items above.
  6. *Segment shape* (asked after the sim gate failed at the adopted point): the
     § 2.2 amendment above — a same-site catch-and-throw is ONE segment
     (STEADY + settle tail). `sim/skills_gate.py` was built first on the split
     form and measured 0–2 of 20 catches; the offline twin
     (`tools/probes/skills_segment_sweep.py`) reproduced it without the plant.
  7. *Release seam.* A splice landing inside a release's detach cone snaps to
     the release knot and is seeded post-release (`release_state_at_knot`: the
     ring's own handoff), so the cone lives in the new window; a CATCH
     dispatched at `t_release − lead` lands there by construction. Without it
     `install_segment` could splice AT a release with `post_release=False` and
     re-solve the cone away (the off-axis shove `replan_tail` refuses).
  8. *Operating point re-confirmed* on the whole-window form through the real
     install chain (six throws, perfect tracker): 0.9 m / 100 mm / 0.30 s /
     hand 3500 / 300-5000-200k — peaks 266 / 4034 / 146k / 3399, plan ≤ 34 ms
     per install (`temp/probes/skills_segment_run2.md`); 150k also passes.
  9. *The sim gate's noise.* With the carried ball physically released, the
     model's default 2 % per-component release scatter (the Ball Butler's, a
     documented placeholder) drifts a landing ~155 mm and the 100 mm hop —
     which uses the whole 5000 mm/s² budget — has no reach margin: refused
     before motion, 0 drops. R2 certifies the software chain with an EXACT
     release and the 0.5 mm tracking noise; the separation-vs-scatter table
     (100 mm: 0 % only; 80 mm: 3/5 seeds at 0.5 %; 60 mm: 5/5 at 0.5 %, fails
     at 1 %) is the R4/R5 sizing input and the R3 sitting measures the
     machine's own 0.9 m scatter to pick from it.
- **Outcome (2026-09-12).** All builds landed; `sim/cycle_gate.py`,
  `sim/unified_gate.py` and their tests deleted; the sim gate MET at the
  operating point (20/20 catches × 5 seeds, 0 drops, plan wall min 10.4 /
  p50 28.3 / max 83.3 ms over 405 installs, 63.1 s); vectorised `validate_cycle` 4.7 ms per
  40-knot segment (21.8×) at 1e-9 parity over a 148-call battery; both
  R1-carried items closed; the splice leads measured (general 6 knots,
  handoff 8, budgets 75 / 125 ms proved by a modelled solve — **re-sized
  2026-09-18 to 9 / 11 knots, budgets 150 / 200 ms**, after the LOADED robot
  solved a CATCH in up to 134 ms and refused 16 of 23 attempts
  `SPLICE_TOO_LATE`; the modelled solve understated the loaded one 3-4×). **Hardware gate MET 2026-09-13** on the third no-motion sitting of
  `tests/hardware/session_skills_r2_plan_gate.md` (`40371fe`, bag `2026-09-13_12-35-28`,
  robot activated on a disarmed wire): rows 15 and 16 PASS on all five gates — worst
  solve 47.9 / 49.0 ms at load average 1.7–3.3, handoff margin 73–74 ms, zero late
  splices, emitter gap ≤ 28.5 ms, nothing moved; the non-gating margin row 17 (two
  extra busy cores) failed G1 (93.9 ms) and G3 (40.9 ms) with zero late splices. The
  first two sittings found and fixed a schedule bug that only appeared on the ROS
  clock (`4d49e04`) and driver defects (`40371fe`):
  `logbook/2026-09-13-skill-stack-r2-gate-sittings.md`. Entry:
  `logbook/2026-09-12-skill-stack-r2-skills-schedule-stream.md`.
  **R3 cleared to start.** **Caveat (2026-09-13):** the 20/20 sim-gate figure
  above relied in part on a drifting tracker anchor that suppressed
  catch-and-throw re-sends; the corrected anchor (landed at R3) surfaces a
  columns `LIMIT_JERK` refusal — filed and carried to R4, not re-opened at R3.

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
- **Carried from the R2 gate rehearsal (2026-09-13): session-start preconditions
  — port before the first powered skill.** The skill path has neither of the FSM's two session-start
  preconditions: no floor lift (`reload_coordinator_node._unified_floor_lift`)
  and no pre-level (`_unified_prelevel`). Measured offline: a THROW straight
  from the ACTIVATE park (hand 0 rev, cup 679.6 mm, 10 mm under the 689.6 mm
  planner floor) refuses `HAND_STROKE`; and a loaded levelling correction brings
  back R1's knot-0 tilt snap. `skills/start_columns` from a freshly activated
  robot will therefore refuse its first throw. The R2 gate works around both
  (a REST pre-position, no `level`). **RESOLVED at R3 (2026-09-13):** the
  schedule opens with a REST at the site (the floor lift, `FLOOR_LIFT_S =
  1.5 s`) and `skill_node` pre-levels before the first lift; both land in the
  executor/schedule units.
- **Carried from the R2 gate sittings (2026-09-13): margin, leg jerk and reach
  — before R3's first powered sitting.** (1) Re-run gate row 17 after the owner's background-load work: with
  two extra busy cores G1 reached 93.9 ms and G3 40.9 ms, and rows 15/16 cleared
  G1 by only 1–2 ms at an ordinary load, so R3's added load (a real ball, the
  tracker, the learner) can push solves over 50 ms; what protects the robot is
  the splice budget and the refuse-and-keep-the-rest-tail path, not G1. (2) Check
  R3's own cycle leg jerk offline before flying it: through the real install
  chain a same-site catch-and-throw peaks at 188 000 mm/s³ (40 mm/s, 1668 mm/s² —
  the jerk is the tilt from catch to throw, not translation) against 146 000 at the
  100 mm hop (`tools/probes/skills_segment_sweep.py --sep 1 40 100`, 2026-09-13),
  and the machine has flown 150 000; any R3 plan above that is a logged ramp under
  `leg-gain-tuning-methodology.md`. (3) At 100 mm every re-aim of a ±3 mm landing
  change was refused (143 of 143, all before motion) — decision 9's reach-margin
  finding, which R3's measured scatter resolves. **Status at R3 close
  (2026-09-13):** (2) RESOLVED — a one-ball single-site cycle commands 0 leg
  jerk in both the split and whole-window form (R2's 188k figure was a
  two-ball splice-seed artefact, not a single-site number). (3) RESOLVED for
  R3 by the single-site admissible box: (P1, P1) collapsed to (0, 0) and,
  after the pin-blend floor landed, regenerated at 150k as xy
  [−40, 40] × [−30, 30] mm, flight 0.750–0.857 s. (1) RE-CARRIED as a
  runsheet prerequisite for R3's sitting — the row 17 re-run needs the
  owner's background-load work, which did not happen this rung.
- **Sitting 1 (2026-09-13 evening, bag `2026-09-13_22-57-18`) — gate NOT
  reached; `logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md`.**
  Fixed 2026-09-14 (Jetson side, no flash): a plain THROW announces from its
  own event instant (five single throws were caught but ended `NO_LANDING`
  with a CONFIRMED track in `/balls`); an ended attempt with a release still
  streaming installs one `trajectory/hold` (latch L1: two hand strokes after
  `ABORTED_NO_RELEASE`, the second at the 3500 rev/s² ceiling, 47.8 A);
  `REJECTED_HAND_NOT_PARKED` was extended to off-park OR tracking error, on
  the opening REST too (latch L2: a REST from a hand at 9.4 rev into the
  bridge's 1 rev/s re-activation slew) — **that extension, and the whole
  refusal, are RETIRED 2026-09-16; see § 2.7's dated paragraph. The seed is
  the enforcement point now and DEACTIVATE/ACTIVATE is no longer a recovery
  for an off-park hand**; the guard line names the axis
  (`hand`, not `leg 6`); the BB reload refuses `REJECTED_RELOAD_RETIRED_R1`
  (its hand prime rode the R1-deleted stroke engine — § 1 item 7 stands;
  R4 re-cuts it as a CATCH skill). **Carried to sitting 2 / R4:** (e) the
  plant throws ~25 % fast — announced 4.17 m/s, measured apex 1.38 m
  (5.2 m/s), hand peak 161 rev/s vs 128 planned, flight 0.975/1.053 s vs
  0.857 — consistent with current saturation then position-loop catch-up;
  the (P1, P1) box's flight range 0.750–0.857 s cannot centre it. **Traced
  2026-09-14 (`logbook/2026-09-14-skill-stack-r3-apex-ladder-prep.md`):** the
  Platform-Teensy engine sent the hand ODrive an acceleration torque
  feedforward (~68 % of J·α) with every frame; the streamed hand lane sends
  zero (`leg_interp.cpp:1021`), so the velocity loop builds the torque from
  tracking error and overshoots after the ramp — the old engine at 177 rev/s
  and 2.8k rev/s² (2026-08-21) landed within 4 %. **Landed 2026-09-15, carried
  to sitting 2 (uncommitted, NOT flashed):** the hand C2 + torque-feedforward
  plumbing — phase-locked, knot-aligned scheduled frames (can-bridge FW 22,
  PROTOCOL_VERSION 8, `HAS_SCHED`/`t_origin_us`) and an in-firmware
  acceleration torque feedforward on the hand axis (`τ = K·J·2π·a_cmd`, gain
  from ROS param `hand_torque_ff_gain`). **Readback gate removed 2026-09-15**
  (`logbook/2026-09-15-hand-torque-ff-gate-removed.md`): ACTIVATE arms the
  hand through a path that never ran the fail-safe readback, so the wire
  gain was silently forced to 0 for a whole sitting; the owner confirmed the
  hand ODrive's `input_torque_scale=1000` and the gate was removed — K now
  goes straight to the wire, and the readback RPC is kept as a manual
  diagnostic. NEXT: the apex ladder
  `tests/hardware/session_skills_r3_apex_ladder.md` is now the FF **A/B**
  sitting itself (K=0 vs K=0.7, 0.5–0.9 m, hand limits unchanged) — it both
  flashes FW 22 and sets R3's gate apex; the offline model
  (`temp/probes/hand_cascade_ff/`) predicts the pre-registered "K=0.7 → ≤1.00"
  criterion is NOT met (1.02–1.05× at K=0.7–1.0), so a further owner decision
  on K follows the sitting rather than a single flash/no-flash call; boxes
  are now apex-scoped (a box swept for one apex was silently reused at any
  other); (f) FIRMWARE: the hand deviation residual
  is the raw plan against the encoder while the bridge slews the emitted
  command at ≤ 1 rev/s after a hand-lane activation (`leg_interp.cpp:777`
  vs `:963`) — the guard trips on a gap the bridge created; (g) the guard
  descent collapses the legs onto measured but leaves the hand command
  frozen; (h) the SEATED possession edge missed 3 of 8 real catches and
  both memory rows read `caught=False`; (i) the executor's fresh-origin
  gate is `idx == 0 and kind != CATCH` (a mid-schedule fresh REST would not
  be gated — none exists in today's schedules); (j) `catch_coordinator_node.py`
  keeps the same dead `smooth_move_hand` client (deleted with the FSM at R4);
  (k) the chained catch fails its 90 % margin for a ring of small landing
  offsets (±10–20 mm) at flights near 0.64 s while (0, 0) and larger
  offsets pass — follows flight time, not apex; pin-blend family; it is why
  the first `--single-apex` sweep's rectangles excluded the origin (fixed:
  a box now always contains the identity offset);
  (l) `/hand_telemetry` is stamped with the Jetson poll clock and its command
  echo is decimated 5:1 at the bridge — fitted delays and 10 ms
  accelerations from it are not measurements;
  (m) the closing REST after a spliced catch is intermittently refused
  `LIMIT_JERK` when dispatched early in its 40 Hz tick — a deterministic
  sweep refuses 24 % of tick phases at 0.9 m and 52 % at 0.6 m (peak leg jerk
  303 720 / 181 483 mm/s³), bit-identical at 9149819, so not new. It ends
  the attempt after the throw and catch, safely; fix it (minimum splice
  distance after the catch, or the REST scheduled a knot later) BEFORE R3's
  chained gate sitting.
- **Sittings 3–4 (2026-09-17), 37 throws, gate not claimed — one root cause
  for all three operator observations** (`logbook/2026-09-17-late-catches-are-a-late-tracker.md`):
  the tracker's Kalman-extrapolated landing ran +0.05..+0.13 s late against a
  ballistic fit of the raw marker, the freeze kept its most lagged sample, and
  the learner converged onto the biased number — true flights 30–90 ms
  shorter than the schedule aim, so the hand was late on every catch (ball
  met at the top of the stroke, cup then accelerating away at > g: the
  HELD/EMPTY/HELD gap, seats 0.26–0.61 s late, ten `caught=False` against two
  real drops). The physical flight repeats to ±0.03 s; the plant is 3–5 %
  fast at K = 0.7; the physical release lags the knot by 5–55 ms. Landed:
  gravity-fixed batch-fit landing for CONFIRMED balls (cup-floor height gate
  on both legs), `CAUGHT_WINDOW_S` 0.70 (reporting floor, not the fix), the
  setpoint-pump step gate as a RATE bound on hand and legs (non-absorbing;
  the SETPOINT_STALE latch was a 64 ms Jetson scheduling hole at the top of a
  throw), `/recover` parks the hand through the ACTIVATE(6) path (the two
  MAX_DEVIATION trips were the opening REST homing an 8.7 rev hand at
  5.4 rev/s while the firmware recovery slew caps it at 1 rev/s — carried item
  (f) above, now with its mechanism), and one WARN per heartbeat-dropout
  episode (the dropouts are real single-axis frame loss, owned by
  `plans/active/leg-bus-frame-drops.md`). **Carried:** the tracker aim at the
  CATCH dispatch instant (0.12 s after the throw) still runs on the KF
  fallback — **RESOLVED 2026-09-18** by the ordered aim (fit > schedule >
  filter): the KF can no longer aim a catch that has a schedule prior, and
  the converged fit refines the committed catch instead (§ 2.7's 2026-09-18
  paragraph); the KF's `predict()` integrates its
  nominal 5 ms against 5.2 ms measured frames (published position/velocity
  only); `MAX_LEAD_HAND_REV` 2.0 vs `MAX_DEVIATION_HAND_REV` 2.5 (owner
  sign-off); the origin of the 55–64 ms Jetson stall (the 10 Hz bridge diag
  callback is the suspect); `trajectory_node`'s emitter backstop has the same
  displacement-vs-rate shape; a hand-lane rate feasibility check at
  `install_segment`; `SPLICE_TOO_LATE` on the 5-throw chain's second catch —
  **RESOLVED 2026-09-18**: the budget, not the solve, was wrong (below).
- **The 2026-09-18 sitting (`temp/logs/launch_r2gate_20260918_1325.log`, bag
  `2026-09-18_13-25-15`) — three fixes, all landed the same day.**
  1. *The hand park fired into the fault tick.* `/clear_errors` → `CLEAR_ERRORS
     fired` → the park's `ACTIVATE(6)` rejected `ERR_BUS_DOWN`
     ("fault_state=MAX_DEVIATION is currently latched") → `HAND NOT PARKED`,
     and the NEXT log line was `Teensy guard fault cleared`. CLEAR_ERRORS is
     acked by the LINK; the latch is released by the firmware's 10 Hz fault
     task. The park now WAITS for the cached `fault_state` to read NONE
     (bounded 1.0 s) and refuses naming the latch if it does not —
     `teensy_bridge_node._wait_for_guard_clear`, inside the one park op, so
     both recovery paths and the new `/park_hand` get it.
  2. *An opening REST homed the hand from the top of the stroke* (the three
     MAX_DEVIATION latches). **CONTRACT, as it stands after the same evening's
     second pass (C-HAND-4):** *a streamed lane may home the hand ONLY inside
     the firmware's resume and follow envelope* — continuous from the knot the
     hand group is holding, peak ≤ `JB_OP_GENTLE_MOVE_VEL_LIMIT_RPS`
     (2.5 rev/s) and ≤ `RECOVER_SLEW_ACCEL_RPS2` (5 rev/s²); outside a
     disarm/arm edge nothing else may move the hand. There is ONE home
     (`sites.REST_HAND_REV` = 0.3071 rev, where every REST leaves the hand) and
     the opening REST's PERIOD is SIZED to the measured displacement
     (`schedule.floor_lift_s`: 9.63 rev ⇒ 7.0 s, realised peaks 2.01 rev/s and
     1.14 rev/s²), so a displaced hand is HOMED, never refused — the owner's
     instruction, verbatim: *"if the hand isn't where it needs to be at the
     start, it should smooth-move down to the start position before beginning
     the cycle"*. **The day's FIRST answer is DELETED after one day:** a
     blocking `/park_hand` before the pre-level plus a park-band precondition
     in `skill_node` measured the start against the ACTIVATE park (0.0 rev)
     while a schedule's REST correctly leaves the hand at 0.3071 rev, so it
     refused every attempt after the first (`hand park REFUSED — the hand is at
     +0.3063 rev … but the wire is ARMED`). `/park_hand` SURVIVES as the
     operator/recovery op (its armed-wire refusal is still right FOR THAT OP:
     it moves the axis out from under a live lane). **The carried "re-seed the
     streamed hand lane onto the park" item is RESOLVED, not deferred** — the
     lane resumes from its HELD knot, which is what the sizing is for, so
     nothing needs to pin it to the encoder. Fail-closed half: the firmware's
     `sched_refused` counter now ENDS the attempt (`HAND_LANE_REFUSED`) and
     installs the hand-less hold, so a lane refused anyway stops walking
     before the deviation can grow. Latch 1's firmware mechanism, established from the
     code: the hand-less hold let the scheduled hand group's cover expire into
     a C2 stop, the REST was then refused as a discontinuous resume
     (`leg_interp.cpp:514-520`, `sched_refused`), the group kept HOLDING — the
     hand did not move at all — and the guard deliberately measured the
     REFUSED incoming command against the encoder (`leg_interp.cpp:1226-1228`,
     FW 22), so it tripped on a lane nobody was following. `sched_refused` /
     `sched_stops` now surface on `/link_status` (they were on the wire and
     unread). Latches 2-3 are the plain `RECOVER_SLEW_VEL_RPS` = 1 rev/s case.
     No firmware change.
  3. *The catch splice budget was smaller than the measured solve.* CATCH
     `install_segment` plan times under sitting load, n=51: min 40.1 / p50 76.4
     / p90 111.0 / p95 113.4 / max 134.2 ms against a budget of 0.083-0.100 s.
     `schedule.SOLVE_BUDGET_KNOTS` = 6 (0.150 s = p95 + one knot, and clears
     the max by 16 ms) is now the one number both leads derive from. Cost:
     every dispatch splices 75 ms further ahead and `CATCH_FREEZE_S` grows with
     it, so the re-aim window shrinks by the same 75 ms.
- **Owner decisions (2026-09-13).**
  1. *Cycle.* Chained single site: apex 0.9 m, dwell 0.30 s, site P1
     (−50, 0), legs 300 / 5000 / 150 000, hand 3500.
  2. *Learner.* k = 16, k_min = 2, h_x = 0.01 m, h_y = (0.05, 0.05, 0.2),
     γ = 1e-2, η = 0.2 (§ 2.5); a new (P1, P1) admissible box swept at 150k.
  3. *Ladder.* Pure in the executor via an observation callable; `skill_node`
     pre-levels (`go_to_pose` identity); the schedule opens with a REST at
     the site (the floor lift).
  4. *Memory.* `temp/learn/<plant_id>/memory.csv`, `plant_id` a `skill_node`
     param defaulting to `'jugglebot'`.
  5. *Outcome.* `ball_tracker_node` landing_z → 830 (`sites.CATCH_CUP_Z_MM`);
     y = the native `/balls` prediction; flight = crossing − the scheduled
     release. (The FSM's catch plane moves too — accepted.)
  6. *Chained lag* accepted: a catch-with-throw's `then_throw` command is
     computed once at CATCH dispatch; re-sends reuse it.
  7. *Band.* Sim 20 mm / 20 ms (exact release + 0.5 mm tracking noise);
     hardware 30 mm / 20 ms; monotone = median + 1.3·MAD.
  8. *Deletion* after the chained sim passes: live `toss_record` symbols move
     to `ball_possession.py`; delete the miner, `mocap_parity_bias`,
     `seat_edge_decomposition`, `toss_cal_analyse`; `TOTAL_MAX_RAD` becomes an
     FSM constant.
  9. *Single-site CATCH* with no landing at dispatch waits for the first
     landing; `NO_LANDING` fires only at the deadline `t_land − 0.278 − lead`.
     **Superseded for catch-with-throw by decision 12.**
  10. *Release lag* (156–371 ms commanded→physical in old bags, unexplained):
      the sitting measures it; the flight definition is unchanged. Risk: if
      real, the flight band fails on hardware and the box's 0.750 s floor
      blocks it.
  11. *Aim-box collapse.* Investigate the planner first (Opus unit R3-i).
      Pre-registered fallback if not traced in one unit: launch THROWs fly
      identity; the learner commands only catch-carried throws (box
      ±40/±30 mm chained).
  12. *Catch-with-throw installs at release* (scheduled dispatch, the splice
      snap), aimed at the predicted landing (`y_d`, from the ball's own
      previous release, ballistic arrival), refined by tracker re-sends;
      standalone catches keep wait-for-landing; a cold start is single-throw
      attempts. **Supersedes decision 9 for catch-with-throw.**

  Assumption: seat offset x = 0 at R3. The columns opening REST is
  re-carried to R4.
- **Outcome (2026-09-13).** `learner.py`, `memory.py`, the single-site
  chained schedule (`compile_self_toss` + the opening REST), outcome capture,
  the precondition ladder (pre-level, floor lift, `NO_ADMISSIBLE_COMMAND`) and
  a working `skill_node` landed. **Sim criterion MET:**
  `python sim/skills_gate.py --learn --policy A --seeds 0-4` ×2
  (`skills_gate_learn_A_run{1,2}.json`, 204.7 / 205.9 s) and `--policy B
  --seeds 0-4` (`skills_gate_learn_B_run1.json`, 191.4 s), all 2026-09-13:
  every seed PASS in-band by throw 3 (policy A) / 5 (policy B), monotone
  true, 0 drops, and the two policy-A runs bit-identical. Defects found and
  fixed this rung: the R2 `skill_node`'s single-threaded spin + blocking
  install wait (every live install would have timed out); CATCH aimed at the
  stale 809.08 mm tracker plane instead of 830; the learner's underflowed
  weight sum producing a NaN command (now a raised, refused skill);
  `compile_self_toss` dispatching a catch before its own release; the 1.0 s
  floor lift refusing `LIMIT_JERK` from the centred park (raised to 1.5 s);
  the admissible sweep never gating the launch throw that carries the
  command; and the columns sim gate's drifting tracker anchor that had
  partly propped up R2's 20/20 result (carried to R4 below). Hardware gate
  outstanding: the operator's sitting, `tests/hardware/session_skills_r3.md`.
  **R4 is NOT cleared.**
- **Delete.** `motion/toss_ilc.py`, `toss_trim.py`, `motion/toss_cal.py`,
  `tests/hardware/ilc_fit*.py`, `toss_fit_lib.py`, `toss_cal_*.py`, the
  `toss_ilc_enabled` flag, and the named probes that mined them (the miner,
  `mocap_parity_bias`, `seat_edge_decomposition`, `toss_cal_analyse`) — done,
  R3. `config/toss_ilc.yaml` and `config/toss_calibration.yaml` never existed
  (decision 8; grep found no such files). `toss_record.py`'s live symbols
  moved to `ball_possession.py` rather than being deleted — still in use
  outside the ILC stack; `TOTAL_MAX_RAD` becomes an FSM constant.
  Grep-to-zero counts (2026-09-13, `git grep` for the learning-stack names): 2535
  before → 698 after, 91 outside `*.md`, each a dated retirement note, provenance
  prose, or an unrelated `toss_*` name in a live FSM module
  (`logbook/2026-09-13-skill-stack-r3-learner-single-site.md` § Fix).
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
- **Carried from R3 (2026-09-13).** (a) Columns jerk creep exposed once the
  sim tracker anchor was fixed: with the correct anchor (no re-sends propping
  up R2's numbers), the columns gate at seed 0 fails `LIMIT_JERK` at install
  15, 204 534 > 200 000 mm/s³ — R2's 20/20 sim gate was partly propped up by
  the drifting-anchor bug (unit m, `probe_columns_resend_trace/`,
  2026-09-13). (b) Banking saturation when the hand's deceleration exceeds g
  (≈12° bank), the likely source of the same-site 188 000 mm/s³ figure
  carried from R2 (unit i, traced 2026-09-13). (c) The columns opening REST
  (R3 assumed seat offset x = 0 and left the columns REST for R4) — and with
  the 2026-09-18 homing sizing this is now also the ONE path that cannot home
  a displaced hand: `compile_columns`'s first skill is a CATCH that splices
  onto the live plan, so `skill_node._hand_home_error` REFUSES a columns start
  whose hand is more than `schedule.HOME_BAND_REV` (0.10 rev) from
  `sites.REST_HAND_REV`, naming the self-toss path that does home it. Giving
  columns its own opening REST closes the refusal. (d)
  `admissible.gate_hash` covers `feasibility.py` and `segments.py` only — a
  `cup_realize.py` edit leaves a stale box undetected (filed as a follow-up
  during R3).
- **Delete** (tag `fsm-final` first): `toss_sequencer.py`, `toss_session.py`,
  `reload_sequencer.py`, `catch_coordinator.py` + node, `catch_reach.py`, the
  ring POLICY in `unified_cycle.py` (`replan_tail`, `latest_supersede_time_s`,
  `is_release_terminal`'s deadline use, `plan_cycle`'s MODE plumbing — NOT
  `extend` / `_concat_plans` / `_gate_joined` / `_seam_check`, which
  `splice_at` reuses since R2), `PlanCycle.srv` and `_svc_plan_cycle`, the
  `unified_cycle_enabled` and `toss_*` flags, and the FSM tests.
  `reload_coordinator_node.py`'s remaining duties fold into `skill_node.py`
  (landed at R2 as the schedule shell).
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
