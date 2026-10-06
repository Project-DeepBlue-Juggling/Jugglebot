---
title: Two-ball skill stack — schedule-driven throw/catch skills, one hand master, memory-based learning
created: 2026-09-09
status: active
owner: Harrison
last_updated: 2026-09-30
related_logbook:
  - 2026-09-09-two-ball-skill-stack-kickoff.md
  - 2026-09-11-skill-stack-r1-one-hand-master.md
  - 2026-09-11-skill-stack-r1-sitting-prelevel.md
  - 2026-09-12-skill-stack-r2-skills-schedule-stream.md
  - 2026-09-13-skill-stack-r2-plan-gate-runsheet.md
  - 2026-09-13-skill-stack-r2-gate-sittings.md
  - 2026-09-13-skill-stack-r3-learner-single-site.md
  - 2026-09-29-skill-stack-r4-gate-met.md
  - 2026-09-30-skill-stack-r5-columns-bb-start.md
  - 2026-09-30-skill-stack-r5-sitting-1.md
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
- **Ball names (owner, 2026-10-04).** Operator-facing text says **Ball 1** for the ball in
  Jugglebot's hand at the start (schedule id 0; "ball A" in code prose) and **Ball 2** for the
  one Ball Butler feeds or that waits at the second site (id 1; "ball B") — numbered by the
  order Jugglebot's hand first throws them (`schedule.ball_label`). Sites keep their names,
  so in fed columns Ball 1 holds at P2 and Ball 2 lands at P1; the crossover is deliberate.

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
- **Carried in from `plans/archived/cup-contact-contract.md` (closed out 2026-09-23,
  its four logbook entries 2026-09-20…23 are the provenance):**
  - Cup-contact contract live in the planner: banking only under seating force,
    `CUP_CONTACT_ACC` floor −0.7 g over the contact window, **τ = 0.125 s** — flown
    2026-09-22, 87/87 caught, seat median +0.089/+0.079 s at 0.6/0.9 m.
  - **`learner_lateral_authority_mm` launch default 20 mm** (owner, 2026-09-23: 40 mm
    dropped balls on the 0.9 m 25-throw chains; the admissible box still admits ±40).
    The lateral learner works: median y miss pulled through zero at both apexes.
  - **Tracker landings are corrected by the measured mocap-to-schedule offset**
    (`skill_node._on_balls`, adopted at every frame check under a 25 mm sanity bound +
    a 2 mm Platform-body stability gate). The offset is a 0.77° lever arm between the
    QTM frame (≈ gravity-level) and the machine's base plane (z-sweep 2026-09-23:
    −0.0135 mm per mm of height); the subtraction is exact at the site height and
    within 0.7 mm over the catch's travel. **Root-cause remedy: shim the base level**
    (~5.5 mm over the 410 mm radius), then re-level and re-align. R4's two sites share
    one height, so one measured offset serves both. Columns apex band collapsed to
    0.900 (2026-09-21).
  - Watch items R4 inherits: (1) the hand recovery park's unresolved upper-stop push
    (2026-09-23 — instruments now stay live through a park; recover a high hand with a
    hand on the E-stop; re-home the hand before the next sitting); (2) hand
    `MAX_DEVIATION` latches from stale-encoder bursts at the throw onset (three now;
    `plans/active/leg-bus-frame-drops.md`); (3) catch re-sends whose splice seed lands
    inside the contact window are refused `CUP_CONTACT_ACC` after the solve — skip them
    before it (**CLOSED at R4, 2026-09-23: declined before the solve, `executor._resend_declined`**); (4) 16 of 23 re-send refusals on 2026-09-22 were `LIMIT_JERK`;
    (5) **the seam tilt-RATE pin** — `start_tilt` pins a splice seam's tilt value but not
    its rate, masked today and on the R4 re-aim critical path (**MEASURED UNNECESSARY at R4, 2026-09-23: 0 of 7 hop seams over the bound, 0.05×; the real defect one layer down — the join kept a truncated head's stale seam VELOCITY — is fixed in `_concat_plans`**)
    (`logbook/2026-09-20-cup-contact-contract-implemented.md` Discussion, "Why the seam
    tilt-RATE pin was NOT built"); (6) `test_the_qp_and_the_gate_stay_within_an_order_of_magnitude`
    exceeds its 10× wall-clock bar in ~2 of 10 quiet-box runs — a re-derived bar or a
    median-of-N is an owner call (same entry, Diagnosis #4). **Added 2026-09-28 (sitting 1):** (7) the
    post-release hold (50 ms) is TRANSLATION-only — pinning the attitude over it refuses on the
    250 mm hop at this operating point (`LIMIT_ACC` 5538–21 711 > 5000 for every span; a
    flat-then-slew tilt schedule by construction, or a longer tail, is the owner call); residual
    centroid speed over the held knots: hop 2.75 mm/s at the shipped 2 knots (13.36 at the rejected 3-knot cut), was 68; (8) the ball separates 20–40 ms after the
    planned release on every throw (free flight extrapolated to t_rel sits 48–172 mm below the
    release point) — a characterisation item the learner absorbs vertically and the hold covers
    laterally; (9) the carried item (1) hand recovery park stays open, and the hop box's x-collapse
    toward the far site (+2 mm) means the hop's residual lateral error is the planner's to remove,
    not the learner's.

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
   **Amended 2026-10-05 (catch plane 830 → 930, above the 860 release):** the
   apex is the FLIGHT-EQUIVALENT apex — the `u` whose `flight_s(u)` the
   planner would have used to put the observed catch-plane crossing speed
   there, given the release-to-catch rise (`schedule.apex_from_crossing`).
   It equals `v_z²/2g` when the two planes coincide; with them apart, the old
   reading made a perfect plant read `y ≠ u` (+14.5 mm at a −30 mm rise,
   −35 mm at +70 mm). Rows recorded before the change were migrated once by
   the same formula (`tools/migrate_memory_catch_plane.py`). **The owner
   returned the catch plane to 830 on 2026-10-06** (contact-speed regression
   from catch high; `logbook/2026-10-06-skill-stack-r5-catch-plane-830-and-
   leg-current-15a.md`) — the FLIGHT-EQUIVALENT apex definition stays
   necessary regardless, since release (860) and catch (830) still do not
   coincide, and no further memory migration was needed (rows are
   release-relative since this amendment, not plane-relative).
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
  from the fit's crossing speed, `h = v_z²/2g` (2026-09-18), rise-aware since
  2026-10-05 (§ 0 item 5, `schedule.apex_from_crossing`). SI
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

**Revised at R5 sitting 3 (2026-10-04) — steps 1 and 3 above are superseded.** The
joint (state, outcome) neighbour metric of step 1 picks, among rows that share a
state, the throws that happened to land nearest the target; their mean landing is
the target by construction, so the fitted intercept d is always about zero and the
command freezes wherever history left ū (sitting 3: u_y = +4 mm against a plant
that lands +11 mm in y beyond the command; the R4 gate's −5 mm was the same law
with a luckier history). The outcome bandwidth h_y now only GATES which rows
count (`support_d2`, rows inside the state/outcome support), the neighbourhood is
the k most RECENT rows inside that gate (memory is appended oldest-first, so row
order is part of the law), ū keeps the paper's weights, and the forward fit of
step 3 uses state-only weights. A query with no row inside the gate returns the
identity prior instead of raising; a non-finite result still raises by name. On
the real memory this moves the self-toss command from (−15.9, +4.1) mm to
(−17.5, −4.5) mm, and a well-spread neighbourhood still identifies a true slope
(88/79/76 % of the way to it in `test_learner.py`, against 55/58/63 % before).
`logbook/2026-10-04-skill-stack-r5-sitting-3.md`.

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
catch plane moves with it, decision 5 — then from 830 to 930 mm on
2026-10-05, the owner's "catch high": the empty hand waits near the top of
its stroke — and back to 830 mm on 2026-10-06, the owner returning it to
restore the original contact speed; see
`logbook/2026-10-06-skill-stack-r5-catch-plane-830-and-leg-current-15a.md`).
The CATCH terminal is the latest
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
0.979–2.985 as written. ⚠ open: the tracker's in-flight estimate ran ~one beat **CLOSED same day: it was the host, not the tracker — `_tracker` served the NEXT announced id; correlation is now per RELEASE (`ball_possession.flight_in_progress`, entry `2026-09-16-tracker-correlation-follows-the-flight-in-progress`), replay 43/43 rows admitted. Interim guard the same day: `learner_lateral_authority_mm` = 0 (the learner corrects flight only) until the banking small-offset defect (`logbook/2026-09-16-banking-saturates-on-small-lateral-offsets.md`: cup_realize `tilt_to_receive` saturates to its 12° clamp for ANY nonzero lateral residual during the pre-catch dive) is fixed at its root. **UNPINNED 2026-09-21**: the cup-contact contract (`plans/archived/cup-contact-contract.md`) fixed banking at its root (amplitude-aware, § 2) and the launch default moved 0 → 40.**
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
| R3 | Learner + single site | `learner.py`, `memory.py`, outcome capture | ILC/trim/cal/record stack, `toss_ilc_enabled` | in-band within 5 throws from cold, sim and hardware; 10 consecutive catches | ✅ **DONE 2026-09-23** — hardware gate met under the cup-contact contract: 26/26 (2026-09-18 16:16), 87/87 (2026-09-22, Block A) and 78/80 with the lateral learner live (2026-09-23, Block B); the learner in band within five throws at both apexes; see the § 0 carry-ins. History: 🟡 **SIM MET, HARDWARE OUTSTANDING (2026-09-13)** — landed: the learner + memory, the single-site chained schedule, outcome capture, the precondition ladder (pre-level, floor lift) and a working `skill_node` shell. **Sim criterion MET 2026-09-13**: policies A and B, seeds 0–4, in-band by throw 3 (A) / 5 (B), monotone, 0 drops, repeat runs bit-identical. ⚠ **Sim criterion RE-OPENED 2026-09-14 in xy only**: the dense apex-scoped re-sweep's (P1, P1) 0.9 m box admits x 0…+40 mm, y 0 (the old ±40 × ±30 mm box claimed offsets the chained catch fails at 90 % margin at 0.77/0.81 s flights, item (k)); on it policies A and B, seeds 0–4, land flight in band by throw 3 / 5 with 0 drops but never enter the xy band (the sim's +8.5 mrad aim error is +y) — restoring xy authority needs item (k) resolved. **Item (k) CLOSED 2026-09-21** (cup-contact contract box re-sweep + `sim/skills_gate.py`'s `_FIT_MIN_SAMPLES=12` fix mirroring the robot's `flight_fit` admission rule): the xy-band xfail is now a plain passing test, 25/25 makes across seeds 0–4. **Sitting 1 (2026-09-13 evening) did not reach the gate**: 5/5 single throws caught but untracked (a plain THROW never announced — fixed), 2–3/5 chained, two hand-axis `MAX_DEVIATION` latches (an ended attempt's plan kept throwing; an opening REST from an un-parked hand), the BB reload retired at R1 — five Jetson-side fixes landed 2026-09-14; ⚠ **the plant throws ~25 % fast (apex 1.38 m for 0.9 m) and the learner's box cannot reach it — owner decision on the hand acceleration ceiling before sitting 2** (`logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md`). Outstanding: sitting 2, `tests/hardware/session_skills_r3.md`. `logbook/2026-09-13-skill-stack-r3-learner-single-site.md`. commits `b403964` (learner + memory), `c737ec9` (planner blend floor), `baab782` (skill path + learning-stack deletion). **apex ladder CLOSED 2026-09-16 (K=0.7 adopted; hand 1.05–1.22× (mean up to 1.13×) → 1.00–1.03×; ball apex 1.25× → 1.08×), entry `logbook/2026-09-16-apex-ladder-k07-ab-result.md`; `tests/hardware/session_skills_r3_apex_ladder.md` §6** ⚠ **Sittings 2026-09-17 (37 throws, 35 caught, gate NOT claimed): every catch mistimed because the tracker's Kalman landing ran +0.05..+0.13 s late and the learner converged onto it (true flight 30–90 ms short of the aim, hand late, HELD/EMPTY/HELD gaps, 10 `caught=False` for 2 drops) — FIXED: ballistic batch-fit landing (`tracking/flight_fit.py`, last-in-flight bias −5 ms), `CAUGHT_WINDOW_S` 0.70; guard chain (SETPOINT_STALE off a 64 ms Jetson hiccup at a displacement gate; MAX_DEVIATION ×2 from the un-parked hand on the recovery slew) — FIXED: rate-bound step gate, `/recover` parks the hand; `temp/learn/jugglebot` quarantined, NEXT sitting cold. `logbook/2026-09-17-late-catches-are-a-late-tracker.md`** **2026-09-18, for the next sitting: the learner's command and outcome are now the same physical quantity at a fixed horizon — a landing xy plus an APEX, all three off the converged fit (§ 0 item 5, § 2.5) — and the catch is aimed from that fit with the schedule as its prior (§ 2.7). Both changes remove the SAME bias in two places: the release-instant slip the 09-17 sitting measured at 0.019–0.137 s. The line to watch in the OUTCOME log is `seat=` — the contact phase, +0.104 s on every smooth catch and +0.015 s on the bouncers.** |
| R4 | Two sites, one ball, BB reset | one-ball HOP schedule (`OneBallPattern`/`compile_one_ball`), the reload as REST → held-axis CATCH → REST → pattern anchored on the BB announcement (`compile_reload`, `CatchEvent.axis`, `CycleGoals.hold_tilt`/`rest_tilt`), boxes keyed by pattern + (release, target) site with xy stamps and the 250 mm hop swept, `Juggle.action` + the GUI relay, the splice seam-velocity fix, fresh-origin RESTs | FSM stack (tag `fsm-final` = 1e7f2d9): coordinators, sequencers, `catch_reach`, the ring half of `trajectory_node`/`unified_cycle`, `PlanCycle`, the Toss/TossContinuous/Reload actions, the old sim gates, 14 config keys | 10 consecutive alternating catches; BB reload → catch → throw chain | ✅ **DONE 2026-09-29 — hardware gate MET at the fourth sitting** (25 consecutive alternating hop catches, 14 by the node's strict verdict; 3/3 one-button BB reloads, each followed by 4 catches; `logbook/2026-09-29-skill-stack-r4-gate-met.md`). Sittings 1–3 NOT MET (2026-09-27/28; see the R4 Outcome) — sim gate MET (hop 25/25 makes × 5 seeds, 0 drops, bit-identical ×2; reload trial 5/5 seeds under the live tracker aim); FSM deleted 2026-09-24 under `fsm-final`; runsheet `tests/hardware/session_skills_r4.md`; ⚠ hop box apex 0.85–0.90 m only, 250 mm re-aims refuse inside ~0.5 s of touch-down (see the R4 Outcome). Entries `logbook/2026-09-23-skill-stack-r4-hop-reload-planner.md`, `logbook/2026-09-24-skill-stack-r4-fsm-deletion.md` |
| R5 | Two-ball columns | Start/Stop phases, limits ramp as sized at R2 | — | five consecutive cycles, then 30 catches; learning curve logged | 🟡 **SOFTWARE LANDED 2026-09-30, HARDWARE GATE OUTSTANDING (2026-09-30 evening)** — sitting 1 flew the 4°/0° cup test (MET at both caps) and the fused reload; Block C (human lob) never claimed a feed and is RETIRED; four evening fix units (receive-level feed catch, pre-throw columns-feed check, landing-timing/lateral-bias corrections, BB FW 5 + retry-once) land the 0° reload gate and a BB-fed columns block as the next gate, superseding sitting 1's human-lob Block C in `tests/hardware/session_skills_r5.md`. Sim columns learner 30/30 × 5 seeds at 0.90 m. 🔴 **Sitting 2 flown 2026-10-02** (`tests/hardware/session_skills_r5_sitting2.md`, superseded by a sitting-3 sheet once fixes land): 0° gate FAILED on timing (10/10 caught but +55.3 ms landing, 1/8 smooth seats); BB-fed columns 0/7 cycles — the `receive_tilt` fix never reached the `InstallSegment` wire, so feeds collided with ball A's own throw in flight (mocap-confirmed, 4/7), and 3/7 refused `ORIGIN_TOO_LATE`; the owner's site-swap fix for the collision costs `LIMIT_ACC` 101% undisplaced. 🟡 **Six fix units landed 2026-10-02 afternoon (not yet flown)** — the wire fix, the site swap + feed-aim offset, the hand cap raised to 3900 rev/s² with a box re-sweep, the `ORIGIN_TOO_LATE` solve budget, a `STALE_STATE` seed fix, and the sim stream-loop mirror — **the hardware gate now moves to sitting 3**, `tests/hardware/session_skills_r5_sitting3.md`. 🔴 **Sitting 3 flown 2026-10-02 evening**: the swap worked, but 21/21 fed attempts died `WINDOW_TOO_SHORT` at the fourth skill (the tracker correlation handed each ball the other's flight), throws wide from the ball in the cup + a frozen learner intercept + a 0.3° level lean (not platform tilt), 26 mm of column clearance, a hand pinched on the funnel ring at 50 A. 🟡 **Seven fix units landed 2026-10-04 (not yet flown)** — association contract + faithful sim, learner law revised, level trim, `columns_1ball`, hand-jam recovery (FW 26 flashed 16:34), geometry apex 0.95 / dwell 0.27 / sep 125 / vel 350 with the box re-swept — **the hardware gate moves to sitting 4**, `tests/hardware/session_skills_r5_sitting4.md`. See the R5 Outcome / owner-decisions block for detail (`logbook/2026-09-30-skill-stack-r5-columns-bb-start.md`, `logbook/2026-09-30-skill-stack-r5-sitting-1.md`, `logbook/2026-10-02-skill-stack-r5-sitting-2.md`) |
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
- **Next unit (owner, 2026-09-18): `plans/archived/cup-contact-contract.md`** — the banking/cup-contact contract that unpins the learner's lateral channel and the tracker aim (prerequisite: the mocap base alignment, owner). R4 follows it; `leg-bus-frame-drops` after R4 unless a sitting says otherwise.
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

- **Owner decisions (2026-09-23), asked before code was written.** D1 the
  pattern is a CROSS-SITE HOP — one ball released at P1 landing at P2, the cup
  coasting under it (117 mm/s lateral at 100 mm, 292 at 250), caught at P2
  carrying the throw back; D2 sweep and fly 250 mm first; D3 `Juggle.action`
  served by `skill_node` with the GUI relay in `orchestrator_node` (rosbridge on
  Foxy has no action client), the Trigger start services deleted; D4 the reload
  is the FSM's proven choreography as skills — the owner's pushback *"if the
  Platform is tilted to face the oncoming ball, the hand only needs to move
  along its usual linear axis to match the ball's vertical and lateral
  velocities"* was load-bearing: my first framing (match the lateral arrival by
  translating the platform) was wrong and three probes showed the unified
  planner could express none of pre-tilt / hold / one-axis stroke; D5 delete the
  FSM before the sitting; D6 the contact-window re-send skip in scope, the
  columns opening REST left to R5.
- **Outcome (software, 2026-09-24).** Nine commits on `skill-stack` from
  `2f4d94b` (planner) to the deletion series, two logbook entries. Landed: the
  hop schedule (one site bit-identical to the retired `compile_self_toss`);
  boxes keyed by pattern + (release, target) site with site-xy stamps, the gate
  hash over six planner files (carried item (d) closed), the 250 mm hop swept
  (x −20…+6 mm toward P2, y ±20, apex 0.850–0.900 at the R4 geometry; a determinism pair 2026-09-23 /
  09-24 with every box and all 16 471 row verdicts identical); the held-axis
  receive catch and attitude-bearing REST in the planner (leg velocity 3.1 mm/s
  across 18–40° arrivals against `LIMIT_VEL` at 620–800 for every earlier
  framing); the reload as skills anchored on the announcement with
  `ABORTED_NO_ANNOUNCEMENT` fail-closed and `REJECTED_BB` verbatim;
  `Juggle.action`; the FSM gone under `fsm-final`. Two seam defects fixed at the
  root: the splice join kept a truncated head's stale seam velocity (2.916 →
  1.422 rad/s², gate window 123 k → 86 k mm/s³ at a 250 mm hop seam — the seam
  tilt-RATE pin, watch item (5), measured unnecessary at 0.05× the bound and
  NOT built), and every REST after a catch is a fresh origin two knots past the
  tail (the closing REST refused `LIMIT_JERK 156 495` at 250 mm; the decay REST
  refused `CUP_CONTACT_ACC` on 5/5 sim seeds — both gone). **Sim criteria MET:**
  `python sim/skills_gate.py --learn --pattern hop --seeds 0 1 2 3 4` ×2, run
  2026-09-24: 25/25 makes every seed, 0 drops, 0 refusals, bit-identical, plan
  p50 ~52 / p95 58–73 / max 62–116 ms on a loaded box; `--reload --seeds 0 1 2
  3 4` under the live tracker aim, 2026-09-24 00:24: PASS 5/5 (2 makes, 0
  drops, 0 pump rejects each). Carried items (a)–(d): (d) closed; (a) the
  columns jerk creep and (c) the columns opening REST move to R5 with the
  two-ball start (owner, D6); (b) closed by the cup-contact contract's
  amplitude-aware banking (2026-09-21). **Carried to the first sitting / R5:**
  (1) the hop box: at the R4 geometry it admitted apex 0.85–0.90 m only (no
  flight-fraction ladder for the hop) — a plant more than ~5 % fast in apex
  plateaus out of band (the sim's +11 % model does); **re-swept 2026-09-27 under the
  kinematic-calibration geometry** (`plans/active/kinematic-calibration.md`, gate hash
  `7f76f68d4943` (superseded 2026-09-28 by **`3fda47b2ad5b`**: the 50 ms post-release hold re-swept the boxes twice, bit-identical; the hop's x bound AWAY from the far site halved — P1→P2 x [−10, +2] mm was [−20, +2], P2→P1 x [−1, +10] was [−0.5, +20]; y ±20 mm, apex 0.85–0.95 m, and every self-toss/columns box unchanged; superseded again 2026-09-28 evening by **`bf095653d422`**: the hold now keeps the launch line, two sweeps bit-identical, only P2→P1 x moved to [−0.5, +10]) after the 2026-09-27 FW 24 stroke-clamp commit `043158e`; bounds unchanged from `3059cc1`): the apex band opened to 0.85–0.95 but the x bound TOWARD the
  far site collapsed — P1→P2 x [−20, +2] mm, P2→P1 x [−0.5, +20] mm (y ±20 kept) — so
  the learner has essentially no lateral authority in the hop's own direction; the
  sitting's measured hop ratio and lateral bias at K = 0.7 decide whether the hop
  sweep needs a finer grid at the +x edge (2 mm steps stop at ±10, then 20) or a
  different separation; (2) at 250 mm a tracker
  re-send inside ~0.5 s of touch-down refuses `LIMIT_JERK` at 2.3–3.6× the
  limit (the TAIL, not the seam; within 25 % of the cap at a 0.70 s lead), so
  late re-aims are refused and non-fatal — the lever is an earlier re-send;
  (3) two `replan_tail`-era tests left skipped with measured reasons
  (`splice_at`'s `report_range_knots` stays `None` — deliberate simplification
  or observability gap, planner owner's call); (4) the held-axis catch's axial
  velocity match is ~51 % satisfied at the reference weight (a weight decision
  if the first reload sitting wants a harder pull); (5) `skills/check` now
  validates the hop boxes and the node's separation default is 250 mm (a
  GUI-started hop uses the node default). Gate: `./run_tests.sh --full`, run
  2026-09-24: **PASS — 5416 passed, 8 skipped, 1 xfailed in 274.47 s (parallel), serial 6 passed in 18.67 s** (`temp/logs/r4_gate_full_20260924_*.log`; the per-commit gate on the same tree: 5378 passed, 8 skipped in 204.22 s — down from the 6361 baseline by the deleted FSM tests, up by this phase's new ones). **Hardware gate outstanding:**
  `tests/hardware/session_skills_r4.md`. **R5 is NOT cleared until the R4
  sitting.**

- **Sitting 1 (2026-09-27) and its follow-ups (2026-09-28).** The first R4 sitting under
  the calibrated geometry was NOT MET on all three rungs, and `logbook/2026-09-28-skill-stack-
  r4-sitting-1-analysis.md` reads four mechanisms off one bag: (1) the learner memory was learned
  against the old geometry (its converged +20 mm y command now lands +20 mm) — quarantined, cold
  restart; (2) every throw after a laterally re-aimed catch missed 30–110 mm because the plan
  dragged the platform back to the site during the dwell with the ball on the descending hand
  (R3 never saw it: lateral authority was 0); (3) the hop overshot 100 mm on the right tilt: the
  ball separates 20–40 ms after the planned release and the plan re-accelerated the centroid
  toward the far site the instant the release knot passed — at the root, the detach cone buys its
  axial specific force by translating the platform (686 mm/s² at 4°); (4) the reload never threw —
  BB's RELOAD command runs a ~1.1 s ball check that refuses the throw sent 5 ms later, the
  announcement was dropped as "no reload awaiting", the outcome relay was unconsumed, and a REST
  the solve outran (381 ms, origin t_now) was refused by the firmware's scheduled lane on all 80
  frames and integrated into a hand `MAX_DEVIATION` with no host END. **Owner decisions
  (2026-09-28, asked before code):** redesign before the next sitting — a catch's carried throw
  RELEASES FROM WHERE THE BALL WAS CAUGHT and re-centres in flight (not re-pin the authority);
  a post-release platform hold in the planner (sized 50 ms — 75 ms refused the R2 columns schedule `LIMIT_VEL` 303.6 > 300); fix every reload defect now; retrain cold.
  **Landed:** `executor._catch_terminal` releases at the clamped landing xy, the release offset
  fills the memory's reserved `x[2:4]` (seat offset) and its fly-back is pre-compensated by the
  learner's own apex ratio (`_offset_flyback_mm`, closed form: a launch-speed scale s multiplies
  apex and lateral reach alike by s²); `unified_cycle.POST_RELEASE_HOLD_S` / `CycleGoals.
  hold_platform_knots` / the hold rows in `cup_cycle` pin the cup velocity to the frozen stroke
  line over 2 knots (50 ms) on every window built off a release (hop residual over the held knots 68 → 2.75 mm/s at 2 knots, 13.36 at the rejected 3-knot cut; the
  ATTITUDE half refuses at this operating point — carried, § 0 item (7)); skill_node sequences the
  reload on BB's IDLE heartbeat, arms the announcement before the throw, consumes
  `bb/throw_outcome`, ends the attempt on any refusal past the bridge executor, and watches
  `sched_refused` independently of the one-skill bridge executor (root cause of the silent
  latch); `install_segment` rebases a late fresh REST and refuses a late fresh THROW/CATCH
  `ORIGIN_TOO_LATE`; `trajectory_node` seeds a resting hand's velocity at 0. The sim gate had its
  own defect exposed by the redesign (a re-send did not supersede its pending release; the ball
  flew with the first dispatch's takeoff from the re-aimed cup) — fixed with a test. Gates:
  2026-09-28 `./run_tests.sh` → PASS 5477 passed, 8 skipped in 218.28 s, serial 3 in 8.72 s;
  2026-09-28 `./run_tests.sh --full` → PASS 5515 passed, 8 skipped, 1 xfailed in 278.17 s, serial 6
  in 19.95 s; `sim/skills_gate.py --learn` self_toss seeds 0–4 PASS ×2 bit-identical, hop 25
  makes / 0 drops ×2 bit-identical; box swept ×2 bit-identical, gate hash `3fda47b2ad5b`. Every
  other triple: the logbook entry's Verification section. **Hardware gate still OUTSTANDING**; runsheet
  `tests/hardware/session_skills_r4.md` § "what changed" carries the sitting-2 watch list.
- **Sitting 2 (2026-09-28 21:41).** Self-toss from a cold memory held (34 caught / 2, stable to
  five cycles). The hop THROW refused `HAND_LIMIT_ACC` 3/3: the post-release hold held the
  plan-frame tilt axis, which the level correction rotates off the launch velocity across the
  hop; the hold now keeps the cup on its own launch line (no correction ⇒ unchanged). The reload
  met a `ball_butler_node` that answered nothing all session (carried: the runsheet checks it before
  the reload block); behind it the reload now skips `bb/reload` on a ball already in hand and waits
  for BB to leave IDLE after one. Box re-swept ×2 bit-identical, gate hash `bf095653d422`. Entry
  `logbook/2026-09-28-skill-stack-r4-sitting-2-analysis.md`. **Hardware gate still OUTSTANDING.**
- **Sitting 3 (2026-09-28 23:55), analysed through the wayfinder map
  `.scratch/r4-throw-precision/map.md`.**
  - **Hop +88/+110 mm long.** The tilt matches the plan; the platform is still sliding +x at about +100 mm/s at separation (the tail of the plan's own fast +x swing, with 25–50 ms of leg lag), and the ball inherits it. The **pre-release hold** (`PRE_RELEASE_HOLD_S` 0.100, live parameter `pre_release_hold_s`) fixes it offline. The owner contests the mechanism, and the diagnostic sitting's A/B decides.
  - **Self-toss.** The spread is about 16 × 23 mm (1σ), release-side, and a still platform does not remove it.
  - **Reload.** The CATCH was dispatched before the pre-tilt ended and was refused; the timing is fixed.
  - **Refusals** are now in plain language.
  - **Box.** Re-swept, gate hash `ad36fa53aab2`. ⚠ **Carried to R5: the box admits NO columns throw under the hold** (the 0.3 s dwell cell cannot fit 100 ms); the chained sim pattern still passes.
  - Entry `logbook/2026-09-29-skill-stack-r4-sitting-3-analysis.md`; next sitting `tests/hardware/session_skills_r4_diag.md`. **Hardware gate still OUTSTANDING.**
- **Sitting 4 (2026-09-29 19:11): GATE MET, R4 closed.** Entry `logbook/2026-09-29-skill-stack-r4-gate-met.md`.
  - **Hop.** 75/82 caught, and 25 consecutive alternating catches in the 30-throw attempt (14 by the strict verdict: one slow seat logged `caught=False` on a ball the next throw launched normally).
  - **Reload.** 3/3 one-button BB reloads, each followed by 4 catches.
  - **The hold removed the hop overshoot.** Landing x is +12.6 / +2.3 mm (was +88/+110); the diagnostic sheet's A/B is dropped.
  - **Fixed after the sitting.** The reload's Juggle result reported COMPLETED 0/0 at the button (a race between the announcement's context pop and its executor swap, which also let a Stop be overwritten); `_on_announcement` now claims the context and clears it only where it installs the executor.
  - The diagnostic sheet was not flown. Its Blocks B–D fold into R5's first sitting.

### R5 — Two-ball columns

- **Carried from R4** (`logbook/2026-09-29-skill-stack-r4-gate-met.md`;
  `logbook/2026-09-29-skill-stack-stop-after-a-drop.md` for the Stop design). Read before R5's
  first unit. Eight items, full detail in those entries: the box admits no columns throw under
  the 100 ms pre-release hold; re-sweep with each apex as its own band plus a 0.80 m row; fly
  two-site patterns with re-aim off (`catch_resend_max 0`); Stop-after-a-drop landed 2026-09-29
  but unflown, columns' own drop policy (a second ball still in flight) left as an R5 decision;
  55/99 catches seated > 0.15 s late; the reload hand could rise during the pre-tilt instead of
  rushing at the catch; self-toss precision (1σ 17.7/15.8 mm, re-aim on) short of the ±10 mm
  wayfinder target; leg-bus frame drops continuing at 1-2 episodes/min (`plans/active/
  leg-bus-frame-drops.md`) — watch `lead_clamp_mask` with two balls loading the legs harder.
- **Owner decisions (2026-09-30), asked before code — the R5 re-scope.** Three probes on the
  real planner (scratchpad `probe_columns_cells.py`, `probe_reattitude.py`,
  `probe_owner_sequence.py`, 2026-09-30; tables in the R5 logbook entry) established, before any
  design was put to the owner: (i) the committed box's columns cell (`tools/admissible_sweep.py::
  _throw_cell(P1, P2, T, dwell)`) models a rest → transit → release throw the pattern never flies;
  the segments columns actually flies (a launch from rest at its OWN site over 0.4 s; the transit
  CATCH-with-throw; the standalone transit CATCH) pass the 100 ms pre-release hold at 0 / 0.05 /
  0.10 s alike, but at 0.9 m / 100 mm / 0.3 s sit at 131–147 k mm/s³ of leg jerk against the
  sweep's 135 k margin, hold or not — R3 carry (a) by another name, and a longer dwell SHORTENS the
  transit (τ = (t_f − d)/2). (ii) A Ball Butler ball from its current perch (1.74 m up, 1.1 m away;
  R4 gate log: pitch 69.7°, 3.34 m/s) arrives at 5.6 m/s and 11.9° off vertical; a level columns
  transit CATCH tolerates ~2° (5° refuses `LIMIT_VEL` 302 > 300, 12° refuses 351 plus the
  cup-contact floor), so B needs the 12° receive attitude, and the held-axis catch refuses
  `CATCH_AXIS` from any post-release seed by construction. (iii) The owner's six-step BB start
  (ball 1 thrown vertically at P2 → transit + 12° tilt to P1 → held-axis catch of BB's ball 2 →
  re-orient + vertical throw → transit back for ball 1) costs 0.50 + 0.20 + 0.40 + 0.278 = 1.38 s
  of serial windows against ball 1's 0.857 s flight today, and 1.28 s vs 0.903 s at the YAML
  ceilings (vel 1000, hand 3900 — a 1.0 m first throw needs ~3800 and refuses `HAND_LIMIT_ACC`
  at the 3500 session cap). What binds: step 4 at 0.479 s is `cup_realize.
  TILT_ACCEL_BUDGET_FRACTION`'s static-lever tilt budget (`TILT_PIN`) while the legs run at
  1135 of 5000 mm/s²; the legs bind at 0.40–0.45 s once it is lifted; a re-level rides INSIDE a
  launch window at no cost beyond the launch's own 0.4 s; lifting the tilt budget makes the
  re-orient-and-throw window worse (`LIMIT_JERK` / `LIMIT_ACC` at every length). The cup sits
  ~0.75 m above the tilt centre, so 12° is a ~150 mm cup excursion, and the start needs two.
  **Decisions.** **D1 Start = Ball-Butler-initiated from BB's CURRENT position, as a re-attitude
  programme inside R5** (owner: "the end result must be that BB can initiate a two-ball
  pattern"; the estimate that this is short by 0.05–0.1 s at every ceiling is on record and the
  owner chose to have the sim prove or refute it): fuse the held-axis catch, the re-orientation
  and the throw into ONE QP window (and, if that is not enough, the post-release transit + tilt
  approach into it); ramp the session limits toward the ceilings with logged measurements; throw
  the first ball at 1.0 m (hand cap 3500 → 3900). Pre-registered criterion: if the fused start's
  measured floor exceeds ball 1's flight at every ceiling, the decision re-opens on BB's placement
  (a release point within ~0.25 m of the catch site at ~1.0 m above the cup plane gives ≤ 5° at
  ≤ 4.5 m/s and closes the cycle with the stock catch). **D1 outcome, same day:** unit F-a
  (Opus, real planner, ten probes) measured the best fused start at 1.303 s (session limits) /
  1.203 s (ceilings) against 0.857 / 0.903 s — the criterion FIRED; the floors depend on the HOLD
  angle, not the arrival, and only a ≤ 4° hold at the ceilings with a 1.0 m first throw fit, by
  +10 ms inside 10 ms islands (`LIMIT_JERK` 202 k > 200 k between). Owner: **hold 4°, let the
  ball enter ~8° off the cup axis, test the cup first** (the FSM era caught 6–28° off-axis). Unit
  F-b then landed the reload-only fused held-catch → re-level → throw window (0.90 → 0.70 s per
  reload at the session limits, 0.65 s at the ceilings — the slew must land two knots before the
  release for a clean seam, which the F-a probe had not seen), and that honest seam puts the 4°
  start 40–90 ms short at every ceiling. Owner: **hold F-c (the capture span + ramps), fly the
  4° cup test, re-open on BB placement after it.** The D2 Stop as first built (a REST from rest to
  the other site in the 0.228 s left after the previous catch's tail) refused `LIMIT_VEL`
  315 > 300 and was redesigned: the LAST throw is aimed at the site where the held ball is, so
  the platform is under the landing by construction and nothing carries a ball.
  **D2 Stop:** one ball held; for the last
  ball the platform moves under the landing as though catching it with the hand kept low, so it
  strikes the held ball and comes to rest on the platform — not a resumable state, the operator
  clears it; cone delivery later. **D3 Drop with the other ball in flight:** keep catching the
  survivor — its next CATCH is dispatched without its carried throw, then the closing REST. **D4
  Box:** re-model the columns cell as the segments columns flies, keep the 100 ms hold (one path),
  separation pinned at 100 mm (75 mm balls collide closer), grid dwell {0.20, 0.25, 0.30} s ×
  apex {0.85, 0.90, 0.95} m as per-apex bands, plus the hop's per-apex bands with a 0.80 m row
  (carry item 2). **D4 outcome, same day:** on the re-modelled cell (seeded from
  `Segment.release_state`, after the sweep's four raw re-solves were found to plan a launch the
  machine does not fly — every chained cell since 2026-09-28 had been) the 90 % box at
  300/5000/150 000 is a needle: 0.85 m admits x ±1 mm / y −4…+8 mm with the apex pinned at 0.85;
  0.90 and 0.95 m are empty; the hop's per-apex bands are healthy (±10 / ±20 mm at 0.95 m). The sim
  ran columns at 0.90 m clean (30/30 × 5 seeds, 0 drops) on the 100 % runtime gate alone. Owner:
  **ramp the session leg jerk to 200 000 mm/s³** (the R2 sizing point and the YAML ceiling; 150 000
  was the 2026-09-16 "safe enough" choice, not a measurement) and re-sweep the whole box at it; the
  launch default stays 150 000 until the ramp is logged per `leg-gain-tuning-methodology.md`, the
  runsheet sets 300/5000/200000 before `skills/check`, and the R5 sitting's first block is the ramp
  measurement on self-toss and hop. Learner memory: retained across attempts within a sitting under
  one `plant_id`, cold per sitting (as R4). The hand rests at home (owner, 2026-09-29).
- **Build (re-scoped 2026-09-30).** The fused receive → throw window and the start schedule
  (`compile_columns` grows an opening pre-tilt REST, two BB catches anchored on two
  announcements, the transit + tilt approach); the Stop and survivor policy of D2/D3; the D4 box;
  `sim/skills_gate.py` running the BB-initiated start with the learner on; the runsheet.
- **Gate.** Sim: the BB-initiated start closes at a limit set inside the YAML ceilings and five
  consecutive columns cycles follow it, five seeds, 0 drops. Hardware: BB initiates the pattern;
  five consecutive cycles (the paper's criterion), then 30 catches; the consecutive-catch count
  per attempt logged as the learning curve.
- **Outcome (software, 2026-09-30).** Entry `logbook/2026-09-30-skill-stack-r5-columns-bb-start.md`
  (ten agent units, Sonnet by default, Opus for the two QP units). Landed on `skill-stack`: the
  fused held-catch → re-level → throw window for the reload (`CatchEvent.held_from_k`,
  `HELD_SLEW_LANDS_BEFORE_RELEASE_KNOTS`, `hold_tilt` on a STEADY; 0.90 → 0.70 s per reload,
  unflown); `Segment.release_state` and the sweep tool's four raw re-solves deleted (every
  chained cell since 2026-09-28 had been seeded from a launch the machine does not fly); the
  columns cell re-modelled, `dwell_s` stamped and enforced, per-apex bands, the hop's 0.80 m row,
  the ladder's timing model fixed (`pattern_flight_s`); the box re-swept at 300/5000/200 000 ×2
  bit-identical (gate `581109806b4e`, 43 890 verdict rows a run) and installed; the Stop as a
  cross-site last throw (`Skill.shadow_landing`); `DROPPED_SURVIVOR_STOPPED`; the feed-triggered
  columns start (`compile_columns(feed=…)`, `_reload_ctx.kind='columns'`, a BB announcement or an
  un-announced tracker fit; `_hand_home_error` and the D3 refusal deleted); `hold_tilt_max_deg`
  for the 4° cup test; `detect_human_throws` a live tracker parameter; a Stop during a feed wait
  now clears it; `sim/skills_gate.py --learn --pattern columns`. **Sim: `python sim/skills_gate.py
  --learn --pattern columns --seeds 0 1 2 3 4 --no-viewer --apex-m 0.90 --target-throws 30`, run
  2026-09-30: PASS 5/5, 0 drops, 30/30 consecutive catches every seed, 103 s** (the BB-initiated
  start's sim criterion is NOT met — see D1's outcome). Gate: `./run_tests.sh --full`, run 2026-09-30 15:10–15:16 (`temp/logs/gate_full_r5_day1_final2_20260930_1510.log`): **PASS — 5760 passed, 9 skipped, 1 xfailed in 282.42 s (parallel), serial 6 passed in 19.73 s, 308 s total.** **Hardware gate
  outstanding: `tests/hardware/session_skills_r5.md`** (colcon build owed; the 200 k ramp block
  first; then the 4° cup test; the columns block flies from a human lob until BB can feed).
  **R6 is NOT cleared** — R5's hardware gate and the BB-start decision stand between.
- **Owner decisions (2026-09-30 evening), asked after sitting 1** (full narrative:
  `logbook/2026-09-30-skill-stack-r5-sitting-1.md`). Sitting 1 flew the 4°/0° cup test: MET at
  both caps (4°: 2/2 smooth; 0°: 6/7, the one miss a precondition refusal, not a rebound) — the
  cup takes a near-level feed. That result reopens the columns feed-catch design D1's outcome had
  left banked at a 12° receive attitude. **Block C (human-lobbed columns start) RETIRED**: no lob
  registered across 3 attempts, even near-vertical ones beside the robot; no more
  human-initiated multi-ball routines, BB-led patterns are the goal.
  **Receive-level feed catch adopted; a held-axis (hard lateral stop) design was drafted and
  rejected on the numbers**: a hard-lateral-stop catch (the ball met at zero lateral velocity)
  needs a rest-to-rest 100 mm transit of ~0.220 s under the 300/5000/200 000 box (bang-bang
  minimum, `T = (32·D/J)^(1/3)`) against ~0.225 s available in the feed catch's own window — a
  margin thin enough that the executor's own ±40 mm lateral landing clamp (D = 140 mm needs
  ~0.246 s) refuses outright. The adopted design (`Skill.receive_tilt` /
  `CatchTerminal.receive_tilt` / `CycleGoals.receive_tilt`; `compile_columns(pattern, feed=…)`
  sets `(0, 0)` on the feed catch skill only) pins the touch-down ATTITUDE level without pinning
  lateral position or velocity — no hard stop, no held-axis span — and the feed catch that
  refused `LIMIT_VEL` 366.5 > 300 without it plans at hand fraction 0.951 with it.
  **BB placement stays the pre-registered fallback** if the next sitting's 0° reload gate fails:
  move Ball Butler ~0.5 m from the cup at the same height, near-vertical lob (pitch ~84°),
  re-run BB's accuracy volley after moving (a placement change invalidates the fitted aim affine
  the same way the 2026-09-27 coplanar-marker recalibration did).
  **0° gate criterion (next sitting, pre-registered)**: 10 BB feeds at `hold_tilt_max_deg 0.0`
  (self_toss reload), ≥ 9 caught AND ≥ 8 smooth seats (`seat=` +0.05..+0.15 s) AND the probe's
  landing-vs-committed within ±30 ms mean; 3 catches filmed on the owner's high-speed camera.
  **Session limits carry forward**: leg jerk ceiling stays 200 000 mm/s³ (sitting 1's ramp HELD
  there — 4/4 self_toss, 8/8 hop, no clamp, no latch); hold default for plain (non-columns)
  reloads stays 12° until the 0° timing is fixed.
  **Settle refusals** (5/19 in sitting 1, all inside the 1.0° position tolerance, traced to a
  150 Hz finite-difference rate estimate dithering past a 3.0°/s gate) fixed by BB firmware FW 5
  (settled = position error inside 1.0° for the last 15 samples AND rate ≤ 12°/s AND in-band
  now) plus a Jetson-side retry-once on the literal `THROW_ABORTED_NOT_SETTLED` token.
  **Landing-timing and lateral-bias constants**: `BB_RELEASE_PUSH_LAG_S = 0.037` s and
  `BB_FLIGHT_BIAS_S = 0.015` s folded into Ball Butler's announced landing (+52 ms total, applied
  once, at the publisher); the +27/+11 mm lateral bias traces to BB's aim affine
  (`throw_affine_correction.json`, fitted 2026-06-09 at a BB pose now 10.3° of yaw stale) — fix
  is re-running BB's accuracy volley (`bb/start_accuracy_calibration`) and refitting the affine
  offline before the next sitting's reload/columns blocks. Runsheet:
  `tests/hardware/session_skills_r5_sitting2.md`.
- **Sitting 2 (2026-10-02)**: the 0° gate caught 10/10 but FAILED its timing criteria (landing
  +55.3 ms outside ±30 ms, 1/8 smooth seats); BB-fed columns 0/7 cycles — 4 attempts let ball A
  throw and then collided with Ball Butler's feed in flight (mocap-confirmed), 3 refused
  `ORIGIN_TOO_LATE`. Root cause: `receive_tilt` (the sitting-1 fix) was never added to the
  `InstallSegment` wire, so the live dispatch still banks `tilt_to_receive`. The owner's
  site-swap fix for the collision costs `LIMIT_ACC` 101% undisplaced; four fix units (U0-U3)
  proposed, none landed. Full detail: `logbook/2026-10-02-skill-stack-r5-sitting-2.md`.
- **Sitting-2 fix units, landed 2026-10-02 afternoon (not yet flown).** Three root causes fixed:
  `receive_tilt` now crosses the `InstallSegment` wire (U0, closed by a wire-map contract test);
  the collision is removed by swapping which site each ball uses, feed aimed 20 mm toward A (U1);
  `ORIGIN_TOO_LATE` now gives a fresh-origin THROW/CATCH the same 0.150 s solve budget a splice
  gets (U2), plus a `STALE_STATE` seed fix U2's look-ahead surfaced. The swapped layout's
  undisplaced catch needed the hand cap raised 3500 → 3900 rev/s² and the box re-swept (gate
  `f96fc30012d7` → `2adc56a46def` → `458bb544b345`, the last step because cup_cycle.py's restated in-QP bound was made to import the YAML value) — the knife edge was ball A's own mid-chain fold, not the feed
  catch. Per-unit test triples in the logbook entry's Fix section. The hardware gate moves to
  sitting 3 (`tests/hardware/session_skills_r5_sitting3.md`); sitting 2's own sheet is superseded.
- **Sitting 3 (2026-10-02 evening, three launches; analysed 2026-10-02/04).** The swapped layout
  flew: the feed no longer collides with A and the feed catch is no longer refused. Fed columns
  reached its third throw on most attempts and never a fourth: 21/21 attempts ended
  `WINDOW_TOO_SHORT` at skill 3 because the per-ball tracker latch resolved to the OTHER ball's
  flight (its own was still `TO_BE_THROWN` 50 ms before release), so the fourth catch aimed at a
  landing already in the past; the same mis-latch credited throw N with throw N-1's flight and
  wrote 41 bogus learner rows. The throws are wide (self-toss sd 17 → 23 mm per axis, y bias +13 mm)
  from the ball's position in the cup, a learner intercept frozen by survivorship in its neighbour
  selection, and a `level` reference leaning ~0.3° toward +y — NOT platform tilt (held to 0.2°,
  R² 0.00). Two 74 mm balls 100 mm apart have 26 mm of clearance; the 20 mm feed aim spent 17 of
  it. Legs 1/2/4 sit on the 10 A clamp in every fast transit (lag, not inertia). One hand E-STOP
  was a ball pinched between the funnel ring and the descending hand; two more pinches never
  latched (5.5 s at 50 A). Full detail: `logbook/2026-10-04-skill-stack-r5-sitting-3.md`.
- **Sitting-3 fix units, landed 2026-10-04 (not yet flown).** The two-ball association contract
  (`throw_time` on the `BallState` wire, one claimed set, one executor gate, the sim rehearsal
  running the real correlator with a negative control); the learner law revised (§ 2.5) and the
  memory purged; a `level_trim_deg` parameter with `tools/probes/level_vs_ballfit.py`; the
  `columns_1ball` test pattern (ball B a phantom: motion only); the hand-jam detector + recovery
  in the bridge with can-bridge FW 26 `HAND_MOVE_TO` (flashed 2026-10-04 16:34, receipt taken); and the
  roomier geometry — apex 0.95, dwell 0.27, separation 125, leg velocity 350, feed aim 10 — with
  the box re-swept (0.25 was jerk-bound and 0.30 acceleration-bound at 125 mm; 0.27 keeps 10 %
  on every limit). The hardware gate moves to sitting 4 (`tests/hardware/session_skills_r5_sitting4.md`),
  run one step at a time: bench the jam recovery, measure and apply the level trim, self-toss,
  one-ball columns, then fed columns. Open after this: feed-catch accuracy (30-40 mm entry
  offset, open-loop), the hop re-aim refusals, leg acceleration feedforward before any
  acceleration-limit raise, skill_node refusing a start while a jam is held.
- **Sitting 4 (2026-10-04 evening, four launches; analysed and fixed the same night).** Fed
  columns never passed four throws. The jam detector could never fire on the robot (its 50 ms
  diagnostic-age gate against a 1 Hz frame; the replay had used the 100 Hz republish age);
  a missed feed never ended the pattern (the one cup sensor confirmed Ball 1's seat flicker as
  Ball 2's release, a 40 Hz tick lottery); the catches are on time (±10 ms) and bounce with
  lateral miss — Ball Butler lands (+29, +25) mm from its request on every feed and none seats
  before the +0.19 s throw reversal; the `LIMIT_*` refusals are landing-time jitter (±20–30 ms)
  against the 10 % margin — more dwell costs transit, and the session acc/jerk already sit on
  the YAML ceilings; attempt 2's guard latch was a false trip on a 100 ms-old hand encoder
  sample. Landed: detector age bound 1.5 s + measured-descent P1 + `HAND_MOVE_TO` as a
  command source (replay PASS on six bags, all three pinches); `MISSED_CATCH` end on a
  sample-complete RAW seat window + `RELEASE_SEAT_EPS_S` 0.10; `columns_1ball_fed` (Ball 1
  phantom, Ball Butler feeds Ball 2; sim PASS 5/5); `columns_feed_bb_bias_mm` with
  `tools/probes/feed_lateral_miss.py`; can-bridge FW 27 (hand guard counts only feedback
  ≤ 30 ms old, MOTOR_FB_STALE on axis 6; flashed 21:44); Ball 1 / Ball 2 names in every
  operator line. **Not fixed: the refusal margin** — needs a ±25 ms landing-time dimension in
  the box sweep and a geometry or ceiling decision (the legs sat on the 10 A clamp in
  transits; raising acc/jerk means raising that clamp — owner's call). Next:
  `tests/hardware/session_skills_r5_sitting5.md` (the fed half alone with the bias measured,
  then fed columns). Entry: `logbook/2026-10-04-skill-stack-r5-sitting-4.md`.
- **Sitting 5, morning (2026-10-05, three launches, nothing flown).** FW 27's receipt is in
  (`BRIDGE_FW_CHECK: OK — can-bridge v27`). Every JUGGLE goal was refused: the box is swept at
  350/5000/200000 and the launch default was still 300/5000/150000, coupled only by a runsheet
  row the morning skipped. The launch default is now the R5 point and the committed box is
  pinned to the launch default (limits, gate hash, dwell) by the suite; the box was re-swept
  at the same limits for the new gate hash. The jam detector fired honestly on both bench
  pinches, and both recoveries ended UNRECOVERED on `raise did not track`: a raise issued as a
  RETARGET of the stalled move is planned by the ODrive from the setpoint that ran on below
  the hand, so the hand is pushed down at the relief current until the setpoint climbs back
  (0.6 s on raise 1, the whole window on raise 2; raise 1 passed its check only on the ball's
  spring-back). Fixed: an ANCHOR (HAND_MOVE_TO to the measured position) before every raise,
  and the raise is an absolute 5.0 rev clearance (`hand_jam.raise_to_rev`, the owner's number)
  instead of 1.0 rev above the stall; the test plant now carries the ODrive setpoint and fails
  the old machine the way the robot did. Entry:
  `logbook/2026-10-05-skill-stack-r5-sitting-5-launch-limits-and-jam-anchor.md`. The sitting-5
  sheet continues from § 2.
- **Sitting 5, afternoon (2026-10-05, bag `2026-10-05_16-42-57`).** The jam recovery passed twice
  on the bench (owner: objective complete); Ball 1's half caught every real throw; Ball 2's half
  caught nothing cleanly in 21 runs — 13 never attempted (the transit into the feed catch refused
  `LIMIT_ACC` 5010-5306 vs 5000), 6 attempted with the cup arriving with the ball, still sliding,
  31-64 mm off centre, while the same sitting's 3 feeds into a PARKED cup were caught 3/3 at the same
  11-12° arrival. Owner decisions (evening): the fed columns start with a **hop entry** (Jugglebot
  holds Ball 1 at the feed site, throws it across to its own site, stays parked for Ball 2;
  `skill_node` `columns_entry`, default `hop`), and **every catch is taken high** (catch plane
  930 mm, release unchanged at 860; the learner's apex made rise-aware, § 0 item 5 amended, memory
  migrated; `sites.py` joins the box's gated files; the cost: less tolerance to a throw that flies
  high — the columns apex ceiling is 0.950 m, and R3's +11 % cold-start plant error is refused from
  ~900 mm; fallback 880 mm). Tilting toward Ball Butler was considered and kept as the fallback. Entry:
  `logbook/2026-10-05-skill-stack-r5-sitting-5-afternoon-parked-feed-and-catch-high.md`; next sheet
  `tests/hardware/session_skills_r5_sitting6.md`.
- **Outcome (sitting 1 + evening fix units, 2026-09-30).** Sitting 1 flew the 4°/0° cup test and
  the fused reload; Block C (human lob) never claimed a feed and is retired (see the owner
  decisions above). Four fix units landed the same evening, uncommitted at the time of this
  update: the receive-level feed catch, a pre-throw columns-feed feasibility check
  (`REJECTED_COLUMNS_FEED_UNCATCHABLE`), the landing-timing/lateral-bias corrections, and BB
  firmware FW 5 + a Jetson-side retry-once for the settle epidemic. Verification (all
  2026-09-30, details in the sitting-1 entry): box re-swept twice bit-identically at 200 k
  (gate `f96fc30012d7`, content unchanged); the fed columns sim rehearsal
  (`sim/skills_gate.py --learn --pattern columns ... --feed-angle-deg 11.9 --feed-speed-mmps
  5600`) **PASS 5/5 seeds, 1 attempt each, 0 drops, 30/30**; `./run_tests.sh --full` (20:04):
  **PASS 5784 passed, 9 skipped, 1 xfailed in 281.32 s, serial 6 passed**. Sitting 2 hardware
  gate (`tests/hardware/session_skills_r5_sitting2.md`): **OUTSTANDING** — the 0° reload gate
  and the BB-fed columns block have not yet flown; BB FW 5 flashed 2026-09-30 22:45 (receipt 4 -> 5).

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
