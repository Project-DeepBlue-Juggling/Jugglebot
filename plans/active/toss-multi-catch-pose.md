---
title: Multi-pose catch cycling — a TossContinuous session that walks a ring of catch poses
created: 2026-08-28
status: proposed   # out of DOCUMENTATION_GUIDE 2.6's active|completed|superseded vocabulary, used
                   # deliberately for a document nobody has approved yet (the convention
                   # `leg-bus-frame-drops.md` established); promote to `active` on approval
owner: Harrison
last_updated: 2026-08-28
related_plan: toss-pipelined-preamble.md
related_logbook:
  - 2026-08-28-displaced-chain-stale-site.md
  - 2026-08-28-pipeline-first-contact-deadlock.md
  - 2026-08-27-phase-b3-session-scoped-arming.md
  - 2026-08-27-phase-b4-two-slot-pipeline.md
  - 2026-08-27-aimed-reach-pretilt.md
  - 2026-08-26-possession-verdicts-become-sensor-only.md
related_config:
  - config/hardware_config.yaml → jugglebot_operational.toss_session_max_throws (20)
  - config/hardware_config.yaml → trajectory_op.lean_gain (0.6)
  - config/hardware_config.yaml → trajectory_op.min_move_duration_s (0.20)
  - config/hardware_config.yaml → trajectory_op.catch_reach_envelope_mm (80)
related_code:
  - ros_ws/src/jugglebot_interfaces/action/TossContinuous.action
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::TrajectoryNode._svc_arm_catch
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_execute_toss_continuous
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_build_toss_cycle
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_arm_session_declare
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_toss_session_center_drift_mm
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_predicted_chain_site_mm
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_toss_already_positioned
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py::_position_platform_for_toss
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py::pre_dispatch_budget_s
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py::note_position_result
  - ros_ws/src/jugglebot/jugglebot/toss_session.py::TossSessionSequencer.required_dwell_s
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py::compute_release_state_tilted
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/shaping.py::LeanShaper
  - ros_ws/docs/catch_reach_envelope.md
---

# Multi-pose catch cycling — a TossContinuous session that walks a ring of catch poses

## 1. Context

### 1.1 The goal, in the owner's terms

> *"I'd like to be able to schedule different catch poses that are cycled through
> until `num_throws` is met."* — owner, 2026-08-28

A `TossContinuous` goal carries a **list** of catch poses `[B₁, B₂, … Bₙ]` instead
of one. The session walks the ring: cycle 1 throws from wherever the platform is
toward `B₁`, reaches to `B₁` during the flight, catches and stays there; cycle 2
throws from `B₁` toward `B₂`; and so on, wrapping at `n` until `num_throws` is
met. The machine already chains at the catch site — that is the shipped intent of
`toss_stay_at_pose_on_caught` — so this plan does not invent chaining. It makes
the chain's *destination* a per-cycle quantity.

### 1.2 What the machine does today, and the one field that forecloses it

`TossContinuous.action`'s goal carries `geometry_msgs/Point catch_position`, and
its own IDL comment already names this plan:

> *"v1 is deliberately ONE point for the whole session: a per-cycle waypoint list
> (A → B → C with a different B each cycle) is the obvious v2 and is explicitly
> OUT OF SCOPE here — it needs the throw-site frame question settled first."*

`_execute_toss_continuous` reads that field once into a local 3-tuple and closes
over it for the whole `while rclpy.ok()` loop. Every cycle receives the same
tuple. From there `B` fans out to **eight** consumers — the release solve
(`compute_release_state_tilted`), the aim/ILC map lookup (`toss_cal.lookup` /
`toss_ilc.lookup`, both keyed on `B.xy`), the POSITIONING target
(`_toss_positioning_xyz`), the FSM's workspace and displacement gates, the
announcement's `landing_position`, the reach-envelope declaration
(`catch/reach_center`), the reach publish's landing prediction, and the toss
record's `goal_catch_xyz_stow_mm`.

Two of those eight are not merely *fed* a single B; they **enforce** it:

* **`trajectory_node._svc_arm_catch` reads-and-clears the pending
  `catch/reach_center` declaration ABOVE its idempotent early return.** Under
  Phase B3's session-scoped latch (S6) the session raises `arm_catch` exactly
  once, so every later declaration is consumed and discarded and the envelope
  centre stays frozen at cycle 1's `B`. This is the architectural blocker.
* **`_prepare_toss_catch`'s `REJECTED_REACH_CENTER_DRIFT` guard** refuses, before
  the throw, any cycle whose `B` sits more than
  `_TOSS_SESSION_REACH_DRIFT_TOL_MM` = **66.53 mm** from the declared session
  centre. That guard was landed *deliberately* to make the foreclosed case fail
  loudly, and its own comment names the forward path this plan takes:

  > *"The forward path, if a session ever genuinely needs a different B per
  > cycle, is the redundant-raise capture (move `_svc_arm_catch`'s idempotent
  > early return AFTER the centre capture for `want=True`): a trajectory_node
  > change with its own evidence, never a widening of this number."*

Multi-pose **requires** that change. An 80 mm envelope centred on `B₁` would
strangle a catch at a `B₂` 140 mm away: the A→B reach is published at
`t_release`, so the refusal would arrive with the ball already airborne — which
is precisely the failure `ros_ws/docs/catch_reach_envelope.md` § 2 exists to
eliminate.

### 1.3 The decisions already taken — settled, not to be re-opened

Owner, 2026-08-28:

1. **The cup catches at the FULL RECEIVE TILT.** Seating quality wins. The
   post-catch re-orientation to the next throw's pre-tilt is therefore a **real
   commanded move**, the displaced-chain dwell floor rises honestly, and the
   runbook publishes the risen floor rather than hiding it. The original "no
   dwell extension" ask is superseded by this decision.
2. **The constant-beat fast path is DEFERRED.** Catching at (or near) the next
   throw's pre-tilt to eliminate the re-orient — the P-4 pre-tilt substitution
   widened from its present ±1° aim authority to ~2–4° — is the named future
   lever, not this plan's work. § 7 carries it with its unlock.
3. **The stale-site fix lands separately and first.** Staged nomination from the
   chain-site prediction, plus commit-tick pose re-validation with a
   discard-to-serial on mismatch, is a concurrent change. This plan builds on it.
   Under it, a displaced chained cycle runs the **serial** path — which is
   correct, and means displaced legs pay serial floors.

### 1.4 What the arithmetic says the ring costs — and the finding that splits it in two

The commissioning premise was that the displaced-chain dwell floor rises to
**~0.55–0.65 s**. Re-derived from the shipped constants and the shipped planner,
that premise is **not one number, and neither of the two numbers it splits into
is 0.55–0.65 s.** The split is geometric and it is exact.

**Finding 1 — a two-pose ring re-orients by nothing.** Walk the ring through the
shipped solver and catch policy and the orientation delta between the receive
tilt at `Bᵢ` and the pre-tilt for the throw to `Bᵢ₊₁` collapses to ~0.1 mrad
after the transient. The reason is physical, not numerical: a ball caught at `B₂`
was travelling −x, so the cup axis that faces its arrival points **+x and up** —
which is exactly the launch direction for the throw back to `B₁`. A back-and-forth
ring receives and re-throws through the *same* cup orientation.

**Finding 2 — a ring that TURNS re-orients on every cycle.** A triangle or a
square changes the lateral direction at each hop, so the receive tilt and the
next pre-tilt differ by the turn, and the delta (15–31 mrad) is an order of
magnitude past `_TOSS_ALREADY_THERE_TOL_RAD` = **2.7061 mrad**.

That tolerance is the census-B1 positioning skip's threshold, so the split lands
exactly on the cadence cliff: **a two-pose ring's steady-state leg takes the
SKIP; a turning ring commands a move every cycle and runs serial at the moving
floor.**

⚠ **The skip is not the same thing as the stage, and on the landed substrate a
ring leg does not stage.** This plan's first draft read the skip as delivering
the pipelined floor for free. It does not, and the reason is the
2026-08-28 stale-site fix (`logbook/2026-08-28-displaced-chain-stale-site.md`):
a staged cycle now nominates its throw site from
`_predicted_chain_site_mm` — where the platform *will be* — while
`_toss_already_positioned` compares that nominated positioning pose against the
**LIVE** commanded pose, read at stage time with the platform still parked at the
PREVIOUS site. On a ring those are two different places by construction (that is
what a ring *is*), so `positioning_move` is honestly True, the skip-only rule
(`toss-pipelined-preamble.md` § 2.4.1) declines the stage, and the leg runs
**serial-with-skip**: it stages nothing, but its POSITIONING still costs nothing
once it arrives. That is correct behaviour of the shipped honest-cache contract,
not a defect in it.

**Consequence, stated once and carried everywhere below: the pipelined two-pose
floor (0.4170 s at `h = 1.0`) is delivered by phase M4, not by the substrate as
it stands.** Until M4 lands, every ring leg — two-pose included — runs
serial-with-skip at the shipped pipelined-preamble floors, i.e. **0.5590 /
0.5370 / 0.5141 s at `h` = 0.80 / 1.00 / 1.30 m** (`toss-pipelined-preamble.md`
§ 2.7's published *shipped* column — the floor Phase B was built to beat, not the
floor it achieves). A turning ring is unaffected by M4: it commands a real move
every cycle and its floor stays the moving one.

Measured today (2026-08-28, tree at `f4f2f69`, `/tmp/probe_pose_ring.py`, the
recipe in § 5.1 P1), at `h = 1.0 m` ⇒ `T = 0.9032 s`, ring `(±70, 0, 170)`:

| hop | site A → B | `|B−A|` | pre-tilt | `|Δxy|` | `|Δθ|` | SKIP? | planned move, lean 0.6 | planned move, lean 0 |
|---|---|---|---|---|---|---|---|---|
| 0 | (0, 0) → (+70, 0) | 70.00 mm | 1.001° | 1.014 mm | 17.469 mrad | **no** | 0.5238 s | 0.2000 s |
| 1 | (+71.135, 0) → (−70, 0) | 141.14 mm | 2.017° | 2.043 mm | 17.682 mrad | **no** | 0.6060 s | 0.2000 s |
| 2 | (−72.288, 0) → (+70, 0) | 142.29 mm | 2.034° | 2.060 mm | **0.168 mrad** | **YES** | — | — |
| 3+ | fixed point | 142.31 mm | 2.034° | 2.060 mm | 0.116 mrad | **YES** | — | — |

and for a three-pose triangle at `r = 70` mm (legs 122.2 mm), every hop from 0
onward reports `|Δθ|` of 15.1–30.5 mrad, **SKIP = no**, planned move 0.5854–0.5863 s
at lean 0.6 and 0.2384–0.2392 s at lean 0. A four-pose square at ±50 mm behaves
the same way (25 mrad, moves forever). A two-pose *diagonal* ring — (+50,+50) /
(−50,−50) — reaches its skipping fixed point at hop 2 exactly like the collinear
one, so the property is "two poses", not "one axis".

**Finding 3 — the re-orient's cost is not the tilt. It is `LeanShaper` acting on
two millimetres.** A tilt-only pose move strokes the legs 3.5 mm per degree and
plans at the jerk floor, 0.200–0.253 s. What makes the shipped path plan
0.52–0.61 s is that the pre-tilt pose is offset from the throw site by the
**cup-swing compensation** (`compute_release_state_tilted` step 4 — 1.014 mm at
1°, 2.060 mm at 2°), and `shaping.LeanShaper` at `trajectory_op.lean_gain = 0.6`
superposes a lean whose leg-jerk contribution scales as `(1/T)⁵` off that xy.
Measured, at `|Δxy| = 2.06 mm` and `|Δθ| = 2°`:

| `lean_gain` | 0.0 | 0.1 | 0.2 | 0.3 | 0.4 | **0.6 (shipped)** | 1.0 |
|---|---|---|---|---|---|---|---|
| planned duration | **0.2531 s** | 0.4263 | 0.4884 | 0.5291 | 0.5601 | **0.6069 s** | 0.6718 |

The cost is a **step at any non-zero gain**, not a slope — lowering the gain does
not buy the move back. With lean off, the whole plausible re-orient domain plans
inside 0.35 s:

| lean OFF, planned duration | `Δθ = 0°` | 1° | 2° | 4° |
|---|---|---|---|---|
| `Δxy = 0` mm | 0.2000 | 0.2000 | 0.2471 | 0.3121 |
| `Δxy = 2.1` mm | 0.2000 | 0.2000 | 0.2532 | 0.3160 |
| `Δxy = 19.8` mm | 0.2628 | 0.2684 | 0.2994 | 0.3477 |

**Finding 4 — and this is the one that would have bitten on the bench: the
shipped moving budget is a lie about the arrival, and multi-pose makes every
cycle pay it.** `pre_dispatch_budget_s(positioning_move=True)` charges
`ceil((TOSS_POSITION_MIN_MOVE_S 0.200 + TOSS_POSITION_SETTLE_PAD_S 0.200)/0.040)
+ 3` ticks = **0.520 s**, i.e. it assumes the positioning move plans at the
planner's `min_move_duration_s` floor. At lean 0.6 the move plans at 0.52–0.61 s,
so the real arrival is 0.72–0.81 s against a 0.400 s allowance. The serial
`_step_preparing` guard has **no slip** (that is the pipelined commit tick's
2026-08-28 fix, and it is not on this path), so the cycle terminalises
`ABORTED_CANT_MAKE_RELEASE`. Today that is survivable only because the exposure
is rare and runs on a 5 s operator delay; the cadence ladder already carries it
as a finding. ⚠ **The exposure widened on 2026-08-28 and this plan is the
document that has to say so: it is now cycle 1 of a sitting AND the first
chained cycle after EVERY displacement**, because the stale-site fix makes a
displaced chained cycle decline its stage and rebuild serially as a MOVING cycle
(`logbook/2026-08-28-displaced-chain-stale-site.md`). At the shipped
`lean_gain` 0.6 that rebuilt cycle's arrival exceeds
`pre_dispatch_budget_s(True)`'s 0.400 s allowance and the serial
`_step_preparing` guard — which has no slip — aborts. **Under a turning ring
every cycle is a moving cycle running on the cadence lead, so the abort becomes
the steady state**, and M2's honest budget (or `lean_gain = 0.0`) is what closes
it for both.

Finding 3 is what makes Finding 4 cheap to close: with lean off, the arrival
lands at 0.400–0.548 s against a 0.400 s allowance — an overrun of **0 to 4 loop
periods** instead of the ~10 the shaped move costs — and the residual is
absorbable rather than fatal.

### 1.5 The honest floors, and what the milestone therefore is

Derived in § 2.7 and summarised here. `h = 1.0 m`, `catch_vel_scale = 0.9`:

| leg class | positioning | dwell floor | period | throws/min |
|---|---|---|---|---|
| two-pose ring, steady state, **M0–M3 only** | SKIP ⇒ **serial** (the stage is declined) | **0.5370 s** | 1.440 s | 41.7 |
| two-pose ring, steady state, **under M4** | **SKIP ⇒ stages** | **0.4170 s** | 1.320 s | 45.4 |
| two-pose ring, transient (cycles 1–2) | move | 1.057 s (granted, stretches) | — | — |
| turning ring (3+ poses), every cycle | move | **1.057 s** | 1.960 s | 30.6 |
| turning ring at the shipped lean gain | move | 1.337 s | 2.240 s | 26.8 |

The first two rows are the finding above, priced: a ring leg that takes the
positioning SKIP still **declines the stage** on the landed substrate, because
its nominated site and the live commanded site are two different places at stage
time. It therefore pays the shipped serial-with-skip floor — 0.5590 / **0.5370**
/ 0.5141 s at `h` = 0.80 / 1.00 / 1.30 m — until **phase M4 (§ 4)** makes the
stage-time positioning decision evaluate against the PREDICTED post-reach state.
M4 is what turns row 1 into row 2.

**The milestone for this plan is therefore stated per ring class, not as one
number: a two-pose ring holding a commanded dwell of 0.55 s at `h = 1.0–1.3 m`,
and a three-pose ring holding 1.15 s at the same heights, both flown with every
cycle's `B` its own and every cycle's envelope centred on it.** Note which phase
each claim needs: the **0.55 s two-pose milestone clears the serial-with-skip
floor (0.5370 s) with 13 ms to spare and is therefore bookable from M3**; any
rung below it — MP-4's 0.45 s — needs **M4**. The owner's ~0.55–0.65 s premise
is *met* by the two-pose ring and *missed by a factor of two* by a turning one;
§ 8.2 Q-1 put that adjudication in front of the owner, and its **2026-08-28
re-adjudication** (after the audit refuted the free-pipelining premise) is what
commissioned M4.

### 1.6 What this plan does NOT get: aim

The bag measured an uncalibrated tilt→lateral gain of **1.3–1.5×**
(`toss_cal_loaded` false), i.e. a displaced throw overshoots its nominated
landing by ~30–50 %. A 142 mm ring leg therefore lands 43–71 mm past `B` —
comparable to, or larger than, the cup. `config/toss_calibration.yaml` **has
never been captured** (`toss-selftuning.md` § 3.7 is retired-as-designed), and
`toss_ilc_enabled` ships **false**.

So: **multi-pose is a miss-rich activity until the aim arc closes, and this plan
does not close it.** The consequences are structural and are designed for rather
than hoped away:

* the validation ladder (§ 6) starts at short legs and scores *systematic,
  repeatable* landing error rather than a catch rate;
* the ladder's aim rows are exactly the aim-excited corpus
  `plans/active/critical-point-ilc.md` needs — a fixed-B session excites one
  spatial cell, a ring excites `n`, which is the corpus the ILC build ladder's
  step 3 has been waiting on. **Sequencing: this plan lands, its sittings
  produce the corpus, and the ILC unpark consumes it** (§ 8.1 P-5);
* a drifted ring is fail-closed by construction. The runtime re-checks
  `REJECTED_DISPLACEMENT` and `REJECTED_WORKSPACE` per cycle against the LIVE
  commanded site, so a ring that has walked off its nominal geometry refuses a
  cycle **before** the throw rather than throwing somewhere nobody validated.
  § 6.4 makes that a stop condition and names it an aim finding, not a plumbing
  one.

---

## 2. Architecture

### 2.1 Current architecture

```
_execute_toss_continuous
  │  catch_pose = (req.catch_position.x, .y, .z)      ← read ONCE, closed over
  │
  │  (an accept-time _predicted_chain_site_mm hop fed a chain_site_reachable
  │   bool ⇒ REJECTED_CHAIN_UNREACHABLE here until 2026-08-29; it went with
  │   the lateral planning box whose cycle-2 refusal it pre-empted. The
  │   predictor stays — a STAGED cycle nominates its throw site from it.)
  │
  ├─ TossSessionSequencer(..., pipelined=True)
  │       required_dwell_s = max(commit_budget_s(v) + handoff_margin_s,
  │                              hand_floor_dwell_s)         ← ONE number
  │
  └─ per cycle: _build_toss_cycle(catch_pose, ...)           ← the SAME tuple
        A = live trajectory/commanded_position (tier 8b)
        release = compute_release_state_tilted(catch_pose, T, throw_site_xy_mm=A)
        POSITIONING target = release.pretilt_pose_stow[:3]   (pre-tilt IN PLACE)
        PREPARE: _arm_session_declare  → declares catch/reach_center ONCE
                 _arm_session          → arm_catch(True)     ONCE
                 drift guard: |B − session centre| > 66.53 ⇒ REJECTED_REACH_CENTER_DRIFT
```

### 2.2 Proposed architecture — the pose ring

```
_execute_toss_continuous
  │  ring = [B1 … Bn]  from catch_positions (empty ⇒ [catch_position], legacy)
  │  pose_index : session STATE, a FOLD over terminals (R1, Q-2) —
  │      starts 0 ; +1 mod n on a CAUGHT ; UNCHANGED on a miss / abort /
  │      reject / reload interlude, so a failed leg REPEATS
  │      (== (k − 1) mod n only while every cycle catches)
  │
  ├─ accept: _toss_cycle_graph(ring, flight, num_throws)   ← the WORST-CASE WALK
  │     for every leg i, from EVERY A it can be entered from (§ 2.6):
  │       nominal A_i (predicted park of leg i−1), B_i itself (the REPEAT
  │       case Q-2 creates), and home (the interlude re-entry)
  │       · |B.z − 170| inside the ±50 mm z band  (±xy box GONE 2026-08-29)
  │       · |B − A| inside reach_displacement_limit_mm(T)
  │       · the aim tilt inside MAX_TILT_DEG (ThrowTiltInfeasible ⇒ refuse)
  │       · the walk can PREDICT each A_k at all (None ⇒ refuse, fail-closed)
  │       · the leg's OWN dwell floor (§ 2.7) against the commanded dwell
  │     ⇒ REJECTED_CHAIN_UNREACHABLE / _DISPLACEMENT / _TILT_CLAMP / _DWELL,
  │       each naming the LEG and the ENTRY it failed from
  │
  └─ per cycle k: _build_toss_cycle(ring[pose_index], ...)   ← the FOLD's value
        POSITIONING: go_to_pose(pretilt pose, lean_gain=0.0 on tier 8b)   ← Finding 3
        verified-arrival tick : _declare_cycle_center(B_k)   publishes catch/reach_center
        PREPARE tick         : arm_catch(True)  ← REDUNDANT raise, re-captures the centre
        deferred-bundle tick : verify catch/reach_center_applied == B_k   ← R2
```

**What moves:** the catch pose becomes per-cycle; the envelope declaration
becomes per-cycle; the accept gate becomes a walk; the dwell floor becomes
per-leg; the positioning move stops being lean-shaped on tier 8b.

**What does not change at all:** the S1′/S6/S7 invariants and the two-slot
pipeline; the single-shot never-retried throw dispatch and its tri-state
classification; the hand ladders and `_MAX_ARM_DISPATCHES`; `ball_seated` as a
hard gate on release; every commanded motion staying a profiled move through
`trajectory_node` → `planner` → `feasibility.validate`; the Teensy-side
`MAX_DEVIATION` guard, which remains the leg-path safety authority and which
nothing here enters; the 80 mm reach envelope's **radius**; the displacement cap;
the tilt clamp; the single `Toss` action.

### 2.3 The invariants — four new, added to `toss_session.py`'s S-block

> **R1 — one ring, walked in order, and the index ADVANCES ONLY ON A CAUGHT.**
> `pose_index` is session STATE, not arithmetic on the cycle index: it starts at
> 0 and is a **fold over the terminal outcomes** —
> `pose_index ← (pose_index + 1) mod n` on a CAUGHT terminal, and **unchanged on
> every other terminal** (a survived miss, an abort, a rejection, a reload
> interlude). `pose_index(k) = (k − 1) mod n` is therefore the *special case*
> that holds only while every cycle catches, and it is not the definition; a
> session that misses at cycle 3 REPEATS that leg (owner, § 8.2 Q-2). No cycle
> chooses its own pose, no terminal re-orders the ring, and a reload interlude
> does not reset the index. A structural test pins the fold against a scripted
> session whose outcome sequence includes a miss.

> **R2 — every cycle declares its own envelope centre, and the declaration is
> VERIFIED before the throw.** `catch/reach_center` is published once per cycle
> and consumed by a redundant `arm_catch(True)`; `trajectory_node` echoes the
> centre it actually captured on `catch/reach_center_applied`; a cycle whose
> applied centre differs from its own `B` by more than
> `_TOSS_SESSION_REACH_DRIFT_TOL_MM` refuses `REJECTED_REACH_CENTER_DRIFT`
> **pre-throw**.
>
> The verification is not tidiness. `catch/reach_center` is a topic and
> `trajectory/arm_catch` is a service: there is no cross-transport ordering
> guarantee, and B3 already had to buy the ordering with a full tick. Under a
> single session declaration a lost publish mis-centres the envelope for one
> sitting and shows up as one WORKSPACE reject; under a per-cycle declaration it
> can happen every cycle, and each occurrence is a ball on the floor with the
> refusal arriving mid-flight. C-REACH-1 § 2's own argument — *"a gate that can
> only fire after the irreversible commitment is not protecting anything — it is
> choosing the moment of failure"* — is what makes the echo the contract-shaped
> answer rather than an optional nicety. The pre-registered alternative (accept
> C-REACH-1 § 5 residual 1 and ship no echo) is § 8.2 Q-3.

> **R3 — the accept gate validates the whole walk, not one hop.** Every cycle the
> session will run is simulated at accept through the *same* release solver and
> the *same* catch policy the runtime uses, and every refusal names the leg. One
> implementation, shared by the accept gate and the M0 probe, so a probe that
> disagrees with the gate is a test failure rather than a discovery on the bench.

> **R4 — the re-orient is POSITIONING and nothing else.** Session invariant S2
> ("the session commands NO motion of its own") is preserved verbatim: no
> terminal handler and no session-level code commands platform motion between a
> verdict and the next cycle's POSITIONING. The "levelling out" an operator sees
> between catches **is** the next cycle's POSITIONING move, and it is charged to
> that cycle's budget. A structural test asserts no new `go_to_pose` /
> `timed_target` / `dynamic_target` call site appears outside
> `_position_platform_for_toss` and `_publish_toss_reach`.

### 2.4 The per-Bᵢ threading map

Every consumer of "the catch pose", and what each becomes:

| consumer | today | under the ring |
|---|---|---|
| `compute_release_state_tilted(B, T, throw_site_xy_mm=A)` | one `B` | `ring[pose_index]` (R1's fold); `A` unchanged (the LIVE commanded site) |
| `toss_cal.lookup(cal, B.x, B.y)` / `toss_ilc.lookup(...)` | one node | **already per-B** — the lookup is keyed on `B.xy`, so a ring excites `n` cells with no code change. This is the corpus value (§ 1.6) |
| `_toss_positioning_xyz(B, release_cmd)` | one pre-tilt pose | per-cycle; plus `lean_gain = 0.0` on tier 8b (§ 2.8) |
| `TossSequencer(catch_pose_stow_mm=…)` workspace / z-band / displacement / tilt-clamp gates | per-cycle already, on one `B` | per-cycle on `B_k` — **no FSM change**; the gates already run every cycle |
| `build_announcement_fields(release)` → `landing_position` | one landing | per-cycle |
| `catch/reach_center` declaration | **once per session** (`_arm_session_declare`'s `if self._toss_session_center_mm is not None: return`) | **once per cycle** — `_declare_cycle_center(B_k)`, on the verified-arrival tick |
| `arm_catch(True)` | **once per session** (`_arm_session`'s `if self._toss_session_armed: return True`) | once per session **plus** a redundant raise per cycle whose only effect is the centre re-capture (§ 2.5) |
| `_toss_session_center_drift_mm` | `|B − the session centre|` | `|B_k − the APPLIED centre|` (R2) |
| `_predicted_chain_site_mm` (accept gate) | **none** — the one-hop accept gate was deleted 2026-08-29 with the planning box it was keyed on | the whole walk, `_toss_cycle_graph` (§ 2.6), re-minted on the *unpredictable-park* premise (§ 9.2) |
| `_predicted_chain_site_mm` (**staged nomination**, `reload_coordinator_node._build_toss_cycle`) | fed the cycle's **OWN** `catch_pose`. Correct for a fixed-B session, where "the pose I will catch at" and "the pose the PREVIOUS cycle left me at" are the same pose, so the two readings are indistinguishable | must be fed the **PREVIOUS** leg's pose — `_predicted_chain_site_mm(ring[pose_index(k−1)], flight)`. Under a ring the current feed is **off by one hop** (it predicts the park *after* this cycle's catch, which is where the platform goes NEXT, not where it throws from). At `n = 2` that one hop is a full ring leg — 142 mm, sign-flipped — so the nomination wears exactly the pre-fix stale-site defect's signature: a throw site the platform is not at, with the commit belt the only thing between it and a ball on the floor, firing **once per cycle** as `REJECTED_SITE_MOVED` |
| `TossSessionSequencer.chain_site_reachable` | **deleted 2026-08-29** (with `chain_site_xy_mm` / `chain_box_xy_mm`) | re-introduced as a per-leg verdict list; the refusal names the leg |
| `required_dwell_s` | one number | per-leg (§ 2.7) |
| toss record `goal_catch_xyz_stow_mm` | constant across a session | varies; joined by two additive fields, `pose_index` and `ring_len` |

Five things are session-scoped and **stay** session-scoped, deliberately:
`catch/prime_hold` and `catch/pretilt_hold` (S6 — raised once, lowered once);
`self._catch_vel_scale` and `self._toss_mocap_body` (session constants by
definition); `_toss_prev_landing_perf` / `_toss_cycle_landing_perf` (the
C-POSSESS-1 arrival-boundary latch, which is about the schedule and knows nothing
about poses); and `_toss_trim` (`SessionTrim`), which is **already multi-node
aware** — its confound test requires `len(by_node) >= 2` — and which a ring
therefore feeds better than a fixed-B session does.

### 2.5 The redundant-raise capture, and the contract change that comes first

`ros_ws/docs/catch_reach_envelope.md` § 7 is normative: *"Change the document
first, then the enforcement point, then the tests — never the other way round."*
So M1 edits the contract's § 3 Capture row and § 4 table before it touches
`_svc_arm_catch`.

**Today** the handler reads-and-clears `_pending_reach_center` and then early-returns
on `want == self._catch_armed`, so a redundant raise consumes a declaration and
throws it away. **The change** is to make the redundant-raise path *capture*:

```
want = bool(request.data)
pending = self._pending_reach_center ; self._pending_reach_center = None
if want == self._catch_armed:
    if want and pending is not None:
        self._catch_envelope_center = np.asarray(pending, dtype=float).copy()
        <publish catch/reach_center_applied ; log the re-centre>
    return response          # NO latch flip, NO freeze reset, NO graceful stop
```

Three properties of that shape are load-bearing and each closes a way of getting
it wrong:

* **No `_install_graceful_stop`.** The redundant raise runs once per cycle. If it
  reached the graceful-stop branch it would silence whatever move is in flight —
  which on a chained cycle is the POSITIONING move it is arming *for*. That is
  the *"catch latch armed mid-move — installed a graceful stop (move silenced)"*
  line B3 exists to make impossible, re-introduced once per cycle. § 6.3 row
  MP-M8 makes its absence an acceptance criterion.
* **No `_reset_catch_reach_freeze()`.** The freeze window
  (`catch_reach_freeze_s` = 0.30 s before arrival) is what stops a late
  `catch/dynamic_target` from re-planning into a landing. Resetting it on a
  redundant raise would re-open that window mid-catch.
* **No commanded-pose fallback when nothing is pending.** The
  `_current_state()[0][:3]` branch is correct only at a true raise edge. Reached
  on a redundant raise it would silently re-centre the envelope on wherever the
  platform happens to be — which is exactly C-REACH-1 § 7's *"do not apply a
  declaration outside the arm raise"* failure, dressed as a no-op. A redundant
  raise with nothing pending stays a pure no-op.

**When the re-capture happens matters, and it costs no ticks.** B3 landed the
declaration and the raise on two different FSM ticks — the verified-arrival tick
and the PREPARE tick — because the topic and the service have no ordering
guarantee between them, and `test_reach_centre_declared_a_tick_before_the_arm_raise`
pins it. Those two ticks already run on every cycle of a session; for cycles ≥ 2
they currently no-op behind the two latches. The ring simply makes them do work
again, so `pre_dispatch_budget_s` does not move.

**The contract change, stated.** C-REACH-1 § 3's Capture row becomes: *a
declaration is scoped to exactly one `arm_catch` **call**, raise or redundant
raise; a redundant raise re-centres the envelope and does nothing else.* § 4
gains a row for the multi-pose toss (declares `B_k`, centre `B_k`). § 5 residual
1 ("a lost declaration degrades to the old behaviour, loudly") is **replaced**,
not weakened, by R2's applied-centre echo: the degradation is now caught
pre-throw. § 5 residual 5 (a leaked pending declaration surviving to arm a later
goal) is likewise closed, because the echo makes a mis-applied centre visible to
its own coordinator.

### 2.6 The cycle-graph accept gate

`_predicted_chain_site_mm` already models the one thing this gate needs: *where
does the platform park after catching at `B`?* It solves the release, builds the
announcement fields, and runs `CatchCoordinator.predicted_catch_command` — the
same policy the deferred reach uses. `_toss_cycle_graph` is that function applied
`num_throws` times with the park fed forward.

The walk converges, and that is measured rather than assumed: for the ±70 mm
collinear ring the park reaches a fixed point at hop 2 (−72.288 mm — § 1.4's
hop-2 row, the sign alternating with the leg — pre-tilt 2.034°, `|Δθ|`
0.168 mrad) and every later hop is bit-stable at `|Δθ|` 0.116 mrad. For the
triangle it reaches a three-cycle orbit at hop 2. Walking `num_throws ≤ 20` hops
is therefore both cheap and exact; there is no need to reason about periodicity.

**The walk is a WORST-CASE walk, not the nominal one — that is Q-2's cost.**
R1's index is a fold over outcomes, so the *nominal* sequence of `A`s is only one
of the sequences a session can fly: a survived miss REPEATS a leg (entering it
from `B_k` itself, a zero-displacement hop), and a reload interlude `go_home`s
(entering the next leg from home). A gate that validated only the nominal walk
would admit a ring whose repeat or re-entry hop is infeasible, and the refusal
would then arrive **mid-session, after a ball has already been thrown** — which
is the exact failure § 9.2's first bullet says the accept walk exists to
prevent. So every leg `i` is validated from **all three** entries it can have:

1. its **nominal** `A_i` — the predicted park of leg `i − 1`;
2. **`B_i` itself** — the repeat case Q-2 creates on a survived miss (this hop
   has zero displacement, so it can only fail the workspace/dwell rows, but it
   changes the *pre-tilt*, hence the leg's positioning class and floor);
3. **home** — the interlude re-entry, whenever `on_empty_cup` is RELOAD or
   `stop_on_miss` is false.

Whichever entry is worst governs the leg. When `stop_on_miss` is true and
`on_empty_cup` is STOP, entries 2 and 3 are unreachable and the gate drops them
— it must not refuse a ring for a walk the goal's own flags forbid.

Per hop the gate checks, in the FSM's own order so a refusal at accept and a
refusal at runtime name the same thing:

| check | bound | refusal |
|---|---|---|
| `A_k` known | tier 8b needs a fresh `trajectory/commanded_position` for hop 0 only; later hops use the predicted park | `REJECTED_POSE_UNKNOWN` |
| **the walk's prediction itself** | `_predicted_chain_site_mm` returns `None` at hop `i` (an infeasible aim from that `A`, or a catch-policy refusal) — the gate cannot say where the platform will be, so it refuses. It is **never** treated as "no move needed" / SKIPPED: an unknown park is fail-closed at accept exactly as it is fail-safe-to-serial at runtime | `REJECTED_CHAIN_UNREACHABLE(leg i)` |
| aim tilt | `MAX_TILT_DEG` = 12° (`ThrowTiltInfeasible`) | `REJECTED_TILT_CLAMP(leg i)` |
| `|B − A|` | `reach_displacement_limit_mm(T)` — the SOLE bound since 2026-08-29 | `REJECTED_DISPLACEMENT(leg i)` |
| `|B.z − 170|` | `TOSS_Z_BAND_MM` = 50 | `REJECTED_WORKSPACE(leg i)` |
| the leg's own dwell floor | § 2.7 | `REJECTED_DWELL(leg i)` |

**No drift allowance is added at accept**, and that is a decision rather than an
omission. The runtime already re-evaluates the displacement and workspace gates
every cycle against the LIVE site, and refuses **before** the throw. Padding the
accept gate would refuse rings that fly, while the thing it would be padding
against — aim error walking the ring off its nominal geometry — is already
fail-closed one layer down and is a *finding worth surfacing*, not a nuisance to
absorb (§ 1.6, § 6.4).

`reach_displacement_limit_mm` is worth one caution the plan must carry into the
ladder: it is jerk-bound and **conservative below `T ≈ 0.75 s`, optimistic above
it** (256 mm at `T = 0.80`). ⚠ **RE-POINTED 2026-08-29**: this read "against a
150 mm cap… at the milestone heights the cap binds, so the closed form is not the
active bound", and the gate took a `min` of the two. The cap is deleted, so the
closed form is now the ONLY bound at every height — including the milestone ones,
where it is the OPTIMISTIC side of that asymmetry. A ring leg that plans but does
not fly is therefore a live possibility at long flights and the walk cannot
pre-empt it. That residual is the owner's accepted cost of the deletion; the
ladder should treat each long-flight leg's first fly as informative.

### 2.7 The floor re-derivation — per leg

Nothing about the floor *derivation* changes. `required_dwell_s` keeps its two
branches and each keeps its single derivation; what changes is that the session
evaluates them **per leg** and takes the max over the legs that are not covered
by the lead grant.

```
leg floor(pipelined leg)  = max(commit_budget_s(v) + handoff_margin_s,
                                hand_floor_dwell_s)
leg floor(moving leg)     = max(dispatch(v) + pre_dispatch_budget_s(True)
                                            + FLOOR_REPRESENTATION_SLACK_S
                                + handoff_margin_s,
                                hand_floor_dwell_s)

required_dwell_s          = max over legs that do NOT command a move
                            (a moving leg's lead is GRANTED, and its achieved
                             dwell stretches by grant − throw_delay_s, exactly
                             as cycle 1 of every sitting does today)
```

The grant carve-out preserves the shipped doctrine verbatim —
`min_throw_delay_s`'s own docstring says charging the moving budget at the
session gate *"would refuse every cadence above ~40 throws/min for a cost only
the FIRST cycle of a sitting ever pays"* — and generalises its premise honestly:
when **every** leg moves, the premise is false, the carve-out is empty, and the
moving floor becomes the session floor. That is the turning ring.

**The moving leg's floor also owes the previous catch's settle hold.** A leg that
COMMANDS its re-orient cannot dispatch that `go_to_pose` until the previous
cycle's catch plan has expired: `build_catch`'s post-arrival tail is
`JB_TRAJ_CATCH_SETTLE_HOLD_S` = 0.50 s, and inside it `_active_move_in_flight()`
is True and `trajectory_node` refuses the move `BUSY` — correctly. Bag-proven
2026-08-28_23-53-25: three deterministic `ABORTED_CYCLE_REJECTED_POSITION(BUSY)`
on a displaced chain whose POSITIONING dispatched 0.09–0.23 s after landing. So
the moving branch above gains a `+ JB_TRAJ_CATCH_SETTLE_HOLD_S` term, anchored at
the **committed arrival** rather than at the dispatch. The runtime BUSY re-poll
that landed 2026-08-29 (`TOSS_POSITION_BUSY_PATIENCE_S`,
`toss_sequencer._absorb_position_busy`) is the **bounded absorb** for the residual
a budget cannot pre-pay; § 2.7's floor is the **budgeted** answer — the same
division of labour § 2.8 draws between its re-cut arrival term and its runtime
absorb. Under **M3**'s every-leg-serial rings this seam is universal, not
displacement-only: a co-located chain escapes it today only because the
census-B1 skip never calls the service at all.

**`pre_dispatch_budget_s(True)` must be made honest before any of this is true.**
Its arrival term is `TOSS_POSITION_MIN_MOVE_S + TOSS_POSITION_SETTLE_PAD_S` =
0.400 s, i.e. it assumes the move plans at the planner's floor. With lean off it
nearly does; the residual overrun is at most four loop periods (§ 1.4 Finding 3's
table tops out at 0.348 s ⇒ arrival 0.548 s ⇒ 14 arrival ticks against the
10 the shipped term buys). The plan re-cuts the
arrival term to a config-keyed `toss_position_move_budget_s` sized from the M0
measurement with the `ARRIVAL_BAND_MAX_S` discipline (ceil to the next 10 ms, so
it is a **bound** rather than a datum) — **0.348 s ceils to 0.35 s** — and adds
a runtime absorb (§ 2.8) for what a bound cannot cover.

Floors, computed against the shipped functions (2026-08-28, tree `f4f2f69`;
recipe in § 5.1 P1), `catch_vel_scale = 0.9`, ILC speed trim disabled as shipped:

| apex `h` | `T` | dispatch | handoff | hand floor | **pipelined leg (needs M4)** | **serial-with-skip leg (M3)** | **moving leg, budget 0.680** | moving leg, at the shipped lean gain |
|---|---|---|---|---|---|---|---|---|
| 0.80 m | 0.8079 s | 0.2802 | 0.1188 | 0.3264 | **0.4390 s** | **0.5590 s** | **1.0790 s** | 1.3590 s |
| **1.00 m** | 0.9032 s | 0.2707 | 0.1063 | 0.2994 | **0.4170 s** | **0.5370 s** | **1.0570 s** | 1.3370 s |
| **1.30 m** | 1.0298 s | 0.2608 | 0.0933 | 0.2713 | **0.3941 s** | **0.5141 s** | **1.0341 s** | 1.3141 s |

and the periods those imply:

| apex | pipelined leg | serial-with-skip leg | moving leg (lean off) | moving leg (lean 0.6) |
|---|---|---|---|---|
| 0.80 | 1.247 s — 48.1/min | 1.367 s — 43.9/min | 1.887 s — 31.8/min | 2.167 s — 27.7/min |
| **1.00** | 1.320 s — **45.4/min** | 1.440 s — **41.7/min** | 1.960 s — **30.6/min** | 2.240 s — 26.8/min |
| **1.30** | 1.424 s — 42.1/min | 1.544 s — 38.9/min | 2.064 s — 29.1/min | 2.344 s — 25.6/min |

The pipelined column is `toss-pipelined-preamble.md` § 2.7's table unchanged;
the serial-with-skip column is that same document's **shipped** floor, and it is
what a ring leg actually pays until **M4** lands (§ 1.4). So the honest form of
the claim is: **a two-pose ring costs the cadence one phase, not nothing** —
0.12 s of dwell, recoverable by M4 and by nothing else on this plan's board. The
moving column is the honest price of a turning ring, and Finding 3 is worth
**0.28 s per cycle** of it.

### 2.8 The re-orient is serial POSITIONING — and the two changes that make that true

The owner's decision 3 of § 1.3 says the re-orient is POSITIONING's job, and the
code already agrees: `_toss_positioning_xyz` returns `release.pretilt_pose_stow`,
a single pose carrying **both** the site and the orientation, and
`_position_platform_for_toss` commands it with one `go_to_pose`. There is no
second move to add. What has to change is that POSITIONING must be *charged
honestly* and must *cost what it should*.

**(a) Lean off for the tier-8b pre-tilt move.** `_position_platform_for_toss`
currently leaves `GoToPose.lean_gain` at its `-1.0` field default, which defers to
`JB_TRAJ_LEAN_GAIN` = 0.6. For tier 8b the positioning move is by construction a
**pre-tilt in place**: the target's xy is the throw site minus the cup-swing
compensation, so the commanded translation is bounded by
`cup_lateral_shift_mm(MAX_TILT_DEG)` ≈ **12.1 mm** and is 2.06 mm at the ring's
steady state. Lean shaping exists to lean the platform into *translation-driven*
lateral acceleration so a seated ball stays seated; on two millimetres it adds a
microscopic pose contribution and **0.354 s of PLANNED DURATION** (0.6069 s
shaped against 0.2532 s bare at the ring's steady state — § 1.4 Finding 3's
table), because its leg-jerk term scales as `(1/T)⁵`. Those are two different
quantities and this plan states both rather than blurring them into one range:
the 0.354 s is the *planner's* delta, while the **0.280 s** the § 2.7 floor table
shows between its two moving-leg columns is the *budget* delta — smaller because
the budget is a ceiled tick-quantised bound (0.680 s vs 0.960 s of preamble), not
the raw duration.

The safety argument is that lean-off here commands **strictly less** motion and
**strictly less** leg jerk than lean-on: the shaped plan is the bare quintic plus
a superposed lean, and removing the lean removes both. The physical cost is the
un-compensated lateral acceleration a seated ball sees during the move — peak
`5.7735 · Δxy / T²` = 297 mm/s² at `Δxy = 2.06 mm`, `T = 0.200 s`, i.e. 1.74° of
equivalent tilt, against a cup the machine already catches into at up to 12°.
Tier 8a keeps the config gain: its positioning move is a genuine translation to
`B` of up to 150 mm, which is exactly what lean is for.

The cheapest confirmation is one service call reading back `planned_duration_s`
(§ 5.1 P2) — an explicit `0.0` forces lean off, where only a negative defers.

**(b) A bounded runtime absorb, so a mis-estimated move slips instead of
aborting.** `note_position_result` already receives the real `planned_duration_s`
and already extends `_positioning_deadline` with it. The plan lets it extend
`_t_release` by the same shortfall, once, bounded by the positioning deadline it
already computes, and records it as `position_overrun_s`. This is the serial
analogue of the 2026-08-28 commit-tick slip, and it is adopted for the same
reasons that fix stated: *nothing is armed, no announcement has gone out, and the
condition resolves by waiting*; and `ABORTED_CANT_MAKE_RELEASE` names the throw
budget when the actual subject is a planner that returned a longer move than the
budget assumed.

**The T-P5 walk, taken before the code is written**, because `_t_release` gaining
a writer is what that property forbids: `start()` sets it once; `_slip` moves it
forward; the absorb moves it forward. All three are monotone increasing, the
absorb runs strictly before the announcement is published, and a cycle that never
dispatches contributes no release. So the absorb can only **delay** a release,
never advance one, and cannot schedule a throw stroke into a live catch. § 5.3
T-P6 makes that mechanical.

**One instrument consequence, named rather than discovered.** A granted or
absorbed leg releases later than `next_release_at(landing)` predicted, and
`_set_toss_next_cycle_perf` feeds the C-POSSESS-1 clamp that *prediction*. On the
serial path there is no staged slot to supply the actual number, so the arrival
window closes earlier than the real next release. C-POSSESS-1.C already governs
this — *"where a clamp leaves no interval at all, the part it governs is UNKNOWN
— never CONFIRMED… and never REJECTED"* — so the behaviour is specified and safe;
it is the by-eye cup watch that gets more load-bearing, and § 6.3 row MP-M5 says
so.

---

## 3. Implementation Phase Summary

| Phase | Scope | Status | Risk | Validates |
|---|---|---|---|---|
| **M0** | Measure first: promote the ring-walk probe, confirm the lean attribution on hardware, read the moving-cycle body cost from a sitting, cut the replay fixture. No production code. | NOT STARTED | none | that §§ 1.4/2.7 are measurements, not models |
| **M1** | `catch_reach_envelope.md` C-REACH-1 change **first**, then the redundant-raise capture in `_svc_arm_catch`, then `catch/reach_center_applied`. One node, one handler. | NOT STARTED | **medium — it edits the enforcement point of a normative contract** | that a per-cycle B can centre its own envelope |
| **M2** | `TossContinuous.action` gains `catch_positions` (+ feedback/result index fields); the accept gate becomes `_toss_cycle_graph`; per-leg `required_dwell_s`; the honest `pre_dispatch_budget_s(True)`; lean off for the 8b pre-tilt move. | NOT STARTED | **medium — an interfaces change and a budget re-cut that touches every existing session's cycle 1** | that an impossible ring is refused at accept, by leg |
| **M3** | Session cycling: per-cycle `B` threading (**including the staged nomination's previous-leg feed**, § 2.4), `_declare_cycle_center`, the re-based drift guard, the runtime position absorb, the **survived-miss** terminal (Q-2), the record/feedback fields. | NOT STARTED | **high — the core** | that the ring is walked and every cycle catches on its own envelope |
| **M4** | **Predicted-state staging**: the stage-time positioning decision is evaluated against the PREDICTED post-reach state, not the live pose, so a ring leg that takes the SKIP also STAGES. The landed commit-tick `staged_site_ok` re-validation is the backstop. | NOT STARTED | **high — it makes a scheduling decision on a state that does not exist yet** | that a two-pose ring reaches the 0.4170 s pipelined floor rather than the 0.5370 s serial-with-skip one |
| **M5** | Hardware validation ladder (§ 6). | NOT STARTED | **hardware** | the milestone, on the machine |
| **M6** | Close-out: runbook rungs, contract and logbook, ILC hand-off, archive. | NOT STARTED | none | that the corpus reaches the arc that consumes it |

Phases are strictly incremental. M1 is valuable on its own (it closes
C-REACH-1 § 5 residuals 1 and 5 whether or not the ring ever lands) and is
independently revertible. M2 ships the schema additively with an empty
`catch_positions` reproducing today's behaviour bit-for-bit, so the interfaces
rebuild and the ring can land in different sittings. M3 is the only phase whose
revert loses the feature. **M4 is a cadence phase, not a capability phase**: the
ring works without it, at the serial-with-skip floor, and its revert costs
0.12 s of dwell and nothing else — which is exactly why it is a separate phase
rather than a clause inside M3.

---

## 4. Implementation Phases (detailed)

### Phase M0: Measure before designing against a number

**New files**
* `tools/probes/toss_pose_ring.py` — committed, outputs to `temp/probes/` per
  `tools/probes/README.md`. Walks a ring through the shipped
  `compute_release_state_tilted` → `build_announcement_fields` →
  `CatchCoordinator.predicted_catch_command` → `planner.build_move` chain and
  prints, per hop: displacement, pre-tilt, `|Δxy|`, `|Δθ|`, the
  `_toss_already_positioned` verdict at the real tolerances, the planned move
  duration at `lean_gain` 0.6 and 0.0, and the leg's dwell floor. **It is the
  generator of §§ 1.4 and 2.7**, and M2's accept gate imports its walk rather
  than restating it (R3).

**Scope.** Four measurements:

1. **P1 — the ring walk.** Promote the throwaway that produced § 1.4 to a
   committed probe and pin its numbers. Answers: which ring geometries take the
   positioning skip, where the walk's fixed point is, and what each leg's floor
   is. Already run once (2026-08-28); M0 makes it reproducible.
2. **P2 — the lean attribution, confirmed on the machine.** One `go_to_pose`
   service call to a ~2° pre-tilt pose with `lean_gain = 0.0`, reading back
   `planned_duration_s`. Predicted 0.20–0.36 s against the shipped path's
   0.52–0.61 s. If it does not collapse, Finding 3 is wrong and M2's lean change
   does not land — the whole moving-leg column of § 2.7 then reverts to the
   1.31–1.36 s row and the ladder's turning rungs re-cut with it.
3. **P3 — the moving cycle's body cost.** Read `loop_body_max_pre_s` and
   `loop_period_max_pre_s` for the *moving* cycles of a sitting via
   `tools/probes/toss_loop_census.py`. The one moving cycle in the B0/P1 corpus
   spent 0.3022 s in a single iteration, 0.2774 s of it inside the blocking
   `go_to_pose`. Under a turning ring that is every cycle, so the number sizes
   both the honest budget and § 7's sibling-work argument.
4. **P4 — a replay fixture.** Extend `tools/probes/possession_replay.py` to emit
   a fixture from a multi-pose sitting, following the house
   probe→fixture→test pattern (three instances deep already).

**Acceptance.** P1 reproduces § 2.7's floor table and § 1.4's per-hop table from
the shipped code. P2 returns a `planned_duration_s` in `[0.20, 0.36]` s at
`lean_gain = 0.0` on the real node. P3 produces a moving-cycle body maximum and a
named dominant term. **No production file is touched in this phase.**

**Dependencies.** P2 and P3 need the graph up (no throwing). P1 and P4 need
nothing.

### Phase M1: The redundant-raise capture, and the contract change that precedes it

**Modified files**
* `ros_ws/docs/catch_reach_envelope.md` — **first**, per its own § 7
* `ros_ws/src/jugglebot/jugglebot/trajectory_node.py` (`_svc_arm_catch`, a new
  `catch/reach_center_applied` publisher)
* `tests/ros/test_trajectory_node.py` (the `test_creach1_*` block)
* `tests/ros/test_toss_integration.py` (T-I3's captured-value assertions)

**Scope.** § 2.5. The handler's redundant-raise path captures a pending
declaration, publishes the applied centre, and returns — no latch flip, no freeze
reset, no graceful stop, and no commanded-pose fallback.
`catch/reach_center_applied` is a plain `geometry_msgs/Point`, chosen for exactly
the reason `catch/reach_center` was (C-REACH-1 § 5 residual 4): a
`jugglebot_interfaces` type here would create a split-build hazard for a channel
that carries three floats.

**Critical details.**
* The existing test `tests/ros/test_toss_integration.py`'s *"consumed and
  DISCARDED"* assertion is a **statement of the old contract** and must be
  re-taken, not deleted — rewritten to assert that a redundant raise with a
  pending declaration re-centres, and that a redundant raise **without** one is a
  pure no-op that leaves `_catch_envelope_center` untouched.
* T-I3 asserts the **captured value** of `_catch_envelope_center` across every
  cycle of a chained run, never publish ordering. B3 already established that
  CS-4 (one declaration per cycle, ≥1 tick before the arm) is a **false green**
  on its own: it stayed green through the whole period the declaration was going
  unapplied.
* The echo publishes on every capture, raise or redundant — so a coordinator can
  verify without knowing which kind it got.

**Acceptance.** `./run_tests.sh --full` green. A test drives a real
`TrajectoryNode` through raise → declare → redundant raise → declare → redundant
raise and asserts the captured centre follows each declaration. A test asserts a
redundant raise never calls `_install_graceful_stop` **with a move in flight** —
the ordered-call-log idiom, not a flag.

**Dependencies.** none.

### Phase M2: The schema, the cycle graph, and the honest budget

**Modified files**
* `ros_ws/src/jugglebot_interfaces/action/TossContinuous.action`
* `ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py`
  (`_execute_toss_continuous` ingest + `_toss_cycle_graph` +
  `_position_platform_for_toss`'s `lean_gain`)
* `ros_ws/src/jugglebot/jugglebot/toss_sequencer.py` (`pre_dispatch_budget_s`'s
  arrival term)
* `ros_ws/src/jugglebot/jugglebot/toss_session.py` (`required_dwell_s` per leg)
* `config/hardware_config.yaml` → `jugglebot_operational.toss_position_move_budget_s`
  + regenerate
* `tools/probes/cadence_rung_check.py` (the moving ladder)
* `tests/ros/`, `tests/motion/`

**Scope — the goal schema, and the backwards-compatibility rule.** The goal gains

```
geometry_msgs/Point[] catch_positions   # ordered ring of catch poses. EMPTY => the
                                        # session uses catch_position for every cycle
                                        # (the v1 behaviour, bit-for-bit). Non-empty =>
                                        # cycle k catches at catch_positions[(k-1) % n].
```

plus two additive fields — `int32 pose_index` on the feedback and
`int32[] per_cycle_pose_index` on the result — so an operator watching feedback
can tell which `B` a cycle is aiming at.

**A repeated `geometry_msgs/Point`, not a flattened `float64[]`**, for three
reasons: the array is self-describing, so the whole "length not a multiple of 3"
class of goal errors does not exist; it matches `catch_position`'s own type, so
the accept gate's numeric validator (`_invalid_toss_goal_field`) extends by a
loop rather than by a second derivation; and unbounded arrays of nested types are
native to the IDL here.

The compatibility rule, and the sentinel argument behind it:

* `catch_positions` **empty** ⇒ every cycle uses `catch_position`. A test pins a
  byte-identical decision stream against the pre-M2 tree.
* `catch_positions` **non-empty** ⇒ it is the ring, and `catch_position` must be
  either the default-constructed zero `Point` or equal to `catch_positions[0]`;
  anything else is `REJECTED_POSE_LIST`. The zero `Point` is an unambiguous unset
  sentinel because `z = 0` is 170 mm from the ACTIVE plane and already outside
  `TOSS_Z_BAND_MM` = 50 mm — it is not a legal catch pose, so it cannot be an
  operator's real intent. This is the same doctrine `release_at_perf = 0.0`
  already uses: *0.0 is the only unset sentinel; anything else is preserved and
  judged.* Silently ignoring a field an operator typed is how a session throws
  somewhere nobody asked.
* `1 <= len(catch_positions) <= toss_session_max_throws` (20), else
  `REJECTED_POSE_LIST`.
* A length-1 list is exactly a v1 session, and a test pins that its decision
  stream matches the legacy encoding's.

**Scope — the cycle graph.** `_toss_cycle_graph(ring, flight, num_throws)`
implements § 2.6, sharing one walk implementation with M0's probe. It returns a
per-leg verdict list; `TossSessionSequencer.chain_site_reachable` is
RE-INTRODUCED as that list (the scalar bool and its two companion fields were
deleted 2026-08-29 — see § 9.2), and `_checking_reject` mints the leg-named
refusal.

**Scope — the honest budget.** `pre_dispatch_budget_s`'s arrival term becomes
`toss_position_move_budget_s + TOSS_POSITION_SETTLE_PAD_S`, with the new key
sized from M0/P1 with the ceil-to-10 ms discipline (§ 2.7's table assumes the
**0.35 s** ceil of § 1.4's 0.348 s worst case ⇒ a 0.680 s preamble; anything in
[0.35, 0.36] s ceils to the same 17 ticks, which is why P2's acceptance band
tops out at 0.36 and the budget does not move inside it). This raises the lead
granted to the first cycle of
**every existing session**, not only multi-pose ones — a behaviour change whose
whole visible effect is that cycle 1's achieved dwell grows by the amount the
budget was previously short, and whose whole purpose is that the cadence ladder's
carried finding stops being carried.

**Scope — lean.** `_position_platform_for_toss` sets `req.lean_gain = 0.0` when
the release is tilted (`_release_is_tilted`, the tier-8b pre-tilt-in-place case)
and leaves the `-1.0` defer default otherwise. A guard logs loudly if the
commanded translation of a lean-off move exceeds
`cup_lateral_shift_mm(MAX_TILT_DEG)` ≈ 12.1 mm, because that would mean the move
is not the pre-tilt-in-place the argument rests on.

**Acceptance.** `./run_tests.sh --full` green. An empty `catch_positions`
reproduces the pre-M2 decision stream over the whole `cadence_rung_check` grid.
`_toss_cycle_graph` refuses each of five hand-built impossible rings by the right
leg and the right code. The § 2.7 floor table reproduces from the shipped
`required_dwell_s`. `cadence_rung_check --grid` reports zero
accept-implies-flies violations on the moving ladder and non-zero with the
budget's arrival term reverted.

**Dependencies.** M1 (not strictly — the schema and the gate are independent of
the envelope — but landing M1 first keeps the diffs separable and lets M2's
tests assert against a working capture).

### Phase M3: Session cycling and per-Bᵢ threading

**Modified files**
* `ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py`
  (`_execute_toss_continuous`, `_build_toss_cycle` call sites,
  `_arm_session_declare` → `_declare_cycle_center`,
  `_toss_session_center_drift_mm`, the `catch/reach_center_applied` subscriber)
* `ros_ws/src/jugglebot/jugglebot/toss_sequencer.py` (`note_position_result`'s
  bounded absorb, `position_overrun_s`)
* `ros_ws/src/jugglebot/jugglebot/toss_record.py` (additive `pose_index`,
  `ring_len`, `position_overrun_s`; no `SCHEMA` bump — the schema's own rule is
  that purely additive fields do not bump)
* `ros_ws/src/jugglebot/jugglebot/toss_session.py` (R1–R4 in the module
  docstring, alongside S1–S7)
* `ros_ws/src/jugglebot/jugglebot/toss_sequencer.py` — **also**
  `_terminal_action`, for the survived-miss terminal (Q-2 obligation 1 below)
* `tests/ros/`, `tests/motion/`

**Scope.** R1's index **fold** (advance on CAUGHT only); the **staged
nomination's previous-leg feed** (§ 2.4 — `_predicted_chain_site_mm` takes
`ring[pose_index(k−1)]`, not the cycle's own `catch_pose`); the per-cycle
declaration on the verified-arrival tick and the redundant raise on the PREPARE
tick; the drift guard re-based on the applied echo (R2); the bounded position
absorb (§ 2.8b); **the survived-miss terminal and its arming choreography
(§ 8.2 Q-2's obligation list)**; the record and feedback fields.

**Critical details.**
* **The drift guard keeps its job and gains teeth.** Re-based on the *applied*
  centre it reads zero when the per-cycle declaration landed and reads a full
  ring leg — 142 mm against a 66.53 mm tolerance — when it did not. It therefore
  fires **iff the declaration path was skipped or lost**, which is precisely
  what it should detect once the foreclosed case it was built for becomes the
  feature. The tolerance is not widened; C-REACH-1 § 7's *"do not raise
  `reach_envelope_mm` to permit a larger requested reach"* is the same rule one
  layer over, and it holds.
* ⚠ **Per-leg staging does NOT fall out, and this is the correction that
  created M4.** The draft said a SKIP leg stages because `_build_toss_cycle`
  forces `staged = bool(staged) and not positioning_move`. It does force that —
  and on a ring `positioning_move` is honestly **True at stage time**, because
  the decision compares the nominated pose against the LIVE commanded pose while
  the platform is still parked at the previous leg's `B`. So every ring leg
  abandons its stage (`POSITIONING_MOVE`) and rebuilds serially. M3 therefore
  ships a ring that runs **serial-with-skip**, which is correct and flyable at
  the 0.5370 s floor; **M4 is what makes a SKIP leg stage.** A moving leg still
  abandons the stage and rebuilds serially — the path the 2026-08-28 fix wave
  made deadlock-free — and no new branch is added *in M3*.
* **The interlude is a `go_home`; a survived MISS is NOT (Q-2).** The owner's
  2026-08-28 directive splits what the draft treated as one discontinuity: a
  **reload interlude** still `go_home`s, so the next cycle's `A` is home; a
  **survived miss** now **holds the pose**, safes the hand in place and resumes
  the chain from `B_{k−1}` with the index unadvanced (the leg repeats). Either
  way the accept gate must have validated the hop it will actually fly, so
  § 2.6's walk carries the home-entry hop for every `B` whenever `on_empty_cup`
  is RELOAD, and the repeat-from-`B_k` hop whenever `stop_on_miss` is false.
* **`_toss_stay` is untouched.** It already leaves the platform holding the catch
  pose and installs no `go_home`, which is what makes the chain a chain.

**Acceptance.** `./run_tests.sh --full` green. A scripted three-cycle ring
session through the real FSMs asserts: the declared centre follows the ring; each
cycle's reach target is inside 80 mm of *its own* `B`; the pose index sequence is
`0,1,2,0,…` under an all-CAUGHT script and repeats the leg under a scripted miss;
a staged cycle's `throw_site_xy_mm` tracks the PREVIOUS leg's predicted park;
`REJECTED_SITE_MOVED` is absent from the healthy run; and no `go_to_pose` is
issued outside `_position_platform_for_toss`.

**Dependencies.** M1, M2.

### Phase M4: Predicted-state staging — a stage that can verify a site it has not reached

**Modified files**
* `ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py`
  (`_build_toss_cycle`'s positioning decision, `_toss_already_positioned`'s
  caller, `_predicted_chain_site_mm`)
* `ros_ws/src/jugglebot/jugglebot/toss_sequencer.py` (docstrings only — the
  commit gate is unchanged, and that is the point)
* `tools/probes/toss_pose_ring.py`, `tools/probes/cadence_rung_check.py`
* `tests/ros/`, `tests/motion/`

**Scope — one decision, re-timed.** For a **staged** ring cycle, the stage-time
positioning decision is evaluated against the **PREDICTED post-reach state**
rather than the live one:

* **site** — `_predicted_chain_site_mm(ring[pose_index(k−1)], flight)`, i.e.
  where the previous leg's deferred A→B reach will have parked the platform.
  This is the *same* call M3 already threads for the throw-site nomination
  (§ 2.4), so M4 adds no second derivation — it feeds the existing one to the
  positioning comparison as well;
* **orientation** — the previous leg's predicted **receive tilt** compared
  against this leg's **pre-tilt**, instead of the live commanded orientation.
  This is the half § 1.4 Finding 1 measures: on a two-pose ring the delta
  collapses to ~0.12 mrad, so the SKIP is honest against the state that will
  exist at the throw;
* **the backstop is already landed and unchanged.** The commit-tick
  `staged_site_ok` re-validation (`logbook/2026-08-28-displaced-chain-stale-site.md`)
  re-asks the same question against the LIVE pose at the instant the throw
  becomes irrevocable, and mints `REJECTED_SITE_MOVED` on a prediction that did
  not come true. The staged slot is discarded and the cycle rebuilds serially —
  nothing armed, nothing announced, nothing thrown.

**Why the optimism is safe, stated as the contract it rides on.** M4 makes a
scheduling decision on a state that does not exist yet, which is exactly the
thing the 2026-08-28 entry convicted. The difference is the belt: that defect
was a stale truth *believed all the way to the CAN frame* with nothing re-asking;
here the prediction is **honest-cache by construction** — it is checked at the
one instant with the property that nothing can move the platform between the
answer and the frame. § 7's non-goal bullet named the unlock as *"a staged slot
that can verify a throw site it has not yet arrived at"*, and the commit
backstop is that verification. So M4 is the non-goal cashed in, not the contract
relaxed: **the tolerance is not widened, the gate is not moved, and the only
change is which state the stage-time question is asked about.**

**Critical details.**
* **Failure mode and its cost.** A prediction that does not come true costs one
  discarded stage and a serial rebuild — the same cost the leg pays *today*,
  every cycle. So M4's worst case is M3's normal case, which is what makes it a
  cadence phase with a bounded downside rather than a risk to the ball.
* **`REJECTED_SITE_MOVED` becomes a RATE, not an absence.** Under M3 it must be
  absent from a healthy session (T-U17). Under M4 a nonzero rate is expected and
  is the honest measure of the prediction's quality; the ladder scores it
  (§ 6.3 MP-M10). A rate above ~10 % means the prediction is not earning the
  stage and M4 should be reverted for that geometry — a decision the record can
  make, which is why the counter is the acceptance instrument.
* **The moving leg is untouched.** A leg whose predicted delta still exceeds the
  tolerance — every leg of a turning ring — declines the stage exactly as before.
* **No new writer of `_t_release`, and no new commanded motion.** M4 changes an
  input to a boolean; R4's structural test (no `go_to_pose` outside
  `_position_platform_for_toss`) is re-run unchanged.

**Acceptance.** `./run_tests.sh --full` green. A two-pose ring session through
the real FSMs reaches `PHASE_STAGED` and COMMITS on its steady-state legs, at an
achieved dwell inside the 0.4170 s pipelined floor; the same session with the
prediction forced wrong refuses `REJECTED_SITE_MOVED` and rebuilds serially with
nothing announced; a turning ring's decision stream is byte-identical to M3's.

**Dependencies.** M3. (M4 is the only phase whose revert is purely a cadence
regression: the ring keeps working at the serial-with-skip floor.)

### Phase M5: Hardware validation

**Scope.** Fly § 6's ladder. Deploy is a **two-package** build (§ 9.5).

**Acceptance.** § 6's PASS rows at rungs MP-0 … MP-3 minimum, with § 6.4's stop
conditions unbreached.

**Dependencies.** M4, and § 8.1's prerequisites. (MP-0 … MP-3 and MP-5/MP-6 are
bookable from M3; **only rungs below the serial-with-skip floor — MP-4 — require
M4**, per § 6.)

### Phase M6: Close-out

**Scope.** Fold the rungs into `tests/hardware/session_cadence_ladder.md`
alongside R0–R5 and P0–P5 (one board carries all bookable rungs); record the
C-REACH-1 change in `ros_ws/docs/catch_reach_envelope.md`'s own history; write
the logbook entry in **full investigation form** (this plan clears at least
three Discussion triggers before it starts — the commissioning dwell premise
withdrawn in § 1.4, the *free-pipelining* premise withdrawn in the same section
and re-adjudicated as § 4's M4, and the accepted tradeoff in § 2.8a); hand the
sitting's corpus to
`plans/active/critical-point-ilc.md` build step 3; archive this plan.

---

## 5. Testing Plan

The rule for this plan, inherited: **every threshold gets a probe before it gets
a test.** No number below is typed into an assertion until a probe has produced
it deterministically against the pinned stack, and the confirmed recipe goes into
the test docstring and the phase's logbook (`tools/probes/README.md`).

### 5.1 Probes — first, and named

| ID | probe | question it answers | output |
|---|---|---|---|
| **P1** | `tools/probes/toss_pose_ring.py` (new, committed) | which ring geometries take the positioning skip; where the walk's fixed point is; each leg's floor | §§ 1.4 and 2.7 tables |
| **P2** | one `ros2 service call trajectory/go_to_pose` with `lean_gain: 0.0` to a ~2° pre-tilt pose | is the 2.4–2.8× planned-duration inflation really `LeanShaper`? | `planned_duration_s`, predicted 0.20–0.36 s |
| **P3** | `tools/probes/toss_loop_census.py` filtered to moving cycles | what does a moving cycle's iteration actually cost, and what dominates it? | body/period maxima + the dominant term |
| **P4** | `tools/probes/possession_replay.py --emit-fixture` (existing) | does the multi-pose verdict path reproduce a recorded sitting row for row? | a new fixture module |
| **P5** | `/tmp/probe_ring_drift.py` (one-off) | how far does a ring walk off its nominal geometry at the measured 1.3–1.5× aim gain before a runtime gate refuses? | the cycle index at first `REJECTED_DISPLACEMENT` |

The re-derivation recipe for every arithmetic number in this document:

```bash
source ~/Desktop/PDJ_venv/venv/bin/activate
python tools/probes/toss_pose_ring.py --ring 70,0:-70,0 --height 1.0 --throws 6 --lean 0.6,0.0
```

⚠ **That recipe does not work yet, and §§ 1.4 and 2.7's tables are therefore NOT
reproducible until M0 lands.** They were produced by a throwaway,
`/tmp/probe_pose_ring.py`, which is gone — `/tmp` does not survive a reboot, and
`tools/probes/README.md` says exactly this is why a reusable probe belongs under
`tools/probes/`. Promoting it is P1's whole content and it is the first item of
the first phase for that reason: until then every number in those two tables is
a **claim on the record, not a re-runnable measurement**, and no test may assert
one (the "every threshold gets a probe before it gets a test" rule above is what
holds the line). M0's acceptance is precisely that the promoted probe
regenerates both tables.

### 5.2 Unit tests (offline, no hardware)

| ID | test | pass criterion |
|---|---|---|
| **T-U1** | `test_an_empty_pose_list_is_the_v1_session_bit_for_bit` | with `catch_positions` empty, the decision stream over the whole `cadence_rung_check` grid is identical to the pre-M2 tree |
| **T-U2** | `test_a_single_element_pose_list_matches_the_legacy_encoding` | `[B]` and `catch_position = B` produce identical decision streams |
| **T-U3** | `test_a_contradicting_catch_position_is_refused_not_ignored` | `catch_positions[0] != catch_position != zero` ⇒ `REJECTED_POSE_LIST`; the zero Point is accepted as unset |
| **T-U4** | `test_the_pose_index_advances_only_on_a_caught_terminal` | R1's **fold**, over `n` in 1..5 and `num_throws` up to 20, driven by a scripted OUTCOME sequence, not a cycle count: `+1 mod n` on CAUGHT and unchanged on a survived miss / abort / rejection / reload interlude. The all-CAUGHT sequence is asserted to reduce to `(k − 1) mod n`, so the special case stays pinned without being the definition |
| **T-U5** | `test_the_accept_gate_refuses_by_leg_and_by_name` | five hand-built impossible rings each refuse with the right code **and the right leg index**; **probe P1 first** |
| **T-U6** | `test_the_two_pose_ring_reaches_a_skipping_fixed_point` | `|Δθ|` at hop ≥ 2 is below `_TOSS_ALREADY_THERE_TOL_RAD`, read from the constant so a tolerance change moves the test |
| **T-U7** | `test_a_turning_ring_moves_on_every_cycle` | the triangle's `|Δθ|` exceeds the tolerance at every hop |
| **T-U8** | `test_the_moving_leg_floor_is_the_moving_budget_plus_the_handoff` | § 2.7's moving column reproduces from the shipped `required_dwell_s`; **probe P1 first** |
| **T-U9** | `test_the_session_floor_excludes_only_granted_legs` | a two-pose ring's floor is the pipelined one; a turning ring's is the moving one; a ring with one moving leg among skips takes the pipelined floor and the moving leg's grant |
| **T-U10** | `test_the_pretilt_move_is_planned_without_lean_on_tier_8b` | the `GoToPose` request carries `lean_gain == 0.0` iff the release is tilted; 8a carries `-1.0` |
| **T-U11** | `test_a_lean_off_move_that_translates_is_loud` | the >12.1 mm guard logs and does not silently proceed |
| **T-U12** | `test_the_position_absorb_only_ever_delays` | over a swept `planned_duration_s`, `_t_release` is non-decreasing and never advances |
| **T-U13** | `test_the_drift_guard_fires_when_the_declaration_is_lost` | suppress the per-cycle declaration and cycle 2 refuses `REJECTED_REACH_CENTER_DRIFT` **pre-throw**; with it, drift is 0 |
| **T-U14** | `test_a_redundant_raise_never_installs_a_graceful_stop` | ordered call log, with a move in flight |
| **T-U15** | `test_the_census_never_feeds_a_budget` (**existing, must keep passing**) | `position_overrun_s` is measured by the FSM's own clock, never plumbed from `LoopPeriodCensus` |
| **T-U16** | `test_a_staged_ring_cycle_nominates_the_PREVIOUS_legs_predicted_park` | on a staged ring cycle `k`, `seq.throw_site_xy_mm` tracks `_predicted_chain_site_mm(ring[pose_index(k−1)], flight)` — **not** the cycle's own `catch_pose`. Drive `n = 2` and `n = 3`; at `n = 2` the own-`B` feed is a full ring leg away, so the test discriminates rather than aliasing (§ 2.4's threading row) |
| **T-U17** | `test_a_healthy_ring_session_never_mints_rejected_site_moved` | over a scripted healthy multi-cycle ring session, `REJECTED_SITE_MOVED` is **ABSENT** from the abandoned-stage reasons. Its per-cycle *presence* is the B2 threading bug's signature — the commit belt catching what the nomination got wrong — so an absence assertion is the only one that fails when the threading regresses |
| **T-U18** | `test_a_survived_miss_holds_the_pose_and_commands_no_go_home` | Q-2's miss path: the terminal issues **no** `go_home` (ordered call log, not a flag), `catch/armed` and the session latch are left in the state § 8.2 Q-2's choreography names, and the NEXT cycle's `A` is the held pose rather than home |

### 5.3 Property tests on the ring

Extending `tests/ros/test_toss_pipeline_properties.py`, whose per-tick-gap
strategy (`_TICK_GAPS`) is the one that found the 2026-08-28 late-tick class.

| ID | property | strategy |
|---|---|---|
| **T-P1** | **The ring is walked in order, and each lap is a PREFIX of the ring with repeats permitted at the failure boundary**: over any `(n, num_throws, terminal sequence)` the pose sequence is `B₀ B₀… B₁ B₁… B₂ …` — never out of order, never skipping a leg, and a repeat occurs only where the previous terminal was not CAUGHT (R1's fold). The pre-Q-2 phrasing ("no repetition inside a lap") is refuted by the owner's advance-on-CAUGHT decision and is replaced, not relaxed | random ring lengths, throw counts, and terminal outcomes |
| **T-P2** | **Every cycle's reach target is inside the envelope about its OWN B** | random rings inside the accept gate's admissible set |
| **T-P3** | **No cycle is dispatched whose declaration was not applied** (R2) | random declaration losses injected |
| **T-P4** | **The schedule stays monotone with the absorb in the writer set** (T-P5's successor) | per-tick gaps × swept `planned_duration_s` |
| **T-P5** | **An accepted ring always flies**: any `(ring, dwell, height)` the gate admits produces no `REJECTED_*` from a per-cycle gate on a healthy plant | the `accept-implies-flies` property `cadence_rung_check` already enforces for cadence |

### 5.4 Replay-driven acceptance

| ID | test | pass criterion |
|---|---|---|
| **T-R1** | `test_the_multipose_verdict_census_is_the_cup_census_exactly` | replay a recorded sitting through the multi-pose path; every verdict is unchanged. The ring must not move a single possession verdict — it has no business touching possession semantics |
| **T-R2** | `test_a_multipose_replay_matches_its_own_log` | after M5's sitting, pin the per-cycle `pose_index`, achieved dwell, `position_overrun_s`, the staged/declined split and landing error from the sitting |
| **T-R3** | the existing pre-audit ladder regressions | unchanged — the regressions stay findable |

### 5.5 Integration tests (real system, safe conditions)

| ID | test | pass criterion |
|---|---|---|
| **T-I1** | the per-cycle declaration through a **real** `TrajectoryNode` under a standing latch | `_catch_envelope_center` equals each cycle's own `B`, across a chained run — the captured **value**, never publish ordering (B3's CS-4 false-green lesson) |
| **T-I2** | a three-cycle ring through the real `CatchCoordinatorNode` with S6's holds standing | no auto-prime fires; no announcement pre-tilt installs motion |
| **T-I3** | a ring session's `go_to_pose` requests captured at the service boundary | every tier-8b request carries `lean_gain == 0.0`; every response's `planned_duration_s` is inside the honest budget |

### 5.6 Regression tests

| ID | test | pass criterion |
|---|---|---|
| **T-G1** | `cadence_rung_check --grid`, `catch_positions` empty | byte-identical decisions to the pre-M2 tree |
| **T-G2** | the published R0–R5 and P0–P5 ladders | unchanged |
| **T-G3** | the single `Toss` action and the Reload path | no decision changes; neither has a ring, and the reload declares no centre at all |
| **T-G4** | the reload's `REJECTED_NOT_CENTERED` and `_RELOAD_CENTERED_TOL_MM` | untouched — the two 66.53 mm constants stay separate names answering separate questions, and M3 must not merge them |

### 5.7 The gate

`./run_tests.sh --full` before **every** commit in this plan and before the M5
sitting — the rule's cases (a) before any hardware sitting and (b) at plan-phase
closure both apply. Report the (date, command, result) triple in each commit
message and each logbook entry.

---

## 6. Hardware validation ladder — the runbook

A runbook sketch; M6 lands it as rungs in
`tests/hardware/session_cadence_ladder.md` alongside R0–R5 and P0–P5.

⚠ **Which rungs are bookable at which phase.** A ring leg runs
**serial-with-skip** until M4 lands (§ 1.4), so the governing floor for a
two-pose steady-state leg is **0.5370 s** at `h = 1.00 m`, not 0.4170 s. **An MP
rung whose commanded dwell sits below the serial-with-skip floor is bookable only
after M4 lands** — that is MP-4 (0.45 s) and nothing else on the present ladder.
MP-3's 0.55 s clears the serial floor by 13 ms and is bookable from M3, though
its clearance is a *razor* until M4 restores the 133 ms the table below credits
it with. Book a sub-floor rung before M4 and the cycle does not run slowly — it
terminalises, because the floor is a refusal, not a target.

### 6.1 Standing settings

```
stop_on_miss   : true      one miss ends the sitting — EXCEPT rung MP-7, which
                           exercises Q-2's survived-miss path on purpose
on_empty_cup   : STOP      never RELOAD on these rungs. Q-2 IS decided (the index
                           repeats the failed leg; a survived miss holds the pose
                           rather than going home), but the reload interlude adds
                           a second discontinuity to the ring and is out of scope
                           for the first ladder
num_throws     : 4         first run of any rung
catch_vel_scale: 0.0       (⇒ the 0.9 config default)
throw_height_m : 1.00      unless the rung says otherwise
```

Record `uptime_ms`, the per-cycle `dwell_s`, `pose_index`, `position_overrun_s`,
the toss-record JSONL and the bag. **Score the miner, not the console.**

**Read § 1.6 before booking anything.** The aim map has never been captured, so a
142 mm ring leg is expected to land 43–71 mm past its `B`. These rungs measure
that; they do not try to beat it. A low catch rate on MP-1 and up is the
**expected** result and is not a stop condition — a *non-repeatable* landing
error is.

### 6.2 The rungs

Floors from § 2.7 at `h = 1.00 m`: pipelined leg **0.4170 s** (needs M4),
serial-with-skip leg **0.5370 s** (M3), moving leg **1.0570 s**. The
"needs" column names the phase a rung's governing floor requires.

| rung | ring | leg length | dwell | governing floor | clearance | needs | what it is for |
|---|---|---|---|---|---|---|---|
| **MP-0** | 2-pose (±40, 0) | 80 mm | 1.10 s | 1.057 (moving, transient) | 43 ms | M3 | the first ring ever flown; short legs so the aim error is inside the cup |
| **MP-1** | 2-pose (±70, 0) | 142 mm | 1.10 s | 1.057 | 43 ms | M3 | full-size legs, near the 150 mm cap |
| **MP-2** | 2-pose (±70, 0) | 142 mm | 0.76 s | 0.537 (serial+skip) / 0.417 (pipelined, under M4) | 223 ms / 343 ms | M3 | the owner's number. Cycles 1–2 stretch on their grant; cycles 3+ run at 0.76 |
| **MP-3** | 2-pose (±70, 0) | 142 mm | 0.55 s | 0.537 / 0.417 | **13 ms** / 133 ms | M3 (razor) | the milestone for a two-pose ring ⭐. **Fly it after M4 if the 13 ms clearance is judged too thin** — it is one loop period, not three |
| **MP-4** | 2-pose (±70, 0) | 142 mm | 0.45 s | 0.417 | 33 ms | **M4 — below the 0.537 s serial floor, so it is REFUSED without it** | the pipelined edge — the same razor class the runbook flags at P4; book last |
| **MP-5** | 3-pose triangle `r = 70` | 122 mm | 1.15 s | 1.057 (moving, **every** cycle) | 93 ms | M3 (M4 is a no-op here) | the first TURNING ring ⭐ |
| **MP-6** | 3-pose triangle `r = 70`, `h = 1.30` | 122 mm | 1.15 s | 1.034 | 116 ms | M3 | the turning ring at the longer flight |
| **MP-7** | 2-pose (±70, 0), **`stop_on_miss: false`**, `num_throws: 6` | 142 mm | 1.10 s | 1.057 | 43 ms | M3 | **the survived-MISS rung, and the ball is dropped ON PURPOSE.** Q-2's directive — *"a survived MISS ought not go_home"* — is choreography that only exists on a path a healthy sitting never takes, so it is flown deliberately: deflect or decline one catch by hand at a cycle the operator picks, then watch that the platform **HOLDS the pose** (no `go_home` traverse), that the leg **REPEATS** (`pose_index` unchanged in the record), and that the arming state through the hold is the one § 8.2 Q-2's obligation 2 names. ⚠ Read that obligation before booking: the hold happens with a ball on the floor and the session latch standing, so the catch envelope is LIVE and unfed. Book it **after MP-1 and before any rung below 1.0 s**, and only with the escape of § 6.4's cancel note confirmed |

**MP-0 is not optional.** It is the first sitting in which the envelope centre
moves between cycles, and its job is to find out whether that is a surprise at a
dwell with 43 ms of clearance and legs short enough that the uncalibrated aim
error stays inside the cup — before anything is near the cadence edge.

**MP-3 and MP-5 are the milestone**, and they are two different claims: a
two-pose ring holds a 0.55 s dwell because it never re-orients, and a turning
ring holds 1.15 s because it re-orients every cycle. Both numbers get published;
collapsing them into one is how a cadence claim becomes a cadence the machine
cannot make.

### 6.3 What each rung measures

| row | measurand | source | why |
|---|---|---|---|
| **MP-M1** | `pose_index` and the commanded `B` per cycle | toss record | R1, tested. The ring is walked in order or it is not |
| **MP-M2** | the POSITIONING move's `planned_duration_s` and `position_overrun_s` per cycle | toss record | § 1.4 Findings 3 and 4, tested against the machine. A `planned_duration_s` near 0.55 s means the lean change did not deploy |
| **MP-M3** | achieved dwell per cycle, split transient vs steady | toss record | scored against the commanded dwell **and** against the leg's own floor. Two claims, both published |
| **MP-M4** | the positioning SKIP verdict per cycle vs M0/P1's prediction | toss record + probe | R3's accept-time classification is a prediction; the disagreement rate is the honest measure of how good it is |
| **MP-M5** | the cup, by eye, at the receive tilt and through the re-orient | the operator | **the owner's decision 1 rides on this row.** Full receive tilt was chosen for seating quality; this is where that is confirmed or withdrawn |
| **MP-M6** | landing error per pose, signed, in the ring frame | miner | is the error systematic per `B` (a map/ILC job) or random (a plant job)? This row IS the aim corpus |
| **MP-M7** | grep the log for *"catch target … exceeds the … reach envelope"* | `/rosout` | **R2's acceptance is an ABSENCE.** Zero lines. Any line is a declaration that did not land |
| **MP-M8** | grep the log for *"catch latch armed mid-move"* | `/rosout` | § 2.5's acceptance is an ABSENCE. Zero lines. Any line means the redundant raise reached the graceful-stop branch |
| **MP-M9** | `dispatch → catch-stroke-end` gap per cycle | hand telemetry | the C-HAND-1 no-overlap margin, inherited verbatim from R4/R5/P-rungs |
| **MP-M10** | the **staged/declined split** per cycle, and the `REJECTED_SITE_MOVED` rate | toss record (`staged_at_s`, `staged_discarded_reason`) | **M4's acceptance instrument, and it is a RATE not an absence** (unlike MP-M7/MP-M8). Before M4 every ring leg reads `POSITIONING_MOVE` and the split is 0 % staged — that is the serial-with-skip baseline. After M4 a steady-state leg must stage; a `REJECTED_SITE_MOVED` rate above ~10 % says the prediction is not earning the stage, and the honest response is to revert M4 for that geometry rather than to widen a tolerance |
| **MP-M11** | on MP-7 only: `pose_index` across the miss, the `go_home` call log, and `catch/armed` through the hold | toss record + `/rosout` | Q-2's choreography, on the machine. The index must NOT advance, no `go_home` may be commanded, and the arming state must be the one § 8.2 Q-2 obligation 2 states — a mismatch here is a live catch envelope nobody wrote down |

### 6.4 Stop conditions

Stop the sitting, do not step down, and debrief on any of:

* any **negative** `dispatch → catch-stroke-end` gap (MP-M9);
* any *"catch target … exceeds the … reach envelope"* line (MP-M7) — a per-cycle
  declaration did not land, and the refusal arrived mid-flight, which is the
  exact failure C-REACH-1 exists to prevent;
* any *"catch latch armed mid-move"* line (MP-M8);
* any `REJECTED_REACH_CENTER_DRIFT` — the declaration path is broken, not the
  ring;
* any `ABORTED_CANT_MAKE_RELEASE` on a moving cycle — the budget is still lying
  and § 2.8's absorb did not cover it. A design finding, not a tuning one;
* any `REJECTED_DISPLACEMENT` or `REJECTED_WORKSPACE` mid-session — **the ring
  has walked off its nominal geometry.** The refusal is correct and fail-closed
  (nothing was thrown), but it is an **aim** finding: record the cycle index, the
  live site and the nominal one, and hand it to the ILC arc rather than widening
  a cap;
* any commanded platform motion between a verdict and the next cycle's
  POSITIONING (R4);
* any HAND row outside `tests/hardware/session_anomaly_fixes.md` § PASS/ABORT.

**`REJECTED_SITE_MOVED` is deliberately NOT on that list, and the operator needs
to know which reading applies.** *Before M4* it should never appear at all —
every ring leg declines its stage as `POSITIONING_MOVE` and never reaches a
commit gate — so one occurrence is a finding worth debriefing but not a stop
(nothing was armed, announced or thrown; the cycle rebuilt serially). *After M4*
it is the expected residual of an optimistic stage and is scored as a **rate**
by MP-M10, not as an event. ⚠ In **both** readings it can equally mean
`/trajectory/commanded_pose` went stale or absent — the gate is fail-closed and
the two look identical on the wire — so check that topic is publishing before
concluding something moved the platform. It is not in the bag's recorded topic
set (§ 7), so this is a live check, not a post-mortem one.

**A cancel is deferred at these dwells and the stop button is not the E-STOP.**
Say it out loud before arming. ⚠ The 2026-08-28 entry's open follow-up 1 stands:
**if the GUI has no cancel button, the only escape from a session-level wedge is
the `_SESSION_STALL_S` watchdog (7.8 s) or the session ceiling.** Confirm the
escape before the first ring sitting.

---

## 7. Non-goals — stated so they are not drifted into

* **The constant-beat fast path.** Catching at (or near) the next throw's
  pre-tilt so the re-orient disappears — the P-4 substitution widened from its
  present ±1° aim authority to ~2–4° — is the owner-deferred future lever
  (§ 1.3 decision 2). It would move a turning ring from the moving floor
  (1.057 s at `h = 1.0`) to the pipelined one (0.417 s), which is a **2.5×
  DWELL-FLOOR ratio — and 1.48× in throws/min** (30.6 → 45.4), because the
  period carries a fixed flight term the floor does not. It is still by far the
  largest single lever this plan leaves on the table; it is not a 2.5× cadence
  gain, and quoting it as one is how a runbook promises a rate the machine
  cannot make. **Named unlock:** MP-M5 showing that seating quality survives a receive
  tilt several degrees off the true arrival direction, plus a re-derivation of
  `_toss_reach_quat`'s substitution window against `toss_cal.TOTAL_MAX_RAD`.
  A two-pose ring does not need it — it already re-orients by nothing.
* **Pipelining a cycle that genuinely MOVES** — a turning ring's every leg, and
  any displaced leg whose predicted re-orient still exceeds
  `_TOSS_ALREADY_THERE_TOL_RAD`. The skip-only staging rule
  (`toss-pipelined-preamble.md` § 2.4.1) makes a mover fall back to the serial
  path. That is correct and is not a regression; it simply does not get faster.
  **Named unlock:** § 7's constant-beat fast path (first bullet), which is what
  removes the move rather than pipelining around it.

  ⚠ **The other half of this bullet is no longer a non-goal — it became phase
  M4.** The draft listed "pipelining a *displaced* cycle" here with the unlock
  *"a staged slot that can verify a throw site it has not yet arrived at"*. The
  2026-08-28 audit showed that unlock is not future work at all: the commit-tick
  `staged_site_ok` re-validation had already landed, and it is exactly the
  verification the bullet was waiting for. So the design moved **into § 4's
  M4** — predicted-state staging, with the commit backstop as what makes the
  optimism safe — rather than staying parked as an unlock nobody had costed.
* **Unblocking the loop from the positioning service round trip.** The measured
  0.16–0.54 s body overruns come from a synchronous `go_to_pose` inside the FSM
  tick, and the determinism directive is explicit that control loops do no
  blocking I/O. Multi-pose makes every displaced cycle hit it, so this is a
  **strongly recommended sibling** rather than an unrelated one — but it is a
  separate change with its own evidence and its own logbook, and this plan
  charges the cost honestly instead of removing it.
* **Capturing the aim map, or unparking ILC.** § 1.6. This plan produces the
  corpus; `plans/active/critical-point-ilc.md` consumes it.
* **The mocap NaN-misalignment anomaly**, **`/trajectory/commanded_pose` missing
  from the bag recorder** (`jugglebot_launch.py`'s `record:=true` topic list
  carries `/trajectory/commanded_position` but not its 2026-08-23 sibling, so a
  bag cannot answer what the commanded *orientation* was — which is exactly the
  quantity the positioning skip turns on), and **the QTM cone-labelling fix for
  `h ≥ 1.3`**. Each is real, each touches a sitting this plan books, and none is
  absorbed here. The recorder omission is the cheapest of the three and is the
  one most likely to cost a re-fly.
* **Relaxing any floor or any envelope.** `reach_envelope_mm`,
  `_TOSS_SESSION_REACH_DRIFT_TOL_MM`,
  `MAX_TILT_DEG`, `hand_floor_dwell_s`, `handoff_margin_s` and
  `CATCH_CONFIRM_WINDOW_S` are unchanged. The one budget that **rises** is
  `pre_dispatch_budget_s(True)`, and it rises because it was under-charging.
* **Two balls.** Nothing here puts a second ball in the air; S1′'s whole safety
  argument rests on exactly one existing at a time.

---

## 8. Prerequisites and open questions

### 8.1 Prerequisites

| # | prerequisite | hard? | why |
|---|---|---|---|
| **P-1** | **The stale-site fix has landed** (staged nomination from the chain-site prediction + commit-tick pose re-validation, discard-to-serial on mismatch) | **hard** | it is what makes the site nomination honest, and every hop of § 2.6's walk is a site nomination. Without it the ring's `A` can be a pose the platform has already left |
| **P-2** | **Two-package deploy**: `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot` | **hard** | M2 changes `TossContinuous.action`. A `jugglebot`-only build raises `ImportError` at module scope in `reload_coordinator_node` and takes `Reload`, `Toss` and `TossContinuous` down together — the documented split-build failure. `ros2 action list \| grep -c jugglebot/toss` (expect 2) is what detects it |
| **P-3** | **M0/P2 confirms the lean attribution on the machine** | **hard for the turning rungs** | if `planned_duration_s` does not collapse at `lean_gain = 0.0`, § 2.7's moving column reverts to 1.31–1.36 s and MP-5/MP-6 re-cut before they are booked |
| **P-4** | **A cancel path exists** (GUI button, or an accepted watchdog-only escape) | **hard** | the 2026-08-28 entry's open follow-up 1. A ring session is longer and more novel than a fixed-B one |
| **P-5** | `plans/active/critical-point-ilc.md` is **not** a prerequisite — it is the consumer | — | recorded here so the sequencing is unambiguous: multi-pose lands first, its sittings are the aim-excited corpus, and the ILC unpark follows |
| **P-6** | the drive-restoration state of 2026-08-18 (`b084f98`) holds | **hard** | pre-2026-08-18 braking-clamp behaviour invalidates every catch-tail number the floors are built on |

### 8.2 Open questions — decisions required before the phase named

**Status 2026-08-28: Q-1, Q-2, Q-4, Q-5 DECIDED by the owner (same day); Q-3
is adjudicated by the orchestrator at M1. The original texts are kept below;
the resolutions:**

* **Q-1 → the TWO-POSE ring is the milestone** (it is the two-balls-in-one-hand
  pattern). The full feature lands for any ring length; turning rings ship at
  their honest 1.057 s floor, documented, improvable later by the lean/budget
  work.

  ⚠ **RE-PUT TO THE OWNER 2026-08-28, after the audit refuted the premise this
  resolution rested on.** The first resolution said the two-pose ring "pipelines
  at the 0.4170 s floor" — free, out of the landed substrate. It does not: the
  stage-time positioning decision compares the nominated site against the LIVE
  pose while the platform is still at the previous leg, so **every** ring leg
  declines the stage and runs serial-with-skip at 0.5370 s (§ 1.4). That makes
  the milestone claim a claim about work nobody had scoped. Three responses were
  put back: (a) publish the serial-with-skip floor and drop MP-4; (b) **own the
  redesign** — a phase that evaluates the stage-time decision against the
  predicted post-reach state; (c) defer the two-pose cadence claim entirely.
  **The owner chose (b)**, and it is § 4's **phase M4**. Q-1's original
  turning-ring adjudication below is unaffected — M4 is worth nothing on a ring
  that genuinely moves.
* **Q-2 → the ring index REPEATS the failed leg** (advances only on CAUGHT).
  **PLUS an owner directive that extends this plan's scope: a survived MISS
  must NOT `go_home`.** Verbatim: *"a survived MISS ought not go_home. It
  should pause until the ball is settled, then resume."* The miss ladder on
  the survived path becomes: hold the current pose, safe the hand IN PLACE
  (no retract-to-home traverse), wait for a ball-settled criterion, then
  resume the chain from the held pose (a following reload interlude does its
  own recentre — which also removes today's double-move). Design obligations
  this creates, owned by M3: the settle criterion (mocap-stationary vs a
  floor-timer — probe first), the hand-safing choreography without the
  SAFE_ABORT ladder's `go_home` rung, S7's drain semantics on a path that no
  longer reaches `_go_home`, and what `DEFAULT_SESSION_MISS_CLEANUP_S`
  becomes when the cleanup no longer includes a traverse. Each is a
  control-choreography change and gets the full walk in M3's design notes.

  **Five further obligations, added 2026-08-28 by audit. They are not
  refinements of the four above — each is a place where "do not `go_home`"
  changes something the plan had already written down as settled:**

  1. **`toss_sequencer._terminal_action` must gain a new terminal action.** The
     survived-miss terminal currently returns `ACTION_SAFE_ABORT`, whose ladder
     *contains* the `go_home` rung. Leaving it in place and suppressing the
     traverse downstream would put two authorities on one decision; the honest
     change is a distinct terminal action for "hold the pose, safe the hand in
     place". `toss_sequencer.py` therefore joins **M3's modified-files list and
     § 9.3's file table** (it was previously listed for the budget and absorb
     only).
  2. **The S6 ARMING half, which nothing in the plan had stated.** A miss path
     that never reaches `_go_home` also never reaches the teardown that lowers
     the latch — so `catch/armed` and the session latch stay **RAISED** across
     the hold. That is a **live catch envelope with the ball on the floor**,
     for however long the settle criterion takes. The choreography must state
     explicitly (a) what the latch's state is during the hold, (b) what the
     CCN's `prime_hold` / `pretilt_hold` are during it, and (c) what re-entry
     does — resume under the standing latch, or lower and re-raise. Whichever
     is chosen, it is written down before it is coded: this is the one
     obligation on the list that can put the hand in motion with nobody
     expecting it.
  3. **§ 3's M3 row names the miss-path scope extension** (done — see the phase
     table).
  4. **A § 5.2 test row** pinning the behaviour offline: **T-U18** — a survived
     miss commands **no** `go_home` (ordered call log, not a flag), leaves
     `catch/armed` and the holds in the state obligation 2 states, and the next
     cycle's `A` is the **held pose**.
  5. **§ 9.4's ordering note is wrong for this path and is corrected there.**
     It says `_disarm_session` runs from `_drain_pipeline_and_disarm`, *"which
     every `go_home` path calls first (S7)"* — true, and now insufficient,
     because the survived-miss path is a terminal that is **not** a `go_home`
     path at all. See § 9.4.

  A **hardware rung** goes with them: § 6.2's **MP-7**, flown with
  `stop_on_miss: false` and a ball dropped on purpose, because this choreography
  exists only on a path a healthy sitting never takes.
* **Q-4 → cycle 1 throws home → `catch_positions[0]`** (no pre-positioning
  move; the first cycle is the ring's entry leg).
* **Q-5 → lean-off approved for the tilt-in-place move class** (≤ ~2 mm
  translation), scoped to that class only, bench-watched at first flight;
  every other move keeps `lean_gain 0.6`.

* **Q-1 — the honest floor for a turning ring is ~2× the commissioning premise.
  Decision required before M5 (the hardware ladder; this said "M4" before the
  M4 phase was inserted).** § 1.4 found the premise splits: a two-pose ring
  meets ~0.55 s comfortably, and a turning ring's floor is 1.03–1.06 s. Three
  responses are available and they are not exclusive: (a) accept it and fly
  MP-5/MP-6 at 1.15 s, which is what § 6.2 assumes; (b) fund the constant-beat
  fast path (§ 7), worth a 2.5× dwell-floor ratio — **1.48× in throws/min** —
  on a turning ring and nothing on a two-pose one;
  (c) restrict the feature to two-pose rings for now, which costs nothing to
  implement because the accept gate refuses by leg anyway. The recommendation is
  **(a) now, (b) next**, because a turning ring at 1.15 s is still a capability
  the machine does not have, and because (b)'s unlock (MP-M5) is a row on (a)'s
  own ladder.
* **Q-2 — what a reload interlude and a survived MISS do to the ring index.
  Decision required before M3.** Both `go_home`, so the platform is at the
  workspace centre afterwards and the next cycle's `A` is home rather than
  `B_{k−1}`. Three semantics: resume at the same index (the ring keeps its
  phase; one hop is a home-entry hop); restart at index 0 (the ring restarts
  cleanly; the pose sequence no longer matches the cycle index); or refuse to
  resume. The recommendation is **resume at the same index**, with the accept
  gate additionally validating the home-entry hop for every `B` in the ring
  whenever a resumption is possible — because the alternative silently changes
  which pose a cycle index means, and the record's `pose_index` would then be the
  only way to reconstruct a sitting.
* **Q-3 — the applied-centre echo, or C-REACH-1 § 5 residual 1.
  Decision required before M1.** R2 adds `catch/reach_center_applied` so a lost
  declaration refuses pre-throw. Without it the machine keeps the shipped
  behaviour — a lost declaration rejects the reach `WORKSPACE` mid-flight and the
  ball lands on the floor, loudly. The recommendation is **the echo**, on
  C-REACH-1 § 2's own argument that a gate which can only fire after the
  irreversible commitment is choosing the moment of failure; and because the
  exposure changes from once-per-session to once-per-cycle, which is the altitude
  change that turns an accepted residual into a class.
* **Q-4 — does cycle 1 throw to `catch_positions[0]`, or pre-position to the last
  pose first? Decision required before M2.** § 2.2 assumes the first, so
  `catch_positions[0]` is the first *landing* and the ring's steady state is
  reached from cycle 2. The alternative pre-positions to `catch_positions[n−1]`
  so cycle 1 is already a full ring leg and every cycle is identical. The
  recommendation is **the first**: it commands no motion before the first throw,
  it matches the operator's phrasing (*"throw from the current site toward
  Bᵢ"*), and the transient is two cycles that the grant already absorbs. The
  cost is that the ladder's first two cycles are not comparable to the rest,
  which § 6.3 row MP-M3 already splits out.
* **Q-5 — is the lean-off pre-tilt move acceptable with a ball seated?
  Decision required before M2.** § 2.8a's argument is that lean-off commands
  strictly less motion and strictly less leg jerk, and that the un-compensated
  lateral acceleration is 297 mm/s² — 1.74° of equivalent tilt against a cup the
  machine catches into at up to 12°. The recommendation is **yes**, with MP-M5
  as the by-eye check and a one-line rollback (restore the `-1.0` defer default)
  if the ball is seen to unseat. Physical intuition that disagrees with this
  framing is load-bearing signal and should be said out loud.

---

## 9. Notes for Collaborators

### 9.1 Safety-critical invariants that must be preserved exactly

| invariant | where | consequence of getting it wrong |
|---|---|---|
| **A redundant `arm_catch(True)` installs no graceful stop and resets no freeze** | `trajectory_node._svc_arm_catch` | the per-cycle re-capture silences the POSITIONING move it is arming for — once per cycle |
| **A redundant raise with nothing pending is a pure no-op** | same | the commanded-pose fallback would re-centre the envelope mid-session on wherever the platform is, which is C-REACH-1 § 7's forbidden mid-catch re-centring |
| **C-REACH-1: the envelope bounds UNREQUESTED excursion; requested reach is gated pre-throw** | `_on_dynamic_target` + the toss FSM's displacement gate | raising the envelope to "buy reach" silently loosens the drift bound by the same amount |
| **`ball_seated` stays a hard gate on any release, read on the dispatch tick** | `toss_sequencer._step_committing` / `_step_checking` | a full-speed empty stroke with the hand ascending from an unverified position |
| **The throw dispatch is single-shot and never retried** | `_step_throwing`, `_dispatch_toss_throw` | a re-dispatch replaces a live stroke and post-release clobbers the armed catch stroke on the last-writer-wins queue |
| **The hand ladders and `_MAX_ARM_DISPATCHES` are retained defence in depth** | `_prime_hand_with_retries`, `_retract_hand_with_retries` | see `project_reload_action_catch_latch` — never blind-re-dispatch a hand move |
| **All motion is profiled; never a step position change** | `trajectory_node` / `planner` / `feasibility.validate` | this plan commands no new motion primitive; it changes one move's shaper and one move's frequency |
| **The Teensy-side `MAX_DEVIATION` guard is the leg-path safety authority** | can-bridge firmware | nothing on the Jetson is in that loop, and nothing here enters it |
| **S2: the session commands no motion of its own** | R4 | the re-orient is POSITIONING's, charged to a cycle, or it is an unbudgeted move nobody accounts for |
| **The two 66.53 mm constants stay separate names** | `_TOSS_SESSION_REACH_DRIFT_TOL_MM`, `_RELOAD_CENTERED_TOL_MM` | they answer different questions and merging them couples the reload's centring rule to the toss's ring |

### 9.2 Architecture decisions and their root causes

* **The ring is walked at accept, not discovered at runtime**, because the
  failure it prevents is a cycle refused *after* the previous ball was thrown and
  caught. The single-hop `REJECTED_CHAIN_UNREACHABLE` gate made that argument
  first; the walk makes it for all hops.

  ⚠ **THE CODE HAS TO BE RE-MINTED, ON A NEW PREMISE (2026-08-29).** That gate
  was deleted along with the lateral planning box it was keyed on — with the box
  gone a chained cycle had no A-side refusal left to pre-empt, and re-keying it
  on the reach bound would have produced a gate that can never fire (the chained
  residual is ~2 % of |B|, under 3.2 mm, against a bound of 83 mm or more that
  cycle 1 has already cleared). This plan's use is DIFFERENT and survives that
  reasoning: here the code refuses a hop whose park cannot be **predicted at
  all** (`_predicted_chain_site_mm` returns `None` — an infeasible aim or a
  catch-policy refusal at that `A`), which is a fail-closed condition rather
  than a threshold. Re-introduce it on that premise only; do not resurrect a
  box.
* **The declaration is per cycle because the envelope's lifetime is scoped to
  the raise it feeds.** That is the root cause B3 identified when it hoisted the
  declaration to session scope, and it is the same root cause that says a
  per-cycle `B` needs a per-cycle raise. The redundant-raise capture is not a
  loophole in the contract; it is the contract applied to a coordinator that now
  has more than one catch point.
* **The lean comes off because the budget's premise has to be made true, not
  inflated.** `pre_dispatch_budget_s(True)` charges the planner's
  `min_move_duration_s`, which is honest arithmetic about a move that is not
  lean-shaped and a lie about one that is. Raising the budget to cover the
  shaped move would pay 0.28 s of real lead per cycle to preserve a shaper doing
  nothing useful on two millimetres.
* **Slip rather than abort, again.** `ABORTED_CANT_MAKE_RELEASE` names the throw
  budget; a positioning move that planned longer than the budget assumed is a
  *planner* fact. The 2026-08-28 fix wave made exactly this argument for the
  commit tick and it transfers verbatim: nothing is armed, no announcement has
  gone out, and the condition resolves by waiting.
* **The floor is per leg because a session is no longer homogeneous.** One number
  was right while every cycle was the same cycle. A ring's legs differ in whether
  they command a move, which is the single largest term in the floor, so one
  number would either refuse rings that fly or admit rings that abort.

### 9.3 Files affected

| file | action | phase |
|---|---|---|
| `ros_ws/docs/catch_reach_envelope.md` | modified (§ 3 Capture, § 4, § 5 residuals 1 and 5) | M1 |
| `ros_ws/src/jugglebot/jugglebot/trajectory_node.py` | modified (`_svc_arm_catch`, the applied echo) | M1 |
| `ros_ws/src/jugglebot_interfaces/action/TossContinuous.action` | modified (**field added** — interfaces rebuild) | M2 |
| `ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py` | modified | M2, M3, M4 |
| `ros_ws/src/jugglebot/jugglebot/toss_sequencer.py` | modified (budget, absorb; **`_terminal_action` — the survived-miss terminal, § 8.2 Q-2 obligation 1**; M4 touches docstrings only) | M2, M3, M4 |
| `ros_ws/src/jugglebot/jugglebot/toss_session.py` | modified (per-leg floor, R1–R4) | M2, M3 |
| `ros_ws/src/jugglebot/jugglebot/toss_record.py` | modified (additive fields) | M3 |
| `config/hardware_config.yaml` + `config/generated/*` | modified | M2 |
| `tools/probes/toss_pose_ring.py` | **created** | M0 |
| `tools/probes/cadence_rung_check.py`, `possession_replay.py`, `toss_loop_census.py` | modified | M0, M2, M4 |
| `tests/ros/test_trajectory_node.py`, `test_toss_integration.py`, `test_toss_coordinator.py`, `test_toss_continuous_node.py`, `test_toss_sequencer.py`, `test_toss_session.py`, `test_toss_pipeline_properties.py` | modified | M1–M4 |
| `tests/motion/test_toss_record.py`, `test_cadence_rung_check.py` | modified | M2, M3, M4 |
| `tests/hardware/session_cadence_ladder.md` | modified (MP rungs) | M6 |

### 9.4 Startup / shutdown ordering

The session still arms once and disarms once (S6): `_arm_session_declare` on
cycle 1's verified-arrival tick, `_arm_session` on its PREPARE tick,
`_disarm_session` from `_drain_pipeline_and_disarm`, which every `go_home` path
calls first (S7). What M3 adds is a **per-cycle declaration + redundant raise on
those same two ticks for cycles ≥ 2**, which lower nothing and therefore cannot
re-open the arm-mid-move seam S6 closed by construction. The teardown order is
untouched: `catch/armed` False → the latch → `prime_hold` → `pretilt_hold`.

⚠ **CORRECTION for the survived-miss path (§ 8.2 Q-2 obligation 5).** *"every
`go_home` path calls `_drain_pipeline_and_disarm` first"* is true and, from M3
onward, **no longer covers every terminal**: Q-2's survived miss is a terminal
that **is not a `go_home` path**, so it reaches neither the drain nor the
disarm. Stated plainly, because it is a hand-motion fact and not a bookkeeping
one: **the catch latch and the CCN holds are STILL RAISED through the
ball-settled hold, with the ball on the floor and the envelope live.** M3 must
therefore say, in writing and before the code, what the latch and both holds are
during the hold and what re-entry does with them — resume under the standing
latch, or lower and re-raise — and MP-7/MP-M11 is where that is checked on the
machine. Whatever is chosen, the S7 invariant itself is unchanged: any path that
*does* `go_home` still drains first.

### 9.5 Rollback plan

Four levels, in increasing cost:

1. **Send a goal with `catch_positions` empty.** The session is a v1 session and
   T-U1 pins that its decision stream is identical to the pre-M2 tree. This is
   the rollback the bench uses, and it needs no rebuild.
2. **Revert M4 alone.** Every ring leg goes back to serial-with-skip: the
   two-pose floor rises 0.4170 → 0.5370 s and **nothing else changes** — no
   capability is lost, no gate moves, and the commit backstop it rides on was
   already there. MP-4 becomes unbookable; every other rung is unaffected. This
   is the cheapest real revert and it is why M4 is its own phase.
3. **Revert the lean change alone** (restore `req.lean_gain = -1.0`). Costs
   0.28 s per moving cycle and re-opens Finding 4's abort unless the budget is
   also reverted; the two land together and revert together.
4. **Revert M3, then M2, then M1.** M1 is independently valuable and should be
   the last thing removed: it closes C-REACH-1 § 5 residuals 1 and 5 whether or
   not any ring is ever flown.

### 9.6 Deploy

```bash
python config/generate_config.py                 # M2's new key, artifacts staged in the same commit
cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot
source install/setup.bash
```

then relaunch. **Both packages, every time, from M2 onward** — the `.action`
gains a field, and `reload_coordinator_node` imports `TossContinuous` at module
scope, so a `jugglebot`-only build takes all three ball-op actions down together.
`ros2 action list | grep -c jugglebot/toss` returning 2 is what detects it. No
firmware flash is involved at any phase.
