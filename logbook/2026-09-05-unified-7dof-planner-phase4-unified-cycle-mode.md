---
title: "Unified 7-DoF planner Phase 4 — the whole cycle becomes one plan: four window kinds that chain at a release, a planner that lives where the live state is, and a release-terminal cliff closed by joining the windows instead of racing the clock"
type: investigation
date: 2026-09-05
status: resolved
phase: "unified-7dof-planner — Phase 4 (software)"
related_plan: unified-7dof-planner.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/throw_envelope.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot_interfaces/srv/PlanCycle.srv
  - ros_ws/src/jugglebot_interfaces/action/TossContinuous.action
  - ros_ws/src/jugglebot_interfaces/msg/TrajectoryStatus.msg
  - ros_ws/src/jugglebot_interfaces/CMakeLists.txt
  - ros_ws/docs/choreography.md
  - sim/unified_gate.py
  - tools/probes/teensy_link_profiling/hermite_xref/teensy_interp.py
  - tests/firmware/test_hermite_xref.py
  - tests/motion/test_cup_cycle.py
  - tests/motion/test_unified_cycle.py
  - tests/motion/test_unified_cycle_budget.py
  - tests/ros/conftest.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_unified_cycle_integration.py
  - tests/sim/test_unified_gate.py
  - run_tests.sh
  - plans/active/unified-7dof-planner.md
  - plans/active/INDEX.md
  - logbook/2026-09-05-unified-7dof-planner-phase4-unified-cycle-mode.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
  - sim
  - tools
tags:
  - safety
  - performance
  - testing
  - docs
---

# Unified 7-DoF planner Phase 4 — Jetson unified-cycle mode

## Summary

Phase 4 of [`plans/active/unified-7dof-planner.md`](../plans/active/unified-7dof-planner.md):
the whole cycle now exists as **one 7-channel plan on one clock** — planned per
cycle off the emitter thread, installed through the existing continuity
machinery, streamed over Phase 2's v6 wire, interpolated by Phase 3's FW 17
lane. **SOFTWARE-COMPLETE, NOT FLOWN.** Both opt-in keys ship false
(superseded 2026-09-05: the build key is now true — see § Withdrawn claims).

- **Wave A — the planner layer.** `motion/unified_cycle.py` (1787 lines, `wc -l`
  2026-09-05), the pure-Python per-cycle orchestrator, plus the generalisation of
  `cup_cycle.plan_window` from one window shape to **four**: `LAUNCH`
  (rest → release), `STEADY` (release → catch → release), `LANDING`
  (release → catch → rest), `SETTLE` (release → rest). Windows **chain at a
  release instant**; `extend` / `replan_tail` make that seam exact.
- **Wave B — interfaces and three nodes.** A new `PlanCycle.srv`
  (NEW / EXTEND / REPLAN), `TossContinuous.action` gains `unified_cycle`,
  `TrajectoryStatus.msg` five cycle fields; `trajectory_node` (+871) grows the
  planning service, a hand continuity term and the supersede-deadline alarm;
  `reload_coordinator_node` (+1250) the session choreography;
  `catch_coordinator_node` gates its reactive hand arm off under a latched
  `catch/unified_mode`.
- **Wave C — sim gate and 7-lane mirror.** `sim/unified_gate.py` (1644 lines)
  drives planner → real emitter → real `SetpointPump` → packed-and-unpacked v6
  bytes → a 7-lane firmware mirror → MuJoCo, re-implementing none of it. **PASS**
  (2026-09-05, `python sim/unified_gate.py --no-viewer`): SET 1 `core_clean`
  **26/26** against a threshold of 24; SET 2's five-release beat exact to
  **4.441e-16 s**.

**The headline is a silent failure this phase found and closed.** A plan
streamed to its last knot **commands a stop at the throw**: the emitter's `τ+dt`
sample falls on `CyclePlan`'s terminal hold, so the release segment's
transmitted `v1` collapses from **93.011 rev/s to 0.0** — 0.3445 rev =
**10.90 mm** of slider error, inside `MAX_LEAD_HAND_REV` (2.0) and
`MAX_DEVIATION_HAND_REV` (2.5), so no guard fires and the only symptom is a
throw that went somewhere else. The fix is not a faster deadline but a
different **plan shape**: the coordinator joins `LAUNCH + LANDING` into one
rest-terminal plan before installing anything, and the class stops existing. It
costs 424 ms of warm solve per install — a **recorded deviation** from the
plan's ≤ 50 ms budget.

## Context — what Phases 0–3 left

Phases 1–3 built every piece except the thing that drives it. Nothing in the
ROS stack had ever asked for a `CyclePlan`, installed one, or streamed one.

**The structural finding that shaped the phase** (verified against
`cup_cycle.py` before any code was written): v1's `plan_window` expressed
**only the steady-state shape** — it starts at a release and ends at a release.
A session cannot be built from that alone, because there is no way in and no way
out. The other three kinds are not conveniences, they are the missing boundary
conditions.

What makes them composable is that **windows chain at a release instant**: a
release is a free-fall terminal state, so the cup's position, velocity and
acceleration there are fully determined by the ballistics and the terminal state
of one window *is* the start state of the next — exactly, not nearly.
`release_state_from_meta` and `extend` make that identity explicit; the measured
seam gap on the shipped join is **0.0 mm** on the pose and **4.07e-13 rev** on
the hand.

## Owner decisions (all 2026-09-04)

1. **Four window kinds, and the chaining lives in the node.** `plan_window`
   generalises to `(has_throw, has_catch, post_release)`; planning runs inside
   `trajectory_node`'s `PlanCycle` service; chaining and replans are the *same*
   mechanism — a same-origin re-install whose head is bit-identical to what is
   already streaming.
2. **Opt-in is two keys, both must turn**: build-time
   `jugglebot_operational.unified_cycle_enabled`
   (`config/hardware_config.yaml:883`, shipped **false** at this phase;
   superseded 2026-09-05 — it is now **true**, see § Withdrawn claims) **AND**
   the goal's own `unified_cycle` field (`TossContinuous.action:223`, default
   false — this is the one that still ships false).
3. **One atomic commit** for the phase; the unrelated bench `moving_gap` stage
   is its own commit and its own entry.
4. **The budget splits.** Core (everything `unified_cycle` owns) **≤ 50 ms**,
   asserted; the whole plan+validate call **≤ 250 ms**. The joined
   `LAUNCH + LANDING` install at ~424 ms warm is **accepted for Phase 4 as a
   recorded deviation**, follow-up named: vectorise `validate_cycle` in its own
   commit behind an output-identical pin.

Orchestrator decisions inside those: `SetTrajectoryLimits.srv` is **not**
widened (nothing here needs live hand limits, and a chain-pin test holds the
seam); the replan budget is **2 per installed window**; T-I4 is `serial` and
per-commit, not `nightly`; the sim gate is a **new file**, leaving
`sim/toss_gate.py` byte-unchanged; `catch/unified_mode` is a latched topic, not
the build constant; the announcement fires at the legacy lead but carries the
**plan's** release instant.

## Discussion

### The budget was one number, and it had to become two

The plan says *"budget ≤ 50 ms"*; the first attribution said ~180 ms. The reflex
— mesh `validate_cycle` more coarsely until it fits — was rejected on the spot:
`samples_per_knot` is an **accuracy** knob on the leg-jerk finite difference, not
a speed knob, and turning it down to pass a test is the "just relax this one
invariant" move the engineering-philosophy section forbids.

The attribution reframed it (2026-09-04, venv, idle box, 1.4 s reference cycle
at 57 knots, median of 15 timed calls after two warm-ups): QP `plan_window`
**10.7 ms**, `tilt_schedule` **3.7**, `decompose` **3.3**, `from_realized`
**0.05**, **`validate_cycle` 156.7**. So the planner is comfortably inside the
owner's number and **the gate is ~88 % of the call** — and the gate is the
*canonical* one this whole stack shares, not Phase 4 surface: one full IK chain
per sample over 225 samples (`accel_to_leg_accels` 0.201 ms/sample,
`compute_condition_number` 0.074, `compute_jacobian` 0.047, the rest under
0.04).

The owner split it: **core ≤ 50 ms** (measured **21.9 ms** = 180.1 total less
the 158.3 gate share) and **total ≤ 250 ms** (measured minimum 177–180 ms).
Asserting 50 ms on the whole call would land a red gate saying *"someone broke
this"*, which is false — nobody broke it, it has never been under 50 ms, and
closing the gap is a `validate_cycle` change needing its own owner decision.

### The release-terminal cliff, and why the answer is plan shape, not a faster clock

Phase 1 documented the terminal-hold inheritance honestly — `TrajectoryPlan`'s
contract holds at `final_pose` with **zero twist**, a cycle ends at the throw
moving at takeoff velocity, and *"Phase 4's orchestrator owns installing the next
cycle before the clock runs off the end."* Wave A's review found what that
sentence costs, and it is worse than a late install.

`KnotEmitter.frame` samples at `τ`, `τ+dt`, `τ+2·dt` and puts the `τ+dt` sample
on the wire as the **u1 knot and its exact velocities** — precisely what the
firmware's Mode-1 Hermite uses as the segment **endpoint velocity** under
`HAS_V1`. At `τ = duration − dt` that sample lands on the terminal hold.
**MEASURED** (2026-09-04, the reference 0.6 s LAUNCH, `/tmp/probe_f1_cliff.py`,
run twice with identical output): true terminal hand knot velocity **93.011
rev/s**, emitted `hand_next_vel_rps` **0.0** — one float ulp earlier it is still
93.011. Reconstructing that segment with `v1 = 0` moves the firmware's Hermite by
up to `|h11|·T·Δv` = **0.3445 rev = 10.90 mm** of slider and steps the
transmitted `vel_ff` by 93 rev/s. **Every number is inside every guard on the
path**: the positions are correct, only the endpoint velocity is wrong, and no
wire gate, pump gate or firmware clamp reads `v1` at all. Nothing rejects the
frame, nothing latches, and the throw is simply wrong.

Two answers existed. *Race the deadline* — export `latest_supersede_time_s`
(= `duration − dt`), publish it, alarm on it, require an `EXTEND` before it. Or
*remove the shape* — join `LAUNCH + LANDING` into one **rest-terminal** plan
before installing anything, so the export returns `math.inf` and there is no
deadline to miss. The second was chosen; the first is kept as a **backstop**.
The root cause is that a release-terminal plan's terminal knot is *a lie about a
trajectory that continues*; a deadline converts a silent wrong throw into a
timing requirement on a stack whose planning callback already costs ~250 ms, and
one missed deadline is one wrong throw with a ball in the air. Making the
trajectory actually continue removes the class. The deadline export survives
because `STEADY` chaining (Phase 5's UH-7) *will* be release-terminal, and
`trajectory_node._check_supersede_deadline` (`trajectory_node.py:3715`) is the
only trace that failure would ever leave — once per install, at ERROR, quoting
the measured numbers. **The accepted cost** is that the whole cycle is solved and
gated in one callback: 424 ms warm, against 67 ms for a bare `LAUNCH`.

### "The coordinator plans" was the wrong reading of the spec

The plan lists `unified_cycle.py` as coordinator-side and says planning runs *"in
the coordinator's service context"*. That fails for a reason that is not about
layering: **only `trajectory_node` holds the live commanded state.** Its
`_current_state()` samples the active plan on the emitter's own clock and it owns
the plan origin. A coordinator planning from the 5 Hz
`trajectory/commanded_pose` topic would seed from a pose up to **200 ms stale**,
and the install-continuity guard (0.06 rev of commanded drift) would then refuse
the install **by construction** — every cycle, forever. So the goal travels over
the wire and the solve happens where the state is (`PlanCycle.srv:6-11`). The
determinism rule still holds — the 40 Hz emitter is a separate thread and keeps
streaming the installed plan — and the callback's cost is precedent-compatible:
`trajectory/go_to_pose` already blocks 90–377 ms on this node today.

### Three smaller places the plan text was wrong

- **"the ships-false precedent."** `JB_OP_TOSS_PIPELINE_ENABLED` ships **true**
  (`config/hardware_config.yaml:704`; only the comment at `:706` still says
  otherwise). The precedent actually wanted is the **mechanism** — one read per
  goal (`reload_coordinator_node.py:6711`) — not the value: re-reading at each
  seam would let a mid-session config reload put the legacy stroke engine and the
  streamed plan on the same axis, exactly the dual-mastery class Phase 3's
  firmware latch exists to make impossible.
- **"and the goal opts in."** No such field existed; one landed
  (`TossContinuous.action:223`).
- **The hand-source bracket.** A session cannot bracket its own hand mastery —
  the firmware refuses a transition whenever the setpoint output is armed
  (`hand_source.cpp:60`) and the wire is armed for the whole ACTIVE state. What
  landed is a **verification, not a switch**: `_set_hand_source(True)` first,
  before anything is armed or commanded, a refusal aborting
  `REJECTED_HAND_SOURCE(REFUSED: …)`, and **no hand-back at teardown** — the
  terminal states where the latch was left and names the two routes back.

### Where the settle sits, and why it is not the park

A window must end somewhere, and the obvious rest is the hand's park —
`HAND_RETRACT_REV` 0.0, cup z 679.6 mm, what `toss_sequencer`'s
`HAND_NOT_PARKED` gate and the firmware's settle band are both centred on. It is
**10 mm below this module's own cup box**, so a window asked to settle there is
refused `SETTLE_SITE` before it plans (MEASURED 2026-09-04, the shipped chained
install at session limits: *"settle site z 0.6796 m is outside the cup box
[0.6896, 0.9846] m"*). `SETTLE_CUP_Z_MM` (`unified_cycle.py:182`) is therefore
the parked height **clamped up into the box**: 689.6 mm = **0.3162 rev**, inside
`HAND_PARK_BAND_REV` (0.5) with 37 % of the band spare, and a state the next
cycle's `LAUNCH` can be planned *from* (probed at 689.60 / 690.0 / 692.0 /
695.0 mm — all ACCEPT). Written as a `max`, so it collapses to the true park the
moment the box reaches it. It is **not** inside the firmware's ±0.10 rev settle
band — carried as an open question rather than papered over.

### Why the replan policy was dead code, and what revived it

`replan_tail` was written for a release-terminal plan: re-solve the catch-side
tail as a `STEADY` window holding the old throw fixed. But the shipped install is
**rest-terminal by design** — that is the cliff fix — so on the only shape the
machine flies, every tracker landing update would have answered `REPLAN_WINDOW`.
The policy would have shipped as a refusal. Generalising it (2026-09-05) took two
non-obvious corrections:

- **The terminal rest is pinned to `goals.settle_site_mm`, not the realised
  knot.** The settle sits exactly on the box floor and the QP holds it only to
  `feas_tol`: the realised terminal measured **689.59999999997694 mm** against
  the 689.6 pin — **2.3e-11 mm below** — and the inclusive `SETTLE_SITE` gate
  refused every shipped replan in the first draft.
- **A second splice bound, `k_s <= k_release + n_detach`.** A joined plan carries
  a release in its *middle*; the interior detach cone at knots 25–26 (after the
  release at knot 24) would otherwise be erased by a `LANDING` tail solved with
  `post_release=False`.

**Probe** (2026-09-05: LAUNCH 0.6 + LANDING 1.4 chained, settle at
`SETTLE_CUP_Z_MM`, session limits 250/3000/150000, banking on; 10 mm catch move
at `t_now` 0.600, `lead_s` 0.30): splice at `k_s` **36 of 81** (`catch_k` 56);
head **bit-identical on all four channels** over `[0, 36)`; catch error
**0.000000 mm**; terminal rest moved **0.000000 mm**; still rest-terminal
(`supersede = inf`); a second replan accepts; bit-reproducible across two runs.
Wall **231.6 / 233.4 ms** idle and **759.1 ms** with the suite beside it —
recorded, and explicitly **not** claimed reproducible.

### Two tradeoffs stated so nobody misreads them later

**The splice may not cut into a detach cone, and one knot is deliberately
conservative.** A splice inside the detach block erases the equalities keeping
the cup's acceleration collinear with the axis the ball left along — i.e. it puts
a lateral shove on a ball already leaving the lip. Measured on the aimed cycle:
**1.126 m/s² of lateral specific force at `k_s = 1` on a 3.24° aim** (0.686 at
1.62°, 0.276 level). `k_s == n_detach` happens to preserve the cone *via the
start-acceleration pin*, not via anything the check knows about; the refusal was
kept inclusive anyway (review F4), because a guarantee holding by accident of
another constraint's side effect is not a guarantee. The cost is one knot of
splice range.

**The mirror's leg lane is `motor_guard`'s clamp, not FW 17's.**
`teensy_interp.py` keeps `MAX_LEAD_REV` 0.15 with `vel_ff` **zeroed** on a
clamped leg, deliberately, because that file exists to be cross-checked against
`motor_guard` by `hermite_xref` and porting the firmware's leg clamp in would
break the xref it serves. `leg_interp.cpp:62-63,641-646` ships **0.10 rev** with
a 3.5 rev/s `VELFF_CAP` and never zeroes. So the gate's leg clamp counter is
*motor_guard's* verdict on the stream; a real board clamps **sooner and more
gently**; the counters are reported split (`_LEG_CLAMP_NOTE`,
`unified_gate.py:356`). The **hand** lane is the firmware's own, constants and
all.

### Three choices that beat reasonable alternatives

- **One mechanism for chaining and replanning.** `EXTEND` and `REPLAN` are the
  same thing twice: *re-install at the same origin with a head bit-identical up
  to `now + lead`*. Two mechanisms would mean two continuity stories, two install
  paths, and two ways for the emitter's `τ` to disagree with the plan's `t = 0`.
  The continuity guard still runs on both — the bit-identical head is a property
  of the *planner*, and the node's job is to check it, not trust it.
- **A new `sim/unified_gate.py`, not a flag on `toss_gate.py`.** The unified gate
  drops `catch_armed` and `hand_arm_infeasible` because under unified mode
  **there is no reactive arm to have fired** — the catch stroke is in the plan. A
  flag would have made those bands conditionally unfalsifiable inside the file
  the legacy path depends on; a new file leaves `toss_gate.py` byte-unchanged and
  states the unified bands positively (`flags_ok` + `mirror_ok` replace them as
  the silent-dead-hand guard).
- **`catch/unified_mode` as a latched topic, not the build constant.** The build
  flag is a *build* fact; hand ownership is a *session* fact. A coordinator
  keying off the constant would arm legacy strokes into a STREAMED latch for
  every legacy goal on a unified build. TRANSIENT_LOCAL depth 1
  (`reload_coordinator_node.py:2131-2133`) so a late subscriber gets the standing
  declaration on connect, matched at the reader.

## Implementation

### Wave A — window kinds and `motion/unified_cycle.py`

`cup_cycle.py` (+266/−99) gains `post_release` on `CupState`, a `settle_site`
boundary condition, the `SETTLE_SITE` refusal reason and the no-catch sentinels
(`catch_k = -1`, zeroed `takeoff_vel`); `_KIND_SHAPE` (`unified_cycle.py:110`) is
the whole of what distinguishes the four calls. `cup_realize` gains a
**`start_tilt` seam pin** — without it a chained window opens with a **0.8035°
tilt gap**, which the 744.3 mm lever turns into a **1.586 mm centroid step in a
single knot**. `build_realize_config` derives the tilt-accel cap from the *live*
limits (leg accel 2000 → 2.1305, 3000 → 3.1957, 5000 → 5.3262 rad/s², the last
being the shipped value Phase 1 recorded). The aim gate pins `TILT_PIN` on the
**unclamped** angle, so a saturated aim refuses rather than silently flying a
different one.

Residuals, all measured 2026-09-04: cup-state round trip **2.98e-11 mm /
3.38e-11 mm/s / 5.33e-14 mm/s²**; forward map over 2000 tilted poses
**2.274e-13 mm / 1.421e-14 rad**; `extend` seam **0.0 mm / 4.07e-13 rev**;
`sim/cycle_gate.py`'s Phase 1 headline **unchanged**.

Two envelope properties carry forward. **Banking cannot be turned off on a
steady cycle**: zero-banking refuses `LIMIT_VEL` at 529 mm/s against the 250
session cap, so unified mode is banking-on by construction. And **the replan
envelope is narrower than the splice rule** — on the 1.4 s reference cycle a
splice at knot 4 validates at 50 891 mm/s³, knot 12 refuses `LIMIT_JERK`
(186 215), knot 20 refuses `LIMIT_ACC` (4233 mm/s²), and it does so *for a replan
to the same catch site*, so the cause is the tilt smoother's pins arriving
through the 744.3 mm lever, not the re-aim. Every one is a **loud refusal from
the canonical gate**: the failure mode is a lost re-aim, never a bad plan.

### Wave B — interfaces, three nodes, and the session choreography

**`PlanCycle.srv`** (183 lines): `MODE_NEW` / `MODE_EXTEND` / `MODE_REPLAN`;
`KIND_LAUNCH` / `STEADY` / `LANDING` / `SETTLE`; the `CycleGoals` verbatim plus
chain fields; a response carrying `t0_mono` / `t_release_mono` / `t_catch_mono`,
`release_terminal`, `supersede_deadline_mono`, `stroke_clear_s`, `arm_lead_s`,
`duration_s`, `plan_wall_ms`, `replans_used` and the two hand peaks.
`TossContinuous.action` gains `unified_cycle`; `TrajectoryStatus.msg` gains five
`cycle_*` fields.

**`trajectory_node.py`** (+871): hand state ingest at index 6; `_svc_plan_cycle`
(`:3393`) with `_svc_timed_target`'s guards in the same order (guard latch → mode
→ seeded → telemetry fresh) and a refusal contract that **holds the last good
plan**; the **hand continuity term**, bound `0.25 × STEP_BOUND_MARGIN ×
hand_vel_limit × knot_dt` = 0.25 × 0.80 × 5.0 = **1.0 rev**, scaled exactly like
the legs' 0.06 (measured drift from rest **4.7e-10 rev**; a 1.5 rev step is
refused `STALE_STATE`). Charging the full 5.0 rev pump gate here, as the first
draft did, admitted an install discontinuity **2× `MAX_DEVIATION_HAND_REV`** — a
step the firmware would E-STOP on and this gate would wave through.
`catch/dynamic_target` routes to `REPLAN` behind the existing freeze and envelope
gates. `_check_supersede_deadline` (`:3715`) publishes
`cycle_supersede_deadline_s` — positive before, **negative after** — and logs
`UNIFIED CYCLE CLIFF` once per install at ERROR.

**`reload_coordinator_node.py`** (+1250). The session timeline as it runs:

| Instant | What happens | Where |
|---|---|---|
| Goal accepted | `unified` resolved **ONCE** (build flag AND goal field) | `:6711` |
| Session start, nothing armed yet | `set_hand_source(True)` **VERIFICATION**; refusal aborts `REJECTED_HAND_SOURCE` | `:9059` |
| immediately after | `catch/unified_mode` **raised** (latched, `:9082`); pipelined forced OFF at session build (`:8982`) | `:2135`, `:9082`, `:8982` |
| immediately after | planner **warm-up** — one throwaway solve, commands nothing | `:6735` |
| `t_release − 1.80 s` | `PlanCycle` **NEW**: `LAUNCH` + chained `LANDING`, joined, installed | `:7014` |
| on install | announcement published from the **plan's** `t_release_mono` | `:7151` |
| the FSM's throw seam | `_dispatch_toss` reports OK and issues **no hand RPC** | `:6988` |
| 0.60 s before a terminal release | `PlanCycle` **EXTEND** chains the next window | `:7220` |
| survived MISS | hold the pose, **no `go_home`**, wait `_UNIFIED_MISS_SETTLE_S` | `:7358` |
| terminal (any exit) | `catch/unified_mode` lowered; ONE INFO line on where the latch is | `:9477` |

Both leads are derived, not chosen. `_UNIFIED_LAUNCH_LEAD_S` = the 0.6 s window +
**`_UNIFIED_PLAN_BUDGET_S` 1.20 s**, deliberately an *over*-estimate (2.8× the
idle median, covering the loaded 1.06 s): the skew is `cost − B`, so
over-estimating only makes the ball leave early relative to the FSM's nominal
instant, which nothing reads (the session schedules off the **plan's** release),
whereas a late release runs past `TOSS_RELEASE_GRACE_S` 0.5 s.
`_UNIFIED_EXTEND_LEAD_S` **0.60 s** covers a whole measured `EXTEND` call
(2026-09-04, idle, 6 consecutive `MODE_EXTEND` calls: min 354, median 355, max
387 ms) plus round trip and the 40 ms poll, with ~55 % margin — `extend`'s
bit-identical-head guarantee holds only **while the head is still playing**.
`_UNIFIED_MISS_SETTLE_S` = **2.03 s** = `DEFAULT_SESSION_MISS_CLEANUP_S` 2.80 −
`GO_HOME_DURATION_S` 2.0 + **1.23 s** of ball settling (the legacy 2.0 s
`go_home` covered that term by accident), spent **only on a survived MISS** — a
cancel or shutdown sleeps for nothing, and the legacy constant is untouched.

The **warm-up** is not an optimisation: **cold 3267 ms against a 424 ms warm
median** for the same joined install (2026-09-04). A 3.3 s solve at the launch
trigger lands the release ~2.1 s late — past the grace — so cycle 1 of *every*
session would abort `ABORTED_NO_RELEASE` with the ball in the air, and raising
the ceiling to cover the cold case would mis-shape every warm cycle to pay for
the first. The cost moves to session start, where nothing is armed and nothing is
airborne; a failure there is swallowed with a WARN. Other measured call costs:
bare `LAUNCH` **67 ms**, `EXTEND` **355 ms**, `REPLAN` **180 ms**.

**`catch_coordinator_node.py`** gates `_arm_hand_catch` **twice** — at the call
site (`:890`) and inside the method (`:1222`). **`toss_sequencer`** carries a
`unified` field; **`throw_envelope`** gains an `arm_window` kwarg (default `True`,
so every legacy caller is unchanged) and the `ARM_WINDOW` carve-out is applied at
all three evaluate sites — the FSM, the speed-authority band, the ILC trim —
under unified only, because there is no arm to fit in a window.
`tests/ros/test_unified_cycle_integration.py` (2463 lines, **77 tests**) covers
T-I1 and T-I3; `tests/ros/conftest.py` grew QoS recording so the latch's
durability is assertable; `ros_ws/docs/choreography.md` was regenerated.
**Measured end to end** (2026-09-05): two `catch/dynamic_target` updates replan
(`replans_used` 1 then 2, the installed plan changing each time), the **third is
refused `REPLAN_BUDGET`** with the last good plan and origin intact, and a bare
`LAUNCH` against a rest-terminal install is refused `NO_CYCLE` **without a
solve**.

**Teardown** (fixed 2026-09-05): the first draft attempted a hand-back and minted
a `STUCK_STREAMED` terminal for the refusal it always got — an alarm the operator
would learn to read past. Deleted. `_toss_hand_source_streamed` is set **only
after** the start verification returns OK, so "REMAINS STREAMED" is never printed
on the path where the bridge said no; the outcome string is untouched and the
stats line appends `hand_source STREAMED`. Event order is pinned as
`[hand_source True, unified_mode True, unified_mode False]` — a re-introduced
hand-back fails the spy the way the firmware would.

### Wave C — the sim gate and the 7-lane mirror

`sim/unified_gate.py` (1644 lines) + `tests/sim/test_unified_gate.py`
(**16 tests, unmarked, ~27 s**, matching `test_cycle_gate.py`'s tier placement).
The mirror had to be built: **no 7-lane Python reference existed** — Phase 3's
T-U9 pinned the C++ natively and the pump on the wire, but nothing in Python
reconstructed all seven lanes. `teensy_interp.py` gains the hand block (its own
knot clock, `MAX_LEAD_HAND` 2.0, `VELFF` 300, clip `[0, 10.8]`, the 1 → 2 → 3
mode ladder), transmitted-`v1` Mode 1 on every lane, a NaN backstop and a
staleness path; `tests/firmware/test_hermite_xref.py` gains 4 + 2 tests pinning
it to `canbridge_config.h`.

**Gate result** (2026-09-05, after the F2 settle-site fix,
`python sim/unified_gate.py --no-viewer`, **PASS**, 73.4 s): SET 1 `core_clean`
**26/26** (threshold 24) with one **advisory** point — the legacy z = 200 mm tier
at 0.80 s, refused `HAND_LIMIT_ACC` **4666** against the 3500 cap, reported and
never gated; SET 2's beat **exact at 4.441e-16 s**; mirror **legs 1.065e-5 rev**
(band 5e-4) and **hand 4.766e-7 rev** (one float32 half-ulp; the test bar is
`MIRROR_TOL_HAND/2`); capture 10.02 mm; post-plan hold 0.1257 mm / 0.00277°; seat
1.82°; hand `dev_max` 0.780 rev; lead clamps **0**.

The mirror band is **non-vacuous only where there is platform motion to catch a
regression in**: masking `HAS_V1` moves the 60 mm ring's leg reconstruction to
**5.836e-4 rev = 1.17× the band** and moves a co-located toss's by *nothing at
all* (9.891e-08 either way — with the platform near-still the `(u2−u1)/T`
fallback reproduces the transmitted `v1`).
`test_the_mirror_band_is_non_vacuous` pins **both** halves.

The legacy tier → `CycleGoals` mapping is identity on xy, flight (`catch_t` =
flight) and displacement; only **height moves onto the cup** (the slider reaches
it). Two facts Phase 5 inherits: the `LANDING` window's **1.4 s is
load-bearing** — at 1.0 s the 50 mm ring refuses `LIMIT_JERK` at 155–198 k; and
the falling-edge decay was finally *observed* — hand velocity reaches 0 at
0.136 s, but the raw wind-down would travel **6.36 rev**, clamped to **2.000** by
`MAX_LEAD_HAND_REV` over 185 lead-clamp ticks.

### The review and the two fix waves

The lean 2-lens shape (owner directive, 2026-09-02). Lens A+C returned **2 HIGH /
11 MEDIUM / 12 LOW**; lens B returned **3 HIGH / 8 MEDIUM / 2 LOW**; **all
adjudicated FIX**. Five fixes are deliberate deviations from the plan text and
are recorded as such: A+C's **F4** (the conservative detach-block splice knot),
**F16** (the true slider floor is 0.6796 m, not 0.6596 — the runway gains
0.010 m) and **F2** (the gate settles at the *catch* xy, which is why the leg
mirror residuals fell from 2.94e-05 / 8.086e-05 to 9.46e-07 / 1.065e-05 on the
two rings); and B's **F1** / **F2** — the release-terminal join and the
rest-terminal replan generalisation, i.e. the two findings this Discussion is
largely about.

## Verification

- Budget attribution (2026-09-04, venv, idle box, 1.4 s reference cycle at 57
  knots, median of 15 timed calls after two warm-ups): QP 10.7 ms / tilt 3.7 /
  decompose 3.3 / `from_realized` 0.05 / **`validate_cycle` 156.7 ms**.
- T-I4 (2026-09-04, standalone, 30 total + 10 gate solves after warm-up):
  **core 21.9 ms** (180.1 − 158.3) against the 50 ms owner budget; **total
  minimum 179.4 ms** against the 250 ms ceiling. An 8×40-solve batch the same day
  read p50 177.3–177.9, p90 177.9–180.5, p99 182.7–206.4 ms.
- Chain seams (2026-09-04): `extend` pose gap **0.0 mm**, hand **4.07e-13 rev**;
  cup round trip 2.98e-11 mm / 3.38e-11 mm/s / 5.33e-14 mm/s²; forward map over
  2000 tilted poses 2.274e-13 mm / 1.421e-14 rad.
- Release-terminal cliff (2026-09-04, `/tmp/probe_f1_cliff.py`, **run twice with
  identical output**): terminal hand knot **93.011 rev/s** emitted as **0.0** at
  `τ = 0.575 s`; **0.3445 rev = 10.90 mm** of reconstructed slider error.
- Replan probe (2026-09-05, LAUNCH 0.6 + LANDING 1.4 chained at 250/3000/150000,
  banking on, 10 mm catch move at `t_now` 0.600, `lead_s` 0.30): `k_s` **36 of
  81**, head bit-identical on all four channels, catch error **0.000000 mm**,
  terminal rest move **0.000000 mm**, still rest-terminal; a second replan
  accepts; bit-reproducible across two runs. Wall **231.6 / 233.4 ms** idle;
  **759.1 ms** with the suite beside it — recorded, **not** claimed reproducible.
- Sim gate (2026-09-05, `python sim/unified_gate.py --no-viewer`, **PASS**,
  73.4 s): SET 1 `core_clean` **26/26**, threshold 24, one advisory point; SET 2
  beat exact at **4.441e-16 s**; mirror leg **1.065e-5 rev** / hand
  **4.766e-7 rev**; capture 10.02 mm; hold 0.1257 mm / 0.00277°; seat 1.82°;
  hand `dev_max` 0.780 rev; clamps 0.
- Integration battery (2026-09-05, `python -m pytest
  tests/ros/test_unified_cycle_integration.py -q -p no:randomly`, **77 passed
  in 10.66 s**).
- Final scoped run (2026-09-05, `python -m pytest
  tests/motion/test_unified_cycle.py tests/motion/test_unified_cycle_budget.py
  tests/motion/test_cup_cycle.py tests/ros/test_unified_cycle_integration.py
  tests/ros/test_toss_integration.py tests/ros/test_toss_continuous_node.py
  tests/ros/test_choreography_map.py tests/sim/test_unified_gate.py -q -p
  no:randomly`, **421 passed in 66.87 s**).
- ROS + motion tiers (2026-09-05, `python -m pytest tests/ros/ tests/motion/ -q`,
  **4895 passed / 4 skipped in 710.30 s**) — `tests/ros/` is **2813** of those,
  i.e. **T-R3 holds: the whole mocked-ROS toss battery passes unchanged** — and
  `tests/sim/test_toss_gate.py` passed unchanged (2026-09-05, `python -m
  pytest tests/sim/test_toss_gate.py -q`, **18 passed in 24.19 s**) —
  `sim/toss_gate.py` is byte-unchanged and `throw_envelope.evaluate`'s new
  `arm_window` kwarg defaults to the legacy bound.
- **The nightly was RED on 2026-09-05T04:03** (`RED 6683/6690 passed, 1 failed,
  0 errored, 2 xfailed, 4 skipped`), on
  `tests/sim/test_sim_import_style.py::test_sim_library_modules_do_not_mutate_sys_path`.
  The nightly runs against the **working tree**, and the cause was the
  then-uncommitted Wave C `sim/unified_gate.py`: it took a **second** `sys.path`
  mutation to reach `teensy_interp`, and the import-style contract allows an
  entry script exactly one (the repo-root preamble `bootstrap_paths()` needs).
  Fixed in the B fix wave — `teensy_interp` now loads **by file path** through
  `importlib.util.spec_from_file_location`, registered in `sys.modules` under its
  own name so the xref's other consumers still see one module identity
  (`sim/unified_gate.py:151-160`). The test passes **4/4** in the current tree.
- Full gate, first run (2026-09-05, `./run_tests.sh --full`, parallel **1 failed / 6684 passed / 4 skipped / 2 xfailed in 432.99 s**, serial 4 passed in 25.90 s): the one failure was `tests/motion/test_unified_cycle.py::test_replan_tail_pins_the_rest_to_the_goal_not_to_the_realised_knot`, which asserted the SIGN of the QP's ~1e-11 mm terminal residual (standalone 2.3e-11 mm below the box floor; under the 4-worker gate 5.8e-12 mm above) — a flake by construction. Rewritten to a magnitude bound (`abs(realised − pin) ≤ 1e-6` mm), and the below-floor case is now constructed deterministically in `test_replan_tail_pin_mechanism_survives_a_realised_knot_forced_outside_the_box` (2026-09-05, `python -m pytest tests/motion/test_unified_cycle.py -q`, **47 passed in 5.15 s**; also green under `-n 4` and `OPENBLAS_NUM_THREADS=1`).
- Full gate, second run after that fix (2026-09-05, `./run_tests.sh --full`, parallel **6686 passed / 4 skipped / 2 xfailed in 443.76 s**, serial **4 passed in 25.82 s**, total 476 s, exit 0) — **GREEN**. This is the pre-commit gate.

## Outcome

**Phase 4 is SOFTWARE-COMPLETE — planned, gated in sim, NOT FLOWN.** The unified
cycle exists end to end: a goal becomes four window kinds, the windows chain at
release instants, the plan is solved where the live state lives, installed
through the continuity machinery it was designed to satisfy, streamed as seven
channels over the v6 wire, and reconstructed by a mirror of the firmware lane
that flew on the bench last week. Both opt-in keys ship false (superseded
2026-09-05: the build key is now true and the goal field is the ships-false
element — see § Withdrawn claims), so nothing about a legacy session changes: `sim/toss_gate.py` is byte-unchanged, `throw_envelope`'s
new kwarg defaults to legacy behaviour, `DEFAULT_SESSION_MISS_CLEANUP_S` is
untouched, and the whole mocked-ROS toss battery passes unchanged. T-I1, T-I2,
T-I3 and T-I4 are landed. What Phase 5 inherits is a machine that has never moved
under this planner, plus the preconditions and open numbers below.

## Withdrawn claims

- [2026-09-05] *"Both opt-in keys ship false"* (three places above).
  **SUPERSEDED, not wrong when written:** the Phase 5 prep of 2026-09-05 flipped
  the build-time `jugglebot_operational.unified_cycle_enabled` to **true** as a
  reviewed config commit, so which build ran the unified planner is answerable
  from git alone. The ships-false element is now the per-goal
  `TossContinuous.unified_cycle` field, and it is what keeps every legacy goal
  legacy. Note there is **no refusal path** on either key: with the unified path
  off, a goal that asks for it runs on the legacy path silently.
  **Superseded by:**
  [`2026-09-05-unified-7dof-phase5-prep.md`](2026-09-05-unified-7dof-phase5-prep.md).
- [2026-09-04] The plan's *"budget ≤ 50 ms"* read as covering the whole
  `plan_cycle` call.
  **WITHDRAWN:** `validate_cycle` is 156.7 of ~180 ms — the canonical gate, not
  Phase 4 surface — so a 50 ms assertion on the whole call asserts a falsehood,
  and the only way to satisfy it is to weaken the gate.
  **Superseded by:** the owner's split, core ≤ 50 / total ≤ 250 ms, plus the
  recorded 424 ms joined-install deviation.
- [2026-09-04] *"The coordinator plans the cycle"* (the plan's § 4 wording).
  **WITHDRAWN:** only `trajectory_node` holds the live commanded state and the
  plan origin; a coordinator-side solve seeds from a ≤ 200 ms-stale pose and the
  0.06 rev install-continuity guard then refuses every install by construction.
  **Superseded by:** planning inside `trajectory_node`'s `PlanCycle` service
  (`PlanCycle.srv:6-11`).
- [2026-09-04] *"the `JB_OP_TOSS_PIPELINE_ENABLED` ships-false precedent."*
  **WITHDRAWN:** it ships **true** (`config/hardware_config.yaml:704`; the comment
  at `:706` is stale).
  **Superseded by:** the precedent taken as the **mechanism** — one read per goal
  (`reload_coordinator_node.py:6711`) — not the value.
- [2026-09-04] *"…and the goal opts in"*, written as though the field existed.
  **WITHDRAWN:** it did not.
  **Superseded by:** `TossContinuous.action:223`, `bool unified_cycle false`.
- [2026-09-04] The session would bracket hand mastery — STREAMED at start, handed
  back at teardown.
  **WITHDRAWN:** no session-scoped arm exists; the firmware refuses a transition
  while the setpoint output is armed (`hand_source.cpp:60`) and the wire is armed
  for the whole ACTIVE state, so a teardown switch could only be refused — the
  first draft's `STUCK_STREAMED` terminal was an alarm for a designed outcome.
  **Superseded by:** a **verification** at session start (fail-closed,
  `REJECTED_HAND_SOURCE`), no hand-back, and one INFO line naming the two routes
  back.
- [2026-09-04] The window would settle at the hand's park, cup z 679.6 mm.
  **WITHDRAWN:** that is 10 mm outside this module's own cup box and is refused
  `SETTLE_SITE` before it plans (*"settle site z 0.6796 m is outside the cup box
  [0.6896, 0.9846] m"*).
  **Superseded by:** `SETTLE_CUP_Z_MM` = 689.6 mm = **0.3162 rev**
  (`unified_cycle.py:182`).
- [2026-09-05] `replan_tail` was believed live on the shipped install.
  **WITHDRAWN:** it accepted only release-terminal plans, and the shipped install
  is rest-terminal **by design** (the cliff fix) — so every tracker update on the
  only shape the machine flies would have answered `REPLAN_WINDOW`. The policy
  would have shipped as dead code.
  **Superseded by:** the rest-terminal generalisation, the `goals.settle_site_mm`
  terminal pin (the realised knot sits 2.3e-11 mm below it and the inclusive gate
  refused everything), and the `k_s <= k_release + n_detach` bound.

## Open Questions

1. **Vectorise `validate_cycle`.** It is 156.7 of ~180 ms and ~88 % of every plan
   call; the joined install's 424 ms is almost entirely it, twice. Own commit,
   **output-identical** pin (the gate's verdicts and numbers must not move); it is
   what brings the joined install under ~150 ms. Owner-accepted as a deviation for
   Phase 4, not waived.
2. **`STEADY` back-to-back chaining is a safety net, not a flown path.** The
   shipped session installs `LAUNCH + LANDING` joined; the `CHAIN_SKEW` machinery
   and `_UNIFIED_EXTEND_LEAD_S` exist and are tested, but UH-7's constant beat
   needs a `release_at_perf` hand-off that does not exist yet. Until it does, a
   release-terminal install is the shape `latest_supersede_time_s` and the cliff
   alarm guard.
3. **The latch return after a unified session is an operator action, and the hand
   does not rest where the firmware wants it.** A session ends with the hand at
   **0.3162 rev** (inside `HAND_PARK_BAND_REV` 0.5) but the firmware's
   `hand_source` settle band is **±0.10 rev**, **unreachable from inside the QP's
   cup box** (it would need 689.6 → 682.8 mm). Two routes back to
   `LEGACY_STROKE`: disarm the setpoint output then `/set_hand_source false`, or
   reboot the can-bridge (it boots LEGACY). **Owner decision open:** whether to
   inset the cup box differently, or move the park height, so a settled session
   lands inside the firmware band. A legacy session started on the same power
   cycle before the latch is returned will have its `SetHandTrajCmd` refused
   `ERR_HAND_SOURCE` by the firmware — fail-closed, but every such catch is a
   MISS until the operator acts.
4. **The operator precondition for any unified sitting.** The latch must be
   **STREAMED before ACTIVATE**, set with the launch **down** via
   `hand_stream_bench.py --source-only` (switch and exit — no stream, no arm). A
   session cannot do it for the operator; it refuses rather than proceeding.
5. **Aimed unified throws are out of scope this phase.** The MP plan's § 7 unlock
   — re-deriving the aim-authority window against `tilt_geometry.MAX_TILT_DEG`
   (12°) — transfers to Phase 5 and must be done before any aimed rung. The
   measured lateral specific force at a splice (1.126 m/s² at a 3.24° aim) is why
   the number matters.
6. **The cup z constants are module-level literals, deliberately.**
   `_UNIFIED_THROW_CUP_Z_MM` 860.0 and `_UNIFIED_CATCH_CUP_Z_MM` 830.0
   (`reload_coordinator_node.py:517-518`) are **not** YAML keys, because they
   exist to be moved by UH-5/UH-6 and a config key with no hardware behind it is a
   knob an operator can turn into an infeasible cycle with no gate to say so. They
   promote to YAML at Phase 5, once tuned.
7. **A latent asymmetry in the replan path.** `_plan_cycle_replan` computes
   `t_now_s` **before** the ~230 ms solve and checks continuity against it;
   `EXTEND` recomputes **after**. Measured leg drift across the solve on the
   vertical self-toss is **0.0000 rev** against the 0.06 bound, so nothing is
   wrong today — but an aimed cycle moving the platform hard mid-flight could
   expose it.
8. **The mirror's leg band is thin where it matters.** `MIRROR_TOL_LEG_REV` 5e-4
   is ~47× the worst observed, but the only configuration that makes it **fail**
   on a `HAS_V1` regression is the 60 mm two-pose ring, at **1.17×** the band. A
   future grid with less platform motion would silently lose that detection.
9. **The detach-block splice keeps one knot it does not have to.**
   `k_s == n_detach` preserves the cone via the start-acceleration pin; the
   refusal is inclusive anyway (F4). Revisiting it needs a proof about the pin,
   not a measurement.
10. **The Phase-5 coast note.** On the falling edge at 75–92 rev/s the hand's raw
    wind-down travels **6.4–7.4 rev (200–235 mm)**; the *only* thing bounding it
    is `MAX_LEAD_HAND_REV` 2.0 (measured in the gate: raw 6.36 rev clamped to
    2.000 over 185 ticks). Anyone shortening or stopping a stream mid-throw on
    hardware should read that as the governing number.
11. **T-I4 asserts the minimum, not p99, and that is a deliberate blind spot.**
    Planning is bit-for-bit deterministic, so between-call spread is the OS, not
    the code — but under `pytest` this box produced a p90 of 354.9 ms on one run
    of eight and a p50 of 278.8 with a 1554 ms max on another, with 417 ms calls
    beside 180 ms neighbours doing byte-identical work. A p99 assertion at N = 30
    would be a shipped load-flake. The minimum loses no detection power on
    deterministic work; what it cannot see is a tail regression, and deterministic
    work has none.
