# The Levelling Frame — contract C-LEVEL-1

**Normative.** This document specifies where the gravity-levelling correction is
applied and — just as importantly — where it must **not** be. It is the written
half of the repo's contract pattern (normative statement + one shared
implementation + a test that fails on the omission); the other two halves are
`jugglebot/motion/levelling.py` and `tests/ros/test_levelling_frame.py`.

Scope: the ROS 2 package `ros_ws/src/jugglebot/`. Neither `sim/` nor
`controller/` has any gravity-levelling concept — grepped 2026-07-25, the only
hit outside this package is the `levelling_complete` **boolean flag** in
`controller/teensy_link/rpc_args.py`, which is a Teensy-persisted per-boot flag
and not the correction. A reader looking for a second implementation should stop
looking: there is none, and adding one is what this contract forbids.

## Why this exists — the failure it closes

The Platform Teensy's inclinometer measures the platform's residual tilt against
gravity while it is commanded level. `orchestrator_node` publishes that
measurement on `/gravity_offset` as `[tilt_x, tilt_y]` radians. The correction is
its inverse, so that commanding "zero tilt" produces a platform level **with
respect to gravity** rather than level with respect to its own imperfectly
mounted frame.

Before 2026-07-25 `trajectory_node` applied that correction on *some* pose-ingest
paths and not others, so **"level" meant two different things inside one node**.
The consequence, measured on the self-toss session of 2026-07-25 15:17:48: the
toss's positioning `go_to_pose` parked the platform at plan-frame `rx = 0` while
the pre-tilt `catch/dynamic_target` asked for plan-frame `rx = −0.7788°`, and
`build_catch` dutifully planned a min-jerk reach between the two — arriving
0.7 s before release. The platform **rested 0.78° off gravity** while the catch
surface asked for level, and the two surfaces disagreed about where "level" was.
The falsification is clean: the un-levelled session of the same day, same action,
same code, held commanded `rx` within ±0.05° for the whole toss.

The bug was not a wrong number. It was an **incomplete enumeration** of the
surfaces a pose can enter through. That is the class this contract closes.

### What C-LEVEL-1 does NOT close — read this before scoring a bench session

> **Update, 2026-07-26 — both of these are now CLOSED, by C-CATCH-1**
> (`ros_ws/docs/catch_arrival_contract.md`). The analysis below is correct and is
> kept verbatim because it is what made the second contract necessary; what has
> changed is only the last clause of each bullet ("removing it means changing the
> arrival twist" / "deliberately not taken"). `build_catch` now takes the
> **gravity-referenced** receive tilt as its own argument, so a gravity-level
> catch gets a zero arrival twist: the swing goes to **0.0000°** and the settle
> lands exactly on the target, `0.0000°` off gravity at ball contact. An
> operator scoring a post-2026-07-26 capture should expect a **flat, monotone**
> catch reach — the "the visible tilt REMAINS" pre-brief below is superseded.
> The reload path's seat survives, with its aim rotated 4.0997°; the numbers are
> in C-CATCH-1's "Consequences at the machine".

**The visible swing, and the 16 mm catch error, are not this contract's.** Both
belong to `planner.build_catch`, and both survive the fix:

* **The swing.** `build_catch` specifies a non-zero *arrival twist* on its reach,
  so even a reach with zero net tilt displacement must excurt and come back. With
  `p0 == p1` the quintic reduces exactly to `v1·T·φ(s)`, `φ = −4s³ + 7s⁴ − 3s⁵`,
  `|φ|` maximal `16/81` at `s = 2/3` — so the excursion is **linear in the catch
  lead** and independent of the correction's magnitude. For the 2026-07-25 offset
  direction that is **0.789132° per second of lead**: at the 3.70 s lead the
  reference session ran, the platform still swings **+2.92°** off gravity before
  every toss. What the contract changes is where it swings *from*: pre-fix
  0.7788° → +3.0992° peak, post-fix 0° → +2.9198°. A 0.18° improvement in a 2.9°
  swing, plus a park that is finally gravity-level. Removing the swing means
  changing the arrival twist —
  `plans/active/catch-reach-degenerate-overshoot.md`, not this contract.
* **The 16 mm.** `build_catch` aims its tilt-through-seat residual along
  `catch_pose[3:5]`, which *with a correction loaded is the correction itself*, so
  a gravity-level catch settles `0.5 × 0.07 × 0.15 = 0.005250 rad = ` **0.3008°
  off gravity-level at ball contact** — 16.5 mm of drift at 3.93 m/s over 0.8 s,
  against the 16 mm the tracker measured. Pre-existing (`catch/dynamic_target` was
  already corrected before 2026-07-25) and pinned as a characterisation test,
  `tests/ros/test_levelling_frame.py::test_catch_through_seat_aims_off_the_gravity_referenced_receive_tilt`.
  Closing it changes commanded motion at ball contact on every catch including the
  shipping reload path — an operator decision, deliberately not taken.

Both were originally attributed to the frame plumbing in this document, in the
plan and in the operator runbook. They are not, and a reader who measures ≈16 mm
after this contract lands and re-audits the six ingest sites is chasing the wrong
tree — which is what the "`go_home` is a no-op" docstring already cost this
investigation once.

> **Both bullets are now CLOSED, by the sibling contract, not by this one.** Kept
> above verbatim because the *attribution* is the load-bearing part and stays
> true; only their "deliberately not taken" status has changed.
> - `407154f` landed **C-CATCH-1** (`ros_ws/docs/catch_arrival_contract.md`).
>   `build_catch` takes the gravity-referenced receive tilt as its own argument,
>   so a gravity-level catch's arrival twist is zero by construction: the swing
>   goes to `0.0000°` and the 16.5 mm residual to `0.0000°`.
> - The operator decision the second bullet said was *"deliberately not taken"*
>   **was taken on 2026-07-26**: `_CATCH_TILT_THROUGH_RATE_RADPS` ships at `0.0`,
>   so the through-seat is not manufactured on *any* catch, including the reload.
>   Platform motion at a catch stays permitted via
>   `build_catch(tilt_through_rate_radps=)`; it is no longer mandated.
>
> The `0.5 × 0.07 × 0.15` arithmetic above is retained as the derivation of the
> 2026-07-25 measurement, not as a live number.

## C-LEVEL-1

> The gravity-levelling correction is applied **exactly once per external pose**,
> through a **single shared implementation** (`jugglebot.motion.levelling`), at
> the enumerated ingest sites and **at no other site**. "External" means the pose
> entered from outside the node: a service request, a wire target, or the
> built-in neutral constant. It is applied to **no** pose derived from
> measurement or from an existing plan. It rewrites the **rotational component
> only** — positions, linear velocities and accelerations pass through untouched.

### Why an enumerated set and not one choke point

A single choke point would have to sit where every pose converges — the planner
entries, or the emitter. It cannot: *derived* poses reach the same planner
entries as external ones and they **outnumber** them (nine derived seeds against
six external ingests, plus four direct plan installs that bypass `planner`
entirely), and a choke point there has no way to tell them apart. It would
double-apply on every FK-seeded hold. The discrimination is only available at
ingest, where the pose's provenance is still known.

So the invariant is deliberately distributed across an *enumerated* set, and the
enumeration is what `tests/ros/test_levelling_frame.py` freezes. Anyone reading
"single enforcement point", counting six call sites and concluding the contract
is already violated would resolve the contradiction the cheap way — by relaxing
the contract text or by collapsing the sites into the emitter. Both are wrong,
for the reasons below.

### The second half is the mirror bug, not decoration

`trajectory_node` seeds holds by forward kinematics from the encoders. Once
plan-space "level" *is* `rx = −0.7788°`, an FK-derived seed reads −0.7788° on its
own; routing that seed through the correction as well yields **−1.5576°** — the
same bug with the sign of the mistake flipped. Measurement-derived and
plan-derived poses are already in the corrected frame *by construction*. The
negative-half tests (every derived surface gets **exactly zero** applications)
exist to catch precisely this.

## The enumerated ingest sites

`E` = external ⇒ corrected exactly once. `D` = derived ⇒ never corrected.
Table A of `plans/active/levelling-frame-contract.md` § Phase 0 — Outcome carries
the full nine `D` rows and the four direct plan installs; the `E` rows are
normative and are reproduced here.

| # | Ingest surface | Corrected in | Planner entry |
|---|---|---|---|
| E1 | `platform_pose_topic` (SpaceMouse/GUI follower) | `_on_platform_pose` | `self._follower.follow` → `planner.build_follow` / `build_graceful_stop` |
| E2 | `catch/dynamic_target` | `_catch_target_from_msg` | `planner.build_catch` |
| E3 | `trajectory/go_to_pose` | `_pose_from_msg` | `planner.build_move` |
| E4 | `trajectory/timed_target` | `_pose_from_msg` | `planner.build_timed` (`target_pose`) |
| E5 | `trajectory/go_home` — the neutral constant | `_corrected_neutral_pose` | `planner.build_return_to_neutral` |
| E6 | `trajectory/timed_target(hold_after=False)` — the neutral constant as the planner's RETURN target | `_corrected_neutral_pose` | `planner.build_timed` (`neutral_pose`) |
| B1 | `platform_pose_topic` in `mpc_bridge_node` | `_on_platform_pose` | none — a pure ZMQ translator, it reaches `planner` zero times |

**E6 is why the enumeration matters.** It was found by the Phase 0 audit, not by
the original diagnosis, and it is a *second, independent* use of the same neutral
constant inside a call that already carries another external pose. `build_timed(…,
target, …, neutral_pose=neutral)` is **one** call node carrying **two**
independent external poses. Any guard keyed on "the target argument of the
enclosing function" is structurally blind to that whole class — which is why the
bypass test keys on the **pose-bearing arguments**, plural, per planner entry.
`build_catch` also accepts `neutral_pose`, so a one-line future change
(`hold_after=False, neutral_pose=…` in `_plan_and_install_catch`) would add a
**seventh** external ingest inside an existing call. The widened key is what
catches it.

### The three neutral reads that are NOT targets

`_neutral_pose` has one definition and five reads. Two are targets (E5, E6). The
other three must stay uncorrected, and each is uncorrected for its own reason:

- `_current_state()`'s pre-seed fallback returns `self._neutral_pose.copy()` when
  no plan is installed. It is a **seed**, not a target, and it is reachable only
  before the first telemetry seed.
- `_last_pose`'s initialiser is the same pre-seed-only seed value; the first
  emitted frame or the first seed overwrites it.
- `_svc_go_home`'s response string reads `_neutral_pose[2]` to format
  `returning to neutral (0,0,170)`. It is a **log format** reading the *position*
  component, which the correction never touches.

## Placement — at ingest, never in the emitter

Two independent arguments, either of which is sufficient:

**1. The feasibility gate must measure the object the emitter runs.** An
emitter-side correction would put the correction *outside* the gate:
`feasibility.validate` would measure an uncorrected plan while the legs execute a
corrected one. A 0.78° tilt is a small leg-space perturbation today, but the
gate's entire value is that it measures the object that actually ships
(`planner.build_catch`'s "single-gate contract" comment makes the same argument
for the assembled catch plan). Ingest-side correction keeps
**plan == emitted == gated**.

**2. It gives the in-flight rule for free, and the alternative puts a step on the
wire.** See below.

## The in-flight rule

> **A live plan keeps the frame it was built in.** A `/gravity_offset` that
> arrives while a plan is executing does **not** re-frame that plan; the next
> plan install picks up the new correction.

This is not a policy choice bolted on afterwards — it is what ingest-side
placement *already does*, because the correction is baked into the plan's
endpoint at **build** time and nothing re-reads the stored `R` for an installed
plan. The alternative is what makes it load-bearing: an emitter-side correction
would step `u0` by the full offset delta on the very next 25 ms knot. Measured
for the 2026-07-25 session offset (probe, 2026-07-25): **2.7736 mm = 0.03908 rev
on one knot** — an instantaneous 110.9 mm/s / 4438 mm/s² leg transient that the
pump's 0.3 rev step gate **would not catch**, because it is only 13% of it. That
is a command discontinuity on the wire, not merely an ungated plan.

The operator levels *after* launch, so the correction genuinely does arrive
mid-session. It is stored once per `/gravity_offset` message and read at each
ingest — which is also why the neutral constant is corrected **at use** and never
at construction: correcting the stored constant would double-apply the moment a
new offset landed.

## Consequences at the machine

These are correct behaviour, and each has been mistaken for a fault before.

- **The resting platform sits ~0.78° off its own frame.** After levelling it is
  level with respect to *gravity*, at all times, including at rest and at
  `go_home`. Total tilt for the 2026-07-25 offset: **0.78185°**.
- **The first `go_home` after `level` is a real move, not a no-op.** ACTIVATE
  TRAP_TRAJs the legs to `JB_OP_ACTIVATE_POSITION_REVS`, the IK of the
  *uncorrected* neutral, so `go_home` to the *corrected* neutral moves the legs
  by up to **2.7736 mm = 0.03908 rev** (per-leg
  `[-1.42, +2.74, +2.77, -0.98, -1.35, -1.75] mm`) over the 2.0 s profile — 13%
  of the pump step gate as a whole-move excursion, 322× the 8.6 µm leg encoder
  dead-band, and therefore visible in `/leg_setpoint_echo`. Peak leg velocity
  2.60 mm/s; per-knot |Δu0| 9.2e-4 rev.
- **Three artefacts pin "commanded level ⇒ activate revs" and all three stay
  true.** `sim/tools/verify_motor_commands.py`,
  `tests/motion/test_trajectory_emitter.py::test_activate_revs_fk_roundtrip_is_neutral`
  and the `HoldPlan(NEUTRAL)` emitter test each construct the level pose
  *directly* and never go near `trajectory_node`. They are statements about the
  IK/emitter layer and about a codegen'd constant. At the **system** level,
  "commanded home == activate revs" is conditional on the loaded correction — and
  none of those tests fails to say so, which is exactly why it is written down
  here.
- **A ballistic aim is now gravity-referenced, and the tilt clamp's two grounds
  have diverged.** `tilt_geometry.MAX_TILT_DEG = 12.0` is clamped on the
  *required aim* before the request (`toss_release.py`, gravity-referenced). The
  commanded plan-frame tilt the legs must track can now exceed it by the
  correction magnitude — up to **12.78°**. The clamp comment cites two grounds,
  *"tilt tracking and the small-angle aim model both degrade past ~12°"*, and
  those two quantities no longer coincide. Accepted deliberately: ~12° is a `bb`
  Rung-0 characterisation, not a mechanical limit; the excess is 6.5%; it is only
  reachable at maximum Tier-8b displacement; and `feasibility.validate` still
  measures the **corrected** commanded pose. If a tilt-tracking measurement at
  11–13° later shows the ceiling is harder than the characterisation implies, the
  fix is to lower `MAX_TILT_DEG` — **not** to move the clamp.

## The correction can be silently absent — so it is now observable, and the toss refuses

**The delivery hazard is UNCHANGED and is not closeable by this contract; what
changed (2026-07-26, plan Phase 3) is that it is no longer silent.** Stated here
because "what level means" is what this document is for.

| Leg | Reality |
|---|---|
| publish | `/gravity_offset` is published with **default (VOLATILE) QoS**, not transient-local |
| frequency | once per `level`, plus **one** latched auto-push on the first IDLE after orchestrator boot |
| storage | per-process, in memory, in each subscribing node — **no re-request path** |
| observability | `trajectory/status.gravity_correction_loaded` — **the applier's own answer**, 5 Hz. `mpc_bridge_node`'s copy is still unobserved; it is dropped from the launch and reaches `planner` zero times, so nothing gates on it. Give it the same field if it is ever revived |

Therefore: if `trajectory_node` restarts after a `level` — a crash, or precisely
the `colcon build` + relaunch that any change to this package requires — its
correction reverts to identity, and `RobotState.levelling_complete` **still
reads True**, because that is a Teensy-persisted per-boot flag that says nothing
about any ROS process's memory. That is a *third* pair of meanings for "level",
produced by the same class of reasoning this contract exists to close.

### C-LEVEL-1.O — the observability half

> Whether a correction is loaded is answered **only** by the node that applies
> it, on `trajectory/status.gravity_correction_loaded`, and only while that
> status is **fresh**. `RobotState.levelling_complete` is not that answer and
> must never be substituted for it. The flag records that an offset was
> *received and decoded into a usable correction*, **not** that it was
> non-zero: a genuinely level machine measures a zero tilt whose correction is
> the identity, and it is levelled. "Usable" is load-bearing — a message with
> fewer than two values, or with a non-finite one, is dropped and the flag
> stays False.

Four ways to get this wrong, and what each costs:

- **Gate on `levelling_complete`.** It passes in exactly the state the gate
  exists to refuse — a relaunched node holding identity — and now with false
  assurance attached, which is strictly worse than no gate at all.
- **Gate on `R != I`.** A correctly-levelled machine with a zero measured offset
  is refused forever, and the operator learns to bypass the gate.
- **Trust a cached True.** Between a `trajectory_node` dying and its
  replacement's first status there is no message to flip a consumer's cache, so
  a consumer without an expiry answers with the dead process's frame. Hence the
  freshness requirement, and hence the flag is published at 5 Hz rather than
  latched once.
- **Set the flag for a message you could not use.**
  `levelling.correction_from_offset` does no finiteness validation, so a NaN or
  inf offset would be negated straight into the stored rotation. The flag would
  read True, the toss would pass CHECKING, and the NaN would surface only
  downstream as a POSITIONING feasibility rejection — *after* the goal has
  claimed the platform. `_on_gravity_offset` therefore drops short **and**
  non-finite messages without setting the flag, keeping the failure where the
  contract wants it: loud, and before the throw.

**Enforcement**: `toss_sequencer._step_checking` refuses the throw with
`REJECTED_NOT_LEVELLED`, ordered after the `mocap_fresh` / `streaming` gates
(a stale graph makes the flag unknowable, and a misleading reject code sends the
operator to the wrong subsystem) and before the hand-evidence chain (those gates
describe *how* the throw is dispatched; this one says the throw cannot be caught
wherever it is dispatched from). **One honest gap in that ordering**: `streaming`
is a sticky last-value with no freshness stamp of its own — deliberately left
alone, because the reload FSM consumes it and changing when a reload refuses is
not this contract's business — so a `trajectory_node` that dies outright
surfaces as `NOT_LEVELLED` (its status stops, the flag expires) rather than as
`NOT_STREAMING`. The reject-code table in
`tests/hardware/toss_trace_recorder.py` names both causes so the operator is not
sent to re-run `level` against a dead node. The justification is geometric, not
procedural:
un-levelled, the launch leaves the cup 0.78° off gravity, which is
`v·sin θ·T = 3.93 m/s × sin 0.78° × 0.8 s =` **43 mm** of lateral drift against a
**~35 mm** cup radius (`GEOM_HAND_RADIUS_MM`). The catch is geometrically
impossible before the ball is in the air.

With `T = 2v/g` and `v = sqrt(2gh)`, that drift is exactly **`4·h·sin θ`** —
linear in apex height, independent of everything else. 43 mm at the ~0.79 m
config-default apex; 33 mm at the 0.6 m the operator runbook uses. The gate is
deliberately **height-independent** all the same: it asks whether the machine
knows where gravity is, not whether one particular goal happens to fit in the
cup, and the cup's usable tolerance is the 35 mm radius less the ball radius
anyway. Do not turn this into a per-goal drift budget — that is the "relax the
invariant for this one case" move this contract exists to refuse.

**What this does NOT do**: it does not redeliver a lost correction. Transient-local
QoS on `/gravity_offset` was considered and **not** taken — see the plan's
Phase 3 outcome for the three failure modes that decided it, the load-bearing one
being that the latch lives in the *publisher*, so the whole-graph relaunch that
motivates this hazard would not benefit at all. The operator runbook's standing
requirement therefore stands unchanged: **level manually after every launch or
relaunch.** What is new is that forgetting costs a loud refusal instead of a ball
on the floor.

Two operational facts that make this the common case rather than a corner:
`levelling_complete` is "since last Teensy bootup", and the operator power-cycles
the can-bridge Teensy before every sitting — so it is False at every launch and
the persisted auto-push never fires first. **In practice every session genuinely
needs a manual `level`.**

## Enforcement

| Part | Where |
|---|---|
| normative statement | this document |
| shared implementation | `ros_ws/src/jugglebot/jugglebot/motion/levelling.py` |
| unit tests (sign, order, round-trip, pinned example) | `tests/motion/test_levelling.py` |
| structural + behavioural bypass test | `tests/ros/test_levelling_frame.py` |
| C-LEVEL-1.O — the applier's affirmation | `trajectory_node._publish_status` → `TrajectoryStatus.gravity_correction_loaded` |
| C-LEVEL-1.O — the consumer + its expiry | `reload_coordinator_node._build_toss_observations` (`_TRAJ_STATUS_STALE_S`) |
| C-LEVEL-1.O — the refusal | `toss_sequencer._step_checking` ⇒ `REJECTED_NOT_LEVELLED` |
| C-LEVEL-1.O — tests | `tests/ros/test_trajectory_node.py` (loaded / zero-offset / malformed / restart), `tests/ros/test_toss_coordinator.py` (freshness table, restart end-to-end, persisted-push-alone passes), `tests/ros/test_toss_sequencer.py` (gate order) |

The structural test `ast.parse`s the live package (never imports it, so it needs
no ROS mocking), discovers every module containing a planner entry or a
`levelling` application, and asserts both the discovered **file sets** and the
per-entry manifests match frozen data carrying each entry's `E`/`D`
classification. A new planner entry, a new pose-bearing argument on an existing
entry, a planner entry in a new module, or a `levelling` application in a new
function all fail until the author adds a row and *declares* the classification.

Two call forms the guard depends on, and pins for that reason. Both manifests key
on the **dotted source name** (`planner.build_*`, `levelling.*`), and the
behavioural counting fixture patches `correct_pose` on the `levelling` module
object — so a direct `from jugglebot.motion.levelling import correct_pose` would
un-instrument the structural half AND the counting half at once, silently, and
both the omission bug and the mirror bug would then land green.
`test_planner_and_levelling_are_reached_by_attribute_access_only` forbids the
direct form (empty allow-list; teach `_Collector` first if one is ever wanted).
Separately, an ingest can reach a planner entry **without any tracked call** by
assigning plan-feeding node state that another scope reads —
`_on_platform_pose` → `self._follower_target` → `_follower_tick` is exactly that
shape — so the writer set of `_follower_target` is frozen by
`test_follower_target_writers_are_frozen`.

**What the guard deliberately does not cover.** It is not a general "no new
plan-feeding node state" guard: `_follower_target` is frozen because it is the one
field a *pose-bearing external message* writes for another scope to consume. A
future ingest that invents a second such field, or that installs a
`TrajectoryPlan` directly without going through `planner`, is outside its reach —
step 1 below (the E-vs-D decision) is then the only thing standing between it and
the defect. `sim/toss_gate.py` calls
`planner.build_move` / `build_hold` / `build_catch` directly and the AST pass will
never see it — correctly, because `sim/` has no gravity-offset concept, so there
is no correction to omit and no failure mode. The manifest's scope is the ROS
package, not "every caller of `planner`". `ros_ws/src/jugglebot/jugglebot/archived/`
holds two dead copies of the old inline transform; neither has a `console_scripts`
entry and neither is imported by any live module, so they are excluded from both
the manifest and the grep counts. Deleting archived code is a separate concern.

## If you are adding a new pose surface

1. Decide **E or D**: did this pose enter from outside the node (a service
   request, a wire target, the neutral constant), or was it derived from
   measurement or from an existing plan? If you cannot answer that in one
   sentence, you have found a design problem, not a paperwork problem.
2. `E` ⇒ apply `levelling.correct_pose(pose, self._gravity_correction)` **once**,
   at ingest, before the pose reaches any planner entry. `D` ⇒ apply nothing.
3. Add the row to the manifest in `tests/ros/test_levelling_frame.py` with its
   classification, and add the row to the E-table above if it is external.
4. Never apply the correction to a linear velocity or a position. `twist` on
   `TimedTarget` and `target_vel` on `DynamicTargetCommand` are externally
   supplied and are deliberately **not** corrected: the correction is a bias on
   the commanded *rotation*, not a re-expression of the platform frame. Rotating
   a linear velocity would be a category error, it would be invisible in
   leg-space telemetry, and today it would land green because `build_catch`
   discards the twist and `timed_target` has no production client.
