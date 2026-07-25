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
  `tests/ros/test_levelling_frame.py::test_catch_through_seat_still_aims_off_the_plan_frame_tilt`.
  Closing it changes commanded motion at ball contact on every catch including the
  shipping reload path — an operator decision, deliberately not taken.

Both were originally attributed to the frame plumbing in this document, in the
plan and in the operator runbook. They are not, and a reader who measures ≈16 mm
after this contract lands and re-audits the six ingest sites is chasing the wrong
tree — which is what the "`go_home` is a no-op" docstring already cost this
investigation once.

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

## Known hazard — the correction can be silently absent

**Unclosed as of Phase 2; Phase 3 of `plans/active/levelling-frame-contract.md`
owns the closure.** Stated here because "what level means" is what this document
is for.

| Leg | Reality |
|---|---|
| publish | `/gravity_offset` is published with **default (VOLATILE) QoS**, not transient-local |
| frequency | once per `level`, plus **one** latched auto-push on the first IDLE after orchestrator boot |
| storage | per-process, in memory, in each subscribing node — **no re-request path** |
| observability | the loaded correction is published on **no topic at all** |

Therefore: if `trajectory_node` alone restarts after a `level` — a crash, or
precisely the `colcon build` + relaunch that any change to this package
requires — its correction silently reverts to identity, and
`RobotState.levelling_complete` **still reads True**, because that is a
Teensy-persisted per-boot flag that says nothing about any ROS process's memory.
That is a *third* pair of meanings for "level", produced by the same class of
reasoning this contract exists to close.

Until Phase 3 lands, the operator runbook's standing requirement stands: **level
manually after every launch or relaunch.** A gate that consumes
`levelling_complete` alone would *pass* in exactly the state it exists to refuse.

## Enforcement

| Part | Where |
|---|---|
| normative statement | this document |
| shared implementation | `ros_ws/src/jugglebot/jugglebot/motion/levelling.py` |
| unit tests (sign, order, round-trip, pinned example) | `tests/motion/test_levelling.py` |
| structural + behavioural bypass test | `tests/ros/test_levelling_frame.py` |

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
