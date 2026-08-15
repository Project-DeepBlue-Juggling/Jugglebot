# The Levelling Frame — contracts C-LEVEL-1 and C-LEVEL-2

**Normative.** This document specifies where the gravity-levelling correction is
applied and — just as importantly — where it must **not** be. It is the written
half of the repo's contract pattern (normative statement + one shared
implementation + a test that fails on the omission); the other two halves are
`jugglebot/motion/levelling.py` and `tests/ros/test_levelling_frame.py`.

**C-LEVEL-1** (the single measured offset) is the whole of the machine's
levelling behaviour until a calibration map exists. **C-LEVEL-2** adds a
*pose-dependent residual* on top of it and is a strict refinement: it changes
what number the correction is built from, never where or how often the
correction is applied. Everything C-LEVEL-1 says about placement, enumeration,
the in-flight rule and the mirror bug applies unchanged. Read C-LEVEL-1 first;
C-LEVEL-2 is meaningless on its own.

Scope: the ROS 2 package `ros_ws/src/jugglebot/`. Neither `sim/` nor
`controller/` has any gravity-levelling concept — grepped 2026-07-25, the only
hit outside this package is the `levelling_complete` **boolean flag** in
`teensy_link/rpc_args.py`, which is a Teensy-persisted per-boot flag
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

Two operational facts made this the common case rather than a corner:
`levelling_complete` is "since last Teensy bootup", and the operator used to
power-cycle the can-bridge Teensy before every sitting — so it was False at every
launch and the persisted auto-push never fired first. **In practice every session
genuinely needs a manual `level`.**

> **Amended 2026-08-15 — the second fact no longer supports the conclusion, for
> two independent reasons.** (i) The power-cycle-before-every-sitting rule is
> **retired**: its cause, the can-bridge uptime lag, was fixed in FW 14 and
> validated at 5.8 h and 15.2 h (`logbook/2026-08-15-fw14-validated-arc-closed.md`),
> so a bridge left powered across sittings is now the normal case. (ii) The
> reasoning was already suspect: `levelling_complete` and `pose_offset_rad` are
> persisted by the **Platform** Teensy, and a *can-bridge* power-cycle does not
> clear that cache — `tests/hardware/session_anomaly_fixes.md` standing rule 2 and
> operator pre-brief item 2 both say so explicitly, while pre-brief item 1 asserts
> the opposite. That contradiction is unreconciled in the record and is flagged
> here rather than silently resolved.
>
> The requirement above is unchanged — **level manually after every launch or
> relaunch** — but do not rely on the boot state to force it, and treat the
> orchestrator's persisted auto-push as a live path that can win the discovery
> race.

## C-LEVEL-2 — the pose-dependent residual map

> The gravity-levelling correction may be built from the single measured offset
> **plus a pose-dependent residual** read from a calibrated (x, y) map. The
> residual is **added to the offset in rotation-vector space** and passed
> through the **same single Rodrigues** as C-LEVEL-1, at the **same enumerated
> ingest sites**, **exactly once per external pose**, keyed on the
> **uncorrected intent pose**. Outside the calibrated hull the lookup **clamps**
> — it never extrapolates and never yields NaN. The map is **non-gating**:
> absent, rejected, **or held dormant while the node is unlevelled**, behaviour
> degrades to *exactly* C-LEVEL-1.

### What the map is, and what a residual means

`level` measures the platform's tilt against gravity at **one** pose (the
firmware ACTIVATE pose) and applies that offset at **every** pose. Any
pose-dependent kinematic error — geometry, compliance, assembly — is invisible
to it by construction. Measured on 2026-07-28 (`logbook/2026-07-28-anomaly-fixes-validation-sitting.md`,
the extremity-tilt table): commanded-level platform tilt error grows from
**0.041° at (60, 0) to 0.604° at (150, −150)**, is **not** linear in (x, y), and
repeats to 0.001–0.014° — 15–40× smaller than the effect. At 41.9 mm/° of
landing displacement (0.6 m toss) against a ~30–40 mm cup basin, the corner of
the workspace is already the "occasional drop" regime.

**The sensor, stated plainly.** The Murata SCL3300 inclinometer is mounted
**rigidly to the platform** and never moves relative to it. It always reads
**absolute inclination with respect to gravity** — it is not a relative or
incremental sensor, and it has no notion of the platform's commanded pose. Its
axes are **not perfectly aligned** with the platform frame, which is the whole
reason for the small **constant** mounting offset
`inclinometer_offset_deg = [-0.6, -0.2]` deg
(`config/hardware_config.yaml`, experimentally found). The residual formula
below adds that offset **exactly as `LevellingHandler` does** — same constant,
same sign, same axis order — so a residual and a `/gravity_offset` are the
same kind of object and can be added.

A **residual** is what `level` cannot see — and the map is **ANCHOR-MEAN
HOME-REFERENCED** (2026-08-10) so that measuring it depends neither on the
level reference's freshness nor on which single arrival at home happened to be
used as the reference. At grid node *i*, with **any constant correction
loaded** and **no map loaded**, the platform is commanded to the *level
orientation* at `(xᵢ, yᵢ, z_grid)`, allowed to settle, and the inclinometer
read N times. The tool measures at each node

    m_i = mean(raw_reading) + radians(inclinometer_offset_deg)

— the exact `LevellingHandler` formula, so `m` is dimensionally and sign-wise
the same object as the `/gravity_offset` the single-offset path already
carries. The home pose is re-measured **k times through the sweep** — at the
start, after every `--home-revisit-every` non-home grid visits (default 4:
k ≈ 3 on a 3×3, k ≈ 7 on the 5×5 default), and at the end — and the tool
**ships**

    M(Pᵢ) := m_i − mean over the k anchors of m_home

The algebra is why freshness stopped being a capture precondition: writing the
platform's true pose-dependent field as `f(P)` and the loaded correction as
`C_cap`, every measurement is `m_i = f(Pᵢ) − C_cap` (the mounting bias cancels
inside the formula above), so

    M(Pᵢ) = m_i − m̄_home = f(Pᵢ) − f(home)

— `C_cap` cancels **exactly**, whatever it is, however stale it is, as long as
it is **constant across the sweep**. With zero anchor scatter this reduces
**bit-identically** to the single-anchor form it replaces, and `map(home) = 0`
exactly; with real scatter the home node ships its own (sub-mrad) departure
from the anchor mean, which is correct and is deliberately not forced to zero.

**Why the mean, and not a time interpolation.** Both C0 captures on 2026-08-10
completed cleanly and then re-measured home **+1.81 / +1.59 mrad on ty**
against the start — reproducibly, with the causal `/gravity_offset` monitor
silent and the ODrive forensics clean, so the loaded correction had *not*
changed. The mechanism is **open**, with the owner's **arrival-repeatability**
hypothesis leading: Jugglebot is hand-built from FDM-printed parts, and
re-arriving at a pose after a workspace tour lands within ~1–2 mrad of tilt
with path-dependent hysteresis — deterministic enough that two identical sweep
sequences reproduced the same offset. **Sensor warm-up / thermal settling**
after the ODrive power-cycle fits the same data. The anchor series recorded on
every future capture discriminates them for free (repeatability scatters about
a mean; warm-up trends monotonically), so no dedicated sitting is owed.

The mean is the right estimator under **both**: it averages per-arrival noise
as `σ/√k` under repeatability, and centres a smooth drift at `±` half its
total under warm-up. A time interpolation would presuppose the drift
hypothesis, and both residuals are far inside the owner-stated repeatability
tolerance of **0.5° (8.7 mrad)** and the 0.15° `θ_acc` — the design responses
here are at the ~0.1° scale, i.e. inside both bounds either way. What remains
of the reference dependence: a second-order vector-vs-rotation term bounded by
`|Δf × m_home| / 2` ≈ **5.3e-5 rad** at the 0.010 rad `|m_home|` WARN bound
(inside the 1e-4 budget of the composition table below), and the systematic
`f(home) − f(activate)` — same position, arrival and orientation history
differing — which **replaces** the previous design's stochastic apply-time
error `ε_cap − ε_now` (σ ≈ 1.7 mrad, the level reference's own single-sample
scatter): a strict improvement unless rung C0's tilted-pose probe surprises.

The previous design asserted `residual(home) ≈ 0 by construction` and ABORTED
the capture when a start-of-capture gate found otherwise ("stale level
reference"). That gate was **mistuned against physics and structurally
untunable** (2026-08-09 C0 sitting, six attempts): the level reference is ONE
int16-quantised SCL3300 sample (LSB = (90/2¹⁴)° = 9.587e-5 rad) whose measured
session-scale scatter is σ ≈ 1.2–1.7 mrad/axis, so the gate's 1.5 mrad floor
sat *inside* the healthy distribution (≈40–60 % false aborts per attempt over
two axes) while a correctly-sized 3σ floor (~3.6–4.6 mrad) collides with its
own 5 mrad staleness ceiling. Home-referencing removes the quantity that gate
was trying to protect.

Its first replacement — a **tight statistical drift gate** on the start-vs-end
home means — was retired the same day it first ran, for the same class of
reason: it aborted **both** complete C0 captures (2026-08-10) over the ~1.6–1.8
mrad ty offset described above, against a read-noise-derived tolerance of
0.75–0.94 mrad. Discarding a finished, usable capture over an effect **an
~4.8x inside the owner's repeatability tolerance (1.8 mrad of 8.7)** was the defect;
the measurement was fine. **The abort-and-discard behaviour was the bug, not
the measurement.**

What enforces the reference now:

- **Anchors are REPORTED on every capture** — the per-anchor table (`t_s`,
  `m_tx`, `m_ty`, sd, `n_ok`), the per-axis peak-to-peak spread, and the signed
  start-to-end trend. The series is evidence, not just a gate input.
- **WARN** (prominent, non-blocking) when any axis's anchor p-p exceeds
  **0.002 rad**. The message names both open mechanisms — arrival
  repeatability of the hand-built FDM structure vs sensor warm-up / thermal
  settling — and says the series across captures will discriminate them.
- **ABORT** only when any axis's p-p exceeds **0.0087 rad (0.5°, the
  owner-stated repeatability tolerance)**, or a single **consecutive-anchor
  step** exceeds **0.005 rad**. A step between two adjacent anchors is a
  discrete *event* — a re-level, a relaunch re-pushing the int16-mrad Teensy
  copy, a mechanical snap — which is a different physical claim from smooth
  wander of the same total size.
- The **causal `/gravity_offset` mid-sweep monitor** is unchanged and remains
  the detector for a correction that actually changed (any message during the
  sweep aborts).
- A `|m_home|` **WARN at 0.010 rad** (level grossly stale — harmless to a
  home-referenced map, worth surfacing) and a **hard abort at 0.05 rad**
  (aligned with `MAX_ABS_RESIDUAL_RAD`).

Smooth sub-WARN variation of the home reading across a sweep is **accepted and
reported**, not a failure.

**Verification scoring is home-referenced too.** A check pose measured after
the map is applied reads `≈ m_home` even when the map is perfect — the
constant the capture correctly tolerated is still in the loop — so the
verification pass re-measures the home node first and scores every check pose
as `|check − m_home_verify| ≤ θ_acc`. Scoring the raw check residual would
fail every pose under a stale-but-constant level whose capture was perfectly
valid.

### Composition — additive in the rotation vector, one Rodrigues

    combined = level_offset + bilinear(map, intent_xy)
    R        = correction_from_offset(combined)          # the C-LEVEL-1 Rodrigues

The two tilts are **summed as rotation vectors**, not composed as two rotations.
The exact alternative — `R(map) @ R(level)` — differs at second order only, and
the second-order term is exactly

    |level_offset × residual| / 2

(probe, 2026-08-02: the closed form matches the measured matrix difference to 5
significant figures across the whole regime). **State the regime with the
number, because the bound is not universal**:

| Regime | Worst-case error | Verdict |
|---|---|---|
| the measured envelope — 0.78185° level offset × 0.604° residual, orthogonal | **7.19e-5 rad** | holds, 28% margin |
| 1° × 1° orthogonal — the corner of "sub-degree" | **1.523e-4 rad** | **exceeds 1e-4** |
| 0.0136 rad offset × the `MAX_ABS_RESIDUAL_RAD` 0.05 rad ceiling | **3.40e-4 rad** | 3.4× over |
| 0.05 × 0.05 rad — what validation alone permits | **1.25e-3 rad** | 12.5× over |

So: the error stays under 1e-4 rad while the *product* of the two magnitudes
stays under 2e-4 rad² (both axes below **0.810°** if they are equal), which the
measured machine satisfies with margin — 7.19e-5 rad is 0.0041°, i.e. 0.17 mm at
the 41.9 mm/° conversion, against a 0.04–0.6° signal. It does **not** hold
merely because a map passed validation, and the earlier draft of this section
claimed "< 1e-4 rad for sub-degree tilts" flatly, which is false at 1° × 1°.
`tests/motion/test_tilt_map.py::test_additive_composition_second_order_bound_over_the_regime`
sweeps the regime rather than sampling one point inside it, and pins **both**
the measured-envelope figure and the 1.523e-4 sub-degree worst case — because a
numeric claim in a normative document that no test measures is a claim that
rots, and a test that measures one lucky point is the same claim wearing a
lab coat.

**If a future capture ever runs at a tilted base with a large `level` offset**
(rung C2 shims the base by 1–2°), re-read that table before assuming the
additive form is still free: at 2° × 1° the term is 3.0e-4 rad. It is still
small against the signal, but it is no longer three orders below it, and the
honest fix at that point is matrix composition, not a wider adjective.

The same second-order arithmetic bounds what home-referencing leaves behind at
**capture**: the constant reference cancels at first order in
`M(P) = m(P) − m(home)`, and the remaining vector-vs-rotation term is
`|Δf × m_home| / 2` — 5.3e-5 rad at the 0.010 rad `|m_home|` WARN bound,
2.6e-4 rad at the 0.05 rad hard abort, which is why the abort sits there.
(Bilinear interpolation is indifferent to the whole scheme: a constant shift
commutes exactly through the interpolation weights, so
`bilinear(M) = bilinear(m) − m_home` with no approximation.)

Additive composition is preferred for a reason beyond arithmetic: it keeps the
number of rotations composed onto a commanded pose at **exactly one**. The
C-LEVEL-1 module docstring's warning — that the composition is non-commutative
and a re-derived copy in the wrong order is a *silent* frame error — applies to
every rotation added to that chain. One Rodrigues has no order to get wrong.

### Keying — the uncorrected intent pose

The lookup keys on `pose[0:2]` of the pose **as it entered the node**, before
this correction rewrites its rotation. Keying on the *corrected* pose would make
the lookup depend on its own output and invite a fixed-point iteration; the
intent pose makes it a single evaluation with a closed form. Position is never
touched by the correction (C-LEVEL-1, rotation-only), so the intent pose's x/y
*are* the commanded x/y — the two only diverge under a tilted base, and by
`sin(tilt)·displacement` (≈5 mm at 2° over 150 mm), which is an accepted and
documented limitation of the whole rotation-only design, not of the keying.

The rotation components never key the lookup. The map is captured at the level
orientation and applied additively at **all** commanded orientations (8b
tilt-aims to 5.75°, reload receive tilt 10.8°). Orientation-dependence of the
residual is **unmeasured** — rung C0 of `plans/archived/2026-08-15 tilt-calibration-grid.md`
sizes it with one tilted-pose probe; a tilt axis would be a follow-on sweep, not
a licence to key on rotation.

### Evaluation happens at ingest, per target — a per-knot lookup is FORBIDDEN

> The map is evaluated **once per external pose, at the ingest sites C-LEVEL-1
> enumerates**. Evaluating it per emitted knot, per interpolated sample, or
> anywhere downstream of `feasibility.validate` is a contract violation.

The idea is superficially attractive — a moving platform passes through the
field, so "correct continuously" sounds strictly better. It is disqualified
three times over, and each disqualifier is independently sufficient:

1. **It desyncs the wire's own derivatives.** The 500 Hz leg interpolator on the
   can-bridge Teensy (`Teensy_code_canbridge/leg_interp.cpp`) is a cubic Hermite
   over `u0/u1/u2` and the **declared** velocities. Rewriting positions per knot
   without rewriting the declared velocities they were derived from makes the
   position ladder and its declared derivative describe different motions —
   the interpolator then smooths between two inconsistent stories at 500 Hz, on
   the safety-authority side of the link.
2. **It escapes the feasibility gate.** `feasibility.validate` measures the plan
   at build time. A correction applied after that measures one object and ships
   another — the exact failure C-LEVEL-1 § "Placement" already refuses for the
   emitter-side variant. The gate's entire value is that **plan == emitted ==
   gated**.
3. **It is invisible whether it is right or wrong.** The per-knot delta is a
   fraction of the whole-move excursion C-LEVEL-1 measured at 0.03908 rev — far
   below the pump's `MAX_POSITION_STEP_REV = 0.3` step gate and the firmware's
   `MAX_DEVIATION_REV = 1.0` guard (`Teensy_code_canbridge/hardware_config.h`,
   `canbridge_config.h`). A correct per-knot lookup and a buggy one produce the
   same telemetry signature: nothing. Debugging it would mean reasoning about a
   quantity no guard can see.

The physical cost of *not* doing it is bounded and small: residual transit error
is `map-gradient × move length`, and it is **exactly zero at the terminal hold**
— which is where both throws and calibration reads happen. Throws leave from a
stationary hold; nothing is thrown mid-transit.

### Outside the calibrated hull — clamp, never extrapolate

A query beyond the grid is clamped into `[x₀, x_N] × [y₀, y_M]` and then
interpolated, so it returns the nearest hull point's residual. Extrapolation is
refused because a wrong-signed edge extrapolation aims a throw **worse than no
map at all**, and the grid's edge is precisely where the field is least linear
(the 07-28 table refutes a 2-gain fit). NaN is refused for the same reason it is
refused on `/gravity_offset`: it would be negated straight into the commanded
rotation and surface only downstream, after a goal has claimed the platform. A
non-finite *query* raises rather than propagating.

### Load validation — all-or-nothing, and loud

A map is loaded only if **every** one of these holds:

| Check | Why it is not optional |
|---|---|
| schema `version` is a known integer (`1`) | a future version may change what the numbers *mean*; best-effort parsing of an unknown schema is a silent frame error |
| both axes strictly increasing, ≥ 2 nodes | a repeated node is a zero-width cell (divide-by-zero in the weights); a single-node axis is a constant offset masquerading as a map |
| `residual_rad.tx` / `.ty` shaped `[len(y_mm)][len(x_mm)]` | catches the transposed grid, which is invisible on a square/symmetric capture |
| every node finite | a failed capture node must be resolved by the tool, never shipped as NaN |
| every `|residual| ≤ 0.05 rad` (≈2.87°) | the measured signal is 0.04–0.6°; beyond ~2.9° it is a capture fault — a leg pinned on its stroke clamp, or a units error. (A stale level reference is **no longer a candidate cause**: it cancels exactly out of the home-referenced residuals; the capture tool's `|m_home|` WARN is the diagnostic that replaced it.) |

Any failure ⇒ **the map is not loaded**, `tilt_map_loaded` stays false, and the
reason is logged at ERROR. There is deliberately no partial load, no per-node
repair and no zero-fill: a half-trusted map is indistinguishable at the machine
from a correct one until a ball misses. An **absent** file is different from an
invalid one — absence is silent (soft absence, non-gating); invalidity is loud.

### Observability, and why it is not a gate

> `TrajectoryStatus.tilt_map_loaded` (bool) and `.tilt_map_version` (string) are
> the applier's own answer to "is a calibration **loaded**, and which one", on the
> same 5 Hz status as `gravity_correction_loaded`.

`tilt_map_version` is `"<captured.date>-<sha256(content)[:8]>"`, hashed over the
**applied numbers only** — the schema `version`, both grid **axes**, and the two
`residual_rad` grids, float-normalised — and not over the capture provenance:
two files with identical numbers apply the same calibration and must report the
same version, while an edit to a single node must change it. `grid.z_mm` and
`grid.orientation` live inside the `grid` block but are **provenance and are
excluded**, so two captures differing only in height report the same version;
read `TiltMap.z_mm` if you need to tell them apart. (An earlier draft of this
section said "`version` + `grid` + `residual_rad`", which was the pre-Phase-1
behaviour and was fixed in code before it shipped.)

**Nothing gates on either field.** `toss_sequencer._step_checking` is unchanged
— `REJECTED_NOT_LEVELLED` keys on `gravity_correction_loaded` and only on that.
This is deliberate and is the one thing most likely to be "improved" by a future
reader: the map is a **refinement**, and refusing to throw without it would trade
a bounded aim error (0.6° worst-case corner, ≈25 mm at the 0.6 m toss) for a
machine that cannot juggle at all until a calibration exists. C-LEVEL-1's gate
asks whether the machine knows *where gravity is*; C-LEVEL-2 asks how precisely
— a different question, and not a precondition for throwing.

`tilt_map_version` is `''` whenever no map is loaded. **The pair reports what is
LOADED, not what is APPLIED** — a loaded map is *dormant* while
`gravity_correction_loaded` is false (see § "The map is gated on the level
reference"), so `tilt_map_loaded=true` with a non-empty version is a legitimate
state in which no residual is being applied. Read the two flags together:
`tilt_map_loaded && gravity_correction_loaded` is the condition under which the
map is actually shaping commanded poses.

### Delivery — the loader, and why its resolution order is inverted

> The map is loaded from `config/tilt_calibration.yaml`, resolved
> `$JUGGLEBOT_TILT_CAL` → **repo source tree** → ament share dir, at node
> construction and on `trajectory/reload_tilt_map`.

That order is the **inverse** of `motion/friction_ff_params.py`'s, deliberately.
The friction constants are hand-tuned into a committed YAML and a `colcon build`
is their natural publish step, so share-first is right there. The tilt map is
*machine-written at runtime by the acquisition tool, which then asks the running
node to reload it* — share-first would serve the previous build's stale
calibration until the next `colcon build`, silently, with `tilt_map_loaded` true
and a plausible-looking version. That is the exact trap `friction_ff_params.py`
documents in its own docstring, and the capture workflow would walk into it every
run.

**The env override is authoritative: when `$JUGGLEBOT_TILT_CAL` is set it is the
only candidate**, existing or not. Falling through would apply *a different
calibration than the operator named* while reporting success — strictly worse
than applying none, because C-LEVEL-2 is non-gating and applying none costs only
precision.

Four load outcomes, and the second one is a design decision, not a default:

| Outcome | Map | `tilt_map_loaded` | Service `success` | Log |
|---|---|---|---|---|
| valid file | swapped in (single reference) | true | true | INFO with version + grid shape |
| **no file** | **unloaded** | false | **true** | INFO (never had one) / WARN (dropped a live one) |
| no file, but `$JUGGLEBOT_TILT_CAL` **is set** | unloaded | false | true | **WARN naming the path** |
| invalid file | **previous kept** | unchanged | false | ERROR naming the failed check |

*No file ⇒ unload, and the call succeeds.* Reload's contract is "make this node's
map agree with the file", and absence is a legitimate non-gating state. It is
also load-bearing for the acquisition tool, whose `--force-uninstall` moves the
file aside and reloads precisely to reach `tilt_map_loaded == false` before a
capture — capturing with a map loaded bakes the map into its own successor.
Keeping a stale in-memory map there would defeat the capture preconditions
silently, in the one workflow that most needs them.

*A set-but-missing override is not plain absence.* Plain absence is the state
every machine is in before its first capture, so it costs an INFO and nothing
else. A **set** `$JUGGLEBOT_TILT_CAL` pointing at a path that does not exist is
an operator who **named a file and is not getting it** — and because the
override is authoritative, nothing else is tried and the whole session silently
runs uncalibrated. At the default log level an INFO there is indistinguishable
from a normal uncalibrated boot, so the typo survives the session. WARN naming
the path is the difference between "degrades loudly" as a claim and as a
behaviour.

*A map is also warned about when its capture height is off the active plane.*
`grid.z_mm` is provenance — excluded from the version hash by design, never a
lookup key — so a grid captured at one height is applied at every other one and
nothing downstream can tell. The load logs a WARN naming both numbers when it
differs from `JB_OP_DEFAULT_ACTIVE_Z_MM` by more than a few mm. WARN, never
refuse: C-LEVEL-2 is non-gating and an off-plane map is a precision question.

### The map is gated on the level reference — loaded is not the same as applied

> A loaded map is applied **only while a `/gravity_offset` correction is
> loaded**. With `gravity_correction_loaded == false` the ingest sites behave
> **exactly** as C-LEVEL-1 unlevelled — identity — and the map term is skipped
> entirely. The map remains loaded and observable (`tilt_map_loaded` stays
> true, the version is still published): it is **DORMANT**, not unloaded, and
> it begins applying the moment the offset arrives, with no reload.

| State | `tilt_map_loaded` | map applied? | commanded rotation |
|---|---|---|---|
| levelled, map loaded | true | **yes** | offset + residual, one Rodrigues |
| **not levelled, map loaded** | **true** | **no (DORMANT)** | identity — C-LEVEL-1 unlevelled |
| levelled, no map | false | n/a | offset only (C-LEVEL-1) |
| not levelled, no map | false | n/a | identity |

**Why the gate exists.** The shipped residual `M(P) = f(P) − f(home)` is
reference-free at *capture* (§ "What the map is" — the loaded correction
cancels), but *applying* it composes a rotation onto the live correction
`C_now` — and before the first `/gravity_offset` arrives the stored offset is
the `(0.0, 0.0)` **placeholder**, which means "we do not know the tilt", not
"the tilt is zero". Composing a residual onto it commands a rotation
referenced to nothing: not a smaller error than applying no map, a
differently-wrong one. The gate **stands** under home-referencing; what moved
(2026-08-10) is only its justification — from "the residual is defined against
the reference" to "the *application* is".

That state is **reachable at every boot** once `config/tilt_calibration.yaml` is
committed, because the map loads in `__init__` while the offset only arrives when
the operator runs `level` — and every session genuinely needs a manual `level`
(§ C-LEVEL-1's two operational facts). So this is the common path, not a corner.
Degrading to identity keeps it *exactly* what the machine did before this phase
existed, and matches what the rest of the stack already assumes an unlevelled
node does: the toss's `REJECTED_NOT_LEVELLED` gate keys on
`gravity_correction_loaded` alone.

Dormancy is announced once per loaded map (WARN, latched — the announcement sits
on the ingest path, so an unlevelled follower stream would otherwise log at
40 Hz). `tilt_map_loaded == true` while nothing is being applied is exactly the
kind of state this contract refuses to leave silent.

### The ingest degrade path — a lookup that raises must not kill a callback

`tilt_map.lookup` **raises** on a non-finite query rather than returning NaN. A
non-finite intent pose is an upstream defect, and before C-LEVEL-2 it flowed
through the rotation-only correction untouched and was refused downstream by
`feasibility.validate`. The map must not convert that into a dead ROS callback:
each ingest catches `TiltMapError`, applies the **offset-only** C-LEVEL-1
correction to that pose, and warns **once per loaded map** (the fault is a
property of the client, so per-pose logging would bury everything else at the
full ingest rate). Non-gating is a property of the code path, not of an
operator's discipline.

### The in-flight rule is inherited, not restated

A live plan keeps the frame it was built in. A map that is loaded — or reloaded
via `trajectory/reload_tilt_map` — while a plan is executing does **not**
re-frame that plan; the next plan install picks it up. This is not a second
policy: it is C-LEVEL-1's in-flight rule, unchanged, and it holds for the same
structural reason (the correction is baked into the plan's endpoint at build
time, and nothing re-reads the stored `R` for an installed plan). The
alternative would step `u0` on the next 25 ms knot by the full map delta —
which is disqualifier 3 above, arriving by a different door.

### Capture preconditions — constancy, quiescence, an armed wire

A capture run is only meaningful under all five of these:

1. **A correction loaded and CONSTANT throughout the sweep.** The
   home-referenced residual `M(P) = m(P) − m̄(home)` cancels the loaded
   correction exactly, so freshness is **not** required — but a correction
   that *changes mid-sweep* (a re-level; a relaunch re-pushing the
   Teensy-persisted copy, which is int16-mrad truncated, worst 1 mrad/axis
   biased toward zero) splits the sweep across two references and corrupts
   every node measured after the change. A fresh `level` immediately before
   is still **RECOMMENDED**: it keeps `|m_home|` small and the 0.010 rad WARN
   meaningful.
2. **No map loaded during the capture.** A map loaded while capturing bakes
   itself into its own successor — the double-application class, arriving
   through the data rather than the code.
3. **Hand quiescent.** Tilt reads block the Platform Teensy loop that streams
   hand moves.
4. **`uptime_ms` logged first and last.** *(Amended 2026-08-15: the "freshly
   rebooted can-bridge Teensy" half is RETIRED.* Tracking lag used to grow with
   bridge uptime — 10 ms fresh → ~240 ms at 30 h — and a settle-then-read capture
   is exactly the kind of measurement that would silently absorb it. The cause was
   the vendored FlexCAN_T4 `_available` RX-ring leak, fixed in **FW 14** and
   validated at 5.8 h and 15.2 h of continuous uptime,
   `logbook/2026-08-15-fw14-validated-arc-closed.md`. *The `uptime_ms` label
   stands* — it is now how a regression would be caught — and the
   `latency_monitor` row on `/link_status` alarms live during the capture.)
5. **An ARMED wire, for the whole sweep.** `go_to_pose` is *accepted* while
   `mpc_active=0` — streaming-while-disarmed is the legal pre-arm phase — so a
   disarmed capture moves nothing, reads one stationary pose at every node, and
   produces an all-zeros map that passes its own verification *because* nothing
   moved. Added 2026-08-04 after review; the same silent-no-op class as the
   2026-07-15 ramp battery.

**(1), (2) and (5) are machine-checkable, and the tool's enforcement says so.**
(1) became checkable when the map went home-referenced (2026-08-10):
*constancy* is observable — a mid-sweep `/gravity_offset` message and a
discrete step in the home-anchor series are both machine-visible — where
*freshness* never was. An earlier draft claimed the tool "refuses to start" on
all four preconditions, which overstated what any program can observe:

| # | Enforcement in `tests/hardware/tilt_cal_grid.py` |
|---|---|
| 1 | **Machine-checked twice — causally and by outcome** (2026-08-10; plus a typed `yes` so the operator *plans* constancy rather than discovering the abort). Causal: a `/gravity_offset` subscription ABORTS the sweep the moment any message arrives mid-capture — a re-level or a relaunch re-push is *observed*, not inferred. Outcome: the **home-anchor series** (k re-measures of the home pose interleaved through the sweep) is reported in full on every capture, **WARNs** above a 0.002 rad per-axis peak-to-peak spread, and **ABORTS** only above 0.0087 rad (0.5°, the owner's repeatability tolerance) or on a **consecutive-anchor step above 0.005 rad** — the discrete-event signature. The tight statistical drift gate this replaced aborted both complete C0 captures over ~1.6–1.8 mrad of reproducible, mechanism-open home variation; see § "What the map is". `\|m_home\|` additionally **WARNs above 0.010 rad** (level grossly stale — harmless to the home-referenced map, worth surfacing) and **hard-aborts above 0.05 rad** (`MAX_ABS_RESIDUAL_RAD` alignment; the second-order term reaches 2.6e-4 rad there). |
| 2 | **Hard refusal.** `tilt_map_loaded` must be false; `--force-uninstall` moves **every existing candidate** aside (the source tree *and* the ament share copy `setup.py` installs — moving only the source file falls through to the share copy and the old map stays loaded), refusing to touch a `$JUGGLEBOT_TILT_CAL`-pointed file. Re-checked from a **fresh** status immediately before the map is written, so a map that loads mid-sweep still refuses. |
| 3 | **Operator confirmation only.** Nothing in the system observes hand quiescence from this client. |
| 4 | **Warn and record**, never refuse — `uptime_ms` is echoed at preflight, warned above 30 min, and written first/last into both `_meta.json` and the map's `captured` block. Refusing would cost a sitting over an effect measured on the hand-dispatch path, not on this request/response read path. |
| 5 | **Hard refusal, in three layers.** `/link_status` `mpc_active == '1'` at preflight and re-checked from a fresh message before every node and check pose (a silent `/link_status` is itself an abort, because the per-move marker mirrors the same topic with no staleness timeout); the `DISARMED` marker on every **accepted** `go_to_pose` response, screened inside the single `go_to` choke point; and a **flat-field WARN** before the write, which catches a platform that failed to move for a reason the first two cannot observe. The flat-field layer warns rather than refuses — a genuinely flat field is a legitimate result. |

`--yes` skips the confirmations for rehearsal and prints a loud warning when it
does so on a live capture. The confirmations are the constancy *intent* for
(1) — the machine checks catch a violation only after motion has been spent on
it — and hand quiescence for (3), which remains a pure *operator* obligation
the runbook states and the tool records, not a guarantee. That distinction is
the honest one: a tool that claimed to enforce (3) would be trusted for
something it cannot see.

### The map artifact

`config/tilt_calibration.yaml` — **committed, machine-written**, schema v1:
`version`; a `captured` block (ISO date, git SHA, tool + args, can-bridge
`uptime_ms` first/last, the `level_offset_rad` loaded at capture, a
`home_reference` block — `referenced: 'anchor-mean'`, the full `anchors[]`
series (`t_s`, `m_tx_rad`, `m_ty_rad`, `sd_tx_rad`, `sd_ty_rad`, `n_ok`),
`anchor_mean_rad`, `anchor_pp_rad`, `trend_rad`, `anchor_verdict`, plus
`m_home_rad` / `m_home_end_rad` for continuity with pre-anchor captures — and
base-condition free text); a `grid` block (`z_mm`, `orientation: level`,
`x_mm[]`, `y_mm[]`); a `residual_rad` block (`tx[iy][ix]`, `ty[iy][ix]`,
**home-referenced**: node value = measured − `anchor_mean_rad`); and an advisory
`stats` block (per-node sd, `n_reads`, `failed_nodes`). The schema is
unchanged by home-referencing — only what the tool writes into the grids
changed meaning, plus the `captured` metadata gained the home-reference
fields; the loader and apply path are untouched. Committed because it is a *measurement
of this machine* — the same class of artefact as the friction-FF constants — and
machine-written because hand-editing a calibration is how a plausible-looking
wrong number gets in. The full schema is reproduced in
`jugglebot/motion/tilt_map.py`'s module docstring, which is the parser's own
statement of it.

### Revival obligations (no code today)

| # | Surface | Obligation before it may ship |
|---|---|---|
| B1 | `mpc_bridge_node._on_platform_pose` | Dropped from the launch and dormant (`plans/active/refactor-2026-07.md` Phase 3). It holds a **second** copy of the C-LEVEL-1 application and today lacks even the offset validation `_on_gravity_offset` does. Reviving the MPC chain **must** give it the map and the same all-or-nothing validation, or the two pose paths will disagree about where level is — the original C-LEVEL-1 bug, re-created between nodes instead of within one. |

## Enforcement

| Part | Where |
|---|---|
| normative statement | this document |
| shared implementation | `ros_ws/src/jugglebot/jugglebot/motion/levelling.py` |
| C-LEVEL-2 — map parse / validate / bilinear lookup | `ros_ws/src/jugglebot/jugglebot/motion/tilt_map.py` |
| C-LEVEL-2 — the single application entry point | `levelling.correction_for_pose(offset, tilt_map, pose)` |
| C-LEVEL-2 — path resolution | `tilt_map.resolve_tilt_map_path` / `tilt_map.tilt_map_candidates` (pure; the node holds no path policy) |
| C-LEVEL-2 — load / reload / degrade | `trajectory_node._load_tilt_map`, `._svc_reload_tilt_map` (`trajectory/reload_tilt_map`, `std_srvs/Trigger`), `._tilt_map_degraded` |
| C-LEVEL-2 — the applier's affirmation | `trajectory_node._publish_status` → `TrajectoryStatus.tilt_map_loaded` / `.tilt_map_version` |
| C-LEVEL-2 — tests (interpolation, hull clamp, every rejection, additive-composition bound) | `tests/motion/test_tilt_map.py` |
| C-LEVEL-2 — tests (loader, reload, status, per-pose keying, one composition, in-flight, degrade) | `tests/ros/test_trajectory_tilt_map.py` |
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

Its `levelling` manifest carries **three kinds**, and C-LEVEL-2 is why there are
three rather than two:

* `store` — once per `/gravity_offset` **message** (`correction_from_offset`,
  `identity_correction`).
* `build:` — once per external **pose**, at ingest (`correction_for_pose`). It
  reads the residual map at that pose's uncorrected x/y, so unlike a `store` it
  cannot be hoisted to the offset callback.
* `apply:` — once per external pose, at ingest (`correct_pose`,
  `apply_gravity_correction`).

Declaring `correction_for_pose` a `store` would have been the one-word fix and
would have retired the per-message vs per-pose distinction the manifest exists to
hold: a build hoisted to the offset callback compiles, passes every behavioural
test that exercises one pose, and applies the *home node's* residual to the whole
workspace. `test_every_apply_has_a_build_in_the_same_scope` closes the mirror of
that — an ingest that applies the correction without building it for its pose is
a working C-LEVEL-1 application that silently ignores the calibration map, and
the only declared exception is B1 below.

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
package, not "every caller of `planner`". `attic/ros-jugglebot-archived/`
(formerly `jugglebot/archived/`, moved 2026-07-31)
holds two dead copies of the old inline transform; neither has a `console_scripts`
entry and neither is imported by any live module, so they are excluded from both
the manifest and the grep counts. Deleting archived code is a separate concern.

## If you are adding a new pose surface

1. Decide **E or D**: did this pose enter from outside the node (a service
   request, a wire target, the neutral constant), or was it derived from
   measurement or from an existing plan? If you cannot answer that in one
   sentence, you have found a design problem, not a paperwork problem.
2. `E` ⇒ **build** the correction for that pose with
   `levelling.correction_for_pose(self._gravity_offset, self._active_tilt_map(), pose)`
   and **apply** it with `levelling.correct_pose(pose, correction)` — **once**,
   at ingest, before the pose reaches any planner entry. Build *and* apply:
   passing the stored `self._gravity_correction` instead is a working C-LEVEL-1
   application that silently ignores the calibration map. The map argument MUST
   be `self._active_tilt_map()`, never `self._tilt_map` directly: the accessor
   is the dormancy gate (§ "The map is gated on the level reference") that
   returns `None` until a level reference is loaded — passing the raw
   `self._tilt_map` re-creates the unlevelled residual-only composition at
   exactly your new surface, and neither the structural manifest (which keys on
   the call, not its arguments) nor the one-pose behavioural tests will catch
   it. Catch `TiltMapError` and fall back to the stored correction — never let
   a lookup kill a callback.
   `D` ⇒ apply nothing.
3. Add **both** rows to the manifest in `tests/ros/test_levelling_frame.py`
   (`build:` and `apply:`) with its classification, and add the row to the
   E-table above if it is external.
4. Never apply the correction to a linear velocity or a position. `twist` on
   `TimedTarget` and `target_vel` on `DynamicTargetCommand` are externally
   supplied and are deliberately **not** corrected: the correction is a bias on
   the commanded *rotation*, not a re-expression of the platform frame. Rotating
   a linear velocity would be a category error, it would be invisible in
   leg-space telemetry, and today it would land green because `build_catch`
   discards the twist and `timed_target` has no production client.
