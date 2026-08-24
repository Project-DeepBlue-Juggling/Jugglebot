# The Catch Reach Envelope Contract — C-REACH-1

**Normative.** This document specifies what `trajectory_node`'s catch reach
envelope bounds, and — the load-bearing half — what it must **not** be used to
bound. It is the written third of the repo's contract pattern (normative
statement + one enforcement point + a test that fails without it); the other two
are `jugglebot/trajectory_node.py`'s `_svc_arm_catch` / `_on_dynamic_target`
pair, and the `test_creach1_*` block in `tests/ros/test_trajectory_node.py`.

Scope: every `catch/dynamic_target` installed while the catch-armed latch is
raised — i.e. the reload path, the toss path (both tiers), and any future
coordinator that arms the latch. Nothing else in the motion layer consults the
envelope.

Sibling contracts: `ros_ws/docs/catch_arrival_contract.md` (C-CATCH-1) and
`ros_ws/docs/levelling_frame.md` (C-LEVEL-1). C-LEVEL-1 matters here for one
reason stated in § 5: the envelope is a **position-only** test, and the levelling
correction rewrites rotation only, so the two cannot interact.

---

## 1. The normative statement

> **C-REACH-1.** The catch reach envelope bounds **unrequested** platform
> excursion during a catch — how far a drifting landing estimate may drag the
> platform away from where the catch was planned to happen. It does **not**
> bound requested displacement.
>
> Its centre is therefore the **nominated catch point**, declared by whoever arms
> the latch (`catch/reach_center`, published before `trajectory/arm_catch`). When
> nothing is declared, the centre is the commanded platform position at the
> arm-latch raise — the co-located default, which is the nominated catch point
> for any coordinator that catches where it already is.
>
> Requested displacement — a coordinator deliberately reaching from A to B — is
> gated by its own **pre-throw** contract (`jugglebot_operational.
> toss_max_displacement_mm`, the closed-form quintic reach bound, and the
> planner's feasibility gate), never by this envelope.

The two quantities are separate and must stay separate. Raising the envelope to
"buy reach" re-couples them and silently loosens the drift bound by the same
amount; lowering the displacement cap to "protect the envelope" refuses throws
that were never a drift risk.

## 2. Why this exists — the failure it closes

The envelope was introduced (2026-07-23) against two observed classes: a
frame-convention regression that put a catch target ~170 mm off, and the platform
chasing a corrupt tracker landing estimate hundreds of mm sideways through a
flight window. Centring it on the **held pose** was correct for both, because the
only catch path at the time — the reload — catches where the platform already is.

Tier 8b then introduced a catch that is deliberately **not** where the platform
is: the displaced toss throws from A and reaches to B during the flight. Under
the held-pose centre, that requested reach was measured against a drift bound,
and past 80 mm it was rejected `WORKSPACE`:

> Hardware, 2026-07-27, bag `2026-07-27_16-07-30`: four Tier-8b goals at
> `(100,100)`, `(90,90)`, `(80,80)×2` — nominal displacements 141/127/113/113 mm —
> **all four refused**, `catch target 146/131/117/117 mm from the armed hold pose
> exceeds the 80 mm reach envelope`. 4/4.

The refusal is not the problem; **when** it arrives is. The reach is published at
`t_release`, so every one of those rejections landed with the ball already
airborne and the platform holding at A. There is no recovery from that state: the
ball lands 113-141 mm from the cup. A gate that can only fire after the
irreversible commitment is not protecting anything — it is choosing the moment of
failure.

Raising the envelope to 250 mm (as an uncommitted bench override did for that
session) would have "fixed" it by letting a genuinely drifting estimate drag the
platform 250 mm instead of 80. That is the coupling this contract breaks.

## 3. The enforcement point

One place captures the centre, one place tests against it — both in
`trajectory_node`:

| | |
|---|---|
| Declaration | `_on_reach_center` stores a **pending** centre. Never applied on arrival — applying it mid-catch would move the envelope out from under an airborne ball. Non-finite values are dropped loudly (a NaN centre makes `excursion > envelope` False for every target, i.e. silently disables the gate). |
| Capture | `_svc_arm_catch`, at the RAISE edge, before the latch flips. Declared centre if one is pending, else `_current_state()[0][:3]`. **Every** `arm_catch` call — raise, lower, or idempotent no-op — consumes and clears the pending declaration first, so a declaration is scoped to exactly one raise and cannot arm the *next* goal's catch. A mode change clears it too (the same force-disarm doctrine as the latch). |
| Test | `_on_dynamic_target`: `‖target[:3] − centre‖ > envelope` ⇒ `WORKSPACE` reject, published on `trajectory/target_feedback`. `WORKSPACE` is deliberate — it is a position-unreachable-by-policy verdict, so the coordinator's feasibility blacklist counts it and a drifting estimate blacklists out instead of dragging the platform all flight. |

The envelope radius itself is `trajectory_catch.reach_envelope_mm`
(`JB_TRAJ_CATCH_REACH_ENVELOPE_MM`, 80 mm) and is unchanged by this contract.

## 4. What each path does

| Path | Declares a centre? | Envelope centre | Behaviour vs pre-contract |
|---|---|---|---|
| **Reload** | No | Commanded pose at the raise (its catch IS the held pose) | **Unchanged.** Byte-identical choreography. |
| **Toss, Tier 8a** | Yes — the goal's B | B, which equals the pre-positioned held pose | Unchanged in value; the envelope simply no longer depends on POSITIONING having actually arrived. |
| **Toss, Tier 8b** | Yes — the goal's B | B | The deferred A→B reach is now judged against B, so displacement is bounded by `toss_max_displacement_mm` and drift is still bounded at 80 mm — about B. |

The toss declares B, **not** the swing-compensated target the reach actually
carries. B is the operator-nominated quantity, the swing shift is a few mm, and
80 mm of envelope absorbs it with room to spare for the tracker refinements the
envelope exists to bound.

## 5. Residuals and the arguments for accepting them

1. **A lost declaration degrades to the old behaviour, loudly.** The envelope
   centres at A, the A→B reach is rejected `WORKSPACE` mid-flight, and both nodes
   log it. That is a missed ball, not a hazard — which is why the declaration
   needs no acknowledgement and no retry.
2. **The declared centre is trusted, not verified.** `trajectory_node` does not
   check that B is reachable, only that later targets are near it. It does not
   need to: the centre is not a command. Every actual motion still goes through
   `_plan_and_install_catch` → `planner.build_catch` → the feasibility gate, and
   the goal that produced B already passed the toss FSM's workspace, displacement
   and reach-bound gates before anything was armed.
3. **Frame.** The centre and the tested target are both position-only 3-vectors.
   C-LEVEL-1's gravity correction rewrites commanded **rotation** and never
   position (`motion/levelling.py::correct_pose`), so it cannot move either side
   of this comparison — the same argument that already held for the commanded-pose
   centre, preserved verbatim.
4. **The declaration is a plain `geometry_msgs/Point`,** not a
   `jugglebot_interfaces` type, so adding this channel needs no interface-package
   rebuild. A split build of `jugglebot_interfaces` against `jugglebot` is a known
   silent failure mode on this machine; not creating one was worth more than the
   header stamp a `PointStamped` would have carried.
5. **The scoping guarantee assumes the `arm_catch` call LANDS.** § 4's "consumed
   and cleared by every `arm_catch` call" is enforced inside `_svc_arm_catch`, so
   a declaration whose `arm_catch(True)` *and* whose teardown `arm_catch(False)`
   both fail to reach the handler (executor congestion, a service timeout) stays
   pending. Nothing else clears it but a control-mode change. A later arm raise —
   including the reload's — would then consume it and centre the envelope where
   nobody nominated. Not reproduced, and the same congestion that drops one
   client's calls generally drops the next one's, so it is documented rather than
   mechanised; if it ever needs closing, stamp the pending declaration with a
   monotonic time and drop it in `_svc_arm_catch` past a few FSM ticks (the
   declaration is always published exactly one 50 ms tick before its raise).
6. **The envelope gave up a second, accidental job: catching a silently-refused
   POSITIONING move.** Before this contract the envelope was centred on the pose
   held at arming, so a Tier-8a goal whose `go_to_pose` silently no-op'd (the
   disarmed-wire / guard-latch class) had its catch target rejected `WORKSPACE`
   and nothing moved. Now the centre is the declared B, so that target is
   accepted and the catch plan is installed — the platform traverses toward B
   during the flight instead of standing still. This is a deliberate trade, not
   an oversight: the envelope existed to bound *drift*, never to verify *arrival*,
   and leaning on it for arrival was the coupling this contract removes. Arrival
   verification belongs to the mocap cross-check (`_TOSS_POSITION_TOL_MM`, off by
   default — no platform body validated live), and for Tier 8b an un-arrived
   pre-tilt corrupts the throw *aim*, which the envelope never protected against
   anyway. The resulting motion is a normal planned catch reach, gated by
   `planner.build_catch` and the feasibility gate like any other.
7. **The throw site A is read as a CENTROID and consumed as a CUP xy.**
   `trajectory/commanded_position` publishes `_current_state()[0][:3]`, the
   commanded platform centroid (its 2026-08-23 sibling
   `trajectory/commanded_pose` carries the same three components plus the
   orientation, and is read only by the census-B1 positioning skip — the throw
   site still comes from the `Point`), while `compute_release_state_tilted`
   documents
   `throw_site_xy_mm` as the point the release *cup* sits at. The two are
   identical whenever the platform is level and differ by
   `hand_catch_offset_mm · sin(tilt)` when it is not — which is exactly the state
   the CAUGHT `STAY` terminal now leaves behind. Measured: a catch at
   `B = (−150, 0, 170)`, `T = 0.80 s` parks the cup at exactly `(−150.00, 0)` and
   the centroid at `(−153.10, 0)`. The aim stays self-consistent (A is nominated,
   and POSITIONING makes it true), so this is not an aim error — the planning box
   and the displacement cap are both applied to the centroid value.
   **RESOLVED FOR CHAINING 2026-08-14**: the box is now config-keyed
   (`toss_workspace_xy_mm`, default 160 = cap × 1.067 > the 2.07 % divergence),
   so the parked centroid sits inside the box and chained tosses at the cap are
   admitted; only a genuinely-requested `|B − A|` past the cap still refuses.
   The underlying frame question (A read as centroid, consumed as cup xy)
   remains open on `single-ball-toss` Phase E but no longer gates chaining.
   Pinned by
   `tests/ros/test_toss_sequencer.py::test_chaining_at_the_cap_box_dissolves_the_frame_divergence`.
8. **A is SAMPLED once at goal accept, with a 1.0 s staleness window on a 5 Hz
   topic, and nothing refuses a goal issued while a move is in flight.**
   `_live_commanded_position` inherits `_TRAJ_STATUS_STALE_S` from
   `trajectory/status` rather than inventing a second window, and
   `trajectory_node` samples `_current_state()` — the *live plan* state, not a
   settled pose. So a Toss issued mid-traverse nominates a pose the platform has
   already passed, and POSITIONING then supersedes the in-flight move with a
   profiled go_to_pose *back* to it: an unrequested backtrack whose magnitude
   scales with how fast the platform was moving when the sample was taken
   (bounded by the session velocity limit × the staleness, so ~200 mm at the
   typical 0.2 s publish age). The goal still succeeds and the aim is still
   self-consistent — A is nominated, and POSITIONING makes it true — which is
   exactly why this is easy to miss in a log. Not closed here because the fix is
   a new precondition (`trajectory/status` already carries `plan_kind` and
   `plan_time_remaining_s`, one field away from the node's cache) and that wants
   its own evidence; the operator rule is in the runbook's § SECTION DISP ladder
   preamble: **do not issue a Toss while a move is executing.**

## 6. The seam this opens elsewhere, and where it is closed

Decoupling requested reach from the drift envelope is what makes a displaced
toss possible, and a displaced toss is what makes **staying at the catch pose**
useful (`jugglebot_operational.toss_stay_at_pose_on_caught`). A platform parked
off centre then breaks a *different* path: the **reload** catch is hard-fixed at
the workspace centre and the reload never pre-positions, so it would arm an
envelope centred off `(0, 0)` and reject the incoming BB ball `WORKSPACE`
mid-flight — the same class of failure this contract exists to eliminate, one
path over.

That is closed in `reload_sequencer._step_checking`, not here: a reload commanded
with the platform farther than one envelope radius from the reload catch point is
refused `REJECTED_NOT_CENTERED` **before** BB is asked to throw. Refusal rather
than an auto-return, because an auto-return injects new commanded platform motion
into the shipping reload choreography and no hardware session has ever run it.

## 7. How to change this contract

Change the document first, then the enforcement point, then the tests — never the
other way round. In particular:

- **Do not raise `reach_envelope_mm` to permit a larger requested reach.** That is
  the coupling C-REACH-1 removed. Raise `toss_max_displacement_mm` instead, and
  bring the evidence the YAML comment asks for.
- **Do not let a coordinator declare a centre it did not compute from its own
  goal.** A centre derived from live tracking would let a corrupt estimate define
  its own envelope, which is precisely the drift failure the envelope bounds.
- **Do not apply a declaration outside the arm raise.** Mid-catch re-centring is
  the one way this channel could move the platform in a way nobody requested.
