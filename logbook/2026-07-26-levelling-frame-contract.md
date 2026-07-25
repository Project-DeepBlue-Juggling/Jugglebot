---
title: One levelling frame — C-LEVEL-1, and the swing that was never the frame's fault
type: bugfix
date: 2026-07-26
status: resolved
phase: "Self-toss anomaly fixes — levelling-frame-contract Phases 1-2"
related_plan: "levelling-frame-contract.md"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/levelling.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/mpc_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/docs/levelling_frame.md
  - tests/motion/test_levelling.py
  - tests/motion/test_levelling_probe.py
  - tests/ros/test_levelling_frame.py
  - tools/probes/levelling_tilt_bag_check.py
  - tools/probes/README.md
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_phase1_hold.md
  - tests/hardware/mvp_bench_runbook.md
  - plans/active/levelling-frame-contract.md
commits:
  - aea7b49
subsystem:
  - ros
  - motion
tags:
  - kinematics
  - testing
  - docs
  - safety
---

# One levelling frame — C-LEVEL-1, and the swing that was never the frame's fault

## Summary

`trajectory_node` applied the gravity-levelling correction on **two** of its six
external pose-ingest surfaces and on none of the other four, so *"level" meant two
different things inside one node*. Phases 1–2 extract the transform into one shared
pure-Python helper (`motion/levelling.py`), land the normative contract
**C-LEVEL-1** (`ros_ws/docs/levelling_frame.md`), migrate all six surfaces plus
`mpc_bridge_node`'s verbatim second copy through it, and freeze the enumeration
with a structural AST guard that fails on the class of omission that caused the
bug — including three shapes no behavioural test can reach.

The finalize pass then **falsified the plan's headline claim**. The plan (and the
contract document, and the operator runbook, and the verdict probe) all said this
fix removes the +2.32° pre-throw tilt swing the operator reported. It does not.
The swing is `planner.build_catch`'s specified *arrival twist*, it is
`(16/81)·rate·|tdir_x|·lead` = **0.789132° per second of catch lead** regardless of
how well the machine is levelled, and at the reference session's own 3.70 s lead
this fix moves it from **+3.0992° to +2.9198°** against gravity. What the fix
actually closes — and it is worth having — is the **park**: the platform rested
0.78° off gravity and the positioning and catch surfaces disagreed about where
level was.

## Problem

On the powered self-toss session of 2026-07-25 15:17:48 the operator reported that
on **every** toss the platform "slowly tilted back, then forward" between sending
the goal and the throw. Five attempts, reliably reproduced. Commanded pose,
FK-reconstructed from `/leg_setpoint_echo`, toss #4: park `rx = 0.0000°`, a
mid-plan excursion to `+2.3204°`, settling at `−1.0784°` about 0.7 s before
release. The tracker-measured catch error on that ball was **16 mm** against a
~35 mm cup radius.

## Root Cause

An **incomplete enumeration** of the surfaces a pose can enter `trajectory_node`
through — not a wrong number. `_on_platform_pose` (E1) and `_on_dynamic_target`
(E2) applied the correction; `go_to_pose` (E3), `timed_target` (E4), `go_home`'s
neutral (E5) and `timed_target(hold_after=False)`'s neutral *return* target (E6)
did not. The toss's positioning `go_to_pose` therefore parked the platform at
plan-frame `rx = 0` while the pre-tilt `catch/dynamic_target` asked for plan-frame
`rx = −0.7788°`, and `build_catch` dutifully planned a min-jerk reach **between
two frames**.

E6 was found only by Phase 0's from-the-entry-surfaces-down audit; a grep of the
existing call sites cannot find it, and no behavioural test could have caught it,
because that surface has never been exercised in production.

`mpc_bridge_node` carried a verbatim second copy of the transform. The composition
`R_gravity @ R_target` is **not commutative** — for a `[0.15, −0.08, 0]` target on
this session's offset the two orders differ by `1.268e-3 rad` (0.0727°), *entirely
in `rz`*, which is invisible in leg-space telemetry — so a re-derived copy is a
silent frame error, not a loud one.

## Discussion

### The hypothesis this phase withdrew

The plan's Context said the mid-plan swing was the reach between two frames and
therefore this plan's to remove. The implementing session had already corrected
half of that claim (the `−1.0784°` settle is `build_catch`'s tilt-through-seat
overshoot, not the frame). The finalize pass, prompted by a reviewer who computed
the post-fix excursion at the *production* catch lead rather than at the 1.2 s
lead the unit fixture uses, found the other half is wrong too.

Reproduced through the production planner, seeding pre-fix at the uncorrected
neutral and post-fix at the corrected one, both targeting the corrected catch
pose:

| lead | PRE peak (plan frame) | PRE above park | POST peak (plan frame) | POST above park |
|---|---|---|---|---|
| 1.2 s | +0.3840° | +0.3840° | +0.1682° | +0.9470° |
| 2.0 s | +0.9906° | +0.9906° | +0.7995° | +1.5783° |
| 3.70 s | +2.3183° | +2.3183° | +2.1410° | +2.9198° |
| 3.90 s | +2.4753° | +2.4753° | +2.2988° | +3.0776° |

The reference bag's `+2.3204°` pins the lead at **3.70 s**. Two things follow, and
both were wrong in four artefacts:

1. **The swing survives.** `build_catch` specifies a non-zero arrival twist
   (`rate · tdir`, `_CATCH_TILT_THROUGH_RATE_RADPS = 0.07`), so a reach whose start
   and end tilts are *identical* must still swing out and come back. With
   `p0 == p1, v0 = a0 = a1 = 0` the quintic collapses **exactly** to `v1·T·φ(s)`,
   `φ = −4s³ + 7s⁴ − 3s⁵`, `|φ|` maximal `16/81 = 0.1975309` at `s = 2/3`. So
   `peak_above_park = (16/81)·rate·|tdir_x|·lead` — linear in the lead, and
   **independent of the correction's magnitude**. Verified against
   `planner.build_catch` at six leads to 4 dp.
2. **The number the runbook gated on goes UP, not down.** `pre(t) = post(t) +
   0.7788°·(1 − h(s))`, so the absolute plan-frame peak always falls (by at most
   the correction) while the peak measured *from the park* always rises, by
   `0.7788°·h(s_peak)`. Pre-fix the park itself sat 0.7788° high and hid part of
   the swing.

Physically: pre-fix the platform rested 0.7788° off gravity and peaked at
`+3.0992°`; post-fix it rests at `0°` and peaks at `+2.9198°`. **0.18° of a 2.9°
swing.** The operator will still see the tilt.

### Why this mattered more than a documentation slip

The runbook's CHECK LVL-3 said *"a value at or above the pre-fix +3.099° is an
ABORT"* for `peak_above_park`. A healthy post-fix system reads `+2.92` at the
reference lead — 0.18° from a hard ABORT — and **crosses it at any lead ≥ 3.93 s**.
Meanwhile the runbook's stated expectation for the same field was `+0.17°` to
`+0.80°` (the *absolute* peaks at 1.2 / 2.0 s leads, three conventions mixed into
one bullet), and the probe printed a third number (`+2.3204`) under the same label.
The instrument would have scored a working fix as a failure and burned a powered
sitting.

That is the failure mode the run brief calls a **one-sided instrument**: this probe
had only ever been validated against the pre-fix bag, the only capture that
existed. Fixing the thresholds without fixing that would have left the next
instrument in the same state, so the probe grew a `--self-check` that scores three
sessions synthesised through the production planner — post-fix must PASS, pre-fix
must FAIL, and a post-fix session contaminated by a long pre-`go_home` ACTIVATE
hold must FAIL *with an ambiguity note* — guarded in CI by
`tests/motion/test_levelling_probe.py`.

That third case is its own defect, found by a reviewer and confirmed here. The
probe scores the **longest** rx plateau as the park. Between ACTIVATE and the first
`go_home` the node holds at the FK of the *uncorrected* activate pose, commanded
`rx ≈ 0` — which is **correct** under C-LEVEL-1's second half (a seed is never
corrected) and is pinned by `test_D1_fk_seed_is_not_corrected`. If the operator
dawdles there while reading the runbook and fetching a ball, that hold wins the
vote and a perfectly healthy session reads `park_rx ≈ 0`: byte-identical to the
pre-fix frame, which the runbook maps to a hard ABORT and a hunt for a
non-existent build/QoS fault. `--t0` is now mandatory in LVL-3 and the probe says
so when it detects the shape.

The retired ABORT is replaced by something that is *not* a threshold on the catch
lead: the probe inverts the linear model and prints the **implied lead**, which on
a healthy post-fix session recovers the toss's actual lead (the pre-fix bag reads
2.94 s against a true 3.70 s, because a frames-disagree reach carries the extra
`−0.7788·h(s)` term).

### Where the contract lives, and why not in `controller/`

`ros_ws/docs/levelling_frame.md`, not `controller/REFERENCE_LAYER_CONTRACT.md`.
Three failure modes drove it: (a) no `controller/` code applies this correction at
all — the only hit outside the ROS package is the `levelling_complete` *boolean*
in `teensy_link/rpc_args.py` — so a normative invariant with zero enforcement in
its own subtree invites a future reader to "apply" it to `controller/target.py`,
i.e. to introduce the double-application mirror bug in a second subsystem; (b) a
`ros_ws` developer adding a seventh ingest has no reason to open a `controller/`
document, so the contract would be absent at the exact moment it decides E-vs-D;
(c) it would make "the reference-layer contract" mean two things, which is the same
one-word-two-meanings failure this plan exists to close.

### Enumerated sites, not one choke point

C-LEVEL-1 deliberately distributes the invariant across an *enumerated* set rather
than a single choke point, which reads like a violation of the repo's
contract-over-patches rule until you count. A choke point would have to sit at the
planner entries or the emitter. **Derived poses reach the same planner entries as
external ones and they outnumber them** — nine derived seeds and four direct plan
installs against six external ingests — and a choke point there cannot tell them
apart, so it would double-apply on every FK-seeded hold: once plan-space "level"
*is* `−0.7788°`, an FK seed already reads `−0.7788°` on its own, and correcting it
again commands `−1.5576°`. The discrimination is only available at ingest, where
provenance is still known. The *transform* is applied in exactly one place; the
*enumeration* is what is distributed, and freezing it is the guard's whole job.

### Correcting the neutral at USE, not at construction

`_corrected_neutral_pose()` is a method, not a corrected constant computed in
`__init__`. A stored corrected constant double-applies the instant a new
`/gravity_offset` arrives mid-session — and the operator levels *after* launch, so
that is the normal case, not an edge case. It also strengthens the structural
guard: the planner-entry manifest records the argument's **source text**, so
`self._corrected_neutral_pose()` at the `build_return_to_neutral` call site is
itself the frame assertion, and reverting it to `self._neutral_pose` fires the
manifest test (mutation-verified).

### The in-flight rule, priced

"The live plan keeps its frame; the next install picks up the new correction." Not
adopted because the plan says so — because re-framing a live plan means stepping
the commanded pose mid-motion, and the counterfactual has a number: an emitter-side
correction would step `u0` by the full offset delta on the very next 25 ms knot —
**2.7736 mm = 0.03908 rev**, an instantaneous 110.9 mm/s / 4438 mm/s² leg
transient — and the pump's 0.3 rev step gate would **accept** it (13 %). A command
discontinuity on the wire, not merely an ungated plan. Ingest-side placement gives
the rule for free, because nothing re-reads the stored `R` for an installed plan.

### The guard's three blind spots, closed

The implementing session's guard closed the two classes the plan named (a new
planner entry; a new pose-bearing argument on an existing entry). Two reviewers
independently found three more, all mutation-proven, all closed here:

- **A new ingest that writes plan-feeding node state instead of calling anything
  tracked.** `_on_platform_pose` corrects E1 and stores it in
  `self._follower_target`; `_follower_tick`, a *different scope*, reads it and
  calls the planner. A second absolute-pose topic whose handler assigned that
  attribute without correcting would add no planner call and no `levelling` call —
  the identical "two meanings of level" defect under a green guard. The writer set
  is now frozen (`test_follower_target_writers_are_frozen`); mutation A2 goes
  34-passed → 1-failed.
- **`from jugglebot.motion.levelling import correct_pose`.** Both AST manifests key
  on the dotted source name and the behavioural counting fixture patches
  `correct_pose` on the *module object*, so a direct binding un-instruments the
  structural half and the counting half at once, silently. Both the omission bug
  and the mirror bug then land green. Forbidden with an empty allow-list
  (`test_planner_and_levelling_are_reached_by_attribute_access_only`);
  mutation A6 fires.
- **Endpoint-only regression assertion.** The plan twice required the tilt to be
  *sampled across the plan* ("the 2026-07-25 signature was a mid-plan excursion, so
  an endpoint-only assertion would have passed while the platform tilted"), and
  both new regression tests sampled exactly one point each. The across-plan half is
  now `test_the_reach_excursion_across_the_plan_is_the_arrival_twist_alone`, which
  pins the excursion as a **rate per second of lead** (`0.789132 °/s`, derived
  in-test from the planner's own constants), asserts the peak lands at `s = 2/3`,
  and asserts the direction of both effects against the pre-fix geometry at the
  identical lead — so the physics above is executable, not prose.

The reason the regression test asserts *net reach displacement* rather than a flat
`rx` trace is now doubly clear: a flat-rx assertion is unreachable while
`build_catch` ramps a through-seat residual, so it would have had to be weakened
later — and weakening a regression test is how a closed investigation reopens.

### The defect that was surfaced and deliberately NOT fixed

`planner.build_catch` reads `catch_pose[3:5]` as "the receive tilt" and ramps a
residual rate along it. That premise holds only while the commanded frame **is**
the gravity frame. With a correction loaded, a gravity-level catch (identity
receive tilt on the wire) arrives as a non-zero plan-frame tilt — the correction
itself — so the through-seat engages and settles the platform
`0.5 × 0.07 × 0.15 = 0.005250 rad = ` **0.3008° off gravity-level at ball contact**,
in the correction's own direction. Closed form for this session's offset:
`−1.078408° rx / −0.095775° ry`; the bag recorded `−1.0784 / −0.0958`; the
committed probe measures the settle plateau at `−1.0775°`, **0.0009°** from the
closed form. 0.3008° ⇒ 16.5 mm of lateral drift at 3.93 m/s over 0.8 s, against the
16 mm the tracker measured.

Not fixed, and that is a decision rather than an omission: the fix changes commanded
motion at ball contact on **every** catch including the shipping reload path, and
the right fix is a genuine fork — pass the gravity-referenced receive tilt
separately, pre-subtract the overshoot, suppress the through-seat when the
gravity-referenced tilt is ~0, or accept it — each with a different ball-seating
consequence. Pinned as a characterisation test so it cannot drift, and escalated
with numbers.

### One reviewer finding refuted, and why it is written down

A reviewer argued that Tier 8b's swing-compensated pre-tilt position becomes
*exact* after Phase 2 (and was 1.3 mm wrong before), on the grounds that
`cup_lateral_shift_mm` is world-referenced. Traced and refuted:
`stow_to_global_mm` is a **pure z translation** (`toss_release.py:83-84`) — there
is no rotation between STOW, "global" and the base frame — so the shift and the
centroid it corrects live in the *same* fixed frame, and the shift must be computed
from the tilt the platform physically holds in that frame, i.e. the
**post-correction** commanded tilt. `_cup_axis_xy`'s docstring word "world" means
the fixed, non-rotating frame as against the platform *body* frame; it does not
mean gravity-aligned. So the ~1.3 mm error is genuinely *introduced* by Phase 2 —
and is worth it, because the same change makes the release *direction*
gravity-correct, which is ~0.78° ≈ 42 mm over a 0.8 s flight. The plan note now
states the frame assumption explicitly, so nobody "restores consistency" by
subtracting the correction and silently re-introduces the 42 mm.

## Fix

**Phase 1 — the shared implementation and the contract.**
`ros_ws/src/jugglebot/jugglebot/motion/levelling.py`: `correction_from_offset` owns
the `[-tilt_x, -tilt_y, 0]` sign convention; `apply_gravity_correction` /
`correct_pose` own the `R_gravity @ R_target` composition; `identity_correction`
returns a *fresh* `np.eye(3)` (a shared module constant is one in-place write from
corrupting every node's frame in the process). Pure Python, no ROS imports.
`ros_ws/docs/levelling_frame.md` states C-LEVEL-1 normatively.

The extraction is **bit-identical**, verified numerically against the verbatim
pre-change inline code from `git HEAD` — 6 offsets × 6 rotvecs including rotvec ≈ π
and 1e-9 offsets, `max |R_old − R_new| = max |rv_old − rv_new| = 0.000e+00` — not
by reading the diff. Two reviewers reproduced it independently.

**Phase 2 — the migration and the guard.** All six external surfaces plus
`mpc_bridge_node`'s B1 route through the helper; the second copy is deleted. E3/E4
are corrected inside `_pose_from_msg` (the observed failure mode is *omission*, so
correcting in the message converter makes a future service corrected by
construction); E5/E6 go through `_corrected_neutral_pose()`. Per-row grep gate:
applications **0**, method definitions **0**, sign-convention constructions outside
`motion/levelling.py` **0**, stored `R` **2 assignment sites**.

`tests/ros/test_levelling_frame.py` freezes two AST manifests (planner entries keyed
on pose-bearing arguments, *plural*; `levelling` call sites) with **discovered**
file sets, multiset-compared, plus a hard failure on any unknown `planner.build_*`,
the attribute-access-only import guard, and the `_follower_target` writer set. The
behavioural half drives every surface with a non-identity correction and counts
applications **per row** — because a final-value check cannot distinguish
"corrected once" from "corrected, re-corrected and accidentally un-corrected".

**The instrument.** `tools/probes/levelling_tilt_bag_check.py` gets: the scoring
core factored out of the bag reader (`score_series`), a lazy `mcap_ros2` import so
it is testable without a bag, `--self-check`, sign-aware settle selection (a
hard-coded `min()` returns the *park* for a positive-rx correction), the
ACTIVATE-hold ambiguity note, the `implied lead` cross-check, and one consistent
convention for `peak_above_park` in the docstring, the constants and the printed
line.

**The narrative sweep.** The 16 mm attribution removed from the contract
document's "Why this exists" and replaced with an explicit "what C-LEVEL-1 does NOT
close" section; the plan's Context corrected a second time; Phase 4's
`peak_above_park` ABORT retired; the Phase-2 grep gate reworded from a reference
count (unachievable at 2, and "fixable" only by re-introducing the duplicate this
phase deletes) to an assignment count; the phase summary table's stale `TODO` rows
and stale Phase-4 gate cell updated; operator pre-brief item 3 added, in as many
words: **the swing is still there and that is correct**.

## Verification

**Full suite** (`pytest tests/ -q`, run 2026-07-26 on this Jetson, foreground, sole
occupancy): **3484 passed, 3 xfailed in 1371.51s (0:22:51)**.

Baseline at `9f35a66`: `pytest tests/ -q`, run 2026-07-26, **3429 passed, 3 xfailed
in 1323.44s**. The delta is **+55 passed, 0 change in xfailed** — exactly the cases
this phase adds: 37 in `tests/ros/test_levelling_frame.py`, 12 in
`tests/motion/test_levelling.py`, 6 in `tests/motion/test_levelling_probe.py`. No
test was weakened, skipped, xfailed or deleted; `git status --porcelain` shows zero
modified pre-existing `tests/**/*.py`.

**Mutation testing of the guard** — six seeded regressions, each reverted, tree
restored and re-verified green:

| seeded regression | caught by |
|---|---|
| drop the correction from `_pose_from_msg` (E3/E4 omission) | 8 failures incl. the structural call-site manifest |
| revert `go_home` to the uncorrected neutral (E5) | 4 failures |
| add `hold_after=False, neutral_pose=…` to `_plan_and_install_catch` | `test_planner_entry_manifest_is_exhaustive` — the case no behavioural test can reach |
| correct the FK seed (the mirror bug) | the call-site manifest + `test_derived_surface_applies_the_correction_exactly_zero_times[seed]` |
| a new ingest writing `self._follower_target` with no correction | `test_follower_target_writers_are_frozen` (34 passed → 1 failed) |
| `from jugglebot.motion.levelling import correct_pose as _cp` | `test_planner_and_levelling_are_reached_by_attribute_access_only` |

**The instrument, both sides.** FLAG: the real pre-fix bag
`2026-07-25_15-17-48 --t0 165 --t1 200` reads `park −0.0001°` (err `+0.7787°`),
`settle −1.0775°` against the closed form `−1.078408°`, `VERDICT: FAIL`, exit 1 —
correctly, because that bag predates the migration. ACCEPT:
`levelling_tilt_bag_check.py --self-check` → `SELF-CHECK: PASS`, exit 0, with the
post-fix synthetic scoring `park −0.7788°` / PASS / implied lead **3.70 s** against
a synthesised 3.70 s.

**Physics reproduction.** The `peak_above_park` table above was computed through
`planner.build_catch` with `TrajectoryLimits.from_config(hw)` and
`StewartGeometry()`; the `16/81` extremum was derived independently from the
quintic Hermite boundary conditions and agrees with the sampled planner output to
4 dp.

**No hardware validation yet.** `tests/hardware/session_anomaly_fixes.md` § Section
LVL carries LVL-0 … LVL-5 with numeric PASS/ABORT criteria.

## Outcome

C-LEVEL-1 is landed with all three parts of the repo's contract pattern: the
normative document, one shared implementation, and a test that fails on the
omission — plus, because the omission can arrive through shapes no behavioural
test reaches, a structural guard whose own coverage is mutation-proven.

The plan's Phase-4 predictions did not survive contact with the planner. Two were
already retired by the implementing session (`rx` flat to ±0.05°; catch error
< 10 mm); a third — the `peak_above_park` ABORT — is retired here, and the
operator pre-brief now leads with the fact that the visible tilt remains. The
honest scorecard for Phases 1–2 is: **the park is fixed, the frames agree, and
the swing is `build_catch`'s.**

**Deployment**: `cd ~/Desktop/Jugglebot/ros_ws && colcon build --packages-select
jugglebot && source install/setup.bash`, then **relaunch** `jugglebot_launch.py` —
the launch runs the *installed* copy. No firmware flash, no interface build, no
config regeneration. `/gravity_offset` is VOLATILE with one latched publish per
orchestrator boot, so the relaunch itself reverts the correction to identity while
`RobotState.levelling_complete` still reads True: **a manual `level` after the
relaunch is mandatory** until Phase 3 closes that structurally.

## Open Questions

1. **`build_catch`'s through-seat aim — operator decision, blocking Phase 4's catch
   number.** Four branches with different ball-seating consequences (above). The
   0.3008° residual is the 16 mm catch error; nothing else in this plan reaches it.
2. **The pre-throw swing — routed to
   `plans/active/catch-reach-degenerate-overshoot.md`.** That plan's "degenerate
   reach" case is precisely what a post-Phase-2 toss reach now *is* (zero net tilt
   displacement, non-zero arrival twist), so its reproduction survives this plan
   rather than disappearing with it. It should also know that the arrival twist is
   a *specified* boundary condition, not an artefact.
3. **Tier 8b's swing compensation is ~1.3 mm off in the base frame after Phase 2**
   (see Discussion). Below the 16 mm catch error, gates nothing today, and the
   trade it buys (a gravity-correct release direction, ~42 mm over a 0.8 s flight)
   is strongly favourable. If the through-seat fork is opened, this is the second
   site — but only with the frame assumption in front of you.
4. **`mpc_bridge_node` still has no test file of its own and no launch entry.** Its
   B1 ingest has one behavioural test inside `tests/ros/test_levelling_frame.py`
   (constructed via `__new__` so nothing binds ZMQ :5558). If that node is ever
   re-enabled, it wants real coverage.
5. **The guard is not a general "no new plan-feeding node state" guard.**
   `_follower_target` is frozen because it is the one field a pose-bearing external
   message writes for another scope to consume. A future ingest that invents a
   second such field, or installs a `TrajectoryPlan` directly, is outside its reach
   — documented in the contract's "what this guard deliberately does not cover".
