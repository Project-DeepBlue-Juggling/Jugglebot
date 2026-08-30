---
title: "The displacement caps were policy, not physics — both knobs deleted, and the gap the deletion opened is closed by a real reach gate"
type: investigation
date: 2026-08-29
status: resolved
phase: "toss-pipelined-preamble — 2026-08-29 second wave, 1 of 2"
related_plan: single-ball-toss.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/catch_reach.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - ros_ws/src/jugglebot/jugglebot/toss_session.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/outcome_detail.py
  - ros_ws/docs/catch_reach_envelope.md
  - ros_ws/docs/levelling_frame.md
  - sim/toss_gate.py
  - tests/hardware/ilc_fit_lib.py
  - tests/hardware/toss_trace_recorder.py
  - tests/hardware/session_cadence_ladder.md
  - tests/hardware/session_anomaly_fixes.md
  - tools/probes/displaced_reach_frontier.py
  - tools/probes/cadence_rung_check.py
  - tools/probes/README.md
  - plans/active/single-ball-toss.md
  - plans/active/toss-multi-catch-pose.md
  - plans/active/INDEX.md
  - tests/motion/test_catch_reach.py
  - tests/motion/test_ilc_fit.py
  - tests/ros/test_toss_sequencer.py
  - tests/ros/test_toss_coordinator.py
  - tests/ros/test_toss_session.py
  - tests/ros/test_toss_continuous_node.py
  - tests/ros/test_outcome_detail.py
  - tests/ros/test_levelling_frame.py
  - tests/ros/test_trajectory_node.py
  - tests/sim/test_toss_gate.py
subsystem:
  - ros
  - motion
  - config
  - sim
  - tools
tags:
  - toss
  - tier8b
  - kinematics
  - safety
  - testing
  - docs
---

# The displacement caps were policy, not physics — both knobs deleted, and the gap the deletion opened is closed by a real reach gate

## Summary

Owner decision, stated twice and unambiguously: **`toss_max_displacement_mm` and
`toss_workspace_xy_mm` are unnecessary policy. Delete them entirely; keep the
physics.** Both YAML keys are gone, with them the six regenerated artifacts, both
`TossSessionSequencer` ctor fields, the `min(cap, reach_bound)` conjunct, three of
the four `REJECTED_WORKSPACE` mint sites, and — as a separate argument with a
separate justification — the whole `REJECTED_CHAIN_UNREACHABLE` gate and code.

The deletion was justified by a claim about the machine: *a too-far goal is refused
pre-throw anyway, by the positioning move's own `go_to_pose` feasibility gate
(`REJECTED_POSITION(UNREACHABLE)`)*. **The wave audit refuted that claim by
measurement, for the tier that actually ships.** Tier 8b pre-positions to the
swing-compensated **pre-tilt pose at the throw site A** and defers the A→B reach to
`t_release`, so nothing pre-throw ever judges **B** laterally. Measured on this
tree: `B = (250, 0)` at `T = 0.80 s` and `B = (500, 0)` at `T = 1.00 s` are both
ADMITTED at CHECKING and then refused `WORKSPACE` **by the planner, mid-flight,
with the ball already in the air** — the C-REACH-1 class, re-opened by the very
change that removed its guard.

The fix is not a restored box. It is a production physics gate: a new shared pure
helper `motion/trajectory/catch_reach.py` plans the **deferred reach itself**, at
cycle build, with the goal still refusable — the same `build_catch` question
`trajectory_node` would have answered at `t_release`, asked ~0.8 s earlier. Median
cost **18.9 ms**, worst measured, on one tick of `_build_toss_cycle`.

*Wave 2 of 2026-08-29, sibling of [[2026-08-29-reload-stay-resume]]. It supersedes
this morning's "kept, not removed" verdict in
[[2026-08-29-rejection-message-enrichment]] — see Withdrawn claims. Unflown.*

## Symptoms

There was no hardware symptom. There was an owner instruction and a working tree
that had already made the argument for it.

The operator had **hand-ramped both keys to 600** in `config/hardware_config.yaml`
to get past a refusal at the machine — and that edit **reddened the suite**. The
relational invariant `toss_workspace_xy_mm >= toss_max_displacement_mm * 1.03`,
pinned by `test_local_constants_match_generated_config` and documented only that
morning in [[2026-08-29-rejection-message-enrichment]] § Open questions, does not
hold at `600 >= 618`. So the shipped ergonomics of "ramp the difficulty" were: edit
two YAML keys in a fixed ratio, re-run `generate_config.py`, rebuild, relaunch —
and get it wrong in a way that fails a unit test rather than the goal.

That is the concrete proof the knobs had become liability rather than protection.
A limit whose correct operation requires the operator to satisfy an algebraic
relation between two numbers, offline, is not a safety limit; it is a trap with a
test attached.

## Diagnosis

### D1 — what each key actually was

| key | provenance | what it bounded |
| --- | --- | --- |
| `toss_max_displacement_mm` (150) | **chosen** — the operator's own ordered working range, set 2026-07-28 (`plans/active/single-ball-toss.md` Phase E, sub-change 3) | the displacement `\|B − A\|`, as `min(cap, reach_displacement_limit_mm(T))` |
| `toss_workspace_xy_mm` (160) | **chosen** — a ±xy planning box added 2026-08-14, sized at `cap × 1.03` so the centroid-vs-cup chain divergence could not re-bind at the cap edge | `\|B.x\|`, `\|B.y\|`, and `\|A\|` |

Neither is derived from anything. The **reach bound** beside them is: it is the
closed-form quintic peak-jerk/velocity/acceleration limit, it follows the **live**
`trajectory/set_limits` session limits, and it is real physics. That asymmetry is
the whole of the owner's argument — one lever that tracks the machine, one that
tracks a YAML file nobody re-derives.

### D2 — the deletion's premise, and why it looked sound

The removal argument was: **nothing is lost, because a far-lateral goal is refused
pre-throw regardless.** The positioning move commands the catch pose, `go_to_pose`
runs the platform's real feasibility check on it, and the FSM mints
`REJECTED_POSITION(UNREACHABLE)` before anything is armed. The box was therefore a
*second, cruder* copy of an authority that already existed, standing in front of it
and answering first with a worse message.

That is true — **for Tier 8a**, whose positioning move commands B itself.

### D3 — the measurement that refuted it (BLOCKING, found by the wave audit)

The shipped tier is **8b**, and 8b does not command B before the throw.
`reload_coordinator_node._toss_positioning_xyz` pre-positions to the
**swing-compensated pre-tilt pose at the throw site A** for any tilted release —
that is the whole point of [[2026-08-27-aimed-reach-pretilt]] — and the A→B
translation is *deferred* to `catch_coordinator` at `t_release`.

So under 8b, the pre-throw gate ladder's surviving bounds on the catch pose are:

* the **z band** — a scalar on `B.z` (`TOSS_Z_BAND_MM` = 50 about
  `TOSS_ACTIVE_Z_MM` = 170), and
* the **closed-form reach bound** — a scalar on the **displacement** `|B − A|`.

**Neither bounds `|B|` itself.** With the box deleted, nothing did.

Measured on the tree, 2026-08-29:

| goal | closed-form bound at that T | CHECKING verdict | planner verdict at `t_release` |
| --- | --- | --- | --- |
| `B = (250, 0)`, `T = 0.80 s` | 256 mm | **ADMITTED** | `WORKSPACE` — refused mid-flight |
| `B = (500, 0)`, `T = 1.00 s` | 500 mm | **ADMITTED** | `WORKSPACE` — refused mid-flight |

The admitted-but-infeasible window is `|B| ∈ (~245, 256] mm` at `T = 0.80` and
widens to `(~245, 500] mm` at `T = 1.00` — because the displacement bound grows
with flight time while the platform's lateral reach does not. That is the
**C-REACH-1 mid-flight refusal class** (`ros_ws/docs/catch_reach_envelope.md`)
exactly: a refusal arriving *after* the ball is committed, which is not a refusal
at all — it is a miss with a log line.

### D4 — `REJECTED_CHAIN_UNREACHABLE` was a contract with no premise left

Separate finding, separate argument. The accept-time chain gate did one hop of
`_predicted_chain_site_mm` and refused a multi-cycle session whose *cycle-2* throw
site would fall outside the box. Its premise was **the box**: it existed to
pre-empt a cycle-2 `REJECTED_WORKSPACE(|A|)` at accept time rather than mid-session.

Delete the box and the gate has nothing to pre-empt. The obvious salvage — re-key
it on the surviving **reach bound** — was checked and does not work either:

> the chained residual (`|predicted A − B|`, the centroid-vs-cup frame divergence)
> is **< 3.2 mm** across the shipped range, against reach bounds of **≥ 83 mm** at
> the shortest admissible flight.

A gate whose refusal condition is `3.2 > 83` is **permanently green by
construction**. That is a dead contract: a normative document, an enforcement
point and a test, all three of which can never fire. It was deleted rather than
kept as reassurance.

`_predicted_chain_site_mm` **itself survives**, because it acquired a second
consumer yesterday: the staged cycle's **throw-site nomination**
(`_build_toss_cycle`), the fix of [[2026-08-28-displaced-chain-stale-site]]. The
predictor is load-bearing; the gate keyed on it was not.

## Discussion

### Why the refutation changed the shape of the fix, not the decision

The audit finding is BLOCKING and it would have been easy to read it as *"the
deletion was wrong, put the box back."* That reading was rejected, and the reason
is worth writing down because it is the same reasoning that made the box a
liability in the first place.

The box did not know anything. It was a number chosen to be a bit larger than
another chosen number. It happened to sit in front of the real gap — but it sat in
front of it **by accident of magnitude**, not by modelling it: `160` is not the
platform's lateral reach at `z = 170` under a 12° receive tilt at any flight time.
Restoring it would restore a coincidence and leave the actual question — *can the
platform get to B in the time the ball is in the air?* — still unasked pre-throw.

So the finding re-scoped the fix from **"delete a knob"** to **"delete a knob and
ask the question it was accidentally standing in for."** That is the climb-one-level
move: the class is not "goals beyond 160 mm", it is **"goals whose deferred reach
is infeasible"**, and only the planner can enumerate that class.

### Why the gate plans the real chain instead of modelling it

`catch_reach.catch_reach_verdict` is production-faithful, not a surrogate: the same
`compute_release_state_tilted` seed, the same receive tilt derived from the arrival
velocity, the same swing-compensated catch centroid that
`catch_coordinator.predicted_catch_command` produces, and the same
`planner.build_catch` — which gates through `feasibility.validate` — with the flight
as the lead.

A closed-form lateral envelope was the cheaper alternative and it was rejected on
the evidence of the thing it would have replaced. **The closed-form displacement
bound is exactly what put us here**: it is measured-conservative below `T ≈ 0.75 s`
and measured-**optimistic** above it, and the optimistic half is the half that
admits an airborne miss. Writing a *second* closed form for the lateral question
would buy a second asymmetry to discover later. Asking `build_catch` costs 18.9 ms
and cannot be wrong about its own answer, because it *is* the answer
`trajectory_node` will give.

### One body, two callers — the probe now delegates

`tools/probes/displaced_reach_frontier.py` mapped this frontier first, and its
`reach_verdict` was the original of this code. The probe now **delegates** to
`catch_reach` rather than carrying its own copy.

This is not tidiness. A probe that measures a frontier the machine does not gate
on is worse than no probe: it produces authoritative-looking numbers for a
boundary nothing enforces, and the next session plans a hardware ladder around
them. Sharing a body means probe and production **cannot disagree** — the
convention that they *should* agree is exactly the kind of thing that survives
three refactors and then quietly stops being true.

### Why the whole planner chain ships, and the ~150 ms fallback was not needed

The audit's contingency was to ship only the **IK + leg-stroke half** of the check
if the full pass proved too expensive for a blocking-loop tick, on a ~150 ms
budget. Measured (2026-08-29, this Jetson, a throwaway `/tmp/probe_reach_cost.py`
against `catch_reach_feasible` at default session limits, median of 15 after one
warm call), the numbers are in the module docstring:

| case | verdict | cost |
| --- | --- | --- |
| `A=(70,70) B=(-70,-70) T=0.95` | `OK` | **18.9 ms** (worst — a refusal-free call pays the whole `build_catch` + `validate`) |
| `A=(0,0) B=(100,0) T=0.80` | `OK` | 17.6 ms |
| `A=(0,0) B=(0,0) T=0.60` | `OK` | 13.1 ms |
| `A=(0,0) B=(250,0) T=0.80` | `WORKSPACE` | 1.5 ms (the IK/stroke half short-circuits) |
| `A=(0,0) B=(500,0) T=1.00` | `WORKSPACE` | 1.5 ms |

18.9 ms is an **eighth** of the budget, so the fallback was dropped: it would have
cost the `LIMIT_VEL` / `LIMIT_ACC` / `LIMIT_JERK` verdicts for nothing. One
implementation detail is load-bearing and is pinned in the docstring — the default
geometry is a **module singleton**, because `feasibility`'s `_WLIMITS_CACHE` is
keyed on `id(geom)` and a fresh `StewartGeometry()` per call re-runs its SVD and
roughly doubles every number above.

The cost lands on **one tick** of `_build_toss_cycle`, once per cycle, inside a
blocking loop. That is honest to name and honest to bound: the measured 0.16–0.54 s
body overruns in that loop come from the **synchronous `go_to_pose` round trip**
sitting in the same tick, and that whole class is owned by *Unblocking the loop
from the positioning service round trip* — M2's named sibling change in
`plans/archived/toss-multi-catch-pose.md` § Explicit non-goals. This gate does not
move that needle and does not try to.

### The gate is LAST of the static CHECKING gates, and the suite caught the wrong order

Placement was a real decision and the first attempt got it backwards.

Every static gate above it **names a knob the operator can move** — a flight time,
a displacement, an aim ceiling, a wire band, a z band — and answers a *scalar*
question. The reach verdict answers **"the leg kinematics say no"**, which is true
of a goal for many reasons at once and points at no single knob.

Put it earlier and it swallows the specific refusals: a `z = 300 mm` goal is
infeasible for the reach plan *too*, so an earlier reach gate reports the
kinematics verdict and sends the operator hunting through leg strokes for a number
they could have typed — instead of `REJECTED_WORKSPACE(B.z … TOSS_Z_BAND_MM)`,
which tells them exactly what to change. **The suite caught this**: the misplaced
gate swallowed the z-band refusal, and the ordering is now pinned by
`test_every_knob_gate_precedes_the_reach_verdict`, parametrised over each knob gate
with the goal that should trip it. Cheapest-and-most-specific first — the same
ordering doctrine the 8b block's own gates already follow.

### What the owner accepted, restated honestly

The deletion's residual is **not** an airborne miss — the reach gate covers that.
What is accepted is:

1. **Ergonomics.** Difficulty is now ramped by hand through `T` and
   `trajectory/set_limits`, live, rather than through a YAML cap plus codegen plus
   a relaunch. That is the point of the change.
2. **The conservative half.** Below `T ≈ 0.75 s` the closed-form displacement bound
   is measured-conservative, so some genuinely feasible throws are refused one gate
   early. That was true before the change too; it is now the *only* surviving
   residual, because —
3. **the optimistic half is now covered.** This is the bonus the morning's entry
   could not claim: above `T ≈ 0.75 s` the closed form admits displacements the
   platform cannot fly, and until today the cap was the crude margin against that.
   The build-time reach plan now refuses those **pre-throw, by name**. The class the
   cap was protecting is closed better than the cap closed it.

The **DISP ladder is now operator discipline**, not a machine-enforced ceiling. The
runbooks say so; `sim/toss_gate.py`'s outer advisory ring is a plain
characterisation radius (150 mm, kept as the value the hardware ladder was written
around) rather than a config read; and the standing fact that **hardware evidence
stops at 70 mm** is unchanged by any of this.

## Fix

**Config (7 files).** `toss_max_displacement_mm` and `toss_workspace_xy_mm` deleted
from `config/hardware_config.yaml` (a comment at the old site records that the cap
sat there until 2026-08-29), then `python config/generate_config.py` regenerated
**six** artifacts: `config/generated/hardware_config.{py,h}` and the four mirrored
copies — `ros_ws/src/jugglebot/jugglebot/hardware_config.py`,
`Teensy_code_canbridge/`, `Teensy_code_platform/` and `CatchingCone_code/`
`hardware_config.h`.

**`toss_sequencer.py` / `toss_session.py`.** Both ctor fields gone
(`chain_site_reachable`, and the box/cap resolution that fed
`TOSS_XY_LIMIT_MM`); the `min(cap, reach_bound)` conjunct collapses to the reach
bound alone; **three of the four `REJECTED_WORKSPACE` mint sites** removed
(`|B.x|`, `|B.y|`, `|A|`) — **the z band survives** as the fourth;
`_displacement_detail` is reach-bound-only. `REJECTED_CHAIN_UNREACHABLE` deleted,
code and gate.

**New: `ros_ws/src/jugglebot/jugglebot/motion/trajectory/catch_reach.py`.**
`catch_reach_verdict(throw_site_xy, catch_xy, flight_s, …)` returns
`(code, detail)`; `catch_reach_feasible(...)` is the production entry point
returning the code alone. Pure `motion/` — numpy plus the generated
`jugglebot.hardware_config` constants, no ROS2 and no repo-root imports, per the
boundary rule. `GRAVITY_MMS2 = 9806.0` (the ballistics-side gravity, never the
tracker's 9810 — same value and same reason as `toss_sequencer.GRAVITY_MMS2`).

**The call site.** `reload_coordinator_node._build_toss_cycle` (`:4465`, tagged
`# (2026-08-29 audit BLOCKING finding.)`) plans the deferred reach **immediately
after `release`** — it needs the release state's arrival velocity for the receive
tilt — and feeds the verdict to the FSM as `reach_verdict`.

**"Tier 8b only" is a NODE property, and the precise phrasing matters for whoever
edits this next.** The node guards with `if tier == TIER_8B and release is not
None:` and normalises `'OK'` to `''`; the FSM gate is a bare `if
self.reach_verdict:` at `toss_sequencer.py:2380`, a **sibling** of the
`if self.tier == TIER_8B:` block rather than a child of it. So the FSM keys on a
non-empty node-fed verdict and would fire on 8a if a node ever fed it one — which
is deliberate (it is a pose-feasibility verdict, not a tier feature) and is pinned
from the other side by `test_an_empty_reach_verdict_does_not_reject`: the node
feeds `''` for a feasible reach, for Tier 8a, and whenever there is no release
state to plan from, and none of those may refuse.

`toss_sequencer` CHECKING mints, last of the static gates:

```
REJECTED_POSITION(<planner code>: the nominated catch pose (x, y, z) mm is not
reachable — the deferred A->B reach would be refused at t_release with the ball
airborne)
```

**POSITION-class** because the thing refused is a platform pose, and it carries the
planner's own code as the leading subcode — the same `<code>: <message>` shape
`_step_positioning` mints, so a consumer reads *why* the same way whether the
refusal came from here or from `go_to_pose`. No new outcome code was minted.

**The three hard-break consumers**, each of which imported a now-deleted symbol or
read a now-deleted key:

* `sim/toss_gate.py` — the outer advisory ring read
  `hw.JB_OP_TOSS_MAX_DISPLACEMENT_MM`; it is now the literal `150.0`, documented as
  a characterisation radius and still **advisory** (the catch rate out there is
  dominated by a release-noise magnitude that is still the Phase-5 T0 placeholder,
  so gating on it would make the gate an artefact of an unmeasured number).
  `--advisory-rings` help text updated.
* `tools/probes/displaced_reach_frontier.py` — `reach_verdict`'s whole body is
  now a four-line **delegation** to `catch_reach.catch_reach_verdict`, and
  `GRAVITY_MMS2` is **re-exported** rather than re-declared so a `--json` run
  cannot quote one *g* while the gate it underpins uses another. Its two config
  reads became the module constants `OFF_CENTRE_RADIUS_MM` / `SURVEY_BOX_MM`
  (150.0) with new `--off-centre-radius` / `--box` flags. Both properties are
  pinned: `test_the_frontier_probe_delegates_to_this_module` (a monkeypatched spy
  must be reached, and the verdicts must match across four (A, B, T) points) and
  `test_the_probe_does_not_carry_its_own_gravity`.
* `tests/hardware/ilc_fit_lib.py` — `admit_command`'s mirror of the FSM's static
  gates drops its lateral half (`TOSS_XY_LIMIT_MM` import gone); the goal check is
  now **z-band-only**. It had already been flagged a stale mirror, and mirroring a
  gate that no longer exists would refuse rows the machine will happily fly.

**Docs, runbooks and plans re-pointed.** `ros_ws/docs/catch_reach_envelope.md` —
the C-REACH-1 normative statement's third paragraph drops
`toss_max_displacement_mm` from the pre-throw contract it names, and a dated
**2026-08-29** block records that *nothing about C-REACH-1 itself changes* (the
envelope still bounds *unrequested* drift about B; requested reach is still gated
elsewhere) while naming the new second pre-throw gate; § 7's residual 7 is re-headed
**"RESOLVED FOR CHAINING 2026-08-14, then DISSOLVED 2026-08-29"** and re-pins from
the deleted `test_chaining_at_the_cap_box_dissolves_the_frame_divergence` onto
`test_the_predicted_chain_residual_stays_a_few_mm`; § 8's "raise
`toss_max_displacement_mm` instead" instruction becomes the `set_limits` lever.

**The C-LEVEL-1 planner-entry manifest gains row `D9`** — in *both* of its homes,
which is the point of that contract: the prose in `ros_ws/docs/levelling_frame.md`
and the **executable** `_PLANNER_MANIFEST` tuple in
`tests/ros/test_levelling_frame.py`, which an AST pass compares as a multiset so
losing or gaining a `planner.*` call fires. `D9` is the first entry of a new shape
— *a planner entry that is a MEASUREMENT rather than a command*: the plan it builds
is read and **discarded**, never installed. Classified `D`, with the reasoning
written out: both pose arguments are **constructed** inside the same function
rather than ingested (the receive tilt is `tilt_to_receive()` of the derived
arrival velocity — the gravity-referenced quantity C-CATCH-1 requires
**un**corrected — and the catch pose's *position* half is one the correction never
touches by rule), the module is pure `motion/` and holds no offset to apply, and
the only real divergence from the commanded path is `trajectory_node` correcting
the catch pose's **rotation** at its own `E2` ingest: the levelling residual,
sub-degree, ~1 mm of cup lever against a hundreds-of-mm leg-stroke verdict. The row
closes the door explicitly — *if a future probe of this shape needs the correction,
pass it in; building one inside `motion/` would be the DOUBLE-apply, not the fix.*

`plans/active/single-ball-toss.md`
Phase E gets a **⚠ SUPERSEDED IN PART** banner at its head (the 2026-07-28 text is
left standing as the record of what was decided and why, marked as no longer a
description of the machine), and sub-change 3's cap paragraph is banner-marked
in place. `plans/archived/toss-multi-catch-pose.md` § M gate table is **re-keyed** —
the `|B.x|,|B.y|` and `|A_k|` box rows deleted, `|B − A|` re-pointed at
`reach_displacement_limit_mm(T)` as the sole bound, and a new row for *the walk can
PREDICT each `A_k` at all (`None` ⇒ refuse, fail-closed)* — plus an explicit
**re-mint note** carrying the whole argument forward so M cannot resurrect the
wrong thing:

> ⚠ **THE CODE HAS TO BE RE-MINTED, ON A NEW PREMISE (2026-08-29).** […]
> re-keying it on the reach bound would have produced a gate that can never fire
> (the chained residual is ~2 % of |B|, under 3.2 mm, against a bound of 83 mm or
> more that cycle 1 has already cleared). This plan's use is DIFFERENT and
> survives that reasoning: here the code refuses a hop whose park cannot be
> **predicted at all** […] which is a fail-closed condition rather than a
> threshold. Re-introduce it on that premise only; do not resurrect a box.

The § 2.6 caution that the closed form is optimistic above `T ≈ 0.75 s` is
re-pointed with a ⚠: it is now the **only** bound at every height, including the
milestone ones, so a ring leg that plans but does not fly is a live possibility at
long flights. `plans/active/INDEX.md`, `tools/probes/README.md`,
`tools/probes/cadence_rung_check.py` (its `REJECTED_CHAIN_UNREACHABLE` rung deleted
in place, with a comment saying what stood there),
`tests/hardware/session_cadence_ladder.md` and
`tests/hardware/session_anomaly_fixes.md` follow — the runbook gets an
*"⚠ AMENDED AGAIN 2026-08-29 — BOTH POLICY KNOBS ARE DELETED"* banner over
SECTION DISP, DISP-0.1's preflight becomes a `grep -c … # expect 0` for **both**
keys, and the RJ-table row for `REJECTED_CHAIN_UNREACHABLE` is marked
**RETIRED — this code no longer exists**. `outcome_detail.py` changes in **one
docstring example only** (its `outcome_subcode` illustration used the now-impossible
`REJECTED_WORKSPACE(|B.y| … 160.0 …)`; it now shows the surviving z-band form) —
no behaviour change, and the module this morning's entry introduced is otherwise
untouched.

**Tests.** **15 whole test functions deleted** — every one whose subject was a
deleted knob or the dead chain gate:
`test_chain_frontier_is_between_146_and_147_mm`,
`test_chain_unreachable_refuses_before_a_ball_flies`,
`test_chain_at_147_is_admitted_at_the_shipped_box`,
`test_chain_gate_does_not_fire_for_a_single_cycle_session`,
`test_chain_check_is_skipped_when_the_pose_is_unknown`,
`test_chain_check_is_skipped_on_tier_8a`,
`test_8a_has_no_displacement_cap_so_a_far_goal_reads_workspace`,
`test_displacement_within_cap_accepted`,
`test_chaining_at_the_cap_box_dissolves_the_frame_divergence`,
`test_workspace_box_is_ctor_config`,
`test_max_displacement_is_ctor_resolved_not_the_module_constant`,
`test_chain_unreachable_rejected_only_when_chaining`,
`test_chain_unreachable_quotes_the_predicted_centroid_and_the_box`,
`test_reject_order_num_throws_before_dwell_before_chain`,
`test_8b_asymmetry_map_radii_reach_the_shipped_cap`. No test **file** was deleted.

New: `tests/motion/test_catch_reach.py` — 7 test functions over the helper and the
probe's delegation. In `tests/motion/test_ilc_fit.py` the old inadmissibility test
was **split**, and the far-lateral leg's assertion **inverted** into
`test_a_far_lateral_goal_is_admitted_now_that_the_planning_box_is_gone`
(`assert not ok and 'WORKSPACE' in why` → `assert ok, why`), whose docstring names
the residual so nobody misreads the flip: *production would still refuse a 400 mm
goal — at POSITIONING on 8a, at the reach gate on 8b — so do not read this as "the
machine accepts 400 mm."* Elsewhere:
`test_a_far_lateral_goal_is_refused_pre_throw_on_both_tiers`,
`test_build_toss_cycle_feeds_the_deferred_reach_verdict`,
`test_live_traj_limits_feeds_the_reach_plan_and_fails_closed_to_the_default`,
`test_reach_verdict_rejects_position_with_the_planner_code_and_the_pose`,
`test_an_empty_reach_verdict_does_not_reject`,
`test_every_knob_gate_precedes_the_reach_verdict` (parametrised over the four knob
gates, each with `reach_verdict='WORKSPACE'` also set, asserting the **knob** code
is what comes back), `test_a_far_lateral_goal_is_no_longer_refused_at_checking`,
`test_a_reachable_displaced_8b_goal_still_reaches_positioning`,
`test_the_predicted_chain_residual_stays_a_few_mm`,
`test_the_predictor_declines_when_the_pose_is_unknown`,
`test_the_predictor_declines_on_tier_8a`,
`test_displacement_within_the_reach_bound_accepted`,
`test_reject_order_num_throws_before_dwell`,
`test_8b_asymmetry_map_radii_span_the_advisory_rings`.

## Verification

* ROS sweep (`python -m pytest tests/ros/ -q -n 4 --dist loadfile`, run
  2026-08-29): **2630 passed, 1 skipped in 102.49 s**.
* Default gate (`./run_tests.sh`, run 2026-08-29): **PASS — 6252 passed, 4 skipped
  in 275.73 s**.
* `full tier: `./run_tests.sh --full` (run 2026-08-29) WEDGED at ~4.5 h in the known OOM class (xdist workers in futex_wait, 1 GB available; killed per the documented remedy) and was NOT re-run at wrap-up (owner asked to close out). Coverage stands on, all 2026-08-29 on this tree: `./run_tests.sh` default gate PASS (**6252 passed, 4 skipped in 275.73 s**); unfiltered `pytest tests/motion/ tests/sim/ -q -n 4 --dist loadfile` (**3389 passed, 3 skipped, 3 xfailed in 435.32 s** — nightly-marked included); post-reach-gate `pytest tests/sim/ -x --ignore=tests/sim/test_toss_gate.py` (**1446 passed, 3 xfailed in 1028.95 s**) + `test_toss_gate.py` (**18 passed**); `pytest tests/ros/ -q -n 4` (**2630 passed, 1 skipped in 102.49 s**). The sole unrun tier is the firmware-native compile battery, on a tree with zero firmware-source changes (the removed generated constants have zero references); the 04:00 nightly re-runs the full tier as the backstop — read `temp/reports/nightly/status` at next session start.`
* **Probe frontier reproduced unchanged** after the delegation
  (`tools/probes/displaced_reach_frontier.py`, run 2026-08-29): **125 mm at
  `T = 0.55 s`**, rising to **~225 mm from `T = 0.70 s`** — the same frontier the
  probe mapped before it shared a body with production, which is the check that the
  lift did not change the answer.
* The refutation itself is a regression pair: the goals that were admitted-then-
  refused-mid-flight (`(250, 0)` at `T = 0.80`, `(500, 0)` at `T = 1.00`) are now
  refused at CHECKING on **both** tiers, and a reachable displaced 8b goal still
  reaches POSITIONING — so the gate cannot pass by refusing everything.

## Outcome

Two chosen numbers left the machine and one measured question entered it. The
operator ramps difficulty with `set_limits` and the flight time, live, instead of
with a YAML pair in a fixed ratio; and a goal the platform cannot reach in the
ball's flight time is refused **before the throw, by the subsystem that actually
knows**, carrying the planner's own code.

The `REJECTED_CHAIN_UNREACHABLE` deletion is the part most likely to be
second-guessed later, so the argument is on the record in two places: its premise
was the deleted box, and a reach-bound-keyed version could never fire
(3.2 mm residual against ≥ 83 mm bounds). It returns under M on a **different**
premise — a hop the walk cannot predict — which is the plan's note, not this
entry's regret.

⚠ **Deploy**: `cd ros_ws && colcon build --packages-select jugglebot` before the
next sitting — the launch runs the install space, so this and its sibling are inert
until it is rebuilt (one build covers the whole wave; `jugglebot_interfaces` is
untouched). Note this wave **regenerates config**, so the build is not optional in
the way a pure-Python edit's is.

**Unflown.** Nothing here has been on hardware. The first displaced sitting is the
first evidence, and the thing to watch is a `REJECTED_POSITION(<planner code>: …)`
appearing at CHECKING where the operator expected a throw — that is the gate
working, and the remedy is a shorter flight or a nearer B, not a wider limit.

Sibling: [[2026-08-29-reload-stay-resume]]. This morning's wave:
[[2026-08-29-rejection-message-enrichment]] (whose displacement verdict this entry
supersedes), [[2026-08-29-position-busy-repoll]] and
[[2026-08-29-bb-reload-busy-patience]].

## Withdrawn claims

- [2026-08-29 early] From [[2026-08-29-rejection-message-enrichment]] § "The
  provenance verdict on `REJECTED_DISPLACEMENT`": *"So: chosen, but **not
  arbitrary, and not removed**"*, kept on the argument that the cap is *"the **only**
  pre-throw stop standing between a too-far goal and the mid-flight-infeasibility
  class C-REACH-1 was written to kill"*, and that raising it is a two-key YAML edit
  offered to the operator.
  **WITHDRAWN**: the owner's decision that afternoon was to delete both keys, and
  the two-key ratio edit was itself the evidence against them (the operator's
  600/600 ramp reddened `test_local_constants_match_generated_config`). The
  *reasoning* about C-REACH-1 was sound and is the reason this entry exists — but
  the conclusion "therefore keep the cap" was wrong: the correct response to "only
  a chosen cap stands between us and an airborne refusal" is to build the
  un-chosen gate, not to keep the chosen one.
  **Superseded by**: this entry's Fix (`motion/trajectory/catch_reach.py`) and
  `plans/active/single-ball-toss.md` Phase E's SUPERSEDED banner.

- [2026-08-29 midday] The deletion's own premise, as written into the first draft
  of the Phase E banner: *"a far-lateral goal is refused pre-throw by the
  positioning move's own feasibility verdict, `REJECTED_POSITION(UNREACHABLE)`"*.
  **WITHDRAWN by measurement** (wave audit, BLOCKING): true for Tier 8a, **false
  for the shipped Tier 8b**, which pre-positions to the pre-tilt pose at A and
  defers the A→B reach to `t_release` — so `go_to_pose` never judges B.
  `B = (250, 0)` at `T = 0.80` and `B = (500, 0)` at `T = 1.00` were admitted and
  then refused `WORKSPACE` mid-flight.
  **Superseded by**: Diagnosis § D3 and the build-time reach gate. The banner in
  `plans/active/single-ball-toss.md` records the gap explicitly as *"a gap for one
  day"* rather than quietly rewriting the premise.

## Open questions / follow-ups

* **The conservative half below `T ≈ 0.75 s` is now the sole residual** on the
  displacement gate, and it is the one that costs the operator feasible throws. The
  thing that would retire it is a measured per-direction reach envelope — the
  "true per-direction physics gate" banked this morning — and the thing that would
  make *that* real is hardware evidence above **70 mm**, which is still where the
  corpus stops.
* **The 18.9 ms lands in a blocking tick.** It is small against the 0.16–0.54 s
  `go_to_pose` overruns already there, and it is M2's *Unblocking the loop* change
  that owns the class. If the reach gate ever shows up in a `tick_max` distribution
  on its own, that is the signal the ordering assumption changed.
* **`sim/toss_gate.py`'s 150 mm ring is now a bare literal.** It no longer tracks
  any shipped number, which is correct (there is no shipped number) but means the
  gate and the hardware ladder agree by convention again — the exact failure mode
  the probe delegation was done to avoid, one directory over. It is advisory and
  labelled so; if the ladder ever re-bases, this is the line to move.
* **M's re-minted `REJECTED_CHAIN_UNREACHABLE`** must be written against the
  unpredictable-park premise and never against a box. The plan carries the note;
  this is the reminder that the note exists.
