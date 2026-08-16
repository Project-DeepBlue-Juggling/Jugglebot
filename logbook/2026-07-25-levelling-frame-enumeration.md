---
title: Levelling-frame ingest enumeration — a sixth external path, a volatile correction, and a guard blind to what it was specified to catch
type: investigation
date: 2026-07-25
status: in-progress
phase: "Self-toss anomaly fixes — levelling-frame-contract Phase 0"
related_plan: "levelling-frame-contract.md"
files_changed:
  - plans/parked/levelling-frame-contract.md
commits:
  - d67c3cd
subsystem:
  - motion
  - ros
tags:
  - kinematics
  - testing
  - docs
  - safety
---

# Levelling-frame ingest enumeration — a sixth external path, a volatile correction, and a guard blind to what it was specified to catch

## Summary

`plans/parked/levelling-frame-contract.md` Phase 0 is an **enumeration** phase:
its whole deliverable is a written list, because the defect the plan closes *is*
an incomplete enumeration — `trajectory_node` applies the gravity-levelling
correction on some pose-ingest paths and not others, so *"level" means two
different things inside one node*. No production code and no tests changed; one
markdown file did.

The enumeration worked, three times over. (1) A **sixth** external ingest path
exists that the original five-row diagnosis missed: `trajectory/timed_target`
with `hold_after=False` passes the stored `_neutral_pose` into the planner as the
**return** target, so fixing only `go_home` would have shipped a half-fix. (2) The
enumeration as first written covered how *poses* arrive but never how the
*correction* arrives — and it arrives over a **volatile** topic, published at most
twice per session, stored in per-process memory with no re-request path and no
observability, which makes Phase 3's gate as wired unable to detect its own
failure case. (3) The structural bypass test that the plan calls "the load-bearing
one" was specified with a manifest key that is **blind to the entire class of
defect this phase just found**.

Findings (2) and (3) came out of the review round, not the first pass. Both are
recorded in the plan; Phase 1 is hard-gated on operator review of the result.

## Symptoms

Not a new symptom — this phase analyses the one already diagnosed in the plan's
Context. During the powered self-toss session of 2026-07-25 15:17:48 the platform
"slowly tilted back, then forward" on every toss between goal and throw.
Commanded `rx` (FK of `/leg_setpoint_echo`) went `+0.0044° → +2.3204° → ~0° →
−1.0784°` across toss #4 with translation never moving; `ry/rx = 0.0888` matched
the session's levelling offset ratio to four significant figures, and the
un-levelled 15:04:35 session with the same action stayed flat to ±0.05°.

## Diagnosis

The enumeration was built from the ROS entry surfaces up (6 subscriptions, 7
services, 2 timers, the emitter thread's post-publish planning, and `follower.py`'s
two planner calls), not by grepping for the existing correction call sites —
otherwise it could only have re-found what was already known.

**Result: 6 external ingest paths, 9 derived seeds, 4 direct plan installs.** Full
table with per-row `file:line` and `E`/`D` classification is in the plan
(§ Phase 0 — Outcome, Table A). The headline row:

| # | Surface | Corrected today |
|---|---|---|
| E1 | `platform_pose_topic` (SpaceMouse/GUI follower) | yes |
| E2 | `catch/dynamic_target` | yes |
| E3 | `trajectory/go_to_pose` | **no** |
| E4 | `trajectory/timed_target` (request pose) | **no** |
| E5 | `trajectory/go_home` (`_neutral_pose`) | **no** |
| **E6** | **`trajectory/timed_target(hold_after=False)` — `_neutral_pose` as the planner's RETURN target** | **no** |

E6 traces `trajectory_node.py:2223` → `:1895` → `planner.build_timed(...,
neutral_pose=...)` → `planner.py:560`, where `_as6(neutral_pose)` is the endpoint
of a real return leg. It has no production client today (no node creates a
`TimedTarget` service client), so correcting it is a latent-path fix.

Both "resolve rather than assume" items are answered from code plus measurement,
not argument:

- **`arm_catch` reach-envelope frame.** Same frame, by construction. Both operands
  are position-only 3-vectors (`self._current_state()[0][:3]` at `:2123-2124`
  versus `target[:3]` at `:2053-2060`) and the correction rewrites `target[3:6]`
  only. Measured position delta under the correction: **exactly 0.000e+00 mm**.
- **`set_limits` / `chase` / `follower`.** No external pose. `SetTrajectoryLimits`
  has three scalar leg-limit fields and no pose field; `chase.py` contains no pose
  constant; `follower._clamp_to_workspace` bisects the `current → target` ray, so
  both endpoints are already-corrected.

The risk-register item resolved **benign**: `motor_guard` consumes the commanded
`pose_6dof` for exactly a condition number and a leg-extension workspace check and
has no notion of level or neutral; DEACTIVATE stows to leg-space `stow_rev = 0.0`;
ACTIVATE trap-trajs to `JB_OP_ACTIVATE_POSITION_REVS` and **nothing compares that
constant to `go_home`'s target**. So no guard, band, or monitor is invalidated.

Neither `sim/` nor `controller/` consumes this correction anywhere — the only hit
across both trees is `controller/teensy_link/rpc_args.py:246-256` encoding the
boolean `levelling_complete` flag. Stated explicitly in the plan so a future
reader does not go looking.

### Quantities (all re-derived independently at finalize)

Session offset `[0.013592347421588673, 0.001207157476773584]` rad through the
production helpers (`motion.ik_solver`, `motion.geometry.StewartGeometry`):

| quantity | value |
|---|---|
| corrected identity-target rotvec | `(-0.77878414°, -0.06916503°, 0°)` |
| total tilt | `0.78184944°` |
| position delta under the correction | `0.000e+00 mm` |
| per-leg extension deltas | `[-1.4206, +2.7355, +2.7736, -0.9783, -1.3493, -1.7535]` mm |
| worst leg | `2.7736 mm = 0.03908 rev` (13.0 % of the 0.3 rev pump step gate; 3.9 % of the firmware 1.0 rev MAX_DEVIATION band; **322×** the 8.607e-3 mm encoder dead-band) |
| naive `plat_radius·sin θ` estimate | `2.9894 mm` — **overstates by 7.8 %**, because the tilt axis is not leg-aligned |
| composition non-commutativity, target `[0.15,-0.08,0]` | `1.268e-3 rad (0.0727°)`, entirely in `rz` |
| additive-lean vs SO(3), 5° lean | `5.931e-4 rad (0.03398°)` about **`ry`**; only `5.267e-5 rad` about `rx` |
| cup lateral swing of the correction | `1.2977 mm` against a 35 mm cup radius |

The plan's pinned worked example and its `0.0888` axis-match claim both reproduce
exactly, so Phase 1 can pin them as written.

## Discussion

### Why the enumeration was built from entry surfaces, not from the existing call sites

The tempting method is `grep _apply_gravity_correction`, list the hits, and correct
the surfaces that are missing. That method **cannot find E6**, and would have
produced a confident five-row table identical to the one already in the plan's
Context. E6 carries no correction call site to grep for and is not a distinct ROS
surface — it is a *second pose argument on a call that another row already owns*.
Enumerating from `create_subscription` / `create_service` / timers / threads
downward, and then asking of each pose "where does it enter, and where does it
reach `planner`", is what made a surface with two independent poses expressible at
all. The cost is real (it re-derives things the grep would have got for free); the
return is the one row that mattered.

### The table shape was the load-bearing decision

The Context table is two columns: *surface* × *corrected today*. That shape
**cannot represent E6** — `timed_target` is one surface with two independent
external poses, so any surface-keyed row must pick one and silently drop the
other. That is not hindsight about how E6 hid; it is mechanically why. Table A
therefore has separate "Pose enters at" and "Planner entry" columns and is keyed
`E1..E6` / `D1..D9` per **pose**, not per surface. Every subsequent finding in
this phase is downstream of that choice.

### The finding I did not expect: the guard reintroduced the blind spot it was specified to close

The first pass specified the structural bypass test's manifest key as *(enclosing
function, source text of the target argument)*. Review showed that key is blind to
the whole E6 class, and the demonstration is airtight:

```
planner.build_timed(self._current_state(), target, twist, lead,
                    self._limits, self._geom,
                    hold_after=hold_after, neutral_pose=neutral)   # :1892-1895
```

is **one `ast.Call` node** carrying **two** external poses. Enclosing function
(`_plan_and_install_timed`) and target source text (`target`) are byte-identical
with or without `neutral_pose=neutral`. So the phase found the sixth path by hand
and then specified a guard that could not see its shape — while the risk register
asserted the opposite ("the bypass test... must be structural, because a
per-surface behavioural test could never have caught E6"). Both halves of that
sentence were true; the *specification* underneath it was not.

This is not a hypothetical gap. `build_catch` also takes `neutral_pose`
(`planner.py:785`, required at `:834`, consumed at `:910`) while
`_plan_and_install_catch` hard-codes `hold_after=True` (`:1948`). A future one-line
change — "make the catch reach return to neutral" — adds a **seventh** external
ingest inside the same enclosing function with the same target argument text.
Under the original key every assertion still passes, the post-catch return parks
at plan-frame `rx = 0` while everything else parks at gravity-level, and the
contract's own guard reports all-clear. That is the identical defect, silently
reintroduced, with a green suite.

The fix widens the key to *(module, enclosing function, planner callee, {argument
name → source text} over the **pose-bearing** arguments)*, enumerates those
arguments per planner function, and **freezes the parsed file set as data** — the
second half being necessary because Table A's own E1/D8 rows cite planner entries
in `follower.py` (`:212`, `:227`), so an AST pass over `trajectory_node.py` alone
could not be equal to the manifest it freezes. Assertion 3 was also restated
per-**row** rather than per-surface: "exactly 1 application per surface" is
literally unimplementable for `timed_target(hold_after=False)`, which must produce
**2**.

The general lesson, and the reason this is in Discussion rather than a bullet in
the plan: **a guard specified in the same pass that found a defect tends to be
keyed on the defect's *symptom* rather than its *shape*.** E6's symptom is "a
neutral pose was uncorrected"; its shape is "one call site, two poses". A key
built from the symptom looks sufficient right up until the shape recurs.

### Why the correction's own delivery belongs in the enumeration

The first pass took "the correction" as a static transform and asked only which
poses meet it. Reviewers, independently of each other, asked the mirror question:
*does the node have one loaded at all?* It is the same class of reasoning error the
plan exists to close — a third pair of meanings of level, "levelled on the Teensy"
versus "a correction is resident in this `trajectory_node` process" — so excluding
it from an enumeration phase would have been arbitrary. Table C now records the
delivery path: VOLATILE QoS, one latched publish per orchestrator boot
(`_startup_offset_sent`) plus one per `level`, in-memory storage with a single
writer, no re-request, and **no field on `TrajectoryStatus`** that would reveal its
absence.

The consequence is a live contradiction inside Phase 3 that would otherwise have
shipped: step 4 says *gate on "a correction is loaded", not "levelling ran this
session"* — correct — but step 3 wires the observation to
`RobotState.levelling_complete`, a **Teensy-persisted per-boot flag** that says
nothing about any ROS process's memory. A `trajectory_node` restarted after `level`
(a crash, or precisely the `colcon build` + relaunch this plan itself mandates for
Phases 1–3) holds identity, `levelling_complete` still reads True, and
`REJECTED_NOT_LEVELLED` would pass in exactly the state it exists to refuse. Worse
after Phase 2 than before: today a missing correction shows up as a visible mid-goal
tilt excursion, whereas afterwards correction-loaded and correction-lost differ only
by a static 2.77 mm leg offset nobody checks.

**Deliberately not decided here.** Two closes are viable — transient-local QoS on
`/gravity_offset`, or expose the loaded correction on `TrajectoryStatus` and gate on
that (the only one that satisfies step 4 literally). Phase 3 owns the gate and owns
the choice; Phase 0's job was to make the hazard knowable and to stop Phase 3 from
wiring the observation it already ruled out. Inventing the wire format two phases
early would have been the sunk-effort failure the checkpoint rule warns about.

### Ingest-side placement gets the in-flight rule for free — a second, stronger argument

The plan justifies ingest-side over emitter-side placement by the feasibility gate:
an emitter-side correction would let `validate` measure an uncorrected plan while
the legs run a corrected one. The enumeration surfaces an independent argument the
plan did not make. Because ingest-side placement **bakes the correction into the
plan's endpoint at build time**, nothing re-reads `R` for an already-installed plan,
so a mid-session offset arrival simply takes effect at the next install — which is
exactly the "live plan keeps its frame" default the plan wanted, obtained for free
rather than enforced. Emitter-side, the same arrival would step `u0` by the full
offset delta on the very next 25 ms knot: a **command discontinuity on the wire**,
not merely an ungated plan, and unbounded in the offset. That is the version a
reader under shipping pressure will find harder to dismiss, so it is now recorded
alongside the gating reason.

### Tradeoffs accepted

- **The enumeration lives inline in the plan, not in a separate findings file.**
  The gate that consumes it is "the operator reviews the enumeration before Phase 1
  starts". A decoupled artefact rots — its line numbers drift, nobody re-reads it —
  and the reviewer never sees it at the moment of the decision. This repo has
  already paid that cost with `/tmp` probe references in the 2026-05-20
  warm-start-deadlock entry, whose findings files sit one power-cycle from
  deletion. The cost accepted is a long plan file.
- **Edits were made in five places *outside* the Phase 0 section** (Context
  root-cause table, C-LEVEL-1, Phase 2 steps 2/4/5/6, Phase 3 steps 3/4, risk
  register).
  A Context table listing five surfaces while the Outcome lists six is precisely
  the cross-document inconsistency the `/audit` gate exists to catch; a Phase 2
  step 2 saying only "correct `_neutral_pose` at use (`go_home`)" would let the
  next implementer ship the half-fix that finding E6 exists to prevent.
- **The probe stays uncommitted** at `/tmp/probe_levelling_frame_p0.py`. It answers
  one-time questions and will not be re-run; promoting it would add an abandoned
  script to a directory whose value is that everything in it is live. Mitigation
  for the volatility: the plan now records the *recipe* (which offset, which
  production helpers, which composition order) rather than the path, so every
  number is re-derivable from the text — and Phase 1 turns the load-bearing one
  into a permanent unit test, which is the right home for it.
- **`tests/hardware/session_anomaly_fixes.md` was deliberately left untouched.**
  Phase 0 has no robot-actuating step, so a section would read "nothing to run
  here". Hollow sections train the operator to skim a runbook whose value is that
  every line is executable. The operator pre-brief the numbers feed is a *Phase 2*
  obligation and is now written into Phase 2 step 6, where the code that makes it
  true will land.

### C-LEVEL-1 said "single enforcement point"; the enumeration proves it cannot be one

The contract as drafted promises the correction is "applied exactly once, at a
single enforcement point", while Table A prescribes helper routing at five sites in
`trajectory_node` plus one in `mpc_bridge_node`. Left as-is, a later `/simplify` or
audit pass would count six call sites, conclude the contract is already violated,
and resolve it the cheap way — relaxing the text, or collapsing the sites into the
emitter (the placement the plan forbids for the gating reason above).

The root cause, restated so the choice can be judged on its merits rather than on
the contract's authority: **derived poses reach the same planner entries as external
ones** — nine `D` rows against six `E` rows, plus four direct plan installs that
bypass `planner` entirely — and a choke point at the
planner entries has no way to tell them apart, so it would double-apply on every
FK-seeded hold. That is the mirror bug C-LEVEL-1's second half exists to prevent
(`rx = −0.7788°` seeded then re-corrected to `−1.5576°`). Provenance is only
available at ingest. So the invariant is deliberately distributed across an
*enumerated* set, and the enumeration is what the bypass test freezes. C-LEVEL-1 now
reads "exactly once per external pose, through a single shared implementation, at
the enumerated ingest sites and at no other site", with that reason attached.

## Fix

One file: `plans/parked/levelling-frame-contract.md`.

- **New § Phase 0 — Outcome** (the deliverable): the grep audit with exclusions and
  dead-copy accounting; **Table A** (6 external / 9 derived / 4 direct installs,
  per-pose keys, `file:line` for both the ingest point and the planner entry);
  **Table B** (`mpc_bridge_node`, one surface, zero planner reach); **Table C** (how
  the correction itself arrives and how it is lost); both resolved questions; the
  `sim/`+`controller/` non-consumption statement; the `motor_guard` / stow /
  ACTIVATE risk-register resolution; the Phase-2 regression surface; and the four
  assertions the structural bypass test needs.
- **Context root-cause table** gains the E6 row, so it no longer contradicts the
  Outcome.
- **C-LEVEL-1** restated (exactly once *per external pose*, single shared
  *implementation*, rotational component only) with the "why not one choke point"
  root cause recorded beneath it.
- **Phase 2** gains step 6 (reword the prose Phase 2 falsifies —
  `build_return_to_neutral`'s docstring and two hardware runbooks), a pointer from
  step 2 to E6, a non-optional structural bypass test in step 4, mid-plan sampling
  in step 5, and a gate restated per grep-table row rather than against a headline
  sum that can never reach zero.
- **Phase 3** steps 3 and 4 rewired away from `RobotState.levelling_complete`, with
  the Table C reason inline.
- **Risk register**: the `go_home`/stow row closed as RESOLVED-benign, the
  "sixth ingest path" row pointed at the manifest-key hazard, and a new row for
  "correction never delivered / lost on partial restart".

### Corrections applied at finalize (review findings, all verified against code first)

| # | Finding | Disposition |
|---|---|---|
| 1 | Manifest key blind to the E6 class; assertion 3 unimplementable per-surface | fixed — key widened to pose-bearing arguments; assertion 3 per-row |
| 2 | AST pass scoped to `trajectory_node.py` while the manifest spans 3 files | fixed — file set frozen as data |
| 3 | Correction delivery volatile/unobservable; Phase 3 gate blind to it | fixed — Table C + risk row + Phase 3 rewire |
| 4 | "Single enforcement point" contradicts 6 helper sites | fixed — C-LEVEL-1 restated with root cause |
| 5 | External twists never enumerated | fixed — recorded as uncorrected *by contract*, with the failure mode |
| 6 | `_neutral_pose` inventory listed 4 of 5 reads (`:1585` missing) | fixed — inventory declared exhaustive at 5 |
| 7 | Grep total "13" sums overlapping categories | fixed — 11 distinct statements (12 incl. the sign temporary); gate restated per row |
| 8 | `TimedTarget` grep claim did not reproduce | fixed — restated as what was checked (no `create_client`; 4 comment hits named) |
| 9 | "The one place pinning commanded level ⇒ activate revs" under-enumerated | fixed — 3 artefacts named, incl. two emitter tests; also corrected the false claim that `verify_motor_commands.py` drives the MPC path (it is a standalone offline IK check) |
| 10 | `build_return_to_neutral` docstring falsified by Phase 2 | fixed — added to Phase 2 step 6 with the wrong-turn it would re-enable |
| 11 | Two hardware runbooks document teardown `go_home` as a no-op | fixed — added to Phase 2 step 6 with the expected magnitude |
| — | *finalize-found:* `follower.follow` cited at `:1301`, actually `:1300` | fixed |
| — | *finalize-found:* dead-band ratio written `323×`, actual `322.25×` | fixed to `322×` |
| — | *finalize-found:* `rz` assertion cited at `test_toss_coordinator.py:1480`, actually `:1482` | fixed |
| — | *finalize-found:* additive-lean discrepancy quoted without its axis (11× smaller about `rx`) | fixed — `ry` pinned as the worst case |

## Verification

No production code and no tests changed, so no test can exercise this phase. What
was verified instead:

- **Every `file:line` citation in the plan re-resolved mechanically.** A checker
  script extracted all **52** distinct fully-qualified `path:line` citations,
  resolved each basename against the live source trees (excluding `ros_ws/build`,
  `ros_ws/install`, and the gitignored repo-root `build/` + `install/`), and printed
  the cited source line for inspection. All 52 resolved — no dangling paths, no
  out-of-range lines. Reading the printed lines is what caught the one whose line was
  valid but pointed at the wrong construct: the `rz` assertion cited at
  `test_toss_coordinator.py:1480`, which is actually the `rx` assertion (`rz` is at
  `:1482`).
- **The bare `:NNNN` citations** (the majority of Table A, written without a repeated
  filename) were spot-checked by hand against `trajectory_node.py`,
  `orchestrator_node.py`, `follower.py`, `planner.py` and the launch file — about 45
  of them, including all 13 `planner.*` call sites, all 5 `_neutral_pose` reads, and
  every line in Table C. One was wrong: `follower.follow` is at `:1300`, cited as
  `:1301`.
- **Every headline number re-derived independently** at finalize against the
  production helpers under the venv, not taken from the implementer's probe. All
  reproduce exactly, including the two that had been rounded or under-specified
  (see the table above).
- **Full suite** (`pytest tests/ -q`, run 2026-07-25 on the Jetson under
  `~/Desktop/PDJ_venv/venv`): **3429 passed, 3 xfailed, 198 warnings in 1331.26 s
  (0:22:11)**. The pre-phase baseline at `e513036` was 3429 passed, 3 xfailed in
  1340.73 s — **identical pass and xfail counts**, as it must be, since this phase
  changed no file that pytest imports. The 9.5 s wall-clock difference is run-to-run
  variance on this Jetson, not a test-count change. The **xfail count held at 3**, so
  no test was weakened to reach green. Both of the known order/load-flaky
  allocation-budget tests (`test_hot_loop_allocation_contract`,
  `test_t3b_h4_on_post_solve_allocates_within_budget`) passed *inside* the full
  suite this run, so no isolated re-run was needed.
- **The tested tree is the committed tree** for every file pytest can observe. This
  phase touches only markdown (`plans/parked/levelling-frame-contract.md`, this
  entry, `logbook/INDEX.md`); no `*.py` and no `*.yaml` changed anywhere, so no
  regeneration step applies and the markdown edits made while the suite ran cannot
  have affected its result.
- **No tracked file was modified by the implement pass**: `git diff` and
  `git diff --cached` were both empty before finalize, so nothing could have been
  xfailed, skipped, or deleted to reach green.

## Outcome

Phase 0 is complete and **hard-gated on operator review** — Phase 1 does not start
until the enumeration has been read. The gate's stated success condition ("if it
turns up a sixth external path nobody expected, that is the phase working") was
met, and two further results changed downstream phases: Phase 3's gate cannot be
wired to `RobotState.levelling_complete`, and Phase 2's bypass test needs a wider
manifest key than first specified.

No hardware validation for this phase — there is no robot-actuating step, so
`tests/hardware/session_anomaly_fixes.md` was deliberately not appended to. The
numbers the eventual Phase-2/4 section needs (operator pre-brief magnitudes, the
`±0.05°` criterion's measurability, reading `/gravity_offset` back out of the
launch rosbag rather than trusting that `level` ran) are recorded in the plan and
wired into Phase 2 step 6.

No deployment: one markdown file, no `ros_ws/src` change, no config YAML, no
firmware. Phases 1–3 will need `colcon build --packages-select jugglebot` **plus a
relaunch** (the launch runs the installed copy); no phase of this plan touches
firmware.

## Open Questions

- **HARD STOP, by design — the E6 decision.** Confirm Phase 2 should correct E6 now
  rather than leave it. It has no production client today, so it is a latent-path
  fix; the argument for now is that a latent uncorrected path is exactly how the
  current bug got in, and a future client would inherit the defect silently.
- **Is "aim relative to gravity" the intended semantics for the throw?** After
  Phase 2, `reload_coordinator_node`'s Tier-8b POSITIONING tilt becomes
  `R_gravity @ R_tilt`, so the ball launches along an axis aimed relative to
  **gravity** rather than to the plan frame. That is what a ballistic aim should
  do. But `MAX_TILT_DEG = 12.0` (`tilt_geometry.py:62`, gated in
  `toss_release.py:295-296`) is evaluated on the **required** aim before the request
  goes out, so afterwards the commanded plan-frame tilt can exceed 12° by up to the
  correction while the physical tilt versus gravity equals the clamped value. Leg
  feasibility is unaffected (the gate still measures the commanded pose), but the
  clamp's units change meaning. Operator physical intuition wanted: is the 12°
  ceiling a **tracking-degradation** limit (gravity-frame is right) or a
  **mechanical** one (plan-frame matters)? `tilt_geometry.py:58-62` says tracking
  degradation, which points to gravity-frame.
- **Phase 3 must choose the "correction loaded" observation** — transient-local QoS
  versus a `TrajectoryStatus` field. Deliberately left open here; see Discussion.
- **Sibling-plan interaction.** After Phase 2 the commanded rotvec excursion on the
  toss catch reach goes to approximately zero, which *is* the near-degenerate case
  `plans/parked/catch-reach-degenerate-overshoot.md` studies. Its Phase 0 should
  re-verify its reproduction against a post-Phase-2 tree rather than assume the
  pre-fix `+2.32°` excursion still exists to measure.
