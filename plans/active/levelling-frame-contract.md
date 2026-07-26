---
title: Levelling-frame contract — one gravity correction, applied to every pose request
created: 2026-07-25
status: active
related_logbook:
  - 2026-07-26-levelling-frame-contract.md
  - 2026-07-25-levelling-frame-enumeration.md
  - 2026-07-25-toss-phase3-trace-validated.md
  - 2026-07-25-toss-phase4-tier8b-displaced-throw.md
  - 2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/levelling.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::_pose_from_msg
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::_corrected_neutral_pose
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::_catch_target_from_msg
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::_on_platform_pose
  - ros_ws/src/jugglebot/jugglebot/mpc_bridge_node.py::_on_platform_pose
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py::build_catch
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py::_step_checking
  - ros_ws/src/jugglebot/jugglebot/orchestrator_node.py
---

# Plan — Levelling-frame contract

**Branch:** `mvp-trajectory-bringup`
**Covers:** fix items 1 (universal correction) and 2 (`REJECTED_NOT_LEVELLED` gate)
from the 2026-07-25 self-toss anomaly investigation.
**Sibling plans:** `plans/active/hand-command-continuity.md` (items 3–6),
`plans/active/fk-convergence-tolerance.md` (item 7),
`plans/active/catch-reach-degenerate-overshoot.md` (item 8).

> The retrospective diagnosis for this plan has **not yet been written to the
> logbook**. Whoever starts Phase 0 should either write it first or confirm with
> the operator that it is being written in parallel — the measurements quoted
> below are the entire evidence base and they currently live only in this plan.

## Context

### The observation

During the powered self-toss session of 2026-07-25 15:17:48 the operator reported
that on **every** toss the platform "slowly tilted back, then forward" between
sending the goal and the throw, with no apparent purpose. Five toss attempts,
reliably reproduced.

### What the data shows

Commanded platform pose, reconstructed by forward kinematics from
`/leg_setpoint_echo` (the *accepted* leg setpoints echoed by `teensy_bridge_node`)
in `~/Desktop/rosbags/2026-07-25_15-17-48/2026-07-25_15-17-48_0.mcap`, toss #4
(ball 11, scheduled release 1784956866.88):

| t rel. release | x (mm) | y (mm) | z (mm) | rx | ry |
|---|---|---|---|---|---|
| −4.23 s | 0.00 | 0.00 | 170.00 | +0.0044° | +0.0004° |
| −1.98 s | 0.00 | 0.00 | 170.00 | **+2.3204°** | +0.2061° |
| −0.92 s | 0.00 | 0.00 | 170.00 | ~0° | ~0° |
| −0.50 s → +2.00 s | 0.00 | 0.00 | 170.00 | **−1.0784°** (flat) | −0.0958° |
| +3.92 s | 0.00 | 0.00 | 170.00 | 0.0000° | 0.0000° |

Translation never moves. It is a pure rotation, dominated by `rx`, and it is
reproduced on toss #3 (+2.37° → −1.10°). Mocap `/rigid_body_poses` confirms the
platform physically follows the command (leg tracking error ≈ 0.005 rev against a
0.08 rev excursion).

Three independent facts identify the cause:

1. **Axis.** Commanded `ry/rx = 0.0958/1.0784 = 0.0888`. The levelling gravity
   offset published that session was `[0.013592347421588673,
   0.001207157476773584] rad`, ratio **0.0888**. Four-significant-figure match:
   the tilt axis *is* the levelling correction axis.
2. **The wire is level.** The only platform target published in that window is a
   single `catch/dynamic_target` with `target_pos = (0, 0, 170)` and an
   **identity** `target_quat`. The tilt is injected downstream of the wire.
3. **Falsification.** Session 2026-07-25 15:04:35 never ran LEVELLING, so the
   correction was identity. Same action, same goal, same code: commanded `rx`
   stays within **±0.05°** for the entire toss — flat. Levelled ⇒ tilt;
   un-levelled ⇒ no tilt.

### Root cause

`trajectory_node` applies the levelling correction on some pose-ingest paths and
not others, so *"level" means two different things inside one node*:

| Ingest surface | `_apply_gravity_correction` today |
|---|---|
| `catch/dynamic_target` (`_on_dynamic_target` → `trajectory_node.py:1983`) | **yes** |
| `platform_pose_topic` — SpaceMouse/GUI follower (`:1240`) | **yes** |
| `trajectory/go_to_pose` (`_pose_from_msg` `:1594`, called `:1663`) | **no** |
| `trajectory/timed_target` (`_pose_from_msg`, called `:2184`) | **no** |
| `trajectory/go_home` (`_neutral_pose` `:200`, used `:1575`) | **no** |
| `trajectory/timed_target(hold_after=False)` — `_neutral_pose` as the planner's RETURN target (`:200`, used `:2223`) — *found by Phase 0, not by the original diagnosis* | **no** |

The toss's POSITIONING `go_to_pose` therefore parks the platform at plan-frame
`rx = 0`; the pre-tilt `catch/dynamic_target` then asks for plan-frame
`rx = −0.7788°`; `build_catch` plans a min-jerk reach between the two with
arrival = landing − 1.5 s = **release − 0.7 s**. That is the observed motion, and
its timing is why it always completes just as the ball is thrown.

Two collateral notes from the same diagnosis:

- The POSITIONING move the operator assumed was the tilt is a genuine **no-op**:
  `/trajectory/diagnostics` shows `move_seq=47` with planned peak leg
  vel/acc/jerk all `0.0`. The motion is the pre-tilt catch reach ~0.6 s later.
- `mpc_bridge_node` carries a **second verbatim copy** of the correction
  (`mpc_bridge_node.py:144-154`). It has only the `platform_pose_topic` path, so
  it is not internally inconsistent today, but two copies of a normative
  transform is exactly how contracts drift.

### Why this matters physically (and why LEVELLING appeared to fix the toss)

The correction exists so that commanding "level" produces a platform that is level
*with respect to gravity*. Lateral drift of a vertical toss is
`v·sin(θ)·T` for a residual cup tilt `θ`:

| state | commanded `rx` at release | net tilt vs gravity | drift over 0.8 s at 3.93 m/s |
|---|---|---|---|
| un-levelled | 0° | 0.78° | **43 mm** |
| levelled, today | −1.08° | ~0.30° | **16 mm** |
| levelled, after this plan | −1.08° | ~0.30° | **~16 mm** — see the correction below |

The cup radius is ~35 mm (`GEOM_HAND_RADIUS_MM`), so un-levelled the catch is
*geometrically impossible* and levelled-today it barely fits. Ball 11's
tracker-measured catch error was **16 mm** — the predicted value for a 0.30°
residual.

> **CORRECTED 2026-07-25 by Phase 2's implementation.** The third row originally
> read *"−0.78° / ~0° / ~0 mm"*, i.e. this plan claimed to close the 16 mm catch
> error. **It does not**, and the reasoning that produced that row conflated two
> different contributions to the observed −1.0784°.
>
> Only the **park frame** is this plan's. The **−1.0784° settle** is
> `planner.build_catch`'s tilt-through-seat ramp: it reads `catch_pose[3:5]` as
> "the receive tilt" and ramps a residual rate along it, a premise that holds
> only while the commanded frame **is** the gravity frame. With a correction
> loaded, a gravity-level catch arrives as a non-zero plan-frame tilt — the
> correction itself — so the through-seat engages and overshoots by
> `_CATCH_TILT_OVERSHOOT_FRAC · rate · decay = 0.5 × 0.07 × 0.15 = 0.005250 rad
> = 0.3008°` in the correction's own direction. Closed form for the session
> offset: **−1.078408° `rx` / −0.095775° `ry`**, against the bag's recorded
> **−1.0784 / −0.0958**; measured through `tools/probes/levelling_tilt_bag_check.py`
> on bag `2026-07-25_15-17-48` (window `--t0 165 --t1 200`, run 2026-07-25) the
> settle plateau reads **−1.0775°**, i.e. **0.0009°** from the closed form.
>
> Two corollaries the plan previously left open:
> - the un-levelled 15:04:35 baseline is flat **because** `tmag == 0` disables
>   the through-seat entirely, not merely because "the frames agree". So Phase 4's
>   *"match the un-levelled baseline"* is unreachable once a correction is loaded;
> - the *"settled at 1.385× the single correction"* residual-amplification question
>   in § Notes for collaborators is **answered**: `(tmag + overshoot)/tmag =
>   1.38473` exactly.
>
> Closing the remaining 0.30° needs `build_catch` to aim the through-seat along
> the **gravity-referenced** receive tilt rather than the commanded plan-frame
> tilt. That changes commanded motion at ball contact on every catch including
> the shipping reload path, so it is an operator decision and was deliberately
> **not** taken by the Phase-2 session. It is pinned as a characterisation test
> (`tests/ros/test_levelling_frame.py::test_catch_through_seat_aims_off_the_gravity_referenced_receive_tilt`)
> so it cannot drift silently.

> **CORRECTED AGAIN 2026-07-26 by the Phase-1/2 finalize pass — the +2.32° swing
> is NOT this plan's either.** The note above still claimed the mid-plan
> excursion as this plan's deliverable. Measured through the production planner
> at the reference session's own catch lead, it is not.
>
> `build_catch` specifies a non-zero **arrival twist** on its reach (`rate ·
> tdir`, `_CATCH_TILT_THROUGH_RATE_RADPS = 0.07`), so a reach whose start and end
> tilts are *identical* must still swing out and come back. With `p0 == p1` the
> quintic collapses exactly to `v1·T·φ(s)`, `φ = −4s³ + 7s⁴ − 3s⁵`, `|φ|` maximal
> `16/81` at `s = 2/3`. Hence
>
> > `peak_above_park = (16/81) · rate · |tdir_x| · lead` = **0.789132° per second
> > of catch lead** for this offset direction — exactly linear, and **independent
> > of the correction's magnitude**.
>
> Verified against `planner.build_catch` at leads 0.8 / 1.2 / 2.0 / 3.0 / 3.7 /
> 5.0 s to 4 dp (2026-07-26). The reference bag's `+2.3204°` plan-frame peak
> corresponds to a **3.70 s** catch lead; at that same lead the post-fix reach
> peaks at `+2.1410°` plan-frame, i.e. **+2.9198° against gravity**, against the
> pre-fix **+3.0992°**. The swing shrinks by **0.18° out of 2.9°**, and the
> quantity `peak_above_park` — measured from where the platform rests — actually
> **rises**, because pre-fix the park itself sat 0.7788° high and hid part of it.
>
> What this plan does close is the **park**: the platform rested 0.78° off
> gravity and `go_to_pose` and `catch/dynamic_target` disagreed about where level
> was. That is real and worth having, and it is what CHECK LVL-3 gates on.
> Removing the swing is `plans/active/catch-reach-degenerate-overshoot.md` —
> which the finalize pass also re-scoped (§ Notes for collaborators).
>
> Two downstream corrections this forced: Phase 4's `peak_above_park` ABORT at
> `+3.099°` is **retired** (it fires on a healthy system at any lead ≥ 3.93 s and
> sat 0.18° from firing at the lead the reference session ran), and the operator
> pre-brief now says in as many words that the visible tilt REMAINS. Pinned as
> `tests/ros/test_levelling_frame.py::test_the_reach_excursion_across_the_plan_is_gone_after_c_catch_1`
>
> > **SUPERSEDED, 2026-07-26 — the swing is GONE, not merely smaller.** The
> > analysis above stands as written (it is what made the second contract
> > necessary), but its operator-facing conclusion is now false. C-CATCH-1
> > (`ros_ws/docs/catch_arrival_contract.md`, landed with
> > `plans/active/catch-reach-degenerate-overshoot.md` Phase 2) passes the
> > gravity-referenced receive tilt to `build_catch` as its own argument, so a
> > gravity-level catch gets **no through-seat at all** and its reach is FLAT:
> > `2.3218–2.3340° → 0.0000°` on all five self-toss reaches of the reference
> > bag. Do **not** brief the operator to expect the `≈+2.92°` swing — a healthy
> > post-fix machine will not produce it, and scoring the flat capture against
> > this paragraph would reject a correct fix. The pinning test name in the line
> > above already reads `..._is_gone_after_c_catch_1`.
> and, on the instrument side, by
> `tools/probes/levelling_tilt_bag_check.py --self-check` +
> `tests/motion/test_levelling_probe.py`.

So the ugly tilt is the correction doing approximately the right thing through
the wrong plumbing; the *plumbing* — and specifically the resting frame — is what
this plan fixes. The swing you can see is `build_catch`'s.

## Architecture

### The contract

> **C-LEVEL-1.** The gravity-levelling correction is applied **exactly once per
> external pose**, through a **single shared implementation**, at the enumerated
> ingest sites (§ Phase 0 — Outcome, Table A rows `E*`) and **at no other site**.
> "External" means it entered from outside the node: a service request, a wire
> target, or the built-in neutral constant. It is applied to **no** pose derived
> from measurement or from an existing plan. It rewrites the **rotational
> component only** — positions, linear velocities and accelerations pass through
> untouched.

**Why "one shared implementation at N enumerated sites" and not "one choke
point".** A single choke point would have to sit where every pose converges —
the planner entries, or the emitter. It cannot: *derived* poses reach the same
planner entries as external ones, and they outnumber them — **nine `D` rows
against six `E` rows in Table A, plus four direct plan installs that bypass
`planner` entirely** — and a choke point there has no way to tell them apart, so it would
double-apply on every FK-seeded hold — the exact mirror bug the contract's
second half exists to prevent. The discrimination is only available at ingest,
where the pose's provenance is still known. So the invariant is deliberately
distributed across an *enumerated* set, and the enumeration is what the bypass
test freezes (Phase 2). Anyone later reading "single enforcement point", counting
six call sites and concluding the contract is already violated would resolve the
contradiction the cheap way — either relaxing the contract text or collapsing the
sites into the emitter, which the § Placement decision below forbids for an
independent reason.

The second half is not decoration — it is the mirror-image bug waiting to be
introduced. `trajectory_node` seeds holds by forward kinematics from the encoders
(`seeded hold at pose … from measured telemetry`). Once plan-space "level" *is*
`rx = −0.7788°`, an FK-derived seed reads −0.7788° on its own; routing the seed
through the correction as well yields −1.5576°. Measurement-derived and
plan-derived poses are already in the corrected frame by construction.

Three parts, landed together (the repo's contract pattern):

1. **Normative statement** — a section in `controller/REFERENCE_LAYER_CONTRACT.md`
   (or a new `ros_ws/docs/levelling_frame.md` if the reviewer judges the reference
   layer to be the wrong home; decide in Phase 1 and record the reason).
2. **One shared implementation** — a single helper that every external pose, and
   only an external pose, passes through (see the note above on why this is an
   enumerated set of call sites rather than one choke point).
3. **A test that fails if a new ingest path bypasses it** — and that fails on the
   *omission*, not merely on a wrong value. Phase 0 specifies what makes it
   structural; the manifest key is itself load-bearing.

### Placement decision (settled, with the reasoning recorded)

Apply at **target ingest**, not in the emitter.

The emitter is one line and superficially attractive, but it puts the correction
*outside* the feasibility gate: `validate` would then measure an uncorrected plan
while the legs execute a corrected one. A 0.78° tilt is a small leg-space
perturbation today, but the gate's whole value is that it measures the object the
emitter actually runs (`planner.build_catch`'s "single-gate contract" comment
makes the same argument). Ingest-side correction keeps plan == emitted == gated.

### Where the helper lives

Pure-python `ros_ws/src/jugglebot/jugglebot/motion/` (no ROS imports), so both
`trajectory_node` and `mpc_bridge_node` consume one implementation and the
`mpc_bridge_node` copy is deleted. Python 3.8: `from __future__ import
annotations`.

## Implementation Phase Summary

| Phase | Scope | Gate | Status |
|---|---|---|---|
| 0 | Ingest-surface enumeration + frame audit (no code) | written enumeration reviewed by the operator | **DONE 2026-07-25 — see § Phase 0 — Outcome. Operator gate CLEARED 2026-07-25 (`3365ac8`). Three results that change later phases: a SIXTH external path (E6, `timed_target(hold_after=False)`'s neutral return target); the correction's own delivery is volatile + unobservable (Table C ⇒ Phase 3 cannot gate on `levelling_complete`); and the bypass-test manifest key must widen or it is blind to the E6 class it was specified to catch.** |
| 1 | Contract doc + `motion` helper + unit tests | full pytest | **DONE 2026-07-26 — `aea7b49`. Operator gate cleared in `3365ac8`. Extraction verified bit-identical (max &#124;Δ&#124; = 0.000e+00 over 36 offset×orientation pairs), contract C-LEVEL-1 at `ros_ws/docs/levelling_frame.md`. See § Phase 1 — Outcome.** |
| 2 | Migrate every ingest path (**six**, not five); delete the ad-hoc call sites; bypass test | full pytest | **DONE 2026-07-26 — `aea7b49`. All six E surfaces + `mpc_bridge_node`'s B1 route through the shared helper; the verbatim second copy is deleted. Two AST manifests with discovered file sets, mutation-verified against six seeded regressions. Surfaced and escalated (NOT fixed): `build_catch`'s through-seat aims off the plan-frame tilt. See § Phase 2 — Outcome.** |
| 3 | `REJECTED_NOT_LEVELLED` gate in toss CHECKING | full pytest | **DONE 2026-07-26 — `e36d60d`.** Table C's contradiction is resolved by candidate **(b) only**: `TrajectoryStatus` gains `gravity_correction_loaded` and the gate observes the APPLIER, on a FRESH status. Transient-local QoS (candidate a) was considered and REJECTED — its latch lives in the publisher, so the whole-graph relaunch that motivates the hazard would not benefit. See § Phase 3 — Outcome. |
| 4 | Hardware validation (operator-run) | park plateau within ±0.05° of `(−tilt_x, −tilt_y)` via `tools/probes/levelling_tilt_bag_check.py`; first `go_home` worst leg 2.77 ± 0.30 mm; mocap parked tilt within ±0.10° of level. Catch error and pre-throw swing are REPORT-only (≈16 mm and ≈+2.92° expected, both unchanged) | TODO — criteria REVISED, see § Phase 4 |

## Implementation Phases

### Phase 0 — Ingest-surface enumeration and frame audit

**No code.** The deliverable is a written enumeration, because the failure mode
this plan closes *is* an incomplete enumeration. Per the repo's grep-before-refactor
rule: list every file and line, count occurrences, and after Phase 2 verify the
count of ad-hoc applications drops to zero.

Enumerate, for `trajectory_node` and `mpc_bridge_node`, every path by which a
platform pose reaches `planner.*`, and classify each as **external** (correct) or
**derived** (must not be corrected). The starting list, from
`trajectory_node.py:396-431`:

- external: `platform_pose_topic`, `catch/dynamic_target`,
  `trajectory/go_to_pose`, `trajectory/timed_target`, `_neutral_pose` (go_home)
- derived: `trajectory/hold`, `trajectory/reseed_from_measured`, the mode-exit and
  input-loss graceful stops, every follower replan, `_current_state()`,
  `_last_pose`

Two items Phase 0 must **resolve rather than assume**:

1. `trajectory/arm_catch` captures the reach-envelope centre from the current
   *commanded* pose, and the 80 mm envelope
   (`JB_TRAJ_CATCH_REACH_ENVELOPE_MM`) is then tested against corrected catch
   targets. Determine whether the centre and the targets are in the same frame
   today, and whether Phase 2 changes that. A tilt is not a translation so the
   centre should be unaffected, but confirm it rather than reason about it.
2. `set_limits` / the `chase` and `follower` paths: confirm no pose enters through
   them from outside.

Also record whether the **sim** consumes the correction anywhere
(`sim/`, `controller/`) — if not, say so explicitly, so a future reader does not
go looking.

**Gate:** the enumeration is reviewed by the operator before Phase 1 starts. If it
turns up a sixth external path nobody expected, that is the phase working.

### Phase 0 — Outcome

**Status: LANDED 2026-07-25 — commit `d67c3cd`; logbook
`logbook/2026-07-25-levelling-frame-enumeration.md`. OPERATOR-REVIEWED 2026-07-25 —
the hard gate on Phase 1 is CLEARED and both deferred decisions are resolved below.**
No production code and no tests changed — this phase's
deliverable is the enumeration below, plus the edits it forced in § Context,
§ C-LEVEL-1, Phase 2, Phase 3 and the risk register. Full suite
(`pytest tests/ -q`, run 2026-07-25): **3429 passed, 3 xfailed, 198 warnings in
1331.26 s (0:22:11)** — identical pass/xfail counts to the pre-phase baseline at
`e513036` (3429 passed, 3 xfailed in 1340.73 s), as it must be, since no imported
file changed.

**Resolved by the operator, 2026-07-25 (both were deferred here; neither is open):**

1. **The E6 decision — RESOLVED: correct all six paths in Phase 2.**
   `timed_target(hold_after=False)`'s neutral return target is corrected alongside
   `go_home`. It has no production client (zero `create_client` hits for
   `TimedTarget`), so this is a latent-path fix with no live behavioural
   consequence — which is the reason to do it now rather than when a client
   appears: a latent uncorrected path is exactly how the present bug got in, and
   the original five-row diagnosis missed this one. Phase 2 step 2 is binding on
   E5 **and** E6.
2. **Throw-aim semantics — RESOLVED: the ~12° ceiling is SOFT; a commanded
   12.78° is accepted.** The clamp stays where it is, on the ballistic aim
   (`toss_release.py:295-296`, gravity-referenced via `_from_vertical_deg`), and
   `MAX_TILT_DEG` is **not** reduced.
   The framing this supersedes: an earlier draft of this section read
   `tilt_geometry.py:58-62` as citing tracking degradation *alone*, and concluded
   "⇒ gravity-frame is right". The comment in fact names **two** grounds — *"tilt
   tracking **and** the small-angle aim model both degrade past ~12°"* — and
   Phase 2 makes those two quantities diverge by the correction magnitude. The
   aim-model ground is gravity-referenced and stays correctly capped at 12°; the
   *tracking* ground is about what the legs must physically produce, and that can
   now reach **12.78°**. So the single constant does double duty on two
   quantities that no longer coincide, and only the first of them is still
   clamped.
   Accepted deliberately, on these grounds: ~12° is a `bb` Rung-0
   *characterisation* rather than a mechanical limit ("leg headroom is fine well
   past this"), 12.78° is 6.5% past it, the excess is only reachable at maximum
   Tier-8b displacement (config-gated, currently capped at 70 mm), and leg
   feasibility is still measured on the **corrected** commanded pose by
   `feasibility.validate` regardless. Phase 2 records this in its logbook; if a
   tilt-tracking measurement at 11–13° later shows the ceiling is harder than the
   characterisation implies, the fix is to lower `MAX_TILT_DEG`, not to move the
   clamp.
3. **No bench check for this phase.** Phase 0 has no robot-actuating step, so
   `tests/hardware/session_anomaly_fixes.md` was deliberately **not** appended to —
   a section reading "nothing to run here" trains the operator to skim a runbook
   whose value is that every line is executable. The operator pre-brief the numbers
   below feed (resting platform visibly ~0.78° off its own frame; first `go_home`
   after `level` becomes a real ~2.77 mm move) is a **Phase 2** obligation and is
   written into Phase 2 step 6, where the code that makes it true lands.

**Headline: the enumeration found a sixth external path.** `trajectory/timed_target`
with `hold_after=False` passes the stored `_neutral_pose` **into the planner as the
return target** (`trajectory_node.py:2223` → `:1895` → `planner.build_timed(...,
neutral_pose=...)` → `planner.py:560`). It is a second, independent use of the
neutral constant that Phase 2 must correct alongside `go_home`; correcting only
`go_home` (the plan's step 2) would leave the reach-out-and-return one-shot parking
at plan-frame `rx = 0` while every other surface parks at gravity-level.

#### Method and grep audit

Symbol counts, `ros_ws/src/**` and the pure-Python trees only. **Excluded as build
artefacts:** `ros_ws/build/`, `ros_ws/install/`, and also the repo-root `build/` and
`install/` trees (both gitignored — `.gitignore:37-38`; the plan brief named only
the `ros_ws` pair, but the root pair holds a second stale copy of every module and
would double every count).

| Symbol | Live references | Files | Phase-2 target |
|---|---|---|---|
| `_apply_gravity_correction` — **applications** | **3** | `trajectory_node.py:1240`, `trajectory_node.py:1983`, `mpc_bridge_node.py:182` | **0** |
| `_apply_gravity_correction` — method definitions | 2 | `trajectory_node.py:1262`, `mpc_bridge_node.py:148` | 0 |
| `rotvec_to_rot_matrix([-tilt_x, -tilt_y, 0])` — the sign convention | 2 | `trajectory_node.py:1257-1258`, `mpc_bridge_node.py:143-144` | 0 |
| `_gravity_correction` attribute | 6 | `trajectory_node.py:311,1257,1265`; `mpc_bridge_node.py:75,144,154` | 2 **assignment sites** — one stored `R` per node. NOT a reference count: see the gate note in Phase 2 |
| **Sum of the rows above** (categories **overlap** — not a site count) | 13 | 2 files | — |
| **Distinct statements** carrying either symbol | **11** | `trajectory_node.py:311, 1240, 1257, 1262, 1265, 1983`; `mpc_bridge_node.py:75, 144, 148, 154, 182` | — |

The 13 is a sum of *categories*, not of sites: `trajectory_node.py:1257` is both
the sign-convention construction and the attribute write, and `mpc_bridge_node.py:144`
sits inside the `:143-144` sign-convention range **and** is counted again as an
attribute reference. Grep for the two symbols returns **11** distinct statements;
adding `mpc_bridge_node.py:143` (the sign-convention temporary `correction_rotvec`,
which carries neither symbol) gives **12 statements** for Phase 2 to remove or
redirect. Note also that the total can never "reach zero" as a whole — the
attribute row's Phase-2 target is **2**, one stored `R` per node. Phase 2's gate
is stated in those per-row terms (see § Phase 2), not against the headline sum.

**Dead copies, deliberately out of scope.** `ros_ws/src/jugglebot/jugglebot/archived/`
holds a **third and fourth** copy of the same transform — `control_loop.py` (7
references: `:306, :510, :511, :972, :1019, :1091, :1092`) and
`trajectory_manager.py` (8 references: `:114, :173, :175, :187, :188, :338, :372,
:387`). Neither has a `console_scripts` entry in `ros_ws/src/jugglebot/setup.py`
and neither is imported by any live module (verified by grep for `archived.`).
Phase 2 must **not** count these 15 references as call sites and must not touch
them; deleting archived code is a separate concern.

**`mpc_bridge_node` reaches `planner.*` zero times.** It imports no planner and no
motion-planning module — it is a pure translator that forwards ZMQ targets on :5558
to the dormant MPC process (`mpc_bridge_node.py:31-41, 188-191`), and it is dropped
from `jugglebot_launch.py` (`:147-150`) while retaining its `setup.py` entry point.
So its copy of the correction is not a second inconsistent ingest surface today; it
is a second *implementation* of a normative transform, which is the drift hazard
Phase 1 closes by moving the transform into `motion/levelling.py`.

#### Table A — every path by which a platform pose reaches `planner.*` (`trajectory_node`)

`E` = **external** (enters from outside the node: a service request, a wire target,
or the built-in neutral constant) ⇒ **must** be corrected exactly once.
`D` = **derived** (from measurement or from an existing plan) ⇒ **must not** be
corrected.

| # | Ingest surface / seed | Pose enters at | Planner entry | Class | Corrected today | Phase 2 |
|---|---|---|---|---|---|---|
| E1 | `platform_pose_topic` (SpaceMouse/GUI follower) | `_on_platform_pose` `:1237-1245` | `follower.follow` `:1300` → `planner.build_follow` `follower.py:227` / `build_graceful_stop` `follower.py:212` | E | **yes** (`:1240`) | route via helper |
| E2 | `catch/dynamic_target` | `_catch_target_from_msg` `:1973-1993` | `planner.build_catch` `:1944` | E | **yes** (`:1983`) | route via helper |
| E3 | `trajectory/go_to_pose` | `_pose_from_msg` `:1594-1601`, called `:1663` | `planner.build_move` `:1692` | E | **no** | correct |
| E4 | `trajectory/timed_target` | `_pose_from_msg` `:1594-1601`, called `:2184` | `planner.build_timed` `:1892` | E | **no** | correct |
| E5 | `trajectory/go_home` — `_neutral_pose` | `:200-201`, used `:1575` | `planner.build_return_to_neutral` `:1574` | E | **no** | correct **at use** |
| **E6** | **`trajectory/timed_target(hold_after=False)` — `_neutral_pose` as the RETURN target** | **`:200-201`, used `:2223`** | **`planner.build_timed(neutral_pose=…)` `:1895` → `planner.py:560`** | **E** | **no** | **correct at use — NEW** |
| D1 | telemetry seed (FK of `robot_state` `pos_estimate`) | `_seed_hold_from` `:890-897` | `planner.build_hold` `:902` | D | no | leave |
| D2 | `trajectory/hold` | — (seed only) | `planner.build_hold` `:1538` | D | no | leave |
| D3 | `trajectory/reseed_from_measured` (/recover step 1) | `_measured_pose` `:984-992` (FK of live encoder) | `build_follow` `:1042` / `build_graceful_stop` `:1050` / `build_follow` `:1055` | D | no | leave |
| D4 | guard-latch rising edge → same descent | `_install_guard_descent` `:1094-1102` | as D3 | D | no | leave |
| D5 | mode-exit / arm-latch-edge graceful stop | — (seed only) | `planner.build_graceful_stop` `:728` | D | no | leave |
| D6 | pending-stop retry (post-publish) | — (seed only) | `planner.build_graceful_stop` `:649` | D | no | leave |
| D7 | follower input-loss stop | — (seed only) | `planner.build_graceful_stop` `:1360` | D | no | leave |
| D8 | every follower replan (the chased pose) | `follower._clamp_to_workspace` → `chase.pose` | `planner.build_follow` `follower.py:227` | D | no | leave |
| D9 | `_current_state()` — the sampled active plan | `:1440-1447` | seed argument of every entry above | D | no | leave |

**Direct plan installs that bypass `planner` entirely** (all `D`, all sourced from
`_last_pose`, the last already-gated emitted frame): the emitter step-bound backstop
`:571`, the A3 escalation backstop `:658`, `_freeze_in_place` `:1021`, and the
stop⊕descent concatenation `:1060-1062` (both halves individually gated). These are
deliberate gate exemptions documented at each site; they carry no external pose and
Phase 2 must not touch them.

**The `_neutral_pose` read inventory is exhaustive: one definition (`:200`) and
five reads (`:344`, `:1446`, `:1575`, `:1585`, `:2223`).** Two of the reads are
targets and are rows E5 and E6 above. The remaining three are **not** targets and
must stay uncorrected — each deserves a line so a future reader neither "fixes"
them nor has to re-derive whether the enumeration simply missed one:

- `_current_state()` `:1446` returns `self._neutral_pose.copy()` when no plan is
  installed. Reachable only pre-seed; the emitter is gated on `_seeded` (`:544`) and
  every planning service requires `_seeded`. The one caller that does not check
  `_seeded` is `_svc_arm_catch` `:2124`, and it reads `[:3]` only — position, which
  the correction never touches (see the resolution below). `_command_within_step_of_
  measured` `:1006` also reads it, but returns early when `_latest_pos_rev is None`
  and is only called while streaming (⇒ seeded).
- `_last_pose` `:344` is initialised to the neutral copy and overwritten by the first
  emitted frame (`:577`) or the first seed (`:916`). Same pre-seed-only reachability.
- `:1585` reads `self._neutral_pose[2]` to format the `go_home` response string
  (`returning to neutral (0,0,{z:.0f})`). It is neither a seed nor a target — it is
  a **log format**, it reads the position component, and the correction never
  touches position. It needs no change under either placement choice.

**External twists are `E`-adjacent and are deliberately *not* corrected.** Two
surfaces carry an externally-supplied arrival *velocity* alongside the pose:
`TimedTarget.velocity_mm_s` (`:2192-2193`, into `planner.build_timed`'s `twist`
argument at `:1893`) and `DynamicTargetCommand.target_vel` (`:1991-1992`, dropped
by `build_catch`, which forces translational arrival velocity to zero). Both stay
uncorrected, by C-LEVEL-1's last clause: the correction is a **bias on the
commanded rotation**, not a re-expression of the platform frame, so rotating a
linear velocity would be a category error. Recorded explicitly because "apply the
correction to every external ingest" reads, to a Phase-2 implementer chasing frame
consistency, like a licence to write `twist[:3] = R_gravity @ twist[:3]`. Today
that would be invisible — `build_catch` discards the twist and `timed_target` has
no production client (below) — so it would land green and stay, and the first
`timed_target` client would inherit a commanded arrival direction 0.78° off the
pose it was computed against. A rotated velocity is among the hardest things to
see in leg-space telemetry. The same clause covers the shaper's
`twist[0:2] -= shift_d; twist[3:5] += tilt_d` (`shaping.py:362`), which is a
separate, already-gated superposition.

#### Table B — `mpc_bridge_node`

| # | Ingest surface | Pose enters at | Destination | Class | Corrected today |
|---|---|---|---|---|---|
| B1 | `platform_pose_topic` | `_on_platform_pose` `:177-185` | ZMQ :5558 → dormant MPC process (**no** `planner.*`) | E | **yes** (`:182`) |

No other pose surface exists in that node. It is therefore not internally
inconsistent, as the Context section says — the defect there is duplication, not
divergence.

#### Table C — how the **correction itself** arrives (and how it is lost)

Tables A and B enumerate how *poses* arrive. The enumeration is incomplete without
the mirror question — **does this node have a correction at all?** — because Phase 2
widens the consequence of "no" from the catch pre-tilt alone to *every parked pose*.

| Leg | Where | QoS / lifetime |
|---|---|---|
| publisher | `orchestrator_node.py:121-122` `create_publisher(Float64MultiArray, 'gravity_offset', 10)` | default ⇒ **VOLATILE**, not transient-local |
| publish 1 | `:329-333` — first IDLE after boot, gated on `ctx.levelling_complete` **and** `not self._startup_offset_sent` (`:130`), which is then latched True | **once per orchestrator boot** |
| publish 2 | `:488-494` `level_send_correction`, inside the LEVELLING routine only | once per `level` |
| subscriber | `trajectory_node.py:403-404`, default QoS; `mpc_bridge_node.py:86-87` | volatile |
| storage | `trajectory_node.py:311` `self._gravity_correction = np.eye(3)`; sole writer `_on_gravity_offset` `:1247-1258` | **per-process, in-memory, no re-request path** |
| observability | `TrajectoryStatus` (`_publish_status` `:2250-2257`) carries streaming/mode/plan_kind/plan_time_remaining_s/seq/max_emit_gap_ms/last_rejection | **the correction is not published anywhere** |

**The hazard, stated plainly.** "Levelled on the Teensy" and "a correction is
resident in *this* `trajectory_node` process" are a **third pair of meanings of
level**, produced by the same class of reasoning this plan exists to close. If
`trajectory_node` alone restarts after a `level` — a crash, or precisely the
`colcon build` + relaunch this plan's own § Deployment mandates for Phases 1–3 —
its correction is identity again, nothing republishes (publish 1 is latched, publish
2 only fires inside LEVELLING), and nothing shows it. `RobotState.levelling_complete`
still reads True, because it is a **Teensy-persisted per-boot flag** and says nothing
about any ROS process's memory.

**This contradicts Phase 3 as currently written**, and Phase 3 must resolve it:
step 4 already says *gate on "a correction is loaded", **not** "levelling ran this
session"* — which is right — but step 3 wires the observation to
`RobotState.levelling_complete` via `orchestrator_node.py:165`, which observes the
Teensy, not the loaded correction. As wired, `REJECTED_NOT_LEVELLED` would **pass**
in exactly the state it exists to refuse, giving false assurance where today there
is merely no gate. Candidate closes, for Phase 3 to pick between and record:
(a) publish `/gravity_offset` TRANSIENT_LOCAL so a late-joining subscriber gets the
last correction; (b) add the loaded correction (or a `gravity_correction_loaded`
bool) to `TrajectoryStatus` and gate on **that** — the node that actually applies
it; (c) both. (b) is the one that satisfies step 4 literally. Note the launch has
no `respawn`, so this needs a manual partial restart rather than happening on its
own — which is why it is a Phase-3 design constraint and not a live defect today.

#### Resolved question 1 — the `arm_catch` reach-envelope frame

**The centre and the targets are in the same frame today, by construction, and
Phase 2 does not change that.** Both sides of the envelope test are
**position-only 3-vectors**, and the correction is applied **only** to the
rotational component:

- centre: `self._catch_envelope_center = np.asarray(self._current_state()[0][:3],
  …).copy()` (`:2123-2124`);
- test: `excursion = norm(target[:3] - self._catch_envelope_center)` against
  `JB_TRAJ_CATCH_REACH_ENVELOPE_MM` (`:2053-2060`);
- `target[:3]` comes verbatim from `msg.target_pos` (`:1984-1987`); only
  `target[3:6]` passes through `_apply_gravity_correction` (`:1983`).

Confirmed empirically rather than by argument (probe `/tmp/probe_levelling_frame_p0.py`,
run 2026-07-25, production `pose_to_leg_lengths` / `rotvec_to_rot_matrix`): applying
the 2026-07-25 session offset to the neutral pose leaves the position delta at
**exactly 0.000e+00 mm**. Phase 2 rewrites the same rotational component only, so
the invariant holds after it.

Second-order effect worth recording, not a blocker: the **physical** cup opening
does swing when the platform tilts, by `arm · cup_axis_xy` — measured
**1.2977 mm** for the 0.78° correction (`cup_lateral_shift_mm`, probe above)
against a 35 mm cup radius (`GEOM_HAND_RADIUS_MM`) and an 80 mm envelope. So after
Phase 2, Tier 8b's swing-compensated pre-tilt position (`release.pretilt_pose_stow`,
computed for the *requested* tilt) is off by ~1.3 mm from the swing the *commanded*
tilt actually produces. That is 1.6 % of the envelope and 3.7 % of the cup radius —
below the current 16 mm catch error the plan is closing.

> **The frame this rests on, stated explicitly (2026-07-26).** A finalize-pass
> review argued the opposite — that the compensation is *world*-referenced and so
> becomes exact after Phase 2. It does not, and the reason is worth writing down
> before someone "fixes" this in the wrong direction. `cup_lateral_shift_mm` is
> the cup opening's **xy offset from the platform centroid**, and
> `pretilt_pose_stow` commands that centroid in STOW coordinates.
> `stow_to_global_mm` is a pure z translation (`toss_release.py:83-84`) — there is
> **no rotation** between STOW, "global" and the base frame. So both the shift and
> the position it corrects live in the *base* frame, and the shift must be
> computed from the tilt the platform physically holds **in that frame**, i.e. the
> post-correction commanded tilt. `_cup_axis_xy`'s docstring word "world" means
> *the fixed, non-rotating frame* (as against the platform body frame), not
> *gravity-aligned*.
>
> Net effect of Phase 2 on the throw, both terms: the release **direction**
> becomes gravity-correct (worth ~0.78°, i.e. ~42 mm over a 0.8 s flight at
> 3.93 m/s) at the cost of ~1.3 mm of base-frame release-**position** error. A
> large win with a small newly-introduced second-order cost. Do **not** subtract
> the correction's contribution from `pretilt_pose_stow` to "restore consistency":
> that would re-introduce the 0.78° aim error this plan removed.
>
> The conclusion depends on the offset being the *base*-vs-gravity rotation and on
> the cup being rigid on the platform. If either is false it flips — say so rather
> than assuming.

**Relevant to
`plans/active/catch-reach-degenerate-overshoot.md` Phase 0** only as a note: that
plan's near-degenerate reach is the same `build_catch` path, and after this plan the
commanded rotvec excursion on the toss reach goes to ~0, which is exactly the
degenerate case it studies — its Phase 0 should re-read its reproduction against a
post-Phase-2 tree rather than assume the pre-fix excursion still exists.

#### Resolved question 2 — `set_limits`, `chase`, `follower`

**No pose enters through any of them from outside.**

- `_svc_set_limits` (`:1773-1801`) reads exactly three scalar fields —
  `leg_vel_limit_mmps`, `leg_acc_limit_mmps2`, `leg_jerk_limit_mmps3` (`:1781-1787`)
  — each clamped to its YAML ceiling. No pose field exists on `SetTrajectoryLimits`.
- `chase.chase_alpha(state0, clamped, limits, geom, horizon_s)` (`follower.py:202`)
  is a pure function of the seed and the already-clamped target; `chase.py` contains
  no pose constant (grepped for `neutral`, `170`, `np.array([0`: no hits).
- `follower._clamp_to_workspace` (`follower.py:266-296`) constructs its clamped pose
  by bisecting the `current → target` ray — both endpoints already in the corrected
  frame — so the clamp output is derived, never a new external pose. `TargetFollower`
  holds only `_last_target` (`follower.py:151, 234`), the previously chased pose.

#### `sim/` and `controller/` do **not** consume this correction — explicitly

Grepped `sim/` and `controller/` for `gravity_correction`, `gravity_offset`,
`levelling`, `leveling` (case-insensitive). The **only** hits are
`controller/teensy_link/rpc_args.py:246-256`, which encode the boolean
`levelling_complete` **flag** into the Platform-Teensy persisted `RobotState` — not
the correction. There is no gravity-levelling transform anywhere in `sim/` or
`controller/`. **A future reader should not go looking:** the correction exists
only in the two ROS nodes named above (plus the two dead `archived/` copies), and
the MuJoCo simulation has no levelling concept at all.

#### Risk-register check — `motor_guard`, stow/park, and the ACTIVATE reference

The register asks whether anything trusts an **uncorrected** neutral. Findings:

- **`motor_guard` does not.** It consumes the commanded `pose_6dof` off the :5557
  wire (`motion/motor_guard.py:473, 518, 549`) for exactly two things: a condition
  number (`compute_condition_number(pos_cart, rot_mat, geom)`) and a leg-extension
  workspace check (`check_workspace_limits(ext, cond, …)`, `:552-560`). It has no
  notion of neutral, level, or a reference orientation, so a 0.78° rotation is just
  another commanded pose to it — and one the feasibility gate has already validated
  before the frame ships.
- **The stow reference does not.** DEACTIVATE descends every leg to `stow_rev = 0.0`
  (`teensy_bridge_node.py:199-200`), a leg-space firmware target with no relation to
  `_neutral_pose`. The hand park-band constants (`JB_OP_HAND_RETRACT_REV`,
  `_HAND_NEAR_TARGET_REV`) are hand-axis positions and are untouched by this plan.
- **The ACTIVATE reference is where the visible change lands, and it is benign.**
  ACTIVATE TRAP_TRAJs each leg to `JB_OP_ACTIVATE_POSITION_REVS`
  (`teensy_bridge_node.py:3234`; `controller/teensy_link/activate.py:19,32`), a
  codegen'd constant documented as *"the IK of the active pose"*
  (`Teensy_code_canbridge/leg_activate.h:6`) — i.e. the IK of the **uncorrected**
  neutral. After Phase 2, ACTIVATE still lands at the uncorrected neutral and
  `trajectory_node` still seeds a hold from **measured** telemetry (`:890-897`), so
  the seed stays honest; the first `go_home` after `level` then produces a small real
  motion where today it is a genuine no-op. **Nothing compares the two references**,
  so no guard, monitor, or band is invalidated. Quantified (probe, run 2026-07-25):
  the corrected neutral moves the legs by at most **2.7736 mm = 0.03908 rev**
  (per-leg deltas `[-1.42, +2.74, +2.77, -0.98, -1.35, -1.75]` mm) — 13 % of the pump
  step gate (`JB_OP_MAX_POSITION_STEP_REV = 0.3` rev) as a **whole-move** excursion
  spread over the `go_home` profile, 3.9 % of the firmware 1.0 rev MAX_DEVIATION
  band, and **322×** the 8.607e-3 mm leg encoder dead-band (so the excursion is
  comfortably observable in `/leg_setpoint_echo`, which is what makes Phase 4's
  ±0.05° FK criterion measurable at all).
- **Three artefacts a future reader could misread as a contract on `go_home`.** All
  three pin *"commanded level ⇒ activate revs"*, all three remain **true and green**
  after Phase 2 because each constructs the level pose directly and none goes near
  `trajectory_node`, and none is a statement about `go_home` — which after Phase 2
  targets the *corrected* neutral whenever a correction is loaded. Phase 1's normative
  document should name all three and say exactly that:
  - `sim/tools/verify_motor_commands.py:7,58,92` — PASSes on *"At Active pose, motor
    commands MATCH `JB_OP_ACTIVATE_POSITION_REVS`"*. It is a **standalone offline IK
    check** (`motor_rev = IK_extension_mm × mm_to_rev`), *not* a drive through the MPC
    path; it never imports the planner or the node.
  - `tests/motion/test_trajectory_emitter.py:41-48`
    (`test_activate_revs_fk_roundtrip_is_neutral`) — FK of `JB_OP_ACTIVATE_POSITION_REVS`
    gives `pos ≈ [0,0,170]`, `rotvec ≈ 0`. A statement about the **codegen'd constant**,
    permanently true.
  - `tests/motion/test_trajectory_emitter.py:53-61` — asserts
    `mr ≈ JB_OP_ACTIVATE_POSITION_REVS` for a `HoldPlan(NEUTRAL)`, under the comment
    *"Hold at neutral ⇒ u0 == the activate revs (bumpless arm at the active pose)"*.
    That comment is the misreadable one: it reads as a **live system invariant** but is
    a statement about the **IK/emitter layer** for a directly-constructed `NEUTRAL`. At
    the *system* level, "commanded home == activate revs" becomes conditional on the
    loaded correction after Phase 2, and the test stays green regardless — so nothing
    fails loudly to correct the misreading.

#### Phase-2 regression surface (what the correction newly reaches)

- **Tier 8b's throw aim.** `reload_coordinator_node._position_platform_for_toss`
  (`:1471-1492`) commands a deliberately **non-identity** tilt via
  `_tilt_quaternion(release.tilt_rx, release.tilt_ry)` (`:1521-1531`) on
  `go_to_pose`. After Phase 2 the commanded tilt becomes `R_gravity @ R_tilt`, so the
  throw aims relative to **gravity** instead of relative to the plan frame — which is
  what a ballistic aim should do, and is the same reasoning that justifies the whole
  plan. Consequences to state in Phase 2's logbook:
  - the `MAX_TILT_DEG = 12.0` loud clamp (`tilt_geometry.py:62`, gated in
    `toss_release.py:295-296`) is evaluated on the **required** aim *before* the
    request, so the commanded plan-frame tilt can now exceed 12° by up to the
    correction magnitude while the **physical** tilt vs gravity equals the clamped
    value. Leg feasibility is unaffected — the gate still measures the commanded
    pose — but the clamp's units change meaning slightly, and that belongs in the
    contract document, not in a code comment.
  - the composition is **not commutative**: for a `[0.15, -0.08, 0]` target the two
    orders differ by **1.268e-3 rad (0.0727°)**, entirely in `rz` (probe above). The
    request-side tests that assert `rotvec[2] == approx(0.0, abs=1e-9)`
    (`tests/ros/test_toss_coordinator.py:1482`, in the block `:1478-1483`) check the
    **wire**, not the node, so
    they stay green — and they are exactly what proves the correction is not being
    applied twice.
  - the POSITIONING mocap arrival cross-check (`_toss_platform_target_mm`,
    `reload_coordinator_node.py:406, 1219-1220`) is **position-only**, so it is
    unaffected.
- **The lean shaper superposes tilt additively in rotvec space**
  (`shaping.py:361`, `pose[3:5] += tilt`) while the correction composes in SO(3).
  After Phase 2 the two are no longer exactly composable. Measured difference for a
  5° lean on top of the 0.78° correction — **the axis matters, so it is pinned**: a
  lean about **`ry`** gives **5.931e-4 rad (0.03398°)** and a lean about `rx` gives
  only 5.267e-5 rad (0.00302°), 11× smaller. `ry` is the worst case (the correction
  is dominated by `rx`, so the cross product is largest for an orthogonal lean) and
  is the one to pin; a Phase-1 implementer who reproduces this against `rx` will not
  match. Either way it is second order, and the `LEAN_TILT_CAP_DEG = 5.0` clamp is
  applied to the **added** lean magnitude only (`shaping.py:353-356`), so the
  correction does not eat into the cap.
- **`trajectory/timed_target` has no production client today.** Stated as what was
  actually checked, so a reader can re-run it: **no node anywhere creates a
  `TimedTarget` service client** (`grep -rn 'create_client' --include='*.py' .` has
  zero `TimedTarget` hits). The symbol grep does return four non-test source hits
  besides `trajectory_node` — `jugglebot_interfaces/action/Toss.action:27`,
  `motion/trajectory/toss_release.py:69` and `:118`, `toss_sequencer.py:433` — and
  every one is a **comment** citing the `TimedTarget.pose` z-convention, plus the
  interface declaration at `jugglebot_interfaces/CMakeLists.txt:61`. Correcting
  E4/E6 is therefore a latent-path fix with no live behavioural consequence — which
  is precisely why it must be done now rather than when a client appears.
- **No test pins the resting platform at `rx = 0`.** The only orientation assertions
  in `tests/ros/test_trajectory_node.py` are the follower-correction test
  (`:886-893`, which asserts the correction IS applied) and identity-quaternion
  *inputs*. `test_timed_target_hold_after_false_returns_to_neutral` (`:1426-1433`)
  asserts `p_end ≈ node._neutral_pose` with no correction installed, so it stays
  green under E6 (identity correction ⇒ no-op). **No existing test needs weakening
  for Phase 2** — the enumeration found none, and if Phase 2 discovers otherwise
  that is a STOP, not a licence to relax an assertion.
- **Prose that Phase 2 falsifies and must reword in the same commit.** None of these
  fails a test, which is exactly why they are dangerous — a stale claim that no test
  guards survives indefinitely and gets *reasoned from*:
  - `planner.py:135-142`, `build_return_to_neutral`'s docstring, verbatim:
    *"``neutral_pose`` is the active pose ``(0, 0, 170, 0, 0, 0)``. If the seed is
    already at ``neutral_pose`` and at rest the segment is degenerate (no motion), so
    ``trajectory/go_home`` from the held active pose is a genuine no-op."* After
    Phase 2 that is false whenever a correction is loaded. **This is the exact
    inference this plan's own Context uses at § What the data shows** to rule
    `go_home` out as the motion source (*"the POSITIONING move the operator assumed
    was the tilt is a genuine no-op"*). Leaving it would re-enable the same wrong
    turn the 2026-07-25 session took: a future investigator seeing unexplained motion
    around a toss reads the docstring, eliminates the actual source, and hunts
    elsewhere. Reword to *"the neutral pose the node supplies; with a levelling
    correction loaded this is the **corrected** active pose, so `go_home` is a no-op
    only when the correction is identity."* Sweep `_svc_go_home`'s docstring too.
  - `tests/hardware/session_phase1_hold.md:168` (`# no-op from the held active pose`)
    and `:174` (PASS criterion *"a genuine no-op: hold pose ≈ neutral"*), and
    `tests/hardware/mvp_bench_runbook.md:372` (*"the teardown `go_home` installed as
    `move_seq=12` with realized peaks 0.0 (a genuine no-op from neutral)"*). Every
    sitting begins with a manual `level`, so post-Phase-2 the teardown `go_home`
    always moves — worst leg **2.7736 mm = 0.03908 rev** over the 2.0 s profile
    (`go_home_duration_s`). The operator would either record a spurious FAIL against
    a written PASS criterion or read visible teardown motion as a new fault. Annotate
    both with the expected magnitude.

#### What the Phase-2 bypass test must assert to be **structural**

A behavioural test per surface proves only that today's surfaces are corrected; it
cannot fail when a **seventh** surface is added tomorrow, which is the failure this
plan exists to close. The bypass test is structural only if it fires on the
*omission*. Four assertions, in dependency order:

1. **Planner-entry manifest equality (the load-bearing one).** `ast.parse` the
   sources (read the files — do **not** import them, so the test needs no ROS
   mocking) and collect every call to `planner.build_*` plus `self._follower.follow`.

   **Key each entry on `(module, enclosing function, planner callee, {argument
   name → source text} over the POSE-BEARING arguments only)`.** Assert that set
   equals a frozen manifest carrying each entry's `E`/`D` classification — Table A
   above — **and assert the parsed file set itself** equals the manifest's file set.
   A new planner entry, a new pose argument on an existing entry, or a planner
   entry in a fourth module all fail until the author adds a row and *declares* the
   classification.

   Two properties of this key are load-bearing, and **the obvious simpler key
   fails on the very defect this phase found**:
   - *Pose-bearing arguments, not just "the target".* Keying on the enclosing
     function plus the source text of **the** target argument cannot see E6 at all.
     `planner.build_timed(self._current_state(), target, twist, lead, self._limits,
     self._geom, hold_after=hold_after, neutral_pose=neutral)` (`:1892-1895`) is
     **one `ast.Call` node** carrying **two** independent external poses — E4's
     `target` and E6's `neutral`. Enclosing function (`_plan_and_install_timed`) and
     target source text (`target`) are byte-identical with or without
     `neutral_pose=neutral`, so that key collapses the two rows Table A deliberately
     splits. This is not hypothetical: `build_catch` takes `neutral_pose` too
     (`planner.py:785`, required at `:834`, consumed at `:910`) while
     `_plan_and_install_catch` hard-codes `hold_after=True` (`:1948`). A one-line
     future change — `hold_after=False, neutral_pose=self._neutral_pose` — adds a
     **seventh** external ingest inside the same enclosing function with the same
     target argument. Under the collapsed key every assertion here still passes and
     the post-catch return parks at plan-frame `rx = 0` while everything else parks
     at gravity-level: the identical defect, silently reintroduced, with the
     contract's own guard reporting all-clear. Enumerate the pose-bearing arguments
     per planner function so the manifest can be checked: `build_timed` →
     `target_pose, neutral_pose`; `build_catch` → `catch_pose, neutral_pose`;
     `build_return_to_neutral` → `neutral_pose`; `build_move` / `build_follow` →
     `target_pose`; `build_hold` / `build_graceful_stop` → none.
   - *Module, and the file set as data.* The manifest spans **three** files, not
     one: Table A's E1 and D8 rows give planner entries in
     `motion/trajectory/follower.py` (`build_graceful_stop :212`, `build_follow
     :227`), and Table B's surface is in `mpc_bridge_node.py`. An AST pass over
     `trajectory_node.py` alone cannot be equal to the manifest it freezes, and
     would not fire on a planner entry added inside `follower.py` — which already
     owns two — nor on `_svc_go_home`'s `build_return_to_neutral` being refactored
     down into a motion helper. Freezing the *file set* is what makes a fourth
     module fail rather than pass silently.

   **Scope note:** the manifest's scope is these three files, not "every caller of
   `planner`". `sim/toss_gate.py:502-523` calls `build_move` / `build_hold` /
   `build_catch` directly and the AST pass will never see it — correctly, because
   `sim/` has no gravity-offset concept at all (§ *`sim/` and `controller/` do not
   consume this correction*), so there is no correction to omit and no failure mode.
   Phase 1's document should say this, so a reader does not mistake the guard's
   scope for the planner's.
2. **Helper call-site set equality.** Same AST pass, collecting callers of the
   `motion/levelling` helper; assert the set of enclosing functions equals exactly
   `{E1…E6}`'s handlers. Catches the reverse error — a correction applied inside a
   *derived* path (the mirror bug).
3. **Application-count interception, per `E`/`D` ROW — not per surface.** Monkeypatch
   the helper with a counting wrapper, install a non-identity correction, drive each
   surface once, and assert the count matches the **number of rows that surface
   carries**: 1 for a single-pose surface, and **2** for `timed_target(hold_after=
   False)`, which carries E4 *and* E6. Assert **exactly 0** for every `D` row. (A
   literal "exactly 1 per surface" is unimplementable for precisely the reason
   assertion 1's key must widen — one surface, two external poses.) A final-rotvec
   equality check cannot distinguish "corrected once" from "corrected, re-corrected,
   and accidentally un-corrected"; a count can. The `D` half must include the FK-seed
   round trip: seed from leg lengths that correspond to a corrected hold and assert
   the seeded hold's rotvec equals the FK rotvec exactly (this is C-LEVEL-1's second
   half made executable).
4. **Assert on the installed plan, never on the message or the log.** For each `E`
   surface, with correction `R ≠ I` and an identity-orientation request, assert
   `plan.state_at(t)[0][3:6] == rot_matrix_to_rotvec(R)` to ~1e-12. The placement
   decision's entire justification is *plan == emitted == gated*; asserting on
   `_follower_target` or on a log string would pass even if the correction were
   dropped between ingest and install.

Plus the regression the plan already specifies (step 5), made precise: with `R ≠ I`,
`go_to_pose(identity)` then `catch/dynamic_target(identity)` at the same xyz must
yield `max_t |rx(t) − rx(0)| < 1e-9` rad **sampled across the plan**, not merely at
its endpoints — the 2026-07-25 signature was a *mid-plan* excursion to +2.32°, so an
endpoint-only assertion would have passed while the platform tilted.

**Manifest maintenance rule:** key the manifest on function names and AST nodes,
never on line numbers. A line-numbered manifest becomes a maintenance tax and gets
deleted the first time an unrelated edit shifts the file — which would silently
remove the whole guard.

#### Deployment

Phase 0 changes one markdown file: **no build, no relaunch, nothing to flash.**
Phases 1–3 touch `ros_ws/src/**` and therefore need
`colcon build --packages-select jugglebot` **plus a relaunch** (the launch runs the
installed copy).

### Phase 1 — Contract document, helper, unit tests

1. Write the normative section (C-LEVEL-1 above, with the measured evidence and
   the ingest/derived split). State the placement decision *and its reason* — a
   future reader under shipping pressure will otherwise "simplify" it into the
   emitter.
2. Add the helper to `motion/` — something like
   `levelling.apply_gravity_correction(rotvec, correction) -> rotvec` plus
   `levelling.correction_from_offset(tilt_x, tilt_y) -> R`, moving the
   `rotvec([-tilt_x, -tilt_y, 0])` sign convention and the
   `R_corrected = R_gravity @ R_target` composition out of both nodes. Keep the
   existing behaviour bit-identical — this phase must not change any number.
3. Unit tests: identity correction is a no-op; the sign convention counter-tilts
   (a `+tilt_x` offset produces a `−rx` command); round-trip
   `correction_from_offset` → `apply` → `rot_matrix_to_rotvec` reproduces
   `[-tilt_x, -tilt_y, 0]` for an identity target; composition with a non-trivial
   target orientation is order-correct (`R_gravity @ R_target`, not the reverse —
   pin this, it is not commutative and the reversed form is a plausible typo).
4. Pin the worked example from this investigation as a regression:
   offset `[0.013592347421588673, 0.001207157476773584]` ⇒ corrected identity
   target `(-0.77878414°, -0.06916503°, 0°)`.

**Files:** `motion/levelling.py` (new), `tests/motion/test_levelling.py` (new),
one normative `*.md`.
**Gate:** `pytest tests/ -q` green. Numbers unchanged anywhere.
**Note:** `*.md` changes mean this commit needs `/audit --unstaged` first.

#### Phase 1 — Outcome (2026-07-26)

**DONE, landed with Phase 2 in commit `aea7b49`.** Logbook:
`logbook/2026-07-26-levelling-frame-contract.md`.

`ros_ws/src/jugglebot/jugglebot/motion/levelling.py` is the single
implementation: `correction_from_offset` owns the `[-tilt_x, -tilt_y, 0]` sign
convention, `apply_gravity_correction`/`correct_pose` own the `R_gravity @
R_target` composition, `identity_correction` returns a *fresh* `np.eye(3)` (a
shared module constant would be one in-place write from corrupting every node's
frame in the process). Contract **C-LEVEL-1** lives at
`ros_ws/docs/levelling_frame.md` — deliberately in `ros_ws/docs/` rather than
`controller/REFERENCE_LAYER_CONTRACT.md`, because no `controller/` code applies
this correction and a normative invariant with zero enforcement in its own
subtree invites a reader to "apply" it there, i.e. to introduce the
double-application mirror bug in a second subsystem.

The extraction is bit-identical, verified **numerically** against the verbatim
pre-change inline code from `git HEAD` rather than by reading the diff: 6 offsets
× 6 rotvecs including rotvec ≈ π and 1e-9 offsets, `max |R_old − R_new| =
0.000e+00`, `max |rv_old − rv_new| = 0.000e+00`. Two reviewers reproduced it
independently.

**Test triple:** `pytest tests/ -q`, run 2026-07-26 on this Jetson:
**3484 passed, 3 xfailed in 1371.51s (0:22:51)** — +55 against the 3429/3
baseline at `9f35a66`, and the xfail count is unchanged (no test weakened).

**Deferred to the operator:** nothing from Phase 1 alone; see Phase 2.

### Phase 2 — Migrate every ingest path

1. Route `_pose_from_msg` through the helper, so `go_to_pose` and `timed_target`
   are corrected.
2. Correct `_neutral_pose` at use (`go_home`) — **not** at construction. It is a
   stored constant read in several places; correcting the stored value would
   double-apply the moment the correction changes mid-session, and the offset
   *does* arrive mid-session (the operator levels after launch).
   **`go_home` is not the only use.** Phase 0 found a second one: `timed_target`
   with `hold_after=False` passes `_neutral_pose` as the planner's RETURN target
   (`trajectory_node.py:2223`, row E6 of § Phase 0 — Outcome). Both use sites must
   be corrected. The read inventory is exhaustive at **five** reads; the three that
   are **not** targets must stay uncorrected — `_current_state()`'s pre-seed
   fallback `:1446` and the `_last_pose` initialiser `:344` are **seeds** (see the
   Phase-0 Outcome for why each is unreachable post-seed), and `:1585` is a
   **response-string format** reading the position component. Neither kind changes.
3. Replace the two existing ad-hoc applications (`:1240`, `:1983`) with the
   helper. Delete `mpc_bridge_node`'s copy and point it at the helper.
4. **The bypass test.** This is the phase's real deliverable, and it is **not**
   optional-with-a-fallback. Phase 0 settled that: a behavioural test per surface
   proves only that *today's* surfaces are corrected and could never have caught
   E6, so the structural half is mandatory and the behavioural half is a
   complement, not a substitute. Both halves, and the four assertions the
   structural half needs — including the manifest key, which is itself load-bearing
   — are specified in § Phase 0 — Outcome, *What the Phase-2 bypass test must
   assert to be structural*. Implement from there. The behavioural complement:
   install a non-identity correction, submit an identity-orientation request on
   each external surface, assert the resulting **plan**'s target rotvec equals the
   corrected value; plus the negative half — drive each *derived* surface and
   assert it is **not** re-corrected.
5. Regression test for the bug this plan fixes: with a non-identity correction,
   a `go_to_pose(identity)` followed by a `catch/dynamic_target(identity)` at the
   same xyz must produce a **degenerate** catch plan (no commanded tilt). Assert
   on the plan, not on a log string — and sample **across** the plan, not only at
   its endpoints: the 2026-07-25 signature was a *mid-plan* excursion to +2.32°,
   which an endpoint-only assertion would have passed through.

6. **Reword the prose Phase 2 falsifies, in the same commit** — `planner.py`'s
   `build_return_to_neutral` docstring and `_svc_go_home`'s, plus the two hardware
   runbooks that document the teardown `go_home` as a genuine no-op
   (`tests/hardware/session_phase1_hold.md:168,174`;
   `tests/hardware/mvp_bench_runbook.md:372`). None of them fails a test, which is
   why they must be swept deliberately — see § Phase 0 — Outcome, *Prose that
   Phase 2 falsifies*. Carry the operator pre-brief (with the measured magnitudes)
   into `tests/hardware/session_anomaly_fixes.md` at the same time.

Mind the ordering hazard in (2): the correction can change while a plan is live.
Decide and document what happens to an in-flight plan when a new
`gravity_offset` arrives — the honest default is "the live plan keeps its frame;
the next install picks up the new correction", because re-framing a live plan
would step the commanded pose. Say so in the contract. Phase 0 supplies a second,
stronger argument for that default: because the correction is baked into the
plan's endpoint at **build** time, an emitter-side correction would instead step
`u0` by the full offset delta on the very next 25 ms knot — a command
discontinuity on the wire, not merely an ungated plan.

**Files:** `trajectory_node.py`, `mpc_bridge_node.py`, `tests/ros/test_trajectory_node.py`,
possibly a new `tests/ros/test_levelling_frame.py`; plus, for step 6,
`motion/trajectory/planner.py` (docstring only),
`tests/hardware/session_phase1_hold.md`, `tests/hardware/mvp_bench_runbook.md`,
`tests/hardware/session_anomaly_fixes.md`.
**Gate:** `pytest tests/ -q` green; grep confirms **zero** remaining ad-hoc
applications, method definitions and sign-convention constructions (the first three
rows of the Phase-0 grep table, target 0/0/0), and the `_gravity_correction`
attribute row reaches its target of **2 assignment sites** — one stored `R` per
node. The headline sum of 13 is a category sum and never reaches zero; do not gate
on it.

> **Gate wording corrected 2026-07-26 (the target was never a reference count).**
> The row is met exactly as *intent*: `trajectory_node.py:315`/`:1273` and
> `mpc_bridge_node.py:81`/`:149` are the only assignments, so there is one stored
> `R` per node. The literal `self._gravity_correction` **reference** count is
> **9** — the four assignments plus five explicit pass-throughs, one per
> enumerated ingest (trajectory_node `:1251` E1, `:1295` E5/E6, `:1652` E3/E4,
> `:2041` E2; mpc_bridge_node `:184` B1). A literal 2 is unreachable under *any*
> design that keeps the shared implementation a free function in a ROS-free
> module: the caller has to hand it the stored `R`. The only way to hit a literal
> 2 is a per-node bound method that owns the matrix — which is precisely the
> duplicate this phase deletes and which C-LEVEL-1 forbids. Gate on the assignment
> count; a reader who gates on the reference count will "fix" it backwards.
**Deployment:** `ros_ws` change ⇒ `colcon build --packages-select jugglebot` +
**relaunch** (the launch runs the installed copy).

#### Phase 2 — Outcome (2026-07-26)

**DONE, commit `aea7b49`**. Logbook:
`logbook/2026-07-26-levelling-frame-contract.md`.

All six external ingest surfaces plus `mpc_bridge_node`'s B1 now route through
`motion/levelling.py`; the verbatim second copy of the transform is deleted. The
grep gate is met per row: applications **0**, method definitions **0**,
sign-convention constructions outside `motion/levelling.py` **0**, stored `R`
**2 assignment sites per the corrected wording above**. E5/E6 go through a new
`_corrected_neutral_pose()` that corrects the stored neutral **at use** —
correcting it at construction would double-apply the moment a new
`/gravity_offset` arrives, and the operator levels *after* launch.

The real deliverable is the structural bypass guard: two AST manifests (planner
entries keyed on **pose-bearing arguments**, plural; `levelling` call sites) with
**discovered** file sets, plus an attribute-access-only import guard and a frozen
writer set for `self._follower_target`. Mutation-verified against six seeded
regressions, each reverted, including the three no behavioural test can reach —
`hold_after=False, neutral_pose=…` added to an existing `_plan_and_install_catch`
call (the "seventh ingest inside an existing call"), a new ingest that writes
`self._follower_target` without any tracked call, and a `from
jugglebot.motion.levelling import correct_pose` that silently un-instruments both
the AST manifests and the behavioural counting fixture.

**Test triple:** `pytest tests/ -q`, run 2026-07-26 on this Jetson:
**3484 passed, 3 xfailed in 1371.51s (0:22:51)**. Baseline at `9f35a66` was
3429 passed, 3 xfailed; the +55 is exactly the cases this phase adds (37 in
`tests/ros/test_levelling_frame.py`, 12 in `tests/motion/test_levelling.py`, 6 in
`tests/motion/test_levelling_probe.py`). xfail count unchanged — no test was
weakened, skipped or deleted, and `git status --porcelain` shows zero modified
pre-existing `tests/**/*.py`.

**Deferred to the operator — two items, both escalated with numbers, neither
fixed:**

1. **`planner.build_catch` aims its tilt-through-seat residual off the plan-frame
   tilt.** With a correction loaded a gravity-level catch settles **0.3008° off
   gravity-level at ball contact** (closed form −1.078408° rx / −0.095775° ry;
   the reference bag recorded −1.0784 / −0.0958; the committed probe measures the
   settle plateau at −1.0775°, err 0.0009°). That is the 16 mm catch error. It is
   **pre-existing** — `catch/dynamic_target` was already corrected before
   2026-07-25 — and the fix is a genuine design fork with different ball-seating
   consequences per branch: (a) pass the gravity-referenced receive tilt to
   `build_catch` separately, (b) pre-subtract the overshoot, (c) suppress the
   through-seat when the gravity-referenced tilt is ~0, (d) accept it. Not taken,
   because it changes commanded motion at ball contact on every catch including
   the shipping reload path. Pinned as
   `test_catch_through_seat_aims_off_the_gravity_referenced_receive_tilt`.

   > **SUPERSEDED, 2026-07-26 — branch (a) WAS taken.** C-CATCH-1
   > (`ros_ws/docs/catch_arrival_contract.md`) passes the gravity-referenced
   > receive tilt to `build_catch` as its own argument. The settle now lands on
   > the target (`−1.078408° → −0.778784°` rx), the 0.3008° residual at ball
   > contact goes to `0.0000°`, and with it the 16.5 mm throw-direction error.
   > The "changes commanded motion at ball contact on the shipping reload path"
   > objection was *correct and was paid, deliberately and with numbers*: the
   > reload seat aim rotates **4.0997°** and its settle moves `+0.021086°` rx /
   > `+0.004297°` ry — bounded, pre-briefed in
   > `tests/hardware/session_anomaly_fixes.md` § Section CCATCH, and gated
   > there. The reload seat RATE at contact is unchanged (`0.070000 rad/s`).
2. **The same reach shaping keeps the visible pre-throw swing** — see the second
   CORRECTED note in § Context. `≈+2.92°` at a 3.70 s lead, post-fix, and that is
   *correct behaviour*. The operator pre-brief in
   `tests/hardware/session_anomaly_fixes.md` § Section LVL says so explicitly, in
   item 3, because a sitting scored against "the tilt should be gone" would fail a
   working fix.

   > **SUPERSEDED, 2026-07-26 — the swing is gone.** C-CATCH-1 removed the
   > through-seat for a gravity-level catch entirely, so the self-toss reach is
   > FLAT (`2.32° → 0.0000°` on all five reference reaches). § Section LVL item 3
   > is a PRE-fix reference; § Section CCATCH supersedes it and says so in its
   > preamble. Briefing an operator to expect `+2.92°` would now fail a working
   > machine.

**Deployment for the operator:** `cd ~/Desktop/Jugglebot/ros_ws && colcon build
--packages-select jugglebot && source install/setup.bash`, then **relaunch**
`jugglebot_launch.py`. No firmware flash, no interface build, no config
regeneration. `/gravity_offset` is VOLATILE with one latched publish per
orchestrator boot, so the relaunch itself reverts the correction to identity while
`RobotState.levelling_complete` still reads True — a manual `level` after the
relaunch is **mandatory** (Phase 3 owns the structural closure).

### Phase 3 — `REJECTED_NOT_LEVELLED` gate in toss CHECKING

Justified by geometry, not process: un-levelled, the launch is 0.78° off gravity
⇒ 43 mm of lateral drift against a ~35 mm cup. The catch cannot succeed, so the
throw should refuse rather than put a ball on the floor. This is the same class of
loud-early reject as `REJECTED_HAND_NOT_PARKED`.

1. `TossObservations` gains `platform_levelled: bool`.
2. `toss_sequencer._step_checking` rejects `REJECTED_NOT_LEVELLED`. Order it with
   the other live-observation gates — after `mocap_fresh` / `streaming` (a stale
   graph makes the flag unknowable) and before the hand-evidence chain.
3. `reload_coordinator_node` feeds it. **Do not wire it to
   `RobotState.levelling_complete` alone** — Phase 0's Table C shows why that
   observation cannot satisfy step 4. `levelling_complete` is a *Teensy-persisted
   per-boot flag*: it reads True regardless of whether any ROS process holds a
   correction, so a `trajectory_node` that restarted after `level` (a crash, or the
   `colcon build` + relaunch this plan itself mandates) would **pass** the gate with
   an identity correction — the precise state the gate exists to refuse, now with
   false assurance attached. Resolve this in Phase 3 and record the choice:
   transient-local QoS on `/gravity_offset`, and/or publish the loaded correction on
   `TrajectoryStatus` and gate on that. The coordinator does **not** currently
   subscribe to `robot_state`; whatever signal is chosen needs the same staleness
   discipline as the other observations.
4. Gate on "a correction is **loaded, in the node that applies it**", **not**
   "levelling ran this session" — the persisted auto-push at
   `orchestrator_node.py:329-333` (first IDLE entry after boot, conditional on
   `levelling_complete`, latched by `_startup_offset_sent`) is a legitimate way to
   satisfy it. Do not gate on the correction being non-identity: a genuinely level
   machine has a zero offset and must not be refused.
5. Add the outcome to `Toss.action`'s outcome documentation and to
   `tests/hardware/toss_trace_recorder.py`'s reject-code table (it has a
   code → human-readable-cause map around line 144).

Operational fact worth putting in the phase notes: `levelling_complete` is "since
last bootup" and the operator reboots the can-bridge Teensy before every session,
so it is False at every launch and the auto-push never fires first. In practice
every session genuinely needs a manual `level`, and this gate makes that a loud
refusal instead of a wasted throw.

**Coordination:** another session is actively editing `reload_coordinator_node.py`
and `Toss.action`. Run `git fetch && git status -sb` before starting and again
before pushing; do not rebase or push over divergence.

**Files:** `toss_sequencer.py`, `reload_coordinator_node.py`,
`tests/ros/test_toss_coordinator.py`, `tests/hardware/toss_trace_recorder.py`,
`Toss.action` (comment only). Depending on which close step 3 picks:
`orchestrator_node.py` (transient-local QoS) and/or `trajectory_node.py` +
`jugglebot_interfaces/msg/TrajectoryStatus.msg` (expose the loaded correction) —
note the latter is an **interface** change, so it needs `colcon build` of
`jugglebot_interfaces` as well as `jugglebot`.
**Gate:** `pytest tests/ -q` green.

#### Phase 3 — Outcome (2026-07-26)

**DONE 2026-07-26 — `e36d60d`.** Operator runbook:
`tests/hardware/session_anomaly_fixes.md` § Section LVLGATE (checks LG-0…LG-5;
**LG-5 is not optional** — it is the only check that exercises the cross-process
ordering, which every test behind this phase is blind to by construction).

**Step 3's fork, resolved: candidate (b) alone.** `TrajectoryStatus` gains
`bool gravity_correction_loaded`, set by `trajectory_node._on_gravity_offset`
and published at 5 Hz by `_publish_status`; the coordinator stamps every
`trajectory/status` arrival and feeds `platform_levelled = status_fresh AND
loaded` into `TossObservations`. The gate therefore observes **the node that
applies the correction**, which is the only reading that cannot lie about it.
Recorded as contract **C-LEVEL-1.O** in `ros_ws/docs/levelling_frame.md`, under
§ *The correction can be silently absent — so it is now observable, and the toss
refuses* (that heading was previously *Known hazard — the correction can be
silently absent*; this phase renamed it, because the hazard is no longer
unclosed). The delivery hazard itself is unchanged — what closed is its
silence.

**Candidate (a), transient-local QoS on `/gravity_offset`, was considered and
NOT taken.** Three failure modes decided it, in order of weight:

1. **It does not help the case that motivates the hazard.** A transient-local
   latch lives in the **publisher**. The `colcon build` + relaunch this plan's
   own § Deployment mandates restarts `orchestrator_node` too, so there is
   nothing latched to redeliver. It would only help a partial restart of
   `trajectory_node` alone while the orchestrator survives — and the launch has
   no `respawn`, so that is a manual act.
2. **It creates a silent DDS-level failure of exactly the class this contract
   exists to close.** A TRANSIENT_LOCAL *subscriber* is incompatible with a
   VOLATILE *publisher* (requested durability exceeds offered), so the
   connection is simply never made — no error, no warning, no message. Any
   hand-run `ros2 topic pub /gravity_offset …` without
   `--qos-durability transient_local`, or a second publisher added later, would
   be silently ignored.
3. **It closes nothing on its own.** It reduces the frequency of the blind state
   without making it observable, so a gate built on it would still be guessing.
   (b) is correct regardless of *how* the correction went missing — never
   published, discovery race, malformed message, node restart, orchestrator
   never reaching IDLE.

**Step 4 satisfied literally.** The gate asks "is a correction loaded", not "did
`level` run this session" — the orchestrator's persisted auto-push
(`orchestrator_node.py`, first IDLE after boot) is a legitimate way to satisfy
it, pinned by
`test_the_persisted_startup_push_alone_satisfies_the_gate`, which drives the
real producer's `_publish_status` output into the real consumer's callback. And
it is **not** gated on the correction being non-identity:
`test_zero_offset_counts_as_loaded_even_though_the_correction_is_identity`
pins that a genuinely level machine (zero measured tilt ⇒ identity correction)
is not refused.

**The restart case is proven at both levels** —
`test_a_restarted_node_reports_no_correction_although_the_teensy_flag_persists`
(the applier) and `test_trajectory_node_restart_flips_the_gate_to_refuse`
(end to end through `_execute_toss`: correction loaded ⇒ serviceable, replacement
process ⇒ `REJECTED_NOT_LEVELLED`).

**The freshness window is probe-sized, not copied.** The sibling windows
(`_MOCAP_STALE_S`, `_HAND_STATE_STALE_S`, `_STATUS_STALE_S`) are all 0.5 s and
are sized for 100–160 Hz sources; `trajectory/status` is **5 Hz**, so 0.5 s is
2.5 periods. Measured inter-arrival over 420 s of the two 2026-07-25 reference
bags (`_15-17-48`, `_15-22-50`; one-off probe, run 2026-07-26, independently
re-measured at finalize): **median 200.0 ms, p99 210.3 / 204.0 ms, max
508.5 ms** — one gap already past 0.5 s and **four** past 0.3 s (two per bag:
384.5 / 508.5 ms and 334.5 / 426.2 ms), so a copied 0.5 s would mint occasional
spurious refusals on a healthy machine. `_TRAJ_STATUS_STALE_S = 1.0` is five periods and ~2× the worst observed
gap. The same probe measured the siblings for contrast: mocap max 463.8 ms at
159 Hz, hand 24.3 ms at 100 Hz.

**Deliberately NOT done: `streaming`'s semantics are unchanged.** It remains a
sticky last-value with no freshness stamp, because the **reload** FSM consumes
it too and changing when a reload refuses is a safety-relevant behaviour change
outside this phase. The visible consequence is recorded in the code and in the
reject-code map: a `trajectory_node` that dies outright surfaces as
`REJECTED_NOT_LEVELLED` (its status stops, the flag expires) rather than as
`REJECTED_NOT_STREAMING`.

**Deferred, with the reason stated (finalize adjudication, 2026-07-26).** A
reviewer proposed closing that gap *here*, with a toss-only `traj_status_fresh`
observation and its own reject code, on the correct observation that doing so
need not touch `streaming` at all. Verified and agreed as a real diagnostic
weakness — a dead `trajectory_node` now names the wrong subsystem, where before
this phase it surfaced later as `REJECTED_POSITION(NO_RESPONSE)`, which named
the right one. Deferred anyway, for a reason that is not scope-protection:
**"is `trajectory_node` alive" would then have two independent representations
in one node** — `streaming` (which the reload FSM consumes, un-expiring) and a
parallel toss-only stamp — and the two would answer differently for the same
fault. That is precisely the one-enforcement-point violation this contract
pattern exists to prevent, and it is how a later correct fix to `streaming`
silently leaves a stale duplicate behind. The single-enforcement-point fix is
one freshness stamp on `streaming` itself, which changes **when a reload
refuses** — a safety-relevant behaviour change, and its own small phase. Until
then the mitigation is documentary and is in place at three levels: the operator
pre-brief item 4 (with the two commands that distinguish the causes),
`REJECT_WIRE_MAP`'s entry naming both causes, and the note above. The
consequential half — no motion, no arming, no ball dropped — is identical under
either code.

**Deployment:** interface change ⇒ `colcon build --packages-select
jugglebot_interfaces jugglebot` **+ relaunch**. Building only `jugglebot` makes
`trajectory_node` **exit**, not merely go quiet: `_publish_status` raises
`AttributeError` on its first 0.2 s tick, rclpy re-raises timer exceptions out
of `spin()` (`rclpy/executors.py` `SingleThreadedExecutor.spin_once`), and
`main()` catches only `KeyboardInterrupt` — so the process tears down ~200 ms
after launch, there is no 40 Hz emitter, and `activate` fails at the A2 arm long
before any toss is sent. LG-0 catches it in three seconds. No firmware flash, no
config regeneration. Verified on this Jetson 2026-07-26: both packages build, the
generated message carries the field, and assigning an unknown field to a real
message raises `AttributeError`.

**Gate:** `pytest tests/ -q`, run 2026-07-26 on this Jetson in the project venv
— **3569 passed, 3 xfailed, 198 warnings in 1399.52s (0:23:19)**, exit 0.
Baseline at `b9fd45e` was 3543 passed, 3 xfailed in 1376.13s: **+26 passed**,
accounted for exactly by the 26 new cases (8 sequencer / 7 trajectory_node / 11
coordinator), **xfail unchanged at 3**.

**Deferred operator handoff.** Software-complete; nothing here is validated on
hardware yet. `tests/hardware/session_anomaly_fixes.md` § Section LVLGATE,
checks **LG-0…LG-5**, is the handoff: LG-0 pre-flight (installed-copy greps
`1 / 3 / 2` plus `ros2 node list | grep trajectory_node`), LG-1 the headline
refusal before `level`, LG-2 the negative half, **LG-3 the decision this phase
turns on** (`levelling_complete: true` AND `gravity_correction_loaded: false`
AND `REJECTED_NOT_LEVELLED`, all three together), LG-4 no spurious refusals
across the sitting, LG-5 the cross-process ordering. Phase 4's own criteria are
unchanged by this phase.

### Phase 4 — Hardware validation (operator-run)

The operator runs all robot-actuating commands; the implementer prepares the exact
commands and the PASS/ABORT criteria and verifies read-only.

Sequence: launch → `level` → activate → trajectory → load a ball → one toss, with
`tests/hardware/toss_trace_recorder.py record` running and the launch rosbag
enabled.

**The criteria below were REVISED by Phase 2's implementation** — two of the
original four were unreachable for a reason orthogonal to this plan (§ Context,
the CORRECTED note). The executable version, with commands, lives in
`tests/hardware/session_anomaly_fixes.md` § Section LVL; this is the summary.

**PASS:**
- The **park plateau** — commanded `rx`/`ry` (FK of `/leg_setpoint_echo`) where
  the platform rests between goals — within **±0.05°** of `(−tilt_x, −tilt_y)`
  on both axes. Verdict command: `tools/probes/levelling_tilt_bag_check.py
  --offset <TILT_X> <TILT_Y>`. This replaces *"flat to ±0.05° across the whole
  goal"*, which cannot be met while `build_catch` aims its through-seat residual
  off the plan-frame tilt. A park at ≈0° is the pre-fix frame; a park at
  ≈**−1.5576°** is double application (the mirror bug).
- The first `go_home` after `level` moves the worst leg **2.77 ± 0.30 mm**
  (`0.0391 ± 0.0042` rev), smoothly, with no pump reject and no guard latch.
- Mocap `Platform` total tilt while parked is within **±0.10° of LEVEL** — it must
  *not* track the commanded −0.78°, because that is the whole point. A physical
  0.78° tilt while parked means the correction's sign is inverted ⇒ stop.
- A toss commanded before `level` returns `REJECTED_NOT_LEVELLED` — **Phase 3
  only; do not score it against Phases 1–2.**

**REPORT, do not gate:**
- Tracker catch error. **Expect ≈16 mm, unchanged** — the original `< 10 mm`
  target assumed this plan closed the 0.30° through-seat residual, which it does
  not. Judge the catch by eye as well; tracker verdicts still read MISSED on real
  catches (see the Phase-7 reload arc).
- Peak commanded `rx` above the park. **Never a gate — the earlier "at or above
  +3.099° ⇒ ABORT" line is retired.** Post-fix this equals the platform's
  physical peak tilt against gravity and equals `0.789132° × (catch lead in s)`
  exactly, so any fixed threshold on it is really a threshold on the catch lead:
  the +3.099° line fires on a healthy system at any lead ≥ 3.93 s and sat only
  0.18° from firing at the 3.70 s lead the reference session ran. Like-for-like
  in the gravity frame: pre-fix **+3.0992°**, healthy post-fix **+2.9198°** at
  that lead. Cross-check the probe's `implied lead` against the toss's actual
  catch lead (±10 %) instead; the pre-fix bag reads 2.94 s against a true 3.70 s.

**ABORT:** any platform motion beyond the null pre-position other than the
expected none; any tilt exceeding 1°; E-STOP.

Do **not** expect the hand dip to be gone — that is
`plans/active/hand-command-continuity.md`. If both plans have landed, validate
them in one sitting but score them separately.

## Testing Plan

| Level | What |
|---|---|
| unit | `tests/motion/test_levelling.py` — sign, order, round-trip, pinned worked example |
| unit | negative half: derived-pose surfaces are not re-corrected |
| structural | bypass test — external-ingest set == helper call-site set |
| integration (mocked ROS) | non-identity correction ⇒ `go_to_pose(identity)` then `catch/dynamic_target(identity)` at the same xyz ⇒ the catch reach's **net tilt displacement is zero**. NOT "a degenerate catch plan": `build_catch`'s specified arrival twist keeps the reach non-degenerate whatever the frame, so a degeneracy assertion would have had to be weakened later |
| integration (mocked ROS) | the same reach **sampled across the plan**, not only at its endpoints — the excursion is the arrival twist alone, `0.789132°` per second of lead, peaking at `s = 2/3` |
| instrument | the verdict probe reads BOTH shapes: post-fix PASS, pre-fix FAIL, ACTIVATE-contaminated FAIL-with-note (`tests/motion/test_levelling_probe.py`, `--self-check`) |
| integration (mocked ROS) | `REJECTED_NOT_LEVELLED` fires; and does **not** fire when only the persisted push has run |
| hardware | Phase 4 |

Mocked-ROS tests are blind to cross-process choreography. The frame question is
single-node, so unit + integration coverage is genuinely sufficient here — but the
`REJECTED_NOT_LEVELLED` observation crosses `orchestrator → reload_coordinator`,
so its wiring wants a real-ordering check (the existing
`toss_trace_recorder` capture is the cheapest one).

## Risk register

| Risk | Mitigation |
|---|---|
| Double-application via a derived pose (the mirror bug) | contract's second half + the negative-half tests; correct `_neutral_pose` at use, not at construction |
| Correction changes mid-plan, stepping the commanded pose | decide + document the in-flight rule in Phase 2; default is "live plan keeps its frame" |
| Composition order silently reversed during the move to `motion/` | Phase 1 pins `R_gravity @ R_target` with a non-commutative test case |
| A sixth ingest path appears later | **A sixth already existed (E6) and Phase 0 found it.** The bypass test is the whole point of Phase 2 — and it must be *structural*, because a per-surface behavioural test could never have caught E6. **The manifest key is itself load-bearing**: keyed on the enclosing function + the target argument it is blind to the whole E6 class (one call, two external poses) and to entries outside `trajectory_node.py`. See § *What the Phase-2 bypass test must assert* for the widened key and the live `build_catch` instance of the blind spot |
| `levelling_complete` semantics drift (Teensy-persisted, per-boot) | Phase 3 gates on "correction loaded" and documents the per-boot reality |
| **The correction is never delivered, or is lost on a partial restart** | **Found by Phase 0 (Table C); CLOSED by Phase 3 (2026-07-26) — made loud, not made impossible.** `/gravity_offset` is still VOLATILE with one latched publish per orchestrator boot and `trajectory_node`'s copy is still in-memory with no re-request path; what changed is that the copy is now **observable**. `TrajectoryStatus.gravity_correction_loaded` carries the applier's own answer at 5 Hz, and `toss_sequencer` refuses the throw (`REJECTED_NOT_LEVELLED`) unless that answer is True on a status < 1.0 s old. Transient-local QoS was **rejected** — its latch lives in the publisher, so the whole-graph relaunch that motivates the hazard gains nothing, and a TRANSIENT_LOCAL subscriber silently refuses to connect to any VOLATILE publisher. Contract **C-LEVEL-1.O**; § Phase 3 — Outcome. The operational requirement is unchanged: **`level` after every launch or relaunch** — forgetting now costs a refusal, not a ball on the floor |
| Correcting `go_home` shifts the stow/park reference the guard trusts | **RESOLVED (Phase 0): nothing trusts an uncorrected neutral.** `motor_guard` reads the commanded pose only for a condition number + leg-extension workspace check; DEACTIVATE stows to leg-space `stow_rev = 0.0`; ACTIVATE TRAP_TRAJs to `JB_OP_ACTIVATE_POSITION_REVS` and nothing compares it to `go_home`'s target. Excursion measured at 2.7736 mm / 0.03908 rev. Details + the one misreadable artefact (`sim/tools/verify_motor_commands.py`) in § Phase 0 — Outcome |

## Notes for collaborators

- Applying the correction to `go_home` and `go_to_pose` means the platform will
  physically sit level at all times after levelling, which is a **visible
  behaviour change** at the machine: the resting platform will look ~0.78° tilted
  relative to its frame and level relative to gravity. That is correct. Tell the
  operator before the first post-fix launch so it is not read as a new fault.
  Phase 0 measured the size of it so the operator knows what to expect: total tilt
  **0.78185°**, per-leg extension deltas
  `[-1.42, +2.74, +2.77, -0.98, -1.35, -1.75] mm` (worst leg **2.7736 mm =
  0.03908 rev**). The first `go_home` after `level` therefore produces a small real
  motion where today it is a genuine no-op — expect movement, not a fault.
- The **other** thing to tell the operator, and the one that would otherwise read
  as "the fix did nothing": the pre-throw tilt swing they reported is **still
  there** after this plan, at roughly its original size. See the second CORRECTED
  note in § Context and pre-brief item 3 in
  `tests/hardware/session_anomaly_fixes.md` § Section LVL.
- The residual amplification question — why the excursion overshot to +2.32° and
  settled at 1.385× the single correction rather than ramping cleanly to
  −0.7788° — was **not** in scope here, and this plan's correctness does not
  depend on it. **Phase 2's implementation answered the settle half of it
  anyway** (§ Context, the CORRECTED note): the 1.385× is exactly
  `(tmag + 0.5·rate·decay)/tmag = 1.38473`, i.e. `build_catch`'s tilt-through-seat
  overshoot, and the settle is `−1.078408°` in closed form against the bag's
  `−1.0784°`. The **swing** half (+2.32°) is the quintic reach acquiring that
  arrival rate, and the finalize pass established that it is **not** this plan's
  either (§ Context, the second CORRECTED note): it is `(16/81)·rate·|tdir_x|·lead`
  = `0.789132°` per second of catch lead, present with or without a frame
  mismatch, and independent of the correction's magnitude. This plan takes it from
  `+3.0992°` to `+2.9198°` against gravity at the reference session's 3.70 s lead
  — 0.18° of 2.9°. The reload's real 11.08° BB pre-tilt was verified clean
  (monotonic, no overshoot) in the same session.
  **`plans/active/catch-reach-degenerate-overshoot.md` now owns the swing**, and
  should re-read its reproduction against two facts: the arrival twist
  `build_catch` injects is a *specified* part of the reach's boundary conditions,
  not an artefact; and its own "degenerate reach" case is what a post-Phase-2 toss
  reach actually is (zero net tilt displacement, non-zero arrival twist), so the
  excursion it studies survives this plan intact rather than disappearing with it.
- Every number quoted in § Phase 0 — Outcome is re-derivable from the text without
  the probe: apply the session offset `[0.013592347421588673,
  0.001207157476773584]` rad through `motion.ik_solver.rotvec_to_rot_matrix` /
  `rot_matrix_to_rotvec` (the composition is `R_gravity @ R_target`) and push the
  result through `motion.ik_solver.pose_to_leg_lengths` with
  `motion.geometry.StewartGeometry`. The one-off probe that produced them lived at
  `/tmp/probe_levelling_frame_p0.py` and is **deliberately not committed** (single
  use, per the reusable-probe rule) — so treat the recipe above, not the path, as
  the reproduction. Phase 1 turns the load-bearing one (the pinned worked example)
  into a permanent unit test, which is the right home for it.
