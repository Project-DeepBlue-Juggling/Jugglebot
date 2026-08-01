# Design proposal — SpaceMouse follower: knot-cadence protection and command-divergence bounding

**Status**: RESOLVED 2026-07-10, ARCHIVED 2026-08-01 — superseded by the implemented **chase-clamp follower
rework** (commit `73dba2b` on `mvp-trajectory-bringup`); **S3 re-flown 2026-07-10: PASS**
("worked perfectly" — operator). See the Resolution section immediately below; the
original proposal text is preserved unchanged underneath as the historical record.
**Trigger**: the 2026-07-09 Phase-3 bench session (S3) locked the platform into a
latched `MAX_DEVIATION` E-STOP plus a permanently-dead setpoint stream.
**Evidence**: rosbag `~/Desktop/rosbags/2026-07-09_13-49-17` + the launch-terminal log.
**Companion**: P0 (arming refuses on a latched guard fault; `ERR_BUS_DOWN`
disambiguation; guard-fault edge logging; `/link_status` recorded) is already landed.

---

## RESOLUTION (2026-07-10)

An independent forensic re-investigation (three parallel analysis agents over both
rosbags, the launch logs, and Jetson-measured cost profiling) confirmed parts of this
proposal, corrected others, and found one root cause it missed entirely. The fix that
shipped is Harrison's **cap-and-chase** concept made rigorous, plus this document's 4.1.
Landed as commit `73dba2b` (8 files, +1826/−176; full suite 2304 passed / 5 skipped /
1 xfailed, `pytest tests/ -q`, 2026-07-10).

### Disposition of the proposals

| § | Proposal | Outcome |
|---|---|---|
| 4.1 | Publish-first emitter + replan budget | **LANDED** as proposed. The knot is sampled and published before any planning; the follower replan and pending-stop retry run post-publish for the next tick. Stop builds are per-tick iteration-budgeted. |
| 4.2 | Clamp-not-reject in the follower | **LANDED, strengthened** into the chase clamp: per-tick, a frozen-Jacobian per-sample α-interval intersection computes the largest feasible progress toward the stick from the current seed (vel/acc/jerk/step-rate × 6 legs, safety 0.85) over an energy+distance-scaled horizon — then ONE plan, ONE validate (bounded stretch fallback). The follower is total; the naive fixed-horizon variant of this idea was itself refuted in design review (it capped tracking at ~25 % of the vel limit). `motion/trajectory/chase.py`. |
| 4.3 | Divergence guard freezing plan-time | **REJECTED** (operator decision, upheld by evidence: never stall or pause plan time). The two divergence drivers are removed at source by 4.1 + the chase clamp; the boundary-park lockup is fixed by the margin clamp (below); residual pathological seeds get a level-triggered escalation latch → graceful stop → HoldPlan-at-last-emitted-pose backstop, loudly. A *fault-reactive* stop consuming P0's `/link_status` remains a sensible follow-up; a time-freeze does not. |
| 4.4 | Pump absorbing-state escalation | **NOT LANDED — demoted.** Forensics contradicted §2.3's causal claim: the pump rejections began **6.3 s AFTER** the guard freeze and self-cleared after ~2 s. The absorbing state is real but was neither the cause of the stall nor of the lockup. It remains a latent availability bug worth fixing on its own schedule (`controller/teensy_link/` was deliberately untouched — a parallel session owns uncommitted work there). |
| 4.5 | Diagnostics | **PARTIALLY LANDED**: `chase_alpha`, `consecutive_rejects`, `escalation_stop`, `follow_block_max_ms` now published via the existing KeyValue pattern. The rolling-window realized-peaks and stale predicted-peaks items remain open (tracked in the runbook's open items). |
| — | *(missed by this proposal)* | **The actual permanent-lockup bug**: `follower._clamp_to_workspace` bisected saturated targets onto the EXACT hard stroke bound — the same surface the gate rejects at — so the last valid plan parked the commanded state on the leg-0 boundary and every subsequent replan (any target, including reachable neutral) rejected `WORKSPACE` forever. **Fixed**: the clamp now targets the envelope tightened by `CLAMP_MARGIN_MM = 0.5` (clamp-space strictly inside gate-space), with defined in-band-seed behaviour and a keep-last that can no longer absorb. |

### Corrections to the analysis above (kept for the record)

- **§2.3 ("this is what turned a recoverable stutter into a dead robot")** — wrong by
  event ordering: the freeze occurred at 14:00:30.4 (guard output-disable, ODrives never
  faulted); the first pump rejection is at 14:00:36.75. The robot was already dead 6 s
  before the pump entered the absorbing state, and the pump recovered ~2 s later.
- **§4.2's envelope arithmetic (d ≤ 10.3 mm)** — wrong in mechanism and value. The true
  z step-envelope at the defaults is ≈ 6 mm and **jerk**-bound (not vel-bound; the 1.82
  factor was a misapplied vel ratio). More importantly, from a REST seed even
  full-deflection targets were *accepted* via duration stretch (~2.4 s plans); the
  reject trigger was **moving-seed reversals** (the first TOO_FAST fired with the stick
  at neutral, target 4.2 mm away). Clamping was still the right fix — for the seed's
  energy, not the target's distance.
- **§2.1 descent asymmetry** — killed: the bag contains ascent flight with the same
  stutter signature.
- **§1 timeline omission** — the SpaceMouse disconnected mid-incident at 14:00:10.6
  (reconnected 14:00:22.7), injecting a go-to-ACTIVE target into the rejecting follower.
- **Reject-path cost** — measured on the Jetson: p50 41 / p99 58 ms (worse than §5.2's
  concern); even accept-with-stretch ran 14.5–48 ms. Emit gaps predated the first
  rejection, which the reject-path-only causality could not explain.

### Validation of the fix

- **Replay of the recorded S3 stick stream** (642.8 s, 25.7k ticks, steady 25 ms,
  default limits; harness reproduces the real incident against the old follower to
  < 0.1 s on the first reject and reproduces the 0.000 mm boundary park): new follower —
  **0 rejects (old: 96), 0 reject streaks (old: 38 ticks), 0 boundary parking (old: 11
  ticks at 0.000 mm; new minimum installed terminal margin exactly 0.500 mm), leg peaks
  92/344 vs limits 100/400, follow() p99 14.9 ms (old 21.0, max 116), tracking error
  identical (−0.02 %), 0 escalations.**
- **Hardware re-fly 2026-07-10: S3 PASS** — smooth throughout ("worked perfectly").
  The mid-flight *unplug* sub-test was not performed: the SpaceMouse connects over
  Bluetooth (no dongle to pull), so a clean physical disconnect isn't easily produced;
  the input-loss path is covered by unit tests and the replay's end-of-stream stop, and
  SpaceMouse control is a test/fun mode, not a production dependency. Accepted.
- Both directions (§5.3's ascent/descent question) flew in the re-fly without asymmetry.

### Open questions (§7) — dispositions

1. Freeze-tau vs re-seed: **neither** — the chase clamp dissolved the dilemma.
2. +25 ms input latency: **accepted and flown**; imperceptible, as predicted.
3. Disarm-vs-stow for the pump: **moot for S3** (4.4 demoted); decide when 4.4 lands.
4. `MAX_LEAD_REV × pos_gain` (6 rev/s) vs ODrive `vel_limit` (4 rev/s): **still open** —
   a firmware-config question outside this arc; carry it to the can-hub backlog.

---

## 1. What actually happened (established, not hypothesised)

Reconstructed from 70 169 `/robot_state` samples, 3 517 `/trajectory/status` samples,
and the operator's launch log.

1. **13:59:50.8** — armed (`mpc_active set to 1`), mode `SPACEMOUSE`, session limits at
   the Phase-1 defaults (100 mm/s, 400 mm/s², 8000 mm/s³).
2. The operator commanded **descending** vertical motion with large puck deflections.
   `z_mult_mm = 140` maps full `-z` deflection to a target of z = 30 mm (from the
   neutral 170 mm).
3. **14:00:03** — `TOO_FAST: follower could not find a feasible horizon in 6 iterations
   (last failure LIMIT_ACC)`. The follower holds the last valid plan.
4. Emit gaps climb from the nominal 25 ms to **62.5 → 77.6 → 145.85 ms** (the running
   max in each 200 ms `/trajectory/status` window). The emitter itself never stalls —
   `seq` keeps advancing at ~40 Hz, worst 200 ms window 19.9 Hz.
5. **~14:00:31** — all six legs freeze at `[1.433, 1.450, 1.456, 1.505, 1.497, 1.455]`
   rev and stay there, to four decimal places, for the remaining 32 s of the bag.
   ODrives remain in `CLOSED_LOOP` (state 8) with `active_errors = 0` and
   `disarm_reason = 0` on **every leg, every sample**, drawing ~2.2 A holding current.
   Bus voltage flat at 45.2 V; peak `iq` 8.3 A against a 10 A limit.
   ⇒ Not an ODrive fault. The **guard suppressed leg output**.
6. **14:00:36.75** — `Setpoint REJECTED (not sent): leg 0 step 0.3312 rev > 0.3 limit
   (cmd=0.9840, prev=0.6528)`. Leg 0's *measured* position at that instant is
   **1.4330 rev**, i.e. the last-accepted command (`0.6528`) sat **0.7802 rev** below
   the encoder — well past `MAX_DEVIATION_REV = 0.5`. This is the direct evidence that
   the E-STOP at step 5 was `MAX_DEVIATION`.
7. **14:00:37.75** — `step 1.1505 rev … (cmd=1.8033, prev=0.6528)`. `prev` is
   **unchanged**. The pump has entered an absorbing state and will never accept again.
8. Everything downstream is refused: unplugging the SpaceMouse publishes an ACTIVE-pose
   target the guard ignores; Ctrl-C's shutdown stow fails; `DEACTIVATE` returns
   `ERR_BUS_DOWN` (the guard-E-STOP branch of `deactivate_allowed()`).

Peak **measured** leg velocity during the flight was **3.555 rev/s = 250.6 mm/s —
2.51× the 100 mm/s session limit** the feasibility gate is supposed to enforce.

## 2. Root cause

> **The follower's commanded trajectory is open-loop in both time and state, and the
> 40 Hz knot cadence — on which the firmware interpolator's correctness depends — is
> not protected from the cost of the planner that produces it.**

Three properties combine. Each is individually defensible; together they are a trap.

**(R1) The plan is time-parameterised against the wall clock.**
`_emit_once` samples `tau = now - t0`. When the emitter is late, the plan has *already*
advanced by the full lateness. Nothing couples plan-time to whether knots actually went
out.

**(R2) The firmware interpolator decays to a standstill when knots are late — correctly.**
`leg_interp.cpp`: Hermite for `dt <= SEG_T (25 ms)`; Taylor extrapolation to
`MAX_EXTRAP_DT_S (50 ms)`; then a decay that reaches zero velocity by ~110 ms. A late
knot therefore *stops the robot* while (R1) keeps the command moving. The command runs
away from the encoder at the plan's full commanded speed.

**(R3) The rejection path is the most expensive path, and it runs inline on the emitter
thread before the knot is sampled.**
`_follower_tick` → `follower.follow` → `build_follow`, which on an infeasible target
runs a **6-iteration** horizon-stretch search, each iteration a full `validate_follow`
(~1.5–4 ms). So precisely when the target is infeasible, the 25 ms tick blows its
budget — which via (R2) stalls the robot, which via (R1) makes the command diverge.

**The feedback loop is vicious**: infeasible target → expensive reject → late knot →
robot stalls → command diverges → next replan seeds from the (now far away) *commanded*
state, not the robot → still infeasible → ... → `MAX_DEVIATION`.

`_current_state()` samples the **active plan**, never the encoder. There is no feedback
term anywhere in the follower.

### 2.1 Why vertical, and why descending

Vertical is the stiffest direction: a pure-z platform move drives all six legs together.
From the **Phase-2 battery's own numbers**, for the same 20 mm displacement:

| axis | predicted peak leg vel | predicted peak leg acc |
|---|---|---|
| z (20 mm) | 64.1 mm/s | 362.8 mm/s² |
| x (20 mm) | 35.2 mm/s | 271.6 mm/s² |
| ratio | **1.82×** | **1.34×** |

Yet `z_mult_mm = 140` is nearly as large as `xy_mult_mm = 150`. So an equal puck
deflection is ~1.8× more demanding in z, and z is where `LIMIT_ACC` fires first —
matching the operator's report that vertical moves stuttered.

Descending is worse than ascending because the firmware's lead clamp
(`MAX_LEAD_REV = 0.15`) bounds how far *ahead* the command may sit, so after each
late-knot re-latch the ODrive sprints to close a 0.15 rev step; with gravity assisting,
the descent overshoots further. Only descent was tested on 2026-07-09, so the asymmetry
is asserted, not measured. **A cheap early experiment should test both directions.**

### 2.2 The velocity limit is not enforced anywhere downstream

This is the part that most deserves scrutiny. `leg_vel_limit_mmps = 100` is enforced
**only inside the planner's gate**, on each individual plan. Downstream:

| layer | bound | implied leg-velocity ceiling |
|---|---|---|
| emitter per-knot step | `JB_OP_MAX_POSITION_STEP_REV = 0.3` rev / 25 ms | 12 rev/s ≈ 846 mm/s |
| firmware lead clamp | `MAX_LEAD_REV = 0.15` rev × `pos_gain = 40` | 6.0 rev/s ≈ 423 mm/s |
| ODrive `vel_limit` | `ODRIVE_LEG_VEL_LIMIT_RPS = 4.0` | 4.0 rev/s ≈ 282 mm/s |
| firmware E-STOP | `MAX_DEVIATION_REV = 0.5` | (position, not velocity) |

The measured 3.555 rev/s is **89 % of the ODrive's own `vel_limit`** — i.e. the only
thing that actually capped the leg speed was the motor controller. The trajectory
layer's 100 mm/s was never a physical constraint on the executed motion once the knot
stream degraded. **Per-plan feasibility is not executed-trajectory feasibility.**

### 2.3 The pump's absorbing state

`controller/teensy_link/setpoint_pump.py:267` updates `_prev_pos` **only on accept**.
The module docstring makes this explicit and deliberate: the gate compares against "the
PRIOR ACCEPTED frame … NOT vs rejected frames." The intent — never let a corrupt frame
become the safety baseline — is right.

The consequence is not. Once one *legitimate* frame trips the gate, `prev` freezes while
the plan keeps advancing, so every subsequent frame is further from `prev` and is *also*
rejected. The stream dies permanently, with:

- no operator-visible reason (the ERROR is throttled and names only a step violation),
- no recovery path short of a restart,
- and `mpc_active` still 1, so the Teensy's staleness watchdog then fires as well.

This is what turned a recoverable stutter into a dead robot. It is a **latent
availability bug independent of the follower** — any source of one oversized step
(a `go_to_pose` seam, a catch replan, a GC pause) triggers it.

## 3. Design principles for the fix

Per CLAUDE.md — climb one level before patching, and prefer contracts over patches.

- **C1 — The knot cadence is a hard real-time contract, not a best-effort target.**
  The emitter must publish a knot every `SEG_T = 25 ms`, no matter what the planner is
  doing. The existing code reasons about the 250 ms `MPC_CMD_STALENESS_US` budget; the
  *binding* budget is **25 ms**, ten times tighter, because `leg_interp.cpp`'s Hermite
  assumes it. The docstring in `_emit_once` ("worst case … ≤ ~26 ms … ≪ the 250 ms
  staleness E-STOP") measures against the wrong constant. Everything after 25 ms is
  already degraded; everything after 110 ms is a stopped robot.

- **C2 — A streaming follower must never reject.** In a 40 Hz control loop, "reject and
  hold" makes the expensive path the common path and strands the platform on a stale
  plan. The follower's contract should be *total*: **every** target yields a feasible
  plan, because out-of-envelope targets are **clamped**, exactly as the existing
  workspace saturation already ray-clamps to the nearest reachable pose. Rejection stays
  the right answer for the *discrete* services (`go_to_pose`, `timed_target`), where a
  human asked for a specific thing and deserves to be told no.

- **C3 — Commanded state must never diverge unboundedly from measured state.** The
  firmware already enforces this destructively (`MAX_DEVIATION` → E-STOP). The Jetson
  should enforce it constructively, before the guard has to.

- **C4 — A safety gate must not have an absorbing state.** A gate that can permanently
  refuse all traffic after one trip is a denial-of-service on the robot's own control
  path. It must either recover or escalate loudly and deliberately.

## 4. Proposed changes

Ordered by value. Each is separable; I recommend landing 4.1 and 4.4 first, since they
are the two that convert a lockup into a recoverable degradation.

### 4.1 Decouple the knot emission from the replan (contract C1)

**Change**: in `_emit_once`, sample and publish the knot from the *currently installed*
plan **first**, then run the follower replan for the *next* tick.

Today: `[gap check] → [pending stop] → [follower replan: 4–26 ms] → [sample] → [publish]`.
Proposed: `[gap check] → [sample] → [publish] → [pending stop] → [follower replan]`.

**Effect**: planner cost can never delay a knot. The knot published at tick *n* uses the
plan installed at tick *n−1*, adding exactly one knot (25 ms) of latency to operator
input — imperceptible for teleop, and a fixed, analysable delay rather than a
load-dependent one.

**Additionally**: hard-budget the replan. If `perf_counter()` overruns a deadline
(propose `0.6 × SEG_T = 15 ms`), abandon the search, keep the active plan, and count the
overrun in `/trajectory/diagnostics`. A budget overrun is a *diagnostic*, not a fault.

**Risk**: MEDIUM. Reorders the emitter's hot loop. `t0` bookkeeping must be re-derived —
with the replan moved after the publish, `seed_mono` is captured after the knot goes out,
so the "no v·Δt rewind" property that motivated `t0 = seed_mono` needs re-checking, not
assuming. This is the change I would most want a second pair of eyes on.

### 4.2 Clamp instead of reject in the follower (contract C2)

**Change**: `build_follow` stops returning `TrajectoryInfeasible`. When the target is
dynamically unreachable within the horizon, clamp it along the current→target ray to the
furthest point that *is* reachable at the current limits, and plan to that. Emit the same
throttled WARN the workspace clamp already emits. Cap the stretch search at **2**
iterations, not 6.

**Why this is not optional.** At the Phase-1 defaults with `spacemouse_horizon_s = 0.35`,
a rest-to-rest quintic covering distance `d` has peak platform velocity `1.875 d / T`.
Requiring peak leg velocity ≤ 100 mm/s with the measured z-factor of 1.82 gives

```
d ≤ 100 × 0.35 / (1.875 × 1.82) ≈ 10.3 mm
```

**A full-deflection z command must be ≤ ~10 mm for the gate to accept it.** Even after
the Phase-4 ramp to 156 mm/s the bound is only ~16 mm. So *any* usable teleop sensitivity
at these limits generates infeasible targets continuously. Lowering `z_mult_mm` is a
band-aid that trades away the whole point of a SpaceMouse; clamping is the actual fix.
With clamping, a full deflection means "go toward there as fast as is safe" — which is
what a velocity-style teleop input should mean anyway.

**Risk**: MEDIUM. Changes what the operator feels. Needs the deadband and the
saturation-WARN throttle re-tuned together.

### 4.3 Bound commanded-vs-measured divergence (contract C3)

**Change**: the follower gains a divergence guard. Each tick, compute
`d = max_i |u0_i − pos_estimate_i|` from the latest telemetry. Then:

- `d > d_warn` (propose 0.15 rev, matching `MAX_LEAD_REV`): **stop advancing plan time**
  — freeze `tau` — until the robot closes the gap. The command waits for the robot
  instead of running away from it.
- `d > d_stop` (propose 0.30 rev, i.e. 60 % of the firmware's 0.5 rev E-STOP):
  install a graceful stop and latch a loud WARN.

This is the constructive analogue of `MAX_DEVIATION`, acting well before the guard must.
It also makes (R1)+(R2) safe by construction: if knot starvation stalls the robot,
plan-time freezes and the command cannot diverge.

**Open question for review**: freezing `tau` is a *time* distortion — the plan's velocity
profile is no longer traversed at unit rate, so the emitted `v0` feedforward will
disagree with the actual `dpos/dt` across the freeze. Cleaner might be to re-seed the
plan from the measured state whenever `d > d_warn`. That introduces encoder feedback into
the planner, with its own stability question at 40 Hz. **I do not have a confident
recommendation here and would rather discuss it than pick.**

**Risk**: HIGH. Introduces feedback into a previously open-loop path. Needs an explicit
control-system analysis (per CLAUDE.md) walking one MPC cycle step by step before any
code is written.

### 4.4 Remove the pump's absorbing state (contract C4)

**Change**: `SetpointPump` must not be able to reject forever.

Proposed semantics — a rejected frame still does not become the baseline, but a *run* of
rejections is escalated rather than silently sustained:

- Count consecutive rejections. On the **first**, log at ERROR (unthrottled) with `cmd`,
  `prev`, and the leg index — as today.
- After `N` consecutive rejections (propose `N = 2`, i.e. 50 ms), the stream is by
  definition unusable. **Disarm** (`mpc_active → 0`) and publish the reason on
  `/link_status`. A deliberate, announced, recoverable disarm beats a silent stream death
  followed by a firmware staleness E-STOP.

I deliberately do **not** propose "re-baseline `prev` to the new `u0`" — that would let a
genuinely corrupt frame slide through by the simple expedient of arriving twice. Failing
closed and loudly is the right default for a safety gate.

**Risk**: LOW–MEDIUM. Well-contained in one class with existing unit tests
(`tests/teensy_link/test_setpoint_pump.py`). The disarm-on-run is a new behaviour that
needs its own test and a line in the runbook.

### 4.5 Diagnostics correctness (not safety, but it blinded this investigation)

- `_install` resets `_run_peak_*` on every install, and the follower installs **every
  tick**, so `realized_peak_*` in `SPACEMOUSE`/`CATCH` is a per-knot value — always ≈0.
  Track realized peaks over a rolling window instead of per-install.
- The teardown `go_home` in the Phase-2 bag installed as `move_seq=12` with realized
  peaks 0.0 (correct) but **predicted** peaks identical to move 11's (44.3 / 303.6 /
  6911) rather than zero — `peak_leg_*` looks stale for a zero-distance plan.
- Publish `max_emit_gap_ms` **and** a per-window overrun count, and surface the emit gap
  in `/diagnose`. The gap is the leading indicator of this entire failure mode: it went
  25 → 62 → 78 → 146 ms over the 30 s preceding the E-STOP.

**S4 depends on both bullets above**: its per-step ramp protocol reads
predicted-vs-realized headroom straight off these fields.

## 5. What I would verify, and how

Per CLAUDE.md's empirical-probe rule, prototype before writing tests that assert on
thresholds.

1. **Reproduce in sim, offline.** Drive `trajectory_node`'s follower with a recorded
   `platform_pose_topic` stream from the 13:49 bag (the real puck trace) and an
   artificially slowed `validate_follow`, and confirm the emit gap → divergence →
   `MAX_DEVIATION` chain reproduces without hardware. This is a `tools/probes/` harness,
   committed, not a `/tmp` throwaway (the trace is reusable for 4.1–4.3).
2. **Measure the real replan cost distribution** on the Jetson: `validate_follow` p50/p99
   for accept vs reject paths. The 1.5–4 ms figure in the Phase-3 outcome is for the
   *accept* path; the 6-iteration reject path was never profiled. Everything in 4.1 hinges
   on this number.
3. **Test both directions** (ascent and descent) at a fixed deflection, to confirm or kill
   the asymmetry claimed in §2.1.
4. Property test for 4.2: for random target streams (including step changes to
   unreachable poses), the emitted knot sequence has bounded discrete vel/acc/jerk and
   **no rejections ever occur**.
5. Property test for 4.4: no sequence of frames can put the pump into a state where it
   rejects indefinitely without disarming.

## 6. Interim operating envelope (before any of this lands)

S3 must not be re-attempted as flown. Until 4.1–4.4 land, if the platform is to move in
`SPACEMOUSE` mode at all:

- Reduce `xy_mult_mm` / `z_mult_mm` substantially (a starting point consistent with §4.2's
  arithmetic: `z_mult_mm ≈ 15`, `xy_mult_mm ≈ 30`). This keeps the *typical* target inside
  the feasible envelope; it does not remove the failure mode at full deflection.
- Watch `/trajectory/status.max_emit_gap_ms` live and **abort on anything above ~40 ms**.
- Fly gently, and test ascent before descent.

My recommendation is to **land 4.1 + 4.4 first** (cadence protection + no absorbing
state), re-fly S3 at reduced sensitivity to confirm the stutter is gone, and only then
take on 4.2 and 4.3 — with 4.3's feedback question settled in discussion first.

## 7. Open questions for Harrison

1. **4.3 is the one I am least sure of.** Freeze plan-time, or re-seed from measured
   state? The second is more principled and more dangerous. Your call, and I would like
   to walk one 40 Hz cycle through both before committing.
2. Is a one-knot (25 ms) added input latency from 4.1 acceptable for teleop feel? I
   believe it is imperceptible, but you are the one who flies it.
3. Should 4.4's disarm-after-N-rejections instead be a *stow*? Disarm leaves the platform
   holding wherever it is; stow brings it down. Disarm is less surprising; stow is safer
   if the operator has walked away.
4. `MAX_LEAD_REV = 0.15` × `pos_gain = 40` permits a 6.0 rev/s velocity command, above the
   4.0 rev/s ODrive ceiling. Is that intentional headroom, or should the lead clamp be
   tightened to `vel_limit / pos_gain = 0.10 rev`? This is a firmware-config question that
   sits outside this proposal but was surfaced by it.

---

## Archival note (2026-08-01)

**Archived as superseded — no open items.** The plan's own header records the
closure: RESOLVED 2026-07-10, superseded by the implemented chase-clamp
follower rework (`73dba2b`), with S3 re-flown the same day to an operator PASS.
Everything below the RESOLUTION section is the original proposal, preserved
unchanged as the record of what was proposed versus what actually shipped.
Last content edit 2026-07-10 (`cf8728b`).

Moved out of `plans/active/` by the 2026-07 refactor programme
(`plans/active/refactor-2026-07.md` § Phase 1, item 5).
