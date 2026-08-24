---
title: Critical-Point ILC — Task-Level Iterative Learning on the Throw and Catch Events
created: 2026-08-11
status: active
related_logbook:
  - 2026-08-11-critical-point-ilc-plan-kickoff.md
  - 2026-08-21-ilc-primary-foldin.md
related_config:
  - config/hardware_config.yaml → jugglebot_operational.toss_ilc_enabled (ships false; owner decision 2026-08-21 makes true acceptable — flipping it is deliberate feature work with a named tripwire)
  - config/toss_ilc.yaml → the learned per-cell artifact (machine-written, schema v1, absent ⇒ exactly zero)
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py (the production stroke closed form — part of the differentiated forward chain)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/toss_release.py::compute_release_state_tilted
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/ballistics_bc.py
  - ros_ws/src/jugglebot/jugglebot/tracking/kalman.py (arrival/release kinematic truth)
---

# Critical-Point ILC — Task-Level Iterative Learning on the Throw and Catch Events

**Status: THE PRIMARY toss learning architecture** (owner decision, 2026-08-21 —
see § The 2026-08-21 fold-in). Phases 0–2 are DONE, audited and shipped DORMANT;
what remains is the build ladder in that section and then Phase 3, an operator hardware
A/B. Both gates that parked this arc on 2026-08-14 have CLOSED (G-1 `9cd2bee`,
G-2 `b084f98`). Reference method: Suresh & Atkeson, *"Learning Dynamic Rope
Manipulation Using Task-Level Iterative Learning Control"* (arXiv:2602.21302);
reference code `github.com/krish-suresh/flying_knots_public` (MIT).

## Context — why this plan exists

Owner framing (2026-08-11): **only the throw and catch events really matter in
a juggling routine**, and the recent calibration/mapping work wants a more
*holistic* mechanism for "the robot is not identical to the models" than
per-channel loops. The current residual-correction stack is layered and
channel-specific — the inclinometer tilt map (gravity frame), the ball-measured
toss aim map + session trim (both on the `mvp-trajectory-bringup` branch,
postdating this branch's base), and the planned tracking-level torque trims
(`learned-ff-residuals.md`). Each is correct in its lane; none can express a
**coupled, multi-channel task error** — landing position AND arrival-velocity
direction AND contact softness responding jointly to aim, stroke velocity and
timing.

The flying-knots method supplies exactly that structure, hardened on a far
worse plant (a rope): a **task error measured on hardware at sparse critical
points**, a **model used only for sensitivities**, and a **small trust-region
update on a low-dimensional command parametrization**, iterated per trial.
Convergence tolerates a crude model because the residual being nulled is the
real one; the model only tilts the update direction. The method reached 100%
task success within 10 hardware trials across 7 rope types from one
demonstration.

Jugglebot is structurally better-placed than the paper's task: the task is
self-specified (ball lands in the cup — no demonstration needed), and the
"hard dynamics" are compressed into two short events (release, contact) whose
surrounding physics (ballistic flight, stroke closed form) is analytic.

## The method, adapted

| Paper concept | Jugglebot adaptation |
|---|---|
| Rope state at the self-collision instant | Ball + hand state at the two critical points: RELEASE→LANDING (throw) and CONTACT (catch) |
| One human demonstration | None needed — the nominal command is the current analytic planner output; the goal is the cup |
| Bézier joint-trajectory command (~100 dims) | The toss-cycle scalar command vector, ≤ 8 dims (§ Command vector) |
| Particle rope model + implicit-function KKT differentiation | Central finite differences through the **production** planning chain (§ Design constraint 1) |
| Linearized joint pos/vel/accel/torque constraints in the QP | **Exact** re-validation of each proposed command by the real admission gates (§ Design constraint 3) |
| Follow-through tracking cost (post-event regularization) | Deferred to the two-ball tier: recovery-readiness regularization (Phase 4 sketch) |

Deliberately **not** imported: Drake, the particle model, the mocap
demonstration pipeline, symbolic/AD differentiation of dynamics, Clarabel. The
ball's flight needs none of it.

## Design constraints (load-bearing)

1. **One forward chain, differentiated numerically — never a symbolic twin.**
   Sensitivities come from central finite differences through the production
   functions themselves (`hand_stroke`, `toss_release`, `ballistics_bc`,
   `tilt_geometry`). Root cause, not convention: a second copy of the stroke
   *timing* has already drifted once in this codebase
   (`JB_OP_HAND_CATCH_PRIME_REV`, 3.2 mm off for the life of the constant —
   `hand_stroke.py` module header tells the story), and a symbolic
   re-derivation for ILC would be exactly such a second copy, failing more
   quietly. The command vector is ≤ 8-dimensional and the chain is smooth
   closed forms; finite differences are accurate here and structurally
   incapable of drifting from what the machine executes.
2. **Batch, offline, one apply point, nothing in the 40 Hz path.** Learning
   runs between sessions from mined records; the learned correction is applied
   **once per toss goal** at goal-build time, the same seam the aim map uses on
   the `mvp` branch. Owner rule (2026-08-10) — determinism always: no blocking
   I/O, no adaptation, no measured-state dependence in control loops.
3. **Updated commands are validated by the real gates, not by linearized
   constraints.** Every proposed `u′ = u + Δu` re-runs the actual production
   admission: the `ThrowTiltInfeasible` clamp gate, workspace/stroke
   feasibility, the limits tier. A refusal shrinks the trust region and
   re-solves; a command that cannot pass the gates is never emitted. The
   paper's linearized limit constraints are an approximation forced by a
   ~100-dim command; at ≤ 8 dims exact re-validation is affordable and
   strictly safer.
4. **Truth channels are fixed by decision, not convenience:**
   - **Possession**: the hand ball sensor tri-state (`ball_held` gated by
     `ball_held_valid`), **never** the mocap caught/dropped verdict — the
     owner distrusts it (2026-08-11: it is "often incorrect in its prediction
     of whether the ball was caught"). Binary, so it enters as an
     admission/abort gate, never as a differenced error channel.
   - **Landing / arrival kinematics**: mined offline from bagged mocap +
     tracker output — the toss-selftuning design's D5 discipline
     (`plans/active/toss-selftuning.md`, `mvp-trajectory-bringup` only)
     carries over: live closure stays forbidden.
   - **Contact softness**: hand-drive channels pending the Phase-0b probe —
     `vel_meas` (TELEMETRY frame, nominally 100 Hz — the 0b census measured
     ~10 % duplicate samples and ~50 Hz effective through fast strokes) and
     `iq_meas`
     (DIAGNOSTIC frame: on-change at |Δiq_setpoint| > 0.5 A, else a 1 Hz
     forced refresh, republished at 100 Hz from cache — the `hand_telemetry`
     publish rate is not the per-field refresh rate).
5. **Provenance-keyed, dormant on mismatch, byte-identical OFF.** The learned
   artifact records the tilt-map version, estimator version and a model-config
   identity; any mismatch loads the artifact DORMANT with a loud WARN (the
   aim map's D3 pattern). The gate flag defaults false; the OFF path emits
   today's bytes exactly.
6. **Learned-magnitude monotonicity is a plant-drift alarm.** A re-learned
   correction must shrink or hold session-over-session; growth triggers an
   investigation of the plant, never a bigger correction (the
   `learned-ff-residuals.md` risk-inversion, adopted verbatim).

## Command vector u and task error e (v1 candidates)

**u, throw side:** aim tilt `(rx, ry)` (rides the tilted release path via
`compute_release_state_tilted` — the toss-selftuning D2 tier-independent
re-keying this presupposes arrived with the G-3 merge on 2026-08-12; before
it, the tilted path was gated on `JB_OP_TOSS_TIER == '8b'` while the shipped
default is `'8a'`); hand `event_vel` trim `δv`; release-timing offset
`δt_rel` (hand-stroke phase relative to the platform cycle — **refused from
v1 at Phase 1**: its Jacobian column is structurally zero, a rigid time
translation; the physical invariance is argued, not measured — see
§ Phase-1 core results).
**u, catch side:** catch-pose offset `(x, y)`; catch `event_vel` trim; catch
timing offset.

The v1 subset is **frozen by the Phase-1 conditioning analysis** — singular
values of the sensitivity matrix `F`; channels whose columns are degenerate
with another channel or below the measurement noise floor are excluded from v1
rather than regularized into noise.

**e, throw critical point:** landing-position error (2); arrival-velocity
direction error at the catch-plane crossing (2); flight-time error (1).
**e, catch critical point:** relative contact velocity `|v_ball − v_hand|` at
the contact instant — a **modeled surrogate**, differentiable through
`hand_stroke` + ballistics — plus the **measured impact transient** as a
validation channel.

**e, catch critical point, third channel — hand-sensor settle (owner,
2026-08-12):** the hand ball sensor is highly reliable, so a catch whose
reading bounces between BALL and EMPTY after arrival is a **messy catch**.
The score counts `ball_held_raw` flips (and debounced `ball_held` verdict
transitions) inside a settle window after the first post-arrival BALL
sample. Roles, fixed here: a graded catch-quality label on every record; a
Phase-3 regression criterion (the messy rate must not increase under
learning); the 0b softness metric's cross-check ground truth (harsher
contact should predict more bounce); and an admission grade for the learner
— **not** a differenced QP channel (no analytic ∂/∂u; the same role split as
the measured transient). Cadence caveats inherited from the toss-build 2a
findings and sharpened by the 0d census: the sensor poll cadence is
session-dependent (per-bag p50 spans 20–76 ms; the reference bag's 71 ms is
not a standing property — read `sensor_poll_dt_ms_median` per record), and
the debounce lag is asymmetric and tracks the poll (≈0 ms on the catch
edge; ≈241 ms on the departure edge at a 71 ms poll, ≈110 ms at 20 ms) — so
Phase 0d opened with the same distinct-sample census discipline as
`iq_meas`.

**The SNR→0 problem, named** (owner, 2026-08-11): as catches soften, the
measured impact transient sinks toward the noise floor exactly at the target.
Role split, by construction: the *optimized* channel is the modeled relative
contact velocity (never degenerates as the catch improves); the *measured*
transient validates the model, monitors regressions, and admission-gates
records — it is not the quantity being nulled.

## Sensitivity and update law

`F = ∂e_model/∂u` by central finite differences through the production chain
at the current command. Update: damped least squares with a box trust region —

```
Δu = argmin ‖F Δu + e_meas‖²_Q + ‖Δu‖²_R,   subject to ‖Δu‖∞ ≤ τ
```

solved in closed form (normal equations + clip) — no QP-solver dependency at
this dimensionality; revisit only if a constraint must bind *inside* the
solve. Each candidate `u + Δu` then passes the exact-gate re-validation loop
(constraint 3). Corrections accumulate into a **per-goal keyed artifact** —
pose-keyed like the aim map's grid, with the cell value generalized from a
2-vector aim to the u-correction vector.

Pinned cross-check: the landing-position/aim block of `F` must reproduce the
known small-angle identity `b = 4·h·θ` (derived in
`plans/active/toss-selftuning.md` § F1 on `mvp-trajectory-bringup` — not in
this tree until G-3; the in-tree statement of the same geometry is
`ros_ws/docs/levelling_frame.md`'s `4·h·sin θ`) within tolerance — the one
block with an independent analytic answer.

## Relationship to the existing correction layers — what this does NOT obviate

- **Layers 0/1/2 stay.** v1 learns the residual *on top of* whatever is
  applied. Per-toss records carry the applied aim, so the fit is a converging
  fixed point rather than a one-shot measurement — the same argument the aim
  map's provenance design already makes.
  **AMENDED 2026-08-21 (decisions 1 and 4).** Layer 0 stays untouched. Layer 1
  is **empty and staying empty** — no aim map was ever captured and the campaign
  that would capture one is retired — so `map_aim_rad` is 0 and this bullet's
  "on top of whatever is applied" reduces to layer 0 plus ILC itself. Layer 2's
  **aim** estimator stays *running* but with **zero authority** (monitor-only);
  its `speed_gain` retires; its `release_latency_ms` stays trim-side and unwired.
- **The toss-selftuning substrate is required infrastructure**: the per-toss
  record schema, the bag miner, the admission filters and the acquisition
  tool's refusal machinery (all on `mvp-trajectory-bringup`, landed 2026-08-11,
  postdating this branch's base — gate G-3). ILC replaces none of it; it rides
  it.
- **What ILC may eventually absorb**: the aim map's *update law* and the
  acquisition ladder's later rungs — an aim map is the 2-DOF special case of
  the per-goal correction vector. That is a **decision required at Phase 3, on
  A/B evidence** — not assumed here, and the selftuning capture sessions stay
  valuable regardless: the same bags feed both loops' corpora.
  **TAKEN 2026-08-21, ahead of Phase 3 (decision 4).** The absorb-or-keep
  decision was made early *because the evidence changed*: the map does not
  exist, so "keep" would have meant spending ~2 h 10 m of bench time building
  the thing to compare against. ILC absorbs the aim map's update law, SC-0…SC-3
  and `toss_fit_lib`'s per-node fit. What that does **not** touch: the record,
  the miner, the labeller, the guards, the clamps, layer 0, or the substrate
  seams `toss-selftuning.md`'s SUPERSESSION NOTICE lists as retained — and the "same bags feed both corpora" argument survives intact,
  because ILC's corpus *is* ordinary session bags.
- **`learned-ff-residuals.md` stays orthogonal** (tracking layer, torque
  channel, explicitly forbidden from touching the reference). This plan
  touches only the *planned command*, upstream of the reference gates: the
  emitted trajectory remains an ordinary gate-validated plan, so the
  `MAX_DEVIATION` guard semantics and the K-contracts are untouched by
  construction.

## Prerequisite gates

- **G-1 (hard, for any hardware-learned artifact):** the bridge
  temporal-trustworthiness arc's latency fix + monitor are closed. Same root
  cause as `learned-ff-residuals.md` G-A: a transport latency drifting
  10 → ~240 ms with bridge uptime is non-repeatable error that iteration
  would chase and bake into artifacts. Defense in depth regardless: the
  learner refuses records whose `uptime_ms` exceeds the healthy threshold.
- **G-2 (hard, for the catch channel on hardware):** the hand-ODrive
  configuration is restored from the backup JSON — the braking clamp
  (measured braking floor ≈ −11.8 A against a −27.2 A commanded feedforward,
  with `torque_soft_min` configured at exactly −10.00 A; diagnosed
  2026-08-10, `logbook/2026-08-10-hand-drive-braking-clamp-diagnosis.md` on
  `mvp-trajectory-bringup`, not in this tree) sits directly in the
  catch-softness path; learning against a clamped drive fits the clamp, not
  the catch.
- **G-3 (mechanical): SATISFIED 2026-08-12** — merge commit `712bcee`
  brought `mvp-trajectory-bringup` at `e75badd` (a superset of the required
  `7cb818d` substrate) into this branch; the gate passed on the merged tree
  (5073/5078, 2026-08-12). Original obligation: merge/rebase this branch
  onto the substrate state before Phase-0 *implementation* starts — this
  workspace deliberately based at `5e046cc`, prior to both in-flight arcs
  (owner direction, 2026-08-11). Post-merge, every "not in this tree" /
  "mvp-only" annotation in this plan is historical: the cited files and
  sections are now in-tree.
- **Not gated on** completion of the toss-selftuning capture campaign — see
  § Relationship for the rationale.

## The 2026-08-21 fold-in — ILC becomes the primary learning law

Recorded in `logbook/2026-08-21-ilc-primary-foldin.md`. The decisions are the
owner's; the derivations under them are this plan's. **This section takes
precedence over any earlier text in this file that contradicts it.** Earlier
text is kept rather than rewritten, so the arc stays reconstructible — where a
paragraph above is now wrong, this section says so by name.

### The six decisions, and the root cause each one closes

| # | Decision | The failure it prevents (not "the plan says so") |
|---|---|---|
| 1 | **ILC is always applied.** `JB_OP_TOSS_ILC_ENABLED` may default **true**; every dormancy, provenance and clamp gate stays. The layer-2 trim's **aim** estimator demotes to **MONITOR-ONLY** — zero authority, still observing and logging. | A default-false learned correction is a correction nobody ever runs: the flag becomes the thing that gets forgotten, and the A/B never happens. `true` is safe *only* because an absent `config/toss_ilc.yaml` is exactly zero — not a hull clamp, not an interpolation (`toss_ilc.lookup`'s miss rule) — so a machine with no artifact is bitwise the machine we fly today, and the `test_shipped_config_has_the_feature_off` tripwire is what has to move deliberately. Two estimators with authority over one quantity is C4; one converging estimator per quantity is the fix. |
| 2 | **C1 (the common mode) is resolved INSIDE ILC**, by transposing § 3.2 of [toss-selftuning.md](toss-selftuning.md): a **session-local common-mode component** (RAM, discarded at goal end, its own evidence gate) plus a persisted per-cell artifact carrying **only the spatial residual**. No persisted cell may absorb one session's `level()` draw. | `level` is one int16 SCL3300 sample with **1.2–1.7 mrad/axis** session-to-session scatter, and D3 says a re-`level` deliberately does *not* invalidate a map. Against the measured per-cell \|aim\| of 9.1–10.5 mrad that is **11–19 % of every persisted cell** being one session's noise draw, frozen forever, re-applied on every future session that never took that draw. |
| 3 | ✅ **RE-TAKEN AND ACCEPTED, operator 2026-08-23.** The 2026-08-22 audit condemned this row's original target (**R5-prime, dwell 0.49 s at T = 0.4949 s, ~61 throws/min**): that goal never threw a ball, and neither did R4 or R5 as published, because the accept-time `throw_delay` floor modelled the kind-0 dispatch budget alone while `_step_preparing` re-checks it against the lead REMAINING after CHECKING + POSITIONING + PREPARE. Both items that gap named landed 2026-08-23 and the operator accepted the corrected ladder: (a) `trajectory/commanded_pose` publishes the commanded ORIENTATION, so `_toss_already_positioned` verifies a pre-tilt pose and the census-B1 skip fires on an AIMED chain — the whole 0.38 s LEVEL/AIMED gap, worth **39.7 → 51.6 throws/min** at the operating point; (b) both delay gates now charge one shared derivation (`toss_sequencer.min_throw_delay_for_release_s` = dispatch budget + pre-dispatch sequence), so a goal that passes accept **cannot** die `ABORTED_CANT_MAKE_RELEASE` — asserted over the whole (T, dwell, delay, aim) grid by `tests/motion/test_cadence_rung_check.py`. **The accepted target is R5-prime — `throw_height_m 0.31`, `dwell_time_s 0.66`, `throw_delay_s 0.44`, period 1.163 s, 51.6 throws/min, with or without an armed aim**; the reachable frontier is 54.3 and the rung sits ~12 ms back from both floors. 61 throws/min is not available on this build and this row no longer asks for it. Everything else in this row STANDS: the ladder R0→R5 is the route, R6 is a DEFERRED firmware fork not being built, and `MIN_TOSS_THROW_DELAY_S` is retired in favour of the ~0.1 s dispatch debounce plus derived state-based interlocks. | The 0.25 s dwell is **unreachable at every admitted flight time** — the hand-stroke floor bottoms at 0.2508 s at the very top of the C-HAND-3 band and is 0.490 s at a juggling-realistic flight, and only a Platform-Teensy `calcCatch` geometry change moves it. Meanwhile `MIN_TOSS_THROW_DELAY_S = 3.5 s` protects a sequence that measurably costs **0.70 s** — it is a policy margin of ~2×, not a physical floor, and leaving it in place while asking for cadence would push every cycle to the edge of a fence that is protecting the wrong thing. Full census: [toss-selftuning.md](toss-selftuning.md) § 11; the executable ladder with per-rung PASS/ABORT is [`tests/hardware/session_cadence_ladder.md`](../../tests/hardware/session_cadence_ladder.md). **LANDED 2026-08-22**: the constant is retired, the event-delay floor is derived `f(v_throw)`, and `required_dwell_s` gained the hand-geometry term the census's own § 4 demands but no code enforced. The census's 0.2508 s / 0.490 s figures reproduce against the tree as **0.2505 s / 0.4871 s** (≤3 ms; its catch-tail column runs ~3 ms high), so R5-prime cleared the HAND floor by **2.9 ms** — which stopped being the binding floor on 2026-08-22 (the audit's `handoff_margin_s` fix added the hand's park re-entry, 0.1933 s at the band floor, to the handoff term; R5-prime as decided is now 43 ms SHORT and is REJECTED_DWELL). And the census's one open firmware question is CLOSED: the C-HAND-1 gate is written against **`t7`, not `t8`** — `buildCatch` emits `tA[4]={t4,t5,t6,t7}` and `t8` is read only by the dead kind-2 `buildCommand`, so `END_PROFILE_HOLD` adds nothing and the 0.49 s point survives. |
| 4 | **SC-0…SC-3, the `toss_cal_grid` acquisition campaign and `toss_fit_lib`'s per-node map fit are RETIRED as designed** — superseded by ILC. The seams ILC rides are KEPT. | `config/toss_calibration.yaml` **has never existed in either tree**. The aim map was never captured, so this plan's Phase-3 baseline ("aim map alone") was never constructible, and the campaign that would build it costs ~2 h 10 m of supervised bench time to produce a 2-DOF special case of a per-goal correction ILC already extracts from ordinary session bags. Spending that sitting to construct a baseline we would then decide to absorb is the expensive way to take a decision that is now cheap. |
| 5 | **`k_v` lives in ILC** (`event_vel_trim`); the trim's `speed_gain` retires with the aim estimator. Release latency `τ` stays trim-side, **unwired**, noted. `catch_timing_offset` lands as the first catch channel **only if** the seam and the sensor catch-event residual make it a clean gated addition. | `toss_trim.SessionTrim.speed_gain` is documented NOT WIRED and its seam is dispatch-time — its own docstring says that bypasses the FSM CHECKING enforcement. ILC's seam is the goal build, *upstream* of the gate, which is the only place a speed trim can be validated before it becomes a command. Two unwired implementations of one knob is how a knob gets wired twice. |
| 6 | **H2 (the fixtured-ball centroid capture) is RETIRED**; **C3 resolves BY DECISION**: `arrival_dir` is the PRIMARY and ONLY aim residual driving the update; `land_err_x/y` demote to MONITOR-ONLY, masked out of the aim update and never Q-arbitrated against `arrival_dir`; absolute centering closes through **catch outcomes**. | Owner pushback, and it is decisive: conventional point markers on the ball would **corrupt the very trackable surface being measured** — the ball is fully tape-covered and QTM sees one blob, so a marker cluster changes the blob whose centroid the measurement is about. The owner's mechanism reading (platform-frame occlusion near the bottom of the stroke) is consistent with the arc's own data: the fitted bias is large at z ≈ 880 (the catch-plane height) and vanishes by z ≈ 1880, and it is parity-EVEN. So the measurement H2 would take is the one measurement this instrument cannot take, and the channel that never carried the bias in the first place is already in hand. |

### Contradiction ledger — the digest's C1–C8 and what each one now is

| # | Contradiction | Resolution | Owner or derived | Lands in |
|---|---|---|---|---|
| **C1** | The persistent per-goal artifact absorbs exactly the common mode § 3.2 exists to keep out of persistence (11–19 % of each cell is one session's `level()` draw). | **APPLY SIDE RESOLVED 2026-08-22**, fit side open. Decision 2 — session-local common-mode component + spatial-residual-only cells. Landed: the artifact's optional top-level `anchor` block (`aim_rad` + `n` + `se_rad`, all-or-nothing, inside the version hash), a **third** parse-time bound on `cell + anchor` that neither half can see, `toss_ilc.IlcSessionCommonMode` with the trim's evidence gate transposed and its three constants pinned equal, and the node lifecycle (seeded at `_toss_trim_begin`, discarded at `_toss_trim_end`, never written back). **Still open — the FIT half**: `ilc_fit_lib`/`ilc_fit.py` computing `mean_over_anchor_visits(û(anchor))`, emitting anchor-referenced cells and stamping the anchor's `n`/`se_rad`. Deferred on evidence, not effort: `n` counts independent `level()` draws and the anchor mean is a between-SESSION shrinkage mean, so choosing "which cell is the anchor" and "what counts as a visit" needs the multi-session corpus C6 says does not exist yet. An artifact written today declares no anchor, which composes to exactly zero. | **Owner** | Build step 2b |
| **C2** | `ILC_SPEED_AUTHORITY = ±0.15` is inadmissible near both ends of the derived flight band: `+0.148` at T = 1.00 s, `+0.043` at 1.10 s, `+0.000` at the 1.1485 s ceiling (`DECEL_FF_HEADROOM`), and **`+0.000` on the negative side at T = 0.4949 s** (`ARM_WINDOW`). *(Numbers are the bisected ones from `tools/probes/ilc_speed_band.py`, re-run 2026-08-22 and unchanged. The digest's first pass reported `+0.145` / `+0.040` from a coarser sweep; both figures are superseded, and consequence (b) below always carried the bisected pair — this row was the one out of step.)* Gate 1's "neither rail is reachable anywhere in the band" is **false on the current machine**. Worse, the node's apply seam checks only `validate_event_vel`, so a trim that clears the wire band but fails the envelope makes `TossSequencer` CHECKING reject the whole toss — **layer 3 becomes a gate**, violating its own "refinement, never a gate" rule. | **RESOLVED 2026-08-21.** Gate on `throw_envelope.evaluate(T, v)` in **both** `ilc_fit_lib.admit_command` and the node's apply seam, and replace the scalar authority with the envelope-derived T-dependent band (0.15 stays as the outer ceiling). A refusal at the apply seam **drops the trim and flies nominal** — never kills the goal. | Derived (digest § 4.5 step 1) | Build step 1 |
| **C3** | `land_err` (plane position, carries `b_y` absolutely) and `arrival_dir` (whole-arc, bias-immune) disagree **systematically by +18.0 mm in y in every cell** while agreeing in x to 0.85 mm; `weight_matrix`'s Q arbitrates a systematic disagreement as though it were noise. | **RESOLVED 2026-08-22.** Decision 6 — `arrival_dir` primary and only; `land_err` monitor-only. `DEFAULT_MASK` is `FULL_MASK - MONITOR_MASK` = `[0, 0, 1, 1, 1]`; `FULL_MASK` survives by name so the 08-13→08-21 answer stays reproducible. The standing replacement validation is `ilc_fit_lib.channel_disagreement`, logged per toss and censused into every fit. **The safety question the demotion had to answer, answered by measurement first**: zeroing two lateral entries removes SNR from the aim columns, and an aim column below `SCREEN_SNR_MIN` would be EXCLUDED — the decision would switch the aim channel OFF rather than re-source it. Whitened aim-column norm 2.9207 (full) → **2.4527** (decision 6) → **1.9330** (decision 6 + D2's sigma), all far above the 1.0 floor, retained set unchanged. | **Owner** | Build step 4a |
| **C4** | Layers 2 and 3 double-count **in both directions**: `toss_trim.reduce_to_aim` reads `applied_aim_rad` (now including ILC) but subtracts `map_aim_rad` only, so with both live the machine over-aims by the ILC contribution while the trim reports CONVERGED; and a converged trim makes ILC's measured residual read `J·ILC_prev`, so the artifact **unlearns itself to zero**. | **RESOLVED 2026-08-21.** Decision 1 — the trim's aim estimator has **zero authority**, so the first direction is closed by construction. The second is closed by the trim contributing 0 to `total_aim_rad`. **The monitor's arithmetic still has to change**: it must subtract `map_aim_rad + ilc_aim_rad`, or its divergence read-out is biased by exactly the quantity it exists to watch. Pin with a test that runs both layers enabled. | **Owner** (authority) + derived (arithmetic) | Build step 2a |
| **C5** | Clamp starvation — cells already sit at 52–60 % of the 1.0° total authority, and a clamp hit refuses layer 3 **whole**, so any future map contributing ≳0.4–0.48° in the same direction silently zeroes the learned correction. | **RESOLVED 2026-08-22.** Dissolved by decision 4: no map is being captured, so `map_aim_rad` is 0 and the D7 clamp arbitrates ILC alone. Composition is now `clamp_total_aim(map + ilc_spatial + ilc_session)` with the single D7 clamp still final, and the refusal semantics are unchanged in kind but sharpened in scope: a hit drops **both** layer-3 components together. Root cause for that, and it is C1's: the cells are referenced TO the anchor, so `spatial` alone is a residual about a baseline the machine is not applying and `session` alone is a baseline with its residual removed — half a decomposition is not a smaller correction, it is a different one, which is risk 5's own argument. | Consequence of decision 4 | Build step 2b |
| **C6** | The corpus is historical: all 19 rows are pre-FW-14 bridge, clamped hand drive, 16/19 at 16.7 h uptime, tier 8a, end stop 11.1. `partition_key` includes `bridge_fw_version`, so a fresh corpus **cannot legitimately pool with it**. | Accepted as the library working correctly. Phase 3 starts from a **fresh capture** on the FW-14 / restored-drive / 8b machine. Whether the +11 % fast throw survives the drive restoration is unknown and cheap to re-measure — and it is now a **prerequisite**, not a curiosity (see § Two consequences). | Derived | Build step 5 |
| **C7** | Tier 8b. `ilc_fit_lib.goal_of` recovers the goal on the assumption "the cup xy IS the goal xy for an 8a toss"; under the shipped 8b default `aim_site = throw_site` (`reload_coordinator_node::_build_toss_cycle`). The record carries `throw_site_xy_mm` (origin 'D') and the fit ignores it. | **Key the cell on the AIM SITE**, resolved by the same rule the node uses — catch xy under 8a, `throw_site_xy_mm` under 8b. That is bitwise today's key for 8a and correct for 8b, and it is the physically right key either way: the aim residual is a property of the **release pose**. Plus a **zero-displacement admission gate** for v1: refuse rows whose \|throw_site − catch_site\| exceeds the key quantisation, because a displaced toss's aim residual is not the same measurand. Refusing 8b rows outright is not an option — 8b is the shipped default, so that refuses everything.  **LANDED 2026-08-22, one half of it, and the corpus changed the design.** The obvious read of `throw_site_xy_mm` is a **192 mm trap**: the record fills it from `getattr(seq, 'throw_site_xy_mm', (0.0, 0.0))` and `TossSequencer`'s CLASS DEFAULT is `(0.0, 0.0)`, which nothing assigns under 8a — and the 2026-08-12 corpus holds exactly that, three admitted rows declaring `toss_tier: '8a'`, site `[0,0]` and a cup at `(±150, −120)`. So `ilc_fit_lib.throw_site_xy_of` is **TIER-GATED**, `goal_of` carries the site into `TossGoal` (the model half), and `throw_site_admissible` implements the zero-displacement gate at `THROW_SITE_KEY_TOL_MM = POSE_CELL_MM` plus a `throw_site_unknown` refusal for an 8b row with no site. **Still open**: keying the cell on the aim site needs `toss_ilc.goal_key` and the node's lookup to move together, and the gate does not by itself prove site and cup share a cell (74 and 224 mm quantise apart). | Derived | Build step 4c |
| **C8** | V2b/V3/V4 read `temp/probes/*.jsonl` (gitignored) and skip on a clean checkout; the headline numbers have no committed provenance beyond the plan text and `mocap_parity_bias.py --self-check`. | **Partially closed 2026-08-21** by the fold-in's corpus-guard fix (one `_corpus_or_skip_reason()` answering "does this tree hold a corpus the fit can *use*", replacing a presence-only guard that let a stale pre-E-1 mine both fail one test and vacuously pass another). **CLOSED 2026-08-22**: `ilc_fit.py --emit-fixture` projects the admitted rows onto `FIXTURE_FIELDS` and `tests/hardware/ilc_corpus_fixture.py` is the committed result (19 rows × 50 fields, cut the way `tests/ros/possession_fixtures.py` was). The `corpus` fixture PREFERS the live mine and falls back to it, so the corpus-backed assertions RUN on a clean checkout — **13 skips → 3** — and `test_the_committed_fixture_reproduces_the_headline_numbers` re-derives D2's sigma, the +11 %-fast flight-time mean, the C3 disagreement and C6's 16/3 partition split from committed bytes. | Derived | Build step 4d |

### C1's design, spelled out

The faithful transposition of § 3.2 has **three** parts, and only taking all
three keeps both the noise out and the correction in:

1. **Cells carry the spatial residual only.** `config/toss_ilc.yaml`'s per-cell
   `aim_rx/aim_ry` become anchor-referenced:
   `M(P) = û(P) − mean_over_anchor_visits(û(anchor))`. `event_vel_trim` is
   **not** anchor-referenced — the quantity C1 is about is a `level()`
   *orientation* draw, which does not touch release speed.
2. **A top-level `anchor_aim_rad` (+ its evidence count) is persisted as a
   shrinkage PRIOR, not as a hard per-cell correction** — § 3.2's second bullet,
   verbatim in role. This is what keeps the correction's magnitude: the measured
   per-cell \|aim\| is 9.1–10.5 mrad and is **consistent across all three goal
   cells**, i.e. the correction ILC found is dominated by common mode. Persisting
   only spatial residuals and applying nothing else would discard most of a
   correction worth ~36–42 mm at the corpus operating point — against a 35 mm
   capture radius, that is the difference between catching and not.
3. **The session-local component is seeded from that prior, applied, and
   discarded at goal end**, with the trim's own evidence gate transposed
   (`N_MIN_APPLY = 6`, `SE_GATE = 2.5` held over 3 updates, deadband, CUSUM
   freeze, freeze-never-zero). Because each session contributes **one
   independent `level()` draw** to the shrinkage mean, the 1.2–1.7 mrad draw
   enters at `1/(n₀+S)` weight instead of being frozen at one sample.

**The honest limitation, stated rather than papered over:** the component cannot
*update within a session*, because there is no live-admissible observable —
`land_err_mm` is mined, and both live candidates are D5-forbidden
(`toss_trim`'s `no_mocap_fit` refusal is the same wall). So today it is
seeded-and-held: applied from the prior, updated between sessions by the fit.
That is a genuine batch-loop constraint, not a gap in the design, and it is why
the prior has to be persisted for the component to be worth anything.

### Two consequences the fold-in derives, both load-bearing

**(a) `SIGMA_E`'s stale `arrival_dir` entry is promoted from micro-decision to
prerequisite.** D2 was bounded-cost while Q arbitrated two lateral channels:
0.00238 vs the measured 0.00302 rad moved the pooled aim requirement 0.00997 →
0.01044 rad and nothing else. Under decision 6, `arrival_dir` is the **only**
aim channel, so its σ no longer trades off against `land_err`'s — it sets the
aim block's weight against `R = diag(ρ/τ²)` directly, and a **27 % understated
σ over-trusts the sole aim channel and takes correspondingly larger steps**.
Re-derive `SIGMA_E[2:4] = 0.00302` **before** the first artifact is written, and
re-run `conditioning()` / `screen_channels()` under the new mask to confirm the
two aim columns still clear `SCREEN_SNR_MIN = 1.0` on `arrival_dir` alone (they
cleared at 2.92σ with both lateral channels feeding them; a mask that drops two
of five rows will move that number, and the screen must be re-read, not
assumed).

**(b) Decisions 3 and 5 collide at exactly the cadence target, and the fix is
one level up.** At T = 0.4949 s — R5-prime's flight time — the admissible
`k_v − 1` on the **negative** side is `+0.000`, bounded by `ARM_WINDOW`. The
corpus's measured demand is `−0.1076` — a slow-down — so at the cadence target
the channel has **zero authority in the only direction the plant asks for**.

*The mechanism is the counter-intuitive half, and it was probed rather than
argued* (`tools/probes/ilc_speed_band.py`, run 2026-08-21; the first draft of this
paragraph had it wrong). `arm_window_s(T, v) = latest − earliest` with `latest`
a function of **T alone** and `earliest = throw_decel_s(v) +
ARM_SUPPRESS_MARGIN_S`. And `throw_decel_s = INERTIA_RATIO · t_acc` **grows as
the release speed falls** — 0.10488 s at v = 2.440 m/s → 0.11654 s at
v = 2.196 — so a *slow-down* trim pushes `earliest` **later** and **narrows**
the arm window. At the band floor that window is exactly `ARM_WINDOW_MARGIN_S`
(0.0500 s) wide by construction, so **any** negative trim breaks it. It is not
that the achieved flight gets shorter: `evaluate` takes `(T, v)` as independent
arguments and never re-derives a flight time from `v`. Measured band, same run:
`[+0.000, ≥+0.5]` at T = 0.4949 (neg bound `ARM_WINDOW`), `[−0.409, ≥+0.5]` at
0.55, `[≤−0.5, +0.270]` at 0.9032, `+0.148` at 1.00, `+0.043` at 1.10,
`+0.000` at 1.1485 (pos bound `DECEL_FF_HEADROOM` from 0.9032 up).

The one-level-up reading: that demand is not a per-cell learned residual at all,
it is a **plant gain** — the hand throws ~11 % fast, consistently, at every cell
(−0.096 / −0.112 / −0.124, n = 8/6/3). A cell-keyed artifact cannot express a
T-independent gain, and `throw_envelope.evaluate` is a **model-space** gate, so
an 11 % uncalibrated gain means the gate's verdict is 11 % wrong about the
machine. **Pre-registered recommendation for build step 1: fold the re-measured
gain into the model** (the nominal release-speed map), so model and plant agree,
the envelope gate becomes honest, and the ILC trim demand collapses toward zero
everywhere — at which point R5-prime's zero negative authority stops mattering,
because nothing is asking for it. The alternative — keeping the +11 % as an ILC
trim — is rejected here on the evidence that it is inadmissible at the operating
point we have chosen to fly. **Prerequisite either way (C6): re-measure the gain
on the restored drive before acting on it**; +11 % was measured through the
−10.00 A braking clamp, and 0c's own caveat says its verdict is re-checked after
restoration.

### Build ladder (digest § 4.5, with the decisions applied)

| Step | Content | Gate |
|---|---|---|
| **1** | **LANDED 2026-08-21.** **Safety envelope.** `throw_envelope.evaluate(goal.flight_time_s, event_vel)` into `ilc_fit_lib.admit_command` (beside `validate_event_vel`) **and** into the node's apply-seam guard in `_build_toss_cycle`, refusal ⇒ drop the refinement, never kill the goal. Replace the scalar `ILC_SPEED_AUTHORITY` with the envelope-derived T-dependent band (`ilc_fit_lib.speed_authority_band`; 0.15 survives as the outer ceiling). The apply seam gained a **third** check the ladder did not name — the UNTRIMMED goal must clear the envelope too, so layer 3 can never be the difference between a refused goal and a flown one. Still open, and deliberately not built here: re-measure the release-speed gain on the restored drive and decide (a) vs the alternative above (blocked on C6's fresh capture). | `test_the_event_vel_band_is_unreachable_inside_the_speed_authority` fails as written and is rewritten against the envelope; a T-sweep test pins the band at 0.4949 / 0.9032 / 1.1485 s |
| **2a** | **LANDED 2026-08-21.** **C4.** Trim aim estimator → monitor-only (`toss_trim.AIM_AUTHORITY`, stamped into the proposal and into every record as `trim_authority`); its demand arithmetic subtracts `map + ilc` (`toss_trim.ilc_aim_rad`, absent/null ⇒ 0, malformed ⇒ refuse by name); `speed_gain` marked RETIRED. The record gains `trim_monitor_aim_rad` as a **new** key rather than repurposing `trim_aim_rad`, because everything that reconstructs the applied aim reads that key as "what was applied". One fact the demotion derived and the ladder did not anticipate: the D7 clamp's TRUNCATION branch is now **unreachable** without layer 3, since `parse_toss_cal` bounds a map node by the same `TOTAL_MAX_RAD` magnitude `clamp_total_aim` does. | a test with **both** layers enabled pins zero trim authority and an unbiased monitor read-out |
| **2b** | **APPLY HALF LANDED 2026-08-22.** **C1 + C5.** The artifact gains an optional top-level `anchor` block (`aim_rad` + `n` + `se_rad`, all-or-nothing when present, absent ⇒ a *declaration* that the cells are not anchor-referenced ⇒ exactly zero), a **third** parse-time bound on `cell + anchor` — the sum neither half can see, D7's own argument one level down — and `toss_ilc.IlcSessionCommonMode`: RAM only, one per goal, seeded from the anchor at `_toss_trim_begin`, read once in `_toss_aim_for_goal`, discarded at `_toss_trim_end`, and with **no write path at all**. The evidence gate is the trim's, transposed and pinned equal (`ANCHOR_N_MIN = N_MIN_APPLY = 6` on independent `level()` draws, `ANCHOR_SE_GATE = SE_GATE = 2.5` per axis, `ANCHOR_DEADBAND_RAD = DEADBAND_RAD`), each of which carries a measurement nobody should re-derive. C5's composition landed with it: `clamp_total_aim(map + ilc_spatial + ilc_session)`, single D7 clamp final, a hit drops **both** components. Two things the ladder did not name: the common mode is deliberately **not** keyed on a cell hit (a common mode is not a function of the cell — the same thing layer 2 has always done), and there is **no CUSUM and no freeze-never-zero** because both defend an *updating* estimator and this one is seeded-and-held. **Still open: the FIT half** — see C1's row. | apply half: `test_a_session_level_draw_never_reaches_a_PERSISTED_cell` + the section-9 batteries in `tests/motion/test_toss_ilc.py` and `tests/ros/test_toss_ilc_node.py`. Fit half: a two-session synthetic pins that an injected per-session `level()` draw does **not** move any persisted cell |
| **3** | **Ride the guards it says it rides.** Call `toss_trim.admit_for_aim` / `admit_for_speed` from `ilc_fit_lib.admit_record` (design constraint 4's possession gate is currently **unimplemented** in the fit), and port the SE gate + CUSUM + freeze-never-zero as a **per-cell evidence gate on `fit_corpus`**: no step written for a cell whose pooled \|e\| is inside `k·se`; freeze on a detected shift. All reuse; the single largest capability gap. | **LANDED 2026-08-22.** Written literally the step refuses **100 % of the corpus** (`admit_for_aim`: 0 of 53 loaded, 0 of the 19 admitted), because every refusal that bites is a precondition of the *trim's* estimator — `applied_aim_unknown` (ILC never forms `A`), `no_geometry` / `no_flight_pair` (declaration fields `goal_of` already reconstructs), `mocap_fit_quality` (the landing-plane fit, i.e. decision 6's monitor). So the guards are **called and scoped**: `GUARD_SCOPE` classifies every reason `toss_trim` can emit as ROW / LAND_ERR / DECLARATION_GAP / SELF_BLINDING, fails closed on an unknown one, and a completeness test reds if `toss_trim` grows a guard the table has not seen. Two single-definition moves upstream made it possible: `toss_trim.aim_refusals`/`speed_refusals`/`timing_refusals` (non-short-circuiting; the `admit_for_*` trio are now their boolean faces, bit-identical) and `cusum_step`/`cusum_alarmed`/`history_sd` (extracted so the online trim and the batch gate drive ONE recursion). **G3 is waived by name**: the corpus hand throws +11 % fast ⇒ +23 % of apex, so `apex_out_of_band` would refuse the machine's own dominant correctable error — the guard refusing the evidence needed to clear it, which is `admit_for_speed`'s own argument one estimator over. `evidence_gate` is per channel (THIN / INSIDE_SE_GATE / FROZEN_CUSUM) with `se = sd/√n` not `sd/√(n₀+n)` (no prior here), and freeze-never-zero falls out of `u_next = u_prev + du`. | a 3-row wide-scatter cell writes **no** step ✔; plus the freeze-holds-`u_prev` pin, the G2 channel-scoping pin and the reason-vocabulary completeness pin |
| **4** | **Measurement questions.** (a) `DEFAULT_MASK → [0,0,1,1,1]` + the per-toss channel-disagreement log (decision 6's standing validation); (b) `SIGMA_E[2:4] → 0.00302` + re-run the screen; (c) C7's aim-site key + zero-displacement gate. H2 is **retired**, not scheduled. | **LANDED 2026-08-22.** (a) done, with `channel_disagreement` / `disagreement_census` returned from every `fit_corpus`. (b) done — and **D2's bounded cost collapsed to zero in the aim channels**: the 0.00997 → 0.01044 rad figure was entirely `Q` arbitrating `arrival_dir` against `land_err`, and with the monitor mask zeroed each aim column is driven by ONE channel, so the scale cancels out of `(FᵀQF)⁻¹FᵀQe` — measured **0.008717 rad at both sigmas, bit-identical**. What the value still moves is the damped step and the screen's SNR. (c) done as far as v1 allows — see C7's row. A second-order finding: 0.00302 was measured over the 17 rows carrying BOTH lateral channels; now that the primary no longer depends on the monitor, 19 are readable and the statistic reads **0.00286**. `SIGMA_E` keeps 0.00302 — over-stating noise is the conservative direction and this array's own doctrine. | the disagreement log is written for every toss ✔; the screen is re-read under the new mask ✔ |
| **5** | **Re-capture and re-fit.** Fresh corpus on the FW-14 / restored-drive / 8b machine across **≥2 flight-time cells** (kills the gain-vs-offset degeneracy and gives V3 a geometry where `channel_sensitivity` varies); first artifact written with `--declare-toss-cal NONE`; Phase-3 criteria re-frozen against a **no-correction** baseline. | Phase 3's re-frozen criteria below |

**The ball's physical radius** — **LANDED 2026-08-22.**
`physics.juggling_ball_radius_mm` was **35.0** carrying the comment "70 mm
diameter / 2"; the owner's caliper measurement makes it **37.0** (74 mm across),
and the 35 mm was an assumed figure the repo had been repeating. Mass is
unchanged. Three stale sites corrected with it: `generate_mjcf.py`'s comment,
`tests/sim/test_mjcf_drift.py`'s pin (which was **guarding the wrong value** —
the failure a drift test is least able to notice about itself, since both sides
agreed), and `tools/probes/mocap_parity_bias.py`, whose local `BALL_RADIUS_MM`
claimed the value was "NOT in hardware_config.yaml" and cited two prose sources
that both state the cup's **CAPTURE RADIUS** — a different quantity with the same
number. It now reads `hw.JUGGLING_BALL_RADIUS_MM`.

Seven sim tests went red on the two-millimetre correction and **none was a
regression** — see `logbook/2026-08-21-ilc-primary-foldin.md` § "Phase G9". Four
were millimetre pins on a quantity measured to swing **3.7 → 39.9 mm
non-monotonically** across a 33-38 mm radius sweep, and are now bounded by
`juggle_catch.SEAT_RADIUS_MM`, the cup seat radius `CatchResult.clean` is already
defined against; three were nightly BREAK characterisations whose *mode* moved
exactly as their own docstrings predicted it would when the catch contact
changed. Every behavioural assertion (separated / caught / held / tilt engaged /
diverges) is untouched.

## Implementation phase summary

| Phase | Content | Gate | Status |
|---|---|---|---|
| 0 | Measurement substrate: arrival-velocity/flight-time mining (0a), catch-softness probe (0b), release-state backcast (0c), sensor-settle census (0d) | 0b/0d outcomes pre-registered | **DONE 2026-08-12** — 0b → outcome (ii), 0d → outcome (i); results + corrections in the phase text |
| 1 | Sensitivity core + update law + offline replay validation + conditioning-based v1 freeze | F-vs-4hθ identity; held-out prediction; NULL-exit repeatability criterion | **core DONE 2026-08-13** — V1–V4 all PASS; **E-1 resolved AND implemented same day** (whole-arc estimators landed, aim channels live, 3-DOF fit consistent across cells); **Gate 1 CLOSED 2026-08-13** (owner: ±0.15 ILC authority; sizing memo approved; next = Phase 2 dormant) |
| 2 | Production wiring, ship dormant | full suite; byte-identical-OFF; one-apply-point structural test | **DONE 2026-08-13** — `motion/toss_ilc.py` + the `_build_toss_cycle` seam + `JB_OP_TOSS_ILC_ENABLED` default false; 161 tests; results + the artifact-writer semantics in the phase text |
| 3 | Hardware A/B vs a **NO-CORRECTION** baseline (operator) — the "aim map alone" baseline this row used to name was never constructible | criteria re-frozen 2026-08-21 (§ Phase 3) + abort signatures; the absorb-or-keep decision is **pre-empted** by owner decision 4 | **G-1 and G-2 both CLOSED**; now gated on build steps 1–5 |
| 4 | Extensions (sketch): measured-softness closure, platform stroke knots, two-ball follow-through | — | after 3 |

## Implementation phases

### Phase 0 — Measurement substrate *(mining + probes; no live-path changes)*

- **0a — Arrival kinematics.** Extend the toss-record miner: arrival-velocity
  vector at the catch-plane crossing from the Kalman track's descending
  branch, and flight-time error, added to the mined record (field additions
  per the record's own schema rules — superseded detail: see Landed below).
  Pure mining; the D5 live-closure prohibition is untouched.
  **Landed 2026-08-12**: 24 additive origin-'M' fields across four blocks
  (`command_ref`, `arrival`, `backcast`, `split`), plus the
  `usable_for_release_fit` selection flag (origin 'X', added at audit:
  `backcast_fit_n`/`arrival_fit_n` ≥ 20 and `release_vel_se_mms` ≤ 50) —
  **no schema bump**, per the record's own rule (additive fields do not
  bump). Velocities are
  whole-branch fits carrying their own standard errors (`*_vel_se_mms`);
  rolling-window fits proved unstable (±17 % against the bag clock) and were
  rejected. Headline: flight-time error **+101 ms median on a commanded
  903 ms**.
- **0b — Catch-softness probe.** From already-bagged catches, extract
  `hand_telemetry` `vel_meas`/`iq_meas` around the contact instant. The
  probe's first question is a **cadence census, not a noise question**:
  `iq_meas` reaches the bag through the on-change/1 Hz DIAGNOSTIC path and is
  republished from cache, so distinct-value transitions per contact window —
  not samples — are what count, and the on-change refresh thins out exactly
  in the soft-catch regime this plan targets. Then: **is the impact transient
  resolvable, at each channel's true cadence, above the noise floor?**
  Pre-registered outcomes: (i) resolvable → the metric (peak |Δvel| over the
  contact window, or an iq impulse integral) is documented with its measured
  noise floor and admission thresholds; (ii) not resolvable in either channel
  → softness stays modeled-surrogate-only in v1 and a firmware-side
  high-rate capture proposal (Platform Teensy samples at stroke rate) is
  written as a follow-on; (iii) resolvable in `vel_meas` but cadence-starved
  in `iq_meas` → a `vel_meas`-only metric carries v1. The measured channel enters the loop only
  after a probe demonstrates it behaves deterministically (the
  empirical-probe-before-thresholds rule).
  **Hand mocap markers** (owner suggestion, 2026-08-11) are the third option
  in a fixed preference order: (1) hand-drive channels, (2) ballistic
  backcast, (3) markers on the hand — markers only if 0b *and* 0c both fail
  to produce trustworthy contact/release truth, because the install is
  invasive and small-cluster rigid-body tracking on a fast hand is unproven
  here.
  **Run 2026-08-12 — the evidence selects outcome (ii)** (probe:
  `tools/probes/hand_contact_softness.py`). The contact-window `vel_meas`
  transient is 8.8× the still-hand noise floor but **1.03× the matched
  cross-label control** (CAUGHT vs MISSED at identical commanded
  choreography — ball present vs absent), so the apparent transient is the
  commanded stroke, not the ball; the iq impulse is 1.45× under the same
  control (shared disjoint quiet-held baseline; under the significance
  bar). `iq_meas`'s on-change gate does open at contact (13× transition
  rate vs quiet-held) but thins with softness exactly as pre-registered
  (3 distinct values/window in the softest third vs 12 in the harshest).
  Two corrections to this plan's own text from the census: `vel_meas` is
  NOT genuinely 100 Hz (10.5 % duplicate samples bag-wide, ~50 Hz effective
  through the fastest stroke — duplicates alias into exactly the impact
  band), and outcome (iii) named the wrong direction — the observed fourth
  cell (cadence-starved `iq` with the stronger within-cycle signal,
  transient-free `vel`) occurred once and is reported UNREGISTERED by the
  probe rather than rounded into a box. Caveat: every bag predates the G-2
  drive restoration, so the census ran through the braking clamp; 0b's
  verdict is re-checked after restoration — the one thing that could move
  it.
- **0c — Release-state backcast.** Fit the ascending track to the ballistic
  model → measured release-velocity vector and release time; compare against
  the commanded release state. This is the throw critical point's input-side
  truth, and it localizes any throw error to *release* vs *flight* — the
  discriminator everything downstream leans on.
  **Landed 2026-08-12.** The vertical channels are clean and the headline is
  large: **the hand throws ~11 % fast, consistently** (+478 mm/s pooled
  median on a commanded 4436, n = 19 under `usable_for_release_fit` across
  the three mocap-bearing bags; reference bag +473, n = 16; independently
  cross-checked by the measured apex, 4924 vs 4900 mm/s). Release-time error vs the announcement
  is −4.6 ms median — which reframes the 2a "release runs late" reading as a
  sensor-cadence artefact (the sensor departure edge sits +172 ms late at a
  71 ms poll and +95 ms at 54 ms, tracking cadence; mocap is the independent
  truth; addendum recorded in the 2a entry). **The release-vs-flight split
  is NOT clean — the phase's most important negative result**: the lateral
  channels carry a ~±100 mm/s repeatable branch-to-branch velocity artefact,
  magnitude and whole-arc average initially read as a mocap marker ~30 mm
  off the ball centre on a ball spinning ~0.5 rev/s (owner input 2026-08-12
  re-ranked this: the ball is fully tape-covered, no discrete marker exists,
  and the leading hypothesis is now visible-centroid bias — see E-1) — so
  the direction-error channels are not trusted until it is resolved (entry
  condition E-1, Phase 1). A pre-existing miner
  bug also fell out: the descending-branch selector cut at the first
  sub-plane sample, which for a self toss is the ball resting in the cup
  below the plane — `land_xy_global_mm` had been null on every self toss
  ever mined; fixed in the miner, reference bag `usable_for_aim_fit`
  0/31 → 7/31, overturning the 2a "this bag cannot support an aim fit"
  finding (addendum in that entry).
- **0d — Sensor-settle census (the messy-catch score; owner metric,
  2026-08-12).** From the same bags, census `ball_held_raw` / `ball_held`
  around each catch: distinct-sample cadence first (re-verify the 2a
  findings — poll p50 ≈ 71 ms, asymmetric debounce — on the mined windows),
  then the flip statistics in a settle window after arrival. Deliverables:
  the messy-catch score definition (flips within a window W, W fixed from
  the census), its distribution over clean catches vs the 2026-08-10
  reference bag's three known quick-drops (labeled ground truth), and
  admission thresholds. Pre-registered outcomes mirror 0b: (i) the score
  discriminates known-messy from clean at the available cadence → it joins
  the mined record fields and the Phase-3 criteria; (ii) it does not → it
  stays a diagnostic, and a faster sensor poll (firmware change) is written
  up as a follow-on.
  **Run 2026-08-12 — the evidence selects outcome (i)** (probe:
  `tools/probes/hand_sensor_settle.py`). Score = **raw `ball_held_raw`
  flips within W = 0.75 s of the arrival edge**, W derived twice over (the
  flip-median plateau's low end, and the smallest window recalling all
  three reference quick-drops — 0.5 s recalls only 1/3). Pooled 11 bags /
  58 catches at threshold ≥ 1: recall 3/3 on the known quick-drops,
  false-clean 0 %, false-messy 3/55 = 5.5 % — and the three false-messy
  rows all score exactly 2 with 9–12 s possession, i.e. plausibly genuinely
  rattly-but-kept, which is what the owner's framing calls messy. The
  debounced count at W ≥ 1.5 s is the ground-truth label restated and
  proves nothing; only the raw count is informative. Cadence correction to
  this plan's own text: the poll's p50 is NOT a standing 71 ms — per-bag
  p50 spans 20–76 ms and the departure debounce lag tracks it, so
  `sensor_poll_dt_ms_median` is read per record, never assumed.

### Phase 1 — Sensitivity core + offline validation *(no wiring)*

**Entry condition E-1 (added 2026-08-12, from 0c):** the lateral
branch-to-branch velocity artefact (~±100 mm/s on exactly the aim channels)
is resolved before any fit trusts `release_dir_err_rad` or the
arrival-direction error. The vertical channels (release speed, flight time)
are unaffected and usable immediately; a Phase-1 v1 restricted to them (the
`event_vel` trim against the ~11 % fast throw) does not wait on E-1.
**Hypothesis re-ranking (owner input, 2026-08-12):** the balls are
completely covered in retroreflective tape — QTM sees one blob, not a
discrete marker — so the original spinning-marker-offset hypothesis loses
its mechanism. Leading hypothesis is now **visible-centroid bias**: the
tracked point is the centroid of the visible reflective cap, which shifts
with viewing geometry and occlusion along the arc, by up to ~a ball radius
(the fitted ~30 mm offset is of exactly that scale) — and, being
geometry-locked, repeats identically on every toss of the same arc, which
is what 0c observed. The owner has separately observed the extreme form: a
*held* ball bisected into two markers in QTM when a platform strut occludes
cameras at the bottom of the stroke. Magnus remains the alternative.
Discrimination is desk-side on the existing tracks: blob-split events near
the arc ends, residual-vs-height/arc-phase structure (centroid bias is
geometry-correlated and smooth; a spin offset would be periodic), and
whether the branch-velocity delta is locked to the room geometry or to the
ball.
**RESOLVED (mechanism) 2026-08-13 — H-centroid CONFIRMED by parity
decomposition** (probe: `temp/probes/e1_artefact/probe_e1_artefact.py` — the
surviving copy; promotion to `tools/probes/mocap_parity_bias.py`
recommended and pending, since `temp/` is gitignored and these numbers
currently have no committed provenance): a position bias is an EVEN function of time about a
ballistic apex while any aerodynamic force is ODD, and the measured even
part — a smooth height-locked curve, b_y −23.3 mm at z=882 → −2.5 mm at
z=1882 — repeats across 19 arcs, three cup positions and two sittings at
1.45 mm cross-toss sd (pairwise r = 0.998), is room-position-locked, and a
bias profile fitted on **other** arcs collapses the branch delta from
−104.4 mm/s to a median 13.1 mm/s leave-one-out (10.5 mm/s cross-bag) — an
8–10× held-out collapse that transfers across sittings. (Subtracting each
arc's own even part leaves −0.7 ± 1.4 mm/s, but that statistic is
near-tautological by construction and is reported only as numerical
closure.)
Magnus refuted twice (parity + a data-derived aero bound: ~3.9 m/s of
indoor wind required at nominal drag); spin refuted (residual periodicity
40–60× too small); the
owner's bisection observed and quantified (7/50 held-ball windows show two
markers, median 53 mm apart — 91 of 1577 two-marker frames sit under 40 mm;
0 of 2554 in-flight frames show a companion within a ball diameter, 11
within 140 mm). Conventional point markers reconstruct at 0.14–0.15 mm on
the platform pairs and 1.5 mm on the base pair — an order of magnitude
under the artefact either way; the fully-taped sphere is the special case. **Adopted resolution — IMPLEMENTED 2026-08-13: never derive a
lateral/direction quantity from a single branch.** The miner's lateral
velocities at both plane crossings now come from a third, whole-arc fit
(vertical components, crossing instants and SEs stay per-branch — they were
never contaminated); `coverage_asym_s` + `usable_for_lateral_fit` landed
with a 0.1 s refusal, and the parity diagnostic is promoted as
`tools/probes/mocap_parity_bias.py` (reproduces the load-bearing E-1
numbers exactly — the parity verdict, held-out collapse and leak figures;
position-lock rows are re-based per-group and the aero bound recomputed;
committed provenance). Re-mined results: arrival and
release directions now AGREE (one shared lateral velocity — they are no
longer independent measurements, recorded in the field docs), the arrival-y
channel moved +10.3 mrad (the predicted phantom was 10.9), and the lateral
flight term collapsed ~58 → ~6 mm — **lateral landing error is
release-side, measured**. Two honest demotions from the implementation:
`coverage_asym_s` is a gross-truncation guard, NOT a leak predictor (a
0.0003 s row still leaks 6 mm/s — it is a first moment; the ~7 mm/s ≈
1.4 mrad residual is a standing lateral uncertainty, documented at
`COVERAGE_ASYM_MAX_S`); and the wind bound is corrected and weakened —
3.9 m/s at nominal drag, 1.5 m/s at the data-implied 2.6× drag bound (the
first-draft 4.3 traced to nothing in the repo) — so the load-bearing draft
refutations are the two-sitting position-lock and the blob census, not the
magnitude. Height-restricting fit windows does NOT work — the
bias gradient is constant over the whole arc. What the resolution avoids:
the per-branch descending arrival direction carries 10.9 mrad ≈ 44 mm of
phantom aim error per toss (derivation, corrected at audit 2026-08-13: the
descending branch's offset from the whole-arc truth is (Δx +16.2, Δy
−53.3) mm/s, |Δv| ≈ 55.7 mm/s over the ~5.1 m/s arrival speed ≈
10.9 mrad; through the production landing gain 4h+Δz ≈ 4020 mm/rad at the
corpus operating point ≈ 43.8 mm — the first-draft 54 mm used an
apex-above-floor h no production gain produces). **Standing caveat**: only the bias *gradient*
is measured; the absolute offset (bounded ~a ball radius) is not, so any
mocap-closed aim loop converges to the measurement's cup, not the world's —
pre-existing across the whole mocap aim stack. The 2026-08-13 re-mine made
this caveat **numerically visible**: expressed at the plane through the
aim gain, `land_err` (a plane-position fit, carrying b_y(z_plane)
absolutely) and `arrival_dir` (whole-arc, bias-immune) disagree by a
systematic **+18.0 mm in y in every goal cell** (+17.4/+19.5/+15.4; x
agrees to 0.85 mm) — the size of the measured b_y span. The pooled aim fit
is a chi-square compromise between two channels that disagree about the
world; documented in `weight_matrix`/`SIGMA_E`, not papered over. Definitive closure is a
~20-minute no-robot capture: the taped ball fixtured with 3+ conventional
point markers, static at several heights/positions, giving b(x,y,z)
absolutely. (Also: the ball radius is not in `hardware_config.yaml`; the
probe assumed 35 mm — the blob-split separations are the only in-data
measurement of the ball's optical scale.)
> **H2 IS RETIRED — owner pushback, 2026-08-21 (decision 6).** The capture
> proposed in the sentences above cannot be taken with this instrument:
> conventional point markers fixtured to the ball would **corrupt the very
> trackable surface being measured**. QTM sees one blob because the ball is
> fully tape-covered; adding a marker cluster changes that blob, so the
> measurement would be of a different object than the one that flies. The
> owner's mechanism reading — platform-frame occlusion near the bottom of the
> stroke — is consistent with the arc's own data (bias large at z ≈ 880, the
> catch-plane height, vanishing by z ≈ 1880; parity-EVEN), which is the same
> signature the probe already convicted. **C3 is therefore resolved by decision
> rather than by measurement**: `arrival_dir` (whole-arc, bias-immune) is the
> PRIMARY and ONLY aim residual driving the update, `land_err_x/y` demote to
> MONITOR-ONLY (masked out of the aim update — never Q-arbitrated against
> `arrival_dir`, optionally reported bias-corrected through the already-fitted
> parity `b(z)`), and **absolute centering closes through catch outcomes**: the
> catch/penalty trend is the ground truth for "centered on the cup", and a
> residual ~10 mm registration bias against the 35 mm capture radius is
> tolerable and visible there. The standing replacement validation is the
> per-toss channel-disagreement log (Phase 3, criterion P-5). The ball's
> **physical** radius still lands in `config/hardware_config.yaml` as an
> operator-measured constant (calipers), replacing the probe's assumed 35 mm.

Pure-Python core (home decided at the G-3 merge: `tests/hardware/` fit-lib
sibling, promoted into the production package only if the apply path needs
it): finite-difference `F` through the production chain; the damped
trust-region update; the exact-gate re-validation loop. Offline validation,
all four required:

1. `F`'s aim/landing block reproduces `b = 4hθ` within tolerance (pinned test).
2. Closed-loop sign test on the mined corpus — the toss fit's 2c pattern,
   generalized to the vector update.
3. Held-out prediction: the fitted correction predicts a real error reduction
   on records it was not trained on.
4. **Repeatability decision, pre-registered NULL-exit** (the
   `learned-ff-residuals.md` T1.1 pattern): if per-goal inter-toss residual
   correlation on a healthy plant falls below the threshold fixed after 0a
   sizes the noise, the plan NULL-exits — the residual is not dominated by
   repeatable structure and nothing ships. A null is a legitimate outcome.

Gate 1: a sizing memo fixing Q, R, τ, the v1 channel subset (SVD screen), and
the artifact key quantization. Operator checkpoint before wiring.

**Phase-1 core results (2026-08-13; `tests/hardware/ilc_fit_lib.py` +
`ilc_fit.py` CLI + 48 tests — 50 after the same-day E-1 mask lift).** All
four validations PASS:

- **V1**: F's aim block equals the production `aim_landing_jacobian` to
  3.4e-11 relative, and the exact identity is now written down and pinned —
  `dL/dθ = 4h + Δz` with Δz = 6.736 mm (measured 4006.736 vs 4h = 4000.0 at
  h = 1.0). The structure is a 90° rotation, not a scaled identity.
- **V2**: synthetic closed loop recovers an injected 3-channel perturbation
  within 10 % (sign-flipped F pinned to FAIL at ×2.0 residual); the
  real-corpus out-of-channel test fits on flight time alone and cancels
  86.8 % of the release-speed rms (459.0 → 60.8 mm/s), with the model's
  dT/dv = 2/g cross-check agreeing to 4.7 %.
- **V3**: leave-one-out 84.9 % (flight time) / 86.4 % (speed) rms
  reduction; a pure-noise control corpus is pinned < 5 %. Honest caveat,
  asserted in the tests: one commanded flight time only — a single
  operating point cannot separate a launch-speed gain from an offset.
- **V4**: R_rep = 0.9858 / 0.9790 against a derived 0.5 NULL-exit
  threshold — the residual is overwhelmingly repeatable. **No NULL-exit.**

**Conditioning screen**: F is block-diagonal; full-size singular values
[2.921, 2.921, 2.608, 0], condition 1.12 over retained.
`release_timing_offset` is REFUSED: its Jacobian column is **structurally
zero by construction** — in this model a dispatch shift enters only as a
rigid translation of the release instant, and every landing quantity is a
difference of instants that both carry it. The invariance of the landing
point itself rests on the physical argument that the platform holds its
pose through the flight, which the model does not test (audit demotion,
2026-08-13: the zero is a modelling structure, not a measurement of the
planner). The real seam is `_dispatch_toss_throw`'s `release_latency`
shift, and the quantity a dispatch shift does move (`release_time_err_ms`)
is a scheduling error owned by `toss_trim.release_latency_ms` and G-1. Under the E-1 mask the screen
reports `aim_rx/aim_ry` as `e1_blocked` (they clear the floor at 2.92σ
unmasked — closing E-1 is a mask change, not a redesign). **v1 =
{event_vel_trim}, a 1-DOF loop** *(superseded 2026-08-13 — the E-1
resolution lifted the mask the same day; v1 is 3-DOF, see § Post-E-1
additions)*. (Risk 4 did not materialise — the
suspected `event_vel`/timing degeneracy is actually exact orthogonality;
the degeneracy rule stays implemented and test-driven.)

**Gate-1 sizing memo (proposed; operator sign-off required):**
Q = diag(1/σ²) from the measured corpus — σ(land_err) 14.7 mm,
σ(arrival_dir) 2.38 mrad, σ(flight_time) 0.0139 s (raw inter-toss sd,
deliberately not the noise-decomposed 0.0122 — understating noise
over-trusts the channel); `release_speed_err` stays a cross-check channel
(exactly proportional to flight time in-model, dT/dv = 2/g — including both
would double-weight one measurement). R = diag(ρ/τ_j²), ρ = 0.25 in scaled
coordinates (a soft continuation of the screen: 80 % step at s = 1; v1's
channel runs s = 2.61, cost 4 %). τ per iteration: aim 0.005818 rad
(TOTAL_MAX_RAD/3 — full D7 authority within Phase 3's k ≤ 3);
event_vel_trim 0.040 (bracket: floor 0.007 = 2× noise-equivalent command,
ceiling 0.10, binding constraint 0.1076/3 = 0.0359). Artifact key: pose xy
150 mm cells (= the tilt/aim grid nodes, so an ILC cell and an aim-map node
name the same pose for Phase 3's absorb-or-keep), pose z 10 mm (2× the
miner's plane-mismatch tolerance), flight time 50 ms (the gain-vs-offset
discrimination bound ±28 ms, taken with margin).

**Gate 1 CLOSED 2026-08-13 (owner decisions):** (1) the speed-authority
question below is DECIDED as option (a) — an ILC-specific ±0.15 authority,
on the measured rail sweep (neither event_vel rail reachable anywhere in
the flight-time band; `validate_event_vel` still gates every command);
(2) the sizing memo below is APPROVED as proposed; (3) the next unit is the
E-1 whole-arc estimator + Phase-2 production wiring shipped dormant.
Original decision framing, retained for the record:
the pooled corpus requires `event_vel_trim = −0.1076`, which exceeds
`toss_trim.SPEED_AUTHORITY` (±0.10) by 7.6 % — and this is not a pooling
artefact: per goal cell the required trim is −0.096 / −0.112 / −0.124
(n = 8/6/3), so two of three cells exceed the authority on their own.
Options: (a) an ILC-specific authority ±0.15 (the safety argument carries:
`STROKE_TOP_REV` is velocity-independent so a speed trim cannot walk the
hand to the end stop, `validate_event_vel` still gates, and a ±0.15
authority cannot reach either rail anywhere in the sequencer's flight-time
band — at T = 0.55 s, −15 % is 2.30 m/s, 7.7× the 0.3 m/s floor; at
T = 1.10 s, +15 % is 6.21 m/s, still inside the 7.0 m/s ceiling at 1.13×
clear, the binding side); (b) accept clamping at −0.10, which removes
~93 % of the vertical residual and leaves ~+7 ms of flight time; (c)
attribute part of the residual to a commanded-height error rather than a
plant gain.

**Corpus provenance facts the fit refuses to hide**: the 19 usable rows
split 16/3 on `bridge_fw_version` (10 vs 12) — the partition rule refuses
to pool them without an explicit flag; and 16 of 19 were recorded at
**16.7 h of can-bridge uptime**, the exact regime G-1 exists to close. The
G-1 uptime refusal is implemented but ships disabled with a loud census —
any threshold below 16.7 h collapses the corpus to 3 rows, and fixing the
healthy threshold is G-1's job, not the fit library's.

**Follow-ups flagged, not fixed here**: in-tree aim-gain doc drift (three
values for one gain — `toss_fit_lib` 3126.5, `toss_trim` 3126.64, exact
3126.736 — and `aim_target_offset_mm`'s 54.578 mm/deg is a secant at 1°,
not the derivative 54.5718; both correct, not interchangeable). E-1
implementation and the probe promotion: LANDED 2026-08-13 (see § E-1).
**Post-E-1 additions (2026-08-13)**: the fitting mask defaults to full-size
(all five channels; `E1_MASK` retained as a named historical constant) with
`lateral_admissible()` as the single coverage-gate enforcement point;
`ILC_SPEED_AUTHORITY = 0.15` landed per the Gate-1 owner decision (
`toss_trim.SPEED_AUTHORITY` untouched, both pinned by test). The 3-DOF fit
on the re-mined corpus: per-cell aim (rx, ry) consistent across all three
cells (|aim| 0.0091–0.0105 rad, 52.4–60.1 % of the D7 clamp; exceeds the
per-iteration τ0 at 1.57–1.80×, so iteration 0 clips and convergence lands
at iteration 2, inside Phase 3's k ≤ 3 — nothing widened); `event_vel_trim`
−0.1076 unchanged to six decimals (71.7 % of the new authority, worst cell
82.6 %). **One open micro-decision**: `SIGMA_E`'s arrival-dir entry is 27 %
stale under the adopted estimator (measured 0.00302 vs the Gate-1-approved
0.00238 rad); re-deriving an approved weight is an owner call — bounded
cost either way (|aim| 0.00997 → 0.01044 rad, 57.1 % → 59.8 % of
authority).

### Phase 2 — Production wiring *(software only, ship dormant)*

Artifact schema + loader + provenance dormancy (constraint 5); the single
apply seam beside the aim-map lookup in the goal build;
`jugglebot_operational.toss_ilc_enabled` default false + codegen — or a
`reload_coordinator_node` parameter matching the session trim's
`toss_trim_enabled` precedent; the choice is part of the G-3 seam decision.
Tests: byte-identical-OFF; one-apply-point
structural test (the aim map's D4 pattern); gate-revalidation refusal path
driven with a deliberately infeasible Δu; provenance-mismatch dormancy; a
`test_shipped_config_has_the_feature_off` tripwire. Full suite + the
(date, command, result) triple; `--full` if anything under `controller/` or
`sim/` is touched (expected: nothing).

**Phase-2 results (2026-08-13; `jugglebot/motion/toss_ilc.py` + the seam +
161 tests).** Shipped DORMANT behind `jugglebot_operational.
toss_ilc_enabled: false` → `JB_OP_TOSS_ILC_ENABLED` — config + codegen
rather than a node parameter, decided on root cause: this arms a *learned*
correction, so which build applied it must be answerable from git alone
when a corpus fitted under it is read back months later; the
`toss_trim_enabled` parameter precedent does not transfer (RAM-only,
dies with its goal). The Phase-3 baseline arm needs no rebuild:
`$JUGGLEBOT_TOSS_ILC` is authoritative-when-set and an absent target is the
silent zero-correction state. **Composition**: `clamp_total_aim(map + trim
+ ilc)` — the existing D7 clamp stays the final authority; a clamp hit
REFUSES the ILC contribution whole (WARN) and re-composes `map + trim`
exactly as before (risk 5's root cause: a truncated step is not the step
that was solved for; layer-1/2 clamp semantics untouched). The
`event_vel` trim is gated by `validate_event_vel`; a refusal drops the
trim and flies the nominal. Applied values are recorded per toss
(`ilc_aim_rad`, `ilc_vel_trim`, origin 'D', additive — no schema bump);
the record's `event_vel_mps` now reads what `_dispatch_toss_throw`
actually sends. Layer 3 depends on layer 1's dormancy (its provenance
records the applied aim map) while layer 2 does not — a deliberate,
documented asymmetry. **The artifact writer** (`ilc_fit.py
--write-artifact`) emits the accumulated `u` (not the step): `--from-artifact`
seeds `u_prev` per cell and the accumulated vector is re-validated through
`admit_command` — Phase 3's k ≤ 3 loop made real. It REFUSES unprovable
provenance: a mined-only corpus carries `toss_cal_version = None`, and
stamping "no aim map" on evidence that says "nobody wrote it down" would
corrupt the dormancy gate — `--declare-toss-cal/--declare-tilt-map` are
the recorded escape hatch, so **the 2026-08-12 corpus cannot be written
without a declaration** (correct, surfaced, not papered over). Named
follow-ups: an `ilc_version` field in the record would make the corpus
self-describing (today: recoverable via `git_sha` + the committed
artifact); `model_config_identity` covers config, not code (a geometry
change moving no constant will not move the hash — `git_sha` is that
channel); `release_timing_offset` is refused by name in the artifact
schema.

### Phase 3 — Hardware A/B *(operator-run, batch learning between runs)*

> **RE-FROZEN 2026-08-21.** The paragraph below this box is the original
> framing and is **superseded**: its baseline ("aim map alone") was never
> constructible — `config/toss_calibration.yaml` has never existed in either
> tree — and its absorb-or-keep decision is pre-empted by owner decision 4.
> Kept for the record; read the re-freeze.

**Baseline = NO CORRECTION.** No aim map exists, the trim's aim estimator is
monitor-only (decision 1), and `$JUGGLEBOT_TOSS_ILC=0` is authoritative-when-set,
so the baseline arm needs no rebuild and is the silent zero-correction state.
Treatment = ILC applied. Same session structure, matched approach history, same
goal cells, same flight-time cells.

**Sequencing — criteria are frozen from the baseline's own scatter, before the
first treatment run.** The historical corpus cannot supply them (C6: a fresh
corpus cannot legitimately pool with pre-FW-14 / clamped-drive rows), so the
order is: capture baseline → freeze the numbers below from *that* capture →
run treatment. Freezing thresholds from a corpus the machine no longer matches
would be pre-registration theatre.

| # | Criterion | How its number is derived (PROVISIONAL until the baseline capture) |
|---|---|---|
| **P-1** | **Primary**: rms \|`arrival_dir`\| falls at k ≤ 3 iterations. `land_err` is **not** in the criterion — it is monitor-only (decision 6). | Target = the V3 leave-one-out prediction (84.9 % / 86.4 % rms reduction) taken at its lower bound, floored at the per-cell repeatability floor `R_rep` implies. A treatment that beats the baseline by less than the baseline's own inter-session scatter has shown nothing. |
| **P-2** | **Possession non-regression**: catch rate must not fall. | Baseline catch rate ± the binomial CI at the session's n. This is also where **absolute** centering is judged (decision 6): a residual ~10 mm registration bias against the 35 mm capture radius is tolerable, and if it is not, the penalty trend is where it shows. |
| **P-3** | **Messy-catch non-regression**: the 0d score (raw `ball_held_raw` flips within W = 0.75 s of the arrival edge, threshold ≥ 1) must not increase. | Already measured and ready: recall 3/3 on the known quick-drops, false-clean 0 %, false-messy 3/55. No new derivation needed. |
| **P-4** | **Non-increasing actuation variance** — the commanded aim must not churn. | Per-goal sd of `total_aim_rad`, baseline vs treatment. |
| **P-5** | **Channel agreement** (decision 6's standing replacement for H2): the per-toss disagreement log shows the plane residual holding the known `b(z)` profile shape while the `arrival_dir`-driven loop converges. | Shape check against the fitted parity profile, not a threshold. **If catch rate plateaus with converged aim, the model is wrong and C3 re-opens.** |

**Abort signatures** (unchanged in spirit, sharpened): any iteration that
worsens the composite error ⇒ freeze learning, revert to the last-good
artifact; trust-region saturation (`MAX_TRUST_SHRINKS` reached, or τ clipping
on every cell) ⇒ stop and resize offline; any possession-rate drop ⇒ stop; a
**growing** learned magnitude session-over-session ⇒ plant investigation, never
a bigger correction (design constraint 6).

**Corpus requirement**: ≥ 2 flight-time cells. One commanded flight time cannot
separate a launch-speed gain from an offset — that degeneracy is exactly why
`FLIGHT_TIME_CELL_S = 0.050` exists (the ±28 ms discrimination bound), and V3's
`channel_sensitivity` needs a geometry where it actually varies.

*Superseded original:* Baseline = aim map alone; treatment = aim map + ILC
vector, same session structure, matched approach history. Success criterion and
abort signatures are frozen at Phase-2 close, before the first run; the criterion
is a composite-task-error reduction target at k ≤ 3 iterations with no
possession-rate regression and non-increasing actuation variance. The
absorb-or-keep decision on the aim map's update law is made here, on evidence.

### Phase 4 — Extensions *(sketch, not committed)*

Measured-softness closure (if 0b outcome (ii), after the firmware capture
lands); platform stroke-shape knots as added u-channels under the same gates;
the two-ball follow-through cost — null-space regularization toward
recovery-readiness for the next catch, the paper's follow-through idea in its
natural Jugglebot home.

## Testing plan

- Every phase closes with `./run_tests.sh` and a logbook entry; test-count
  claims carry the (date, command, result) triple.
- Thresholds and failure-mode drivers are probe-verified before the test
  asserting them is written (0b is itself the probe for the softness
  channel); confirmed recipes live in the test docstrings.
- The Phase-3 A/B is the load-bearing hardware verification; its criteria are
  pre-registered in the plan before the first run, per the phase text.

## Out of scope

- Online / in-flight adaptation of any kind — the loop is batch by
  construction (constraint 2).
- The dormant MPC chain; leg gains (FROZEN); tracking-level learning
  (`learned-ff-residuals.md` owns that lane); the tilt map (layer 0) and its
  capture process; BB-side aim.
- Clubs / rings — noted as the arc where a genuinely dynamic object model (the
  paper's full machinery, model rollout + differentiation) becomes necessary;
  nothing in v1 precludes it, and the critical-point framing is the same.

## Risks (ranked)

1. **Learning a moving plant** (the uptime latency drift). G-1, plus the
   per-record uptime refusal, plus the magnitude-monotonicity alarm
   (constraint 6).
2. **Softness channel unmeasurable at the available cadence** (`iq_meas` is
   on-change/1 Hz-refreshed, not truly 100 Hz). Pre-registered 0b outcomes;
   the modeled surrogate carries v1 either way.
3. **Double-counting against layers 0–2.** Provenance keys + applied values
   recorded per toss + dormancy on mismatch (constraint 5). **SHARPENED
   2026-08-21 — this risk was live, in both directions (C4), and provenance
   keys were never going to catch it**: with both layers applied the trim
   subtracts `map_aim_rad` only and the machine over-aims by the ILC
   contribution while the trim reports CONVERGED; mirrored, a converged trim
   makes ILC's residual read `J·ILC_prev` and the artifact unlearns itself to
   zero. Closed by decision 1 (one converging estimator per quantity: the trim's
   aim goes to zero authority) plus the monitor's arithmetic subtracting
   `map + ilc`. Provenance keys are also blind to a re-`level` by design (D3),
   which is C1 — closed separately by the session-local component.
4. **Conditioning** — coupled channels (e.g. `event_vel` vs release timing)
   may be near-degenerate in `F`. The Phase-1 SVD screen excludes degenerate
   channels from v1 rather than regularizing them into noise.
5. **Gate-clamp interaction** — a Δu partially truncated by the
   `ThrowTiltInfeasible` clamp would desynchronize applied-u from recorded-u.
   The apply path records the POST-GATE u (the toss-record convention), so
   the learner always sees what the machine actually did.
6. **Corpus contamination by non-possession tosses.** The sensor-truth
   admission gate (constraint 4) filters them; the mocap verdict is never
   consulted.

## Notes for collaborators

- **Workspace**: branch `critical-point-ilc`, worktree
  `/home/jetson/Desktop/Jugglebot-ilc`, base `5e046cc` (the tilt-cal arc
  wrap-up — the last commit before the catch-robustness/toss-selftuning arc
  (`2328d0a`) and the bridge-temporal arc (`9099d1e`) began; owner-directed
  base, 2026-08-11). Consequence: modules **and design-document sections**
  this plan cites from those arcs — the per-toss record, the miner, the aim
  map/trim, the sensor-source merge, and every `§ F*` / `§ D*` / phase-`2*`
  reference, all of which resolve to `plans/active/toss-selftuning.md` — are
  **not in this tree** until the G-3 merge; their cited state is `7cb818d`
  on `mvp-trajectory-bringup`.
- The method reference and its code map: arXiv:2602.21302 §§ IV-B/C/G
  (Algorithm 1, critical-point objective, inverse-model QP);
  `flying_knots_public` — `main/learning.py`, `simulation/inverse_model.py`,
  `simulation/forward_model.py`. The adaptation table above is the contract
  for what is and is not being imported.
