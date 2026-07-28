---
title: Catch-reach near-degenerate overshoot — a 0.78° target produced a 2.32° excursion
created: 2026-07-25
status: active
related_logbook:
  - 2026-07-25-toss-phase3-trace-validated.md
  - 2026-07-24-phase7-fourth-sitting-openloop-telemetry-ladders.md
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py::build_catch
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py::_on_dynamic_target
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py::_republish_pretilt
---

# Plan — Catch-reach near-degenerate overshoot

**Branch:** `mvp-trajectory-bringup`
**Covers:** fix item 8 from the 2026-07-25 self-toss anomaly investigation.
**Sibling plans:** `levelling-frame-contract.md` (items 1–2),
`hand-command-continuity.md` (3–6), `fk-convergence-tolerance.md` (7).

> **Priority re-rated 2026-07-26 (Phase 0), from "lowest of the four" to LIVE.**
> The original rating assumed `levelling-frame-contract.md` removes the trigger.
> It does not. C-LEVEL-1 makes the *park* gravity-level; the through-seat aim in
> `build_catch` still reads a **plan-frame** tilt as "the receive tilt", so with a
> correction loaded every catch — including the shipping reload path — engages the
> through-seat along the correction. Phase 0 reproduced all **seven** catch reaches
> in the reference session offline (five self-toss pre-tilts and both reload
> pre-tilts) and showed they are the same mechanism at two amplifications. Post
> C-LEVEL-1 the toss pre-tilt's requested tilt shrinks toward zero, and the
> amplification is **inversely proportional to the requested tilt** — so the fix to
> plan 1 makes this worse, not better. The defect is pinned (deliberately unfixed)
> by `tests/ros/test_levelling_frame.py::test_catch_through_seat_aims_off_the_gravity_referenced_receive_tilt`.

## Context

### The anomaly

While diagnosing the pre-throw platform tilt, the commanded platform pose (FK of
`/leg_setpoint_echo`, bag `~/Desktop/rosbags/2026-07-25_15-17-48/`) did something a
single rest-to-**rest** quintic cannot do. Toss #4 (the 2nd of five *self*-thrown
announcements in the bag — `catch_reach_replay.py --toss 2`), times relative to the
scheduled release at 1784956866.88:

| t rel. release | commanded `rx` |
|---|---|
| −4.23 s | +0.0044° |
| −1.98 s | **+2.3204°** (peak) |
| −0.92 s | ~0° (crossing) |
| −0.50 s → +2.00 s | **−1.0784°**, flat |
| +3.92 s | 0.0000° |

The catch target it was reaching for, once the levelling correction is applied at
ingest, is `rx = −0.77878414°` (verified numerically against the session's
published offset `[0.013592347421588673, 0.001207157476773584]`).

So a **−0.78° target produced a +2.32° excursion in the opposite direction first**,
then settled and held at **−1.0784° = 1.385× the target** — a value sitting between
a single application of the correction (−0.7788°) and a double one (−1.5576°),
matching neither.

There is also a sharp event at release − 0.6 s: `/trajectory/diagnostics`
`realized_peak_leg_acc_mmps2` jumps 13.9 → 138.1 and
`realized_peak_leg_jerk_mmps3` → 2403, right where the tilt goes flat.

### What is NOT the explanation

- **Not a large pre-tilt.** The reload leg's real BB receive pre-tilt in the *same
  session* is clean: monotonic to `rx = +1.8235°, ry = −10.9330°` (11.08° total)
  with the matching swing-compensated translation to `(11.88, 2.87, 171.16)`, held,
  then unwound to exactly zero. No overshoot, no reversal.
  > **Correction, 2026-07-26 (Phase 0).** The observation stands; the inference
  > drawn from it — *"so this is not a general property of `build_catch` — it is
  > specific to the near-degenerate case"* — does **not**. It is exactly the same
  > property of `build_catch`, at 1/22 the amplification: the reload settle
  > `+1.8235 / −10.9330°` **is** the through-seat overshoot (target × 1.0279),
  > not the target. What is degenerate-specific is the *ratio*, not the mechanism.
  > See `(16/81)·rate·T/|tilt|` below.
- **Not multiple wire targets.** Only one `catch/dynamic_target` is published in
  the excursion window; the ~39 Hz republish burst starts only after release.
- **Not the lean shaper.** `lean_gain` reads `0.00` for the active plan throughout
  (it is reset to 0 on every non-follower install), and there is no lateral
  translation to lean into — x/y stay at 0.00 mm.

### The weakest link in the above — SETTLED 2026-07-26 (Phase 0)

> *Original text, kept because the reasoning matters:* My inference that **one**
> plan spanned the whole excursion rests on two soft signals: `move_seq` staying at
> 48, and `realized_peak_leg_*` ramping monotonically without a reset. But
> `peak_leg_vel/acc/jerk` (the *planned* peaks) read identically `14.2 / 142.4 /
> 3950` both **before** and **after** the install, which strongly suggests that
> field is stale or cached rather than per-plan. If it is stale, my "single
> install" reading may be wrong and the excursion could be a sequence of re-plans
> after all. **Phase 0 must settle this rather than inherit my conclusion.**

**Verdict: the single-install reading is CORRECT, and the staleness worry is
refuted.** Settled from raw messages by `tools/probes/catch_reach_replay.py`, not
inherited.

- **One install.** Three load-bearing signals agree over the whole plan window:
  `move_seq` holds at 48 (a monotonic counter, so a 5 Hz sample still catches
  every `_install`); `plan_kind` is `move` at every sample (a `HoldPlan` reads
  `hold`, which is what exposes the emitter step backstop, the escalation hold
  and the freeze-in-place — the three paths that install by direct assignment and
  bump nothing); and there are zero accepted `target_feedback` events and zero
  `catch/dynamic_target` messages inside the window. All five self-tosses in the
  session score the same way.
  > **Correction, 2026-07-26 (review).** This bullet originally called a fourth
  > statistic — `t + plan_time_remaining_s` drifting **1.9 ms across 21 status
  > samples** — *decisive*, on the reasoning that an install resets the plan-time
  > origin. Both halves of that were wrong and the correction is worth keeping,
  > because the wrong version is the kind a future session would inherit.
  > (a) The statistic is **structurally blind to the very hypothesis it was
  > credited with refuting**: `_plan_and_install_catch` computes
  > `lead = arrival_perf − perf_counter()` and installs at `t0 = perf_counter()`,
  > so the plan end is `arrival_perf + tilt_decay + settle_hold` *no matter when
  > the install happened* — a re-plan to the same absolute arrival (hypothesis 1
  > below, verbatim) leaves it untouched. The same bag supplies the
  > counterexample: the post-release republish burst runs `move_seq` 53 → 64 with
  > 14 accepted feedbacks across release +0.10…+0.70 s, and the plan end drifts
  > **1.02 ms — less than the 1.9 ms across the genuinely zero-install window.**
  > (b) There is no "status rate vs 5 Hz" advantage: one 5 Hz timer
  > (`trajectory_node.py:478 → _publish_status`) publishes *both* topics, 1453
  > messages each over the 290.6 s session. Plan-end drift is kept in the census
  > as one clause of five — it does catch installs that move the plan *end* — but
  > it is not the reason to believe the verdict.
- **`peak_leg_*` is per-plan, not stale.** It is written at every install that
  carries a `FeasibilityReport`; in this very window the preceding `go_to_pose`
  install wrote `0.0 / 0.0 / 0` and the catch install wrote `14.2 / 142.4 / 3950`,
  **identical to the rebuilt plan's report at the published precision** (the
  publisher formats vel/acc to `%.1f` and jerk to `%.0f`; rebuilt
  `14.2401 / 142.4440 / 3949.7`, so bit-identity is not verifiable from a bag in
  principle). That `0.0 / 0.0 / 0` one install earlier is the decisive half — the
  field is *written*, not carried.
  The values repeat **across these particular catch installs** because a catch
  plan's predicted peaks are the fixed-shape 0.15 s decay's peaks *whenever the
  reach is small enough for the decay to dominate*. That qualifier is
  load-bearing: a reload-sized reach is **not** lead-invariant — measured
  2026-07-26 through the production planner, its `peak_leg_vel_mmps` runs
  `82.5 → 30.1 → 24.7 → 15.2 → 14.0 mm/s` at leads `0.8 / 2.0 / 2.371 / 3.707 /
  8.0 s`, and the bag agrees (`move_seq` 4 and 15, the two reload catch installs,
  published `23.6` and `23.8`). So predicted peaks *moving* between two catch
  installs is normal and is **not** evidence that a non-catch plan came in
  between. Both halves pinned in
  `tests/motion/test_catch_reach_replay_probe.py`.
- **There IS a narrow staleness hole, and it is now annotated in the code.** Six
  install paths bump `move_seq` without writing the field — `_svc_hold`,
  `_svc_go_home`, `_install_guard_descent`, `_retry_pending_stop`,
  `_install_graceful_stop` and the follower's input-loss stop — so after any of
  those, `peak_leg_*` describes the superseded plan. None of them ran in this
  window. Annotated at both `trajectory_node.__init__` and the
  `_publish_status` publisher rather than fixed: the fix requires reordering
  `_svc_go_to_pose`'s write to *after* its `_install` call (it currently writes
  before), which is a behavioural change to a safety-adjacent install path and
  does not belong in an analysis phase.

## Why it still matters after the levelling fix

Once `go_to_pose` and `catch/dynamic_target` agree on "level", the toss's pre-tilt
target equals the held pose exactly and the plan is genuinely degenerate — the
un-levelled 15:04:35 session is that case and its commanded `rx` is flat to ±0.05°.
So the trigger is gone for the vertical toss.

But a *small* non-zero receive tilt is a normal operating point, not an exotic one:
a self-tossed ball that drifts a few millimetres produces a receive tilt of a
fraction of a degree, and `compute_catch_orientation` will duly ask for it. If a
0.78° request can become a 2.3° excursion peaking in the wrong direction, then the
catch reach has a regime where small corrections are amplified — and it would do it
0.7 s before a catch. That is the risk being retired here.

**Phase 0 sharpened this into one number.** The amplification is

    peak UNREQUESTED excursion / requested tilt displacement
        = (16/81) · rate · T / |tilt|

identical on both tilt axes (excursion and displacement carry the same `|tdir_i|`
factor), depending on the tilt *direction* and the lead but **never on the tilt
magnitude**. So it is inversely proportional to how much tilt was asked for.
Measured through `tools/probes/catch_reach_replay.py` on the reference session:
**3.76** for the 0.78° toss pre-tilt (lead 3.71 s) against **0.17** for the 11.08°
reload pre-tilt (lead 2.37 s) — one code path, 22× apart. The commanded tilt leaves
the park in the *wrong direction* once the ratio exceeds `ψ(2/3) = 0.790`: for the
toss target, any lead beyond 0.78 s; for the reload target, beyond 10.75 s.

> **Correction, 2026-07-26 (Phase 2).** The `0.790` threshold in the sentence
> above is **wrong, and wrong in the permissive direction** — kept visible
> because a future reader will meet the number in earlier artefacts. `ψ(2/3) =
> 0.790` is where the value **at `s = 2/3`** crosses zero, not where the reach
> first leaves the park on the far side from its target. The true first crossing
> is `min_s ψ(s)/|φ(s)| = 5/2`, i.e. **`40/81 = 0.4938`** in these amplification
> units — measured over a 4M-point sweep, `2.500000000625` at `s = 1e-9`, with
> the wrong-side excursion exactly `0` at 2.50 and `+1.6e-8` at 2.51. So the
> published figure is **1.6× too permissive**: sizing a gate off 0.790 passes a
> reach that has already reversed.
>
> The corrected leads, from the same measured amplification slopes: the toss
> target reverses beyond **0.487 s** (not 0.78 s), the reload target beyond
> **6.72 s** (not 10.75 s). Normative home is
> `ros_ws/docs/catch_arrival_contract.md`; enforced at
> `planner._CATCH_ARRIVAL_RATE_BOUND`.

This is why the levelling fix does not retire the risk. C-LEVEL-1 drives the toss's
requested tilt *toward zero*, and the amplification goes as `1/|tilt|`.

## Implementation Phase Summary

| Phase | Scope | Gate | Status |
|---|---|---|---|
| 0 | Offline reproduction from the recorded session | the excursion reproduced from recorded inputs | **DONE** (2026-07-26) — all 7 catch reaches reproduced; `tools/probes/catch_reach_replay.py`; see Outcome |
| 1 | Root cause, written | mechanism traced end-to-end in prose | **DONE** (2026-07-26) — folded into Phase 0; all three features traced, blast radius stated; see Outcome |
| 2 | Fix + invariant + test | full pytest | **DONE** (2026-07-26) — C-CATCH-1 landed (`ros_ws/docs/catch_arrival_contract.md`), enforced at `planner._catch_arrival_rate`; `pytest tests/ -q`, run 2026-07-26 on the Jetson in the project venv: **3543 passed, 3 xfailed in 1376.13 s (22:56)**; see Outcome |
| 3 | The manufactured seat rate ships at **zero** (operator decision, added after Phase 2) | full pytest + all 7 recorded reaches still `REPRODUCED` + a mutation test proving the C-CATCH-1 block is not vacuous | **DONE** (2026-07-26) — `pytest tests/ -q`: **3574 passed, 3 xfailed in 1382.00 s (23:02)**; mutation test: both bound halves removed ⇒ 5 of 7 `ccatch1` tests fail, 2 survivors are negative cases by construction; see Outcome |
| 4 | Hardware validation (operator-run) — **the seat-removal EXPERIMENT, not a regression check** | reload catch **RATE ≥ 0.63** (≥ 8/12, ≥ 12/19 — the sitting mandates only `n ≥ 12`, so score the rate; **≤ 0.58 aborts, strictly between is INCONCLUSIVE**), bounce-outs **≤ 1**; level-catch reach flat/monotone with settle on target to ±0.05°; self-toss tracker catch error **< 10 mm** | **RUN 2026-07-27 — `ZSEAT-2` ABORTED; the reach rows VALIDATED.** Verdicts in `logbook/2026-07-28-anomaly-fixes-validation-sitting.md`. **`ZSEAT-2` net ABORT.** Its **rate** arm PASSES at **13 caught / 16 attempted = 0.8125** (≥ 0.63), numerator confirmed independently of the operator by a mocap floor census; its **flatness** arm PASSES (commanded tilt `span_deg = 0.0000`, position span `0.00 mm` over the last 0.8 s before every landing measured; the installed `planner.py:837` carries `0.0` with a pre-session mtime) — so **the experiment did measure the zero seat**. But its **bounce-out** arm trips **BOTH** ABORT clauses: **3 bounce-outs, all 3 consecutive** (attempts 1–3). Classification is trace inference — balls 6 and 9 **HIGH** confidence (contact tracked continuously: ball 6's descent rate collapses `−5548 → −1780 mm/s` with no ballistic explanation; ball 9 **dwells 130 ms at z≈705–735** against a 687 mm seated rest, then slides out), ball 11 **MEDIUM-HIGH** (contact occluded; it reappears **345 mm above** the free-fall prediction and its floor-arrival time budget cannot be reconciled with free fall). **Zero missed-arrivals**: all 16 balls arrived within ~40 mm of the cup axis, so this is not BB scatter. **ATTRIBUTION IS INCONCLUSIVE** and must not be logged as a clean confirmation: the same capture holds a competing measured cause — the three drops arrived **~26–39 mm further +x** than the thirteen catches (`x@z1000` drops mean **−8.5** vs catches mean **−34.9**, sd 8.9), a **monotonic BB warm-up drift** plateauing around the sixth throw, with the platform pose repeatable to **< 0.5 mm**. That drift is the proximate trigger but **does not exonerate the zero seat**: a ~30 mm off-centre arrival on a stationary 10.8° rim *is* the disturbance the seat existed to reject, and the marginality is measured — **attempt 3 dropped at `x = −10.7` and attempt 4 caught at `x = −14.5`, 3.8 mm apart**. This capture contains **no throws at ~+30 mm offset with a non-zero seat**, so the two cannot be separated from it. Three robot-side alternatives were **refuted** with measured (not acked) telemetry: prime/stroke identical on all 16 attempts, stroke timing inside the catch distribution, install not stale. **The A/B that settles it**: burn in the BB (~6 throws, robot parked, arrival `x` plateaued to < 10 mm successive change) *before* counting attempt 1; then ~6 throws biased **+30 mm in x** at seat rate `0.0`, then the same 6 at `0.07 rad/s`. **ANSWERED 2026-07-28 — asked, and it confirms the classification.** The operator eye-witnessed all three opening reload attempts and reports every one **touched the rim and bounced out**, so ball 11's `MEDIUM-HIGH` no longer rests on an occluded contact and all three are eye-witnessed rather than inferred. They add that the throws were **visibly off during those attempts and it resolved afterwards** — an independent eye-witness match to the measured BB warm-up drift, which is therefore the operator-corroborated **proximate trigger**. **The attribution verdict does NOT change and stays INCONCLUSIVE**: what the testimony settles is *that they bounced out* and *what put them off-centre*; what remains open is the **capture-basin width** — whether a non-zero seat would have caught a ball ~30 mm off-centre that the zero seat dropped. That is untouched by any of it, and the A/B below is the only thing that can answer it. See `logbook/2026-07-28-anomaly-fixes-validation-sitting.md` § Discussion → *Operator testimony and decisions*. **VALIDATED**: `CCATCH-2` rows 1/2/3/5 (wrong-side excursion **0.0000°** against a model peak of `+1.90…+2.24°`; settle `|err| ≤ 0.0001°`; residual vs gravity **0.0000°**, with the counterfactual putting **+0.297780°** of removed through-seat residual on the record; `peak_leg_*` published `0.0` where pre-fix was ≈142 / ≈3950), `CCATCH-5` (18 of 209 `move_seq` values non-zero, cleanly bounded), `ZSEAT-1` (10/10 OK), `ZSEAT-4` (35/35 stage 6, and **11/11 pre-release exactly still on Tier 8b** where the deferred reach genuinely fires, dispatch `+0.013…+0.050 s` after release, never early). **NOT SCORABLE / NOT RUN**: `CCATCH-2` row 4 (segment count unreadable from the bag — all observables consistent with 2); **`CATCH-2`** — `catch_reach_replay.py:354` pins `THROUGH_SEAT_RATE_RADPS = 0.07` as a capture record and deliberately rebuilds the **pre-fix** plan, so it reads `NOT-REPRODUCED` on any post-2026-07-26 capture; **this blocks `CCATCH-3` and `ZSEAT-3` too** until the probe gains a session-rate override; `CCATCH-4` (no `go_home`/`go_to_pose` was issued anywhere in the sitting); `CCATCH-2t` (tracker corruption confined to **Z**, so the lateral half is recoverable at **0–4 mm** but was not formally scored). `ZSEAT-3` flagged, not scored: the commanded reload settle `ry −10.6823°` is **outside** its ±0.02° band by −0.0460°. Score any CAP-WORK replay from a **clean 8a worktree** — the build crossed `toss_tier 8a → 8b` at 15:59, after CAP-WORK. **Since 2026-07-28 that is no longer `HEAD`**: the 8b flip is committed and `HEAD` is an 8b tree, so check out **`6641400`** (the last commit before the flip) for any such replay — `git worktree add ../jb-8a 6641400`. Run it from `tests/hardware/session_anomaly_fixes.md` § THE RUN SHEET (stage 6 rows CCATCH-2, CCATCH-2t, CCATCH-3, CCATCH-4, CCATCH-5, ZSEAT-2, ZSEAT-3, ZSEAT-4, CATCH-2). **ZSEAT-2 was the only genuinely open question in the whole run**: removing the seat leaves an 11.08°-tilted reload rim stationary at ball contact, which is exactly the geometry the bb-sim finding says deflects a ball. Its ABORT path is a one-line default change, with C-CATCH-1 already in force to bound whatever replaces it |

## Implementation Phases

### Phase 0 — Reproduce it offline

No production code changes. The goal is a deterministic offline reproduction, so
the root cause can be found by reading rather than by hardware sittings.

1. Establish the ground truth of what was installed. From the bag, extract the
   complete `/trajectory/status` and `/trajectory/diagnostics` series across the
   excursion (not decimated) plus `/trajectory/target_feedback`, and settle:
   how many plan installs occurred, what `plan_kind` each was, and whether
   `peak_leg_*` is per-plan or stale. Fix or annotate the stale field as a
   by-product — a diagnostic that reports a previous plan's peaks is a trap for the
   next investigation.
2. Replay the recorded inputs through the production layer: seed
   `build_catch` with the pose/twist/accel that `_current_state()` would have
   returned at install time and the corrected target, and compare the sampled
   `rx(t)` against the FK-reconstructed commanded `rx(t)`.
3. This is a reusable replay harness (it is the same shape as the offline-faithful
   replay pattern that has already paid off once), so it belongs in
   `tools/probes/` with a README entry and outputs to `temp/probes/` — not `/tmp`.

Candidate hypotheses to discriminate, roughly in order of cheapness:

- **stale-diagnostic illusion** — there were several installs, each re-seeded with
  the previous plan's live twist/accel and a *shrinking* duration to a fixed
  absolute arrival; C2 continuity across those turns a small offset into a
  ringing approach;
- **non-zero seed boundary conditions** — `state0` carried twist/accel that made
  the quintic bulge (a quintic with `p0 ≈ p1` but `v0 ≠ 0` does exactly this);
- **freeze/stop interaction** — the reach-freeze window or a graceful stop
  truncated the plan part-way, which would explain both the flat hold at a value
  matching no clean multiple *and* the acceleration spike at release − 0.6 s;
- **frame composition at the endpoints** — the seed and target rotvecs derived
  through different compositions, so the interpolation endpoints are not what they
  appear to be.

**Gate:** the excursion is reproduced offline from recorded inputs, within the
resolution of the 33 Hz setpoint echo. If it cannot be reproduced, that is itself
the finding — write it up and stop; do not fix by guesswork.

#### Outcome — 2026-07-26: GATE MET

Harness: `tools/probes/catch_reach_replay.py` (self-check guarded in CI by
`tests/motion/test_catch_reach_replay_probe.py`). Seeded only from recorded
inputs — FK of `/leg_setpoint_echo` for the seed pose, `levelling.correct_pose`
of the `catch/dynamic_target` wire pose for the target, and
`landing_time − _PRETILT_EARLY_S` for the arrival. One scalar is fitted (the
emit → Teensy → bridge → echo → record pipeline lag); the zero-lag residual is
reported next to it.

Measured 2026-07-26, `python tools/probes/catch_reach_replay.py --bag
~/Desktop/rosbags/2026-07-25_15-17-48 --toss {1..5}` and `--thrower ball_butler
--toss {1,2}`:

| reach | lead | rx rms / max | ry rms / max | echo lag | verdict |
|---|---|---|---|---|---|
| self-toss ×5 | 3.705–3.720 s | 0.0049–0.0062° / ≤ 0.0391° | ≤ 0.0006° / ≤ 0.0035° | +10.0 … +11.0 ms | REPRODUCED |
| reload ×2 | 2.370–2.371 s | 0.0021–0.0025° / ≤ 0.0093° | ≤ 0.0149° / ≤ 0.0558° | +9.5 ms | REPRODUCED |

(The reload's larger `ry` residual is proportionate: its `ry` excursion is 10.93°
against the toss's 0.78° in `rx`, so 0.056° is 0.5 % — which is why the harness's
tolerance is `max(--tol-deg, --tol-frac × commanded span)` rather than a flat
absolute bound.)

Corroborating, none of it fitted: the plan end predicted from the recorded
arrival lands within **1 ms** of `t + plan_time_remaining_s` measured off
`/trajectory/status`; the rebuilt plan's `FeasibilityReport` peaks are
**identical to the published `peak_leg_*` at the published precision**
(`14.2401 / 142.4440 / 3949.7` → `14.2 / 142.4 / 3950`); and the fitted lag is positive
and under half a 25 ms knot, as physics requires. Pushing the assumed install
instant later (toward the accepted-feedback log time) drives the fitted lag
*negative* — the echo cannot precede the command — which is what pins the install
to within a few ms of its triggering message, matching the single-digit-ms
install latency the node documents.

**Outcome, 2026-07-26 — LANDED.** Commit `a680298` (harness, tests, annotation, plan,
runbooks; SHA backfilled in the immediate follow-up). Logbook:
`logbook/2026-07-25-catch-reach-overshoot-repro.md`. Full suite, run 2026-07-26:
`source ~/Desktop/PDJ_venv/venv/bin/activate && python -m pytest tests/ -q` →
**3527 passed, 3 xfailed in 1367.68 s (0:22:47)** (+10 passed against the `0c0c829`
baseline of 3517; xfail unchanged at 3). Two-sided instrument acceptance, run
2026-07-26: `--self-check` → `SELF-CHECK: PASS` 8/8 exit 0; FLAG side `--toss 2`
→ `REPRODUCED` exit 0; ACCEPT side `--thrower ball_butler --toss 2` →
`REPRODUCED` exit 0. **Deferred operator handoff:** nothing at the bench for this
phase (offline, read-only, gate already met against a captured bag); § Section
CATCH of `tests/hardware/session_anomaly_fixes.md` carries CATCH-1/2/3 for
whenever a capture is scored. The `peak_leg_*` staleness **fix** is deferred as
its own commit with its own logbook entry — annotated only, see the Notes for
collaborators.

All four candidate hypotheses were discriminated:

| hypothesis | verdict | evidence |
|---|---|---|
| stale-diagnostic illusion (several installs) | **refuted** | one `move_seq`, one `plan_kind`, zero accepted `target_feedback` and zero `catch/dynamic_target` in the window; and `peak_leg_*` is per-plan (see above). *Not* on plan-end drift, which is blind to this hypothesis — see the correction above |
| non-zero seed boundary conditions | **refuted as stated, half-right in shape** | the seed was at rest (previous plan past its duration; commanded park flat to 0.0000°) and the replay with zero seed twist/accel reproduces to 0.005° rms. The bulge *is* a boundary condition — the specified **arrival** twist `v1`, at the far end |
| freeze/stop interaction | **refuted** | the reach-freeze first fires at release + 0.506 s, after the plan ended; no graceful stop installed; the flat hold is the plan's own 0.5 s quiescent-hold segment and the acc spike is its 0.15 s decay segment |
| frame composition at the endpoints | **refuted as arithmetic, confirmed as premise** | `levelling.correct_pose` of the wire pose reproduces the target to 4 dp. The frame error is that `build_catch` reads a **plan-frame** tilt as "the receive tilt" — the composition is right, the interpretation is not |

### Phase 1 — Root cause, in writing

Trace the mechanism end-to-end in prose before proposing a fix — the repo's
standard for "diagnosis is clear". Specifically account for **all three** measured
features, because a hypothesis that explains only the overshoot is not the answer:

1. the +2.32° excursion opposite in sign to a −0.78° target;
2. the settle at 1.385× the target, matching neither single nor double application;
3. the acceleration/jerk spike at release − 0.6 s coinciding with the tilt going
   flat.

Then state whether the mechanism is specific to `build_catch` or general to the
C2-replan-to-fixed-arrival pattern (the follower and chase paths share the shape).
That determines the blast radius and therefore the fix's shape.

**Gate:** the written trace is reviewed. If the mechanism turns out to be general
rather than near-degenerate-specific, **stop and re-prioritise this plan upward** —
it would then be a live risk on the shipping reload path, not a robustness item.

#### Outcome — 2026-07-26: GATE MET, and the escalation clause FIRED

**Outcome, 2026-07-26 — LANDED, folded into Phase 0's commits.** Commit `a680298` (as for Phase 0); logbook
`logbook/2026-07-25-catch-reach-overshoot-repro.md`. Same full-suite triple as
Phase 0: `python -m pytest tests/ -q`, run 2026-07-26 → **3527 passed, 3 xfailed
in 1367.68 s (0:22:47)**. Phase 1 was folded into Phase 0's deliverable rather than
deferred, because all three features were fully traced by the reproduction itself
and a separate session would have had to reload the entire trace to write the
same prose. **Deferred operator handoff:** the gate's escalation clause FIRED —
execution-order item 9 (P1 → P2) must not start without operator
re-prioritisation, and C-CATCH-1's rewritten first clause needs ratification at
the same point.

The mechanism is **not** degenerate-specific. It runs on every catch reach with a
non-zero plan-frame tilt, including the shipping reload path; the near-degenerate
case is simply where the amplification is large. The plan is re-rated LIVE (see
the priority note at the top). It is nonetheless **specific to `build_catch`**:
`build_timed`, `build_follow` and the chase path all arrive at a caller-supplied
twist and none of them *manufacture* one, so the blast radius is the catch path
only.

The three features, end to end:

1. **The +2.32° excursion opposite in sign to a −0.78° target.** `build_catch`
   gives the reach a non-zero arrival twist `rate · tdir` (tilt-through-seat: a
   parked tilted rim deflects the ball, so the tilt must still be *moving* at
   contact). A quintic from rest to `(p1, v1, 0)` decomposes exactly as
   `p(s) = p0 + (p1−p0)·ψ(s) + v1·T·φ(s)` with `ψ = 10s³−15s⁴+6s⁵` and
   `φ = −4s³+7s⁴−3s⁵`. `φ` has extremum **−16/81 at s = 2/3**, so a negative
   specified arrival rate drives a *positive* excursion first. At the reference
   lead the twist term contributes **+2.925°** and the displacement term
   **−0.615°** at the same instant; net **+2.32°**, matching the bag to 0.001° at
   the same sample. It is a specified boundary condition of the reach, and it
   grows **linearly with the catch lead** (0.789°/s here).
   *(Sampling note: the `+2.3204°` in the anomaly table above is the echo sample
   at −1.98 s; the true peak sample sits at −2.0009 s reading `+2.3224°`, which
   is what the harness compares its `+2.3236°` against. Same excursion, adjacent
   25 ms knots — not a discrepancy.)*
2. **The settle at 1.385× the target.** The through-seat overshoot:
   `settle = target · (1 + 0.5·rate·decay/|tilt|)`. For this offset
   `|tilt| = 0.0136459 rad` and `0.5 · 0.07 · 0.15 = 0.00525 rad`, so the factor
   is **1.384732** — closed form `−1.078408° / −0.095775°` against a recorded
   `−1.0784 / −0.0958`. It matches neither a single nor a double application of
   the levelling correction because it is neither.
3. **The acc/jerk spike at release − 0.6 s.** The tilt-through-seat **decay
   segment**. Gated segment-by-segment at the reference lead: the 3.707 s reach
   peaks at `13.9 mm/s² / 35 mm/s³`; the 0.150 s decay peaks at
   `142.4 mm/s² / 3950 mm/s³` — 10.2× the acceleration and 113× the jerk, in a
   segment 25× shorter. It lands at the arrival, which the coordinator schedules
   at `landing − 1.5 s` = release − 0.700 s; the 5 Hz diagnostics window
   −0.814 → −0.614 s straddles exactly that boundary. The tilt "going flat" at the
   same moment is the same event: the decay ends and the quiescent hold begins.
   Reproducing the 40 Hz realized-peak tracker over the rebuilt plan needs **two
   comparisons kept apart** — conflating them produced a false claim in this
   paragraph's first draft, so both are spelled out:
   - *Like-for-like, running max truncated at the same instant.* The bag's
     `−0.6138 s` diagnostics sample is a still-climbing running max, not an
     end-of-plan value. Truncating the model at the matching `τ = 3.7934 s`
     gives acc `141.2` / jerk **`2416`** against the bag's `138.1` / **`2403`**.
     That is the agreement.
   - *Full-plan, phase-swept band.* The decay is only ~6 knots long and the
     emitter's grid is not phase-locked to the install, so sweeping the phase
     over one 25 ms knot spans `138.8–142.4 mm/s²` and `2558–3141 mm/s³`
     (measured 2026-07-26 at the reference lead; the probe prints `138.7–142.4`
     and `2551–3148` at the bag's actual 3.7072 s lead). The bag's *end-of-plan*
     realized values are `138.8 / 2496`. The acceleration is inside the band; the
     **jerk lands ~2 % under it and that is expected**, because realized jerk is
     a difference of consecutive knots and so carries the emitter's tick jitter
     (25.0–27.6 ms measured this session) on top of the phase.
   > **Correction, 2026-07-26 (review).** The first draft quoted the truncated
   > model jerk `2416` *inside* the full-plan band `2558–3141` — impossible, since
   > the band's floor is a superset of the phase-0 value — and said the band was
   > "bracketing the recorded values" when neither recorded value is inside it
   > (`138.1 < 138.8`, `2403 < 2558`). Quoting it that way would have had an
   > operator score a healthy post-fix capture's under-band jerk as a failure.
   > Feature 3 is **reported and never gated**, precisely for this reason.

**Why the through-seat engages for a level catch at all** is the root cause under
all three: `build_catch` reads `catch_pose[3:5]` as "the receive tilt", a premise
that holds only while the commanded frame *is* the gravity frame. With a levelling
correction loaded, a gravity-level catch arrives as a non-zero **plan-frame** tilt
— the correction itself — so the through-seat aims along the correction and the
reach acquires an arrival rate nobody asked for. Pinned by
`tests/ros/test_levelling_frame.py::test_catch_through_seat_aims_off_the_gravity_referenced_receive_tilt`.

Not to be lost: the leg magnitudes here are small in absolute terms
(`14.2 mm/s`, `142.4 mm/s²`, `3950 mm/s³` against session limits of
`1000 / 5000 / 30000`). The defect is **aim**, not violence — the platform is up
to 2.3° off where it should be through the flight and settles 0.30° off
gravity-level at ball contact, on a rim whose whole job is to seat a bouncing
ball.

> **The 0.30° and the 16 mm — which lever arm, corrected 2026-07-26 (review).**
> An earlier draft of this paragraph asserted that the session's 16 mm tracker
> catch error must *not* be attributed to the 0.30° residual, on the grounds that
> "0.30° of cup-axis tilt is order 1 mm at the seat". That reasoning uses the
> wrong lever arm and **contradicts a landed normative contract**
> (`ros_ws/docs/levelling_frame.md` § "The 16 mm"), which was not amended — a
> silent contract drift, which this repo's engineering philosophy forbids
> outright. The contract is right and the draft was wrong:
> - **~1 mm** is the *seat-geometry* offset — the cup's own displacement under a
>   0.30° platform tilt, over a lever arm of a few hundred mm. Real, but not what
>   the tracker measures.
> - **16.5 mm** is the *throw-direction* offset, and it is the contract's claim:
>   the catch plan's quiescent-hold segment runs to release − 0.05 s and
>   `hold_after=True` holds the settle pose through release, so the hand throws
>   from a platform sitting 0.3008° off gravity-level. `0.005250 rad × 3.93 m/s ×
>   0.8 s = 16.5 mm` of landing error. This plan's own Context table confirms the
>   premise from the bag: commanded `rx` is flat at `−1.0784°` from −0.50 s to
>   +2.00 s, i.e. across release.
>
> So the two numbers are not in conflict and neither is "separate" — they are
> different lever arms on the same residual, and the contract's is the one that
> predicts the tracker error. Phase 2's post-fix gate stays as the contract and
> plan 1 state it (`tracker catch error < 10 mm`).

(`_CATCH_TILT_THROUGH_RATE_RADPS`'s
docstring estimates the induced leg velocity at "~7 mm/s"; measured 14.2 mm/s —
a 2× underestimate, still negligible, worth correcting when that constant is next
touched.)

### Phase 2 — Fix, invariant, test

What is pre-committed is the form of the deliverable: an invariant, one enforcement
point, and a test — not a special case for the degenerate configuration.

#### The invariant — REWRITTEN 2026-07-26

The original first clause read *"a catch reach whose target differs from the
current commanded pose by less than the arrival tolerance shall command no
motion"*. **Phase 0 proposes withdrawing that clause — NOT YET RATIFIED; see the
caveat below.** The concrete failure mode it would cause: it is a stationarity
*mandate*, so it would reject a future planner that deliberately specifies an
arrival twist for a moving-platform catch. Stationary-platform catches are the
near-term preference; more aggressive juggling will need a moving platform at
catch and at throw, and this invariant would forbid the right answer for it.
Platform motion during a catch should be an *output* of the planner deciding it
produces a better trajectory — which is exactly what today's defect is not, since
`build_catch` manufactures the arrival twist from a module constant.

> **Ratification caveat, 2026-07-26 (review).** The first draft justified this
> withdrawal by citing "the operator principle of 2026-07-26", which appears
> **nowhere in the repository** — no logbook entry, no sibling plan, no memory
> file, no `docs/` note. That is an appeal to an uncitable authority, and it
> would have let a fresh Phase-2 session read the withdrawal as operator-ratified
> and never re-surface it. The root cause above stands on its own and needs no
> such appeal. But whether the platform may be *moving* at ball contact is an
> operator call on a change the risk register says "changes commanded motion at
> ball contact on **every** catch, including the shipping reload path" — so this
> rewrite must be put to the operator at the same re-prioritisation the fired
> STOP already requires (execution-order item 9), not carried forward as settled.

The replacement bounds *unrequested* excursion against *what was actually asked
for*, which still catches a 0.78° request becoming a 2.32° swing while leaving a
deliberately-specified arrival twist legal:

> **C-CATCH-1.** Over its whole duration, a catch plan's commanded pose shall not
> depart from the straight-line path between its seed pose and its target pose by
> more than a bounded factor of what the *request* implies — the target
> displacement plus the excursion any **explicitly specified** arrival twist
> requires. Motion the caller did not ask for, in any degree of freedom, is
> bounded; motion the caller did ask for is not.

Two consequences, both deliberate. (a) An arrival twist the *caller* supplies is
requested motion and is measured into the bound, so a future moving-platform catch
passes. (b) An arrival twist `build_catch` *manufactures* from a constant is not
requested motion, so today's plan fails the bound — which is the point.

#### The fix direction Phase 0/1 points at

Pass the **gravity-referenced receive tilt** to `build_catch` separately from the
commanded pose. Today `build_catch` infers the through-seat direction from
`catch_pose[3:5]`, which is a plan-frame quantity, so a level catch under a
levelling correction gets a through-seat aimed along the correction. Given the
receive tilt explicitly, the residual follows from what the *ball* is doing and
goes to zero for a level catch **by construction** — not by a threshold, not by a
degenerate-case branch.

Note what this does **not** fix on its own: with a genuine small receive tilt the
amplification `(16/81)·rate·T/|tilt|` is still large, because `rate` is a constant
while `|tilt|` is not. Sizing the residual rate *relative to* the requested tilt
(or capping the excursion it may produce) is the second half.

Resist the tempting narrow fix — special-casing "target ≈ current pose" to a hold.
It hides the amplifier rather than bounding it, and post-C-LEVEL-1 the amplifier is
pointed straight at the small-but-real receive tilts described above.

**Gate:** `pytest tests/ -q` green; a test that fails against the pre-fix code
(verify this explicitly — a test that passes both ways proves nothing).
`tools/probes/catch_reach_replay.py --self-check` must be re-run and its
mirrored constants re-derived, since the fix moves at least one of them. The
self-check will now *tell you* which: case 7 compares each mirrored value
(`build_catch(tilt_decay_s=)`, `_CATCH_TILT_THROUGH_RATE_RADPS`,
`_CATCH_TILT_OVERSHOOT_FRAC`, `JB_TRAJ_CATCH_SETTLE_HOLD_S`, the coordinator's
`_PRETILT_EARLY_S`) against its production source and fails loudly on drift.
Before the 2026-07-26 review it caught only two of the five — `build_replay`
forced the probe's own `tilt_decay_s` into every production call, so moving
`build_catch`'s default left the self-check green while the probe silently kept
building the old decay.

**Also required before scoring the post-fix capture:** re-derive the CATCH-3
pre-fix baselines in `tests/hardware/session_anomaly_fixes.md` § CATCH. A fix
that resizes or removes the through-seat **will** move the reload settle
(`+1.8235 / −10.9330°`), so that number is a pre-fix reference, never a post-fix
pass criterion.

#### Outcome — DONE, 2026-07-26

Commits **407154f** (C-CATCH-1) and **30e9723** (the separate `peak_leg_*`
diagnostics fix). Logbook: `logbook/2026-07-26-catch-reach-overshoot-fix.md`.

`build_catch` now takes the **gravity-referenced receive tilt** as its own
argument, so a level catch's arrival twist is zero *by construction* (`smag == 0`)
rather than by a threshold, and contract **C-CATCH-1**
(`ros_ws/docs/catch_arrival_contract.md`) bounds every departure from the target
that a *derived* arrival twist creates — the reach excursion **and** the settle
overshoot — to `40/81` of the catch's physical tilt scale. One enforcement point,
`planner._catch_arrival_rate`. The bound factor is the quintic's own geometry
(`min_s ψ/|φ| = 5/2`, measured), not a fitted value.

On the reference bag all five self-toss reaches go from `2.32°` of unrequested
excursion to `0.0000°`, the settle lands on the target (`−1.078408° →
−0.778784°`), and the `0.3008°` residual at ball contact — worth 16.5 mm of
throw-direction error — goes to zero. Both reload reaches keep their seat, with
the aim rotated `4.0997°` and the settle moved `+0.021086° / +0.004297°`.

**The correction made during finalize, because it is the part a future reader will
need.** The bound was first sized against the *residual seed → target travel*.
Three independent reviewers converged on the same defect and it reproduced: on the
shipping reload path the coordinator re-installs the catch every balls tick from
`arrival + settle_hold` to `arrival − reach_freeze`, each install seeded already
on the target, so the residual collapses and with it the bound — the arrival rate
of the plan actually frozen through **ball contact** fell `0.070000 → 0.004460
rad/s`, a **15.7× de-rate** that parks the rim at the instant the through-seat
exists for. The scale is therefore the **larger** of the residual travel and the
receive-tilt magnitude: the displacement-only reading degenerates exactly where
its own physical meaning does, since once the target *is* the seed, every nonzero
arrival velocity is "wrong-side" by definition. Pinned by
`test_ccatch1_keeps_the_seat_on_an_on_pose_supersede` (fails against the
residual-only bound, verified by monkeypatch).

Known, accepted, and pinned: the two **advisory** `T = 0.95` Tier-8b spot checks
in `sim/toss_gate.py` do bind (`0.070000 → 0.059469 rad/s`, settle overshoot
`0.3008° → 0.2555°`) — correct behaviour, since there the pre-tilt at A leans
opposite B's seat and the manufactured excursion reaches 0.581 of the scale. Both
sit outside the binding 50 mm ring, so no gate PASS band moves.

**Test triple:** `pytest tests/ -q`, run 2026-07-26 on the Jetson in the project venv: **3543 passed, 3 xfailed in 1376.13 s (22:56)**. Probe acceptance, 2026-07-26:
`tools/probes/catch_reach_replay.py --self-check` → `SELF-CHECK: PASS` 10/10,
exit 0; all seven recorded reaches (`--toss 1..5`, `--thrower ball_butler --toss
1,2`) → `REPRODUCED`, exit 0.

**Deferred to the operator:** the bench sitting. `tests/hardware/session_anomaly_fixes.md`
§ Section CCATCH (CCATCH-1..5) carries the copy-pasteable commands and numeric
PASS/ABORT criteria. Requires `colcon build --packages-select jugglebot` **and a
relaunch** (the launch runs the installed copy; a stale install reproduces the
pre-fix behaviour exactly). No firmware flash, no config regeneration.

### Phase 3 — the manufactured seat rate ships at ZERO

Added **2026-07-26, after Phase 2 landed**, by an operator decision this plan was
not written to anticipate. One line of behaviour:
`planner._CATCH_TILT_THROUGH_RATE_RADPS = 0.07 → 0.0`.

**The decision, in the operator's own framing, because the framing is the
load-bearing part.** Platform motion during a catch or a throw is **PERMITTED but
never MANDATED**: the only reason the platform should be moving at contact is that
*the planner determined it produces a better trajectory*. It should never be a
constant in the trajectory builder. Setting the manufactured fallback to zero is
the cleanest expression of that — the builder now manufactures nothing, and motion
appears only when a caller asks for it. Stationary catches and throws are the
near-term preference; more aggressive juggling will later need a moving platform.

**So this is a DEFAULT, and explicitly not a rule.** Nothing was added that asserts
a catch commands no motion. `tilt_through_rate_radps` is unchanged, the decay-segment
code path is unchanged, and C-CATCH-1's rule that an explicitly-supplied rate is
returned verbatim and **unbounded** is unchanged — that is the seam a future
optimising planner uses, and it is pinned by
`test_the_shipped_default_manufactures_nothing_but_still_obeys_a_caller`.

**The vacuity trap, which is most of the work in this phase.** At a zero default
the C-CATCH-1 bound is unreachable in two independent ways, both silent:
(a) `assert rate == approx(planner._CATCH_TILT_THROUGH_RATE_RADPS)` degenerates to
`0 == 0`; (b) passing `tilt_through_rate_radps=` takes the deliberately-unbounded
*requested* branch. Between them the whole `test_ccatch1_*` block stays green while
proving nothing. Every such test now restores a non-zero **module constant** via
`_set_seat_rate(monkeypatch)` — never via the kwarg — and the block was
mutation-tested by deleting the bound. The same trap in the ROS tests is handled by
a `_PRE_FIX_SEAT_RATE_RADPS = 0.07` capture-record constant, and in the probe by
splitting its single mirror into a *live default* mirror (`0.0`, compared against
production) and a *capture record* (`0.07`, pinned to itself so nobody syncs it and
makes every pre-2026-07-26 bag score NOT-REPRODUCED).

**The physical risk, surfaced and deliberately not solved in code.** The `0.07`
existed because *a parked tilted rim deflects the ball* (the bb-sim geometry
finding). A vertical catch seats level, so zero costs it nothing — under C-CATCH-1
its seat rate was already zero. The **reload** catch seats at 11.08°, which is
exactly that geometry, and its rim is now stationary at contact. Two mitigating
facts, stated alongside the risk and not instead of it: the constant has **never**
been validated on hardware, and until `407154f` its aim was wrong on every levelled
catch, so no bench impression of it was formed on a correctly-aimed seat. Scored at
the bench by `tests/hardware/session_anomaly_fixes.md` § Section ZSEAT (ZSEAT-2 is
the scored row: reload seating and bounce-out); a failure routes back to this phase
and its fix is a one-line default, with C-CATCH-1 already in force to bound
whatever value goes in.

#### Outcome — 2026-07-26

Measured through the committed harness, `tools/probes/catch_reach_replay.py`
against `~/Desktop/rosbags/2026-07-25_15-17-48`: all **seven** recorded reaches
still read `REPRODUCED`, exit 0 (5 self-toss + 2 reload) — `build_replay` passes
the rate explicitly, so the verdict half is untouched by the default. `--self-check`
`SELF-CHECK: PASS` 10/10, exit 0.

The **reload delta** — the path this change actually alters, at the recorded
geometry (lead 2.3712 s, wire receive tilt 10.87°), C-CATCH-1 at `0.07` → shipped
`0.0`:

| | rate 0.07 | rate 0.0 | delta |
|---|---|---|---|
| arrival tilt rate at contact | `0.070000 rad/s` | **`0.000000`** | −0.070000 |
| settle `rx` / `ry` | `+1.844635 / −10.928741°` | **`+1.774062 / −10.636334°`** (= the target) | `−0.070573 / +0.292407°` |
| residual past the seat | `0.300803°` | **`0.000000°`** | −0.3008° |
| segments / duration | 3 / lead + 0.65 s | **2 / lead + 0.50 s** | −1 / −0.15 s |
| predicted vel / acc / jerk | `23.8 / 142.0 / 3935` | **`29.0 / 37.9 / 170`** | +22 % / −73 % / −96 % |

The **on-pose supersede** — the plan actually frozen through ball contact — goes
further: at `0.0` the previous plan's settle *is* the target, so the supersede is a
literal no-motion plan (all predicted peaks `0.0`) and the commanded tilt over the
last 0.8 s before landing goes from a `≈0.9°` round trip to flat. That inverts one
of § CHECK CCATCH-3's ABORT criteria, which is corrected in place by a banner there.

The self-toss (level) catch is **unchanged in every respect** — its receive-tilt
magnitude was already zero, so `smag == 0` had already disabled the seat at
`407154f`.

Note the predicted leg **velocity rises** 22 % while acc and jerk collapse. That is
arithmetic, not a surprise: with `p' = d·ψ'/T + v1·φ'` and `φ'(0.5) = −0.4375`, a
terminal rate along the travel lets the reach coast slower through its middle, so
removing it restores the plain rest-to-rest `1.875·d/T` peak (29 mm/s against a
1000 mm/s session ceiling). Recorded because "everything got smaller" would be a
false expectation at the bench.

**Throw side, confirmed rather than assumed.** There is no residual *commanded*
platform rate at the release instant on either tier, before or after this change.
Tier 8a's release falls inside the catch plan's zero-twist quiescent hold (arrival
`landing − 1.5 s` + 0.5 s hold, then `hold_after=True`) for the whole shipped
`0.55 … 1.10 s` flight band — a release inside the reach would need a flight
> 1.5 s. Tier 8b suppresses that pre-tilt via `catch/pretilt_hold` and holds the
positioned pose at A open-loop, publishing its deferred A→B reach **at**
`t_release`. What this change removes on 8a is the `0.3008°` *pose* offset the hold
was holding — and for a level catch that had already gone to zero at `407154f`.

### Phase 4 — Hardware validation (operator-run)

Run 2026-07-27. Verdicts, every measured number and the Discussion are in
`logbook/2026-07-28-anomaly-fixes-validation-sitting.md`; the status cell above is
the summary. Two things landed *afterwards*, on 2026-07-28, and are recorded here so
the phase's status is readable without reconstructing it from three documents.

**Outcome, 2026-07-28 — the Tier-8b default landed; `ZSEAT-2` stays ABORT and its
experiment stays open.** Commit `<SHA-1>` makes `jugglebot_operational.toss_tier:
"8b"` the shipped default — the operator's decision on this phase's own T4 evidence
(11/11 displaced throws accepted out to the 70 mm cap, `ZSEAT-4` reading `0.0000°`
tilt span and `0.0000 mm` position span pre-release on all 11). Commit `<SHA-2>`
amends the sitting record with the operator testimony that arrived after scoring.
Gate: `pytest tests/ -q`, run 2026-07-28 on the Jetson in the project venv:
**3967 passed, 3 xfailed in 1409.71 s** (`+1` on the `6641400` baseline of 3966 + 3
— one added parametrisation; xfail unchanged at 3).

What this does **not** close, handed to the operator:

- **`ZSEAT-2`'s attribution is still INCONCLUSIVE.** The testimony confirms the three
  drops were bounce-outs and corroborates the BB warm-up drift as the trigger, but
  the **capture-basin width** — whether a non-zero seat catches a ball ~30 mm
  off-centre that the zero seat drops — is untouched. The pre-registered A/B (burn in
  the BB, then ~6 throws biased `+30 mm` in x at seat rate `0.0`, then the same 6 at
  `0.07`) is scheduled for the next sitting and is the only thing that can answer it.
  **The shipped default stays `0.0` until it returns a verdict.**
- **The flip is not deployed until `colcon build --packages-select jugglebot` +
  relaunch.** The launch runs the *installed* copy of `hardware_config.py`, so until
  the rebuild the robot executes Tier 8a while the repo, the tests and this plan all
  say 8b — and that failure is invisible from the Jetson except by reading the
  installed file. Row `TIER-PREREQ` of `tests/hardware/session_anomaly_fixes.md`
  § SECTION TIER checks exactly that, and is the first row of the next sitting.
- **The co-located (`B == A`) 8b path has never run on hardware.** All 11 validated
  T4 throws were displaced. Row `TIER-D` covers it.

## Risk register

| Risk | Mitigation | Status |
|---|---|---|
| Inheriting my possibly-wrong "single install" reading | Phase 0 step 1 settles it from raw data before anything else | **retired** — settled; the reading was correct, the staleness worry refuted |
| Cannot reproduce offline, so the plan stalls | reproduction failure is an acceptable, publishable outcome; do not fix by guesswork | **retired** — all 7 catch reaches reproduced |
| The mechanism is general, not degenerate-specific | Phase 1's gate escalates the plan's priority if so | **FIRED** — general to every catch with a plan-frame tilt; plan re-rated LIVE |
| A narrow degenerate-case special case ships and hides the amplifier | Phase 2 pre-commits to a bounded-excursion invariant instead | **retired** (2026-07-26) — no threshold shipped. A level receive tilt yields a zero arrival twist *by construction* (`smag == 0`), and the bounded-excursion invariant landed alongside it as C-CATCH-1 |
| The Phase-2 invariant enshrines a stationary platform | Phase 0 proposes withdrawing the first clause; C-CATCH-1 bounds *unrequested* excursion only, so a deliberately-specified arrival twist stays legal | **retired** (2026-07-26) — resolved in the contract itself rather than by citation. C-CATCH-1 as landed does not mention platform stationarity at all: it distinguishes **requested** motion from **manufactured** motion. An arrival twist a caller passes explicitly (`tilt_through_rate_radps=`) is honoured verbatim and unbounded, so a future moving-platform catch is legal by construction; only the rate `build_catch` falls back to from its OWN module constant is bounded. The failure mode that drove this, stated plainly rather than as an appeal: a blanket stationarity clause would mandate a parked rim just as surely as the old constant mandated motion, and a parked tilted rim deflects the ball. No operator ratification is outstanding — there is no stationarity clause left to ratify |
| The instrument mis-scores a HEALTHY capture and routes a correct fix back for rework | two-sided validation is the acceptance criterion: `--self-check` (no bag) plus both the FLAG case (`--toss 2`) and the ACCEPT case (`--thrower ball_butler --toss 2`) must pass, and the verdict tolerance scales with the commanded span | **retired for three known paths** — the 2026-07-26 review found and fixed all three: a sticky `last_rejection` latch that forced NOT-REPRODUCED on every reach after one unrelated session rejection; an unbounded `/gravity_offset` lookup that scored pre-re-level reaches with a post-re-level correction; and a self-check blind to two of five mirrored constants |
| Zeroing the manufactured seat rate (Phase 3) parks the **tilted** reload rim at ball contact, which is exactly the geometry the bb-sim deflection finding concerns | Not mitigated in code, on purpose — the constant was never hardware-validated and its aim was wrong until `407154f`, so the bench is the only place this can be settled. Scored by `tests/hardware/session_anomaly_fixes.md` § Section ZSEAT, CHECK ZSEAT-2 (caught/attempted ≥ 12/19; **bounce-outs ≤ 1**, ≥ 3 or ≥ 2 consecutive aborts) | **open — deliberately.** A failure routes back to Phase 3 and its fix is a one-line default; C-CATCH-1 is kept live and will bound whatever value replaces the zero |
| Zeroing the default silently makes the whole C-CATCH-1 test block vacuous | Every `test_ccatch1_*` restores a non-zero rate by monkeypatching the **module constant** (`_set_seat_rate`), never by passing `tilt_through_rate_radps` (the unbounded requested branch); mutation-tested by deleting the bound | **retired** (2026-07-26) — see the Phase 3 Outcome; two tests pass without the bound *by design* (they assert the bound is a no-op) and that is unchanged from before Phase 3, verified on the pre-change tree |
| "The default is zero, so the parameter and the decay path are dead code" — a future simplification converts the default into a mandate | `test_the_shipped_default_manufactures_nothing_but_still_obeys_a_caller` pins both halves: nothing manufactured, and an explicit rate honoured verbatim including the decay segment and the settle overshoot | **retired** (2026-07-26) |
| Fixing this changes commanded motion at ball contact on **every** catch, including the shipping reload path | Phase 2 needs a powered sitting; the reload leg's own numbers are now in the plan (settle `+1.8235 / −10.9330°` = target × 1.0279) so a regression is measurable, and `catch_reach_replay.py` scores a post-fix capture the same way | **open — deliberately, and now quantified.** The reload change is aim-only: seat aim rotates `4.0997°`, settle moves `+0.021086°` rx / `+0.004297°` ry, predicted acc/jerk `139.7/3873 → 142.0/3935` (+1.6 %, three orders under the session ceilings). The reload seat RATE at contact is **unchanged** at `0.070000 rad/s` — that took a correction during finalize (see the Phase 2 Outcome: sizing the bound on residual travel alone de-rated it 15.7×). Gated at the bench by `tests/hardware/session_anomaly_fixes.md` § Section CCATCH, CCATCH-3 |

## Notes for collaborators

- Everything needed to start Phase 0 is already recorded: the bag
  `~/Desktop/rosbags/2026-07-25_15-17-48/2026-07-25_15-17-48_0.mcap` contains
  `/leg_setpoint_echo`, `/rigid_body_poses`, `/catch/dynamic_target`,
  `/trajectory/status`, `/trajectory/diagnostics` and
  `/trajectory/target_feedback` for five toss attempts, plus two reload attempts
  with a clean 11.08° pre-tilt as the contrast case. No hardware time is needed
  before Phase 2.
- The clean-reload contrast is the most useful single fact in this plan: whatever
  the mechanism is, it must explain why an 11.08° reach is well-behaved and a
  0.78° one is not. **It does, in one number** —
  `(16/81)·rate·T/|tilt|` is 3.76 for the toss and 0.17 for the reload. And the
  reload is not "well behaved" for any reason of its own: it carries the same
  through-seat overshoot (settle = target × 1.0279 instead of × 1.3847), just 22×
  smaller relative to what it was asked to do. Both are reproduced by the same
  harness, so the contrast is now a runnable command rather than an observation:

      source ~/Desktop/PDJ_venv/venv/bin/activate
      python tools/probes/catch_reach_replay.py \
          --bag ~/Desktop/rosbags/2026-07-25_15-17-48 --toss 2
      python tools/probes/catch_reach_replay.py \
          --bag ~/Desktop/rosbags/2026-07-25_15-17-48 --thrower ball_butler --toss 2

- Index caveat: the plan's prose calls the excursion "toss #4" counting all seven
  attempts in the session. In the harness, self-thrown and BB-thrown announcements
  are indexed separately — the excursion is `--toss 2` (of five self-tosses).
  `--list` prints the mapping: one table over **every** thrower on a shared time
  origin, carrying both the all-throwers index and the per-thrower index that
  `--toss` takes. (It printed one thrower at a time, each zero-based on its own
  first announcement, until the 2026-07-26 review — which made the two indexings
  impossible to reconcile without going back to raw `release_abs`, so an operator
  asking for "toss #4" would have scored `--toss 4` = release +73.8 s, the wrong
  reach, and never known.)
- The harness's verdict tolerance is `max(--tol-deg, --tol-frac × commanded span)`.
  A purely absolute bound scores the clean 11° reload as NOT-REPRODUCED on a 0.5 %
  error, which is the "instrument fails a working system" trap the sibling
  levelling probe's two-sided self-check exists to prevent.
- Deployment: nothing in Phase 0 changes runtime behaviour. The
  `trajectory_node.py` edit is **comment-only** (the `peak_leg_*` staleness
  annotation); a `colcon build --packages-select jugglebot` + relaunch is still
  wanted so the installed copy matches source, but no behaviour depends on it.
- **Still open, handed to the operator, not fixed here.** The `peak_leg_*`
  staleness hole was *annotated* rather than fixed, because closing it means
  reordering `_svc_go_to_pose`'s `_last_peak_*` write to **after** its
  `self._install(plan)` call (it currently writes before) so that a future
  `_install` may clear the field — a behavioural reordering in a safety-adjacent
  install path, with a live assertion in `tests/ros/test_trajectory_node.py` that
  `_last_peak_*` matches the report after `go_to_pose`. Stated symbolically on
  purpose: line numbers in a hand-over go stale within one commit, and following
  a stale one here would land the moved write *before* the install and ship a
  worse defect than the one being fixed. Per the repo's convention this is its
  own commit with its own logbook entry. The same hole is recorded independently
  as open item 7 in `tests/hardware/mvp_bench_runbook.md`, from a 2026-07-09
  hardware observation — so it is not hypothetical.
