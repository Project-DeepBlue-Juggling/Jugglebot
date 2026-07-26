---
title: "Catch-reach near-degenerate overshoot reproduced offline — the excursion is a specified arrival twist, and the amplification goes as 1/|tilt|"
type: investigation
date: 2026-07-25
status: in-progress
phase: "Self-toss anomaly fixes — catch-reach-degenerate-overshoot Phases 0-1"
related_plan: "catch-reach-degenerate-overshoot.md"
files_changed:
  - tools/probes/catch_reach_replay.py
  - tools/probes/README.md
  - tests/motion/test_catch_reach_replay_probe.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/mvp_bench_runbook.md
  - plans/active/catch-reach-degenerate-overshoot.md
commits:
  - a680298
subsystem:
  - motion
  - ros
tags:
  - kinematics
  - testing
  - docs
---

# Catch-reach near-degenerate overshoot reproduced offline

## Summary

A `−0.778784°` catch pre-tilt target produced a `+2.32°` commanded platform
excursion in the *opposite* direction, settled at `−1.0784°` (1.385× the target),
and threw a leg acc/jerk spike 0.6 s before release. All three are **one plan,
installed once**, doing exactly what `planner.build_catch` specifies. All seven
catch reaches in `~/Desktop/rosbags/2026-07-25_15-17-48` — five self-toss
pre-tilts and both reload pre-tilts — now reproduce offline through the
production planner, seeded only from recorded inputs, at `rx` rms 0.002–0.006°.

The mechanism is **not** degenerate-specific, which fires the plan's own STOP
condition: it runs on every catch with a levelling correction loaded, including
the shipping reload path. What *is* degenerate-specific is the amplification
ratio, `(16/81)·rate·T/|tilt|` — **3.76** for the 0.78° toss pre-tilt against
**0.17** for the 11.08° reload, one code path, 22× apart. Because it goes as
`1/|tilt|`, the levelling fix (which drives the requested tilt toward zero) makes
this **worse**, not better — the exact opposite of the premise on which this plan
was originally rated "lowest of the four".

No production behaviour changed. The only `ros_ws/` edit is comment-only.

## Symptoms

From FK of `/leg_setpoint_echo` in `2026-07-25_15-17-48`, times relative to the
scheduled release at `1784956866.878554` (the plan's "toss #4" = the harness's
`--toss 2` of five self-throws):

| t rel. release | commanded `rx` |
|---|---|
| −4.23 s | `+0.0044°` |
| −2.00 s | **`+2.3224°`** (peak) |
| −0.92 s | ~0° (crossing) |
| −0.50 s → +2.00 s | **`−1.0784°`**, flat |

Plus `/trajectory/diagnostics` `realized_peak_leg_acc_mmps2` stepping
`13.9 → 138.1` and `realized_peak_leg_jerk_mmps3 → 2403` at release − 0.6 s,
coinciding with the tilt going flat.

A rest-to-**rest** quintic cannot do this. The settle matched neither a single
application of the levelling correction (`−0.7788°`) nor a double one
(`−1.5576°`).

## Diagnosis

### The gate: reproduce it, do not theorise about it

`tools/probes/catch_reach_replay.py` rebuilds the plan through the production
`planner.build_catch`, seeded **only** from recorded inputs: FK of
`/leg_setpoint_echo` for the seed pose, `levelling.correct_pose` of the recorded
`catch/dynamic_target` wire pose with the bag's own `/gravity_offset` for the
target, and `landing_time − _PRETILT_EARLY_S` for the arrival. Exactly one
nuisance scalar is fitted (the emit → Teensy → bridge → echo → record pipeline
lag), on a bounded grid, with the zero-lag residual printed beside it.

Measured 2026-07-26 against `~/Desktop/rosbags/2026-07-25_15-17-48`, 0 FK
failures:

| reach | lead | rx rms / max | ry rms / max | echo lag | verdict |
|---|---|---|---|---|---|
| self-toss ×5 | 3.705–3.720 s | 0.0049–0.0062° / ≤ 0.0391° | ≤ 0.0006° / ≤ 0.0035° | +10.0…+11.0 ms | REPRODUCED |
| reload ×2 | 2.370–2.371 s | 0.0021–0.0025° / ≤ 0.0093° | ≤ 0.0149° / ≤ 0.0558° | +9.5 ms | REPRODUCED |

The fit buys very little, which is the point: the zero-lag residual is rms
0.0190° / max 0.0656° against 0.0049° / 0.0269° at the fitted +10.5 ms.

### The three features, end to end

1. **The opposite-sign excursion is the reach ACQUIRING its specified terminal
   arrival rate.** `build_catch` gives the reach a non-zero arrival twist
   `rate · tdir` (the tilt-through-seat residual: a parked tilted rim deflects
   the ball, so the tilt must still be *moving* through the seat angle at
   contact). A quintic from rest to `(p1, v1, 0)` decomposes exactly as
   `p(s) = p0 + (p1−p0)·ψ(s) + v1·T·φ(s)`, with `ψ = 10s³−15s⁴+6s⁵` and
   `φ = −4s³+7s⁴−3s⁵`. **`φ` has extremum `−16/81` at `s = 2/3`**, so a negative
   specified arrival rate drives a *positive* excursion first. At the reference
   lead the twist term contributes `+2.925°` and the displacement term `−0.615°`
   at the same instant; net `+2.32°`, against the echo's `+2.3224°` at the same
   25 ms knot. Verified equal to the production `QuinticSegment` to **2.8e-14**.
2. **The 1.385× settle is the through-seat overshoot**,
   `settle = target · (1 + 0.5·rate·decay/|tilt|)`. With `|tilt| = 0.0136459 rad`
   and `0.5 · 0.07 · 0.15 = 0.00525 rad` the factor is **1.384732**, closed form
   `−1.078408° / −0.095775°` against a recorded `−1.0784 / −0.0958`. It matches
   neither a single nor a double correction because it is neither.
3. **The acc/jerk spike is the 0.150 s through-seat decay segment.** Gated
   segment-by-segment at the reference lead: the 3.707 s reach peaks at
   `13.9 mm/s² / 35 mm/s³`, the decay at `142.4 / 3950` — 10.2× and 113× in a
   segment 25× shorter. It lands at the arrival, which the coordinator schedules
   at `landing − 1.5 s` = release − 0.700 s. The tilt "going flat" at the same
   moment is the same event: the decay ends and the quiescent hold begins.

### The root cause under all three

`build_catch` reads `catch_pose[3:5]` as "the receive tilt" — a premise that
holds only while the commanded frame **is** the gravity frame. With a levelling
correction loaded, a gravity-level catch arrives as a non-zero **plan-frame**
tilt (the correction itself), so the through-seat aims along the correction and
the reach acquires an arrival rate nobody asked for. Pinned, deliberately
unfixed, by
`tests/ros/test_levelling_frame.py::test_catch_through_seat_still_aims_off_the_plan_frame_tilt`.

### Blast radius, and the STOP that fired

Phase 1's gate says: *if the mechanism is general rather than
near-degenerate-specific, stop and re-prioritise.* Precisely:

- **Not** general to the C2-replan-to-fixed-arrival pattern. `build_timed`,
  `build_follow` and the chase path all arrive at a **caller-supplied** twist;
  none of them *manufacture* one from a module constant. Blast radius is
  `build_catch` only.
- **But** general across every catch with a correction loaded — including the
  shipping reload path, which carries the same overshoot at
  `target × 1.0279`. That is the substance of the concern, so the STOP is
  **FIRED** and the plan is re-rated LIVE. Execution-order item 9 (this plan's
  P1 → P2) must not start without operator re-prioritisation.

### The question the brief required to be re-derived, not inherited

The plan's author flagged their own weakest link: *"`peak_leg_*` reads
identically `14.2 / 142.4 / 3950` before AND after the install, which strongly
suggests that field is stale or cached — if it is, my single-install reading may
be wrong."* Settled from raw messages:

- **The single-install reading is CORRECT.** `move_seq` holds at 48 across all 21
  diagnostics samples in the window; `plan_kind` is `move` at every sample; zero
  accepted `target_feedback` and zero `catch/dynamic_target` inside the window.
- **The staleness worry is REFUTED.** The field *is* written — the immediately
  preceding `go_to_pose` install (`move_seq` 47) published `0.0 / 0.0 / 0` and the
  catch install (48) published `14.2 / 142.4 / 3950`. The repetition across catch
  installs is real lead-invariance, not carry-over.
- **A narrow staleness hole IS real**, and it is the one already recorded from
  hardware as open item 7 in `tests/hardware/mvp_bench_runbook.md`: six install
  paths (`_svc_hold`, `_svc_go_home`, `_install_guard_descent`,
  `_retry_pending_stop`, `_install_graceful_stop`, the follower's input-loss
  stop) call `_install` — which bumps `move_seq` and resets the *realized* peaks —
  without writing `_last_peak_*`. None of them ran in this window. Annotated, not
  fixed; see Discussion.

## Discussion

### Why annotate the `peak_leg_*` hole rather than fix it

The brief offered "fix or annotate". Annotating won on a distinction that only
became visible once the census was built: **the trap the plan feared does not
exist on the path that misled this investigation.** The field was per-plan there;
the repetition was lead-invariance. So the fix is no longer urgent-in-context —
it is a general diagnostics hazard, correctly scoped as its own unit.

Against that, the fix is not as small as it looks. Making `_install` clear
`_last_peak_*` requires reordering `_svc_go_to_pose`'s write to **after** its
`self._install(plan)` call, because it currently writes *before* — a behavioural
reordering in a safety-adjacent install path, with a live assertion in
`tests/ros/test_trajectory_node.py` that `_last_peak_*` matches the report after
`go_to_pose`. Doing that inside a phase chartered as analysis-only would mix a
behavioural change into a commit whose whole claim is "zero executable lines".

The tradeoff accepted: a diagnostics hazard stays open one more session, in
exchange for the annotation landing exactly where the next investigator reads it
(`trajectory_node.__init__` and the `_publish_status` KeyValue block) and a full
enumeration being handed over. The hand-over is written **symbolically, not by
line number** — a review caught that the first draft's line numbers were already
two lines stale, and following them literally would have placed the moved write
*before* the install, shipping a worse defect than the one being fixed.

### Withdrawing Phase 2's pre-committed invariant clause

The plan pre-committed to *"a catch reach whose target differs from the current
commanded pose by less than the arrival tolerance shall command no motion"*.
Phase 0 proposes withdrawing that clause, and the reason is a concrete failure
mode rather than a preference: it is a **stationarity mandate**, so it would
reject a future planner that deliberately specifies an arrival twist for a
moving-platform catch. More aggressive juggling needs exactly that. The
replacement (C-CATCH-1) bounds *unrequested* excursion against what the request
implies, which still fails today's plan — because `build_catch` **manufactures**
the arrival twist from a constant rather than receiving it from the caller. That
distinction is what makes the invariant discriminating instead of prohibitive.

**This is not ratified.** The first draft justified the withdrawal by citing "the
operator principle of 2026-07-26", which a review found exists **nowhere in the
repository** — not a logbook entry, not a sibling plan, not a memory file. That
is an appeal to an uncitable authority, and it would have let a fresh Phase-2
session treat the withdrawal as settled and never re-surface it. The root cause
stands on its own; the citation is deleted and the rewrite is explicitly parked
for the operator at the same re-prioritisation the fired STOP already requires.

### Two plan hedges that did not survive, and one that reversed

The brief said not to inherit the plan's hedges. Two were false:

1. *"`peak_leg_*` is stale or cached rather than per-plan."* False — see above.
   Inheriting it would have sent this phase hunting a phantom sequence of
   re-plans.
2. *"This is not a general property of `build_catch` — it is specific to the
   near-degenerate case."* False as stated. Same property, 1/22 the
   amplification; the reload leg carries the same overshoot. Degenerate-specific
   is the *ratio*, not the mechanism.

And the plan's priority premise reversed outright. It was rated lowest of four
because "the levelling fix removes the trigger". C-LEVEL-1 drives the requested
tilt toward zero and the amplification goes as `1/|tilt|` — so plan 1 makes this
one **worse**. That single observation is why the plan is now LIVE.

### The evidence narrative was wrong even though the verdict was right

The most valuable thing this phase's review produced was not a new fact but a
correction to *why* a right answer is right — the kind of error that survives
indefinitely because nobody re-checks a conclusion they agree with.

The first draft elevated one statistic to **decisive**: `t + plan_time_remaining_s`
drifting only 1.9 ms across 21 status samples, on the reasoning that an install
resets the plan-time origin. Both halves were wrong.

- It is **structurally blind to the exact hypothesis it was credited with
  refuting.** `_plan_and_install_catch` computes `lead = arrival_perf −
  perf_counter()` and installs at `t0 = perf_counter()`, so the plan end is
  `arrival_perf + tilt_decay + settle_hold` **regardless of when the install
  happened**. A re-plan to the same absolute arrival — verbatim the plan's
  hypothesis 1 — leaves it untouched. The reference bag supplies the
  counterexample for free: the post-release republish burst runs `move_seq`
  53 → 64 with 14 accepted feedbacks across release +0.10…+0.70 s and the plan
  end drifts **1.02 ms**, *less* than across the genuinely zero-install window.
- There is no rate advantage. `/trajectory/status` and `/trajectory/diagnostics`
  are published by **one 5 Hz timer** (`trajectory_node.py:478 → _publish_status`
  publishes both), 1453 messages each over the 290.6 s session.

The verdict survives, over-determined, on the three signals that *do*
discriminate. But the wrong narrative had already reached a production source
comment, the probe docstring, the README and the plan — four places a future
session would have inherited it from. The honest replacement, now in
`install_census`'s docstring, states per signal what it can and cannot see, and
adds `plan_kind` to the gate: a `HoldPlan` reads `hold`, which is the direct,
per-sample detector of the three paths that install by direct assignment and
bump nothing at all.

### An instrument validated on only one shape will fail a working system

This phase's deliverable is an instrument, so its acceptance criterion is
two-sided: it must produce the right reading on the shape it must FLAG *and* on
the shape it must ACCEPT. Three ways it would have failed a healthy capture were
found and closed:

1. **The sticky rejection latch.** `trajectory_node._last_rejection` is assigned
   `''` in exactly one place — `__init__` — and republished on every 5 Hz status
   forever after. The census treated any non-empty value as an in-window install,
   so **one** early rejection anywhere in a session (a `WORKSPACE` reach-envelope
   reject, a `STALE_STATE` retry, a deliberate pre-`level` refusal) would force
   `SINGLE INSTALL: False` → `NOT-REPRODUCED` → exit 1 on **every healthy reach
   after it**. The runbook reads a non-reproduction as a finding, so that routes a
   correct fix back for rework and burns a powered sitting. It never fired on the
   reference bag (0 non-empty values across all 1453 status samples), which is why
   it survived the first pass — a good reminder that "it passed on the evidence
   bag" is not the same as "it works". Now only a rejection *raised* inside the
   window counts; the latched carry-over is reported as an advisory.
2. **The self-check was blind to two of five mirrored constants.** The mirroring
   block promises the self-check will "FAIL loudly rather than silently re-score
   old captures under new physics", but `build_replay` passed the probe's own
   `tilt_decay_s=0.15` into *every* production call — so mutating
   `build_catch`'s default to 0.25 left the self-check passing 7/7 (verified
   in-process), and nothing compared `_PRETILT_EARLY_S` against the coordinator at
   all. Both are constants Phase 2 is expected to move. Fixed by comparing the
   mirrored values against their production sources **directly** rather than via a
   downstream symptom, and mutation-pinned by a new test.
   `_PRETILT_EARLY_S` is read by parsing the coordinator's source with `ast`,
   which keeps the probe's "needs no ROS" scope claim intact.
3. **The `/gravity_offset` lookup was unbounded.** It took the last offset in the
   whole bag; the node applies whatever was live at *ingest*. `level` is per-boot
   and a mid-session re-`level` is a real operating pattern, so earlier reaches
   would be scored against a correction the node never applied to them — and at
   this session's scale a 0.5 mrad delta is ~0.03° of target error against a
   0.05° tolerance. Now bounded at the install, with the distinct-value count
   reported so a re-level is visible rather than absorbed.

The FLAG/ACCEPT pair is run end-to-end as the acceptance criterion: `--toss 2`
(the excursion) and `--thrower ball_butler --toss 2` (the clean 11° reload) both
read `REPRODUCED`, exit 0.

### A contract contradiction, resolved in favour of the contract

The first draft added a parenthetical telling readers **not** to attribute the
session's 16 mm tracker catch error to the 0.3008° through-seat residual, on the
grounds that "0.30° of cup-axis tilt is order 1 mm at the seat". That
contradicted a landed normative document — `ros_ws/docs/levelling_frame.md`
§ "The 16 mm" — without amending it, which is precisely the silent contract drift
this repo's engineering philosophy forbids.

Adjudicated by lever arm rather than by authority. Both numbers are right about
different things:

- **~1 mm** is the seat-geometry offset: the cup's own displacement under a
  0.30° platform tilt, over a lever arm of a few hundred mm.
- **16.5 mm** is the throw-direction offset, and it is what the contract claims.
  The catch plan's quiescent-hold segment runs to release − 0.05 s and
  `hold_after=True` holds the settle pose *through* release, so the hand throws
  from a platform sitting 0.3008° off gravity-level:
  `0.005250 rad × 3.93 m/s × 0.8 s = 16.5 mm` of landing error. The premise is
  confirmed by this very session — commanded `rx` is flat at `−1.0784°` from
  −0.50 s to +2.00 s, i.e. across release.

So the contract is right, the draft was wrong, and the correction went into the
**plan**, not the contract. Phase 2's post-fix gate stays `< 10 mm`.

### Smaller forks, with the failure mode each one prevents

- **Reuse `levelling_tilt_bag_check.reconstruct` for the FK** rather than writing
  a second reader. Prevents two probes drifting to two FK conventions, so that a
  change to `ik_solver`'s convergence criterion (item 1 of this very run) would
  silently make them disagree about what the commanded pose was. Only a one-message
  `echo_epoch()` helper was added, because `reconstruct` deliberately zero-bases
  its time axis and this probe must align against a perf-domain arrival.
- **Anchor the install at the triggering message and fit ONE scalar**, rather than
  free-fitting the lead. A two-parameter fit can absorb a genuinely wrong
  mechanism into its parameters and still look reproduced. With the lead fixed by
  recorded inputs, the fitted lag becomes a **falsifier**: it must be positive (an
  echo cannot precede its command) and under one 25 ms knot. Measured +9.5 to
  +11.0 ms on all seven, and it goes *negative* if the install is assumed 64.5 ms
  later — which is what pins the install.
- **Tolerance `max(absolute, fraction × commanded span)`.** A purely absolute
  bound scores the clean 11° reload as NOT-REPRODUCED on a 0.5 % error; a purely
  relative one lets a near-degenerate reach pass on a large absolute error, which
  is the case under investigation.
- **Report feature 3 as a phase-swept BAND, never a point, and never gate it.**
  The emitter's 40 Hz grid is not phase-locked to the install and the decay is
  only ~6 knots long. A point comparison reads as a 2 % failure of a correct
  model. A related error was caught in the plan's own prose: it quoted a
  *truncated running max* (2416) inside a *full-plan* band (2558–3141) and called
  the band "bracketing the recorded values" when neither recorded value is inside
  it. Both comparisons are now stated separately and labelled.

## Fix

No production behaviour changed. `git diff` on `ros_ws/` is comment-only — zero
executable lines.

**New instrument.** `tools/probes/catch_reach_replay.py`, committed per the
reusable-probe convention (the 2026-05-20 DVFS probe's findings live on `/tmp`
and its logbook references are one power-cycle from rotting; this harness is what
Phase 2 must re-run to prove its fix, and the runbook names it by path). Outputs
to `temp/probes/`. README row added. Three outputs: the install census, the
`peak_leg_*` verdict, and the replay, plus a `--self-check` that needs no bag.

**Annotation** at `trajectory_node.__init__` and the `_publish_status` KeyValue
block: which install paths write `_last_peak_*`, which six bump `move_seq`
without writing it, and — the half that misled the original investigation — that
a catch plan's predicted peaks are the fixed-shape 0.15 s decay's peaks
*whenever the reach is small enough for the decay to dominate*, **not** as a
general property. A reload-sized reach is not lead-invariant: measured
2026-07-26 through the production planner its `peak_leg_vel_mmps` runs
`82.5 → 30.1 → 24.7 → 15.2 → 14.0 mm/s` over leads `0.8 → 8.0 s`, and the bag
agrees (`move_seq` 4 and 15, the two reload catch installs, published `23.6` and
`23.8` against the toss catches' `14.2`). Losing that qualifier would have
recreated the very inference error the annotation exists to kill.

**Tests** — `tests/motion/test_catch_reach_replay_probe.py`, 10 cases: the
self-check; a mutation test proving the self-check *fails* when a production
constant moves; the closed form vs `QuinticSegment`; `φ`'s extremum; the
amplification contrast both ways; the settle scale; the segment attribution;
lead-invariance **with its reload-sized counterexample**; the census needing every
signal; and a rejection latched *before* the window still reading a clean single
install.

**Documentation** — plan Outcome sections for Phases 0 and 1 with the corrections
above; `tests/hardware/session_anomaly_fixes.md` § Section CATCH (CATCH-1
instrument health, CATCH-2 scoring with numeric PASS/ABORT, CATCH-3 the pre-fix
baseline); `tests/hardware/mvp_bench_runbook.md` open item 7 cross-referenced as
diagnosed-not-fixed.

## Outcome

Phase 0's gate (**the excursion reproduced offline from recorded inputs**) and
Phase 1's gate (**the mechanism traced end-to-end in prose**) are both met.
Phase 1's STOP condition FIRED; Phase 2 is blocked on operator re-prioritisation
and on ratification of C-CATCH-1's rewrite.

**Deployment:** `colcon build --packages-select jugglebot` + relaunch, for
source/install parity only — no behaviour depends on it. **No firmware flash. No
config regeneration** (no YAML touched). The probe, its tests and the documents
need no deployment at all.

**Hardware:** nothing required. Phase 0/1 are offline and read-only, and their
gates are met against an already-captured bag. § Section CATCH records what a
future capture needs.

### Verification

Full suite, run 2026-07-26 on the Jetson:
`source ~/Desktop/PDJ_venv/venv/bin/activate && python -m pytest tests/ -q` →
**3527 passed, 3 xfailed in 1367.68 s (0:22:47)**.

Against the 2026-07-26 baseline at `0c0c829` (3517 passed, 3 xfailed in
1351.82 s): **+10 passed**, xfail unchanged at 3. The delta reconciles exactly —
`tests/motion/test_catch_reach_replay_probe.py` is new in this phase and collects
**10 tests** (`pytest … --collect-only`, run 2026-07-26), so `3517 + 10 = 3527`
with nothing else moving. No test was weakened, skipped, xfailed or deleted, and
no existing test needed to change. Neither of the two known order/load-flaky
allocation-budget tests failed, so no isolated re-run was needed.

The suite was the last action before staging. The only edits after it were to
this entry, `logbook/INDEX.md`, the plan and the commit message — **markdown that
no test imports** (verified: the `logbook/` and `plans/active/` mentions under
`tests/` are docstring text, not file reads), correcting the drafted test count
from an estimated 3519 to the measured 3527. No re-run was therefore required.

Instrument, run 2026-07-26 (the two-sided acceptance criterion):

- `python tools/probes/catch_reach_replay.py --self-check` → `SELF-CHECK: PASS`,
  8/8 OK, exit 0.
- FLAG side, `--bag ~/Desktop/rosbags/2026-07-25_15-17-48 --toss 2` →
  `VERDICT: REPRODUCED`, exit 0; rx rms 0.0049° / max 0.0269°, lag +10.5 ms,
  model peak `+2.3236°` vs echo `+2.3224°` at the same knot, settle `−1.0784°`
  both.
- ACCEPT side, `--thrower ball_butler --toss 2` → `VERDICT: REPRODUCED`, exit 0;
  rx rms 0.0025° / max 0.0093°, ry max 0.0558° inside a `±0.1093°` span-scaled
  tolerance, lag +9.5 ms, amplification 0.174.

`peak_leg_*` consumer enumeration, re-run 2026-07-26:
`grep -rEn "peak_leg_vel|peak_leg_acc|peak_leg_jerk" --include="*.py"
--include="*.msg" --include="*.js" --include="*.md" . | grep -v
"/ros_ws/build/\|/ros_ws/install/" | wc -l` → **180** after this phase, **136**
excluding the files it created. (The implementer's report claimed "177,
unchanged at 177"; a review showed that is arithmetically impossible, since the
two new files alone contribute 37 matches. No symbol was renamed, so there is no
count-drops-to-zero audit — the load-bearing artefact is the consumer list.)
Consumers of the *published* KeyValues, which a future fix must not break:
`sim/analysis/diagnose.py`, `tests/sim/test_diagnose_trajectory.py`,
`tests/ros/test_trajectory_node.py`, `tools/probes/gravity_ff_ab_extract.py`,
and the two operator runbooks. Everything else is `FeasibilityReport.peak_leg_*`
on the producer side.

### Still open

- **Execution-order item 9 (this plan's P1 → P2) must not start** without
  operator re-prioritisation. The STOP fired.
- **C-CATCH-1's rewrite is unratified** — see Discussion.
- **The `peak_leg_*` staleness fix** (six non-writing install paths) is deferred
  as its own commit with its own entry. Enumerated above and in the plan;
  independently recorded as open item 7 in `tests/hardware/mvp_bench_runbook.md`
  from a 2026-07-09 hardware observation, so it is not hypothetical.
- **`_CATCH_TILT_THROUGH_RATE_RADPS`'s docstring** estimates the induced leg
  velocity at "~7 mm/s"; measured through the production gate it is **14.24
  mm/s** — a 2× underestimate. Still negligible against the 1000 mm/s session
  limit, so not a defect, but worth correcting when Phase 2 touches that
  constant.
- **Translation is not scored.** The reload contrast was verified on `rx`/`ry`
  only; the harness reports the commanded position span but does not score the
  swing-compensated translation to `(11.88, 2.87, 171.16)`. If Phase 2's fix
  changes the swing compensation, that half needs its own check.
