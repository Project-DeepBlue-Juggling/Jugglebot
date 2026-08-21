---
title: The hand-catch prime becomes the derived stroke top — and the review found the profile model three of its neighbours were sized against was the wrong one
type: bugfix
date: 2026-07-26
status: resolved
phase: "Self-toss anomaly fixes — hand-command-continuity Phase 3"
related_plan: "hand-command-continuity.md"
files_changed:
  - config/hardware_config.yaml
  - config/generate_config.py
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/hand_stroke.py
  - ros_ws/src/jugglebot/jugglebot/catch_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/reload_coordinator_node.py
  - ros_ws/src/jugglebot/jugglebot/toss_sequencer.py
  - tests/motion/test_hand_stroke.py
  - tests/ros/test_catch_coordinator_node.py
  - tests/ros/test_reload_coordinator_node.py
  - tests/ros/test_toss_coordinator.py
  - tests/hardware/session_anomaly_fixes.md
  - tests/hardware/session_phase7_reload.md
  - plans/archived/hand-command-continuity.md
commits:
  - 94fe817
subsystem:
  - config
  - ros
  - motion
tags:
  - testing
  - docs
  - safety
---

# The hand-catch prime becomes the derived stroke top

## Summary

`jugglebot_operational.hand_catch_prime_rev` was a hand-maintained `9.858` rev
while the quantity its own YAML comment claimed it was — x3, the throw stroke's
end and the catch trajectory's **first sample** — is `9.9594` rev. It had drifted
`0.101403` rev = **3.207 mm** for the life of the constant, so every "catch from
rest at the top" opened with a real 76.5 ms prelude: the no-op case was never a
no-op. The prime is now derived (`HAND_STROKE_TOP_REV`, emitted by
`generate_config.py`) with the YAML key kept as a documented override and a
three-route drift guard. **The hand parks 3.2 mm higher; that is the entire
motion change.**

The review panel then found something the phase had not gone looking for: the
`sqrt(a·Δx)` "peak ascent speed" that three of the prime's neighbours are sized
against — a guard comment, a test, and a brand-new operator bench row — is a
**bang-bang** figure, and the firmware ships a **quintic**. Both a duration
(0.63 s vs the real 0.758 s) and a velocity (31.56 rev/s vs the real 24.63 rev/s)
were wrong, in opposite directions, and the phase had propagated both while
updating them for the new prime.

## Problem

Three distinct defects, one landed fix each.

**P1 — the prime had drifted.** `JB_OP_HAND_CATCH_PRIME_REV = 9.858` against
x3 = `9.95940313273228`. `Trajectory.h`'s `makeSmoothMove` prepends a prelude
from wherever the hand physically is to the first sample of whatever is being
packed, and a kind-1 catch begins at x3. At 9.858 that prelude was **0.101403
rev / 3.207 mm / 76.5 ms** of real commanded travel on every primed catch,
charged against the arm-fit budget before the catch profile started.

**P2 — the profile model.** `MIN_THROW_EVENT_DELAY_S`'s justification comment
bounded the full-stroke prelude at "~0.63 s (100 rev/s² triangular)". The
firmware solves `T = sqrt(|Δ|·QUINTIC_S2_MAX/A)` (`Trajectory.h:257`), which is
**0.758 s** — the stated upper bound was violated by 20 %, in the unsafe
direction. The same wrong model appears as a *velocity*: `sqrt(a·Δx)` = 31.56
rev/s is quoted as "the prelude's peak ascent speed" in
`_THROW_STROKE_VEL_RPS`'s comment and in
`test_stroke_watch_threshold_clears_smooth_move_prelude`, while the quintic
actually peaks at `|Δ|·1.875/T` = **24.63 rev/s**, 22 % lower.

**P3 — a bench row calibrated on the wrong figure, and a test that would go red
on an improvement.** The phase's new operator row H3.7 was written `PASS <= 35`
rev/s against the 31.56 bound; and
`test_prime_at_the_stroke_top_costs_no_commanded_prelude_travel` pinned the
YAML's 4-decimal **rounding artefact** (`residual == approx(3.13e-6)`,
`duration == approx(0.05)`), so writing a *more precise* YAML value would have
turned it red.

## Root Cause

P1: a derived quantity was stored as an independently-editable literal with
nothing comparing the two. The YAML comment stated the intent ("top of stroke")
correctly the whole time; only the number rotted.

P2/P3 share a root cause of their own: `sqrt(a·Δx)` and `2·sqrt(Δ/a)` are the
closed forms for a **bang-bang** profile, and they are the forms that come to
hand when you sketch "constant-acceleration move over Δ". `makeSmoothMove` is a
minimum-jerk quintic whose duration is chosen *so that* peak acceleration equals
the limit — which makes it slower and slower-peaking than a triangle over the
same stroke by fixed factors (`sqrt(QUINTIC_S2_MAX)/2` = 1.20 on duration,
`1.875/sqrt(QUINTIC_S2_MAX)` = 0.780 on velocity). Nothing in the codebase named
either factor, so each site re-derived the wrong one independently.

## Discussion

### The convergent finding decided the shape of the fix

Two reviewers, from different starting lenses (safety-window enumeration and
contract integrity), independently landed on `toss_sequencer.py:223`'s "~0.63 s
triangular". A third route to the same root cause came from the contract lens
alone: the *velocity* form of the same error, in the test and the bench row. That
convergence is why the fix is **a named model in `hand_stroke`**
(`smooth_move_peak_vel_rps`, `smooth_move_peak_vel_bound_rps`) rather than three
corrected literals. Three literals would have been half the work and would have
left the fourth site — whichever one gets written next — free to re-derive the
triangle a fourth time. `hand_stroke` already owns `smooth_move_duration_s` and
already exists *because* this codebase has demonstrably duplicated hand-stroke
timing; adding the velocity form to the same module is the cheap completion of a
decision already made.

**The tradeoff accepted:** two functions where one would do. Keeping
`smooth_move_peak_vel_bound_rps` at all is deliberate — `_THROW_STROKE_VEL_RPS`'s
job is to clear the prelude, and clearing a conservative *bound* is genuinely the
right thing for a guard to do. Deleting the bound and re-sizing the guard against
24.63 rev/s would have tightened a safety threshold for tidiness, which is the
opposite of the trade this guard was chosen for. So both figures ship, named,
with the docstring stating which one a bench criterion may use and which one a
guard may use. The failure that shapes this: a future reader who finds only
`smooth_move_peak_vel_rps` will "fix" the guard comment's 31.56 to 24.63 and then
wonder why the threshold has so much margin — and may shave it.

### Why the bench row was the finding that mattered, not the test assertion

The review split on the velocity error's severity. One lens called it harmless:
the guard clears the bound, the bound exceeds the real peak, so the guard is
conservative in the safe direction — true, and it is why **no motion is unsafe
today**. The other lens escalated it because the phase had just calibrated an
*operator-facing verdict* on the wrong figure.

That is the reading that wins, on a rule this run is explicit about: an
instrument has to give the right answer on the shape it must accept as well as
the shape it must flag. H3.7 at `PASS <= 35` accepts a **26 % overspeed** (a
31 rev/s reading) as "on model" — and overspeed on a smooth move is the
signature of a re-seeded or clobbered profile, which is the entire failure class
`hand-command-continuity` exists to close. An instrument that scores that PASS is
worse than no instrument: it converts the plan's own target defect into a green
row. The row is now `PASS <= 30`, DEBRIEF `30-40`, ABORT `>= 40`, with the
commanded 24.63 rev/s stated as the expectation and a *low*-side note (`< 20`
rev/s means it was not a full stroke) so the row cannot be passed by a capture
that never ascended.

### Restating a derived literal in a test — adjudicated, not assumed

The implementer flagged one test change for the panel: `prelude_peak ==
approx(31.4)` had to become `approx(31.56)` because the prime it restates moved.
The run brief lists "any test that would have to be weakened" as a STOP.

**Ruling: ACCEPT**, on three verified facts rather than on the brief's wording.
`git diff -U0 -- tests/` deletes exactly one assertion line across the whole
phase (this literal); the tolerance stays `abs=0.1`; both behavioural
inequalities and both latch samples (35.0 / 45.0 rev/s, still correctly
bracketing) are untouched; and no `xfail`, `skip` or `mark` appears anywhere in
the diff. The literal is a restatement of a value derived from the config
constant this phase deliberately changes — mechanically the same ripple as the
regenerated `hardware_config.py` — and a phase chartered to "derive this instead
of hand-maintaining it" is impossible under the strict reading. **The general
rule this sets:** a literal restating a derived config value may be restated in
the same commit that changes the config, provided no tolerance widens and no
assertion is removed. Anything else is a STOP.

The same lens also raised, at NOT-PROVEN confidence, whether the brief's
"alters commanded motion magnitudes" STOP fires here. It does not, and it is
worth saying why in writing rather than resolving it silently: the run's own
execution order charters this phase as "prime derived from stroke geometry",
whose entire product is moving that magnitude — a STOP that fires on a phase's
chartered deliverable would make the phase unexecutable. What the operator is
owed instead is the physics, stated: **peak commanded acceleration is IDENTICAL
at 100.0 rev/s²** (the duration formula solves for exactly that, so a taller
stroke buys duration, never a harder command), peak commanded jerk *falls*
1377.5 → 1370.5 rev/s³, peak commanded velocity rises 24.50 → 24.63 rev/s
(+0.5 %), and end-stop headroom drops 1.2420 → 1.1406 rev (39.3 → 36.1 mm).

### The override-vs-derived fork, and why the enforcement point is a test

The YAML key stays an **explicit override** rather than becoming derived-only.
Derived-only fails two ways, and the second is worse than the drift being fixed:
delete the key and the documented intent plus the operator's bench escape hatch
go with it; keep it but have codegen ignore it and a future engineer editing
`9.9594 → 9.5` for a bench trial regenerates and **nothing changes** — a silent
no-op, which also breaks CLAUDE.md's "hardware_config.yaml is the single source
of truth". A third option, hard-failing `generate_config.py` on mismatch, was
rejected because codegen sits on the critical path of *every* YAML edit: an
operator doing unrelated tuning would be unable to regenerate until they also
fixed the prime, with no escape hatch.

**The honest cost of that choice, recorded because it is a real weakness.** The
enforcement point is a *test*, not code — nothing prevents a future YAML edit
from re-drifting except a red suite. That is a weaker contract than the K1-K6
pattern CLAUDE.md holds up as canonical (normative doc + one code enforcement
point + test). What makes it acceptable here is that the override is a *feature*:
the invariant is "equal unless someone deliberately says otherwise", and only a
test can express "deliberately" — code enforcement would have to forbid the
thing the key exists for.

### Two hypotheses that did not survive

**Withdrawn: "codegen dirtied an external repo, origin unidentified."** The
review found `~/Desktop/BallButler/ball_butler_main/hardware_config.h` holding
the **pre-change** `9.858f` with an mtime 21 minutes *after* the in-repo codegen
run, and flagged it as an unexplained stale-YAML codegen run to surface under the
parallel-session rule. The origin is identified and benign: a reviewer's own
mutation test reverted the YAML to 9.858, ran `generate_config.py`, and restored
the in-repo files from a tar — which cannot restore a delivery target that lives
outside this git repo. Re-running codegen fixed it (BB now reads `9.9594f`) and
the in-repo tree was unchanged by that run, which is also the determinism proof.
No parallel session is implicated. **The generalisable lesson is real though:**
`generate_config.py` writes outside its own repo, so any mutation test that
touches `hardware_config.yaml` must re-run codegen to restore, not `git checkout`.

**Withdrawn: "the 26.5 ms returns to Phase 1's arm-fit budget."** The
implementer's own docstring said the saving makes the arm-fit check
"conservative, never optimistic". Traced: `PRELUDE_ALLOWANCE_S` reaches
production only through `required_arm_lead_s`, whose sole caller is
`_throw_stroke_gate_ok`, which returns True immediately unless
`_throw_stroke_clear_ros` is set — and only the **self-throw** announcement
handler sets it. On the self-toss path the hand reaches x3 via the throw stroke
itself, so the prime never entered that arithmetic. The 26.5 ms is real, but it
accrues on the **primed** path (BB catch / reload) to the firmware's own fit
check at `Teensy_code.ino:533`. This matters because the arm window is 115 ms
wide at `FLIGHT_TIME_MIN_S` with ~16 ms of floor headroom; a future session
sizing `ARM_SUPPRESS_MARGIN_S` off the wrong sentence would shave a window that
gained nothing here.

### Asserting a property, not a rounding artefact

The prelude test pinned `residual == approx(3.13e-6, rel=0.05)` — the residual
between the YAML's 4-decimal `9.9594` and the full-precision x3. Confirmed
empirically: at `9.959403` the residual is `1.33e-7`, which falls **under**
`makeSmoothMove`'s 1e-6 dead-band, so the duration becomes `0.0` and three
assertions fail. A strictly-better config would have gone red, with a failure
message pointing at prelude physics rather than at YAML precision — and both the
YAML comment and the generated file display the full-precision value to any
reader, so the repo actively invites that edit. This is exactly the
false-alarm-on-improvement the drift guard's own tolerance was shaped to avoid;
the two tests disagreed with each other. The residual is now asserted as a
property (`<= tol`, `duration <= 0.05`), and the "not zero" claim — which is the
honest correction the phase fought for — is pinned where it actually lives: on
the **floor**, which any hardware-reachable residual hits regardless of YAML
precision.

### One window the constant-grep sweep could not see

`_PRIME_INFLIGHT_S = 1.2` is sized from the prime **ascent duration** but never
names the constant, so a grep-of-`JB_OP_HAND_CATCH_PRIME_REV` sweep misses it
structurally. It does not bind (commanded ascent 0.7583 s; 0.44 s of model margin,
0.15 s against the observed 1.05 s upper bound) but nothing pinned it. If a later
prime raise or Phase 4's duration-formula change pushes the ascent past 1.2 s,
the retry tick re-dispatches a kind-3 mid-ascent, rebuilding the Teensy profile
from the live position at `v(0) = 0` — the 2026-07-23 stutter, 5/12 ascents
stalled 60-70 ms with velocity reversals to −4 rev/s. Pinned against the
*commanded* duration with 1.5× headroom, deliberately not against the observed
0.68-1.05 s band: the observed upper bound includes dispatch and settle time the
model does not claim to cover, so asserting against it would go red on ordinary
telemetry scatter.

## Fix

**The derivation.** `config/generate_config.py:595` emits
`HAND_STROKE_TOP_REV = x3_m * TEENSY_LINEAR_GAIN` = `9.95940313273228` rev
alongside the x2/x5 landmarks it already derives, in **rev** rather than the
siblings' metres — deliberate unit inconsistency, because the guard compares in
rev and arithmetic inside a drift guard is exactly where a rev/mm slip hides
(this project has already shipped a 14× rev/mm bug). `hand_catch_prime_rev`
moves to `9.9594` with an 18-line override rationale.

**The guard** (`tests/motion/test_hand_stroke.py::test_catch_prime_equals_the_stroke_top`),
tolerance `5e-5` rev = 1.6 µm — 2000× smaller than the drift it catches and than
the settle band the bench gates on. Three routes, each load-bearing: the
generated constant, the host `hand_stroke` model, and x3 re-derived from the
**parsed shipped firmware header**.

**The profile model** (`hand_stroke.smooth_move_peak_vel_rps` and
`_bound_rps`), quoted by `_THROW_STROKE_VEL_RPS`'s comment,
`MIN_THROW_EVENT_DELAY_S`'s comment (now `0.758 s` quintic, with the reachable
prelude capped at ~0.17 s by the `hand_parked` re-check at THROWING entry — so
the 1.0 s floor is conservative twice over), the toss test, and bench row H3.7.

**No threshold was widened, moved or relaxed.** Every changed line in
`reload_coordinator_node.py`, `toss_sequencer.py` and `catch_coordinator_node.py`
is a comment. The `hand_parked` gate the run brief names as the physical hazard
is keyed to `JB_OP_HAND_RETRACT_REV = 0.0` — the **bottom** band — so the kind-0
off-band-dispatch path is untouched by construction, and that keying is now
pinned so a future re-key is caught.

## Verification

**Margins, each measured at both primes.** Near-band
`[9.3580, 10.3580] → [9.4594, 10.4594]`: against the 2026-07-24 parked-top
spread `[9.675, 10.044]`, margins `0.3170 / 0.3140` rev if the spread translates
with the target, `0.2156 rev = 6.8 mm` in the pessimistic stays-put reading
(which is what the test pins). Bridge headroom `1.2420 → 1.1406` rev
(39.3 → 36.1 mm) against the 11.1 rev overextension guard. Guard margin above
the prelude: bound `8.6025 → 8.4415` rev/s, commanded `15.4994 → 15.3738` rev/s.
Residual prelude at the prime: `0.101403 rev / 76.5 ms → 3.13e-6 rev / the
50 ms floor`.

**Two-sided check on the drift guard**, run 2026-07-26 — it must FLAG the drift
*and* ACCEPT a legitimate improvement. Recipe (one-off, not committed; ~20 lines,
reproduce by importing `tests.motion.test_hand_stroke`, rebinding
`hw.JB_OP_HAND_CATCH_PRIME_REV`, and calling
`test_catch_prime_equals_the_stroke_top` and
`test_prime_at_the_stroke_top_costs_no_commanded_prelude_travel` under
`try/except AssertionError`):

| prime | guard | prelude test | expected |
|---|---|---|---|
| `9.858` (the defect) | RED | RED | RED |
| `9.9594` (shipped, 4 dp) | GREEN | GREEN | GREEN |
| `9.959403` (more precise) | GREEN | GREEN | GREEN |
| `9.95940313273228` (full float) | GREEN | GREEN | GREEN |
| `9.9594 + 1e-4` (3.2 µm drift) | RED | RED | RED |
| `9.9594 + 1e-5` (0.3 µm, below resolution) | GREEN | GREEN | GREEN |

Against the **as-implemented** assertions the two precision-improvement rows were
RED (residual `1.33e-7` rev falls under the 1e-6 dead-band ⇒ duration `0.0` ⇒
three assertions fail) — that is the defect this finalize fixed.

**Codegen determinism**: `python config/generate_config.py` run twice on
2026-07-26; md5sums of `config/generated/hardware_config.py`,
`config/generated/hardware_config.h` and
`ros_ws/src/jugglebot/jugglebot/hardware_config.py` identical across runs, and
the second run left `git status --porcelain` unchanged. All four in-repo
`hardware_config.h` copies share one md5 (`e04b9dc…`), as does the external
BallButler delivery target.

**Phase-5 verdict instrument re-checked on the final tree**:
`python tools/probes/hand_stroke_timeline.py --gate` → `GATE PASS — 25/25 rows
within tolerance` and `GATE PASS — fixed-shape branch`, exit 0.

**Full suite** (`pytest tests/ -q`, run 2026-07-26 on the Jetson under
`~/Desktop/PDJ_venv/venv`): **3531 passed, 3 xfailed, 196 warnings in 1356.73 s
(0:22:36)**, exit 0, zero `FAILED`/`ERROR` lines. Baseline at HEAD `fbd8d13` was
**3527 passed, 3 xfailed in 1367.68 s**. The **+4** is exactly the four new test
functions this phase adds (`test_catch_prime_equals_the_stroke_top`,
`test_prime_at_the_stroke_top_costs_no_commanded_prelude_travel`,
`test_prime_move_leaves_the_park_band_windows_open`,
`test_prime_inflight_window_covers_the_commanded_prime_ascent`) — verified by
`grep -c "def test_"` against `HEAD` on all four touched test files, which shows
+2/+1/+1/+0. Four further assertions were added inside pre-existing tests, which
add no case count. **The xfail count is unchanged at 3** — nothing was weakened,
skipped, deleted or xfailed. Neither order/load-flaky allocation-budget test
failed, so no isolated re-run was needed.

**Deployment**: `colcon build --packages-select jugglebot` **+ relaunch** (the
launch runs the installed copy). **No firmware flash** — `JBOp::HAND_CATCH_PRIME_REV`
is referenced by no `.ino`/`.h`/`.cpp` in any of the three sketches; it is a dead
`constexpr` codegen delivers for completeness. Bench validation is
`tests/hardware/session_anomaly_fixes.md` § CHECK HAND-3, deferred to the
operator.

## Related

- Plan: `plans/archived/hand-command-continuity.md` — Phase 3
- Phases 1-2: `logbook/2026-07-26-hand-command-continuity-arm-gating.md`
- Phase 0 probe: `logbook/2026-07-26-hand-stroke-timeline-probe.md`
- Operator runbook: `tests/hardware/session_anomaly_fixes.md` § CHECK HAND-3

### Known-stale duplicates, deliberately out of scope

- `sim/plant/mujoco_plant.py:129-130` hardcodes `9.858 * 2π * 5.21`, pinned by
  `tests/sim/test_hand.py:69`. **Do not fix by syncing the number**: the sim's
  conversion omits `linear_gain_factor = 1.035` entirely, computing 322.7 mm
  where the firmware puts 9.858 rev at 311.8 mm. That 10.9 mm gain error exists
  today, independent of the 20 mm stroke inset Phase 0 recorded, and a
  number-only sync would make it *look* fixed. A third candidate contributor to
  the sim-catch fidelity gap; wants its own scoped fix deriving the sim prime
  from `TEENSY_LINEAR_GAIN`. The prime move made it *less* wrong (+10.93 →
  +7.71 mm), not more.
- `plans/archived/reload-action-catch-latch.md:44` cites the old 9.858. Left
  untouched — a plan a parallel session may be editing.
- `ros_ws/.../archived/catch_dropped_ball_node.py:24` and
  `archived/catch_from_ball_butler_node.py:42` hardcode 9.858 under their own
  names; neither is referenced by `setup.py` or any launch file.
- `~/Desktop/BallButler/ball_butler_main/hardware_config.h` is a codegen
  delivery target outside this repo and now shows modified in the BallButler
  repo. Cosmetic there too (BB firmware does not read the constant); the
  operator decides whether it needs its own commit.
## Close-out — 2026-08-21

**Status `in-progress` → `resolved`.** Gated on the operator's `colcon build` +
relaunch and § CHECK HAND-3. Both happened at the 2026-07-27 sitting and all
seven rows passed: prime settles **9.9571–9.9586 rev**, spread **0.05 mm**,
**zero** overshoot, peak prime velocity on model to 4 %.
Verdicts: `logbook/2026-07-28-anomaly-fixes-validation-sitting.md`.

**The sim-side follow-on named above is now owned.** The MuJoCo plant's
`_hand_prime_mm` was still the pre-Phase-3 `9.858 * 2π * 5.21` (and with the
wrong gain — no `LINEAR_GAIN_FACTOR`), with `tests/sim/test_hand.py` pinning the
same literal. Both are **derived from `HAND_STROKE_TOP_REV` since 2026-08-21**.
The remaining question — whether the sim should keep its 20 mm stroke inset at
all, which is what makes the derived prime 335 mm rather than the firmware's
315 mm — is re-homed to
`plans/parked/hand-trajectory-generator-overhaul.md` § 6 *Inherited findings*.
