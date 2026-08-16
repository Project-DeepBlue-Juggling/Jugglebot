---
title: Anomaly-run close-out — six 9b review repairs, and the operator runbook read top-to-bottom for the first time
type: bugfix
date: 2026-07-27
status: resolved
phase: "Self-toss anomaly fixes — run close-out"
related_plan: "PROMPT-anomaly-fixes-orchestration.md"
files_changed:
  - tests/hardware/session_anomaly_fixes.md
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_levelling_frame.py
  - tests/motion/test_trajectory_planner_catch.py
  - tools/probes/catch_reach_replay.py
  - tools/probes/README.md
  - ros_ws/docs/catch_arrival_contract.md
  - plans/parked/levelling-frame-contract.md
  - plans/active/hand-command-continuity.md
  - plans/parked/catch-reach-degenerate-overshoot.md
  - plans/archived/fk-convergence-tolerance.md
  - plans/active/PROMPT-anomaly-fixes-orchestration.md
  - CLAUDE.md
commits:
  - 2dacc3a
  - 2fa13ed
subsystem:
  - motion
  - ros
  - config
tags:
  - testing
  - docs
  - safety
---

# Anomaly-run close-out — six 9b review repairs, and the operator runbook read top-to-bottom for the first time

## Summary

Two defect classes, one close-out. **(1)** Five tests and one probe self-check
case went *bit-identically inert* the moment
`planner._CATCH_TILT_THROUGH_RATE_RADPS` shipped at `0.0` — `build_catch` gates
its whole seat block on `rate > 0.0`, so every assertion about the seat's
existence, aim, decay segment and knot joins became unfalsifiable while staying
green. Each now restores a manufactured rate through a local `_set_seat_rate`
helper and carries a discriminating contrast. **(2)** The operator runbook
`tests/hardware/session_anomaly_fixes.md` accumulated append-only across eleven
phases, so rows written early contradict code that landed later. Read
top-to-bottom for the first time, it would have aborted a *healthy* sitting on at
least four rows. It gained an executable front matter (run sheet, deployment
matrix, standing rules, residual risk) and in-place supersession banners, and
this close-out repaired the contradictions the front matter itself introduced.

## Problem

The 9b review panel returned six findings against the phase-11 work plus, on this
close-out, sixteen more across three lenses. The load-bearing ones share a shape
that is worth naming, because it is the *same* shape three times over:

**A criterion that fires on correct behaviour is worse than no criterion.** It
does not merely fail to catch a defect — it routes correct work back for rework
and burns a powered sitting. Four instances, all live in the tree before this
commit:

1. **`§ CHECK CCATCH-3`'s `peak off the park`** — the superseded banner declared
   it "unaffected" by the zero-rate default. It is the ry-axis peak and moves by
   the identical `0.292407°` the banner already corrected for the ry settle, i.e.
   **2.9×** that row's own `> 0.10°` ABORT.
2. **`§ PASS / ABORT per throw` row 3** — `hard abort at > 10.5 rev`, against
   H4.4/H4.5's `10.060 < peak <= 10.60 ⇒ section abort` and `> 10.60 ⇒ E-STOP`.
   A reading of `10.55` had two incompatible responses, one of them E-STOP-grade.
3. **`§ STANDING RULES` rule 2** — "run a manual `level` after every launch and
   every relaunch, *including the mid-sitting relaunch CHECK LG-3 requires*". LG-3
   requires the relaunch *without* re-levelling; obeying rule 2 makes a healthy
   machine produce LG-3's verbatim ABORT ("the gate is wired to the Teensy flag —
   stop and report").
4. **`§ CHECK LVL-4`** — found by this close-out, not by the panel. See Discussion.

And one gap of a different kind: **`ZSEAT-2`, the single genuinely open
experiment of the run, had an unscored verdict band.** Its criterion was a fixed
count (`≥ 12/19`) against a capture that only mandates `n ≥ 12`. At n = 12 with 7
catches the rate is `0.583`, which satisfies neither `≥ 0.63` nor `≤ 0.58` — no
verdict, at the bench, for the one question the section exists to answer. Both
the operator and contract lenses found this independently, from different
starting points; that convergence is why it was fixed first.

## Root Cause

One cause, two surfaces.

**The tests**: `_CATCH_TILT_THROUGH_RATE_RADPS` was changed from `0.07` to `0.0`
in `e58ed89` as an operator decision. `planner.build_catch` gates its entire
through-seat block on `smag > 1e-9 and rate > 0.0 and decay > 0.0`
(`planner.py:1108`), so at a zero rate the block is unreachable for *every*
caller and every receive tilt. Any assertion whose subject is inside that block —
the arrival twist, the decay segment, the reach→decay knot join, the settle
overshoot — became unfalsifiable. Nothing failed, because nothing *could*. A
green test that cannot fail is worse than a deleted one: the deleted one is
visible in the diff.

**The runbook**: it was built append-only, one phase at a time, by eleven agents
that each read only their own section. That is the right protocol for *writing*
(it is what kept eleven phases from clobbering each other) but it has no reader
of the whole. Every number a later phase moved left a stale copy in an earlier
phase's section, and nobody was positioned to see the pair. The four
abort-on-correct-behaviour rows are all instances of that, and so is the F5
number the panel caught in `catch_arrival_contract.md`.

## Discussion

### The convergent finding went first, deliberately

Two reviewers reaching the same defect from unrelated lenses (one reading the
document as an operator at a bench, one auditing contract three-part structure)
is the strongest signal the panel produces. ZSEAT-2's gap was verified
arithmetically before anything else was touched: at the mandated minimum `n = 12`
the achievable rates bracketing the band are `7/12 = 0.5833` and `8/12 = 0.6667`,
and `0.5833` satisfies neither predicate. The fix states the criterion as a
**rate** with an explicit third branch (INCONCLUSIVE), and the same edit lands in
`catch-reach-degenerate-overshoot.md`'s Phase-4 gate cell so the two cannot drift
apart again. Stating it only in the runbook was rejected: the plan's gate cell is
what closes the phase, and a phase that cannot be closed from its own sitting is
the failure mode.

### LVL-4: the instrument the row never had, and the premise that was backwards

The operator lens flagged (MEDIUM) that three run-sheet rows carry numeric gates
with no instrument — `grep -rn rigid_body_poses tools/probes/` returns nothing.
The obvious disposition was to downgrade them to REPORT and move on. **That would
have been wrong, and building the instrument is what proved it.**

The brief's own rule is that an instrument's acceptance criterion is two-sided:
it must read correctly on the shape it must FLAG *and* on the shape it must
ACCEPT. So rather than assert the gap, a `/rigid_body_poses` reader was written
and run against the reference bag (`~/Desktop/rosbags/2026-07-25_15-17-48`,
46 161 samples over 290.3 s, 0 failures). Two things fell out:

- The reader works and cross-confirms the whole geometry story independently of
  the FK path: `ry_deg max +11.33°` against the reload's commanded `−10.64°`
  settle, `rx_deg` inside `[−3.63, +1.40]°`. That is the accept-side validation.
- **The parked Platform sits `0.087°` off the mocap `Base` body** (0.132° median
  over the quiet 5–40 s window) — while the inclinometer measured **0.782°** off
  gravity at that same physical state (the bag's own `/gravity_offset`:
  `hypot(0.013592, 0.001207) = 0.013646 rad`). Both cannot be level.

So `Base` is itself ~0.78° off gravity; its local frame is a QTM body-definition
artefact, not a plumb line. LVL-4 assumed the opposite ("it must **not** track the
commanded `−0.78°`, because the correction's whole purpose is that a commanded
`−0.78°` *is* physically level"). But the correction moves the platform
*physically* — that is what "is physically level" means — so a correctly-levelled
post-fix park is **expected** to read ≈ 0.78° against `Base`, which is verbatim
LVL-4's old ABORT: *"the platform is physically tilted by ~0.78° while parked …
Stop the sitting."*

**Hypothesis withdrawn mid-phase.** The working assumption on opening this
finding was "the row is fine, it just lacks a command". After the measurement that
did not survive: the row is not merely uninstrumented, its PASS band is the
*pre-fix* reading. Rescuing it by re-pointing the gate at `≈ 0.78°` was
considered and rejected — the absolute reference is an unmeasured constant (the
QTM body definition, whose stability across sessions is unknown), whereas LVL-3's
reference is the published offset and *is* measured. LVL-4 is therefore
**REPORT-only** with named signatures (`≈ 0.087°` unchanged ⇒ the correction never
reached the legs; `≈ 1.56°` ⇒ double-applied), and the gated levelling verdict
stays with LVL-3, which is instrumented. Tradeoff accepted: the run loses a hard
gate on an instrument that does not share the FK path. That is a real loss, and it
is the honest one — the alternative was a hard gate that stops a working sitting.

### ZSEAT-4 / ZSEAT-2 flatness: the recipe exists, it just was not written down

The reviewer proposed adding an explicit `--post 1.5` invocation of
`catch_reach_replay --csv` for the release window. Measured instead of assumed:
`--thrower ball_butler --toss 2 --post 1.5` still ends at `t_rel_release +0.026 s`,
because that CSV is the *model-vs-commanded* series and is trimmed to the span
where the model is defined (the plan). Raising `--post` cannot extend it. The
instrument that *does* work is `levelling_tilt_bag_check.py --t0/--t1` — its
plateau table's `span_deg` is exactly the "flat to `< 0.02°`" number, verified on
a 3.0 s window (`121 samples, span_deg 0.0000, FK failures 0`). The runbook now
carries that recipe, with the explicit warning to ignore that probe's `VERDICT:`
line, which scores the *park* against the offset and is meaningless on a 0.2 s
window. Trusting the reviewer's stated mechanism would have shipped a recipe that
silently produces nothing.

### R2 — the repair worked, its stated mechanism did not

The probe's case-9 repair comment claimed a plan-frame fallback "produces three
segments and a ~2.3 deg excursion". Rebuilt through the production planner: the
excursion is **0.000000°**, not 2.3°. `build_fixed` leaves the rate to the
planner, so C-CATCH-1 throttles the fallback to `2.5·scale/T = 0.009203 rad/s` —
precisely the ratio at which a rest-seeded quintic first leaves its seed on the
far side, so the bound pins the excursion to zero *by construction*. The 2.3°
belongs to `build_replay`'s deliberately-unbounded requested-rate path. The repair
still discriminates (3 segments, settle `−0.81817503` vs `−0.77878414`), so the
test is sound; but a future session reading the comment would treat the two
load-bearing conjuncts as redundant next to the excursion check and drop one,
re-opening the exact vacuity the case was landed to close. The comment now names
what actually discriminates. This is the *same* failure as F5 in the same review
round — a stated mechanism trusted instead of measured — which is why it was worth
a fix rather than a shrug.

### CLAUDE.md — the clause was right in intent and false in its two examples

The implementer added a clause narrowing the "docs-only, so no tests needed"
exemption, citing `tests/sim/test_logbook_search.py` and asserting that a
malformed front-matter block or a broken `INDEX.md` row "is a test failure, not a
documentation nit". Traced rather than accepted: `sim/analysis/logbook_search.py`
puts `INDEX.md` in `_SKIP_FILES`, and `load_entries` silently `continue`s past any
entry lacking a `title`, while the test asserts only `len(entries) >= 2` plus the
shape of the alphabetically-first entry (a 2026-03-30 file). **Both cited examples
pass green.** The clause would have licensed exactly the unjustified exemption it
was written to stop.

Two forks, both decided against the cheaper option:

- *Revert the hunk* (the contract lens's suggestion, on scope grounds) was
  rejected because the lesson is real and this run generated it. The concrete
  failure mode of reverting: the next finalizer repeats the inference.
- *Keep it as written* was rejected outright — a normative document carrying a
  false verification mechanism is worse than one carrying none, because it
  converts "I should check" into "I already checked".

It was rewritten to state the traced mechanism, and — answering the scope
objection — landed as its **own commit** so `git blame` on the pytest rule points
at a process decision rather than at a runbook coherence pass.

### What was deliberately NOT fixed

`sim/plant/mujoco_plant.py:130` still hardcodes the pre-Phase-3 hand prime
`9.858 * 2π * 5.21`, and `tests/sim/test_hand.py:69` pins the same literal, while
the source of truth moved to `9.9594` in `94fe817`. Verified as a real defect. It
is deferred because it is a `sim/` **production** change plus a test change — a
separate logical unit with its own rollback granularity and its own control-system
question (sim fidelity for the hand path), and folding it into a documentation
close-out would make the revert of either impossible. It cannot affect the coming
sitting (nothing on the run sheet touches the sim) and is recorded as
residual-risk item 11 so it is met with numbers attached rather than rediscovered.

The regression lens also reported (HIGH) that a concurrent process was mutating
and restoring `planner.py` in the shared tree during the review window. That is
correct as an observation and is a process hazard, not a code defect: it was a
fellow reviewer running mutation batteries in the shared tree (the timestamps and
the observed `if False and smag > 1e-9 …` line match the contract lens's MUT1
exactly). Mitigated rather than fixed: `planner.py` and `trajectory_node.py` were
md5-verified byte-identical to HEAD twice, minutes apart, and again immediately
before staging; every path was staged explicitly, never `git add -A`.

## Fix

**The six 9b repairs** (implementer's work, verified here):

| # | repair |
|---|---|
| F1 | `§ CHECK CCATCH-3` gains a fifth banner row for `peak off the park` (post-fix `10.6363°`, `0.000000°` past target) plus a STALE clause for `arrival-rate bound`, which at a `0.0` default prints `MANUFACTURES NOTHING` and structurally cannot print `BINDS` |
| F2/F4/F6 | Five tests restore a manufactured seat rate through a local `_set_seat_rate` helper (the module constant, never the kwarg — the kwarg takes C-CATCH-1's deliberately-unbounded requested branch) and each carries a discriminating contrast |
| F3 | Probe self-check case 9 rebuilds `build_fixed` once at the capture-record `0.07` (try/finally) and once at the live default, so a plan-frame fallback can no longer print `OK` |
| F5 | `ros_ws/docs/catch_arrival_contract.md`'s `rate 0.07` row took acc from the pre-fix column and jerk from the post-fix one; re-measured through the production planner and corrected `139.7 → 142.0` |

**The close-out repairs** (this entry):

| lens | repair |
|---|---|
| operator HIGH | Standing rule 2 carves out the stage-5 CAP-RELAUNCH explicitly |
| operator HIGH | `CLAUDE.md`'s docs-only clause rewritten to the traced mechanism (own commit) |
| operator MED | LVL-4 gains a validated `/rigid_body_poses` reader and is demoted to REPORT; ZSEAT-4 and ZSEAT-2-flatness gain the `levelling_tilt_bag_check --t0/--t1 → span_deg` recipe; the "read by eye" sentence is replaced by a three-row instrument table |
| operator MED | `§ PASS / ABORT per throw` row 3: `> 10.5` → `> 10.60`, with the section-abort band named |
| operator+contract LOW (convergent) | ZSEAT-2 scored as a **rate** with an INCONCLUSIVE branch, in the runbook and in the plan's Phase-4 gate cell |
| operator LOW | `--t0` documented as **seconds from bag start**, with the two-pass recipe |
| operator LOW | Run-sheet HAND-1 row carries HAND-4's row-4/row-7 qualifier |
| contract LOW | Deployment matrix row A: `PF<n>_STALE` on the run sheet, `INSTALLED_STALE` in the per-section pre-flights |
| regression MED | Probe case-9 comment names the conjuncts that actually discriminate |
| regression MED | `§ Section ZSEAT` head: "four rows" → "five (and a sixth stale)" |
| regression MED | `--gate` fixed-shape branch is **five** cases, not four (runbook + `tools/probes/README.md`) |
| regression LOW | `hand-command-continuity.md` gains an in-place banner above the superseded 546/208 arm-window table |

Refuted or deferred: the `mujoco_plant` prime (deferred, own unit), and the
concurrent-mutation report (process hazard, mitigated, not a code defect).

## Verification

Full suite, run 2026-07-27 on the Jetson in the project venv:
`pytest tests/ -q` — **3947 passed, 3 xfailed in 1383.73 s (23:03)**, exit 0.

**The delta against the `1e78f3f` baseline is `+4` passed, and the prediction
that it would be zero was wrong.** Baseline: `pytest tests/ -q`, run 2026-07-27,
**3943 passed, 3 xfailed in 1399.35 s** — measured at `1e78f3f`, i.e. *before*
the implementer's test repairs, which is what the prediction missed. The `+4` is
accounted for exactly and is entirely parametrisation: four
`@pytest.mark.parametrize('seat_rate', (0.0, _SEAT_RATE_RADPS))` decorators, each
turning one test into two legs (`test_dynamic_target_catch_knots_pump_accepted`,
`test_every_catch_knot_pump_accepted`,
`test_catch_hold_after_false_returns_to_neutral`,
`test_too_tight_lead_rejected_too_fast`). Collected-item arithmetic closes:
`3946 = 3943 + 3` at baseline, `3950 = 3947 + 3` now, and
`pytest tests/ -q --collect-only` reports **3950**.

**The xfail count stayed at 3.** No test was weakened, skipped or deleted to reach
green, verified independently of the reviewers: the diff adds zero `xfail` /
`skip` / `skipif` markers, assert counts rise in all three touched test files
(71→75, 83→88, 455→458) and fall in none, and a sorted diff of test-function
names against HEAD is empty in all three (no removals, no renames). Neither
order/load-flaky allocation-budget test failed, so no isolated re-run was needed.

Instrument health, all run 2026-07-27 in the project venv:

- `python tools/probes/catch_reach_replay.py --self-check` → `SELF-CHECK: PASS`,
  10/10, exit 0 — re-run **after** the case-9 comment edit.
- `python tools/probes/hand_stroke_timeline.py --gate` → exit 0, both
  `GATE PASS` lines, and **five** labelled fixed-shape cases (`clean`,
  `overshoot`, `short-flight`, `braking-prelude`, `deep-brake`) — which is how the
  "four cases" drift was caught.
- `python tools/probes/levelling_tilt_bag_check.py --bag <ref> --offset … --t0 78 --t1 81`
  → `samples 121  span 3.0 s  FK failures 0`, plateau `span_deg 0.0000` — the
  two-sided proof that the ZSEAT-4 / ZSEAT-2-flatness recipe produces a number.
- `pytest tests/sim/test_logbook_search.py -q`, run 2026-07-27 → **24 passed in
  0.19 s** (re-run after this entry and the INDEX row landed, since that file
  parses the real `logbook/` directory).

Production source untouched: `git diff --stat -- ros_ws/src/ sim/ controller/ config/`
is empty, and `planner.py` / `trajectory_node.py` were md5-verified byte-identical
to HEAD immediately before staging.

## Related

- `plans/active/PROMPT-anomaly-fixes-orchestration.md` — the run's own record;
  execution-order table now carries a Status column and § Run close-out — Outcome.
- `tests/hardware/session_anomaly_fixes.md` — the deliverable; § THE RUN SHEET is
  the authority for the operator sitting.
- `logbook/2026-07-27-velocity-continuous-prelude.md`,
  `logbook/2026-07-26-catch-through-seat-rate-zero.md` — the two phases whose
  numbers this close-out reconciled across documents.
