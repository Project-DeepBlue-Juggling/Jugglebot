---
title: The seat rate becomes a config key, and the A/B that needs it gets an instrument
type: feature
date: 2026-07-25
status: tuned
phase: "Self-toss anomaly fixes — Phase C (seat-experiment prep)"
related_plan: "catch-reach-degenerate-overshoot.md"
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/docs/catch_arrival_contract.md
  - tests/motion/test_trajectory_planner_catch.py
  - tests/motion/test_ball_arrival_offset_probe.py
  - tests/hardware/session_anomaly_fixes.md
  - tools/probes/ball_arrival_offset.py
  - tools/probes/catch_reach_replay.py
  - tools/probes/README.md
  - plans/parked/catch-reach-degenerate-overshoot.md
commits:
  - 2bc3ba2
subsystem:
  - motion
  - config
tags:
  - testing
  - docs
---

# The seat rate becomes a config key, and the A/B that needs it gets an instrument

## Summary

The manufactured catch seat rate is now `trajectory_op.catch_seat_rate_radps` in
`config/hardware_config.yaml` (generated as `JB_TRAJ_CATCH_SEAT_RATE_RADPS`,
shipping at **`0.0`** — commanded behaviour is byte-identical), bound once at
import into the existing `planner._CATCH_TILT_THROUGH_RATE_RADPS`. The
pre-registered seat A/B that Phase 4 left open gets a scored protocol
(`tests/hardware/session_anomaly_fixes.md` § SECTION SEAT-EXP) and, because its
central measurement had **no runnable command anywhere in the repo**, a new
committed instrument: `tools/probes/ball_arrival_offset.py`.

Nothing about commanded motion, park bands, arming or the kind-3 path is touched.

## Context

Phase 4's hardware sitting (2026-07-27) ended with `ZSEAT-2` ABORTed and its
attribution **INCONCLUSIVE**: three eye-confirmed bounce-outs on a stationary
10.8° reload rim, against a measured 26–39 mm Ball-Butler warm-up drift in
arrival `x` that explains them equally well. The capture cannot separate the two
because it contains no throws at a large `+x` offset with a non-zero seat. The
A/B that settles it needs to move the seat rate *during a sitting* — and until
this phase the only way to do that was to edit `planner.py` at the bench.

## Discussion

### Why config, and why the binding point is the whole design

Two failure modes drove this, not tidiness. A source edit made mid-sitting is
invisible in the **installed** tree, so *"which rate did that block actually
run?"* becomes unanswerable from the capture — and the edit can be forgotten and
shipped as an accidental permanent default. Config-keying makes the deployed
value greppable out of `ros_ws/install/.../hardware_config.py`, which is what the
deployment rows `SEAT-EXP-1` and `SEAT-EXP-3` read.

The genuinely load-bearing decision is **import-time vs call-time binding**, and
it is a control-integrity question rather than a style one. Reading `hw.` inside
`_catch_arrival_rate` reads better and is strictly worse: every C-CATCH-1 test
reaches the bound by monkeypatching the *module attribute* (`_set_seat_rate`),
never by the kwarg — because the kwarg takes the deliberately-**unbounded**
requested branch. A call-time lookup would ignore all twelve of those
monkeypatches across three files and take the entire `test_ccatch1_*` block
vacuous **while staying green**. That is the exact trap the 2026-07-26 zeroing
phase spent most of its effort closing, so it is pinned executably
(`test_the_module_attribute_is_still_the_monkeypatch_surface`) rather than
documented.

Section choice was `trajectory_op` (`JB_TRAJ_`) rather than
`jugglebot_operational` (`JB_OP_`, which the brief suggested with an "e.g.").
Both already have `HW_SECTIONS` rows so neither needed a new one. The failure
`trajectory_op` prevents: the three sibling catch-plan keys
(`catch_reach_freeze_s`, `catch_settle_hold_s`, `catch_reach_envelope_mm`) all
live there, so an operator grepping `catch_` in that block finds three keys, does
not find the seat rate, and concludes it is still a source edit — the exact state
this phase exists to remove.

### A hypothesis that was withdrawn: "three drift guards, three links"

The phase shipped three guards on the intent that each breaks a different link
(YAML→generated, generated→module, module-as-patch-surface). Review showed the
middle one **did not hold at the shipped value**. Its assertion was
`planner._CATCH_TILT_THROUGH_RATE_RADPS == approx(hw.JB_TRAJ_CATCH_SEAT_RATE_RADPS)`,
and with both sides `0.0` a re-hardcoded `= 0.0` literal in `planner.py` satisfies
it exactly — i.e. it passed against the very mutation (a) its own docstring named.
It would only have started to bite the day somebody flipped the YAML: at the
bench, during the sitting, when nobody runs pytest.

One reviewer claimed to have mutation-verified this guard and reported it
catching the mutation. That claim does not survive arithmetic (`0.0 == 0.0`), and
the disagreement was resolved by running it rather than by weighing confidence.
The guard is now **value-independent**: it monkeypatches the generated constant
to a sentinel, `importlib.reload`s the planner, and requires the module attribute
to follow — which holds at `0.0`, at `0.07`, and at whatever the experiment lands
on. Reload is safe here and that was checked, not assumed: `planner` defines zero
classes (so no `isinstance` identity hazard) and nothing in the repo does
`from ...planner import <symbol>` (so no stale from-import bindings).

### The instrument, and the two-sided criterion it initially failed

The brief asked for the arrival-offset scoring "command included". No such
command existed — the sitting's own gap list records that the number *"appears in
no log line"* and it was computed by hand, once. Citing a non-existent instrument
is precisely the § CHECK LVL-4 defect the 2026-07-27 close-out caught, so the
probe was built and validated against the real sitting bag.

The instrument's acceptance is `--self-check`, and the runbook gates the whole
scoring pass on it (`SEAT-EXP-4`: *any BAD ⇒ do not score*). Review found its
most important case was **vacuous**, and this was confirmed by mutation rather
than by reading: case 2 grafted a synthetic bounce-out at `z = 700..739`, i.e.
*below* the fit plane, where the band filter (`plane <= z <= plane+band`) drops
those samples whether or not the descending-branch cut exists. A mutant probe with
the cut deleted scored **8/8 PASS, exit 0**. An instrument validated only against
the shape it must accept will score a correct fix as a failure — or, here, accept
a confounded estimator — and that burns a powered sitting.

Fixed by grafting back **inside** the band (which is what a ball deflected off the
rim physically does — ball 11 reappeared 345 mm above its free-fall prediction)
and additionally requiring the naive fit that kept those samples to move. Verified
both ways: mutant now `FAIL (1 case)`, exit 1; unmutated `PASS`, exit 0.

The bias this protects against is not noise but **confounding**: a bounce-out's
post-contact excursion is ~70 mm and points differently on different balls, so
including it perturbs each track by an amount *correlated with the outcome being
scored*.

### Criteria that fire on correct behaviour — the recurring defect class

Two gates in the proposed protocol were of this class, and both were replaced
with measured thresholds rather than adjectives:

1. **Burn-in.** The sitting proposed *"successive-throw change < 10 mm"*. On its
   own demonstrably-settled plateau the successive changes run
   `0.3, 15.6, 0.8, 11.5, 10.0, 9.0, 6.3` mm — **three of seven over the line**.
   A plateau is the absence of a *trend*, not of scatter (throw-to-throw sd is
   9–11 mm). Replaced by a trend slope (`|b| <= 3 mm/throw`), which separates the
   measured warm-up (`−7.13`) from the measured plateau (`−1.75`) cleanly. Both
   the accepted and the **rejected** criterion are pinned in the probe self-check
   so the bad one cannot be re-adopted from prose.
2. **Matched arms.** `SEAT-EXP-6` originally gated on a flat `≤ 5 mm` gap between
   block means. At the expected pooled sd (9–11 mm) and `n = 12`/arm, the standard
   error of the *difference of means* is 3.67–4.65 mm, so 5 mm is only 1.36–1.07
   se out: **two blocks flown at identical aim exceed it 17–28 % of the time**.
   That would ABORT a correct experiment about once every four to six sittings.
   Now `max(5 mm, 2·se)`, and the probe prints the gap in mm, in se, and the
   resulting verdict — so runbook and instrument express the question in the same
   quantity, which they previously did not.

A third gate was biased rather than wrong: `SEAT-EXP-2.2` (the sitting's go/no-go)
asks whether the **plateau** sits in the marginal band, but the command it was
scored with printed the mean over *all* scored throws, warm-up included. That
statistic is biased toward `+x`, i.e. **toward passing**: on the reference capture
it reads `−28.8 mm` against a true plateau of `−38.9 mm`, a 10.1 mm optimism —
two thirds of the entire margin between the band edge (`−15`) and the basin edge
(`−12`). The probe now prints a `PLATEAU (last 8 scored)` line and the row names it.

### The marginal band is two-sided, and arm membership is physical

The band was written as a floor (`≥ −15 mm`). Both directions waste the sitting:
under-bias to `−14.5` gives an expected `k0 ≈ 4.6/12`, *below* the `k0 ≥ 5` the
decision table needs; over-bias to `+10` drops ~98 % in both arms and is equally
undecidable. It is now `−15 … −5 mm` with the reasoning stated.

The sharpest operator hazard found was arm membership. `--group` selects on the
probe's **attempt index**, which counts `/throw_announcements` — *every*
announcement, including ones that never produce a ball. On the reference capture
**4 of 18 announcements yielded NO TRACK**, and the sitting independently records
two Ball-Butler throws aborting. So "the 12 reloads I threw" and "attempts 9–20"
are not the same set: two refusals during CONTROL push its last two real balls
into the EXPERIMENT range, mixing the arms and tripping `SEAT-EXP-5` on a sitting
that captured 24 good throws. The protocol now requires **one bag per arm**, so
the boundary is physical rather than counted.

### Deliberately not decided here

- **How the operator biases BB aim by +25–35 mm** is left unspecified. That is
  Ball-Butler aim, and writing an unverified command into a runbook is how
  § CHECK LVL-4 and FW-1 broke. What the protocol prescribes instead is the
  measurable precondition and an explicit INCONCLUSIVE-BY-DESIGN exit.
- **`SEAT-EXP-3.4` (commanded tilt span) was demoted to REPORT.** Two reviewers
  produced contradictory offline models of the window (one via the `_republish_pretilt`
  path predicting ~0.9°, one via a single-plan rebuild predicting 2–4°), and the
  zero-seat side is the only half that is *measured* (`span_deg = 0.0000`). Gating a
  deployment check on the derived half risks failing a correct deployment; rows
  `3.1`/`3.2` are independently decisive, so this one records and does not abort.
- **The floor census stays coarse and untuned.** It over-counts a rolling ball and
  under-counts two balls resting within `--floor-radius`. Labelled REPORT
  everywhere; the operator's eye is primary.

### A number that did not reproduce, and was corrected rather than buried

The sitting's decisive marginality is quoted throughout the plan and its logbook as
*"attempt 3 dropped at `x = −10.7` and attempt 4 caught at `x = −14.5` — 3.8 mm
apart"*. Scored with a defined, deterministic estimator those two read **`−11.507`
and `−11.525`** — **0.02 mm apart**. The old pair came from an ad-hoc fit whose
window is unrecorded, and attempt 3 is the capture's sparsest track (3 samples, all
above `z = 1100`), so it is the value most sensitive to that choice.

The qualitative conclusion is unchanged and **sharper**: a drop and a catch at the
*same* measured offset is a basin edge that is **degenerate**, not merely narrow,
which is precisely why the two hypotheses cannot be separated from that capture.
This matters beyond bookkeeping — a future session reading "3.8 mm" would conclude
the basin edge was measurable and that the matched-arrival requirement is
over-engineering it can drop to save a block. The figure was therefore corrected at
its two most durable sites in this change-set (the shipped YAML comment and the
`git blame`-reachable `planner.py` comment) and in the plan's Phase-4 cell.

## Implementation

- **`config/hardware_config.yaml`** — new `trajectory_op.catch_seat_rate_radps: 0.0`.
  `trajectory_op` already had its `HW_SECTIONS` row, so no generator change was
  needed (verified by reading the table *and* by the constant appearing in all six
  generated consumers).
- **`planner.py`** — the single changed code line:
  `_CATCH_TILT_THROUGH_RATE_RADPS = float(hw.JB_TRAJ_CATCH_SEAT_RATE_RADPS)`,
  replacing the `0.0` literal. `_catch_arrival_rate` is untouched and still resolves
  the module global at call time.
- **`tools/probes/ball_arrival_offset.py`** (new) — per-throw arrival `(x, y)` at a
  horizontal plane, by least-squares fit over the descending band, cut at the first
  sub-plane sample. Reports `SPARSE` (`n < 5`) and refuses (`None`) rather than
  fabricating; `stats()` returns `sd = None` for `n = 1` rather than `0.0`, because a
  naive zero sd would turn the instrument's own data loss into a decisive result.
- **`tools/probes/catch_reach_replay.py`** — `_print_row` now reads the **live**
  planner default instead of the probe's mirror. `build_fixed` passes no rate, so it
  always rode on the live value; the printed line could assert "MANUFACTURES NOTHING"
  while the rebuild had manufactured 0.07. Unreachable while the rate was a literal —
  **made reachable by this phase's own config key**, which is why it is in-phase.
- **`ros_ws/docs/catch_arrival_contract.md`** — amendment recording that nothing in
  C-CATCH-1 changes, and that the import-time binding must not be "improved".

## Verification

Full suite: `source ~/Desktop/PDJ_venv/venv/bin/activate && python -m pytest tests/ -q`,
run **2026-07-28** on the Jetson in the project venv:
**4068 passed, 3 xfailed in 1422.44 s (0:23:42)**, exit 0. Against the `f5da09c`
baseline of 4059 passed + 3 xfailed that is **+9 passed**, accounted for exactly:
6 new tests in `tests/motion/test_ball_arrival_offset_probe.py` and 3 new in
`tests/motion/test_trajectory_planner_catch.py`. **xfail unchanged at 3** — no test
was weakened, skipped, deleted or renamed, and no `xfail`/`skip` marker was added.

Instrument acceptance, all 2026-07-28:

- `python tools/probes/ball_arrival_offset.py --self-check` → `SELF-CHECK: PASS`, exit 0
- the same probe **mutated** (descending-branch cut deleted) → `SELF-CHECK: FAIL (1 case(s))`,
  exit 1 — the two-sided check that was missing before review
- `python tools/probes/catch_reach_replay.py --self-check` → `SELF-CHECK: PASS` 10/10, exit 0
- scored against `~/Desktop/rosbags/2026-07-27_15-39-38`: worst error **0.6 mm** vs the
  three published bounce-out entry points at `--plane 1050`; plateau mean **−38.9 mm**,
  sd **9.0 mm**; burn-in slope **−7.13** vs plateau **−1.75 mm/throw**

Codegen determinism: `python config/generate_config.py` run twice, second run
byte-identical (md5-compared across the generated consumers).

Commanded motion at both rates, measured through the production planner at the
recorded reload geometry (lead 2.3712 s, receive tilt 10.87°): `0.0` → 2 segments /
2.8712 s / arrival `0.000000 rad/s` / peaks `29.0, 37.9, 170`; `0.07` → 3 segments /
3.0212 s / arrival `0.070000` / peaks `23.8, 142.0, 3935`. C-CATCH-1's bound at that
geometry is `0.200047 rad/s`, so `0.07` is returned unclipped. Against session limits
`1000/5000/30000` that is `2.4 % / 2.8 % / 13.1 %` — **jerk is the binding one**, and
leg *velocity falls* (a terminal rate lets the reach coast slower through its middle),
which the runbook states so "everything got smaller" is not a false expectation.

## Follow-ups

- **The sitting itself is deferred to the operator** — § SECTION SEAT-EXP is a
  separate, later sitting. § THE RUN SHEET is EXECUTED and is not re-run.
- **How to bias BB arrival by +25–35 mm in `x` is unanswered** and should be settled
  *before* the robot is powered; the protocol's go/no-go depends on it.
- The 3.8 mm figure still appears in `logbook/2026-07-28-anomaly-fixes-validation-sitting.md`
  (three places) as that session's honest finding; a correction pointer was added at
  its primary claim site rather than rewriting another session's record.
