---
title: "Toss Phase 3 validated: the dry trace's own self-announcement seeds a predicted track — checker contract corrected"
type: bugfix
date: 2026-07-25
status: resolved
phase: "single-ball-toss Phase 3 (real-ordering trace)"
related_plan: single-ball-toss.md
subsystem: tools
tags: [testing, tracking]
commits:
  - b927ef1
files_changed:
  - tests/hardware/toss_trace_recorder.py
  - tests/hardware/session_phase8_toss_trace.md
  - plans/active/single-ball-toss.md
---

# Toss Phase 3 validated: self-announcement seeds a predicted track — checker contract corrected

## Summary

The Phase-3 real-ordering dry trace was executed on hardware (three captures
across two ACTIVATE cycles). Every cross-process **ordering** invariant passed,
validating the toss choreography. But the `--dry` checker also reported five
FAILs, none of them ordering bugs: two were artifacts of a ball being in the cup
during the first capture, and three were **mis-specified invariants** —
`toss_trace_recorder.py` assumed a no-ball toss leaves the tracker quiescent,
but a *self-announced* toss always seeds its own predicted-ball track. The three
invariants (DT-7, DT-13, DT-14) were corrected as a contract fix — distinguishing
the intrinsic predicted track from a real detection via the `tracking` field, not
weakening the guard. Corrected `check --dry` on the clean no-ball trace =
**12 PASS / 0 FAIL**; the correction was adversarially verified (it still fails on
the real-ball trace). Phase 3 is VALIDATED.

## Problem

Capture D (waived dry choreography) failed 5 of the DT invariants across two
runs. Run 1 (`toss_trace_2026-07-25_15-06-38`, a ball inadvertently in the cup):
DT-7, DT-9, DT-12, DT-13, DT-14 FAIL. Run 2 (`…_15-24-25`, hand empty): DT-7,
DT-13, DT-14 FAIL (DT-9, DT-12 now PASS). The runbook's gate rule is "any FAIL =
Phase 3 not met," so on its face the trace did not pass — despite the choreography
ordering (DT-1/2/5/6/8/10/11) being green in both runs.

## Root Cause

Two independent causes, separated by the ball/no-ball A/B:

1. **Ball-in-hand artifacts (Run 1 only): DT-9, DT-12.** With a ball present, the
   FSM's `THROWING → BALL_IN_FLIGHT` transition (`toss_sequencer.py:824`,
   `if obs.throw_stroke_seen or obs.ball_track_confirmed`) fired early on the
   `ball_track_confirmed` branch — the tracker confirmed the in-hand ball ~1.6 s
   before the physical stroke — inverting DT-9's "stroke precedes BALL_IN_FLIGHT."
   DT-12 saw a real flight time in the outcome string. Both flipped to PASS with
   the hand empty (Run 2), proving they were premise violations, not bugs.

2. **Mis-specified invariants (both runs): DT-7, DT-13, DT-14.** A self-announced
   toss publishes a `ThrowAnnouncement` (`thrower_name='jugglebot'`,
   `target_id='jugglebot'`), and `ball_prediction_node` synthesises a *predicted*
   `BallState` from it — `destination=jugglebot`, `tracking=0`, no mocap detection,
   walking `TO_BE_THROWN → IN_FLIGHT → UNKNOWN` on the announced schedule. That
   predicted track is the target the catch pipeline pre-positions against, so it
   necessarily produces `balls` rows (DT-14), a `dynamic_target` stream (DT-7,
   which wrongly demanded "exactly 1"), and an active (non-`hold`) plan_kind
   through the flight (DT-13). None of these can ever be absent for a toss trace,
   ball or not — the invariants encoded a false "no ball ⇒ quiet tracker"
   assumption.

Evidence the phantom is intrinsic and not a stray object: in Run 2 the track was a
single id=3, `tracking=0` throughout (never a real detection), born 14 ms after
the announcement.

## Discussion

The instructive reframe was DT-9. On first read it looked like a *real* ordering
finding — the FSM declaring the ball airborne 1.6 s before the hand moved is
exactly the Phase-7 bug class (a phase anchored to the wrong event). The
hypothesis "DT-9 is a genuine cross-process timing bug, not a ball artifact" was
withdrawn only after reading `toss_sequencer.py:824`: the transition is
`throw_stroke_seen OR ball_track_confirmed`, and with a real ball the `OR` branch
fires early. The no-ball re-run was the decisive test — DT-9 flipped to PASS
(stroke → BALL_IN_FLIGHT +17 ms). This is the CLAUDE.md discipline in miniature:
the confidence of the "it's a real bug" reading was not evidence for it; one
contradicting data point (the clean re-run) retired it.

On the fix: the temptation was to relax DT-14 to "balls rows allowed." That would
have destroyed a real guard — a genuine stray ball/marker in the volume *would*
corrupt a dry trace and must still fail. The `tracking` field is the principled
discriminator: the intrinsic predicted track is `tracking=0` (predicted-only); a
physical object yields `tracking=1` (mocap detection). The corrected DT-14 fails
only on `tracking=1` or a foreign destination, preserving the guard's purpose
while admitting the intrinsic track. DT-7 was likewise *strengthened*, not
loosened: instead of a meaningless "exactly 1" count it now asserts the whole
pre-position stream stays within 30 mm of the nominated (0,0,170) — a wandering
target means a real ball pulled the reach, which the count could never catch.
DT-13 kept its load-bearing checks (streaming=True + mode=TRAJECTORY throughout)
and dropped only the mis-specified "return to hold within the window," which the
catch pre-position legitimately prevents.

The root reason the checker was wrong is that the Phase-3 harness was authored by
an unattended run whose charter forbade hardware — it could never observe that a
self-announcement seeds a predicted track. The invariants were a reasonable guess
that only real traces could correct. This is the expected shape of a
prep-then-validate phase, not a defect in the build.

## Fix

`tests/hardware/toss_trace_recorder.py`:
- **DT-14** — pollution guard re-keyed to `tracking`/destination: fail on any
  `tracking!=0` or `destination!=jugglebot` entry (a real stray object); pass the
  intrinsic predicted track. New `ROBOT_NAME` constant.
- **DT-7** — drop the "exactly 1 dynamic_target" count; require ≥1 and assert
  every pre-position target within `TARGET_POS_TOL_MM` of (0,0,170); keep the
  pre-tilt-log + accepted-catch-feedback checks.
- **DT-13** — drop "plan_kind returns to 'hold'"; keep streaming/mode as the hard
  gate, report plan_kind for the record.

`tests/hardware/session_phase8_toss_trace.md` — the "No ball anywhere" note and
the DT-7/DT-13/DT-14 table rows updated to match.

## Verification

- Corrected `check --dry` on `temp/logs/toss_trace_2026-07-25_15-24-25.jsonl`
  (the clean no-ball trace): **12 PASS, 0 FAIL, 2 AMBIGUOUS** (the tolerated
  in-bundle DT-3/DT-4 same-tick orderings). Exit 0.
- Adversarial: corrected `check --dry` on the real-ball trace
  `…_15-06-38.jsonl` still **FAILs** DT-14 (`tracking=1 id=35 destination=''`)
  and DT-9 — the guard was corrected, not gutted.
- Capture R (`…_15-05-03`) `check --reject`: 4/4 PASS (unchanged; not touched).
- Full suite: `pytest tests/ -q`, run 2026-07-25 — **3397 passed, 3 xfailed in
  1332.51 s**. (The trace harness is not pytest-collected; the suite guards
  against incidental breakage.)

## Open Questions

- **Two real tosses occurred during the arc and both MISSED** (flight ~1.09 s).
  The choreography works end-to-end on hardware; the miss is throw/catch tuning —
  the substance of Phase 5. These stand as early T0/T1 data points.
- **Minor robustness note (not a blocker):** `ball_track_confirmed` advanced the
  FSM to BALL_IN_FLIGHT early on an already-tracked ball. In normal single-ball
  operation the only tracked ball is the thrown one, so this is benign — but a
  stray/pre-existing tracked ball can advance the phase. Worth a look if Phase 5
  ever runs with residual tracked objects.
