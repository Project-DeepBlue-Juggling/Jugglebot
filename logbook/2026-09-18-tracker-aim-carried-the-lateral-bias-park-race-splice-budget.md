---
title: "Tracker aim carried the lateral bias into the catch site; the park raced the guard's fault task; the splice budget was smaller than the measured solve"
type: investigation
date: 2026-09-18
status: done
phase: "two-ball-skill-stack — R3"
related_plan: two-ball-skill-stack.md
files_changed:
  - plans/active/two-ball-skill-stack.md (R3 carried items: lateral clamp / guard-wait park / splice budget resolved; one new carried item — re-seed the streamed hand lane for an armed-wire park)
  - ros_ws/docs/choreography.md (regenerated for the new `/park_hand` service)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md (new C-CATCH-2 row: the lateral-clamp contract)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (`_clamp_lateral_to_schedule`; `_catch_terminal` clamps before dispatch and both resend paths; the resend "moved" test measures the clamped landing against the committed terminal)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py (`SOLVE_BUDGET_KNOTS = 6`; `LEAD_KNOTS`/`HANDOFF_LEAD_KNOTS` derived from it)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py (`_park_hand` calls `/park_hand` unconditionally before the pre-level and before `t0` is read)
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py (`_wait_for_guard_clear`/`_GUARD_CLEAR_WAIT_S`; `_park_hand(*, lane_cleared=...)` refuses an off-band hand on an armed wire instead of parking it; `/link_status` surfaces `sched_refused`/`sched_stops`)
  - tests/hardware/session_skills_r3.md (§ 7 recovery sequence + row 4 budgets)
  - tests/motion/test_skills_executor.py (new lateral-clamp section; lead-number literals updated to the derived budget)
  - tests/motion/test_skills_schedule.py (contract test: every lead, pinned or not, leaves ≥ 0.150 s of solve budget)
  - tests/ros/test_skill_node.py (park-before-pre-level / park-before-columns-t0 tests)
  - tests/ros/test_skills_plan_bench.py (budget literal update)
  - tests/ros/test_teensy_bridge_node_read.py (`/link_status` surfaces the new counters)
  - tests/ros/test_teensy_bridge_node_recover.py (guard-wait-then-park, latch-never-clears, `/park_hand` band/armed-wire tests)
  - tests/sim/test_skills_gate.py (stale lead literal in a docstring)
subsystem:
  - motion
  - ros
tags:
  - safety
  - performance
  - testing
---

## Symptom

The 2026-09-18 13:25 sitting (`temp/logs/launch_r2gate_20260918_1325.log`, bag `2026-09-18_13-25-15`):
23 schedules, 41 outcome rows, 39 caught, the learner converged cleanly at both apexes (0.6 m:
command ≈0.53 → observed 0.60–0.67 m; 0.9 m: 0.80 → 0.84–0.95 m), and contact phase sat in the
smooth `seat=` +0.10..+0.15 s regime on nearly every catch — the best sitting yet by the operator's
own read ("overall very happy with this change over the last run"). Against that: two attempts (0.6
and 0.9) had the platform move in +y and drop the ball; after a few cycles at 0.9 m the Teensy guard
FAULT LATCHed, once with the hand near the top of its stroke, and `clear_errors → ACTIVATE →
TRAJECTORY → start_self_toss` met the latch again; and a few cycles ended prematurely on
`REJECTED_CYCLE_INFEASIBLE` or a solve that ran long.

## Diagnosis

### 1 — the tracker's converged aim carries the ball's own lateral bias into the catch site

Observed lateral landing `y` ran +0.05..+0.12 m on every row, growing with apex — a constant
lateral launch-velocity error of ≈0.1 m/s, not noise. `AIM_TRACKER` (landed 2026-09-18, see
`2026-09-18-learn-the-apex-aim-from-the-tracker.md`) puts a converged `from_fit` tracker landing
ahead of the schedule prior with no step waiting on it, so every re-aim moved the catch site toward
that bias. Two re-sends were ACCEPTED and moved the site 84.4 mm and 95.1 mm in +y
("RESEND skill 2: the fit moved the landing 84.4 mm / +0.080 s (re-aim 2 of 2)") — the platform
followed inside the dive and dropped the ball, once per apex. 52 more re-send solves were refused
`REJECTED_CYCLE_INFEASIBLE` (`LIMIT_VEL 20 / LIMIT_ACC 6 / LIMIT_JERK 8`, the banking-saturation
class of `2026-09-16-banking-saturates-on-small-lateral-offsets.md`), each costing 25–130 ms of
solve time it did not have to spend. Meanwhile the schedule-aimed catches at (−50, 0) caught balls
landing 50–75 mm off with no platform move at all: the cup tolerates a lateral miss the platform
does not tolerate chasing.

### 2 — the park raced the guard's own fault-clear mechanism

`CLEAR_ERRORS` is acked by the LINK; the latch is released by the firmware's 10 Hz fault task on its
*next tick*. Log lines 977–982: `CLEAR_ERRORS fired` → `hand park: ACTIVATE(axis 6) … from
+9.6227 rev` → `ACTIVATE rejected: ERR_BUS_DOWN … fault_state=MAX_DEVIATION is currently latched` →
`HAND NOT PARKED`, and the *next* line is `Teensy guard fault cleared (fault_state=NONE)` — lost by
under one fault tick. That refusal escalated the armed `/clear_errors` fallback
(`recover-failed → disarm + direct clear`), which is why the operator had to ACTIVATE/TRAJECTORY by
hand, and it disarmed the wire the next schedule needed. From there: the opening REST streamed the
hand home at ~5.4 rev/s onto a firmware-held stroke (latch 1, no motion at all — see Discussion b),
then the plain recovery slew (`RECOVER_SLEW_VEL_RPS = 1.0 rev/s`) tripped MAX_DEVIATION twice more
on the way back (latches 2–3, measured `vel_ff` = −1.00, tripped at −2.68 / −3.13 rev after
0.6–0.7 s).

### 3 — the splice budget was sized below the measured solve tail

CATCH `install_segment` plan times (n=51) against the old budget:

| stat | min | p50 | p90 | p95 | max |
|---|---|---|---|---|---|
| solve time (ms) | 40.1 | 76.4 | 111.0 | 113.4 | 134.2 |

17 `SPLICE_TOO_LATE` lines / 18 refusals read "solve took 0.093–0.125 s … budget
0.083–0.100 s" — the refused catches were on the unpinned lead (`LEAD_KNOTS`), not the handoff one.
The old `LEAD_KNOTS = 6` gave a floor of 0.150 s only at the *pinned* handoff lead; the unpinned
lead's floor sat at 0.083–0.100 s, well under p90.

## Discussion

**(a) Why clamp the tracker's lateral aim rather than trust it.** The 2026-09-18 apex fix put the
tracker's converged fit ahead of the schedule prior specifically because the schedule's flight time
is no longer learnable and mocap coverage is a real gap the prior has to fill — that reasoning
covers *when* to trust a converged fit, not *how far* it should be allowed to move the platform once
trusted. Diagnosis #1 shows the two are separable: the fit's timing and height corrections were
converging the learner correctly on the same rows where its lateral component was dragging the
platform into an infeasible or ball-dropping move. `_clamp_lateral_to_schedule` bounds only the
tracker's *lateral* contribution to the catch site (to `lateral_authority_m` around the schedule's
own commanded landing) while leaving `z`, velocity, timing and `from_fit` untouched — the same
authority knob that already bounds the learner's lateral throw command (`lateral_authority_m` was
pinned to 0 for the throw side on 2026-09-16) now bounds the catch side too, one parameter instead
of a second one. The resend "moved" test is updated to compare the *clamped* landing against the
committed terminal, not the raw fit, so a fit that only asks for a further lateral move the catch
may not take reads `WITHIN-TOLERANCE` rather than spending a re-solve on a move that would be
clamped away anyway.

**(b) Why the park waits on the fault task, and why an armed-wire schedule now REFUSES rather than
parks (the non-obvious tradeoff).** The straightforward fix for #2 — wait for the fault task before
ACTIVATE — surfaced a second hazard while landing it: `_park_hand` moves the axis while a streamed
lane may still be commanding it. After a hand-less hold the firmware lane HOLDS its last knot and
keeps commanding it; the guard measures that raw held command against the encoder, so parking the
axis out from under a *live* lane reopens the same MAX_DEVIATION gap with the encoder moving instead
of the plan. Only an `s_output_enabled` false→true edge clears the lane — exactly what a latch+clear
recovery gives, and exactly what a schedule start does *not* give. The straight fix ("wait, then
park") would have been correct for recovery and silently wrong for a fresh schedule on an armed
wire. The tradeoff accepted: `_park_hand(*, lane_cleared=…)` — recovery passes `True` (the lane was
just cleared) and gets a real park; `/park_hand` passes `False`, and an off-band hand found on an
ARMED wire is REFUSED, naming DEACTIVATE→ACTIVATE, rather than parked. An already-parked hand stays
a free no-op on an armed wire (every healthy start). This trades one class of pre-motion refusal
(operator has to cycle DEACTIVATE/ACTIVATE) for the alternative of parking into a live lane and
re-tripping the guard mid-schedule — a worse failure because it happens after the schedule is
already running. **Carried, not built here**: re-seeding the streamed hand lane itself (a
hand-bearing hold, or an interp verb pinning the lane to the encoder) would let a schedule park on
an armed wire without this refusal.

**(c) Why the budget is sized from p95 + one knot, and what it costs.** `SOLVE_BUDGET_KNOTS = 6`
(0.150 s) is p95 (113.4 ms) plus one knot of margin, clearing the measured max (134.2 ms) by 16 ms —
a number read off this sitting's own distribution, not a round figure. Both leads now DERIVE from
it (`LEAD_KNOTS = WIRE_READ_KNOTS + SOLVE_BUDGET_KNOTS = 9` / 0.225 s; `HANDOFF_LEAD_KNOTS =
LEAD_KNOTS + HANDOFF_LEAD_EXTRA_KNOTS = 11` / 0.275 s) rather than each being bumped independently,
so `compile_columns`'s dispatch-monotonicity check keeps exactly the gap it had and the pinned lead
can never end up below the unpinned one. Cost, stated in the docstring: every dispatch splices
75 ms further ahead, `executor.CATCH_FREEZE_S` grows to 0.250 s, and the re-aim window shrinks by
75 ms — directly narrowing the window Diagnosis #1's clamp is meant to operate inside. The two
fixes pull in opposite directions on purpose: the clamp bounds how far a late-arriving fit can move
the site, the wider budget buys the solve more time to *use* that bounded correction before the
freeze closes.

**Reframing two of yesterday's fixes.** The AIM_TRACKER default (2026-09-18) was correct that a
converged fit should out-rank the schedule prior; it did not yet say how far that fit's *lateral*
component should be trusted to move a committed platform pose, which #1 above answers. The
opening-REST-homes-the-hand fix (2026-09-16) was correct that the REST should reconcile the hand
from the encoder; it did not account for a hand held by the firmware at the top of its stroke by a
FW 22 latched scheduled hold, which is a state the REST can walk into without ever going through
`_park_hand` — hence B2's unconditional `/park_hand` call ahead of both the pre-level and `t0`.

## Fix

1. **`executor.py`** — `_clamp_lateral_to_schedule(idx, skill, landing)` clips
   `landing.pos_mm[:2]` to `lateral_authority_m` around the schedule's own predicted landing
   (`None` authority or no prior = unclamped); `_catch_terminal` calls it before a `Landing`
   becomes a `CatchTerminal`, so dispatch, `_resend_hand_corrected_catch` and `_resend_live_catch`
   all pass through it; the resend "moved" test compares the clamped landing to the committed
   terminal; one deduped `AIM-LATERAL-CLAMPED skill N: tracker landing +D mm in y/x, catch keeps
   the schedule's site (authority A mm)` log line per catch.
2. **`INVARIANTS.md`** — new `C-CATCH-2` row in § 7 naming the lateral-clamp contract, its
   enforcement point, and the four tests below.
3. **`teensy_bridge_node.py`** — `_wait_for_guard_clear` polls the same cached
   `HeartbeatT2J.fault_state` the `Teensy guard fault cleared` log line is derived from
   (`_GUARD_CLEAR_WAIT_S = 1.0 s`, 10 heartbeats against a ~100 ms mechanism), called from inside
   `_park_hand` so both existing recovery paths and the new `/park_hand` service get it for free;
   no heartbeat or an expired wait both refuse, naming the latch. `_park_hand(*, lane_cleared=…)`
   refuses an off-band hand on an armed, uncleared wire instead of parking it (§ Discussion b).
   `/link_status` now surfaces `sched_refused`/`sched_stops` from the wire.
4. **`skill_node.py`** — `_park_hand` calls the bridge's `/park_hand` unconditionally before the
   pre-level in `_svc_start_self_toss` and before `t0` is read in `_svc_start_columns`; no local
   copy of the park band, an already-parked hand is a free no-op.
5. **`schedule.py`** — `SOLVE_BUDGET_KNOTS = 6` is the one constant (p95 + one knot);
   `LEAD_KNOTS`/`HANDOFF_LEAD_KNOTS` derive from it instead of being set independently.
6. **`ros_ws/docs/choreography.md`** — regenerated for the new `/park_hand` service
   (`tools/gen_choreography_map.py`).
7. **Tests** — new lateral-clamp section in `test_skills_executor.py`; lead-number literals
   updated across `test_skills_executor.py`, `test_skills_schedule.py`,
   `test_skills_plan_bench.py`, `test_skills_gate.py`; guard-wait / `/park_hand` /
   armed-wire-refusal tests in `test_teensy_bridge_node_recover.py`; `/link_status` counter test
   in `test_teensy_bridge_node_read.py`; park-before-pre-level / park-before-columns tests in
   `test_skill_node.py`; `session_skills_r3.md` § 7 recovery sequence and row 4 budgets updated to
   match.

## Verification

| what | date, command | result |
|---|---|---|
| lateral clamp (executor) | 2026-09-18, `pytest tests/motion/test_skills_executor.py -q -p no:cacheprovider` | **111 passed in 3.24 s** |
| guard-wait + `/park_hand` (recover) | 2026-09-18, `pytest tests/ros/test_teensy_bridge_node_recover.py -q -p no:randomly` | **30/30 pass** (was 20; +10 new) |
| skill_node park ordering | 2026-09-18, `pytest tests/ros/test_skill_node.py -q -p no:randomly` | **68/68 pass** (+5 new) |
| `/link_status` counters | 2026-09-18, `pytest tests/ros/test_teensy_bridge_node_read.py -q -p no:randomly` | **64/64 pass** (+1 new) |
| splice budget contract | 2026-09-18, `pytest tests/motion/test_skills_schedule.py tests/ros/test_skills_plan_bench.py -q -p no:randomly` | **118/118 pass** (+1 new) |
| broad ros+motion+sim+hardware sweep | 2026-09-18, `pytest tests/ros tests/motion/test_skills_schedule.py tests/sim/test_skills_gate.py tests/hardware -q -p no:randomly` | **3001 passed, 2 failed, 17 errors** — the 2 failures were `choreography.md` stale from `/park_hand` (regenerated, then 30/30 on `test_choreography_map.py`); the 17 errors are the real-robot `tests/hardware/*_test.py` harnesses, which only collect when named explicitly (pre-existing, not from this change) |
| executor lead-literal handoff | 2026-09-18, `pytest tests/motion/test_skills_executor.py -q -p no:randomly` | 3 pre-existing failures fixed test-side (splice-knot literal 22→25, handoff budget literal 0.125→0.200 s, catch-freeze probe re-derived from `LEAD_S`) |

(2026-09-18, `./run_tests.sh --full`, log `temp/logs/sitting0918_fixes_full_20260918.log`, the committed tree): **PASS — parallel 6289 passed, 9 skipped, 2 xfailed in 317.14 s; serial 6 passed in 19.41 s.**
(2026-09-18, `./run_tests.sh`, log `temp/logs/sitting0918_fixes_gate2_20260918.log`, after the phase audit's one logging fix — the hand-corrected re-aim's CATCH-AIM line now reports the installed, clamped terminal): **PASS — parallel 6251 passed, 9 skipped, 1 xfailed in 254.71 s; serial 3 passed in 7.71 s.**

## Carried

* **Re-seed the streamed hand lane onto the park** (§ Discussion b) — a hand-bearing hold, or an
  interp verb pinning the lane to the encoder, would let a schedule park itself on an armed wire
  instead of refusing.
* **The lateral trim** — the ≈0.1 m/s lateral launch-velocity error the clamp hides rather than
  removes (Diagnosis #1); carried from `2026-09-18-learn-the-apex-aim-from-the-tracker.md`'s
  `+55 mm lateral trim` item, now with a fresh per-sitting measurement.
* **Cup ≤ g downward-acceleration contract** — deferred again; the contact-phase numbers this
  sitting sat in the smooth `seat=` +0.10..+0.15 s regime, so the >g dive that motivated the
  contract was not the active failure mode this sitting.
* **Mocap coverage** (25–55 %, typically 40 %) remains the live risk under `AIM_TRACKER` — watch
  the `RESEND-SKIPPED NO-CONVERGED-FIT` rate at the next sitting.
* **`resend_max_per_catch`-style budget for `REJECTED_CYCLE_INFEASIBLE` resends** — 52 refused
  re-send solves this sitting cost 25–130 ms each with nothing to show for it once the lateral
  clamp is in place; worth capping independent of the tolerance/authority fences already in
  `_resend_live_catch`.
