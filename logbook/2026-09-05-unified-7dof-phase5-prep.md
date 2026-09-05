---
title: "Unified 7-DoF Phase 5 prep — the build-time key flips true, a thin bench driver for the two sub-cycle rungs, a plain-language ladder runbook, and one defect the carry probe found before the ball did"
type: feature
date: 2026-09-05
status: resolved
phase: "unified-7dof-planner — Phase 5 (prep, NOT FLOWN)"
related_plan: unified-7dof-planner.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/hardware_config.h
  - ros_ws/src/jugglebot/jugglebot/hardware_config.py
  - ros_ws/src/jugglebot/Teensy_code_canbridge/hardware_config.h
  - ros_ws/src/jugglebot/Teensy_code_platform/hardware_config.h
  - ros_ws/src/jugglebot/CatchingCone_code/hardware_config.h
  - tests/hardware/unified_cycle_bench.py
  - tests/hardware/session_unified7_cycle_ladder.md
  - tests/ros/test_unified_cycle_bench.py
  - logbook/2026-09-05-unified-7dof-phase5-prep.md
  - logbook/INDEX.md
  - plans/active/unified-7dof-planner.md
subsystem:
  - ros
  - motion
  - config
tags:
  - safety
  - testing
  - docs
---

# Phase 5 prep — the software the hardware ladder needs

## Summary

Phase 4 left a unified cycle that has never moved a robot. This lands what the
Phase 5 sitting needs and nothing else: the build-time opt-in flipped, a thin
ROS-client bench driver for the two rungs the shipped session cannot express
(UH-3 carry, UH-5 throw), an operator runbook in plain language, and — surfaced
by the probe that was supposed to just pick a default — a fix to a boundary
condition that would have made UH-3's own pass criterion unreachable.

Nothing here has been flown. **The arming decision is the owner's, 2026-09-05:
Phase 5 rungs fly with the hand deviation guard ARMED from UH-3 on** — `hand7
arm` joins the session-start checklist and a trip is data, not a failure.

## Discussion

### The carry probe found a fabricated free-fall — logged separately

`--rung carry` was specified as `PlanCycle NEW SETTLE` from rest to a laterally
displaced settle site. Probing it before writing the driver (the probe-first
rule) showed the plan is not a carry at all: `_cycle_start_state` supplied no
`cup_accel_mm_s2` for a post-release kind, `CycleState.to_cup_state` took its
documented **`g`** fallback, and the QP was handed free fall as a hard boundary
condition — knot 0 cup `a_z` **−9.806 m/s²** (apparent gravity in the cup
exactly zero, the seated ball weightless), knot 1 **+19.612**, a **75.42 mm**
cup-z arc and **2.3845 rev** of slider on a purely lateral move. `validate_cycle`
accepts both shapes, so nothing downstream refuses the falling one, and
T-H5/UH-3's entire pass criterion is *"no visible ball disturbance"*.

**That defect and its fix are recorded in
[`2026-09-05-plan-cycle-settle-from-rest-free-fall.md`](2026-09-05-plan-cycle-settle-from-rest-free-fall.md),
which is its canonical record** — the audit of this prep work found two more
defects on the same path (an `at_rest` predicate whose 841 mm/s platform bound
could never fire, and detach-cone rows assembled for a window that follows no
release) plus the structural reason none of them was caught, and all four
changes ship as their own commit. It is named here because the probe that found
it was this prep's, and because it is the reason UH-3 can be scored on the
criterion it was written with.

**Blast radius on shipped paths: zero.** The shipped session calls `MODE_NEW`
only with `KIND_LAUNCH` (post-release false, already rest-checked); `MODE_EXTEND`
seeds from `release_state_from_meta`, which supplies the exact cup acceleration;
`MODE_REPLAN` re-solves an installed tail. The only caller reaching the changed
branch today is the new bench driver.

### The other probe finding: a verdict on the command could never fail

The driver's stroke-headroom check was first written as commanded peak plus one
full `MAX_LEAD_HAND_REV`. Probing the nominal UH-5 throw (0.5 m apex, session
limits) gave a commanded peak of **9.643 rev** — and 9.643 + 2.0 = 11.643 fails
against the 10.8 hard stop, i.e. the check would have failed the nominal rung.

The composition was wrong, not the number. The **command** cannot reach metal:
`validate_cycle` refuses outside `[0, JB_OP_HAND_CATCH_PRIME_REV]` (9.9594) with
`HAND_STROKE`, and the firmware clips every setpoint to `[0, 10.8]` after that,
so a verdict on `hand_peak_rev` restates two gates and cannot fail. What *can*
reach metal is the slider overshooting its command, which sitting two measured
directly: encoder **10.4693 rev** against the 10.8 stop, 0.33 rev of margin, on
a legacy stroke. V3 is now scored on the encoder, with the commanded margin
reported beside it and the bar a `--metal-margin` knob (0.20 rev default),
because the tier ramp is what moves it.

### Why the config key flips despite the per-goal field defaulting false

`unified_cycle_enabled` is a **reviewed config commit** by design — which build
ran the unified planner has to be answerable from git alone when a bench trace
is read back months later. Every prerequisite the ships-false comment named is
now aboard: the v6 wire (Phase 2), FW 17's hand lane flashed with nine ladder
rows PASS (Phase 3, 2026-09-04), and the Jetson-side cycle sim-gated (Phase 4).

Flipping it does **not** put the hand on the stream. The path is opt-in twice,
and the second key — `unified_cycle` on `TossContinuous` — still defaults false
and is re-applied by the node, so every legacy goal still runs the legacy
firmware stroke engine.

**There is no refusal path, and that is the thing worth saying out loud.** The
two keys are ANDed fail-closed into one bool (`_unified_enabled`,
`reload_coordinator_node.py:6732`, one read per goal), so while the build key is
false a goal that *asks* for the unified path is not rejected — it silently runs
on the legacy one, with nothing downstream reporting which planner won. That is
what the flip buys: not the removal of a refusal, but the removal of a silent
substitution. The YAML comment was rewritten to say exactly that, because a
reader finding `true` with no explanation would reasonably conclude the hand was
already streamed, and a reader finding `false` would reasonably expect an error.

## Implementation

**A — the flip.** `jugglebot_operational.unified_cycle_enabled: false → true`,
regenerated with `python config/generate_config.py`. Six artifacts moved, all
one line (`JB_OP_UNIFIED_CYCLE_ENABLED` / `UNIFIED_CYCLE_ENABLED`); **none under
`sim/` or `controller/`**. No test pins the shipped value — the flag is
monkeypatched everywhere it is read — so nothing needed weakening.

**B — `tests/hardware/unified_cycle_bench.py`** (1262 L, with 761 L of offline tests in `tests/ros/test_unified_cycle_bench.py`). A request-only rclpy
client on `trajectory/plan_cycle`, modelled on `tilt_cal_grid.py` /
`traj_ramp_battery.py`: pure core (requests, preconditions, verdict, ballistics)
with `rclpy` imported inside `run()` only, so `--dry-run` rehearses on a box with
a stale colcon build. It never opens the UDP link, never arms, never sets limits,
never switches `hand_source`; its only safety behaviour is a refusal. Five
preconditions, each carrying the command that fixes it: status fresh +
TRAJECTORY + streaming, `/robot_state` fresh, hand axis 6 CLOSED_LOOP,
`/link_status` `hand_source == STREAMED`, and the catch-capable limits
250/3000/150000. Per-tick CSV + `_meta.json` under `temp/logs/`, PASS/FAIL/SKIP
block, exit 0 iff no FAIL.

**C — `tests/hardware/session_unified7_cycle_ladder.md`** (193 L). The Phase 5
runbook, written for the operator in plain language on the owner's explicit
instruction: numbered steps, each with what to do, the exact command, and what to
expect. UH-4 is folded into UH-6 (the session is the only planned-catch path);
UH-7 is marked not-this-sitting. The close-out recovery is the operator's own
measured recipe from sitting three: deactivate/launch down → idle the hand in the
ODrive GUI → push it by hand into the retract band → `--source-only legacy`.

**D — the boundary-condition fix is NOT in this change set.** It ships as its
own commit against its own entry,
[`2026-09-05-plan-cycle-settle-from-rest-free-fall.md`](2026-09-05-plan-cycle-settle-from-rest-free-fall.md)
(`trajectory_node.py`, `motion/unified_cycle.py` and their tests).

## Verification

- (2026-09-05) `python -m pytest tests/ros/test_unified_cycle_bench.py -q` —
  **55 passed in 0.59 s**.
- (2026-09-05) `python -m pytest tests/ros/test_unified_cycle_bench.py
  tests/ros/test_unified_cycle_integration.py tests/sim/test_logbook_front_matter.py
  tests/sim/test_logbook_search.py tests/firmware/test_config_drift.py -q` —
  **192 passed in 15.17 s**.
- Full gate (2026-09-05, `./run_tests.sh --full`, parallel **6747 passed / 4 skipped / 2 xfailed in 450.02 s**, serial **4 passed in 26.08 s**, total 482 s, exit 0) — **GREEN**, run on the tree carrying all three commits of this change set (the sitting-three write-up, the settle-from-rest fix, and this prep).
- (2026-09-05) `/usr/bin/python3 -m py_compile
  tests/hardware/unified_cycle_bench.py` — clean under the system 3.8 the tool
  actually runs on.
- (2026-09-05) `/usr/bin/python3 tests/hardware/unified_cycle_bench.py --rung
  carry --dry-run` and `--rung throw --dry-run` — both print a complete request
  and exit 0 with no ROS calls.
- Probes, all 2026-09-05 under the venv, offline against the production planner:
  the free-fall carry table above; the throw rung planning at flight 0.50 /
  0.64 / 0.80 s with chain periods 0.8 / 1.0 / 1.4 s (joined duration 2.0 s,
  wall 442 ms, hand peak 9.643 rev at 99.2 rev/s, release vz **3137.9 mm/s**,
  which is `g·T/2` at that rung's own 0.64 s flight to **0.00 %** — the 3131.5
  mm/s the driver's V7 check quotes is `g·T/2` at the EXACT 0.5 m apex flight,
  0.6387 s, and the 6.4 mm/s between them is the rounded flight time, not error).

## Outcome

Phase 5 is prepped, not flown. The sitting has a driver, a runbook, an armed
guard decision and a config commit that says from git which build ran. This
change set carries no production code change at all — the free-fall fix the
carry probe surfaced is a separate commit against
[`2026-09-05-plan-cycle-settle-from-rest-free-fall.md`](2026-09-05-plan-cycle-settle-from-rest-free-fall.md).

## Open Questions

1. **`_UNIFIED_THROW_CUP_Z_MM` / `_UNIFIED_CATCH_CUP_Z_MM` still promote to YAML
   after this ladder tunes them** (Phase 4 open question 6). The driver exposes
   `--throw-z` so UH-5 can move it without an edit.
2. **The settle rest is still outside the firmware's `hand_source` band.** The
   operator's sitting-three close-out confirmed it the hard way at +1.06 rev.
   Whether to inset the cup box or move the park height remains open; until it
   is settled, the runbook's four-step recovery is the answer.
3. **UH-7 needs a `release_at_perf` hand-off** that does not exist. Marked
   not-this-sitting rather than attempted.
