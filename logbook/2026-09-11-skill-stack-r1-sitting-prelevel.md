---
title: Skill-stack R1 sitting — one hand master flies; the launch's tilt snap is a levelling-frame seam
type: investigation
date: 2026-09-11
status: resolved
phase: "two-ball-skill-stack — R1 sitting"
related_plan: two-ball-skill-stack.md
---

# R1 sitting: one caught self-toss, and the tilt snap behind two aborts

## Outcome

R1's gate passed: **one streamed self-toss, thrown and caught, with no hand-source
latch step anywhere** (the latch is deleted). The catch was slightly late — a
tuning detail, not scored here. The pair had been flashed (bridge FW 21 / proto
7, Platform FW 7) by the operator; this entry is the first powered sitting.

Getting there surfaced one real defect and one R2-scoped limit:

- **The launch's banking plan snaps the platform tilt at knot 0** when the
  machine rests level-to-base while a gravity-levelling correction is loaded.
  Measured 5× leg-jerk inflation. It refused the session-start floor lift
  outright and inflated the launch to the ceiling. **Fixed this session** by
  pre-levelling the platform before the session's first lift.
- **`num_throws=5` aborts `ABORTED_NO_RELEASE`** — the chained multi-throw solve
  is not warm-started and pays the ~2 s cold cost, overrunning the 1.8 s launch
  lead. This is the UH-7a chaining problem; **R2**, not R1.

## Discussion

### What the operator saw

Two aborts before the throw flew:

1. `HAND_BELOW_FLOOR` — "hand below zero." The hand rested ~10 mm under the
   planner floor, so the session tried its 1.0 s floor lift first, and the lift
   was refused `LIMIT_JERK` at **152 455 mm/s³**. A refused lift never raises the
   hand, so every launch then refused `HAND_BELOW_FLOOR`.
2. After the operator set the hand to 0.3 rev by hand, `LIMIT_JERK` again — the
   launch itself, at ~65 k mm/s³ against the **30 k default cap**, because the
   `set_limits` step (jerk → 150 k) had been skipped.

Then, with limits raised and the hand above the floor, a single throw flew and
was caught. A follow-up `num_throws=5` aborted with no throws.

### Root cause of the jerk refusals — one seam, not two bugs

Reproduced offline from the sitting's own bag (`~/Desktop/rosbags/2026-09-11_
19-36-52`, gravity offset `[0.0133, 0.0009] rad` ≈ 0.76°):

| Window | Seed = level-to-base (what happened) | Seed = gravity-level |
|---|---|---|
| Floor lift (`KIND_SETTLE`) | refused, **152 k** mm/s³ | ~12 |
| Launch | **72 k** mm/s³ | 14.5 k |

The unified cycle's banking schedule is built **gravity-referenced** and
re-expressed into the plan frame with the levelling correction (C-LEVEL-1 row
E8), so a healthy cycle rests the cup at the gravity-level counter-tilt
(−0.76° in plan terms). But knot 0 is pinned to the **seed**, and
`trajectory_node._cycle_start_state` reads the seed's tilt straight off the
commanded state (`state.pose[3:5]`) — a *derived* pose, never corrected, so at
session start it is the FK-seeded STANDBY hold, level to the base. The whole
correction angle then lands as a **tilt step across the first few knots**, and
through the 744.3 mm tilt-centre lever that step is `arm·sinθ` of leg centroid
motion per 25 ms knot — the jerk spike. The floor lift is the loudest because it
has almost no genuine cup motion, so the tilt step is the whole of its jerk.

This is the same "two meanings of level in one node" class C-LEVEL-1 closes,
resurfacing at the E8 seed/banking seam. The seed is *correctly* uncorrected
(the machine really was at level-to-base); the banking body is *correctly*
gravity-level. The defect is that **the machine was never brought to gravity-level
before the launch was seeded from it.**

### Why it wasn't caught before, and why the single throw worked

The per-cycle POSITIONING move (`_position_platform_for_toss`) already sends an
identity ("level") intent that `trajectory_node`'s E3 ingest corrects into the
gravity-level counter-tilt — so cycles 2..n launch from a physically gravity-level
machine. The single throw flew precisely because its positioning ran first. The
**session-start floor lift is the one window that runs before any positioning**,
so it alone seeds from the un-positioned hold. On the 20:40 five-throw run the
session-start lift happened to succeed because the platform was left tilted by the
prior successful throw — the failure was position-dependent, which is exactly the
signature of a missing pre-condition rather than a wrong number.

### The fix

`reload_coordinator_node._unified_prelevel`: before the session-start floor lift,
command a corrected `go_to_pose` at the live xy/z with an identity intent (a pure
attitude move to gravity-level, worst leg ~2.8 mm — the same move `go_home` makes
after a `level`), and wait it out so the lift and launch seed from a stopped,
gravity-level machine. It reuses the exact E3 machinery the per-cycle positioning
already relies on; it is non-fatal (a refusal is reported and the lift/launch then
refuse by name, as before). No new correction of a seed — C-LEVEL-1's rule that
derived poses are never corrected is untouched; the platform is *physically* moved
into the frame instead.

### `num_throws=5` — deferred to R2

The chained cycle's solve ballooned to 2.2 s (`plan 2257 ms`) against the 1.8 s
launch lead, so the scheduled release passed before the plan installed and the
chain was torn down `ABORTED_NO_RELEASE`. The warm planner warms only a settle
shape; the first chained launch pays the full cold-solve. This is the UH-7a
chaining rung's problem and is on the R2 list, alongside making the launch robust
to the tilt snap without relying on raised limits (this fix does that for the
first cycle; the chain path should warm-start too).

### Row 18 (live ARMED cold-trip) is not reachable through the bench driver

`hand_stream_bench.py` clamps `--gap-delta` at ±1.5 rev, and its own deviation
belt (`--max-dev`) caps at 2.0 rev — below the 2.5 rev firmware
`MAX_DEVIATION_HAND` band — so the belt aborts before the guard by design. The
runbook and the driver help text both described a 3.0 rev trip that the clamp
forbids; corrected both. The ARMED cold-trip is proven per-commit by
`tests/firmware/native/test_fault_machine.cpp` ("hand deviation: observe-first
reports only; `hand7 arm`ed it LATCHES") at a genuine >2.5 rev exceed, so the row
is covered; a live-driver trip affordance is an R2 item.

## Fix

- `reload_coordinator_node.py`: `_unified_prelevel(why)` added; called in the
  unified session-start bring-up, after the warm-up and before the floor lift.
- `tests/ros/test_unified_launch_floor.py`: session-start ordering test extended
  to assert the pre-level precedes the lift; four focused `_unified_prelevel`
  tests (identity intent + live xy/z, refusal reported, stale-pose refusal,
  service-unavailable reported).
- `tests/hardware/hand_stream_bench.py`, `session_skill_stack_r1_flash.md`: row-18
  contradiction fixed (driver help + runbook), the R1 gate's `set_limits`
  precondition made explicit, the auto pre-level noted.

## Verification

- Offline reproduction of the two refusals and the fix's numbers from the sitting
  bag (`~/Desktop/rosbags/2026-09-11_19-36-52`), values in the table above.
- `pytest tests/ros/test_unified_launch_floor.py -q` (2026-09-11): **19 passed**.
- `./run_tests.sh --full` (2026-09-11): **6365 passed, 9 skipped, 1 xfailed** in
  321.13 s (parallel) + **4 passed** in 25.68 s (serial); `RESULT: PASS`.
