---
title: "The catch through-seat rate ships at zero — the trajectory builder manufactures no platform motion nobody asked for"
type: feature
date: 2026-07-26
status: in-progress
phase: "Self-toss anomaly fixes — catch-reach-degenerate-overshoot Phase 3"
related_plan: "catch-reach-degenerate-overshoot.md"
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/planner.py
  - ros_ws/docs/catch_arrival_contract.md
  - ros_ws/docs/levelling_frame.md
  - tools/probes/catch_reach_replay.py
  - tools/probes/levelling_tilt_bag_check.py
  - tools/probes/README.md
  - tests/motion/test_trajectory_planner_catch.py
  - tests/ros/test_levelling_frame.py
  - tests/ros/test_trajectory_node.py
  - tests/hardware/session_anomaly_fixes.md
  - plans/active/catch-reach-degenerate-overshoot.md
commits:
  - <pending>
subsystem:
  - motion
  - ros
tags:
  - safety
  - testing
---

## Summary

`planner._CATCH_TILT_THROUGH_RATE_RADPS` changes from `0.07` to **`0.0`**, so the
catch trajectory builder no longer *manufactures* an arrival tilt rate. A catch
nobody gave an opinion about is now **reach + quiescent hold** — two segments, the
rim stationary at ball contact, the settle exactly on the commanded target, and the
plan 0.15 s shorter.

One line of behaviour; the work is everywhere else. Most of it is keeping the
C-CATCH-1 test block from going silently vacuous, and writing down a physical risk
this change accepts rather than solves.

## Motivation

Operator decision, 2026-07-26, taken after Phase 2 landed and stated in their own
framing because the framing is the load-bearing part:

> Platform motion during a catch (or throw) is **permitted but never mandated**.
> The only reason the platform should be moving at contact is that *the planner
> determined it produces a more optimal trajectory* — never because a constant in
> the trajectory builder imposed it.

Stationary-platform catches are the near-term preference — only the hand moving to
receive the ball. That is a *lean, not a rule*: more aggressive juggling will later
need a moving platform during catches and throws, so nothing may enshrine
stationarity either.

Setting the manufactured fallback to zero is the cleanest expression of that. The
builder now has no opinion of its own, and motion appears only when a caller asks
for it.

## Design

**A default, explicitly not a rule.** Nothing was added that asserts a catch
commands no motion, and nothing should be. `tilt_through_rate_radps` is unchanged,
the decay-segment code path is unchanged, and C-CATCH-1's rule that an
explicitly-supplied rate is returned **verbatim and unbounded** is unchanged. That
is the seam a future optimising planner uses, and it is pinned by a test rather
than left as an intention.

**C-CATCH-1 stays live and stays tested.** At a zero default the bound never binds.
It is kept because it is the guard for the moment the constant is raised again —
which its own docstring has always anticipated as a seat-tuning session. A bound
deleted the day it stops binding is a bound that will not be there when the value
moves.

## Discussion

**Why the vacuity trap was most of the work.** With the default at zero, the
C-CATCH-1 bound becomes unreachable in two independent and entirely silent ways:

1. assertions of the form `rate == approx(planner._CATCH_TILT_THROUGH_RATE_RADPS)`
   degenerate to `0 == 0`;
2. tests that pass `tilt_through_rate_radps=` take the *requested* branch of
   `_catch_arrival_rate`, which is deliberately unbounded — so they stop exercising
   the bound at all.

Between them the whole `test_ccatch1_*` block would have stayed **green while
proving nothing**, which is worse than deleting it: a green vacuous suite is read
as coverage. Every such test now restores a non-zero **module constant** through a
`_set_seat_rate(monkeypatch)` helper — never through the kwarg, because that is the
one route that cannot reach the bounded branch.

That claim was verified by mutation rather than by reading. Both halves of the
bound were removed from `_catch_arrival_rate` and the block re-run: **5 of the 7
`ccatch1` tests fail**, and the two that survive are by construction negative cases
(`..._leaves_a_request_alone_where_it_is_already_satisfied` and
`..._keeps_the_seat_on_an_on_pose_supersede` both assert the bound does *not* bind,
so passing without it is correct). The tree was restored and re-verified by md5.

The same trap appears twice more and is closed the same way in both: the ROS tests
capture `_PRE_FIX_SEAT_RATE_RADPS = 0.07` as a record rather than a mirror, and the
replay probe's single mirror was split into a *live default* mirror (`0.0`, compared
against production) and a *capture record* (`0.07`, pinned to itself). Without that
split, syncing the probe to the new default would make every pre-2026-07-26 bag
score `NOT-REPRODUCED` — the harness would have reported the historical record as
broken rather than reporting itself as re-baselined.

**The physical risk this accepts, stated rather than mitigated.** The `0.07`
existed for one reason: *a parked **tilted** rim deflects the ball* (the bb-sim
geometry finding at `planner.py:774-782`). The two catch paths have very different
exposure:

- the **self-toss** catch seats level, so a level rim has no deflection geometry and
  zero costs it nothing — under C-CATCH-1 its seat rate was already zero;
- the **reload** catch seats at 11.08°, which is exactly the geometry that finding
  concerns, and its rim is now stationary at contact. This is the live risk.

It is deliberately not compensated for in code. Two facts bound it without removing
it, and both are worth knowing before scoring a miss: `0.07 rad/s` has **never been
validated on hardware** (its own leg-velocity sizing note was 2× wrong until it was
measured — 14.24 mm/s, not ~7), and until commit `407154f` the seat was aimed off
the *plan-frame* tilt, so on every levelled catch it pointed along the levelling
correction rather than at the ball. No bench impression of the seat was ever formed
on a correctly-aimed one. The reload is therefore the experiment, and
`tests/hardware/session_anomaly_fixes.md` § Section ZSEAT scores it — ZSEAT-2 sets
PASS/ABORT by binomial sigma against sitting 4's 15/19 and distinguishes
*bounce-out* (the seat-deflection signature) from *missed-arrival* (BB scatter, not
evidence about the seat).

**A stale runbook row that was inverted, not merely wrong.** § CHECK CCATCH-3 was
written one phase earlier and told the operator to ABORT if the commanded tilt is
flat over the last 0.8 s. Flat is now the **PASS**. Rather than rewrite a sibling
section — the runbook is append-only precisely so parallel phases cannot clobber
each other — a superseded banner at the head of CCATCH-3 corrects the four affected
rows individually, so a top-to-bottom reader cannot act on any of them. A runbook
row that contradicts the shipped code is worse than a missing one.

**Why this landed as its own phase.** It changes commanded motion on the shipping
reload path, so it gets its own commit, its own entry and its own bench section
rather than riding along inside Phase 2. The rollback is a one-line default, with
C-CATCH-1 already in force to bound whatever value replaces it.

## Implementation

- `planner._CATCH_TILT_THROUGH_RATE_RADPS = 0.0`, with the constant's comment block
  rewritten to state the decision, the default-not-a-rule distinction, the seam that
  keeps motion permitted, the reload risk, and the recipe for bringing a non-zero
  rate back.
- `_catch_arrival_rate`'s docstring records that the fallback ships at zero, that
  the bound therefore does not bind today, and that the tests reach it by
  monkeypatching the module constant.
- `tests/motion/test_trajectory_planner_catch.py`: a `_set_seat_rate(monkeypatch)`
  helper, used by 12 tests including every `test_ccatch1_*`.
  `test_catch_has_reach_decay_hold_segments` keeps its 3-segment case under a
  patched rate and gains 2-segment coverage at the shipped default.
- `tools/probes/catch_reach_replay.py` and `levelling_tilt_bag_check.py`: mirror
  split into live-default vs capture-record, so recorded sessions keep reproducing.
- `ros_ws/docs/catch_arrival_contract.md` records that the contract is dormant at
  the shipped default and why it is retained.

## Verification

- **Mutation test of the C-CATCH-1 block** (2026-07-26): both bound halves removed
  from `_catch_arrival_rate`, `pytest tests/motion/test_trajectory_planner_catch.py
  -q -k ccatch1` → **5 failed, 2 passed** (the 2 are negative cases by design).
  Tree restored and confirmed byte-identical by md5.
- **Scoped** (`pytest tests/motion/test_trajectory_planner_catch.py -q`, run
  2026-07-26): 25 passed in 27.21 s.
- **Full suite**: `pytest tests/ -q`, run 2026-07-26 on this Jetson in the project
  venv: **3574 passed, 3 xfailed, 198 warnings in 1382.00 s (23:02)**, exit 0.
  Baseline at HEAD `490ad34` was 3569 passed, 3 xfailed in 1399.52 s. Delta
  **+5 passed**, accounted for exactly: +3 functions in
  `test_trajectory_planner_catch.py` (21 → 24), +1 in `test_trajectory_node.py`
  (130 → 131), and +1 case from `test_quiescent_hold_is_still` gaining a
  `parametrize` over `(0.0, _SEAT_RATE_RADPS)` so the hold-is-still property is
  swept over both the shipped two-segment and the seated three-segment shapes.
  **Zero test functions were removed** from any file (verified by diffing the
  `def test_` name sets against HEAD), zero `xfail`/`skip` added, and the xfail
  count is unchanged at 3. The run was the last action before staging and no
  `*.py` was modified after it started (verified with `find -newermt`), so the
  tested tree is the committed tree for every file pytest reads.
- **Recorded-session replay**: `tools/probes/catch_reach_replay.py` against
  `~/Desktop/rosbags/2026-07-25_15-17-48` — all seven reaches still `REPRODUCED`.

## Open questions

- Whether the seat rate should return non-zero for tilted catches after the bench
  scores ZSEAT-2. If it should, the fix is the one-line default and C-CATCH-1
  bounds it automatically.
- Whether the rate belongs in `config/hardware_config.yaml` rather than as a planner
  constant, so a bench sweep needs no code change. Left as a planner constant here:
  config-keying adds codegen surface for a value that is currently zero.

## Related

- `plans/active/catch-reach-degenerate-overshoot.md` § Phase 3
- `ros_ws/docs/catch_arrival_contract.md` — C-CATCH-1
- `logbook/2026-07-26-catch-reach-overshoot-fix.md` — Phase 2, which landed C-CATCH-1
- `tests/hardware/session_anomaly_fixes.md` § Section ZSEAT — the bench score
