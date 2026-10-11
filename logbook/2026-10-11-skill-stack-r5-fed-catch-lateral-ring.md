---
title: a level-pinned feed catch still chased the raw arrival's lateral velocity — unified_cycle._events_for never projected it onto the pinned cup axis
type: investigation
date: 2026-10-11
status: resolved
phase: "two-ball-skill-stack — R5 (Ball Butler calibration) / catching"
related_plan: two-ball-skill-stack.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - tests/motion/test_unified_cycle.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skill_node.py
  - tests/hardware/session_skills_r5_sitting7.md
  - logbook/2026-10-11-skill-stack-r5-fed-catch-lateral-ring.md
  - logbook/INDEX.md
subsystem:
  - motion
tags:
  - catching
  - dynamics
  - kinematics
---

# A level-pinned feed catch still chased the raw arrival's lateral velocity

## Summary

`unified_cycle._events_for` built the `CatchEvent` the QP's matched-velocity term solves against from the RAW
arrival whenever a catch's touch-down tilt was pinned (`receive_tilt`) — only `hold_tilt` ever set the event's
`axis`, so a level-pinned feed catch still asked the legs to swing the platform to chase a hard sideways arrival, in
direct contradiction of the field's own docstring ("the cup absorbs the arrival's lateral skid instead of the legs
chasing it with the platform"). Fixed by projecting the matched-velocity TARGET onto the pinned cup axis
(`tg.cup_axis(rx, ry)`) and carrying it as a new `CatchEvent.vel_target`, which only the soft match reads;
`CatchEvent.vel` and the reported `CatchMark` keep the raw arrival.

## Symptoms

Bag `2026-10-11_11-27-53`, `columns_1ball_fed` x4: the fed ball's touch-down commanded the platform ±12/+9 mm
out-and-back through the catch, leg jerk at 89% of the session cap, the platform ringing ~4 Hz around touch-down,
and the cup still sliding ~(+65..73, +37..49) mm/s laterally at the `then_throw` release. All 4 attempts' next throw
landed +56..+74 mm / +39..+56 mm off the far site, and the following catch's lateral re-aim clamps at 20 mm, so all
4 ended `MISSED_CATCH`.

## Diagnosis

- `cup_cycle.py:1047-1059` (pre-fix numbering) sets the touchdown cup velocity TARGET to `catch_vel_ratio` (0.7, `cup_cycle.py:232`) ×
  the arrival's horizontal velocity, weight 200 (`:233`) — a soft pull toward a fraction of whatever `CatchEvent.vel`
  carries.
- `unified_cycle.py:1466-1483` (`_events_for`) built that event from `goals.catch_vel_mm_s` UNPROJECTED whenever
  `goals.hold_tilt is None` — which is every `receive_tilt`-pinned catch, since `hold_tilt` and `receive_tilt` are
  mutually exclusive (`plan_cycle`, :1601-1607). So the level-pinned feed catch (`schedule.py:931-940`,
  `receive_tilt=(0.0, 0.0)`) still handed the QP a 23-26° sideways target at the catch knot, even though the
  catch's own touch-down ATTITUDE (`unified_cycle._realize`'s `recv` pin, :1963-1967) was correctly held level. The
  platform's tilt was right; the velocity it was asked to chase was not.
- Offline re-plan (`f_replan.py` / `f_fix.py`, scratchpad, Opus brief F — nothing measured on hardware yet) confirmed
  the mechanism and sized the fix across a 5-arrival × 3-landing-offset matrix (`f_matrix.txt`): projecting the
  matched-velocity target onto the pinned cup axis before the solve collapses the chase. A1 (1279, 545, -3280 mm/s,
  undisplaced landing): cup |dxy| touchdown→release 10.32 mm → 0.22 mm offline, 10.71 mm → 0.215 mm reproduced
  through the production hop-entry construction (`test_skills_executor.py`'s new fixture, below); peak leg jerk
  177898 → 56209 mm/s³ offline (89% → 28% of the 200000 cap), 179118 → 56055 reproduced in-repo. Over the full
  15-case matrix the fixed |dxy| stays ≤ 1.34 mm and jerk ≤ 105k; `v_z` at touchdown and at release/takeoff is
  UNCHANGED (only the lateral pair moves); a baseline `LIMIT_JERK` refusal (the hardest arrival, 1600/700/-3800 mm/s,
  landing displaced -20 mm — 204513 mm/s³ against the 200000 cap) now plans at 97514.
- Steady (non-fed) columns catches are untouched: their arrivals are near-vertical, so the projection changes very
  little there, and the fix only triggers on `goals.receive_tilt is not None` — a field no steady catch sets.
- The swept admissible box's cells never set `receive_tilt`, so the re-sweep this change requires (it touches a
  `_GATED_FILES` member, `unified_cycle.py`) should be content-identical to the committed box — only the stamped
  `gate_hash` changes. Confirmed: `tests/motion/test_skills_admissible.py::test_the_committed_box_is_swept_at_the_
  launch_defaults_and_the_live_gate` fails on a hash mismatch alone (`gate_hash='1cd76c2d3c4a'` vs the new live
  `'c9d57ccab8fb'`, the first cut's) with this change in place and no re-sweep run yet — expected; the final code was
  re-swept as S9, gate `f974f9463a75`, content-identical — see Verification.

## Discussion

**A hypothesis was withdrawn.** The first pass over the bag blamed residual platform tilt / incomplete settle after
the previous throw for the lateral ring — i.e. that the platform was still carrying motion from the prior cycle into
the catch, not that the catch's own target was wrong. That did not survive a direct check: the pre-tilt REST already
puts the machine level and at rest before the catch window opens (`_start_tilt_for` / the held-attitude seam checks
elsewhere in `unified_cycle.py` exist exactly to guarantee this), and the re-plan with the seed unchanged and only
the velocity TARGET corrected reproduced the bag's own numbers almost exactly (10.71 mm / 89.6% jerk before, 0.215 mm
/ 28.0% after) — the window opens clean; the ringing is manufactured inside it, by the target the QP is handed.

**H1 (a site round-trip) was also considered and refuted.** The thought was that the far-site `then_throw`'s release
site might itself be off the cup's held line, forcing a lateral move independent of the catch's velocity target. The
offline matrix's `LAND=0,0` row (no landing displacement at all) still shows the full 10.3 mm / 320 mm/s chase before
the fix and the same collapse after it, so the site is not the mechanism — the velocity target is sufficient on its
own to produce (and to explain away) the whole effect.

**Alternatives considered and rejected** (owner, 2026-10-11):
- *Hold the fed ball one beat longer before the re-throw*, giving the platform more time to absorb the lateral
  velocity before it has to be level and still again. Rejected: it would no longer exercise the fed throw the way
  `columns` actually flies it — the whole point of R5's fed-columns box is the catch-to-throw dwell at its real
  duration, and padding it changes what is being tested, not just how it is tested.
- *Zero the lateral terminal directly in the executor* (i.e. force `landing_vel_mm_s`'s lateral components to zero
  before the terminal is built), rather than projecting inside `_events_for`. Rejected: that would misreport the
  arrival — every other consumer of `CatchTerminal.landing_vel_mm_s` (the learner's command, the outcome row, any
  future re-aim logic) needs the ball's REAL velocity, not a value already laundered for the QP's convenience. The
  fix has to live between the raw arrival and the one place that solves against a target — the soft match
  in `cup_cycle`, fed by `_events_for` — not upstream of it.
- *Apply the axial match to every level catch, pinned or not* (i.e. make this the default receive-velocity behaviour
  rather than gating it on `receive_tilt is not None`). Rejected: no measurable gain for an ordinary (non-fed)
  level catch, whose arrival is already close to vertical — the projection is nearly the identity there — and it
  would force a re-sweep of the WHOLE admissible box rather than the content-identical one this scoped fix produces,
  for no behavioural change on the box's own (near-vertical) cells.

## Fix

`ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py`, `_events_for`'s catch branch:

```python
catch_vel_m_s = _vec3(goals.catch_vel_mm_s, 'catch_vel_mm_s') / 1000.0
vel_target = None
if goals.receive_tilt is not None:
    recv_axis = tg.cup_axis(float(goals.receive_tilt[0]),
                            float(goals.receive_tilt[1]))
    vel_target = recv_axis * float(np.dot(catch_vel_m_s, recv_axis))
events.append(cc.CatchEvent(..., vel=catch_vel_m_s, axis=axis, vel_target=vel_target))
```

and `ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py`: `CatchEvent` gains
`vel_target: Optional[np.ndarray] = None`, and the velocity-match block reads
`match_vel = catch.vel if catch.vel_target is None else catch.vel_target` for both its per-axis target and the
held-axis `v_axial`; everything else keeps reading `catch_vel` (the raw `catch.vel`).

This is deliberately NOT `catch.axis`/`held_from_k` — that is `hold_tilt`'s held-LINE machinery
(`cup_cycle.py:965-1010` after the fix), which demands a non-`post_release` seed already seated exactly on the kappa line and
raises `CATCH_AXIS` otherwise; a fed catch following a THROW release does not meet those preconditions and setting
`axis` here would make every such catch unconditionally infeasible. The fix only reshapes the SOFT matched-velocity
target the existing per-axis branch (`cup_cycle.py:1063-1067` after the fix) already solves with `catch_vel_ratio` / `catch_vel_
weight` — one extra projection feeding that branch, which for `receive_tilt=(0.0, 0.0)` reduces exactly to
keeping `v_z` and zeroing the lateral pair.

`CatchMark` (`unified_cycle._meta_for`, ~`:2128`) is untouched: it already reads `goals.catch_vel_mm_s` directly, not
the event, so the reported catch velocity stays the raw arrival with no further change needed.

**Why a separate field, not a projected `vel` (audit, 2026-10-11).** The first cut overwrote `CatchEvent.vel`
with the projection. `cup_cycle` reuses the same local for `catch_runway_requirement` (`:1480` after the fix, reading `[2]`), the
STRUCTURAL slider-runway gate that must decelerate the ball's TRUE vertical speed. For the one live caller, the
`(0.0, 0.0)` pin, the projection keeps `v_z` exactly, so behaviour was identical; a future tilted pin would have fed the
gate a smaller axial `v_z` and under-reported the runway. `vel_target` keeps the field meaning what its docstring
says ("the OBSERVED incoming ball's touch-down velocity"). Other consumers: `segments.py:376,400` pass the raw
arrival into `CycleGoals` as intended; `cup_realize.py` reads no catch velocity. No other instance of the defect.

The docstring at `unified_cycle.py:469-470` ("no lateral channel is slaved, unlike `hold_tilt`") needed a word: it is
still true in the structural sense (no held-line equality), but the soft match is now projected onto the pinned
axis — amended in place.

## Verification

Scoped (`pytest tests/motion/test_skills_segments.py tests/motion/test_unified_cycle*.py tests/motion/test_skills_*.py
-q -p no:cacheprovider`, run 2026-10-11): **652 passed, 2 skipped, 1 deselected in 18.58 s**, deselecting
`test_skills_admissible.py::test_the_committed_box_is_swept_at_the_launch_defaults_and_the_live_gate` (the expected
`gate_hash` drift from editing a `_GATED_FILES` member — see Diagnosis; the orchestrator's re-sweep clears it). No
other existing pinned number moved.

New tests, confirmed to fail without the fix and pass with it (2026-10-11, the projection branch disabled in place:
`pytest tests/motion/test_skills_executor.py tests/motion/test_unified_cycle.py -q -k 'level_pinned or events_for'`
→ 3 failed; restored → 3 passed):
- `tests/motion/test_unified_cycle.py::test_events_for_projects_the_catch_velocity_onto_a_pinned_receive_tilt` — a
  pinned `CatchEvent`'s `vel_target` has exactly zero lateral velocity and unchanged `v_z` for `(0.0, 0.0)` while its
  `vel` stays raw; a tilted pin `(0.10, -0.05)` keeps `vel` raw and projects `vel_target` onto its axis; an unpinned
  one has no `vel_target`; a real `plan_landing` with `receive_tilt` set still reports the raw arrival in
  `CatchMark`.
- `tests/motion/test_skills_executor.py::test_a_level_pinned_hop_fed_catch_matches_a1s_arrival_axially_not_laterally`
  — the hop-entry feed catch for A1's arrival: cup |dxy| touchdown→release 10.71 mm → 0.215 mm, |vxy| at touchdown
  320.97 → 9.52 mm/s (both now ≤ the 1 mm / 20 mm/s bound); `CatchMark` keeps the raw A1 arrival.
- `tests/motion/test_skills_executor.py::test_a_level_pinned_hop_fed_catch_keeps_leg_jerk_well_inside_the_cap` — peak
  leg jerk 179118 → 56055 mm/s³ (89.6% → 28.0% of the 200000 cap, well inside the 50% bound).

Box re-sweep S9 (`tools/admissible_sweep.py`, the three runs of S7/S8 — columns, hop, single — dwell 0.27,
350/5000/200000, hand 3900, BLAS 1 thread, run 2026-10-11, logs `temp/logs/admissible_sweep_S9_*_20261011.log`):
gate `f974f9463a75`; all 19 boxes content-identical to the committed 2026-10-09 box (`1cd76c2d3c4a`) — only
`swept_at` and `gate_hash` moved, as expected (no swept cell sets `receive_tilt`).

Fed sims (`python sim/skills_gate.py --learn --no-viewer --pattern columns --apex-m 0.95 --separation-mm 125
--target-throws 30 --feed-angle-deg 11.9 --feed-speed-mmps 5600 [--one-ball-fed] --columns-entry hop`, run
2026-10-11 on the final code, log `temp/logs/sims_fed_axial_final_20261011.log`): two-ball PASS 5/5 (33 makes,
0 drops), one-ball fed PASS 5/5 (37 makes, 0 drops) — the same counts as before the fix. The sim tracks the legs
closely and never showed the ring, so this is a no-regression check, not evidence the fix works on hardware.

**One existing test moved, by design.** `tests/ros/test_skill_node.py::test_columns_bb_announcement_refuses_an_uncatchable_feed`
used a feed 30 mm past P1 because the cup had to reverse against Ball Butler's lateral arrival to match it; with the
match axial that feed now plans. Re-probed through the test's own path (live 350/5000/200000, run 2026-10-11,
`ex.plan_columns_first_cycle` wrapped to print its refusal): 0-35 mm plan clean, 40 mm LIMIT_VEL 353.8 mm/s (101 %),
60 mm LIMIT_VEL 419.7 mm/s (120 %). The test now uses 60 mm. The fix widened the feed's catchable band by ~10 mm.

Full gate (`./run_tests.sh --full`, run 2026-10-11, log `temp/logs/gate_full_fed_catch_axial_20261011.log`):
parallel 6668 passed, 9 skipped, 1 xfailed, **3 failed**; serial 6 passed in 22.60 s; RESULT FAIL, exit 1. The three
failures are the GUI replay `node --check` tests (`test_gui_replay_chart.py`, `_digest.py`, `_ball_history.py`),
outside this change: the gate's shell resolved Node v20.10.0 first on PATH, which does not load these ES modules
without a `"type": "module"` package.json. Re-run with Node v22.22.3 first on PATH (`pytest
tests/ros/test_gui_replay_chart.py tests/ros/test_gui_replay_digest.py tests/ros/test_gui_replay_ball_history.py -q`,
2026-10-11): 39 passed in 23.09 s.

## Outcome

The level-pinned feed catch now matches its own docstring: the platform's tilt was already pinned level, and the
catch's matched-velocity target is now pinned to the same axis, so the legs no longer chase the arrival's lateral
component at all. The trade-off this accepts is MORE in-cup lateral skid at contact — the cup is no longer catching
up to any of the arrival's lateral velocity, so the ball's relative speed across the cup floor at touch-down rises
from ~0.99 to ~1.29 m/s on the offline matrix's worst case. Whether that skid is still comfortably inside the cup's
physical catch envelope is a hardware question, not an offline one — see Open Questions.

## Withdrawn claims

- [2026-10-11] Claimed the bag's lateral ring was residual platform tilt / incomplete settle carried over from the
  previous cycle.
  WITHDRAWN: the pre-tilt REST already guarantees a level, at-rest seed before the catch window opens; a re-plan
  from that same seed with only the velocity TARGET corrected reproduced the bag's numbers almost exactly, so the
  seed was never the problem.
  Superseded by: the Diagnosis section above — the matched-velocity target `_events_for` built was unprojected.
- [2026-10-11] Hypothesis H1: a site round-trip (the `then_throw` release site off the cup's held line) as an
  independent contributor to the lateral ring.
  WITHDRAWN: the offline matrix's undisplaced-landing row (`LAND=0,0`) shows the full chase before the fix and the
  full collapse after it with no site displacement at all.
  Superseded by: the Diagnosis section above — the velocity target alone is sufficient to produce and to explain the
  effect.

## Open Questions

- **Hardware check owed**: `tests/hardware/session_skills_r5_sitting7.md` — sitting 7's runsheet includes the
  in-cup lateral skid check (the ~0.99 → ~1.29 m/s trade-off above) at contact, on the real hand/ball, which nothing
  offline can confirm.
- The admissible box re-sweep (`tools/admissible_sweep.py`) is owed before this change ships to the committed box's
  `gate_hash` — expected to be content-identical per the Diagnosis note, but unconfirmed until run.
