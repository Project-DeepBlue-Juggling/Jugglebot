---
title: "R5 sitting 2 (2026-10-02): the 0 deg gate fails on timing not catch rate, Ball Butler's feed collides with Jugglebot's own throw because the sitting-1 receive_tilt fix never reached the wire, and the owner's site-swap fix for the collision costs 101% leg acceleration undisplaced"
type: investigation
date: 2026-10-02
status: in-progress
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-09-30-skill-stack-r5-sitting-1.md
  - 2026-09-30-skill-stack-r5-columns-bb-start.md
files_changed:
  - ros_ws/src/jugglebot_interfaces/srv/InstallSegment.srv
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/trajectory_node.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/sites.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - sim/skills_gate.py
  - config/hardware_config.yaml
  - config/generated/admissible_box.yaml
  - config/generated/hardware_config.py
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - tests/motion/test_cup_cycle.py
  - tests/motion/test_validate_cycle.py
  - tests/ros/conftest.py
  - tests/ros/test_install_segment.py
  - tests/ros/test_skill_node.py
  - tests/motion/test_skills_executor.py
  - tests/motion/test_skills_install_origin.py
  - tests/sim/test_skills_gate.py
  - tests/hardware/skills_plan_bench.py
  - tests/hardware/session_skills_r5_sitting2.md
  - tests/hardware/session_skills_r5_sitting3.md
  - plans/active/two-ball-skill-stack.md
subsystem:
  - motion
  - ros
  - tracking
tags:
  - performance
  - kinematics
---

# R5 sitting 2: the wire omission, the columns collision, and the swap that isn't free

## Summary

Sitting 2 (2026-10-02, launch 12:43-13:01, bag `2026-10-02_12-43-28`) set out to confirm
sitting 1's evening fix units at the 0° reload gate, then fly Ball Butler-fed columns for the
first time. The 0° gate caught all 10 fed balls but missed both of its timing criteria (landing
+55.3 ms vs. a ±30 ms band; 1/8 seats smooth) — **FAIL** against the pre-registered criterion,
catch rate alone passing. BB-fed columns never completed a cycle: of 7 attempts, 4 let ball A's
own throw fire and then had the feed catch refused at install, with the two balls' mocap markers
optically merging in flight in all 4 — confirming the owner's "reliably collided" read — and the
other 3 had A's own throw refused `ORIGIN_TOO_LATE` before anything left the cup. Root cause of
the live refusal: the `receive_tilt` level-pin fix that landed 2026-09-30 evening was never
wired onto the `InstallSegment` service, so the live dispatch still plans with the banked
`tilt_to_receive` pin it was meant to replace, while the offline pre-throw check and the sim
gate plan in-process and never exercise the wire. The owner's proposed collision fix (swap which
site each ball uses) is geometrically right but not free: the swapped layout's undisplaced catch
refuses `LIMIT_ACC` at 101 %, clearing only with the feed re-aimed 20-30 mm toward A's site.
Status `in-progress`: three mechanisms found, and six fix units landed the same afternoon (Fix,
below) — but none have flown yet, so the hardware gate moves to sitting 3.

## What the owner reported

2026-10-02 afternoon, verbatim: did **not** re-run the BB accuracy-volley re-fit (cone throws
have been reliable; the fit is slow). `self_toss` reload at `hold_tilt_max_deg 0.0`: every feed
caught, cleanest "silky smooth". BB-fed columns: many attempts, none successful — some
`ORIGIN_TOO_LATE` ("solves taking too long"), in the others both robots threw their first throw
and Jugglebot's throw "reliably collided" with Ball Butler's shortly after release. Owner's
read: BB aims at x=+50 while JB throws from x=-50; proposal: JB starts at +50, moves to -50 to
receive the feed, then columns; also asked whether apex/separation could rise. A separate Claude
session's uncommitted `skill_node.py`/`test_skill_node.py`/`ros_ws/gui` edits (GUI goal
announcements) were on the tree during this analysis and are not touched here.

## Measured

1. **Session.** Legs 300/5000/200000, hand 3500; reloads at `hold_tilt_max_deg 0.0`; columns
   apex 0.9 m, separation 100 mm, 6 cycles, `reload=true`; the installed `ball_butler_node.py`
   carries sitting 1's `+37/+15 ms` announcement constants.
2. **Feeds at 0° (10 flown, 10 caught).** Landing vs. committed mean **+55.3 ms** (sd 24.2,
   n=8): release -11.2 ms vs. announced (the push-lag constant works), flight **+88.1 ms** longer
   than BB's predicted ToF 0.882 s (sitting 1: +14.7 ms — the flight residual got worse, not
   better). Seat (HELD after landing) 121-340 ms, 1/8 inside the +0.05..+0.15 s band. Landing
   offset +10..+38 mm +x, y ±22 mm (sitting 1: +11 x / +27 y). BB's own aim correction (+17,
   -200) mm vs. sitting 1's (+30, -190) — the stale 2026-06-09 affine, not re-fitted. BB solution
   yaw 15.7°/pitch 68.3°/3.37 m/s (sitting 1: 17.7°/69.7°/3.34 m/s); predicted ToF 0.882 s, both
   sittings identical.
3. **Columns attempts (7, `launch.log` 1855-2113).** Feed aimed at P2=(50, 0, 830); ball A
   thrown from P1=(-50, 0) — owner's geometry read is correct. Attempts 1, 3, 6, 7: A's THROW
   installed (44-66 ms), then the feed CATCH-with-throw **refused at install**: `LIMIT_VEL`
   349.5/352.2/350.4/377.7 mm/s > 300, plus cup contact `a_cup,z` under the -6864 (-0.70 g) floor
   at 2-3/17 knots over `contact_knots=[(6, 22)]`. Attempts 2, 4, 5: the THROW itself refused
   `ORIGIN_TOO_LATE` (solve 0.076/0.083/0.084 s, "first knot would be 0.151-0.159 s old at the
   wire") — A never threw.
4. **Collision (raw mocap).** In all four attempts where both balls flew, the ball markers
   become optically unresolvable (< ~75 mm, below the 74 mm two-ball-touch threshold) +0.17..
   +0.19 s after A's release, 548-583 mm above the 830 mm plane (attempts 1/3/6: 2-4 frames;
   attempt 7: merge at +0.05 s, ~1.0 m up, unresolved for 0.8 s — more severe). Attempt 1's A
   track shows no kink (a graze); 3 and 6 show a real velocity change after the merge (which ball
   is ambiguous). B's descent crosses A's column (x=-50) ~0.5 m above the cup, ~0.09 s before its
   own landing, while A is at 0.5-0.6 m up.
5. **Root cause of the live feed-catch refusal.** `receive_tilt` is **not on the
   `InstallSegment` wire**: the `.srv` carries `hold_tilt_{set,rad}`/`rest_tilt_{set,rad}` only;
   `skill_node.py:1542` encodes `hold_tilt` only; `trajectory_node.py:4098-4105` rebuilds the
   `CatchTerminal` with `hold_tilt=` only, so `receive_tilt` comes back `None` and the banked
   `tilt_to_receive` pin fires. Offline, the flown layout's undisplaced feed catch WITH the pin
   plans at 240.2 mm/s (80 % of cap); WITHOUT it (`receive_tilt=None`, as the wire sends it):
   `LIMIT_VEL: peak leg velocity 366.5 mm/s > 300.0; cup contact ... -7039 ... 2/17 knots violate`
   — the live refusal's signature (the live 349.5 differs only by the announced velocity/seed).
6. **`ORIGIN_TOO_LATE` mechanism** (`executor.install_segment`, fresh-origin branch, ~984-1071).
   `t0 = t_now_s` is fixed **before** the solve; the schedule's own `LEAD_S` = 9 knots = 0.225 s
   is already spent as window, so the staleness check sees only `WIRE_READ_KNOTS*dt` = 0.075 s
   (3 knots), never the 0.150 s `SOLVE_BUDGET_KNOTS` a splice gets. Plans of 44/66 ms passed;
   76/83/84 ms refused. `trajectory_node` passes `lead_s = sk_exec.LEAD_S` for every install
   (`trajectory_node.py:865, 4222-4227`).
7. **Layout feasibility** (pin ON, apex 0.9/sep 100/dwell 0.3/hand 3500/limits 300-5000-200000):
   **FLOWN** (A@P1, feed@P2): ok through dx ≤ +20 (240/4088/168.9k mm/s,mm/s²,mm/s³ = 80/82/84 %
   at dx 0; 292/4620 = 97/92 % at +20); refuses at +30 (`LIMIT_VEL` 323.8, 108 %); dy ±30
   unchanged; hand 3264 (93 %). **SWAPPED** (A@P2, feed@P1): dx 0 refuses `LIMIT_ACC` 5038.9
   (101 %); dx -10/-20 refuse `LIMIT_VEL` 311.6/341.1; dx +10/+20/+30 ok at 88/80/72 % leg vel
   (margins 91/83/76 % acc, 74/73/n.m. % jerk) — the cup must reverse against the ball's own
   lateral velocity, costing the extra margin. Separation 125/150 refuse `LIMIT_VEL` 358.7/434.2
   at every apex (100 mm already spends the whole 0.278 s transit near 300 mm/s). Apex 1.0:
   swapped undisplaced refuses `HAND_LIMIT_ACC` 3668.7 @ 3500 cap, ok @ 3900 (94 % hand) — but
   the columns box only exists at apex 0.9 today. Apex 1.1: A's own THROW refuses
   `HAND_LIMIT_ACC` at both 3500 and 3900 caps.
8. **Other (carried, not this sitting's focus).** Hop at 0°: five `ABORTED_NO_RELEASE` after a
   MISSED throw 1 (23-52 mm off, re-aim refused `INFEASIBLE`/`HAND_LIMIT_C2`); every hop re-aim
   this sitting refused at 0.95 m, yet 2/2×5 and 4/4×2 caught open loop. One `self_toss` reload
   ended `ABORTED_NO_RELEASE` after throw 3/4 MISSED (+49, +48 mm). Leg heartbeat dropouts as
   before (known load-gated class).

## Discussion

**(a) Root cause, and why the pre-throw check and sim gate both passed anyway.**
`Skill.receive_tilt` (`schedule.py:307`) and `CatchTerminal.receive_tilt` (`segments.py:177`) are
wired correctly *inside the process* — `compile_columns` sets `(0.0, 0.0)` on the feed catch
(`schedule.py:725-734`) and `executor._catch_terminal` passes `skill.receive_tilt` straight
through (`executor.py:1658`). What never happened is the round trip through
`trajectory/install_segment`, the path the live hardware dispatch actually uses: no
`receive_tilt_*` field exists on `InstallSegment.srv`, so `skill_node.py:1542`'s encode and
`trajectory_node.py:4098-4105`'s decode both only touch `hold_tilt`. Both
`executor.plan_columns_first_cycle` (the pre-throw check `_install_columns_schedule` runs before
accepting the columns goal) and `sim/skills_gate.py`'s feed branch call the same *in-process*
`_catch_terminal`/`plan_segment` machinery with the executor's own terminal — which still carries
`receive_tilt=(0.0, 0.0)` because it was never serialised. Neither exercises the wire, so neither
could have caught the omission, and no existing test round-trips a `CatchTerminal` with
`receive_tilt` set through the service. The sitting-2 planning facts even recorded
"`InstallSegment.srv` did NOT change tonight" as a fact about that evening's diff — in hindsight,
that is exactly the line that should have been read as a flag.

**(b) Two hypotheses withdrawn on the way to (a).**
- *"The live catch aimed at the tracker's fit, past the +30 mm feasibility cliff."* Withdrawn: a
  replay through `install_segment` read `/balls` at the CATCH dispatch instant and found
  `landing_from_fit=False` both times — per `_catch_aim`'s own ranking the executor falls back to
  the schedule prior, and replaying that prior with the executor's own terminal plans cleanly at
  257 mm/s. The tracker fit was never the live input.
- *"The live splice landed ~5 knots late."* Withdrawn: the pinned, passing offline plan carries
  the identical `contact_knots=[(6, 22)]` as the live refusal — the contact window is not what
  differs.

**(c) Why the owner's site swap is right for the collision but not free.** The collision
geometry (Measured 3-4) matches the owner's read exactly: BB always aims at P2 and A always
throws from P1, so B's descent always crosses A's column — swapping sites removes the crossing
by construction. But Measured 7 shows it is not a free substitution: the *undisplaced* swapped
catch refuses `LIMIT_ACC` at 101 %, worse than the flown layout's own +30 mm cliff. The
mechanism is directional: in the flown layout the cup's transit and the ball's lateral velocity
point the same way; in the swapped layout they oppose, so the cup must decelerate, stop, and
reverse to match the ball at touch-down — that reversal, not distance, costs the extra
~950 mm/s². It only clears with the feed re-aimed 20-30 mm toward A's site — a measured, 72-91 %
margin, but a second coordinated change (BB's aim), not a drop-in site relabel. Separation and
apex do not help on their own: 125/150 mm refuse `LIMIT_VEL` at every apex tried, and 1.0 m needs
the hand cap raised to 3900 plus a fresh box sweep (today's box only covers 0.9 m).

**(d) The `ORIGIN_TOO_LATE` mechanism, in one sentence.** A fresh-origin THROW/CATCH fixes its
wire timestamp before the solve runs, and the schedule's own 0.225 s lead is already spent as
window — so the staleness check sees only the 0.075 s `WIRE_READ_KNOTS` margin, not the 0.150 s
`SOLVE_BUDGET_KNOTS` a splice gets. A 76-84 ms columns THROW-from-rest solve (three of seven
attempts here) is within what the system tolerates for a *splice* but has no equivalent budget
as a *fresh* origin — the REST branch already rebases instead of refusing (Measured 6); U2 below
proposes the same affordance for THROW/CATCH.

**(e) The 0° gate against its criterion, and where the +88 ms residual sits.** Sitting 1's
criterion (≥9/10 caught AND ≥8 smooth seats AND landing within ±30 ms, 3 filmed) is **not met**:
catch rate passes (10/10), landing (+55.3 ms) and seat band (1/8) do not. The miss is on the
flight-duration term specifically — release is now close to on-time (-11.2 ms, confirming
`BB_RELEASE_PUSH_LAG_S` works) but flight ran +88.1 ms over BB's own predicted ToF, worse than
sitting 1's +14.7 ms. That ToF prediction (0.882 s) is identical across both sittings despite the
throw solution's yaw/pitch/speed shifting slightly — pointing at a fixed bias in BB's own
ballistic model (U3), not at anything this sitting changed on the Jugglebot side.

**(f) One hypothesis withdrawn while building the `STALE_STATE`-class fix (2026-10-02
afternoon).**
- *"A still-moving REST hits the documented 'starts from REST but the machine is moving'
  `STALE_STATE` branch and refuses on that literal message."* Withdrawn: probing
  `_cycle_start_state`'s call site found that branch structurally dead for this caller — it is
  always invoked with kind `SETTLE`, whose `_KIND_SHAPE` sets `post_release=True`, so the literal
  branch never executes. The real path is that a still-moving REST feeds its live, nonzero
  velocity/acceleration into the seed (physically wrong for a reserved origin, since the REST is
  at rest by t0), and the QP either solves awkwardly or the existing post-solve
  `_install_continuity_ok` check refuses it under the generic `STALE_STATE` "commanded state moved
  during planning" message instead. The fix below targets the actual path, not the named-but-dead
  branch.

## Fix (landed 2026-10-02 afternoon)

Owner decisions (`AskUserQuestion`): swap the layout with the feed aimed 20 mm toward A, not a
straight site relabel; fix `ORIGIN_TOO_LATE` now (Opus, design-bearing); the other session's
GUI/announcement commits (`7d4e6151`, `adadc006`, `6ed328b8`) were already in, so no coordination
wait; skip the BB volley re-fit again (the +x bias is benign under the swapped layout); raise the
hand cap to 3900 rev/s² (pre-approved 2026-09-30 "if the probe needs it") and re-sweep. Six units
landed, in this order:

**U0 — `receive_tilt` over the `InstallSegment` wire** (the root cause, Discussion (a)).
`InstallSegment.srv` gains `bool receive_tilt_set` / `float64[2] receive_tilt_rad`;
`skill_node.py`'s CATCH-branch builder (~1542) encodes `terminal.receive_tilt` via
`_wire_tilt_out`; `trajectory_node.py`'s `_segment_terminal_from_request` CATCH branch decodes it
via `_wire_tilt`; `tests/ros/conftest.py`'s mocked `Request` grows the two fields. New in
`tests/ros/test_install_segment.py`: 3 round-trip tests plus a **wire-map contract**
(`_THROW/_CATCH/_REST_WIRE_MAP`, `_REST_WIRE_EXCLUDED`, `_assert_wire_map_complete`, three
`test_every_*_terminal_field_is_wire_mapped_or_excluded` tests) — a terminal field added without
matching wire plumbing now fails this gate by name, closing the whole class, not just this field.
**Deferred, found not fixed:** `RestTerminal.holds_ball` never crossed the wire either — the
node-side REST rebuild defaults it `True` for every REST, over-constraining an empty-cup REST's
cup-contact floor; documented in `_REST_WIRE_EXCLUDED` rather than silently flipped (Open
Questions).
Triple: 2026-10-02, `pytest tests/ros/test_install_segment.py tests/ros/test_skill_node.py
tests/ros/test_skills_plan_bench.py tests/ros/test_unified_cycle_levelling.py -q` → **293 passed
in 14.40 s**.

**U1 — swapped layout, feed aimed toward A** (the owner's collision fix, Discussion (c)). A now
holds/throws at P2 (+50), the feed is caught at P1 (-50), aimed 20 mm toward A (x = -30 relative
to P1). `skill_node.py`: `_columns_feed_aim_site(feed_site, a_site, offset_mm)`; params
`columns_feed_site` (`'P1'` default, `'P2'` restores the flown layout) and
`columns_feed_aim_toward_a_mm` (20.0, clamped `[0, 40]`, WARN past the clamp); `_run_columns`
derives `feed_site`/`a_site`, threads them through the `Pattern`, the reload call, the bridge
compile and the log lines. `sim/skills_gate.py` mirrors it (`feed_aim_toward_a_mm` +
`--feed-aim-toward-a-mm`, `columns_feed_site` + `--columns-feed-site`). Tests:
`test_skill_node.py` (aim-site-relative landings, cup xy `(-30, 0)`, 5 new parameter tests),
`test_skills_gate.py` (landing recomputed from the offset).
Triples: 2026-10-02 `pytest tests/ros/test_skill_node.py tests/sim/test_skills_gate.py
tests/motion/test_skills_schedule.py -q` → **325 passed in 81.43 s**; the sim rehearsal at the
*old* 3500 hand cap (`--pattern columns --seeds 0 1 2 3 4 --apex-m 0.90 --feed-angle-deg 11.9
--feed-speed-mmps 5600`) **FAILED 5/5 seeds** — every seed refused after 8 throws,
`HAND_LIMIT_ACC` 3545 > 3500, which drove the hand-cap unit next, not a layout defect; leg-channel
margins unchanged by the cap question (dx +20: vel 240.6/acc 4164.2/jerk 145886; dx 0 still
`LIMIT_ACC` 5038.9, the undisplaced knife edge Discussion (c) measured).

**Hand cap 3500 → 3900 rev/s², box re-swept.** Diagnosis (`sim_hand_margin_report.md`): the
segment refusing the FAIL-5/5 rehearsal above was *not* the feed catch — it was ball A's 4th
same-site catch-and-throw fold (skill_idx 8): 3538.3 rev/s² (101.1%) swapped, 3551.3 (101.5%)
flown, the swap does not move it. Per-fold hand peaks climb 93.3→101.1% across a 6-fold attempt
(a knife edge, not a drift); the ball that *starts an attempt HELD* runs 3-6 points hotter than
the fed ball, both layouts — the 2026-09-30 sim PASS 5/5 landed the same knife edge on the safe
side by chance. `config/hardware_config.yaml` `hand_acc_limit_rps2` 3500 → 3900 (the ceiling,
`hand_acc_ceiling_rps2`, was already 3900 — "just under the measured C-HAND-2 authority bound of
3925.5"); `generate_config.py` regenerated `hardware_config.{py,h}`, the three firmware header
copies, and — externally — `BallButler/ball_butler_main/{hardware_config.h, protocol_config.h}`
(stale since the 2026-09-27 kincal apply; BB firmware does not read the JB section, inert there).
No Teensy source reads `HAND_ACC_LIMIT_RPS2`, so no flash needed. `SetTrajectoryLimits.srv` is
legs-only and `skill_node._live_limits` reads the config constant directly — the launch default
**is** the session cap, no `set_limits` for hand exists. Box re-swept (gate `f96fc30012d7` →
`2adc56a46def` for the regenerated hardware_config.py, then → `458bb544b345` once cup_cycle.py's restated bound was made to import it — see the paragraph after the triple, `scratchpad/sweep_run_hand.sh <run> 200000 3900`: columns/hop/single in parallel,
BLAS 1 thread); run G and run H bit-identical-checked, merged, installed.
Triple: `2026-10-02, `bash <scratchpad>/sweep_run_hand.sh G2 200000 3900` then the same for H2 (`tools/admissible_sweep.py --site-pairs columns|hop|single ... --leg-jerk 200000 --hand-acc 3900`, OMP/OPENBLAS 1 thread; logs `temp/logs/admissible_sweep_r5_{G2,H2}_{columns,hop,single}_20261002.log`): run G2 17:22, run H2 17:45, every per-pattern file bit-identical (md5 columns b861f44f0b0f9eb54c1ff15e51dfbdef, hop e6728484bc6cdfa03b75bb7a4f1b6f4c, single 7cf97592ee2f4a2aefbb07df3fe6ac1c); run H2 merged into `config/generated/admissible_box.yaml` (md5 2337f5ccd22c8f80e83445aa5a71b30d, 19 boxes, gate 458bb544b345, limits 300/5000/200000, hand 3900) — the 0.88-0.93 columns cells keep their xy authority ([-4, 1] x [-20, 20] at P2, [-2, 4] x [-20, 10] at P1) with the apex range widened 0.576-0.992, and the 0.92-0.97 band is no longer EMPTY ([-8, 6] / [-6, 8] x [-20, 20])`

**The first 3900 sweep was wrong, and a test said so.** The first full-suite run after the config change (16:45) failed `tests/motion/test_cup_cycle.py::test_runway_default_decel_is_the_signed_off_hand_limit` on its pre-existing assertion `cc.HAND_ACC_LIMIT_RPS2 == hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2`: `motion/trajectory/cup_cycle.py` restated the hand cap as a literal 3500 (the YAML's comment said cup_cycle "mirrors it for the in-QP bound" — by hand), so runs G/H (16:21/16:44, gate `2adc56a46def`, box md5 e43b03365f7cf943ef02b80bfd8d8a19) and the 16:54 fed-columns PASS 5/5 at that box ran with the gate at 3900 and the QP bound at 3500. The constant now imports the YAML value (a gated file, so the gate hash moved to `458bb544b345`) and the box was swept a second time — the cells are identical, the header is not. The same lesson as 2026-09-27: a config constant's DERIVED-literal consumers are what a grep misses; the fixtures that pinned 3500 (seven node-test boxes, the validate-cycle detail and runway strings) now follow `hw.JB_TRAJ_HAND_ACC_LIMIT_RPS2`, and the held-LANDING fingerprint guard freezes its runway decel at 3500/HAND_REV_PER_M because its `bc` fingerprint was captured there.

**U2 — fresh-origin solve budget** (the `ORIGIN_TOO_LATE` mechanism, Discussion (d); Opus, design-bearing, `design_fresh_origin_budget.md`). Verified against source first (CLAUDE.md control-cycle rule): the emitter already samples knot 0 for tau ≤ 0 (frames before t0 are the flat hold already streaming), and firmware never sees a plan origin — every frame carries its own knot timestamp (`leg_interp.cpp:714-717`), the only check is the 250 ms future cap, and `sched_apply` (492-529) compares curves, so promotion is exact whether the lane plays or holds. `executor.install_segment(..., reserve_fresh_lead=False)`: with the flag, a fresh THROW/CATCH plans from `t0 = t_now + lead_s` (0.225 s) behind a `WINDOW_TOO_SHORT` pre-check, and `ORIGIN_TOO_LATE` reuses the splice arithmetic — the budget becomes 0.150 s (`SOLVE_BUDGET_KNOTS`, what a splice already gets, not the 0.075 s `WIRE_READ_KNOTS` a fresh origin had) and no knot is skipped; REST rebase and the non-fresh path unchanged. `SkillExecutor(..., dispatch_lookahead_s=0.0)`; `skill_node._DISPATCH_LOOKAHEAD_S = 1/_TICK_HZ + JB_TRAJ_KNOT_DT_S` (0.050 s: one tick + the planner node's knot-grid rounding) on all five executor constructions — every start dispatches 50 ms earlier (first-throw window `[0.4-transit, 0.45]`, not the accidental 0.600; reload-catch window 0.5, not 0.725). Tests: `test_skills_install_origin.py` (+9 cases), `test_skills_executor.py` (look-ahead ×2), `test_install_segment.py::test_throw_from_rest_accepts_at_a_fresh_origin` pins `t0 - LEAD_S == at`; `skills_plan_bench.py` passes the flag + look-ahead; `INVARIANTS.md` K1 row and `schedule.py`'s ~1341 comment corrected. Worked cycle (attempt 2, this bag): release 839.043, dispatch due 838.418, snapped t_now 838.443 — OLD: t0=838.443, solve 0.076 s ends 838.519 > the 0.075 s margin → `ORIGIN_TOO_LATE`; NEW: t0=838.668, window 0.375 s, accepted, six ticks (838.525-838.650) send knot 0 (rest, zero velocity/feedforward), plan runs from 838.668, release stays at 839.043.
Triples: 2026-10-02 `pytest tests/motion/test_skills_install_origin.py tests/motion/test_skills_executor.py tests/ros/test_skill_node.py -q` → **352 passed in 12.27 s**; combined with `test_install_segment.py`/`test_skills_plan_bench.py` → **453 passed in 16.76 s**.
**Knot-skip finding (follow-up, not landed).** Under the OLD rule every *accepted* fresh THROW (the 44-66 ms solves attempts 1/3/6/7 passed) already skipped its first 2-4 knots — a hand step of 0.015-0.07 rev / 0.7-1.4 rev/s against firmware promotion tolerances of 0.005 rev / 0.5 rev/s; firmware only counts it (lane playing, not held), it does not refuse — a candidate for the "jerky" first-throw look the owner reported 2026-09-30. Confirming it needs this bag's bridge hand scheduled-group counters `promo_over`/`promo_dp`/`promo_dv`/`promo_da` (`s_sg[SG_HAND]`, `leg_interp.cpp:1758-1774`) — not pulled this session, carried to sitting 3.

**`STALE_STATE`-class seed fix** (found landing U2's look-ahead — Discussion (f)). `trajectory_node.py`'s at-rest gate (`_cycle_start_state`) reads the *live* pose at dispatch; a still-streaming REST moves 2.4-33 mm/s in the 25 ms before its own end, and a rebased REST ends 0.16-0.18 s after its nominal end — so a fresh skill dispatched at the previous REST's nominal end (U2's window, moved 50 ms earlier) is refused fail-closed even though the REST will be at rest by the reserved origin. Sitting 2's `hold_tilt 0.0` (a near-null pre-tilt REST) hid this; the look-ahead widens the window it bites in. Fix (`trajectory_node.py` `_svc_install_segment`, one hunk): when a record is still streaming (`t_now_s < record.end_s`) but the fresh condition fires (`t_now_s + lead_s >= record.end_s`), skip `_cycle_start_state` and seed from the record's terminal rest (`_rest_seed(record)` — exact zero velocity/accel, inherited levelling), the trust model a splice already uses; with no record, or a fully-ended one, unchanged. Worked cycle: pre-tilt REST rebased +0.17 s; CATCH dispatched 50 ms before nominal end; `t0 = dispatch + 0.225 = T+0.175 ≥` the REST's real end `T+0.17` (5 ms margin). Tests (+3): a fresh THROW installs while the previous REST still streams (hand 1.6-3.4 rev/s); nothing streaming + a moving hand still refuses (`_IN_MOTION`, pre-existing); a streaming REST drifted > 1.0 rev still refuses via `_install_continuity_ok`'s `STALE_STATE` "commanded state moved during planning" — the real safety net stays.
Triple: 2026-10-02 `pytest tests/ros/test_install_segment.py tests/ros/test_skill_node.py -q` → **99 failed / 123 passed** — all 99 in `test_skill_node.py`, identical with the hunk reverted (box-loading tests refusing on the hand-cap/gate mismatch pending the 3900 box install, not this fix); `pytest tests/ros/test_trajectory_node.py -q` → **129 passed, 1 failed** (`test_emitter_cadence_40hz`, wall-clock cadence under load, passes in isolation — known load-flake class). Audit (2026-10-02, behaviour finding, applied): the skip is gated to THROW/CATCH — a fresh REST is NOT lead-reserved (`install_segment` keeps its origin at the dispatch instant), so it must keep the live seed, moving or not; `test_a_fresh_rest_while_the_previous_rest_still_streams_seeds_from_the_live_state` pins that knot 0 sits on the live hand (2026-10-02, `pytest tests/ros/test_install_segment.py -q` → 36 passed in 3.05 s).

**Sim stream loop** (`sim/skills_gate.py` mirrors the fixes above). Local `_DISPATCH_LOOKAHEAD_S` (0.050 s, names the node constant rather than importing `skill_node`, which pulls `rclpy`) on all 4 executor constructions; the installer passes `reserve_fresh_lead=True` with the dispatch tick as `t_now`; frames before a future t0 sample knot 0 (the hold) — before this the sim sent the plan's first knot the instant it installed, which the robot never does. Frame stamps come from a never-restarting counter; the firmware-mirror check now runs against the plan the last frame came from and covers the pre-t0 hold. Sim hand cap: `_SESSION_HAND_ACC_RPS2 = 3900.0`, the learner config follows it (an agent's attempt at this edit was declined by the auto-mode classifier as a safety-limit change from an agent message; the orchestrator applied it directly after the owner's own answer).
Triple: 2026-10-02 `pytest tests/sim/test_skills_gate.py -q -p no:cacheprovider` → **24 passed in 73.65 s** — the mirror test that had failed at a 0.137 rev gap now passes; predates the hand-cap/config change, so it verifies the stream-loop mechanism only, not the gate at 3900.

## Verification

All commands run 2026-10-02 against this sitting's own bag (`~/Desktop/rosbags/
2026-10-02_12-43-28`) and launch log (`~/.ros/log/2026-10-02-12-43-28-061299-jetson-3268694/
launch.log`).

- `python tools/probes/feed_catch_bag_probe.py --bag ~/Desktop/rosbags/2026-10-02_12-43-28 --log
  ~/.ros/log/2026-10-02-12-43-28-061299-jetson-3268694/launch.log`: **10 flown feeds, 10 caught;
  landing vs. committed: mean=+55.3 ms stdev=24.2 ms (n=8)**.
- `scratchpad/probe_feed_nopin.py` (one-off): flown layout WITH the pin "ok, leg vel 240.2 acc
  4088 jerk 168897"; WITHOUT it "REFUSED ... LIMIT_VEL 366.5 > 300.0 ... 2/17 knots violate" —
  matches the live signature.
- `scratchpad/probe_feed_swap.py` + `probe_feed_margin.py` (one-off): dx/dy and separation/apex/
  hand scans reproduce the Measured 7 / Discussion (c) margins and the 125/150 mm and apex
  1.0/1.1 refusals exactly.

No sitting-flight test suite ran during the original analysis above — bag/log analysis plus
one-off probes only. The Fix section's per-unit triples (2026-10-02 afternoon) cover the six
units individually; these cover the combined working tree, filled in before commit:

- `2026-10-02, `bash <scratchpad>/sweep_run_hand.sh G2 200000 3900` then the same for H2 (`tools/admissible_sweep.py --site-pairs columns|hop|single ... --leg-jerk 200000 --hand-acc 3900`, OMP/OPENBLAS 1 thread; logs `temp/logs/admissible_sweep_r5_{G2,H2}_{columns,hop,single}_20261002.log`): run G2 17:22, run H2 17:45, every per-pattern file bit-identical (md5 columns b861f44f0b0f9eb54c1ff15e51dfbdef, hop e6728484bc6cdfa03b75bb7a4f1b6f4c, single 7cf97592ee2f4a2aefbb07df3fe6ac1c); run H2 merged into `config/generated/admissible_box.yaml` (md5 2337f5ccd22c8f80e83445aa5a71b30d, 19 boxes, gate 458bb544b345, limits 300/5000/200000, hand 3900) — the 0.88-0.93 columns cells keep their xy authority ([-4, 1] x [-20, 20] at P2, [-2, 4] x [-20, 10] at P1) with the apex range widened 0.576-0.992, and the 0.92-0.97 band is no longer EMPTY ([-8, 6] / [-6, 8] x [-20, 20])` — the hand-cap re-sweep's G/H bit-identical check (gate `2adc56a46def`).
- `2026-10-02 17:45-17:49, `python sim/skills_gate.py --learn --no-viewer --apex-m 0.90 ...` at the installed 3900 box (gate `458bb544b345`; logs `temp/logs/skills_gate_<name>_3900_20261002_1745.log`): fed columns `--pattern columns --seeds 0 1 2 3 4 --target-throws 30 --feed-angle-deg 11.9 --feed-speed-mmps 5600` → **PASS 5/5 seeds in 105.7 s** (the swapped layout, feed aimed 20 mm toward A, the reserved fresh origin and the 50 ms look-ahead all in); `--pattern self_toss --seeds 0 1 --target-throws 20` → PASS 2/2 (54.9 s); `--pattern hop --seeds 0 1 --target-throws 20` → FAIL 2/2 (53.5 s; pre-existing at HEAD, see Open follow-ups); `--pattern self_toss --reload --seeds 0 1 --target-throws 20` → seed 0 FAIL `LIMIT_JERK` on a REST / seed 1 PASS (8.5 s; pre-existing at HEAD, see Open follow-ups)` — the working-tree `sim/skills_gate.py` table at hand 3900, all three
  patterns.
- `2026-10-02 17:49, `./run_tests.sh --full` (every tier, `nightly` included; log `temp/logs/gate_full_r5_sitting2_fixes_20261002_1749.log`): **5817 passed, 9 skipped, 1 xfailed in 291.80 s**, then the serial phase 6 passed in 19.88 s — run after the last code edit (the audit's REST gate) and before the commit; colcon rebuilt both packages afterwards from the same tree (`temp/logs/colcon_build_r5_sitting2_final_20261002_1745.log`)` — `./run_tests.sh --full` on the combined working tree, before commit.
- `/audit --unstaged` (2026-10-02, audit-reporter over the whole phase diff): ISSUES FOUND — one behaviour WARNING (the fresh-origin seed-skip in `trajectory_node._svc_install_segment` was not gated to THROW/CATCH; a fresh REST is not lead-reserved and must keep the live seed — applied, with `test_a_fresh_rest_while_the_previous_rest_still_streams_seeds_from_the_live_state`) and one narrative NOTE ("three of six" → "three of seven" in Discussion (d) — applied); the wire round-trip and mutual exclusion, the ORIGIN_TOO_LATE arithmetic at its boundaries, every hand-cap consumer, the look-ahead threading and the two new parameters' validation were verified clean.

**Hardware gate for sitting 3.** None of the six Fix units above have flown. The 0° timing gate,
the BB-fed-columns collision, and `ORIGIN_TOO_LATE` were all found and fixed from bag/log analysis
and offline probes, same discipline as this entry's own Measured section — the hardware gate moves
to `tests/hardware/session_skills_r5_sitting3.md`.

## Open Questions

- **`RestTerminal.holds_ball` wire gap** (U0, found not fixed) — the node-side REST rebuild
  defaults every REST to holding a ball, over-constraining an empty-cup REST's cup-contact floor;
  flown that way for weeks, documented in `_REST_WIRE_EXCLUDED` rather than silently flipped.
- **Hop sim-gate re-aim refusals** — the fed-columns gate table also ran hop: FAIL at HEAD *and*
  on the working tree (spliced CATCH re-aims refuse `INFEASIBLE`/`SINGULAR`, 38 installs accepted)
  — pre-existing, matches the hardware's own re-aim refusals every sitting since R4.
- **Reload sim-gate closing-REST `LIMIT_JERK`** — also pre-existing at HEAD (211212/207571 >
  200000 mm/s³); working tree reproduces it on seed 0, not seed 1 — not isolated.
- **The `promo_over` confirmation** — the U2 knot-skip finding needs this bag's bridge hand
  scheduled-group counters pulled and compared against the owner's "jerky first throw" report
  from 2026-09-30; not done this session.
- **The BB flight-ToF residual** (+88.1 ms vs. sitting 1's +14.7 ms, same predicted 0.882 s ToF
  both sittings) — BB-side (U3), not traced further here.
- **The BallButler repo's dirty headers** — `generate_config.py`'s regeneration wrote
  `BallButler/ball_butler_main/{hardware_config.h, protocol_config.h}`, an external repo this
  worktree does not manage; needs its own commit or revert there.
- **The BB `platformio.ini` upload-hook item**, carried from sittings 1-2: still calls the main
  checkout's protocol-6 host tool, dark against this worktree's protocol-9 bridge; use
  `tools/teensy_link_bridge.py --fw-update --target bb` directly.
- **The 12° hold default** — `hold_tilt_max_deg`'s operator-facing default stays 12° (R4-flown);
  the columns feed catch no longer reads it (fixed receive-level attitude), but Block A patterns
  still need it set deliberately per block, same as sitting 2.
- The `self_toss` reload miss on throw 3/4 (+49/+48 mm) — a single instance, no pattern.
- Leg heartbeat dropouts, same known load-gated class (`plans/active/leg-bus-frame-drops.md`).
