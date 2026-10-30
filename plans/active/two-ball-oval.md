---
title: Two-ball oval — two balls in one hand, every throw carried back in the cup
created: 2026-10-17
status: active
owner: Harrison
last_updated: 2026-10-30
pathway: two-ball-oval
scope: >-
  Two balls in one hand on an oval: every throw T to C, the ball carried back in the cup. Five phases, each
  gated by a hardware rung: P1 the oval pattern dispatchable (fixed roles, cross-site fold, C-CUP-4 check, box
  cell at dwell 0.65) and rung 1 on the loaded carry at 150 mm / 0.65 s / 0.95 m; P2 the banking leak fix
  (retired, the leak does not reproduce); rung 2 at dwell 0.55 then 0.50 on the as-is level planner; P3 the
  pose-level carry planner behind `level | pose` with the C-CUP-4 rows; P4 one working-point move
  600/8000/200k, acc ceiling 10000, 30 A, dwell ladder to the first plannable dwell (0.45 today; amended 2026-10-24); P5 the two-ball `oval` with the
  hop-shaped fed entry, closed by 30 consecutive Ball Butler-fed catches in one attempt. "Not with this cup"
  at rung 1 is a valid end. Where we are lives in § 3 of the plan.
steps:
  - {id: P1, title: The oval pattern dispatchable, kind: phase, status: complete, date: 2026-10-18, effort: m, hardware: none}
  - {id: P2, title: Banking leak fix, kind: phase, status: retired, date: 2026-10-23, effort: s, hardware: none}
  - {id: P3, title: Pose-level carry planner, kind: phase, status: complete, date: 2026-10-30, effort: m, hardware: robot}
  - {id: P4, title: Working point and the dwell ladder, kind: phase, status: not_started, effort: m, hardware: robot}
  - {id: P5, title: Two-ball oval, kind: phase, status: not_started, effort: l, hardware: robot}
related_logbook:
  - 2026-10-17-two-ball-oval-plan-written.md
  - 2026-10-18-two-ball-oval-p1-oval-pattern-dispatchable.md
  - 2026-10-23-oval-p2-banking-leak-not-reproduced.md
  - 2026-10-23-oval-box-cells-dwell-055-050-swept.md
  - 2026-10-23-oval-rung2-flown-dwell-050.md
related_config:
  - config/hardware_config.yaml → trajectory_op.leg_acc_ceiling_mmps2 (5000 → 10000 at P4, decision 53)
  - config/hardware_config.yaml → trajectory_op.leg_{vel,acc,jerk}_limit_* (350/5000/200000 → 600/8000/200000 at P4, decision 54)
  - config/hardware_config.yaml → leg_curr_limit_a (15 → 30 at P4, with the ODrive backup JSON, decision 55)
  - config/generated/admissible_box.yaml (oval cells; dwell_s joins the select() key, decision 39)
related_code:
  - ros_ws/src/jugglebot/jugglebot/motion/skills/schedule.py::OneBallPattern.roles / _append_one_ball_pattern / ThenThrow.release_site / _fold_catch_throw_pairs / compile_oval (P5)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py (release-site threading, § 4 P1)
  - ros_ws/src/jugglebot/jugglebot/motion/skills/admissible.py::PATTERNS / select / check_limits
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py::_assemble (C-CUP-4 rows, level | pose mode)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py (banking request; P2 RETIRED 2026-10-23, the sweep-as-request idea is a note in P3)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py (CUP_SEATING_CONE / CUP_SEATING_FLOOR / CATCH_CLOSING_SPEED, CATCH_SKID diagnostic)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py, sim/skills_gate.py, tools/admissible_sweep.py
---

# Two-ball oval

Branch: `main` in `~/Desktop/Jugglebot` (merged from `skill-stack` / `two-ball-oval` 2026-10-23; the oval worktree is retired; the `two-ball-oval` branch is kept at `main`'s tip, but work continues on `main` directly, fly after a FULL launch restart).
Design record: the wayfinder map was at `.scratch/two-ball-oval/` in the retired oval worktree (gitignored, may no longer exist; the
58 decisions it holds are restated here in § 1.3 so the plan stands alone). Runbook for the first
rung: `tests/hardware/session_oval_rung1.md`. Build runs phase by phase through
`/next-phase-prompt two-ball-oval <n>`. **§ 0 of `two-ball-skill-stack.md` (archived) stays
normative for every unit of work here**: one path per function, delete what is replaced in the
same rung, refuse by name, no hand-picked thresholds without a measured origin.

## 1. Context

### 1.1 The pattern

Two balls in one hand on an oval in the vertical x–z plane: every throw is released at the THROW
site T and caught at the CATCH site C, 150 mm away; the caught ball rides back to T in the cup
(the *carry*) inside the dwell and is thrown again. Columns (the flown R5 pattern, 32 consecutive
catches 2026-10-13) is the only carry-free two-ball one-hand pattern; a flown return needs apex
> 3 m and alternating directions on one line collide. The oval is therefore the first pattern in
which the cup translates while loaded — the loaded carry is the only new physics on the robot,
and whether this cup can carry a ball 150 mm in 0.30 s is what the plan finds out.

### 1.2 What the machine dictates

- Columns timing identity `t_f = d + 2τ` holds with `d` now holding the LOADED carry; the empty
  transit τ is jerk-bound at ~0.28–0.30 s. Lower throws shrink the carry budget, and the
  ball-clearance bound (`sep·(1 + d/t_f)/2 ≥ 75 mm + buffer`) forbids an oval narrower than
  ~150 mm. 175 mm is hand-bound (the empty hand's post-release braking at the stroke top, cap
  3900 rev/s² 0.7 % under the measured C-HAND-2 authority) and is OUT.
- Cup: spherical base R 35, cone to the Ø 85 rim, ball Ø 75 seats on the cone ring; a rigid
  frictionless ball stays centred for any required specific force within **55°** of the cup
  axis. C-CUP-4 (§ 2.4) keeps the planned force inside 30° of the prescribed axis as a CHECK (45 deg from the stroke end since 2026-10-19; 30 deg the fall-back) (25° of margin; amended 2026-10-18, 20° is the P3 shaper target).
- Measured offline (production planner, two runs byte-identical per probe): the one-ball oval
  plans at today's limits 350/5000/200k from dwell 0.40 at 150 mm; the two-ball oval at dwell
  0.30 plans only under the pose-level carry planner at 600/8000/200k, apex 1.0 (leg jerk 194k,
  residual 17.1°). A hard capture-span catch (lateral slaved to axial through touch-down) is
  QP-infeasible at every two-ball cell: the lowest-jerk carry turns the cup round laterally
  THROUGH the touch-down. The level catch program's measured skid is 18–84 mm/s; the BB feed
  catch already tolerates 1.3 m/s.
- The sim gate's capture is kinematic (`contact_carry=False`): seating is a hardware-only
  verdict. The sim certifies choreography, fold, timing, accept codes and learner convergence.
- Every rung at new limits is a YAML working point + box re-sweep + commit + relaunch: one box
  file, one limits stamp pinned to the launch default, a live raise refuses `LimitsMismatch`.

### 1.3 Decision ledger (owner, 2026-10-14 → 2026-10-17; numbers are the map's)

- **Scope (1, 2, 6, 8, 9, 12, 14):** destination = this plan; the loaded carry IS the pattern;
  BB-fed starts only (hand starts out); gate 30 consecutive; out of scope = cup/hand hardware,
  BB firmware, mocap, three balls, two hands; the shake test is scrapped (the first rung is the
  carry's first real test); "parallel ovals" is the named backup destination if the carry fails.
- **Geometry (4, 10, 11, 16, 18, 37, 47, 57)** (16 amended 2026-10-18, see Amendments)**:** separation ≥ 150 mm, work up from 150; 55° cup
  cone; one-ball oval first; first rung = 150 mm / dwell 0.50 s / apex 0.95 m at today's limits,
  no limits or box-size change before it; sites symmetric `columns_sites(150)` with fixed roles;
  **T = P1 (x = −75, nearest BB), C = P2 (x = +75)** — JB throws away from BB; 175 mm out.
- **Carry (17, 19–22, 25–27, 33):** one release-to-release STEADY window, carry folded
  (`ThenThrow` gains a release site; split rest-terminal segments rejected at 260k); the carry
  bank is ONLY the sweep between the two event tilts; the banking leak fix is its own phase
  under principle (a) (loaded-span request = the sweep; C-CUP-1's per-instant rule applies to
  the free cup only); seated-carry constraint = QP rows (shaper) AND realised-plan check;
  never-stop principle recorded with its lever (free seam dz state) and its caveat.
  **Amendment 2026-10-23 (owner):** the banking-leak phase is RETIRED (§ 4 P2) - the 40-110k leak
  did not reproduce on today's planner; the "sweep as the request" idea survives only as a note in P3.
- **Catch (28b, 40, 48)** (48 amended 2026-10-18)**:** the level program with a receive-pin TILT hold
  `[k_td − h_catch_pre, k_td + h_catch_post]`, no lateral slaving; throw pin held `[n−2, n]`;
  skid LOGGED only; closing speed REFUSES below 20 % of the 0.7× match target; the BB feed
  catch keeps the program each rung already flies (`reload` held-axis at one ball, columns LEVEL
  pin + axial-only match at two balls).
- **Contract (29–31, 35)** (29–31 amended 2026-10-18)**:** C-CUP-4 — 20° axis-relative cone + `a_z + g ≥ 0.5 g` on every
  interior loaded knot of every pattern; check in P1, QP rows in P3; codes `CUP_SEATING_CONE`
  / `CUP_SEATING_FLOOR`; `INVARIANTS.md` entry.
- **Planner (23, 24, 32, 36, 58):** the oval pattern is P1 because nothing live dispatches a
  fixed-role oval; the pose-level carry planner is the steady formulation for EVERY pattern
  behind a `level | pose` mode, default `level` until decision 24's hardware gate (flipped to `pose` 2026-10-30); named knobs
  h_pre 2 / h_catch_pre 2 / h_catch_post 1 / w 10 with one Phase-3 re-tune (the catch holds may
  go to zero); the seam-dz lever is a P3 ablation that ships only with the release-state
  composition; a null result closes decision 33.
- **Software shape (34, 38, 39, 41–45, 49)** (38, 39, 44 amended 2026-10-18)**:** phases P1–P5 with a hardware rung between each;
  `workspace_xy_m` stays the plant constant with one accept-time refusal (|site xy| + box
  half-extent + 20 mm ≤ workspace, mirrored in the sweep); box labels `oval_1ball` / `oval`,
  fed variants share the sibling box, `dwell_s` joins the `select()` key, dwell stays a node
  parameter; learner shared `plant_id`, rows keyed by the RELEASE site xy (T), catch offset kept
  as state; a carried throw releases at T exactly and every throw is aimed from its planned
  release to its landing target (release-position error is a diagnostic); one-ball = fixed-role
  rule on the existing one-ball path, two-ball `compile_oval` at P5; the one-ball fed entry is
  `compile_reload` at T + the fixed-role tail, no entry code.
- **Entry / exit (46, 50, 51):** BB feeds T; the two-ball entry is fed columns' hop-entry shape
  verbatim (parked LEVEL feed catch at T, self-tossed after one dwell, cup transits to C, steady
  oval from ball 1's catch), first throw a LEARNED throw, timing machinery carried over
  unchanged; exit = the columns Stop (held catch at C, last ball lands on it; NOT landed in P1, see the P1 Outcome).
- **Limits (5, 13, 52–56):** two-ball point apex 1.0 at 600/8000/200k, jerk untouched; only
  `leg_acc_ceiling_mmps2` moves (5000 → 10000); one working-point move at P4 start with the box
  re-swept (oval + columns cells, same commit); leg current 15 → 30 A in that commit; ramp
  sitting = five columns canary attempts, then the one-ball oval at 0.40 (the 0.40 rung IS the
  ramp).

- **Amendments, P1 (owner 2026-10-18; originals above stay for the record; measured origin per line;
  full record `logbook/2026-10-18-two-ball-oval-p1-oval-pattern-dispatchable.md`):**
  - *16* — first rung dwell 0.50 → **0.65**: the dispatched plan carries the R4 pre-release hold (0.100 s =
    4 knots); 0.50 reads 32.1°, 0.60 21.5°, 0.65 passes, 0.70 14.9°. The probe's 18.7° was a hold-less plan
    nothing dispatches. Ladder 0.65 → 0.50 → 0.40 → 0.30; P3's shaper earns 0.50 back.
  - *Residue, 2026-10-19 (C-CUP-4 amended to 45°, judged from the hand stroke's end; 30° the fall-back —
    `logbook/2026-10-19-cup-seating-cone-refuses-columns-stroke-braking.md`)* — the minimum passing dwell 0.51 (logbook
    2026-10-18) was measured at the 30° cone and needs RE-MEASURING at 45° (indicative only: the rung-1 window gate reads dwell
    0.45 OK at 39.4°, 0.40 refused at 49.1°); the committed cell stays 0.65 until then. The *29–31* wording
    below (30° / 0.30 g) is the pre-amendment value. Rung 1 is the first loaded CARRY flown under 45°: record
    `max_cup_braking_deg` and restore 30° if a carry loses the ball.
  - *29–31* — C-CUP-4 check **30° / 0.30 g on every pattern** (20° / 0.5 g stays the P3 shaper TARGET): at
    20° / 0.5 g the check refused flown patterns (250 mm hop with landing offset 28.7° / 0.320 g; `reload`
    held-axis feed catch 20.0–20.2°). Estimator = the REALISED cup-opening acceleration, not the QP's
    (18.7° vs 26.6° on one plan). "Columns passes trivially" withdrawn: it passes (the 1.3° / 0.02 g margin is the HOP's, 28.7° / 0.320 g vs 30° / 0.30 g; columns' own margin is unmeasured, the sim-gate columns point at dwell 0.25 or 0.30 is refused while 0.27 passes).
  - *38* — workspace rule `|site xy| + box half-extent ≤ workspace_xy_m`, **no 20 mm margin** (125 + 20 + 20
    refused the flown columns and hop); code `REJECTED_SITE_OUTSIDE_WORKSPACE`, every pattern.
  - *39* — `dwell_s` a REQUIRED `select` / `describe_miss` keyword; `check_limits` validates file-level limits.
    *Amended 2026-10-21 (owner):* `dwell_s` travels on the Juggle goal (it was a node parameter); the GUI pre-fills it per pattern from the box's swept dwell (columns/hop/self_toss 0.27, oval 0.65), editable; the node parameter is the fallback when the goal sends 0. The 22:53 sitting's oval was refused for a 0.27 node dwell against the 0.65 box.
  - *44* — "shared `plant_id`" → oval sittings use a **separate `plant_id`**: T = P1 = (−75, 0) is the same
    `x[0:2]` as every columns / self_toss P1 row and `h_seat` does not separate them.
  - *48* — `CATCH_CLOSING_SPEED` 50 % → 25 % → **20 %** of the 0.7× target, held-axis catches exempt (cup stationary
    at touch-down by design). Achieved ratios: p50 0.58 over 479 windows, columns cells 0.44–0.48, BB feed
    0.42, hop landing 0.47–0.49, hop carry 0.349, near-hard capture 0.114, the oval 0.533. Origin of 20 %: the lowest catch ratio ever flown and installed is the sitting-6 (2026-10-06) high catch at the historical 930 mm plane (arrival −5520 mm/s, realised −960 vs target −3864 mm/s = 0.248), replayed by `tests/ros/test_install_segment.py` (pinned at 930 mm on purpose); 0.20 sits below it with margin and 1.75× above the 0.114 near-hard capture catch; the first re-base to 0.25 refused that replay.
  - *P1 wording* — box: EVERY cell re-swept (`feasibility.py` is in the gate hash), values-equal pin; executor:
    eight sites plus the settle rest, Stop 'after' rest, BUMP-CARRIED fence; Stop mid-carry declines 'before'
    (`STOP-BEFORE-CARRIED`): on a carried fold the Stop offers only the 'after' rest, the throw completes, the cup rests at T, the ball is NOT caught (as hop today). Decision 57 (175 mm out) stands; the CHECK no longer refuses it (21.4°).

- **Amendments, P3 (owner 2026-10-23; measured on the PRODUCTION segment path with the real holds, which the ticket-11
  prototype never saw; full record `logbook/2026-10-23-two-ball-oval-p3-pose-level-carry-planner.md`):**
  - *23 / 36 (the z band "stays in")* — the free centroid-z block ships **disabled** (`pose_z_band_mm: 0`, the block is
    not assembled; the knob stays for a hardware A/B). Measured: a ±30 mm band saturates on every accepted oval cell and
    costs +57..+62 mm/s leg velocity and +270..+320 mm/s² leg acc for the SAME peak jerk, and refuses the 0.45 cell on the
    sweep margin where pinned z passes. Leg velocity is not in the objective. **Decision 33 (seam-dz lever) closes null**;
    decision 58's ablation is moot without a dz block.
  - *32 (knobs h_pre 2 / h_catch_pre 2 / h_catch_post 1 / w 10)* — **h_pre 2, w 10 confirmed; h_catch_pre 0,
    h_catch_post 0.** The receive holds cost 11 / 13 / 10 % of window jerk at 0.50 / 0.45 / 0.40 and change nothing
    else: the quintic sweep arrives at the pin with zero rate and acceleration. h_pre 0 → hand 4166 rev/s² at 0.40; 4 →
    +35 % jerk at 0.50. w 1 → leg velocity 427 vs 401 at 0.40; 30 → `HAND_LIMIT_ACC` at 0.45.
  - *28b / 40 (skid logged only)* — stands, with the number: under the leg-jerk objective the cup arrives at C moving
    225 / 379 / 494 mm/s across the receive axis at 0.65 / 0.55 / 0.50 (level 14 / 12 / 33). The two levers were
    measured and declined (lateral catch weight ×100: 10 / 48 / 105 mm/s at +18 % jerk, hand refusal at 0.40; hard
    touch-down match: 55 mm/s at +28 % jerk / +67 % leg acc). The swept column's seat-bit read-back judges it.
  - *29–31 (C-CUP-4 rows in P3)* — landed as specified: 20° / 0.5 g rows about the PRESCRIBED cup axis (inscribed 8-gon)
    on knots catch+1 .. release−2, in pose mode only; the check stays 45° / 0.30 g. The rows do ALL the seating work
    (without them the pose arm reads 37–76°); 30° buys no jerk, so the 20° target is free.
  - *24 (the mode switch)* — `SegmentConfig.planner_mode` + ONE live parameter (skill_node's, carried on every
    `InstallSegment` request, selecting the box cell AND the solve) + the box stamped `planner_mode` (`select` refuses a
    mismatch by name; the oval cells coexist under both planners). Default `level` until the § 8 A/B sitting.
  - *Shaper target* — at dwell 0.40 the residual is 18.7° (target met) and the refusal moves to leg velocity (357 >
    350 mm/s): the cone hands the ladder to P4's working-point raise, as P3 was meant to.
- **Amendments, P3 tail (owner 2026-10-24; from the two A/B sittings of 2026-10-24, bags `2026-10-24_10-44-21` and
  `2026-10-24_11-34-29`; full record `logbook/2026-10-24-two-ball-oval-p3-ab-sittings-skid-bound.md`):**
  - *28b / 40 (skid logged only)* — **RE-DECIDED: the skid is a BOUND** (`I-PLAN-14`, `PoseConfig.touchdown_skid_mmps`
    175 mm/s, inscribed 8-gon rows on the cup's lateral velocity at the catch time, pose mode only). Hardware read the
    cup moving INWARD at landing, median −507 mm/s at dwell 0.50 under `pose` (−174 at 0.65, `level` +20), and 9 of
    the 10 `pose` misses were balls landing > 15 mm inward meeting the advancing rim (9 of 16 missed; 0 of 7 under `level`);
    throw quality was identical across columns (landing sd 16–19 / 13–15 mm). 175 = the 0.65 figure that caught 63/64.
    Cost: window jerk 44.3k → 53.3k at 0.50 (level 111.5k), 56.5k → 73.9k at 0.45; 0.40 now refused `HAND_LIMIT_ACC`.
  - *24 (the hardware gate)* — **NO-GO for `pose` as built at dwell 0.50**: 47/57 (82 %) against `level`'s 38/38
    (rung 2) and 32/32 (the 0.50 `level` column flown 11:34); equal at 0.65 (63/64 vs 32/32). The debounced seat bit
    broke on 9/77 pose carries (late-carry flickers, ball still thrown). **Default stays `level`.** The gate is re-flown
    with the bound: one `level` 0.50 column and one `pose` 0.50 column in one Sitting, 30 attempts each (runbook § 9).
  - *Solve time under load (new)* — the pose CATCH install ran 168 / 156 / 236 ms median (0.65 / 0.50 S1 / 0.50 S2) against level's 133 / 207 ms
    (0.65 / 0.50 S2) on the loaded Jetson (idle: 46 vs 42 ms wall at 0.50, +11 ms in the pose QP stage of which ~8–10 ms is ASSEMBLY, the solver +2–6 ms; iterations 45 vs 44);
    two of three pose goals in the second Sitting self-ended `SPLICE_TOO_LATE` on the handoff deadline. The idle budget
    test cannot see this (UH-3's lesson); the loaded ratio is recorded, the deadline itself is the SPLICE_TOO_LATE
    residue, stack-wide, not P3's.
  - *52–58 (the 0.30 / apex 1.0 / 30 A destination)* — **AMENDED to the FIRST PLANNABLE DWELL.** Offline at P4's working
    point 600 / 8000 / 200k (2026-10-24): `level` is still `CUP_SEATING_CONE`-refused at 0.40 (the cone is not a limits
    problem), `LIMIT_ACC` at 0.35 / 0.30; `pose` with the bound clears 0.45 (73.9k), is `HAND_LIMIT_ACC`-refused at 0.40
    (hand acc 3900 rev/s², not a leg knob) and QP-INFEASIBLE at 0.35 / 0.30 (the 0.100 + 0.050 s release holds eat half
    the window). P4's gate and P5's cell move to the shortest dwell the chosen planner clears (0.45 today); the hand-acc
    headroom and the release-hold budget are P4 design items that may re-open 0.40 and 0.30.
  - *New prerequisite* — the single-leg ~10 Hz vibrations (three events on legs 1 and 2, both planners, two with the iq setpoint on the 15 A clamp: S1 leg 2, S2 leg 1) are the
    can-bridge LEAD CLAMP amplifying leg-bus encoder frame drops (`leg_interp.cpp`, no age extrapolation): archived plan
    `leg-bus-frame-drops.md` Workstream A, re-opened as `plans/archived/leg-lead-clamp-age-aware.md`, **a prerequisite of
    the next powered oval Sitting and of P4's `leg_curr_limit_a` 15 → 30.**
    **MET 2026-10-27:** that plan's P3 Sitting flew FW 33 with zero staircase episodes and zero clamp ticks through real
    frame drops (`logbook/2026-10-27-leg-lead-clamp-age-aware-p3-robot.md`); the plan is archived.
  - *P3-5 (0.45 under pose)* — refused by the box because no 0.45 pose cell had been swept (the runbook row assumed one);
    the cell is in the box from this re-sweep. The install log already names the planner per install
    (`install_segment … planner pose`); no observability change.

### 1.4 Relationship to other plans

`two-ball-skill-stack.md` (archived) is the substrate: skills, schedule, executor, learner, box,
sim gate. `cup-contact-contract.md` (archived) holds C-CUP-1/2/3; C-CUP-4 extends it. The
2026-09-16 banking-saturation defect is C-CUP-1's (fixed: `BANKING_SEATING_MIN_G` / `_banking_raw`); the oval's
separate "leak" claim (P2) was a second, since-unreproduced one (retired 2026-10-23, § 4 P2). `skill-stack-dress-rehearsals-and-residue.md`
keeps the columns residue (SPLICE_TOO_LATE, emit gap); the columns canary at P4 reads against it.
`columns-empty-cup-two-feed.md` (active) owns the cup-keyed accept rule every oval goal passes
through; `bb-calibration-one-attempt.md` (archived 2026-10-19) owned the calibration sweep the runbook's row 6a
reads.

**Open question (JB-34 survey, 2026-10-28):** the offline replica reads a cup-seating-cone angle of 16.5-21.7 degrees at the knot after the Catch, while hardware read 31.6-41.7 degrees; the gap is unexplained (`logbook/2026-10-19-cup-seating-cone-refuses-columns-stroke-braking.md`). P4 already moved to `pose` mode partly because `level` stays CUP_SEATING_CONE-refused.

## 2. Architecture

### 2.1 Sites and circulation

`columns_sites(150)` unchanged: P1 = (−75, 0, 830), P2 = (+75, 0, 830). Roles are a property of
the pattern, not the constructor: `OneBallPattern.roles = 'alternate' | 'fixed'` applied at the
two site/target lines of `_append_one_ball_pattern`, so `compile_reload` inherits it. Fixed
roles: every throw's site is T = P1, target C = P2. Ball Butler feeds sites[0] = T, so the
feed's descent from the −x side never crosses a ball over the +x half, and no carry ever starts
from a feed catch.

### 2.2 The oval window

One STEADY window per cycle, release to release: empty transit T → C, CATCH at C, carry C → T,
THROW at T. `ThenThrow` gains `release_site`; `_fold_catch_throw_pairs` folds CATCH(C) → THROW(T)
across sites; `segments._plan_catch_throw` already passes `throw_site_mm` and `catch_site_mm`
separately, so the planner side is unchanged. The executor threads the release site everywhere
it today takes the CATCH skill's site (§ 4 P1 list). The one-ball and two-ball ovals share the
window; only the empty span's length differs (one ball: the whole flight; two: τ = (t_f − d)/2).

### 2.3 Catch and the carry's tilt series

Prescribed tilt series on STEADY: sweep (seam pin → receive pin) / HOLD the receive pin over
`[k_td − h_catch_pre, k_td + h_catch_post]` / sweep (receive → throw, the carry bank) / HOLD the
throw pin over `[n − h_pre, n]`. The lateral channel is free through the catch (the carry's
reversal runs through touch-down). Two realised-plan checks at the catch knot, beside
`CUP_CONTACT_ACC` in `feasibility.py`: `CATCH_CLOSING_SPEED` refuses when the realised axial cup
speed at touch-down is below 20 % of the match target (0.7 × arrival speed along the receive
axis; 50 %, then 25 %, then 20 %, amended 2026-10-18, held-axis catches exempt) — the unrefused −345 mm/s near-hard catch of the capture-span probe (ratio 0.114) is the failure it
closes; `CATCH_SKID` is a logged cross-axis cup velocity at touch-down, no threshold.

### 2.4 C-CUP-4, the seated carry (normative)

On every interior loaded knot (touch-down + 1 … release − 1) of every pattern, the cup's planned
specific force lies inside a cone about the PRESCRIBED cup axis (inscribed 8-gon) AND
`a_z + g ≥ g·f`. **Check thresholds (2026-10-18): 30° / 0.30 g; the shaper TARGET is 20° / 0.5 g** (P3). Shaper: inequality rows in `cup_cycle._assemble` after the C-CUP-2 block (cup
xyz jerk are the variables, hand inside cup z; no column index moves, warm-start key preserved).
Check: the realised-plan residual re-measured beside `feasibility._cup_contact_floor_check` in
`validate_cycle` on the REALISED plan's cup-opening acceleration, codes `CUP_SEATING_CONE` / `CUP_SEATING_FLOOR`.
Columns passes (the 1.3° / 0.02 g margin is the HOP's, 28.7° / 0.320 g; columns' own margin is unmeasured, and the sim-gate columns point at dwell 0.25 or 0.30 is refused while 0.27 passes); 175 mm (21.4°) is no longer refused by the check, only by decision 57.
The check lands in P1 (the as-is bag gets the residual metric; the dispatched rung-1 cell, dwell 0.65, passes
with its 0.100 s pre-release hold, reading 17.8°; 18.68° is the hold-less number, a plan nothing dispatches), the rows in P3.

### 2.5 Pose-level carry planner, behind `level | pose`

Variables: cup xyz jerk + platform z jerk (the free centroid z band); tilt PRESCRIBED (§ 2.3);
objective: cup jerk + leg jerk through one fixed Jacobian, weight `w`; C-CUP-4 rows. It becomes
the steady formulation of `cup_cycle` for every pattern behind a named mode, default `level` (flipped to `pose` 2026-10-30)
until decision 24's gate passes, then flipped with columns re-gated (box re-sweep + sim gate +
one columns check sitting). **Amended 2026-10-23 (P3, measured):** the z band ships at 0 — the
block is not assembled (`pose_z_band_mm`, knob kept for a hardware A/B); with it on, the band
saturated on every cell and spent leg velocity and acceleration for no peak-jerk gain. The
seam-dz ablation is therefore moot and decision 33 is closed null. **Amended 2026-10-24 (P3
tail, measured on the robot):** the touch-down skid is a BOUND, not a logged diagnostic —
`PoseConfig.touchdown_skid_mmps` (175 mm/s) adds inscribed-8-gon rows on the cup's lateral
velocity at the catch time (`I-PLAN-14`); without them the leg-jerk-minimal carry passes
through the catch site at −507 mm/s and drops the inward-landing balls. Applies to the folded
STEADY only (both events, no held axis); LAUNCH / LANDING / SETTLE and the held feed catch are
the level program in both modes, and `CycleMeta.planner_mode` names what was solved.

### 2.6 Admissible box, workspace, dwell

`PATTERNS` gains `oval_1ball` and `oval`; `oval_1ball_fed` / `oval_fed` use the sibling box;
`dwell_s` joins the `select()` key so the ladder's cells (0.50 / 0.40 / 0.30) coexist in one
YAML (the ladder is now 0.65 / 0.50 / 0.40 / 0.30); `dwell_s` is a required `select` keyword and stays a node parameter, refused on mismatch naming the field. One accept-time
refusal (new `REJECTED_` code) and the same check in `tools/admissible_sweep.py`: |site xy| +
box half-extent ≤ `workspace_xy_m` (no margin constant; code `REJECTED_SITE_OUTSIDE_WORKSPACE`, amended 2026-10-18). The sweep records per cell the
seating residual, the closing-speed ratio and the skid. **P3 (2026-10-23): every cell is stamped
`planner_mode`** (the planner it was swept under; `tools/admissible_sweep.py --planner-mode`); `select` /
`describe_miss` require it and refuse a mismatch by name (an unstamped cell matches nothing) — the same
unenforced-twin class the `dwell_s` stamp closed. The oval cells at 0.65 / 0.55 / 0.50 coexist under
`level` and `pose`; flipping the default means re-sweeping every family.

### 2.7 Learner, release and aim

Oval sittings use a SEPARATE `plant_id` (amended 2026-10-18); memory rows keyed by the release site xy (T); the catch offset stays in the
learner state `x[2:4]` (it shapes the carry's entry) but no longer shifts the release or the
flyback. A carried throw releases at T exactly: the planner carries from the actual landing to
T. The aim is computed from the planned release toward the landing target; release-position
error is a diagnostic. The 2026-09-28 rule narrows to "a cup that is not carried does not
translate" (executor docstring + `INVARIANTS.md`). Caveat recorded: two patterns sharing a
release xy share rows.

### 2.8 Entry and exit

One ball (P1): `compile_reload` feeding T → held-axis catch at the live cap → `DECAY_S` rest →
ordinary launch from rest under T → first STEADY catch at C → first carry. Two balls (P5):
`compile_oval` modelled on `compile_columns(entry='hop')` — ball 1 reloaded at T, thrown T → C
with the cup parked, the feed caught PARKED at T (LEVEL pin + axial-only match), self-tossed
after one dwell, cup transits to C, steady from ball 1's catch; carried over unchanged:
announcement-authoritative landing time, THROW 0 at `t_land − τ`, the 2.5 s delay floor,
`columns_feed_bb_bias_mm`, phantom-catch guard, cancel-on-end; dropped: the hop's learner/box
bypass and the toward-A offset. Accept is cup-keyed as for `columns` (SEATED → throw first;
EMPTY → RELOAD_FIRST only once that plan's two-feed chain lands, refused until then). Exit: the
second-to-last catch at C is a standalone HELD catch and the last throw lands on it (P5; P1's Stop does not do this).

## 3. Implementation Phase Summary

| Phase | Lands | Hardware rung that gates the next phase | Status | Date |
|---|---|---|---|---|
| P1 | The oval pattern dispatchable: `oval_1ball(_fed)`, fixed roles, cross-site fold, executor threading, codes, box cell at dwell 0.65, C-CUP-4 check, GUI, sim label, tests | **Rung 1** (`session_oval_rung1.md`): 150 / 0.65 / 0.95 at today's limits, as-is banking — go/no-go on the carry (the 30-consecutive gate WAIVED by owner 2026-10-23: longest oval attempt 10/10; 28/28 caught = 18/18 oval, plus 10/10 hop control; the 30-run bar lives at P5) | COMPLETE | 2026-10-18; rung 1 FLOWN 2026-10-23: 28/28 caught (18/18 oval, plus 10/10 hop control), owner: "Oval 1 ball patterns are working really nicely. I think we're ready to advance on this." |
| P2 | Banking leak fix (principle a) | ~~Rung 1 re-flown = the leak-fixed column~~ | RETIRED 2026-10-23 (row kept): the leak does not reproduce on today's planner, delta 0 at every accepted dwell 0.65-0.45, § 4 P2 | 2026-10-23 |
| Rung 2 | No planner code (box cells at 0.55 / 0.50 swept 2026-10-23, see the logbook): one-ball oval at dwell 0.55 then 0.50 on the as-is `level` planner at today's limits, bundled with the sitting that flies the three owed unflown fixes first | Row-27 metrics per step against the 0.65 baseline (runbook `session_oval_rung1.md` § 7) sets P3's shaper target | FLOWN 2026-10-23, MET (owner GO): 0.50 38/38 with a 32-run, 0.55 4/4, 0.65 75/76; leg 2 peak 10.1 A accepted (15 A clamp) | `2026-10-23-oval-rung2-flown-dwell-050` |
| P3 | Pose-level carry planner behind `level \| pose`, C-CUP-4 rows, knob re-tune, cycle budget, seam-dz ablation (design starts AFTER rung 2 flies) | Starts from the dwell-0.50 as-is baseline (38/38); rung 1 re-flown = the swept column → decision 24 gate; columns re-gated if the mode flips | CODE COMPLETE 2026-10-23; **A/B FLOWN 2026-10-24 (two Sittings): `pose` NO-GO at 0.50 as built (47/57; the logged skid was the drop cause), equal at 0.65; default stays `level` (flipped to `pose` 2026-10-30). Skid BOUND landed the same day (`I-PLAN-14`, 175 mm/s, pose cells re-swept + 0.45 cell); re-gate = the next Sitting (runbook § 9), AFTER the lead-clamp prerequisite plan** | 2026-10-24 |
| P4 | Working point 600/8000/200k, acc ceiling 10000, 30 A (gated by `leg-lead-clamp-age-aware.md`, MET 2026-10-27), box re-sweep (oval + columns), one-ball cells at the FIRST PLANNABLE dwells (0.45 today; 0.40 / 0.30 only if the hand-acc and release-hold design items open them — amendment 52–58, 2026-10-24) | Ramp sitting (five columns canary, one-ball oval at the shortest admissible dwell) | NOT STARTED — premise amended 2026-10-24 | |
| P5 | Two-ball `oval(_fed)`: `compile_oval`, hop-shaped entry, Stop exit, clearance pre-flight | 30 consecutive BB-fed catches in one attempt at P4's gate dwell | NOT STARTED | |

Each rung is the gate (P3's gate is rung 2's outcome): a phase does not start until the rung before it has flown and its
outcome is logged. Rung 1's GO/NO-GO bar is stated under P1 in § 4. "Not with this cup" at rung 1 ends the plan (archive `superseded`, the
backup destination is a fresh effort).

## 4. Implementation Phases (detailed)

### P1 — The oval pattern dispatchable — COMPLETE 2026-10-18

Paths under `ros_ws/src/jugglebot/jugglebot/`.
- **Compiler** (`motion/skills/schedule.py`): `OneBallPattern.roles`; the two site/target lines of
  `_append_one_ball_pattern`; `Schedule.pattern` accepts `'oval_1ball'`; `ThenThrow.release_site`;
  `_fold_catch_throw_pairs` folds across sites when the pattern is fixed-role.
- **Executor** (`motion/skills/executor.py`) — every site that takes the CATCH skill's site as the
  release site: `_catch_terminal` release xy (T, not the caught xy); `_command_u` site argument
  (learner `x[0:2]`, box key `(T, C)`); `_release_offset_m` and its four callers
  (`_offset_flyback_mm`, `_learned_prior_landing`, `_register_outcome`, the `_command_u` call);
  `_previous_release`; `_release_pos_mm`; `_register_outcome`; `_outward_unit`. The 2026-09-28
  docstring narrows per § 2.7.
- **Box** (`motion/skills/admissible.py`, `tools/admissible_sweep.py`): labels, dwell in the key,
  the workspace refusal (no margin), oval sweep function (chain = the folded window), the cell
  `oval_1ball` / (P1, P2) / 150 mm / apex band covering 0.95 / dwell 0.65 / 350-5000-200000 /
  hand 3900, swept twice bit-identical with BLAS at one thread. **Full re-sweep, every cell**
  (single / hop / columns / oval, same commit): `feasibility.py` is in the box gate hash, so "columns cells
  byte-unchanged" was impossible; `tests/motion/test_skills_admissible.py::test_the_oval_resweep_changed_only_the_stamp_and_added_one_oval_cell`
  pins that the pre-existing cells' VALUES equal `tests/fixtures/admissible_box_pre_oval.yaml` (only
  `gate_hash` / `swept_at` may differ).
- **Checks** (`motion/trajectory/feasibility.py`): C-CUP-4 realised-plan check beside
  `_cup_contact_floor_check` with `CUP_SEATING_CONE` / `CUP_SEATING_FLOOR`; `CATCH_CLOSING_SPEED`;
  the `CATCH_SKID` diagnostic line. Thresholds are named constants with their origin in the
  comment (30°, 0.30 g, 20 %; the 20° / 0.5 g shaper target is recorded in the same comment); skid has NO threshold — the 100 mm/s that covered every
  measured cell is a comment-level reference for reading the log line, never a refusal.
- **Node and GUI**: `skill_node._PATTERNS` + `_run_one_ball` dispatch for `oval_1ball(_fed)`
  (fed = `_start_reload(site=sites[0])`, no entry code); rows in the cup-keyed accept table
  `_CUP_RULE` (`columns-empty-cup-two-feed.md` D3, 2026-10-17 — the goal carries no `reload`
  flag, the debounced cup evidence picks the first skill): `oval_1ball` as `hop` (EMPTY →
  RELOAD_FIRST, SEATED → THROW_FIRST), `oval_1ball_fed` as `columns_1ball_fed` (EMPTY →
  RELOAD_FIRST, SEATED → `REJECTED_BALL_IN_HAND`); `skills/check` prints the oval box;
  `ros_ws/gui/js/juggle-panel.js` list entry; `sim/skills_gate.py` label + `_sites_for`.
- **Docs**: `INVARIANTS.md` rows (C-CUP-4, closing speed, skid, translation rule, learner key, Stop decline,
  dwell key); the archived `cup-contact-contract.md` carries a one-line pointer; `docs/motion_planner` has no pattern table.
- **Tests**: new `tests/motion/test_skills_oval.py` (fold + executor); the checks' tests in
  `tests/motion/test_validate_cycle.py` beside `CUP_CONTACT_ACC`; table-shaped tests grow in place.
- **Gate**: (merge done 2026-10-23: `skill-stack` and `two-ball-oval` are both on `main`; the BB-calibration phases, the handoff and
  splice fixes and the calibration ledger the runbook reads are all present); `./run_tests.sh --full` green;
  then rung 1 on the loaded Jetson with every accept-time refusal reported at once (runbook
  rows 16b/16c). **GO** = a 30-cycle attempt completes with the hand ball sensor's seat bit
  continuous through every carry (runbook rows 22a/25); **NO-GO** = the ball leaves the cup
  in the same carry region twice (row 26). **The 30-cycle bar was WAIVED by the owner 2026-10-23** (longest oval attempt 10/10; 28/28 caught across attempts = 18/18 oval, plus 10/10 hop control; the 30-run bar lives at P5). Outcome recorded in the runbook § 6 and § 3 here.
- **Outcome (2026-10-18):** commits 7ae390cb (code, tests, fixtures, box, logbook, INVARIANTS) + the plan/runbook commit that follows it; gate `./run_tests.sh --full` 2026-10-18 parallel 6792 passed / 9 skipped / 1 failed (a GUI replay-overview thread race, untouched code, 3/3 green in isolation) + serial 7 passed (logbook Verification); box re-sweep
  sweep 2026-10-18 `temp/sweeps/sweep_P1.sh` (four invocations: single/hop/columns at dwell 0.27, oval at 0.65 - 150 mm, apex 0.95 +/- 0.025 - all at 350/5000/200000/hand 3900, BLAS one thread), each run twice, `cmp` IDENTICAL for all four; merged single->hop->columns->oval; gate_hash 606b53e3912f (first merge 20436164b087 at the 25 % closing threshold; re-swept after the 20 % re-base with every cell identical, four files cmp IDENTICAL again), swept_at 2026-10-18, 20 cells, merged-file md5 5e2ea1b3ba8a1b75718c78b5d3035096; hop and columns cells value-identical to the pre-oval snapshot, self_toss cells shrunk (see Tradeoffs). The dispatched carry reads 32.1° at dwell 0.50 (pre-release hold), so rung 1 flies
  at 0.65. Rung 1 cleared to fly; P2 starts after its outcome is logged. Open residue: oval Stop exit: REST+CATCH pair at C (decision 51), not landed in P1.

### P2 — Banking leak fix — RETIRED 2026-10-23

Owner decision 2026-10-23 (measure-first review, `logbook/2026-10-23-oval-p2-banking-leak-not-reproduced.md`): the
premise ("40-110k of leg jerk leaked by the saturated ±12° banking demand; clipped to the pin envelope the oval first
plans at 200k") came from the gitignored wayfinder's pre-P1 prototype grid (dwell 0.30-0.50, 12 vs 20° clamps), not
from a dispatched cell. It does not reproduce:
- **Offline** (`python tools/probes/oval_dwell_sweep.py --dwells 0.65,0.60,0.55,0.50,0.45,0.40 --mode both --controls --control-dwells 0.65,0.50,0.27`, 2026-10-23, HEAD 48fd8d5a): a probe-only
  "sweep as the request" patch gives delta 0 at every accepted dwell 0.65 / 0.60 / 0.55 / 0.50 / 0.45 (window jerk 58.7k /
  71.4k / 88.3k / 111.5k / 143.8k mm/s³, realised bank ±2.26° identical with or without the clamped demand). The first
  refusal is `CUP_SEATING_CONE` at dwell 0.40 (49.1° > 45°), P3's shaper job, not jerk.
- **Hardware** (rung-1 bag, `tools/probes/oval_carry_bag_metrics.py`): 15 steady carries at 0.65 median 35.3k commanded
  jerk, max 83.1k; legs 1/2 peak 4.5 / 6.9 A against the 15 A clamp; no carry `LIMIT_*`.
- **Not neutral outside carried folds**: on hop cells the same patch moves jerk by -106k / -76k / +7k (0.65 / 0.50 /
  0.27 dwell), so "feed the sweep as the request" was never safe ungated.
Where the idea lives now: a note inside P3 (below). Code never started; nothing to revert.

### P3 — Pose-level carry planner — COMPLETE 2026-10-30 (GO)

- **Sequencing (owner 2026-10-23):** rung 2 flew (MET, owner GO): the as-is `level` baseline for P3 is dwell 0.50 (38/38 caught, leg 2 peak 10.1 A of 15, `ball_held` unbroken 36/36). P3's shaper target = the cone refusal at dwell 0.40
  (49.1° > 45°); the physical thing to watch is the raw seat-bit dropout rise (`ball_held_raw` dropouts in 56 % of carries at 0.65 to 69 % at 0.50) as the dwell shortens. P3 design starts now. The retired P2 idea (feed the receive-to-throw sweep as the banking request on carried folds only) is a
  candidate sub-item here, to be gated to carried folds and measured against the hop cells before it is adopted.
- `cup_cycle`: the `level | pose` mode (default `level`, flipped to `pose` 2026-10-30); the free centroid z block, analytic
  dz in `decompose`, the prescribed tilt series with its two holds, the leg-jerk objective,
  C-CUP-4 rows after the C-CUP-2 block. Solve time was ~2× today's offline: measure the cycle
  budget (`tests/motion/test_unified_cycle_budget.py`, `test_solver_worker_lateness.py`) and
  keep them green at every cell of the box.
- Knob re-tune, once, across the oval box cells: h_pre, h_catch_pre, h_catch_post (both allowed
  to reach zero — their cost is unmeasured), w; values recorded with the (date, command, result)
  triple; selection-effect caveat carried.
- Seam-dz ablation (decision 58): free dz at the release seam, judged by planned leg jerk + the
  in-dwell leg-velocity reversal diagnostic against the ticket-11 baseline (no planned gain with
  seams at rest). Ships only with the release-state composition; a null result closes decision 33
  in the logbook.
- Gate: rung 1 re-flown as the swept column; **decision 24** — offline: residual ≤ 20° on every
  interior knot at 150 mm AND peak leg jerk ≤ the pinned-z no-lean baseline; hardware: the swept
  variant matches or beats the as-is catch rate over 30 attempts with leg current not higher.
  Pass → mode default flips to `pose` with columns re-gated (box re-sweep, sim gate, one columns
  check sitting). A loss keeps banking an option, not a phase; the ladder continues on `level`.
- **Outcome (2026-10-23):** commits under `git log --grep "Logbook-Entry: 2026-10-23-two-ball-oval-p3-pose-level-carry-planner"` (the entry `logbook/2026-10-23-two-ball-oval-p3-pose-level-carry-planner.md`). The
  pose planner is landed behind `SegmentConfig.planner_mode` (`level` | `pose`, default `level` from
  `trajectory_op.planner_mode`), applied to the folded STEADY only: prescribed tilt (`cup_realize.prescribed_tilt_fn`),
  leg-jerk objective through one Jacobian, C-CUP-4 rows (20° about the PRESCRIBED axis, 0.5 g) after every level column
  (`pose=None` is byte-identical). **Decision 24's offline gate PASSES at every box cell** (production path, 150 mm /
  0.95 / today's limits): residual 12.0 / 18.6 / 18.6° at dwell 0.65 / 0.55 / 0.50 against level's 17.8 / 26.2 / 32.1°,
  window jerk 44.3k at all three against 58.7k / 88.3k / 111.5k, leg acc −42 %; 0.45 becomes admissible (56.5k, 18.6°);
  0.40 is refused on **leg velocity** (357 > 350 mm/s), no longer on the cone — P4's raise. **Measured re-decisions
  (owner, 2026-10-23):** the free centroid-z band SHIPS AT 0 (knob kept): ±30 mm saturated on every cell and cost +57..+62
  mm/s leg velocity and +270..+320 mm/s² leg acc for the same peak jerk, refusing 0.45 on the sweep margin — leg velocity is
  not in the objective, so the channel is spent shaving jerk that is not the peak (decision 33 closes null; the seam-dz
  ablation is moot); the knobs are **h_pre 2, w 10, h_catch_pre 0, h_catch_post 0** (the receive holds cost 10–13 % of
  carry jerk for nothing the ball can feel: the quintic is already at rest at the pin); the touch-down skid the leg-jerk
  objective introduces (225 / 379 / 494 mm/s vs 14 / 12 / 33) is ACCEPTED as the logged diagnostic decision 28b made it,
  the lateral catch weight recorded as the first lever (×100: 10 / 48 / 105 mm/s at +18 % jerk and a hand refusal at
  0.40; a hard match: 55 mm/s at +28 % jerk / +67 % acc). The admissible box is stamped `planner_mode` (unstamped matches
  nothing; `select` refuses a mismatch by name) and carries the three oval cells under BOTH planners: re-swept 2026-10-24 (`temp/sweeps/sweep_P3.sh`, nine families, two rounds byte-identical, BLAS one thread; gate_hash e3eb288fc787, 25 cells, every one of the 22 level cells value-identical to HEAD's box — pinned by `test_p3_resweep_changed_only_the_stamp_and_added_the_pose_cells`). One live
  switch — skill_node's `planner_mode` parameter, carried on every `InstallSegment` request (trajectory_node plans with
  it; its own `SegmentConfig` default serves a request carrying none). Cycle budget: idle Jetson 2026-10-24, the 61-knot oval cell (dwell 0.65), 30 solves: pose plan_cycle total min 50.4 ms / core 43.0 ms against level's 48.4 / 40.4 (the pose solve is 37 vs 25 ms but skips the 10 ms banking schedule), both inside the 50 ms core budget; `test_unified_cycle_budget.py::test_pose_mode_steady_fits_the_same_budget` keeps it there. Gate: `./run_tests.sh --full`, run 2026-10-24 02:40 on the final tree: **parallel 7140 passed / 4 skipped in 328.54 s + serial 8 passed in 29.54 s, PASS** (baseline before the phase 7098 / 7).
  **The default was `level` until the 2026-10-30 re-gate (GO); it is `pose` since. The hardware gate was the A/B sitting in `tests/hardware/session_oval_rung1.md` § 8** (as-is
  column vs the swept column at dwell 0.50, 30 attempts each, leg current not higher; a GO flips the default with every
  family re-swept under `pose` + the sim gate + one columns check sitting). ~~P4 is cleared to start after that sitting's
  outcome is logged; its first item (sweep the pose planner's one-ball cells at 0.40 / 0.30) is unchanged.~~ (struck
  2026-10-24: see the A/B outcome below — P4 is NOT cleared; its premise is amended in § 1.3.)
- **A/B outcome (2026-10-24, two Sittings; entry `logbook/2026-10-24-two-ball-oval-p3-ab-sittings-skid-bound.md`):**
  **NO-GO for `pose` as built at dwell 0.50.** Sitting 1 (10:44, bag `2026-10-24_10-44-21`) flew the as-is column at 0.65
  by mistake (32/32) and `pose` at 0.65 (63/64) and 0.50 (47/57, ten goals each ended by one missed catch); Sitting 2
  (11:34, bag `2026-10-24_11-34-29`) flew the `level` 0.50 column (32/32, legs 1/2 carry peaks 7.32 / 9.87 A apart from
  one lead-clamp event, jerk median 66.5k) and three `pose` 0.50 goals (7/8 then a `SPLICE_TOO_LATE` solver-late end; a
  transit-margin end at the first REST; a feed-site first-throw miss). Combined at 0.50: `level` 32/32, `pose` 54/66 of
  the attempts that reached a catch. Root cause of the pose misses: the touch-down skid decision 28b had logged — the cup
  moving inward at −507 mm/s median when the ball lands, 9 of the 10 pose-0.50 misses a ball > 15 mm inward meeting the rim (9/16 vs 0/7
  under `level`); throw quality identical. Three re-decisions (owner 2026-10-24, § 1.3 Amendments, P3 tail): the skid is
  a bound (`I-PLAN-14`, 175 mm/s, landed this day with the box re-swept and a 0.45 pose cell added — 26 cells), the
  single-leg vibrations seen in both Sittings (leg 1 twice, leg 2 once; the can-bridge lead clamp on stale encoder
  frames, twice with the iq setpoint on the 15 A clamp, once mid-carry) are a PREREQUISITE plan `leg-lead-clamp-age-aware.md` (MET 2026-10-27), and P4/P5 aim at the
  first plannable dwell. P3 stays `in_progress` until the re-gate Sitting (runbook § 9) is flown after the prerequisite.
- **2026-10-28:** re-gate still UNFLOWN; every Sitting 10-24..10-28 ran `level`. It flies the DEFAULT launch (CFS solver worker): `level` column then `pose` column in one launch (runbook § 9; `num_cycles: 50`, gate still ≥ 30 attempts; `ball_held` flicker with the ball thrown is logged, not a fail; `CLAMP_DUTY` warnings recorded, not a stop).
  Prerequisites all met: lead-clamp plan 2026-10-27, JB-76 closed 2026-10-28. Three pinned readings: `level` 0.50 refs 35/35, 35/35, 50/50; leg-current refs 7.49 / 10.05 A unchanged; the 22:26 FIFO read CATCH wall/CPU 1.01.
  History: the JB-95 SCHED_FIFO worker stalled the setpoint stream 60-96 ms per CATCH solve (GIL-coupled preemption, inferred; past the 70 ms firmware cover) and tripped the hand guard on 3 of 5 goals at 22:26, so it is off by default (`solver_rt_priority` 0, checked at P3R-1); JB-98 keeps only the unexplained 21:02 trip as a WATCH item.
  Power-cycle the bridge first. Next = the re-gate Sitting.
- **2026-10-30 re-gate FLOWN, GO (owner; runbook § 9.1, bag `2026-10-30_16-17-11`, entry `logbook/2026-10-30-two-ball-oval-p3-regate-go-and-dwell-headroom.md`):** one CFS launch, `level` 0.50 50/50 then `pose` 0.50 50/50 (+6/6). Against the same-launch `level` column `pose` had legs 1/2 peak 6.59 / 9.45 A vs 7.38 / 9.96 A, `ball_held` unbroken 55/55, touch-down v_x median -73 mm/s (bound 175, 0 carries over; 10-24 read -507), same landing scatter (sd 14-16 mm). `pose` 0.45 caught 40/40, then the goal ended at throw 41 on one 80 ms emitter gap (firmware STOP, hand `MAX_DEVIATION`: the JB-98 reframe, not the planner). Owner: the default flips to `pose`, every family re-swept under `pose`, sim gate, one columns check Sitting (flip, re-sweep and sim gate landed 2026-10-30; the columns check Sitting remains); P3R-4's leg-current reference is the same-launch `level` column; a pre-release-hold rung (P4.0) is scheduled ahead of the P4 working-point commit; the next CPU lever is JB-94.

### P4 — Working point and the dwell ladder — NOT STARTED (premise amended 2026-10-24, 2026-10-30)

- **Premise amended 2026-10-24 (§ 1.3 Amendments, P3 tail):** neither planner reaches dwell 0.30 at 150 mm at this
  working point — `level` is cone-refused at 0.40 at ANY leg limit, `pose` (with the skid bound) clears 0.45, is
  `HAND_LIMIT_ACC`-refused at 0.40 and QP-infeasible at 0.35 / 0.30. P4 therefore ladders to the **first plannable
  dwell** and carries two design items that may open the next rungs: (a) the hand acceleration headroom at 0.40
  (`hand_acc_rps2` 3900 is the refusal, not a leg knob), (b) the release-hold budget (0.100 + 0.050 s of a 0.30 s
  window). P4 presumes the `pose` planner passed its re-gate (runbook § 9); if it did not, the ladder ends at 0.45 on
  `level` and the plan's destination is re-examined.
- **Headroom finding (offline, 2026-10-30; scratch probe, to be promoted to `tools/probes/` before P4):** the dwell limit
  under `pose` is the 0.100 s PRE-RELEASE HOLD, not the legs. At 0.40 the refusal is `HAND_LIMIT_ACC` 4308 vs 3900, flat
  across apex, and it vanishes when the hold shrinks (hold 0.050: 0.40 admissible at today's limits, hand -10 %, leg
  velocity -12 %; hold 0.025: 0.35). The working point 600/8000/200k yields IDENTICAL pose plans (limits only gate), so it is
  margin, not reach. Apex is capped at about 1.05 m by hand acceleration (3489 / 3657 / 3843 / 3902 rev/s² at 0.95 / 1.00 /
  1.05 / 1.06). Two balls in one hand: T_f = dwell + 2τ. A hand estimate put τ_min at about 0.30 s (0.050 post-hold + 0.075 capture + 0.175 transit at 150 mm), giving minimum apex 1.48 m at dwell 0.50, 1.35 at 0.45, 1.23 at 0.40, 1.11 at 0.35, 0.99 at 0.30. The two admissible cells are planner solves, not table entries: the two-ball fold at the P4 working point (600/8000/200k), pre-release hold 0, post-release hold 0.050, τ = (T_flight − dwell)/2. Apex 1.00 / dwell 0.35 solves at τ = 0.277 s (throw hand 3925; needs the hand limit raised from 3900 to the measured authority 3925.5) and apex 1.05 / dwell 0.30 at τ = 0.313 s (hand 3843; fits under the configured 3900). The planner accepting τ = 0.277 makes the hand estimate's 0.175 transit term about 25 ms conservative: at that τ the table's 1.11 m becomes 1.00 m. With the hand limit hypothetically 6000, 1.1 / 0.30 and 1.2 / 0.30–0.40 also solve (throw hand 4119 / 4629, beyond the measured authority), so apex is hand-capped, not planner-capped. Caveat: the two-ball fold was the probe's own construction (no production
  two-ball compiler exists), and it contradicts § 1.2's 10-17 note "0.30 plans at apex 1.0 under P4", which predates the skid
  bound and C-CUP-4; that result is reproduced on today's code first (offline item below). The destination is unchanged
  (owner, 2026-10-30).
- **Prerequisite:** `leg-lead-clamp-age-aware.md` (the frame-drop lead-clamp amplifier) is flown and
  passed before the clamp raise below — a 15 A staircase on one leg today becomes a 30 A one. **MET 2026-10-27** (plan
  archived; `logbook/2026-10-27-leg-lead-clamp-age-aware-p3-robot.md`). JB-76 (`SPLICE_TOO_LATE` ending half the goals)
  still stands in front of any 30-catch run.
- **P4.0 pre-release hold rung (new first item; owner 2026-10-30, map JB-102):** the ideal result is NO hold, the Platform
  moving continuously through the pattern. Design: release from a moving Platform, composing the Platform's velocity into
  the release state / aim and accounting for the 45 + 30 ms tracking delay with measured, not commanded, state (the hold
  exists because of the R4 finding: the Platform still moving +70..+100 mm/s at separation added overshoot). Ladder
  0.100 → 0.075 → 0.050 → 0 at dwell 0.45. Gate: landing scatter not worse than the 0.100 baseline (sd 14-16 mm) and
  catch rate ≥ 30/30 at each rung. Lands ahead of the working-point commit.
  **Mean-bias criterion (added 2026-10-30, C6):** scatter + catch rate cannot see a pure mean shift the learner and catch
  absorb, so each rung (one shared memory; ≥ 20 throws) is also read on the plant residual (y − u) along the throw axis
  (`tools/probes/hold_rung_bias.py --latest`, which attributes goals and holds from the launch log and prints the next hold). Delay effect confirmed = 0.050 lands ≥ 10 mm short of 0.100 and beyond 2 SE of the rung difference, with |dy|, |dapex| < 10 mm (inconclusive → 20 more throws at 0.050, else fall back to 0.075 and stop).
  Branch: predicted hold-0 bias ≤ 35 mm fly 0; 35–55 warm up at 0.025 first; > 55 do not fly 0 (needs delay-aware
  composition). Fallback = the shortest hold with |Δ| ≤ 20 mm, ≥ 30/30, sd ≤ baseline + 3 mm. The Sitting plan is Runbook § 11.
- **2026-10-30 evening:** 10-17 "0.30 at apex 1.0" reproduces ONLY at hold 0 (C1); the hold protects tracking delay, not an un-composed velocity (C2); hold 0 predicted −81..−103 mm short at 0.45 → the two-ball destination needs delay-aware composition = new P4 design item (gated, after the Sitting's δ_eff). Box now carries oval cells at apex 1.00 / 1.05 (54 cells, gate_hash unchanged; entry `logbook/2026-10-30-sitting-prep-columns-check-hold-ladder-and-apex-cells.md`).
- **Offline item:** reproduce the 10-17 "0.30 at apex 1.0 under P4" result on today's code (skid bound, C-CUP-4,
  `pose`) and promote the headroom probe (`tools/probes/oval_dwell_sweep.py` plus the scratch cells script) to
  `tools/probes/`, before the working-point commit.
- First item after P4.0 (the working-point commit follows it): sweep the pose planner's one-ball cells at every dwell the probe clears at the new working point
  (0.50 / 0.45 today; 0.40 if item (a) lands first).
- One commit: `leg_acc_ceiling_mmps2` 5000 → 10000; session defaults 350/5000/200000 →
  600/8000/200000; `leg_curr_limit_a` 15 → 30 with `config/ODrive config Files/` updated;
  `python config/generate_config.py`; box re-swept with oval cells (150 mm at the plannable dwells,
  apex band covering 0.95–1.0) AND the columns cells, twice, bit-identical; relaunch.
- Ramp sitting: five columns attempts at the proven geometry under the new box (guard margin,
  leg current, tracking lag against the 32-consecutive baseline), then the one-ball oval at the
  shortest admissible dwell (peak leg current per transit from the bag is the 30 A measurement).
- Gate: one-ball oval at the shortest admissible dwell and 150 mm sustains a 30-cycle attempt.

### P5 — Two-ball oval — NOT STARTED

- `compile_oval` (modelled on `compile_columns`): fixed per-ball roles, the hop-shaped entry
  (§ 2.8), beat = the steady window, `REJECTED_BEAT_TOO_SHORT` inherited, Stop inherited;
  `oval_fed` dispatch in `skill_node._run_columns`'s shape; box cell `oval` / 150 / apex band
  covering 1.0 / P4's gate dwell (0.30 only if P4's design items open it) at 600/8000/200k; sim-gate `oval` label.
- Pre-flight: re-run the sitting-6 ball–ball clearance script at 150 mm (two balls on one arc
  one beat apart pass at centre-to-centre `sep·(1 + d/t_f)/2`).
- Gate: **30 consecutive BB-fed catches in one attempt** at apex 1.0 / 150 mm / P4's gate dwell.
  Then archive `completed`.

## 5. Testing Plan

- **Unit (per commit)**: fixed roles produce T → C for every throw and inherit into
  `compile_reload`; the cross-site fold yields one window with `release_site` = T and the
  executor's release xy = T (not the caught xy); memory rows key on T; box `select()` refuses a
  dwell mismatch naming `dwell_s` and the workspace refusal fires at |site| + half-extent
  > workspace (equality passes); C-CUP-4 check (re-based 30° / 0.30 g; **amended 2026-10-19: 45°, judged from the hand stroke's end, braking knots exempt from the cone only (the 0.30 g floor keeps its span), 30° the fall-back** — INVARIANTS C-CUP-4): the 175 mm and dwell-0.40 stored windows
  PASS (P3 shaper-target pins; the rung-1 carry at dwell 0.50 now PASSES at 32.1° — `test_rung1_through_the_production_segment_path_passes_the_45_deg_cone` — and the cone refusal is pinned at dwell 0.40, 49.1°, by `test_a_faster_rung1_carry_is_refused_after_the_stroke_ends`), a 10 mm bump refuses `CUP_SEATING_FLOOR`, columns passes; the closing-
  speed check (20 %) refuses the −345 mm/s fixture and passes −1742; P3: the QP rows hold the realised
  residual ≤ 20° at the box cells, `level` mode output is unchanged for columns (bit-exact against
  the pre-P3 box hash), cycle-budget tests green at the slowest cell.
- **Sim gate** (`sim/skills_gate.py`, nightly tier where it runs long): `oval_1ball` and `oval`
  sustain N cycles with the ball held — choreography, fold, timing, accept codes, learner
  convergence. Seating is NOT a sim claim.
- **Box sweeps**: every cell twice, bit-identical, BLAS one thread, same commit as any YAML
  change; `tests/motion/test_skills_admissible.py` keeps the stamp pinned to the launch default.
- **Hardware**: rung 1 twice (as-is / swept), the ramp sitting, the 0.40 and
  0.30 rungs, the two-ball gate. Every sitting: `./run_tests.sh --full` first, dress rehearsal of
  the accept path with all refusals reported at once, bag + launch log by path, logbook entry.

## 6. Notes for Collaborators

- **Safety**: the Teensy `MAX_DEVIATION` guard is the leg-path authority; nothing here changes
  it. The carry is the first LOADED translation — the first rung's stop rules (ball leaves the cup
  twice in one region, FET > 60 °C, a leg on the current clamp > 50 ms, lane recovery failed) are
  in the runbook. 30 A is the owner's number (~30 A judged safe 2026-10-12); the 0.40 rung's bag
  is the measurement. The hand cap 3900 is NOT raisable.
- **Non-obvious choices**: the first rung flies at dwell 0.65 (pre-release hold; the cone moved to 45° since b28b129f so 0.50 passes the planner and cone offline (32.1° < 45°) but is NOT dispatchable until the oval box cells at 0.55 and 0.50 are swept; rung 2 flew it 2026-10-23 once the cells were swept); symmetric sites (every one-sided probe placement produced edge
  artefacts); the catch stays level because the capture span is infeasible at two balls (probe,
  two runs); the mode switch exists so decision 24's as-is column stays producible; dwell is a
  parameter, not a goal field, so a box swept at one dwell refuses the other by name.
- **Row-27 baseline exists (2026-10-23)**: the rung-1 bag is read into the runbook's § 6 as-is column (commanded jerk, bank,
  leg 1/2 current, seat-bit continuity, landing, LIMIT tally). Re-run per rung: `tools/probes/oval_carry_bag_metrics.py` (bag) and
  `tools/probes/oval_dwell_sweep.py` (offline dwell ladder); rung 2 reads against the 0.65 numbers. Flown 2026-10-23 ahead of rung 2: the three then-unflown fixes
  (I-PLAN-11 rest_slew daa495e0, columns parity 21e02cfa, feed-line latch 3fe2cdee). No planner code landed in
  `cup_realize` / `cup_cycle` before that sitting (the I-PLAN-12 REST-rebase fix landed the same evening, after it, unflown). Oval SPLICE_TOO_LATE (11/16 re-aims refused) stays residue (archived residue plan § 2).
- **Rung 2 residue (2026-10-23, owner decisions)**: (a) the four two-ball `columns` MISSED_CATCH ends in the 15:53 launch were REAL misses, lateral launch
  outliers (77 / 39 / 29 / 191 mm/s) beyond the clamped aim authority (10 out / 35 in / 20 y mm), inside the sitting-8 streak range, no regression from the three fixes; OPEN,
  unowned, confirming replay = `sim/skills_gate.py` two-ball columns with a refused re-aim before a Ball 2 release, parent of 21e02cfa vs HEAD. (b) Leg/hand heartbeat dropouts (21 in launch B) = the known leg-bus frame-loss
  class, 2.4/min of trajectory like rung 1: WATCH, escalate past an episode near 2.0 s, a `MAX_DEVIATION` or miss coincident with a lead-clamp mask, or > 4/min. (c) `SPLICE_TOO_LATE` (175 mentions in launch B) not tallied per re-aim. (d) Three 0.65 echo-jerk spikes (417-496k) undiagnosed, probe-artefact suspected. See the sitting entry.
- **P3 landed 2026-10-23 (code complete, default `level`)**: the pose planner's defaults are config keys (`trajectory_op.pose_*`,
  each with its measurement in the YAML comment); the live switch is `ros2 param set /skill_node planner_mode pose` (applies
  from the next goal; the install log and the `InstallSegment` response name the mode solved). The production ladder in
  either mode is `tools/probes/oval_dwell_sweep.py --mode asis --planner-mode {level,pose}`. The window-jerk floor the pose
  arm reads at 0.65–0.50 (44,329 mm/s³) is the SETTLE tail's (leg 5, 150 ms after release), identical in both modes —
  the carry's own jerk is below it. Watch item for the A/B: the raw seat-bit dropout rate (skid is the first suspect).
- **Rollback**: `level` is the default mode until the gate (flipped to `pose` 2026-10-30); P1 is additive on the one-ball
  path (`roles='alternate'` is today's hop); P4's working point reverts by one YAML commit + box
  re-sweep.
- **Files**: see `related_code` above; new files are `tests/hardware/session_oval_rung1.md`
  (P1) and the P4/P5 runbooks written at their phases.

## 7. Vocabulary

- *Oval* — two balls in one hand, every throw T → C; *one-ball oval* — the hop with fixed roles (first
  rung). *T* / *C* — throw site (P1, near BB) / catch site (P2). *Carry* — loaded cup transit C → T in the
  dwell; *empty transit* τ — T → C. *Sweep* — the carry bank between the event tilts; *banking leak* — leg
  jerk spent on a bank request that delivers none (P2).
- *Residual* — angle between planned specific force and prescribed cup axis; *C-CUP-4* — check 30° (45 deg from the stroke end since 2026-10-19; 30 deg the fall-back) /
  `a_z + g ≥ 0.30 g` (P3 target 20° / 0.5 g). *Skid* — lateral cup velocity at touch-down (logged).
  *Closing speed* — axial cup speed at touch-down vs the match target (refused below 20 %).
- *Pose-level carry planner* — the `pose` mode of `cup_cycle` (`planner_mode`, P3): prescribed tilt series,
  leg-jerk objective, C-CUP-4 rows; *seam dz* — platform z velocity across a release seam (decision 33, closed
  null at P3). *Planner stamp* — the `planner_mode` a box cell was swept under. *Parked feed catch* — feed
  caught with the cup already at the site. *Ramp* — the P4 sitting that first flies the new working point.
