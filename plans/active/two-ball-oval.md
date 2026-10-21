---
title: Two-ball oval — two balls in one hand, every throw carried back in the cup
created: 2026-10-17
status: active
owner: Harrison
last_updated: 2026-10-18
related_logbook:
  - 2026-10-17-two-ball-oval-plan-written.md
  - 2026-10-18-two-ball-oval-p1-oval-pattern-dispatchable.md
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
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_realize.py (banking leak, P2)
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/feasibility.py (CUP_SEATING_CONE / CUP_SEATING_FLOOR / CATCH_CLOSING_SPEED, CATCH_SKID diagnostic)
  - ros_ws/src/jugglebot/jugglebot/skill_node.py, sim/skills_gate.py, tools/admissible_sweep.py
---

# Two-ball oval

Branch `two-ball-oval`, worktree `~/Desktop/Jugglebot-oval` (from `skill-stack` @ 6e4a0bab).
Design record: the wayfinder map at `.scratch/two-ball-oval/` in that worktree (gitignored; the
58 decisions it holds are restated here in § 1.3 so the plan stands alone). Runsheet for the first
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
- **Catch (28b, 40, 48)** (48 amended 2026-10-18)**:** the level program with a receive-pin TILT hold
  `[k_td − h_catch_pre, k_td + h_catch_post]`, no lateral slaving; throw pin held `[n−2, n]`;
  skid LOGGED only; closing speed REFUSES below 20 % of the 0.7× match target; the BB feed
  catch keeps the program each rung already flies (reload held-axis at one ball, columns LEVEL
  pin + axial-only match at two balls).
- **Contract (29–31, 35)** (29–31 amended 2026-10-18)**:** C-CUP-4 — 20° axis-relative cone + `a_z + g ≥ 0.5 g` on every
  interior loaded knot of every pattern; check in P1, QP rows in P3; codes `CUP_SEATING_CONE`
  / `CUP_SEATING_FLOOR`; `INVARIANTS.md` entry.
- **Planner (23, 24, 32, 36, 58):** the oval pattern is P1 because nothing live dispatches a
  fixed-role oval; the pose-level carry planner is the steady formulation for EVERY pattern
  behind a `level | pose` mode, default `level` until decision 24's hardware gate; named knobs
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
    20° / 0.5 g the check refused flown patterns (250 mm hop with landing offset 28.7° / 0.320 g; reload
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

### 1.4 Relationship to other plans

`two-ball-skill-stack.md` (archived) is the substrate: skills, schedule, executor, learner, box,
sim gate. `cup-contact-contract.md` (archived) holds C-CUP-1/2/3; C-CUP-4 extends it. The open
2026-09-16 banking-saturation defect is P2 here. `skill-stack-dress-rehearsals-and-residue.md`
keeps the columns residue (SPLICE_TOO_LATE, emit gap); the columns canary at P4 reads against it.
`columns-empty-cup-two-feed.md` (active) owns the cup-keyed accept rule every oval goal passes
through; `bb-calibration-one-attempt.md` (archived 2026-10-19) owned the calibration sweep the runsheet's row 6a
reads.

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
the steady formulation of `cup_cycle` for every pattern behind a named mode, default `level`
until decision 24's gate passes, then flipped with columns re-gated (box re-sweep + sim gate +
one columns check sitting). The z band never binds at two-ball (centroid path a tilted line)
and stays in by owner decision. Seam-dz ablation (§ 4 P3): free the platform's z velocity across
the release seam within 350 mm/s; ships only with ballistics + learner prior that include the
platform's release velocity.

### 2.6 Admissible box, workspace, dwell

`PATTERNS` gains `oval_1ball` and `oval`; `oval_1ball_fed` / `oval_fed` use the sibling box;
`dwell_s` joins the `select()` key so the ladder's cells (0.50 / 0.40 / 0.30) coexist in one
YAML (the ladder is now 0.65 / 0.50 / 0.40 / 0.30); `dwell_s` is a required `select` keyword and stays a node parameter, refused on mismatch naming the field. One accept-time
refusal (new `REJECTED_` code) and the same check in `tools/admissible_sweep.py`: |site xy| +
box half-extent ≤ `workspace_xy_m` (no margin constant; code `REJECTED_SITE_OUTSIDE_WORKSPACE`, amended 2026-10-18). The sweep records per cell the
seating residual, the closing-speed ratio and the skid.

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
| P1 | The oval pattern dispatchable: `oval_1ball(_fed)`, fixed roles, cross-site fold, executor threading, codes, box cell at dwell 0.65, C-CUP-4 check, GUI, sim label, tests | **Rung 1** (`session_oval_rung1.md`): 150 / 0.65 / 0.95 at today's limits, as-is banking — go/no-go on the carry | COMPLETE | 2026-10-18 |
| P2 | Banking leak fix (principle a) | Rung 1 re-flown = the leak-fixed column | NOT STARTED | |
| P3 | Pose-level carry planner behind `level \| pose`, C-CUP-4 rows, knob re-tune, cycle budget, seam-dz ablation | Rung 1 re-flown = the swept column → decision 24 gate; columns re-gated if the mode flips | NOT STARTED | |
| P4 | Working point 600/8000/200k, acc ceiling 10000, 30 A, box re-sweep (oval + columns), one-ball cells at 0.40 / 0.30 | Ramp sitting (five columns canary, one-ball oval at 0.40), then one-ball at 0.30 | NOT STARTED | |
| P5 | Two-ball `oval(_fed)`: `compile_oval`, hop-shaped entry, Stop exit, clearance pre-flight | 30 consecutive BB-fed catches in one attempt | NOT STARTED | |

Each rung is the gate: a phase does not start until the rung before it has flown and its
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
  `_CUP_RULE` (`columns-empty-cup-two-feed.md` D3, 2026-10-17 — the goal carries no reload
  flag, the debounced cup evidence picks the first skill): `oval_1ball` as `hop` (EMPTY →
  RELOAD_FIRST, SEATED → THROW_FIRST), `oval_1ball_fed` as `columns_1ball_fed` (EMPTY →
  RELOAD_FIRST, SEATED → `REJECTED_BALL_IN_HAND`); `skills/check` prints the oval box;
  `ros_ws/gui/js/juggle-panel.js` list entry; `sim/skills_gate.py` label + `_sites_for`.
- **Docs**: `INVARIANTS.md` rows (C-CUP-4, closing speed, skid, translation rule, learner key, Stop decline,
  dwell key); the archived `cup-contact-contract.md` carries a one-line pointer; `docs/motion_planner` has no pattern table.
- **Tests**: new `tests/motion/test_skills_oval.py` (fold + executor); the checks' tests in
  `tests/motion/test_validate_cycle.py` beside `CUP_CONTACT_ACC`; table-shaped tests grow in place.
- **Gate**: merge `skill-stack` into `two-ball-oval` first (the live stack flies from
  `skill-stack`, 35 commits ahead at 2026-10-17: the BB-calibration phases, the handoff and
  splice fixes, the calibration ledger the runsheet reads); `./run_tests.sh --full` green;
  then rung 1 on the loaded Jetson with every accept-time refusal reported at once (runsheet
  rows 16b/16c). **GO** = a 30-cycle attempt completes with the hand ball sensor's seat bit
  continuous through every carry (runsheet rows 22a/25); **NO-GO** = the ball leaves the cup
  in the same carry region twice (row 26). Outcome recorded in the runsheet § 6 and § 3 here.
- **Outcome (2026-10-18):** commits 7ae390cb (code, tests, fixtures, box, logbook, INVARIANTS) + the plan/runsheet commit that follows it; gate `./run_tests.sh --full` 2026-10-18 parallel 6792 passed / 9 skipped / 1 failed (a GUI replay-overview thread race, untouched code, 3/3 green in isolation) + serial 7 passed (logbook Verification); box re-sweep
  sweep 2026-10-18 `temp/sweeps/sweep_P1.sh` (four invocations: single/hop/columns at dwell 0.27, oval at 0.65 - 150 mm, apex 0.95 +/- 0.025 - all at 350/5000/200000/hand 3900, BLAS one thread), each run twice, `cmp` IDENTICAL for all four; merged single->hop->columns->oval; gate_hash 606b53e3912f (first merge 20436164b087 at the 25 % closing threshold; re-swept after the 20 % re-base with every cell identical, four files cmp IDENTICAL again), swept_at 2026-10-18, 20 cells, merged-file md5 5e2ea1b3ba8a1b75718c78b5d3035096; hop and columns cells value-identical to the pre-oval snapshot, self_toss cells shrunk (see Tradeoffs). The dispatched carry reads 32.1° at dwell 0.50 (pre-release hold), so rung 1 flies
  at 0.65. Rung 1 cleared to fly; P2 starts after its outcome is logged. Open residue: oval Stop exit: REST+CATCH pair at C (decision 51), not landed in P1.

### P2 — Banking leak fix — NOT STARTED

`motion/trajectory/cup_realize.py`: on a loaded span the banking request IS the event-tilt
sweep (feed the sweep itself as the request — zeroing it pulls the smoother toward level and the
seam re-gate refuses); C-CUP-1's per-instant rule applies only to the free cup. Evidence the fix
targets: 40–110k of leg jerk leaked by the saturated ±12° demand on unequal anchor gaps; clipped
to the pin envelope the oval first plans at 200k. Test: the loaded-span request equals the sweep
on a fixture with unequal anchor gaps, and the planned leg jerk at the first-rung cell does not
rise. Gate: rung 1 re-flown as the leak-fixed column (same runsheet, column 2 of § 6).

### P3 — Pose-level carry planner — NOT STARTED

- `cup_cycle`: the `level | pose` mode (default `level`); the free centroid z block, analytic
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

### P4 — Working point and the dwell ladder — NOT STARTED

- First item: sweep the pose planner's one-ball cells at 0.40 and 0.30 (unswept).
- One commit: `leg_acc_ceiling_mmps2` 5000 → 10000; session defaults 350/5000/200000 →
  600/8000/200000; `leg_curr_limit_a` 15 → 30 with `config/ODrive config Files/` updated;
  `python config/generate_config.py`; box re-swept with oval cells (150 mm at 0.50 / 0.40 / 0.30,
  apex band covering 0.95–1.0) AND the columns cells, twice, bit-identical; relaunch.
- Ramp sitting: five columns attempts at the proven geometry under the new box (guard margin,
  leg current, tracking lag against the 32-consecutive baseline), then the one-ball oval at
  0.40 (peak leg current per transit from the bag is the 30 A measurement), then 0.30.
- Gate: one-ball oval at dwell 0.30 and 150 mm sustains a 30-cycle attempt.

### P5 — Two-ball oval — NOT STARTED

- `compile_oval` (modelled on `compile_columns`): fixed per-ball roles, the hop-shaped entry
  (§ 2.8), beat = the steady window, `REJECTED_BEAT_TOO_SHORT` inherited, Stop inherited;
  `oval_fed` dispatch in `skill_node._run_columns`'s shape; box cell `oval` / 150 / apex band
  covering 1.0 / dwell 0.30 at 600/8000/200k; sim-gate `oval` label.
- Pre-flight: re-run the sitting-6 ball–ball clearance script at 150 mm (two balls on one arc
  one beat apart pass at centre-to-centre `sep·(1 + d/t_f)/2`).
- Gate: **30 consecutive BB-fed catches in one attempt** at apex 1.0 / 150 mm / dwell 0.30.
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
- **Hardware**: rung 1 three times (as-is / leak-fixed / swept), the ramp sitting, the 0.40 and
  0.30 rungs, the two-ball gate. Every sitting: `./run_tests.sh --full` first, dress rehearsal of
  the accept path with all refusals reported at once, bag + launch log by path, logbook entry.

## 6. Notes for Collaborators

- **Safety**: the Teensy `MAX_DEVIATION` guard is the leg-path authority; nothing here changes
  it. The carry is the first LOADED translation — the first rung's stop rules (ball leaves the cup
  twice in one region, FET > 60 °C, a leg on the current clamp > 50 ms, lane recovery failed) are
  in the runsheet. 30 A is the owner's number (~30 A judged safe 2026-10-12); the 0.40 rung's bag
  is the measurement. The hand cap 3900 is NOT raisable.
- **Non-obvious choices**: the first rung flies at dwell 0.65 (pre-release hold; P3 earns 0.50 back); symmetric sites (every one-sided probe placement produced edge
  artefacts); the catch stays level because the capture span is infeasible at two balls (probe,
  two runs); the mode switch exists so decision 24's as-is column stays producible; dwell is a
  parameter, not a goal field, so a box swept at one dwell refuses the other by name.
- **Rollback**: `level` is the default mode until the gate; P1–P2 are additive on the one-ball
  path (`roles='alternate'` is today's hop); P4's working point reverts by one YAML commit + box
  re-sweep.
- **Files**: see `related_code` above; new files are `tests/hardware/session_oval_rung1.md`
  (P1) and the P4/P5 runsheets written at their phases.

## 7. Vocabulary

- *Oval* — two balls in one hand, every throw T → C; *one-ball oval* — the hop with fixed roles (first
  rung). *T* / *C* — throw site (P1, near BB) / catch site (P2). *Carry* — loaded cup transit C → T in the
  dwell; *empty transit* τ — T → C. *Sweep* — the carry bank between the event tilts; *banking leak* — leg
  jerk spent on a bank request that delivers none (P2).
- *Residual* — angle between planned specific force and prescribed cup axis; *C-CUP-4* — check 30° (45 deg from the stroke end since 2026-10-19; 30 deg the fall-back) /
  `a_z + g ≥ 0.30 g` (P3 target 20° / 0.5 g). *Skid* — lateral cup velocity at touch-down (logged).
  *Closing speed* — axial cup speed at touch-down vs the match target (refused below 20 %).
- *Pose-level carry planner* — the `pose` mode of `cup_cycle`; *seam dz* — platform z velocity across a
  release seam (P3). *Parked feed catch* — feed caught with the cup already at the site. *Ramp* — the P4
  sitting that first flies the new working point.
