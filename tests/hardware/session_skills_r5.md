# R5 hardware runsheet — the 4° cup, the fused reload, and the human-lobbed columns start

Skill-stack R5 (`plans/active/two-ball-skill-stack.md` § R5, re-scoped 2026-09-30). Same
machine as `session_skills_r4.md` — **that sheet stays the reference for everything this one
does not restate** (QTM preconditions, bring-up rows, the guard/`/recover`/hand-homing recovery
flow, the R4 watch items carried below). Driver: `jugglebot/juggle` action, patterns
`self_toss` / `hop` / `columns`; `skills/check`; `jugglebot/juggle_stop`; the GUI reload button.
`session_skills_r4_diag.md` is the model for a data-gathering (no pre-registered pass/fail gate)
sitting shape — this sheet borrows that shape for §§ 3.5–6.

**What this sitting is, in seven lines.**
1. Four blocks, increasing risk: a **leg-jerk ramp measurement** (300/5000/**200 000** mm/s³,
   owner decision 2026-09-30 — read below); **Block A**, tightening the reload's receive tilt
   from R4's 12° ceiling to a 4° cap (does the cup still seat a ball arriving ~8° off the cup
   axis instead of the ~12°-hold's near-0°?); **Block B**, the fused catch→re-level→throw
   reload window (F-b's work: one fewer stop, the throw ~0.2 s sooner); **Block C**, the interim
   two-ball columns start — the operator seats ball A, a human LOBS ball B near P2, the platform
   catches it, re-aims, and the schedule takes over.
2. `pattern: columns` is now flyable end-to-end: `reload` is no longer refused for it (S-2's D3
   lift), and it takes EITHER a Ball-Butler reload (aimed at P2, the generalised `_start_reload`)
   OR — new this rung — a human-thrown, tracker-resolved feed with no BB round trip at all (D1's
   interim start, since BB cannot yet aim at two live sites in one round trip).
3. **Gate: none pre-registered for the hardware sitting.** D1 says the sim proves the fused BB
   start before hardware; this sitting gathers the numbers §7's verdict table names, not a
   pass/fail line. The one thing that DOES gate is the ramp block's stop rule (§ 2.5) — it
   decides which session limits the rest of the sitting flies at.
4. **Session limits this sitting: legs 300 / 5000 / 200 000 mm/s³ (owner decision 2026-09-30 —
   the leg jerk ceiling in `hardware_config.yaml`, ramped up from R4's 150 000 launch default),
   hand 3500 rev/s²** (unchanged) — conditional on the ramp block (§ 2.5) not stopping it; a
   stop reverts the WHOLE sitting to 150 000 and the 150 000-stamped box. D1's own 1.0 m /
   hand-3900 fused-BB-start scenario is NOT flown this sitting (still analytical, brief_common's
   "measured on the real planner" table) — out of scope for Blocks A–C.
5. New end codes and refusals since R4: `ABORTED_NO_COLUMNS_FEED` (the human lob never arrived
   near P2 within the deadline), `DROPPED_SURVIVOR_STOPPED` (D3, a drop with the other ball
   still in flight), the D2 shadow-landing end line (the last ball rests on the platform, held
   — not resumable). `LimitsMismatch` (`admissible.py::check_limits`) is new to watch for: it
   names the exact field and value that disagrees, e.g. `leg_jerk_mmps3=150000.0` vs a live
   `200000.0` — this is what fires if `set_limits` (§ 2) is skipped or run AFTER `skills/check`.
6. **`set_limits` MUST precede `skills/check` and every dress-rehearsal row** — the re-swept box
   is stamped for 200 000, and `check_limits` refuses by name (verbatim message below, § 7) on
   any mismatch, ratio or ordering. This reorders nothing from R4's own row order (R4's
   `set_limits` was already row 11, before row 17's `skills/check`) — just changes the numbers
   and makes the ordering dependency explicit for R5's ramp block.
7. Rung order: dress rehearsal → **ramp measurement** → Block A (4°) → Block B (fused reload,
   12°) → Block C (columns from a lob) → close-out. Each block is a smaller, better-understood
   bet than the one after it — same discipline as R4 sheet row 6.

---

## 0. Why this sitting, and what you can push back on

**If your physical intuition disagrees with the framing below — the tilt cap, the ramp, the lob
bound, anything else — that is load-bearing signal, say so before we start.**

Two owner decisions this sitting tests, in the owner's own words:

**The 4° cup test** (owner, 2026-09-30, handoff_S / brief_R): tighten the reload's receive-tilt
cap from R4's flown 12° ceiling down to 4°, and watch whether the cup still seats a ball
arriving ~8° off the cup axis — rather than the ~0° a 12° hold nearly achieves — cleanly, with
no rebound. Block A (§ 4) runs 4°, then re-flies 8° and 12° as controls on the SAME sitting so
the comparison is same-day, same-geometry.

**D1 (columns start)**, verbatim from `plans/active/two-ball-skill-stack.md` § R5: *"Start =
Ball-Butler-initiated from BB's CURRENT position, as a re-attitude programme inside R5: fuse the
held-axis catch, the re-orientation and the throw into ONE QP window (and, if that is not
enough, the post-release transit + tilt approach into it); ramp the session limits toward the
ceilings with logged measurements; throw the first ball at 1.0 m (hand cap 3500 → 3900). The sim
proves the start before hardware. Pre-registered criterion: if the fused start's measured floor
exceeds ball 1's flight at every ceiling, the decision re-opens on BB's placement."* This sitting
flies the INTERIM start D1 also names (a human lob resolved by the tracker, no BB round trip) —
Block C (§ 6) — and the leg-jerk half of "ramp the session limits toward the ceilings" (§ 2.5);
it does NOT fly the fused BB-current-position start itself, which stays analytical
(brief_common's floor table) until its own sitting.

Three places especially worth a sanity check from the person standing next to the robot:

- **The 4° hold is a real, much shallower platform attitude than anything flown so far** — R4's
  12° hold visibly tilted the platform to meet the ball; 4° is close to level. Watch whether the
  ball's approach geometry (its velocity relative to the cup axis) still looks like a controlled
  catch, or like the cup catching a ball arriving mostly sideways to it.
- **The leg-jerk ramp is a genuine step up in what the legs are asked to do** (150 000 →
  200 000 mm/s³, the hard `set_limits` ceiling) — this is new territory for a THROWING pattern
  this rung (R2/R3/R4 all flew 150 000). Keep a hand on the E-stop through the ramp block
  specifically, same as any first-time-at-a-new-limit rung.
- **The columns human lob asks an operator to hit a moving target with a thrown ball** — ±20 mm
  at P2, timed to land inside a ~30 s window the console names as armed. Expect several misses
  before a first accepted feed; a miss is data (the deficit/off-bound message), not a failure.

## 1. Before the robot is powered (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at or after the R5 commits (schedule/executor/segments/skill_node/cup_cycle/cup_realize/unified_cycle changes named in this rung's handoffs). |
| 2 | (ROS) `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash && cd ..` | Builds clean. Owed since `2bb2c030` — the skill node, the executor and the console all changed this rung; nothing new in the `.srv`/`.action` interfaces this rung unless a later handoff says otherwise (D2/D3/D1 all landed as plain Python + parameter changes, not new interface fields). |
| 3 | (venv) `PYTHONPATH=ros_ws/src/jugglebot python -c "from jugglebot.motion.skills import admissible as ab; print(ab.gate_hash())"` then `grep -m1 gate_hash config/generated/admissible_box.yaml` | The two hashes EQUAL. F-b's `cup_cycle.py`/`cup_realize.py`/`segments.py` edits are inside the gate hash's inputs — if they landed after the last sweep, this WILL differ; do not fly on a stale box. `python tools/admissible_sweep.py --site-pairs all --single-apex 0.5 0.6 0.7 0.8 0.9` (~35 min) before flying if it doesn't match. |
| 3a | (venv) `grep -m1 "leg_jerk_mmps3" config/generated/admissible_box.yaml` | Reads `200000.0` (or whatever value the box was actually swept at — the sitting's `set_limits` in § 2 must match this exactly, or `skills/check` and every goal refuse `LimitsMismatch`, § 7). If the box is still stamped `150000.0`, the ramp (§ 2.5) cannot run this sitting — fly Blocks A–C at 150 000 instead and flag the mismatch, don't hand-edit the box. |
| 3b | (venv) `grep -c "pattern: hop" config/generated/admissible_box.yaml` ; `grep -c "pattern: columns" config/generated/admissible_box.yaml` | Hop ≥ 2 (`(P1,P2)`/`(P2,P1)`); columns ≥ 2 (`(P1,P1)`/`(P2,P2)`). A count alone cannot tell a populated band from R4's empty `.nan` placeholders (R4's box carried two of those), so ALSO run `grep -A6 "pattern: columns" config/generated/admissible_box.yaml \| grep -c nan` and expect the `nan` count to be confined to the 0.95 m bands (row 3's `gate_hash` match is what guarantees the sweep is fresh; this row only confirms the 0.90 m band is real before Block C). |
| 4 | `cat temp/reports/nightly/status` (or the nightly ticker per CLAUDE.md) | Fresh GREEN, or a RED read against `git status`/`git stash list` before trusting it. |
| 5 | (venv) `./run_tests.sh --full 2>&1 | tee temp/logs/presitting_full_r5_$(date +%Y%m%d).log` — launch DOWN | `RESULT: PASS`. Record the pass count + wall time in § 11 (the (date, command, result) triple). Not run by this unit — the main session runs the gate. |
| 6 | Home the hand (GUI Home, or `/recover` if it is not already parked) **before** Activate | The 2026-09-23 upper-stop push is still unresolved and on watch (R4 sheet § 1 row 6) — a freshly homed hand removes the one lever that trips it. |
| 7 | Base level (R4 sheet § 1 row 7's two options, unchanged) | Either is flyable; a non-zero frame-check offset if not re-shimmed is the known lever arm, not a new finding. |
| 8 | QTM: `Catching Cone` rigid body DISABLED, Ball Butler reflectors MASKED; `Platform` rigid body defined and tracked | Hard preconditions, unchanged from R3/R4. |

## 2. Bring-up (launch UP, robot powered, ball present)

Run R4-sheet rows 8b–12 with these numbers:

| # | Step | Expect |
|---|---|---|
| 8b | `level` FIRST, before any skill dispatch | `level` completes; `trajectory/status.gravity_correction_loaded` reads fresh. |
| 9 | `ros2 action list \| grep jugglebot/juggle` ; `ros2 service list \| grep -E 'skills/check\|jugglebot/juggle_stop'` | All present. |
| 10 | `ros2 param set /skill_node apex_m 0.9` ; `dwell_s 0.30` ; `plant_id r5-$(date +%Y%m%d)` — a FRESH `plant_id` (cold learner memory, same contract as R3/R4) | Set. |
| 10a | `ros2 param set /skill_node catch_resend_max 0` **for the columns block specifically** (§ 6) — leave it at the operator's own live default for Blocks A/B/the ramp (S's handoff did NOT hard-code 0; it reads `self._catch_resend_max()`, the existing operator-set parameter, for every pattern including columns) | `Set parameter successful`. Revert to whatever Blocks A/B used before § 6. |
| 11 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 300.0, leg_acc_limit_mmps2: 5000.0, leg_jerk_limit_mmps3: 200000.0}"` — **200 000, not R4's 150 000** (owner decision 2026-09-30, § 0); this row MUST run before row 17 / any `skills/check` or dress-rehearsal call (§ 0 point 6) | `applied_*` echoes `300 / 5000 / 200000`. If the box is still stamped 150 000 (row 3a came back 150000.0), set `200000.0` here anyway is a `LimitsMismatch` waiting to happen — set `150000.0` instead and skip the ramp block. |
| 12 | Seat a ball in the hand; confirm on `/hand_telemetry` (`ball_held_valid: true`, `ball_held_raw: true`) | Visual + topic check. |

## 3. The dress rehearsal (every refusal reported at once)

| # | Step | Expect |
|---|---|---|
| 13 | (venv, no ROS) `python3 tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss --arm A --attempts 3` | Three clean attempts, memory rows 1→2→3, verdict G1/G2/G4 PASS. Compare against R4's numbers (33 ms G1, 26–33 ms G2) — this bench does not see the live 200 000 limit (it is a local compiler rehearsal), so a regression here is schedule/executor-side, not limits-side. |
| 14 | (system python3, ROS sourced, robot ACTIVATE only — **never ARM**) `python3 tests/hardware/skills_plan_bench.py --via-action --pattern self-toss --n-throws 3` | `outcome=COMPLETED`, 3/3 dispatched, DISARMED. |
| 15 | Same, `--pattern hop --separation-mm 250 --n-throws 3` (still DISARMED) | `outcome=COMPLETED`. Read every `per_throw` line and any REJECTED/ABORTED message together. |
| 16 | Same, `--pattern columns --separation-mm 100 --n-throws 3 --reload` — DISARMED (ACTIVATE only) | Rehearses the BB reload path through `skills/check` and the action's accept ladder (box lookup for BOTH `(P1,P1)`/`(P2,P2)`, opening REST sizing) without moving anything. A goal ACCEPTED that then times out on BB (no real BB round trip completing while disarmed, expected) reads `ABORTED_BB_THROW_TIMEOUT` or similar — that is this row proving the ladder, not a finding. |
| 16a | Same, `--pattern columns --separation-mm 100 --n-throws 3` (no `--reload`, DISARMED) — **the feed wait timing out** | Goal ACCEPTED; console logs `"columns started: bridge REST holds ball A at P1 -- waiting up to 30.0 s for ball B to land near P2 ..."` (handoff_S2's exact start line), then, with no tracker landing offered, `ABORTED_NO_COLUMNS_FEED` after ~30 s: `"no tracker landing resolved the columns feed within the deadline -- the platform is at rest holding ball A (the bridge REST's rest tail)"` (handoff_S2 verbatim). This IS the row that proves the timeout path before Block C risks it live. |
| 17 | Ctrl-C mid-attempt on one of rows 14–16a (repeat the row after) | `^C -- cancelling the goal (cancel on stop)`, `outcome=STOPPED` (or the current end_code). |
| 18 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK: ...`, `box OK: ('P1','P1') 0.85-0.95; ('P2','P2') 0.85-0.95; ...` (whatever bands the 200 000 sweep produced — read the live line, don't assume R4's numbers), `hop box OK: P1->P2 at 250.0 mm`, `hop box OK: P2->P1 at 250.0 mm`. **No dedicated "columns box OK" line exists** (`_svc_check` only checks self_toss's default pair and hop's two pairs, per `skill_node.py:3595-3653` as of this rung) — the columns `(P1,P1)`/`(P2,P2)` bands, if swept, show up folded into the same `box OK: ...` listing by site-pair name; confirm their apex band reads **0.90 m**, not R4's 0.85 (owner decision 2026-09-30, § 0) before Block C. A `LimitsMismatch` anywhere in this line means row 11 ran after row 3a's box, or with the wrong number — stop, do not power further. |
| 19 | Now ARM | Ready for § 2.5. |

## 2.5. The leg-jerk ramp measurement (owner decision 2026-09-30 — pre-registered stop rule)

Modelled on `plans/active/leg-gain-tuning-methodology.md`'s ramp discipline: measure at the new
limit before trusting a throwing pattern to it. 300 / 5000 / **200 000** is already live (§ 2 row
11) — this block is the FIRST thing thrown at it, watched closely enough to revert cleanly if it
doesn't hold.

**Pre-registered stop rule**: if ANY of the following occurs during this block, STOP the ramp,
`ros2 service call /trajectory/set_limits ... leg_jerk_limit_mmps3: 150000.0`, confirm
`skills/check`'s `box OK` line now matches the 150 000-stamped box (row 3a's alternate value —
re-sweep or re-select as needed), and fly the REST of this sitting (Blocks A–C) at 150 000:
- `lead_clamp_mask` (on `/link_status`) reads non-zero at any point.
- `torque_clamp_mask` (on `/link_status`) reads non-zero at any point.
- A guard E-STOP latches (`GUARD_LATCHED`, § 7).
- A visibly rougher throw/catch than the 150 000 baseline, by the operator's own eye (the
  standing pushback sentence, § 0, applies here specifically).

| # | Step | Expect |
|---|---|---|
| 20 | `ros2 topic echo /link_status --once` | Baseline `lead_clamp_mask: 0`, `torque_clamp_mask: 0` before any throw. |
| 21 | 3–5 × `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, num_cycles: 1}"` | COMPLETED, `caught=1` each; record `seat=` and landing per throw. Watch `/link_status` between throws. |
| 22 | `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: hop, apex_m: 0.9, separation_mm: 250.0, num_cycles: 5}"` (a SHORT chain, not R4's 10-catch gate — this block is measuring the ramp, not re-running R4's gate) | Watch every throw's `/link_status` for the two clamp masks; record consecutive catches, any `LIMIT_JERK` re-send count (R4's own watch item, § 9 below), and any guard latch. |
| 23 | Apply the stop rule against everything observed in rows 21–22 | Record the verdict (ramp HELD at 200 000 / ramp REVERTED to 150 000) in § 11 before Block A — it decides the session limits value quoted in every later block's table row. |

## 4. Block A — the 4° cup test

The R4 reload (self_toss pattern, GUI or CLI), `hold_tilt_max_deg` swept 4° → 8° → 12° same
sitting. `ros2 param set /skill_node hold_tilt_max_deg 4.0` (handoff_S's exact line; takes
effect on the NEXT BB announcement, not retroactively — it is read once per
`_install_announced_reload` call). Revert with `hold_tilt_max_deg 12.0` when done with this
block.

3–5 reloads at 4.0, then the same count at 8.0 and 12.0 as controls. What to watch, every
reload: the pre-tilt REST settles at ~ the commanded cap; the ball enters ~2× the cap off the
cup axis (4° cap → ~8° off-axis, matching handoff_S's derivation of the receive-tilt geometry);
the `OUTCOME` line's `seat=` (+0.05…+0.15 s smooth, > 0.20 s or none = rebound, R4 sheet's own
signal); the hand sensor EMPTY→HELD once, not HELD→EMPTY→HELD; by eye whether it rattles.

| # | Step | Expect |
|---|---|---|
| 24 | `ros2 param set /skill_node hold_tilt_max_deg 4.0` | `Set parameter successful`. |
| 25 | GUI: pattern self_toss, **Reload first**, hold-to-confirm Start. Repeat 3–5×. | Debug line reads `hold tilt capped at 4.0 deg, resulting hold <=4.000 deg`; operator INFO line reads `(hold tilt <=4.0 deg, cap 4.0 deg)` (handoff_S's exact wording). Record per reload: hold angle logged, `seat=`, caught Y/N, rattle by eye. |
| 26 | `ros2 param set /skill_node hold_tilt_max_deg 8.0` ; repeat row 25, 3×. | Same table, cap 8.0. |
| 27 | `ros2 param set /skill_node hold_tilt_max_deg 12.0` ; repeat row 25, 3×. | Same table, cap 12.0 (the R4-flown control — should read like R4 sitting 4's reload rows). |
| 28 | Verdict: ≥ 4 of 5 at 4° seat smoothly with no rebound signature → the cup takes 4°. | Record in § 7/§ 11 — the numbers decide the F-c re-open (a capture-span catch design), not impressions. |

## 5. Block B — the fused reload

**Set `hold_tilt_max_deg` back to 12.0** (the R4-flown, F-b-measured operating point;
F-b's floor table in this rung's handoff is at the 12° / (4.871°, −10.849°) BB arrival, not 4°)
before this block.

Fly the R4 reload again (self_toss, then hop, GUI button), watching specifically for what F-b's
fused window changes versus R4's separate catch → DECAY REST → throw: the catch now CARRIES the
throw with **no decay REST** (one fewer stop between catch and the next launch); the throw
release ~0.2 s sooner than R4's timing; `hold_tilt` released exactly at touch-down (not held
through a separate decay tail); no `HAND_LANE_REFUSED`; no guard latch. F-b's floor table has
two measured points: **0.70 s at 300/5000/150 000 (hand 3500)** and **0.65 s at the full YAML
ceilings (1000/5000/200 000, hand 3900)**. The ramp's own point (300/5000/200 000, hand 3500)
was NOT probed: expect something between the two, nearer 0.65 s if leg jerk alone binds the
window (F-b's reading), and the planner refuses by name (`LIMIT_JERK` / `HOLD_WINDOW`) if it does
not fit — there is no pre-registered timing gate in this block. If the ramp reverted to 150 000,
expect the 0.70 s floor and a correspondingly later observed release.

| # | Step | Expect |
|---|---|---|
| 29 | GUI: pattern self_toss, Reload first, hold-to-confirm Start. Two or three. | PRE-TILT REST → **held-tilt CATCH that carries the throw directly** (no separate DECAY REST line on console) → the pattern's own throws. Time the catch-to-release gap by eye/bag against R4's separate-REST timing. |
| 30 | Same, pattern hop, 2×. | Same fused shape, then the hop's own alternating throws. |
| 31 | `ros2 service call skills/check std_srvs/srv/Trigger` between attempts | Still clean. |

## 6. Block C — columns from a human lob (the interim start, D1)

`ros2 param set /skill_node separation_mm 100.0` first (the columns operating point —
`sites.columns_sites(100.0)`: P1 (−50, 0), P2 (+50, 0) mm; catch plane z 830, release z 860). Set
`catch_resend_max 0` (§ 2 row 10a) before this block specifically.

The operator seats ball A in the hand at what will become P1, then starts `columns` with
`reload: false`. Console prints (handoff_S2 verbatim, `skill_node.py`'s `_run_columns`):
`"columns started: bridge REST holds ball A at P1 -- waiting up to 30.0 s for ball B to land
near P2 ..."`. Once that line reads (the wait is armed), the operator LOBS ball B **≥ 1.6 m above
the cup** to land within **±20 mm of P2** (`COLUMNS_FEED_BOUND_X_MM=10.0`,
`COLUMNS_FEED_BOUND_Y_MM=20.0` — handoff_S2 notes these bounds are taken verbatim from the R5
brief text, not independently re-derived from a sweep cell this session could locate; flag any
observed miss against these numbers as a finding for the main session to trace, not something to
adjust here). **`detect_human_throws` must be turned on for this block**:
`ros2 param set /ball_tracker_node detect_human_throws true` before lobbing, `... false` after
(handoff_S's runsheet line — live-updatable, no relaunch, confirmed in the tracker's ready INFO
line and a live-change INFO line `detect_human_throws -> <bool> (ros2 param set)`).

A lob that lands but is too LATE for the transit catch is not claimed — logged once:
`"columns feed: ball %d lands in %.3f s, %.3f s short of the %.3f s the transit catch needs (tau
... + launch ... + lead ... + one tick) -- still waiting"` (handoff_S2 verbatim) — the wait
continues; a second short lob is silent (no repeat spam). A lob that lands OUTSIDE the ±10/±20 mm
bound is simply never claimed (no message at all — read the mocap by eye to know it happened). A
good lob claims: `"columns feed accepted: ball B lands in %.2f s -- catching it, then %d
throw%s"`.

| # | Step | Expect |
|---|---|---|
| 32 | `ros2 param set /skill_node separation_mm 100.0` ; `catch_resend_max 0` ; `/ball_tracker_node detect_human_throws true` | All `Set parameter successful`. |
| 33 | `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns, apex_m: 0.9, separation_mm: 100.0, num_cycles: 6}"` (no `reload` field — defaults false) | `"columns started: ..."` line. Lob ball B once the wait is armed. |
| 34 | Watch: the deficit line for a too-late lob, the accepted line for a good one, `ABORTED_NO_COLUMNS_FEED` if 30 s pass with nothing claimed (message: `"no tracker landing resolved the columns feed within the deadline -- the platform is at rest holding ball A (the bridge REST's rest tail)"`) | Record attempts, misses (with reason: late / off-bound / no-throw), the first accepted feed. |
| 35 | Once a feed is accepted and the schedule installs: watch the catch of ball B, then the alternating throws (box apex band **0.90 m**, not R4's 0.85 — owner decision 2026-09-30, confirmed at row 18). Record consecutive catches — the learning curve, not a pass/fail count. | |
| 36 | Repeat row 33 with `num_cycles: 20` once a `num_cycles: 6` attempt lands its feed cleanly. | Longer chain, same watch list. |
| 37 | **D2 — the Stop**: end an attempt deliberately (GUI Stop / `jugglebot/juggle_stop`) mid-chain with ball B still in the platform's possession-adjacent state. | The platform moves under the LAST ball's landing as though catching it with the hand kept low, so it strikes the already-held ball and comes to rest ON the platform — **not a resumable state**. Operator end line: `"... done (one ball held, the last ball rests on the platform — not resumable, cone delivery is a later rung): ..."` (handoff_S's exact wording, `report.py::end_line`'s `shadow_landing_stop` kwarg). Clear the platform by hand before the next attempt. |
| 38 | **D3 — a drop with the survivor still in flight**: if one ball is dropped mid-chain with the other genuinely still airborne (do not engineer this — just record it if it happens naturally). | The survivor's next CATCH is dispatched WITHOUT its carried throw, then the closing REST. End code `DROPPED_SURVIVOR_STOPPED` (ERROR severity on the end line). No memory row is written for the dropped ball's flight (E-2's design — shadow-landed / dropped flights are excluded from the learner on purpose). |
| 39 | `ros2 service call skills/check std_srvs/srv/Trigger` after each attempt | Still clean. |

## 7. Pre-registered verdict table

R3/R4 sheets have the full launch/precondition refusal table
(`REJECTED_MOCAP_STALE`, `REJECTED_NOT_LEVELLED`, `REJECTED_HAND_STALE`, `REJECTED_BALL_UNKNOWN`,
`REJECTED_NO_BALL`, `REJECTED_FRAME_OFFSET`, `ABORTED_MODE_CHANGED`, `ABORTED_NO_RELEASE`,
`NO_ADMISSIBLE_COMMAND`, `NO_LANDING`, `SPLICE_TOO_LATE`, `WINDOW_TOO_SHORT`, `UNREACHABLE`,
`LIMIT_JERK`/`LIMIT_ACC`/`LIMIT_VEL`/`HAND_STROKE`/`HAND_ACC`, `STALE_STATE`/`WRONG_MODE`,
`CUP_CONTACT_ACC`, `GUARD_LATCHED`, `SUPERSEDED_BY_HOLD`, `REJECTED_BB(<message>)`,
`ABORTED_BB_NOT_READY(<state>, ball_in_hand=<bool>)`, `ORIGIN_TOO_LATE`, `ABORTED_BB_THROW_TIMEOUT`,
`ABORTED_NO_ANNOUNCEMENT`, `hop refused: no admissible box covers site pair(s) ... at apex ...`)
— **unchanged this sitting, still applies verbatim.** New or changed for R5:

| Code | Physical fact |
|---|---|
| `LimitsMismatch` ("admissible box for site pair %r was swept with %s=%r but the live session limit is %r -- regenerate config/generated/admissible_box.yaml (tools/admissible_sweep.py)") | The box on disk was swept at a different leg vel/acc/jerk or hand-acc limit than the live session — the ramp block (§ 2.5) changed `leg_jerk_limit_mmps3` and the box wasn't re-swept to match, or `set_limits` ran after `skills/check` instead of before (§ 0 point 6). Fix: match the live limit to the box, or re-sweep. |
| `ABORTED_NO_COLUMNS_FEED` | Columns' feed wait (§ 6) timed out — `COLUMNS_TRACKER_FEED_DEADLINE_S` (30.0 s, a policy ceiling not a hardware measurement, handoff_S2 flags it explicitly for owner confirmation) elapsed with no BB announcement and no tracker landing near P2 claimed. The platform is at rest holding ball A. |
| `DROPPED_SURVIVOR_STOPPED` | D3: one ball was dropped (no release evidence within the deadline) while the OTHER ball was still in flight — the survivor's own catch is kept, its carried throw is stripped, and the attempt ends through a closing REST at the survivor's site once that catch finalises. ERROR severity, no memory row for the dropped ball. |
| the D2 shadow-landing end line (empty `end_code`, `shadow_landing_stop=True`) | A clean columns finish with one ball still held: the last ball's landing is deliberately aimed at the platform's other, already-held ball (D2) — not a fault, the designed stop shape. Not resumable; clear the platform before the next attempt. |
| `hold tilt capped at %.1f deg, resulting hold %.3f deg` (DEBUG) / `(hold tilt %.1f deg, cap %.1f deg)` (operator INFO) | Block A's `hold_tilt_max_deg` parameter is in effect for this reload — read the actual resulting angle against the requested cap; they should match unless the BB arrival velocity's natural receive-tilt was already below the cap (in which case the resulting angle is smaller than the cap, not a bug — R4's 12° cap example: an 11.892° natural tilt resulted in <12°, unclamped). |
| `columns feed: ball %d lands in %.3f s, %.3f s short of the %.3f s the transit catch needs (tau ... + launch ... + lead ... + one tick) -- still waiting` (INFO, once per short candidate) | A human lob was seen and would land near P2, but too late for the transit catch to reach it in time — the wait continues; this is not a refusal of the attempt, just of that one candidate. |

## 8. If something refuses or latches

Unchanged from R4 sheet § 8 (never re-dispatch a hand move by hand; Ctrl-C / `jugglebot/juggle_stop`
/ GUI Stop are all safe at any time; an ended attempt installs one `trajectory/hold`). One
R5-specific note: **a `LimitsMismatch` at `skills/check` or at a goal's accept ladder is not a
hardware fault** — it means the live `set_limits` and the box on disk disagree; do not power past
it hoping it clears itself, fix the ordering/value (§ 0 point 6, § 7) and re-check.

## 9. Carried watch items (plan § 0, physical operating point)

R4 sheet § 9's seven items (hand recovery park's upper-stop push, hand `MAX_DEVIATION` latches
from stale-encoder bursts, the contact-window re-send skip, re-send refusals mostly `LIMIT_JERK`,
the seam tilt-RATE pin, the flaky `test_the_qp_and_the_gate_stay_within_an_order_of_magnitude`,
the translation-only post-release hold) all still apply verbatim — this sitting doesn't retire
any of them. Add:

8. **The ramp block's stop rule (§ 2.5) is itself a carried decision for the REST of this
   sitting** — if it reverted to 150 000, every later block's table numbers (Block A/B/C) are at
   150 000, not 200 000; record which in § 11 before filling in any per-throw table so a later
   reader isn't comparing apples to oranges.
9. **`COLUMNS_TRACKER_FEED_DEADLINE_S` (30.0 s) is an unvalidated policy number**, not a measured
   one (handoff_S2) — if it reads too short (a deliberate, unhurried human throw gets cut off) or
   too long (an idle wait feels wrong operationally), that is feedback for the owner to retune,
   not a bug in this sitting.
10. **`COLUMNS_FEED_BOUND_X_MM`/`_Y_MM` (10/20 mm) provenance is incomplete** (handoff_S2
    could not locate the "unit B origin table" the brief cited) — if the human lob block shows a
    lob that LOOKS well-placed by eye but is never claimed, or a poorly-placed one that IS
    claimed, that is a finding about this bound's correctness, not the operator's throwing arm.

## 10. Close-out

| # | Step | Expect |
|---|---|---|
| 40 | GUI Deactivate (or the orchestrator command topic) | Robot stows. |
| 41 | Stop the launch and the load capture. | |
| 42 | `python tools/probes/throw_outcome_bag_probe.py --bag <id>` | Independent ground-truth check against `skill_node`'s own logged memory rows, same as R3/R4. **R5 note**: the probe's `FSM_ERA_CATCH_Z_MM` plane (previously `TRACKER_LANDING_Z_MM`, previously asserted `== 809.08`) is now RECOMPUTED from config (812.98 mm since the 2026-09-27 kinematic calibration) with no literal assert — candidate (a) in the probe's summary is scored against that plane for historical continuity with pre-R3 bags only; it is NOT the live tracker's plane (that's `CATCH_CUP_Z_MM`, 830 mm, candidates b/c/e/e2) and a candidate-(a) error on an R5 bag should not be read as a tracker defect. |
| 43 | Copy `temp/learn/<plant_id>/memory.csv` to a dated path. | |
| 44 | Send: the bag folder name, `temp/learn/<plant_id>/memory.csv`, the loadavg/launch logs, the ramp verdict, and the throw-by-throw tables from §§ 2.5–6. | |
| 45 | Log the sitting (`/log feature`, or `/investigate` if anything in § 7 fired unexpectedly or a reload catch showed a rebound signature). Update plan § R5 Outcome and the memory file. | |

## 11. Results

### Ramp verdict (§ 2.5)

| Item | Result |
|---|---|
| `lead_clamp_mask` / `torque_clamp_mask` seen non-zero? | |
| Guard latch during the ramp block? | |
| Verdict: HELD at 200 000 / REVERTED to 150 000 | |
| Session limits used for Blocks A–C (fill in once decided) | |

### Pre-power (§ 1) / bring-up (§ 2) / dress rehearsal (§ 3)

| Item | Result |
|---|---|
| Date | *(fill in)* |
| `./run_tests.sh --full` — (date, command, result) triple | |
| gate_hash match (row 3) | |
| box `leg_jerk_mmps3` stamp (row 3a) | |
| hop + columns box rows present (row 3b) | |
| Row 13 (`--rehearse --pattern self-toss`) — G1/G2/G4 verdict | |
| Row 14/15 (`--via-action`, self-toss/hop, disarmed) — outcome | |
| Row 16/16a (`--via-action --pattern columns`, reload/no-reload, disarmed) — outcome, feed-timeout observed? | |
| Row 18 (`skills/check`) — box bands, columns apex band (expect 0.90) | |

### Block A — 4° cup test (§ 4)

| Cap (deg) | Attempt | Hold angle logged | `seat=` | Caught | Rattle by eye | Notes |
|---|---|---|---|---|---|---|

**Verdict (≥4/5 at 4° smooth, no rebound → cup takes 4°):** ___

### Block B — fused reload (§ 5)

| Pattern | PRE-TILT settle OK? | CATCH carries throw (no DECAY REST line)? | Release ~0.2 s sooner (by eye/bag)? | Caught | Notes |
|---|---|---|---|---|---|

### Block C — columns from a human lob (§ 6)

| Attempt | Lob outcome (late / off-bound / accepted) | Feed lands in (s) | Deficit (s), if logged | Ball B caught | Consecutive catches after | Notes |
|---|---|---|---|---|---|---|

**D2 Stop observed?** ___ **D3 drop-with-survivor observed?** ___

### Bag / boot-banner record

| Rung | Time | Bag folder | `BLAS` on both up lines | Notes |
|---|---|---|---|---|
| Dress rehearsal | | | | |
| Ramp measurement | | | | |
| Block A | | | | |
| Block B | | | | |
| Block C | | | | |
