---
title: "R5 sitting 5 (2026-10-05 morning): every JUGGLE goal refused because the launch default and the swept box disagreed on the leg limits, and the hand-jam recovery's raise pushed down first because a retargeted HAND_MOVE_TO plans from the setpoint that ran on below the stalled hand"
type: investigation
date: 2026-10-05
status: resolved
phase: "two-ball-skill-stack — R5"
related_plan: two-ball-skill-stack.md
related_entries:
  - 2026-10-04-skill-stack-r5-sitting-4.md
  - 2026-10-04-skill-stack-r5-sitting-3.md
files_changed:
  - config/hardware_config.yaml
  - config/generated/hardware_config.py
  - config/generated/admissible_box.yaml
  - ros_ws/src/jugglebot/jugglebot/motion/hand_jam.py
  - tests/motion/test_hand_jam.py
  - tests/motion/test_skills_admissible.py
  - tests/ros/test_teensy_bridge_node_hand_jam.py
  - tests/ros/test_trajectory_node.py
  - tests/ros/test_skill_node.py
  - tests/motion/test_trajectory_plan.py
  - tests/motion/test_trajectory_planner_move.py
  - tests/hardware/session_skills_r5_sitting5.md
tags: [skill-stack, R5, hand-jam, admissible-box, session-limits, can-bridge]
---

# R5 sitting 5, morning: the launch refused every pattern, and the jam recovery's second raise never moved

## Summary

Two findings from the first three launches of `tests/hardware/session_skills_r5_sitting5.md`
(bags `2026-10-05_10-30-08`; launch logs `~/.ros/log/2026-10-05-{09-53-35,09-58-11,10-30-08}-*`).

1. **Every `jugglebot/juggle` goal was refused** with `admissible box for site pair ('P1', 'P1')
   was swept with leg_vel_mmps=350.0 but the live session limit is 300.0`. The box has been
   swept at 350/5000/200000 since sitting 3 (the roomier R5 geometry); the launch default was
   still the R2/R3 point 300/5000/150000; the only thing coupling them was runsheet row 10
   (`trajectory/set_limits`), and no launch this morning logged a `leg limits set:` line, so
   the ramp never ran. The fix makes the launch default the R5 working point and pins the
   committed box to the launch default in the suite, so the two can no longer drift apart
   between sittings.
2. **The hand-jam recovery fired honestly on both bench pinches and ended `HAND_JAM_UNRECOVERED`
   both times** with `raise did not track (rose -0.001 rev in 0.3 s)`. The owner's reading was
   that a 1.0 rev raise is too small for the ball to pass under the hand, and that is true,
   but it is one of three defects. The bag shows the first raise was passed by the tracking
   check only because the ball sprang back 0.31 rev when the current dropped from 50 A to
   10 A; the hand then sat pressed on the ball at −8.6 A for 0.6 s before rising, at ~1 rev/s
   instead of the commanded 2.5. The second raise sat at −10.5 A for its whole window. A
   HAND_MOVE_TO issued while another is in flight is a firmware RETARGET, and the ODrive
   plans a retargeted trap trajectory from its own setpoint, which kept descending while the
   hand was stalled. The fix anchors the setpoint on the hand before every raise (a
   HAND_MOVE_TO to the measured position, which ARRIVES at once and hands the axis to
   PASSTHROUGH on the hand), makes the raise an absolute clearance height (5.0 rev, the
   owner's number) instead of a lift above the stall, and replaces the test plant with one
   that carries the ODrive's setpoint so the old machine fails in it the way the robot did.

**If your physical intuition disagrees with the framing below — the anchor, the 5.0 rev
clearance, the launch default moving to 350 — that's load-bearing signal; say so.**

## What the owner reported

> The jam detection is working now, but we need to update it to have the hand raise to 5.0
> rev after the stall is detected to allow the ball to pass underneath the hand. Raising by
> 1.0 rev is not enough. I then tried Step 3, but every time I tried to send any JUGGLE
> command, I was met with "ERROR jugglebot/juggle REJECTED: admissible box refused: admissible
> box for site pair ('P1', 'P1') was swept with leg_vel_mmps=350.0 but the live session limit
> is 300.0 …" Why is that happening now? Please fix this.

## Measured

### The refusal

- `config/generated/admissible_box.yaml` (committed 2026-10-04, `8df4e11a`): `limits:
  leg_vel_mmps 350, leg_acc 5000, leg_jerk 200000, hand_acc 3900`, gate `458bb544b345`.
- `config/hardware_config.yaml` `trajectory_op`: `leg_vel_limit_mmps 300, leg_jerk_limit_mmps3
  150000` (the 2026-09-16 R2/R3 point). `skill_node._live_limits` reads the live
  `trajectory/status`, which carries the launch default until `trajectory/set_limits` runs.
- The three launch logs: `BRIDGE_FW_CHECK: OK — can-bridge v27` in each (FW 27's receipt,
  deferred from sitting 4, is now in hand); `level_trim_deg = [+0.0000, +0.0000]` at startup
  and no later trim line (row 9 not run); **no `leg limits set:` line in any launch** (row 10
  not run; `_svc_set_limits` logs one on every call); 8 refusals, all the same text.
- Sitting 4's sheet row 8 and sitting 5's row 10 both ramp 350/5000/200000 by hand at bring-up;
  both sheets' stop rules note that reverting to 300 ends every pattern. The coupling was a
  runsheet row.

### The jam recovery (bag `2026-10-05_10-30-08`, pinch 1 at 1791156698.5; pinch 2 at
1791156795.4 is the same shape)

| t (s, rel. 1791156698.5) | event | hand pos (rev) | iq (A) |
|---|---|---|---|
| −0.2 … 0.04 | bench lower (HAND_MOVE_TO 0.0 at 1 rev/s) stalls on the ball | 2.957 → 2.813 | −39 → −49.6 (clamp) |
| 0.04 | `HAND JAM detected … stalled at +2.813 rev` → relief 10 A, clear, disarm | 2.813 | −49.6 |
| 0.09 | raise 1 commanded (+3.813 = stall + 1.0), a RETARGET of the bench lower | **3.223 → 3.125** (the ball springs back 0.31 rev as the clamp is relieved) | −8.6 |
| 0.09 … 0.69 | hand pressed on the ball, not moving | 3.125 | −8.6 |
| 0.69 … 1.39 | hand rises at ~0.96 rev/s (the bench move's 1 rev/s cruise, not the raise's 2.5) | 3.129 → 3.801 | −7 → +2 |
| 1.54 | ARRIVED at 3.827; dwell 0.6 s | 3.827 | ≈ 0 |
| 2.04 | lower (a fresh HAND_MOVE_TO at 2.5 rev/s) | 3.824 → 3.26 | −1 → −6 |
| 2.39 | stalls on the ball again, 0.44 rev above the first stall | 3.26 | −10.5 |
| 2.69 | raise 2 commanded (+5.754 = stall + 2.5), a RETARGET of the stalled lower | 3.253 | −10.5 |
| 2.69 … 2.99 | hand pressed on the ball for the whole tracking window | 3.253 | −10.5 |
| 2.99 | `raise did not track (rose -0.001 rev in 0.3 s)` → hold at measured, `HAND_JAM_UNRECOVERED` | 3.28 → 3.30 | −5 |

Pinch 2: detected at 2.788, raise 1 to 3.79, lower stalled at 3.242, raise 2 to 5.74,
`rose -0.000 rev in 0.3 s`, UNRECOVERED. Both `/recover` resumes lowered to the park and
restored 50 A cleanly once the ball was removed.

The relevant firmware facts (`Teensy_code_canbridge/leg_activate.cpp`, FW 26/27):

- A HAND_MOVE_TO from IDLE runs the ACTIVATE ladder: SETUP seeds `set_input_pos(cur)` (the
  measured position), sends the cruise and the trap accel/decel limits (30 rev/s², the YAML
  `trap_acc_limit_rps2`), switches to POSITION/TRAP_TRAJ, settles 10 ms, then commands the
  target. The ODrive plans from the seeded setpoint, i.e. from the hand.
- A HAND_MOVE_TO while one is in flight is a RETARGET: the old request is answered
  SUPERSEDED, the cruise is re-sent and the new target commanded; the ODrive's trap planner
  replans **from its current setpoint position and velocity**, which is not the hand when
  the hand is stalled.
- Arrival is judged on the MEASURED position (`TARGET_REACHED_POS_TOL_REV` 0.01,
  `..._VEL_TOL_RPS` 0.1); on arrival the axis is handed to PASSTHROUGH, where the ODrive's
  setpoint becomes the last `input_pos` — the target.

The recovery (`motion/hand_jam.py`) issued every raise-again and the hold as a RETARGET by
design ("never by waiting on the firmware's 10 s move timeout"). Its test plant moved the hand
itself at the cruise, with no setpoint, so none of this was visible to the suite.

## Discussion

**Hypothesis withdrawn.** "Raising by 1.0 rev is not enough" was the owner's reading of a hand
that came back down onto the ball; it is correct about the height (a 1.0 rev lift from the
squashed stall put the cup 0.56 rev above the ball's relaxed height of 3.25 rev, and the ball
needs to pass under the cup, not merely be released by it), but it does not explain the
second raise, which did not move at all. The height fix alone would have reproduced this
morning's outcome with a bigger number in the log.

**Three defects, one physical cause for two of them.** (D1) The raise targets were relative to
a stall position measured with the ball squashed at 50 A; the owner's physical knowledge says
5.0 rev clears a ball resting on the ring. (D2) A retargeted raise plans from the lower's
run-ahead setpoint: at 2.5 rev/s cruise, the 0.15 s stall confirmation alone puts it 0.4 rev
below the hand, the replan reverses a −2.5 rev/s setpoint through 30 rev/s², and the hand does
not move until the setpoint climbs back past it — longer than the 0.3 s tracking window by
construction. (D3) "Relief" was current-only: for as long as the setpoint sat below the hand
the controller kept pushing the ball at the relief current (−8.6 to −10.5 A, 0.6 s on raise 1).
The first raise "tracked" by a side effect of the relief, the spring-back, which is why
sitting 4's bench check and this morning's pinch 1 both looked like a working raise 1 and a
failed raise 2.

**Why an anchor, and not a wider tracking window.** Widening the window to a second would let
the retargeted raise pass, with the hand pushing the ball down for the first few hundred
milliseconds of every raise. The invariant is "relieved before anything else and never pushed
through"; a raise that pushes first violates it even when it eventually rises. The anchor
(a HAND_MOVE_TO to the measured position once the hand has rested 0.15 s) uses the firmware
exactly as written: the arrival test is on the measured position, so the anchor ARRIVES on the
next MONITOR tick regardless of where the setpoint is, and the PASSTHROUGH hand-off snaps the
setpoint onto the hand. The push stops there, and the raise that follows is a fresh move
seeded at the hand, at its own cruise. It costs one deferred RPC and ~20 ms. It is applied
before every raise, including the first (where in production, after a streamed-lane jam, no
move is in flight and the anchor is a trivial seeded move — uniform, no bridge-side
"is a move in flight" bookkeeping) and before the final raise on a resumed lower.

**Why the anchor waits for rest.** The relief lets the squashed ball push the hand up 0.3 rev
in ~100 ms. An anchor taken at the stall position during that spring-back would be a command
to drive the hand back down onto the ball and would never ARRIVE (the ball holds the hand
above it at 10 A). The rest wait is the lower stall monitor's own criterion (|v| < 0.3 rev/s
for 0.15 s), bounded at 1.0 s so a hand that will not settle is anchored where it is and the
2 s arrival wait decides. An anchor that does not confirm is retried once at the new measured
position, then UNRECOVERED with the relief in place.

**The alternative not taken: a firmware "cancel at the hand".** A HAND_MOVE_TO flag that
re-seeds the setpoint on a retarget would be the same effect inside the ODrive mode machine
(write `input_pos = cur` while still in TRAP_TRAJ, then switch modes), with an FW 28 and a
flash. The anchor achieves it with the existing wire and is observable in the bag (the
SUPERSEDED/ARRIVED pair per anchor).

**The limits: why the launch default moves, not the box.** The box is swept at the limits the
planner is gated against; the R5 geometry needs 350 mm/s and the ceiling jerk (sitting 3's
entry). The precedent is the 2026-09-16 YAML note: the launch default *is* the current arc's
operating point and the sweep's. The alternative, skill_node applying the box's limits itself,
would make a goal a limit-setter — a side effect on a safety setting from a pattern request —
and the strict equality in `check_limits` is deliberate in both directions. The class of
failure is "two sources of truth coupled by a runsheet row", and the contract that closes it is
a test that loads the committed box and refuses the commit when its limits, gate hash or dwell
differ from what a default launch starts with. **The trade-off accepted:** the YAML change
touches `hardware_config.py`, one of the seven gated files, so the box had to be re-swept
(same limits, new gate hash) — and `cup_realize.TILT_JERK_LIMIT_DEFAULT_RAD_S3` derives from
the YAML jerk default; the live planner takes its tilt caps from the session limits
(`unified_cycle`, "the session limit enters exactly here, and exactly once"), so the sweep's
result should not move. The re-sweep is the empirical check of that claim: see Verification.

**What this does not change.** The jam detector itself (P1–P6) is as sitting 4 left it and
fired honestly on both pinches at the clamp within ~0.2 s of the stall. The relief current,
the dwell, the lower's stall monitor and the final-raise-and-stay are unchanged. The bench
cruise (`hand_jam.bench_vel_rps` 1.0) is unchanged.

## Fix

- `config/hardware_config.yaml`: `trajectory_op.leg_vel_limit_mmps` 300 → 350,
  `leg_jerk_limit_mmps3` 150000 → 200000, with an "R5 WORKING POINT (2026-10-05)" paragraph
  after the 2026-09-16 one; `python config/generate_config.py` (the `.py`/`.h` constants only).
- `config/generated/admissible_box.yaml`: re-swept at the same limits (now the defaults),
  gate `6f535fc4f888`; see Verification for the bit-identity against sitting 3's box.
- `tests/motion/test_skills_admissible.py::test_the_committed_box_is_swept_at_the_launch_defaults_and_the_live_gate`
  and `tests/ros/test_skill_node.py::test_the_committed_box_is_swept_at_the_node_default_dwell`:
  the enforcement point for the launch/box contract. `tests/ros/test_trajectory_node.py`'s
  working-point tripwire now pins 350/5000/200000 and says why.
- `motion/hand_jam.py`: `raise_to_rev` 5.0 (absolute; ROS parameter `hand_jam.raise_to_rev`,
  replacing `hand_jam.raise_rev`) with the fixed `raise_min_rev` 1.0 floor and
  `raise_attempts` 2; `Step.ANCHOR` / `ANCHORING` before every raise (after the disarm; after
  every lower stall; before the final raise), with `anchor_rest_s` 0.15, `anchor_wait_max_s`
  1.0, `anchor_attempts` 2; the summary line counts anchors; the dry-run text names the
  anchor; the module docstring carries the new failure-mode row.
- `tests/motion/test_hand_jam.py`: `_Plant` now carries the ODrive setpoint (trap trajectory
  at the cruise and 30 rev/s², retarget from the setpoint, fresh start seeded at the hand,
  measured-position arrival with the PASSTHROUGH hand-off, the relief spring-back); six new
  tests, one of which reproduces the bag's retarget outcome in the plant and one of which
  shows the anchored raise2 tracking in the same plant.
- `tests/hardware/session_skills_r5_sitting5.md`: row 4 (gate hash), row 10 (the ramp is now a
  read-back), rows 16–17 (the anchor, the 5.0 rev clearance), the FW 27 receipt marked
  received.
- `tests/motion/test_skills_admissible.py::tiny_sweep`: limits pinned at the point it
  characterised (see Verification).

## Verification

All runs 2026-10-05 unless stated. Console output by path under `temp/logs/`.

- **Mutation check** (`scratchpad/s5/mutation_check.py`, HEAD's `hand_jam.py` driven through
  the new `_Plant`): "ball stays through dwell 1" → `HAND_JAM_UNRECOVERED | raise did not
  track (rose +0.000 rev in 0.3 s)`, moves `raise1, lower, raise2, hold`; "wedged" → the same;
  i.e. the old machine fails in the new plant exactly as the robot did. With the fix the same
  scenarios end RECOVERED / UNRECOVERED-raised-at-5.0 with every raise tracking.
- **Admissible re-sweep** (`scratchpad/s5/sweep_run_s5.sh`, sitting 3's arguments, `--dwell-s
  0.27 --leg-vel 350 --leg-acc 5000 --leg-jerk 200000 --hand-acc 3900` with the columns /
  hop / single grids, `OMP/OPENBLAS_NUM_THREADS=1`, runs S5a 10:53–11:18 and S5b 11:18–11:47;
  logs `temp/logs/admissible_sweep_s5_{S5a,S5b}_{columns,hop,single}_20261005.log`, driver
  `temp/logs/sweep_s5_driver.log`): S5a/S5b `cmp` IDENTICAL for all three files (md5 columns
  `7749b1558667cec9b24cec3e505a3e3c`, hop `0bd88b6bb8dcc16f5e15b28e7974ee33`, single
  `8ed1ff6241b70563c13ac96fdfac2b96`); each S5a file is IDENTICAL to sitting 3's S4d file apart
  from the `gate_hash` / `swept_at` lines; S5b merged into `config/generated/admissible_box.yaml`
  in the (single, hop, columns) order that reproduces the committed file byte-for-byte from the
  S4d parts (`scratchpad/s5/merge_s5.py`; md5 `0ddcd692f679743387951734585fdc0a`, 19 boxes,
  gate `6f535fc4f888`): **`git diff` against `8df4e11a`'s box is exactly two lines, `swept_at`
  and `gate_hash`** — every landing band and apex band is unchanged, so the derived tilt-jerk
  default is confirmed off the sweep's path.
- **The audit's one behavioural finding** (`/audit --unstaged`, 11:13–11:46): the pre-existing
  `tests/motion/test_skills_admissible.py::test_tiny_sweep_produces_one_box_inside_the_swept_grid`
  read the tool's default limits, which follow the YAML launch default, and at 350/5000/200000
  its −5 mm x cell passes (the planner is more permissive), breaking its "x pinned at 0"
  characterisation. Pinned to the 300/5000/150000 it was characterised at (`_TINY_SWEEP_LIMITS`)
  — a characterisation test freezes its conditions. The audit's other narrative findings were
  the pending state of this very section (now filled) and a plant-trivial assertion in the new
  anchor test, replaced by a per-move push-time assertion (raise 1 and raise 2 push < 0.02 s;
  the stalled lower > 0.1 s; the old machine in the same plant: 0.3 s during raise 2).
- `pytest tests/ros/test_skill_node.py -q` (after the merge; before it the gate-hash change
  failed 19 of its tests on `LimitsMismatch`, as the audit counted): **203 passed in 9.37 s**.
- `pytest tests/motion/test_skills_admissible.py -k "committed_box or check_limits" -q`:
  **9 passed**; `-k tiny_sweep`: **2 passed**.
- `pytest tests/motion/test_hand_jam.py -q` (BLAS 1 thread): **61 passed in 0.47 s**.
- `pytest tests/ros/test_teensy_bridge_node_hand_jam.py -q`: **13 passed in 14.87 s**.
- Two more default-limit consumers surfaced by the first full gate (`./run_tests.sh --full`,
  12:00, `temp/logs/gate_full_r5_sitting5_fixes_20261005.log`: 2 failed, 6022 passed):
  `tests/motion/test_trajectory_plan.py::test_too_fast_move_raises_limit` (at 350/200000 the
  80 mm move refuses LIMIT_JERK first) and
  `tests/motion/test_trajectory_planner_move.py::test_minimal_feasible_when_duration_none`
  (a 20 mm move now fits inside the 0.20 s `min_move_duration_s` floor, so 90 % of the found
  duration is still feasible — the floor binds, not a limit). Both pinned at 300/5000/150000,
  the point they characterised.
- **Full gate** (`./run_tests.sh --full`, 2026-10-05 12:08,
  `temp/logs/gate_full_r5_sitting5_fixes_20261005b.log`): **6024 passed, 9 skipped, 1 xfailed
  in 296.06 s; serial 6 passed.**

## Open questions

- The bench raise 1 rose at ~1 rev/s under a 2.5 rev/s request. The retarget path re-sends
  `set_traj_vel_limit` and `set_input_pos` in the same tick; `Set_Input_Pos` (0x00C) wins CAN
  arbitration over `Set_Traj_Vel_Limit` (0x011) if both are pending, so the plan may be made
  at the superseded move's cruise. The anchor makes every raise a fresh START (SETUP sends the
  limits a tick and 10 ms before the target), so the recovery no longer depends on it; it is
  noted for the firmware's RETARGET path, not fixed.
- The owner's 5.0 rev is a physical judgement (the cup must clear a ball resting on the ring).
  If a pinch ever stalls above 4.0 rev, the floor lifts a further 1.0 rev; nothing in the band
  [1.0, 3.6] reaches that.
- Whether 350/5000/200000 as the launch default is right for every non-pattern move (jogs,
  `level`, the stow) is the owner's call; it is what sittings 3 and 4 flew for everything
  after bring-up. Reverting is one YAML line plus a re-sweep.
