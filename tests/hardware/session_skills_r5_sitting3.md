# R5 hardware runsheet, sitting 3 — the swapped-layout BB-fed columns gate

Skill-stack R5 (`plans/active/two-ball-skill-stack.md` § R5). Same machine as sitting 1
(`session_skills_r5.md`) and sitting 2 (`session_skills_r5_sitting2.md`) — those sheets stay the
reference for everything this one does not restate (QTM preconditions, the guard/`/recover`/
hand-homing recovery flow, the full refusal-code table, the carried R4/R5 watch items). Driver:
`jugglebot/juggle` action, pattern `columns`; `skills/check`; `jugglebot/juggle_stop`; the GUI
reload button.

**If your physical intuition disagrees with the framing below — the swapped layout, the hand
cap, anything else — that's load-bearing signal, say so before we start.**

**Why this sitting.** Sitting 2 (2026-10-02) found and fixed three mechanisms, offline, from bag/
log analysis — none have flown: (1) the `receive_tilt` level-pin fix never reached the
`InstallSegment` wire, so the live feed catch banked the old `tilt_to_receive` pin; (2) the flown
site layout let Ball Butler's feed collide with Jugglebot's own throw (mocap-confirmed merge in
all 4 attempts where both threw); (3) a fresh-origin THROW/CATCH had no solve-time budget
(`ORIGIN_TOO_LATE`), killing 3/7 attempts before any ball left the cup. Full analysis and fix
detail: `logbook/2026-10-02-skill-stack-r5-sitting-2.md`. This sitting is the first flight of all
six fix units together.

**Operator runs every motion command; Claude never does.** Claude may read logs/bags and propose
diagnosis between blocks, but every `ros2 action send_goal`, `ros2 service call`,
`ros2 param set`, GUI click, and hand/platform touch is the operator's own hands.

## 0. Pre-sitting (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at or after the six sitting-2 fix-unit commits. |
| 2 | (ROS) `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash && cd ..` | **Two-package build** — `InstallSegment.srv` changed (U0, `receive_tilt`), unlike sitting 2's single-package build. |
| 3 | (venv) `PYTHONPATH=ros_ws/src/jugglebot python -c "from jugglebot.motion.skills import admissible as ab; print(ab.gate_hash())"` then `grep -m1 gate_hash config/generated/admissible_box.yaml` | The two hashes EQUAL `458bb544b345` — the hand-3900 box. If the live gate hash differs, do not fly; the box did not get re-swept/installed. |
| 3a | (venv) `grep -m1 "leg_jerk_mmps3" config/generated/admissible_box.yaml` | Reads `200000.0`. |
| 4 | `cat temp/reports/nightly/status` (or the nightly ticker per CLAUDE.md) | Fresh GREEN, or a RED read against `git status`/`git stash list` before trusting it. |
| 5 | (venv) `./run_tests.sh --full 2>&1 \| tee temp/logs/presitting_full_r5_sitting3_$(date +%Y%m%d).log` — launch DOWN | `RESULT: PASS`. Record pass count and wall time in a close-out note. |
| 6 | QTM: `Catching Cone` rigid body DISABLED, Ball Butler reflectors MASKED, `Platform` rigid body tracked | Same standing preconditions as sittings 1-2. |

**Session limits, read before bring-up.** Legs: the LAUNCH default (`config/hardware_config.yaml`
`leg_jerk_limit_mmps3: 150000.0`) is BELOW what this box was swept at — `set_limits` still MUST
run before `skills/check` and every dress-rehearsal call, exactly as sitting 2 did (its
`launch.log` line 231: `"leg limits set: 300 mm/s, 5000 mm/s², 200000 mm/s³"`). Hand: the launch
default is now **3900 rev/s²** (sitting 2's fix) — there is no `set_limits` field for hand
(`SetTrajectoryLimits.srv` is legs-only), so nothing to set.

## 1. Bring-up (launch UP, robot powered, ball present)

Same procedure as sitting 2 § 2 — `level` FIRST, before any skill dispatch; **no separate
leg-jerk ramp block** (stays held at 200000 mm/s³ from sittings 1-2; if `lead_clamp_mask`/
`torque_clamp_mask` go non-zero or a guard latches anywhere this sitting, revert to 150000,
re-check `skills/check`'s box match, and fly the rest of the sitting there).

| # | Step | Expect |
|---|---|---|
| 7 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 300.0, leg_acc_limit_mmps2: 5000.0, leg_jerk_limit_mmps3: 200000.0}"` — before any `skills/check` or goal | `applied_*` echoes `300 / 5000 / 200000`. |
| 8 | `ros2 param set /skill_node apex_m 0.9` ; `separation_mm 100.0` ; `plant_id r5-sitting3-$(date +%Y%m%d)` — a FRESH `plant_id` (cold learner memory) | Set. |
| 9 | Seat a ball in the hand; confirm `/hand_telemetry` (`ball_held_valid: true`, `ball_held_raw: true`) | Visual + topic check. |
| 10 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK`, `box OK`, including the columns site pairs at the 0.90 m apex band. |

## 2. Warm-up: 3 reloads at `hold_tilt_max_deg 0.0`

Confirms U0 (the `receive_tilt` wire fix) and U2/the `STALE_STATE` fix together, on the simplest
case, before asking for BB-fed columns.

| # | Step | Expect |
|---|---|---|
| 11 | `ros2 param set /skill_node hold_tilt_max_deg 0.0` | Set. |
| 12 | GUI: pattern `self_toss`, Reload first, hold-to-confirm Start. Repeat x3. | Each feed caught; watch `seat=` — the reload CATCH window moved to **0.5 s** (was 0.725 s pre-sitting-2) via U2's look-ahead. No `STALE_STATE` refusal on the catch dispatched at the pre-tilt REST's nominal end. |
| 13 | `python tools/probes/feed_catch_bag_probe.py --bag <bag> --log <launch.log>` | Per-feed landing-vs-committed table (data point, not gated this block). |

## 3. The BB-fed columns gate (swapped layout)

**Layout.** A is held/thrown at P2 (+50, 0); the feed is caught at P1 (-50, 0), aimed 20 mm
toward A — params `columns_feed_site` (`'P1'`, the default) and
`columns_feed_aim_toward_a_mm` (`20.0`, the default): **nothing to set**, this is now the
operating point.

| # | Step | Expect |
|---|---|---|
| 14 | `ros2 param set /skill_node catch_resend_max 0` for this block (sitting 1/2's convention — revert after) | Set. |
| 15 | Seat ball A in the hand at P2. `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns, apex_m: 0.9, separation_mm: 100.0, num_cycles: 6, reload: true}"` | Goal ACCEPTED; bridge REST streams holding A at P2; BB is asked to throw at P1. |
| 16 | Watch the console | `"columns feed accepted: ball B lands in %.2f s..."` (INFO), then the catch of ball B at P1 — this is the receive-level catch, should look controlled, not banked — then A's throw fires (one transit of B's flight before B lands), then alternating throws. |
| 17 | **D2 — the Stop**: end an attempt deliberately (GUI Stop / `jugglebot/juggle_stop`) mid-chain | Last throw aimed at the held ball's site; platform rests holding both; clear by hand. |
| 18 | `ros2 service call skills/check std_srvs/srv/Trigger` after each attempt | Still clean. |
| 19 | Once a `num_cycles: 6` attempt completes cleanly, repeat row 15 with `num_cycles: 20` | Longer chain, same watch list. |

**Refusal names worth reading precisely this sitting** (all three changed meaning since sitting
2's fix units landed):

| Code | What it now means |
|---|---|
| `REJECTED_COLUMNS_FEED_UNCATCHABLE` | The feed's landing is too far in **-x** (away from A, past P1) — the swapped layout's cliff direction, opposite sitting 2's flown-layout cliff (which was +x). Names the offset (dx, dy) from P1. |
| `ORIGIN_TOO_LATE` | Now means the solve took **> 150 ms** (the U2 fix moved the budget from 75 ms to the splice's 150 ms `SOLVE_BUDGET_KNOTS`) — a refusal here is a real slow solve, not the old false-positive on a 76-84 ms columns THROW. |
| `HAND_LIMIT_ACC` on a mid-chain fold (not the feed catch itself) | The knife edge sitting 2 found (ball A's 4th same-site fold, ~101% at the old 3500 cap) returning — expected to clear at the new 3900 cap; if it fires, the box/cap mismatch needs checking first (§ 0 row 3). |

**Acceptance**: one clean 6-cycle run, then `num_cycles: 20`.

## 4. What to count

- `NOT_SETTLED`-class refusals (BB settle epidemic, sitting-1 baseline 5/19) — should stay rare
  with BB FW 5 + the Jetson retry-once.
- Hop-style re-aim refusals (`LIMIT_JERK`/`INFEASIBLE`/`HAND_LIMIT_C2`) if hop is flown this
  sitting — carried, known gap, not expected to be fixed by anything landed today.
- The bridge's hand scheduled-group `promo_over` counter, before and after the columns block (the
  sched status line, `leg_interp.cpp:1758-1774`) — confirms or refutes sitting 2's knot-skip
  finding (first-throw knots silently skipped under the old `ORIGIN_TOO_LATE` rule).
- Feed landing-vs-committed, via `python tools/probes/feed_catch_bag_probe.py --bag <bag> --log
  <launch.log>` — same probe as sittings 1-2.

## 5. Stop rules (unchanged from sitting 2 § 6)

Any guard latch or `HAND_LANE_REFUSED` ends the block. A feed miss with a ball on the floor ends
the attempt (clear by hand). `lead_clamp_mask`/`torque_clamp_mask` going non-zero reverts the leg
jerk to 150000 for the rest of the sitting (§ 1 above).

## 6. Pre-registered placement fallback (unchanged from sitting 2 § 7)

If the swapped layout still cannot clear a clean `num_cycles: 6` after a reasonable number of
attempts, the fallback is unchanged: move Ball Butler to a placement ~0.5 m from the cup at the
same height, near-vertical lob (pitch ~84°), re-running the accuracy volley after the move. Not
exercised unless the layout fix itself fails to close the gate — that is a different failure mode
than sitting 2's (which never reached a clean attempt at all).

## 7. Close-out

| # | Step | Expect |
|---|---|---|
| 20 | GUI Deactivate; stop launch and load capture | Robot stows. |
| 21 | `python tools/probes/throw_outcome_bag_probe.py --bag <id>` | Ground-truth check against `skill_node`'s own logged memory rows. |
| 22 | `python tools/probes/feed_catch_bag_probe.py --bag <bag> --log <launch.log>` (final pass) | Per-feed landing table for the whole sitting. |
| 23 | Copy `temp/learn/r5-sitting3-<date>/memory.csv` to a dated path | |
| 24 | Log the sitting (`/log feature`, or `/investigate` if anything in § 3's refusal table fired unexpectedly). Update plan § R5 Outcome. | |
