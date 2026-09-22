---
title: "Cup-contact contract first sitting — τ = 0.125 s holds (87/87 caught); the 8.5 mm frame offset is a lever arm the learner must subtract, not a limit to relax; both hand latches traced (stale encoder bursts, then a park the firmware always rejected)"
type: investigation
date: 2026-09-23
status: tuned
phase: "two-ball-skill-stack — R3"
related_plan: cup-contact-contract.md
sessions:
  - ~/Desktop/rosbags/2026-09-22_23-21-58
  - ~/Desktop/rosbags/2026-09-22_23-40-54
  - ~/Desktop/rosbags/2026-09-22_23-52-57
  - temp/logs/launch_r2gate_20260922_2321.log
files_changed:
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - ros_ws/src/jugglebot/jugglebot/teensy_bridge_node.py
  - ros_ws/src/jugglebot/jugglebot/ARMING_CONTRACT.md
  - tests/ros/test_skill_node.py
  - tests/ros/test_teensy_bridge_node_recover.py
  - tests/hardware/session_cup_contact.md
  - tests/hardware/session_skills_r3.md
  - plans/active/cup-contact-contract.md
  - logbook/INDEX.md
subsystem:
  - motion
  - ros
  - tracking
tags:
  - hardware
  - safety
  - learner
  - mocap
---

# Cup-contact contract first sitting (2026-09-22 evening) — analysed 2026-09-23

## Summary

Block A of `tests/hardware/session_cup_contact.md` flew: 87 self-tosses with a learner row (36 at
0.6 m, 51 at 0.9 m — the 51st is the throw the first latch cut short at a 0.48 m apex),
**87/87 caught**, learner lateral command pinned at 0. The contract's τ = 0.125 s
verdict is **PASS**: seat median +0.089 / +0.079 s (sim predicted +0.063…+0.085; the 09-18
16:16 pre-contract control read +0.086 / +0.091), late seats and HELD/EMPTY/HELD hops no worse
than the control, and the pre-registered stop rule (≥ 3 rebound signatures in the first ten
at either apex) was not reached (0 and 1). Block B never flew: the session-start frame check
refused it at **(−1.56, −8.53) mm** after a QTM recalibration, and the offset is not
calibration noise — it is a lever arm (STOW height × levelling pose offset), stable to
0.03 mm, invariant under relocating the base. The fix is not to relax the 5 mm limit but to
**subtract the measured offset from every tracker landing** at the one point they enter the
skill node, so a landing means "relative to where the cup really is" and the learner cannot
correct a frame offset into the throws. Two hand MAX_DEVIATION latches were traced: the
first to the hand's encoder feedback freezing in ~90 ms bursts during a 112 rev/s throw onset
(the known load-gated frame loss), the second to the recovery's hand park having **never
once succeeded in any log** — the firmware rejects a cold-start ACTIVATE while the stream is
armed — so the recovery now disarms before it parks and holds the state machine in FAULT
until the hand is home.

## Symptoms

Operator's notes (2026-09-22, pasted into the analysis session):

- 0.6 m: every throw caught; catches 7–9 of the chained set "a smidge off positionally";
  the 15+ set had "some very hard landings".
- 0.9 m: every throw caught; "messy at times", some balls "so far off (+x, +y) I thought
  they'd be dropped"; the 15+ set had "very hard catches", one where "the hand moved quite a
  bit after the ball had landed".
- "On the second-last attempt at 0.9 m the Teensy guard latch fired again on the hand"; the
  operator saw nothing wrong with the hand's motion.
- The ball never visibly separated from the cup.
- Block B refused: even after recalibrating QTM, the Platform body read ≈ (−1.0, −8.5) mm from
  the command, "confirmed with the base translated and rotated into different positions
  around the room".

## Diagnosis

Probes: a streaming MCAP reader over `/rigid_body_poses`, `/trajectory/commanded_position`,
`/robot_state`, `/hand_telemetry`, `/balls` and `/mocap_data` (scratchpad scripts, not
promoted — the numbers below are their output), plus `grep` over the launch log.

### 1. The catch quality — τ = 0.125 s (run 2026-09-23 over the three bags + the 09-18 16:16 bag as control)

| | 0.6 m 09-22 | 0.9 m 09-22 | 0.6 m 09-18 ctrl | 0.9 m 09-18 ctrl |
|---|---|---|---|---|
| OUTCOME rows with a seat / caught | 36 / 36 | 50 / 50 | 11 / 11 | 15 / 15 |
| seat median | +0.089 s | +0.079 s | +0.086 s | +0.091 s |
| seat p10 / p90 | +0.066 / +0.097 | +0.067 / +0.244 | +0.070 / +0.250 | +0.070 / +0.213 |
| late seats (> 0.20 s) | 3 | 6 | 3 | 2 |
| HELD/EMPTY/HELD hops (< 0.5 s) | 4 | 5 | 1 | 1 |
| signatures in the first ten | 0 | 1 | 3 | 2 |

(The 0.9 m column excludes the latch-cut 0.48 m throw — caught, seat −0.161 s against a
schedule it never flew — so the seat statistics are 86 rows, the catch count 87.) The nine
hops all coincide with EARLY seats (+0.04…+0.07 s) and last 20–150 ms; the late
seats are the operator's "very hard landings" (three in a row at 0.9 m, +0.246/+0.244/+0.247).
The 09-18 control would have FAILED the first-ten stop rule on late seats alone, so a late
seat is not by itself a rebound; the contract is judged on the hop rate, which is unchanged
within counting noise, and the late-seat rate, which improved. Landing scatter at zero
lateral command (the R3-carried measurement): 0.9 m sd 18.5 / 16.1 mm (x / y), max +63 mm in
x; 0.6 m sd 10.4 / 10.2 mm.

Runsheet § 8 counters from the launch log: `SPLICE_TOO_LATE` 1 (a 173 ms solve),
`AIM-LATERAL-CLAMPED` 91 (authority 0), re-sends 57 accepted / 23 refused (16 `LIMIT_JERK`,
3 `CUP_CONTACT_ACC`, 2 `INFEASIBLE`, 1 `GUARD_LATCHED`, 1 `SPLICE_TOO_LATE`), guard latches 2,
drops 0. The three `CUP_CONTACT_ACC` refusals are re-aims of ≤ 0.06 s whose splice seed
landed mid-dive (a_z −24.6…−29.4 m/s²) with the contact window opening at knot 0 — the gate
doing its job on a re-send that should have been skipped before the solve (Open Questions).

### 2. The frame offset is a lever arm (bags 2 and 3, after the operator's QTM recalibration)

| bag | Base body in QTM | Platform − command (x, y) mm | sd | Platform roll/pitch in QTM |
|---|---|---|---|---|
| 23:21 (sitting, pre-levelled) | (−0.3, +0.3), 0.0035 / 0.0039 rad | (+2.81, −7.04) | 0.64 / 0.53 | −0.0003 / +0.0046 |
| 23:52 (recalibrated, NOT pre-levelled) | (0.0, 0.0), 0.0001 / −0.0001 rad | (−1.56, −8.53) | 0.15 / **0.03** | +0.0136 / +0.0017 |

The QTM frame sits on the Base body to 0.1 mm and 0.0001 rad. The offset is constant to
0.03 mm over 77 s. `config/hardware_config.yaml` `initial_height_mm` = 574.3 and the
platform's persisted levelling pose offset is (0.015, 0.002) rad (`/robot_state`
`pose_offset_rad`; the 23:52 bag's three `level` commands returned 0.0133, 0.015, 0.0207 —
a 0.4° spread between reads): **574.3 × (0.015, 0.002) = (8.6, 1.1) mm**. The IK rotates
the platform about its own centre (`motion/ik_solver.py:161`,
`plat_world = platform_centre + R @ plat_nodes`) and the levelling correction is
rotation-only (`motion/levelling.py::correct_pose`), so the software cannot displace the
centre by tilting it; the mocap `Platform` body's origin sits 7 mm above the commanded
centre (mocap z 177.2 at command z 170), which under 0.0136 rad moves 0.1 mm. What CAN
produce a height-proportional lateral offset is the frame QTM reports in: a rigid body's
axes are frozen at definition time, so if the `Base` body was defined while the global frame
was floor-level and the base plane was ~0.8° from it, the QTM frame is tilted from the
machine's base plane by that fixed angle wherever the base is moved, and a point ~600 mm up
the base axis appears ~8 mm sideways. Two facts fit: in the un-pre-levelled bag the Platform
body reads 0.0136 rad of roll in the QTM frame (the inclinometer's number), and a quadratic
fit of the raw ball markers over 35 flights (z residual sd 2.0 mm) shows the QTM z-axis
0.0066 ± 0.0005 rad from gravity in y (+62 mm/s²) — neither gravity-level nor base-plane
locked. Sign conventions could not be pinned from the bags alone, so the mechanism is
strongly indicated, not proven; the z-sweep in the runsheet (§ 9) settles it on the rig.

Whichever frame is "right", the consequence for the learner is the same: the commanded site
is in the base frame, the tracker's landing in the QTM frame, and the cup is physically at
command + offset in the QTM frame. A learner with authority would aim the landing at the
commanded site — 8.5 mm from the cup. Today's rows (authority 0) already carry it: the
landing means were (+6.8, −12.2) mm at 0.6 m and (+8.3, −15.6) mm at 0.9 m, i.e. real
misses of about (+4, −5) and (+5.5, −9) mm once the platform's own (+2.8, −7.0) is removed.

### 3. The first hand latch, 23:34:42.5 (`dev=+3.261 rev at trip`)

`/hand_telemetry` around the trip: `pos_meas` frozen in eight consecutive 90–100 ms runs
from −0.76 s to −0.02 s before the latch (the bridge's "hand heartbeat dropout episode
ENDED: 795 ms stale, 23 encoder frames co-dropped" logged 0.4 s earlier), while the command
ran the catch dive at −45 rev/s and then the throw onset at +112 rev/s. The firmware's guard
compares the raw command against the age-extrapolated encoder (`leg_interp.cpp`, age capped
at 150 ms) — at 112 rev/s the 2.5 rev budget is consumed in 22 ms of feedback age, and the
extrapolation used the stale +1.08 rev/s. Nothing was wrong with the hand's motion; this is
the known load-gated leg-bus frame loss (`plans/active/leg-bus-frame-drops.md`) reaching the
hand at the one instant it cannot be absorbed. The heartbeat dropout counter, not the guard,
is the observable.

### 4. The second hand latch, 23:35:14.0 (`dev=+7.321 rev at trip`) — the park that never worked

Log sequence: `armed /clear_errors` → `_svc_recover` converged and fired CLEAR_ERRORS →
`hand park: ACTIVATE(axis 6) TRAP_TRAJ from +5.5694 rev` → **`ACTIVATE: ERR_REJECTED`** →
"HAND NOT PARKED … DEACTIVATE then ACTIVATE" → fallback disarm + direct clear → the operator
started the next schedule without the DEACTIVATE/ACTIVATE. The opening REST homed the hand
5.57 → 0.31 rev at 2.5 rev/s, but a lane re-entering after a disarm is bound by the
firmware's 1 rev/s `RECOVER_SLEW_VEL_RPS` (telemetry shows the transmitted command
descending at exactly −1.00 rev/s), so at the THROW the hand was still 1.2 rev behind; the
throw streamed into a refused lane and the guard read the refused command 7.3 rev from the
encoder. Root cause of the rejection: `leg_activate.cpp:129` — `if (fault_mpc_active())
return ERR_REJECTED` — the MPC-stream interlock refuses any cold-start op while the wire is
armed, and `/recover` deliberately keeps the wire armed. The 2026-09-18 wait-for-guard-clear
fixed the ERR_BUS_DOWN race but not this: `grep "hand parked ("` over every launch log finds
**zero** successes against four rejections (2026-09-18 ×2, 2026-09-22 ×2).

## Discussion

**Hypothesis withdrawn: "relax the frame limit" / "the platform moves back in y".** The
operator's two framings were both reasonable and both wrong in the same way — they treat the
offset as either noise or a plant defect. A 0.03 mm-stable number equal to height × levelling
tilt is neither; it is a frame-definition difference, and the right response is to measure
it and subtract it, which works whether the QTM frame or the base plane turns out to be the
tilted one. Raising the limit to 10 mm would have let the learner spend Block B moving the
landing 8.5 mm off the cup.

**Why subtract in the skill node, not in the tracker or the executor.** The tracker is
shared with the Ball Butler path and reports in the mocap frame by contract; the executor is
pure Python and should not know about ROS topics. `_on_balls` is the single point where a
`/balls` message becomes a `Landing` that both the catch aim and the learner outcome read —
one enforcement point, so the OUTCOME lines, the memory rows and the AIM-LATERAL lines can
never disagree about the frame. The offset is adopted only from an evaluable, in-bound
measurement; a later bad measurement keeps the earlier correction and warns (a stale good
number beats none). The check's limit becomes a 25 mm sanity bound (the 2026-09-18 +30/−50 mm
cases were wrong alignments, not lever arms) plus a 2 mm stability requirement on the
Platform body.

**Why the recovery disarms rather than the firmware learning to park under an armed stream.**
A firmware carve-out ("allow ACTIVATE(6) while armed if the hand lane is dead") is a second
interlock semantic to keep consistent with the lane state machine, on a flash-gated path.
Disarming is already what the armed `/clear_errors` fallback does and is safe by the same
argument: at `mpc_active=0` the guard terms are inert and the ODrives hold every axis. The
cost is that the wire must be re-armed after the park — which the orchestrator's A2 phase
already does on FAULT → ACTIVE (it did exactly that at 23:35:03 after the fallback disarm).
The one race this opens — the orchestrator arming 200 ms after the clear, mid-park, and
exhausting its ten attempts — is closed by publishing `fault_state=RECOVERING` while the park
runs: the orchestrator stays in FAULT (guard-only, control mode preserved), the trajectory
node stays at its measured hold, and the NONE edge that follows re-arms and reseeds as a
plain clear would. The A1 pre-check also refuses by name during the park. The trajectory
node's hold is hand-less after a latch (descent/hold plans carry no hand track), so A1 does
not compare the hand and the next opening REST plans the hand from its measured, parked,
position.

**What was accepted, not fixed.** The first latch's cause (frame loss at the hand) is the
open leg-bus-frame-drops plan; the recovery fix removes its consequence, not its cause.

## Fix

- `skill_node.py`: `_FRAME_CHECK_LIMIT_MM` 5 → 25 (sanity), new
  `_FRAME_CHECK_PLAT_SPREAD_MM` = 2.0 (Platform body bounding-box diagonal inside the
  window; over it the check refuses to evaluate); `FrameCheckResult.plat_spread_mm`;
  `_frame_check_error` adopts an evaluable in-bound offset as `_mocap_to_schedule_mm` and
  logs "tracker landings are now corrected by (x, y) mm"; `_on_balls` subtracts it from
  every landing's xy (z and velocity untouched).
- `teensy_bridge_node.py`: `_park_hand` refuses on an armed wire from every caller (the
  `lane_cleared` assertion is gone — it was true and irrelevant); new `_park_hand_disarmed`
  (stop output → confirm disarmed on the wire → park → leave disarmed) used by both
  `/recover` paths and, new, by the armed `/clear_errors` fallback; `_recovery_park_in_progress`
  publishes `fault_state=RECOVERING` (`_published_fault_state`) and is A1 precondition (a4).
- Tests: 6 new bridge tests (disarm-then-park with an ACTIVATE-time arm-state spy, armed
  `_park_hand` refusal, fallback park, RECOVERING only during the park, A1 refusal, disarm
  even on an already-parked hand) and the
  wrong-premise "still parks an armed wire" test rewritten; 6 new skill-node tests (moving
  body refused, spread reported, the sitting's own (−1.56, −8.53) adopted, `_on_balls`
  subtracts xy only, pass-through before any measurement, a bad later check keeps the
  earlier correction).
- Runsheet: rows 8/9/15 re-worded for the new check semantics; new § 9 z-sweep block.
- `temp/learn/jugglebot/memory.csv` (the 87 rows, raw mocap frame) quarantined to
  `temp/learn/_quarantine_20260922/` — they are inconsistent with corrected rows.

## Outcome

- Scoped tests (run 2026-09-23): `python -m pytest tests/ros/test_skill_node.py
  tests/ros/test_teensy_bridge_node_recover.py -q` — **133 passed in 13.32 s**.
- Default gate (run 2026-09-23, after the phase-end audit's fixes): `./run_tests.sh` —
  **6360 passed, 9 skipped in 222.23 s; serial tail 3 passed; RESULT: PASS**
  (`temp/logs/gate_20260923_b.log`). No `controller/` or `sim/` file changed, so the full
  tier is the operator's pre-sitting step (runsheet row 5), not owed here.
- Not yet flown. The next sitting (runsheet § 9, then Block B) is the verification of both
  fixes on hardware: the frame check should now read an adopted offset and the first
  recovery after a latch should end with "hand parked … wire DISARMED" and an automatic
  re-arm. `colcon build` is owed before it.

## Withdrawn claims

- *"The latch fired once, on the second-last 0.9 m attempt"* (operator) → two latches, 32 s
  apart; the second, on the next attempt's first throw, was a consequence of the first.
- *"Maybe the platform is moving back in y … the levelling offset?"* (operator) → the
  platform centre is where the kinematics say; the levelling angle enters only through the
  frame the offset is measured in (Diagnosis § 2).
- *"The recovery path passes `lane_cleared=True` and MUST still park while armed: the latch's
  own edge killed the lane, so there is nothing to race"* (bridge docstring and test,
  2026-09-18) → the lane was dead; the firmware's interlock reads the arm state, not the
  lane, and rejected every such park.

## Open Questions

- Which frame is tilted: run the runsheet § 9 z-sweep (offset vs commanded z at 0 / 100 /
  170 / 250 mm) — a lever arm changes by ≈ 1.4 mm per 100 mm, a translation does not. If it
  is the Base body definition, redefining it with the base plane's normal removes the offset
  at the source (the subtraction stays as the fail-safe).
- The raw-marker fit puts QTM's z-axis 0.0066 rad from gravity in y. The batch fit
  (`tracking/flight_fit.py`) assumes gravity along −z; over a 0.86 s flight the unmodelled
  +62 mm/s² biases a whole-flight linear-y fit by ≈ a·T²/12 ≈ 4 mm and early predictions by
  more. Worth fitting a free lateral acceleration once, then either correcting the frame or
  the model.
- Re-sends whose splice knot lands inside or one knot before the contact window should be
  skipped before the solve (3 `CUP_CONTACT_ACC` refusals of ≤ 0.06 s re-aims), and 16 of 23
  refusals were still `LIMIT_JERK` (57 accepted) — better than 18 of 21 on 09-18, not rare.
- The levelling reads spread 0.4° across three consecutive `level` commands (0.0133 →
  0.0207 rad) in the 23:52 bag; the inclinometer's repeatability is worth a bench number.
- The first latch's mechanism (hand encoder frames dropped for 0.76 s at a throw onset) is
  the leg-bus frame-drops plan's; the guard's extrapolation cannot cover a stroke onset with
  a stale velocity, so instrumenting the drop (per-axis frame age at the trip, in the
  latch snapshot) remains the next observability step.
