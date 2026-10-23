---
title: "Held-axis reload catch refuses every tracker re-aim (CATCH_AXIS at 0.1 mm), and an ended attempt leaves the platform tilted"
type: investigation
date: 2026-10-19
status: resolved
phase: "two-ball-skill-stack - post-R6 (columns empty-cup two-feed, row 33 sitting)"
related_plan: columns-empty-cup-two-feed.md
related:
  - 2026-10-17-columns-cup-keyed-accept-lob-retired.md
files_changed:
  - ros_ws/src/jugglebot/jugglebot/motion/trajectory/cup_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/unified_cycle.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/executor.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/segments.py
  - ros_ws/src/jugglebot/jugglebot/motion/skills/INVARIANTS.md
  - ros_ws/src/jugglebot/jugglebot/skill_node.py
  - sim/skills_gate.py
  - tests/motion/test_cup_capture.py
  - tests/motion/test_skills_capture_replay.py
  - tests/motion/test_cup_cycle.py
  - tests/motion/test_skills_executor.py
  - tests/ros/test_skill_node.py
  - tests/fixtures/capture_move_replay_2026-10-19.yaml
  - tests/fixtures/capture_move_replay/
  - tests/hardware/session_skill_stack_rehearsals.md
  - tools/probes/capture_move_replay_extract.py
  - tools/probes/capture_move_sim.py
  - tools/probes/README.md
  - plans/archived/columns-empty-cup-two-feed.md
subsystem:
  - motion
  - ros
tags:
  - safety
  - testing
---

# Held-axis reload catch refuses every tracker re-aim, and an ended attempt leaves the platform tilted

## Summary

On the 2026-10-19 morning sitting (rehearsal row 33, columns empty-cup two-feed) the owner pressed Stop during the refill wait, then two consecutive BB-fed reload catches ENDED with `CATCH_AXIS` (28.3 mm, then 20.8 mm) and the platform sat visibly tilted for about 70 s. The session's first reading was that the Stop had left the platform displaced and tilted. The bag says otherwise: the platform sat level at the feed site for 63 s after the Stop, and the tilt began 3 s into the next attempt. The real cause is a planner tolerance: a held-axis CATCH is seeded on the line through the ANNOUNCED landing and then checked against the line through the REFINED (tracker) landing with a 0.1 mm seed tolerance, so it admits no lateral re-aim at all. Earlier reloads passed only because the tracker fit lost a ~20 ms race and the refused re-aim hit a non-fatal re-send path. Second, separate defect: an attempt that ENDs after the pre-tilt REST leaves the platform at the tilted terminal indefinitely. Status in-progress: diagnosed; the fix chosen by the owner (in-window capture move plus a decay-to-level on END) is built, audited and gate-tested the same day (see Fix and Verification) and waits on hardware rehearsal rows 39-42 before this entry resolves.

## Symptoms

Artefacts: launch log `~/.ros/log/2026-10-19-11-04-03-078330-jetson-3068207/launch.log` (cited `LL:<line>`); bag `~/Desktop/rosbags/2026-10-19_11-04-03` (`/rigid_body_poses` Platform body, `/trajectory/commanded_position`, `/balls`). Installed code = `Jugglebot-skills` worktree HEAD 84519014 (built 10:35, byte-identical to source for `skill_node.py` and every `motion/skills/*.py`); it differs from the oval worktree (40b38556) only by the Phase 3 robot-frame change, which is not implicated (the world-to-robot snapshot is near-identity every attempt, e.g. LL:1061 "origin (-0.1, 0.6, 1.1) mm, yaw -0.016 deg"). Times are local, launch 11:04:03 = epoch 1792368243 (`t=519.149` below means 1792368519.149).

Sitting outcomes: self-toss 4/4, hop 4/4, row 32 columns two-feed 3/4 (the last throw missed, ball landed on the platform).

| local | epoch | event | evidence |
|---|---|---|---|
| 11:05:45 | 345.4 | self_toss (reload) accepted; pre-level go_to_pose x=-0.5 (current xy held) | LL:384-388 |
| 11:05:48 | 348.57 | reload CATCH dispatched source=schedule (-50,0,830), fresh-origin install; re-aim RESEND-REFUSED CATCH_AXIS 12.0 mm; schedule install kept; 4/4 caught | LL:417-422, 520 |
| 11:06:34 | 394.8 | columns (reload) accepted; pre-level x=-125.5; pre-tilt REST carries -122.7 -> -46.1 mm, 0.5 -> 10.2 deg | LL:677-681; bag 395-398.2 s |
| 11:06:38 | 398.18 | CATCH source=schedule installed; RESEND-REFUSED CATCH_AXIS 18.0 mm; ball 1 caught | LL:715-719, 727 |
| 11:06:44-51 | 404.5-411 | ball 2 fed (hop entry), 3/4 caught, throw 4/4 MISSED after a LIMIT_JERK + C-CUP-4 re-aim refusal | LL:740-841 |
| 11:07:29 | 449.2 | columns (reload) accepted; pre-level x=62.0 (platform was at +65.7 mocap); pre-tilt REST carries it +65.7 -> -45.6 mm, tilt 10.2 deg | LL:913-926; bag 450.9-452.6 s |
| 11:07:32 | 452.53 | CATCH source=schedule installed; RESEND-REFUSED CATCH_AXIS 24.9 mm (aim clamped from -36.4 mm y); ball 1 caught | LL:944-948, 955 |
| 11:07:33 | 453.37 | decay REST (skill 2, 1.25 s) installed: cmd x -39.4 -> -63.0, level | LL:952-954; bag cmd 453.9-454.9 |
| **11:07:34** | **454.22** | **Stop** during the refill wait: executor-less branch, "nothing else to command" | LL:957-962; `skill_node.py:5694-5708` |
| 11:07:34-11:08:37 | 454.8-517.4 | platform LEVEL at the feed site: mocap x -60.0 y 1.2, tilt 0.50-0.53 deg (this body's level baseline; 0.48 deg at 345.7); cmd (-63.0, 0.1, 170.0) | bag |
| 11:08:35 | 515.78 | columns (reload) accepted; pre-level go_to_pose x=-63.0 y=0.1 z=170.0 over 0.20 s (no-op: already level and there) | LL:1064-1068 |
| 11:08:37-39 | 517.43-519.1 | pre-tilt REST (skill 0, 1.75 s) carries -60.0 -> -46.2 mm, tilt 0.5 -> 10.2 deg; BB throws (bb/throw OK 519.045) | LL:1084-1095; bag |
| 11:08:39 | 519.124 | first converged fit for BB ball 17 in `/balls`: landing (-94.1, -20.6) world | bag `/balls` |
| **11:08:39** | **519.149** | CATCH fresh-origin dispatch aimed at the TRACKER fit (AIM-LATERAL-CLAMPED -34.4 mm x); install refused **CATCH_AXIS 28.3 mm**; no prior install, attempt **ENDED** | LL:1097-1100 |
| 11:08:39-11:09:49 | 519.1-589.2 | platform held TILTED 10.18 deg at x -46.5 (cmd -48.8, 1.0) for 70 s: the pre-tilt REST's terminal, never un-tilted | bag (520, 560, 587-589.2 s) |
| 11:09:49 | 589.41 | reload accepted; pre-level go_to_pose x=-48.8 y=1.0 z=170.0 over 0.22 s: it DID level (10.18 -> 0.54 deg by 589.72), xy held | LL:1213-1219; bag |
| 11:09:50-52 | 589.7-592.8 | pre-tilt REST re-tilts -45.9 -> -33.5 mm, 0.5 -> 10.4 deg (cmd -48.8 -> -36.1) | bag |
| **11:09:52** | **592.80** | first fit 592.771; CATCH fresh-origin dispatch tracker-aimed (AIM-LATERAL-CLAMPED -24.3 mm y); refused **CATCH_AXIS 20.8 mm**; ENDED | LL:1248-1253 |
| 11:09:52-11:11:08 | 592.8-668 | platform held TILTED ~10.4 deg at x ~-33 (one sample 11.2 deg at 600 s) | bag (600, 640 s) |
| 11:11:08 | 668.15 | operator go_home -> level, then disarm/stow (z 1.2 mm from 678 s) | LL:1414 (go_home), LL:1452 (stowed 11:11:18) |
| 11:12:37 | 757.66 | skills/check: REJECTED_NO_BALL (cup EMPTY) + REJECTED_FRAME_OFFSET (commanded_position stale 79 s, robot stowed) | LL:1556-1559 |

The 11:12:37 FRAME_OFFSET refusal is correct behaviour (stale command after the stow), not a defect; it rides along with the empty-cup refusal. No REJECTED_FRAME_OFFSET occurs anywhere else in the session.

## Diagnosis

Paths below are under `ros_ws/src/jugglebot/jugglebot/` in the `Jugglebot-skills` worktree (`S/`).

### Defect 1 (the refusal): a held-axis reload CATCH cannot take ANY lateral re-aim, and a tracker fit that lands before the dispatch makes the dispatch itself the re-aim

1. Compile pins the start. `schedule.compile_reload` gives the PRE-TILT REST (skill 0) a `rest_site_mm = segments.hold_axis_site(announced landing, hold_tilt, z)`, a point on the held-axis line through the ANNOUNCED landing (`S/motion/skills/segments.py:300-320`; abut/lead layout `S/motion/skills/schedule.py:1690-1706`). That REST is a planned rest-terminal transit from the commanded state, so it already carries the platform from wherever it is into the pose (11:07:29: 111 mm and 10.2 deg in 1.75 s, no refusal).
2. The CATCH seed is that REST's terminal. trajectory_node seeds a fresh install from the active plan's commanded state at t_now (`schedule.py:1693-1697`; "fresh origin" at LL:1097).
3. The CATCH site is whatever `_catch_aim` returns at dispatch (`S/motion/skills/executor.py:3055-3105`). Rank 1 is a converged tracker fit (`from_fit`), rank 2 the prior (announced/schedule landing). The fit is clamped laterally to the authority, 20 mm per axis on a one-site/reload catch (`_clamp_lateral_to_schedule`, `executor.py:2510-2623`).
4. `_catch_terminal` re-derives only the CATCH's TERMINAL rest from the refined landing (`executor.py:2747-2760`). The SEED stays on the announced line.
5. The planner checks the seed against the line through the refined site: `S/motion/trajectory/cup_cycle.py:990-1008` (`_axis_offsets(state0.pos, ..., site, kappa)`) against `HELD_AXIS_SEED_TOL = (1e-4 m, 1e-3 m/s, 1e-2 m/s^2)` (`cup_cycle.py:866`), i.e. **0.1 mm**. Message: "its resting position is X mm off the line". "Resting position" is the CATCH seed (pre-tilt REST terminal, on the announced line); "the line" is the held cup axis through the clamped, tracker-refined landing. The offset is the clamped lateral re-aim: 28.3 mm = sqrt(20^2 + 20^2) (both axes at the 20 mm clamp: fit -34.4 mm x and about -21 mm y after the frame correction); 20.8 mm is about (+5, -20); the earlier 12.0 / 18.0 / 24.9 mm likewise. Any re-aim over 0.1 mm refuses. This is by construction, not a marginal limit. `held_from_k > 0` (swing onto the line mid-window) is refused explicitly as "not built yet" (`cup_cycle.py:979-985`).
6. What decides END versus survive is whether a converged fit exists at the CATCH dispatch, about 0.775 s before landing (0.5 window + 0.225 lead + 0.05 look-ahead). The fit converges about 0.1 s after release, i.e. about 0.78 s before landing for BB's 0.88 s flight, so the two coincide to tens of ms:

   | ball | first fit in bag | CATCH dispatch | aim | outcome |
   |---|---|---|---|---|
   | 1 (11:05:48) | 348.597 | 348.574 | schedule | installed; resend refused; caught |
   | 10 (11:06:38) | 398.151 | 398.177 | schedule | installed; resend refused; caught |
   | 16 (11:07:32) | 452.514 | 452.527 | schedule | installed; resend refused; caught |
   | 17 (11:08:39) | 519.124 | 519.149 | tracker (clamped) | dispatch refused, END |
   | 18 (11:09:52) | 592.771 | 592.800 | tracker (clamped) | dispatch refused, END |

   Balls 10 and 16 show a bag-stamped fit 13-26 ms before the dispatch yet aimed at the schedule, so the exact threshold is not recoverable from bag `log_time` (skill_node's `/balls` receipt time is not recorded). What is certain for 17 and 18 is the AIM-LATERAL-CLAMPED note co-located with the refusal (LL:1098, 1251) and no accepted `install_segment CATCH` before it: the dispatch landing was a clamped tracker fit. Nothing about the Stop changes this race; the two post-Stop attempts lost it, the three pre-Stop ones won it.
7. With no CATCH ever installed the executor has no fallback and ENDs `CATCH_AXIS` (LL:1099-1100, 1252-1253). Pre-Stop, the identical refusal hit the RE-SEND, which is non-fatal: the schedule-aimed install kept streaming and caught balls that landed 20-38 mm off.

Why the earlier reloads passed: not a different rest pose. In all five attempts the platform reached the same kind of pose (level start anywhere, pre-tilt REST to about -46 mm / 10.2 deg on the announced line). The difference is the aim source at the CATCH dispatch (table above). The later tracker re-aim hit the same CATCH_AXIS wall (12.0 / 18.0 / 24.9 mm, LL:422, 719, 949) but as a non-fatal RESEND-REFUSED. Consequence: no BB-fed reload catch today was laterally re-aimed at all. The held-axis catch is effectively open-loop on the announced landing, and it still caught 3/3.

Side finding, misleading wording: the AIM-LATERAL-CLAMPED note says "catch keeps the schedule's site", but the catch is moved by up to the authority (`executor.py:2614-2618` against `2619-2623`). That wording made the post-Stop refusals look like schedule-aimed ones.

### Defect 2 (what the owner saw): an attempt that ENDs after the pre-tilt REST leaves the platform tilted about 10 deg indefinitely

- On END, `_maybe_hold_pending_event` (`S/skill_node.py:2410-2450`) acts only when the streaming plan still carries a future event (or a forced lane hold); otherwise "whatever is streaming already ends at rest", here the pre-tilt REST's terminal, which is a TILTED rest on the hold axis. No decay REST is installed. Result: 10.2 deg held 519.1 -> 589.2 s and about 10.4 deg 592.8 -> 668 s (bag), until the operator's go_home.
- The per-attempt pre-level is a pure attitude move holding the live commanded xy/z (`S/skill_node.py:4219-4258`, `orientation=Quaternion()`, `duration_s=0.0`). It ran both times: at 11:08:35 it was a no-op (already level); at 11:09:49 it DID level the platform (10.18 -> 0.54 deg in under 0.5 s, bag 589.22 -> 589.72) and kept xy at -48.8. The platform then re-tilted because the new attempt's own pre-tilt REST moved it into the next hold pose. So "stayed tilted through both attempts" is accurate between attempts (Defect 2) and during each attempt (by design); it is not that the pre-level failed or that the Stop held a pose.
- Bag evidence that the Stop is innocent: after the 11:07:34 Stop the decay REST finished and the platform sat level at the feed site for 63 s (454.8-517.4 s, tilt 0.50-0.53 deg). The tilted pose is real but began at 11:08:39, after the first refused attempt.
- A Stop during the pre-tilt REST or the CATCH window would leave the same tilted terminal (same "nothing else to command" reasoning, `skill_node.py:5694-5708`), so the Defect 2 fix must cover Stop too.

### Other observations (not diagnosed further)

- LIMIT_JERK re-aim refusals are routine on every pattern: 6 RESEND-REFUSED LIMIT_JERK lines this session, 214k-313k against the 200k leg-jerk cap (LL:452, 573, 590, 607, 622, 823). Re-aims after a fit mostly cannot be afforded; catches survive on the original install.
- Throw 4/4 MISSED at 11:06:51 (LL:840): preceded by a re-aim refused LIMIT_JERK 313k plus C-CUP-4 33.4 deg off the cup axis (LL:822-823); landed +3/-6 mm, release +5 ms, seat -0.09 s. Worth one look in the next analysis.
- The pre-level from 10.2 deg completed in 0.22 s (LL:1213-1214), about 46 deg/s average attitude rate; accepted by go_to_pose's profile, but it only arises because of Defect 2.
- `mocap_node` reported Robot frame UNKNOWN (RMS_OVER_CAP / TILT_FROM_BUILD) repeatedly after 11:10 (LL:1270-1354); no attempt ran then. The 11:13/11:14 launches were skipped per owner (unrelated QTM/frame work).

## Discussion

### (a) Hypothesis withdrawn: "the Stop left the platform displaced/tilted"

The session's first framing, from the owner's report, was that the 11:07:34 Stop left the platform displaced and tilted, and that the reload compile assumes the rest pose lies on the ball's axis, so the next attempt began from a pose that violated that assumption. That did not survive the bag: the platform was level at the feed site (tilt 0.50-0.53 deg, this body's level baseline) for 63 s after the Stop, and the tilt began 3 s into the next attempt, as the pre-tilt REST's own terminal. The real cause is the re-aim tolerance (Defect 1); the tilted hold and the odd xy were consequences of an ENDed attempt (Defect 2), not causes of it. The pre-tilt REST is also already a planned transit from the commanded state, so "move from whatever pose it is in into the feed pose" was already satisfied for the START pose. What is missing is the move from the pre-tilt pose onto the REFINED line once the tracker knows where the ball lands. Recorded under Withdrawn claims.

### (b) Fix shapes weighed, and what the owner chose

1. **Pre-position before the throw request: rejected.** Moving to the feed pose before requesting `bb/throw_at_target` adds the transit (1.75 s today) ahead of BB's 2.5-3.0 s countdown, lengthening every feed, and it does not address this failure: the pre-tilt REST already reaches the announced-line pose before the ball is thrown. The mismatch is with the refined landing that only exists about 0.78 s before touch-down.
2. **Interim: re-dispatch on the schedule (prior) landing when a tracker-sourced fresh-origin CATCH is refused CATCH_AXIS: declined by the owner.** It was the smallest change (the exact install that caught 3/3, one extra refusal of about 2-5 ms plus a 40-55 ms solve inside the 150 ms fresh budget; LOW risk) but it leaves the feed catch open-loop laterally, as it de facto was all session. The owner chose to go straight to the capture move rather than ship an interim and re-fly row 33 on it.
3. **CHOSEN: build the in-window capture move, plus the Defect 2 decay.**
   - Start state. The plan starts at the COMMANDED state (the pre-tilt REST terminal: pose, tilt, zero velocity and acceleration), never the measured `/robot_state` or mocap pose. The Teensy `MAX_DEVIATION` guard measures deviation from the streamed command, so a measured-pose seed would put a step (tracking error plus mocap noise, about 5 mm frame offset today) into the first knot. What changes is only that the seed is no longer required to lie on the refined line.
   - `CATCH_AXIS` is evaluated at the hold-start knot k_h, not at the seed: the plan translates (tilt held, no ball contact yet) during knots 0..k_h, from k_h to touch-down the slaved held-axis rows apply, and the check asserts the PLANNED state at k_h is on the refined line in position, velocity AND acceleration (`HELD_AXIS_SEED_TOL` applied at k_h). This is exactly the `held_from_k > 0` branch `cup_cycle.py:979-985` refuses as "not built yet", with `cup_realize.py:1205`'s matching refusal; `_catch_terminal` must re-derive the seed-side site symmetrically (today only the terminal rest, `executor.py:2747-2760`).
   - One-cycle control walk-through (25 ms knots, 500 Hz Teensy interpolation). Knot 0 is the commanded seed: no step, vel = acc = 0 matches the REST terminal, so feedforward is continuous at the splice. Knots 1..k_h: lateral min-jerk of d up to 28.3 mm (the authority diagonal). Over 0.25 s: peak 210 mm/s, about 2.6 m/s^2, cup jerk about 1.1e5 mm/s^3; leg jerk will be higher, and today's re-aims already refuse LIMIT_JERK at 214k-313k against 200k. Over 0.40 s the jerk drops about 4x (about 2.7e4), hence the **capture leg is spread to about 0.4 s to stay under LIMIT_JERK**. At k_h the lateral velocity and acceleration must equal the slaved values or the acceleration feedforward has a discontinuity (a jerk spike the gate refuses, or worse, flies).
   - Time budget: no change to BB's countdown (2.5-3.0 s) and nothing before `bb/throw_at_target`; the capture lives inside the existing CATCH window (0.5 s + 0.225 s lead). The fit arrives about 0.78 s before landing, so a capture planned at the dispatch or first re-send has about 0.5-0.7 s; solve budgets (150 ms fresh, 250 ms splice) are unchanged. Translating at a held tilt with no ball in the cup is harmless; the hold matters only from just before touch-down, which is why k_h can sit 3-6 knots before it.
   - Risk HIGH: design-bearing planner change (new QP span, new INVARIANTS row for CATCH_AXIS-at-k_h), needs sim proof and LIMIT_JERK characterisation.
   - Defect 2: an attempt that ENDs after the pre-tilt REST (and a Stop in that window) installs the reload's own decay REST to level, scheduled **after the announced landing plus a margin**. At the refusal (519.149) BB's ball is still in the air (landing about 519.9); un-tilting under a falling ball moves the cup out from under it. Risk LOW-MEDIUM; this is choreography, and mocked-ROS tests are blind to it, so it needs a dress rehearsal before it goes on a runsheet.

### (c) Root-cause framing: the class this closes

The class is "a plan pinned at compile time on an announced landing, then re-aimed at dispatch against a tolerance that admits no re-aim". Here the pinned quantity is the held-axis seed (the pre-tilt REST terminal on the announced line), the re-aim is the tracker's refined landing, and the tolerance is `HELD_AXIS_SEED_TOL` at 0.1 mm. The failure is not one value: every re-aim of a held-axis catch refuses by construction, and which failure mode you see (non-fatal RESEND-REFUSED, or fatal END at a fresh-origin dispatch) depends on a race between tracker convergence and the dispatch. The invariant that should close the class: **a plan's pinned geometry may be re-aimed up to the authority without being refused, because the plan carries the capture move that absorbs the re-aim; CATCH_AXIS is checked where the held-axis rows begin (position, velocity and acceleration at k_h), never at the seed.** Landing it should follow the contract pattern: the INVARIANTS.md row (normative), one enforcement point (the planner's k_h check), and one test that fails if a clamped re-aim of the maximum authority is refused. The same discipline applies to the END path: no ENDed attempt may leave an actuated, non-rest-at-level terminal streaming (Defect 2).

## Fix

Status: BUILT, not committed, not flown. Gate and hardware rehearsal results are under Verification (Pending). Paths below are under `ros_ws/src/jugglebot/jugglebot/` unless noted.

### Defect 1: the capture move (planner)

`motion/trajectory/cup_cycle.py`. A held-axis catch that ends at rest (`CatchEvent.capture`, set by `unified_cycle._events_for` for every LANDING with a hold tilt and no carried throw) and whose seed is off the refined line no longer refuses at the seed. The plan starts at the COMMANDED state of the installed plan (knot 0, or the splice knot of a re-send), never a measured pose, so the Teensy `MAX_DEVIATION` reference stays continuous at the splice. Three constraints replace the seed check, all at the hold-start knot k_h (`contact_window(...)[0]`, the first knot the ball may be in the cup): position, velocity and acceleration of the line residual `dp`, `dv`, `da` within `HELD_AXIS_SEED_TOL` (0.1 mm, 1 mm/s, 10 mm/s^2). Between knot 0 and k_h the platform translates at the held tilt; from k_h on, the existing slaved held-axis rows apply. Held STEADY windows (a carried throw) and a window without the flag still refuse an off-line seed exactly as before (`C-CUP-5`).

Pre-solve gate (`_capture_gate`). Two constants and one formula decide, before any solve, whether the slide is affordable:

- `CAPTURE_MAX_OFFSET_M = hypot(0.035, 0.020)` = 40.31 mm, a cap on the 2-norm of the per-axis line residual. Derivation: the widest re-aim the executor's clamps can emit is 35 mm inward along the two-site axis (`catch_inward_authority_mm`) with 20 mm across (`learner_lateral_authority_mm`). The first design said 20 mm per axis; the owner decision replaced it with the norm so a two-site inward 35 mm re-aim is not refused. A diagonal outward unit of a two-site schedule could reach hypot(35, 20 sqrt 2) = 45.0 mm and would be refused; every flown pattern has its sites on one axis. A pin test (`tests/ros/test_skill_node.py`) fails if the node's declared authorities and this constant drift apart.
- `CAPTURE_MIN_KNOTS = 3`, the rank floor of the three constraints.
- Knot formula. A minimum-jerk slide of size e0 over k_h knots of step dt has peak jerk `6|e0| / (k_h (k_h - 1) dt^3)`. The gate takes `K_need` = the smallest k >= 3 with that peak at or below `J_cap` (`CupCycleConfig.capture_jerk_mps3`, filled from the live leg jerk limit, 200 000 mm/s^3, by `plan_cycle`; else `max_jerk_xy`), and refuses when `k_h < K_need`. At dt = 25 ms and 200 m/s^3 this gives `k(k-1) >= 54.3` for the 28.3 mm diagonal clamp, so a minimum k_h of 8 (200 ms), and `k(k-1) >= 77.4` for the 40.31 mm cap, so a minimum k_h of 10 (250 ms). A dispatch 0.725-0.775 s before landing has k_h 15-17 (closed-form peak 57-74 m/s^3 at the cap, 28-37 % of the 200 m/s^3 limit), so a first dispatch always passes; the knot refusal bites on a late SECOND re-aim spliced into a capture already in flight.
- `CAPTURE_JERK_LEG_FACTOR = 1.0`. The gate budgets the PLATFORM jerk itself. The realised peak leg jerk was measured at 0.36-0.45 of that closed form (max 0.452 at (-20,-20), k_h 17; test pinned at <= 0.5), so the factor never exceeds 1 and the margin is at least 2.2x.

Refusals keep the reason `CATCH_AXIS` and name the slide (stable substrings `slide`, `sideways onto the ball's line`, `past the`, `steps are left`). A cap or knot refusal is a pre-solve arithmetic check, not a solve; a post-solve check at k_h guards a solver defect only (refused rather than flown).

Objective and jerk box on the capture knots. Both restated, because measurement showed the first formulation did not do what it appeared to: the cup QP's lateral term penalises the CUP xy acceleration `a_xy = a_e + kappa a_z`, which on a freed span lets the platform cancel the stroke's lateral projection. The platform then swung under the stroke: a 5 mm re-aim gave the same peak leg jerk as a 28 mm one (77k mm/s^3 against 34 mm/s at k_h 15, and a peak |j_e| 186 m/s^3, 20x the closed form), growing with k_h. On the capture knots the term is therefore `(1+w)(a_e^2 + kappa^2 a_z^2)`, the platform's OWN acceleration; on the held span (a_e = 0) this is knot for knot today's value, so the on-line program is unchanged (byte-identity test). The lateral jerk box row on knots 0..k_h-1 is on `j_e = j_xy - kappa j_z` for the same reason: a `j_xy` box at 300 m/s^3 would cap the stroke's own projection (measured |j_xy| 180-600 m/s^3 on the span), the mis-attribution the held-span comment already removed. After both changes the solved peak `|j_e|` equals the closed form (ratio 1.000 on 30 cases: offsets (5,0) (20,0) (+-20,+-20) (35,20) (5.6,-20) mm against k_h in {8,10,12,15,17}).

Measured at live limits 600/5000/200k (probe in the scratchpad, promoted as the replay probe below): (20,20) at k_h 15 gives leg jerk 20.4k mm/s^3, acceleration 485 mm/s^2, velocity 44 mm/s; (35,20) at k_h 10 gives 63.9k / 1511 / 90. Solve time, `plan_landing` end to end, best of 7 on the unloaded Jetson: capture 17.7 ms against 19.4 ms on-line at k_h 15 (19.1 against 21.3 at k_h 17), because the freed span shrinks the program. A loaded-box timing is owed to rehearsal row 39.

`motion/unified_cycle.py`: `_events_for` sets `capture`, and `plan_cycle` fills `capture_jerk_mps3` from `limits.leg_jerk_mmps3` for capture windows only.

### Defect 1: executor and node

`motion/skills/executor.py` needed NO behavioural change: with the planner flag the real `compile_reload` schedule through `SkillExecutor` and the real `install_segment` installs REST then CATCH for balls 17 and 18 (the re-send path already treats any refusal as non-fatal, `_live_catch = None`, "no further re-aim for this catch", so the cheap pre-solve refusal flows through it). `_catch_terminal` re-derives only the SETTLE rest site from the refined landing, which is correct under a capture; its comment now says nothing there may assume start == line. `segments.hold_axis_site` docstring amended (a REST/settle site off the line is still refused; a seed off the line is captured). The `AIM-LATERAL-CLAMPED` message was wrong ("catch keeps the schedule's site") and now reads "aim landing +X mm in <axis> off the schedule's site, clamped to +Y mm applied (the catch moves that far from the site)"; the side suffix the tests key on is unchanged. `segments.py` is one of the eight admissible-box gated files, so `gate_hash` moves and the box needs a re-sweep (cell values expected unchanged; the sweep windows have on-line seeds).

### Defect 2: level after END (executor + node)

`SkillExecutor._plan_closing` / `_advance_closing`, decided once at the first look after `attempt_ended`. When the last ACCEPTED install is a REST with non-level `rest_tilt` or a CATCH with `hold_tilt` and no carried throw, and the schedule's own level DECAY REST after it is undispatched (only plain CATCHes between; a THROW, carried CATCH or dispatched REST in between plans nothing), that REST is dispatched at its success-path instant (announced landing + `REST_TAIL_S` + margin - lead), delayed by any converged tracker landing's offset from the announcement, with a terminal that never squeezes the slew (`max(scheduled, now + lead + window)`). Level-after-END applies to every end code except `LANE_REFUSED` (the firmware holds the lane; the node's hand-less hold is the safe end) and `ABORTED_MODE_CHANGED` (the operator has the platform); `end_code` is never overwritten; a refused decay is logged `DECAY-ON-END refused` with no fallback; `level_on_end` (default True) is the escape hatch. The ordering after the announced landing, not at the END, is the point: at the 519.149 refusal the ball was still in the air.

Stop, cancel and go_home table. An operator Stop KEEPS the decay (the tilted terminal is the failure). A Ball Butler cancel confirmed more than 0.9 s before the release (`_BB_CANCEL_TRUST_S` = the 0.6 s `_BB_CANCEL_WINDUP_S` + 0.3 s service / CAN latency; `SkillExecutor.note_ball_cancelled` from `skill_node._on_bb_cancel_done`) levels immediately; a cancel inside the 0.9 s trust window, or a failed one, keeps the scheduled time (the confirmation means "cancel SENT", not BB's verdict, and BB drops a throw only before its wind-up). `go_home`, `go_to_pose` and `hold` cancel the decay through the new `Observations.plan_superseded` (a fresh trajectory status, stamped more than `_SUPERSEDE_STATUS_SETTLE_S` = 0.5 s after the node's last accepted install, whose plan is not a cycle plan): the trajectory node's `go_home` installs a non-cycle plan and leaves the control mode at TRAJECTORY, so the existing `in_trajectory_mode` field does not see it. Disarm / E-stop (mode leaves TRAJECTORY) and a rising `hand_lane_refused` cancel it too (WARN `DECAY-ON-END cancelled`). Operator-visible lines come through `executor.notices` drained by `skill_node._drain_notices`.

Debug parameter `debug_force_catch_refusal_once` (`skill_node.py`, default False): when armed, the first CATCH install is answered `CATCH_AXIS` (`DEBUG ...`) without calling `trajectory/install_segment`, WARN on each firing, then the parameter resets itself. A runtime `ros2 param set ... true` is refused while any goal is live. It exists so rehearsal row 40 can exercise Defect 2 without a lucky race; it must never be set in a production launch. `_stop_attempt` now appends "the decay to level is still due in X s -- a Stop keeps it".

### Contracts, rehearsal rows, fixtures, probes

- `motion/skills/INVARIANTS.md`: `C-CUP-5` (the held-axis seed check, previously an unnamed refusal loop; the exception to it is the capture), `I-CATCH-6` (the capture move: commanded start, the three constraints at k_h, the pre-solve gate, the restated objective and `j_e` box, the post-solve check), `I-PLAN-9` (level after END and its exclusion/Stop table); the `C-CATCH-2` fate column now states the clamp is also the capture bound.
- `tests/hardware/session_skill_stack_rehearsals.md` section 1c, rows 39-42: 39 the capture dress rehearsal on the loaded Jetson (`skills/check` plus `test_cup_capture.py` under load); 40 the forced-refusal decay; 41 Stop / go_home / BB cancel during the decay wait; 42 five BB-fed reload catches with a ball (no deliberate bias: the clamped re-aim is already 20-28 mm on a typical feed). Expect benign non-fatal `RESEND-REFUSED CATCH_AXIS` lines after a capture.
- Tests: `tests/motion/test_cup_capture.py` (20, planner: byte identity on-line, flag-off refusal, cap and knot refusals, splice and k_h continuity, closed-form jerk, the post-solve check, replay of balls 17 and 18), `tests/motion/test_skills_capture_replay.py` (8, real solves through the executor), 14 D2 tests in `tests/motion/test_skills_executor.py`, 8 in `tests/ros/test_skill_node.py` plus the authority pin; `test_a_capture_span_is_refused_by_name_until_it_is_built` in `tests/motion/test_cup_cycle.py` was renamed `test_a_held_steady_still_refuses_a_capture_span`, its fingerprint test unchanged.
- Fixtures `tests/fixtures/capture_move_replay_2026-10-19.yaml` and `tests/fixtures/capture_move_replay/`, written by `tools/probes/capture_move_replay_extract.py` from the 2026-10-19 bag and launch log (balls 10, 17, 18; reproduces the logged offsets 28.28 / 20.77 mm against the logged 28.3 / 20.8). `tools/probes/capture_move_sim.py` is the sim-gate harness below. Both have README rows.

### Sim check

`sim/skills_gate.py` gained `SelfTossGateConfig.reload_bias_mm` and the CLI `--reload-bias-mm BX BY` (the ball lands that far off the ANNOUNCED landing; default (0,0) is bit-for-bit the old behaviour). The stock gate cannot reach the capture (flight lead 0.35 s, so the reload CATCH is never re-aimed; authorities unset), so the check runs through `tools/probes/capture_move_sim.py` with flight lead 0.65 s and the live authorities (2026-10-19 ~12:50-13:00 AEDT, seed 0):

| run | result |
|---|---|
| stock gate, bias -20 -20 | FAIL makes 1, `LIMIT_JERK` (post-catch REST); skill 1 never re-aimed |
| stock gate, bias 5.6 -20 | identical FAIL |
| stock gate, no bias (control) | identical FAIL |
| capture, bias 0 0 | PASS makes 2; captured to (-20, 0) (immature fit x-noise clamped); leg jerk 35.7k mm/s^3, plan xy speed 175 mm/s |
| capture, bias -20 -20 | PASS makes 2, no END; leg jerk 48.4k, 245 mm/s, plan xy (-33.14, 0) -> (-53.71, -20.00) by k_h = 8; measured platform at (-53.2, -20.0) at touch-down |
| capture, bias 5.6 -20 | PASS makes 2, no END; same capture leg |

No `LIMIT_JERK`, `LIMIT_ACC`, C-CUP-4 or closing-speed refusal on the capture leg (cap 200k). Two caveats. First, the clamp (-20,-20) = 28 mm is what is captured in all three runs: the first, immature fit lands -65 to -91 mm in x regardless of the bias, so the sim does not discriminate (5.6,-20) from (-20,-20). Second, a second re-send at +0.14 s is refused `CATCH_AXIS` ("only 3 steps left") although the fit moved 0.2-11.5 mm: the seed is mid-capture, so the planner correctly says a new slide does not fit; it is non-fatal, but it spends the catch's last re-aim, so a catch whose first fit is the noisy one cannot be refined. Expect such lines on hardware (row 42). The stock gate's own `LIMIT_JERK` is not the capture: it is the post-catch REST after the unclamped 41-45 mm tracker re-aim on the second catch; with the 20 mm clamp the same chain passes (noted in the plan residue, not fixed here).

## Verification

All runs 2026-10-19 from `~/Desktop/Jugglebot-oval` with the project venv.

- Unit tests by unit (scoped, during the build): planner `pytest tests/motion/test_cup_cycle.py tests/motion/test_validate_cycle.py tests/motion/test_unified_cycle.py tests/motion/test_cup_capture.py -q` -> **267 passed, 2 skipped**; executor + replay `pytest tests/motion/test_skills_executor.py tests/motion/test_skills_schedule.py tests/motion/test_skills_segments.py tests/motion/test_cup_capture.py tests/motion/test_skills_capture_replay.py -q -p no:cacheprovider` -> **481 passed in 11.54 s**; after the audit follow-ups, the same executor/node files with the gate hash pinned -> **614 passed in 63.39 s**.
- Admissible box re-sweep (`bash temp/sweeps/sweep_P1.sh`, four families x2, BLAS one thread): all four `cmp` IDENTICAL; merged 20 cells, every cell VALUE identical to the committed box, only `gate_hash` (606b53e3912f -> 4c4d65a39ae1 after the final gated-file edits) and `swept_at` moved; one post-fix oval re-sweep compared cell-for-cell identical to the full run. `pytest tests/motion/test_skills_admissible.py tests/ros/test_skill_node.py -q` -> **433 passed in 54.22 s**.
- Sim check: `tools/probes/capture_move_sim.py` recipe (flight lead 0.65 s, authority on, seed 0) at bias (0, 0), (-20, -20) and (+5.6, -20): all PASS, captured, leg jerk 35.7-48.4k mm/s^3 of 200k, peak platform xy 175-245 mm/s, no refusal on the capture leg (logs `temp/logs/skills_gate_capture_*_20261019.log`).
- Audit: `/audit --unstaged` -> 0 BLOCKING, 2 WARNING, 6 NOTE; all applied (the STALE_STATE retry bound and the 0.9 s cancel trust window are owner-approved behaviour changes, the rest narrative/test). A first full-gate run (`temp/logs/gate_full_capture_20261019.log`) showed 3 phantom failures caused by a gated-file comment edit landing mid-run (hash moved); not a code defect.
- **The gate:** `./run_tests.sh --full` (log `temp/logs/gate_full_capture2_20261019.log`) -> parallel **6946 passed, 9 skipped in 314.04 s**; serial **7 passed in 27.88 s**; RESULT PASS, exit 0.
- Hardware: NOT FLOWN. Rehearsal rows 39-42 in `tests/hardware/session_skill_stack_rehearsals.md` are the check (build + full launch restart first).

## Outcome

Fix built, audited and gate-tested on 2026-10-19; not flown. A held-axis BB feed now captures a tracker re-aim of up to hypot(35, 20) mm inside the catch window from the commanded state, with CATCH_AXIS judged at the hold-start knot; an attempt that ENDs after its pre-tilt REST levels the platform at the schedule's own decay instant (or immediately on a Ball Butler cancel confirmed more than 0.9 s before release). This morning's two refused balls replay as installed catches. The entry stays in-progress until rows 39-42 fly; the separate ~20 mm y feed bias between Ball Butler's announced and tracked landings (every feed today) is noted for the columns plan, not fixed here.

Flown on 2026-10-21 and 2026-10-22: the six empty-cup two-feed chains ran with the held-axis capture fix and the C-CUP-4 cone refusals did not recur in about 30 columns goals; the feed y bias (about 20 mm) stayed open as residue, and the plan was archived on 2026-10-23.

## Withdrawn claims

- [2026-10-19] Claimed the 11:07:34 Stop left the platform displaced and tilted, and that the reload compile assumes the rest pose lies on the ball's axis (owner's report, adopted as the first framing in the investigating session).
  WITHDRAWN: the bag shows the platform level at the feed site (tilt 0.50-0.53 deg) from 454.8 to 517.4 s, 63 s after the Stop; the tilt began at 519.1 s, 3 s into the next attempt, as the pre-tilt REST's terminal after the attempt ENDed on CATCH_AXIS. The refusal comes from the 0.1 mm held-axis seed tolerance against a tracker re-aim, not from the starting pose.
  Superseded by: Diagnosis, Defect 1 and Defect 2.

## Open Questions

1. Does the owner's recollection match the bag, i.e. the tilt appearing about 3 s after pressing go on the next attempt rather than immediately after the Stop?
2. Defect 2 timing: decay to level only after the announced landing plus margin (ball possibly bouncing off a 10 deg platform), or immediately? The chosen design above says after landing plus margin; confirm the margin value.
3. The capture move makes the feed catch laterally closed-loop for the first time (every held-axis re-aim today was refused). Its effect on catch success needs a sim proof and a hardware rung of its own.
4. Throw 4/4 miss at 11:06:51 (C-CUP-4 33.4 deg plus LIMIT_JERK re-aim refusal): not analysed here.
