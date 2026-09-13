# R3 hardware gate — single-site self-toss, learner on

Skill-stack R3's first-ever powered attempt with a learner running on this
machine (`plans/active/two-ball-skill-stack.md` § 0 / R3: *"the first rung a
learner ever runs on this machine"*). Same shape as `session_skills_r2_plan_gate.md`
(that sheet stays live only for re-running its row 17 margin check, see § 1
below) but this one **throws a real ball**. Driver:
`tests/hardware/skills_plan_bench.py` (`--pattern self-toss`), pure core
tested in `tests/ros/test_skills_plan_bench.py`. Node under test:
`skill_node.py` (`skills/start_self_toss`, `skills/check`, `skills/stop`).

**What this sitting is, in six lines.**
1. One site, P1 = (−50, 0) mm. One ball. `skills/start_self_toss` pre-levels
   the platform, then compiles a THROW → CATCH(+throw) → … → REST self-toss.
2. Cold-start policy A: single-throw attempts (`n_throws:=1`) until the
   memory holds 2 rows, then a CHAINED attempt. Reason: an unlearned throw
   lands ~84 ms late against a fixed carried release, which squeezes the
   dwell to ~0.22 s — chaining before the memory can correct for that is
   what single-throw attempts avoid.
3. Gate: land in-band within 5 throws from a cold memory, then 10 consecutive
   catches at the site.
4. Session limits: legs 300 / 5000 / 150 000 mm/s³ (the most ever flown on
   this machine — the single-site cycle's own jerk is zero, see § 1), hand
   3500 rev/s² (chained peak measured 3341).
5. Perception preconditions are HARD: the QTM `Catching Cone` rigid body
   DISABLED, Ball Butler reflectors MASKED — § 2 will not proceed without
   both, and `skills/check` cannot see either one (they are QTM-side, not on
   any topic this node reads).
6. Two pre-power findings this sheet's own rehearsal surfaced (both dated
   2026-09-13) are now RESOLVED — read § 1 for what changed and why it
   matters for this sitting.

---

## 0. Why the robot throws this time (and what you can push back on)

R2 never moved the platform; R3 does, with a learner writing a memory file
that changes what the NEXT throw commands. **If your physical intuition
disagrees with this framing — the site, the cold-start policy, the session
limits, anything below — that is load-bearing signal, say so before we start.**
Two places that are especially worth a sanity check from the person standing
next to the robot: (a) the R2→R3 jerk ceiling actually DROPS (200 000 →
150 000 mm/s³) even though R3 is the first rung that throws — is that the
ceiling you expect for a single ball at 0.9 m apex; (b) the perception
preconditions (cone disabled, reflectors masked) are enforced by QTM
configuration, not by this node — nothing here will catch a session where
that step was skipped.

## 1. Before the robot is powered (no ROS, any time)

Every terminal: `source /opt/ros/foxy/setup.bash && source
~/Desktop/Jugglebot-skills/ros_ws/install/setup.bash`. Use the project venv
(`source ~/Desktop/PDJ_venv/venv/bin/activate`) for anything below marked
"venv"; plain `python3` with ROS sourced for anything marked "ROS".

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git status -sb` | Clean, at or after the commit that landed this sheet. |
| 2 | (venv) `./run_tests.sh --full` | Green — plan § 0 Rigor: "a dress rehearsal on the loaded Jetson... before any powered sitting" needs the gate itself green first. Record pass count + wall time in § 7. |
| 3 | (ROS) `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash && cd ..` | Builds — new node code + the tracker plane since R2. |
| 4 | (venv) `python3 tests/hardware/skills_plan_bench.py --dry-run --pattern self-toss --n-throws 1` | Prints the cold-start (1-throw) schedule: 4 skills (opening REST, THROW, CATCH, closing REST), site P1, apex 0.90 m, dwell 0.30 s, the two splice budgets (125 / 75 ms), and the five gate criteria. |
| 5 | (venv) `python3 tests/hardware/skills_plan_bench.py --dry-run --pattern self-toss --n-throws 10` | Prints the chained (10-throw) schedule: 13 skills (1 opening REST + 1 THROW + 9 CATCH+throw + 1 standalone CATCH + 1 closing REST — `n_throws + 3`, verified against the real compiler in `tests/ros/test_skills_plan_bench.py::test_build_self_toss_schedule_has_n_throws_plus_3_skills`). |
| 6 | (venv) `python3 tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss --arm A --attempts 3` (run twice) | **Both runs identical and clean**: `blas threads: 1`, three attempts each `ended_early=False (4/4 skills)`, memory rows 1→2→3, and **verdict G1/G2/G4 PASS, G3/G5 SKIP** (offline — no emitter/wire). Reference, 2026-09-13 (idle Jetson): G1 worst 33.1–33.6 ms over 15 solves (bar 50 ms); G2 handoff max 33.1–33.6 of 125 ms, unpinned max 25.8–26.3 of 75 ms; memory row 3's command is NOT the identity prior (the learner is active by attempt 3 — see Finding B below, now RESOLVED, for why that mattered before the fix landed). |
| 7 | (venv) `python3 tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss --arm B --attempts 3` | Historical reference only, pre-fix (see Finding B, now RESOLVED). 2026-09-13: attempts 0–1 clean (re-sends refuse as expected, exactly like columns' arm B); attempt 2 **ended early** (`LIMIT_JERK` on the primary THROW dispatch, not a re-send) once the learner went warm — root-caused and fixed (§ 1 Finding B); re-running this rehearsal is not required to satisfy this row. |
| 8 | (venv) `python tools/probes/skills_single_site_sweep.py --study grid --apex 0.9 --jerk 150000 --site-xy=-50,0` (or cite the existing run) | The single-site cycle's OWN leg jerk is **zero** at this operating point (2026-09-13) — no ramp needed for the chained/10-catch run's steady-state cycle. This is a DIFFERENT number from Finding A below (which is about the SESSION-START move onto the site, not the steady cycle). |
| 9 | Re-run `session_skills_r2_plan_gate.md` row 17 (margin, not gating) if the owner's background-load work has landed since the third sitting | Record G1/G3 maxima in § 7 alongside the original 93.88 / 40.9 ms — R3's added load (a real ball, the tracker, the learner) can only push solves further from the 50 ms bar, so this number bounds what R3 should expect, not what it must pass. |
| 10 | QTM: disable the `Catching Cone` rigid body; mask the Ball Butler reflectors | **Hard precondition (plan § 2.7)** — 2026-09-06's bag showed the ball binding to the stale cone body on 5/7 throws, and a 2026-09-13 probe found only 10/27 historical self-tosses passed the observed predicate. Nothing downstream can catch a skipped step here. |

### Finding A — the opening REST needs more than 1.0 s at this session limit (RESOLVED)

`schedule.compile_self_toss`'s own opening REST (`schedule.FLOOR_LIFT_S`, the
floor lift from the ACTIVATE park to site P1's rest position) **refuses
`LIMIT_JERK` at the R3 session limit at its original 1.0 s value**, measured
through the real install chain, 2026-09-13 (`ex.install_segment`, park →
`site.rest_site_mm()`, `leg_jerk_mmps3=150000`):

| Window | Result | Peak leg jerk |
|---|---|---|
| 1.0 s (`FLOOR_LIFT_S` pre-fix) | REFUSED `LIMIT_JERK` | 177 241 mm/s³ |
| 1.2 s | REFUSED `LIMIT_JERK` (non-monotonic solver artifact — worse, not better) | 205 051 mm/s³ |
| 1.5 s | ACCEPTED | — |

The SAME move (park → site P1 rest) is what columns' R2 gate already runs at
its own (higher) 200 000 mm/s³ ceiling without incident — this is a NEW
margin gap R3's lower ceiling opens up, not a regression in anything R2
touched. **`schedule.FLOOR_LIFT_S` has been widened 1.0 → 1.5 s (landed, unit
h1)** — the 1.5 s window measured ACCEPTED above, so the first-ever
`skills/start_self_toss` call this session is no longer expected to refuse
on this segment. Nothing else about the move changed (still park → site P1
rest at the R3 session limits): if it refuses anyway on the day, treat that
as a NEW finding, not a repeat of this one.

### Finding B — a warm learner can push the launch THROW over the jerk ceiling (RESOLVED)

Once the memory crosses `k_min` (2 rows) and the learner starts commanding a
non-identity `(landing offset, flight)`, that command is applied to whichever
skill releases next — including the **launch THROW** (a fresh-origin,
`launch_s` = 0.4 s window, the form EVERY cold-start attempt after the second
one uses). First measured 2026-09-13 (`--rehearse --pattern self-toss --arm B
--attempts 3`, deterministic by seed): attempt 2's launch THROW refused
`LIMIT_JERK` at 150 000 mm/s³ with a learner-commanded `u ≈ (−31 mm, 0 mm,
0.78 s)` that was **inside** the swept admissible box for (P1, P1) — the box
alone did not catch this.

Traced (unit i, `cup_realize._accel_bounded_schedule`) to the tilt pin-blend
width being sized from the pin gap, producing 2-knot corners (knot 1 plus a
one-sided-FD slope break at the release seam) whose sharpness grew as the aim
offset shrank the pin gap. **Fix, landed**: `cup_realize._TILT_BLEND_MIN_KNOTS
= 8.0` floors that width. Measured effect on a single-site launch's leg jerk
vs aim offset: 10 / 20 / 40 mm now give 14.6k / 29.2k / 58.5k mm/s³, against
123k / 125k / 140k before the floor — jerk now scales with the aim instead of
sitting near-constant regardless of it.

Second half of the fix (unit h2): the admissible box now gates the launch
throw from rest, not just the steady catch-and-throw form, and was
regenerated at 150 000 mm/s³ (`python tools/admissible_sweep.py --site-pairs
both --leg-jerk 150000`, 2026-09-13, run twice with identical results):
(P1, P1) `landing_xy_m` ±(40, 30) mm, `flight_s` [0.750, 0.857] s. One caveat
carries forward: the box cannot lengthen a flight past nominal, so a SLOW
throw is uncorrectable by construction, not a residual bug.

With both changes landed, the attempt-2 refusal above does not reproduce —
watch for it anyway during the cold-start attempts below (§ 3) and the
chained attempt (§ 5); if it recurs on the real robot, that is a NEW finding,
not a repeat of this one.

### Open risks (informational — not blocking, but state these plainly before starting)

- **Release-lag gap, unexplained.** Older bags show a commanded→physical
  release lag of 156–371 ms with no diagnosed cause. This sitting's flight
  timings are measured from the bag, not assumed — if the lag is real on
  this session's bag too, the 20 ms flight band can fail on hardware for a
  reason that has nothing to do with the learner. Check the bag's own
  release timestamp before reading a flight-band miss as a learner defect.
- **Perception preconditions are hard even when followed correctly.** Only
  10 of 27 historical self-toss flights passed the "observed" predicate
  (row 10) — most flights are not cleanly tracked even with the cone
  disabled and reflectors masked. Expect some throws this sitting to be
  unobservable through no fault of the setup.
- **Tracker catch plane moved.** The tracker now predicts the landing at the
  830 mm catch plane (was 809.08 mm) — the same change moves the FSM's catch
  plane too; it is not scoped to this sitting alone.

## 2. Bring-up (launch UP, robot powered, ball present)

| # | Step | Expect |
|---|---|---|
| 11 | Load capture, own terminal, left running: `( while true; do echo "$(date +%H:%M:%S) $(cat /proc/loadavg)"; sleep 1; done ) \| tee temp/logs/loadavg_r3_$(date +%Y%m%d).txt` | One line a second. |
| 12 | `ros2 launch jugglebot jugglebot_launch.py record:=true auto_arm:=true 2>&1 \| tee temp/logs/launch_r3_$(date +%Y%m%d_%H%M).log` | **`auto_arm:=true` this time — R3 throws for real.** Note the bag folder it prints. |
| 13 | `grep 'blas threads' temp/logs/launch_r3_*.log` | `blas threads: 1` for `trajectory_node` AND `skill_node`. Anything else: stop (the UH-3 E-STOP class). |
| 14 | `ros2 service list \| grep -E 'skills/(start_self_toss\|check\|stop)'` | All three listed. Missing = the launch sourced the main install (step 1's note in the R2 sheet applies here too). |
| 15 | Open the GUI (http://localhost:8081); start QTM streaming (cone body disabled, reflectors masked — step 10). | Both part of the load being measured. |
| 16 | GUI: **Home**, then **Activate**. | Robot at the active pose, hand parked at 0 rev. |
| 17 | `ros2 param set /skill_node site_x_mm -50.0`, `... site_y_mm 0.0`, `... apex_m 0.9`, `... dwell_s 0.30`, `... plant_id r3-$(date +%Y%m%d)` | A FRESH `plant_id` this session — `memory.memory_path`'s own contract: a fresh id starts a cold memory. Record the exact id used in § 7. |
| 18 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 300.0, leg_acc_limit_mmps2: 5000.0, leg_jerk_limit_mmps3: 150000.0}"` | `applied_*` echoes **300 / 5000 / 150000** — the launch default is 1000 / 5000 / 30000 (`hardware_config.py:174-176`), so this call is mandatory, not conditional on it already matching. |
| 19 | `ros2 service call skills/check std_srvs/srv/Trigger` | **Every current refusal at once** (item 8): expect `ladder REFUSED: REJECTED_NOT_LEVELLED` before `level`/pre-level has run, plus `box OK` (limits match) once step 18's `set_limits` call has applied the session limits. If MORE than the expected rows refuse, fix each named one and re-run — never re-dispatch around a refusal by hand. |
| 20 | Seat a ball in the hand. | The hand's possession sensor should read SEATED — `skills/check`'s ladder cannot see this directly (no `ball_evidence` row printed by `_svc_check` — only the launch-time `_dispatch` path checks it), so confirm visually and via `ros2 topic echo /hand_telemetry` (`ball_held_valid: true`, `ball_held_raw: true`) before the first `start_self_toss` call. |

## 3. The dress rehearsal (every refusal reported at once, robot NOT yet armed to throw)

Before the first REAL throw, run `skills/check` again after every step above
is satisfied, and confirm the ladder reads clean:

| # | Step | Expect |
|---|---|---|
| 21 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK; box OK: 3 site pair(s), limits match` — if this is not clean, STOP and work the refusal list before any `start_self_toss` call. Finding A is RESOLVED (`FLOOR_LIFT_S` widened to 1.5 s, § 1); the first call is no longer expected to refuse on that segment — a refusal here is a NEW finding, not Finding A recurring. |
| 22 | `ros2 param set /skill_node n_throws 1` | Cold-start policy A's own first attempt. |

## 4. Cold-start attempts (n_throws := 1, repeat until in-band within 5 throws)

| # | Step | Expect |
|---|---|---|
| 23 | `ros2 service call skills/start_self_toss std_srvs/srv/Trigger` | Accepted (`success: true`), message names skills/throws/`plant_id`/memory rows. Finding A is RESOLVED, so this call is no longer expected to refuse on the opening REST — **if it refuses anyway, that is a NEW finding**, not Finding A recurring. |
| 24 | Watch `skill_node`'s log for the memory-row line (`memory row appended: x=... u=... y=... caught=...`) and the throw's landing. | One memory row per completed attempt. Record `(throw index, landing error mm, in-band Y/N)` for each. |
| 25 | Repeat 23–24, incrementing `plant_id`'s attempt count only in your notes (NOT the parameter — memory is append-only under the SAME `plant_id` all sitting) | **Gate: in-band (xy ≤ 30 mm, flight ≤ 20 ms of nominal) within 5 throws of this cold memory.** Record the throw index it lands in-band at (or that it did not, within 5). |

## 5. Chained attempt (10 consecutive catches)

| # | Step | Expect |
|---|---|---|
| 26 | `ros2 param set /skill_node n_throws 10` | The chained form — every throw after the first CARRIED by the previous catch (`then_throw`). |
| 27 | `ros2 service call skills/start_self_toss std_srvs/srv/Trigger` | Accepted. Watch for Finding B (a learner-warm launch THROW refusing `LIMIT_JERK`) — this attempt's OWN launch throw is a fresh-origin form and is the one Finding B is about. |
| 28 | Count consecutive catches from the possession sensor / bag. | **Gate: 10 consecutive catches at P1.** If the attempt ends early before 10, record the end code (§ pre-registered verdict table below) and the throw index. |
| 29 | If it ends early, `ros2 service call skills/stop std_srvs/srv/Trigger`, seat/re-seat the ball, and repeat 26–28. | A refused skill leaves the rest tail already streaming — never re-dispatch a hand move by hand. |

## 6. Pre-registered verdict table (decided before the sitting)

Every code below names ONE physical fact — if a code fires that is not on
this list, treat it as a NEW finding, not a known outcome.

| Code | Physical fact |
|---|---|
| `REJECTED_MOCAP_STALE` | The mocap graph has gone silent (no fresh `rigid_body_poses` in 0.5 s) — a QTM/network problem, not a robot one. |
| `REJECTED_NOT_LEVELLED` | `trajectory/status.gravity_correction_loaded` is false on a fresh status — the platform's commanded frame isn't gravity-corrected (no `level`, or `_prelevel` hasn't run/succeeded yet). |
| `REJECTED_HAND_STALE` | No fresh `hand_telemetry` in 0.5 s — the hand's own position feed is silent. |
| `REJECTED_HAND_NOT_PARKED` | (launch only) The hand's MEASURED position isn't within the homing park band of its COMMANDED position — the seed a fresh THROW would launch from isn't where the machine actually is. |
| `REJECTED_BALL_UNKNOWN` | (launch only) The possession sensor has no confident reading — nobody knows if there is a ball in the cup. |
| `REJECTED_NO_BALL` | (launch only) The possession sensor confidently reads EMPTY — there is no ball to throw. |
| `ABORTED_MODE_CHANGED` | Something left `trajectory_node` out of TRAJECTORY mode mid-attempt — the rest tail already streaming is the safe stop. |
| `ABORTED_NO_RELEASE` | No possession-EMPTY or tracker evidence the ball left the hand within 0.5 s of the commanded release — the throw's physics were never confirmed; no learner row for it. |
| `NO_ADMISSIBLE_COMMAND` | Either the swept box for (P1, P1) is empty, or the learner's local fit diverged to a non-finite command — nothing here can hand the platform a command it can stand behind (see Finding B, now RESOLVED, for the ADJACENT case: a command INSIDE the box that still overran the launch window's jerk before the aim-jerk floor landed). |
| `NO_LANDING` | Only a STANDALONE catch (not carrying a throw) can end here: it waits for its own ball's tracked landing and refuses at the deadline `t_land − 0.278 s − lead` if none arrives — and a tracked landing only counts if it is later than that ball's own previous release. A catch that carries the next throw never waits on this: it installs AT RELEASE, aimed at the ball's PREDICTED landing (from its previous release), then refines via tracker re-sends (owner decision, 2026-09-13). |
| `SPLICE_TOO_LATE` | The solve finished after the wire had already read past the intended splice knot — a slow solve would otherwise rewrite trajectory already being interpolated. |
| `WINDOW_TOO_SHORT` | Fewer knots from the splice to the event than the gate needs to measure a jerk at all. |
| `UNREACHABLE` | A malformed terminal (a `ValueError` building it) — a parameter/programming error, not a physical-limit refusal. |
| `LIMIT_JERK` / `LIMIT_ACC` / `LIMIT_VEL` / `HAND_STROKE` / `HAND_ACC` (trajectory_node/planner) | The named leg or hand axis would exceed its session limit or physical travel band on this specific segment. |
| `STALE_STATE` / `WRONG_MODE` (trajectory_node/planner) | `trajectory_node`'s own seed is stale, or the node isn't in the mode this install assumes. |

## 7. If something refuses

A refused skill ends the attempt through the rest tail already streaming —
**never re-dispatch a hand move by hand.** `skills/stop` is safe to call at
any time: it stops further dispatch, and since 2026-09-14 (sitting 1, latch
L1) an ended attempt — `stop` or any abort — installs ONE `trajectory/hold`
when the committed segment still carries a future release, so a
catch-with-throw that was already installed no longer throws after the
attempt has ended (the hold is a profiled decel-to-rest of the platform; the
hold plan carries no hand track, so the hand lane sees a HAS_HAND falling
edge and the bridge firmware DECAYS it to rest where it is — the same rule
the guard freeze relies on; the ball, if in the air, is not caught). If `skills/check` shows more than
the expected ladder rows refused, work each one via § 6's table before
calling `skills/start_self_toss` again.

**Sitting-1 recoveries (2026-09-13):**
- `REJECTED_HAND_NOT_PARKED` now also fires when the hand is not at park
  (|measured − 0.00| > 0.50 rev), on the opening REST as well as a THROW.
  Recover with **DEACTIVATE then ACTIVATE** (ACTIVATE re-parks the hand at
  0 rev through the bridge's own 1 rev/s re-activation slew). Never try to
  park it with a segment: the guard measures the raw plan against the
  encoder while the bridge slews, and a >2.5 rev gap latches MAX_DEVIATION
  (latch L2).
- A `MAX_DEVIATION` line now names the axis: `hand first to cross` is the
  hand (axis 6). The six-entry `live_dev` that follows is legs only.
- After ANY guard latch, before re-arming: the guard descent collapses the
  legs onto measured but NOT the hand — the hand command stays frozen
  (58 s at 9.43 rev against a 0.76 rev droop on 2026-09-13). DEACTIVATE /
  ACTIVATE before the next attempt.
- The Ball-Butler reload is REFUSED at accept (`REJECTED_RELOAD_RETIRED_R1`)
  — R3's reset is operator placement (plan § 1 item 7); the reload returns
  as a CATCH skill at R4. Do not use the GUI reload button this rung.

## 8. Close-out

| # | Step | Expect |
|---|---|---|
| 30 | `ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'deactivate'"` | Robot stows. |
| 31 | Stop the launch and the load capture. | |
| 32 | `python tools/probes/throw_outcome_bag_probe.py --bag <id>` | Independent ground-truth learning curve (landing error vs throw index) from the bag, cross-checked against `skill_node`'s own logged memory rows. |
| 33 | Copy `temp/learn/<plant_id>/memory.csv` to a dated path before the next sitting reuses the same convention. | The memory row that produced the learning curve. |
| 34 | Send: the bag folder name, `temp/learn/<plant_id>/memory.csv`, `temp/logs/loadavg_r3_*.txt`, `temp/logs/launch_r3_*.log`, and the throw-by-throw table from §§ 3–4. | |
| 35 | Log the sitting (`/log feature` or `/investigate` if anything in § 6 fired unexpectedly). | The learning curve (landing error vs throw index) belongs in the logbook entry with the bag id (plan § R3 Gate). |

## 9. Results

### Pre-power (§ 1)

| Item | Result |
|---|---|
| Date | 2026-09-13 |
| `./run_tests.sh --full` (row 2) | *(operator fills in before the sitting)* |
| Rehearsal (row 6), run 1 — G1 max / G2 handoff max / unpinned max / verdict | 33.08 ms / 33.08 of 125 / 26.27 of 75 ms / **G1, G2, G4 PASS; G3, G5 SKIP** |
| Rehearsal (row 6), run 2 — same | 33.56 ms / 33.56 of 125 / 25.76 of 75 ms / **G1, G2, G4 PASS; G3, G5 SKIP** |
| Rehearsal memory row 3 command vs identity prior | differs both runs (`u_dy ≈ (−30.8, 0.0) mm`, `u_flight ≈ 0.780 s` vs prior `(0, 0)`, `0.857 s`) — the learner IS exercising command changes offline. |
| Arm B rehearsal (row 7) | attempt 2 ended early, `LIMIT_JERK` on the launch THROW (pre-fix) — root-caused and fixed as **Finding B**, now RESOLVED. |
| Finding A window sweep (§ 1) | 1.0 s REFUSED (177 241 mm/s³), 1.2 s REFUSED (205 051 mm/s³), 1.5 s ACCEPTED — landed as `schedule.FLOOR_LIFT_S = 1.5`; **Finding A RESOLVED**. |
| R2 gate row 17 re-run (row 9) | *(operator fills in — not gating)* |

### Sim (R3-d, parallel unit) — landed 2026-09-13

Two separate (date, command, result) triples — the hardware gate below is
NOT satisfied by these; it is a distinct gate.

**Policy A (cold-start, single-throw until 2 memory rows, then chained)** —
`python sim/skills_gate.py --learn --policy A`, seeds 0–4, 2026-09-13, run
twice with identical per-throw results both times:

| Item | Result |
|---|---|
| Command | `python sim/skills_gate.py --learn --policy A` (seeds 0–4) |
| Result, all 5 seeds | PASS — xy and flight error enter the band (20 mm / 20 ms) at throw 3, monotone over throws 6–25; 3 attempts, 25 makes, 0 drops |
| Reports | `temp/reports/skills_gate_learn_A_run1.json`, `temp/reports/skills_gate_learn_A_run2.json` |

**Policy B (chained from throw 1)** — `python sim/skills_gate.py --learn
--policy B`, seeds 0–4, 2026-09-13:

| Item | Result |
|---|---|
| Command | `python sim/skills_gate.py --learn --policy B` (seeds 0–4) |
| Result, all 5 seeds | PASS — band entered at throw 5; 1 attempt (all 25 throws in one chain); 0 drops |
| Report | `temp/reports/skills_gate_learn_B_run1.json` |

Both satisfy the plan's sim-validation gate (cold-start-within-5-throws,
monotone over the next 20 — plan § R3 "Sim validation").

### Hardware gate (§§ 2–5) — filled in during the sitting

| Item | Result |
|---|---|
| Date, commit, bag | **Sitting 1: 2026-09-13 evening, `0778ca5` + the R3 software commits, bag `2026-09-13_22-57-18`, log `temp/logs/launch_r2gate_20260913_2257.log`** |
| `plant_id` used | `r3-20260913` (2 rows, both from the chained attempt) |
| Cold-start throws to in-band (gate: ≤ 5) | **NOT MEASURED** — all five single throws were caught but ended `NO_LANDING`: a plain THROW never announced, so the node never correlated its ball (fixed 2026-09-14, `logbook/2026-09-14-skill-stack-r3-first-powered-sitting.md`) |
| Chained attempt: consecutive catches (gate: 10) | **2–3 of 5** (operator), then `ABORTED_NO_RELEASE` → hand `MAX_DEVIATION` latch (L1) |
| Any code from § 6 that fired outside the pre-registered list | `MAX_DEVIATION` ×2 on the HAND axis (L1: the ended attempt's plan kept throwing at the 3500 rev/s² ceiling, 47.8 A saturation; L2: opening REST from an un-parked hand into the bridge's 1 rev/s slew); `ABORTED_PRIME_FAILED` on a reload (retired at R1) |
| `load1` range | `temp/logs/loadavg_r3_20260913.txt` (not analysed — no throw reached the learner) |
| Learning curve (landing error vs throw index) | none — **the plant is ~25 % fast**: apex 1.38 m for 0.9 m commanded, flight 0.975/1.053 s for 0.857 s; the box's flight range 0.750–0.857 s cannot reach it. Owner decision on the hand acceleration ceiling before sitting 2. |
