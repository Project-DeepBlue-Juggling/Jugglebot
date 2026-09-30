# R5 hardware runsheet, sitting 2 — the receive-level feed catch and BB-fed columns

Skill-stack R5 (`plans/active/two-ball-skill-stack.md` § R5, "Owner decisions (2026-09-30
evening)"). Same machine as `session_skills_r5.md` (sitting 1) and, behind it,
`session_skills_r4.md` — **those sheets stay the reference for everything this one does not
restate** (QTM preconditions, bring-up rows, the guard/`/recover`/hand-homing recovery flow, the
carried R4/R5 watch items). Driver: `jugglebot/juggle` action, patterns `self_toss` / `hop` /
`columns`; `skills/check`; `jugglebot/juggle_stop`; the GUI reload button; plus, new this
sitting, `bb/start_accuracy_calibration` on `ball_butler_node`.

**What this sitting is, in six lines.**
1. Sitting 1 (2026-09-30 afternoon) found: the cup takes a feed arriving ~8 deg off axis at a 4
   deg hold, and still catches most feeds all the way down to a 0 deg hold; the YAW settle
   refusal epidemic (5/19) traces to a firmware rate-term dither, not a real unsettled axis; the
   feed lands 52 ms LATE against the schedule, not early; the lateral bias (+27/+11 mm) traces to
   Ball Butler's stale aim affine; Block C (a human-lobbed columns feed) never registered a
   single lob and is retired. Four fix units landed tonight to close those out (§ 0).
2. This sitting has no software surprises left to find in Block A/B — it is here to confirm the
   fixes hold, then push into the block sitting 1 could not reach: **Ball Butler feeding columns
   directly** (`reload: true`), which sitting 1's software did not yet support cleanly (the level
   feed catch that makes it admissible under the box was built tonight, after the sitting).
3. **Gate: the 0 deg gate (§ 5) is pre-registered pass/fail; the BB-fed columns block (§ 6) is
   data-gathering**, same discipline as sitting 1 — a learning-curve log, not a pass/fail line,
   pending R5's own hardware gate (five consecutive cycles, then 30 catches, plan § R5 Gate).
4. **Session limits: legs 300 / 5000 / 200 000 mm/s³, hand 3500 rev/s².** The leg-jerk ramp
   already HELD at 200 000 in sitting 1 (§ 2.5 of that sheet; 4/4 self_toss + 8/8 hop, no clamp,
   no latch) — **this sitting does not re-run the ramp block**, it starts directly at the ramped
   value. `set_limits` still MUST precede `skills/check` and every dress-rehearsal row (same
   `LimitsMismatch` hazard as sitting 1 § 0 point 6).
5. Two software surfaces are new since sitting 1 and untested on hardware: BB firmware FW 5 (a
   real settle criterion — position error AND rate AND in-band, not position alone) and the
   receive-level feed catch (a fixed, near-zero touch-down attitude for the columns feed catch,
   replacing the banked `tilt_to_receive` pin that a 12 deg-attitude catch needed). Both are
   flown live for the first time this sitting.
6. Rung order: pre-power (build, FW 5 flash) → bring-up (`set_limits`) → dress rehearsal (kept,
   not run by the owner) → BB accuracy-volley re-fit → the 0 deg gate → BB-fed columns → fallback
   if the 0 deg gate fails → close-out.

---

## 0. Why this sitting, and what you can push back on

**If your physical intuition disagrees with the framing below — the receive-level catch, the
0 deg cap, the BB placement fallback, anything else — that's load-bearing signal, say so before
we start.**

**The framing pivot, in the owner's own words and sitting 1's own numbers:**

Sitting 1 set out to test whether the cup could take a feed shallower than R4's flown 12 deg
hold. It could: 4 deg worked "perfectly every time" (2/2), and the owner pushed all the way to
0 deg, where 6/7 were still caught, "the timing seems a little off," with a hunch that 0 deg
could be made seamless. That result reopens Block C: the human-lob columns start never claimed a
single lob (`ABORTED_NO_COLUMNS_FEED` x3) and the owner retired human-initiated multi-ball
routines outright — **BB-led patterns are the goal, and the cup-takes-0-deg result is what makes
a BB-fed columns start buildable without the 12 deg banked catch** (that catch is what busted leg
jerk 1.72x, vel 1.2-1.3x and the cup-contact floor when the day-1 planner probes tried a level
transit catch against a 12 deg BB arrival).

Four fixes landed tonight, after the sitting, in direct response to sitting 1's findings (see the
handoffs in the scratchpad for detail; the logbook entry is
`logbook/2026-09-30-skill-stack-r5-sitting-1.md`):
- **The settle epidemic** (5/19 refusals, all inside the 1.0 deg position tolerance) traced to a
  150 Hz finite-difference velocity estimate dithering past a 3.0 deg/s rate gate on a
  sigma-delta deadzone — fixed in BB firmware (FW 5: settled = error inside 1.0 deg for the last
  15 samples AND rate <= 12 deg/s AND in-band now), plus a Jetson-side retry-once so a stray
  refusal does not end the attempt.
- **The lateral bias** (+27/+11 mm, all 14 feeds) traced to Ball Butler's aim affine being fitted
  at a BB pose 10.3 deg of yaw away from tonight's live self-calibration (the 2026-09-27
  coplanar-marker recalibration moved BB's read position) — fixed by re-running BB's accuracy
  volley (§ 4).
- **The receive-level feed catch** replaces the banked `tilt_to_receive` pin with a fixed
  near-zero touch-down attitude for the columns feed catch specifically
  (`Skill.receive_tilt` / `CatchTerminal.receive_tilt` / `CycleGoals.receive_tilt`,
  `compile_columns(pattern, feed=...)` sets it on the feed catch skill only) — this is the piece
  that makes a BB-fed columns start admissible under the box at all; without it the feed catch
  refuses `LIMIT_VEL` 366.5 > 300, with it the feed catch plans at hand 0.951.
- **A pre-throw feasibility check** on the columns announcement path: before ball A is thrown,
  the node plans A's throw and B's feed catch offline; a refusal ends the attempt
  `REJECTED_COLUMNS_FEED_UNCATCHABLE` naming the limit and the landing offset, platform at rest
  holding A, rather than stranding ball A in the air under an uncatchable feed.

**Three places especially worth a sanity check from the person standing next to the robot:**

- **The receive-level catch is close to level, not banked** — watch whether the feed catch (ball
  B, arriving at ~5.6 m/s and 11.9 deg off vertical from Ball Butler's current perch) still looks
  like a controlled catch at a near-zero platform tilt, the way the 0 deg self_toss reloads did
  in sitting 1, or like the cup taking the ball mostly on its side.
- **BB firmware FW 5 is untested on this robot** — the settle criterion changed from
  position-only to position-and-rate-and-in-band; watch for any NEW refusal shape (not just fewer
  of the old one), and watch the retry-once line (`reload: Ball Butler was not settled --
  retrying the throw once`) actually fire and clear rather than compounding.
- **The BB-fed columns block is the first time this rung asks Ball Butler to feed a live,
  running pattern rather than a single reload** — expect more misses and refusals than a plain
  reload while the timing settles; a `REJECTED_COLUMNS_FEED_UNCATCHABLE` naming a landing offset
  is data, not a failure (same posture as sitting 1's lob misses).

## 1. Before the robot is powered (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at or after tonight's fix-unit commits (receive_tilt, the pre-throw feed check, the bias correction, the Jetson retry-once — named in the handoffs this sitting cites). |
| 2 | (ROS) `cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash && cd ..` | Builds clean. Single package this sitting (`InstallSegment.srv` did not change tonight, unlike sitting 1's two-package build) — if a later handoff DID touch a `.srv`/`.action`, build `jugglebot_interfaces jugglebot` instead. |
| 3 | (venv) `PYTHONPATH=ros_ws/src/jugglebot python -c "from jugglebot.motion.skills import admissible as ab; print(ab.gate_hash())"` then `grep -m1 gate_hash config/generated/admissible_box.yaml` | The two hashes EQUAL. Tonight's receive_tilt edits touched two gated files (`unified_cycle.py`, `segments.py`), which moves the live gate hash — a box sweep against the new hash was reported running late tonight but its completion was not confirmed at handoff time (one handoff still saw the OLD box, stamped `581109806b4e`, against a live gate of `f96fc30012d7`). **Do not assume a match** — check live, and if it does not match, do not fly, re-sweep first (`python tools/admissible_sweep.py --site-pairs all --single-apex 0.5 0.6 0.7 0.8 0.9`, ~35 min). |
| 3a | (venv) `grep -m1 "leg_jerk_mmps3" config/generated/admissible_box.yaml` | Reads `200000.0` — must match row 8 of § 2's `set_limits`. |
| 4 | `cat temp/reports/nightly/status` (or the nightly ticker per CLAUDE.md) | Fresh GREEN, or a RED read against `git status`/`git stash list` before trusting it. |
| 5 | (venv) `./run_tests.sh --full 2>&1 \| tee temp/logs/presitting_full_r5_sitting2_$(date +%Y%m%d).log` — launch DOWN | `RESULT: PASS`. Record the pass count and wall time in § 11. Not run by this unit — the main session runs the gate before the sitting. |
| 6 | **BB firmware FW 5 flash** — launch DOWN, Claude runs it: `cd ~/Desktop/BallButler/ball_butler_main && pio run -e teensy40_can -t upload` | Receipt = the tool's own `"FW version: 4 -> 5"` line — an md5-only match is NOT a flash, same rule as the can-bridge/Platform Teensies. If FW 5 was already flashed before this sitting (check the boot banner once the launch is up, § 2 row 8b), skip this row. |
| 6a | **If FW 5 is not flashed** (skipped, or the flash could not be confirmed) | Nothing on the Jetson reads BB's firmware version at runtime — a board still on FW 4 is SILENT, not dark: no error, no refused command, just the sitting-1 refusal rate unchanged (5/19-style yaw `THROW_ABORTED_NOT_SETTLED`, position-only settle). Flag this in § 11 before Blocks 4-6 if it happens; the Jetson-side retry-once (§ 0) still helps either way. |
| 7 | Home the hand (GUI Home, or `/recover` if it is not already parked) **before** Activate | Unchanged from R4/R5 sheet § 1 — the 2026-09-23 upper-stop push is still on watch. |
| 8 | Base level (R4 sheet § 1 row 7's two options, unchanged) | Either is flyable. |
| 9 | QTM: `Catching Cone` rigid body DISABLED, Ball Butler reflectors MASKED (for reload/columns blocks); `Platform` rigid body defined and tracked | Hard preconditions, unchanged from R3/R4/R5 sitting 1. **Ball Butler reflectors are UNMASKED specifically for § 4's accuracy volley** — remask before § 5/§ 6. |

## 2. Bring-up (launch UP, robot powered, ball present)

Run R4-sheet rows 8b-12 with these numbers — **no separate ramp block this sitting** (sitting 1's
§ 2.5 already held at 200 000; if `lead_clamp_mask`/`torque_clamp_mask` go non-zero or a guard
latches anywhere in this sitting, treat it exactly as sitting 1's pre-registered stop rule would
have: revert to 150 000, re-check `skills/check`'s box match, and fly the rest of the sitting
there).

| # | Step | Expect |
|---|---|---|
| 8b | `level` FIRST, before any skill dispatch | `level` completes; `trajectory/status.gravity_correction_loaded` reads fresh. |
| 9 | `ros2 action list \| grep jugglebot/juggle` ; `ros2 service list \| grep -E 'skills/check\|jugglebot/juggle_stop\|bb/start_accuracy_calibration'` | All present. |
| 10 | Confirm the boot banner's BB firmware line (if printed) or ask BB's own version query, per § 1 row 6a | FW 5 if flashed; note whichever it reads in § 11. |
| 11 | `ros2 param set /skill_node apex_m 0.9` ; `dwell_s 0.30` ; `plant_id r5-sitting2-$(date +%Y%m%d)` — a FRESH `plant_id` (cold learner memory) | Set. |
| 12 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 300.0, leg_acc_limit_mmps2: 5000.0, leg_jerk_limit_mmps3: 200000.0}"` — MUST run before row 14 / any `skills/check` or dress-rehearsal call | `applied_*` echoes `300 / 5000 / 200000`. |
| 13 | Seat a ball in the hand; confirm on `/hand_telemetry` (`ball_held_valid: true`, `ball_held_raw: true`) | Visual + topic check. |

## 3. The dress rehearsal (kept — the owner has not been running these)

Sitting 1 skipped this section entirely (owner: does not run it). It stays in the sheet as the
reference procedure — run it if there is time or if anything in §§ 4-6 refuses in a way that
looks schedule-side rather than hardware-side, since it is the fastest way to separate the two.

| # | Step | Expect |
|---|---|---|
| 14 | (venv, no ROS) `python3 tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss --arm A --attempts 3` | Three clean attempts, verdict G1/G2/G4 PASS. |
| 15 | (system python3, ROS sourced, robot ACTIVATE only, never ARM) `python3 tests/hardware/skills_plan_bench.py --via-action --pattern self-toss --n-throws 3` | `outcome=COMPLETED`, 3/3 dispatched, DISARMED. |
| 16 | Same, `--pattern columns --separation-mm 100 --n-throws 3 --reload` — DISARMED | Rehearses the BB-reload columns ladder (box lookup for `(P1,P1)`/`(P2,P2)`) without moving anything. |
| 17 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK`, `box OK` lines including the columns site pairs at the 0.90 m apex band. |

## 4. Ball Butler accuracy-volley re-fit

The fitting tool lives in the BallButler repo, not this one:
`~/Desktop/BallButler/zTesting/throw_testing/accuracy_testing/fit_affine.py`, with its own
`README.md` in the same directory. Read-only reference for this checkout; do not edit the
BallButler repo from here.

**What the volley needs.** A fresh BB self-calibration already latched
(`self._bb_position_mm is not None`, from `bb/calibration_result`) -- this needs Ball Butler's OWN
reflectors visible to QTM and unmasked (row 18 below, the same precondition § 1 row 9 states for
the whole sitting). BB's own hopper loaded with enough balls for up to 50 throws (`calib_throws_per_cell`
2 x a 5x5 `calib_grid_divisions` grid) -- the volley reloads itself between throws via its own
`bb/reload` cycle and BB's own heartbeat (`calib_min_reload_heartbeats=3`), no operator hand-reload
between throws. There is no physical target board or cone for this block: `calib_grid_center_mm`
(default `[0, 0, 750]` mm) and `calib_grid_size_mm` (default `[1000, 1000]` mm) describe a virtual
5x5 grid of aim points in the same global/mocap frame the rest of the sitting uses, not a marked
object -- the QTM "Catching Cone" rigid body stays DISABLED throughout, per § 1 row 9, unchanged
for this block. **(operator confirms)** whether the grid's default center overlaps the platform's
physical envelope and, if so, clears or guards that airspace before starting (recommend the
platform parked / DEACTIVATED for this block), and where the thrown balls' physical catch surface
(net, mat, or floor) should sit. QTM must be recording continuously through the whole volley --
there is no per-throw record trigger.

**How it is started and stopped.** `ros2 service call bb/start_accuracy_calibration
std_srvs/srv/Trigger` fires the whole scheduled sequence (up to 50 throws) unattended; it ends on
its own once every scheduled throw has fired. To cut it short:
`ros2 service call bb/cancel_accuracy_calibration std_srvs/srv/Trigger` (**operator confirms** --
not exercised this sitting).

**Where things land.** `ball_butler_node` writes a session metadata JSON to
`~/bb_calibration_sessions/accuracy_session_<timestamp>.json` -- the randomised schedule of AIMED
targets (cell index, global x/y/z, the BB pose and yaw offset it was aimed from), not the observed
landings and not a fitted correction matrix. The service does NOT itself measure where the balls
land -- that is QTM's job: once the volley completes, export the recording to a trajectory JSON
from QTM's own UI (**operator confirms** the exact export naming/location; the BallButler repo's
historical convention there is a dated descriptive name, e.g.
`2026-06-10-BB-calibration-32-throws-CORRECTED.json`, kept alongside `fit_affine.py`).

**The fit.** Turning the volley's aimed-vs-landed pairs into a new 2D affine is a separate,
offline step, run from the `accuracy_testing` directory. Verbatim from that directory's
`README.md`:
```
python fit_affine.py \
  ~/bb_calibration_sessions/accuracy_session_<timestamp>.json \
  ~/Desktop/qtm_export.json \
  --out throw_affine_correction.json
```
It pairs each QTM-tracked landing to its commanded target by throw order (no manual
cluster-mapping), transforms both into BB-local frame using the session's own recorded BB
pose/yaw, fits the 2x3 affine, and prints a before/after error report while writing the matrix
JSON (`matrix` + `n_pairs` + `provenance`). Needs at least 3 paired throws. `--plane-z`,
`--skip-throws`, and `--no-plot` exist as optional overrides on the script itself; not needed for
the default 750 mm-center grid.

**Which copy of `throw_affine_correction.json` the node loads, and whether a build is needed.**
The node loads it ONCE, in `__init__`, gated by the `apply_aim_correction` parameter (launch-arg
default `true`, `jugglebot_launch.py`) and `aim_correction_file` (default
`throw_affine_correction.json`, a bare name). `_load_aim_correction` resolves that name via
`get_package_share_directory('jugglebot')/resources/...` -- **the INSTALL copy**
(`ros_ws/install/jugglebot/share/jugglebot/resources/throw_affine_correction.json`), not the
source tree directly. The source of truth to EDIT is
`ros_ws/src/jugglebot/resources/throw_affine_correction.json`; a `colcon build` copies it into the
install tree. **There is no live-reload service or parameter for this matrix** -- after editing
the source file and rebuilding, `ball_butler_node` must be RELAUNCHED (a fresh `__init__` call)
for the new matrix to take effect; a `ros2 param set` alone does nothing here.

**Sanity check, before deploying.** Read the new matrix's own `provenance` block: `n_pairs`
(should sit well above the 3-throw floor and close to the throw count the volley actually
completed, not the stale deployed fit's 41), `mean_error_before_mm` / `mean_error_after_mm` (the
script's printed report shows the same two numbers), and `bb_yaw_offset_rad_at_fit` converted to
degrees. Compare that yaw figure against this sitting's own live BB self-calibration reading (the
`mocap_node` "BB calibrated: ... yaw offset ..." INFO line in this sitting's own launch log, from
bring-up). The stale fit being replaced carries `bb_yaw_offset_rad_at_fit = -0.14582 rad =
-8.35 deg` (recorded 2026-06-09), against sitting 1's live self-calibration of `+1.95 deg` on
2026-09-30 (`logbook/2026-09-30-skill-stack-r5-sitting-1.md`, scratchpad `y_bias_report.md`) -- a
10.3 deg mismatch that is the leading suspect for the +26.9 mm y-bias this re-fit exists to fix.
The new fit's yaw should land within a degree or two of today's live reading, not tens of degrees
off; if it does not, do not deploy -- re-check that BB's self-calibration was current when the
volley ran (`bb_yaw_offset_rad` in the session metadata JSON carries the same value) before
trusting the fit.

**Acceptance.** Deploy, then run 3 reloads at `hold_tilt_max_deg 12.0` (self_toss, GUI Reload) and
`python tools/probes/feed_catch_bag_probe.py --bag <bag> --log <launch.log>`. Target: landing
table `|y|` bias under 10 mm, down from sitting 1's measured +26.9 mm (stdev 4.7 mm,
`y_bias_report.md`).

| # | Step | Expect |
|---|---|---|
| 18 | QTM: unmask Ball Butler reflectors (§ 1 row 9). Confirm BB's hopper is loaded, the grid airspace is clear per the note above, and QTM is recording. | BB rigid body tracked; `/bb/calibration_result` fresh. |
| 19 | `ros2 service call bb/start_accuracy_calibration std_srvs/srv/Trigger` | `success: true`, message names the throw count and the session JSON path, e.g. `Accuracy calibration started: N throws. Metadata at <path>`. |
| 20 | Let the volley run to completion (up to 50 throws, BB self-reloads between each) | `Accuracy calibration starting: N throws to M/25 reachable cells`, then one INFO line per throw; ends with a completion log (not independently confirmed by this unit, read the live line). |
| 21 | Stop the QTM recording and export the trajectory to a JSON file, then run the fit command quoted above from the `accuracy_testing` directory | Printed before/after error report (mean/max/std, mm); `throw_affine_correction.json` and a `.png` written to that directory. |
| 22 | Sanity check per the note above: `n_pairs`, `mean_error_after_mm`, and `bb_yaw_offset_rad_at_fit` (degrees) against today's live self-calibration | `n_pairs` well above 3 and near the completed throw count, not the stale 41; yaw within about 1-2 deg of the live reading, not the stale fit's 10.3 deg mismatch. If not, do not deploy, re-check the volley's live BB calibration first. |
| 23 | Copy the new matrix to `ros_ws/src/jugglebot/resources/throw_affine_correction.json`, then `cd ros_ws && colcon build --packages-select jugglebot && source install/setup.bash && cd ..`, then RELAUNCH (not just a param set) | Fresh matrix loaded; boot log line `Aim correction loaded from .../throw_affine_correction.json` (DEBUG) confirms it, `n_pairs` differs from the stale 41. |
| 24 | QTM: remask Ball Butler reflectors (back to the standing precondition). Verify: 3 reloads at `hold_tilt_max_deg 12.0` (self_toss, GUI Reload), then `python tools/probes/feed_catch_bag_probe.py --bag <bag> --log <launch.log>` | Landing table `\|y\|` bias < 10 mm (down from sitting 1's measured +26.9 mm, stdev 4.7). If not met, record the new bias and treat § 5/§ 6 numbers as still carrying the old lateral offset. |

## 5. The 0 deg gate (pre-registered)

**Criterion, verbatim (sitting2_facts, owner decision 2026-09-30 evening)**: 10 BB feeds at
`hold_tilt_max_deg 0.0` (self_toss reload), **>= 9 caught AND >= 8 smooth seats (`seat=` in
+0.05..+0.15 s) AND the probe's landing-vs-committed within +/-30 ms mean**. The owner films 3
catches on the high-speed camera. Count `THROW_ABORTED_NOT_SETTLED` refusals against sitting 1's
5/19 baseline and note any `"reload: Ball Butler was not settled -- retrying the throw once"`
line firing.

**Probe command** (same tool as sitting 1, re-run after this block):
`python tools/probes/feed_catch_bag_probe.py --bag <bag> --log <launch.log>`

| # | Step | Expect |
|---|---|---|
| 25 | `ros2 param set /skill_node hold_tilt_max_deg 0.0` | `Set parameter successful`. Takes effect on the NEXT BB announcement (read once per `_install_announced_reload`), not retroactively. |
| 26 | GUI: pattern self_toss, Reload first, hold-to-confirm Start. Repeat until 10 feeds are logged (count refusals separately, they do not count toward the 10). | Debug line reads `hold tilt capped at 0.0 deg, resulting hold <=0.000 deg`. Record per feed: caught Y/N, `seat=`, any `THROW_ABORTED_NOT_SETTLED` (and whether the retry-once line fired and cleared it). |
| 27 | High-speed camera: film 3 of the 10 catches | Filmed, noted which throw indices. |
| 28 | `python tools/probes/feed_catch_bag_probe.py --bag <bag> --log <launch.log>` | Per-feed landing-vs-committed table; compute the mean against the +/-30 ms band. |
| 29 | Verdict against the criterion above | Record PASS/FAIL and every sub-metric (caught count, smooth-seat count, mean landing-vs-committed, refusal count vs 5/19 baseline) in § 11 before moving to § 6. |

## 6. Ball Butler-fed columns

`ros2 param set /skill_node separation_mm 100.0` — the columns operating point
(`sites.columns_sites(100.0)`: P1 (-50, 0), P2 (+50, 0) mm; catch plane z 830, release z 860).
`ros2 param set /skill_node catch_resend_max 0` for this block specifically (§ 2 row 10a's
convention from sitting 1 — revert to the operator's live default afterward). Set
`hold_tilt_max_deg` back to whatever Block-A-style patterns need next (12.0 is the R4-flown,
F-b-measured operating point) — **this does not gate the columns feed catch itself**, which now
uses the fixed receive-level attitude (§ 0) regardless of `hold_tilt_max_deg`.

Unlike sitting 1's Block C (`reload: false`, a human lob), this goal sets **`reload: true`**,
which routes through the SAME bridge-then-BB-round-trip choreography the ordinary one-ball reload
uses (`_start_reload(kind='columns', aim_site=P2, holds_ball=True)`), generalised rather than
duplicated — so BB is asked for a real throw via `bb/reload`/`bb/throw`, not a tracker-resolved
human lob.

**Expected console lines, quoted verbatim from `skill_node.py`** (grepped this checkout; all
three exist as written, none are placeholder):
- On a good feed: `"columns feed accepted: ball B lands in %.2f s -- catching it, then %d
  throw%s"` (INFO) — followed by the catch of ball B, then the alternating throws.
- On an unplannable feed: end code `REJECTED_COLUMNS_FEED_UNCATCHABLE`, message `"columns feed:
  the feed catch of ball B cannot be planned -- %s -- announced landing %.1f mm off site %s
  (dx %+.1f, dy %+.1f mm) -- no columns schedule installed; the platform stays at rest holding
  ball A (the bridge REST is already streaming)"` — this is the pre-throw feasibility check (§ 0)
  firing before ball A is thrown, costing about 56 ms at the announcement (measured, well inside
  budget). The known cliff (measured against the real executor terminal, whose 2026-09-28 rule
  has the carried release ride the caught xy) is a landing **+30 mm in x beyond P2** (further
  from P1) — `0` to `+20` mm plan clean, `+30` mm is the smallest that refuses (`LIMIT_VEL`
  323.8 mm/s, 108 % of the 300 mm/s cap), and the cliff steepens from there (+40 mm: 358.4 mm/s,
  119 %; +60 mm: 428.5 mm/s, 143 %; +100 mm: QP infeasible). The cliff is directional — offsets
  toward P1 (negative x) all plan clean up to at least -100 mm.
- On a settle refusal during the reload round trip: `"reload: Ball Butler was not settled --
  retrying the throw once"` (INFO) — the SAME line as § 5, since `kind='columns'` shares
  `_on_bb_throw_outcome`'s retry path with the ordinary reload.

There is no distinct "columns started: bridge REST holds ball A..." announcement line for
`reload: true` (that line is specific to sitting 1's retired `reload: false` human-lob path) —
the first line this block prints, once BB's announcement resolves, is the "columns feed accepted"
line above.

| # | Step | Expect |
|---|---|---|
| 30 | `ros2 param set /skill_node separation_mm 100.0` ; `catch_resend_max 0` | Both `Set parameter successful`. |
| 31 | Seat ball A in the hand at P1. `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: columns, apex_m: 0.9, separation_mm: 100.0, num_cycles: 6, reload: true}"` | Goal ACCEPTED; the bridge REST streams holding ball A; BB is asked to throw at P2. |
| 32 | Watch for the "columns feed accepted" line, or `REJECTED_COLUMNS_FEED_UNCATCHABLE` naming the offset, or a settle refusal + retry | Record which fired, and (if accepted) the catch of ball B and the number of consecutive alternating throws after it (the learning curve, not a pass/fail count — box apex band 0.90 m). |
| 33 | **D2 -- the Stop**: end an attempt deliberately (GUI Stop / `jugglebot/juggle_stop`) mid-chain. Carried from sitting 1 § 6 row 37. | The last throw is aimed at the site where the held ball is; platform comes to rest on the platform holding both, not resumable, operator end line as sitting 1's sheet quotes it. Clear the platform by hand before the next attempt. |
| 34 | **D3 -- a drop with the survivor still in flight**: do not engineer this, just record it if it happens. Carried from sitting 1 § 6 row 38. | Survivor's next CATCH dispatched without its carried throw, then a closing REST. End code `DROPPED_SURVIVOR_STOPPED`. No memory row for the dropped ball's flight. |
| 35 | `ros2 service call skills/check std_srvs/srv/Trigger` after each attempt | Still clean. |
| 36 | Once a `num_cycles: 6` attempt lands its feed cleanly and completes, repeat row 31 with `num_cycles: 20` | Longer chain, same watch list. |

**Stop rule for this block**: any guard latch or `HAND_LANE_REFUSED` ends the block; a feed miss
with a ball on the floor ends the attempt (clear by hand) — same as sitting 1's Block C stop
rule.

## 7. Pre-registered placement fallback

If the 0 deg gate (§ 5) FAILS: move Ball Butler to a new placement ~0.5 m from the cup, at the
same height, throwing a near-vertical lob (pitch ~84 deg) rather than the current ~1.1 m-away,
69.7 deg-pitch throw. This closes the arrival angle enough (per the day-1 planner probes:
release within ~0.25 m of the catch site at ~1.0 m above the cup plane gives <= 5 deg at
<= 4.5 m/s) that the stock (non-receive-level) catch handles it without needing the 0 deg hold to
land. Re-run § 4's accuracy volley after moving BB — the aim affine is fitted at a specific BB
pose, and a placement change invalidates it exactly the way tonight's 2026-09-27 recalibration
did.

**Decision criterion**: this fallback is exercised ONLY if § 5's 0 deg gate does not meet
>= 9/10 caught, >= 8/10 smooth seats, and landing-vs-committed within +/-30 ms. It is not a
consolation prize — it is the owner's own pre-registered next move (sitting 1's D1/F-c decision
trail, plan § R5).

## 8. Pre-registered verdict table

| Item | Criterion | Result |
|---|---|---|
| BB accuracy volley | landing table \|y\| bias < 10 mm on 3 reloads at 12 deg | PENDING |
| 0 deg gate | >= 9/10 caught AND >= 8/10 smooth seats AND landing-vs-committed within +/-30 ms mean | PENDING |
| BB-fed columns, num_cycles 6 | feed accepted, ball B caught, consecutive throws logged (data, not pass/fail) | PENDING |
| BB-fed columns, num_cycles 20 | longer chain flown after a clean 6-cycle feed | PENDING |
| D2 Stop | observed and matches the cross-site design | PENDING |
| D3 drop-with-survivor | observed if it happens naturally; matches the designed policy if so | PENDING |
| Fallback exercised? | only if the 0 deg gate FAILS | PENDING |

## 9. Refusal table

R3/R4/R5-sitting-1 sheets have the full launch/precondition refusal table
(`REJECTED_MOCAP_STALE`, `REJECTED_NOT_LEVELLED`, `REJECTED_HAND_STALE`, `REJECTED_BALL_UNKNOWN`,
`REJECTED_NO_BALL`, `REJECTED_FRAME_OFFSET`, `ABORTED_MODE_CHANGED`, `ABORTED_NO_RELEASE`,
`NO_ADMISSIBLE_COMMAND`, `NO_LANDING`, `SPLICE_TOO_LATE`, `WINDOW_TOO_SHORT`, `UNREACHABLE`,
`LIMIT_JERK`/`LIMIT_ACC`/`LIMIT_VEL`/`HAND_STROKE`/`HAND_ACC`, `STALE_STATE`/`WRONG_MODE`,
`CUP_CONTACT_ACC`, `GUARD_LATCHED`, `SUPERSEDED_BY_HOLD`, `REJECTED_BB(<message>)`,
`ABORTED_BB_NOT_READY(<state>, ball_in_hand=<bool>)`, `ORIGIN_TOO_LATE`, `ABORTED_BB_THROW_TIMEOUT`,
`ABORTED_NO_ANNOUNCEMENT`, `LimitsMismatch`, `ABORTED_NO_COLUMNS_FEED`,
`DROPPED_SURVIVOR_STOPPED`) — unchanged this sitting, still applies verbatim. New or specifically
relevant this sitting:

| Code / line | Physical fact |
|---|---|
| `REJECTED_COLUMNS_FEED_UNCATCHABLE` | The pre-throw feasibility check (built tonight) found ball B's feed catch cannot be planned before ball A was ever thrown -- the platform stays at rest holding A. Message names the refusal reason and the landing offset from site P2 in mm (dx, dy). The known cliff is directional: 0 to +20 mm in x beyond P2 plan clean, +30 mm is the smallest that refuses (LIMIT_VEL 323.8 mm/s, 108 percent of the 300 cap), steepening at +40/+60/+100 mm; offsets toward P1 (negative x) plan clean to at least -100 mm. |
| `"reload: Ball Butler was not settled -- retrying the throw once"` (INFO) | The Jetson-side retry-once fired on a literal `THROW_ABORTED_NOT_SETTLED` from BB during the reload/columns round trip. A SECOND refusal on the same attempt ends it `REJECTED_BB(THROW_ABORTED_NOT_SETTLED)` -- no further retry. |
| BB serial abort line `settled=N/15 rate=R` (FW 5, off-Jetson) | The new firmware settle criterion: error inside 1.0 deg for the last 15 samples at 150 Hz AND rate <= 12 deg/s AND currently in-band. If BB is still on FW 4, this line will not appear at all (§ 1 row 6a). |
| `hold tilt capped at %.1f deg, resulting hold %.3f deg` (DEBUG) / `(hold tilt %.1f deg, cap %.1f deg)` (operator INFO) | `hold_tilt_max_deg` in effect for a plain self_toss/hop reload -- read the resulting angle against the requested cap. Does NOT apply to the columns feed catch, which uses the fixed receive-level attitude regardless of this parameter. |
| `LimitsMismatch` | Unchanged from sitting 1 -- the box on disk was swept at a different limit than the live session, or `set_limits` ran after `skills/check`. |

## 10. Close-out

| # | Step | Expect |
|---|---|---|
| 37 | GUI Deactivate (or the orchestrator command topic) | Robot stows. |
| 38 | Stop the launch and the load capture. | |
| 39 | `python tools/probes/throw_outcome_bag_probe.py --bag <id>` | Independent ground-truth check against `skill_node`'s own logged memory rows, same as R3/R4/R5 sitting 1. |
| 40 | `python tools/probes/feed_catch_bag_probe.py --bag <bag> --log <launch.log>` (final pass, whole sitting) | Per-feed landing table and lateral-bias table for the whole sitting, for the logbook entry's Verification section. |
| 41 | Copy `temp/learn/r5-sitting2-<date>/memory.csv` to a dated path. | |
| 42 | Send: the bag folder name, the memory.csv copy, the loadavg/launch logs, the 0 deg gate verdict, and the throw-by-throw tables from §§ 4-6. | |
| 43 | Log the sitting (`/log feature`, or `/investigate` if anything in § 9 fired unexpectedly or a catch showed a rebound signature). Update plan § R5 Outcome and the memory file. | |

## 11. Results

(blank -- fill in after the sitting)

### BB accuracy-volley re-fit (§ 4)

| Item | Result |
|---|---|
| Volley completed (throws / cells) | |
| Fit procedure used | |
| New matrix `n_pairs` / source | |
| Rebuild + relaunch confirmed | |
| Verify: 3 reloads at 12 deg, \|y\| bias | |

### The 0 deg gate (§ 5)

| Item | Result |
|---|---|
| Feeds attempted / caught | |
| Smooth seats (+0.05..+0.15 s) | |
| `THROW_ABORTED_NOT_SETTLED` refusals (vs 5/19 baseline) | |
| Retry-once fired and cleared? | |
| Landing-vs-committed mean (probe) | |
| High-speed camera clips (3) | |
| **Verdict (PASS / FAIL against § 5's criterion)** | |

### Ball Butler-fed columns (§ 6)

| Attempt | `num_cycles` | Feed outcome (accepted / `REJECTED_COLUMNS_FEED_UNCATCHABLE` / settle refusal) | Ball B caught | Consecutive catches after | Notes |
|---|---|---|---|---|---|
| | | | | | |

**D2 Stop observed?**
**D3 drop-with-survivor observed?**

### Fallback (§ 7)

**Exercised?**

### Bag / boot-banner record

| Rung | Time | Bag folder | Notes |
|---|---|---|---|
| Pre-power / FW 5 flash | | | |
| BB accuracy volley | | | |
| 0 deg gate | | | |
| BB-fed columns | | | |
