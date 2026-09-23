# R4 hardware runsheet — two sites, one ball, the Ball Butler reset

Skill-stack R4 (`plans/active/two-ball-skill-stack.md` § R4). Same machine as
`session_skills_r3.md` and `session_cup_contact.md` — **those two sheets stay
the reference for everything this one does not restate** (QTM preconditions,
bring-up rows 11–20, the guard/`/recover`/hand-homing recovery flow, the
cup-contact dive contract). Driver: `tests/hardware/skills_plan_bench.py`
(`--rehearse`, `--via-action`). Node under test: `skill_node.py` — the
`jugglebot/juggle` action, patterns `self_toss` / `hop`; `skills/check`;
`jugglebot/juggle_stop`.

**What this sitting is, in six lines.**
1. Two sites, P1/P2, 250 mm apart (hop, D2). One ball hops P1→P2→P1…; one
   hand. `jugglebot/juggle` (D3) is now the ONE start surface — the Trigger
   services `skills/start_self_toss`/`skills/start_columns`/`skills/stop`
   this repo used through R3 are **deleted**; `skills/check` is unaffected.
2. Reload is a **faithful port of the FSM's own choreography** (D4), re-cut
   as a CATCH skill: level→**PRE-TILT REST** (≥1.0 s) → **held-tilt CATCH**
   (cup opening on the 12° receive axis through Ball Butler's announced
   landing, 29.8 mm off the site at the ceiling) → **DECAY REST** (tilt→
   level, 1.0 s) → the pattern's own throws. **This exact catch shape has
   never been flown** — U2's probes are offline-only.
3. Gate (plan § R4): **10 consecutive catches alternating P1/P2** on
   hardware, and a BB reload → catch → throw → catch chain dispatched from
   the GUI with one button.
4. Session limits unchanged from R2/R3: legs **300 / 5000 / 150 000 mm/s³**,
   hand **3500 rev/s²** (torque FF `K = 0.7`, launch default — nothing to set
   by hand, plan § 0).
5. The FSM is **deleted outright** (tag `fsm-final`, U6): every coordinator,
   sequencer, action and Trigger service this repo used through R3 for reload
   and toss is gone. If a step below references one, that is this sheet's
   bug, not yours — stop and say so.
6. Rung order matters: dress rehearsal → self-toss regression (nothing
   physical changed for it) → hop singles → hop chained (the R4 gate) →
   reload (self_toss first, then hop) — each rung is a smaller, better-
   understood bet than the one after it.

---

## 0. Why this sitting, and what you can push back on

R3 threw one ball at one site with a learner running. R4 adds a second site
the same ball hops between, and — for the first time — lets an external
thrower (Ball Butler) put a ball in the cup while the platform holds a tilt.
**If your physical intuition disagrees with the framing below — the site
separation, the receive tilt, the session limits, anything else — that is
load-bearing signal, say so before we start.**

Three places especially worth a sanity check from the person standing next
to the robot:

- **The receive tilt is a real platform attitude, held while the ball is
  moving toward the cup.** The owner's own physics (2026-09-23, quoted in
  the U2 probe notes): *"if the Platform is tilted to face the oncoming
  ball, the hand only needs to move along its usual linear axis to match the
  ball's vertical and lateral velocities."* Confirmed by the RELOAD
  sequence's FSM-era track record ("working flawlessly," same note) — but
  this is a NEW code path re-deriving that choreography from scratch, on a
  planner that has changed since (unified cycle, cup-contact contract). Watch
  the pre-tilt REST settle before trusting the catch that follows it.
- **The hop's apex band is narrow (0.850–0.900 m) and the plant may be
  running fast.** R3's apex ladder found the plant ~8 % fast in apex at
  K=0.7 (not fully corrected). If the hop plant runs similarly fast, the
  learner has nowhere to command BELOW 0.85 m to compensate — it plateaus
  out of band by construction (U7a finding, § 5). This is a decision input
  for the owner, not a sitting failure.
- **The reload catch has never left the simulator.** Every number in U2's
  probe notes (`scratchpad/logbook_r4_notes.md`) is analytical or MuJoCo —
  the first real one is the one you are about to fly. Keep a hand on the
  E-stop for it (§ 6).

## What changed since R3 / cup-contact (read once, applies everywhere below)

- **`jugglebot/juggle` is the one start surface** (an action: pattern +
  apex_m + separation_mm + num_cycles + reload; `pattern` is REQUIRED, no
  default). `jugglebot/juggle_stop` (Trigger) cancels; a goal CANCEL is the
  same "cancel on stop" every prior sheet's `Ctrl-C` already relied on.
  `skills/check` is unchanged in shape.
- **`separation_mm`'s launch default is still 100 mm** (the columns/R2
  value) — every hop goal in this sheet must name `separation_mm: 250.0`
  explicitly. The GUI's relay (`orchestrator_node._svc_juggle_request`)
  ALWAYS sends `apex_m=0.0 / separation_mm=0.0 / num_cycles=0` (the "use the
  node's own parameter" sentinel) — it has no fields for these. **Before any
  GUI-driven hop or hop+reload (§ 6), set the node's own parameters first**:
  `ros2 param set /skill_node separation_mm 250.0` (and `n_throws` if you
  want other than the launch default of 4) — the GUI cannot override them
  per click.
- **`skills/check` now certifies BOTH the self_toss box and the hop's two boxes** at the
  node's own `separation_mm` / `apex_m` (`_svc_check`, since `1d6d7f5`): expect
  `box OK: ...; hop box OK: P1->P2 at 250.0 mm; hop box OK: P2->P1 at 250.0 mm`, or
  `hop box REFUSED: no hop box covers P1->P2 at separation 250.0 mm, apex 0.900 m`
  (a refusal at the dry run, not at the goal). §1 row 3a's `grep -c "pattern: hop"` is
  belt and braces, not the only pre-power check any more.
- **`InstallSegment.srv` gained four fields** (`hold_tilt_set` + `hold_tilt_rad`,
  `rest_tilt_set` + `rest_tilt_rad`) for the held-axis catch and attitude-bearing REST —
  the `*_set` flag says a tilt is given (rosidl zero-fills the array, and (0, 0) is a real
  level target, so zeros can never mean "none"); a stale `jugglebot_interfaces` install
  is DARK for a reload goal (the fields do not exist), so the colcon build in §1 is
  mandatory, not optional.
- **Reload is real** (D4): `REJECTED_RELOAD_RETIRED_R1` is gone. A
  `reload: true` goal opens with `bb/reload` + `bb/throw_at_target`, then
  compiles the real schedule off Ball Butler's `ThrowAnnouncement`
  (`skill_node._on_announcement`) — nothing here is speculative about BB's
  own throw, it is the SAME external-thrower path the FSM flew. `columns` +
  `reload` is refused until R5 (not flown this sitting).
- **The reload always targets `pattern.sites[0]`** — for hop that is P1 (the
  bridge REST parks the cup there while BB's round trip is in flight), never
  "wherever the cup happens to be" — the FSM's "whatever site the cup is at"
  framing (checklist D4) collapses to a fixed site for a schedule that has
  not thrown yet.
- **The seam tilt-RATE pin scoped in D6 was NOT built** — the real defect
  found during U4 (a stale seam `pose_vel` in the splice `_join`) fixed the
  same symptom a different way (the splice now re-derives the seam knot's
  commanded velocity from the joined series); seams measured **within 0.05×
  the jerk bound** after the fix. If a re-aimed catch's seam looks rough on
  the day, that is new territory, not the known-masked gap plan § 0 used to
  name.
- **The frame-check bound is 25 mm now, not 5 mm** (cup-contact-contract §
  1, carried in) — an offset under 25 mm is measured and **subtracted**, not
  refused; `learner_lateral_authority_mm` launch default is **20 mm**.

## 1. Before the robot is powered (no ROS, any time)

| # | Step | Expect |
|---|---|---|
| 1 | `cd ~/Desktop/Jugglebot-skills && git fetch && git status -sb && git log --oneline -1` | On `skill-stack`, in sync, at or after the `fsm-final` tag and the R4 commits. |
| 2 | (ROS) `cd ros_ws && colcon build --packages-select jugglebot_interfaces jugglebot && source install/setup.bash && cd ..` | Builds clean. **Mandatory** — `Juggle.action` and `InstallSegment.srv`'s two new fields are new interface definitions; a stale install has no `jugglebot/juggle` action at all (dark, not degraded) or silently drops `hold_tilt_rad`/`rest_tilt_rad`. |
| 3 | (venv) `PYTHONPATH=ros_ws/src/jugglebot python -c "from jugglebot.motion.skills import admissible as ab; print(ab.gate_hash())"` then `grep -m1 gate_hash config/generated/admissible_box.yaml` | The two hashes EQUAL. If not: `python tools/admissible_sweep.py --site-pairs all --single-apex 0.5 0.6 0.7 0.8 0.9` (~35 min) before flying — do not fly on a stale box (§ "what changed", `skills/check` cannot see this for hop). |
| 3a | (venv) `grep -c "pattern: hop" config/generated/admissible_box.yaml` | ≥ 2 (`(P1,P2)` and `(P2,P1)`) — confirms the box actually has hop rows, independent of the gate-hash check above. |
| 4 | `cat temp/reports/nightly/status` (or the nightly ticker per CLAUDE.md) | Fresh GREEN, or a RED read against `git status`/`git stash list` before trusting it. |
| 5 | (venv) `./run_tests.sh --full 2>&1 | tee temp/logs/presitting_full_r4_$(date +%Y%m%d).log` — launch DOWN | `RESULT: PASS`. Record the pass count + wall time in § 11 (the (date, command, result) triple). |
| 6 | Home the hand (GUI Home, or `/recover` if it is not already parked) **before** Activate | The 2026-09-23 safety event (cup-contact sheet § 8): a recovery park pushed the hand against the upper stop for up to 20 s with the bridge dark; the push mechanism is UNRESOLVED and on watch — starting from a freshly homed hand removes the one lever that trips it. |
| 7 | Base level: either (a) the base has been shimmed and QTM re-aligned since 2026-09-23 (root-cause fix, plan § 0), or (b) it has not — in which case rely on the landed mocap-offset subtraction (`_on_balls`/`_on_announcement`, adopted under the 25 mm bound, § "what changed") | Either is flyable. If (b), expect the frame check (row 21a-equivalent, § 3) to report a non-zero offset every time — that is the known lever arm, not a new finding. |
| 8 | QTM: `Catching Cone` rigid body DISABLED, Ball Butler reflectors MASKED (R3 sheet row 10); `Platform` rigid body defined and tracked (R3 sheet row 10a) | Hard preconditions, unchanged from R3/cup-contact. Nothing downstream catches a skipped step here. |

## 2. Bring-up (launch UP, robot powered, ball present)

Run R3-sheet rows **11–16** unchanged: load capture teed to
`temp/logs/loadavg_r4_$(date +%Y%m%d).txt`, launch with `record:=true
auto_arm:=true` teed to `temp/logs/launch_r4_$(date +%Y%m%d_%H%M).log`,
`blas threads: 1` for both `trajectory_node` and `skill_node`, GUI open +
QTM streaming, Home → Activate. Then, instead of R3's `skills/(start_self_
toss|check|stop)` service check, confirm the action + services this sheet
actually uses:

| # | Step | Expect |
|---|---|---|
| 9 | `ros2 action list \| grep jugglebot/juggle` ; `ros2 service list \| grep -E 'skills/check\|jugglebot/juggle_stop'` | All three present. Missing = the launch sourced the wrong install (R2 sheet row 1's note). |
| 10 | `ros2 param set /skill_node apex_m 0.9` ; `dwell_s 0.30` ; `plant_id r4-$(date +%Y%m%d)` | A FRESH `plant_id` — same memory contract as R3 (append-only within the sitting). `apex_m` 0.9 already matches the node default; set it anyway so the record is explicit. |
| 11 | `ros2 service call /trajectory/set_limits jugglebot_interfaces/srv/SetTrajectoryLimits "{leg_vel_limit_mmps: 300.0, leg_acc_limit_mmps2: 5000.0, leg_jerk_limit_mmps3: 150000.0}"` | `applied_*` echoes 300/5000/150000 — mandatory, the launch default is 1000/5000/30000. |
| 12 | Seat a ball in the hand; confirm on `/hand_telemetry` (`ball_held_valid: true`, `ball_held_raw: true`) | Visual + topic check, same as R3 row 20 — `skills/check`'s ladder does not see this directly. |

## 3. The dress rehearsal (every refusal reported at once)

Two separate checks — self_toss has a local compiler this bench can rehearse
fully offline; hop does not (`--pattern hop` has none — it only runs
through `--via-action`, the real production path, disarmed).

| # | Step | Expect |
|---|---|---|
| 13 | (venv, no ROS) `python3 tests/hardware/skills_plan_bench.py --rehearse --pattern self-toss --arm A --attempts 3` | Same shape as R3 sheet row 6: three clean attempts, memory rows 1→2→3, verdict G1/G2/G4 PASS (G3/G5 SKIP, offline). If this regresses from R3's numbers (33 ms G1, 26–33 ms G2), that is a finding before anything hop-related is tried. |
| 14 | (system python3, ROS sourced, robot ACTIVATE only — **never ARM** for this row) `python3 tests/hardware/skills_plan_bench.py --via-action --pattern self-toss --n-throws 3` | `outcome=COMPLETED`, 3/3 skills-throws dispatched. The wire is DISARMED (Activate, not Arm) so nothing moves — this exercises `skill_node`'s real accept ladder (pre-level, box, frame check) on the loaded box without risking a bad first hop. |
| 15 | Same, `--pattern hop --separation-mm 250 --n-throws 3` (still DISARMED) | `outcome=COMPLETED`. **Read every printed `per_throw` OUTCOME line and any REJECTED/ABORTED message together** — this is the "report every refusal at once" pass for hop; work every named refusal via § 7 before arming anything. |
| 16 | Ctrl-C mid-attempt on one of rows 14/15 (repeat the row after) | `^C -- cancelling the goal (cancel on stop)`, `outcome=STOPPED` (or the current end_code) — the cancel path itself is being rehearsed, not just the happy path. |
| 17 | `ros2 service call skills/check std_srvs/srv/Trigger` | `ladder OK`, `frame check OK: ...` (or the informational line if pinned), `box OK: ('P1','P1') 0.85-0.95; ...`, `hop box OK: P1->P2 at 250.0 mm`, `hop box OK: P2->P1 at 250.0 mm` — a `hop box REFUSED` line here means the box file does not cover the hop at the node's separation/apex: stop, do not power. |
| 18 | Now ARM (`record:=true auto_arm:=true` was already set at launch — confirm via GUI/`/robot_state` that the wire reads ARMED) | Ready for § 4. |

## 4. Self-toss regression at 0.9 m (nothing physical changed for this rung)

The R3 pattern, through the new action. 5 throws is enough to confirm the
plant, tracker and learner all still behave as R3 left them — this is a
regression check, not a new gate.

| # | Step | Expect |
|---|---|---|
| 19 | `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, num_cycles: 5}"` | Blocks, streams feedback, ends `outcome=COMPLETED throws=5`. Compare `caught`/per-throw `seat=` against the R3 §9 hardware-gate table and the cup-contact §8 Block A/B numbers — a material regression here is a NEW finding, stop and work it before touching hop. |
| 20 | `ros2 service call skills/check std_srvs/srv/Trigger` | Still clean. |

## 5. The hop at 250 mm (the R4 gate)

**Singles first** (`num_cycles: 1`, repeated) so a bad first hop is one
throw, not a chain; **then chained**. Watch, every throw:

- **The box**: `x` in **−20…+6 mm**, `y` in **±20 mm**, apex **0.850–0.900
  m** (0.95 refused — swept, not a typo). A plant more than ~5 % fast in
  apex will plateau against the 0.90 m ceiling with no headroom to correct
  down (U7a finding) — **record the measured apex ratio (physical apex /
  commanded 0.9 m) every throw**; that ratio is the decision input for
  whether the box needs widening before the next sitting, not this
  sitting's problem to fix.
- **Re-aims inside ~0.5 s of touch-down**: expect `LIMIT_JERK` refusals on
  the re-send, non-fatal — U4 measured 2.3–3.6× the jerk limit for a re-send
  seeded inside 0.45 s of landing at 250 mm; **count them**, do not treat
  each one as a new finding. A catch re-aimed later than ~0.5 s before
  landing is effectively open-loop at this separation (measured fact, not a
  bug to chase this sitting).
- **The closing REST is a fresh origin** — no `LIMIT_JERK` expected at the
  end of an attempt (U1's fix: the closing REST's dispatch is padded 2
  knots past the last catch specifically so it never splices into a settle
  tail). A refusal here IS a new finding.

| # | Step | Expect |
|---|---|---|
| 21 | `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: hop, apex_m: 0.9, separation_mm: 250.0, num_cycles: 1}"` | Goal ACCEPTED (if REJECTED, read the message against § 7 before retrying — it is naming a physical fact, not asking for a resend). Ends `outcome=COMPLETED throws=1 caught=<0 or 1>`. |
| 22 | Record: throw index, release site, target site, landing x/y, apex ratio, `seat=`, caught Y/N, any refusal code seen. Repeat row 21 (re-seat the ball) until 3–5 singles land cleanly in both directions (P1→P2 AND P2→P1). | A table for § 11. |
| 23 | `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: hop, apex_m: 0.9, separation_mm: 250.0, num_cycles: 10}"` | Chained: every throw after the first carried by the previous catch, alternating P1/P2. |
| 24 | Count consecutive catches from the possession sensor / streamed `caught` feedback. | **Gate: 10 consecutive catches alternating P1/P2.** If it ends early, record the end code + throw index (§ 7), re-seat, repeat row 23. |
| 25 | `ros2 service call skills/check std_srvs/srv/Trigger` between chained attempts | Still clean — a ladder refusal mid-sitting means something drifted (mocap, hand telemetry, mode) between attempts. |

## 6. The Ball Butler reload chain (GUI, one button)

**Never flown — keep a hand on the E-stop for the first reload catch.**
Fly `self_toss` first (one site, the simplest reload target), then `hop`.

| # | Step | Expect |
|---|---|---|
| 26 | `ros2 param set /skill_node separation_mm 250.0` (needed for the hop half below — see § "what changed": the GUI never overrides this per click) ; confirm `n_throws` reads the value you want after the reload's own catch (launch default 4) | `Set parameter successful`. |
| 27 | Confirm Ball Butler is loaded and ready to throw at this robot (its own bring-up, out of scope here). | |
| 28 | GUI: pattern = **Self-toss**, check **Reload first**, hold-to-confirm **Start**. Operator keeps a hand on the E-stop. | GUI status line moves `Dispatching self_toss (reload)…` → the reload round trip (`bb/reload` then `bb/throw_at_target`) → BB throws → **PRE-TILT REST** (watch it settle, ≥ 1.0 s) → **held-tilt CATCH** → **DECAY REST** → the pattern's own throws (`n_throws`). |
| 29 | Watch, on the catch: the `OUTCOME ball 0: ... seat=` line, the hand sensor (EMPTY→HELD once, not HELD→EMPTY→HELD), and by eye whether the ball settles or rattles — same signals `session_cup_contact.md` § 4 defines (+0.05…+0.15 s good, > ~0.20 s or no seat = a rebound). The 12° ceiling means the platform visibly tilts to meet the ball — that tilt IS the plan, not a fault. | Smooth seat, `caught=True`. |
| 30 | If the catch does not seat, or a guard latches: **do not re-dispatch by hand.** GUI **Stop** (immediate, no hold), let the rest tail settle, capture the bag, work § 7/§ 8. | |
| 31 | Once self_toss + reload lands clean at least once: GUI pattern = **Hop**, **Reload first** checked, hold-to-confirm **Start**. | Same PRE-TILT REST / CATCH / DECAY REST sequence, then the hop's own `n_throws` alternating between P1/P2 — reload always targets P1 (§ "what changed"), so the first post-catch throw goes P1→P2. |
| 32 | Watch the same signals as row 29, plus the hop's own watch list (§ 5) for every throw after the reload catch. | |
| 33 | `ros2 service call skills/check std_srvs/srv/Trigger` after each reload attempt | Still clean. |

## 7. Pre-registered verdict table

Every code below names ONE physical fact. R3 sheet § 6 has the full launch/
precondition table (`REJECTED_MOCAP_STALE`, `REJECTED_NOT_LEVELLED`,
`REJECTED_HAND_STALE`, `REJECTED_BALL_UNKNOWN`, `REJECTED_NO_BALL`,
`REJECTED_FRAME_OFFSET` at 25 mm now not 5, `ABORTED_MODE_CHANGED`,
`ABORTED_NO_RELEASE`, `NO_ADMISSIBLE_COMMAND`, `NO_LANDING`,
`SPLICE_TOO_LATE`, `WINDOW_TOO_SHORT`, `UNREACHABLE`, `LIMIT_JERK`/
`LIMIT_ACC`/`LIMIT_VEL`/`HAND_STROKE`/`HAND_ACC`, `STALE_STATE`/
`WRONG_MODE`) — **unchanged this sitting, still applies verbatim.** New or
changed for R4:

| Code | Physical fact |
|---|---|
| `CUP_CONTACT_ACC` | The plan's cup deceleration in the contact window (0.125 s before touch-down to rest) would exceed the −0.7 g floor — the planner should never PRODUCE a plan its own gate refuses; a re-send whose splice seed lands inside the window is meant to be SKIPPED before the solve (D6, "watch 3") — seeing this code fire anyway is a finding, not an expected refusal. |
| `GUARD_LATCHED` | A Teensy guard E-STOP is latched — command advancement is frozen at the measured hold until `/clear_errors`/`/recover`. Same recovery as R3 sheet § 7. |
| `SUPERSEDED_BY_HOLD` | A `trajectory/hold` landed while this segment's solve was running (e.g. a Stop/cancel raced an in-flight install) — the segment it planned was cancelled; the hold already has the wire. Not an error, just a race the operator's own Stop usually caused. |
| `columns refused: reload is not available for columns until R5` | Exactly what it says — not reachable this sitting since columns is not flown, but if you see it, you dispatched the wrong pattern. |
| `REJECTED_BB(<message>)` | Ball Butler's OWN refusal, surfaced verbatim (`bb/reload` or `bb/throw_at_target` returned `success=False`) — read BB's message, this is its diagnostic, not skill_node's. |
| `reload refused: bb/reload unavailable` / `... did not answer in <n> s` | Ball Butler's service is down or not responding — a BB bring-up problem, not this robot's. |
| `ABORTED_NO_ANNOUNCEMENT` | Ball Butler accepted the throw request but no `ThrowAnnouncement` arrived within the deadline (`throw_delay_s + predicted_tof_s + 1.0 s`) — BB's own countdown cannot be aborted (the ball may still come); the platform is already resting under the aim point, the safest place to wait. If the ball does land late, it is uncaught this attempt — re-dispatch reload once BB confirms it threw. |
| `reload bridge schedule refused: ...` / `the reload CATCH window (...) ` | The announced landing leaves no room for the opening REST + pre-tilt + catch window ahead of it — BB's `throw_delay_s` was too short for this robot's own homing/pre-tilt time. A BB-side timing tune, not a code bug. |
| `hop refused: no admissible box covers site pair(s) ... at apex ...` | No swept box (or none within band) for the hop pair/apex requested — the ONLY place a stale/missing hop box surfaces (§ "what changed" — `skills/check` does not catch this). |

## 8. If something refuses or latches

Unchanged from R3 sheet § 7 (never re-dispatch a hand move by hand; `Ctrl-C`
/ `jugglebot/juggle_stop` / GUI Stop are all safe at any time; an ended
attempt installs one `trajectory/hold`) and cup-contact sheet § 6 (a guard
latch during the dive: note the axis, the knot time relative to touch-down,
keep the bag). Two R4-specific notes:

- **A reload's bridge REST (the wait for BB's announcement) has no hand
  track of its own beyond the homing move** — cancelling it (Stop / GUI
  Stop) during the wait is safe exactly like any other REST; BB's own throw,
  once released, is Ball Butler's problem, not this robot's abort path.
- **Recovering a HIGH hand** (well off the settle clamp, e.g. after a
  guard latch mid-throw): keep a hand on the E-stop while `/recover` runs —
  the 2026-09-23 upper-stop push (§ 1 row 6) is unresolved and most likely
  to reproduce on a large recovery move, not a small one.

## 9. Carried watch items (plan § 0, physical operating point)

This sitting can observe all six of the items `plans/active/
two-ball-skill-stack.md` § 0 carried in from the cup-contact contract:

1. **Hand recovery park's upper-stop push** — unresolved, on watch (§ 1 row
   6, § 8).
2. **Hand `MAX_DEVIATION` latches from stale-encoder bursts at throw onset**
   — three so far; `plans/active/leg-bus-frame-drops.md`. Note the axis and
   the attempt phase if one fires.
3. **Contact-window re-send skip** — landed this unit (D6, "watch 3"); a
   `CUP_CONTACT_ACC` refusal firing anyway (§ 7) is the observable that it
   did not work.
4. **Re-send refusals mostly `LIMIT_JERK`** — 16/23 on 2026-09-22; § 5's hop
   watch list already asks you to count these.
5. **The seam tilt-RATE pin** — NOT built (§ "what changed"); the real fix
   landed a different way. Watch for seam roughness on a re-aimed catch as
   the observable that this needs revisiting.
6. **`test_the_qp_and_the_gate_stay_within_an_order_of_magnitude` flakes**
   ~2/10 quiet-box runs — if § 1 row 5's `--full` run shows this failing,
   re-run it alone before treating the gate as RED.

## 10. Close-out

| # | Step | Expect |
|---|---|---|
| 34 | GUI Deactivate (or `ros2 topic pub -t 3 -r 2 /orchestrator_command std_msgs/msg/String "data: 'deactivate'"`) | Robot stows. |
| 35 | Stop the launch and the load capture. | |
| 36 | `python tools/probes/throw_outcome_bag_probe.py --bag <id>` | Independent ground-truth check against `skill_node`'s own logged memory rows, same as R3 row 32. |
| 37 | Copy `temp/learn/<plant_id>/memory.csv` to a dated path. | |
| 38 | Send: the bag folder name, `temp/learn/<plant_id>/memory.csv`, `temp/logs/loadavg_r4_*.txt`, `temp/logs/launch_r4_*.log`, and the throw-by-throw tables from §§ 4–6. | |
| 39 | Log the sitting (`/log feature`, or `/investigate` if anything in § 7 fired unexpectedly or the reload catch showed a rebound signature). Update plan § R4 Outcome and the memory file. | |

## 11. Results

### Pre-power (§ 1)

| Item | Result |
|---|---|
| Date | *(fill in)* |
| `./run_tests.sh --full` (row 5) — (date, command, result) triple | |
| gate_hash match (row 3) | |
| hop box rows present (row 3a) | |

### Dress rehearsal (§ 3)

| Item | Result |
|---|---|
| Row 13 (`--rehearse --pattern self-toss`) — G1/G2/G4 verdict | |
| Row 14 (`--via-action --pattern self-toss`, disarmed) — outcome | |
| Row 15 (`--via-action --pattern hop`, disarmed) — outcome, every refusal listed | |

### Self-toss regression (§ 4)

| Throw | Caught | `seat=` | Notes |
|---|---|---|---|

### Hop (§ 5)

| Throw | Release→Target | Caught | landing x/y (mm) | apex ratio | `seat=` | Notes |
|---|---|---|---|---|---|---|

**Singles clean by throw:** ___ **Chained consecutive catches (gate: 10):** ___

### Reload (§ 6)

| Attempt | Pattern | PRE-TILT settle OK? | CATCH `seat=` | Caught | DECAY OK? | Subsequent throws caught | Notes |
|---|---|---|---|---|---|---|---|

### Bag / boot-banner record

| Rung | Time | Bag folder | `blas threads` line (both nodes) | Notes |
|---|---|---|---|---|
| Dress rehearsal | | | | |
| Self-toss regression | | | | |
| Hop | | | | |
| Reload | | | | |
