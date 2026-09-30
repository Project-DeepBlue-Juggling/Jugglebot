# R4 hardware runsheet — two sites, one ball, the Ball Butler reset

> **R4 gate MET 2026-09-29 (sitting 4, § 11 below).** The follow-on sitting is
> `session_skills_r5.md` (the 4° cup test, the fused reload, the interim
> human-lobbed columns start) — this sheet stays the reference for its
> bring-up, dress-rehearsal and recovery rows, unchanged.

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

- **Since sitting 2 (2026-09-28 21:41, `logbook/2026-09-28-skill-stack-r4-sitting-2-analysis.md`),
  landed 2026-09-28:**
  - **The hop's THROW plans again under a real level offset.** Sitting 2's three hops all refused
    `HAND_LIMIT_ACC 4969.7 > 3500` at the THROW install. Not the hand's cogging jitter: the number
    was identical to 0.1 across three seeds. The 50 ms post-release hold held the platform-tilt
    axis, which the level correction (-3.9, +5.8 mrad) rotates off the launch velocity ACROSS the
    hop, and the planner answered the one-knot sideways demand by collapsing the hand's stroke.
    The hold now keeps the cup on its own launch line. The sim gates never apply a level
    correction, which is why they passed; the regression test now does.
  - **Reload skips `bb/reload` when Ball Butler already holds a ball** (owner, 2026-09-28), and
    after a `bb/reload` waits for BB to leave IDLE before it trusts an IDLE heartbeat.
  - **Prove `ball_butler_node` is listening before the reload block** (§ 6 step 27): in sitting 2
    it was alive and answered nothing all session, which is both the reload timeouts and the GUI
    yaw/pitch freeze. Not reproduced offline; the step says what to capture if it recurs.
  - **The GUI's BB yaw/pitch fields no longer freeze the tab.** rosbridge now runs each service
    call on its own thread, and the aim module keeps one call in flight with a 3 s timeout. An
    unanswered aim now reads `bb/aim failed: bb/aim timed out after 3 s` within 3 s. Refresh the
    browser tab after the launch so it loads the new GUI files.
  - **The admissible box is re-swept** (the planner file is in its gate hash), `bf095653d422`.
- **Since sitting 1 (2026-09-27 22:37, `logbook/2026-09-28-skill-stack-r4-sitting-1-analysis.md`),
  landed 2026-09-28 — read that entry's Diagnosis once:**
  - **The learner memory is COLD.** The pre-calibration rows are quarantined
    (`temp/learn/_quarantine_20260928/`); they aimed every catch 20 mm off under the
    new geometry. Expect the first 2–3 throws of the sitting at the identity prior.
  - **A catch's carried throw releases from WHERE THE BALL WAS CAUGHT**, not from
    the site: the platform no longer translates while it holds the ball between
    a catch and its release (that translation, ~100 mm/s during the 0.35 s dwell,
    put +85..+110 mm/s of lateral velocity into every throw after a re-aimed catch
    and dropped all four chains). Re-centring on the site happens in the SETTLE
    tail after the ball has left. By eye: after a re-aimed catch the platform
    STAYS PUT until the ball is up, then drifts back. `learner_lateral_authority_mm`
    stays 20. The release offset is learner state (`x[2:4]`) and its fly-back is
    pre-compensated by the learner's own apex ratio — no knob.
  - **A 50 ms platform hold after every release** (`POST_RELEASE_HOLD_S`, 2 knots; 75 ms refused the R2 columns schedule): the plan
    used to re-accelerate the centroid toward the far site the instant the release
    knot passed, and the ball separates 20–40 ms late — the hop's +100 mm overshoot.
    Translation is held (hop residual 2.75 mm/s over the 2 held knots from the banking slew, was 68);
    the ATTITUDE is not (§ 9 item 7). By eye on the hop: the platform pauses at the
    release pose for a beat before it moves to the far site.
  - **Reload sequencing** (revised after sitting 2, 2026-09-28): if BB's heartbeat
    already reads IDLE + ball in hand, `bb/reload` is NOT sent (its ~1 s ball check is
    for an empty hand) and `bb/throw_at_target` fires on the next tick; only an empty
    hand gets `bb/reload`, after which skill_node waits (up to 10 s) for BB to leave
    IDLE and come back IDLE with a ball. Sitting 2 fired the throw on the IDLE from
    BEFORE the check began, ~20 ms after the reload. The announcement wait is armed
    before the throw call; a Teensy rejection on `bb/throw_outcome` ends the attempt
    the same tick (`REJECTED_BB(<token>)`); a BB refusal after the bridge REST is
    live ENDS the attempt instead of leaving it "in progress"; a firmware
    `sched_refused` bump during the reload wait now ends and holds even after the
    one-skill bridge executor has retired (the 09-27 guard latch had no END line).
  - **A fresh origin the solve outran** is REBASED (REST) or refused
    `ORIGIN_TOO_LATE` (THROW/CATCH) instead of reaching the firmware stale (§ 7).
    A hand seeded from the encoder seeds velocity 0 (the 09-27 `HAND_STROKE`).
  - **Watch `sched_refused` in `/link_status`** on every attempt: it must not move.
    On 09-27 it went 0 → 80 in two seconds with nothing on the console until the
    E-stop.

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
`BLAS 1 thread` on both the `trajectory up:` and `skill_node ready` lines, GUI open +
QTM streaming, Home → Activate. Then, instead of R3's `skills/(start_self_
toss|check|stop)` service check, confirm the action + services this sheet
actually uses:

| # | Step | Expect |
|---|---|---|
| 8b | **`level` FIRST, before any skill dispatch** (2026-09-27: the kinematic-calibration commit changed `jugglebot_geometry` in `hardware_config.yaml`, so the persisted inclinometer offset is STALE under the new geometry — `tests/hardware/session_kincal_apply.md` rung A; also confirm no stale `tilt_calibration.yaml` copy remains under `ros_ws/install/.../share`) | `level` completes and `trajectory/status.gravity_correction_loaded` reads fresh; a skill dispatched before this pre-levels against the OLD offset. |
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

## 4. Self-toss regression at 0.9 m (the geometry and the carried throw both changed since R3)

The R3 pattern, through the new action, from a COLD memory. Sitting 1 (2026-09-27) failed this
rung: 4/4 singles caught, no 5-cycle chain completed. What changed since (§ "what changed"):
the carried throw releases from the caught position and the platform holds 50 ms after release.
Watch for exactly the 09-27 signature — a throw after a re-aimed catch landing 30–110 mm off
toward the side the catch moved to — as the observable that the fix did not take. Expect the
first 2–3 throws at the identity prior (apex ~0.885–0.90 for 0.9 commanded; the calibrated
plant is near-identity in y).

| # | Step | Expect |
|---|---|---|
| 19 | `ros2 action send_goal /jugglebot/juggle jugglebot_interfaces/action/Juggle "{pattern: self_toss, num_cycles: 5}"` | Blocks, streams feedback, ends `outcome=COMPLETED throws=5`. Compare `caught`/per-throw `seat=` against the R3 §9 hardware-gate table and the cup-contact §8 Block A/B numbers — a material regression here is a NEW finding, stop and work it before touching hop. |
| 20 | `ros2 service call skills/check std_srvs/srv/Trigger` | Still clean. |

## 5. The hop at 250 mm (the R4 gate)

**Singles first** (`num_cycles: 1`, repeated) so a bad first hop is one
throw, not a chain; **then chained**. Watch, every throw:

- **The box** (re-swept 2026-09-27 under the kinematic-calibration geometry,
  gate hash `7f76f68d4943` (superseded 2026-09-28 by **`3fda47b2ad5b`**: the 50 ms post-release hold re-swept the boxes twice, bit-identical; the hop's x bound AWAY from the far site halved — P1→P2 x [−10, +2] mm was [−20, +2], P2→P1 x [−1, +10] was [−0.5, +20]; y ±20 mm, apex 0.85–0.95 m, and every self-toss/columns box unchanged; superseded again 2026-09-28 evening by **`bf095653d422`**: the hold now keeps the launch line, two sweeps bit-identical, only P2→P1 x moved to [−0.5, +10]) after `043158e` (bounds unchanged from `3059cc1`; the bridge
  now runs FW 24 with the stroke clamp generated from the calibrated geometry — a launch
  against any other bridge FW shows the SKEW advisory) — read `skills/check`, it prints the live bands):
  P1→P2 `x` in **−20…+2 mm**, P2→P1 `x` in **−0.5…+20 mm**, `y` in **±20 mm**,
  apex **0.85–0.95 m**. The x bound TOWARD the far site is essentially zero,
  so the lateral learner has no authority in the hop's own direction — **record
  the landing's x miss relative to the target site every throw** (the
  `OUTCOME` line's `y=(x, y)`): a consistent overshoot toward the far site is
  a finding for the sweep grid (2 mm steps stop at ±10 mm, then 20), not
  something the learner can correct. Also **record the measured apex ratio
  (physical apex / commanded 0.9 m) every throw**; it is the decision input for
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
| 27 | Confirm Ball Butler is loaded and ready to throw at this robot (its own bring-up, out of scope here): `ros2 topic echo /bb/heartbeat --once` must read `state: 1` (IDLE), `ball_in_hand: true`, `connected: true`. With that heartbeat skill_node skips `bb/reload` entirely (BB's RELOAD command runs a ~1 s ball check in CHECKING_BALL, state 6, that a loaded hand does not need). **Then prove `ball_butler_node` is listening** (sitting 2's whole reload block failed on a node that was alive but answered nothing, the GUI yaw/pitch fields included): after the BB calibration, `grep "BB calibration received" <launch log>` must show one line, and `ros2 service call /bb/aim jugglebot_interfaces/srv/BallButlerAim "{yaw_deg: 0.0, pitch_deg: 45.0}"` must answer within a second (a refusal message is fine; silence is not). | `state: 1`, `ball_in_hand: true`, the calibration line, an answer. Silence → relaunch; before relaunching, if you can, capture `sudo ~/Desktop/PDJ_venv/venv/bin/py-spy dump --pid $(pgrep -f lib/jugglebot/ball_butler_node)` to `temp/logs/`. A heartbeat stuck in 6 or 4 → wait; in 127 (ERROR) → BB reset first. |
| 28 | GUI: pattern = **Self-toss**, check **Reload first**, hold-to-confirm **Start**. Operator keeps a hand on the E-stop. | GUI status line moves `Dispatching self_toss (reload)…` → the reload round trip (`reload: Ball Butler already holds a ball -- bb/reload skipped` → `bb/throw_at_target` fired from the next tick with a delay derived from the bridge REST's own settle) → BB announces at dispatch and throws ~2.7 s later (a Teensy rejection now ends the attempt within one tick as `REJECTED_BB(<token>)`, no 4.6 s wait) → **PRE-TILT REST** (watch it settle, ≥ 1.0 s) → **held-tilt CATCH** → **DECAY REST** → the pattern's own throws (`n_throws`). |
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
| `REJECTED_BB(<message>)` | Ball Butler's OWN refusal, surfaced verbatim: `bb/reload`'s `success=False` message, `bb/throw_at_target`'s, or — new 2026-09-28 — the Teensy's terminal `CMD_RESULT` token relayed on `bb/throw_outcome` (`THROW_REJECTED_BAD_STATE`, `THROW_REJECTED_NO_BALL`, `THROW_REJECTED_CANT_MAKE_LEAD`, `ABORTED_NOT_SETTLED`, …) which ends the attempt the SAME tick, whether the throw was still awaited or its announcement had already compiled the reload schedule. Read BB's code, this is its diagnostic, not skill_node's. |
| `ABORTED_BB_NOT_READY(<state>, ball_in_hand=<bool>)` | New 2026-09-28: skill_node waits for a fresh BB heartbeat reading IDLE + ball in hand + connected before `bb/throw_at_target` (after a `bb/reload`, only an IDLE that follows BB leaving IDLE counts) — this code means it never did within `RELOAD_BB_READY_TIMEOUT_S` (3.0 s, ball already in hand) or `RELOAD_BB_FETCH_TIMEOUT_S` (10.0 s after a `bb/reload`; a ceiling, BB's own fetch time has never been recorded). The state named is what the heartbeat last said (BOOT/IDLE/TRACKING/THROWING/RELOADING/CALIBRATING/CHECKING_BALL/ERROR); `ball_in_hand=False` means BB has no ball — load it. |
| `ORIGIN_TOO_LATE` | New 2026-09-28: a fresh-origin THROW/CATCH whose solve took longer than the wire lead (75 ms) — its first knot would already be stale at the wire and the firmware's scheduled lane refuses a block discontinuous with the held lane (on 2026-09-27 that refusal integrated into a hand `MAX_DEVIATION` E-stop). A fresh REST in the same situation is not refused but REBASED (log line `fresh origin REBASED +<n> s`): same motion, later. Either on the console = the Jetson was slow (load, a long window); check `uptime` before re-flying. |
| `reload refused: bb/reload unavailable` / `... did not answer in <n> s` | Ball Butler's service is down or not responding — a BB bring-up problem, not this robot's. |
| `ABORTED_BB_THROW_TIMEOUT` (`bb/throw_at_target did not answer in 2.0 s -- ball_butler_node itself answered nothing`) | Sitting 2 (2026-09-28 21:41), all three reloads: `ball_butler_node` was alive but processed no callback all session (no calibration line, the GUI's `bb/aim` timed out at 50 s). Not a Ball Butler firmware refusal, which answers `REJECTED_BB`. Run the step-27 listening check; relaunch; capture the py-spy dump if it recurs. |
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
7. **The post-release hold is translation-only** (2026-09-28, 2 knots = 50 ms): freezing the
   attitude over the hold refuses on the 250 mm hop at this operating point
   (`LIMIT_ACC` 5538–21 711 > 5000 for every span). Residual centroid speed
   over the 2 held knots: hop 2.75 mm/s (was 68; 13.36 at the rejected 3-knot cut). If the hop still
   overshoots the far site with the platform visibly pausing at release, the
   remaining error is the ball's late separation (20–40 ms) riding the
   banking slew through the 224–261 mm cup lever — record `apex ratio` and
   the landing x; the learner's x authority toward the far site is +2 mm.
8. **The ball separates 20–40 ms after the planned release** on every throw
   (bag 2026-09-27: free flight extrapolated to t_rel sits 48–172 mm below
   the release point). Characterisation item; `tools/probes/
   release_motion_bag_probe.py` prints it per throw from a bag.
9. **`sched_refused` must stay flat** (§ "what changed"). A bump with no
   `HAND_LANE_REFUSED` END on the console is the defect class this sitting
   closed; if you see one, stop and capture the bag.

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

### Sitting 1 — 2026-09-27 22:37 (one launch, bag `~/Desktop/rosbags/2026-09-27_22-37-26`)

NOT MET. Self-toss: 4/4 singles caught (apex 0.885, landing y +20 mm = the stale memory's
+0.020 command); 0/4 chains completed (every throw after a re-aimed catch +32..+114 mm off).
Hop: 3/3 dropped, +87..+104 mm long, platform tilt at release matched the plan (3.9–4.0°).
Reload: 0/5 threw (`THROW_REJECTED_BAD_STATE` ×5, `ABORTED_NO_ANNOUNCEMENT` ×5), one hand
`MAX_DEVIATION` latch on a reload REST the firmware refused on all 80 frames. Full analysis and
the four fixes: `logbook/2026-09-28-skill-stack-r4-sitting-1-analysis.md`.

### Sitting 2 — 2026-09-28 21:41 (one launch, bag `~/Desktop/rosbags/2026-09-28_21-41-10`)

NOT MET. Self-toss from a cold memory: 34 caught / 2 not over the session's singles and chains,
stable to `num_cycles: 5`; the one `num_cycles: 10` ended `ABORTED_NO_RELEASE` after ~5 throws.
Almost every catch re-aimed by the full 20 mm lateral authority (§ 9 watch item). Hop: 3/3
refused at the THROW install, `HAND_LIMIT_ACC 4969.7` (the hold's axis under the level
correction, fixed). Reload: 3/3 `ABORTED_BB_THROW_TIMEOUT` (`ball_butler_node` deaf all session;
the throw would also have raced BB's ball check, fixed). Analysis:
`logbook/2026-09-28-skill-stack-r4-sitting-2-analysis.md`.

### Sitting 3 — 2026-09-28 23:55 (one launch, bag `~/Desktop/rosbags/2026-09-28_23-55-19`)

NOT MET. Self-toss steady (landing scatter ~16 × 23 mm 1σ, release-side). Hop +88/+110 mm long
(the platform still sliding +x at separation; the pre-release hold landed after). Reload: the
CATCH refused `CATCH_AXIS` (dispatched mid-pre-tilt; timing fixed), the hand stayed parked and
the ball was still caught. Analysis: `logbook/2026-09-29-skill-stack-r4-sitting-3-analysis.md`.

### Sitting 4 — 2026-09-29 19:11 (one launch, bag `~/Desktop/rosbags/2026-09-29_19-11-49`)

**MET.** Hop 75/82 caught; the 30-throw attempt made 25 consecutive alternating catches (14 by
the node's strict verdict, one slow seat logged `caught=False` on a ball the next throw launched
normally), ending on a P2→P1 throw 60 mm long. Hop landing x +12.6 / +2.3 mm (P1→P2 / P2→P1).
Self-toss 24/24. Reload 3/3 from the GUI button (one self_toss, two hop), each followed by 4
catches, no refusal; owner: the hand could rise during the pre-tilt instead of rushing up at the
catch. The GUI read every reload as `COMPLETED (0/0 caught)` at the button, a result-reporting
race fixed after the sitting. Analysis and the R5 carry list:
`logbook/2026-09-29-skill-stack-r4-gate-met.md`.

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

| Rung | Time | Bag folder | `BLAS` on both up lines | Notes |
|---|---|---|---|---|
| Dress rehearsal | | | | |
| Self-toss regression | | | | |
| Hop | | | | |
| Reload | | | | |
